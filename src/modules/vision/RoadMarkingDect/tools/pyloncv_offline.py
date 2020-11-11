'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)
offline version need to read a video(in bird-eye view)

'''
import sys
sys.path.append('../')
import cv2
import numpy as np
import torch
import time
from pathlib import Path
from pdb import set_trace as b
import argparse

from dataset import Apollo_data
from postproc_lane import process_tensor

dtype = torch.float32


def init_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, default='../ckpt/epoch_5_loss_0.06971_acc_0.97669_acc-cls_0.60299_'
                                                          'mean-iu_0.44240_fwavacc_0.95725_lr_0.0002000000.pth',
                        required=True, help='pretrained model path')
    parser.add_argument('--eh_type', type=str, default='no',
                        help='which equal hist type "no" or "rgb" or "hsv"')
    return parser.parse_args()

args = init_args()

# prepare neural network
from models import FCN8s_ResNet
def load_GPUS(model,model_path,kwargs):
    state_dict = torch.load(model_path,**kwargs)
    # create new OrderedDict that does not contain `module.`
    from collections import OrderedDict
    new_state_dict = OrderedDict()
    for k, v in state_dict.items():
        name = k[7:] # remove `module.`
        new_state_dict[name] = v
    # load params
    model.load_state_dict(new_state_dict)
    return model

def equal_hist(img0, eh_type='no'):

    #adjust the brightness
    # blank = np.zeros(img0.shape, img0.dtype)
    #img = cv2.addWeighted(img0, 1, blank, 0, -100)
    if eh_type == 'no':
        return img0
    elif eh_type == 'rgb_eh':
        (b, g, r) = cv2.split(img0)
        bH = cv2.equalizeHist(b)
        gH = cv2.equalizeHist(g)
        rH = cv2.equalizeHist(r)
        #merge every channel
        img = cv2.merge((bH, gH, rH))
        return img
    elif eh_type == 'hsv_eh':
        img_hsv = cv2.cvtColor(img0, cv2.COLOR_RGB2HSV)
        (h, s, v) = cv2.split(img_hsv)
        vH = cv2.equalizeHist(v)
        img_hsv_eh = cv2.merge((h, s, vH))
        img_rgb_eh = cv2.cvtColor(img_hsv_eh,cv2.COLOR_HSV2RGB)
        return img_rgb_eh
    else:
        assert(0)

NUM_CLASSES = 38
net = FCN8s_ResNet(num_classes=NUM_CLASSES).cuda()
kwargs = {'map_location': lambda storage, loc: storage.cuda(0)}
pretrained_model_path = args.model_path
net = load_GPUS(net, pretrained_model_path, kwargs)
net.eval()

template = torch.tensor([1,2,3,4,5,6,7,8,9,10,11,12,13,18,19,20,21,22,23,24,25,26,27,28,30])[None, :, None, None].cuda()

video_path = '/home/neousys/Desktop/videos/record2.avi'
cap = cv2.VideoCapture(video_path)
count_i = 0

while cap.isOpened():
    count_i += 1
    ret, img = cap.read()
    if True:
        time1 = time.time()
        # do equal hist or not
        img = equal_hist(img, type=args.eh_type)
        bev = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).cuda()
        img_np = bev.permute(0, 2, 3, 1).detach()[0].cpu().numpy().astype(np.uint8)
        mean, std = (255. * torch.tensor([0.51, 0.51, 0.51], dtype=bev.dtype, device=bev.device) , 
                     255. * torch.tensor([0.294, 0.297, 0.295], dtype=bev.dtype, device=bev.device) )
        bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]
        
        outputs = net(bev)
        pred = outputs.data.max(1, keepdims=True)[1].squeeze_(1)
        pred = (template == pred).to(dtype=torch.uint8)[0].cpu().numpy()
        img_np_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        print(process_tensor(img, pred))
        #print("now is ---------%d"%ii)
        if 1:
            predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img_np_bgr, 1.0, pre_np, 1.0, 0)
            mix_img = np.concatenate((img_np_bgr, pre_np, mask_img), axis=1)
            cv2.namedWindow('title', cv2.WINDOW_NORMAL)
            cv2.imshow('title', mix_img)
        
        time2 = time.time()


        print(time2-time1)
        k = cv2.waitKey(0)
        if k == 27:
           break

# Releasing the resource    
cap.release()
cv2.destroyAllWindows()
