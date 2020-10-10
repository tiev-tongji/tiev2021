'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)

'''

import sys
sys.path.append('/home/neousys/Desktop/tiev2019/src/modules/Visual/')
import cv2
import numpy as np
import torch
import time
import glob
import os
from pdb import set_trace as b

from dataset import Apollo_data
from postproc_lane import process_tensor

intrinsics = [
    [1072.8,                0, 956.3], 
    [               0,   1074.1, 619.7], 
    [               0,                0,                1]
]

dtype = torch.float32

raw_height, raw_width = (1200, 1920)


rx = torch.tensor([1.54], requires_grad=False)
rz = torch.tensor([-.01], requires_grad=False)
ry = torch.tensor([0.00], requires_grad=False)

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

NUM_CLASSES = 38
net = FCN8s_ResNet(num_classes=NUM_CLASSES).cuda()
kwargs = {'map_location': lambda storage, loc: storage.cuda(0)}
pretrained_model_path = '/home/neousys/Desktop/tiev2019/src/modules/Visual/ckpt/epoch_5_loss_0.06971_acc_0.97669_acc-cls_0.60299_mean-iu_0.44240_fwavacc_0.95725_lr_0.0002000000.pth'
net = load_GPUS(net, pretrained_model_path, kwargs)
net.eval()

# conecting to the first available camera

# Grabing Continusely (video) with minimal delay
# converting to opencv bgr format
template = torch.arange(38)[None, :, None, None].cuda()

root = '/home/neousys/Desktop/videos/'
#video_path = '/home/neousys/Desktop/tiev2019/src/modules/Visual/ckpt/record.avi'

video_path = '/home/neousys/Desktop/videos/record_9G.avi'

#for ii in range(len(pic_list)):
cap = cv2.VideoCapture(video_path)
arrow_id = torch.arange(19,31)[None, :, None, None].cuda()
count_i = 0

while cap.isOpened():
    count_i += 1
    if count_i % 2 == 0:
        continue
    ret, img0 = cap.read()
    if img0 is None:
        break
    count_i += 1
    #img0 = cv2.imread(pic_path)
    print("FRAME: ", count_i)
    #pic_path = os.path.join(root, 'pic-%04d.jpeg'%ii)
    #if os.path.exists(pic_path):
        # Access the image data
    if True:
        time1 = time.time()
        #img = cv2.imread(pic_path)
        #img = cv2.resize(img, (256,256))
        blank = np.zeros(img0.shape, img0.dtype)
        # img = cv2.addWeighted(img1, 1, blank, 0, -100)
        (b, g, r) = cv2.split(img0)
        bH = cv2.equalizeHist(b)
        gH = cv2.equalizeHist(g)
        rH = cv2.equalizeHist(r)
        # 合并每一个通道
        img = cv2.merge((bH, gH, rH)) 
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        bev = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).cuda()
        img_np = bev.permute(0, 2, 3, 1).detach()[0] \
            .cpu().numpy().astype(np.uint8)
        mean, std = (255. * torch.tensor([0.51, 0.51, 0.51], dtype=bev.dtype, device=bev.device) , 
                     255. * torch.tensor([0.294, 0.297, 0.295], dtype=bev.dtype, device=bev.device) )
        bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]
        
        outputs = net(bev)
        pred0 = outputs.data.max(1, keepdims=True)[1].squeeze_(1) 
        pred = (template == pred0).to(dtype=torch.uint8)[0].cpu().numpy()
        aa = (arrow_id == pred0).to(dtype=torch.uint8)[0]
        if torch.sum(aa) > 50:      
            #import pdb;pdb.set_trace()
            img_save_path = os.path.join(root, 'arrow_9G_jpg/%05d.png'%(count_i+1))
            npy_save_path = os.path.join(root, 'arrow_9G_npy/%05d.npy'%(count_i+1))
            cv2.imwrite(img_save_path, img0)
            np.save(npy_save_path, pred)
            print('_____sum points: _____   ', torch.sum(aa).data)
            #import pdb;pdb.set_trace()

        #img_np_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)


        #small_image =np.ascontiguousarray(cv2.resize(img_np_bgr, (256, 512)))
        #pred = np.ascontiguousarray(cv2.resize(pred, (256, 512)))
        #b()
        #print(process_tensor(img0, pred))
        #print("now is ---------%d"%ii)
        if 0:
            predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img0, 1.0, pre_np, 1.0, 0)
            mix_img = np.concatenate((img0, pre_np, mask_img), axis=1)
            cv2.namedWindow('title', cv2.WINDOW_NORMAL)
            cv2.imshow('title', mix_img)
        
        time2 = time.time()


        #print(time2-time1)

        #k = cv2.waitKey(0)
        #if k == 27:
           # break
    
# Releasing the resource    
cap.release()

cv2.destroyAllWindows()
