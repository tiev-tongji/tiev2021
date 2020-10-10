'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)

'''
from zerocm import ZCM
import sys
sys.path.append('../')
sys.path.append('../postproc_lane/')
sys.path.insert(0, './')
from structNAVINFO import structNAVINFO
import cv2
import numpy as np
import torch
import time
from pdb import set_trace as b

from easydict import EasyDict as edict
from bevutils import PerspectiveTransformerLayer
from pypylon import pylon
from dataset import Apollo_data
from postproc_lane import process_tensor

intrinsics = [
    [1072.8,                0, 956.3], 
    [               0,   1074.1, 619.7], 
    [               0,                0,                1]
]

dtype = torch.float32

raw_height, raw_width = (1200, 1920)

warpPerspective = PerspectiveTransformerLayer((1024, 256), (raw_height, raw_width), intrinsics, translate_z=-40, rotation_order='xyz', dtype=dtype)#!

#pitch = 0
pitch_mean = 0
pitch_relative = 0
lamda = 0.95
def handler(channel, msg):
    #global pitch
    global pitch_mean
    global pitch_relative
    new_pitch  = msg.mPitch
    #new_pitch  = msg.mPitch
    pitch_mean = lamda*pitch_mean + (1-lamda)*new_pitch
    pitch_relative = new_pitch - pitch_mean
    #print(pitch)

zcm = ZCM("")

if not zcm.good():

    print("Unable to initialize zcm")

    exit()

zcm.start()

subs = zcm.subscribe("NAVINFO", structNAVINFO, handler)

rx = torch.tensor([1.538], requires_grad=False)
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
pretrained_model_path = '../ckpt/epoch_5_loss_0.06971_acc_0.97669_acc-cls_0.60299_mean-iu_0.44240_fwavacc_0.95725_lr_0.0002000000.pth'#!
net = load_GPUS(net, pretrained_model_path, kwargs)
net.eval()

# conecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# converting to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_RGB8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned


#template = torch.arange(38)[None, :, None, None].cuda()
template = torch.tensor([1,2,3,4,5,6,7,8,9,10,11,12,13,18,19,20,21,22,23,24,25,26,27,28,30])[None, :, None, None].cuda()


while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        time1 = time.time()
        
        image = converter.Convert(grabResult)
        img = image.GetArray()

        #blank = np.zeros(img0.shape, img0.dtype)
        # img = cv2.addWeighted(img1, 1, blank, 0, -100)
        #(b, g, r) = cv2.split(img0)
        #bH = cv2.equalizeHist(b)
        #gH = cv2.equalizeHist(g)
        #rH = cv2.equalizeHist(r)
        # 合并每一个通道
        #img = cv2.merge((bH, gH, rH))

        #img_hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
        # img = cv2.addWeighted(img1, 1, blank, 0, -100)
        #(h, s, v) = cv2.split(img_hsv)
        #vH = cv2.equalizeHist(v)
        #img_hsv_eh = cv2.merge((h, s, vH))
        #img_rgb_eh = cv2.cvtColor(img_hsv_eh,cv2.COLOR_HSV2RGB)
        inp = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).cuda()
        print(pitch_relative)
        bev = warpPerspective(inp, rx+(pitch_relative)/180*3.14159265358, ry, rz)
        img_np = bev.permute(0, 2, 3, 1).detach()[0] \
            .cpu().numpy().astype(np.uint8)
        mean, std = (255. * torch.tensor([0.51, 0.51, 0.51], dtype=bev.dtype, device=bev.device) , 
                     255. * torch.tensor([0.294, 0.297, 0.295], dtype=bev.dtype, device=bev.device) )#!
        bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]
        
        outputs = net(bev)
        pred = outputs.data.max(1, keepdims=True)[1].squeeze_(1) 
        pred = (template == pred).to(dtype=torch.uint8)[0].cpu().numpy()
        img_np_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        #small_image =np.ascontiguousarray(cv2.resize(img_np_bgr, (256, 512)))
        #pred = np.ascontiguousarray(cv2.resize(pred, (256, 512)))
        #b()
        print(process_tensor(img_np_bgr, pred))

        if 1:
            predictions = outputs.data.max(1, keepdims=True)[1].squeeze_(1).cpu().numpy()
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img_np_bgr, 1.0, pre_np, 1.0, 0)

            mix_img = np.concatenate((img_np_bgr, pre_np, mask_img), axis=1)

            cv2.namedWindow('title', cv2.WINDOW_NORMAL)
            cv2.imshow('title', mix_img)
            #print(">>>>>>>>>>>>>>>>.", mix_img.shape)

        
        time2 = time.time()
        
        print(time2-time1)
        k = cv2.waitKey(1)
        if k == 27:
            break

    grabResult.Release()
    
# Releasing the resource    
camera.StopGrabbing()
#camera.Release()
cap.release()
zcm.stop()

#cv2.destroyAllWindows()
