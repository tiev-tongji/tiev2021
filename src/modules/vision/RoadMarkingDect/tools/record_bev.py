'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)

'''
import cv2
import numpy as np
import torch
from zerocm import ZCM
import sys
sys.path.append('/home/autolab/tiev2019/src/modules/Visual/')
from structNAVINFO import structNAVINFO

from easydict import EasyDict as edict
from bevutils import PerspectiveTransformerLayer
from pypylon import pylon
from dataset import Apollo_data

intrinsics = [
    [1072.8,                0, 956.3], 
    [               0,   1074.1, 619.7], 
    [               0,                0,                1]
]

dtype = torch.float32

raw_height, raw_width = (1200, 1920)

warpPerspective = PerspectiveTransformerLayer((1024, 256), (raw_height, raw_width), intrinsics, translate_z=-40, rotation_order='xyz', dtype=dtype)

pitch = 0
def handler(channel, msg):
    global pitch
    #pdb.set_trace()
    pitch  = msg.mPitch
    print(pitch)

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
#import sys
#from models import FCN8s
#NUM_CLASSES = 38
#net = FCN8s(num_class=NUM_CLASSES).cuda()
#net.load_stat.e_dict(torch.load('/home/autolab/huangyuyao/SegNet_bymyself/ckpt/epoch_6_loss_386264.01375_acc_0.97124_acc-cls_0.59021_mean-iu_0.40153_fwavacc_0.94774_lr_0.0000000010.pth'))
#net.eval()

# conecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# converting to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

cap = cv2.VideoWriter('record.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25, (256, 1024))
assert cap.isOpened()



while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()
        inp = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).cuda()
        #bev = warpPerspective(inp, rx, ry, rz)
        bev = warpPerspective(inp, rx+(pitch+2.65)/180*3.14159265358, ry, rz)
        img_np = bev.permute(0, 2, 3, 1).detach()[0] \
            .cpu().numpy().astype(np.uint8)
        cap.write(img_np)
        cv2.namedWindow('title', cv2.WINDOW_NORMAL)
        cv2.imshow('title', img_np)
        k = cv2.waitKey(1)
        if k == 27:
            break
        continue
        mean, std = (torch.tensor([0.464, 0.442, 0.431], dtype=bev.dtype, device=bev.device) * 255., 
                     torch.tensor([0.276, 0.285, 0.276], dtype=bev.dtype, device=bev.device) * 255.)
        bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]
        #outputs = net(bev)
        predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy() * 255.
        predictions_pil = Apollo_data.colorize_mask(predictions[0])
        #predictions_pil = predictions_pil.convert('RGB')
        pre_np = np.asarray(predictions_pil)
        pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
        mask_img = cv2.addWeighted(img_np, 1.0, pre_np, 1.0, 0)

        mix_img = np.concatenate((img_np, pre_np, mask_img), axis=1)

    grabResult.Release()
    
# Releasing the resource    
camera.StopGrabbing()
cap.release()

cv2.destroyAllWindows()
#
