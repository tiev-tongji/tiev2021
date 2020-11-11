'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)
offline version need to read .jpg images and .npy of detection net's output

'''
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
sys.path.append('../')
sys.path.append('./postproc_lane')
sys.path.append('./')
import cv2
import numpy as np
import torch
import time
from zerocm import ZCM
from structLASERMAP import structLASERMAP
import glob
from pathlib import Path
from pdb import set_trace as b

from dataset import Apollo_data
from postproc_lane import process_tensor

dtype = torch.float32

class VideoWriter:
    def __init__(self, name, width, height, fps=20):
        self.__name = name
        self.__height = height
        self.__width = width
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.__writer = cv2.VideoWriter(name, fourcc, fps, (width,height))

    def write(self, frame):
        if frame.dtype != np.uint8:
            raise ValueError('frame dtype should be np.uint8')
        row, col, _ = frame.shape
        if row != self.__height or col != self.__width:
            warning.warn('wrong shape')
            return
        self.__writer.write(frame)

    def close(self):
        self.__writer.release()

def equal_hist(img0, type='rgb_eh'):
    # adjust the brightness
    # blank = np.zeros(img0.shape, img0.dtype)
    # img = cv2.addWeighted(img0, 1, blank, 0, -100)
    if type == 'rgb_eh':
        (b, g, r) = cv2.split(img0)
        bH = cv2.equalizeHist(b)
        gH = cv2.equalizeHist(g)
        rH = cv2.equalizeHist(r)
        # merge every channel
        img = cv2.merge((bH, gH, rH))
        return img
    elif type == 'hsv_eh':
        img_hsv = cv2.cvtColor(img0, cv2.COLOR_RGB2HSV)
        (h, s, v) = cv2.split(img_hsv)
        vH = cv2.equalizeHist(v)
        img_hsv_eh = cv2.merge((h, s, vH))
        img_rgb_eh = cv2.cvtColor(img_hsv_eh, cv2.COLOR_HSV2RGB)
        return img_rgb_eh

timestamp = 0
map = np.zeros((401,151), np.int8)
def handler(channel, msg):
    #global pitch
    global timestamp
    timestamp = msg.timestamp
    global map
    for i0 in range(401):
        map[i0] = (bytearray(msg.cells[i0][:151]))

zcm = ZCM("ipc")

if not zcm.good():
    print("Unable to initialize zcm")
    exit()

zcm.start()

subs = zcm.subscribe("LASERMAP", structLASERMAP, handler)


template = torch.tensor([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 30])[None, :, None, None]#.cuda()
root = '../outdata/001/'
pic_list = glob.glob('{:s}/**/*.png'.format(root), recursive=True)
pic_list.sort()
vw = VideoWriter('no_curb.mp4', 256*3, 1024)

for ind, pic_path in enumerate(pic_list):
    img = cv2.imread(pic_path)
    if True:
        # do equal hist or not
        # img = equal_hist(img, type='hsv_eh')
        #bev = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).cuda()
        #img_np = bev.permute(0, 2, 3, 1).detach()[0].cpu().numpy().astype(np.uint8)
        #mean, std = (255. * torch.tensor([0.51, 0.51, 0.51], dtype=bev.dtype, device=bev.device),
         #            255. * torch.tensor([0.294, 0.297, 0.295], dtype=bev.dtype, device=bev.device))
        #bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]

        np_file_path = pic_path.replace('.png', '.npy')
        pred_npy = np.load(np_file_path)
        #pred = outputs.data.max(1, keepdims=True)[1].squeeze_(1)

        pred = (template == torch.tensor(pred_npy)).to(dtype=torch.uint8)[0].cpu().numpy()
        #img_np_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        #b()
        time1 = time.time()
        status_list = list()
        #[have_result, zcmok, Lane_num, lane_type(from right to left), line_type(from right to left), stop_line_exist, boundary_detected]
        status_list = process_tensor(img, pred, map, status_list)
        print(status_list);
        time2 = time.time()
        print(timestamp)
        # print(status_list)
        # print("now is ---------%d"%ii)
        if 1:
            predictions = pred_npy
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img, 1.0, pre_np, 1.0, 0)
            mix_img = np.concatenate((img, pre_np, mask_img), axis=1)
            # if(ind > 898-564):
            #     vw.write(mix_img)
            cv2.namedWindow('title', cv2.WINDOW_NORMAL)
            cv2.imshow('title', mix_img)

vw.close()

        #print(time2 - time1)
        #k = cv2.waitKey(0)
        #if k == 27:
        #    break

# Releasing the resource
cv2.destroyAllWindows()
