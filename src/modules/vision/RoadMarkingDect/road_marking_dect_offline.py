'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)
offline version need to read a video(in bird-eye view)

'''
import sys
sys.path.append('../')
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
# sys.path.append('./postproc_lane')
sys.path.append('./')
import cv2
import numpy as np
import torch
import time
from pathlib import Path
from pdb import set_trace as b
import argparse
from dataset import Apollo_data
from zerocm import ZCM
from structLASERMAP import structLASERMAP


dtype = torch.float32

def init_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, default='../ckpt/epoch_6_loss_0.06795_acc_0.97691_acc-cls_0.59547_'
                                                          'mean-iu_0.46755_fwavacc_0.95794_lr_0.0002000000.pth',
                        help='pretrained model path')
    parser.add_argument('--eh_type', type=str, default='no', help='which equal hist type "no" or "rgb" or "hsv"')
    parser.add_argument('--video_path', type=str, default='../data/record_imu_bev.avi',help='path to test video')
    parser.add_argument('--with_post', type=bool, default=True, help='with post processing or not')
    parser.add_argument('--visual', type=bool, default=false, help='visualise the result')
    parser.add_argument('--device', type=str, default='cpu',help='cuda or cpu')
    return parser.parse_args()


class RoadMarkingOffLine():
    def __init__(self, pretrained_model_path, eh_type='no', device='cuda'):
        super(RoadMarkingOffLine, self).__init__()
        self.pretrained_model_path = pretrained_model_path
        self.device = device
        self.eh_type = eh_type

    def _load_GPUS(self, model, model_path):
        state_dict = torch.load(model_path, map_location=torch.device(self.device))
        # create new OrderedDict that does not contain `module.`
        from collections import OrderedDict
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            name = k[7:] # remove `module.`
            new_state_dict[name] = v
        # load params
        model.load_state_dict(new_state_dict)
        return model
    def _equal_hist(self, img0, eh_type='no'):
        #adjust the brightness
        # blank = np.zeros(img0.shape, img0.dtype)
        #img = cv2.addWeighted(img0, 1, blank, 0, -100)
        if eh_type == 'no':
            return img0
        elif eh_type == 'rgb':
            (b, g, r) = cv2.split(img0)
            bH = cv2.equalizeHist(b)
            gH = cv2.equalizeHist(g)
            rH = cv2.equalizeHist(r)
            #merge every channel
            img = cv2.merge((bH, gH, rH))
            return img
        elif eh_type == 'hsv':
            img_hsv = cv2.cvtColor(img0, cv2.COLOR_RGB2HSV)
            (h, s, v) = cv2.split(img_hsv)
            vH = cv2.equalizeHist(v)
            img_hsv_eh = cv2.merge((h, s, vH))
            img_rgb_eh = cv2.cvtColor(img_hsv_eh,cv2.COLOR_HSV2RGB)
            return img_rgb_eh
        else:
            assert(0)

    def init_net(self):
        from models import FCN8s_ResNet
        NUM_CLASSES = 38
        net = FCN8s_ResNet(num_classes=NUM_CLASSES).to(self.device)
        # kwargs = {'map_location': lambda storage, loc: storage.cuda(0)}
        pretrained_model_path = self.pretrained_model_path
        net = self._load_GPUS(net, pretrained_model_path)
        net.eval()
        return net
    def go_through_CNN(self, net, img):
        # b()
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        img = self._equal_hist(img, self.eh_type)
        bev = torch.tensor(img, dtype=dtype)[None, :, :, :].permute(0, 3, 1, 2).to(self.device)
        # img_np = bev.permute(0, 2, 3, 1).detach()[0].cpu().numpy().astype(np.uint8)
        mean, std = (255. * torch.tensor([0.51, 0.51, 0.51], dtype=bev.dtype, device=bev.device) ,
                     255. * torch.tensor([0.294, 0.297, 0.295], dtype=bev.dtype, device=bev.device) )
        bev = (bev - mean[None, :, None, None]) / std[None, :, None, None]

        outputs = net(bev)
        pred = outputs.data.max(1, keepdims=True)[1].squeeze_(1)
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        return img, pred

    def post_process(self, post_pro_path, img, pred, map, visual):

        template = torch.tensor([1,2,3,4,5,6,7,8,9,10,11,12,13,18,19,20,21,22,23,24,25,26,27,28,30]) \
            [None, :, None, None].to(self.device)
        pred_split = (template == pred).to(dtype=torch.uint8)[0].cpu().numpy()
        status_list = list()
        sys.path.append(post_pro_path)
        from postproc_lane import process_tensor
        # status_list = process_tensor(img, pred_split, status_list)
        #[have_result, zcmok, Lane_num, lane_type(from right to left), line_type(from right to left), stop_line_exist, boundary_detected]
        status_list = process_tensor(img, pred_split, map, status_list)
        #print("now is ---------%d"%ii)
        if visual:
            predictions = pred.cpu().numpy()
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img, 1.0, pre_np, 1.0, 0)
            mix_img = np.concatenate((img, pre_np, mask_img), axis=1)
            cv2.namedWindow('title', cv2.WINDOW_NORMAL)
            cv2.imshow('title', mix_img)
        return status_list


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


if __name__ == '__main__':
    args = init_args()
    roadmarking_dect = RoadMarkingOffLine(args.model_path, args.eh_type, args.device)
    net = roadmarking_dect.init_net()
    video_path = args.video_path
    cap = cv2.VideoCapture(video_path)
    count_i = 0
    while cap.isOpened():
        ret, img = cap.read()
        if ret:
            time1 = time.time()
            img, pred = roadmarking_dect.go_through_CNN(net, img)
            if args.with_post:
                from postproc_lane import process_tensor
                #[have_any_msg, zcmok, Lane_num, lane_type(from right to left), line_type(from right to left), stop_line_exist]
                # status_list = roadmarking_dect.post_process(img, pred)
                time1 = time.time()
                status_list = list()
                post_pro_path = "./postproc_lane"
                #[have_result, zcmok, Lane_num, lane_type(from right to left), line_type(from right to left), stop_line_exist, boundary_detected]
                status_list = post_process(post_pro_path, img, pred, map, args.visual)
                # print(status_list)
            time2 = time.time()
            print(time2-time1)
            k = cv2.waitKey(1)
            if k == 27:
                break
    cap.release()
    cv2.destroyAllWindows()