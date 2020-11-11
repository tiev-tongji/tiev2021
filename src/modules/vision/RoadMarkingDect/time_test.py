from road_marking_dect_online import RoadMarkingOnLine
import argparse
import time
import cv2
from bevutils import PerspectiveTransformerLayer
import pdb
import torch

dtype = torch.float32

def init_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, default='./ckpt/epoch_6_loss_0.06795_acc_0.97691_acc-cls_0.59547_'
                                                          'mean-iu_0.46755_fwavacc_0.95794_lr_0.0002000000.pth',
                        help='pretrained model path')
    parser.add_argument('--eh_type', type=str, default='no', help='which equal hist type "no" or "rgb" or "hsv"')
    parser.add_argument('--with_post', type=bool, default=False, help='with post processing or not')
    parser.add_argument('--device', type=str, default='cuda',help='cuda or cpu')
    return parser.parse_args()

if __name__ == '__main__':
    args = init_args()
    roadmarking_dect = RoadMarkingOnLine(args.model_path, eh_type=args.eh_type, device=args.device)
    inp = cv2.imread('time_test.png')
    inp = cv2.resize(inp, (1200,1920), cv2.INTER_LINEAR)
    pitch_relative = 0
    time_sum = 0
    count = 0
    intrinsics = [
        [1072.8, 0, 956.3],
        [0, 1074.1, 619.7],
        [0, 0, 1]]
    raw_height, raw_width = (1200, 1920)
    warpPerspective = PerspectiveTransformerLayer((1024, 256), (raw_height, raw_width), intrinsics,
                                                  translate_z=-40,
                                                  rotation_order='xyz', dtype=dtype)
    bev = roadmarking_dect.wrap(inp, pitch_relative, warpPerspective)
    net = roadmarking_dect.init_net()
    while count<200:
        time1 = time.time()
        bev = roadmarking_dect.wrap(inp, pitch_relative, warpPerspective)
        img, pred = roadmarking_dect.go_through_CNN(net, bev)
        time2 = time.time()
        print(time2 - time1)
        time_sum += (time2 - time1)
        count += 1
    print(time_sum/count)

