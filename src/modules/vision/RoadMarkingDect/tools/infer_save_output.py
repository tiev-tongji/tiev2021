'''
first pick images from .avi in 'data' directory:
ffmpeg -i record_imu_bev.avi -r 20 -q:v 2 -f image2 %05d.png
'''

from models import FCN8s_ResNet
import torch
import torchvision.transforms as standard_transforms
from PIL import Image
from torch.autograd import Variable
from dataset import Apollo_data
import os
import pdb
import glob
import numpy as np
import argparse
import cv2
import utils.extend_transform as extended_transforms
import copy

num_classes = 38


def init_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, default='./ckpt/epoch_23_loss_130037.40958_acc_0.96911_acc-cls_0.33367_mean-iu_0.28692_fwavacc_0.94391_lr_0.0008000000.pth', help='pretrained model path')
    parser.add_argument('--eh_type', type=str, default='no',
                        help='which equal hist type "no" or "rgb" or "hsv"')
    parser.add_argument('--data_root', type=str, default='./data/')
    return parser.parse_args()

class Reference(object):
    def __init__(self, net):
        self.net = net

    def __call__(self, inputs):
        outputs = self.net(inputs)

        return outputs


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
    else:
        img0 = np.array(img0)
        if eh_type == 'rgb_eh':
            (b, g, r) = cv2.split(img0)
            bH = cv2.equalizeHist(b)
            gH = cv2.equalizeHist(g)
            rH = cv2.equalizeHist(r)
            #merge every channel
            img = cv2.merge((bH, gH, rH))
            return Image.fromarray(img)
        elif eh_type == 'hsv_eh':
            img_hsv = cv2.cvtColor(img0, cv2.COLOR_RGB2HSV)
            (h, s, v) = cv2.split(img_hsv)
            vH = cv2.equalizeHist(v)
            img_hsv_eh = cv2.merge((h, s, vH))
            img_rgb_eh = cv2.cvtColor(img_hsv_eh,cv2.COLOR_HSV2RGB)
            return Image.fromarray(img_rgb_eh)
        else:
            assert(0)

def main(pretrained_model_path):
    net = FCN8s_ResNet(num_classes=num_classes).cuda()
    kwargs = {'map_location': lambda storage, loc: storage.cuda(0)}
    net = load_GPUS(net, pretrained_model_path, kwargs)
    # net.load_state_dict(torch.load(pretrained_model_path))
    print('training resumes from' + pretrained_model_path)
    net.eval()
    # refer = Reference(net)
    mean_std = ([0.51, 0.51, 0.51], [0.294, 0.297, 0.295])
    input_transform1 = standard_transforms.ToTensor()
    input_transform2 = standard_transforms.Normalize(*mean_std)

    restore_transform = standard_transforms.Compose([
        extended_transforms.DeNormalize(*mean_std),
        standard_transforms.ToPILImage()
    ])
    root = args.data_root
    pic_list = glob.glob('{:s}/**/*.png'.format(root), recursive=True)
    pic_list.sort()
    with torch.no_grad():
        for i, path in enumerate(pic_list):
            percent = round(1.0 * i / len(pic_list) * 100, 2)
            print('当前进度 : %s [%d/%d]' % (str(percent) + '%', i + 1, len(pic_list)), end='\r')
            img = Image.open(path).convert('RGB')

            img = equal_hist(img, eh_type=args.eh_type)
            inputs = input_transform1(img)
            inputs = Variable(inputs, volatile=True).cuda()
            inputs = input_transform2(inputs)
            inputs = inputs.unsqueeze(0)
            outputs = net(inputs)

            pred = outputs.data.max(1, keepdims=True)[1].squeeze_(1).cpu().numpy()
            save_path = path.replace('.png', '.npy')
            save_dir = os.path.dirname(save_path)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)

            np.save(save_path, pred)


if __name__ == '__main__':
    args = init_args()
    main(args.model_path)

