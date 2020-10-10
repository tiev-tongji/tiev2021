import os
from models import FCN8s_ResNet
import torch
import pdb
import utils.joint_transforms as joint_transforms
from PIL import Image
import numpy as np
from torch.autograd import Variable
import torchvision.transforms as standard_transforms
import time

class CenterCrop(object):
    def __init__(self, size):
        self.size = size

    def __call__(self, img):
        w, h = img.size
        th, tw = self.size
        x1 = int(round((w - tw) / 2.))
        y1 = int(round((h - th) / 2.))
        return img.crop((x1, y1, x1+tw, y1+th))

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




def main(loops):
    loops = loops
    net = FCN8s_ResNet(num_classes=38).cuda()
    pretrained_model_path = '/mnt/ExtraDisk/xiaozhou_temp/fcn_resnet2/fcn8s/epoch_4_loss_154150.81835_acc_0.96201_acc-cls_0.29344_mean-iu_0.24716_fwavacc_0.93127_lr_0.0008000000.pth'
    print('training resumes from' + pretrained_model_path)
    kwargs = {'map_location': lambda storage, loc: storage.cuda(2)}
    net = load_GPUS(net, pretrained_model_path, kwargs)

    # pretrained_model = torch.load(pretrained_model_path)
    # net.load_state_dict(pretrained_model)
    net.eval()
    mean_std = ([0.464, 0.442, 0.431], [0.276, 0.285, 0.276])
    trans = CenterCrop((512, 2048))
    input_transform1 = standard_transforms.ToTensor()
    input_transform2 = standard_transforms.Normalize(*mean_std)
    time_os = 0
    time_net = 0
    time_all = 0
    time_open = 0
    time_trans = 0
    time_inp_trans = 0
    time_it1 = 0
    time_it2 = 0
    time_unsquzz = 0
    time_Vari = 0
    image_path = '/mnt/ExtraDisk/small_arrow_picked_ApolloScapeLaneBEV/ColorImage_road02/ColorImage/Record001/Camera 5/170927_063847007_Camera_5_bv.png'
    img_roi = Image.open(image_path).convert('RGB')
    for i in range(loops):
        time1 = time.time()
        with torch.no_grad():

            time2 = time.time()
            time_open += time2 - time1

            img = trans(img_roi)

            time3 = time.time()
            time_trans += time3 - time2

            # pdb.set_trace()
            img = input_transform1(img)

            time4 = time.time()
            time_it1 += time4 - time3

            img = Variable(img, volatile=True).cuda()

            img = input_transform2(img)

            time5 = time.time()
            # pdb.set_trace()
            time_it2 += time5 - time4

            img = img.unsqueeze(0)

            time6 = time.time()
            time_unsquzz += time6 - time5

            time_os += time6 - time1

            outputs = net(img)
            # pdb.set_trace()
            time7 = time.time()
            time_net += time7 - time6
            time_all += time7 - time1
            # predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
            # predictions_pil = Apollo_data.colorize_mask(predictions[0])
    print("open时间:%.5fms/%d张图" % (time_open * 1000, loops))
    print("open_fps:%.5f" % (loops / time_open))
    print("trans时间:%.5fms/%d张图" % (time_trans * 1000, loops))
    print("trans_fps:%.5f" % (loops / time_trans))
    print("input_trans1时间:%.5fms/%d张图" % (time_it1 * 1000, loops))
    print("input_trans1_fps:%.5f" % (loops / time_it1))
    print("input_trans2时间:%.5fms/%d张图" % (time_it2 * 1000, loops))
    print("input_trans2_fps:%.5f" % (loops / time_it2))
    print("unsquzze时间:%.5fms/%d张图" % (time_unsquzz * 1000, loops))
    print("unsquzze_fps:%.5f" % (loops / time_unsquzz))
    # print("Vari_trans时间:%.5fms/%d张图" % (time_Vari * 1000, loops))
    # print("Vari_fps:%.5f" % (100 / time_Vari))

    print("os时间:%.5fms/%d张图" % (time_os*1000, loops))
    print("os_fps:%.5f" % (loops/time_os))
    print("net时间:%.5fms/%d张图" % (time_net * 1000, loops))
    print("net_fps:%.5f" % (loops / time_net))
    print("总时间:%.5fms/%d张图" % (time_all * 1000, loops))
    print("总fps:%.5f" % (loops / time_all))

if __name__== '__main__':

    main(100)


