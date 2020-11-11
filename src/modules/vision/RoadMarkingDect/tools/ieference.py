from models import FCN8s, FCN8s_bn, FCN8s_ResNet
import torch
import torchvision.transforms as standard_transforms
from PIL import Image
from torch.autograd import Variable
from dataset import Apollo_data
import os
import pdb
import glob
import numpy as np
import cv2
import utils.extend_transform as extended_transforms
import copy



num_classes = 38

class CenterCrop(object):
    def __init__(self, size):
        self.size = size

    def __call__(self, img):
        w, h = img.size
        th, tw = self.size
        x1 = int(round((w - tw) / 2.))
        y1 = int(round((h - th) / 2.))
        return img.crop((x1, y1, x1+tw, y1+th))


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




def main(pretrained_model_path, image_path='/mnt/ExtraDisk/arrow_picked_ApolloScapeLaneBEV/ColorImage_road02/ColorImage/Record001/Camera 5/170927_063847007_Camera_5_bv.png'):
    net = FCN8s_ResNet(num_classes=num_classes).cuda()
    # net = FCN8s_bn(num_class=num_classes).cuda()
    to_save_dir = '/home/renxiaozhou/code/SegNet_bymyself/temp_save/'
    to_save_dir = 'temp_save'

    kwargs = {'map_location': lambda storage, loc: storage.cuda(0)}
    net = load_GPUS(net, pretrained_model_path, kwargs)
    # net.load_state_dict(torch.load(pretrained_model_path))
    print('training resumes from' + pretrained_model_path)
    net.eval()
    # refer = Reference(net)
    mean_std = ([0.51, 0.51, 0.51], [0.294, 0.297, 0.295])
    # mean_std = ([0.564, 0.542, 0.531], [0.276, 0.285, 0.276])
    trans = CenterCrop((2048, 1024))
    input_transform1 = standard_transforms.ToTensor()
    input_transform2 = standard_transforms.Normalize(*mean_std)

    restore_transform = standard_transforms.Compose([
        extended_transforms.DeNormalize(*mean_std),
        standard_transforms.ToPILImage()
    ])
    i = 1
    # root = '/DataSets2/arrow_picked_ApolloScapeLaneBEV/'
    # root = '/mnt/ExtraDisk/Unit_seg_datasets/Color_img/train/TSD_Lane'
    root = '/mnt/ExtraDisk/TiEV2019'
    root = '/home/autolab/huangyuyao/SEG_1/tools/temp_image'
    toadd_image_path = os.path.join(root, 'ColorImage_road04')
    toadd_data_path_list = glob.glob('{:s}/**/*.png'.format(root), recursive=True)
    # toadd_data_path_list = glob.glob('{:s}/**/*.png'.format(toadd_image_path), recursive=True)
    # toadd_data_path_list = toadd_data_path_list[:5810]
    with torch.no_grad():
        for i, path in enumerate(toadd_data_path_list):
            percent = round(1.0 * i / len(toadd_data_path_list) * 100, 2)
            print('当前进度 : %s [%d/%d]' % (str(percent) + '%', i + 1, len(toadd_data_path_list)), end='\r')
            # print('当前进度：%s 已生成：%d 个npy' % (str(percent)+'%' , i), end='\r')

            img0 = Image.open(path).convert('RGB')
            img1 = np.asarray(img0)
            blank = np.zeros(img1.shape, img1.dtype)
            # img = cv2.addWeighted(img1, 1, blank, 0, -100)
            (b, g, r) = cv2.split(img1)
            bH = cv2.equalizeHist(b)
            gH = cv2.equalizeHist(g)
            rH = cv2.equalizeHist(r)
            # 合并每一个通道
            img = cv2.merge((bH, gH, rH))
            # img = cv2.equalizeHist(img1)
            img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)
            img = Image.fromarray(img)
            # pdb.set_trace()
            # img = img.crop((0, 0, 1024, 4096))
            # img = img.resize((512, 2048), Image.BILINEAR)

            # img = trans(img)
            inputs = input_transform1(img)
            inputs = Variable(inputs, volatile=True).cuda()
            inputs = input_transform2(inputs)


            inputs = inputs.unsqueeze(0)
            outputs = net(inputs)

            # aa = outputs.data.max(1)[1]
            #
            # for ind in range(num_classes):
            #     bb = (aa == ind)
            #     if ind == 0:
            #         one_hot_out = bb
            #     else:
            #         one_hot_out = torch.cat((one_hot_out, bb), 0)
            #
            # one_hot_out = one_hot_out.cpu().numpy()
            # save_path = path.replace('Unit_seg_datasets/Color_img/', 'One_hot_class_npy/').replace('.png', '.npy')
            # save_dir = os.path.dirname(save_path)
            # if not os.path.exists(save_dir):
            #     os.makedirs(save_dir)
            #
            # np.save(save_path, one_hot_out)


            predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
            if i % 100 == 0:
                pdb.set_trace()
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            predictions_pil = predictions_pil.convert('RGB')
            # img_np = np.asarray(img)
            # img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            inputs = inputs.data.cpu()
            input_pil = restore_transform(inputs[0])
            img_np = np.asarray(input_pil)
            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            pre_np = np.asarray(predictions_pil)
            pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
            mask_img = cv2.addWeighted(img_np, 1.0, pre_np, 1.0, 0)

            mix_img = np.concatenate((img1, img_np, pre_np, mask_img), axis=1)

           # cv2.namedWindow("Image")
           # cv2.imshow("Image", mix_img)
           # cv2.waitKey(0)
            cv2.imwrite(os.path.join(to_save_dir, '%d_pred_masked.png' % i), mix_img)
            # mask_img = Image.blend(img, predictions_pil, 0.4)
            # mask_img.save(os.path.join(to_save_dir, '%d_pred_masked.png' % i))
            # print('prediction_masked_image is saved in' + to_save_dir)


if __name__ == '__main__':

    root_dir = '/mnt/ExtraDisk/xiaozhou_temp/fcn_resnet2/fcn8s/'
    # root_dir = '/DataSets2/hyy_vggsegnet_xiaozhou/fcn8s/'
    ckpt_name = 'epoch_4_loss_154150.81835_acc_0.96201_acc-cls_0.29344_mean-iu_0.24716_fwavacc_0.93127_lr_0.0008000000.pth'

    #pretrained_model_path = os.path.join(root_dir, 'epoch_5_loss_403966.92588_acc_0.97011_acc-cls_0.58638_mean-iu_0.40037_fwavacc_0.94615_lr_0.0000000010.pth')
    pretrained_model_path = os.path.join(root_dir, ckpt_name)
    pretrained_model_path = '/home/autolab/huangyuyao/SEG_1/ckpt/epoch_17_loss_126496.77680_acc_0.96925_acc-cls_0.32808_mean-iu_0.28384_fwavacc_0.94385_lr_0.0008000000.pth'

    main(pretrained_model_path)

