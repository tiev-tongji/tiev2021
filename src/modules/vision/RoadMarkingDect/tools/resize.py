import os
import glob
import numpy as np
import torch
import math
from PIL import Image
from torch.utils import data
import pdb


root = '/DataSets2/arrow_picked_ApolloScapeLaneBEV/'

tar = '/DataSets2/small_arrow_picked_ApolloScapeLaneBEV/'

def make_dataset(mode):
    print("开始获取图像文件路径...")
    mode = mode
    toadd_image_path = os.path.join(root, 'ColorImage_road04')
    toadd_data_path_list = glob.glob('{:s}/**/*.png'.format(toadd_image_path), recursive=True)
    if mode == "train":
        image_path = os.path.join(root, 'ColorImage_road02')
        # import pdb;pdb.set_trace()
        train_data_path_list1 = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        image_path = os.path.join(root, 'ColorImage_road03')
        train_data_path_list2 = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        data_path_list = train_data_path_list1 + train_data_path_list2 + toadd_data_path_list[5810:]
        # data_path_list = data_path_list[:20]
    elif mode == 'valid':
        data_path_list = toadd_data_path_list
    elif mode == 'test':
        image_path = os.path.join(root, 'Test_ColorImage_road05')
        data_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)

    return data_path_list


labels_num = [0 for i in range(37)]

classid = np.arange(0,36)
classid = np.append(classid, 255)

for mode in ['train', 'valid', 'test']:
# for mode in ['valid']:

    img_path_list = make_dataset(mode)
    for i, img_path in enumerate(img_path_list):
        percent = round(1.0 * i / len(img_path_list) * 100, 2)
        print('当前进度 : %s [%d/%d]' % (str(percent) + '%', i + 1, len(img_path_list)), end='\r')
        mask_path = img_path.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels').\
            replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')
        if not os.path.exists(mask_path):
            pdb.set_trace()
        img = Image.open(img_path).convert('RGB')
        mask = Image.open(mask_path).convert('RGB')

        # pdb.set_trace()

        img = img.crop((0, 0, 1024, 4096))
        img = img.resize((512, 2048), Image.BILINEAR)

        mask = mask.crop((0, 0, 1024, 4096))
        mask = mask.resize((512, 2048), Image.BILINEAR)

        img_target_path = img_path.replace('arrow_picked_ApolloScapeLaneBEV', 'small_arrow_picked_ApolloScapeLaneBEV')
        mask_tar_path = mask_path.replace('arrow_picked_ApolloScapeLaneBEV', 'small_arrow_picked_ApolloScapeLaneBEV')

        img_target_dir = os.path.dirname(img_target_path)

        if not os.path.exists(img_target_dir):
            os.makedirs(img_target_dir)

        mask_tar_dir = os.path.dirname(mask_tar_path)

        if not os.path.exists(mask_tar_dir):
            os.makedirs(mask_tar_dir)

        img.save(img_target_path)
        mask.save(mask_tar_path)


