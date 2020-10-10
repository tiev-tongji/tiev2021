import os
import glob
import numpy as np
import torch
from PIL import Image
from torch.utils import data
import pdb


num_classes = 38
ignore_label = 255

root = '/mnt/ExtraDisk/Unit_seg_datasets/Color_img/'
# root = '/DataSets2/small_arrow_picked_ApolloScapeLaneBEV/'

palette = [0, 0, 0, 70, 130, 180, 220, 20, 60, 128, 0, 128, 255, 0, 0, 0, 0, 60, 0, 60, 100, 0, 0, 142,
           119, 11, 32, 244, 35, 232, 0, 0, 160, 153, 153, 153, 220, 220, 0, 250, 170, 30, 102, 102, 156, 128, 0, 0,
           128, 64, 128, 238, 232, 170, 190, 153, 153, 0, 0, 230, 128, 128, 0, 128, 78, 160, 150, 100, 100, 255, 165, 0,
           180, 165, 180, 107, 142, 35, 201, 255, 229, 0, 191, 255, 51, 255, 51, 250, 128, 114, 127, 255, 0, 255, 128,
           0, 0, 255, 255, 178, 132, 190, 128, 128, 64, 102, 0, 204, 0, 153, 153, 255, 255, 255]
for i in range(256*3-len(palette)):
    palette.append(0)


def colorize_mask(mask):
    #todo confirm type change
    new_mask = Image.fromarray(mask.astype(np.uint8)).convert('P')
    new_mask.putpalette(palette)

    return new_mask


def make_dataset(mode, bs, set_name):
    print("开始获取图像文件路径...")
    mode = mode

    if mode == "train":
        # data_path_list = []
        image_path = os.path.join(root, 'train/{}'.format(set_name))
        data_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        # for sets in ['Apollo', 'RoadMarking', 'TSD_Lane', 'Tsinghua_TRoM']:
        #     image_path = os.path.join(root, 'train/{}'.format(sets))
        #     to_add_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        #     lenth = len(to_add_path_list) - len(to_add_path_list)%bs
        #     to_add_path_list = to_add_path_list[0: lenth]
        #     data_path_list += to_add_path_list
    elif mode == 'valid':
        image_path = os.path.join(root, 'valid/{}'.format(set_name))
        data_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        # data_path_list = []
        #
        # for sets in ['Apollo', 'RoadMarking', 'TSD_Lane', 'Tsinghua_TRoM']:
        #     image_path = os.path.join(root, 'valid/{}'.format(sets))
        #     to_add_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        #     lenth = len(to_add_path_list) - len(to_add_path_list) % bs
        #     to_add_path_list = to_add_path_list[0: lenth]
        #     data_path_list += to_add_path_list
    elif mode == 'test':
        data_path_list = []
        for sets in ['Apollo', 'RoadMarking', 'TSD_Lane', 'Tsinghua_TRoM']:
            image_path = os.path.join(root, 'test/{}'.format(sets))
            to_add_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
            lenth = len(to_add_path_list) - len(to_add_path_list) % bs
            to_add_path_list = to_add_path_list[0: lenth]
            data_path_list += to_add_path_list

    return data_path_list


class Unit(data.Dataset):

    def __init__(self, mode, bs=1, set_name='', joint_transform=None,  transform=None, target_transform=None):
        self.img_path_list = make_dataset(mode, bs, set_name)
        # self.img_path_list = self.img_path_list[0: 120]
        if len(self.img_path_list) == 0:
            raise RuntimeError('Found 0 images, please check your data set')
        self.mode = mode
        self.joint_transform = joint_transform
        self.transform = transform
        self.target_transform = target_transform

    def __getitem__(self, index):
        # pdb.set_trace()
        from_which_dataset = None
        image_path = self.img_path_list[index]
        mask_path0 = image_path.replace('Color_img', 'Label_img')
        if 'Apollo' in image_path:
            from_which_dataset = 'Apollo'
            mask_path = mask_path0.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels').replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')
        if 'RoadMarking' in image_path:
            from_which_dataset = 'RoadMarking'
            mask_path = mask_path0
        if 'TSD_Lane' in image_path:
            from_which_dataset = 'TSD_Lane'
            mask_path = mask_path0
        if 'Tsinghua_TRoM' in image_path:
            from_which_dataset = 'Tsinghua_TRoM'
            mask_path = mask_path0.replace('Tsinghua_TRoM/', 'Tsinghua_TRoM/gt_')

        if not os.path.exists(mask_path):
            pdb.set_trace()
        img = Image.open(image_path).convert('RGB')
        mask = Image.open(mask_path).convert('RGB')

        mask = np.array(mask)
        assert mask.shape[2] == 3
        mask = mask[:, :, 1]
        #todo confirm astype change sth ornot
        mask = Image.fromarray(mask.astype(np.uint8))
        if self.joint_transform is not None:
            img, mask = self.joint_transform(img, mask)

        if self.transform is not None:
            img = self.transform(img)

        if self.target_transform is not None:
            mask = self.target_transform(mask)

        return img, mask, from_which_dataset


    # def getitem(self, index):
    #     # pdb.set_trace()
    #     image_path = self.img_path_list[index]
    #     mask_path0 = image_path.replace('Color_img', 'Label_img')
    #
    #     if 'Apollo' in image_path:
    #         mask_path = mask_path0.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels').\
    #             replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')
    #     if not os.path.exists(mask_path):
    #         pdb.set_trace()
    #     img = Image.open(image_path).convert('RGB')
    #     mask = Image.open(mask_path).convert('RGB')
    #
    #     mask = np.array(mask)
    #     assert mask.shape[2] == 3
    #     mask = mask[:, :, 1]
    #     #todo confirm astype change sth ornot
    #     mask = Image.fromarray(mask.astype(np.uint8))
    #     if self.joint_transform is not None:
    #         img, mask = self.joint_transform(img, mask)
    #
    #     if self.transform is not None:
    #         img = self.transform(img)
    #
    #     if self.target_transform is not None:
    #         mask = self.target_transform(mask)
    #
    #     return img, mask

    def __len__(self):
        return len(self.img_path_list)
