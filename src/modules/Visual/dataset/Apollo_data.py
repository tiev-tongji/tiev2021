import os
import glob
import numpy as np
import torch
from PIL import Image
from torch.utils import data
import pdb


num_classes = 38
ignore_label = 255

# root = '/DataSets2/arrow_picked_ApolloScapeLaneBEV/'
root = '/DataSets2/small_arrow_picked_ApolloScapeLaneBEV/'

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


def make_dataset(mode):
    print("开始获取图像文件路径...")
    mode = mode
    # toadd_image_path = os.path.join(root, 'ColorImage_road04')
    # toadd_data_path_list = glob.glob('{:s}/**/*.png'.format(toadd_image_path), recursive=True)
    if mode == "train":
        image_path = os.path.join(root, 'ColorImage_road02')
        # import pdb;pdb.set_trace()
        train_data_path_list1 = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        image_path = os.path.join(root, 'ColorImage_road03')
        train_data_path_list2 = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        image_path = os.path.join(root, 'ColorImage_road04')
        train_data_path_list3 = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        data_path_list = train_data_path_list1 + train_data_path_list2 + train_data_path_list3
        # data_path_list = data_path_list[:20]
    elif mode == 'valid':
        image_path = os.path.join(root, 'ColorImage_road04_valid')
        # import pdb;pdb.set_trace()
        data_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)
        # data_path_list = toadd_data_path_list[:5810]
    elif mode == 'test':
        image_path = os.path.join(root, 'Test_ColorImage_road05')
        data_path_list = glob.glob('{:s}/**/*.png'.format(image_path), recursive=True)

    return data_path_list


class Apollo(data.Dataset):

    def __init__(self, mode, joint_transform=None,  transform=None, target_transform=None):
        self.img_path_list = make_dataset(mode)
        # self.img_path_list = self.img_path_list[0: 160]
        if len(self.img_path_list) == 0:
            raise RuntimeError('Found 0 images, please check your data set')
        self.mode = mode
        self.joint_transform = joint_transform
        self.transform = transform
        self.target_transform = target_transform

    def __getitem__(self, index):
        # pdb.set_trace()
        image_path = self.img_path_list[index]
        mask_path = image_path.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels').\
            replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')
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

        return img, mask


    def __len__(self):
        return len(self.img_path_list)
