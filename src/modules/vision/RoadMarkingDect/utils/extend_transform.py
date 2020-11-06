import torch
from PIL import Image, ImageFilter
import random
import numpy as np
import cv2


class MaskToTensor(object):
    def __call__(self, img):
        return torch.from_numpy(np.array(img, dtype=np.int32)).long()


class DeNormalize(object):
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std

    def __call__(self, tensor):
        for t, m, s in zip(tensor, self.mean, self.std):
            t.mul_(s).add_(m)
        return tensor


class EqualizeHist(object):
    def __call__(self, img):
        img1 = np.asarray(img)
        (b, g, r) = cv2.split(img1)
        bH = cv2.equalizeHist(b)
        gH = cv2.equalizeHist(g)
        rH = cv2.equalizeHist(r)
        # 合并每一个通道
        img = cv2.merge((bH, gH, rH))
        return Image.fromarray(img)

class Add_x_y_Cordinfo(object):
    def __call__(self, img_tensor):
        w = img_tensor.size(2)
        h = img_tensor.size(1)

        add_x_cord = torch.zeros(h, w)
        add_y_cord = torch.zeros(h, w)

        deltax = 2/w
        deltay = 2/h
        x_cord = torch.tensor([-1 + ii * deltax for ii in range(w)])
        y_cord = torch.tensor([-1 + jj * deltay for jj in range(h)])
        for xx in range(h):
            add_x_cord[xx, :] = x_cord
        for yy in range(w):
            add_y_cord[:, yy] = y_cord

        mix0 = torch.cat((add_x_cord.unsqueeze(0), add_y_cord.unsqueeze(0)), 0)
        mix = torch.cat((mix0, img_tensor), 0)

        return mix

        # print(img_tensor.size(2))

