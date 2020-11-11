import random
from PIL import Image, ImageOps
import numpy as np
import numbers


class Compose(object):
    def __init__(self, transforms):
        self.transforms = transforms

    def __call__(self, img, mask):

        assert img.size == mask.size

        for tran in self.transforms:
            img, mask = tran(img, mask)
        return img, mask


class Resize(object):
    def __init__(self, w, h):
        self.w = w
        self.h = h

    def __call__(self, img, mask):

        assert img.size == mask.size

        img = img.crop((0, 0, 1024, 4096))
        img = img.resize((self.w, self.h), Image.BILINEAR)

        mask = mask.crop((0, 0, 1024, 4096))
        mask = mask.resize((self.w, self.h), Image.BILINEAR)

        return img, mask


class Scale(object):
    def __init__(self, size):
        self.size = size

    def __call__(self, img, mask):
        assert img.size == mask.size
        w, h = img.size
        if (w >= h and w == self.size) or (h >= w and h == self.size):
            return img, mask
        if w > h:
            ow = self.size
            oh = int(self.size*h/w)
            return img.resize((ow, oh), Image.BILINEAR), mask.resize((ow, oh), Image.NEAREST)
        else:
            oh = self.size
            ow = int(self.size*w/h)
            return img.resize((ow, oh), Image.BILINEAR), mask.resize((ow, oh), Image.NEAREST)


class RandomCrop(object):
    def __init__(self, size, padding=0):
        if isinstance(size, numbers.Number):
            self.size = (int(size), int(size))
        else:
            self.size = size
        self.padding = padding

    def __call__(self, img, mask):
        if self.padding > 0:
            img = ImageOps.expand(img, border=self.padding, fill=0)
            mask = ImageOps.expand(mask, border=self.padding, fill=0)

        assert img.size == mask.size
        w, h = img.size
        th, tw = self.size
        # print("="*80, (w, h), (tw, th))
        if w == tw and h == th:
            return img, mask
        if w < tw or h < th:
            return img.resize((tw, th), Image.BILINEAR), mask.resize((tw, th), Image.NEAREST)

        x1 = random.randint(0, w-tw)
        y1 = random.randint(0, h-th)
        return img.crop((x1, y1, x1+tw, y1+th)), mask.crop((x1, y1, x1+tw, y1+th))


class RandomHorizontallyFlip(object):
    def __call__(self, img, mask):
        if random.random() < 0.5:
            return img.transpose(Image.FLIP_LEFT_RIGHT), mask.transpose(Image.FLIP_LEFT_RIGHT)
        return img, mask


class CenterCrop(object):
    def __init__(self, size):
        if isinstance(size, numbers.Number):
            self.size = (int(size), int(size))
        else:
            self.size = size

    def __call__(self, img, mask):
        assert img.size == mask.size
        w, h = img.size
        tw, th = self.size

        x1 = int(round((w - tw) / 2.))
        y1 = int(round((h - th) / 2.))
        return img.crop((x1, y1, x1+tw, y1+th)), mask.crop((x1, y1, x1+tw, y1+th))

class UpCrop(object):
    def __init__(self, size):
        if isinstance(size, numbers.Number):
            self.size = (int(size), int(size))
        else:
            self.size = size

    def __call__(self, img, mask):
        assert img.size == mask.size
        w, h = img.size
        tw, th = self.size

        return img.crop((0, 0, tw, th)), mask.crop((0, 0, tw, th))


