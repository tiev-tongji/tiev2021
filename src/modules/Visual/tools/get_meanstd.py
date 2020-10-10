from PIL import Image
import numpy as np
import os
import glob
import cv2
import shutil
import pdb

#root = '/mnt/ExtraDisk/Unit_seg_datasets/Color_img'
video_path = '/home/neousys/Desktop/tiev2019/src/modules/Visual/ckpt/record.avi'
imgs = np.zeros([1024,256,3,1],dtype=np.uint8)
#data_path_list = glob.glob('{:s}/**/*.png'.format(root), recursive=True)

counter = [0,0,0]
sum = [0,0,0]

#for i, image_path in enumerate(data_path_list):
cap = cv2.VideoCapture(video_path)
i = 0
while cap.isOpened():

    ret, img = cap.read()
    #percent = round(1.0 * i / len(data_path_list) * 100, 2)
   # print('当前进度 : %s [%d/%d]' % (str(percent) + '%', i + 1, len(data_path_list)), end='\r')

    #img = Image.open(image_path).convert('RGB')
    img = np.asarray(img)
    try:
        (b, g, r) = cv2.split(img)
    except:
        pdb.set_trace()

    bH = cv2.equalizeHist(b)
    gH = cv2.equalizeHist(g)
    rH = cv2.equalizeHist(r)
    # 合并每一个通道
    img = cv2.merge((bH, gH, rH))
    img = img[:, :, :, None]

    means, stdevs = [], []
    imgs2 = img.astype(np.float32) / 255
    for ind in range(3):
        pix = imgs2[:, :, ind, :].ravel()
        sum[ind] += np.sum(pix)
        counter[ind] += len(pix)

        # means.append(np.mean(pix))
        # stdevs.append(np.std(pix))
    means = [sum[i]/counter[i] for i in range(3)]
    i = i+ 1
    if (i+1) % 200 == 0:
        print(means)
'''

    for ind in range(3):
        pix = imgs2[:, :, ind, :].ravel()
        std_2 = [(pix[ii] - mean[ind]) * (pix[ii] - mean[ind]) for ii in range(len(pix))]
        sum[ind] += np.sum(std_2)
        counter[ind] += len(pix)

        # means.append(np.mean(pix))
        # stdevs.append(np.std(pix))
    std = [sum[i]/counter[i] for i in range(3)]
    if (i+1) % 200 == 0:
        print(std)

    # print(stdevs)
        # pdb.set_trace()
'''

# imgs = imgs.astype(np.float32)/255
# for ind in range(3):
#     pix = imgs[:, :, ind, :].ravel()
#     means.append(np.mean(pix))
#     stdevs.append(np.std(pix))


