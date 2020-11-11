import cv2
import numpy as np
from pathlib import Path
from postproc_lane import process_tensor
from pdb import set_trace as b

import sys
sys.path.append('../')
from dataset import Apollo_data

idx_new2old = {new : old for new, old in enumerate([1,2,3,4,5,6,7,8,9,10,11,12,13,18,19,20,21,22,23,24,25,26,27,28,30])}
idx_old2new = {old : new for new, old in enumerate([1,2,3,4,5,6,7,8,9,10,11,12,13,18,19,20,21,22,23,24,25,26,27,28,30])}

root = Path('TIEV2019_IMU/')
imgp = root / 'images'
lblp = root / 'labels'
count_i = 0
for lblf in sorted(lblp.iterdir()):
    count_i += 1
    if count_i <= 2200:#1300+200
        continue
    print(count_i)
    imgf = imgp / (str(lblf.name)[:-3]+'png')
    if not imgf.exists():
        continue
    try:
        old_lbl = np.load(str(lblf))
    except:
        continue
    #lbl = np.stack(tuple(cv2.resize(lbl[i], (256, 1024)) for i in range(38)))
    new_lbl = np.stack(tuple(cv2.resize(old_lbl[oldid], (256, 1024)) for newid, oldid in idx_new2old.items()))
    img = cv2.resize(cv2.imread(str(imgf)), (256, 1024))
    retval = process_tensor(img, new_lbl)
    print("lane size:", retval)

    pre = np.zeros((1024, 256), np.uint8)

    for newid, oldid in idx_new2old.items():
        pre += new_lbl[newid] * oldid

    #for ii in range(38):
    #    #pdb.set_trace()
    #    pre += lbl[ii]*ii
    #    continue

    #    aa = np.where(lbl[ii] == ii)
    #    ax=aa[0]
    #    ay=aa[1]
    #    for jj in range(len(ax)):
    #        pre[ay[jj], ax[jj]]=ii

    #predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
    predictions_pil = Apollo_data.colorize_mask(pre)
    predictions_pil = predictions_pil.convert('RGB')
    pre_np = np.asarray(predictions_pil)
    pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
    mask_img = cv2.addWeighted(img, 1.0, pre_np, 1.0, 0)
    mix_img = np.concatenate((img, pre_np, mask_img), axis=1)
    cv2.namedWindow('title', cv2.WINDOW_NORMAL)
    cv2.imshow('title', mix_img)
    cv2.waitKey(0);

#    if retval < 0:
#        b()
