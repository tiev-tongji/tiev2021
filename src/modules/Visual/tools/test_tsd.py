import cv2
import numpy as np
from pathlib import Path
from pdb import set_trace as b
import sys

sys.path.append('/home/zhousihong/Desktop/git_res/tiev2019/src/modules/Visual/postproc_lane/')
from postproc_lane import process_tensor

root = Path('TSD_BEV_test/')
imgp = root / 'images'
lblp = root / 'labels'
for lblf in lblp.iterdir():
    imgf = imgp / (str(lblf.name)[:-3]+'png')
    if not imgf.exists():
        continue
    lbl = np.load(str(lblf))
    lbl = np.stack(tuple(cv2.resize(lbl[i], (256, 1024)) for i in range(38)))
    img = cv2.resize(cv2.imread(str(imgf)), (256, 1024))
    process_tensor(img, lbl)
