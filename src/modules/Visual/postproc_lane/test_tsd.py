import cv2
import numpy as np
from pathlib import Path
from postproc_lane import process_tensor
import sys
sys.path.append('/home/zhousihong/Desktop/git_res/tiev2019/src/modules/Visual')
from dataset import Apollo_data
import pdb

root = Path('/home/zhousihong/Desktop/data')
imgp = root / 'arrow_jpg'
lblp = root / 'arrow_npy'
count_i = 0
for lblf in sorted(lblp.iterdir()):
    count_i += 1
    #if count_i <= 2150:
    #    continue
    print(count_i)
    imgf = imgp / (str(lblf.name)[:-3]+'png')
    if not imgf.exists():
        continue
    lbl = np.load(str(lblf))
    lbl = np.stack(tuple(cv2.resize(lbl[i], (256, 1024)) for i in range(38)))
    img = cv2.resize(cv2.imread(str(imgf)), (256, 1024))
    print("lane size:",process_tensor(img, lbl))
    
    pre = np.zeros((1024, 256), np.uint8)
    
    for ii in range(38):
        #pdb.set_trace()
        pre += lbl[ii]*ii
        continue
        
        aa = np.where(lbl[ii] == ii)
        ax=aa[0]
        ay=aa[1]
        for jj in range(len(ax)):
            pre[ay[jj], ax[jj]]=ii
    
    #predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
    predictions_pil = Apollo_data.colorize_mask(pre)
    predictions_pil = predictions_pil.convert('RGB')
    pre_np = np.asarray(predictions_pil)
    pre_np = cv2.cvtColor(pre_np, cv2.COLOR_RGB2BGR)
    mask_img = cv2.addWeighted(img, 1.0, pre_np, 1.0, 0)
    mix_img = np.concatenate((img, pre_np, mask_img), axis=1)
    cv2.namedWindow('title', cv2.WINDOW_NORMAL)
    cv2.imshow('title', mix_img)
    k = cv2.waitKey(0)
    if k == 27:
         break



