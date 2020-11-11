import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np


curb_img = np.zeros((1024,256,1), dtype=np.uint8)
curb_img[:, 202:207] = [40]
cv2.imwrite("manual_curb.png", curb_img)