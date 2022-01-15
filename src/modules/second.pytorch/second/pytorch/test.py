import SECOND as sed
import numpy as np
import os
import time
points=np.fromfile("/home/autolab/tiev/src/modules/second.pytorch/second/pytorch/000005.bin", dtype=np.float32, count=-1).reshape([-1, 4])
print(sed.get_boxes(points))
'''
num=1
path="/home/autolab/tianxuebo/second.pytorch/second/TIEV_DATASET_ROOT/training/velodyne/"
bins=os.listdir(path)
tt=time.time()
points=[]
for file in bins:
    if num>100:
        break
    num+=1
    points.append(np.fromfile(path+file, dtype=np.float32, count=-1).reshape([-1, 4]))
print("read bin sum time:",time.time()-tt)
start=time.time()
for point in points:
    sed.get_boxes(point)
print("get box sum time:",time.time()-start)
'''
