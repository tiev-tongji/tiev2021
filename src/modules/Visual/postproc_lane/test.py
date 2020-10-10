import cv2
import numpy as np
from postproc_lane import process_tensor

def emulate_onehot_predict(predict):
    predict = predict[:, :, 2]
    predict[predict>=250] = 0
    NCHANNELS = 12
    predict1 = np.zeros((predict.shape[0], predict.shape[1], NCHANNELS))
    for i in range(NCHANNELS):
        predict1[:, :, i][predict == i] = 1
    predict1 = np.transpose(predict1, (2, 0, 1))
    predict1 = np.ascontiguousarray(predict1)
    return predict1

# test images is scaled so that the lane width is around 5 pixel
image = cv2.imread('test1_image.png')
predict = cv2.imread('test1_predict.png')
predict_onehot = emulate_onehot_predict(predict)

process_tensor(image, predict_onehot)
