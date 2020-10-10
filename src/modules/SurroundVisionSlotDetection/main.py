"""
    Requirements:
        conda create -n [your_env_name] python=3.7
        conda activate [your_env_name]
        conda install pytorch torchvision cudatoolkit=10.0 -c pytorch
        conda install -c conda-forge opencv
        conda install -c anaconda scikit-learn
        conda install numba

        zcm install need to be added
"""

# Shielding the wrong call of opencv-python2.7 on the industrial computer
import sys

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except ValueError:
    pass

import cv2
import time
import torch
from Components.bev_capture import Capture
from Components.panorama import Panorama
from Components.slot_detector import Detector

from zerocm import ZCM
from zcm_msg.structPARKINGSLOTS import structPARKINGSLOTS
from zcm_msg.Slot import Slot


def mask_image(file_name, image, detect_results, resolution=600, fps=0.0, show=True):
    for detect_result in detect_results:
        pt0 = (int((detect_result[0][0] + 5) * resolution / 9.6), int((2 - detect_result[0][1]) * resolution / 9.6))
        pt1 = (int((detect_result[1][0] + 5) * resolution / 9.6), int((2 - detect_result[1][1]) * resolution / 9.6))
        pt2 = (int((detect_result[2][0] + 5) * resolution / 9.6), int((2 - detect_result[2][1]) * resolution / 9.6))
        pt3 = (int((detect_result[3][0] + 5) * resolution / 9.6), int((2 - detect_result[3][1]) * resolution / 9.6))
        cv2.line(image, pt0, pt1, (0, 255, 0), thickness=2)
        cv2.line(image, pt0, pt2, (0, 0, 255), thickness=2)
        cv2.line(image, pt1, pt3, (0, 0, 255), thickness=2)
        cv2.line(image, pt2, pt3, (0, 0, 255), thickness=2)
    cv2.putText(image, "%.2f fps" % fps, (30, 30), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255))

    if show:
        cv2.imshow(file_name, image)
        cv2.waitKey(20)
        #cv2.destroyAllWindows()
    else:
        cv2.imwrite(file_name, image)


def make_slot_t(slot_result):
    # axis: right +x, front +y
    # origin: vehicle
    slot_info = Slot()
    slot_info.front_left.x = slot_result[0][0]
    slot_info.front_left.y = slot_result[0][1]
    slot_info.front_right.x = slot_result[1][0]
    slot_info.front_right.y = slot_result[1][1]
    slot_info.rear_left.x = slot_result[2][0]
    slot_info.rear_left.y = slot_result[2][1]
    slot_info.rear_right.x = slot_result[3][0]
    slot_info.rear_right.y = slot_result[3][1]
    return slot_info


if __name__ == "__main__":
    # Initial Model
    panorama = Panorama()
    detector = Detector("Parameters/stable_parameter_0914.pkl")
    capture = Capture()
    #exit(0)
    zcm = ZCM('')
    if not zcm.good():
        print("Unable to initialize zcm")
        exit()

    time_span = 1.0
    error_index = 0
    image_index = 0
    i = 0
    while True:
        tic = time.time()
        # Read images
        print("read demo_image")
        demo_image = cv2.resize(panorama.build_panorama(*capture.read()), (224, 224))
        if(image_index % 20 == 0):
            record_image = "RecordImages/%03d.png" % i
            cv2.imwrite(record_image, demo_image)
            i += 1
        image_index += 1
        #cv2.imshow("Frame", demo_image);
        #cv2.waitKey(10)
        #continue
        inference_image = torch.from_numpy(cv2.cvtColor(demo_image, cv2.COLOR_BGR2GRAY)).float().cuda()
       
        try:
            inference_result = detector(inference_image, threshold=0.8)
        except Exception as e:
            error_image_filename = "ErrorImages/%04d.png" % error_index
            print(e, 'Error image saved to %s' % error_image_filename)
            cv2.imwrite(error_image_filename, demo_image)
            error_index += 1
            continue
        print("Wangluookle")
        # ZCM
        slot_msg = structPARKINGSLOTS()
        for result in inference_result:
            slot = make_slot_t(result)
            slot_msg.parking_slots.append(slot)
        slot_msg.num = len(slot_msg.parking_slots)
        slot_msg.timestamp = int(time.time())
        zcm.publish('PARKINGSLOTS', slot_msg)

        toc = time.time()

        # Calculate the time span
        time_span = time_span * 0.5 + 0.5 * (toc - tic)
        infer_fps = 1 / (time_span + 1e-5)
        print("Time used:{:.3f}, FPS:{:.3f}".format(time_span * 1000, infer_fps), end='\r')

        # Visualize the merge image with result
        demo_resolution = 600
        demo_image = cv2.resize(demo_image, (demo_resolution, demo_resolution))
        print("mask_image")
        mask_image("Result.jpg", demo_image, inference_result, resolution=demo_resolution, fps=infer_fps, show=True)

    # import numpy as np
    #
    #
    # def hough_line(heat_map):
    #     heat_map = (heat_map * 255).astype(np.uint8)
    #     return cv2.HoughLinesP(heat_map, 1.0, np.pi / 180, threshold=16, lines=None, minLineLength=56, maxLineGap=32)
    #
    #
    # detector = Detector("Parameters/stable_parameter_0914.pkl")
    # demo_image = cv2.resize(cv2.imread("Debug/reverse/10920.jpg", cv2.IMREAD_GRAYSCALE), (224, 224))
    # inference_image = torch.from_numpy(demo_image).float().cuda()
    # mask, entry, side, mask_points = detector(inference_image, threshold=0.1)
    #
    # print(mask_points)
    # print(hough_line(entry))
    # print(hough_line(side))
    #
    # result = np.concatenate((np.concatenate((demo_image / 256, mask), axis=1),
    #                          np.concatenate((entry, side), axis=1)), axis=0)
    # cv2.imshow("Result", result)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
