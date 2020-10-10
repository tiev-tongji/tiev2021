import cv2
import copy
import numpy as np
import numba as nb
from numba import jit, jitclass
import warnings

warnings.filterwarnings("ignore")
spec = [('img', nb.uint8[:, :, ::1]),
        ('imgDis', nb.uint8[:, :, ::1]),
        ('map1', nb.float32[:, ::1]),
        ('map2', nb.float32[:, ::1])]


@jitclass(spec)
class Param(object):
    def __init__(self, img, img_dis, map1, map2):
        self.img = img
        self.imgDis = img_dis
        self.map1 = map1
        self.map2 = map2


@jit(parallel=True)
def thread_map(pb):
    if pb.img is not None:
        pb.imgDis = cv2.remap(pb.img, pb.map1, pb.map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    return pb.imgDis


class Panorama(object):
    def __init__(self, image_width=1000, image_height=1000):
        self.image_width = image_width
        self.image_height = image_height

        # Read files
        self.mask_back = cv2.resize(cv2.imread("Parameters/mask_back.bmp"), (image_width, image_height))
        self.mask_front = cv2.resize(cv2.imread("Parameters/mask_front.bmp"), (image_width, image_height))
        self.mask_left = cv2.resize(cv2.imread("Parameters/mask_left.bmp"), (image_width, image_height))
        self.mask_right = cv2.resize(cv2.imread("Parameters/mask_right.bmp"), (image_width, image_height))
        self.mask = cv2.resize(cv2.imread("Parameters/mask.bmp", cv2.IMREAD_GRAYSCALE), (image_width, image_height))

        fs = cv2.FileStorage("Parameters/panorama.yml", cv2.FileStorage_READ)
        intri_front = fs.getNode('intri_front').mat()
        intri_back = fs.getNode('intri_back').mat()
        intri_left = fs.getNode('intri_left').mat()
        intri_right = fs.getNode('intri_right').mat()
        distort_front = fs.getNode('distort_front').mat()
        distort_back = fs.getNode('distort_back').mat()
        distort_left = fs.getNode('distort_left').mat()
        distort_right = fs.getNode('distort_right').mat()
        homo_back = fs.getNode('Homography_back').mat()
        homo_front = fs.getNode('Homography_front').mat()
        homo_left = fs.getNode('Homography_left').mat()
        homo_right = fs.getNode('Homography_right').mat()
        fs.release()

        # Create Maps
        cm2 = copy.deepcopy(intri_right)
        cm2[0, 0] = intri_right[0, 0] * 1.0 / 2.0
        cm2[1, 1] = intri_right[1, 1] * 1.0 / 2.0
        cm2[0, 2] = intri_right[0, 2] + 200
        cm2[1, 2] = intri_right[1, 2] + 100

        map1_1, map1_2 = cv2.fisheye.initUndistortRectifyMap(
            intri_back, distort_back, np.eye(3), cm2, (image_width, image_height), cv2.CV_32F)
        map2_1, map2_2 = cv2.fisheye.initUndistortRectifyMap(
            intri_front, distort_front, np.eye(3), cm2, (image_width, image_height), cv2.CV_32F)
        map3_1, map3_2 = cv2.fisheye.initUndistortRectifyMap(
            intri_left, distort_left, np.eye(3), cm2, (image_width, image_height), cv2.CV_32F)
        map4_1, map4_2 = cv2.fisheye.initUndistortRectifyMap(
            intri_right, distort_right, np.eye(3), cm2, (image_width, image_height), cv2.CV_32F)

        self.map11 = [cv2.warpPerspective(map1_1, homo_back, (image_width, image_height)),
                      cv2.warpPerspective(map2_1, homo_front, (image_width, image_height)),
                      cv2.warpPerspective(map3_1, homo_left, (image_width, image_height)),
                      cv2.warpPerspective(map4_1, homo_right, (image_width, image_height))]
        self.map22 = [cv2.warpPerspective(map1_2, homo_back, (image_width, image_height)),
                      cv2.warpPerspective(map2_2, homo_front, (image_width, image_height)),
                      cv2.warpPerspective(map3_2, homo_left, (image_width, image_height)),
                      cv2.warpPerspective(map4_2, homo_right, (image_width, image_height))]

    def build_panorama(self, image_back, image_front, image_left, image_right):
        image_dis = np.zeros((self.image_width, self.image_height, 3), dtype=np.uint8)
        param_back = Param(image_back, image_dis, self.map11[0], self.map22[0])
        param_front = Param(image_front, image_dis, self.map11[1], self.map22[1])
        param_left = Param(image_left, image_dis, self.map11[2], self.map22[2])
        param_right = Param(image_right, image_dis, self.map11[3], self.map22[3])
        param = [param_back, param_front, param_left, param_right]

        res = list(range(4))
        for i in range(4):
            res[i] = thread_map(param[i])

        image_back_dis = cv2.bitwise_and(res[0], self.mask_back)
        image_front_dis = cv2.bitwise_and(res[1], self.mask_front)
        image_back_front = cv2.bitwise_or(image_back_dis, image_front_dis)
        image_left_dis = cv2.bitwise_and(res[2], self.mask_left)
        image_right_dis = cv2.bitwise_and(res[3], self.mask_right)
        image_left_right = cv2.bitwise_or(image_left_dis, image_right_dis)
        result = cv2.bitwise_or(image_left_right, image_back_front)

        return result
