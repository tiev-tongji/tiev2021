import cv2


class Capture:
    def __init__(self, img_width=640, img_height=480):
        self.img_width = img_width
        self.img_height = img_height

        self.cap_id = {
            'f': 2,
            'l': 0,
            'r': 1,
            'b': 3,
        }

        self.caps = {}
        for k, v in self.cap_id.items():
            self.caps[k] = cv2.VideoCapture(v)
            self.caps[k].set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
            self.caps[k].set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)

    def read(self):
        if not all([v.isOpened() for k, v in self.caps.items()]):
            return None
        images = {}
        for cam_id, cap in self.caps.items():
            
            ret, images[cam_id] = cap.read()
            images[cam_id] = cv2.flip(images[cam_id], -1)
        
            if not ret:
                return None
        return images['b'], images['f'], images['l'], images['r']
