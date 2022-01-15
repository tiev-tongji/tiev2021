from pathlib import Path
import numpy as np
import pickle
import time
from functools import partial
import fire
from second.data import kitti_common as kitti
from second.data.dataset import Dataset, register_dataset
from second.core import box_np_ops
from second.utils.progress_bar import progress_bar_iter as prog_bar
from second.utils.eval import get_coco_eval_result, get_official_eval_result
from second.core import preprocess as prep
@register_dataset
class TievDataset(Dataset):
    NumPointFeatures = 4
    def __init__(self,
                info_path=None,
                root_path=None,
                class_names=None,
                prep_func=None,
                num_point_features=None):
        self._class_names = class_names
        self._prep_func = prep_func

    def getvoxel(self, bindata):
        input_dict = self.get_sensor_data(bindata)
        example = self._prep_func(input_dict=input_dict)
        example["metadata"] = {}
        if "image_idx" in input_dict["metadata"]:
            example["metadata"] = input_dict["metadata"]
        if "anchors_mask" in example:
            example["anchors_mask"] = example["anchors_mask"].astype(np.uint8)
        return example

    def get_sensor_data(self, bindata):
        res = {
            "lidar": {
                "type": "lidar",
                "points": None,
            },
            "metadata": {
                "image_idx": 0,
            },
            "calib": None,
            "cam": {}
        }
        res["lidar"]["points"] = bindata
        return res

if __name__ == '__main__':
    fire.Fire()
