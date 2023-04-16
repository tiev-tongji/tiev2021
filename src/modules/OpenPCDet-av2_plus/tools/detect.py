import argparse
import glob
import os
from pathlib import Path

import numpy as np
import torch

from tensorboardX import SummaryWriter

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from pcdet.utils import box_utils, calibration_kitti
from pcdet.datasets.processor.data_processor import DataProcessor
from pcdet.datasets.processor.point_feature_encoder import PointFeatureEncoder
from pcdet.datasets import build_dataloader
from collections import defaultdict
import time

# os.environ["CUDA_VISIBLE_DEVICES"] = "3"
# cfg.ROOT_DIR = (Path(__file__).resolve().parent / '../').resolve()


def get_calib(calib_file):
    return calibration_kitti.Calibration(calib_file)

def get_lidar(lidar_file):
    return np.fromfile(str(lidar_file), dtype=np.float32).reshape(-1, 4)

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='/home/autolab/tiev/src/modules/OpenPCDet-av2_plus/tools/cfgs/argoverse2_models/centerpoint.yaml',
                        help='specify the config for demo')
    parser.add_argument('--ckpt', type=str, default="/home/autolab/tiev/src/modules/OpenPCDet-av2_plus/checkpoint_epoch_39.pth", help='specify the pretrained model')
    # parser.add_argument('--cfg_file', type=str, default='/home/autolab/huyinghao/projects/tiev_plus_lidar/OpenPCDet_plus/tools/cfgs/argoverse2_models/centerpoint.yaml',
    #                     help='specify the config for demo')
    # parser.add_argument('--ckpt', type=str, default="/home/autolab/huyinghao/projects/log/opendet_av2/centerpoint/wo_map/checkpoint_epoch_20.pth", help='specify the pretrained model')

    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg


class CENTERPOINT_RCNN():
    def __init__(self):
        self.args, self.cfg = parse_config()
        self.args.batch_size = 1
        self.dataset_cfg = self.cfg.DATA_CONFIG
        self.class_names = self.cfg.CLASS_NAMES


        self.test_set, self.test_loader, self.sampler = build_dataloader(
            dataset_cfg=self.dataset_cfg,
            class_names=self.class_names,
            batch_size=self.args.batch_size,
            dist=None, workers=1, training=False
        )

        self.model = build_network(model_cfg=self.cfg.MODEL, num_class=len(self.cfg.CLASS_NAMES), dataset=self.test_set)
        self.model.load_params_from_file1(filename=self.args.ckpt, to_cpu=False)
        self.model.cuda()
        self.model.eval()

        self.point_cloud_range = np.array(self.dataset_cfg.POINT_CLOUD_RANGE,
                                          dtype=np.float32)
        self.point_feature_encoder = PointFeatureEncoder(
            self.dataset_cfg.POINT_FEATURE_ENCODING,
            point_cloud_range=self.point_cloud_range
        )
        self.data_processor = DataProcessor(
            self.dataset_cfg.DATA_PROCESSOR,
            point_cloud_range=self.point_cloud_range,
            training=False,
            num_point_features=self.point_feature_encoder.num_point_features
        )


    def prepare_data(self, data_dict):
        """
        Args:
            data_dict:
                points: optional, (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                ...

        Returns:
            data_dict:
                frame_id: string
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                use_lead_xyz: bool
                oxels: optional (num_oxels, max_points_per_oxel, 3 + C)
                oxel_coords: optional (num_oxels, 3)
                oxel_num_points: optional (num_oxels)
                ...
        """
        if data_dict.get('points', None) is not None:
            data_dict = self.point_feature_encoder.forward(data_dict)

        data_dict = self.data_processor.forward(
            data_dict=data_dict
        )

        data_dict.pop('gt_names', None)

        return data_dict

    def collate_batch(self, data_dict, _unused=False):
        for key, val in data_dict.items():
            if key in ['points', 'voxel_coords']:
                data_dict[key] = np.pad(data_dict[key], ((0, 0), (1, 0)),
                                    mode='constant', constant_values=0)
            elif key in ["images", "depth_maps"]:
                # Get largest image size (H, W)
                max_h = 0
                max_w = 0
                max_h = max(max_h, val.shape[0])
                max_w = max(max_w, val.shape[1])

                # Change size of images
                pad_h = common_utils.get_pad_params(desired_size=max_h,
                                                    cur_size=
                                                    val.shape[0])
                pad_w = common_utils.get_pad_params(desired_size=max_w,
                                                    cur_size=
                                                    val.shape[1])
                pad_width = (pad_h, pad_w)
                pad_value = 0

                if key == "images":
                    pad_width = (pad_h, pad_w, (0, 0))
                elif key == "depth_maps":
                    pad_width = (pad_h, pad_w)

                data_dict[key] = np.pad(val,
                                    pad_width=pad_width,
                                    mode='constant',
                                    constant_values=pad_value)
            elif key in ['calib']:
                data_dict[key] = val
            elif key in ["points_2d"]:
                max_len = len(data_dict[key])
                pad_value = 0
                pad_width = ((0, max_len - len(data_dict[key])), (0, 0))
                data_dict[key] = np.pad(data_dict[key],
                                    pad_width=pad_width,
                                    mode='constant',
                                    constant_values=pad_value)
            else:
                pass

        return data_dict

    def cart_to_hom(self, pts):
        """
        :param pts: (N, 3 or 2)
        :return pts_hom: (N, 4 or 3)
        """
        pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1), dtype=np.float32)))
        return pts_hom

    '''
    # def lidar_to_rect(self, pts_lidar):
    #     """
    #     :param pts_lidar: (N, 3)
    #     :return pts_rect: (N, 3)
    #     """
    #     pts_lidar_hom = self.cart_to_hom(pts_lidar)
    #     pts_rect = np.dot(pts_lidar_hom, np.dot(self.2C.T, self.R0.T))
    #     # pts_rect = reduce(np.dot, (pts_lidar_hom, self.2C.T, self.R0.T))
    #     return pts_rect

    # def boxes3d_lidar_to_kitti_camera(self, boxes3d_lidar, calib):
    #     """
    #     :param boxes3d_lidar: (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center
    #     :param calib:
    #     :return:
    #         boxes3d_camera: (N, 7) [x, y, z, l, h, w, r] in rect camera coords
    #     """
    #     xyz_lidar = boxes3d_lidar[:, 0:3]
    #     l, w, h, r = boxes3d_lidar[:, 3:4], boxes3d_lidar[:, 4:5], boxes3d_lidar[:, 5:6], boxes3d_lidar[:, 6:7]

    #     xyz_lidar[:, 2] -= h.reshape(-1) / 2
    #     xyz_cam = calib.lidar_to_rect(xyz_lidar)
    #     # xyz_cam[:, 1] += h.reshape(-1) / 2
    #     r = -r - np.pi / 2
    #     return np.concatenate([xyz_cam, l, h, w, r], axis=-1)
    '''
    def boxes3d_lidar_to_kitti_camera(self, boxes3d_lidar):
        """
        :param boxes3d_lidar: (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center
        :param calib:
        :return:
            boxes3d_camera: (N, 7) [x, y, z, l, h, w, r] in rect camera coords
        """
        xyz_lidar = boxes3d_lidar[:, 0:3]
        l, w, h, r = boxes3d_lidar[:, 3:4], boxes3d_lidar[:, 4:5], boxes3d_lidar[:, 5:6], boxes3d_lidar[:, 6:7]

        # xyz_lidar[:, 2] -= h.reshape(-1) / 2
        xyz_cam = xyz_lidar
        # xyz_cam[:, 1] += h.reshape(-1) / 2
        r = -r# - np.pi / 2
        # return np.concatenate([xyz_cam, l, h, w, r], axis=-1)
        return np.concatenate([xyz_cam, l, w, h, r], axis=-1)
    

    def generate_prediction_dicts(self, data_dict, pred_dicts, class_names):
        """
        Args:
            batch_dict:
                frame_id:
            pred_dicts: list of pred_dicts
                pred_boxes: (N, 7), Tensor
                pred_scores: (N), Tensor
                pred_labels: (N), Tensor
            class_names:
            output_path:

        Returns:

        """
        def get_template_prediction(num_samples):
            ret_dict = {
                'name': np.zeros(num_samples), 'truncated': np.zeros(num_samples),
                'occluded': np.zeros(num_samples), 'alpha': np.zeros(num_samples),
                'bbox': np.zeros([num_samples, 4]), 'dimensions': np.zeros([num_samples, 3]),
                'location': np.zeros([num_samples, 3]), 'rotation_y': np.zeros(num_samples),
                'score': np.zeros(num_samples), 'boxes_lidar': np.zeros([num_samples, 7])
            }
            return ret_dict

        pred_scores = pred_dicts['pred_scores'].cpu().numpy()
        pred_boxes = pred_dicts['pred_boxes'].cpu().numpy()
        pred_labels = pred_dicts['pred_labels'].cpu().numpy()
        pred_dict = get_template_prediction(pred_scores.shape[0])
        if pred_scores.shape[0] == 0:
            return []

        pred_boxes_camera = self.boxes3d_lidar_to_kitti_camera(pred_boxes)
	
        pred_dict['labels'] = pred_labels - 1
        pred_dict['name'] = np.array(class_names)[pred_labels - 1]
        pred_dict['alpha'] = -np.arctan2(-pred_boxes[:, 1], pred_boxes[:, 0]) + pred_boxes_camera[:, 6]
        pred_dict['dimensions'] = pred_boxes_camera[:, 3:6]
        pred_dict['location'] = pred_boxes_camera[:, 0:3]
        pred_dict['rotation_y'] = pred_boxes_camera[:, 6]
        pred_dict['score'] = pred_scores
        pred_dict['boxes_lidar'] = pred_boxes

        ret_dicts = []
        for i, cur_class_name in enumerate(pred_dict['name']):
            ret_dict = []
            # 类名
            ret_dict.append(pred_dict['labels'][i])
            # 坐标[x, y, z]
            ret_dict.append(pred_dict['location'][i][0])
            ret_dict.append(pred_dict['location'][i][1])
            ret_dict.append(pred_dict['location'][i][2])
            # 形状[l, h, w]
            ret_dict.append(pred_dict['dimensions'][i][0])
            ret_dict.append(pred_dict['dimensions'][i][1])
            ret_dict.append(pred_dict['dimensions'][i][2])
            # 旋转
            ret_dict.append(pred_dict['rotation_y'][i])
            # 置信度
            ret_dict.append(pred_dict['score'][i])

            ret_dicts.append(ret_dict)

        return ret_dicts


    def detect(self, points=None):
        if points is None:
            points = get_lidar('/moyujian/moyujian/Data/yujmo/KITTI/object/training/velodyne/000028.bin')

        input_dict = {
            'points': points,
            'calib': None,
        }
        data_dict = self.prepare_data(input_dict)
        #print(data_dict)
        # batch_dict1['voxel_coords'] = np.hstack((np.zeros((batch_dict1['oxel_coords'].shape[0], 1)), batch_dict1['voxel_coords']))
        data_dict['batch_size'] = 1
        data_dict = self.collate_batch(data_dict)
        load_data_to_gpu(data_dict)
        with torch.no_grad():
            pred_dicts, ret_dict = self.model(data_dict)

        annos = self.generate_prediction_dicts(data_dict, pred_dicts[0], self.class_names)

        return annos


centerpoint_rcnn = CENTERPOINT_RCNN()
def get_boxes(databin):
    s_time = time.time()
    result = centerpoint_rcnn.detect(databin)
    e_time = time.time()
    print(e_time - s_time)
    return result

def detect_test():
    points = get_lidar('/home/autolab/tiev/src/modules/OpenPCDet-av2_plus/000005.bin')
    result = get_boxes(points)
    for pre in result:
        print(pre)


if __name__ == '__main__':
    detect_test()
