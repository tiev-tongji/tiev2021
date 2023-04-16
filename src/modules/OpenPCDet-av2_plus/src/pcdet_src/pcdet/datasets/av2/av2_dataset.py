import copy
import pickle
import torch
import time
import os

import numpy as np
import pandas as pd
from skimage import io
from pathlib import Path
from dataclasses import dataclass, field
#from functools import cached_property
from rich.progress import track

from ...ops.roiaware_pool3d import roiaware_pool3d_utils
from ...utils import box_utils, common_utils
from ..dataset import DatasetTemplate
'''
from av2.geometry import geometry as geometry_utils
from av2.datasets.sensor.sensor_dataloader import SensorDataloader
from av2.datasets.sensor.constants import RingCameras, StereoCameras

from av2.map.map_api import ArgoverseStaticMap, RasterLayerType, DrivableAreaMapLayer
from av2.structures.cuboid import CuboidList
from av2.structures.sweep import Sweep

from av2.utils.io import TimestampedCitySE3EgoPoses, read_city_SE3_ego, read_feather, read_img
'''
from pyarrow import feather
from pcdet.ops.pointnet2.pointnet2_stack.pointnet2_utils import FarthestPointSampling, farthest_point_sample

# CLASSES = ["REGULAR_VEHICLE",  "PEDESTRIAN", "SIGN", "LARGE_VEHICLE", "BICYCLE", "DOG"]
CLASSES = ["Car",  "Pedestrian", "Cyclist", "Large_car"]
Groups = [
    ["REGULAR_VEHICLE"],
    ["PEDESTRIAN"],
    ["BICYCLIST", "MOTORCYCLIST", "BICYCLE", "MOTORCYCLE",],
    ["LARGE_VEHICLE",
    "BUS",
    "BOX_TRUCK",
    "TRUCK",
    "VEHICULAR_TRAILER",
    "TRUCK_CAB",
    "SCHOOL_BUS",
    "ARTICULATED_BUS",
    "MESSAGE_BOARD_TRAILER",],
]


group1 = ["REGULAR_VEHICLE"]
group2 = [
    "PEDESTRIAN",
    "BICYCLIST",
    "MOTORCYCLIST",
    "WHEELED_RIDER",
]

group3 = [
    "BOLLARD",
    "CONSTRUCTION_CONE",
    "SIGN",
    "CONSTRUCTION_BARREL",
    "STOP_SIGN",
    "MOBILE_PEDESTRIAN_CROSSING_SIGN",
]
group4 = [
    "LARGE_VEHICLE",
    "BUS",
    "BOX_TRUCK",
    "TRUCK",
    "VEHICULAR_TRAILER",
    "TRUCK_CAB",
    "SCHOOL_BUS",
    "ARTICULATED_BUS",
    "MESSAGE_BOARD_TRAILER",
]

group5 = [
    "BICYCLE",
    "MOTORCYCLE",
    "WHEELED_DEVICE",
    "WHEELCHAIR",
    "STROLLER",
]
group6 = [
    "DOG",
]

all_classes = group1 + group2 + group3 + group4 + group5 + group6
# CLASSES = tuple([c.lower().capitalize() for c in all_classes])


@dataclass
class Argo2Dataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        if training:
            self.split = 'train'
        else:
            self.split = 'val'
        
        self.logger = logger
        '''
        self.dataloader = SensorDataloader(self.root_path,
                                            with_annotations=True,
                                            with_cache=dataset_cfg.WITH_CACHE,
                                            cam_names=dataset_cfg.CAMERAS,
                                            split=self.split,
                                            logger=self.logger)

        self.split_lidar_cache: pd.DataFrame = self.dataloader.sensor_cache.xs((self.split, slice(None), 'lidar'))
        self.log_id_list = self.split_lidar_cache.index.get_level_values('log_id')
        self.log_id_list = sorted(list(set(self.log_id_list)))
        
        self.use_map = self.dataset_cfg.USE_MAP
        self.pre_load_map = self.dataset_cfg.PRE_LOAD_MAP
        self.pre_load_calib = self.dataset_cfg.PRE_LOAD_CALIB
        self.pre_load_ego_pos = self.dataset_cfg.PRE_LOAD_EGO_POS
        self.pre_load_annos = self.dataset_cfg.PRE_LOAD_ANNOS
        
        self.map_cache = {}
        self.calib_cache = {}
        self.annos_cache = {}
        self.ego_pos_cache = {}
        
        self._load_log_common_cache()
        '''
    '''
    def _load_log_common_cache(self):
        start_time = time.time()
        if self.logger != None:
            self.logger.info("Loading log common information: START")
        
        for i in track(range(len(self.log_id_list))):
            log_dir = self.root_path / self.split / self.log_id_list[i]
            map_dir = log_dir / 'map'
            calib_dir = log_dir / 'calibration'

            if self.use_map and self.pre_load_map:
                avm = ArgoverseStaticMap.from_map_dir(map_dir, build_raster=False)
                self.map_cache[self.log_id_list[i]] = avm
                
            if self.pre_load_ego_pos:
                self.ego_pos_cache[self.log_id_list[i]] = read_city_SE3_ego(log_dir=log_dir)
                
            if self.pre_load_annos:
                if os.path.exists(log_dir / 'annos_kitti.feather'):
                    data = read_feather(log_dir / 'annos_kitti.feather')
                else:
                    data = read_feather(log_dir / 'annotations.feather')
                    data = self._pre_process_of_annos(data)
                    feather.write_feather(data, log_dir / 'annos_kitti.feather')
                    
                self.annos_cache[self.log_id_list[i]] = data
                # cuboids = list(filter(lambda x: x.timestamp_ns == sweep_timestamp_ns, cuboid_list.cuboids))
            
            if self.pre_load_calib:
                pass
        
        end_time = time.time()
        if self.logger != None:
            self.logger.info("Loading log common information: DONE    Time cost: %.4f" % (end_time - start_time))
            
    def _pre_process_of_annos(self, annos_data):
        for i, cur_class in enumerate(annos_data['category']):
            if cur_class in Groups[0]:
                annos_data.loc[i, 'category'] = CLASSES[0]
            elif cur_class in Groups[1]:
                annos_data.loc[i, 'category'] = CLASSES[1]
            elif cur_class in Groups[2]:
                annos_data.loc[i, 'category'] = CLASSES[2]
            elif cur_class in Groups[3]:
                annos_data.loc[i, 'category'] = CLASSES[3]
            else:
                annos_data.loc[i, 'category'] = "Uncare"
        annos_data = annos_data[annos_data.category != "Uncare"].reset_index(drop=True)
        return annos_data
    
    def _convert_classes(self, gt_names):
        for i, cur_gtname in enumerate(gt_names):
            if cur_gtname in group1:
                gt_names[i] = self.class_names[0]
            if cur_gtname in group2:
                gt_names[i] = self.class_names[1]
            if cur_gtname in group3:
                gt_names[i] = self.class_names[2]
            if cur_gtname in group4:
                gt_names[i] = self.class_names[3]
            if cur_gtname in group5:
                gt_names[i] = self.class_names[4]
            if cur_gtname in group6:
                gt_names[i] = self.class_names[5]
        return gt_names
    
    def _convert_classes_new(self, gt_names):
        mask = [True for i in range(gt_names.shape[0])]
        for i, cur_gtname in enumerate(gt_names):
            flag = False
            for j, cur_group in enumerate(Groups):
                if cur_gtname in cur_group:
                    gt_names[i] = self.class_names[j]
                    flag = True

            mask[i] = flag
        return gt_names, mask
    
    def _get_annos(self, split, log_id, timestamp_ns):
        annos_file = self.root_path / split/ log_id/ 'annotations' / f"{timestamp_ns}.feather"
        annos_data = read_feather(annos_file)
        return annos_data

    def _generate_format_annos(self, split, log_id, timestamp_ns):
        if self.dataloader.with_annotations:
            if self.pre_load_annos:
                log_data = self.annos_cache[log_id]
                annos = log_data[log_data.timestamp_ns == timestamp_ns]
                
                rotation = geometry_utils.quat_to_mat(annos.loc[:, ["qw", "qx", "qy", "qz"]].to_numpy())
                translation_m = annos.loc[:, ["tx_m", "ty_m", "tz_m"]].to_numpy()
                length_m = annos.loc[:, "length_m"].to_numpy()
                width_m = annos.loc[:, "width_m"].to_numpy()
                height_m = annos.loc[:, "height_m"].to_numpy()
                category = annos.loc[:, "category"].to_numpy()
                
                loc = translation_m
                dims = np.stack([length_m, width_m, height_m], axis=1)
                rots = geometry_utils.mat_to_xyz(rotation)[:, -1].reshape((-1, 1))
                gt_names = category
                # cuboid_list = self.annos_cache[log_id]
                # annos = CuboidList(list(filter(lambda x: x.timestamp_ns == timestamp_ns, cuboid_list.cuboids)))
            else:
                # annos = self.dataloader._load_annotations(split, log_id, timestamp_ns)
        
                # loc = annos.xyz_center_m
                # dims = annos.dims_lwh_m
                # rots = np.array([geometry_utils.mat_to_xyz(cuboid.dst_SE3_object.rotation)[-1] for cuboid in annos.cuboids]).reshape((-1, 1))
                # gt_names = np.array(annos.categories)
                
                annos = self._get_annos(split, log_id, timestamp_ns)
                rotation = geometry_utils.quat_to_mat(annos.loc[:, ["qw", "qx", "qy", "qz"]].to_numpy())
                translation_m = annos.loc[:, ["tx_m", "ty_m", "tz_m"]].to_numpy()
                length_m = annos.loc[:, "length_m"].to_numpy()
                width_m = annos.loc[:, "width_m"].to_numpy()
                height_m = annos.loc[:, "height_m"].to_numpy()
                category = annos.loc[:, "category"].to_numpy()
                
                loc = translation_m
                dims = np.stack([length_m, width_m, height_m], axis=1)
                rots = geometry_utils.mat_to_xyz(rotation)[:, -1].reshape((-1, 1))
                gt_names = category
                
                
                
        gt_boxes_lidar = np.concatenate([loc, dims, rots], axis=1)
        
        # if self.pre_load_annos:
        #     gt_names, mask = self._convert_classes_new(gt_names)
            
        #     gt_names = gt_names[mask]
        #     gt_boxes_lidar = gt_boxes_lidar[mask]
    
        data_dict = {
            'gt_names': gt_names,
            'gt_boxes': gt_boxes_lidar
        }
        
        return data_dict

    
    def num_sweeps(self) -> int:
        """Return the number of unique lidar sweeps."""
        return int(self.split_counts[self.split])
    
    
    def split_counts(self) -> pd.Series:
        """Return the number of records for each sensor."""
        sensor_counts: pd.Series = self.dataloader.sensor_cache.xs(key='lidar', level=2).index.get_level_values('split').value_counts()
        return sensor_counts

    def __len__(self):
        return self.num_sweeps

    def __getitem__(self, index):
        if self._merge_all_iters_to_one_epoch:
            index = index % self.num_sweeps


        # print(index)
        split = self.split
        log_id, timestamp_ns = self.split_lidar_cache.iloc[index].name
        
        # print(log_id, index)
        
        # start_time = time.time()
        # idx = np.where(self.dataloader.sensor_cache.xs(key='lidar', level=2).index.get_level_values('timestamp_ns') == timestamp_ns)[0].item()
        # search_time = time.time()
        # datum = self.dataloader.__getitem__(idx)
        
        # ----- load data ------
        log_dir = self.root_path / split / log_id
        sensor_dir = log_dir / "sensors"
        lidar_feather_path = sensor_dir / "lidar" / f"{timestamp_ns}.feather"
        sweep = Sweep.from_feather(lidar_feather_path=lidar_feather_path)
        
        if self.pre_load_ego_pos:
            timestamp_city_SE3_ego_dict = self.ego_pos_cache[log_id][timestamp_ns]
        else:
            timestamp_city_SE3_ego_dict = read_city_SE3_ego(log_dir=log_dir)
            timestamp_city_SE3_ego = timestamp_city_SE3_ego_dict[timestamp_ns]
        
        try:
            if self.use_map:
                if self.pre_load_map:
                    avm = self.map_cache[log_id]
                else:
                    if log_id not in self.map_cache.keys():
                        self.map_cache = {}
                        map_dir = log_dir / 'map'
                        self.map_cache[log_id] = ArgoverseStaticMap.from_map_dir(map_dir, build_raster=False)
                    
                    avm = self.map_cache[log_id]
        except:
            if self.logger != None:
                self.logger.info(f'ERROR: '
                            f'split: {split}, '
                            f'log_dir: {log_dir}, '
                            f'timestamp_ns: {timestamp_ns}, '
                            ) 
                
        if self.dataloader.cam_names:
            synchronized_imagery = self.dataloader._load_synchronized_cams(split, sensor_dir, log_id, timestamp_ns)
        
        # get_info_time = time.time()

        hd_map = None

        get_item_list = self.dataset_cfg.get('GET_ITEM_LIST', ['points'])

        input_dict = {
            'frame_id': index,
            'split': split,
            'log_id': log_id,
            'timestamp_ns': timestamp_ns,
            'timestamp_city_SE3_ego': timestamp_city_SE3_ego,
            'hd_map': hd_map,
        }

        annos_dict = self._generate_format_annos(split, log_id, timestamp_ns)

        input_dict.update(annos_dict)

        road_plane = None
        if road_plane is not None:
            input_dict['road_plane'] = road_plane

        if "points" in get_item_list:
            points =  np.concatenate([sweep.xyz, sweep.intensity.reshape((-1, 1))], axis=1)
            input_dict['points'] = points
            
        """ if index % 20 == 0:
            lidar_file = str('/moyujian/huyinghao/dataset/av2_sample/lidar/%04d.txt' % (index))
            np.savetxt(lidar_file, input_dict['points'], fmt='%.5f', delimiter=' ')
            
            annos_file = str('/moyujian/huyinghao/dataset/av2_sample/annos/%04d.txt' % (index))
            class_to_index = {class_name:i for i, class_name in enumerate(self.class_names)}
            labels = np.array([class_to_index[class_name] for class_name in input_dict['gt_names']]).reshape(-1, 1)
            annos = np.concatenate([labels, input_dict['gt_boxes'][:, 3:6], input_dict['gt_boxes'][:, 0:3], input_dict['gt_boxes'][:, 6].reshape(-1, 1)], axis=1)
            np.savetxt(annos_file, annos, fmt='%.5f', delimiter=' ') """

        if self.dataset_cfg.ONLY_DRIVABLE_AREA_VALID:
            self._remove_non_drivable_area_points_and_boxes(input_dict)
            # input_dict['points'] = self._FPS(input_dict['points'], int(input_dict['points'].shape[0] / 2))

        data_dict = self.prepare_data(data_dict=input_dict)
        data_dict['points_num'] = data_dict['points'].shape[0]
        # process_data_time = time.time()
        
        # process_data_time -= get_info_time
        # get_info_time -= start_time
        # self.logger.info(f'index: {index}, '
        #                  f'get_info_time: {get_info_time:.4f}, '
        #                  f'process_data_time: {process_data_time:.4f}, '
        #                  ) 
        return data_dict
    
    def _remove_non_drivable_area_points_and_boxes(self, data_dict):
        static_map = self.map_cache[data_dict['log_id']]
        city_SE3_ego = data_dict['timestamp_city_SE3_ego']
        points = data_dict['points']
    
        lidar_points_city = city_SE3_ego.transform_point_cloud(points[:, :3])
        drivable_areas = list(static_map.vector_drivable_areas.values())
        static_map.raster_drivable_area_layer = DrivableAreaMapLayer.from_vector_data(drivable_areas=drivable_areas)
        
        is_da_boolean_arr = static_map.get_raster_layer_points_boolean(lidar_points_city, layer_name=RasterLayerType.DRIVABLE_AREA)
        data_dict['points'] = points[is_da_boolean_arr]
        
        box_center_points = data_dict['gt_boxes']
        box_center_points_city = city_SE3_ego.transform_point_cloud(box_center_points[:, :3])
        box_center_is_da_boolean_arr = static_map.get_raster_layer_points_boolean(box_center_points_city, layer_name=RasterLayerType.DRIVABLE_AREA)
        data_dict['gt_boxes'] = box_center_points[box_center_is_da_boolean_arr]
        data_dict['gt_names'] = data_dict['gt_names'][box_center_is_da_boolean_arr]
        return data_dict
    
    """ def _FPS(self, points, npoint):
        start_t = time.time()
        batch_data = np.array([points[:, :3]])
        cuda_batch_points = torch.from_numpy(batch_data).float()
        points_idxs = farthest_point_sample(cuda_batch_points, npoint)
        points_idxs = np.array(points_idxs.to_cpu())
        points = points[points_idxs[0]]
        end_t = time.time()
        print(f'FPS Cost Time: {end_t - start_t}')
        return points """

    def get_lidar(self, log_id, timestamp_ns):
        log_dir = self.root_path / self.split / log_id
        sensor_dir = log_dir / "sensors"
        lidar_feather_path = sensor_dir / "lidar" / f"{timestamp_ns}.feather"
        sweep = Sweep.from_feather(lidar_feather_path=lidar_feather_path)
        return np.concatenate([sweep.xyz, sweep.intensity.reshape((-1, 1))], axis=1)
    
    def get_image(self, log_id, timestamp_ns):
        
        
        pass
    
    def get_label(self, log_id, timestamp_ns):
        
        
        pass

    def get_hdmap(self, log_id):
        
        pass
    
    def get_drivable_area(self, log_id):
        
        pass
    
    def get_ego_pose(self, log_id, timestamp_ns):
        
        pass
    
    def get_road_plane(self, log_id):
        
        pass
    
    def generate_prediction_dicts(self, batch_dict, pred_dicts, output_path=None):
        """
        Args:
            batch_dict:
                frame_id:
            pred_dicts: list of pred_dicts
                pred_boxes: (N, 7 or 9), Tensor
                pred_scores: (N), Tensor
                pred_labels: (N), Tensor
            output_path:

        Returns:

        """
        
        def get_template_prediction(num_samples):
            ret_dict = {
                'tx_m': np.zeros(num_samples), 'ty_m': np.zeros(num_samples), 'tz_m': np.zeros(num_samples),
                'length_m': np.zeros(num_samples), 'width_m': np.zeros(num_samples), 'height_m': np.zeros(num_samples),
                'qw': np.zeros(num_samples), 'qx': np.zeros(num_samples), 'qy': np.zeros(num_samples), 'qz': np.zeros(num_samples),
                'score': np.zeros(num_samples),
            }
            return ret_dict
    
        

        def generate_single_sample_dict(frame_id, box_dict):
            log_id, timestamp_ns = self.split_lidar_cache.iloc[frame_id].name
            
            pred_scores = box_dict['pred_scores'].cpu().numpy()
            pred_boxes = box_dict['pred_boxes'].cpu().numpy()
            pred_labels = box_dict['pred_labels'].cpu().numpy()
            
            num_obj = pred_scores.shape[0]
            
            out_result_dicts = get_template_prediction(num_obj)
            
            out_result_dicts['tx_m'] = pred_boxes[:, 0]
            out_result_dicts['ty_m'] = pred_boxes[:, 1]
            out_result_dicts['tz_m'] = pred_boxes[:, 2]
            out_result_dicts['length_m'] = pred_boxes[:, 3]
            out_result_dicts['width_m'] = pred_boxes[:, 4]
            out_result_dicts['height_m'] = pred_boxes[:, 5]
            
            yaws = pred_boxes[:, 6]
            rots = np.zeros((num_obj, 3))
            rots[:, 2] = yaws
            quat = geometry_utils.mat_to_quat(geometry_utils.xyz_to_mat(rots))
            
            out_result_dicts['qw'] = quat[:, 0]
            out_result_dicts['qx'] = quat[:, 1]
            out_result_dicts['qy'] = quat[:, 2]
            out_result_dicts['qz'] = quat[:, 3]
            out_result_dicts['score'] = pred_scores
            out_result_dicts['log_id'] = np.array([log_id for i in range(num_obj)])
            out_result_dicts['timestamp_ns'] = np.array([timestamp_ns for i in range(num_obj)])
            
            out_result_dicts['category'] = np.array([self.class_names[label - 1] for label in pred_labels])
            return out_result_dicts
        
        annos = [generate_single_sample_dict(batch_dict['frame_id'][index], box_dict) for index, box_dict in enumerate(pred_dicts)]

        # compute the whole number pf boxes
        """ num_obj = 0
        for index, box_dict in enumerate(pred_dicts):
            frame_id = batch_dict['frame_id'][index]
            num_obj += box_dict['pred_scores'].cpu().numpy().shape[0]
        
        dt_dicts = get_template_prediction(num_obj)
        dt_dicts['log_id'] = np.zeros(num_obj, dtype=int)
        dt_dicts['timestamp_ns'] = np.zeros(num_obj, dtype=int)
        dt_dicts['category'] = np.zeros(num_obj, dtype=int)
        cur_obj_idx = 0
        for index, box_dict in enumerate(pred_dicts):
            frame_id = batch_dict['frame_id'][index]
            pred_scores = box_dict['pred_scores'].cpu().numpy()
            pred_boxes = box_dict['pred_boxes'].cpu().numpy()
            pred_labels = box_dict['pred_labels'].cpu().numpy()
            
            yaws = pred_boxes[:, 6]
            rots = np.zeros((num_obj, 3))
            rots[:, 2] = yaws
            quat = geometry_utils.mat_to_quat(geometry_utils.xyz_to_mat(rots))
            
            cur_frame_obj_num = pred_scores.shape[0]
            s_index = cur_obj_idx
            e_index = s_index + cur_frame_obj_num
            
            dt_dicts['tx_m'][s_index:e_index]           = pred_boxes[:, 0]
            dt_dicts['ty_m'][s_index:e_index]           = pred_boxes[:, 1]
            dt_dicts['tz_m'][s_index:e_index]           = pred_boxes[:, 2]
            dt_dicts['length_m'][s_index:e_index]       = pred_boxes[:, 3]
            dt_dicts['width_m'][s_index:e_index]        = pred_boxes[:, 4]
            dt_dicts['height_m'][s_index:e_index]       = pred_boxes[:, 5]
            dt_dicts['qw'][s_index:e_index]             = quat[:, 0]
            dt_dicts['qx'][s_index:e_index]             = quat[:, 1]
            dt_dicts['qy'][s_index:e_index]             = quat[:, 2]
            dt_dicts['qz'][s_index:e_index]             = quat[:, 3]
            dt_dicts['score'][s_index:e_index]          = pred_scores
            dt_dicts['log_id'][s_index:e_index]         = np.array([frame_id for i in range(cur_frame_obj_num)])
            dt_dicts['timestamp_ns'][s_index:e_index]   = np.array([frame_id for i in range(cur_frame_obj_num)])
            dt_dicts['category'][s_index:e_index]       = pred_labels - 1
            
            cur_obj_idx = e_index """
            
        # dt_dicts['log_id']          = np.array([self.split_lidar_cache.iloc[value].name[0] for value in dt_dicts['log_id']])
        # dt_dicts['timestamp_ns']    = np.array([self.split_lidar_cache.iloc[value].name[1] for value in dt_dicts['timestamp_ns']])
        # dt_dicts['category']        = np.array([self.class_names[value] for value in dt_dicts['category']])
        return annos
    
    def generate_prediction_dicts_1(self, batch_dict, pred_dicts, output_path=None):
        """
        Args:
            batch_dict:
                frame_id:
            pred_dicts: list of pred_dicts
                pred_boxes: (N, 7 or 9), Tensor
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

        def generate_single_sample_dict(batch_index, box_dict):
            pred_scores = box_dict['pred_scores'].cpu().numpy()
            pred_boxes = box_dict['pred_boxes'].cpu().numpy()
            pred_labels = box_dict['pred_labels'].cpu().numpy()
            pred_dict = get_template_prediction(pred_scores.shape[0])
            if pred_scores.shape[0] == 0:
                return pred_dict

            num_obj = pred_scores.shape[0]
            pred_dict['name'] = np.array(self.class_names)[pred_labels - 1]
            pred_dict['alpha'] = -10 * np.ones(num_obj, dtype=np.float64)
            pred_dict['bbox'] = np.zeros((num_obj, 4), dtype=np.float64)
            pred_dict['dimensions'] = pred_boxes[:, 3:6]
            pred_dict['location'] = pred_boxes[:, 0:3]
            pred_dict['rotation_y'] = pred_boxes[:, 6]
            pred_dict['score'] = pred_scores
            pred_dict['boxes_lidar'] = pred_boxes

            return pred_dict

        annos = []
        for index, box_dict in enumerate(pred_dicts):
            frame_id = batch_dict['frame_id'][index]

            single_pred_dict = generate_single_sample_dict(index, box_dict)
            single_pred_dict['frame_id'] = frame_id
            annos.append(single_pred_dict)

            if output_path is not None:
                cur_det_file = output_path / ('%s.txt' % frame_id)
                with open(cur_det_file, 'w') as f:
                    bbox = single_pred_dict['bbox']
                    loc = single_pred_dict['location']
                    dims = single_pred_dict['dimensions']  # lhw -> hwl

                    for idx in range(len(bbox)):
                        print('%s -1 -1 %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f'
                              % (single_pred_dict['name'][idx], single_pred_dict['alpha'][idx],
                                 bbox[idx][0], bbox[idx][1], bbox[idx][2], bbox[idx][3],
                                 dims[idx][1], dims[idx][2], dims[idx][0], loc[idx][0],
                                 loc[idx][1], loc[idx][2], single_pred_dict['rotation_y'][idx],
                                 single_pred_dict['score'][idx]), file=f)

        return annos
    
    def evaluation1(self, det_annos,  class_names, **kwargs):
        from pcdet.datasets.kitti.kitti_object_eval_python.eval import print_str, do_eval
        def get_official_eval_result(gt_annos, dt_annos, current_classes, PR_detail_dict=None):
            overlap_0_7 = np.array([[0.7, 0.5, 0.5, 0.7,
                                    0.5, 0.7], [0.7, 0.5, 0.5, 0.7, 0.5, 0.7],
                                    [0.7, 0.5, 0.5, 0.7, 0.5, 0.7]])
            overlap_0_5 = np.array([[0.7, 0.5, 0.5, 0.7,
                                    0.5, 0.5], [0.5, 0.25, 0.25, 0.5, 0.25, 0.5],
                                    [0.5, 0.25, 0.25, 0.5, 0.25, 0.5]])
            min_overlaps = np.stack([overlap_0_7, overlap_0_5], axis=0)  # [2, 3, 5]
            # class_to_name = {
            #     0: 'REGULAR_VEHICLE',
            #     1: 'PEDESTRIAN',
            #     2: 'SIGN',
            #     3: 'LARGE_VEHICLE',
            #     4: 'BICYCLE',
            #     5: 'DOG'
            # }
            class_to_name = {i: n for i, n in enumerate(CLASSES)}
            name_to_class = {v: n for n, v in class_to_name.items()}
            if not isinstance(current_classes, (list, tuple)):
                current_classes = [current_classes]
            current_classes_int = []
            for curcls in current_classes:
                if isinstance(curcls, str):
                    current_classes_int.append(name_to_class[curcls])
                else:
                    current_classes_int.append(curcls)
            current_classes = current_classes_int
            min_overlaps = min_overlaps[:, :, current_classes]
            result = ''
            # check whether alpha is valid
            compute_aos = False
            for anno in dt_annos:
                if anno['alpha'].shape[0] != 0:
                    if anno['alpha'][0] != -10:
                        compute_aos = True
                    break
            mAPbbox, mAPbev, mAP3d, mAPaos, mAPbbox_R40, mAPbev_R40, mAP3d_R40, mAPaos_R40 = do_eval(
                gt_annos, dt_annos, current_classes, min_overlaps, compute_aos, PR_detail_dict=PR_detail_dict)

            ret_dict = {}
            for j, curcls in enumerate(current_classes):
                # mAP threshold array: [num_minoverlap, metric, class]
                # mAP result: [num_class, num_diff, num_minoverlap]
                for i in range(min_overlaps.shape[0]):
                    result += print_str(
                        (f"{class_to_name[curcls]} "
                        "AP@{:.2f}, {:.2f}, {:.2f}:".format(*min_overlaps[i, :, j])))
                    result += print_str((f"bbox AP:{mAPbbox[j, 0, i]:.4f}, "
                                        f"{mAPbbox[j, 1, i]:.4f}, "
                                        f"{mAPbbox[j, 2, i]:.4f}"))
                    result += print_str((f"bev  AP:{mAPbev[j, 0, i]:.4f}, "
                                        f"{mAPbev[j, 1, i]:.4f}, "
                                        f"{mAPbev[j, 2, i]:.4f}"))
                    result += print_str((f"3d   AP:{mAP3d[j, 0, i]:.4f}, "
                                        f"{mAP3d[j, 1, i]:.4f}, "
                                        f"{mAP3d[j, 2, i]:.4f}"))

                    if compute_aos:
                        result += print_str((f"aos  AP:{mAPaos[j, 0, i]:.2f}, "
                                            f"{mAPaos[j, 1, i]:.2f}, "
                                            f"{mAPaos[j, 2, i]:.2f}"))
                        # if i == 0:
                        # ret_dict['%s_aos/easy' % class_to_name[curcls]] = mAPaos[j, 0, 0]
                        # ret_dict['%s_aos/moderate' % class_to_name[curcls]] = mAPaos[j, 1, 0]
                        # ret_dict['%s_aos/hard' % class_to_name[curcls]] = mAPaos[j, 2, 0]

                    result += print_str(
                        (f"{class_to_name[curcls]} "
                        "AP_R40@{:.2f}, {:.2f}, {:.2f}:".format(*min_overlaps[i, :, j])))
                    result += print_str((f"bbox AP:{mAPbbox_R40[j, 0, i]:.4f}, "
                                        f"{mAPbbox_R40[j, 1, i]:.4f}, "
                                        f"{mAPbbox_R40[j, 2, i]:.4f}"))
                    result += print_str((f"bev  AP:{mAPbev_R40[j, 0, i]:.4f}, "
                                        f"{mAPbev_R40[j, 1, i]:.4f}, "
                                        f"{mAPbev_R40[j, 2, i]:.4f}"))
                    result += print_str((f"3d   AP:{mAP3d_R40[j, 0, i]:.4f}, "
                                        f"{mAP3d_R40[j, 1, i]:.4f}, "
                                        f"{mAP3d_R40[j, 2, i]:.4f}"))
                    if compute_aos:
                        result += print_str((f"aos  AP:{mAPaos_R40[j, 0, i]:.2f}, "
                                            f"{mAPaos_R40[j, 1, i]:.2f}, "
                                            f"{mAPaos_R40[j, 2, i]:.2f}"))
                        if i == 0:
                            ret_dict['%s_aos/easy_R40' % class_to_name[curcls]] = mAPaos_R40[j, 0, 0]
                            ret_dict['%s_aos/moderate_R40' % class_to_name[curcls]] = mAPaos_R40[j, 1, 0]
                            ret_dict['%s_aos/hard_R40' % class_to_name[curcls]] = mAPaos_R40[j, 2, 0]

                    if i == 0:
                        # ret_dict['%s_3d/easy' % class_to_name[curcls]] = mAP3d[j, 0, 0]
                        # ret_dict['%s_3d/moderate' % class_to_name[curcls]] = mAP3d[j, 1, 0]
                        # ret_dict['%s_3d/hard' % class_to_name[curcls]] = mAP3d[j, 2, 0]
                        # ret_dict['%s_bev/easy' % class_to_name[curcls]] = mAPbev[j, 0, 0]
                        # ret_dict['%s_bev/moderate' % class_to_name[curcls]] = mAPbev[j, 1, 0]
                        # ret_dict['%s_bev/hard' % class_to_name[curcls]] = mAPbev[j, 2, 0]
                        # ret_dict['%s_image/easy' % class_to_name[curcls]] = mAPbbox[j, 0, 0]
                        # ret_dict['%s_image/moderate' % class_to_name[curcls]] = mAPbbox[j, 1, 0]
                        # ret_dict['%s_image/hard' % class_to_name[curcls]] = mAPbbox[j, 2, 0]

                        ret_dict['%s_3d/easy_R40' % class_to_name[curcls]] = mAP3d_R40[j, 0, 0]
                        ret_dict['%s_3d/moderate_R40' % class_to_name[curcls]] = mAP3d_R40[j, 1, 0]
                        ret_dict['%s_3d/hard_R40' % class_to_name[curcls]] = mAP3d_R40[j, 2, 0]
                        ret_dict['%s_bev/easy_R40' % class_to_name[curcls]] = mAPbev_R40[j, 0, 0]
                        ret_dict['%s_bev/moderate_R40' % class_to_name[curcls]] = mAPbev_R40[j, 1, 0]
                        ret_dict['%s_bev/hard_R40' % class_to_name[curcls]] = mAPbev_R40[j, 2, 0]
                        ret_dict['%s_image/easy_R40' % class_to_name[curcls]] = mAPbbox_R40[j, 0, 0]
                        ret_dict['%s_image/moderate_R40' % class_to_name[curcls]] = mAPbbox_R40[j, 1, 0]
                        ret_dict['%s_image/hard_R40' % class_to_name[curcls]] = mAPbbox_R40[j, 2, 0]

            return result, ret_dict
        
        eval_det_annos = copy.deepcopy(det_annos)
        
        eval_gt_annos = []
        for index in range(self.num_sweeps):
            log_id, timestamp_ns = self.split_lidar_cache.iloc[index].name
            annos_dict = self._generate_format_annos(self.split, log_id, timestamp_ns)
            num_obj = len(annos_dict['gt_names'])
            
            annotations = {}
            annotations['frame_id'] = index
            annotations['split'] = self.split
            annotations['log_id'] = log_id
            annotations['timestamp_ns'] = timestamp_ns
            annotations['name'] = annos_dict['gt_names']
            annotations['truncated'] = np.zeros(num_obj, dtype=np.float64)
            annotations['occluded'] = np.zeros(num_obj, dtype=np.float64)
            annotations['alpha'] = -10 * np.ones(num_obj, dtype=np.float64)
            annotations['bbox'] = np.zeros((num_obj, 4), dtype=np.float64)
            annotations['dimensions'] = annos_dict['gt_boxes'][:, 3:6]
            annotations['location'] = annos_dict['gt_boxes'][:, :3]
            annotations['rotation_y'] = annos_dict['gt_boxes'][:, 6]
            
            eval_gt_annos.append(annotations)
                
        # eval_gt_annos = [copy.deepcopy(info['annos']) for info in self.kitti_infos]
        ap_result_str, ap_dict = get_official_eval_result(eval_gt_annos, eval_det_annos, self.class_names)

        return ap_result_str, ap_dict

    def generate_av2_detection_dicts(self, pred_dicts, output_path=None):
        def get_template_prediction(num_samples):
            ret_dict = {
                'tx_m': np.zeros(num_samples), 'ty_m': np.zeros(num_samples), 'tz_m': np.zeros(num_samples),
                'length_m': np.zeros(num_samples), 'width_m': np.zeros(num_samples), 'height_m': np.zeros(num_samples),
                'qw': np.zeros(num_samples), 'qx': np.zeros(num_samples), 'qy': np.zeros(num_samples), 'qz': np.zeros(num_samples),
                'score': np.zeros(num_samples),
            }
            return ret_dict
        
        num_obj = 0
        for annos in pred_dicts:
            num_obj += annos['score'].shape[0]
        dt_dicts = get_template_prediction(num_obj)
        dt_dicts['log_id']          = []
        dt_dicts['timestamp_ns']    = np.zeros(num_obj, dtype=int)
        dt_dicts['category']        = []
        cur_obj_idx = 0
        for annos in track(pred_dicts, "Change data form of pred result..."):
            cur_frame_obj_num = annos['score'].shape[0]
            s_index = cur_obj_idx
            e_index = s_index + cur_frame_obj_num
            for key in dt_dicts.keys():
                if key not in ['log_id', 'category']:
                    dt_dicts[key][s_index:e_index] = annos[key]
            cur_obj_idx = e_index
            dt_dicts['log_id'] += annos['log_id'].tolist()
            dt_dicts['category'] += annos['category'].tolist()
        
        dt_dicts['log_id'] = np.array(dt_dicts['log_id'])
        dt_dicts['category'] = np.array(dt_dicts['category'])
    
        """ out_dicts = pd.DataFrame()
        dt_dicts = get_template_prediction(0)
        dt_dicts['log_id'] = np.array([], dtype=str)
        dt_dicts['timestamp_ns'] = np.zeros(0, dtype=int)
        dt_dicts['category'] = np.array([], dtype=str)
        
        for box_dict in track(pred_dicts, "Turning pred_dicts to av2 format..."):
            frame_id = box_dict['frame_id']
            log_id, timestamp_ns = self.split_lidar_cache.iloc[frame_id].name
            
            pred_scores = box_dict['score']
            pred_boxes = box_dict['boxes_lidar']
            num_obj = pred_scores.shape[0]
            
            dt_dicts['tx_m'] = np.append(dt_dicts['tx_m'], pred_boxes[:, 0])
            dt_dicts['ty_m'] = np.append(dt_dicts['ty_m'], pred_boxes[:, 1])
            dt_dicts['tz_m'] = np.append(dt_dicts['tz_m'], pred_boxes[:, 2])
            dt_dicts['length_m'] = np.append(dt_dicts['length_m'], pred_boxes[:, 3])
            dt_dicts['width_m'] = np.append(dt_dicts['width_m'], pred_boxes[:, 4])
            dt_dicts['height_m'] = np.append(dt_dicts['height_m'], pred_boxes[:, 5])
            
            yaws = pred_boxes[:, 6]
            rots = np.zeros((num_obj, 3))
            rots[:, 2] = yaws
            quat = geometry_utils.mat_to_quat(geometry_utils.xyz_to_mat(rots))
            
            dt_dicts['qw'] = np.append(dt_dicts['qw'], quat[:, 0])
            dt_dicts['qx'] = np.append(dt_dicts['qx'], quat[:, 1])
            dt_dicts['qy'] = np.append(dt_dicts['qy'], quat[:, 2])
            dt_dicts['qz'] = np.append(dt_dicts['qz'], quat[:, 3])
            dt_dicts['score'] = np.append(dt_dicts['score'], pred_scores)
            dt_dicts['log_id'] = np.append(dt_dicts['log_id'], np.array([log_id for i in range(num_obj)]))
            dt_dicts['timestamp_ns'] = np.append(dt_dicts['timestamp_ns'], np.array([timestamp_ns for i in range(num_obj)]))
            
            dt_dicts['category'] = np.append(dt_dicts['category'], box_dict['name']) """
            
        
        
        """ dt_dicts = pd.DataFrame()
        for box_dict in track(pred_dicts, "Turning pred_dicts to av2 format..."):
            frame_id = box_dict['frame_id']
            log_id, timestamp_ns = self.split_lidar_cache.iloc[frame_id].name
            
            pred_scores = box_dict['score']
            pred_boxes = box_dict['boxes_lidar']
            num_obj = pred_scores.shape[0]
            
            det_dicts = get_template_prediction(num_obj)
            det_dicts['tx_m'] = pred_boxes[:, 0]
            det_dicts['ty_m'] = pred_boxes[:, 1]
            det_dicts['tz_m'] = pred_boxes[:, 2]
            det_dicts['length_m'] = pred_boxes[:, 3]
            det_dicts['width_m'] = pred_boxes[:, 4]
            det_dicts['height_m'] = pred_boxes[:, 5]
            
            yaws = pred_boxes[:, 6]
            rots = np.zeros((num_obj, 3))
            rots[:, 2] = yaws
            quat = geometry_utils.mat_to_quat(geometry_utils.xyz_to_mat(rots))
            
            det_dicts['qw'] = quat[:, 0]
            det_dicts['qx'] = quat[:, 1]
            det_dicts['qy'] = quat[:, 2]
            det_dicts['qz'] = quat[:, 3]
            det_dicts['score'] = pred_scores
            det_dicts['log_id'] = np.array([log_id for i in range(num_obj)])
            det_dicts['timestamp_ns'] = np.array([timestamp_ns for i in range(num_obj)])
            
            det_dicts['category'] = box_dict['name']
            
            dt_dicts = pd.concat([dt_dicts, pd.DataFrame(det_dicts)], axis=0, ignore_index=True) """
        
        
        return pd.DataFrame(dt_dicts)    
    
    def generate_av2_gt_dicts(self, output_path=None):
        def get_template_dict(num_samples):
            ret_dict = {
                'tx_m': np.zeros(num_samples), 'ty_m': np.zeros(num_samples), 'tz_m': np.zeros(num_samples),
                'length_m': np.zeros(num_samples), 'width_m': np.zeros(num_samples), 'height_m': np.zeros(num_samples),
                'qw': np.zeros(num_samples), 'qx': np.zeros(num_samples), 'qy': np.zeros(num_samples), 'qz': np.zeros(num_samples),
                'num_interior_pts': np.zeros(num_samples),
            }
            return ret_dict
        
        num_obj = 0
        for key, value in self.annos_cache.items():
            num_obj += value.timestamp_ns.to_numpy().shape[0]
            
        gt_dicts = get_template_dict(num_obj)
        gt_dicts['log_id'] = []
        gt_dicts['timestamp_ns'] = np.zeros(num_obj, dtype=int)
        gt_dicts['category'] = []
        cur_obj_idx = 0
        for log_id, log_data in track(self.annos_cache.items(), "Loading GT annos av2 format..."):
            timestamp_ns = log_data.timestamp_ns.to_numpy()
            cur_frame_obj_num = timestamp_ns.shape[0]

            s_index = cur_obj_idx
            e_index = s_index + cur_frame_obj_num
            for key in gt_dicts.keys():
                if key not in ['log_id', 'category']:
                    gt_dicts[key][s_index:e_index] = log_data.loc[:, key].to_numpy()
            gt_dicts['log_id'] += [log_id for i in range(cur_frame_obj_num)]
            gt_dicts['category'] += list(log_data.loc[:, 'category'])
            cur_obj_idx = e_index
        
        gt_dicts['log_id'] = np.array(gt_dicts['log_id'])
        gt_dicts['category'] = np.array(gt_dicts['category'])
        
        # if self.pre_load_annos:
        #     gt_dicts['category'], mask = self._convert_classes_new(gt_dicts['category'])
            
        #     for key in gt_dicts.keys():
        #         gt_dicts[key] = gt_dicts[key][mask]
        
        """ gt_dicts = get_template_dict(0)
        gt_dicts['log_id'] = np.array([], dtype=str)
        gt_dicts['timestamp_ns'] = np.zeros(0, dtype=int)
        gt_dicts['category'] = np.array([], dtype=str)
        
        for key, value in track(self.annos_cache.items(), "Loading GT annos av2 format..."):
            log_id = key
            log_data = value
            
            timestamp_ns = log_data.timestamp_ns.to_numpy()
            quat = log_data.loc[:, ["qw", "qx", "qy", "qz"]].to_numpy()
            translation_m = log_data.loc[:, ["tx_m", "ty_m", "tz_m"]].to_numpy()
            length_m = log_data.loc[:, "length_m"].to_numpy()
            width_m = log_data.loc[:, "width_m"].to_numpy()
            height_m = log_data.loc[:, "height_m"].to_numpy()
            category = log_data.loc[:, "category"].to_numpy()
            num_interior_pts = log_data.loc[:, "num_interior_pts"].to_numpy()
            gt_names = self._convert_classes(category)
            
            num_obj = timestamp_ns.shape[0]
            
            gt_dicts['tx_m'] = np.append(gt_dicts['tx_m'], translation_m[:, 0])
            gt_dicts['ty_m'] = np.append(gt_dicts['ty_m'], translation_m[:, 1])
            gt_dicts['tz_m'] = np.append(gt_dicts['tz_m'], translation_m[:, 2])
            gt_dicts['length_m'] = np.append(gt_dicts['length_m'], length_m)
            gt_dicts['width_m'] = np.append(gt_dicts['width_m'], width_m)
            gt_dicts['height_m'] = np.append(gt_dicts['height_m'], height_m)
            gt_dicts['qw'] = np.append(gt_dicts['qw'], quat[:, 0])
            gt_dicts['qx'] = np.append(gt_dicts['qx'], quat[:, 1])
            gt_dicts['qy'] = np.append(gt_dicts['qy'], quat[:, 2])
            gt_dicts['qz'] = np.append(gt_dicts['qz'], quat[:, 3])
            gt_dicts['num_interior_pts'] = np.append(gt_dicts['num_interior_pts'], num_interior_pts)
            gt_dicts['log_id'] = np.append(gt_dicts['log_id'], np.array([log_id for i in range(num_obj)]))
            gt_dicts['timestamp_ns'] = np.append(gt_dicts['timestamp_ns'], timestamp_ns)
            gt_dicts['category'] = np.append(gt_dicts['category'], gt_names) """

        
        """ gt_dicts = pd.DataFrame()
        for key, value in track(self.annos_cache.items(), "Loading GT annos av2 format..."):
            log_id = key
            log_data = value
            
            timestamp_ns = log_data.timestamp_ns.to_numpy()
            quat = log_data.loc[:, ["qw", "qx", "qy", "qz"]].to_numpy()
            translation_m = log_data.loc[:, ["tx_m", "ty_m", "tz_m"]].to_numpy()
            length_m = log_data.loc[:, "length_m"].to_numpy()
            width_m = log_data.loc[:, "width_m"].to_numpy()
            height_m = log_data.loc[:, "height_m"].to_numpy()
            category = log_data.loc[:, "category"].to_numpy()
            num_interior_pts = log_data.loc[:, "num_interior_pts"].to_numpy()
            gt_names = self._convert_classes(category)
            
            num_obj = timestamp_ns.shape[0]
            gt_dict = get_template_dict(num_obj)
            gt_dict['tx_m'] = translation_m[:, 0]
            gt_dict['ty_m'] = translation_m[:, 1]
            gt_dict['tz_m'] = translation_m[:, 2]
            gt_dict['length_m'] = length_m
            gt_dict['width_m'] = width_m
            gt_dict['height_m'] = height_m
            gt_dict['qw'] = quat[:, 0]
            gt_dict['qx'] = quat[:, 1]
            gt_dict['qy'] = quat[:, 2]
            gt_dict['qz'] = quat[:, 3]
            gt_dict['num_interior_pts'] = num_interior_pts
            gt_dict['log_id'] = np.array([log_id for i in range(num_obj)])
            gt_dict['timestamp_ns'] = timestamp_ns
            gt_dict['category'] = gt_names
            
            gt_dicts = pd.concat([gt_dicts, pd.DataFrame(gt_dict)], axis=0, ignore_index=True) """

        return pd.DataFrame(gt_dicts)
    
    def evaluate(self,
                 pred_dicts,
                 ):
        """Evaluation in KITTI protocol.

        Args:
            results (list[dict]): Testing results of the dataset.
            metric (str | list[str]): Metrics to be evaluated.
                Default: 'waymo'. Another supported metric is 'kitti'.
            logger (logging.Logger | str | None): Logger used for printing
                related information during evaluation. Default: None.
            pklfile_prefix (str | None): The prefix of pkl files. It includes
                the file path and the prefix of filename, e.g., "a/b/prefix".
                If not specified, a temp file will be created. Default: None.
            submission_prefix (str | None): The prefix of submission datas.
                If not specified, the submission data will not be generated.
            show (bool): Whether to visualize.
                Default: False.
            out_dir (str): Path to save the visualization results.
                Default: None.
            pipeline (list[dict], optional): raw data loading for showing.
                Default: None.

        Returns:
            dict[str: float]: results of each evaluation metric
        """
        from av2.evaluation.detection.constants import CompetitionCategories
        from av2.evaluation.detection.utils import DetectionCfg
        from av2.evaluation.detection.eval import evaluate
        
        self.logger.info("Starting to turn the pred_dicts to av2 format...")
        start_time = time.time()
        dt_dframe = self.generate_av2_detection_dicts(pred_dicts)
        
        last_time = time.time() - start_time
        self.logger.info(f"End, time cost: {last_time}")
        if self.dataset_cfg.SAVE_VAL_RESULT_AV2_FORMAT:
            dt_dframe.to_json('val_det_av2_result.json')
        
        
        self.logger.info("Starting to load GT annos of av2 format...")
        start_time = time.time()
        gt_dframe = self.generate_av2_gt_dicts()
        last_time = time.time() - start_time
        self.logger.info(f"End, time cost: {last_time}")
        
        dataset_dir = Path(self.root_path) / self.split
        classes = tuple(CLASSES)
        competition_cfg = DetectionCfg(
                dataset_dir=dataset_dir,
                categories=classes,
            )  # Defaults to competition parameters.

        # Evaluate using Argoverse detection API.
        self.logger.info("Starting to evaluate the result...")
        start_time = time.time()
        eval_dts, eval_gts, metrics = evaluate(
            dt_dframe.reset_index(), gt_dframe.reset_index(), competition_cfg, logger = self.logger
        )
        last_time = time.time() - start_time
        self.logger.info(f"End, time cost: {last_time}")
        
        valid_categories = self.class_names  + ["AVERAGE_METRICS"]
        
        valid_metrics_data = metrics.loc[valid_categories].to_numpy()
        result_dicts = {}
        columns = metrics.loc[valid_categories].columns.to_numpy()
        for j, cur_column in enumerate(columns):
            for i, category in enumerate(valid_categories):
                result_dicts[f"{category}-{cur_column}"] = valid_metrics_data[i][j]
        return metrics.loc[valid_categories], result_dicts
    
    def evaluation(self, det_annos,  class_names, **kwargs):
        return self.evaluate(det_annos)
    '''
    
