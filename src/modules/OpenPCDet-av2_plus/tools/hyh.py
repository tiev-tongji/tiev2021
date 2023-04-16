from pathlib import Path
from typing import Dict, Final, List, Optional, Tuple, Union
from rich.progress import track
from av2.datasets.sensor.utils import convert_path_to_named_record
from av2.datasets.sensor.sensor_dataloader import SensorDataloader
from av2.utils.io import TimestampedCitySE3EgoPoses, read_city_SE3_ego, read_feather, read_img
from pyarrow import feather
from rich.progress import track

import numpy as np
import pandas as pd
import sys
import os

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

LIDAR_PATTERN: Final[str] = "**/sensors/lidar/*.feather"
CAMERA_PATTERN: Final[str] = "**/sensors/cameras/*/*.jpg"

data_root = Path('/moyujian/moyujian/Data/av2/sensor')
# data_root = Path('/heyufei1/huyinghao/dataset/samples/av2')
split = 'train'

def populate_lidar_records(dataset_dir) -> pd.DataFrame:
    """Obtain (log_id, sensor_name, timestamp_ns) 3-tuples for all LiDAR sweeps in the dataset.

    Returns:
        DataFrame of shape (N,3) with `log_id`, `sensor_name`, and `timestamp_ns` columns.
            N is the number of sweeps for all logs in the dataset, and the `sensor_name` column
            should be populated with `lidar` in every entry.
    """
    lidar_paths = sorted(dataset_dir.glob(LIDAR_PATTERN), key=lambda x: int(x.stem))
    lidar_record_list = [
        convert_path_to_named_record(x) for x in track(lidar_paths, description="Loading lidar records ...")
    ]

    # Concatenate into single dataframe (list-of-dicts to DataFrame).
    lidar_records = pd.DataFrame(lidar_record_list)
    return lidar_records

def check_dataset_files():
    lidar_records = populate_lidar_records(data_root)
    lidar_records.set_index(["split", "log_id", "sensor_name", "timestamp_ns"], inplace=True)
    lidar_records.sort_index(inplace=True)
    
    log_id = lidar_records.xs(key=split, level=0).index.get_level_values('log_id')
    log_id = sorted(list(set(log_id)))
    
    for i in range(len(log_id)):
        log_dir = data_root / split / log_id[i]
        calib_dir = log_dir / 'calibration'
        map_dir = log_dir / 'map'
        lidar_dir = log_dir / 'sensors' / 'lidar'
        ring_front_center_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_front_center'
        ring_front_left_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_front_left'
        ring_front_right_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_front_right'
        ring_rear_left_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_rear_left'
        ring_rear_right_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_rear_right'
        ring_side_left_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_side_left'
        ring_side_right_camera_dir = log_dir / 'sensors' / 'cameras' / 'ring_side_right'
        stereo_front_left_camera_dir = log_dir / 'sensors' / 'cameras' / 'stereo_front_left'
        stereo_front_right_camera_dir = log_dir / 'sensors' / 'cameras' / 'stereo_front_right'
        
        
        log_files_num = len(os.listdir(log_dir))
        calib_files_num = len(os.listdir(calib_dir))
        map_files_num = len(os.listdir(map_dir))
        lidar_files_num = len(os.listdir(lidar_dir))
        ring_front_center_camera_files_num = len(os.listdir(ring_front_center_camera_dir))
        ring_front_left_camera_files_num = len(os.listdir(ring_front_left_camera_dir))
        ring_front_right_camera_files_num = len(os.listdir(ring_front_right_camera_dir))
        ring_rear_left_camera_files_num = len(os.listdir(ring_rear_left_camera_dir))
        ring_rear_right_camera_files_num = len(os.listdir(ring_rear_right_camera_dir))
        ring_side_left_camera_files_num = len(os.listdir(ring_side_left_camera_dir))
        ring_side_right_camera_files_num = len(os.listdir(ring_side_right_camera_dir))
        stereo_front_left_camera_files_num = len(os.listdir(stereo_front_left_camera_dir))
        stereo_front_right_camera_files_num = len(os.listdir(stereo_front_right_camera_dir))
        print(" %s   [log, calib, map, lidar, [cameras]]:   %d  %d  %d  %d [%d %d %d %d %d %d %d %d %d]" % (
                                                                    log_id[i], 
                                                                    log_files_num,
                                                                    calib_files_num,
                                                                    map_files_num,
                                                                    lidar_files_num,
                                                                    ring_front_center_camera_files_num,
                                                                    ring_front_left_camera_files_num,
                                                                    ring_front_right_camera_files_num,
                                                                    ring_rear_left_camera_files_num,
                                                                    ring_rear_right_camera_files_num,
                                                                    ring_side_left_camera_files_num,
                                                                    ring_side_right_camera_files_num,
                                                                    stereo_front_left_camera_files_num,
                                                                    stereo_front_right_camera_files_num
                                                                    ))
        
def split_dataset_annos():
    split = 'train'
    lidar_records = populate_lidar_records(data_root)
    lidar_records.set_index(["split", "log_id", "sensor_name", "timestamp_ns"], inplace=True)
    lidar_records.sort_index(inplace=True)
    
    log_id = lidar_records.xs(key=split, level=0).index.get_level_values('log_id')
    log_id = sorted(list(set(log_id)))
    
    for cur_log_id in track(log_id, 'Spliting annoations of av2 dataset'):
        log_dir = data_root / split / cur_log_id
        annos = read_feather(log_dir / 'annotations.feather')
        timestamp_ns_list = lidar_records.xs((split, cur_log_id, 'lidar', slice(None))).index.to_numpy()
        
        annos_dir = log_dir / "annotations"
        annos_dir.mkdir(parents=True, exist_ok=True)
        for timestamp_ns in timestamp_ns_list:
            cur_annos = pd.DataFrame(annos[annos.timestamp_ns == timestamp_ns])
            cur_annos = cur_annos.reset_index(drop=True)
            for i, cur_class in enumerate(cur_annos['category']):
                if cur_class in Groups[0]:
                    cur_annos.loc[i, 'category'] = CLASSES[0]
                elif cur_class in Groups[1]:
                    cur_annos.loc[i, 'category'] = CLASSES[1]
                elif cur_class in Groups[2]:
                    cur_annos.loc[i, 'category'] = CLASSES[2]
                elif cur_class in Groups[3]:
                    cur_annos.loc[i, 'category'] = CLASSES[3]
                else:
                    cur_annos.loc[i, 'category'] = "Uncare"
                
            cur_annos = pd.DataFrame(cur_annos[cur_annos.category != "Uncare"])
            feather.write_feather(cur_annos, annos_dir / f"{timestamp_ns}.feather")
            
            # check_data = read_feather(annos_dir / f"{timestamp_ns}.feather")
            # print(1)
    
    
    
    
if __name__ == '__main__':
    split_dataset_annos()
    pass
    #3153b5b3-d381-3664-8f82-1d3c5ca841d2
    #31f

