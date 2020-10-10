import copy
import json
import os
from pathlib import Path
import pickle
import shutil
import time
import re 
import fire
import numpy as np
import torch
from google.protobuf import text_format
import second.data.kitti_common as kitti
import torchplus
from second.builder import target_assigner_builder, voxel_builder
from second.core import box_np_ops
from second.data.preprocess import merge_second_batch, merge_second_batch_multigpu
from second.protos import pipeline_pb2
from second.pytorch.builder import (box_coder_builder, input_reader_builder,
                                    lr_scheduler_builder, optimizer_builder,
                                    second_builder)
from second.utils.log_tool import SimpleModelLog
from second.utils.progress_bar import ProgressBar
import psutil
import second.data.tiev_dataset

anchors=None
class SECOND():
    def __init__(self,config_path='/home/autolab/tiev2019/src/modules/second.pytorch/second/configs/all.tiev.config',
                    model_dir='/home/autolab/tiev2019/src/modules/second.pytorch/second/model_dir_apolo_all_4.0/',measure_time=False):
        #/home/autolab/tianxuebo/second.pytorch/second/model_dir_mix_1_1.0/
        self.model_dir = str(Path(model_dir).resolve())
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if isinstance(config_path, str):
            config = pipeline_pb2.TrainEvalPipelineConfig()
            with open(config_path, "r") as f:
                proto_str = f.read()
                text_format.Merge(proto_str, config)
        else:
            config = config_path
        self.input_cfg = config.eval_input_reader
        self.model_cfg = config.model.second
        self.train_cfg = config.train_config
        self.net = build_network(self.model_cfg).to(self.device)
        self.target_assigner = self.net.target_assigner
        self.voxel_generator = self.net.voxel_generator
        assert model_dir is not None
        torchplus.train.try_restore_latest_checkpoints(self.model_dir, [self.net])
        if self.train_cfg.enable_mixed_precision:
            self.float_dtype = torch.float16
        else:
            self.float_dtype = torch.float32
        self.net.eval()
        self.tiev_datacls = input_reader_builder.build2(
            self.input_cfg,
            self.model_cfg,
            training=False,
            voxel_generator=self.voxel_generator,
            target_assigner=self.target_assigner)

    def detect(self, examplebin):
        example=self.tiev_datacls.getvoxel(examplebin)
        new_clom=np.zeros((example['coordinates'].shape[0],1),dtype = np.int32)
        example['coordinates']=np.append(new_clom, example['coordinates'],axis=1)
        example['num_voxels']=[example['num_voxels']]
        example['metrics']=[example['metrics']]
        example['anchors']=[example['anchors']]
        example = example_convert_to_torch(example, self.float_dtype, self.device)
        with torch.no_grad():
            result=self.net(example)
        return result

def example_convert_to_torch(example, dtype=torch.float32,device=None) -> dict:
    device = device or torch.device("cuda:0")
    example_torch = {}
    float_names = [
        "voxels", "reg_targets", "reg_weights", "bev_map", "importance"
    ]
    for k, v in example.items():
        if k in float_names:
            example_torch[k] = torch.tensor(
                v, dtype=torch.float32, device=device).to(dtype)
        elif k in [ "anchors"]:
            global anchors
            if anchors is None:
                anchors = example_torch[k] = torch.tensor(v, dtype=torch.float32, device=device).to(dtype)
            else:
                 example_torch[k]=anchors
        elif k in ["coordinates", "labels", "num_points"]:
            example_torch[k] = torch.tensor(
                v, dtype=torch.int32, device=device)
        elif k in ["anchors_mask"]:
            example_torch[k] = torch.tensor(
                v, dtype=torch.uint8, device=device)
        elif k == "num_voxels":
            example_torch[k] = torch.tensor(v)
        else:
            example_torch[k] = v
    return example_torch

def build_network(model_cfg, measure_time=False):
    voxel_generator = voxel_builder.build(model_cfg.voxel_generator)
    bv_range = voxel_generator.point_cloud_range[[0, 1, 3, 4]]
    box_coder = box_coder_builder.build(model_cfg.box_coder)
    target_assigner_cfg = model_cfg.target_assigner
    target_assigner = target_assigner_builder.build(target_assigner_cfg,
                                                    bv_range, box_coder)
    box_coder.custom_ndim = target_assigner._anchor_generators[0].custom_ndim
    net = second_builder.build(
        model_cfg, voxel_generator, target_assigner, measure_time=measure_time)
    return net
       
second=SECOND()

ef get_boxes(bindata):
       print("hhhhh")
def get_boxes(bindata):
     print("hhhhh")
    #bindata=np.array([i for i in bindata])
    print("bindata shape:",bindata.shape,"\n",bindata[0])
    result=second.detect(bindata)
    lab=result[0].get('label_preds').cpu().numpy().reshape(-1,1)
    result=np.append(lab, result[0].get('box3d_lidar').cpu().numpy(),axis=1)
    print("result shape",result.shape)
    return result

f __name__ == '__main__':
   fire.Fire()
