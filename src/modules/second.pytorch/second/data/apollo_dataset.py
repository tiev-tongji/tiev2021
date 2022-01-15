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
import second.data.custome_eval as custome_eval
@register_dataset
class ApolloDataset(Dataset):
    NumPointFeatures = 3
    def __init__(self,
                 root_path,
                 info_path,
                 class_names=None,
                 prep_func=None,
                 num_point_features=None):
        assert info_path is not None
        with open(info_path, 'rb') as f:
            infos = pickle.load(f)
        self._root_path = Path(root_path)
        self._apollo_infos = infos
        self._class_names = class_names
        self._prep_func = prep_func
        print("remain number of infos:", len(self._apollo_infos))

    def __len__(self):
        return len(self._apollo_infos)

    def __getitem__(self, idx):
        input_dict = self.get_sensor_data(idx)
        example = self._prep_func(input_dict=input_dict)
        example["metadata"] = {}
        if "image_idx" in input_dict["metadata"]:
            example["metadata"] = input_dict["metadata"]
        if "anchors_mask" in example:
            example["anchors_mask"] = example["anchors_mask"].astype(np.uint8)
        return example

    def get_sensor_data(self, query):
        read_image = False
        idx = query
        if isinstance(query, dict):
            read_image = "cam" in query
            assert "lidar" in query
            idx = query["lidar"]["idx"]
        info = self._apollo_infos[idx]
        res = {
            "lidar": {
                "type": "lidar",
                "points": None,
            },
            "metadata": {
                "image_idx": info["image"]["image_idx"],
            },
            "calib": None,
            "cam": {}
        }

        pc_info = info["point_cloud"]
        velo_path = Path(self._root_path) / pc_info['velodyne_path']
        points = np.fromfile(
            str(velo_path), dtype=np.float32,
            count=-1).reshape([-1, self.NumPointFeatures])
        res["lidar"]["points"] = points

        if 'annos' in info:
            annos = info['annos']
            # we need other objects to avoid collision when sample
            annos = remove_dontcare(annos)
            locs = annos["location"]
            dims = annos["dimensions"]
            rots = annos["rotation_y"]
            gt_names = annos["name"]
            gt_boxes = np.concatenate([locs, dims, rots[..., np.newaxis]],
                                      axis=1).astype(np.float32)
            res["lidar"]["annotations"] = {
                'boxes': gt_boxes,
                'names': gt_names,
            }
            res["cam"]["annotations"] = {
                'boxes': annos["bbox"],
                'names': gt_names,
            }
        return res
        
    def convert_detection_to_apollo_annos(self, detection):
        class_names = self._class_names
        annos = []
        for i in range(len(detection)):
            det = detection[i]
            final_box_preds = det["box3d_lidar"].detach().cpu().numpy()
            label_preds = det["label_preds"].detach().cpu().numpy()
            scores = det["scores"].detach().cpu().numpy()
            if final_box_preds.shape[0] != 0:
                bbox = np.full((final_box_preds.shape[0], 4), 50)
            anno = kitti.get_start_result_anno()
            num_example = 0
            box3d_lidar = final_box_preds
            for j in range(box3d_lidar.shape[0]):
                anno["bbox"].append(bbox[j])
                anno["alpha"].append(-10)
                anno["dimensions"].append(box3d_lidar[j, 3:6])
                anno["location"].append(box3d_lidar[j, :3])
                anno["rotation_y"].append(box3d_lidar[j, 6])
                anno["name"].append(class_names[int(label_preds[j])])
                anno["truncated"].append(0.0)
                anno["occluded"].append(0)
                anno["score"].append(scores[j])
                num_example += 1
            if num_example != 0:
                anno = {n: np.stack(v) for n, v in anno.items()}
                annos.append(anno)
            else:
                annos.append(kitti.empty_result_anno())
            num_example = annos[-1]["name"].shape[0]
            annos[-1]["metadata"] = det["metadata"]
        return annos

    def evaluation(self, detections, output_dir):
        labels_path=self._root_path / 'training/label_2/'
        dt_annos = self.convert_detection_to_apollo_annos(detections)
        custome_eval.get_det_results(dt_annos,labels_path,'apollo')
        return None
    

def remove_dontcare(image_anno):
    img_filtered_annotations = {}
    relevant_annotation_indices = [
        i for i, x in enumerate(image_anno['name']) if x != "DontCare"
    ]
    for key in image_anno.keys():
        img_filtered_annotations[key] = (
            image_anno[key][relevant_annotation_indices])
    return img_filtered_annotations

def _read_imageset_file(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    return [int(line) for line in lines]

#create infos files
def create_apollo_info_file(data_path=None, save_path=None, relative_path=True):
    #read file
    data_path="/home/autolab/tianxuebo/second.pytorch/second/DATASET/WAYMO_WITH_INTEN_DATASET_ROOT/"
    train_img_ids = _read_imageset_file("/home/autolab/tianxuebo/second.pytorch/second/data/ImageSets/waymo2_train.txt")
    val_img_ids = _read_imageset_file("/home/autolab/tianxuebo/second.pytorch/second/data/ImageSets/waymo2_val.txt")
    print("Generate info. this may take several minutes.")
    save_path = data_path
    #prepare train infos
    apollo_infos_train = get_apollo_image_info(
        data_path,
        training=True,
        velodyne=True,
        calib=True,
        image_ids=train_img_ids,
        relative_path=relative_path)
    filename = save_path+ '/apollo_infos_train.pkl'
    print(f"Apollo info train file is saved to {filename}")
    with open(filename, 'wb') as f:
        pickle.dump(apollo_infos_train, f)
    #prepare val infos
    apollo_infos_val = get_apollo_image_info(
        data_path,
        training=True,
        velodyne=True,
        calib=True,
        image_ids=val_img_ids,
        relative_path=relative_path)
    filename = save_path +'/apollo_infos_val.pkl'
    print(f"Apollo info val file is saved to {filename}")
    with open(filename, 'wb') as f:
        pickle.dump(apollo_infos_val, f)
    #prepare trainval infos
    filename = save_path + '/apollo_infos_trainval.pkl'
    print(f"Apollo info trainval file is saved to {filename}")
    with open(filename, 'wb') as f:
        pickle.dump(apollo_infos_train + apollo_infos_val, f)

#get the infos of a list    
def get_apollo_image_info(data_path,
                         training=True,
                         label_info=True,
                         velodyne=False,
                         calib=False,
                         image_ids=9,
                         extend_matrix=True,
                         num_worker=8,
                         relative_path=True,):
    
    if not isinstance(image_ids, list):
        image_ids = list(range(image_ids))
    infos = []
    for ids in image_ids:
        info=get_one_info(ids,data_path)
        infos.append(info)
    return list(infos)    

#get info of one id
def get_one_info(idx,data_path):
    info = {}
    idx_6="%06d" % idx
    pc_info = {'num_features': 3}
    pc_info['velodyne_path'] = 'training/velodyne/'+idx_6+'.bin'
    img_info={'image_idx': idx}
    calib_info = {}
    annotations =  get_label_anno(data_path+'training/label_2/'+idx_6+'.txt')
    info["point_cloud"] = pc_info
    info['image']=img_info
    if annotations is not None:
        info['annos'] = annotations
    return info  
  
#get label info
def get_label_anno(label_path):
    annotations = {}
    annotations.update({
        'name': [],
        'truncated': [],
        'occluded': [],
        'alpha': [],
        'bbox': [],
        'dimensions': [],
        'location': [],
        'rotation_y': []
    })
    with open(label_path, 'r') as f:
        lines = f.readlines()
    # if len(lines) == 0 or len(lines[0]) < 15:
    #     content = []
    # else:
    content = [line.strip().split(' ') for line in lines]
    num_objects = len([x[0] for x in content if x[0] != 'DontCare'])
    annotations['name'] = np.array([x[0] for x in content])
    num_gt = len(annotations['name'])
    annotations['truncated'] = np.array([float(x[1]) for x in content])
    annotations['occluded'] = np.array([int(x[2]) for x in content])
    annotations['alpha'] = np.array([float(x[3]) for x in content])
    annotations['bbox'] = np.array(
        [[float(info) for info in x[4:8]] for x in content]).reshape(-1, 4)
    # dimensions will convert hwl format to standard lhw(camera) format.
    annotations['dimensions'] = np.array(
        [[float(info) for info in x[8:11]] for x in content]).reshape(
            -1, 3)[:, [2, 0, 1]]
    annotations['location'] = np.array(
        [[float(info) for info in x[11:14]] for x in content]).reshape(-1, 3)[:,[2,0,1]]
    annotations['rotation_y'] = np.array(
        [float(x[14]) for x in content]).reshape(-1)
    if len(content) != 0 and len(content[0]) == 16:  # have score
        annotations['score'] = np.array([float(x[15]) for x in content])
    else:
        annotations['score'] = np.zeros((annotations['bbox'].shape[0], ))
    index = list(range(num_objects)) + [-1] * (num_gt - num_objects)
    annotations['index'] = np.array(index, dtype=np.int32)
    annotations['group_ids'] = np.arange(num_gt, dtype=np.int32)
    annotations['difficulty']=np.zeros((annotations['bbox'].shape[0], ))
    return annotations

def drop_arrays_by_name(gt_names, used_classes):
    inds = [
        i for i, x in enumerate(gt_names) if x not in used_classes
    ]
    inds = np.array(inds, dtype=np.int64)
    return inds   
    
if __name__ == '__main__':
    fire.Fire()
