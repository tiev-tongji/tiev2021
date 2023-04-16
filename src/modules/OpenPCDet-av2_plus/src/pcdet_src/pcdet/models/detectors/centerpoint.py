from .detector3d_template import Detector3DTemplate
import time
from ...ops.pointnet2.pointnet2_stack.pointnet2_utils import farthest_point_sample

class CenterPoint(Detector3DTemplate):
    def __init__(self, model_cfg, num_class, dataset):
        super().__init__(model_cfg=model_cfg, num_class=num_class, dataset=dataset)
        self.module_list = self.build_networks()

    def forward(self, batch_dict):
        # self._FPS(batch_dict)
        fd_time = {}
        for cur_module in self.module_list:
            fd_time[cur_module.model_cfg['NAME']] = time.time()
            batch_dict = cur_module(batch_dict)
            fd_time[cur_module.model_cfg['NAME']] = time.time() - fd_time[cur_module.model_cfg['NAME']]

        if self.training:
            loss, tb_dict, disp_dict = self.get_training_loss()
            disp_dict['fd_time'] = fd_time

            ret_dict = {
                'loss': loss
            }
            return ret_dict, tb_dict, disp_dict
        else:
            pred_dicts, recall_dicts = self.post_processing(batch_dict)
            return pred_dicts, recall_dicts
    
    def _FPS(self, batch_dict):
        start_t = time.time()
        cur_index = 0
        num_frame = 0
        for cur_points_num in batch_dict['points_num']:
            cur_points_num = int(cur_points_num)
            cur_points_xyz = batch_dict['points'][cur_index:cur_index + cur_points_num][:, 1:4]
            cur_pts_fps_idxs = farthest_point_sample(
                cur_points_xyz.view(1, -1, 3).contiguous(), int(cur_points_num / 2)
            ).long()[0]
            cur_points_xyz = cur_points_xyz[cur_pts_fps_idxs]
            cur_index += cur_points_num
            num_frame += 1
            
        end_t = time.time()
        print(f"\nGPU FPS Time Cost: {(end_t - start_t) / num_frame}")
        pass

    def get_training_loss(self):
        disp_dict = {}

        loss_rpn, tb_dict = self.dense_head.get_loss()
        tb_dict = {
            'loss_rpn': loss_rpn.item(),
            **tb_dict
        }

        loss = loss_rpn
        return loss, tb_dict, disp_dict

    def post_processing(self, batch_dict):
        post_process_cfg = self.model_cfg.POST_PROCESSING
        batch_size = batch_dict['batch_size']
        final_pred_dict = batch_dict['final_box_dicts']
        recall_dict = {}
        for index in range(batch_size):
            pred_boxes = final_pred_dict[index]['pred_boxes']

            recall_dict = self.generate_recall_record(
                box_preds=pred_boxes,
                recall_dict=recall_dict, batch_index=index, data_dict=batch_dict,
                thresh_list=post_process_cfg.RECALL_THRESH_LIST
            )

        return final_pred_dict, recall_dict
