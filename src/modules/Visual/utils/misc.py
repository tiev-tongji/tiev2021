import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
import os

def get_upasmpling_weight(in_channels, out_channels, kernel_size):
    factor = (kernel_size + 1) //2
    if kernel_size % 2 ==1:
        center = factor -1
    else:
        center = factor - 0.5
    og = np.ogrid[:kernel_size, :kernel_size]
    filt = (1 - abs(og[0]-center) / factor) * (1 - abs(og[1]-center) / factor)
    weight = np.zeros((in_channels, out_channels, kernel_size, kernel_size), dtype=np.float64)
    weight[list(range(in_channels)), list(range(out_channels)), :, :] = filt
    return torch.from_numpy(weight).float()


def select_from_uniid(fwd, inputs):
    if fwd == 'Apollo':
        MAP_FILE = './utils/map_apolloscape_laneseq.yml'

    if fwd == 'RoadMarking':
        MAP_FILE = './utils/map_Road_Mark_dataset.yml'
    if fwd == 'TSD_Lane':
        MAP_FILE = './utils/map_Tsd_lane.yml'
    if fwd == 'Tsinghua_TRoM':
        MAP_FILE = './utils/map_Tsinghua_TRoM.yml'

    # MAP_FILE = './utils/map_Tsinghua_TRoM.yml'
    mapdef = MapDef(MAP_FILE)
    map_dict = {}
    for k, v in mapdef.items():
        map_dict[v.origin_id] = v.unified_ids
        # print(k, ':', v)
    # import pdb; pdb.set_trace()
    for k in sorted(map_dict):
        v = map_dict[k]
        # import pdb;pdb.set_trace()
        vt = torch.LongTensor(v).cuda()
        selected = torch.index_select(inputs, 1, vt)
        if k == 0:
            inputs_oriid = selected.sum(1).unsqueeze(1)
        else:
            inputs_oriid = torch.cat((inputs_oriid, selected.sum(1).unsqueeze(1)), 1)

    return inputs_oriid


class CrossEntropyLoss2d(nn.Module):
    def __init__(self, weight=None, ignore_index=255):
        super(CrossEntropyLoss2d, self).__init__()
        self.nll_loss = nn.NLLLoss(weight, ignore_index)

    def forward(self, inputs, targets):
        return self.nll_loss(F.log_softmax(inputs), targets)


class Uni_CrossEntropyLoss2d(nn.Module):
    def __init__(self, weight=None, reduction='mean', ignore_index=255):
        super(Uni_CrossEntropyLoss2d, self).__init__()
        self.nll_loss = nn.NLLLoss(weight=weight, ignore_index=ignore_index, reduction=reduction)


    def forward(self, inputs, targets, fwd):

        def my_softmax(inputs_oriid_exp, inputs_uniid_exp):
            uni_sum = torch.sum(inputs_uniid_exp, 1).unsqueeze(1)#+1e-15
            s = inputs_oriid_exp / uni_sum
            if s.min() < 0 or s.max() > 1 :
                import pdb;pdb.set_trace()
            else:
                pass
                # print(s)
            pp = torch.log(s)#+ 1e-15)
            return pp

        max_ = inputs.max()
        inputs_uniid_exp = torch.exp(inputs - max_)
        inputs_oriid_exp = select_from_uniid(fwd, inputs_uniid_exp)
        bb = my_softmax(inputs_oriid_exp, inputs_uniid_exp)
        # aa = F.log_softmax(inputs_oriid)
        return self.nll_loss(bb, targets)


def check_mkdir(dir_name):
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)


class AverageMeter(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count


#todo confirm the use of num_classes
def _fast_hist(label_pred, label_true, num_classes, fwd):

    if not fwd is None:
        if fwd == 'Apollo':
            MAP_FILE = './utils/map_apolloscape_laneseq.yml'
        if fwd == 'RoadMarking':
            MAP_FILE = './utils/map_Road_Mark_dataset.yml'
        if fwd == 'TSD_Lane':
            MAP_FILE = './utils/map_Tsd_lane.yml'
        if fwd == 'Tsinghua_TRoM':
            MAP_FILE = './utils/map_Tsinghua_TRoM.yml'
        # MAP_FILE = './utils/map_Tsinghua_TRoM.yml'
        mapdef = MapDef(MAP_FILE)
        map_dict = {}
        for k, v in mapdef.items():
            map_dict[v.origin_id] = v.unified_ids
            # print(k, ':', v)
        # import pdb; pdb.set_trace()
        for k in sorted(map_dict):
            v = map_dict[k]
            loc = np.where(label_pred == k)
            label_pred[loc] = v[0]
            loc2 = np.where(label_true == k)
            label_true[loc2] = v[0]

    mask = (label_true >= 0) & (label_true < num_classes)
    hist = np.bincount(
        num_classes * label_true[mask].astype(int) +
        label_pred[mask], minlength=num_classes ** 2).reshape(num_classes, num_classes)
    return hist


def get_hist(predictions, gts, num_classes, fwd=None):
    hist = np.zeros((num_classes, num_classes))
    for lp, lt in zip(predictions, gts):
        hist += _fast_hist(lp.flatten(), lt.flatten(), num_classes, fwd)
    return hist

def evaluate(hist):
    # hist = np.zeros((num_classes, num_classes))
    # for lp, lt in zip(predictions, gts):
    #     hist += _fast_hist(lp.flatten(), lt.flatten(), num_classes)
    # axis 0: gt, axis 1: prediction
    # import pdb;pdb.set_trace()
    acc = np.diag(hist).sum() / hist.sum()
    #不区分种类，像素级准确度
    acc_cls = np.diag(hist) / hist.sum(axis=1)
    acc_cls = np.nanmean(acc_cls)
    #每个种类计算准确度，求绝对平均值
    iu = np.diag(hist) / (hist.sum(axis=1) + hist.sum(axis=0) - np.diag(hist))
    mean_iu = np.nanmean(iu)
    #每个种类计算：正检/（正检+漏检+误检），求绝对平均
    freq = hist.sum(axis=1) / hist.sum()
    fwavacc = (freq[freq > 0] * iu[freq > 0]).sum()
    #类别（检测）出现的频率*这一类的正检/（正检+漏检+误检），再对所有类别求和
    return acc, acc_cls, mean_iu, fwavacc


