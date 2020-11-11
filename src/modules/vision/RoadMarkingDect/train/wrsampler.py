import torch
import os
import numpy as np
import torch.nn as nn
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Sampler
from torchvision import datasets


class WeightedRandomSampler(Sampler):
    """
    初始化均匀self.weights,输出初次抽样索引列表
    每一轮训练结束后用update函数更新self.weights
    下一轮开始时iter输出抽样索引列表
    """

    def __init__(self, num_samples):
        """
        初始化权重均相同，跟据该权重初次抽样进入网络
        :param num_samples:
        """
        Sampler.__init__(self, data_source=None)
        self.weights = torch.zeros(num_samples).fill_(1 / num_samples)
        self.num_samples = num_samples
        
    def __iter__(self):
        self.sampled_index = torch.multinomial(self.weights, self.num_samples, replacement=True).tolist()
        # print(self.weights,self.sampled_index)
        return iter(self.sampled_index)

    def __len__(self):
        return self.num_samples

    def __call__(self, k):
        top_k, top_index = torch.topk(self.weights, k)
        # print(top_index[:100])
        sample_top_k_total = torch.tensor(0)
        for i in range(k):
            sample_top_k_total = torch.where(torch.tensor(self.sampled_index) == top_index[i], sample_top_k_total + 1,
                                             sample_top_k_total)
        print(sample_top_k_total.sum())

    def update(self, loss):
        """
        传入loss，计算对应于上轮采样索引的weights，之后更新self.weights，将self.weights对应到[0,1,2……n]的初始样本索引
        :param loss:
        :return:
        """
        self.weights = loss / torch.sum(loss)
        # print(self.weights)
        new_weights = torch.zeros(self.num_samples).fill_(1 / self.num_samples)
        old_sampled_index = self.sampled_index
        for i in range(self.num_samples):
            new_weights[old_sampled_index[i]] = self.weights[i]
        self.weights = new_weights
        # print(self.weights)
