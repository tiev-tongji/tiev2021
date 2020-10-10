import torch
from torch import nn
from torch.autograd import Variable
import torch.nn.functional as F
import numpy as np

def softmax(x):
    x_exp = torch.exp(x)
    #如果是列向量，则axis=0
    x_sum = torch.sum(x_exp, 1).unsqueeze(1)
    s = x_exp / x_sum
    return torch.log(s)


m = nn.LogSoftmax()
input = Variable(torch.randn(1,2,5,4))
print(input)
print(m(input))
print(F.log_softmax(input))
print(softmax(input))