import torch
from torchvision import models
from torchvision.models.resnet import ResNet, BasicBlock, Bottleneck
from torch import nn
from torch.nn import init
import pdb
# from .vgg_net import vgg16, vgg11
# from utils import get_upasmpling_weight


# Returns 2D convolutional layer with space-preserving padding
def conv(in_planes, out_planes, kernel_size=3, stride=1, dilation=1, bias=False, transposed=False):
    if transposed:
        layer = nn.ConvTranspose2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride, padding=1, output_padding=1, dilation=dilation, bias=bias)
        # Bilinear interpolation init
        w = torch.Tensor(kernel_size, kernel_size)
        centre = kernel_size % 2 == 1 and stride - 1 or stride - 0.5
        for y in range(kernel_size):
            for x in range(kernel_size):
                w[y, x] = (1 - abs((x - centre) / stride)) * (1 - abs((y - centre) / stride))
        layer.weight.data.copy_(w.div(in_planes).repeat(in_planes, out_planes, 1, 1))
    else:
        padding = (kernel_size + 2 * (dilation - 1)) // 2
        layer = nn.Conv2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride, padding=padding, dilation=dilation, bias=bias)
    if bias:
        init.constant(layer.bias, 0)
    return layer


# Returns 2D batch normalisation layer
def bn(planes):
    layer = nn.BatchNorm2d(planes)
    # Use mean 0, standard deviation 1 init
    init.constant_(layer.weight, 1)
    init.constant_(layer.bias, 0)
    return layer




class FeatureResNet(ResNet):
    def __init__(self):
        super().__init__(BasicBlock, [2,2,2,2], 1000)

    def forward(self, x):
        x1 = self.conv1(x)
        x = self.bn1(x1)
        x = self.relu(x)
        x2 = self.maxpool(x)
        x = self.layer1(x2)
        x3 = self.layer2(x)
        x4 = self.layer3(x3)
        x5 = self.layer4(x4)
        return x1, x2, x3, x4, x5


class FCN8s_ResNet(nn.Module):
    def __init__(self, num_classes, pretrained=False):
        super().__init__()
        resnet = models.resnet18(pretrained)
        self.feature_resnet = FeatureResNet()
        self.feature_resnet.load_state_dict(resnet.state_dict())

        #Resnet34,18
        self.relu = nn.ReLU(inplace=True)
        self.conv5 = conv(512, 256, stride=2, transposed=True)
        self.bn5 = bn(256)
        self.conv6 = conv(256, 128, stride=2, transposed=True)
        self.bn6 = bn(128)
        self.conv7 = conv(128, 64, stride=2, transposed=True)
        self.bn7 = bn(64)
        self.conv8 = conv(64, 64, stride=2, transposed=True)
        self.bn8 = bn(64)
        self.conv9 = conv(64, 32, stride=2, transposed=True)
        self.bn9 = bn(32)
        self.conv10 = conv(32, num_classes, kernel_size=7)

        #Resnet50
        # self.conv5 = conv(2048, 1024, stride=2, transposed=True)
        # self.bn5 = bn(1024)
        # self.conv6 = conv(1024, 512, stride=2, transposed=True)
        # self.bn6 = bn(512)
        # self.conv7 = conv(512, 64, stride=2, transposed=True)
        # self.bn7 = bn(64)
        # self.conv8 = conv(64, 64, stride=2, transposed=True)
        # self.bn8 = bn(64)
        # self.conv9 = conv(64, 32, stride=2, transposed=True)
        # self.bn9 = bn(32)
        # self.conv10 = conv(32, num_classes, kernel_size=7)

        init.constant_(self.conv10.weight, 0)  # Zero init

    def forward(self, x):
        x1, x2, x3, x4, x5 = self.feature_resnet(x)
        x = self.relu(self.bn5(self.conv5(x5)))
        x = self.relu(self.bn6(self.conv6(x + x4)))
        x = self.relu(self.bn7(self.conv7(x + x3)))
        x = self.relu(self.bn8(self.conv8(x + x2)))
        x = self.relu(self.bn9(self.conv9(x + x1)))
        x = self.conv10(x)
        # pdb.set_trace()
        return x

