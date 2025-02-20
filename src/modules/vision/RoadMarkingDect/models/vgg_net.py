import torch.nn as nn
# import torch.utils.model_zoo as model_zoo
import math
import torch
import os

__all__ = ['VGG', 'vgg16']

model_urls = {
    'vgg16': 'https://download.pytorch.org/models/vgg16-397923af.pth'
}

root = '/home/renxiaozhou/code/SegNet_bymyself/'

vgg_16_bn_model_path = os.path.join(root, 'pretrained_modl/vgg16_bn-6c64b313.pth')

vgg_11_bn_model_path = os.path.join(root, 'pretrained_modl/vgg11_bn-6002323d.pth')
vgg_11_model_path = os.path.join(root, 'pretrained_modl/vgg11-bbd30ac9.pth')
vgg_16_model_path = os.path.join(root, 'pretrained_modl/vgg16-397923af.pth')


class VGG(nn.Module):

    def __init__(self, features, num_classes=1000, init_weights=True):
        super(VGG, self).__init__()
        self.features = features
        self.classifier = nn.Sequential(
            nn.Linear(512*7*7, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, num_classes)
        )
        if init_weights:
            self._initialze_weights()

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0),-1)
        x= self.classifier(x)
        return x

    def _initialze_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2./n))
                if m.bias is not None:
                    m.bias.data.zero_()
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()
            elif isinstance(m, nn.Linear):
                m.weight.data.normal_(0, 0.01)
                m.bias.data.zero_()


def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)

            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)

cfg = {
    'A': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'B': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'D': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M']
}

def vgg16_bn(pretrained=False, **kwargs):

    if pretrained:
        kwargs['init_weights'] = False
    model = VGG(make_layers(cfg['D'], batch_norm=True), **kwargs)
    if pretrained:
        model.load_state_dict(torch.load(vgg_16_bn_model_path))
    return model

def vgg16(pretrained=False, **kwargs):

    if pretrained:
        kwargs['init_weights'] = False
    model = VGG(make_layers(cfg['D']), **kwargs)
    if pretrained:
        model.load_state_dict(torch.load(vgg_16_model_path))
    return model


def vgg11_bn(pretrained=False, **kwargs):

    if pretrained:
        kwargs['init_weights'] = False
    model = VGG(make_layers(cfg['A'], batch_norm=True), **kwargs)
    if pretrained:
        model.load_state_dict(torch.load(vgg_11_bn_model_path))
    return model

def vgg11(pretrained=False, **kwargs):

    if pretrained:
        kwargs['init_weights'] = False
    model = VGG(make_layers(cfg['A']), **kwargs)
    if pretrained:
        model.load_state_dict(torch.load(vgg_11_model_path))
    return model
