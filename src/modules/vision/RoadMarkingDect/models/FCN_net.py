import torch
from torch import nn
import pdb
from .vgg_net import vgg16, vgg11_bn
from utils import get_upasmpling_weight

class FCN8s_bn(nn.Module):
    def __init__(self, num_class, pretrained=True):
        super(FCN8s_bn, self).__init__()
        vgg = vgg11_bn(pretrained)
        features, classifier = list(vgg.features.children()), list(vgg.classifier.children())
        # pdb.set_trace()
        # features[0].padding = (100, 100)

        for f in features:
            if 'MaxPool' in f.__class__.__name__:
                f.ceil_mode = True
            elif 'ReLU' in f.__class__.__name__:
                f.inplace = True

        # self.features3 = nn.Sequential(*features[:24])
        # self.features4 = nn.Sequential(*features[24:34])
        # self.features5 = nn.Sequential(*features[34:])
        #
        self.features3 = nn.Sequential(*features[:15])
        self.features4 = nn.Sequential(*features[15:22])
        self.features5 = nn.Sequential(*features[22:])

        score_pool3_cov = nn.Conv2d(256, num_class, kernel_size=1)
        score_pool4_cov = nn.Conv2d(512, num_class, kernel_size=1)
        score_pool3_cov.weight.data.zero_()
        # nn.init.kaiming_normal_(self.score_pool3.weight, mode='fan_out', nonlinearity='relu')
        score_pool3_cov.bias.data.zero_()
        score_pool4_cov.weight.data.zero_()
        # nn.init.kaiming_normal_(self.score_pool4.weight, mode='fan_out', nonlinearity='relu')
        score_pool4_cov.bias.data.zero_()
        self.score_pool4 = nn.Sequential(score_pool4_cov, nn.BatchNorm2d(num_class))
        self.score_pool3 = nn.Sequential(score_pool3_cov, nn.BatchNorm2d(num_class))


        fc6 = nn.Conv2d(512, 1024, kernel_size=1)
        nn.init.kaiming_normal_(fc6.weight, mode='fan_out', nonlinearity='relu')
        # fc6.weight.data.copy_(classifier[0].weight.data.view(4096, 512, 7, 7))
        fc6.bias.data.zero_()
        fc7 = nn.Conv2d(1024, 1024, kernel_size=1)
        # fc7.weight.data.copy_(classifier[3].weight.data.view(1024, 1024, 1, 1))
        nn.init.kaiming_normal_(fc7.weight, mode='fan_out', nonlinearity='relu')
        # fc7.bias.data.copy_(classifier[3].bias.data)
        fc7.bias.data.zero_()
        score_fr = nn.Conv2d(1024, num_class, kernel_size=1)
        # score_fr.weight.data.zero_()
        nn.init.kaiming_normal_(score_fr.weight, mode='fan_out', nonlinearity='relu')
        score_fr.bias.data.zero_()
        self.score_fr = nn.Sequential(
            fc6, nn.BatchNorm2d(1024), nn.ReLU(inplace=True), nn.Dropout(), fc7, nn.BatchNorm2d(1024), nn.ReLU(inplace=True),
            nn.Dropout(), score_fr, nn.BatchNorm2d(num_class))

        upscore2_conv = nn.ConvTranspose2d(num_class, num_class, kernel_size=4, stride=2, padding=1, bias=False)
        upscore2_conv.weight.data.copy_(get_upasmpling_weight(num_class, num_class, 4))
        self.upscore2 = nn.Sequential(upscore2_conv, nn.BatchNorm2d(num_class))
        upscore4_conv = nn.ConvTranspose2d(num_class, num_class, kernel_size=4, stride=2, padding=1, bias=False)
        upscore4_conv.weight.data.copy_(get_upasmpling_weight(num_class, num_class, 4))
        self.upscore4 = nn.Sequential(upscore4_conv, nn.BatchNorm2d(num_class))
        self.upscore8 = nn.ConvTranspose2d(num_class, num_class, kernel_size=16, stride=8, padding=4, bias=False)
        self.upscore8.weight.data.copy_(get_upasmpling_weight(num_class, num_class, 16))

    def forward(self, x):
        # import numpy as  np
        # x = np.random.rand(1,3,256,512)
        # x = torch.tensor(x).float().cuda()
        x_size = x.size()
        # import pdb;pdb.set_trace()
        pool3 = self.features3(x)
        pool4 = self.features4(pool3)
        pool5 = self.features5(pool4)

        score_pool5 = self.score_fr(pool5)
        upscore2 = self.upscore2(score_pool5)

        score_pool4 = self.score_pool4(0.01 * pool4)
        upscore4 = self.upscore4(score_pool4
                                 + upscore2)
        score_pool3 = self.score_pool3(0.0001 * pool3)
        upscore8 = self.upscore8(score_pool3
                                 + upscore4)

        return upscore8.contiguous()

