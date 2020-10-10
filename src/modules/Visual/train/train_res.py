import os
from torch.backends import cudnn
from tensorboardX import SummaryWriter
import torch
import datetime
import random
from torch import nn
import numpy as np
import torchvision.transforms as standard_transforms
from torch import optim
from torch.autograd import Variable
from torch.optim.lr_scheduler import ReduceLROnPlateau
from torch.utils.data import DataLoader
import torchvision.utils as vutils
import gc
import time
from dataset import Apollo_data
from models import FCN8s_ResNet
import utils.joint_transforms as joint_transforms
import utils.extend_transform as extended_transforms
from utils import check_mkdir, get_hist, evaluate, AverageMeter, CrossEntropyLoss2d
import pdb

cudnn.benchmark = True

num_classes = 38
ignore_label = 255
# device_ids = [1, 3]

ckpt_path = '/mnt/ExtraDisk/xiaozhou_temp/fcn_resnet2/'
exp_name = 'fcn8s'
writer = SummaryWriter(os.path.join(ckpt_path, 'event', exp_name))

args = {
    'train_batch_size': 10,
    'epoch_num': 5000,
    'lr': 8e-6,
    'weight_decay': 5e-4,
    'input_size': (512, 2048), #(w, h)
    'momentum': 0.95,
    'lr_patience': 100,
    'snapshot': 'epoch_4_loss_154150.81835_acc_0.96201_acc-cls_0.29344_mean-iu_0.24716_fwavacc_0.93127_lr_0.0008000000.pth',
    'print_freq': 10,
    'val_batch_size': 10,
    'val_save_to_img_file': False,
    'val_img_sample_rate': 0.002
}


def main():
    # pdb.set_trace()

    net = FCN8s_ResNet(num_classes=num_classes).cuda()
        # cuda(device_ids[0])
    net = nn.DataParallel(net)

    if len(args['snapshot']) == 0:
        curr_epoch = 1
        #todo confim what's every word mean
        args['best_record'] = {'epoch': 0, 'val_loss': 1e12, 'acc': 0, 'acc_cls': 0, 'mean_iu': 0, 'fwavacc': 0}

    else:
        print('training resumes from' + args['snapshot'])
        pretrained_model = torch.load(os.path.join(ckpt_path, exp_name, args['snapshot']))
        # pdb.set_trace()
        # pretrained_dict = pretrained_model.module.state_dict()
        net.load_state_dict(pretrained_model)
        split_snapshot = args['snapshot'].split('_')
        curr_epoch = int(split_snapshot[1]) + 1
        args['best_record'] = {'epoch': int(split_snapshot[1]), 'val_loss': float(split_snapshot[3]),
                               'acc': float(split_snapshot[5]), 'acc_cls': float(split_snapshot[7]),
                               'mean_iu': float(split_snapshot[9]), 'fwavacc': float(split_snapshot[11])}

    net.train()
    #todo caculate real mean&std
    mean_std = ([0.51, 0.51, 0.51], [0.294, 0.297, 0.295])
    # short_size = int(max(args['input_size'])/0.875)

    #todo confirm the shape after every transform
    # train_joint_transform = joint_transforms.Compose([
    #     # joint_transforms.Scale(short_size),
    #     joint_transforms.RandomCrop(args['input_size']),
    #     # joint_transforms.RandomHorizontallyFlip()
    # ])

    train_joint_transform = joint_transforms.Compose([
        # joint_transforms.Scale(short_size),
        # joint_transforms.Resize(512, 2048)
        # joint_transforms.RandomHorizontallyFlip()
    ])

    # val_joint_transform = joint_transforms.Compose([
    #     # joint_transforms.Scale(short_size),
    #     joint_transforms.CenterCrop(args['input_size'])
    # ])
    val_joint_transform = joint_transforms.Compose([
        # joint_transforms.Scale(short_size),
        # joint_transforms.Resize(512, 2048)
    ])

    input_transform = standard_transforms.Compose([
        extended_transforms.EqualizeHist(),
        standard_transforms.ToTensor(),
        standard_transforms.Normalize(*mean_std)
    ])

    target_transform = extended_transforms.MaskToTensor()

    restore_transform = standard_transforms.Compose([
        extended_transforms.DeNormalize(*mean_std),
        standard_transforms.ToPILImage()
    ])

    visualize = standard_transforms.ToTensor()

    train_set = Apollo_data.Apollo('train', joint_transform=None, transform=input_transform,
                                   target_transform=target_transform)
    # train_set.get(1)
    train_loader = DataLoader(train_set, batch_size=args['train_batch_size'], num_workers=8, shuffle=True)
    valid_set = Apollo_data.Apollo('valid', joint_transform=None, transform=input_transform,
                                   target_transform=target_transform)
    valid_loader = DataLoader(valid_set, batch_size=args['val_batch_size'], num_workers=8, shuffle=False)

    criterion = CrossEntropyLoss2d(size_average=False, ignore_index=ignore_label).cuda()
    #why -4: eg:name = 'features3.0.bias'
    optimizer = optim.Adam([
        {'params': [param for name, param in net.named_parameters() if name[-4:] == 'bias'],
         'lr': 2 * args['lr']},
        {'params': [param for name, param in net.named_parameters() if name[-4:] != 'bias'],
         'lr': args['lr'], 'weight_decay': args['weight_decay']}
    ])#, momentum=args['momentum']

    # optimizer = nn.DataParallel(optimizer, device_ids=device_ids)

    if len(args['snapshot']) > 0:
        optimizer.load_state_dict(torch.load(os.path.join(ckpt_path, exp_name, 'opt_' + args['snapshot'])))
        optimizer.param_groups[0]['lr'] = 2 * args['lr']
        optimizer.param_groups[1]['lr'] = args['lr']

    check_mkdir(ckpt_path)
    check_mkdir(os.path.join(ckpt_path, exp_name))
    open(os.path.join(ckpt_path, exp_name, str(datetime.datetime.now()) + '.txt'), 'w').write(str(args) + '\n\n')

    scheduler = ReduceLROnPlateau(optimizer, 'min', patience=args['lr_patience'], min_lr=1e-14)
    for epoch in range(curr_epoch, args['epoch_num'] + 1):
        # pdb.set_trace()
        train(train_loader, net, criterion, optimizer, epoch, args)
        # pdb.set_trace()
        # if (epoch-1) % 4 == 0:
        val_loss = validate(valid_loader, net, criterion, optimizer, epoch, args, restore_transform, visualize)
        scheduler.step(val_loss)


def train(train_loader, net, criterion, optimizer, epoch, train_args):
    train_loss = AverageMeter()
    curr_iter = (epoch - 1) * len(train_loader)
    # pdb.set_trace()
    time1 = time.time()
    for i, data in enumerate(train_loader):
        inputs, labels = data
        assert inputs.size()[2:] == labels.size()[1:]
        N = inputs.size(0)
        # pdb.set_trace()
        inputs = Variable(inputs).cuda()
        labels = Variable(labels).cuda()

        optimizer.zero_grad()
        # pdb.set_trace()
        outputs = net(inputs)
        # pdb.set_trace()
        assert outputs.size()[2:] == labels.size()[1:]
        assert outputs.size()[1] == num_classes

        loss = criterion(outputs, labels) / N
        loss.backward()
        optimizer.step()
        # pdb.set_trace()
        train_loss.update(loss.data.item(), N)

        curr_iter += 1
        writer.add_scalar('train_loss', train_loss.avg, curr_iter)
        if (i + 1) % train_args['print_freq'] == 0:
            time2 = time.time()
            time_delta = time2 - time1
            time1 = time2
            print('[epoch %d], [iter %d / %d], [train loss %.5f], [time %.3f s]' % (
                epoch, i + 1, len(train_loader), train_loss.avg, time_delta))
        if (i+1) % 2000 == 0:
            snapshot_name = 'train_epoch_%d_iter_%d_loss_%.5f_lr_%.10f' % (
                epoch, i + 1, train_loss.avg, optimizer.param_groups[1]['lr']
            )
            torch.save(net.state_dict(), os.path.join(ckpt_path, exp_name, snapshot_name + '.pth'))
            torch.save(optimizer.state_dict(), os.path.join(ckpt_path, exp_name, 'opt_' + snapshot_name + '.pth'))
            print('saving a new model')


def validate(val_loader, net, criterion, optimizer, epoch, train_args, restore, visualize):
    net.eval()

    val_loss = AverageMeter()
    inputs_all, gts_all, predictions_all = [], [], []
    # pdb.set_trace()
    hist = np.zeros((num_classes, num_classes))
    datai = []
    for vi, data in enumerate(val_loader):
        with torch.no_grad():
            if random.random() > train_args['val_img_sample_rate']:
                pass
            else:
                datai.append(data)
            inputs, gts = data
            N = inputs.size(0)
            inputs = Variable(inputs, volatile=True).cuda()
            gts = Variable(gts, volatile=True).cuda()
            outputs = net(inputs)
        #todo: condirm the shape
        predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
        val_loss.update(criterion(outputs, gts).data.item() / N, N)
        gts = gts.data.cpu().numpy()
        # gts_all.append(gts.data.cpu().numpy())
        # predictions_all.append(predictions)
        if (vi + 1) % train_args['print_freq'] == 0:
            print('[epoch %d], [iter %d / %d], [valid loss %.5f]' % (
                epoch, vi + 1, len(val_loader), val_loss.avg))
        # pdb.set_trace()
        # gts_all.append(gts)
        # gts_all = np.concatenate(gts_all)
        # predictions_all.append(predictions)
        # predictions_all = np.concatenate(predictions_all)
        hist += get_hist(predictions, gts, num_classes)
    # pdb.set_trace()
    # gts_all = np.concatenate(gts_all)
    # predictions_all = np.concatenate(predictions_all)
    acc, acc_cls, mean_iu, fwavacc = evaluate(hist)

    print(
        '-----------------------------------------------------------------------------------------------------------')
    print('[epoch %d], [val loss %.5f], [acc %.5f], [acc_cls %.5f], [mean_iu %.5f], [fwavacc %.5f]' % (
        epoch, val_loss.avg, acc, acc_cls, mean_iu, fwavacc))
#todo!!!!!!!!!!
    if mean_iu > train_args['best_record']['mean_iu']:
        train_args['best_record']['val_loss'] = val_loss.avg
        train_args['best_record']['epoch'] = epoch
        train_args['best_record']['acc'] = acc
        train_args['best_record']['acc_cls'] = acc_cls
        train_args['best_record']['mean_iu'] = mean_iu
        train_args['best_record']['fwavacc'] = fwavacc
        snapshot_name = 'epoch_%d_loss_%.5f_acc_%.5f_acc-cls_%.5f_mean-iu_%.5f_fwavacc_%.5f_lr_%.10f' % (
            epoch, val_loss.avg, acc, acc_cls, mean_iu, fwavacc, optimizer.param_groups[1]['lr']
        )
        torch.save(net.state_dict(), os.path.join(ckpt_path, exp_name, snapshot_name + '.pth'))
        torch.save(optimizer.state_dict(), os.path.join(ckpt_path, exp_name, 'opt_' + snapshot_name + '.pth'))

        if train_args['val_save_to_img_file']:
            to_save_dir = os.path.join(ckpt_path, exp_name, str(epoch))
            check_mkdir(to_save_dir)

        val_visual = []
        for vii, dataii in enumerate(datai):
            with torch.no_grad():
                inputs, gts = dataii
                inputs = Variable(inputs).cuda()
                gts = Variable(gts).cuda()
                outputs = net(inputs)
            predictions = outputs.data.max(1)[1].squeeze_(1).cpu().numpy()
            gts = gts.data.cpu().numpy()
            inputs = inputs.data.cpu()

            # pdb.set_trace()

            input_pil = restore(inputs[0])
            gt_pil = Apollo_data.colorize_mask(gts[0])
            predictions_pil = Apollo_data.colorize_mask(predictions[0])
            if train_args['val_save_to_img_file']:
                input_pil.save(os.path.join(to_save_dir, '%d_input.png' % vii))
                predictions_pil.save(os.path.join(to_save_dir, '%d_prediction.png' % vii))
                gt_pil.save(os.path.join(to_save_dir, '%d_gt.png' % vii))
            val_visual.extend([visualize(input_pil.convert('RGB')), visualize(gt_pil.convert('RGB')),
                               visualize(predictions_pil.convert('RGB'))])
        # pdb.set_trace()
        if len(val_visual) > 0:
            val_visual = torch.stack(val_visual, 0)
            val_visual = vutils.make_grid(val_visual, nrow=3, padding=5)
            writer.add_image(snapshot_name, val_visual)



    print('best record: [val loss %.5f], [acc %.5f], [acc_cls %.5f], [mean_iu %.5f], [fwavacc %.5f], [epoch %d]' % (
        train_args['best_record']['val_loss'], train_args['best_record']['acc'],
        train_args['best_record']['acc_cls'],
        train_args['best_record']['mean_iu'], train_args['best_record']['fwavacc'],
        train_args['best_record']['epoch']))

    print(
        '-----------------------------------------------------------------------------------------------------------')

    writer.add_scalar('val_loss', val_loss.avg, epoch)
    writer.add_scalar('acc', acc, epoch)
    writer.add_scalar('acc_cls', acc_cls, epoch)
    writer.add_scalar('mean_iu', mean_iu, epoch)
    writer.add_scalar('fwavacc', fwavacc, epoch)
    writer.add_scalar('lr', optimizer.param_groups[1]['lr'], epoch)

    net.train()
    return val_loss.avg


if __name__ == '__main__':
    main()
