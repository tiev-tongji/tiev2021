# vision-Roadmarking

**单一前视相机输入的路面标记检测程序，目前可实现车道线（区分颜色和虚实），转向箭头（区分左右前），停止线的检测，并进行后处理拟合滤波**


## 使用

### Dependency install

`pip3 install -r requirements.txt`

& dependencies from apt

see 'apt_installs.txt'

### API
1. 声明引用
    ```from road_marking_dect_offline import RoadMarkingOffLine```
    or 
    ```from road_marking_dect_online import RoadMarkingOnLine```

2. 初始化可选参数
 - pretrained_model_path: 预训练模型路径
 - eh_type: 直方图归一化模式，"no" or "rgb" or "hsv"
 - device: 运行设备, 'cuda' or 'cpu'
 - video_path(仅 offline):离线测试视频（已转换到鸟瞰图的）
 - with_post: 是否运行后处理相关代码

## tiev2019车道线检测程序重构后离线定性测试效果
使用模型：
epoch_6_loss_0.06795_acc_0.97691_acc-cls_0.59547_mean-iu_0.46755_fwavacc_0.95794_lr_0.0002000000.pth

获取方式：

`wget https://gitlab.com/tjiv/nn_weights/-/raw/vision_roadmarking/vision_roadmarking/epoch_6_loss_0.06795_acc_0.97691_acc-cls_0.59547_mean-iu_0.46755_fwavacc_0.95794_lr_0.0002000000.pth`

.![avatar](modules/src/vision/RoadMarkingDect/test_result.png)
.![avatar](modules/src/vision/RoadMarkingDect/test_result2.png)

## 性能测试

### FPS
| 功能 | 时间(ms)| 运行设备 | 备注 |
| :------: | :------: | :------: |:------: |
| 读取视频流 | **/** | **暂无可用测试设备** |  |
| 生成鸟瞰图 | 29 | 设备1 | 离线数据测试 |
| 特征网络 | 18 | 设备1（占用显存1055M）| FCN+ResNet18|
| 后处理 | 50 | 设备2 | 虚拟机上运行 |

设备1：
 - CPU：Intel(R) Xeon(R) CPU E5-2678 v3 @ 2.50GHz
 - GPU：NVIDIA Corporation GP102 [GeForce GTX 1080 Ti]

设备2：
 - CPU：Intel(R) Core(TM) i7-6700HQ CPU @ 2.60GHz（笔记本虚拟机）
 

### Accuracy
*以下测试为在ApolloScape数据集上进行测试的结果*

| 指标 |像素acc | 分类别平均acc | mean iou | 按类别频率加权iou |
| :------: | :------: | :------: |:------: |:------: |
| 数值 | 0.977 | 0.595 | 0.468 | 0.958 |


## ToDo 
* [] 验证训练模型在Apollo数据集上反变换回原视图的mIou是否得到提升
* [] 训练优化，寻找更佳的模型（网络层数，类别权重，图像前处理）


