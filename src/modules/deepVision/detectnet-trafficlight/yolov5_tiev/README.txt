2020.10.18 模型从EfficientDet D2更换为Yolov5s，显著减少漏检。

模型训练
python train.py --data trafficLight12.yaml --cfg yolov5s.yaml --weights yolov5s.pt --batch-size 48 --epochs 100

速度测试
python test.py --weights best.pt --data trafficLight12.yaml --batch-size 1 --device 0

使用
python detect.py --weights best1.pt --source basler

资源占用：GPU(TitanX) ram 655Mib，CPU 12%，ram 1g

速度：在 TitanX上 9.5ms 

精度：mAP 0.5 = 0.989，mAP 0.5:0.95 = 0.755，recall = 0.995
