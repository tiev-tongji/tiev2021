# 连接RTK
Modified by jianfengli 201910

## 运行差分信号接收程序
1. cd tiev代码根目录/env/NtripClientPy
2. ./Run-ntripclient-tiev-sh.sh （可能需要指定连接设备串口或USB，修改-S 内容）
3. 若GPS差分信号接收成功，GPS接收机中间指示灯闪绿灯

## 运行IMU接收程序
1. 设置网络
点击ubuntu右上角网络连接的标志（那个信号的标志）-> 编辑链接  ->  点击以太网然后点增加，连接类型选以太网 -> 新建 -> ipv4设置  ->  方法选择成手动  -> 然后地址右边的增加　-> 地址设置成192.168.0.0（192.168.0.9~192.168.0.254都ok,　只要保证跟IMU一个网段但是不一样，即不能是192.168.0.8）-> 子网掩码设置成255.255.255.0

2. cd tiev代码根目录/src/modules/IMU_Receiver_linux_zcm
编译 运行./IMU_Receiver

3. 车速开至30km/h, 若rtk status变为1即可
