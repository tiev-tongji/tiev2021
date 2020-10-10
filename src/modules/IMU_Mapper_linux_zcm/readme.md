# IMU Mapper
## Function
采集roadmap模块
## Design
提供交互式界面，能够显示定位模块状态，并且在线对roadmap属性进行设置。
其中roadmap属性包括：id lon lat heading curvature mode speedmode laneno laneseq lanewidth，数据类型为int double double float float int int int int float
界面设计基于现有IMU_Mapper
## Implementation
采用Qt5进行界面开发，利用C++实现代码
流程：运行程序->接收LCM的NAVINFO消息->显示状态->交互式控制->建图暂停->建图完毕->筛点->保存路径
优化界面布局