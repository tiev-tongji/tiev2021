# TiEV 坐标系说明
## 车身局部坐标系
+ 车身坐标系原点为车辆前轴中心，车辆正前方为y轴正方向，车辆右侧为x轴正方向，以米为单位。局部坐标系的角度定位为x轴正方向（向右）为0度，逆时针为正，范围为-pi到pi
+ Planner模块为可视化便利，采用了车辆正后方向为x轴正方向，车辆右侧为y轴正方向坐标系。
## 全局坐标系
### 经纬度
经纬度采用wgs84坐标系，epsg4326。
### 平面直角坐标系
平面直角坐标系为utm投影，投影区域为51（华东上海苏州）
It should be very careful that USING UTM coord and mGPSHeading (raw heading) will suffer from grid convergence error.
This cause is the north of mGPSHeading is the true north while the north of UTM is grid north.
formulas are existed for calculation, such as:
convergence  = atan2(tan(lon - central meridian)*sin(lan))
However, the correct formula is to be verified.

------------------------Junqiao 20171119
