## STEP1 BUILD S-T GRAPH

### 1. class definition

### PathPoint // 路径点

```C++
int64_t t;
double x;
double y;
double theta;
double kappa;
float v;
float a;
```

### Obstacle // 障碍物（动态）

```C++
int16_t id;
uint8_t type;
double x;
double y;
float v;
float theta;
float width;
float length;
```

### STPoint // s-t图上的点

```c++
double s;
double t;
```

### LineSegment // s-t图上的线段

```C++
STPoint start_;
STPoint end_;
double length_;
```

### STBoundary // 表示s-t图上的障碍物占用区域(ST图上的多边形)

```C++
std::vector<STPoint> points_; // 顶点集合
int num_points;
std::vector<LineSegment> line_segments_; // 边集合
STPoint bottom_left_point_;
STPoint bottom_right_point_;
STPoint upper_left_point_;
STPoint upper_right_point_;
```

### Vec // 笛卡尔坐标系向量

```C++
class Vec {
    double x_;
    double y_;
}
```

### Box // 障碍物BOX碰撞模型

```C++
class Box {
    Vec center_;
    double length_;
    double width_;
    double heading_;
    double cos_heading_;
    double sin_heading_;
}
```

### SLCord // Frenet坐标系坐标

### SLBoundary // Frenet坐标系下障碍物占用区域

```C++
class SLBoundary {
    double start_s_;
    double end_s_;
    double start_l_;
    double end_l_;
}
```



### 2. ToDoList

将障碍物在s-t图上进行投影：

* SetDynamicObstacles(obstacle, path)
  - GetPointAtTime(obstacle, time): PathPoint**(Done)**
    - InterpolateUsingLineaerApproximation()
  - GetBoundingBox(PathPoint): Box**(Done)**
  - ComputeObstacleBoundary(Box, path): SLBoundary**(Done)**
    * GetPathFrenetCoordinate(path, x, y): SLPoint



## STEP2: DP SPEED OPTIMIZER

### 1. Class definitions

class SpeedOptimizer:

* Class GriddedPathTimeGraph definition
  - vector<vector\<StGraphPoint\> > // cost table
  - PathTimeGraph // STBoundary and some other stuffs, **already implemented**
  
  - PathPoint // initial status
  - vector<Obstacle*> // obstacle list
  - double // unit_s
  - double // unit_t
  
* Configurations and parameters // **To be decided...**

  - DpStConfig // reference: apollo/modules/planning/proto/dp_st_speed_config.proto
  - VehicleParam 
  - DpStCost

* Search(SpeedData* speed_data) // dynamic programming method

  - IsPointInBoundary()
    - 利用与四边形顶点构成的向量叉乘结果作为判断依据
  
  * InitCostTable()
    * 初始化S-T网格
  * CalculateTotalCost() **(TODO: 怎么利用给定约束(v, a, jerk...)计算网格的Cost)**
    - 逐列(t)拓展行(s)的范围进行计算权值
  * RetrieveSpeedProfile()
    - 获取最佳(totalcost最小)的speed_profile

### 2. Cost calculation

* GetObstacleCost(c, r): 对于当前S-T图中的每一个STBoundary，根据当前网格(c, r)与其距离($\triangle s$)计算网格(c, r)的障碍物权值$C_{obs}$。总障碍物权值为$\sum C_{obs}$。考虑以下四种情形：
  * (c, r)与st_boundary在时间上无交互 - $C_{obs}=0$
  * (c, r)在st_boundary当中(IsPointInBoundary(c, r) == true)) - $C_{obs}=Inf$
  * (c, r)在st_boundary的下方(s < s_lower)，首先确定跟车距离$follow\_distance=V_{obs}\times safe\_time\_buffer$
    - 若$s+follow\_distance<s\_lower$ - $C_{obs}=0$
    - 若$s+follow\_distance>s\_lower$ - $C_{obs}=weight\times \triangle s^2$
  * (c, r)在st_boundary的上方(s>s_upper)
    - 若$s>s\_upper+safe\_distance$ - $C_{obs}=0$
    - 若$s<s\_upper+safe\_distance$ - $C_{obs}=weight\times \triangle s^2$
* GetSpeedCost(): 以上限作为参考速度 **(待确定)**
* GetAccelCost(): 加速度的平方作为Cost
  * $a>0, C_{acc}=acc\_weight\times a^2$
  * $a<0,C_{acc}=dec\_weight\times a^2$
* GetJerkCost(): 加加速度的平方作为Cost
  * $jerk>0,C_{jerk}=positive\_jerk\_weight\times jerk^2$
  * $jerk<0, C_{jerk}=negative\_jerk\_weight\times jerk^2$

