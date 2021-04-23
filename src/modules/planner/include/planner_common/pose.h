#ifndef __POSE__H__
#define __POSE__H__

#include <iostream>
#include "point2d.h"
namespace TiEV {

/**
 * 定义UTM坐标点
 */
struct UtmPosition {
  double utm_x;    // rad
  double utm_y;    // rad
  double heading;  // rad
  UtmPosition(double utm_x_ = 0, double utm_y_ = 0, double heading_ = 0)
      : utm_x(utm_x_), utm_y(utm_y_), heading(heading_){};
  friend std::ostream& operator<<(std::ostream& out,
                                  const UtmPosition& position) {
    out << "UtmPosition:{" << position.utm_x << ", " << position.utm_y << ", "
        << position.heading << "}";
    return out;
  }
};

// longititude and latitude
struct LonLatPosition {
  double lon;
  double lat;
  double heading;
  LonLatPosition(double lon_ = 0, double lat_ = 0, double heading_ = 0)
      : lon(lon_), lat(lat_), heading(heading_){};
  friend std::ostream& operator<<(std::ostream& out,
                                  const LonLatPosition& position) {
    out << "LonLatPosition:{" << position.lon << ", " << position.lat << ", "
        << position.heading << "}";
    return out;
  }
};

/**
 * 定义轨迹
 */
struct Pose : public Point2d {
  UtmPosition utm_position;
  double ang;  // rad
  double a;
  double k;  // curvature 1/m
  double v;  // velocity m/s
  double s;  // lenth meter
  double t;  // time second
  bool backward;
  Pose(double x_ = -1, double y_ = -1, double ang_ = 0, double k_ = 0,
       double v_ = 0, double a_ = 0, double s_ = 0, double t_ = 0,
       bool backward_ = false)
      : Point2d(x_, y_),
        ang(ang_),
        k(k_),
        v(v_),
        a(a_),
        s(s_),
        t(t_),
        backward(backward_),
        utm_position() {}

  inline Point2d getDirectionVec() const { return Point2d(cos(ang), sin(ang)); }

  inline Pose getLateralPose(const double l) const {
    double _tmp = l / GRID_RESOLUTION;
    double dx = -sin(ang) * _tmp;
    double dy = cos(ang) * _tmp;
    Pose res_point = *this;
    res_point.x = x + dx;
    res_point.y = y + dy;
    return res_point;
  }

  friend std::ostream& operator<<(std::ostream& out, const Pose& pose) {
    out << "Pose:{utm position:(" << pose.utm_position.utm_x << ", "
        << pose.utm_position.utm_y << ", " << pose.utm_position.heading
        << ") x=" << pose.x << ", y=" << pose.y << ", ang=" << pose.ang
        << ", k=" << pose.k << ", v=" << pose.v << " , a=" << pose.a
        << ", s=" << pose.s << ", t=" << pose.t
        << ", backward=" << pose.backward << "}";
    return out;
  }

  inline void updateGlobalCoordinate(const Pose& standard_point) {
    double stdh = standard_point.utm_position.heading - standard_point.ang;
    double sinstdh = sin(stdh), cosstdh = cos(stdh);
    double px = x - standard_point.x;
    double py = y - standard_point.y;
    double qx = (px * cosstdh - py * sinstdh);
    double qy = (px * sinstdh + py * cosstdh);
    utm_position.utm_x =
        qx * GRID_RESOLUTION + standard_point.utm_position.utm_x;
    utm_position.utm_y =
        qy * GRID_RESOLUTION + standard_point.utm_position.utm_y;
    double hd = standard_point.utm_position.heading + ang - standard_point.ang;
    while (hd > PI) hd -= 2 * PI;
    while (hd <= -PI) hd += 2 * PI;
    utm_position.heading = hd;
  }

  inline double cosDeltaAngle(const Pose& other_pose) {
    double cross = (*this).dot(other_pose);
    double cos = cross / ((*this).len() * other_pose.len());
    return cos;
  }

  void updateLocalCoordinate(const Pose& standard_point) {
    double stdh = standard_point.ang - standard_point.utm_position.heading;
    double qx = (utm_position.utm_x - standard_point.utm_position.utm_x) /
                GRID_RESOLUTION;
    double qy = (utm_position.utm_y - standard_point.utm_position.utm_y) /
                GRID_RESOLUTION;
    double sinstdh = sin(stdh), cosstdh = cos(stdh);
    double px = (qx * cosstdh - qy * sinstdh);
    double py = (qx * sinstdh + qy * cosstdh);
    x = standard_point.x + px;
    y = standard_point.y + py;

    double ag = utm_position.heading - standard_point.utm_position.heading +
                standard_point.ang;
    while (ag > PI) ag -= 2 * PI;
    while (ag <= -PI) ag += 2 * PI;
    ang = ag;
  }
};

/**
 * 定义语义地图上的路径点
 */
enum HDMapEvent {
  NONE,
  ENTRY_INTERSECTION,
  EXIT_INTERSECTION,
  STOP,
  CHANGE_HDMAP
};
enum HDMapMode {
  NORMAL,
  INTERSECTION_SOLID,
  INTERSECTION,
  PARKING,
  CHANGE,
  IN_PARK,
  UNKNOWN_MODE
};
enum HDMapSpeed {
  BACK_SPEED,
  STOP_SPEED,
  VERY_LOW,
  LOW,
  MIDDLE,
  HIGH,
  VERY_HIGH
};
enum RoadDirection {
  LEFT = 4,
  STRAIGHT = 2,
  RIGHT = 1,
  UTURN = 8
};  //二进制表示0000，最高位表示uturn,剩下为左直右
enum BlockType {
  BlockNone,
  BlockRight = 1,
  BlockLeft = 2,
  BlockAll = 3
};  //二进制表示00，1表示封闭
struct HDMapPoint : public Pose {
  HDMapEvent event;
  HDMapMode mode;
  HDMapSpeed speed_mode;
  int lane_num;
  int lane_seq;
  double lane_width;
  RoadDirection direction;
  BlockType block_type;
  HDMapPoint(double utm_x_ = 0, double utm_y_ = 0, double heading_ = 0,
             double k_ = 0, HDMapMode mode_ = HDMapMode::NORMAL,
             HDMapSpeed speed_mode_ = HDMapSpeed::MIDDLE,
             HDMapEvent event_ = HDMapEvent::NONE,
             BlockType block_type_ = BlockType::BlockAll, int lane_num_ = 0,
             int lane_seq_ = 0, double lane_width_ = 0,
             RoadDirection direction_ = RoadDirection::STRAIGHT, double x_ = 0,
             double y_ = 0, double ang_ = 0)
      : Pose(x_, y_, ang_, k_) {
    event = event_;
    mode = mode_;
    speed_mode = speed_mode_;
    lane_num = lane_num_;
    lane_seq = lane_seq_;
    lane_width = lane_width_;
    direction = direction_;
    block_type = block_type_;
    utm_position = UtmPosition(utm_x_, utm_y_, heading_);
  }

  inline Point2d toPoint2d() const { return Point2d(x, y); }

  friend std::ostream& operator<<(std::ostream& out, const HDMapPoint& point) {
    out << "HDMapPoint:{utm position:(" << point.utm_position.utm_x << ", "
        << point.utm_position.utm_y << ", " << point.utm_position.heading
        << ") curve=" << point.k << " mode=" << point.mode
        << " speed_mode=" << point.speed_mode << " event=" << point.event
        << " block_type=" << point.block_type << " lane_num=" << point.lane_num
        << " lane_seq=" << point.lane_seq << " lane_width=" << point.lane_width
        << " direction=" << point.direction
        << " block type=" << point.block_type << " x=" << point.x
        << ", y=" << point.y << ", ang=" << point.ang << ", k=" << point.k
        << ", v=" << point.v << ", s=" << point.s << ", t=" << point.t << "}";
    return out;
  }
};

/**
 * 参考路生成的车道线点
 * 每个点得知道是否需要当成虚拟墙，从而好生成决策地图
 */
enum LineType { UNKNOWN_LINE, DASH, SOLID, BOUNDARY };
struct LinePoint : public Point2d {
  LineType type;
  LinePoint(double x_ = -1, double y_ = -1,
            LineType type_ = LineType::UNKNOWN_LINE)
      : Point2d(x_, y_), type(type_){};
  friend std::ostream& operator<<(std::ostream& out, const LinePoint& point) {
    out << "LinePoint:{x=" << point.x << " y=" << point.y
        << " type=" << point.type << "}";
    return out;
  }
};

}  // namespace TiEV

#endif  //!__POSE__H__
