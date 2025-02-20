#ifndef __POSE__H__
#define __POSE__H__

#include <iostream>
#include <vector>

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
  friend std::ostream& operator<<(std::ostream&      out,
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
  friend std::ostream& operator<<(std::ostream&         out,
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
  double      ang;  // rad
  double      k;    // curvature 1/m
  double      dk;   // derivative of curvature
  double      v;    // velocity m/s
  double      a;
  double      s;  // lenth meter
  double      t;  // time second
  bool        backward = false;
  bool        passed   = false;
  UtmPosition utm_position;
  Pose(double x_ = 0, double y_ = 0, double ang_ = 0, double k_ = 0,
       double v_ = 0, double a_ = 0, double s_ = 0, double t_ = 0,
       bool backward_ = false, UtmPosition utm_position_ = UtmPosition())
      : Point2d(x_, y_),
        ang(ang_),
        k(k_),
        v(v_),
        a(a_),
        s(s_),
        t(t_),
        backward(backward_),
        utm_position(utm_position_) {}

  inline void set_x(double x_) { x = x_; }
  inline void set_y(double y_) { y = y_; }
  inline void set_s(double s_) { s = s_; }
  inline void set_theta(double ang_) { ang = ang_; }
  inline void set_kappa(double kappa_) { k = kappa_; }
  inline void set_dkappa(double dkappa_) { dk = dkappa_; }
  inline void set_v(double v_) { v = v_; }
  inline void set_a(double a_) { a = a_; }
  inline void set_t(double t_) { t = t_; }
  inline void set_utm_position(UtmPosition utm_position_) {
    utm_position = utm_position_;
  }

  inline Point2d getDirectionVec() const { return Point2d(cos(ang), sin(ang)); }

  inline Point2d getNormalVec() const { return Point2d(-sin(ang), cos(ang)); }

  inline void offset(const double l) {
    const double _l         = l / GRID_RESOLUTION;
    const auto&  offset_vec = getNormalVec() * _l;

    x += offset_vec.x;
    y += offset_vec.y;
  };

  inline Pose getLateralPose(const double l) const {
    // left positive
    double _tmp      = l / GRID_RESOLUTION;
    double dx        = -sin(ang) * _tmp;
    double dy        = cos(ang) * _tmp;
    Pose   res_point = *this;
    res_point.x      = x + dx;
    res_point.y      = y + dy;
    if (res_point.k != 0.0) {
      res_point.k = 1 / (1 / res_point.k - l);
    }
    return res_point;
  }

  friend std::ostream& operator<<(std::ostream& out, const Pose& pose) {
    out << "Pose:{utm position:(" << pose.utm_position.utm_x << ", "
        << pose.utm_position.utm_y << ", " << pose.utm_position.heading
        << ") \tx=" << pose.x << ", y=" << pose.y << ", ang=" << pose.ang
        << ", k=" << pose.k << ", v=" << pose.v << " , a=" << pose.a
        << ", \ts=" << pose.s << ", t=" << pose.t
        << ", backward=" << pose.backward << "}" << std::endl;
    return out;
  }

  inline void updateGlobalCoordinate(const Pose& standard_point) {
    double stdh    = standard_point.utm_position.heading - standard_point.ang;
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

  inline const double deltaAngle(const Pose& other_pose) const {
    double ang1 = this->ang;
    double ang2 = other_pose.ang;
    while (ang1 > 2 * PI) ang1 -= 2 * PI;
    while (ang1 <= 0) ang1 += 2 * PI;
    while (ang2 > 2 * PI) ang2 -= 2 * PI;
    while (ang2 <= 0) ang2 += 2 * PI;
    double cos = fabs(ang1 - ang2);
    return cos;
  }

  void updateLocalCoordinate(const Pose& standard_point) {
    double stdh = standard_point.ang - standard_point.utm_position.heading;
    double qx   = (utm_position.utm_x - standard_point.utm_position.utm_x) /
                GRID_RESOLUTION;
    double qy = (utm_position.utm_y - standard_point.utm_position.utm_y) /
                GRID_RESOLUTION;
    double sinstdh = sin(stdh), cosstdh = cos(stdh);
    double px = (qx * cosstdh - qy * sinstdh);
    double py = (qx * sinstdh + qy * cosstdh);
    x         = standard_point.x + px;
    y         = standard_point.y + py;

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
  LEFT     = 4,
  STRAIGHT = 2,
  RIGHT    = 1,
  UTURN    = 8
};  //二进制表示0000，最高位表示uturn,剩下为左直右
enum BlockType {
  BlockNone,
  BlockRight = 1,
  BlockLeft  = 2,
  BlockAll   = 3
};  //二进制表示00，1表示封闭

struct LaneCenterPoint : public Point2d {
  // lane center with priority
  bool   have_priority = true;
  double accumulate_dis_with_priority;
  bool   accumulate_by_static_obs = false;
  LaneCenterPoint(const double x_ = 0, const double y_ = 0,
                  const bool   have_priority_                = true,
                  const double accumulate_dis_with_priority_ = 0.0,
                  const bool   accumulate_by_static_obs_     = false)
      : Point2d(x_, y_),
        have_priority(have_priority_),
        accumulate_dis_with_priority(accumulate_dis_with_priority_),
        accumulate_by_static_obs(accumulate_by_static_obs_) {}

  friend std::ostream& operator<<(std::ostream&          out,
                                  const LaneCenterPoint& point) {
    out << "LaneCenterPoint: {x=" << point.x << " y=" << point.y
        << " priority=" << point.have_priority
        << " accumu=" << point.accumulate_dis_with_priority
        << " accumu by static=" << point.accumulate_by_static_obs;
    return out;
  }
};

struct HDMapPoint : public Pose {
  LonLatPosition lon_lat_position;
  HDMapEvent     event;
  HDMapMode      mode;
  HDMapSpeed     speed_mode;
  int            lane_num;
  int            lane_seq;
  double         lane_width;
  RoadDirection  direction;
  BlockType      block_type;
  // for lateral lane center
  std::vector<LaneCenterPoint> neighbors;
  HDMapPoint(double lon = 0, double lat = 0, double utm_x_ = 0,
             double utm_y_ = 0, double heading_ = 0, double k_ = 0,
             HDMapMode  mode_       = HDMapMode::NORMAL,
             HDMapSpeed speed_mode_ = HDMapSpeed::MIDDLE,
             HDMapEvent event_      = HDMapEvent::NONE,
             BlockType block_type_ = BlockType::BlockAll, int lane_num_ = 0,
             int lane_seq_ = 0, double lane_width_ = 0,
             RoadDirection direction_ = RoadDirection::STRAIGHT, double x_ = 0,
             double y_ = 0, double ang_ = 0)
      : Pose(x_, y_, ang_, k_) {
    event            = event_;
    mode             = mode_;
    speed_mode       = speed_mode_;
    lane_num         = lane_num_;
    lane_seq         = lane_seq_;
    lane_width       = lane_width_;
    direction        = direction_;
    block_type       = block_type_;
    utm_position     = UtmPosition(utm_x_, utm_y_, heading_);
    lon_lat_position = LonLatPosition(lon, lat, heading_);
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
