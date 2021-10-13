#ifndef __TIEV_UTILS__H__
#define __TIEV_UTILS__H__
#include <stdint.h>

#include <ctime>
#include <mutex>
#include <vector>

#include "pose.h"

namespace TiEV {
//两点之间的距离
double point2PointDis(const Point2d& p1, const Point2d& p2);
//两点之间的平方距离
double point2PointSqrDis(const Point2d& p1, const Point2d& p2);
//向一个向量垂直方向偏移距离d得到新的点，左正右负
Point2d offsetPoint(const Point2d& p, const double dis);
//点是否在局部地图内
bool isInLocalMap(const Point2d& p);
// 获取横向偏移店
Pose getLateralPoint(const Pose& p, double lateral_distance);

void normalizeAngle(double& rad_angle);

//点到一条路径的最短距离
template <class T, class Y>
double point2LineDis(const T& p, const std::vector<Y>& path) {
  if (path.empty()) return 0;
  int    shortest_point_index = shortestPointIndex(p, path);
  double min_dis =
      1e9;  //              = point2PointDis(p, path[shortest_point_index]);
  Point2d vec_p  = p - path[shortest_point_index];
  int     pre_i  = shortest_point_index - 1;
  int     next_i = shortest_point_index + 1;
  if (pre_i >= 0) {
    Point2d vec_pre = path[pre_i] - path[shortest_point_index];
    if (vec_p.dot(vec_pre) > 0)
      min_dis = vec_p.cross(vec_pre) / vec_pre.len();
    else
      min_dis = copysign(point2PointDis(p, path[shortest_point_index]),
                         vec_p.cross(vec_pre));
  }
  if (next_i < path.size()) {
    Point2d vec_next = path[next_i] - path[shortest_point_index];
    double  dis;
    if (vec_p.dot(vec_next) > 0) {
      dis = vec_next.cross(vec_p) / vec_next.len();
    } else
      dis = copysign(point2PointDis(p, path[shortest_point_index]),
                     vec_next.cross(vec_p));
    if (fabs(dis) < fabs(min_dis)) min_dis = dis;
  }
  return min_dis;
}
// 路径上到某个点距离最短的点的索引
template <class T, class Y>
int shortestPointIndex(const T& p, const std::vector<Y>& path) {
  if (path.empty()) return -1;
  double min_dis              = point2PointSqrDis(p, path.front());
  int    shortest_point_index = 0;
  for (int i = 1; i < path.size(); ++i) {
    double sqr_dis = point2PointSqrDis(p, path[i]);
    if (sqr_dis >= min_dis) continue;
    min_dis              = sqr_dis;
    shortest_point_index = i;
  }
  return shortest_point_index;
}

template <class T>
void lineInterpolation(std::vector<T>& line, double min_step = 2) {
  if (line.size() < 4) return;
  std::vector<T> new_line;
  int            pre = 1;
  new_line.push_back(line.front());
  for (int i = 2; i < line.size() - 1; ++i) {
    T       ppp  = line[pre - 1];
    T       pp   = line[pre];
    T       np   = line[i];
    T       nnp  = line[i + 1];
    Point2d pvec = pp - ppp;
    Point2d vec  = np - pp;
    Point2d nvec = nnp - np;
    if (vec.len() < min_step) continue;
    double a = nvec.dot(pvec);
    a /= nvec.len() * pvec.len();
    if (vec.len() >= min_step * 2 && a > cos(PI / 6)) {
      int num = vec.len() / min_step;
      for (int k = num; k > 0; --k) {
        Point2d off_vec = vec * (double(k) / (num + 1));
        T       ip;
        ip.x = np.x - off_vec.x;
        ip.y = np.y - off_vec.y;
        if (ip.in_map()) new_line.push_back(ip);
      }
    }
    pre = i;
    new_line.push_back(np);
  }
  line = new_line;
}

//从nature中移来的，时间函数
std::tm*    gettm(int64_t timestamp);
std::time_t getTimeStamp();

// (k * curvature + b) * (v ^ 2) == 1
// https://sm.ms/image/YUv7qXxGQKi9aH3
constexpr double KV_LINE_K         = 0.9808;
constexpr double KV_LINE_B         = -0.0031;
constexpr double CAR_MAX_CURVATURE = 0.181818182;  // 1/m

inline double max_curvature_under_velocity(double velocity_m_s) {
  double curvature =
      (1.0 / (velocity_m_s * velocity_m_s) - KV_LINE_B) / KV_LINE_K;
  return std::max(std::min(curvature, CAR_MAX_CURVATURE), 0.0);
}

inline double max_velocity_for_curvature(double curvature_1_m) {
  return std::sqrt(
      1.0 /
      std::max(1e-8, KV_LINE_K * std::max(curvature_1_m, 0.0) + KV_LINE_B));
}

// we assume the steering wheel can rotate PI every 1.5 seconds
// at its maximum angular speed.
constexpr double MAX_STEERING_WHEEL_ROTATE_SPEED = M_PI / 1.5;         // ang/s
constexpr double MAX_STEERING_WHEEL_ANGLE        = 2 * M_PI + M_PI_2;  // ang

inline double max_sigma_under_velocity(double velocity_m_s) {
  return CAR_MAX_CURVATURE /
         (std::max(fabs(velocity_m_s), 5.0 / 3.6) *
          (MAX_STEERING_WHEEL_ANGLE / MAX_STEERING_WHEEL_ROTATE_SPEED));
}
}  // namespace TiEV

#endif  //!__TIEV_UTILS__H__