#pragma once

#include <vector>

#include "linear_interpolation.h"
#include "pose.h"

namespace TiEV {

struct IdDis {
  int    id;
  double signed_dis;
};

/**
 * @class
 * @brief Math a given coordinate(X/Y) to a given path and convert it to Frenet
 * coordinate
 */
class PathMatcher {
 public:
  PathMatcher() = delete;

  /**
   * @brief Match a cartesian coordinate to a given path
   * @param path
   * @param x
   * @param y
   * @return The matched path point
   */
  static Pose MatchToPath(const std::vector<Pose>& path, const double x,
                          const double y, std::string type = "speed_planner");
  static Pose MatchToPath(const std::vector<HDMapPoint>& path, const double x,
                          const double y, std::string type = "speed_planner");

  // returned distance is positive if p is on the left hand side of the path
  template <class P1, class P2>
  static IdDis MatchToPath(const std::vector<P1>& path, const P2& p) {
    double dis_min = std::numeric_limits<double>::max();
    int    idx_min = -1;
    if (path.empty()) return {idx_min, dis_min};
    for (int i = 0; i < path.size(); ++i) {
      double dis_tmp = std::sqrt(std::pow(path[i].x - p.x, 2) +
                                 std::pow(path[i].y - p.y, 2));
      if (dis_tmp < dis_min) {
        dis_min = dis_tmp;
        idx_min = i;
      }
    }
    const auto matched_point = path[idx_min];
    Point2d    v1(std::cos(matched_point.ang), std::sin(matched_point.ang));
    Point2d    v2(p.x - matched_point.x, p.y - matched_point.y);
    bool       left_side = (v1.cross(v2) > 0);
    double     signed_dis;
    left_side == true ? signed_dis = dis_min : signed_dis = -dis_min;
    return {idx_min, signed_dis};
  }

  /**
   * @brief Match given s to a given path
   * @param path
   * @param s
   * @return The matched path point
   */
  static Pose MatchToPath(const std::vector<Pose>& path, const double s);
  /**
   * @brief Convert a cartesian coordinate to a Frenet coordinate
   * @param path
   * @param x
   * @param y
   * @return Frenet coordinate
   */
  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<Pose>& path, const double x, const double y);

 private:
  /**
   * @brief Find the projection point of given point on the given path
   *        using linear interpolation
   * @param p0 The first path point
   * @param p1 The second path point
   * @param x
   * @param y
   * @return The projection point
   */
  static Pose FindProjectionPoint(const Pose& p0, const Pose& p1,
                                  const double x, const double y,
                                  std::string type = "speed_planner");
};

}  // namespace TiEV