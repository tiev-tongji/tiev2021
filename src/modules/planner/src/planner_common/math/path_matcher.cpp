#include "path_matcher.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "tievlog.h"

namespace TiEV {

Pose PathMatcher::MatchToPath(const std::vector<Pose>& path, const double x,
                              const double y, std::string type) {
  auto func_dis_square = [](const Pose& point, const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };

  double      dis_min = func_dis_square(path.front(), x, y);
  std::size_t idx_min = 0;

  for (std::size_t i = 1; i < path.size(); i++) {
    double dis_tmp = func_dis_square(path[i], x, y);
    if (dis_tmp < dis_min) {
      dis_min = dis_tmp;
      idx_min = i;
    }
  }

  std::size_t idx_start = (idx_min == 0) ? idx_min : idx_min - 1;
  std::size_t idx_end   = (idx_min + 1 == path.size()) ? idx_min : idx_min + 1;

  // if idx_min = 0, find projection point between path[0] and path[1], rather
  // than return path[0]
  if (idx_start == idx_end) {
    return path[idx_start];
  } else {
    return FindProjectionPoint(path[idx_start], path[idx_end], x, y, type);
  }
  // more than 2 points
  //   const auto& projectionDis = [](const Pose& p0, const Pose& p1, double x,
  //                                  double y) {
  //     double v0x = (x - p0.x);
  //     double v0y = (y - p0.y);

  //     double v1x = (p1.x - p0.x);
  //     double v1y = (p1.y - p0.y);

  //     double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  //     double dot     = v0x * v1x + v0y * v1y;

  //     return dot / v1_norm;
  //   };
  //   const auto& sqrtDistance = [&](const Pose& p0, const Pose& p1) {
  //     return std::sqrt(func_dis_square(p0, p1.x, p1.y));
  //   };
  //   const auto pp0 = projectionDis(path[idx_start], path[idx_start + 1], x,
  //   y); const auto pp1 = projectionDis(path[idx_start + 1], path[idx_end], x,
  //   y);
}

Pose PathMatcher::MatchToPath(const std::vector<HDMapPoint>& path,
                              const double x, const double y,
                              std::string type) {
  auto func_dis_square = [](const HDMapPoint& point, const double x,
                            const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };

  double      dis_min = func_dis_square(path.front(), x, y);
  std::size_t idx_min = 0;

  for (std::size_t i = 1; i < path.size(); i++) {
    double dis_tmp = func_dis_square(path[i], x, y);
    if (dis_tmp < dis_min) {
      dis_min = dis_tmp;
      idx_min = i;
    }
  }

  std::size_t idx_start = (idx_min == 0) ? idx_min : idx_min - 1;
  std::size_t idx_end   = (idx_min + 1 == path.size()) ? idx_min : idx_min + 1;

  if (idx_start == idx_end) {
    return path[idx_start];
  } else {
    return FindProjectionPoint(path[idx_start], path[idx_end], x, y, type);
  }
}

Pose PathMatcher::MatchToPath(const std::vector<Pose>& path, const double s) {
  auto comp = [](const Pose& point, const double s) { return point.s < s; };

  auto it_lower = std::lower_bound(path.begin(), path.end(), s, comp);
  if (it_lower == path.begin())
    return path.front();
  else if (it_lower == path.end())
    return path.back();

  return InterpolateUsingLinearApproximationWithS(*(it_lower - 1), *it_lower,
                                                  s);
}

Pose PathMatcher::FindProjectionPoint(const Pose& p0, const Pose& p1,
                                      const double x, const double y,
                                      std::string type) {
  double v0x = (x - p0.x);
  double v0y = (y - p0.y);

  double v1x = (p1.x - p0.x);
  double v1y = (p1.y - p0.y);

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot     = v0x * v1x + v0y * v1y;

  double delta_s = dot / v1_norm * GRID_RESOLUTION;
  // in speed planner, all s in pose is in unit m
  // in lattice planner, all s in pose is in unit GRID
  if (type == "lattice_planner") {
    delta_s /= GRID_RESOLUTION;
  }
  return InterpolateUsingLinearApproximationWithS(p0, p1, p0.s + delta_s);
}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(
    const std::vector<Pose>& path, const double x, const double y) {
  Pose matched_path_point =
      MatchToPath(path, x, y, "speed_planner");  // return pose.xy is gird
  double rtheta  = matched_path_point.ang;
  double rx      = matched_path_point.x;
  double ry      = matched_path_point.y;
  double delta_x = x - rx;
  double delta_y = y - ry;

  // The original coord represents gridded cells
  // Here we need to transfer it to actual distance
  // only for speed_planner
  double dis_x = delta_x * GRID_RESOLUTION;
  double dis_y = delta_y * GRID_RESOLUTION;

  // Check the sign of lateral value using vector crossing
  double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
  std::pair<double, double> coord;
  coord.first  = matched_path_point.s;
  coord.second = std::copysign(std::hypot(dis_x, dis_y), side);
  return coord;
}
}  // namespace TiEV
