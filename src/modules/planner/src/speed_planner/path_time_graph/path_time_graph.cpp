#include "path_time_graph.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "linear_interpolation.h"
#include "path_matcher.h"
#include "st_point.h"
#include "tievlog.h"
namespace TiEV {
PathTimeGraph::PathTimeGraph(std::vector<Obstacle>&   obstacles,
                             const std::vector<Pose>& path,
                             const double s_start, const double s_end,
                             const double t_start, const double t_end,
                             double current_speed) {
  path_range_.first  = s_start;
  path_range_.second = s_end;
  time_range_.first  = t_start;
  time_range_.second = t_end;
  path_length_       = s_end - s_start;
  total_time_        = t_end - t_start;
  current_speed_     = current_speed;
  SetupObstacles(obstacles, path);
}

Pose PathTimeGraph::GetPointAtTime(const Obstacle& obstacle,
                                   double          time) const {
  const std::vector<Pose> points = obstacle.path;
  if (points.size() < 2) return *points.begin();
  auto comp = [](const Pose p, const int64_t time) { return p.t < time; };

  auto it_lower = std::lower_bound(points.begin(), points.end(), time, comp);

  if (it_lower == points.begin()) {
    return *points.begin();
  } else if (it_lower == points.end()) {
    return *points.rbegin();
  }
  return InterpolateUsingLinearApproximationWithT(*(it_lower - 1), *it_lower,
                                                  time);
}

Box PathTimeGraph::GetStaticBoundingBox(const Obstacle& obstacle) {
  return Box(Vec(obstacle.path.front().x, obstacle.path.front().y),
             obstacle.length, obstacle.width, obstacle.path.front().ang);
}

Box PathTimeGraph::GetDynamicBoundingBox(const Obstacle& obstacle,
                                         const Pose&     point) {
  return Box(Vec(point.x, point.y), obstacle.length, obstacle.width, point.ang);
}

SLBoundary PathTimeGraph::ComputeObstacleSLBoundary(
    const std::vector<Vec>& vertices, const std::vector<Pose>& path) {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  // LOG(WARNING) << "----Obstale----";
  for (const auto& point : vertices) {
    std::pair<double, double> sl_point =
        PathMatcher::GetPathFrenetCoordinate(path, point.x(), point.y());
    // LOG(INFO) << "vertice:[" << point.x() << " " << point.y()
    //           << "] sl_point: s=" << sl_point.first << " l=" <<
    //           sl_point.second;
    // std::cout << "sl result: " << sl_point.second << std::endl;
    start_s = std::fmin(start_s, sl_point.first);
    end_s   = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l   = std::fmax(end_l, sl_point.second);
  }

  SLBoundary sl_boundary;
  sl_boundary.set_start_s(start_s);
  sl_boundary.set_end_s(end_s);
  sl_boundary.set_start_l(start_l);
  sl_boundary.set_end_l(end_l);

  return sl_boundary;
}

void PathTimeGraph::SetupObstacles(std::vector<Obstacle>&   obstacles,
                                   const std::vector<Pose>& path) {
  for (auto& obstacle : obstacles) {
    if (obstacle.path.size() <= 1 || obstacle.path.front().v == 0) {
      SetStaticObstacle(obstacle, path);
    } else {
      SetDynamicObstacle(obstacle, path);
    }
  }
}

void PathTimeGraph::SetStaticObstacle(Obstacle&                obstacle,
                                      const std::vector<Pose>& path) {
  const Box box = GetStaticBoundingBox(obstacle);
  std::cout << "obstacle details: " << obstacle.path.front().x << ' '
            << obstacle.path.front().y << ' ' << obstacle.length << ' '
            << obstacle.width << ' ' << obstacle.path.front().ang << std::endl;
  std::cout << current_speed_ << std::endl;
  for (const auto& i : box.corners()) {
    std::cout << i.x() << ' ' << i.y() << std::endl;
  }
  // LOG(INFO) << "Static Obstacle";

  SLBoundary sl_boundary = ComputeObstacleSLBoundary(box.corners(), path);
  double     left_width  = Default_Path_Width_ * 0.6 + current_speed_ * 0.1;
  double     right_width = Default_Path_Width_ * 0.6 + current_speed_ * 0.1;
  std::cout << "finally: " << sl_boundary.start_s() << ' '
            << sl_boundary.end_s() << ' ' << sl_boundary.start_l() << ' '
            << sl_boundary.end_l() << ' ' << path_range_.first << ' '
            << path_range_.second << std::endl;
  // Out of path range
  if (sl_boundary.start_s() > path_range_.second ||
      sl_boundary.end_s() < path_range_.first ||
      sl_boundary.start_l() > left_width ||
      sl_boundary.end_l() < -right_width ||
      sl_boundary.end_s() - sl_boundary.start_s() > 10 ||
      sl_boundary.end_l() - sl_boundary.start_l() > 10) {
    return;
  }
  std::cout << "stop!" << std::endl;
  STPoint blp(sl_boundary.start_s(), 0);
  STPoint brp(sl_boundary.start_s(), total_time_);
  STPoint ulp(sl_boundary.end_s(), 0);
  STPoint urp(sl_boundary.end_s(), total_time_);

  STBoundary st_boundary(blp, brp, ulp, urp);
  st_boundary.obs_type = obstacle.type;
  st_boundaries_.emplace_back(st_boundary);
  obstacle.set_st_boundary(st_boundary);
}

void PathTimeGraph::SetDynamicObstacle(Obstacle&                obstacle,
                                       const std::vector<Pose>& path) {
  double  relative_time = time_range_.first;
  STPoint bottom_left(0, 0);
  STPoint bottom_right(0, 0);
  STPoint upper_left(0, 0);
  STPoint upper_right(0, 0);
  // LOG(INFO) << "Dynamic Obstacle";

  bool left_edge_set = false;
  while (relative_time < time_range_.second) {
    Pose       point       = GetPointAtTime(obstacle, relative_time);
    Box        box         = GetDynamicBoundingBox(obstacle, point);
    SLBoundary sl_boundary = ComputeObstacleSLBoundary(box.corners(), path);

    double left_width  = Default_Path_Width_ * 0.5;
    double right_width = Default_Path_Width_ * 0.5;

    // If obstacle is out of lane at this time
    if (sl_boundary.start_s() > path_range_.second ||
        sl_boundary.end_s() < path_range_.first ||
        sl_boundary.start_l() > left_width ||
        sl_boundary.end_l() < -right_width) {
      relative_time += Trajectory_Time_Resolution;
      continue;
    }

    if (!left_edge_set) {
      bottom_left.set_s(sl_boundary.start_s());
      bottom_left.set_t(relative_time);
      upper_left.set_s(sl_boundary.end_s());
      upper_left.set_t(relative_time);

      bottom_right.set_s(sl_boundary.start_s());
      bottom_right.set_t(relative_time);
      upper_right.set_s(sl_boundary.end_s());
      upper_right.set_t(relative_time);
      left_edge_set = true;
    } else {
      bottom_right.set_s(sl_boundary.start_s());
      bottom_right.set_t(relative_time);
      upper_right.set_s(sl_boundary.end_s());
      upper_right.set_t(relative_time);
    }
    relative_time += Trajectory_Time_Resolution;
  }
  if (left_edge_set) {
    STBoundary st_boundary(bottom_left, bottom_right, upper_left, upper_right);
    st_boundary.obs_type = obstacle.type;
    st_boundaries_.emplace_back(st_boundary);
    obstacle.set_st_boundary(st_boundary);
  }
}

}  // namespace TiEV
