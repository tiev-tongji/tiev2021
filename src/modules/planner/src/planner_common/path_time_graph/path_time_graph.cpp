#include "path_time_graph.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "cartesian_frenet_conversion.h"
#include "lattice_planner_params.h"
#include "linear_interpolation.h"
#include "path_matcher.h"
#include "st_point.h"
#include "tievlog.h"

namespace TiEV {
PathTimeGraph::PathTimeGraph(std::vector<Obstacle>&   obstacles,
                             const std::vector<Pose>& path,
                             const double s_start, const double s_end,
                             const double t_start, const double t_end,
                             double current_speed, std::string type)
    : obstacles_(obstacles) {
  path_range_.first          = s_start;
  path_range_.second         = s_end;
  time_range_.first          = t_start;
  time_range_.second         = t_end;
  path_length_               = s_end - s_start;
  total_time_                = t_end - t_start;
  current_speed_             = current_speed;
  Default_Path_Width_        = 1.0 * CAR_WIDTH / GRID_RESOLUTION;  // unit grid
  Trajectory_Time_Resolution = 0.05;  // Temporarily set
  type_                      = type;
  SetupObstacles(obstacles, path);
}

PathTimeGraph::PathTimeGraph(std::vector<Obstacle>&   obstacles,
                             const std::vector<Pose>& path,
                             const double s_start, const double s_end,
                             const double t_start, const double t_end,
                             double current_speed)
    : obstacles_(obstacles) {
  path_range_.first          = s_start;
  path_range_.second         = s_end;
  time_range_.first          = t_start;
  time_range_.second         = t_end;
  path_length_               = s_end - s_start;
  total_time_                = t_end - t_start;
  current_speed_             = current_speed;
  Default_Path_Width_        = 1.0 * CAR_WIDTH;  // unit m
  Trajectory_Time_Resolution = 0.05;             // Temporarily set
  type_                      = "speed_planner";
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
    std::pair<double, double> sl_point;
    if (type_ == "speed_planner") {
      sl_point =
          PathMatcher::GetPathFrenetCoordinate(path, point.x(), point.y());
    } else {
      Pose   matched_point;
      double x = point.x(), y = point.y();
      auto   func_dis_square = [](const Pose& point, const double x,
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
      std::size_t idx_end =
          (idx_min + 1 == path.size()) ? idx_min : idx_min + 1;

      // a rough estimation. don't interpolate for now
      matched_point = path[idx_start];
      double s_val, d_val;
      CartesianFrenetConverter::cartesian_to_frenet(
          matched_point.s, matched_point.x, matched_point.y, matched_point.ang,
          point.x(), point.y(), &s_val, &d_val);
      sl_point.first  = s_val;
      sl_point.second = d_val;
    }

    // LOG(INFO) << "vertice:[" << point.x() << " " << point.y()
    //           << "] sl_point: s=" << sl_point.first << " l=" <<
    //           sl_point.second;
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
  // std::cout << "box: " << std::endl;
  // std::cout << box << std::endl;
  SLBoundary sl_boundary = ComputeObstacleSLBoundary(box.corners(), path);
  double     left_width  = Default_Path_Width_ * 0.6 + current_speed_ * 0.1;
  double     right_width = Default_Path_Width_ * 0.6 + current_speed_ * 0.1;
  // Out of lane
  if (sl_boundary.start_s() > path_range_.second ||
      sl_boundary.end_s() < path_range_.first ||
      sl_boundary.start_l() > left_width ||
      sl_boundary.end_l() < -right_width) {
    return;
  }
  STPoint blp(sl_boundary.start_s(), 0);
  STPoint brp(sl_boundary.start_s(), total_time_);
  STPoint ulp(sl_boundary.end_s(), 0);
  STPoint urp(sl_boundary.end_s(), total_time_);

  STBoundary st_boundary(blp, brp, ulp, urp);
  st_boundary.obs_type = obstacle.type;
  st_boundary.set_id(obstacle.id);
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
    st_boundary.set_id(obstacle.id);
    st_boundaries_.emplace_back(st_boundary);
    obstacle.set_st_boundary(st_boundary);
  }
}

std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  // ACHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : st_boundaries_) {
    // std::cout << "not sure if path_time obstacles_ is st_boundries "
    //           << std::endl;
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution) {
    intervals.push_back(GetPathBlockingIntervals(t));
  }
  return intervals;
}

std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const int& obstacle_id, const double s_dist) const {
  // ACHECK(t_min_density > 0.0);
  std::vector<STPoint> pt_pairs;
  if (st_boundaries_.empty()) return pt_pairs;

  STBoundary pt_obstacle;
  bool       found_obs = false;
  for (const auto& obs : st_boundaries_) {
    if (obs.id() == obstacle_id) {
      pt_obstacle = obs;
      found_obs   = true;
    }
  }
  if (!found_obs || FLAGS_num_follow_samples_t <= 0) return pt_pairs;

  double s0 = 0.0;
  double s1 = 0.0;

  double t0 = 0.0;
  double t1 = 0.0;
  if (s_dist > 0.0) {
    s0 = pt_obstacle.upper_left_point().s();
    s1 = pt_obstacle.upper_right_point().s();

    t0 = pt_obstacle.upper_left_point().t();
    t1 = pt_obstacle.upper_right_point().t();
  } else {
    s0 = pt_obstacle.bottom_left_point().s();
    s1 = pt_obstacle.bottom_right_point().s();

    t0 = pt_obstacle.bottom_left_point().t();
    t1 = pt_obstacle.bottom_right_point().t();
  }

  double time_origin = (t0 + t1) / 2;
  double time_gap    = t1 - t0;
  // ACHECK(time_gap > -FLAGS_numerical_epsilon);
  time_gap = std::fabs(time_gap);

  size_t num_sections = FLAGS_num_follow_samples_t;
  double t_interval   = time_gap / static_cast<double>(num_sections);

  for (size_t i = 0; i < num_sections; ++i) {
    double t = t_interval * static_cast<double>(i) + time_origin;
    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}

}  // namespace TiEV
