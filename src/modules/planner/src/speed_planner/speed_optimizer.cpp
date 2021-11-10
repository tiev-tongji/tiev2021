#include "speed_optimizer.h"

#include <ctime>
#include <fstream>
#include <iostream>

namespace TiEV {

SpeedPath SpeedOptimizer::RunSpeedOptimizer(
    const std::vector<DynamicObj>& obstacle_list, std::vector<Pose>& trajectory,
    const std::vector<std::pair<double, double>>& speed_limit,
    double total_path_length, double current_speed) {
  // double                                 max_acceleration = 2.5;
  double                                 max_deceleration = -1.0;
  double                                 init_v = trajectory.front().v;
  std::vector<std::pair<double, double>> modified_speed_limit;
  for (const auto& s_v : speed_limit) {
    double s = s_v.first;
    double v = s_v.second;
    if (v < init_v) {
      double min_v = sqrt(max(0.0, init_v * init_v + 2 * max_deceleration * s));
      v            = max(v, min_v);
    }
    modified_speed_limit.emplace_back(s, v);
  }

  SpeedPath speed_path;
  if (trajectory.empty()) {
    speed_path.success = false;
    return speed_path;
  }
  vector<Obstacle> obj_list;
  for (const auto& obstacle : obstacle_list) {
    Obstacle obj(obstacle);

    // Specially take care of pedestrians for safety's sake
    if (obj.type == 2) {  // pedestrian
      // TODO: Need testing in practice
      obj.width  = 1;
      obj.length = 1;
    }
    obj_list.emplace_back(obj);
  }
  const double   TOTAL_TIME = 5.0;
  SpeedOptimizer speed_optimizer(obj_list, trajectory, modified_speed_limit, 0,
                                 total_path_length, 0, TOTAL_TIME,
                                 current_speed);
  if (speed_optimizer.Process(speed_path)) {
    speed_path.success = true;
  } else {
    speed_path.success = false;
  }
  speed_path.st_boundaries = speed_optimizer.st_data_.st_boundaries();
  speed_path.path          = trajectory;
  speed_path.dp_speed_data = speed_optimizer.dp_speed_data_;

  return speed_path;
}

SpeedOptimizer::SpeedOptimizer(
    std::vector<Obstacle>& obstacle_list, std::vector<Pose>& trajectory,
    const std::vector<std::pair<double, double>>& speed_limit, double s_start,
    double s_end, double t_start, double t_end, double current_speed)
    : st_data_(obstacle_list, trajectory, s_start, s_end, t_start, t_end,
               current_speed),
      gridded_path_time_graph_(st_data_, obstacle_list, trajectory.front(),
                               SpeedLimit(speed_limit), current_speed),
      obstacle_list_(obstacle_list),
      trajectory_(trajectory),
      path_range_(s_start, s_end),
      time_range_(t_start, t_end),
      qp_speed_optimizer_(trajectory.front(), trajectory.back(),
                          SpeedLimit(speed_limit)) {}

bool SpeedOptimizer::DP_Process() {
  return gridded_path_time_graph_.Search(&dp_speed_data_);
}

bool SpeedOptimizer::QP_Process() {
  return qp_speed_optimizer_.Process(st_data_, dp_speed_data_);
}

bool SpeedOptimizer::Process(SpeedPath& speed_path) {
  clock_t start_t, end_t;
  start_t = clock();
  if (!DP_Process()) {
    return false;
  }
  end_t = clock();

  // std::cout << "SpeedTime: dp time cost=" << static_cast<double>(end_t -
  // start_t) / CLOCKS_PER_SEC << std::endl;

  start_t = clock();
  if (trajectory_.back().s > 5.0 && QP_Process() &&
      qp_speed_optimizer_.RetrieveSpeedData(spline_speed_data_)) {
    speed_path.qp_success = true;
    speed_path.splines    = splines();
  } else {
    // If failing to solve qp problem,
    // we adopt an alternative method
    // to generate cubic splines
    const int           NUM_POINT = dp_speed_data_.size();
    SplineLib::Vec2f    points[NUM_POINT];
    SplineLib::Vec2f    velocity[NUM_POINT];
    SplineLib::cSpline2 splines[NUM_POINT - 1];

    for (size_t i = 0; i < dp_speed_data_.size(); ++i) {
      points[i] =
          SplineLib::Vec2f(dp_speed_data_[i].t(), dp_speed_data_[i].s());
      velocity[i] =
          SplineLib::Vec2f(dp_speed_data_[i].t(), dp_speed_data_[i].v());
    }
    int numSplines =
        SplineLib::SplinesFromHermite(NUM_POINT, points, velocity, splines);
    speed_path.cubic_splines.clear();
    for (int i = 0; i < numSplines; i++) {
      speed_path.cubic_splines.push_back(splines[i]);
    }
    const float T_RESOLUTION = 0.005;
    for (size_t i = 0; i + 1 < NUM_POINT; ++i) {
      float relative_time = 0.0;
      while (relative_time < 1.0) {
        double s = SplineLib::Position(splines[i], relative_time).y;
        double v = SplineLib::Velocity(splines[i], relative_time).y;
        double a = SplineLib::Acceleration(splines[i], relative_time).y;
        double t = relative_time;
        spline_speed_data_.AppendSpeedPoint(s, t, v, a, 0);
        relative_time += T_RESOLUTION;
      }
    }
  }
  end_t = clock();
  //   std::cout << "SpeedTime: qp time cost="
  //   << static_cast<double>(end_t - start_t) / CLOCKS_PER_SEC << std::endl;

  for (auto& point : trajectory_) {
    point.v = spline_speed_data_.GetVelocityByS(point.s);
    point.a = spline_speed_data_.GetAccelByS(point.s);
    point.t = spline_speed_data_.GetTimeByS(point.s);
  }

  return true;
}
}  // namespace TiEV
