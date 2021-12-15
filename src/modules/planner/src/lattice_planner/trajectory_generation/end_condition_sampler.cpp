/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "end_condition_sampler.h"

#include <algorithm>

#include "lattice_planner_params.h"
#include "path_matcher.h"

// #include "cyber/common/log.h"
// #include "modules/planning/common/planning_gflags.h"

namespace TiEV {

using State     = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(const State& init_s,
                                         const State& init_d)
    : init_s_(init_s),
      init_d_(init_d)
// feasible_region_(init_s),
{}

std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const {
  std::vector<Condition> end_d_conditions;
  for (const auto& s : FLAGS_end_s_candidates) {
    for (const auto& d : FLAGS_end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  // CHECK_GT(FLAGS_num_velocity_sample, 1U);

  // time interval is one second plus the last one 0.01
  std::array<double, FLAGS_num_of_time_samples> time_samples;
  for (size_t i = 1; i < FLAGS_num_of_time_samples; ++i) {
    auto ratio = static_cast<double>(i) /
                 static_cast<double>(FLAGS_num_of_time_samples - 1);
    time_samples[i] = FLAGS_trajectory_time_horizon * ratio;
  }
  time_samples[0] = FLAGS_minimal_time;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {
    std::cout << "V_upper, V_lower not implemented " << std::endl;
    double v_upper = ref_cruise_speed;
    double v_lower = 0;
    // double v_upper = std::min(feasible_region_.VUpper(time),
    // ref_cruise_speed); double v_lower = feasible_region_.VLower(time);

    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);

    double v_range = v_upper - v_lower;
    // Number of sample velocities
    size_t num_of_mid_points =
        std::min(static_cast<size_t>(FLAGS_num_velocity_sample - 1),
                 static_cast<size_t>(v_range / FLAGS_min_velocity_sample_gap));

    if (num_of_mid_points > 0) {
      double velocity_seg =
          v_range / static_cast<double>(num_of_mid_points + 1);
      for (size_t i = 1; i <= num_of_mid_points; ++i) {
        State end_s = {0.0, v_lower + velocity_seg * static_cast<double>(i),
                       0.0};
        end_s_conditions.emplace_back(end_s, time);
      }
    }
  }
  return end_s_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForStopping(
    const double ref_stop_point) const {
  // time interval is one second plus the last one 0.01
  std::array<double, FLAGS_num_of_time_samples> time_samples;
  for (size_t i = 1; i < FLAGS_num_of_time_samples; ++i) {
    auto ratio = static_cast<double>(i) /
                 static_cast<double>(FLAGS_num_of_time_samples - 1);
    time_samples[i] = FLAGS_trajectory_time_horizon * ratio;
  }
  time_samples[0] = FLAGS_minimal_time;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<Condition>
EndConditionSampler::SampleLonEndConditionsForPathTimePoints(
    const std::vector<Pose>& reference_line) const {
  std::vector<Condition> end_s_conditions;

  std::vector<SamplePoint> sample_points =
      QueryPathTimeObstacleSamplePoints(reference_line);
  for (const SamplePoint& sample_point : sample_points) {
    if (sample_point.path_time_point.t() < FLAGS_minimal_time) {
      continue;
    }
    double s = sample_point.path_time_point.s();
    double v = sample_point.ref_v;
    double t = sample_point.path_time_point.t();
    // if (s > feasible_region_.SUpper(t) || s <
    // feasible_region_.SLower(t))
    // {
    //   continue;
    // }
    State end_state = {s, v, 0.0};
    end_s_conditions.emplace_back(end_state, t);
  }
  return end_s_conditions;
}

std::vector<SamplePoint> EndConditionSampler::QueryPathTimeObstacleSamplePoints(
    const std::vector<Pose>& reference_line) const {
  std::vector<SamplePoint> sample_points;
  // for (const auto& path_time_obstacle :
  //      ptr_path_time_graph_->GetPathTimeObstacles()) {
  //   const int obstacle_id = path_time_obstacle.id();
  //   QueryFollowPathTimePoints(obstacle_id, &sample_points, reference_line);
  //   // QueryOvertakePathTimePoints(obstacle_id, &sample_points,
  //   reference_line);
  // }
  return sample_points;
}

void EndConditionSampler::QueryFollowPathTimePoints(
    const int obstacle_id, std::vector<SamplePoint>* const sample_points,
    const std::vector<Pose>& reference_line) const {
  return;
}

void EndConditionSampler::QueryOvertakePathTimePoints(
    const int obstacle_id, std::vector<SamplePoint>* sample_points,
    const std::vector<Pose>& reference_line) const {
  return;
}

}  // namespace TiEV
