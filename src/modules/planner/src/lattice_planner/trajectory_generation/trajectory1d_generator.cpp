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

#include "trajectory1d_generator.h"

namespace TiEV {

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
typedef std::array<double, 3>                 State;
typedef std::pair<State, double>              Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const State& lon_init_state, const State& lat_init_state,
    std::shared_ptr<std::vector<Pose>> ptr_reference_line)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state),
      ptr_reference_line_(ptr_reference_line) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    Trajectory1DBundle*   ptr_lon_trajectory_bundle,
    Trajectory1DBundle*   ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);

  // std::cout << "lon sample points: " << std::endl;
  for (const auto& curve : *ptr_lon_trajectory_bundle) {
    double t_max = curve->ParamLength();
    double s     = curve->Evaluate(0, t_max);
    double s0    = curve->Evaluate(0, 0.0);
    double v     = curve->Evaluate(1, t_max);
    double v0    = curve->Evaluate(1, 0.0);
    // std::cout << "s0=" << s0 << " s=" << s << " v0=" << v0 << " v=" << v
    //           << " t=" << t_max << std::endl;
  }
  // std::cout << "lat sample points: " << std::endl;
  for (const auto& curve : *ptr_lat_trajectory_bundle) {
    double s_max = curve->ParamLength();
    double d     = curve->Evaluate(0, s_max);
    double d0    = curve->Evaluate(0, 0.0);
    // std::cout << "d0=" << d0 << " d=" << d << " s=" << s_max << std::endl;
  }
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double        target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);
  if (end_conditions.empty()) {
    return;
  }

  // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
  // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
  // invoke the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const double        stop_point,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints(
          *ptr_reference_line_);
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    Trajectory1DBundle*   ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),
                                   ptr_lon_trajectory_bundle);

  // GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

  // if (planning_target.has_stop_point()) {
  //   GenerateSpeedProfilesForStopping(planning_target.stop_point().s,
  //                                    ptr_lon_trajectory_bundle);
  // }
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {
  if (true) {
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();

    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
                                  ptr_lat_trajectory_bundle);
  }
  // don't optimize lateral trajectory for now
  // else {
  //   double s_min = init_lon_state_[0];
  //   double s_max = s_min + FLAGS_max_s_lateral_optimization;

  //   double delta_s = FLAGS_default_delta_s_lateral_optimization;

  //   auto lateral_bounds =
  //       ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);

  //   // LateralTrajectoryOptimizer lateral_optimizer;
  //   std::unique_ptr<LateralQPOptimizer> lateral_optimizer(
  //       new LateralOSQPOptimizer);

  //   lateral_optimizer->optimize(init_lat_state_, delta_s, lateral_bounds);

  //   auto lateral_trajectory = lateral_optimizer->GetOptimalTrajectory();

  //   ptr_lat_trajectory_bundle->push_back(
  //       std::make_shared<PiecewiseJerkTrajectory1d>(lateral_trajectory));
  // }
}

}  // namespace TiEV
