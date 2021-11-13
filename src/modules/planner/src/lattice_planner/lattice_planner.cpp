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

#include "lattice_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cartesian_frenet_conversion.h"
#include "path_matcher.h"
#include "path_time_graph.h"
#include "prediction_querier.h"
// #include "collision_checker.h"
#include "constraint_checker.h"
// #include "backup_trajectory_generator.h"
#include "lattice_trajectory1d.h"
#include "trajectory1d_generator.h"
#include "trajectory_combiner.h"
#include "trajectory_evaluator.h"

#define FLAGS_speed_lon_decision_horizon 10
#define FLAGS_trajectory_time_length     10

namespace TiEV {

std::vector<Pose> ToDiscretizedReferenceLine(
    const std::vector<HDMapPoint>& ref_points) {
  double            s = 0.0;
  std::vector<Pose> path_points;
  for (const auto& ref_point : ref_points) {
    Pose path_point;
    path_point.set_x(ref_point.x);
    path_point.set_y(ref_point.y);
    path_point.set_theta(ref_point.ang);
    path_point.set_kappa(ref_point.k);
    path_point.set_dkappa(ref_point.dk);

    if (!path_points.empty()) {
      double dx = path_point.x - path_points.back().x;
      double dy = path_point.y - path_points.back().y;
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void ComputeInitFrenetState(const Pose&            matched_point,
                            const Pose&            cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s, matched_point.x, matched_point.y, matched_point.ang,
      matched_point.k, matched_point.dk, cartesian_state.x, cartesian_state.y,
      cartesian_state.v, cartesian_state.a, cartesian_state.ang,
      cartesian_state.k, ptr_s, ptr_d);
}

bool LatticePlanner::Plan(
    const Pose&                                 planning_init_point,
    const std::vector<DynamicObj>&              dynamic_obj_list,
    const std::vector<std::vector<HDMapPoint>>& reference_line_list) {
  size_t                         success_line_count = 0;
  size_t                         index              = 0;
  std::vector<ReferenceLineInfo> reference_line_info_list;
  for (const auto& reference_line : reference_line_list) {
    reference_line_info_list.emplace_back(reference_line);
  }
  std::vector<Obstacle> obstacle_list;
  std::cout << "obstacle_list in LatticePlanner::Plan not implemented "
            << std::endl;
  for (auto& reference_line_info : reference_line_info_list) {
    auto status = PlanOnReferenceLine(planning_init_point, obstacle_list,
                                      &reference_line_info);

    if (status) {
      success_line_count++;
    }
    ++index;
  }

  if (success_line_count > 0) {
    return true;
  }
  return false;
}

bool LatticePlanner::PlanOnReferenceLine(
    const Pose& planning_init_point, std::vector<Obstacle>& obstacle_list,
    ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles           = 0;
  static size_t num_planning_succeeded_cycles = 0;

  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the Pose format.
  auto ptr_reference_line = std::make_shared<std::vector<Pose>>(
      ToDiscretizedReferenceLine(reference_line_info->reference_line()));

  // 2. compute the matched point of the init planning point on the reference
  // line.
  Pose matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.x, planning_init_point.y);

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  auto ptr_prediction_querier =
      std::make_shared<PredictionQuerier>(obstacle_list, ptr_reference_line);

  // 4. parse the decision and get the planning target.
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      obstacle_list, *ptr_reference_line, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length);

  double speed_limit = reference_line_info->GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);

  PlanningTarget planning_target = reference_line_info->planning_target();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  // Get instance of collision checker and constraint checker
  // CollisionChecker collision_checker(obstacles, init_s[0], init_d[0],
  //                                    *ptr_reference_line,
  //                                    reference_line_info,
  //                                    ptr_path_time_graph);

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  size_t constraint_failure_count          = 0;
  size_t collision_failure_count           = 0;
  size_t combined_constraint_failure_count = 0;

  size_t lon_vel_failure_count   = 0;
  size_t lon_acc_failure_count   = 0;
  size_t lon_jerk_failure_count  = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count   = 0;
  size_t lat_jerk_failure_count  = 0;

  size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory =
        TrajectoryCombiner::Combine(*ptr_reference_line, *trajectory_pair.first,
                                    *trajectory_pair.second, 0);

    // check longitudinal and lateral acceleration
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
      }
      continue;
    }

    // check collision with other obstacles
    // if (collision_checker.InCollision(combined_trajectory)) {
    //   ++collision_failure_count;
    //   continue;
    // }

    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);
    break;
  }

  if (num_lattice_traj > 0) {
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return true;
  } else {
    std::cout << "lattice planner failed " << std::endl;
    return false;
  }

  // don't consider backup_trajectory for now
  // else {
  //   if (FLAGS_enable_backup_trajectory) {
  //     AERROR << "Use backup trajectory";
  //     BackupTrajectoryGenerator backup_trajectory_generator(
  //         init_s, init_d, planning_init_point.relative_time(),
  //         std::make_shared<CollisionChecker>(collision_checker),
  //         &trajectory1d_generator);
  //     DiscretizedTrajectory trajectory =
  //         backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

  //     reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
  //     reference_line_info->SetTrajectory(trajectory);
  //     reference_line_info->SetDrivable(true);
  //     return Status::OK();

  //   } else {
  //     reference_line_info->SetCost(std::numeric_limits<double>::infinity());
  //   }
  // }
}

}  // namespace TiEV
