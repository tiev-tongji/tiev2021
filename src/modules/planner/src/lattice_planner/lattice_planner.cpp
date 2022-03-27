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
#include "const.h"
#include "lattice_planner_params.h"
#include "lattice_trajectory1d.h"
#include "log.h"
#include "map_manager.h"
#include "opencv2/opencv.hpp"
#include "path_matcher.h"
#include "path_time_graph.h"
#include "tiev_utils.h"
#include "trajectory1d_generator.h"
#include "trajectory_combiner.h"

namespace TiEV {

std::vector<Pose> HDMapPoint2Pose(const std::vector<HDMapPoint>& ref_points) {
  std::vector<Pose> path_points;
  for (const auto& ref_point : ref_points) {
    Pose path_point;
    path_point.set_x(ref_point.x);
    path_point.set_y(ref_point.y);
    path_point.set_theta(ref_point.ang);
    path_point.set_kappa(ref_point.k);
    path_point.set_dkappa(ref_point.dk);
    path_point.set_s(ref_point.s);
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

// preprocess
// convert unit m,m/s,m/s^2 to unit grid ...
std::vector<Pose> LatticePlanner::Plan(
    const Pose& init_point, const std::vector<DynamicObj>& dynamic_obj_list,
    const std::vector<HDMapPoint>& reference_line) {
  std::vector<Pose> empty_path;
  if (reference_line.empty()) return empty_path;
  size_t                         success_line_count = 0;
  size_t                         index              = 0;
  std::vector<ReferenceLineInfo> reference_line_info_list;
  // convert reference line to reference line info
  Pose planning_init_point = init_point;
  planning_init_point.v /= GRID_RESOLUTION;
  planning_init_point.a = 0;  // don't consider acc for now
  time_t start_time, end_time;
  start_time = getTimeStamp();
  reference_line_info_list.emplace_back(reference_line);
  end_time                 = getTimeStamp();
  int original_ref_line_id = reference_line[0].lane_seq;
  for (int i = 1; i <= reference_line.front().lane_num; ++i) {
    break;  // don't shift lane
    if (i == original_ref_line_id) continue;
    ReferenceLineInfo new_ref_line_info = reference_line_info_list.front();
    new_ref_line_info.ShiftRefLine(original_ref_line_id, i);
    reference_line_info_list.push_back(new_ref_line_info);
  }
  // convert dynamic_obj information to local coordinate
  // don't consider obstacles while predicting other vehicles' trajectories
  std::vector<Obstacle> obstacle_list;
  // punish lane change
  double min_distance_to_refline = std::numeric_limits<double>::max();
  int    lane_seq_car_in         = -1;
  for (int i = 0; i < reference_line_info_list.size(); ++i) {
    Pose matched_point = PathMatcher::MatchToPath(
        reference_line_info_list[i].reference_line(), planning_init_point.x,
        planning_init_point.y, "lattice_planner");
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s,
                           &init_d);
    if (fabs(init_d[0]) < min_distance_to_refline) {
      min_distance_to_refline = fabs(init_d[0]);
      lane_seq_car_in =
          reference_line_info_list[i].reference_line()[0].lane_seq;
    }
  }
  for (int i = 0; i < reference_line_info_list.size(); ++i) {
    int cur_lane_seq = reference_line_info_list[i].reference_line()[0].lane_seq;
    if (fabs(cur_lane_seq - lane_seq_car_in) > 1) {
      reference_line_info_list[i].SetPriorityCost(
          100);  // only allow to change to next lane
    } else {
      reference_line_info_list[i].SetPriorityCost(
          fabs(cur_lane_seq - lane_seq_car_in) * 3);
    }
  }
  // plan for each reference_line
  int    best_line_id = -1;
  double min_cost     = std::numeric_limits<double>::max();
  for (int i = 0; i < reference_line_info_list.size(); ++i) {
    std::cout << "REFERENCE_LINE_PLANNING-------------------------------- " << i
              << std::endl;
    auto& reference_line_info = reference_line_info_list[i];
    bool  status = PlanOnReferenceLine(planning_init_point, obstacle_list,
                                      &reference_line_info);

    if (status) {
      success_line_count++;
      if (reference_line_info.Cost() < min_cost) {
        best_line_id = i;
        min_cost     = reference_line_info.Cost();
      }
    }
  }
  // convert unit grid to unit m
  if (success_line_count > 0 && best_line_id >= 0) {
    std::cout << "multi lane cost: " << std::endl;
    for (const auto& line_info : reference_line_info_list) {
      std::cout << line_info.Cost() << std::endl;
    }
    std::vector<Pose> best_traj =
        reference_line_info_list[best_line_id].trajectory();
    for (auto& p : best_traj) {
      p.v *= GRID_RESOLUTION;
      p.a *= GRID_RESOLUTION;
      p.s *= GRID_RESOLUTION;
      p.k /= GRID_RESOLUTION;
      p.dk /= GRID_RESOLUTION;
    }
    return best_traj;
  }
  std::vector<Pose> empty_traj;
  return empty_traj;
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
      HDMapPoint2Pose(reference_line_info->reference_line()));

  // 2. compute the matched point of the init planning point on the reference
  // line.
  Pose matched_point =
      PathMatcher::MatchToPath(*ptr_reference_line, planning_init_point.x,
                               planning_init_point.y, "lattice_planner");

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  // 4. configure
  double speed_limit = reference_line_info->GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(planning_init_point.v);
  PlanningTarget planning_target = reference_line_info->planning_target();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator                 trajectory1d_generator(init_s, init_d,
                                               ptr_reference_line);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  // 6. combine two 1d trajectories to one 2d trajectory
  size_t num_lattice_traj = 0;
  auto   trajectory_pair =
      std::make_pair(lon_trajectory1d_bundle[0], lat_trajectory1d_bundle[0]);
  double            trajectory_pair_cost = 0;
  std::vector<Pose> combined_trajectory  = TrajectoryCombiner::Combine(
      *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second, 0);

  // std::cout << "lon traj: " << std::endl;
  // std::cout << trajectory_pair.first;
  // std::cout << "lat traj: " << std::endl;
  // std::cout << trajectory_pair.second;
  auto draw_curve = [](std::shared_ptr<Curve1d> curve, std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
    for (double x = 0.0; x < curve->ParamLength(); x += 1) {
      img.at<cv::Vec3b>((int)x, (int)curve->Evaluate(0, x) + 125) =
          cv::Vec3b(0x00, 0x00, 0x00);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 2,
               cv::Vec3b(0x00, 0x00, 0xff), 2);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(50);
  };
  auto draw_path = [](std::vector<Pose> path, std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
    for (const auto& p : path) {
      img.at<cv::Vec3b>((int)p.x, (int)p.y) = cv::Vec3b(0x00, 0x00, 0x00);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 2,
               cv::Vec3b(0x00, 0x00, 0xff), 2);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(50);
  };
  auto draw_path_compare = [](std::vector<Pose> path1, std::vector<Pose> path2,
                              std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC4, {255, 255, 255, 255});
    for (const auto& p : path1) {
      img.at<cv::Vec4b>((int)p.x, (int)p.y) = cv::Vec4b(0x00, 0x00, 0xff, 0xff);
    }
    for (int i = 0; i < path2.size(); i += 5) {
      auto p = path2[i];
      cv::circle(img, cv::Point2d((int)p.y, (int)p.x), 0.1,
                 cv::Vec4b(0x00, 0xff, 0x00, 0x00), 0);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 1.5,
               cv::Vec4b(0x00, 0x00, 0x00, 0xff), 0);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(10);
  };
  // draw_curve(trajectory_pair.first, "s-t");
  // draw_curve(trajectory_pair.first, "l-s");
  // draw_path(combined_trajectory, "x-y");
  // draw_path(*ptr_reference_line, "ref_path");
  // draw_path_compare(combined_trajectory, *ptr_reference_line,
  //                   "red-ref, green-traj");

  num_lattice_traj += 1;
  reference_line_info->SetTrajectory(combined_trajectory);
  reference_line_info->SetCost(reference_line_info->PriorityCost() +
                               trajectory_pair_cost);
  reference_line_info->SetDrivable(true);

  if (num_lattice_traj > 0) {
    num_planning_succeeded_cycles += 1;
    return true;
  } else {
    // std::cout << "lattice planner failed " << std::endl;
    return false;
  }
}

bool LatticePlanner::PlanOnReferenceLine(
    const Pose& planning_init_point, ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles           = 0;
  static size_t num_planning_succeeded_cycles = 0;

  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the Pose format.
  auto ptr_reference_line = std::make_shared<std::vector<Pose>>(
      HDMapPoint2Pose(reference_line_info->reference_line()));

  // 2. compute the matched point of the init planning point on the reference
  // line.
  Pose matched_point =
      PathMatcher::MatchToPath(*ptr_reference_line, planning_init_point.x,
                               planning_init_point.y, "lattice_planner");

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  // 4. configure
  double speed_limit = reference_line_info->GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);
  PlanningTarget planning_target = reference_line_info->planning_target();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator                 trajectory1d_generator(init_s, init_d,
                                               ptr_reference_line);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  // 6. combine two 1d trajectories to one 2d trajectory
  size_t num_lattice_traj = 0;
  auto   trajectory_pair =
      std::make_pair(lon_trajectory1d_bundle[0], lat_trajectory1d_bundle[0]);
  double            trajectory_pair_cost = 0;
  std::vector<Pose> combined_trajectory  = TrajectoryCombiner::Combine(
      *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second, 0);

  // std::cout << "lon traj: " << std::endl;
  // std::cout << trajectory_pair.first;
  // std::cout << "lat traj: " << std::endl;
  // std::cout << trajectory_pair.second;
  auto draw_curve = [](std::shared_ptr<Curve1d> curve, std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
    for (double x = 0.0; x < curve->ParamLength(); x += 1) {
      img.at<cv::Vec3b>((int)x, (int)curve->Evaluate(0, x) + 125) =
          cv::Vec3b(0x00, 0x00, 0x00);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 2,
               cv::Vec3b(0x00, 0x00, 0xff), 2);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(50);
  };
  auto draw_path = [](std::vector<Pose> path, std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
    for (const auto& p : path) {
      img.at<cv::Vec3b>((int)p.x, (int)p.y) = cv::Vec3b(0x00, 0x00, 0x00);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 2,
               cv::Vec3b(0x00, 0x00, 0xff), 2);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(50);
  };
  auto draw_path_compare = [](std::vector<Pose> path1, std::vector<Pose> path2,
                              std::string windowname) {
    cv::namedWindow(windowname, cv::WINDOW_KEEPRATIO);
    cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC4, {255, 255, 255, 255});
    for (const auto& p : path1) {
      img.at<cv::Vec4b>((int)p.x, (int)p.y) = cv::Vec4b(0x00, 0x00, 0xff, 0xff);
    }
    for (int i = 0; i < path2.size(); i += 5) {
      auto p = path2[i];
      cv::circle(img, cv::Point2d((int)p.y, (int)p.x), 0.1,
                 cv::Vec4b(0x00, 0xff, 0x00, 0x00), 0);
    }
    cv::circle(img, cv::Point2d(CAR_CEN_COL, CAR_CEN_ROW), 1.5,
               cv::Vec4b(0x00, 0x00, 0x00, 0xff), 0);
    cv::imshow(windowname, img);
    cv::resizeWindow(windowname, 1600, 800);
    cv::waitKey(10);
  };
  // draw_curve(trajectory_pair.first, "s-t");
  // draw_curve(trajectory_pair.first, "l-s");
  // draw_path(combined_trajectory, "x-y");
  // draw_path(*ptr_reference_line, "ref_path");
  // draw_path_compare(combined_trajectory, *ptr_reference_line,
  //                   "red-ref, green-traj");

  num_lattice_traj += 1;
  reference_line_info->SetTrajectory(combined_trajectory);
  reference_line_info->SetCost(reference_line_info->PriorityCost() +
                               trajectory_pair_cost);
  reference_line_info->SetDrivable(true);

  if (num_lattice_traj > 0) {
    num_planning_succeeded_cycles += 1;
    return true;
  } else {
    // std::cout << "lattice planner failed " << std::endl;
    return false;
  }
}

}  // namespace TiEV
