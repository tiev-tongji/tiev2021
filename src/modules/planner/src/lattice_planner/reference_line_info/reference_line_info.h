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

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "modules/common/proto/drive_state.pb.h"
// #include "modules/common/proto/pnc_point.pb.h"
// #include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
// #include "modules/map/hdmap/hdmap_common.h"
// #include "modules/map/pnc_map/pnc_map.h"
// #include "modules/planning/common/path/path_data.h"
// #include "modules/planning/common/path_boundary.h"
// #include "modules/planning/common/path_decision.h"
// #include "modules/planning/common/planning_context.h"
// #include "modules/planning/common/planning_gflags.h"
// #include "modules/planning/common/speed/speed_data.h"
// #include "modules/planning/common/st_graph_data.h"
// #include "modules/planning/common/trajectory/discretized_trajectory.h"
// #include "modules/planning/proto/lattice_structure.pb.h"
// #include "modules/planning/proto/planning.pb.h"
#include "pose.h"

namespace TiEV {

class PlanningTarget {
 public:
  PlanningTarget() : cruise_speed_(0), has_stop_point_(false) {}
  bool   has_stop_point() const { return has_stop_point_; }
  Pose   stop_point() const { return stop_point_; }
  double cruise_speed() const { return cruise_speed_; }
  void   set_cruise_speed(double speed) { cruise_speed_ = speed; }
  void   set_stop_point(Pose stop_point) { stop_point_ = stop_point; }

 private:
  double cruise_speed_;  // cruise_speed_ here is actually speed_limit
  Pose   stop_point_;
  bool   has_stop_point_;
};

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;
  ReferenceLineInfo(std::vector<HDMapPoint> reference_line);

  double GetSpeedLimitFromS(double s);

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  const std::vector<HDMapPoint>& reference_line() const {
    return reference_line_;
  };

  // For lattice planner'speed planning target
  void SetLatticeCruiseSpeed(double speed) {
    planning_target_.set_cruise_speed(speed);
  };

  double                Cost() const { return cost_; }
  void                  AddCost(double cost) { cost_ += cost; }
  void                  SetCost(double cost) { cost_ = cost; }
  double                PriorityCost() const { return priority_cost_; }
  void                  SetPriorityCost(double cost) { priority_cost_ = cost; }
  const PlanningTarget& planning_target() { return planning_target_; }

  void SetDrivable(bool drivable) { is_drivable_ = drivable; };
  bool IsDrivable() const { return is_drivable_; };

  void SetTrajectory(const std::vector<Pose>& trajectory) {
    discretized_trajectory_ = trajectory;
  }
  std::vector<Pose>& trajectory() { return discretized_trajectory_; }

  void ShiftRefLine(const int src_lane_seq, const int dest_lane_seq);

  double GetLaneWidth() const;

 private:
  std::vector<HDMapPoint> reference_line_;
  std::vector<Pose>       discretized_trajectory_;
  double                  cost_ = 0.0;

  bool is_drivable_ = true;

  bool is_on_reference_line_ = false;

  bool is_path_lane_borrow_ = false;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;
};

}  // namespace TiEV
