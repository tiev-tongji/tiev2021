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

#include "reference_line_info.h"

#include <algorithm>

#define FLAGS_default_cruise_speed 1.1

// #include "absl/strings/str_cat.h"
// #include "cyber/task/task.h"
// #include "modules/common/configs/vehicle_config_helper.h"
// #include "modules/common/util/point_factory.h"
// #include "modules/common/util/util.h"
// #include "modules/map/hdmap/hdmap_common.h"
// #include "modules/map/hdmap/hdmap_util.h"
// #include "modules/planning/proto/planning_status.pb.h"
// #include "modules/planning/proto/sl_boundary.pb.h"

namespace TiEV {

bool PlanningTarget::has_stop_point() const { return has_stop_point_; }
Pose PlanningTarget::stop_point() const {return stop_point_;}
double PlanningTarget::cruise_speed() const {return cruise_speed_;}
void PlanningTarget::set_cruise_speed(double speed) {cruise_speed_ = speed;}


ReferenceLineInfo::ReferenceLineInfo(std::vector<HDMapPoint> reference_line) {
  std::cout << "ReferenceLineInfo constructor not implemented " << std::endl;
}

void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

double ReferenceLineInfo::GetCruiseSpeed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }



// bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
//   const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
//   // stitching point
//   const auto& path_point = adc_planning_point_.path_point();
//   Vec2d position(path_point.x(), path_point.y());
//   Vec2d vec_to_center(
//       (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
//       (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
//   Vec2d center(position + vec_to_center.rotate(path_point.theta()));
//   Box2d box(center, path_point.theta(), param.length(), param.width());
//   // realtime vehicle position
//   Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
//   Vec2d vehicle_center(vehicle_position +
//                        vec_to_center.rotate(vehicle_state_.heading()));
//   Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
//                     param.width());

//   if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
//     AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
//     return false;
//   }

//   InitFirstOverlaps();

//   if (adc_sl_boundary_.end_s() < 0 ||
//       adc_sl_boundary_.start_s() > reference_line_.Length()) {
//     AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
//           << " is not on reference line:[0, " << reference_line_.Length()
//           << "]";
//   }
//   static constexpr double kOutOfReferenceLineL = 10.0;  // in meters
//   if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
//       adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
//     AERROR << "Ego vehicle is too far away from reference line.";
//     return false;
//   }
//   is_on_reference_line_ = reference_line_.IsOnLane(adc_sl_boundary_);
//   if (!AddObstacles(obstacles)) {
//     AERROR << "Failed to add obstacles to reference line";
//     return false;
//   }

//   const auto& map_path = reference_line_.map_path();
//   for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
//     // -1 and + 1.0 are added to make sure it can be sampled.
//     reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
//                                   speed_bump.end_s + 1.0,
//                                   FLAGS_speed_bump_speed_limit);
//   }

//   SetCruiseSpeed(FLAGS_default_cruise_speed);

//   // set lattice planning target speed limit;
//   SetLatticeCruiseSpeed(FLAGS_default_cruise_speed);

//   vehicle_signal_.Clear();

//   return true;
// }




}  // namespace TiEV
