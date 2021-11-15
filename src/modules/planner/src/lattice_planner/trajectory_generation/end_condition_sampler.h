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

#include <memory>
#include <string>
#include <utility>
#include <vector>

// #include "modules/common/configs/vehicle_config_helper.h"
#include "st_point.h"
// #include "modules/planning/lattice/behavior/feasible_region.h"
#include "path_time_graph.h"

namespace TiEV {

struct SamplePoint {
  STPoint              path_time_point;
  double               ref_v;
  friend std::ostream& operator<<(std::ostream& out, const SamplePoint& p) {
    cout << "s: " << p.path_time_point.s() << "\tt: " << p.path_time_point.t()
         << "\tv: " << p.ref_v << "\n";
    return out;
  }
};

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
 public:
  EndConditionSampler(const std::array<double, 3>&   init_s,
                      const std::array<double, 3>&   init_d,
                      std::shared_ptr<PathTimeGraph> ptr_path_time_graph);

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions()
      const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const double ref_stop_point) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForPathTimePoints(
      const std::vector<Pose>& reference_line) const;

 private:
  std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints(
      const std::vector<Pose>& reference_line) const;

  void QueryFollowPathTimePoints(const int                 obstacle_id,
                                 std::vector<SamplePoint>* sample_points,
                                 const std::vector<Pose>& reference_line) const;

  void QueryOvertakePathTimePoints(
      const int obstacle_id, std::vector<SamplePoint>* sample_points,
      const std::vector<Pose>& reference_line) const;

  double ProjectVelocityAlongReferenceLine(
      const int obstacle_id, const double s, const double t,
      const std::vector<Pose>& reference_line) const;

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  //   FeasibleRegion feasible_region_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};

}  // namespace TiEV
