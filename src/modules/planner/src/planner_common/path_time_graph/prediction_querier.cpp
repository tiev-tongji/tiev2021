/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "prediction_querier.h"

#include "linear_interpolation.h"
#include "path_matcher.h"
#include <algorithm>

namespace TiEV {

PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<Pose>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  for (const auto ptr_obstacle : obstacles) {
    // if (common::util::InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->Id(),
    //                                      ptr_obstacle)) {
    //   obstacles_.push_back(ptr_obstacle);
    // } else {
    //   AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
    // }
    if (true) {
      std::cout << "PredictionQuerier not implemented " << std::endl;
      obstacles_.push_back(ptr_obstacle);
    }
  }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  // ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->path;
  int num_traj_point = trajectory.size();
  if (num_traj_point < 2) {
    return 0.0;
  }

  if (t < trajectory.front().t ||
      t > trajectory.back().t) {
    return 0.0;
  }

  auto matched_it =
      std::lower_bound(trajectory.begin(),
                       trajectory.end(), t,
                       [](const Pose& p, const double t) {
                         return p.t < t;
                       });

  double v = matched_it->v;
  double theta = matched_it->ang;
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);

  Pose obstacle_point_on_ref_line =
      PathMatcher::MatchToPath(*ptr_reference_line_, s);
  auto ref_theta = obstacle_point_on_ref_line.ang;

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

}  // namespace TiEV
