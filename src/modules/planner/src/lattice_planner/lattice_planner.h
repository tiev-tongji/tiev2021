#pragma once
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


#include <memory>
#include <string>
#include <vector>

// #include "status.h"
// #include "modules/planning/common/frame.h"
#include "reference_line_info.h"
// #include "modules/planning/planner/planner.h"
// #include "modules/planning/proto/planning_config.pb.h"
#include "pose.h"
#include "tiev_class.h"
#include "path_time_graph.h"

namespace TiEV {


class LatticePlanner {
 public:
  LatticePlanner() = delete;

  ~LatticePlanner() = default;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return true if planning succeeds; false otherwise.
   */
  bool Plan(const Pose& planning_init_point,
            const std::vector<DynamicObj>& dynamic_obj_list, 
            const std::vector<std::vector<HDMapPoint>>& reference_line_list);

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  bool PlanOnReferenceLine(const Pose& planning_init_point,
            std::vector<Obstacle>& obstacle_list,
            ReferenceLineInfo* reference_line_info);
};


}  // namespace TiEV
