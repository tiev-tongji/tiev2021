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

#include "map_manager.h"

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

ReferenceLineInfo::ReferenceLineInfo(std::vector<HDMapPoint> reference_line)
    : reference_line_(reference_line) {
  std::cout << "ReferenceLineInfo constructor not implemented " << std::endl;
}

double ReferenceLineInfo::GetSpeedLimitFromS(double s) {
  std::cout << "GetSpeedLimitFromS is not implemented " << std::endl;
  return 20;
}

}  // namespace TiEV
