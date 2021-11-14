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

#include "const.h"
#include "map_manager.h"

namespace TiEV {

ReferenceLineInfo::ReferenceLineInfo(std::vector<HDMapPoint> reference_line,
                                     Pose                    stop_point) {
  std::cout << "ReferenceLineInfo constructor not implemented " << std::endl;
  // convert reference_line and stop_point information to local coordination
  stop_point.s /= GRID_RESOLUTION;
  planning_target_.set_stop_point(stop_point);

  auto clamp = [](double x, double lb, double ub) {
    if (x > ub) return ub;
    if (x < lb) return lb;
    return x;
  };
  auto caclCurvature = [](const Point2d& xim1, const Point2d& xi,
                          const Point2d& xip1) {
    Point2d Dxi      = xi - xim1;
    Point2d Dxip1    = xip1 - xi;
    double  absDxi   = Dxi.len();
    double  absDxip1 = Dxip1.len();
    double  Dphi     = 0;
    if (absDxi == 0 || absDxip1 == 0)
      return 0.0;
    else {
      double costheta = Dxi.dot(Dxip1) / (absDxi * absDxip1);
      if (costheta > 1) costheta = 1;
      if (costheta < -1) costheta = -1;
      Dphi = std::acos(costheta);
    }
    double kappa = Dphi / absDxi;
    if (std::isnan(kappa)) {
      kappa = 0.0;
    }

    return kappa;
  };

  reference_line_ = reference_line;
  Point2d xim1, xi, xip1;
  for (int i = 1; i + 1 < reference_line_.size(); ++i) {
    xim1 = Point2d(reference_line_[i - 1].x, reference_line_[i - 1].y);
    xi   = Point2d(reference_line_[i].x, reference_line_[i].y);
    xip1 = Point2d(reference_line_[i + 1].x, reference_line_[i + 1].y);
    reference_line_[i].s /= GRID_RESOLUTION;
    reference_line_[i].k  = caclCurvature(xim1, xi, xip1);
    reference_line_[i].dk = 0;  // don't consider dk term for now
    if (i == reference_line_.size() - 2) {
      reference_line_[0].s /= GRID_RESOLUTION;
      reference_line_[0].k = reference_line_[1].k;
      reference_line_[i + 1].s /= GRID_RESOLUTION;
      reference_line_[i + 1].k = reference_line_[i].k;
    }
  }
}

double ReferenceLineInfo::GetSpeedLimitFromS(double s) {
  std::cout << "GetSpeedLimitFromS is not implemented " << std::endl;
  return 20;
}

}  // namespace TiEV
