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
#include <string>

#include "const.h"
#include "map_manager.h"
#include "path_smoother.h"

namespace TiEV {

ReferenceLineInfo::ReferenceLineInfo(std::vector<HDMapPoint> reference_line,
                                     Pose                    stop_point) {
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

  reference_line_.clear();
  PathSmoother         ps;
  int                  num_interpolate_points = 4;
  std::vector<Point2d> smoothed_path =
      ps.smoothPath(reference_line, num_interpolate_points, "lattice_planner");
  for (int i = 0; i < smoothed_path.size(); ++i) {
    int        in_ref_line_id = std::min(i / (num_interpolate_points + 1),
                                  static_cast<int>(reference_line.size() - 1));
    HDMapPoint hdmap_point;
    hdmap_point.x          = smoothed_path[i].x;
    hdmap_point.y          = smoothed_path[i].y;
    hdmap_point.lane_num   = reference_line[in_ref_line_id].lane_num;
    hdmap_point.lane_width = reference_line[in_ref_line_id].lane_width;
    hdmap_point.event      = reference_line[in_ref_line_id].event;
    hdmap_point.mode       = reference_line[in_ref_line_id].mode;
    hdmap_point.speed_mode = reference_line[in_ref_line_id].speed_mode;
    // following lane_seq, direction, block_type need to be updated
    hdmap_point.lane_seq   = reference_line[in_ref_line_id].lane_seq;
    hdmap_point.direction  = reference_line[in_ref_line_id].direction;
    hdmap_point.block_type = reference_line[in_ref_line_id].block_type;
    reference_line_.push_back(hdmap_point);
  }
  Point2d xim1, xi, xip1, vec;
  double  accumulated_s = 0;
  for (int i = 1; i + 1 < reference_line_.size(); ++i) {
    xim1 = Point2d(reference_line_[i - 1].x, reference_line_[i - 1].y);
    xi   = Point2d(reference_line_[i].x, reference_line_[i].y);
    xip1 = Point2d(reference_line_[i + 1].x, reference_line_[i + 1].y);
    vec  = xi - xim1;
    accumulated_s += vec.len();
    reference_line_[i].s   = accumulated_s;
    reference_line_[i].ang = std::atan2(vec.y, vec.x);
    reference_line_[i].k   = caclCurvature(xim1, xi, xip1);
    reference_line_[i].dk  = 0;  // don't consider dk term for now
    if (i == reference_line_.size() - 2) {
      reference_line_[0].s       = 0;
      reference_line_[0].k       = reference_line_[1].k;
      reference_line_[0].ang     = reference_line_[1].ang;
      reference_line_[i + 1].s   = reference_line_[i].s;
      reference_line_[i + 1].k   = reference_line_[i].k;
      reference_line_[i + 1].ang = reference_line_[i].ang;
    }
  }
}

double ReferenceLineInfo::GetSpeedLimitFromS(double s) {
  std::cout << "GetSpeedLimitFromS is not implemented " << std::endl;
  return 20;
}

// shift x,y cooridnate and update lane_seq
void ReferenceLineInfo::ShiftRefLine(const int src_lane_seq,
                                     const int dest_lane_seq) {
  auto offset = [](HDMapPoint& map_point, const double shift_dist) {
    auto        normal_vec = Point2d(-sin(map_point.ang), cos(map_point.ang));
    const auto& offset_vec = normal_vec * shift_dist;
    map_point.x += offset_vec.x;
    map_point.y += offset_vec.y;
  };
  int shift_left_lane_num = dest_lane_seq - src_lane_seq;
  for (int i = 0; i < reference_line_.size(); ++i) {
    auto& p    = reference_line_[i];
    p.lane_seq = dest_lane_seq;
    if ((shift_left_lane_num + 1) > p.lane_num) {
      reference_line_.resize(i);
      break;
    }
    offset(p, p.lane_width * shift_left_lane_num / GRID_RESOLUTION);
  }
}

}  // namespace TiEV
