#include "st_boundary.h"

#include <cmath>
#include <iostream>

#include "vec.h"

namespace TiEV {

STBoundary::STBoundary(STPoint blp, STPoint brp, STPoint ulp, STPoint urp) {
  bottom_left_point_  = blp;
  bottom_right_point_ = brp;
  upper_left_point_   = ulp;
  upper_right_point_  = urp;

  min_s_ = std::fmin(bottom_left_point_.s(), bottom_right_point_.s());
  max_s_ = std::fmax(upper_left_point_.s(), upper_right_point_.s());
  min_t_ = std::fmin(bottom_left_point_.t(), upper_left_point_.t());
  max_t_ = std::fmax(bottom_right_point_.t(), upper_right_point_.t());

  line_segments_.emplace_back(LineSegment(blp, brp));
  line_segments_.emplace_back(LineSegment(ulp, urp));
  line_segments_.emplace_back(LineSegment(blp, ulp));
  line_segments_.emplace_back(LineSegment(brp, urp));
}

bool STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }

  Vec upper_left_vec   = Vec(upper_left_point_.t() - st_point.t(),
                           upper_left_point_.s() - st_point.s());
  Vec upper_right_vec  = Vec(upper_right_point_.t() - st_point.t(),
                            upper_right_point_.s() - st_point.s());
  Vec bottom_left_vec  = Vec(bottom_left_point_.t() - st_point.t(),
                            bottom_left_point_.s() - st_point.s());
  Vec bottom_right_vec = Vec(bottom_right_point_.t() - st_point.t(),
                             bottom_right_point_.s() - st_point.s());

  /**
   * If st_point is in the STBoundary, the upper-cross-product
   * and the bottom-cross-product should be signed differnetly
   */
  double check_upper = upper_left_vec.x() * upper_right_vec.y() -
                       upper_left_vec.y() * upper_right_vec.x();
  double check_bottom = bottom_left_vec.x() * bottom_right_vec.y() -
                        bottom_left_vec.y() * bottom_right_vec.x();

  return (check_upper * check_bottom < 0);
}

bool STBoundary::HasOverlap(const STPoint& start, const STPoint& end) const {
  LineSegment lineSeg = LineSegment(start, end);
  if (IsPointInBoundary(start) || IsPointInBoundary(end)) {
    return true;
  } else {
    for (const auto& edge : line_segments_) {
      if (LineSegIntersec(lineSeg, edge)) {
        return true;
      }
    }
  }
  return false;
}

bool STBoundary::GetUnblockSRange(const double curr_time,
                                  const double total_path_length,
                                  double* s_upper, double* s_lower) const {
  *s_upper = total_path_length;
  *s_lower = 0.0;
  if (curr_time < min_t_ || curr_time > max_t_) {
    return true;
  }

  const double r_upper = (curr_time - upper_left_point_.t()) /
                         (upper_right_point_.t() - upper_left_point_.t());

  double upper_cross_s =
      upper_left_point_.s() +
      r_upper * (upper_right_point_.s() - upper_left_point_.s());

  const double r_lower = (curr_time - bottom_left_point_.t()) /
                         (bottom_right_point_.t() - bottom_left_point_.t());

  double lower_cross_s =
      bottom_left_point_.s() +
      r_lower * (bottom_right_point_.s() - bottom_left_point_.s());

  if (boundary_type_ == BoundaryType::YIELD) {
    *s_upper = std::fmax(0.0, lower_cross_s - 3);
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    *s_lower = std::fmax(*s_lower, upper_cross_s + 2);
  } else if (boundary_type_ == BoundaryType::CAR_COLLISION) {
    *s_upper = 0;
  } else {
    return false;
  }

  return true;
}
}  // namespace TiEV
