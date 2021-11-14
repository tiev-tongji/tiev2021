#pragma once

#include <limits>
#include <vector>

#include "config.h"
#include "line_segment.h"
#include "st_point.h"

namespace TiEV {

/**
 * @class STBoundary
 * @brief area of blocking on S-T graph
 */
class STBoundary {
 public:
  enum class BoundaryType { CAR_COLLISION, UNKNOWN, OVERTAKE, YIELD };
  ObjectType obs_type = ObjectType::UNKNOWN;

 private:
  int                      id_;
  std::vector<LineSegment> line_segments_;
  STPoint                  bottom_left_point_;
  STPoint                  bottom_right_point_;
  STPoint                  upper_left_point_;
  STPoint                  upper_right_point_;

  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

 public:
  STBoundary() = default;

  STBoundary(STPoint blp, STPoint brp, STPoint ulp, STPoint urp);

  int id() const { return id_; }

  void set_id(int id) { id_ = id; }

  void set_bottom_left_point(STPoint p) { bottom_left_point_ = p; }

  void set_bottom_right_point(STPoint p) { bottom_right_point_ = p; }

  void set_upper_left_point(STPoint p) { upper_left_point_ = p; }

  void set_upper_right_point(STPoint p) { upper_right_point_ = p; }

  void set_boundary_type(const BoundaryType& boundary_type) {
    boundary_type_ = boundary_type;
  }

  STPoint bottom_left_point() const { return bottom_left_point_; }

  STPoint bottom_right_point() const { return bottom_right_point_; }

  STPoint upper_left_point() const { return upper_left_point_; }

  STPoint upper_right_point() const { return upper_right_point_; }

  BoundaryType boundary_type() { return boundary_type_; }

  double min_s() const { return min_s_; }

  double max_s() const { return max_s_; }

  double min_t() const { return min_t_; }

  double max_t() const { return max_t_; }

  bool IsPointInBoundary(const STPoint& st_point) const;

  /**
   * @brief Check whether a segment overlaps the s-t boundary
   * @param start The start point of the segment
   * @param end The end point of the segment
   * @return True or false
   */
  bool HasOverlap(const STPoint& start, const STPoint& end) const;

  bool GetUnblockSRange(const double curr_time, const double total_path_length,
                        double* s_upper, double* s_lower) const;
};
}  // namespace TiEV
