#include "box.h"

#include <cmath>

#include "const.h"

namespace TiEV {
Box::Box(Vec center, double length, double width, double heading) {
  center_      = center;
  length_      = length;
  width_       = width;
  heading_     = heading;
  cos_heading_ = std::cos(heading);
  sin_heading_ = std::sin(heading);
  InitCorners();
}

void Box::InitCorners() {
  const double dx1 = length_ / 2.0 * cos_heading_ / GRID_RESOLUTION;
  const double dy1 = length_ / 2.0 * sin_heading_ / GRID_RESOLUTION;
  const double dx2 = width_ / 2.0 * sin_heading_ / GRID_RESOLUTION;
  const double dy2 = -width_ / 2.0 * cos_heading_ / GRID_RESOLUTION;

  corners_.clear();
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  for (auto& corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}
}  // namespace TiEV