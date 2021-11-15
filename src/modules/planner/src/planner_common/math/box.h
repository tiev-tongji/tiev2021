#pragma once

#include <iostream>
#include <limits>
#include <vector>

#include "vec.h"

namespace TiEV {

/**
 * @class
 * @brief bounding box
 *
 * X/Y in our system is East/North, heading is 0 at East
 */
class Box {
 private:
  Vec              center_;
  double           length_      = 0.0;
  double           width_       = 0.0;
  double           heading_     = 0.0;
  double           cos_heading_ = 1.0;
  double           sin_heading_ = 0.0;
  std::vector<Vec> corners_;
  double           max_x_ = std::numeric_limits<double>::lowest();
  double           min_x_ = std::numeric_limits<double>::max();
  double           max_y_ = std::numeric_limits<double>::lowest();
  double           min_y_ = std::numeric_limits<double>::max();

 public:
  Box() = default;

  Box(Vec center, double length, double width, double heading);

  void InitCorners();

  std::vector<Vec> corners() const { return corners_; }

  friend std::ostream& operator<<(std::ostream& out, const Box& box) {
    for (const auto& p : box.corners_) {
      out << p << "\n";
    }
    return out;
  }
};
}  // namespace TiEV