#pragma once

#include "linear_interpolation.h"
#include "st_boundary.h"
#include "tiev_class.h"

namespace TiEV {
/**
 *@file
 *@brief class Obstacle definition for temporary use in speed planning and
 *lattice planning
 **/

class Obstacle : public DynamicObj {
 private:
  STBoundary st_boundary_;

 public:
  Obstacle() = default;

  virtual ~Obstacle() = default;

  // constructor for speed_planner
  explicit Obstacle(const DynamicObj& obj) : DynamicObj(obj), st_boundary_() {
    for (auto& point : path) {
      point.ang = NormalizeAngle(PI - point.ang);
    }
  }

  // constructor for lattice_planner
  explicit Obstacle(const DynamicObj& obj, std::string type)
      : DynamicObj(obj) {}

  void set_st_boundary(const STBoundary& st_boundary) {
    st_boundary_ = st_boundary;
  }

  const STBoundary& st_boundary() const { return st_boundary_; }

  friend std::ostream& operator<<(std::ostream& out, const Obstacle& obs) {
    out << "x=" << obs.path.front().x << " y=" << obs.path.front().y
        << " v=" << obs.path.front().v << " ang=" << obs.path.front().ang
        << " width=" << obs.width << " length=" << obs.length << "\n";
    return out;
  }
};

}  // namespace TiEV
