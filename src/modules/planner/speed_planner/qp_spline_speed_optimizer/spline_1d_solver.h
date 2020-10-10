#pragma once

#include <vector>

#include <eigen3/Eigen/Core>

#include "spline_1d.h"
#include "spline_1d_constraint.h"
#include "spline_1d_kernel.h"

namespace TiEV {

class Spline1dSolver {
 public:
  Spline1dSolver(const std::vector<double>& x_knots, const uint32_t order)
      : spline_(x_knots, order),
        constraint_(x_knots, order),
        kernel_(x_knots, order) {}

  virtual void Reset(const std::vector<double>& x_knots, const uint32_t order) {
    spline_ = Spline1d(x_knots, order);
    constraint_ = Spline1dConstraint(x_knots, order);
    kernel_ = Spline1dKernel(x_knots, order);
  }

  virtual Spline1dConstraint* mutable_spline_constraint() {
    return &constraint_;
  }

  virtual Spline1dKernel* mutable_spline_kernel() { return &kernel_; }

  virtual bool Solve() = 0;

  // output
  virtual Spline1d spline() { return spline_; }

 protected:
  Spline1d spline_;
  Spline1dConstraint constraint_;
  Spline1dKernel kernel_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};
} 
