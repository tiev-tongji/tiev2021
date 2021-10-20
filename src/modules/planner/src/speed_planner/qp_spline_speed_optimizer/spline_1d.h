#pragma once

#include <vector>

#include "polynomial_xd.h"
#include "affine_constraint.h"
#include "spline_1d_seg.h"

namespace TiEV {

class Spline1d {
public:
    Spline1d() = default;

    Spline1d(const std::vector<double>& x_knots, const uint32_t order);

    // @brief: given x return f(x) value, derivative, second order derivative and
    // the third order;
    double operator()(const double x) const;
    double Derivative(const double x) const;
    double SecondOrderDerivative(const double x) const;
    double ThirdOrderDerivative(const double x) const;

    // @brief: set spline segments
    bool SetSplineSegs(const Eigen::MatrixXd& param_matrix, const uint32_t order);

    const std::vector<double>& x_knots() const;
    uint32_t spline_order() const;

    const std::vector<Spline1dSeg>& splines() const;

    private:
    uint32_t FindIndex(const double x) const;

    private:
    std::vector<Spline1dSeg> splines_;
    std::vector<double> x_knots_;
    uint32_t spline_order_;
};
}
