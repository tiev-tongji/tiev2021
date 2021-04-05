#include "math/linear_interpolation.h"

#include <cmath>
#include <iostream>

namespace TiEV {

constexpr double kMathEpsilon = 1e-10;

/**
* @brief Get a normalized angle: [-M_PI, +M_PI]
* @param angle
* @return Normalized angle
*/
double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if(a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double slerp(const double a0, const double t0, const double a1, const double t1, const double t) {
    if(std::abs(t1 - t0) <= kMathEpsilon) {
        return NormalizeAngle(a0);
    }
    const double a0_n = NormalizeAngle(a0);
    const double a1_n = NormalizeAngle(a1);
    double       d    = a1_n - a0_n;
    if(d > M_PI) {
        d = d - 2 * M_PI;
    }
    else if(d < -M_PI) {
        d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;
    return NormalizeAngle(a);
}

Pose InterpolateUsingLinearApproximationWithT(const Pose& p0, const Pose& p1, const double t) {
    double t0 = p0.t;
    double t1 = p1.t;

    Pose p;
    p.s   = lerp(p0.s, t0, p1.s, t1, t);
    p.t   = t;
    p.x   = lerp(p0.x, t0, p1.x, t1, t);
    p.y   = lerp(p0.y, t0, p1.y, t1, t);
    p.ang = slerp(p0.ang, t0, p1.ang, t1, t);
    p.k   = lerp(p0.k, t0, p1.k, t1, t);
    p.v   = lerp(p0.v, t0, p1.v, t1, t);
    p.a   = lerp(p0.a, t0, p1.a, t1, t);

    return p;
}

Pose InterpolateUsingLinearApproximationWithS(const Pose& p0, const Pose& p1, const double s) {
    double s0 = p0.s;
    double s1 = p1.s;

    Pose   path_point;
    double weight  = (s - s0) / (s1 - s0);
    double x       = (1 - weight) * p0.x + weight * p1.x;
    double y       = (1 - weight) * p0.y + weight * p1.y;
    double theta   = slerp(p0.ang, s0, p1.ang, s1, s);
    double kappa   = (1 - weight) * p0.k + weight * p1.k;
    path_point.x   = x;
    path_point.y   = y;
    path_point.ang = theta;
    path_point.k   = kappa;
    path_point.s   = s;

    return path_point;
}
}