#pragma once

#include <cmath>

#include "pose.h"

namespace TiEV {

    double NormalizeAngle(const double angle);

/**
* @brief Linear interpolation between two points of type T
* @tparam T
* @param x0 The cord of the first point
* @param t0 The interpolation parameter of the first point
* @param x1 The cord of the second point
* @param t1 The interpolation parameter of the second point
* @param t The cord of the interpolated point
* @return Interpolated point
*/
template <typename T> T lerp(const T& x0, const double t0, const T& x1, const double t1, const double t) {
    if(std::abs(t1 - t0) <= 1.0e-6) {
        return x0;
    }
    const double r = (t - t0) / (t1 - t0);
    const T      x = x0 + r * (x1 - x0);
    return x;
}

/**
* @brief Spherical linear interpolation between two angles [-M_PI, M_PI
* @param a0 The first angle
* @param t0 The interpolation parameter of the first angle
* @param a1 The second angle
* @param t1 The interpolation parameter of the second angle
* @param t The interpolation parameter for interpolation
* @return Interpolated angle
*/
double slerp(const double a0, const double t0, const double a1, const double t1, const double t);

/**
* @brief Linear interpolation between two path point using parameter t
* @param p0 The first path point
* @param p1 The second path point
* @param t The interpolation parameter t for interpolation
* @return Interpolated path point
*/
Pose InterpolateUsingLinearApproximationWithT(const Pose& p0, const Pose& p1, const double t);

/**
 * @brief Linear interpolation between two path point using parameter s
 * @param p0 The first path point
 * @param p1 The second path point
 * @param s The interpolation parameter s for interpolation
 * @return Interpolated path point
 */
Pose InterpolateUsingLinearApproximationWithS(const Pose& p0, const Pose& p1, const double s);
}
