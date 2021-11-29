#ifndef MATH_UTIL_H
#define MATH_UTIL_H
#include <math.h>

namespace math_util {
static double normalizeAngleRad(const double angle) {
    double a = std::fmod(angle + M_PI, M_2_PI);
    if(a < 0) {
        a += M_2_PI;
    }
    return a - M_PI;
}
}  // namespace math_util
#endif
