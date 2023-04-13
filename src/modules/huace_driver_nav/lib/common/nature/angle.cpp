#include "angle.h"
#include <cmath>

namespace TiEV{

const double Angle::PI = 3.1415926535898;

void Angle::setByDegree(double angle, const double offsetOfeast, const Dirction D) {
    if (D == CLOCKWISE) {
        angle = -angle;
    }
    val = (angle + offsetOfeast) / 180 * PI;
}

void Angle::setByRad(double angle, const double offsetOfeast, const Dirction D) {
    if (D == CLOCKWISE) {
        angle = -angle;
    }
    val = angle + offsetOfeast;
}

double Angle::getDegree() const {
    return val / PI * 180;
}

double Angle::getRad() const {
    return val;
}

double Angle::cos() const {
    return std::cos(val);
}

double Angle::sin() const {
    return std::sin(val);
}

double Angle::tan() const {
    return std::tan(val);
}

}
