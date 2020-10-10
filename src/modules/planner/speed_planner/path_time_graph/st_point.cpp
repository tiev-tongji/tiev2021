#include "st_point.h"

namespace TiEV {
    STPoint::STPoint() {
        s_ = 0.0;
        t_ = 0.0;
    }

    STPoint::STPoint(double s, double t) {
        s_ = s;
        t_ = t;
    }

    void STPoint::set_s(double s) {
        s_ = s;
    }

    void STPoint::set_t(double t) {
        t_ = t;
    }

    double STPoint::s() const { return s_; }

    double STPoint::t() const { return t_; }
}
