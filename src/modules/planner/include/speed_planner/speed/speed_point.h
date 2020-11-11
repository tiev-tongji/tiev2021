#pragma once

namespace TiEV {

class SpeedPoint {
private:
    double s_ = 0;
    double t_ = 0;
    double v_ = 0; // speed
    double a_ = 0; // acceleration
    double da_ = 0; // jerk

public:
    SpeedPoint() = default;

    SpeedPoint(double s, double t, double v, double a, double da) {
        s_ = s;
        t_ = t;
        v_ = v;
        a_ = a;
        da_ = da;
    }

    double s() const { return s_; }

    double t() const { return t_; }

    double v() const { return v_; }

    double a() const { return a_; }

    double da() const { return da_; }

    void set_s(double s) { s_ = s; }

    void set_t(double t) { t_ = t; }

    void set_v(double v) { v_ = v; }

    void set_a(double a) { a_ = a; }

    void set_da(double da) { da_ = da; }
};
}
