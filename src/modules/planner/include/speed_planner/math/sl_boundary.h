#pragma once

namespace TiEV {

/**
 * @class
 * @brief SL boundary in Frenet cord system
 */
class SLBoundary {
private:
    double start_s_;
    double end_s_;
    double start_l_;
    double end_l_;

public:
    SLBoundary() = default;

    SLBoundary(double start_s, double end_s, double start_l, double end_l)
        : start_s_(start_s),
        end_s_(end_s),
        start_l_(start_l),
        end_l_(end_l) {}

    double start_s() { return start_s_; }

    double end_s() { return end_s_ ;}

    double start_l() { return start_l_; }

    double end_l() { return end_l_; }

    void set_start_s(double start_s) { start_s_ = start_s; }

    void set_end_s(double end_s) { end_s_ = end_s; }

    void set_start_l(double start_l) { start_l_ = start_l; }

    void set_end_l(double end_l) { end_l_ = end_l; }
};
}