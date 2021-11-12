#pragma once

namespace TiEV {

/**
 * @class
 * @brief vector in Cartesian coordinate system
 */
class Vec {
private:
    double x_;
    double y_;

public:
    Vec() = default;

    Vec(double x, double y) { x_ = x; y_ = y; }

    void set_x(double x) { x_ = x; }

    void set_y(double y) { y_ = y; }

    double x() const { return x_; }

    double y() const { return y_; }
};

class Vec2d {
private:
    double x_;
    double y_;

public:
    Vec2d() = default;

    Vec2d(double x, double y) { x_ = x; y_ = y; }

    void set_x(double x) { x_ = x; }

    void set_y(double y) { y_ = y; }

    double x() const { return x_; }

    double y() const { return y_; }
};
}