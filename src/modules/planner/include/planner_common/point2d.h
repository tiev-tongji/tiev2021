#ifndef POINT2D_H
#define POINT2D_H
#include "const.h"
#include <iostream>
#include <math.h>
namespace TiEV {
struct Point2d {
    double x;
    double y;
    Point2d(double x_ = 0, double y_ = 0) : x(x_), y(y_){};

    inline bool in_map() const {
        return !(x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL);
    }
    inline double dot(const Point2d other) const {
        return x * other.x + y * other.y;
    };
    inline double cross(const Point2d other) const {
        return x * other.y - y * other.x;
    };
    inline double len() const {
        return sqrt(sqrLen());
    };
    inline double sqrLen() const {
        return x * x + y * y;
    };
    inline double getRad() const {
        return atan2(y, x);
    }
    inline Point2d operator*(const double k) const {
        return Point2d(x * k, y * k);
    };
    inline Point2d operator+(const Point2d other) const {
        return Point2d(x + other.x, y + other.y);
    };
    inline Point2d operator-(const Point2d other) const {
        return Point2d(x - other.x, y - other.y);
    };
    friend std::ostream& operator<<(std::ostream& out, const Point2d& point) {
        out << "Point2d:{x=" << point.x << " y=" << point.y << "}";
        return out;
    }
};

}  // namespace TiEV

#endif