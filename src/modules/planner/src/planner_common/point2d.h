#ifndef POINT2D_H
#define POINT2D_H
#include <math.h>

#include <iostream>

#include "const.h"
namespace TiEV {
struct Point2d {
  double x;
  double y;
  Point2d(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}

  inline bool in_map() const {
    return !(x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL);
  }
  inline const double dot(const Point2d other) const {
    return x * other.x + y * other.y;
  }
  inline const double cross(const Point2d other) const {
    return x * other.y - y * other.x;
  }
  inline Point2d ort(const Point2d& b) {
    Point2d a(this->x, this->y);
    Point2d c;
    if (b.len() == 0) return c;
    c = a - b * a.dot(b) / b.sqrLen();
    return c;
  }
  inline const double  len() const { return sqrt(sqrLen()); }
  inline const double  sqrLen() const { return x * x + y * y; }
  inline const double  getRad() const { return atan2(y, x); }
  inline const Point2d getDirection() const {
    return Point2d(x / len(), y / len());
  }
  inline Point2d operator*(const double k) const {
    return Point2d(x * k, y * k);
  }
  friend Point2d operator*(const double k, const Point2d& p) { return p * k; }
  inline Point2d operator/(const double k) const {
    return Point2d(x / k, y / k);
  }
  inline Point2d operator+(const Point2d other) const {
    return Point2d(x + other.x, y + other.y);
  }
  inline Point2d operator-(const Point2d other) const {
    return Point2d(x - other.x, y - other.y);
  }
  inline Point2d       operator-() const { return Point2d(-x, -y); }
  friend std::ostream& operator<<(std::ostream& out, const Point2d& point) {
    out << "Point2d:{x=" << point.x << " y=" << point.y << "}";
    return out;
  }
  inline const double cosDeltaAngle(const Point2d& other_point) const {
    double dot = this->dot(other_point);
    if (this->len() == 0 || other_point.len() == 0) return 0;
    double cos   = dot / (this->len() * other_point.len());
    return cos;
  }
};

class Segment {
 public:
  Segment() = default;
  Segment(const Point2d& start, const Point2d& end) {
    start_ = start;
    end_   = end;
    x_     = end.x - start.x;
    y_     = end.y - start.y;
  }
  Segment(const double& x, const double& y) : x_(x), y_(y){};

  const Point2d       start() { return start_; }
  const Point2d       end() { return end_; }
  const double        x() const { return x_; }
  const double        y() const { return y_; }
  inline const double dot(const Segment& other) const {
    return x_ * other.x() + y_ * other.y();
  };
  inline double cross(const Segment& other) const {
    return x_ * other.y() - y_ * other.x();
  };
  inline double  len() const { return sqrt(sqrLen()); };
  inline double  sqrLen() const { return x_ * x_ + y_ * y_; };
  inline double  getRad() const { return atan2(y_, x_); }
  inline Point2d getDirection() const {
    return Point2d(x_ / len(), y_ / len());
  }
  inline Segment operator*(const double k) const {
    return Segment(x_ * k, y_ * k);
  };
  inline Segment operator/(const double k) const {
    return Segment(x_ / k, y_ / k);
  }
  inline Segment operator+(const Segment& other) const {
    return Segment(x_ + other.x(), y_ + other.y());
  };
  inline Segment operator-(const Segment& other) const {
    return Segment(x_ - other.x(), y_ - other.y());
  };
  friend std::ostream& operator<<(std::ostream& out, const Segment& segment) {
    out << "Segment:{x=" << segment.x_ << " y=" << segment.y_ << "}";
    return out;
  }

 private:
  Point2d start_, end_;
  double  x_, y_;
};

}  // namespace TiEV

#endif