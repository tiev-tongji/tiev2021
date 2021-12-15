#include "vec.h"

#include <cmath>

namespace TiEV {

#define kMathEpsilon 1e-6

Vec Vec::CreateUnitVec(const double angle) {
  return Vec(std::cos(angle), std::sin(angle));
}

double Vec::Length() const { return std::hypot(x_, y_); }

double Vec::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec::Angle() const { return std::atan2(y_, x_); }

void Vec::Normalize() {
  const double l = Length();
  if (l > kMathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec::DistanceTo(const Vec &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec::DistanceSquareTo(const Vec &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec::CrossProd(const Vec &other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec::InnerProd(const Vec &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec Vec::rotate(const double angle) const {
  return Vec(x_ * cos(angle) - y_ * sin(angle),
             x_ * sin(angle) + y_ * cos(angle));
}

void Vec::SelfRotate(const double angle) {
  double tmp_x = x_;
  x_           = x_ * cos(angle) - y_ * sin(angle);
  y_           = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec Vec::operator+(const Vec &other) const {
  return Vec(x_ + other.x(), y_ + other.y());
}

Vec Vec::operator-(const Vec &other) const {
  return Vec(x_ - other.x(), y_ - other.y());
}

Vec Vec::operator*(const double ratio) const {
  return Vec(x_ * ratio, y_ * ratio);
}

Vec Vec::operator/(const double ratio) const {
  return Vec(x_ / ratio, y_ / ratio);
}

Vec &Vec::operator+=(const Vec &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec &Vec::operator-=(const Vec &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec &Vec::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec &Vec::operator/=(const double ratio) {
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec::operator==(const Vec &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec operator*(const double ratio, const Vec &vec) { return vec * ratio; }

}  // namespace TiEV
