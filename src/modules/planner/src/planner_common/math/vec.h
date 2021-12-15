#pragma once

#include <iostream>
#include <string>

namespace TiEV {
class Vec {
 public:
  //! Constructor which takes x- and y-coordinates.
  Vec(const double x, const double y) : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  Vec() : Vec(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec CreateUnitVec(const double angle);

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec &other) const;

  //! Returns the "cross" product between these two Vec (non-standard).
  double CrossProd(const Vec &other) const;

  //! Returns the inner product between these two Vec.
  double InnerProd(const Vec &other) const;

  //! rotate the vector by angle.
  Vec rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle);

  //! Sums two Vec
  Vec operator+(const Vec &other) const;

  //! Subtracts two Vec
  Vec operator-(const Vec &other) const;

  //! Multiplies Vec by a scalar
  Vec operator*(const double ratio) const;

  //! Divides Vec by a scalar
  Vec operator/(const double ratio) const;

  //! Sums another Vec to the current one
  Vec &operator+=(const Vec &other);

  //! Subtracts another Vec to the current one
  Vec &operator-=(const Vec &other);

  //! Multiplies this Vec by a scalar
  Vec &operator*=(const double ratio);

  //! Divides this Vec by a scalar
  Vec &operator/=(const double ratio);

  //! Compares two Vec
  bool operator==(const Vec &other) const;

  //! Returns a human-readable string representing this object
  std::string DebugString() const;

  friend std::ostream &operator<<(std::ostream &out, const Vec &v) {
    out << "x= " << v.x() << " y=" << v.y();
    return out;
  }

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec by a given scalar
Vec operator*(const double ratio, const Vec &vec);

}  // namespace TiEV