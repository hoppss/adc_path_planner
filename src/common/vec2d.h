/**
 * @file
 * @brief Defines the Vec2d class.
 */

#pragma once

#include <cmath>
#include <string>
#include "absl/strings/str_cat.h"

namespace common {

constexpr double kMathEpsilon = 1e-10;

class Vec2d {
 public:
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  static Vec2d CreateUnitVec2d(const double angle);

  double x() const { return x_; }
  double y() const { return y_; }

  void set_x(const double x) { x_ = x; }
  void set_y(const double y) { y_ = y; }

  double Length() const;

  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2d &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2d &other) const;

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Vec2d &other) const;

  //! Returns the inner product between these two Vec2d.
  double InnerProd(const Vec2d &other) const;

  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle);

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d &other) const;

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d &other) const;

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(const double ratio) const;

  //! Divides Vec2d by a scalar
  Vec2d operator/(const double ratio) const;

  //! Sums another Vec2d to the current one
  Vec2d &operator+=(const Vec2d &other);

  //! Subtracts another Vec2d to the current one
  Vec2d &operator-=(const Vec2d &other);

  //! Multiplies this Vec2d by a scalar
  Vec2d &operator*=(const double ratio);

  Vec2d &operator/=(const double ratio);

  bool operator==(const Vec2d &other) const;

  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d &vec);

}  // namespace common