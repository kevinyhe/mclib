// mclib
#pragma once

#include "Eigen/Core"

namespace mclib {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

double clamp(double val, double min, double max);
double wrapAngle(double rad);

struct Pose2D {
  double x;
  double y;
  double theta;

  constexpr Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  constexpr Pose2D(double x_in, double y_in, double theta_in)
      : x(x_in), y(y_in), theta(theta_in) {}

  Pose2D operator+(const Pose2D& other) const;
  double distanceTo(const Pose2D& other) const;

  Vec2 translation() const;
  Vec3 vector() const;
};

Mat2 rotationMatrix(double rad);
Vec2 rotate(const Vec2& vec, double rad);

}  // namespace mclib
