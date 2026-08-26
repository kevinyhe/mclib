// mclib
#include "mclib/math.hpp"

namespace mclib {

double clamp(double val, double min, double max) {
  if (min > max) {
    const double tmp = min;
    min = max;
    max = tmp;
  }

  if (val < min) {
    return min;
  }
  if (val > max) {
    return max;
  }
  return val;
}

double wrapAngle(double rad) {
  double wrapped = std::fmod(rad, kTwoPi);
  if (wrapped > kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped < -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

Pose2D Pose2D::operator+(const Pose2D& other) const {
  return Pose2D{x + other.x, y + other.y, wrapAngle(theta + other.theta)};
}

double Pose2D::distanceTo(const Pose2D& other) const {
  const double dx = other.x - x;
  const double dy = other.y - y;
  return std::sqrt(dx * dx + dy * dy);
}

Vec2 Pose2D::translation() const {
  return Vec2{x, y};
}

Vec3 Pose2D::vector() const {
  return Vec3{x, y, theta};
}

Mat2 rotationMatrix(double rad) {
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  Mat2 mat;
  mat << c, -s, s, c;
  return mat;
}

Vec2 rotate(const Vec2& vec, double rad) {
  return rotationMatrix(rad) * vec;
}

}  // namespace mclib
