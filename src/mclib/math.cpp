// mclib
#include "mclib/math.hpp"

#include <cmath>
#include <limits>

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

Pose2D compose(const Pose2D& base, const Pose2D& local) {
  const Vec2 offset = robotToField(Vec2{local.x, local.y}, base.theta);
  return Pose2D{base.x + offset.x(), base.y + offset.y(),
                wrapAngle(base.theta + local.theta)};
}

Vec2 headingVector(double rad) {
  return Vec2{std::sin(rad), std::cos(rad)};
}

double headingToward(const Vec2& from, const Vec2& to) {
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  if (dx == 0.0 && dy == 0.0) {
    return 0.0;
  }
  // Compass frame: x first, y second.
  return std::atan2(dx, dy);
}

Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad) {
  const double c = std::cos(heading_rad);
  const double s = std::sin(heading_rad);
  // Robot right axis is (cos, -sin); robot forward axis is (sin, cos).
  return Vec2{field_vec.x() * c - field_vec.y() * s,
              field_vec.x() * s + field_vec.y() * c};
}

Vec2 robotToField(const Vec2& robot_vec, double heading_rad) {
  const double c = std::cos(heading_rad);
  const double s = std::sin(heading_rad);
  return Vec2{robot_vec.x() * c + robot_vec.y() * s,
              -robot_vec.x() * s + robot_vec.y() * c};
}

Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose) {
  const Vec2 delta{field_point.x() - robot_pose.x, field_point.y() - robot_pose.y};
  return fieldToRobot(delta, robot_pose.theta);
}

Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose) {
  const Vec2 field_offset = robotToField(robot_point, robot_pose.theta);
  return Vec2{robot_pose.x + field_offset.x(), robot_pose.y + field_offset.y()};
}

double arcRadius(const Pose2D& from, const Vec2& target) {
  const Vec2 local = fieldPointToRobot(target, from);
  const double lateral = local.x();
  if (lateral == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return local.squaredNorm() / (2.0 * lateral);
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
