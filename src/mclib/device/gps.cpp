// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/gps.hpp"

#include "pros/error.h"

#include <cmath>

namespace mclib {
namespace device {

namespace {

constexpr double kMetresPerInch = 0.0254;

double inchesToMetres(double inches) {
  return inches * kMetresPerInch;
}

double metresToInches(double metres) {
  return metres / kMetresPerInch;
}

/// Wrap degrees into [0, 360).
double wrapDeg360(double deg) {
  double wrapped = std::fmod(deg, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

}  // namespace

Gps::Gps(std::uint8_t port, double x_offset_in, double y_offset_in)
    : m_gps(port, inchesToMetres(x_offset_in), inchesToMetres(y_offset_in)),
      m_heading_offset_deg(0.0) {}

Gps::Gps(std::uint8_t port, const Pose2D& initial_pose, double x_offset_in,
         double y_offset_in)
    : m_gps(port, inchesToMetres(initial_pose.x), inchesToMetres(initial_pose.y),
            wrapDeg360(initial_pose.theta * 180.0 / kPi),
            inchesToMetres(x_offset_in), inchesToMetres(y_offset_in)),
      m_heading_offset_deg(0.0) {}

std::optional<Pose2D> Gps::pose() const {
  const double x_m = m_gps.get_position_x();
  const double y_m = m_gps.get_position_y();
  const double heading_deg = getHeadingDeg();
  if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(heading_deg)) {
    return std::nullopt;
  }
  return Pose2D(metresToInches(x_m), metresToInches(y_m),
                heading_deg * kPi / 180.0);
}

double Gps::getHeadingDeg() const {
  const double raw = m_gps.get_heading();
  if (!std::isfinite(raw)) {
    return raw;
  }
  return wrapDeg360(raw + m_heading_offset_deg);
}

std::optional<units::QAngle> Gps::heading() const {
  const double deg = getHeadingDeg();
  if (!std::isfinite(deg)) {
    return std::nullopt;
  }
  return deg * units::degree;
}

double Gps::getXIn() const {
  const double metres = m_gps.get_position_x();
  return std::isfinite(metres) ? metresToInches(metres) : metres;
}

double Gps::getYIn() const {
  const double metres = m_gps.get_position_y();
  return std::isfinite(metres) ? metresToInches(metres) : metres;
}

double Gps::getErrorM() const {
  return m_gps.get_error();
}

std::optional<units::QLength> Gps::error() const {
  const double metres = m_gps.get_error();
  if (!std::isfinite(metres)) {
    return std::nullopt;
  }
  return metres * units::metre;
}

std::int32_t Gps::setPose(const Pose2D& pose) {
  const double robot_heading_deg = pose.theta * 180.0 / kPi;
  return m_gps.set_position(inchesToMetres(pose.x), inchesToMetres(pose.y),
                            wrapDeg360(robot_heading_deg - m_heading_offset_deg));
}

std::int32_t Gps::setOffset(double x_offset_in, double y_offset_in) {
  return m_gps.set_offset(inchesToMetres(x_offset_in),
                          inchesToMetres(y_offset_in));
}

void Gps::setHeadingOffsetDeg(double offset_deg) {
  m_heading_offset_deg = offset_deg;
}

double Gps::getHeadingOffsetDeg() const {
  return m_heading_offset_deg;
}

std::int32_t Gps::setDataRateMs(std::uint32_t ms) {
  return m_gps.set_data_rate(ms);
}

}  // namespace device
}  // namespace mclib
