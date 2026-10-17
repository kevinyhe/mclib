// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/inertial.hpp"

#include "pros/error.h"

#include <cmath>

namespace mclib {
namespace device {

Inertial::Inertial(std::uint8_t port, double gain)
    // A gain of 0 or not finite would make every heading 0 and
    // setRotationDeg() divide by zero: use 1.
    : m_imu(port), m_gain(std::isfinite(gain) && gain > 0 ? gain : 1.0) {}

void Inertial::reset(bool blocking) {
  m_imu.reset(blocking);
}

double Inertial::getRotationDeg() const {
  const double raw = m_imu.get_rotation();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  return raw * m_gain;
}

double Inertial::getHeadingDeg() const {
  const double raw = m_imu.get_heading();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  // Wrap after the gain: PROS's heading is already [0, 360), and scaling
  // it moved 359 to 362.6 with a gain of 1.01.
  const double heading = std::fmod(raw * m_gain, 360.0);
  return heading < 0 ? heading + 360.0 : heading;
}

void Inertial::setRotationDeg(double heading_deg) {
  m_imu.set_rotation(heading_deg / m_gain);
}

std::optional<units::QAngle> Inertial::rotation() const {
  const double degrees = getRotationDeg();
  if (!std::isfinite(degrees)) {
    return std::nullopt;
  }
  return degrees * units::degree;
}

std::optional<units::QAngle> Inertial::heading() const {
  const double degrees = getHeadingDeg();
  if (!std::isfinite(degrees)) {
    return std::nullopt;
  }
  return degrees * units::degree;
}

void Inertial::setRotation(units::QAngle heading) {
  setRotationDeg(heading.deg());
}

}  // namespace device
}  // namespace mclib
