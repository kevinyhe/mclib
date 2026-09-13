// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/optical.hpp"

#include "pros/error.h"

#include <cmath>

namespace mclib {
namespace device {

Optical::Optical(std::uint8_t port) : m_optical(port) {}

double Optical::getHue() const {
  return m_optical.get_hue();
}

double Optical::getSaturation() const {
  return m_optical.get_saturation();
}

double Optical::getBrightness() const {
  return m_optical.get_brightness();
}

std::int32_t Optical::getProximity() const {
  return m_optical.get_proximity();
}

pros::c::optical_rgb_s_t Optical::getRgb() const {
  return m_optical.get_rgb();
}

std::optional<double> Optical::hue() const {
  const double value = m_optical.get_hue();
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<double> Optical::saturation() const {
  const double value = m_optical.get_saturation();
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<double> Optical::brightness() const {
  const double value = m_optical.get_brightness();
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<std::int32_t> Optical::proximity() const {
  const std::int32_t value = m_optical.get_proximity();
  if (value == PROS_ERR) {
    return std::nullopt;
  }
  return value;
}

std::optional<Optical::Rgb> Optical::rgb() const {
  const pros::c::optical_rgb_s_t raw = m_optical.get_rgb();
  if (!std::isfinite(raw.red) || !std::isfinite(raw.green) ||
      !std::isfinite(raw.blue) || !std::isfinite(raw.brightness)) {
    return std::nullopt;
  }
  return Rgb{raw.red, raw.green, raw.blue, raw.brightness};
}

bool Optical::isColor(double hue_min, double hue_max,
                      std::int32_t min_proximity) const {
  const std::optional<std::int32_t> near = proximity();
  if (!near || *near < min_proximity) {
    return false;
  }
  const std::optional<double> h = hue();
  if (!h) {
    return false;
  }
  if (hue_min <= hue_max) {
    return *h >= hue_min && *h <= hue_max;
  }
  // Range crosses 360 -> 0, e.g. [340, 20]: either side of the seam counts.
  return *h >= hue_min || *h <= hue_max;
}

std::int32_t Optical::setLedPwm(std::uint8_t pwm) {
  return m_optical.set_led_pwm(pwm);
}

std::int32_t Optical::getLedPwm() const {
  return m_optical.get_led_pwm();
}

std::int32_t Optical::setIntegrationTimeMs(double ms) {
  return m_optical.set_integration_time(ms);
}

double Optical::getIntegrationTimeMs() const {
  return m_optical.get_integration_time();
}

std::int32_t Optical::enableGesture() {
  return m_optical.enable_gesture();
}

std::int32_t Optical::disableGesture() {
  return m_optical.disable_gesture();
}

Optical::Gesture Optical::gesture() const {
  switch (m_optical.get_gesture()) {
    case pros::c::UP:
      return Gesture::Up;
    case pros::c::DOWN:
      return Gesture::Down;
    case pros::c::LEFT:
      return Gesture::Left;
    case pros::c::RIGHT:
      return Gesture::Right;
    default:
      return Gesture::None;
  }
}

}  // namespace device
}  // namespace mclib
