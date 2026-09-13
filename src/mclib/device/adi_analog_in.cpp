// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_analog_in.hpp"

#include "pros/error.h"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace device {

namespace {
constexpr std::int32_t kAdcMax = 4095;
}

AdiAnalogIn::AdiAnalogIn(char port)
    : m_input(static_cast<std::uint8_t>(port)) {}

AdiAnalogIn::AdiAnalogIn(std::uint8_t expander_port, char port)
    : m_input(pros::adi::ext_adi_port_pair_t{
          expander_port, static_cast<std::uint8_t>(port)}) {}

std::int32_t AdiAnalogIn::getRaw() const {
  return m_input.get_value();
}

std::optional<std::int32_t> AdiAnalogIn::raw() const {
  const std::int32_t value = m_input.get_value();
  if (value == PROS_ERR) {
    return std::nullopt;
  }
  return value;
}

std::int32_t AdiAnalogIn::calibrate() {
  return m_input.calibrate();
}

std::int32_t AdiAnalogIn::getCalibrated() const {
  return m_input.get_value_calibrated();
}

std::int32_t AdiAnalogIn::getCalibratedHighRes() const {
  return m_input.get_value_calibrated_HR();
}

double AdiAnalogIn::fraction() const {
  const std::optional<std::int32_t> value = raw();
  if (!value) {
    return NAN;
  }
  const std::int32_t clamped = std::clamp<std::int32_t>(*value, 0, kAdcMax);
  return static_cast<double>(clamped) / static_cast<double>(kAdcMax);
}

}  // namespace device
}  // namespace mclib
