// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_potentiometer.hpp"

#include "pros/error.h"

#include <cmath>

namespace mclib {
namespace device {

namespace {
pros::adi_potentiometer_type_e_t toPros(PotentiometerType type) {
  switch (type) {
    case PotentiometerType::V1:
      return pros::E_ADI_POT_EDR;
    case PotentiometerType::V2:
    default:
      return pros::E_ADI_POT_V2;
  }
}
}  // namespace

AdiPotentiometer::AdiPotentiometer(char port, PotentiometerType type)
    : m_pot(static_cast<std::uint8_t>(port), toPros(type)) {}

AdiPotentiometer::AdiPotentiometer(std::uint8_t expander_port, char port,
                                   PotentiometerType type)
    : m_pot(pros::adi::ext_adi_port_pair_t{expander_port,
                                           static_cast<std::uint8_t>(port)},
            toPros(type)) {}

double AdiPotentiometer::getAngleDeg() const {
  return m_pot.get_angle();
}

std::optional<units::QAngle> AdiPotentiometer::angle() const {
  const double deg = m_pot.get_angle();
  if (!std::isfinite(deg)) {
    return std::nullopt;
  }
  return deg * units::degree;
}

std::int32_t AdiPotentiometer::getRaw() const {
  return m_pot.get_value();
}

std::int32_t AdiPotentiometer::calibrate() {
  return m_pot.calibrate();
}

std::int32_t AdiPotentiometer::getRawCalibrated() const {
  return m_pot.get_value_calibrated();
}

}  // namespace device
}  // namespace mclib
