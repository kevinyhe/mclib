// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_ultrasonic.hpp"

#include "pros/error.h"

namespace mclib {
namespace device {

AdiUltrasonic::AdiUltrasonic(char output_port, char input_port)
    : m_ultrasonic(static_cast<std::uint8_t>(output_port),
                   static_cast<std::uint8_t>(input_port)) {}

AdiUltrasonic::AdiUltrasonic(std::uint8_t expander_port, char output_port,
                             char input_port)
    : m_ultrasonic(pros::adi::ext_adi_port_tuple_t{
          expander_port, static_cast<std::uint8_t>(output_port),
          static_cast<std::uint8_t>(input_port)}) {}

std::int32_t AdiUltrasonic::getDistanceCm() const {
  return m_ultrasonic.get_value();
}

std::optional<units::QLength> AdiUltrasonic::distance() const {
  const std::int32_t cm = m_ultrasonic.get_value();
  if (cm == PROS_ERR || cm <= 0) {
    return std::nullopt;
  }
  return static_cast<double>(cm) * units::centimetre;
}

}  // namespace device
}  // namespace mclib
