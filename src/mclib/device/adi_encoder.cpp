// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_encoder.hpp"

#include "pros/error.h"

namespace mclib {
namespace device {

AdiEncoder::AdiEncoder(char top_port, char bottom_port, bool reversed)
    : m_encoder(static_cast<std::uint8_t>(top_port),
                static_cast<std::uint8_t>(bottom_port), reversed) {}

AdiEncoder::AdiEncoder(std::uint8_t expander_port, char top_port,
                       char bottom_port, bool reversed)
    : m_encoder(pros::adi::ext_adi_port_tuple_t{
                    expander_port, static_cast<std::uint8_t>(top_port),
                    static_cast<std::uint8_t>(bottom_port)},
                reversed) {}

std::int32_t AdiEncoder::getTicks() const {
  return m_encoder.get_value();
}

double AdiEncoder::getPositionDeg() const {
  return static_cast<double>(m_encoder.get_value());
}

std::optional<units::QAngle> AdiEncoder::position() const {
  const std::int32_t ticks = m_encoder.get_value();
  if (ticks == PROS_ERR) {
    return std::nullopt;
  }
  return static_cast<double>(ticks) * units::degree;
}

std::int32_t AdiEncoder::reset() {
  return m_encoder.reset();
}

}  // namespace device
}  // namespace mclib
