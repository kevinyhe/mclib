// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/rotation.hpp"

#include "pros/error.h"

namespace mclib {
namespace device {

namespace {
std::int8_t signedPort(std::int8_t port, bool reversed) {
  const std::int8_t absolute = port < 0 ? -port : port;
  return (reversed || port < 0) ? -absolute : absolute;
}
}  // namespace

Rotation::Rotation(std::int8_t port, bool reversed)
    : m_rotation(signedPort(port, reversed)) {}

double Rotation::getPositionDeg() const {
  return m_rotation.get_position() / 100.0;
}

std::optional<units::QAngle> Rotation::position() const {
  const std::int32_t centidegrees = m_rotation.get_position();
  if (centidegrees == PROS_ERR) {
    return std::nullopt;
  }
  return (centidegrees / 100.0) * units::degree;
}

void Rotation::resetPosition() {
  m_rotation.reset_position();
}

bool Rotation::isInstalled() const {
  return m_rotation.is_installed();
}

}  // namespace device
}  // namespace mclib
