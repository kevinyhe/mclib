// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_digital_in.hpp"

namespace mclib {
namespace device {

AdiDigitalIn::AdiDigitalIn(char port)
    : m_input(static_cast<std::uint8_t>(port)), m_was_pressed(false) {}

AdiDigitalIn::AdiDigitalIn(std::uint8_t expander_port, char port)
    : m_input(pros::adi::ext_adi_port_pair_t{expander_port,
                                             static_cast<std::uint8_t>(port)}),
      m_was_pressed(false) {}

bool AdiDigitalIn::isPressed() const {
  // PROS_ERR is INT32_MAX, so a plain truthiness check would report an
  // unplugged switch as pressed. Compare to 1 exactly.
  return m_input.get_value() == 1;
}

bool AdiDigitalIn::newPress() {
  const bool pressed = isPressed();
  const bool edge = pressed && !m_was_pressed;
  m_was_pressed = pressed;
  return edge;
}

std::int32_t AdiDigitalIn::getValue() const {
  return m_input.get_value();
}

}  // namespace device
}  // namespace mclib
