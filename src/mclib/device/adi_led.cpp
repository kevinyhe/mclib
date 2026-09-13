// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/adi_led.hpp"

namespace mclib {
namespace device {

AdiLed::AdiLed(char port, std::uint32_t length)
    : m_led(static_cast<std::uint8_t>(port), length), m_length(length) {}

AdiLed::AdiLed(std::uint8_t expander_port, char port, std::uint32_t length)
    : m_led(pros::adi::ext_adi_port_pair_t{expander_port,
                                           static_cast<std::uint8_t>(port)},
            length),
      m_length(length) {}

std::int32_t AdiLed::setAll(std::uint32_t rgb) {
  return m_led.set_all(rgb);
}

std::int32_t AdiLed::setPixel(std::uint32_t rgb, std::uint32_t index) {
  return m_led.set_pixel(rgb, index);
}

std::int32_t AdiLed::clear() {
  return m_led.clear_all();
}

std::int32_t AdiLed::clearPixel(std::uint32_t index) {
  return m_led.clear_pixel(index);
}

std::uint32_t AdiLed::length() const {
  return m_length;
}

}  // namespace device
}  // namespace mclib
