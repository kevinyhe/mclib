// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "pros/adi.hpp"

#include <cstdint>

namespace mclib {
namespace device {

/**
 * @brief Wrapper for an addressable (WS2812) LED strip on a 3-wire port.
 *
 * Colours are 0xRRGGBB, e.g. 0xFF0000 is red. PROS caps a strip at 64
 * LEDs per port; a longer `length` is an error at construction.
 *
 * Every set/clear call writes the whole strip out to the port. Writing is
 * slow relative to a control loop (about 1 ms per 64 LEDs and it holds the
 * ADI bus), so change the strip when something happens, not every tick.
 */
class AdiLed {
public:
  /**
   * @param port    ADI port the strip's data wire is in.
   * @param length  Number of LEDs on the strip, 1..64.
   */
  AdiLed(char port, std::uint32_t length);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiLed(std::uint8_t expander_port, char port, std::uint32_t length);

  /// @brief Set every LED to `rgb` (0xRRGGBB). PROS_ERR on failure.
  std::int32_t setAll(std::uint32_t rgb);

  /**
   * @brief Set one LED. Same argument order as PROS (colour first, then
   *        index) so code ported from pros::adi::Led keeps working - both
   *        arguments are uint32, so a swap compiles without complaint.
   * @param index 0-based position on the strip.
   */
  std::int32_t setPixel(std::uint32_t rgb, std::uint32_t index);

  /// @brief Turn every LED off.
  std::int32_t clear();

  /// @brief Turn one LED off.
  std::int32_t clearPixel(std::uint32_t index);

  /// @brief Number of LEDs this strip was constructed with.
  std::uint32_t length() const;

private:
  pros::adi::Led m_led;
  std::uint32_t m_length;
};

}  // namespace device
}  // namespace mclib
