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
 * @brief Wrapper for a 3-wire digital input: limit switch or bumper.
 *
 * Both devices are a plain switch, so one class covers them; `LimitSwitch`
 * and `Bumper` below are aliases for readability at the call site.
 *
 * newPress() is tracked here rather than through PROS's get_new_press().
 * PROS keeps one "was pressed" flag per port, shared by every caller, so
 * two loops polling the same switch would each see only some of the edges.
 * With the flag in the wrapper, each AdiDigitalIn object sees every edge
 * for itself. Poll it from one place per object.
 */
class AdiDigitalIn {
public:
  explicit AdiDigitalIn(char port);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiDigitalIn(std::uint8_t expander_port, char port);

  /// @brief True while the switch is held. False on a PROS error.
  bool isPressed() const;

  /**
   * @brief True once per press: the first call after the switch goes from
   *        released to pressed. Every later call returns false until it is
   *        released and pressed again.
   */
  bool newPress();

  /// @brief Raw PROS value: 1 pressed, 0 released, PROS_ERR on failure.
  std::int32_t getValue() const;

private:
  pros::adi::DigitalIn m_input;
  bool m_was_pressed;
};

using LimitSwitch = AdiDigitalIn;
using Bumper = AdiDigitalIn;

}  // namespace device
}  // namespace mclib
