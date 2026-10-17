// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/types.hpp"

#include "mclib/units/units.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace mclib {
namespace device {

/**
 * @brief Wrapper for a V5 controller: sticks, buttons, screen, rumble and
 *        battery.
 *
 * The screen has 3 lines of 15 characters (line 0-2, column 0-14). The
 * controller takes a screen or rumble update about every 50 ms; PROS drops
 * writes sent faster than that, so a loop that writes every tick should
 * write only when the text changes.
 */
class Controller {
public:
  explicit Controller(ControllerId id = ControllerId::Master);

  /// @brief True when the controller is connected to the brain.
  bool isConnected() const;

  /// @brief Stick position, raw -127..127.
  std::int32_t getAnalog(AnalogAxis axis) const;

  /**
   * @brief Stick position as a -1.0..1.0 fraction of full deflection.
   *
   * Sibling of getAnalog(), which returns the raw -127..127 count. A stick is
   * dimensionless, so this buys no dimensional safety - what it buys is that
   * its range matches Motor::setPercent()'s fraction convention, so
   * `motor.setPercent(controller.analog(axis))` is right by construction
   * instead of 127x too big.
   */
  units::QNumber analog(AnalogAxis axis) const;

  /// @brief True while @p button is held down.
  bool getDigital(DigitalButton button) const;

  /**
   * @brief True once per press: the first call after @p button goes down.
   *
   * PROS keeps one edge flag per button per task, so call it from one place
   * (one task) per button, or presses are split between the callers.
   */
  bool getNewPress(DigitalButton button) const;

  /// @brief True once per release, the first call after @p button comes up.
  ///        Same one-caller rule as getNewPress().
  bool getNewRelease(DigitalButton button) const;

  /// @brief Write @p text at (@p line, @p col). True when PROS accepted it.
  bool setText(std::uint8_t line, std::uint8_t col, const char* text);
  bool setText(std::uint8_t line, std::uint8_t col, const std::string& text);

  /// @brief printf-style setText(). Longer than 15 characters is cut.
  bool print(std::uint8_t line, std::uint8_t col, const char* format, ...)
      __attribute__((format(printf, 4, 5)));

  /// @brief Blank one line of the screen.
  bool clearLine(std::uint8_t line);

  /// @brief Blank the whole screen.
  bool clear();

  /**
   * @brief Rumble a pattern: '.' short, '-' long, ' ' pause, up to 8
   *        characters, e.g. ".-." .
   */
  bool rumble(const char* pattern);

  /// @brief Controller battery charge, percent, or nothing when it can't be
  ///        read (disconnected).
  std::optional<std::int32_t> batteryCapacity() const;

  /// @brief Controller battery level as PROS reports it, or nothing when it
  ///        can't be read.
  std::optional<std::int32_t> batteryLevel() const;

  /// @brief Which controller this is.
  ControllerId id() const { return m_id; }

private:
  ControllerId m_id;
  mutable pros::Controller m_controller;
};

}  // namespace device
}  // namespace mclib
