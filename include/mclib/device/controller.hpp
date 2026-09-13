// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/types.hpp"

#include "mclib/units/units.hpp"

#include <cstdint>

namespace mclib {
namespace device {

class Controller {
public:
  explicit Controller(ControllerId id = ControllerId::Master);

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
  bool getDigital(DigitalButton button) const;
  void setText(std::uint8_t line, std::uint8_t col, const char* text);

private:
  mutable pros::Controller m_controller;
};

}  // namespace device
}  // namespace mclib
