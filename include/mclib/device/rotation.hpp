// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

#include "api.h"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

class Rotation {
public:
  /// A negative port or reversed=true requests reversal; using both does not cancel it.
  explicit Rotation(std::int8_t port, bool reversed = false);

  double getPositionDeg() const;

  /**
   * @brief Sensor position as a typed angle, or nullopt when the sensor is not
   *        reporting.
   *
   * Units-typed sibling of getPositionDeg(): same value, same sign, same
   * unbounded (non-wrapping) accumulation. Feed it straight to
   * `DriveGeometry::encoderToDistance()`.
   *
   * getPositionDeg() turns a disconnected sensor's PROS_ERR into 21 474 836.47
   * degrees, which odometry differences into a 21-million-degree jump in the
   * pose. Same reasoning as Inertial: a dropout has to be unwrappable, not
   * silently numeric.
   */
  std::optional<units::QAngle> position() const;
  void resetPosition();
  bool isInstalled() const;

private:
  mutable pros::Rotation m_rotation;
};

}  // namespace device
}  // namespace mclib
