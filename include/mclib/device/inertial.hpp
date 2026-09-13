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

class Inertial {
public:
  explicit Inertial(std::uint8_t port, double gain = 1.0);

  void reset(bool blocking = false);
  double getRotationDeg() const;
  double getHeadingDeg() const;
  void setRotationDeg(double heading_deg);

  /**
   * @brief Unbounded rotation as a typed angle, or nullopt if the IMU is not
   *        reporting.
   *
   * Units-typed sibling of getRotationDeg(). That one returns NAN on a
   * dropout, which is a value the compiler is happy to let you add to a pose:
   * every downstream number silently becomes NaN and the robot drives on with
   * a poisoned position. A QAngle would propagate NaN exactly as quietly, so
   * the typed accessors return std::optional instead - a dropout is now
   * something the caller has to unwrap, and forgetting is a compile error
   * rather than a lost match.
   */
  std::optional<units::QAngle> rotation() const;

  /**
   * @brief Heading wrapped to [0, 360) as a typed angle, or nullopt if the IMU
   *        is not reporting. Typed sibling of getHeadingDeg().
   */
  std::optional<units::QAngle> heading() const;

  /// @brief Typed sibling of setRotationDeg().
  void setRotation(units::QAngle heading);

private:
  pros::Imu m_imu;
  double m_gain;
};

}  // namespace device
}  // namespace mclib
