// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

#include "pros/adi.hpp"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

/**
 * @brief Which potentiometer is plugged in. The two have different travel,
 *        so PROS needs to know to turn the raw reading into degrees.
 */
enum class PotentiometerType {
  V1,  ///< Original EDR potentiometer, about 250 degrees of travel.
  V2,  ///< Current V2 potentiometer, about 330 degrees of travel.
};

/**
 * @brief Wrapper for the 3-wire potentiometer.
 *
 * Reads an absolute angle: unlike an encoder it survives a power cycle
 * without resetting, which is why it is the usual sensor for an arm or
 * lift with a fixed range.
 *
 * Defaults to V2 because that is what is sold now. PROS itself defaults to
 * V1, so if you copy a port number from PROS code and the angle range looks
 * wrong, this is why.
 */
class AdiPotentiometer {
public:
  explicit AdiPotentiometer(char port,
                            PotentiometerType type = PotentiometerType::V2);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiPotentiometer(std::uint8_t expander_port, char port,
                   PotentiometerType type = PotentiometerType::V2);

  /// @brief Angle in degrees, 0 to the type's travel. PROS_ERR_F on failure.
  double getAngleDeg() const;

  /// @brief Typed sibling of getAngleDeg(): nullopt when not reporting.
  std::optional<units::QAngle> angle() const;

  /// @brief Raw 12-bit ADC value 0..4095. PROS_ERR on failure.
  std::int32_t getRaw() const;

  /**
   * @brief Sample the current position for ~500 ms and make it the zero of
   *        getRawCalibrated(). Blocks; call it during initialize().
   */
  std::int32_t calibrate();

  /// @brief Raw value minus the calibrate() zero, -4095..4095.
  std::int32_t getRawCalibrated() const;

private:
  pros::adi::Potentiometer m_pot;
};

}  // namespace device
}  // namespace mclib
