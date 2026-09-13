// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "pros/adi.hpp"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

/**
 * @brief Wrapper for a generic 3-wire analog input (line tracker, light
 *        sensor, or anything else that puts a voltage on the signal pin).
 *
 * The V5 ADC is 12-bit, so raw readings run 0..4095. fraction() rescales
 * that to 0..1 so a threshold can be written as "0.6" instead of "2457".
 */
class AdiAnalogIn {
public:
  explicit AdiAnalogIn(char port);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiAnalogIn(std::uint8_t expander_port, char port);

  /// @brief Raw 12-bit reading 0..4095. PROS_ERR on failure.
  std::int32_t getRaw() const;

  /// @brief Typed sibling of getRaw(): nullopt when the port is not reporting.
  std::optional<std::int32_t> raw() const;

  /**
   * @brief Average the input for about 500 ms and store it as the zero for
   *        getCalibrated(). Blocks for that long; call it in initialize()
   *        with the sensor in its rest state.
   */
  std::int32_t calibrate();

  /// @brief Raw reading minus the calibrate() zero, -4095..4095. PROS_ERR on failure.
  std::int32_t getCalibrated() const;

  /**
   * @brief Same as getCalibrated() but with 4 extra bits of averaging,
   *        -16384..16384. Use it when integrating (e.g. an accelerometer)
   *        where the extra resolution reduces drift.
   */
  std::int32_t getCalibratedHighRes() const;

  /**
   * @brief Raw reading as a fraction of full scale, clamped to [0, 1].
   *        NAN when PROS reports an error, so a bad port does not read as
   *        "fully lit".
   */
  double fraction() const;

private:
  pros::adi::AnalogIn m_input;
};

}  // namespace device
}  // namespace mclib
