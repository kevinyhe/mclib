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
 * @brief Wrapper for the 3-wire ultrasonic range finder.
 *
 * It uses two ADI ports: the OUTPUT (ping) wire and the INPUT (echo) wire,
 * in adjacent ports starting on an odd one (A/B, C/D, ...). Readings are
 * centimetres, roughly 3 cm to 300 cm, and are unreliable on soft or
 * round targets.
 */
class AdiUltrasonic {
public:
  /**
   * @param output_port  ADI port of the OUTPUT (ping) wire.
   * @param input_port   ADI port of the INPUT (echo) wire.
   */
  AdiUltrasonic(char output_port, char input_port);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiUltrasonic(std::uint8_t expander_port, char output_port, char input_port);

  /**
   * @brief Distance in centimetres exactly as PROS reports it: 0 or -1 when
   *        nothing echoed back, PROS_ERR on a bad port.
   */
  std::int32_t getDistanceCm() const;

  /**
   * @brief Typed sibling of getDistanceCm(): nullopt when there is no
   *        reading at all - PROS_ERR, or the 0 / -1 the sensor gives when
   *        no echo returned.
   *
   * Unlike the V5 Distance sensor, "nothing in range" here is not a big
   * number, it is zero. Left raw that reads as "object touching the sensor",
   * which is the opposite of the truth. A real reading is never below the
   * sensor's 3 cm minimum, so anything at or below 0 is treated as no reading.
   */
  std::optional<units::QLength> distance() const;

private:
  pros::adi::Ultrasonic m_ultrasonic;
};

}  // namespace device
}  // namespace mclib
