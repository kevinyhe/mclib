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
 * @brief Wrapper for the 3-wire optical shaft encoder (quadrature encoder).
 *
 * The encoder uses two adjacent ADI ports: top wire in one, bottom wire in
 * the next. It produces 360 ticks per revolution, so a tick is one degree
 * and getPositionDeg() is the raw count. The count is cumulative and does
 * not wrap, which is what an odometry tracking wheel needs: feed position()
 * to `DriveGeometry::encoderToDistance()` exactly like a Rotation sensor.
 *
 * `reversed` flips the sign so wheel-forward reads positive regardless of
 * which way the encoder is mounted.
 */
class AdiEncoder {
public:
  /**
   * @param top_port     ADI port of the top wire, 'A'..'H'.
   * @param bottom_port  ADI port of the bottom wire; must be the next port
   *                     up from top_port and the pair must start on an odd
   *                     port (A/B, C/D, E/F, G/H).
   */
  AdiEncoder(char top_port, char bottom_port, bool reversed = false);

  /// @brief Same, on a 3-wire expander plugged into `expander_port`.
  AdiEncoder(std::uint8_t expander_port, char top_port, char bottom_port,
             bool reversed = false);

  /// @brief Cumulative ticks since reset(). PROS_ERR on failure.
  std::int32_t getTicks() const;

  /// @brief Cumulative degrees (1 tick == 1 degree). PROS_ERR on failure.
  double getPositionDeg() const;

  /**
   * @brief Typed sibling of getPositionDeg(): nullopt when the port is not
   *        reporting.
   *
   * A missing sensor comes back from PROS as INT32_MAX ticks. Differenced by
   * odometry that is a 2-billion-degree jump in the pose; nullopt makes the
   * caller drop the sample instead.
   */
  std::optional<units::QAngle> position() const;

  /// @brief Zero the count.
  std::int32_t reset();

private:
  pros::adi::Encoder m_encoder;
};

}  // namespace device
}  // namespace mclib
