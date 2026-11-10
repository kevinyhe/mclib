// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/odometry.hpp"
#include "mclib/control/pose_filter.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>

namespace mclib {
namespace control {

/**
 * @brief Runs a `PoseFusion` on the odometry pose and writes its correction
 *        back into odometry.
 *
 * The odometry task steps one for `OdometrySetup::fusion`. The physics
 * simulator steps the same class, so it shows what the robot will do.
 *
 * A pose that was set, not driven, restarts the filter at the new pose:
 * `Chassis::setPose()`, `wallReset()`, a snapshot. Stepping the filter across
 * the jump would count it as distance driven, and carry the correction the
 * filter still owed the old pose over to the new one.
 *
 * The restart is no surer of the position than the filter was before it. A
 * jump says where the robot is, not how well that is known: `wallReset()`
 * fixes the coordinate along the wall's normal and takes the other as given.
 * Both coordinates restart at the filter's larger position sigma, or 0.5 in
 * if that is more. Heading restarts at `PoseFusion::reset()`'s 0.5 deg: the
 * filter corrects position only, and heading stays with the IMU.
 */
class OdometryFusion {
 public:
  /// @brief Starts @p fusion at the pose odometry has now. Not owned; it must
  ///        outlive this object.
  explicit OdometryFusion(PoseFusion& fusion) : m_fusion(&fusion) {
    const OdometryReading reading = readOdometry();
    m_jumps = reading.jumps;
    m_fusion->reset(reading.pose, kStartPositionSigmaIn);
  }

  /**
   * @brief One tick. Call once after each `odometryTick()`.
   * @param now_ms Clock, for the sensors' periods.
   */
  void step(std::uint32_t now_ms) {
    const OdometryReading reading = readOdometry();
    if (reading.jumps != m_jumps) {
      m_jumps = reading.jumps;
      m_fusion->reset(reading.pose,
                      std::max(kStartPositionSigmaIn, m_fusion->positionSigmaIn()));
      return;
    }
    const std::optional<Vec2> corrected = m_fusion->step(reading.pose, now_ms);
    if (corrected.has_value()) {
      correctOdometryPosition(corrected->x(), corrected->y(), reading.jumps);
    }
  }

 private:
  static constexpr double kStartPositionSigmaIn = 0.5;

  PoseFusion* m_fusion;
  std::uint32_t m_jumps = 0;
};

}  // namespace control
}  // namespace mclib
