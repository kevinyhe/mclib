// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include "api.h"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

/**
 * @brief Wrapper for the V5 GPS sensor, reporting in mclib's field frame.
 *
 * PROS gives position in metres with (0, 0) at field centre and heading in
 * degrees, 0 = north, clockwise positive. mclib's frame (see math.hpp) is
 * the same picture in inches and radians: 0 = +Y, clockwise positive. So the
 * conversion is unit scaling only; no axis flips.
 *
 * Offsets: the sensor is rarely at the robot's turning centre. Give the
 * offset in the robot frame, inches, +X to the robot's right and +Y forward,
 * and PROS corrects the reported position for it.
 *
 * Heading offset: PROS has no way to say "the sensor is mounted facing the
 * robot's left". `setHeadingOffsetDeg()` handles that in the wrapper: the
 * value is added to every heading read and removed from every heading write.
 */
class Gps {
public:
  /**
   * @param port         Smart port 1..21.
   * @param x_offset_in  Sensor position right of the turning centre, inches.
   * @param y_offset_in  Sensor position forward of the turning centre, inches.
   */
  explicit Gps(std::uint8_t port, double x_offset_in = 0.0,
               double y_offset_in = 0.0);

  /**
   * @brief Same, and also tells the sensor where the robot starts. The GPS
   *        can locate itself from the field strips alone, but seeding the
   *        pose makes the first readings usable before it has a full lock.
   */
  Gps(std::uint8_t port, const Pose2D& initial_pose, double x_offset_in = 0.0,
      double y_offset_in = 0.0);

  /**
   * @brief Field pose in inches and compass radians, or nullopt when any
   *        of x, y or heading came back as PROS_ERR_F.
   *
   * One bad component poisons the whole pose, so it is all or nothing:
   * odometry that fuses this must not get a real x with an infinite heading.
   */
  std::optional<Pose2D> pose() const;

  /// @brief Heading [0, 360) degrees, offset applied. PROS_ERR_F on failure.
  double getHeadingDeg() const;
  /// @brief Typed sibling of getHeadingDeg().
  std::optional<units::QAngle> heading() const;

  /// @brief X in inches from field centre. PROS_ERR_F on failure.
  double getXIn() const;
  /// @brief Y in inches from field centre. PROS_ERR_F on failure.
  double getYIn() const;

  /**
   * @brief Position RMS error in metres, as PROS reports it. PROS_ERR_F on
   *        failure. Large values mean the sensor cannot see enough of the
   *        field strip; treat the pose as untrustworthy.
   */
  double getErrorM() const;
  /// @brief Typed sibling of getErrorM().
  std::optional<units::QLength> error() const;

  /// @brief Tell the sensor where the robot is now (inches, compass radians).
  std::int32_t setPose(const Pose2D& pose);

  /// @brief Update the sensor's mounting offset (robot frame, inches).
  std::int32_t setOffset(double x_offset_in, double y_offset_in);

  /**
   * @brief Degrees added to the sensor's heading so that a sensor mounted
   *        facing off-axis still reports the robot's heading. Mounted facing
   *        the robot's left means the sensor reads 90 less than the robot,
   *        so pass +90.
   */
  void setHeadingOffsetDeg(double offset_deg);
  double getHeadingOffsetDeg() const;

  /// @brief Update rate in ms; PROS clamps to a minimum of 5.
  std::int32_t setDataRateMs(std::uint32_t ms);

private:
  pros::Gps m_gps;
  double m_heading_offset_deg;
};

}  // namespace device
}  // namespace mclib
