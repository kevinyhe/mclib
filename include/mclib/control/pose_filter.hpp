// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/math.hpp"
#include "mclib/snapshot/collision_map.hpp"
#include "mclib/snapshot/types.hpp"

#include <cstdint>

/**
 * @file pose_filter.hpp
 * @brief Blend odometry with distance sensors and GPS: an extended Kalman
 *        filter over (x, y, heading).
 *
 * Odometry is smooth but drifts: every slip adds error that never goes away.
 * A distance sensor aimed at a field wall, or the GPS sensor, gives an
 * absolute reading that doesn't drift but is noisy and sometimes wrong (a
 * robot or game object in the way). The filter keeps a pose and how uncertain
 * it is, and weighs each new reading against that uncertainty:
 *
 * - `predict()` moves the pose by each odometry step and grows the
 *   uncertainty with the distance travelled and the angle turned.
 * - `updateDistance()` compares a distance reading with the range
 *   `snapshot::predict_range()` expects from the current pose, and pulls the
 *   pose toward agreeing with it, by an amount set by both uncertainties.
 * - `updateGps()` does the same with a GPS pose.
 *
 * A reading further from its prediction than `PoseFilterConfig::gate`
 * standard deviations is rejected and changes nothing.
 *
 * The field frame is the snapshot map's: 0 to 144 in on both axes, heading 0
 * along +Y, clockwise positive. Odometry has to be reset into that frame for
 * the distance readings to mean anything.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace control {

/**
 * @brief Noise levels for `PoseFilter`.
 *
 * Odometry error is treated as a random walk: its variance grows in
 * proportion to the distance travelled and the angle turned, however many
 * predict() calls that takes. So these are variances per inch or per degree.
 * The defaults grow the position error by about 3 in per 100 in travelled
 * and the heading error by about 1 deg per 90 deg turned.
 */
struct PoseFilterConfig {
  /// @brief Variance along the heading per inch travelled, in^2/in.
  double odom_along_var_per_in = 0.09;
  /// @brief Variance across the heading per inch travelled, in^2/in: slip.
  double odom_side_var_per_in = 0.09;
  /// @brief Heading variance per degree turned, deg^2/deg.
  double odom_heading_var_per_deg = 0.011;
  /// @brief Heading variance per inch travelled, deg^2/in: IMU drift while
  ///        driving straight.
  double odom_heading_var_per_in = 0.0005;
  /// @brief Distance sensor error: this fraction of the reading ...
  double distance_fraction = 0.03;
  /// @brief ... but never less than this, inches.
  double distance_min_in = 0.5;
  /// @brief Reject a reading more than this many standard deviations from
  ///        what the filter expects.
  double gate = 3.0;
  /// @brief Field walls a distance ray may hit, when the sensor's own
  ///        `field_mask` is 0.
  std::uint32_t field_mask = snapshot::MAP_PERIMETER;
};

/**
 * @brief Extended Kalman filter over the robot pose. Not thread-safe; call it
 *        from one task.
 */
class PoseFilter {
 public:
  explicit PoseFilter(PoseFilterConfig config = {});

  /**
   * @brief Start from @p pose with the given uncertainty.
   * @param position_sigma_in Standard deviation of x and y, inches.
   * @param heading_sigma_deg Standard deviation of heading, degrees.
   */
  void reset(const Pose2D& pose, double position_sigma_in = 1.0,
             double heading_sigma_deg = 1.0);

  /**
   * @brief Move by one odometry step.
   *
   * The step is taken in the robot frame of @p previous_odom, so the filter
   * applies it from its own pose rather than jumping to the odometry's.
   *
   * @param previous_odom Odometry pose at the last call.
   * @param current_odom Odometry pose now.
   */
  void predict(const Pose2D& previous_odom, const Pose2D& current_odom);

  /**
   * @brief Blend in one distance sensor reading.
   * @param geometry Where the sensor is on the robot and what it may hit.
   * @param measured_in The reading, inches. Zero, negative or not finite is
   *        ignored.
   * @return True when the reading was used; false when it was rejected, the
   *         ray hits no wall in range, or the reading was invalid.
   */
  bool updateDistance(const snapshot::SensorGeometry& geometry, double measured_in);

  /**
   * @brief Blend in a GPS pose.
   * @param measured x, y in inches and heading in radians, field frame.
   * @param position_sigma_in The sensor's position error, e.g. from
   *        `device::Gps`'s reported error.
   * @param heading_sigma_deg Heading error; negative ignores the heading.
   * @return True when the reading was used.
   */
  bool updateGps(const Pose2D& measured, double position_sigma_in,
                 double heading_sigma_deg = -1.0);

  /// @brief The current estimate.
  const Pose2D& pose() const { return m_pose; }
  /// @brief Covariance of (x in, y in, heading rad).
  const Mat3& covariance() const { return m_cov; }
  /// @brief Standard deviation of position, inches: the larger axis.
  double positionSigmaIn() const;

  /// @brief Readings used and rejected since the last reset().
  int accepted() const { return m_accepted; }
  int rejected() const { return m_rejected; }

  const PoseFilterConfig& config() const { return m_config; }
  void setConfig(const PoseFilterConfig& config) { m_config = config; }

 private:
  PoseFilterConfig m_config;
  Pose2D m_pose{};
  Mat3 m_cov = Mat3::Identity();
  int m_accepted = 0;
  int m_rejected = 0;
};

}  // namespace control
}  // namespace mclib
