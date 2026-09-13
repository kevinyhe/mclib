// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/odometry.hpp"

#include "mclib/control/robot_state.hpp"
#include "mclib/sync.hpp"

#include <cmath>

namespace mclib {
namespace control {

namespace {

/// @brief Below this heading change the arc collapses to a straight line.
constexpr double kStraightEpsilonRad = 1e-9;

/// @brief Encoder degrees -> inches, given inches per revolution.
double degreesToInches(double degrees, double inches_per_revolution) {
  return degrees * inches_per_revolution / 360.0;
}

/**
 * @brief The arc-shortening factor `2 sin(dtheta/2) / dtheta`.
 *
 * A straight line has this equal to 1; a quarter turn shrinks the net
 * displacement to about 0.9 of the arc length.
 */
double chordFactor(double dtheta) {
  if (std::fabs(dtheta) < kStraightEpsilonRad) {
    return 1.0;
  }
  return 2.0 * std::sin(dtheta * 0.5) / dtheta;
}

}  // namespace

Odometry::Odometry(OdometryConfig config) : m_config(config) {}

void Odometry::setConfig(const OdometryConfig& config) {
  // Samples from another layout/scale are not a usable delta baseline. In
  // particular, disabled tracker fields may legitimately contain NaN.
  if (config.drive_inches_per_revolution != m_config.drive_inches_per_revolution ||
      config.use_vertical_tracker != m_config.use_vertical_tracker ||
      config.vertical_circumference != m_config.vertical_circumference ||
      config.vertical_offset_right != m_config.vertical_offset_right ||
      config.use_horizontal_tracker != m_config.use_horizontal_tracker ||
      config.horizontal_circumference != m_config.horizontal_circumference ||
      config.horizontal_offset_forward != m_config.horizontal_offset_forward) {
    m_has_baseline = false;
  }
  m_config = config;
}

OdometryConfig Odometry::getConfig() const {
  return m_config;
}

void Odometry::reset(const Pose2D& pose) {
  // A caller that builds the pose out of a faulted IMU reading hands us a NaN
  // heading. Take the position and keep the heading we already had rather
  // than poisoning the pose - same rule as update().
  const double theta =
      std::isfinite(pose.theta) ? wrapAngle(pose.theta) : m_pose.theta;
  m_pose = Pose2D{pose.x, pose.y, theta};
  m_has_baseline = false;
}

bool Odometry::correctPosition(double x_in, double y_in) {
  if (!std::isfinite(x_in) || !std::isfinite(y_in)) return false;
  m_pose.x = x_in;
  m_pose.y = y_in;
  return true;
}

Pose2D Odometry::update(const OdometrySample& sample) {
  // Bug 3: one NAN out of the IMU used to poison the pose forever. Reject the
  // whole sample and leave the baseline where it is, so the travel covered
  // during the dropout is folded in on the next good reading instead of being
  // dropped or turned into NaN.
  bool finite = std::isfinite(sample.heading_rad);
  if (m_config.use_vertical_tracker) {
    finite = finite && std::isfinite(sample.vertical_deg);
  } else {
    finite = finite && std::isfinite(sample.left_deg) &&
             std::isfinite(sample.right_deg);
  }
  if (m_config.use_horizontal_tracker) {
    finite = finite && std::isfinite(sample.horizontal_deg);
  }
  if (!finite) {
    ++m_fault_count;
    return m_pose;
  }

  // Bug 2: the old code assumed a zero starting heading instead of reading
  // one. The first sample here only records where the sensors are.
  if (!m_has_baseline) {
    m_previous = sample;
    m_has_baseline = true;
    return m_pose;
  }

  const double dtheta = wrapAngle(sample.heading_rad - m_previous.heading_rad);

  // Forward travel of the tracking centre, in inches.
  double forward_in = 0.0;
  double forward_offset_right_in = 0.0;
  if (m_config.use_vertical_tracker) {
    forward_in = degreesToInches(sample.vertical_deg - m_previous.vertical_deg,
                                 m_config.vertical_circumference.in());
    forward_offset_right_in = m_config.vertical_offset_right.in();
  } else {
    // Bug 1: the old code added half the track width for both sides. Averaging
    // the two encoders is the same arc formula with the signs done right - the
    // +/- pair cancels, which is exactly why an in-place spin must produce no
    // translation.
    const double inches_per_rev = m_config.drive_inches_per_revolution.in();
    const double left_in =
        degreesToInches(sample.left_deg - m_previous.left_deg, inches_per_rev);
    const double right_in = degreesToInches(
        sample.right_deg - m_previous.right_deg, inches_per_rev);
    forward_in = (left_in + right_in) * 0.5;
    forward_offset_right_in = 0.0;
  }

  // Sideways travel of the tracking centre, in inches. Without a horizontal
  // tracker the robot is assumed not to strafe.
  double sideways_in = 0.0;
  double sideways_offset_forward_in = 0.0;
  if (m_config.use_horizontal_tracker) {
    sideways_in =
        degreesToInches(sample.horizontal_deg - m_previous.horizontal_deg,
                        m_config.horizontal_circumference.in());
    sideways_offset_forward_in = m_config.horizontal_offset_forward.in();
  }

  // Undo the rotation the wheels saw because of where they are mounted.
  const double dy_body = forward_in + dtheta * forward_offset_right_in;
  const double dx_body = sideways_in - dtheta * sideways_offset_forward_in;

  // Integrate the arc, then rotate into the field. Compass frame: x uses sin,
  // y uses cos.
  const double k = chordFactor(dtheta);
  const double psi = m_pose.theta + dtheta * 0.5;
  const double sin_psi = std::sin(psi);
  const double cos_psi = std::cos(psi);

  m_pose.x += k * (dx_body * cos_psi + dy_body * sin_psi);
  m_pose.y += k * (-dx_body * sin_psi + dy_body * cos_psi);
  m_pose.theta = wrapAngle(m_pose.theta + dtheta);

  m_previous = sample;
  return m_pose;
}

Pose2D Odometry::getPose() const {
  return m_pose;
}

bool Odometry::hasBaseline() const {
  return m_has_baseline;
}

std::uint32_t Odometry::faultCount() const {
  return m_fault_count;
}

// ---------------------------------------------------------------------------
// The library's single instance.
//
// Lock order is always odometry then RobotState. Nothing takes them the other
// way round, so there is no cycle to deadlock on.
// ---------------------------------------------------------------------------

namespace {
/**
 * @brief Function-local statics, not namespace-scope objects.
 *
 * A Chassis constructed at namespace scope calls tare(), which calls
 * resetOdometry(). If the mutex lived at namespace scope, that could run
 * before this translation unit's dynamic initialisation and lock an
 * unconstructed mutex. Function-local statics are constructed on first use, so
 * the order stops mattering.
 */
sync::Mutex& odometryMutex() {
  static sync::Mutex mutex;
  return mutex;
}

Odometry& odometryInstance() {
  static Odometry odometry;
  return odometry;
}
}  // namespace

Pose2D odometryTick(const OdometrySample& sample) {
  sync::LockGuard lock(odometryMutex());
  const Pose2D pose = odometryInstance().update(sample);
  robotState().setPose(pose);
  return pose;
}

void resetOdometry(const Pose2D& pose) {
  sync::LockGuard lock(odometryMutex());
  odometryInstance().reset(pose);
  // Publish what the odometry actually took, not what was asked for: reset()
  // rejects a non-finite heading and keeps the old one.
  robotState().setPose(odometryInstance().getPose());
}

bool correctOdometryPosition(double x_in, double y_in) {
  sync::LockGuard lock(odometryMutex());
  if (!odometryInstance().correctPosition(x_in, y_in)) return false;
  robotState().setPose(odometryInstance().getPose());
  return true;
}

void setOdometryConfig(const OdometryConfig& config) {
  sync::LockGuard lock(odometryMutex());
  odometryInstance().setConfig(config);
}

OdometryConfig getOdometryConfig() {
  sync::LockGuard lock(odometryMutex());
  return odometryInstance().getConfig();
}

std::uint32_t odometryFaultCount() {
  sync::LockGuard lock(odometryMutex());
  return odometryInstance().faultCount();
}

}  // namespace control
}  // namespace mclib
