// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/chassis/holonomic_chassis.hpp"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/control/drive_curve.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/device/controller.hpp"
#include "mclib/math.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"

#include <memory>

namespace mclib {

/// @brief Tuning for HolonomicController. Gains are `double` for the reason PIDGains gives.
struct HolonomicControllerConfig {
  /// @brief Volts per inch of distance still to travel.
  PIDGains translation_pid{0.4, 0.0, 3.0};
  /// @brief Volts per degree of heading error.
  PIDGains heading_pid{0.3, 0.0, 1.5};
  /// @brief Exit conditions on the remaining distance, inches.
  PIDExit translation_exit{0.5, 1.5, 50.0 * units::millisecond,
                           250.0 * units::millisecond, 5.0};
  /// @brief Exit conditions on the heading error, degrees.
  PIDExit heading_exit{1.0, 3.0, 50.0 * units::millisecond,
                       250.0 * units::millisecond, 4.5};
  /// @brief Cap on every wheel, and on each loop's output before mixing.
  QVoltage max_voltage = 12.0 * units::volt;
  /// @brief makeDriveCommand() drives field-centric (sticks are field axes)
  ///        when true, robot-centric when false.
  bool field_centric_teleop = true;
};

/**
 * @brief The holonomic drive subsystem: a scheduler-driven move-to-pose, and
 *        a teleop command.
 *
 * A holonomic drive can translate and turn at the same time, so there is one
 * kind of goal: go to a field pose. Each `periodic()` reads the odometry
 * pose from `control::robotState()`, runs a distance PID on how far the
 * target still is, points that output along the field-frame bearing to the
 * target, rotates it into the robot frame, adds a heading PID's output as
 * the turn, mixes the three into four wheel voltages and writes them.
 * The goal settles when both loops report `targetArrived()`, or gives up at
 * the timeout. Every exit path leaves the motors in a defined state, as
 * `ChassisController::finishGoal()` does.
 *
 * Everything positional is the odometry's - the loop needs
 * `control::startOdometry()` running, or the pose never moves and the goal
 * runs to its timeout. Heading is the pose's `theta` too, so the position and
 * the heading it steers by come from the same locked read.
 *
 * @code
 * mclib::HolonomicChassis drive({1}, {-2}, {3}, {-4}, Gearset::Green, Kind::Mecanum, imu);
 * mclib::HolonomicController controller(drive);
 * controller.setDefaultCommand(controller.makeDriveCommand(master));
 * controller.registerSelf();
 *
 * // In autonomous:
 * controller.makeMoveToPoseCommand({24.0, 24.0, 90.0 * mclib::kPi / 180.0}, 3_s)->schedule();
 * @endcode
 */
class HolonomicController : public Subsystem {
 public:
  explicit HolonomicController(HolonomicChassis& chassis,
                               HolonomicControllerConfig config = {});

  void setConfig(const HolonomicControllerConfig& config);
  HolonomicControllerConfig getConfig() const;

  /**
   * @brief Start a scheduler-driven move to a field pose.
   *
   * @param target      Field pose: x, y in inches, theta in radians, compass frame.
   * @param timeout     Give up after this long. Zero means no timeout.
   * @param stop_at_end Hold the drive when the goal finishes. False commands
   *                    zero volts instead, so a chained move inherits the
   *                    robot's momentum.
   */
  void moveToPose(Pose2D target,
                  QTime timeout = 0.0 * units::second,
                  bool stop_at_end = true);

  /// @brief moveToPose() to (x, y), holding the heading the robot has now.
  void moveToPoint(QLength x,
                   QLength y,
                   QTime timeout = 0.0 * units::second,
                   bool stop_at_end = true);

  /// @brief Abort the current goal and hold the drive. No-op when idle.
  void cancel();
  void onDisabled() override { cancel(); m_chassis.stop(); }

  bool isSettled() const;
  bool isActive() const;

  void periodic() override;

  std::unique_ptr<Command> makeMoveToPoseCommand(
      Pose2D target,
      QTime timeout = 0.0 * units::second,
      bool stop_at_end = true);
  std::unique_ptr<Command> makeMoveToPointCommand(
      QLength x,
      QLength y,
      QTime timeout = 0.0 * units::second,
      bool stop_at_end = true);

  /**
   * @brief The teleop drive command; meant to be the default command.
   *
   * Reads three stick axes, shapes each through its own `DriveCurve` (so the
   * slew limit is per axis), and drives the chassis. With
   * `config.field_centric_teleop` the forward/strafe sticks are field +Y and
   * field +X: pushing the left stick up always moves the robot toward the
   * far end of the field, whichever way it is facing. Otherwise they are
   * robot forward and robot right. The turn stick is always clockwise
   * positive.
   *
   * The curves are reset when the command starts, and the drive is commanded
   * to zero when it ends.
   */
  std::unique_ptr<Command> makeDriveCommand(
      device::Controller& controller,
      control::DriveCurveConfig curve = {},
      device::AnalogAxis forward_axis = device::AnalogAxis::LeftY,
      device::AnalogAxis strafe_axis = device::AnalogAxis::LeftX,
      device::AnalogAxis turn_axis = device::AnalogAxis::RightX);

 private:
  static void applyExit(PID& pid, const PIDExit& exit);
  void startGoal(double x_in, double y_in, double theta_rad,
                 QTime timeout, bool stop_at_end);
  void runMoveToPose();
  /**
   * @brief End the goal, leaving the drive in a defined state.
   * @param force_stop Brake and hold regardless of `stop_at_end`: a timeout or
   *                   a cancel does not hand momentum to the next goal.
   */
  void finishGoal(bool force_stop);
  bool timedOut() const;

  HolonomicChassis& m_chassis;
  HolonomicControllerConfig m_config;
  PID m_translation_pid;
  PID m_heading_pid;
  bool m_active = false;
  bool m_settled = true;
  double m_target_x_in = 0.0;
  double m_target_y_in = 0.0;
  double m_target_theta_rad = 0.0;
  double m_start_time_ms = 0.0;
  double m_timeout_ms = 0.0;
  bool m_stop_at_end = true;
};

}  // namespace mclib
