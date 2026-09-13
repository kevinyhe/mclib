// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/chassis/chassis.hpp"
#include "mclib/chassis/chassis_math.hpp"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/runCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/control/motion.hpp"
#include "mclib/control/drive_curve.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/device/controller.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>

namespace mclib {

/**
 * @brief The tuning record for a ChassisController.
 *
 * The same struct every drive loop in mclib reads - see
 * `control/motion_config.hpp`. `ChassisController::setConfig()` installs it
 * as the active `control::motionConfig()`, so the scheduler-driven loops in
 * this class and the blocking routines in `control/motion.hpp` cannot be
 * tuned differently by accident.
 */
using ChassisControllerConfig = control::MotionConfig;

/**
 * @brief The drive subsystem: two built-in scheduler loops, plus commands that
 *        wrap the blocking routines in `control/motion.hpp`.
 *
 * `driveDistance()` and `turnToHeading()` run inside `periodic()`, one tick per
 * scheduler pass. Everything named `makeXxxCommand()` instead launches the
 * corresponding free function from `motion.hpp` on its own task and cancels it
 * cooperatively; those are the routines an autonomous is normally built from.
 *
 * Constructing a ChassisController binds its Chassis as the drivetrain those
 * free functions run on (`control::bindDrive()`) and installs its config as
 * the active `control::motionConfig()`. One ChassisController per program.
 */
class ChassisController : public Subsystem {
public:
  enum class Mode {
    Idle,
    DriveDistance,
    TurnToHeading,
  };

  explicit ChassisController(Chassis& chassis,
                             ChassisControllerConfig config = {});

  void setConfig(const ChassisControllerConfig& config);
  ChassisControllerConfig getConfig() const;

  /**
   * @brief Start a scheduler-driven straight-line move.
   *
   * @param distance    How far to travel; negative drives backward.
   * @param timeout     Give up after this long. Zero means no timeout.
   * @param stop_at_end Hold the drive when the goal finishes.
   * @param max_voltage Voltage cap; anything not positive uses the config's.
   */
  void driveDistance(QLength distance,
                     QTime timeout = 0.0 * units::second,
                     bool stop_at_end = true,
                     QVoltage max_voltage = -1.0 * units::volt);

  /**
   * @brief Start a scheduler-driven turn to an absolute heading.
   *
   * @param heading     Absolute compass heading, not a delta.
   * @param timeout     Give up after this long. Zero means no timeout.
   * @param stop_at_end Hold the drive when the goal finishes.
   * @param max_voltage Voltage cap; anything not positive uses the config's.
   */
  void turnToHeading(QAngle heading,
                     QTime timeout = 0.0 * units::second,
                     bool stop_at_end = true,
                     QVoltage max_voltage = -1.0 * units::volt);
  void cancel();
  void onDisabled() override { cancel(); m_chassis.stop(); }

  bool isSettled() const;
  bool isActive() const;
  Mode getMode() const;

  void periodic() override;

  std::unique_ptr<Command> makeDriveDistanceCommand(
      QLength distance,
      QTime timeout = 0.0 * units::second,
      bool stop_at_end = true,
      QVoltage max_voltage = -1.0 * units::volt);
  std::unique_ptr<Command> makeTurnToHeadingCommand(
      QAngle heading,
      QTime timeout = 0.0 * units::second,
      bool stop_at_end = true,
      QVoltage max_voltage = -1.0 * units::volt);

  // The rest wrap the blocking routines in control/motion.hpp one for one;
  // the parameter documentation lives there.
  std::unique_ptr<Command> makeTurnToAngleCommand(
      QAngle turn_angle,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeDriveToCommand(
      QLength distance,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeCurveCircleCommand(
      QAngle result_angle,
      QLength center_radius,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      QVoltage min_speed = -1.0 * units::volt,
      bool reverse = false);
  std::unique_ptr<Command> makeCurveCircleReverseCommand(
      QAngle result_angle,
      QLength center_radius,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeSwingCommand(
      QAngle swing_angle,
      double drive_direction,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeCorrectHeadingCommand();
  std::unique_ptr<Command> makeWallResetCommand(
      QLength reset_x,
      QLength reset_y,
      QAngle reset_heading,
      QVoltage drive_power,
      QTime time_limit,
      QCurrent current_threshold = 2500.0 * units::milliampere,
      QAngularVelocity velocity_threshold = 5.0 * units::rpm);
  std::unique_ptr<Command> makeTurnToPointCommand(
      QLength x,
      QLength y,
      int direction = 1,
      QTime time_limit = 1000.0 * units::millisecond,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeMoveToPointCommand(
      QLength x,
      QLength y,
      int dir,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      bool overturn = true,
      QVoltage min_speed = -1.0 * units::volt);
  std::unique_ptr<Command> makeBoomerangCommand(
      QLength x,
      QLength y,
      int dir,
      QAngle final_heading,
      double dlead,
      QTime time_limit,
      bool exit = true,
      QVoltage max_output = 12.0 * units::volt,
      bool overturn = true,
      QVoltage min_speed = -1.0 * units::volt);
  // Driver control. Each returns a command meant to be the subsystem's
  // default: it runs whenever no autonomous command owns the drive, reads
  // the sticks every scheduler pass, shapes them through the curves in
  // `control/drive_curve.hpp`, and writes the motors. The curves' slew state
  // is reset each time the command starts, so a hand-back from autonomous
  // never begins with a stale ramp.

  /**
   * @brief Arcade: one stick axis drives, one turns.
   *
   * @param forward_curve Deadzone, expo gain, stiction floor and slew for the
   *                      drive axis.
   * @param turn_curve    The same for the turn axis. Turning usually wants
   *                      a larger gain than driving.
   */
  std::unique_ptr<Command> makeArcadeDriveCommand(
      device::Controller& controller,
      control::DriveCurveConfig forward_curve = {},
      control::DriveCurveConfig turn_curve = {},
      device::AnalogAxis forward_axis = device::AnalogAxis::LeftY,
      device::AnalogAxis turn_axis = device::AnalogAxis::RightX);

  /**
   * @brief Curvature ("cheesy") drive: the turn stick sets the curvature of
   *        the path rather than a wheel speed difference, so the same stick
   *        deflection bends the path the same amount at any speed.
   *
   * With the drive stick centred it falls back to turning in place.
   */
  std::unique_ptr<Command> makeCurvatureDriveCommand(
      device::Controller& controller,
      control::DriveCurveConfig forward_curve = {},
      control::DriveCurveConfig turn_curve = {},
      device::AnalogAxis forward_axis = device::AnalogAxis::LeftY,
      device::AnalogAxis turn_axis = device::AnalogAxis::RightX);

  /// @brief Tank: each stick drives its own side.
  std::unique_ptr<Command> makeTankDriveCommand(
      device::Controller& controller,
      control::DriveCurveConfig curve = {},
      device::AnalogAxis left_axis = device::AnalogAxis::LeftY,
      device::AnalogAxis right_axis = device::AnalogAxis::RightY);

private:
  static double clampVoltage(double volts, double max_voltage);
  static void applyExit(PID& pid, const PIDExit& exit);

  void runDriveDistance();
  void runTurnToHeading();
  /**
   * @brief End the current goal, leaving the drive in a defined state.
   *
   * @param force_stop Brake and hold regardless of `stop_at_end`. True for a
   *                   timeout or a cancel: a goal that did not reach its
   *                   target does not hand momentum to the next one.
   */
  void finishGoal(bool force_stop);
  bool timedOut() const;
  std::unique_ptr<Command> makeAsyncControlCommand(
      std::function<void()> action,
      control::CancelToken token = control::CancelToken::Motion);
  /**
   * @brief The shared shape of the three teleop commands.
   *
   * @param mix Turns the two shaped stick values into a left/right pair. For
   *            tank it is the identity.
   */
  std::unique_ptr<Command> makeTeleopCommand(
      device::Controller& controller,
      control::DriveCurveConfig first_curve,
      control::DriveCurveConfig second_curve,
      device::AnalogAxis first_axis,
      device::AnalogAxis second_axis,
      std::function<chassis_math::DrivePair(double, double)> mix);

  Chassis& m_chassis;
  ChassisControllerConfig m_config;
  PID m_distance_pid;
  PID m_turn_pid;
  PID m_heading_pid;
  Mode m_mode = Mode::Idle;
  /// @brief Inches in DriveDistance, degrees in TurnToHeading. Hence `double`.
  double m_goal = 0.0;
  double m_start_distance_in = 0.0;
  double m_start_time_ms = 0.0;
  double m_timeout_ms = 0.0;
  double m_goal_max_voltage = 12.0;
  bool m_stop_at_end = true;
  bool m_settled = true;
};

}  // namespace mclib
