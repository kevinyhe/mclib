// mclib
#pragma once

#include "mclib/chassis/chassis.hpp"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/runCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/control/motion.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/device/controller.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>

namespace mclib {

/**
 * @brief PID coefficients for one loop.
 *
 * Deliberately `double`. A gain's dimension is output over input, and the same
 * struct serves a distance loop (volts per inch) and a turn loop (volts per
 * degree). There is no one type that is right for both.
 */
struct PIDGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

/**
 * @brief When a PID loop is allowed to declare itself settled.
 *
 * The two error tolerances and the derivative tolerance are in the units of
 * whichever loop this configures - inches for the distance loop, degrees for
 * the turn loop - so they stay `double` for the same reason PIDGains does. The
 * durations are times and say so.
 */
struct PIDExit {
  double small_error = 1.0;  ///< Inches or degrees, per the loop.
  double big_error = 3.0;    ///< Inches or degrees, per the loop.
  QTime small_duration = 50.0 * units::millisecond;
  QTime big_duration = 250.0 * units::millisecond;
  double derivative = 5.0;  ///< Inches or degrees per tick, per the loop.
};

struct ChassisControllerConfig {
  PIDGains distance_pid{0.4, 0.0, 3.0};
  PIDGains turn_pid{0.3, 0.0, 1.5};
  PIDGains heading_pid{0.3, 0.0, 1.5};
  PIDExit distance_exit{};
  PIDExit turn_exit{1.0, 3.0, 50.0 * units::millisecond, 250.0 * units::millisecond, 4.5};
  /// @brief Voltage cap for the built-in loops when a goal does not override it.
  QVoltage max_voltage = 12.0 * units::volt;
  /// @brief Voltage floor for the built-in distance loop. Zero disables it.
  QVoltage min_voltage = 0.0 * units::volt;
  bool heading_correction = true;
};

/**
 * @brief The drive subsystem: two built-in scheduler loops, plus commands that
 *        wrap the blocking routines in `control/motion.hpp`.
 *
 * `driveDistance()` and `turnToHeading()` run inside `periodic()`, one tick per
 * scheduler pass. Everything named `makeXxxCommand()` instead launches the
 * corresponding free function from `motion.hpp` on its own task and cancels it
 * cooperatively; those are the routines an autonomous is normally built from.
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
  std::unique_ptr<Command> makeArcadeDriveCommand(
      device::Controller& controller,
      device::AnalogAxis forward_axis = device::AnalogAxis::LeftY,
      device::AnalogAxis turn_axis = device::AnalogAxis::RightX,
      double scale = 127.0);

private:
  static double clampVoltage(double volts, double max_voltage);
  static void applyExit(PID& pid, const PIDExit& exit);

  void runDriveDistance();
  void runTurnToHeading();
  void finishGoal();
  bool timedOut() const;
  std::unique_ptr<Command> makeAsyncControlCommand(
      std::function<void()> action,
      control::CancelToken token = control::CancelToken::Motion);

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
