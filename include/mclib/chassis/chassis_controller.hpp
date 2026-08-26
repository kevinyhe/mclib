// mclib
#pragma once

#include "mclib/chassis/chassis.hpp"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/runCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/device/controller.hpp"
#include "mclib/pid.hpp"

#include <functional>
#include <memory>

namespace mclib {

struct PIDGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

struct PIDExit {
  double small_error = 1.0;
  double big_error = 3.0;
  double small_duration_ms = 50.0;
  double big_duration_ms = 250.0;
  double derivative = 5.0;
};

struct ChassisControllerConfig {
  PIDGains distance_pid{0.4, 0.0, 3.0};
  PIDGains turn_pid{0.3, 0.0, 1.5};
  PIDGains heading_pid{0.3, 0.0, 1.5};
  PIDExit distance_exit{};
  PIDExit turn_exit{1.0, 3.0, 50.0, 250.0, 4.5};
  double max_voltage = 12.0;
  double min_voltage = 0.0;
  bool heading_correction = true;
};

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

  void driveDistance(double distance_in,
                     double timeout_ms = 0.0,
                     bool stop_at_end = true,
                     double max_voltage = -1.0);
  void turnToHeading(double heading_deg,
                     double timeout_ms = 0.0,
                     bool stop_at_end = true,
                     double max_voltage = -1.0);
  void cancel();

  bool isSettled() const;
  bool isActive() const;
  Mode getMode() const;

  void periodic() override;

  std::unique_ptr<Command> makeDriveDistanceCommand(
      double distance_in,
      double timeout_ms = 0.0,
      bool stop_at_end = true,
      double max_voltage = -1.0);
  std::unique_ptr<Command> makeTurnToHeadingCommand(
      double heading_deg,
      double timeout_ms = 0.0,
      bool stop_at_end = true,
      double max_voltage = -1.0);
  std::unique_ptr<Command> makeTurnToAngleCommand(
      double turn_angle,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeDriveToCommand(
      double distance_in,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeCurveCircleCommand(
      double result_angle_deg,
      double center_radius,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      double min_speed = -1.0,
      bool reverse = false);
  std::unique_ptr<Command> makeCurveCircleReverseCommand(
      double result_angle_deg,
      double center_radius,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeSwingCommand(
      double swing_angle,
      double drive_direction,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeCorrectHeadingCommand();
  std::unique_ptr<Command> makeWallResetCommand(
      double reset_x,
      double reset_y,
      double reset_heading,
      double drive_power,
      double time_limit_msec,
      double current_threshold = 2500.0,
      double velocity_threshold = 5.0);
  std::unique_ptr<Command> makeTurnToPointCommand(
      double x,
      double y,
      int direction = 1,
      double time_limit_msec = 1000.0,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeMoveToPointCommand(
      double x,
      double y,
      int dir,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      bool overturn = true,
      double min_speed = -1.0);
  std::unique_ptr<Command> makeBoomerangCommand(
      double x,
      double y,
      int dir,
      double a,
      double dlead,
      double time_limit_msec,
      bool exit = true,
      double max_output = 12.0,
      bool overturn = true,
      double min_speed = -1.0);
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
  std::unique_ptr<Command> makeAsyncControlCommand(std::function<void()> action);

  Chassis& m_chassis;
  ChassisControllerConfig m_config;
  PID m_distance_pid;
  PID m_turn_pid;
  PID m_heading_pid;
  Mode m_mode = Mode::Idle;
  double m_goal = 0.0;
  double m_start_distance_in = 0.0;
  double m_start_time_ms = 0.0;
  double m_timeout_ms = 0.0;
  double m_goal_max_voltage = 12.0;
  bool m_stop_at_end = true;
  bool m_settled = true;
};

}  // namespace mclib
