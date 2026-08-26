// mclib
#include "mclib/chassis/chassis_controller.hpp"

#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion.hpp"
#include "mclib/control/state.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace mclib {

namespace {
struct AsyncControlState {
  std::atomic_bool done{false};
};

class AsyncControlCommand : public Command {
public:
  AsyncControlCommand(std::function<void()> action, Subsystem* requirement)
      : m_action(std::move(action)), m_requirements{requirement} {}

  void initialize() override {
    if (m_task != nullptr) {
      m_task->remove();
      m_task.reset();
    }

    m_state->done.store(false);
    const auto state = m_state;
    const auto action = m_action;
    m_task = std::make_unique<pros::Task>(
        [state, action]() {
          action();
          state->done.store(true);
        },
        "mclib chassis");
  }

  bool isFinished() override {
    return m_state->done.load();
  }

  void end(bool interrupted) override {
    if (interrupted && m_task != nullptr && !m_state->done.load()) {
      m_task->remove();
      stopChassis(device::BrakeMode::Hold);
      is_turning = false;
    }
    m_task.reset();
  }

  std::vector<Subsystem*> getRequirements() override {
    return m_requirements;
  }

  ~AsyncControlCommand() override {
    if (m_task != nullptr && !m_state->done.load()) {
      m_task->remove();
    }
  }

private:
  std::function<void()> m_action;
  std::shared_ptr<AsyncControlState> m_state =
      std::make_shared<AsyncControlState>();
  std::unique_ptr<pros::Task> m_task;
  std::vector<Subsystem*> m_requirements;
};
}  // namespace

ChassisController::ChassisController(Chassis& chassis,
                                     ChassisControllerConfig config)
    : m_chassis(chassis),
      m_config(config),
      m_distance_pid(config.distance_pid.kp,
                     config.distance_pid.ki,
                     config.distance_pid.kd),
      m_turn_pid(config.turn_pid.kp, config.turn_pid.ki, config.turn_pid.kd),
      m_heading_pid(config.heading_pid.kp,
                    config.heading_pid.ki,
                    config.heading_pid.kd) {
  applyExit(m_distance_pid, m_config.distance_exit);
  applyExit(m_turn_pid, m_config.turn_exit);
  m_heading_pid.setArrive(false);
}

void ChassisController::setConfig(const ChassisControllerConfig& config) {
  m_config = config;
  m_distance_pid.setCoefficient(config.distance_pid.kp,
                                config.distance_pid.ki,
                                config.distance_pid.kd);
  m_turn_pid.setCoefficient(config.turn_pid.kp,
                            config.turn_pid.ki,
                            config.turn_pid.kd);
  m_heading_pid.setCoefficient(config.heading_pid.kp,
                               config.heading_pid.ki,
                               config.heading_pid.kd);
  applyExit(m_distance_pid, m_config.distance_exit);
  applyExit(m_turn_pid, m_config.turn_exit);
  m_heading_pid.setArrive(false);
}

ChassisControllerConfig ChassisController::getConfig() const {
  return m_config;
}

void ChassisController::driveDistance(double distance_in,
                                      double timeout_ms,
                                      bool stop_at_end,
                                      double max_voltage) {
  m_mode = Mode::DriveDistance;
  m_goal = distance_in;
  m_start_distance_in = m_chassis.averageDistanceIn();
  m_start_time_ms = pros::millis();
  m_timeout_ms = timeout_ms;
  m_goal_max_voltage = max_voltage > 0.0 ? max_voltage : m_config.max_voltage;
  m_stop_at_end = stop_at_end;
  m_settled = false;

  m_distance_pid.reset();
  m_heading_pid.reset();
  m_distance_pid.setTarget(distance_in);
  m_heading_pid.setTarget(m_chassis.headingDeg());
}

void ChassisController::turnToHeading(double heading_deg,
                                      double timeout_ms,
                                      bool stop_at_end,
                                      double max_voltage) {
  m_mode = Mode::TurnToHeading;
  m_goal = heading_deg;
  m_start_time_ms = pros::millis();
  m_timeout_ms = timeout_ms;
  m_goal_max_voltage = max_voltage > 0.0 ? max_voltage : m_config.max_voltage;
  m_stop_at_end = stop_at_end;
  m_settled = false;

  m_turn_pid.reset();
  m_turn_pid.setTarget(heading_deg);
}

void ChassisController::cancel() {
  finishGoal();
}

bool ChassisController::isSettled() const {
  return m_settled;
}

bool ChassisController::isActive() const {
  return m_mode != Mode::Idle;
}

ChassisController::Mode ChassisController::getMode() const {
  return m_mode;
}

void ChassisController::periodic() {
  m_chassis.updateOdometry();

  switch (m_mode) {
    case Mode::DriveDistance:
      runDriveDistance();
      break;
    case Mode::TurnToHeading:
      runTurnToHeading();
      break;
    case Mode::Idle:
    default:
      break;
  }
}

std::unique_ptr<Command> ChassisController::makeDriveDistanceCommand(
    double distance_in,
    double timeout_ms,
    bool stop_at_end,
    double max_voltage) {
  return std::make_unique<FunctionalCommand>(
      [this, distance_in, timeout_ms, stop_at_end, max_voltage]() {
        driveDistance(distance_in, timeout_ms, stop_at_end, max_voltage);
      },
      []() {},
      [this](bool interrupted) {
        if (interrupted) {
          cancel();
        }
      },
      [this]() { return isSettled(); },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> ChassisController::makeTurnToHeadingCommand(
    double heading_deg,
    double timeout_ms,
    bool stop_at_end,
    double max_voltage) {
  return std::make_unique<FunctionalCommand>(
      [this, heading_deg, timeout_ms, stop_at_end, max_voltage]() {
        turnToHeading(heading_deg, timeout_ms, stop_at_end, max_voltage);
      },
      []() {},
      [this](bool interrupted) {
        if (interrupted) {
          cancel();
        }
      },
      [this]() { return isSettled(); },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> ChassisController::makeTurnToAngleCommand(
    double turn_angle,
    double time_limit_msec,
    bool exit,
    double max_output,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    turnToAngle(turn_angle, time_limit_msec, exit, max_output, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeDriveToCommand(
    double distance_in,
    double time_limit_msec,
    bool exit,
    double max_output,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    driveTo(distance_in, time_limit_msec, exit, max_output, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(
    double result_angle_deg,
    double center_radius,
    double time_limit_msec,
    bool exit,
    double max_output,
    double min_speed,
    bool reverse) {
  return makeAsyncControlCommand([=]() {
    curveCircle(result_angle_deg,
                center_radius,
                time_limit_msec,
                exit,
                max_output,
                min_speed,
                reverse);
  });
}

std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(
    double result_angle_deg,
    double center_radius,
    double time_limit_msec,
    bool exit,
    double max_output,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    curveCircleReverse(result_angle_deg,
                       center_radius,
                       time_limit_msec,
                       exit,
                       max_output,
                       min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeSwingCommand(
    double swing_angle,
    double drive_direction,
    double time_limit_msec,
    bool exit,
    double max_output,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    swing(swing_angle,
          drive_direction,
          time_limit_msec,
          exit,
          max_output,
          min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeCorrectHeadingCommand() {
  return makeAsyncControlCommand([]() { correctHeading(); });
}

std::unique_ptr<Command> ChassisController::makeWallResetCommand(
    double reset_x,
    double reset_y,
    double reset_heading,
    double drive_power,
    double time_limit_msec,
    double current_threshold,
    double velocity_threshold) {
  return makeAsyncControlCommand([=]() {
    wallReset(reset_x,
              reset_y,
              reset_heading,
              drive_power,
              time_limit_msec,
              current_threshold,
              velocity_threshold);
  });
}

std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(
    double x,
    double y,
    int direction,
    double time_limit_msec,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    turnToPoint(x, y, direction, time_limit_msec, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeMoveToPointCommand(
    double x,
    double y,
    int dir,
    double time_limit_msec,
    bool exit,
    double max_output,
    bool overturn,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    moveToPoint(x,
                y,
                dir,
                time_limit_msec,
                exit,
                max_output,
                overturn,
                min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeBoomerangCommand(
    double x,
    double y,
    int dir,
    double a,
    double dlead,
    double time_limit_msec,
    bool exit,
    double max_output,
    bool overturn,
    double min_speed) {
  return makeAsyncControlCommand([=]() {
    boomerang(x,
              y,
              dir,
              a,
              dlead,
              time_limit_msec,
              exit,
              max_output,
              overturn,
              min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeArcadeDriveCommand(
    device::Controller& controller,
    device::AnalogAxis forward_axis,
    device::AnalogAxis turn_axis,
    double scale) {
  return run(
      [this, &controller, forward_axis, turn_axis, scale]() {
        const double forward = controller.getAnalog(forward_axis) / scale;
        const double turn = controller.getAnalog(turn_axis) / scale;
        m_chassis.arcade(forward, turn);
      });
}

double ChassisController::clampVoltage(double volts, double max_voltage) {
  return std::clamp(volts, -max_voltage, max_voltage);
}

void ChassisController::applyExit(PID& pid, const PIDExit& exit) {
  pid.setSmallBigErrorTolerance(exit.small_error, exit.big_error);
  pid.setSmallBigErrorDuration(exit.small_duration_ms, exit.big_duration_ms);
  pid.setDerivativeTolerance(exit.derivative);
}

void ChassisController::runDriveDistance() {
  const double travelled = m_chassis.averageDistanceIn() - m_start_distance_in;
  double output = m_distance_pid.update(travelled);

  if (m_config.min_voltage > 0.0 && std::fabs(output) < m_config.min_voltage &&
      !m_distance_pid.targetArrived()) {
    output = output >= 0.0 ? m_config.min_voltage : -m_config.min_voltage;
  }

  double correction = 0.0;
  if (m_config.heading_correction) {
    correction = m_heading_pid.update(m_chassis.headingDeg());
  }

  const double left = clampVoltage(output + correction, m_goal_max_voltage);
  const double right = clampVoltage(output - correction, m_goal_max_voltage);
  m_chassis.tankVoltage(left, right);

  if (m_distance_pid.targetArrived() || timedOut()) {
    finishGoal();
  }
}

void ChassisController::runTurnToHeading() {
  const double output = clampVoltage(m_turn_pid.update(m_chassis.headingDeg()),
                                     m_goal_max_voltage);
  m_chassis.tankVoltage(output, -output);

  if (m_turn_pid.targetArrived() || timedOut()) {
    finishGoal();
  }
}

void ChassisController::finishGoal() {
  if (m_stop_at_end) {
    m_chassis.stop(device::BrakeMode::Hold);
  }
  m_mode = Mode::Idle;
  m_settled = true;
}

bool ChassisController::timedOut() const {
  return m_timeout_ms > 0.0 &&
         (static_cast<double>(pros::millis()) - m_start_time_ms) >= m_timeout_ms;
}

std::unique_ptr<Command> ChassisController::makeAsyncControlCommand(
    std::function<void()> action) {
  return std::make_unique<AsyncControlCommand>(std::move(action), this);
}

}  // namespace mclib
