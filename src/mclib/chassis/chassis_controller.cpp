// mclib
#include "mclib/chassis/chassis_controller.hpp"

#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/time.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace mclib {

namespace {
struct AsyncControlState {
  std::atomic_bool done{false};
};

/**
 * @brief How long end()/~AsyncControlCommand() wait for a cancelled routine.
 *
 * Every loop in motion.cpp checks control::cancelRequested() next to its
 * timeout and delays 10 ms, so the real figure is one loop period. This is the
 * ceiling before we give up waiting, not the expected cost.
 */
constexpr std::uint32_t kCancelJoinTimeoutMs = 500;

/**
 * @brief Runs a blocking motion routine on its own task.
 *
 * The old version stopped that task with pros::Task::remove(), a hard kill
 * with no cooperation point: the victim could be halfway through updating the
 * shared scalars, and the caller then patched up two of the six by hand. It
 * also raced -- done.load() could turn true between the check and the
 * remove().
 *
 * Cancellation is cooperative now. We set a flag, the routine's loop notices
 * it at the next 10 ms boundary, unwinds normally, and we wait for it.
 * remove() survives only as a last resort for a routine that has wedged.
 */
class AsyncControlCommand : public Command {
public:
  AsyncControlCommand(std::function<void()> action,
                      Subsystem* requirement,
                      control::CancelToken token)
      : m_action(std::move(action)),
        m_token(token),
        m_requirements{requirement} {}

  void initialize() override {
    stopRunningTask();

    m_state->done.store(false);
    control::clearCancel(m_token);
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
    if (interrupted) {
      stopRunningTask();
    }
    m_task.reset();
  }

  std::vector<Subsystem*> getRequirements() override {
    return m_requirements;
  }

  /**
   * @brief Cancels a still-running routine and de-energises the drive.
   *
   * The old destructor called remove() and nothing else, so a command
   * destroyed mid-motion left the motors driving.
   * Routine::MotionStep::rebuild() destroys and recreates commands on every
   * .withXxx() call, so this is a path a user reaches by writing an ordinary
   * auton.
   */
  ~AsyncControlCommand() override {
    stopRunningTask();
  }

private:
  /**
   * @brief Ask the routine to stop, wait for it, then put the drive to rest.
   *
   * Does nothing when there is no task, or when the routine already finished
   * on its own -- in that case it has stopped the chassis itself.
   */
  void stopRunningTask() {
    if (m_task == nullptr || m_state->done.load()) {
      return;
    }

    control::requestCancel(m_token);

    const std::uint32_t deadline = pros::millis() + kCancelJoinTimeoutMs;
    while (!m_state->done.load() && pros::millis() < deadline) {
      pros::delay(2);
    }

    if (m_state->done.load()) {
      // The routine unwound on its own. Nothing holds a lock, so the full
      // repair is safe.
      stopChassis(device::BrakeMode::Hold);
      control::robotState().clearMotionOutputs();
    } else {
      // The routine ignored the flag for half a second. Every loop in
      // motion.cpp checks it at a 10 ms boundary, so this should be
      // unreachable; the hard kill is what is left when it is not.
      //
      // Nothing after remove() may take an mclib lock. FreeRTOS does not
      // release a mutex held by a deleted task, so a repair that touched
      // RobotState or the odometry could block the scheduler for good if the
      // victim happened to die inside one. stopChassis() only writes motors.
      m_task->remove();
      stopChassis(device::BrakeMode::Hold);
    }

    control::clearCancel(m_token);
    m_task.reset();
  }

  std::function<void()> m_action;
  /// @brief Which family of routines this command's action belongs to.
  control::CancelToken m_token;
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
  m_start_time_ms = mclib::time::millis();
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
  m_start_time_ms = mclib::time::millis();
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
  // No odometry here any more. There is one odometry, it runs on its own task
  // (control/odometry_task.hpp), and it no longer depends on the scheduler
  // getting round to this subsystem.
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
  // correctHeading() runs alongside the motions rather than instead of them,
  // so it gets its own cancel token. Sharing the motion one would tear the
  // heading hold down the first time any other routine was interrupted.
  return makeAsyncControlCommand([]() { correctHeading(); },
                                 control::CancelToken::HeadingCorrection);
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
         (static_cast<double>(mclib::time::millis()) - m_start_time_ms) >= m_timeout_ms;
}

std::unique_ptr<Command> ChassisController::makeAsyncControlCommand(
    std::function<void()> action,
    control::CancelToken token) {
  return std::make_unique<AsyncControlCommand>(std::move(action), this, token);
}

}  // namespace mclib
