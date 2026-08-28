// mclib
#include "mclib/chassis/chassis_controller.hpp"

#include "mclib/chassis/chassis_math.hpp"
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

void ChassisController::driveDistance(QLength distance,
                                      QTime timeout,
                                      bool stop_at_end,
                                      QVoltage max_voltage) {
  const double distance_in = distance.in();
  const double max_voltage_volts = max_voltage.volts();

  m_mode = Mode::DriveDistance;
  m_goal = distance_in;
  m_start_distance_in = m_chassis.averageDistanceIn();
  m_start_time_ms = mclib::time::millis();
  m_timeout_ms = timeout.ms();
  m_goal_max_voltage = max_voltage_volts > 0.0 ? max_voltage_volts
                                               : m_config.max_voltage.volts();
  m_stop_at_end = stop_at_end;
  m_settled = false;

  m_distance_pid.reset();
  m_heading_pid.reset();
  m_distance_pid.setTarget(distance_in);
  m_heading_pid.setTarget(m_chassis.headingDeg());
}

void ChassisController::turnToHeading(QAngle heading,
                                      QTime timeout,
                                      bool stop_at_end,
                                      QVoltage max_voltage) {
  const double heading_deg = heading.deg();
  const double max_voltage_volts = max_voltage.volts();

  // With an IMU, headingDeg() is getRotationDeg(): accumulated rotation, not a
  // wrapped heading. After two clockwise turns it reads 720, so an absolute
  // target of 0 would be an error of -720 to be unwound the long way round.
  // Shift the target by whole turns onto the branch the robot is actually on,
  // exactly as control::normalizeTarget() does for motion.cpp's turnToAngle().
  //
  // Without one it is the odometry pose angle, which Odometry wraps to
  // [-180, 180]. Normalising against a wrapped measurement can put the target
  // outside the range that measurement can ever report -- at theta = 170 deg,
  // a target of -170 would normalise to 190, an error that bottoms out at
  // 10 deg and never clears the exit tolerance. Since turnToHeading()'s
  // default timeout is zero (no timeout), that hangs. A wrapped source is
  // already on the only branch there is, so leave the target alone.
  const double target_deg =
      m_chassis.imu() != nullptr
          ? chassis_math::normalizeHeadingTarget(heading_deg,
                                                 m_chassis.headingDeg())
          : heading_deg;

  m_mode = Mode::TurnToHeading;
  m_goal = target_deg;
  m_start_time_ms = mclib::time::millis();
  m_timeout_ms = timeout.ms();
  m_goal_max_voltage = max_voltage_volts > 0.0 ? max_voltage_volts
                                               : m_config.max_voltage.volts();
  m_stop_at_end = stop_at_end;
  m_settled = false;

  m_turn_pid.reset();
  m_turn_pid.setTarget(target_deg);
}

void ChassisController::cancel() {
  // Nothing to cancel, and nothing to grab. ParallelCommandGroup::end(true)
  // calls end(true) on children that already finished, so a settled goal's
  // command reaches here routinely -- and by then an async motion.cpp routine
  // on another task may own the motors. Braking them from under it is not this
  // controller's business.
  if (m_mode == Mode::Idle) {
    return;
  }

  // An abort, not a hand-off: a cancelled goal stops the drive whatever the
  // caller asked for at the start.
  finishGoal(true);
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
    QLength distance,
    QTime timeout,
    bool stop_at_end,
    QVoltage max_voltage) {
  return std::make_unique<FunctionalCommand>(
      [this, distance, timeout, stop_at_end, max_voltage]() {
        driveDistance(distance, timeout, stop_at_end, max_voltage);
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
    QAngle heading,
    QTime timeout,
    bool stop_at_end,
    QVoltage max_voltage) {
  return std::make_unique<FunctionalCommand>(
      [this, heading, timeout, stop_at_end, max_voltage]() {
        turnToHeading(heading, timeout, stop_at_end, max_voltage);
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
    QAngle turn_angle,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    turnToAngle(turn_angle, time_limit, exit, max_output, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeDriveToCommand(
    QLength distance,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    driveTo(distance, time_limit, exit, max_output, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(
    QAngle result_angle,
    QLength center_radius,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    QVoltage min_speed,
    bool reverse) {
  return makeAsyncControlCommand([=]() {
    curveCircle(result_angle,
                center_radius,
                time_limit,
                exit,
                max_output,
                min_speed,
                reverse);
  });
}

std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(
    QAngle result_angle,
    QLength center_radius,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    curveCircleReverse(result_angle,
                       center_radius,
                       time_limit,
                       exit,
                       max_output,
                       min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeSwingCommand(
    QAngle swing_angle,
    double drive_direction,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    swing(swing_angle,
          drive_direction,
          time_limit,
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
    QLength reset_x,
    QLength reset_y,
    QAngle reset_heading,
    QVoltage drive_power,
    QTime time_limit,
    QCurrent current_threshold,
    QAngularVelocity velocity_threshold) {
  return makeAsyncControlCommand([=]() {
    wallReset(reset_x,
              reset_y,
              reset_heading,
              drive_power,
              time_limit,
              current_threshold,
              velocity_threshold);
  });
}

std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(
    QLength x,
    QLength y,
    int direction,
    QTime time_limit,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    turnToPoint(x, y, direction, time_limit, min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeMoveToPointCommand(
    QLength x,
    QLength y,
    int dir,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    bool overturn,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    moveToPoint(x,
                y,
                dir,
                time_limit,
                exit,
                max_output,
                overturn,
                min_speed);
  });
}

std::unique_ptr<Command> ChassisController::makeBoomerangCommand(
    QLength x,
    QLength y,
    int dir,
    QAngle final_heading,
    double dlead,
    QTime time_limit,
    bool exit,
    QVoltage max_output,
    bool overturn,
    QVoltage min_speed) {
  return makeAsyncControlCommand([=]() {
    boomerang(x,
              y,
              dir,
              final_heading,
              dlead,
              time_limit,
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
  pid.setSmallBigErrorDuration(exit.small_duration.ms(), exit.big_duration.ms());
  pid.setDerivativeTolerance(exit.derivative);
}

void ChassisController::runDriveDistance() {
  const double travelled = m_chassis.averageDistanceIn() - m_start_distance_in;
  double output = m_distance_pid.update(travelled);

  const double min_voltage = m_config.min_voltage.volts();
  if (min_voltage > 0.0 && std::fabs(output) < min_voltage &&
      !m_distance_pid.targetArrived()) {
    output = output >= 0.0 ? min_voltage : -min_voltage;
  }

  double correction = 0.0;
  if (m_config.heading_correction) {
    correction = m_heading_pid.update(m_chassis.headingDeg());
  }

  // Clamp the drive term first -- PIDController does not bound its own output,
  // and at the default kp any move past 30 in starts above the rail. Clamping
  // the two sides independently instead returned the cap on both and left a
  // differential of exactly zero: no heading authority at all for the whole
  // first half of a long move.
  const double drive = clampVoltage(output, m_goal_max_voltage);
  const auto pair =
      chassis_math::mixDriveCorrection(drive, correction, m_goal_max_voltage);
  m_chassis.tankVoltage(pair.left * units::volt, pair.right * units::volt);

  if (m_distance_pid.targetArrived()) {
    finishGoal(false);
  } else if (timedOut()) {
    finishGoal(true);
  }
}

void ChassisController::runTurnToHeading() {
  const double output = clampVoltage(m_turn_pid.update(m_chassis.headingDeg()),
                                     m_goal_max_voltage);
  m_chassis.tankVoltage(output * units::volt, -output * units::volt);

  if (m_turn_pid.targetArrived()) {
    finishGoal(false);
  } else if (timedOut()) {
    finishGoal(true);
  }
}

void ChassisController::finishGoal(bool force_stop) {
  // The drive must end in a state the caller chose, on every exit path.
  // Returning without writing the motors left the last PID voltage latched:
  // periodic() stops calling runDriveDistance() the moment the mode goes Idle,
  // so nothing ever overwrote it. On the timeout path that voltage is whatever
  // the loop was commanding when the clock ran out -- several volts -- and the
  // robot drove on for the rest of the match.
  if (m_stop_at_end || force_stop) {
    m_chassis.stop(device::BrakeMode::Hold);
  } else {
    // stop_at_end == false means "do not stop between chained moves", so the
    // next command is entitled to a robot that is still rolling. It is not
    // entitled to one still being driven by a controller that has stopped
    // running. Command zero volts: momentum carries into the next move, and if
    // no next move comes the robot rolls to a halt instead of driving away.
    //
    // Not setBrakeMode(Coast). A zero voltage already freewheels -- the brake
    // mode only applies to motor_brake(), which stop() calls and this does not
    // -- and the mode is drive-wide state nothing here restores, so setting it
    // would leave teleop coasting after any chained move.
    m_chassis.tankVoltage(0.0 * units::volt, 0.0 * units::volt);
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
