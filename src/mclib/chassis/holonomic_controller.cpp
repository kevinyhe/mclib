// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/holonomic_controller.hpp"

#include "mclib/chassis/holonomic_math.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <memory>

namespace mclib {

HolonomicController::HolonomicController(HolonomicChassis& chassis,
                                         HolonomicControllerConfig config)
    : m_chassis(chassis),
      m_config(config),
      m_translation_pid(config.translation_pid.kp,
                        config.translation_pid.ki,
                        config.translation_pid.kd),
      m_heading_pid(config.heading_pid.kp,
                    config.heading_pid.ki,
                    config.heading_pid.kd) {
  applyExit(m_translation_pid, m_config.translation_exit);
  applyExit(m_heading_pid, m_config.heading_exit);
  // Both loops keep correcting after they arrive. The default drops the
  // output to zero on arrival, and on a holonomic drive the loop that arrives
  // first would then stop holding while the other one is still moving the
  // robot -- the heading turn scrubs the position off by an inch and nothing
  // brings it back. Arrival must remain true on both axes on the same tick.
  m_translation_pid.setHoldOutput(true);
  m_heading_pid.setHoldOutput(true);
  m_translation_pid.setLatchArrival(false);
  m_heading_pid.setLatchArrival(false);
}

void HolonomicController::setConfig(const HolonomicControllerConfig& config) {
  m_config = config;
  m_translation_pid.setCoefficient(config.translation_pid.kp,
                                   config.translation_pid.ki,
                                   config.translation_pid.kd);
  m_heading_pid.setCoefficient(config.heading_pid.kp,
                               config.heading_pid.ki,
                               config.heading_pid.kd);
  applyExit(m_translation_pid, m_config.translation_exit);
  applyExit(m_heading_pid, m_config.heading_exit);
}

HolonomicControllerConfig HolonomicController::getConfig() const {
  return m_config;
}

void HolonomicController::moveToPose(Pose2D target, QTime timeout, bool stop_at_end) {
  startGoal(target.x, target.y, target.theta, timeout, stop_at_end);
}

void HolonomicController::moveToPoint(QLength x, QLength y, QTime timeout, bool stop_at_end) {
  const Pose2D now = control::robotState().pose();
  startGoal(x.in(), y.in(), now.theta, timeout, stop_at_end);
}

void HolonomicController::cancel() {
  // Same reasoning as ChassisController::cancel(): a group's end(true)
  // reaches finished children too, and by then something else may own the
  // motors. Only an active goal gets stopped.
  if (!m_active) {
    return;
  }
  finishGoal(true);
}

bool HolonomicController::isSettled() const {
  return m_settled;
}

bool HolonomicController::isActive() const {
  return m_active;
}

void HolonomicController::periodic() {
  if (m_active) {
    runMoveToPose();
  }
}

std::unique_ptr<Command> HolonomicController::makeMoveToPoseCommand(
    Pose2D target, QTime timeout, bool stop_at_end) {
  return std::make_unique<FunctionalCommand>(
      [this, target, timeout, stop_at_end]() {
        moveToPose(target, timeout, stop_at_end);
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

std::unique_ptr<Command> HolonomicController::makeMoveToPointCommand(
    QLength x, QLength y, QTime timeout, bool stop_at_end) {
  return std::make_unique<FunctionalCommand>(
      [this, x, y, timeout, stop_at_end]() {
        moveToPoint(x, y, timeout, stop_at_end);
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

std::unique_ptr<Command> HolonomicController::makeDriveCommand(
    device::Controller& controller,
    control::DriveCurveConfig curve,
    device::AnalogAxis forward_axis,
    device::AnalogAxis strafe_axis,
    device::AnalogAxis turn_axis) {
  // One curve per axis so each slews on its own. Shared between the three
  // callbacks, which FunctionalCommand stores as separate std::functions.
  struct Curves {
    control::DriveCurve forward;
    control::DriveCurve strafe;
    control::DriveCurve turn;
  };
  auto curves = std::make_shared<Curves>(Curves{control::DriveCurve(curve),
                                                control::DriveCurve(curve),
                                                control::DriveCurve(curve)});

  return std::make_unique<FunctionalCommand>(
      [curves]() {
        curves->forward.reset();
        curves->strafe.reset();
        curves->turn.reset();
      },
      [this, &controller, curves, forward_axis, strafe_axis, turn_axis]() {
        const double forward = curves->forward.apply(controller.getAnalog(forward_axis) / 127.0);
        const double strafe = curves->strafe.apply(controller.getAnalog(strafe_axis) / 127.0);
        const double turn = curves->turn.apply(controller.getAnalog(turn_axis) / 127.0);
        if (m_config.field_centric_teleop) {
          // The sticks are field axes here: strafe is field +X, forward is field +Y.
          m_chassis.driveFieldCentric(strafe, forward, turn);
        } else {
          m_chassis.drive(forward, strafe, turn);
        }
      },
      [this](bool /*interrupted*/) {
        // Whatever takes over writes the motors itself on its first tick;
        // until then the last stick value must not stay latched.
        m_chassis.drive(0.0, 0.0, 0.0);
      },
      []() { return false; },
      std::initializer_list<Subsystem*>{this});
}

void HolonomicController::applyExit(PID& pid, const PIDExit& exit) {
  pid.setSmallBigErrorTolerance(exit.small_error, exit.big_error);
  pid.setSmallBigErrorDuration(exit.small_duration.ms(), exit.big_duration.ms());
  pid.setDerivativeTolerance(exit.derivative);
}

void HolonomicController::startGoal(double x_in, double y_in, double theta_rad,
                                    QTime timeout, bool stop_at_end) {
  m_target_x_in = x_in;
  m_target_y_in = y_in;
  m_target_theta_rad = wrapAngle(theta_rad);
  m_start_time_ms = mclib::time::millis();
  m_timeout_ms = timeout.ms();
  m_stop_at_end = stop_at_end;
  m_active = true;
  m_settled = false;

  // Both loops run on an error the tick computes -- remaining distance and
  // wrapped heading delta -- fed in as a negative input against a target of
  // zero, so `target - input` is the error itself. The heading one is done
  // this way because the pose theta is wrapped: at 179 deg with a target of
  // -179 the naive error is -358 and the robot would turn the long way. A
  // wrapped delta is 2 deg, the short way, on every tick.
  m_translation_pid.reset();
  m_heading_pid.reset();
  m_translation_pid.setTarget(0.0);
  m_heading_pid.setTarget(0.0);
}

void HolonomicController::runMoveToPose() {
  const Pose2D pose = control::robotState().pose();
  const double max_volts = m_config.max_voltage.volts();
  if (!(std::isfinite(max_volts) && max_volts > 0.0) ||
      !std::isfinite(m_chassis.headingDeg()) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
      !std::isfinite(m_target_x_in) || !std::isfinite(m_target_y_in) ||
      !std::isfinite(m_target_theta_rad)) {
    // No authority at all: nothing sensible can be commanded. Stop cleanly.
    finishGoal(true);
    return;
  }

  // Translation: one loop on the remaining distance, pointed along the
  // field-frame bearing to the target. With equal gains this is what x and y
  // loops would sum to, and its exit is "within N inches", radially, which
  // is what a tolerance in inches means.
  const double dx = m_target_x_in - pose.x;
  const double dy = m_target_y_in - pose.y;
  const double distance = std::hypot(dx, dy);
  const double drive =
      std::clamp(m_translation_pid.update(-distance), -max_volts, max_volts);
  double field_x = 0.0;
  double field_y = 0.0;
  if (distance > 1e-9) {
    field_x = drive * dx / distance;
    field_y = drive * dy / distance;
  }
  const holonomic::RobotFrameInput robot =
      holonomic::fieldToRobot(field_x, field_y, pose.theta);

  // Heading: wrapped delta in degrees, positive when the target is clockwise
  // of the robot, so a positive output is a clockwise turn.
  const double heading_error_deg =
      wrapAngle(m_target_theta_rad - pose.theta) * 180.0 / kPi;
  const double turn =
      std::clamp(m_heading_pid.update(-heading_error_deg), -max_volts, max_volts);

  // mix() works in fractions and scales the four down together when they
  // would exceed the rail, so the direction of travel survives saturation.
  holonomic::WheelSpeeds wheels = holonomic::mix(robot.forward / max_volts,
                                                 robot.strafe / max_volts,
                                                 turn / max_volts,
                                                 m_chassis.kind());
  wheels.front_left *= max_volts;
  wheels.front_right *= max_volts;
  wheels.back_left *= max_volts;
  wheels.back_right *= max_volts;
  m_chassis.driveVoltage(wheels);

  if (m_translation_pid.targetArrived() && m_heading_pid.targetArrived()) {
    finishGoal(false);
  } else if (timedOut()) {
    finishGoal(true);
  }
}

void HolonomicController::finishGoal(bool force_stop) {
  // Every exit writes the motors. Leaving the last PID voltage latched is how
  // a timed-out goal keeps driving for the rest of the match -- see
  // ChassisController::finishGoal() for the full account.
  if (m_stop_at_end || force_stop) {
    m_chassis.stop(device::BrakeMode::Hold);
  } else {
    // A chained move is entitled to a rolling robot, not to one still being
    // driven by a loop that has stopped running. Zero volts freewheels; the
    // brake mode is left alone for the same reason as in ChassisController.
    m_chassis.driveVoltage(holonomic::WheelSpeeds{});
  }
  m_active = false;
  m_settled = true;
}

bool HolonomicController::timedOut() const {
  return m_timeout_ms > 0.0 &&
         (static_cast<double>(mclib::time::millis()) - m_start_time_ms) >= m_timeout_ms;
}

}  // namespace mclib
