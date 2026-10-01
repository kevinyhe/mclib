// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/characterize.hpp"

#include "api.h"
#include "mclib/control/async_motion.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/robot_state.hpp"

#include <cmath>
#include <cstdint>

namespace mclib {
namespace control {

namespace {

double encoderInches(double degrees) {
  DriveHardware* drive = boundDrive();
  if (drive == nullptr) return NAN;
  return drive->driveGeometry().encoderToDistance(degrees * units::degree).in();
}

// The same stop checks every motion makes, for the open-loop routines.
MotionResult stopReason(std::uint32_t start, double limit_ms) {
  if (cancelRequested(CancelToken::Motion)) return MotionResult::Cancelled;
  if (pros::competition::is_disabled()) return MotionResult::Disabled;
  if (!std::isfinite(getInertialHeading()) || !std::isfinite(getLeftRotationDegree()) ||
      !std::isfinite(getRightRotationDegree())) {
    return MotionResult::InvalidValue;
  }
  if (static_cast<double>(static_cast<std::uint32_t>(pros::millis() - start)) >= limit_ms)
    return MotionResult::TimedOut;
  return MotionResult::Reached;  // keep going
}

// Hold (left, right) for @p ms, recording each 10 ms tick. Returns Reached
// when the hold ran its time, otherwise why it stopped.
MotionResult hold(TuningLog& log, int phase, double left, double right, double ms) {
  DriveHardware* drive = boundDrive();
  const std::uint32_t start = pros::millis();
  while (true) {
    const MotionResult reason = stopReason(start, ms);
    if (reason == MotionResult::TimedOut) return MotionResult::Reached;
    if (reason != MotionResult::Reached) return reason;
    drive->setDriveVoltage(left, right);
    log.record(phase, left, right);
    pros::delay(10);
  }
}

MotionResult runSteps(TuningLog& log, int phase, const std::vector<QVoltage>& volts, QTime each,
                      bool spin) {
  if (requireDrive(spin ? "characterizeSpin" : "characterizeDrive") == nullptr)
    return MotionResult::NoDrive;
  if (!(each.ms() > 0)) return MotionResult::InvalidValue;
  for (const QVoltage v : volts)
    if (!std::isfinite(v.volts())) return MotionResult::InvalidValue;
  clearCancel(CancelToken::Motion);
  log.zeroEncoders();
  MotionResult result = MotionResult::Reached;
  for (const QVoltage v : volts) {
    const double volts_out = std::fabs(v.volts());
    for (const double direction : {1.0, -1.0}) {
      const double left = direction * volts_out;
      const double right = spin ? -left : left;
      result = hold(log, phase, left, right, each.ms());
      if (result == MotionResult::Reached) result = hold(log, 0, 0, 0, 300);
      if (result != MotionResult::Reached) break;
    }
    if (result != MotionResult::Reached) break;
  }
  boundDrive()->setDriveVoltage(0, 0);
  stopChassis(device::BrakeMode::Hold);
  return result;
}

}  // namespace

TuningLog::TuningLog(telemetry::Logger& logger)
    : m_logger(logger),
      m_phase(logger.addNumber("phase")),
      m_left_cmd(logger.addNumber("left_cmd", "V")),
      m_right_cmd(logger.addNumber("right_cmd", "V")),
      m_left(logger.addNumber("left", "in")),
      m_right(logger.addNumber("right", "in")),
      m_heading(logger.addNumber("heading", "deg")),
      m_x(logger.addNumber("x", "in")),
      m_y(logger.addNumber("y", "in")),
      m_target_x(logger.addNumber("target_x", "in")),
      m_target_y(logger.addNumber("target_y", "in")),
      m_target_heading(logger.addNumber("target_heading", "deg")) {
  zeroEncoders();
}

void TuningLog::zeroEncoders() {
  m_left_zero = getLeftRotationDegree();
  m_right_zero = getRightRotationDegree();
}

void TuningLog::record(int phase, double left_volts, double right_volts, double target_x_in,
                       double target_y_in, double target_heading_deg) {
  const Pose2D pose = robotState().pose();
  m_logger.set(m_phase, static_cast<double>(phase));
  m_logger.set(m_left_cmd, left_volts);
  m_logger.set(m_right_cmd, right_volts);
  m_logger.set(m_left, encoderInches(getLeftRotationDegree() - m_left_zero));
  m_logger.set(m_right, encoderInches(getRightRotationDegree() - m_right_zero));
  m_logger.set(m_heading, getInertialHeading());
  m_logger.set(m_x, pose.x);
  m_logger.set(m_y, pose.y);
  m_logger.set(m_target_x, target_x_in);
  m_logger.set(m_target_y, target_y_in);
  m_logger.set(m_target_heading, target_heading_deg);
  m_logger.sample();
}

MotionResult characterizeDrive(TuningLog& log, std::vector<QVoltage> volts, QTime each) {
  return runSteps(log, 1, volts, each, false);
}

MotionResult characterizeSpin(TuningLog& log, std::vector<QVoltage> volts, QTime each) {
  return runSteps(log, 2, volts, each, true);
}

MotionResult traceMotion(TuningLog& log, std::function<MotionResult()> motion) {
  AsyncMotion running = AsyncMotion::start(std::move(motion));
  while (running.isRunning()) {
    const MotionTelemetry t = robotState().motionTelemetry();
    const double drive = std::isfinite(t.drive_volts) ? t.drive_volts : 0.0;
    const double yaw = std::isfinite(t.yaw_volts) ? t.yaw_volts : 0.0;
    log.record(3, drive + yaw, drive - yaw, t.carrot_x, t.carrot_y, t.target_heading_deg);
    pros::delay(10);
  }
  log.record(0, 0, 0);
  return running.wait();
}

}  // namespace control
}  // namespace mclib
