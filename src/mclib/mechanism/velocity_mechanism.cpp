// mclib
#include "mclib/mechanism/velocity_mechanism.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace mechanism {

VelocityMechanism::VelocityMechanism(VelocitySource velocity_source,
                                     VoltageSink voltage_sink,
                                     const VelocityMechanismConfig& config)
    : StateMechanism<double>(0.0),
      m_config(config),
      m_velocity_source(std::move(velocity_source)),
      m_voltage_sink(std::move(voltage_sink)),
      m_pid(config.kp, config.ki, config.kd) {
  // A ratio of 0 would divide by zero converting an output target into a motor
  // target, and a negative one would flip the sign of the loop so it drove away
  // from the target. An infinite one divides every target down to 0, so the
  // mechanism would sit still and the integral gate would quietly switch off.
  // None of those is ever what the caller meant, so fall back to the ungeared
  // 1.0 and store that, keeping getConfig().ratio honest about what is actually
  // running. The > 0.0 test also rejects NaN.
  if (!(std::isfinite(m_config.ratio) && m_config.ratio > 0.0)) {
    m_config.ratio = 1.0;
  }

  // PID zeroes its own output once it decides it has "arrived" at the target.
  // That is right for a position move and wrong for a velocity loop: dropping
  // the output to 0 V would immediately let the mechanism coast back down.
  // Disable arrival entirely and track settling here with atSpeed() instead.
  m_pid.setArrive(false);

  // Integral windup: during spin-up the error is huge for a long time, so an
  // ungated integral would saturate and then overshoot badly once the speed
  // arrives. Only accumulate inside integral_range_rpm of the target, and cap
  // the term's contribution in volts.
  //
  // integral_range_rpm is quoted in output RPM like the rest of the public API,
  // but the PID sees motor-RPM error, so convert. 0 stays 0 and keeps meaning
  // "no gate".
  m_pid.setIntegralRange(m_config.integral_range_rpm / m_config.ratio);
  m_pid.setIntegralMax(m_config.integral_max_volts);

  // PID also throws the integral away whenever |error| drops below its small
  // error tolerance, which defaults to 1 (one degree, for a position move).
  // A velocity loop lives inside a few RPM of its target at steady state, so
  // that default would erase the integral exactly when it is needed. Zero the
  // tolerances so only the sign-change reset survives.
  m_pid.setSmallBigErrorTolerance(0.0, 0.0);
  m_pid.setTarget(0.0);
}

void VelocityMechanism::setTargetRpm(double rpm) {
  setState(rpm);
}

double VelocityMechanism::getTargetRpm() const {
  return getState();
}

double VelocityMechanism::getCurrentMotorRpm() const {
  return m_velocity_source ? m_velocity_source() : 0.0;
}

double VelocityMechanism::getCurrentRpm() const {
  return getCurrentMotorRpm() * m_config.ratio;
}

bool VelocityMechanism::atSpeed() const {
  return m_at_speed;
}

bool VelocityMechanism::isSpinningUp() const {
  return getTargetRpm() != 0.0 && !m_at_speed;
}

void VelocityMechanism::stop() {
  setTargetRpm(0.0);
}

const VelocityMechanismConfig& VelocityMechanism::getConfig() const {
  return m_config;
}

std::unique_ptr<Command> VelocityMechanism::makeSpinCommand(double rpm) {
  return run([this, rpm]() { setTargetRpm(rpm); });
}

std::unique_ptr<Command> VelocityMechanism::makeSpinUpCommand(
    double rpm, double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, rpm, start_time]() {
        setTargetRpm(rpm);
        *start_time = static_cast<double>(pros::millis());
      },
      [this, rpm]() { setTargetRpm(rpm); },
      [](bool) {},
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(pros::millis()) - *start_time >= timeout_ms;
        return atSpeed() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> VelocityMechanism::makeStopCommand() {
  // run(), not runOnce(): an instant command would release the subsystem on
  // the same tick and let a spin default command take it straight back.
  return run([this]() { stop(); });
}

void VelocityMechanism::applyState(const double& target_rpm) {
  // target_rpm is the state, so it is output RPM. Settling is judged in that
  // same space, which is what the user quoted tolerance_rpm in.
  const double current_motor_rpm = getCurrentMotorRpm();
  const double current_rpm = current_motor_rpm * m_config.ratio;
  updateAtSpeed(target_rpm - current_rpm, target_rpm);

  if (!m_voltage_sink) {
    return;
  }

  if (target_rpm == 0.0) {
    // A zero target coasts to a halt. Never command a braking voltage: on a
    // flywheel that means driving the motor backwards against its own inertia,
    // which is hard on the gearbox and buys nothing.
    m_pid.reset();
    m_pid.setTarget(0.0);
    m_voltage_sink(0.0);
    return;
  }

  // The loop itself runs in motor RPM: that is the space the velocity source
  // reports in and the space kp/ki/kd/kv are tuned in. ratio > 0 is guaranteed
  // by the constructor, so the sign of the motor target matches the sign of the
  // output target and the clamp below still points the right way.
  const double target_motor_rpm = target_rpm / m_config.ratio;
  m_pid.setTarget(target_motor_rpm);
  const double correction = m_pid.update(current_motor_rpm);
  const double feedforward = m_config.kv * target_motor_rpm;
  double output = feedforward + correction;

  // Clamp to the sign of the target for the same reason: the loop may only
  // push the mechanism towards the target, never brake it.
  if (target_rpm > 0.0) {
    output = std::clamp(output, 0.0, m_config.max_voltage);
  } else {
    output = std::clamp(output, -m_config.max_voltage, 0.0);
  }

  m_voltage_sink(output);
}

void VelocityMechanism::onStateChanged(const double& target_rpm) {
  m_pid.reset();
  m_pid.setTarget(target_rpm / m_config.ratio);
  m_at_speed = false;
  m_in_tolerance = false;
  m_in_tolerance_since_ms = 0.0;
}

void VelocityMechanism::updateAtSpeed(double error_rpm, double target_rpm) {
  if (target_rpm == 0.0) {
    m_at_speed = false;
    m_in_tolerance = false;
    return;
  }

  // error_rpm and tolerance_rpm are both output RPM. Comparing an output-space
  // tolerance against a motor-space error is the bug this mechanism is built to
  // avoid: at ratio 2.0 it would latch at twice the speed error the user asked
  // for, and nothing would look wrong.
  if (std::fabs(error_rpm) > m_config.tolerance_rpm) {
    m_at_speed = false;
    m_in_tolerance = false;
    return;
  }

  const double now_ms = static_cast<double>(pros::millis());
  if (!m_in_tolerance) {
    m_in_tolerance = true;
    m_in_tolerance_since_ms = now_ms;
  }
  if (now_ms - m_in_tolerance_since_ms >= m_config.dwell_ms) {
    m_at_speed = true;
  }
}

}  // namespace mechanism
}  // namespace mclib
