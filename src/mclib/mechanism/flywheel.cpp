// mclib
#include "mclib/mechanism/flywheel.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace mechanism {

Flywheel::Flywheel(const FlywheelConfig& config)
    : StateMechanism<double>(0.0),
      m_config(config),
      m_motors(config.motor_ports, config.gearset),
      m_pid(config.kp, config.ki, config.kd) {
  // Trap 1: integral windup during spin-up. Spin-up error is large and lasts
  // for seconds, so an unbounded integral saturates the output and then keeps
  // pushing long past the target. Two guards: the integral only accumulates
  // once the error is inside integral_range_rpm, and its own contribution is
  // capped at integral_max_volts, well under max_voltage so the cap actually
  // binds before the output clamp does.
  //
  // PID reads an integral_range of 0 as "no band at all", which is precisely
  // the windup this is meant to prevent, so a non-positive band falls back to
  // a few times the tolerance.
  const double integral_range = m_config.integral_range_rpm > 0.0
                                    ? m_config.integral_range_rpm
                                    : m_config.tolerance_rpm * 4.0;
  m_pid.setIntegralRange(integral_range);
  m_pid.setIntegralMax(m_config.integral_max_volts);

  // PID's own arrival logic latches and then forces its output to zero, which
  // would cut the correction the moment the wheel reached speed. Turn it off
  // and do the tolerance-plus-dwell check in applyState instead.
  m_pid.setArrive(false);

  // With arrive off, small_error_tolerance only still controls one thing: PID
  // dumps the accumulated integral whenever the error is inside it. Feeding it
  // the at-speed tolerance would drop the integral in a single tick right at
  // the edge of the band, the wheel would sag back out, and atSpeed() would
  // chatter forever. Keep it at zero so the integral survives inside the band.
  m_pid.setSmallBigErrorTolerance(0.0, 0.0);
}

Flywheel::Flywheel(std::initializer_list<std::int8_t> ports,
                   device::Gearset gearset)
    : Flywheel([&] {
        // Set only the two fields this constructor takes and let every other
        // field keep its in-class default. Positional aggregate initialization
        // here would silently reset the gains.
        FlywheelConfig config;
        config.motor_ports = std::vector<std::int8_t>(ports);
        config.gearset = gearset;
        return config;
      }()) {}

void Flywheel::setTargetRpm(double rpm) {
  setState(rpm);
}

void Flywheel::stop() {
  setTargetRpm(0.0);
}

double Flywheel::getTargetRpm() const {
  return getState();
}

double Flywheel::getCurrentRpm() const {
  return m_motors.getAverageActualVelocity() * m_config.ratio;
}

bool Flywheel::atSpeed() const {
  return m_at_speed;
}

std::unique_ptr<Command> Flywheel::makeSpinCommand(double rpm) {
  return run([this, rpm]() { setTargetRpm(rpm); });
}

std::unique_ptr<Command> Flywheel::makeSpinUpCommand(double rpm,
                                                     double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, rpm, start_time]() {
        setTargetRpm(rpm);
        *start_time = static_cast<double>(pros::millis());
      },
      [this, rpm]() { setTargetRpm(rpm); },
      [](bool) {
        // Deliberately leaves the flywheel spinning. A shooter almost always
        // wants to hold speed once spin-up finishes; makeStopCommand() winds
        // it down.
      },
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(pros::millis()) - *start_time >= timeout_ms;
        return atSpeed() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Flywheel::makeStopCommand() {
  return run([this]() { stop(); });
}

void Flywheel::applyState(const double& target_rpm) {
  const double current_rpm = getCurrentRpm();
  const double error = target_rpm - current_rpm;

  // Tolerance plus dwell, so a single tick passing through the band on the way
  // up does not report at speed.
  const double now_ms = static_cast<double>(pros::millis());
  if (target_rpm > 0.0 && std::fabs(error) <= m_config.tolerance_rpm) {
    if (!m_in_band) {
      m_in_band = true;
      m_in_band_since_ms = now_ms;
    }
    m_at_speed = now_ms - m_in_band_since_ms >= m_config.dwell_ms;
  } else {
    m_in_band = false;
    m_at_speed = false;
  }

  if (target_rpm <= 0.0) {
    // Coast down. Never drive the motors to brake a flywheel.
    m_motors.setVoltage(0.0);
    return;
  }

  m_pid.setTarget(target_rpm);
  const double correction = m_pid.update(current_rpm);
  const double feedforward = m_config.kv * target_rpm;

  // Trap 2: the output must never go negative. A negative voltage brakes
  // against the flywheel's inertia, which stalls the motors, spikes current
  // and browns out the brain. Overspeed is corrected by letting the wheel
  // coast, so the floor is 0 V rather than -max_voltage.
  const double output =
      std::clamp(feedforward + correction, 0.0, m_config.max_voltage);
  m_motors.setVoltage(output);
}

void Flywheel::onStateChanged(const double& target_rpm) {
  // reset() clears the accumulated integral so a target change does not carry
  // the old target's windup into the new one. It leaves setArrive(false) in
  // place.
  m_pid.reset();
  m_pid.setTarget(target_rpm);
  m_at_speed = false;
  m_in_band = false;
}

}  // namespace mechanism
}  // namespace mclib
