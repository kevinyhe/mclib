// mclib
#include "mclib/mechanism/arm.hpp"

#include "pros/rtos.hpp"

#include <algorithm>

namespace mclib {
namespace mechanism {

Arm::Arm(const ArmConfig& config)
    : StateMechanism<double>(0.0),
      m_config(config),
      m_motors(config.motor_ports, config.gearset),
      m_rotation(config.rotation_port, config.rotation_reversed),
      m_pid(config.kp, config.ki, config.kd) {
  m_pid.setSmallBigErrorTolerance(config.small_error_deg, config.big_error_deg);
  m_pid.setSmallBigErrorDuration(config.small_duration_ms,
                                 config.big_duration_ms);
  m_pid.setDerivativeTolerance(config.derivative_tolerance);
}

Arm::Arm(std::initializer_list<std::int8_t> motor_ports,
         std::int8_t rotation_port,
         device::Gearset gearset)
    : Arm(ArmConfig{std::vector<std::int8_t>(motor_ports),
                    rotation_port,
                    false,
                    gearset}) {}

void Arm::moveTo(double target_deg) {
  m_manual = false;
  setState(target_deg);
  m_pid.setTarget(target_deg);
}

void Arm::setManualVoltage(double volts) {
  m_manual = true;
  m_manual_voltage = clampVoltage(volts, m_config.max_voltage);
}

void Arm::stop() {
  setManualVoltage(0.0);
}

double Arm::positionDeg() const {
  return m_rotation.getPositionDeg();
}

double Arm::targetDeg() const {
  return getState();
}

bool Arm::atTarget() {
  return m_pid.targetArrived();
}

std::unique_ptr<Command> Arm::makeMoveToCommand(double target_deg,
                                                double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, target_deg, start_time]() {
        moveTo(target_deg);
        *start_time = static_cast<double>(pros::millis());
      },
      []() {},
      [this](bool interrupted) {
        if (interrupted) {
          stop();
        }
      },
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(pros::millis()) - *start_time >= timeout_ms;
        return atTarget() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Arm::makeManualCommand(double volts) {
  return run([this, volts]() { setManualVoltage(volts); });
}

std::unique_ptr<Command> Arm::makeStopCommand() {
  return run([this]() { stop(); });
}

void Arm::applyState(const double& target_deg) {
  if (m_manual) {
    m_motors.setVoltage(m_manual_voltage);
    return;
  }

  m_pid.setTarget(target_deg);
  const double output =
      clampVoltage(m_pid.update(positionDeg()), m_config.max_voltage);
  m_motors.setVoltage(output);
}

void Arm::onStateChanged(const double& target_deg) {
  m_manual = false;
  m_pid.reset();
  m_pid.setTarget(target_deg);
}

double Arm::clampVoltage(double volts, double max_voltage) {
  return std::clamp(volts, -max_voltage, max_voltage);
}

}  // namespace mechanism
}  // namespace mclib
