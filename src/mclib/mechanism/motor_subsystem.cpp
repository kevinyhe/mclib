// mclib
#include "mclib/mechanism/motor_subsystem.hpp"

#include <algorithm>
#include <utility>

namespace mclib {
namespace mechanism {

MotorSubsystem::MotorSubsystem(std::initializer_list<std::int8_t> ports,
                               device::Gearset gearset)
    : MotorSubsystem(std::vector<std::int8_t>(ports), gearset) {}

MotorSubsystem::MotorSubsystem(std::vector<std::int8_t> ports,
                               device::Gearset gearset)
    : StateMechanism<double>(0.0),
      m_motors(std::move(ports), gearset) {}

void MotorSubsystem::setVoltage(double volts) {
  setState(std::clamp(volts, -12.0, 12.0));
}

void MotorSubsystem::setPercent(double percent) {
  setVoltage(std::clamp(percent, -1.0, 1.0) * 12.0);
}

void MotorSubsystem::stop() {
  setVoltage(0.0);
}

double MotorSubsystem::getCommandedVoltage() const {
  return getState();
}

device::MotorGroup& MotorSubsystem::motors() {
  return m_motors;
}

void MotorSubsystem::applyState(const double& volts) {
  m_motors.setVoltage(volts);
}

std::unique_ptr<Command> MotorSubsystem::makeVoltageCommand(double volts) {
  return makeStateCommand(std::clamp(volts, -12.0, 12.0));
}

std::unique_ptr<Command> MotorSubsystem::makePercentCommand(double percent) {
  return run([this, percent]() { setPercent(percent); });
}

std::unique_ptr<Command> MotorSubsystem::makeStopCommand() {
  return run([this]() { stop(); });
}

}  // namespace mechanism
}  // namespace mclib
