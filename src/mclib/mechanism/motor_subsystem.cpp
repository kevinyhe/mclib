// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/mechanism/motor_subsystem.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace mechanism {

namespace {
/// @brief Clamp to the motor range. NaN and infinity become 0 V, which is what
/// device::MotorGroup sends for them, so getCommandedVoltage() reports what
/// the motors actually get.
double toVolts(double volts) {
  if (!std::isfinite(volts)) return 0.0;
  return std::clamp(volts, -12.0, 12.0);
}
}  // namespace

MotorSubsystem::MotorSubsystem(std::initializer_list<std::int8_t> ports,
                               device::Gearset gearset)
    : MotorSubsystem(std::vector<std::int8_t>(ports), gearset) {}

MotorSubsystem::MotorSubsystem(std::vector<std::int8_t> ports,
                               device::Gearset gearset)
    : StateMechanism<double>(0.0),
      m_motors(std::move(ports), gearset) {}

void MotorSubsystem::setVoltage(double volts) {
  setState(toVolts(volts));
}

void MotorSubsystem::setPercent(double percent) {
  if (!std::isfinite(percent)) {
    setVoltage(0.0);
    return;
  }
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
  return makeStateCommand(toVolts(volts));
}

std::unique_ptr<Command> MotorSubsystem::makePercentCommand(double percent) {
  return run([this, percent]() { setPercent(percent); });
}

std::unique_ptr<Command> MotorSubsystem::makeStopCommand() {
  return run([this]() { stop(); });
}

}  // namespace mechanism
}  // namespace mclib
