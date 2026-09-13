// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/mechanism/pneumatic_subsystem.hpp"

#include <utility>

namespace mclib {
namespace mechanism {

PneumaticSubsystem::PneumaticSubsystem(std::initializer_list<char> adi_ports,
                                       bool initial_extended,
                                       bool extended_state)
    : PneumaticSubsystem(makePneumatics(adi_ports, initial_extended, extended_state),
                         initial_extended) {}

PneumaticSubsystem::PneumaticSubsystem(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
    bool initial_extended)
    : StateMechanism<bool>(initial_extended),
      m_pneumatics(std::move(pneumatics)),
      m_group(m_pneumatics) {
  // Drive the hardware to the seeded logical state so the cache and the
  // solenoids agree before the first periodic() tick.
  m_group.set_value(initial_extended);
}

void PneumaticSubsystem::setExtended(bool extended) {
  setState(extended);
}

bool PneumaticSubsystem::isExtended() const {
  return getState();
}

void PneumaticSubsystem::extend() {
  setExtended(true);
}

void PneumaticSubsystem::retract() {
  setExtended(false);
}

void PneumaticSubsystem::toggle() {
  setExtended(!isExtended());
}

std::size_t PneumaticSubsystem::count() const {
  return m_pneumatics.size();
}

device::PneumaticGroup& PneumaticSubsystem::group() {
  return m_group;
}

std::vector<bool> PneumaticSubsystem::rawValues() const {
  std::vector<bool> values;
  values.reserve(m_pneumatics.size());

  for (const auto& pneumatic : m_pneumatics) {
    values.push_back(pneumatic ? pneumatic->get_value() : false);
  }

  return values;
}

std::unique_ptr<Command> PneumaticSubsystem::makeSetCommand(bool extended) {
  return makeStateOnceCommand(extended);
}

std::unique_ptr<Command> PneumaticSubsystem::makeExtendCommand() {
  return makeSetCommand(true);
}

std::unique_ptr<Command> PneumaticSubsystem::makeRetractCommand() {
  return makeSetCommand(false);
}

std::unique_ptr<Command> PneumaticSubsystem::makeToggleCommand() {
  return runOnce([this]() { toggle(); });
}

std::unique_ptr<Command> PneumaticSubsystem::makeHoldCommand(bool extended) {
  return makeStateCommand(extended);
}

std::unique_ptr<Command> PneumaticSubsystem::makeExtendForCommand(QTime duration) {
  return makeStateForCommand(true, duration);
}

void PneumaticSubsystem::applyState(const bool& extended) {
  m_group.set_value(extended);
}

std::vector<std::shared_ptr<device::Pneumatic>> PneumaticSubsystem::makePneumatics(
    std::initializer_list<char> adi_ports,
    bool initial_extended,
    bool extended_state) {
  std::vector<std::shared_ptr<device::Pneumatic>> pneumatics;
  pneumatics.reserve(adi_ports.size());

  for (const char port : adi_ports) {
    pneumatics.push_back(
        std::make_shared<device::Pneumatic>(port, initial_extended, extended_state));
  }

  return pneumatics;
}

}  // namespace mechanism
}  // namespace mclib
