// mclib
#include "mclib/mechanism/pneumatic_subsystem.hpp"

#include <utility>

namespace mclib {
namespace mechanism {

PneumaticSubsystem::PneumaticSubsystem(std::initializer_list<char> adi_ports,
                                       bool default_state,
                                       bool extended_state)
    : PneumaticSubsystem(makePneumatics(adi_ports, default_state, extended_state),
                         default_state) {}

PneumaticSubsystem::PneumaticSubsystem(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
    bool initial_state)
    : StateMechanism<bool>(initial_state),
      m_pneumatics(std::move(pneumatics)),
      m_group(m_pneumatics) {}

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

std::unique_ptr<Command> PneumaticSubsystem::makeSetCommand(bool extended) {
  return makeStateCommand(extended);
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

void PneumaticSubsystem::applyState(const bool& extended) {
  if (extended) {
    m_group.extend();
  } else {
    m_group.retract();
  }
}

std::vector<std::shared_ptr<device::Pneumatic>> PneumaticSubsystem::makePneumatics(
    std::initializer_list<char> adi_ports,
    bool default_state,
    bool extended_state) {
  std::vector<std::shared_ptr<device::Pneumatic>> pneumatics;
  pneumatics.reserve(adi_ports.size());

  for (const char port : adi_ports) {
    pneumatics.push_back(
        std::make_shared<device::Pneumatic>(port, default_state, extended_state));
  }

  return pneumatics;
}

}  // namespace mechanism
}  // namespace mclib
