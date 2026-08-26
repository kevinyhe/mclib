// mclib
#include "mclib/mechanism/toggle_mechanism.hpp"

#include <utility>

namespace mclib {
namespace mechanism {

ToggleMechanism::ToggleMechanism(Actuator actuator, ToggleMechanismConfig config)
    : StateMechanism<bool>(config.initial_extended),
      m_actuator(std::move(actuator)),
      m_inverted(config.inverted) {
  if (config.apply_on_construct && m_actuator) {
    m_actuator(rawValue(config.initial_extended));
  }
}

ToggleMechanism::ToggleMechanism(std::shared_ptr<device::Pneumatic> pneumatic,
                                 ToggleMechanismConfig config)
    : ToggleMechanism(makeActuator(std::move(pneumatic)), config) {}

ToggleMechanism::ToggleMechanism(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
    ToggleMechanismConfig config)
    : ToggleMechanism(makeActuator(std::move(pneumatics)), config) {}

void ToggleMechanism::set(bool extended) {
  setState(extended);
  applyState(extended);
}

void ToggleMechanism::apply() {
  applyState(getState());
}

void ToggleMechanism::extend() {
  set(true);
}

void ToggleMechanism::retract() {
  set(false);
}

void ToggleMechanism::toggle() {
  set(!isExtended());
}

bool ToggleMechanism::isExtended() const {
  return getState();
}

bool ToggleMechanism::isInverted() const {
  return m_inverted;
}

bool ToggleMechanism::rawValue(bool extended) const {
  return m_inverted ? !extended : extended;
}

std::unique_ptr<Command> ToggleMechanism::makeSetCommand(bool extended) {
  return runOnce([this, extended]() { set(extended); });
}

std::unique_ptr<Command> ToggleMechanism::makeExtendCommand() {
  return makeSetCommand(true);
}

std::unique_ptr<Command> ToggleMechanism::makeRetractCommand() {
  return makeSetCommand(false);
}

std::unique_ptr<Command> ToggleMechanism::makeToggleCommand() {
  return runOnce([this]() { toggle(); });
}

std::unique_ptr<Command> ToggleMechanism::makeSetForCommand(bool extended,
                                                            QTime duration) {
  return makeStateForCommand(extended, duration);
}

std::unique_ptr<Command> ToggleMechanism::makeExtendForCommand(QTime duration) {
  return makeSetForCommand(true, duration);
}

std::unique_ptr<Command> ToggleMechanism::makeRetractForCommand(QTime duration) {
  return makeSetForCommand(false, duration);
}

void ToggleMechanism::applyState(const bool& extended) {
  if (m_actuator) {
    m_actuator(rawValue(extended));
  }
}

ToggleMechanism::Actuator ToggleMechanism::makeActuator(
    std::shared_ptr<device::Pneumatic> pneumatic) {
  return [pneumatic = std::move(pneumatic)](bool value) {
    if (pneumatic) {
      pneumatic->set_value(value);
    }
  };
}

ToggleMechanism::Actuator ToggleMechanism::makeActuator(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics) {
  return [pneumatics = std::move(pneumatics)](bool value) {
    for (const auto& pneumatic : pneumatics) {
      if (pneumatic) {
        pneumatic->set_value(value);
      }
    }
  };
}

}  // namespace mechanism
}  // namespace mclib
