// mclib
#include "mclib/mechanism/intake.hpp"

#include <cstddef>

namespace mclib {
namespace mechanism {

Intake::Intake(const IntakeConfig& config)
    : MotorStateMechanism<IntakeState>(
          {config.bottom_port, config.top_port},
          config.gearset,
          IntakeState::Disabled,
          // Safe: the base only stores the map here and calls it from
          // applyState(), which runs after m_config is initialized.
          [this](const IntakeState& state) { return voltagesFor(state); }),
      m_config(config) {}

const IntakeConfig& Intake::config() const {
  return m_config;
}

std::vector<double> Intake::voltagesFor(IntakeState state) const {
  double bottom = 0.0;
  double top = 0.0;

  switch (state) {
    case IntakeState::Index:
      bottom = m_config.index_voltage;
      break;
    case IntakeState::Score:
      bottom = m_config.score_voltage;
      top = m_config.score_voltage;
      break;
    case IntakeState::Reverse:
      bottom = m_config.reverse_voltage;
      top = m_config.reverse_voltage;
      break;
    case IntakeState::Disabled:
    default:
      break;
  }

  return {bottom, top};
}

void Intake::setBrakeMode(device::BrakeMode mode) {
  for (std::size_t i = 0; i < motorCount(); ++i) {
    motor(i).setBrakeMode(mode);
  }
}

void Intake::disable() {
  setState(IntakeState::Disabled);
}

std::unique_ptr<Command> Intake::makeIndexCommand() {
  return makeStateCommand(IntakeState::Index);
}

std::unique_ptr<Command> Intake::makeScoreCommand() {
  return makeStateCommand(IntakeState::Score);
}

std::unique_ptr<Command> Intake::makeReverseCommand() {
  return makeStateCommand(IntakeState::Reverse);
}

std::unique_ptr<Command> Intake::makeDisableCommand() {
  return makeStateCommand(IntakeState::Disabled);
}

}  // namespace mechanism
}  // namespace mclib
