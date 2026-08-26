// mclib
#include "mclib/mechanism/intake.hpp"

namespace mclib {
namespace mechanism {

Intake::Intake(const IntakeConfig& config)
    : MotorStateMechanism<IntakeState>(
          {config.bottom_port, config.top_port},
          config.gearset,
          IntakeState::Disabled,
          [config](const IntakeState& state) {
            return voltagesForState(config, state);
          }),
      m_config(config) {}

void Intake::setState(IntakeState state) {
  MotorStateMechanism<IntakeState>::setState(state);
}

IntakeState Intake::getState() const {
  return MotorStateMechanism<IntakeState>::getState();
}

void Intake::disable() {
  setState(IntakeState::Disabled);
}

std::unique_ptr<Command> Intake::makeStateCommand(IntakeState state) {
  return MotorStateMechanism<IntakeState>::makeStateCommand(state);
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

std::vector<double> Intake::voltagesForState(const IntakeConfig& config,
                                             IntakeState state) {
  double bottom = 0.0;
  double top = 0.0;

  switch (state) {
    case IntakeState::Index:
      bottom = config.index_voltage;
      break;
    case IntakeState::Score:
      bottom = config.score_voltage;
      top = config.score_voltage;
      break;
    case IntakeState::Reverse:
      bottom = config.reverse_voltage;
      top = config.reverse_voltage;
      break;
    case IntakeState::Disabled:
    default:
      break;
  }

  return {bottom, top};
}

}  // namespace mechanism
}  // namespace mclib
