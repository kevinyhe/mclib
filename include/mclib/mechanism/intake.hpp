// mclib
#pragma once

#include "mclib/mechanism/motor_state_mechanism.hpp"

#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

enum class IntakeState {
  Disabled,
  Index,
  Score,
  Reverse,
};

struct IntakeConfig {
  std::int8_t bottom_port = 0;
  std::int8_t top_port = 0;
  device::Gearset gearset = device::Gearset::Blue;
  double index_voltage = 12.0;
  double score_voltage = 12.0;
  double reverse_voltage = -12.0;
};

class Intake : public MotorStateMechanism<IntakeState> {
public:
  explicit Intake(const IntakeConfig& config);

  void setState(IntakeState state);
  IntakeState getState() const;
  void disable();

  std::unique_ptr<Command> makeStateCommand(IntakeState state);
  std::unique_ptr<Command> makeIndexCommand();
  std::unique_ptr<Command> makeScoreCommand();
  std::unique_ptr<Command> makeReverseCommand();
  std::unique_ptr<Command> makeDisableCommand();

private:
  static std::vector<double> voltagesForState(const IntakeConfig& config,
                                              IntakeState state);

  IntakeConfig m_config;
};

}  // namespace mechanism
}  // namespace mclib
