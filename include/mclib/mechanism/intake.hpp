// mclib
#pragma once

#include "mclib/device/types.hpp"
#include "mclib/mechanism/motor_state_mechanism.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/// @brief States a two-motor intake can be commanded into.
enum class IntakeState {
  Disabled,
  Index,
  Score,
  Reverse,
};

/// @brief Ports, gearset, and per-state voltages for an Intake.
struct IntakeConfig {
  std::int8_t bottom_port = 0;
  std::int8_t top_port = 0;
  device::Gearset gearset = device::Gearset::Blue;
  double index_voltage = 12.0;
  double score_voltage = 12.0;
  double reverse_voltage = -12.0;
};

/// @brief Two-motor intake driven by IntakeState.
///
/// Index runs only the bottom motor; Score and Reverse run both. State is
/// managed by StateMechanism<IntakeState>: use its setState/getState and the
/// makeState* command factories, plus the named helpers below.
class Intake : public MotorStateMechanism<IntakeState> {
public:
  explicit Intake(const IntakeConfig& config);

  /// @brief Not copyable or movable: the voltage map installed on the base
  /// class holds `this`, so a copy or move would leave the map pointing at the
  /// source object. Subsystems are registered by pointer, so hold one in place.
  Intake(const Intake&) = delete;
  Intake& operator=(const Intake&) = delete;
  Intake(Intake&&) = delete;
  Intake& operator=(Intake&&) = delete;

  /// @brief Configuration this intake was built from.
  const IntakeConfig& config() const;

  /// @brief Voltages this intake would apply in @p state, bottom motor first.
  std::vector<double> voltagesFor(IntakeState state) const;

  /// @brief Set the brake mode of every intake motor.
  void setBrakeMode(device::BrakeMode mode);

  /// @brief Shorthand for setState(IntakeState::Disabled).
  void disable();

  std::unique_ptr<Command> makeIndexCommand();
  std::unique_ptr<Command> makeScoreCommand();
  std::unique_ptr<Command> makeReverseCommand();
  std::unique_ptr<Command> makeDisableCommand();

private:
  IntakeConfig m_config;
};

}  // namespace mechanism
}  // namespace mclib
