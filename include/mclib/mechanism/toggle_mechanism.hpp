// mclib
#pragma once

#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Settings for a ToggleMechanism.
 */
struct ToggleMechanismConfig {
  /// Logical state the mechanism starts in: true means "extended".
  bool initial_extended = false;
  /// When true the actuator is driven with the opposite of the logical state,
  /// so an inverted device still reports "extended" correctly. Use this for
  /// raw actuators only. device::Pneumatic already takes a logical value and
  /// does its own inversion through its extended_state flag, so setting this
  /// with either Pneumatic constructor inverts twice and leaves the Pneumatic's
  /// own get_value() reporting the opposite of isExtended(). For a Pneumatic,
  /// pass extended_state = false to the device instead.
  bool inverted = false;
  /// When true the actuator is written once from the constructor so the
  /// hardware matches the initial logical state before the scheduler runs.
  bool apply_on_construct = true;
};

/**
 * @brief A generic two-state mechanism driven by an injected actuator.
 *
 * The logical state ("is extended") is kept separate from the raw value handed
 * to the actuator. With ToggleMechanismConfig::inverted set, the actuator
 * receives the negation of the logical state, so isExtended() still answers the
 * question the caller actually asked.
 *
 * The actuator is a std::function<void(bool)>, so the same type covers
 * solenoids, motor-driven two-position mechanisms, and test mocks.
 *
 * The actuator is written every scheduler tick from applyState(), which only
 * happens once the mechanism is registered with
 * CommandScheduler::registerSubsystem. set(), extend(), retract() and toggle()
 * also write the actuator right away, whether or not the value changed, so
 * they work without the scheduler and can resync hardware that drifted.
 * Calling the inherited setState() directly does not write the actuator until
 * the next periodic().
 */
class ToggleMechanism : public StateMechanism<bool> {
public:
  using Actuator = std::function<void(bool)>;

  /**
   * @brief Construct from an arbitrary actuator.
   *
   * @param actuator Called with the raw value whenever the state is applied.
   * @param config Initial state, inversion, and construct-time apply.
   */
  explicit ToggleMechanism(Actuator actuator, ToggleMechanismConfig config = {});

  /**
   * @brief Construct from a single pneumatic.
   *
   * @param pneumatic The pneumatic to drive. A null pointer is ignored.
   * @param config Initial state, inversion, and construct-time apply.
   */
  explicit ToggleMechanism(std::shared_ptr<device::Pneumatic> pneumatic,
                           ToggleMechanismConfig config = {});

  /**
   * @brief Construct from a group of pneumatics driven together.
   *
   * @param pneumatics The pneumatics to drive. Null entries are ignored.
   * @param config Initial state, inversion, and construct-time apply.
   */
  explicit ToggleMechanism(std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
                           ToggleMechanismConfig config = {});

  /// @brief Set the logical state and write the actuator. True means extended.
  void set(bool extended);
  /// @brief Write the actuator with the current logical state.
  void apply();
  /// @brief Move to the extended state.
  void extend();
  /// @brief Move to the retracted state.
  void retract();
  /// @brief Flip to the opposite of the current logical state.
  void toggle();
  /// @brief Whether the mechanism is logically extended.
  bool isExtended() const;
  /// @brief Whether the actuator is driven with the inverse of the state.
  bool isInverted() const;
  /// @brief The raw value the actuator is given for a logical state.
  bool rawValue(bool extended) const;

  /// @brief Command that sets the state once and finishes immediately.
  std::unique_ptr<Command> makeSetCommand(bool extended);
  /// @brief Command that extends once and finishes immediately.
  std::unique_ptr<Command> makeExtendCommand();
  /// @brief Command that retracts once and finishes immediately.
  std::unique_ptr<Command> makeRetractCommand();
  /// @brief Command that toggles once and finishes immediately.
  std::unique_ptr<Command> makeToggleCommand();

  /// @brief Command that holds a state for a duration, then finishes and
  /// leaves the mechanism in that state. It does not restore the old state.
  std::unique_ptr<Command> makeSetForCommand(bool extended, QTime duration);
  /// @brief Command that holds the extended state for a duration, then
  /// finishes and stays extended.
  std::unique_ptr<Command> makeExtendForCommand(QTime duration);
  /// @brief Command that holds the retracted state for a duration, then
  /// finishes and stays retracted.
  std::unique_ptr<Command> makeRetractForCommand(QTime duration);

protected:
  void applyState(const bool& extended) override;

private:
  static Actuator makeActuator(std::shared_ptr<device::Pneumatic> pneumatic);
  static Actuator makeActuator(
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics);

  Actuator m_actuator;
  bool m_inverted;
};

}  // namespace mechanism
}  // namespace mclib
