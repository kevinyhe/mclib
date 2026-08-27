// mclib
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <type_traits>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Settings for a ToggleGroupMechanism.
 */
struct ToggleGroupMechanismConfig {
  /// Logical state every channel starts in: true means "extended". Ignored for
  /// any channel covered by initial_states.
  bool initial_extended = false;
  /// Per channel starting states. Entries past the end of this vector fall back
  /// to initial_extended, and entries past the channel count are ignored.
  std::vector<bool> initial_states = {};
  /// Per channel inversion. A true entry means the actuator for that channel is
  /// driven with the opposite of the logical state, so an actuator plumbed
  /// backwards still reports "extended" correctly. Entries past the end of this
  /// vector are treated as false.
  ///
  /// Use this for raw std::function<void(bool)> actuators only.
  /// device::Pneumatic already takes a logical value and does its own inversion
  /// through its extended_state flag, so inverting a channel that drives a
  /// Pneumatic inverts twice and leaves Pneumatic::get_value() reporting the
  /// opposite of get(). For a backwards pneumatic write
  /// Pneumatic(port, false, false) and leave that inversion entry false.
  std::vector<bool> inverted = {};
  /// When true every actuator is written once from the constructor so the
  /// hardware matches the initial logical states before the scheduler runs.
  bool apply_on_construct = true;
};

/**
 * @brief N independently addressable two-state channels in one subsystem.
 *
 * ToggleMechanism models one shared boolean across a group of actuators: every
 * actuator moves together. ToggleGroupMechanism instead gives each actuator its
 * own boolean while keeping them inside a single Subsystem, so they share one
 * scheduler requirement and cannot be driven by two commands at the same time.
 *
 * Channels are addressed by index. Any scoped enum works as an index too: the
 * enum overloads static_cast the enumerator to std::size_t, so
 * `set(Channel::Left, true)` and `set(0, true)` are the same call. The class
 * itself names nothing, which keeps it usable across seasons.
 *
 * Out of range indices are ignored by the mutators and read back as false, so a
 * bad index cannot corrupt the state or crash the scheduler.
 *
 * The state is a std::vector<bool> of logical values, one per channel. Each
 * channel's actuator receives that value, negated if that channel is marked
 * inverted. Actuators are std::function<void(bool)>, so the same type covers
 * solenoids, motor driven two-position mechanisms, and test mocks.
 *
 * Every actuator is written each scheduler tick from applyState(), which only
 * happens once the mechanism is registered. set(), toggle(), setAll() and
 * toggleAll() also write right away, whether or not the value changed, so they
 * work without the scheduler and can resync hardware that drifted. Calling the
 * inherited setState() directly does not write anything until the next
 * periodic().
 *
 * All command factories terminate after one tick, which releases the
 * requirement and lets CommandScheduler::run() immediately reschedule the
 * default command. The default command MUST therefore be idleCommand(). A
 * "retract everything" default would undo every set on the very next tick.
 *
 * Copy and move are deleted. The command factories capture `this` and the
 * CommandScheduler holds this address as a requirement, so relocating the
 * object, for example by growing a std::vector that holds one by value, would
 * leave those captures writing to freed memory and to real hardware.
 */
class ToggleGroupMechanism : public StateMechanism<std::vector<bool>> {
public:
  using Actuator = std::function<void(bool)>;

  /**
   * @brief Construct from one actuator per channel.
   *
   * @param actuators Called with the raw value for their channel whenever the
   * state is applied. Empty entries are ignored. The channel count is the size
   * of this vector.
   * @param config Initial states, per channel inversion, construct-time apply.
   */
  explicit ToggleGroupMechanism(std::vector<Actuator> actuators,
                                ToggleGroupMechanismConfig config = {});

  /**
   * @brief Construct from one pneumatic per channel.
   *
   * @param pneumatics The pneumatics to drive, one per channel. Null entries
   * are ignored. The channel count is the size of this vector.
   * @param config Initial states, per channel inversion, construct-time apply.
   * Leave the inversion entries false here and pass extended_state = false to
   * the Pneumatic instead.
   */
  explicit ToggleGroupMechanism(
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
      ToggleGroupMechanismConfig config = {});

  ToggleGroupMechanism(const ToggleGroupMechanism&) = delete;
  ToggleGroupMechanism& operator=(const ToggleGroupMechanism&) = delete;
  ToggleGroupMechanism(ToggleGroupMechanism&&) = delete;
  ToggleGroupMechanism& operator=(ToggleGroupMechanism&&) = delete;

  /// @brief Set one channel and write its actuator. True means extended.
  void set(std::size_t index, bool extended);
  /// @brief The logical state of one channel, false if the index is invalid.
  bool get(std::size_t index) const;
  /// @brief Flip one channel to the opposite of its current logical state.
  void toggle(std::size_t index);
  /// @brief Set every channel to the same logical state.
  void setAll(bool extended);
  /// @brief Flip every channel independently.
  void toggleAll();
  /// @brief Write every actuator with the current logical states.
  void apply();
  /// @brief Whether every channel is extended. True when there are no channels.
  bool allSet() const;
  /// @brief Whether at least one channel is extended.
  bool anySet() const;
  /// @brief How many channels are extended.
  std::size_t setCount() const;
  /// @brief The number of channels.
  std::size_t count() const;
  /// @brief Whether a channel drives its actuator with the inverse of the
  /// logical state. False if the index is invalid.
  bool isInverted(std::size_t index) const;
  /// @brief The raw value a channel's actuator is given for a logical state.
  bool rawValue(std::size_t index, bool extended) const;

  /// @brief Enum friendly overload of set().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  void set(ChannelT channel, bool extended) {
    set(toIndex(channel), extended);
  }

  /// @brief Enum friendly overload of get().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  bool get(ChannelT channel) const {
    return get(toIndex(channel));
  }

  /// @brief Enum friendly overload of toggle().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  void toggle(ChannelT channel) {
    toggle(toIndex(channel));
  }

  /// @brief Enum friendly overload of isInverted().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  bool isInverted(ChannelT channel) const {
    return isInverted(toIndex(channel));
  }

  /// @brief Command that sets one channel once and finishes immediately.
  std::unique_ptr<Command> makeSetCommand(std::size_t index, bool extended);
  /// @brief Command that toggles one channel once and finishes immediately.
  std::unique_ptr<Command> makeToggleCommand(std::size_t index);
  /// @brief Command that sets every channel once and finishes immediately.
  std::unique_ptr<Command> makeSetAllCommand(bool extended);
  /// @brief Command that toggles every channel once and finishes immediately.
  std::unique_ptr<Command> makeToggleAllCommand();

  /// @brief Command that holds one channel in a state for a duration, then
  /// finishes and leaves it there. It does not restore the old state, and it
  /// does not touch the other channels.
  std::unique_ptr<Command> makeSetForCommand(std::size_t index, bool extended,
                                             QTime duration);
  /// @brief Command that holds every channel in a state for a duration, then
  /// finishes and leaves them there. It does not restore the old states.
  std::unique_ptr<Command> makeSetAllForCommand(bool extended, QTime duration);

  /// @brief Enum friendly overload of makeSetCommand().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  std::unique_ptr<Command> makeSetCommand(ChannelT channel, bool extended) {
    return makeSetCommand(toIndex(channel), extended);
  }

  /// @brief Enum friendly overload of makeToggleCommand().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  std::unique_ptr<Command> makeToggleCommand(ChannelT channel) {
    return makeToggleCommand(toIndex(channel));
  }

  /// @brief Enum friendly overload of makeSetForCommand().
  template <typename ChannelT,
            typename = std::enable_if_t<std::is_enum_v<ChannelT>>>
  std::unique_ptr<Command> makeSetForCommand(ChannelT channel, bool extended,
                                             QTime duration) {
    return makeSetForCommand(toIndex(channel), extended, duration);
  }

protected:
  void applyState(const std::vector<bool>& states) override;

private:
  template <typename ChannelT>
  static std::size_t toIndex(ChannelT channel) {
    return static_cast<std::size_t>(
        static_cast<std::underlying_type_t<ChannelT>>(channel));
  }

  static std::vector<Actuator> makeActuators(
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics);
  static std::vector<bool> makeInitialStates(
      std::size_t channel_count, const ToggleGroupMechanismConfig& config);
  static std::vector<bool> makeInverted(
      std::size_t channel_count, const ToggleGroupMechanismConfig& config);

  std::vector<Actuator> m_actuators;
  std::vector<bool> m_inverted;
};

}  // namespace mechanism
}  // namespace mclib
