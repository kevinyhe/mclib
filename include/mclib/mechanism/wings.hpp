// mclib
#pragma once

#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Which wing an operation applies to.
 */
enum class WingSide {
  Left,
  Right,
  Both,
};

/**
 * @brief Logical extension state of both wings.
 *
 * `true` always means "extended", regardless of how either solenoid is
 * plumbed. The raw solenoid value lives in device::Pneumatic.
 */
struct WingsState {
  bool left = false;
  bool right = false;

  bool operator==(const WingsState& other) const {
    return left == other.left && right == other.right;
  }

  bool operator!=(const WingsState& other) const {
    return !(*this == other);
  }
};

struct WingsConfig {
  char left_port = 'A';
  char right_port = 'B';
  bool left_extended_state = true;
  bool right_extended_state = true;
  bool default_state = false;
};

/**
 * @brief Two independently actuated wings / doinkers.
 *
 * Each side has its own solenoid and its own polarity, so the two can be
 * plumbed opposite to each other. The state this mechanism caches is the
 * logical "is extended" value for each side, never the raw solenoid value.
 */
class Wings : public StateMechanism<WingsState> {
public:
  explicit Wings(const WingsConfig& config);
  Wings(char left_port, char right_port, bool default_state = false);

  /**
   * @brief Set one side (or both) to a logical extended value.
   */
  void set(WingSide side, bool extended);
  void extend(WingSide side);
  void retract(WingSide side);

  /**
   * @brief Flip one side, or flip both sides independently.
   *
   * With WingSide::Both each side is inverted on its own, so a
   * left-extended/right-retracted pair becomes left-retracted/right-extended.
   */
  void toggle(WingSide side);

  /**
   * @brief Whether the given side is extended.
   *
   * WingSide::Both returns true only when BOTH wings are extended.
   */
  bool isExtended(WingSide side) const;

  void extendBoth();
  void retractBoth();

  /**
   * @brief The four commands below terminate after a single scheduler pass.
   *
   * They release the subsystem immediately, so the scheduler reschedules the
   * default command on the same tick. Register `idleCommand()` as the default
   * for a Wings subsystem, or a retract default will undo every extend.
   */
  std::unique_ptr<Command> makeSetCommand(WingSide side, bool extended);
  std::unique_ptr<Command> makeExtendCommand(WingSide side);
  std::unique_ptr<Command> makeRetractCommand(WingSide side);
  std::unique_ptr<Command> makeToggleCommand(WingSide side);

  /**
   * @brief Extend a side, hold it for `duration`, then finish.
   *
   * The command does not retract on end; schedule a retract command after it
   * if that is what you want.
   */
  std::unique_ptr<Command> makeExtendForCommand(WingSide side, QTime duration);

protected:
  void applyState(const WingsState& state) override;

private:
  WingsState withSide(WingSide side, bool extended) const;

  device::Pneumatic m_left;
  device::Pneumatic m_right;
};

}  // namespace mechanism
}  // namespace mclib
