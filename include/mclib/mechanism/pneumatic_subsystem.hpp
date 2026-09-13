// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief A group of solenoids driven as one two-state mechanism.
 *
 * The state of a PneumaticSubsystem is always the *logical* state: true means
 * "extended", false means "retracted". Solenoid wiring polarity is a separate
 * concept and lives entirely in device::Pneumatic, which maps logical to raw
 * as `raw = (logical == extended_state)`. Nothing above the device layer sees
 * the raw value, so isExtended() means the same thing for a normal and for an
 * inverted solenoid.
 */
class PneumaticSubsystem : public StateMechanism<bool> {
public:
  /**
   * @brief Construct from ADI ports.
   *
   * @param adi_ports The 3 wire ports the solenoids are connected to
   * @param initial_extended The logical state the mechanism starts in
   * (true = extended). This seeds both the cached state and the hardware.
   * @param extended_state Wiring polarity: whether a solenoid value of true
   * extends the cylinder. This never reaches the cached state.
   */
  PneumaticSubsystem(std::initializer_list<char> adi_ports,
                     bool initial_extended = false,
                     bool extended_state = true);

  /**
   * @brief Construct from already-built pneumatics.
   *
   * @param pneumatics The solenoids to drive as one group
   * @param initial_extended The logical state the mechanism starts in. The
   * hardware is driven to match it immediately, so the cache never lies.
   */
  PneumaticSubsystem(std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
                     bool initial_extended = false);

  /** @brief Set the logical state (true = extended). */
  void setExtended(bool extended);
  /** @brief The cached logical state (true = extended). */
  bool isExtended() const;
  /** @brief Shorthand for setExtended(true). */
  void extend();
  /** @brief Shorthand for setExtended(false). */
  void retract();
  /** @brief Flip the logical state. */
  void toggle();

  /** @brief Number of solenoids in the group. */
  std::size_t count() const;
  /**
   * @brief The underlying group, for direct device-layer access.
   *
   * Writing through this reference does not update the cached state, and
   * periodic() overwrites it on the next tick. Use setExtended() to move the
   * cylinder; use this only for diagnostics or one-off device calls.
   */
  device::PneumaticGroup& group();
  /**
   * @brief Per-solenoid values as reported by the device layer.
   *
   * These are logical values (true = extended), not solenoid pin levels; the
   * device layer never exposes the pin level. Useful for spotting a solenoid
   * that has drifted out of sync with the rest of the group.
   */
  std::vector<bool> rawValues() const;

  /** @brief Set the state once, then finish. */
  std::unique_ptr<Command> makeSetCommand(bool extended);
  /** @brief Extend once, then finish. */
  std::unique_ptr<Command> makeExtendCommand();
  /** @brief Retract once, then finish. */
  std::unique_ptr<Command> makeRetractCommand();
  /** @brief Flip the state once, then finish. */
  std::unique_ptr<Command> makeToggleCommand();
  /**
   * @brief Force a state on every tick. Never finishes.
   *
   * Do NOT use this as the default command: it would drag the mechanism back
   * as soon as any one-shot command releases the requirement. Use
   * Subsystem::idleCommand() as the default and this only when something must
   * be pinned to a state for as long as the command runs.
   */
  std::unique_ptr<Command> makeHoldCommand(bool extended);
  /** @brief Extend, hold for `duration`, then finish. */
  std::unique_ptr<Command> makeExtendForCommand(QTime duration);

protected:
  void applyState(const bool& extended) override;

private:
  static std::vector<std::shared_ptr<device::Pneumatic>> makePneumatics(
      std::initializer_list<char> adi_ports,
      bool initial_extended,
      bool extended_state);

  std::vector<std::shared_ptr<device::Pneumatic>> m_pneumatics;
  device::PneumaticGroup m_group;
};

}  // namespace mechanism
}  // namespace mclib
