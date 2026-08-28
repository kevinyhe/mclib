// mclib
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/mechanism/mechanism.hpp"

#include <cstdint>
#include <functional>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Tuning values for an AutoTriggerMechanism.
 */
struct AutoTriggerConfig {
  /// The trigger condition must read true continuously for this long (ms)
  /// before the action fires. 0 fires on the first tick the condition is true.
  /// Any tick that reads false restarts the window, so a noisy sensor that
  /// flickers false never accumulates time.
  double debounce_ms = 0.0;
  /// Whether the mechanism starts armed. A disarmed mechanism never calls the
  /// trigger condition, so a sensor read costs nothing while it is off.
  bool enabled_on_construct = true;
  /// When true the mechanism disarms itself after the first fire, so the action
  /// runs exactly once per setArmed(true). When false it keeps watching and
  /// fires again on every later false->true edge that passes the latch.
  bool fire_once = false;
};

/**
 * @brief Watches a sensor condition and fires an action on the false->true
 *        edge, with a latch so a manual override is not undone on the next tick.
 *
 * This is the generic form of "auto-clamp when a goal is in range",
 * "auto-index when an object is detected", "auto-retract when a limit is hit".
 * Nothing here knows what the sensor is or what the action does: the condition
 * is a std::function<bool()> and the action is a std::function<void()>, so the
 * same type covers a distance sensor, an optical sensor, a limit switch, a
 * motor current reading, or a test fake.
 *
 * ### Edge-triggered, not level-triggered
 *
 * The action fires when the debounced condition goes from false to true. It
 * does not fire again while the condition stays true. Holding a goal in the
 * clamp does not re-fire the clamp every 10 ms.
 *
 * ### The latch
 *
 * Firing sets a latch. While the latch is set, no fire can happen. The latch
 * clears when:
 *  - the re-arm condition returns true, if one was supplied, or
 *  - the debounced condition reads false, if no re-arm condition was supplied.
 *
 * Clearing the latch is not enough to fire again on its own: a fresh false->true
 * edge is still required. So a re-arm condition that goes true while the trigger
 * is still true re-arms the mechanism without immediately re-firing it.
 *
 * disarmUntilReset() sets the latch by hand. That is what a driver's manual
 * override should call: the override takes effect and survives, because the
 * mechanism will not reapply the action until the latch clears and a new edge
 * arrives. rearm() clears the latch by hand.
 *
 * ### Validity gates belong in the condition, and must not freeze the latch
 *
 * If the condition wraps a sensor with a confidence or range check, an
 * "absent / no reading" result must return **false**, not "hold the previous
 * value". With no re-arm condition the latch clears only on a false reading, so
 * a gate that swallows the "nothing there" case latches the mechanism forever
 * and it never fires again. A gate that cannot tell "no object" from "bad read"
 * needs an explicit re-arm condition instead.
 *
 * For pros::Distance specifically: confidence is pinned at 10 for anything
 * under 200 mm (pros/distance.h), so a confidence floor above 10 discards every
 * reading a close-range mechanism would ever see. 0 mm is a real reading, for a
 * target inside the sensor's ~20 mm minimum range, so it must not be treated as
 * an error. The error value is PROS_ERR, which is INT32_MAX, not a negative
 * number: a `reading < 0` guard never fires and lets INT32_MAX through as a
 * huge distance. Compare against PROS_ERR explicitly.
 *
 * ### Stepping
 *
 * The condition is polled from periodic(), so the mechanism must be registered
 * with CommandScheduler::registerSubsystem (or a MechanismManager) to run on its
 * own. makeWaitForTriggerCommand() also polls from execute(), so it works
 * unregistered; a second poll in the same millisecond is ignored so the action
 * cannot fire twice for one edge.
 *
 * The state of the underlying StateMechanism<bool> is "armed". That is the one
 * piece of state a caller sets, reads, and drives from a command, so it gets the
 * state machinery (setState, makeStateOnceCommand, onStateChanged) for free. The
 * latch, the debounce timer, and the fire count are derived bookkeeping and are
 * plain members. Note that armed is separate from Subsystem::setEnabled: enabled
 * gates periodic() for the whole scheduler, armed is this mechanism's own switch.
 */
class AutoTriggerMechanism : public StateMechanism<bool> {
public:
  using Condition = std::function<bool()>;
  using Action = std::function<void()>;

  /**
   * @brief Construct a watcher.
   *
   * @param trigger_condition Polled every tick while armed. A null function
   *                          reads as false, so the mechanism never fires.
   * @param action Called once per accepted edge. A null action still counts as
   *               a fire (the latch is set and fireCount() increments), which
   *               keeps the state machine testable without side effects.
   * @param config Debounce, initial armed state, one-shot behaviour.
   */
  AutoTriggerMechanism(Condition trigger_condition, Action action,
                       AutoTriggerConfig config = {});

  /// Capturing `this` in the injected std::functions means an instance cannot
  /// be relocated without dangling. Copy and move are deleted, hold it in place.
  AutoTriggerMechanism(const AutoTriggerMechanism&) = delete;
  AutoTriggerMechanism& operator=(const AutoTriggerMechanism&) = delete;
  AutoTriggerMechanism(AutoTriggerMechanism&&) = delete;
  AutoTriggerMechanism& operator=(AutoTriggerMechanism&&) = delete;

  /// @brief Replace the trigger condition. Clears the debounce window and the
  /// edge history, so the new condition needs its own false->true edge.
  void setTriggerCondition(Condition trigger_condition);
  /// @brief Replace the action fired on an accepted edge.
  void setAction(Action action);
  /**
   * @brief Supply the condition that clears the latch after a fire.
   *
   * Without one the latch clears when the trigger condition reads false. With
   * one the trigger going false is no longer enough; only this returning true
   * clears the latch. Pass an empty function to go back to the default.
   */
  void setRearmCondition(Condition rearm_condition);

  /// @brief Arm or disarm the watcher. Arming clears the latch and the edge
  /// history, so the next false->true edge fires. Disarming stops polling.
  void setArmed(bool armed);
  /// @brief Whether the watcher is polling its condition.
  [[nodiscard]] bool isArmed() const;

  /// @brief Suppress the next fire. The manual-override entry point: call it
  /// after undoing the action by hand so the next tick does not reapply it.
  void disarmUntilReset();
  /// @brief Clear the latch by hand. A fresh false->true edge is still needed
  /// before the action fires again.
  void rearm();
  /// @brief Whether the latch is currently blocking a fire.
  [[nodiscard]] bool isLatched() const;

  /// @brief Whether the action has fired at least once since the last reset().
  [[nodiscard]] bool hasFired() const;
  /// @brief How many times the action has fired since the last reset().
  [[nodiscard]] std::uint32_t fireCount() const;
  /// @brief Clear the latch, the debounce window and the fire count. Does not
  /// change the armed state and does not call the action, now or on the next
  /// tick: the edge history survives, so a mechanism that has already fired
  /// still needs a fresh false->true edge. Use setArmed(true) to also forgive
  /// the edge and let a currently-true condition fire again.
  void reset();

  /// @brief The debounced value of the trigger condition as of the last poll.
  /// False until the condition has held true for AutoTriggerConfig::debounce_ms.
  [[nodiscard]] bool isConditionMet() const;

  /// @brief Poll the condition once and fire if the edge is accepted. Called
  /// from periodic(); public so a caller can step it without the scheduler.
  /// Repeated calls within the same millisecond do nothing.
  void poll();

  /// @brief Fire the action now, ignoring the condition and the latch. Sets the
  /// latch and increments fireCount(), exactly as an automatic fire does.
  void fireNow();

  /// @brief The config this mechanism was built with.
  [[nodiscard]] const AutoTriggerConfig& getConfig() const;

  /// @brief Command that arms the watcher once and finishes immediately.
  std::unique_ptr<Command> makeArmCommand();
  /// @brief Command that disarms the watcher once and finishes immediately.
  std::unique_ptr<Command> makeDisarmCommand();

  /**
   * @brief Command that arms the watcher and finishes when the action fires.
   *
   * It only arms a mechanism that was disarmed. One that was already armed is
   * left exactly as it was, latch and edge included, so this command waits for
   * the next real fire instead of manufacturing one: arming clears the latch,
   * and clearing a latch that disarmUntilReset() set would reapply the action
   * the driver just overrode, with no false->true edge behind it.
   *
   * If it found the mechanism disarmed, it disarms it again on the way out,
   * but only if the mechanism is still armed at that point. Anything that
   * disarmed it mid-command (a setArmed(false), or AutoTriggerConfig::fire_once)
   * wins over the restore. A mid-command setArmed(true) is indistinguishable
   * from the command's own arming and is undone with it, so drive the armed
   * state from one place while this command is running.
   *
   * @param timeout_ms Give up after this long. <= 0 waits forever.
   */
  std::unique_ptr<Command> makeWaitForTriggerCommand(double timeout_ms = 0.0);

protected:
  /// Polls the condition. The state is "armed", so a false state means the
  /// mechanism sits idle and never touches the sensor.
  void applyState(const bool& armed) override;
  void onStateChanged(const bool& armed) override;

private:
  void fire();
  void clearEdgeTracking();
  static double nowMs();

  AutoTriggerConfig m_config;
  Condition m_trigger_condition;
  Condition m_rearm_condition;
  Action m_action;

  /// Set after a fire, blocks the next one until the re-arm rule is satisfied.
  bool m_latched = false;
  /// True once the debounced condition has been seen false while the latch was
  /// clear. A fire needs this, which is what makes the mechanism edge- not
  /// level-triggered even when the latch clears while the condition is true.
  bool m_edge_ready = true;
  /// The debounced condition value from the last poll.
  bool m_condition_met = false;
  /// Whether the raw condition is in an unbroken run of true readings.
  bool m_raw_held = false;
  /// When the current unbroken run of true readings started (ms).
  double m_raw_since_ms = 0.0;
  /// Same-millisecond guard for poll().
  double m_last_poll_ms = 0.0;
  bool m_polled = false;

  std::uint32_t m_fire_count = 0;
  /// Monotonic fire counter. Unlike m_fire_count it is never cleared, so a
  /// reset() while makeWaitForTriggerCommand is running cannot make the command
  /// finish as though the action had fired.
  std::uint32_t m_fire_serial = 0;
};

}  // namespace mechanism
}  // namespace mclib
