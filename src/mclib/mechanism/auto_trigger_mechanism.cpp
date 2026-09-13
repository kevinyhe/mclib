// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/mechanism/auto_trigger_mechanism.hpp"

#include "mclib/time.hpp"

#include <utility>

namespace mclib {
namespace mechanism {

AutoTriggerMechanism::AutoTriggerMechanism(Condition trigger_condition,
                                           Action action,
                                           AutoTriggerConfig config)
    : StateMechanism<bool>(config.enabled_on_construct),
      m_config(config),
      m_trigger_condition(std::move(trigger_condition)),
      m_action(std::move(action)) {}

void AutoTriggerMechanism::setTriggerCondition(Condition trigger_condition) {
  m_trigger_condition = std::move(trigger_condition);
  // Drop the debounce window: time the old condition spent true says nothing
  // about the new one. Whether a fire is pending is left alone, so swapping the
  // condition neither creates nor cancels one.
  clearEdgeTracking();
}

void AutoTriggerMechanism::setAction(Action action) {
  m_action = std::move(action);
}

void AutoTriggerMechanism::setRearmCondition(Condition rearm_condition) {
  m_rearm_condition = std::move(rearm_condition);
}

void AutoTriggerMechanism::setArmed(bool armed) {
  if (armed) {
    // Clear here rather than relying on onStateChanged, which setState() only
    // calls on a real change: re-arming something already armed must still
    // release the latch.
    m_latched = false;
    m_edge_ready = true;
    clearEdgeTracking();
  }
  setState(armed);
}

bool AutoTriggerMechanism::isArmed() const {
  return getState();
}

void AutoTriggerMechanism::disarmUntilReset() {
  m_latched = true;
  // Also drop the edge: once the latch clears, a fresh false->true transition
  // is required. Without this a manual override made while the condition is
  // true would be reapplied the instant the re-arm condition went true.
  m_edge_ready = false;
}

void AutoTriggerMechanism::rearm() {
  m_latched = false;
}

bool AutoTriggerMechanism::isLatched() const {
  return m_latched;
}

bool AutoTriggerMechanism::hasFired() const {
  return m_fire_count > 0;
}

std::uint32_t AutoTriggerMechanism::fireCount() const {
  return m_fire_count;
}

void AutoTriggerMechanism::reset() {
  m_latched = false;
  m_fire_count = 0;
  // m_edge_ready is deliberately untouched. Clearing it here would hand a
  // still-true condition a fresh firing opportunity, so a routine that zeroes
  // the counters would reapply the action the driver just undid by hand.
  clearEdgeTracking();
}

bool AutoTriggerMechanism::isConditionMet() const {
  return m_condition_met;
}

const AutoTriggerConfig& AutoTriggerMechanism::getConfig() const {
  return m_config;
}

void AutoTriggerMechanism::poll() {
  if (!isArmed()) {
    return;
  }

  const double now = nowMs();
  if (m_polled && now == m_last_poll_ms) {
    // Already polled this millisecond (periodic() plus a command's execute()).
    // Polling twice would read the sensor twice for one tick and could hand the
    // debounce window a reading it never actually saw separately.
    return;
  }
  m_polled = true;
  m_last_poll_ms = now;

  // A missing condition reads false. It cannot fire, and it keeps the latch
  // clearing rather than freezing.
  const bool raw = m_trigger_condition ? m_trigger_condition() : false;

  if (raw) {
    if (!m_raw_held) {
      m_raw_held = true;
      m_raw_since_ms = now;
    }
    m_condition_met = now - m_raw_since_ms >= m_config.debounce_ms;
  } else {
    // Any false reading restarts the debounce window, so a sensor that
    // flickers never accumulates enough continuous time to fire.
    m_raw_held = false;
    m_raw_since_ms = 0.0;
    m_condition_met = false;
  }

  if (m_latched) {
    if (m_rearm_condition) {
      if (m_rearm_condition()) {
        m_latched = false;
      }
    } else if (!m_condition_met) {
      // No re-arm condition supplied: the trigger going away is the re-arm.
      // This is why a validity gate must report "nothing there" as false. If it
      // swallows the absent reading the latch never clears.
      m_latched = false;
    }
  }

  if (!m_latched && !m_condition_met) {
    // The condition has been seen false while nothing was blocking a fire, so
    // the next true is a real edge. Deliberately not tracked while latched: a
    // condition that cycles false->true behind a set latch must not queue up a
    // fire that goes off the moment the re-arm condition is satisfied.
    m_edge_ready = true;
  }

  if (m_condition_met && m_edge_ready && !m_latched) {
    fire();
  }
}

void AutoTriggerMechanism::fireNow() {
  fire();
}

void AutoTriggerMechanism::applyState(const bool& armed) {
  if (!armed) {
    // Disarmed: do not touch the sensor at all.
    return;
  }
  poll();
}

void AutoTriggerMechanism::onStateChanged(const bool& armed) {
  // Covers setState(true) called directly instead of setArmed(true).
  if (armed) {
    m_latched = false;
    m_edge_ready = true;
    clearEdgeTracking();
  }
}

void AutoTriggerMechanism::fire() {
  // Bookkeeping first, so an action that reads fireCount() or isLatched(), or
  // that calls disarmUntilReset() on itself, sees the post-fire state.
  m_latched = true;
  m_edge_ready = false;
  ++m_fire_count;
  ++m_fire_serial;

  if (m_action) {
    m_action();
  }

  if (m_config.fire_once) {
    // Straight to setState: setArmed(false) is the same here, but going through
    // the state directly keeps the "arming clears the latch" rule in one place.
    setState(false);
  }
}

void AutoTriggerMechanism::clearEdgeTracking() {
  m_condition_met = false;
  m_raw_held = false;
  m_raw_since_ms = 0.0;
  m_polled = false;
  m_last_poll_ms = 0.0;
}

double AutoTriggerMechanism::nowMs() {
  return static_cast<double>(mclib::time::millis());
}

std::unique_ptr<Command> AutoTriggerMechanism::makeArmCommand() {
  return runOnce([this]() { setArmed(true); });
}

std::unique_ptr<Command> AutoTriggerMechanism::makeDisarmCommand() {
  return runOnce([this]() { setArmed(false); });
}

std::unique_ptr<Command> AutoTriggerMechanism::makeWaitForTriggerCommand(
    double timeout_ms) {
  auto start_ms = std::make_shared<double>(0.0);
  auto start_count = std::make_shared<std::uint32_t>(0);
  auto was_armed = std::make_shared<bool>(false);

  return std::make_unique<FunctionalCommand>(
      [this, start_ms, start_count, was_armed]() {
        *start_ms = nowMs();
        *start_count = m_fire_serial;
        *was_armed = isArmed();
        // Only arm what is not already armed. setArmed(true) clears m_latched
        // and m_edge_ready, so calling it on an already-armed mechanism
        // destroys a latch that disarmUntilReset() deliberately set after a
        // driver override, and the still-true condition fires again with no
        // false->true edge. Same reason the end lambda below refuses to call
        // setArmed(true).
        if (!*was_armed) {
          setArmed(true);
        }
      },
      // Poll here as well as from periodic(), so the command works whether or
      // not the mechanism is registered. A second poll in the same millisecond
      // is ignored.
      [this]() { poll(); },
      [this, was_armed](bool) {
        // Only undo the arming this command did. If the mechanism is no longer
        // armed, something else disarmed it while the command ran (a setArmed
        // call, or fire_once) and that decision wins. If it was already armed
        // on entry there is nothing to restore, and calling setArmed(true)
        // again would wrongly clear the latch the fire just set. A mid-command
        // setArmed(true) cannot be told apart from this command's own arming
        // and is undone with it.
        if (!*was_armed && isArmed()) {
          setArmed(false);
        }
      },
      [this, start_ms, start_count, timeout_ms]() {
        // Compare the monotonic serial, not m_fire_count: a reset() while
        // this command runs zeroes the count and would otherwise read as a fire.
        const bool fired = m_fire_serial != *start_count;
        const bool timed_out =
            timeout_ms > 0.0 && nowMs() - *start_ms >= timeout_ms;
        return fired || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

}  // namespace mechanism
}  // namespace mclib
