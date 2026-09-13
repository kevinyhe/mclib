// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/subsystem.h"
#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Settings for a DiscreteActuatorMechanism.
 */
struct DiscreteActuatorMechanismConfig {
  /// Per actuator polarity. Actuator `i` is written with the negation of the
  /// table value when `inverted[i]` is true. Indices past the end of this
  /// vector count as false, so an empty vector means "invert nothing".
  ///
  /// This is for RAW `std::function<void(bool)>` actuators only.
  /// device::Pneumatic already takes a logical value and inverts internally
  /// through its extended_state flag, so inverting here as well inverts twice
  /// and leaves Pneumatic::get_value() disagreeing with the cached state. For a
  /// Pneumatic, pass extended_state = false to the device instead.
  std::vector<bool> inverted = {};
  /// When true the actuators are written once from the constructor so the
  /// hardware matches the initial state before the scheduler runs. Skipped when
  /// the initial state is not decodable.
  bool apply_on_construct = true;
};

/**
 * @brief A mechanism whose state is one of N named combinations of on/off
 * actuators.
 *
 * ToggleMechanism drives one boolean actuator. MultiPositionMechanism maps a
 * state to one number. Neither covers the common case of several solenoids
 * whose combined pattern encodes a handful of named positions: two cylinders
 * give four patterns, three give eight. This class owns that decode table and
 * pushes the pattern of the current state into a bank of injected actuators
 * every scheduler tick.
 *
 * The table is the only source of truth. There is no if/else chain anywhere:
 * to add a position, add a row.
 *
 * ```cpp
 * enum class Angle { Low, Mid, High };
 *
 * DiscreteActuatorMechanism<Angle> tilt(
 *     Angle::Low,
 *     {{Angle::Low,  {false, false}},
 *      {Angle::Mid,  {true,  false}},
 *      {Angle::High, {true,  true}}},
 *     std::vector<std::shared_ptr<device::Pneumatic>>{front, back});
 * ```
 *
 * The table is a `std::vector<std::pair<StateT, std::vector<bool>>>` rather
 * than a fixed-width array because the actuator count is a runtime property of
 * the wiring, not a compile-time property of the state type, and because the
 * vector keeps `StateT` free of any ordering requirement: rows are found by
 * `operator==`, so a scoped enum works with no `operator<` and no hashing.
 * The table is ordered, and that order is what next(), previous() and cycle()
 * step through. It is fixed at construction.
 *
 * Stepping matches MultiPositionMechanism:
 * - next() and previous() clamp. At the last row next() does nothing; at the
 *   first row previous() does nothing.
 * - cycle() wraps. From the last row it goes back to the first.
 *
 * Safety. applyState() runs every tick, so every degenerate case has to be
 * cheap and harmless:
 * - Rows whose combination width does not match the actuator count are dropped
 *   at construction. Every row that survives can be applied whole, so the
 *   mechanism can never drive a partial or half decoded combination. Dropping
 *   is silent, and a mistyped row therefore turns into a button that does
 *   nothing on the robot: droppedRowCount() reports how many rows were thrown
 *   away, so check it once at startup if a position looks dead.
 * - A state that is not in the table is undecodable. applyState() writes
 *   nothing for it: the actuators keep the last combination that was written.
 *   Holding the last valid pattern is safer than inventing a default one, since
 *   a made up pattern moves real hardware.
 * - setDiscreteState() refuses to move to an undecodable state, so an
 *   undecodable state can never be cached by a command. The inherited
 *   setState() and the inherited state command factories are hidden behind
 *   private using declarations for the same reason: they do not consult the
 *   table and would let a command latch a state that can never be applied.
 * - Only the initial state passed to the constructor can be undecodable. In
 *   that case nothing is written until a decodable state is set, and the first
 *   next()/previous()/cycle() resyncs to row 0.
 * - An empty table makes every state undecodable, so nothing is ever written.
 *
 * The table is searched front to back and the first match wins, so a state
 * listed twice makes the later row unreachable. List each state once.
 *
 * All command factories terminate after one tick, which releases the subsystem
 * requirement and lets the scheduler immediately reschedule the default
 * command. The default command MUST therefore be idleCommand(): a default that
 * commands a state would drag the mechanism back one tick after every button
 * press. periodic() keeps re-applying the cached state, so the actuators stay
 * where the last command put them.
 */
template <typename StateT>
class DiscreteActuatorMechanism : public StateMechanism<StateT> {
public:
  using Actuator = std::function<void(bool)>;
  using Combination = std::vector<bool>;
  using Entry = std::pair<StateT, Combination>;
  using Table = std::vector<Entry>;

  /// Returned by indexOf() and currentIndex() for an undecodable state.
  static constexpr std::size_t kNoState = static_cast<std::size_t>(-1);

  /**
   * @brief Construct from a bank of raw actuators.
   *
   * @param initial_state State to start in. An undecodable state is allowed and
   * simply writes nothing until a decodable state is set.
   * @param table Decode table. Rows whose width is not `actuators.size()` are
   * dropped.
   * @param actuators One callable per actuator, in the same column order the
   * table rows use. Null entries keep their column, so indices stay aligned,
   * and are never called. A bare `{}` here is ambiguous against the pneumatic
   * overload, so spell a zero column bank `std::vector<Actuator>{}`.
   * @param config Polarity and construct time apply.
   */
  DiscreteActuatorMechanism(StateT initial_state,
                            Table table,
                            std::vector<Actuator> actuators,
                            DiscreteActuatorMechanismConfig config = {})
      : StateMechanism<StateT>(std::move(initial_state)),
        m_actuators(std::move(actuators)),
        m_inverted(std::move(config.inverted)) {
    const std::size_t given = table.size();
    m_table = pruneTable(std::move(table), m_actuators.size());
    m_dropped_rows = given - m_table.size();
    if (config.apply_on_construct) {
      apply();
    }
  }

  /// @brief Construct from a bank of raw actuators with a braced table.
  DiscreteActuatorMechanism(StateT initial_state,
                            std::initializer_list<Entry> table,
                            std::vector<Actuator> actuators,
                            DiscreteActuatorMechanismConfig config = {})
      : DiscreteActuatorMechanism(std::move(initial_state),
                                  Table(table),
                                  std::move(actuators),
                                  std::move(config)) {}

  /**
   * @brief Construct from a bank of pneumatics.
   *
   * The table values are logical: true means extended. device::Pneumatic does
   * its own wiring inversion, so leave
   * DiscreteActuatorMechanismConfig::inverted empty here and pass
   * extended_state = false to any device that is wired backwards.
   *
   * @param initial_state State to start in.
   * @param table Decode table. Rows whose width is not `pneumatics.size()` are
   * dropped.
   * @param pneumatics One pneumatic per column, in table column order. Null
   * entries keep their column and are never driven.
   * @param config Polarity and construct time apply.
   */
  DiscreteActuatorMechanism(
      StateT initial_state,
      Table table,
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
      DiscreteActuatorMechanismConfig config = {})
      : DiscreteActuatorMechanism(std::move(initial_state),
                                  std::move(table),
                                  makeActuators(std::move(pneumatics)),
                                  std::move(config)) {}

  /// @brief Construct from a bank of pneumatics with a braced table.
  DiscreteActuatorMechanism(
      StateT initial_state,
      std::initializer_list<Entry> table,
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
      DiscreteActuatorMechanismConfig config = {})
      : DiscreteActuatorMechanism(std::move(initial_state),
                                  Table(table),
                                  makeActuators(std::move(pneumatics)),
                                  std::move(config)) {}

  /// The command factories capture `this`, so the mechanism must not be copied
  /// or moved once any of them has been handed to the scheduler.
  DiscreteActuatorMechanism(const DiscreteActuatorMechanism&) = delete;
  DiscreteActuatorMechanism& operator=(const DiscreteActuatorMechanism&) = delete;
  DiscreteActuatorMechanism(DiscreteActuatorMechanism&&) = delete;
  DiscreteActuatorMechanism& operator=(DiscreteActuatorMechanism&&) = delete;

  /// @brief Number of decodable states, that is, surviving table rows.
  std::size_t stateCount() const {
    return m_table.size();
  }

  /// @brief Number of actuator columns.
  std::size_t actuatorCount() const {
    return m_actuators.size();
  }

  /// @brief How many table rows were dropped at construction for having the
  /// wrong width. Anything but 0 means a row in the table is mistyped and the
  /// state it named is now unreachable.
  std::size_t droppedRowCount() const {
    return m_dropped_rows;
  }

  /// @brief True when no state is decodable.
  bool empty() const {
    return m_table.empty();
  }

  /// @brief Row index of @p state, or kNoState when it is not in the table.
  std::size_t indexOf(const StateT& state) const {
    for (std::size_t i = 0; i < m_table.size(); ++i) {
      if (m_table[i].first == state) {
        return i;
      }
    }
    return kNoState;
  }

  /// @brief Row index of the current state, or kNoState when undecodable.
  std::size_t currentIndex() const {
    return indexOf(StateMechanism<StateT>::getState());
  }

  /// @brief Whether @p state has a usable row in the table.
  bool isDecodable(const StateT& state) const {
    return indexOf(state) != kNoState;
  }

  /// @brief Pointer to the state stored at @p index, or nullptr when @p index
  /// is out of range. Valid for the lifetime of the mechanism.
  const StateT* stateAt(std::size_t index) const {
    if (index >= m_table.size()) {
      return nullptr;
    }
    return &m_table[index].first;
  }

  /**
   * @brief The actuator combination mapped to @p state.
   *
   * @return The row, always `actuatorCount()` values wide, or an empty vector
   * when @p state is undecodable. An empty result and a genuinely zero width
   * row are only ambiguous when actuatorCount() is 0; use isDecodable() if you
   * need to tell them apart.
   */
  Combination combinationFor(const StateT& state) const {
    const std::size_t index = indexOf(state);
    if (index == kNoState) {
      return Combination();
    }
    return m_table[index].second;
  }

  /// @brief The combination of the current state, empty when undecodable.
  Combination currentCombination() const {
    return combinationFor(StateMechanism<StateT>::getState());
  }

  /// @brief Whether actuator @p index is driven with the inverse of the table
  /// value.
  bool isInverted(std::size_t index) const {
    return index < m_inverted.size() && m_inverted[index];
  }

  /// @brief The value actuator @p index is given for table value @p value.
  bool rawValue(std::size_t index, bool value) const {
    return isInverted(index) ? !value : value;
  }

  /**
   * @brief Move to @p state and write the actuators immediately.
   *
   * An undecodable state is ignored: the mechanism keeps its current state and
   * the actuators keep their current values.
   *
   * @return true when the state was accepted.
   */
  bool setDiscreteState(StateT state) {
    if (!isDecodable(state)) {
      return false;
    }
    this->setState(std::move(state));
    apply();
    return true;
  }

  /// @brief The current state. Same as the inherited getState(), named to match
  /// setDiscreteState().
  const StateT& getDiscreteState() const {
    return StateMechanism<StateT>::getState();
  }

  /// @brief Rewrite the actuators from the cached state. A no-op when the
  /// current state is undecodable.
  void apply() {
    applyState(StateMechanism<StateT>::getState());
  }

  /**
   * @brief Step forward one row, clamping at the last row.
   *
   * Does nothing with an empty table. From an undecodable state this moves to
   * row 0 so the mechanism can recover.
   */
  void next() {
    stepTo(currentIndex(), 1, false);
  }

  /**
   * @brief Step back one row, clamping at the first row.
   *
   * Does nothing with an empty table. From an undecodable state this moves to
   * row 0 so the mechanism can recover.
   */
  void previous() {
    stepTo(currentIndex(), -1, false);
  }

  /// @brief Step forward one row, wrapping from the last row back to the first.
  /// This is the only stepper that wraps; next() clamps instead.
  void cycle() {
    stepTo(currentIndex(), 1, true);
  }

  /// @brief Command that moves to @p state once and finishes. Does nothing when
  /// @p state is undecodable.
  std::unique_ptr<Command> makeDiscreteStateCommand(StateT state) {
    return this->runOnce(
        [this, state = std::move(state)]() { setDiscreteState(state); });
  }

  /// @brief Command that steps forward once and finishes, clamping at the last
  /// row.
  std::unique_ptr<Command> makeNextCommand() {
    return this->runOnce([this]() { next(); });
  }

  /// @brief Command that steps back once and finishes, clamping at the first
  /// row.
  std::unique_ptr<Command> makePreviousCommand() {
    return this->runOnce([this]() { previous(); });
  }

  /// @brief Command that steps forward once and finishes, wrapping past the
  /// last row.
  std::unique_ptr<Command> makeCycleCommand() {
    return this->runOnce([this]() { cycle(); });
  }

  /**
   * @brief Command that holds @p state for @p duration, then finishes and
   * leaves the mechanism in that state. It does not restore the old state.
   *
   * An undecodable @p state yields a command that does nothing and finishes
   * immediately. The table cannot change after construction, so this check
   * cannot go stale.
   *
   * This is the one mutator that does not write the actuators itself. It caches
   * the state and lets periodic() push it, matching
   * ToggleMechanism::makeSetForCommand. Normally that costs one tick. While the
   * subsystem is disabled it costs more: periodic() is skipped, so this command
   * moves nothing at all, whereas setDiscreteState() and the one-shot factories
   * still drive the hardware. Do not rely on this one while disabled.
   */
  std::unique_ptr<Command> makeDiscreteStateForCommand(StateT state,
                                                       QTime duration) {
    if (!isDecodable(state)) {
      return this->runOnce([]() {});
    }
    return this->StateMechanism<StateT>::makeStateForCommand(std::move(state),
                                                             duration);
  }

protected:
  /// @brief Write every actuator with the combination of @p state. Writes
  /// nothing at all when @p state is undecodable.
  void applyState(const StateT& state) override {
    const std::size_t index = indexOf(state);
    if (index == kNoState) {
      return;
    }

    const Combination& combination = m_table[index].second;
    if (combination.size() != m_actuators.size()) {
      return;
    }

    for (std::size_t i = 0; i < m_actuators.size(); ++i) {
      if (m_actuators[i]) {
        m_actuators[i](rawValue(i, combination[i]));
      }
    }
  }

private:
  /// Hidden on purpose. These set a state without consulting the table, which
  /// would let a command cache a state applyState() can never drive. Use
  /// setDiscreteState() and the makeDiscrete* factories instead.
  using StateMechanism<StateT>::setState;
  using StateMechanism<StateT>::makeStateCommand;
  using StateMechanism<StateT>::makeStateOnceCommand;
  using StateMechanism<StateT>::makeStateUntilCommand;
  using StateMechanism<StateT>::makeStateForCommand;

  static Table pruneTable(Table table, std::size_t width) {
    Table pruned;
    pruned.reserve(table.size());
    for (Entry& entry : table) {
      if (entry.second.size() == width) {
        pruned.push_back(std::move(entry));
      }
    }
    return pruned;
  }

  static std::vector<Actuator> makeActuators(
      std::vector<std::shared_ptr<device::Pneumatic>> pneumatics) {
    std::vector<Actuator> actuators;
    actuators.reserve(pneumatics.size());
    for (std::shared_ptr<device::Pneumatic>& pneumatic : pneumatics) {
      if (!pneumatic) {
        actuators.emplace_back();
        continue;
      }
      actuators.emplace_back(
          [pneumatic](bool value) { pneumatic->set_value(value); });
    }
    return actuators;
  }

  void stepTo(std::size_t from, int delta, bool wrap) {
    if (m_table.empty()) {
      return;
    }
    if (from == kNoState) {
      setDiscreteState(m_table.front().first);
      return;
    }

    const std::size_t last = m_table.size() - 1;
    std::size_t target = from;
    if (delta > 0) {
      if (from == last) {
        if (!wrap) {
          return;
        }
        target = 0;
      } else {
        target = from + 1;
      }
    } else {
      if (from == 0) {
        if (!wrap) {
          return;
        }
        target = last;
      } else {
        target = from - 1;
      }
    }

    setDiscreteState(m_table[target].first);
  }

  Table m_table;
  std::vector<Actuator> m_actuators;
  std::vector<bool> m_inverted;
  std::size_t m_dropped_rows = 0;
};

}  // namespace mechanism
}  // namespace mclib
