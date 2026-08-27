// mclib
#pragma once

#include "mclib/command/subsystem.h"
#include "mclib/mechanism/mechanism.hpp"

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief A mechanism whose state is one of N discrete named positions.
 *
 * Useful for arm presets, lift stages, hood angles: anything that moves between
 * a fixed, ordered list of setpoints. The mechanism owns the table that maps
 * each state to a numeric setpoint, and pushes the setpoint of the current
 * state into an injected sink every scheduler tick. The sink is usually a
 * lambda that feeds a position controller, so this class never has to know how
 * the position is actually held.
 *
 * The table is a `std::vector<std::pair<StateT, double>>` and is ordered.
 * Ordering is what gives next(), previous(), and cycle() a meaning: they step
 * through the table in the order it was given, not in numeric setpoint order.
 *
 * Stepping:
 * - next() and previous() clamp. At the last entry next() does nothing; at the
 *   first entry previous() does nothing.
 * - cycle() wraps. From the last entry it goes back to the first.
 *
 * `StateT` only needs `operator==` and `operator!=`, so a scoped enum works.
 *
 * Degenerate cases never invoke undefined behavior. applyState() runs every
 * tick, so a lookup miss must be cheap and safe:
 * - Empty table: the default setpoint is pushed every tick.
 * - Current state absent from the table: the default setpoint is pushed. This
 *   is the same number setpointFor() and currentSetpoint() report for that
 *   state, so an at-target check never disagrees with what the sink was given.
 *   The default is a constructor argument and is 0.0 unless you pass one.
 * - Stepping from a state that is not in the table moves to the first entry.
 *   Entry 0 is the home position by convention, so this resyncs somewhere
 *   known and low rather than somewhere surprising and high.
 *
 * The table is searched front to back and the first match wins, so a state
 * listed twice makes the later entry unreachable. List each state once.
 */
template <typename StateT>
class MultiPositionMechanism : public StateMechanism<StateT> {
public:
  using SetpointSink = std::function<void(double)>;
  using Entry = std::pair<StateT, double>;
  using Table = std::vector<Entry>;

  /// Returned by currentIndex() when the current state is not in the table.
  static constexpr std::size_t kNoPosition = static_cast<std::size_t>(-1);

  MultiPositionMechanism(StateT initial_state,
                         std::initializer_list<Entry> positions,
                         SetpointSink setpoint_sink,
                         double default_setpoint = 0.0)
      : MultiPositionMechanism(std::move(initial_state),
                               Table(positions),
                               std::move(setpoint_sink),
                               default_setpoint) {}

  MultiPositionMechanism(StateT initial_state,
                         Table positions,
                         SetpointSink setpoint_sink,
                         double default_setpoint = 0.0)
      : StateMechanism<StateT>(std::move(initial_state)),
        m_positions(std::move(positions)),
        m_setpoint_sink(std::move(setpoint_sink)),
        m_default_setpoint(default_setpoint) {}

  /// Number of entries in the setpoint table.
  std::size_t positionCount() const {
    return m_positions.size();
  }

  /// True when the table has no entries.
  bool empty() const {
    return m_positions.empty();
  }

  /// Pointer to the state stored at @p index, or nullptr if @p index is out of
  /// range. The pointer is invalidated by anything that rebuilds the table.
  const StateT* positionAt(std::size_t index) const {
    if (index >= m_positions.size()) {
      return nullptr;
    }
    return &m_positions[index].first;
  }

  /// Index of @p state in the table, or kNoPosition when it is absent.
  std::size_t indexOf(const StateT& state) const {
    for (std::size_t i = 0; i < m_positions.size(); ++i) {
      if (m_positions[i].first == state) {
        return i;
      }
    }
    return kNoPosition;
  }

  /// Index of the current state, or kNoPosition when it is not in the table.
  std::size_t currentIndex() const {
    return indexOf(StateMechanism<StateT>::getState());
  }

  /// Setpoint mapped to @p state, or the default setpoint when it is absent.
  double setpointFor(const StateT& state) const {
    const std::size_t index = indexOf(state);
    if (index == kNoPosition) {
      return m_default_setpoint;
    }
    return m_positions[index].second;
  }

  /// Setpoint of the current state.
  double currentSetpoint() const {
    return setpointFor(StateMechanism<StateT>::getState());
  }

  /// The setpoint returned for states that are not in the table.
  double defaultSetpoint() const {
    return m_default_setpoint;
  }

  /// Move to @p state. Same as setState(), named for readability at call sites.
  void setPosition(StateT state) {
    this->setState(std::move(state));
  }

  /**
   * Step forward one entry, clamping at the last entry.
   *
   * With an empty table this does nothing. If the current state is not in the
   * table, this moves to the first entry so the mechanism can recover.
   */
  void next() {
    stepTo(currentIndex(), 1, false);
  }

  /**
   * Step back one entry, clamping at the first entry.
   *
   * With an empty table this does nothing. If the current state is not in the
   * table, this moves to the first entry so the mechanism can recover.
   */
  void previous() {
    stepTo(currentIndex(), -1, false);
  }

  /**
   * Step forward one entry, wrapping from the last entry back to the first.
   *
   * This is the only stepper that wraps; next() clamps instead.
   */
  void cycle() {
    stepTo(currentIndex(), 1, true);
  }

  /// Command that holds @p state for as long as it runs.
  std::unique_ptr<Command> makePositionCommand(StateT state) {
    return this->makeStateCommand(std::move(state));
  }

  /// One-shot command that steps forward, clamping at the last entry.
  std::unique_ptr<Command> makeNextCommand() {
    return this->runOnce([this]() { next(); });
  }

  /// One-shot command that steps back, clamping at the first entry.
  std::unique_ptr<Command> makePreviousCommand() {
    return this->runOnce([this]() { previous(); });
  }

  /// One-shot command that steps forward, wrapping past the last entry.
  std::unique_ptr<Command> makeCycleCommand() {
    return this->runOnce([this]() { cycle(); });
  }

protected:
  void applyState(const StateT& state) override {
    if (!m_setpoint_sink) {
      return;
    }
    m_setpoint_sink(setpointFor(state));
  }

private:
  void stepTo(std::size_t from, int delta, bool wrap) {
    if (m_positions.empty()) {
      return;
    }
    if (from == kNoPosition) {
      this->setState(m_positions.front().first);
      return;
    }

    const std::size_t last = m_positions.size() - 1;
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

    this->setState(m_positions[target].first);
  }

  Table m_positions;
  SetpointSink m_setpoint_sink;
  double m_default_setpoint;
};

}  // namespace mechanism
}  // namespace mclib
