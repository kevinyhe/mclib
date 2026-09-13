// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>
#include <utility>

namespace mclib {
namespace mechanism {

/**
 * @brief Subsystem that owns one state value and pushes it to hardware every
 * periodic() tick.
 *
 * @details Subclasses implement applyState() to write the state to motors or
 * pneumatics. Not copyable and not movable, see the deleted members below.
 */
template <typename StateT>
class StateMechanism : public Subsystem {
public:
  using State = StateT;

  explicit StateMechanism(StateT initial_state) : m_state(std::move(initial_state)) {}

  // Not copyable and not movable, on purpose.
  //
  // Subclasses routinely store a std::function that captured `this`
  // (MappedMechanism::ApplyState, MotorStateMechanism's voltage map, the
  // callbacks in the position/velocity/conveyor/PTO mechanisms). Copying or
  // moving one of those objects copies the callable but not what it points at,
  // so the new object's periodic() would drive hardware through a pointer to
  // the old, possibly destroyed, object. Storing mechanisms by value in a
  // container that reallocates, e.g. std::vector<PositionMechanism>, is the
  // obvious way to ask for that.
  //
  // A mechanism is also an identity: it is registered with the
  // CommandScheduler by address and owns its default command, so a second copy
  // of one was never meaningful. Hold mechanisms in place (static storage, or
  // std::vector<std::unique_ptr<T>>) instead.
  //
  // Today the base class already blocks both by accident: Subsystem holds a
  // std::unique_ptr<Command> (so the copy constructor is implicitly deleted)
  // and declares a virtual destructor (so no move constructor is implicitly
  // declared, and a move request falls back to that deleted copy). Relying on
  // that is fragile - dropping the unique_ptr member would silently re-enable
  // copying - and the resulting error message points at Subsystem's members
  // rather than at the rule. Deleting them here pins the guarantee and makes
  // the diagnostic say what is actually wrong.
  StateMechanism(const StateMechanism&) = delete;
  StateMechanism& operator=(const StateMechanism&) = delete;
  StateMechanism(StateMechanism&&) = delete;
  StateMechanism& operator=(StateMechanism&&) = delete;

  void setState(StateT state) {
    if (state != m_state) {
      m_state = std::move(state);
      onStateChanged(m_state);
    }
  }

  const StateT& getState() const {
    return m_state;
  }

  void periodic() override {
    applyState(m_state);
  }

  std::unique_ptr<Command> makeStateCommand(StateT state) {
    return run([this, state = std::move(state)]() { setState(state); });
  }

  std::unique_ptr<Command> makeStateOnceCommand(StateT state) {
    return runOnce([this, state = std::move(state)]() { setState(state); });
  }

  std::unique_ptr<Command> makeStateUntilCommand(StateT state,
                                                std::function<bool()> is_finished) {
    return runUntil(
        [this, state = std::move(state)]() { setState(state); },
        std::move(is_finished));
  }

  std::unique_ptr<Command> makeStateForCommand(StateT state, QTime duration) {
    auto start_time = std::make_shared<QTime>(0.0);
    return std::make_unique<FunctionalCommand>(
        [this, state, start_time]() {
          setState(state);
          *start_time = mclib::time::now();
        },
        [this, state]() { setState(state); },
        [](bool) {},
        [start_time, duration]() {
          return mclib::time::now() - *start_time >= duration;
        },
        std::initializer_list<Subsystem*>{this});
  }

protected:
  virtual void applyState(const StateT& state) = 0;
  virtual void onStateChanged(const StateT& state) {
    (void)state;
  }

private:
  StateT m_state;
};

/**
 * @brief StateMechanism whose applyState() is supplied as a callable instead of
 * being overridden.
 *
 * @details The stored callable usually captures `this` or hardware handles, so
 * this type inherits StateMechanism's deleted copy and move operations.
 */
template <typename StateT>
class MappedMechanism : public StateMechanism<StateT> {
public:
  using ApplyState = std::function<void(const StateT&)>;

  MappedMechanism(StateT initial_state, ApplyState apply_state)
      : StateMechanism<StateT>(std::move(initial_state)),
        m_apply_state(std::move(apply_state)) {}

protected:
  void applyState(const StateT& state) override {
    m_apply_state(state);
  }

private:
  ApplyState m_apply_state;
};

}  // namespace mechanism
}  // namespace mclib
