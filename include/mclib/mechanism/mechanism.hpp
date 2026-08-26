// mclib
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/units/units.hpp"
#include "pros/rtos.hpp"

#include <functional>
#include <memory>
#include <utility>

namespace mclib {
namespace mechanism {

template <typename StateT>
class StateMechanism : public Subsystem {
public:
  using State = StateT;

  explicit StateMechanism(StateT initial_state) : m_state(std::move(initial_state)) {}

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
          *start_time = pros::millis() * millisecond;
        },
        [this, state]() { setState(state); },
        [](bool) {},
        [start_time, duration]() {
          return pros::millis() * millisecond - *start_time >= duration;
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
