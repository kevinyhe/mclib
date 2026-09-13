// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/mechanism/toggle_group_mechanism.hpp"

#include "mclib/time.hpp"

#include <algorithm>
#include <initializer_list>
#include <utility>

namespace mclib {
namespace mechanism {

ToggleGroupMechanism::ToggleGroupMechanism(std::vector<Actuator> actuators,
                                           ToggleGroupMechanismConfig config)
    : StateMechanism<std::vector<bool>>(
          makeInitialStates(actuators.size(), config)),
      m_actuators(std::move(actuators)),
      m_inverted(makeInverted(m_actuators.size(), config)) {
  if (config.apply_on_construct) {
    apply();
  }
}

ToggleGroupMechanism::ToggleGroupMechanism(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
    ToggleGroupMechanismConfig config)
    : ToggleGroupMechanism(makeActuators(std::move(pneumatics)),
                           std::move(config)) {}

void ToggleGroupMechanism::setChannelStates(std::vector<bool> states) {
  states.resize(count(), false);
  Base::setState(std::move(states));
}

void ToggleGroupMechanism::set(std::size_t index, bool extended) {
  if (index >= count()) {
    return;
  }

  std::vector<bool> states = getState();
  states.resize(count(), false);
  states[index] = extended;
  setChannelStates(std::move(states));
  apply();
}

bool ToggleGroupMechanism::get(std::size_t index) const {
  const std::vector<bool>& states = getState();
  if (index >= count() || index >= states.size()) {
    return false;
  }
  return states[index];
}

void ToggleGroupMechanism::toggle(std::size_t index) {
  if (index >= count()) {
    return;
  }
  set(index, !get(index));
}

void ToggleGroupMechanism::setAll(bool extended) {
  setChannelStates(std::vector<bool>(count(), extended));
  apply();
}

void ToggleGroupMechanism::toggleAll() {
  std::vector<bool> states = getState();
  states.resize(count(), false);
  for (std::size_t i = 0; i < states.size(); ++i) {
    states[i] = !states[i];
  }
  setChannelStates(std::move(states));
  apply();
}

void ToggleGroupMechanism::apply() {
  applyState(getState());
}

bool ToggleGroupMechanism::allSet() const {
  for (std::size_t i = 0; i < count(); ++i) {
    if (!get(i)) {
      return false;
    }
  }
  return true;
}

bool ToggleGroupMechanism::anySet() const {
  for (std::size_t i = 0; i < count(); ++i) {
    if (get(i)) {
      return true;
    }
  }
  return false;
}

std::size_t ToggleGroupMechanism::setCount() const {
  std::size_t total = 0;
  for (std::size_t i = 0; i < count(); ++i) {
    if (get(i)) {
      ++total;
    }
  }
  return total;
}

std::size_t ToggleGroupMechanism::count() const {
  return m_actuators.size();
}

bool ToggleGroupMechanism::isInverted(std::size_t index) const {
  if (index >= count() || index >= m_inverted.size()) {
    return false;
  }
  return m_inverted[index];
}

bool ToggleGroupMechanism::rawValue(std::size_t index, bool extended) const {
  return isInverted(index) ? !extended : extended;
}

std::unique_ptr<Command> ToggleGroupMechanism::makeSetCommand(std::size_t index,
                                                              bool extended) {
  return runOnce([this, index, extended]() { set(index, extended); });
}

std::unique_ptr<Command> ToggleGroupMechanism::makeToggleCommand(
    std::size_t index) {
  return runOnce([this, index]() { toggle(index); });
}

std::unique_ptr<Command> ToggleGroupMechanism::makeSetAllCommand(bool extended) {
  return runOnce([this, extended]() { setAll(extended); });
}

std::unique_ptr<Command> ToggleGroupMechanism::makeToggleAllCommand() {
  return runOnce([this]() { toggleAll(); });
}

std::unique_ptr<Command> ToggleGroupMechanism::makeSetForCommand(
    std::size_t index, bool extended, QTime duration) {
  auto start_time = std::make_shared<QTime>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, index, extended, start_time]() {
        set(index, extended);
        *start_time = mclib::time::now();
      },
      [this, index, extended]() { set(index, extended); },
      [](bool) {},
      [start_time, duration]() {
        return mclib::time::now() - *start_time >= duration;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> ToggleGroupMechanism::makeSetAllForCommand(
    bool extended, QTime duration) {
  auto start_time = std::make_shared<QTime>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, extended, start_time]() {
        setAll(extended);
        *start_time = mclib::time::now();
      },
      [this, extended]() { setAll(extended); },
      [](bool) {},
      [start_time, duration]() {
        return mclib::time::now() - *start_time >= duration;
      },
      std::initializer_list<Subsystem*>{this});
}

void ToggleGroupMechanism::applyState(const std::vector<bool>& states) {
  const std::size_t channels = std::min(states.size(), m_actuators.size());
  for (std::size_t i = 0; i < channels; ++i) {
    if (m_actuators[i]) {
      m_actuators[i](rawValue(i, states[i]));
    }
  }
}

std::vector<ToggleGroupMechanism::Actuator> ToggleGroupMechanism::makeActuators(
    std::vector<std::shared_ptr<device::Pneumatic>> pneumatics) {
  std::vector<Actuator> actuators;
  actuators.reserve(pneumatics.size());

  for (auto& pneumatic : pneumatics) {
    actuators.push_back([pneumatic = std::move(pneumatic)](bool value) {
      if (pneumatic) {
        pneumatic->set_value(value);
      }
    });
  }

  return actuators;
}

std::vector<bool> ToggleGroupMechanism::makeInitialStates(
    std::size_t channel_count, const ToggleGroupMechanismConfig& config) {
  std::vector<bool> states(channel_count, config.initial_extended);
  const std::size_t given = std::min(channel_count, config.initial_states.size());

  for (std::size_t i = 0; i < given; ++i) {
    states[i] = config.initial_states[i];
  }

  return states;
}

std::vector<bool> ToggleGroupMechanism::makeInverted(
    std::size_t channel_count, const ToggleGroupMechanismConfig& config) {
  std::vector<bool> inverted(channel_count, false);
  const std::size_t given = std::min(channel_count, config.inverted.size());

  for (std::size_t i = 0; i < given; ++i) {
    inverted[i] = config.inverted[i];
  }

  return inverted;
}

}  // namespace mechanism
}  // namespace mclib
