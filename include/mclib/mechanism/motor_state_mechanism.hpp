// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/motor.hpp"
#include "mclib/mechanism/mechanism.hpp"

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <utility>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Motors driven at a fixed voltage per state, e.g. an intake with
 *        In / Out / Off.
 *
 * @details On disable the mechanism goes to its off state, so when the robot
 * is enabled again it does not drive the voltages it had before. Name the off
 * state with the constructors that take `off_state`; the others use
 * `initial_state`.
 */
template <typename StateT>
class MotorStateMechanism : public StateMechanism<StateT> {
public:
  using VoltageMap = std::function<std::vector<double>(const StateT&)>;

  /// @brief Off state is @p initial_state.
  MotorStateMechanism(std::initializer_list<std::int8_t> ports,
                      device::Gearset gearset,
                      StateT initial_state,
                      VoltageMap voltage_map)
      : MotorStateMechanism(std::vector<std::int8_t>(ports),
                            gearset,
                            initial_state,
                            initial_state,
                            std::move(voltage_map)) {}

  /// @brief Off state is @p initial_state.
  MotorStateMechanism(std::vector<std::int8_t> ports,
                      device::Gearset gearset,
                      StateT initial_state,
                      VoltageMap voltage_map)
      : MotorStateMechanism(std::move(ports),
                            gearset,
                            initial_state,
                            initial_state,
                            std::move(voltage_map)) {}

  /// @param off_state The state set on disable.
  MotorStateMechanism(std::initializer_list<std::int8_t> ports,
                      device::Gearset gearset,
                      StateT initial_state,
                      StateT off_state,
                      VoltageMap voltage_map)
      : MotorStateMechanism(std::vector<std::int8_t>(ports),
                            gearset,
                            std::move(initial_state),
                            std::move(off_state),
                            std::move(voltage_map)) {}

  /// @param off_state The state set on disable.
  MotorStateMechanism(std::vector<std::int8_t> ports,
                      device::Gearset gearset,
                      StateT initial_state,
                      StateT off_state,
                      VoltageMap voltage_map)
      : StateMechanism<StateT>(std::move(initial_state)),
        m_off_state(std::move(off_state)),
        m_voltage_map(std::move(voltage_map)) {
    m_motors.reserve(ports.size());
    for (const std::int8_t port : ports) {
      m_motors.emplace_back(port, gearset);
    }
  }

  std::size_t motorCount() const {
    return m_motors.size();
  }

  device::Motor& motor(std::size_t index) {
    return m_motors[index];
  }

  /// @brief The state set on disable.
  const StateT& offState() const {
    return m_off_state;
  }

  /// @brief Go to the off state. Nothing is written until the next
  ///        periodic(), which the scheduler skips while disabled.
  void onDisabled() override {
    this->setState(m_off_state);
  }

protected:
  void applyState(const StateT& state) override {
    const std::vector<double> voltages = m_voltage_map(state);
    if (voltages.empty()) {
      for (auto& motor : m_motors) {
        motor.setVoltage(0.0);
      }
      return;
    }

    if (voltages.size() == 1) {
      for (auto& motor : m_motors) {
        motor.setVoltage(voltages.front());
      }
      return;
    }

    for (std::size_t i = 0; i < m_motors.size(); ++i) {
      const double voltage = i < voltages.size() ? voltages[i] : 0.0;
      m_motors[i].setVoltage(voltage);
    }
  }

private:
  StateT m_off_state;
  std::vector<device::Motor> m_motors;
  VoltageMap m_voltage_map;
};

}  // namespace mechanism
}  // namespace mclib
