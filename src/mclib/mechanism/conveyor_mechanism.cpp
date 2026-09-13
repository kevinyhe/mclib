// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/mechanism/conveyor_mechanism.hpp"

#include "mclib/time.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace mechanism {

ConveyorMechanism::ConveyorMechanism(const ConveyorConfig& config,
                                     SensorGate sensor_gate)
    : StateMechanism<ConveyorState>(ConveyorState::Stopped),
      m_config(config),
      m_motors(config.motor_ports, config.gearset),
      m_sensor_gate(std::move(sensor_gate)) {}

ConveyorMechanism::ConveyorMechanism(
    std::initializer_list<std::int8_t> motor_ports, device::Gearset gearset)
    : ConveyorMechanism(ConveyorConfig{std::vector<std::int8_t>(motor_ports),
                                       gearset}) {}

void ConveyorMechanism::setConveyorState(ConveyorState state) {
  setState(state);
}

ConveyorState ConveyorMechanism::getConveyorState() const {
  return getState();
}

void ConveyorMechanism::stop() {
  setConveyorState(ConveyorState::Stopped);
}

void ConveyorMechanism::setSensorGate(SensorGate sensor_gate) {
  m_sensor_gate = std::move(sensor_gate);
  m_object_present = false;
  m_index_latched = false;
}

bool ConveyorMechanism::isJammed() const {
  return m_jammed;
}

bool ConveyorMechanism::hasObject() const {
  return m_object_present;
}

void ConveyorMechanism::clearJam() {
  resetRecovery();
}

std::unique_ptr<Command> ConveyorMechanism::makeForwardCommand() {
  return makeStateCommand(ConveyorState::Forward);
}

std::unique_ptr<Command> ConveyorMechanism::makeReverseCommand() {
  return makeStateCommand(ConveyorState::Reverse);
}

std::unique_ptr<Command> ConveyorMechanism::makeStopCommand() {
  return makeStateCommand(ConveyorState::Stopped);
}

std::unique_ptr<Command> ConveyorMechanism::makeIndexCommand(double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, start_time]() {
        beginIndex();
        *start_time = static_cast<double>(mclib::time::millis());
      },
      []() {},
      [this](bool) {
        // Leaving the state alone on a latched jam keeps isJammed() readable
        // by the caller. applyState already holds the motors at zero volts.
        if (!m_jammed) {
          stop();
        }
      },
      [this, start_time, timeout_ms]() {
        if (!m_sensor_gate) {
          return true;
        }
        if (m_index_latched || m_jammed) {
          return true;
        }
        return timeout_ms > 0.0 &&
               static_cast<double>(mclib::time::millis()) - *start_time >= timeout_ms;
      },
      std::initializer_list<Subsystem*>{this});
}

void ConveyorMechanism::applyState(const ConveyorState& state) {
  const double now = static_cast<double>(mclib::time::millis());

  m_object_present = m_sensor_gate ? m_sensor_gate() : false;
  if (state == ConveyorState::IndexToSensor && m_object_present) {
    m_index_latched = true;
  }

  // device::MotorGroup reports current in milliamps.
  const double current_amps = m_motors.getAverageCurrentDraw() / 1000.0;
  const double velocity_rpm = std::fabs(m_motors.getAverageActualVelocity());

  if (m_unjamming) {
    if (now < m_unjam_end_ms) {
      m_motors.setVoltage(clampVoltage(m_config.unjam_voltage));
      return;
    }
    m_unjamming = false;
    m_jam_timing = false;
  }

  if (m_jammed) {
    m_motors.setVoltage(0.0);
    return;
  }

  const double voltage = voltageForState(state);

  if (voltage != 0.0) {
    const bool stalling = current_amps > m_config.jam_current_amps &&
                          velocity_rpm < m_config.jam_velocity_rpm;

    if (!stalling) {
      m_jam_timing = false;
      // The conveyor is moving again. Once it has run clean for long enough,
      // give the retry budget back so a later, unrelated jam still recovers.
      if (m_unjam_retries > 0) {
        if (!m_clear_timing) {
          m_clear_timing = true;
          m_clear_start_ms = now;
        } else if (now - m_clear_start_ms >= m_config.jam_clear_ms) {
          m_clear_timing = false;
          m_unjam_retries = 0;
        }
      }
    } else if (!m_jam_timing) {
      m_clear_timing = false;
      m_jam_timing = true;
      m_jam_start_ms = now;
    } else if (now - m_jam_start_ms >= m_config.jam_dwell_ms) {
      m_jam_timing = false;
      if (m_unjam_retries >= m_config.max_unjam_retries) {
        m_jammed = true;
        m_motors.setVoltage(0.0);
        return;
      }
      ++m_unjam_retries;
      m_unjamming = true;
      m_unjam_end_ms = now + m_config.unjam_ms;
      m_motors.setVoltage(clampVoltage(m_config.unjam_voltage));
      return;
    }
  } else {
    m_jam_timing = false;
    m_clear_timing = false;
  }

  m_motors.setVoltage(voltage);
}

void ConveyorMechanism::onStateChanged(const ConveyorState& state) {
  (void)state;
  m_index_latched = false;
  resetRecovery();
}

double ConveyorMechanism::voltageForState(ConveyorState state) const {
  switch (state) {
    case ConveyorState::Forward:
      return clampVoltage(m_config.forward_voltage);
    case ConveyorState::Reverse:
      return clampVoltage(m_config.reverse_voltage);
    case ConveyorState::IndexToSensor:
      // Driven off the live gate reading, not the latch, so removing the
      // object re-arms the conveyor for the next one.
      if (!m_sensor_gate || m_object_present) {
        return 0.0;
      }
      return clampVoltage(m_config.index_voltage);
    case ConveyorState::Stopped:
    default:
      return 0.0;
  }
}

double ConveyorMechanism::clampVoltage(double volts) const {
  return std::clamp(volts, -m_config.max_voltage, m_config.max_voltage);
}

void ConveyorMechanism::resetRecovery() {
  m_jam_timing = false;
  m_jam_start_ms = 0.0;
  m_clear_timing = false;
  m_clear_start_ms = 0.0;
  m_unjamming = false;
  m_unjam_end_ms = 0.0;
  m_unjam_retries = 0;
  m_jammed = false;
}

void ConveyorMechanism::beginIndex() {
  setConveyorState(ConveyorState::IndexToSensor);
  m_index_latched = false;
  resetRecovery();
}

}  // namespace mechanism
}  // namespace mclib
