// mclib
#include "mclib/mechanism/homing_mechanism.hpp"

#include "mclib/time.hpp"

#include <cmath>
#include <utility>

namespace mclib {
namespace mechanism {

HomingMechanism::HomingMechanism(const HomingMechanismConfig& config,
                                 VoltageSink voltage_sink)
    : StateMechanism<HomingState>(HomingState::Idle),
      m_config(config),
      m_voltage_sink(std::move(voltage_sink)) {}

void HomingMechanism::setPositionReset(PositionReset position_reset) {
  m_position_reset = std::move(position_reset);
}

void HomingMechanism::setLimitSwitch(LimitSwitch limit_switch) {
  m_limit_switch = std::move(limit_switch);
}

void HomingMechanism::setCurrentSource(CurrentSource current_source) {
  m_current_source = std::move(current_source);
}

void HomingMechanism::setVelocitySource(VelocitySource velocity_source) {
  m_velocity_source = std::move(velocity_source);
}

void HomingMechanism::startHoming() {
  // Reset explicitly: setState() only fires onStateChanged() on a real change,
  // so restarting a run that is already Seeking would otherwise keep the old
  // timers.
  m_phase_start_ms = nowMs();
  m_run_start_ms = m_phase_start_ms;
  m_stall_start_ms = 0.0;
  m_current_start_ms = 0.0;
  m_stepped = false;
  m_stalling = false;
  m_over_current = false;
  m_has_moved = false;
  setState(HomingState::Seeking);
}

void HomingMechanism::cancelHoming() {
  if (isHoming()) {
    stopMotor();
    setState(HomingState::Idle);
  }
}

bool HomingMechanism::isHoming() const {
  return getState() == HomingState::Seeking ||
         getState() == HomingState::BackingOff;
}

bool HomingMechanism::isHomed() const {
  return getState() == HomingState::Homed;
}

bool HomingMechanism::hasFailed() const {
  return getState() == HomingState::Failed;
}

const HomingMechanismConfig& HomingMechanism::getConfig() const {
  return m_config;
}

std::unique_ptr<Command> HomingMechanism::makeHomeCommand(double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, start_time]() {
        *start_time = nowMs();
        startHoming();
      },
      // Step the state machine here as well as from periodic(), so the command
      // works whether or not the mechanism is registered with the scheduler.
      // A second step in the same millisecond is ignored.
      [this]() { periodic(); },
      [this](bool interrupted) {
        // Stop the motor on every exit path, interruption included. A homing
        // command that leaves the motor driving into a hard stop burns it out.
        stopMotor();
        if (isHoming()) {
          // Still seeking at the end means the command timed out. Report that
          // as Failed so callers can tell it apart from a run that never
          // started. An interrupted run just goes back to Idle.
          setState(interrupted ? HomingState::Idle : HomingState::Failed);
        }
      },
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 && nowMs() - *start_time >= timeout_ms;
        return isHomed() || hasFailed() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

void HomingMechanism::applyState(const HomingState& state) {
  if (state != HomingState::Seeking && state != HomingState::BackingOff) {
    // Idle on purpose: another controller may own the motor once homing is
    // done, so do not keep writing 0 V over it.
    return;
  }

  const double now = nowMs();
  if (m_stepped && now == m_last_step_ms) {
    // Already stepped this millisecond (periodic() plus the command's
    // execute()). Stepping twice would double the applied voltage writes.
    return;
  }
  m_stepped = true;
  m_last_step_ms = now;

  switch (state) {
    case HomingState::Seeking: {
      if (m_config.timeout_ms > 0.0 &&
          now - m_run_start_ms >= m_config.timeout_ms) {
        stopMotor();
        setState(HomingState::Failed);
        return;
      }

      setVoltage(m_config.homing_voltage);

      if (stopDetected()) {
        stopMotor();
        // Zero at the stop, before any back-off, so the recorded zero does not
        // depend on how far the mechanism coasts while backing away.
        recordZero();
        if (m_config.backoff_ms > 0.0 && m_config.backoff_voltage != 0.0) {
          setState(HomingState::BackingOff);
        } else {
          setState(HomingState::Homed);
        }
      }
      return;
    }

    case HomingState::BackingOff: {
      if (now - m_phase_start_ms >= m_config.backoff_ms) {
        stopMotor();
        setState(HomingState::Homed);
        return;
      }
      // Back-off drives away from the stop, so the sign is flipped relative to
      // homing_voltage.
      setVoltage(m_config.homing_voltage < 0.0 ? std::abs(m_config.backoff_voltage)
                                               : -std::abs(m_config.backoff_voltage));
      return;
    }

    default:
      return;
  }
}

void HomingMechanism::onStateChanged(const HomingState& state) {
  m_phase_start_ms = nowMs();
  m_stall_start_ms = 0.0;
  m_current_start_ms = 0.0;
  m_stalling = false;
  m_over_current = false;
  if (state == HomingState::Seeking) {
    // Covers setState(Seeking) called directly, without startHoming().
    m_run_start_ms = m_phase_start_ms;
    m_stepped = false;
    m_has_moved = false;
  }
}

bool HomingMechanism::stopDetected() {
  const double now = nowMs();
  // The first few ticks after voltage is applied are ramp-up: inrush current is
  // high and the mechanism has not started moving yet.
  const bool past_grace = now - m_run_start_ms >= m_config.startup_grace_ms;

  // A limit switch is never a false positive, so it is checked immediately: if
  // it is already pressed, the mechanism is already at the stop.
  if (m_limit_switch && m_limit_switch()) {
    return true;
  }

  // Current, with a dwell of its own. The stall detector waits stall_dwell_ms
  // before believing a low velocity; the current detector used to believe a
  // single over-threshold sample, so a loaded mechanism that was still
  // accelerating tripped it on its own inrush and zeroed the sensor somewhere
  // in the middle of its range. startup_grace_ms only covers the first
  // startup_grace_ms of a run; the dwell covers every spike after it.
  if (m_current_source && m_config.current_threshold_amps > 0.0) {
    const bool over = m_current_source() >= m_config.current_threshold_amps;
    if (!over || !past_grace) {
      // Below threshold, or still inside the ramp-up window where the reading
      // means nothing. Either way any timer in progress is stale.
      m_over_current = false;
      m_current_start_ms = 0.0;
    } else {
      if (!m_over_current) {
        m_over_current = true;
        m_current_start_ms = now;
      }
      if (now - m_current_start_ms >= m_config.current_dwell_ms) {
        return true;
      }
    }
  }

  if (m_velocity_source && m_config.velocity_threshold_rpm > 0.0) {
    const double speed = std::abs(m_velocity_source());
    if (speed >= m_config.velocity_threshold_rpm) {
      // The mechanism is moving, so the stall detector is armed from here on
      // and any stall timer in progress is stale.
      m_has_moved = true;
      m_stalling = false;
      m_stall_start_ms = 0.0;
      return false;
    }

    // Below threshold. Ignore it during the ramp-up window unless the mechanism
    // has already been seen moving; past the window it counts either way, since
    // a mechanism that starts out against the stop never moves at all.
    if (!m_has_moved && !past_grace) {
      return false;
    }

    if (!m_stalling) {
      m_stalling = true;
      m_stall_start_ms = now;
    }
    if (now - m_stall_start_ms >= m_config.stall_dwell_ms) {
      return true;
    }
  }

  return false;
}

void HomingMechanism::setVoltage(double volts) {
  if (m_voltage_sink) {
    m_voltage_sink(volts);
  }
}

void HomingMechanism::stopMotor() {
  setVoltage(0.0);
}

void HomingMechanism::recordZero() {
  if (m_position_reset) {
    m_position_reset();
  }
}

double HomingMechanism::nowMs() {
  return static_cast<double>(mclib::time::millis());
}

}  // namespace mechanism
}  // namespace mclib
