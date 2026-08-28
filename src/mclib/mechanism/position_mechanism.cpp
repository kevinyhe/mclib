// mclib
#include "mclib/mechanism/position_mechanism.hpp"

#include "mclib/time.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace mechanism {

PositionMechanism::PositionMechanism(PositionSource position_source,
                                     VoltageSink voltage_sink,
                                     const PositionMechanismConfig& config)
    : StateMechanism<double>(0.0),
      m_position_source(std::move(position_source)),
      m_voltage_sink(std::move(voltage_sink)),
      m_config(config),
      m_pid(config.kp, config.ki, config.kd) {
  m_pid.setSmallBigErrorTolerance(config.small_error, config.big_error);
  m_pid.setSmallBigErrorDuration(config.small_duration_ms,
                                 config.big_duration_ms);
  m_pid.setDerivativeTolerance(config.derivative_tolerance);
  m_pid.setHoldOutput(config.hold_output);
  // Bound the integral. PID's own default cap is 500, which is no cap at all
  // in volts, and with hold_output the loop keeps integrating after arrival:
  // a mechanism stuck outside small_error would otherwise walk the command up
  // to max_voltage and stall a motor there indefinitely.
  m_pid.setIntegralMax(config.integral_max_volts);
}

void PositionMechanism::moveTo(double target) {
  // StateMechanism::setState only calls onStateChanged when the value actually
  // changes, so the loop is restarted here as well. Without this, re-issuing
  // the current target after a manual override would never take control back.
  StateMechanism<double>::setState(target);
  beginClosedLoop(target);
}

void PositionMechanism::setState(double state) {
  moveTo(state);
}

void PositionMechanism::hold() {
  if (m_manual) {
    beginClosedLoop(getState());
  } else {
    m_pid.setTarget(getState());
  }
}

void PositionMechanism::setManualVoltage(double volts) {
  m_manual = true;
  m_manual_voltage = clampVoltage(volts);
}

void PositionMechanism::stop() {
  // Brake, not coast: latch the present position as the target and let the
  // loop hold it.
  moveTo(position());
}

double PositionMechanism::position() const {
  return m_position_source ? m_position_source() : 0.0;
}

double PositionMechanism::target() const {
  return getState();
}

double PositionMechanism::positionDeg() const {
  return position();
}

double PositionMechanism::targetDeg() const {
  return target();
}

bool PositionMechanism::atTarget() const {
  return !m_manual && m_arrived;
}

bool PositionMechanism::isManual() const {
  return m_manual;
}

const PositionMechanismConfig& PositionMechanism::getConfig() const {
  return m_config;
}

std::unique_ptr<Command> PositionMechanism::makeMoveToCommand(
    double target, double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, target, start_time]() {
        moveTo(target);
        *start_time = static_cast<double>(mclib::time::millis());
      },
      []() {},
      [this](bool interrupted) {
        if (interrupted) {
          stop();
        }
      },
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(mclib::time::millis()) - *start_time >= timeout_ms;
        return atTarget() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> PositionMechanism::makeHoldCommand() {
  return run([this]() { hold(); });
}

std::unique_ptr<Command> PositionMechanism::makeManualCommand(double volts) {
  return run([this, volts]() { setManualVoltage(volts); });
}

std::unique_ptr<Command> PositionMechanism::makeStopCommand() {
  return startEnd([this]() { stop(); }, []() {});
}

std::unique_ptr<Command> PositionMechanism::makeStateCommand(double state) {
  return run([this, state]() { moveTo(state); });
}

std::unique_ptr<Command> PositionMechanism::makeStateOnceCommand(double state) {
  return runOnce([this, state]() { moveTo(state); });
}

std::unique_ptr<Command> PositionMechanism::makeStateUntilCommand(
    double state, std::function<bool()> is_finished) {
  return runUntil([this, state]() { moveTo(state); }, std::move(is_finished));
}

std::unique_ptr<Command> PositionMechanism::makeStateForCommand(
    double state, QTime duration) {
  auto start_time = std::make_shared<QTime>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, state, start_time]() {
        moveTo(state);
        *start_time = mclib::time::now();
      },
      []() {},
      [](bool) {},
      [start_time, duration]() {
        return mclib::time::now() - *start_time >= duration;
      },
      std::initializer_list<Subsystem*>{this});
}

void PositionMechanism::applyState(const double& target) {
  if (!m_voltage_sink) {
    return;
  }

  if (m_manual) {
    m_voltage_sink(m_manual_voltage);
    return;
  }

  const double current = position();

  if (m_pid.targetArrived()) {
    m_arrived = true;
    // Without hold_output, PID::update() returns a hard 0 on every tick once
    // it has latched, so a settled mechanism has no holding torque. Re-arm the
    // loop once the mechanism has drifted back outside the small-error band.
    // atTarget() stays true across the re-arm so an already-finished move does
    // not restart.
    //
    // With hold_output the PID keeps driving after arrival, so this re-arm is
    // both redundant and harmful: the reset would throw away the accumulated
    // integral and restart both settle timers every time the drift crossed the
    // band, for a loop that is already driving. Exactly one of the two
    // mechanisms is active.
    //
    // It would not spike the derivative: reset() sets first_time, and that
    // branch copies previous_error from the current error before the delta is
    // taken, so the tick after a reset has derivative 0.
    if (!m_config.hold_output &&
        std::fabs(target - current) > m_config.small_error) {
      m_pid.reset();
    }
  }

  m_pid.setTarget(target);
  m_voltage_sink(clampVoltage(m_pid.update(current)));

  if (m_pid.targetArrived()) {
    m_arrived = true;
  }
}

void PositionMechanism::onStateChanged(const double& target) {
  beginClosedLoop(target);
}

void PositionMechanism::beginClosedLoop(double target) {
  m_manual = false;
  m_manual_voltage = 0.0;
  m_arrived = false;
  m_pid.reset();
  m_pid.setTarget(target);
}

double PositionMechanism::clampVoltage(double volts) const {
  return std::clamp(volts, -m_config.max_voltage, m_config.max_voltage);
}

}  // namespace mechanism
}  // namespace mclib
