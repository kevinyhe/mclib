// mclib
#include "mclib/mechanism/lift.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace mechanism {

Lift::Lift(const LiftConfig& config)
    : StateMechanism<double>(config.down_deg),
      m_config(config),
      m_motors(config.motor_ports, config.gearset),
      m_rotation(config.rotation_port, config.rotation_reversed),
      m_pid(config.kp, config.ki, config.kd) {
  m_pid.setSmallBigErrorTolerance(config.small_error_deg, config.big_error_deg);
  m_pid.setSmallBigErrorDuration(config.small_duration_ms,
                                 config.big_duration_ms);
  m_pid.setDerivativeTolerance(config.derivative_tolerance);
  m_pid.setTarget(config.down_deg);
  m_motors.setBrakeMode(device::BrakeMode::Hold);
}

void Lift::setPreset(LiftPreset preset) {
  m_preset = preset;
  retarget(presetDeg(preset));
}

LiftPreset Lift::getPreset() const {
  return m_preset;
}

// Clamped at both ends. Stepping past a rail does nothing at all: re-issuing
// the current preset would reset the PID and drop atTarget() back to false for
// small_duration_ms, which would stall any command polling atTarget().
void Lift::next() {
  if (m_preset == LiftPreset::High) {
    return;
  }
  setPreset(static_cast<LiftPreset>(static_cast<int>(m_preset) + 1));
}

void Lift::previous() {
  if (m_preset == LiftPreset::Down) {
    return;
  }
  setPreset(static_cast<LiftPreset>(static_cast<int>(m_preset) - 1));
}

void Lift::moveTo(double target_deg) {
  m_preset = nearestPreset(target_deg);
  retarget(target_deg);
}

void Lift::setManualVoltage(double volts) {
  m_manual = true;
  m_manual_voltage = clampVoltage(volts, m_config.max_voltage);
}

void Lift::stop() {
  setManualVoltage(0.0);
}

double Lift::presetDeg(LiftPreset preset) const {
  switch (preset) {
    case LiftPreset::Low:
      return m_config.low_deg;
    case LiftPreset::Mid:
      return m_config.mid_deg;
    case LiftPreset::High:
      return m_config.high_deg;
    case LiftPreset::Down:
    default:
      return m_config.down_deg;
  }
}

double Lift::positionDeg() const {
  return m_rotation.getPositionDeg();
}

double Lift::targetDeg() const {
  return getState();
}

bool Lift::atTarget() {
  return m_pid.targetArrived();
}

std::unique_ptr<Command> Lift::makePresetCommand(LiftPreset preset,
                                                 double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  return std::make_unique<FunctionalCommand>(
      [this, preset, start_time]() {
        setPreset(preset);
        *start_time = static_cast<double>(pros::millis());
      },
      []() {},
      [](bool) {},
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(pros::millis()) - *start_time >= timeout_ms;
        return atTarget() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Lift::makeNextCommand() {
  return runOnce([this]() { next(); });
}

std::unique_ptr<Command> Lift::makePreviousCommand() {
  return runOnce([this]() { previous(); });
}

std::unique_ptr<Command> Lift::makeManualCommand(double volts) {
  return run([this, volts]() { setManualVoltage(volts); });
}

std::unique_ptr<Command> Lift::makeStopCommand() {
  return runOnce([this]() { stop(); });
}

void Lift::applyState(const double& target_deg) {
  if (m_manual) {
    if (m_manual_voltage == 0.0) {
      // move_voltage(0) coasts the motor into a back-drive; brake() is what
      // honours BrakeMode::Hold.
      m_motors.stop();
    } else {
      m_motors.setVoltage(m_manual_voltage);
    }
    return;
  }

  // PID::update latches arrived and then returns 0 forever until reset(), so a
  // settled lift would sag freely. Re-arm the PID once the lift has drifted
  // past the big-error tolerance so it drives back to the target.
  if (m_pid.targetArrived() &&
      std::abs(target_deg - positionDeg()) > m_config.big_error_deg) {
    m_pid.reset();
  }

  m_pid.setTarget(target_deg);
  const double output =
      clampVoltage(m_pid.update(positionDeg()), m_config.max_voltage);
  m_motors.setVoltage(output);
}

double Lift::clampVoltage(double volts, double max_voltage) {
  return std::clamp(volts, -max_voltage, max_voltage);
}

LiftPreset Lift::nearestPreset(double target_deg) const {
  const LiftPreset presets[] = {LiftPreset::Down, LiftPreset::Low,
                                LiftPreset::Mid, LiftPreset::High};
  LiftPreset best = LiftPreset::Down;
  double best_distance = std::abs(target_deg - presetDeg(best));
  for (const LiftPreset preset : presets) {
    const double distance = std::abs(target_deg - presetDeg(preset));
    if (distance < best_distance) {
      best_distance = distance;
      best = preset;
    }
  }
  return best;
}

// Unconditional: unlike Arm, this does not rely on StateMechanism::setState
// firing onStateChanged, which it only does when the value actually changes.
// Re-commanding the current target after a manual override must still clear
// manual mode and reset the PID.
void Lift::retarget(double target_deg) {
  m_manual = false;
  m_manual_voltage = 0.0;
  setState(target_deg);
  m_pid.reset();
  m_pid.setTarget(target_deg);
}

}  // namespace mechanism
}  // namespace mclib
