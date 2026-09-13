// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/mechanism/mechanism.hpp"

#include <functional>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Phase of a homing run.
 *
 * A plain bool (homing / idle) cannot tell "finished successfully" apart from
 * "gave up", and it cannot express the back-off phase, which still drives the
 * motor but is no longer looking for the hard stop. So this is a scoped enum
 * instead of StateMechanism<bool>.
 */
enum class HomingState {
  Idle,        ///< Not homing. The mechanism does not touch the motor.
  Seeking,     ///< Driving toward the hard stop, watching the stop detectors.
  BackingOff,  ///< Stop found, driving away from it before zeroing.
  Homed,       ///< Zero found, position reset callback fired.
  Failed,      ///< Timed out before any detector fired.
};

/**
 * @brief Tuning values for a homing run.
 */
struct HomingMechanismConfig {
  /// Voltage used to drive into the hard stop. Signed: direction matters.
  double homing_voltage = -3.0;
  /// Current draw at or above this (amps) counts as a hard stop. <= 0 disables.
  /// The injected current source must return amps, not milliamps.
  double current_threshold_amps = 2.0;
  /// How long the over-current must persist before it counts (ms).
  /// Without a dwell, a loaded mechanism that is still accelerating trips the
  /// detector on its own inrush and zeroes the sensor mid-travel.
  /// startup_grace_ms covers the inrush at the very start of a run; this covers
  /// every later current spike - a load change, a hit, a direction reversal.
  /// 0 fires on the first over-threshold reading, which is the old behaviour.
  double current_dwell_ms = 100.0;
  /// |velocity| below this (rpm) counts as stalled. <= 0 disables.
  double velocity_threshold_rpm = 5.0;
  /// How long the stall must persist before it counts (ms).
  double stall_dwell_ms = 150.0;
  /// Ramp-up window at the start of a run (ms). Inrush current is ignored for
  /// this long, and the stall detector arms once it has passed even if the
  /// mechanism was never seen moving (it may already be against the stop).
  double startup_grace_ms = 150.0;
  /// Give up after this long (ms). <= 0 means never give up.
  double timeout_ms = 3000.0;
  /// Voltage driven away from the stop after it is found. 0 disables back-off.
  double backoff_voltage = 2.0;
  /// How long to drive backoff_voltage (ms). 0 disables back-off.
  double backoff_ms = 150.0;
};

/**
 * @brief Finds a mechanism's zero by driving into a hard stop, then re-zeroing
 *        the position sensor.
 *
 * Every hardware access is injected, so this works with a Motor, a MotorGroup,
 * a Rotation sensor, or any mix of them without depending on the device layer.
 *
 * Three stop detectors can be enabled independently; the first one to fire wins:
 *  1. a limit-switch predicate,
 *  2. current draw above HomingMechanismConfig::current_threshold_amps,
 *     sustained for HomingMechanismConfig::current_dwell_ms,
 *  3. a velocity stall sustained for HomingMechanismConfig::stall_dwell_ms.
 *
 * Both the current and the stall detector need a dwell for the same reason: a
 * single sample is not evidence. A loaded arm still accelerating draws inrush
 * current well past startup_grace_ms, and a detector with no dwell reads that
 * as the hard stop and zeroes the sensor in the middle of the travel.
 *
 * The stall detector is armed once the mechanism has been seen moving
 * (|rpm| >= velocity_threshold_rpm), so the ramp-up from rest does not read as
 * a stall. It also arms after startup_grace_ms even if no movement was seen,
 * which is the case when the mechanism starts out already against the stop.
 *
 * The sensor is zeroed the moment the stop is found, before any back-off, so
 * the recorded zero is the stop itself and not wherever the back-off ended.
 *
 * The state machine is stepped from periodic(), so the mechanism must be
 * registered with CommandScheduler::registerSubsystem. makeHomeCommand() also
 * steps it from execute(), so it works without registration too; stepping twice
 * in the same millisecond is ignored.
 */
class HomingMechanism : public StateMechanism<HomingState> {
public:
  using VoltageSink = std::function<void(double)>;
  using CurrentSource = std::function<double()>;
  using VelocitySource = std::function<double()>;
  using PositionReset = std::function<void()>;
  using LimitSwitch = std::function<bool()>;

  HomingMechanism(const HomingMechanismConfig& config, VoltageSink voltage_sink);

  /// Sensor zeroing callback. Called once per run, when the stop is found.
  void setPositionReset(PositionReset position_reset);
  /// Enables the limit-switch detector. Predicate is true when pressed.
  void setLimitSwitch(LimitSwitch limit_switch);
  /// Enables the current detector (needs current_threshold_amps > 0).
  /// The source must return amps; device::Motor::getCurrentDraw() and
  /// device::MotorGroup::getAverageCurrentDraw() return milliamps, so divide
  /// them by 1000.
  void setCurrentSource(CurrentSource current_source);
  /// Enables the stall detector (needs velocity_threshold_rpm > 0).
  void setVelocitySource(VelocitySource velocity_source);

  void startHoming();
  void cancelHoming();
  void onDisabled() override { cancelHoming(); stopMotor(); }

  bool isHoming() const;
  bool isHomed() const;
  bool hasFailed() const;

  const HomingMechanismConfig& getConfig() const;

  /**
   * @brief Command that runs one homing attempt and always finishes.
   *
   * @param timeout_ms Extra command-level timeout. <= 0 relies on the config
   *                   timeout only.
   */
  std::unique_ptr<Command> makeHomeCommand(double timeout_ms = 0.0);

protected:
  void applyState(const HomingState& state) override;
  void onStateChanged(const HomingState& state) override;

private:
  bool stopDetected();
  void setVoltage(double volts);
  void stopMotor();
  void recordZero();
  static double nowMs();

  HomingMechanismConfig m_config;
  VoltageSink m_voltage_sink;
  CurrentSource m_current_source;
  VelocitySource m_velocity_source;
  PositionReset m_position_reset;
  LimitSwitch m_limit_switch;

  double m_phase_start_ms = 0.0;
  double m_run_start_ms = 0.0;
  double m_stall_start_ms = 0.0;
  double m_current_start_ms = 0.0;
  double m_last_step_ms = 0.0;
  bool m_stepped = false;
  bool m_stalling = false;
  bool m_over_current = false;
  bool m_has_moved = false;
};

}  // namespace mechanism
}  // namespace mclib
