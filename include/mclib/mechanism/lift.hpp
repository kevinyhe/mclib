// mclib
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/device/rotation.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/pid.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief The four stops of a preset lift, ordered from lowest to highest.
 */
enum class LiftPreset {
  Down,
  Low,
  Mid,
  High,
};

/**
 * @brief Hardware ports, PID gains, and preset angles for a Lift.
 */
struct LiftConfig {
  std::vector<std::int8_t> motor_ports;
  std::int8_t rotation_port = 0;
  bool rotation_reversed = false;
  device::Gearset gearset = device::Gearset::Blue;
  double kp = 0.08;
  double ki = 0.0;
  double kd = 0.0;
  double max_voltage = 12.0;
  double small_error_deg = 2.0;
  double big_error_deg = 5.0;
  double small_duration_ms = 50.0;
  double big_duration_ms = 250.0;
  double derivative_tolerance = 5.0;
  double down_deg = 0.0;
  double low_deg = 90.0;
  double mid_deg = 180.0;
  double high_deg = 270.0;
};

/**
 * @brief A preset-based PID lift driven by a rotation sensor.
 *
 * The state type is `double` (the PID target in degrees) rather than
 * `LiftPreset`, because `applyState` needs an angle to feed the PID and
 * `moveTo` must be able to command an angle that is not a preset. The preset
 * stays the primary interface: `setPreset`/`next`/`previous` are the normal way
 * to drive the lift, and `getPreset` is always meaningful because `moveTo`
 * snaps it to the preset nearest the requested angle.
 *
 * Holding: `PID::update` zeroes its output once it has arrived, so `applyState`
 * watches for the lift drifting more than `big_error_deg` from the target and
 * resets the PID to drive it back. The motors also sit in `BrakeMode::Hold`,
 * and `stop()` brakes rather than commanding 0 V, so an idle lift resists
 * gravity mechanically as well.
 */
class Lift : public StateMechanism<double> {
public:
  explicit Lift(const LiftConfig& config);

  void setPreset(LiftPreset preset);
  LiftPreset getPreset() const;
  void next();
  void previous();

  void moveTo(double target_deg);
  void setManualVoltage(double volts);
  void stop();

  double presetDeg(LiftPreset preset) const;
  double positionDeg() const;
  double targetDeg() const;
  bool atTarget();

  std::unique_ptr<Command> makePresetCommand(LiftPreset preset,
                                             double timeout_ms = 0.0);
  std::unique_ptr<Command> makeNextCommand();
  std::unique_ptr<Command> makePreviousCommand();
  std::unique_ptr<Command> makeManualCommand(double volts);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& target_deg) override;

private:
  static double clampVoltage(double volts, double max_voltage);
  LiftPreset nearestPreset(double target_deg) const;
  void retarget(double target_deg);

  LiftConfig m_config;
  device::MotorGroup m_motors;
  device::Rotation m_rotation;
  PID m_pid;
  LiftPreset m_preset = LiftPreset::Down;
  bool m_manual = false;
  double m_manual_voltage = 0.0;
};

}  // namespace mechanism
}  // namespace mclib
