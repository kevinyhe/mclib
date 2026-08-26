// mclib
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/device/types.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/pid.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Tuning and hardware description for a Flywheel.
 *
 * Every field has a default, so a config can be written with designated
 * initializers and only the interesting fields set.
 */
struct FlywheelConfig {
  /// Motor ports. Negative port numbers mean a reversed motor.
  std::vector<std::int8_t> motor_ports;
  device::Gearset gearset = device::Gearset::Blue;
  /// External gear ratio: wheel RPM = motor RPM * ratio.
  double ratio = 1.0;
  double kp = 0.02;
  double ki = 0.0005;
  double kd = 0.0;
  /**
   * Feedforward volts per RPM of target. This carries the steady state, which
   * is why the integral can stay small. The default is 12 V over the free
   * speed of a blue cartridge at a 1:1 ratio; retune it for your gearing.
   */
  double kv = 12.0 / 600.0;
  double max_voltage = 12.0;
  /// How close the wheel has to be, in wheel RPM, to count as at speed.
  double tolerance_rpm = 50.0;
  /// How long it has to stay inside the tolerance band before atSpeed() is true.
  double dwell_ms = 150.0;
  /**
   * Integral band, in wheel RPM. The integral only accumulates once the error
   * is inside this band. Outside it the flywheel is still spinning up, and an
   * accumulating integral would command full voltage long past the target.
   * Zero or negative falls back to four times tolerance_rpm; there is no way
   * to ask for an unbounded integral.
   */
  double integral_range_rpm = 200.0;
  /// Cap on the integral term's own contribution, in volts.
  double integral_max_volts = 3.0;
};

/**
 * @brief A closed-loop flywheel. The mechanism state is the target wheel RPM.
 *
 * The output is a velocity feedforward (kv * target) plus a PID correction on
 * the RPM error, clamped to [0, max_voltage].
 */
class Flywheel : public StateMechanism<double> {
public:
  explicit Flywheel(const FlywheelConfig& config);
  /// Convenience form: ports and gearset only, everything else defaulted.
  Flywheel(std::initializer_list<std::int8_t> ports,
           device::Gearset gearset = device::Gearset::Blue);

  void setTargetRpm(double rpm);
  void stop();

  double getTargetRpm() const;
  double getCurrentRpm() const;
  /// True once the wheel has held the tolerance band for dwell_ms.
  bool atSpeed() const;

  /// Spins at rpm and never finishes on its own.
  std::unique_ptr<Command> makeSpinCommand(double rpm);
  /// Spins up to rpm and finishes at speed, or after timeout_ms if it is > 0.
  std::unique_ptr<Command> makeSpinUpCommand(double rpm,
                                             double timeout_ms = 0.0);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& target_rpm) override;
  void onStateChanged(const double& target_rpm) override;

private:
  FlywheelConfig m_config;
  device::MotorGroup m_motors;
  PID m_pid;
  bool m_at_speed = false;
  double m_in_band_since_ms = 0.0;
  bool m_in_band = false;
};

}  // namespace mechanism
}  // namespace mclib
