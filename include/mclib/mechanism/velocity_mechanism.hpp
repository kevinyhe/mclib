// mclib
#pragma once

#include "mclib/mechanism/mechanism.hpp"
#include "mclib/pid.hpp"

#include <functional>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Tuning and limits for a VelocityMechanism.
 *
 * kv is the dominant term in a velocity loop: it should be roughly max_voltage
 * divided by the free speed of the mechanism in RPM. The PID only has to make
 * up the difference.
 */
struct VelocityMechanismConfig {
  double kp = 0.02;
  double ki = 0.0;
  double kd = 0.0;
  /// Feedforward volts per RPM of target. 12.0 / free_speed_rpm is a good start.
  double kv = 0.0;
  double max_voltage = 12.0;
  /// How close to the target counts as "at speed", in RPM.
  double tolerance_rpm = 100.0;
  /// How long the measured speed must stay inside tolerance_rpm before
  /// atSpeed() latches true, in milliseconds.
  double dwell_ms = 150.0;
  /// The integral only accumulates once |error| drops below this, in RPM.
  /// 0 disables the gate and lets it accumulate everywhere.
  double integral_range_rpm = 200.0;
  /// Cap on the integral's contribution, in volts. 0 means no cap.
  double integral_max_volts = 2.0;
};

/**
 * @brief Generic closed-loop velocity mechanism. The state is the target RPM.
 *
 * The velocity source and the voltage output are injected as callbacks, so this
 * works with a device::MotorGroup, a single device::Motor, an external encoder,
 * or a test double. The usual pairing is:
 *
 * @code
 * VelocityMechanism spinner(
 *     [&motors] { return motors.getAverageActualVelocity(); },
 *     [&motors](double volts) { motors.setVoltage(volts); },
 *     config);
 * @endcode
 */
class VelocityMechanism : public StateMechanism<double> {
public:
  using VelocitySource = std::function<double()>;
  using VoltageSink = std::function<void(double)>;

  VelocityMechanism(VelocitySource velocity_source,
                    VoltageSink voltage_sink,
                    const VelocityMechanismConfig& config = {});

  void setTargetRpm(double rpm);
  double getTargetRpm() const;
  double getCurrentRpm() const;

  /// True once the measured RPM has stayed within tolerance_rpm of a non-zero
  /// target continuously for dwell_ms. Goes false again as soon as the speed
  /// leaves tolerance, so a flywheel drained by a shot reports not-at-speed.
  bool atSpeed() const;
  /// True while a non-zero target is commanded but atSpeed() has not latched.
  bool isSpinningUp() const;

  void stop();

  const VelocityMechanismConfig& getConfig() const;

  /// Holds the target speed forever. Never finishes.
  std::unique_ptr<Command> makeSpinCommand(double rpm);
  /// Commands the target speed and finishes when atSpeed() latches, or after
  /// timeout_ms if timeout_ms > 0. Leaves the mechanism spinning.
  std::unique_ptr<Command> makeSpinUpCommand(double rpm,
                                             double timeout_ms = 0.0);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& target_rpm) override;
  void onStateChanged(const double& target_rpm) override;

private:
  void updateAtSpeed(double error_rpm, double target_rpm);

  VelocityMechanismConfig m_config;
  VelocitySource m_velocity_source;
  VoltageSink m_voltage_sink;
  PID m_pid;
  bool m_at_speed = false;
  bool m_in_tolerance = false;
  double m_in_tolerance_since_ms = 0.0;
};

}  // namespace mechanism
}  // namespace mclib
