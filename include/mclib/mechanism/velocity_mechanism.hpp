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
 * divided by the motor's free speed in RPM. The PID only has to make up the
 * difference.
 *
 * RPM spaces. Two of them exist and mixing them up is the classic silent bug,
 * so they are pinned down here:
 *   - Motor RPM: what the velocity source reports, what the motor actually
 *     spins at. kp, ki, kd and kv are all volts per motor RPM.
 *   - Output RPM: what the geared-down (or up) shaft spins at. tolerance_rpm
 *     and integral_range_rpm are both output RPM, and so is every RPM the
 *     public API takes or returns.
 * ratio converts between them: output_rpm = motor_rpm * ratio.
 */
struct VelocityMechanismConfig {
  double kp = 0.02;
  double ki = 0.0;
  double kd = 0.0;
  /// Feedforward volts per *motor* RPM. 12.0 / motor_free_speed_rpm is a good
  /// start. Gearing does not change this number; ratio handles that.
  double kv = 0.0;
  double max_voltage = 12.0;
  /// External gear ratio, as output RPM per motor RPM:
  /// output_rpm = motor_rpm * ratio, motor_rpm = output_rpm / ratio.
  /// A mechanism geared 1:2 for speed (output spins twice as fast as the
  /// motor) is ratio = 2.0; geared 2:1 for torque it is 0.5. 1.0 means the
  /// output is the motor shaft, which is the default and reproduces the
  /// ungeared behaviour exactly.
  /// Must be > 0. A value <= 0 would divide by zero or invert the loop, so the
  /// constructor rejects it and falls back to 1.0; getConfig().ratio then
  /// reports the 1.0 that is actually in use, not the bad value passed in.
  double ratio = 1.0;
  /// How close to the target counts as "at speed", in *output* RPM.
  double tolerance_rpm = 100.0;
  /// How long the measured speed must stay inside tolerance_rpm before
  /// atSpeed() latches true, in milliseconds.
  double dwell_ms = 150.0;
  /// The integral only accumulates once |error| drops below this, in *output*
  /// RPM. 0 disables the gate and lets it accumulate everywhere.
  double integral_range_rpm = 200.0;
  /// Cap on the integral's contribution, in volts. 0 means no cap.
  double integral_max_volts = 2.0;
};

/**
 * @brief Generic closed-loop velocity mechanism. The state is the target output
 * RPM.
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
 *
 * The velocity source reports *motor* RPM. Everything on the public API -
 * setTargetRpm(), getTargetRpm(), getCurrentRpm(), makeSpinCommand(),
 * makeSpinUpCommand() - is in *output* RPM, the speed of the shaft after
 * config.ratio, because that is the speed a user actually cares about. The
 * conversion is output_rpm = motor_rpm * config.ratio.
 *
 * The control loop itself runs in motor RPM: the PID error and the kv
 * feedforward are both motor-space, so kp/ki/kd/kv keep their natural "volts
 * per motor RPM" meaning and do not have to be retuned when the gearing
 * changes. The user-facing config numbers that are in output RPM
 * (tolerance_rpm, integral_range_rpm) are divided by ratio on the way in.
 */
class VelocityMechanism : public StateMechanism<double> {
public:
  using VelocitySource = std::function<double()>;
  using VoltageSink = std::function<void(double)>;

  VelocityMechanism(VelocitySource velocity_source,
                    VoltageSink voltage_sink,
                    const VelocityMechanismConfig& config = {});

  /// Commands a target speed, in output RPM.
  void setTargetRpm(double rpm);
  /// The commanded target, in output RPM.
  double getTargetRpm() const;
  /// The measured speed, in output RPM: the velocity source's motor RPM times
  /// config.ratio.
  double getCurrentRpm() const;
  /// The measured speed straight from the velocity source, in motor RPM.
  double getCurrentMotorRpm() const;

  /// True once the measured output RPM has stayed within tolerance_rpm of a
  /// non-zero target continuously for dwell_ms. Goes false again as soon as the
  /// speed leaves tolerance, so a flywheel drained by a shot reports
  /// not-at-speed.
  bool atSpeed() const;
  /// True while a non-zero target is commanded but atSpeed() has not latched.
  bool isSpinningUp() const;

  void stop();

  const VelocityMechanismConfig& getConfig() const;

  /// Holds the target speed, in output RPM, forever. Never finishes.
  std::unique_ptr<Command> makeSpinCommand(double rpm);
  /// Commands the target speed, in output RPM, and finishes when atSpeed()
  /// latches, or after timeout_ms if timeout_ms > 0. Leaves it spinning.
  std::unique_ptr<Command> makeSpinUpCommand(double rpm,
                                             double timeout_ms = 0.0);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& target_rpm) override;
  void onStateChanged(const double& target_rpm) override;

private:
  /// Both arguments are output RPM.
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
