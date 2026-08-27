// mclib
#pragma once

#include "mclib/mechanism/mechanism.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Tuning and limits for a PositionMechanism.
 */
struct PositionMechanismConfig {
  double kp = 0.08;
  double ki = 0.0;
  double kd = 0.0;
  double max_voltage = 12.0;
  double small_error = 2.0;
  double big_error = 5.0;
  double small_duration_ms = 50.0;
  double big_duration_ms = 250.0;
  double derivative_tolerance = 5.0;
};

/**
 * @brief Generic closed-loop position mechanism.
 *
 * The state is the target position, in whatever unit the position source
 * reports (degrees for device::Rotation, ticks for a raw encoder, and so on).
 * The mechanism never touches a device itself: it reads through a
 * std::function<double()> and writes volts through a
 * std::function<void(double)>, so the same class drives an arm on a rotation
 * sensor, a lift on a motor encoder, or a turret on a potentiometer.
 *
 * A mechanism starts idle at 0 V. It only begins driving once moveTo(),
 * hold(), stop(), or setManualVoltage() is called.
 */
class PositionMechanism : public StateMechanism<double> {
public:
  /// Reads the current position.
  using PositionSource = std::function<double()>;
  /// Applies a voltage, in volts.
  using VoltageSink = std::function<void(double)>;

  PositionMechanism(PositionSource position_source,
                    VoltageSink voltage_sink,
                    const PositionMechanismConfig& config = {});

  /**
   * @brief Drive to @p target under PID control.
   *
   * Always restarts the loop, even when @p target equals the current target.
   * That matters after a manual-voltage override: re-issuing the same target
   * has to take control back.
   */
  void moveTo(double target);

  /**
   * @brief Set the target position.
   *
   * Hides StateMechanism::setState, which skips onStateChanged when the value
   * is unchanged and so would leave a manual override in place. This one is
   * just moveTo(). The state-command factories below are hidden for the same
   * reason.
   */
  void setState(double state);

  /// Resume closed-loop control at the existing target.
  void hold();

  /// Override the loop and apply @p volts directly until the next moveTo().
  void setManualVoltage(double volts);

  /**
   * @brief Actively hold the present position.
   *
   * This is a brake, not a coast: the present position becomes the target and
   * the loop holds it, so a loaded arm does not fall. Use
   * setManualVoltage(0.0) if you want the mechanism to go limp instead.
   */
  void stop();

  double position() const;
  double target() const;

  /// Alias for position(), for mechanisms measured in degrees.
  double positionDeg() const;
  /// Alias for target(), for mechanisms measured in degrees.
  double targetDeg() const;

  /**
   * @brief True once the loop has settled on the target.
   *
   * Sticky until the next moveTo(): drifting back off the target re-engages
   * the loop but does not un-finish a move that already completed. Always
   * false under a manual override.
   */
  bool atTarget() const;

  /// True while a manual voltage is overriding the loop.
  bool isManual() const;

  const PositionMechanismConfig& getConfig() const;

  /**
   * @brief Move to @p target and finish on arrival.
   * @param timeout_ms A value above 0.0 also finishes the command after that
   *        many milliseconds, whether or not the mechanism arrived.
   */
  std::unique_ptr<Command> makeMoveToCommand(double target,
                                             double timeout_ms = 0.0);
  /// Hold the current target for as long as the command is scheduled.
  std::unique_ptr<Command> makeHoldCommand();
  /// Apply @p volts for as long as the command is scheduled.
  std::unique_ptr<Command> makeManualCommand(double volts);
  /// Actively hold the present position for as long as the command runs.
  std::unique_ptr<Command> makeStopCommand();

  std::unique_ptr<Command> makeStateCommand(double state);
  std::unique_ptr<Command> makeStateOnceCommand(double state);
  std::unique_ptr<Command> makeStateUntilCommand(
      double state, std::function<bool()> is_finished);
  std::unique_ptr<Command> makeStateForCommand(double state, QTime duration);

protected:
  void applyState(const double& target) override;
  void onStateChanged(const double& target) override;

private:
  void beginClosedLoop(double target);
  double clampVoltage(double volts) const;

  PositionSource m_position_source;
  VoltageSink m_voltage_sink;
  PositionMechanismConfig m_config;
  PID m_pid;
  bool m_manual = true;
  double m_manual_voltage = 0.0;
  bool m_arrived = false;
};

}  // namespace mechanism
}  // namespace mclib
