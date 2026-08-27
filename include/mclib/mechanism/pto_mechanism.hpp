// mclib
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/device/pneumatic.hpp"
#include "mclib/device/types.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Configuration for a PTOMechanism.
 *
 * A power take-off shares one set of motors between two consumers. This library
 * calls them the disengaged consumer (the side the motors drive when the
 * solenoid is retracted, usually the drivetrain) and the engaged consumer (the
 * side the motors drive when the solenoid is thrown, usually a lift).
 */
struct PTOConfig {
  /// Ports of the shared motors.
  std::vector<std::int8_t> motor_ports;
  /// Gearset of the shared motors.
  device::Gearset gearset = device::Gearset::Blue;
  /// Three-wire port of the solenoid that throws the PTO.
  char adi_port = 'A';
  /// Smart port of a three-wire expander, or 0 when the solenoid is on the brain.
  int extender_port = 0;
  /// True when extending the solenoid routes the motors to the engaged consumer.
  bool engaged_when_extended = true;
  /// Which consumer owns the motors at construction time.
  bool initial_engaged = false;
  /**
   * How long after a shift drive writes stay blocked.
   *
   * Shifting a PTO moves a dog gear or a sliding collar. If the motors are
   * powered while that happens the teeth grind and can shear off, so the
   * mechanism stops the motors on every shift and refuses drive writes until
   * this much time has passed. 250 ms is a deliberately generous default: a
   * VEX solenoid throws in well under 100 ms, and the extra margin costs a
   * quarter second of driver control but protects the gearbox.
   */
  QTime shift_settle_time = 250 * millisecond;
  /**
   * How long an accepted drive write stays live before it decays to zero.
   *
   * The owning consumer is expected to write every scheduler tick, the way a
   * driver-control loop does. If it stops writing — its command ended, its
   * task died — the last voltage would otherwise stay latched on the motors
   * forever. Set to 0 to disable the watchdog and latch indefinitely.
   */
  QTime drive_timeout = 100 * millisecond;
};

/**
 * @brief One set of motors mechanically switched between two consumers.
 *
 * The state is the PTO position: true means the motors are routed to the
 * engaged consumer, false means they are routed to the disengaged consumer.
 *
 * Voltage never reaches the motors except through driveEngaged() and
 * driveDisengaged(). Each of those drops the write unless the caller's side
 * currently owns the PTO and the shift has settled. The guard is the point of
 * the class: a lift command writing volts while the PTO is routed to the
 * drivetrain would drive the robot across the field.
 */
class PTOMechanism : public StateMechanism<bool> {
public:
  explicit PTOMechanism(const PTOConfig& config);
  PTOMechanism(std::initializer_list<std::int8_t> motor_ports,
               char adi_port,
               device::Gearset gearset = device::Gearset::Blue);
  /**
   * @brief Build a PTO around hardware that is owned elsewhere.
   *
   * Use this when the drivetrain already owns the motor group. Either pointer
   * may be null, in which case that piece of hardware is built from the config
   * instead, so the mechanism never holds a null device.
   *
   * From then on the PTO is the only thing allowed to set voltage on that
   * group: periodic() writes the commanded voltage every tick and would fight
   * anyone else. The drivetrain must route its writes through
   * driveDisengaged() too.
   */
  PTOMechanism(std::shared_ptr<device::MotorGroup> motors,
               std::shared_ptr<device::IPneumatic> pneumatic,
               const PTOConfig& config);

  void engage();
  void disengage();
  void setEngaged(bool engaged);
  void toggle();
  bool isEngaged() const;

  /// True once shift_settle_time has elapsed since the last shift.
  bool isShiftSettled() const;
  /// Milliseconds still to wait before drive writes are accepted, 0 when settled.
  QTime remainingSettleTime() const;

  /**
   * @brief Drive the motors from the engaged consumer.
   *
   * @return true if the write reached the motors, false if it was dropped
   *         because the PTO is routed elsewhere or is still settling.
   */
  bool driveEngaged(double volts);

  /**
   * @brief Drive the motors from the disengaged consumer.
   *
   * @return true if the write reached the motors, false if it was dropped
   *         because the PTO is routed elsewhere or is still settling.
   */
  bool driveDisengaged(double volts);

  /**
   * @brief Drive from a side named by the state it owns.
   *
   * @return true if the write reached the motors, false if it was dropped.
   */
  bool drive(bool from_engaged_side, double volts);

  /**
   * @brief Zero the commanded voltage, whichever side owns the PTO.
   *
   * Unconditional on purpose: zeroing the motors can never move the robot, and
   * refusing it would leave stale voltage applied. It is an emergency stop, not
   * a per-side stop — calling it from the side that does not own the PTO kills
   * the owner's motion. To stop only your own side, use `driveEngaged(0.0)` or
   * `driveDisengaged(0.0)`, which the guard drops when you do not own the PTO.
   */
  void stop();

  /// The voltage currently commanded by whichever side owns the PTO.
  double getCommandedVoltage() const;

  /**
   * @brief The shared motor group, for configuration and telemetry.
   *
   * Brake modes, encoder positions, temperatures, current draw. Writing
   * voltage through this reference bypasses the ownership guard and defeats
   * the whole mechanism, so route every drive write through driveEngaged() or
   * driveDisengaged() instead. motorsFor() is the accessor that keeps the
   * guard.
   */
  device::MotorGroup& motors();

  /**
   * @brief The shared motor group, but only for the side that owns it.
   *
   * Ownership-checked configuration and telemetry: brake mode, encoders,
   * temperatures. Voltage still belongs to driveEngaged()/driveDisengaged() —
   * periodic() rewrites the commanded voltage every tick, so a setVoltage()
   * through this pointer is overwritten on the next scheduler pass.
   *
   * @return the group when the named side owns a settled PTO, nullptr
   *         otherwise. A null return means "not yours, do not touch".
   *
   * The result is only valid for the tick it was fetched on. Caching it across
   * ticks reintroduces exactly the bug this class exists to stop: the pointer
   * stays valid after a shift while the ownership behind it has changed. Call
   * it again every tick, or use driveEngaged()/driveDisengaged().
   */
  device::MotorGroup* motorsFor(bool engaged_side);

  /// One-shot: routes the PTO to the engaged consumer and finishes.
  std::unique_ptr<Command> makeEngageCommand();
  /// One-shot: routes the PTO to the disengaged consumer and finishes.
  std::unique_ptr<Command> makeDisengageCommand();
  /// One-shot: flips the PTO and finishes.
  std::unique_ptr<Command> makeToggleCommand();
  /**
   * @brief Shift the PTO and hold the subsystem until the shift has settled.
   *
   * Unlike the one-shot commands this one keeps requiring the mechanism for
   * the whole settle window, so a drive or lift command that requires the PTO
   * cannot start writing into a half-thrown gearbox.
   */
  std::unique_ptr<Command> makeShiftCommand(bool engaged);

protected:
  void applyState(const bool& engaged) override;
  void onStateChanged(const bool& engaged) override;

private:
  static std::shared_ptr<device::MotorGroup> makeMotors(const PTOConfig& config);
  static std::shared_ptr<device::IPneumatic> makePneumatic(const PTOConfig& config);
  static PTOConfig makeConfig(std::vector<std::int8_t> motor_ports,
                              char adi_port,
                              device::Gearset gearset);

  bool acceptsWriteFrom(bool engaged_side) const;
  bool isDriveWriteFresh() const;
  void applySolenoid(bool engaged);

  PTOConfig m_config;
  std::shared_ptr<device::MotorGroup> m_motors;
  std::shared_ptr<device::IPneumatic> m_pneumatic;
  double m_commanded_volts = 0.0;
  QTime m_last_shift_time = 0.0;
  QTime m_last_write_time = 0.0;
};

}  // namespace mechanism
}  // namespace mclib
