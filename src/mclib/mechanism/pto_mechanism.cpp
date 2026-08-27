// mclib
#include "mclib/mechanism/pto_mechanism.hpp"

#include "mclib/command/functionalCommand.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <utility>

namespace mclib {
namespace mechanism {

PTOMechanism::PTOMechanism(const PTOConfig& config)
    : StateMechanism<bool>(config.initial_engaged),
      m_config(config),
      m_motors(makeMotors(config)),
      m_pneumatic(makePneumatic(config)),
      m_last_shift_time(pros::millis() * millisecond),
      m_last_write_time(pros::millis() * millisecond) {
  // Boot counts as a shift: nothing may drive until the hardware has had the
  // settle window to reach the position the config claims it starts in.
  applySolenoid(m_config.initial_engaged);
}

PTOMechanism::PTOMechanism(std::initializer_list<std::int8_t> motor_ports,
                           char adi_port,
                           device::Gearset gearset)
    : PTOMechanism(makeConfig(std::vector<std::int8_t>(motor_ports),
                              adi_port,
                              gearset)) {}

PTOMechanism::PTOMechanism(std::shared_ptr<device::MotorGroup> motors,
                           std::shared_ptr<device::IPneumatic> pneumatic,
                           const PTOConfig& config)
    : StateMechanism<bool>(config.initial_engaged),
      m_config(config),
      m_motors(motors ? std::move(motors) : makeMotors(config)),
      m_pneumatic(pneumatic ? std::move(pneumatic) : makePneumatic(config)),
      m_last_shift_time(pros::millis() * millisecond),
      m_last_write_time(pros::millis() * millisecond) {
  applySolenoid(m_config.initial_engaged);
}

void PTOMechanism::engage() {
  setEngaged(true);
}

void PTOMechanism::disengage() {
  setEngaged(false);
}

void PTOMechanism::setEngaged(bool engaged) {
  setState(engaged);
}

void PTOMechanism::toggle() {
  setEngaged(!isEngaged());
}

bool PTOMechanism::isEngaged() const {
  return getState();
}

bool PTOMechanism::isShiftSettled() const {
  return pros::millis() * millisecond - m_last_shift_time >= m_config.shift_settle_time;
}

QTime PTOMechanism::remainingSettleTime() const {
  const QTime elapsed = pros::millis() * millisecond - m_last_shift_time;
  return std::max(0.0, m_config.shift_settle_time - elapsed);
}

bool PTOMechanism::acceptsWriteFrom(bool engaged_side) const {
  // The whole safety argument lives in these two conditions.
  //
  // 1. engaged_side == isEngaged(): the caller only owns the motors while the
  //    PTO is routed to it. A lift command writing 12 V while the PTO is on
  //    the drivetrain would send the robot across the field, so that write is
  //    dropped, never queued and never applied later.
  // 2. isShiftSettled(): the dog gear is still moving during a shift. Powering
  //    the motors mid-throw grinds the teeth, so both sides are locked out for
  //    the whole settle window.
  return engaged_side == isEngaged() && isShiftSettled();
}

bool PTOMechanism::driveEngaged(double volts) {
  return drive(true, volts);
}

bool PTOMechanism::driveDisengaged(double volts) {
  return drive(false, volts);
}

bool PTOMechanism::drive(bool from_engaged_side, double volts) {
  if (!acceptsWriteFrom(from_engaged_side)) {
    return false;
  }

  m_commanded_volts = std::clamp(volts, -12.0, 12.0);
  m_last_write_time = pros::millis() * millisecond;
  return true;
}

void PTOMechanism::stop() {
  // Always allowed, from either side and mid-shift: zeroing the motors can
  // never move the robot, and blocking it would leave stale voltage applied.
  m_commanded_volts = 0.0;
  m_last_write_time = pros::millis() * millisecond;
  // Hit the hardware now instead of waiting for the next periodic() tick, so
  // stop() still works when the mechanism was never registered with the
  // CommandScheduler or the scheduler task is stuck.
  m_motors->stop();
}

bool PTOMechanism::isDriveWriteFresh() const {
  if (m_config.drive_timeout <= 0.0) {
    return true;
  }

  return pros::millis() * millisecond - m_last_write_time < m_config.drive_timeout;
}

void PTOMechanism::applySolenoid(bool engaged) {
  // set_value() takes the logical extended flag; the Pneumatic itself maps that
  // onto the solenoid's raw value.
  m_pneumatic->set_value(engaged == m_config.engaged_when_extended);
}

double PTOMechanism::getCommandedVoltage() const {
  return m_commanded_volts;
}

device::MotorGroup& PTOMechanism::motors() {
  return *m_motors;
}

device::MotorGroup* PTOMechanism::motorsFor(bool engaged_side) {
  if (!acceptsWriteFrom(engaged_side)) {
    return nullptr;
  }

  return m_motors.get();
}

std::unique_ptr<Command> PTOMechanism::makeEngageCommand() {
  return makeStateOnceCommand(true);
}

std::unique_ptr<Command> PTOMechanism::makeDisengageCommand() {
  return makeStateOnceCommand(false);
}

std::unique_ptr<Command> PTOMechanism::makeToggleCommand() {
  return runOnce([this]() { toggle(); });
}

std::unique_ptr<Command> PTOMechanism::makeShiftCommand(bool engaged) {
  return std::make_unique<FunctionalCommand>(
      [this, engaged]() { setEngaged(engaged); },
      [this]() { stop(); },
      [this](bool) { stop(); },
      [this, engaged]() { return isEngaged() == engaged && isShiftSettled(); },
      std::initializer_list<Subsystem*>{this});
}

void PTOMechanism::applyState(const bool& engaged) {
  // Keeps the solenoid matching the state even if something else moved it.
  applySolenoid(engaged);

  // Second half of the guard. m_commanded_volts is only ever non-zero because
  // the owning side passed acceptsWriteFrom(), and a shift zeroes it, so a
  // stale command can never survive a change of owner.
  if (!isShiftSettled()) {
    // brake(), not setVoltage(0). move_voltage(0) would release the brake set
    // in onStateChanged and let a raised lift free-fall through the shift.
    m_motors->stop();
    return;
  }

  if (!isDriveWriteFresh()) {
    // The owner stopped writing. Decay to zero rather than latching its last
    // voltage on the motors forever.
    m_commanded_volts = 0.0;
  }

  if (m_commanded_volts == 0.0) {
    // brake(), not move_voltage(0), for the same reason as above: zero volts
    // is coast, and a raised lift on a coasting PTO falls. stop() applies
    // whatever brake mode the owner configured through motors().
    m_motors->stop();
    return;
  }

  m_motors->setVoltage(m_commanded_volts);
}

void PTOMechanism::onStateChanged(const bool& engaged) {
  // Order matters: kill the torque first, then throw the solenoid. Shifting a
  // loaded PTO is how gearboxes lose teeth.
  m_commanded_volts = 0.0;
  m_motors->stop();

  // Throw the solenoid here rather than waiting for the next periodic() tick.
  // periodic() only runs for a subsystem registered with the CommandScheduler,
  // and the settle clock starts now: if the state said "engaged" while the
  // hardware was still on the drivetrain, the guard would open onto the wrong
  // gearbox.
  applySolenoid(engaged);

  m_last_shift_time = pros::millis() * millisecond;
}

std::shared_ptr<device::MotorGroup> PTOMechanism::makeMotors(const PTOConfig& config) {
  return std::make_shared<device::MotorGroup>(config.motor_ports, config.gearset);
}

std::shared_ptr<device::IPneumatic> PTOMechanism::makePneumatic(const PTOConfig& config) {
  // default_state is the logical state at boot, extended_state maps logical
  // "extended" onto the solenoid's raw value.
  if (config.extender_port != 0) {
    return std::make_shared<device::Pneumatic>(
        config.extender_port,
        config.adi_port,
        config.initial_engaged == config.engaged_when_extended,
        true);
  }

  return std::make_shared<device::Pneumatic>(
      config.adi_port,
      config.initial_engaged == config.engaged_when_extended,
      true);
}

PTOConfig PTOMechanism::makeConfig(std::vector<std::int8_t> motor_ports,
                                   char adi_port,
                                   device::Gearset gearset) {
  PTOConfig config;
  config.motor_ports = std::move(motor_ports);
  config.adi_port = adi_port;
  config.gearset = gearset;
  return config;
}

}  // namespace mechanism
}  // namespace mclib
