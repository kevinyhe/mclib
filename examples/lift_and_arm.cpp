// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: mechanisms that go to a position and hold it. A chain lift on a
// PID loop, zeroed against its bottom stop. An arm that moves between named
// angles. A wrist that sends named angles to a PID of your own. The robot has
// no drivetrain. Change every port and every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

// CommandController is the same controller, wrapped so a button can start a
// command. It has no screen calls, so this program keeps both.
CommandController primary(mclib::device::ControllerId::Master);

mclib::device::MotorGroup lift_motors({1, -2}, mclib::device::Gearset::Green);
mclib::device::LimitSwitch lift_bottom_switch('A');

mclib::device::MotorGroup arm_motors({-11, 12}, mclib::device::Gearset::Red);
// On the arm's own shaft, so it reads arm degrees whatever the gearing between
// the motors and the arm.
mclib::device::Rotation arm_sensor(13);

mclib::device::Motor wrist_motor(7, mclib::device::Gearset::Green);
mclib::device::AdiPotentiometer wrist_pot('B');

// ---------------------------------------------------------------------------
// Lift: PositionMechanism. A PID loop between a function that reads a
// position and a function that takes volts.
// ---------------------------------------------------------------------------

// The loop works in the unit the position function returns. This one returns
// inches the carriage has risen, so the gear ratio goes here: a 12-tooth gear
// on the motors turns a 24-tooth gear, on the shaft of a 12-tooth sprocket
// that pulls 4.63 in of chain per turn.
const double lift_motor_turns_per_sprocket_turn = 24.0 / 12.0;
const double lift_inches_per_sprocket_turn = 4.63;

// With no motor answering, the average is NaN. The lift then outputs 0 V and
// hasSensorFault() is true.
double liftHeight() {
  const double sprocket_turns = lift_motors.getAveragePositionDeg() / 360.0 /
                                lift_motor_turns_per_sprocket_turn;
  return sprocket_turns * lift_inches_per_sprocket_turn;
}

const mclib::mechanism::PositionMechanismConfig lift_config{
    // Volts per inch of error. ki is volts per inch summed over the 10 ms
    // passes, kd volts per inch of change between two passes.
    .kp = 1.5,
    .ki = 0.02,
    .kd = 12.0,
    // The most the loop may output. Manual voltages are cut to it as well.
    .max_voltage = 10.0,
    // The lift has arrived once it has stayed within small_error for
    // small_duration_ms, or within big_error for big_duration_ms, while moving
    // less than derivative_tolerance per pass. Inches.
    .small_error = 0.25,
    .big_error = 0.75,
    .small_duration_ms = 80.0,
    .big_duration_ms = 300.0,
    .derivative_tolerance = 0.05,
    // The most the integral may add, in volts. It is what keeps a lift that is
    // stuck short of its target from being driven at max_voltage by ki alone.
    .integral_max_volts = 2.0,
    // True keeps the loop driving after it arrives, so a loaded lift does not
    // drop. It can still sit up to small_error low: the integral is zero that
    // close to the target. False outputs 0 V on arrival and starts again once
    // the lift has dropped past small_error.
    .hold_output = true,
};

mclib::mechanism::PositionMechanism lift(
    liftHeight, [](double volts) { lift_motors.setVoltage(volts); },
    lift_config);

// Inches above the bottom stop.
const double lift_down_height = 0.5;
const double lift_score_height = 16.0;
const double lift_top_height = 24.0;
const double lift_move_limit_ms = 3000.0;

// Finishes when the lift has arrived or the time is up, whichever is first.
// Without the second argument it waits for arrival with no limit.
std::unique_ptr<Command> lift_to_score =
    lift.makeMoveToCommand(lift_score_height, lift_move_limit_ms);
std::unique_ptr<Command> lift_to_bottom =
    lift.makeMoveToCommand(lift_down_height, lift_move_limit_ms);

// A fixed voltage for as long as the command runs. PositionMechanism has no
// travel limits, so these end themselves at the top and at the bottom.
std::unique_ptr<Command> lift_jog_up =
    Command::until(lift.makeManualCommand(6.0),
                   [] { return lift.position() >= lift_top_height; });
std::unique_ptr<Command> lift_jog_down =
    Command::until(lift.makeManualCommand(-4.0),
                   [] { return lift.position() <= lift_down_height; });

// Use this, not lift.atTarget(), after a move command has ended. By then the
// lift's default command has called stop(), which makes the present height
// the target: atTarget() turns true after a move that ran out of time too.
bool liftIsNear(double height) {
  return std::abs(lift.position() - height) <= lift_config.big_error;
}

void reportLiftShort(double wanted_height) {
  if (lift.hasSensorFault()) {
    master.setText(1, 0, "LIFT NO ENCODER");
    return;
  }
  master.print(1, 0, "LIFT %4.1f/%4.1f", lift.position(), wanted_height);
}

// ---------------------------------------------------------------------------
// Lift zero: HomingMechanism. Drives into a hard stop and zeroes the sensor
// there.
// ---------------------------------------------------------------------------
const mclib::mechanism::HomingMechanismConfig lift_homing_config{
    // Negative is down for this lift. Keep it low: it runs into the stop.
    .homing_voltage = -3.0,
    // The stop is found by whichever of three comes first: the limit switch,
    // the current staying at or above current_threshold_amps for
    // current_dwell_ms, or the speed staying under velocity_threshold_rpm for
    // stall_dwell_ms. A threshold of 0 turns that detector off.
    .current_threshold_amps = 1.8,
    .current_dwell_ms = 100.0,
    .velocity_threshold_rpm = 5.0,
    .stall_dwell_ms = 150.0,
    // For this long after the motors start, the current is ignored and a lift
    // that has not moved yet does not count as stalled.
    .startup_grace_ms = 150.0,
    // With no stop found in this time the run has failed.
    .timeout_ms = 3000.0,
    // After zeroing, drive off the stop at this voltage for this long. 0 in
    // either leaves the lift on the stop.
    .backoff_voltage = 2.0,
    .backoff_ms = 150.0,
};

// Homing sends its volts through the lift, not straight to the motors. The
// lift writes its own voltage to them on every scheduler pass, and two writers
// would overwrite each other.
mclib::mechanism::HomingMechanism lift_homing(
    lift_homing_config, [](double volts) { lift.setManualVoltage(volts); });

void connectHomingSensors() {
  // A detector is off until it is given a function to read.
  lift_homing.setLimitSwitch([] { return lift_bottom_switch.isPressed(); });
  // The threshold is in amps. The motors report milliamps.
  lift_homing.setCurrentSource(
      [] { return lift_motors.getAverageCurrentDraw() / 1000.0; });
  lift_homing.setVelocitySource(
      [] { return lift_motors.getAverageActualVelocity(); });
  // Called once, at the stop, before the lift backs off.
  lift_homing.setPositionReset([] { lift_motors.tarePosition(); });
}

// hasFailed(): no stop was found in time, or a current or speed reading was
// not a number.
void reportHomingFailure() {
  if (lift_homing.hasFailed()) {
    master.setText(1, 0, "LIFT NOT HOMED");
  }
}

// race: both halves run and the group ends when homing does. The idle half
// only uses the lift, so that no other lift command, its default command
// included, runs while homing has the motors. makeHomeCommand(ms) adds a time
// limit for this one command, for when it has to be shorter than timeout_ms.
std::unique_ptr<Command> home_lift = Command::andThen(
    Command::race(lift_homing.makeHomeCommand(), lift.idleCommand()),
    lift_homing.runOnce(reportHomingFailure));

// ---------------------------------------------------------------------------
// Arm: PresetPositionMechanism. The same PID loop, with named targets.
// ---------------------------------------------------------------------------
enum class ArmPreset { Stow, Load, Score };

mclib::mechanism::PresetPositionMechanism<ArmPreset> arm(
    ArmPreset::Stow,
    // Degrees up from the stowed angle. next() and previous() step in the
    // order written here.
    {{ArmPreset::Stow, 0.0},
     {ArmPreset::Load, 45.0},
     {ArmPreset::Score, 130.0}},
    [] { return arm_sensor.getPositionDeg(); },
    [](double volts) { arm_motors.setVoltage(volts); },
    // The same config as the lift's. Fields left out keep their defaults.
    mclib::mechanism::PositionMechanismConfig{
        .kp = 0.12, .kd = 0.4, .max_voltage = 10.0, .small_error = 1.5});

const double arm_move_limit_ms = 1500.0;
const double arm_press_down_degrees = 20.0;

// One press, one preset. next() stays on the last preset and previous() on
// the first. makeCycleCommand() goes from the last back to the first.
std::unique_ptr<Command> arm_next = arm.makeNextCommand();
std::unique_ptr<Command> arm_previous = arm.makePreviousCommand();

// A move that runs out of time ends, but the loop goes on pushing for the
// target. Against a jammed arm that is a stalled motor. This holds the arm
// where it stopped and says so.
void stopArmIfShort() {
  if (arm.atTarget()) {
    return;
  }
  // controller() is the PositionMechanism inside the arm. Read it, tune it,
  // never register it.
  if (arm.controller().hasSensorFault()) {
    master.setText(2, 0, "ARM NO SENSOR");
  } else {
    master.print(2, 0, "ARM %3.0f/%3.0f", arm.positionValue(),
                 arm.targetValue());
  }
  arm.stop();
}

// The check is a step of the same command, so nothing else can move the arm
// between the move and the check.
std::unique_ptr<Command> arm_to_score = Command::andThen(
    arm.makePresetCommand(ArmPreset::Score, arm_move_limit_ms),
    arm.runOnce(stopArmIfShort));

// makeManualCommand() has no end step. Under the idle default command this
// arm needs, its voltage would stay on after the button is released. These
// hold the arm instead.
std::unique_ptr<Command> arm_jog_up =
    arm.startEnd([] { arm.setManualVoltage(5.0); }, [] { arm.stop(); });
std::unique_ptr<Command> arm_jog_down =
    arm.startEnd([] { arm.setManualVoltage(-5.0); }, [] { arm.stop(); });

// Holds the arm where it is. arm.setManualVoltage(0.0) lets it drop instead.
// After a stop, getPreset() names the preset nearest to where the arm
// stopped, and next() steps from there.
std::unique_ptr<Command> arm_hold_here = arm.makeStopCommand();

// ---------------------------------------------------------------------------
// Wrist: MultiPositionMechanism. Named positions and no loop of its own. On
// every scheduler pass it hands the number for the present position to a
// function of yours. Here that function runs a PID.
// ---------------------------------------------------------------------------
enum class WristAngle { Tucked, Level, Dump };

// Volts per degree of error.
PID wrist_pid(0.15, 0.0, 0.6);
const double wrist_max_voltage = 8.0;
const double wrist_tolerance_degrees = 3.0;

// A potentiometer that is not answering reads NaN. PID::update() returns 0
// for NaN, so the wrist goes limp.
void driveWristTo(double target_degrees) {
  wrist_pid.setTarget(target_degrees);
  const double volts = wrist_pid.update(wrist_pot.getAngleDeg());
  wrist_motor.setVoltage(
      std::clamp(volts, -wrist_max_voltage, wrist_max_voltage));
}

mclib::mechanism::MultiPositionMechanism<WristAngle> wrist(
    WristAngle::Tucked,
    {{WristAngle::Tucked, 15.0},
     {WristAngle::Level, 100.0},
     {WristAngle::Dump, 190.0}},
    driveWristTo,
    // Sent for a position that is missing from the table. The arm's
    // constructor takes the same argument, after its config.
    15.0);

bool wristIsAt(WristAngle angle) {
  return std::abs(wrist_pot.getAngleDeg() - wrist.setpointFor(angle)) <=
         wrist_tolerance_degrees;
}

void showWristPosition() {
  master.print(0, 0, "WRIST %d OF %d",
               static_cast<int>(wrist.currentIndex()) + 1,
               static_cast<int>(wrist.positionCount()));
}

// makeCycleCommand() goes round the table. makeNextCommand() and
// makePreviousCommand() stop at its ends, as the arm's do.
std::unique_ptr<Command> wrist_step = Command::andThen(
    wrist.makeCycleCommand(), wrist.runOnce(showWristPosition));

// ---------------------------------------------------------------------------
// Registration. A mechanism writes its output from the scheduler, so it does
// nothing until it is registered.
// ---------------------------------------------------------------------------
void registerMechanisms() {
  // The default command runs whenever no other command is using the lift.
  // makeStopCommand() holds the lift where it is: a jog stops on release, and
  // a move that ran out of time stops pushing. makeHoldCommand() would go back
  // to the last target instead.
  //
  // With this default, move the lift with commands. The default command calls
  // stop() when it starts, and that replaces a target set by a direct
  // lift.moveTo().
  lift.setDefaultCommand(lift.makeStopCommand());
  lift.registerSelf();

  // No default command: homing only runs when it is asked to.
  lift_homing.registerSelf();

  // The arm's default has to be idle. makeNextCommand() finishes on the pass
  // it starts, and a default command that called stop() would hold the arm
  // where it is before it had moved. The arm outputs 0 V until its first
  // preset or jog.
  arm.setDefaultCommand(arm.idleCommand());
  arm.registerSelf();

  // The wrist drives to its first position, Tucked, from the first scheduler
  // pass.
  wrist.setDefaultCommand(wrist.idleCommand());
  wrist.registerSelf();
}

// ---------------------------------------------------------------------------
// Buttons. Bind from initialize(), not at global scope: a binding reads its
// button when it is made.
// ---------------------------------------------------------------------------
void bindButtons() {
  primary.getTrigger(DigitalButton::L1)->onTrue(lift_to_score.get());
  primary.getTrigger(DigitalButton::L2)->onTrue(lift_to_bottom.get());
  primary.getTrigger(DigitalButton::Up)->whileTrue(lift_jog_up.get());
  primary.getTrigger(DigitalButton::Down)->whileTrue(lift_jog_down.get());
  // The lift heights count from where the lift sat when the program started,
  // until it is homed.
  primary.getTrigger(DigitalButton::Y)->onTrue(home_lift.get());

  primary.getTrigger(DigitalButton::R1)->onTrue(arm_next.get());
  primary.getTrigger(DigitalButton::R2)->onTrue(arm_previous.get());
  primary.getTrigger(DigitalButton::A)->onTrue(arm_to_score.get());
  primary.getTrigger(DigitalButton::Right)->whileTrue(arm_jog_up.get());
  primary.getTrigger(DigitalButton::Left)->whileTrue(arm_jog_down.get());
  primary.getTrigger(DigitalButton::B)->onTrue(arm_hold_here.get());

  primary.getTrigger(DigitalButton::X)->onTrue(wrist_step.get());
}

// ---------------------------------------------------------------------------
// Autonomous. Mechanisms only move inside CommandScheduler::run(), so code
// that waits for one has to keep calling it.
// ---------------------------------------------------------------------------
void runToEnd(Command& command) {
  command.schedule();
  while (command.scheduled()) {
    CommandScheduler::run();
    pros::delay(10);
  }
}

// False means the time ran out first.
bool waitFor(const std::function<bool()>& done, QTime limit) {
  const std::uint32_t start_ms = mclib::time::millis();
  while (!done()) {
    if (mclib::time::hasElapsed(start_ms, limit)) {
      return false;
    }
    CommandScheduler::run();
    pros::delay(10);
  }
  return true;
}

bool armHasArrived() { return arm.atTarget(); }

WaitCommand let_piece_fall(300_ms);

void scoreOnePiece() {
  runToEnd(*home_lift);
  if (!lift_homing.isHomed()) {
    return;
  }

  // The arm takes its target from a direct call and moves while the lift
  // command runs: each scheduler pass updates every registered mechanism.
  // setPreset() restarts the loop, so call it once. holdPreset() is the one
  // for code that runs on every pass.
  arm.setPreset(ArmPreset::Score);
  runToEnd(*lift_to_score);
  if (!liftIsNear(lift_score_height)) {
    // The lift is already held where it stopped, by its default command. The
    // arm has no such default.
    reportLiftShort(lift_score_height);
    arm.stop();
    return;
  }
  if (!waitFor(armHasArrived, 1500_ms)) {
    stopArmIfShort();
    return;
  }

  // A target that is not a preset. The arm stays on it until the next
  // setPreset() or step.
  arm.moveTo(arm.presetSetpoint() - arm_press_down_degrees);
  if (!waitFor(armHasArrived, 800_ms)) {
    stopArmIfShort();
    return;
  }

  wrist.setPosition(WristAngle::Dump);
  if (!waitFor([] { return wristIsAt(WristAngle::Dump); }, 1_s)) {
    master.setText(2, 0, "WRIST SHORT");
    wrist.setPosition(WristAngle::Tucked);
    return;
  }
  runToEnd(let_piece_fall);

  wrist.setPosition(WristAngle::Tucked);
  arm.setPreset(ArmPreset::Stow);
  runToEnd(*lift_to_bottom);
  if (!liftIsNear(lift_down_height)) {
    reportLiftShort(lift_down_height);
  }
  if (!waitFor(armHasArrived, 1500_ms)) {
    stopArmIfShort();
  }
}

void initialize() {
  // The arm has to be resting stowed when the program starts: this makes
  // that angle zero. The lift is zeroed by homing.
  arm_sensor.resetPosition();

  // PID::update() returns 0 once the error has settled inside its tolerances,
  // and a loaded wrist would drop. This keeps it driving. PositionMechanism
  // sets the same thing on its own PID from hold_output.
  wrist_pid.setHoldOutput(true);

  connectHomingSensors();
  registerMechanisms();
  bindButtons();
}

void disabled() {
  // Ends every command and calls onDisabled() on every registered mechanism.
  // The lift and the arm set their output to 0 V there. The wrist's output is
  // this program's own function, so this program stops that motor.
  CommandScheduler::disable();
  wrist_motor.stop();
}

void competition_initialize() {}

void autonomous() {
  scoreOnePiece();
  // Keep the scheduler running to the end of autonomous. Once it stops, each
  // motor keeps the last voltage it was given and nothing corrects it.
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}

void opcontrol() {
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
