// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: mechanisms that spin. A flywheel held at a speed, intake rollers
// with a voltage for each state, a conveyor that stops a ball at a sensor and
// clears its own jams, a feeder wheel, and watchers that start an action when
// a sensor turns on. The robot has no drivetrain. Change every port and every
// number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ. The intake, the conveyor
// and the feeder build their own motors from port numbers.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

// CommandController is the same controller, wrapped so a button can start a
// command. It has no screen calls, so this program keeps both.
CommandController primary(mclib::device::ControllerId::Master);

mclib::device::MotorGroup flywheel_motors({1, -2},
                                          mclib::device::Gearset::Blue);

// One at the mouth of the intake, one at the top of the conveyor where a ball
// waits for the feeder.
mclib::device::Distance intake_sensor(10);
mclib::device::Distance ready_sensor(11);

// A sensor that is not answering gives no distance, and that counts as no
// ball. The watchers below depend on it: one whose test stayed true through a
// dead sensor would run its action once and never again.
bool seesBall(const mclib::device::Distance& sensor) {
  const std::optional<QLength> distance = sensor.distance();
  return distance.has_value() && *distance < 60_mm;
}

// ---------------------------------------------------------------------------
// Flywheel: VelocityMechanism. Feedforward plus a PID loop, between a function
// that reads motor RPM and a function that takes volts.
// ---------------------------------------------------------------------------
const mclib::mechanism::VelocityMechanismConfig flywheel_config{
    // Volts per RPM of the motor, not of the wheel. ki is per RPM summed over
    // the 10 ms passes, kd per RPM of change between two passes.
    .kp = 0.02,
    .ki = 0.001,
    .kd = 0.0,
    // Feedforward: the volts that hold a speed before any correction. Start
    // from 12 V over the motor's free speed, 600 RPM for a blue cartridge.
    // Gearing does not change it.
    .kv = 12.0 / 600.0,
    .max_voltage = 12.0,
    // Wheel RPM per motor RPM: a 60-tooth gear on the motors drives a 12-tooth
    // gear on the wheel. Every RPM the mechanism takes or returns is wheel
    // RPM. Only the speed function passed to it reports motor RPM.
    .ratio = 60.0 / 12.0,
    // atSpeed() is true once the wheel has stayed within tolerance_rpm of the
    // target for dwell_ms, and false again as soon as it leaves. Wheel RPM.
    .tolerance_rpm = 50.0,
    .dwell_ms = 150.0,
    // The integral only builds within this many wheel RPM of the target, and
    // adds at most integral_max_volts.
    .integral_range_rpm = 300.0,
    .integral_max_volts = 2.0,
};

mclib::mechanism::VelocityMechanism flywheel(
    [] { return flywheel_motors.getAverageActualVelocity(); },
    [](double volts) { flywheel_motors.setVoltage(volts); },
    flywheel_config);

const double shot_rpm = 2400.0;

// Holds the speed until something else uses the flywheel. The output never
// pushes against the spin, so a lower target or a stop coasts down.
std::unique_ptr<Command> flywheel_spin = flywheel.makeSpinCommand(shot_rpm);

// ---------------------------------------------------------------------------
// Intake rollers: MotorStateMechanism. A fixed voltage for each state.
// ---------------------------------------------------------------------------
enum class IntakeState { Off, In, Out };

mclib::mechanism::MotorStateMechanism<IntakeState> intake(
    {5, -6}, mclib::device::Gearset::Green,
    IntakeState::Off,
    // The state the intake is put in when the robot is disabled, so it does
    // not run again by itself when the robot is enabled. Leave this argument
    // out and the starting state is used.
    IntakeState::Off,
    // One voltage for every motor, one for each motor in port order, or none
    // for 0 V.
    [](const IntakeState& state) -> std::vector<double> {
      switch (state) {
        case IntakeState::In: return {12.0, 9.0};
        case IntakeState::Out: return {-12.0};
        default: return {};
      }
    });

// ---------------------------------------------------------------------------
// Conveyor: ConveyorMechanism. Runs forward, back, or until a sensor sees a
// ball, and backs off by itself when it jams.
// ---------------------------------------------------------------------------
const mclib::mechanism::ConveyorConfig conveyor_config{
    // A negative port reverses that motor. Motors that face each other have to
    // be reversed here: jam detection reads the average speed of the group.
    .motor_ports = {7, -8},
    .gearset = mclib::device::Gearset::Green,
    .forward_voltage = 12.0,
    .reverse_voltage = -12.0,
    // Slower, so the ball stops at the sensor and not past it.
    .index_voltage = 7.0,
    // A jam is more than jam_current_amps while turning slower than
    // jam_velocity_rpm, for jam_dwell_ms.
    .jam_current_amps = 2.0,
    .jam_velocity_rpm = 10.0,
    .jam_dwell_ms = 250.0,
    // The conveyor answers a jam by running the other way for unjam_ms. Only
    // the size of unjam_voltage is used, not its sign.
    .unjam_ms = 250.0,
    .unjam_voltage = 8.0,
    // After max_unjam_retries tries in a row it stops and isJammed() turns
    // true. jam_clear_ms of running freely starts the count again.
    .jam_clear_ms = 1000.0,
    .max_unjam_retries = 3,
    // No voltage above is allowed past this.
    .max_voltage = 12.0,
};

mclib::mechanism::ConveyorMechanism conveyor(
    conveyor_config, [] { return seesBall(ready_sensor); });

// Runs until the sensor sees a ball, the conveyor gives up on a jam, or the
// time is up. The only conveyor command that finishes by itself.
std::unique_ptr<Command> index_ball = conveyor.makeIndexCommand(2000.0);
std::unique_ptr<Command> conveyor_forward = conveyor.makeForwardCommand();

// A conveyor that has given up on a jam stays stopped until its state changes
// or clearJam() is called. This command uses no subsystem, so it does not
// interrupt the conveyor command that jammed.
InstantCommand clear_jam(
    [] {
      conveyor.clearJam();
      master.clearLine(1);
    },
    {});

// ---------------------------------------------------------------------------
// Feeder wheel: MotorSubsystem. Motors at whatever voltage the last command
// asked for.
// ---------------------------------------------------------------------------
mclib::mechanism::MotorSubsystem feeder({9}, mclib::device::Gearset::Green);

std::unique_ptr<Command> feed_ball =
    Command::withTimeout(feeder.makeVoltageCommand(12.0), 400_ms);

// ---------------------------------------------------------------------------
// Commands that use more than one of the mechanisms above.
// ---------------------------------------------------------------------------

// Ends when a ball reaches the top of the conveyor. hasObject() is the
// conveyor's own sensor reading, taken on every scheduler pass.
std::unique_ptr<Command> intake_until_staged = intake.makeStateUntilCommand(
    IntakeState::In, [] { return conveyor.hasObject(); });

// The state stays Out when the time is up. The intake's default command is
// what turns it off.
std::unique_ptr<Command> intake_reverse_briefly =
    intake.makeStateForCommand(IntakeState::Out, 300_ms);

// with: all three run together. The percent is of 12 V, from -1 to 1.
std::unique_ptr<Command> spit_out = Command::with(
    Command::with(intake.makeStateCommand(IntakeState::Out),
                  conveyor.makeReverseCommand()),
    feeder.makePercentCommand(-0.5));

// ---------------------------------------------------------------------------
// Watchers: AutoTriggerMechanism. A test and an action. The action runs once
// each time the test turns from false to true, not for as long as it is true.
// ---------------------------------------------------------------------------

// Moves each ball the intake picks up to the top of the conveyor.
mclib::mechanism::AutoTriggerMechanism auto_index(
    [] {
      return intake.getState() == IntakeState::In && seesBall(intake_sensor);
    },
    [] { index_ball->schedule(); },
    mclib::mechanism::AutoTriggerConfig{
        // The test has to stay true this long. One false reading starts the
        // count again.
        .debounce_ms = 40.0,
        // Watching from the start. False waits for setArmed(true).
        .enabled_on_construct = true,
        // False keeps watching after each ball.
        .fire_once = false,
    });

// Fires one ball as soon as the wheel is at speed and a ball is waiting. It
// starts switched off and switches itself off again after one shot.
mclib::mechanism::AutoTriggerMechanism single_shot(
    [] { return flywheel.atSpeed() && conveyor.hasObject(); },
    [] { feed_ball->schedule(); },
    mclib::mechanism::AutoTriggerConfig{.enabled_on_construct = false,
                                        .fire_once = true});

// With no config a watcher starts switched on, has no debounce and keeps
// watching.
mclib::mechanism::AutoTriggerMechanism jam_alarm(
    [] { return conveyor.isJammed(); },
    [] { master.setText(1, 0, "CONVEYOR JAMMED"); });

std::unique_ptr<Command> auto_index_on = auto_index.makeArmCommand();
std::unique_ptr<Command> auto_index_off = auto_index.makeDisarmCommand();

// Switching a watcher on forgets what it saw before: a test that is already
// true runs the action at once.
std::unique_ptr<Command> shoot_when_ready = single_shot.makeArmCommand();
std::unique_ptr<Command> cancel_shot = single_shot.makeDisarmCommand();

// Runs the action now, whatever the test says.
InstantCommand shoot_now([] { single_shot.fireNow(); }, {});

// ---------------------------------------------------------------------------
// Registration. A mechanism writes its output from the scheduler, so it does
// nothing until it is registered.
// ---------------------------------------------------------------------------
void registerMechanisms() {
  // Each default command runs whenever no other command is using its
  // mechanism, and writes "off" on every scheduler pass. A direct call such as
  // flywheel.setTargetRpm() or conveyor.setConveyorState() is undone on the
  // next pass, so change these mechanisms with commands.
  flywheel.setDefaultCommand(flywheel.makeStopCommand());
  flywheel.registerSelf();

  intake.setDefaultCommand(intake.makeStateCommand(IntakeState::Off));
  intake.registerSelf();

  conveyor.setDefaultCommand(conveyor.makeStopCommand());
  conveyor.registerSelf();

  feeder.setDefaultCommand(feeder.makeStopCommand());
  feeder.registerSelf();

  // A watcher reads its test from the scheduler as well.
  auto_index.registerSelf();
  single_shot.registerSelf();
  jam_alarm.registerSelf();
}

// ---------------------------------------------------------------------------
// Buttons. Bind from initialize(), not at global scope: a binding reads its
// button when it is made.
// ---------------------------------------------------------------------------
void bindButtons() {
  primary.getTrigger(DigitalButton::L1)->onTrue(intake_until_staged.get());
  primary.getTrigger(DigitalButton::L2)->whileTrue(spit_out.get());
  primary.getTrigger(DigitalButton::X)->onTrue(intake_reverse_briefly.get());

  primary.getTrigger(DigitalButton::R1)->toggleOnTrue(flywheel_spin.get());
  primary.getTrigger(DigitalButton::R2)->onTrue(shoot_when_ready.get());
  primary.getTrigger(DigitalButton::Left)->onTrue(cancel_shot.get());
  primary.getTrigger(DigitalButton::B)->onTrue(&shoot_now);

  primary.getTrigger(DigitalButton::A)->onTrue(index_ball.get());
  primary.getTrigger(DigitalButton::Right)->whileTrue(conveyor_forward.get());
  primary.getTrigger(DigitalButton::Y)->onTrue(&clear_jam);
  primary.getTrigger(DigitalButton::Up)->onTrue(auto_index_on.get());
  primary.getTrigger(DigitalButton::Down)->onTrue(auto_index_off.get());
}

// ---------------------------------------------------------------------------
// Autonomous: pick up one ball and shoot it.
// ---------------------------------------------------------------------------
mclib::auton::Routine shoot_one_ball;

void reportMissedShot() {
  if (single_shot.hasFired()) {
    return;
  }
  if (flywheel.hasSensorFault()) {
    master.setText(1, 0, "FLYWHEEL NO RPM");
  } else if (flywheel.isSpinningUp()) {
    master.print(1, 0, "FLY %4.0f/%4.0f", flywheel.getCurrentRpm(),
                 flywheel.getTargetRpm());
  } else {
    master.setText(1, 0, "NO BALL STAGED");
  }
}

void buildShootRoutine() {
  shoot_one_ball
      // trigger(): starts the command and moves on. It runs until the routine
      // ends. auto_index moves the ball up when it arrives.
      .trigger(intake.makeStateCommand(IntakeState::In))
      // then(): waits for the command to finish. This one finishes at speed,
      // or after 3 s, and leaves the wheel spinning. A routine keeps every
      // mechanism its then() steps use until it ends, so the flywheel's
      // default command cannot stop the wheel between steps.
      .then(flywheel.makeSpinUpCommand(shot_rpm, 3000.0))
      // Gives up after 3 s and goes on to the next step all the same.
      .waitUntil([] { return conveyor.hasObject(); }, 3_s)
      // Switches the watcher on, waits for its action to run, and switches it
      // off again. The time limit matters: without a ball the test never
      // turns true.
      .then(single_shot.makeWaitForTriggerCommand(2000.0))
      .runOnce(reportMissedShot)
      // Time for feed_ball to finish before the routine lets go of the wheel.
      .wait(500_ms);
  // No then() step uses the conveyor or the feeder. The watchers start
  // commands on those two, and a command that takes a mechanism the routine
  // is keeping ends the routine.
}

void initialize() {
  // A ratio that is zero, negative or not a number. The flywheel would output
  // 0 V whatever it was asked for.
  if (flywheel.hasConfigurationError()) {
    master.setText(0, 0, "FLYWHEEL RATIO");
  }

  registerMechanisms();
  bindButtons();
  buildShootRoutine();
}

void disabled() {
  CommandScheduler::disable();
}

void competition_initialize() {}

void autonomous() {
  // hasFired() counts from the last reset().
  single_shot.reset();
  shoot_one_ball.runBlocking();

  // The routine has ended its commands, but the flywheel and the intake keep
  // their last voltage until their default commands run. Those only run inside
  // CommandScheduler::run().
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
