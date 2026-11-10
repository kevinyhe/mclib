// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: mechanisms with two or a few fixed positions, and motors shared
// between two jobs. Cylinders that move one at a time, together, or in
// combinations, and two power take-offs. No robot has all of these: each
// section stands by itself, so keep the ones your robot has. Change every
// port and every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::device::DigitalButton;
using mclib::device::Pneumatic;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ. Each mechanism below builds
// or is handed its own solenoids and motors.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

// CommandController is the same controller, wrapped so a button can start a
// command. It has no screen or rumble calls, so this program keeps both.
CommandController primary(mclib::device::ControllerId::Master);
CommandController partner(mclib::device::ControllerId::Partner);

// Smart ports of two 3-wire expanders.
const int expander = 20;
const int second_expander = 21;

mclib::device::Distance goal_sensor(8);

// ---------------------------------------------------------------------------
// Clamp: ToggleMechanism on one solenoid. True is extended.
// ---------------------------------------------------------------------------

// Plumbing goes on the Pneumatic. The third argument says this cylinder is
// out while its solenoid is off, so isExtended() still means "out".
mclib::mechanism::ToggleMechanism clamp(
    std::make_shared<Pneumatic>('A', false, false));

// Every ToggleMechanism command finishes by itself: these on the pass they
// start.
std::unique_ptr<Command> clamp_toggle = clamp.makeToggleCommand();

// A sensor that is not answering gives no distance, and that counts as no
// goal. A test that stayed true through a dead sensor would shut the clamp
// once and never again.
bool goalIsInReach() {
  const std::optional<QLength> distance = goal_sensor.distance();
  return distance.has_value() && *distance < 60_mm;
}

// Shuts the clamp when a goal comes into reach. extend() writes the solenoid
// at once. The action runs when the test turns true, not while it stays true.
mclib::mechanism::AutoTriggerMechanism auto_clamp(
    goalIsInReach, [] { clamp.extend(); },
    mclib::mechanism::AutoTriggerConfig{.debounce_ms = 60.0});

// Opens the clamp and has the watcher leave alone the goal that is still in
// front of the sensor. It shuts again only for a goal that arrives after this
// one has gone.
InstantCommand release_goal(
    [] {
      clamp.retract();
      auto_clamp.disarmUntilReset();
    },
    {&clamp});

// ---------------------------------------------------------------------------
// Doinkers: ToggleMechanism on several solenoids that always move together.
// ---------------------------------------------------------------------------
mclib::mechanism::ToggleMechanism doinkers(
    std::vector<std::shared_ptr<Pneumatic>>{std::make_shared<Pneumatic>('B'),
                                            std::make_shared<Pneumatic>('C')});

std::unique_ptr<Command> doinkers_toggle = doinkers.makeToggleCommand();

// ---------------------------------------------------------------------------
// Hang lock: ToggleMechanism on any function that takes a bool. This one
// writes a plain PROS digital output.
// ---------------------------------------------------------------------------
pros::adi::DigitalOut hang_lock_pin({expander, 'F'});

mclib::mechanism::ToggleMechanism hang_lock(
    [](bool pin_high) { hang_lock_pin.set_value(pin_high); },
    mclib::mechanism::ToggleMechanismConfig{
        // The state it starts in. True is "extended": here, locked.
        .initial_extended = true,
        // The lock is shut while the pin is low, so the function is given the
        // opposite of the state. For functions only. A Pneumatic is told its
        // plumbing when it is built, as the clamp's is.
        .inverted = true,
        // True calls the function once from the constructor, to put the
        // hardware in the starting state. Make it false if the function uses
        // something that is not built yet at that point, such as a global in
        // another file.
        .apply_on_construct = true,
    });

// ---------------------------------------------------------------------------
// Wings: ToggleGroupMechanism. One subsystem, one state per solenoid.
// ---------------------------------------------------------------------------

// An enum can name the channels. Its values are positions in the list below.
enum class Wing : std::size_t { Left = 0, Right = 1 };

mclib::mechanism::ToggleGroupMechanism wings(
    std::vector<std::shared_ptr<Pneumatic>>{std::make_shared<Pneumatic>('D'),
                                            std::make_shared<Pneumatic>('E')});

std::unique_ptr<Command> left_wing_toggle = wings.makeToggleCommand(Wing::Left);
std::unique_ptr<Command> right_wing_toggle =
    wings.makeToggleCommand(Wing::Right);

// makeToggleAllCommand() flips each wing by itself: one out and one in would
// swap. This puts both the same way.
std::unique_ptr<Command> wings_together =
    wings.runOnce([] { wings.setAll(!wings.anySet()); });

// ---------------------------------------------------------------------------
// Flaps: ToggleGroupMechanism on functions, one per channel.
// ---------------------------------------------------------------------------
pros::adi::DigitalOut left_flap_pin({expander, 'G'});
pros::adi::DigitalOut right_flap_pin({expander, 'H'});

mclib::mechanism::ToggleGroupMechanism flaps(
    std::vector<mclib::mechanism::ToggleGroupMechanism::Actuator>{
        [](bool pin_high) { left_flap_pin.set_value(pin_high); },
        [](bool pin_high) { right_flap_pin.set_value(pin_high); }},
    mclib::mechanism::ToggleGroupMechanismConfig{
        // The starting state of every channel initial_states does not reach.
        .initial_extended = false,
        // One starting state per channel, in order.
        .initial_states = {false, true},
        // One entry per channel, for functions only. The right flap's valve is
        // on while its pin is low.
        .inverted = {false, true},
        .apply_on_construct = true,
    });

// A channel is a number when there is no enum for it.
std::unique_ptr<Command> left_flap_toggle = flaps.makeToggleCommand(0);
std::unique_ptr<Command> right_flap_toggle = flaps.makeToggleCommand(1);

// ---------------------------------------------------------------------------
// Intake lift: PneumaticSubsystem from 3-wire ports on the brain.
// ---------------------------------------------------------------------------

// The second argument is the state it starts in, written to the solenoid at
// once. The third says this cylinder is out while its solenoid is off.
mclib::mechanism::PneumaticSubsystem intake_lift(
    {'H'}, /*initial_extended=*/true, /*extended_state=*/false);

std::unique_ptr<Command> intake_lift_toggle = intake_lift.makeToggleCommand();

// ---------------------------------------------------------------------------
// Hang hooks: PneumaticSubsystem from pneumatics built beforehand. These two
// are on an expander, which a list of port letters cannot say.
// ---------------------------------------------------------------------------
mclib::mechanism::PneumaticSubsystem hang_hooks(
    std::vector<std::shared_ptr<Pneumatic>>{
        std::make_shared<Pneumatic>(expander, 'A'),
        std::make_shared<Pneumatic>(expander, 'B')},
    /*initial_extended=*/false);

// ---------------------------------------------------------------------------
// Tilt: DiscreteActuatorMechanism. A few named positions, each one a
// combination of cylinders.
// ---------------------------------------------------------------------------
enum class Tilt { Flat, Angled, Steep };

mclib::mechanism::DiscreteActuatorMechanism<Tilt> tilt(
    Tilt::Flat,
    // One row per position: which cylinders are out, front then back. To add
    // a position, add a row. next() and previous() step in the order written
    // here.
    {{Tilt::Flat, {false, false}},
     {Tilt::Angled, {true, false}},
     {Tilt::Steep, {true, true}}},
    std::vector<std::shared_ptr<Pneumatic>>{std::make_shared<Pneumatic>('F'),
                                            std::make_shared<Pneumatic>('G')});

// next() stays on the last row and previous() on the first.
// makeCycleCommand() goes from the last back to the first.
std::unique_ptr<Command> tilt_up = tilt.makeNextCommand();
std::unique_ptr<Command> tilt_down = tilt.makePreviousCommand();
std::unique_ptr<Command> tilt_flat = tilt.makeDiscreteStateCommand(Tilt::Flat);

// ---------------------------------------------------------------------------
// Gear shifter: DiscreteActuatorMechanism on functions, with its table built
// beforehand.
// ---------------------------------------------------------------------------
enum class Gear { Low, Neutral, High };

pros::adi::DigitalOut low_gear_pin({second_expander, 'A'});
pros::adi::DigitalOut high_gear_pin({second_expander, 'B'});

const mclib::mechanism::DiscreteActuatorMechanism<Gear>::Table gear_table{
    {Gear::Low, {true, false}},
    {Gear::Neutral, {false, false}},
    {Gear::High, {false, true}},
};

mclib::mechanism::DiscreteActuatorMechanism<Gear> shifter(
    Gear::Neutral, gear_table,
    std::vector<mclib::mechanism::DiscreteActuatorMechanism<Gear>::Actuator>{
        [](bool pin_high) { low_gear_pin.set_value(pin_high); },
        [](bool pin_high) { high_gear_pin.set_value(pin_high); }},
    mclib::mechanism::DiscreteActuatorMechanismConfig{
        // One entry per output, for functions only. The high-gear valve is on
        // while its pin is low.
        .inverted = {false, true},
        .apply_on_construct = true,
    });

std::unique_ptr<Command> shifter_step = shifter.makeCycleCommand();

// ---------------------------------------------------------------------------
// Lift PTO: PTOMechanism. One set of motors, switched by a cylinder between
// two jobs. Here "disengaged" runs the intake and "engaged" runs the lift.
// ---------------------------------------------------------------------------
const mclib::mechanism::PTOConfig lift_pto_config{
    .motor_ports = {9, -10},
    .gearset = mclib::device::Gearset::Green,
    // The solenoid that shifts the PTO. extender_port is the smart port of
    // its 3-wire expander, or 0 for a port on the brain.
    .adi_port = 'C',
    .extender_port = expander,
    // True: the engaged side has the motors while the cylinder is out.
    .engaged_when_extended = true,
    // The side that has the motors when the program starts.
    .initial_engaged = false,
    // After each shift the motors are stopped and every write is refused for
    // this long, so the gears are not under power while they move.
    .shift_settle_time = 250_ms,
    // A write lasts this long. If the side that has the motors stops asking,
    // the motors stop. 0_ms keeps the last voltage with no limit.
    .drive_timeout = 100_ms,
};

// PTOMechanism({9, -10}, 'C', gearset) is the short form, for a solenoid on
// the brain and every other setting left at its default.
mclib::mechanism::PTOMechanism lift_pto(lift_pto_config);

const bool engaged_side = true;
const bool disengaged_side = false;

// Asks for `volts` from one side on every scheduler pass. The PTO passes them
// on only while that side has the motors and the shift has settled; any other
// write is dropped and returns false. A lift command left running can not
// drive the intake. driveEngaged(volts) and driveDisengaged(volts) are
// drive(true, volts) and drive(false, volts).
std::unique_ptr<Command> makePtoDriveCommand(
    mclib::mechanism::PTOMechanism& pto, bool side, double volts) {
  return std::make_unique<FunctionalCommand>(
      [&pto, side] {
        // motorsFor() is null unless that side may drive now. The buzz tells
        // the driver the button did nothing.
        if (pto.motorsFor(side) == nullptr) {
          master.rumble(".");
        }
      },
      [&pto, side, volts] { pto.drive(side, volts); },
      // Stops the motors now. Without this they stop when drive_timeout runs
      // out.
      [&pto, side](bool) { pto.drive(side, 0.0); },
      [] { return false; },
      std::initializer_list<Subsystem*>{&pto});
}

std::unique_ptr<Command> intake_in =
    makePtoDriveCommand(lift_pto, disengaged_side, 12.0);
std::unique_ptr<Command> lift_up =
    makePtoDriveCommand(lift_pto, engaged_side, 10.0);
std::unique_ptr<Command> lift_down =
    makePtoDriveCommand(lift_pto, engaged_side, -6.0);

// Shifts, and keeps the PTO to itself until the shift has settled.
// makeEngageCommand(), makeDisengageCommand() and makeToggleCommand() shift
// and finish at once.
std::unique_ptr<Command> shift_to_lift = lift_pto.makeShiftCommand(true);
std::unique_ptr<Command> shift_to_intake = lift_pto.makeShiftCommand(false);

// ---------------------------------------------------------------------------
// Hang PTO: PTOMechanism around hardware built beforehand. Two cylinders
// shift this one, one at each gearbox, and a config has room for one solenoid.
// ---------------------------------------------------------------------------
auto hang_shifters = std::make_shared<mclib::device::PneumaticGroup>(
    std::vector<std::shared_ptr<Pneumatic>>{
        std::make_shared<Pneumatic>(expander, 'D'),
        std::make_shared<Pneumatic>(expander, 'E')});

// nullptr for either part means: build that part from the config. A motor
// group is passed the same way, when something else has to own it. From then
// on only the PTO may set that group's voltage.
mclib::mechanism::PTOMechanism hang_pto(
    nullptr, hang_shifters,
    mclib::mechanism::PTOConfig{.motor_ports = {-18, 19}});

// A "for" command sets its state and then keeps the mechanism for the time
// given, which lets the cylinder move before the next step starts. The state
// stays when the command ends.
std::unique_ptr<Command> prepare_hang = Command::andThen(
    Command::andThen(hang_hooks.makeExtendForCommand(400_ms),
                     hang_lock.makeRetractForCommand(200_ms)),
    hang_pto.makeShiftCommand(true));

// race: both run while the button is held. makeHoldCommand() writes its state
// on every pass and keeps the intake lift to itself, so nothing can drop the
// intake while the winch pulls. Never make it a default command.
std::unique_ptr<Command> winch_in =
    Command::race(makePtoDriveCommand(hang_pto, engaged_side, 12.0),
                  intake_lift.makeHoldCommand(true));

// ---------------------------------------------------------------------------
// Registration. A mechanism writes its output from the scheduler, so it does
// nothing until it is registered.
// ---------------------------------------------------------------------------

// Most commands in this file finish on the pass they start, and a default
// command starts again as soon as one has. So the default has to do nothing:
// one that set a state would undo each button press on the next pass. The
// mechanism goes on writing its last state by itself.
void registerWithIdleDefault(Subsystem& mechanism) {
  mechanism.setDefaultCommand(mechanism.idleCommand());
  mechanism.registerSelf();
}

void registerMechanisms() {
  registerWithIdleDefault(clamp);
  registerWithIdleDefault(auto_clamp);
  registerWithIdleDefault(doinkers);
  registerWithIdleDefault(hang_lock);
  registerWithIdleDefault(wings);
  registerWithIdleDefault(flaps);
  registerWithIdleDefault(intake_lift);
  registerWithIdleDefault(hang_hooks);
  registerWithIdleDefault(tilt);
  registerWithIdleDefault(shifter);
  registerWithIdleDefault(lift_pto);
  registerWithIdleDefault(hang_pto);
}

// ---------------------------------------------------------------------------
// Buttons. Bind from initialize(), not at global scope: a binding reads its
// button when it is made.
// ---------------------------------------------------------------------------
void bindButtons() {
  primary.getTrigger(DigitalButton::L1)->onTrue(clamp_toggle.get());
  primary.getTrigger(DigitalButton::L2)->onTrue(&release_goal);
  primary.getTrigger(DigitalButton::R1)->onTrue(doinkers_toggle.get());
  primary.getTrigger(DigitalButton::Left)->onTrue(left_wing_toggle.get());
  primary.getTrigger(DigitalButton::Right)->onTrue(right_wing_toggle.get());
  primary.getTrigger(DigitalButton::Down)->onTrue(wings_together.get());
  primary.getTrigger(DigitalButton::Y)->onTrue(left_flap_toggle.get());
  primary.getTrigger(DigitalButton::A)->onTrue(right_flap_toggle.get());
  primary.getTrigger(DigitalButton::X)->onTrue(intake_lift_toggle.get());
  primary.getTrigger(DigitalButton::R2)->whileTrue(intake_in.get());

  partner.getTrigger(DigitalButton::R1)->onTrue(tilt_up.get());
  partner.getTrigger(DigitalButton::R2)->onTrue(tilt_down.get());
  partner.getTrigger(DigitalButton::B)->onTrue(tilt_flat.get());
  partner.getTrigger(DigitalButton::Y)->onTrue(shifter_step.get());
  partner.getTrigger(DigitalButton::L1)->onTrue(shift_to_lift.get());
  partner.getTrigger(DigitalButton::L2)->onTrue(shift_to_intake.get());
  partner.getTrigger(DigitalButton::Up)->whileTrue(lift_up.get());
  partner.getTrigger(DigitalButton::Down)->whileTrue(lift_down.get());
  partner.getTrigger(DigitalButton::X)->onTrue(prepare_hang.get());
  partner.getTrigger(DigitalButton::A)->whileTrue(winch_in.get());
}

// ---------------------------------------------------------------------------
// Autonomous: open the wings, take a goal, tip it up.
// ---------------------------------------------------------------------------
mclib::auton::Routine take_goal;

void reportNoGoal() {
  if (!clamp.isExtended()) {
    master.setText(1, 0, "NO GOAL");
  }
}

void buildTakeGoalRoutine() {
  take_goal
      .then(wings.makeSetAllForCommand(true, 300_ms))
      // Finishes when the watcher shuts the clamp, or after 3 s. In a real
      // routine a drive step brings the robot to the goal meanwhile.
      .then(auto_clamp.makeWaitForTriggerCommand(3000.0))
      .runOnce(reportNoGoal)
      .then(tilt.makeDiscreteStateForCommand(Tilt::Steep, 400_ms))
      .then(wings.makeSetAllCommand(false));
}

void initialize() {
  // A table row with the wrong number of values is left out when the
  // mechanism is built, and the position it named then does nothing.
  if (tilt.droppedRowCount() > 0 || shifter.droppedRowCount() > 0) {
    master.setText(0, 0, "BAD TABLE ROW");
  }

  // Once the clamp is shut the goal stays in front of the sensor. Without
  // this, the watcher is ready to act again each time that reading drops out
  // for a moment. With it, only once the clamp has been opened.
  auto_clamp.setRearmCondition([] { return !clamp.isExtended(); });

  // A PTO brakes its motors whenever no volts are asked for. motors() is for
  // settings and readings like this one, never for voltage. Hold keeps a
  // raised lift up.
  lift_pto.motors().setBrakeMode(mclib::device::BrakeMode::Hold);
  hang_pto.motors().setBrakeMode(mclib::device::BrakeMode::Hold);

  registerMechanisms();
  bindButtons();
  buildTakeGoalRoutine();
}

void disabled() {
  CommandScheduler::disable();
}

void competition_initialize() {}

void autonomous() {
  // False means the table has no row for that state.
  if (!shifter.setDiscreteState(Gear::Low)) {
    master.setText(0, 0, "NO LOW GEAR");
  }

  take_goal.runBlocking();

  // The watcher only reads its sensor inside CommandScheduler::run().
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
