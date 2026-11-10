// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: a whole robot's mechanisms kept together. Two mechanisms of your
// own built on the base class, mechanisms built from ports kept as data and
// built in a loop, one MechanismManager that owns every default command and
// registers the lot, and the same mechanisms used from buttons and from an
// autonomous routine. The robot has no drivetrain. Change every port and
// every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::device::DigitalButton;
using mclib::mechanism::ConveyorState;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

// CommandController is the same controller, wrapped so a button can start a
// command. It has no screen calls, so this program keeps both.
CommandController primary(mclib::device::ControllerId::Master);

mclib::device::Pneumatic gate_piston('B');
mclib::device::Motor gate_roller(9, mclib::device::Gearset::Green);
mclib::device::LimitSwitch ball_switch('C');

// Ports kept as data, so the wiring can be read and changed in one place.
// The constructors that take a std::vector of ports are for this.
const std::vector<std::int8_t> intake_ports = {1, -2};
const std::vector<std::int8_t> feeder_ports = {7};

struct ArmPorts {
  std::int8_t motor;
  std::int8_t rotation_sensor;
};
const std::vector<ArmPorts> arm_ports = {{11, 13}, {-12, -14}};

// ---------------------------------------------------------------------------
// A mechanism of your own: derive from StateMechanism. It keeps one state and
// calls applyState() with it on every scheduler pass. Use a class when the
// mechanism needs members of its own, as this one needs a time.
// ---------------------------------------------------------------------------
enum class SweeperState { Stowed, Sweeping };

class Sweeper : public mclib::mechanism::StateMechanism<SweeperState> {
 public:
  Sweeper(char solenoid_port, std::int8_t roller_port)
      : mclib::mechanism::StateMechanism<SweeperState>(SweeperState::Stowed),
        m_arm(solenoid_port),
        m_roller(roller_port, mclib::device::Gearset::Green) {}

  // The scheduler calls this when the robot is disabled. Without it the
  // sweeper would come back in the state it was disabled in.
  void onDisabled() override {
    setState(SweeperState::Stowed);
    m_roller.stop();
  }

 protected:
  // Write the outputs here and nowhere else.
  void applyState(const SweeperState& state) override {
    const bool sweeping = state == SweeperState::Sweeping;
    m_arm.set_value(sweeping);
    const bool arm_has_swung_out =
        mclib::time::hasElapsed(m_state_changed_ms, 250_ms);
    m_roller.setVoltage(sweeping && arm_has_swung_out ? 10.0 : 0.0);
  }

  // Optional. Called once each time setState() is given a different state.
  void onStateChanged(const SweeperState&) override {
    m_state_changed_ms = mclib::time::millis();
  }

 private:
  mclib::device::Pneumatic m_arm;
  mclib::device::Motor m_roller;
  std::uint32_t m_state_changed_ms = 0;
};

Sweeper sweeper('A', 10);

// ---------------------------------------------------------------------------
// The same without a class: MappedMechanism takes the applyState() function
// as an argument. Enough when one function from state to outputs is all
// there is.
// ---------------------------------------------------------------------------
enum class GateState { Closed, Open, Eject };

mclib::mechanism::MappedMechanism<GateState> gate(
    GateState::Closed, [](const GateState& state) {
      gate_piston.set_value(state != GateState::Closed);
      gate_roller.setVoltage(state == GateState::Eject ? -8.0 : 0.0);
    });

// ---------------------------------------------------------------------------
// Library mechanisms, built the short ways. examples/flywheel_and_intake.cpp
// and examples/lift_and_arm.cpp show every setting of these.
// ---------------------------------------------------------------------------
enum class IntakeState { Off, In, Out };

mclib::mechanism::MotorStateMechanism<IntakeState> intake(
    intake_ports, mclib::device::Gearset::Blue,
    // With no off state named after it, this starting state is also the one
    // the intake is put in when the robot is disabled.
    IntakeState::Off,
    [](const IntakeState& state) -> std::vector<double> {
      switch (state) {
        case IntakeState::In: return {12.0};
        case IntakeState::Out: return {-12.0};
        default: return {};
      }
    });

// No sensor, and every voltage and jam setting at its default.
mclib::mechanism::ConveyorMechanism conveyor({3, -4},
                                             mclib::device::Gearset::Green);

mclib::mechanism::MotorSubsystem feeder(feeder_ports,
                                        mclib::device::Gearset::Green);

// ---------------------------------------------------------------------------
// Mechanisms built in a loop. A mechanism cannot be copied or moved: the
// scheduler and the mechanism's own commands hold its address. So a list of
// them is a list of std::unique_ptr, never a std::vector<Arm>.
// ---------------------------------------------------------------------------
enum class ArmPreset { Down, Up };
using Arm = mclib::mechanism::PresetPositionMechanism<ArmPreset>;

// One table for both arms. The table can be braced in place in the
// constructor call, or built beforehand like this one.
const Arm::Table arm_presets{{ArmPreset::Down, 0.0}, {ArmPreset::Up, 95.0}};

std::vector<std::unique_ptr<Arm>> arms;

// The arm keeps its two functions for as long as it lives. They must not
// refer to anything that ends when this function returns, so each holds its
// device through a shared_ptr of its own. A function that captured a local
// by reference would read freed memory on the first scheduler pass.
std::unique_ptr<Arm> makeArm(const ArmPorts& ports) {
  auto motor = std::make_shared<mclib::device::Motor>(
      ports.motor, mclib::device::Gearset::Red);
  auto sensor =
      std::make_shared<mclib::device::Rotation>(ports.rotation_sensor);
  return std::make_unique<Arm>(
      ArmPreset::Down, arm_presets,
      [sensor] { return sensor->getPositionDeg(); },
      [motor](double volts) { motor->setVoltage(volts); },
      mclib::mechanism::PositionMechanismConfig{
          .kp = 0.12, .max_voltage = 10.0, .small_error = 1.5});
}

const double arm_move_limit_ms = 1500.0;

// with: both arms move at once, finished when both have arrived or run out
// of time.
std::unique_ptr<Command> makeArmsCommand(ArmPreset preset) {
  return Command::with(arms[0]->makePresetCommand(preset, arm_move_limit_ms),
                       arms[1]->makePresetCommand(preset, arm_move_limit_ms));
}

// ---------------------------------------------------------------------------
// The manager. The other examples hand each default command to its mechanism
// with setDefaultCommand() and register the mechanisms one by one. A manager
// is the other way: it owns every default command, registers them in one
// call, and finds a mechanism by name. Use one way or the other for a
// mechanism, never both.
//
// Declare the manager after every mechanism it is given, in the same file:
// it ends their commands when it is destroyed.
// ---------------------------------------------------------------------------
mclib::mechanism::MechanismManager mechanisms;

// False from add(): the mechanism or the command is null, the manager already
// has that mechanism, or it already has that name. Nothing is kept.
void addMechanism(Subsystem& mechanism,
                  std::unique_ptr<Command> default_command,
                  const std::string& name) {
  if (!mechanisms.add(&mechanism, std::move(default_command), name)) {
    master.print(0, 0, "NOT ADDED %s", name.c_str());
  }
}

// Set false for a mechanism that is off the robot today.
const bool sweeper_is_fitted = true;

void registerMechanisms() {
  // The default command runs whenever no other command is using the
  // mechanism. These write "off" on every scheduler pass, so a direct call
  // such as gate.setState() is undone on the next pass: change these
  // mechanisms with commands.
  addMechanism(sweeper, sweeper.makeStateCommand(SweeperState::Stowed),
               "sweeper");
  addMechanism(gate, gate.makeStateCommand(GateState::Closed), "gate");
  addMechanism(intake, intake.makeStateCommand(IntakeState::Off), "intake");
  addMechanism(conveyor, conveyor.makeStopCommand(), "conveyor");
  addMechanism(feeder, feeder.makeStopCommand(), "feeder");
  // An entry added with no name is called mechanism0, mechanism1 and so on,
  // by its place in the list.
  addMechanism(*arms[0], arms[0]->idleCommand(), "left_arm");
  addMechanism(*arms[1], arms[1]->idleCommand(), "right_arm");

  // A disabled entry is skipped by registerAll(). Its mechanism is never
  // updated, so its commands run and nothing moves. This is not
  // Subsystem::setEnabled(), which pauses a mechanism that is registered.
  mechanisms.setEnabled("sweeper", sweeper_is_fitted);

  // Registers every enabled entry that is not registered yet, and returns
  // how many. An entry that something else registered first does not count.
  mechanisms.registerAll();
  for (const std::string& name : mechanisms.getNames()) {
    if (mechanisms.isEnabled(name) && !mechanisms.isRegistered(name)) {
      master.print(1, 0, "NOT REG %s", name.c_str());
    }
  }
}

// ---------------------------------------------------------------------------
// Commands. StateMechanism makes four kinds from a state, for your own
// mechanisms and for most of the library's.
// ---------------------------------------------------------------------------

// Holds the state while it runs, and never finishes. When the button is
// released the default command puts the mechanism back.
std::unique_ptr<Command> intake_in = intake.makeStateCommand(IntakeState::In);
std::unique_ptr<Command> sweep =
    sweeper.makeStateCommand(SweeperState::Sweeping);

// Holds the state for a time. The state stays when the time is up: the
// default command is what closes the gate again.
std::unique_ptr<Command> gate_eject =
    gate.makeStateForCommand(GateState::Eject, 400_ms);

// Holds the state until the test is true.
std::unique_ptr<Command> gate_open_until_ball = gate.makeStateUntilCommand(
    GateState::Open, [] { return ball_switch.isPressed(); });

// makeStateOnceCommand() sets the state and finishes on the same pass. Under
// default commands like these it is undone at once. It is for a mechanism
// with an idle default, and for then() steps in a routine: see below.

std::unique_ptr<Command> feed = Command::with(conveyor.makeForwardCommand(),
                                              feeder.makeVoltageCommand(10.0));

// Scheduling a default command by hand interrupts whatever was using its
// mechanism. Done for every entry, it stops the robot's mechanisms.
InstantCommand stop_all(
    [] {
      for (const std::string& name : mechanisms.getNames()) {
        if (mechanisms.isRegistered(name)) {
          mechanisms.getDefaultCommand(name)->schedule();
        }
      }
    },
    {});

// One line per entry on the PROS terminal: its name, whether it is enabled
// and registered, and whether its default command, another command or no
// command has it now.
InstantCommand print_status(
    [] { std::printf("%s", mechanisms.describe().c_str()); }, {});

std::unique_ptr<Command> arms_up;
std::unique_ptr<Command> arms_down;

// ---------------------------------------------------------------------------
// Buttons. Bind from initialize(), not at global scope: a binding reads its
// button when it is made.
// ---------------------------------------------------------------------------
void bindButtons() {
  primary.getTrigger(DigitalButton::R1)->whileTrue(intake_in.get());
  primary.getTrigger(DigitalButton::R2)->whileTrue(feed.get());
  primary.getTrigger(DigitalButton::L1)->whileTrue(sweep.get());
  primary.getTrigger(DigitalButton::A)->onTrue(gate_eject.get());
  primary.getTrigger(DigitalButton::B)->onTrue(gate_open_until_ball.get());
  primary.getTrigger(DigitalButton::Up)->onTrue(arms_up.get());
  primary.getTrigger(DigitalButton::Down)->onTrue(arms_down.get());
  primary.getTrigger(DigitalButton::X)->onTrue(&stop_all);
  primary.getTrigger(DigitalButton::Y)->onTrue(&print_status);
}

// ---------------------------------------------------------------------------
// Autonomous. A Routine runs mechanism commands as steps. It works without a
// chassis for steps like these.
// ---------------------------------------------------------------------------
mclib::auton::Routine score_routine;

// A move that ran out of time has ended, but its loop is still pushing for
// the target. This holds each arm where it stopped.
void stopArmsIfShort() {
  for (const std::unique_ptr<Arm>& arm : arms) {
    if (!arm->atTarget()) {
      master.print(1, 0, "ARM %3.0f/%3.0f", arm->positionValue(),
                   arm->targetValue());
      arm->stop();
    }
  }
}

void buildScoreRoutine() {
  score_routine
      // trigger(): starts the command and moves on. It runs like a command
      // from a button, and the routine ends it when the routine ends.
      .trigger(intake.makeStateCommand(IntakeState::In))
      // then(): waits for the command to finish.
      .then(makeArmsCommand(ArmPreset::Up))
      // A routine keeps every mechanism its then() steps use, from its first
      // step to its last. No other command, and no default command, can touch
      // the arms in between, so atTarget() here still describes that move.
      .runOnce(stopArmsIfShort)
      // The same rule changes what a then() step leaves behind. The gate's
      // default command cannot close it, so this stays open ...
      .then(gate.makeStateOnceCommand(GateState::Open))
      // ... and the conveyor is still running forward after the 1.5 s.
      .then(conveyor.makeStateForCommand(ConveyorState::Forward, 1500_ms))
      // Each has to be put back by a step.
      .then(conveyor.makeStateOnceCommand(ConveyorState::Stopped))
      .then(gate.makeStateOnceCommand(GateState::Closed))
      // The feeder is in no then() step, so the routine does not keep it.
      // Its default command stops it when this ends.
      .trigger(Command::withTimeout(feeder.makeVoltageCommand(10.0), 500_ms))
      .wait(500_ms)
      .then(makeArmsCommand(ArmPreset::Down))
      .runOnce(stopArmsIfShort);
}

void initialize() {
  for (const ArmPorts& ports : arm_ports) {
    arms.push_back(makeArm(ports));
  }
  arms_up = makeArmsCommand(ArmPreset::Up);
  arms_down = makeArmsCommand(ArmPreset::Down);

  registerMechanisms();
  bindButtons();
  buildScoreRoutine();
}

void disabled() {
  CommandScheduler::disable();
}

void competition_initialize() {}

void autonomous() {
  score_routine.runBlocking();

  // The routine has ended its commands and let go of its mechanisms. Their
  // default commands only take over inside CommandScheduler::run(), and the
  // arms are only held from there.
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
