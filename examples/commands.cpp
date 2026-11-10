// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: the command framework in mclib/command. Subsystems, every kind of
// command, the two ways to combine commands, triggers and event loops. The
// robot is an intake, a lift and a claw, with no drivetrain. Change every port
// and every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::device::AnalogAxis;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

// CommandController is the same controller, wrapped so a button can start a
// command. It has no screen or rumble calls, so this program keeps both.
CommandController primary(mclib::device::ControllerId::Master);
CommandController partner(mclib::device::ControllerId::Partner);

mclib::device::MotorGroup intake_motors({-20, -21},
                                        mclib::device::Gearset::Blue);
mclib::device::MotorGroup lift_motors({-3, 4}, mclib::device::Gearset::Blue);
mclib::device::Pneumatic claw_piston('G');

// ---------------------------------------------------------------------------
// Subsystems. One class for each part of the robot that only one command may
// drive at a time.
// ---------------------------------------------------------------------------
class Intake : public Subsystem {
 public:
  explicit Intake(mclib::device::MotorGroup& motors) : m_motors(motors) {}

  void spin(QVoltage voltage) { m_motors.setVoltage(voltage); }
  void stop() { m_motors.stop(); }

  // The scheduler calls this when the robot is disabled. A motor keeps its
  // last voltage until something writes a new one.
  void onDisabled() override { stop(); }

 private:
  mclib::device::MotorGroup& m_motors;
};

class Lift : public Subsystem {
 public:
  explicit Lift(mclib::device::MotorGroup& motors) : m_motors(motors) {}

  // The scheduler calls this at the start of every pass, whether or not a
  // command is using the lift.
  void periodic() override {
    m_angle = m_motors.averagePosition();
    m_reporting = m_angle.has_value();
  }

  void onDisabled() override { hold(); }

  void drive(QVoltage voltage) { m_motors.setVoltage(voltage); }
  void hold() { m_motors.stop(); }

  // False from the first read that fails: a motor is unplugged or its port
  // is dead.
  bool isReporting() const { return m_reporting; }

  // With no reading neither of these is ever true, so every command that
  // waits on one also has a time limit.
  bool isAbove(QAngle angle) const { return m_angle && *m_angle >= angle; }
  bool isBelow(QAngle angle) const { return m_angle && *m_angle <= angle; }

 private:
  mclib::device::MotorGroup& m_motors;
  std::optional<QAngle> m_angle;
  bool m_reporting = true;
};

class Claw : public Subsystem {
 public:
  explicit Claw(mclib::device::Pneumatic& piston) : m_piston(piston) {}

  void close() { m_piston.extend(); }
  void open() { m_piston.retract(); }
  void toggle() { m_piston.toggle(); }
  bool isClosed() { return m_piston.get_value(); }

 private:
  mclib::device::Pneumatic& m_piston;
};

Intake intake(intake_motors);
Lift lift(lift_motors);
Claw claw(claw_piston);

// Lift heights, in rotation of the lift motors from where they sat when the
// program started.
const QAngle lift_down_angle = 15_deg;
const QAngle lift_score_angle = 540_deg;
const QAngle lift_hang_angle = 700_deg;
const QTime lift_move_limit = 1500_ms;

bool liftIsUp() { return lift.isAbove(lift_score_angle); }
bool liftIsDown() { return lift.isBelow(lift_down_angle); }

// Stick counts out of 127.
const std::int32_t lift_stick_deadband = 15;

bool driverMovesLift() {
  return std::abs(primary.getAnalog(AnalogAxis::RightY)) > lift_stick_deadband;
}

// ---------------------------------------------------------------------------
// Commands, one of each class. Building a command only stores functions and
// pointers: nothing moves until the scheduler runs it. The scheduler keeps a
// plain pointer to each command it runs and never deletes one, so every
// command here is a global.
// ---------------------------------------------------------------------------

// InstantCommand runs its function once and finishes on the same pass. The
// list after the function names the subsystems the command uses. Scheduling a
// command interrupts whatever else is using one of them.
InstantCommand claw_close([] { claw.close(); }, {&claw});
InstantCommand claw_open([] { claw.open(); }, {&claw});

// With an empty list a command interrupts nothing and nothing interrupts it.
InstantCommand rumble_short([] { master.rumble("."); }, {});
InstantCommand rumble_long([] { master.rumble("-"); }, {});
InstantCommand show_lift_fault(
    [] { master.print(0, 0, "%s NO SENSOR", lift.getName().c_str()); }, {});
InstantCommand clear_lift_fault([] { master.clearLine(0); }, {});

// RunCommand runs its function on every pass and never finishes. This one
// becomes the lift's default command: the right stick, whenever no other
// command has the lift. Zero volts lets the lift fall, so a centred stick
// brakes instead.
RunCommand lift_manual(
    [] {
      if (driverMovesLift()) {
        lift.drive(primary.getAnalog(AnalogAxis::RightY) / 127.0 * 12_V);
      } else {
        lift.hold();
      }
    },
    {&lift});

// FunctionalCommand takes all four steps: start, every pass, end, and the
// test for being finished. `interrupted` is true when something else ended
// the command first. Here that is a time limit or another lift command, and
// either way the lift did not get to its height.
FunctionalCommand lift_raise(
    [] {},
    [] { lift.drive(10_V); },
    [](bool interrupted) {
      lift.hold();
      if (interrupted) {
        master.rumble("-");
      }
    },
    liftIsUp,
    {&lift});

// WaitCommand finishes after a time. WaitUntilCommand finishes when its test
// first returns true. Neither uses a subsystem.
WaitCommand eject_time(500_ms);
WaitCommand alarm_gap(1_s);
WaitUntilCommand driver_takes_lift(driverMovesLift);

// A subsystem builds the common shapes itself, as std::unique_ptr<Command>,
// with itself as the subsystem used:
//   run(f)             f every pass, never finishes
//   runOnce(f)         f once
//   startEnd(f, g)     f at the start, g at the end, never finishes
//   runUntil(f, test)  f every pass until test returns true
//   idleCommand()      nothing, never finishes
std::unique_ptr<Command> claw_toggle = claw.runOnce([] { claw.toggle(); });

// The end step is what stops the motor. Without one, a command that finishes
// inside a group leaves the motor running until the whole group ends.
std::unique_ptr<Command> intakeAt(QVoltage voltage) {
  return intake.startEnd([voltage] { intake.spin(voltage); },
                         [] { intake.stop(); });
}

std::unique_ptr<Command> intake_in = intakeAt(12_V);
std::unique_ptr<Command> intake_out = intakeAt(-12_V);
std::unique_ptr<Command> intake_slow = intakeAt(5_V);

// runUntil has no end step, so run this one by itself only. When it finishes,
// the intake's default command stops the motor on the next pass.
std::unique_ptr<Command> intake_until_lifted = intake.runUntil(
    [] { intake.spin(12_V); }, [] { return lift.isAbove(lift_down_angle); });

// A command class of your own, for what the ready-made ones do not cover.
// CancelIncoming makes this one keep the lift: any other command that needs
// the lift is refused until this one ends. It therefore has to be able to end
// by itself, which the time limit guarantees.
class Hang : public Command {
 public:
  explicit Hang(Lift& lift_to_raise) : m_lift(lift_to_raise) {}

  void initialize() override { m_started_ms = mclib::time::millis(); }
  void execute() override { m_lift.drive(12_V); }
  bool isFinished() override {
    return m_lift.isAbove(lift_hang_angle) ||
           mclib::time::hasElapsed(m_started_ms, 3_s);
  }
  void end(bool /*interrupted*/) override { m_lift.hold(); }

  std::vector<Subsystem*> getRequirements() override { return {&m_lift}; }
  CommandCancelBehavior getCancelBehavior() override {
    return CommandCancelBehavior::CancelIncoming;
  }

 private:
  Lift& m_lift;
  std::uint32_t m_started_ms = 0;
};

Hang hang(lift);

// cancel() ends a command whatever its cancel behaviour, so this stops a hang
// too. The lift's default command takes over on the next pass.
InstantCommand stop_lift_command(
    [] {
      if (const std::optional<Command*> user =
              CommandScheduler::getRequiring(&lift)) {
        (*user)->cancel();
      }
    },
    {});

// ---------------------------------------------------------------------------
// Combining commands, borrowing form. a.andThen(&b) and the other member
// forms return a new command that points at `a` and `b` without owning them.
// That is safe when both are globals, as here. It is not safe on a command
// built in the same statement: `x.andThen(&y)->withTimeout(2_s)` destroys the
// sequence at the semicolon and leaves the timeout pointing at freed memory.
// Store each result before combining it further.
// ---------------------------------------------------------------------------

// withTimeout: ends the command after a time, as interrupted.
std::unique_ptr<Command> lift_raise_limited =
    lift_raise.withTimeout(lift_move_limit);

// until: ends the command when the test returns true.
std::unique_ptr<Command> lift_lowering =
    lift.startEnd([] { lift.drive(-8_V); }, [] { lift.hold(); });
std::unique_ptr<Command> lift_lower_to_bottom =
    lift_lowering->until(liftIsDown);
std::unique_ptr<Command> lift_lower =
    lift_lower_to_bottom->withTimeout(lift_move_limit);

// with: both run, finished when both have finished. The result uses the lift
// and the claw, and so does every group it goes into. Two commands that use
// the same subsystem cannot go in one parallel group: building it stops the
// program.
std::unique_ptr<Command> raise_with_claw_shut =
    lift_raise_limited->with(&claw_close);

// andThen: one, then the other.
std::unique_ptr<Command> alarm_beat = alarm_gap.andThen(&rumble_long);

// ---------------------------------------------------------------------------
// The group classes. Each decorator above builds one of these around two
// commands; the classes take any number.
//
// A command that uses a subsystem can be a member of several groups: those
// groups all use that subsystem, so only one of them runs at a time. A command
// that uses none, such as a wait, would be run by two groups at once. Give
// each group its own.
// ---------------------------------------------------------------------------

// ParallelRaceGroup: all members run, finished when the first one finishes.
// The rest are interrupted.
ParallelRaceGroup eject({intake_out.get(), &eject_time});

// asProxy makes a ProxyCommand: it schedules its target as a separate command
// and finishes when the target does. A group holds every subsystem its members
// use for as long as the group runs. A proxy uses none, so the group below
// leaves the intake free until the eject step starts.
std::unique_ptr<Command> eject_separately = eject.asProxy();

// ScheduleCommand starts another command and finishes at once. It does not
// wait for that command and does not end it: after this step the intake runs
// until something else uses the intake.
ScheduleCommand resume_intake(intake_in.get());

// Sequence: one member after another.
Sequence score({raise_with_claw_shut.get(), eject_separately.get(),
                &resume_intake, lift_lower.get()});

// repeatedly: starts the command again each time it finishes. race: the
// two-member ParallelRaceGroup. Moving the lift stick ends the whole cycle.
std::unique_ptr<Command> score_cycle = score.repeatedly();
std::unique_ptr<Command> score_until_override =
    score_cycle->race(&driver_takes_lift);

// ParallelCommandGroup: all members run, finished when every one has.
ParallelCommandGroup drop_all({lift_lower.get(), &eject, &claw_open});

// RepeatCommand: what repeatedly() builds.
RepeatCommand lift_fault_alarm(alarm_beat.get());

// ConditionalCommand: picks one of two commands each time it starts, and uses
// the subsystems of both.
ConditionalCommand lift_toggle(lift_lower.get(), lift_raise_limited.get(),
                               liftIsUp);

// Cancelling a command that is not running does nothing.
InstantCommand release_lift_toggle([] { lift_toggle.cancel(); }, {});

// ---------------------------------------------------------------------------
// Combining commands, owning form. Command::andThen(a, b) and the other static
// forms take their parts as std::unique_ptr and keep them alive inside the
// result. Use them to build a command in one statement, or in a function that
// returns it.
// ---------------------------------------------------------------------------

// The same move as lift_lower above, without the two extra globals.
std::unique_ptr<Command> moveLiftUntil(QVoltage voltage,
                                       std::function<bool()> arrived) {
  return Command::withTimeout(
      Command::until(lift.startEnd([voltage] { lift.drive(voltage); },
                                   [] { lift.hold(); }),
                     arrived),
      lift_move_limit);
}

std::unique_ptr<Command> scoreOnce() {
  return Command::andThen(
      // The intake keeps hold of the piece for the first part of the raise.
      Command::with(moveLiftUntil(10_V, liftIsUp),
                    Command::withTimeout(intakeAt(4_V), 300_ms)),
      Command::andThen(
          Command::withTimeout(intakeAt(-12_V), 500_ms),
          // The intake reloads for as long as the lift takes to come down.
          Command::race(moveLiftUntil(-8_V, liftIsDown), intakeAt(12_V))));
}

std::unique_ptr<Command> match_routine = scoreOnce();
std::unique_ptr<Command> skills_routine = Command::repeatedly(scoreOnce());

// The claw step is a proxy, so this command uses the lift only. The driver can
// work the claw during the raise without cancelling it.
std::unique_ptr<Command> shut_then_raise = Command::andThen(
    Command::asProxy(claw.runOnce([] { claw.close(); })),
    moveLiftUntil(10_V, liftIsUp));

// Set this from your autonomous selector.
bool run_skills = false;

// A ProxyCommand built from a function picks its target each time it starts.
ProxyCommand selected_routine([]() -> Command* {
  return run_skills ? skills_routine.get() : match_routine.get();
});

// ---------------------------------------------------------------------------
// Subsystem registration and default commands.
// ---------------------------------------------------------------------------
void registerSubsystems() {
  // A registered subsystem gets periodic() on every pass, and its default
  // command runs whenever no other command is using it. The subsystem can own
  // the default command.
  intake.setName("intake");
  intake.setDefaultCommand(intake.run([] { intake.stop(); }));
  intake.registerSelf();

  // registerSelf() is short for this call.
  claw.setName("claw");
  claw.setDefaultCommand(claw.idleCommand());
  CommandScheduler::registerSubsystem(&claw);

  // Or the program owns the default command, and the scheduler is handed a
  // pointer to it.
  lift.setName("lift");
  CommandScheduler::registerSubsystem(&lift, &lift_manual);
}

// ---------------------------------------------------------------------------
// Triggers. A trigger is a test that returns true or false; a binding starts
// or cancels a command when the answer changes. Each binding reads its test
// once when it is made, so bind from initialize(), not at global scope where
// the controller cannot be read yet.
// ---------------------------------------------------------------------------
void bindDriverButtons() {
  // getTrigger() is true while the button is held. The controller keeps one
  // Trigger for each button and hands back the same one every call.
  // Button bindings are only checked during driver control.

  // whileTrue: starts the command on press and cancels it on release.
  primary.getTrigger(DigitalButton::R2)->whileTrue(intake_out.get());
  primary.getTrigger(DigitalButton::B)->whileTrue(score_until_override.get());

  // onTrue: starts the command on press. It runs until it finishes.
  primary.getTrigger(DigitalButton::A)->onTrue(claw_toggle.get());
  primary.getTrigger(DigitalButton::L1)->onTrue(&lift_toggle);
  primary.getTrigger(DigitalButton::Left)->onTrue(&stop_lift_command);
  partner.getTrigger(DigitalButton::A)->onTrue(shut_then_raise.get());

  // onFalse: starts the command on release. Bindings chain: hold Right to
  // open the claw, let go to close it.
  primary.getTrigger(DigitalButton::Right)
      ->onTrue(&claw_open)
      ->onFalse(&claw_close);

  // toggleOnTrue: a press starts the command and the next press cancels it.
  // toggleOnFalse does the same on release.
  primary.getTrigger(DigitalButton::X)->toggleOnTrue(intake_until_lifted.get());
  primary.getTrigger(DigitalButton::Down)->toggleOnFalse(intake_slow.get());

  // A stick has no getTrigger(). A Trigger given the scheduler's teleop loop
  // is checked when the button triggers are: in driver control only. Moving
  // the lift stick takes the lift back from a move L1 started. A Trigger
  // given no loop goes on CommandScheduler::getEventLoop(), which is checked
  // on every pass.
  Trigger lift_stick(driverMovesLift, CommandScheduler::getTeleopEventLoop());
  lift_stick.onTrue(&release_lift_toggle);
}

void bindButtonCombinations() {
  // andOther, orOther and negate make a trigger out of others and point at
  // them without owning them. A trigger that another one points at has to
  // stay alive for as long as the binding does, so these are static.
  //
  // A combined trigger is checked on every scheduler pass, in autonomous too,
  // even when its parts are controller buttons.
  static Trigger* const shift = primary.getTrigger(DigitalButton::L2);
  static Trigger* const intake_button = primary.getTrigger(DigitalButton::R1);
  static const std::unique_ptr<Trigger> no_shift = shift->negate();
  static const std::unique_ptr<Trigger> plain_intake =
      intake_button->andOther(no_shift.get());
  static const std::unique_ptr<Trigger> shifted_intake =
      intake_button->andOther(shift);
  plain_intake->whileTrue(intake_in.get());
  shifted_intake->whileTrue(intake_slow.get());

  static Trigger* const drop_button = primary.getTrigger(DigitalButton::Y);
  static Trigger* const partner_drop_button =
      partner.getTrigger(DigitalButton::Y);
  static const std::unique_ptr<Trigger> either_drop_button =
      drop_button->orOther(partner_drop_button);
  either_drop_button->onTrue(&drop_all);
}

void bindSensorTriggers() {
  // A Trigger built from a test alone is checked on every scheduler pass, in
  // autonomous as well as driver control. A binding keeps its own copy of the
  // test, so these can be locals.
  Trigger lift_reporting([] { return lift.isReporting(); });

  // whileFalse: runs the command from the pass the test turns false until the
  // pass it turns true again. The text goes out first and the first rumble a
  // second later: the controller takes one screen or rumble write about every
  // 50 ms and drops the rest.
  lift_reporting.onFalse(&show_lift_fault)
      ->whileFalse(&lift_fault_alarm)
      ->onTrue(&clear_lift_fault);

  // onChange: starts the command on every change, in either direction.
  Trigger claw_closed([] { return claw.isClosed(); });
  claw_closed.onChange(&rumble_short);

  // A Trigger built from a command is true while that command is scheduled.
  Trigger cycling(score_until_override.get());
  cycling.onFalse(&rumble_short);
}

// An event loop of your own. Its bindings are checked only when the program
// calls poll(), which opcontrol() does for the last 20 seconds of the match.
EventLoop endgame_loop;
const QTime endgame_starts_after = 85_s;

void bindEndgame() {
  // The second argument puts the trigger on that loop.
  Trigger hang_button([] { return primary.getDigital(DigitalButton::Up); },
                      &endgame_loop);
  hang_button.onTrue(&hang);

  // bind() takes any function and runs it on every poll.
  endgame_loop.bind([announced = false]() mutable {
    if (!announced) {
      master.rumble("--");
      announced = true;
    }
  });

  // clear() removes every binding on the loop. Once the hang has started
  // there is nothing left for this loop to do.
  endgame_loop.bind([] {
    if (hang.scheduled()) {
      endgame_loop.clear();
    }
  });
}

void initialize() {
  lift_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  // The lift has to be resting at the bottom when the program starts: this
  // makes that position zero.
  lift_motors.tarePosition();

  registerSubsystems();
  bindDriverButtons();
  bindButtonCombinations();
  bindSensorTriggers();
  bindEndgame();
}

void disabled() {
  // Ends every command as interrupted and calls onDisabled() on every
  // registered subsystem. While the robot is disabled, schedule() does
  // nothing.
  CommandScheduler::disable();
}

void competition_initialize() {}

void autonomous() {
  // Commands only advance inside CommandScheduler::run(). The skills routine
  // repeats and never leaves this loop: PROS stops this task when autonomous
  // is over, and disabled() then ends the commands.
  selected_routine.schedule();
  while (selected_routine.scheduled()) {
    CommandScheduler::run();
    pros::delay(10);
  }
}

void opcontrol() {
  const std::uint32_t driver_start_ms = mclib::time::millis();
  while (true) {
    // One pass: periodic() on every registered subsystem, then the triggers,
    // then every scheduled command, then the default command of each
    // subsystem nothing is using.
    CommandScheduler::run();
    if (mclib::time::hasElapsed(driver_start_ms, endgame_starts_after)) {
      endgame_loop.poll();
    }
    pros::delay(10);
  }
}
