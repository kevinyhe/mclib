# Autonomous routines and the selector

Building an autonomous routine, budgeting its time, and picking one on the brain screen.

[Documentation index](README.md) · [Project README](../README.md)

## Autonomous Routines

Use `mclib::auton::Routine` to compose a whole autonomous as an
owned sequence. Motion steps run in order, and mechanism commands can be
triggered between motions.

```cpp
mclib::mechanism::ConveyorMechanism conveyor({
    .motor_ports = {-20, -21},
    .gearset = mclib::device::Gearset::Blue,
});

mclib::auton::Routine red_safe(drive);

void initialize() {
  drive.setName("drive");
  drive.setDefaultCommand(drive.makeArcadeDriveCommand(controller));
  drive.registerSelf();

  conveyor.setName("conveyor");
  conveyor.setDefaultCommand(conveyor.makeStopCommand());
  conveyor.registerSelf();

  red_safe
      .driveTo(24_in, 2_s)
          .withMaxVoltage(10_V)
          .withMinVoltage(2_V)
      .trigger(conveyor.makeForwardCommand())
      .wait(300 * millisecond)
      .turnToAngle(90_deg, 1500_ms)
          .withMaxVoltage(8_V)
      .trigger(conveyor.makeStopCommand())
      .moveToPoint(mclib::auton::Point{48_in, 24_in}, 2500_ms)
          .withDirection(1)
          .withMaxVoltage(9_V)
          .withoutOverturn();
}

void autonomous() {
  red_safe.runBlocking();
}
```

Use `then(command)` or `add(command)` when the routine should wait for a
command to finish before moving on. Use `trigger(command)` when the command
should be scheduled and the routine should immediately continue to the next
step. Motion steps support fluent options such as `withMaxVoltage`,
`withMinVoltage`, `withDirection`, `reversed`, `withoutStop`, and
`withoutOverturn`.

Every motion parameter carries its unit in its type: `24_in`, `90_deg`,
`2_s`, `10_V`. `driveTo` always means a relative distance -- it used to be
overloaded so that `driveTo(24, 1000)` drove 24 inches while
`driveTo(24, 36, 1000)` drove to the field point `(24, 36)`, two different
motions told apart only by argument count. Say `moveToPoint(Point{...}, t)`
for a field point.

Mechanisms are built from generic stateful subsystem templates. `StateMechanism<T>`
owns the command-facing state machine, while concrete mechanisms decide how that
state is applied to device wrappers.

```cpp
// #include "mclib/device/line.hpp". Line is in the global namespace.
Line line_sensor('A');

mclib::mechanism::ConveyorMechanism conveyor(
    {
        .motor_ports = {-20, -21},
        .gearset = mclib::device::Gearset::Blue,
        .forward_voltage = 12.0,
        .reverse_voltage = -12.0,
        .index_voltage = 8.0,
        // MotorGroup reports current in milliamps; ConveyorConfig wants amps,
        // and ConveyorMechanism does the conversion internally.
        .jam_current_amps = 2.0,
    },
    [] { return line_sensor.get(); });

void initialize() {
  conveyor.setName("conveyor");
  conveyor.setDefaultCommand(conveyor.makeStopCommand());
  conveyor.registerSelf();
}
```

`ConveyorMechanism` watches average current draw and average velocity while it
runs. A sustained stall triggers a bounded number of reversing unjam attempts;
when those run out it stops the motors and latches `isJammed()`.
`makeIndexCommand(timeout_ms)` runs at `index_voltage` until the sensor gate
reads true, then stops, so it can be sequenced with `then()` in a routine.

The same pattern works for other mechanisms:

```cpp
mclib::device::MotorGroup arm_motors({-3, 4}, mclib::device::Gearset::Blue);
mclib::device::Rotation arm_sensor(8);
mclib::mechanism::PneumaticSubsystem wings({'E', 'F'});

mclib::mechanism::PositionMechanism arm(
    [] { return arm_sensor.getPositionDeg(); },
    [](double volts) { arm_motors.setVoltage(volts); },
    mclib::mechanism::PositionMechanismConfig{
        .kp = 0.09, .max_voltage = 10.0, .small_error = 1.5});

void initialize() {
  arm.setName("arm");
  arm.setDefaultCommand(arm.makeStopCommand());
  arm.registerSelf();

  wings.setName("wings");
  wings.setDefaultCommand(wings.idleCommand());
  wings.registerSelf();
}

std::unique_ptr<Command> makeLowCommand() {
  return arm.makeMoveToCommand(45.0, 2000.0);
}
```

The lambdas take no capture because everything is at file scope. Inside a
function you can capture with `[&]`, but the captured devices must outlive the
mechanism: it stores the lambdas and calls them every `periodic()` tick, so
capturing function-local devices by reference leaves it calling into destroyed
objects. Make them members, statics, or file-scope objects.

The tuning above is example tuning. `kp`, `max_voltage` and the error/duration
tolerances are per-robot; start from the `PositionMechanismConfig` defaults.

`makeStopCommand()` is the right default here: it holds the present position
instead of coasting, and it runs until interrupted, so it takes over again the
moment a terminating command such as `makeMoveToCommand` releases the arm.

For custom mechanisms, subclass `StateMechanism<State>` or use
`MotorStateMechanism<State>` when an enum maps to one or more motor voltages.
Timed and condition-based commands come from the same generic helpers:
`makeStateForCommand(state, 500 * millisecond)` and
`makeStateUntilCommand(state, [] { return done; })`.

### Migrating from `Intake`

`Intake`, `IntakeState`, and `IntakeConfig` were removed. `ConveyorMechanism`
does the same job without being named after one robot's mechanism.

| `Intake` | `ConveyorMechanism` |
| --- | --- |
| `IntakeConfig::bottom_port`, `top_port` | `ConveyorConfig::motor_ports` |
| `IntakeConfig::gearset` | `ConveyorConfig::gearset` |
| `index_voltage` (default 12.0) | `index_voltage` (default 8.0, so set it explicitly if you relied on the old value) |
| `score_voltage` | `forward_voltage` |
| `reverse_voltage` | `reverse_voltage` |
| `IntakeState::Disabled` | `ConveyorState::Stopped` |
| `IntakeState::Index` | `ConveyorState::IndexToSensor` |
| `IntakeState::Score` | `ConveyorState::Forward` |
| `IntakeState::Reverse` | `ConveyorState::Reverse` |
| `disable()` / `makeDisableCommand()` | `stop()` / `makeStopCommand()` |
| `makeScoreCommand()` | `makeForwardCommand()` |
| `makeReverseCommand()` | `makeReverseCommand()` |
| `makeIndexCommand()` | `makeIndexCommand(timeout_ms)` |
| `setState()` / `getState()` | `setState()` / `getState()`, or the typed `setConveyorState()` / `getConveyorState()` |

Two behaviour differences worth knowing before you swap:

**Indexing needs a sensor.** `IntakeState::Index` just ran the motor and never
stopped on its own. `ConveyorState::IndexToSensor` runs at `index_voltage` while
the sensor gate reads false and holds at zero volts while it reads true, and
`makeIndexCommand` finishes once the gate latches. With no gate injected,
`IndexToSensor` behaves like `Stopped` and `makeIndexCommand` finishes
immediately. If you want the old unconditional behaviour, use
`ConveyorState::Forward`.

**Per-motor voltages are gone.** `Intake` drove its two motors at different
voltages per state: `Index` ran the bottom motor only, while `Score` and
`Reverse` ran both. `ConveyorMechanism` drives all of its motors as one
`device::MotorGroup` at a single voltage, so it cannot express that split. Model
a two-stage path as two `ConveyorMechanism` instances, one per stage:

```cpp
mclib::mechanism::ConveyorMechanism bottom(
    {.motor_ports = {-20}}, [] { return line_sensor.get(); });
mclib::mechanism::ConveyorMechanism top({.motor_ports = {-21}});

// ParallelCommandGroup stores raw Command*, so the two stage commands have to
// outlive the group that points at them.
std::unique_ptr<Command> bottom_index;
std::unique_ptr<Command> bottom_forward;
std::unique_ptr<Command> top_forward;
std::unique_ptr<Command> score;

void initialize() {
  bottom.setName("conveyor_bottom");
  bottom.setDefaultCommand(bottom.makeStopCommand());
  bottom.registerSelf();

  top.setName("conveyor_top");
  top.setDefaultCommand(top.makeStopCommand());
  top.registerSelf();

  // Old IntakeState::Index. Bottom stage only, gated on the sensor. The top
  // stage keeps its stop default while this runs.
  bottom_index =
      bottom.makeStateCommand(mclib::mechanism::ConveyorState::IndexToSensor);

  // Old IntakeState::Score. Both stages at once.
  bottom_forward = bottom.makeForwardCommand();
  top_forward = top.makeForwardCommand();
  score = std::make_unique<ParallelCommandGroup>(
      std::initializer_list<Command*>{bottom_forward.get(), top_forward.get()});
}

void opcontrol() {
  bottom_index->schedule();  // index
  score->schedule();         // score
}
```

Drive the stages with scheduled commands, not with bare `setConveyorState`
calls. Both stop defaults re-assert `Stopped` every tick, so a direct state
write is undone on the next `CommandScheduler::run()` unless a command holds
the requirement.

Two instances also give each stage its own voltages, its own sensor gate, and
its own jam detection, which one shared `MotorGroup` average could never do.

## Autonomous selector

`mclib/auton/selector.hpp`. Pick which autonomous runs, on the brain screen
or the controller, without reflashing.

```cpp
#include "mclib/auton/selector.hpp"

mclib::auton::AutonSelector selector;
mclib::device::Controller master;

void initialize() {
  selector.add("Left side", leftSideAuton);
  selector.add("Right side", rightSideAuton);
  selector.add("Skills", skillsAuton);
  selector.add("Do nothing", nullptr);

  selector.bindController(master);   // Left / Right step the pick; line 0 shows it
  selector.loadSelection();          // last saved pick from /usd/, if there is one
  selector.startPolling();           // task: draw, touch, buttons, every 50 ms
}

void autonomous() {
  selector.runSelected();            // false if nothing is selected
}
```

`startPolling()` runs a `pros::Task` that redraws the buttons, takes touches,
and watches the two controller buttons. The task stops itself the moment
`pros::competition::is_autonomous()` turns true, so it never fights the
routine for the screen or the controller. `stopPolling()` joins it early if
you want to.

The brain screen shows one button per entry in a two-column grid below the
PROS status bar; the selected one is blue. Touch a button to pick it. On the
controller, the two bound buttons (Left and Right by default; any two
`DigitalButton`s) step back and forward through the list with wraparound,
and line 0 shows the current name. Controller text is written at most every
50 ms because the V5 controller drops faster updates.

`saveSelection()` writes the selected entry's *name* to
`/usd/auton_selection.txt`; `loadSelection()` reads it back and selects that
entry. Saving the name and not the index means adding or reordering routines
in code never changes which one the saved file picks. A name that is no longer
in the list leaves the selection alone and returns false. Nothing here needs
the SD card; without one, both just return false.

The list logic - add, select, wraparound, which button a touch lands on - is
`SelectorModel` in `mclib/auton/selector_model.hpp`. It has no PROS include,
so `tests/selector_model_test.cpp` checks it on the host. `AutonSelector`
adds only the screen, controller, task and file calls, and exposes the model
through `model()` and `setLayout()` if you want a different grid.
