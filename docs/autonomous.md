# Autonomous

## Routines

`mclib::auton::Routine` builds an autonomous from motions and mechanism
commands. Steps run in order.

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

| Step | Behavior |
| --- | --- |
| `then(command)`, `add(command)` | runs the command and waits for it to finish |
| `trigger(command)` | starts the command and moves to the next step immediately |
| `wait(time)` | waits |
| `driveTo(distance, timeout)` | drives a distance relative to the current position |
| `turnToAngle(angle, timeout)` | turns to a field heading |
| `moveToPoint(Point{x, y}, timeout)` | drives to a field position |

Motion options: `withMaxVoltage`, `withMinVoltage`, `withDirection`, `reversed`,
`withoutStop` and `withoutOverturn`.

All motion arguments take units: `24_in`, `90_deg`, `2_s`, `10_V`.

## Mechanisms in routines

Mechanisms are subsystems, so register them in `initialize()` and use their
command factories as steps. See [Mechanisms](mechanisms.md).

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

`makeIndexCommand(timeout_ms)` runs the conveyor until the sensor reads true,
then finishes, so it works with `then()`.

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

`makeStopCommand()` holds the arm in place and runs until interrupted, so it
takes over whenever a command such as `makeMoveToCommand` finishes. The gains in
this example are placeholders; tune them for your robot.

### Custom mechanisms

Subclass `StateMechanism<State>`, or use `MotorStateMechanism<State>` to map an
enum to motor voltages. Both provide `makeStateForCommand(state, 500 *
millisecond)` and `makeStateUntilCommand(state, [] { return done; })`.

## Selector

`mclib/auton/selector.hpp`. Selects the autonomous on the brain screen or
controller.

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

`startPolling()` starts a task that draws the screen and reads touches and
controller buttons. It stops when autonomous begins. `stopPolling()` stops it
early.

The brain screen shows one button per routine in two columns; the selected one
is blue. On the controller, the two bound buttons (Left and Right by default)
step through the list and line 0 shows the selection. Controller text updates
at most every 50 ms.

`saveSelection()` writes the selected routine's name to
`/usd/auton_selection.txt`. `loadSelection()` selects the routine with that
name. Both return `false` if there is no SD card or no matching routine.

The list logic is `SelectorModel` in `mclib/auton/selector_model.hpp`, which is
tested on the host. Use `model()` and `setLayout()` to change the grid.
