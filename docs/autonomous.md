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
  drive.setDefaultCommand(drive.makeArcadeDriveCommand(master));
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
| `followTrajectory(trajectory, config, timeout)` | follows a trajectory with RAMSETE; see [Motion](motion.md#following-a-trajectory-ramsete) |

Motion options: `withMaxVoltage`, `withMinVoltage`, `withDirection`, `reversed`,
`withoutStop`, `withoutOverturn` and `withEarlyExit`. `followTrajectory` uses
only `withMaxVoltage`, `withoutExit` and `withEarlyExit`.

`until(condition)` ends a motion step early. The routine checks the condition
when the step starts and then every tick. When it returns true, the motion is
cancelled, the drive brakes, and the next step starts. If it is already true
when the step starts, the motion does not run at all:

```cpp
red_safe.moveToPoint(mclib::auton::Point{24_in, 36_in}, 1, 2_s).until([] {
  return chassis.getPose().distanceTo(mclib::Pose2D(24, 36, 0)) < 3;
});
```

`until()` always stops the robot, even with `withoutExit()`. To carry speed
into the next step, use `withoutExit()` and no `until()`.

`until()` only works on a motion step. Written after `runOnce()`, `wait()` or
another step that is not a motion, it does not compile. For a step that only
waits, use `waitUntil()`.

### Commands during a motion

A `trigger()` written after a motion step starts when that motion ends. To
start a command while the robot is still moving, attach it to the motion step:

```cpp
red_safe.followTrajectory(curve, follow, 3_s)
    .atTime(200_ms, conveyor.makeForwardCommand())
    .atDistance(24_in, conveyor.makeStopCommand())
    .atDistanceFromEnd(6_in, clamp.makeExtendCommand());
```

| Method | Starts the command once |
| --- | --- |
| `atTime(time, command)` | the step has run for `time` |
| `atDistance(distance, command)` | the robot has travelled `distance` since the step started |
| `atDistanceFromEnd(distance, command)` | the robot is `distance` from the end of the step |

The command runs alongside the routine, as with `trigger()`, and keeps running
after the step ends. A step can have any number of them. If the step ends
before one comes due, that command never starts: the terminal prints a line,
and `missedEvents()` counts it.

`atDistance` measures the path the odometry pose has covered, so it counts
along a curve. `atDistanceFromEnd` uses the distance the motion steers by:

- `moveToPoint`, `boomerang`: straight-line distance to the target
- `followTrajectory`, `driveTo`: distance left along the path
- `curveCircle`, `curveCircleReverse`: distance left along the outer wheel's
  path. With `useRamseteForArcs()` on and the step not chained, distance left
  along the robot centre's path.

On any other step it is a configuration error.

### Chaining motions

`withoutExit()` chains a motion into the next one. It ends without braking and
leaves the drive on. A chained motion ends when:

- `driveTo`, `turnToAngle`, `swing`, `curveCircle`: the robot passes the
  target.
- `moveToPoint`, `boomerang`: the robot is inside
  `MotionConfig::distance_exit.big_error` of the target (1.5 in by default), or
  it passes the target within 8 in.
- `followTrajectory`: the plan's time runs out.

A chained motion that times out still brakes.

`withEarlyExit(range)` chains the step and ends it sooner: once the robot is
`range` from the end, still moving. The next motion takes over before this one
slows down for its target. Turns take an angle; the rest take a length.

```cpp
red_safe
    .moveToPoint(mclib::auton::Point{24_in, 24_in}, 1, 2_s).withEarlyExit(6_in)
    .moveToPoint(mclib::auton::Point{48_in, 24_in}, 1, 2_s).withEarlyExit(6_in)
    .turnToAngle(90_deg, 1_s).withEarlyExit(10_deg)
    .moveToPoint(mclib::auton::Point{48_in, 48_in}, 1, 2_s);
```

Range is measured the same way as `atDistanceFromEnd`. `withExit()` after it
turns off the early exit and the chaining. `withStop()` after it turns off only
the early exit. These are configuration errors:

- the wrong unit
- a negative range
- `withEarlyExit` on `turnToPoint`, `turnToHeading`, `driveDistance` or
  `wallReset`

A `followTrajectory` step keeps its own copy of the trajectory, so it can be
built from a local:

```cpp
{
  const mclib::path::Trajectory curve = mclib::path::Trajectory::generate(route, limits);
  red_safe.followTrajectory(curve, follow, curve.duration() + 2_s)
      .withMaxVoltage(10_V);
}
```

All motion arguments take units: `24_in`, `90_deg`, `2_s`, `10_V`.

### Time budget

A routine has no idea how long autonomous is unless you tell it. With
`withTimeBudget()` it stops driving in time and still runs the steps that
score.

```cpp
routine.withTimeBudget(15_s)
    .driveTo(24_in, 2_s)
    .moveToPoint(mclib::auton::Point{24_in, 36_in}, 1, 3_s)
    .runOnce([] { claw.open(); }).mustRun();
```

- A motion whose time limit is longer than what is left gets the shorter one.
- When the budget runs out the routine interrupts the running step and stops
  the drive. It skips the remaining steps except those marked `mustRun()`.
- The `mustRun()` steps share the grace window: the last 1.5 s of the budget,
  held back for them, so they run before the field disables the robot. Change
  it with `withGrace()`.
- `withDeadlinePolicy(DeadlinePolicy::Abandon)` runs nothing after the budget
  ends, and holds no time back.
- `remaining()`, `budgetExpired()` and `skippedSteps()` report what happened.

Mark only short actions `mustRun()`. A long drive marked `mustRun()` defeats
the budget. Without `withTimeBudget()` a routine behaves as before.
`mclib::auton::TimeBudget` in `mclib/auton/time_budget.hpp` is the same clock
for code that calls the blocking motions itself;
[examples/background_motion_and_safety.cpp](../examples/background_motion_and_safety.cpp)
shows both.

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
  selector.runSelected();            // false if nothing is selected, or the pick has no function
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
name. Both return `false` if there is no SD card or no matching routine. The
selector never saves by itself. Call `saveSelection()` when the pick changes,
from a loop in `disabled()`, not from `autonomous()` where it costs match
time. [examples/routines_and_selector.cpp](../examples/routines_and_selector.cpp)
has that loop.

A routine exported from the [path planner](simulator.md#path-planner) is a
plain function, such as `runLeftSide()` in `src/autons/left_side.cpp`. Declare
it in `main.cpp` and add it like any other: `selector.add("Left side",
runLeftSide);`.

The list logic is `SelectorModel` in `mclib/auton/selector_model.hpp`, which is
tested on the host. Use `model()` and `setLayout()` to change the grid.
