<!-- // mclib -->
# mclib

`mclib` is a PROS library project for VEX V5. Its layout follows the same
basic pattern as LemLib: public headers live in `include/mclib/`,
implementations live in `src/mclib/`, and the repo is also a buildable PROS
project with `Makefile`, `common.mk`, `project.pros`, `firmware/`, `include/`,
and `src/`.

This generated PROS 4 project builds as `gnu++20` because the current PROS
kernel headers use C++20 constructs.

The starter API uses Eigen for fixed-size linear algebra only. Avoid dynamic
Eigen types such as `MatrixXd` and `VectorXd` on the V5 brain.

## Layout

```text
mclib/
  firmware/
  include/
    Eigen/
    mclib/
      auton/
      chassis/
      command/
      device/
      mechanism/
      snapshot/
      units/
      config.hpp
      control/
      control.hpp
      math.hpp
      mclib.hpp
      pid.hpp
      utils.hpp
    main.h
  src/
    mclib/
      auton/
      chassis/
      device/
      mechanism/
      snapshot/
      config.cpp
      control/
      math.cpp
      pid.cpp
      utils.cpp
    main.cpp
  Makefile
  common.mk
  project.pros
```

## Eigen

Eigen is header-only here. Keep the headers local so this path exists:

```text
include/Eigen/Core
```

You can also use this alternate layout:

```text
eigen/Eigen/Core
```

The `Makefile` already adds `eigen/` as an extra include directory. Do not add
Eigen files to the compiled source list.

## Building

Build the PROS project:

```sh
pros make
```

Build the PROS library archive:

```sh
pros make library
```

Create a PROS template package:

```sh
pros make template
```

## Usage

```cpp
#include "main.h"
#include "mclib/mclib.hpp"

void autonomous() {
  mclib::Pose2D start{0.0, 0.0, 0.0};
  mclib::Pose2D delta{12.0, 6.0, 3.5};

  mclib::Pose2D target = start + delta;
  double heading = mclib::wrapAngle(target.theta);
  double distance = target.distanceTo(start);
  mclib::Vec2 field_delta = target.translation() - start.translation();

  (void)heading;
  (void)distance;
  (void)field_delta;
}
```

## Chassis And Commands

```cpp
#include "main.h"
#include "mclib/mclib.hpp"

mclib::device::Controller controller(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);
mclib::Chassis chassis(
    {-11, 13, 14},
    {-16, 17, -18},
    mclib::device::Gearset::Blue,
    {.wheel_diameter_in = 2.75, .track_width_in = 11.375},
    imu);

mclib::ChassisControllerConfig drive_config{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
    .max_voltage = 12.0,
    .min_voltage = 1.0,
};

mclib::ChassisController drive(chassis, drive_config);
std::unique_ptr<Command> default_drive;
std::unique_ptr<Command> drive_forward;
std::unique_ptr<Command> turn_to_goal;

void initialize() {
  default_drive = drive.makeArcadeDriveCommand(controller);
  drive_forward = drive.makeDriveDistanceCommand(24.0, 3000.0);
  turn_to_goal = drive.makeTurnToAngleCommand(90.0, 1500.0);

  CommandScheduler::registerSubsystem(&drive, default_drive.get());
}

void autonomous() {
  drive_forward->schedule();
  while (drive_forward->scheduled()) {
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
```

`ChassisController` also exposes command factories for the sorted legacy motion
routines from `control/`: `makeDriveToCommand`, `makeCurveCircleCommand`,
`makeSwingCommand`, `makeWallResetCommand`, `makeTurnToPointCommand`,
`makeMoveToPointCommand`, and `makeBoomerangCommand`.

## Autonomous Routines

Use `mclib::auton::Routine` to compose a whole autonomous as an
owned sequence. Motion steps run in order, and mechanism commands can be
triggered between motions.

```cpp
mclib::auton::Routine red_safe(drive);

void initialize() {
  default_drive = drive.makeArcadeDriveCommand(controller);
  intake_idle = intake.makeDisableCommand();

  CommandScheduler::registerSubsystem(&drive, default_drive.get());
  CommandScheduler::registerSubsystem(&intake, intake_idle.get());

  red_safe
      .driveTo(24.0, 2000.0)
          .withMaxSpeed(10.0)
          .withMinSpeed(2.0)
      .trigger(intake.makeIndexCommand())
      .wait(300 * millisecond)
      .turnToAngle(90.0, 1500.0)
          .withMaxSpeed(8.0)
      .trigger(intake.makeDisableCommand())
      .driveTo(48.0, 24.0, 2500.0)
          .withDirection(1)
          .withMaxSpeed(9.0)
          .withoutOverturn();
}

void autonomous() {
  red_safe.runBlocking();
}
```

Use `then(command)` or `add(command)` when the routine should wait for a
command to finish before moving on. Use `trigger(command)` when the command
should be scheduled and the routine should immediately continue to the next
step. Motion steps support fluent options such as `withMaxSpeed`,
`withMinSpeed`, `withDirection`, `reversed`, `withoutStop`, and
`withoutOverturn`.

Mechanisms are built from generic stateful subsystem templates. `StateMechanism<T>`
owns the command-facing state machine, while concrete mechanisms decide how that
state is applied to device wrappers.

```cpp
mclib::mechanism::Intake intake({
    .bottom_port = -20,
    .top_port = -21,
    .gearset = mclib::device::Gearset::Blue,
});

std::unique_ptr<Command> intake_idle;
std::unique_ptr<Command> intake_score;

void initialize() {
  intake_idle = intake.makeDisableCommand();
  intake_score = intake.makeScoreCommand();

  CommandScheduler::registerSubsystem(&intake, intake_idle.get());
}
```

The same pattern works for other mechanisms:

```cpp
mclib::mechanism::Arm arm({-3, 4}, 8);
mclib::mechanism::PneumaticSubsystem wings({'E', 'F'});

std::unique_ptr<Command> arm_low;
std::unique_ptr<Command> arm_stop;
std::unique_ptr<Command> wings_out;
std::unique_ptr<Command> wings_in;

void initialize() {
  arm_low = arm.makeMoveToCommand(45.0, 2000.0);
  arm_stop = arm.makeStopCommand();
  wings_out = wings.makeExtendCommand();
  wings_in = wings.makeRetractCommand();

  CommandScheduler::registerSubsystem(&arm, arm_stop.get());
  CommandScheduler::registerSubsystem(&wings, wings_in.get());
}
```

For custom mechanisms, subclass `StateMechanism<State>` or use
`MotorStateMechanism<State>` when an enum maps to one or more motor voltages.
Timed and condition-based commands come from the same generic helpers:
`makeStateForCommand(state, 500 * millisecond)` and
`makeStateUntilCommand(state, [] { return done; })`.

## Core Math API

```cpp
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

double clamp(double val, double min, double max);
double wrapAngle(double rad);

struct Pose2D {
  double x;
  double y;
  double theta;

  Pose2D operator+(const Pose2D& other) const;
  double distanceTo(const Pose2D& other) const;
  Vec2 translation() const;
  Vec3 vector() const;
};

Mat2 rotationMatrix(double rad);
Vec2 rotate(const Vec2& vec, double rad);
```

Other modules are split into matching header/source pairs:

- `auton/*.hpp` / `auton/*.cpp`: owning autonomous routine builder
- `chassis/*.hpp` / `chassis/*.cpp`: chassis hardware and PID controller
- `config.hpp` / `config.cpp`: robot hardware and tuning globals
- `control/*.hpp` / `control/*.cpp`: sorted chassis I/O, scaling, motion, odometry, and shared state helpers
- `pid.hpp` / `pid.cpp`: PID controller
- `utils.hpp` / `utils.cpp`: angle and geometry utilities
- `device/*.hpp` / `device/*.cpp`: the only place that calls PROS motor, controller, pneumatic, and sensor APIs directly
- `mechanism/*.hpp` / `mechanism/*.cpp`: generic stateful mechanisms, plus intake, arm, motor, and pneumatic subsystem examples
- `snapshot/*.hpp` / `snapshot/*.cpp`: distance-sensor pose snapshot helpers

## Subsystem lifecycle

Command factories return `std::unique_ptr<Command>`, but the scheduler stores raw
`Command*`. That mismatch is easy to get wrong:

```cpp
// BROKEN. The unique_ptr dies at the end of the statement, so the scheduler is
// left holding a dangling pointer.
intake.makeIndexCommand()->schedule();
```

The subsystem itself is the owner. Hand it the default command with
`setDefaultCommand` and register with `registerSelf`:

```cpp
Intake intake{...};
Arm arm{...};

void initialize() {
  intake.setName("intake");
  intake.setDefaultCommand(intake.makeDisableCommand());
  intake.registerSelf();

  arm.setName("arm");
  arm.setDefaultCommand(arm.makeStopCommand());
  arm.registerSelf();
}
```

No global `std::unique_ptr` variables and no lifetime bookkeeping. The subsystem
keeps the default command alive for as long as it is alive.

The older two-argument form still works, and is still the right tool when the
default command must live somewhere other than the subsystem. You keep ownership,
so the command has to outlive the registration:

```cpp
std::unique_ptr<Command> intake_idle;

void initialize() {
  intake_idle = intake.makeDisableCommand();
  CommandScheduler::registerSubsystem(&intake, intake_idle.get());
}
```

Commands that are not defaults still need an owner. Store the `unique_ptr`
somewhere that outlives the scheduling, then schedule the raw pointer:

```cpp
std::unique_ptr<Command> index;

void opcontrol() {
  index = intake.makeIndexCommand();
  index->schedule();
}
```

### Subsystem API

| Method | What it does |
| --- | --- |
| `setDefaultCommand(std::unique_ptr<Command>)` | Take ownership of the default command. Destroys any previous one. |
| `getDefaultCommand()` | Non owning `Command*`, or `nullptr` if none was set. |
| `registerSelf()` | Register with the `CommandScheduler` using the stored default command. |
| `setName(std::string)` / `getName()` | Human readable name, useful for logging. |
| `setEnabled(bool)` / `isEnabled()` | A disabled subsystem skips `periodic()`. It does not stop the hardware, see below. |
| `runPeriodic()` | Non virtual. Called by the scheduler, checks `isEnabled()` and then calls the virtual `periodic()`. Override `periodic()`, not this. |

`setEnabled(false)` parks a subsystem without unregistering it. Commands can still
be scheduled against it, they just have no effect until it is enabled again.
Because `runPeriodic()` is non virtual and does the check, this works for
subclasses that override `periodic()`, such as `StateMechanism` and
`ChassisController`.

It does not stop the hardware. PROS motors hold the last voltage they were
given, so a disabled `StateMechanism` keeps driving at whatever `applyState`
last wrote. Command a safe state first, then disable. And a `periodic()` that
integrates sensor deltas, like `ChassisController` updating odometry, misses
everything that happens while disabled and folds it into one step when
re-enabled, which corrupts the pose.

### Scheduler API

```cpp
CommandScheduler::registerSubsystem(&intake);                  // uses the stored default command
CommandScheduler::registerSubsystem(&intake, intake_idle.get());  // caller owned default command
CommandScheduler::unregisterSubsystem(&intake);                // cancels its command, stops periodic()
```

Registering a null subsystem, or one that is already registered, is a no-op
rather than an assertion failure. Asserts compile out in release builds, so they
were not a real guard.

`unregisterSubsystem` cancels whatever command currently requires the subsystem
and its default command, drops its requirement entry, and stops `runPeriodic()`
from being called on it. It is safe to call on a subsystem that was never
registered. `~Subsystem` does the same cleanup automatically, minus the `end()`
callbacks, so a subsystem that goes out of scope cannot leave the scheduler
holding dangling pointers.

`setDefaultCommand` is also safe to call after registration: the old default
command is cancelled and scrubbed from the scheduler before it is destroyed, and
the registration is repointed at the new one.
## PositionMechanism

`mechanism/position_mechanism.hpp` is a generic PID position mechanism. The
state is the target position, and the device is injected as two callables, so
the same class works with a rotation sensor, a motor encoder, or a
potentiometer.

```cpp
mclib::device::MotorGroup lift_motors({1, -2}, mclib::device::Gearset::Green);
mclib::device::Rotation lift_sensor(3);

mclib::mechanism::PositionMechanismConfig cfg;
cfg.kp = 0.09;
cfg.max_voltage = 10.0;
cfg.small_error = 1.5;

mclib::mechanism::PositionMechanism lift(
    [&] { return lift_sensor.getPositionDeg(); },
    [&](double volts) { lift_motors.setVoltage(volts); },
    cfg);
```

`moveTo(target)` starts closed-loop control and restarts the PID every time,
even when the target is unchanged, so re-issuing a target after a manual
override takes control back. `setManualVoltage(volts)` overrides the loop until
the next `moveTo()`. `stop()` brakes rather than coasts: it latches the present
position as the target and holds it, which keeps a loaded arm from falling.
Call `setManualVoltage(0.0)` if you want it to go limp instead. A mechanism
starts idle at 0 V until one of those is called.

Commands:

```cpp
lift.makeMoveToCommand(90.0);            // finishes on atTarget()
lift.makeMoveToCommand(90.0, 1500.0);    // also finishes after 1.5 s
lift.makeHoldCommand();                  // hold the current target
lift.makeManualCommand(6.0);             // 6 V while scheduled
lift.makeStopCommand();                  // hold where it is
```

`makeMoveToCommand` is the only factory that ends on its own; the others run
until interrupted.

`PID` stops driving once it flags arrival, so a settled mechanism has no
holding torque. `PositionMechanism` re-arms the loop when the position drifts
back outside `small_error`, and `atTarget()` stays true across that re-arm so a
finished move does not restart.
## VelocityMechanism

A closed-loop velocity mechanism whose state is the target RPM. The velocity
sensor and the voltage output are injected as callbacks, so it is not tied to
any particular device.

```cpp
ml::device::MotorGroup motors({1, -2}, ml::device::Gearset::Blue);

ml::mechanism::VelocityMechanismConfig config;
config.kv = 12.0 / 600.0;  // volts per RPM, roughly max volts / free speed
config.kp = 0.01;
config.tolerance_rpm = 50.0;
config.dwell_ms = 200.0;

ml::mechanism::VelocityMechanism spinner(
    [&motors] { return motors.getAverageActualVelocity(); },
    [&motors](double volts) { motors.setVoltage(volts); },
    config);

spinner.setTargetRpm(500.0);
spinner.getCurrentRpm();
spinner.atSpeed();      // within tolerance_rpm for dwell_ms straight
spinner.isSpinningUp(); // target commanded, not there yet
spinner.stop();         // coasts down
```

Output is `kv * target_rpm` feedforward plus PID correction, clamped to the
sign of the target so the mechanism is never driven backwards to brake. The
integral only accumulates within `integral_range_rpm` of the target, which
keeps it from winding up during spin-up.

`applyState` runs from `periodic()`, so the mechanism has to be registered with
the scheduler or nothing moves:

```cpp
std::unique_ptr<Command> spinner_stop;
std::unique_ptr<Command> spinner_hold;   // holds speed, never finishes
std::unique_ptr<Command> spinner_ready;  // finishes at speed or on timeout

void initialize() {
  spinner_stop = spinner.makeStopCommand();
  spinner_hold = spinner.makeSpinCommand(500.0);
  spinner_ready = spinner.makeSpinUpCommand(500.0, 2000.0);

  CommandScheduler::registerSubsystem(&spinner, spinner_stop.get());
}
```
