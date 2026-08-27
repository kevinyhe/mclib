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
std::unique_ptr<Command> wings_idle;

void initialize() {
  arm_low = arm.makeMoveToCommand(45.0, 2000.0);
  arm_stop = arm.makeStopCommand();
  wings_out = wings.makeExtendCommand();
  wings_in = wings.makeRetractCommand();
  wings_idle = wings.idleCommand();

  CommandScheduler::registerSubsystem(&arm, arm_stop.get());
  CommandScheduler::registerSubsystem(&wings, wings_idle.get());
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
## Conveyor Mechanism

`mechanism/conveyor_mechanism.hpp` adds a sensor-gated conveyor with jam
recovery.

```cpp
mclib::mechanism::ConveyorConfig config;
config.motor_ports = {11, -12};
config.forward_voltage = 12.0;
config.index_voltage = 8.0;
config.jam_current_amps = 2.0;
config.jam_velocity_rpm = 10.0;
config.jam_dwell_ms = 250.0;
config.unjam_voltage = -8.0;
config.unjam_ms = 250.0;
config.jam_clear_ms = 1000.0;
config.max_unjam_retries = 3;

mclib::device::Distance sensor(13);
mclib::mechanism::ConveyorMechanism conveyor(
    config, [&sensor] { return sensor.getDistanceMm() < 60.0; });

auto conveyor_stop = conveyor.makeStopCommand();
CommandScheduler::registerSubsystem(&conveyor, conveyor_stop.get());

conveyor.setConveyorState(mclib::mechanism::ConveyorState::Forward);
conveyor.stop();
bool ready = conveyor.hasObject();
bool stuck = conveyor.isJammed();
```

`applyState` runs from `periodic()`, so the conveyor must be registered with
`CommandScheduler` or its motors never move.

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
config.kv = 12.0 / 600.0;  // volts per motor RPM, max volts / motor free speed
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

Output is a `kv * target` feedforward plus PID correction, clamped to the sign
of the target so the mechanism is never driven backwards to brake. The integral
only accumulates within `integral_range_rpm` of the target, which keeps it from
winding up during spin-up.

### Gear ratio

`config.ratio` is the external gearing between the motor and the output shaft,
written as **output RPM per motor RPM**:

```
output_rpm = motor_rpm * ratio
motor_rpm  = output_rpm / ratio
```

Geared 1:2 for speed, so the output spins twice as fast as the motor, is
`ratio = 2.0`. Geared 2:1 for torque is `ratio = 0.5`. The default `1.0` means
the output *is* the motor shaft, and reproduces the ungeared behaviour exactly.

Which RPM is which matters, so it is pinned down:

| Quantity | Space |
| --- | --- |
| `setTargetRpm`, `getTargetRpm`, `getCurrentRpm` | output RPM |
| `makeSpinCommand`, `makeSpinUpCommand` | output RPM |
| `tolerance_rpm`, `integral_range_rpm` | output RPM |
| the velocity source callback, `getCurrentMotorRpm()` | motor RPM |
| `kp`, `ki`, `kd`, `kv` | volts per motor RPM |

The rule: **everything you say to the mechanism is output RPM**, because the
output speed is what you actually care about. The loop internally divides by
`ratio` and runs in motor RPM, so the gains keep their natural "volts per motor
RPM" meaning. For `kv` that means the number itself is unchanged by gearing - it
is set by the motor's free speed. `kp`, `ki` and `kd` still want retuning when
you change the gearing, because the load inertia reflected back to the motor
scales with `ratio` squared; what does not change is the units they are in.

A worked 1:2 example, with a blue-cartridge motor whose free speed is 600 motor
RPM driving a wheel through 1:2. The config has to be handed to the constructor,
so build the mechanism from it:

```cpp
ml::mechanism::VelocityMechanismConfig config;
config.ratio = 2.0;              // wheel spins 2x the motor
config.kv = 12.0 / 600.0;        // 0.02 V per motor RPM - unchanged by gearing
config.kp = 0.02;
config.tolerance_rpm = 10.0;     // 10 wheel RPM, i.e. 5 motor RPM
config.integral_range_rpm = 100.0;  // 100 wheel RPM, i.e. 50 motor RPM

ml::mechanism::VelocityMechanism spinner(
    [&motors] { return motors.getAverageActualVelocity(); },
    [&motors](double volts) { motors.setVoltage(volts); },
    config);

spinner.setTargetRpm(600.0);     // 600 wheel RPM = 300 motor RPM
```

With the motor measured at 280 RPM, the wheel is at 560 RPM:

- `getCurrentRpm()` returns `560.0`, `getCurrentMotorRpm()` returns `280.0`
- the PID error is the motor-space `300 - 280 = 20`, so P contributes
  `0.02 * 20 = 0.4 V`
- the feedforward is `kv * 300 = 6.0 V`, not `kv * 600`
- output is `6.4 V`

`atSpeed()` is judged in output RPM too. At `ratio = 2.0` with
`tolerance_rpm = 10.0`, a motor error of 6 RPM is a wheel error of 12 RPM and
does **not** count as at speed, even though 6 is under 10. Applying the ratio to
the target but not to the tolerance is the classic silent bug here; it is not
done that way.

`ratio` must be finite and greater than zero - zero would divide by zero, a
negative value would invert the loop, and an infinite one would divide every
target down to nothing. The constructor rejects anything else (NaN included) and
falls back to `1.0`, and `getConfig().ratio` then reports the `1.0` actually in
use.

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
## ToggleMechanism

`mclib::mechanism::ToggleMechanism` is a two-state mechanism built on
`StateMechanism<bool>`. It drives a `std::function<void(bool)>` actuator, so one
type covers solenoids, motor-driven two-position mechanisms, and test mocks.

The logical state ("is extended") is kept separate from the raw value written to
the actuator, so `isExtended()` always answers the question you asked.
## MechanismManager

`CommandScheduler::registerSubsystem` stores a raw `Command*`, but every
mechanism factory hands back a `std::unique_ptr<Command>`. Something has to keep
that command alive for as long as the scheduler can reach it. The examples above
do it with a global `std::unique_ptr<Command>` per mechanism; drop the global and
the scheduler is left holding a dangling pointer.

`mclib::mechanism::MechanismManager` owns those commands instead:

```cpp
#include "mclib/mclib.hpp"

using mclib::mechanism::ToggleMechanism;
using mclib::mechanism::ToggleMechanismConfig;

// One solenoid, wired so that false on the port means extended. Tell the
// device about the wiring; do not set ToggleMechanismConfig::inverted here.
ToggleMechanism clamp(std::make_shared<mclib::device::Pneumatic>('A', false, false));

// Several pneumatics driven together.
ToggleMechanism wings(std::vector<std::shared_ptr<mclib::device::Pneumatic>>{
    std::make_shared<mclib::device::Pneumatic>('E'),
    std::make_shared<mclib::device::Pneumatic>('F'),
});

// Any actuator at all. `inverted` belongs here, on raw actuators, where
// nothing else already flips the value.
ToggleMechanism motor_latch([](bool value) { printf("%d\n", value); },
                            ToggleMechanismConfig{
                                .initial_extended = false,
                                .inverted = true,
                            });
```

`set()`, `extend()`, `retract()` and `toggle()` write the actuator immediately,
even when the value did not change, so they work before the scheduler is
running. Once registered, `periodic()` rewrites it every tick.

Every command factory finishes on its own, unlike `PneumaticSubsystem` where
only the toggle command terminates:

```cpp
CommandController primary(mclib::device::ControllerId::Master);

std::unique_ptr<Command> wings_in;
std::unique_ptr<Command> wings_toggle;
std::unique_ptr<Command> wings_pulse;

void initialize() {
  wings_in = wings.makeRetractCommand();
  wings_toggle = wings.makeToggleCommand();
  wings_pulse = wings.makeExtendForCommand(500 * millisecond);

  CommandScheduler::registerSubsystem(&wings, wings_in.get());

  primary.getTrigger(mclib::device::DigitalButton::L1)
      ->onTrue(wings_toggle.get());
}
```

`makeSetCommand`, `makeExtendCommand`, `makeRetractCommand` and
`makeToggleCommand` act once and end. The `...ForCommand(QTime)` variants hold
the state for the duration, then end and leave the mechanism in that state --
they do not restore the previous one.
## MultiPositionMechanism

`mclib::mechanism::MultiPositionMechanism<StateT>` is for a mechanism that moves
between a fixed, ordered list of setpoints: arm presets, lift stages, hood
angles. You give it a table mapping each state to a number and a sink that
accepts that number. Every scheduler tick it pushes the current state's setpoint
into the sink, so it works with any position controller without knowing about
it.

```cpp
enum class LiftStage { Down, Middle, High };

double lift_target = 0.0;

mclib::mechanism::MultiPositionMechanism<LiftStage> lift(
    LiftStage::Down,
    {{LiftStage::Down, 0.0}, {LiftStage::Middle, 12.5}, {LiftStage::High, 30.0}},
    [](double setpoint) { lift_target = setpoint; });

lift.setPosition(LiftStage::Middle);
lift.next();      // -> High, and stays at High if called again
lift.previous();  // -> Middle
lift.cycle();     // -> High, then wraps to Down

auto lift_hold = lift.idleCommand();
CommandScheduler::registerSubsystem(&lift, lift_hold.get());

auto to_high = lift.makePositionCommand(LiftStage::High);
auto step_up = lift.makeNextCommand();      // one-shot, finishes immediately
auto step_dn = lift.makePreviousCommand();  // one-shot
auto rotate  = lift.makeCycleCommand();     // one-shot
```

The mechanism only moves once it is registered with `CommandScheduler`, which
is what calls `periodic()`. Register it with an idle default command, not with
`makePositionCommand(...)`: a position command runs forever and re-asserts its
state every tick, so if it is the default it will undo a `makeNextCommand()`
press on the tick right after that one-shot finishes.

`next()` and `previous()` clamp at the ends of the table. `cycle()` is the one
that wraps from the last entry back to the first. The table order, not the
numeric setpoints, decides what "next" means.

Degenerate cases are safe because `applyState` runs every tick. With an empty
table, or when the current state is not in the table, the sink is given the
default setpoint (constructor argument, `0.0` unless you pass one). That is the
same number `setpointFor()` and `currentSetpoint()` report, so an at-target
check never disagrees with what the sink was actually given. `currentIndex()`
returns `kNoPosition` for a state that is off the table, and `positionAt()`
returns `nullptr` for an index that is out of range. Calling `next()`,
`previous()`, or `cycle()` while the state is off the table moves to the first
entry, which is the home position by convention.

The table is searched front to back and the first match wins, so listing a
state twice makes the later entry unreachable. List each state once.

## HomingMechanism

`mechanism/homing_mechanism.hpp` finds a mechanism's zero by driving into a hard
stop, then re-zeroing the position sensor. All hardware access is injected, so it
works with a `device::Motor`, a `device::MotorGroup`, a `device::Rotation`, or any
mix of them.

```cpp
mclib::mechanism::HomingMechanismConfig cfg;
cfg.homing_voltage = -3.0;         // signed: direction matters
cfg.current_threshold_amps = 2.0;  // <= 0 disables the current detector
cfg.velocity_threshold_rpm = 5.0;  // <= 0 disables the stall detector
cfg.stall_dwell_ms = 150.0;
cfg.startup_grace_ms = 150.0;      // ramp-up window
cfg.timeout_ms = 3000.0;
cfg.backoff_voltage = 2.0;         // 0 or backoff_ms 0 disables back-off
cfg.backoff_ms = 150.0;

mclib::mechanism::HomingMechanism homing(
    cfg, [&](double volts) { arm_motors.setVoltage(volts); });

homing.setVelocitySource([&] { return arm_motors.getAverageActualVelocity(); });
// getAverageCurrentDraw() is in milliamps; the config threshold is in amps.
homing.setCurrentSource(
    [&] { return arm_motors.getAverageCurrentDraw() / 1000.0; });
homing.setPositionReset([&] { arm_rotation.resetPosition(); });

// Optional third detector, any predicate that is true at the stop.
pros::adi::DigitalIn arm_limit('A');
homing.setLimitSwitch([&] { return arm_limit.get_value() != 0; });
```

Three stop detectors can be enabled independently, and the first one to fire
wins: the limit-switch predicate, current draw above `current_threshold_amps`,
and a velocity stall (`|rpm| < velocity_threshold_rpm` held for
`stall_dwell_ms`). Inrush current is ignored for the first `startup_grace_ms`,
and the stall detector arms once the mechanism has been seen moving — or once
that window has passed, which covers a mechanism that starts out already resting
against the stop. The sensor is zeroed at the stop, before any back-off.

Drive it either from the state machine directly (`startHoming()`,
`cancelHoming()`, `isHoming()`, `isHomed()`, `hasFailed()`) or with a command:

```cpp
CommandScheduler::registerSubsystem(&homing, nullptr);

auto home_arm = homing.makeHomeCommand(4000 * millisecond);
home_arm->schedule();
while (home_arm->scheduled()) {
  pros::delay(10);
}
```

`makeHomeCommand` always finishes — on success, on failure, or on its own
timeout — and stops the motor on every exit path, interruption included. It also
steps the state machine from `execute()`, so it still works if the mechanism is
not registered with the scheduler.

## PTO (`mechanism/pto_mechanism.hpp`)

`PTOMechanism` is one set of motors mechanically switched between two consumers —
usually the drivetrain and a lift. It is a `StateMechanism<bool>`: `true` means the
motors are routed to the engaged consumer, `false` to the disengaged one.

```cpp
mclib::mechanism::PTOConfig pto_config{
    .motor_ports = {1, -2},
    .adi_port = 'A',
    .engaged_when_extended = true,   // extending the solenoid routes to the lift
    .shift_settle_time = 250 * millisecond,
    .drive_timeout = 100 * millisecond,
};

mclib::mechanism::PTOMechanism pto(pto_config);
std::unique_ptr<Command> pto_idle;

void initialize() {
  // The default command must never finish and must not force a state: an
  // instant default would re-disengage the PTO on the tick after every engage.
  pto_idle = pto.idleCommand();
  CommandScheduler::registerSubsystem(&pto, pto_idle.get());
ml::mechanism::MotorSubsystem flywheel{{1, -2}};
ml::mechanism::PneumaticSubsystem wings{{'A'}};

ml::mechanism::MechanismManager mechanisms;

void initialize() {
  mechanisms.add(&flywheel, flywheel.makeStopCommand(), "flywheel");
  mechanisms.add(&wings, wings.makeRetractCommand(), "wings");

  mechanisms.registerAll();
}

void opcontrol() {
  while (true) {
    // Drive writes are tagged with the side asking for them.
    pto.driveDisengaged(12.0);  // drivetrain: accepted while the PTO is disengaged
    pto.driveEngaged(12.0);     // lift: dropped while the PTO is disengaged

    CommandScheduler::run();
    pros::delay(10);
  }
}
```

Both drive calls return `true` when the write was accepted and `false` when it was
dropped. The accepted voltage reaches the motors on the next `periodic()` tick, so
register the mechanism with `CommandScheduler`. A write from the side that does not
own the PTO is discarded, not queued: a lift command writing volts while the motors
are geared to the wheels would drive the robot across the field.

Shifting under load shears gear teeth, so `engage()`, `disengage()` and `toggle()`
zero and brake the motors, throw the solenoid immediately, and refuse drive writes
from both sides for `shift_settle_time` (default 250 ms). A commanded voltage of
zero brakes rather than coasts, so a raised lift stays put. `isShiftSettled()` and
`remainingSettleTime()` report where the transition is.

The owning side is expected to write every tick. If it stops for longer than
`drive_timeout` (default 100 ms) the commanded voltage decays to zero instead of
latching on the motors; set `drive_timeout` to 0 to keep the last value.

`motors()` exposes the shared `device::MotorGroup` for brake modes, encoders, and
temperatures. Writing voltage through it bypasses the guard; use
`motorsFor(engaged_side)` for ownership-checked configuration and telemetry, and
re-fetch it every tick rather than caching it across a shift. Voltage always goes
through the drive calls: `periodic()` rewrites the commanded voltage every tick, so
a direct `setVoltage()` is overwritten on the next scheduler pass. The same holds
for the constructor that takes an existing `MotorGroup` — once the PTO shares it,
the drivetrain must write through `driveDisengaged()` too.

Commands: `makeEngageCommand()`, `makeDisengageCommand()` and `makeToggleCommand()`
are one-shot and finish immediately. `makeShiftCommand(engaged)` requires the
mechanism for the whole settle window and finishes only once the shift has settled,
so nothing can drive into a half-thrown gearbox.
conveyor.makeForwardCommand();
conveyor.makeReverseCommand();
conveyor.makeStopCommand();
conveyor.makeIndexCommand(1500.0);  // finishes when the gate latches, or on timeout
conveyor.makeStateCommand(mclib::mechanism::ConveyorState::Reverse);
```

`makeIndexCommand` is the only factory here that finishes on its own. The rest
run until interrupted.

Jam handling: current above `jam_current_amps` while `|velocity|` is below
`jam_velocity_rpm` for `jam_dwell_ms` triggers an unjam at `unjam_voltage` for
`unjam_ms`, then the previous state resumes. After `max_unjam_retries`
consecutive attempts the motors stop and `isJammed()` latches true until the
state changes or `clearJam()` is called; the retry count refills after
`jam_clear_ms` of unstalled running. Without a sensor gate, `IndexToSensor`
holds at zero volts, `hasObject()` is always false, and `makeIndexCommand`
finishes immediately.

Sign-correct opposed motors in `motor_ports` (a negative port reverses that
motor). Jam detection reads the group average, so uncorrected ports cancel to
a near-zero velocity and look permanently stalled.
The manager must outlive the scheduler's use of the commands, so give it static
or program-long storage — a file-scope object like above is fine.

`add()` returns `false` and stores nothing if the subsystem or command is null,
if that subsystem is already held, or if the name is already taken. `name` is
optional and defaults to `mechanism0`, `mechanism1`, and so on.

`registerAll()` is safe to call twice; it skips entries it already registered and
returns how many it registered this time. The scheduler's own duplicate check is
an `assert`, which does nothing in a release build.

The rest of the API:

```cpp
mechanisms.size();                       // how many entries are held
mechanisms.contains("wings");
mechanisms.getSubsystem("wings");        // Subsystem*, or nullptr
mechanisms.getDefaultCommand("wings");   // Command*, manager keeps ownership
mechanisms.getEntry("wings");            // const Entry*, or by index
mechanisms.getNames();                   // insertion order
mechanisms.isRegistered("wings");
mechanisms.isEnabled("wings");
mechanisms.setEnabled("wings", false);   // skip it on the next registerAll()
mechanisms.setAllEnabled(true);
printf("%s", mechanisms.describe().c_str());
```

`describe()` prints one line per entry — name, enabled, registered, and which
command currently holds the scheduler's requirement on that subsystem:

```
flywheel enabled=yes registered=yes requirement=default
wings enabled=yes registered=yes requirement=other
```

Disabling an entry only stops a later `registerAll()` from registering it. The
scheduler has no API to drop a subsystem once registered, so disabling an
already-registered entry changes the record and nothing else.

A subsystem must be registered by exactly one path. If you already call
`CommandScheduler::registerSubsystem` for a mechanism, do not also add it to a
manager: the scheduler catches the duplicate with an `assert`, which is compiled
out in release, and the second registration silently overwrites the first while
the original default command keeps the requirement.

`MechanismManager` is neither copyable nor movable. It owns the commands the
scheduler holds raw pointers to, and the scheduler has no unregister call, so
moving the manager would leave the scheduler pointing at freed commands.

## PneumaticSubsystem: polarity and command termination

`PneumaticSubsystem` state is always the logical state. `true` means the
cylinder is extended, `false` means it is retracted. Wiring polarity is a
separate flag and stays inside `device::Pneumatic`, which maps
`raw = (logical == extended_state)`. Nothing above the device layer sees the
solenoid pin level.

```cpp
// Normal solenoid: pin high extends.
mclib::mechanism::PneumaticSubsystem wings({'E', 'F'});

// Inverted solenoid: pin low extends. Starts extended.
mclib::mechanism::PneumaticSubsystem clamp({'G'}, /*initial_extended=*/true,
                                           /*extended_state=*/false);

clamp.isExtended();  // true, for either wiring, and from construction on
```

The second constructor argument is the logical starting state, not a pin
level. The constructor drives the solenoids to that state, so `isExtended()`
agrees with the hardware before the first `periodic()` tick.

Command termination:

- `makeSetCommand`, `makeExtendCommand`, `makeRetractCommand`, and
  `makeToggleCommand` all set the state once and finish. Setting a solenoid is
  a one-shot action, so holding the subsystem requirement forever is wrong.
- `makeExtendForCommand(500 * millisecond)` extends, holds, then finishes.
- `makeHoldCommand(extended)` forces a state on every tick and never finishes.
  Do not register it as a default command: the scheduler reschedules the
  default as soon as a one-shot command releases the requirement, so a hold
  default would drag the cylinder back one tick after every `makeExtendCommand`.
  Register `idleCommand()` instead. `periodic()` keeps re-applying the cached
  state, so the cylinder stays where the last command put it.

Accessors: `count()`, `group()` for direct device access, and `rawValues()`
for the per-solenoid values the device layer reports. Those values are logical
too, not pin levels; they are for spotting one solenoid out of sync with the
rest of the group. Writes through `group()` do not update the cached state and
are overwritten by the next `periodic()`.

## Library fixes: mechanism copy/move, and PID holding output

Two defects that several mechanisms were each working around locally are now
fixed at the source. The local workarounds are still in place and still
correct; removing them is a separate follow-up, so mechanism behaviour is
unchanged by this alone.

### `StateMechanism` is explicitly non-copyable and non-movable

`StateMechanism<StateT>` now deletes its copy constructor, copy assignment,
move constructor and move assignment.

Subclasses routinely store a `std::function` that captured `this` -
`MappedMechanism::ApplyState`, `MotorStateMechanism`'s voltage map, the
callbacks in the position, velocity, conveyor and PTO mechanisms. Copying or
moving such an object copies the callable but not what it points at, so the new
object's `periodic()` would drive real motors through a pointer to the old,
possibly destroyed, object. A mechanism is also an identity: the
`CommandScheduler` registers it by address and it owns its default command, so
a second copy was never meaningful.

Both operations were in fact already blocked by accident - `Subsystem` holds a
`std::unique_ptr<Command>`, which implicitly deletes its copy constructor, and
declares a virtual destructor, which suppresses the implicit move constructor.
That is fragile (removing the `unique_ptr` member would silently re-enable
copying) and the compiler error pointed at `Subsystem`'s members instead of at
the rule. The explicit deletions pin the guarantee and make the diagnostic say
what is wrong.

Hold mechanisms in place: static or program-long storage, or
`std::vector<std::unique_ptr<T>>`. `std::vector<PositionMechanism>` does not
compile, which is the point.

### `PID::setHoldOutput(bool)`

`PID::update()` used to return a hard `0` on every tick once `arrived` latched,
until `reset()` was called. A settled positional mechanism therefore had zero
holding torque and sagged under gravity.

`setHoldOutput(true)` keeps computing `P + I + D` after arrival. Arrival is
still detected and `targetArrived()` still latches exactly as before, so a
caller can use arrival as a "motion finished" signal while the loop holds the
setpoint.

What you get inside the settle band is a P (plus D) hold, not a zero-error
hold: arrival requires `|error| <= small_error_tolerance`, and `update()` zeroes
`sum_error` over that same band, so the integral is 0 on every held tick. A
loaded arm settles wherever `kp * error` balances the load, up to
`small_error_tolerance` of steady droop.

The default is `false`, which is the original behaviour, so existing callers
are unaffected. `ChassisController` and the legacy routines in
`control/motion.cpp` end their motions on `targetArrived()`, which this change
does not touch either way.

`setHoldOutput` is on `PID` itself. `Arm` and `PositionMechanism` hold their
`PID` privately with no accessor and no config flag, so today only code that
owns a bare `PID` can turn it on. Plumbing it through those configs is the
follow-up that also retires their local workarounds.

Three useful configurations:

| Configuration | Behaviour | Use for |
| --- | --- | --- |
| `setArrive(true)`, hold off (default) | Move, latch, then output 0 | Chassis motion that ends on arrival |
| `setArrive(true)`, `setHoldOutput(true)` | Latch and report arrival, keep driving | Lift or arm that would otherwise sag |
| `setArrive(false)` | Never latch, never zero | Velocity control, where arrival is meaningless |

### The `small_error_tolerance` trap

Worth knowing, and now documented in `pid.hpp`: `PID`'s default
`small_error_tolerance` is `1`, and `update()` zeroes `sum_error` whenever
`|error| <= small_error_tolerance`. For any loop that operates inside an error
of 1 - velocity control in rpm, a position loop in revolutions - that silently
disables `ki` entirely. Call `setSmallBigErrorTolerance(0, 0)` for those loops
(and `setArrive(false)`, since the same tolerances drive arrival detection).

`pid.hpp` now documents every method and the interaction between `arrive`,
`arrived`, `hold_output`, the two error tolerances and the two settle
durations.
## ToggleGroupMechanism

`ToggleMechanism` moves a whole group of actuators with one shared boolean.
`ToggleGroupMechanism` gives each actuator its own boolean while keeping them
in a single subsystem, so they share one scheduler requirement and two commands
can never fight over them.

Channels are addressed by index, and any scoped enum works as an index:

```cpp
enum class Side : std::size_t { Left = 0, Right = 1 };

// One pneumatic per channel. Both plumbed so pin high extends.
mclib::mechanism::ToggleGroupMechanism sides({
    std::make_shared<mclib::device::Pneumatic>('A'),
    std::make_shared<mclib::device::Pneumatic>('B'),
});

void example() {
  sides.set(Side::Left, true);
  sides.get(Side::Right);   // false
  sides.toggle(Side::Right);
  sides.setAll(false);
  sides.toggleAll();
  sides.allSet();           // every channel extended
  sides.anySet();           // at least one extended
  sides.setCount();         // how many are extended
  sides.count();            // how many channels there are
}
```

Every command factory finishes on its own: the one-shot ones after a single
tick, the `...ForCommand(QTime)` variants once their duration is up. The timed
variants stay where they finish, they do not restore the old state.

`Trigger::onTrue` takes a raw `Command*`, so something has to keep the command
alive for as long as the scheduler can reach it. Hold each one in a
`std::unique_ptr` that outlives the scheduler, or hand them to a
`MechanismManager`; binding a temporary leaves a dangling pointer.

```cpp
CommandController primary(mclib::device::ControllerId::Master);

std::unique_ptr<Command> left_toggle;
std::unique_ptr<Command> right_toggle;
std::unique_ptr<Command> both_out;
std::unique_ptr<Command> both_toggle;
std::unique_ptr<Command> left_pulse;

void initialize() {
  left_toggle = sides.makeToggleCommand(Side::Left);
  right_toggle = sides.makeToggleCommand(Side::Right);
  both_out = sides.makeSetAllCommand(true);
  both_toggle = sides.makeToggleAllCommand();
  left_pulse = sides.makeSetForCommand(Side::Left, true, 500 * millisecond);

  sides.setName("sides");
  sides.setDefaultCommand(sides.idleCommand());  // see below, this matters
  sides.registerSelf();

  primary.getTrigger(mclib::device::DigitalButton::L1)->onTrue(left_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::R1)->onTrue(right_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::A)->onTrue(both_out.get());
  primary.getTrigger(mclib::device::DigitalButton::B)->onTrue(both_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::X)->onTrue(left_pulse.get());
}
```

The default command MUST be `idleCommand()`. Every factory above terminates,
which releases the requirement, and `CommandScheduler::run()` reschedules the
default on the very next tick. A "retract everything" default would therefore
undo every set one tick after it happened. `periodic()` keeps re-writing the
cached states, so the actuators stay where the last command put them.

Per channel inversion is for raw `std::function<void(bool)>` actuators only:

```cpp
bool left_raw = false;
bool right_raw = false;

// Channel 1's actuator is wired backwards, so it gets the negated value while
// get(1) still answers "is it extended".
mclib::mechanism::ToggleGroupMechanism raw_sides(
    {[](bool v) { left_raw = v; }, [](bool v) { right_raw = v; }},
    {.initial_states = {false, true}, .inverted = {false, true}});
```

Lambdas that capture by reference cannot live at namespace scope, so build the
mechanism inside a function if your actuators need captures.

Do not set `inverted` for a channel driving a `device::Pneumatic`. `Pneumatic`
already takes and returns a logical value and inverts internally through its
`extended_state` flag. Inverting on top of that inverts twice and leaves
`Pneumatic::get_value()` reporting the opposite of `get(index)`. For a solenoid
wired backwards, write `Pneumatic(port, false, false)` and leave the inversion
entry false.

Out of range indices are ignored by the mutators and read back as `false`, so a
bad enum value cannot corrupt the state. Copy and move are deleted.

The channel count is fixed at construction by the number of actuators. The
inherited `setState()` and the generic `makeState*Command()` factories are
hidden, because they take a `std::vector<bool>` of any length and caching a
short one would make the trailing channels unreachable and latch their
actuators wherever they happened to be. Use `setAll()` and the factories above.
## `DiscreteActuatorMechanism`

`ToggleMechanism` drives one boolean actuator. `MultiPositionMechanism` maps a
state to one number. Neither covers several solenoids whose combined pattern
encodes a handful of named positions. `DiscreteActuatorMechanism<StateT>` owns
that decode table and pushes the pattern of the current state into a bank of
injected actuators every tick.

```cpp
enum class Tilt { Flat, Angled, Steep };

// Two cylinders, three named angles. The table is the only source of truth:
// to add an angle, add a row. There is no if/else chain anywhere.
mclib::mechanism::DiscreteActuatorMechanism<Tilt> tilt(
    Tilt::Flat,
    {{Tilt::Flat,   {false, false}},
     {Tilt::Angled, {true,  false}},
     {Tilt::Steep,  {true,  true}}},
    std::vector<std::shared_ptr<mclib::device::Pneumatic>>{front, back});

void initialize() {
  tilt.setName("tilt");
  // MUST be idleCommand(). Every factory below finishes after one tick, which
  // releases the requirement and lets the scheduler reschedule the default
  // immediately. A default that commanded a state would drag the cylinders
  // back one tick after every button press. periodic() keeps re-applying the
  // cached state, so the cylinders stay where the last command put them.
  tilt.setDefaultCommand(tilt.idleCommand());
  tilt.registerSelf();
}
```

Table values are logical: `true` means extended. `device::Pneumatic` maps
`raw = (logical == extended_state)` itself, so a backwards-wired cylinder gets
`extended_state = false` at the device and the table stays readable. The
`inverted` flag in `DiscreteActuatorMechanismConfig` is for raw
`std::function<void(bool)>` actuators only; using it with a `Pneumatic` inverts
twice.

Stepping follows the same convention as `MultiPositionMechanism`:

- `next()` and `previous()` clamp. At the last row `next()` does nothing.
- `cycle()` wraps. From the last row it goes back to the first.

Commands, all terminating: `makeDiscreteStateCommand(state)`,
`makeNextCommand()`, `makePreviousCommand()`, `makeCycleCommand()`, and
`makeDiscreteStateForCommand(state, 500 * millisecond)` which holds a state for
a duration then finishes and stays there.

```cpp
// Triggers take a raw pointer, so keep the commands alive yourself.
std::unique_ptr<Command> tilt_cycle;
std::unique_ptr<Command> tilt_flat;

void initialize() {
  tilt_cycle = tilt.makeCycleCommand();
  tilt_flat = tilt.makeDiscreteStateCommand(Tilt::Flat);

  primary.getTrigger(mclib::device::DigitalButton::R1)
      ->onTrue(tilt_cycle.get());
  primary.getTrigger(mclib::device::DigitalButton::A)
      ->onTrue(tilt_flat.get());
}
```

Nothing here can drive a half-decoded combination. Rows whose width does not
match the actuator count are dropped at construction, so every surviving row
applies whole. A state that is not in the table is undecodable:
`applyState()` writes nothing for it and the actuators hold the last pattern
they were given, `setDiscreteState()` refuses to move to it, and the inherited
`setState()` / `makeStateCommand()` are private so no command can latch a state
that can never be applied. Only the initial state can be undecodable, and the
first `next()`, `previous()`, or `cycle()` resyncs to row 0.

Dropping is silent, so one mistyped row turns into a button that does nothing.
`droppedRowCount()` reports how many rows were thrown away; check it once at
startup if a position looks dead.

`makeDiscreteStateForCommand` is the only mutator that does not write the
actuators itself: it caches the state and lets `periodic()` push it, the same
as `ToggleMechanism::makeSetForCommand`. That costs one tick normally, and
costs everything while the subsystem is disabled, since `periodic()` is then
skipped entirely.

Accessors: `stateCount()`, `actuatorCount()`, `droppedRowCount()`, `empty()`, `indexOf(state)`,
`currentIndex()`, `stateAt(index)`, `isDecodable(state)`,
`combinationFor(state)`, `currentCombination()`, `rawValue(index, value)`.
`StateT` only needs `operator==`, so a scoped enum works.
