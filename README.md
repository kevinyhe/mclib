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

## Tests

```sh
make test
```

`make test` compiles every `tests/*.cpp` into its own host binary with the
system `g++` and runs it. It never touches the ARM toolchain, never links PROS,
and never runs the firmware build. Binaries land in `bin/tests/`, which is
already gitignored, and `make clean` removes them.

`tests/` sits at the repo root, deliberately outside `src/`. `common.mk` globs
`src/**` recursively into the firmware, so a test directory under `src/` would
be compiled into the library. Do not move it, and do not add `tests/` to
`TEMPLATE_FILES`.

### Writing a test

One test per file, each with its own `int main()`. There is no framework, no
registration macro, and nothing to add to the `Makefile` — the glob picks up
any new `tests/*.cpp` on the next run.

```cpp
// mclib
#include "mclib/math.hpp"

#include "test_assert.hpp"

int main() {
  CHECK_NEAR(mclib::wrapAngle(3.0 * mclib::kPi), mclib::kPi, 1e-12);
  CHECK_EQ(mclib::clamp(11.0, 0.0, 10.0), 10.0);
  CHECK(mclib::clamp(5.0, 10.0, 0.0) == 5.0);
  return mclib::test::summary("my feature");
}
```

`tests/test_assert.hpp` gives you three macros:

| Macro | Use |
| --- | --- |
| `CHECK(cond)` | boolean condition |
| `CHECK_NEAR(actual, expected, eps)` | floating point within a tolerance |
| `CHECK_EQ(actual, expected)` | exact equality on doubles |

Prefer the numeric ones. A failure prints the file, the line, the expression,
and both values:

```text
== utils_test
  FAIL ./tests/utils_test.cpp:44: CHECK_NEAR(getRadius(0.0, 0.0, 3.0, 4.0, 0.0), 6.4, 1e-12)
       expected 6.4, got 3.125 (diff -3.275)
FAIL utils (1 of 120 checks failed)

FAILED TESTS: utils_test
```

A failed assertion does not stop the run, so one invocation reports every
broken check. `mclib::test::summary()` returns 0 when everything passed and 1
otherwise; `make test` exits non-zero and names each failing binary.

### Documenting a bug you are not allowed to fix

Do not `CHECK` the wrong value. Pinning known-bad behaviour means the person
who eventually fixes it gets a red build blamed on their commit. Use
`mclib::test::knownBug(still_present, "what is wrong")` instead — it prints
either `KNOWN BUG (still present)` or `KNOWN BUG (appears FIXED, update this
test)` and never touches the exit code.

```cpp
double left = -6.0;
double right = -6.0;
scaleToMax(left, right, 4.0);
mclib::test::knownBug(left < -4.0, "scaleToMax does not cap equal negatives");
```

### What can be tested

Tests link against `HOST_TEST_SRC` in the `Makefile` — the library sources that
compile without PROS headers. Today that is:

- `src/mclib/math.cpp`
- `src/mclib/utils.cpp`
- `src/mclib/control/scaling.cpp`
- `src/mclib/control/robot_state.cpp`
- `src/mclib/control/odometry.cpp`

Those last two are PROS-free by construction, not by accident. `sync.hpp`
picks `std::mutex` over `pros::Mutex` when `MCLIB_HOST_BUILD` is defined, and
`Odometry` takes a struct of raw sensor readings rather than reading devices
itself -- the task that does read them lives in `control/odometry_task.cpp`,
which is not host-testable and holds no math.

This list is expected to grow. Anything that pulls in `pros/...` cannot be
linked on the host, so making a source testable usually means putting a seam in
front of the PROS call (a time source, a motor interface) rather than changing
the test setup. When a source becomes PROS-free, add it to `HOST_TEST_SRC` and
it is available to every test.

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

## Units (`units/units.hpp`)

Every dimensioned value in mclib is a `Quantity`: one `double` wrapped in a type
that carries five integer exponents - length, time, angle, voltage, current.
Multiplying adds the exponents, dividing subtracts them, so `QLength / QTime`
*is* `QVelocity` and `QLength + QTime` does not compile.

```cpp
QLength distance = 24_in;
QTime   timeout  = 1500_ms;
QAngle  heading  = 90_deg;

QVelocity speed = distance / timeout;   // QVelocity, checked at compile time
QLength   back  = speed * timeout;      // 24 in again
// QLength wrong = distance + timeout;  // error: no operator+
```

A `Quantity` is the same size as the `double` it replaces, all its operations
are `constexpr` and `inline`, and none of it survives to the ELF - the section
sizes of `bin/cold.package.elf` are unchanged by the migration.

It is still floating point underneath, so do not expect exact decimal
round-trips: `(24_in).in()` is `23.999999999999996`, because `24.0 * 0.0254` is
not representable. Compare with a tolerance, never with `==`.

### Aliases

`QNumber`, `QLength`, `QArea`, `QTime`, `QAngle`, `QVoltage`, `QCurrent`,
`QVelocity`, `QAcceleration`, `QJerk`, `QAngularVelocity`,
`QAngularAcceleration`, `QCurvature` (1/length), `QFrequency` (1/time).

Anything else you need is a valid type already: `QVoltage / QCurrent` is a
resistance, `square(QLength{}) ` is an area.

### Literals

`_m` `_cm` `_mm` `_in` `_ft` `_tile` - length (a `_tile` is 24 in)
`_s` `_ms` `_min` - time
`_rad` `_deg` `_rot` - angle
`_V` `_mV` `_A` `_mA` - voltage and current
`_mps` `_inps` `_mps2` `_radps` `_degps` `_rpm` - rates

Named constants exist for all of them too, for when the number is not a
literal: `metre`, `inch`, `tile`, `minute`, `degree`, `radian`, `rotation`,
`volt`, `millivolt`, `ampere`, `milliampere`, `rpm`, `percent`. These live in
`mclib::units` and are **not** global, so qualify them or pull the namespace in:

```cpp
using namespace mclib::units;
QLength target = 24 * inch;
```

The two exceptions are `millisecond` and `second`, which are global for
back-compat — `250 * millisecond` and `pros::millis() * millisecond` work
unqualified anywhere.

### Getting a raw double back out

Both directions of the escape hatch are explicit, because both are where unit
bugs come from.

```cpp
double ms = timeout.ms();       // named accessor - says what the number means
double in = distance.in();
double dg = heading.deg();
double mv = battery.mV();

QLength from_raw{0.6096};       // explicit ctor, value in SI base units
```

Accessors: `.m() .cm() .mm() .in() .ft()`, `.s() .ms()`, `.rad() .deg()`,
`.volts() .mV()`, `.amps() .mA()`, `.mps() .inps()`, `.radps() .degps() .rpm()`.
Each is constrained to its own dimension, so `timeout.in()` is a compile error
rather than a wrong number. Free-function spellings - `inches(x)`,
`milliseconds(t)`, `degrees(a)` - exist for call sites where they read better.
`.raw()` gives the stored value in SI base units and is the escape hatch of last
resort.

### Angles

`QAngle` stores **radians**. The public motion API of this library speaks
degrees and `Pose2D::theta` speaks radians; making the conversion a method call
(`.deg()` / `.rad()`) is the point. Angle is a real dimension, so
`radius * angle` does not type-check on its own - use the sanctioned crossings:

```cpp
QLength arc     = arcLength(radius, angle);      // radius * angle
QAngle  swept   = arcAngle(arc, radius);         // the inverse
QVelocity rim   = rimVelocity(radius, omega);    // radius * angular velocity
QAngularVelocity turn = turnRate(speed, curvature);   // speed * curvature
QCurvature k    = turnCurvature(speed, omega);        // the inverse
```

`turnRate` is the one pure pursuit needs: `speed * curvature` on its own is a
`QFrequency` — right arithmetic, wrong dimension.

`wrap(angle)` folds an angle into [-180 deg, +180 deg], using the same algorithm
and the same closed range as the pre-existing `mclib::wrapAngle(double)`,
including at exactly -180 deg. Two wrapping functions that disagreed on that
edge would be a trap for motion code migrating from `double` to `QAngle`.

### Math helpers

`abs`, `min`, `max` and `clamp` return the same dimension they were given.
`sign` returns a plain double (-1, 0 or +1) and `square` doubles the exponents.
`sin/cos/tan` take a `QAngle` and return a plain double; `asin/acos/atan` go the
other way; `atan2(QLength, QLength)` returns a `QAngle` and
`hypot(QLength, QLength)` a `QLength`.

### Namespaces and back-compat

Everything lives in `mclib::units`. For back-compat the type aliases, the
`millisecond` / `second` constants and all literal operators are also pulled
into the global namespace, so the pre-existing idiom keeps working unchanged:

```cpp
QTime now = pros::millis() * millisecond;
if (now - start >= 250 * millisecond) { /* ... */ }
```

The global surface is split in two. `QTime`, `millisecond` and `second` are
unconditional, because mclib's own headers use them unqualified at ~33 sites —
making those conditional would only mean the library stops compiling. Everything
else — the other 13 aliases and all the literal suffixes — is convenience, and
defining `MCLIB_NO_GLOBAL_UNITS` before including mclib switches it off.

That opt-out exists for a specific collision: okapilib declares the same
`QLength` / `QAngle` / `QArea` / `QJerk` / `QFrequency` / `QAcceleration` names
and the same `_in` / `_ft` / `_deg` / `_rad` / `_ms` / `_s` / `_rpm` suffixes, so
a project doing `using namespace okapi;` alongside mclib would get ambiguity on
all of them. With the macro defined, reach for `mclib::units::` instead.

The `Quantity` template itself is never exported globally — spell it
`mclib::units::Quantity` — because a downstream global `class Quantity` would
otherwise become ambiguous.

One deliberate hole in the type safety: `QNumber` (all exponents zero) converts
implicitly to and from `double`, because gains and gear ratios have to
interoperate with plain arithmetic. Every other dimension requires the explicit
constructor.

### Tests

`tests/units_test.cpp` is a standalone host program:

```sh
g++ -std=gnu++20 -Iinclude -o /tmp/units_test tests/units_test.cpp && /tmp/units_test
```

It covers dimension composition, literal values, round-tripping and the old
`QTime` millisecond semantics. The checks that dimensionally-wrong code does
*not* compile are written as concepts whose negation is asserted - if
`QLength + QTime` ever starts compiling, that file stops building.

## Coordinate Frame

mclib has one canonical frame, the **compass / field frame**:

- `theta = 0` points along **+Y**.
- `theta` increases **clockwise**, so **+90 deg points along +X**.
- The unit vector for a heading is `(sin(theta), cos(theta))` -- x uses sin, y
  uses cos.
- A bearing from A to B is `atan2(b.x - a.x, b.y - a.y)` -- x first, y second.

Both of those are the transpose of the usual textbook formulas. This is
deliberate: it matches how VEX field diagrams are drawn, and it is what
`control/odometry.cpp`, `control/motion.cpp` and `snapshot/raycast.cpp`
already do.

Units: angles are **radians** everywhere inside `math.hpp` (`Pose2D::theta`,
`wrapAngle`), and **degrees** at the public motion API (`Chassis`,
`control/motion.cpp`, `RobotState::correctAngleDeg()`). Convert at that boundary with
`degToRad` / `radToDeg` from `utils.hpp`. Translations are inches.

The robot frame is chosen to coincide with the field frame at `theta = 0`:
**+Y is forward, +X is the robot's right.**

### Converting between frames

Use these instead of writing `sin`/`cos` by hand:

```cpp
Pose2D compose(const Pose2D& base, const Pose2D& local);

Vec2 headingVector(double rad);                  // (sin, cos)
double headingToward(const Vec2& from, const Vec2& to);  // compass bearing

Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad);
Vec2 robotToField(const Vec2& robot_vec, double heading_rad);
Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose);
Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose);

double arcRadius(const Pose2D& from, const Vec2& target);
```

`fieldToRobot` / `robotToField` take a displacement and only rotate it. The
`...Point...` variants translate first, so they answer "where is this field
point relative to the robot".

### `rotationMatrix` and `rotate` are not the field convention

`rotationMatrix(rad)` is the standard textbook rotation: counter-clockwise,
zero along +X, `[[cos, -sin], [sin, cos]]`. Keep using it for generic linear
algebra. Do **not** use it to move a pose or a waypoint between frames.

Feeding it a compass heading turns the wrong way: `rotate({0, 1}, rad)` gives
`(-sin, cos)`, while a robot at that heading actually points at `(sin, cos)`.
Because the two frames are transposes, `rotationMatrix(rad)` happens to equal
the field-to-robot matrix, so `rotate()` with a compass heading silently does
`fieldToRobot()` -- the inverse of what "rotate my local offset into the
field" means. Call `fieldToRobot` / `robotToField` and the bug cannot happen.

### `Pose2D::operator+` is not a compose

`operator+` adds `x`, `y`, and `theta` component-wise and wraps `theta`. It
does **not** rotate the incoming translation by the existing heading, so it is
"add a field-frame offset", not "move in my own frame". `compose(base, local)`
is the real SE(2) operation: `local` is interpreted in `base`'s frame, with
`local.x` to the right and `local.y` forward.

```cpp
Pose2D p{0, 0, degToRad(90)};            // facing +X
p + Pose2D{0, 10, 0}      // -> (0, 10) : offset added in field coordinates
compose(p, Pose2D{0, 10, 0})  // -> (10, 0) : 10 inches forward
```

## Core Math API

```cpp
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

double clamp(double val, double min, double max);
double wrapAngle(double rad);  // radians, wraps into [-pi, pi] (closed both ends)

struct Pose2D {
  double x;      // inches, field frame
  double y;      // inches, field frame
  double theta;  // radians, 0 = +Y, clockwise-positive

  Pose2D operator+(const Pose2D& other) const;  // component-wise, not a compose
  double distanceTo(const Pose2D& other) const;
  Vec2 translation() const;
  Vec3 vector() const;
};

Pose2D compose(const Pose2D& base, const Pose2D& local);

Vec2 headingVector(double rad);
double headingToward(const Vec2& from, const Vec2& to);
Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad);
Vec2 robotToField(const Vec2& robot_vec, double heading_rad);
Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose);
Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose);
double arcRadius(const Pose2D& from, const Vec2& target);

// Standard frame (CCW, 0 = +X). NOT the field convention -- see above.
Mat2 rotationMatrix(double rad);
Vec2 rotate(const Vec2& vec, double rad);
```

`utils.hpp` lives in the **global namespace** and holds the degree/radian
bridge plus `getRadius`:

```cpp
double degToRad(double deg);
double radToDeg(double rad);

// x/y/x1/y1 in inches (field frame), angle in DEGREES (compass frame).
// Legacy and frame-buggy -- see below. Returns +infinity when the
// denominator degenerates.
double getRadius(double x, double y, double x1, double y1, double angle);
```

`getRadius` is a legacy helper with a frame bug: its denominator uses
`delta_y` where the target's **lateral** offset in the robot frame belongs, so
a target 10 in dead ahead of a robot at heading 0 -- a straight line, infinite
radius -- comes back as 5. `mclib::arcRadius(from, target)` computes it
correctly. `getRadius` is left alone because `boomerang`'s tuning was fitted
around its behavior; rewiring the caller is Phase 3 work.

The one change made here is the degenerate case: it used to return a magic
`999`, which silently became a finite speed limit downstream, and now returns
infinity. Its one caller, `control/motion.cpp:1074`, feeds it to
`sqrt(chase_power * getRadius(...) * 9.8)` -- an expression that mixes a
voltage-ish tuning constant, a radius in inches, and g in m/s^2, takes the
square root of a value that can be negative, and now yields NaN when
`chase_power` is 0. That is a known problem and out of scope here.

Other modules are split into matching header/source pairs:

- `auton/*.hpp` / `auton/*.cpp`: owning autonomous routine builder
- `chassis/*.hpp` / `chassis/*.cpp`: chassis hardware and PID controller
- `config.hpp` / `config.cpp`: robot hardware and tuning globals
- `control/*.hpp` / `control/*.cpp`: sorted chassis I/O, scaling, motion, odometry, and shared state helpers
- `pid.hpp` / `pid.cpp`: PID controller
- `utils.hpp` / `utils.cpp`: angle and geometry utilities
- `device/*.hpp` / `device/*.cpp`: the only place that calls PROS motor, controller, pneumatic, and sensor APIs directly
- `mechanism/*.hpp` / `mechanism/*.cpp`: generic stateful mechanisms, plus intake, motor, and pneumatic subsystem examples
- `mechanism/*.hpp` / `mechanism/*.cpp`: generic stateful mechanisms (conveyor, position, velocity, toggle, multi-position, homing, PTO) plus motor and pneumatic subsystem wrappers
- `snapshot/*.hpp` / `snapshot/*.cpp`: distance-sensor pose snapshot helpers

## Subsystem lifecycle

Command factories return `std::unique_ptr<Command>`, but the scheduler stores raw
`Command*`. That mismatch is easy to get wrong:

```cpp
// BROKEN. The unique_ptr dies at the end of the statement, so the scheduler is
// left holding a dangling pointer.
conveyor.makeForwardCommand()->schedule();
```

The subsystem itself is the owner. Hand it the default command with
`setDefaultCommand` and register with `registerSelf`:

```cpp
Intake intake{...};
PositionMechanism arm{...};
ConveyorMechanism conveyor{{-20, -21}};
Arm arm{...};

void initialize() {
  conveyor.setName("conveyor");
  conveyor.setDefaultCommand(conveyor.makeStopCommand());
  conveyor.registerSelf();

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
std::unique_ptr<Command> conveyor_idle;

void initialize() {
  conveyor_idle = conveyor.makeStopCommand();
  CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());
}
```

Commands that are not defaults still need an owner. Store the `unique_ptr`
somewhere that outlives the scheduling, then schedule the raw pointer:

```cpp
std::unique_ptr<Command> index;

void opcontrol() {
  index = conveyor.makeIndexCommand(1500.0);
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
CommandScheduler::registerSubsystem(&conveyor);                       // uses the stored default command
CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());  // caller owned default command
CommandScheduler::unregisterSubsystem(&conveyor);                     // cancels its command, stops periodic()
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

### Migrating from `Arm`

`mechanism::Arm` is gone. It was a `PositionMechanism` with a `device::MotorGroup`
and a `device::Rotation` hardcoded inside it. Build the two devices yourself and
pass them in as lambdas:

| Before (`Arm`) | After (`PositionMechanism`) |
| --- | --- |
| `ArmConfig::motor_ports`, `gearset` | `device::MotorGroup motors(ports, gearset)`, wrapped in the voltage sink |
| `ArmConfig::rotation_port`, `rotation_reversed` | `device::Rotation sensor(port, reversed)`, wrapped in the position source |
| `ArmConfig::small_error_deg` / `big_error_deg` | `PositionMechanismConfig::small_error` / `big_error` |
| `kp`, `ki`, `kd`, `max_voltage`, `small_duration_ms`, `big_duration_ms`, `derivative_tolerance` | same names on `PositionMechanismConfig` |
| `Arm arm({-3, 4}, 8);` | build the `MotorGroup` and `Rotation` yourself and pass them as lambdas |
| `moveTo`, `setManualVoltage`, `positionDeg`, `targetDeg`, `atTarget` | same names, same meaning |
| `makeMoveToCommand`, `makeManualCommand`, `makeStopCommand` | same names, same meaning |
| `stop()` cut the motors to 0 V | `stop()` holds the present position; use `setManualVoltage(0.0)` to coast |

Two behaviour changes to know about:

- `stop()` now brakes instead of coasting, as described above.
- `Arm` inherited the `PID` arrival latch, so once it settled it output a hard
  0 V and a loaded arm sagged. `PositionMechanism` re-arms the loop when the
  position drifts back outside `small_error`, and `atTarget()` stays true across
  that re-arm, so a finished move does not restart.

`PositionMechanism` is not tied to a rotation sensor. The position source is any
`std::function<double()>`, so a motor encoder or a potentiometer works the same
way.

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
latching on the motors; set `drive_timeout` to `0 * millisecond` to keep the
last value.

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
## PresetPositionMechanism: presets on top of a PID loop

`PresetPositionMechanism<StateT>` is `MultiPositionMechanism` and
`PositionMechanism` wired together. The first owns the ordered preset table and
`next` / `previous` / `cycle`; the second owns the PID loop, the tolerance and
dwell exit, the voltage clamp and the manual-voltage override. Neither is
re-implemented. Use it for an articulated joint, a lift, a tilter, an indexer:
anything with discrete stops. The preset enum is yours.

```cpp
enum class ArmPreset { Down, Load, Score };

mclib::device::MotorGroup arm_motors({-11, 12});
mclib::device::Rotation arm_sensor(13);

mclib::mechanism::PresetPositionMechanism<ArmPreset> arm(
    ArmPreset::Down,
    {{ArmPreset::Down, 0.0}, {ArmPreset::Load, 45.0}, {ArmPreset::Score, 130.0}},
    [&]() { return arm_sensor.getAngleDeg(); },
    [&](double volts) { arm_motors.setVoltage(volts); },
    mclib::mechanism::PositionMechanismConfig{.kp = 0.12, .small_error = 1.5});

void initialize() {
  arm_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  arm.setName("arm");
  arm.setDefaultCommand(arm.idleCommand());
  arm.registerSelf();
}
```

One registration, not two. `MultiPositionMechanism` and `PositionMechanism` are
both `Subsystem`s, and registering both for one physical mechanism would let two
commands drive it at the same time. So the class *derives* from
`MultiPositionMechanism` (that is the scheduler requirement) and *owns* the
`PositionMechanism` as a component that is never registered; it is ticked from
the outer `applyState()`. `controller()` exposes it for tuning and diagnostics.
Do not register what `controller()` returns.

API: `setPreset(preset)`, `getPreset()`, `next()`, `previous()`, `cycle()`,
`moveTo(raw)`, `positionValue()`, `targetValue()`, `presetSetpoint()`,
`atTarget()`, `isManual()`, `setManualVoltage(volts)`, `stop()`.

Commands: `makePresetCommand(preset, timeout_ms = 0.0)` and
`makeMoveToCommand(raw, timeout_ms = 0.0)` finish on `atTarget()` or on the
timeout, and brake if interrupted. `makeNextCommand()`, `makePreviousCommand()`
and `makeCycleCommand()` are one-shots. `makeManualCommand(volts)` and
`makeStopCommand()` hold the requirement for as long as they run.

Behaviors worth knowing:

- Nothing drives until you ask. A fresh mechanism sits at 0 V with the loop
  disengaged, like a bare `PositionMechanism`. The first `setPreset()`,
  step, `moveTo()`, `setManualVoltage()` or `stop()` engages it.
- The table pushes its setpoint every tick, but the mechanism only retargets
  when the setpoint actually changed or when a retarget was asked for. Feeding
  the sink straight into `moveTo()` would reset the PID 50 times a second and
  the loop would never settle.
- Re-issuing the *same* preset after a manual override still takes control
  back. `setState` only fires `onStateChanged` on a real value change, so
  `setPreset()` requests the retarget explicitly.
- `next()` at the last preset and `previous()` at the first do nothing at all
  while the table is in charge: no PID reset, no dwell restart, `atTarget()`
  does not flip back to false. If a manual voltage or a raw target *was* in
  charge, a clamped step still hands control back to the table, otherwise a
  latched manual voltage would keep driving.
- `setPreset()` restarts the loop on every call, which is what takes control
  back from a manual override. Do not call it every tick. `holdPreset()` is
  the idempotent version for commands that execute every frame, and it is what
  `makeStateCommand` / `makeStateUntilCommand` use.
- `moveTo(raw)` and `setManualVoltage(volts)` put the table on hold so it
  cannot pull the mechanism back. After `moveTo()` and `stop()` the reported
  preset snaps to the entry with the closest setpoint, so `getPreset()` is
  never stale and a following `next()` steps from somewhere sensible.
- `stop()` brakes, it does not coast: the present position becomes the target
  and the loop holds it. Pair it with `BrakeMode::Hold` on the motors so a
  raised mechanism cannot back-drive. Use `setManualVoltage(0.0)` to go limp.
- Neither copyable nor movable: the setpoint sink and the command factories
  capture `this`.
## AutoTriggerMechanism: auto-fire on a sensor edge

`AutoTriggerMechanism` watches a `std::function<bool()>` and calls a
`std::function<void()>` when it goes from false to true. That is the whole
type. It knows nothing about what the sensor is or what the action does, which
is why it covers auto-clamping on a goal, auto-indexing on a detected object,
auto-retracting on a limit switch, and auto-stopping on a proximity reading with
one implementation.

Composed with a `ToggleMechanism`, it replaces a hand-written game-specific
clamp subsystem:

```cpp
#include "mclib/mclib.hpp"

using namespace mclib;

// The thing being actuated: a solenoid, nothing game-specific about it.
mechanism::ToggleMechanism grabber(std::make_shared<device::Pneumatic>('H'));

device::Distance sensor(7);

// The sensor gate. Note what it returns for "nothing there": false, not
// "unknown". With no re-arm condition the latch clears only on a false
// reading, so a gate that swallows the absent case latches forever.
//
// confidence >= 10, not 50: PROS pins distance confidence at 10 for anything
// under 200 mm, so a higher floor discards every reading a close-range
// mechanism would ever see. 0 mm is a real reading, for a target inside the
// sensor's ~20 mm minimum range, so it is not an error. The error value is
// PROS_ERR, which is INT32_MAX, not a negative number: an `mm < 0` guard never
// fires and lets INT32_MAX through as a very large distance, which reads as
// true for any "far enough away" threshold.
auto goal_in_range = []() {
  const std::int32_t mm = sensor.getDistanceMm();
  const std::int32_t confidence = sensor.getConfidence();
  if (mm == PROS_ERR || confidence == PROS_ERR || confidence < 10) {
    return false;  // bad read: treat as "nothing there" so the latch clears
  }
  return mm <= 60;
};

mechanism::AutoTriggerConfig auto_grab_config{
    .debounce_ms = 60.0,        // the reading must hold for 60 ms
    .enabled_on_construct = true,
    .fire_once = false,         // keep watching after each grab
};

mechanism::AutoTriggerMechanism auto_grab(
    goal_in_range, []() { grabber.extend(); }, auto_grab_config);

void initialize() {
  grabber.setName("grabber");
  grabber.setDefaultCommand(grabber.idleCommand());
  grabber.registerSelf();

  auto_grab.setName("auto_grab");
  auto_grab.setDefaultCommand(auto_grab.idleCommand());
  auto_grab.registerSelf();

  // Re-arm when the grabber is open again. Without this the latch clears as
  // soon as the sensor stops seeing the target, which is fine for most
  // mechanisms but wrong here: the target is being carried, so it stays in
  // range the whole time.
  auto_grab.setRearmCondition([]() { return !grabber.isExtended(); });
}
```

`disarmUntilReset()` is the manual override. It suppresses the fire that has
not happened yet: the driver opens the grabber ahead of a target they do not
want picked up, and the watcher stays out of the way until the target has
passed and a fresh edge arrives.

```cpp
// Driver pre-emptively opens the grabber. Without disarmUntilReset() the
// watcher closes it again as soon as the target comes into range.
void skipThisOne() {
  grabber.retract();
  auto_grab.disarmUntilReset();
}
```

Releasing a target that is already held does not need it: the fire that closed
the grabber left the edge consumed, and the edge only comes back once the sensor
reads false, which it does not while the target is still in the clamp.

Rules the type follows:

- **Edge, not level.** The action fires on the false to true transition of the
  debounced condition, never on every tick the condition holds.
- **The latch survives a manual override.** After a fire (or a
  `disarmUntilReset()`), no further fire happens until the latch clears *and* a
  fresh false to true edge arrives. Clearing the latch while the condition is
  still true does not fire.
- **The latch clears on a false reading** when no re-arm condition was supplied,
  and only on the re-arm condition when one was.
- **`debounce_ms` needs an unbroken run.** One false reading restarts the
  window, so a flickering sensor never accumulates enough time.

Autonomous routines use `makeWaitForTriggerCommand(timeout_ms)`, which arms the
watcher, finishes the moment the action fires (or on timeout), and restores the
armed state it found unless something changed it while the command ran:

```cpp
// Drive forward until the sensor sees the target and the grabber closes.
auto grab_it = auto_grab.makeWaitForTriggerCommand(2000.0);
```

`makeArmCommand()` and `makeDisarmCommand()` are one-shot and finish
immediately. `setArmed()` / `isArmed()` are this mechanism's own switch and are
separate from `Subsystem::setEnabled()`, which gates `periodic()` for the whole
scheduler. `hasFired()`, `fireCount()`, `isLatched()` and `isConditionMet()`
report state; `reset()` clears everything but the armed flag.

`AutoTriggerMechanism` is neither copyable nor movable: the condition and action
callbacks routinely capture `this`, and moving the object would leave them
pointing at the old address.

## Time (`mclib/time.hpp`)

`mclib::time::millis()` returns milliseconds since the program started, exactly
like `pros::millis()` does, and `mclib::time::now()` returns the same value as a
`QTime`.

```cpp
#include "mclib/time.hpp"

const std::uint32_t t = mclib::time::millis();  // raw milliseconds
const QTime now = mclib::time::now();           // same value as a QTime
```

`time.hpp` includes no PROS header. `src/mclib/time.cpp` is the single
translation unit in the library that reads `pros::millis()`, through the
out-of-line function `mclib::time::systemMillis()`. That is what `millis()`
calls when no clock has been installed, so the clock is live from the first
static constructor onwards - there is no initialisation order to get wrong -
and every caller's undefined reference to it forces the linker to pull
`time.cpp.o` out of `mclib.a`.

### Converted so far

`pid.cpp`, `chassis_controller.cpp`, `mechanism.hpp`, `preset_position_mechanism.hpp`,
`waitCommand.h` and the mechanism sources (`auto_trigger`, `conveyor`, `homing`,
`position`, `pto`, `toggle_group`, `velocity`) all read time through the seam.
`control/motion.cpp`, `control/odometry.cpp` and `auton/autonomous_routine.cpp`
still call `pros::millis()` and `pros::delay()` directly; they are Phase 3 work.
Until then a fake clock does not affect those loops, so do not mix a
`ScopedClock` with a routine that drives them.

### Host tests

Because the seam is a function pointer, a host test can install its own clock,
step it by hand, and check timing behaviour without a robot or the PROS
toolchain. A host build does not link `time.cpp`, so it supplies its own
`systemMillis()` - one line, used only before a fake clock is installed:

```cpp
namespace mclib {
namespace time {
std::uint32_t systemMillis() { return 0; }
}  // namespace time
}  // namespace mclib

static std::uint32_t fake_ms = 0;

{
  mclib::time::ScopedClock clock([]() { return fake_ms; });

  PID pid(1.0, 0.0, 0.0);
  pid.setTarget(10.0);
  for (int tick = 0; tick < 6; ++tick) {
    fake_ms = tick * 20;
    pid.update(10.0);
  }
  // small settle window is 100 ms, so arrival latches on the tick at t = 100 ms
  assert(pid.targetArrived());
}
// ScopedClock put the previous clock back here
```

`setClock()` installs a clock and returns the previous one, `getClock()` reports
it, and `restoreSystemClock()` goes back to the platform clock. `ScopedClock`
does the save/restore for you.

`src/mclib/pid.cpp` compiles and links with no PROS headers reachable at all,
which is what makes host-side testing of the control code possible.

## Telemetry: CSV logging to the SD card

Tuning a PID by watching the robot and guessing is the slowest way to do it.
`mclib/telemetry/telemetry.hpp` gives you a fixed-rate CSV log on the V5 SD
card, with columns that say what unit they are in.

```cpp
#include "mclib/telemetry/telemetry.hpp"

void autonomous() {
  // A Logger is one-shot: register, sample, close. Build a fresh one per run
  // so a second autonomous over field control gets a second log. The ring
  // storage is static because sizeof(Row) is ~200 bytes and 256 rows has no
  // business on a task stack; the logger is not.
  static mclib::telemetry::RowBuffer<256> tlm_buffer;
  mclib::telemetry::SdCardSink tlm_sink("auton");
  mclib::telemetry::Logger tlm(tlm_sink, tlm_buffer,
                               {.period = 20 * mclib::units::millisecond});

  // Register channels before the first sample. One line each.
  auto x = tlm.addLength("x");            // column "x_in"
  auto heading = tlm.addAngle("heading"); // column "heading_deg"
  auto cmd = tlm.addVoltage("left_cmd");  // column "left_cmd_V"
  auto err = tlm.addNumber("error");      // column "error", no unit

  // Starts a background pros::Task that does the SD writes. Declared after the
  // logger, so it is destroyed first and never outlives what it drains.
  mclib::telemetry::FlushTask flusher(tlm);

  while (!controller.settled()) {
    // ... control tick ...
    tlm.set(x, pose.x);
    tlm.set(heading, pose.theta);
    tlm.set(cmd, left_command);
    tlm.set(err, controller.error());
    tlm.sample();  // no-op until the 20 ms period elapses
    pros::delay(10);
  }
  flusher.stop();  // joins the task, drains the buffer, closes the file
}
```

If the logger has to live longer than one run - shared between auton and
driver control, say - keep it in a `std::optional` and re-construct it at the
start of each run. Registration, sampling and `close()` are all one-way on a
given `Logger`: once `close()` has run, that object is finished.

The file lands at `/usd/auton000.csv`, then `auton001.csv`, and so on - the
sink picks the first index that does not already exist, so a re-run never
overwrites the previous match's log. Output looks like this:

```
t_ms,x_in,heading_deg,left_cmd_V,error
0,0,0,12,24
20,1.4,0.2,12,22.6
40,3.1,0.4,11.8,20.9
...
# rows=412 dropped=0
```

Column names carry the unit, which is what makes a log readable a week later.
`24_in` written into an `addLength` channel reads back as `24.0` in the column
called `x_in`.

### What it costs the control loop

`set()` is one double store. `sample()` is a clock read, and on a sampling tick
a `sizeof(Row)` copy (about 200 bytes) into a lock-free ring plus one release
store. No allocation, no lock, no file I/O on the producer side. That is the
whole worst case a control tick pays - sub-microsecond on the V5's Cortex-A9,
and independent of how slow or jittery the SD card is.

The writing happens in `FlushTask`, a `pros::Task` at priority
`TASK_PRIORITY_DEFAULT - 1`, which drains the ring every 100 ms by default.
Batching means one `fwrite` per five rows at a 20 ms period rather than one per
row. The ring is single-producer / single-consumer over two `std::atomic`
indices, so the control loop never waits on the flush task and never waits on
the card.

### Bounded resources

Nothing here can grow without limit:

- **Buffer full: drop newest.** The new row is discarded and the buffered rows
  are kept, so the log is a contiguous prefix of the run with a gap at the end.
  A contiguous prefix beats a log with a hole punched in the middle, and it is
  the only policy a lock-free SPSC ring can offer without the producer racing
  the consumer's read cursor. Dropped rows are counted and the count is written
  into the CSV trailer, so a truncated log says so.
- **Row cap.** `LoggerConfig::max_rows` defaults to 30000 - ten minutes at
  20 ms. Past it the logger stops committing.
- **File size cap.** `SdSinkConfig::max_bytes` defaults to 4 MiB. Past it the
  sink closes the file. A log that fills the card mid-match is worse than none.
- **Channel cap.** 24 columns, names truncated to 32 characters. Registering a
  25th channel returns an invalid handle; writing through one is ignored.

### No SD card

`SdCardSink` probes for the card exactly once, on the first flush, by opening
the file. If there is no card the sink reports itself unavailable and every
later call is a no-op: no exceptions, no crash, no retry storm. The logger
still drains its ring so the control loop never wedges behind a dead sink. The
same holds for a card that dies mid-match.

### Testing without a card

The sink is an interface. `MemorySink` captures everything in a `std::string`
and `NullSink` is permanently unavailable, so `tests/telemetry_test.cpp` drives
the logger under a `mclib::time::ScopedClock` and asserts on real CSV text -
the header, the timestamps, the drop policy, and the round-trip of `24_in` back
to `24.0`. Only `src/mclib/telemetry/flush_task.cpp` includes a PROS header;
the logger and both test sinks are header-only.

## Motion profiling and drivetrain feedforward

`mclib/control/profile.hpp` and `mclib/control/feedforward.hpp`.

Every motion in this library used to be a PID against a position error. The
only way to make one faster was to raise `kp` until it oscillated, and the only
acceleration limit was `max_slew_accel_fwd` and friends - a rate limit on
*voltage*, which is a guess at a rate limit on acceleration. A profile plus a
feedforward model replaces both: the profile says what velocity to be at right
now, the model says what voltage that velocity costs, and the PID only has to
clean up the difference.

Both files are pure arithmetic over `mclib::units` - no PROS, no hardware, no
clock inside the profile at all - so `tests/profile_test.cpp` and
`tests/feedforward_test.cpp` check every number on the host.

### The profile

```cpp
using namespace mclib::control;

ProfileConstraints limits{
    .max_velocity = 48 * mclib::units::inps,
    .max_acceleration = 96 * inps2,
};
const MotionProfile profile = MotionProfile::generate(48_in, limits);
const ProfileState now = profile.sample(750_ms);
// now.position == 24 in, now.velocity == 48 in/s, now.acceleration == 0
```

That worked example: 48 inches at 48 in/s with 96 in/s² both ways is 0.5 s of
acceleration over 12 inches, 0.5 s of cruise over 24 inches, 0.5 s of
deceleration over 12 inches. **Total 1.5 s, final position 48.000000000 in.**

`ProfileConstraints` carries four magnitudes, two of which have a "zero means
something else" convention that keeps the common case a two-field aggregate:

| Field | Zero means |
| --- | --- |
| `max_velocity` | nothing - a zero here gives an empty profile |
| `max_acceleration` | nothing - a zero here gives an empty profile |
| `max_deceleration` | same as `max_acceleration` |
| `max_jerk` | unbounded, i.e. `generate()` builds a trapezoid |

Set `max_jerk` and `generate()` builds an S-curve instead: acceleration ramps
in and out at the jerk limit, so the voltage command has no steps in it.
Over the same 48 inches with a 384 in/s³ jerk limit that is 0.75 s of ramp
over 18 inches, 0.25 s of cruise over 12 inches, 0.75 s of ramp down - **1.75 s
total**. Smoothness costs a quarter of a second.

The cases that break naive implementations are all defined behaviour and all
pinned by tests:

- **Too short to cruise.** 6 inches under the same limits is a triangle
  peaking at 24 in/s after 0.25 s, and it still lands exactly on 6 inches.
- **Asymmetric accel/decel.** This drivetrain already has four slew constants,
  so `max_deceleration` is separate and `DirectionalConstraints` holds a
  forward set and a reverse set. 48 inches with 96 in/s² accel and 48 in/s²
  decel splits 0.50 s / 0.25 s / 1.00 s - 1.75 s total.
- **Zero distance.** Empty profile, `duration()` 0, `sample()` all zeros.
- **Negative distance.** A normal profile running the other way; velocity is
  negative throughout.
- **Non-zero initial velocity.** Already cruising at 48 in/s over 48 inches
  drops the acceleration phase entirely: 0.75 s of cruise, 0.5 s of decel,
  1.25 s. An initial velocity pointing *away* from the target accelerates back
  through zero at `max_acceleration`.
- **Too fast to stop.** 2 inches while doing 48 in/s needs 12 inches of
  braking room. The profile becomes a single deceleration at the limit,
  `overshoots()` returns true, and `netDisplacement()` reports the 12 inches it
  really covers. Refusing to build anything would be worse - the robot is
  moving either way and "brake at the limit" is the best command available.
  The mirror case is handled the same way: asking to *reach* 48 in/s within
  4 inches needs 12 inches of runway, so the profile accelerates at the limit,
  overshoots, and says so.

`DirectionalConstraints::select()` takes the initial velocity as a tiebreak, so
a zero-distance "stop from where you are" while rolling backwards gets the
reverse deceleration limit rather than the forward one.

For a path follower, `velocityAtDistance()` and `timeAtDistance()` give the
same profile parameterised by distance along the path instead of by a
stopwatch.

### The feedforward model

```
V = kS * sgn(v) + kV * v + kA * a
```

`kS` is volts, `kV` is volts per (in/s), `kA` is volts per (in/s²), and all
three are typed - handing `kV` a bare number is a compile error, not a loop
that is 39.37x wrong.

```cpp
SimpleMotorFeedforward model({.kS = 0.8_V,
                              .kV = 0.2_V / mclib::units::inps,
                              .kA = 0.02_V / inps2});
model.calculate(30 * mclib::units::inps);              // 6.8 V
model.calculate(30 * mclib::units::inps, 100 * inps2); // 8.8 V
model.maxAchievableVelocity(12_V, {});                 // 56 in/s
```

`kS` takes the sign of the velocity, or of the acceleration when the velocity
setpoint is exactly zero, so the first tick of a motion still gets the static
term. With both zero the output is 0 V, not +kS: a robot that is meant to stand
still should not be pushed in an arbitrary direction.

### Identifying kS, kV and kA on your robot

A feedforward model nobody can measure is decoration, so both fits ship with
the library.

**kS and kV.** On a long clear stretch of field, or on blocks:

1. Command a fixed voltage to both sides of the drive. Start around 2 V.
2. Wait for the speed to stop changing - 500 ms is plenty.
3. Record the commanded voltage and the measured velocity as one
   `VelocitySample`. Take the velocity from odometry or from the drive
   encoders through `DriveGeometry::encoderToDistance()`, **not** from the
   motor's own `get_actual_velocity()`, which is already filtered.
4. Repeat in roughly 1 V steps up to 12 V. Eight to twelve points is plenty.
5. Do the whole run again in reverse and append those samples too.

```cpp
const VelocityFit fit = fitVelocityGains(samples, count);
// fit.kS, fit.kV, fit.r_squared, fit.used
```

Reverse samples are folded onto the forward branch internally - each sample's
voltage and velocity are both multiplied by the sign of its velocity - so a
bidirectional run fits one symmetric model rather than two half-models.
Samples slower than `min_speed` (1 in/s by default) are dropped, so the "3 V,
did not move" point cannot drag the intercept down. **Check `r_squared`.**
Below about 0.98 the data is wrong, not the model: samples taken before the
speed settled, a battery that sagged during the run, or a drivetrain binding.

**kA**, once kS and kV are known:

1. Ramp the voltage linearly 0 → 12 V over about two seconds, logging voltage,
   velocity and acceleration every tick.
2. Acceleration comes from differencing the velocity, and a raw difference of
   encoder velocity is mostly noise - filter it. A three-point central
   difference over a 10 ms loop is usually enough.
3. `fitAccelerationGain(samples, count, {.kS = fit.kS, .kV = fit.kV})`.

That fit is a least-squares line **through the origin** of the leftover
voltage `V - kS*sgn(v) - kV*v` against acceleration, because the model says
that residual is exactly `kA * a`. Fitting an intercept instead would silently
absorb an error in kS.

kA is the least important of the three and the hardest to measure. A model
with `kA` left at zero still takes most of the work off the PID; a fitted kA
you do not trust is worse than none.

### The follower

```cpp
ProfileFollower follower({
    .gains = {.kS = 0.8_V, .kV = kv, .kA = ka},
    .kp = 1.0_V / mclib::units::inch,
});
follower.follow(MotionProfile::generate(48_in, limits));

const QLength start = travelledDistance();
while (!follower.isFinished()) {
  const QVoltage command = follower.update(travelledDistance() - start);
  driveVoltage(command, command);
  pros::delay(10);
}
```

The PID inside runs with `setUseDt(true)` - this is new code, so there are no
gains fitted against the historical raw-delta numerics to protect, and real
rates are what `kd` should mean. It also runs with `setArrive(false)`: a
profile ends when the profile ends, and a latched arrival mid-cruise would zero
the output while the robot was still moving. The integral is clamped at 2 V by
default rather than PID's unbounded 0, because a follower stalled against a
wall would otherwise wind up the whole battery.

With a good model `kp` is small. It is correcting modelling error, not driving
the motion.

`calculate(setpoint, measured, dt)` is the seam a path follower uses: it brings
its own setpoint and its own timestep and never touches the follower's
stopwatch.

`attachTelemetry(&logger)` registers six columns - setpoint position and
velocity, measured position, error, and the feedforward/feedback split of the
command - and writes them on every `update()`. That split is the whole tuning
story: a feedback term that is large next to the feedforward means the model is
wrong, not that `kp` needs raising. The follower never calls `sample()`; the
control loop owns the cadence.
