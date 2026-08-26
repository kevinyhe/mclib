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

## Intake API

`mclib::mechanism::Intake` derives from `MotorStateMechanism<IntakeState>` and
does not redeclare the state API. `setState`, `getState`, and the generic
`makeState*` command factories come from `StateMechanism<IntakeState>`, so
calling them through a base reference and through an `Intake` gives the same
behaviour.

```cpp
mclib::mechanism::Intake intake({
    .bottom_port = -20,
    .top_port = -21,
    .gearset = mclib::device::Gearset::Blue,
    .index_voltage = 12.0,
    .score_voltage = 12.0,
    .reverse_voltage = -12.0,
});

intake.setBrakeMode(mclib::device::BrakeMode::Coast);

intake.setState(mclib::mechanism::IntakeState::Score);
mclib::mechanism::IntakeState state = intake.getState();

const mclib::mechanism::IntakeConfig& cfg = intake.config();
std::vector<double> volts = intake.voltagesFor(mclib::mechanism::IntakeState::Index);
// volts == {cfg.index_voltage, 0.0} - Index drives only the bottom motor.

std::unique_ptr<Command> index = intake.makeIndexCommand();
std::unique_ptr<Command> score = intake.makeScoreCommand();
std::unique_ptr<Command> reverse = intake.makeReverseCommand();
std::unique_ptr<Command> idle = intake.makeDisableCommand();
```

- `config()` returns the `IntakeConfig` the intake was built from; it is the
  single source of truth for the voltages applied each `periodic()`.
- `voltagesFor(state)` returns `{bottom, top}` volts for a state without
  commanding the motors.
- `setBrakeMode(mode)` forwards to every motor via `motorCount()` / `motor(i)`.
- `disable()` is shorthand for `setState(IntakeState::Disabled)`.
