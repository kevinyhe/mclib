# Setting up your robot

Ports, geometry, odometry and your first working opcontrol.

[Documentation index](README.md) · [Project README](../README.md)

Nothing in mclib knows what ports your motors are on. A robot program declares
its devices, builds a `Chassis` from them, and builds a `ChassisController`
on the Chassis. Constructing the controller does two things for the whole
library: it binds that Chassis as the drivetrain every motion routine in
`control/motion.hpp` runs on, and it installs its config as the one tuning
record every drive loop reads. `src/main.cpp` is a complete example; the core
of it is:

```cpp
#include "main.h"
#include "mclib/mclib.hpp"

using namespace mclib::units::literals;

mclib::device::Controller master;
auto imu = std::make_shared<mclib::device::Inertial>(15);

mclib::Chassis chassis(
    {-11, 13, 14},                       // left motors, negative = reversed
    {-16, 17, -18},                      // right motors
    mclib::device::Gearset::Blue,
    mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in),
                             11.375_in,  // track width
                             1.0},       // wheel revs per motor rev
    imu);

mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},   // volts per inch
    .turn_pid = {0.3, 0.0, 1.5},       // volts per degree
    .heading_pid = {0.3, 0.0, 1.5},    // volts per degree, while driving
});

void initialize() {
  imu->reset(true);
  mclib::control::startOdometry();     // heading + drive encoders, no tracking wheels

  static auto default_drive = drive.makeArcadeDriveCommand(master);
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
}

void autonomous() {
  moveToPoint(0_in, 24_in, 1, 2_s);
  turnToAngle(90_deg, 1_s);
}

void opcontrol() {
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
```

`ChassisControllerConfig` is `mclib::control::MotionConfig`
(`mclib/control/motion_config.hpp`): three gain sets, two exit rules, the
voltage cap, the stiction floor (`min_voltage`, 1.5 V by default), the slew
rates and the boomerang `chase_power`. Every field has a default from one
tuned robot; expect to retune the gains for yours. `drive.setConfig()` changes
the tuning for every loop at once.

There are no defaults for the geometry. `units::Wheel` has no constructor
taking a bare number, so you have to say which measurement you took:
`Wheel::fromDiameter(2.75_in)` for calipers across the wheel,
`Wheel::fromCircumference(9.06_in)` for a tape around the tread. A wrong wheel
size is a silent 5% scaling error on every autonomous. `mclib/robot_geometry.hpp`
is an optional place to write the numbers down once.

### Odometry

The odometry runs on its own task and is the only thing that writes the pose.
`startOdometry()` with no arguments reads heading and drive encoders from the
bound Chassis. To add tracking wheels, describe them:

```cpp
mclib::device::Rotation vertical_tracker(-6);        // forward-rolling wheel
mclib::device::AdiEncoder horizontal_tracker('A', 'B');  // sideways-rolling wheel

void initialize() {
  imu->reset(true);
  auto setup = mclib::control::odometrySetupFrom(chassis);
  setup.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker),
      mclib::units::TrackingWheel{mclib::units::Wheel::fromDiameter(2_in),
                                  0_in,      // offset to the robot's right
                                  1.0}};
  setup.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker),
      mclib::units::TrackingWheel{mclib::units::Wheel::fromDiameter(2_in),
                                  -3_in,     // offset ahead of centre
                                  1.0}};
  mclib::control::startOdometry(setup);
}
```

Any sensor with a `position()` returning `std::optional<QAngle>` works as a
tracking wheel through `encoderReader()`: `Rotation` and `AdiEncoder` both do.
A vertical wheel replaces the drive encoders for forward travel and is immune
to drive slip; a horizontal wheel catches sideways push. Without either, the
drive encoders measure forward travel and sideways motion is invisible.

Reset the IMU and the pose together. The odometry tracks heading as deltas
from the pose it was reset to, while the motion routines steer on the raw IMU
rotation; `Chassis::setPose()` writes both.

Without an IMU, a differential `Chassis` derives clockwise-positive heading from
left minus right wheel travel divided by track width. Set a positive measured
track width; wheel slip affects this estimate. `setPose()` and drive taring keep
the encoder heading frame aligned. Snapshot position corrections update the
odometry integrator as well as its published pose and preserve encoder baselines.

### Driver control

`ChassisController` has three teleop commands, each meant as the subsystem's
default command:

```cpp
mclib::control::DriveCurveConfig sticks{.deadzone = 0.05, .gain = 5.0};
mclib::control::DriveCurveConfig turn{.deadzone = 0.05, .gain = 10.0};

drive.makeArcadeDriveCommand(master, sticks, turn);     // left Y drives, right X turns
drive.makeCurvatureDriveCommand(master, sticks, turn);  // turn stick bends the path
drive.makeTankDriveCommand(master, sticks);             // each stick drives a side
```

`DriveCurveConfig` (`mclib/control/drive_curve.hpp`) shapes a stick: a
deadzone that stays continuous at its edge, an exponential curve whose `gain`
makes small deflections finer without losing full speed, a `min_output` to
step over drive stiction, and a `slew_per_tick` to limit how fast the command
can change. All default to off except the deadzone.

### Without a ChassisController

The blocking motion routines only need a bound drive. If you do not want the
command framework, bind the Chassis yourself and set the tuning:

```cpp
mclib::control::bindDrive(&chassis);
mclib::control::setMotionConfig(my_config);
```

With no drive bound every routine prints one diagnostic and returns at once,
so a program that forgot this fails loudly instead of spinning on a NaN
heading until its timeout.

### Commands for the motion routines

`ChassisController` also exposes command factories for the blocking routines
in `control/motion.hpp`, so an autonomous can be a command sequence:
`makeTurnToAngleCommand`, `makeDriveToCommand`, `makeCurveCircleCommand`,
`makeSwingCommand`, `makeWallResetCommand`, `makeTurnToPointCommand`,
`makeMoveToPointCommand`, and `makeBoomerangCommand`. Each launches the routine
on its own task and cancels it cooperatively when the command is interrupted.
The scheduler-driven `makeDriveDistanceCommand` and `makeTurnToHeadingCommand`
run a step per scheduler pass instead, on the same config.
