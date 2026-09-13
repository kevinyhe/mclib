# Getting started

Declare the devices, build a `Chassis`, then build a `ChassisController` on it.
The controller binds the chassis as the drivetrain for every routine in
`control/motion.hpp` and sets the tuning they use. `src/main.cpp` is a complete
example.

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

## Configuration

`ChassisControllerConfig` is `mclib::control::MotionConfig`
(`control/motion_config.hpp`). It holds:

- distance, turn and heading PID gains
- two exit rules
- the voltage cap
- `min_voltage`, the minimum output needed to get the drive moving (default 1.5 V)
- slew rates (how fast output voltage may change)
- `chase_power` for `boomerang` moves

The defaults come from one robot. Retune for yours. `drive.setConfig()`
updates every loop.

Geometry has no defaults. `units::Wheel` is built from a measurement:

- `Wheel::fromDiameter(2.75_in)`
- `Wheel::fromCircumference(9.06_in)`

`mclib/robot_geometry.hpp` is an optional place to keep these values.

## Odometry

Odometry runs on its own task and is the only writer of the pose.
`startOdometry()` with no arguments uses the chassis heading and drive encoders.
To add tracking wheels:

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

Any sensor whose `position()` returns `std::optional<QAngle>` works with
`encoderReader()`, including `Rotation` and `AdiEncoder`.

| Sensors | Forward travel | Sideways travel |
| --- | --- | --- |
| Drive encoders only | drive encoders | not measured |
| Vertical tracker | tracker | not measured |
| Vertical and horizontal trackers | tracker | tracker |

Reset the IMU and the pose together. `Chassis::setPose()` sets both.

Without an IMU, a differential `Chassis` computes heading from the difference in
left and right wheel travel divided by track width. Set a measured, positive
track width. Wheel slip affects this estimate.

Snapshot position corrections update the odometry integrator and keep encoder
baselines.

## Driver control

```cpp
mclib::control::DriveCurveConfig sticks{.deadzone = 0.05, .gain = 5.0};
mclib::control::DriveCurveConfig turn{.deadzone = 0.05, .gain = 10.0};

drive.makeArcadeDriveCommand(master, sticks, turn);     // left Y drives, right X turns
drive.makeCurvatureDriveCommand(master, sticks, turn);  // turn stick bends the path
drive.makeTankDriveCommand(master, sticks);             // each stick drives a side
```

`DriveCurveConfig` (`control/drive_curve.hpp`):

| Field | Effect |
| --- | --- |
| `deadzone` | ignores small stick values; output stays continuous at the edge |
| `gain` | exponential curve for finer control near center |
| `min_output` | minimum output outside the deadzone |
| `slew_per_tick` | limits how fast the output changes |

All are off by default except the deadzone.

## Without a ChassisController

```cpp
mclib::control::bindDrive(&chassis);
mclib::control::setMotionConfig(my_config);
```

If no drive is bound, every motion routine prints an error and returns.

## Motion commands

`ChassisController` wraps each blocking routine as a command:
`makeTurnToAngleCommand`, `makeDriveToCommand`, `makeCurveCircleCommand`,
`makeSwingCommand`, `makeWallResetCommand`, `makeTurnToPointCommand`,
`makeMoveToPointCommand` and `makeBoomerangCommand`. Each runs the routine on
its own task and cancels it when interrupted.

`makeDriveDistanceCommand` and `makeTurnToHeadingCommand` run one step per
scheduler pass instead.
