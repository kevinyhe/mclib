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

With a forward tracking wheel fitted, `driveTo()` measures its distance along
the odometry pose instead of the drive encoders, which count wheel slip as
progress. `MotionConfig::drive_distance_source` picks `Auto` (the default),
`Encoders` or `Pose`.

### Correcting odometry continuously: `PoseFilter`

`mclib/control/pose_filter.hpp`. Odometry drifts: every slip adds error that
stays. `PoseFilter` is a Kalman filter that keeps its own pose and how
uncertain it is, and blends in distance sensor and GPS readings as they arrive:

```cpp
mclib::control::PoseFilter filter;
filter.reset(chassis_start_pose, 0.5, 0.5);  // pose, sigma in, sigma deg

// Two sensors on one side, apart front to back, make heading correctable.
const snapshot::SensorGeometry left_front{-6.0f, 5.0f, -90.0f};
const snapshot::SensorGeometry left_back{-6.0f, -5.0f, -90.0f};

// Every loop:
filter.predict(previous_odometry_pose, odometry_pose);
filter.updateDistance(left_front, left_front_sensor.distance().in());
filter.updateDistance(left_back, left_back_sensor.distance().in());
// filter.pose() is the corrected pose.
```

- `predict()` moves the estimate by the odometry step and grows its
  uncertainty with the distance travelled and angle turned.
- `updateDistance()` compares a reading with the range the field walls give
  from the current estimate (`snapshot::predict_range()`), and corrects by
  an amount set by both uncertainties. A reading more than 3 standard
  deviations off (a robot or game object in the way) is rejected.
- `updateGps()` blends in a GPS pose.
- The field frame is the snapshot map's: 0-144 in on both axes.

In the host test, a robot drives four 60 in square laps with odometry that
counts 4% too far and turns 91° per 90°. Raw odometry ends 12.18 in off. With
four wall sensors at 2% noise, a tenth of their readings blocked, the filter
ends 0.20 in and 0.58° off, and rejects the blocked readings.

In the physics simulator, with the same four sensors reading the true pose
against a 144 in perimeter at 2% noise, the filter's position is pulled back
into odometry each tick, by at most 60% of the distance moved that tick plus
0.002 in. The limit keeps the pose still while a robot settles on a target;
correcting all at once made it jump with sensor noise, and point moves timed
out.

| Drive encoders only | Odometry alone | With the filter |
| --- | --- | --- |
| speed_base boomerang | 10.05 in off (odometry 9.53) | 3.16 in (odometry 2.06) |
| six_motor_450 boomerang | 6.74 in (5.79) | 1.11 in (0.26), passes |
| six_motor_450 point move | 2.56 in (1.32) | 1.33 in (0.47), passes |
| four_motor_200 RAMSETE arc | 4.06 in (4.32) | 2.36 in (2.03), passes |

With drive encoders only, 5 of the 15 moves pass with the filter against 2
without. With two tracking wheels the odometry is already within 0.1 in, and
the filter adds 0.1-0.3 in of sensor noise to it: 6 of 15 pass either way.
Use it on robots without tracking wheels. `curveCircle()` arcs don't improve
either way, because they steer on the encoders, not the pose.

To have the motions steer on the corrected pose, hand the odometry task a
`PoseFusion`. It runs the filter after every odometry tick and writes the
corrected position back, with the limit above. The simulator's bridge runs the
same class.

```cpp
mclib::device::Distance left_front_sensor(3), left_back_sensor(4);
auto read = [](mclib::device::Distance& sensor) {
  return [&sensor] {
    const auto d = sensor.distance();
    return d.has_value() ? d->in() : -1.0;
  };
};

mclib::control::PoseFusionConfig fusion_config;
// Odometry is already in field coordinates (0-144 in) here. If it starts at
// 0 in the field centre instead, use 72 and 72.
fusion_config.field_offset_x_in = 0;
fusion_config.field_offset_y_in = 0;
mclib::control::PoseFusion fusion(fusion_config, {
    {{-6.0f, 5.0f, -90.0f}, read(left_front_sensor)},
    {{-6.0f, -5.0f, -90.0f}, read(left_back_sensor)},
});

void initialize() {
  auto setup = mclib::control::odometrySetupFrom(chassis);
  setup.fusion = &fusion;   // must outlive the task
  mclib::control::startOdometry(setup);
}
```

`fusion.pose()`, `accepted()`, `rejected()` and `positionSigmaIn()` can be read
from any task. Heading stays with the IMU; only position is corrected.

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

If no drive is bound, every motion routine prints an error and returns
`MotionResult::NoDrive`.

## Motion commands

`ChassisController` wraps each blocking routine as a command:
`makeTurnToAngleCommand`, `makeDriveToCommand`, `makeCurveCircleCommand`,
`makeSwingCommand`, `makeWallResetCommand`, `makeTurnToPointCommand`,
`makeMoveToPointCommand`, `makeBoomerangCommand` and
`makeFollowTrajectoryCommand`. Each runs the routine on its own task and
cancels it when interrupted.

`makeDriveDistanceCommand` and `makeTurnToHeadingCommand` run one step per
scheduler pass instead.
