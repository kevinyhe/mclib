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
- three exit rules: distance, turn and arc
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
| Horizontal tracker | drive encoders | tracker |
| Vertical and horizontal trackers | tracker | tracker |
| Accelerometer, with either of the first two | as without it | estimated from the IMU |

Which wheel to fit first, from the physics simulator. All three simulated
drives have omni wheels on every corner, which slide sideways in turns; a
drive with traction wheels slides less, which favours a vertical tracker
more than this does. Odometry error at the end of a move, mean / worst:

| Sensors | Straight moves | Curves (arcs, boomerang, S-curve) | Scenarios passing |
| --- | --- | --- | --- |
| Drive encoders only | 0.43 / 0.78 in | 4.56 / 9.79 in | 100 of 130 |
| Vertical tracker | 0.15 / 0.19 in | 5.08 / 10.72 in | 100 of 130 |
| Horizontal tracker | 0.78 / 1.53 in | 0.88 / 3.35 in | 120 of 130 |
| Both | 0.15 / 0.19 in | 0.09 / 0.11 in | 123 of 130 |

On curves the drive-encoder error is almost all sideways sliding, which only
a horizontal tracker sees. Forward slip is small on these drives. With
`PoseFusion`, set `odom_along_var_per_in` and `odom_side_var_per_in` apart
for one tracker: low in the direction it measures, about 0.09 in the other.

### No sideways wheel: the accelerometer

Without a horizontal tracker the odometry assumes the robot never moves
sideways. In a fast curve an omni drive slides outward, and the odometry
misses all of it. `OdometrySetup::accelerometer` estimates that slide from
the IMU's accelerometer:

```cpp
void initialize() {
  imu->reset(true);
  auto setup = mclib::control::odometrySetupFrom(chassis);
  mclib::control::AccelerometerSensor accelerometer{
      .right_ips2 = mclib::control::sidewaysAccelReader(
          *imu, mclib::device::ImuAxis::MinusY)};
  accelerometer.slide.imu_forward_in = 3.0;   // IMU ahead of the tracking centre
  accelerometer.slide.imu_right_in = -2.0;    // and to its right: 2 in left
  setup.accelerometer = accelerometer;
  mclib::control::startOdometry(setup);
}
```

- The axis is the IMU's own axis that points to the robot's right. Show
  `imu->accelerationG(PlusX)`, `PlusY` and `PlusZ` on the screen, push the
  robot to its right by hand, and see which one jumps and which way.
- Measure where the IMU sits from the tracking centre. Spinning at 6 rad/s,
  an IMU 3 in off the centre reads 0.28 g that is the turn, not a slide.
  `slide.imu_position_sigma_in` says how well you measured it: 0.25 in by
  default.
- Keep the robot still from `startOdometry()` until it first moves, as
  calibrating the IMU needs anyway. The odometry learns the accelerometer's
  zero then. The chip's zero can be 40 mg off once soldered; added up for one
  second that is 15 in/s.
- It works with drive encoders or a vertical tracker, not with a horizontal
  tracker, which measures the slide itself. `startOdometry()` refuses the
  pair.

In the physics simulator, every move again with the accelerometer added.
vexsim's own accelerometer is cleaner than a real one, so the runner adds a
real IMU's errors: zero, sensitivity, a tilting floor, shaking and a
misplaced IMU. Some of those numbers are guesses (`tests/vexsim/README.md`
lists them). Odometry error at the end of a move, mean / worst, over four
drives (three with a vertical tracker):

| Sensors | Straight moves and turns in place | Curves: arcs, boomerangs, swings, S-curves, point moves | Moves passing |
| --- | --- | --- | --- |
| Drive encoders only | 0.24 / 0.78 in | 4.24 / 9.79 in | 42 of 64 |
| Drive encoders and accelerometer | 0.33 / 0.79 in | 1.52 / 5.50 in | 59 of 64 |
| Vertical tracker | 0.10 / 0.19 in | 4.53 / 10.72 in | 31 of 48 |
| Vertical tracker and accelerometer | 0.14 / 0.29 in | 1.14 / 6.52 in | 47 of 48 |

What it costs:

- A move that turns hard and then drives (`moveToPoint()` to a point off
  to the side) can end up worse with drive encoders alone. The drive wheels
  slip in the turn, the odometry takes the turn for forward travel, and the
  accelerometer's turn pull is worked out from that. Two drives failed their
  point move this way: 2.08 to 5.50 in and 1.45 to 4.21 in off. A vertical
  tracker removes it.
- S-curves gain little. The two turns' slides cancel without the
  accelerometer; with it the errors of the accelerometer add up instead.
- A turn in place drifts up to 0.4 in.

The Override cascade drive (drive encoders only) ends its RAMSETE arcs 0.9 to
1.1 in off instead of 6.7 in, and its boomerang 2.6 in instead of 4.7 in.

None of this has run on a robot. To check it on yours, drive a full circle
back to where you started, with and without the accelerometer, and see which
pose on the screen ends closer to the start.

Reset the IMU and the pose together. `Chassis::setPose()` sets both. Pass
units so the heading is in degrees:

```cpp
chassis.setPose(0_in, -58_in, 180_deg);
```

`Pose2D::theta` is radians. `chassis.setPose(mclib::Pose2D(0, 0, 180))` sets a
heading of 180 rad, which is about 233°.

Without an IMU, a differential `Chassis` computes heading from the difference in
left and right wheel travel divided by track width. Set a measured, positive
track width: `tools/tune_drive.py` prints it ("track width for heading from
encoders") from a `characterizeSpin()` log taken with an IMU fitted, or spin
the robot 10 turns by hand and divide the difference in left and right travel
by 20π. Wheel slip still affects this estimate: in the physics simulator, a
robot with spring-loaded tracking wheels spun its drive wheels at the start of
a 12 V turn and ended 31° off.

Snapshot position corrections update the odometry integrator and keep encoder
baselines.

With a forward tracking wheel fitted, `driveTo()` measures its distance along
the odometry pose instead of the drive encoders, which count wheel slip as
progress. `MotionConfig::drive_distance_source` picks `Auto` (the default),
`Encoders` or `Pose`.

Near the target, a distance PID's output shrinks to about what the drive needs
to overcome friction, and the robot crawls. Stopped distance moves keep at
least `MotionConfig::settle_min_voltage` (1.5 V by default) until they are
inside `distance_exit.small_error`. Set it above your drive's kS, or to 0 to
turn it off.

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

// A sensor with nothing in range reports 9999 mm. The filter skips a reading
// that is zero, negative or NaN, so pass -1 for anything past the sensor's
// 78 in range.
auto inches = [](mclib::device::Distance& sensor) {
  const double reading = sensor.getDistanceIn();
  return reading < 78.0 ? reading : -1.0;
};

// Every loop:
filter.predict(previous_odometry_pose, odometry_pose);
filter.updateDistance(left_front, inches(left_front_sensor));
filter.updateDistance(left_back, inches(left_back_sensor));
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
| speed_base boomerang | 9.40 in off (odometry 9.34) | 2.28 in (odometry 1.56) |
| six_motor_450 boomerang | 5.90 in (5.75) | 0.84 in (0.68), passes |
| six_motor_450 point move | 2.56 in (1.32) | 1.33 in (0.47), passes |
| four_motor_200 RAMSETE arc | 4.06 in (4.32) | 2.36 in (2.03), passes |

With drive encoders only, 5 of the 15 moves pass with the filter against 2
without. With two tracking wheels the odometry is already within 0.1 in, and
the filter adds 0.1-0.3 in of sensor noise to it: 6 of 15 pass either way.
Use it on robots without tracking wheels.

To have the motions steer on the corrected pose, hand the odometry task a
`PoseFusion`. It runs the filter after every odometry tick and writes the
corrected position back, with the limit above. The simulator's bridge runs the
same class.

```cpp
mclib::device::Distance left_front_sensor(3), left_back_sensor(4);
auto read = [](mclib::device::Distance& sensor) {
  return [&sensor] {
    const auto d = sensor.distance();
    // Nothing in range reads 9999 mm, so anything past 78 in is no reading.
    return d.has_value() && d->in() < 78.0 ? d->in() : -1.0;
  };
};

// Odometry is already in field coordinates (0-144 in) here. If it starts at
// 0 in the field centre instead, use 72 and 72.
mclib::control::PoseFusionConfig fusion_config{.field_offset_x_in = 0,
                                               .field_offset_y_in = 0};
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

When the pose is set, by `setPose()`, `wallReset()` or a snapshot, the task
restarts the filter at the new pose. A restart is no surer of the position
than the filter was before it.

### GPS

`PoseFusion::setGps()` adds a V5 GPS sensor, with or without distance
sensors. It corrects position only; heading stays with the IMU. The GPS
reports from the field centre, so `GpsInput::origin_x_in` and `origin_y_in`
(72 and 72 by default) say where that is on the 0-144 in map.

```cpp
mclib::device::Gps gps(5, 0.0, -4.0);  // port, mounted 4 in behind centre
fusion.setGps({.read = [] { return gps.pose(); },
               .error_in = [] { return gps.getErrorM() * 39.37; }});
```

In the physics simulator (GPS noise 0.47 in, 5% of reads stale), with drive
encoders only, 9 of 12 GPS moves pass:

| Drive encoders only | Odometry alone | Distance sensors | GPS |
| --- | --- | --- | --- |
| four_motor_200 point diagonal | 1.10 in | 1.13 in | 0.40 in, passes |
| four_motor_200 boomerang | 3.41 in | 2.55 in | 0.47 in, passes |
| four_motor_200 ramsete arc | 4.06 in | 2.36 in | 0.78 in, passes |
| six_motor_450 point diagonal | 1.44 in | 0.37 in | 0.16 in, passes |
| six_motor_450 boomerang | 5.90 in | 0.84 in | 2.19 in |
| six_motor_450 ramsete arc | 5.30 in | 3.71 in | 1.44 in, passes |
| speed_base point diagonal | 1.15 in | 1.19 in | 0.94 in, passes |
| speed_base boomerang | 9.40 in | 2.28 in | 3.75 in |
| speed_base ramsete arc | 5.64 in | 4.23 in | 1.06 in, passes |

With two tracking wheels 11 of 12 pass; the four_motor_200 plain arc runs
out its 4 s limit with or without it.

#### Late readings

A reading is already old when the program reads it. At 60 in/s, a GPS pose
taken 50 ms ago is 3 in behind the robot, and blending it in pulls the pose
back. `GpsInput::latency_ms` and `DistanceSensorInput::latency_ms` say how
old a reading is. The filter then goes back to the pose it had when the
reading was taken, applies the reading there, and steps forward again
through the odometry since. It remembers `PoseFilterConfig::history_ms` of
steps, 250 ms by default. `setGps()` refuses a longer latency, and
`startOdometry()` refuses a fusion with a distance sensor that has one.

The default is 0: each reading is applied where the robot is when it is
read, as before. mclib has no measured latency for the V5 GPS or distance
sensor; measure yours before setting it.

In the physics simulator, with drive encoders only and the GPS readings held
back, odometry error at the end of a move (9 moves, mean / worst):

| GPS readings | `latency_ms` 0 | `latency_ms` set to the delay |
| --- | --- | --- |
| on time | 0.19 / 0.30 in | - |
| 50 ms late | 0.45 / 0.85 in | 0.14 / 0.20 in |
| 100 ms late | 1.20 / 2.41 in | 0.21 / 0.43 in |

With readings 100 ms late, the robot ends 3.39 in from the target on average
with `latency_ms` 0 and 1.98 in with it set; on-time readings give 1.50 in.

A `PoseFilter` you step yourself takes the time too: `predict(previous,
current, now_ms)` and `updateGps(measured, sigma, heading_sigma, taken_ms)`
or `updateDistance(geometry, measured_in, taken_ms)`. The `predict()`
without a time forgets the steps.

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

If no drive is bound, the first motion routine called prints an error, and
every motion routine returns `MotionResult::NoDrive` (`wallReset()` returns
false).

## Motion commands

`ChassisController` wraps each blocking routine as a command:
`makeTurnToAngleCommand`, `makeDriveToCommand`, `makeCurveCircleCommand`,
`makeCurveCircleReverseCommand`, `makeSwingCommand`,
`makeCorrectHeadingCommand`, `makeWallResetCommand`, `makeTurnToPointCommand`,
`makeMoveToPointCommand`, `makeBoomerangCommand` and
`makeFollowTrajectoryCommand`. Each runs the routine on its own task and
cancels it when interrupted.

`makeDriveDistanceCommand` and `makeTurnToHeadingCommand` run one step per
scheduler pass instead.
