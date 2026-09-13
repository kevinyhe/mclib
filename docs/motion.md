# Motion

## Motion profiles

`mclib/control/profile.hpp`. A motion profile plans speed over time for a
move: speed up at a set acceleration, cruise, then slow down so the robot stops
on the target.

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

This profile accelerates for 0.5 s over 12 in, cruises for 0.5 s over 24 in and
decelerates for 0.5 s over 12 in: 1.5 s total.

| `ProfileConstraints` field | 0 means |
| --- | --- |
| `max_velocity` | empty profile |
| `max_acceleration` | empty profile |
| `max_deceleration` | same as `max_acceleration` |
| `max_jerk` | no jerk limit (trapezoid) |

Setting `max_jerk` ramps acceleration in and out (an S-curve). With a
384 in/s³ limit the same move takes 1.75 s.

| Case | Result |
| --- | --- |
| Too short to reach cruise speed | accelerates then decelerates (a triangle); 6 in peaks at 24 in/s after 0.25 s |
| Different accel and decel | 48 in at 96 in/s² accel, 48 in/s² decel: 0.50 s / 0.25 s / 1.00 s |
| Zero distance | empty profile |
| Negative distance | runs in reverse |
| Already moving | starts from the current velocity |
| Too fast to stop in time | decelerates at the limit and overshoots; `overshoots()` returns true and `netDisplacement()` reports the real distance |

`DirectionalConstraints` holds separate forward and reverse limits.
`select()` picks one using the distance, or the initial velocity when the
distance is zero.

`velocityAtDistance()` and `timeAtDistance()` look up the profile by distance
instead of time, for path following.

## Feedforward

`mclib/control/feedforward.hpp`. Feedforward predicts the voltage a speed needs,
so the PID only corrects the error:

```
V = kS * sgn(v) + kV * v + kA * a
```

| Gain | Meaning | Unit |
| --- | --- | --- |
| `kS` | voltage to overcome friction | V |
| `kV` | voltage per unit of speed | V per in/s |
| `kA` | voltage per unit of acceleration | V per in/s² |

```cpp
SimpleMotorFeedforward model({.kS = 0.8_V,
                              .kV = 0.2_V / mclib::units::inps,
                              .kA = 0.02_V / inps2});
model.calculate(30 * mclib::units::inps);              // 6.8 V
model.calculate(30 * mclib::units::inps, 100 * inps2); // 8.8 V
model.maxAchievableVelocity(12_V, {});                 // 56 in/s
```

`kS` uses the sign of the velocity, or of the acceleration when velocity is
zero. When both are zero, the output is 0 V.

### Measuring kS and kV

1. Apply a fixed voltage to both sides of the drive, starting at 2 V.
2. Wait about 500 ms for the speed to settle.
3. Record the voltage and measured speed as a `VelocitySample`. Measure speed
   with odometry or `DriveGeometry::encoderToDistance()`, not the motor's
   `get_actual_velocity()`.
4. Repeat in 1 V steps up to 12 V.
5. Repeat in reverse.

```cpp
const VelocityFit fit = fitVelocityGains(samples, count);
// fit.kS, fit.kV, fit.r_squared, fit.used
```

Samples slower than `min_speed` (default 1 in/s) are ignored. An `r_squared`
below about 0.98 means bad data: the speed had not settled, the battery sagged,
or the drivetrain is binding.

### Measuring kA

1. Ramp the voltage from 0 to 12 V over about 2 s, logging voltage, velocity and
   acceleration each tick.
2. Compute acceleration from velocity with a three-point central difference.
3. Call `fitAccelerationGain(samples, count, {.kS = fit.kS, .kV = fit.kV})`.

`kA` matters least. Leave it at 0 if the fit is unreliable.

### Following a profile

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

`ProfileFollower` combines the profile, feedforward and a P loop on position.
Its PID uses the real time step, does not stop early on arrival, and limits the
integral to 2 V. With good feedforward, `kp` stays small.

`calculate(setpoint, measured, dt)` computes one step from a given setpoint and
time step.

`attachTelemetry(&logger)` logs setpoint position and velocity, measured
position, error, and the feedforward and feedback parts of the output. If
feedback is large compared to feedforward, re-measure the gains.

## Paths and pure pursuit

`mclib/path/`. Pure pursuit follows a path by steering toward a point a fixed
distance ahead on it (the lookahead), so the robot drives through waypoints
without stopping.

| Header | Contents |
| --- | --- |
| `path/path.hpp` | `Waypoint`, `PathPoint`, `Path`: samples with position, heading, curvature and distance along the path |
| `path/spline.hpp` | `generateSpline()`: a smooth curve through every waypoint |
| `path/pure_pursuit.hpp` | `PurePursuit`, `curvatureSpeedLimit()`, `approachSpeedLimit()`, `wheelSpeeds()` |

```cpp
#include "mclib/mclib.hpp"

using namespace mclib;
using namespace mclib::path;

Path route = generateSpline({
    Waypoint{0_in, 0_in},
    Waypoint{24_in, 24_in},
    Waypoint{48_in, 12_in},
});

PurePursuitConfig config;
config.lookahead = 12_in;
config.track_width = 11.375_in;
config.max_velocity = 48_in / 1_s;

PurePursuit follower(route, config);
while (true) {
  const PurePursuitOutput out = follower.update(control::robotState().pose());
  if (out.finished) {
    break;
  }
  drive(out.wheels.left, out.wheels.right);
  pros::delay(10);
}
```

Query a path with `atDistance()`, `atParameter()`, `length()` and
`maxAbsCurvature()`.

### Signs

Heading 0 is +Y and positive is clockwise. Positive curvature turns right.
`cross_track_error` is positive when the robot is right of the path.

### Splines

`generateSpline()` builds a centripetal Catmull-Rom spline. The curve passes
through every waypoint, and position and heading are continuous across them.
Curvature can change suddenly at a waypoint.

### Follower behavior

| Situation | Behavior |
| --- | --- |
| Lookahead circle crosses the path more than once | follows the first crossing ahead of its last position |
| No crossing found ahead | searches again from the closest point |
| Robot farther from the path than the lookahead | aims at the path one lookahead past the closest point and sets `off_path`; `finished` stays false |
| Target behind the robot | turns as tightly as allowed toward it |
| Near the end | aims at the last point and sets `at_end`; curvature is limited to `max_curvature` (default 6 in radius) |
| Within `finish_tolerance` of the end | sets `finished` and outputs zero |
| Path doubles back near itself | the closest-point search only looks `search_window` (default 24 in) ahead |

### Speed

The commanded speed is the smallest of:

| Limit | Formula |
| --- | --- |
| `max_velocity` | fixed |
| Cornering | `sqrt(max_lateral_accel / k)` for the tightest curvature within one lookahead |
| Stopping | `sqrt(2 * max_decel * remaining)` |

`max_lateral_accel` or `max_decel` at 0 or below disables that limit.
`min_velocity` sets a minimum. Both wheels scale down together if either would
exceed `max_velocity`.

With 60 in/s² lateral acceleration: a 48 in radius runs at 42.7 in/s, 10 in at
24.5 in/s, 8 in at 21.9 in/s and 2 in at 11.0 in/s.

To use a motion profile instead, ignore `PurePursuitOutput::velocity` and pass
the profile speed and the reported curvature to `wheelSpeeds()`.

### Logging

`PurePursuit::attachLogger(logger)` logs goal x, goal y, cross-track error,
curvature (1/in) and commanded velocity on every `update()`.

## Holonomic drives

X-drive and mecanum support, in three parts:

| Header | Contents |
| --- | --- |
| `chassis/holonomic_math.hpp` | `holonomic::mix(forward, strafe, turn, kind)` converts -1..1 commands to four wheel outputs, scaled down together if any exceeds 1. `holonomic::fieldToRobot(field_x, field_y, heading_rad)` converts a field-relative command. |
| `chassis/holonomic_chassis.hpp` | `HolonomicChassis`: four motor groups and an optional IMU |
| `chassis/holonomic_controller.hpp` | `HolonomicController`: the subsystem, with driver control and `moveToPose()` |

Strafe is positive to the right; turn is positive clockwise. Motor order is
front-left, front-right, back-left, back-right, viewed from above with the front
at the top. Use negative ports for reversed motors. Check wiring with
`drive(1, 0, 0)`: the robot should drive straight forward.

`moveToPose()` drives toward the target with one PID on distance and one on
heading, using the odometry pose. It finishes when both arrive or on timeout.
By default it holds at the end; with `stop_at_end = false` it sets 0 V so a
following move starts with the robot still moving. Requires
`control::startOdometry()`.

```cpp
#include "mclib/chassis/holonomic_chassis.hpp"
#include "mclib/chassis/holonomic_controller.hpp"

auto imu = std::make_shared<mclib::device::Inertial>(10);
mclib::HolonomicChassis drive({1}, {-2}, {3}, {-4},
                              mclib::device::Gearset::Green,
                              mclib::holonomic::Kind::Mecanum,
                              imu);

mclib::HolonomicControllerConfig config;
config.translation_pid = {0.4, 0.0, 3.0};  // volts per inch
config.heading_pid = {0.3, 0.0, 1.5};      // volts per degree
config.field_centric_teleop = true;        // left stick is a field direction
mclib::HolonomicController controller(drive, config);

void initialize() {
  drive.setPose({0.0, 0.0, 0.0});
  mclib::control::startOdometry();
  mclib::control::DriveCurveConfig curve;
  curve.gain = 10.0;
  controller.setDefaultCommand(controller.makeDriveCommand(master, curve));
  controller.registerSelf();
}

// x, y in inches; theta in radians, 0 = +Y, clockwise positive.
std::unique_ptr<Command> to_corner =
    controller.makeMoveToPoseCommand({24.0, 24.0, mclib::kPi / 2.0}, 3.0 * mclib::units::second);
std::unique_ptr<Command> back_home =
    controller.makeMoveToPointCommand(0.0 * mclib::units::inch, 0.0 * mclib::units::inch,
                                      3.0 * mclib::units::second);

void autonomous() {
  to_corner->schedule();
  while (to_corner->scheduled()) {
    CommandScheduler::run();
    pros::delay(10);
  }
  back_home->schedule();
}
```

`makeDriveCommand()` reads left Y (forward), left X (strafe) and right X (turn),
each shaped by its own `DriveCurve`. With `field_centric_teleop = true`, pushing
the left stick forward moves the robot toward field +Y regardless of its
heading. `drive.driveFieldCentric(x, y, turn)` and
`drive.drive(forward, strafe, turn)` are the underlying calls.
