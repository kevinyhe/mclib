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
| `path/trajectory.hpp` | `Trajectory`: speeds and times planned along a path before the robot moves |

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

### Trajectories

`Trajectory::generate()` plans the whole run before the robot moves: where it
should be at each moment, how fast, and how hard it should turn. A follower
that tracks a target pose over time needs this. Pure pursuit does not.

```cpp
TrajectoryConstraints limits;
limits.max_velocity = 48_in / 1_s;
limits.max_acceleration = 96_in / (1_s * 1_s);
limits.max_lateral_acceleration = 60_in / (1_s * 1_s);
limits.track_width = 11.375_in;

const Trajectory traj = Trajectory::generate(route, limits);
const TrajectoryState target = traj.sample(0.5_s);
// target.x, target.y, target.heading, target.velocity, target.curvature
```

The path is resampled every `spacing` (default 0.5 in). Each sample's speed is
the smallest of:

| Limit | Formula |
| --- | --- |
| `max_velocity` | fixed |
| Cornering | `sqrt(max_lateral_acceleration / k)`, using the tightest curvature within half a sample |
| Outer wheel | `max_velocity / (1 + k * track_width / 2)` |
| Speeding up | `max_acceleration` from the previous sample |
| Slowing down | `max_deceleration` (0 = same as `max_acceleration`) to the next sample |

`start_velocity` and `end_velocity` set the speed at each end, for chaining.
`reversed = true` drives the path with the back of the robot leading: speeds
are negative and the body heading is the path heading + 180°.

A 48 in straight line at 48 in/s and 96 in/s² takes 1.5 s: 0.5 s speeding up,
0.5 s at full speed and 0.5 s slowing down.

Use a spline. A polyline from `Path::fromWaypoints()` has zero curvature and a
heading jump at every corner.

### Following a trajectory (RAMSETE)

`followTrajectory()` drives a tank drive along a trajectory. Each 10 ms it
compares the odometry pose with where the trajectory says the robot should be,
and corrects speed and turn rate toward it. `driveTo()` and `curveCircle()`
measure progress with the drive encoders, which keep counting when a wheel
slips. RAMSETE uses the pose, so slip shows up as error and gets corrected.

```cpp
control::RamseteConfig follow;
follow.feedforward = {0.8_V, 0.18_V / inps, 0.02_V / inps2};  // per side, measured
// follow.gains = {2.0, 0.7};  // b and zeta, SI units; these are the defaults

const Trajectory traj = Trajectory::generate(route, limits);
if (followTrajectory(traj, follow, traj.duration() + 500_ms) !=
    control::MotionResult::Reached) {
  return;
}
```

- Plan the trajectory from where the robot is. It is not shifted to the
  current pose.
- The feedforward is per side of the drive: volts for a wheel speed. Measure it
  as in [Measuring kS and kV](#measuring-ks-and-kv). Without it, nothing drives
  the robot, so a `kV` of 0 returns `InvalidValue`.
- It returns when the trajectory ends. RAMSETE stops correcting once the
  planned speed reaches zero, so it does not wait to settle. Compare
  `robotState().pose()` with `traj.states().back()` if the end position
  matters.
- `Ramsete`, `tankWheelSpeeds()` and `ramseteVoltages()` in
  `control/ramsete.hpp` are the same steps without the loop, for use in a
  command.

- Set `follow.track_width` to the drive's effective track width. A skid-steer
  drive's wheels scrub sideways in a turn, so it turns slower than its real
  track width predicts. Spin in place with `V` volts on each side (opposite
  signs), read the turn rate `w` in rad/s, and use `2 * (V - kS) / kV / w`.
  In the physics simulator this came out 10-65% wider than the real track.
- The commanded speed never goes above the trajectory's top speed, even while
  catching up. Without that cap, a 450 rpm drive in the simulator took a
  24 in arc at 46 in/s, 8 in/s over plan, and slid out.

`b` sets how hard it pulls back toward the path; `zeta` sets damping. In the
host test, on an S-bend with the left side slipping 8%:

| Follower | Miss at the end |
| --- | --- |
| Feedforward only | 17.300 in |
| RAMSETE, b = 2 | 4.110 in |
| RAMSETE, b = 10 | 1.105 in |

A steady slip is a constant push, and RAMSETE's correction is proportional, so
it shrinks the error instead of removing it.

In the physics simulator, with two tracking wheels, on the same 90° arc of
24 in radius that `curveCircle()` drives (feedforward and effective track
width measured in the simulator, planned at 60 in/s² and 60 in/s²
cornering):

| Drive | `curveCircle()` | RAMSETE b = 2 | RAMSETE b = 50 |
| --- | --- | --- | --- |
| four_motor_200, forward / reverse | 7.31 / 5.93 in | 6.01 / 4.84 in | 0.42 / 0.37 in |
| six_motor_450, forward / reverse | 9.54 / 9.53 in | 8.28 / 7.17 in | 1.98 / 1.95 in |
| speed_base, forward / reverse | 12.51 / 15.10 in | 11.46 / 10.67 in | 1.03 / 0.86 in |

RAMSETE stops correcting when the plan does, so a robot that lags ends with
the error it had: up to 12° off heading on the fast drives. With `exit` true,
`followTrajectory()` then settles in the time left:

1. It turns in place to the final heading
   (`RamseteConfig::turn_to_final_heading`).
2. If the miss along that heading is more than
   `distance_exit.big_error` (1.5 in by default), it drives straight to close
   it (`RamseteConfig::settle_position`).

Both are on by default. It does not settle a sideways miss: a tank drive can
only close a few inches sideways by pivoting, and settling with `boomerang()`
that way ran out of time in the simulator, up to 179° off heading.

With both on, all nine arc and S-curve scenarios pass with two tracking
wheels: 0.38-1.60 in and 0.1-2.0° off. Settling takes up to 1.2 s of the time
limit.

Two stress cases still fail, because what is left is mostly sideways:

| Case (six_motor_450, 90° arc) | Before settling | After |
| --- | --- | --- |
| Low grip (friction 0.65), feedforward measured at full grip | 12.09 in | 9.59 in |
| Starting 3.6 in and 8° off the path | 4.53 in | 3.67 in |

In both the robot slides outward through the arc. Lowering b to 10-30 changes
the miss by at most 1.4 in, and capping the turn rate made every case worse.
Plan slower on a slippery field.

The default b = 2 is the usual value for full-size robots, and it is too soft
at VEX scale. Start near 50 and lower it if the robot weaves about the path.
With drive encoders only (no tracking wheels), RAMSETE at b = 50 missed the
same arcs by 4.07-4.42 in, against 6.65-14.56 in for `curveCircle()`. It
can't do better than the pose it corrects toward, and that pose came from the
same slipping wheels: odometry was 4.3-4.9 in off at the end.

`generate()` returns an empty trajectory when the path is not `valid()` or a
required limit is zero, negative or not finite.

### Logging

`PurePursuit::attachLogger(logger)` logs goal x, goal y, cross-track error,
curvature (1/in) and commanded velocity on every `update()`.

## Holonomic drives

X-drive and mecanum support, in three parts:

| Header | Contents |
| --- | --- |
| `chassis/holonomic_math.hpp` | `holonomic::mix(forward, strafe, turn, kind)` converts -1..1 commands to four wheel outputs, scaled down together if any exceeds 1. `holonomic::fieldToRobot(field_x, field_y, heading_rad)` converts a field-relative command. |
| `chassis/holonomic_chassis.hpp` | `HolonomicChassis`: four motor groups and an optional IMU |
| `chassis/holonomic_controller.hpp` | `HolonomicController`: the subsystem, with driver control, `moveToPose()` and `followTrajectory()` |
| `control/holonomic_follower.hpp` | `holonomicFollowStep()`: one tick of trajectory following, without the subsystem |

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

### Following a trajectory

`followTrajectory(trajectory, final_heading, timeout)` drives along a
[trajectory](#trajectories) while turning from the current heading to
`final_heading`, evenly over the trajectory's duration. The direction of
travel and the heading are separate: the robot can strafe along a curve while
facing one way.

Each tick it:

1. samples the trajectory at the time since the start;
2. takes the planned speed along the path, and adds `translation_kp` times the
   x and y position error (in/s per inch);
3. adds `heading_kp` times the heading error to the planned turn rate;
4. rotates the field velocity into the robot frame with the odometry heading;
5. converts forward, strafe and turn speeds to volts with
   `config.follower.feedforward`, and mixes them onto the wheels.

When the trajectory ends it switches to `moveToPose()` on the last point and
`final_heading`, on the same timeout clock, and settles with the usual exit
conditions.

```cpp
config.follower.feedforward.forward = {0.6_V, 0.30_V / inps, {}};
config.follower.feedforward.strafe = {0.9_V, 0.42_V / inps, {}};  // mecanum slips sideways
config.follower.feedforward.turn = {0.8_V, 2.0_V / radps};
controller.setConfig(config);

TrajectoryConstraints limits;
limits.max_velocity = 30_in / 1_s;
limits.max_acceleration = 60_in / (1_s * 1_s);
const Trajectory traj = Trajectory::generate(route, limits);  // reversed = false
controller.makeFollowTrajectoryCommand(traj, 90_deg, 5_s)->schedule();
```

Measure each feedforward axis on its own, as in
[Measuring kS and kV](#measuring-ks-and-kv): drive forward for `forward`,
strafe for `strafe`, spin in place for `turn`. `forward.kV` must be positive,
or the goal ends at once with the drive held.

Plan `max_velocity` within what the drive can do while strafing and turning.
In the host test, a mecanum drive planned at 24 in/s needs 10.7 V to strafe
before any turn is added. The wheel mix scales everything down, and tracking
error reaches 4.32 in. Planned at 16 in/s it stays within 0.55 in.

