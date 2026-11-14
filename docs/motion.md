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

### Tuning from a log

`mclib/control/characterize.hpp` has robot routines that do the measuring
above and log it, and `tools/tune_drive.py` fits the numbers on a computer.
`mclib/mclib.hpp` does not include this header.

```cpp
#include "mclib/control/characterize.hpp"

static mclib::telemetry::RowBuffer<512> buffer;
mclib::telemetry::SdCardSink sink("tune");
mclib::telemetry::Logger log(sink, buffer, {.period = 10_ms});
mclib::control::TuningLog columns(log);
mclib::telemetry::FlushTask flusher(log);
mclib::control::characterizeDrive(columns);  // 3, 6, 9 V, forward and back
mclib::control::characterizeSpin(columns);   // 4, 6, 8 V, both ways
mclib::control::traceMotion(columns, [&] { return followTrajectory(path, follow, 8_s); });
flusher.stop();
```

```
python3 tools/tune_drive.py tune000.csv
```

It prints kS, kV and kA from the straight runs, the effective track width
from the spins, and a `RamseteConfig` snippet to paste. Speed comes from the
odometry position, so wheel slip is in the numbers. A fit error above about
0.2 V RMS means slip or a dragging side.

For a traced motion it splits the gap between the target and the robot into
"behind" and "to the side", and says what to change:

| What the log shows | Change |
| --- | --- |
| More than 1 in behind on average | Feedforward is too weak: refit kS, kV, kA |
| More than 1 in ahead on average | Feedforward is too strong |
| Crossing the path more than once a second | Lower b, or raise zeta toward 0.9 |
| More than 1 in to the side, not crossing | Raise b (try 1.5x) |

In the physics simulator (six_motor_450, two tracking wheels) the routines
and script give kS 0.696 V, kV 0.1330 V per in/s and a 13.51 in track, against
0.699, 0.1315 and 13.45 measured from the simulated robot's true motion.

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
  driveChassis(command, command);
  pros::delay(10);
}
```

`travelledDistance()` is your own distance reading, from odometry or
`DriveGeometry::encoderToDistance()`.

`ProfileFollower` combines the profile, feedforward and a P loop on position.
Its PID uses the real time step, does not stop early on arrival, and limits the
integral to 2 V. With good feedforward, `kp` stays small.

`calculate(setpoint, measured, dt)` computes one step from a given setpoint and
time step.

`attachTelemetry(&logger)` logs setpoint position and velocity, measured
position, error, and the feedforward and feedback parts of the output. If
feedback is large compared to feedforward, re-measure the gains.

## Profiled moves

`mclib/control/profiled_moves.hpp`. By default `driveTo()`, `turnToAngle()`,
`swing()` and `moveToPoint()` set their voltage from a PID on the distance or
angle left, with a slew limit. The voltage shrinks with the error, so the
robot slows for the whole last part of every move. After
`useProfiledMoves()`, each of them with `exit` true plans its speed instead:
speed up at `max_acceleration`, cruise, slow down to stop on the target. The
voltage is the feedforward for the planned speed and acceleration, plus `kp`
and `kd` on how far the wheels are behind the plan. Once the plan ends, the
move's own PID settles as before, with the `settle_min_voltage` floor.

```cpp
#include "mclib/control/profiled_moves.hpp"

control::ProfiledMoveConfig moves;
moves.follower.gains = {0.48_V, 0.131_V / units::inps, 0.0176_V / control::inps2};
moves.follower.kp = 2_V / 1_in;
moves.follower.kd = 0.1_V / 1_in;   // volts per in/s
moves.turn_gains = {1.28_V, 0.112_V / units::inps, 0.0149_V / control::inps2};
moves.limits.max_velocity = 70 * units::inps;          // 80% of top speed
moves.limits.max_acceleration = 150 * control::inps2;  // 70% of what the wheels grip
moves.track_width = 14_in;                             // effective, measured
control::useProfiledMoves(moves);  // false, with the reason printed, if a value is bad

driveTo(24_in, 2_s);  // now on a planned speed
```

`usePidMoves()` goes back. Chained moves (`exit` false), `curveCircle()`,
`turnToPoint()`, `boomerang()` and `followTrajectory()` run as before.

What each move does:

- `driveTo()` plans the distance. A move that starts while the robot still
  moves from a chained move plans from the speed its last voltage drives at.
- `turnToAngle()` plans the arc each wheel rolls: the turn times half of
  `track_width`, with `turn_gains`.
- `swing()` plans the arc the moving side rolls: the turn times
  `track_width`, with `turn_gains`.
- `moveToPoint()` sets its speed each tick from the distance left along its
  heading: no faster than it can stop from at `max_deceleration`, and no
  more than `max_acceleration` faster than the tick before. It turns in place
  until it faces within 30° of the target, not 60°: steering more than that
  at speed slid the robot sideways, and odometry without a sideways tracking
  wheel can't see a slide. Inside `distance_exit.small_error` its distance
  PID settles.

The plan skips the slew limit: it already limits the acceleration.

### Measuring the numbers

- `follower.gains`: kS and kV as in [Measuring kS and kV](#measuring-ks-and-kv).
  Measure kA from a low-voltage start. From a high voltage the drive speeds
  up at its grip or current limit, not at what the voltage asks for, and kA
  comes out too high. In the simulator a 4 V start gave 0.0176 V per in/s²
  on six_motor_450 and an 8 V start 0.0235. With the high value the robot
  ran ahead of the plan speeding up and stopped short of it slowing down.
- `turn_gains`: the same fit from spins in place at two voltages, with the
  wheel speed taken as the turn rate in rad/s times half of `track_width`.
  Turning scrubs the wheels sideways, so its kS is higher: 1.28 V against
  0.48 V driving straight on six_motor_450. With the straight-line gains,
  turns stopped 4° short and crept the rest.
- `max_acceleration`: about 70% of the fastest the drive speeds up at
  12 V. six_motor_450 speeds up at 210 in/s² at most.

### In the simulator

Every move of the stopping scenarios, on each drive, with the moves'
default PID gains against `useProfiledMoves()` tuned as in
`tests/vexsim/run.py` (kS, kV, kA and the turning gains measured in the
simulator, 80% of top speed, 150 in/s², `kp` 2 V per in, `kd` 0.1 V per
in/s). Two tracking wheels; the Override cascade drive has its one sideways
wheel. Time, then miss.

| Drive | Move | PID | Profiled |
| --- | --- | --- | --- |
| four_motor_200 | `driveTo(24_in)` | 2.61 s, 0.29 in | 1.03 s, 0.29 in |
| four_motor_200 | `turnToAngle(90_deg)` | 1.07 s, 2.18° | 0.68 s, 0.44° |
| four_motor_200 | `moveToPoint(24, 24)` | 3.07 s, 0.41 in | 1.41 s, 0.16 in |
| four_motor_200 | `swing(90_deg)` | 1.62 s, 2.38° | 1.03 s, 0.49° |
| six_motor_450 | `driveTo(24_in)` | 1.23 s, 0.07 in | 0.86 s, 0.01 in |
| six_motor_450 | `turnToAngle(90_deg)` | 0.61 s, 1.26° | 0.61 s, 0.09° |
| six_motor_450 | `moveToPoint(24, 24)` | 1.52 s, 0.03 in | 1.06 s, 0.30 in |
| six_motor_450 | `swing(90_deg)` | 0.65 s, 0.45° | 0.85 s, 0.38° |
| speed_base | `driveTo(24_in)` | 0.96 s, 0.43 in | 0.86 s, 0.14 in |
| speed_base | `turnToAngle(90_deg)` | 0.85 s, 2.15° | 0.67 s, 0.12° |
| speed_base | `moveToPoint(24, 24)` | 1.22 s, 0.50 in | 1.10 s, 0.18 in |
| speed_base | `swing(90_deg)` | 1.52 s, 2.27° | 1.11 s, 0.81° |
| override_cascade | `driveTo(24_in)` | 1.47 s, 0.80 in | 0.87 s, 0.10 in |
| override_cascade | `turnToAngle(90_deg)` | 0.65 s, 1.27° | 0.63 s, 0.34° |
| override_cascade | `moveToPoint(24, 24)` | 1.77 s, 1.64 in | 1.09 s, 0.41 in |
| override_cascade | `swing(90_deg)` | 0.78 s, 0.88° | 0.96 s, 0.54° |

Over the seven stopping moves on every drive, the profiled moves took a third
less time in each odometry setup: 18.4 s against 29.1 s with two tracking
wheels, 24.4 s against 36.6 s with drive encoders only. They are slower
where the PID move already runs flat out: a swing on six_motor_450 and the
cascade drive is held to 150 in/s². On six_motor_450 a point move stops
0.3-0.4 in off, against 0.02-0.03 in for the PID.

`tests/vexsim/chain_audit.py` ends chains of moves with `exit` false in a
stopping `moveToPoint()`, 27 stops in all. With profiled moves 1 stop timed
out and the stops took 32.5 s; with the PID moves 6 timed out and they took
41.1 s. The planned approach hands over to the PID for good once the target
is within `distance_exit.small_error` ahead along the robot's heading. A stop
that starts with the target to the side or behind is already past that, so
it runs as the PID move.

The profiled moves stay opt-in: they need measured gains, and the swings
above are slower.

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

void followRoute(const control::SimpleMotorFeedforward& model) {
  const Path route = generateSpline({
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
    driveChassis(model.calculate(out.wheels.left), model.calculate(out.wheels.right));
    pros::delay(10);
  }
}
```

`model` is the feedforward from [Feedforward](#feedforward).

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
| Speed zone | the zone's `max_velocity`, inside a `SpeedZone` |
| Speeding up | `max_acceleration` from the previous sample |
| Slowing down | `max_deceleration` (0 = same as `max_acceleration`) to the next sample |

`start_velocity` and `end_velocity` set the speed at each end, for chaining.
`reversed = true` drives the path with the back of the robot leading: speeds
are negative and the body heading is the path heading + 180°.

`speed_zones` slows one stretch of the path. Each zone has a start and end,
measured along the path from its first point, and a top speed. The plan brakes
before the zone, within `max_deceleration`, and speeds back up after it, so
there is no need to split the path:

```cpp
limits.speed_zones = {
    SpeedZone{20_in, 36_in, 15_in / 1_s},  // 15 in/s from 20 in to 36 in
};
```

Zones may overlap; the lowest speed wins. A zone may run past the end of the
path. The slow stretch can begin and end up to one `spacing` (0.5 in) outside
the zone. On the 48 in line below, a 12 in/s zone from 18 in to 30 in raises
the time from 1.5 s to 2.55 s. A zone that starts below 0, ends at or before
its start, or has a speed of 0 or less gives an empty trajectory.

A 48 in straight line at 48 in/s and 96 in/s² takes 1.5 s: 0.5 s speeding up,
0.5 s at full speed and 0.5 s slowing down.

Use a spline. A polyline from `Path::fromWaypoints()` has zero curvature and a
heading jump at every corner.

### Following a trajectory (RAMSETE)

`followTrajectory()` drives a tank drive along a trajectory. Each 10 ms it
compares the odometry pose with where the trajectory says the robot should be,
and corrects speed and turn rate toward it, along a planned speed.
`curveCircle()` also measures progress on the pose, but does not plan a
speed: a distance PID sets the voltage, and it slows down to keep sideways
acceleration under `MotionConfig::arc_max_lateral_accel`.

```cpp
control::RamseteConfig follow;
follow.feedforward = {0.8_V, 0.18_V / units::inps, 0.02_V / control::inps2};  // per side, measured
// follow.gains = {2.0, 0.7};  // b and zeta, SI units; these are the defaults

const Trajectory traj = Trajectory::generate(route, limits);
if (followTrajectory(traj, follow, traj.duration() + 2_s) !=
    control::MotionResult::Reached) {
  return;
}
```

- Plan the trajectory from where the robot is. It is not shifted to the
  current pose.
- The feedforward is per side of the drive: volts for a wheel speed. Measure it
  as in [Measuring kS and kV](#measuring-ks-and-kv). Without it, nothing drives
  the robot, so a `kV` of 0 returns `InvalidValue`.
- RAMSETE stops correcting once the planned speed reaches zero. With `exit`
  false it returns then, without settling. Compare `robotState().pose()`
  with `traj.states().back()` if the end position matters. With `exit` true
  it settles first, as described below.
- `Ramsete`, `tankWheelSpeeds()` and `ramseteVoltages()` in
  `control/ramsete.hpp` are the same steps without the loop, for use in a
  command.
- With `exit` false, `early_exit_range` ends it once that much of the path is
  left, so the next motion takes over at speed. The distance left is the
  planned distance left, minus how far the robot is ahead of the planned
  point.
  `robotState().motionTelemetry().remaining_in` shows it while it runs.

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
| four_motor_200, forward / reverse | 1.77 / 1.62 in | 6.01 / 4.84 in | 0.42 / 0.37 in |
| six_motor_450, forward / reverse | 2.11 / 2.02 in | 8.28 / 7.17 in | 1.98 / 1.95 in |
| speed_base, forward / reverse | 2.46 / 2.53 in | 11.46 / 10.67 in | 1.03 / 0.86 in |

RAMSETE stops correcting when the plan does, so a robot that lags or slides
ends with the error it had: up to 12° off heading on the fast drives, and up
to 12 in off on low grip. With `exit` true, `followTrajectory()` then settles
in the time left:

1. It turns in place to the final heading
   (`RamseteConfig::turn_to_final_heading`).
2. If the robot is more than `settle_side_tolerance` (3 in) to the side of the
   end point, it backs off along its heading (4x the sideways miss, 6-18 in)
   and comes back in with `boomerang()` at lead 0.6. The approach then stays
   within about 15° of the final heading. Turning to face a point a few inches
   to the side instead ran out of time in the simulator, up to 179° off.
3. Otherwise, if the miss along the heading is more than
   `distance_exit.big_error` (1.5 in), it drives straight to close it.

Steps 2 and 3 are `RamseteConfig::settle_position`. Both settings are on by
default. Settling takes up to 1.2 s for a turn and straight correction, and
2-5 s for a back-off, so give `followTrajectory()` a time limit well above
the trajectory's duration.

In the simulator with two tracking wheels, all nine arc and S-curve
scenarios pass (0.38-1.60 in, 0.1-2.0° off), and so do the stress cases:

| Case (six_motor_450, 90° arc) | No settling | Turn + straight | With back-off |
| --- | --- | --- | --- |
| Low grip (friction 0.65), feedforward measured at full grip | 12.09 in | 9.59 in | 2.34 in, 6.33 s |
| Starting 3.6 in and 8° off the path | 4.53 in | 3.67 in | 1.06 in, 5.80 s |
| Battery at 8% | 2.44 in | 1.60 in | 1.60 in |

On a slippery field, planning slower is still the better fix: the back-off
costs seconds.

### Arcs with RAMSETE

`curveCircle()` and `curveCircleReverse()` measure progress on the pose: the
angle swept around the arc's centre, fixed at the start. They steer back
toward the circle when the robot drifts off it, and stop steering back over
the last quarter of the sweep so the arc ends on its heading. When a side
would pass the voltage cap, they slow the arc and keep the heading
correction whole. They slow down to keep sideways acceleration under
`arc_max_lateral_accel` (60 in/s^2), and finish a stopped arc with a turn if
it ends more than `turn_exit.big_error` (3°) off heading. A zero radius still
runs on the encoders. After one call to
`useRamseteForArcs()`, they plan the same arc with `path::planArc()` and follow
it with `followTrajectory()` instead, settle included. The calls, targets,
time limits and voltage caps in your autonomous stay the same.

```cpp
control::RamseteConfig follow;
follow.feedforward = {0.7_V, 0.13_V / units::inps, {}};  // measured, per side
follow.gains = {50.0, 0.7};
follow.track_width = 13.5_in;                     // effective, measured
TrajectoryConstraints arc_limits;
arc_limits.max_velocity = 60_in / 1_s;
arc_limits.max_acceleration = 60_in / (1_s * 1_s);
arc_limits.max_lateral_acceleration = 60_in / (1_s * 1_s);
control::useRamseteForArcs(follow, arc_limits);

curveCircle(90_deg, 24_in, 5_s);  // now followed on the odometry pose
```

The encoder arc still runs for a chained arc (`exit` false) and a zero
radius. `useEncoderArcs()` switches back.

A reverse arc needs the radius and the heading change to have opposite signs.
Backing round a circle turns the heading away from the side its centre is on.
`curveCircleReverse()` with the same sign on both returns `InvalidValue` and
does not move, and prints why. Half a circle is the exception. Its heading
change wraps to +180° or -180° with the side of its planned heading the robot
starts on, so a same-sign change of 160° or more is backed round the other
way, with `useRamseteForArcs()` or without.

In the physics simulator, on the 90° arcs of 24 in radius, with two tracking
wheels:

| Drive | `curveCircle()` (fwd / rev) | With `useRamseteForArcs()` |
| --- | --- | --- |
| four_motor_200 | 1.70 / 1.07 in | 0.41 / 0.37 in |
| six_motor_450 | 1.83 / 1.68 in | 1.59 / 1.55 in |
| speed_base | 2.27 / 3.35 in | 0.77 / 0.98 in |

four_motor_200 still runs out its 4 s limit: its arc distance PID slows from
20 in out and crawls the last few inches. With drive encoders only, plain arcs
end 4.21-7.97 in off and RAMSETE arcs 4.06-4.94 in,
limited by the odometry, which is 4-5 in off there. Add distance sensors and a
`PoseFusion` for those robots (see getting-started.md).

The default b = 2 is the usual value for full-size robots, and it is too soft
at VEX scale. Start near 50 and lower it if the robot weaves about the path.
With drive encoders only (no tracking wheels), RAMSETE at b = 50 missed the
same arcs by 4.07-4.42 in, against 4.21-7.97 in for `curveCircle()`. It
can't do better than the pose it corrects toward, and that pose came from the
same slipping wheels: odometry was 4.3-4.9 in off at the end.

`generate()` returns an empty trajectory when the path is not `valid()` or a
required limit is zero, negative or not finite.

### Driving to a pose (moveToPose)

`moveToPose()` arrives at a point facing a chosen way, as `boomerang()` does.
It plans a curve from the odometry pose with `path::planToPose()`, plans a
trajectory along it with your limits, and runs `followTrajectory()`, settle
included. `boomerang()` chases a moving carrot with two PIDs. This plans the
speed along the whole curve and corrects on the pose.

```cpp
moveToPose(24_in, 24_in, 1, 90_deg, 0.4, follow, limits, 5_s);
moveToPose(0_in, 0_in, -1, 0_deg, 0.4, follow, limits, 5_s);  // backward
```

The curve is a cubic Bezier. It leaves along the start heading and comes in
along the target heading, each control arm `lead` times the straight-line
distance. 0.39 makes a 90° corner nearly a circle; larger swings wider. A
curve that folds back on itself, where the robot would have to stop and
reverse, returns `InvalidValue` without moving: a target behind the start
heading, one to arrive at against the start heading, or a target straight
ahead with a lead of 1 or more. Drive those in two moves.

`limits.reversed` is set from `dir`. `Routine::moveToPose()` and
`ChassisController::makeMoveToPoseCommand()` plan the curve when the step
starts, from wherever the robot is.

In the physics simulator, to (24, 24) arriving at 90° from the origin, lead
0.4, with the RAMSETE scenarios' tuning in `tests/vexsim/run.py` (b = 50,
measured kS and kV, 70% of top speed, 60 in/s²). Time, then miss:

| Drive | Odometry | `boomerang()` lead 0.5 | `moveToPose()` |
| --- | --- | --- | --- |
| four_motor_200 | two tracking wheels | 3.61 s, 0.72 in | 2.14 s, 0.28 in |
| six_motor_450 | two tracking wheels | 3.32 s, 0.47 in | 2.50 s, 0.17 in |
| speed_base | two tracking wheels | 2.55 s, 0.41 in | 2.45 s, 0.33 in |
| four_motor_200 | sideways wheel, drive encoders | 3.56 s, 1.62 in | 2.14 s, 0.13 in |
| six_motor_450 | sideways wheel, drive encoders | 2.61 s, 1.24 in | 2.53 s, 0.15 in |
| speed_base | sideways wheel, drive encoders | 2.72 s, 2.97 in | 2.41 s, 0.60 in |
| override_cascade | sideways wheel, drive encoders | 1.93 s, 0.40 in | 2.88 s, 0.33 in |

It ended closer every time and was faster on all but the cascade drive,
where the plan's 60 in/s² holds it back: at 120 in/s² it took 1.75 s and
missed by 0.27 in. Backing to (24, -24) arriving at -90° came within
0.14 s and 0.08 in of the forward drive.

With drive encoders only, or one forward tracking wheel, both miss by 3 to
11 in: the odometry is that far off after a curve. With b = 2 (the library
default) and two tracking wheels it still arrived within 0.6 in, but took
5.3-7.5 s.

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

mclib::HolonomicControllerConfig config{
    .translation_pid = {0.4, 0.0, 3.0},  // volts per inch
    .heading_pid = {0.3, 0.0, 1.5},      // volts per degree
    .field_centric_teleop = true,        // left stick is a field direction
};
mclib::HolonomicController controller(drive, config);

// A holonomic chassis is not a differential drive, so startOdometry() with no
// arguments has no drive encoders to read and returns false. Give it two
// tracking wheels.
mclib::device::Rotation vertical_tracker(-6);
mclib::device::Rotation horizontal_tracker(7);
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 0.0_in, 1.0};
const mclib::units::TrackingWheel horizontal_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), -3.5_in, 1.0};

void initialize() {
  mclib::control::OdometrySetup odometry;
  odometry.heading_deg = [] { return drive.headingDeg(); };
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  odometry.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker), horizontal_wheel};
  mclib::control::startOdometry(odometry);
  drive.setPose({0.0, 0.0, 0.0});
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

// A scheduled command only moves the robot while something calls
// CommandScheduler::run().
void runToEnd(Command& command) {
  command.schedule();
  while (command.scheduled()) {
    CommandScheduler::run();
    pros::delay(10);
  }
}

void autonomous() {
  runToEnd(*to_corner);
  runToEnd(*back_home);
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
config.follower.feedforward.forward = {0.6_V, 0.30_V / mclib::units::inps, {}};
config.follower.feedforward.strafe = {0.9_V, 0.42_V / mclib::units::inps, {}};  // mecanum slips sideways
config.follower.feedforward.turn = {0.8_V, 2.0_V / mclib::units::radps};
controller.setConfig(config);

mclib::path::TrajectoryConstraints limits;
limits.max_velocity = 30_in / 1_s;
limits.max_acceleration = 60_in / (1_s * 1_s);
const mclib::path::Trajectory traj = mclib::path::Trajectory::generate(route, limits);  // reversed = false
// The scheduler does not own a command, so keep it alive while it runs.
static std::unique_ptr<Command> follow = controller.makeFollowTrajectoryCommand(traj, 90_deg, 5_s);
runToEnd(*follow);
```

Measure each feedforward axis on its own, as in
[Measuring kS and kV](#measuring-ks-and-kv): drive forward for `forward`,
strafe for `strafe`, spin in place for `turn`. `forward.kV` must be positive,
or the goal ends at once with the drive held.

Plan `max_velocity` within what the drive can do while strafing and turning.
In the host test, a mecanum drive planned at 24 in/s needs 10.7 V to strafe
before any turn is added. The wheel mix scales everything down, and tracking
error reaches 4.32 in. Planned at 16 in/s it stays within 0.55 in.

