# Motion: profiles, paths and holonomic drives

Motion profiling, drivetrain feedforward, pure pursuit, and X-drive/mecanum kinematics.

[Documentation index](README.md) · [Project README](../README.md)

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

## Paths and pure pursuit (`mclib/path/`)

Multi-segment autons used to mean chaining point-to-point moves, each one
stopping dead at its endpoint. `mclib/path/` replaces that with a path the
robot follows without stopping.

Three files, all PROS-free and host-tested:

- `path/path.hpp` - `Waypoint`, `PathPoint`, and `Path`: an ordered list of
  baked samples in field coordinates, each carrying position, tangent heading,
  signed curvature and arc length from the start. Query it by arc length
  (`atDistance`), by normalised parameter (`atParameter`), or ask for
  `length()` and `maxAbsCurvature()`.
- `path/spline.hpp` - `generateSpline()`, a centripetal Catmull-Rom.
- `path/pure_pursuit.hpp` - the follower, plus the free functions
  `curvatureSpeedLimit()`, `approachSpeedLimit()` and `wheelSpeeds()`.

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

### Frame and sign

Field frame, compass convention: heading 0 is +Y, clockwise-positive
(`math.hpp`). **Curvature is signed the way `mclib::arcRadius()` is: positive
curves to the robot's right.** `PurePursuitOutput::cross_track_error` is
positive when the robot is to the *right* of the path, so a correct follower
answers a positive cross-track error with a negative curvature. `tests/
path_test.cpp` asserts both directions explicitly, because a transposed frame
compiles and runs and merely drives into a wall.

The sign comes from the heading of the *segment* the projection landed on, not
from `Path::atDistance()`. A polyline's heading is only defined per segment, so
`atDistance()` ramps between vertex headings, which scales the reported error
by `cos(the ramp)` — and flips its sign outright on a corner sharper than 90
degrees, which is exactly the hairpin case.

The frame check that anchors the whole unit: a straight path from `(0, 0)` to
`(0, 10)`, robot at the origin at heading 0, lookahead 5 in. The goal point
comes back as exactly `(0, 5)` and the curvature as exactly `0`.

### Why Catmull-Rom

An auton is written as "drive through these field positions". Catmull-Rom
**interpolates** - the curve passes through every waypoint you type. A Bezier
approximates: its interior control points are not on the curve, so the author
places handles that mean nothing on a field diagram and the robot does not go
through the numbers they wrote down.

The parameterisation is **centripetal** (alpha = 0.5). Uniform Catmull-Rom
overshoots and can form a cusp or a loop when waypoint spacing is uneven, which
is the exact shape that makes a pure-pursuit follower spin.

The curve is **C1**: the two segments either side of a waypoint share a
tangent, so position and heading are continuous across it. It is not C2 -
curvature steps at a waypoint. `tests/path_test.cpp` samples 4000 points across
a four-waypoint spline and asserts the heading change across each interior knot
is no more than the local curvature times the arc length across it.

End conditions are the part that naive Catmull-Rom gets wrong. Reflecting a
phantom point through the first knot yields the one-sided slope: exact for a
straight line, badly wrong for anything curved. On a 48 in circular arc it made
the first segment's curvature swing through zero and overshoot to 2/R, and the
curvature velocity limiter believed it and halved the speed for the first two
inches of every path. The end tangents are the derivative of the quadratic
through the first (and last) three knots instead, which reproduces a circle to
0.4%: measured curvature 0.020793 /in against an exact 1/48 = 0.020833 /in.

### The cases that break naive followers

- **More than one intersection.** The lookahead circle can cut the path in
  several places. The follower takes the **first intersection at or after a
  monotone lookahead cursor**, walking segments forward from where it stopped
  last tick. "First ahead" and not "furthest along" is the point: on a hairpin
  the far branch is also inside the circle, and chasing it cuts the corner and
  abandons the rest of the path.
  The search runs twice. The first pass walks forward from the lookahead
  cursor. The second restarts at the closest point, and only runs when the
  first found nothing — without it, a robot shoved back two inches leaves the
  cursor ahead of its own lookahead circle and the follower reports the end of
  the path with 30 in still to drive. The retry never starts behind the closest
  point, so it cannot undo the doubling-back guarantee.
- **Off the path.** When the closest path point is further away than the
  lookahead, no intersection exists at all. The follower aims at the path point
  one lookahead *beyond the closest point* - a rejoin, not a lunge at the
  endpoint - and reports `off_path = true`. `finished` is vetoed while
  `off_path` is set, so being shoved past the end of the route does not count
  as arriving.
- **The goal ends up behind the robot.** This is the one that drives into a
  wall. `arcRadius()` returns `+infinity` for a goal straight ahead **and** for
  one straight behind — both are a zero lateral offset — so a reversed robot
  gets curvature 0 and full speed away from the path, and the forward-only
  cursor means it never recovers. Anything strictly behind the robot gets the
  tightest turn available instead, toward whichever side the goal is on.
  Exactly abeam is left alone: the arc through it is a well-defined semicircle,
  not a degenerate case.
- **End of path.** When the search runs off the end, the goal is the final path
  point and `at_end` is true. The effective lookahead then shrinks as the robot
  arrives, so commanded curvature is clamped to `max_curvature` (default a 6 in
  radius). Once the remaining arc length is inside `finish_tolerance`,
  `finished` is true and both wheel speeds are zero.
- **Doubling back.** Both search cursors move forward only, and the
  closest-point search is bounded to `search_window` (default 24 in) of arc
  length ahead. The test drives up an outbound leg whose return leg is 4 in
  away in field space; an unguarded nearest-point search latches onto the
  return leg, this one does not. The window is enforced on the parameter
  *inside* a segment, not only on whole samples — one segment of a
  `fromWaypoints()` polyline can be longer than the whole window, and without
  that a single bad pose skips the route for good.

### Speed

Three limits, smallest wins:

| Limit | Formula |
| --- | --- |
| Configured cap | `max_velocity` |
| Cornering | `sqrt(max_lateral_accel / k)` over the tightest curvature in the next lookahead of path |
| Stopping | `sqrt(2 * max_decel * remaining)` |

A non-positive `max_lateral_accel` or `max_decel` means "no limit", not "speed
zero" — switching the endpoint ramp off must not pin the robot at
`min_velocity` for the whole path.

`min_velocity` is a floor under the result while the path is unfollowed, and
the pair is scaled down together if `wheelSpeeds()` would put either wheel over
`max_velocity`. Measured, with 60 in/s^2 of lateral budget: a 48 in radius arc
runs at 42.7 in/s (the 48 in/s cap, scaled by the 1.125 outer-wheel spread), a
10 in radius arc at 24.5 in/s, an 8 in radius at 21.9, a 2 in radius at 11.0.

These are stateless kinematic one-liners on purpose. A trapezoidal or S-curve
profile layers on top by ignoring `PurePursuitOutput::velocity` and feeding the
profile's speed through `wheelSpeeds()` with the reported curvature.

### Logging

`PurePursuit::attachLogger(logger)` registers five channels - goal x, goal y,
cross-track error, curvature in 1/in, and commanded velocity - and every
`update()` writes them. A path follower you cannot plot is a path follower you
cannot tune.

### Do not use `getRadius()`

`utils.hpp`'s `getRadius()` is frame-transposed and returns 5.0 for a target
dead ahead, where the true radius is infinite. `mclib::arcRadius()` in
`math.hpp` is the correct primitive and is what this unit calls.

## Holonomic drive (X-drive and mecanum)

Three pieces, mirroring `Chassis` / `ChassisController` for a four-wheel holonomic base.

`include/mclib/chassis/holonomic_math.hpp` is the arithmetic, host-tested. `holonomic::mix(forward, strafe, turn, kind)` turns three -1..1 commands into four wheel fractions (`front_left`, `front_right`, `back_left`, `back_right`), scaled down together if any would exceed 1 so the direction of travel survives saturation. Strafe is positive to the right, turn positive clockwise. `holonomic::fieldToRobot(field_x, field_y, heading_rad)` rotates a field-frame command into the robot frame using the library's compass convention (0 = +Y, clockwise positive). Both layouts use the same four sums; the enum records which robot you have.

`HolonomicChassis` owns the four motor groups and an optional IMU. Wheel order is front_left, front_right, back_left, back_right looking down with forward at the top; reverse a motor with a negative port. Test `drive(1, 0, 0)` first: a wheel wired backward makes forward look like a strafe plus a turn.

`HolonomicController` is the subsystem. `moveToPose()` runs a distance PID along the bearing to the target and a heading PID on the wrapped heading error, both on the odometry pose from `control::robotState()`, and settles when both arrive or the timeout expires. Every exit writes the motors: hold at the end by default, zero volts when `stop_at_end` is false so a chained move keeps its momentum. It needs `control::startOdometry()` running, or the pose never moves.

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

`makeDriveCommand()` reads three stick axes (defaults: left Y forward, left X strafe, right X turn), shapes each through its own `DriveCurve`, and drives field-centric when the config says so: pushing the left stick up always moves the robot toward field +Y whichever way it faces. Set `field_centric_teleop = false` for robot-centric sticks. `drive.driveFieldCentric(x, y, turn)` and `drive.drive(forward, strafe, turn)` are the two primitives underneath.
