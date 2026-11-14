# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow
[semantic versioning](https://semver.org/).

## Unreleased

### Fixed

- A stopping `moveToPoint()` given a voltage floor (`min_speed`) held it
  until the target, so a robot that came in fast from a chained move could
  not brake and slid past. Inside 8 in the floor now also stops once
  braking at 150 in/s² from the robot's speed would only just stop it at the
  target. In `tests/vexsim/chain_audit.py`, which now passes each step its
  floor (the planner's change to `run()` had dropped it), 6 of 18 stops on
  the 450 rpm drives timed out with 4-8 V floors and 2 still do. Those 18
  stops took 25.8 s in all, down from 29.2 s, and none took longer. On
  `four_motor_200` two stops got faster and none slower. Calls without a
  floor are unchanged.

## 0.4.3

Update vexsim for the path planner: `git -C ../vexsim pull`. It now comes
with the Override field shapes.

### Added

- `wallReset()` takes a `control::WallDistanceCheck`: a distance sensor that
  has to read the wall where it should after the stall. Without it, anything
  that holds the robot still reads as the wall. With no reading, or a reading
  more than `tolerance` from `expected`, the call returns false and the pose
  and IMU stay as they were. Routine steps take it with
  `.withWallDistanceCheck(...)`.
- `characterizeGrip()` and `characterizeWallPush()` with a `GripLog`
  (`mclib/control/characterize.hpp`): spins, circles, launches and a push
  into a wall, open loop, logged with the sideways tracking wheel and the
  battery voltage. Between runs the robot drives back to where it started
  with `moveToPoint()` and `turnToAngle()`, so it needs odometry running.
  `examples/tire_grip.cpp` is a whole program.
- The vexsim bridge runs an `AsyncMotion` in turns with its caller, so
  `traceMotion()` and code that drives while logging run in the simulator.
- `tests/vexsim/fit_grip.py` fits vexsim's tire grip (forward and sideways,
  static and sliding) to those logs, and `test_fit_grip.py` checks that it
  gives back grip vexsim was given.

### Changed

- The template's `firmware/mclib.a` has no debug info: 2.7 MB, down from
  57.5 MB, and the zip is 10 MB, down from 65 MB. The program on the brain
  is the same. Crash backtraces inside mclib show function names but not
  file and line.
- A release's page shows its changelog section. The release fails if
  `CHANGELOG.md` has no section for the tagged version.
- `curveCircle()` and `curveCircleReverse()` keep their heading correction
  whole when a side would pass the voltage cap, and slow the arc instead.
  They stop steering back toward the circle over the last quarter of the
  sweep, so they end on their heading. In the physics simulator, a 90° arc
  of 24 in radius used to end 5-11° past its heading and spend the rest of
  its time turning back. On the 200 rpm drive it ran out of its 4 s. With two
  tracking wheels it now takes 3.84 s on the 200 rpm drive, 2.10 s on the
  450 rpm drive (was 2.59 s) and 1.85 s on the fast base (was 2.23 s), and
  ends 0.7-2.1° off heading (was 1.9-3.2°). With drive encoders only it ends
  2.0-2.6° off, under the 3° that starts the finishing turn (was 0.1-0.4°
  after that turn).
- The physics scenarios also run vexsim's `override_cascade` drive, in the
  `drive` and `horizontal` modes: four 450 rpm motors on 2.75 in omnis and
  one sideways tracking wheel.
- The Override field shapes come with vexsim, so path planner setup no
  longer cuts them from VEX's CAD (about 20 minutes). When they are missing
  the planner says to update vexsim. CI needs no key, so pull requests from
  forks run the planner tests too.

## 0.4.2

vexsim is public: https://github.com/kevinyhe/vexsim (MPL-2.0). Set up the
path planner once:

```sh
git clone https://github.com/kevinyhe/vexsim ../vexsim
cd ../vexsim
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
npm install --no-save three@0.160.0
```

The Override field shapes are cut from VEX's field CAD, so they are not
published. Download the Override field CAD from VEX and run vexsim's
`tools/override_cad.py` on it (vexsim's README, "Setup"; about 20 minutes).

### Changed

- CI checks out the public vexsim without a key. Only the Override field
  shapes still need `VEXSIM_CAD_DEPLOY_KEY`. Pull requests from forks skip
  the planner tests.

- Path planner: start it with vexsim's venv Python,
  `../vexsim/.venv/bin/python tests/vexsim/planner/server.py --vexsim ../vexsim`.
  It now needs vexsim 2.0, MuJoCo and numpy (`tests/vexsim/requirements.txt`),
  the Override field shapes and Three.js 0.160.0 in the vexsim checkout.
  The server checks all of these at start and exits with what is missing
  and the command that fixes it.
- Path planner: the 3D view is on the Override field only. It loads Three.js
  from the vexsim checkout at `/three/three.module.js`.

### Removed

- Path planner: the Push Back field. Plan on Override or the open floor.
- The Push Back game harness in `tests/vexsim/`: `metro_*`, `game_viewer.py`
  and the physics game audits.
- The validation reports in `tests/vexsim/` (`REPORT.md`, `*_VALIDATION.md`,
  `METRO_FRICTION.md`, `MOVEMENT_SCORING_AUDIT.md`). `ci_baseline.json`
  lists which scenarios pass.

## 0.4.1

Upgrading from 0.4.0:

- `wallReset()` stalls at 1000 mA by default, not 2500 mA, and
  `Routine::wallReset()` pushes at 3 V, not 4 V. A reset that never
  happened may now happen. Anything that holds the robot still for 50 ms
  reads as the wall: a game piece, another robot, or a push too weak to
  start it moving. Check the return value, and pass your own voltage and
  threshold if your drive stalls on something else first.
- `curveCircleReverse()` returns `InvalidValue` and does not move when the
  radius and the heading change have the same sign. It used to back the
  wrong way until its time limit. Flip the sign of the radius.
- After `Chassis::setPose()` to a new heading, `driveTo()` holds the new
  heading. It used to turn back toward the old one.
- `Routine::MotionStep::reversed(false)` drives the step forward. It used
  to leave a reversed step reversed.
- `control::correctOdometryPosition(x, y)` counts as setting the pose. A
  `PoseFusion` on the odometry task restarts there.
- `CommandController::getTrigger()` returns a trigger the controller owns.
  Remove any `delete` of it.

### Added

- Path planner (`tests/vexsim/planner`), replacing the motion builder.
  Click the field to place points; drag points and curve handles; set the
  heading at a point, a speed limit and a reverse flag per section. The
  path is coloured by the speed mclib's trajectory planner plans for it,
  with the planned time and top speed. **Run simulation** drives the whole
  routine in vexsim with mclib's C++ and checks each step, saying what was
  measured and the limit. Playback with telemetry, run comparison, the 3D
  view, undo and redo, JSON save and load, mirroring, and a C++ export
  that uses the same path code the simulator ran.
- Path planner: **Export C++** writes each routine as its own function
  straight into a PROS project folder: "Left side" becomes `runLeftSide()`
  in `src/autons/left_side.cpp`, with the two lines that add it to an
  `AutonSelector`. The robot's kS, kV, track width and ports go in
  `include/planner/robot_drive.hpp` and `robot_cascade.hpp`, which are
  written once and never replaced.
- Path planner: **Keep moving into the next step** runs a step with
  `exit = false`, so the next step starts at once without stopping or
  settling. A path chained into another path carries its speed across the
  join. A path that settles reports how far off it was when its plan ran
  out, and says when `followTrajectory()` had to back off and drive back in.
  Start it with `python3 tests/vexsim/planner/server.py --vexsim ../vexsim`.
- `examples/`: a complete robot program for each part of the library, from
  tracking wheels and every motion to mechanisms, commands, devices and
  telemetry. `make examples` compiles all of them with warnings as errors,
  and CI runs it.
- Path planner: eight more example routines, 18 in all. Between them they
  use every step type, drivetrain, odometry setup, heading source, lift
  height and action time, and a test fails when one is left out. The new
  ones cover the steps that are not paths, keeping moving between steps,
  path settings and gains, a worn field, no inertial sensor, a stack on the
  Center Goal, the lift's move times, and a wall reset on an Override wall.
  A second test runs them and checks what each description says.
- `control::readOdometry()` returns the pose with a count of how many times
  the pose has been set, and `control::odometryJumpCount()` returns the
  count alone. `control::correctOdometryPosition(x, y, jumps)` writes a
  position only while the count still matches.
- `control::OdometryFusion` steps a `PoseFusion` on the odometry pose and
  writes its correction back. The odometry task and the physics simulator
  both run it.
- `path::planArcThrough()` plans an arc from the turn to make, which can be
  more than half a circle.

### Changed

- `wallReset()` stalls at 1000 mA by default, down from 2500 mA. In the
  simulator a 14 lb drive with four 450 RPM motors never draws more than
  2.0 A a motor against a wall, so 2500 mA could not be reached. The same
  default changed on `ChassisController::makeWallResetCommand()` and
  `Routine::wallReset()`.
- `Routine::wallReset()` pushes at 3 V by default, down from 4 V. At 4 V the
  wheels of that drive slip at 27 RPM, above the 5 RPM stall limit, and the
  reset never happens.
- Path planner: a wall reset step starts at 3 V and 1000 mA, down from 6 V
  and 2500 mA.
- `curveCircleReverse()`, and `curveCircle()` with `reverse = true`, return
  `InvalidValue` without moving when the radius and the heading change have
  the same sign. That arc can only be driven forward. It used to back round
  the circle until its time limit and never count it as progress. It prints
  the radius and the two headings when it refuses. Half a circle runs
  whichever way its heading change wraps: a same-sign change of 160° or more
  is backed round the other way. Before, a `curveCircleReverse(180_deg, ...)`
  from 0° finished or ran out its time limit depending on which side of 0°
  the robot started.
- `control::correctOdometryPosition(x, y)` adds one to
  `control::odometryJumpCount()` when it moves the position.
- `CommandController::getTrigger()` returns the same trigger each time for a
  button, and the controller owns it. Do not delete it.

### Fixed

- `Chassis::setPose()` moves the heading that `driveTo()` and
  `correctHeading()` hold to the heading the IMU reads after the write.
  After a `setPose()` to a new heading, the next `driveTo()` used to turn
  back toward the old one.
- `Routine::MotionStep::reversed(false)` puts the step back to forward. It
  used to leave a reversed step reversed. On an arc step it clears the
  reverse flag, also for a step made with `curveCircleReverse()`: the signs
  of the radius and the heading change then pick the direction, as in
  `curveCircle()`.
- The odometry task restarts `PoseFusion` when the pose is set, by
  `setPose()`, `wallReset()`, `resetOdometry()` or a snapshot. The filter
  used to read the jump as distance driven, and carry the correction it
  still owed the old pose over to the new one. A correction worked out
  before the jump is dropped instead of written over the new pose. The
  restart keeps the filter's position uncertainty when it is above 0.5 in,
  as a circle of its larger axis. Heading uncertainty restarts at 0.5°.
- `CommandController::getTrigger()` no longer leaks a trigger on every call.

## 0.4.0

Upgrading from 0.3.0:

- `.until()` on a `Routine` no longer compiles. Put `.until()` on the motion
  step, or use `waitUntil()`.
- A `Routine` with a configuration error, such as a motion added with no
  `setChassis()`, no longer runs at all. It prints why and finishes.
- A `VelocityMechanism` ratio or an `Inertial` gain that is not a finite
  number above 0 is a configuration error instead of being replaced with
  1.0. The mechanism outputs 0 V; the IMU reads NaN, so motions refuse to
  run. Check `hasConfigurationError()` if one stopped working.
- Chained `moveToPoint()` and `boomerang()` hand off sooner: inside
  `distance_exit.big_error` (1.5 in) of the target, or when they pass it
  within 8 in. Routes that chain them reach the next move earlier.
- A stopping `moveToPoint()` turns in place when the target is more than
  30° off the nose within 8 in, and an explicit `min_speed` stops applying
  near the target. With a 4 V floor, some short or behind-the-robot moves
  take 0.5-0.8 s longer.

### Added

- Owning forms of the command decorators: `Command::andThen(a, b)`,
  `with(a, b)`, `race(a, b)`, `withTimeout(a, t)`, `until(a, condition)`,
  `repeatedly(a)` and `asProxy(a)` take the commands by `std::unique_ptr`
  and keep them alive inside the result. The member forms borrow, so
  `drive.makeMoveToPointCommand(...)->until(c)` left the result pointing at
  a command destroyed at the end of the statement.
- `Routine` motion steps take `.until(condition)`. The step ends as soon as
  the condition returns true: the motion is cancelled, the drive brakes and
  the next step starts. A condition already true when the step starts skips
  the motion. Unlike `Command::until()`, the routine owns the motion, so it
  can be built inline in the chain.
- `TrajectoryConstraints::speed_zones` caps the speed on stretches of a
  trajectory: `SpeedZone{20_in, 36_in, 15_in / 1_s}` keeps the robot at or
  under 15 in/s from 20 in to 36 in along the path. The plan brakes before
  the zone and speeds up after it. Before this, a slow section meant
  splitting the path into two trajectories, and the two splines met at
  different headings.
- `Routine` motion steps take `.atTime(t, command)`, `.atDistance(d, command)`
  and `.atDistanceFromEnd(d, command)`. Each starts its command partway
  through the motion, while the robot is still moving: a `trigger()` after a
  motion step only starts once the motion ends. `missedEvents()` counts the
  commands whose step ended before they came due.
- `early_exit_range` on `turnToAngle()`, `driveTo()`, `curveCircle()`,
  `curveCircleReverse()`, `swing()`, `moveToPoint()`, `boomerang()` and
  `followTrajectory()`, and on the matching `ChassisController` commands, and
  `.withEarlyExit(range)` on `Routine` motion steps. A chained motion ends
  once the robot is that close to its end, still moving. Above zero with
  `exit` true, or negative, the motion returns `InvalidValue`.
- `followTrajectory()` publishes `remaining_in` in its motion telemetry.
- `Chassis::setPose(QLength, QLength, QAngle)`:
  `chassis.setPose(0_in, -58_in, 180_deg)`. `Pose2D::theta` is radians, so
  `setPose(Pose2D(0, 0, 180))` sets 180 rad, about 233°.

### Changed

- A `VelocityMechanism` ratio that is not finite and > 0 is now a
  configuration error instead of being replaced with 1.0. The mechanism
  outputs 0 V, `getCurrentRpm()` is NaN and `hasConfigurationError()` is
  true. A typo like `ratio = 0` used to run the flywheel ungeared, at the
  wrong speed, with no error.
- The same for an `Inertial` gain: instead of being replaced with 1.0, a bad
  gain makes every reading NaN, like an unplugged IMU, and
  `hasConfigurationError()` is true. Motions then refuse to run.
- `.until()` on a `Routine` no longer compiles. It used to call
  `Command::until()` on the whole routine and throw the result away, so the
  condition was never checked. Use `until()` on a motion step, or
  `waitUntil()`.
- A `Routine` with a configuration error no longer runs at all. It prints
  why and finishes on its first tick. A motion added with no chassis used
  to become a step that did nothing while the rest of the routine ran, so
  a missing `setChassis()` showed up as a robot that ran its mechanisms and
  never drove.
- A chained `moveToPoint()` hands off as soon as the robot is inside
  `distance_exit.big_error` (1.5 in by default) of the target, or when it
  passes the target within 8 in. It used to hand off only when it crossed
  the line through the target within 1.5 in of it. On `six_motor_450` in the
  simulator it stalled 1.3 in short and timed out after 2 s, and at an 8 V
  floor the second move of a zigzag timed out.
- A chained `moveToPoint()` no longer sets the drive to 0 V when it starts.
  One that hands off on its first tick keeps the previous motion's wheel
  commands, as `boomerang()` already did.
- A chained `boomerang()` also hands off when it passes the target within
  8 in. It used to wait for the 1.5 in band, and in the simulator every
  chained boomerang timed out after 2.5 s, turning 215-420°.

### Fixed

- Examples in the docs and header comments that did not compile or did the
  wrong thing. Some called functions that do not exist
  (`Rotation::getAngleDeg()`, `makeDisableCommand()`, `driveVoltage()`),
  set config fields with assignments at file scope, or registered a default
  command held by a local that is destroyed when `initialize()` returns. Docs
  that disagreed with the code now match it, including `followTrajectory()`'s
  settle steps and how `driveTo()` and `curveCircle()` measure distance. The
  example `src/main.cpp` now sets its starting pose with `Chassis::setPose()`.
- A stopping `moveToPoint()` still circled its target on drives whose inner
  wheel stalls below its friction voltage or that slide sideways, such as
  all-omni bases. For the last 8 in it now drives a straight line through
  the target: it turns in place first when the target is more than 30° off
  the nose, backs up along the line if it passes the target, and turns in
  place again if the target drifts more than 30° off the line. Farther out,
  if it turns 90° without getting 1 in closer, it turns in place to face the
  target. An explicit voltage floor stops applying once it turns in place
  within 8 in, or comes within 0.5 in of the target along its line. The
  physics simulator ran 16 drivetrains (100-600 rpm; 2.75, 3.25 and 4 in
  wheels; 4 and 6 motors; all-omni, center traction and all-traction; one
  on 5.5 W motors), each with two tunings, on 13 routes at five voltage
  floors. Stops ended Reached within 1.5 in in 1726 of 1760 runs, up from
  1040. Timeouts fell from 652 to 23,
  and 21 of those are a 100 rpm drive that needs more than the 4 s limit.
  Runs that turned more than a full circle fell from 694 to 6, and the most
  any run turned fell from 1290° to 528°. The cost: with a 4 V floor on the
  stop, 15 of 293 short or behind-the-robot moves take 0.5-0.8 s longer,
  because the robot turns in place first and then runs without the floor.
- `moveToPoint()` could circle its target until the time limit. Inside 8 in
  it let the heading correction reach the full voltage cap, so a target
  passed a few inches to the side pulled the robot round it (455° and a
  5 in miss in the simulator, from 12 V with drive kP 0.76). The correction
  is now capped at the arc that runs from the current heading through the
  target.
- `turnToPoint()` with direction -1 always turned clockwise, up to 360 deg.
  It added the 180 deg after wrapping the target to the nearest turn instead
  of before, so the back of a robot at heading 0 reached a point at bearing
  170 deg by turning 350 deg instead of -10 deg. It now takes the short way.
- `correctHeading()` sized its integral window from the target heading
  instead of the error: no window at all for a target of 0 deg, 360 deg
  wide at 900 deg. With a non-zero `heading_pid` ki, the same error was
  integrated differently depending on where the robot pointed. It now uses
  the same 1 deg window as the heading correction inside the drive motions.
- `PositionMechanism::makeStateCommand()` and `makeStateUntilCommand()`
  restarted the PID every tick, so the mechanism never reported
  `atTarget()` and the D term did nothing. A `makeStateUntilCommand()` that
  waited for `atTarget()` never ended. They now restart the loop only when
  the target changes or a manual voltage has taken over.
- A `Sequence` with no commands read past the end of its list when it
  started, which crashed on the host (segfault) and is undefined behaviour
  on the brain. It now finishes on its first tick.

## 0.3.0

Upgrading from 0.2.0:

- Device getters that return a plain number give NaN (or 0 for counts and
  the raw controller stick) when a device is unplugged, instead of PROS's
  2147483647 or infinity. A check like `if (motor.getTemperature() > 55)`
  is now false on a dead port. Code that compared against `PROS_ERR` /
  `PROS_ERR_F` from those getters should use `std::isnan()` or the
  `std::optional` versions.
- `Inertial::reset()` waits for calibration by default. Call
  `reset(false)` for the old behaviour.
- Stopped moves keep at least `MotionConfig::settle_min_voltage` (1.5 V)
  near the target, and `curveCircle()` steers on the odometry pose. Set
  `settle_min_voltage` to 0 to turn the floor off.

### Added

- `device::Controller` covers the rest of `pros::Controller`: press and
  release edges (`getNewPress`, `getNewRelease`), `print`, `clearLine`,
  `clear`, `rumble`, `isConnected`, `batteryCapacity`, `batteryLevel` and
  `id`. Writes return whether PROS took them; a disconnected controller's
  stick reads 0 from `analog()` instead of PROS_ERR / 127.
- `PoseFusion::setGps()`: blend a V5 GPS into the pose, with or without
  distance sensors. In the physics simulator with drive encoders only, RAMSETE
  arcs end 0.78-1.44 in off with the GPS, against 2.36-4.23 in with distance
  sensors.
- `tools/tune_drive.py` prints the track width to use for heading from the
  encoders (no IMU), from the `characterizeSpin()` log.
- Drive tuning from a log. `characterizeDrive()`, `characterizeSpin()` and
  `traceMotion()` (`mclib/control/characterize.hpp`) log fixed-voltage runs
  and a traced motion; `tools/tune_drive.py` fits kS, kV, kA and the
  effective track width and says which RAMSETE gain to change. In the
  physics simulator the fit is within 1.2% of the true kV and 0.4% of the
  true track width.

### Changed

- `Inertial::reset()` waits for calibration (about 2 s) by default.
  `reset(false)` keeps the old return-at-once behaviour.
- Stopped distance moves (`driveTo()`, `moveToPoint()`) keep at least
  `MotionConfig::settle_min_voltage` (1.5 V; 0 turns it off) while outside
  `distance_exit.small_error`, instead of crawling at the friction limit. In
  the physics simulator, moves that passed before take 14% less time and end
  44% closer (with tracking wheels 2.79 -> 2.41 s and 1.07 -> 0.60 in on
  average); a 24 in drive on a 450 rpm base goes from 2.16 s and 1.26 in to
  1.38 s and 0.03 in.

### Fixed

- Device getters that return a plain number no longer pass PROS's error
  value through on an unplugged or misconfigured device. Decimal getters
  (motor position, velocity, current and temperature; `Rotation`,
  `AdiEncoder` and `AdiPotentiometer` angles; `Distance::getDistanceIn`;
  GPS x, y, heading and error; optical hue, saturation and brightness)
  return NaN, which fails every comparison: `temperature > 55` and
  `current > 2500` are false on an unplugged motor instead of true.
  `Vision::count` and `AiVision::count` return 0 (a loop to `count()` ran
  2^31 times), `Controller::getAnalog` returns 0, and `Line::get` reads no
  line instead of "dark line found". Raw integer getters (`getTicks`,
  `getRaw`, `getDistanceMm`, `getConfidence`) still return the raw value.
- `MotorGroup` averages skip motors that are not reporting; one dead motor
  made `getAveragePositionDeg()` infinite. None reporting gives NaN, not 0.
- `setPercent()` rounds: 0.999 sent 126 of 127.
- `Inertial::getHeadingDeg()` wraps after applying the gain (359 deg at gain
  1.01 read 362.6), and a gain of 0 is replaced by 1.
- `Pneumatic` records its new state only when the write succeeds; a failed
  write reported the new state and the next `toggle()` went the wrong way.
- The example `src/main.cpp` shows "ODOM NOT RUNNING" on the controller if
  `startOdometry()` rejects its setup.
- `boomerang()` started its final turn at the closest approach while still
  moving, and coasted out of the band: speed_base backed in at 7.6 in/s and
  ended 1.67 in off. It now waits until the robot is under 4 in/s.
- The physics simulator put 30-38% of the robot's weight on the two
  spring-loaded tracking wheels (vexsim PR #1), so robots with tracking
  wheels lost grip and spun their wheels at launch. Every tracking-wheel
  result before this was measured on that robot. The simulator also runs
  with only a vertical or only a horizontal tracker now; see
  getting-started.md for which to fit first.
- `curveCircle()` arcs ended 6-15 in off and ran out their time limit in the
  physics simulator. They measured progress on the outer wheel's encoder,
  which slips forward, and crawled the last inches. They now measure
  progress on the pose (the angle swept around the arc's centre), steer
  back onto the circle, keep `settle_min_voltage`, slow to keep sideways
  acceleration under the new `MotionConfig::arc_max_lateral_accel` (60
  in/s^2), and finish a stopped arc with a turn if it ends off heading. With
  tracking wheels 8 of the 12 arc scenarios pass (0 before); with drive
  encoders only, 8 of 12 (0 before). four_motor_200 still runs out its 4 s
  limit with tracking wheels.
- `boomerang()` arrived facing the wrong way on some drives (#101). Close
  in, the bearing to the carrot swung by up to 150 deg, and four_motor_200
  and speed_base ran out their 4 s limit 43 and 102 deg off. Within 3x the
  position band it now holds the bearing it had, turns toward the final
  heading over the last few inches, and starts the final turn at its
  closest approach, with the same `settle_min_voltage` push as `driveTo()`.
  If that approach is too far out it re-aims at the target, at most twice.
  In the physics simulator with tracking wheels every boomerang scenario
  passes (2 of 6 before), ending 0.23-1.05 in and 0.8-2.4 deg off.
  RAMSETE's low-grip back-off, which uses `boomerang()`, ends 2.34 in off
  in 6.33 s, against 1.09 in in 6.48 s before.
- A `boomerang()` that turned in place to its final heading and drifted just
  outside the position band went back to driving, then turned again, until
  its time limit. It now resumes only past twice the band.
- `driveTo()` stopped short on fast drivetrains: it measured distance with the
  drive encoders, which count wheel slip as progress. In the physics
  simulator a 24 in drive on a 450 rpm base stopped 2.7-4.8 in short with
  tracking wheels fitted. New `MotionConfig::drive_distance_source`; the
  default, `Auto`, measures along the odometry pose when a forward tracking
  wheel is fitted. Those drives now stop 1.1-1.3 in from the target.

## 0.2.0

Upgrading from 0.1.0: the eight blocking motions (`turnToAngle`, `driveTo`,
`curveCircle`, `curveCircleReverse`, `swing`, `turnToPoint`, `moveToPoint`,
`boomerang`) now return `mclib::control::MotionResult` instead of `void`.
Code that ignores the return value compiles unchanged; code that took their
address as a `void` function pointer needs the new type.
`ConveyorConfig::unjam_voltage` is now a magnitude: its sign is ignored.

### Added

- Motion builder (`tests/vexsim/builder`): follow steps take a list of
  waypoints, **Plan speeds** draws the C++ planner's trajectory coloured by
  speed, and **Export C++** turns a sequence into an mclib `autonomous()`.
- `useRamseteForArcs()`: `curveCircle()` and `curveCircleReverse()` plan
  their arc (`path::planArc()`) and follow it with RAMSETE on the odometry
  pose. In the simulator with tracking wheels, 24 in arcs land within
  0.37-1.59 in instead of 5.9-15.1 in.
- `mclib::control::PoseFilter`: a Kalman filter that blends odometry with
  distance sensor readings against the field walls and GPS poses, rejecting
  readings that don't fit.
- `mclib::control::PoseFusion` and `OdometrySetup::fusion`: the odometry task
  runs a `PoseFilter` from distance sensors after every tick and corrects
  odometry's position toward it, limited to 60% of the distance moved per
  tick so a settling robot sees a still pose.
- `mclib::control::AsyncMotion`: run a motion on its own task, wait for a
  time, distance or condition partway through, then collect its
  `MotionResult`. Starting a new one cancels the old one.
- `HolonomicController::followTrajectory()` and
  `makeFollowTrajectoryCommand()`: X-drive and mecanum trajectory following
  with a heading target separate from the direction of travel, then settling
  on the last pose. The step itself is `control::holonomicFollowStep()`.
- `mclib::path::Trajectory`: plans speed and time along a path from limits on
  speed, acceleration, deceleration, cornering and outer-wheel speed.
- `followTrajectory()` and `mclib::control::Ramsete`: a RAMSETE follower for
  tank drives that corrects using the odometry pose instead of the drive
  encoders. `RamseteConfig::track_width` takes the drive's effective track
  width, and the commanded speed is capped at the trajectory's top speed.
- `ChassisController::makeFollowTrajectoryCommand()` and
  `Routine::followTrajectory()`: RAMSETE following as a command and as a
  routine step. Both keep their own copy of the trajectory.
  With `exit` true it finishes with a turn to the final heading
  (`RamseteConfig::turn_to_final_heading`) and a straight drive to close any
  miss along it (`RamseteConfig::settle_position`).

### Changed

- `turnToAngle`, `driveTo`, `curveCircle`, `curveCircleReverse`, `swing`,
  `turnToPoint`, `moveToPoint` and `boomerang` return a
  `mclib::control::MotionResult` instead of `void`, so a routine can tell a
  reached target from a timeout, cancel, disable, bad value or missing drive.
  Code that ignores the return value still compiles.

### Fixed

- `followTrajectory()` could end several inches to the side of its end point
  when the robot slid (12 in on low grip in the simulator). When the sideways
  miss is over `RamseteConfig::settle_side_tolerance` (3 in), it now backs
  off along its heading and comes back in with `boomerang()`.
- `ConveyorMechanism` unjammed at a fixed `unjam_voltage` (-8 V), so a jam in
  Reverse was "unjammed" in the same direction it jammed. The unjam now runs
  opposite to the jammed direction; only the size of `unjam_voltage` is used.
- `MotorStateMechanism` kept its state through a disable, so after re-enable
  it drove the pre-disable voltages for two ticks, or until a command changed
  the state. It now goes to an off state on disable. New constructors take the
  off state; the existing ones use the initial state.
- Timed commands could finish one scheduler tick late. `makeStateForCommand`,
  `WaitCommand`, `WaitUntilTimeoutCommand`, and the timed commands of
  `PositionMechanism`, `PresetPositionMechanism` and `ToggleGroupMechanism`
  compared times as seconds in a `double`, so 100 ms read as 99.99999 ms for
  59% of start times. They now compare whole milliseconds with the new
  `time::hasElapsed()`. `WaitCommand` now finishes when its duration has
  passed, not a tick after.
- `EventLoop::poll()` no longer reads freed memory when a binding calls
  `bind()` or `clear()` on the same loop, for example a command started by a
  `Trigger` that creates another `Trigger`. New bindings run from the next
  poll; `clear()` skips the rest of the current poll.
- `ProxyCommand` no longer calls through a null pointer when its supplier
  returns `nullptr`. The proxy finishes on the next tick instead.
- `MotorSubsystem` stored NaN as its commanded voltage while the motors got
  0 V, and turned an infinite voltage or percent into ±12 V. Both now become
  0 V, so `getCommandedVoltage()` matches what the motors get.
- `PTOMechanism` settle window and drive watchdog could run one scheduler tick
  long. They compared times as seconds in a `double`, so 250 ms could read as
  249.99999 ms depending on the clock value at the shift or write. They now
  compare whole milliseconds.

## 0.1.0

Initial release.

- Tank, X-drive and mecanum drivetrains with PID distance, turn and heading
  control, boomerang moves, swings, arcs and wall reset
- Odometry from drive encoders or tracking wheels
- Motion profiles, feedforward and a profile follower
- Catmull-Rom splines and pure pursuit
- Command scheduler with subsystems, sequences, parallel and race groups,
  triggers and command decorators
- Mechanisms: position, velocity, toggle, toggle group, multi-position, preset
  position, discrete actuator, conveyor, homing, PTO, pneumatic and
  auto-trigger, plus `MechanismManager`
- Device wrappers for motors, rotation, IMU, distance, optical, vision, AI
  vision, GPS, line and ADI sensors
- CSV telemetry to the SD card or USB serial
- Autonomous routine builder and selector
- Compile-time units
- Host test suite and physics simulator harness
