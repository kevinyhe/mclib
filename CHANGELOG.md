# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow
[semantic versioning](https://semver.org/).

## Unreleased

### Fixed

- `moveToPoint()` could circle its target until the time limit. Inside 8 in
  it let the heading correction reach the full voltage cap, so a target
  passed a few inches to the side pulled the robot round it (455° and a
  5 in miss in the simulator, from 12 V with drive kP 0.76). The correction
  is now capped at the arc that runs from the current heading through the
  target.

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
