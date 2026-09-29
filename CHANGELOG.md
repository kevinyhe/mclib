# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow
[semantic versioning](https://semver.org/).

## Unreleased

### Added

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
