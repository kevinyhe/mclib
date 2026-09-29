# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow
[semantic versioning](https://semver.org/).

## Unreleased

### Added

- `HolonomicController::followTrajectory()` and
  `makeFollowTrajectoryCommand()`: X-drive and mecanum trajectory following
  with a heading target separate from the direction of travel, then settling
  on the last pose. The step itself is `control::holonomicFollowStep()`.
- `mclib::path::Trajectory`: plans speed and time along a path from limits on
  speed, acceleration, deceleration, cornering and outer-wheel speed.

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
