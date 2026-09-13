# Changelog

Notable changes to mclib. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[semantic versioning](https://semver.org/).

## Unreleased

Nothing yet.

## 0.1.0

First release. Packaged as a PROS template:
`pros c fetch mclib@0.1.0.zip` then `pros c apply mclib`.

### Added

- **Chassis and motion.** Differential and holonomic (X-drive, mecanum)
  drivetrains, PID distance/turn/heading loops, slew limiting, a stiction
  floor, boomerang point moves, swing and arc turns, and wall reset.
- **Odometry.** Arc-based pose tracking on a dedicated task, from drive
  encoders or dedicated tracking wheels, behind a lock.
- **Motion profiling.** Trapezoidal profiles and a drivetrain feedforward model
  (`kS`, `kV`, `kA`) with a profile follower.
- **Paths.** Catmull-Rom spline generation and a pure pursuit follower with
  curvature and approach speed limits.
- **Command framework.** A WPILib-shaped scheduler with subsystems,
  requirements, default commands, sequences, parallel and race groups,
  conditional and proxy commands, triggers and an event loop.
- **Mechanisms.** Position, velocity, toggle, toggle group, multi-position,
  preset position, discrete actuator, conveyor with jam recovery, homing
  against a hard stop, PTO, pneumatics, and sensor auto-triggers, plus a
  `MechanismManager` that owns their default commands.
- **Devices.** Typed wrappers for motors, motor groups, rotation sensors, IMUs,
  distance, optical, vision, AI vision, GPS, line and the ADI analog, digital,
  encoder, LED, potentiometer and ultrasonic ports.
- **Telemetry.** Fixed-rate CSV logging to the SD card or over USB serial, with
  a background flush task and overflow accounting.
- **Autonomous.** A routine builder with a time budget, and an on-screen
  selector that persists the last choice.
- **Units.** A compile-time dimensional analysis library, so a gain with the
  wrong units fails to compile.
- **Testing.** 37 host test binaries that build with a host `g++`, plus a
  physics simulator that replays the real C++ controller.

### Fixed

- **Command decorators leaked.** `andThen()`, `with()`, `race()`,
  `withTimeout()`, `until()`, `repeatedly()` and `asProxy()` returned a raw
  `new` pointer that nothing ever deleted, and `withTimeout()`/`until()` also
  leaked the helper command they built internally. They now return
  `[[nodiscard]] std::unique_ptr<Command>`, and the wrapper owns the helper it
  creates. `Trigger::andOther()`, `orOther()` and `negate()` return
  `std::unique_ptr<Trigger>` for the same reason.
- **`device::Line` used a deprecated PROS typedef**, which produced a warning in
  every project that included it. It uses `pros::adi::AnalogIn` now.
- **Two warnings in `RepeatCommand`**: an unused parameter and a
  copy-elision-defeating `std::move` on a return value.
- **The README was structurally broken.** A merge in an early pull request
  dropped code fences and spliced the tails of several sections into their
  neighbours, so large parts of the file rendered as one code block on GitHub.
  The document has been repaired and split into `docs/`.
