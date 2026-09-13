# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow
[semantic versioning](https://semver.org/).

## Unreleased

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
