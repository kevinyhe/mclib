# Migrating

Moving off the old `config.cpp` layout.

[Documentation index](README.md) · [Project README](../README.md)

Earlier versions shipped one robot's devices as globals inside the library
(`left_chassis`, `right_chassis`, `inertial_sensor`, `intake`, `hood`, ...)
and a set of bare tuning globals (`distance_kp`, `min_output`,
`max_slew_accel_fwd`, ...) in `config.cpp`. Both are gone. What replaces them:

| Was | Now |
|---|---|
| `left_chassis`, `right_chassis`, `inertial_sensor` | your own `mclib::Chassis`, bound by `ChassisController` or `control::bindDrive()` |
| `distance_kp` and the other gain globals | `ChassisControllerConfig` fields (`distance_pid`, `turn_pid`, `heading_pid`) |
| `min_output` | `ChassisControllerConfig::min_voltage` |
| `max_slew_*`, `dir_change_start`, `dir_change_end` | `ChassisControllerConfig::slew`, `dir_change_start`, `dir_change_end` |
| `chase_power`, `heading_correction` | the same names on `ChassisControllerConfig` |
| `startOdometry()` reading `vertical_tracker` | `startOdometry(setup)` with an `OdometrySetup` naming your sensors |
| `odometryConfigFromGlobals()` | `odometryConfigFrom(setup)` |
| `left_reset` / `right_reset` as default snapshot sensors | no defaults; call `snapshot_config_set_sensors()` |
| `mclib::config::robot_drive_geometry` read by `motion.cpp` | nothing reads it; pass it to `Chassis` yourself |

The motion routines themselves (`moveToPoint`, `turnToAngle`, `boomerang`,
...) have the same signatures. `ChassisController::makeArcadeDriveCommand()`
takes `DriveCurveConfig`s where it used to take a scale factor.
