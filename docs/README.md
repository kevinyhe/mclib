# mclib documentation

Start with [Installation](installation.md), then
[Setting up your robot](getting-started.md). Read
[Safety and lifecycle](safety.md) and
[Measured accuracy and known limits](accuracy.md) before an autonomous runs on a
field.

## Getting going

| Page | What is in it |
| --- | --- |
| [Installation and building](installation.md) | Installing the PROS template, building from source, Eigen, running the tests |
| [Setting up your robot](getting-started.md) | Ports, chassis geometry, odometry, your first opcontrol |
| [Safety and lifecycle](safety.md) | What stops a motion, what happens on field disable, what to verify yourself |
| [Measured accuracy and known limits](accuracy.md) | What is tested, what is not, and which motions still fail at default gains |

## The API

| Page | What is in it |
| --- | --- |
| [Units and the coordinate frame](units.md) | Compile-time units, the compass field frame |
| [Core math API](math.md) | Geometry and helper functions |
| [Commands and subsystems](commands.md) | The scheduler, subsystem registration, command ownership |
| [Mechanisms](mechanisms.md) | Position, velocity, toggle, conveyor, homing, PTO, pneumatics, presets, auto-triggers |
| [Motion](motion.md) | Motion profiles, drivetrain feedforward, pure pursuit, X-drive and mecanum |
| [Autonomous routines and the selector](autonomous.md) | Building routines, time budgeting, on-screen selection |
| [Telemetry and time](telemetry.md) | CSV logging to SD, USB serial streaming, the clock seam |
| [Device wrappers](devices.md) | Typed wrappers over the PROS device APIs |

## Everything else

| Page | What is in it |
| --- | --- |
| [Simulator and motion builder](simulator.md) | Replaying the real controller against the physics simulator |
| [Migrating](migrating.md) | Moving off the old `config.cpp` layout |
| [Contributing](../CONTRIBUTING.md) | Building, testing and submitting changes |
| [Changelog](../CHANGELOG.md) | What changed between releases |
