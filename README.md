<!-- // mclib -->
# mclib

A PROS 4 library for VEX V5: odometry, motion control, a command-based
framework, and classes for common mechanisms (lifts, flywheels, conveyors,
pneumatics).

Ships as a PROS template, the same way LemLib does. Headers live in
`include/mclib/`, implementations in `src/mclib/`, and the repository is itself
a buildable PROS project.

**[Documentation](docs/README.md)** ·
[Installation](docs/installation.md) ·
[Getting started](docs/getting-started.md) ·
[Safety](docs/safety.md) ·
[Known limits](docs/accuracy.md)

## Install

Download `mclib@<version>.zip` from the
[releases page](https://github.com/kevinyhe/mclib/releases), then, inside your
PROS project:

```sh
pros c fetch mclib@0.1.0.zip
pros c apply mclib
```

```cpp
#include "mclib/mclib.hpp"
```

Full instructions, including version pinning and upgrades, are in
[docs/installation.md](docs/installation.md).

## What it does

- **Drivetrain control.** Differential and holonomic (X-drive, mecanum)
  chassis, with PID distance/turn/heading loops, slew limiting and a stiction
  floor.
- **Odometry.** Arc-based pose tracking on its own task, from drive encoders or
  dedicated tracking wheels, behind a lock so a torn read is impossible.
- **Motion.** Trapezoidal profiles, drivetrain feedforward (`kS`/`kV`/`kA`),
  Catmull-Rom splines and a pure pursuit follower.
- **Commands.** A WPILib-shaped scheduler: subsystems, requirements, default
  commands, sequences, parallel groups, triggers.
- **Mechanisms.** Position, velocity, toggle, conveyor with jam recovery,
  homing against a hard stop, PTO, pneumatics, named presets, sensor
  auto-triggers.
- **Devices.** Typed wrappers over the PROS motor, rotation, IMU, distance,
  optical, vision, GPS and ADI APIs.
- **Telemetry.** Fixed-rate CSV to the SD card or over USB serial, with a
  background flush task.
- **Autonomous.** Routine builder with a time budget, and an on-screen
  selector that remembers the last choice.
- **Units.** Compile-time dimensional analysis, so handing `kV` a bare number
  is a compile error rather than a loop that is 39.37x wrong.

## A first program

```cpp
#include "main.h"
#include "mclib/mclib.hpp"

using namespace mclib::units::literals;

mclib::device::Controller master;
auto imu = std::make_shared<mclib::device::Inertial>(15);

mclib::Chassis chassis(
    {-11, 13, 14},                       // left motors, negative = reversed
    {-16, 17, -18},                      // right motors
    mclib::device::Gearset::Blue,
    mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in),
                             11.375_in,  // track width
                             1.0},       // wheel revs per motor rev
    imu);

mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},   // volts per inch
    .turn_pid = {0.3, 0.0, 1.5},       // volts per degree
    .heading_pid = {0.3, 0.0, 1.5},    // volts per degree, while driving
});

void initialize() {
  imu->reset(true);
  mclib::control::startOdometry();

  static auto default_drive = drive.makeArcadeDriveCommand(master);
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
}

void autonomous() {
  moveToPoint(0_in, 24_in, 1, 2_s);
  turnToAngle(90_deg, 1_s);
}

void opcontrol() {
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
```

There are no default dimensions. A wrong wheel size silently scales every
autonomous by 5%, so mclib makes you say which measurement you took.

## Before you trust it on a field

The shipped gains were tuned against a simulated robot. Tune them on yours.
At default gains, several arc and boomerang motions still miss their endpoints.
[Measured accuracy and known limits](docs/accuracy.md) has the exact numbers and
the reason: drive encoders cannot see a wheel slipping sideways.

The safety checks all pass. A motion that cannot reach its target still stops,
reports failure, and leaves the drive de-energised. Read
[Safety and lifecycle](docs/safety.md) and test with the wheels raised first.

## Building from source

Requires the PROS CLI and the Arm GNU toolchain (`arm-none-eabi-g++`).

```sh
make              # build the PROS project
make library      # build bin/mclib.a
make template     # package mclib@<version>.zip
make test         # build and run the host test suite
```

`make test` runs on any machine with `g++`. Every test is a single file under
`tests/` with its own `main()`.

## Requirements

- PROS kernel 4.2.2 or newer
- `gnu++20` (the PROS kernel headers already require C++20)
- Eigen, bundled in `include/Eigen` and shipped with the template. Fixed-size
  types only; dynamic types like `MatrixXd` do not belong on the brain.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Bug reports and pull requests are
welcome.

## License

[Mozilla Public License 2.0](LICENSE). You can use mclib in your robot code
without opening that code; changes to mclib's own files stay open.
