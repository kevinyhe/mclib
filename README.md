<!-- // mclib -->
# mclib

A PROS 4 library for VEX V5 robots. It tracks the robot's position on the
field, drives it to targets in autonomous, and provides ready-made classes for
common mechanisms.

[Documentation](docs/README.md) ·
[Installation](docs/installation.md) ·
[Getting started](docs/getting-started.md) ·
[Safety](docs/safety.md) ·
[Known limits](docs/accuracy.md)

## Install

Download `mclib@<version>.zip` from the
[releases page](https://github.com/kevinyhe/mclib/releases). In your PROS
project:

```sh
pros c fetch mclib@0.1.0.zip
pros c apply mclib
```

```cpp
#include "mclib/mclib.hpp"
```

See [docs/installation.md](docs/installation.md) for version pinning and
upgrades.

## Features

- **Drivetrains.** Tank (differential), X-drive and mecanum. PID loops for
  driving, turning and holding heading, with acceleration limits and a minimum
  voltage to overcome friction.
- **Odometry.** Tracks the robot's x, y and heading in the background using
  drive encoders or tracking wheels.
- **Motion.** Drive to a point, turn to an angle, follow curved paths (pure
  pursuit), and plan smooth speed changes (motion profiles).
- **Commands.** A command-based framework like FRC's: subsystems, default
  commands, sequences, parallel groups and controller button triggers.
- **Mechanisms.** Lifts and arms, flywheels, pneumatics, conveyors with jam
  recovery, homing against a hard stop, PTOs and sensor auto-triggers.
- **Devices.** Wrappers for PROS motors and sensors. Readings come back empty
  when a device is unplugged, instead of as an error code.
- **Telemetry.** Log values to a CSV file on the SD card or to the PROS terminal.
- **Autonomous.** Chain motions and mechanism actions into a routine, and pick a
  routine on the brain screen.
- **Units.** Values carry units (`24_in`, `90_deg`, `2_s`), and mixing
  incompatible units is a compile error.

## Example

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

## Building

Requires the PROS CLI and the Arm GNU toolchain (`arm-none-eabi-g++`).

```sh
make              # build the PROS project
make library      # build bin/mclib.a
make template     # package mclib@<version>.zip
make test         # build and run the host test suite
```

`make test` needs only a host `g++`.

## Requirements

- PROS kernel 4.2.2 or newer
- `gnu++20`
- Eigen, bundled in `include/Eigen`. Use fixed-size types only.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MPL-2.0](LICENSE).
