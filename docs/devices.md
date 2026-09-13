# Device wrappers

Thin typed wrappers over the PROS device APIs.

[Documentation index](README.md) · [Project README](../README.md)

## Device wrappers

Every wrapper lives in `mclib::device`, owns the PROS object, and exposes two kinds of getter. The plain one (`getDistanceCm()`, `getHue()`, ...) returns exactly what PROS returns, including `PROS_ERR` / `PROS_ERR_F` on a bad port. The typed one (`distance()`, `hue()`, `position()`, ...) returns `std::optional` and is `nullopt` on those errors, so an unplugged sensor is something you have to unwrap rather than a 2-billion-degree angle that flows into odometry.

ADI (3-wire) wrappers take the port as a char (`'A'`..`'H'`) and have a second constructor `(expander_smart_port, adi_port, ...)` for a 3-wire expander.

| Class | PROS device | Header |
|---|---|---|
| `Motor`, `MotorGroup` | `pros::Motor` | `mclib/device/motor.hpp`, `motor_group.hpp` |
| `Controller` | `pros::Controller` | `mclib/device/controller.hpp` |
| `Inertial` | `pros::Imu` | `mclib/device/inertial.hpp` |
| `Rotation` | `pros::Rotation` | `mclib/device/rotation.hpp` |
| `Distance` | `pros::Distance` | `mclib/device/distance.hpp` |
| `Optical` | `pros::Optical` | `mclib/device/optical.hpp` |
| `Gps` | `pros::Gps` | `mclib/device/gps.hpp` |
| `Vision` | `pros::Vision` | `mclib/device/vision.hpp` |
| `AiVision` | `pros::AIVision` | `mclib/device/ai_vision.hpp` |
| `Pneumatic` | `pros::adi::DigitalOut` | `mclib/device/pneumatic.hpp` |
| `AdiEncoder` | `pros::adi::Encoder` | `mclib/device/adi_encoder.hpp` |
| `AdiPotentiometer` | `pros::adi::Potentiometer` | `mclib/device/adi_potentiometer.hpp` |
| `AdiDigitalIn` (`LimitSwitch`, `Bumper`) | `pros::adi::DigitalIn` | `mclib/device/adi_digital_in.hpp` |
| `AdiAnalogIn` | `pros::adi::AnalogIn` | `mclib/device/adi_analog_in.hpp` |
| `AdiUltrasonic` | `pros::adi::Ultrasonic` | `mclib/device/adi_ultrasonic.hpp` |
| `AdiLed` | `pros::adi::Led` | `mclib/device/adi_led.hpp` |

Things to know:

- `Gps::pose()` is already in mclib's field frame (inches, 0 deg = +Y, clockwise). PROS metres and its 0 = north heading map onto it with unit scaling only. Offsets are in inches, robot frame (+X right, +Y forward).
- `AdiEncoder` reads 1 tick per degree and does not wrap, so `position()` drops into `DriveGeometry::encoderToDistance()` the same way a `Rotation` does.
- `AdiDigitalIn::newPress()` keeps its own edge flag. PROS's `get_new_press()` shares one flag per port across every caller.
- `AdiUltrasonic::distance()` is `nullopt` for the 0 / -1 "no echo" readings as well as `PROS_ERR`; left raw, "nothing in range" reads as 0 cm.
- `AdiPotentiometer` defaults to the V2 pot. PROS defaults to V1.
- `Vision` sets the sensor to centre-origin, so object `x`/`y` are pixels off centre (+y down). `AiVision` objects are top-left origin in a 320 x 240 frame; use `center_x - 160`.
- `Optical::isColor(hue_min, hue_max, min_proximity)` handles ranges that cross 360, so red is `isColor(340, 20, 100)`.

Example: stop the intake when a ring of the wrong colour is seen, and track a wheel with a shaft encoder.

```cpp
#include "mclib/mclib.hpp"

mclib::device::Optical ring_sensor(3);
mclib::device::AdiEncoder tracking_wheel('A', 'B');
mclib::device::LimitSwitch arm_stop(1, 'C');  // on the expander in port 1

void initialize() {
  ring_sensor.setLedPwm(100);           // constant light, stable hue
  ring_sensor.setIntegrationTimeMs(20); // fast enough to catch a passing ring
  tracking_wheel.reset();
}

void opcontrol() {
  while (true) {
    if (ring_sensor.isColor(340, 20, 100)) {   // red, and close
      intake.stop();
    }
    if (auto angle = tracking_wheel.position()) {
      // *angle is a QAngle; nullopt means the encoder is unplugged
    }
    if (arm_stop.newPress()) {
      arm.hold();
    }
    pros::delay(10);
  }
}
```
