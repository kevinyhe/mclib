# Devices

Wrappers live in `mclib::device` and own their PROS object. Each has two kinds
of getter:

- Plain getters (`getDistanceCm()`, `getHue()`) return the PROS value, including
  `PROS_ERR` / `PROS_ERR_F`.
- Typed getters (`distance()`, `hue()`, `position()`) return `std::optional`,
  empty on error.

ADI wrappers take a port char (`'A'`..`'H'`). A second constructor,
`(expander_smart_port, adi_port, ...)`, uses a 3-wire expander.

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

## Notes

- `Gps::pose()` is in the field frame: inches, 0° = +Y, clockwise. Offsets are
  inches in the robot frame (+X right, +Y forward).
- `AdiEncoder` reads 1 tick per degree and does not wrap.
- `AdiDigitalIn::newPress()` keeps its own edge flag. PROS `get_new_press()`
  shares one flag per port.
- `AdiUltrasonic::distance()` is empty for 0 and -1 (no echo) and `PROS_ERR`.
- `AdiPotentiometer` defaults to the V2 potentiometer.
- `Vision` uses a centered origin: `x`/`y` are pixels from center, +y down.
  `AiVision` uses a top-left origin in a 320 × 240 frame.
- `Optical::isColor(hue_min, hue_max, min_proximity)` handles ranges that wrap
  past 360, e.g. red is `isColor(340, 20, 100)`.

## Example

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
