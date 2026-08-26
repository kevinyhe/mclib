// mclib
#pragma once

#include "api.h"

namespace mclib {
namespace device {

enum class Gearset {
  Red,
  Green,
  Blue,
};

enum class BrakeMode {
  Coast,
  Brake,
  Hold,
};

enum class ControllerId {
  Master,
  Partner,
};

enum class AnalogAxis {
  LeftX,
  LeftY,
  RightX,
  RightY,
};

enum class DigitalButton {
  A,
  B,
  X,
  Y,
  Up,
  Down,
  Left,
  Right,
  L1,
  L2,
  R1,
  R2,
};

pros::MotorGears toProsGearset(Gearset gearset);
pros::motor_brake_mode_e_t toProsBrakeMode(BrakeMode mode);
pros::controller_id_e_t toProsControllerId(ControllerId id);
pros::controller_analog_e_t toProsAnalogAxis(AnalogAxis axis);
pros::controller_digital_e_t toProsDigitalButton(DigitalButton button);

}  // namespace device
}  // namespace mclib
