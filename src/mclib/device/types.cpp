// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/types.hpp"

namespace mclib {
namespace device {

pros::MotorGears toProsGearset(Gearset gearset) {
  switch (gearset) {
    case Gearset::Red:
      return pros::MotorGears::red;
    case Gearset::Green:
      return pros::MotorGears::green;
    case Gearset::Blue:
    default:
      return pros::MotorGears::blue;
  }
}

double gearsetRpm(Gearset gearset) {
  switch (gearset) {
    case Gearset::Red:
      return 100.0;
    case Gearset::Green:
      return 200.0;
    case Gearset::Blue:
      break;
  }
  return 600.0;
}

double voltsFor5_5WMotor(double volts_for_11w) {
  return volts_for_11w * kMotor5_5WFullVolts / 12.0;
}

double readingScaleFor5_5WMotor(Gearset gearset) {
  return gearsetRpm(gearset) / kMotor5_5WRpm;
}

pros::motor_brake_mode_e_t toProsBrakeMode(BrakeMode mode) {
  switch (mode) {
    case BrakeMode::Coast:
      return pros::E_MOTOR_BRAKE_COAST;
    case BrakeMode::Hold:
      return pros::E_MOTOR_BRAKE_HOLD;
    case BrakeMode::Brake:
    default:
      return pros::E_MOTOR_BRAKE_BRAKE;
  }
}

pros::controller_id_e_t toProsControllerId(ControllerId id) {
  return id == ControllerId::Partner ? pros::E_CONTROLLER_PARTNER
                                     : pros::E_CONTROLLER_MASTER;
}

pros::controller_analog_e_t toProsAnalogAxis(AnalogAxis axis) {
  switch (axis) {
    case AnalogAxis::LeftX:
      return pros::E_CONTROLLER_ANALOG_LEFT_X;
    case AnalogAxis::RightX:
      return pros::E_CONTROLLER_ANALOG_RIGHT_X;
    case AnalogAxis::RightY:
      return pros::E_CONTROLLER_ANALOG_RIGHT_Y;
    case AnalogAxis::LeftY:
    default:
      return pros::E_CONTROLLER_ANALOG_LEFT_Y;
  }
}

pros::controller_digital_e_t toProsDigitalButton(DigitalButton button) {
  switch (button) {
    case DigitalButton::A:
      return pros::E_CONTROLLER_DIGITAL_A;
    case DigitalButton::B:
      return pros::E_CONTROLLER_DIGITAL_B;
    case DigitalButton::X:
      return pros::E_CONTROLLER_DIGITAL_X;
    case DigitalButton::Y:
      return pros::E_CONTROLLER_DIGITAL_Y;
    case DigitalButton::Up:
      return pros::E_CONTROLLER_DIGITAL_UP;
    case DigitalButton::Down:
      return pros::E_CONTROLLER_DIGITAL_DOWN;
    case DigitalButton::Left:
      return pros::E_CONTROLLER_DIGITAL_LEFT;
    case DigitalButton::Right:
      return pros::E_CONTROLLER_DIGITAL_RIGHT;
    case DigitalButton::L1:
      return pros::E_CONTROLLER_DIGITAL_L1;
    case DigitalButton::L2:
      return pros::E_CONTROLLER_DIGITAL_L2;
    case DigitalButton::R2:
      return pros::E_CONTROLLER_DIGITAL_R2;
    case DigitalButton::R1:
    default:
      return pros::E_CONTROLLER_DIGITAL_R1;
  }
}

}  // namespace device
}  // namespace mclib
