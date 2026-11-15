// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "api.h"

namespace mclib {
namespace device {

/// @brief The cartridge in an 11 W motor. A 5.5 W motor has no cartridge.
enum class Gearset {
  Red,
  Green,
  Blue,
};

/// @brief Top speed of an 11 W motor's output shaft with @p gearset: 100,
///        200 or 600 rpm.
double gearsetRpm(Gearset gearset);

/// @brief Top speed of a 5.5 W motor's output shaft (276-4842), rpm. PROS
///        reads it like a green cartridge whatever gearset it is given.
inline constexpr double kMotor5_5WRpm = 200.0;

/**
 * @brief The voltage at which a 5.5 W motor reaches full speed. It applies no
 *        more than this, however high the command.
 *
 * Not measured on a robot. LemLib uses 7.2 V and vexide 8 V. 8 V is the safer
 * guess: if the motor tops out at 7.2 V it is flat out for the top 10% of
 * commands, where a 7.2 V guess on an 8 V motor would hold it 10% under the
 * 11 W motors it is geared to, dragging them at full speed.
 */
inline constexpr double kMotor5_5WFullVolts = 8.0;

/**
 * @brief The voltage for a 5.5 W motor that runs it at the same fraction of
 *        full speed as an 11 W motor sent @p volts_for_11w, -12 to 12 V.
 */
double voltsFor5_5WMotor(double volts_for_11w);

/**
 * @brief What to multiply a 5.5 W motor's position or speed by to put it on
 *        the shaft of an 11 W motor with @p gearset, when both are geared to
 *        the same top speed: 3 for blue, 1 for green, 0.5 for red.
 */
double readingScaleFor5_5WMotor(Gearset gearset);

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
