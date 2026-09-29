// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <map>

/**
 * @brief The state behind the host stand-ins for pros::MotorGroup, pros::Imu
 *        and pros::Controller. See tests/support/host_devices.cpp.
 *
 * Everything is keyed by port number. Motor entries are keyed by the absolute
 * port, and hold what the motor itself does: a group built with port -3
 * commanded to +6 V shows -6 V on port 3, as a reversed motor on the robot
 * would.
 */
namespace mclib {
namespace test {
namespace host_devices {

/// @brief One motor as the stand-in drives it.
struct Motor {
  /// @brief Output in volts after the port's reversal. move() is scaled from
  ///        -127..127 to -12..12 V.
  double volts = 0.0;
  /// @brief The last brake mode set, as the pros::motor_brake_mode_e_t value.
  ///        -1 until one is set.
  int brake_mode = -1;
  /// @brief How many times brake() reached this motor.
  int brake_calls = 0;
  /// @brief Encoder reading in degrees. Only tare_position_all() writes it.
  double position_deg = 0.0;
};

/// @brief Every motor a stand-in MotorGroup has touched, by absolute port.
extern std::map<int, Motor> motors;
/// @brief IMU rotation in degrees, by port.
extern std::map<int, double> imu_rotation_deg;
/// @brief Controller stick values, -127..127, by pros::controller_analog_e_t value.
extern std::map<int, int> analog;

/// @brief Forget every device. Call at the start of each test case.
void reset();

}  // namespace host_devices
}  // namespace test
}  // namespace mclib
