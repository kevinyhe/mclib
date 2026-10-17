// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

/**
 * @brief The state behind the host stand-ins for pros::Motor,
 *        pros::MotorGroup, pros::Imu and pros::Controller. See
 *        tests/support/host_devices.cpp.
 *
 * Every test links these (they are in HOST_TEST_SRC). Call reset() at the
 * start of each test case.
 */
namespace mclib {
namespace test {
namespace host_devices {

/// @brief One smart motor port. A pros::Motor and every motor in a
///        pros::MotorGroup share this record, keyed by the port number
///        without its sign.
struct Motor {
  /// @brief True when the last motor or group built on this port used the
  ///        negative port number.
  bool reversed = false;
  /// @brief Last command, in millivolts, as mclib sent it: before the port's
  ///        reversal. move() is scaled from -127..127 to -12000..12000.
  ///        brake() sets it to 0.
  double millivolts = 0.0;
  /// @brief How many move() and move_voltage() calls reached this port.
  int voltage_writes = 0;
  /// @brief True after brake(), false after the next move() or move_voltage().
  bool braked = false;
  /// @brief How many brake() calls reached this port.
  int brakes = 0;
  /// @brief The last brake mode set, as the pros::motor_brake_mode_e_t value.
  ///        -1 until one is set.
  int brake_mode = -1;

  // What the port reports. A test sets these; tare_position zeroes position.
  /// @brief Encoder reading, in degrees.
  double position_deg = 0.0;
  /// @brief Velocity reading, in RPM.
  double rpm = 0.0;
  /// @brief Current reading, in milliamps.
  std::int32_t current_ma = 0;

  /// @brief The last command in volts, before reversal.
  double volts() const { return millivolts / 1000.0; }
  /// @brief What the motor itself outputs: the command after reversal. A
  ///        motor on port -3 sent +6 V outputs -6 V.
  double outputVolts() const { return reversed ? -volts() : volts(); }
};

/// @brief Every motor port a stand-in Motor or MotorGroup has touched.
extern std::map<int, Motor> motors;

/// @brief IMU rotation in degrees, by port.
extern std::map<int, double> imu_rotation_deg;

/// @brief Controller stick values, -127..127, by pros::controller_analog_e_t.
extern std::map<int, int> analog;
/// @brief Controller buttons held down, by pros::controller_digital_e_t.
extern std::map<int, bool> button_down;
/// @brief Every Controller::set_text() string, in order.
extern std::vector<std::string> controller_text;
/// @brief Controller connected. True after reset().
extern bool controller_connected;
/// @brief Controller battery capacity and level. PROS_ERR when < 0.
extern int controller_battery_capacity;
extern int controller_battery_level;
/// @brief Presses and releases not yet read, by pros::controller_digital_e_t.
///        get_digital_new_press() / _release() take one each call.
extern std::map<int, int> new_presses;
extern std::map<int, int> new_releases;
/// @brief Every Controller::rumble() pattern, in order.
extern std::vector<std::string> rumbles;
/// @brief Lines cleared by clear_line(), in order; -1 for clear().
extern std::vector<int> cleared_lines;

/// @brief Forget every device. Call at the start of each test case.
void reset();

}  // namespace host_devices
}  // namespace test
}  // namespace mclib
