// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstdint>
#include <map>
#include <vector>

/**
 * @brief The motor state the host stand-in pros::MotorGroup reads and writes.
 *        See tests/support/host_motor_group.cpp.
 */
namespace mclib {
namespace test {
namespace host_motor_group {

/// @brief One motor group as the stand-in sees it. Every motor in the group
///        reports the same reading.
struct Group {
  /// Ports as passed to the constructor, sign included.
  std::vector<std::int8_t> ports;
  /// Last voltage sent with move_voltage(), in volts. 0 after brake().
  double volts = 0.0;
  /// True after brake(), false after the next move_voltage().
  bool braked = false;
  /// How many times move_voltage() ran.
  int voltage_writes = 0;
  /// How many times brake() ran.
  int brakes = 0;
  /// Reading returned by get_actual_velocity_all(), per motor, in RPM.
  double rpm = 0.0;
  /// Reading returned by get_current_draw_all(), per motor, in milliamps.
  std::int32_t current_ma = 0;
};

/// @brief Every group built so far, keyed by its first port as passed in.
extern std::map<std::int8_t, Group> group;
/// @brief Forget every group. Call at the start of each test case.
void reset();

}  // namespace host_motor_group
}  // namespace test
}  // namespace mclib
