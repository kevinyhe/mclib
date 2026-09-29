// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstdint>
#include <map>

/**
 * @brief What the host stand-in pros::Motor and pros::MotorGroup were told to
 *        do. See tests/support/host_motor.cpp.
 */
namespace mclib {
namespace test {
namespace host_motor {

/// @brief Last commands sent to one smart port.
struct Port {
  /// Last move_voltage() argument, in millivolts.
  std::int32_t millivolts = 0;
  /// Number of move_voltage() calls.
  int voltage_writes = 0;
  /// Last move() argument, -127 to 127.
  std::int32_t power = 0;
  /// Number of move() calls.
  int power_writes = 0;
  /// Number of brake() calls.
  int brakes = 0;
};

/// @brief Commands per port, keyed by the port number without its sign. A
///        reversed motor (negative port) is recorded under the positive port,
///        with the value it was given, before any reversal.
extern std::map<int, Port> ports;

/// @brief Forget every port. Call at the start of each test case.
void reset();

}  // namespace host_motor
}  // namespace test
}  // namespace mclib
