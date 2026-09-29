// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

/**
 * @brief State behind the host stand-ins for the brain screen, controller,
 *        clock and task calls AutonSelector makes. See
 *        tests/support/host_screen.cpp.
 *
 * Only auton_selector_test links this. It defines pros::millis(), which
 * motion_safety_test also defines, so it cannot go in HOST_TEST_SRC.
 */
namespace mclib {
namespace test {
namespace host_screen {

/// @brief One fill_rect() call and the pen it used.
struct FilledRect {
  std::uint32_t pen;
  int x0, y0, x1, y1;
};

/// @brief What the screen stand-in saw since the last reset().
extern int erase_count;
extern std::vector<FilledRect> filled;
extern std::vector<std::string> printed;

/// @brief What touch_status() returns: pressed or not, and where.
extern bool touch_down;
extern int touch_x;
extern int touch_y;

/// @brief Controller buttons held down, keyed by pros::controller_digital_e_t.
extern std::map<int, bool> button_down;
/// @brief Every set_text() call, in order.
extern std::vector<std::string> controller_text;

/// @brief What pros::millis() returns.
extern std::uint32_t now_ms;
/// @brief Called from Task::delay_until() after the clock advances.
extern std::function<void()> on_delay;
/// @brief How many tasks were started.
extern int tasks_started;

/// @brief Clear the drawing log only. Call before the draw you want to check.
void clearDrawing();
/// @brief Forget everything. Call at the start of each test case.
void reset();

}  // namespace host_screen
}  // namespace test
}  // namespace mclib
