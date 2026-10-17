// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// device::Controller: sticks, held buttons, press and release edges, the
// screen (text, printf, clearing), rumble, battery and connection, against the
// pros::Controller stand-in in tests/support/host_devices.cpp.

#include "mclib/device/controller.hpp"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <string>

using mclib::device::AnalogAxis;
using mclib::device::Controller;
using mclib::device::ControllerId;
using mclib::device::DigitalButton;
namespace host = mclib::test::host_devices;

namespace {

int key(pros::controller_digital_e_t button) { return static_cast<int>(button); }

void sticks() {
  host::reset();
  Controller master;
  host::analog[static_cast<int>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = 127;
  host::analog[static_cast<int>(pros::E_CONTROLLER_ANALOG_RIGHT_X)] = -64;
  CHECK_EQ(master.getAnalog(AnalogAxis::LeftY), 127);
  CHECK_NEAR(master.analog(AnalogAxis::LeftY).raw(), 1.0, 1e-12);
  CHECK_NEAR(master.analog(AnalogAxis::RightX).raw(), -64.0 / 127.0, 1e-12);
  CHECK_NEAR(master.analog(AnalogAxis::LeftX).raw(), 0.0, 1e-12);
  // A disconnected controller reads PROS_ERR; the fraction is 0, not huge.
  host::analog[static_cast<int>(pros::E_CONTROLLER_ANALOG_LEFT_X)] = PROS_ERR;
  CHECK_NEAR(master.analog(AnalogAxis::LeftX).raw(), 0.0, 1e-12);
}

void buttons() {
  host::reset();
  Controller master;
  CHECK(!master.getDigital(DigitalButton::A));
  host::button_down[key(pros::E_CONTROLLER_DIGITAL_A)] = true;
  CHECK(master.getDigital(DigitalButton::A));
  CHECK(!master.getDigital(DigitalButton::B));

  // One press reads true once.
  host::new_presses[key(pros::E_CONTROLLER_DIGITAL_R1)] = 1;
  CHECK(master.getNewPress(DigitalButton::R1));
  CHECK(!master.getNewPress(DigitalButton::R1));
  // R1 and R2 are not swapped.
  host::new_presses[key(pros::E_CONTROLLER_DIGITAL_R2)] = 1;
  CHECK(!master.getNewPress(DigitalButton::R1));
  CHECK(master.getNewPress(DigitalButton::R2));

  host::new_releases[key(pros::E_CONTROLLER_DIGITAL_L2)] = 1;
  CHECK(!master.getNewRelease(DigitalButton::L1));
  CHECK(master.getNewRelease(DigitalButton::L2));
  CHECK(!master.getNewRelease(DigitalButton::L2));
}

void screen() {
  host::reset();
  Controller master;
  CHECK(master.setText(0, 0, "Auton: left"));
  CHECK(master.setText(1, 0, std::string("Ready")));
  CHECK(!master.setText(2, 0, static_cast<const char*>(nullptr)));
  CHECK(master.print(2, 0, "Batt %d%%", 87));
  // Cut at 15 characters, the width of the screen.
  CHECK(master.print(2, 0, "%s", "0123456789abcdefghij"));
  CHECK_EQ(host::controller_text.size(), 4u);
  CHECK(host::controller_text[0] == "Auton: left");
  CHECK(host::controller_text[1] == "Ready");
  CHECK(host::controller_text[2] == "Batt 87%");
  CHECK(host::controller_text[3] == "0123456789abcde");

  CHECK(master.clearLine(1));
  CHECK(master.clear());
  CHECK_EQ(host::cleared_lines.size(), 2u);
  CHECK_EQ(host::cleared_lines[0], 1);
  CHECK_EQ(host::cleared_lines[1], -1);

  CHECK(master.rumble(".-."));
  CHECK(!master.rumble(nullptr));
  CHECK_EQ(host::rumbles.size(), 1u);
  CHECK(host::rumbles[0] == ".-.");
}

void statusReads() {
  host::reset();
  Controller partner(ControllerId::Partner);
  CHECK(partner.id() == ControllerId::Partner);
  CHECK(partner.isConnected());
  host::controller_connected = false;
  CHECK(!partner.isConnected());

  host::controller_battery_capacity = 64;
  host::controller_battery_level = 3;
  CHECK(partner.batteryCapacity().has_value());
  CHECK_EQ(*partner.batteryCapacity(), 64);
  CHECK_EQ(*partner.batteryLevel(), 3);
  // PROS_ERR (unreadable) comes back as nothing.
  host::controller_battery_capacity = -1;
  host::controller_battery_level = -1;
  CHECK(!partner.batteryCapacity().has_value());
  CHECK(!partner.batteryLevel().has_value());
}

}  // namespace

int main() {
  sticks();
  buttons();
  screen();
  statusReads();
  return mclib::test::summary("controller");
}
