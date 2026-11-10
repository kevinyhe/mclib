// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// CommandController::getTrigger(): the controller owns one trigger for each
// button, and a binding made through it fires on that button.

#include "mclib/command/commandController.h"
#include "mclib/command/instantCommand.h"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cstdio>

using mclib::device::DigitalButton;
namespace host = mclib::test::host_devices;

namespace {

int key(pros::controller_digital_e_t button) { return static_cast<int>(button); }

void oneTriggerForEachButton() {
  std::printf("-- getTrigger() returns the same trigger each time for a button\n");
  host::reset();
  CommandController primary;
  Trigger* const a = primary.getTrigger(DigitalButton::A);
  CHECK(a != nullptr);
  CHECK(a == primary.getTrigger(DigitalButton::A));
  CHECK(a != primary.getTrigger(DigitalButton::B));

  CommandController partner(mclib::device::ControllerId::Partner);
  CHECK(a != partner.getTrigger(DigitalButton::A));
}

void aBindingFiresOnItsButton() {
  std::printf("-- a binding made through getTrigger() fires on that button\n");
  host::reset();
  CommandController primary;
  int presses = 0;
  InstantCommand count([&presses] { ++presses; }, {});
  primary.getTrigger(DigitalButton::A)->onTrue(&count);

  EventLoop* const buttons = CommandScheduler::getTeleopEventLoop();
  host::button_down[key(pros::E_CONTROLLER_DIGITAL_B)] = true;
  buttons->poll();
  CHECK(!count.scheduled());

  host::button_down[key(pros::E_CONTROLLER_DIGITAL_A)] = true;
  buttons->poll();
  CHECK(count.scheduled());

  buttons->clear();
  CommandScheduler::forgetCommand(&count);
}

}  // namespace

int main() {
  oneTriggerForEachButton();
  aBindingFiresOnItsButton();
  return mclib::test::summary("command_controller");
}
