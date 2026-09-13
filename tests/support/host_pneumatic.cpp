// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-in for src/mclib/device/pneumatic.cpp, which cannot link on the
// host: it constructs pros::adi::DigitalOut and writes ports through libpros.
// This defines the same Pneumatic and PneumaticGroup methods over a map of
// pin levels a test can inspect. Listed in HOST_TEST_SRC together with the
// real pneumatic_subsystem.cpp, so every test binary links both.
#include "host_pneumatic.hpp"

#include "mclib/device/pneumatic.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace mclib {
namespace test {
namespace host_pneumatic {
std::map<char, bool> pin;
std::map<char, int> pin_writes;
void reset() {
  pin.clear();
  pin_writes.clear();
}
}  // namespace host_pneumatic
}  // namespace test
}  // namespace mclib

namespace mclib {
namespace device {

Pneumatic::Pneumatic(char adi_port, bool default_state, bool extended_state)
    : extender_port(0),
      adi_port(adi_port),
      default_state(default_state),
      extended_state(extended_state),
      current_state(default_state) {
  mclib::test::host_pneumatic::pin[adi_port] = (default_state == extended_state);
  ++mclib::test::host_pneumatic::pin_writes[adi_port];
}

Pneumatic::Pneumatic(int extender_port, char adi_port, bool default_state, bool extended_state)
    : extender_port(extender_port),
      adi_port(adi_port),
      default_state(default_state),
      extended_state(extended_state),
      current_state(default_state) {
  mclib::test::host_pneumatic::pin[adi_port] = (default_state == extended_state);
  ++mclib::test::host_pneumatic::pin_writes[adi_port];
}

void Pneumatic::set_value(bool value) {
  current_state = value;
  mclib::test::host_pneumatic::pin[adi_port] = (value == extended_state);
  ++mclib::test::host_pneumatic::pin_writes[adi_port];
}

void Pneumatic::extend() { set_value(true); }
void Pneumatic::retract() { set_value(false); }
void Pneumatic::toggle() { set_value(!current_state); }
bool Pneumatic::get_value() { return current_state; }

PneumaticGroup::PneumaticGroup(std::vector<std::shared_ptr<Pneumatic>> pneumatics)
    : pneumatics(std::move(pneumatics)) {}

void PneumaticGroup::set_value(bool value) {
  for (const auto& pneumatic : pneumatics) {
    if (pneumatic) {
      pneumatic->set_value(value);
    }
  }
}

void PneumaticGroup::extend() { set_value(true); }
void PneumaticGroup::retract() { set_value(false); }
void PneumaticGroup::toggle() { set_value(!get_value()); }

bool PneumaticGroup::get_value() {
  if (pneumatics.empty() || !pneumatics.front()) {
    return false;
  }
  return pneumatics.front()->get_value();
}

std::vector<bool> PneumaticGroup::get_all_values() {
  std::vector<bool> values;
  for (const auto& pneumatic : pneumatics) {
    values.push_back(pneumatic ? pneumatic->get_value() : false);
  }
  return values;
}

}  // namespace device
}  // namespace mclib
