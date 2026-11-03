// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The path planner's calls into cascade_helpers.hpp, linked into the
// simulator library next to tests/vexsim/bridge.cpp. The lift, claw and
// Toggle arm are the real mclib PresetPositionMechanisms; their sensors and
// motors are the simulated ones, through two callbacks. Rig::tick() runs
// every time the simulated clock moves 10 ms, as its task does on the robot.
#include "cascade_helpers.hpp"

#include "mclib/control/robot_state.hpp"

#include <cstdint>
#include <memory>
#include <vector>

extern "C" void (*sim_after_refresh_hook)(std::uint32_t millis);

namespace {

using ReadDegrees = double (*)(int mechanism);
using WriteVolts = void (*)(int mechanism, double volts);

constexpr int kActionValues = 4;

std::unique_ptr<cascade::Rig> rig;

void tick(std::uint32_t millis) {
  const auto pose = mclib::control::robotState().pose();
  rig->tick(pose.x, pose.y, millis);
}

cascade::Rig::Degrees reader(ReadDegrees read, cascade::Mechanism mechanism) {
  return [read, mechanism] { return read(static_cast<int>(mechanism)); };
}

cascade::Rig::Volts writer(WriteVolts write, cascade::Mechanism mechanism) {
  return [write, mechanism](double volts) { write(static_cast<int>(mechanism), volts); };
}

}  // namespace

// The callbacks take the mechanism number: 0 the lift, 1 the claw, 2 the
// Toggle arm.
extern "C" void planner_cascade_init(ReadDegrees read, WriteVolts write) {
  using cascade::Mechanism;
  rig = std::make_unique<cascade::Rig>(reader(read, Mechanism::Lift), writer(write, Mechanism::Lift),
                                       reader(read, Mechanism::Claw), writer(write, Mechanism::Claw),
                                       reader(read, Mechanism::ToggleArm), writer(write, Mechanism::ToggleArm));
  sim_after_refresh_hook = tick;
  tick(mclib::time::millis());
}

// Each action is 4 doubles: when (0 at the start, 1 after a distance in
// inches, 2 after a time in ms, 3 when the motion ends), the distance or
// time, the mechanism number, and the preset number.
extern "C" void planner_cascade_begin_actions(const double* values, int count) {
  std::vector<cascade::Action> actions;
  for (int i = 0; i < count; ++i) {
    const double* v = values + kActionValues * i;
    actions.push_back({static_cast<cascade::Action::When>(static_cast<int>(v[0])), v[1],
                       static_cast<cascade::Mechanism>(static_cast<int>(v[2])), static_cast<int>(v[3])});
  }
  const auto pose = mclib::control::robotState().pose();
  rig->beginActions(std::move(actions), pose.x, pose.y, mclib::time::millis());
}

extern "C" int planner_cascade_finish_actions() { return rig->finishActions(); }

// Moves a mechanism to a preset and waits: 0 on the time limit, 1 at the
// preset, 2 when the claw stopped closing on something.
extern "C" int planner_cascade_move(int mechanism, int preset, int timeout_ms) {
  const cascade::MoveResult result =
      rig->move(static_cast<cascade::Mechanism>(mechanism), preset, static_cast<std::uint32_t>(timeout_ms));
  return static_cast<int>(result);
}
