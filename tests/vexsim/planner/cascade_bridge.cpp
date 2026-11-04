// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The path planner's calls into cascade_helpers.hpp, linked into the
// simulator library next to tests/vexsim/bridge.cpp. The lift is the real
// mclib PresetPositionMechanism; its sensor and motors are the simulated
// ones, through two callbacks, and a third opens and closes the simulated
// claw's valve. Rig::tick() runs every time the simulated clock moves 10 ms,
// as its task does on the robot.
#include "cascade_helpers.hpp"

#include "mclib/control/robot_state.hpp"

#include <cstdint>
#include <memory>
#include <vector>

extern "C" void (*sim_after_refresh_hook)(std::uint32_t millis);

namespace {

using ReadLiftDegrees = double (*)();
using WriteLiftVolts = void (*)(double volts);
using SetClawValve = void (*)(int closed);

constexpr int kActionValues = 4;

std::unique_ptr<cascade::Rig> rig;

void tick(std::uint32_t millis) {
  const auto pose = mclib::control::robotState().pose();
  rig->tick(pose.x, pose.y, millis);
}

}  // namespace

extern "C" void planner_cascade_init(ReadLiftDegrees read_lift, WriteLiftVolts write_lift,
                                     SetClawValve set_claw) {
  rig = std::make_unique<cascade::Rig>([read_lift] { return read_lift(); },
                                       [write_lift](double volts) { write_lift(volts); },
                                       [set_claw](bool closed) { set_claw(closed ? 1 : 0); });
  sim_after_refresh_hook = tick;
  tick(mclib::time::millis());
}

// Each action is 4 doubles: when (0 at the start, 1 after a distance in
// inches, 2 after a time in ms, 3 when the motion ends), the distance or
// time, the mechanism (0 the lift, 1 the claw), and the preset number.
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

// Moves the lift to a preset and waits: 0 on the time limit, 1 at the preset.
extern "C" int planner_cascade_move_lift(int preset, int timeout_ms) {
  const cascade::MoveResult result =
      rig->moveLift(static_cast<cascade::LiftPreset>(preset), static_cast<std::uint32_t>(timeout_ms));
  return static_cast<int>(result);
}

extern "C" void planner_cascade_set_claw(int preset) { rig->setClaw(static_cast<cascade::ClawPreset>(preset)); }
