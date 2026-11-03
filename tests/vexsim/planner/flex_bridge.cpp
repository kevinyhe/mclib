// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The path planner's calls into flex_helpers.hpp, linked into the simulator
// library next to tests/vexsim/bridge.cpp. The arm and claw are the real
// mclib PresetPositionMechanisms; their sensors and motors are the simulated
// ones, through two callbacks. Rig::tick() runs every time the simulated
// clock moves 10 ms, as its task does on the robot.
#include "flex_helpers.hpp"

#include "mclib/control/robot_state.hpp"

#include <cstdint>
#include <memory>
#include <vector>

extern "C" void (*sim_after_refresh_hook)(std::uint32_t millis);

namespace {

using ReadDegrees = double (*)(int motor);
using WriteVolts = void (*)(int motor, double volts);

constexpr int kArmMotor = 0;
constexpr int kClawMotor = 1;
constexpr int kActionValues = 4;

std::unique_ptr<flex::Rig> rig;

void tick(std::uint32_t millis) {
  const auto pose = mclib::control::robotState().pose();
  rig->tick(pose.x, pose.y, millis);
}

}  // namespace

extern "C" void planner_flex_init(ReadDegrees read, WriteVolts write) {
  rig = std::make_unique<flex::Rig>([read] { return read(kArmMotor); },
                                    [write](double volts) { write(kArmMotor, volts); },
                                    [read] { return read(kClawMotor); },
                                    [write](double volts) { write(kClawMotor, volts); });
  sim_after_refresh_hook = tick;
  tick(mclib::time::millis());
}

// Each action is 4 doubles: when (0 at the start, 1 after a distance in
// inches, 2 after a time in ms), the distance or time, 1 for the arm or 0 for
// the claw, and the preset number.
extern "C" void planner_flex_begin_actions(const double* values, int count) {
  std::vector<flex::Action> actions;
  for (int i = 0; i < count; ++i) {
    const double* v = values + kActionValues * i;
    actions.push_back({static_cast<flex::Action::When>(static_cast<int>(v[0])), v[1], v[2] != 0,
                       static_cast<int>(v[3])});
  }
  const auto pose = mclib::control::robotState().pose();
  rig->beginActions(std::move(actions), pose.x, pose.y, mclib::time::millis());
}

extern "C" int planner_flex_finish_actions() { return rig->finishActions(); }

// Moves the arm (1) or claw (0) to a preset and waits: 0 on the time limit,
// 1 at the preset, 2 when the claw stopped closing on something.
extern "C" int planner_flex_move(int arm, int preset, int timeout_ms) {
  const auto limit = static_cast<std::uint32_t>(timeout_ms);
  const flex::MoveResult result = arm ? rig->moveArm(static_cast<flex::ArmPreset>(preset), limit)
                                      : rig->moveClaw(static_cast<flex::ClawPreset>(preset), limit);
  return static_cast<int>(result);
}
