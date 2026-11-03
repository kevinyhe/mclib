// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The Override robot's lift and claw, and the actions a motion step runs on
// them, shared by the simulator (flex_bridge.cpp) and the C++ export.
#pragma once

#include "api.h"
#include "mclib/mechanism/preset_position_mechanism.hpp"
#include "mclib/sync.hpp"

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

// BEGIN EXPORTED HELPERS
namespace flex {

// Arm motor degrees: 0 with the lift down, 2000 at the top, the way VEX VR's
// Flex counts them. The Above presets hold a Pin gripped low on its bottom
// half just clear of that Goal's top. IntoAllianceGoal puts its tip in.
// UnderToggleAxle and OverToggleAxle put the claw where driving it into a
// Toggle turns it toward the Quadrant's colour or toward the other one.
enum class ArmPreset {
  Down,
  IntoAllianceGoal,
  AboveAllianceGoal,
  AboveNeutralGoal,
  AboveCenterGoal,
  UnderToggleAxle,
  OverToggleAxle,
  Top
};
// Claw motor degrees: 0 fully open, 467 fully closed.
enum class ClawPreset { Open, Closed };

using Arm = mclib::mechanism::PresetPositionMechanism<ArmPreset>;
using Claw = mclib::mechanism::PresetPositionMechanism<ClawPreset>;

inline Arm::Table armPresets() {
  return {{ArmPreset::Down, 0.0},           {ArmPreset::IntoAllianceGoal, 290.0},
          {ArmPreset::AboveAllianceGoal, 520.0}, {ArmPreset::AboveNeutralGoal, 820.0},
          {ArmPreset::AboveCenterGoal, 1200.0},  {ArmPreset::UnderToggleAxle, 1350.0},
          {ArmPreset::OverToggleAxle, 1625.0},   {ArmPreset::Top, 2000.0}};
}

inline Claw::Table clawPresets() {
  return {{ClawPreset::Open, 60.0}, {ClawPreset::Closed, 467.0}};
}

inline mclib::mechanism::PositionMechanismConfig armConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 0.06;
  config.kd = 0.002;
  config.max_voltage = 12.0;
  config.small_error = 15.0;
  config.big_error = 40.0;
  return config;
}

// Closed is past where the jaws stop on a Pin, so the loop keeps squeezing.
inline mclib::mechanism::PositionMechanismConfig clawConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 0.05;
  config.max_voltage = 8.0;
  config.small_error = 10.0;
  config.big_error = 30.0;
  return config;
}

// Something to do while a motion runs: move the arm or the claw to a preset
// when the step starts, once the robot is `value` inches from where the step
// started, or `value` milliseconds after it started.
struct Action {
  enum class When { Start, AfterDistance, AfterTime };
  When when;
  double value;
  bool arm;
  int preset;
};

enum class MoveResult { TimedOut, Reached, Gripped };

// The lift, the claw and the actions of the motion that is running.
// tick() runs them and must be called every 10 ms from its own task; the
// other calls come from the autonomous task.
class Rig {
 public:
  Rig(Arm::PositionSource arm_degrees, Arm::VoltageSink arm_volts, Claw::PositionSource claw_degrees,
      Claw::VoltageSink claw_volts)
      : m_arm(ArmPreset::Down, armPresets(), std::move(arm_degrees), std::move(arm_volts), armConfig()),
        m_claw(ClawPreset::Closed, clawPresets(), std::move(claw_degrees), std::move(claw_volts),
               clawConfig()) {
    m_arm.setPreset(ArmPreset::Down);
    m_claw.setPreset(ClawPreset::Closed);
  }

  void tick(double x_in, double y_in, std::uint32_t now_ms) {
    mclib::sync::LockGuard lock(m_mutex);
    m_arm.periodic();
    m_claw.periodic();
    const double moved = std::hypot(x_in - m_start_x, y_in - m_start_y);
    const double elapsed = static_cast<double>(now_ms - m_start_ms);
    for (std::size_t i = 0; i < m_actions.size(); ++i) {
      if (m_fired[i]) continue;
      const Action& action = m_actions[i];
      const bool due = action.when == Action::When::Start ||
                       (action.when == Action::When::AfterDistance && moved >= action.value) ||
                       (action.when == Action::When::AfterTime && elapsed >= action.value);
      if (due) fire(i);
    }
  }

  // Call right before the motion starts.
  void beginActions(std::vector<Action> actions, double x_in, double y_in, std::uint32_t now_ms) {
    mclib::sync::LockGuard lock(m_mutex);
    m_actions = std::move(actions);
    m_fired.assign(m_actions.size(), false);
    m_start_x = x_in;
    m_start_y = y_in;
    m_start_ms = now_ms;
  }

  // Call right after the motion ends. Runs whatever has not fired yet and
  // returns how many that was.
  int finishActions() {
    mclib::sync::LockGuard lock(m_mutex);
    int late = 0;
    for (std::size_t i = 0; i < m_actions.size(); ++i) {
      if (m_fired[i]) continue;
      fire(i);
      ++late;
    }
    m_actions.clear();
    m_fired.clear();
    return late;
  }

  MoveResult moveArm(ArmPreset preset, std::uint32_t timeout_ms) {
    return move(true, static_cast<int>(preset), timeout_ms);
  }

  // Closing stops early, as Gripped, once the jaws have not moved for 100 ms.
  MoveResult moveClaw(ClawPreset preset, std::uint32_t timeout_ms) {
    return move(false, static_cast<int>(preset), timeout_ms);
  }

  double armDegrees() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_arm.positionValue();
  }

  double clawDegrees() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_claw.positionValue();
  }

 private:
  void fire(std::size_t i) {
    const Action& action = m_actions[i];
    setPreset(action.arm, action.preset);
    m_fired[i] = true;
  }

  void setPreset(bool arm, int preset) {
    if (arm)
      m_arm.setPreset(static_cast<ArmPreset>(preset));
    else
      m_claw.setPreset(static_cast<ClawPreset>(preset));
  }

  MoveResult move(bool arm, int preset, std::uint32_t timeout_ms) {
    const bool closing = !arm && preset == static_cast<int>(ClawPreset::Closed);
    std::uint32_t still_ms = 0;
    double last = 0.0;
    {
      mclib::sync::LockGuard lock(m_mutex);
      setPreset(arm, preset);
      last = arm ? m_arm.positionValue() : m_claw.positionValue();
    }
    const std::uint32_t start = mclib::time::millis();
    while (mclib::time::millis() - start < timeout_ms) {
      pros::delay(10);
      mclib::sync::LockGuard lock(m_mutex);
      if (arm ? m_arm.atTarget() : m_claw.atTarget()) return MoveResult::Reached;
      const double now = arm ? m_arm.positionValue() : m_claw.positionValue();
      still_ms = std::abs(now - last) < 0.5 ? still_ms + 10 : 0;
      last = now;
      if (closing && still_ms >= 100) return MoveResult::Gripped;
    }
    return MoveResult::TimedOut;
  }

  mclib::sync::Mutex m_mutex;
  Arm m_arm;
  Claw m_claw;
  std::vector<Action> m_actions;
  std::vector<bool> m_fired;
  double m_start_x = 0.0;
  double m_start_y = 0.0;
  std::uint32_t m_start_ms = 0;
};

}  // namespace flex
// END EXPORTED HELPERS
