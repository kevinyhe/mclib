// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The Override robot's cascade lift, claw and Toggle arm, and the actions a
// motion step runs on them, shared by the simulator (cascade_bridge.cpp) and
// the C++ export.
#pragma once

#include "api.h"
#include "mclib/mechanism/preset_position_mechanism.hpp"
#include "mclib/sync.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>

// BEGIN EXPORTED HELPERS
namespace cascade {

// The claw's grip line, inches above the floor, from the lift motors'
// degrees. The motors turn an 18-tooth sprocket on 0.385 in chain, 6.93 in
// of chain a turn, into the first of three stages; the carriage moves three
// times as far.
constexpr double kLowestClawIn = 4.2;
constexpr double kChainPerTurnIn = 18 * 0.385;
constexpr int kLiftStages = 3;

inline double clawHeightIn(double lift_motor_degrees) {
  return kLowestClawIn + lift_motor_degrees / 360.0 * kChainPerTurnIn * kLiftStages;
}

// Lift presets are claw heights. The claw holds a Pin by its flange, so the
// flange is at that height. The Over presets hang the Pin 0.15 in above that
// Goal's top; the aligner under the claw catches the Goal there.
enum class LiftPreset { Down, PinInCup, OverAllianceGoal, OverNeutralGoal, OverCenterGoal, Top };
// Claw motor degrees: 0 fully open (jaws 4 in apart), 180 fully closed.
enum class ClawPreset { Open, Closed };
// Toggle arm motor degrees, geared 3:1: 0 folded forward, 540 swung straight
// back onto its stop with the pad 0.75 in under a Toggle's axle, 464.5 with it
// 0.95 in over the axle.
enum class ToggleArmPreset { Stowed, OverTheAxle, UnderTheAxle };
enum class Mechanism { Lift, Claw, ToggleArm };

using Lift = mclib::mechanism::PresetPositionMechanism<LiftPreset>;
using Claw = mclib::mechanism::PresetPositionMechanism<ClawPreset>;
using ToggleArm = mclib::mechanism::PresetPositionMechanism<ToggleArmPreset>;

inline Lift::Table liftPresets() {
  return {{LiftPreset::Down, kLowestClawIn},     {LiftPreset::PinInCup, 6.8},
          {LiftPreset::OverAllianceGoal, 6.65},  {LiftPreset::OverNeutralGoal, 9.17},
          {LiftPreset::OverCenterGoal, 12.17},   {LiftPreset::Top, 40.2}};
}

inline Claw::Table clawPresets() {
  return {{ClawPreset::Open, 0.0}, {ClawPreset::Closed, 180.0}};
}

inline ToggleArm::Table toggleArmPresets() {
  return {{ToggleArmPreset::Stowed, 0.0},
          {ToggleArmPreset::OverTheAxle, 464.5},
          {ToggleArmPreset::UnderTheAxle, 540.0}};
}

// What the lift motors need to hold the stages up against the rubber bands.
constexpr double kLiftHoldVolts = 1.4;

inline mclib::mechanism::PositionMechanismConfig liftConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 6.0;
  config.kd = 20.0;
  config.max_voltage = 12.0;
  config.small_error = 0.15;
  config.big_error = 0.5;
  config.derivative_tolerance = 0.05;
  return config;
}

// Closed is past where the jaws stop on a Pin, so the loop keeps squeezing.
inline mclib::mechanism::PositionMechanismConfig clawConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 0.1;
  config.max_voltage = 12.0;
  config.small_error = 5.0;
  config.big_error = 15.0;
  config.derivative_tolerance = 2.0;
  return config;
}

inline mclib::mechanism::PositionMechanismConfig toggleArmConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 0.12;
  config.kd = 0.6;
  config.max_voltage = 12.0;
  config.small_error = 4.0;
  config.big_error = 12.0;
  config.derivative_tolerance = 3.0;
  return config;
}

// Something to do while a motion runs: move a mechanism to a preset when the
// step starts, once the robot is `value` inches from where the step started,
// `value` milliseconds after it started, or when the motion ends.
struct Action {
  enum class When { Start, AfterDistance, AfterTime, End };
  When when;
  double value;
  Mechanism mechanism;
  int preset;
};

enum class MoveResult { TimedOut, Reached, Gripped };

// The lift, the claw, the Toggle arm and the actions of the motion that is
// running. tick() runs them and must be called every 10 ms from its own
// task; the other calls come from the autonomous task.
class Rig {
 public:
  using Degrees = std::function<double()>;
  using Volts = std::function<void(double)>;

  Rig(Degrees lift_motor_degrees, Volts lift_volts, Degrees claw_degrees, Volts claw_volts,
      Degrees toggle_arm_degrees, Volts toggle_arm_volts)
      : m_lift(LiftPreset::Down, liftPresets(),
               [lift_motor_degrees] { return clawHeightIn(lift_motor_degrees()); },
               [lift_volts](double volts) { lift_volts(volts + kLiftHoldVolts); }, liftConfig()),
        m_claw(ClawPreset::Closed, clawPresets(), std::move(claw_degrees), std::move(claw_volts),
               clawConfig()),
        m_toggle_arm(ToggleArmPreset::Stowed, toggleArmPresets(), std::move(toggle_arm_degrees),
                     std::move(toggle_arm_volts), toggleArmConfig()) {
    m_lift.setPreset(LiftPreset::Down);
    m_claw.setPreset(ClawPreset::Closed);
    m_toggle_arm.setPreset(ToggleArmPreset::Stowed);
  }

  void tick(double x_in, double y_in, std::uint32_t now_ms) {
    mclib::sync::LockGuard lock(m_mutex);
    m_lift.periodic();
    m_claw.periodic();
    m_toggle_arm.periodic();
    const double moved = std::hypot(x_in - m_start_x, y_in - m_start_y);
    const double elapsed = static_cast<double>(now_ms - m_start_ms);
    for (std::size_t i = 0; i < m_actions.size(); ++i) {
      if (m_fired[i] || m_actions[i].when == Action::When::End) continue;
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

  // Call right after the motion ends. Runs the End actions and whatever has
  // not fired yet, and returns how many of those were not End actions.
  int finishActions() {
    mclib::sync::LockGuard lock(m_mutex);
    int late = 0;
    for (std::size_t i = 0; i < m_actions.size(); ++i) {
      if (m_fired[i]) continue;
      fire(i);
      if (m_actions[i].when != Action::When::End) ++late;
    }
    m_actions.clear();
    m_fired.clear();
    return late;
  }

  MoveResult moveLift(LiftPreset preset, std::uint32_t timeout_ms) {
    return move(Mechanism::Lift, static_cast<int>(preset), timeout_ms);
  }

  // Closing stops early, as Gripped, once the jaws have not moved for 100 ms.
  MoveResult moveClaw(ClawPreset preset, std::uint32_t timeout_ms) {
    return move(Mechanism::Claw, static_cast<int>(preset), timeout_ms);
  }

  MoveResult moveToggleArm(ToggleArmPreset preset, std::uint32_t timeout_ms) {
    return move(Mechanism::ToggleArm, static_cast<int>(preset), timeout_ms);
  }

  MoveResult move(Mechanism mechanism, int preset, std::uint32_t timeout_ms) {
    const bool closing = mechanism == Mechanism::Claw && preset == static_cast<int>(ClawPreset::Closed);
    std::uint32_t still_ms = 0;
    double last = 0.0;
    {
      mclib::sync::LockGuard lock(m_mutex);
      setPreset(mechanism, preset);
      last = position(mechanism);
    }
    const std::uint32_t start = mclib::time::millis();
    while (mclib::time::millis() - start < timeout_ms) {
      pros::delay(10);
      mclib::sync::LockGuard lock(m_mutex);
      if (atTarget(mechanism)) return MoveResult::Reached;
      const double now = position(mechanism);
      still_ms = std::abs(now - last) < 0.5 ? still_ms + 10 : 0;
      last = now;
      if (closing && still_ms >= 100) return MoveResult::Gripped;
    }
    return MoveResult::TimedOut;
  }

  double clawHeight() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_lift.positionValue();
  }

  double clawDegrees() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_claw.positionValue();
  }

  double toggleArmDegrees() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_toggle_arm.positionValue();
  }

 private:
  void fire(std::size_t i) {
    const Action& action = m_actions[i];
    setPreset(action.mechanism, action.preset);
    m_fired[i] = true;
  }

  void setPreset(Mechanism mechanism, int preset) {
    switch (mechanism) {
      case Mechanism::Lift:
        m_lift.setPreset(static_cast<LiftPreset>(preset));
        return;
      case Mechanism::Claw:
        m_claw.setPreset(static_cast<ClawPreset>(preset));
        return;
      case Mechanism::ToggleArm:
        m_toggle_arm.setPreset(static_cast<ToggleArmPreset>(preset));
        return;
    }
  }

  double position(Mechanism mechanism) {
    switch (mechanism) {
      case Mechanism::Lift:
        return m_lift.positionValue();
      case Mechanism::Claw:
        return m_claw.positionValue();
      case Mechanism::ToggleArm:
        return m_toggle_arm.positionValue();
    }
    std::abort();
  }

  bool atTarget(Mechanism mechanism) {
    switch (mechanism) {
      case Mechanism::Lift:
        return m_lift.atTarget();
      case Mechanism::Claw:
        return m_claw.atTarget();
      case Mechanism::ToggleArm:
        return m_toggle_arm.atTarget();
    }
    std::abort();
  }

  mclib::sync::Mutex m_mutex;
  Lift m_lift;
  Claw m_claw;
  ToggleArm m_toggle_arm;
  std::vector<Action> m_actions;
  std::vector<bool> m_fired;
  double m_start_x = 0.0;
  double m_start_y = 0.0;
  std::uint32_t m_start_ms = 0;
};

}  // namespace cascade
// END EXPORTED HELPERS
