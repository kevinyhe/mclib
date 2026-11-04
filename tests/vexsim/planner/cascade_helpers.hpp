// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The Override robot's cascade lift and pneumatic claw, and the actions a
// motion step runs on them, shared by the simulator (cascade_bridge.cpp) and
// the C++ export.
#pragma once

#include "api.h"
#include "mclib/mechanism/preset_position_mechanism.hpp"
#include "mclib/sync.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

// BEGIN EXPORTED HELPERS
namespace cascade {

// The height of the claw's middle above the floor, inches, from the lift
// motors' degrees. The motors turn an 18-tooth sprocket on 0.385 in chain,
// 6.93 in of chain a turn, into the first of three stages; the carriage
// moves three times as far. Fully down, the claw's middle is 0.28 in under
// the top rim of a Cup standing on the floor.
constexpr double kLowestClawIn = 6.2;
constexpr double kChainPerTurnIn = 18 * 0.385;
constexpr int kLiftStages = 3;

inline double clawHeightIn(double lift_motor_degrees) {
  return kLowestClawIn + lift_motor_degrees / 360.0 * kChainPerTurnIn * kLiftStages;
}

// Lift presets are heights of the claw's middle. The claw holds a Cup by its
// top rim with a Pin standing in it. On sets the held Cup's rim on the Pin
// in that Goal; Over carries the Cup 0.25 in over that Pin's tip.
enum class LiftPreset {
  Down,
  OverNeutralGoal,
  OnNeutralGoal,
  OverStackOnNeutralGoal,
  OnStackOnNeutralGoal,
  OverCenterGoal,
  OnCenterGoal,
  Top
};
enum class ClawPreset { Open, Closed };
enum class Mechanism { Lift, Claw };

using Lift = mclib::mechanism::PresetPositionMechanism<LiftPreset>;

inline Lift::Table liftPresets() {
  return {{LiftPreset::Down, kLowestClawIn},          {LiftPreset::OverNeutralGoal, 15.79},
          {LiftPreset::OnNeutralGoal, 12.61},         {LiftPreset::OverStackOnNeutralGoal, 22.91},
          {LiftPreset::OnStackOnNeutralGoal, 19.73},  {LiftPreset::OverCenterGoal, 18.79},
          {LiftPreset::OnCenterGoal, 15.61},          {LiftPreset::Top, 33.2}};
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

// The claw's piston strokes in about 5 ms. A claw step waits one 10 ms tick.
constexpr std::uint32_t kClawStrokeMs = 10;

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

enum class MoveResult { TimedOut, Reached };

// The lift, the claw and the actions of the motion that is running. tick()
// runs them and must be called every 10 ms from its own task; the other
// calls come from the autonomous task. The claw starts open.
class Rig {
 public:
  using Degrees = std::function<double()>;
  using Volts = std::function<void(double)>;
  using Valve = std::function<void(bool closed)>;

  Rig(Degrees lift_motor_degrees, Volts lift_volts, Valve claw_valve)
      : m_lift(LiftPreset::Down, liftPresets(),
               [lift_motor_degrees] { return clawHeightIn(lift_motor_degrees()); },
               [lift_volts](double volts) { lift_volts(volts + kLiftHoldVolts); }, liftConfig()),
        m_claw_valve(std::move(claw_valve)) {
    m_lift.setPreset(LiftPreset::Down);
    setClawValve(ClawPreset::Open);
  }

  void tick(double x_in, double y_in, std::uint32_t now_ms) {
    mclib::sync::LockGuard lock(m_mutex);
    m_lift.periodic();
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
    {
      mclib::sync::LockGuard lock(m_mutex);
      m_lift.setPreset(preset);
    }
    const std::uint32_t start = mclib::time::millis();
    while (mclib::time::millis() - start < timeout_ms) {
      pros::delay(10);
      mclib::sync::LockGuard lock(m_mutex);
      if (m_lift.atTarget()) return MoveResult::Reached;
    }
    return MoveResult::TimedOut;
  }

  // Opens or closes the claw and waits for the piston to finish its stroke.
  void setClaw(ClawPreset preset) {
    {
      mclib::sync::LockGuard lock(m_mutex);
      setClawValve(preset);
    }
    pros::delay(kClawStrokeMs);
  }

  double clawHeight() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_lift.positionValue();
  }

  bool clawClosed() {
    mclib::sync::LockGuard lock(m_mutex);
    return m_claw_closed;
  }

 private:
  void fire(std::size_t i) {
    const Action& action = m_actions[i];
    switch (action.mechanism) {
      case Mechanism::Lift:
        m_lift.setPreset(static_cast<LiftPreset>(action.preset));
        break;
      case Mechanism::Claw:
        setClawValve(static_cast<ClawPreset>(action.preset));
        break;
    }
    m_fired[i] = true;
  }

  void setClawValve(ClawPreset preset) {
    m_claw_closed = preset == ClawPreset::Closed;
    m_claw_valve(m_claw_closed);
  }

  mclib::sync::Mutex m_mutex;
  Lift m_lift;
  Valve m_claw_valve;
  bool m_claw_closed = false;
  std::vector<Action> m_actions;
  std::vector<bool> m_fired;
  double m_start_x = 0.0;
  double m_start_y = 0.0;
  std::uint32_t m_start_ms = 0;
};

}  // namespace cascade
// END EXPORTED HELPERS
