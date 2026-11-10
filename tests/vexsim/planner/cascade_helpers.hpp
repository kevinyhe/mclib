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
#include "mclib/control/chassis_io.hpp"
#include "mclib/mechanism/preset_position_mechanism.hpp"
#include "mclib/sync.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

// BEGIN EXPORTED HELPERS
namespace cascade {

// The height of the claw's middle above the floor, inches, from the lift
// motors' degrees. Three motors turn a 1.0 in drum through 48:72 gears, so
// the drum winds in pi / 1.5 = 2.09 in of string a motor turn. The string
// lifts the first of five stages and the carriage moves five times as far.
// Fully down, the claw's middle is level with the ridge of a stack standing
// on the floor, where the Pin's flange sits on the Cup's rim. Once the
// stages hang on the strings, the strings stretch and the claw sits
// kStringStretchIn under where the wound string alone would put it.
constexpr double kLowestClawIn = 6.62;
constexpr double kDrumDiameterIn = 1.0;
constexpr double kMotorTurnsPerDrumTurn = 1.5;
constexpr int kLiftStages = 5;
constexpr double kStringStretchIn = 0.25;

inline double clawHeightIn(double lift_motor_degrees) {
  const double string_in = lift_motor_degrees / 360.0 * M_PI * kDrumDiameterIn / kMotorTurnsPerDrumTurn;
  return std::max(kLowestClawIn, kLowestClawIn + string_in * kLiftStages - kStringStretchIn);
}

// Lift presets are heights of the claw's middle. Each jaw holds a stack at
// two heights: the Cup below its rim and the Pin above its flange. On sets
// the held Cup's rim on the Pin in that Goal; Over carries the Cup 0.25 in
// over that Pin's tip. OffTheFloor lifts a stack taken from a Loader 0.5 in,
// so its Cup does not drag on the foam as the robot backs out, and keeps the
// claw's top under a raised Loader's tube.
enum class LiftPreset {
  Down,
  OffTheFloor,
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
  return {{LiftPreset::Down, kLowestClawIn},         {LiftPreset::OffTheFloor, 7.12},
          {LiftPreset::OverNeutralGoal, 15.97},       {LiftPreset::OnNeutralGoal, 12.58},
          {LiftPreset::OverStackOnNeutralGoal, 22.66}, {LiftPreset::OnStackOnNeutralGoal, 19.27},
          {LiftPreset::OverCenterGoal, 18.97},        {LiftPreset::OnCenterGoal, 15.58},
          {LiftPreset::Top, 46.72}};
}

// What the lift motors need to hold the stages up with the rubber bands'
// help: 0.6 V at the bottom, 0.9 V at 16 in and at the top.
constexpr double kLiftHoldVolts = 0.9;

inline mclib::mechanism::PositionMechanismConfig liftConfig() {
  mclib::mechanism::PositionMechanismConfig config;
  config.kp = 6.0;
  config.kd = 40.0;
  config.max_voltage = 12.0;
  config.small_error = 0.15;
  config.big_error = 0.5;
  config.derivative_tolerance = 0.05;
  return config;
}

// The claw's piston closes in 130 ms and opens in 160 ms, as fast as air
// gets through the valve. A claw step waits for the slower one.
constexpr std::uint32_t kClawStrokeMs = 170;

// The roller's motor turns its axle through 12:36 chain.
constexpr double kRollerMotorTurnsPerTurn = 3.0;

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

// The lift, the claw, the Toggle roller and the actions of the motion that
// is running. tick() runs them and must be called every 10 ms from its own
// task; the other calls come from the autonomous task. The claw starts as
// `claw_start` says: closed on a preload Pin, or open.
class Rig {
 public:
  using Degrees = std::function<double()>;
  using Volts = std::function<void(double)>;
  using Valve = std::function<void(bool closed)>;

  Rig(Degrees lift_motor_degrees, Volts lift_volts, Valve claw_valve, Degrees roller_motor_degrees,
      Volts roller_volts, ClawPreset claw_start)
      : m_lift(LiftPreset::Down, liftPresets(),
               [lift_motor_degrees] { return clawHeightIn(lift_motor_degrees()); },
               [lift_volts](double volts) { lift_volts(volts + kLiftHoldVolts); }, liftConfig()),
        m_claw_valve(std::move(claw_valve)),
        m_roller_motor_degrees(std::move(roller_motor_degrees)),
        m_roller_volts(std::move(roller_volts)) {
    m_lift.setPreset(LiftPreset::Down);
    setClawValve(claw_start);
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

  // Turns the roller `roller_degrees` at `volts`, then stops it. Positive
  // volts take the flaps behind the axle up, which turns a Toggle's top
  // toward the wall. Each flap that turns a Toggle pushes the robot off the
  // wall, so the drive pushes back at `press_volts` the whole time.
  MoveResult spinRoller(mclib::control::DriveHardware& drive, double roller_degrees, double volts,
                        double press_volts, std::uint32_t timeout_ms) {
    const double start = rollerDegrees();
    drive.setDriveVoltage(-press_volts, -press_volts);
    m_roller_volts(volts);
    const std::uint32_t started = mclib::time::millis();
    MoveResult result = MoveResult::TimedOut;
    while (mclib::time::millis() - started < timeout_ms) {
      pros::delay(10);
      if (std::abs(rollerDegrees() - start) >= roller_degrees) {
        result = MoveResult::Reached;
        break;
      }
    }
    m_roller_volts(0.0);
    drive.setDriveVoltage(0.0, 0.0);
    return result;
  }

  double rollerDegrees() { return m_roller_motor_degrees() / kRollerMotorTurnsPerTurn; }

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
  Degrees m_roller_motor_degrees;
  Volts m_roller_volts;
  bool m_claw_closed = false;
  std::vector<Action> m_actions;
  std::vector<bool> m_fired;
  double m_start_x = 0.0;
  double m_start_y = 0.0;
  std::uint32_t m_start_ms = 0;
};

}  // namespace cascade
// END EXPORTED HELPERS
