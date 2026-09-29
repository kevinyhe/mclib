// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// PresetPositionMechanism: idle until commanded, preset changes, a new preset
// arriving mid-move, the rails of next()/previous(), presets missing from the
// table, raw targets and manual voltage putting the table on hold, stop(), the
// command factories through a real CommandScheduler, and disable.
//
// The position is a plain double the test moves by hand. Everything runs on a
// mclib::time::ScopedClock so the arrival dwell is exact.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/preset_position_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <type_traits>

using mclib::mechanism::PositionMechanismConfig;
using mclib::mechanism::PresetPositionMechanism;

namespace {

std::uint32_t g_fake_ms = 0;
constexpr std::uint32_t kTickMs = 10;

enum class Arm { Down, Load, Score, Unlisted };

using ArmMech = PresetPositionMechanism<Arm>;

static_assert(!std::is_copy_constructible_v<ArmMech>);
static_assert(!std::is_move_constructible_v<ArmMech>);

PositionMechanismConfig config() {
  PositionMechanismConfig c;
  c.kp = 0.5;
  c.ki = 0.0;
  c.kd = 0.0;
  c.max_voltage = 12.0;
  c.small_error = 2.0;
  c.big_error = 5.0;
  c.small_duration_ms = 50.0;
  c.big_duration_ms = 250.0;
  c.derivative_tolerance = 5.0;
  return c;
}

/// One mechanism, its position, and the last voltage it wrote.
struct Rig {
  double position = 0.0;
  double volts = -99.0;
  int writes = 0;
  ArmMech arm;

  explicit Rig(Arm initial = Arm::Down, double default_setpoint = 0.0)
      : arm(initial,
            {{Arm::Down, 0.0}, {Arm::Load, 45.0}, {Arm::Score, 130.0}},
            [this]() { return position; },
            [this](double v) {
              volts = v;
              ++writes;
            },
            config(),
            default_setpoint) {}

  void tick() {
    g_fake_ms += kTickMs;
    arm.runPeriodic();
  }

  /// Put the arm on its target and tick until it reports arrival. Returns
  /// the number of ticks, or -1.
  int settle(int max_ticks = 50) {
    position = arm.targetValue();
    for (int i = 1; i <= max_ticks; ++i) {
      tick();
      if (arm.atTarget()) return i;
    }
    return -1;
  }
};

void idle_until_commanded() {
  std::printf("-- a new mechanism does nothing until commanded\n");
  Rig rig;
  CHECK(!rig.arm.atTarget());
  CHECK(rig.arm.isManual());
  CHECK(!rig.arm.isBypassingPresets());
  rig.position = 20.0;
  for (int i = 0; i < 5; ++i) rig.tick();
  // Position 20 against preset Down (0): an engaged loop would push -10 V.
  CHECK_EQ(rig.volts, 0.0);
  CHECK_EQ(rig.arm.targetValue(), 0.0);
  CHECK(!rig.arm.atTarget());
}

void preset_changes() {
  std::printf("-- setPreset, holdPreset, and a new preset mid-move\n");
  Rig rig;
  ArmMech& arm = rig.arm;

  arm.setPreset(Arm::Load);
  // The target changes at the call, not on the next tick.
  CHECK_EQ(arm.targetValue(), 45.0);
  CHECK(arm.getPreset() == Arm::Load);
  CHECK(!arm.isManual());
  rig.tick();
  CHECK_EQ(rig.volts, 12.0);  // 0.5 * 45 = 22.5, clamped
  const int ticks = rig.settle();
  std::printf("   settled on Load after %d ticks\n", ticks);
  CHECK(ticks > 0);
  CHECK(arm.atTarget());

  // Settled on Load, then asked for Score: atTarget() must be false before
  // any tick, or a command finishing on atTarget() would end at once.
  arm.setPreset(Arm::Score);
  CHECK(!arm.atTarget());
  CHECK_EQ(arm.targetValue(), 130.0);

  // Mid-move: halfway to Score, switch back to Load.
  rig.position = 90.0;
  rig.tick();
  CHECK(!arm.atTarget());
  arm.setPreset(Arm::Load);
  CHECK_EQ(arm.targetValue(), 45.0);
  rig.tick();
  CHECK_EQ(rig.volts, -12.0);  // 0.5 * (45 - 90) = -22.5, clamped
  for (int i = 0; i < 5; ++i) rig.tick();
  CHECK_EQ(arm.targetValue(), 45.0);  // the table push keeps the new target
  CHECK(rig.settle() > 0);

  // setPreset() on the current preset restarts the loop; holdPreset() does not.
  arm.holdPreset(Arm::Load);
  CHECK(arm.atTarget());
  arm.setPreset(Arm::Load);
  CHECK(!arm.atTarget());
  CHECK(rig.settle() > 0);

  // setState and setPosition are routed through setPreset.
  arm.setState(Arm::Score);
  CHECK_EQ(arm.targetValue(), 130.0);
  arm.setPosition(Arm::Down);
  CHECK_EQ(arm.targetValue(), 0.0);
}

void rails_and_missing_presets() {
  std::printf("-- next/previous at the rails, cycle, missing presets\n");
  Rig rig(Arm::Down, -10.0);
  ArmMech& arm = rig.arm;

  // A step also engages a fresh mechanism.
  arm.next();
  CHECK(arm.getPreset() == Arm::Load);
  CHECK_EQ(arm.targetValue(), 45.0);
  arm.next();
  CHECK(rig.settle() > 0);
  CHECK(arm.getPreset() == Arm::Score);

  // At the last entry next() does nothing at all: no PID reset, atTarget()
  // stays true.
  arm.next();
  CHECK(arm.getPreset() == Arm::Score);
  CHECK(arm.atTarget());
  CHECK_EQ(arm.targetValue(), 130.0);

  arm.cycle();
  CHECK(arm.getPreset() == Arm::Down);
  CHECK_EQ(arm.targetValue(), 0.0);
  CHECK(rig.settle() > 0);
  arm.previous();
  CHECK(arm.getPreset() == Arm::Down);
  CHECK(arm.atTarget());

  // A preset missing from the table drives to the default setpoint.
  arm.setPreset(Arm::Unlisted);
  CHECK(arm.getPreset() == Arm::Unlisted);
  CHECK_EQ(arm.targetValue(), -10.0);
  CHECK_EQ(arm.presetSetpoint(), -10.0);
  rig.tick();
  CHECK_EQ(arm.targetValue(), -10.0);
  // Stepping from it resyncs to the first entry.
  arm.next();
  CHECK(arm.getPreset() == Arm::Down);
  CHECK_EQ(arm.targetValue(), 0.0);
}

void raw_target_and_manual() {
  std::printf("-- moveTo, setManualVoltage and stop put the table on hold\n");
  Rig rig;
  ArmMech& arm = rig.arm;

  arm.moveTo(50.0);
  CHECK(arm.isBypassingPresets());
  CHECK_EQ(arm.targetValue(), 50.0);
  // The reported preset snaps to the nearest entry.
  CHECK(arm.getPreset() == Arm::Load);
  CHECK_EQ(arm.presetSetpoint(), 45.0);
  for (int i = 0; i < 5; ++i) rig.tick();
  // The table does not pull the loop back to 45.
  CHECK_EQ(arm.targetValue(), 50.0);
  CHECK(rig.settle() > 0);

  // Ties go to the earlier entry.
  arm.moveTo(22.5);
  CHECK(arm.getPreset() == Arm::Down);

  // The next step hands control back and steps from the snapped preset.
  arm.moveTo(100.0);
  CHECK(arm.getPreset() == Arm::Score);
  arm.previous();
  CHECK(!arm.isBypassingPresets());
  CHECK(arm.getPreset() == Arm::Load);
  CHECK_EQ(arm.targetValue(), 45.0);

  // Manual voltage overrides the loop and the table.
  rig.position = 45.0;
  arm.setManualVoltage(3.0);
  CHECK(arm.isManual());
  CHECK(arm.isBypassingPresets());
  CHECK(!arm.atTarget());
  for (int i = 0; i < 3; ++i) rig.tick();
  CHECK_EQ(rig.volts, 3.0);
  arm.setManualVoltage(50.0);
  rig.tick();
  CHECK_EQ(rig.volts, 12.0);  // clamped to max_voltage

  // A step at a rail does not move the preset but still ends the override.
  arm.setPreset(Arm::Score);
  arm.setManualVoltage(-4.0);
  arm.next();
  CHECK(arm.getPreset() == Arm::Score);
  CHECK(!arm.isManual());
  CHECK(!arm.isBypassingPresets());
  CHECK_EQ(arm.targetValue(), 130.0);

  // Re-issuing the same preset also ends an override.
  arm.setManualVoltage(-4.0);
  arm.setPreset(Arm::Score);
  CHECK(!arm.isManual());
  CHECK_EQ(arm.targetValue(), 130.0);

  // stop() holds where the arm is now, and snaps the preset.
  rig.position = 70.0;
  arm.stop();
  CHECK_EQ(arm.targetValue(), 70.0);
  CHECK(arm.getPreset() == Arm::Load);
  CHECK(arm.isBypassingPresets());
  CHECK(!arm.isManual());
}

void empty_table() {
  std::printf("-- empty table\n");
  double position = 0.0;
  double volts = 0.0;
  ArmMech arm(Arm::Score, ArmMech::Table{}, [&]() { return position; },
              [&](double v) { volts = v; }, config(), 7.0);
  arm.moveTo(50.0);
  CHECK(arm.getPreset() == Arm::Score);  // left alone
  arm.next();
  arm.cycle();
  CHECK(arm.getPreset() == Arm::Score);
  arm.setPreset(Arm::Down);
  CHECK_EQ(arm.targetValue(), 7.0);
  arm.runPeriodic();
  CHECK_EQ(volts, 0.5 * 7.0);
}

void commands() {
  std::printf("-- commands, a new preset mid-command\n");
  mclib::test::setCompetitionStatus(0);
  Rig rig;
  ArmMech& arm = rig.arm;
  arm.setName("arm");
  arm.setDefaultCommand(arm.idleCommand());
  arm.registerSelf();

  auto run_ticks = [&](int n) {
    for (int i = 0; i < n; ++i) {
      g_fake_ms += kTickMs;
      CommandScheduler::run();
    }
  };

  // makePresetCommand finishes on arrival.
  std::unique_ptr<Command> to_load = arm.makePresetCommand(Arm::Load);
  CommandScheduler::schedule(to_load.get());
  CHECK_EQ(arm.targetValue(), 45.0);
  run_ticks(3);
  CHECK(CommandScheduler::scheduled(to_load.get()));
  rig.position = 45.0;
  run_ticks(10);
  CHECK(!CommandScheduler::scheduled(to_load.get()));
  CHECK(arm.atTarget());

  // Already settled on Load, a command to Load still has to start a fresh
  // move: it must not finish in the pass that scheduled it.
  CommandScheduler::schedule(to_load.get());
  CommandScheduler::run();
  CHECK(CommandScheduler::scheduled(to_load.get()));
  run_ticks(10);
  CHECK(!CommandScheduler::scheduled(to_load.get()));

  // Mid-move: Score is running, then Load is scheduled. The interrupted
  // Score command calls stop() in end(true), which must not win over Load.
  std::unique_ptr<Command> to_score = arm.makePresetCommand(Arm::Score);
  CommandScheduler::schedule(to_score.get());
  rig.position = 80.0;
  run_ticks(3);
  CommandScheduler::schedule(to_load.get());
  CHECK(!CommandScheduler::scheduled(to_score.get()));
  CHECK(arm.getPreset() == Arm::Load);
  CHECK_EQ(arm.targetValue(), 45.0);
  CHECK(!arm.isBypassingPresets());
  run_ticks(3);
  CHECK_EQ(arm.targetValue(), 45.0);
  rig.position = 45.0;
  run_ticks(10);
  CHECK(!CommandScheduler::scheduled(to_load.get()));

  // Timeout: finishes after 100 ms without arriving, and does not stop().
  g_fake_ms = 10000;
  std::unique_ptr<Command> timed = arm.makePresetCommand(Arm::Score, 100.0);
  CommandScheduler::schedule(timed.get());
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += kTickMs;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(timed.get())) {
      finished_on = i;
      break;
    }
  }
  std::printf("   100 ms timeout at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  CHECK_EQ(arm.targetValue(), 130.0);

  // makeMoveToCommand: raw target, interrupted by a preset command.
  std::unique_ptr<Command> raw = arm.makeMoveToCommand(60.0);
  CommandScheduler::schedule(raw.get());
  CHECK(arm.isBypassingPresets());
  CHECK_EQ(arm.targetValue(), 60.0);
  CommandScheduler::schedule(to_score.get());
  CHECK(!CommandScheduler::scheduled(raw.get()));
  CHECK_EQ(arm.targetValue(), 130.0);
  CHECK(!arm.isBypassingPresets());

  // makeStateCommand runs holdPreset every tick, so the loop can settle.
  CommandScheduler::cancel(to_score.get());
  std::unique_ptr<Command> hold_down = arm.makeStateCommand(Arm::Down);
  CommandScheduler::schedule(hold_down.get());
  rig.position = 0.0;
  run_ticks(10);
  CHECK(CommandScheduler::scheduled(hold_down.get()));
  CHECK(arm.atTarget());

  // One-shot steppers.
  CommandScheduler::cancel(hold_down.get());
  std::unique_ptr<Command> next = arm.makeNextCommand();
  CommandScheduler::schedule(next.get());
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(next.get()));
  CHECK(arm.getPreset() == Arm::Load);

  // Manual command applies its voltage every tick while it runs.
  std::unique_ptr<Command> manual = arm.makeManualCommand(5.0);
  CommandScheduler::schedule(manual.get());
  run_ticks(2);
  CHECK_EQ(rig.volts, 5.0);
  CommandScheduler::cancel(manual.get());

  CommandScheduler::unregisterSubsystem(&arm);
}

void disable() {
  std::printf("-- disable\n");
  mclib::test::setCompetitionStatus(0);
  Rig rig;
  ArmMech& arm = rig.arm;
  arm.setName("arm");
  arm.setDefaultCommand(arm.idleCommand());
  arm.registerSelf();

  std::unique_ptr<Command> to_score = arm.makePresetCommand(Arm::Score);
  CommandScheduler::schedule(to_score.get());
  rig.position = 60.0;
  for (int i = 0; i < 3; ++i) {
    g_fake_ms += kTickMs;
    CommandScheduler::run();
  }
  CHECK_EQ(rig.volts, 12.0);

  // Disable mid-move. The command is cancelled and onDisabled() writes 0 V
  // at once, without waiting for a tick.
  const int writes = rig.writes;
  mclib::test::setCompetitionStatus(1);
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(to_score.get()));
  CHECK(rig.writes > writes);
  CHECK_EQ(rig.volts, 0.0);
  CHECK(arm.isManual());
  CHECK(arm.isBypassingPresets());
  CHECK(!arm.atTarget());

  // Re-enabled: the arm stays limp. It does not resume the old move.
  mclib::test::setCompetitionStatus(0);
  for (int i = 0; i < 5; ++i) {
    g_fake_ms += kTickMs;
    CommandScheduler::run();
  }
  CHECK_EQ(rig.volts, 0.0);
  CHECK(!arm.atTarget());

  // holdPreset on the old preset takes control back, because the table was
  // on hold.
  arm.holdPreset(arm.getPreset());
  CHECK(!arm.isManual());
  CHECK(!arm.isBypassingPresets());

  // Calling onDisabled() directly twice is safe.
  arm.onDisabled();
  arm.onDisabled();
  CHECK_EQ(rig.volts, 0.0);

  CommandScheduler::unregisterSubsystem(&arm);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  idle_until_commanded();
  preset_changes();
  rails_and_missing_presets();
  raw_target_and_manual();
  empty_table();
  commands();
  disable();
  return mclib::test::summary("preset_position_mechanism");
}
