// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// PneumaticSubsystem: logical state versus solenoid polarity, what the
// constructor drives, group() writes, and command termination through a real
// CommandScheduler on a mclib::time::ScopedClock.
//
// device::Pneumatic owns a pros::adi::DigitalOut, so src/mclib/device/
// pneumatic.cpp cannot link on the host. This file supplies its own
// definitions of device::Pneumatic and device::PneumaticGroup instead, the
// same way tests/support/host_pros.cpp stands in for pros::competition. The
// stand-in keeps the device layer's contract - `raw = (logical ==
// extended_state)`, get_value() returns the logical value - and records the
// pin level per port so the test can see what the solenoid would have done.
// What is under test is the subsystem: the device mapping itself is
// replicated here, not verified.

#include "mclib/command/commandScheduler.h"
#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/pneumatic_subsystem.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "support/host_pneumatic.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <utility>
#include <vector>

using mclib::device::Pneumatic;
using mclib::mechanism::PneumaticSubsystem;

namespace {

std::uint32_t g_fake_ms = 0;

auto& g_pin = mclib::test::host_pneumatic::pin;
auto& g_pin_writes = mclib::test::host_pneumatic::pin_writes;

void resetPins() {
  mclib::test::host_pneumatic::reset();
}

void retire(const std::unique_ptr<Command>& command) {
  CommandScheduler::endAndForget(command.get());
}

}  // namespace


namespace {

// ---------------------------------------------------------------------------
// 1. The README's two constructors: polarity never reaches the cached state.
// ---------------------------------------------------------------------------
void polarity() {
  std::printf("-- polarity stays in the device layer\n");
  resetPins();

  // Normal solenoid: pin high extends. Starts retracted, so both pins low.
  PneumaticSubsystem wings({'E', 'F'});
  CHECK(!wings.isExtended());
  CHECK_EQ(static_cast<double>(wings.count()), 2.0);
  CHECK(g_pin['E'] == false);
  CHECK(g_pin['F'] == false);
  {
    const std::vector<bool> raw = wings.rawValues();
    CHECK(raw.size() == 2 && !raw[0] && !raw[1]);
  }

  // Inverted solenoid: pin low extends. Starts extended, so the pin is low,
  // and isExtended() still says true.
  PneumaticSubsystem clamp({'G'}, /*initial_extended=*/true, /*extended_state=*/false);
  CHECK(clamp.isExtended());
  CHECK(g_pin['G'] == false);
  {
    // rawValues() is logical too, not the pin level.
    const std::vector<bool> raw = clamp.rawValues();
    CHECK(raw.size() == 1 && raw[0] == true);
  }

  // The constructor drove the hardware, so the cache and the solenoid agree
  // before any periodic() tick. One write from Pneumatic's constructor, one
  // from the subsystem's.
  CHECK_EQ(static_cast<double>(g_pin_writes['G']), 2.0);
  CHECK_EQ(static_cast<double>(g_pin_writes['E']), 2.0);

  // Moving each: the logical state is the same, the pins are opposite.
  wings.extend();
  clamp.extend();
  wings.runPeriodic();
  clamp.runPeriodic();
  CHECK(wings.isExtended());
  CHECK(clamp.isExtended());
  CHECK(g_pin['E'] == true);
  CHECK(g_pin['F'] == true);
  CHECK(g_pin['G'] == false);

  wings.retract();
  clamp.retract();
  wings.runPeriodic();
  clamp.runPeriodic();
  CHECK(!wings.isExtended());
  CHECK(!clamp.isExtended());
  CHECK(g_pin['E'] == false);
  CHECK(g_pin['G'] == true);
}

// ---------------------------------------------------------------------------
// 2. The pre-built constructor seeds the hardware from initial_extended.
// ---------------------------------------------------------------------------
void prebuiltPneumatics() {
  std::printf("-- pre-built pneumatics\n");
  resetPins();

  // Both devices start retracted on their own; the subsystem says extended and
  // drives them there at once.
  std::vector<std::shared_ptr<Pneumatic>> devices = {
      std::make_shared<Pneumatic>('A', false, true),
      std::make_shared<Pneumatic>('B', false, false),
  };
  PneumaticSubsystem lift(devices, /*initial_extended=*/true);
  CHECK(lift.isExtended());
  CHECK(g_pin['A'] == true);   // normal wiring, extended -> high
  CHECK(g_pin['B'] == false);  // inverted wiring, extended -> low
  CHECK(devices[0]->get_value());
  CHECK(devices[1]->get_value());
  {
    const std::vector<bool> raw = lift.rawValues();
    CHECK(raw.size() == 2 && raw[0] && raw[1]);
  }

  // A null entry is tolerated and reads back false.
  std::vector<std::shared_ptr<Pneumatic>> holey = {std::make_shared<Pneumatic>('C'), nullptr};
  PneumaticSubsystem partial(holey, false);
  CHECK_EQ(static_cast<double>(partial.count()), 2.0);
  partial.extend();
  partial.runPeriodic();
  {
    const std::vector<bool> raw = partial.rawValues();
    CHECK(raw.size() == 2 && raw[0] && !raw[1]);
  }
  CHECK(partial.group().get_value());

  // No pneumatics at all.
  PneumaticSubsystem empty(std::vector<std::shared_ptr<Pneumatic>>{}, true);
  CHECK(empty.isExtended());
  CHECK_EQ(static_cast<double>(empty.count()), 0.0);
  CHECK(empty.rawValues().empty());
  empty.toggle();
  empty.runPeriodic();
  CHECK(!empty.isExtended());
}

// ---------------------------------------------------------------------------
// 3. Mutators cache; periodic() writes; group() writes are overwritten.
// ---------------------------------------------------------------------------
void cacheAndPeriodic() {
  std::printf("-- cache, periodic, and group()\n");
  resetPins();
  PneumaticSubsystem p({'H'});
  const int after_construct = g_pin_writes['H'];

  // setExtended()/toggle() update the cache. The hardware follows on the next
  // periodic() tick, and every tick re-applies it.
  p.setExtended(true);
  CHECK(p.isExtended());
  CHECK_EQ(static_cast<double>(g_pin_writes['H']), static_cast<double>(after_construct));
  p.runPeriodic();
  CHECK(g_pin['H'] == true);
  CHECK_EQ(static_cast<double>(g_pin_writes['H']), static_cast<double>(after_construct + 1));
  p.runPeriodic();
  p.runPeriodic();
  CHECK_EQ(static_cast<double>(g_pin_writes['H']), static_cast<double>(after_construct + 3));

  p.toggle();
  CHECK(!p.isExtended());
  p.toggle();
  CHECK(p.isExtended());

  // A write through group() moves the solenoid but not the cache, and the
  // next tick puts the solenoid back.
  p.group().set_value(false);
  CHECK(g_pin['H'] == false);
  CHECK(p.rawValues().front() == false);
  CHECK(p.isExtended());
  p.runPeriodic();
  CHECK(g_pin['H'] == true);
  CHECK(p.rawValues().front() == true);

  // Disabled: periodic() is skipped, the cache still moves.
  p.setEnabled(false);
  p.retract();
  p.runPeriodic();
  CHECK(!p.isExtended());
  CHECK(g_pin['H'] == true);
  p.setEnabled(true);
  p.runPeriodic();
  CHECK(g_pin['H'] == false);
}

// ---------------------------------------------------------------------------
// 4. Command termination, and the hold-as-default trap.
// ---------------------------------------------------------------------------
void commands() {
  std::printf("-- command termination\n");
  resetPins();
  PneumaticSubsystem p({'J'});
  p.setName("pneumatic");
  p.setDefaultCommand(p.idleCommand());
  p.registerSelf();
  CommandScheduler::run();

  // One-shots: set at schedule(), gone after one run(), state stays.
  std::unique_ptr<Command> extend = p.makeExtendCommand();
  CommandScheduler::schedule(extend.get());
  CHECK(p.isExtended());
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(extend.get()));
  CHECK(g_pin['J'] == true);
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(p.isExtended());
  CHECK(g_pin['J'] == true);

  std::unique_ptr<Command> retract = p.makeRetractCommand();
  CommandScheduler::schedule(retract.get());
  CommandScheduler::run();
  CHECK(!p.isExtended());
  CHECK(!CommandScheduler::scheduled(retract.get()));
  CHECK(g_pin['J'] == false);

  std::unique_ptr<Command> toggle = p.makeToggleCommand();
  CommandScheduler::schedule(toggle.get());
  CommandScheduler::run();
  CHECK(p.isExtended());
  CHECK(!CommandScheduler::scheduled(toggle.get()));

  std::unique_ptr<Command> set_false = p.makeSetCommand(false);
  CommandScheduler::schedule(set_false.get());
  CommandScheduler::run();
  CHECK(!p.isExtended());
  CHECK(!CommandScheduler::scheduled(set_false.get()));

  // makeExtendForCommand: extend, hold, finish on the duration, stay extended.
  g_fake_ms = 1000;
  std::unique_ptr<Command> pulse = p.makeExtendForCommand(100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(pulse.get());
  CHECK(p.isExtended());
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(pulse.get())) {
      finished_on = i;
      break;
    }
    CHECK(p.isExtended());
  }
  std::printf("   100 ms pulse at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  CHECK(p.isExtended());
  CommandScheduler::run();
  CHECK(p.isExtended());

  // makeHoldCommand never finishes and pins the state every tick.
  std::unique_ptr<Command> hold = p.makeHoldCommand(false);
  CommandScheduler::schedule(hold.get());
  for (int i = 0; i < 5; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    CHECK(CommandScheduler::scheduled(hold.get()));
    CHECK(!p.isExtended());
  }
  // Something else flips the cache; the hold flips it straight back.
  p.extend();
  CommandScheduler::run();
  CHECK(!p.isExtended());

  retire(extend);
  retire(retract);
  retire(toggle);
  retire(set_false);
  retire(pulse);
  retire(hold);
  CommandScheduler::unregisterSubsystem(&p);

  // The trap the README describes: a hold as the default command drags the
  // cylinder back one tick after every one-shot.
  std::printf("-- hold command as the default\n");
  PneumaticSubsystem trapped({'K'});
  trapped.setName("trapped");
  trapped.setDefaultCommand(trapped.makeHoldCommand(false));
  trapped.registerSelf();
  CommandScheduler::run();  // default scheduled

  std::unique_ptr<Command> extend_once = trapped.makeExtendCommand();
  CommandScheduler::schedule(extend_once.get());  // interrupts the hold
  CHECK(trapped.isExtended());
  CommandScheduler::run();  // one-shot finishes; hold rescheduled
  CHECK(trapped.isExtended());
  CommandScheduler::run();  // hold executes: back to retracted
  CHECK(!trapped.isExtended());
  // run() ticks periodic() before it executes commands, so the solenoid saw
  // the old cache on that pass and follows on the next one.
  CHECK(g_pin['K'] == true);
  CommandScheduler::run();
  CHECK(g_pin['K'] == false);

  retire(extend_once);
  CommandScheduler::unregisterSubsystem(&trapped);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  polarity();
  prebuiltPneumatics();
  cacheAndPeriodic();
  commands();
  return mclib::test::summary("pneumatic_subsystem");
}
