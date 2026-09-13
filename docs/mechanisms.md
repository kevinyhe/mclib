# Mechanisms

The mechanism classes: position, velocity, toggle, conveyor, homing, PTO, pneumatics and the rest.

[Documentation index](README.md) · [Project README](../README.md)

## PositionMechanism

`mechanism/position_mechanism.hpp` is a generic PID position mechanism. The
state is the target position, and the device is injected as two callables, so
the same class works with a rotation sensor, a motor encoder, or a
potentiometer.

```cpp
mclib::device::MotorGroup lift_motors({1, -2}, mclib::device::Gearset::Green);
mclib::device::Rotation lift_sensor(3);

mclib::mechanism::PositionMechanismConfig cfg;
cfg.kp = 0.09;
cfg.max_voltage = 10.0;
cfg.small_error = 1.5;

mclib::mechanism::PositionMechanism lift(
    [&] { return lift_sensor.getPositionDeg(); },
    [&](double volts) { lift_motors.setVoltage(volts); },
    cfg);
```

`moveTo(target)` starts closed-loop control and restarts the PID every time,
even when the target is unchanged, so re-issuing a target after a manual
override takes control back. `setManualVoltage(volts)` overrides the loop until
the next `moveTo()`. `stop()` brakes rather than coasts: it latches the present
position as the target and holds it, which keeps a loaded arm from falling.
Call `setManualVoltage(0.0)` if you want it to go limp instead. A mechanism
starts idle at 0 V until one of those is called.

Commands:

```cpp
lift.makeMoveToCommand(90.0);            // finishes on atTarget()
lift.makeMoveToCommand(90.0, 1500.0);    // also finishes after 1.5 s
lift.makeHoldCommand();                  // hold the current target
lift.makeManualCommand(6.0);             // 6 V while scheduled
lift.makeStopCommand();                  // hold where it is
```

`makeMoveToCommand` is the only factory that ends on its own; the others run
until interrupted.

`PID` stops driving once it flags arrival, so a settled mechanism would have no
holding torque. `PositionMechanismConfig::hold_output` defaults to true, which
keeps the loop driving after arrival so a loaded mechanism holds instead of
sagging. `atTarget()` latches on the same tick either way, so a finished move
still finishes.

This is a P (plus D) hold, not a zero-error hold: arrival needs `|error| <=
small_error` and the PID zeroes its integral over that same band, so a loaded
mechanism droops up to `small_error`. Lower `small_error` to shrink that, at
the cost of a tighter arrival test.

Set `hold_output = false` for the historical behaviour - 0 V on arrival, with
the loop re-arming only once the position drifts back outside `small_error`, so
the hold has a `small_error` deadband. `atTarget()` stays true across that
re-arm.

### Migrating from `Arm`

`mechanism::Arm` is gone. It was a `PositionMechanism` with a `device::MotorGroup`
and a `device::Rotation` hardcoded inside it. Build the two devices yourself and
pass them in as lambdas:

| Before (`Arm`) | After (`PositionMechanism`) |
| --- | --- |
| `ArmConfig::motor_ports`, `gearset` | `device::MotorGroup motors(ports, gearset)`, wrapped in the voltage sink |
| `ArmConfig::rotation_port`, `rotation_reversed` | `device::Rotation sensor(port, reversed)`, wrapped in the position source |
| `ArmConfig::small_error_deg` / `big_error_deg` | `PositionMechanismConfig::small_error` / `big_error` |
| `kp`, `ki`, `kd`, `max_voltage`, `small_duration_ms`, `big_duration_ms`, `derivative_tolerance` | same names on `PositionMechanismConfig` |
| `Arm arm({-3, 4}, 8);` | build the `MotorGroup` and `Rotation` yourself and pass them as lambdas |
| `moveTo`, `setManualVoltage`, `positionDeg`, `targetDeg`, `atTarget` | same names, same meaning |
| `makeMoveToCommand`, `makeManualCommand`, `makeStopCommand` | same names, same meaning |
| `stop()` cut the motors to 0 V | `stop()` holds the present position; use `setManualVoltage(0.0)` to coast |

Two behaviour changes to know about:

- `stop()` now brakes instead of coasting, as described above.
- `Arm` inherited the `PID` arrival latch, so once it settled it output a hard
  0 V and a loaded arm sagged. `PositionMechanismConfig::hold_output` defaults
  to true, so the loop keeps driving after it arrives and holds the setpoint.
  `atTarget()` latches on the same tick either way, so a finished move still
  finishes. Set `hold_output = false` for the old settle-then-release
  behaviour; the loop then re-arms only once the position drifts back outside
  `small_error`.

`PositionMechanism` is not tied to a rotation sensor. The position source is any
`std::function<double()>`, so a motor encoder or a potentiometer works the same
way.

## Conveyor Mechanism

`mechanism/conveyor_mechanism.hpp` adds a sensor-gated conveyor with jam
recovery.

```cpp
mclib::mechanism::ConveyorConfig config;
config.motor_ports = {11, -12};
config.forward_voltage = 12.0;
config.index_voltage = 8.0;
config.jam_current_amps = 2.0;
config.jam_velocity_rpm = 10.0;
config.jam_dwell_ms = 250.0;
config.unjam_voltage = -8.0;
config.unjam_ms = 250.0;
config.jam_clear_ms = 1000.0;
config.max_unjam_retries = 3;

mclib::device::Distance sensor(13);
mclib::mechanism::ConveyorMechanism conveyor(
    config, [&sensor] { return sensor.getDistanceMm() < 60.0; });

auto conveyor_stop = conveyor.makeStopCommand();
CommandScheduler::registerSubsystem(&conveyor, conveyor_stop.get());

conveyor.setConveyorState(mclib::mechanism::ConveyorState::Forward);
conveyor.stop();
bool ready = conveyor.hasObject();
bool stuck = conveyor.isJammed();
```

`applyState` runs from `periodic()`, so the conveyor must be registered with
`CommandScheduler` or its motors never move.

Commands:

```cpp
conveyor.makeForwardCommand();
conveyor.makeReverseCommand();
conveyor.makeStopCommand();
conveyor.makeIndexCommand(1500.0);  // finishes when the gate latches, or on timeout
conveyor.makeStateCommand(mclib::mechanism::ConveyorState::Reverse);
```

`makeIndexCommand` is the only factory here that finishes on its own. The rest
run until interrupted.

Jam handling: current above `jam_current_amps` while `|velocity|` is below
`jam_velocity_rpm` for `jam_dwell_ms` triggers an unjam at `unjam_voltage` for
`unjam_ms`, then the previous state resumes. After `max_unjam_retries`
consecutive attempts the motors stop and `isJammed()` latches true until the
state changes or `clearJam()` is called; the retry count refills after
`jam_clear_ms` of unstalled running. Without a sensor gate, `IndexToSensor`
holds at zero volts, `hasObject()` is always false, and `makeIndexCommand`
finishes immediately.

Sign-correct opposed motors in `motor_ports` (a negative port reverses that
motor). Jam detection reads the group average, so uncorrected ports cancel to
a near-zero velocity and look permanently stalled.

## VelocityMechanism

A closed-loop velocity mechanism whose state is the target RPM. The velocity
sensor and the voltage output are injected as callbacks, so it is not tied to
any particular device.

```cpp
ml::device::MotorGroup motors({1, -2}, ml::device::Gearset::Blue);

ml::mechanism::VelocityMechanismConfig config;
config.kv = 12.0 / 600.0;  // volts per motor RPM, max volts / motor free speed
config.kp = 0.01;
config.tolerance_rpm = 50.0;
config.dwell_ms = 200.0;

ml::mechanism::VelocityMechanism spinner(
    [&motors] { return motors.getAverageActualVelocity(); },
    [&motors](double volts) { motors.setVoltage(volts); },
    config);

spinner.setTargetRpm(500.0);
spinner.getCurrentRpm();
spinner.atSpeed();      // within tolerance_rpm for dwell_ms straight
spinner.isSpinningUp(); // target commanded, not there yet
spinner.stop();         // coasts down
```

Output is a `kv * target` feedforward plus PID correction, clamped to the sign
of the target so the mechanism is never driven backwards to brake. The integral
only accumulates within `integral_range_rpm` of the target, which keeps it from
winding up during spin-up.

### Gear ratio

`config.ratio` is the external gearing between the motor and the output shaft,
written as **output RPM per motor RPM**:

```
output_rpm = motor_rpm * ratio
motor_rpm  = output_rpm / ratio
```

Geared 1:2 for speed, so the output spins twice as fast as the motor, is
`ratio = 2.0`. Geared 2:1 for torque is `ratio = 0.5`. The default `1.0` means
the output *is* the motor shaft, and reproduces the ungeared behaviour exactly.

Each value is in one of two RPM spaces:

| Quantity | Space |
| --- | --- |
| `setTargetRpm`, `getTargetRpm`, `getCurrentRpm` | output RPM |
| `makeSpinCommand`, `makeSpinUpCommand` | output RPM |
| `tolerance_rpm`, `integral_range_rpm` | output RPM |
| the velocity source callback, `getCurrentMotorRpm()` | motor RPM |
| `kp`, `ki`, `kd`, `kv` | volts per motor RPM |

Everything you pass to the mechanism is output RPM, because output speed is
what you are trying to control. The loop internally divides by
`ratio` and runs in motor RPM, so the gains keep their natural "volts per motor
RPM" meaning. For `kv` that means the number itself is unchanged by gearing - it
is set by the motor's free speed. `kp`, `ki` and `kd` still want retuning when
you change the gearing, because the load inertia reflected back to the motor
scales with `ratio` squared; what does not change is the units they are in.

A worked 1:2 example, with a blue-cartridge motor whose free speed is 600 motor
RPM driving a wheel through 1:2. The config has to be handed to the constructor,
so build the mechanism from it:

```cpp
ml::mechanism::VelocityMechanismConfig config;
config.ratio = 2.0;              // wheel spins 2x the motor
config.kv = 12.0 / 600.0;        // 0.02 V per motor RPM - unchanged by gearing
config.kp = 0.02;
config.tolerance_rpm = 10.0;     // 10 wheel RPM, i.e. 5 motor RPM
config.integral_range_rpm = 100.0;  // 100 wheel RPM, i.e. 50 motor RPM

ml::mechanism::VelocityMechanism spinner(
    [&motors] { return motors.getAverageActualVelocity(); },
    [&motors](double volts) { motors.setVoltage(volts); },
    config);

spinner.setTargetRpm(600.0);     // 600 wheel RPM = 300 motor RPM
```

With the motor measured at 280 RPM, the wheel is at 560 RPM:

- `getCurrentRpm()` returns `560.0`, `getCurrentMotorRpm()` returns `280.0`
- the PID error is the motor-space `300 - 280 = 20`, so P contributes
  `0.02 * 20 = 0.4 V`
- the feedforward is `kv * 300 = 6.0 V`, not `kv * 600`
- output is `6.4 V`

`atSpeed()` is judged in output RPM too. At `ratio = 2.0` with
`tolerance_rpm = 10.0`, a motor error of 6 RPM is a wheel error of 12 RPM and
does **not** count as at speed, even though 6 is under 10. Scaling the target
but not the tolerance would be a silent bug, so both are scaled.

`ratio` must be finite and greater than zero - zero would divide by zero, a
negative value would invert the loop, and an infinite one would divide every
target down to nothing. The constructor rejects anything else (NaN included) and
falls back to `1.0`, and `getConfig().ratio` then reports the `1.0` actually in
use.

`applyState` runs from `periodic()`, so the mechanism has to be registered with
the scheduler or nothing moves:

```cpp
std::unique_ptr<Command> spinner_stop;
std::unique_ptr<Command> spinner_hold;   // holds speed, never finishes
std::unique_ptr<Command> spinner_ready;  // finishes at speed or on timeout

void initialize() {
  spinner_stop = spinner.makeStopCommand();
  spinner_hold = spinner.makeSpinCommand(500.0);
  spinner_ready = spinner.makeSpinUpCommand(500.0, 2000.0);

  CommandScheduler::registerSubsystem(&spinner, spinner_stop.get());
}
```

## ToggleMechanism

`mclib::mechanism::ToggleMechanism` is a two-state mechanism built on
`StateMechanism<bool>`. It drives a `std::function<void(bool)>` actuator, so one
type covers solenoids, motor-driven two-position mechanisms, and test mocks.

The logical state ("is extended") is kept separate from the raw value written to
the actuator, so `isExtended()` always answers the question you asked.

```cpp
#include "mclib/mclib.hpp"

using mclib::mechanism::ToggleMechanism;
using mclib::mechanism::ToggleMechanismConfig;

// One solenoid, wired so that false on the port means extended. Tell the
// device about the wiring; do not set ToggleMechanismConfig::inverted here.
ToggleMechanism clamp(std::make_shared<mclib::device::Pneumatic>('A', false, false));

// Several pneumatics driven together.
ToggleMechanism wings(std::vector<std::shared_ptr<mclib::device::Pneumatic>>{
    std::make_shared<mclib::device::Pneumatic>('E'),
    std::make_shared<mclib::device::Pneumatic>('F'),
});

// Any actuator at all. `inverted` belongs here, on raw actuators, where
// nothing else already flips the value.
ToggleMechanism motor_latch([](bool value) { printf("%d\n", value); },
                            ToggleMechanismConfig{
                                .initial_extended = false,
                                .inverted = true,
                            });
```

`set()`, `extend()`, `retract()` and `toggle()` write the actuator immediately,
even when the value did not change, so they work before the scheduler is
running. Once registered, `periodic()` rewrites it every tick.

Every command factory finishes on its own, unlike `PneumaticSubsystem` where
only the toggle command terminates:

```cpp
CommandController primary(mclib::device::ControllerId::Master);

std::unique_ptr<Command> wings_in;
std::unique_ptr<Command> wings_toggle;
std::unique_ptr<Command> wings_pulse;

void initialize() {
  wings_in = wings.makeRetractCommand();
  wings_toggle = wings.makeToggleCommand();
  wings_pulse = wings.makeExtendForCommand(500 * millisecond);

  CommandScheduler::registerSubsystem(&wings, wings_in.get());

  primary.getTrigger(mclib::device::DigitalButton::L1)
      ->onTrue(wings_toggle.get());
}
```

`makeSetCommand`, `makeExtendCommand`, `makeRetractCommand` and
`makeToggleCommand` act once and end. The `...ForCommand(QTime)` variants hold
the state for the duration, then end and leave the mechanism in that state.
They do not restore the previous one.

## MechanismManager

`CommandScheduler::registerSubsystem` stores a raw `Command*`, but every
mechanism factory hands back a `std::unique_ptr<Command>`. Something has to keep
that command alive for as long as the scheduler can reach it. The examples above
do it with a global `std::unique_ptr<Command>` per mechanism; drop the global and
the scheduler is left holding a dangling pointer.

`mclib::mechanism::MechanismManager` owns those commands instead:

```cpp
#include "mclib/mclib.hpp"

ml::mechanism::MotorSubsystem flywheel{{1, -2}};
ml::mechanism::PneumaticSubsystem wings{{'A'}};

ml::mechanism::MechanismManager mechanisms;

void initialize() {
  mechanisms.add(&flywheel, flywheel.makeStopCommand(), "flywheel");
  mechanisms.add(&wings, wings.makeRetractCommand(), "wings");

  mechanisms.registerAll();
}

void opcontrol() {
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
```

The manager must outlive the scheduler's use of the commands, so give it static
or program-long storage. A file-scope object like the one above works.

`add()` returns `false` and stores nothing if the subsystem or command is null,
if that subsystem is already held, or if the name is already taken. `name` is
optional and defaults to `mechanism0`, `mechanism1`, and so on.

`registerAll()` is safe to call twice; it skips entries it already registered and
returns how many it registered this time. The scheduler's own duplicate check is
an `assert`, which does nothing in a release build.

The rest of the API:

```cpp
mechanisms.size();                       // how many entries are held
mechanisms.contains("wings");
mechanisms.getSubsystem("wings");        // Subsystem*, or nullptr
mechanisms.getDefaultCommand("wings");   // Command*, manager keeps ownership
mechanisms.getEntry("wings");            // const Entry*, or by index
mechanisms.getNames();                   // insertion order
mechanisms.isRegistered("wings");
mechanisms.isEnabled("wings");
mechanisms.setEnabled("wings", false);   // skip it on the next registerAll()
mechanisms.setAllEnabled(true);
printf("%s", mechanisms.describe().c_str());
```

`describe()` prints one line per entry: name, enabled, registered, and which
command currently holds the scheduler's requirement on that subsystem:

```
flywheel enabled=yes registered=yes requirement=default
wings enabled=yes registered=yes requirement=other
```

Disabling an entry only stops a later `registerAll()` from registering it. The
scheduler has no API to drop a subsystem once registered, so disabling an
already-registered entry changes the record and nothing else.

A subsystem must be registered by exactly one path. If you already call
`CommandScheduler::registerSubsystem` for a mechanism, do not also add it to a
manager: the scheduler catches the duplicate with an `assert`, which is compiled
out in release, and the second registration silently overwrites the first while
the original default command keeps the requirement.

`MechanismManager` is neither copyable nor movable. It owns the commands the
scheduler holds raw pointers to, and the scheduler has no unregister call, so
moving the manager would leave the scheduler pointing at freed commands.

## MultiPositionMechanism

`mclib::mechanism::MultiPositionMechanism<StateT>` is for a mechanism that moves
between a fixed, ordered list of setpoints: arm presets, lift stages, hood
angles. You give it a table mapping each state to a number and a sink that
accepts that number. Every scheduler tick it pushes the current state's setpoint
into the sink, so it works with any position controller without knowing about
it.

```cpp
enum class LiftStage { Down, Middle, High };

double lift_target = 0.0;

mclib::mechanism::MultiPositionMechanism<LiftStage> lift(
    LiftStage::Down,
    {{LiftStage::Down, 0.0}, {LiftStage::Middle, 12.5}, {LiftStage::High, 30.0}},
    [](double setpoint) { lift_target = setpoint; });

lift.setPosition(LiftStage::Middle);
lift.next();      // -> High, and stays at High if called again
lift.previous();  // -> Middle
lift.cycle();     // -> High, then wraps to Down

auto lift_hold = lift.idleCommand();
CommandScheduler::registerSubsystem(&lift, lift_hold.get());

auto to_high = lift.makePositionCommand(LiftStage::High);
auto step_up = lift.makeNextCommand();      // one-shot, finishes immediately
auto step_dn = lift.makePreviousCommand();  // one-shot
auto rotate  = lift.makeCycleCommand();     // one-shot
```

The mechanism only moves once it is registered with `CommandScheduler`, which
is what calls `periodic()`. Register it with an idle default command, not with
`makePositionCommand(...)`: a position command runs forever and re-asserts its
state every tick, so if it is the default it will undo a `makeNextCommand()`
press on the tick right after that one-shot finishes.

`next()` and `previous()` clamp at the ends of the table. `cycle()` is the one
that wraps from the last entry back to the first. The table order, not the
numeric setpoints, decides what "next" means.

Degenerate cases are safe because `applyState` runs every tick. With an empty
table, or when the current state is not in the table, the sink is given the
default setpoint (constructor argument, `0.0` unless you pass one). That is the
same number `setpointFor()` and `currentSetpoint()` report, so an at-target
check never disagrees with what the sink was actually given. `currentIndex()`
returns `kNoPosition` for a state that is off the table, and `positionAt()`
returns `nullptr` for an index that is out of range. Calling `next()`,
`previous()`, or `cycle()` while the state is off the table moves to the first
entry, which is the home position by convention.

The table is searched front to back and the first match wins, so listing a
state twice makes the later entry unreachable. List each state once.

## HomingMechanism

`mechanism/homing_mechanism.hpp` finds a mechanism's zero by driving into a hard
stop, then re-zeroing the position sensor. All hardware access is injected, so it
works with a `device::Motor`, a `device::MotorGroup`, a `device::Rotation`, or any
mix of them.

```cpp
mclib::mechanism::HomingMechanismConfig cfg;
cfg.homing_voltage = -3.0;         // signed: direction matters
cfg.current_threshold_amps = 2.0;  // <= 0 disables the current detector
cfg.velocity_threshold_rpm = 5.0;  // <= 0 disables the stall detector
cfg.stall_dwell_ms = 150.0;
cfg.startup_grace_ms = 150.0;      // ramp-up window
cfg.timeout_ms = 3000.0;
cfg.backoff_voltage = 2.0;         // 0 or backoff_ms 0 disables back-off
cfg.backoff_ms = 150.0;

mclib::mechanism::HomingMechanism homing(
    cfg, [&](double volts) { arm_motors.setVoltage(volts); });

homing.setVelocitySource([&] { return arm_motors.getAverageActualVelocity(); });
// getAverageCurrentDraw() is in milliamps; the config threshold is in amps.
homing.setCurrentSource(
    [&] { return arm_motors.getAverageCurrentDraw() / 1000.0; });
homing.setPositionReset([&] { arm_rotation.resetPosition(); });

// Optional third detector, any predicate that is true at the stop.
pros::adi::DigitalIn arm_limit('A');
homing.setLimitSwitch([&] { return arm_limit.get_value() != 0; });
```

Three stop detectors can be enabled independently, and the first one to fire
wins: the limit-switch predicate, current draw above `current_threshold_amps`,
and a velocity stall (`|rpm| < velocity_threshold_rpm` held for
`stall_dwell_ms`). Inrush current is ignored for the first `startup_grace_ms`,
and the stall detector arms once the mechanism has been seen moving, or once
that window has passed, which covers a mechanism that starts out resting
against the stop. The sensor is zeroed at the stop, before any back-off.

Drive it either from the state machine directly (`startHoming()`,
`cancelHoming()`, `isHoming()`, `isHomed()`, `hasFailed()`) or with a command:

```cpp
CommandScheduler::registerSubsystem(&homing, nullptr);

auto home_arm = homing.makeHomeCommand(4000 * millisecond);
home_arm->schedule();
while (home_arm->scheduled()) {
  pros::delay(10);
}
```

`makeHomeCommand` always finishes (on success, on failure, or on its own
timeout) and stops the motor on every exit path, interruption included. It also
steps the state machine from `execute()`, so it still works if the mechanism is
not registered with the scheduler.

## PTO (`mechanism/pto_mechanism.hpp`)

`PTOMechanism` is one set of motors mechanically switched between two consumers,
usually the drivetrain and a lift. It is a `StateMechanism<bool>`: `true` means the
motors are routed to the engaged consumer, `false` to the disengaged one.

```cpp
mclib::mechanism::PTOConfig pto_config{
    .motor_ports = {1, -2},
    .adi_port = 'A',
    .engaged_when_extended = true,   // extending the solenoid routes to the lift
    .shift_settle_time = 250 * millisecond,
    .drive_timeout = 100 * millisecond,
};

mclib::mechanism::PTOMechanism pto(pto_config);
std::unique_ptr<Command> pto_idle;

void initialize() {
  // The default command must never finish and must not force a state: an
  // instant default would re-disengage the PTO on the tick after every engage.
  pto_idle = pto.idleCommand();
  CommandScheduler::registerSubsystem(&pto, pto_idle.get());
}

void opcontrol() {
  while (true) {
    // Drive writes are tagged with the side asking for them.
    pto.driveDisengaged(12.0);  // drivetrain: accepted while the PTO is disengaged
    pto.driveEngaged(12.0);     // lift: dropped while the PTO is disengaged

    CommandScheduler::run();
    pros::delay(10);
  }
}
```

Both drive calls return `true` when the write was accepted and `false` when it was
dropped. The accepted voltage reaches the motors on the next `periodic()` tick, so
register the mechanism with `CommandScheduler`. A write from the side that does not
own the PTO is discarded, not queued: a lift command writing volts while the motors
are geared to the wheels would drive the robot across the field.

Shifting under load shears gear teeth, so `engage()`, `disengage()` and `toggle()`
zero and brake the motors, throw the solenoid immediately, and refuse drive writes
from both sides for `shift_settle_time` (default 250 ms). A commanded voltage of
zero brakes rather than coasts, so a raised lift stays put. `isShiftSettled()` and
`remainingSettleTime()` report where the transition is.

The owning side is expected to write every tick. If it stops for longer than
`drive_timeout` (default 100 ms) the commanded voltage decays to zero instead of
latching on the motors; set `drive_timeout` to `0 * millisecond` to keep the
last value.

`motors()` exposes the shared `device::MotorGroup` for brake modes, encoders, and
temperatures. Writing voltage through it bypasses the guard; use
`motorsFor(engaged_side)` for ownership-checked configuration and telemetry, and
re-fetch it every tick rather than caching it across a shift. Voltage always goes
through the drive calls: `periodic()` rewrites the commanded voltage every tick, so
a direct `setVoltage()` is overwritten on the next scheduler pass. The same holds
for the constructor that takes an existing `MotorGroup`. Once the PTO shares it,
the drivetrain must write through `driveDisengaged()` too.

Commands: `makeEngageCommand()`, `makeDisengageCommand()` and `makeToggleCommand()`
are one-shot and finish immediately. `makeShiftCommand(engaged)` requires the
mechanism for the whole settle window and finishes only once the shift has settled,
so nothing can drive into a half-thrown gearbox.

## PneumaticSubsystem: polarity and command termination

`PneumaticSubsystem` state is always the logical state. `true` means the
cylinder is extended, `false` means it is retracted. Wiring polarity is a
separate flag and stays inside `device::Pneumatic`, which maps
`raw = (logical == extended_state)`. Nothing above the device layer sees the
solenoid pin level.

```cpp
// Normal solenoid: pin high extends.
mclib::mechanism::PneumaticSubsystem wings({'E', 'F'});

// Inverted solenoid: pin low extends. Starts extended.
mclib::mechanism::PneumaticSubsystem clamp({'G'}, /*initial_extended=*/true,
                                           /*extended_state=*/false);

clamp.isExtended();  // true, for either wiring, and from construction on
```

The second constructor argument is the logical starting state, not a pin
level. The constructor drives the solenoids to that state, so `isExtended()`
agrees with the hardware before the first `periodic()` tick.

Command termination:

- `makeSetCommand`, `makeExtendCommand`, `makeRetractCommand`, and
  `makeToggleCommand` all set the state once and finish. Setting a solenoid is
  a one-shot action, so holding the subsystem requirement forever is wrong.
- `makeExtendForCommand(500 * millisecond)` extends, holds, then finishes.
- `makeHoldCommand(extended)` forces a state on every tick and never finishes.
  Do not register it as a default command: the scheduler reschedules the
  default as soon as a one-shot command releases the requirement, so a hold
  default would drag the cylinder back one tick after every `makeExtendCommand`.
  Register `idleCommand()` instead. `periodic()` keeps re-applying the cached
  state, so the cylinder stays where the last command put it.

Accessors: `count()`, `group()` for direct device access, and `rawValues()`
for the per-solenoid values the device layer reports. Those values are logical
too, not pin levels; they are for spotting one solenoid out of sync with the
rest of the group. Writes through `group()` do not update the cached state and
are overwritten by the next `periodic()`.

## ToggleGroupMechanism

`ToggleMechanism` moves a whole group of actuators with one shared boolean.
`ToggleGroupMechanism` gives each actuator its own boolean while keeping them
in a single subsystem, so they share one scheduler requirement and two commands
can never fight over them.

Channels are addressed by index, and any scoped enum works as an index:

```cpp
enum class Side : std::size_t { Left = 0, Right = 1 };

// One pneumatic per channel. Both plumbed so pin high extends.
mclib::mechanism::ToggleGroupMechanism sides({
    std::make_shared<mclib::device::Pneumatic>('A'),
    std::make_shared<mclib::device::Pneumatic>('B'),
});

void example() {
  sides.set(Side::Left, true);
  sides.get(Side::Right);   // false
  sides.toggle(Side::Right);
  sides.setAll(false);
  sides.toggleAll();
  sides.allSet();           // every channel extended
  sides.anySet();           // at least one extended
  sides.setCount();         // how many are extended
  sides.count();            // how many channels there are
}
```

Every command factory finishes on its own: the one-shot ones after a single
tick, the `...ForCommand(QTime)` variants once their duration is up. The timed
variants stay where they finish, they do not restore the old state.

`Trigger::onTrue` takes a raw `Command*`, so something has to keep the command
alive for as long as the scheduler can reach it. Hold each one in a
`std::unique_ptr` that outlives the scheduler, or hand them to a
`MechanismManager`; binding a temporary leaves a dangling pointer.

```cpp
CommandController primary(mclib::device::ControllerId::Master);

std::unique_ptr<Command> left_toggle;
std::unique_ptr<Command> right_toggle;
std::unique_ptr<Command> both_out;
std::unique_ptr<Command> both_toggle;
std::unique_ptr<Command> left_pulse;

void initialize() {
  left_toggle = sides.makeToggleCommand(Side::Left);
  right_toggle = sides.makeToggleCommand(Side::Right);
  both_out = sides.makeSetAllCommand(true);
  both_toggle = sides.makeToggleAllCommand();
  left_pulse = sides.makeSetForCommand(Side::Left, true, 500 * millisecond);

  sides.setName("sides");
  sides.setDefaultCommand(sides.idleCommand());  // see below, this matters
  sides.registerSelf();

  primary.getTrigger(mclib::device::DigitalButton::L1)->onTrue(left_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::R1)->onTrue(right_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::A)->onTrue(both_out.get());
  primary.getTrigger(mclib::device::DigitalButton::B)->onTrue(both_toggle.get());
  primary.getTrigger(mclib::device::DigitalButton::X)->onTrue(left_pulse.get());
}
```

The default command MUST be `idleCommand()`. Every factory above terminates,
which releases the requirement, and `CommandScheduler::run()` reschedules the
default on the very next tick. A "retract everything" default would therefore
undo every set one tick after it happened. `periodic()` keeps re-writing the
cached states, so the actuators stay where the last command put them.

Per channel inversion is for raw `std::function<void(bool)>` actuators only:

```cpp
bool left_raw = false;
bool right_raw = false;

// Channel 1's actuator is wired backwards, so it gets the negated value while
// get(1) still answers "is it extended".
mclib::mechanism::ToggleGroupMechanism raw_sides(
    {[](bool v) { left_raw = v; }, [](bool v) { right_raw = v; }},
    {.initial_states = {false, true}, .inverted = {false, true}});
```

Lambdas that capture by reference cannot live at namespace scope, so build the
mechanism inside a function if your actuators need captures.

Do not set `inverted` for a channel driving a `device::Pneumatic`. `Pneumatic`
already takes and returns a logical value and inverts internally through its
`extended_state` flag. Inverting on top of that inverts twice and leaves
`Pneumatic::get_value()` reporting the opposite of `get(index)`. For a solenoid
wired backwards, write `Pneumatic(port, false, false)` and leave the inversion
entry false.

Out of range indices are ignored by the mutators and read back as `false`, so a
bad enum value cannot corrupt the state. Copy and move are deleted.

The channel count is fixed at construction by the number of actuators. The
inherited `setState()` and the generic `makeState*Command()` factories are
hidden, because they take a `std::vector<bool>` of any length and caching a
short one would make the trailing channels unreachable and latch their
actuators wherever they happened to be. Use `setAll()` and the factories above.

## `DiscreteActuatorMechanism`

`ToggleMechanism` drives one boolean actuator. `MultiPositionMechanism` maps a
state to one number. Neither covers several solenoids whose combined pattern
encodes a handful of named positions. `DiscreteActuatorMechanism<StateT>` owns
that decode table and pushes the pattern of the current state into a bank of
injected actuators every tick.

```cpp
enum class Tilt { Flat, Angled, Steep };

// Two cylinders, three named angles. The table is the only source of truth:
// to add an angle, add a row. There is no if/else chain anywhere.
mclib::mechanism::DiscreteActuatorMechanism<Tilt> tilt(
    Tilt::Flat,
    {{Tilt::Flat,   {false, false}},
     {Tilt::Angled, {true,  false}},
     {Tilt::Steep,  {true,  true}}},
    std::vector<std::shared_ptr<mclib::device::Pneumatic>>{front, back});

void initialize() {
  tilt.setName("tilt");
  // MUST be idleCommand(). Every factory below finishes after one tick, which
  // releases the requirement and lets the scheduler reschedule the default
  // immediately. A default that commanded a state would drag the cylinders
  // back one tick after every button press. periodic() keeps re-applying the
  // cached state, so the cylinders stay where the last command put them.
  tilt.setDefaultCommand(tilt.idleCommand());
  tilt.registerSelf();
}
```

Table values are logical: `true` means extended. `device::Pneumatic` maps
`raw = (logical == extended_state)` itself, so a backwards-wired cylinder gets
`extended_state = false` at the device and the table stays readable. The
`inverted` flag in `DiscreteActuatorMechanismConfig` is for raw
`std::function<void(bool)>` actuators only; using it with a `Pneumatic` inverts
twice.

Stepping follows the same convention as `MultiPositionMechanism`:

- `next()` and `previous()` clamp. At the last row `next()` does nothing.
- `cycle()` wraps. From the last row it goes back to the first.

Commands, all terminating: `makeDiscreteStateCommand(state)`,
`makeNextCommand()`, `makePreviousCommand()`, `makeCycleCommand()`, and
`makeDiscreteStateForCommand(state, 500 * millisecond)` which holds a state for
a duration then finishes and stays there.

```cpp
// Triggers take a raw pointer, so keep the commands alive yourself.
std::unique_ptr<Command> tilt_cycle;
std::unique_ptr<Command> tilt_flat;

void initialize() {
  tilt_cycle = tilt.makeCycleCommand();
  tilt_flat = tilt.makeDiscreteStateCommand(Tilt::Flat);

  primary.getTrigger(mclib::device::DigitalButton::R1)
      ->onTrue(tilt_cycle.get());
  primary.getTrigger(mclib::device::DigitalButton::A)
      ->onTrue(tilt_flat.get());
}
```

Nothing here can drive a half-decoded combination. Rows whose width does not
match the actuator count are dropped at construction, so every surviving row
applies whole. A state that is not in the table is undecodable:
`applyState()` writes nothing for it and the actuators hold the last pattern
they were given, `setDiscreteState()` refuses to move to it, and the inherited
`setState()` / `makeStateCommand()` are private so no command can latch a state
that can never be applied. Only the initial state can be undecodable, and the
first `next()`, `previous()`, or `cycle()` resyncs to row 0.

Dropping is silent, so one mistyped row turns into a button that does nothing.
`droppedRowCount()` reports how many rows were thrown away; check it once at
startup if a position looks dead.

`makeDiscreteStateForCommand` is the only mutator that does not write the
actuators itself: it caches the state and lets `periodic()` push it, the same
as `ToggleMechanism::makeSetForCommand`. That costs one tick normally, and
costs everything while the subsystem is disabled, since `periodic()` is then
skipped entirely.

Accessors: `stateCount()`, `actuatorCount()`, `droppedRowCount()`, `empty()`, `indexOf(state)`,
`currentIndex()`, `stateAt(index)`, `isDecodable(state)`,
`combinationFor(state)`, `currentCombination()`, `rawValue(index, value)`.
`StateT` only needs `operator==`, so a scoped enum works.

## PresetPositionMechanism: presets on top of a PID loop

`PresetPositionMechanism<StateT>` is `MultiPositionMechanism` and
`PositionMechanism` wired together. The first owns the ordered preset table and
`next` / `previous` / `cycle`; the second owns the PID loop, the tolerance and
dwell exit, the voltage clamp and the manual-voltage override. Neither is
re-implemented. Use it for an articulated joint, a lift, a tilter, an indexer:
anything with discrete stops. The preset enum is yours.

```cpp
enum class ArmPreset { Down, Load, Score };

mclib::device::MotorGroup arm_motors({-11, 12});
mclib::device::Rotation arm_sensor(13);

mclib::mechanism::PresetPositionMechanism<ArmPreset> arm(
    ArmPreset::Down,
    {{ArmPreset::Down, 0.0}, {ArmPreset::Load, 45.0}, {ArmPreset::Score, 130.0}},
    [&]() { return arm_sensor.getAngleDeg(); },
    [&](double volts) { arm_motors.setVoltage(volts); },
    mclib::mechanism::PositionMechanismConfig{.kp = 0.12, .small_error = 1.5});

void initialize() {
  arm_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  arm.setName("arm");
  arm.setDefaultCommand(arm.idleCommand());
  arm.registerSelf();
}
```

One registration, not two. `MultiPositionMechanism` and `PositionMechanism` are
both `Subsystem`s, and registering both for one physical mechanism would let two
commands drive it at the same time. So the class *derives* from
`MultiPositionMechanism` (that is the scheduler requirement) and *owns* the
`PositionMechanism` as a component that is never registered; it is ticked from
the outer `applyState()`. `controller()` exposes it for tuning and diagnostics.
Do not register what `controller()` returns.

API: `setPreset(preset)`, `getPreset()`, `next()`, `previous()`, `cycle()`,
`moveTo(raw)`, `positionValue()`, `targetValue()`, `presetSetpoint()`,
`atTarget()`, `isManual()`, `setManualVoltage(volts)`, `stop()`.

Commands: `makePresetCommand(preset, timeout_ms = 0.0)` and
`makeMoveToCommand(raw, timeout_ms = 0.0)` finish on `atTarget()` or on the
timeout, and brake if interrupted. `makeNextCommand()`, `makePreviousCommand()`
and `makeCycleCommand()` are one-shots. `makeManualCommand(volts)` and
`makeStopCommand()` hold the requirement for as long as they run.

What to expect:

- Nothing drives until you ask. A fresh mechanism sits at 0 V with the loop
  disengaged, like a bare `PositionMechanism`. The first `setPreset()`,
  step, `moveTo()`, `setManualVoltage()` or `stop()` engages it.
- The table pushes its setpoint every tick, but the mechanism only retargets
  when the setpoint changed or when a retarget was asked for. Feeding
  the sink straight into `moveTo()` would reset the PID 50 times a second and
  the loop would never settle.
- Re-issuing the *same* preset after a manual override still takes control
  back. `setState` only fires `onStateChanged` on a real value change, so
  `setPreset()` requests the retarget explicitly.
- `next()` at the last preset and `previous()` at the first do nothing at all
  while the table is in charge: no PID reset, no dwell restart, `atTarget()`
  does not flip back to false. If a manual voltage or a raw target *was* in
  charge, a clamped step still hands control back to the table, otherwise a
  latched manual voltage would keep driving.
- `setPreset()` restarts the loop on every call, which is what takes control
  back from a manual override. Do not call it every tick. `holdPreset()` is
  the idempotent version for commands that execute every frame, and it is what
  `makeStateCommand` / `makeStateUntilCommand` use.
- `moveTo(raw)` and `setManualVoltage(volts)` put the table on hold so it
  cannot pull the mechanism back. After `moveTo()` and `stop()` the reported
  preset snaps to the entry with the closest setpoint, so `getPreset()` is
  never stale and a following `next()` steps from somewhere sensible.
- `stop()` brakes, it does not coast: the present position becomes the target
  and the loop holds it. Pair it with `BrakeMode::Hold` on the motors so a
  raised mechanism cannot back-drive. Use `setManualVoltage(0.0)` to go limp.
- Neither copyable nor movable: the setpoint sink and the command factories
  capture `this`.

## AutoTriggerMechanism: auto-fire on a sensor edge

`AutoTriggerMechanism` watches a `std::function<bool()>` and calls a
`std::function<void()>` when it goes from false to true. It knows nothing
about what the sensor is or what the action does, so one implementation covers
auto-clamping on a goal, auto-indexing on a detected object, auto-retracting on
a limit switch, and auto-stopping on a proximity reading.

Composed with a `ToggleMechanism`, it replaces a hand-written game-specific
clamp subsystem:

```cpp
#include "mclib/mclib.hpp"

using namespace mclib;

// The thing being actuated: a solenoid, nothing game-specific about it.
mechanism::ToggleMechanism grabber(std::make_shared<device::Pneumatic>('H'));

device::Distance sensor(7);

// The sensor gate. Note what it returns for "nothing there": false, not
// "unknown". With no re-arm condition the latch clears only on a false
// reading, so a gate that swallows the absent case latches forever.
//
// confidence >= 10, not 50: PROS pins distance confidence at 10 for anything
// under 200 mm, so a higher floor discards every reading a close-range
// mechanism would ever see. 0 mm is a real reading, for a target inside the
// sensor's ~20 mm minimum range, so it is not an error. The error value is
// PROS_ERR, which is INT32_MAX, not a negative number: an `mm < 0` guard never
// fires and lets INT32_MAX through as a very large distance, which reads as
// true for any "far enough away" threshold.
auto goal_in_range = []() {
  const std::int32_t mm = sensor.getDistanceMm();
  const std::int32_t confidence = sensor.getConfidence();
  if (mm == PROS_ERR || confidence == PROS_ERR || confidence < 10) {
    return false;  // bad read: treat as "nothing there" so the latch clears
  }
  return mm <= 60;
};

mechanism::AutoTriggerConfig auto_grab_config{
    .debounce_ms = 60.0,        // the reading must hold for 60 ms
    .enabled_on_construct = true,
    .fire_once = false,         // keep watching after each grab
};

mechanism::AutoTriggerMechanism auto_grab(
    goal_in_range, []() { grabber.extend(); }, auto_grab_config);

void initialize() {
  grabber.setName("grabber");
  grabber.setDefaultCommand(grabber.idleCommand());
  grabber.registerSelf();

  auto_grab.setName("auto_grab");
  auto_grab.setDefaultCommand(auto_grab.idleCommand());
  auto_grab.registerSelf();

  // Re-arm when the grabber is open again. Without this the latch clears as
  // soon as the sensor stops seeing the target, which is fine for most
  // mechanisms but wrong here: the target is being carried, so it stays in
  // range the whole time.
  auto_grab.setRearmCondition([]() { return !grabber.isExtended(); });
}
```

`disarmUntilReset()` is the manual override. It suppresses the fire that has
not happened yet: the driver opens the grabber ahead of a target they do not
want picked up, and the watcher stays out of the way until the target has
passed and a fresh edge arrives.

```cpp
// Driver pre-emptively opens the grabber. Without disarmUntilReset() the
// watcher closes it again as soon as the target comes into range.
void skipThisOne() {
  grabber.retract();
  auto_grab.disarmUntilReset();
}
```

Releasing a target that is already held does not need it: the fire that closed
the grabber left the edge consumed, and the edge only comes back once the sensor
reads false, which it does not while the target is still in the clamp.

Rules the type follows:

- **Edge, not level.** The action fires on the false to true transition of the
  debounced condition, never on every tick the condition holds.
- **The latch survives a manual override.** After a fire (or a
  `disarmUntilReset()`), no further fire happens until the latch clears *and* a
  fresh false to true edge arrives. Clearing the latch while the condition is
  still true does not fire.
- **The latch clears on a false reading** when no re-arm condition was supplied,
  and only on the re-arm condition when one was.
- **`debounce_ms` needs an unbroken run.** One false reading restarts the
  window, so a flickering sensor never accumulates enough time.

Autonomous routines use `makeWaitForTriggerCommand(timeout_ms)`, which finishes
the moment the action fires or on timeout, and restores the armed state it found
unless something changed it while the command ran:

```cpp
// Drive forward until the sensor sees the target and the grabber closes.
auto grab_it = auto_grab.makeWaitForTriggerCommand(2000.0);
```

Two things to know about it:

- **It only arms a mechanism that was disarmed.** One that is already armed is
  left exactly as it is, latch and edge included, because arming clears the
  latch and that would reapply the action a driver just overrode with
  `disarmUntilReset()`.
- **The fire it waits for is not guaranteed.** Armed and latched, no re-arm
  condition, trigger condition stuck true: the latch only clears when the
  condition goes away, so nothing ever fires and the timeout is the only way
  out. That is why `timeout_ms` defaults to a finite
  `kDefaultWaitForTriggerTimeoutMs` (5 s) rather than to wait-forever. Pass `0`
  if you really do want to wait forever, and call `rearm()` first if clearing
  the latch is what you meant.

`makeArmCommand()` and `makeDisarmCommand()` are one-shot and finish
immediately. `setArmed()` / `isArmed()` are this mechanism's own switch and are
separate from `Subsystem::setEnabled()`, which gates `periodic()` for the whole
scheduler. `hasFired()`, `fireCount()`, `isLatched()` and `isConditionMet()`
report state; `reset()` clears everything but the armed flag.

`AutoTriggerMechanism` is neither copyable nor movable: the condition and action
callbacks routinely capture `this`, and moving the object would leave them
pointing at the old address.

## Mechanism copy/move, and PID holding output

Two rules that apply to every mechanism: they cannot be copied or moved, and
their PID can keep driving after it arrives.

### `StateMechanism` is explicitly non-copyable and non-movable

`StateMechanism<StateT>` deletes its copy constructor, copy assignment,
move constructor and move assignment.

Subclasses routinely store a `std::function` that captured `this` -
`MappedMechanism::ApplyState`, `MotorStateMechanism`'s voltage map, the
callbacks in the position, velocity, conveyor and PTO mechanisms. Copying or
moving such an object copies the callable but not what it points at, so the new
object's `periodic()` would drive real motors through a pointer to the old,
possibly destroyed, object. A mechanism is also an identity: the
`CommandScheduler` registers it by address and it owns its default command, so
a second copy was never meaningful.

Both operations were already blocked by accident: `Subsystem` holds a
`std::unique_ptr<Command>`, which implicitly deletes its copy constructor, and
declares a virtual destructor, which suppresses the implicit move constructor.
That is fragile (removing the `unique_ptr` member would silently re-enable
copying) and the compiler error pointed at `Subsystem`'s members instead of at
the rule. The explicit deletions pin the guarantee and make the diagnostic say
what is wrong.

Hold mechanisms in place: static or program-long storage, or
`std::vector<std::unique_ptr<T>>`. `std::vector<PositionMechanism>` does not
compile.

### `PID::setHoldOutput(bool)`

`PID::update()` used to return a hard `0` on every tick once `arrived` latched,
until `reset()` was called. A settled positional mechanism therefore had zero
holding torque and sagged under gravity.

`setHoldOutput(true)` keeps computing `P + I + D` after arrival. Arrival is
still detected and `targetArrived()` still latches exactly as before, so a
caller can use arrival as a "motion finished" signal while the loop holds the
setpoint.

What you get inside the settle band is a P (plus D) hold, not a zero-error
hold: arrival requires `|error| <= small_error_tolerance`, and `update()` zeroes
`sum_error` over that same band, so the integral is 0 on every held tick. A
loaded arm settles wherever `kp * error` balances the load, up to
`small_error_tolerance` of steady droop.

The default on a bare `PID` is `false`, the original behaviour.
`ChassisController` and the routines in `control/motion.cpp` end their motions
on `targetArrived()`, which works the same with hold on or off.

`PositionMechanismConfig::hold_output` turns it on for position mechanisms, and
defaults to `true` there.

Three useful configurations:

| Configuration | Behaviour | Use for |
| --- | --- | --- |
| `setArrive(true)`, hold off (default) | Move, latch, then output 0 | Chassis motion that ends on arrival |
| `setArrive(true)`, `setHoldOutput(true)` | Latch and report arrival, keep driving | Lift or arm that would otherwise sag |
| `setArrive(false)` | Never latch, never zero | Velocity control, where arrival is meaningless |

### The `small_error_tolerance` trap

`PID`'s default `small_error_tolerance` is `1`, and `update()` zeroes `sum_error` whenever
`|error| <= small_error_tolerance`. For any loop that operates inside an error
of 1 - velocity control in rpm, a position loop in revolutions - that silently
disables `ki` entirely. Call `setSmallBigErrorTolerance(0, 0)` for those loops
(and `setArrive(false)`, since the same tolerances drive arrival detection).

`pid.hpp` documents every method and the interaction between `arrive`,
`arrived`, `hold_output`, the two error tolerances and the two settle
durations.
