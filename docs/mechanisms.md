# Mechanisms

Every mechanism is a [subsystem](commands.md). Its `periodic()` writes the motor
or pneumatic output on each `CommandScheduler::run()`, so a mechanism does
nothing until it is registered with the scheduler.

Mechanisms take their sensors and motors as functions (lambdas), so one class
works with any device. Capture devices that live as long as the mechanism, such
as globals. A lambda that captures a local variable by reference breaks once
that variable goes out of scope.

| Class | Use for |
| --- | --- |
| [`PositionMechanism`](#positionmechanism) | arms and lifts held at an angle or height |
| [`VelocityMechanism`](#velocitymechanism) | flywheels and rollers at a set RPM |
| [`ToggleMechanism`](#togglemechanism) | one or more pneumatics that move together |
| [`ToggleGroupMechanism`](#togglegroupmechanism) | several pneumatics that move independently |
| [`PneumaticSubsystem`](#pneumaticsubsystem) | pneumatics with a hold command |
| [`MultiPositionMechanism`](#multipositionmechanism) | a fixed list of setpoints |
| [`PresetPositionMechanism`](#presetpositionmechanism) | named positions with a PID loop |
| [`DiscreteActuatorMechanism`](#discreteactuatormechanism) | several cylinders whose combination sets a position |
| [`ConveyorMechanism`](#conveyormechanism) | intakes and conveyors with a sensor and jam recovery |
| [`HomingMechanism`](#homingmechanism) | finding zero by driving into a hard stop |
| [`PTOMechanism`](#ptomechanism) | motors shared between the drive and another mechanism |
| [`AutoTriggerMechanism`](#autotriggermechanism) | running an action when a sensor turns on |
| [`MechanismManager`](#mechanismmanager) | owning and registering many mechanisms |

## PositionMechanism

`mechanism/position_mechanism.hpp`. Holds a position with a PID loop.

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

| Method | Behavior |
| --- | --- |
| `moveTo(target)` | starts the PID toward `target`, and takes back control from manual voltage |
| `setManualVoltage(volts)` | drives at a fixed voltage until the next `moveTo()` |
| `stop()` | holds the current position |
| `setManualVoltage(0.0)` | lets the mechanism go limp |

A new mechanism outputs 0 V until one of these is called.

```cpp
lift.makeMoveToCommand(90.0);            // finishes on atTarget()
lift.makeMoveToCommand(90.0, 1500.0);    // also finishes after 1.5 s
lift.makeHoldCommand();                  // hold the current target
lift.makeManualCommand(6.0);             // 6 V while scheduled
lift.makeStopCommand();                  // hold where it is
```

Only `makeMoveToCommand` finishes on its own.

### Holding

With `hold_output = true` (the default), the PID keeps driving after it reaches
the target, so a loaded arm holds its position. `atTarget()` becomes true as
soon as the target is reached either way.

The hold uses P and D only. Within `small_error` of the target the integral is
zeroed, so a loaded arm can droop by up to `small_error`. Lower `small_error`
to reduce the droop.

With `hold_output = false`, output drops to 0 V on arrival and the loop restarts
only after the position drifts outside `small_error`.

## VelocityMechanism

`mechanism/velocity_mechanism.hpp`. Holds a speed, for flywheels and rollers.

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

Output is `kv * target` plus PID correction. `kv` is the voltage per RPM needed
to hold a speed (feedforward); the PID corrects the remaining error. Output
never has the opposite sign of the target, so the mechanism coasts down instead
of braking in reverse. The integral only builds within `integral_range_rpm` of
the target.

### Gear ratio

`config.ratio` is output RPM per motor RPM:

```
output_rpm = motor_rpm * ratio
motor_rpm  = output_rpm / ratio
```

A 1:2 speed-up is `ratio = 2.0`. A 2:1 reduction is `ratio = 0.5`. The default
is `1.0`.

| Value | Measured at |
| --- | --- |
| `setTargetRpm`, `getTargetRpm`, `getCurrentRpm` | output |
| `makeSpinCommand`, `makeSpinUpCommand` | output |
| `tolerance_rpm`, `integral_range_rpm` | output |
| the velocity source function, `getCurrentMotorRpm()` | motor |
| `kp`, `ki`, `kd`, `kv` | volts per motor RPM |

`kv` depends only on the motor, so it does not change with gearing. Retune
`kp`, `ki` and `kd` after changing gearing.

Example with a 600 RPM motor geared 1:2:

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

With the motor at 280 RPM, the wheel is at 560 RPM:

- `getCurrentRpm()` returns `560.0`; `getCurrentMotorRpm()` returns `280.0`
- PID error is `300 - 280 = 20` motor RPM, so P gives `0.02 * 20 = 0.4 V`
- feedforward is `kv * 300 = 6.0 V`
- output is `6.4 V`

`atSpeed()` is checked in output RPM: a 6 motor RPM error is 12 wheel RPM, which
is outside `tolerance_rpm = 10`.

`ratio` must be finite and greater than zero. Other values fall back to `1.0`,
which `getConfig().ratio` reports.

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

`mechanism/toggle_mechanism.hpp`. A mechanism with two states, extended and
retracted. It works with one pneumatic, several pneumatics moving together, or
any function that takes a `bool`.

`isExtended()` always reports the logical state, independent of wiring.

```cpp
#include "mclib/mclib.hpp"

using mclib::mechanism::ToggleMechanism;
using mclib::mechanism::ToggleMechanismConfig;

// One solenoid that extends when the port is low. Set wiring on the device,
// not with ToggleMechanismConfig::inverted.
ToggleMechanism clamp(std::make_shared<mclib::device::Pneumatic>('A', false, false));

// Several pneumatics driven together.
ToggleMechanism wings(std::vector<std::shared_ptr<mclib::device::Pneumatic>>{
    std::make_shared<mclib::device::Pneumatic>('E'),
    std::make_shared<mclib::device::Pneumatic>('F'),
});

// Any function. Use `inverted` only for functions like this.
ToggleMechanism motor_latch([](bool value) { printf("%d\n", value); },
                            ToggleMechanismConfig{
                                .initial_extended = false,
                                .inverted = true,
                            });
```

`set()`, `extend()`, `retract()` and `toggle()` write the output immediately, so
they work before the scheduler runs. Once registered, `periodic()` writes it
every tick.

All command factories finish on their own:

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
`makeToggleCommand` act once. The `...ForCommand(QTime)` variants hold a state
for a duration, then finish and stay in that state.

## ToggleGroupMechanism

`mechanism/toggle_group_mechanism.hpp`. Several pneumatics in one subsystem,
each with its own state. Channels are indexed by number or by an enum.

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

All command factories finish on their own. Timed variants stay in their final
state.

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

The default command must be `idleCommand()`. The scheduler restarts the default
command as soon as another command finishes, so a default that sets a state
would undo every button press. `periodic()` keeps writing the last states.

Per-channel inversion applies to function outputs only:

```cpp
bool left_raw = false;
bool right_raw = false;

// Channel 1's actuator is wired backwards, so it gets the negated value while
// get(1) still answers "is it extended".
mclib::mechanism::ToggleGroupMechanism raw_sides(
    {[](bool v) { left_raw = v; }, [](bool v) { right_raw = v; }},
    {.initial_states = {false, true}, .inverted = {false, true}});
```

Do not set `inverted` for a `device::Pneumatic` channel. For a cylinder plumbed
in reverse, use `Pneumatic(port, false, false)`.

Out-of-range indices are ignored and read as `false`. The number of channels is
fixed by the number of outputs passed to the constructor; use `setAll()` and
the factories above to change state.

## PneumaticSubsystem

`mechanism/pneumatic_subsystem.hpp`. `true` means extended. Wiring polarity is
set on the device.

```cpp
// Normal solenoid: pin high extends.
mclib::mechanism::PneumaticSubsystem wings({'E', 'F'});

// Inverted solenoid: pin low extends. Starts extended.
mclib::mechanism::PneumaticSubsystem clamp({'G'}, /*initial_extended=*/true,
                                           /*extended_state=*/false);

clamp.isExtended();  // true, for either wiring, and from construction on
```

The second constructor argument is the starting state, applied immediately.

| Command | Finishes |
| --- | --- |
| `makeSetCommand`, `makeExtendCommand`, `makeRetractCommand`, `makeToggleCommand` | immediately |
| `makeExtendForCommand(500 * millisecond)` | after the duration |
| `makeHoldCommand(extended)` | never |

Use `idleCommand()` as the default command, not `makeHoldCommand`.

`count()` returns the number of solenoids, `group()` the device group, and
`rawValues()` each solenoid's state. Writes through `group()` are overwritten
on the next `periodic()`.

## MultiPositionMechanism

`mechanism/multi_position_mechanism.hpp`. Moves between a fixed list of
setpoints. Each tick it passes the current setpoint to a function you provide,
such as a PID target.

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

Use `idleCommand()` as the default. `makePositionCommand()` never finishes, so as
a default it would undo every `makeNextCommand()`.

- `next()` and `previous()` stop at the ends of the table.
- `cycle()` wraps from the last entry to the first.
- Table order defines "next".
- A state missing from the table sends the default setpoint (`0.0` unless given
  to the constructor). `currentIndex()` returns `kNoPosition` for it.
- `next()`, `previous()` and `cycle()` from a missing state go to the first
  entry.
- List each state once; later duplicates are ignored.

## PresetPositionMechanism

`mechanism/preset_position_mechanism.hpp`. Named positions (from
`MultiPositionMechanism`) driven by a PID loop (from `PositionMechanism`).

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

Register only the `PresetPositionMechanism`. `controller()` returns the inner
`PositionMechanism` for tuning; do not register it.

| Group | Methods |
| --- | --- |
| Presets | `setPreset(preset)`, `getPreset()`, `next()`, `previous()`, `cycle()`, `presetSetpoint()` |
| Position | `moveTo(raw)`, `positionValue()`, `targetValue()`, `atTarget()` |
| Manual | `isManual()`, `setManualVoltage(volts)`, `stop()` |

| Command | Finishes |
| --- | --- |
| `makePresetCommand(preset, timeout_ms = 0.0)`, `makeMoveToCommand(raw, timeout_ms = 0.0)` | at target or on timeout; holds if interrupted |
| `makeNextCommand()`, `makePreviousCommand()`, `makeCycleCommand()` | immediately |
| `makeManualCommand(volts)`, `makeStopCommand()` | when interrupted |

Behavior:

- A new mechanism outputs 0 V until the first `setPreset()`, step, `moveTo()`,
  `setManualVoltage()` or `stop()`.
- `setPreset()` restarts the PID on every call. Use `holdPreset()` from commands
  that run every tick.
- `next()` at the last preset and `previous()` at the first do nothing, unless
  manual voltage or `moveTo()` is in control, in which case they return control
  to the preset.
- After `moveTo()` or `stop()`, `getPreset()` reports the preset closest to the
  target.
- `stop()` holds the current position. Use `BrakeMode::Hold` on the motors as
  well. `setManualVoltage(0.0)` goes limp.

## DiscreteActuatorMechanism

`mechanism/discrete_actuator_mechanism.hpp`. Several on/off outputs whose
combination selects a named state, e.g. two cylinders giving three angles.

```cpp
enum class Tilt { Flat, Angled, Steep };

// Two cylinders, three angles. Add a row to add an angle.
mclib::mechanism::DiscreteActuatorMechanism<Tilt> tilt(
    Tilt::Flat,
    {{Tilt::Flat,   {false, false}},
     {Tilt::Angled, {true,  false}},
     {Tilt::Steep,  {true,  true}}},
    std::vector<std::shared_ptr<mclib::device::Pneumatic>>{front, back});

void initialize() {
  tilt.setName("tilt");
  // Must be idleCommand(); a default that sets a state undoes every command.
  tilt.setDefaultCommand(tilt.idleCommand());
  tilt.registerSelf();
}
```

Table values are logical (`true` = extended). Set reverse plumbing on the
`device::Pneumatic`, not with `inverted`.

`next()` and `previous()` stop at the ends; `cycle()` wraps.

All commands finish on their own: `makeDiscreteStateCommand(state)`,
`makeNextCommand()`, `makePreviousCommand()`, `makeCycleCommand()` and
`makeDiscreteStateForCommand(state, 500 * millisecond)`.

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

Rows with the wrong number of values are dropped at construction;
`droppedRowCount()` reports how many. A state not in the table is never applied,
and `setDiscreteState()` refuses it.

`makeDiscreteStateForCommand` applies its state on the next `periodic()`, so it
has no effect while the subsystem is disabled.

Accessors: `stateCount()`, `actuatorCount()`, `droppedRowCount()`, `empty()`,
`indexOf(state)`, `currentIndex()`, `stateAt(index)`, `isDecodable(state)`,
`combinationFor(state)`, `currentCombination()`, `rawValue(index, value)`.

## ConveyorMechanism

`mechanism/conveyor_mechanism.hpp`. Runs intakes and conveyors. It can stop when
a sensor detects an object and reverses briefly when jammed.

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

```cpp
conveyor.makeForwardCommand();
conveyor.makeReverseCommand();
conveyor.makeStopCommand();
conveyor.makeIndexCommand(1500.0);  // finishes when the gate latches, or on timeout
conveyor.makeStateCommand(mclib::mechanism::ConveyorState::Reverse);
```

Only `makeIndexCommand` finishes on its own.

States:

| State | Behavior |
| --- | --- |
| `Forward` | runs at `forward_voltage` |
| `Reverse` | runs at `reverse_voltage` |
| `Stopped` | 0 V |
| `IndexToSensor` | runs at `index_voltage` until the sensor reads true, then holds at 0 V |

Without a sensor function, `IndexToSensor` behaves like `Stopped`, `hasObject()`
is always false, and `makeIndexCommand` finishes immediately.

### Jam recovery

A jam is current above `jam_current_amps` with speed below `jam_velocity_rpm` for
`jam_dwell_ms`. The conveyor then runs at `unjam_voltage` for `unjam_ms` and
returns to its previous state. After `max_unjam_retries` jams in a row it stops,
and `isJammed()` stays true until the state changes or `clearJam()` is called.
The retry count resets after `jam_clear_ms` of normal running.

Reverse ports (negative numbers) for motors that face opposite directions.
Jam detection uses the average speed of the group.

### Two-stage conveyors

All motors in a `ConveyorMechanism` run at one voltage. For stages that run
separately, use one `ConveyorMechanism` per stage:

```cpp
mclib::mechanism::ConveyorMechanism bottom(
    {.motor_ports = {-20}}, [] { return line_sensor.get(); });
mclib::mechanism::ConveyorMechanism top({.motor_ports = {-21}});

// ParallelCommandGroup stores raw Command*, so the stage commands must outlive
// the group.
std::unique_ptr<Command> bottom_index;
std::unique_ptr<Command> bottom_forward;
std::unique_ptr<Command> top_forward;
std::unique_ptr<Command> score;

void initialize() {
  bottom.setName("conveyor_bottom");
  bottom.setDefaultCommand(bottom.makeStopCommand());
  bottom.registerSelf();

  top.setName("conveyor_top");
  top.setDefaultCommand(top.makeStopCommand());
  top.registerSelf();

  // Bottom stage only, stopping at the sensor.
  bottom_index =
      bottom.makeStateCommand(mclib::mechanism::ConveyorState::IndexToSensor);

  // Both stages.
  bottom_forward = bottom.makeForwardCommand();
  top_forward = top.makeForwardCommand();
  score = std::make_unique<ParallelCommandGroup>(
      std::initializer_list<Command*>{bottom_forward.get(), top_forward.get()});
}

void opcontrol() {
  bottom_index->schedule();
  score->schedule();
}
```

Change stages through commands. The stop default commands reset the state every
tick, so a direct `setConveyorState` call is undone unless a command is running.

## HomingMechanism

`mechanism/homing_mechanism.hpp`. Finds a mechanism's zero by driving into a
hard stop, then resets the position sensor.

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

The first detector to trigger stops homing:

- the limit switch function returns true
- current rises above `current_threshold_amps`
- speed stays below `velocity_threshold_rpm` for `stall_dwell_ms`

Current is ignored for the first `startup_grace_ms`. The sensor is zeroed at the
stop, before backing off.

Call `startHoming()`, `cancelHoming()`, `isHoming()`, `isHomed()` and
`hasFailed()` directly, or use a command:

```cpp
CommandScheduler::registerSubsystem(&homing, nullptr);

auto home_arm = homing.makeHomeCommand(4000 * millisecond);
home_arm->schedule();
while (home_arm->scheduled()) {
  pros::delay(10);
}
```

`makeHomeCommand` always finishes (on success, failure or timeout) and stops the
motor when it ends. It works whether or not the mechanism is registered.

## PTOMechanism

`mechanism/pto_mechanism.hpp`. A power take-off (PTO) shifts one set of motors
between two uses, usually the drivetrain and a lift. `true` means engaged.

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
  // Must be idleCommand(); a default that sets a state undoes every shift.
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

`driveEngaged()` and `driveDisengaged()` return `true` if the write was
accepted. Writes from the side that is not connected are dropped. Accepted
voltage is applied on the next `periodic()`.

`engage()`, `disengage()` and `toggle()` brake the motors, switch the solenoid,
and block all drive writes for `shift_settle_time` (default 250 ms).
`isShiftSettled()` and `remainingSettleTime()` report progress. A 0 V command
brakes.

Write every tick. If no write arrives for `drive_timeout` (default 100 ms), the
output drops to 0 V. Set `drive_timeout` to `0 * millisecond` to disable this.

`motors()` returns the shared `device::MotorGroup` for brake modes, encoders and
temperatures. Voltage written through it is overwritten on the next
`periodic()`. `motorsFor(engaged_side)` returns the group only if that side is
connected and the shift has settled, and `nullptr` otherwise. Call it every tick
instead of storing the result. When built from an existing `MotorGroup`, the
drivetrain must also write through `driveDisengaged()`.

| Command | Finishes |
| --- | --- |
| `makeEngageCommand()`, `makeDisengageCommand()`, `makeToggleCommand()` | immediately |
| `makeShiftCommand(engaged)` | when the shift has settled |

## AutoTriggerMechanism

`mechanism/auto_trigger_mechanism.hpp`. Calls an action when a condition changes
from false to true, e.g. clamp a goal when a distance sensor sees it.

```cpp
#include "mclib/mclib.hpp"

using namespace mclib;

// The grabber solenoid.
mechanism::ToggleMechanism grabber(std::make_shared<device::Pneumatic>('H'));

device::Distance sensor(7);

// Returns false for "nothing there" and for bad readings, so the latch can
// clear.
//
// PROS reports confidence 10 for anything under 200 mm, so use >= 10.
// 0 mm is a valid reading. Errors are PROS_ERR (INT32_MAX), not negative.
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

  // Re-arm when the grabber opens. The carried goal stays in front of the
  // sensor, so the condition alone would never clear the latch.
  auto_grab.setRearmCondition([]() { return !grabber.isExtended(); });
}
```

Rules:

- **Edges only.** The action runs when the condition turns true, not while it
  stays true.
- **Latch.** After the action runs, it will not run again until the latch clears
  and the condition turns true again.
- **Clearing the latch.** Without a re-arm condition, the latch clears when the
  condition reads false. With one, it clears when the re-arm condition is true.
- **Debounce.** The condition must stay true for `debounce_ms`. Any false reading
  restarts the timer.

`disarmUntilReset()` skips the next trigger, e.g. when the driver opens the
grabber for a goal they do not want:

```cpp
// Open the grabber and skip the next goal.
void skipThisOne() {
  grabber.retract();
  auto_grab.disarmUntilReset();
}
```

In autonomous, `makeWaitForTriggerCommand(timeout_ms)` finishes when the action
runs or on timeout:

```cpp
// Drive forward until the sensor sees the target and the grabber closes.
auto grab_it = auto_grab.makeWaitForTriggerCommand(2000.0);
```

- It arms a disarmed mechanism and restores the previous state afterward. An
  already-armed mechanism is left unchanged.
- If the mechanism is latched with the condition stuck true, the action cannot
  run and the command times out. The default timeout is
  `kDefaultWaitForTriggerTimeoutMs` (5 s); `0` waits forever. Call `rearm()`
  first to clear the latch.

`makeArmCommand()` and `makeDisarmCommand()` finish immediately. `setArmed()` /
`isArmed()` are separate from `Subsystem::setEnabled()`. `hasFired()`,
`fireCount()`, `isLatched()` and `isConditionMet()` report state; `reset()`
clears everything except the armed flag.

## MechanismManager

`mechanism/mechanism_manager.hpp`. Owns mechanisms' default commands and
registers them together.

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

Keep the manager global or static.

`add()` returns `false` and stores nothing if the subsystem or command is null,
the subsystem is already added, or the name is taken. Names default to
`mechanism0`, `mechanism1`, and so on. `registerAll()` skips entries already
registered and returns the number it registered.

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

`describe()` prints the name, enabled and registered flags, and which command
holds each subsystem:

```
flywheel enabled=yes registered=yes requirement=default
wings enabled=yes registered=yes requirement=other
```

`setEnabled(name, false)` only prevents a later `registerAll()` from registering
that entry.

Register each subsystem once: either add it to a manager or call
`CommandScheduler::registerSubsystem` for it, not both.

## Copy and move

Mechanisms cannot be copied or moved: the scheduler and their callbacks refer to
them by address. Store them as globals, statics or
`std::vector<std::unique_ptr<T>>`.

## PID hold

`PID::setHoldOutput(true)` keeps the PID driving after it reaches the target.
`targetArrived()` still reports arrival.

| Configuration | Behavior | Use for |
| --- | --- | --- |
| `setArrive(true)`, hold off (default) | outputs 0 after arriving | drive motions |
| `setArrive(true)`, `setHoldOutput(true)` | reports arrival and keeps driving | lifts and arms |
| `setArrive(false)` | never stops | velocity control |

`PositionMechanismConfig::hold_output` sets this for position mechanisms and
defaults to `true`.

`PID` zeroes its integral whenever `|error| <= small_error_tolerance`, which
defaults to `1`. For loops whose error is usually below 1 (RPM, revolutions),
call `setSmallBigErrorTolerance(0, 0)` and `setArrive(false)`, or `ki` does
nothing.
