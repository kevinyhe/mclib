# Commands and subsystems

The command scheduler, subsystem registration, and who owns a command.

[Documentation index](README.md) · [Project README](../README.md)

## Subsystem lifecycle

Command factories return `std::unique_ptr<Command>`, but the scheduler stores raw
`Command*`. That mismatch is easy to get wrong:

```cpp
// BROKEN. The unique_ptr dies at the end of the statement, so the scheduler is
// left holding a dangling pointer.
conveyor.makeForwardCommand()->schedule();
```

The subsystem itself is the owner. Hand it the default command with
`setDefaultCommand` and register with `registerSelf`:

```cpp
PositionMechanism arm{...};
ConveyorMechanism conveyor{{-20, -21}};

void initialize() {
  conveyor.setName("conveyor");
  conveyor.setDefaultCommand(conveyor.makeStopCommand());
  conveyor.registerSelf();

  arm.setName("arm");
  arm.setDefaultCommand(arm.makeStopCommand());
  arm.registerSelf();
}
```

You do not need a global `std::unique_ptr` for it. The subsystem keeps the
default command alive for as long as the subsystem is alive.

The older two-argument form still works, and is still the right tool when the
default command must live somewhere other than the subsystem. You keep ownership,
so the command has to outlive the registration:

```cpp
std::unique_ptr<Command> conveyor_idle;

void initialize() {
  conveyor_idle = conveyor.makeStopCommand();
  CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());
}
```

Commands that are not defaults still need an owner. Store the `unique_ptr`
somewhere that outlives the scheduling, then schedule the raw pointer:

```cpp
std::unique_ptr<Command> index;

void opcontrol() {
  index = conveyor.makeIndexCommand(1500.0);
  index->schedule();
}
```

### Subsystem API

| Method | What it does |
| --- | --- |
| `setDefaultCommand(std::unique_ptr<Command>)` | Take ownership of the default command. Destroys any previous one. |
| `getDefaultCommand()` | Non owning `Command*`, or `nullptr` if none was set. |
| `registerSelf()` | Register with the `CommandScheduler` using the stored default command. |
| `setName(std::string)` / `getName()` | Human readable name, useful for logging. |
| `setEnabled(bool)` / `isEnabled()` | A disabled subsystem skips `periodic()`. It does not stop the hardware, see below. |
| `runPeriodic()` | Non virtual. Called by the scheduler, checks `isEnabled()` and then calls the virtual `periodic()`. Override `periodic()`, not this. |

`setEnabled(false)` parks a subsystem without unregistering it. Commands can still
be scheduled against it, they just have no effect until it is enabled again.
Because `runPeriodic()` is non virtual and does the check, this works for
subclasses that override `periodic()`, such as `StateMechanism` and
`ChassisController`.

It does not stop the hardware. PROS motors hold the last voltage they were
given, so a disabled `StateMechanism` keeps driving at whatever `applyState`
last wrote. Command a safe state first, then disable. And a `periodic()` that
integrates sensor deltas, like `ChassisController` updating odometry, misses
everything that happens while disabled and folds it into one step when
re-enabled, which corrupts the pose.

### Scheduler API

```cpp
CommandScheduler::registerSubsystem(&conveyor);                       // uses the stored default command
CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());  // caller owned default command
CommandScheduler::unregisterSubsystem(&conveyor);                     // cancels its command, stops periodic()
```

Registering a null subsystem, or one that is already registered, is a no-op
rather than an assertion failure. Asserts compile out in release builds, so they
were not a real guard.

`unregisterSubsystem` cancels whatever command currently requires the subsystem
and its default command, drops its requirement entry, and stops `runPeriodic()`
from being called on it. It is safe to call on a subsystem that was never
registered. `~Subsystem` does the same cleanup automatically, minus the `end()`
callbacks, so a subsystem that goes out of scope cannot leave the scheduler
holding dangling pointers.

`setDefaultCommand` is also safe to call after registration: the old default
command is cancelled and scrubbed from the scheduler before it is destroyed, and
the registration is repointed at the new one.

## Who owns a command

Every command is heap-allocated and the scheduler stores raw `Command*`. It
never deletes anything. So something in your code has to own each command and
outlive the scheduler's use of it.

There are three ways to own one, in order of preference:

1. **Give it to the subsystem.** `subsystem.setDefaultCommand(cmd)` takes
   ownership of the default command. Nothing else to do.
2. **Give it to a `MechanismManager`.** It owns the default commands of every
   mechanism you add to it. See [Mechanisms](mechanisms.md#mechanismmanager).
3. **Hold the `std::unique_ptr` yourself**, in storage that lives as long as the
   program. A file-scope `std::unique_ptr<Command>` works, as in the `index`
   example above.

Do not hold it in a local `unique_ptr` inside a function. The command dies at
the closing brace while the scheduler still points at it.

### Decorators

`andThen()`, `with()`, `race()`, `withTimeout()`, `until()`, `repeatedly()` and
`asProxy()` build a new command that wraps the one you called them on. Each
returns a `std::unique_ptr<Command>`, so the result is yours to own:

```cpp
std::unique_ptr<Command> lift_move;
std::unique_ptr<Command> timed_lift;

void initialize() {
  lift_move = lift.makeMoveToCommand(90.0);
  timed_lift = lift_move->withTimeout(2.0 * second);
}
```

They are `[[nodiscard]]`, so discarding the result is a compiler warning.
Before 0.1.0 it was a silent leak.

Two rules:

- **The wrapper borrows what you hand it.** `a.andThen(&b)` does not adopt `b`.
  Both `a` and `b` have to outlive the sequence. The only exception is the
  helper a decorator builds for itself - the `WaitCommand` inside
  `withTimeout()`, the `WaitUntilCommand` inside `until()` - which the wrapper
  owns and destroys.
- **Do not chain off a temporary.** Name each intermediate:

```cpp
// Wrong: the sequence is destroyed at the end of the statement, and the
// timeout wrapper is left holding a dangling pointer to it.
auto bad = a.andThen(&b)->withTimeout(2.0 * second);

// Right.
std::unique_ptr<Command> sequence = a.andThen(&b);
std::unique_ptr<Command> timed = sequence->withTimeout(2.0 * second);
```

`Trigger::andOther()`, `orOther()` and `negate()` work the same way: they return
an owned `std::unique_ptr<Trigger>` that captures the source triggers by
pointer, so the sources must outlive it.

### Before you schedule

A command must be owned before it is scheduled, and it must stay alive until the
scheduler is done with it. To destroy one early, first take it back with
`CommandScheduler::endAndForget(cmd)`.
