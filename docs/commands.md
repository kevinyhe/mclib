# Commands

mclib includes a command-based framework modeled on FRC's WPILib.

- A **subsystem** is one part of the robot, such as the drivetrain or a lift.
- A **command** is an action that uses one or more subsystems, such as "move the
  lift to 90°".
- A command's **requirements** are the subsystems it uses. Scheduling a command
  interrupts any running command with the same requirement, so two commands
  never drive the same motors.
- A subsystem's **default command** runs whenever no other command is using it.
- `CommandScheduler::run()` updates every command and subsystem. Call it every
  10 ms in `opcontrol()` and during autonomous.

## Registering subsystems

Give the subsystem its default command, then register it:

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

The subsystem owns its default command.

To keep ownership yourself, pass the command to the scheduler directly. It must
stay alive while registered:

```cpp
std::unique_ptr<Command> conveyor_idle;

void initialize() {
  conveyor_idle = conveyor.makeStopCommand();
  CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());
}
```

### Subsystem API

| Method | Description |
| --- | --- |
| `setDefaultCommand(std::unique_ptr<Command>)` | Takes ownership of the default command, replacing any previous one. |
| `getDefaultCommand()` | The default command, or `nullptr`. |
| `registerSelf()` | Registers with the scheduler using the stored default command. |
| `setName(std::string)` / `getName()` | Name for logging. |
| `setEnabled(bool)` / `isEnabled()` | A disabled subsystem skips `periodic()`. |
| `runPeriodic()` | Called by the scheduler. Override `periodic()` instead. |

Disabling a subsystem does not stop its motors. PROS motors keep their last
voltage, so command a safe state before disabling. A subsystem that integrates
sensor changes in `periodic()` (such as odometry in `ChassisController`) misses
the motion that happens while disabled.

`setDefaultCommand` can be called after registration; the old default is
cancelled first.

### Scheduler API

```cpp
CommandScheduler::registerSubsystem(&conveyor);                       // uses the stored default command
CommandScheduler::registerSubsystem(&conveyor, conveyor_idle.get());  // caller owned default command
CommandScheduler::unregisterSubsystem(&conveyor);                     // cancels its command, stops periodic()
```

Registering a null or already-registered subsystem does nothing.
`unregisterSubsystem` is safe on a subsystem that was never registered. A
subsystem removes itself from the scheduler when destroyed.

## Command ownership

Factories such as `makeIndexCommand()` return `std::unique_ptr<Command>`. The
scheduler stores a plain pointer and never deletes commands, so something in
your program must keep each command alive while it can run:

1. the subsystem, for default commands (`setDefaultCommand`)
2. a `MechanismManager` (see [Mechanisms](mechanisms.md#mechanismmanager))
3. a global `std::unique_ptr<Command>`

```cpp
std::unique_ptr<Command> index;

void opcontrol() {
  index = conveyor.makeIndexCommand(1500.0);
  index->schedule();
}
```

Scheduling a temporary leaves the scheduler with a dangling pointer:

```cpp
// BROKEN. The unique_ptr dies at the end of the statement, so the scheduler is
// left holding a dangling pointer.
conveyor.makeForwardCommand()->schedule();
```

To destroy a command early, call `CommandScheduler::endAndForget(cmd)` first.

## Combining commands

| Method | Result |
| --- | --- |
| `a.andThen(&b)` | runs `a`, then `b` |
| `a.with(&b)` | runs both; ends when both finish |
| `a.race(&b)` | runs both; ends when either finishes |
| `a.withTimeout(t)` | ends `a` after `t` |
| `a.until(condition)` | ends `a` when `condition` returns true |
| `a.repeatedly()` | restarts `a` each time it finishes |
| `a.asProxy()` | schedules `a` separately from the group it is in |

Each returns a new `std::unique_ptr<Command>`. Store it like any other command:

```cpp
std::unique_ptr<Command> lift_move;
std::unique_ptr<Command> timed_lift;

void initialize() {
  lift_move = lift.makeMoveToCommand(90.0);
  timed_lift = lift_move->withTimeout(2.0 * second);
}
```

The new command holds pointers to the commands it combines. Keep those alive
too, and store each step in its own variable:

```cpp
// Wrong: the sequence is destroyed at the end of the statement, and the
// timeout wrapper is left holding a dangling pointer to it.
auto bad = a.andThen(&b)->withTimeout(2.0 * second);

// Right.
std::unique_ptr<Command> sequence = a.andThen(&b);
std::unique_ptr<Command> timed = sequence->withTimeout(2.0 * second);
```

`Trigger::andOther()`, `orOther()` and `negate()` return
`std::unique_ptr<Trigger>` and hold pointers to the triggers they combine.
