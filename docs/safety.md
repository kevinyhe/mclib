# Safety

Blocking motions stop on timeout, cancellation, competition disable or an
invalid sensor reading, including when `exit=false`. Only a successful chained
motion leaves the drive energized. Run one blocking motion at a time.

Each motion returns a `mclib::control::MotionResult` that says how it ended:

| Result | Meaning |
| --- | --- |
| `Reached` | settled on the target, or crossed it when chained |
| `TimedOut` | `time_limit` ran out first, including during the stop ramp |
| `Cancelled` | `requestCancel(CancelToken::Motion)` was called |
| `Disabled` | the robot was disabled |
| `InvalidValue` | a parameter, sensor reading or output was NaN or infinite, or `time_limit` was not positive |
| `NoDrive` | no drive is bound |

```cpp
using mclib::control::MotionResult;

if (driveTo(24_in, 1500_ms) != MotionResult::Reached) {
  return;  // skip the rest of the routine
}
```

`toString()` gives the name for logs.

`wallReset()` returns `true` only after sustained wall contact, and then resets
the pose. On failure it stops, returns `false` and leaves the pose unchanged.

## Competition disable

Call `CommandScheduler::disable()` from `disabled()` if the scheduler loop does
not run while disabled. `run()` also handles the disabled state.

On disable:

- active commands are interrupted
- queued commands are discarded
- each subsystem's `onDisabled()` clears actuator state

Commands do not resume on enable. Default commands may start again. Custom
subsystems that drive motors or pneumatics must implement `onDisabled()`, and it
must be safe to call more than once.

The scheduler is single-task. Do not call it from more than one task.

## Sensor faults

Velocity and position mechanisms expose `hasSensorFault()`. Invalid feedback
sets their output to 0 V and clears their ready flag. Homing reports `Failed`
on invalid velocity.

Test on your robot with the wheels raised before running an autonomous.
