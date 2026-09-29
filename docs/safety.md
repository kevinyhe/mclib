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

## Running a motion in the background

`mclib::control::AsyncMotion` runs a motion on its own task and returns at
once, so the routine can act partway through:

```cpp
using mclib::control::AsyncMotion;

AsyncMotion drive = AsyncMotion::start([] { return driveTo(30_in, 2_s); });
drive.waitUntilTravelled(12_in);   // straight-line distance from the start
intake.setVoltage(12_V);
if (drive.wait() != MotionResult::Reached) {
  return;
}
```

| Call | Returns |
| --- | --- |
| `wait()` | the `MotionResult`, once it finishes |
| `waitUntilElapsed(t)`, `waitUntilTravelled(d)`, `waitUntil(condition)` | `true` when the condition is met while the motion runs, `false` if the motion ended first |
| `cancel()` | `Cancelled`, after the motion has stopped and braked |
| `isRunning()`, `result()` | without blocking |

- Starting a new `AsyncMotion` cancels the running one and waits for it.
- Destroying the handle cancels its motion. Keep it until the motion is done.
- Cancelling uses `CancelToken::Motion`, like chassis commands: the motion
  stops at its next 10 ms tick. A motion that ignores it for 500 ms has its
  task killed.
- Don't run blocking motions or chassis commands at the same time. They share
  the drive and the cancel flag.

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
