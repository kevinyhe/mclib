# Safety and lifecycle

What stops a motion, what the scheduler does when the field disables you, and what you must verify on your own robot.

[Documentation index](README.md) · [Project README](../README.md)

Blocking motions stop on timeout, cancellation, competition disable, or invalid
sensor readings, including when `exit=false`. Only successful chained motions
leave the drive energized. Run at most one blocking motion at a time.

`wallReset()` returns `true` only when sustained wall contact is detected and
the pose is reset. On failure it stops and returns `false`, leaving the pose
unchanged. Check that result before relying on the new position.

Call `CommandScheduler::disable()` from your competition `disabled()` callback
when the scheduler loop does not run while disabled (as in the example).
`run()` also handles disabled competition status. Active commands are interrupted,
queued commands are discarded, and subsystem `onDisabled()` hooks clear actuator
state. Commands do not resume automatically on enable; default commands may start
again. Custom actuator subsystems must implement an idempotent `onDisabled()`.
The scheduler is single-task code: do not call its methods concurrently.

Velocity and position mechanisms expose `hasSensorFault()`. Invalid feedback
sets their voltage to zero and clears their ready flag. Homing reports `Failed`
instead of treating invalid velocity as a hard stop. Verify these behaviors on
your robot with the wheels raised before running autonomous on the field.
