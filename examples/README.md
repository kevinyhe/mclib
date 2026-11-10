# Examples

Each file is a whole robot program: its own devices, `initialize()`,
`autonomous()` and `opcontrol()`. To try one, copy it over `src/main.cpp` in a
PROS project that has mclib installed, then change the ports and the numbers
to match your robot. Nothing in this folder is compiled into the library.

`make examples` compiles every file against the library headers with
warnings as errors, and CI runs it. That is the only check: none of these has
run on a robot. The gains, distances and voltages in them are placeholders.

## Driving and knowing where the robot is

| File | Shows |
| --- | --- |
| [tracking_wheels.cpp](tracking_wheels.cpp) | every odometry layout: drive encoders only, a forward wheel, a sideways wheel, both, or your own sensor readers. Wheel sizes and offsets, checking that odometry started, setting and reading the pose |
| [driver_control.cpp](driver_control.cpp) | every way to drive in `opcontrol()`: stick shaping, arcade on two sticks or one, tank, curvature, mixing the sticks yourself, commands that borrow the drive, brake modes, and the same robot with no `ChassisController` |
| [pose_fusion.cpp](pose_fusion.cpp) | correcting odometry from other sensors: distance sensors and GPS blended in while driving, a one-off fix from distance sensors while standing still, and a wall reset |
| [holonomic_drive.cpp](holonomic_drive.cpp) | an X-drive or mecanum robot in driver control and in autonomous |

## Autonomous motion

| File | Shows |
| --- | --- |
| [motions.cpp](motions.cpp) | every blocking motion, one use per routine: `driveTo`, turns, `moveToPoint`, `boomerang`, swings, arcs, `wallReset`, with time limits, voltage limits and chaining |
| [trajectories.cpp](trajectories.cpp) | paths and the ways to follow them: splines, arcs, speed limits and speed zones, `followTrajectory()` with RAMSETE, pure pursuit, motion profiles, feedforward and the routines that measure it |
| [background_motion_and_safety.cpp](background_motion_and_safety.cpp) | a motion running in the background, acting partway through it, ending it early, the cancel requests, what `disabled()` must do, sensor faults, and the autonomous time budget |
| [routines_and_selector.cpp](routines_and_selector.cpp) | `Routine` step by step with every step and option, commands that run beside the steps, a routine exported from the path planner, the selector, and saving the pick to the SD card |

## Mechanisms

| File | Shows |
| --- | --- |
| [lift_and_arm.cpp](lift_and_arm.cpp) | mechanisms that go to a position and hold it: a lift on a PID loop zeroed against its stop, an arm with named angles, a wrist on a PID of your own |
| [flywheel_and_intake.cpp](flywheel_and_intake.cpp) | mechanisms that spin: a flywheel held at a speed, rollers with a voltage for each state, a conveyor that stops at a sensor and clears its own jams, and watchers that act when a sensor turns on |
| [pneumatics_and_pto.cpp](pneumatics_and_pto.cpp) | mechanisms with two or a few fixed positions, and motors shared between two jobs: cylinders one at a time, together or in combinations, and two power take-offs |
| [mechanism_manager.cpp](mechanism_manager.cpp) | a whole robot's mechanisms kept together: your own mechanism classes, mechanisms built from a table of ports, one `MechanismManager` for the default commands, used from buttons and from a routine |

## Commands, devices and logging

| File | Shows |
| --- | --- |
| [commands.cpp](commands.cpp) | the command framework: subsystems, every kind of command, the two ways to combine commands, triggers and event loops |
| [devices.cpp](devices.cpp) | every device wrapper, and what to do when a device stops answering |
| [telemetry.cpp](telemetry.cpp) | logging a mechanism to a CSV file on the SD card, streaming columns over USB, and the clock |
| [units_and_math.cpp](units_and_math.cpp) | typed units and their literals, the field frame and the pose helpers, a PID controller used by itself, and a mutex shared between two tasks |

## Autonomous routines you can run

The path planner has 18 example routines that run in the physics simulator:
8 on the Override field with the cascade robot, and 10 on an open floor that
show one motion or setting at a time. Its **Export C++** writes any of them
as a function for an `AutonSelector`. See the
[planner guide](../tests/vexsim/planner/README.md#examples).
