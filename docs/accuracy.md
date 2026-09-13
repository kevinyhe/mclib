# Measured accuracy and known limits

What the library has been shown to do, what it has not, and which failures are
still open. Read this before you trust a number from `mclib` on a field.

[Documentation index](README.md) · [Project README](../README.md)

## What is verified

- **37 host test binaries** covering odometry, path generation, pure pursuit,
  motion profiles, the PID, every mechanism class, the command scheduler and
  the telemetry sinks. They run on any machine with `g++`: `make test`.
- **The same 37 under AddressSanitizer and UndefinedBehaviorSanitizer.** These
  catch memory leaks and undefined behaviour in the library.
- **An ARM firmware build** with `arm-none-eabi-g++`, the compiler that builds
  the code for the brain.
- **A physics simulator** replaying the real C++ controller and odometry. See
  [the simulator docs](simulator.md).

## What is not verified

Every number below comes from simulation. None of it is a measurement of a real
robot, and simulation cannot produce one. Gains, wheel sizes, tracking-wheel
geometry and surface friction are yours to measure.

## Open failures at default gains

From [the validation findings](../tests/vexsim/BUILDER_VALIDATION.md), with a
four-second deadline per move:

| Matrix | Passing | What is still failing |
| --- | --- | --- |
| Physical drive-encoder, default gains | 70/80 | three boomerangs, six arcs, one six-motor diagonal point move |
| Two-tracker physical variant | 67/80 | six arcs, three encoder-distance drives, two boomerangs, one four-motor point deadline, the encoder-heading stress case |
| Ideal plant with no inertia, default gains | 28/36 | two slow arc deadlines, four fast turn cases, two fast boomerangs |
| Targeted completion, default gains | 24/27 | three fast endpoint-only turns |
| Targeted completion, tuned ideal plant | 27/27 | none, under ideal test conditions |

All 40 safety checks and all 4 wall-reset checks pass in both physical
matrices. A motion that cannot reach its target still
stops, still reports failure, and still leaves the drive de-energised.

## Why the arcs fail

Lateral slip is not observable through drive encoders. `driveTo()` and
`curveCircle()` close their distance loop around the outer drive wheel, so a
wheel that slips sideways reports travel that did not happen. Tracking wheels
do not fix this. With them, the arcs' odometry error is 0.055-0.090 inches
while the robot misses the endpoint by 5.970-15.473 inches, because the loop
controls wheel travel and wheel travel is what slips.

Two things would close that gap, and neither is in the library today:

1. A path-following controller that closes the loop on position rather than on
   an outer-wheel distance primitive.
2. Your robot's measured dynamics and tuned gains.

## Before your first match

Tune against your own robot; the shipped gains were tuned in simulation. Run
every autonomous with the wheels raised first, then at half speed on a field.
Check `atTarget()` and the motion return values after each move.
