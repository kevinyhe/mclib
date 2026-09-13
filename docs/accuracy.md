# Measured accuracy and known limits

What the library has been shown to do, what it has not, and which failures are
still open. Read this before you trust a number from `mclib` on a field.

[Documentation index](README.md) · [Project README](../README.md)

## What is verified

- **37 host test binaries** covering odometry, path generation, pure pursuit,
  motion profiles, the PID, every mechanism class, the command scheduler and
  the telemetry sinks. They run on any machine with `g++`; no brain and no ARM
  toolchain required. `make test`.
- **The same 37 under AddressSanitizer and UndefinedBehaviorSanitizer**, which
  is what catches leaks and undefined behaviour in the library itself.
- **An ARM firmware build** with `arm-none-eabi-g++`, so the code that runs on
  the brain is the code that was compiled, not a host approximation.
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
| Two-tracker physical variant | 67/80 | deadlines and settlement, drive-encoder distance loops, encoder heading under slip |
| Independent ideal motion matrix | 30/36 | two slow arc deadlines, four fast turn oscillation cases |
| Targeted completion, default gains | 24/27 | three fast endpoint-only turns |
| Targeted completion, tuned ideal plant | 27/27 | none, under ideal test conditions |

Safety behaviour passes in full: **40/40 safety checks** and **4/4 wall-reset
checks** in both physical matrices. A motion that cannot reach its target still
stops, still reports failure, and still leaves the drive de-energised.

## Why the arcs fail

Lateral slip is not observable through drive encoders. `driveTo()` and
`curveCircle()` close their distance loop around the outer drive wheel, so a
wheel that slips sideways reports travel that did not happen. Better pose
estimation does not fix it: with tracking wheels the *pose* can be within
0.14-0.32 inches of truth while the body still misses the endpoint by several
inches, because the thing being controlled is wheel travel, not position.

Two things would close that gap, and neither is in the library today:

1. A path-following controller that closes the loop on position rather than on
   an outer-wheel distance primitive.
2. Your robot's measured dynamics and tuned gains.

## What this means for your season

Tune against your own robot. The shipped gains are a starting point that passes
a simulated plant, not a calibration. Run every autonomous with the wheels
raised first, then at half speed on a field, and watch `atTarget()` and the
motion return values rather than assuming a move landed.
