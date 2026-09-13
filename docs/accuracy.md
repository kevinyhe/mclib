# Known limits

## Testing

| Test | Scope |
| --- | --- |
| `make test` | 37 host test programs: odometry, paths, pure pursuit, profiles, PID, mechanisms, scheduler, telemetry |
| AddressSanitizer and UndefinedBehaviorSanitizer | the same 37 programs, checking for memory errors and leaks |
| ARM build | the firmware compiles with `arm-none-eabi-g++` |
| Physics simulator | the C++ motion routines and odometry against a simulated robot ([Simulator](simulator.md)) |

None of these measure a real robot. The default gains were tuned in simulation.
Tune them on your robot.

## Simulator results at default gains

Each move has a 4 s limit. See
[BUILDER_VALIDATION.md](../tests/vexsim/BUILDER_VALIDATION.md) for details.

| Test set | Passing | Failing |
| --- | --- | --- |
| Drive encoders only | 70/80 | 3 boomerangs, 6 arcs, 1 six-motor diagonal point move |
| Two tracking wheels | 67/80 | 6 arcs, 3 encoder-distance drives, 2 boomerangs, 1 four-motor point move timeout, the encoder-heading stress test |
| Ideal robot (no slip or inertia) | 28/36 | 2 slow arc timeouts, 4 fast turns, 2 fast boomerangs |
| Completion tests | 24/27 | 3 fast turns with no drive distance |
| Completion tests, tuned gains | 27/27 | none |

All 40 safety tests and all 4 wall-reset tests pass in both physical test sets.

## Arcs

`driveTo()` and `curveCircle()` control distance using the drive wheel encoders.
When a wheel slips sideways, the encoder still counts travel. Tracking wheels
improve the pose estimate (0.055-0.090 in error on arcs) but not the motion
itself, which still misses by 5.970-15.473 in.

Accurate arcs need a controller that steers using the robot's position, plus
gains tuned on the robot.

## Before a match

1. Run each autonomous with the wheels off the ground.
2. Run it at half speed on a field.
3. Check `atTarget()` and each motion's return value.
