# mclib physics validation, 2026-09-08

> Historical 64/80 run. See [FIX_VALIDATION.md](FIX_VALIDATION.md) for the corrected
> simulator, algorithm fixes, current 70/80 run and remaining acceptance failures.

> These results describe this version of vexsim. They are not measurements of
> a robot. A later physics audit found places where vexsim did not match its
> specification and defects in its conservation and contact code. Do not use
> the endpoint errors or visualizations below to calibrate a robot or to claim
> tracking accuracy. See [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md)
> for the simulator gate and the separate algorithm-only validation.

## Outcome

The C++ blocking motion algorithms ran against vexsim with their default gains
and a `DriveHardware` adapter. 64 of 80 scenarios met the acceptance checks, so
the autonomous algorithms are not yet shown to be ready for competition on these
drivetrains.

The abort handling passed all 40 physics tests: timeout, cancellation,
competition disable, invalid IMU readings and invalid encoder readings, across
all eight motion entry points. Each test checks that the controller stops
commanding motion; the robot still takes some distance to stop. Every case had
the drive energized before the abort.

| Scenario group | Passed | Total |
| --- | ---: | ---: |
| Four-motor 200 RPM drivetrain | 4 | 11 |
| Six-motor 450 RPM drivetrain | 6 | 11 |
| Faster four-inch-wheel drivetrain | 8 | 11 |
| Battery, grip, encoder-heading checks | 3 | 3 |
| Motion aborts | 40 | 40 |
| Wall/stall handling | 3 | 4 |
| Total | 64 | 80 |

The mclib host suite passed 36/36 test binaries. The simulator's own headless
suite ran 200 tests with one GUI test skipped. Its
initial non-headless run had one environment error because `DISPLAY=:0` was set
but no display server was reachable; clearing the display variables resolved
that without modifying the simulator.

## New findings

### 1. `moveToPoint()` can stop its PID but never exit the motion

This is a control-flow bug, separate from any endpoint-accuracy threshold.
[`motion.cpp`](../../src/mclib/control/motion.cpp) configures distance-PID arrival
with a 1.5-inch big-error tolerance, but the outer point-move loop only exits on
the perpendicular-line test, cancellation, or timeout. It never checks
`pid_distance.targetArrived()`. Once the PID's arrival latch sets, its default
behavior is to output zero; the line test can still require further travel.

On the six-motor 450 RPM preset, `moveToPoint(0_in, 24_in, ...)` first commands
zero on both sides at **2.23 s**, with estimated Y **22.60 in**. It stays at zero
until the **4.00 s** timeout. A repeat with a **10.00 s** timeout produces the
same stationary endpoint and still times out. True final Y is **22.29 in**,
while the estimated endpoint is **22.61 in**. The diagonal point case also
continues to timeout at 10 seconds.

The arrival and outer-loop completion rules need to agree. Increasing the timeout
does not fix this case. Changing the behavior of successful chained moves needs
careful handling separately from stopped moves.

### 2. Boomerang's completion criterion permits a loose endpoint

The stopping branch in [`motion.cpp`](../../src/mclib/control/motion.cpp) can
exit once estimated distance is below **5 in** and 200 ms have elapsed. It does
not require a settled final-heading error. That criterion combines with drive
wheel slip in this simulation:

| Preset | Return time | True position error after holding | Heading error | Odometry error at return |
| --- | ---: | ---: | ---: | ---: |
| Six-motor 450 RPM | 2.05 s | 6.92 in | 4.20° | 2.54 in |
| Faster four-inch wheels | 1.80 s | 8.97 in | 7.73° | 5.41 in |

These motions returned before timeout, but failed even the harness's relaxed
5.5-inch position check. Part of the position error is odometry error, which the
last column separates out. A tighter completion rule and better odometry need
separate evaluation. A longer timeout cannot help a routine that already
returned.

### 3. The default wall threshold is incompatible with default simulated motors

`wallReset()` requires **average current strictly greater than 2500 mA** by
default. vexsim's default V5 motor current limit is **2500 mA**, so that
threshold can never be reached in this model. At the test's 6 V command, each
locked motor draws **2230.48 mA**.

With the default threshold the routine times out without resetting the pose,
as it should. With a **2000 mA** threshold set in the config, the same locked
fixture detects the stall and resets the pose in **0.24 s**. The no-contact and
body-pinned/wheels-free fixtures both stop without applying a false pose reset.

The failure-path handling works; the default threshold is set above what the
motors can draw. Pick a threshold below the stall current for your current
limit and above ordinary load and startup current. A fully locked fixture does
not model a robot pushing a wall with slipping tires.

### 4. Default tuning and encoder-only position estimates need further validation

At the four-second acceptance deadline, the 200 RPM preset misses its forward
and reverse drive completion windows; each finishes approximately **1.39 in**
from its true target. That alone does not show a logic defect, because these
are generic simulator presets running untuned library gains.

All six nominal arc cases miss the four-second deadline and the faster presets
have substantial path error. With a ten-second allowance on the 450 RPM preset,
the forward and reverse arcs return at **5.58 s** and **5.91 s**, respectively,
but true endpoint errors remain **8.40 in** in each direction. On the faster
four-inch-wheel preset, the four-second arc endpoint is **12.64 in** from the
ideal circle target, with **8.22 in** of odometry error already present at return.

Explaining those errors needs work on wheel slip, speed and tuning, and
tracking-wheel or other position feedback. The tests do not show the arc
geometry formulas are wrong, and they do not show one set of gains works across
these drivetrains.

## Reproduction and evidence

From the mclib root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py --output /tmp/mclib-vexsim-final
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py --filter six_motor_450/ --motion-timeout-ms 10000 --output /tmp/mclib-vexsim-long
make -j8 test
```

The first command exited 1 with 64/80 checks passing. The longer-timeout run
exited 1 with 6/11 passing. The failures stay reported as failures.

The run wrote these artifacts, which lived in `/tmp` and no longer exist:

- `/tmp/mclib-vexsim-final/results.json`
- `/tmp/mclib-vexsim-long/results.json`
- `/tmp/vexsim-headless-tests.log`
- `/tmp/mclib-vexsim-host-tests.log`

Each scenario also has a physics CSV and a `_control.csv` trace beside its JSON
results. The control trace records true and estimated position and the
requested motor modes and voltages. The harness and this report are in the
repository; the artifacts are not.

## Scope and methodology

A graphify map located the simulator's step and sensor entry points, and the
source was then read directly. An independent read-only audit of
the bridge confirmed the coordinate signs, reciprocal gearing, motor voltage
conversion, brake mapping, and nominal arc targets. Its cautions about hard-stall
fixtures and synchronous odometry are reflected in this report and the harness
documentation.

No production algorithm or tuning changed during this validation, and vexsim
was left unchanged. See [README.md](README.md) for the adapter assumptions,
tolerances and exclusions. These runs do not
cover real PROS motor wrappers, threaded scheduler behavior, tracking-wheel
configurations, holonomic controllers, mechanisms, or path followers in physics.
