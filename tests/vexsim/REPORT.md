# mclib physics validation — 2026-09-08

> Historical 64/80 run. See [FIX_VALIDATION.md](FIX_VALIDATION.md) for the corrected
> simulator, algorithm fixes, current 70/80 run and remaining acceptance failures.

> **Qualification after independent simulator audit:** these results describe
> this version of vexsim, not measured robot behavior. The subsequent physics
> audit reproduced specification mismatches and conservation/contact defects.
> The endpoint errors and visualizations below must not be used to calibrate
> a robot or certify tracking accuracy. See [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md)
> for the simulator gate and the separate algorithm-only validation.

## Outcome

The actual C++ blocking motion algorithms were run against vexsim, using their
existing default gains and a `DriveHardware` adapter. **64 of 80 scenarios met
the stated acceptance checks.** This does not yet establish that the autonomous
algorithms are ready for competition use on these drivetrains.

The previously fixed abort behavior passed every physics test: **40/40** for
timeout, cancellation, competition disable, invalid IMU readings, and invalid
encoder readings across all eight motion entry points. These are checks that
the controller stops commanding motion, not promises of zero physical stopping
distance. All cases were actually energized before the abort condition.

| Scenario group | Passed | Total |
| --- | ---: | ---: |
| Four-motor 200 RPM drivetrain | 4 | 11 |
| Six-motor 450 RPM drivetrain | 6 | 11 |
| Faster four-inch-wheel drivetrain | 8 | 11 |
| Battery, grip, encoder-heading checks | 3 | 3 |
| Motion aborts | 40 | 40 |
| Wall/stall handling | 3 | 4 |
| Total | 64 | 80 |

The existing mclib host suite also passed **36/36 test binaries**. The simulator's
own headless suite completed **200 tests, OK with one GUI test skipped**. Its
initial non-headless run had one environment error because `DISPLAY=:0` was set
but no display server was reachable; clearing the display variables resolved
that without modifying the simulator.

## New findings

### 1. `moveToPoint()` can stop its PID but never exit the motion

This is a control-flow issue, not merely an endpoint-accuracy threshold.
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
5.5-inch position check. The position error is not all controller error: the
table distinguishes the estimator's disagreement with ground truth. A tighter
pose-completion contract and better odometry need separate evaluation; simply
increasing a timeout cannot affect a routine that has already returned.

### 3. The default wall threshold is incompatible with default simulated motors

`wallReset()` requires **average current strictly greater than 2500 mA** by
default. vexsim's default V5 motor current limit is **2500 mA**, making that
strict threshold unreachable in this model. At the test's 6 V command, each
locked motor actually draws **2230.48 mA**.

With the default threshold the routine correctly times out without resetting
the pose. With an explicitly configured **2000 mA** threshold, the same locked
fixture detects the stall and resets the pose in **0.24 s**. The no-contact and
body-pinned/wheels-free fixtures both stop without applying a false pose reset.

This is a threshold/drive-power compatibility finding, not a defect in the
recently fixed failure-path handling. Real thresholds must be selected below
the achievable stall current for the chosen power/current limit, while still
rejecting ordinary load and startup current. A fully locked fixture is not a
model of arbitrary physical wall contact with slipping tires.

### 4. Default tuning and encoder-only position estimates need further validation

At the four-second acceptance deadline, the 200 RPM preset misses its forward
and reverse drive completion windows; each finishes approximately **1.39 in**
from its true target. That observation alone is not proof of a logic defect:
these are generic simulator presets using unretuned library gains.

All six nominal arc cases miss the four-second deadline and the faster presets
have substantial path error. With a ten-second allowance on the 450 RPM preset,
the forward and reverse arcs return at **5.58 s** and **5.91 s**, respectively,
but true endpoint errors remain **8.40 in** in each direction. On the faster
four-inch-wheel preset, the four-second arc endpoint is **12.64 in** from the
ideal circle target, with **8.22 in** of odometry error already present at return.

Those errors require evaluating wheel slip, speed/tuning, and tracking-wheel or
other position feedback. The current tests do not establish that the arc
geometry formulas themselves are wrong, nor that one set of gains will work
across these drivetrains.

## Reproduction and evidence

From the mclib root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py --output /tmp/mclib-vexsim-final
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py --filter six_motor_450/ --motion-timeout-ms 10000 --output /tmp/mclib-vexsim-long
make -j8 test
```

The first command currently exits 1 with 64/80 checks passing. The longer-timeout
run exits 1 with 6/11 passing. Those failures are retained rather than relabeled
as expected successes.

The artifacts from this run are:

- [/tmp/mclib-vexsim-final/results.json](/tmp/mclib-vexsim-final/results.json)
- [/tmp/mclib-vexsim-long/results.json](/tmp/mclib-vexsim-long/results.json)
- [/tmp/vexsim-headless-tests.log](/tmp/vexsim-headless-tests.log)
- [/tmp/mclib-vexsim-host-tests.log](/tmp/mclib-vexsim-host-tests.log)

Each scenario also has a physics CSV and a `_control.csv` trace beside its JSON
results. The latter records true/estimated position and requested motor modes
and voltages. Temporary artifacts are not durable across machine cleanup;
the harness and this report are retained in the repository.

## Scope and methodology

The existing graphify map located the simulator's step/sensor entry points; the
current source was then inspected directly. An independent read-only audit of
the bridge confirmed the coordinate signs, reciprocal gearing, motor voltage
conversion, brake mapping, and nominal arc targets. Its cautions about hard-stall
fixtures and synchronous odometry are reflected in this report and the harness
documentation.

There were **no production algorithm or tuning changes** during this physics
validation, and vexsim was left unchanged. See [README.md](README.md) for the
adapter assumptions, explicit tolerances, and exclusions. These runs do not
cover real PROS motor wrappers, threaded scheduler behavior, tracking-wheel
configurations, holonomic controllers, mechanisms, or path followers in physics.
