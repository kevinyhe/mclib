# Simulator-first validation — 2026-09-08

> Historical pre-fix audit. The implementation changes and current verification
> are recorded in [FIX_VALIDATION.md](FIX_VALIDATION.md); findings below preserve
> the original evidence rather than describing the current source as unfixed.

## Verdict and evidence standard

**vexsim is not currently validated as a physically accurate VEX robot model.**
Independent checks reproduce conservation, specification, contact, timing, and
sensor-reset failures. Its existing test suite passes, but some tests encode the
implementation's assumptions rather than an independent physical reference.

The earlier [64/80 motion results](REPORT.md) and their visualizations are
observations of this simulator, not predictions with established hardware
accuracy. In particular, their inch-level endpoint errors do not prove that
mclib's odometry integration formula is wrong.

Validation was ordered in two stages: inspect and test the simulator first;
then test real C++ tracking/control against independent ideal motion. A failed
physics gate does not prevent testing arithmetic or completion logic, but it
does prevent certifying real-world tracking accuracy using that simulator.
No robot measurements were supplied for this audit. No production physics or
control algorithms were modified.

## Scope: what actually runs

| Component | Actual implementation | Validation scope |
| --- | --- | --- |
| Drivetrain | Python `motor`, `battery`, `chassis`, `wheels`, `dynamics`, `sim`; normally 1 ms | All motor models/cartridges, wheel families, presets, force/power equations, boundary cases and convergence |
| Sensors | Python encoders, rotation sensor, IMU, GPS, distance sensor | Units, quantization, sampling, seeded repeatability, limits, reset and geometry |
| Field/game | Custom Python `game`/`field`, normally 4 ms block substeps | Ballistics, contacts, momentum, reaction transfer and simplified mechanism scope |
| Interactive 3D | Python `viz3d` state with three.js 0.160 rendering | Clock, telemetry, coordinate/quaternion mapping and animation consistency |
| Rapier 0.20 | Offline `tools/bake_intake.mjs` prototype only | Bounded headless run; not the live robot physics engine |

The existing graphify graph was used read-only to locate relationships; current
source and reproducible experiments, not graph descriptions, determine findings.

## Baseline and independent checks

The simulator's own suite completed **200 tests: OK, one GUI test skipped**.
Clearing `DISPLAY`/`WAYLAND_DISPLAY` was necessary in this headless environment.
There were no baseline physics-test failures.

| Independent audit | Test methods/checks | Passed | Failed |
| --- | ---: | ---: | ---: |
| Electrical/thermal | 22 | 10 | 12 |
| Mechanical | 18 | 13 | 5 |
| Sensors/orchestration | 16 | 7 | 9 |
| Game/3D driver | 10 | 6 | 4 |
| Browser transforms/animation | 4 | 3 | 1 |

Electrical failures include 30 assertions/subtests across 12 failing methods.
Counts are diagnostic coverage, not independent bug counts or a physical-accuracy
percentage. Some failures identify deliberately simplified, unmodeled sensor
limits; others are strict conservation or geometry violations. None are hidden
as expected-failure passes. These standalone audit files are intentionally not
part of mclib's ordinary `make test` target.

### Electrical and motor findings

1. **Backdriven gearbox power is not conserved.** `motor.py:356` applies gearbox
   efficiency in the same direction for motoring and braking. A green motor at
   half free speed, shorted in BRAKE, absorbs 11.369691 W from its shaft but
   dissipates 12.498031 W copper plus 0.746365 W friction with zero bus input:
   a 1.874705 W deficit. External gearing has the same directional error at
   `chassis.py:88`: a test gearbox transmits 60 W upstream while absorbing only
   48 W at its wheel. These are independent of friction calibration.
2. **Loss/heat accounting is incomplete.** At the same positive half-speed
   operating point, 2.005165 W of internal gearbox loss is absent from the
   reported loss sum and lumped thermal model (`motor.py:366`, `:373`). Separately,
   yaw scrub removes 0.186808 W in an isolated chassis experiment while all
   mechanical loss outputs report zero (`dynamics.py:282`). Reported energy
   columns therefore cannot serve as a complete conservation ledger.
3. **The default 11 W model misses loaded-speed regulation.** At 30% of
   published stall torque, equilibrium speeds are 84.330/168.660/505.980 RPM,
   versus nominal 100/200/600. VEX documents maintaining nominal speed to about
   35% stall torque, including RPM restrictions in raw PWM mode. The simulator
   fits nominal RPM as the natural unloaded speed instead. This affects the
   presets used in the earlier C++ runs.
   [VEX motor performance](https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance).
4. **11 W thermal limits disagree with documented firmware.** The code uses
   80/60/40/20% current limits and a 75°C stop; documented levels are
   50/25/12.5/0%, with shutdown at 70°C. Its 65°C limit is 1 A rather than
   0.3125 A; at 70°C it still permits 0.5 A (`motor.py:252`). The exact 5.5 W
   threshold schedule was not established by the sources and is not asserted.
   [VEX performance](https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance),
   [VEX technical-support temperature thresholds](https://www.vexforum.com/t/v5-motor-current-limit-vs-temperature/107172),
   [VEX motor API](https://api.vex.com/v5/home/cpp/Motors_and_MotorControllers/motor_and_motor_group.html).
5. **The 5.5 W motor is overpowered and overspeed.** Solving the actual model
   produces 271.404 RPM, 0.800037 Nm stall, and 7.062481 W peak, versus published
   200 RPM, 0.5 Nm, and 5.5 W. It is a fixed-gearing motor, not a smaller motor
   that happens to use the same green cartridge (`motor.py:93`).
   [VEX 5.5 W specifications](https://kb.vex.com/hc/en-us/articles/10002101702932-Understanding-V5-Smart-Motor-5-5W-Performance).
6. **The 393 is calibrated at the wrong voltage.** At its manufacturer-specified
   7.2 V supply, the standard model gives 56.498 RPM, 0.876673 Nm and 2.88 A,
   instead of 100 RPM, 1.67 Nm and 4.8 A. The code inherits 12 V (`motor.py:110`),
   which makes its reporting properties look closer to the intended values.
   [Manufacturer-authored 393 datasheet](https://projectpartners.pltw.org/Training3/story_content/external_files/VEX_2_Wire_Motor_393.pdf).
7. **Current clipping breaks bridge constraints.** BRAKE at nominal green free
   speed reports -12.178750 W electrical power but zero bus draw. At three times
   free speed, active zero-voltage mode claims 28.0645 V across a 12 V supplied
   bridge. Open-circuit coast is excluded from this voltage test (`motor.py:333`).
8. **A depleted finite battery never becomes unavailable.** SOC clamps to zero
   but OCV remains 11.2 V (`battery.py:64`, `:88`). A robot starting at zero charge
   receives 5.928427 J in 100 ms. A direct 10 A load can consume another 10 Ah
   over an hour without shutdown. Finite stored charge cannot support this,
   regardless of the exact real pack cutoff voltage.
9. **Configured brain current budgets are not hard limits.** A stalled six-motor
   case configured for 4 A draws 7.128725 A on its first step; a zero budget
   still allows 0.055770 A because scaling has a 0.05 floor (`sim.py:188`). The
   default six-motor 20 A case does not exceed its budget in this test.
10. **Firmware controller timing is idealized incorrectly.** Motor PID runs
    every physics tick (1 ms), rather than the documented 10 ms (`sim.py:179`).
    This affects velocity, position and hold behavior, not direct-voltage PID
    calculations in mclib.
    [VEX firmware description](https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance).

Passing electrical checks include signed Ohm's-law/current-limit cases, PWM
power transfer where not clipped, torque symmetry, 11 W basic endpoints within
explicit tolerances, analytic RL response/composition, one inductance advance
per step despite solver retries, thermal RC response, battery coulomb counting,
and an independently solved battery quadratic. The default bus solution differs
from that quadratic by only 1.34 microvolts.

### Mechanical findings and convergence

11. **Normal-load solving can drop feasible moments.** The singular fallback
    assigns equal loads, and the active-set solver can discard supports needed
    by a feasible solution (`dynamics.py:109`). A two-contact case requiring
    54.03325/44.03325 N instead gets 49.03325/49.03325 N, giving zero roll moment
    rather than -2 Nm. An irregular, feasible four-contact case leaves a
    5.735536 Nm pitch residual. Ordinary static/moderately accelerating presets
    do balance in the passing checks.
12. **Rotating-frame integration adds finite-step energy.** A force-free body
    with forward speed 1 m/s and yaw rate 5 rad/s gains 2.53148% translational
    kinetic energy in one second at 1 ms. Gains reduce to 1.25784% at 0.5 ms
    and 0.50125% at 0.2 ms (`dynamics.py:320`). This is a convergent numerical
    error, not evidence that every default trajectory is unstable.

For an **open-loop** `speed_base` probe, constant 12 V left / 8 V right for three seconds,
reducing 1 ms to 0.2 ms changes the endpoint only **0.427 mm (0.0168 in)** and
heading about **0.140°**. Mirrored commands mirror the physical path. Large
lateral velocity persists (about 1.13 m/s at the end), even with a much shorter
tire relaxation length. In a milder 6 V / 3 V arc it is about 0.009 m/s.

This probe is not the earlier C++ `curveCircle` run: its refinement bound cannot
be transferred to that closed-loop controller's endpoint error. The open-loop
probe's strong drift is primarily a result of the chosen tire/traction model,
not a simple timestep artifact. This does **not** establish whether that
amount of slip is realistic on VRC foam. Friction coefficients, creep speed,
relaxation length, drag, inertia and load distribution need measurements.

Other passing checks cover all wheel families' steady dissipation/force bounds,
traction-wheel friction ellipse, steered/mecanum kinematics, reflected inertia,
constant-force acceleration, world-frame rotation covariance, and unpowered
coasting. Lagged tire forces sometimes return energy; because elastic contact
can do so, that alone was not classified as a proven active-force defect.
However, logging absolute slip work hides its sign and omits a stored-energy
state, so it is not a rigorous thermodynamic ledger.

The configured 4 N tracking-wheel preload produces **10.702 N per tracker** in
the static `odom_bot` preset. The solver treats these wheels as additional rigid
load-bearing supports. This is an important model/wording ambiguity, not a
proven violation if preload was intended only as an additive bias.

### Sensor and orchestration findings

13. **Reset leaves old sensor clocks and control phase.** `Simulator.reset()`
    does not reset GPS, distance or tracking-wheel sensor timestamps, nor
    `_control_accum` (`sim.py:404`). Resetting a run at X=1 m leaves GPS reporting
    X=1 m after the robot is reset to X=0. Updates resume only when the new clock
    catches the prior timestamp. A half-completed controller period survives
    reset as well. Fresh instances used by the earlier bridge avoid this bug.
14. **The IMU invents an initial integration interval.** With zero errors and a
    1 rad/s rate supplied at t=0, heading immediately advances 0.01 rad. At
    t=0.22 s it reports 0.23 rad (`sensors.py:110`). This is distinct from sensor
    noise and does not explain multi-inch lateral slip by itself.
15. **Distance rays hit infinite wall extensions.** With a box [-1,1]², a ray
    from (2,2) facing west misses the finite walls but returns a valid 1 m hit
    (`sensors.py:260`). The default core permits positions outside the field;
    containment is supplied by interactive drivers.
16. **Hardware sensor limits are not represented.** A 1 mm wall distance is
    reported valid although VEX specifies a 20–2000 mm operating range. The
    IMU accurately reports imposed 8 g and 2000°/s without saturation or invalid
    feedback despite documented 4 g / 1000°/s limits. The exact overloaded
    hardware response is unspecified; these tests flag missing limit behavior.
    They impose a conservative in-range-or-invalid acceptance policy, not a
    uniquely manufacturer-mandated clipping or invalid-flag implementation.
    [VEX distance specifications](https://api.vex.com/v5/home/blocks/sensing/distance-sensor.html),
    [VEX inertial specifications](https://kb.vex.com/hc/en-us/articles/360037382272-Using-the-Inertial-Sensor-with-VEX-V5).

Passing sensor checks cover unit round trips, encoder half-tick error bounds for
all cartridge resolutions and rotation sensors, sample-and-hold, repeated fresh
seeded instances, interior wall-ray geometry, maximum distance rejection, and
the default user-control clock. Noise/bias/GPS dropout distributions have no
measured calibration in this project. Software-reversed motor travel also needs
an explicit physical-versus-logical coordinate contract; it was not counted as
a proven defect because the builder describes that flag as modeling miswiring.

### Contact, game and visual findings

17. **Passive structure contact creates kinetic energy.** An ordinary chassis
    touching one off-center static post, with yaw motion and no motor step,
    gains **26.3% kinetic energy**: 0.095879496 → 0.121117256 J. The collision
    code adjusts translation and yaw independently rather than using a coupled
    effective-mass impulse (`game.py:1531`).
18. **Batched block reactions are lost.** A 4 ms block contact exposes -14.2 N;
    batching the same initial event into 8 ms exposes zero reaction, dropping
    -0.0568 Ns while the block still acquires 1.42 m/s (`game.py:792`). The driver
    advances the robot for an entire batch before stepping the game.
19. **The interactive clock discards fractional timesteps.** A controlled run
    with 100 wall-time intervals of 2.9 ms advances only 200 ms of simulation
    rather than 290 ms, because each loop truncates its step count without
    retaining the remainder (`viz3d.py:119`). This does not affect the earlier
    directly stepped C++ physics bridge.
20. **The dashboard labels winding current as battery current.** At a locked
    6 V command, it shows 13.38 A / 190.1 W instead of the pack's
    5.654179 A / 80.297398 W (`viz3d.py:264`, `:285`). Signed sums can also cancel
    during turns. The core battery fields, not this panel, are the appropriate
    electrical measurements.
21. **Wheel animation continues during pause.** The actual page function moves
    a 100 RPM wheel by 1.047197551 rad over 0.1 s even when paused
    (`push_back.html:1301`). The sampled robot/block coordinate and 90° yaw
    quaternion transforms pass; arbitrary pitch/roll/composed rotations were
    not exhaustively tested.

Passing contact checks include airborne ballistics within the timestep error
bound, free spin/unit quaternions, isolated equal-mass momentum/restitution,
wall reflection and tangential motion. CAD hinge and roller-direction probes
also pass. These do not validate every mesh contact or empirical restitution.

The offline Rapier intake prototype was run for eight simulated seconds with
output intercepted in memory: 27 rollers, eight blocks, 481 frames, zero blocks
scored. It stalls around five inches up the tower, consistent with the project's
own tools README. The live page does not use that baked simulation.

Important scope limits are intentional: no chassis vertical/pitch/roll dynamics
or suspension, no robot-versus-robot model, no field collision geometry in the
core drivetrain, and no dynamically coupled intake/lift mass or battery load.
Game contact support is simplified; carried blocks use a one-dimensional route
with explicit coordinate placement, loaders project stacks, and park platforms
add drag without raising the chassis. The 3D presentation does not remove these
limitations.

## Stage 2: tracking/control without the disputed physics

These tests ran after the physics gate failed. They compile actual mclib code
and use independently specified motion oracles. No gains or algorithms were
changed to compensate for the simulator.

| Algorithm check | Result | Meaning |
| --- | --- | --- |
| Odometry numerical oracle | 6,845 assertions, 1 failure | Fixed-layout integration passes; live layout replacement has a boundary bug |
| Pure Pursuit geometry/ideal trajectories | 72,007 assertions, 4 failures | All 15 nominal trajectory runs pass; two raw-sample path issues |
| Blocking motions on ideal differential drive | 27/36 scenarios pass | Confirms point completion deadlock; other failures depend on ideal plant/default gains |
| Existing mclib host suite | 36/36 binaries pass | Existing host checks still pass |

### Odometry

`tracking_analytic_audit.cpp` derives world and sensor-contact velocities and
integrates them with independent Simpson quadrature (2,048 intervals for the
high-accuracy reference). It never calls mclib's chord/arc or frame helpers.
The reference includes gearing, signed mounting offsets and sensor baselines.

- 2,160 randomized constant-twist scenarios across zero, one and two tracking
  wheels: worst position difference **1.933e-12 inches**.
- 108 signed multi-turn/offset cases: worst difference **1.575e-12 inches**.
- Changing curvature: errors decrease from approximately 0.01781 to 0.004456,
  0.001114 and 0.0001783 inches as sampling improves from 100 to 50, 25 and
  10 ms, showing expected discretization convergence.
- A 12-inch lateral slide is invisible to zero/one-tracker layouts and captured
  by a horizontal tracker: errors 12/12/0 inches. A 50% drive-encoder overcount
  over 20 inches gives errors 10/0/0 inches. Those are sensor observability
  limits, not integrator arithmetic errors.
- Constant-curvature dropout recovery passes. Changing motion order during
  missing samples is not recoverable from totals alone: two paths can produce
  identical recovered readings with endpoints 28.284 inches apart. The audit
  demonstrates this instead of promising exact dropout recovery.

**New boundary finding:** `Odometry::setConfig()` only replaces configuration
(`src/mclib/control/odometry.cpp:39`). If disabled tracker fields in the previous
sample contained NaN, enabling those trackers subtracts those ignored NaNs from
the next finite sample. Pose becomes `(NaN, NaN, 0)` with zero reported faults.
A sensor-layout change needs a fresh baseline; normal `startOdometry` setup
already resets afterward. This does not explain earlier fixed-layout arc drift.

### Pure Pursuit

`pursuit_kinematic_audit.cpp` drives an ideal body from actual follower wheel
outputs. Its exact held-command solution is cross-checked against 20-substep
RK4 (worst difference 3.78e-14 inches). Another 105 checks compare curvature
against `2*x_local/(x_local²+y_local²)`, with worst error 1.39e-16 per inch.

All 15 nominal trajectory runs finish in 1.51–4.43 seconds with 0.451–0.499-inch
endpoint error under an explicitly configured **0.5-inch** finish tolerance.
Coverage includes north/east straights, mirrored circles, an L polyline, an
analytical S bend, generated spline, off-path/before-start recovery and a
backward-facing start. Progress bounds, reset/path replacement, finite wheel
limits and signed wheel-speed conversion also pass.

Two findings concern **raw `Path(std::vector<PathPoint>)` construction**, not
normal waypoint/spline builders, which remove duplicates:

1. Three identical `(0,0)` samples are accepted as valid despite zero route
   length. From `(30,-30,0)`, the follower commands 4.8/7.2 in/s wheel speeds
   (6 in/s centre speed) instead of treating the route as degenerate.
   See `include/mclib/path/path.hpp:119`, `src/mclib/path/path.cpp:71`, and
   `src/mclib/path/pure_pursuit.cpp:370`, `:398`.
2. Raw `(0,0),(0,0),(24,0)` gives signed cross-track error zero at
   `(0,3,pi/2)`, rather than -3 inches. The initial zero-length segment retains
   a default heading and a tied projection cannot replace it
   (`pure_pursuit.cpp:148`, `:190`). This reproduction affects telemetry;
   lookahead still matches the deduplicated route.

There is no reverse-route mode; backward-facing recovery turns to follow
forward. Also, `finished` explicitly means route progress, not guaranteed
endpoint arrival: its documented off-path/past-end case can finish 30.017 inches
from the endpoint. Neither behavior is counted as a new defect.

### Blocking motions

`kinematic_motion_audit.py` runs actual `motion.cpp` with perfect encoder/heading
feedback and exact constant-wheel arcs. It imports no vexsim physics and has no
slip, lag, inertia, battery or noise. Wheel speed is proportional to voltage,
using declared speed scales 34/76/94 in/s at 12 V, 11.5-inch track width,
3.25-inch wheels and ratio 1. Stops halt the ideal wheels immediately. Names
borrowed from presets identify speed scales, not physical recreations of those
robots. Coverage is 11 nominal motions at each scale plus three 10-second
forward-point probes.

**The point-move completion bug is independent of vexsim.** At the slow scale,
`moveToPoint(0,24)` reaches Y=22.843438 inches, then commands zero from 2.930 s
through its 4-second timeout. A 10-second deadline leaves the identical endpoint
with 7.070 seconds of zero output. Estimated and ideal true positions agree;
remaining error is 1.156562 inches. The slow diagonal case also deadlocks.

The distance PID can latch arrival within 1.5 inches while the outer loop still
requires the 1-inch geometric exit line (`src/mclib/control/motion.cpp:1035`).
The outer loop does not exit when that PID latches zero. Faster-scale probes
cross the exit line first and pass; that does not invalidate the counterexample.

Boomerang returns **4.948–4.985 inches short even with perfect tracking**, with
2.25–3.87° heading error. This agrees with its roughly 5-inch positional exit
rule and explains a controller component of earlier misses without attributing
all slip to the controller. These runs pass the earlier harness's loose
5.5-inch criterion, not a precision-arrival test.

Other ideal-plant failures are slow arc convergence or saturated turn oscillation
under unchanged default gains. Instantaneous voltage-to-speed response is not
motor physics, so these are **not** claimed as further hardware controller bugs.
All 36 cases stop outputs. Their maximum odometry-versus-ideal-position gap is
2.554e-13 inches; the separate quadrature audit is the stronger independent
test of the integration formula.

### Remaining hardware-validation gate

Fix the simulator's conservation, limits, contact and reset defects first. Then
calibrate geometry/mass/inertia, motor curves and braking, battery sag, tire
behavior, and sensor latency/noise against measured robot runs. Hold some runs
out of calibration; compare forward/reverse straights, turns and arcs across
speeds, charge levels and loads. Only afterward use the C++ closed-loop matrix
for physical tracking acceptance and before/after visual claims.

These algorithm checks do not cover real PROS scheduling, sensor-driver latency,
tracking-wheel lift/contact mechanics, V5 firmware execution or hardware
calibration. Holonomic controllers and other mechanisms retain existing host
coverage, not new physical certification from this harness.

## Reproduction and retained artifacts

From mclib, using the installed Python/Node and existing locked JS dependencies:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_electrical_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_mechanical_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_mechanical_audit.py --diagnostics
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_sensor_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_game_audit.py
node tests/vexsim/physics_game_audit.mjs
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/kinematic_motion_audit.py
make -j8 test
```

Standalone algorithm builds, from mclib:

```sh
g++ -std=gnu++20 -O2 -DMCLIB_HOST_BUILD -Iinclude -pthread \
  tests/vexsim/tracking_analytic_audit.cpp src/mclib/control/odometry.cpp \
  src/mclib/control/robot_state.cpp src/mclib/math.cpp \
  -o /tmp/mclib-tracking-analytic-audit-20260908
/tmp/mclib-tracking-analytic-audit-20260908

g++ -std=gnu++20 -O1 -g -DMCLIB_HOST_BUILD -Iinclude -Itests -pthread \
  tests/vexsim/pursuit_kinematic_audit.cpp src/mclib/math.cpp \
  src/mclib/path/path.cpp src/mclib/path/pure_pursuit.cpp \
  src/mclib/path/spline.cpp -o /tmp/mclib-pursuit-audit-20260908
/tmp/mclib-pursuit-audit-20260908
```

The optional `node tests/vexsim/physics_game_audit.mjs --bake` runs the existing
Rapier prototype without writing its bake output. Normal audit commands return
nonzero for the unresolved failures above.

Root-agent rerun logs are `/tmp/vexsim-{electrical,mechanical,sensor,game,browser}-audit-20260908.log`.
The baseline is `/tmp/vexsim-physics-baseline-20260908.log` and extended mechanics
are `/tmp/vexsim-mechanical-audit-toHJY9/diagnostics.json`. Source fingerprints
are `/tmp/vexsim-audit-source-sha256-20260908.txt`; audited simulator source
remained unchanged. There were no dependency installations, firmware builds,
commits or simulator output/cache writes. Temporary evidence can disappear when
the environment clears `/tmp`; scripts and this report remain in mclib.

Root algorithm logs are `/tmp/mclib-tracking-analytic-audit-20260908.log`,
`/tmp/mclib-pursuit-audit-20260908.log`, and
`/tmp/mclib-kinematic-motion-20260908.log`; motion traces/results/build outputs
are under `/tmp/mclib-kinematic-motion-20260908/`. Host-test output is
`/tmp/mclib-post-physics-host-tests-20260908.log`; the normal ignored `bin/tests`
build artifacts were reused/refreshed. Pure Pursuit was also run with ASan/UBSan:
the same four assertions fail with no additional sanitizer errors. Leak checking
was disabled because LeakSanitizer cannot operate under this environment's
ptrace, so leaks remain unverified. The user's 66 staged paths were preserved.
