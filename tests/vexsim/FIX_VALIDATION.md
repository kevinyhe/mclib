# Physics and algorithm fixes — 2026-09-08

This is the post-fix result. [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md)
preserves the original audit and its reproductions; [REPORT.md](REPORT.md)
preserves the earlier 64/80 motion run. Neither historical result describes
the corrected implementation.

## What was fixed

Simulator changes cover all 21 numbered findings from the original audit:

| Area | Corrections |
| --- | --- |
| Motors/electrical | Direction-aware internal and external gearbox efficiency; reported gearbox heat; corrected 11 W loaded-speed envelope and thermal schedule; fixed-gearing 5.5 W equivalent; 7.2 V 393 fit; feasible bridge voltage and consistent signed winding/pack power |
| Supply/firmware | Empty-pack cutoff; charge-limited discharge; explicit rejected-regeneration heat at full charge; converged bus/current-budget solve including zero budget; 10 ms internal PID with immediate cancellation of the previous voltage on stop |
| Mechanics | Nonnegative support-load solve that retains feasible moments, including redundant constraints; explicit infeasible support diagnostics; world-frame velocity integration; yaw scrub loss accounting |
| Sensors | Complete reset of timestamps, values, seeded noise streams and control phase; no invented first IMU interval; finite wall-ray segments; distance operating range; explicit IMU overload policy |
| Game/display | Coupled passive static-contact impulses; two-body block/robot reactions applied once at contact; batch-averaged reaction telemetry; fractional wall-clock accumulator; interleaved game/robot ticks; correct battery telemetry; paused wheels stay still |

The final motor/tire model needed smaller internal integration steps: the
three-second high-speed arc differed by 2.193 mm between 1 ms and 0.2 ms,
exceeding the existing 2 mm audit budget. `max_physics_dt=0.0005` now bounds
internal steps while preserving the public 1 ms tick, user-control cadence and
logging frequency. The unchanged refinement test passes. No tire coefficients
were adjusted to improve its result.

The electrical-to-shaft energy report now includes motor/internal-gear friction
and external-gear losses and samples shaft work at the same integration stage
as torque. Chassis drag includes yaw scrub; its separately reported scrub
channel must not be added twice. Finite-step inductance and tire elastic
storage are still not a complete thermodynamic ledger.

mclib changes:

- `Odometry::setConfig` re-seeds when layout/geometry changes, preserving pose;
  identical configuration preserves the baseline. Ignored NaN tracker fields
  cannot contaminate a newly enabled layout.
- Raw `Path` construction removes consecutive duplicate positions, retaining
  the first sample's metadata. Degenerate paths stop and initial projection
  reports the correct signed cross-track error.
- Stopped `moveToPoint` honors PID settlement; chained moves do not latch
  their distance output off while waiting for a crossing. Nearby sideways or
  rearward targets retain steering instead of disabling it inside eight inches.
- Stopped `boomerang` settles actual endpoint distance, then final heading,
  reacquiring position if necessary. It no longer succeeds five inches short.
  Final alignment uses symmetric yaw commands, avoiding translation from the
  directional drive slew limiter. Chained approach direction also handles reverse.
- `wallReset` accepts equality at its current threshold, making a 2500 mA
  threshold reachable at a 2500 mA current limit. Voltage, thermal derating and
  telemetry still determine whether that threshold is appropriate on hardware.

## Independent checks and algorithm results

| Check | Post-fix result |
| --- | --- |
| Electrical audit | 22/22 test methods pass |
| Mechanical audit | 18/18 pass |
| Sensor/orchestration audit | 16/16 pass |
| Contact/driver audit | 10/10 pass |
| Browser checks | 4/4 pass |
| New simulator regressions | 31/31 pass |
| Complete simulator suite (includes those regressions) | 231 tests, OK; one GUI test skipped headlessly |
| mclib ordinary host suite | All 36 test binaries pass |
| Independent odometry oracle | All 6,845 assertions pass |
| Independent pursuit oracle | All 72,007 assertions pass |
| Targeted completion tests, explicitly tuned ideal plant | 27/27 pass |
| Targeted completion tests, unchanged default gains | 24/27 pass |
| Original ideal-motion matrix, unchanged default gains | 30/36 pass |
| Corrected-physics motion matrix, unchanged default gains | 70/80 pass, including 40/40 safety and 4/4 wall-reset cases |

The 31 simulator regressions include 840 feasible support configurations,
250 randomized passive contacts, sensor reset/range checks, electrical limits,
stop transitions, substep equivalence and electrical-to-shaft work accounting.
A separate peer probe checked 3,000 signed motor operating points and 300
bus/SOC/current-budget combinations; maximum observed power residual was
4.3e-10 W. These are coverage counts, not an accuracy percentage.

The new completion audit tests near-side, near-diagonal, behind, reverse and
endpoint-only-turn cases at 34/76/94 inches per second. With explicitly stated
ideal-plant yaw tuning, position error stays below 1.163 inches, heading error
below 0.876 degrees, and endpoint-only turns introduce zero translation.
The three default-gain failures at 94 in/s reproduce the same oscillation in
the existing standalone turn routine. No production gains were changed or
failing default-gain cases relabeled as passes.

The original ideal matrix's six remaining failures are two slow-plant arc
deadlines and four fast-plant turn/turn-to-point oscillation cases. The corrected
physics matrix's ten failures are all three boomerangs, six forward/reverse
arcs, and the six-motor diagonal point move. Endpoint position checks include
300 ms of post-return holding. Boomerang acceptance was tightened from the old
5.5 inches to the configured 1.5-inch position band; arc acceptance was not
relaxed.

For example, the six-motor boomerang returns with estimated position
`(22.698, 24.023)` inches for target `(24,24)`, but true position near
`(18.530, 28.758)`: 6.309 inches of odometry error and 7.251 inches of final
target error. The speed-base curved runs reach about 10.16 inches of odometry
error. Drive encoders plus heading cannot observe lateral slip; the independent
odometry oracle demonstrates that limitation. These failures remain visible,
not suppressed or fixed by feeding ground truth into the controller.

## Test-oracle changes

Six old simulator fixtures assumed behavior corrected by the audit. They now
test applied PWM voltage instead of commanded voltage under current limiting;
the published 11 W thermal steps; 10 ms firmware timing; exhausted charge
instead of assumed speed loss at 5% SOC; a governor-bounded feedforward speed;
and the actual driver's interleaved intake cadence while retaining both
nine-block capacity assertions.

The independent battery quadratic now derives stall power from the selected
motor's declared resistance/current limit instead of hardcoding the old
resistance. The contact audit measures actual body impulse after replacing the
old external-force transport; batch/split impulse equality is still required.
Paused-wheel checks compare unchanged quaternions directly, avoiding `acos`
roundoff on identical orientations. No expected-failure annotations were added.

## Remaining physical-fidelity limits

Passing the physics gate establishes the tested equations, constraints and
specification envelopes—not hardware calibration. Motor equivalents, governor
droop, gearbox efficiency, inertia, thermal RC, battery OCV/resistance, tire
friction/relaxation and sensor noise still need measurements. The 11 W output
cap is conservative rather than a reproduction of every published transient
peak. The 5.5 W thermal schedule is explicitly assumed. IMU saturation/invalid
flags and full-pack regeneration shunting are declared simulation policies,
not identified VEX firmware or protection circuits.

The robot remains planar: no suspension, physical tipping/falling, robot-robot
collision model, or dynamically coupled intake/lift mass and electrical load.
Tracking-wheel preloads are lower-bound biases on rigid supports, not measured
spring forces. The offline Rapier intake prototype is still unfinished and is
not the live physics engine. The interactive clock intentionally retains its
250 ms catch-up cap for long host stalls.

Real-robot logs and the actual sensor/drivetrain configuration are needed to
calibrate these parameters and tune autonomous trajectories. The remaining
motion acceptance failures must not be presented as competition-ready tracking.

## Reproduce

From the sibling simulator:

```sh
DISPLAY= WAYLAND_DISPLAY= PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

From mclib:

```sh
make -j8 test
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_electrical_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_mechanical_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_sensor_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_game_audit.py
node tests/vexsim/physics_game_audit.mjs ../vexsim
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/motion_completion_audit.py --gains ideal
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/motion_completion_audit.py --gains both
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/kinematic_motion_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py
```

The last three commands intentionally retain nonzero exit status when their
reported acceptance failures occur. C++ oracle compiler commands are in the
historical physics report. V5/ARM compilation and hardware execution were not
available; host success does not substitute for them.

Artifacts for this fix pass are under `/tmp/vexsim-fixed-*.log`,
`/tmp/mclib-fixed-physics/`, `/tmp/mclib-fixed-kinematic/`,
`/tmp/mclib-motion-completion-n9_wtnkc/`, and the two
`/tmp/mclib-fixed-{tracking,pursuit}-audit.log` files. Shared libraries and trace
CSVs were generated in `/tmp`; `make test` also writes its existing ignored host
build outputs. No dependencies, lockfiles, generated CAD/browser assets or Git
history were changed. The existing graphify graph was used read-only to locate
dependencies; source and independent numerical checks determined the fixes.

## Installed source and recovery

The validated files were installed into `../vexsim` after an
approved workspace-boundary escalation: 12 existing files updated and five new
regression files added. Every installed file was compared byte-for-byte with
the tested copy, and every original was checked for concurrent changes before
installation. Originals remain in `/tmp/vexsim-original-8KoKf4/`; no files were
deleted. Post-install audit logs use `/tmp/vexsim-installed-*.log`.
