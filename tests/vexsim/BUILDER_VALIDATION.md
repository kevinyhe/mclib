# Motion builder and remaining-failure investigation, 2026-09-09

The interactive builder runs the **C++ mclib controller and odometry** against
the sibling `vexsim` physics engine, and its Canvas renderer replays the
recorded measurements. The controller is the real C++ code, it gets no perfect
position feedback, and neither the tolerances nor the production gains were
changed to make runs pass.

Start with the [builder guide](builder/README.md). The current local server is
`http://127.0.0.1:8765`; its run artifacts are in
`bin/boomerang-continuation-2jKnx4/builder/`. Restart it with that `--output`
directory to keep history. Earlier evidence in `/tmp` was lost when the machine
reset, so later runs write to the ignored `bin/` directory. None of these
artifacts are committed.

The later [action-sequence validation](SEQUENCE_VALIDATION.md) records 120
mixed-action cases. Execution order passes. It also found a replay-inspection
bug at step boundaries, and motion-acceptance failures remain. It includes current
**Sequence check / …** recordings and exact timing evidence.

## Continuation: gyro timing and chained handoffs

Two more defects were reproduced and fixed after the direction fix.

**Simulator gyro timing.** The IMU integrated the latest angular rate over an
entire publication interval, discarding intervening samples. Constant-rate
checks did not expose this. In a reverse boomerang, gyro heading was wrong by
up to 2.498° at its own sample timestamp. Offline reconstruction isolated the
cause: physical passive-wheel readings produced 0.102-inch odometry error;
published tracker readings paired with timestamp-correct true heading produced
0.104 inches; the same readings with the published gyro produced 0.363 inches.
The C++ and independent same-sample integrators agreed to approximately 1e-14
inches. True heading was used only in this offline diagnosis and never reached
the controller.

The simulator now integrates every supplied gyro sample with trapezoidal
integration and publishes held outputs on the existing port clock. Bias/noise
draws retain their publication cadence, reset clears unpublished history, and
gyro saturation remains enforced. A tiny numerical scheduling epsilon prevents
10 ms refreshes from becoming 11 ms through floating-point roundoff. No physical
coefficients or default error magnitudes changed. The independent sensor audit
now passes 20 methods; the sibling's sensor tests pass 12, including seven new
timing regressions. Gyro integration uses the simulator's published samples
(normally 1 ms); it does not see true heading or the internal 0.5 ms mechanical
substeps. Unsampled
nonlinear motion can still introduce error. Correcting refresh jitter also
changes historical seeded trajectories despite unchanged nominal noise settings.

The frozen-source physics-first baseline passed all **14 checks**: independent
electrical/mechanical/sensor/contact/browser-physics audits (**22/18/20/10/4**),
the complete **238-test** simulator suite (one headless GUI skip), all **36** host
binaries, **17** builder tests, **206** preview-geometry checks, **11** sensor-bridge
regressions, and the **6,845 / 72,007** tracking/pursuit oracle assertions and their
builds. Baseline ID: `baseline-32f59b6a270e`. This validates the modeled
equations and software behaviour. Nothing here is calibrated against hardware.

**Chained endpoint acceptance.** The previous infinite line could return three
inches early or 100 inches sideways from the target, and its edge latch could
prevent a later valid arrival. A chained no-op also cleared live motor outputs
while republishing a nonzero stored baseline. Chaining now preserves incoming
output and hands off inside the existing configured position band, without
waiting for position or final-heading settlement.

The exact-plane intermediate design is preserved as a failed experiment:
18/18 ideal cases timed out just short of the plane. Enabling the existing
minimum-voltage floor alone did not solve it, because signed derivative braking
could create a small limit cycle. The final design uses the already-configured
1.5-inch acceptance band and leaves braking and gains alone.
The configured floor remains active outside that band for a non-stopping
approach; an explicit zero still disables it.

The final moving-chain audit passes **18/18** cases at three declared ideal-plant
speeds, in both directions and both diagonal reflections: handoff at
0.950–2.890 seconds, within 1.410–1.486 inches, with live directional output.
The host suite passes all **36 binaries / 728 motion-safety assertions**.
Both the chain audit and direction audit are required by workflow `verify`.

The fresh focused comparison uses the same target, default gains, lead 0.5,
12 V, 4-second deadline and seed; only the modeled tracker layout differs.

| Corrected-physics measurement | Drive encoders | Two trackers |
| --- | ---: | ---: |
| Controller time | 2.090 s | 3.270 s |
| True endpoint error after 300 ms hold | 6.765 in | 1.317 in |
| True heading error | 0.728° | 1.282° |
| Odometry error at return | 5.813 in | 0.075 in |
| Unchanged acceptance | Fail | Pass |

The focused stage stays **failed** because the drive-encoder baseline fails.
The better tracker result does not count in its place.

The eight physical forward/reverse/reflected boomerangs now pass **8/8** under
the same gains, 4-second deadline and endpoint limits. The previously failing
reverse +90° case improves from **1.681 to 1.346 inches**, and its reflection
from **1.700 to 1.371 inches**. Their odometry errors at return improve from
0.363/0.385 to **0.160/0.168 inches**. Both finish in 3.690 seconds. This fixes
sensor integration; other physical presets still fail.

The lead-zero profile also passes again: **2.870 s, 1.314 inches, 0.557°**,
heading range 0–90.660°, path length 33.160 inches. The default lead-0.5 Forward
+90° profile still reaches 262.852° and accumulates 482.346° of turning over
52.886 inches. It passes the endpoint check, but the path is **not** a smooth
curve.

Chromium passes **10/10** real-run checks, including failed/passed results,
comparison, replay, editing, JSON round trips, persistence, cancellation and
desktop/mobile layout. Current history contains **Gyro fix / Forward +90**,
**Forward -90**, **Reverse +90**, **Reverse -90**, and **Point then align**, each
with the `Gyro fix /` prefix; all five pass. Select a recording and press Play.
For the simpler visual demonstration, choose **Gyro fix / Point then align**.

### Current full gate and remaining failures

The full `verify-f04bc4e170f4` attempt completed against the same source fingerprint
as the passing baseline. It **failed**.

| Current matrix | Result | Outstanding acceptance failures |
| --- | --- | --- |
| Physical drive encoders, default gains / 4 s | 70/80 | Three boomerangs, six arcs, six-motor diagonal point |
| Physical two trackers, default gains / 4 s | 67/80 | Six arcs, three encoder-distance drives, two boomerangs, four-motor point deadline, encoder-heading stress |
| Ideal kinematic plant, default gains / 4 s | 28/36 | Two slow arc deadlines, four fast turn/turn-to-point cases, two fast boomerangs |
| Targeted completion, default gains | 24/27 | Three fast endpoint-only turns |
| Targeted completion, declared ideal gains | 27/27 | None within these conditions |
| Boomerang direction, declared ideal yaw gains | 48/48 plus 42/42 symmetry checks | None within these conditions |
| Boomerang chain, declared ideal yaw gains | 18/18 | None within these conditions |

Both physical matrices retain **40/40 safety and 4/4 wall-reset checks**; all
80 cases in each stop without runtime exceptions. Their pass counts match the
historical matrices, but the measurements below are newly recorded:

- Drive-encoder boomerangs return with estimated target errors of
  1.351/1.305/0.954 inches, while physical errors are 4.092/6.765/10.102 inches.
  The corresponding odometry gaps are 3.064/5.813/9.571 inches. The drive-encoder
  pose drifted away from truth during the move; the robot did not drift after
  returning.
- Tracker arcs have only 0.055–0.090 inches of odometry error but still miss by
  5.970–15.473 inches and all reach the deadline. Their distance loop still
  regulates drive-encoder travel. The three tracker straight-drive misses are
  2.699/3.427/4.781 inches; a better pose estimate cannot fix a loop that
  regulates encoder travel.
- The four-motor and speed-base tracker boomerangs time out with position errors
  of 1.748/2.590 inches and heading errors of 91.764°/139.575°, despite odometry
  gaps of only 0.109/0.169 inches. The speed-base run additionally moves 0.608
  inches during the separately labeled hold.
- The four-motor tracker point misses its deadline, not its 2.5-inch position
  limit. Encoder-heading stress finishes near 90.378° estimated but 56.832°
  physically: a 33.168° miss.

The ideal default-gain result is **28/36, down from the historical 30/36**:
six-motor and speed-base boomerangs are additional failures after the direction
correction. Both enter a ±12 V yaw two-cycle with zero translation demand left
after yaw prioritization: 334/400 and 394/400 ticks, respectively. The carrot
remains ahead throughout; the new behind-carrot reorientation predicate never
triggers. At 76 inches/s, heading alternates 21.906°↔29.479° at ±757.30°/s;
at 94 inches/s, it alternates 4.339°↔13.706° at ±936.66°/s. The fast standalone
turn failures exhibit the same default-gain instability on this no-inertia plant.

For the local discrete yaw PD model, let `q` be degrees of rotation per 10 ms
tick per yaw volt. Stability requires `q * (kp + 2*kd) < 2`; the unchanged
`kp=0.3, kd=1.5` gives 2.083/2.576 at these two speeds, versus 0.932 for the slow
plant. This comes from a simplified local model and is not a tuning rule for a
real drivetrain.
Two bounded diagnostics change **only test heading kd from 1.5 to 0**, retaining
all distance gains, final-turn gains, lead, limits and endpoint checks. They pass
at **1.870 s / 0.892 inches / 0.570°** and **1.620 s / 0.808 inches / 0.329°**.
Those diagnostic passes are separate from the unchanged **28/36** acceptance
result. No production gain changed, and the diagnostic gains were tuned for a
plant with no inertia, so do not use them on a robot.

No production gains, acceptance thresholds or physical coefficients were retuned
to make these matrices pass. Choosing hardware tuning, or replacing the
encoder-distance primitives with pose feedback, needs the robot's gearing,
wheels, tracking layout and measured dynamics.

Commands run for the current staged gate:

```sh
python3 -B tests/vexsim/builder/workflow.py baseline --output bin/boomerang-continuation-2jKnx4/workflow
python3 -B tests/vexsim/builder/workflow.py focused --output bin/boomerang-continuation-2jKnx4/workflow \
  --spec tests/vexsim/builder/examples/two_tracker_boomerang.json
python3 -B tests/vexsim/builder/workflow.py verify --output bin/boomerang-continuation-2jKnx4/workflow
python3 -B tests/vexsim/builder/workflow.py report --output bin/boomerang-continuation-2jKnx4/workflow
```

The workflow report records every underlying build/test command and exit code.
The focused and verify commands return 1 for the failures retained above.

Persistent investigation evidence:

- `bin/boomerang-sensors-3ET3NX/`: sensor timing decomposition and pre-fix traces.
- `bin/boomerang-chain-9sbsI4/`: distant/early chained-arrival reproduction.
- `bin/boomerang-chain-band-before-sbxuT8/`: 0/18 intermediate exact-plane cases.
- `bin/boomerang-chain-band-after-5d2yBJ/`: 18/18 final position-band handoffs.
- `bin/boomerang-sensor-fix-replay-b8V21N/`: all eight physical direction cases,
  lead-zero case, truth/sensor traces and source/library hashes after the gyro fix.
- `bin/boomerang-ideal-yaw-isolation-pD2155/`: the two default-gain ideal failures,
  annotated yaw cycles and separate heading-derivative-only diagnostic passes.
- `bin/boomerang-continuation-2jKnx4/workflow/`: staged physics-first gate,
  source fingerprints, exact commands, logs, results and report.
- `bin/boomerang-continuation-2jKnx4/browser/`: inspected desktop/mobile screenshots
  and exported routine from the ten-check real-browser verification.

All validation builds, traces, caches and screenshots were generated under
ignored `bin/` paths. No dependencies or lockfiles changed. V5 firmware
compilation and runs on a physical robot were not part of this work.

The direction-fix measurements below predate the gyro-timing fix. They are kept
for comparison and are not current results.

## Boomerang direction follow-up

The builder's Forward `+1`, Reverse `-1`, and clockwise heading mapping were
correct. Two controller defects survived the earlier review:

- Directional slew ran **after** wheel mixing. A requested `(+12,-12)` V pivot
  became `(+1,-12)` V, commanding **-5.5 V translation** in a Forward run.
  Boomerang now slews the translation scalar before adding yaw, using the
  previous mean output and the selected forward/reverse rates. Stopped motions
  retain immediate signed PID braking; chaining retains both translation limits.
- With a carrot behind the requested travel direction, the negative cosine
  could reverse propulsion. Outside the endpoint band, the controller now
  commands a symmetric pivot before translating. Small endpoint recovery and
  opposite-voltage braking remain permitted.

Final heading means **body heading**, including in Reverse: `0°` points +Y,
`+90°` points +X, and `-90°` points -X. Negating final heading at the same target
changes the approach geometry; it does not select reverse travel.

The new `boomerang_direction_audit.py` uses the real C++ routine and the exact
ideal plant, with declared ideal yaw gains and unchanged distance gains. Its 6-second deadline is separate from the physical 4-second acceptance
matrix. Before the fix, **34/48 motion cases and 18/42 symmetry comparisons**
passed; afterwards **48/48 and 42/42 pass**. All 24 reverse-travel pairs failed
before the fix. This audit is now required by the workflow's `verify` stage.
The host suite passes **36 binaries / 432 motion-safety assertions**; the builder
backend passes **17 tests**. Host regressions cover both turn signs,
unequal forward/reverse slew rates, chained baselines, lower voltage caps,
behind-carrot pivots and legitimate braking.

Eight six-motor/two-tracker physical direction cases retain unchanged gains,
lead 0.5, 12 V, seed 1, a 4-second deadline and 300 ms hold. **6/8 pass**; all stop
before the deadline. The reported Forward `(24,24,-90°)` case now starts with
exactly zero mean command for 110 ms. Backward body-projected movement drops
from **0.144 to 0.015 inches**; the physical model still drifts during a
pivot. The Forward ±90° runs pass at approximately **0.994/1.005 inches** error.
The reverse `(-24,-24,+90°)` case and its reflection still fail at
**1.681/1.700 inches**. Their estimated endpoint errors are about 1.334 inches,
inside the configured 1.5-inch band, but tracking error is 0.363/0.385 inches.
Post-return drift is only 0.004 inches and slightly reduces the error.

This fixes the direction commands. The trajectory is **still not smooth**. The Forward
+90° case still travels 52.711 inches and accumulates 485.488° of turning. Its
heading reaches 268.924° before returning near 88.227°. The simpler `lead=0`
example was retested: **2.89 s, 1.368 inches, 0.540°**, heading range
0–89.511°, path length 33.083 inches. It still passes the existing no-loop check.
Across the original three physical boomerang presets, drive-encoder cases remain
**0/3**, and two-tracker cases **1/3**; the other tracker presets time out.

Fresh builder history entries are named **Direction fix / Forward -90**,
**Direction fix / Forward +90**, and **Direction fix / Reverse +90**. The last
entry keeps its failed position check. Select a recording
and press Play; changing the editable draft does not rerun an old recording.

Reproduction commands:

```sh
make -j8 test
python3 -B tests/vexsim/boomerang_direction_audit.py
python3 -B tests/vexsim/builder/test_builder.py
python3 -B tests/vexsim/motion_completion_audit.py --gains both
python3 -B tests/vexsim/run.py --filter boomerang --tracking-mode drive
python3 -B tests/vexsim/run.py --filter boomerang --tracking-mode two
```

Artifacts: `/tmp/mclib-boomerang-direction-baseline-audit/`,
`/tmp/mclib-boomerang-direction-after-audit/`,
`/tmp/mclib-boomerang-direction-physics-GRfVu2/`,
`/tmp/mclib-boomerang-lead-zero-2YazrG/`, and
`/tmp/mclib-boomerang-direction-presets-{drive,two}/`.

The sections below preserve the **earlier review, from before the direction
fix**. The complete physics suite and the full 80-case motion matrices were not
rerun for that narrow controller fix, so their gate result is out of date. No
physics source or production gains changed. A Graphify map was used to navigate
the simulator; reading the C++ and independent regressions established these
defects.

## Verified fixes

| Defect | Correction and evidence |
| --- | --- |
| Chained arcs could latch their distance PID at zero before crossing the target | Disable arrival latching for chaining and preserve the preceding live output at crossing; reproduced before the fix |
| Arc length could be calculated from a stale requested heading | Calculate it from the measured entry heading, including an already-reached target |
| Tight arcs incorrectly drove both wheels in the same direction | Preserve the signed inner-wheel arc; below half-track radius the inner tread reverses, and radius zero becomes a pure pivot |
| Arc and swing motion could publish a stale ramp baseline | Publish the final left/right commands, or zero after stopping |
| Swings could choose the wrong held tread or lose chained output | Use measured entry heading, disable chained arrival latching, and retain live output at crossing |
| The builder could falsely fail a drive after a timed-out turn | Read the real C++ heading target instead of reconstructing it from the previous requested angle |
| Nonzero start pose discarded the requested random seed | Preserve the IMU's seed across pose reset; verify frames, signs, circumference, offsets and motor/wheel gearing independently |
| A wall reset could look like enormous tracking error | Mark its coordinate-frame discontinuity, use wrapped heading comparison, and require it to be the last step |
| Source edits during compilation could mislabel cached results | Lock the cache, check the source fingerprint before marking a build complete and before publishing a result |
| Incorrect arc/swing preview geometry | Match signed radius semantics; swing preview translates about the stationary tread using the selected track width |
| Stale playback and layout overlap | Clear the old recording on submission; validate playback, cancellation, desktop bounds, mobile width and CSS behavior in Chromium |

`MotionConfig::arc_exit` now exposes the existing outer-wheel distance settle
settings independently. Its defaults are unchanged: 0.3/0.9 inches, 50/250 ms,
and derivative tolerance 2.25 per tick. The small-radius ideal-plant regression
uses tighter controller settlement and declared gains, which are not tuning for
a physical robot.

The safety suite now has **338 assertions**. The arc reproductions cover both
directions/sides, stale headings, chained endpoints, tight radii and zero-radius
pivots: **36/36 pass**, maximum position error 0.103 inches and heading error
1.282 degrees, with exactly zero pivot translation in the ideal plant.

## Visible comparison, unchanged checks

The two example boomerangs use the same six-motor drivetrain preset, target
`(24,24,90°)`, gains, 12 V cap, 4-second deadline and sensor seed. One changes
the modeled robot by adding two passive tracking wheels.

| Measurement | Drive-encoder baseline | Two-tracker variant |
| --- | ---: | ---: |
| Controller time | 2.13 s | 3.33 s |
| True endpoint error, after 300 ms hold | 7.251 in | 0.675 in |
| True heading error, modulo 360° | 0.767° | 0.972° |
| Acceptance | Fail | Pass |

The pass is an **endpoint check**. It says nothing about whether the trajectory
was smooth or efficient.
The unchanged aggressive approach can overshoot and turn a full revolution
before final alignment. The replay retains this motion. True heading telemetry
is unwrapped while the estimated pose heading is wrapped; 451° and 91° represent
the same final orientation. Inspect the entire recording, not only its last row.

A bounded follow-up tested 12 declared simulation profiles, adding a no-loop
requirement (heading stays between -45° and 180°) without changing endpoint
checks. Only **lead = 0**, with unchanged gains and 12 V, passed every condition:
2.94 seconds, 1.365 inches, 0.571°, heading range 0–89.491°, path length 33.274
inches. The default lead = 0.5 path travels 52.714 inches and reaches 471.732°.
The [point-then-align example](builder/examples/point_then_align.json) exposes
that simpler approach: it drives to the point, then turns to align. No tested nonzero-lead profile satisfied every added condition.
Probe specifications and traces are in `/tmp/mclib-smooth-boomerang-QgAh3g/`.

The timeout-recovery example fails its 50 ms turn on purpose. Its following
12-inch drive correctly passes at 1.473 inches / 0.060 degrees. The old builder
bookkeeping would have reported approximately 15.6 inches / 88.2 degrees by
checking the drive against the abandoned 90° target.

## Validation results and remaining failures

The independent physics checks pass: **22 electrical, 18 mechanical, 16 sensor,
10 contact/orchestration, and 4 browser-physics checks**. The complete simulator
suite reports **231 tests, OK, one GUI test skipped headlessly**. These test
equations and constraints; they do not measure accuracy on hardware.

The ordinary mclib host suite passes all **36 binaries**. The builder backend's
**16 regressions**, preview geometry's **206 checks**, and real-browser workflow's
**10 checks** pass. The browser tests run the compiled controller; nothing is
mocked. They cover failed/passed runs, replay, comparison, editing,
JSON round trips, local persistence, cancellation, restored inputs and layout.
The sensor bridge passes **11 independent regressions**; the separate odometry
and pursuit oracles pass **6,845** and **72,007 assertions**, respectively.

The default-configuration motion results are kept separate:

| Matrix | Result | Remaining causes |
| --- | --- | --- |
| Original physical drive-encoder matrix, 4 s | 70/80 | Three boomerangs, six arcs, six-motor diagonal point move |
| Two-tracker physical variant, 4 s | 67/80 | Deadlines/settlement, remaining drive-encoder distance loops, and encoder-derived heading under slip |
| Independent ideal motion matrix, default gains | 30/36 | Two slow arc deadlines and four fast turn/turn-to-point oscillation cases |
| Targeted completion, default gains | 24/27 | Three fast endpoint-only turns |
| Targeted completion, tuned ideal plant | 27/27 | None within the stated ideal test conditions |

Both physical matrices retain **40/40 safety** and **4/4 wall-reset** checks.
Tracking wheels do not improve every routine. The passive wheels also change
the modeled loads, and `driveTo`/`curveCircle` still close their distance loops
around drive encoders, which a more accurate pose does not change. Encoder-derived heading also remains vulnerable to slip.

The six-motor point move improves from 2.619 to 1.371 inches with trackers.
For arcs, tracker odometry can be within roughly 0.14–0.32 inches of the true
position while the body still misses its endpoint by many inches, so the
problem is the drive-travel feedback and not the pose integrator. Bounded voltage
and gain probes found no arc profile that worked across presets. The marginal
fits were not made defaults.

Extending only the labeled tracker-boomerang diagnostic to 10 seconds
allows all three presets to settle: 6.14/3.33/5.27 seconds for four-motor,
six-motor and speed-base presets. This does not turn their original 4-second
failures into passes, and it does not solve drive-encoder arc errors.

**The original ten physical acceptance failures are still failures.** Correcting
software defects does not make lateral slip observable through drive encoders.
A production solution needs the robot's sensor layout, measured dynamics and
tuning; arcs that ignore slip would also need a
position-feedback path controller rather than the existing outer-drive-wheel
distance primitive. No ground truth was fed into the controller to conceal this.

## Dynamic workflow and evidence

The CLI workflow keeps each attempt in its own directory and logs the commands
and exit codes. Its order is physics audits → complete
simulator suite → host/backend/preview tests → independent tracking/pursuit
oracles → focused comparison → separate full matrices. Failed or stale baselines
block downstream comparisons. A lock prevents concurrent invocations from
overwriting the same workflow record. Interrupted attempts remain visible.

Use the commands in the [builder guide](builder/README.md). Current evidence:

- `/tmp/mclib-builder-workflow-final/workflow.json` and `report.md`: staged gate,
  complete command lines, revision fingerprints and logs.
- `/tmp/mclib-builder-browser-final/`: desktop/mobile screenshots and exported
  JSON from the real browser checks.
- `/tmp/mclib-builder-arc-completion-final/`: 36 independent arc reproductions.
- `/tmp/mclib-builder-timeout-recovery-final/`: a turn that fails on purpose,
  followed by a drive that is checked correctly and passes.
- `/tmp/mclib-builder-physics-default-final/` and
  `/tmp/mclib-builder-physics-trackers-final/`: preserved physical matrices.
- `/tmp/mclib-builder-tracker-boomerang-10s/`: separately labeled deadline probe.

The final frozen-source workflow completed: baseline `33e798139d99` passed all
14 checks; focused `310f48adbfd5` returned 1 because its drive-encoder case failed
while its tracker case passed; verify `0c4755e5715a` returned 1 with the matrix
counts recorded above. The report stage completed. The earlier interrupted
baseline stays recorded as an error.

The final verification returns nonzero because these acceptance failures remain.
The browser's five-step indicator covers only the selected run's checks, not the
full gate.

No dependencies were installed, and no simulator source, lockfiles or vendored
assets changed. Host builds used the ignored `bin/tests/`; shared libraries,
traces, screenshots and reports used `/tmp`. V5/ARM firmware compilation and
runs on physical hardware were not performed. A Graphify map was used for
navigation; reading the source and independent tests determined the changes.
