# Motion debugger and native playback follow-up

This change adds observation and playback, not a new controller or physics model.
The native viewer integration is validated separately from motion acceptance.

## Native playback and debugging behavior

Recorded/live playback uses the existing sibling
`vexsim/web/push_back.html` Three.js renderer, served through checked local
adapter routes. The adapter disables native manual driving, state polling and
game HUD; the motion worker still owns the only physics/controller clock.
The same selected frame drives the native scene and adjacent telemetry.
Native coordinates are `(mclib Y, -mclib X) * 0.0254` metres, with negated
heading in radians. Rendered scene coordinates are acknowledged back to the
builder and checked against the recording.

Only open-floor geometry is shown: the motion bridge does not enable game-object
or perimeter contacts. The native simplified chassis uses recorded dimensions;
older recordings lacking those dimensions show an explicit rerun message rather
than an invented robot shape. Local Three.js assets are allowlisted; playback
does not fetch a CDN or send `/input`/`/state` requests to another simulator.

The debugger also fixes these replay problems:

- Result inspection pins the last frame owned by that step, including when the
  next action starts at the same timestamp. Skipped steps have no fake frame.
- Cancelled/error jobs retain their own partial recording as **incomplete**;
  interrupted actions and unverified final-stop state are not called passes.
- A floating end timestamp such as `3.569999999999718` no longer rounds the
  range control down to `3.569` and selects the preceding 10 ms frame. Only the
  slider grid and a 1 ns comparison tolerance change; raw recordings stay intact.
- Native **Fit run** includes actual chassis geometry, mesh height and status
  overlay clearance, including after resizing. It no longer fits only the
  robot-centre path and crops the body at the edge.
- Selecting a history entry disables **Load in editor** until that exact entry
  arrives. Late responses cannot replace newer selections; a failed fetch
  restores the displayed recording without changing the draft.
- Comparison requests have the same ownership checks: clearing a comparison
  or changing the current recording invalidates pending responses. Starting a
  run disables and guards history/comparison controls before the POST resolves,
  preventing late history responses from replacing the live run.

## Controller observation

Actual blocking C++ controller ticks publish their phase, active heading
target/error, remaining distance, moving carrot, pre-mix translation/yaw requests,
and slew/voltage limiter activity. Missing fields are unavailable, not zero.
Snapshots reset at entry/return and are never read by control logic.

Instrumentation-only parity compared the preceding compiled controller with the
instrumented controller in separate processes: **20 scenarios, 6,005 frames,
exact serialized equality** for all pre-existing physics, sensor, odometry,
command, return and final-pose fields. This includes all nine builder action
kinds, forward/reverse motions, non-stopping handoffs, wall contact and no-contact.
See [retained parity evidence](../../bin/motion-debugger-validation-TlnzF4/controller-parity/README.md).

The focused host safety suite increased from 728 to **1,044 checks**. The bridge
suite includes sensor units/signs, raw tracking-wheel travel, side current and
temperature aggregation, changing C++ carrot targets, and observer/no-observer
determinism. No V5 firmware build or hardware scheduling measurement is implied.

## Browser and native-renderer verification

The native adapter suite passes **9 tests**, including **1,320** body projection
checks using the installed Three.js `PerspectiveCamera`, `Box3` and the native
placeholder mesh. Desktop/mobile sizes, rotations, long/short paths, path
corners and resizing are covered without substituting a mock projection model.

An independent real-browser check replayed all **631 frames** of the original
boomerang-point sequence at both 1366 × 768 and 390 × 844. The actual native
scene's projected body stayed inside the camera and clear of the status overlay
for every frame. Screenshots were manually inspected after the camera-fit fix.
See [desktop](../../bin/motion-debugger-validation-TlnzF4/native-fit/native-telemetry.png),
[mobile](../../bin/motion-debugger-validation-TlnzF4/native-fit/native-telemetry-mobile.png),
and [per-frame projections](../../bin/motion-debugger-validation-TlnzF4/native-fit/checks.json).

The complete browser smoke run passed **15 checks** against real local simulator
jobs, including the expected drive-encoder failure and the tracking-wheel pass,
native pose/heading acknowledgement, last-frame scrubbing, result-step ownership,
legacy missing-data handling, incomplete cancellation playback, editor
import/export, history loading and desktop/mobile layout.
Screenshots are retained under
[`browser-full-4`](../../bin/motion-debugger-validation-TlnzF4/browser-full-4/).

After the final asynchronous-state fixes, **7/7 GET-only browser checks** pass
against real recording responses with controlled delivery delays. Comparison
clear/reselection/error/current-record changes and history loading are covered.
The pure JavaScript suite passes **206 geometry + 70 debugger + 21 actual-handler
request-ownership checks**. The last group uses deferred transport stubs only;
it does not claim to test physical motion. See
[final browser log](../../bin/motion-debugger-validation-TlnzF4/browser-inspect-final.log)
and [handler log](../../bin/motion-debugger-validation-TlnzF4/browser-async-handlers-final.log).

After the final physics-first gate, a fresh actual-C++ boomerang-point run
(`ca5ea07e62d244668f11f3dab7f3a24d`, **Telemetry check / Boomerang-point -90**)
passed **230 live/replay checks** on the same final source fingerprint. Nineteen
distinct live timestamps were observed before completion; actual native scene
position/heading agreed with recorded physics. Telemetry followed the same time,
live scrubbing stayed disabled, pause/scrub and step-owned hold inspection worked,
and both endpoint checks passed with sequential action timing. This is not a
pass for the approach-shape defect below. See
[check details](../../bin/motion-debugger-validation-TlnzF4/native-sequence-final/checks.json)
and [native playback screenshot](../../bin/motion-debugger-validation-TlnzF4/native-sequence-final/native-telemetry.png).

Browser tooling reused an existing Playwright/Chromium installation; no
dependencies were installed. Software WebGL rendered the actual native scene.
These checks verify playback and observation, not calibration against a robot.

## Final physics-first gate

`baseline-e8743c2c6906` passed all **15 command stages** on source fingerprint
`2399c49cc255bbdecfe6c6da3d0b8ab1b45a57a5f779240a4426c4aefc3f2906`.
The source remained unchanged throughout the run. Reproduce with:

```sh
/usr/bin/python3 -B tests/vexsim/builder/workflow.py baseline \
  --output bin/motion-debugger-validation-TlnzF4/workflow
```

Physics ran before tracking/controller checks:

| Validation | Result |
| --- | --- |
| Electrical / mechanical / sensor / game audits | 22 / 18 / 20 / 10 tests passed |
| Native browser physics-coordinate audit | 4 checks passed |
| Full vexsim suite | 238 tests, OK; 1 GUI test skipped in headless mode |
| Host tests | All 36 binaries passed, including 1,044 motion-safety checks |
| Builder backend / native adapter | 19 / 9 tests passed |
| JavaScript contracts | 206 geometry + 70 debugger + 21 async-ownership checks passed |
| Tracking bridge | 13 tests passed |
| Independent analytic odometry | 6,845 checks passed |
| Independent kinematic pursuit | 72,007 checks passed |

Exact subprocess commands, exit codes and logs are in the
[workflow report](../../bin/motion-debugger-validation-TlnzF4/workflow/report.md).
Earlier source-stale/interrupted attempts are retained as errors, not relabeled
passes. The passing baseline does **not** claim that the broader motion
acceptance matrix or the boomerang approach defect below is fixed.

Validation created ignored reports, screenshots, recording JSON and compiled
host/bridge/oracle artifacts under `bin/`, plus temporary bridge builds under
`/tmp/`. No dependency installation, lockfile/vendor change, commit or staging
operation was performed. The existing 66 staged paths were preserved. This turn
did not modify the sibling vexsim source; native assets are read in place.

## Remaining boomerang approach defect

The sequence-check fixture targets `(15.5, 18.5)` with final body heading `-90°`,
forward travel, lead `0.5`, six-motor 450 RPM chassis, two tracking wheels,
default gains, seed 1 and a 4 s deadline. Endpoint acceptance passes, but the
body reaches approximately **-207.32°** before final alignment.

The moving-carrot target itself passes -200°. Reconstructing its actual P+D
request matched recorded yaw voltage within 0.000980 V (millivolt quantization).
This is chiefly a geometry/translation-braking problem, not overshoot of a
fixed -90° target or an unexpected derivative kick.

Two isolated braking experiments were **not merged**:

| Fixture result | Original | Ray-distance cap | Derivative-only pilot |
| --- | --- | --- | --- |
| Duration | 3.41 s | 4 s timeout | 3.24 s |
| Endpoint error | 1.389 in | 3.261 in | 1.340 in |
| Minimum heading | -207.32° | -87.17° | -172.77° |
| Path length | 35.576 in | 30.394 in | 34.218 in |

The ray-distance cap passed only 2/12 fixtures versus the original's 12/12,
including serious straight-drive regressions from nearly parallel ray geometry.
The derivative-only pilot still does not give the requested -90° approach and
has not passed a regression matrix. Neither is represented as a fix. See the
[frozen experiment report](../../bin/boomerang-approach-braking-2sgVCD/report.md).

Previous broader acceptance failures remain documented in
[BUILDER_VALIDATION.md](BUILDER_VALIDATION.md) and
[SEQUENCE_VALIDATION.md](SEQUENCE_VALIDATION.md). Adding a viewer or telemetry
does not turn those failures into passes.
