# Motion builder

A local browser workbench for the **actual mclib C++ motion
routines** driving the sibling `vexsim` physics engine. The recorded/live view
reuses vexsim's native Three.js viewer; the 2D canvas is only the draft editor.
Neither viewer is a second motion controller or physics model.

From the mclib root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/server.py
```

Open **http://127.0.0.1:8765**. Python 3, `g++`, and the existing `../vexsim`
checkout are required. Native playback also uses the existing local Three.js
0.160.0 files in that checkout; it does not fetch CDN scripts or install packages.
There is no frontend build.
Use `--vexsim PATH`, `--port NUMBER`, or `--output DIR` if needed. Stop the
server with Ctrl+C. It binds only to loopback, not your network interfaces.

## See and edit motion

1. Load the baseline boomerang example, then **Build & run**. It intentionally
   exposes the drive-encoder slip failure; a red result is meaningful evidence.
2. Press Play, scrub the timeline, or change playback speed in **vexsim playback**.
   The native viewer and telemetry use the same selected recorded frame.
   Switch to **Edit targets** for the 2D editor: solid black is
   simulated body truth; dashed red is mclib's sensor-based odometry. The gray
   reference is geometric intent, not a recorded planner path. Robot icons are
   orientation markers, not collision footprints.
3. Load the two-tracker variant and run again. Select the previous run under
   **Compare with** to compare endpoint errors; **Edit targets** also overlays
   its true path in the 2D view.
   This changes modeled hardware, not production controller gains.
4. Add, reorder, copy, or remove steps. Drag point targets/start position on
   the field, or use the numeric inspector. Configure the drivetrain, sensors,
   starting pose, battery, traction, noise seed, voltage, deadlines and gains.
5. Inspect each failed check and its diagnostics. Export the draft as JSON
   to reproduce the exact inputs. Run records retain their original inputs;
   editing a draft does not alter recorded motion.

For boomerang, **Direction** selects forward/reverse travel; **Heading** is the
final body orientation in both modes (`0°` = +Y, `+90°` = +X, `-90°` = -X).
Changing the heading's sign does not select reverse. Press **Build & run** after
editing; Play replays the selected recording's original settings. The direction
regression and remaining trajectory issues are documented in the
[latest follow-up](../BUILDER_VALIDATION.md#continuation-gyro-timing-and-chained-handoffs).

The example files are in [examples/](examples/), including a deliberately timed-out
turn followed by a drive to demonstrate correct recovery/target bookkeeping.
For a simpler motion without the default approach's overshoot/full rotation,
import [point_then_align.json](examples/point_then_align.json), or load Tracking
wheels and set **Lead = 0**. This explicitly tested simulation profile approaches
the point and then aligns; it is not a smooth curved-carrot trajectory or hardware
calibration. All controller gains and endpoint checks stay unchanged.
Every sequence requests a
stop at each step and includes a separate 300 ms hold. This builder currently
does not expose blended/chained output. Cancellation terminates that run's
isolated worker and compiler children; it cannot operate physical hardware.

## Reproducible, staged workflow

Use a new output directory to preserve the starting evidence:

```sh
python3 -B tests/vexsim/builder/workflow.py baseline --output bin/mclib-workflow
python3 -B tests/vexsim/builder/workflow.py focused --output bin/mclib-workflow \
  --spec tests/vexsim/builder/examples/drive_encoder_boomerang.json
python3 -B tests/vexsim/builder/workflow.py verify --output bin/mclib-workflow
python3 -B tests/vexsim/builder/workflow.py report --output bin/mclib-workflow
```

The workflow records stages, commands, logs, exit codes and source fingerprints.
Physics validation precedes tracking/controller validation. A changed source
fingerprint requires a fresh baseline; a failed prerequisite does not become a
pass by proceeding to later stages. Rerunning a stage preserves earlier attempts.
The verify stage includes actual-C++ boomerang forward/reverse and mirrored
trajectory regressions and non-stopping handoff checks on the independent ideal
plant, with declared test gains. Chained boomerangs hand off inside the configured
position band; they do not wait for final-heading settlement.

Use focused runs to distinguish sensor error, stopping drift, deadline misses,
and controller oscillation. Compare the same scenario before and after a change,
then run the full gate. The unchanged drive-encoder matrix remains separate from
the tracker variant and from explicitly tuned ideal-plant tests. **The full gate
currently reports remaining default-configuration acceptance failures**; it must
not be advertised as all green. See [../BUILDER_VALIDATION.md](../BUILDER_VALIDATION.md).

The browser's per-step results do not certify that the separate full CLI
validation gate has passed.

## Live telemetry and recorded inspection

The compact workspace places the sequence/settings on the left, the field in
the centre, and a separate telemetry panel on the right. Build/run and cancel
remain in the toolbar. During execution the panel follows live snapshots from
inside each action and its 300 ms braking hold; after completion it follows the
playback cursor. Wall-clock publication is throttled to about 100 ms without
adding simulated time. A short action may finish between browser polls.

Editor selection is separate from the recorded frame's step. Inspecting a
completed result selects that step's final recorded frame, even when the next
step starts at the same timestamp. Skipped steps have no telemetry. Editing a
draft does not change recorded targets or controller readings.
Cancelled/error jobs retain their own captured partial trace as an explicitly
incomplete recording. An interrupted action is not marked passed; a run stopped
before its first sample has nothing to replay.

The panel and exported trace distinguish these measurements:

| Group | Meaning |
| --- | --- |
| Pose | Simulated body truth versus sensor-based C++ odometry; inches and clockwise degrees. |
| Body motion | Forward/right lateral speed in in/s and clockwise yaw rate in deg/s. These are simulator truth, not control inputs. |
| Sensors | Published IMU heading/rate, raw untared side-average motor encoder degrees/RPM, and raw parallel/perpendicular tracking-wheel travel. Perpendicular travel is right-positive; offsets have not been subtracted from the raw travel. |
| Motor/battery | Actual quantized left/right voltage commands and stop modes; summed absolute winding current and hottest motor per side; battery terminal voltage and pack current. Winding and pack current are different quantities. |
| Controller | Actual C++ phase, active steering target/error, remaining distance, boomerang carrot, and translation/yaw requests before mixing and slew/voltage limits. These are not reconstructed from the editable draft. |

Unavailable fields, including controller details absent from older recordings,
display `—`. Idle resets controller targets and requests so a completed action
cannot leave stale values in the next one. Read the powered frames before the
hold when inspecting the final controller command. The command and heading
history plots show the selected recording, not a newly calculated trajectory.

### Native vexsim playback

The builder serves the existing `vexsim/web/push_back.html` renderer through a
small, checked adapter. Native scene, lighting, camera controls and the simplified
chassis mesh are reused; no second robot simulation or native manual-drive worker
is started. The same-origin iframe receives the exact replay/live frame selected
by the builder. Inches/CW headings are converted to vexsim's metres/CCW frame.
Pausing or scrubbing does not integrate another physics step. Keys 1–4 select
native camera views; manual drive/game actions are disabled.
**Fit run** includes the recorded chassis footprint and native mesh height,
keeps the robot clear of the status overlay, and refits after viewport resizing.

This is **open-floor drivetrain physics**, not a Push Back game run: field
objects, perimeter collision geometry and game scoring are not enabled by the
motion runner, so the native adapter hides their visuals. The native simplified
chassis represents recorded body dimensions, not a calibrated CAD/contact model.
Older recordings without recorded dimensions must identify their missing visual
metadata; their positions and headings remain original recorded measurements.
Native source compatibility or local asset errors are shown explicitly instead
of silently substituting the 2D renderer. No sibling native source, dependency
directory or lockfile is rewritten.
See [debugger validation](../DEBUGGER_VALIDATION.md) for playback checks,
controller-instrumentation parity, and the remaining boomerang approach defect.

## Measurement and safety boundaries

- Coordinates: inches, +X right, +Y forward, clockwise degrees from +Y.
- Wheel-scroll zooms the field; arrow keys pan it when focused. Fit view restores
  the recorded path bounds. Raw truth heading is unwrapped; odometry heading is
  wrapped, so headings separated by 360 degrees have the same orientation.
- The C++ bridge receives refreshed/quantized motor encoders, IMU and optional
  passive tracking-wheel measurements. Ground truth is recorded only for checks.
- Two trackers are real modeled undriven wheels, with their own dynamics and
  noise. Their frame signs, offsets, unit scaling and reset behavior have
  independent bridge regressions. They are not perfect position sensors.
- `driveTo` and `curveCircle` still use drive-encoder travel for their distance
  loops, even if position odometry uses trackers. A tracker cannot by itself
  make those loops slip-independent.
- Wall reset is an idealized stall experiment, not field-wall collision. It
  must be the final step because it changes the odometry coordinate frame.
  A successful reset is not physical teleportation or a meaningful body-to-reset
  position error.
- The server accepts bounded, finite scenario JSON only. It has a fixed static
  file allowlist, local Host/Origin checks, a bounded queue, and one subprocess
  per run because the C++ adapter holds process-global state.
- All run files and compiled caches default to `/tmp/mclib-builder-*`. The
  browser retains the editable draft in local storage. The server retains up to
  40 recent jobs in memory; older artifact directories are not deleted. Reuse
  `--output` to restore that directory's recent history after a server restart.
  For evidence that survives temporary-directory cleanup, use an output path
  under the existing ignored `bin/` directory, such as `--output bin/mclib-builder`.
- Numerically validated equations are not a calibrated robot. No V5 firmware
  build, physical hardware execution or measured tire/motor tuning is implied.

Run the focused backend tests with:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/test_builder.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/test_native_viewer.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/tracking_bridge_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/arc_completion_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/boomerang_direction_audit.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/boomerang_chain_audit.py
node --check tests/vexsim/builder/builder.js
node tests/vexsim/builder/reference_geometry_test.mjs
```

HTTP tests and the interactive server need permission to open localhost sockets.
Optional end-to-end browser checks reuse an already-installed Playwright and
Chromium; they do not install dependencies into either repository.

```sh
node tests/vexsim/builder/browser_smoke.mjs \
  --playwright /absolute/path/to/playwright/index.mjs \
  --browser /absolute/path/to/chrome \
  --output /tmp/mclib-browser-check
```

Keep the source unchanged while a validation stage runs. Source edits invalidate
that attempt; rerun the baseline after implementing the next fix.
