# Motion builder

A browser tool for building motion sequences, running mclib's C++ motion code
against vexsim, and replaying the results.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/server.py
```

Open http://127.0.0.1:8765. Requires Python 3, `g++` and `../vexsim`. The 3D view
uses the Three.js files in the vexsim checkout; nothing is downloaded or built.

| Option | Effect |
| --- | --- |
| `--vexsim PATH` | simulator location |
| `--port NUMBER` | server port |
| `--output DIR` | where runs are saved (default `/tmp/mclib-builder-*`) |

The server listens on localhost only.

## Using the builder

1. Load an example and press **Build & run**. The drive-encoder boomerang
   example fails because of wheel slip.
2. Play or scrub the recording. **Edit targets** shows the 2D editor: solid
   black is the simulated robot, dashed red is odometry, gray is the planned
   geometry.
3. Load the two-tracker example, run it, and select the earlier run under
   **Compare with**.
4. Add, reorder or remove steps. Drag targets on the field or type values.
   Configure the drivetrain, sensors, start pose, battery, traction, noise seed,
   voltage, time limits and gains.
5. Check each failed check's details. Export the sequence as JSON to share or
   rerun it.

For boomerang, **Direction** picks forward or reverse and **Heading** is the
final robot heading (`0°` = +Y, `+90°` = +X).

Each step stops the robot and holds for 300 ms. Chained motions are not
supported.

### Examples

| File | Shows |
| --- | --- |
| [drive_encoder_boomerang.json](examples/drive_encoder_boomerang.json) | boomerang failing from wheel slip |
| [two_tracker_boomerang.json](examples/two_tracker_boomerang.json) | the same move with tracking wheels |
| [point_then_align.json](examples/point_then_align.json) | drive to a point, then turn |
| [timeout_recovery.json](examples/timeout_recovery.json) | a turn that times out, followed by a drive |

## Telemetry

The right panel shows live values during a run and the selected frame during
playback.

| Group | Values |
| --- | --- |
| Pose | simulated and odometry position (in) and heading (°, clockwise) |
| Body motion | forward and sideways speed (in/s), turn rate (°/s) from the simulator |
| Sensors | IMU heading and rate, motor encoders, tracking wheel travel |
| Motor/battery | left and right voltage, stop mode, motor current, hottest motor, battery voltage and current |
| Controller | phase, heading target and error, remaining distance, boomerang carrot, requested drive and turn before limits |

Missing values show `—`. Cancelled runs keep their partial recording, marked
incomplete.

## 3D view

The 3D view reuses vexsim's `web/push_back.html` renderer. Only the open floor
is simulated: field elements, walls and scoring are hidden. Keys 1-4 change the
camera. **Fit run** frames the whole path.

## Validation workflow

```sh
python3 -B tests/vexsim/builder/workflow.py baseline --output bin/mclib-workflow
python3 -B tests/vexsim/builder/workflow.py focused --output bin/mclib-workflow \
  --spec tests/vexsim/builder/examples/drive_encoder_boomerang.json
python3 -B tests/vexsim/builder/workflow.py verify --output bin/mclib-workflow
python3 -B tests/vexsim/builder/workflow.py report --output bin/mclib-workflow
```

| Stage | Runs |
| --- | --- |
| `baseline` | physics audits, simulator tests, host tests, builder tests |
| `focused` | one scenario |
| `verify` | full test sets and boomerang regression tests |
| `report` | summary of all stages |

Each stage records its commands, logs, exit codes and a hash of the source. A
source change requires a new baseline. `verify` currently fails; see
[BUILDER_VALIDATION.md](../BUILDER_VALIDATION.md).

## Notes

- Coordinates are inches, +X right, +Y forward, degrees clockwise from +Y.
- Tracking wheels are simulated wheels with their own noise.
- `driveTo` and `curveCircle` use drive encoders for distance even with tracking
  wheels.
- Wall reset must be the last step; it changes the odometry frame.
- Results come from a simulator, not a calibrated robot.
- Use `--output bin/mclib-builder` to keep runs after `/tmp` is cleared. The
  server keeps the 40 most recent runs in memory.

## Tests

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

Browser tests use an existing Playwright and Chromium install:

```sh
node tests/vexsim/builder/browser_smoke.mjs \
  --playwright /absolute/path/to/playwright/index.mjs \
  --browser /absolute/path/to/chrome \
  --output /tmp/mclib-browser-check
```
