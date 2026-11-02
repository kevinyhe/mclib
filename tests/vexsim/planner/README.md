# Path planner

Plan an autonomous routine on the field, see the speeds mclib plans for it,
run it in the vexsim physics simulator with mclib's real C++ motion code,
and export it as `autonomous()`.

```sh
python3 tests/vexsim/planner/server.py --vexsim ../vexsim
```

Open http://127.0.0.1:8765. You need Python 3, `g++` and a vexsim checkout.
Nothing is installed or downloaded. The server listens on 127.0.0.1 only.

The first launch compiles mclib in the background. That takes 1 to 2 minutes.
Measuring the simulated robot then takes a few seconds. The status pill at
the top right says what is happening, and **Activity** lists each stage with
its time. Later launches reuse the build until the C++ or vexsim changes.

| Option | Effect |
| --- | --- |
| `--vexsim PATH` | the vexsim checkout (default `../vexsim`) |
| `--port NUMBER` | port on 127.0.0.1 (default 8765) |
| `--output DIR` | where runs and the compiled library are kept (default `bin/path-planner`) |
| `--no-prepare` | do not compile and measure at launch; the first plan or run does it |

## First run

1. Click three places on the field. Each click adds a point to the path.
2. The path turns into colours: that is mclib's planned speed, blue slow and
   red fast. The step shows the planned time and top speed.
3. Press **Run simulation** (or Ctrl+Enter). Each step gets a ✓ or ✗ with the
   distance it ended from its target.
4. Press play, or drag the time bar, to watch the simulated robot.

No setting has to change before the first run.

## The page

| Part | What it does |
| --- | --- |
| Top bar | name, New, Open, Save, Examples, Undo, Redo, Mirror, Export C++, status, Run |
| Routine (left) | the start pose and the steps in order, with each step's plan or result. **+ Add a step** adds one after the selected step. |
| Activity (left) | each stage of the current job with its time, then a log of what happened |
| Field (centre) | the routine on a 144 in field. Drag to edit. Top view or vexsim's 3D view. |
| Playback | play, scrub, playback speed, which run to show, which run to compare with |
| Graph | planned and simulated speed, motor voltage, heading, or odometry error over time. Click it to jump there. |
| Step tab (right) | the selected step's settings, its plan, and its checks from the last run |
| Robot & settings tab | drivetrain, odometry sensors, start pose, path defaults, simulated battery and grip, PID gains |
| Telemetry tab | every recorded value at the playback time: position, speeds, controller, motors, sensors |

## Paths

A path is a chain of curves from where the robot is to each point in turn.
The part between two points is a section.

- **Heading** at a point is the way the robot faces there. Automatic follows
  the line from the point before to the point after.
- **Curve handles** set the heading and how wide the curve is. Select a point
  to show them, then drag. Double-click a point to put both back on automatic.
- **Speed limit** is per section, as a percentage of the robot's top speed.
  Sections without one use the default under Robot & settings (70%).
- **Reverse** backs up along a section. The robot stops where the direction
  changes, then drives the next section the other way.
- Click the selected path's line to insert a point. Right-click a point, or
  select it and press Delete, to remove it.

mclib's trajectory planner plans each run of sections with the same speed
limit and direction, with 60 in/s² acceleration and 60 in/s² cornering. The
robot follows the plan with RAMSETE (`b` = 50, `zeta` = 0.7), then turns to
the final heading and settles on the last point. A path that doubles back on
itself cannot be planned; the message says which section and what to change.

## Other steps

Drive straight, turn to heading, turn to face a point, drive to a point,
drive to a pose (boomerang), arc, reverse arc, swing turn and wall reset.
Each one calls the mclib function named next to its title. Drag its target
or its round heading knob on the field, or type the values. A wall reset
must be the last step: the simulator has no walls, so it locks the motors
to make them stall.

## Checks

Each step is checked after a 0.3 s pause:

| Check | Limit |
| --- | --- |
| Position | 2.5 in from the target; 1.5 in for drive to a pose; 5.5 in for arcs and reverse arcs |
| Heading | 5° from the target heading |
| Time | finished before the step's time limit |
| Stopped | the drive is not powered at the end |

A failed check says what was measured and the limit, for example
"Ended 3.10 in from the target. The limit is 2.5 in." With **Stop at the
first failed step** on, later steps are skipped, as in the exported C++.

A path step also reports the largest gap between the robot and the plan, and
how long it took against the planned time.

## Saving and exporting

- **Save** downloads the routine as JSON. **Open** loads one; files from the
  old motion builder are converted (follow steps become paths).
- The page keeps the routine you are editing in this browser between visits.
- **Export C++** shows `autonomous()` for the routine. Path steps use the
  helpers in [path_helpers.hpp](path_helpers.hpp), copied into the output.
  The simulator compiles the same file, so the robot runs the code that was
  simulated. kS, kV and track width come from the simulated robot; measure
  your own (see [docs/motion.md](../../../docs/motion.md)).
- **Mirror** flips the routine for the other side of the field: left ↔ right
  negates x, front ↔ back negates y. Headings flip and arcs bend the other way.

## Keyboard and mouse

| Keys | Action |
| --- | --- |
| Click the field | add a point to the selected path, or place the selected step's target |
| Click the selected path's line | insert a point |
| Drag a point, handle or target | move it (Shift: 6 in grid and 15° headings) |
| Drag empty space, scroll | pan, zoom |
| Double-click a point | heading and handles back to automatic |
| Right-click a point | delete it |
| Delete or Backspace | delete the selected point, or the selected step |
| Arrow keys | nudge the selected point 0.5 in (Shift: 6 in) |
| Ctrl+Z, Ctrl+Shift+Z or Ctrl+Y | undo, redo |
| Ctrl+Enter | run the simulation |
| Space | play or pause |
| `,` and `.` | step playback 0.1 s back or forward |
| `[` and `]` | select the previous or next step |
| F | fit the routine in view |
| Esc | deselect the point |
| ? | help and this list |
| 1 to 4 in the 3D view | change the camera |

## Examples

| File | Shows |
| --- | --- |
| [s_curve.json](examples/s_curve.json) | one path, all automatic |
| [speed_zones.json](examples/speed_zones.json) | slow through the corner, fast on the straights |
| [out_and_back.json](examples/out_and_back.json) | a reverse section after a forward one |
| [mixed_steps.json](examples/mixed_steps.json) | a path, a turn, a backward drive, a boomerang and a wall reset |
| [wheel_slip.json](examples/wheel_slip.json) | a boomerang that misses by about 6 in with drive encoders; switch to tracking wheels and it passes |

## Files

| File | Role |
| --- | --- |
| `server.py` | the local web server; starts every job in its own process |
| `engine.py` | compiles mclib, measures the robot, plans paths, runs routines, scores steps |
| `project.py` | the routine format, its limits and its error messages |
| `geometry.py` | where each step leaves the robot and the curve of each path |
| `path_helpers.hpp`, `path_bridge.cpp` | path planning and following in C++, shared with the export |
| `export_cpp.py` | the C++ export |
| `native_viewer.py` | serves vexsim's 3D view for recorded runs |
| `static/` | the page: `index.html`, `planner.js`, `geometry.js`, `planner.css` |

Each run is kept in its own folder under `--output`: the routine, progress,
`result.json` with every check and the full recording, `trace.csv`, and
`run.log`.

## Tests

```sh
cd tests/vexsim/planner
VEXSIM_PATH=../../../../vexsim python3 -m unittest -v test_planner test_geometry test_server test_native_viewer
```

`test_planner` compiles mclib once. Set `MCLIB_PLANNER_CACHE` to a folder to
keep that build between test runs. The browser test drives the real page in
Chromium against a running server, with a Playwright you already have:

```sh
node tests/vexsim/planner/browser_smoke.mjs --playwright PATH/playwright/index.mjs \
  --browser PATH/chrome --output bin/path-planner-screenshots
```

## Limits

- Only the open floor is simulated: no field elements, walls or game objects.
- Every simulated drive has omni wheels on all corners.
- The 3D view needs Three.js 0.160.0 in the vexsim checkout's
  `node_modules` (`npm install --prefix ../vexsim --no-save three@0.160.0`).
- A routine can hold 40 steps and a path 30 points. The time limits must add
  up to at most 120 s.
