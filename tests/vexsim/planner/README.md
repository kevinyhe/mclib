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
must be the last step. On the open floor there are no walls, so it locks
the motors to make them stall.

## Keep moving into the next step

By default every step stops and settles, then the planner waits 0.3 s
before the next one. Tick **Keep moving into the next step** (under **When
it ends**) to chain a step into the next:

- The step runs with `exit = false`. It ends as soon as the robot reaches or
  passes its target, with the drive still powered, and the next step starts
  at once. There is no 0.3 s pause.
- A path that keeps moving into another path carries its speed across the
  join. The first path ends at the fastest speed the second can start at,
  and the second starts at the speed the first reached. The plan card shows
  both. Where the direction flips at the join, the robot stops there.
- A path that keeps moving has no final turn or settle, so its automatic
  time limit is the planned time plus 0.5 s.
- Any other step after it starts from whatever speed the robot has.

Not allowed on turn to face a point (`turnToPoint()` has no `exit` option),
on a wall reset, or on the last step. A chained step is checked where it
hands off: position against the usual limit, heading only for turns, swings
and arcs, and no Stopped check. Export C++ writes `false` for `exit`, and
the join speeds for chained paths.

## Fields

**Field** on the Robot & settings tab picks what the robot drives on:

| Field | What is there |
| --- | --- |
| Override (2026-27) | walls, Goals, Toggles, Loaders, Pins and Cups. New routines start here. |
| Push Back (2025-26) | walls, Long Goals, the Center Goal and Loaders. No game objects. |
| Open floor, no walls | nothing. Files saved before fields existed open with this. |

On a field the robot stops against what it hits, and the drive motors feel
it: current rises and the wheels slow. Each hit is listed under its step,
for example "Hit the red left Goal at (21, -47) 1.07 s into the step, peak
66 N, drive current up to 2.2 A."

The Override field follows the Game Manual version 2.0 (3 September 2026).
Coordinates: red's Alliance Station is at the bottom (-Y). The left Toggle
is at (0, -70.1), on red's wall. New routines start at (0, -62.7) facing
+Y, the robot's back against that wall.

## The Override robot: cascade lift, claw and Toggle arm

On the Override field the robot is vexsim's cascade robot, built like the
Override robots teams have published:

- a three-stage cascade lift on two 11 W motors, as on
  [55885X](https://github.com/Jim-Ox/55885X-Inquisitors-2026-2027-VEX-Override-Code),
  [6104M](https://github.com/k1nguy3n/6104M-Monsoon-VEX-Override-2026-2027) and
  [569C](https://github.com/daydayparker/VexOverride569C), with rubber bands
  pulling it up as on
  [1862X](https://github.com/ShadowKingYT444/Goofy_Goobers_code_OVERRIDE);
- a claw on the top stage, 4 in wide open, that holds a Pin by its flange in
  a groove along each jaw;
- an aligner: two sloped standoffs under the claw that catch a Goal's top and
  slide the robot onto its middle;
- a Toggle arm on a 5.5 W motor geared 3:1, as on 55885X, that swings a pad
  behind the robot;
- a drive of four 11 W blue motors on 3.25 in omnis at 450 rpm.

vexsim's `vexsim/cascade.py` builds it and lists its sources and guesses.
The lift, claw and Toggle arm are mclib `PresetPositionMechanism`s, written
in [cascade_helpers.hpp](cascade_helpers.hpp). The simulator runs that
file, and **Export C++** copies it.

| Lift preset | Claw height | Use |
| --- | --- | --- |
| Down | 4.2 in | driving; nothing is held this low |
| At a Pin standing in a Cup | 6.8 in | grip the yellow Pin that starts in a Cup |
| Over an Alliance Goal | 6.65 in | hang a Pin 0.15 in above an Alliance Goal |
| Over a Neutral Goal | 9.17 in | hang a Pin 0.15 in above a Neutral Goal |
| Over the Center Goal | 12.17 in | hang a Pin 0.15 in above the Center Goal |
| Top | 40.2 in | highest; the robot is 41.5 in tall |

The claw height is where the claw holds a Pin's flange. The lift motors turn
an 18-tooth sprocket on 0.385 in chain, and the top stage moves three times
as far as the first: 20.8 in of claw height per motor turn.

| Claw preset | Claw motor | |
| --- | --- | --- |
| Open | 0° | jaws 4 in apart |
| Closed | 180° | past where the jaws stop on a Pin, so the motor keeps squeezing |

| Toggle arm preset | Toggle arm motor | Pad |
| --- | --- | --- |
| Stowed | 0° | folded forward over the robot |
| Over the axle | 464.5° | 2 in behind the robot, 0.95 in over a Toggle's axle |
| Under the axle | 540° | 2 in behind the robot, 0.75 in under a Toggle's axle |

Three ways to use them:

- **Move the lift**, **Open or close the claw** and **Swing the Toggle arm**
  steps wait for the mechanism. The drive holds still. A claw step that
  closes on a Pin ends 0.1 s after the jaws stop: "Stopped closing on a game
  object".
- **Mechanisms during this step**, on any drive step: move the lift, claw or
  Toggle arm when the step starts, after the robot has gone some inches,
  some milliseconds after the step started, or when the motion ends. The
  step does not wait for it. An action that has not run when the motion ends
  runs then, and the step says so.
- **Preload**: none, or a red Alliance Pin held in the claw with its red or
  its yellow half up.

To turn a Toggle, swing the arm back with the robot clear of the wall, then
back into the wall under the Toggle at a steady few volts: a chained drive
with Max voltage about 4 V, then a short drive onto the wall. A drive that
slows to a stop can leave the Toggle half turned. Stow the arm before
driving away, or the pad drags the Toggle back.
[override_autonomous.json](examples/override_autonomous.json) does this
twice.

The step lists what the claw did: "Gripped a yellow Pin at (24, -24), 6.8 in
up, 0.17 s into the step." or "Dropped a red/yellow Pin at (24, -47), 5.6 in
up: the claw opened."

The first step checks the robot against the rules: at most 18 in long, wide
and tall at the start (SG1), 88 W of motors in all (R10) and 55 W on the
drive (R11). The cascade's lift, claw and Toggle arm motors are 33 W, so a
six-motor drive (66 W) fails: "The drive motors are rated 66 W and all the
motors 99 W. The limits are 55 W for the drive (R11) and 88 W in all (R10)."
New Override routines use the four-motor drive (44 W). During the run a
step fails if the robot gets wider or longer than 24 in (SG2), taller than
50 in (SG3), or holds more than one Pin and one Cup (SG6).

## The Override field in 3D

On the Override field, **3D view** shows the field built from the manual's
sizes: tiles, walls, Goals, Loaders, Toggles, every Pin and Cup, and the
robot with its cascade stages, claw, aligner and Toggle arm. Playback moves
all of it. Drag to turn the
view and scroll to zoom. Click the view, then press 1 to 4 for a camera or
5 to follow the robot. The score panel stays on top.

VEX's Field CAD is not used. Its download page refused this machine (HTTP
403). The shapes come from the manual's drawings instead, so curved parts
are simplified. On the other fields the 3D view is vexsim's, as before.

## Override scoring

The score is worked out from where the Pins and Cups are, as the manual's
<SC1> to <SC8> say. The score panel on the field shows it at the playback
time. After the run it also shows:

- the score at 15 s, when autonomous ends, and who gets the Autonomous Bonus;
- the Autonomous Win Point: Pin halves owned and Goals with two of them,
  against the 6 halves and 2 Goals needed;
- whether the robot is in the Midfield.

A Toggle turns the way the robot pushes it where they touch. Pushed under
the axle, the bottom goes toward the wall and your colour comes up facing
the field. Pushed over the axle, the top goes toward the wall. If it cannot
turn that way, the robot is held back as by a wall.

A Toggle sets the colour of the yellow halves in its Quadrant only while it
sits flat on a face and nothing touches it. A robot that is still pushing it
when autonomous ends gets nothing for it. The step says when a Toggle
turned: "Turned the left Toggle: red now faces into the field (yellow
before)."

## Guessed numbers

The teams the robot follows publish code, not drawings, and VEX publishes
no masses for game objects. These numbers are guesses. Everything else on
the Override field comes from the manual, the Q&A or VEX's field drawings.

The robot's sizes, masses and frictions that no source gives are in
`GUESSES` in vexsim's `vexsim/cascade.py`. The planner adds these:

| What | Guess | Why |
| --- | --- | --- |
| Lift presets | the claw heights in the table above | worked out from the Goal heights: 0.15 in of clearance over each Goal |
| Lift gains | kp 6.0 and kd 20 on inches of claw height, 12 V, plus 1.4 V to hold against the rubber bands | tuned in the simulator |
| Claw gains | kp 0.1 on degrees, 12 V | tuned in the simulator |
| Toggle arm gains | kp 0.12 and kd 0.6 on degrees, 12 V | tuned in the simulator so the pad holds while the robot backs into a Toggle |
| Toggle arm presets | 464.5° and 540° | worked out from the arm length and the Toggle's axle height |
| Pin mass | 60 g | hollow moulded part of this size |
| Cup mass | 45 g | thin moulded shell |
| Toggle mass | 0.40 kg | 26 in triangular extrusion |
| Pin flange | 0.64 in thick | 6.50 - 2 x 2.93 in |
| Cup shape | straight taper from rim to waist | the drawing shows a curve |
| Goal shape | neutral and Center Goal risers use the Alliance Goal's 5.61 in base; game objects see the octagon as a circle | |
| Toggle face | equilateral, 2.05 in | the glossary's face width |
| Toggle seated | within 3° of a face | |
| Toggle seat | the bar lifts onto a corner when turned and drops onto a face | what makes it click into three positions |
| Toggle damping | 2e-3 N m s | a dropped bar stops rocking in about 0.1 s |
| Toggle push | a robot turns it without slowing; Pins and Cups do not touch it | 0.1 N m is a few newtons on a robot |
| Toggle contact | the middle of where a part and the bar overlap; friction 0.35 there can hold the bar | the robot on field element figure |
| Friction | floor 0.5, piece on piece 0.3, piece on robot 0.4, robot on field element 0.35 | plastic on foam tile and plastic |
| Bounce | restitution 0.15 | moulded plastic |
| Rest | a piece slower than 0.02 m/s and 0.4 rad/s for 0.2 s sleeps | |
| Nesting | a Pin tip inside a Goal or Cup opening, lined up within 25°, drops in and is held | |
| Hit force | averaged over 20 ms | how long a frame and bumper take to give |
| Exported ports | lift on ports 7 and -8, claw on port 9, Toggle arm on port 10, green cartridges | placeholders: change them to your robot's |

## Checks

Each step that stops is checked after a 0.3 s pause:

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
how long it took against the planned time. When it ends, it reports how far
off odometry was where the plan ran out and how long the settle took. If the
robot was more than 3 in to the side, `followTrajectory()` backs off and
drives back in with `boomerang()`, which costs 2 to 5 s; the hint says so
and what to change.

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
| 5 in the Override 3D view | follow the robot |

## Examples

| File | Shows |
| --- | --- |
| [s_curve.json](examples/s_curve.json) | one path, all automatic |
| [speed_zones.json](examples/speed_zones.json) | slow through the corner, fast on the straights |
| [out_and_back.json](examples/out_and_back.json) | a reverse section after a forward one |
| [mixed_steps.json](examples/mixed_steps.json) | a path, a turn, a backward drive, a boomerang and a wall reset |
| [wheel_slip.json](examples/wheel_slip.json) | a boomerang that misses by about 6 in with drive encoders; switch to tracking wheels and it passes |
| [override_autonomous.json](examples/override_autonomous.json) | red's 15 s Override autonomous: both red Toggles and the preload, 55 points and the Win Point in 12.6 s, with mechanism actions during chained motions |
| [override_preload.json](examples/override_preload.json) | the preload into a Goal, with lift and claw actions at the start, at the end and after a time; 5 points |
| [override_cup_to_goal.json](examples/override_cup_to_goal.json) | turn the left Toggle, take the yellow Pin out of a Cup and drop it in a Goal; 40 points in 11.5 s |

## Files

| File | Role |
| --- | --- |
| `server.py` | the local web server; starts every job in its own process |
| `engine.py` | compiles mclib, measures the robot, plans paths, runs routines, scores steps |
| `project.py` | the routine format, its limits and its error messages |
| `geometry.py` | where each step leaves the robot and the curve of each path |
| `path_helpers.hpp`, `path_bridge.cpp` | path planning and following in C++, shared with the export |
| `cascade_helpers.hpp`, `cascade_bridge.cpp`, `mechanisms.py` | the Override lift, claw and Toggle arm in C++, shared with the export; the score and the robot rules |
| `fields.py` | the fields: building them in vexsim, drawing them, and reporting hits |
| `export_cpp.py` | the C++ export |
| `native_viewer.py` | serves vexsim's 3D view for recorded runs |
| `static/field3d.html`, `static/field3d.js` | the Override field in 3D |
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

- Push Back has no game objects. On Override nothing starts in the Loaders
  and there is no human player to load them.
- The lift, claw and Toggle arm draw no current from the simulated
  battery, and their encoders are exact.
- The Toggle arm's pad collides as a square that does not turn with the
  arm.
- Pins and Cups do not touch the Toggles.
- Every simulated drive has omni wheels on all corners.
- The 3D view needs Three.js 0.160.0 in the vexsim checkout's
  `node_modules` (`npm install --prefix ../vexsim --no-save three@0.160.0`).
- A routine can hold 40 steps and a path 30 points. The time limits must add
  up to at most 120 s.
