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

**Wait** does nothing for its time, 10 ms to 120 s, and exports as
`pros::delay()`. The drive stays as the last step left it. Use it to wait
for the Driver Controlled Period before a Match Load.

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

## The Override robot: cascade lift, pneumatic claw and Toggle bars

On the Override field the robot is vexsim's cascade robot, built like the
Override robots teams have published:

- a bite: a round notch cut into the front of the frame, between the front
  wheels. A Goal or a stack the robot drives onto slides into it, and its
  round back stops the Goal's middle right under the claw. It replaces the
  sloped standoffs the robot had before;
- a three-stage cascade lift behind the bite on two 11 W motors, as on
  [55885X](https://github.com/Jim-Ox/55885X-Inquisitors-2026-2027-VEX-Override-Code),
  [6104M](https://github.com/k1nguy3n/6104M-Monsoon-VEX-Override-2026-2027) and
  [569C](https://github.com/daydayparker/VexOverride569C), with rubber bands
  pulling it up as on
  [1862X](https://github.com/ShadowKingYT444/Goofy_Goobers_code_OVERRIDE).
  It goes from fully down to fully up in about 0.58 s at 12 V. The robot's
  owner chose that speed;
- a pneumatic claw on the top stage, over the middle of the bite, as on
  1862X and [Cosmic Electrons](https://github.com/CyrusKuhlmann/CE_OVERRIDE).
  Its V-shaped jaws open to 4.4 in and close to 3.09 in. That is narrower
  than a Cup's rims (3.16 in) and wider than anything on a Pin (3.02 in at
  the flange). So the claw holds a standing Cup by its top rim, with or
  without a Pin in it, and closes past a lone Pin. The piston strokes in
  5 ms and uses no motor;
- two fixed bars on the lift tower, sloping down and forward to tips 2 in
  ahead of the frame and 13.8 in up, 0.6 in over a Toggle's axle. Driven
  into a Toggle at 11.5 in/s or faster, they turn its top toward the wall.
  They replace the Toggle arm and its motor;
- a drive of four 11 W blue motors on 3.25 in omnis at 450 rpm.

The robot is 17.0 in long, 12.5 in wide and 17.91 in tall at the start.
vexsim's `vexsim/cascade.py` builds it and lists its sources, its guesses
and the choices the robot's owner made. The lift is an mclib
`PresetPositionMechanism` and the claw a valve, both in
[cascade_helpers.hpp](cascade_helpers.hpp). The simulator runs that file,
and **Export C++** copies it.

| Lift preset | Claw height | Use |
| --- | --- | --- |
| Down | 6.2 in | grip a Cup standing on the floor, 0.28 in under its top rim |
| Over a Neutral Goal | 15.79 in | carry a stack over a Neutral Goal's Pin |
| On a Neutral Goal | 12.61 in | the held Cup sits on a Neutral Goal's Pin |
| Over a stack on a Neutral Goal | 22.91 in | carry a stack over a stack already on a Neutral Goal |
| On a stack on a Neutral Goal | 19.73 in | the held Cup sits on that stack's Pin |
| Over the Center Goal | 18.79 in | carry a stack over the Center Goal's Pin |
| On the Center Goal | 15.61 in | the held Cup sits on the Center Goal's Pin |
| Top | 42.2 in | highest |

The claw height is the middle of the jaws. The lift motors turn an
18-tooth sprocket on 0.385 in chain, and the top stage moves three times as
far as the first: 20.8 in of claw height per motor turn.

| Claw preset | Jaws |
| --- | --- |
| Open | 4.4 in apart |
| Closed | 3.09 in apart, on a Cup's top rim |

Four ways to use them:

- **Move the lift** waits until the lift gets to the preset or the time
  limit runs out. The drive holds still.
- **Open or close the claw** always takes 10 ms: the 5 ms stroke and a
  margin. It has no time limit.
- **Mechanisms during this step**, on any drive step: move the lift or the
  claw when the step starts, after the robot has gone some inches, some
  milliseconds after the step started, or when the motion ends. The step
  does not wait for it. An action that has not run when the motion ends
  runs then, and the step says so.
- **Preload**: a red/yellow Pin lying on the floor touching the robot, as
  <SG5> asks: along its right side ("beside", the default for a robot
  backed onto the wall) or across its back ("behind"), or none. The claw
  cannot hold a lone Pin, so the preload stays where it lies.

To score a stack, drive the bite onto it with the lift down, close the
claw, lift over a Neutral Goal, drive the bite onto the Goal, lower the
lift onto it and open the claw.
[override_stack_to_goal.json](examples/override_stack_to_goal.json) does
this. Driving onto a stack along a curve can leave the claw an inch off
its middle. Turn to face it first, then drive straight.

To turn a Toggle, face the wall and drive into the Toggle at 11.5 in/s or
faster: a chained drive that would end past the wall. A slower push jams
it about 36° in, short of the corner. Each hit turns it one face. On red's
side (the left and bottom Toggles) the first hit brings blue to face the
field and the second red. On blue's side (top and right) the first hit
brings red. [override_toggle_bars.json](examples/override_toggle_bars.json)
hits the left Toggle twice. Backing off at any speed, the bars drag the
Toggle about 60° back toward the field until they slide over its top
corner, and it falls back onto the face it was on. A hit at about 75 in/s
can spin it two faces at once, so the long drive into the bottom Toggle in
[override_autonomous.json](examples/override_autonomous.json) runs at 8 V.

The robot may start facing the wall with its bars over a Toggle, as long as
they do not touch it (Q&A 3278: extending over the Field Perimeter and a
Toggle is legal; <SG1c> forbids only touching the Toggle).
[override_autonomous.json](examples/override_autonomous.json) starts that
way and backs off before its first hit.

**Match load** is a step for the Drive Team. They lift one of red's
Loaders by its handle and stand a Cup with a Pin in it on the floor under
it, through the Loader's back (<SG11b>). The Loader stays up, and the robot
takes the stack out through the bottom (<SG11d>): with the front of its
frame on the wall the claw is over the Loader's middle. The robot does
nothing during the step, and a robot standing where the Cup would land
stops it. The rules:

- Match Loads go through the alliance's own Loaders, and only in the Driver
  Controlled Period (<SG11a>). The step fails if it starts in the first
  15 s. Put a 15000 ms **Wait** before it.
- In a Robot Skills Match they can go in at any time, through red's
  Loaders (<RSC2d>).
- A Drive Team Member may push down on the Loader's handle so the Loader
  holds the Pin and Cup in place for the robot (Q&A 3285).

The step lists what the claw did: "Gripped a Cup with a yellow Pin at
(24, -23), 3.2 in up, 0.01 s into the step." or "Dropped a Cup with a
yellow Pin at (-24, -47), 9.8 in up, 0.00 s into the step: the claw
opened." A held stack is one Pin and one Cup, inside <SG6> (Q&A 3166).

The first step checks the robot against the rules: at most 18 in long, wide
and tall at the start (SG1), 88 W of motors in all (R10) and 55 W on the
drive (R11). The two lift motors are 22 W and the claw uses no motor, so a
six-motor drive (66 W) fails: "The drive motors are rated 66 W and all the
motors 88 W. The limits are 55 W for the drive (R11) and 88 W in all
(R10)." New Override routines use the four-motor drive (44 W, 66 W in
all). During the run a step fails if the robot gets wider or longer than
24 in (SG2), taller than 50 in (SG3), or holds more than one Pin and one
Cup (SG6).

## The Override field in 3D

On the Override field, **3D view** shows the field built from the manual's
sizes: tiles, walls, Goals, Loaders, Toggles, every Pin and Cup, and the
robot with its bite, cascade stages, claw and Toggle bars. Playback moves
all of it, and a Loader the Drive Team lifted for a Match Load is drawn
raised. Drag to turn the view and scroll to zoom. Click the view, then press 1 to 4 for a camera or
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
the axle, the bottom goes toward the wall. Pushed over the axle, as the
Toggle bars do, the top goes toward the wall. If it cannot turn that way,
the robot is held back as by a wall.

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
| Lift presets | the claw heights in the table above | worked out from the Goal and Pin heights: "on" sets the held Cup's rim on the Pin's flange, "over" carries the Cup's bottom 0.25 in over the Pin's tip; vexsim's `LIFT_HEIGHTS` |
| Lift gains | kp 6.0 and kd 20 on inches of claw height, 12 V, plus 1.4 V to hold against the rubber bands | tuned in the simulator |
| Claw step | 10 ms | the 5 ms piston stroke in vexsim's `GUESSES`, doubled |
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
| Toggle drag | a part backing away from the wall side of the bar turns it just far enough to pass, and the bar keeps no spin from it | a real bar dragged to within 2° of its corner by a fast back-off might roll on over |
| Friction | floor 0.5, piece on piece 0.3, piece on robot 0.4, robot on field element 0.35 | plastic on foam tile and plastic |
| Bounce | restitution 0.15 | moulded plastic |
| Rest | a piece slower than 0.02 m/s and 0.4 rad/s for 0.2 s sleeps | |
| Nesting | a Pin tip inside a Goal or Cup opening, lined up within 25°, drops in and is held | |
| Hit force | averaged over 20 ms | how long a frame and bumper take to give |
| Robot rocking | the drive is a solid box the frame's size; the lift's stages, its carriage and a held stack are weights at their heights | no robot's inertia has been published; vexsim's `dynamics.GUESSES` |
| Motor writes | mclib's 10 ms tick falls 2.5 ms before the brain's next write, so each command waits 2.5 ms | PROS writes to the motors every 5 ms ([`DATA_WRITE_RATE`](https://docs.rs/pros-devices/0.2.0/src/pros_devices/smart/motor.rs.html)) but does not say when against the user's loop; the middle of 0 to 5 ms, vexsim's `sim.GUESSES` |
| Standing still | under 0.5 in/s and 2°/s | when the odometry gap at rest is read; at 0.5 in/s odometry 10 ms late trails by 0.005 in |
| Exported ports | lift on ports 7 and -8 with green cartridges, claw valve on ADI port A | placeholders: change them to your robot's |

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

Any step fails if the robot falls over, and the run stops there. vexsim rocks
the robot up onto one edge of its wheels when it brakes, speeds up or turns
harder than the height of its centre of mass allows. A step where it rocked
0.5° or more says how far, and at what angle it would have fallen. On the
Override robot the centre of mass rises from 5.0 in with the lift down to
8.75 in fully up. Braking from full speed with the lift fully up rocks it
less than 0.1°: the motors cannot brake hard enough to tip it.

A path step also reports the largest gap between the robot and the plan, and
how long it took against the planned time. When it ends, it reports how far
off odometry was where the plan ran out and how long the settle took. If the
robot was more than 3 in to the side, `followTrajectory()` backs off and
drives back in with `boomerang()`, which costs 2 to 5 s; the hint says so
and what to change.

Every step that moves the robot says how far odometry was from the
simulated position when the step ended. Odometry trails a moving robot, so
for a step that ended moving (0.5 in/s or faster) it also gives the gap the
next time the robot stood still (under 0.5 in/s and 2°/s), if that came
before the end of the next step. A drive handed
on at 75.9 in/s ended 0.79 in off and was 0.14 in off once still.

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
| [override_autonomous.json](examples/override_autonomous.json) | red's 15 s Override autonomous: both red Toggles, two hits each, and the yellow stack at (23.5, -23.5) on the neutral bottom Goal; 50 points and the Autonomous Bonus in 12.5 s, with claw and lift actions during a drive |
| [override_stack_to_goal.json](examples/override_stack_to_goal.json) | grip the yellow stack at (23.5, -23.5) and set it on the neutral left Goal, with lift actions during the turn and the drive away; 0 points, because the left Toggle is still yellow |
| [override_toggle_bars.json](examples/override_toggle_bars.json) | turn to face red's wall and hit the left Toggle twice with the bars: blue, then red; 20 points |
| [override_match_load.json](examples/override_match_load.json) | wait 15 s, take a Match Load from the red right Loader, back 60 in along the wall and set it on the neutral bottom Goal; the red half up scores 5 points, and the run ends at 24.0 s |

## Files

| File | Role |
| --- | --- |
| `server.py` | the local web server; starts every job in its own process |
| `engine.py` | compiles mclib, measures the robot, plans paths, runs routines, scores steps |
| `project.py` | the routine format, its limits and its error messages |
| `geometry.py` | where each step leaves the robot and the curve of each path |
| `path_helpers.hpp`, `path_bridge.cpp` | path planning and following in C++, shared with the export |
| `cascade_helpers.hpp`, `cascade_bridge.cpp`, `mechanisms.py` | the Override lift and claw in C++, shared with the export; Match Loads, the score and the robot rules |
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

- Push Back has no game objects. On Override nothing starts in the Loaders;
  a **Match load** step puts a stack there.
- The lift's encoders are exact: the planner reads the simulated shaft with
  no 10 ms delay.
- The claw's 200 mL tank starts at 100 psi and loses air on each stroke
  (100 strokes take it to 25 psi). It never leaks.
- Pins and Cups do not touch the Toggles.
- Every simulated drive has omni wheels on all corners.
- The 3D view needs Three.js 0.160.0 in the vexsim checkout's
  `node_modules` (`npm install --prefix ../vexsim --no-save three@0.160.0`).
- A routine can hold 40 steps and a path 30 points. The time limits must add
  up to at most 120 s.
