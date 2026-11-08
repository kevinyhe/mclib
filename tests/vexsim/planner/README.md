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
| `--parallel-runs N` | runs simulated at once (default: a third of the CPU's threads, 8 on 24). Each run holds about 400 MB. |

## First run

1. Click three places on the field. Each click adds a point to the path.
2. The path turns into colours: that is mclib's planned speed, blue slow and
   red fast. The step shows the planned time and top speed.
3. Press **Run simulation** (or Ctrl+Enter). Every step runs, and each one
   shows how far it ended from its target. The run stops early only when a
   step breaks a game rule.
4. Press play, or drag the time bar, to watch the simulated robot.

No setting has to change before the first run.

You can press **Run simulation** again while runs are going. Each run is its
own process, and up to `--parallel-runs` of them go at once, with 4 more
waiting. The page shows the newest one; pick another under Playback. Cancel
stops the run on screen.

## The page

| Part | What it does |
| --- | --- |
| Top bar | name, New, Open, Save, Examples, Undo, Redo, Mirror, Export C++, status, Run |
| Routine (left) | the start pose and the steps in order, with each step's plan or result. **+ Add a step** adds one after the selected step. |
| Activity (left) | each stage of the current job with its time, then a log of what happened |
| Field (centre) | the routine on a 144 in field. Drag to edit. Top view or vexsim's 3D view. |
| Playback | play, scrub, playback speed, which run to show, which run to compare with |
| Graph | planned and simulated speed, motor voltage, heading, or odometry error over time. Click it to jump there. |
| Step tab (right) | the selected step's settings, its plan, and what it did in the last run |
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

## Time limit, Max voltage and Min voltage

Every drive step has these under **Limits**:

- **Time limit** ends the motion when it runs out, even short of the
  target, and the next step starts. The robot is often still moving then:
  a 30 in drive cut off at 0.30 s on the six-motor drive says "The motion
  ended with the robot at 50.3 in/s. It braked to a stop 0.21 s and 5.4 in
  later."
- **Max voltage** caps the drive voltage. 12 V is full power.
- **Min voltage** is the motion's `min_speed`, the least voltage the drive
  gets so it does not stall short of the target. Automatic leaves it to
  mclib, which uses 1.5 V and only while a step keeps moving into the next
  one, except in `turnToPoint()`, which always uses it. A number applies it
  for the whole motion, except in `turnToAngle()` and `swing()`, which only
  use a floor while they keep moving. 0 turns it off. On a step that stops,
  too high a floor keeps the robot from settling: with 6 V a 24 in drive
  rocked about 3 in either way across its target until its time limit.

## Speed-up and slow-down

**Speed-up and slow-down** in the robot settings are mclib's slew limits:
how many volts a drive step may raise or lower the drive voltage each
10 ms. The default, 1 V, takes 0.12 s from 0 to 12 V. On the Override
robot that spins the wheels at launch: the wheels push about 53 N against
about 52 N of grip (2.75 in omnis, 0.84 static and 0.68 sliding, from a
VEX forum pull test). The centre of mass is 0.78 in behind the middle, so
launching forward the front wheels carry only 4.9 N each and slip. A 12 V
start in the middle of the field overcounts on the drive encoders by
4.50 in forward and 2.48 in backward. At 0.3 V per 10 ms, 0.4 s from 0 to
12 V, it overcounts 0.19 in forward and 0.02 in short backward.
The Override examples use 0.3 V. The roller step sets the drive voltage
itself and is not limited. **Export C++** sets the same limits in
`motionConfig().slew`.

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
Coordinates: red's Alliance Station is at the bottom (-Y). The left Toggle's
axle is at (0, -71.45), 1.25 in behind the inside face of red's wall, as in
VEX's field CAD. New routines start at (0, -61.7) facing
+Y, the robot's back 1 in off that wall, so the roller's flaps are 0.25 in
in front of the left Toggle and do not touch it.

## The Override robot: cascade lift, pneumatic claw and Toggle roller

On the Override field the robot is vexsim's cascade robot, built like the
Override robots teams have published:

- a bite: a round notch cut into the front of the frame, between the front
  wheels. A Goal or a stack the robot drives onto slides into it, and its
  round back stops the Goal's middle right under the claw. It replaces the
  sloped standoffs the robot had before;
- a five-stage cascade lift behind the bite, standing on the frame, as on
  [55885X](https://github.com/Jim-Ox/55885X-Inquisitors-2026-2027-VEX-Override-Code),
  [6104M](https://github.com/k1nguy3n/6104M-Monsoon-VEX-Override-2026-2027) and
  [569C](https://github.com/daydayparker/VexOverride569C), with rubber bands
  pulling it up as on
  [1862X](https://github.com/ShadowKingYT444/Goofy_Goobers_code_OVERRIDE).
  Three 11 W green motors turn a 1.0 in drum through 48:72 gears. The drum
  winds string that lifts the first stage, and string between the stages
  makes each one move as far on the one below, so the carriage moves five
  times as far as the first stage. Each stage travels 8.02 in, so the
  robot starts 17.9 in tall, under SG1's 18 in, and reaches 50 in. It goes
  from fully down to fully up in 1.17 s at 12 V; the robot's owner
  measured about 1.2 s. The strings stretch: holding, the claw sits about 0.25 in under
  where the wound string alone would put it;
- a pneumatic claw on the top stage, over the middle of the bite, as on
  1862X and [Cosmic Electrons](https://github.com/CyrusKuhlmann/CE_OVERRIDE).
  Its V-shaped jaws are 4.4 in apart open and 3.1 in closed. Each jaw has a
  bottom band, a recess and a top band. On a stack (a Pin standing in a
  Cup) the bottom band presses on the Cup below its rim and the top band on
  the Pin above its flange, with the ridge where they meet in the recess,
  as on the owner's robot. A lone Cup is held by the bottom bands. A lone
  Pin hangs with its flange on the bottom bands and the top bands squeeze
  it, which is how the preload starts. A Pin standing alone on the floor is
  too thin where the bands reach. A 12 mm piston closes the jaws in 130 ms
  and opens them in 160 ms, 59 N a jaw at 100 psi. It uses no motor. In a
  full-voltage spin a held stack tilts under 2.5°;
- a roller across the back of the robot, its axle 13.5 in up and 1.75 in
  in from the frame's back, with two rubber flaps on each side whose tips
  reach 2.5 in from the axle, 0.75 in behind the frame. One 11 W green
  motor turns it through 12:36 chain, about 67 rpm at 12 V. Each flap that
  sweeps up behind the robot turns a Toggle one face, its top toward the
  wall;
- a drive of four 11 W blue motors on 2.75 in omnis at 450 rpm, 14 lb in
  all with its centre of mass 3.5 in up, as the owner gave them, and one
  sprung 2 in tracking wheel at the centre that measures sideways travel.
  The planner's Override robot uses it with the drive encoders.

The 3D view draws it like an FRC robot: a boxy frame with bumpers around
its outline and into the bite, and the wheels hidden inside. The bumpers
are the outside 1 in of the frame, not extra collision. Every piece in the
3D view is the piece vexsim collides with, the same size and moved the
same way: the fixed stage stands on the frame, each stage on the one
below, the carriage between the top stage's rails, the plate on the
carriage, the jaws on the plate, and the roller and its flaps on two
posts at the back, turned as vexsim turns them.

The robot is 15.75 in long with the flaps, 12.5 in wide and 17.9 in tall at
the start.
vexsim's `vexsim/cascade.py` builds it and lists its sources, its guesses
and the choices the robot's owner made. The lift is an mclib
`PresetPositionMechanism` and the claw a valve, both in
[cascade_helpers.hpp](cascade_helpers.hpp). The simulator runs that file,
and **Export C++** copies it.

| Lift preset | Claw height | Use |
| --- | --- | --- |
| Down | 6.62 in | grip a stack standing on the floor, level with its ridge |
| Off the floor | 7.12 in | lift a stack taken from a Loader 0.5 in before backing out, so the Cup does not drag on the foam and tip; at 0.98 in the claw's top hits a raised Loader's tube |
| Over a Neutral Goal | 15.97 in | carry a stack over a Neutral Goal's Pin |
| On a Neutral Goal | 12.58 in | the held Cup sits on a Neutral Goal's Pin |
| Over a stack on a Neutral Goal | 22.66 in | carry a stack over a stack already on a Neutral Goal |
| On a stack on a Neutral Goal | 19.27 in | the held Cup sits on that stack's Pin |
| Over the Center Goal | 18.97 in | carry a stack over the Center Goal's Pin |
| On the Center Goal | 15.58 in | the held Cup sits on the Center Goal's Pin |
| Top | 46.72 in | highest |

The claw height is the middle of the jaws. A motor turn winds
pi / 1.5 = 2.09 in of string and raises the claw five times that,
10.47 in. The lift reads its height from the motors' angle and takes off
the 0.25 in the strings stretch once the stages hang on them.

| Claw preset | Jaws |
| --- | --- |
| Open | 4.4 in apart |
| Closed | on a stack's Cup below its rim and its Pin above its flange, on a lone Cup, or on a hung Pin; 3.1 in apart with nothing between them |

Four ways to use them:

- **Move the lift** waits until the lift gets to the preset or the time
  limit runs out. The drive holds still.
- **Open or close the claw** always takes 170 ms: the 160 ms opening
  stroke and a margin. It has no time limit.
- **Mechanisms during this step**, on any drive step: move the lift or the
  claw when the step starts, after the robot has gone some inches, some
  milliseconds after the step started, or when the motion ends. The step
  does not wait for it. An action that has not run when the motion ends
  runs then, and the step says so.
- **Preload**: a red/yellow Pin hanging in the closed claw by its flange,
  as <SG5> asks, with its yellow half up (the default) or its red half up,
  or none. The claw holds it until a step opens the claw.

To score a stack, drive the bite up to it with the lift down, close the
claw, lift, drive the bite onto the Goal, lower the lift onto it and open
the claw. [override_stack_to_goal.json](examples/override_stack_to_goal.json)
does this. Four things make it work:

- Arrive with the stack within 0.1 in of the claw's middle side to side.
  Further off, the front end of a jaw pushes the stack ahead and tips it.
  Turn to face the stack, then drive straight; along a curve the claw can
  end an inch off. The robot's middle slides about 1 in as it turns in
  place, so aim beside the stack: for the one at (23.5, -23.5), seen from
  (0, -58.9), aim at (24.0, -23.83), 0.53 in to its right.
- Drive the bite until the stack is under the claw's middle: from 37.0 to
  37.9 in in the example. Closing centres the stack side to side but not
  front to back, because the rubber pads grip too well for the V to slide
  it back. A stack held 0.4 in or more ahead of the middle lands on the
  side of the Goal's Pin.
- Carry it high, at Over a stack on a Neutral Goal (22.66 in), until the
  bite has taken the Goal. Carried at Over a Neutral Goal, 0.25 in over the
  Pin's tip, the Pin knocks the stack out of the jaws as the robot stops
  on the Goal, even at 10 in/s.
- Give the drive onto the Goal a short time limit. The Goal stops the robot
  about 1.1 s in, and the drive then pushes on it until the limit runs out.

To turn a Toggle, back up to 2 in or less off the wall in the middle of
the Toggle and add a **Turn a Toggle with the roller** step. It turns the
roller as far as you set, read from its motor's encoder, while the drive
pushes the robot back onto the wall, then stops both. Each flap turns the
Toggle one face, its top toward the wall. On red's wall (the left and
bottom Toggles) the first face from yellow is blue and the second red. On
blue's wall (top and right) the first is red and the second blue. With the
flaps level at the start, a flap reaches the bar at about 177° and seats
the first face by 255°; the second flap seats the next face by 435°, and a
third reaches the bar at 530°. 470° turns two faces.

The drive has to push back. Each flap that turns a Toggle pushes the robot
off the wall. With the drive off, a robot 1 in off the wall stays there and
the flaps miss the bar. From 2 V up both faces seat, and the robot is pushed
at most 0.3 in off the wall. The roller tips the robot onto its front
wheels, so the back wheels slip and odometry ends off by 0.1 in at 2 V,
0.3 in at 3 V and 1.4 in at 4 V. At 4 V the drive also pushes with 97% of
what the wheels can grip. The step's default is 3 V. Run backward, the flaps press the Toggle onto its plates
and it does not move, so the roller only turns a Toggle one way.
[override_roller.json](examples/override_roller.json) turns the left
Toggle to red.

The flaps cannot be over a Toggle at the start without touching it, so a
robot backed onto the wall at a Toggle starts 1 in off the wall.
[override_autonomous.json](examples/override_autonomous.json) starts that
way, at (0, -61.7).

**Match load** is a step for the Drive Team. They lift one of red's
Loaders by its handle and stand a Cup with a Pin in it on the floor under
it, through the Loader's back (<SG11b>). The Loader stays up, and the robot
takes the stack out through the bottom (<SG11d>): with the front of its
frame on the wall the claw is over the Loader's middle. The robot does
nothing during the step, and a robot standing where the Cup would land
stops it. The rules:

- Match Loads go through the alliance's own Loaders, and only in the Driver
  Controlled Period (<SG11a>). A Match Load that starts in the first 15 s
  breaks this rule, and the run stops after it. Put a 15000 ms **Wait**
  before it.
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
drive (R11). The three lift motors are 33 W and the claw uses no motor, so
a six-motor drive (66 W) breaks both: "The drive motors are rated 66 W and
all the motors 99 W. The limits are 55 W for the drive (R11) and 88 W in
all (R10)." New Override routines use the owner's four-motor drive (44 W,
77 W in all). During the run a step breaks a rule if the robot gets wider or
longer than 24 in (SG2), taller than 50 in (SG3), or holds more than one
Pin and one Cup (SG6).

A step that breaks a rule finishes, and then the run stops: later steps are
skipped. These are the only things that stop a run.

## The Override field in 3D

On the Override field, **3D view** shows the tiles and walls, the Goals,
Loaders, Toggles, Pins and Cups as VEX's field CAD has them, and the robot
drawn like an FRC robot, with its bumpers, cascade stages, claw and Toggle
arms. Playback moves
all of it, and a Loader the Drive Team lifted for a Match Load is drawn
raised. Drag to turn the view and scroll to zoom. Click the view, then press 1 to 4 for a camera or
5 to follow the robot. The score panel stays on top.

The meshes are the ones vexsim's `tools/override_cad.py` makes from the
CAD into `cad/override/` of the vexsim checkout (vexsim's README, "Override
field CAD"); the server sends them as `/cad/view.json` and `/cad/view.bin`.
vexsim collides with the same shapes. A Toggle bar turns on its axle and
rises up its brackets' slots as vexsim's does. On the other fields the 3D
view is vexsim's, as before.

## Override scoring

The score is worked out from where the Pins and Cups are, as the manual's
<SC1> to <SC8> say. A Pin half nested in a Goal or in a Cup's gray half is
not visible and does not score (<SC3>): the bottom half of the Pin in a
Goal never scores, and each Cup hides one half. The score panel on the field shows it at the playback
time. After the run it also shows:

- the score at 15 s, when autonomous ends, and who gets the Autonomous Bonus;
- the Autonomous Win Point: Pin halves owned and Goals with two of them,
  against the 6 halves and 2 Goals needed;
- whether the robot is in the Midfield.

A Toggle turns the way the robot pushes it where they touch. Pushed under
the axle, the bottom goes toward the wall. Pushed over the axle, as the
roller's flaps do, the top goes toward the wall. If it cannot turn that way,
the robot is held back as by a wall.

A Toggle sets the colour of the yellow halves in its Quadrant only while it
sits flat on a face and nothing touches it. A robot that is still pushing it
when autonomous ends gets nothing for it. The step says when a Toggle
turned: "Turned the left Toggle: red now faces into the field (yellow
before)."

## Guessed numbers

The teams the robot follows publish code, not drawings, and VEX publishes
no mass for a Toggle. These numbers are guesses. The Pins and Cups were
weighed. Their shapes, and the Goals', Loaders' and Toggles', come from
VEX's field CAD. Everything else comes from the manual and the Q&A.

vexsim lists its guesses, each with its reason, in `GUESSES` in
`vexsim/cascade.py` (the robot), `vexsim/pieces.py` (friction, bounce and
how stiff contacts are), `vexsim/override.py` (when the claw holds a
piece), `vexsim/toggles.py`, `vexsim/override_field.py`,
`vexsim/mujoco_drive.py`, `vexsim/arena.py` (how hit forces are averaged)
and `vexsim/foam.py`. The planner adds these:

| What | Guess | Why |
| --- | --- | --- |
| Lift presets | the claw heights in the table above | worked out from the Goal and Pin heights: "on" sets the held Cup's rim on the Pin's flange, "over" carries the Cup's bottom 0.25 in over the Pin's tip; vexsim's `LIFT_HEIGHTS` |
| Lift gains | kp 6.0 and kd 40 on inches of claw height, 12 V, plus 0.9 V to hold the stages up with the rubber bands' help | tuned in the simulator: every preset ends within 0.1 in, except On a Neutral Goal, where the stack resting on the Pin stops it up to 0.27 in high |
| String stretch | 0.25 in | measured in the simulator: 0.24 in holding nothing, 0.26 to 0.31 in holding a stack |
| Claw step | 170 ms | the 160 ms opening stroke from vexsim's valve and 12 mm piston, guesses in its `GUESSES`, and a margin |
| Motor writes | mclib's 10 ms tick falls 2.5 ms before the brain's next write, so each command waits 2.5 ms | PROS writes to the motors every 5 ms ([`DATA_WRITE_RATE`](https://docs.rs/pros-devices/0.2.0/src/pros_devices/smart/motor.rs.html)) but does not say when against the user's loop; the middle of 0 to 5 ms, vexsim's `sim.GUESSES` |
| Standing still | under 0.5 in/s and 2°/s | when the odometry gap at rest is read; at 0.5 in/s odometry 10 ms late trails by 0.005 in |
| Exported ports | lift on ports 7, -8 and 9 with green cartridges, claw valve on ADI port A | placeholders: change them to your robot's |

## What each step reports

Steps are not graded. Each step that stops is measured after a 0.3 s pause:

| Reading | Example |
| --- | --- |
| Position | "Ended 3.10 in from the target." |
| Heading | "Ended 1.2° off the target heading." |
| Time | "Finished in 1.42 s of its 2.50 s time limit." or "The 2.50 s time limit ended the motion." |
| Stopped | "The drive was stopped at the end." |
| Stopping | "The motion ended with the robot at 4.2 in/s. It braked to a stop 0.05 s and 0.1 in later." |

A step that misses by more than 2.5 in (1.5 in for drive to a pose, 5.5 in
for arcs), ends more than 5° off its heading, or runs out of time gets a
hint on why. The next step runs either way, as in the exported C++, which
calls each step in turn.

A robot that falls over lies there for the rest of the run, and the step it
fell in says so. vexsim rocks
the robot up onto one edge of its wheels when it brakes, speeds up or turns
harder than the height of its centre of mass allows. A step where it rocked
0.5° or more says how far, and at what angle it would have fallen. On the
Override robot the centre of mass rises from 5.0 in with the lift down to
7.81 in fully up. Braking from full speed with the lift fully up rocks it
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
| Hover over a planned path | show the distance and time from the path's start to that point, out of the path's total, and the planned speed there |
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
| [wheel_slip.json](examples/wheel_slip.json) | a boomerang that misses by about 6 in with drive encoders; switch to tracking wheels and run again to compare |
| [override_autonomous.json](examples/override_autonomous.json) | red's 15 s Override autonomous: the left Toggle with the roller, then the yellow stack at (23.5, -23.5) on the neutral left Goal: red 20, and 2 of the 6 Pin halves the Autonomous Win Point needs. Turning the bottom Toggle as well only starts at 18 s, past the end of the autonomous |
| [override_stack_to_goal.json](examples/override_stack_to_goal.json) | grip the yellow stack at (23.5, -23.5) and set it on the neutral left Goal, with lift actions during the drive to the Goal and the drive away; the Goal stops the robot 1.1 s into the 2 s drive to it. 0 points, because the left Toggle is still yellow |
| [override_roller.json](examples/override_roller.json) | turn the left Toggle two faces with the roller, blue then red, pushing back at 3 V; 10 points, for the top half of the neutral left Goal's yellow Pin, and the Autonomous Bonus |
| [override_match_load.json](examples/override_match_load.json) | wait 15 s, take a Match Load from the red right Loader, back 60 in along the wall, turn at 6 V and set it on the neutral bottom Goal; the red half up scores 5 points, and the run ends at 26.5 s. At 12 V the turn swings the stack forward in the jaws and it lands on the side of the Pin |

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
- Override needs numpy, MuJoCo and the shapes made from VEX's field CAD in
  the vexsim checkout (vexsim's README, "Override field CAD"). Run the
  server and the tests with the Python that has them, such as vexsim's
  `.venv/bin/python`.
- A routine can hold 40 steps and a path 30 points. The time limits must add
  up to at most 120 s.
