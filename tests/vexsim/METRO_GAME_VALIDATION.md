# Original Metro autonomous in native vexsim

Updated September 10, 2026. This harness runs the original Metro C++ code in
native vexsim physics. It does not use mclib's motion functions.

## View and reproduce

The current passive native replay is served at http://127.0.0.1:8766. Port 8765
remains the existing mclib motion builder. Play, pause, Restart, speed, exact-frame
scrub, and native camera controls do not rerun or alter physics. Restart returns
to the first recorded frame and starts playback at the selected speed.

The amber translucent CAD robot shows Metro's recorded odometry estimate beside
the solid robot's simulated true pose. A line joins their centres; the telemetry
panel lists both poses and position error. **Toggle odometry ghost** hides/shows
the estimate without changing playback. Both robots use the same recorded frame.

Current replay: **14 lb (6.35 kg), (-13.5, -46) inches, heading 0°**.
Tire friction is **14% below original** (surface multiplier 0.86). The new recording is
in `bin/metro-friction-minus14/`.
Drivetrain voltage is scaled to **0.9** for roughly 10% slower driving; the
resulting speed and routine duration still depend on the controller and physics.
Loader pickup cooldown is **0.165 s**, half the native 0.33 s. Floor pickup
cooldown remains 0.33 s. Stack settling and intake capacity can limit throughput.
This speeds up pulling the six prefilled blocks out of the loader. It does not
add the blocks a human would feed during a match.
Goal sliding duration is now **150% longer than the previous replay**: 2.5 times
as long. Goal-contact stopping rate changes from 4.0 to **1.6 times native**.
The isolated 1 m/s supported-block test stops in **0.536 s versus 0.214 s**
previously (native baseline 0.859 s). Floor, airborne and loader motion are
unchanged. Collisions and simulation time retain their real rates; in-game
slide duration also depends on the incoming speed and other blocks.

### Editing points and regenerating playback

The current run uses friction multiplier 0.86 and these saved points: (-9,24),
(-30,-10), (-31.5,-27.5), (-31.5,22), (38,23). Saving a point edit does not
update a recording that is already being served. Rebuild the recording, restart
the viewer with the new file, and refresh the browser page. The current
recording was rebuilt from those exact values.
All non-coordinate autonomous code and the controller arc/odometry fixes remain.

Edit **`POINTS` in [metro_waypoints.py](metro_waypoints.py)**:

```python
POINTS = [
    (-9, 24),        # First collection
    (-30, -10),      # After first curve
    (-31.5, -27.5),  # Matchloader
    (-31.5, 22),     # Reverse to long goal
    (38, 23),        # Middle goal after XY reset
]
```

Values are **local odometry inches** relative to where the routine started. XY
starts at (0,0) wherever the robot is placed; the original delayed reset
establishes another local origin before the middle point. Edit both numbers in
the desired pair. Directions, speeds, deadlines and all other source remain
unchanged. Edited points compile into a hashed build copy, with metadata marking
the override. Leave the private source snapshot and generated build files alone.

After editing, generate a new recording using a fresh output directory:

```sh
python3 -B tests/vexsim/metro_run.py \
  --metro /tmp/metro-sim-source-rxDms9/metro \
  --output /tmp/metro-my-points
python3 -B tests/vexsim/game_viewer.py /tmp/metro-my-points/recording.json --port 8766
```

Stop the existing port-8766 viewer before starting the new one. Refresh/restart
playback alone does not rerun the controller. The input snapshot is temporary;
see reproduction requirements below if it is no longer available.
The longer goal sliding remains at stopping rate 1.6.

Validation: point tests verify that edited values are applied, that the
original values restore the original source exactly, single-coordinate edits preserve other
code, and source drift is rejected. New build/recording artifacts are ignored under
`bin/metro-user-points-01/`.
An earlier run used 1.14, which raised friction instead of lowering it; ignore
it. The corrected setting is **0.86**, 14% below the original baseline.
New ignored artifacts are under `bin/metro-friction-minus14/`.
All 10 recording checks pass. Every frame has multiplier 0.86 and effective tire
coefficients 0.903/0.731 longitudinal, 0.2236/0.1892 lateral. The compiled user
points and other physical settings are unchanged. The live replay matches the
new file; the routine returns at 8.280 s and conserves all 61 blocks.

Earlier recordings with shifted or retuned points are kept for history. Their
scoring results do not apply to the restored original autonomous.

Used-mat friction is uncalibrated; the 14% reduction is a chosen scenario
setting. See [mass and friction research](METRO_FRICTION.md) for sources,
values and how to measure this wheel and mat pair. Run/build artifacts are ignored under the existing
validation directory, and previous recordings are preserved.

### Competition field correction

Checked the official [Push Back 4.0 manual, Appendix A, pages A-5–A-7](https://content.vexrobotics.com/docs/25-26/v5rc-push-back/docs/Push-Back-4.0.pdf)
and supplied 276-9142 field CAD. The 36 floor blocks now occupy four three-block
L groups, eight positions beneath the long goals, four two-block corner groups,
and four positions in each opposing park zone. Each loader contains three
near-alliance blocks below three opposing blocks. Native perimeter, goal and
loader dimensions already follow the reference drawing; contact shapes remain
approximations, including the native park platform model.

The viewer retains all four CAD loader assemblies and both CAD park zones,
replacing the simplified cylinders and pads. Embedded CAD blocks, including
those nested inside loaders, are removed before merging so only the recorded
dynamic blocks appear. Perimeter and tile rendering remain simplified. This
single-robot autonomous recording has 61 active blocks; the official 88-block
match inventory additionally includes station loads and other robot preloads.

The field/speed update passed 17 game-coupling tests, 2 drivetrain/stopping tests,
12 viewer tests, 10 recording checks and 162 browser checks. All recorded drive commands were
checked against the 0.9 scale. Local build/recording artifacts are in the run
directory above; browser screenshots are under `/tmp/mclib-field-speed-browser`.
A CAD material check also verifies the corrected red park rails without
recoloring the blue zone's shared source materials.
The subsequent curve/goal-stopping update passes 2 compiled arc-geometry checks,
18 game-coupling checks (including isolated 4x goal stopping), 8 host checks and
10 recording checks. Its generated C++ build copy and recordings remain in the
ignored run directory; temporary test binaries are cleaned up. The new replay
preserves 61 blocks and shows inner-wheel reversal during the tight left curve.

### Applied curve direction correction

The unchanged Metro snapshot's first call is `curveCircle(-120, -2, ...)`.
Its inner arc uses `fabs((fabs(radius) - track/2) * heading_delta)`, discarding
the negative inner-wheel travel needed when the radius is inside the track.
With a 2-inch radius and 11.375-inch track, the inner/outer ratio should be
about -0.480, but Metro uses +0.480 before heading correction. The recorded
heading turns left toward -120 degrees; the rendering direction is consistent
with the C++ commands. The defect is in the original controller's arc path, so
the viewer and drivetrain sides are correct as they are. The repository's
current `src/mclib/control/motion.cpp` already keeps the signed inner arc.
The runner now compiles a build copy of `control.cpp` with that expression and
the odometry offset below corrected. The source checkout stays unchanged. The manifest and recording
identify `signed_inner_arc`, the compiled path and its SHA-256; the recording's
runtime fingerprint verifies the corrected source too. All other Metro control
logic is retained. `test_metro_curve.py --metro PATH` compiles the corrected
geometry and verifies the inferred wheel-path radius for both heading signs,
tight turns, a stationary inner-wheel pivot, and a zero-radius spin.

### Ghost turning-offset correction

The CAD side audit confirmed every drive spinner uses its correct left/right
recording channel. In the preceding run at 0.75 s, the right/outer wheel was
+336 RPM versus -279 RPM on the left/inner wheel. But the ghost travelled
backward while the physical chassis still travelled forward. Metro's
`trackNoOdomWheel` added the half-track turning offset on both sides, creating
false translation during rotation. The build now subtracts the left offset
and adds the right offset. The correction is recorded as
`opposite_odometry_turn_offsets`; wheel animations still use the recorded
wheel angles. The host pure-yaw regression checks
both turn directions for zero estimated translation. Tire slip can still
separate the ghost from the physical chassis.
The regenerated run passes all 8 host checks, including pure yaw in both
directions. At 0.75 s the ghost now travels forward at 8.05 in/s while physical
forward speed is 13.21 in/s; outer/right wheel speed is +218 RPM and inner/left
speed is -138 RPM. Fixing odometry changes autonomous steering, so the new path
differs from the earlier recordings, and this run does not count toward scoring
acceptance.

Odometry uses sampled left/right drivetrain motor encoders plus IMU
(`trackNoOdomWheel`), with no tracking wheels. Results below describe earlier
recordings, not the current 14 lb replay.

### Momentum after motions

The motion return does not zero physical velocity. The adapter forwards Metro's
voltage/coast/brake/hold requests to native motor physics and continues
stepping during delays and after routine return, through the 15-second recording.
Hold is a motor position controller limited by torque and tire grip, so it
cannot freeze the chassis in place. Chained motions inherit the current velocity.

In the preserved 14 lb / 0.93-friction recording, the hold request at 5.510 s still
has 4.545 in/s of body speed. The recorded body travels another 0.143 inches
through that hold interval before the next drive command. At routine return
(8.120 s), speed is 0.238 in/s and the remaining recorded body travel is 0.035
inches. These are simulated path distances, including any settling or
reversal; nobody measured coast distance on hardware. The recording ends at
competition disable, so it shows nothing after 15 seconds.

`python3 -B tests/vexsim/test_metro_stopping.py` checks the output adapter
at the current mass, track width and friction. It verifies that coast, brake and
hold preserve body/wheel momentum at the command instant, continue moving while
slowing, and that coast travels farther than either braking mode. A collision-free
6 V, 0.5-second acceleration followed by 2 seconds of stopping is the test setup;
the autonomous routine still uses its original stop modes.

### Scraper timing and remaining physics gap

The original `leftSideSevenMiddle()` schedules `scraper.set_value(true)` after
500 ms. Its following `pros::delay(300)` only keeps that task asleep; it does
not issue a retraction. `scraper.set_value(false)` happens after the later
`moveToPoint(38, 23, -1, 1200, true)` returns. In the historical 0.93-friction
recording the down command lasts from 0.500 s to 5.590 s. Retraction time must
come from each run's C++ output events, since motion completion can change it.
The replay now displays the recorded UP/DOWN command. That command does not
tell you the real deployment delay or whether the scraper touched the floor.

The front notched plate, both rotating channels, upper cross-shaft and its two
plate-bearing brackets now animate as one rigid assembly around the lower
fixed mounting-bearing pair. These six original CAD parts are kept out of
static mesh merging. The amber odometry ghost includes the same articulation.
Angle is a deterministic function of the selected recording time, using the
C++ scraper event timestamps (frame states for legacy recordings), so
pause, seek, speed changes and Restart stay synchronized.

This is an **illustrative 90° stroke over 300 ms**, with smooth easing in both
directions. Neither angle nor actuation time was recorded or measured. The
300 ms was picked for the animation and has nothing to do with the delayed
task's 300 ms sleep.
The CAD's folded pose is zero; the front assembly rotates forward/down about
the transverse lower bearing axis. This articulation does not change chassis
physics, odometry, the command log or scoring. The next physical-model step
remains the scraper's changing mass distribution.

The scraper has **no ground contact**. Do not add a
scraper contact patch, floor drag or a constrained pivot. The missing effect is
the moving assembly's mass distribution, with Delrin plastic plus approximately
50 g of metal, screws, spacers and other hardware. It is included in the 14 lb
robot total, not added on deployment. A vertical translation alone does not
change yaw inertia; changes in horizontal mass position or part orientation do.

The scraper is the **front rectangular plate with the small triangular notch**,
and the low rear plate is a different part. The original mesh confirms this as
`Component2131`; its associated side arms are
`1x1_Thick_5x_Half-C_Alu_v1_(8)1` and `(8)2`. An earlier 74.47 g estimate was
for rear `Component1561`, the wrong part, and should not be used.

Revised estimate from the original, unsimplified CAD triangles:

| Moving component | CAD volume | Estimated mass |
| --- | ---: | ---: |
| Front notched Delrin plate | 23.711 cm³ | 33.67 g |
| Left aluminum half-channel | 6.252 cm³ | 16.76 g |
| Right aluminum half-channel | 6.252 cm³ | 16.76 g |
| Other hardware (allowance, excluding channels above) | — | 50 g |
| **Total moving assembly** | — | **117.18 g; use approximately 120 g** |

The density assumptions are [Delrin 1.42 g/cm³](https://www.delrin.com/delrin-design-guide-mod-3/)
and [5052 aluminum 2.68 g/cm³](https://www.hulamin.com/sites/default/files/downloads/5052%20Data%20Sheet.pdf).
[VEX specifies 5052-H32 for its aluminum channels](https://www.vexrobotics.com/channel.html);
the CAD names also identify these arms as aluminum. The hardware allowance is
an estimate; nobody weighed the parts. A heavy shaft/cylinder or other
moving part beyond that allowance would change the total. The whole assembly
remains included in the 14 lb robot mass.

For inertia, place the plate mass and each channel mass at their own centres
instead of putting the whole assembly at the plate. The channels' mass centres are
approximately 6.36 inches forward, 5.85 inches high and ±5.50 inches across;
the plate centre is approximately 6.25 inches forward and 8.36 inches high in
the CAD pose. Coordinates use the renderer's existing 15-inch drivetrain scale.
The visualization uses the lower bearing pair as its pivot and an assumed
angle/duration as described above; the current replay still has fixed chassis
inertia. No moving-mass dynamics or ground-contact forces have been applied.

The recording regression checks every frame against the scraper command
events and verifies that the 300 ms sleep does not retract it automatically.

Eight-inch-left run: 15 seconds, 1,501 frames, routine return at 8.370 s,
61 blocks conserved, zero scored. Eight host checks and nine recording checks
passed; the live recording's starting pose and drivetrain-encoder metadata
were verified. Ignored run/build and host-test artifacts remain under the
existing validation directory; previous runs remain intact.

Corrected-side run: 15 seconds, 1,501 frames, routine return at 8.320 s,
61 blocks conserved, two held at the end, zero scored. All nine recording
checks passed, plus a check of the initial physical pose and unchanged local
`(0,0,0)` origin. The earlier recordings were kept.

```sh
python3 -B tests/vexsim/metro_run.py \
  --metro /tmp/metro-sim-source-rxDms9/metro \
  --output /tmp/metro-new-run
python3 -B tests/vexsim/game_viewer.py /tmp/metro-new-run/recording.json --port 8766
```

The source input must be a clean `kevinyhe/metro` checkout at
`0e13c4c6fc6aa7097f97074d6969c05d1d43dc05`. The temporary checkout is not a
permanent dependency. Private source is not added to tracked files; the corrected
control build copy lives with the ignored run artifacts. Build uses GNU
C++23, the six original control/PID/utilities/config/autonomous/intake source
files (with the documented arc and odometry corrections), original command
scheduler headers, and Linux host hardware/task shims.
Original source hashes, adapter hashes, compiler arguments and library hash are
recorded. A float-to-integer overflow sanitizer rejects invalid original motor
voltage conversions. No dependency installation is needed.

## Staged validation

1. Native physics first: electrical 22, mechanical 18, sensor 20, and game 10
   focused checks passed. Run each `physics_*_audit.py` directly; the mechanical
   script initializes imports in its entry point, so broad unittest discovery
   produced 18 spurious NameErrors; running the script directly passed.
2. Native game coupling: `python3 -B tests/vexsim/test_metro_game.py`: 15 passed.
   Includes exact powered-tick parity with native Game/Driver, impulses applied
   once, block conservation, zero-command behavior and non-mutating snapshots.
3. Original C++ execution: `python3 -B tests/vexsim/test_metro_host.py --library
   PATH/libmetro_sim.so`: 8 passed, each in an isolated process. Covers original
   task timing, sensor-dependent PRIME, zero-translation pure-yaw odometry, XY
   reset, disable and invalid-input shutdown. The stationary-sensor fixture's
   return at 9.420 s checks that the source runs as written; it does not predict
   physical motion.
4. Recorded full run: `python3 -B tests/vexsim/test_metro_recording.py
   PATH/recording.json`: 9 passed. Checks 1,501 exact-time frames, original
   routine return, concurrent scraper task, no physical teleport at XY reset,
   startup-only encoder tares, final disabled outputs and current fingerprints.
5. Native viewer: `python3 -B tests/vexsim/test_game_viewer.py`: 12 passed.
   Browser test `game_browser_smoke.mjs`: 161 checks passed, including scraper
   deployment/retraction, backward seeking, ghost visibility and Restart.
   It also checks CAD loading,
   rendered robot coordinates, all recorded blocks/quaternions, telemetry,
   scores, pause/seek and local GET-only requests. It uses Playwright and
   Chromium installations passed on the command line and installs nothing.

The sibling's full declared command, `python3 -B -m unittest discover -s tests -v`
from vexsim, ran 238 tests: 237 passed; the Tk window smoke test errored because
this Python has no `tkinter`, so the full suite did not pass.
Native browser CAD loading is checked separately.

## Observed run (does not count as scoring acceptance)

See [the current movement/scoring audit](MOVEMENT_SCORING_AUDIT.md) for the
70/80 physical and 28/36 ideal movement results, corrected middle/low feeder
mapping, and the separate goal-alignment and ghost-origin errors. The results
below are historical.

Artifacts: `bin/metro-game-validation-3tlF7o/run-final/`. These are ignored local
validation artifacts, including build log, shared library, manifest, recording
and summary. Additional smoke, host-test and browser artifacts are under the
same parent. The prior recording and final rebuild have the
same 8.290 s routine return and zero score.

- Simulated 15 seconds in 1 ms native physics steps, captured every 10 ms.
- Original `leftSideSevenMiddle` returned at 8.290 seconds.
- 61 blocks throughout: 36 field, 24 loader, one configured preload.
- Held count went 1 → 2 at 1.440 s, then 2 → 1 at 3.700 s and 1 → 0 at 3.790 s.
- Both alliance scores remained zero. The blocks were released away from a
  scoring location in this provisional setup. No blocks were placed into goals
  by hand to fake a seven-block result.
- Scraper asserted at exactly 0.500 s; its callback finished its own 300 ms
  sleep at 0.800 s while the main routine continued issuing drive commands.
- The routine's delayed XY reset changed only the estimated coordinates. The
  reported maximum truth/estimate separation was 41.09 inches, but that number
  includes the coordinate reset, so most of it is not drift.

## Fidelity boundaries and remaining requirements

The original preview's physical start was provisionally `(13.5, -46)` inches,
heading 0°, red, one preload, based on the routine comment. That was on the wrong
side of the park zone. The runner's default is now `(-13.5, -46)` with the same
heading, alliance and preload; one run moved it another eight inches left, to
`(-21.5, -46)`. Nobody measured the exact placement.
Wheel diameter 3.25 in and gear ratio 48/36 motor turns per wheel turn use native
six-motor-450 assumptions. Mass defaults to 14 lb (see
[METRO_FRICTION.md](METRO_FRICTION.md)); this section's earlier runs used 16.5 lb. Track width 11.375 in is from
Metro. Physical travel per motor turn is 7.66 in versus Metro's 9.06 in
calibration. Real hardware dimensions/gearing/mass and placement are needed
before this can predict the real robot. CLI flags expose these assumptions.

The original startup uses `trackNoOdomWheel` instead of the declared vertical
tracker. Original turning odometry adds the same half-track sign for both
sides, so a synthetic pure 90° yaw reports a false translation of about 5.66 by
5.69 inches. This section describes runs that kept that defect on purpose,
without swapping in mclib's odometry; the ghost turning-offset correction above
fixed it for later runs. Encoder taring occurs only at startup in this routine.
Original IMU gain, minimum voltage, PID, exit behavior, scheduler and direct
motor-command overwrites are also preserved. The host runs deterministic
creation-order cooperative tasks; real V5 preemptive timing is not calibrated.

Native physics includes chassis/motor/battery/slip/sensors, field collisions,
block contacts and native scoring. It uses vexsim's robot CAD and 18-sided
Push Back blocks. The CAD robot is vexsim's native asset and was never checked
against the real robot's build. Native rendering uses its own
optimized level of detail, CAD goals/loaders/park zones and a drawn perimeter.

The native intake uses a constrained path and command-scaled stages. The
Metro output-to-long/center/lower route mapping is approximate; native route
changes can reposition held blocks along that constrained path. Electrical
motor signs are converted to travel along the selected route. PRIME backs
blocks down the long route with the gate closed; the optical proxy follows
their simulated position instead of a scripted sensor clear.
Optical and distance input is an uncalibrated geometric region that reads the
simulated block positions. Scraper/wing states are recorded but have no collision
geometry; intake electrical loading and pneumatic dynamics are not modeled.
The simulation adds no opponents, extra robot preloads or human match-loading.

The viewer's native right HUD uses native X/Y and CCW heading. The left Metro
panel converts truth to +X right, +Y forward, clockwise degrees
and separately labels original local odometry. Recorded telemetry includes
wheel forces/slip, motor current/RPM/temperature, battery, drive commands,
heading target, intake state and task/output events. Browser animation selects
recorded frames only; it does not integrate RPM or simulate new collisions.
