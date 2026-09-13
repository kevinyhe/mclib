# Physics integration tests

For the supplied **original Metro `leftSideSevenMiddle` routine with the native
robot, Push Back field and game pieces**, see [METRO_GAME_VALIDATION.md](METRO_GAME_VALIDATION.md).
Its separate native replay runs on port 8766 using Metro's controllers with a
documented signed-inner-arc and odometry-offset corrections. Edit the five `POINTS` pairs in [metro_waypoints.py](metro_waypoints.py) to change
point targets. Defaults reproduce the original Metro autonomous exactly; all
other movement and mechanism parameters stay original. Regenerate the recording
after editing (see the Metro validation guide).
Physical and mechanism limits are explicit.

This harness compiles the current mclib C++ blocking motion routines, PID, and
odometry into a shared library and connects their `DriveHardware` interface to
the sibling `vexsim` project. It does not call vexsim's Python motion controllers
or translate mclib's control algorithms into Python.

**Read [FIX_VALIDATION.md](FIX_VALIDATION.md) before interpreting endpoint errors
as hardware predictions.** It records the corrected simulator and algorithms,
passing physics checks and remaining default-gain/slip-related motion failures.
[PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md) preserves the original audit.
Neither numerical validation substitutes for robot measurements. Standalone
audits return nonzero on failed checks and do not modify the sibling simulator.

Run from the mclib root with Python 3 and the host C++ compiler available:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py
```

The default simulator path is `../vexsim`. No dependency installation or simulator
modifications are required. The shared library, build log, results JSON, physics
CSVs, and controller/odometry trace CSVs go into a newly allocated `/tmp` directory.
Use `--output /tmp/my-run` to choose a location; reusing a location replaces that
run's artifacts. Existing files unrelated to the run are not removed.

Useful focused runs:

```sh
python3 tests/vexsim/run.py --vexsim ../vexsim --filter safety/
python3 tests/vexsim/run.py --filter six_motor_450/ --motion-timeout-ms 10000
python3 tests/vexsim/run.py --filter wall/
```

The harness returns a nonzero status when any acceptance check fails. These are
real failures to inspect, not expected-failure tests that are silently counted
as passing. See `REPORT.md` for the initial validation findings.

## Interactive motion builder

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/server.py
```

Open **http://127.0.0.1:8765** to edit motion sequences, run the actual C++
controller, replay truth versus odometry, and compare preserved runs.
The [builder guide](builder/README.md) explains the staged physics-first
validation workflow and the explicit drive-encoder/tracking-wheel variants.

## Visual replay

With Pillow and the Linux DejaVu Sans fonts available, render a completed full
matrix run without rerunning the physics:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/visualize.py /tmp/my-run
```

This creates `visual-summary.png`, `motion-replay.gif`, and a still
`replay-preview.png` beside the run artifacts. The summary highlights the
initial failing `speed_base` boomerang/arc and `six_motor_450` forward point move;
it requires those three scenarios and their trace CSVs. It is a visualization
of the initial validation findings, not a general-purpose plotting dashboard.
Blue is simulated ground truth, amber is mclib odometry, and the dashed curve
is an ideal geometric arc reference, not a recorded planner trajectory.

## What is measured

- Three unmodified drivetrain presets: a four-motor 200 RPM base, a six-motor
  450 RPM base, and the faster four-inch-wheel `speed_base`.
- Forward/reverse travel, turns, turns toward points, point moves, boomerang,
  forward/reverse arcs, and swings, using mclib's existing default gains.
- A depleted battery, reduced traction, and encoder-derived heading.
- Cancellation, competition disable, IMU/encoder faults, and timeout during all
  eight motion entry points. A fault must actually be injected after the drive
  was energized; an immediate no-op cannot pass the safety checks.
- Wall reset without contact, with fully locked motors, and with the body pinned
  while the wheels remain free to spin.

Nominal moves must return before their four-second deadline, stop the requested
drive output, and finish within 2.5 inches / 5 degrees where those targets apply.
Arc position checks allow 5.5 inches; boomerang now requires the configured
1.5-inch position band. These are explicit simulator
acceptance criteria, not claims that the library promises those tolerances.
Ground-truth endpoint checks include 300 ms of braking/holding after return.
Odometry error is separately measured against ground truth at the exact return
time so stopping drift is not incorrectly counted as odometry error.

## Adapter assumptions and limits

- Public physics ticks advance at 1 ms, with internal integration bounded at
  0.5 ms; the actual C++ `pros::delay()` calls determine the
  controller cadence. Sensor samples retain vexsim's smart-port refresh delay,
  quantization, and seeded IMU error. Simulated shaft velocity is derived from
  its refreshed encoder sensor, not taken directly from true wheel velocity.
- vexsim's field frame is +X forward, +Y left, counterclockwise-positive. The
  adapter maps this to mclib's +Y forward, +X right, clockwise-positive frame.
  IMU heading is unwrapped before passing it to mclib.
- vexsim stores motor turns per wheel turn; mclib's `DriveGeometry` stores wheel
  turns per encoder turn. The adapter takes the reciprocal.
- Motor voltage writes reproduce the wrapper's 12 V clamp and integer-millivolt
  conversion. Coast, brake, and hold map to the simulator's motor modes.
- The encoder-heading case exercises the real heading math helper, not the
  hardware `Chassis` wrapper's constructor or its physical PROS devices.
- Odometry runs synchronously with samples. This tests integration/control
  behavior, not task races or V5 firmware scheduling.
- Locked motors are an idealized hard stall. Pinning the body without locking
  wheels models slipping against a constraint, not collision/contact geometry.
  All motors share the same modeled condition; per-port telemetry is averaged.
- Optional `--tracking-mode two` adds two modeled passive tracking wheels and
  connects their refreshed rotation measurements to the real C++ odometry.
  The default remains `drive`; tracker results are a distinct hardware variant.
  The simulator is not a substitute for hardware validation. Holonomic control,
  mechanisms, and path followers remain outside this particular physics harness.
  The separate host tests still cover those APIs.

The simulator's own baseline can be checked headlessly without creating caches:

```sh
cd ../vexsim
DISPLAY= WAYLAND_DISPLAY= PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```
