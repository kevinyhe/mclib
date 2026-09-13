# Physics integration tests

For the original Metro `leftSideSevenMiddle` routine with the native robot,
Push Back field and game pieces, see [METRO_GAME_VALIDATION.md](METRO_GAME_VALIDATION.md).
Its native replay runs on port 8766 using Metro's controllers, with two
documented corrections: the signed inner arc and the odometry offset. To change
point targets, edit the five `POINTS` pairs in [metro_waypoints.py](metro_waypoints.py).
The defaults reproduce the original Metro autonomous, and every other movement
and mechanism parameter stays original. Regenerate the recording after editing
(see the Metro validation guide), which also lists the physical and mechanism
limits.

This harness compiles the current mclib C++ blocking motion routines, PID, and
odometry into a shared library and connects their `DriveHardware` interface to
the sibling `vexsim` project. It does not call vexsim's Python motion controllers
or translate mclib's control algorithms into Python.

Read [FIX_VALIDATION.md](FIX_VALIDATION.md) before treating endpoint errors as
predictions for a real robot. It records the corrected simulator and algorithms,
the passing physics checks, and the motion failures that remain at default gains
because of wheel slip. [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md) preserves
the original audit. Neither replaces measuring the robot. Standalone
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

The harness returns a nonzero status when any acceptance check fails. Each
failure is a real one to inspect; none are marked as expected. See `REPORT.md` for the initial validation findings.

## Interactive motion builder

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/server.py
```

Open **http://127.0.0.1:8765** to edit motion sequences, run the C++
controller, replay true position against odometry, and compare saved runs.
The [builder guide](builder/README.md) explains the validation workflow, which
checks physics before the controller, and the drive-encoder and tracking-wheel
variants.

## Visual replay

With Pillow and the Linux DejaVu Sans fonts available, render a completed full
matrix run without rerunning the physics:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/visualize.py /tmp/my-run
```

This creates `visual-summary.png`, `motion-replay.gif`, and a still
`replay-preview.png` beside the run artifacts. The summary highlights the
initial failing `speed_base` boomerang/arc and `six_motor_450` forward point move;
it requires those three scenarios and their trace CSVs. It only plots those
initial findings. Blue is simulated ground truth, amber is mclib odometry, and
the dashed curve is the ideal geometric arc, drawn for reference.

## What is measured

- Three unmodified drivetrain presets: a four-motor 200 RPM base, a six-motor
  450 RPM base, and the faster four-inch-wheel `speed_base`.
- Forward/reverse travel, turns, turns toward points, point moves, boomerang,
  forward/reverse arcs, and swings, using mclib's existing default gains.
- A depleted battery, reduced traction, and encoder-derived heading.
- Cancellation, competition disable, IMU/encoder faults, and timeout during all
  eight motion entry points. Each fault is injected after the drive is
  energized, so a routine that returns without moving cannot pass.
- Wall reset without contact, with fully locked motors, and with the body pinned
  while the wheels remain free to spin.

Nominal moves must return before their four-second deadline, stop the requested
drive output, and finish within 2.5 inches / 5 degrees where those targets apply.
Arc position checks allow 5.5 inches; boomerang now requires the configured
1.5-inch position band. These are the simulator's acceptance criteria; the
library does not promise those tolerances on a robot.
Ground-truth endpoint checks include 300 ms of braking/holding after return.
Odometry error is separately measured against ground truth at the exact return
time, so drift while stopping is not counted as odometry error.

## Adapter assumptions and limits

- Public physics ticks advance at 1 ms, with internal integration bounded at
  0.5 ms; the C++ `pros::delay()` calls set the controller cadence. Sensor samples retain vexsim's smart-port refresh delay,
  quantization, and seeded IMU error. Simulated shaft velocity comes from the
  refreshed encoder reading instead of the true wheel velocity.
- vexsim's field frame is +X forward, +Y left, counterclockwise-positive. The
  adapter maps this to mclib's +Y forward, +X right, clockwise-positive frame.
  IMU heading is unwrapped before passing it to mclib.
- vexsim stores motor turns per wheel turn; mclib's `DriveGeometry` stores wheel
  turns per encoder turn. The adapter takes the reciprocal.
- Motor voltage writes reproduce the wrapper's 12 V clamp and integer-millivolt
  conversion. Coast, brake, and hold map to the simulator's motor modes.
- The encoder-heading case exercises the heading math helper. It does not run
  the `Chassis` constructor or any PROS devices.
- Odometry runs synchronously with samples. This tests integration and control,
  and cannot catch task races or V5 firmware scheduling problems.
- Locked motors are an idealized hard stall. Pinning the body without locking
  wheels models slipping against a constraint; there is no collision geometry.
  All motors share the same modeled condition; per-port telemetry is averaged.
- Optional `--tracking-mode two` adds two modeled passive tracking wheels and
  connects their refreshed rotation measurements to the real C++ odometry.
  The default remains `drive`; tracker results are a distinct hardware variant.
  The simulator does not replace testing on hardware. Holonomic control,
  mechanisms and path followers are not in this physics harness; the host tests
  cover those APIs.

The simulator's own baseline can be checked headlessly without creating caches:

```sh
cd ../vexsim
DISPLAY= WAYLAND_DISPLAY= PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```
