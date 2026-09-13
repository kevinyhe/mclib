# Physics tests

Compiles mclib's motion routines, PID and odometry into a shared library and
runs them against the vexsim physics engine, checked out at `../vexsim`.

## Running

From the mclib root, with Python 3 and `g++`:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/run.py
```

| Option | Effect |
| --- | --- |
| `--vexsim PATH` | simulator location (default `../vexsim`) |
| `--output DIR` | where results go (default: a new `/tmp` directory) |
| `--filter NAME/` | run matching scenarios, e.g. `safety/`, `wall/`, `six_motor_450/` |
| `--motion-timeout-ms MS` | per-move time limit |
| `--tracking-mode two` | add two tracking wheels |

Output includes a build log, `results.json`, and a physics CSV and controller
trace CSV per scenario. The command exits nonzero if any check fails.

## Scenarios

- Three drivetrains: four-motor 200 RPM, six-motor 450 RPM, and a faster
  four-inch-wheel `speed_base`.
- Drive forward and reverse, turns, turn to point, move to point, boomerang,
  arcs and swings, at default gains.
- Low battery, reduced traction and encoder-only heading.
- Cancellation, competition disable, IMU and encoder faults, and timeout during
  each of the eight motion functions. Faults are injected while the drive is
  powered.
- Wall reset with no contact, with locked motors, and with the robot pinned but
  wheels free.

## Pass criteria

| Move | Requirement |
| --- | --- |
| All | returns within 4 s and stops the drive |
| Drive, turn, point | within 2.5 in and 5° |
| Arc | within 5.5 in |
| Boomerang | within 1.5 in |

Position is measured 300 ms after the move returns. Odometry error is measured
at the moment it returns.

These are simulator criteria. The library does not guarantee them on a robot.
Results are in [docs/accuracy.md](../../docs/accuracy.md).

## Simulation model

- Physics steps every 1 ms, internally at 0.5 ms or finer. The controller runs at
  the rate set by its own `pros::delay()` calls.
- Sensors have vexsim's refresh delay, rounding and seeded IMU noise.
- vexsim uses +X forward, +Y left, counter-clockwise positive. The adapter
  converts to mclib's frame.
- Motor voltage is clamped to 12 V and rounded to whole millivolts, as in the
  mclib motor wrapper.
- Odometry runs in step with the simulation, so task timing is not tested.
- Locked motors are an ideal stall. A pinned robot slides against a constraint;
  there is no wall collision.
- `driveTo` and `curveCircle` use drive encoders for distance in both tracking
  modes.
- Holonomic drives, mechanisms and path following are covered by the host tests
  instead.

## Motion builder

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/builder/server.py
```

Open http://127.0.0.1:8765. See the [builder guide](builder/README.md).

## Visual replay

Requires Pillow and the DejaVu Sans fonts:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/visualize.py /tmp/my-run
```

Creates `visual-summary.png`, `motion-replay.gif` and `replay-preview.png` for a
full run. Blue is the simulated position, amber is odometry, and the dashed line
is the ideal arc.

## Metro routine

[METRO_GAME_VALIDATION.md](METRO_GAME_VALIDATION.md) covers running the Metro
`leftSideSevenMiddle` autonomous with the full Push Back field on port 8766.
Change its target points in [metro_waypoints.py](metro_waypoints.py).

## Simulator tests

```sh
cd ../vexsim
DISPLAY= WAYLAND_DISPLAY= PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

## Reports

| Report | Contents |
| --- | --- |
| [BUILDER_VALIDATION.md](BUILDER_VALIDATION.md) | current results and open failures |
| [FIX_VALIDATION.md](FIX_VALIDATION.md) | simulator and controller fixes |
| [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md) | simulator physics audit |
| [SEQUENCE_VALIDATION.md](SEQUENCE_VALIDATION.md) | multi-step sequences |
| [DEBUGGER_VALIDATION.md](DEBUGGER_VALIDATION.md) | builder playback and telemetry |
| [REPORT.md](REPORT.md) | first test run |
