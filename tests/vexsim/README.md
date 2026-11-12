# Physics tests

Compiles mclib's motion routines, PID and odometry into a shared library and
runs them against the [vexsim](https://github.com/kevinyhe/vexsim) physics
engine, checked out at `../vexsim`:

```sh
git clone https://github.com/kevinyhe/vexsim ../vexsim
```

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
| `--tracking-mode MODE` | `drive` (default): drive encoders only. `two`: forward and sideways tracking wheels. `vertical` / `horizontal`: one tracking wheel, drive encoders for the rest |

Output includes a build log, `results.json`, and a physics CSV and controller
trace CSV per scenario. The command exits nonzero if any check fails.

## CI

The `Physics simulator` job in `.github/workflows/ci.yml` runs all four tracking
modes on every push and pull request. Some scenarios fail on main (arcs and
some boomerangs), so the job does not use `run.py`'s exit code. It runs
`check_baseline.py`, which fails only when a scenario listed in
`ci_baseline.json` fails.

When a change makes more scenarios pass, update the baseline in the same PR:

```sh
python3 tests/vexsim/run.py --output /tmp/sim-drive
python3 tests/vexsim/check_baseline.py drive /tmp/sim-drive/results.json --update
python3 tests/vexsim/run.py --tracking-mode two --output /tmp/sim-two
python3 tests/vexsim/check_baseline.py two /tmp/sim-two/results.json --update
```

The `Path planner` job runs the planner tests and the browser smoke test as
two parallel legs. Both need the Override field shapes, which the job copies
from the public [vexsim-cad](https://github.com/kevinyhe/vexsim-cad) into
`vexsim/cad/override/`.

The job installs `mujoco` and `numpy` at the versions in
[requirements.txt](requirements.txt), the ones the planner tests pass with.

## Scenarios

- Four drivetrains: four-motor 200 RPM, six-motor 450 RPM, a faster
  four-inch-wheel `speed_base`, and `override_cascade`, the drive of an
  Override cascade robot (four 450 RPM motors on 2.75 in omnis, 14 lb, one
  sideways tracking wheel at the centre). It has no forward
  tracking wheel, so it runs only in the `drive` and `horizontal` modes. With
  its sideways wheel all 16 moves pass; with drive encoders alone the arcs and
  the boomerang end 4.9-6.9 in off.
- Drive forward and reverse, turns, turn to point, move to point, boomerang,
  arcs and swings, at default gains.
- The same two arcs through `followTrajectory()` (RAMSETE):
  `ramsete_arc` and `ramsete_reverse_arc`, plus `ramsete_s_curve` (90° right
  then 90° left, 24 in radius, to (48, 48)). `stress/ramsete_*` runs the arc
  at low battery, low grip and from a start 3.6 in and 8° off the path, and
  the `safety/*/9` cases inject each fault and a timeout into it. Before each, the runner measures
  the preset's kS, kV and effective track width in the simulator the way
  `docs/motion.md` describes, then plans at 70% of top speed.
  `--ramsete-b`, `--ramsete-zeta` and `--ramsete-accel` change the tuning.
- Low battery, reduced traction and encoder-only heading.
- `fusion/*`: point moves, boomerangs, arcs and a RAMSETE arc with a
  `PoseFilter` correcting odometry from four distance sensors. The readings
  are computed from the true pose against a 144 in box around the start, with
  2% noise and a 78 in range, every 20 ms. The filter's position is fed back
  into odometry by at most 60% of the distance moved each tick. Tracking wheels
  get a lower odometry noise (0.0005 in²/in) than drive encoders (0.09).
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
`ci_baseline.json` lists the scenarios that pass in each tracking mode.

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
- Every simulated drive has omni wheels on all corners. A drive with
  traction wheels slides sideways less in turns.
- Holonomic drives, mechanisms and path following are covered by the host tests
  instead.

## Tire grip

vexsim's tire grip on the tiles is partly guessed: sideways grip of an omni
(0.26 static, 0.22 sliding) comes from carpet, and the sliding share of
forward grip (0.81) from nothing. `fit_grip.py` fits all four to logs from
your own robot.

On the robot, run `characterizeGrip()` and `characterizeWallPush()` through a
`GripLog` (`mclib/control/characterize.hpp`);
[examples/tire_grip.cpp](../../examples/tire_grip.cpp) is a whole program.

| Runs | Phase | What it shows | Farthest from the start, in vexsim |
| --- | --- | --- | --- |
| Spins at 4, 8, 12 V for 1.5 s, both ways | 4 | how hard the wheels scrub turning in place | 0 in |
| Circles at 6/3, 9/4.5, 12/6 and 12/3 V for 2 s, both ways | 5 | how far the robot slides sideways, on the sideways tracking wheel | 48, 54, 59 and 35 in |
| 12 V launches for 0.8 s, forward and back | 6 | wheelspin off the line | 50 in |
| A push into a wall, 0 to 12 V over 3 s | 7 | the voltage where the wheels break loose, and how fast they spin sliding | the front square against a wall |

Every run starts from rest where the robot started. After each one the
robot coasts at 0 V for 1 s, drives back with `moveToPoint()` and
`turnToAngle()` on odometry, and waits 1 s; those rows are phase 0. So
odometry must be running. The first three kinds take about 90 s and the
wall push 4 s.

Space, measured in vexsim on `override_cascade` with vexsim's grip: the
robot's centre went up to 45 in forward, 53 in back, 55 in left and 56 in
right of where it started. Add half the robot on each side. With more grip
the circles are wider: at 1.0 forward and 0.45 sideways, the 9/4.5 V and
12/6 V circles went 72 and 71 in out, and the whole routine 62 in forward, 53 in
back, 65 in left and 63 in right. With little grip the drive encoders count
slip as travel and the returns drift: at 0.6 forward and 0.18 sideways, the
last run started 31 in from the first. Shorten `GripTests::circle_time` or
drop a circle if the space is tight. Then:

```sh
python3 tests/vexsim/fit_grip.py grip000.csv wall000.csv --vexsim ../vexsim
```

It replays every run in vexsim from rest with the logged voltages and battery
voltage, through vexsim's encoder, IMU and rotation sensor models, and picks
the static and sliding grip, forward and sideways, that make the wheel
speeds, turn rate and sideways slide match best. It prints them beside
vexsim's numbers and the `foam.py` entries they replace, the mismatch for
each kind of run, and how much the match gets worse 10% either side of each
number. Under 2% is marked not pinned: the logs do not show that number.
The fit uses vexsim's plain drive model (`Simulator`); the MuJoCo world uses
the same tire model. On 20 cores it takes 2 to 5 minutes.

Grip is the only thing the fit changes, so anything else vexsim has wrong
about the robot ends up in the grip numbers. Logs from a robot 10% heavier
than the preset, with vexsim's own grip, fit to forward grip 1.04 instead of
0.84, with a mismatch of 0.96 and spins still 10 deg/s off. Logs that match
the robot except for grip fit to a mismatch near 0.01. A large mismatch left
after the fit means the preset's weight, motors or gearing need fixing
before the grip numbers mean anything.

`test_fit_grip.py` checks the fit: mclib's routines run in vexsim with grip
unlike vexsim's own numbers, and the fit must give it back.

```sh
cd tests/vexsim && VEXSIM_PATH=../../../vexsim python3 -m unittest test_fit_grip
```

## Path planner

```sh
../vexsim/.venv/bin/python tests/vexsim/planner/server.py --vexsim ../vexsim
```

Setup for the vexsim checkout is in the [planner guide](planner/README.md).
Open http://127.0.0.1:8765. Plan paths on the field, run whole routines here
with mclib's C++, play them back and export `autonomous()`. See the
[planner guide](planner/README.md).

## Visual replay

Requires Pillow and the DejaVu Sans fonts:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/visualize.py /tmp/my-run
```

Creates `visual-summary.png`, `motion-replay.gif` and `replay-preview.png` for a
full run. Blue is the simulated position, amber is odometry, and the dashed line
is the ideal arc.

## Simulator tests

```sh
cd ../vexsim
DISPLAY= WAYLAND_DISPLAY= PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```
