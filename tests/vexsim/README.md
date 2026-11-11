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
two parallel legs. Both need the Override field shapes that vexsim's
`tools/override_cad.py` cuts from VEX's field CAD. They are made from VEX's
CAD, so they are in a private repo, `kevinyhe/vexsim-cad`, and the job copies
them into `vexsim/cad/override/`. It reads that repo with a read-only deploy
key, the `VEXSIM_CAD_DEPLOY_KEY` secret on mclib. Without the secret the job
fails and says so, except on pull requests from forks, which get no secrets:
there the planner steps skip. To add or replace the key:

```sh
ssh-keygen -t ed25519 -N "" -C "mclib CI reads vexsim-cad" -f /tmp/vexsim_cad_deploy
gh api repos/kevinyhe/vexsim-cad/keys -f title="mclib CI (read-only)" \
  -f key="$(cat /tmp/vexsim_cad_deploy.pub)" -F read_only=true
gh secret set VEXSIM_CAD_DEPLOY_KEY -R kevinyhe/mclib < /tmp/vexsim_cad_deploy
rm /tmp/vexsim_cad_deploy /tmp/vexsim_cad_deploy.pub
```

The job installs `mujoco` and `numpy` at the versions in
[requirements.txt](requirements.txt), the ones the planner tests pass with.

## Scenarios

- Three drivetrains: four-motor 200 RPM, six-motor 450 RPM, and a faster
  four-inch-wheel `speed_base`.
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
