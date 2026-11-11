# Simulator

`tests/vexsim/` runs the C++ motion routines and odometry against the
[vexsim](https://github.com/kevinyhe/vexsim) physics engine, checked out next
to this repository at `../vexsim`.

## Path planner

```sh
../vexsim/.venv/bin/python tests/vexsim/planner/server.py --vexsim ../vexsim
```

The vexsim checkout needs a venv with its `requirements.txt`, Three.js and
the Override field shapes; the
[planner guide](../tests/vexsim/planner/README.md) has the setup commands.
Open http://127.0.0.1:8765. Click the field to place points, see the speeds
mclib plans along the path, press **Run simulation** to drive the routine in
vexsim with mclib's C++, then play it back and write it into your robot
project as C++. The first launch compiles mclib, which takes 1 to 2 minutes.
See the
[planner guide](../tests/vexsim/planner/README.md).

## Tire grip

vexsim's sideways grip for omni wheels and the sliding share of forward grip
are guesses. `characterizeGrip()` and `characterizeWallPush()` log your robot
driving open loop, and `tests/vexsim/fit_grip.py` fits vexsim's grip to the
logs. See [tests/vexsim/README.md](../tests/vexsim/README.md#tire-grip) and
[examples/tire_grip.cpp](../examples/tire_grip.cpp).

## Test matrix

```sh
python3 tests/vexsim/run.py
```

See [tests/vexsim/README.md](../tests/vexsim/README.md) for options and
pass criteria.
