# Simulator

`tests/vexsim/` runs the C++ motion routines and odometry against the vexsim
physics engine, checked out next to this repository at `../vexsim`.

## Motion builder

```sh
python3 tests/vexsim/builder/server.py
```

Open http://127.0.0.1:8765 to build a sequence, run it, and compare the
simulated position with odometry. See the
[builder guide](../tests/vexsim/builder/README.md).

## Test matrix

```sh
python3 tests/vexsim/run.py
```

See [tests/vexsim/README.md](../tests/vexsim/README.md) for options and
[known limits](accuracy.md) for results.
