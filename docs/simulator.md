# Simulator and motion builder

Replaying the real C++ controller against the physics simulator, and what those runs prove.

[Documentation index](README.md) · [Project README](../README.md)

Run `python3 tests/vexsim/builder/server.py` and open **http://127.0.0.1:8765**
to build autonomous sequences and replay the C++ controller driving
`../vexsim`. The workbench compares the simulated robot's true position with
what odometry reported, keeps failed runs, and checks the physics before it
checks the controller. See the [builder guide](../tests/vexsim/builder/README.md)
and [validation findings](../tests/vexsim/BUILDER_VALIDATION.md).

Simulation does not replace measuring your robot. The motions that still fail at
default gains are listed in [accuracy.md](accuracy.md); the tolerances were not
loosened to make them pass.
