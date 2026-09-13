# Simulator and motion builder

Replaying the real C++ controller against the physics simulator, and what that validation does and does not prove.

[Documentation index](README.md) · [Project README](../README.md)

## Motion builder and simulator validation

Run `python3 tests/vexsim/builder/server.py` and open **http://127.0.0.1:8765**
to build autonomous sequences and replay the actual C++ controller driving
`../vexsim`. The workbench compares physical truth with sensor-based odometry,
preserves failed runs, and supports a staged physics-first validation workflow.
See the [builder guide](../tests/vexsim/builder/README.md) and
[validation findings](../tests/vexsim/BUILDER_VALIDATION.md). Simulation does not
replace measured robot calibration; remaining default-gain/slip failures are
reported separately, not hidden by relaxed tolerances.
