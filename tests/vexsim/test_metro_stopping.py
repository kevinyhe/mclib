"""Check Metro output mapping and physical stopping without field collisions.

Run: python3 -B tests/vexsim/test_metro_stopping.py
These are model regressions, not measured hardware stopping distances.
"""
import math
import os
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(os.environ.get(
    "VEXSIM_PATH", Path(__file__).resolve().parents[2].parent / "vexsim")).resolve()))

from vexsim import Simulator, presets
from vexsim.sim import SimConfig
from vexsim.units import INCH
from metro_run import MetroRecorder


class MetroStoppingTests(unittest.TestCase):
    def test_reduced_drive_voltage_preserves_stop_behavior(self):
        sim = Simulator(presets.six_motor_450(mass_lb=14), config=SimConfig(log_every=0))
        recorder = MetroRecorder.__new__(MetroRecorder)
        recorder.now, recorder.events, recorder.errors = 0, [], []
        recorder.commands = [(0, 0.0), (0, 0.0)]
        recorder.drive_scale = .9
        recorder.sides = [[m for unit in units for m in unit.motors]
                          for units in (sim.chassis.left_units(), sim.chassis.right_units())]
        for request, expected in ((12, 10.8), (-12, -10.8), (20, 10.8), (6, 5.4)):
            recorder.output(0, 0, 0, request)
            self.assertAlmostEqual(recorder.commands[0][1], expected)
            self.assertEqual(recorder.events[-1]['requested_volts'], request)
        for _ in range(100):
            sim.step(.001)
        before = (sim.state.u, sim.state.v, sim.state.r)
        recorder.output(0, 0, 1, 0)
        self.assertEqual((sim.state.u, sim.state.v, sim.state.r), before)
        self.assertEqual(recorder.commands[0], (1, 0))
        self.assertEqual(recorder.errors, [])

    def test_stop_commands_preserve_momentum_and_coast_farther_than_braking(self):
        distances = {}
        for mode, name in ((1, "coast"), (2, "brake"), (3, "hold")):
            with self.subTest(mode=name):
                chassis = presets.six_motor_450(mass_lb=14)
                chassis.surface_mu = 1.0
                for unit in chassis.wheel_units:
                    unit.y = math.copysign(11.375 * INCH / 2, unit.y)
                sim = Simulator(chassis, config=SimConfig(log_every=0))
                # Exercise the actual C++ callback receiver, without launching
                # a routine that would overwrite the isolated stop command.
                recorder = MetroRecorder.__new__(MetroRecorder)
                recorder.now, recorder.events, recorder.errors = 0, [], []
                recorder.commands = [(0, 0.0), (0, 0.0)]
                recorder.sides = [[m for unit in units for m in unit.motors]
                                  for units in (chassis.left_units(), chassis.right_units())]
                for side in (0, 1):
                    recorder.output(0, side, 0, 6.0)
                for _ in range(500):
                    sim.step(.001)

                def momentum_state():
                    state = sim.state
                    return (state.x, state.y, state.theta, state.u, state.v, state.r,
                            tuple(unit.omega for unit in chassis.wheel_units),
                            tuple(motor.velocity for motor in sim.motors))

                before = momentum_state()
                self.assertGreater(sim.state.speed, 20 * INCH)
                for side in (0, 1):
                    recorder.output(0, side, mode, 0.0)
                self.assertEqual(momentum_state(), before, "Stopping must not erase momentum")
                self.assertEqual(recorder.errors, [])
                self.assertTrue(all(motor.mode.value == name for motor in sim.motors))
                travel, previous = 0.0, (sim.state.x, sim.state.y)
                for tick in range(2000):
                    sim.step(.001)
                    position = (sim.state.x, sim.state.y)
                    travel += math.dist(position, previous)
                    previous = position
                    if tick == 0:
                        self.assertGreater(sim.state.speed, 0)
                        self.assertGreater(travel, 0)
                self.assertGreater(travel, INCH, "The robot must travel while slowing down")
                self.assertLess(sim.state.speed, INCH)
                distances[name] = travel
                print(f"{name}: {travel / INCH:.2f} in traveled in 2 s after stop")
        self.assertGreater(distances["coast"], distances["brake"])
        self.assertGreater(distances["coast"], distances["hold"])


if __name__ == "__main__":
    unittest.main()
