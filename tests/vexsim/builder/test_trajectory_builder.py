#!/usr/bin/env python3
"""The trajectory builder's two outputs, checked against the real code.

- The C++ export for a sequence with every step type compiles against the
  library headers (needs node and g++).
- The planner, run as the server runs it (runner.py --plan in its own
  process), returns plans that go through their waypoints and stay within
  the limits it reports (needs the vexsim checkout; builds the library once).
"""
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
VEXSIM = Path(os.environ.get("VEXSIM_PATH", ROOT.parent / "vexsim")).resolve()

ALL_STEPS = dict(
    name="Every step", preset="six_motor_450", tracking_mode="two", stop_on_failure=True,
    start_pose=dict(x=0, y=0, heading=0),
    steps=[
        dict(type="drive", distance=24, timeout_ms=4000, volts=12),
        dict(type="turn", heading=90, timeout_ms=4000, volts=12),
        dict(type="point", x=24, y=24, direction=1, timeout_ms=4000, volts=12),
        dict(type="turn_to_point", x=0, y=0, direction=-1, timeout_ms=1000),
        dict(type="boomerang", x=12, y=36, heading=45, direction=1, lead=0.5, timeout_ms=4000, volts=10),
        dict(type="arc", heading=180, radius=18, timeout_ms=4000, volts=12),
        dict(type="reverse_arc", heading=90, radius=18, timeout_ms=4000, volts=12),
        dict(type="swing", heading=0, direction=-1, timeout_ms=4000, volts=12),
        dict(type="follow", waypoints=[dict(x=12, y=24), dict(x=-6.5, y=30)], x=0, y=48,
             direction=-1, b=40, timeout_ms=8000, volts=12),
        dict(type="wall_reset", x=0, y=0, heading=0, current_ma=2500, timeout_ms=1000, volts=6),
    ])


@unittest.skipUnless(shutil.which("node") and shutil.which("g++"), "needs node and g++")
class CppExportTests(unittest.TestCase):
    def export(self, spec, plan=None):
        script = ("const { motionCppExport } = require(process.argv[1]);"
                  "process.stdout.write(motionCppExport(JSON.parse(process.argv[2]), JSON.parse(process.argv[3])));")
        done = subprocess.run(["node", "-e", script, str(HERE / "builder.js"), json.dumps(spec),
                               json.dumps(plan)], capture_output=True, text=True, check=True)
        return done.stdout

    def compiles(self, code):
        with tempfile.TemporaryDirectory() as scratch:
            source = Path(scratch) / "autonomous.cpp"
            source.write_text(code)
            done = subprocess.run(["g++", "-std=gnu++20", "-fsyntax-only", f"-I{ROOT / 'include'}",
                                   "-DMCLIB_HOST_BUILD", "-Wno-deprecated-declarations", str(source)],
                                  capture_output=True, text=True)
            return done.returncode, done.stderr

    def test_every_step_type_compiles(self):
        plan = dict(feedforward=dict(ks=0.699, kv=0.1315, top_ips=86, track_in=13.45),
                    limits=dict(max_velocity_ips=60.18, max_acceleration_ips2=60,
                                max_lateral_acceleration_ips2=60))
        for spec, answer in ((ALL_STEPS, plan), (ALL_STEPS, None),
                             (dict(ALL_STEPS, stop_on_failure=False), plan)):
            code = self.export(spec, answer)
            status, errors = self.compiles(code)
            self.assertEqual(status, 0, errors[-2000:])


@unittest.skipUnless((VEXSIM / "vexsim/sim.py").is_file(), "needs the vexsim checkout")
class PlannerTests(unittest.TestCase):
    def test_plan_goes_through_waypoints_within_its_limits(self):
        request = dict(preset="six_motor_450", tracking_mode="two", paths=[
            dict(start=dict(x=0, y=0), points=[[12, 24], [0, 48]], direction=1, b=50),
            dict(start=dict(x=0, y=48), points=[[-12, 24], [0, 0]], direction=-1, b=50),
            dict(start=dict(x=0, y=0), points=[[0, 0]], direction=1),
        ])
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "plan.json"
            path.write_text(json.dumps(request))
            done = subprocess.run([sys.executable, "-B", str(HERE / "runner.py"), "--plan", str(path),
                                   "--vexsim", str(VEXSIM), "--cache", str(Path(scratch) / "cache")],
                                  cwd=ROOT, capture_output=True, text=True, timeout=600)
        self.assertEqual(done.returncode, 0, done.stderr[-2000:])
        answer = json.loads(done.stdout)
        top = answer["limits"]["max_velocity_ips"]
        self.assertGreater(answer["feedforward"]["kv"], 0)
        out, back, empty = answer["plans"]
        for plan, waypoint, end in ((out, (12, 24), (0, 48)), (back, (-12, 24), (0, 0))):
            self.assertTrue(plan["valid"])
            states = plan["states"]
            self.assertEqual(states[0][3], 0.0)
            self.assertAlmostEqual(states[-1][1], end[0], places=3)
            self.assertAlmostEqual(states[-1][2], end[1], places=3)
            # Sampled every ~20 ms, so the waypoint is passed within a tick.
            nearest = min(math.dist(s[1:3], waypoint) for s in states)
            self.assertLess(nearest, 1.0)
            self.assertTrue(all(0 <= s[3] <= top + 1e-6 for s in states))
            self.assertGreater(plan["duration_s"], 0)
        # Reversed plans run the same geometry backward; speeds are reported
        # as magnitudes.
        self.assertAlmostEqual(out["length_in"], back["length_in"], places=6)
        # Start and only point the same: nothing to follow.
        self.assertFalse(empty["valid"])


if __name__ == "__main__":
    unittest.main()
