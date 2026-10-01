#!/usr/bin/env python3
"""tools/tune_drive.py against logs with known answers.

- Synthetic: a drive that obeys V = kS sgn(v) + kV v + kA a exactly, and
  traces with a known lag, offset or weave. The fit and the advice must
  come back exact.
- End to end (needs ../vexsim): the robot routines in characterize.hpp run
  in the physics simulator, write a real log, and the script's kS, kV and
  effective track width must match what the simulator measures from the
  true robot motion.
"""
import csv
import io
import math
import os
import re
from pathlib import Path
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import tune_drive  # noqa: E402

COLUMNS = ["t_ms", "phase", "left_cmd_V", "right_cmd_V", "left_in", "right_in", "heading_deg",
           "x_in", "y_in", "target_x_in", "target_y_in", "target_heading_deg"]
KS, KV, KA, TRACK = 0.8, 0.15, 0.02, 14.0


def drive_log():
    """Holds at 3, 6, 9 V forward and back, then spins, from the model."""
    rows, t, x, y, heading = [], 0, 0.0, 0.0, 0.0
    def hold(phase, volts, ms):
        nonlocal t, x, y, heading
        v = 0.0
        for _ in range(ms // 10):
            push = abs(volts) - KS
            # First-order approach to the steady speed through kA.
            target = math.copysign(push / KV, volts) if push > 0 else 0.0
            accel = (volts - KS * ((v > 0) - (v < 0) if v else (volts > 0) - (volts < 0)) - KV * v) / KA \
                if push > 0 else -v / 0.01
            v_next = v + accel * 0.01
            if (target - v) * (target - v_next) < 0:
                v_next = target
            if phase == 2:
                heading += math.degrees(2 * v_next / TRACK * 0.01)
            else:
                x += v_next * math.sin(math.radians(heading)) * 0.01
                y += v_next * math.cos(math.radians(heading)) * 0.01
            v = v_next
            left, right = volts, (-volts if phase == 2 else volts)
            rows.append(dict(t_ms=t, phase=phase, left_cmd_V=left, right_cmd_V=right, left_in=0, right_in=0,
                             heading_deg=heading, x_in=x, y_in=y, target_x_in=float("nan"),
                             target_y_in=float("nan"), target_heading_deg=float("nan")))
            t += 10
    for volts in (3, 6, 9):
        for sign in (1, -1):
            hold(1, sign * volts, 1200)
            hold(0, 0, 300)
    for volts in (4, 6, 8):
        for sign in (1, -1):
            hold(2, sign * volts, 1500)
            hold(0, 0, 300)
    return rows


def trace_log(lag=0.0, offset=0.0, weave=0.0):
    """Straight along +Y at 30 in/s; the robot is @p lag behind, @p offset to
    the right, and weaves @p weave in at 2 Hz."""
    rows = []
    for i in range(200):
        t = i * 0.01
        target_y = 30 * t
        rows.append(dict(t_ms=t * 1000, phase=3, left_cmd_V=6, right_cmd_V=6, left_in=0, right_in=0,
                         heading_deg=0, x_in=-offset + weave * math.sin(2 * math.pi * 2 * t),
                         y_in=target_y - lag, target_x_in=0, target_y_in=target_y, target_heading_deg=0))
    return rows


class FitTests(unittest.TestCase):
    def test_feedforward_and_track_from_an_exact_model(self):
        rows = drive_log()
        ff = tune_drive.fit_feedforward(rows)
        self.assertAlmostEqual(ff["ks"], KS, delta=0.01)
        self.assertAlmostEqual(ff["kv"], KV, delta=0.002)
        self.assertAlmostEqual(ff["ka"], KA, delta=0.005)
        self.assertEqual(ff["holds"], 6)
        track = tune_drive.fit_track(rows, ff["ks"], ff["kv"])
        self.assertAlmostEqual(track["track_in"], TRACK, delta=0.2)
        self.assertEqual(track["spins"], 6)

    def test_reads_logger_csv_with_trailer_and_nan(self):
        rows = drive_log()[:50]
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "tune000.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=COLUMNS)
                writer.writeheader()
                for row in rows:
                    writer.writerow({k: ("nan" if isinstance(v, float) and math.isnan(v) else v)
                                     for k, v in row.items()})
                file.write("# rows=50 dropped=0 discarded=0\n")
            parsed = tune_drive.read_rows([path])
        self.assertEqual(len(parsed), 50)
        self.assertTrue(math.isnan(parsed[0]["target_x_in"]))

    def test_trace_advice(self):
        good = tune_drive.advice(tune_drive.analyse_trace(trace_log()))
        self.assertIn("Leave the gains", good[0])
        behind = tune_drive.analyse_trace(trace_log(lag=3))
        self.assertAlmostEqual(behind["mean_along_in"], 3, places=6)
        self.assertIn("feedforward is too weak", tune_drive.advice(behind)[0])
        aside = tune_drive.analyse_trace(trace_log(offset=2))
        self.assertAlmostEqual(aside["mean_abs_side_in"], 2, places=6)
        self.assertIn("raise b", tune_drive.advice(aside)[0])
        weaving = tune_drive.analyse_trace(trace_log(weave=2))
        self.assertGreater(weaving["side_crossings_per_s"], 1)
        self.assertIn("weaving", tune_drive.advice(weaving)[0])

    def test_script_output(self):
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "tune.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=COLUMNS)
                writer.writeheader()
                writer.writerows(drive_log() + trace_log(offset=2))
            stdout = sys.stdout
            sys.stdout = out
            try:
                self.assertEqual(tune_drive.main([str(path)]), 0)
            finally:
                sys.stdout = stdout
        text = out.getvalue()
        kv = re.search(r"follow\.feedforward\.kV = ([\d.]+)_V / mclib::units::inps;", text)
        track = re.search(r"follow\.track_width = ([\d.]+)_in;", text)
        self.assertAlmostEqual(float(kv.group(1)), KV, delta=0.002)
        self.assertAlmostEqual(float(track.group(1)), TRACK, delta=0.2)
        self.assertIn("raise b", text)


VEXSIM = Path(os.environ.get("VEXSIM_PATH", ROOT.parent / "vexsim")).resolve()


@unittest.skipUnless((VEXSIM / "vexsim/sim.py").is_file(), "needs the vexsim checkout")
class SimulatorTests(unittest.TestCase):
    def test_robot_routines_fit_what_the_simulator_measures(self):
        sys.path.insert(0, str(ROOT / "tests/vexsim"))
        sys.path.insert(0, str(VEXSIM))
        import run
        with tempfile.TemporaryDirectory() as scratch:
            lib = run.build(Path(scratch))
            truth = run.measure_feedforward(lib, "six_motor_450", "two", {})
            run.PhysicsBridge(lib, "six_motor_450", tracking_mode="two")
            log = Path(scratch) / "tune000.csv"
            self.assertEqual(lib.sim_characterize(str(log).encode(), 3), 0)
            rows = tune_drive.read_rows([log])
        ff = tune_drive.fit_feedforward(rows)
        track = tune_drive.fit_track(rows, ff["ks"], ff["kv"])
        # The routine fits from odometry during fixed holds; the simulator's
        # own measure uses the true body speed at two holds. Within a few %.
        self.assertAlmostEqual(ff["ks"], truth["ks"], delta=0.05)
        self.assertAlmostEqual(ff["kv"], truth["kv"], delta=0.03 * truth["kv"])
        self.assertAlmostEqual(track["track_in"], truth["track_in"], delta=0.05 * truth["track_in"])
        self.assertGreater(ff["ka"], 0)


if __name__ == "__main__":
    unittest.main()
