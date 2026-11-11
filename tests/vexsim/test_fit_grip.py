"""fit_grip.py gives back the grip vexsim was given.

mclib's characterizeGrip() and characterizeWallPush() run in vexsim on the
override_cascade drive with grip unlike vexsim's own numbers, and log through
the same GripLog and logger a robot uses, with vexsim's encoder, IMU and
rotation sensor models in between. fit_grip.py then fits those logs.

    cd tests/vexsim && VEXSIM_PATH=../../../vexsim python3 -m unittest test_fit_grip

About 5 minutes on 20 cores; the fit replays each run about 175 times.
"""
import multiprocessing
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
VEXSIM = Path(os.environ.get("VEXSIM_PATH", HERE.parents[2].parent / "vexsim")).resolve()
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(VEXSIM))

import fit_grip  # noqa: E402
import run  # noqa: E402

GIVEN = fit_grip.Grip(long_static=0.72, long_share=0.70, lat_static=0.34, lat_share=0.75)
TOLERANCE = fit_grip.Grip(long_static=0.02, long_share=0.02, lat_static=0.01, lat_share=0.02)


def give_grip(bridge, grip: fit_grip.Grip) -> None:
    for unit in bridge.sim.chassis.wheel_units:
        if unit.wheel.driven:
            tire = unit.wheel.tire
            tire.mu_long_static = grip.long_static
            tire.mu_long_slide = grip.long_static * grip.long_share
            tire.mu_lat_static = grip.lat_static
            tire.mu_lat_slide = grip.lat_static * grip.lat_share


def write_logs(directory: Path, grip: fit_grip.Grip):
    build = directory / "build"
    build.mkdir()
    lib = run.build(build)
    logs = []
    for which, name, against_wall in ((1, "grip.csv", False), (2, "wall.csv", True)):
        bridge = run.PhysicsBridge(lib, "override_cascade", tracking_mode="horizontal",
                                   constrained=against_wall)
        give_grip(bridge, grip)
        sideways = next(u for u in bridge.sim.chassis.wheel_units if u.name == "track_perp")
        path = directory / name
        result = lib.sim_characterize_grip(str(path).encode(), which, 2 * sideways.radius / bridge.inch)
        if result != 0:
            raise RuntimeError(f"the grip routine for {name} ended with MotionResult {result}")
        logs.append(path)
    return logs


HEADER = "t_ms,phase,left_cmd_V,right_cmd_V,left_in,right_in,heading_deg,sideways_in,battery_V\n"


def spin_rows(heading_step: float) -> str:
    return "".join(f"{10 * i},4,4,-4,{0.15 * i},{-0.15 * i},{heading_step * i},0,14.2\n" for i in range(20))


class ReadingLogsTest(unittest.TestCase):
    def write(self, directory: str, text: str) -> Path:
        path = Path(directory) / "grip000.csv"
        path.write_text(text)
        return path

    def test_a_log_with_dropped_rows_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write(directory, HEADER + spin_rows(1.4) + "# rows=20 dropped=3 discarded=0\n")
            with self.assertRaisesRegex(SystemExit, "dropped 3 rows"):
                fit_grip.read_log(path)

    def test_a_log_without_the_grip_columns_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write(directory, "t_ms,phase,left_cmd_V\n0,4,4\n")
            with self.assertRaisesRegex(SystemExit, "no right_cmd_V"):
                fit_grip.read_log(path)

    def test_a_heading_that_turns_the_wrong_way_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write(directory, HEADER + spin_rows(-1.4))
            runs = fit_grip.split_runs(path, fit_grip.read_log(path))
            with self.assertRaisesRegex(SystemExit, "heading goes down"):
                fit_grip.check_signs(runs)

    def test_a_sideways_wheel_that_counts_the_wrong_way_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            right_turn = "".join(f"{10 * i},5,12,3,{0.5 * i},{0.1 * i},{2.0 * i},{0.3 * i},14.2\n"
                                 for i in range(40))
            path = self.write(directory, HEADER + right_turn)
            runs = fit_grip.split_runs(path, fit_grip.read_log(path))
            with self.assertRaisesRegex(SystemExit, "reverse its port"):
                fit_grip.check_signs(runs)

    def test_a_run_that_starts_moving_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            moving = "".join(f"{10 * i},0,0,0,{0.3 * i},{0.3 * i},0,0,14.2\n" for i in range(10))
            spin = "".join(f"{100 + 10 * i},4,4,-4,{3 + 0.15 * i},{3 - 0.15 * i},{1.4 * i},0,14.2\n"
                           for i in range(20))
            path = self.write(directory, HEADER + moving + spin)
            with self.assertRaisesRegex(SystemExit, "still moving"):
                fit_grip.split_runs(path, fit_grip.read_log(path))


class FitGripTest(unittest.TestCase):
    def test_the_fit_gives_back_the_grip_the_logs_were_made_with(self):
        with tempfile.TemporaryDirectory() as directory:
            logs = write_logs(Path(directory), GIVEN)
            result = fit_grip.fit(logs, VEXSIM, "override_cascade", multiprocessing.cpu_count())
        print("\n" + fit_grip.report(result, fit_grip.current_grip(VEXSIM, "override_cascade")))
        for (name, _, _), fitted, given, tolerance, rise in zip(
                fit_grip.PARAMETERS, result.grip.as_list(), GIVEN.as_list(), TOLERANCE.as_list(), result.rises):
            with self.subTest(name):
                self.assertAlmostEqual(fitted, given, delta=tolerance)
                self.assertGreater(rise, fit_grip.NOT_PINNED_RISE)


if __name__ == "__main__":
    unittest.main()
