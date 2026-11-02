"""Routine checks, the C++ export, and the engine end to end in vexsim.

The engine tests compile mclib once (1 to 2 minutes). Set
MCLIB_PLANNER_CACHE to a folder to keep that build between test runs, and
VEXSIM_PATH to the vexsim checkout (default: ../vexsim next to mclib).
"""
import copy
import json
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
sys.path.insert(0, str(HERE))
from export_cpp import export, exported_helpers  # noqa: E402
from project import (DEFAULT_PROJECT, ProjectError, carries_speed, config, step_time_limit,  # noqa: E402
                     strict_json, validate_project)

VEXSIM = Path(os.environ.get("VEXSIM_PATH", ROOT.parent / "vexsim")).resolve()


def example(name):
    raw = strict_json((HERE / "examples" / f"{name}.json").read_text())
    raw.pop("description")
    return raw


def path_step(points, reverse=(), speeds=None):
    speeds = speeds or [None] * len(points)
    return dict(type="path", points=[dict(x=x, y=y) for x, y in points],
                sections=[dict(speed_percent=speed, reverse=i in reverse) for i, speed in enumerate(speeds)])


class ProjectTests(unittest.TestCase):
    def test_default_project_is_valid_and_config_is_plain_json(self):
        before = copy.deepcopy(DEFAULT_PROJECT)
        project = validate_project(DEFAULT_PROJECT)
        self.assertEqual(project["steps"][0]["type"], "path")
        self.assertEqual(DEFAULT_PROJECT, before)
        json.dumps(config(), allow_nan=False)

    def test_every_example_is_valid(self):
        for path in sorted((HERE / "examples").glob("*.json")):
            with self.subTest(path.name):
                validate_project(example(path.stem))

    def test_errors_name_the_step_and_the_field(self):
        cases = [
            (dict(type="turn", heading=90, speed=3), "Step 1 (Turn to heading) has fields the planner does not know: speed"),
            (dict(type="turn", heading=True), "Step 1 (Turn to heading): Heading must be a number"),
            (dict(type="drive", distance=900), "Step 1 (Drive straight): Distance must be between -500 and 500 in, not 900"),
            (dict(type="arc", heading=90, radius=0.5), "Step 1 (Arc): Radius must be at least 1 in either way, not 0.5"),
            (dict(type="boomerang", x=0, y=0, heading=0, lead=1), "Step 1 (Drive to a pose): Lead must be between 0 and 0.99, not 1"),
            (path_step([(0, 0)], speeds=[3]), "Step 1 (Path), section 1: Speed limit must be between 5 and 100 %, not 3"),
            (dict(type="path", points=[dict(x=0, y=0)], sections=[]), "Step 1 (Path): needs one section per point (1), found 0"),
        ]
        for step, message in cases:
            with self.subTest(message):
                with self.assertRaises(ProjectError) as caught:
                    validate_project(dict(steps=[step]))
                self.assertEqual(str(caught.exception), message)
                self.assertEqual(caught.exception.step, 0)

    def test_wall_reset_must_be_last(self):
        with self.assertRaises(ProjectError) as caught:
            validate_project(dict(steps=[dict(type="wall_reset", x=0, y=63, heading=0), dict(type="turn", heading=0)]))
        self.assertIn("a wall reset must be the last step", str(caught.exception))

    def test_keep_moving_is_refused_where_mclib_cannot_chain(self):
        cases = [
            ([dict(type="turn", heading=90, chain=True)],
             "Step 1 (Turn to heading): cannot keep moving into the next step. The last step has to stop: "
             "there is no next step to keep moving into."),
            ([dict(type="turn_to_point", x=0, y=24, chain=True), dict(type="turn", heading=0)],
             "Step 1 (Turn to face a point): cannot keep moving into the next step. turnToPoint() has no exit "
             "option in mclib, so it always stops."),
            ([dict(type="drive", distance=12, chain="yes"), dict(type="turn", heading=0)],
             "Step 1 (Drive straight): Keep moving into the next step must be true or false"),
        ]
        for steps, message in cases:
            with self.subTest(message):
                with self.assertRaises(ProjectError) as caught:
                    validate_project(dict(steps=steps))
                self.assertEqual(str(caught.exception), message)
        project = validate_project(dict(steps=[dict(type="drive", distance=12, chain=True), dict(type="turn", heading=0)]))
        self.assertEqual([step["chain"] for step in project["steps"]], [True, False])

    def test_speed_carries_only_between_chained_paths_in_the_same_direction(self):
        def project(*steps):
            return validate_project(dict(steps=list(steps)))
        chained = dict(path_step([(0, 0)]), chain=True)
        self.assertTrue(carries_speed(project(chained, path_step([(0, 24)])), 0))
        self.assertFalse(carries_speed(project(chained, path_step([(0, -24)], reverse=[0])), 0))
        self.assertFalse(carries_speed(project(chained, dict(type="turn", heading=0)), 0))
        self.assertFalse(carries_speed(project(path_step([(0, 0)]), path_step([(0, 24)])), 0))

    def test_files_with_repeated_keys_or_nan_are_refused(self):
        with self.assertRaisesRegex(ValueError, "repeats the key \"name\""):
            strict_json('{"name": "a", "name": "b"}')
        with self.assertRaisesRegex(ValueError, "NaN, which is not a number"):
            strict_json('{"x": NaN}')
        with self.assertRaisesRegex(ValueError, "not a path planner project"):
            validate_project(dict(format="something-else"))

    def test_automatic_path_time_limit_is_planned_time_plus_settle(self):
        step = validate_project(dict(steps=[path_step([(0, 0)])]))["steps"][0]
        self.assertIsNone(step["timeout_ms"])
        self.assertEqual(step_time_limit(step, dict(duration_s=2.345)), 6400)
        step["timeout_ms"] = 3000
        self.assertEqual(step_time_limit(step, dict(duration_s=2.345)), 3000)

    def test_automatic_time_limit_of_a_path_that_keeps_moving_has_no_settle(self):
        steps = [dict(path_step([(0, 0)]), chain=True), dict(type="turn", heading=0)]
        step = validate_project(dict(steps=steps))["steps"][0]
        self.assertEqual(step_time_limit(step, dict(duration_s=2.345)), 2900)


class ExportTests(unittest.TestCase):
    SETTINGS = dict(feedforward=dict(ks=.58, kv=.131, top_ips=87, track_in=14.1), ramsete_b=50, ramsete_zeta=.7,
                    accel_ips2=60, cornering_ips2=60, track_width_in=14.1, top_speed_ips=87)

    def every_step(self):
        raw = example("mixed_steps")
        raw["steps"][3:3] = [dict(type="reverse_arc", heading=0, radius=18),
                             dict(type="turn_to_point", x=10, y=10, direction=1),
                             dict(type="point", x=0, y=0, direction=-1),
                             dict(type="swing", heading=45, direction=-1),
                             dict(type="arc", heading=90, radius=-18)]
        return validate_project(raw)

    def compile(self, code):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "autonomous.cpp"
            source.write_text(code)
            done = subprocess.run(["g++", "-std=gnu++20", "-fsyntax-only", "-Wall", "-Wextra", "-DMCLIB_HOST_BUILD",
                                   "-Wno-deprecated-declarations", "-Iinclude", str(source)],
                                  cwd=ROOT, capture_output=True, text=True)
            own_warnings = [line for line in done.stderr.splitlines() if line.startswith(f"{source}:")]
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        self.assertEqual(own_warnings, [])

    def test_export_compiles_and_carries_the_simulated_helpers(self):
        project = self.every_step()
        plans = [dict(duration_s=2.44, length_in=80.0, top_speed_ips=60.0, requested_start_ips=0.0,
                      requested_end_ips=0.0) if step["type"] == "path" else None
                 for step in project["steps"]]
        code = export(project, self.SETTINGS, plans)
        self.assertIn(exported_helpers(), code)
        for call in ("path_planner::planSections", "path_planner::followSections", "turnToAngle(180_deg",
                     "driveTo(-24_in", "curveCircleReverse(0_deg, 18_in", "turnToPoint(10_in, 10_in, 1",
                     "moveToPoint(0_in, 0_in, -1", "swing(45_deg, -1", "curveCircle(90_deg, -18_in",
                     "boomerang(24_in, 24_in, 1, 90_deg, 0.5", "wallReset(24_in, 63_in, 0_deg",
                     "follow.gains.b = 50;", "6500_ms"):
            self.assertIn(call, code)
        self.compile(code)

    def test_export_of_chained_steps_passes_exit_false_and_the_join_speeds(self):
        project = validate_project(dict(steps=[
            dict(type="drive", distance=12, chain=True),
            dict(path_step([(0, 36)]), chain=True),
            path_step([(24, 60)]),
            dict(type="turn", heading=0)]))
        plans = [None,
                 dict(duration_s=1.5, length_in=24.0, top_speed_ips=50.0, requested_start_ips=0.0,
                      requested_end_ips=41.25),
                 dict(duration_s=1.2, length_in=34.0, top_speed_ips=50.0, requested_start_ips=38.5,
                      requested_end_ips=0.0),
                 None]
        code = export(project, self.SETTINGS, plans)
        self.assertIn("driveTo(12_in, 4000_ms, false, 12_V)", code)
        self.assertIn("}, limits, 0, 41.25);", code)
        self.assertIn("}, limits, 38.5, 0);", code)
        self.assertEqual(code.count(", 12_V, false)"), 1)
        self.assertEqual(code.count(", 12_V, true)"), 1)
        self.assertEqual(code.count("Keeps moving into the next step"), 2)
        self.compile(code)

    def test_export_without_a_plan_marks_the_placeholders(self):
        code = export(validate_project(example("s_curve")))
        self.assertIn("placeholder", code.lower())
        self.compile(code)


@unittest.skipUnless((VEXSIM / "vexsim/sim.py").is_file(), f"no vexsim checkout at {VEXSIM}")
class EngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scratch = Path(tempfile.mkdtemp(prefix="planner-test-"))
        cls.cache = Path(os.environ["MCLIB_PLANNER_CACHE"]) if "MCLIB_PLANNER_CACHE" in os.environ else cls.scratch / "cache"

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.scratch)

    def engine(self, command, project, *arguments):
        path = self.scratch / f"{command}-{len(list(self.scratch.iterdir()))}.json"
        path.write_text(json.dumps(project))
        return subprocess.run([sys.executable, "-B", str(HERE / "engine.py"), command, "--project", str(path),
                               *arguments, "--vexsim", str(VEXSIM), "--cache", str(self.cache)],
                              cwd=ROOT, capture_output=True, text=True, timeout=900)

    def run_routine(self, project):
        output = self.scratch / f"run-{len(list(self.scratch.iterdir()))}"
        done = self.engine("run", project, "--output", str(output))
        return done, output

    def test_1_run_plans_simulates_and_scores_every_step(self):
        project = example("speed_zones")
        project["steps"].append(dict(type="turn", heading=90))
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["passed_steps"], 2)
        stages = {stage["id"]: stage for stage in result["stages"]}
        self.assertEqual([stage["status"] for stage in result["stages"]].count("failed"), 0)
        self.assertIn("Planned 1 path", stages["plan"]["detail"])
        path = result["steps"][0]
        self.assertTrue(any(check["id"] == "position" and "limit 2.5 in" in check["message"] for check in path["checks"]))
        self.assertTrue(any(line.startswith("Largest gap from the planned path") for line in path["info"]))
        plan = result["plans"][0]
        self.assertEqual(plan["parts"], 3)
        self.assertGreater(len(result["trace"]), 100)
        self.assertTrue((output / "trace.csv").is_file())

    def test_2_plan_matches_the_plan_the_run_used(self):
        project = example("speed_zones")
        done = self.engine("plan", project)
        answer = json.loads(done.stdout)
        self.assertNotIn("error", answer, answer)
        plan = answer["plans"][0]
        self.assertEqual(plan["parts"], 3)
        self.assertAlmostEqual(plan["states"][-1][3], 24, places=6)
        self.assertAlmostEqual(plan["states"][-1][4], 48, places=6)
        self.assertLessEqual(plan["top_speed_ips"], .9 * answer["settings"]["top_speed_ips"] + 1e-6)
        slow = [state for state in plan["states"] if state[0] == 1]
        self.assertLessEqual(max(abs(state[6]) for state in slow), .35 * answer["settings"]["top_speed_ips"] + 1e-6)

    def test_3_a_section_that_doubles_back_names_the_step_and_the_fix(self):
        project = dict(steps=[path_step([(0, -12)]), dict(type="turn", heading=180), path_step([(0, -24)], reverse=[0])])
        answer = json.loads(self.engine("plan", project).stdout)
        self.assertEqual(answer["step"], 2)
        self.assertIn("Step 3 (Path): section 1 doubles back on itself", answer["error"])
        self.assertIn("Turn off Reverse for this section", answer["error"])

    def test_4_a_failed_check_reports_measured_value_and_limit(self):
        project = dict(steps=[dict(type="drive", distance=30, timeout_ms=300)], stop_on_failure=True)
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        row = strict_json((output / "result.json").read_text())["steps"][0]
        self.assertFalse(row["passed"])
        self.assertIn("Used the whole 0.30 s time limit without finishing.", row["reason"])
        self.assertRegex(row["reason"], r"Ended \d+\.\d\d in from the target\. The limit is 2\.5 in\.")

    def test_5_a_chained_drive_hands_off_without_stopping(self):
        project = dict(steps=[dict(type="drive", distance=24, chain=True), dict(type="turn", heading=90)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        drive, turn = strict_json((output / "result.json").read_text())["steps"]
        self.assertNotIn("stopped", [check["id"] for check in drive["checks"]])
        self.assertIn("stopped", [check["id"] for check in turn["checks"]])
        self.assertRegex(drive["reason"], "All checks passed")
        self.assertTrue(any(check["message"].startswith("Handed off") for check in drive["checks"]))
        self.assertEqual(drive["end_time_s"], turn["start_time_s"])
        handoff = next(line for line in drive["info"] if line.startswith("Kept moving into step 2 at"))
        self.assertGreater(float(handoff.split(" at ")[1].split(" in/s")[0]), 5)

    def test_6_chained_paths_carry_their_speed_across_the_join(self):
        def routine(chain):
            return dict(steps=[dict(path_step([(0, -12)]), chain=chain), path_step([(0, 24)]),
                               dict(type="turn", heading=90)])
        answer = json.loads(self.engine("plan", routine(True)).stdout)
        self.assertNotIn("error", answer, answer)
        first, second = answer["plans"][:2]
        self.assertGreater(first["end_speed_ips"], 10)
        self.assertAlmostEqual(second["start_speed_ips"], first["end_speed_ips"], places=6)
        self.assertLessEqual(first["end_speed_ips"], first["requested_end_ips"] + 1e-9)
        totals = {}
        for chain in (True, False):
            done, output = self.run_routine(routine(chain))
            self.assertEqual(done.returncode, 0, done.stderr[-3000:])
            result = strict_json((output / "result.json").read_text())
            totals[chain] = result["steps"][1]["return_time_s"]
            if chain:
                self.assertIn("no final turn or settle", " ".join(result["steps"][0]["info"]))
        self.assertLess(totals[True], totals[False] - 0.5)


if __name__ == "__main__":
    unittest.main()
