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
from project import (DEFAULT_PROJECT, ProjectError, carries_speed, config, default_project,  # noqa: E402
                     step_time_limit, strict_json, validate_project)

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

    def test_new_routines_use_override_and_old_files_the_open_floor(self):
        self.assertEqual(validate_project(default_project())["field"], "override")
        self.assertEqual(validate_project({})["field"], "open")
        with self.assertRaisesRegex(ValueError, "Field must be one of"):
            validate_project(dict(field="moon"))

    def test_old_files_get_no_preload_and_no_actions(self):
        project = validate_project({})
        self.assertEqual(project["preload"], "none")
        self.assertEqual(project["steps"][0]["actions"], [])
        self.assertEqual(validate_project(default_project())["preload"], "yellow")

    def test_the_lift_and_claw_need_the_override_field(self):
        drive_with_action = dict(type="drive", actions=[dict(when="start", mechanism="claw", preset="open")])
        cases = [
            (dict(field="open", steps=[dict(type="arm")]), "Step 1 .*only the Override robot"),
            (dict(field="push_back", steps=[drive_with_action]), "only the Override robot"),
            (dict(field="open", preload="yellow"), "A preload needs the Override field"),
            (dict(field="override", preload="green"), "Preload must be one of"),
            (dict(field="override", steps=[dict(type="arm", arm_preset="sky")]), "Lift position must be one of"),
            (dict(field="override", steps=[dict(type="arm", actions=drive_with_action["actions"])]),
             "actions go on drive steps"),
            (dict(field="override", steps=[dict(type="claw", chain=True), dict(type="drive")]),
             "drive is not moving during a claw step"),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="time", value=2.5, mechanism="arm",
                                                                             preset="top")])]),
             "time must be a whole number"),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="soon", mechanism="arm",
                                                                             preset="top")])]),
             "when must be one of"),
        ]
        for raw, message in cases:
            with self.subTest(message), self.assertRaisesRegex(ValueError, message):
                validate_project(raw)

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

    def test_export_of_the_lift_claw_and_actions_compiles(self):
        project = validate_project(dict(field="override", preload="red", steps=[
            dict(type="drive", distance=12, actions=[dict(when="start", mechanism="arm", preset="above_alliance_goal"),
                                                     dict(when="distance", value=8, mechanism="claw", preset="open")]),
            dict(path_step([(0, -24)]), actions=[dict(when="time", value=250, mechanism="arm", preset="down")]),
            dict(type="arm", arm_preset="into_alliance_goal"),
            dict(type="claw", claw_preset="closed", timeout_ms=800),
            dict(type="wall_reset", x=0, y=-63, heading=180,
                 actions=[dict(when="start", mechanism="arm", preset="top")])]))
        code = export(project)
        self.assertIn(exported_helpers("flex_helpers.hpp"), code)
        for text in ("static pros::Task flex_task", "Preload: Red Pin, red half up",
                     "flex_rig.moveArm(flex::ArmPreset::IntoAllianceGoal, 1500)",
                     "flex_rig.moveClaw(flex::ClawPreset::Closed, 800)",
                     "{flex::Action::When::AfterDistance, 8, false, static_cast<int>(flex::ClawPreset::Open)}",
                     "{flex::Action::When::AfterTime, 250, true, static_cast<int>(flex::ArmPreset::Down)}",
                     "const bool result = wallReset("):
            self.assertIn(text, code)
        self.assertEqual(code.count("flex_rig.finishActions();"), 3)
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


    def test_7_override_reports_where_and_how_hard_the_robot_hit(self):
        project = dict(field="override", start=dict(x=24, y=-50, heading=180),
                       steps=[dict(type="drive", distance=20, timeout_ms=1500)])
        done, output = self.run_routine(project)
        result = strict_json((output / "result.json").read_text())
        step = result["steps"][0]
        hit = next(line for line in step["info"] if line.startswith("Hit the wall at"))
        self.assertRegex(hit, r"peak \d+ N")
        self.assertEqual(step["contacts"][0]["element"], "the wall")
        self.assertGreater(step["contacts"][0]["peak_force_n"], 20)
        self.assertTrue(any(frame.get("contact") == "the wall" for frame in result["trace"]))

    def test_8_override_preload_goes_into_the_red_goal_and_scores(self):
        project = dict(field="override", preload="yellow", start=dict(x=0, y=-62.7, heading=0), steps=[
            dict(type="drive", distance=15.6, actions=[dict(when="start", mechanism="arm",
                                                             preset="above_alliance_goal")]),
            dict(type="turn", heading=90),
            dict(type="drive", distance=13.47),
            dict(type="arm", arm_preset="into_alliance_goal"),
            dict(type="claw", claw_preset="open"),
            dict(type="drive", distance=-10, actions=[dict(when="distance", value=4, mechanism="arm",
                                                            preset="down")])])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["passed_steps"], 6)
        drop = next(line for line in result["steps"][4]["info"] if line.startswith("Dropped"))
        self.assertRegex(drop, r"red/yellow Pin at \(2[34], -4[67]\).*: the claw opened\.")
        self.assertIn("Lift reached Into an Alliance Goal (290°)", result["steps"][3]["checks"][0]["message"])
        self.assertEqual(result["score"]["red"], dict(alliance_pins=1, yellow_pins=0, total=5))
        self.assertEqual(len(result["pieces"]), 74)
        last = result["trace"][-1]
        self.assertEqual((last["score_red"], last["score_blue"]), (5, 0))
        self.assertLess(last["arm_deg"], 30)
        self.assertIsNone(last["holding"])
        self.assertEqual(result["trace"][0]["holding"], "a red/yellow Pin")

    def test_9_override_autonomous_example_earns_the_win_point(self):
        done, output = self.run_routine(example("override_autonomous"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["failed_steps"], 0)
        autonomous = result["autonomous"]
        self.assertLessEqual(autonomous["t"], 15.0)
        self.assertEqual((autonomous["red"], autonomous["blue"]), (55, 0))
        self.assertEqual(autonomous["bonus"], dict(red=12, blue=0))
        self.assertEqual(autonomous["toggles"], dict(left="red", bottom="red", top="yellow", right="yellow"))
        self.assertTrue(autonomous["win_point"]["earned"])
        self.assertFalse(any(contact["element"].endswith("Goal") for row in result["steps"]
                             for contact in row["contacts"]))
        turned = [line for row in result["steps"] for line in row["info"] if line.startswith("Turned the")]
        self.assertEqual([line.split(":")[0] for line in turned], ["Turned the left Toggle", "Turned the bottom Toggle"])

    def test_10_cup_to_goal_example_grips_and_drops_the_pin(self):
        done, output = self.run_routine(example("override_cup_to_goal"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["failed_steps"], 0)
        grip = result["steps"][6]
        self.assertEqual(grip["move_result"], "Stopped closing on a game object")
        self.assertTrue(any(line.startswith("Gripped a yellow Pin at (2") for line in grip["info"]))
        self.assertTrue(any(line.startswith("Dropped a yellow Pin at (23, -47)") for line in result["steps"][11]["info"]))
        self.assertEqual((result["autonomous"]["red"], result["autonomous"]["blue"]), (40, 0))


if __name__ == "__main__":
    unittest.main()
