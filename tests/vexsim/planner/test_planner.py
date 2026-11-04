"""Routine checks, the C++ export, and the engine end to end in vexsim.

The engine tests compile mclib once (1 to 2 minutes). Set
MCLIB_PLANNER_CACHE to a folder to keep that build between test runs, and
VEXSIM_PATH to the vexsim checkout (default: ../vexsim next to mclib).
"""
import copy
import json
import os
from pathlib import Path
import re
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
        self.assertEqual(validate_project(default_project())["preload"], "beside")

    def test_the_mechanisms_need_the_override_field(self):
        drive_with_action = dict(type="drive", actions=[dict(when="start", mechanism="claw", preset="open")])

        def drive_with(**action):
            return dict(field="override", steps=[dict(type="drive", actions=[action])])

        cases = [
            (dict(field="open", steps=[dict(type="lift")]), "Step 1 .*only the Override robot"),
            (dict(field="open", steps=[dict(type="match_load")]), "Step 1 .*Match Loads are part of the Override"),
            (dict(field="push_back", steps=[drive_with_action]), "only the Override robot"),
            (dict(field="open", preload="behind"), "A preload needs the Override field"),
            (dict(field="override", preload="green"), "Preload must be one of"),
            (dict(field="override", steps=[dict(type="lift", lift_preset="sky")]), "Lift position must be one of"),
            (dict(field="override", steps=[dict(type="lift", actions=drive_with_action["actions"])]),
             "actions go on drive steps"),
            (dict(field="override", steps=[dict(type="match_load", actions=drive_with_action["actions"])]),
             "actions go on drive steps"),
            (dict(field="override", steps=[dict(type="claw", chain=True), dict(type="drive")]),
             "drive is not moving during a claw step"),
            (dict(field="override", steps=[dict(type="match_load", chain=True), dict(type="drive")]),
             "drive is not moving during a Match Load"),
            (dict(field="override", steps=[dict(type="claw", timeout_ms=1000)]),
             r"Step 1 \(Open or close the claw\): this step always takes 10 ms, so it has no time limit"),
            (dict(field="override", steps=[dict(type="match_load", loader="blue_left")]), "Loader must be one of"),
            (drive_with(when="time", value=2.5, mechanism="lift", preset="top"), "time must be a whole number"),
            (drive_with(when="soon", mechanism="lift", preset="top"), "when must be one of: start, distance, time, end"),
            (drive_with(when="start", mechanism="claw", preset="half"), "the claw preset must be one of: open, closed"),
            (drive_with(when="start", mechanism="wrist", preset="top"), "mechanism must be one of: lift, claw"),
        ]
        for raw, message in cases:
            with self.subTest(message), self.assertRaisesRegex(ValueError, message):
                validate_project(raw)

    def test_a_file_made_for_the_old_arm_says_what_replaced_it(self):
        replaced = (r"the arm was the Flex robot's inclined lift, which the planner no longer has\. Use the lift "
                    r"instead \(\"lift\", with lift_preset one of: down, over_neutral_goal, on_neutral_goal")
        for raw in (dict(field="override", steps=[dict(type="arm", arm_preset="down")]),
                    dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="arm",
                                                                                    preset="down")])])):
            with self.subTest(raw), self.assertRaisesRegex(ProjectError, replaced):
                validate_project(raw)

    def test_a_file_made_for_the_flange_claw_says_what_replaced_it(self):
        bars = r"no Toggle arm to move any more\. It turns a Toggle with the two fixed arms on the back of its lift tower"
        rim = r"the lift preset \"pin_in_cup\" held a Pin by its flange, and the claw now holds a Cup by its top rim"
        cases = [
            (dict(field="override", steps=[dict(type="toggle_arm", toggle_arm_preset="stowed")]), bars),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="toggle_arm",
                                                                             preset="stowed")])]), bars),
            (dict(field="override", steps=[dict(type="lift", lift_preset="pin_in_cup")]), rim),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="lift",
                                                                             preset="over_alliance_goal")])]),
             "the lift preset \"over_alliance_goal\" held a Pin"),
            (dict(field="override", preload="yellow"),
             r"The preload \"yellow\" was a Pin held in the claw by its flange.*Set preload to \"behind\""),
        ]
        for raw, message in cases:
            with self.subTest(message), self.assertRaisesRegex(ValueError, message):
                validate_project(raw)

    def test_an_end_action_has_no_value(self):
        step = validate_project(dict(field="override", steps=[dict(type="drive", actions=[
            dict(when="end", mechanism="claw", preset="open")])]))["steps"][0]
        self.assertEqual(step["actions"], [dict(when="end", value=0, mechanism="claw", preset="open")])

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

    def test_a_wait_takes_exactly_its_time(self):
        step = validate_project(dict(steps=[dict(type="wait", wait_ms=15000)]))["steps"][0]
        self.assertEqual((step["wait_ms"], step["timeout_ms"]), (15000, 15000))
        cases = [
            (dict(type="wait", wait_ms=1500.5), "Step 1 (Wait): Wait must be a whole number"),
            (dict(type="wait", wait_ms=5), "Step 1 (Wait): Wait must be between 10 and 120000 ms, not 5"),
            (dict(type="wait", wait_ms=2000, timeout_ms=4000),
             "Step 1 (Wait): a wait always takes its 2000 ms, so it has no time limit of its own. Remove timeout_ms"),
        ]
        for step, message in cases:
            with self.subTest(message), self.assertRaises(ProjectError) as caught:
                validate_project(dict(steps=[step]))
            self.assertEqual(str(caught.exception), message)

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

    def test_export_of_the_lift_claw_match_load_and_actions_compiles(self):
        project = validate_project(dict(field="override", preload="behind", steps=[
            dict(type="drive", distance=12, actions=[dict(when="start", mechanism="lift", preset="over_neutral_goal"),
                                                     dict(when="distance", value=8, mechanism="claw", preset="open")]),
            dict(path_step([(0, -24)]), actions=[dict(when="time", value=250, mechanism="lift", preset="down"),
                                                 dict(when="end", mechanism="claw", preset="closed")]),
            dict(type="lift", lift_preset="on_stack_on_neutral_goal"),
            dict(type="claw", claw_preset="closed"),
            dict(type="wait", wait_ms=15000),
            dict(type="match_load", loader="red_right", match_load_pin="yellow"),
            dict(type="wall_reset", x=0, y=-63, heading=180,
                 actions=[dict(when="start", mechanism="lift", preset="top")])]))
        code = export(project)
        self.assertIn(exported_helpers("cascade_helpers.hpp"), code)
        for text in ("static pros::Task cascade_task", "Preload: Red/yellow Pin lying on the floor across the robot's back",
                     "mclib::device::MotorGroup lift_motors({7, -8}, mclib::device::Gearset::Green);",
                     "mclib::device::Pneumatic claw_piston('A');",
                     "[](bool closed) { claw_piston.set_value(closed); }",
                     "cascade_rig.moveLift(cascade::LiftPreset::OnStackOnNeutralGoal, 1500)",
                     "cascade_rig.setClaw(cascade::ClawPreset::Closed);",
                     "// The Drive Team stands a Cup under the red right Loader with a yellow/yellow Pin in it. The robot "
                     "does nothing.",
                     "{cascade::Action::When::AfterDistance, 8, cascade::Mechanism::Claw, "
                     "static_cast<int>(cascade::ClawPreset::Open)}",
                     "{cascade::Action::When::AfterTime, 250, cascade::Mechanism::Lift, "
                     "static_cast<int>(cascade::LiftPreset::Down)}",
                     "{cascade::Action::When::End, 0, cascade::Mechanism::Claw, "
                     "static_cast<int>(cascade::ClawPreset::Closed)}",
                     "  pros::delay(15000);\n",
                     "const bool result = wallReset("):
            self.assertIn(text, code)
        for gone in ("claw_motor", "toggle_arm", "ToggleArm"):
            self.assertNotIn(gone, code)
        self.assertEqual(code.count("cascade_rig.finishActions();"), 3)
        self.compile(code)

    def test_the_exported_lift_presets_are_the_planner_heights(self):
        from mechanisms import LIFT_HEIGHTS_IN
        helpers = exported_helpers("cascade_helpers.hpp")
        table = dict(re.findall(r"\{LiftPreset::(\w+), ([\w.]+)\}", helpers))
        lowest = re.search(r"kLowestClawIn = ([\d.]+);", helpers).group(1)
        exported = {name: float(lowest if value == "kLowestClawIn" else value) for name, value in table.items()}
        self.assertEqual(exported, {"".join(word.capitalize() for word in name.split("_")): height
                                    for name, height in LIFT_HEIGHTS_IN.items()})

    def test_override_examples_export_and_compile(self):
        for name in ("override_autonomous", "override_stack_to_goal", "override_toggle_arms", "override_match_load"):
            with self.subTest(name):
                code = export(validate_project(example(name)))
                self.assertIn("static pros::Task cascade_task", code)
                self.compile(code)

    def test_export_without_a_plan_marks_the_placeholders(self):
        code = export(validate_project(example("s_curve")))
        self.assertIn("placeholder", code.lower())
        self.compile(code)


@unittest.skipUnless((VEXSIM / "vexsim/sim.py").is_file(), f"no vexsim checkout at {VEXSIM}")
class FieldDrawingTests(unittest.TestCase):
    def test_override_3d_field_has_the_manual_sizes(self):
        from fields import drawings
        drawing = drawings(VEXSIM)["override"]
        solid = drawing["solid"]
        self.assertEqual((solid["wall_height_in"], solid["wall_thickness_in"]), (11.54, 2.0))
        self.assertEqual({goal["name"]: goal["height_in"] for goal in solid["goals"]}["center"], 8.77)
        self.assertEqual(sorted({goal["height_in"] for goal in solid["goals"]}), [3.25, 5.77, 8.77])
        left = next(toggle for toggle in solid["toggles"] if toggle["quadrant"] == "left")
        self.assertEqual((left["axle"], left["inward"], left["faces"]), ([0.0, -70.079], [-0.0, 1.0], ["yellow", "red", "blue"]))
        self.assertEqual((left["axle_height_in"], left["length_in"]), (13.2, 25.99))
        self.assertEqual(solid["pin_half_profile"][-1], [3.25, 0.7])
        kinds = [piece["kind"] for piece in drawing["pieces"]]
        self.assertEqual((kinds.count("pin"), kinds.count("cup")), (37, 36))

    def test_the_3d_robot_is_vexsims_cascade_robot(self):
        from fields import drawings
        sys.path.insert(0, str(VEXSIM))
        robot = drawings(VEXSIM)["override"]["robot"]
        self.assertEqual((robot["claw"]["lowest_in"], robot["claw"]["highest_in"]), (6.2, 33.2))
        self.assertEqual((robot["claw"]["open_gap_in"], robot["claw"]["closed_gap_in"]), (4.4, 3.09))
        names = [part["name"] for part in robot["parts"]]
        self.assertEqual({name: names.count(name) for name in set(names)},
                         {"drivetrain": 11, "lift": 6, "carriage": 1, "claw": 3, "toggle arm": 2})
        arms = [part for part in robot["parts"] if part["name"] == "toggle arm"]
        self.assertEqual({(part["back"], part["bottom"], part["top"]) for part in arms}, {(-8.0, 13.3002, 14.35)})
        frame = robot["frame"]
        drivetrain = [part for part in robot["parts"] if part["name"] == "drivetrain"]
        for axis, low, high in ((0, "back", "front"), (1, "right", "left")):
            self.assertEqual(min(point[axis] for point in frame["outline"]), min(part[low] for part in drivetrain))
            self.assertEqual(max(point[axis] for point in frame["outline"]), max(part[high] for part in drivetrain))
        self.assertLess(robot["preloads"]["behind"]["forward_in"], -7.5)
        self.assertLess(robot["preloads"]["beside"]["left_in"], -6.25)

    def test_each_3d_piece_moves_onto_vexsims_piece_with_the_lift_down_and_up(self):
        from fields import drawings
        sys.path.insert(0, str(VEXSIM))
        from vexsim.cascade import CascadeRobot
        from vexsim.presets import four_motor_450
        from vexsim.units import INCH
        robot = drawings(VEXSIM)["override"]["robot"]
        for lift_up, claw_closed in ((False, False), (True, False), (True, True)):
            simulated = CascadeRobot()
            if lift_up:
                simulated.mechanism.lift.motors[0].position = simulated.mechanism.lift.full
            simulated.mechanism.claw.command(claw_closed)
            simulated.mechanism.claw.stroke = 1.0 if claw_closed else 0.0
            rise = simulated.grip_height() / INCH - robot["claw"]["lowest_in"]
            closing = (robot["claw"]["open_gap_in"] - simulated.claw_gap() / INCH) / 2
            parts = simulated.parts(four_motor_450())
            self.assertEqual(len(parts), len(robot["parts"]))
            for drawn, part in zip(robot["parts"], parts):
                with self.subTest(lift_up=lift_up, claw_closed=claw_closed, piece=drawn["key"]):
                    up, left = drawn["rise_share"] * rise, -drawn["jaw_side"] * closing
                    posed = dict(back=drawn["back"], front=drawn["front"], right=drawn["right"] + left,
                                 left=drawn["left"] + left, bottom=drawn["bottom"] + up, top=drawn["top"] + up)
                    for side, value in posed.items():
                        self.assertAlmostEqual(value, getattr(part, side) / INCH, delta=1e-3)
                    for (forward, height), (x, z) in zip(drawn["profile"], part.profile):
                        self.assertAlmostEqual(forward, x / INCH, delta=1e-3)
                        self.assertAlmostEqual(height + up, z / INCH, delta=1e-3)


@unittest.skipUnless((VEXSIM / "vexsim/sim.py").is_file(), f"no vexsim checkout at {VEXSIM}")
class MatchesVexsimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(VEXSIM))

    def test_lift_presets_are_the_claw_heights_vexsim_uses(self):
        from mechanisms import LIFT_HEIGHTS_IN
        from vexsim.cascade import LIFT_HEIGHTS
        vexsim_names = dict(down="down", over_neutral_goal="over a neutral goal", on_neutral_goal="on a neutral goal",
                            over_stack_on_neutral_goal="over a stack on a neutral goal",
                            on_stack_on_neutral_goal="on a stack on a neutral goal",
                            over_center_goal="over the center goal", on_center_goal="on the center goal", top="top")
        self.assertEqual(set(vexsim_names.values()), set(LIFT_HEIGHTS))
        for name, height in LIFT_HEIGHTS_IN.items():
            with self.subTest(name):
                self.assertAlmostEqual(height, LIFT_HEIGHTS[vexsim_names[name]], places=9)

    def test_exported_motor_degrees_turn_into_vexsims_heights(self):
        from vexsim.cascade import GEOMETRY
        helpers = exported_helpers("cascade_helpers.hpp")
        lowest = float(re.search(r"kLowestClawIn = ([\d.]+);", helpers).group(1))
        teeth, pitch = re.search(r"kChainPerTurnIn = (\d+) \* ([\d.]+);", helpers).groups()
        stages = int(re.search(r"kLiftStages = (\d+);", helpers).group(1))
        self.assertEqual((int(teeth), float(pitch), stages),
                         (GEOMETRY.sprocket_teeth, GEOMETRY.chain_pitch, GEOMETRY.stages))
        self.assertAlmostEqual(lowest, GEOMETRY.grip_lowest, places=9)

    def test_a_robot_that_falls_over_fails_the_step_and_one_that_rocks_says_how_far(self):
        from engine import TipWatch
        from vexsim import Simulator, presets
        from vexsim.sim import SimConfig
        from vexsim.units import INCH

        def brake_from_full_speed(cg_height_in):
            chassis = presets.four_motor_450()
            chassis.cg_height = cg_height_in * INCH
            sim = Simulator(chassis, config=SimConfig(log_every=0))
            tips = TipWatch(sim)
            for volts, ms in ((12.0, 1500), (-12.0, 600)):
                for motor in sim.motors:
                    motor.set_voltage(volts)
                if volts < 0:
                    tips.begin_step()
                for t in range(ms):
                    sim.step(0.001)
                    if t % 10 == 0:
                        tips.watch()
            return tips

        fell = brake_from_full_speed(15.0)
        self.assertTrue(fell.fallen)
        [upright] = fell.fall_checks()
        self.assertFalse(upright["passed"])
        self.assertRegex(upright["message"], r"^The robot fell over at 2\.\d\d s\. It rocked onto its front wheels "
                                             r"past 19°, where its centre of mass, 15\.0 in up, was over them\.$")
        self.assertEqual(fell.rock_lines(), [])
        rocked = brake_from_full_speed(13.0)
        self.assertEqual(rocked.fall_checks(), [])
        self.assertRegex(rocked.rock_lines()[0], r"^The robot rocked 7\.\d° onto its front wheels\. "
                                                 r"It falls over at 22°\.$")
        level = brake_from_full_speed(5.0)
        self.assertEqual((level.fall_checks(), level.rock_lines()), ([], []))

    def test_drive_watts_are_the_motors_vexsim_puts_on_each_base(self):
        from project import PRESETS
        from vexsim import presets
        for name, preset in PRESETS.items():
            with self.subTest(name):
                chassis = presets.get(name)
                watts = sum(motor.spec.peak_power_rating for unit in chassis.wheel_units for motor in unit.motors)
                self.assertEqual(preset["drive_watts"], watts)


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
        at_rest = [re.search(r"stood still, at ([\d.]+) s in step (\d+), odometry was ([\d.]+) in", line)
                   for line in drive["info"]]
        at_rest = [found for found in at_rest if found]
        self.assertEqual(len(at_rest), 1, drive["info"])
        still_s, still_step, still_gap = at_rest[0].groups()
        self.assertGreater(float(still_s), drive["return_time_s"])
        self.assertLessEqual(float(still_s), turn["end_time_s"])
        self.assertEqual(still_step, "2")
        self.assertLess(float(still_gap), drive["odometry_error_in"])

    def test_5b_the_gap_at_rest_is_left_out_when_the_robot_only_stops_two_steps_later(self):
        project = dict(steps=[dict(type="drive", distance=24, chain=True), dict(type="drive", distance=12, chain=True),
                              dict(type="turn", heading=90)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        first, second, turn = strict_json((output / "result.json").read_text())["steps"]
        self.assertFalse(any("stood still" in line for line in first["info"]), first["info"])
        self.assertTrue(any("stood still, at" in line and "in step 3" in line for line in second["info"]),
                        second["info"])

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
        project = dict(field="override", robot=dict(preset="four_motor_450"), start=dict(x=36, y=-50, heading=180),
                       steps=[dict(type="drive", distance=20, timeout_ms=1500)])
        done, output = self.run_routine(project)
        result = strict_json((output / "result.json").read_text())
        step = result["steps"][0]
        hit = next(line for line in step["info"] if line.startswith("Hit the wall at"))
        self.assertRegex(hit, r"peak \d+ N")
        self.assertEqual(step["contacts"][0]["element"], "the wall")
        self.assertGreater(step["contacts"][0]["peak_force_n"], 20)
        self.assertTrue(any(frame.get("contact") == "the wall" for frame in result["trace"]))

    def test_8_the_claw_carries_a_stack_to_a_neutral_goal(self):
        done, output = self.run_routine(example("override_stack_to_goal"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["failed_steps"], 0)
        info = [line for row in result["steps"] for line in row["info"]]
        grips = [line for line in info if line.startswith("Gripped")]
        drops = [line for line in info if line.startswith("Dropped")]
        self.assertEqual(len(grips), 1, grips)
        self.assertEqual(len(drops), 1, drops)
        self.assertRegex(grips[0], r"^Gripped a Cup with a yellow Pin at \(2[34], -2[34]\)")
        self.assertRegex(drops[0], r"^Dropped a Cup with a yellow Pin at \(-2[34], -4[67]\).*: the claw opened\.")
        self.assertEqual(len(result["pieces"]), 74)
        first, last = result["trace"][0], result["trace"][-1]
        self.assertIsNone(first["holding"])
        self.assertIsNone(last["holding"])
        self.assertLess(last["lift_height_in"], 6.5)
        self.assertEqual(result["score"]["red"]["total"], 0)
        rules = result["autonomous"]["rules"]
        self.assertEqual(rules["most_held"], dict(pins=1, cups=1, passed=True))
        self.assertLessEqual(max(rules["start_in"][key] for key in ("length", "width", "height")), 18.0)
        self.assertEqual(rules["motors_w"], dict(drive=44, total=66.0, drive_limit=55.0, total_limit=88.0, passed=True))
        checks = [check["message"] for check in result["steps"][0]["checks"]]
        self.assertIn("The drive motors are rated 44 W and all the motors 66 W. The limits are 55 W for the drive "
                      "(R11) and 88 W in all (R10).", checks)

    def test_9_override_autonomous_example_turns_both_toggles_and_scores_a_stack(self):
        done, output = self.run_routine(example("override_autonomous"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["failed_steps"], 0)
        self.assertLessEqual(result["summary"]["simulated_s"], 14.0)
        autonomous = result["autonomous"]
        self.assertEqual((autonomous["red"], autonomous["blue"]), (50, 0))
        self.assertEqual(autonomous["bonus"], dict(red=12, blue=0))
        self.assertEqual(autonomous["toggles"], dict(left="red", bottom="red", top="yellow", right="yellow"))
        self.assertFalse(autonomous["win_point"]["earned"])
        self.assertTrue(all(rule["passed"] for rule in autonomous["rules"].values()), autonomous["rules"])
        goal_hits = [contact for row in result["steps"] for contact in row["contacts"]
                     if contact["element"].endswith("Goal")]
        self.assertEqual({contact["element"] for contact in goal_hits}, {"the neutral bottom Goal"})
        self.assertLess(max(contact["peak_force_n"] for contact in goal_hits), 150)
        info = [line for row in result["steps"] for line in row["info"]]
        turned = [line.split(" now")[0] for line in info if line.startswith("Turned the")]
        self.assertEqual(turned, ["Turned the left Toggle: blue", "Turned the left Toggle: red",
                                  "Turned the bottom Toggle: blue", "Turned the bottom Toggle: red"])
        self.assertEqual(len([line for line in info if line.startswith("Gripped")]), 1)
        self.assertEqual(len([line for line in info if line.startswith("Dropped")]), 1)

    def test_10_a_match_load_in_the_autonomous_period_breaks_sg11a(self):
        project = dict(field="override", preload="none", robot=dict(preset="four_motor_450"),
                       start=dict(x=0, y=-61.7, heading=0),
                       steps=[dict(type="match_load", loader="red_left", match_load_pin="red_yellow_up")])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        row = strict_json((output / "result.json").read_text())["steps"][0]
        self.assertFalse(row["passed"])
        self.assertIn("Match Loads may only be loaded in the Driver Controlled Period (SG11a).", row["reason"])
        self.assertIn("The Drive Team stood a Cup under the red left Loader with a red/yellow Pin in it, yellow half up.",
                      [check["message"] for check in row["checks"]])

    def test_10b_a_wait_past_15_s_lets_a_match_load_score_in_driver_control(self):
        done, output = self.run_routine(example("override_match_load"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertEqual(result["summary"]["failed_steps"], 0)
        self.assertGreater(result["summary"]["simulated_s"], 20.0)
        wait, match_load = result["steps"][:2]
        self.assertIn("Waited 15.00 s, to 15.00 s into the match.", [check["message"] for check in wait["checks"]])
        self.assertAlmostEqual(match_load["start_time_s"], 15.0, places=3)
        self.assertIn("Loaded 15.00 s into the match, after the 15 s autonomous period (SG11a).",
                      [check["message"] for check in match_load["checks"]])
        info = [line for row in result["steps"] for line in row["info"]]
        grips = [line for line in info if line.startswith("Gripped")]
        drops = [line for line in info if line.startswith("Dropped")]
        self.assertEqual(len(grips), 1, grips)
        self.assertEqual(len(drops), 1, drops)
        self.assertRegex(grips[0], r"^Gripped a Cup with a red/yellow Pin at \(59, -6[78]\)")
        self.assertRegex(drops[0], r"^Dropped a Cup with a red/yellow Pin at \(4[67], 2[34]\).*: the claw opened\.")
        self.assertEqual((result["autonomous"]["red"], result["autonomous"]["blue"]), (0, 0))
        self.assertEqual(result["score"]["red"], dict(alliance_pins=1, yellow_pins=0, total=5))
        self.assertEqual(result["score"]["blue"]["total"], 0)

    def test_11_a_step_stopped_by_a_goal_names_the_goal_not_the_controller(self):
        project = dict(field="override", preload="none", robot=dict(preset="four_motor_450"),
                       start=dict(x=24, y=-62.7, heading=0), steps=[dict(type="drive", distance=30, timeout_ms=2500)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        row = strict_json((output / "result.json").read_text())["steps"][0]
        self.assertFalse(row["passed"])
        self.assertTrue(any(line.startswith("Hit the red left Goal") for line in row["info"]))
        self.assertEqual(len(row["notes"]), 1, row["notes"])
        self.assertRegex(row["notes"][0], r"^The robot was stopped by the red left Goal: it moved \d\.\d in of the "
                                          r"30\.0 in asked for\. The controller was not at fault\.")

    def test_12_the_toggle_arms_turn_a_toggle_top_toward_the_wall_and_slide_off_it(self):
        def hit(toggle, start):
            project = dict(field="override", preload="none", robot=dict(preset="four_motor_450"), start=start,
                           steps=[dict(type="drive", distance=-12, chain=True),
                                  dict(type="drive", distance=8)])
            done, output = self.run_routine(project)
            self.assertEqual(done.returncode, 0, done.stderr[-3000:])
            result = strict_json((output / "result.json").read_text())
            angles = [frame["toggles"][toggle]["angle_deg"] for frame in result["trace"]]
            turned = [line for row in result["steps"] for line in row["info"] if line.startswith("Turned the")]
            return angles, turned

        for toggle, start, color in (("left", dict(x=0, y=-50, heading=0), "blue"),
                                     ("bottom", dict(x=50, y=0, heading=270), "blue"),
                                     ("right", dict(x=0, y=50, heading=180), "red")):
            with self.subTest(toggle):
                angles, turned = hit(toggle, start)
                self.assertGreater(max(angles), 60)
                self.assertGreater(min(angles), -0.5)
                self.assertEqual(len(turned), 1, turned)
                self.assertTrue(turned[0].startswith(f"Turned the {toggle} Toggle: {color} now faces into the field "
                                                     "(yellow before)."), turned)

    def test_13_a_drive_over_55_w_breaks_r11_on_the_first_step(self):
        project = dict(field="override", preload="none", robot=dict(preset="six_motor_450"),
                       start=dict(x=0, y=-61.7, heading=0), steps=[dict(type="drive", distance=5),
                                                                  dict(type="drive", distance=-5)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        first = result["steps"][0]
        self.assertFalse(first["passed"])
        self.assertIn("The drive motors are rated 66 W and all the motors 88 W. The limits are 55 W for the drive "
                      "(R11) and 88 W in all (R10).", first["reason"])
        self.assertEqual(result["autonomous"]["rules"]["motors_w"]["passed"], False)


if __name__ == "__main__":
    unittest.main()
