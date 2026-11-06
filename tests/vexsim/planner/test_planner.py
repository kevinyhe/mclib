"""Routine checks, the C++ export, and the engine end to end in vexsim.

The engine tests compile mclib once (1 to 2 minutes). Set
MCLIB_PLANNER_CACHE to a folder to keep that build between test runs, and
VEXSIM_PATH to the vexsim checkout (default: ../vexsim next to mclib).
"""
import copy
import json
import math
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


def turned(side_view, hinge, degrees):
    """Side-view points (forward, up) turned `degrees` about `hinge`, as
    vexsim turns a roller piece: positive takes what is behind it up."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    pivot_forward, pivot_up = hinge["forward_in"], hinge["up_in"]
    return [(pivot_forward + (forward - pivot_forward) * c + (up - pivot_up) * s,
             pivot_up - (forward - pivot_forward) * s + (up - pivot_up) * c) for forward, up in side_view]


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

    def test_the_mechanisms_need_the_override_field(self):
        drive_with_action = dict(type="drive", actions=[dict(when="start", mechanism="claw", preset="open")])

        def drive_with(**action):
            return dict(field="override", steps=[dict(type="drive", actions=[action])])

        cases = [
            (dict(field="open", steps=[dict(type="lift")]), "Step 1 .*only the Override robot"),
            (dict(field="open", steps=[dict(type="match_load")]), "Step 1 .*Match Loads are part of the Override"),
            (dict(field="push_back", steps=[drive_with_action]), "only the Override robot"),
            (dict(field="open", preload="yellow"), "A preload needs the Override field"),
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
             r"Step 1 \(Open or close the claw\): this step always takes 170 ms, so it has no time limit"),
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
                    r"instead \(\"lift\", with lift_preset one of: down, off_the_floor, over_neutral_goal, on_neutral_goal")
        for raw in (dict(field="override", steps=[dict(type="arm", arm_preset="down")]),
                    dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="arm",
                                                                                    preset="down")])])):
            with self.subTest(raw), self.assertRaisesRegex(ProjectError, replaced):
                validate_project(raw)

    def test_a_file_made_for_the_flange_claw_says_what_replaced_it(self):
        bars = r"no Toggle arm to move any more\. It turns a Toggle with the flap roller on its back"
        rim = r"the lift preset \"pin_in_cup\" held a Pin by its flange, and the claw now holds a Cup by its top rim"
        cases = [
            (dict(field="override", steps=[dict(type="toggle_arm", toggle_arm_preset="stowed")]), bars),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="toggle_arm",
                                                                             preset="stowed")])]), bars),
            (dict(field="override", steps=[dict(type="lift", lift_preset="pin_in_cup")]), rim),
            (dict(field="override", steps=[dict(type="drive", actions=[dict(when="start", mechanism="lift",
                                                                             preset="over_alliance_goal")])]),
             "the lift preset \"over_alliance_goal\" held a Pin"),
            (dict(field="override", preload="behind"),
             r"The preload \"behind\" was a Pin lying on the floor.*Set preload to \"yellow\" or \"red\""),
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

    def test_min_voltage_is_automatic_unless_set_and_never_above_the_max(self):
        project = validate_project(dict(steps=[dict(type="drive", distance=12), dict(type="turn", heading=90, min_volts=2)]))
        self.assertEqual([step["min_volts"] for step in project["steps"]], [None, 2])
        with self.assertRaisesRegex(ValueError, r"Step 1 \(Drive straight\): Min voltage \(7 V\) must not be more than "
                                                r"Max voltage \(6 V\)"):
            validate_project(dict(steps=[dict(type="drive", distance=12, volts=6, min_volts=7)]))
        with self.assertRaisesRegex(ValueError, "fields the planner does not know: min_volts"):
            validate_project(dict(field="override", steps=[dict(type="lift", lift_preset="top", min_volts=2)]))

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

    def test_export_passes_a_set_min_voltage_and_leaves_automatic_to_mclib(self):
        project = validate_project(dict(steps=[
            dict(type="drive", distance=12), dict(type="drive", distance=6, min_volts=2.5),
            dict(type="point", x=10, y=10, min_volts=3), dict(type="turn_to_point", x=0, y=40, min_volts=4),
            dict(type="boomerang", x=-10, y=20, heading=0, min_volts=1),
            dict(type="swing", heading=45, chain=True, min_volts=2), dict(type="turn", heading=0)]))
        code = export(project)
        for text in ("driveTo(12_in, 4000_ms, true, 12_V);", "driveTo(6_in, 4000_ms, true, 12_V, 2.5_V);",
                     "moveToPoint(10_in, 10_in, 1, 4000_ms, true, 12_V, true, 3_V);",
                     "turnToPoint(0_in, 40_in, 1, 4000_ms, 4_V);",
                     "boomerang(-10_in, 20_in, 1, 0_deg, 0.5, 4000_ms, true, 12_V, true, 1_V);",
                     "swing(45_deg, 1, 4000_ms, false, 12_V, 2_V);", "turnToAngle(0_deg, 4000_ms, true, 12_V);"):
            self.assertIn(text, code)
        self.compile(code)

    def test_export_of_the_lift_claw_match_load_and_actions_compiles(self):
        project = validate_project(dict(field="override", preload="yellow", steps=[
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
        for text in ("static pros::Task cascade_task", "Preload: Red/yellow Pin held in the closed claw by its flange, yellow half up",
                     "[](double volts) { roller_motor.setVoltage(volts); },\n"
                     "                         cascade::ClawPreset::Closed);",
                     "mclib::device::MotorGroup lift_motors({7, -8, 9}, mclib::device::Gearset::Green);",
                     "mclib::device::Pneumatic claw_piston('A');",
                     "[](bool closed) { claw_piston.set_value(closed); }",
                     "mclib::device::MotorGroup roller_motor({10}, mclib::device::Gearset::Green);",
                     "[] { return roller_motor.getAveragePositionDeg(); }",
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
                     "    wallReset(0_in, -63_in, 180_deg, 6_V, 4000_ms, 2500 * mclib::units::milliampere);\n",
                     "Claw to Open: jaws 4.4 in apart after driving 8 in from the step start."):
            self.assertIn(text, code)
        for gone in ("claw_motor", "toggle_arm", "ToggleArm", "Reached) return;", "TimedOut) return;", "if (!wallReset("):
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
        for name in ("override_autonomous", "override_stack_to_goal", "override_roller", "override_match_load"):
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
        self.assertEqual({goal["name"]: goal["height_in"] for goal in drawing["goals"]}["center"], 8.77)
        self.assertEqual(sorted({goal["height_in"] for goal in drawing["goals"]}), [3.25, 5.77, 8.77])
        left = next(toggle for toggle in solid["toggles"] if toggle["quadrant"] == "left")
        self.assertEqual((left["axle"], left["inward"], left["faces"]), ([0.0, -71.451], [-0.0, 1.0], ["yellow", "red", "blue"]))
        self.assertEqual(left["axle_height_in"], 13.202)
        kinds = [piece["kind"] for piece in drawing["pieces"]]
        self.assertEqual((kinds.count("pin"), kinds.count("cup")), (37, 36))

    def test_the_3d_views_cad_goals_and_pieces_are_as_tall_as_vexsims(self):
        import numpy as np
        from fields import drawings
        drawing = drawings(VEXSIM)["override"]
        parts = drawing["solid"]["cad"]
        layout = json.loads((VEXSIM / "cad/override/view.json").read_text())
        blob = (VEXSIM / "cad/override/view.bin").read_bytes()

        def part_points(number):
            mesh = layout["meshes"][number]
            return np.frombuffer(blob, np.float32, mesh["vertices"] * 3, mesh["vertex_offset"]).reshape(-1, 3)

        def goal_at(x, y):
            return min(drawing["goals"], key=lambda goal: np.hypot(goal["center"][0] + y, goal["center"][1] - x))["name"]

        tops = {}
        for number in parts["goal_parts"]:
            for instance in layout["instances"][number]:
                points = part_points(number) @ np.array(layout["cad_axes"]).T @ np.array(instance["rotation"]).T
                name = goal_at(*instance["position_in"][:2])
                tops[name] = max(tops.get(name, 0.0), float(points[:, 2].max() + instance["position_in"][2]))
        for goal in drawing["goals"]:
            self.assertAlmostEqual(tops[goal["name"]], goal["height_in"], delta=0.01, msg=goal["name"])
        size = drawing["piece_size"]
        for kind, length in (("pin", size["pin_length_in"]), ("cup", size["cup_length_in"])):
            low, high = (part_points(parts[f"{kind}_half"]) @ np.array(half["rotation"]).T + half["offset_in"]
                         for half in parts["halves"][kind])
            self.assertAlmostEqual(float(low[:, 2].min()), -length / 2.0, delta=0.01, msg=kind)
            self.assertAlmostEqual(float(high[:, 2].max()), length / 2.0, delta=0.01, msg=kind)
            self.assertLess(float(high[:, 2].min()), 0.0, msg=kind)

    def test_the_3d_robot_is_vexsims_cascade_robot(self):
        from fields import drawings
        sys.path.insert(0, str(VEXSIM))
        robot = drawings(VEXSIM)["override"]["robot"]
        self.assertEqual((robot["claw"]["lowest_in"], robot["claw"]["highest_in"]), (6.62, 46.72))
        self.assertEqual((robot["claw"]["open_gap_in"], robot["claw"]["closed_gap_in"]), (4.4, 3.1))
        names = [part["name"] for part in robot["parts"]]
        self.assertEqual({name: names.count(name) for name in set(names)},
                         {"drivetrain": 11, "lift": 10, "carriage": 1, "claw": 13, "roller post": 2,
                          "roller axle": 1, "roller": 2, "roller flap": 4})
        frame = robot["frame"]
        frame_back = min(point[0] for point in frame["outline"])
        turning = [part for part in robot["parts"] if part["hinges"]]
        self.assertEqual({part["name"] for part in turning}, {"roller axle", "roller", "roller flap"})
        self.assertEqual({(hinge["name"], hinge["forward_in"] - frame_back, hinge["up_in"])
                          for part in turning for hinge in part["hinges"][:1]}, {("roller", 1.75, 13.5)})
        flaps = [part for part in turning if part["name"] == "roller flap"]
        self.assertEqual({hinge["forward_in"] - part["back"] for part in flaps for hinge in part["hinges"][:1]
                          if part["back"] < hinge["forward_in"]}, {2.5})
        drivetrain = [part for part in robot["parts"] if part["name"] == "drivetrain"]
        for axis, low, high in ((0, "back", "front"), (1, "right", "left")):
            self.assertEqual(min(point[axis] for point in frame["outline"]), min(part[low] for part in drivetrain))
            self.assertEqual(max(point[axis] for point in frame["outline"]), max(part[high] for part in drivetrain))
        from vexsim.cascade import CascadeRobot
        flange_on_the_bottom_bands = robot["claw"]["lowest_in"] - CascadeRobot().preload_middle_under_claw()
        for upper, axis_up in (("yellow", 1.0), ("red", -1.0)):
            preload = robot["preloads"][upper]
            self.assertEqual((preload["forward_in"], preload["left_in"]), (robot["claw"]["middle_ahead_in"], 0.0))
            self.assertAlmostEqual(preload["up_in"], flange_on_the_bottom_bands, delta=1e-4)
            self.assertEqual(preload["axis"], [0.0, 0.0, axis_up])

    def test_each_3d_piece_moves_onto_vexsims_piece_with_the_lift_down_and_up(self):
        from fields import drawings
        sys.path.insert(0, str(VEXSIM))
        from vexsim.cascade import GEOMETRY, CascadeRobot
        from vexsim.presets import override_cascade
        from vexsim.units import INCH
        robot = drawings(VEXSIM)["override"]["robot"]
        components = CascadeRobot().components(override_cascade())
        self.assertEqual(len(components), len(robot["parts"]))
        hinge_names = {hinge["name"] for part in robot["parts"] for hinge in part["hinges"]}
        for lift_up, claw_closed, roller_deg, flap_deg in ((False, False, 0.0, 0.0), (True, False, 40.0, 15.0),
                                                           (True, True, 200.0, -10.0)):
            rise = GEOMETRY.lift_travel if lift_up else 0.0
            closing = GEOMETRY.jaw_travel if claw_closed else 0.0
            stage_rise = [stage / GEOMETRY.stages * rise for stage in range(GEOMETRY.stages + 1)]
            turns_deg = {name: roller_deg if name == "roller" else flap_deg for name in hinge_names}
            turns = {name: math.radians(degrees) for name, degrees in turns_deg.items()}
            parts = [component.posed(stage_rise[component.stage(GEOMETRY.stages)] * INCH, closing * INCH, 0.0, turns)
                     for component in components]
            for drawn, part in zip(robot["parts"], parts):
                with self.subTest(lift_up=lift_up, claw_closed=claw_closed, roller_deg=roller_deg,
                                  piece=drawn["key"]):
                    up, left = stage_rise[drawn["stage"]], -drawn["jaw_side"] * closing
                    profile = [(forward, height + up) for forward, height in drawn["profile"]]
                    for hinge in reversed(drawn["hinges"]):
                        profile = turned(profile, hinge, turns_deg[hinge["name"]])
                    posed = dict(back=drawn["back"], front=drawn["front"], right=drawn["right"] + left,
                                 left=drawn["left"] + left, bottom=drawn["bottom"] + up, top=drawn["top"] + up)
                    if drawn["hinges"]:
                        posed.update(back=min(f for f, _ in profile), front=max(f for f, _ in profile),
                                     bottom=min(h for _, h in profile), top=max(h for _, h in profile))
                    for side, value in posed.items():
                        self.assertAlmostEqual(value, getattr(part, side) / INCH, delta=1e-3)
                    for (forward, height), (x, z) in zip(profile, part.profile):
                        self.assertAlmostEqual(forward, x / INCH, delta=1e-3)
                        self.assertAlmostEqual(height, z / INCH, delta=1e-3)


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
        planner_only = dict(off_the_floor=LIFT_HEIGHTS["down"] + 0.5)
        self.assertEqual(set(LIFT_HEIGHTS_IN), set(vexsim_names) | set(planner_only))
        for name, height in LIFT_HEIGHTS_IN.items():
            with self.subTest(name):
                expected = planner_only[name] if name in planner_only else LIFT_HEIGHTS[vexsim_names[name]]
                self.assertAlmostEqual(height, expected, places=9)

    def test_a_held_stack_is_named_and_counted_with_the_pin_standing_in_its_cup(self):
        from mechanisms import describe_held, possession
        from vexsim import Simulator, cascade, presets
        from vexsim.override import OverrideWorld
        from vexsim.sim import SimConfig
        from vexsim.units import INCH
        stack_x, stack_y = -23.5 * INCH, -23.5 * INCH
        sim = Simulator(presets.override_cascade(), config=SimConfig(log_every=0))
        chassis_to_claw = sim.chassis.length / 2.0 - cascade.GEOMETRY.bite_seat_back * INCH
        sim.set_pose(stack_x - chassis_to_claw, stack_y, 0.0)
        world = OverrideWorld(sim, robot=cascade.CascadeRobot())
        world.mechanism.claw.command(True)
        for _ in range(200):
            world.step(0.001)
        self.assertEqual(describe_held(world), "a Cup with a yellow Pin")
        self.assertEqual(possession(world), (1, 1))

    def test_a_start_inside_a_piece_or_a_wall_is_refused_and_says_what_it_is_inside(self):
        import math
        from fields import make_world
        from mechanisms import start_overlap_error
        from vexsim import Simulator, presets
        from vexsim.override import StartOverlapError
        from vexsim.sim import SimConfig
        from vexsim.units import INCH

        def error_at(x, y, heading, preload="none"):
            sim = Simulator(presets.override_cascade(), config=SimConfig(log_every=0))
            sim.set_pose(y * INCH, -x * INCH, -math.radians(heading))
            try:
                make_world("override", sim, preload)
            except StartOverlapError as error:
                return start_overlap_error(error.overlaps)
            return None

        self.assertRegex(error_at(0, -65, 0), r"^The robot starts inside the red Alliance Station wall, 2\.\d\d in "
                                              r"deep; the left Toggle, 1\.\d\d in deep\. Move the start pose so the "
                                              r"robot is clear of them")
        self.assertRegex(error_at(40, -65, 0), r"^The robot starts inside the red Alliance Station wall, 2\.\d\d in "
                                               r"deep\. Move the start pose so the robot is clear of it")
        on_stack = error_at(0, -23.5, 0)
        self.assertRegex(on_stack, r"^The robot starts inside a Cup at \(0\.0, -23\.5\), \d\.\d\d in deep; "
                                   r"a blue/red Pin at \(0\.0, -23\.5\), \d\.\d\d in deep\. .* clear of them")
        self.assertIn("the red left Goal", error_at(23.5, -47.1, 0))
        for name in ("override_autonomous", "override_match_load", "override_stack_to_goal", "override_roller"):
            with self.subTest(name):
                project = validate_project(example(name))
                start = project["start"]
                self.assertIsNone(error_at(start["x"], start["y"], start["heading"], project["preload"]))

    def test_exported_motor_degrees_turn_into_vexsims_heights(self):
        from vexsim.cascade import GEOMETRY
        helpers = exported_helpers("cascade_helpers.hpp")
        lowest = float(re.search(r"kLowestClawIn = ([\d.]+);", helpers).group(1))
        drum = float(re.search(r"kDrumDiameterIn = ([\d.]+);", helpers).group(1))
        ratio = float(re.search(r"kMotorTurnsPerDrumTurn = ([\d.]+);", helpers).group(1))
        stages = int(re.search(r"kLiftStages = (\d+);", helpers).group(1))
        self.assertEqual((drum, ratio, stages), (GEOMETRY.drum_diameter, GEOMETRY.lift_gear_ratio, GEOMETRY.stages))
        self.assertAlmostEqual(lowest, GEOMETRY.grip_lowest, places=9)

    def test_a_robot_that_falls_over_says_so_in_that_step_only_and_one_that_rocks_says_how_far(self):
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
        [fall] = fell.fall_readings(started=1.5)
        self.assertRegex(fall["message"], r"^The robot fell over at 2\.\d\d s\. It tipped toward its front past "
                                          r"19°, where its centre of mass, 15\.0 in up, was over the lowest edge of "
                                          r"the robot on that side\. It lies there for the rest of the run\.$")
        self.assertEqual(fell.fall_readings(started=fell.fell_at + 0.001), [])
        self.assertEqual(fell.rock_lines(), [])
        rocked = brake_from_full_speed(13.0)
        self.assertEqual(rocked.fall_readings(started=1.5), [])
        self.assertRegex(rocked.rock_lines()[0], r"^The robot rocked 7\.\d° onto its front wheels\. "
                                                 r"It falls over at 22°\.$")
        level = brake_from_full_speed(5.0)
        self.assertEqual((level.fall_readings(started=1.5), level.rock_lines()), ([], []))

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

    def test_1_run_plans_simulates_and_reports_every_step(self):
        project = example("speed_zones")
        project["steps"].append(dict(type="turn", heading=90))
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertTrue(result["legal"])
        self.assertEqual(result["summary"]["ran_steps"], 2)
        stages = {stage["id"]: stage for stage in result["stages"]}
        self.assertEqual([stage["status"] for stage in result["stages"]].count("failed"), 0)
        self.assertIn("Planned 1 path", stages["plan"]["detail"])
        path = result["steps"][0]
        self.assertTrue(any(item["id"] == "position" and re.fullmatch(r"Ended \d+\.\d\d in from the target\.", item["message"])
                            for item in path["readings"]))
        self.assertFalse(any("passed" in item or "limit" in item for item in path["readings"]))
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

    def test_4_a_step_that_falls_short_says_what_it_did_and_the_run_goes_on(self):
        project = dict(steps=[dict(type="drive", distance=30, timeout_ms=300), dict(type="turn", heading=90)],
                       stop_on_failure=True)
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        short, turn = result["steps"]
        messages = [item["message"] for item in short["readings"]]
        self.assertIn("The 0.30 s time limit ended the motion.", messages)
        self.assertTrue(any(re.fullmatch(r"Ended \d+\.\d\d in from the target\.", message) for message in messages))
        slide = next(item for item in short["readings"] if item["id"] == "stopping")
        self.assertRegex(slide["message"], r"^The motion ended with the robot at \d\d\.\d in/s\. It braked to a stop "
                                           r"0\.\d\d s and \d+\.\d in later\.$")
        self.assertGreater(slide["measured"], 2)
        self.assertEqual(len(short["notes"]), 1, short["notes"])
        self.assertRegex(short["notes"][0], r"^The time limit ended the motion \d+\.\d in from its target\. If the step "
                                            r"should reach its target, raise the time limit")
        self.assertTrue(turn["executed"])
        self.assertEqual(result["summary"], dict(steps=2, ran_steps=2, skipped_steps=0, broke_rule_in_step=None,
                                                 simulated_s=result["summary"]["simulated_s"]))
        self.assertNotIn("stop_on_failure", result["project"])

    def test_5_a_chained_drive_hands_off_without_stopping(self):
        project = dict(steps=[dict(type="drive", distance=24, chain=True), dict(type="turn", heading=90)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        drive, turn = strict_json((output / "result.json").read_text())["steps"]
        self.assertNotIn("stopped", [item["id"] for item in drive["readings"]])
        self.assertIn("stopped", [item["id"] for item in turn["readings"]])
        self.assertEqual(drive["rule_breaks"], [])
        self.assertTrue(any(item["message"].startswith("Handed off") for item in drive["readings"]))
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
        project = dict(field="override", robot=dict(preset="override_cascade", tracking="horizontal"), start=dict(x=36, y=-50, heading=180),
                       steps=[dict(type="drive", distance=20, timeout_ms=1500)])
        done, output = self.run_routine(project)
        result = strict_json((output / "result.json").read_text())
        step = result["steps"][0]
        hit = next(line for line in step["info"] if line.startswith("Hit the red Alliance Station wall at"))
        self.assertRegex(hit, r"peak \d+ N")
        self.assertEqual(step["contacts"][0]["element"], "the red Alliance Station wall")
        self.assertGreater(step["contacts"][0]["peak_force_n"], 20)
        self.assertTrue(any(frame.get("contact") == "the red Alliance Station wall" for frame in result["trace"]))

    def test_8_the_claw_carries_a_stack_to_a_neutral_goal(self):
        done, output = self.run_routine(example("override_stack_to_goal"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertTrue(result["legal"])
        self.assertEqual(result["summary"]["skipped_steps"], 0)
        info = [line for row in result["steps"] for line in row["info"]]
        grips = [line for line in info if line.startswith("Gripped")]
        drops = [line for line in info if line.startswith("Dropped")]
        self.assertEqual(len(grips), 1, grips)
        self.assertEqual(len(drops), 1, drops)
        self.assertRegex(grips[0], r"^Gripped a Cup with a yellow Pin at \(2[34], -2[34]\)")
        self.assertRegex(drops[0], r"^Dropped a (Cup with a )?yellow Pin at \(-2[345], -4[678]\).*: the claw opened\.")
        def on_the_goal(pose):
            return math.hypot(pose[0] + 23.55, pose[1] + 47.09) < 0.2
        already_there = {str(piece["id"]) for piece in result["pieces"] if on_the_goal(piece["pose"])}
        last_seen = {}
        for frame in result["trace"]:
            last_seen.update({key: pose for key, pose in frame["pieces"].items() if pose})
        set_down = [pose for key, pose in last_seen.items() if key not in already_there and on_the_goal(pose)]
        self.assertEqual(len(set_down), 2, set_down)
        for pose in set_down:
            self.assertGreater(abs(pose[5]), math.cos(math.radians(2.0)), pose)
        self.assertEqual(len(result["pieces"]), 73)
        first, last = result["trace"][0], result["trace"][-1]
        self.assertIsNone(first["holding"])
        self.assertIsNone(last["holding"])
        self.assertLess(last["lift_height_in"], 6.9)
        self.assertEqual(result["score"]["red"]["total"], 0)
        rules = result["autonomous"]["rules"]
        self.assertEqual(rules["most_held"], dict(pins=1, cups=1, passed=True))
        self.assertLessEqual(max(rules["start_in"][key] for key in ("length", "width", "height")), 18.0)
        self.assertEqual(rules["motors_w"], dict(drive=44, total=88.0, drive_limit=55.0, total_limit=88.0, passed=True))
        self.assertIn("The drive motors are rated 44 W and all the motors 88 W. The limits are 55 W for the drive "
                      "(R11) and 88 W in all (R10).", result["steps"][0]["info"])

    def test_9_override_autonomous_example_turns_the_left_toggle_and_scores_a_stack(self):
        done, output = self.run_routine(example("override_autonomous"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertTrue(result["legal"])
        self.assertEqual(result["summary"]["skipped_steps"], 0)
        self.assertLessEqual(result["summary"]["simulated_s"], 13.0)
        autonomous = result["autonomous"]
        self.assertEqual((autonomous["red"], autonomous["blue"]), (20, 0))
        self.assertEqual(autonomous["bonus"], dict(red=12, blue=0))
        self.assertEqual(autonomous["toggles"], dict(left="red", bottom="yellow", top="yellow", right="yellow"))
        self.assertFalse(autonomous["win_point"]["earned"])
        self.assertTrue(all(rule["passed"] for rule in autonomous["rules"].values()), autonomous["rules"])
        goal_hits = [contact for row in result["steps"] for contact in row["contacts"]
                     if contact["element"].endswith("Goal")]
        self.assertEqual({contact["element"] for contact in goal_hits}, {"the neutral left Goal"})
        self.assertLess(max(contact["peak_force_n"] for contact in goal_hits), 150)
        info = [line for row in result["steps"] for line in row["info"]]
        turned = [line.split(" now")[0] for line in info if line.startswith("Turned the")]
        self.assertEqual(turned, ["Turned the left Toggle: red"])
        grips = [line for line in info if line.startswith("Gripped")]
        drops = [line for line in info if line.startswith("Dropped")]
        self.assertEqual(len(grips), 1, grips)
        self.assertEqual(len(drops), 1, drops)
        self.assertRegex(drops[0], r"^Dropped a Cup with a yellow Pin at \(-2[34], -4[67]\).*: the claw opened\.")

    def test_10_a_match_load_in_the_autonomous_period_breaks_sg11a(self):
        project = dict(field="override", preload="none", robot=dict(preset="override_cascade", tracking="horizontal"),
                       start=dict(x=0, y=-61.7, heading=0),
                       steps=[dict(type="match_load", loader="red_left", match_load_pin="red_yellow_up")])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        row = result["steps"][0]
        self.assertFalse(result["legal"])
        self.assertEqual(result["summary"]["broke_rule_in_step"], 0)
        self.assertEqual(len(row["rule_breaks"]), 1, row["rule_breaks"])
        self.assertTrue(row["rule_breaks"][0].startswith(
            "Match Loads may only be loaded in the Driver Controlled Period (SG11a)."))
        self.assertIn("The Drive Team stood a Cup under the red left Loader with a red/yellow Pin in it, yellow half up, "
                      "0.00 s into the match.", [item["message"] for item in row["readings"]])

    def test_10b_a_wait_past_15_s_lets_a_match_load_score_in_driver_control(self):
        done, output = self.run_routine(example("override_match_load"))
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        self.assertTrue(result["legal"])
        self.assertGreater(result["summary"]["simulated_s"], 20.0)
        wait, match_load = result["steps"][:2]
        self.assertIn("Waited 15.00 s, to 15.00 s into the match.", [item["message"] for item in wait["readings"]])
        self.assertAlmostEqual(match_load["start_time_s"], 15.0, places=3)
        self.assertEqual(match_load["rule_breaks"], [])
        self.assertTrue(any(re.fullmatch(r"The Drive Team stood .+, 15\.00 s into the match\.", item["message"])
                            for item in match_load["readings"]), match_load["readings"])
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

    def test_10c_a_run_that_starts_inside_a_goal_is_refused_before_it_simulates(self):
        project = dict(field="override", preload="none", robot=dict(preset="override_cascade", tracking="horizontal"),
                       start=dict(x=23.5, y=-47.1, heading=0), steps=[dict(type="drive", distance=10)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 2, done.stderr[-3000:])
        error = strict_json((output / "error.json").read_text())
        self.assertRegex(error["error"], r"^The robot starts inside the red left Goal, \d\.\d\d in deep\.")
        self.assertFalse((output / "result.json").exists())

    def test_11_a_step_stopped_by_a_goal_names_the_goal_not_the_controller(self):
        project = dict(field="override", preload="none", robot=dict(preset="override_cascade", tracking="horizontal"),
                       start=dict(x=8, y=-47.1, heading=90), steps=[dict(type="drive", distance=30, timeout_ms=2500)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        row = strict_json((output / "result.json").read_text())["steps"][0]
        self.assertEqual(row["rule_breaks"], [])
        self.assertTrue(any(line.startswith("Hit the red left Goal") for line in row["info"]))
        self.assertEqual(len(row["notes"]), 1, row["notes"])
        self.assertRegex(row["notes"][0], r"^The robot was stopped by the red left Goal: it moved \d\.\d in of the "
                                          r"30\.0 in asked for\. The controller was not at fault\.")

    def test_12_each_roller_flap_turns_a_toggle_one_face_top_toward_the_wall_and_it_stays(self):
        def roll(start, roller_deg):
            project = dict(field="override", preload="none",
                           robot=dict(preset="override_cascade", tracking="horizontal"), start=start,
                           steps=[dict(type="roller", roller_deg=roller_deg, roller_volts=12, press_volts=3,
                                       timeout_ms=2000),
                                  dict(type="drive", distance=6)])
            done, output = self.run_routine(project)
            self.assertEqual(done.returncode, 0, done.stderr[-3000:])
            result = strict_json((output / "result.json").read_text())
            turned = [line.split(" now")[0] for row in result["steps"] for line in row["info"]
                      if line.startswith("Turned the")]
            return result, turned

        for toggle, start, roller_deg, faces in (("left", dict(x=0, y=-61.7, heading=0), 300, ["blue"]),
                                                 ("right", dict(x=0, y=61.7, heading=180), 470, ["red", "blue"])):
            with self.subTest(toggle):
                result, turned = roll(start, roller_deg)
                self.assertEqual(turned, [f"Turned the {toggle} Toggle: {faces[-1]}"])
                self.assertEqual(result["autonomous"]["toggles"][toggle], faces[-1])
                self.assertEqual(result["steps"][0]["move_result"], "Reached the preset")
                self.assertRegex(result["steps"][0]["readings"][0]["message"],
                                 rf"^The roller turned {roller_deg // 10}\d° in \d\.\d\d s\.$")

    def test_14_min_voltage_holds_up_a_chained_turn_and_too_much_keeps_a_drive_from_settling(self):
        project = dict(steps=[dict(type="turn", heading=90, chain=True, min_volts=4),
                              dict(type="drive", distance=24, timeout_ms=2000, min_volts=6)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        turn, drive = result["steps"]
        turning = [frame for frame in result["trace"] if frame["step"] == 0 and frame["phase"] == "motion"
                   and frame["t"] - turn["start_time_s"] > 0.25]
        self.assertGreater(len(turning), 5)
        self.assertGreaterEqual(min(abs(frame["left_volts"]) for frame in turning), 4 - 1e-9)
        self.assertIn("The 2.00 s time limit ended the motion.", [item["message"] for item in drive["readings"]])
        self.assertTrue(drive["notes"][0].startswith("The 6 V Min voltage kept the drive from slowing enough to "
                                                     "settle"), drive["notes"])

    def test_15_a_lift_cut_off_by_its_time_limit_reports_the_claw_where_it_was_then(self):
        project = dict(field="override", preload="none", robot=dict(preset="override_cascade", tracking="horizontal"),
                       start=dict(x=0, y=-50, heading=0), steps=[dict(type="lift", lift_preset="top", timeout_ms=200)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 0, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        row = result["steps"][0]
        found = re.fullmatch(r"The lift had not reached Top: .* when its 0\.20 s time limit ran out: the claw is "
                             r"(\d+\.\d\d) in up\.", row["readings"][0]["message"])
        self.assertIsNotNone(found, row["readings"])
        at_limit = next(frame for frame in result["trace"] if frame["t"] >= row["return_time_s"])
        self.assertAlmostEqual(float(found.group(1)), at_limit["lift_height_in"], delta=0.05)
        self.assertLess(at_limit["lift_height_in"], result["trace"][-1]["lift_height_in"])

    def test_13_a_drive_over_55_w_breaks_r11_on_the_first_step_and_stops_the_run(self):
        project = dict(field="override", preload="none", robot=dict(preset="six_motor_450"),
                       start=dict(x=0, y=-61.7, heading=0), steps=[dict(type="drive", distance=5),
                                                                  dict(type="drive", distance=-5)])
        done, output = self.run_routine(project)
        self.assertEqual(done.returncode, 1, done.stderr[-3000:])
        result = strict_json((output / "result.json").read_text())
        first, second = result["steps"]
        self.assertIn("The drive motors are rated 66 W and all the motors 110 W. The limits are 55 W for the drive "
                      "(R11) and 88 W in all (R10).", first["rule_breaks"])
        self.assertFalse(second["executed"])
        self.assertEqual(second["skipped_because"], "Not run: step 1 broke a rule, so the run stopped there.")
        self.assertEqual(result["summary"]["broke_rule_in_step"], 0)
        self.assertEqual(result["autonomous"]["rules"]["motors_w"]["passed"], False)


if __name__ == "__main__":
    unittest.main()
