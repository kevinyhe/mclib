"""geometry.js draws what geometry.py plans and the engine runs. These tests
run both on the same routines and require the same numbers."""
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from geometry import designed_poses, path_layout, wrap  # noqa: E402
from project import PRESETS, ProjectError, strict_json, validate_project  # noqa: E402

GEOMETRY_JS = HERE / "static" / "geometry.js"
NODE_SCRIPT = """
const geometry = require(process.argv[1]);
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
const answers = input.projects.map((project) => {
  const poses = geometry.designedPoses(project, input.track);
  const layouts = project.steps.map((step, index) => step.type === "path" && step.points.length
    ? geometry.pathLayout(step, poses[index].start) : null);
  return { poses, layouts, mirrored: geometry.mirrorProject(geometry.mirrorProject(project, input.axis), input.axis) };
});
process.stdout.write(JSON.stringify(answers));
"""


def example(name):
    raw = strict_json((HERE / "examples" / f"{name}.json").read_text())
    raw.pop("description")
    return validate_project(raw)


def every_step_project():
    return validate_project(dict(
        name="Every step", start=dict(x=-30, y=-50, heading=20),
        steps=[
            dict(type="path", start_handle=10,
                 points=[dict(x=-10, y=-20, heading=45, handle_in=6, handle_out=14), dict(x=20, y=0),
                         dict(x=0, y=-24), dict(x=-12, y=-40, heading=None, handle_in=None, handle_out=3)],
                 sections=[dict(speed_percent=None, reverse=False), dict(speed_percent=30, reverse=False),
                           dict(speed_percent=None, reverse=True), dict(speed_percent=80, reverse=True)]),
            dict(type="drive", distance=-18),
            dict(type="turn", heading=-135),
            dict(type="turn_to_point", x=30, y=30, direction=-1),
            dict(type="point", x=10, y=10, direction=1),
            dict(type="boomerang", x=-20, y=30, heading=270, direction=-1, lead=.4),
            dict(type="arc", heading=60, radius=-20),
            dict(type="reverse_arc", heading=0, radius=15),
            dict(type="swing", heading=-90, direction=-1),
            dict(type="wall_reset", x=0, y=63, heading=0),
        ]))


def run_node(projects, axis):
    done = subprocess.run(["node", "-e", NODE_SCRIPT, str(GEOMETRY_JS)], input=json.dumps(
        dict(projects=projects, track=PRESETS["six_motor_450"]["track_width_in"], axis=axis)),
        capture_output=True, text=True, check=True)
    return json.loads(done.stdout)


class SameGeometryTests(unittest.TestCase):
    def assertClose(self, python_value, js_value, where):
        self.assertTrue(math.isclose(python_value, js_value, abs_tol=1e-9), f"{where}: {python_value} != {js_value}")

    def test_layouts_and_end_poses_match(self):
        projects = [example(name) for name in ("s_curve", "speed_zones", "out_and_back", "mixed_steps")]
        projects.append(every_step_project())
        answers = run_node(projects, "x")
        for project, answer in zip(projects, answers):
            poses = designed_poses(project)
            for index, ((start, end), js) in enumerate(zip(poses, answer["poses"])):
                where = f"{project['name']} step {index + 1}"
                for python_pose, js_pose in ((start, js["start"]), (end, js["end"])):
                    for value, key in zip(python_pose, ("x", "y", "heading")):
                        self.assertClose(value, js_pose[key], f"{where} {key}")
                self.assertIsNone(js["problem"])
                step = project["steps"][index]
                if step["type"] != "path":
                    continue
                layout = path_layout(step, start)
                for i, (heading, js_heading) in enumerate(zip(layout["headings"], answer["layouts"][index]["headings"])):
                    self.assertClose(heading, js_heading, f"{where} heading {i}")
                for i, (section, js_section) in enumerate(zip(layout["sections"], answer["layouts"][index]["sections"])):
                    for key, js_key in (("start", "start"), ("start_handle", "startHandle"),
                                        ("end_handle", "endHandle"), ("end", "end")):
                        self.assertClose(section[key][0], js_section[js_key]["x"], f"{where} section {i} {key} x")
                        self.assertClose(section[key][1], js_section[js_key]["y"], f"{where} section {i} {key} y")
                    self.assertEqual(section["reverse"], js_section["reverse"])

    def test_mirroring_twice_gives_the_routine_back(self):
        def same_headings(value):
            if isinstance(value, list):
                return [same_headings(item) for item in value]
            if not isinstance(value, dict):
                return value
            return {key: wrap(item) if key == "heading" and isinstance(item, (int, float)) else same_headings(item)
                    for key, item in value.items()}

        project = every_step_project()
        for axis in ("x", "y"):
            mirrored = run_node([project], axis)[0]["mirrored"]
            self.assertEqual(json.dumps(same_headings(mirrored), sort_keys=True),
                             json.dumps(same_headings(project), sort_keys=True))

    def test_points_too_close_are_refused_by_both(self):
        project = example("s_curve")
        project["steps"][0]["points"][1].update(x=24.2, y=-12.1)
        with self.assertRaises(ProjectError) as caught:
            designed_poses(project)
        self.assertEqual(caught.exception.step, 0)
        self.assertIn("point 2 is 0.22 in from point 1. Points must be at least 0.5 in apart", str(caught.exception))
        problem = run_node([project], "x")[0]["poses"][0]["problem"]
        self.assertIn("point 2 is 0.22 in from point 1", problem)


NEAREST_SCRIPT = """
const geometry = require(process.argv[1]);
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
process.stdout.write(JSON.stringify(input.points.map((point) => geometry.planStateNearest(input.states, point))))
"""


def nearest_plan_states(states, points):
    done = subprocess.run(["node", "-e", NEAREST_SCRIPT, str(GEOMETRY_JS)],
                          input=json.dumps(dict(states=states, points=points)), capture_output=True, text=True,
                          check=True)
    return json.loads(done.stdout)


class PathProbeTests(unittest.TestCase):
    FORWARD_THEN_BACK = [[0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         [0, 1.0, 20.0, 0.0, 20.0, 0.0, 40.0, 0.0],
                         [0, 2.0, 40.0, 0.0, 40.0, 0.0, 0.0, 0.0],
                         [1, 2.0, 40.0, 0.0, 40.0, 180.0, 0.0, 0.0],
                         [1, 3.0, 50.0, 10.0, 40.0, 270.0, -20.0, 0.0]]

    def test_a_point_beside_the_path_reads_the_distance_time_and_speed_there(self):
        near = nearest_plan_states(self.FORWARD_THEN_BACK, [dict(x=3, y=10)])[0]
        self.assertAlmostEqual(near["gap_in"], 3.0)
        part, time, distance, x, y, _, speed, _ = near["state"]
        self.assertEqual((part, x, y), (0, 0.0, 10.0))
        self.assertAlmostEqual(time, 0.5)
        self.assertAlmostEqual(distance, 10.0)
        self.assertAlmostEqual(speed, 20.0)

    def test_a_reversed_part_reads_its_distance_from_the_path_start_and_a_backward_speed(self):
        near = nearest_plan_states(self.FORWARD_THEN_BACK, [dict(x=5, y=41)])[0]
        _, time, distance, _, _, _, speed, _ = near["state"]
        self.assertAlmostEqual(time, 2.5)
        self.assertAlmostEqual(distance, 45.0)
        self.assertAlmostEqual(speed, -10.0)

    def test_points_past_either_end_read_the_end(self):
        before, after = nearest_plan_states(self.FORWARD_THEN_BACK, [dict(x=0, y=-6), dict(x=16, y=40)])
        self.assertEqual(before["state"][1:3], [0.0, 0.0])
        self.assertAlmostEqual(before["gap_in"], 6.0)
        self.assertEqual(after["state"][1:3], [3.0, 50.0])
        self.assertAlmostEqual(after["gap_in"], 6.0)


FIRST_REACH_SCRIPT = """
const geometry = require(process.argv[1]);
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
process.stdout.write(JSON.stringify(input.reaches.map((reach) => geometry.firstReach(input.route, reach))))
"""


def first_reaches(route, reaches):
    done = subprocess.run(["node", "-e", FIRST_REACH_SCRIPT, str(GEOMETRY_JS)],
                          input=json.dumps(dict(route=route, reaches=reaches)), capture_output=True, text=True,
                          check=True)
    return json.loads(done.stdout)


class ActionPlaceTests(unittest.TestCase):
    """A distance action runs once the robot is that far, in a straight line,
    from where its step started (cascade::Rig::tick), not that far along the path."""
    RIGHT_THEN_UP = [dict(x=0, y=0), dict(x=10, y=0), dict(x=10, y=10)]

    def test_a_reach_past_the_first_corner_lands_where_the_straight_line_distance_first_gets_there(self):
        at = first_reaches(self.RIGHT_THEN_UP, [12])[0]
        self.assertAlmostEqual(at["x"], 10.0)
        self.assertAlmostEqual(at["y"], math.sqrt(12 ** 2 - 10 ** 2))

    def test_no_reach_lands_at_the_start_and_one_the_route_never_gets_is_none(self):
        start, beyond = first_reaches(self.RIGHT_THEN_UP, [0, 15])
        self.assertEqual(start, dict(x=0, y=0))
        self.assertIsNone(beyond)

    def test_a_route_that_comes_back_counts_its_first_reach(self):
        out_and_back = [dict(x=0, y=0), dict(x=20, y=0), dict(x=0, y=0)]
        self.assertEqual(first_reaches(out_and_back, [5])[0], dict(x=5, y=0))


if __name__ == "__main__":
    unittest.main()
