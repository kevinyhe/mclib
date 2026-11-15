"""robotlog.js: reading the CSV mclib's telemetry Logger writes on the robot,
and setting it against a simulated run. Runs the page's own code in node."""
import json
from pathlib import Path
import subprocess
import unittest

ROBOT_LOG_JS = Path(__file__).resolve().parent / "static" / "robotlog.js"
NODE_SCRIPT = """
const robotLog = require(process.argv[1]);
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
let answer;
try {
  const log = robotLog.parseRobotLog(input.text);
  answer = { log, at: input.at.map((t) => robotLog.robotLogPoseAt(log, input.offset, t)),
             compared: input.result ? robotLog.compareRobotLog(log, input.offset, input.result) : null };
} catch (error) {
  answer = { error: error.message };
}
process.stdout.write(JSON.stringify(answer));
"""

LOGGER_CSV = """PROS terminal banner
t_ms,x_in,y_in,heading_deg
1500,0,-48,0
1520,0,-47.5,0.5
# the controller did not take "hello"
1540,0.1,-47,nan
1560,0.2,-46.5,1
# rows=4 dropped=2 discarded=0
"""


def run_node(text, offset=0, at=(), result=None):
    done = subprocess.run(["node", "-e", NODE_SCRIPT, str(ROBOT_LOG_JS)], capture_output=True, text=True, check=True,
                          input=json.dumps(dict(text=text, offset=offset, at=list(at), result=result)))
    return json.loads(done.stdout)


def frame(t, x, y, reset=False):
    return dict(t=t, estimated_x=x, estimated_y=y, coordinate_reset=reset)


def simulated_run():
    """Two steps: a path straight up from (0, 0) to (0, 10) from 0 to 1 s,
    then a turn in place to 90 deg from 1 to 1.5 s."""
    trace = [frame(t / 10, 0, t) for t in range(11)] + [frame(1 + t / 10, 0, 10) for t in range(1, 6)]
    steps = [dict(index=0, executed=True, start_time_s=0, return_time_s=1, estimated_pose_at_exit=[0, 10, 0],
                  target=dict(x=0, y=10, heading=0), coordinate_reset=False),
             dict(index=1, executed=True, start_time_s=1, return_time_s=1.5, estimated_pose_at_exit=[0, 10, 90],
                  target=dict(x=None, y=None, heading=90), coordinate_reset=False)]
    plans = [dict(states=[[0, t / 10, t, 0, t, 0, 0, 0] for t in range(11)]), None]
    return dict(trace=trace, steps=steps, plans=plans)


def log_text(rows, step=False):
    header = "t_ms,x_in,y_in,heading_deg" + (",step" if step else "")
    return "\n".join([header, *(",".join(str(value) for value in row) for row in rows)]) + "\n"


class ParseTests(unittest.TestCase):
    def test_reads_the_loggers_csv_from_its_header_on_and_skips_hash_lines(self):
        answer = run_node(LOGGER_CSV)
        log = answer["log"]
        self.assertEqual([sample["t"] for sample in log["samples"]], [0, .02, .04, .06])
        self.assertEqual(log["samples"][1], dict(t=.02, x=0, y=-47.5, heading=.5, step=None))
        self.assertIsNone(log["samples"][2]["heading"])
        self.assertEqual((log["dropped"], log["hasHeading"], log["hasStep"]), (2, True, False))
        self.assertAlmostEqual(log["duration"], .06)

    def test_a_log_without_the_pose_says_what_to_log(self):
        error = run_node("t_ms,left_cmd_V\n0,1\n20,2\n")["error"]
        self.assertEqual(error, 'The log needs x_in and y_in columns: log the pose with addLength("x") and '
                                'addLength("y"). Its columns are t_ms, left_cmd_V.')
        self.assertIn("No header line starting with t_ms", run_node("x,y\n1,2\n")["error"])

    def test_a_broken_row_or_a_second_run_is_named_by_line(self):
        self.assertEqual(run_node("t_ms,x_in,y_in\n0,1,2\n20,1\n")["error"],
                         "Line 3 has 2 values where the header names 3: 20,1")
        self.assertEqual(run_node("t_ms,x_in,y_in\n0,1,2\n20,1,\n")["error"],
                         "Line 3 has 3 values where the header names 3: 20,1,")
        self.assertEqual(run_node("t_ms,x_in,y_in\n500,1,2\n20,1,2\n")["error"],
                         "Line 3 goes back in time, from 500 to 20 ms. Keep one run per file.")
        self.assertEqual(run_node("t_ms,x_in,y_in\n0,1,2\n")["error"], "The log has fewer than two rows.")

    def test_the_pose_at_a_time_is_the_last_row_by_then_after_the_offset(self):
        at = run_node(LOGGER_CSV, offset=1, at=(.5, 1.0, 1.03, 9))["at"]
        self.assertIsNone(at[0])
        self.assertEqual([pose["y"] for pose in at[1:]], [-48, -47.5, -46.5])


class CompareTests(unittest.TestCase):
    def test_gaps_are_measured_against_the_simulated_odometry_the_plan_and_the_target(self):
        rows = [[t * 100, 1, t, 0] for t in range(11)] + [[1000 + t * 100, 1, 10, 80] for t in range(1, 6)]
        path, turn = run_node(log_text(rows), result=simulated_run())["compared"]
        self.assertEqual(path["samples"], 11)
        self.assertAlmostEqual(path["simulated_largest_in"], 1)
        self.assertAlmostEqual(path["simulated_end_in"], 1)
        self.assertAlmostEqual(path["target_in"], 1)
        self.assertAlmostEqual(path["plan_largest_in"], 1)
        self.assertEqual(turn["target_heading_deg"], 10)
        self.assertIsNone(turn["target_in"])
        self.assertIsNone(turn["plan_largest_in"])

    def test_a_row_between_simulated_frames_is_set_against_the_odometry_between_them(self):
        rows = [[t * 50, 1, t / 2, 0] for t in range(21)]
        path, _ = run_node(log_text(rows), result=simulated_run())["compared"]
        self.assertEqual(path["samples"], 21)
        self.assertAlmostEqual(path["simulated_largest_in"], 1)

    def test_a_step_column_splits_the_log_by_the_robots_own_step_times(self):
        rows = [[t * 200, 0, t * 2, 0, 1] for t in range(6)] + [[1000 + t * 200, 0, 10, 90, 2] for t in range(1, 4)]
        path, turn = run_node(log_text(rows, step=True), result=simulated_run())["compared"]
        self.assertEqual((path["from"], path["to"]), (0, 1))
        self.assertAlmostEqual(path["simulated_largest_in"], 0)
        self.assertEqual((turn["from"], turn["to"], turn["samples"]), (1.2, 1.6, 3))

    def test_a_log_offset_past_a_step_leaves_it_without_rows(self):
        rows = [[t * 100, 0, t, 0] for t in range(6)]
        path, turn = run_node(log_text(rows), offset=1.2, result=simulated_run())["compared"]
        self.assertEqual(path, dict(index=0, samples=0))
        self.assertEqual(turn["samples"], 4)


if __name__ == "__main__":
    unittest.main()
