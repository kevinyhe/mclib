"""Run many: how each run is varied, what counts as a step finishing or
ending on target, and how the report sorts and spreads the runs. No C++ or
physics runs here."""
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from batch import (badness, batch_report, end_spread, repeat_settings, run_outcome, step_finished,  # noqa: E402
                   step_outcome, thinned_path, variations)
from project import default_project, validate_project  # noqa: E402

SETTINGS = repeat_settings({})


def row(**changes):
    value = dict(index=0, type="drive", executed=True, configuration_error=None, rule_breaks=[], coordinate_reset=False,
                 move_result=None, duration_s=1.0, timeout_ms=4000, position_error_in=.5, heading_error_deg=1.0)
    value.update(changes)
    return value


def step(**changes):
    return dict(dict(type="drive", chain=False), **changes)


def outcome(red=10, legal=True, on_target=1, end_error_in=1.0, final_red=None):
    return dict(legal=legal, autonomous=dict(red=red, blue=0, win_point=False, bonus=True),
                final_score=None if final_red is None else dict(red=final_red, blue=0),
                steps=[dict(on_target=index < on_target) for index in range(3)], end_error_in=end_error_in)


class VariationTests(unittest.TestCase):
    def test_runs_stay_inside_the_settings_and_the_same_seed_repeats_them(self):
        project = validate_project(dict(default_project(), conditions=dict(battery_percent=90, grip_percent=80)))
        settings = repeat_settings(dict(count=50, placement_in=.5, placement_deg=1, grip_percent=10, battery_percent=30))
        runs = variations(project, settings, 3)
        self.assertEqual(len(runs), 50)
        self.assertEqual(runs, variations(project, settings, 3))
        self.assertNotEqual(runs, variations(project, settings, 4))
        for run in runs:
            self.assertLessEqual(abs(run["placement_x_in"]), .5)
            self.assertLessEqual(abs(run["placement_y_in"]), .5)
            self.assertLessEqual(abs(run["placement_heading_deg"]), 1)
            self.assertTrue(72 <= run["grip_percent"] <= 88, run["grip_percent"])
            self.assertTrue(60 <= run["battery_percent"] <= 90, run["battery_percent"])
            validate_project(dict(project, conditions=run))

    def test_a_run_is_never_varied_past_what_a_routine_allows(self):
        project = validate_project(dict(default_project(), conditions=dict(battery_percent=10, placement_x_in=5.9)))
        settings = repeat_settings(dict(count=30, placement_in=6, battery_percent=100))
        for run in variations(project, settings, 1):
            validate_project(dict(project, conditions=run))
            self.assertGreaterEqual(run["battery_percent"], 0)
            self.assertLessEqual(run["placement_x_in"], 6)

    def test_settings_name_what_is_wrong(self):
        with self.assertRaisesRegex(ValueError, "Run many: Placement, up to must be between 0 and 6 in, not 7"):
            repeat_settings(dict(placement_in=7))


class StepTests(unittest.TestCase):
    def test_a_motion_finishes_when_it_ends_before_its_time_limit(self):
        self.assertTrue(step_finished(row(duration_s=3.99, timeout_ms=4000)))
        self.assertFalse(step_finished(row(duration_s=4.0, timeout_ms=4000)))

    def test_mechanisms_wall_resets_and_loads_finish_on_their_own_terms(self):
        self.assertTrue(step_finished(row(type="lift", move_result="Reached the preset", duration_s=1.5,
                                          timeout_ms=1500)))
        self.assertFalse(step_finished(row(type="roller", move_result="Used the whole time limit")))
        self.assertTrue(step_finished(row(type="wall_reset", coordinate_reset=True, duration_s=9, timeout_ms=1)))
        self.assertFalse(step_finished(row(type="wall_reset", coordinate_reset=False)))
        self.assertTrue(step_finished(row(type="claw", duration_s=.17, timeout_ms=170)))
        self.assertFalse(step_finished(row(type="match_load", configuration_error="no room")))
        self.assertFalse(step_finished(row(rule_breaks=["SG11a"])))

    def test_on_target_needs_the_position_and_heading_within_the_settings(self):
        self.assertTrue(step_outcome(row(position_error_in=2.0, heading_error_deg=5.0), step(), SETTINGS)["on_target"])
        self.assertFalse(step_outcome(row(position_error_in=2.1), step(), SETTINGS)["on_target"])
        self.assertFalse(step_outcome(row(heading_error_deg=5.1), step(), SETTINGS)["on_target"])
        self.assertFalse(step_outcome(row(duration_s=4), step(), SETTINGS)["on_target"])

    def test_a_step_that_keeps_moving_is_held_to_its_heading_only_if_heading_is_its_goal(self):
        chained = step_outcome(row(heading_error_deg=30), step(chain=True), SETTINGS)
        self.assertTrue(chained["on_target"])
        self.assertIsNone(chained["heading_error_deg"])
        arc = step_outcome(row(type="arc", heading_error_deg=30), step(type="arc", chain=True), SETTINGS)
        self.assertFalse(arc["on_target"])

    def test_a_skipped_step_did_not_run(self):
        self.assertEqual(step_outcome(dict(index=2, executed=False), step(), SETTINGS), dict(index=2, ran=False))


class OutcomeTests(unittest.TestCase):
    def test_a_run_comes_down_to_its_steps_score_end_and_path(self):
        trace = [dict(t=t / 100, true_x=t / 100, true_y=0, true_heading=90) for t in range(101)]
        result = dict(
            legal=True, summary=dict(simulated_s=1.0), trace=trace,
            project=dict(steps=[step(), step(type="lift")]),
            steps=[row(position_error_in=.4), row(index=1, type="lift", move_result="Reached the preset",
                                                   position_error_in=None, heading_error_deg=None)],
            autonomous=dict(red=20, blue=5, bonus=dict(red=8, blue=0), win_point=dict(earned=False)),
            score=dict(red=dict(total=28), blue=dict(total=5)),
            placement=dict(placement_x_in=0, placement_y_in=-.3, placement_heading_deg=.5, moved_back_from=["the left Toggle"]))
        value = run_outcome(result, SETTINGS)
        self.assertEqual([item["on_target"] for item in value["steps"]], [True, True])
        self.assertEqual(value["autonomous"], dict(red=20, blue=5, win_point=False, bonus=True))
        self.assertEqual(value["final_score"], dict(red=28, blue=5))
        self.assertEqual(value["end_pose"], [1.0, 0, 90])
        self.assertEqual(value["end_error_in"], .4)
        self.assertEqual(len(value["path"]), 11)
        self.assertEqual(value["placement"], dict(placement_x_in=0, placement_y_in=-.3, placement_heading_deg=.5, moved_back_from=["the left Toggle"]))

    def test_the_path_keeps_a_point_every_tenth_of_a_second_and_the_last(self):
        trace = [dict(t=t / 100, true_x=t, true_y=0) for t in range(25)]
        self.assertEqual([point[0] for point in thinned_path(trace)], [0, 10, 20, 24])


class ReportTests(unittest.TestCase):
    def test_worst_is_a_broken_rule_then_the_lowest_score_then_misses(self):
        runs = {1: outcome(red=20), 2: outcome(red=10), 3: outcome(red=20, legal=False), 4: outcome(red=10, on_target=0),
                5: outcome(red=10, end_error_in=4.0)}
        self.assertEqual(sorted(runs, key=lambda number: badness(runs[number])), [3, 4, 5, 2, 1])

    def test_end_headings_spread_across_south(self):
        poses = [dict(end_pose=[0, 0, 179]), dict(end_pose=[2, 0, -179])]
        value = end_spread(poses)
        self.assertEqual(value["middle"], [1, 0])
        self.assertAlmostEqual(value["furthest_in"], 1)
        self.assertAlmostEqual(abs(value["heading_deg"]), 180)
        self.assertAlmostEqual(value["heading_high_deg"] - value["heading_low_deg"], 2)

    def test_steps_and_scores_are_counted_across_the_runs(self):
        runs = [dict(legal=True, simulated_s=10 + n, end_pose=[n, 0, 0], end_error_in=n,
                     autonomous=dict(red=red, blue=0, win_point=red > 10, bonus=True), final_score=None,
                     steps=[dict(index=0, ran=True, finished=True, on_target=n < 2, broke_rule=False,
                                 position_error_in=n, heading_error_deg=None), dict(index=1, ran=False)])
                for n, red in enumerate((20, 20, 10))]
        report = batch_report(runs, 2, [3, 1, 2])
        self.assertEqual(report["autonomous_red"], [dict(value=20, runs=2), dict(value=10, runs=1)])
        self.assertEqual(report["win_point"], 2)
        self.assertEqual(report["steps"][0]["on_target"], 2)
        self.assertEqual(report["steps"][0]["position_error_in"], dict(low=0, median=1, high=2))
        self.assertEqual(report["steps"][1]["ran"], 0)
        self.assertIsNone(report["final_red"])
        self.assertEqual(report["worst_first"], [3, 1, 2])
        self.assertIsNone(batch_report([], 2, []))


if __name__ == "__main__":
    unittest.main()
