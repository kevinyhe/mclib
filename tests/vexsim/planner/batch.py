"""Run a routine many times: the start and conditions of each run, what
each run did in a few numbers, and how those numbers spread across the runs."""
import copy
import math
import random
import statistics

from engine import HEADING_GOALS
from mechanisms import MOVE_RESULTS
from project import CONDITIONS, REPEAT, STILL_STEPS, _number, _object

REACHED_PRESET = MOVE_RESULTS[1]
PATH_SAMPLE_S = .1


def repeat_settings(raw):
    """A checked copy of the run-many settings, with defaults filled in."""
    settings = {key: info["default"] for key, info in REPEAT.items()}
    settings.update(_object(raw, REPEAT, "Run many"))
    for key, info in REPEAT.items():
        settings[key] = _number(settings[key], info["low"], info["high"], f"Run many: {info['label']}", info["unit"],
                                integer=key == "count")
    return settings


def variations(project, settings, seed):
    """Each run's conditions: the routine's own, with the robot placed off the
    start pose and the grip, battery and sensor noise changed, all at random
    within @p settings. The same seed gives the same runs."""
    rng = random.Random(seed)
    base = project["conditions"]

    def condition(key, value):
        return round(min(CONDITIONS[key]["high"], max(CONDITIONS[key]["low"], value)), 3)

    def either_way(limit):
        return rng.uniform(-1, 1) * limit

    runs = []
    for _ in range(settings["count"]):
        runs.append(dict(
            base,
            placement_x_in=condition("placement_x_in", base["placement_x_in"] + either_way(settings["placement_in"])),
            placement_y_in=condition("placement_y_in", base["placement_y_in"] + either_way(settings["placement_in"])),
            placement_heading_deg=condition("placement_heading_deg",
                                            base["placement_heading_deg"] + either_way(settings["placement_deg"])),
            grip_percent=condition("grip_percent", base["grip_percent"] * (1 + either_way(settings["grip_percent"]) / 100)),
            battery_percent=condition("battery_percent",
                                      base["battery_percent"] - rng.uniform(0, settings["battery_percent"])),
            noise_seed=rng.randrange(1, CONDITIONS["noise_seed"]["high"])))
    return runs


def varied_project(project, conditions):
    varied = copy.deepcopy(project)
    varied["conditions"] = conditions
    return varied


def step_finished(row):
    """Whether a step that ran did what it set out to before its time limit:
    a motion ended on its own, the lift or roller reached its preset, the
    wall reset found the wall, the Drive Team could load."""
    kind = row["type"]
    if row["rule_breaks"] or row["configuration_error"]:
        return False
    if kind == "wall_reset":
        return row["coordinate_reset"]
    if kind in ("lift", "roller"):
        return row["move_result"] == REACHED_PRESET
    if kind in STILL_STEPS:
        return True
    return row["duration_s"] * 1000 < row["timeout_ms"] - .5


def step_outcome(row, step, settings):
    if not row["executed"]:
        return dict(index=row["index"], ran=False)
    finished = step_finished(row)
    position, heading = row.get("position_error_in"), row.get("heading_error_deg")
    if heading is not None and step["chain"] and row["type"] not in HEADING_GOALS:
        heading = None
    on_target = (finished and (position is None or position <= settings["on_target_in"]) and
                 (heading is None or heading <= settings["on_target_deg"]))
    return dict(index=row["index"], ran=True, broke_rule=bool(row["rule_breaks"]), finished=finished,
                on_target=on_target, position_error_in=position, heading_error_deg=heading)


def thinned_path(trace):
    """The simulated robot's path: its first point in each PATH_SAMPLE_S, and its last."""
    points, last_slot = [], None
    for frame in trace:
        slot = math.floor(frame["t"] / PATH_SAMPLE_S + 1e-6)
        if slot != last_slot:
            points.append([round(frame["true_x"], 2), round(frame["true_y"], 2)])
            last_slot = slot
    end = [round(trace[-1]["true_x"], 2), round(trace[-1]["true_y"], 2)]
    return points if points[-1] == end else points + [end]


def run_outcome(result, settings):
    """What one run did, in the few numbers the report compares."""
    project = result["project"]
    steps = [step_outcome(row, project["steps"][row["index"]], settings) for row in result["steps"]]
    last = result["trace"][-1]
    autonomous = result["autonomous"]
    final = result["score"]
    return dict(
        legal=result["legal"], simulated_s=result["summary"]["simulated_s"], steps=steps,
        end_pose=[last["true_x"], last["true_y"], last["true_heading"]],
        end_error_in=final_step_error(result),
        autonomous=None if autonomous is None else dict(
            red=autonomous["red"], blue=autonomous["blue"], win_point=autonomous["win_point"]["earned"],
            bonus=autonomous["bonus"]["red"] > 0),
        final_score=None if final is None else dict(red=final["red"]["total"], blue=final["blue"]["total"]),
        placement=result["placement"], path=thinned_path(result["trace"]))


def final_step_error(result):
    """How far the last step that ran and has a target position ended from it."""
    for row in reversed(result["steps"]):
        if row["executed"] and row.get("position_error_in") is not None:
            return row["position_error_in"]
    return None


def badness(outcome):
    """Sorts the worst run first: a broken rule, then the lowest red score at
    the end of the autonomous period, then the lowest final red score, then
    the fewest steps on target, then the furthest last miss."""
    auton = outcome["autonomous"]
    final = outcome["final_score"]
    on_target = sum(step.get("on_target", False) for step in outcome["steps"])
    return (outcome["legal"], auton["red"] if auton else 0, final["red"] if final else 0, on_target,
            -(outcome["end_error_in"] or 0))


def spread(values):
    if not values:
        return None
    return dict(low=min(values), median=statistics.median(values), high=max(values))


def counts(values):
    """How many runs had each value, most common first."""
    tally = {}
    for value in values:
        tally[value] = tally.get(value, 0) + 1
    return [dict(value=value, runs=runs) for value, runs in sorted(tally.items(), key=lambda item: (-item[1], item[0]))]


def step_report(index, outcomes):
    ran = [outcome["steps"][index] for outcome in outcomes if outcome["steps"][index]["ran"]]
    return dict(index=index, runs=len(outcomes), ran=len(ran),
                finished=sum(step["finished"] for step in ran), on_target=sum(step["on_target"] for step in ran),
                broke_rule=sum(step["broke_rule"] for step in ran),
                position_error_in=spread([step["position_error_in"] for step in ran
                                          if step["position_error_in"] is not None]),
                heading_error_deg=spread([step["heading_error_deg"] for step in ran
                                          if step["heading_error_deg"] is not None]))


def end_spread(outcomes):
    """Where the runs ended: their middle, how far the furthest ended from it,
    and the range of headings."""
    poses = [outcome["end_pose"] for outcome in outcomes]
    middle = (statistics.fmean(pose[0] for pose in poses), statistics.fmean(pose[1] for pose in poses))
    headings = [pose[2] for pose in poses]
    middle_heading = math.degrees(math.atan2(sum(math.sin(math.radians(h)) for h in headings),
                                             sum(math.cos(math.radians(h)) for h in headings)))
    turned = [math.remainder(heading - middle_heading, 360) for heading in headings]
    return dict(middle=[round(middle[0], 2), round(middle[1], 2)],
                furthest_in=max(math.dist(pose[:2], middle) for pose in poses),
                heading_deg=middle_heading, heading_low_deg=middle_heading + min(turned),
                heading_high_deg=middle_heading + max(turned))


def batch_report(outcomes, step_count, worst_first):
    """How the finished runs spread. @p worst_first lists every finished run's
    number, worst first."""
    if not outcomes:
        return None
    autonomous = [outcome["autonomous"] for outcome in outcomes if outcome["autonomous"]]
    finals = [outcome["final_score"] for outcome in outcomes if outcome["final_score"]]
    return dict(
        runs=len(outcomes), legal=sum(outcome["legal"] for outcome in outcomes),
        steps=[step_report(index, outcomes) for index in range(step_count)],
        autonomous_red=counts([item["red"] for item in autonomous]) if autonomous else None,
        win_point=sum(item["win_point"] for item in autonomous) if autonomous else None,
        bonus=sum(item["bonus"] for item in autonomous) if autonomous else None,
        final_red=counts([item["red"] for item in finals]) if finals else None,
        simulated_s=spread([outcome["simulated_s"] for outcome in outcomes]),
        end=end_spread(outcomes),
        end_error_in=spread([outcome["end_error_in"] for outcome in outcomes if outcome["end_error_in"] is not None]),
        worst_first=worst_first)
