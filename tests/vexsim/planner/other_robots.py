"""The partner's and opponents' robots a routine puts on the Override field:
from the project's planner inches and degrees to vexsim's OtherRobot, and
what they did back in planner coordinates."""

import math

from fields import to_planner


def vexsim_robots(robots):
    """The project's other robots as vexsim OtherRobots, in vexsim's field frame."""
    from vexsim.other_robots import OtherRobot
    from vexsim.units import INCH

    def field_point(x, y):
        return y * INCH, -x * INCH

    return [OtherRobot(robot["role"], *field_point(robot["x"], robot["y"]), -math.radians(robot["heading"]),
                       length=robot["length_in"] * INCH, width=robot["width_in"] * INCH,
                       height=robot["height_in"] * INCH, mass_lb=robot["mass_lb"], drive=robot["drive"],
                       traction=robot["traction"],
                       path=tuple(field_point(point["x"], point["y"]) for point in robot["path"]),
                       volts=robot["volts"], start_s=robot["start_ms"] / 1000)
            for robot in robots]


def planner_frames(others):
    """vexsim telemetry's other robots as {role: [x, y, heading]} in planner inches and degrees."""
    return {role: [*to_planner(pose["x"], pose["y"]), round(-math.degrees(pose["theta"]), 2)]
            for role, pose in others.items()}


def end_reports(world):
    """Where each other robot ended up and whether it drove its whole path."""
    from vexsim.units import INCH
    reports = []
    for report in world.others.reports():
        x, y = to_planner(report["x"], report["y"])
        reports.append(dict(role=report["role"], x=x, y=y, heading=round(-math.degrees(report["theta"]), 2),
                            moved_in=round(report["moved"] / INCH, 2),
                            from_end_in=round(report["from_end"] / INCH, 2),
                            finished=report["finished"], fallen=report["fallen"]))
    return reports


#: Moved less than this, a robot holding still was not pushed
PUSHED_IN = 0.1


def describe_end(report, drives_a_path):
    where = f"({report['x'] + 0.0:.0f}, {report['y'] + 0.0:.0f}) facing {report['heading'] + 0.0:.0f}°"
    if drives_a_path:
        done = ("finished its path" if report["finished"] else "had not finished its path") + \
               f", {report['from_end_in']:.1f} in from its end, and ended at {where}"
    elif report["moved_in"] < PUSHED_IN:
        done = f"held still at {where}"
    else:
        done = f"was pushed {report['moved_in']:.1f} in from where it started, and ended at {where}"
    return f"The {report['role']} robot {done}." + (" It tipped over." if report["fallen"] else "")
