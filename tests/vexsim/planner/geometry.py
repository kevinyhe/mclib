"""Where each step is meant to leave the robot, and the curve of each path.

geometry.js holds the same rules for drawing. test_geometry.py checks that
the two agree.

Coordinates are inches, +X right, +Y forward. Headings are degrees, 0 facing
+Y, clockwise positive.
"""
import math

from project import LIMITS, PRESETS, ProjectError, step_name

AUTO_HANDLE_FRACTION = 0.4


def wrap(angle):
    return (angle + 180) % 360 - 180


def bearing(a, b):
    return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1]))


def along(heading, length):
    radians = math.radians(heading)
    return length * math.sin(radians), length * math.cos(radians)


def path_layout(step, start):
    """The resolved curve of a path step that starts at pose @p start.

    Returns the body heading at every anchor (start first) and, for each
    section, its four Bezier points, its direction and its speed limit
    (None means the path default)."""
    anchors = [(start[0], start[1])] + [(p["x"], p["y"]) for p in step["points"]]
    sections = step["sections"]
    count = len(step["points"])
    travel = [180 if section["reverse"] else 0 for section in sections]
    headings = [start[2]]
    for i in range(1, count + 1):
        point = step["points"][i - 1]
        if point["heading"] is not None:
            headings.append(point["heading"])
            continue
        offset = travel[i - 1]
        smooth = i < count and travel[i] == offset and math.dist(anchors[i - 1], anchors[i + 1]) >= LIMITS["min_point_gap_in"]
        tangent = bearing(anchors[i - 1], anchors[i + 1]) if smooth else bearing(anchors[i - 1], anchors[i])
        headings.append(wrap(tangent - offset))
    handle_out = [step.get("start_handle")] + [p["handle_out"] for p in step["points"]]
    handle_in = [None] + [p["handle_in"] for p in step["points"]]
    curves = []
    for i in range(1, count + 1):
        a, b = anchors[i - 1], anchors[i]
        chord = math.dist(a, b)
        out_length = handle_out[i - 1] if handle_out[i - 1] is not None else AUTO_HANDLE_FRACTION * chord
        in_length = handle_in[i] if handle_in[i] is not None else AUTO_HANDLE_FRACTION * chord
        out_x, out_y = along(headings[i - 1] + travel[i - 1], out_length)
        in_x, in_y = along(headings[i] + travel[i - 1], in_length)
        curves.append(dict(start=a, start_handle=(a[0] + out_x, a[1] + out_y),
                           end_handle=(b[0] - in_x, b[1] - in_y), end=b,
                           reverse=sections[i - 1]["reverse"],
                           speed_percent=sections[i - 1]["speed_percent"],
                           handle_lengths=(out_length, in_length)))
    return dict(headings=headings, sections=curves)


def check_path_spacing(step, start, index):
    where = step_name(index, step)
    anchors = [(start[0], start[1])] + [(p["x"], p["y"]) for p in step["points"]]
    for i in range(1, len(anchors)):
        gap = math.dist(anchors[i - 1], anchors[i])
        if gap < LIMITS["min_point_gap_in"]:
            before = "where the robot starts this step" if i == 1 else f"point {i - 1}"
            raise ProjectError(f"{where}: point {i} is {gap:.2f} in from {before}. "
                               f"Points must be at least {LIMITS['min_point_gap_in']} in apart", index)


def arc_points(pose, end_heading, signed_radius, samples=40):
    delta = wrap(end_heading - pose[2])
    heading = math.radians(pose[2])
    cx = pose[0] + math.cos(heading) * signed_radius
    cy = pose[1] - math.sin(heading) * signed_radius
    points = []
    for part in range(1, samples + 1):
        angle = math.radians(pose[2] + delta * part / samples)
        points.append((cx - math.cos(angle) * signed_radius, cy + math.sin(angle) * signed_radius))
    return points


def step_end_pose(step, pose, track_width_in):
    """The pose @p step is meant to leave the robot in, starting from @p pose."""
    x, y, heading = pose
    kind = step["type"]
    if kind == "drive":
        dx, dy = along(heading, step["distance"])
        return (x + dx, y + dy, heading)
    if kind == "turn":
        return (x, y, step["heading"])
    if kind in ("lift", "claw", "toggle_arm"):
        return pose
    if kind == "turn_to_point":
        return (x, y, wrap(bearing((x, y), (step["x"], step["y"])) + (180 if step["direction"] == -1 else 0)))
    if kind == "point":
        facing = wrap(bearing((x, y), (step["x"], step["y"])) + (180 if step["direction"] == -1 else 0))
        return (step["x"], step["y"], facing)
    if kind in ("boomerang", "wall_reset"):
        return (step["x"], step["y"], step["heading"])
    if kind in ("arc", "reverse_arc", "swing"):
        if kind == "swing":
            delta = wrap(step["heading"] - heading)
            radius = math.copysign(1, delta) * step["direction"] * track_width_in / 2 if delta else 0
        else:
            radius = step["radius"]
        end = arc_points(pose, step["heading"], radius)[-1]
        return (end[0], end[1], step["heading"])
    if kind == "path":
        if not step["points"]:
            return pose
        last = step["points"][-1]
        layout = path_layout(step, pose)
        return (last["x"], last["y"], layout["headings"][-1])
    raise ValueError(f"unknown step type {kind}")


def designed_poses(project):
    """(start, end) pose of every step, chained from the project's start."""
    track = PRESETS[project["robot"]["preset"]]["track_width_in"]
    pose = (project["start"]["x"], project["start"]["y"], project["start"]["heading"])
    poses = []
    for index, step in enumerate(project["steps"]):
        if step["type"] == "path":
            check_path_spacing(step, pose, index)
        end = step_end_pose(step, pose, track)
        poses.append((pose, end))
        pose = end
    return poses


def section_values(layout, default_speed_percent, top_speed_ips):
    """The flat numbers path_bridge.cpp reads, 10 per section."""
    values = []
    for section in layout["sections"]:
        percent = section["speed_percent"] if section["speed_percent"] is not None else default_speed_percent
        values += [*section["start"], *section["start_handle"], *section["end_handle"], *section["end"],
                   percent / 100 * top_speed_ips, 1.0 if section["reverse"] else 0.0]
    return values
