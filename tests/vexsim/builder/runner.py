#!/usr/bin/env python3
"""Validated motion sequences, executed by mclib C++ in an isolated process."""
import argparse
import contextlib
import copy
import csv
import fcntl
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import run as harness

PRESETS = [dict(id="four_motor_200", label="4 motors · 200 RPM", track_width_in=11.5),
           dict(id="six_motor_450", label="6 motors · 450 RPM", track_width_in=11.5),
           dict(id="speed_base", label="Speed base · 4 inch wheels", track_width_in=11.0)]
TUNING = {f"{loop}_{term}": dict(default=value, min=0, max=100,
                               label=f"{loop.title()} {term.upper()}")
          for loop, values in (("drive", (.4, 0, 3)), ("heading", (.3, 0, 1.5)),
                               ("turn", (.3, 0, 1.5)))
          for term, value in zip(("kp", "ki", "kd"), values)}
STEP_FIELDS = {
    "drive": {"distance"}, "turn": {"heading"},
    "point": {"x", "y", "direction"}, "turn_to_point": {"x", "y", "direction"},
    "boomerang": {"x", "y", "heading", "direction", "lead"},
    "arc": {"heading", "radius"}, "reverse_arc": {"heading", "radius"},
    "swing": {"heading", "direction"},
    "wall_reset": {"x", "y", "heading", "current_ma"},
    # followTrajectory() (RAMSETE) along a spline from the current pose,
    # through each of `waypoints` ([{"x", "y"}, ...], up to 8), to (x, y).
    "follow": {"waypoints", "x", "y", "direction", "b"},
}
DEFAULTS = dict(name="Boomerang / tracking wheels", preset="six_motor_450", tracking_mode="two",
                start_pose=dict(x=0, y=0, heading=0),
                environment=dict(soc=1, friction=1, encoder_heading=False, seed=1,
                                 locked=False, constrained=False),
                tuning={key: item["default"] for key, item in TUNING.items()},
                stop_on_failure=True,
                steps=[dict(type="boomerang", x=24, y=24, heading=90, direction=1,
                            lead=.5, timeout_ms=4000, volts=12)])
LIMITS = dict(max_steps=16, max_simulation_ms=120000, max_timeout_ms=20000,
              hold_ms=300, max_body_bytes=65536, queue_size=4, history_size=40)
ASSUMPTIONS = [
    "Actual mclib C++ controls vexsim physics; Python does not implement a motion controller.",
    "Coordinates are inches: +X right, +Y forward, clockwise heading in degrees.",
    "Each step requests a full stop; target checks include a separate 300 ms hold.",
    "Drive targets use the entry odometry position and the last commanded heading; arc targets are ideal geometry.",
    "Drive encoder odometry cannot observe lateral slip. Two tracking wheels are modeled sensors, not perfect ground truth.",
    "Wall reset uses idealized locked motors or a constrained body, not field collision geometry.",
    "Acceptance is a simulator check, not hardware calibration or competition readiness.",
]


def config():
    return dict(defaults=copy.deepcopy(DEFAULTS), presets=PRESETS,
                step_types=[dict(id=key, label=key.replace("_", " ").title())
                            for key in STEP_FIELDS], tuning=TUNING, limits=LIMITS,
                assumptions=ASSUMPTIONS)


def strict_json(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    def invalid(value):
        raise ValueError(f"non-finite JSON value: {value}")
    return json.loads(text, parse_constant=invalid, object_pairs_hook=pairs)


def obj(value, allowed, location):
    if not isinstance(value, dict):
        raise ValueError(f"{location} must be an object")
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f"{location}: unsupported fields {', '.join(sorted(unknown))}")
    return value


def number(value, low, high, location, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{location} must be a finite number")
    if not low <= value <= high or not math.isfinite(value):
        raise ValueError(f"{location} must be between {low} and {high}")
    if integer and int(value) != value:
        raise ValueError(f"{location} must be an integer")
    return int(value) if integer else value


def boolean(value, location):
    if not isinstance(value, bool):
        raise ValueError(f"{location} must be true or false")
    return value


def validate_spec(raw):
    obj(raw, DEFAULTS, "scenario")
    spec = copy.deepcopy(DEFAULTS)
    for key in ("name", "preset", "tracking_mode", "stop_on_failure"):
        if key in raw:
            spec[key] = raw[key]
    if not isinstance(spec["name"], str) or not 1 <= len(spec["name"]) <= 80:
        raise ValueError("name must contain 1 to 80 characters")
    if any(ord(char) < 32 for char in spec["name"]):
        raise ValueError("name cannot contain control characters")
    if spec["preset"] not in [p["id"] for p in PRESETS]:
        raise ValueError("unknown drivetrain preset")
    if spec["tracking_mode"] not in ("drive", "two"):
        raise ValueError("tracking_mode must be drive or two")
    boolean(spec["stop_on_failure"], "stop_on_failure")
    pose = obj(raw.get("start_pose", {}), spec["start_pose"], "start_pose")
    spec["start_pose"].update(pose)
    for key, value in spec["start_pose"].items():
        number(value, -3600 if key == "heading" else -500,
               3600 if key == "heading" else 500, f"start_pose.{key}")
    environment = obj(raw.get("environment", {}), spec["environment"], "environment")
    spec["environment"].update(environment)
    for key in ("encoder_heading", "locked", "constrained"):
        boolean(spec["environment"][key], f"environment.{key}")
    for key, low, high in (("soc", 0, 1), ("friction", .05, 2), ("seed", 0, 2147483647)):
        spec["environment"][key] = number(spec["environment"][key], low, high,
                                            f"environment.{key}", key == "seed")
    tuning = obj(raw.get("tuning", {}), TUNING, "tuning")
    spec["tuning"].update(tuning)
    for key, value in spec["tuning"].items():
        number(value, TUNING[key]["min"], TUNING[key]["max"], f"tuning.{key}")
    steps = raw.get("steps")
    if not isinstance(steps, list) or not 1 <= len(steps) <= LIMITS["max_steps"]:
        raise ValueError("steps must contain 1 to 16 motions")
    spec["steps"] = []
    for index, value in enumerate(steps):
        location = f"steps[{index}]"
        if not isinstance(value, dict) or not isinstance(value.get("type"), str):
            raise ValueError(f"{location}.type is required")
        kind = value["type"]
        if kind not in STEP_FIELDS:
            raise ValueError(f"{location}: unknown motion type")
        if kind == "follow" and ("via_x" in value or "via_y" in value):
            # Files saved before waypoints lists had exactly one via point.
            value = dict(value)
            value["waypoints"] = [dict(x=value.pop("via_x", 0), y=value.pop("via_y", 0))]
        obj(value, STEP_FIELDS[kind] | {"type", "timeout_ms", "volts"}, location)
        step = dict(type=kind, timeout_ms=4000, volts=6 if kind == "wall_reset" else 12)
        step.update(value)
        if "direction" in STEP_FIELDS[kind]:
            step.setdefault("direction", 1)
        if kind == "boomerang":
            step.setdefault("lead", .5)
        if kind == "follow":
            step.setdefault("b", 50)
            step.setdefault("waypoints", [])
            if not isinstance(step["waypoints"], list) or len(step["waypoints"]) > 8:
                raise ValueError(f"{location}.waypoints must be a list of up to 8 points")
            points = []
            for n, point in enumerate(step["waypoints"]):
                where = f"{location}.waypoints[{n}]"
                obj(point, {"x", "y"}, where)
                if "x" not in point or "y" not in point:
                    raise ValueError(f"{where} needs x and y")
                points.append(dict(x=number(point["x"], -500, 500, f"{where}.x"),
                                   y=number(point["y"], -500, 500, f"{where}.y")))
            step["waypoints"] = points
        if kind == "wall_reset":
            if index != len(steps) - 1:
                raise ValueError("wall_reset must be last: it changes the odometry reference frame")
            step.setdefault("current_ma", 2500)
        for key in STEP_FIELDS[kind]:
            if key not in step:
                raise ValueError(f"{location}.{key} is required")
        for key in ("x", "y", "distance"):
            if key in step:
                number(step[key], -500, 500, f"{location}.{key}")
        if "heading" in step:
            number(step["heading"], -3600, 3600, f"{location}.heading")
        if "radius" in step:
            number(step["radius"], -250, 250, f"{location}.radius")
            if abs(step["radius"]) < 1:
                raise ValueError(f"{location}.radius magnitude must be at least 1 inch")
        if "direction" in step:
            number(step["direction"], -1, 1, f"{location}.direction", True)
            if step["direction"] not in (-1, 1):
                raise ValueError(f"{location}.direction must be -1 or 1")
        if "lead" in step:
            number(step["lead"], 0, 1, f"{location}.lead")
            if step["lead"] == 1:
                raise ValueError(f"{location}.lead must be less than 1")
        if "current_ma" in step:
            number(step["current_ma"], 0, 10000, f"{location}.current_ma")
        if "b" in step:
            number(step["b"], 0.1, 500, f"{location}.b")
        step["timeout_ms"] = number(step["timeout_ms"], 50, 20000,
                                     f"{location}.timeout_ms", True)
        number(step["volts"], .1, 12, f"{location}.volts")
        if kind == "turn_to_point" and step["volts"] != 12:
            raise ValueError("turn_to_point uses the library's fixed 12 V cap")
        spec["steps"].append(step)
    if sum(s["timeout_ms"] + LIMITS["hold_ms"] for s in spec["steps"]) > 120000:
        raise ValueError("total step deadlines plus holds must not exceed 120 seconds")
    return spec


def write_json(path, value):
    """Atomic writes let the server poll progress without reading partial JSON."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


class LivePublisher:
    """Wall-clock throttling only; the observer never changes controller timing."""
    def __init__(self, destination, snapshot, interval=.1):
        self.path = Path(destination) / "partial.json"
        self.snapshot = snapshot
        self.interval = interval
        self.last_write = -math.inf

    def publish(self, frame=None, *, force=False):
        if force or time.monotonic() - self.last_write >= self.interval:
            write_json(self.path, self.snapshot())
            self.last_write = time.monotonic()


def source_fingerprint(vexsim):
    digest = hashlib.sha256()
    files = [ROOT / "Makefile", ROOT / "common.mk"]
    files += sorted(path for path in (ROOT / "tests").rglob("*")
                    if path.is_file() and path.suffix in (".py", ".cpp", ".hpp", ".h", ".mjs", ".js", ".css", ".html"))
    files += sorted((ROOT / "src/mclib").rglob("*.cpp"))
    files += sorted((ROOT / "include/mclib").rglob("*.hpp"))
    files += sorted((Path(vexsim) / "vexsim").rglob("*.py"))
    files += sorted((Path(vexsim) / "vexsim/web").glob("*.html"))
    files += [path for path in (Path(vexsim) / "package.json", Path(vexsim) / "package-lock.json")
              if path.is_file()]
    for path in files:
        digest.update(str(path).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def motion_target(step, entry_pose, commanded_heading):
    kind = step["type"]
    target = dict(x=None, y=None, heading=None)
    if kind == "drive":
        radians = math.radians(commanded_heading)
        target.update(x=entry_pose[0] + step["distance"] * math.sin(radians),
                      y=entry_pose[1] + step["distance"] * math.cos(radians),
                      heading=commanded_heading)
    elif kind in ("point", "boomerang", "wall_reset", "follow"):
        target.update(x=step["x"], y=step["y"], heading=step.get("heading"))
    elif kind == "turn_to_point":
        angle = math.degrees(math.atan2(step["x"] - entry_pose[0], step["y"] - entry_pose[1]))
        target["heading"] = angle + (180 if step["direction"] == -1 else 0)
    elif kind in ("arc", "reverse_arc"):
        initial, final = math.radians(entry_pose[2]), math.radians(step["heading"])
        target.update(x=entry_pose[0] + step["radius"] * (math.cos(initial) - math.cos(final)),
                      y=entry_pose[1] + step["radius"] * (math.sin(final) - math.sin(initial)),
                      heading=step["heading"])
    else:
        target["heading"] = step["heading"]
    return target


def invoke(bridge, step, feedforward=None):
    if step["type"] == "follow":
        # Heading is not checked: it is wherever the spline ends up pointing.
        bridge.set_ramsete(feedforward, b=step["b"])
        return bridge.follow([(p["x"], p["y"]) for p in step["waypoints"]] + [(step["x"], step["y"])],
                             reversed=step["direction"] == -1,
                             timeout=step["timeout_ms"], volts=step["volts"])
    actions = dict(turn=0, drive=1, arc=2, reverse_arc=3, swing=4,
                   turn_to_point=5, point=6, boomerang=7, wall_reset=8)
    kind = step["type"]
    if kind == "reverse_arc" and math.remainder(step["heading"] - bridge.pose()[2], 360) * step["radius"] > 1e-8:
        raise ValueError("Reverse arc requires radius × live heading change <= 0; change the radius sign or target heading")
    a = step.get("distance", step.get("x", step.get("heading", 0)))
    b = step.get("y", step.get("radius", 0))
    kwargs = dict(a=a, b=b, heading=step.get("heading", 0),
                  timeout=step["timeout_ms"], direction=step.get("direction", 1),
                  stop=True, volts=step["volts"], current=step.get("current_ma", 2500))
    if kind == "boomerang":
        kwargs["lead"] = step["lead"]
    return bridge.run(actions[kind], **kwargs)


def load_library(vexsim, cache, fallback_dir):
    """Build (or reuse) the simulator library for the current sources."""
    sys.path.insert(0, str(Path(vexsim).resolve()))
    fingerprint = source_fingerprint(vexsim)
    build_dir = Path(cache) / fingerprint if cache else Path(fallback_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    library = build_dir / "libmclib_vexsim.so"
    marker = build_dir / "complete.json"
    with (build_dir / "build.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if marker.is_file() and library.is_file():
            lib = harness.load_library(library)
        else:
            lib = harness.build(build_dir)
            if source_fingerprint(vexsim) != fingerprint:
                raise RuntimeError("Source changed during compilation; submit the run again")
            write_json(marker, dict(source_fingerprint=fingerprint))
    return lib, fingerprint, build_dir


def cached_feedforward(lib, build_dir, preset, tracking_mode):
    """measure_feedforward(), kept next to the library it was measured with,
    so only the first plan or run for a preset waits for it."""
    path = Path(build_dir) / f"feedforward-{preset}-{tracking_mode}.json"
    try:
        return strict_json(path.read_text())
    except (OSError, ValueError):
        value = harness.measure_feedforward(lib, preset, tracking_mode, {})
        write_json(path, value)
        return value


def plan(request, vexsim, cache=None):
    """Plan every follow path in @p request without driving:
    {"preset", "tracking_mode", "paths": [{"start": {"x", "y"}, "points": [[x, y], ...],
    "direction", "b"}]} -> feedforward, the limits used, and each plan's states."""
    if not isinstance(request, dict):
        raise ValueError("plan request must be an object")
    preset = request.get("preset")
    if preset not in {item["id"] for item in PRESETS}:
        raise ValueError("unknown preset")
    tracking_mode = request.get("tracking_mode", "two")
    if tracking_mode not in ("drive", "two"):
        raise ValueError("tracking_mode must be drive or two")
    paths = request.get("paths")
    if not isinstance(paths, list) or len(paths) > LIMITS["max_steps"]:
        raise ValueError("paths must be a list of up to 16 plans")
    scratch = Path(cache or tempfile.mkdtemp(prefix="mclib-plan-"))
    lib, _, build_dir = load_library(vexsim, cache, scratch / "build")
    feedforward = cached_feedforward(lib, build_dir, preset, tracking_mode)
    bridge = harness.PhysicsBridge(lib, preset, tracking_mode=tracking_mode)
    plans = []
    for index, item in enumerate(paths):
        where = f"paths[{index}]"
        start = item.get("start", {})
        sx = number(start.get("x", 0), -500, 500, f"{where}.start.x")
        sy = number(start.get("y", 0), -500, 500, f"{where}.start.y")
        points = item.get("points", [])
        if not isinstance(points, list) or not 1 <= len(points) <= 9:
            raise ValueError(f"{where}.points must hold 1 to 9 points")
        points = [(number(p[0], -500, 500, where), number(p[1], -500, 500, where)) for p in points]
        bridge.set_ramsete(feedforward, b=number(item.get("b", 50), .1, 500, f"{where}.b"))
        states = bridge.plan((sx, sy), points, reversed=item.get("direction", 1) == -1)
        if states is None:
            plans.append(dict(valid=False, reason="Two points in a row are the same"))
            continue
        length = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(states, states[1:]))
        plans.append(dict(valid=True, duration_s=states[-1][0], length_in=length,
                          top_ips=max(s[3] for s in states),
                          states=[[round(v, 4) for v in s] for s in states]))
    cruise = 0.7 * feedforward["top_ips"]
    return dict(feedforward=feedforward, b=50, zeta=0.7,
                limits=dict(max_velocity_ips=cruise, max_acceleration_ips2=60.0,
                            max_lateral_acceleration_ips2=60.0,
                            track_width_in=feedforward["track_in"]),
                plans=plans)


def execute(spec, destination, vexsim, cache=None):
    spec = validate_spec(spec)
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    write_json(destination / "spec.json", spec)
    progress = dict(phase="build", step=0, total=len(spec["steps"]))
    write_json(destination / "progress.json", progress)
    lib, fingerprint, build_dir = load_library(vexsim, cache, destination / "build")
    shutil.copyfile(build_dir / "build.log", destination / "build.log")
    # followTrajectory() needs feedforward. Measure it the way the docs tell a
    # user to, on a healthy robot, before the run: measuring re-initialises
    # the library, so it can't happen once the sequence has started.
    feedforward = (cached_feedforward(lib, build_dir, spec["preset"], spec["tracking_mode"])
                   if any(step["type"] == "follow" for step in spec["steps"]) else None)
    bridge = harness.PhysicsBridge(lib, spec["preset"], **spec["environment"],
                                   tracking_mode=spec["tracking_mode"], tuning=spec["tuning"],
                                   start_pose=tuple(spec["start_pose"][k] for k in ("x", "y", "heading")),
                                   sim_time_limit=120)
    rows, trace = [], []
    halted = False
    for index, step in enumerate(spec["steps"]):
        progress.update(phase="run", step=index)
        write_json(destination / "progress.json", progress)
        if halted:
            rows.append(dict(index=index, type=step["type"], executed=False, passed=False,
                             reason="Skipped after previous step failed", checks=[], target=None,
                             position_error_in=None, heading_error_deg=None, duration_ms=0,
                             start_time_ms=round(bridge.sim.t * 1000), end_time_ms=round(bridge.sim.t * 1000)))
            continue
        target = motion_target(step, bridge.pose(), bridge.commanded_heading())
        started, trace_start = bridge.sim.t, len(bridge.control_trace)
        initial = dict(bridge.capture(), phase="motion", step=index)
        trace.append(initial)
        active = dict(index=index, type=step["type"], status="running", executed=True,
                      passed=None, target=target, spec=step, checks=[],
                      start_time_ms=round(started * 1000))
        segment_start, phase, reset_frame = trace_start, "motion", False

        def snapshot():
            pending = [dict(frame, phase=phase, step=index)
                       for frame in bridge.control_trace[segment_start:]]
            if reset_frame:
                for frame in pending:
                    frame.update(odometry_error_in=None, coordinate_reset=True)
            return dict(steps=[*rows, active], trace=[*trace, *pending], live=True,
                        active_step=index, phase=phase)

        publisher = LivePublisher(destination, snapshot)
        bridge.on_capture = publisher.publish
        publisher.publish(force=True)
        configuration_error = None
        try:
            wall_result = invoke(bridge, step, feedforward)
        except ValueError as error:
            configuration_error = str(error)
            wall_result = 0
        returned = bridge.sim.t
        truth_at_exit, estimate_at_exit = bridge.truth(), bridge.pose()
        trace.extend(dict(frame, phase="motion", step=index)
                     for frame in bridge.control_trace[trace_start:])
        reset_frame = step["type"] == "wall_reset" and bool(wall_result)
        exit_frame = dict(bridge.capture(), phase="motion", step=index)
        if reset_frame:
            exit_frame.update(odometry_error_in=None, coordinate_reset=True)
        trace.append(exit_frame)
        hold_start = len(bridge.control_trace)
        segment_start, phase = hold_start, "hold"
        active.update(return_time_ms=round(returned * 1000))
        progress.update(phase="hold")
        write_json(destination / "progress.json", progress)
        publisher.publish(force=True)
        bridge.advance_idle(LIMITS["hold_ms"])
        bridge.on_capture = None
        hold_frames = [dict(frame, phase="hold", step=index)
                       for frame in bridge.control_trace[hold_start:]]
        hold_frames.append(dict(bridge.capture(), phase="hold", step=index))
        if reset_frame:
            for frame in hold_frames:
                frame.update(odometry_error_in=None, coordinate_reset=True)
        trace.extend(hold_frames)
        actual = bridge.truth()
        duration = (returned - started) * 1000
        position_error = (math.hypot(actual[0] - target["x"], actual[1] - target["y"])
                          if target["x"] is not None else None)
        heading_error = (abs(math.remainder(actual[2] - target["heading"], 360))
                         if target["heading"] is not None else None)
        tolerance = 1.5 if step["type"] == "boomerang" else 5.5 if step["type"] in ("arc", "reverse_arc") else 2.5
        checks = [dict(name="drive_stopped", passed=bridge.stopped(), actual=bridge.stopped(), expected=True)]
        if configuration_error:
            checks.append(dict(name="configuration", passed=False, actual=configuration_error,
                               expected="Compatible motion parameters"))
        if step["type"] == "wall_reset":
            # A successful reset changes the reference frame, not the physical body.
            # Check the requested odometry reset at return; do not compare that
            # reference coordinate with the unchanged physical position.
            reset_error = max(abs(estimate_at_exit[0] - step["x"]),
                              abs(estimate_at_exit[1] - step["y"]),
                              abs(math.remainder(estimate_at_exit[2] - step["heading"], 360)))
            checks += [dict(name="wall_contact_detected", passed=bool(wall_result), actual=bool(wall_result), expected=True),
                       dict(name="pose_reset", passed=bool(wall_result) and reset_error < 1e-6,
                            actual=reset_error, expected="< 0.000001")]
            position_error = heading_error = None
        else:
            checks.append(dict(name="deadline", passed=duration < step["timeout_ms"] - .5,
                               actual=round(duration, 3), expected=f"< {step['timeout_ms']} ms"))
            if position_error is not None:
                checks.append(dict(name="position", passed=position_error <= tolerance,
                                   actual=position_error, expected=f"<= {tolerance} in"))
            if heading_error is not None:
                checks.append(dict(name="heading", passed=heading_error <= 5,
                                   actual=heading_error, expected="<= 5 deg"))
        passed = all(check["passed"] for check in checks)
        reason = "All acceptance checks passed" if passed else "; ".join(
            check["name"].replace("_", " ") + " failed" for check in checks if not check["passed"])
        coordinate_reset = step["type"] == "wall_reset" and bool(wall_result)
        odometry_error = None if coordinate_reset else math.dist(estimate_at_exit[:2], truth_at_exit[:2])
        diagnosis = []
        if configuration_error:
            diagnosis.append(configuration_error + ". This step was rejected before voltage was applied.")
        if coordinate_reset:
            diagnosis.append("Wall reset changed the odometry coordinate frame at return. The body did not teleport; truth-to-odometry distance is not comparable after this reset.")
        if odometry_error is not None and odometry_error > 2.5:
            diagnosis.append("Odometry and physical position diverged before return; inspect slip and tracker configuration before changing gains.")
        if duration >= step["timeout_ms"] - .5:
            diagnosis.append("The motion reached its deadline; compare heading/output traces for oscillation or insufficient output.")
        if position_error is not None and position_error > tolerance and odometry_error <= 2.5:
            diagnosis.append("Position missed its simulator tolerance with smaller odometry error; inspect control convergence and stopping drift.")
        row = dict(index=index, type=step["type"], executed=True, passed=passed, reason=reason,
                   target=target, checks=checks, position_error_in=position_error, heading_error_deg=heading_error,
                   duration_ms=round(duration, 3), hold_ms=LIMITS["hold_ms"],
                   start_time_ms=round(started * 1000), return_time_ms=round(returned * 1000),
                   end_time_ms=round(bridge.sim.t * 1000), true_pose_at_exit=truth_at_exit,
                   estimated_pose_at_exit=estimate_at_exit, true_pose=actual,
                   odometry_error_at_exit_in=odometry_error, coordinate_reset=coordinate_reset, diagnosis=diagnosis)
        rows.append(row)
        halted = not passed and spec["stop_on_failure"]
        write_json(destination / "partial.json", dict(steps=rows, trace=trace, live=True,
                                                     active_step=index, phase="hold"))
    progress.update(phase="compare")
    write_json(destination / "progress.json", progress)
    if source_fingerprint(vexsim) != fingerprint:
        raise RuntimeError("Source changed while the simulation ran; rerun before interpreting this result")
    result = dict(passed=all(row["passed"] for row in rows), steps=rows, trace=trace,
                  summary=dict(passed_steps=sum(row["passed"] for row in rows),
                               failed_steps=sum(row["executed"] and not row["passed"] for row in rows),
                               skipped_steps=sum(not row["executed"] for row in rows),
                               elapsed_ms=round(bridge.sim.t * 1000), hold_ms=LIMITS["hold_ms"]),
                  source_fingerprint=fingerprint, library=str(build_dir / "libmclib_vexsim.so"),
                  assumptions=ASSUMPTIONS)
    write_json(destination / "result.json", result)
    write_json(destination / "partial.json", dict(steps=rows, trace=trace, live=False,
                                                 active_step=None, phase="complete"))
    if trace:
        fields = list(dict.fromkeys(key for frame in trace for key in frame))
        with (destination / "trace.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(trace)
    progress.update(phase="complete", step=len(spec["steps"]))
    write_json(destination / "progress.json", progress)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plan", type=Path,
                        help="plan the follow paths in this request and print JSON; no run")
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--cache", type=Path)
    args = parser.parse_args()
    if args.plan:
        try:
            # Standard output carries only the JSON answer; progress lines
            # from building and measuring go to stderr.
            with contextlib.redirect_stdout(sys.stderr):
                answer = plan(strict_json(args.plan.read_text()), args.vexsim, args.cache)
            print(json.dumps(answer))
            return 0
        except Exception as error:
            print(json.dumps(dict(error=str(error))))
            return 2
    if not args.spec or not args.output:
        parser.error("--spec and --output are required for a run")
    try:
        result = execute(strict_json(args.spec.read_text()), args.output, args.vexsim, args.cache)
        print(json.dumps(result["summary"]), flush=True)
        return 0 if result["passed"] else 1
    except Exception as error:
        args.output.mkdir(parents=True, exist_ok=True)
        write_json(args.output / "error.json", dict(error=str(error)))
        print(f"Run error: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
