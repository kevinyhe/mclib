#!/usr/bin/env python3
"""Compile mclib, measure feedforward, plan paths and run routines in vexsim.

server.py starts each job as its own process, so a crash in the C++ never takes
the server down. Every job reports its stages in progress.json: what it is
doing, why, and how long each stage took.

    engine.py prepare --preset six_motor_450 --tracking two --cache DIR --progress FILE
    engine.py plan --project FILE --cache DIR
    engine.py run --project FILE --output DIR --cache DIR
"""
import argparse
import contextlib
import csv
import ctypes as C
import fcntl
import hashlib
import json
import math
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import run as harness  # noqa: E402
from geometry import designed_poses, path_layout, section_values, wrap  # noqa: E402
from project import (LIMITS, PRESETS, STEP_TYPES, ProjectError, step_name, step_time_limit, strict_json,
                     validate_project)  # noqa: E402

PATH_BRIDGE = HERE / "path_bridge.cpp"
STATE_VALUES = 8
MAX_STATES = 40000
MOTION_RESULTS = ("Reached", "TimedOut", "Cancelled", "Disabled", "InvalidValue", "NoDrive")
ACTIONS = dict(turn=0, drive=1, arc=2, reverse_arc=3, swing=4, turn_to_point=5, point=6, boomerang=7,
               wall_reset=8)
POSITION_LIMIT_IN = dict(boomerang=1.5, arc=5.5, reverse_arc=5.5)
DEFAULT_POSITION_LIMIT_IN = 2.5
HEADING_LIMIT_DEG = 5.0
ODOMETRY_DRIFT_IN = 2.5


def write_json(path, value):
    """Atomic, so the server never reads half a file."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False) + "\n")
    temporary.replace(path)


class StageError(Exception):
    """A failure the user can act on, with the stage and step it happened in."""

    def __init__(self, message, step=None):
        super().__init__(message)
        self.step = step


class Progress:
    """The stages of one job, written to progress.json after every change."""

    def __init__(self, path, stages):
        self.path = Path(path) if path else None
        self.stages = [dict(id=key, label=label, status="waiting", detail="", seconds=None)
                       for key, label in stages]
        self.step = None
        self.started = {}
        self.write()

    def _stage(self, key):
        return next(stage for stage in self.stages if stage["id"] == key)

    def active(self):
        return next((stage["id"] for stage in self.stages if stage["status"] == "active"), None)

    def begin(self, key, detail=""):
        self._stage(key).update(status="active", detail=detail, started_at=time.time())
        self.started[key] = time.monotonic()
        self.write()

    def note(self, key, detail):
        self._stage(key)["detail"] = detail
        self.write()

    def finish(self, key, detail, status="done"):
        seconds = time.monotonic() - self.started.get(key, time.monotonic())
        self._stage(key).update(status=status, detail=detail, seconds=round(seconds, 2))
        self.write()

    def skip(self, key, detail):
        self._stage(key).update(status="skipped", detail=detail, seconds=0)
        self.write()

    def write(self):
        if self.path:
            write_json(self.path, dict(stages=self.stages, step=self.step))


def source_fingerprint(vexsim):
    """Changes whenever the compiled library or the physics could change."""
    files = [HERE.parent / "run.py", HERE.parent / "bridge.cpp", PATH_BRIDGE, HERE / "path_helpers.hpp"]
    files += sorted((ROOT / "src/mclib").rglob("*.cpp")) + sorted((ROOT / "include/mclib").rglob("*.hpp"))
    simulator = Path(vexsim).resolve() / "vexsim"
    files += sorted(simulator.rglob("*.py"))
    digest = hashlib.sha256()
    for path in files:
        name = path.relative_to(simulator.parent) if simulator in path.parents else path.relative_to(ROOT)
        digest.update(str(name).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def build_dir_for(vexsim, cache):
    fingerprint = source_fingerprint(vexsim)
    return fingerprint, Path(cache) / fingerprint[:24]


def feedforward_path(build_dir, preset, tracking):
    return Path(build_dir) / f"feedforward-{preset}-{tracking}.json"


def readiness(vexsim, cache, preset, tracking):
    """What is already done for planning, without loading any C++."""
    fingerprint, build_dir = build_dir_for(vexsim, cache)
    compiled = (build_dir / "complete.json").is_file()
    measured = feedforward_path(build_dir, preset, tracking)
    return dict(fingerprint=fingerprint, compiled=compiled,
                feedforward=strict_json(measured.read_text()) if compiled and measured.is_file() else None)


def declare_planner(lib):
    doubles = C.POINTER(C.c_double)
    lib.planner_plan.argtypes = [doubles, C.c_int, doubles, doubles, C.c_int]
    lib.planner_plan.restype = C.c_int
    lib.planner_follow.argtypes = [doubles, C.c_int, doubles, doubles, C.c_double, C.c_double]
    lib.planner_follow.restype = C.c_int
    lib.planner_folded_section.argtypes = [doubles, C.c_int]
    lib.planner_folded_section.restype = C.c_int
    return lib


def load_library(vexsim, cache, progress, compile_allowed=True):
    """The simulator library for the current sources. Compiles once per
    source fingerprint; later jobs reuse it."""
    sys.path.insert(0, str(Path(vexsim).resolve()))
    fingerprint, build_dir = build_dir_for(vexsim, cache)
    build_dir.mkdir(parents=True, exist_ok=True)
    library = build_dir / "libmclib_vexsim.so"
    marker = build_dir / "complete.json"
    if marker.is_file():
        progress.skip("compile", "Already compiled for these sources.")
        return declare_planner(harness.load_library(library)), fingerprint, build_dir
    if not compile_allowed:
        raise StageError("mclib is not compiled for the current sources yet")
    progress.begin("compile", "Compiling mclib's motion code with g++. This takes 1 to 2 minutes the first time; "
                              "later runs reuse it until the C++ changes.")
    with (build_dir / "build.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not marker.is_file():
            try:
                harness.build(build_dir, extra_sources=[PATH_BRIDGE])
            except RuntimeError as error:
                errors = [line for line in str(error).splitlines() if "error" in line][:8]
                raise StageError("g++ could not compile mclib. First errors:\n" + "\n".join(errors) +
                                 f"\nFull log: {build_dir / 'build.log'}")
            if source_fingerprint(vexsim) != fingerprint:
                raise StageError("The C++ sources changed while compiling. Run again.")
            write_json(marker, dict(source_fingerprint=fingerprint))
    progress.finish("compile", f"Compiled {library}.")
    return declare_planner(harness.load_library(library)), fingerprint, build_dir


def describe_feedforward(value):
    return (f"kS {value['ks']:.2f} V, kV {value['kv']:.4f} V per in/s, top speed {value['top_ips']:.1f} in/s, "
            f"effective track width {value['track_in']:.1f} in.")


def measure_feedforward(lib, build_dir, preset, tracking, progress):
    """measure_feedforward() from run.py, kept next to the library it was
    measured with, so only the first job for a robot waits for it."""
    path = feedforward_path(build_dir, preset, tracking)
    if path.is_file():
        value = strict_json(path.read_text())
        progress.skip("measure", "Measured earlier: " + describe_feedforward(value))
        return value
    label = PRESETS[preset]["label"]
    progress.begin("measure", f"Measuring kS, kV and track width for {label} in the simulator. "
                              "This takes a few seconds, once per robot.")
    with (Path(build_dir) / f"feedforward-{preset}-{tracking}.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not path.is_file():
            with contextlib.redirect_stdout(sys.stderr):
                write_json(path, harness.measure_feedforward(lib, preset, tracking, {}))
    value = strict_json(path.read_text())
    progress.finish("measure", describe_feedforward(value))
    return value


def doubles(values):
    return (C.c_double * max(1, len(values)))(*values)


def path_inputs(project, step, start, feedforward):
    layout = path_layout(step, start)
    settings = project["path_defaults"]
    values = section_values(layout, settings["speed_percent"], feedforward["top_ips"])
    limits = [settings["accel_ips2"], settings["cornering_ips2"], feedforward["track_in"]]
    return layout, values, limits


def plan_path(lib, project, index, start, feedforward):
    """mclib's plan for path step @p index starting at @p start."""
    step = project["steps"][index]
    layout, values, limits = path_inputs(project, step, start, feedforward)
    out = (C.c_double * (STATE_VALUES * MAX_STATES))()
    count = lib.planner_plan(doubles(values), len(layout["sections"]), doubles(limits), out, MAX_STATES)
    where = step_name(index, step)
    if count == -1:
        folded = lib.planner_folded_section(doubles(values), len(layout["sections"]))
        if folded < 0:
            raise StageError(f"{where}: mclib could not plan this path. Check the speed and acceleration "
                             "settings are above zero.", index)
        raise StageError(f"{where}: section {folded + 1} doubles back on itself, so it has no direction "
                         f"to plan along. {fold_hint(layout, folded)}", index)
    if count == -2:
        raise StageError(f"{where}: the plan has more than {MAX_STATES} samples. Split it into shorter paths.", index)
    states = [out[STATE_VALUES * i:STATE_VALUES * (i + 1)] for i in range(count)]
    curved = [abs(state[7]) for state in states if abs(state[7]) > 1e-6]
    tightest = 1 / max(curved) if curved else None
    top_speed = feedforward["top_ips"]
    sections = []
    for number, section in enumerate(layout["sections"]):
        percent = section["speed_percent"] if section["speed_percent"] is not None else \
            project["path_defaults"]["speed_percent"]
        sections.append(dict(index=number, speed_percent=percent, speed_ips=percent / 100 * top_speed,
                             reverse=section["reverse"], start=section["start"],
                             start_handle=section["start_handle"], end_handle=section["end_handle"],
                             end=section["end"]))
    stops = sum(1 for a, b in zip(layout["sections"], layout["sections"][1:]) if a["reverse"] != b["reverse"])
    return dict(index=index, duration_s=states[-1][1], length_in=states[-1][2],
                top_speed_ips=max(abs(state[6]) for state in states),
                tightest_radius_in=tightest, parts=int(states[-1][0]) + 1, stops=stops,
                headings=layout["headings"], sections=sections,
                states=[[int(s[0]), round(s[1], 4), round(s[2], 3), round(s[3], 3), round(s[4], 3),
                         round(s[5], 2), round(s[6], 3), round(s[7], 5)] for s in states])


def fold_hint(layout, number):
    section = layout["sections"][number]
    leaving = (section["start_handle"][0] - section["start"][0], section["start_handle"][1] - section["start"][1])
    chord = (section["end"][0] - section["start"][0], section["end"][1] - section["start"][1])
    if leaving[0] * chord[0] + leaving[1] * chord[1] >= 0:
        return "Drag its curve handles so they do not cross, or move one of its points."
    facing = layout["headings"][number]
    if section["reverse"]:
        return (f"The robot faces {facing:.0f}° there, so backing up moves it toward {wrap(facing + 180):.0f}°, "
                "away from the next point. Turn off Reverse for this section, or turn the robot first.")
    return (f"The robot faces {facing:.0f}° there, away from the next point. Turn on Reverse for this section, "
            "or turn the robot first.")


def plan_paths(lib, project, feedforward, poses):
    """One plan per step: None for steps that are not paths or have no points."""
    return [plan_path(lib, project, index, poses[index][0], feedforward)
            if step["type"] == "path" and step["points"] else None
            for index, step in enumerate(project["steps"])]


def path_settings(project, feedforward):
    settings = project["path_defaults"]
    return dict(feedforward=feedforward, ramsete_b=settings["ramsete_b"], ramsete_zeta=settings["ramsete_zeta"],
                accel_ips2=settings["accel_ips2"], cornering_ips2=settings["cornering_ips2"],
                track_width_in=feedforward["track_in"], top_speed_ips=feedforward["top_ips"])


def plan(raw, vexsim, cache):
    """Plans every path in the project. Needs a compiled library and a
    measured feedforward; prepare() makes both."""
    project = validate_project(raw)
    poses = designed_poses(project)
    robot = project["robot"]
    state = readiness(vexsim, cache, robot["preset"], robot["tracking"])
    if not state["compiled"] or state["feedforward"] is None:
        return dict(not_ready=True, error="Speeds are not planned yet: the planner is still compiling mclib "
                                          "or measuring this robot.")
    progress = Progress(None, [("compile", "")])
    lib, _, _ = load_library(vexsim, cache, progress, compile_allowed=False)
    plans = plan_paths(lib, project, state["feedforward"], poses)
    return dict(settings=path_settings(project, state["feedforward"]), plans=plans)


def prepare(preset, tracking, vexsim, cache, progress_path):
    progress = Progress(progress_path, [("compile", "Compile mclib"), ("measure", "Measure the robot")])
    lib, _, build_dir = load_library(vexsim, cache, progress)
    return measure_feedforward(lib, build_dir, preset, tracking, progress)


class Recorder:
    """Keeps every frame of a run and streams them to live.jsonl so the page
    can draw the robot while the simulation is still going."""

    def __init__(self, destination, interval=.1):
        self.trace = []
        self.pending = []
        self.live = (Path(destination) / "live.jsonl").open("w")
        self.interval = interval
        self.last_flush = time.monotonic()
        self.phase, self.step = "motion", 0
        self.reset_frame = False

    def record(self, frame):
        frame = dict(frame, phase=self.phase, step=self.step)
        if self.reset_frame:
            frame.update(odometry_error_in=None, coordinate_reset=True)
        self.trace.append(frame)
        self.pending.append(frame)
        if time.monotonic() - self.last_flush >= self.interval:
            self.flush()

    def flush(self):
        for frame in self.pending:
            self.live.write(json.dumps(frame, allow_nan=False) + "\n")
        self.live.flush()
        self.pending.clear()
        self.last_flush = time.monotonic()

    def close(self):
        self.flush()
        self.live.close()


def motion_target(step, end_pose, entry_pose, commanded_heading):
    """Where the controller is asked to finish, as it would be on the robot:
    relative moves count from where the robot is when the step starts."""
    kind = step["type"]
    if kind == "drive":
        radians = math.radians(commanded_heading)
        return dict(x=entry_pose[0] + step["distance"] * math.sin(radians),
                    y=entry_pose[1] + step["distance"] * math.cos(radians), heading=commanded_heading)
    if kind in ("point", "wall_reset"):
        return dict(x=step["x"], y=step["y"], heading=step.get("heading"))
    if kind in ("boomerang", "path"):
        return dict(x=end_pose[0], y=end_pose[1], heading=end_pose[2])
    if kind == "turn_to_point":
        angle = math.degrees(math.atan2(step["x"] - entry_pose[0], step["y"] - entry_pose[1]))
        return dict(x=None, y=None, heading=angle + (180 if step["direction"] == -1 else 0))
    if kind in ("arc", "reverse_arc"):
        initial, final = math.radians(entry_pose[2]), math.radians(step["heading"])
        return dict(x=entry_pose[0] + step["radius"] * (math.cos(initial) - math.cos(final)),
                    y=entry_pose[1] + step["radius"] * (math.sin(final) - math.sin(initial)),
                    heading=step["heading"])
    return dict(x=None, y=None, heading=step["heading"])


def follow_path(lib, bridge, project, step, start, feedforward, timeout_ms):
    layout, values, limits = path_inputs(project, step, start, feedforward)
    settings = project["path_defaults"]
    ramsete = [feedforward["ks"], feedforward["kv"], settings["ramsete_b"], settings["ramsete_zeta"],
               feedforward["track_in"]]
    result = lib.planner_follow(doubles(values), len(layout["sections"]), doubles(limits), doubles(ramsete),
                                timeout_ms, step["volts"])
    if bridge.exceptions:
        raise RuntimeError("; ".join(bridge.exceptions))
    if result < 0:
        raise ValueError("mclib could not plan this path")
    return result


def invoke(lib, bridge, project, step, start, feedforward, timeout_ms):
    """Runs one step with the real mclib call. Returns followTrajectory()'s
    MotionResult for paths, wallReset()'s result for wall resets."""
    kind = step["type"]
    if kind == "path":
        return follow_path(lib, bridge, project, step, start, feedforward, timeout_ms)
    if kind == "reverse_arc" and math.remainder(step["heading"] - bridge.pose()[2], 360) * step["radius"] > 1e-8:
        raise ValueError("A reverse arc needs the radius and the heading change to have opposite signs "
                         "from where the robot is now. Flip the radius sign or change the heading")
    if kind == "wall_reset":
        bridge.sim.dynamics.locked = True
    first = step.get("distance", step.get("x", step.get("heading", 0)))
    second = step.get("y", step.get("radius", 0))
    return bridge.run(ACTIONS[kind], a=first, b=second, heading=step.get("heading", 0),
                      timeout=timeout_ms, direction=step.get("direction", 1), stop=True, volts=step["volts"],
                      current=step.get("current_ma", 2500), lead=step.get("lead", .5))


def path_gap(frames, states):
    """Largest distance between the simulated robot and the nearest planned
    point, and when it happened."""
    worst, when = 0.0, None
    planned = [(state[3], state[4]) for state in states]
    for frame in frames:
        position = (frame["true_x"], frame["true_y"])
        gap = min(math.dist(position, point) for point in planned)
        if gap > worst:
            worst, when = gap, frame["t"]
    return worst, when


def check(key, label, passed, message, measured=None, limit=None, unit=""):
    return dict(id=key, label=label, passed=bool(passed), message=message, measured=measured, limit=limit,
                unit=unit)


def score_step(step, target, timeout_ms, duration_ms, actual, estimate_at_exit, truth_at_exit, stopped,
               motion_result, configuration_error):
    """The pass/fail checks for one step, each with the measured number and
    its limit in words."""
    kind = step["type"]
    checks = []
    if configuration_error:
        checks.append(check("setup", "Setup", False, f"Not run: {configuration_error}."))
    checks.append(check("stopped", "Stopped", stopped, "The drive was stopped at the end." if stopped else
                        "The drive was still powered when the step ended."))
    if kind == "wall_reset":
        touched = bool(motion_result)
        checks.append(check("wall", "Wall contact", touched, "Detected the wall (motor stall)." if touched else
                            f"Did not detect the wall: the motor current never stayed above {step['current_ma']:g} mA."))
        miss = max(math.dist(estimate_at_exit[:2], (step["x"], step["y"])),
                   abs(math.remainder(estimate_at_exit[2] - step["heading"], 360)))
        checks.append(check("reset", "Odometry reset", touched and miss < 1e-6,
                            f"Odometry was set to ({step['x']:g}, {step['y']:g}) facing {step['heading']:g}°."
                            if touched and miss < 1e-6 else "Odometry was not set to the requested pose.",
                            measured=miss, limit=1e-6))
        return checks, None, None
    seconds, limit_s = duration_ms / 1000, timeout_ms / 1000
    finished = duration_ms < timeout_ms - .5
    checks.append(check("time", "Time", finished,
                        f"Finished in {seconds:.2f} s (limit {limit_s:.2f} s)." if finished else
                        f"Used the whole {limit_s:.2f} s time limit without finishing.",
                        measured=round(seconds, 3), limit=limit_s, unit="s"))
    position_error = heading_error = None
    if target["x"] is not None:
        position_error = math.hypot(actual[0] - target["x"], actual[1] - target["y"])
        limit = POSITION_LIMIT_IN.get(kind, DEFAULT_POSITION_LIMIT_IN)
        passed = position_error <= limit
        checks.append(check("position", "Position", passed,
                            f"Ended {position_error:.2f} in from the target (limit {limit:g} in)." if passed else
                            f"Ended {position_error:.2f} in from the target. The limit is {limit:g} in.",
                            measured=round(position_error, 3), limit=limit, unit="in"))
    if target["heading"] is not None:
        heading_error = abs(math.remainder(actual[2] - target["heading"], 360))
        passed = heading_error <= HEADING_LIMIT_DEG
        checks.append(check("heading", "Heading", passed,
                            f"Ended {heading_error:.1f}° off the target heading (limit {HEADING_LIMIT_DEG:g}°)."
                            if passed else f"Ended {heading_error:.1f}° off the target heading. "
                                           f"The limit is {HEADING_LIMIT_DEG:g}°.",
                            measured=round(heading_error, 3), limit=HEADING_LIMIT_DEG, unit="°"))
    return checks, position_error, heading_error


def advice(step, checks, position_error, odometry_error, finished):
    notes = []
    if odometry_error is not None and odometry_error > ODOMETRY_DRIFT_IN:
        notes.append(f"Odometry and the simulated robot were {odometry_error:.1f} in apart when the step ended. "
                     "Wheel slip fools drive encoders; check the tracking setup before changing gains.")
    if not finished:
        notes.append("The step ran out of time. Look at the voltage and heading graphs for oscillation or "
                     "too little output, or raise the time limit.")
    position = next((c for c in checks if c["id"] == "position"), None)
    if position and not position["passed"] and odometry_error is not None and odometry_error <= ODOMETRY_DRIFT_IN:
        notes.append("Odometry agreed with the real position, so the controller itself stopped short. "
                     "Check its exit conditions and gains.")
    return notes


def execute(raw, destination, vexsim, cache):
    """Runs the whole routine in vexsim with mclib's C++ and scores each step."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    progress = Progress(destination / "progress.json",
                        [("check", "Check the routine"), ("compile", "Compile mclib"),
                         ("measure", "Measure the robot"), ("plan", "Plan path speeds"),
                         ("simulate", "Simulate in vexsim"), ("score", "Score each step")])
    try:
        return _execute(raw, destination, vexsim, cache, progress)
    except Exception as error:
        stage = progress.active() or "check"
        if stage:
            progress.finish(stage, str(error), status="failed")
        step = getattr(error, "step", None)
        write_json(destination / "error.json", dict(error=str(error), stage=stage, step=step))
        raise


def _execute(raw, destination, vexsim, cache, progress):
    progress.begin("check", "Checking every step and the points of every path.")
    try:
        project = validate_project(raw)
        poses = designed_poses(project)
    except ProjectError as error:
        raise StageError(str(error), error.step)
    for index, step in enumerate(project["steps"]):
        if step["type"] == "path" and not step["points"]:
            raise StageError(f"{step_name(index, step)} has no points yet. Click on the field to add at least one.",
                             index)
    write_json(destination / "project.json", project)
    paths = [i for i, step in enumerate(project["steps"]) if step["type"] == "path"]
    progress.finish("check", f"{len(project['steps'])} steps, {len(paths)} of them paths.")
    lib, fingerprint, build_dir = load_library(vexsim, cache, progress)
    robot = project["robot"]
    feedforward = plans = None
    if paths:
        feedforward = measure_feedforward(lib, build_dir, robot["preset"], robot["tracking"], progress)
        progress.begin("plan", "Planning speeds along every path with mclib's trajectory planner.")
        plans = plan_paths(lib, project, feedforward, poses)
        total = sum(item["duration_s"] for item in plans if item)
        count = len(paths)
        progress.finish("plan", f"Planned {count} path{'s' * (count != 1)}: {total:.2f} s of driving.")
    else:
        progress.skip("measure", "No path steps, so no feedforward is needed.")
        progress.skip("plan", "No path steps to plan.")
    time_limits = [step_time_limit(step, plans[index] if plans else None)
                   for index, step in enumerate(project["steps"])]
    budget_s = (sum(time_limits) + LIMITS["hold_ms"] * len(time_limits)) / 1000
    if budget_s > LIMITS["max_simulated_s"]:
        raise StageError(f"The time limits add up to {budget_s:.1f} s with the 0.3 s pause after each step. "
                         f"The simulator runs at most {LIMITS['max_simulated_s']} s. Lower some time limits "
                         "or split the routine.")
    conditions = project["conditions"]
    start = project["start"]
    bridge = harness.PhysicsBridge(lib, robot["preset"], soc=conditions["battery_percent"] / 100,
                                   friction=conditions["grip_percent"] / 100,
                                   encoder_heading=robot["heading_source"] == "encoders",
                                   seed=conditions["noise_seed"], tracking_mode=robot["tracking"],
                                   tuning=project["gains"], start_pose=(start["x"], start["y"], start["heading"]),
                                   sim_time_limit=LIMITS["max_simulated_s"])
    recorder = Recorder(destination)
    bridge.on_capture = recorder.record
    progress.begin("simulate", "Running each step with the real mclib motion calls against vexsim physics.")
    rows = []
    halted = False
    total = len(project["steps"])
    wall_start = time.monotonic()
    for index, step in enumerate(project["steps"]):
        label = STEP_TYPES[step["type"]]["label"]
        progress.step = dict(index=index, total=total, label=label)
        progress.note("simulate", f"Step {index + 1} of {total}: {label}.")
        timeout_ms = time_limits[index]
        if halted:
            rows.append(dict(index=index, type=step["type"], executed=False, passed=False,
                             reason="Skipped: an earlier step failed and Stop at the first failed step is on.",
                             checks=[], notes=[], target=None, timeout_ms=timeout_ms,
                             start_time_s=bridge.sim.t, end_time_s=bridge.sim.t))
            continue
        designed_start, designed_end = poses[index]
        target = motion_target(step, designed_end, bridge.pose(), bridge.commanded_heading())
        started = bridge.sim.t
        recorder.phase, recorder.step = "motion", index
        recorder.record(bridge.capture())
        configuration_error = None
        try:
            motion_result = invoke(lib, bridge, project, step, designed_start, feedforward, timeout_ms)
        except ValueError as error:
            configuration_error = str(error)
            motion_result = None
        returned = bridge.sim.t
        truth_at_exit, estimate_at_exit = bridge.truth(), bridge.pose()
        coordinate_reset = step["type"] == "wall_reset" and bool(motion_result)
        recorder.reset_frame = coordinate_reset
        recorder.record(bridge.capture())
        recorder.phase = "hold"
        bridge.advance_idle(LIMITS["hold_ms"])
        recorder.record(bridge.capture())
        actual = bridge.truth()
        duration_ms = (returned - started) * 1000
        checks, position_error, heading_error = score_step(
            step, target, timeout_ms, duration_ms, actual, estimate_at_exit, truth_at_exit, bridge.stopped(),
            motion_result, configuration_error)
        passed = all(item["passed"] for item in checks)
        odometry_error = None if coordinate_reset else math.dist(estimate_at_exit[:2], truth_at_exit[:2])
        finished = duration_ms < timeout_ms - .5
        notes = advice(step, checks, position_error, odometry_error, finished)
        info = []
        if odometry_error is not None:
            info.append(f"Odometry was {odometry_error:.2f} in from the simulated position when the step ended.")
        if coordinate_reset:
            info.append("The wall reset moved the odometry frame, so odometry is no longer compared with the "
                        "simulated position.")
        if step["type"] == "path":
            moving = [f for f in recorder.trace if f["step"] == index and f["phase"] == "motion"
                      and f["t"] <= returned]
            gap, when = path_gap(moving, plans[index]["states"])
            info.append(f"Largest gap from the planned path: {gap:.2f} in" +
                        (f", {when - started:.2f} s into the step." if when is not None else "."))
            info.append(f"Planned to take {plans[index]['duration_s']:.2f} s; the step took "
                        f"{duration_ms / 1000:.2f} s including the final turn and settle.")
            if motion_result is not None:
                info.append(f"followTrajectory() returned {MOTION_RESULTS[motion_result]}.")
        failed = [item["message"] for item in checks if not item["passed"]]
        rows.append(dict(index=index, type=step["type"], executed=True, passed=passed,
                         reason="All checks passed." if passed else " ".join(failed),
                         checks=checks, notes=notes, info=info, target=target, timeout_ms=timeout_ms,
                         position_error_in=position_error, heading_error_deg=heading_error,
                         odometry_error_in=odometry_error, duration_s=round(duration_ms / 1000, 3),
                         start_time_s=started, return_time_s=returned, end_time_s=bridge.sim.t,
                         true_pose=actual, estimated_pose_at_exit=estimate_at_exit,
                         coordinate_reset=coordinate_reset,
                         motion_result=MOTION_RESULTS[motion_result] if step["type"] == "path" and
                         motion_result is not None else None))
        halted = not passed and project["stop_on_failure"]
        write_json(destination / "live.json", dict(steps=rows))
    bridge.on_capture = None
    recorder.close()
    progress.step = None
    progress.finish("simulate", f"Simulated {bridge.sim.t:.2f} s of robot time in "
                                f"{time.monotonic() - wall_start:.1f} s.")
    progress.begin("score", "Collecting the checks of every step.")
    if source_fingerprint(vexsim) != fingerprint:
        raise StageError("The C++ or the simulator changed while this ran. Run again before trusting the result.")
    passed_steps = sum(row["passed"] for row in rows)
    summary = dict(steps=len(rows), passed_steps=passed_steps,
                   failed_steps=sum(row["executed"] and not row["passed"] for row in rows),
                   skipped_steps=sum(not row["executed"] for row in rows),
                   simulated_s=round(bridge.sim.t, 3))
    result = dict(passed=passed_steps == len(rows), summary=summary, steps=rows, trace=recorder.trace,
                  plans=plans or [None] * len(rows), settings=path_settings(project, feedforward) if feedforward else None,
                  designed_poses=poses, project=project, source_fingerprint=fingerprint,
                  stages=progress.stages)
    write_json(destination / "result.json", result)
    with (destination / "trace.csv").open("w", newline="") as handle:
        fields = list(dict.fromkeys(key for frame in recorder.trace for key in frame))
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(recorder.trace)
    progress.finish("score", f"{passed_steps} of {len(rows)} steps passed.")
    result["stages"] = progress.stages
    write_json(destination / "result.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("prepare", "plan", "run"))
    parser.add_argument("--project", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--preset", choices=sorted(PRESETS))
    parser.add_argument("--tracking", choices=("two", "drive"))
    parser.add_argument("--progress", type=Path)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--cache", type=Path, required=True)
    args = parser.parse_args()
    if not (args.vexsim / "vexsim/sim.py").is_file():
        parser.error(f"--vexsim {args.vexsim} is not a vexsim checkout")
    if args.command == "prepare":
        if not args.preset or not args.tracking:
            parser.error("prepare needs --preset and --tracking")
        try:
            print(json.dumps(prepare(args.preset, args.tracking, args.vexsim, args.cache, args.progress)))
            return 0
        except Exception as error:
            print(f"Prepare failed: {error}", file=sys.stderr)
            return 2
    if not args.project:
        parser.error(f"{args.command} needs --project")
    if args.command == "plan":
        try:
            with contextlib.redirect_stdout(sys.stderr):
                answer = plan(strict_json(args.project.read_text()), args.vexsim, args.cache)
        except (ValueError, StageError) as error:
            answer = dict(error=str(error), step=getattr(error, "step", None))
        print(json.dumps(answer, allow_nan=False))
        return 0
    if not args.output:
        parser.error("run needs --output")
    try:
        result = execute(strict_json(args.project.read_text()), args.output, args.vexsim, args.cache)
    except Exception as error:
        print(f"Run failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result["summary"]))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
