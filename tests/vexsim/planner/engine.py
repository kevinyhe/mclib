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
from fields import contact_dict, contacts_between, describe_contact, make_world, piece_list  # noqa: E402
from mechanisms import (AUTONOMOUS_S, MECHANISMS, MOVE_RESULTS, CascadeRig, RuleWatch,  # noqa: E402
                        autonomous_summary, claw_events, declare_cascade, describe_match_load, frame_telemetry,
                        load_match, mechanism_reading, score_summary, toggle_events, toggle_faces)
from geometry import designed_poses, path_layout, section_values, wrap  # noqa: E402
from project import (LIMITS, PRESETS, STEP_TYPES, STILL_STEPS, ProjectError, carries_speed, step_name,  # noqa: E402
                     step_time_limit, strict_json, validate_project)

PATH_BRIDGE = HERE / "path_bridge.cpp"
CASCADE_BRIDGE = HERE / "cascade_bridge.cpp"
EXTRA_SOURCES = [PATH_BRIDGE, CASCADE_BRIDGE, ROOT / "src/mclib/command/subsystem.cpp",
                 ROOT / "src/mclib/mechanism/position_mechanism.cpp"]
STATE_VALUES = 8
MAX_STATES = 40000
MOTION_RESULTS = ("Reached", "TimedOut", "Cancelled", "Disabled", "InvalidValue", "NoDrive")
ACTIONS = dict(turn=0, drive=1, arc=2, reverse_arc=3, swing=4, turn_to_point=5, point=6, boomerang=7,
               wall_reset=8)
POSITION_LIMIT_IN = dict(boomerang=1.5, arc=5.5, reverse_arc=5.5)
DEFAULT_POSITION_LIMIT_IN = 2.5
HEADING_LIMIT_DEG = 5.0
ODOMETRY_DRIFT_IN = 2.5
HEADING_GOALS = {"turn", "swing", "arc", "reverse_arc"}
RAMSETE_SIDE_TOLERANCE_IN = 3.0
PLANNER_RAMSETE_B = 50
RAMSETE_BACK_OFF_IN = (6.0, 18.0)
#: Rocking up less than this onto the edge of the wheels is not reported
ROCK_REPORT_DEG = 0.5
#: Slower than both of these, the robot is standing still
STILL_IPS = 0.5
STILL_DPS = 2.0


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
    files = [HERE.parent / "run.py", HERE.parent / "bridge.cpp", PATH_BRIDGE, HERE / "path_helpers.hpp",
             CASCADE_BRIDGE, HERE / "cascade_helpers.hpp"]
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
    lib.planner_plan.argtypes = [doubles, C.c_int, doubles, C.c_double, C.c_double, doubles, C.c_int]
    lib.planner_plan.restype = C.c_int
    lib.planner_follow.argtypes = [doubles, C.c_int, doubles, doubles, C.c_double, C.c_double, C.c_double,
                                   C.c_double, C.c_int]
    lib.planner_follow.restype = C.c_int
    lib.planner_folded_section.argtypes = [doubles, C.c_int]
    lib.planner_folded_section.restype = C.c_int
    declare_cascade(lib)
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
                harness.build(build_dir, extra_sources=EXTRA_SOURCES)
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


def planned_states(lib, project, index, start, feedforward, start_ips, end_ips):
    """mclib's planned states for path step @p index starting at pose
    @p start, entering at up to @p start_ips and leaving at up to @p end_ips."""
    step = project["steps"][index]
    layout, values, limits = path_inputs(project, step, start, feedforward)
    out = (C.c_double * (STATE_VALUES * MAX_STATES))()
    count = lib.planner_plan(doubles(values), len(layout["sections"]), doubles(limits), start_ips, end_ips, out,
                             MAX_STATES)
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
    return layout, [out[STATE_VALUES * i:STATE_VALUES * (i + 1)] for i in range(count)]


def plan_path(lib, project, index, start, feedforward, start_ips=0.0, end_ips=0.0):
    """mclib's plan for path step @p index starting at @p start."""
    layout, states = planned_states(lib, project, index, start, feedforward, start_ips, end_ips)
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
                requested_start_ips=start_ips, requested_end_ips=end_ips,
                start_speed_ips=abs(states[0][6]), end_speed_ips=abs(states[-1][6]),
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
    """One plan per step: None for steps that are not paths or have no points.

    Where a path keeps moving into another path, the first ends at the
    fastest speed the second can start at (last to first), and the second
    starts at the speed the first reached (first to last)."""
    steps = project["steps"]
    end_ips = [0.0] * len(steps)
    for index in reversed(range(len(steps))):
        if carries_speed(project, index):
            following = index + 1
            _, fastest = planned_states(lib, project, following, poses[following][0], feedforward,
                                        feedforward["top_ips"], end_ips[following])
            end_ips[index] = abs(fastest[0][6])
    plans, start_ips = [], 0.0
    for index, step in enumerate(steps):
        plan = plan_path(lib, project, index, poses[index][0], feedforward, start_ips, end_ips[index]) \
            if step["type"] == "path" and step["points"] else None
        plans.append(plan)
        start_ips = plan["end_speed_ips"] if carries_speed(project, index) else 0.0
    return plans


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
    if kind in STILL_STEPS:
        return dict(x=None, y=None, heading=None)
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


def follow_path(lib, bridge, project, step, start, feedforward, timeout_ms, plan):
    layout, values, limits = path_inputs(project, step, start, feedforward)
    settings = project["path_defaults"]
    ramsete = [feedforward["ks"], feedforward["kv"], settings["ramsete_b"], settings["ramsete_zeta"],
               feedforward["track_in"]]
    result = lib.planner_follow(doubles(values), len(layout["sections"]), doubles(limits), doubles(ramsete),
                                plan["requested_start_ips"], plan["requested_end_ips"], timeout_ms, step["volts"],
                                0 if step["chain"] else 1)
    if bridge.exceptions:
        raise RuntimeError("; ".join(bridge.exceptions))
    if result < 0:
        raise ValueError("mclib could not plan this path")
    return result


def invoke(lib, bridge, rig, project, step, start, feedforward, timeout_ms, plan):
    """Runs one step with the real mclib call. Returns followTrajectory()'s
    MotionResult for paths, wallReset()'s result for wall resets, and
    cascade::MoveResult for the lift and the claw."""
    kind = step["type"]
    if kind in MECHANISMS:
        return rig.move(kind, step[f"{kind}_preset"], timeout_ms)
    if kind == "path":
        return follow_path(lib, bridge, project, step, start, feedforward, timeout_ms, plan)
    if kind == "reverse_arc" and math.remainder(step["heading"] - bridge.pose()[2], 360) * step["radius"] > 1e-8:
        raise ValueError("A reverse arc needs the radius and the heading change to have opposite signs "
                         "from where the robot is now. Flip the radius sign or change the heading")
    if kind == "wall_reset" and project["field"] == "open":
        bridge.sim.dynamics.locked = True
    first = step.get("distance", step.get("x", step.get("heading", 0)))
    second = step.get("y", step.get("radius", 0))
    return bridge.run(ACTIONS[kind], a=first, b=second, heading=step.get("heading", 0),
                      timeout=timeout_ms, direction=step.get("direction", 1), stop=not step["chain"],
                      volts=step["volts"],
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


def wheel_side(edge):
    """Which wheels a footprint edge of vexsim's chassis is: front, back, left or right."""
    if abs(edge.nx) >= abs(edge.ny):
        return "front" if edge.nx > 0 else "back"
    return "left" if edge.ny > 0 else "right"


class TipWatch:
    """How far the robot rocks up onto one edge of its wheels in each step,
    and when it falls over."""

    def __init__(self, sim):
        self.sim = sim
        self.fell_at = None

    def watch(self):
        if self.fell_at is None and self.sim.dynamics.tilt.fallen:
            self.fell_at = self.sim.t

    def begin_step(self):
        self.sim.dynamics.tilt.clear_most()

    @property
    def fallen(self):
        return self.sim.dynamics.tilt.fallen

    def fall_checks(self):
        """A failed check once the robot has fallen over, else none."""
        from vexsim.units import INCH
        tilt = self.sim.dynamics.tilt
        if not tilt.fallen:
            return []
        return [check("upright", "Upright", False,
                      f"The robot fell over at {self.fell_at:.2f} s. It rocked onto its {wheel_side(tilt.edge)} "
                      f"wheels past {math.degrees(tilt.balance):.0f}°, where its centre of mass, "
                      f"{self.sim.chassis.cg_height / INCH:.1f} in up, was over them.")]

    def rock_lines(self):
        """How far the robot rocked onto the edge of its wheels this step,
        when it rocked enough to say and did not fall."""
        tilt = self.sim.dynamics.tilt
        if tilt.fallen or math.degrees(tilt.most) < ROCK_REPORT_DEG:
            return []
        return [f"The robot rocked {math.degrees(tilt.most):.1f}° onto its {wheel_side(tilt.most_edge)} wheels. "
                f"It falls over at {math.degrees(tilt.most_balance):.0f}°."]


class OdometryAtRest:
    """Odometry trails a moving robot, so a step that ends moving measures
    that trail along with any drift. For such a step this adds the gap the
    next time the robot stands still, unless a wall reset comes first."""

    def __init__(self, trace):
        self.trace = trace
        self.scanned = 0
        self.waiting = []

    def watch(self, row, speed_ips):
        if row["odometry_error_in"] is not None and speed_ips >= STILL_IPS:
            self.waiting.append((row, speed_ips))

    def report(self):
        for frame in self.trace[self.scanned:]:
            if frame.get("coordinate_reset"):
                self.waiting.clear()
            elif frame["speed_ips"] < STILL_IPS and abs(frame["yaw_rate_dps"]) < STILL_DPS:
                resolved = [entry for entry in self.waiting if entry[0]["return_time_s"] < frame["t"]]
                for row, speed_ips in resolved:
                    self.waiting.remove((row, speed_ips))
                    after = next(number for number, line in enumerate(row["info"])
                                 if line.startswith("Odometry was")) + 1
                    row["info"].insert(after, f"The step ended with the robot moving at {speed_ips:.1f} in/s, "
                                              "and odometry trails a moving robot. The next time the robot "
                                              f"stood still, at {frame['t']:.2f} s in step {frame['step'] + 1}, "
                                              f"odometry was {frame['odometry_error_in']:.2f} in from the "
                                              "simulated position.")
        self.scanned = len(self.trace)


def score_mechanism_step(step, timeout_ms, duration_ms, motion_result, reading):
    kind = step["type"]
    preset = MECHANISMS[kind]["presets"][step[f"{kind}_preset"]]
    seconds, limit_s = duration_ms / 1000, timeout_ms / 1000
    reached = motion_result == MOVE_RESULTS.index("Reached the preset")
    if kind == "claw":
        verb = "closed" if step["claw_preset"] == "closed" else "opened"
        return [check("time", "Time", reached, f"The claw {verb} in {seconds:.2f} s: {reading}.",
                      measured=round(seconds, 3), limit=limit_s, unit="s")]
    if reached:
        message = f"The lift reached {preset} in {seconds:.2f} s (limit {limit_s:.2f} s): {reading}."
    else:
        message = f"The lift did not reach {preset} in the {limit_s:.2f} s time limit: {reading}."
    return [check("time", "Time", reached, message, measured=round(seconds, 3), limit=limit_s, unit="s")]


def score_match_load(step, started_s, error):
    """<SG11a>: Match Loads only in the Driver Controlled Period."""
    if started_s < AUTONOMOUS_S:
        rules = check("rules", "Rules", False,
                      f"Match Loads may only be loaded in the Driver Controlled Period (SG11a). This step starts "
                      f"{started_s:.2f} s into the match, inside the {AUTONOMOUS_S:g} s autonomous period.",
                      measured=round(started_s, 2), limit=AUTONOMOUS_S, unit="s")
    else:
        rules = check("rules", "Rules", True,
                      f"Loaded {started_s:.2f} s into the match, after the {AUTONOMOUS_S:g} s autonomous period "
                      "(SG11a).", measured=round(started_s, 2), limit=AUTONOMOUS_S, unit="s")
    load = describe_match_load(step)
    if error is None:
        loaded = check("loaded", "Loaded", True, f"The Drive Team stood {load}.")
    else:
        loaded = check("loaded", "Loaded", False, f"The Drive Team could not stand {load}: {error}.")
    return [rules, loaded]


def rule_checks(report):
    """Checks for the first step: the robot's size at the start and its motors against the rules."""
    start, motors = report["start_in"], report["motors_w"]
    size = f"{start['length']:.2f} in long, {start['width']:.2f} in wide and {start['height']:.2f} in tall"
    return [check("start_size", "Size at the start", start["passed"],
                  f"The robot starts {size}. Each must be at most {start['limit']:g} in (SG1).",
                  measured=max(start["length"], start["width"], start["height"]), limit=start["limit"], unit="in"),
            check("motors", "Motors", motors["passed"],
                  f"The drive motors are rated {motors['drive']:g} W and all the motors {motors['total']:g} W. "
                  f"The limits are {motors['drive_limit']:g} W for the drive (R11) and {motors['total_limit']:g} W "
                  "in all (R10).", measured=motors["total"], limit=motors["total_limit"], unit="W")]


def score_step(step, target, timeout_ms, duration_ms, actual, estimate_at_exit, truth_at_exit, stopped,
               motion_result, configuration_error):
    """The pass/fail checks for one step, each with the measured number and
    its limit in words."""
    kind = step["type"]
    chained = step["chain"]
    ended = "Handed off" if chained else "Ended"
    checks = []
    if configuration_error:
        checks.append(check("setup", "Setup", False, f"Not run: {configuration_error}."))
    if not chained:
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
                            f"{ended} {position_error:.2f} in from the target (limit {limit:g} in)." if passed else
                            f"{ended} {position_error:.2f} in from the target. The limit is {limit:g} in.",
                            measured=round(position_error, 3), limit=limit, unit="in"))
    if target["heading"] is not None:
        heading_error = abs(math.remainder(actual[2] - target["heading"], 360))
    if heading_error is not None and (not chained or kind in HEADING_GOALS):
        passed = heading_error <= HEADING_LIMIT_DEG
        checks.append(check("heading", "Heading", passed,
                            f"{ended} {heading_error:.1f}° off the target heading (limit {HEADING_LIMIT_DEG:g}°)."
                            if passed else f"{ended} {heading_error:.1f}° off the target heading. "
                                           f"The limit is {HEADING_LIMIT_DEG:g}°.",
                            measured=round(heading_error, 3), limit=HEADING_LIMIT_DEG, unit="°"))
    return checks, position_error, heading_error


def settle_report(plan, frames, started, returned, ramsete_b):
    """What followTrajectory() did after its plan ran out: an info line, and
    a hint when it had to back off and drive back in."""
    plan_end = started + plan["duration_s"]
    after = [frame for frame in frames if frame["t"] >= plan_end]
    if not after:
        return None, None
    frame, end = after[0], plan["states"][-1]
    heading = math.radians(end[5])
    dx, dy = end[3] - frame["estimated_x"], end[4] - frame["estimated_y"]
    side = abs(dx * math.cos(heading) - dy * math.sin(heading))
    miss = math.hypot(dx, dy)
    settle_s = returned - plan_end
    info = (f"When the plan ran out, odometry put the robot {miss:.1f} in from the end point "
            f"({side:.1f} in to the side). The final turn and settle took {settle_s:.2f} s.")
    if side <= RAMSETE_SIDE_TOLERANCE_IN:
        return info, None
    back_off = min(max(4 * side, RAMSETE_BACK_OFF_IN[0]), RAMSETE_BACK_OFF_IN[1])
    hint = (f"The robot finished the plan {side:.1f} in to the side of the end point. followTrajectory() fixes a "
            f"sideways miss over {RAMSETE_SIDE_TOLERANCE_IN:g} in by backing off {back_off:.0f} in and driving back "
            f"in with boomerang(), which took {settle_s:.2f} s here. {staying_on_the_plan(ramsete_b)}")
    return info, hint


def staying_on_the_plan(ramsete_b):
    weak = ramsete_b < PLANNER_RAMSETE_B
    gain = f"raise RAMSETE b (now {ramsete_b:g}; the planner's default is {PLANNER_RAMSETE_B}), " if weak else ""
    return f"To stay on the plan: {gain}lower the speed limit through tight turns, or make the turns wider."


def chained_path_hint(position_error, ramsete_b):
    return (f"A path that keeps moving ends when its plan runs out, with no settle, so the next step starts "
            f"{position_error:.1f} in from where it was planned to. {staying_on_the_plan(ramsete_b)}")


def advice(checks, position_error, odometry_error, finished, chained_path, ramsete_b):
    notes = []
    if odometry_error is not None and odometry_error > ODOMETRY_DRIFT_IN:
        notes.append(f"Odometry and the simulated robot were {odometry_error:.1f} in apart when the step ended. "
                     "Wheel slip fools drive encoders; check the tracking setup before changing gains.")
    if not finished:
        notes.append("The step ran out of time. Look at the voltage and heading graphs for oscillation or "
                     "too little output, or raise the time limit.")
    position = next((c for c in checks if c["id"] == "position"), None)
    if position and not position["passed"] and chained_path:
        notes.append(chained_path_hint(position_error, ramsete_b))
    elif position and not position["passed"] and odometry_error is not None and odometry_error <= ODOMETRY_DRIFT_IN:
        notes.append("Odometry agreed with the real position, so the controller itself stopped short. "
                     "Check its exit conditions and gains.")
    return notes


def stopped_by_hit(contacts, entry, end, target):
    """The one hint for a step that failed after the robot hit something:
    what stopped it and how much of the move it made."""
    hardest = max(contacts, key=lambda contact: contact.peak_force)
    if target["x"] is not None:
        asked = math.dist(entry[:2], (target["x"], target["y"]))
        made = f"it moved {math.dist(entry[:2], end[:2]):.1f} in of the {asked:.1f} in asked for"
    else:
        asked = abs(math.remainder(target["heading"] - entry[2], 360))
        made = f"it turned {abs(math.remainder(end[2] - entry[2], 360)):.0f}° of the {asked:.0f}° asked for"
    return (f"The robot was stopped by {hardest.element}: {made}. The controller was not at fault. "
            "Move the route clear of it, or end the step before it.")


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
    stopping = sum(not step["chain"] for step in project["steps"])
    budget_s = (sum(time_limits) + LIMITS["hold_ms"] * stopping) / 1000
    if budget_s > LIMITS["max_simulated_s"]:
        raise StageError(f"The time limits add up to {budget_s:.1f} s with the 0.3 s pause after each step that "
                         "stops. "
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
    world = make_world(project["field"], bridge.sim, project["preload"])
    rig = CascadeRig(lib, bridge, world) if project["field"] == "override" else None
    rules = RuleWatch(world, PRESETS[robot["preset"]]["drive_watts"]) if rig else None
    rule_breaks = []
    pieces = piece_list(world.pieces) if rig else None
    if world is not None:
        bridge.step_world = world.step

    autonomous = None
    tips = TipWatch(bridge.sim)
    odometry_at_rest = OdometryAtRest(recorder.trace)

    def with_world(frame):
        nonlocal autonomous
        tips.watch()
        if world is None:
            return frame
        if rig:
            broken = rules.watch()
            if broken:
                rule_breaks.append((bridge.sim.t, broken))
        if rig and autonomous is None and bridge.sim.t >= AUTONOMOUS_S:
            autonomous = autonomous_summary(world, rules)
        return dict(frame, **(frame_telemetry(world) if rig else world.telemetry()))

    def capture():
        return with_world(bridge.capture())

    bridge.on_capture = lambda frame: recorder.record(with_world(frame))
    write_json(destination / "live.json", dict(steps=[], pieces=pieces))
    progress.begin("simulate", "Running each step with the real mclib motion calls against vexsim physics.")
    rows = []
    halted = None
    total = len(project["steps"])
    wall_start = time.monotonic()
    for index, step in enumerate(project["steps"]):
        label = STEP_TYPES[step["type"]]["label"]
        progress.step = dict(index=index, total=total, label=label)
        progress.note("simulate", f"Step {index + 1} of {total}: {label}.")
        timeout_ms = time_limits[index]
        if halted:
            rows.append(dict(index=index, type=step["type"], executed=False, passed=False, reason=halted,
                             checks=[], notes=[], target=None, timeout_ms=timeout_ms,
                             start_time_s=bridge.sim.t, end_time_s=bridge.sim.t))
            continue
        designed_start, designed_end = poses[index]
        target = motion_target(step, designed_end, bridge.pose(), bridge.commanded_heading())
        started = bridge.sim.t
        recorder.phase, recorder.step = "motion", index
        recorder.record(capture())
        entry_truth = bridge.truth()
        configuration_error = None
        faces_before = toggle_faces(world) if rig else None
        if rig:
            rig.begin_actions(step["actions"])
        tips.begin_step()
        try:
            if step["type"] == "match_load":
                motion_result = load_match(world, step)
            else:
                motion_result = invoke(lib, bridge, rig, project, step, designed_start, feedforward, timeout_ms,
                                       plans[index] if plans else None)
        except ValueError as error:
            configuration_error = str(error)
            motion_result = None
        late_actions = rig.finish_actions() if rig else 0
        returned = bridge.sim.t
        truth_at_exit, estimate_at_exit = bridge.truth(), bridge.pose()
        coordinate_reset = step["type"] == "wall_reset" and bool(motion_result)
        recorder.reset_frame = coordinate_reset
        recorder.record(capture())
        handoff_speed_ips = recorder.trace[-1]["speed_ips"]
        if not step["chain"]:
            recorder.phase = "hold"
            bridge.advance_idle(LIMITS["hold_ms"])
            recorder.record(capture())
        actual = bridge.truth()
        duration_ms = (returned - started) * 1000
        if step["type"] in MECHANISMS:
            reading, _ = mechanism_reading(world, step["type"])
            checks = score_mechanism_step(step, timeout_ms, duration_ms, motion_result, reading)
            position_error = heading_error = None
        elif step["type"] == "match_load":
            checks = score_match_load(step, started, configuration_error)
            position_error = heading_error = None
        else:
            checks, position_error, heading_error = score_step(
                step, target, timeout_ms, duration_ms, actual, estimate_at_exit, truth_at_exit, bridge.stopped(),
                motion_result, configuration_error)
        if rig and index == 0:
            checks = rule_checks(rules.report()) + checks
        checks += [check("rules", "Rules", False, message) for t, message in rule_breaks
                   if started <= t <= bridge.sim.t]
        checks += tips.fall_checks()
        passed = all(item["passed"] for item in checks)
        odometry_error = None if coordinate_reset else math.dist(estimate_at_exit[:2], truth_at_exit[:2])
        finished = duration_ms < timeout_ms - .5
        chained_path = step["chain"] and step["type"] == "path"
        contacts = contacts_between(world, started, bridge.sim.t)
        hit_hint = (stopped_by_hit(contacts, entry_truth, actual, target)
                    if contacts and not passed and step["type"] not in STILL_STEPS else None)
        if step["type"] == "lift":
            notes = [] if finished else ["The lift ran out of time. Check the lift current in the telemetry: high "
                                         "current means it was pushing against something."]
        elif step["type"] in STILL_STEPS:
            notes = []
        elif hit_hint:
            notes = [hit_hint]
        else:
            notes = advice(checks, position_error, odometry_error, finished, chained_path,
                           project["path_defaults"]["ramsete_b"])
        info = tips.rock_lines()
        if odometry_error is not None:
            info.append(f"Odometry was {odometry_error:.2f} in from the simulated position when the step ended.")
        if coordinate_reset:
            info.append("The wall reset moved the odometry frame, so odometry is no longer compared with the "
                        "simulated position.")
        if step["chain"]:
            facing = f", {heading_error:.1f}° off this step's target heading" if heading_error is not None else ""
            info.append(f"Kept moving into step {index + 2} at {handoff_speed_ips:.1f} in/s{facing}. "
                        "The next step started at once, with no pause.")
        if step["type"] == "path":
            moving = [f for f in recorder.trace if f["step"] == index and f["phase"] == "motion"
                      and f["t"] <= returned]
            gap, when = path_gap(moving, plans[index]["states"])
            info.append(f"Largest gap from the planned path: {gap:.2f} in" +
                        (f", {when - started:.2f} s into the step." if when is not None else "."))
            if step["chain"]:
                info.append(f"Planned to take {plans[index]['duration_s']:.2f} s; the step took "
                            f"{duration_ms / 1000:.2f} s. It kept moving, so there was no final turn or settle.")
            else:
                info.append(f"Planned to take {plans[index]['duration_s']:.2f} s; the step took "
                            f"{duration_ms / 1000:.2f} s including the final turn and settle.")
                settle_info, settle_hint = settle_report(plans[index], moving, started,
                                                         returned, project["path_defaults"]["ramsete_b"])
                if settle_info:
                    info.append(settle_info)
                if settle_hint and not hit_hint:
                    notes.append(settle_hint)
            if motion_result is not None:
                info.append(f"followTrajectory() returned {MOTION_RESULTS[motion_result]}.")
        if late_actions:
            info.append(f"{late_actions} mechanism action{'s' * (late_actions != 1)} had not run when the "
                        "motion ended, so " + ("they" if late_actions != 1 else "it") + " ran then.")
        info.extend(describe_contact(contact, started) for contact in contacts)
        if rig:
            info.extend(claw_events(world, started, bridge.sim.t))
            info.extend(toggle_events(world, faces_before))
        failed = [item["message"] for item in checks if not item["passed"]]
        rows.append(dict(index=index, type=step["type"], executed=True, passed=passed,
                         contacts=[contact_dict(contact, started) for contact in contacts],
                         reason="All checks passed." if passed else " ".join(failed),
                         checks=checks, notes=notes, info=info, target=target, timeout_ms=timeout_ms,
                         position_error_in=position_error, heading_error_deg=heading_error,
                         odometry_error_in=odometry_error, duration_s=round(duration_ms / 1000, 3),
                         start_time_s=started, return_time_s=returned, end_time_s=bridge.sim.t,
                         true_pose=actual, estimated_pose_at_exit=estimate_at_exit,
                         coordinate_reset=coordinate_reset,
                         score=score_summary(world) if rig else None,
                         move_result=MOVE_RESULTS[motion_result] if step["type"] in MECHANISMS and
                         motion_result is not None else None,
                         motion_result=MOTION_RESULTS[motion_result] if step["type"] == "path" and
                         motion_result is not None else None))
        odometry_at_rest.watch(rows[-1], handoff_speed_ips)
        odometry_at_rest.report()
        if tips.fallen:
            halted = f"Skipped: the robot fell over in step {index + 1}."
        elif not passed and project["stop_on_failure"]:
            halted = "Skipped: an earlier step failed and Stop at the first failed step is on."
        write_json(destination / "live.json", dict(steps=rows, pieces=pieces))
    bridge.on_capture = None
    recorder.close()
    if rig and autonomous is None:
        autonomous = autonomous_summary(world, rules)
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
                  stages=progress.stages, pieces=pieces, score=score_summary(world) if rig else None,
                  autonomous=autonomous)
    write_json(destination / "result.json", result)
    with (destination / "trace.csv").open("w", newline="") as handle:
        fields = list(dict.fromkeys(key for frame in recorder.trace for key in frame if key != "pieces"))
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
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
