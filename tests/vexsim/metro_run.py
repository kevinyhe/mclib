#!/usr/bin/env python3
"""Run Metro C++ autonomous, odometry and intake against native vexsim.

The source checkout is an input, never modified. Hardware/task shims are host
adapters, not substitute motion controllers. The build applies the documented
signed-inner-arc correction to a private build copy. One game tick is one ms.
"""
import argparse
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

from metro_waypoints import correct_waypoints

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
METRO_COMMIT = "0e13c4c6fc6aa7097f97074d6969c05d1d43dc05"
SOURCE_FILES = ("src/control.cpp", "src/pid.cpp", "src/utils.cpp", "src/config.cpp",
                "src/autonomous.cpp", "src/mechanism/intake.cpp")
INTAKE_STATES = ("DISABLED", "INDEX", "MIDDLE_GOAL_PRIME", "MIDDLE_GOAL",
                 "MIDDLE_GOAL_SCORE", "SCORE", "REVERSE", "REVERSE_SLOW", "DEJAM")
EVENT_NAMES = ("task_created", "task_sleep", "task_ended", "routine_returned",
               "disabled", "runtime_error", "encoder_tare", "imu_set")


class MetroSample(C.Structure):
    _fields_ = [(key, C.c_double) for key in (
        "heading", "left_deg", "right_deg", "left_current_ma", "right_current_ma",
        "left_rpm", "right_rpm", "tracker_centideg", "optical_proximity",
        "optical_hue", "distance_mm")]


OUTPUT = C.CFUNCTYPE(None, C.c_int, C.c_int, C.c_int, C.c_double)
EVENT = C.CFUNCTYPE(None, C.c_int, C.c_int, C.c_double, C.c_double)


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False, separators=(",", ":")) + "\n")


def source_manifest(source):
    paths = [source / name for name in SOURCE_FILES]
    paths += [p for p in (source / "include").rglob("*") if p.is_file()
              and p.suffix in (".h", ".hpp")
              and p.relative_to(source / "include").parts[0]
              not in ("Eigen", "lemlib", "liblvgl", "pros")]
    return {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def adapter_manifest():
    paths = [HERE / name for name in ("metro_run.py", "metro_game.py", "metro_host.cpp", "metro_host.hpp", "metro_waypoints.py")]
    paths += list((HERE / "metro_shims").rglob("*.h*"))
    return {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths) if p.is_file()}


def correct_curve_inner_arc(control):
    """Keep inner-wheel reversal for radii inside the track; fail on drift."""
    old = "in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);"
    new = "in_arc = (fabs(center_radius) - (distance_between_wheels / 2)) * fabs(result_angle);"
    if control.count(old) != 1:
        raise ValueError("Expected exactly one reviewed Metro inner-arc expression")
    return control.replace(old, new)


def correct_odometry_turn_offset(control):
    """Opposite wheel sides require opposite rotation offsets."""
    start = control.index("void trackNoOdomWheel()")
    end = control.index("    // Update global position", start)
    section = control[start:end]
    old = "delta_left_in / delta_heading_rad + distance_between_wheels / 2.0"
    if section.count(old) != 1:
        raise ValueError("Expected one reviewed left-wheel odometry offset")
    return control[:start] + section.replace(old, old.replace(" + ", " - ")) + control[end:]


def build(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    revision = subprocess.run(["git", "-C", str(source), "rev-parse", "HEAD"],
                              check=True, capture_output=True, text=True).stdout.strip()
    if revision != METRO_COMMIT:
        raise ValueError(f"Metro revision {revision} has not been reviewed; expected {METRO_COMMIT}")
    dirty = subprocess.run(["git", "-C", str(source), "status", "--porcelain"],
                           check=True, capture_output=True, text=True).stdout
    if dirty:
        raise ValueError("Use an unchanged Metro source snapshot; source modifications are not accepted")
    before = source_manifest(source)
    adapter_before = adapter_manifest()
    output.mkdir(parents=True, exist_ok=True)
    corrected_control = output / "control-corrected.cpp"
    corrected_control.write_text(correct_odometry_turn_offset(
        correct_curve_inner_arc((source / "src/control.cpp").read_text())))
    corrections = [dict(name="signed_inner_arc", source="src/control.cpp",
                        description="Preserve negative inner-wheel travel when radius is smaller than half the track.",
                        compiled_path=str(corrected_control),
                        sha256=hashlib.sha256(corrected_control.read_bytes()).hexdigest())]
    corrections.append(dict(corrections[0], name="opposite_odometry_turn_offsets",
                            description="Subtract the left-wheel turn offset; retain positive right-wheel offset."))
    original_autonomous = (source / "src/autonomous.cpp").read_text()
    autonomous_text = correct_waypoints(original_autonomous)
    corrected_autonomous = source / "src/autonomous.cpp"
    if autonomous_text != original_autonomous:
        corrected_autonomous = output / "autonomous-waypoints.cpp"
        corrected_autonomous.write_text(autonomous_text)
        corrections.append(dict(name="metro_edited_points", source="src/autonomous.cpp",
                                description="User-editable POINTS in metro_waypoints.py; all non-coordinate autonomous code remains original.",
                                compiled_path=str(corrected_autonomous),
                                sha256=hashlib.sha256(corrected_autonomous.read_bytes()).hexdigest()))
    library = output / "libmetro_sim.so"
    command = ["g++", "-std=gnu++23", "-O1", "-g", "-shared", "-fPIC", "-pthread",
               "-fsanitize=float-cast-overflow", "-fno-sanitize-recover=float-cast-overflow",
               "-include", str(HERE / "metro_host.hpp"),
               "-I" + str(HERE / "metro_shims"), "-I" + str(source / "include"),
               "-I" + str(source / "include/mechanism"), str(HERE / "metro_host.cpp")]
    command += [str(corrected_control if name == "src/control.cpp" else
                    corrected_autonomous if name == "src/autonomous.cpp" else source / name)
                for name in SOURCE_FILES]
    command += ["-Wl,-z,defs", "-o", str(library)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
    (output / "build.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(f"Metro host compilation failed; see {output / 'build.log'}")
    if source_manifest(source) != before:
        raise RuntimeError("Metro source changed during compilation")
    if adapter_manifest() != adapter_before:
        raise RuntimeError("Host adapter changed during compilation")
    manifest = dict(repository="kevinyhe/metro", source_commit=revision, source_files=before,
                    adapter_files=adapter_before,
                    controller_corrections=corrections,
                    command=command, library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    write_json(output / "source-manifest.json", manifest)
    return library, manifest


def load_library(path):
    lib = C.CDLL(str(Path(path).resolve()))
    lib.metro_init.argtypes = [C.POINTER(MetroSample), OUTPUT, EVENT]
    lib.metro_init.restype = C.c_int
    lib.metro_tick.argtypes = [C.c_uint32, C.POINTER(MetroSample)]
    lib.metro_tick.restype = C.c_int
    lib.metro_state.argtypes = [C.POINTER(C.c_double)]
    lib.metro_state.restype = None
    lib.metro_disable.argtypes = []
    lib.metro_disable.restype = None
    lib.metro_error.argtypes = []
    lib.metro_error.restype = C.c_char_p
    lib.metro_task_name.argtypes = [C.c_int]
    lib.metro_task_name.restype = C.c_char_p
    lib.metro_now.argtypes = []
    lib.metro_now.restype = C.c_uint32
    return lib


def config_number(source, name):
    text = (Path(source) / "src/config.cpp").read_text()
    match = re.search(r"\bdouble\s+" + re.escape(name) + r"\s*=\s*([0-9.]+)\s*;", text)
    if not match:
        raise ValueError(f"Cannot resolve Metro configuration {name}")
    return float(match[1])


class MetroRecorder:
    def __init__(self, lib, world, *, start_pose, optical_forward_in=-1.8,
                 optical_height_in=10.0, optical_radius_in=2.0, drive_scale=1.0):
        self.lib, self.world, self.sim = lib, world, world.sim
        self.start_pose = start_pose
        self.odom_origin = [0.0, 0.0]
        self.previous_local_xy = [0.0, 0.0]
        self.drive_scale = drive_scale
        self.optical_region = (optical_forward_in, optical_height_in, optical_radius_in)
        self.now = 0
        self.events, self.frames, self.errors = [], [], []
        self.commands = [(0, 0.0), (0, 0.0)]
        self.outputs = dict(bottom=0, top=0, hood=False, flap=False, scraper=False, wing=False)
        self.pneumatics = {}
        self.last_imu = self.sim.imu.heading
        self.unwrapped_imu = self.last_imu
        self.initial_imu = self.last_imu
        self.sides = [[m for unit in units for m in unit.motors] for units in
                      (self.sim.chassis.left_units(), self.sim.chassis.right_units())]
        self.output_callback, self.event_callback = OUTPUT(self.output), EVENT(self.event)
        sample = self.sample()
        self.check(self.lib.metro_init(C.byref(sample), self.output_callback, self.event_callback))
        self.tick(0)

    def check(self, code):
        if code != 0 or self.errors:
            message = (self.lib.metro_error() or b"runtime failure").decode()
            raise RuntimeError("; ".join([message, *self.errors]))

    def event(self, kind, task, a, b):
        try:
            name = self.lib.metro_task_name(task)
            self.events.append(dict(t=self.now / 1000, type=EVENT_NAMES[kind], task_id=task,
                                    task=name.decode() if name else "", a=a, b=b))
        except Exception as error:
            self.errors.append(str(error))

    def output(self, kind, channel, mode, value):
        try:
            from vexsim.motor import BrakeMode
            if not math.isfinite(value):
                raise ValueError("Nonfinite Metro actuator output")
            if kind == 0:
                if channel not in (0, 1) or mode not in (0, 1, 2, 3):
                    raise ValueError("Invalid Metro drivetrain output")
                voltage = max(-12.0, min(12.0, value)) * getattr(self, "drive_scale", 1.0)
                changed = self.commands[channel] != (mode, voltage)
                self.commands[channel] = (mode, voltage)
                for motor in self.sides[channel]:
                    if mode == 0:
                        motor.set_voltage(voltage)
                    else:
                        motor.stop((BrakeMode.COAST, BrakeMode.BRAKE, BrakeMode.HOLD)[mode - 1])
                if changed:
                    self.events.append(dict(t=self.now / 1000, type="drive_output", side=channel,
                                            mode=mode, volts=voltage, requested_volts=value))
            elif kind == 1:
                key = ("bottom", "top")[channel]
                command = max(-127, min(127, int(value)))
                if self.outputs[key] != command:
                    self.events.append(dict(t=self.now / 1000, type="intake_output", motor=key, command=command))
                self.outputs[key] = command
            elif kind == 2:
                key = {65: "hood", 66: "flap", 68: "scraper", 69: "wing"}.get(channel)
                if self.pneumatics.get(channel) != bool(value):
                    self.events.append(dict(t=self.now / 1000, type="pneumatic_output",
                                            port=chr(channel), name=key, value=bool(value)))
                self.pneumatics[channel] = bool(value)
                if key:
                    self.outputs[key] = bool(value)
            else:
                raise ValueError(f"Unknown Metro output kind {kind}")
        except Exception as error:
            self.errors.append(str(error))

    def optical_sample(self):
        # Uncalibrated, declared sensor region. Read native block positions;
        # never schedule a synthetic clear merely to finish PRIME.
        forward, height, radius = self.optical_region
        sample = self.world.optical_sample(forward_in=forward, height_in=height,
                                          detection_radius_in=radius)
        return sample["proximity"], sample["hue"], sample["distance_mm"]

    def sample(self):
        heading = self.sim.imu.heading
        self.unwrapped_imu += math.remainder(heading - self.last_imu, 2 * math.pi)
        self.last_imu = heading
        values = {}
        for label, motors in zip(("left", "right"), self.sides):
            encoders = [self.sim.encoders[m.name] for m in motors]
            values[label + "_deg"] = sum(math.degrees(e.position) for e in encoders) / len(encoders)
            values[label + "_rpm"] = sum(e.velocity for e in encoders) * 30 / math.pi / len(encoders)
            values[label + "_current_ma"] = sum(abs(m.current) for m in motors) * 1000 / len(motors)
        proximity, hue, distance = self.optical_sample()
        return MetroSample(heading=-math.degrees(self.unwrapped_imu - self.initial_imu),
                           tracker_centideg=0, optical_proximity=proximity,
                           optical_hue=hue, distance_mm=distance, **values)

    def state(self):
        values = (C.c_double * 10)()
        self.lib.metro_state(values)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Metro runtime returned nonfinite telemetry")
        return list(values)

    def tick(self, millis):
        self.now = millis
        sample = self.sample()
        self.check(self.lib.metro_tick(millis, C.byref(sample)))
        state = self.state()
        if state[0] == 0 and state[1] == 0 and any(self.previous_local_xy):
            # This routine explicitly rebases xpos/ypos mid-run. Keep the
            # display in field coordinates without feeding truth into control.
            for axis in (0, 1):
                self.odom_origin[axis] += self.previous_local_xy[axis]
            self.events.append(dict(t=millis / 1000, type="odometry_origin_reset",
                                    origin_in=list(self.odom_origin)))
        self.previous_local_xy = state[:2]
        self.world.set_outputs(**self.outputs, state=INTAKE_STATES[int(state[7])])

    def capture(self):
        state = self.state()
        x, y, heading = state[:3]
        fx, fy = x + self.odom_origin[0], y + self.odom_origin[1]
        sx, sy, sh = self.start_pose
        angle = math.radians(sh)
        field_pose = (sx + fx * math.cos(angle) + fy * math.sin(angle),
                      sy - fx * math.sin(angle) + fy * math.cos(angle), sh + heading)
        telemetry = dict(local_pose_in_deg=[x, y, heading], heading_target_deg=state[3],
                         odometry_origin_in=list(self.odom_origin),
                         is_turning=bool(state[4]), previous_outputs_volts=state[5:7],
                         intake_state=INTAKE_STATES[int(state[7])], routine_returned=bool(state[8]),
                         routine_return_ms=state[9], motor_commands=dict(self.outputs),
                         optical_proximity=self.optical_sample()[0])
        left, right = (command[1] for command in self.commands)
        commands = dict(fwd=(left + right) / 2, turn=(right - left) / 2,
                        left_volts=left, right_volts=right,
                        left_mode=self.commands[0][0], right_mode=self.commands[1][0])
        frame = self.world.snapshot(field_pose, telemetry=telemetry, commands=commands)
        self.frames.append(frame)
        return frame

    def run(self, duration_ms=15000, capture_ms=10):
        self.capture()
        for millis in range(1, duration_ms + 1):
            self.world.step()
            if millis == duration_ms:
                self.now = millis
                self.lib.metro_disable()
                self.world.set_outputs(**self.outputs, state="DISABLED")
            else:
                self.tick(millis)
            if millis % capture_ms == 0 or millis == duration_ms:
                self.capture()
            if self.errors:
                raise RuntimeError("; ".join(self.errors))
        return self.frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metro", type=Path, required=True)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-x", type=float, default=-13.5)
    parser.add_argument("--start-y", type=float, default=-46.0)
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--alliance", choices=("red", "blue"), default="red")
    parser.add_argument("--preloads", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--duration-ms", type=int, default=15000)
    parser.add_argument("--wheel-diameter", type=float, default=3.25)
    parser.add_argument("--gear-ratio", type=float, default=48 / 36)
    parser.add_argument("--mass-lb", type=float, default=14.0,
                        help="Robot mass in pounds (default: user-estimated 14 lb)")
    parser.add_argument("--surface-mu", type=float, default=0.86,
                        help="Tire friction multiplier (default: 14%% below original friction)")
    parser.add_argument("--drive-scale", type=float, default=0.9,
                        help="Drivetrain voltage scale (default: 0.9, approximately 10%% slower)")
    parser.add_argument("--matchload-rate", type=float, default=2.0,
                        help="Loader pickup rate multiplier (default: twice the native rate)")
    parser.add_argument("--goal-stopping-rate", type=float, default=1.6,
                        help="Goal-contact stopping rate (default: 1.6x, 150%% longer sliding than previous 4x)")
    parser.add_argument("--optical-forward", type=float, default=-1.8)
    parser.add_argument("--optical-height", type=float, default=10.0)
    parser.add_argument("--optical-radius", type=float, default=2.0)
    args = parser.parse_args()
    for name, low, high in (("start_x", -60, 60), ("start_y", -60, 60), ("heading", -360, 360),
                            ("wheel_diameter", 2, 5), ("gear_ratio", .5, 3), ("mass_lb", 5, 40),
                            ("surface_mu", 0, 2),
                            ("drive_scale", .1, 1), ("matchload_rate", .1, 10),
                            ("goal_stopping_rate", 1, 10),
                            ("optical_forward", -10, 10), ("optical_height", 0, 20),
                            ("optical_radius", .1, 5), ("duration_ms", 1, 15000), ("preloads", 0, 9)):
        value = getattr(args, name)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f"{name} must be finite and in [{low}, {high}]")
    args.output = args.output.resolve()
    if (args.output / "recording.json").exists():
        parser.error("Choose a new output directory; recordings are not overwritten")
    sys.path.insert(0, str(args.vexsim.resolve()))
    from dataclasses import replace
    from vexsim import Simulator, presets
    from vexsim.sim import SimConfig
    from vexsim.units import INCH, LB
    from metro_game import MetroGame
    library, manifest = build(args.metro, args.output)
    chassis = presets.six_motor_450(mass_lb=args.mass_lb)
    chassis.surface_mu = args.surface_mu
    track = config_number(args.metro, "distance_between_wheels")
    for unit in chassis.wheel_units:
        unit.y = math.copysign(track * INCH / 2, unit.y)
        unit.gear_ratio = args.gear_ratio
        diameter = args.wheel_diameter * INCH
        unit.wheel = replace(unit.wheel, diameter=diameter,
                             inertia=unit.wheel.inertia * (diameter / unit.wheel.diameter) ** 2)
    sim = Simulator(chassis, config=SimConfig(log_every=0))
    sim.imu.seed = args.seed
    sim.imu._rng.seed(args.seed)
    sim.set_pose(args.start_y * INCH, -args.start_x * INCH, -math.radians(args.heading))
    assumptions = [
        "POINTS in metro_waypoints.py are editable local-odometry targets and default to the original Metro coordinates. With default points, the original autonomous.cpp is compiled directly. All non-coordinate autonomous code remains original. Controller corrections are listed separately; no physical truth is fed into steering.",
        f"Goal-contact stopping losses use {args.goal_stopping_rate:g}x the native rate (default 1.6x increases sliding duration by 150% relative to the previous 4x stopping rate). Only supported channel blocks receive increased sliding/rattle losses; airborne and floor motion retain native rates. Scenario tuning, not a measured material coefficient.",
        "Metro controller correction: preserve signed inner-wheel arc travel for tight curveCircle turns. Applied to a hashed build copy; original source snapshot is unchanged.",
        f"Physical start is ({args.start_x:g},{args.start_y:g}) inches, heading {args.heading:g} degrees, {args.alliance} alliance, {args.preloads} preload(s). Default X=-13.5 restores the corrected park-zone side after undoing the additional 8-inch shift; exact placement and hardware remain provisional.",
        f"Physical mass is {args.mass_lb:g} lb; default 14 lb is the user's approximate robot weight. Yaw inertia is derived from that mass and the native body dimensions, not measured mass distribution.",
        "Physical wheel diameter/gearing are unconfirmed native six_motor_450 assumptions; source track width is used. Original gains are retained; controller corrections are listed in metadata.",
        "Display odometry keeps an accumulated field origin when this routine explicitly resets local X/Y to zero. This affects visualization only; neither physical truth nor an offset is fed into the C++ controller.",
        "Scoring outputs map into native route directions: negative upper motor feeds the middle route; reverse feeds the front/low route; PRIME backs blocks away from the sensor with the gate closed. Goal entry is resolved by contact physics, not assigned by the output state.",
        f"Drivetrain commands use voltage scale {args.drive_scale:g}; 0.9 targets roughly 10% slower motion, not an exact trajectory/time scale. Native coast/brake/hold and momentum remain physical.",
        f"Loader pickup cooldown is {0.33 / args.matchload_rate:g} s ({args.matchload_rate:g}x native rate); floor intake remains 0.33 s. Actual throughput also depends on stack settling, alignment and capacity; no human station loads are spawned.",
        f"Used-tile friction is uncalibrated. Native omni coefficients (longitudinal peak/sliding 1.05/0.85, lateral 0.26/0.22) use surface multiplier {args.surface_mu:g}; default 0.86 reduces original friction by 14% at the user's request. This is a scenario setting, not a measured used-mat coefficient; see tests/vexsim/METRO_FRICTION.md.",
        "Metro drive-encoder trackNoOdomWheel uses corrected opposite-sign turning offsets so pure rotation does not create false translation. The declared vertical tracker is not used. Encoder tares occur only at startup.",
        "IMU begins at zero relative to physical starting heading; original Metro gain 0.9972299169 remains applied.",
        "Virtual tasks execute deterministically by creation ID (odometry, intake scheduler, autonomous, delayed tasks), not measured V5 preemptive scheduling.",
        "Optical/distance sensor placement is unknown: a declared upper-tower sphere tests real block positions, using proximity255/0 and red/blue hue0/210; not calibrated sensor optics.",
        "Native game mechanisms are an approximation; scraper/wing collision geometry and intake-motor electrical load are not simulated.",
        "Push Back 4.0 Appendix A competition layout: 12 L-cluster blocks, 8 under long goals, 8 at corners, 8 in opposing park zones; each loader has 3 near-alliance blocks below 3 opposing blocks. One robot plus configured preloads; other robots and station match loads are absent.",
    ]
    world = MetroGame(sim, alliance=args.alliance, seed=args.seed, preloads=args.preloads,
                      assumptions=assumptions)
    world.game.matchload_rate = args.matchload_rate
    world.game.goal_stopping_rate = args.goal_stopping_rate
    world.game.start()
    recorder = MetroRecorder(load_library(library), world,
                             start_pose=(args.start_x, args.start_y, args.heading),
                             optical_forward_in=args.optical_forward,
                             optical_height_in=args.optical_height,
                             optical_radius_in=args.optical_radius, drive_scale=args.drive_scale)
    started = time.monotonic()
    error = None
    try:
        recorder.run(args.duration_ms)
    except Exception as failure:
        error = str(failure)
        recorder.lib.metro_disable()
    metadata = dict(routine="leftSideSevenMiddle", source_commit=manifest["source_commit"],
                    controller_corrections=manifest["controller_corrections"],
                    tracking=dict(algorithm="trackNoOdomWheel",
                                  translation="sampled_left_right_drivetrain_motor_encoders",
                                  heading="simulated_imu_with_original_metro_gain",
                                  tracking_wheels=False),
                    source_repository=manifest["repository"], assumptions=assumptions,
                    physical_start_in_deg=[args.start_x, args.start_y, args.heading],
                    physical=dict(wheel_diameter_in=args.wheel_diameter,
                                  motor_turns_per_wheel_turn=args.gear_ratio,
                                  mass_lb=sim.chassis.mass / LB, mass_kg=sim.chassis.mass,
                                  inertia_z_kg_m2=sim.chassis.inertia_z,
                                  surface_mu=sim.chassis.surface_mu,
                                  drive_voltage_scale=args.drive_scale,
                                  matchload_rate=args.matchload_rate,
                                  goal_stopping_rate=args.goal_stopping_rate,
                                  friction_calibrated=False, track_width_in=track,
                                  encoder_inches_per_turn=math.pi * args.wheel_diameter / args.gear_ratio),
                    controller_encoder_inches_per_turn=config_number(args.metro, "wheel_distance_in"),
                    optical_region_in=[args.optical_forward, args.optical_height, args.optical_radius],
                    alliance=args.alliance, preloads=args.preloads, seed=args.seed,
                    sim_dt_ms=1, frame_dt_ms=10, requested_duration_ms=args.duration_ms,
                    acceptance="Illustrative native-game simulation, not calibrated robot/scoring validation")
    if adapter_manifest() != manifest["adapter_files"]:
        raise RuntimeError("Host adapter changed during simulation; recording provenance is invalid")
    provenance_paths = list((args.vexsim.resolve() / "vexsim").rglob("*.py"))
    metadata["runtime_source_sha256"] = {
        str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(provenance_paths) if path.is_file()}
    metadata["runtime_source_sha256"].update(manifest["adapter_files"])
    metadata["runtime_source_sha256"].update({c["compiled_path"]: c["sha256"]
                                              for c in manifest["controller_corrections"]})
    final_state = recorder.state()
    summary = dict(completed=error is None, error=error, routine_returned=bool(final_state[8]),
                   routine_return_ms=final_state[9], simulated_ms=round(sim.t * 1000),
                   frames=len(recorder.frames), blocks=len(world.game.blocks), held=len(world.game.held),
                   score={a: world.game.score(a) for a in ("red", "blue")},
                   wall_seconds=round(time.monotonic() - started, 3))
    recording = dict(schema="metro-game-recording-v1", metadata=metadata, frames=recorder.frames,
                     events=recorder.events, summary=summary)
    write_json(args.output / "recording.json", recording)
    write_json(args.output / "summary.json", summary)
    print(json.dumps(summary, indent=2))
    print(f"Recording: {args.output / 'recording.json'}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
