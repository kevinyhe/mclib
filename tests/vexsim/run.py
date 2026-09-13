#!/usr/bin/env python3
"""Run compiled mclib motion.cpp against the sibling vexsim physics engine.

No Python motion/PID implementations are used. All controller state, PID updates,
motion exits and odometry integration run in the compiled C++ library.
"""
import argparse
import csv
import ctypes as C
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TUNING = dict(drive_kp=.4, drive_ki=0., drive_kd=3.,
                      heading_kp=.3, heading_ki=0., heading_kd=1.5,
                      turn_kp=.3, turn_ki=0., turn_kd=1.5)
CONTROLLER_PHASES = ("idle", "drive", "turn", "arc", "swing", "point", "pursuit",
                     "pivot", "final_align", "decelerate", "wall")
CONTROLLER_FIELDS = ("target_heading_deg", "heading_error_deg", "remaining_in",
                     "carrot_x", "carrot_y", "drive_volts", "yaw_volts",
                     "slew_limited", "voltage_limited")


class Sample(C.Structure):
    _fields_ = [(name, C.c_double) for name in
                ("heading", "left", "right", "current_ma", "velocity_rpm")]
    _fields_ += [("millis", C.c_uint32), ("disabled", C.c_int), ("cancel", C.c_int)]
    _fields_ += [("vertical", C.c_double), ("horizontal", C.c_double)]


ADVANCE = C.CFUNCTYPE(None, C.c_uint32, C.POINTER(Sample))
OUTPUT = C.CFUNCTYPE(None, C.c_int, C.c_int, C.c_double)


def build(destination):
    sources = ["control/motion", "control/chassis_io", "control/motion_config",
               "control/motion_math", "control/scaling", "control/robot_state",
               "control/odometry", "chassis/chassis_math", "pid", "math", "utils"]
    library = destination / "libmclib_vexsim.so"
    command = ["g++", "-std=gnu++20", "-O1", "-g", "-shared", "-fPIC",
               "-DMCLIB_HOST_BUILD", "-Wno-deprecated-declarations", "-Iinclude",
               "tests/vexsim/bridge.cpp"]
    command += [f"src/mclib/{source}.cpp" for source in sources]
    command += ["-pthread", "-Wl,-z,defs", "-o", str(library)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    (destination / "build.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr)
    return load_library(library)


def load_library(library):
    """Load a compiled artifact; each concurrent run must use its own process."""
    lib = C.CDLL(str(library))
    lib.sim_init.argtypes = [ADVANCE, OUTPUT, C.c_double, C.c_double, C.c_double, C.c_int]
    lib.sim_init.restype = None
    lib.sim_run.argtypes = [C.c_int] + [C.c_double] * 4 + [C.c_int] * 2 + [C.c_double] * 2
    lib.sim_run.restype = C.c_int
    lib.sim_pose.argtypes = [C.POINTER(C.c_double)]
    lib.sim_pose.restype = None
    lib.sim_correct.argtypes = [C.c_double, C.c_double]
    lib.sim_correct.restype = None
    lib.sim_last_integrated_pose.argtypes = [C.POINTER(C.c_double)]
    lib.sim_last_integrated_pose.restype = None
    lib.sim_refresh.argtypes = [C.c_uint32]
    lib.sim_refresh.restype = None
    lib.sim_set_pose.argtypes = [C.c_double] * 3
    lib.sim_set_pose.restype = None
    lib.sim_set_trackers.argtypes = [C.c_double] * 4
    lib.sim_set_trackers.restype = None
    lib.sim_set_gains.argtypes = [C.POINTER(C.c_double)]
    lib.sim_set_gains.restype = None
    lib.sim_set_lead.argtypes = [C.c_double]
    lib.sim_set_lead.restype = None
    lib.sim_heading_target.argtypes = []
    lib.sim_heading_target.restype = C.c_double
    if hasattr(lib, "sim_motion_telemetry"):
        lib.sim_motion_telemetry.argtypes = [C.POINTER(C.c_double)]
        lib.sim_motion_telemetry.restype = None
    return lib


class PhysicsBridge:
    def __init__(self, lib, preset, soc=1.0, friction=1.0, fault=None, locked=False, constrained=False,
                 encoder_heading=False, seed=1, tracking_mode="drive", tuning=None,
                 start_pose=(0.0, 0.0, 0.0), sim_time_limit=120.0):
        from vexsim import Simulator, presets
        from vexsim.chassis import add_tracking_wheels
        from vexsim.motor import BrakeMode
        from vexsim.sim import SimConfig
        from vexsim.units import INCH
        self.lib, self.inch = lib, INCH
        chassis = presets.get(preset)
        if tracking_mode not in ("drive", "two"):
            raise ValueError("tracking_mode must be drive or two")
        self.tracking_mode = tracking_mode
        self.sim_time_limit = float(sim_time_limit)
        if not math.isfinite(self.sim_time_limit) or not 0 < self.sim_time_limit <= 120:
            raise ValueError("sim_time_limit must be in (0, 120] seconds")
        if tracking_mode == "two" and not any(u.name == "track_parallel" for u in chassis.wheel_units):
            add_tracking_wheels(chassis, forward_offset_in=-3.0)
        chassis.surface_mu = friction
        self.sim = Simulator(chassis, config=SimConfig(log_every=10))
        self.sim.battery.reset(soc)
        self.sim.imu.seed = seed
        self.sim.imu._rng.seed(seed)
        self.sim.dynamics.locked = locked
        self.sim.dynamics.constrained = constrained
        self.sides = [[m for unit in units for m in unit.motors] for units in
                      (chassis.left_units(), chassis.right_units())]
        self.modes = [BrakeMode.COAST, BrakeMode.BRAKE, BrakeMode.HOLD]
        self.fault = fault
        self.fault_injected = False
        self.exceptions = []
        self.control_trace = []
        # Observer only: publishing a snapshot must never advance simulation time
        # or substitute simulator truth for the sampled controller inputs.
        self.on_capture = None
        self.commands = [(1, 0), (1, 0)]
        self.peak_command = 0
        self.last_heading = self.unwrapped_heading = 0.0
        self.step_callback = ADVANCE(self.advance)
        self.output_callback = OUTPUT(self.write)
        wheel = chassis.left_units()[0]
        lib.sim_init(self.step_callback, self.output_callback, 2 * wheel.radius / INCH,
                     chassis.track_width / INCH, 1.0 / wheel.gear_ratio, encoder_heading)
        if tracking_mode == "two":
            trackers = {u.name: u for u in chassis.wheel_units if u.name.startswith("track_")}
            vertical, horizontal = trackers["track_parallel"], trackers["track_perp"]
            lib.sim_set_trackers(2 * math.pi * vertical.radius / INCH,
                                 -(vertical.y - chassis.cg_y) / INCH,
                                 2 * math.pi * horizontal.radius / INCH,
                                 (horizontal.x - chassis.cg_x) / INCH)
        if tuning:
            self.set_tuning(tuning)
        if tuple(start_pose) != (0.0, 0.0, 0.0):
            self.set_pose(*start_pose)

    def set_tuning(self, tuning):
        if not isinstance(tuning, dict) or set(tuning) - DEFAULT_TUNING.keys():
            raise ValueError("unknown tuning keys")
        merged = dict(DEFAULT_TUNING, **tuning)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
               not math.isfinite(value) or not 0 <= value <= 100 for value in merged.values()):
            raise ValueError("PID gains must be finite values in [0, 100]")
        self.lib.sim_set_gains((C.c_double * 9)(*merged.values()))

    def set_pose(self, x, y, heading):
        if not all(math.isfinite(v) for v in (x, y, heading)):
            raise ValueError("start pose must be finite")
        self.sim.set_pose(y * self.inch, -x * self.inch, -math.radians(heading))
        self.last_heading = self.unwrapped_heading = -math.radians(heading)
        self.lib.sim_set_pose(x, y, heading)

    def capture(self):
        estimate, truth = self.pose(), self.truth()
        frame = dict(t=self.sim.t, estimated_x=estimate[0], estimated_y=estimate[1],
                    estimated_heading=estimate[2], true_x=truth[0], true_y=truth[1],
                    true_heading=truth[2], left_mode=self.commands[0][0],
                    left_volts=self.commands[0][1], right_mode=self.commands[1][0],
                    right_volts=self.commands[1][1], battery_volts=self.sim.battery.voltage,
                    battery_amps=self.sim.battery.current,
                    speed_ips=self.sim.state.speed / self.inch,
                    odometry_error_in=math.dist(estimate[:2], truth[:2]))
        # Body velocities and IMU use mclib's right-positive / clockwise frame.
        # Motor encoder positions are raw (not controller-tared), side RPM is
        # the mean sampled encoder RPM, current is summed winding magnitude,
        # and temperature is the hottest motor on each side.
        frame.update(forward_speed_ips=self.sim.state.u / self.inch,
                     lateral_speed_ips=-self.sim.state.v / self.inch,
                     yaw_rate_dps=-math.degrees(self.sim.state.r),
                     imu_heading_deg=-math.degrees(self.sim.imu.heading),
                     imu_rate_dps=-math.degrees(self.sim.imu.rate),
                     body_length_in=self.sim.chassis.length / self.inch,
                     body_width_in=self.sim.chassis.width / self.inch)
        for label, units in (("left", self.sim.chassis.left_units()),
                             ("right", self.sim.chassis.right_units())):
            frame[f"{label}_wheel_angle_rad"] = sum(u.angle for u in units) / len(units)
        for label, motors in zip(("left", "right"), self.sides):
            encoders = [self.sim.encoders[m.name] for m in motors]
            frame.update({f"{label}_encoder_deg": sum(math.degrees(e.position) for e in encoders) / len(encoders),
                          f"{label}_encoder_rpm": sum(e.velocity for e in encoders) * 30 / math.pi / len(encoders),
                          f"{label}_current_amps": sum(abs(m.current) for m in motors),
                          f"{label}_temp_c": max(m.temperature for m in motors)})
        for label, name, sign in (("parallel", "track_parallel", 1),
                                  ("perpendicular", "track_perp", -1)):
            unit = next((u for u in self.sim.chassis.wheel_units if u.name == name), None)
            frame[f"{label}_tracker_in"] = (sign * self.sim.tracking[name].position * unit.radius / self.inch
                                            if self.tracking_mode == "two" and unit is not None else None)
        values = (C.c_double * 10)(*[math.nan] * 10)
        if hasattr(self.lib, "sim_motion_telemetry"):
            self.lib.sim_motion_telemetry(values)
        frame["controller_phase"] = (CONTROLLER_PHASES[int(values[0])]
                                     if math.isfinite(values[0]) and values[0].is_integer()
                                     and 0 <= values[0] < len(CONTROLLER_PHASES) else None)
        for field, value in zip(CONTROLLER_FIELDS, values[1:]):
            frame[f"controller_{field}"] = (None if not math.isfinite(value) else
                                            bool(value) if field.endswith("limited") else value)
        return frame

    def advance_idle(self, millis):
        """Advance powered/braked physics with real sensor-based C++ odometry."""
        if not isinstance(millis, int) or not 0 <= millis <= 120000:
            raise ValueError("idle duration must be 0..120000 integer milliseconds")
        for offset in range(0, millis, 10):
            self.lib.sim_refresh(min(10, millis - offset))
        if self.exceptions:
            raise RuntimeError("; ".join(self.exceptions))

    def write(self, side, mode, volts):
        try:
            if not math.isfinite(volts):
                raise ValueError(f"invalid motor voltage {volts}")
            # Match MotorGroup::setVoltage: clamp to the rail, then integer mV.
            volts = math.trunc(max(-12.0, min(12.0, volts)) * 1000) / 1000
            self.commands[side] = (mode, volts)
            self.peak_command = max(self.peak_command, abs(volts))
            for motor in self.sides[side]:
                if mode == 0:
                    motor.set_voltage(volts)
                else:
                    motor.stop(self.modes[mode - 1])
        except Exception as error:
            self.exceptions.append(str(error))
            self.sim.stop_all(self.modes[1])

    def advance(self, millis, sample):
        try:
            if millis:
                self.control_trace.append(self.capture())
                if self.on_capture is not None:
                    self.on_capture(self.control_trace[-1])
            for _ in range(millis):
                self.sim.step(0.001)
            heading = self.sim.imu.heading
            self.unwrapped_heading += math.remainder(heading - self.last_heading, 2 * math.pi)
            self.last_heading = heading
            reading = sample.contents
            reading.millis = round(self.sim.t * 1000)
            reading.heading = -math.degrees(self.unwrapped_heading)
            for side, name in zip(self.sides, ("left", "right")):
                setattr(reading, name, sum(math.degrees(self.sim.encoders[m.name].position)
                                          for m in side) / len(side))
            if self.tracking_mode == "two":
                reading.vertical = math.degrees(self.sim.tracking["track_parallel"].position)
                # The perpendicular wheel rolls left-positive in vexsim;
                # mclib's horizontal tracking measurement is right-positive.
                reading.horizontal = -math.degrees(self.sim.tracking["track_perp"].position)
            reading.current_ma = sum(abs(m.current) for m in self.sim.motors) * 1000 / len(self.sim.motors)
            reading.velocity_rpm = sum(abs(self.sim.encoders[m.name].velocity)
                                       for m in self.sim.motors) * 30 / math.pi / len(self.sim.motors)
            triggered = self.sim.t >= 0.25
            self.fault_injected |= triggered and self.fault is not None
            reading.disabled = triggered and self.fault == "disabled"
            reading.cancel = (triggered and self.fault == "cancel") or bool(self.exceptions)
            if triggered and self.fault == "imu":
                reading.heading = math.nan
            if triggered and self.fault == "encoder":
                reading.left = math.nan
            if self.sim.t > self.sim_time_limit:
                raise RuntimeError("motion exceeded its simulated-time guard")
        except Exception as error:
            self.exceptions.append(str(error))
            sample.contents.cancel = 1
            sample.contents.heading = math.nan

    def run(self, action, a=0, b=0, heading=0, timeout=4000, direction=1,
            stop=True, volts=12, current=2500, lead=0.5):
        if not math.isfinite(lead) or not 0 <= lead < 1:
            raise ValueError("boomerang lead must be in [0, 1)")
        self.lib.sim_set_lead(lead)
        result = self.lib.sim_run(action, a, b, heading, timeout, direction, stop, volts, current)
        if self.exceptions:
            raise RuntimeError("; ".join(self.exceptions))
        return result

    def pose(self):
        pose = (C.c_double * 3)()
        self.lib.sim_pose(pose)
        return list(pose)

    def commanded_heading(self):
        """Read the actual C++ heading target, including safety-abort resets."""
        return self.lib.sim_heading_target()

    def last_integrated_pose(self):
        pose = (C.c_double * 3)()
        self.lib.sim_last_integrated_pose(pose)
        return list(pose)

    def truth(self):
        state = self.sim.state
        # vexsim: +X forward, +Y left, CCW. mclib: +Y forward, +X right, CW.
        return [-state.y / self.inch, state.x / self.inch, -math.degrees(state.theta)]

    def stopped(self):
        return all(mode != 0 or volts == 0 for mode, volts in self.commands)


def scenarios():
    for preset in ("four_motor_200", "six_motor_450", "speed_base"):
        for name, action, args, target, angle in (
            ("drive_forward", 1, {"a": 24}, (0, 24), 0),
            ("drive_reverse", 1, {"a": -24}, (0, -24), 0),
            ("turn_right", 0, {"a": 90}, None, 90),
            ("turn_left", 0, {"a": -90}, None, -90),
            ("turn_to_point", 5, {"a": 24, "b": 24}, None, 45),
            ("point_forward", 6, {"b": 24}, (0, 24), 0),
            ("point_diagonal", 6, {"a": 24, "b": 24}, (24, 24), None),
            ("boomerang", 7, {"a": 24, "b": 24, "heading": 90}, (24, 24), 90),
            ("arc", 2, {"a": 90, "b": 24}, (24, 24), 90),
            ("reverse_arc", 3, {"a": -90, "b": 24}, (24, -24), -90),
            ("swing", 4, {"a": 90}, None, 90),
        ):
            yield dict(name=f"{preset}/{name}", preset=preset, action=action,
                       args=args, target=target, angle=angle,
                       position_tolerance=(1.5 if action == 7 else
                                           5.5 if action in (2, 3) else 2.5))
    for name, options in (("low_battery", {"soc": 0.08}),
                          ("low_grip", {"friction": 0.65}),
                          ("encoder_heading", {"encoder_heading": True})):
        yield dict(name=f"stress/{name}", preset="six_motor_450", action=0,
                   args={"a": 90}, angle=90, options=options)
    for fault in ("cancel", "disabled", "imu", "encoder"):
        for action in (0, 1, 2, 3, 4, 5, 6, 7):
            yield dict(name=f"safety/{fault}/{action}", preset="six_motor_450",
                       action=action, args={"a": 60, "b": 24, "stop": False},
                       options={"fault": fault}, safety=True)
    for action in range(8):
        yield dict(name=f"safety/timeout/{action}", preset="six_motor_450",
                   action=action, args={"a": 60, "b": 24, "stop": False, "timeout": 200},
                   timeout_safety=True)
    for name, options, threshold, expected in (
        ("no_contact", {}, 2500, False),
        ("locked_default_threshold", {"locked": True}, 2500, True),
        ("locked_2000mA_threshold", {"locked": True}, 2000, True),
        ("body_pinned_wheels_free", {"constrained": True}, 2000, False),
    ):
        yield dict(name=f"wall/{name}", preset="six_motor_450", action=8,
                   args={"a": 50, "b": 60, "heading": 0, "timeout": 800,
                         "volts": 6, "current": threshold}, options=options,
                   wall_expected=expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--filter", default="", help="Run names containing this substring")
    parser.add_argument("--tracking-mode", choices=("drive", "two"), default="drive",
                        help="Drive encoders (baseline) or two modeled passive tracking wheels")
    parser.add_argument("--motion-timeout-ms", type=int, default=4000,
                        help="Deadline for nominal motion cases (default: 4000)")
    args = parser.parse_args()
    cases = [case for case in scenarios() if args.filter in case["name"]]
    if not cases:
        parser.error("no scenarios matched --filter")
    if not 0 < args.motion_timeout_ms <= 20000:
        parser.error("--motion-timeout-ms must be between 1 and 20000")
    for case in cases:
        if not case.get("safety") and not case.get("timeout_safety") and "wall_expected" not in case:
            case["args"]["timeout"] = args.motion_timeout_ms
    sys.path.insert(0, str(args.vexsim.resolve()))
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-vexsim-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    lib = build(destination)
    rows = []
    for case in cases:
        bridge = PhysicsBridge(lib, case["preset"], tracking_mode=args.tracking_mode,
                               **case.get("options", {}))
        error = None
        try:
            wall_result = bridge.run(case["action"], **case["args"])
        except RuntimeError as failure:
            error = str(failure)
            wall_result = 0
        elapsed = bridge.sim.t
        estimated_at_exit = bridge.pose()
        truth_at_exit = bridge.truth()
        # Include actual coast/hold drift after the control routine returns.
        for _ in range(300):
            bridge.sim.step(0.001)
        truth = bridge.truth()
        position_error = (math.hypot(truth[0] - case["target"][0], truth[1] - case["target"][1])
                          if case.get("target") else None)
        heading_error = (abs(math.remainder(truth[2] - case["angle"], 360))
                         if case.get("angle") is not None else None)
        met_deadline = elapsed < case["args"].get("timeout", 4000) / 1000 - 0.0005
        passed = bridge.stopped() and error is None
        if not case.get("safety") and not case.get("timeout_safety") and "wall_expected" not in case:
            passed &= met_deadline
        if position_error is not None:
            passed &= position_error <= case.get("position_tolerance", 2.5)
        if heading_error is not None:
            passed &= heading_error <= 5
        if case.get("safety"):
            passed &= bridge.fault_injected and 0.25 <= elapsed <= 0.27 and bridge.peak_command > 0
        if case.get("timeout_safety"):
            passed &= 0.20 <= elapsed <= 0.22 and bridge.peak_command > 0
        if "wall_expected" in case:
            passed &= bool(wall_result) == case["wall_expected"]
            if not wall_result:
                passed &= all(abs(a - b) < 1e-9 for a, b in
                              zip(estimated_at_exit, bridge.last_integrated_pose()))
            else:
                passed &= all(abs(a - b) < 1e-9 for a, b in
                              zip(estimated_at_exit, (50, 60, 0)))
        row = dict(name=case["name"], passed=bool(passed), elapsed=round(elapsed, 3),
                   tracking_mode=args.tracking_mode,
                   true_pose=truth, estimated_pose_at_exit=estimated_at_exit,
                   true_pose_at_exit=truth_at_exit,
                   odometry_error_at_exit_in=math.dist(estimated_at_exit[:2], truth_at_exit[:2]),
                   position_error_in=position_error, heading_error_deg=heading_error,
                   stopped=bridge.stopped(), peak_command_V=bridge.peak_command,
                   met_deadline=met_deadline, error=error,
                   wall_success=bool(wall_result) if case["action"] == 8 else None)
        rows.append(row)
        stem = case["name"].replace("/", "_")
        bridge.sim.log.to_csv(str(destination / (stem + ".csv")))
        if bridge.control_trace:
            with (destination / (stem + "_control.csv")).open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(bridge.control_trace[0]))
                writer.writeheader()
                writer.writerows(bridge.control_trace)
        print(f"{'PASS' if passed else 'FAIL'} {case['name']}: "
              f"{elapsed:.2f}s position_error={position_error} heading_error={heading_error}", flush=True)
    (destination / "results.json").write_text(json.dumps(rows, indent=2, allow_nan=False) + "\n")
    failed = sum(not row["passed"] for row in rows)
    print(f"{len(rows) - failed}/{len(rows)} scenarios passed. Results: {destination / 'results.json'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
