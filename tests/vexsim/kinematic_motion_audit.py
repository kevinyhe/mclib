#!/usr/bin/env python3
"""Run actual mclib motion.cpp against an independent ideal kinematic plant.

No vexsim modules are imported. Wheel speed is voltage/12 times an explicitly
selected 34, 76, or 94 in/s; track is 11.5 in and wheels are 3.25 in, ratio 1.
Every 1 ms integrates the exact constant-wheel SE(2) arc. Sensors are exact,
with no slip, lag, noise, inertia, electrical behavior, or contact model.
All stop modes set ideal wheel speed to zero immediately. These checks isolate
controller geometry and exits; they cannot validate real drivetrain tuning.
"""

import argparse
import csv
import ctypes as C
import json
import math
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
from run import ADVANCE, OUTPUT, build, scenarios

TRACK = 11.5
DIAMETER = 3.25
SPEEDS = {"four_motor_200": 34.0, "six_motor_450": 76.0, "speed_base": 94.0}


def exact_step(x, y, heading, left_distance, right_distance):
    """mclib frame: x right, y forward, clockwise heading from +y."""
    angle = (left_distance - right_distance) / TRACK
    half = angle / 2.0
    chord = (left_distance + right_distance) / 2.0
    if abs(half) > 1e-12:
        chord *= math.sin(half) / half
    return (x + chord * math.sin(heading + half),
            y + chord * math.cos(heading + half), heading + angle)


class IdealBridge:
    def __init__(self, lib, speed):
        self.lib, self.speed = lib, speed
        self.x = self.y = self.heading = 0.0
        self.left_distance = self.right_distance = 0.0
        self.millis = 0
        self.commands = [(1, 0.0), (1, 0.0)]
        self.exceptions, self.trace = [], []
        self.peak_voltage = 0.0
        self.advance_callback, self.output_callback = ADVANCE(self.advance), OUTPUT(self.write)
        lib.sim_init(self.advance_callback, self.output_callback,
                     DIAMETER, TRACK, 1.0, 0)

    def write(self, side, mode, volts):
        try:
            if side not in (0, 1) or mode not in (0, 1, 2, 3) or not math.isfinite(volts):
                raise ValueError("invalid output command")
            volts = math.trunc(max(-12.0, min(12.0, volts)) * 1000) / 1000
            self.commands[side] = (mode, volts)
            self.peak_voltage = max(self.peak_voltage, abs(volts))
        except Exception as error:
            self.exceptions.append(str(error))
            self.commands = [(2, 0.0), (2, 0.0)]

    def record(self):
        estimate = self.pose()
        self.trace.append({"t_s": self.millis / 1000, "true_x_in": self.x,
                           "true_y_in": self.y, "true_heading_deg": math.degrees(self.heading),
                           "odom_x_in": estimate[0], "odom_y_in": estimate[1],
                           "odom_heading_deg": estimate[2],
                           "left_mode": self.commands[0][0], "left_V": self.commands[0][1],
                           "right_mode": self.commands[1][0], "right_V": self.commands[1][1]})

    def advance(self, millis, sample):
        try:
            if self.millis + millis > 20000:
                raise RuntimeError("controller exceeded 20 s guard")
            if millis:
                self.record()
            left, right = [self.speed * volts / 12 if mode == 0 else 0.0
                           for mode, volts in self.commands]
            for _ in range(millis):
                dl, dr = left * 0.001, right * 0.001
                self.x, self.y, self.heading = exact_step(self.x, self.y, self.heading, dl, dr)
                self.left_distance += dl
                self.right_distance += dr
            self.millis += millis
            reading = sample.contents
            reading.heading = math.degrees(self.heading)
            reading.left = self.left_distance * 360 / (math.pi * DIAMETER)
            reading.right = self.right_distance * 360 / (math.pi * DIAMETER)
            reading.current_ma = 0.0  # Electrical behavior is deliberately absent.
            reading.velocity_rpm = (abs(left) + abs(right)) / 2 / (math.pi * DIAMETER) * 60
            reading.millis = self.millis
            reading.disabled = 0
            reading.cancel = bool(self.exceptions)
        except Exception as error:
            self.exceptions.append(str(error))
            sample.contents.cancel = 1
            sample.contents.heading = math.nan

    def pose(self):
        value = (C.c_double * 3)()
        self.lib.sim_pose(value)
        return list(value)

    def stopped(self):
        return all(mode != 0 or volts == 0 for mode, volts in self.commands)

    def run(self, action, a=0, b=0, heading=0, timeout=4000, direction=1,
            stop=True, volts=12, current=2500):
        result = self.lib.sim_run(action, a, b, heading, timeout, direction, stop, volts, current)
        self.record()
        if self.exceptions:
            raise RuntimeError("; ".join(self.exceptions))
        return result


def verify_plant():
    straight = exact_step(0, 0, 0, 24, 24)
    assert max(abs(a-b) for a, b in zip(straight, (0, 24, 0))) < 1e-12
    angle, radius = math.pi / 2, 24.0
    curve = exact_step(0, 0, 0, (radius+TRACK/2)*angle, (radius-TRACK/2)*angle)
    assert max(abs(a-b) for a, b in zip(curve, (24, 24, angle))) < 1e-12
    turn = exact_step(0, 0, 0, TRACK/2*angle, -TRACK/2*angle)
    assert max(abs(a-b) for a, b in zip(turn, (0, 0, angle))) < 1e-12


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--filter", default="")
    args = parser.parse_args()
    verify_plant()
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-kinematic-motion-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    lib = build(destination)
    cases = [case for case in scenarios() if case["name"].split("/")[0] in SPEEDS]
    for preset in SPEEDS:
        cases.append(dict(name=f"{preset}/point_forward_10s", preset=preset, action=6,
                          args={"b": 24, "timeout": 10000}, target=(0, 24), angle=0,
                          position_tolerance=2.5))
    rows = []
    for case in cases:
        if args.filter not in case["name"]:
            continue
        plant = IdealBridge(lib, SPEEDS[case["preset"]])
        error = None
        try:
            plant.run(case["action"], **case["args"])
        except RuntimeError as failure:
            error = str(failure)
        target = case.get("target")
        position_error = math.hypot(plant.x-target[0], plant.y-target[1]) if target else None
        angle = case.get("angle")
        heading_error = abs(math.remainder(math.degrees(plant.heading)-angle, 360)) if angle is not None else None
        deadline = plant.millis < case["args"].get("timeout", 4000)
        estimate = plant.pose()
        odom_error = math.hypot(estimate[0]-plant.x, estimate[1]-plant.y)
        # Last nonzero voltage interval, excluding the final braking command.
        last_active_end = 0.0
        for before, after in zip(plant.trace, plant.trace[1:]):
            if any(before[f"{side}_mode"] == 0 and before[f"{side}_V"] != 0
                   for side in ("left", "right")):
                last_active_end = after["t_s"]
        zero_tail = plant.millis / 1000 - last_active_end
        passed = (error is None and plant.stopped() and deadline
                  and odom_error < 1e-8
                  and (position_error is None or position_error <= case["position_tolerance"])
                  and (heading_error is None or heading_error <= 5.0))
        row = dict(name=case["name"], passed=passed, max_speed_ips=plant.speed,
                   elapsed_s=plant.millis/1000, deadline_met=deadline,
                   timeout_s=case["args"].get("timeout", 4000)/1000,
                   position_error_in=position_error, heading_error_deg=heading_error,
                   truth=[plant.x, plant.y, math.degrees(plant.heading)], odometry=estimate,
                   odometry_error_in=odom_error, stopped=plant.stopped(),
                   trailing_zero_output_s=zero_tail, peak_voltage=plant.peak_voltage,
                   final_commands=plant.commands, error=error)
        rows.append(row)
        trace_name = case["name"].replace("/", "__") + ".csv"
        with (destination / trace_name).open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(plant.trace[0]))
            writer.writeheader()
            writer.writerows(plant.trace)
        print(f"{'PASS' if passed else 'FAIL'} {case['name']}: "
              f"t={row['elapsed_s']:.3f}s pos={position_error} heading={heading_error} "
              f"zero_tail={zero_tail:.3f}s odom={odom_error:.3g}in", flush=True)
    if not rows:
        parser.error("no scenarios matched --filter")
    with (destination / "results.json").open("w") as handle:
        json.dump({"plant": __doc__, "track_in": TRACK, "wheel_diameter_in": DIAMETER,
                   "rows": rows}, handle, indent=2)
    failed = sum(not row["passed"] for row in rows)
    print(f"{len(rows)-failed} passed, {failed} failed; {destination}", flush=True)
    return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
