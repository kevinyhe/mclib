#!/usr/bin/env python3
"""Check moving C++ boomerang handoffs on an independent exact ideal plant.

Every motion starts at the origin, travels 24 inches straight or diagonally,
and requests exit=false. Both directions and diagonal reflections must enter
the configured 1.5-inch position band, return before six seconds, and preserve
a live command in the travel direction. Chaining does not promise final heading
settlement or exact approach-plane crossing. A positional PID may approach the
plane asymptotically; the existing position band defines the handoff tolerance.

Only test yaw gains are changed: kP = 5 / (2*speed/(12*track)*180/pi), kD = 0.
Distance gains, chaining flags, and the API's resolved default voltage floor
remain unchanged. No new minimum-speed value is supplied. The exact plant has
no inertia, slip, noise, sensor delay, or electrical model; these checks do not
establish physical drivetrain tuning or traction performance.
"""

import argparse
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
from kinematic_motion_audit import IdealBridge, SPEEDS, TRACK, verify_plant
from run import DEFAULT_TUNING, build, load_library


TIMEOUT_MS = 6000
LEAD = 0.5
POSITION_BAND_IN = 1.5
PLANE_EPSILON_IN = 1e-9


def cases():
    for name, x, y, heading in (("straight", 0, 24, 0),
                                 ("diagonal", 24, 24, 90)):
        for mirror in ((1,) if x == 0 else (1, -1)):
            for direction in (1, -1):
                yield dict(name=f"{name}_mirror{mirror:+d}_dir{direction:+d}",
                           x=x * mirror * direction, y=y * direction,
                           heading=heading * mirror, direction=direction)


def annotate_trace(trace, case):
    approach = math.radians(case["heading"] + (180 if case["direction"] < 0 else 0))
    previous = None
    for row in trace:
        dx, dy = row["true_x_in"] - case["x"], row["true_y_in"] - case["y"]
        row["remaining_in"] = math.hypot(dx, dy)
        row["along_in"] = dx * math.sin(approach) + dy * math.cos(approach)
        row["mean_command_V"] = sum(
            row[f"{side}_V"] if row[f"{side}_mode"] == 0 else 0.0
            for side in ("left", "right")) / 2
        row["travel_projected_displacement_in"] = 0.0
        if previous is not None:
            midpoint = math.radians((row["true_heading_deg"] + previous["true_heading_deg"]) / 2)
            row["travel_projected_displacement_in"] = case["direction"] * (
                (row["true_x_in"] - previous["true_x_in"]) * math.sin(midpoint)
                + (row["true_y_in"] - previous["true_y_in"]) * math.cos(midpoint))
        previous = row


def run_case(lib, speed, case):
    plant = IdealBridge(lib, speed)
    kp = 5 / (2 * speed / (12 * TRACK) * 180 / math.pi)
    gains = dict(DEFAULT_TUNING)
    for prefix in ("heading", "turn"):
        gains.update({f"{prefix}_kp": kp, f"{prefix}_ki": 0, f"{prefix}_kd": 0})
    lib.sim_set_gains((C.c_double * 9)(*gains.values()))
    lib.sim_set_lead(LEAD)
    issues = []
    try:
        plant.run(7, a=case["x"], b=case["y"], heading=case["heading"],
                  direction=case["direction"], stop=False, timeout=TIMEOUT_MS)
    except RuntimeError as error:
        issues.append(str(error))
    annotate_trace(plant.trace, case)
    final = plant.trace[-1]
    position_error, along = final["remaining_in"], final["along_in"]
    odometry_error = max(math.hypot(row["odom_x_in"] - row["true_x_in"],
                                    row["odom_y_in"] - row["true_y_in"])
                         for row in plant.trace)
    odometry_heading_error = max(abs(math.remainder(row["odom_heading_deg"]
                                                    - row["true_heading_deg"], 360))
                                 for row in plant.trace)
    travel_progress = sum(row["travel_projected_displacement_in"] for row in plant.trace)
    if plant.millis >= TIMEOUT_MS:
        issues.append("motion reached its deadline")
    if not math.isfinite(position_error) or position_error > POSITION_BAND_IN + 1e-8:
        issues.append("handoff outside configured 1.5 inch position band")
    if not math.isfinite(travel_progress) or travel_progress <= 0:
        issues.append("motion made no net progress in the selected travel direction")
    if plant.stopped() or any(mode != 0 for mode, _ in plant.commands):
        issues.append("handoff did not preserve active voltage control")
    if case["direction"] * final["mean_command_V"] <= 0:
        issues.append("handoff did not preserve translation in the selected direction")
    if not math.isfinite(odometry_error) or odometry_error > 1e-8:
        issues.append("exact sensor odometry disagrees with exact plant position")
    if not math.isfinite(odometry_heading_error) or odometry_heading_error > 1e-8:
        issues.append("exact sensor odometry disagrees with exact plant heading")
    first_inside = next((row["t_s"] * 1000 for row in plant.trace
                         if row["remaining_in"] <= POSITION_BAND_IN), None)
    return plant.trace, dict(
        case=case["name"], configuration=case, speed_ips=speed,
        yaw_kp=kp, yaw_kd=0,
        distance_gains=[gains[f"drive_{term}"] for term in ("kp", "ki", "kd")],
        passed=not issues, elapsed_ms=plant.millis,
        position_error_in=position_error, along_in=along,
        travel_projected_progress_in=travel_progress,
        first_inside_position_band_ms=first_inside,
        ever_crossed_plane=any(row["along_in"] >= -PLANE_EPSILON_IN for row in plant.trace),
        odometry_error_in=odometry_error, odometry_heading_error_deg=odometry_heading_error,
        pose=[plant.x, plant.y, math.degrees(plant.heading)],
        commands=plant.commands, mean_command_V=final["mean_command_V"],
        stopped=plant.stopped(), issues=issues)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--library", type=Path,
                        help="Use a previously built C++ library without rebuilding")
    args = parser.parse_args()
    if args.library and not args.library.is_file():
        parser.error("--library must name an existing shared library")
    verify_plant()
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-boomerang-chain-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    library = args.library.resolve() if args.library else destination / "libmclib_vexsim.so"
    lib = load_library(library) if args.library else build(destination)
    rows = []
    for speed in SPEEDS.values():
        for case in cases():
            trace, row = run_case(lib, speed, case)
            rows.append(row)
            with (destination / f"{speed:g}_{case['name']}.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
            print(f"{'PASS' if row['passed'] else 'FAIL'} {speed:g}/{case['name']} "
                  f"t={row['elapsed_ms']}ms remaining={row['position_error_in']:.6g}in "
                  f"along={row['along_in']:.6g}in mean={row['mean_command_V']:.6g}V "
                  f"issues={row['issues']}", flush=True)
    result = dict(profile=__doc__, library=str(library),
                  library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  lead=LEAD, timeout_ms=TIMEOUT_MS, position_band_in=POSITION_BAND_IN,
                  plane_epsilon_in=PLANE_EPSILON_IN,
                  handoff_contract="Existing configured position band with live translation; no exact-plane or heading-settlement requirement",
                  design_note="Exact-plane variants timed out before the plane with unchanged distance PID gains, both without and with the configured voltage floor. The handoff uses the existing position band rather than suppressing signed PID braking or retuning gains; along-plane error remains diagnostic.",
                  voltage_floor="Resolved API default; no explicit override",
                  rows=rows)
    (destination / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    passed = sum(row["passed"] for row in rows)
    print(f"Handoffs: {passed}/{len(rows)} passed", flush=True)
    return passed != len(rows)


if __name__ == "__main__":
    raise SystemExit(main())
