#!/usr/bin/env python3
"""Audit actual C++ boomerang travel direction on an independent exact plant.

The plant has no inertia or slip. Only its test yaw/heading gains are changed:
kP = 5 / (2*speed/(12*track)*180/pi), kD = 0. Distance gains stay unchanged.
Final heading always describes the robot body, including during reverse travel.
Endpoint checks and paired trajectory symmetry are required. Opposite commands
later in a run are recorded separately because PID braking may require them.
This does not establish physical drivetrain tuning or traction performance.
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
from kinematic_motion_audit import IdealBridge, SPEEDS, TRACK, verify_plant
from run import DEFAULT_TUNING, build, load_library


TIMEOUT_MS = 6000
LEAD = 0.5
# Commands are quantized to integer millivolts by the shared bridge. These
# tolerances allow accumulated rounding, but not a directional bias or turn.
SYMMETRY_TOLERANCES = dict(command_V=0.005, position_in=0.01,
                           heading_deg=0.05, elapsed_ms=10)


def cases():
    for name, x, y, heading in (
            ("straight", 0, 24, 0),
            ("lateral", 24, 0, 90),
            ("diagonal_aligned", 24, 24, 90),
            ("diagonal_opposed", 24, 24, -90),
            ("behind", 0, -24, 0)):
        for mirror in ((1,) if x == 0 else (1, -1)):
            for direction in (1, -1):
                yield dict(name=f"{name}_mirror{mirror:+d}_dir{direction:+d}",
                           geometry=name, mirror=mirror, direction=direction,
                           x=x * mirror * direction, y=y * direction,
                           heading=heading * mirror)


def effective_output(row, side):
    return row[f"{side}_V"] if row[f"{side}_mode"] == 0 else 0.0


def carrot_projection(row, case):
    """Carrot displacement along the selected travel axis, in inches."""
    hypotenuse = math.hypot(case["x"] - row["true_x_in"],
                            case["y"] - row["true_y_in"])
    approach = math.radians(case["heading"])
    carrot_x = case["x"] - LEAD * hypotenuse * math.sin(approach) * case["direction"]
    carrot_y = case["y"] - LEAD * hypotenuse * math.cos(approach) * case["direction"]
    heading = math.radians(row["true_heading_deg"])
    return case["direction"] * ((carrot_x - row["true_x_in"]) * math.sin(heading)
                                 + (carrot_y - row["true_y_in"]) * math.cos(heading))


def annotate_trace(trace, case):
    previous = None
    for row in trace:
        left, right = (effective_output(row, side) for side in ("left", "right"))
        row["mean_command_V"] = (left + right) / 2
        row["travel_command_V"] = row["mean_command_V"] * case["direction"]
        row["carrot_projection_in"] = carrot_projection(row, case)
        row["body_projected_displacement_in"] = 0.0
        if previous is not None:
            midpoint_heading = math.radians((row["true_heading_deg"]
                                             + previous["true_heading_deg"]) / 2)
            row["body_projected_displacement_in"] = (
                (row["true_x_in"] - previous["true_x_in"]) * math.sin(midpoint_heading)
                + (row["true_y_in"] - previous["true_y_in"]) * math.cos(midpoint_heading))
        row["travel_projected_displacement_in"] = (
            row["body_projected_displacement_in"] * case["direction"])
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
                  direction=case["direction"], timeout=TIMEOUT_MS)
    except RuntimeError as error:
        issues.append(str(error))
    annotate_trace(plant.trace, case)
    position_error = math.hypot(plant.x - case["x"], plant.y - case["y"])
    heading_error = abs(math.remainder(math.degrees(plant.heading) - case["heading"], 360))
    odometry_error = max(math.hypot(row["odom_x_in"] - row["true_x_in"],
                                    row["odom_y_in"] - row["true_y_in"])
                         for row in plant.trace)
    odometry_heading_error = max(abs(math.remainder(row["odom_heading_deg"]
                                                    - row["true_heading_deg"], 360))
                                 for row in plant.trace)
    if plant.millis >= TIMEOUT_MS:
        issues.append("motion reached its deadline")
    if not plant.stopped():
        issues.append("motion left a drive command active")
    if not math.isfinite(position_error) or position_error > 1.5 + 1e-8:
        issues.append("endpoint outside configured 1.5 inch position band")
    if not math.isfinite(heading_error) or heading_error > 3 + 1e-8:
        issues.append("heading outside configured 3 degree turn band")
    if not math.isfinite(odometry_error) or odometry_error > 1e-8:
        issues.append("exact sensor odometry disagrees with exact plant position")
    if not math.isfinite(odometry_heading_error) or odometry_heading_error > 1e-8:
        issues.append("exact sensor odometry disagrees with exact plant heading")
    first = plant.trace[0]
    if first["travel_command_V"] < -1e-8:
        issues.append("startup command translates opposite the selected direction")
    startup_pivot = []
    for index, row in enumerate(plant.trace):
        if row["carrot_projection_in"] >= -1e-8:
            break
        startup_pivot.append(row)
        if index + 1 < len(plant.trace):
            after = plant.trace[index + 1]
            if math.hypot(after["true_x_in"], after["true_y_in"]) > 1e-8:
                issues.append("startup pivot with carrot behind translated the chassis")
                break
        if abs(row["mean_command_V"]) > 1e-8:
            issues.append("startup pivot with carrot behind has nonzero mean voltage")
            break
    opposite_distance = -sum(min(0.0, row["travel_projected_displacement_in"])
                             for row in plant.trace)
    opposite_command_samples = sum(row["travel_command_V"] < -0.001
                                   for row in plant.trace)
    return plant.trace, dict(
        case=case["name"], configuration=case, speed_ips=speed,
        yaw_kp=kp, yaw_kd=0, distance_gains=[gains[f"drive_{term}"] for term in ("kp", "ki", "kd")],
        passed=not issues, elapsed_ms=plant.millis,
        position_error_in=position_error, heading_error_deg=heading_error,
        odometry_error_in=odometry_error, odometry_heading_error_deg=odometry_heading_error,
        pose=[plant.x, plant.y, math.degrees(plant.heading)], stopped=plant.stopped(),
        initial_mean_command_V=first["mean_command_V"],
        initial_travel_command_V=first["travel_command_V"],
        startup_pivot_samples=len(startup_pivot),
        opposite_travel_distance_in=opposite_distance,
        opposite_command_samples=opposite_command_samples,
        opposite_travel_over_0_01in=opposite_distance > 0.01,
        minimum_travel_command_V=min(row["travel_command_V"] for row in plant.trace),
        issues=issues)


def check_symmetry(first, second, relation, speed):
    trace_a, row_a = first
    trace_b, row_b = second
    # Inverting travel: position'=-position, heading'=heading, L'=-R, R'=-L.
    # Reflecting the field X axis: x'=-x, y'=y, heading'=-heading, L'=R, R'=L.
    invert = relation == "reverse_travel"
    command_sign = -1 if invert else 1
    y_sign = -1 if invert else 1
    heading_sign = 1 if invert else -1
    metrics = dict(command_V=0.0, position_in=0.0, heading_deg=0.0,
                   elapsed_ms=abs(row_a["elapsed_ms"] - row_b["elapsed_ms"]))
    by_time = {round(row["t_s"] * 1000): row for row in trace_b}
    for a in trace_a:
        b = by_time.get(round(a["t_s"] * 1000))
        if b is None:
            continue
        # The first routine to settle may switch to hold one tick earlier.
        # Check command symmetry while both are still executing motion.
        if all(row[f"{side}_mode"] == 0 for row in (a, b) for side in ("left", "right")):
            metrics["command_V"] = max(metrics["command_V"],
                abs(b["left_V"] - command_sign * a["right_V"]),
                abs(b["right_V"] - command_sign * a["left_V"]))
        metrics["position_in"] = max(metrics["position_in"],
            math.hypot(b["true_x_in"] + a["true_x_in"],
                       b["true_y_in"] - y_sign * a["true_y_in"]))
        metrics["heading_deg"] = max(metrics["heading_deg"],
            abs(math.remainder(b["true_heading_deg"] - heading_sign * a["true_heading_deg"], 360)))
    pose_a, pose_b = row_a["pose"], row_b["pose"]
    metrics["position_in"] = max(metrics["position_in"],
        math.hypot(pose_b[0] + pose_a[0], pose_b[1] - y_sign * pose_a[1]))
    metrics["heading_deg"] = max(metrics["heading_deg"],
        abs(math.remainder(pose_b[2] - heading_sign * pose_a[2], 360)))
    issues = [f"{quantity} symmetry residual {value:g} exceeds {SYMMETRY_TOLERANCES[quantity]:g}"
              for quantity, value in metrics.items()
              if not math.isfinite(value) or value > SYMMETRY_TOLERANCES[quantity]]
    return dict(relation=relation, speed_ips=speed, first=row_a["case"], second=row_b["case"],
                passed=not issues, maximum_residuals=metrics, issues=issues)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--library", type=Path, help="Use a previously built C++ library without rebuilding")
    args = parser.parse_args()
    if args.library and not args.library.is_file():
        parser.error("--library must name an existing shared library")
    verify_plant()
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-boomerang-direction-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    lib = load_library(args.library.resolve()) if args.library else build(destination)
    lib.sim_set_gains.argtypes = [C.POINTER(C.c_double)]
    lib.sim_set_gains.restype = None
    rows, symmetries = [], []
    for speed in SPEEDS.values():
        records = {}
        for case in cases():
            trace, row = run_case(lib, speed, case)
            records[(case["geometry"], case["mirror"], case["direction"])] = (trace, row)
            rows.append(row)
            with (destination / f"{speed:g}_{case['name']}.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
            print(f"{'PASS' if row['passed'] else 'FAIL'} {speed:g}/{case['name']} "
                  f"t={row['elapsed_ms']}ms pos={row['position_error_in']:.5g}in "
                  f"heading={row['heading_error_deg']:.5g}deg "
                  f"opposite={row['opposite_travel_distance_in']:.5g}in issues={row['issues']}", flush=True)
        for (geometry, mirror, direction), first in records.items():
            pairs = []
            if direction == 1:
                pairs.append(("reverse_travel", (geometry, mirror, -1)))
            if mirror == 1 and (geometry, -1, direction) in records:
                pairs.append(("reflect_x", (geometry, -1, direction)))
            for relation, key in pairs:
                row = check_symmetry(first, records[key], relation, speed)
                symmetries.append(row)
                print(f"{'PASS' if row['passed'] else 'FAIL'} symmetry/{speed:g}/{relation}/"
                      f"{row['first']} issues={row['issues']}", flush=True)
    results = dict(profile=__doc__, library=str(args.library.resolve()) if args.library else "current source build",
                   lead=LEAD, timeout_ms=TIMEOUT_MS, position_band_in=1.5, heading_band_deg=3,
                   symmetry_tolerances=SYMMETRY_TOLERANCES,
                   reflection_note="Exact 180 degree behind-target yaw ties are excluded from reflection pairs.",
                   rows=rows, symmetries=symmetries)
    (destination / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    passed = sum(row["passed"] for row in rows)
    symmetry_passed = sum(row["passed"] for row in symmetries)
    print(f"Motions: {passed}/{len(rows)} passed; symmetry: {symmetry_passed}/{len(symmetries)} passed", flush=True)
    return passed != len(rows) or symmetry_passed != len(symmetries)


if __name__ == "__main__":
    raise SystemExit(main())
