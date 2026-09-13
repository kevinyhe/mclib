#!/usr/bin/env python3
"""Bounded completion regressions using actual mclib and the exact ideal plant.

No vexsim physics is imported. Default production gains and explicitly tuned
ideal-plant gains are reported separately. The ideal plant has no inertia, so
its yaw loop uses proportional control with a 5/s continuous error-decay rate
and zero derivative gain. Production gains/configuration are never edited.

All selected cases must satisfy their assertions; defaults that oscillate are
reported as failures, not expected passes. Use --gains ideal to isolate the
completion regressions under the stated plant tuning.
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
from kinematic_motion_audit import IdealBridge, SPEEDS, TRACK, verify_plant
from run import ROOT, build


GAIN_SHIM = r'''
#include "mclib/control/motion_config.hpp"
extern "C" void completion_set_yaw_gains(double kp, double kd) {
  auto config = mclib::control::motionConfig();
  config.turn_pid = {kp, 0.0, kd};
  config.heading_pid = {kp, 0.0, kd};
  mclib::control::setMotionConfig(config);
}
'''


def gain_setter(destination):
    """Compile a small test-only adapter against the already built real library."""
    output = destination / "libcompletion_gains.so"
    command = ["g++", "-std=gnu++20", "-DMCLIB_HOST_BUILD", "-shared", "-fPIC",
               "-Iinclude", "-x", "c++", "-", "-x", "none",
               str(destination / "libmclib_vexsim.so"), "-Wl,-z,defs",
               "-o", str(output)]
    result = subprocess.run(command, input=GAIN_SHIM, text=True,
                            capture_output=True, cwd=ROOT)
    (destination / "gain-shim-build.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr)
    shim = C.CDLL(str(output))
    setter = shim.completion_set_yaw_gains
    setter.argtypes = [C.c_double, C.c_double]
    setter.restype = None
    return shim, setter


def cases():
    for heading in (0, 90, -90, 180):
        yield dict(name=f"boomerang_at_target_{heading:+d}", action=7,
                   x=0, y=0, heading=heading, direction=1, pure_turn=True)
    for name, x, y, direction in (
            ("point_near_side", 4, 0, 1),
            ("point_near_diagonal", 4, 4, 1),
            ("point_near_behind", 0, -4, 1),
            ("point_reverse", 0, -4, -1),
            ("point_reverse_diagonal", 4, -4, -1)):
        yield dict(name=name, action=6, x=x, y=y, heading=None,
                   direction=direction, pure_turn=False)


def run_case(lib, setter, speed, case, gains):
    plant = IdealBridge(lib, speed)
    # Plant yaw rate is 2*speed/(12*track) radians/s per yaw volt.
    yaw_gain_deg_per_s_per_v = 2 * speed / (12 * TRACK) * 180 / math.pi
    kp = 5.0 / yaw_gain_deg_per_s_per_v if gains == "ideal" else .3
    kd = 0.0 if gains == "ideal" else 1.5
    if gains == "ideal":
        setter(kp, kd)
    exceptions = []
    try:
        plant.run(case["action"], a=case["heading"] if case["action"] == 0 else case["x"], b=case["y"],
                  heading=case["heading"] or 0, direction=case["direction"],
                  timeout=4000)
    except RuntimeError as error:
        exceptions.append(str(error))
    position_error = math.hypot(plant.x - case["x"], plant.y - case["y"])
    heading_error = (None if case["heading"] is None else
                     abs(math.remainder(math.degrees(plant.heading) - case["heading"], 360)))
    max_translation = max((math.hypot(row["true_x_in"], row["true_y_in"])
                           for row in plant.trace), default=0)
    estimate = plant.pose()
    odometry_error = math.hypot(estimate[0] - plant.x, estimate[1] - plant.y)
    issues = exceptions[:]
    if plant.millis >= 4000:
        issues.append("motion reached its deadline")
    if not plant.stopped():
        issues.append("motion left a drive command active")
    if not math.isfinite(position_error) or position_error > 1.5 + 1e-8:
        issues.append("endpoint outside configured 1.5 inch position band")
    if heading_error is not None and (not math.isfinite(heading_error) or heading_error > 3 + 1e-8):
        issues.append("heading outside configured 3 degree turn band")
    if case["pure_turn"] and max_translation > 1e-8:
        issues.append("turning at the endpoint introduced translation")
    if not math.isfinite(odometry_error) or odometry_error > 1e-8:
        issues.append("exact sensor odometry disagrees with exact plant")
    return plant.trace, dict(case=case["name"], speed_ips=speed, gains=gains,
                            yaw_kp=kp, yaw_kd=kd, passed=not issues,
                            elapsed_ms=plant.millis, position_error_in=position_error,
                            heading_error_deg=heading_error,
                            max_translation_in=max_translation,
                            odometry_error_in=odometry_error,
                            pose=[plant.x, plant.y, math.degrees(plant.heading)],
                            stopped=plant.stopped(), issues=issues)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--gains", choices=("default", "ideal", "both"), default="both")
    args = parser.parse_args()
    verify_plant()
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-motion-completion-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    lib = build(destination)
    shim, setter = gain_setter(destination)
    gain_modes = ("default", "ideal") if args.gains == "both" else (args.gains,)
    rows = []
    for gains in gain_modes:
        for speed in SPEEDS.values():
            for case in cases():
                trace, row = run_case(lib, setter, speed, case, gains)
                rows.append(row)
                path = destination / f"{gains}_{speed:g}_{case['name']}.csv"
                with path.open("w", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(trace[0]))
                    writer.writeheader()
                    writer.writerows(trace)
                print(f"{'PASS' if row['passed'] else 'FAIL'} {gains}/{speed:g}/{case['name']} "
                      f"t={row['elapsed_ms']}ms pos={row['position_error_in']:.6g}in "
                      f"heading={row['heading_error_deg']} issues={row['issues']}", flush=True)
    references = []
    if "default" in gain_modes:
        # A matched existing turn routine distinguishes an ideal-plant gain
        # limit cycle from the new boomerang completion logic.
        for heading in (90, -90, 180):
            case = dict(name=f"standalone_turn_{heading:+d}", action=0,
                        x=0, y=0, heading=heading, direction=1, pure_turn=True)
            trace, row = run_case(lib, setter, max(SPEEDS.values()), case, "default")
            row["last_headings_deg"] = [entry["true_heading_deg"] for entry in trace[-6:]]
            references.append(row)
            print(f"REFERENCE default/{row['speed_ips']:g}/{case['name']} "
                  f"t={row['elapsed_ms']}ms heading_error={row['heading_error_deg']:.6g}deg "
                  f"passed={row['passed']}", flush=True)
    (destination / "results.json").write_text(json.dumps(dict(
        plant="Exact differential-drive kinematics; speed linear with voltage; no inertia or slip",
        tuning="ideal: yaw/heading kP = 5 / (2*speed/(12*track)*180/pi), kD=0; distance gains unchanged",
        position_band_in=1.5, heading_band_deg=3, timeout_ms=4000,
        rows=rows, standalone_turn_references=references), indent=2) + "\n")
    for gains in gain_modes:
        selected = [row for row in rows if row["gains"] == gains]
        count = sum(row["passed"] for row in selected)
        print(f"{gains}: {count} passed, {len(selected) - count} failed", flush=True)
    return any(not row["passed"] for row in rows)


if __name__ == "__main__":
    raise SystemExit(main())
