#!/usr/bin/env python3
"""Exercise real C++ arc chaining, entry geometry and tight-radius kinematics.

The ideal plant has no inertia or slip. Its explicit test profile uses distance
kP=1.2, kD=0, arc exit bands of 0.1/0.3 inches for the small radii, and a yaw
error decay of 5/s with no derivative term. Chained arcs
use a 0.05 V floor: slow enough for the old arrival latch to stop the drive
before crossing, but sufficient for a live controller to finish in four seconds.
These are controller regressions, not physical drivetrain gain recommendations.
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


SHIM = r'''
#include "api.h"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
extern "C" void arc_prepare(double yaw_kp, bool chain,
                             double entry_degrees, double prior_degrees) {
  using namespace mclib::control;
  MotionConfig config;
  config.distance_pid = {1.2, 0.0, 0.0};
  config.heading_pid = {yaw_kp, 0.0, 0.0};
  config.arc_exit.small_error = 0.1;
  config.arc_exit.big_error = 0.3;
  config.dir_change_end = !chain;
  config.min_voltage = 0.05 * mclib::units::volt;
  setMotionConfig(config);
  pros::delay(0);
  resetOdometry({0, 0, entry_degrees * std::acos(-1.0) / 180});
  pros::delay(0);
  robotState().setCorrectAngleDeg(prior_degrees);
}
extern "C" void arc_previous_outputs(double* outputs) {
  outputs[0] = mclib::control::robotState().prevLeftOutput();
  outputs[1] = mclib::control::robotState().prevRightOutput();
}
'''


def adapter(destination):
    path = destination / "libarc_probe.so"
    command = ["g++", "-std=gnu++20", "-DMCLIB_HOST_BUILD", "-shared", "-fPIC",
               "-Wno-deprecated-declarations", "-Iinclude", "-x", "c++", "-",
               "-x", "none", str(destination / "libmclib_vexsim.so"),
               "-Wl,-z,defs", "-o", str(path)]
    result = subprocess.run(command, input=SHIM, text=True, capture_output=True, cwd=ROOT)
    (destination / "probe-build.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr)
    shim = C.CDLL(str(path))
    shim.arc_prepare.argtypes = [C.c_double, C.c_bool, C.c_double, C.c_double]
    shim.arc_prepare.restype = None
    shim.arc_previous_outputs.argtypes = [C.POINTER(C.c_double)]
    shim.arc_previous_outputs.restype = None
    return shim


def cases():
    for side in (-1, 1):
        for direction in (-1, 1):
            yield dict(name=f"chain_{side:+d}_{direction:+d}", chain=True,
                       side=side, direction=direction, entry=0, prior=0)
        yield dict(name=f"measured_entry_{side:+d}", chain=False,
                   side=side, direction=1, entry=30, prior=-90)
        for direction in (-1, 1):
            yield dict(name=f"tight_radius_{side:+d}_{direction:+d}", chain=False,
                       side=side, direction=direction, entry=0, prior=0, radius=3)
        yield dict(name=f"zero_radius_{side:+d}", chain=False,
                   side=side, direction=1, entry=0, prior=0, radius=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    destination = (args.output or Path(tempfile.mkdtemp(prefix="mclib-arc-completion-"))).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {destination}", flush=True)
    verify_plant()
    lib = build(destination)
    shim = adapter(destination)
    rows = []
    for speed in SPEEDS.values():
        for case in cases():
            plant = IdealBridge(lib, speed)
            plant.heading = math.radians(case["entry"])
            yaw_kp = 5 / (2 * speed / (12 * TRACK) * 180 / math.pi)
            shim.arc_prepare(yaw_kp, case["chain"], case["entry"], case["prior"])
            target_heading = case["entry"] + case["side"] * case["direction"] * 90
            radius = case.get("radius", 24) * case["side"]
            target = (radius * (math.cos(plant.heading) - math.cos(math.radians(target_heading))),
                      radius * (math.sin(math.radians(target_heading)) - math.sin(plant.heading)))
            issues = []
            try:
                plant.run(2, a=target_heading, b=radius, stop=not case["chain"], timeout=4000)
            except RuntimeError as error:
                issues.append(str(error))
            position_error = math.dist((plant.x, plant.y), target)
            heading_error = abs(math.remainder(math.degrees(plant.heading) - target_heading, 360))
            odometry_error = math.dist((plant.x, plant.y), plant.pose()[:2])
            outputs = (C.c_double * 2)()
            shim.arc_previous_outputs(outputs)
            if plant.millis >= 4000:
                issues.append("arc reached its deadline")
            if position_error > 1.5 or heading_error > 3:
                issues.append("arc missed the geometric endpoint")
            if odometry_error > 1e-8:
                issues.append("exact wheel odometry disagrees with the exact plant")
            if radius == 0 and max(math.hypot(v["true_x_in"], v["true_y_in"])
                                   for v in plant.trace) > 1e-8:
                issues.append("a zero-radius arc translated the chassis")
            if case["chain"]:
                if plant.stopped() or case["direction"] * sum(v for mode, v in plant.commands) <= 0:
                    issues.append("chaining failed to preserve forward/reverse drive output")
                if max(abs(outputs[i] - plant.commands[i][1]) for i in (0, 1)) > .0011:
                    issues.append("published slew baseline differs from applied millivolt output")
            elif not plant.stopped() or any(outputs):
                issues.append("stopped arc left a drive output or slew baseline active")
            row = dict(case=case["name"], speed_ips=speed, passed=not issues,
                       elapsed_ms=plant.millis, position_error_in=position_error,
                       heading_error_deg=heading_error, odometry_error_in=odometry_error,
                       yaw_kp=yaw_kp,
                       target=target, commands=plant.commands, issues=issues)
            rows.append(row)
            with (destination / f"{speed:g}_{case['name']}.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(plant.trace[0]))
                writer.writeheader()
                writer.writerows(plant.trace)
            print(f"{'PASS' if row['passed'] else 'FAIL'} {speed:g}/{case['name']} "
                  f"t={plant.millis}ms pos={position_error:.5g}in heading={heading_error:.5g}deg "
                  f"issues={issues}", flush=True)
    (destination / "results.json").write_text(json.dumps(dict(profile=__doc__, rows=rows), indent=2) + "\n")
    passed = sum(row["passed"] for row in rows)
    print(f"{passed}/{len(rows)} passed", flush=True)
    return passed != len(rows)


if __name__ == "__main__":
    raise SystemExit(main())
