#!/usr/bin/env python3
"""Run chained moveToPoint() and boomerang() routes in the physics simulator.

Each route is two or three motions with exit=false, ending in one stopping
moveToPoint(). Every route runs at three voltage floors (the library default,
4 V and 8 V), given to every motion in it, on three drives: a 4-motor 450 rpm
drive on 2.75 in omnis with one team's tuning, and the six_motor_450 and
four_motor_200 presets with the default tuning. Two tracking wheels supply
the pose.

For each motion it prints how long it ran, its MotionResult, how close the
robot came to the target, how far it turned, and where it ended.

Known failures, with the 2 s limit on each stop:
- On the 450 rpm drives, 2 of 18 stops time out with a 4-8 V floor. Each
  starts at 26-50 in/s after the hand-off, slides past the target and turns
  299-322 deg coming back.
- On four_motor_200, 4 of 9 stops time out. Without a floor the distance PID
  crawls the last 3 in. With an 8 V floor the zigzag's stop holds the floor
  while it steers, arcs 9 in past the target and turns 217 deg.

    python3 tests/vexsim/chain_audit.py --output /tmp/chain
"""
import argparse
import math
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
from run import ROOT, PhysicsBridge, build

TEAM_TUNING = dict(drive_kp=.76, drive_ki=0., drive_kd=3., heading_kp=.3, heading_ki=0.,
                   heading_kd=1.5, turn_kp=.35, turn_ki=0., turn_kd=1.5)
RESULT = {0: "Reached", 1: "TimedOut", 2: "Cancelled", 3: "Disabled", 4: "Invalid", 5: "NoDrive"}
MIN_VOLTS = (-1, 4, 8)
MOVE_TO_POINT = 6
BOOMERANG = 7

# (label, sim_run arguments, target (x, y))
ROUTES = {
    "zigzag": [
        ("mtp (0,24) chained", dict(action=MOVE_TO_POINT, a=0, b=24, timeout=2000, stop=False), (0, 24)),
        ("mtp (24,48) chained", dict(action=MOVE_TO_POINT, a=24, b=48, timeout=2000, stop=False), (24, 48)),
        ("mtp (24,72) stop", dict(action=MOVE_TO_POINT, a=24, b=72, timeout=2000), (24, 72)),
    ],
    "sharp": [
        ("mtp (0,30) chained", dict(action=MOVE_TO_POINT, a=0, b=30, timeout=2000, stop=False), (0, 30)),
        ("mtp (24,36) chained", dict(action=MOVE_TO_POINT, a=24, b=36, timeout=2000, stop=False), (24, 36)),
        ("mtp (48,36) stop", dict(action=MOVE_TO_POINT, a=48, b=36, timeout=2000), (48, 36)),
    ],
    "boomerang": [
        ("boom (18,30) h90 chained", dict(action=BOOMERANG, a=18, b=30, heading=90, timeout=2500,
                                          stop=False), (18, 30)),
        ("mtp (42,30) stop", dict(action=MOVE_TO_POINT, a=42, b=30, timeout=2000), (42, 30)),
    ],
}


def team_drive():
    from vexsim.chassis import differential_drive
    return differential_drive(
        mass_lb=14.0, wheel="omni_2.75", cartridge="blue", motors_per_side=2,
        wheels_per_side=2, external_ratio=48.0 / 36.0, track_width_in=11.5,
        wheel_base_in=11.0, name="4-motor 450 rpm 2.75 in omni")


def run_route(lib, preset, tuning, route, min_volts):
    bridge = PhysicsBridge(lib, preset, tracking_mode="two", tuning=tuning)
    bridge.set_pose(0, 0, 0)
    rows = []
    for label, step, target in ROUTES[route]:
        start = bridge.sim.t
        frames = []
        bridge.on_capture = frames.append
        bridge.run(**step, min_volts=min_volts)
        closest = min(math.hypot(f["estimated_x"] - target[0], f["estimated_y"] - target[1])
                      for f in frames)
        headings = [f["estimated_heading"] for f in frames]
        turned = sum(abs((b - a + 180) % 360 - 180) for a, b in zip(headings, headings[1:]))
        rows.append((label, bridge.sim.t - start, RESULT[lib.sim_last_motion_result()],
                     closest, turned, bridge.pose()))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--output", type=Path, help="Build directory (default: a temporary one)")
    args = parser.parse_args()
    sys.path.insert(0, str(args.vexsim.resolve()))
    from vexsim import presets
    presets.ALL["team_4m_450_275"] = team_drive

    output = args.output or Path(tempfile.mkdtemp(prefix="mclib-chain-"))
    output.mkdir(parents=True, exist_ok=True)
    lib = build(output)
    for preset, tuning in (("team_4m_450_275", TEAM_TUNING), ("six_motor_450", None),
                           ("four_motor_200", None)):
        for route in ROUTES:
            for min_volts in MIN_VOLTS:
                print(f"\n{preset}  {route}  min {min_volts} V")
                for label, seconds, result, closest, turned, (x, y, h) in run_route(
                        lib, preset, tuning, route, min_volts):
                    print(f"  {label:26s} {seconds:5.2f} s {result:9s} closest {closest:5.2f} in"
                          f"  turned {turned:6.1f} deg  end ({x:6.2f}, {y:6.2f}, {h:7.1f})")


if __name__ == "__main__":
    main()
