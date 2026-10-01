#!/usr/bin/env python3
"""Fit drive feedforward, effective track width and RAMSETE advice from a
tuning log written by mclib/control/characterize.hpp.

    python3 tools/tune_drive.py tune000.csv [more.csv ...]

The log's `phase` column says what each row was doing:

  1  characterizeDrive(): straight at fixed voltages, forward and back
  2  characterizeSpin():  spinning in place at fixed voltages
  3  traceMotion():       a motion and the target it was chasing

From phase 1 it fits kS and kV with V = kS * sgn(v) + kV * v, using the
second half of each voltage hold, where the speed has settled. Speed comes
from the odometry position, so it is the speed the robot actually moved at,
slip included; that is what RAMSETE needs to turn a speed into volts. kA is
then fitted from the start of each hold, from what is left of the voltage
once kS and kV are taken out.

From phase 2 it works out the effective track width: the wheel speed the
feedforward predicts for the spin voltage, against the turn rate the IMU saw,
2 * (V - kS) / kV / turn rate. A skid-steer drive's wheels scrub sideways, so
this is wider than the real track.

From phase 3 it splits the gap between the target and the robot into the
part along the target's heading (behind or ahead) and the part across it
(off to the side), and says which gain to change.

Standard library only.
"""
import argparse
import csv
import math
import statistics
import sys

PHASE_DRIVE, PHASE_SPIN, PHASE_TRACE = 1, 2, 3


def read_rows(paths):
    rows = []
    for path in paths:
        with open(path, newline="") as file:
            lines = [line for line in file if line.strip() and not line.startswith("#")]
        for record in csv.DictReader(lines):
            try:
                rows.append({key: float(value) for key, value in record.items()})
            except (TypeError, ValueError):
                continue
    return rows


def segments(rows, phase):
    """Runs of consecutive rows in @p phase with the same commanded voltages."""
    runs, current, key = [], [], None
    for row in rows:
        this = (row["phase"], round(row["left_cmd_V"], 3), round(row["right_cmd_V"], 3))
        if this != key:
            if current and key[0] == phase:
                runs.append(current)
            current, key = [], this
        current.append(row)
    if current and key[0] == phase:
        runs.append(current)
    return runs


def forward_speeds(run):
    """Speed along the heading from the odometry position, in/s, centred
    differences over 3 ticks on each side to keep encoder steps out."""
    out = []
    for i in range(3, len(run) - 3):
        a, b = run[i - 3], run[i + 3]
        dt = (b["t_ms"] - a["t_ms"]) / 1000
        if dt <= 0:
            continue
        heading = math.radians(run[i]["heading_deg"])
        dx, dy = b["x_in"] - a["x_in"], b["y_in"] - a["y_in"]
        out.append((run[i], (dx * math.sin(heading) + dy * math.cos(heading)) / dt))
    return out


def fit_lines(points):
    """Least squares V = kS * sgn(v) + kV * v over (v, V) points."""
    s11 = s12 = s22 = b1 = b2 = 0.0
    for v, volts in points:
        sign = (v > 0) - (v < 0)
        s11 += 1 if sign else 0
        s12 += sign * v
        s22 += v * v
        b1 += sign * volts
        b2 += v * volts
    det = s11 * s22 - s12 * s12
    if abs(det) < 1e-12:
        return None
    return (b1 * s22 - b2 * s12) / det, (s11 * b2 - s12 * b1) / det


def fit_feedforward(rows):
    runs = [run for run in segments(rows, PHASE_DRIVE) if len(run) > 20]
    steady = []
    for run in runs:
        volts = (run[0]["left_cmd_V"] + run[0]["right_cmd_V"]) / 2
        speeds = forward_speeds(run)
        steady += [(v, volts) for _, v in speeds[len(speeds) // 2:]]
    fit = fit_lines(steady)
    if fit is None:
        return None
    ks, kv = fit
    # kA from the starts, where the speed is still changing.
    num = den = 0.0
    for run in runs:
        volts = (run[0]["left_cmd_V"] + run[0]["right_cmd_V"]) / 2
        speeds = forward_speeds(run)
        for (row_a, va), (row_b, vb) in zip(speeds, speeds[1:len(speeds) // 2]):
            dt = (row_b["t_ms"] - row_a["t_ms"]) / 1000
            if dt <= 0:
                continue
            accel = (vb - va) / dt
            if abs(accel) < 20:
                continue
            residual = volts - ks * ((va > 0) - (va < 0)) - kv * va
            num += residual * accel
            den += accel * accel
    ka = max(0.0, num / den) if den > 0 else 0.0
    residuals = [volts - (ks * ((v > 0) - (v < 0)) + kv * v) for v, volts in steady]
    return dict(ks=ks, kv=kv, ka=ka, points=len(steady), holds=len(runs),
                rms_v=math.sqrt(statistics.fmean(r * r for r in residuals)) if residuals else 0.0)


def fit_track(rows, ks, kv):
    widths = []
    for run in segments(rows, PHASE_SPIN):
        if len(run) < 20:
            continue
        volts = abs(run[0]["left_cmd_V"])
        tail = run[len(run) // 2:]
        dt = (tail[-1]["t_ms"] - tail[0]["t_ms"]) / 1000
        if dt <= 0:
            continue
        # Unwrapped degrees from the IMU, so the difference is the turn.
        rate = abs(math.radians(tail[-1]["heading_deg"] - tail[0]["heading_deg"]) / dt)
        wheel = (volts - ks) / kv
        if rate > 1e-3 and wheel > 0:
            widths.append(2 * wheel / rate)
    if not widths:
        return None
    return dict(track_in=statistics.fmean(widths), spins=len(widths),
                spread_in=(max(widths) - min(widths)) if len(widths) > 1 else 0.0)


def analyse_trace(rows):
    along, side, times = [], [], []
    for row in rows:
        if row["phase"] != PHASE_TRACE or not all(
                math.isfinite(row[key]) for key in ("target_x_in", "target_y_in", "target_heading_deg")):
            continue
        heading = math.radians(row["target_heading_deg"])
        dx, dy = row["target_x_in"] - row["x_in"], row["target_y_in"] - row["y_in"]
        along.append(dx * math.sin(heading) + dy * math.cos(heading))   # + : target ahead
        side.append(dx * math.cos(heading) - dy * math.sin(heading))    # + : target to the right
        times.append(row["t_ms"] / 1000)
    if len(along) < 10:
        return None
    duration = times[-1] - times[0]
    # A crossing is going from more than 0.5 in on one side to more than
    # 0.5 in on the other, however many ticks that takes.
    crossings, last = 0, 0
    for s in side:
        now = 1 if s > 0.5 else -1 if s < -0.5 else 0
        if now and last and now != last:
            crossings += 1
        if now:
            last = now
    return dict(samples=len(along), duration_s=duration,
                mean_along_in=statistics.fmean(along), worst_along_in=max(along, key=abs),
                mean_abs_side_in=statistics.fmean(abs(s) for s in side), worst_side_in=max(side, key=abs),
                side_crossings_per_s=crossings / duration if duration > 0 else 0.0)


def advice(trace):
    """Which gain to change, from the trace, in order of what to fix first."""
    notes = []
    if trace["mean_along_in"] > 1.0:
        notes.append("The robot ran behind the target by %.2f in on average: the feedforward is too weak. "
                     "Refit kS and kV (and kA) before changing b." % trace["mean_along_in"])
    elif trace["mean_along_in"] < -1.0:
        notes.append("The robot ran ahead of the target by %.2f in on average: the feedforward is too strong."
                     % -trace["mean_along_in"])
    if trace["side_crossings_per_s"] > 1.0:
        notes.append("The robot crossed the path %.1f times a second: it is weaving. Lower b, or raise zeta "
                     "toward 0.9." % trace["side_crossings_per_s"])
    elif trace["mean_abs_side_in"] > 1.0:
        notes.append("The robot was %.2f in to the side on average without weaving: raise b (try 1.5x)."
                     % trace["mean_abs_side_in"])
    if not notes:
        notes.append("Within 1 in along and across the path on average, without weaving. Leave the gains.")
    return notes


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="tuning logs from characterize.hpp")
    parser.add_argument("--json", action="store_true", help="print the results as JSON")
    args = parser.parse_args(argv)
    rows = read_rows(args.csv)
    if not rows:
        print("No rows: is this a log from characterize.hpp?", file=sys.stderr)
        return 1
    result = dict(rows=len(rows), feedforward=fit_feedforward(rows))
    ff = result["feedforward"]
    result["track"] = fit_track(rows, ff["ks"], ff["kv"]) if ff else None
    result["trace"] = analyse_trace(rows)
    result["advice"] = advice(result["trace"]) if result["trace"] else []
    if args.json:
        import json
        print(json.dumps(result, indent=2))
        return 0

    print(f"{len(rows)} rows from {', '.join(args.csv)}")
    if ff:
        print(f"Feedforward from {ff['holds']} straight holds ({ff['points']} steady samples):")
        print(f"  kS = {ff['ks']:.3f} V   kV = {ff['kv']:.4f} V per in/s   kA = {ff['ka']:.4f} V per in/s^2")
        print(f"  fit error {ff['rms_v']:.3f} V RMS; above ~0.2 V, check for wheel slip or a dragging side")
    else:
        print("No straight holds (phase 1): run characterizeDrive().")
    track = result["track"]
    if track:
        print(f"Effective track width from {track['spins']} spins: {track['track_in']:.2f} in"
              f" (spread {track['spread_in']:.2f} in)")
    elif ff:
        print("No spins (phase 2): run characterizeSpin() for the effective track width.")
    trace = result["trace"]
    if trace:
        print(f"Motion trace, {trace['duration_s']:.2f} s:")
        print(f"  along the path: {trace['mean_along_in']:+.2f} in on average (+ behind), worst {trace['worst_along_in']:+.2f} in")
        print(f"  to the side:    {trace['mean_abs_side_in']:.2f} in on average, worst {trace['worst_side_in']:+.2f} in,"
              f" {trace['side_crossings_per_s']:.1f} crossings/s")
        for note in result["advice"]:
            print("  -> " + note)
    if ff:
        print("\nmclib::control::RamseteConfig follow;")
        print(f"follow.feedforward.kS = {ff['ks']:.3f}_V;")
        print(f"follow.feedforward.kV = {ff['kv']:.4f}_V / mclib::units::inps;")
        print(f"follow.feedforward.kA = {ff['ka']:.4f}_V / mclib::control::inps2;")
        if track:
            print(f"follow.track_width = {track['track_in']:.2f}_in;")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
