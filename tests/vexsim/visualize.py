#!/usr/bin/env python3
"""Plot recorded C++/vexsim results without rerunning or altering the simulation.

Requires Pillow. Produces an exact-data PNG summary and an animated GIF replay.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

BG = "#f1f3f6"
INK = "#172337"
MUTED = "#667286"
GRID = "#e3e8ef"
BLUE = "#2468ed"
AMBER = "#ce8314"
RED = "#b4444e"
GREEN = "#19755e"
WHITE = "#ffffff"
FONT_ROOT = Path("/usr/share/fonts/truetype/dejavu")
FONTS = {}


def font(size, bold=False):
    key = size, bold
    if key not in FONTS:
        path = FONT_ROOT / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")
        FONTS[key] = ImageFont.truetype(str(path), size)
    return FONTS[key]


def label(draw, xy, value, size=20, color=INK, bold=False, anchor=None):
    draw.text(xy, str(value), font=font(size, bold), fill=color, anchor=anchor)


def dashed(draw, points, color, width=2, dash=8):
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        if length == 0:
            continue
        for offset in range(0, math.ceil(length), dash * 2):
            a, b = offset / length, min(offset + dash, length) / length
            draw.line((start[0] + (end[0] - start[0]) * a,
                       start[1] + (end[1] - start[1]) * a,
                       start[0] + (end[0] - start[0]) * b,
                       start[1] + (end[1] - start[1]) * b), fill=color, width=width)


def read_csv(path):
    with path.open() as file:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(file)]


def load_case(directory, results, name):
    stem = name.replace("/", "_")
    control = read_csv(directory / (stem + "_control.csv"))
    physics = read_csv(directory / (stem + ".csv"))
    result = results[name]
    actual = [(0.0, 0.0, 0.0, 0.0)] + [
        (r["t"], -r["y_in"], r["x_in"], -r["heading_deg"]) for r in physics]
    odometry = [(r["t"], r["estimated_x"], r["estimated_y"]) for r in control]
    odometry.append((result["elapsed"], *result["estimated_pose_at_exit"][:2]))
    return dict(result=result, actual=actual, odometry=odometry, control=control)


class Field:
    def __init__(self, draw, x, y, size, text_size=18):
        self.draw, self.x, self.y, self.size = draw, x, y, size
        self.low, self.high = -3, 37
        draw.rectangle((x, y, x + size, y + size), fill="#fafbfd", outline=GRID, width=1)
        for value in (0, 12, 24, 36):
            px, py = self.point(value, value)
            draw.line((px, y, px, y + size), fill=GRID, width=1)
            draw.line((x, py, x + size, py), fill=GRID, width=1)
            label(draw, (px, y + size + 8), value, text_size, MUTED, anchor="mt")
            label(draw, (x - 12, py), value, text_size, MUTED, anchor="rm")
        label(draw, (x + size / 2, y + size + 37), "X right (inches)", text_size, MUTED, anchor="mt")
        label(draw, (x, y - 30), "Y forward (inches)", text_size, MUTED)

    def point(self, x, y):
        scale = self.size / (self.high - self.low)
        return self.x + (x - self.low) * scale, self.y + self.size - (y - self.low) * scale

    def line(self, xy, color, width=4):
        points = [self.point(x, y) for x, y in xy]
        if len(points) > 1:
            self.draw.line(points, fill=color, width=width, joint="curve")

    def marker(self, x, y, color, diamond=False, radius=7):
        px, py = self.point(x, y)
        if diamond:
            self.draw.polygon(((px, py - radius), (px + radius, py),
                               (px, py + radius), (px - radius, py)), fill=color)
        else:
            self.draw.ellipse((px - radius, py - radius, px + radius, py + radius),
                              fill=color, outline=WHITE, width=2)

    def reference(self, arc=False, text_size=18):
        if arc:
            # An ideal geometric circle, not a recorded planner trajectory.
            for k in range(0, 90, 5):
                points = [self.point(24 * (1 - math.cos(math.radians(t))),
                                     24 * math.sin(math.radians(t))) for t in (k, k + 2.5)]
                self.draw.line(points, fill="#8a94a3", width=3)
        px, py = self.point(24, 24)
        self.draw.line((px - 8, py, px + 8, py), fill=INK, width=3)
        self.draw.line((px, py - 8, px, py + 8), fill=INK, width=3)
        label(self.draw, (px + 12, py - 26), "Target (24, 24)", text_size, INK)
        sx, sy = self.point(0, 0)
        label(self.draw, (sx + 10, sy - 24), "Start", text_size, MUTED)


def legend(draw, x, y, size=19):
    for offset, color, text in ((0, BLUE, "Actual motion"), (260, AMBER, "mclib odometry"),
                                (555, "#8a94a3", "Ideal arc reference (arc only)")):
        draw.line((x + offset, y + 12, x + offset + 30, y + 12), fill=color, width=4)
        label(draw, (x + offset + 42, y), text, size, MUTED)


def summary(directory, results, cases):
    image = Image.new("RGB", (1800, 1510), BG)
    draw = ImageDraw.Draw(image)
    label(draw, (56, 35), "mclib × vexsim", 21, MUTED, True)
    label(draw, (56, 72), "What the robot actually did", 46, INK, True)
    label(draw, (56, 133), "Recorded C++ control runs • default tuning • simulated noise and wheel slip", 21, MUTED)
    passes = sum(r["passed"] for r in results.values())
    safety = [r for r in results.values() if r["name"].startswith("safety/")]
    for x, number, text, color in ((56, f"{passes}/{len(results)}", "scenarios met the checks", INK),
                                   (625, f"{sum(r['passed'] for r in safety)}/{len(safety)}", "abort checks stopped drive output", GREEN),
                                   (1285, str(len(results) - passes), "scenarios need attention", RED)):
        label(draw, (x, 190), number, 42, color, True)
        label(draw, (x, 245), text, 20, MUTED)
    legend(draw, 58, 294)
    for x, title, name, is_arc in ((56, "Boomerang: endpoint is short", "boomerang", False),
                                   (922, "Arc: real path drifts wide", "arc", True)):
        case = cases[name]
        result = case["result"]
        draw.rounded_rectangle((x, 343, x + 822, 985), radius=12, fill=WHITE)
        label(draw, (x + 26, 367), title, 26, INK, True)
        label(draw, (x + 26, 409), "Four-inch-wheel speed_base • target heading 90°", 18, MUTED)
        field = Field(draw, x + 52, 480, 430)
        field.reference(is_arc, 16)
        field.line([(p[1], p[2]) for p in case["actual"]], BLUE, 5)
        field.line([(p[1], p[2]) for p in case["odometry"]], AMBER, 4)
        field.marker(*result["true_pose"][:2], BLUE)
        field.marker(*result["estimated_pose_at_exit"][:2], AMBER, diamond=True)
        dashed(draw, [field.point(*result["true_pose_at_exit"][:2]),
                      field.point(*result["estimated_pose_at_exit"][:2])], AMBER, 2, 5)
        cx = x + 535
        label(draw, (cx, 481), f"{result['position_error_in']:.2f} in", 33, RED, True)
        label(draw, (cx, 527), "true target error", 18, MUTED)
        label(draw, (cx, 554), "after 0.3 s of holding", 17, MUTED)
        label(draw, (cx, 617), f"{result['odometry_error_at_exit_in']:.2f} in", 29, AMBER, True)
        label(draw, (cx, 658), "odometry gap", 18, MUTED)
        label(draw, (cx, 685), "at the same exit time", 17, MUTED)
        label(draw, (cx, 749), f"{result['elapsed']:.2f} s", 29, INK, True)
        label(draw, (cx, 789), "timed out" if not result["met_deadline"] else "routine returned", 18,
              RED if not result["met_deadline"] else MUTED)
        label(draw, (cx, 852), f"Heading error {result['heading_error_deg']:.1f}°", 18, MUTED)

    point = cases["point"]
    rows = point["control"]
    timeout = point["result"]["elapsed"]
    actual_y = point["result"]["true_pose_at_exit"][1]
    zero_start = next(r["t"] for i, r in enumerate(rows)
                      if r["t"] > 1 and all(p["left_volts"] == 0 and p["right_volts"] == 0 for p in rows[i:]))
    draw.rounded_rectangle((56, 1018, 1744, 1440), radius=12, fill=WHITE)
    label(draw, (84, 1042), "moveToPoint(): drive output reaches zero, but the routine keeps waiting", 26, INK, True)
    label(draw, (84, 1086), f"Six-motor 450 RPM base • zero output from {zero_start:.2f} s until the {timeout:.2f} s timeout", 20, MUTED)
    left, right, top, bottom = 205, 1535, 1150, 1260
    time_x = lambda t: left + t / timeout * (right - left)
    position_y = lambda v: bottom - v / 26 * (bottom - top)
    voltage_y = lambda v: 1366 - v / 10 * 68
    stop_x = time_x(zero_start)
    draw.rectangle((stop_x, top, right, 1366), fill="#fff0ec")
    for v in (0, 12, 24):
        yy = position_y(v)
        draw.line((left, yy, right, yy), fill=GRID, width=1)
        label(draw, (left - 15, yy), f"{v} in", 17, MUTED, anchor="rm")
    dashed(draw, [(left, position_y(24)), (right, position_y(24))], INK, 2)
    for column, color in (("true_y", BLUE), ("estimated_y", AMBER)):
        draw.line([(time_x(r["t"]), position_y(r[column])) for r in rows], fill=color, width=4)
    label(draw, (1555, position_y(24) - 12), "24 in target", 18, INK)
    label(draw, (1555, position_y(actual_y) + 12), f"{actual_y:.2f} in actual", 17, BLUE)
    label(draw, (85, top + 35), "Position", 18, INK, True)
    label(draw, (85, 1305), "Left", 18, INK, True)
    label(draw, (85, 1332), "voltage", 18, INK, True)
    for v in (0, 9):
        yy = voltage_y(v)
        draw.line((left, yy, right, yy), fill=GRID, width=1)
        label(draw, (left - 15, yy), f"{v} V", 17, MUTED, anchor="rm")
    draw.line([(time_x(r["t"]), voltage_y(r["left_volts"])) for r in rows], fill=BLUE, width=4)
    dashed(draw, [(stop_x, top), (stop_x, 1366)], RED, 2)
    label(draw, (stop_x + 22, 1275), "No more drive voltage", 19, RED, True)
    for i in range(5):
        t = timeout * i / 4
        label(draw, (time_x(t), 1382), f"{t:g} s", 18, MUTED, anchor="mt")
    label(draw, (1555, 1330), "Both sides = 0 V", 17, BLUE)
    label(draw, (56, 1469), "Source: saved simulation traces, not an illustration. Default tuning on generic presets; not hardware validation.", 18, MUTED)
    image.save(directory / "visual-summary.png")
    return image


def replay(directory, cases):
    frames = []
    duration = max(cases[name]["actual"][-1][0] for name in ("boomerang", "arc"))
    for frame in range(math.ceil(duration * 20) + 1):
        t = min(frame / 20, duration)
        image = Image.new("RGB", (1160, 760), BG)
        draw = ImageDraw.Draw(image)
        label(draw, (36, 24), "Actual motion vs. mclib odometry", 28, INK, True)
        label(draw, (36, 68), "Recorded simulation • both moves start at (0, 0) facing +Y", 17, MUTED)
        label(draw, (1124, 27), f"{t:.2f} s", 26, INK, True, anchor="rt")
        draw.line((36, 109, 69, 109), fill=BLUE, width=4)
        label(draw, (80, 96), "Actual position + heading", 18, BLUE)
        draw.line((390, 109, 423, 109), fill=AMBER, width=4)
        label(draw, (434, 96), "mclib odometry (last sample held after routine exit)", 18, AMBER)
        for x, name, title in ((36, "boomerang", "Boomerang"), (610, "arc", "90° arc")):
            case = cases[name]
            result = case["result"]
            draw.rounded_rectangle((x, 139, x + 514, 714), radius=10, fill=WHITE)
            label(draw, (x + 20, 157), title, 23, INK, True)
            field = Field(draw, x + 54, 231, 401, 15)
            field.reference(name == "arc", 15)
            index = max(0, bisect.bisect_right([p[0] for p in case["actual"]], t) - 1)
            pose = case["actual"][index]
            field.line([(p[1], p[2]) for p in case["actual"][:index + 1]], BLUE, 4)
            odom = [p for p in case["odometry"] if p[0] <= t + 1e-9]
            field.line([(p[1], p[2]) for p in odom], AMBER, 3)
            if odom:
                field.marker(odom[-1][1], odom[-1][2], AMBER, True, 5)
            field.marker(pose[1], pose[2], BLUE, radius=6)
            px, py = field.point(pose[1], pose[2])
            angle = math.radians(pose[3])
            nose = (px + math.sin(angle) * 18, py - math.cos(angle) * 18)
            draw.line((px, py, *nose), fill=BLUE, width=4)
            label(draw, (x + 20, 691), f"Recording ended at {case['actual'][-1][0]:.2f} s"
                  if t > case["actual"][-1][0] + 1e-9
                  else "Timed out" if t >= result["elapsed"] and not result["met_deadline"]
                  else "Returned + holding" if t >= result["elapsed"] else "Controller running",
                  17, RED if t >= result["elapsed"] else MUTED)
        label(draw, (36, 730), "Markers show position and heading, not robot size. Dashed curve is an ideal arc reference.", 16, MUTED)
        frames.append(image.quantize(colors=128))
    image.save(directory / "replay-preview.png")
    durations = [50] * len(frames)
    durations[-1] = 1600
    frames[0].save(directory / "motion-replay.gif", save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, optimize=False, disposal=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", type=Path)
    args = parser.parse_args()
    directory = args.results_dir.resolve()
    results = {r["name"]: r for r in json.loads((directory / "results.json").read_text())}
    cases = {name: load_case(directory, results, key) for name, key in (
        ("boomerang", "speed_base/boomerang"), ("arc", "speed_base/arc"),
        ("point", "six_motor_450/point_forward"))}
    summary(directory, results, cases)
    replay(directory, cases)
    print(directory / "visual-summary.png")
    print(directory / "motion-replay.gif")


if __name__ == "__main__":
    main()
