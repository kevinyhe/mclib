#!/usr/bin/env python3
"""Fit vexsim's tire grip to a robot's characterizeGrip() and
characterizeWallPush() logs.

    python3 tests/vexsim/fit_grip.py grip000.csv wall000.csv --vexsim ../vexsim

Each run in the logs (a spin, a circle, a launch, a wall push) is replayed in
vexsim from rest with the same commanded voltages, on the preset's drive with
candidate grip on its driven wheels. The fit picks the four numbers that make
the simulated wheel speeds, turn rate and sideways slide closest to the
logged ones:

  forward grip, static        mu_long_static
  forward grip, sliding       mu_long_slide, as a share of static
  sideways grip, static       mu_lat_static
  sideways grip, sliding      mu_lat_slide, as a share of static

It prints them next to vexsim's current values, the `foam.py` entries they
replace, and how well each kind of run matches. A number the logs barely
change the match for is marked as not pinned: run the test that shows it.
"""
import argparse
import csv
import math
import multiprocessing
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]

PHASE_NAMES = {4: "spins", 5: "circles", 6: "launches", 7: "wall push"}
WALL_PUSH = 7
COLUMNS = ("t_ms", "phase", "left_cmd_V", "right_cmd_V", "left_in", "right_in", "heading_deg",
           "sideways_in", "battery_V")
# Residual scales: what counts as one unit of mismatch.
SPEED_SCALE_IPS = 2.0
TURN_SCALE_DPS = 10.0
# Centred differences over this many ticks either side.
HALF_WINDOW = 3
AT_REST_IPS = 1.0
AT_REST_DPS = 5.0
# Slower sideways slide than this is too small to tell the tracking wheel's sign from.
SIGN_CHECK_SLIDE_IPS = 1.0
NOT_PINNED_RISE = 0.02


@dataclass(frozen=True)
class Grip:
    long_static: float
    long_share: float
    lat_static: float
    lat_share: float

    def as_list(self) -> List[float]:
        return [self.long_static, self.long_share, self.lat_static, self.lat_share]


PARAMETERS = (
    # name, lower bound, upper bound
    ("forward grip, static", 0.3, 1.5),
    ("forward grip, sliding share", 0.3, 1.0),
    ("sideways grip, static", 0.03, 1.0),
    ("sideways grip, sliding share", 0.3, 1.0),
)


@dataclass
class Run:
    source: str
    phase: int
    t_ms: List[float]
    left_cmd: List[float]
    right_cmd: List[float]
    battery_v: float
    signals: Dict[str, List[float]]


def read_log(path: Path) -> List[dict]:
    lines = path.read_text().splitlines()
    for line in lines:
        if line.startswith("# rows="):
            dropped = int(line.split("dropped=")[1].split()[0])
            if dropped:
                raise SystemExit(f"{path}: the logger dropped {dropped} rows; the runs have gaps. "
                                 "Log again with a bigger RowBuffer or a FlushTask.")
    data = [line for line in lines if line.strip() and not line.startswith("#")]
    reader = csv.DictReader(data)
    missing = [name for name in COLUMNS if name not in (reader.fieldnames or [])]
    if missing:
        raise SystemExit(f"{path}: no {', '.join(missing)} column. Log with GripLog "
                         "(mclib/control/characterize.hpp).")
    return [{name: float(row[name]) for name in COLUMNS} for row in reader]


def centred_rates(times_ms: Sequence[float], values: Sequence[float]) -> List[float]:
    rates = []
    for i in range(HALF_WINDOW, len(values) - HALF_WINDOW):
        dt = (times_ms[i + HALF_WINDOW] - times_ms[i - HALF_WINDOW]) / 1000
        rates.append((values[i + HALF_WINDOW] - values[i - HALF_WINDOW]) / dt)
    return rates


def signals_of(times_ms, left_in, right_in, heading_deg, sideways_in) -> Dict[str, List[float]]:
    return {
        "left": centred_rates(times_ms, left_in),
        "right": centred_rates(times_ms, right_in),
        "turn": centred_rates(times_ms, heading_deg),
        "sideways": centred_rates(times_ms, sideways_in),
    }


def unwrapped(degrees: Sequence[float]) -> List[float]:
    out = [degrees[0]]
    for value in degrees[1:]:
        out.append(out[-1] + math.remainder(value - out[-1], 360.0))
    return out


def split_runs(path: Path, rows: List[dict]) -> List[Run]:
    starts = [i for i, row in enumerate(rows)
              if row["phase"] != 0 and (i == 0 or rows[i - 1]["phase"] != row["phase"])]
    out = []
    for start in starts:
        end = start
        while end < len(rows) and rows[end]["phase"] == rows[start]["phase"]:
            end += 1
        run_rows = rows[start:end]
        phase = int(run_rows[0]["phase"])
        if phase not in PHASE_NAMES:
            raise SystemExit(f"{path}: phase {phase} is not a grip test. Log with characterizeGrip() "
                             "or characterizeWallPush().")
        if len(run_rows) <= 2 * HALF_WINDOW + 2:
            raise SystemExit(f"{path}: a {PHASE_NAMES[phase]} run at {run_rows[0]['t_ms']:.0f} ms "
                             f"has only {len(run_rows)} rows.")
        before = rows[max(0, start - 2 * HALF_WINDOW - 1):start + 1]
        if len(before) > 2 * HALF_WINDOW:
            still = signals_of([r["t_ms"] for r in before], [r["left_in"] for r in before],
                               [r["right_in"] for r in before], unwrapped([r["heading_deg"] for r in before]),
                               [r["sideways_in"] for r in before])
            if (max(abs(still["left"][-1]), abs(still["right"][-1])) > AT_REST_IPS
                    or abs(still["turn"][-1]) > AT_REST_DPS):
                raise SystemExit(f"{path}: the robot was still moving when the {PHASE_NAMES[phase]} run at "
                                 f"{run_rows[0]['t_ms']:.0f} ms began. Lengthen GripTests::pause.")
        # Each run is replayed from rest, so every signal counts from its first row.
        first = run_rows[0]
        signals = signals_of([r["t_ms"] for r in run_rows],
                             [r["left_in"] - first["left_in"] for r in run_rows],
                             [r["right_in"] - first["right_in"] for r in run_rows],
                             unwrapped([r["heading_deg"] - first["heading_deg"] for r in run_rows]),
                             [r["sideways_in"] - first["sideways_in"] for r in run_rows])
        out.append(Run(path.name, phase, [r["t_ms"] for r in run_rows], [r["left_cmd_V"] for r in run_rows],
                       [r["right_cmd_V"] for r in run_rows], first["battery_V"], signals))
    return out


def check_signs(runs: List[Run]) -> None:
    """Stop on a log whose sensors read the wrong way round."""
    spins = [run for run in runs if run.phase == 4 and run.left_cmd[0] > 0]
    for run in spins:
        middle = len(run.signals["turn"]) // 2
        if run.signals["left"][middle] <= 0:
            raise SystemExit(f"{run.source}: the left encoder counts down when the left side is driven "
                             "forward. Fix the drive's motor directions.")
        if run.signals["turn"][middle] <= 0:
            raise SystemExit(f"{run.source}: the heading goes down when the robot spins clockwise. "
                             "mclib's heading is clockwise-positive; check the IMU setup.")
    right_turns = [run for run in runs if run.phase == 5 and run.left_cmd[0] > run.right_cmd[0]]
    for run in right_turns:
        second_half = run.signals["sideways"][len(run.signals["sideways"]) // 2:]
        if sum(second_half) / len(second_half) > SIGN_CHECK_SLIDE_IPS:
            raise SystemExit(f"{run.source}: the sideways tracking wheel counts up as the robot slides out "
                             "to its left on a right turn. It must count up when the robot moves right; "
                             "reverse its port.")


# --------------------------------------------------------------- replay ---

_WORKER: dict = {}


def _start_worker(vexsim: str, preset: str, runs: List[Run]) -> None:
    sys.path.insert(0, vexsim)
    _WORKER.update(preset=preset, runs=runs)


def _chassis_with(grip: Grip):
    from vexsim import presets
    chassis = presets.get(_WORKER["preset"])
    for unit in chassis.wheel_units:
        if unit.wheel.driven:
            tire = unit.wheel.tire
            tire.mu_long_static = grip.long_static
            tire.mu_long_slide = grip.long_static * grip.long_share
            tire.mu_lat_static = grip.lat_static
            tire.mu_lat_slide = grip.lat_static * grip.lat_share
    return chassis


def _battery_at(volts: float):
    from vexsim.battery import Battery
    battery = Battery()
    full, empty = Battery(soc=1.0).open_circuit_voltage(), Battery(soc=0.0).open_circuit_voltage()
    if not empty - 0.1 <= volts <= full + 0.1:
        raise ValueError(f"battery at {volts:.2f} V is outside vexsim's {empty:.2f}-{full:.2f} V")
    low, high = 0.0, 1.0
    for _ in range(40):
        battery.reset((low + high) / 2)
        if battery.open_circuit_voltage() < volts:
            low = (low + high) / 2
        else:
            high = (low + high) / 2
    battery.reset((low + high) / 2)
    return battery


def replay(run: Run, grip: Grip) -> Dict[str, List[float]]:
    """The run's signals in vexsim with @p grip on the driven wheels, read
    through vexsim's models of the same sensors the robot logged with."""
    from vexsim import Simulator
    from vexsim.sim import SimConfig
    from vexsim.units import INCH
    chassis = _chassis_with(grip)
    sim = Simulator(chassis, battery=_battery_at(run.battery_v), config=SimConfig(log_every=0))
    sim.dynamics.constrained = run.phase == WALL_PUSH
    sides = [[m for u in units for m in u.motors] for units in (chassis.left_units(), chassis.right_units())]
    inches_per_motor_radian = [[u.radius / u.gear_ratio / INCH for u in units for _ in u.motors]
                               for units in (chassis.left_units(), chassis.right_units())]
    sideways_wheel = next(u for u in chassis.wheel_units if u.name == "track_perp")
    sideways_sensor = sim.tracking["track_perp"]

    def side_inches(side):
        travel = [sim.encoders[m.name].position * (-1.0 if m.reversed else 1.0) * scale
                  for m, scale in zip(sides[side], inches_per_motor_radian[side])]
        return sum(travel) / len(travel)

    left, right, heading, sideways = [], [], [], []
    for i, t_ms in enumerate(run.t_ms):
        left.append(side_inches(0))
        right.append(side_inches(1))
        heading.append(-math.degrees(sim.imu.heading))
        # vexsim's sideways wheel rolls left-positive; the log is right-positive.
        sideways.append(-sideways_sensor.position * sideways_wheel.radius / INCH)
        for side, volts in ((0, run.left_cmd[i]), (1, run.right_cmd[i])):
            for motor in sides[side]:
                motor.set_voltage(volts)
        steps = round((run.t_ms[i + 1] - t_ms) if i + 1 < len(run.t_ms) else 10.0)
        for _ in range(steps):
            sim.step(0.001)
    return signals_of(run.t_ms, left, right, unwrapped(heading), sideways)


def _run_residuals(job: Tuple[int, Tuple[float, ...]]) -> Dict[str, float]:
    index, values = job
    run = _WORKER["runs"][index]
    simulated = replay(run, Grip(*values))
    sums = {}
    for name, scale in (("left", SPEED_SCALE_IPS), ("right", SPEED_SCALE_IPS),
                        ("turn", TURN_SCALE_DPS), ("sideways", SPEED_SCALE_IPS)):
        sums[name] = sum(((a - b) / scale) ** 2 for a, b in zip(run.signals[name], simulated[name]))
    sums["rows"] = len(run.signals["left"])
    return sums


class Replayer:
    def __init__(self, vexsim: Path, preset: str, runs: List[Run], jobs: int):
        self.runs = runs
        self.pool = multiprocessing.get_context("spawn").Pool(
            jobs, initializer=_start_worker, initargs=(str(vexsim), preset, runs))
        self.evaluations = 0

    def per_run(self, grip: Grip) -> List[Dict[str, float]]:
        self.evaluations += 1
        return self.pool.map(_run_residuals, [(i, tuple(grip.as_list())) for i in range(len(self.runs))])

    def loss(self, grip: Grip) -> float:
        results = self.per_run(grip)
        total = sum(r["left"] + r["right"] + r["turn"] + r["sideways"] for r in results)
        return total / sum(r["rows"] for r in results)

    def close(self) -> None:
        self.pool.close()
        self.pool.join()


# ------------------------------------------------------------------ fit ---

def clamp_to_bounds(values: Sequence[float]) -> List[float]:
    return [min(max(v, low), high) for v, (_, low, high) in zip(values, PARAMETERS)]


def nelder_mead(loss, start: Sequence[float], step: float, max_evaluations: int, tolerance: float):
    """Bounded Nelder-Mead: points outside the bounds are clamped onto them."""
    def evaluate(point):
        point = clamp_to_bounds(point)
        return loss(Grip(*point)), point

    simplex = [evaluate(start)]
    for i in range(len(start)):
        point = list(start)
        point[i] *= 1 + step
        simplex.append(evaluate(point))
    evaluations = len(simplex)
    while evaluations < max_evaluations:
        simplex.sort(key=lambda entry: entry[0])
        best, worst = simplex[0], simplex[-1]
        spread = max(abs(a - b) for entry in simplex[1:] for a, b in zip(entry[1], best[1]))
        if spread < tolerance:
            break
        centroid = [sum(entry[1][i] for entry in simplex[:-1]) / (len(simplex) - 1)
                    for i in range(len(start))]
        reflected = evaluate([c + (c - w) for c, w in zip(centroid, worst[1])])
        evaluations += 1
        if reflected[0] < best[0]:
            expanded = evaluate([c + 2 * (c - w) for c, w in zip(centroid, worst[1])])
            evaluations += 1
            simplex[-1] = expanded if expanded[0] < reflected[0] else reflected
        elif reflected[0] < simplex[-2][0]:
            simplex[-1] = reflected
        else:
            contracted = evaluate([c + 0.5 * (w - c) for c, w in zip(centroid, worst[1])])
            evaluations += 1
            if contracted[0] < worst[0]:
                simplex[-1] = contracted
            else:
                simplex = [best] + [evaluate([b + 0.5 * (p - b) for b, p in zip(best[1], entry[1])])
                                    for entry in simplex[1:]]
                evaluations += len(simplex) - 1
    simplex.sort(key=lambda entry: entry[0])
    return Grip(*simplex[0][1]), simplex[0][0]


def current_grip(vexsim: Path, preset: str) -> Grip:
    sys.path.insert(0, str(vexsim))
    from vexsim import presets
    tire = next(u.wheel.tire for u in presets.get(preset).wheel_units if u.wheel.driven)
    return Grip(tire.mu_long_static, tire.mu_long_slide / tire.mu_long_static,
                tire.mu_lat_static, tire.mu_lat_slide / tire.mu_lat_static)


def pinned(replayer: Replayer, grip: Grip, best_loss: float) -> List[float]:
    """How much the loss rises with each number 10% either side of the fit."""
    rises = []
    for i in range(len(PARAMETERS)):
        rise = math.inf
        for factor in (0.9, 1.1):
            values = grip.as_list()
            values[i] *= factor
            values = clamp_to_bounds(values)
            if values[i] == grip.as_list()[i]:
                continue
            rise = min(rise, replayer.loss(Grip(*values)) / best_loss - 1)
        rises.append(rise)
    return rises


@dataclass
class Fit:
    grip: Grip
    loss: float
    start_loss: float
    rises: List[float]
    per_phase: Dict[int, Dict[str, float]]
    per_phase_start: Dict[int, Dict[str, float]]
    evaluations: int


def rms_by_phase(runs: List[Run], results: List[Dict[str, float]]) -> Dict[int, Dict[str, float]]:
    out = {}
    for phase in sorted({run.phase for run in runs}):
        chosen = [r for run, r in zip(runs, results) if run.phase == phase]
        rows = sum(r["rows"] for r in chosen)
        out[phase] = {
            "wheels": math.sqrt(sum(r["left"] + r["right"] for r in chosen) / (2 * rows)) * SPEED_SCALE_IPS,
            "turn": math.sqrt(sum(r["turn"] for r in chosen) / rows) * TURN_SCALE_DPS,
            "sideways": math.sqrt(sum(r["sideways"] for r in chosen) / rows) * SPEED_SCALE_IPS,
        }
    return out


def fit(paths: Sequence[Path], vexsim: Path, preset: str, jobs: int, max_evaluations: int = 160) -> Fit:
    runs = []
    for path in paths:
        runs += split_runs(path, read_log(path))
    if not runs:
        raise SystemExit("no grip runs in the logs")
    check_signs(runs)
    start = current_grip(vexsim, preset)
    replayer = Replayer(vexsim, preset, runs, jobs)
    try:
        start_results = replayer.per_run(start)
        start_loss = replayer.loss(start)
        grip, loss = nelder_mead(replayer.loss, start.as_list(), 0.15, max_evaluations, 0.002)
        rises = pinned(replayer, grip, loss)
        best_results = replayer.per_run(grip)
        return Fit(grip, loss, start_loss, rises, rms_by_phase(runs, best_results),
                   rms_by_phase(runs, start_results), replayer.evaluations)
    finally:
        replayer.close()


def report(result: Fit, start: Grip) -> str:
    replaces = (
        "foam.OMNI_GRIP_MEASURED (forum pull test, 0.84 for 2.75 in)",
        'foam.GUESSES["Sliding grip"] (wheels.SLIDING_SHARE)',
        'foam.GUESSES["Omni sideways grip"], static (wheels._omni_tire)',
        'foam.GUESSES["Omni sideways grip"], sliding (wheels._omni_tire)',
    )
    lines = ["| Number | Fitted | vexsim now | Loss rise at +-10% | Replaces |", "| --- | --- | --- | --- | --- |"]
    fitted, now = result.grip.as_list(), start.as_list()
    for (name, _, _), value, current, rise, entry in zip(PARAMETERS, fitted, now, result.rises, replaces):
        mark = "" if rise >= NOT_PINNED_RISE else " (not pinned)"
        lines.append(f"| {name} | {value:.3f}{mark} | {current:.3f} | {rise * 100:.1f}% | {entry} |")
    lines.append("")
    lines.append(f"Sliding coefficients: forward {result.grip.long_static * result.grip.long_share:.3f}, "
                 f"sideways {result.grip.lat_static * result.grip.lat_share:.3f}.")
    lines.append(f"Mismatch (mean squared, scaled): {result.loss:.3f} fitted, {result.start_loss:.3f} "
                 f"with vexsim's numbers, after {result.evaluations} replays.")
    lines.append("")
    lines.append("RMS mismatch per kind of run, fitted (vexsim's numbers):")
    lines.append("| Runs | Wheel speed, in/s | Turn rate, deg/s | Sideways slide, in/s |")
    lines.append("| --- | --- | --- | --- |")
    for phase, rms in result.per_phase.items():
        before = result.per_phase_start[phase]
        lines.append(f"| {PHASE_NAMES[phase]} | {rms['wheels']:.2f} ({before['wheels']:.2f}) | "
                     f"{rms['turn']:.1f} ({before['turn']:.1f}) | "
                     f"{rms['sideways']:.2f} ({before['sideways']:.2f}) |")
    pinned_note = [name for (name, _, _), rise in zip(PARAMETERS, result.rises) if rise < NOT_PINNED_RISE]
    if pinned_note:
        lines.append("")
        lines.append("Not pinned: " + ", ".join(pinned_note) + ". The logs match about as well "
                     "either way. Forward grip needs the wall push; sliding needs runs where the "
                     "wheels or the robot slide.")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path, help="CSV files written through GripLog")
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim", help="the vexsim checkout")
    parser.add_argument("--preset", default="override_cascade", help="vexsim's drive preset for the robot")
    parser.add_argument("--jobs", type=int, default=multiprocessing.cpu_count(), help="replays run at once")
    args = parser.parse_args()
    if not (args.vexsim / "vexsim" / "foam.py").exists():
        parser.error(f"{args.vexsim} is not a vexsim checkout with the foam tile model")
    if args.jobs < 1:
        parser.error("--jobs must be at least 1")
    start = current_grip(args.vexsim, args.preset)
    print(report(fit(args.logs, args.vexsim.resolve(), args.preset, args.jobs), start))
    return 0


if __name__ == "__main__":
    sys.exit(main())
