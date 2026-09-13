#!/usr/bin/env python3
"""Validate the Metro host ABI, virtual tasks, and original source behavior.

Run with --library /absolute/path/to/libmetro_host.so. Every scenario loads the
one-shot runtime in a fresh subprocess. Inputs are explicitly synthetic sensor
fixtures, not a physics plant: passing these checks does not mean the autonomous
routine reaches its physical goals. The build includes the documented signed
arc and opposite-side odometry offset corrections.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import subprocess
import sys


class MetroSample(ctypes.Structure):
    _fields_ = [
        (name, ctypes.c_double)
        for name in (
            "heading", "left_deg", "right_deg", "left_current_ma",
            "right_current_ma", "left_rpm", "right_rpm", "tracker_centideg",
            "optical_proximity", "optical_hue", "distance_mm",
        )
    ]


OUTPUT = ctypes.CFUNCTYPE(
    None, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double
)
EVENT = ctypes.CFUNCTYPE(
    None, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double
)
CASES = (
    "stationary_timing",
    "prime_blocked_sensor",
    "pure_yaw_odometry_no_translation",
    "xy_reset_preserves_encoder_reference",
    "disable_cancels_pending",
    "nonfinite_sample",
    "skipped_tick",
    "second_initialization",
)


class Fixture:
    def __init__(self, library: Path, *, heading: float = 0, proximity: int = 0):
        self.lib = ctypes.CDLL(str(library))
        self.lib.metro_init.argtypes = [ctypes.POINTER(MetroSample), OUTPUT, EVENT]
        self.lib.metro_init.restype = ctypes.c_int
        self.lib.metro_tick.argtypes = [ctypes.c_uint32, ctypes.POINTER(MetroSample)]
        self.lib.metro_tick.restype = ctypes.c_int
        self.lib.metro_state.argtypes = [ctypes.POINTER(ctypes.c_double)]
        self.lib.metro_state.restype = None
        self.lib.metro_disable.argtypes = []
        self.lib.metro_disable.restype = None
        self.lib.metro_error.argtypes = []
        self.lib.metro_error.restype = ctypes.c_char_p
        self.lib.metro_now.argtypes = []
        self.lib.metro_now.restype = ctypes.c_uint32
        self.lib.metro_task_name.argtypes = [ctypes.c_int]
        self.lib.metro_task_name.restype = ctypes.c_char_p
        self.outputs: list[list[float | int]] = []
        self.events: list[list[float | int]] = []
        # Callback objects remain alive until disable has detached both pointers.
        self.output_callback = OUTPUT(self.record_output)
        self.event_callback = EVENT(self.record_event)
        self.sample = MetroSample(
            heading=heading, optical_proximity=proximity, distance_mm=1000
        )
        self.now = -1
        status = self.lib.metro_init(
            ctypes.byref(self.sample), self.output_callback, self.event_callback
        )
        require(status == 0, self.error())

    def record_output(self, kind: int, channel: int, mode: int, value: float):
        self.outputs.append([self.lib.metro_now(), kind, channel, mode, value])

    def record_event(self, kind: int, task: int, a: float, b: float):
        self.events.append([self.lib.metro_now(), kind, task, a, b])

    def error(self) -> str:
        return self.lib.metro_error().decode()

    def state(self) -> list[float]:
        state = (ctypes.c_double * 10)()
        self.lib.metro_state(state)
        return list(state)

    def tick(self) -> list[float]:
        self.now += 1
        status = self.lib.metro_tick(self.now, ctypes.byref(self.sample))
        require(status == 0, f"tick {self.now}: {self.error()}")
        return self.state()

    def until_return(self) -> list[float]:
        while self.now < 16000:
            state = self.tick()
            if state[8]:
                return state
        raise AssertionError("synthetic fixture did not return by 16000 ms")

    def disable(self):
        self.lib.metro_disable()

    def task_name(self, task: int) -> str:
        return self.lib.metro_task_name(task).decode()


def require(condition: bool, message: object):
    if not condition:
        raise AssertionError(message)


def require_stopped(fixture: Fixture):
    for side in (0, 1):
        last = [o for o in fixture.outputs if o[1:3] == [0, side]][-1]
        require(last[3] in (1, 2, 3) and last[4] == 0, last)
    for channel in (0, 1):
        last = [o for o in fixture.outputs if o[1:3] == [1, channel]][-1]
        require(last[4] == 0, last)
    require(fixture.state()[7] == 0, fixture.state())


def stationary_timing(fixture: Fixture) -> dict:
    # Init models the end of pre-autonomous IMU calibration. It must zero the
    # raw sensor frame without a physical heading command or a clock advance.
    require(fixture.state()[2] == 0, fixture.state())
    state = fixture.until_return()
    require(state[9] == 9420, state)
    creates = [e for e in fixture.events if e[1] == 0]
    require([e[2] for e in creates] == list(range(7)), creates)
    require(
        [fixture.task_name(i) for i in range(3)]
        == ["odom_no", "auton_scheduler", "leftSideSevenMiddle"],
        creates,
    )
    require([e[0] for e in fixture.events if e[1] == 6] == [0, 0], "extra encoder taring")
    require([500, 2, ord("D"), 0, 1.0] in fixture.outputs, "scraper deadline")
    require([500, 1, 3, 300.0, 800.0] in fixture.events, "nested callback delay")
    require([800, 2, 3, 0.0, 0.0] in fixture.events, "callback completion")
    # Routine/odometry continue while the scraper callback sleeps for 300 ms.
    require(
        any(e[1:3] == [1, 0] and 500 < e[0] < 800 for e in fixture.events),
        "odom stopped during delayed callback",
    )
    require(
        any(e[1:3] == [1, 2] and 500 < e[0] < 800 for e in fixture.events),
        "routine stopped during delayed callback",
    )
    raw_zero_task = next(e for e in creates if e[2] == 4)
    xy_reset_task = next(e for e in creates if e[2] == 5)
    require(raw_zero_task[0] == xy_reset_task[0] == 4510, creates)
    require([4810, 2, 4, 0.0, 0.0] in fixture.events, "raw zero delay")
    require([5010, 2, 5, 0.0, 0.0] in fixture.events, "XY reset delay")
    require([5060, 1, 1, 0, -127.0] in fixture.outputs, "direct top command")
    require([5070, 1, 1, 0, -90.0] in fixture.outputs, "PRIME scheduler override")
    require([5120, 1, 1, 0, 0.0] in fixture.outputs, "PRIME clear after 50 ms")
    require([6480, 2, 6, 0.0, 0.0] in fixture.events, "MID_SCORE callback deadline")
    require([6490, 1, 1, 0, -65.0] in fixture.outputs, "MID_SCORE scheduler effect")
    # Return is observable immediately; the host did not insert a 300 ms hold.
    require(fixture.now == 9420, fixture.now)
    return {"return_ms": state[9], "task_count": len(creates)}


def run_case(name: str, library: Path) -> dict:
    require(ctypes.sizeof(MetroSample) == 88, "MetroSample ABI size")
    fixture = Fixture(
        library,
        heading=123 if name == "stationary_timing" else 0,
        proximity=255 if name == "prime_blocked_sensor" else 0,
    )
    details = {}
    try:
        if name == "stationary_timing":
            details = stationary_timing(fixture)
        elif name == "prime_blocked_sensor":
            state = fixture.until_return()
            require(state[9] == 9420, state)
            # Unlike the clear-sensor fixture, PRIME does not end at its 50 ms
            # minimum when optical proximity remains blocked. It ends only when
            # the actual delayed MID_SCORE state replaces it.
            require([5120, 1, 1, 0, -90.0] in fixture.outputs, "PRIME lost sensor dependency")
            require([6480, 1, 1, 0, -90.0] in fixture.outputs, "PRIME stopped prematurely")
            require([6490, 1, 1, 0, -65.0] in fixture.outputs, "delayed score not applied")
            details = {"proximity": 255, "return_ms": state[9]}
        elif name == "pure_yaw_odometry_no_translation":
            fixture.tick()
            for ms in range(1, 11):
                fixture.sample.heading = 90 * ms / 10
                fixture.sample.left_deg = 180 * ms / 10
                fixture.sample.right_deg = -180 * ms / 10
                state = fixture.tick()
            theta = math.radians(90 * 0.9972299169)
            expected = [0.0, 0.0]
            require(math.dist(state[:2], expected) < 1e-9, (state, expected))
            require(abs(state[2] - math.degrees(theta)) < 1e-9, state)
            for ms in range(9, -1, -1):
                fixture.sample.heading = 90 * ms / 10
                fixture.sample.left_deg = 180 * ms / 10
                fixture.sample.right_deg = -180 * ms / 10
                state = fixture.tick()
                require(math.dist(state[:2], expected) < 1e-9, state)
            details = {"pure_turn_translation_in": state[:2], "both_turn_directions": True}
        elif name == "xy_reset_preserves_encoder_reference":
            deadline = None
            while fixture.now < 16000:
                fixture.sample.left_deg = fixture.now + 1
                fixture.sample.right_deg = fixture.now + 1
                state = fixture.tick()
                reset = next((e for e in fixture.events if e[1:3] == [0, 5]), None)
                if reset is not None:
                    deadline = int(reset[0]) + 500
                if fixture.now == deadline:
                    require(state[:2] == [0, 0], state)
                    require(fixture.sample.left_deg == deadline, "reset modified sensor sample")
                    break
            require(deadline is not None and fixture.now == deadline, "no scheduled reset observed")
            for _ in range(10):
                fixture.sample.left_deg += 1
                fixture.sample.right_deg += 1
                state = fixture.tick()
            require(abs(state[1] - 10 * 9.06 / 360) < 1e-9, state)
            require([e[0] for e in fixture.events if e[1] == 6] == [0, 0], "reset tared encoders")
            details = {"reset_ms": deadline, "next_10ms_odom_y_in": state[1]}
        elif name == "disable_cancels_pending":
            while fixture.now < 100:
                fixture.tick()
            fixture.disable()
            require_stopped(fixture)
            previous_count = len(fixture.outputs)
            while fixture.now < 1000:
                fixture.tick()
            require(len(fixture.outputs) == previous_count, "output after disable")
            require(not fixture.state()[8], "disabled routine falsely returned")
            require(not any(o[1:3] == [2, ord("D")] and o[4] for o in fixture.outputs), "delayed scraper survived disable")
            details = {"disabled_ms": 100, "observed_until_ms": 1000}
        else:
            fixture.tick()
            if name == "nonfinite_sample":
                fixture.sample.heading = math.nan
                status = fixture.lib.metro_tick(1, ctypes.byref(fixture.sample))
                expected = "nonfinite MetroSample"
            elif name == "skipped_tick":
                status = fixture.lib.metro_tick(2, ctypes.byref(fixture.sample))
                expected = "exactly one ms"
            elif name == "second_initialization":
                status = fixture.lib.metro_init(
                    ctypes.byref(fixture.sample), fixture.output_callback, fixture.event_callback
                )
                expected = "one init per process"
            else:
                raise ValueError(name)
            require(status == -1 and expected in fixture.error(), (status, fixture.error()))
            require_stopped(fixture)
            details = {"expected_error": fixture.error()}
        state = fixture.state()
        return {
            "case": name, "passed": True, "details": details, "state": state,
            "output_count": len(fixture.outputs), "event_count": len(fixture.events),
        }
    finally:
        fixture.disable()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="Directory for generated JSON results")
    parser.add_argument("--case", choices=CASES, help=argparse.SUPPRESS)
    args = parser.parse_args()
    library = args.library.resolve()
    if not library.is_file():
        parser.error(f"library does not exist: {library}")
    if args.case:
        print(json.dumps(run_case(args.case, library)))
        return 0
    env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
    results = []
    for case in CASES:
        try:
            completed = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--library", str(library), "--case", case],
                env=env, capture_output=True, text=True, timeout=30, check=False,
            )
            if completed.returncode:
                result = {"case": case, "passed": False, "returncode": completed.returncode,
                          "stdout": completed.stdout, "stderr": completed.stderr}
            else:
                result = json.loads(completed.stdout)
        except (subprocess.TimeoutExpired, json.JSONDecodeError) as error:
            result = {"case": case, "passed": False, "error": str(error)}
        results.append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {case}")
        if not result["passed"]:
            print(json.dumps(result, indent=2))
    summary = {
        "description": "Synthetic Metro host/source-fidelity checks, not physical motion acceptance",
        "library": str(library), "passed": sum(r["passed"] for r in results),
        "total": len(results), "results": results,
    }
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "metro_host_tests.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"{summary['passed']}/{summary['total']} Metro host checks passed")
    return 0 if all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
