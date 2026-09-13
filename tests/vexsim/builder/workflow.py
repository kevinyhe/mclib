#!/usr/bin/env python3
"""Persist baseline → focused comparison → verification → report as explicit stages.

Run each stage separately with the same --output directory. A failed or stale
physics baseline blocks subsequent motion comparisons. Acceptance failures stay
failed in logs and reports; running with tracking wheels never replaces the
default drive-encoder result. No Git operations or dependency installs occur.
"""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from runner import DEFAULTS, ROOT, source_fingerprint, strict_json, validate_spec, write_json


def command_check(name, command, destination, journal, persist, timeout=900, env=None):
    """Save running status before execution and the actual exit code afterwards."""
    record = dict(name=name, command=[str(arg) for arg in command], status="running",
                  started_at=time.time(), log=str(destination / f"{name}.log"))
    journal["checks"].append(record)
    persist()
    environment = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1", "DISPLAY": "", "WAYLAND_DISPLAY": ""}
    environment.update(env or {})
    process = None
    try:
        with Path(record["log"]).open("wb") as log:
            process = subprocess.Popen(record["command"], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                       env=environment, start_new_session=True)
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                record.update(status="error", error=f"Exceeded {timeout} seconds")
                code = process.returncode
        record["returncode"] = code
        if record["status"] == "running":
            record["status"] = "passed" if code == 0 else "failed"
    except (OSError, KeyboardInterrupt) as error:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        record.update(status="error", error=str(error) or "Interrupted by user")
        if isinstance(error, KeyboardInterrupt):
            raise
    finally:
        record["finished_at"] = time.time()
        persist()
    print(f"{record['status'].upper():6} {name}: {record['log']}", flush=True)
    return record["status"] == "passed"


def render_report(state, output):
    lines = ["# Motion investigation workflow", "",
             "Recorded subprocess results only. Historical or unrun stages do not count as passes.", "",
             "The physics baseline gates motion comparisons. Tracking wheels change the sensor layout;",
             "the original drive-encoder result remains a separate acceptance result.", ""]
    for attempt in state["attempts"]:
        lines += [f"## {attempt['stage']} · {attempt['id']}", "",
                  f"Status: **{attempt['status']}**. Source fingerprint: `{attempt['source_fingerprint']}`.", ""]
        if attempt.get("error"):
            lines += [attempt["error"], ""]
        for check in attempt["checks"]:
            code = check.get("returncode", "not completed")
            lines += [f"- {check['name']}: **{check['status']}**, exit `{code}`; log `{check['log']}`."]
        lines.append("")
        for label, path in attempt.get("results", {}).items():
            file = Path(path)
            if not file.is_file():
                continue
            try:
                result = strict_json(file.read_text())
            except (OSError, ValueError):
                continue
            lines += [f"### {label}", ""]
            if isinstance(result, dict) and "summary" in result:
                summary = result["summary"]
                lines += [f"{summary['passed_steps']} passed, {summary['failed_steps']} failed, "
                          f"{summary['skipped_steps']} skipped; simulated duration {summary['elapsed_ms']} ms.", ""]
                for row in result["steps"]:
                    lines += [f"- Step {row['index'] + 1} ({row['type']}): {row['reason']}."]
                    for diagnosis in row.get("diagnosis", []):
                        lines += [f"  {diagnosis}"]
            else:
                rows = result if isinstance(result, list) else result.get("results", [])
                if isinstance(rows, list) and rows:
                    failures = [row for row in rows if not row.get("passed", False)]
                    lines += [f"{len(rows) - len(failures)}/{len(rows)} scenarios passed.", ""]
                    lines += [f"- Failed: {row.get('name', 'unnamed')}" for row in failures]
                else:
                    lines += [f"Structured artifact: `{path}`."]
            lines += [""]
    lines += ["## Next decision", "",
              "If a physics baseline fails, investigate its log before interpreting motion endpoints.",
              "For a failed motion, inspect its deadline, body-versus-odometry error at return,",
              "heading/output oscillation, and the separately labeled 300 ms hold. Repeat the same",
              "scenario and seed after one configuration change. Gains remain explicit in spec.json.",
              "A successful simulation still requires robot measurements and firmware validation.", ""]
    path = output / "report.md"
    path.write_text("\n".join(lines))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stage", choices=("baseline", "focused", "verify", "report"))
    parser.add_argument("--output", type=Path, required=True, help="Persistent workflow directory; reuse it across stages")
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--spec", type=Path, help="Focused sequence JSON exported by the builder")
    args = parser.parse_args()
    focused_spec = None
    if args.stage == "focused":
        try:
            focused_spec = validate_spec(strict_json(args.spec.read_text())) if args.spec else copy.deepcopy(DEFAULTS)
        except (OSError, ValueError) as error:
            parser.error(str(error))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "workflow.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another workflow invocation owns this output directory; let it finish before starting this stage.", file=sys.stderr)
            return 2
        return run_stage(args, output, focused_spec)


def run_stage(args, output, focused_spec):
    state_path = output / "workflow.json"
    state = strict_json(state_path.read_text()) if state_path.is_file() else dict(version=1, attempts=[])
    # An interrupted process is not an active or successful validation.
    for old in state["attempts"]:
        if old["status"] == "running":
            old.update(status="error", error="Previous workflow invocation did not complete")
    if args.stage == "report":
        write_json(state_path, state)
        print(render_report(state, output))
        return 0
    fingerprint = source_fingerprint(args.vexsim)
    identity = f"{args.stage}-{uuid.uuid4().hex[:12]}"
    destination = output / identity
    destination.mkdir()
    attempt = dict(id=identity, stage=args.stage, status="running", source_fingerprint=fingerprint,
                   started_at=time.time(), checks=[], results={})
    state["attempts"].append(attempt)
    def persist():
        write_json(state_path, state)
        write_json(destination / "stage.json", attempt)
    persist()
    if args.stage != "baseline":
        baseline = next((item for item in reversed(state["attempts"][:-1]) if item["stage"] == "baseline"), None)
        if baseline is None or baseline["status"] != "passed" or baseline["source_fingerprint"] != fingerprint:
            attempt.update(status="blocked", error="Run a passing baseline for the current source revision before focused/verify.")
            persist()
            print(attempt["error"], file=sys.stderr)
            print(render_report(state, output))
            return 2
    python = [sys.executable, "-B"]
    checks = []
    if args.stage == "baseline":
        for name in ("electrical", "mechanical", "sensor", "game"):
            command = python + [HERE.parent / f"physics_{name}_audit.py"]
            if name != "electrical":
                command += ["--vexsim", args.vexsim]
            checks.append((f"physics-{name}", command))
        checks += [("browser-physics", ["node", HERE.parent / "physics_game_audit.mjs", args.vexsim]),
                   ("physics-suite", python + ["-m", "unittest", "discover", "-s", args.vexsim / "tests", "-v"]),
                   ("host", ["make", "-j2", "test"]),
                   ("builder", python + [HERE / "test_builder.py"]),
                   ("native-viewer", python + [HERE / "test_native_viewer.py"]),
                   ("builder-geometry", ["node", HERE / "reference_geometry_test.mjs"]),
                   ("tracking-bridge", python + [HERE.parent / "tracking_bridge_audit.py", "--vexsim", args.vexsim])]
        tracking = destination / "tracking-oracle"
        pursuit = destination / "pursuit-oracle"
        checks += [("tracking-oracle-build", ["g++", "-std=gnu++20", "-O2", "-DMCLIB_HOST_BUILD", "-Iinclude", "-pthread",
                    "tests/vexsim/tracking_analytic_audit.cpp", "src/mclib/control/odometry.cpp",
                    "src/mclib/control/robot_state.cpp", "src/mclib/math.cpp", "-o", tracking]),
                   ("tracking-oracle", [tracking]),
                   ("pursuit-oracle-build", ["g++", "-std=gnu++20", "-O1", "-g", "-DMCLIB_HOST_BUILD", "-Iinclude", "-Itests", "-pthread",
                    "tests/vexsim/pursuit_kinematic_audit.cpp", "src/mclib/math.cpp", "src/mclib/path/path.cpp",
                    "src/mclib/path/pure_pursuit.cpp", "src/mclib/path/spline.cpp", "-o", pursuit]),
                   ("pursuit-oracle", [pursuit])]
    elif args.stage == "focused":
        spec = focused_spec
        for mode in ("drive", "two"):
            case = copy.deepcopy(spec)
            case["tracking_mode"] = mode
            spec_path = destination / f"{mode}-spec.json"
            write_json(spec_path, case)
            run_output = destination / mode
            attempt["results"][mode] = str(run_output / "result.json")
            checks.append((f"focused-{mode}", python + [HERE / "runner.py", "--spec", spec_path,
                          "--output", run_output, "--vexsim", args.vexsim, "--cache", output / "cache"]))
    elif args.stage == "verify":
        for mode in ("drive", "two"):
            run_output = destination / f"matrix-{mode}"
            command = python + [HERE.parent / "run.py", "--vexsim", args.vexsim,
                                "--output", run_output, "--tracking-mode", mode]
            attempt["results"][f"matrix-{mode}"] = str(run_output / "results.json")
            checks.append((f"matrix-{mode}", command))
        checks += [("completion", python + [HERE.parent / "motion_completion_audit.py", "--gains", "both",
                                             "--output", destination / "completion"]),
                   ("boomerang-direction", python + [HERE.parent / "boomerang_direction_audit.py",
                                                      "--output", destination / "boomerang-direction"]),
                   ("boomerang-chain", python + [HERE.parent / "boomerang_chain_audit.py",
                                                  "--output", destination / "boomerang-chain"]),
                   ("kinematic", python + [HERE.parent / "kinematic_motion_audit.py", "--output", destination / "kinematic"])]
        attempt["results"]["boomerang-direction"] = str(destination / "boomerang-direction" / "results.json")
        attempt["results"]["boomerang-chain"] = str(destination / "boomerang-chain" / "results.json")
    try:
        for name, command in checks:
            physics_failed = any(c["status"] != "passed" for c in attempt["checks"]
                                 if c["name"].startswith("physics-") or c["name"] == "browser-physics")
            dependency = next((c for c in attempt["checks"] if c["name"] == name + "-build"), None)
            if ((args.stage == "baseline" and not name.startswith("physics-") and name != "browser-physics" and physics_failed)
                    or (dependency and dependency["status"] != "passed")):
                attempt["checks"].append(dict(name=name, command=[str(item) for item in command], status="skipped",
                                              error="Required physics or build check failed", log="not executed"))
                persist()
                continue
            command_check(name, command, destination, attempt, persist,
                          env={"VEXSIM_PATH": str(args.vexsim.resolve()),
                               "PYTHONPATH": str(args.vexsim.resolve())})
        attempt["status"] = "passed" if all(c["status"] == "passed" for c in attempt["checks"]) else "failed"
    except KeyboardInterrupt:
        attempt.update(status="error", error="Workflow interrupted; unfinished checks are not passes")
    finally:
        attempt["finished_at"] = time.time()
        if source_fingerprint(args.vexsim) != fingerprint:
            attempt.update(status="error", error="Source changed during this stage; rerun before interpreting results")
        persist()
        print(render_report(state, output))
    return 0 if attempt["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
