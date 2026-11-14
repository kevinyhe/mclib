#!/usr/bin/env python3
"""The mclib path planner: plan autonomous paths on a field and test them in
vexsim with mclib's real C++. Listens on localhost. Run it with a Python that
has mujoco and numpy (tests/vexsim/requirements.txt), such as vexsim's venv:

    ../vexsim/.venv/bin/python tests/vexsim/planner/server.py
"""
import argparse
from collections import OrderedDict
import copy
import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from engine import ROOT, readiness, write_json  # noqa: E402
from export_cpp import YOU, export  # noqa: E402
from fields import drawings  # noqa: E402
from geometry import designed_poses  # noqa: E402
from project import (LIMITS, PRESETS, TRACKING, TRACTION, ProjectError, check_traction, config, step_name,  # noqa: E402
                     strict_json, validate_project)
from three_files import ThreeMissing, asset as three_asset  # noqa: E402

FINISHED = {"finished", "broke_rule", "cancelled", "error"}
#: A run keeps about 1.5 cores busy and holds about 400 MB.  On 24 threads, 8 at once finish one every
#: 13.6 s, against one every 61.5 s one at a time, though each takes 108 s instead of 61.5 s.
PARALLEL_RUNS = max(1, os.cpu_count() // 3)
STATIC = HERE / "static"
CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
                 ".js": "text/javascript; charset=utf-8", ".svg": "image/svg+xml"}
EXAMPLES = HERE / "examples"
#: The Override field's meshes for the 3D view, which come with vexsim
CAD_FILES = {"/cad/view.json": ("view.json", "application/json; charset=utf-8"),
             "/cad/view.bin": ("view.bin", "application/octet-stream")}
PAGE_POLICY = ("default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data: blob:; "
               "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
FRAMED_PAGES = {"/field3d.html"}
FRAMED_POLICY = PAGE_POLICY.replace("frame-ancestors 'none'", "frame-ancestors 'self'")


class BusyError(Exception):
    pass


def engine_command(*arguments, vexsim, cache):
    return [sys.executable, "-B", str(HERE / "engine.py"), *map(str, arguments),
            "--vexsim", str(vexsim), "--cache", str(cache)]


def check_runnable(project):
    """The checks a run would fail in its first stage, so the page hears at once."""
    designed_poses(project)
    for index, step in enumerate(project["steps"]):
        if step["type"] == "path" and not step["points"]:
            raise ProjectError(f"{step_name(index, step)} has no points yet. Click on the field to add at least one.",
                               index)


SOURCE_SUFFIXES = {".c", ".cpp", ".h", ".hpp"}


def local_path(text):
    """@p text as a path on this computer. Under WSL a Windows path such as
    C:\\Users\\you\\robot is the same folder at /mnt/c/Users/you/robot."""
    drive = re.fullmatch(r"([A-Za-z]):[\\/]+(.*)", text)
    if drive and os.name == "posix":
        return Path("/mnt", drive.group(1).lower(), *re.split(r"[\\/]+", drive.group(2)))
    try:
        return Path(text).expanduser()
    except RuntimeError as error:
        raise ValueError(f"Nothing was written. {text} names a home folder this computer does not have.") from error


def robot_project(text):
    """The PROS project folder typed as @p text."""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("Nothing was written. Type the folder of your robot project: the one that holds "
                         "project.pros.")
    folder = local_path(text.strip())
    if not folder.is_absolute():
        raise ValueError("Nothing was written. Give the robot project folder as a full path, such as "
                         f"/home/you/robot. Got {text.strip()}.")
    if not folder.is_dir():
        raise ValueError(f"Nothing was written. {folder} is not a folder on this computer.")
    if not (folder / "project.pros").is_file():
        raise ValueError(f"Nothing was written. {folder} has no project.pros, so it is not a PROS project.")
    return folder


def write_files(folder, files):
    """Write an export's files under @p folder and say what happened to each.
    A file that is the user's to edit is kept once it exists."""
    report = []
    for file in files:
        target = folder / file["path"]
        code = file["code"].encode()
        try:
            if target.exists() and file["owner"] == YOU:
                status = "kept"
            elif target.exists() and target.read_bytes() == code:
                status = "unchanged"
            else:
                status = "replaced" if target.exists() else "written"
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(code)
        except OSError as error:
            changed = [entry["path"] for entry in report if entry["status"] in ("written", "replaced")]
            before = f" Written before that: {', '.join(changed)}." if changed else " Nothing was written."
            raise ValueError(f"Could not write {target}: {error}.{before}") from error
        report.append(dict(path=file["path"], status=status))
    return report


def mentioned_in(folder, function, routine_path):
    """The first source file under @p folder, other than the routine's own,
    that names @p function: where it is already declared or run."""
    name = re.compile(rb"\b" + function.encode() + rb"\b")
    for top in ("src", "include"):
        for file in sorted((folder / top).rglob("*")):
            relative = file.relative_to(folder).as_posix()
            if file.suffix not in SOURCE_SUFFIXES or relative == routine_path or not file.is_file():
                continue
            try:
                found = name.search(file.read_bytes())
            except OSError as error:
                raise ValueError(f"Could not read {file}: {error}. Nothing was written.") from error
            if found:
                return relative
    return None


class Planner:
    """Starts every job in its own process, up to `parallel_runs` runs at
    once; this server never loads C++."""

    def __init__(self, output, vexsim, wall_timeout=900, parallel_runs=PARALLEL_RUNS):
        self.output, self.vexsim = Path(output).resolve(), Path(vexsim).resolve()
        self.cache = self.output / "cache"
        self._field_drawings = None
        self.output.mkdir(parents=True, exist_ok=True)
        self.wall_timeout = wall_timeout
        self.jobs = OrderedDict()
        self.lock = threading.RLock()
        self.pending = queue.Queue(maxsize=LIMITS["queue_size"])
        self.preparing = {}
        self.shutdown_event = threading.Event()
        self.processes = {}
        self.next_number = 1
        self._restore()
        self.workers = [threading.Thread(target=self._worker, name=f"run-worker-{n}", daemon=True)
                        for n in range(parallel_runs)]
        for worker in self.workers:
            worker.start()

    @property
    def field_drawings(self):
        if self._field_drawings is None:
            self._field_drawings = drawings(self.vexsim)
        return self._field_drawings

    def _restore(self):
        paths = sorted(self.output.glob("[0-9a-f]" * 32 + "/job.json"),
                       key=lambda path: path.stat().st_mtime)[-LIMITS["history_size"]:]
        for path in paths:
            try:
                job = strict_json(path.read_text())
            except (OSError, ValueError):
                continue
            if not isinstance(job, dict) or job.get("id") != path.parent.name:
                continue
            job["artifact_dir"] = str(path.parent)
            if job.get("status") not in FINISHED:
                job.update(status="error", error="The server stopped before this run finished.")
                write_json(path, job)
            self.jobs[job["id"]] = job
            self.next_number = max(self.next_number, job.get("number", 0) + 1)

    def engine_state(self, preset, tracking, traction):
        state = readiness(self.vexsim, self.cache, preset, tracking, traction)
        with self.lock:
            task = copy.deepcopy(self.preparing.get((preset, tracking, traction)))
        if task and task["status"] == "running":
            task["progress"] = self._read(Path(task["directory"]), "progress.json", task.get("progress"))
        return dict(preset=preset, tracking=tracking, traction=traction, compiled=state["compiled"],
                    feedforward=state["feedforward"], ready=state["feedforward"] is not None, task=task)

    def prepare(self, preset, tracking, traction):
        """Compile mclib and measure @p preset in the background, once."""
        robot = (preset, tracking, traction)
        with self.lock:
            task = self.preparing.get(robot)
            if task and task["status"] == "running":
                return self.engine_state(*robot)
            if readiness(self.vexsim, self.cache, *robot)["feedforward"] is not None:
                return self.engine_state(*robot)
            directory = Path(tempfile.mkdtemp(prefix="prepare-", dir=self.output))
            task = dict(status="running", started_at=time.time(), directory=str(directory), error=None)
            self.preparing[robot] = task
        thread = threading.Thread(target=self._prepare, args=(robot, task, directory), daemon=True)
        thread.start()
        return self.engine_state(*robot)

    def _prepare(self, robot, task, directory):
        preset, tracking, traction = robot
        command = engine_command("prepare", "--preset", preset, "--tracking", tracking, "--traction", traction,
                                 "--progress", directory / "progress.json", vexsim=self.vexsim, cache=self.cache)
        try:
            done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=self.wall_timeout)
            failed = done.returncode != 0
            error = (done.stderr.strip().splitlines() or ["no output"])[-1] if failed else None
        except subprocess.TimeoutExpired:
            failed, error = True, f"Preparing took longer than {self.wall_timeout} s and was stopped."
        with self.lock:
            task.update(status="error" if failed else "done", error=error, finished_at=time.time(),
                        progress=self._read(directory, "progress.json"))
        shutil.rmtree(directory, ignore_errors=True)

    def plan(self, raw):
        project = validate_project(raw)
        designed_poses(project)
        with tempfile.TemporaryDirectory(prefix="plan-", dir=self.output) as scratch:
            path = Path(scratch) / "project.json"
            write_json(path, project)
            command = engine_command("plan", "--project", path, vexsim=self.vexsim, cache=self.cache)
            try:
                done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
            except subprocess.TimeoutExpired:
                return dict(error="Planning took longer than 120 s and was stopped.")
        try:
            return json.loads(done.stdout)
        except ValueError:
            return dict(error="The planner crashed: " + (done.stderr.strip().splitlines() or ["no output"])[-1])

    def export(self, raw):
        project = validate_project(raw)
        designed_poses(project)
        answer = self.plan(project) if any(s["type"] == "path" and s["points"] for s in project["steps"]) else {}
        return dict(export(project, answer.get("settings"), answer.get("plans")),
                    plan_error=None if "settings" in answer else answer.get("error"))

    def write_export(self, body):
        if not isinstance(body, dict) or set(body) != {"project", "folder"}:
            raise ValueError("Send the project and the robot project folder.")
        folder = robot_project(body["folder"])
        exported = self.export(body["project"])
        if exported["drive_numbers"] == "placeholders":
            raise ValueError("Nothing was written. Until the paths are planned, their speeds, time limits and "
                             f"the drive numbers would be placeholders. The planner said: {exported['plan_error']}")
        named_in = mentioned_in(folder, exported["function"], exported["files"][0]["path"])
        return dict(folder=str(folder), function=exported["function"], mentioned_in=named_in,
                    files=write_files(folder, exported["files"]))

    def submit(self, raw):
        project = validate_project(raw)
        check_runnable(project)
        with self.lock:
            if self.shutdown_event.is_set() or self.pending.full():
                raise BusyError(f"{LIMITS['queue_size']} runs are already waiting. Wait for one to finish or cancel one.")
            identity = uuid.uuid4().hex
            destination = self.output / identity
            destination.mkdir()
            job = dict(id=identity, number=self.next_number, name=project["name"], status="queued",
                       created_at=time.time(), summary=None, error=None, error_stage=None, error_step=None,
                       artifact_dir=str(destination))
            self.next_number += 1
            write_json(destination / "project.json", project)
            write_json(destination / "job.json", job)
            self.jobs[identity] = job
            self.pending.put_nowait(identity)
            self._trim()
            return copy.deepcopy(job)

    def _trim(self):
        while len(self.jobs) > LIMITS["history_size"]:
            oldest = next((key for key, job in self.jobs.items() if job["status"] in FINISHED), None)
            if oldest is None:
                break
            del self.jobs[oldest]

    @staticmethod
    def _read(directory, name, fallback=None):
        try:
            return strict_json((Path(directory) / name).read_text())
        except (OSError, ValueError):
            return fallback

    @staticmethod
    def _frames(directory, start):
        try:
            text = (Path(directory) / "live.jsonl").read_text()
        except OSError:
            return []
        complete = text[:text.rfind("\n") + 1].splitlines()
        return [json.loads(line) for line in complete[start:]]

    def get(self, identity, frames_from=0):
        with self.lock:
            job = self.jobs.get(identity)
            if job is None:
                return None
            value = copy.deepcopy(job)
        directory = value["artifact_dir"]
        value["progress"] = self._read(directory, "progress.json")
        value["project"] = self._read(directory, "project.json")
        if value["status"] in FINISHED:
            value["result"] = self._read(directory, "result.json")
        if value.get("result") is None:
            live = self._read(directory, "live.json") or {}
            value["live"] = dict(steps=live.get("steps", []), pieces=live.get("pieces"),
                                 frames_from=frames_from, frames=self._frames(directory, frames_from))
        return value

    def list(self):
        with self.lock:
            return [{key: value for key, value in job.items() if key != "artifact_dir"}
                    for job in reversed(self.jobs.values())]

    def _stop_process(self, identity):
        process = self.processes.get(identity)
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def cancel(self, identity):
        with self.lock:
            job = self.jobs.get(identity)
            if job is None:
                return None
            if job["status"] not in FINISHED:
                job.update(status="cancelled", finished_at=time.time())
                write_json(Path(job["artifact_dir"]) / "job.json", job)
                self._stop_process(identity)
        return self.get(identity)

    def _worker(self):
        while not self.shutdown_event.is_set():
            try:
                identity = self.pending.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                self._execute(identity)
            finally:
                self.pending.task_done()

    def _execute(self, identity):
        with self.lock:
            job = self.jobs.get(identity)
            if job is None or job["status"] == "cancelled":
                return
            destination = Path(job["artifact_dir"])
            job.update(status="running", started_at=time.time())
            write_json(destination / "job.json", job)
        command = engine_command("run", "--project", destination / "project.json", "--output", destination,
                                 vexsim=self.vexsim, cache=self.cache)
        try:
            with (destination / "run.log").open("wb") as log:
                with self.lock:
                    if job["status"] == "cancelled" or self.shutdown_event.is_set():
                        return
                    process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True,
                                               env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
                    self.processes[identity] = process
                self._wait(job, process)
                with self.lock:
                    if job["status"] != "cancelled":
                        result = self._read(destination, "result.json")
                        failure = self._read(destination, "error.json", {})
                        if process.returncode in (0, 1) and isinstance(result, dict) and not job["error"]:
                            job.update(status="finished" if result["legal"] else "broke_rule", summary=result["summary"],
                                       finished_at=time.time())
                        else:
                            job.update(status="error", finished_at=time.time(),
                                       error=job["error"] or failure.get("error") or
                                       f"The run stopped with exit code {process.returncode}. See {destination / 'run.log'}.",
                                       error_stage=failure.get("stage"), error_step=failure.get("step"))
        except OSError as error:
            with self.lock:
                if job["status"] != "cancelled":
                    job.update(status="error", error=str(error), finished_at=time.time())
        finally:
            with self.lock:
                job.setdefault("finished_at", time.time())
                write_json(destination / "job.json", job)
                self.processes.pop(identity, None)
                self._trim()

    def _wait(self, job, process):
        deadline = time.monotonic() + self.wall_timeout
        kill_at = None
        while process.poll() is None:
            with self.lock:
                cancelled = job["status"] == "cancelled" or self.shutdown_event.is_set()
                expired = time.monotonic() > deadline
                if cancelled or expired:
                    if kill_at is None:
                        self._stop_process(job["id"])
                        kill_at = time.monotonic() + 2
                        if expired and not cancelled:
                            job["error"] = f"The run took longer than {self.wall_timeout} s of real time and was stopped."
                    elif time.monotonic() > kill_at:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
            time.sleep(.05)

    def close(self):
        self.shutdown_event.set()
        with self.lock:
            for identity, job in self.jobs.items():
                if job["status"] not in FINISHED:
                    self.cancel(identity)
        deadline = time.monotonic() + 5
        for worker in self.workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))


def examples():
    items = []
    for path in sorted(EXAMPLES.glob("*.json")):
        project = strict_json(path.read_text())
        items.append(dict(id=path.stem, name=project["name"], description=project.get("description", "")))
    return items


def example(name):
    path = EXAMPLES / f"{name}.json"
    if not re.fullmatch(r"[a-z0-9_]+", name) or not path.is_file():
        return None
    project = strict_json(path.read_text())
    project.pop("description", None)
    return validate_project(project)


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, port, planner):
        self.planner = planner
        self.static = {"/" + path.name: path for path in STATIC.iterdir() if path.suffix in CONTENT_TYPES}
        self.static["/"] = STATIC / "index.html"
        super().__init__(("127.0.0.1", port), Handler)

    def server_close(self):
        super().server_close()
        self.planner.close()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, format, *args):
        pass

    def send(self, status, payload, content_type="application/json; charset=utf-8", policy=None):
        data = payload if isinstance(payload, bytes) else json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", policy or PAGE_POLICY)
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        self.wfile.write(data)

    def local_request(self):
        port = self.server.server_port
        hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        given = self.headers.get_all("Host", [])
        if len(given) != 1 or given[0].lower() not in hosts:
            self.send(403, dict(error="Only requests to the local planner are accepted."))
            return False
        origins = self.headers.get_all("Origin", [])
        if len(origins) > 1 or (origins and origins[0] not in {"http://" + host for host in hosts}):
            self.send(403, dict(error="Requests from other sites are not accepted."))
            return False
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc:
            self.send(400, dict(error="Absolute request targets are not supported."))
            return False
        return True

    def do_GET(self):
        if not self.local_request():
            return
        parsed = urlsplit(self.path)
        path, query = parsed.path, parse_qs(parsed.query)
        planner = self.server.planner
        if path == "/api/config":
            self.send(200, dict(config(), examples=examples(), vexsim=str(planner.vexsim),
                                output=str(planner.output), field_drawings=planner.field_drawings))
        elif path == "/api/engine":
            preset, tracking, traction = (query.get(key, [""])[0] for key in ("preset", "tracking", "traction"))
            if preset not in PRESETS or tracking not in TRACKING or traction not in TRACTION:
                self.send(400, dict(error="Unknown robot, tracking or traction wheel setup."))
            else:
                self.send(200, planner.engine_state(preset, tracking, traction))
        elif re.fullmatch(r"/api/examples/[a-z0-9_]+", path):
            project = example(path.rsplit("/", 1)[1])
            self.send(200 if project else 404, project or dict(error="No such example."))
        elif path == "/api/runs":
            self.send(200, dict(runs=planner.list()))
        elif re.fullmatch(r"/api/runs/[0-9a-f]{32}", path):
            start = query.get("frames_from", ["0"])[0]
            job = planner.get(path.rsplit("/", 1)[1], int(start) if start.isdigit() else 0)
            self.send(200 if job else 404, job or dict(error="No such run."))
        elif path.startswith("/three/"):
            try:
                found = three_asset(path, planner.vexsim)
            except ThreeMissing as error:
                self.send(503, dict(error=str(error)))
                return
            if found is None:
                self.send(404, dict(error="No such 3D view file."))
            else:
                self.send(200, *found)
        elif path in CAD_FILES:
            name, content_type = CAD_FILES[path]
            file = planner.vexsim / "cad" / "override" / name
            if file.is_file():
                self.send(200, file.read_bytes(), content_type)
            else:
                self.send(503, dict(error=f"{file} is missing. The Override field shapes come with vexsim; "
                                          f"update it: git -C {planner.vexsim} pull"))
        elif path in self.server.static:
            file = self.server.static[path]
            self.send(200, file.read_bytes(), CONTENT_TYPES[file.suffix], FRAMED_POLICY if path in FRAMED_PAGES else None)
        else:
            self.send(404, dict(error="Not found."))

    def read_body(self):
        if self.headers.get("Transfer-Encoding"):
            raise ValueError("Transfer-Encoding is not supported.")
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or not re.fullmatch(r"[0-9]{1,7}", lengths[0]):
            raise LengthError("Send a single Content-Length.")
        size = int(lengths[0])
        if size > LIMITS["max_body_bytes"]:
            raise LengthError(f"The request is larger than {LIMITS['max_body_bytes'] // 1024} KiB.")
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
            raise ValueError("Content-Type must be application/json.")
        data = self.rfile.read(size)
        if len(data) != size:
            raise ValueError("The request body was cut short.")
        return strict_json(data.decode("utf-8"))

    def do_POST(self):
        if not self.local_request():
            return
        planner = self.server.planner
        path = urlsplit(self.path).path
        try:
            body = self.read_body()
            if path == "/api/runs":
                self.send(202, planner.submit(body))
            elif path == "/api/plan":
                answer = planner.plan(body)
                self.send(409 if answer.get("not_ready") else 400 if "error" in answer else 200, answer)
            elif path == "/api/export":
                self.send(200, planner.export(body))
            elif path == "/api/export/write":
                self.send(200, planner.write_export(body))
            elif path == "/api/validate":
                project = validate_project(body)
                designed_poses(project)
                self.send(200, project)
            elif path == "/api/prepare":
                if not isinstance(body, dict) or set(body) != {"preset", "tracking", "traction"}:
                    raise ValueError("Send the preset, tracking and traction wheels to prepare.")
                if body["preset"] not in PRESETS or body["tracking"] not in TRACKING or \
                        body["traction"] not in TRACTION:
                    raise ValueError("Unknown robot, tracking or traction wheel setup.")
                check_traction(body["preset"], body["traction"], "Robot")
                self.send(202, planner.prepare(body["preset"], body["tracking"], body["traction"]))
            elif re.fullmatch(r"/api/runs/[0-9a-f]{32}/cancel", path):
                if body != {}:
                    raise ValueError("Send an empty object to cancel a run.")
                job = planner.cancel(path.split("/")[3])
                self.send(200 if job else 404, job or dict(error="No such run."))
            else:
                self.send(404, dict(error="Not found."))
        except BusyError as error:
            self.send(429, dict(error=str(error)))
        except LengthError as error:
            self.send(413 if "larger" in str(error) else 411, dict(error=str(error)))
        except ProjectError as error:
            self.send(400, dict(error=str(error), step=error.step))
        except (ValueError, UnicodeError, RecursionError) as error:
            self.send(400, dict(error=str(error)))
        except TimeoutError:
            self.send(408, dict(error="The request body took too long to arrive."))


class LengthError(Exception):
    pass


def vexsim_problems(vexsim):
    """Why the planner cannot run on @p vexsim with this Python, one line each."""
    if not (vexsim / "vexsim/sim.py").is_file():
        return [f"--vexsim {vexsim} is not a vexsim checkout (no vexsim/sim.py)."]
    problems = []
    if not (vexsim / "vexsim/override.py").is_file():
        problems.append(f"{vexsim} has no Override field (vexsim/override.py). Pull its main branch.")
    missing = [name for name in ("mujoco", "numpy") if importlib.util.find_spec(name) is None]
    if missing:
        problems.append(f"This Python ({sys.executable}) cannot import {' or '.join(missing)}. Install the "
                        f"simulator's packages: {sys.executable} -m pip install -r {ROOT / 'tests/vexsim/requirements.txt'}")
    shapes = vexsim / "cad" / "override"
    if not all((shapes / name).is_file() for name in ("shapes.json", "field.json")):
        problems.append(f"{shapes} is missing the Override field shapes, which come with vexsim. "
                        f"Update it: git -C {vexsim} pull")
    try:
        three_asset("/three/three.module.js", vexsim)
    except ThreeMissing as error:
        problems.append(str(error))
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8765, help="port on 127.0.0.1 (default 8765)")
    parser.add_argument("--output", type=Path, default=ROOT / "bin" / "path-planner",
                        help="where runs and the compiled library are kept (default: bin/path-planner)")
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim", help="the vexsim checkout")
    parser.add_argument("--no-prepare", action="store_true",
                        help="do not start compiling mclib and measuring the default robot at launch")
    parser.add_argument("--parallel-runs", type=int, default=PARALLEL_RUNS,
                        help=f"runs simulated at once (default {PARALLEL_RUNS}, a third of this machine's CPU threads)")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.parallel_runs < 1:
        parser.error("--parallel-runs must be at least 1")
    problems = vexsim_problems(args.vexsim)
    if problems:
        parser.error("\n".join(problems))
    planner = Planner(args.output, args.vexsim, parallel_runs=args.parallel_runs)
    planner.field_drawings  # built before serving, so a vexsim that cannot draw the fields stops here
    try:
        server = LocalServer(args.port, planner)
    except OSError:
        planner.close()
        raise
    print(f"mclib path planner: http://127.0.0.1:{args.port}\nRuns and the compiled library: {planner.output}",
          flush=True)
    if not args.no_prepare:
        robot = config()["default_project"]["robot"]
        planner.prepare(robot["preset"], robot["tracking"], robot["traction"])
        print("Compiling mclib and measuring the default robot in the background "
              "(1 to 2 minutes the first time).", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
