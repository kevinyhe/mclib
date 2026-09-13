#!/usr/bin/env python3
"""Serve the local motion builder on http://127.0.0.1:8765 (standard library only)."""
import argparse
from collections import OrderedDict
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import urlsplit
import uuid

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from runner import LIMITS, ROOT, config, strict_json, validate_spec, write_json
from native_viewer import NativeViewerError, asset as native_asset

FINISHED = {"passed", "failed", "cancelled", "error"}


class BusyError(Exception):
    pass


class JobManager:
    """A single worker owns subprocess creation; it never loads C++ itself."""
    def __init__(self, output, vexsim, wall_timeout=300):
        self.output, self.vexsim = Path(output).resolve(), Path(vexsim).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.wall_timeout = wall_timeout
        self.jobs = OrderedDict()
        self.lock = threading.RLock()
        self.pending = queue.Queue(maxsize=LIMITS["queue_size"])
        self.shutdown_event = threading.Event()
        self.process = None
        self.active_id = None
        self._restore()
        self.worker = threading.Thread(target=self._worker, name="motion-worker", daemon=True)
        self.worker.start()

    def _restore(self):
        paths = sorted(self.output.glob("[0-9a-f]" * 32 + "/job.json"),
                       key=lambda path: path.stat().st_mtime)[-LIMITS["history_size"]:]
        for path in paths:
            try:
                job = strict_json(path.read_text())
                if not isinstance(job, dict) or job.get("id") != path.parent.name:
                    continue
                job["artifact_dir"] = str(path.parent)
                if job.get("status") not in FINISHED:
                    job.update(status="error", error="Server stopped before this run finished")
                    write_json(path, job)
                self.jobs[job["id"]] = job
            except (OSError, ValueError):
                continue

    def submit(self, raw):
        spec = validate_spec(raw)
        with self.lock:
            if self.shutdown_event.is_set() or self.pending.full():
                raise BusyError("The run queue is full; wait for a run to finish or cancel one")
            identity = uuid.uuid4().hex
            destination = self.output / identity
            destination.mkdir()
            job = dict(id=identity, status="queued", spec=spec, created_at=time.time(),
                       progress=dict(phase="queued", step=0, total=len(spec["steps"])),
                       result=None, error=None, artifact_dir=str(destination))
            write_json(destination / "spec.json", spec)
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

    def _read(self, job, name, fallback=None):
        try:
            return strict_json((Path(job["artifact_dir"]) / name).read_text())
        except (OSError, ValueError):
            return fallback

    def get(self, identity, include_result=True):
        with self.lock:
            job = self.jobs.get(identity)
            if job is None:
                return None
            value = copy.deepcopy(job)
            if job["status"] == "running":
                value["progress"] = self._read(job, "progress.json", value["progress"])
            if include_result:
                value["partial"] = self._read(job, "partial.json")
            if include_result and job["status"] in FINISHED:
                value["result"] = self._read(job, "result.json")
            return value

    def list(self):
        with self.lock:
            return [self.get(key, include_result=False) for key in reversed(self.jobs)]

    def _stop_process(self):
        if self.process is not None and self.process.poll() is None:
            try:
                # start_new_session makes this group exclusive to this job and
                # its compiler children. No other run or user process belongs to it.
                os.killpg(self.process.pid, signal.SIGTERM)
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
                if self.active_id == identity:
                    self._stop_process()
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
            self.active_id = identity
        command = [sys.executable, "-B", str(HERE / "runner.py"), "--spec", str(destination / "spec.json"),
                   "--output", str(destination), "--vexsim", str(self.vexsim),
                   "--cache", str(self.output / "cache")]
        try:
            with (destination / "run.log").open("wb") as log:
                with self.lock:
                    if job["status"] == "cancelled" or self.shutdown_event.is_set():
                        return
                    self.process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                                    start_new_session=True,
                                                    env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
                    process = self.process
                deadline = time.monotonic() + self.wall_timeout
                termination_deadline = None
                while process.poll() is None:
                    with self.lock:
                        cancel = job["status"] == "cancelled" or self.shutdown_event.is_set()
                        expired = time.monotonic() > deadline
                        if cancel or expired:
                            self._stop_process()
                            if termination_deadline is None:
                                termination_deadline = time.monotonic() + 2
                                if expired and not cancel:
                                    job["error"] = f"Run exceeded {self.wall_timeout} seconds of wall time"
                            elif time.monotonic() > termination_deadline:
                                try:
                                    os.killpg(process.pid, signal.SIGKILL)
                                except ProcessLookupError:
                                    pass
                    self.shutdown_event.wait(.1) if not self.shutdown_event.is_set() else time.sleep(.05)
                with self.lock:
                    if job["status"] != "cancelled":
                        result = self._read(job, "result.json")
                        error = self._read(job, "error.json", {})
                        job["error"] = job["error"] or error.get("error")
                        if process.returncode in (0, 1) and isinstance(result, dict) and not job["error"]:
                            job["status"] = "passed" if process.returncode == 0 and result.get("passed") else "failed"
                        else:
                            job.update(status="error", error=job["error"] or f"Runner exited with code {process.returncode}; see run.log")
                        job["progress"] = self._read(job, "progress.json", job["progress"])
        except Exception as error:
            with self.lock:
                if job["status"] != "cancelled":
                    job.update(status="error", error=str(error))
        finally:
            with self.lock:
                job["finished_at"] = time.time()
                write_json(destination / "job.json", job)
                self.process = None
                self.active_id = None
                self._trim()

    def close(self):
        self.shutdown_event.set()
        with self.lock:
            for identity, job in self.jobs.items():
                if job["status"] not in FINISHED:
                    self.cancel(identity)
            self._stop_process()
        self.worker.join(timeout=5)


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, port, manager):
        self.manager = manager
        super().__init__(("127.0.0.1", port), Handler)

    def server_close(self):
        super().server_close()
        self.manager.close()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, format, *args):
        pass

    def send(self, status, payload, content_type="application/json; charset=utf-8", csp=None):
        data = payload if isinstance(payload, bytes) else json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", csp or "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        self.wfile.write(data)

    def local_request(self):
        port = self.server.server_port
        authorities = {f"127.0.0.1:{port}", f"localhost:{port}"}
        hosts = self.headers.get_all("Host", [])
        if len(hosts) != 1 or hosts[0].lower() not in authorities:
            self.send(403, dict(error="Only the local builder host is accepted"))
            return False
        origins = self.headers.get_all("Origin", [])
        if len(origins) > 1 or (origins and origins[0] not in {"http://" + host for host in authorities}):
            self.send(403, dict(error="Cross-origin requests are not accepted"))
            return False
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc:
            self.send(400, dict(error="Absolute request targets are not supported"))
            return False
        return True

    def do_GET(self):
        if not self.local_request():
            return
        path = urlsplit(self.path).path
        if path == "/api/config":
            self.send(200, config())
        elif path == "/api/runs":
            self.send(200, dict(runs=self.server.manager.list()))
        elif re.fullmatch(r"/api/runs/[0-9a-f]{32}", path):
            job = self.server.manager.get(path.rsplit("/", 1)[1])
            self.send(200 if job else 404, job or dict(error="Run not found"))
        elif path.startswith("/native/"):
            try:
                asset = native_asset(path, self.server.manager.vexsim)
                if asset is None:
                    self.send(404, dict(error="Native viewer asset not found"))
                else:
                    body, content_type, csp = asset
                    self.send(200, body, content_type, csp)
            except NativeViewerError as error:
                self.send(503, dict(error=str(error)))
        else:
            static = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/index.html": ("index.html", "text/html; charset=utf-8"),
                      "/builder.css": ("builder.css", "text/css; charset=utf-8"),
                      "/builder.js": ("builder.js", "text/javascript; charset=utf-8")}
            if path not in static:
                self.send(404, dict(error="Not found"))
                return
            filename, content_type = static[path]
            try:
                self.send(200, (HERE / filename).read_bytes(), content_type)
            except FileNotFoundError:
                self.send(404, dict(error="Frontend asset is unavailable"))

    def do_POST(self):
        if not self.local_request():
            return
        if self.headers.get("Transfer-Encoding"):
            self.send(400, dict(error="Transfer-Encoding is unsupported"))
            return
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or not re.fullmatch(r"[0-9]+", lengths[0]):
            self.send(411, dict(error="A single Content-Length is required"))
            return
        if len(lengths[0]) > 6:
            self.send(413, dict(error="Request body exceeds 64 KiB"))
            return
        size = int(lengths[0])
        if size > LIMITS["max_body_bytes"]:
            self.send(413, dict(error="Request body exceeds 64 KiB"))
            return
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
            self.send(415, dict(error="Content-Type must be application/json"))
            return
        try:
            data = self.rfile.read(size)
            if len(data) != size:
                raise ValueError("Incomplete request body")
            body = strict_json(data.decode("utf-8"))
            path = urlsplit(self.path).path
            if path == "/api/runs":
                self.send(202, self.server.manager.submit(body))
            elif re.fullmatch(r"/api/runs/[0-9a-f]{32}/cancel", path):
                if body != {}:
                    raise ValueError("Cancellation body must be an empty object")
                job = self.server.manager.cancel(path.split("/")[3])
                self.send(200 if job else 404, job or dict(error="Run not found"))
            else:
                self.send(404, dict(error="Not found"))
        except BusyError as error:
            self.send(429, dict(error=str(error)))
        except (ValueError, UnicodeError, RecursionError) as error:
            self.send(400, dict(error=str(error)))
        except TimeoutError:
            self.send(408, dict(error="Request body timed out"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not (args.vexsim / "vexsim/sim.py").is_file():
        parser.error("--vexsim must name the existing simulator repository")
    output = args.output or Path(tempfile.mkdtemp(prefix="mclib-builder-"))
    manager = JobManager(output, args.vexsim)
    try:
        server = LocalServer(args.port, manager)
    except OSError:
        manager.close()
        raise
    print(f"Motion builder: http://127.0.0.1:{args.port}\nArtifacts: {manager.output}", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
