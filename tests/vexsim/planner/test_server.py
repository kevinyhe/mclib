"""The planner's HTTP server: what it accepts, what it refuses, and how it
queues, cancels and restores runs. No C++ or physics runs here."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import server  # noqa: E402
from project import LIMITS, ProjectError, validate_project  # noqa: E402

ONE_POINT = dict(steps=[dict(type="path", points=[dict(x=0, y=0)], sections=[dict(speed_percent=None, reverse=False)])])


class StubPlanner:
    def __init__(self):
        self.vexsim, self.output = Path("/vexsim"), Path("/output")
        self.field_drawings = dict(override=dict(half_in=70.2), push_back=dict(half_in=70.2), open=None)
        self.submitted = []

    def engine_state(self, preset, tracking):
        return dict(preset=preset, tracking=tracking, ready=False, task=None)

    def prepare(self, preset, tracking):
        return dict(preset=preset, tracking=tracking, ready=False, task=dict(status="running"))

    def plan(self, raw):
        validate_project(raw)
        return dict(not_ready=True, error="not compiled")

    def export(self, raw):
        return dict(code="void autonomous() {}", measured=False, note=None)

    def submit(self, raw):
        project = validate_project(raw)
        server.check_runnable(project)
        self.submitted.append(project)
        return dict(id="a" * 32, number=1, status="queued")

    def get(self, identity, frames_from=0):
        return dict(id=identity, frames_from=frames_from) if identity == "a" * 32 else None

    def list(self):
        return []

    def cancel(self, identity):
        return self.get(identity)

    def close(self):
        pass


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.planner = StubPlanner()
        self.server = server.LocalServer(0, self.planner)
        self.port = self.server.server_port
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs=dict(poll_interval=.05), daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        data = body if isinstance(body, bytes) or body is None else json.dumps(body).encode()
        sent = {"Content-Type": "application/json"} if data is not None else {}
        sent.update(headers or {})
        connection.request(method, path, body=data, headers=sent)
        response = connection.getresponse()
        payload = response.read()
        connection.close()
        return response.status, response.getheader("Content-Type"), payload

    def status_for_declared_length(self, length):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.putrequest("POST", "/api/runs")
        connection.putheader("Content-Type", "application/json")
        connection.putheader("Content-Length", str(length))
        connection.endheaders()
        status = connection.getresponse().status
        connection.close()
        return status

    def json(self, method, path, body=None, headers=None):
        status, _, payload = self.request(method, path, body, headers)
        return status, json.loads(payload)

    def test_listens_on_loopback_only(self):
        self.assertEqual(self.server.server_address[0], "127.0.0.1")

    def test_config_examples_and_validate(self):
        status, config = self.json("GET", "/api/config")
        self.assertEqual(status, 200)
        self.assertEqual({item["id"] for item in config["examples"]},
                         {"s_curve", "speed_zones", "out_and_back", "mixed_steps", "wheel_slip", "override_autonomous",
                          "override_stack_to_goal", "override_roller", "override_match_load"})
        status, project = self.json("GET", "/api/examples/s_curve")
        self.assertEqual(status, 200)
        self.assertNotIn("description", project)
        self.assertEqual(self.json("GET", "/api/examples/..%2fserver")[0], 404)
        status, project = self.json("POST", "/api/validate", ONE_POINT)
        self.assertEqual((status, project["steps"][0]["volts"]), (200, 12))

    def test_bad_routines_come_back_with_the_step(self):
        status, answer = self.json("POST", "/api/runs", dict(steps=[dict(type="turn", heading="left")]))
        self.assertEqual(status, 400)
        self.assertEqual(answer, dict(error="Step 1 (Turn to heading): Heading must be a number", step=0))
        status, answer = self.json("POST", "/api/runs", dict(steps=[dict(type="path", points=[], sections=[])]))
        self.assertEqual((status, answer["step"]), (400, 0))
        self.assertIn("has no points yet", answer["error"])
        self.assertEqual(self.json("POST", "/api/runs", ONE_POINT)[0], 202)
        self.assertEqual(self.json("POST", "/api/plan", ONE_POINT)[0], 409)

    def test_refuses_other_hosts_origins_and_bodies(self):
        self.assertEqual(self.request("GET", "/api/config", headers={"Host": "example.com"})[0], 403)
        self.assertEqual(self.request("POST", "/api/runs", ONE_POINT, {"Origin": "http://example.com"})[0], 403)
        self.assertEqual(self.request("POST", "/api/runs", b"{}", {"Content-Type": "text/plain"})[0], 400)
        self.assertEqual(self.request("POST", "/api/runs", b'{"name": "a", "name": "b"}')[0], 400)
        self.assertEqual(self.status_for_declared_length(LIMITS["max_body_bytes"] + 1), 413)
        self.assertEqual(self.request("POST", "/api/prepare", dict(preset="huge", tracking="two"))[0], 400)
        self.assertEqual(self.request("POST", f"/api/runs/{'a' * 32}/cancel", dict(now=True))[0], 400)

    def test_serves_only_the_page_files(self):
        status, content_type, page = self.request("GET", "/")
        self.assertEqual((status, content_type), (200, "text/html; charset=utf-8"))
        self.assertIn(b"planner.js", page)
        for name in ("/planner.js", "/geometry.js", "/planner.css"):
            self.assertEqual(self.request("GET", name)[0], 200)
        for name in ("/server.py", "/../server.py", "/static/planner.js", "/examples/s_curve.json"):
            self.assertEqual(self.request("GET", name)[0], 404, name)

    def test_page_policy_blocks_outside_scripts(self):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request("GET", "/")
        response = connection.getresponse()
        response.read()
        policy = response.getheader("Content-Security-Policy")
        connection.close()
        self.assertIn("script-src 'self'", policy)
        self.assertIn("connect-src 'self'", policy)
        self.assertIn("frame-ancestors 'none'", policy)

    def policy(self, path):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request("GET", path)
        response = connection.getresponse()
        response.read()
        connection.close()
        return response.status, response.getheader("Content-Security-Policy")

    def test_only_the_3d_field_may_be_framed_and_only_by_the_planner(self):
        status, policy = self.policy("/field3d.html")
        self.assertEqual(status, 200)
        self.assertIn("frame-ancestors 'self'", policy)
        self.assertIn("script-src 'self';", policy)
        self.assertIn("frame-ancestors 'none'", self.policy("/field3d.js")[1])

    def test_serves_only_the_3d_views_cad_meshes_from_the_vexsim_checkout(self):
        with tempfile.TemporaryDirectory() as folder:
            self.planner.vexsim = Path(folder)
            status, payload = self.json("GET", "/cad/view.json")
            self.assertEqual(status, 503)
            self.assertIn("tools/override_cad.py", payload["error"])
            made = Path(folder) / "cad" / "override"
            made.mkdir(parents=True)
            (made / "view.json").write_text('{"meshes": {}}')
            (made / "view.bin").write_bytes(b"\x00\x01")
            (made / "shapes.json").write_text("{}")
            self.assertEqual(self.request("GET", "/cad/view.json"), (200, "application/json; charset=utf-8", b'{"meshes": {}}'))
            self.assertEqual(self.request("GET", "/cad/view.bin"), (200, "application/octet-stream", b"\x00\x01"))
            for name in ("/cad/shapes.json", "/cad/../cad/override/view.json", "/cad/"):
                self.assertEqual(self.request("GET", name)[0], 404, name)


def sleeping_engine(*arguments, vexsim, cache):
    return [sys.executable, "-c", "import time; time.sleep(30)"]


class QueueTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.output = Path(self.folder.name)

    def tearDown(self):
        self.folder.cleanup()

    def wait_for(self, planner, identity, statuses):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            job = planner.get(identity)
            if job["status"] in statuses:
                return job
            time.sleep(.05)
        self.fail(f"run {identity} never reached {statuses}")

    def test_cancel_stops_the_run_process(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output)
            try:
                job = planner.submit(ONE_POINT)
                self.wait_for(planner, job["id"], {"running"})
                started = time.monotonic()
                planner.cancel(job["id"])
                while planner.process is not None and time.monotonic() - started < 5:
                    time.sleep(.05)
                self.assertIsNone(planner.process)
                self.assertLess(time.monotonic() - started, 3)
                self.assertEqual(planner.get(job["id"])["status"], "cancelled")
            finally:
                planner.close()

    def test_queue_is_bounded(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output)
            try:
                with self.assertRaises(server.BusyError):
                    for _ in range(LIMITS["queue_size"] + 2):
                        planner.submit(ONE_POINT)
            finally:
                planner.close()

    def test_runs_left_unfinished_are_restored_as_errors(self):
        identity = "b" * 32
        (self.output / identity).mkdir()
        (self.output / identity / "job.json").write_text(json.dumps(dict(id=identity, number=7, status="running")))
        planner = server.Planner(self.output, self.output)
        try:
            job = planner.get(identity)
            self.assertEqual(job["status"], "error")
            self.assertEqual(job["error"], "The server stopped before this run finished.")
            self.assertEqual(planner.submit(ONE_POINT)["number"], 8)
        finally:
            planner.close()

    def test_empty_path_is_refused_before_queueing(self):
        planner = server.Planner(self.output, self.output)
        try:
            with self.assertRaises(ProjectError):
                planner.submit(dict(steps=[dict(type="path", points=[], sections=[])]))
            self.assertEqual(planner.list(), [])
        finally:
            planner.close()


if __name__ == "__main__":
    unittest.main()
