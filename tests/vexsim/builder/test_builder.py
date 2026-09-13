#!/usr/bin/env python3
"""Fast schema, HTTP boundary and isolated-worker lifecycle regressions."""
import copy
import fcntl
import http.client
import itertools
import json
import math
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
from runner import DEFAULTS, LIMITS, LivePublisher, execute, invoke, motion_target, strict_json, validate_spec
from server import BusyError, JobManager, LocalServer


class SchemaTests(unittest.TestCase):
    def test_defaults_are_valid_and_not_mutated(self):
        before = copy.deepcopy(DEFAULTS)
        self.assertEqual(validate_spec(DEFAULTS), DEFAULTS)
        self.assertEqual(DEFAULTS, before)

    def test_rejects_nonfinite_bool_unknown_path_and_unbounded_work(self):
        variants = []
        for value in (float("nan"), float("inf"), True, "24", 10 ** 400):
            candidate = copy.deepcopy(DEFAULTS)
            candidate["steps"][0]["x"] = value
            variants.append(candidate)
        for key, value in (("preset", "../../etc/passwd"), ("tracking_mode", "perfect"),
                           ("stop_on_failure", "false"), ("output", "/tmp/arbitrary"),
                           ("steps", []), ("steps", DEFAULTS["steps"] * 17)):
            candidate = copy.deepcopy(DEFAULTS)
            candidate[key] = value
            variants.append(candidate)
        candidate = copy.deepcopy(DEFAULTS)
        candidate["steps"] = [dict(type="drive", distance=1, timeout_ms=20000)] * 6
        variants.append(candidate)
        for candidate in variants:
            with self.subTest(candidate=str(candidate)[:120]):
                with self.assertRaises(ValueError):
                    validate_spec(candidate)

    def test_json_rejects_duplicates_and_extensions(self):
        for text in ('{"steps": [], "steps": []}', '{"x":NaN}', '{"x":Infinity}'):
            with self.assertRaises(ValueError):
                strict_json(text)

    def test_wall_reset_frame_boundary_and_boomerang_lead(self):
        with self.assertRaisesRegex(ValueError, "must be last"):
            validate_spec(dict(steps=[dict(type="wall_reset", x=0, y=0, heading=0),
                                      dict(type="drive", distance=1)]))
        with self.assertRaisesRegex(ValueError, "less than 1"):
            validate_spec(dict(steps=[dict(type="boomerang", x=1, y=1, heading=0, lead=1)]))

    def test_reverse_arc_contradiction_never_calls_controller(self):
        bridge = mock.Mock()
        bridge.pose.return_value = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, "Reverse arc requires"):
            invoke(bridge, dict(type="reverse_arc", heading=90, radius=24))
        bridge.run.assert_not_called()

    def test_world_targets_use_live_arc_entry_and_drive_command_heading(self):
        drive = motion_target(dict(type="drive", distance=24), [10, 20, 80], 90)
        self.assertAlmostEqual(drive["x"], 34)
        self.assertAlmostEqual(drive["y"], 20)
        arc = motion_target(dict(type="arc", radius=24, heading=180), [10, 20, 90], 0)
        self.assertAlmostEqual(arc["x"], 34)
        self.assertAlmostEqual(arc["y"], -4)
        reverse = motion_target(dict(type="reverse_arc", radius=24, heading=-90), [0, 0, 0], 0)
        self.assertAlmostEqual(reverse["x"], 24)
        self.assertAlmostEqual(reverse["y"], -24)


class StubManager:
    def __init__(self):
        self.submitted = []
        self.vexsim = HERE.parents[3] / "vexsim"

    def submit(self, body):
        self.submitted.append(validate_spec(body))
        return dict(id="a" * 32, status="queued")

    def get(self, identity):
        return None

    def list(self):
        return []

    def cancel(self, identity):
        return dict(id=identity, status="cancelled")

    def close(self):
        pass


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manager = StubManager()
        cls.server = LocalServer(0, cls.manager)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def request(self, method, path, body=None, headers=None):
        client = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        client.request(method, path, body=body, headers=headers or {})
        response = client.getresponse()
        result = response.status, response.read(), dict(response.getheaders())
        client.close()
        return result

    def test_config_and_submission(self):
        status, data, headers = self.request("GET", "/api/config")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(data)["limits"]["max_steps"], 16)
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")
        self.assertEqual(self.request("POST", "/api/runs", json.dumps(DEFAULTS),
                                     {"Content-Type": "application/json"})[0], 202)

    def test_host_origin_and_content_type_boundaries(self):
        for headers in ({"Host": "attacker.example"},
                        {"Origin": "https://attacker.example"}, {"Origin": "null"}):
            self.assertEqual(self.request("GET", "/api/config", headers=headers)[0], 403)
        self.assertEqual(self.request("POST", "/api/runs", "{}",
                                     {"Content-Type": "text/plain"})[0], 415)
        self.assertEqual(self.request("POST", "/api/runs", "{}",
                                     {"Content-Type": "application/json", "Transfer-Encoding": "chunked"})[0], 400)
        self.assertEqual(self.request("POST", "/api/runs", "{}",
                                     {"Content-Type": "application/json", "Content-Length": "65537"})[0], 413)

    def test_static_allowlist_and_invalid_scenario(self):
        for path in ("/../server.py", "/%2e%2e/server.py", "/server.py", "/api/runs/../../README.md"):
            self.assertEqual(self.request("GET", path)[0], 404)
        self.assertEqual(self.request("POST", "/api/runs", '{"steps":NaN}',
                                     {"Content-Type": "application/json"})[0], 400)
        self.assertEqual(self.request("POST", "/api/runs/" + "a" * 32 + "/cancel", "{}",
                                     {"Content-Type": "application/json"})[0], 200)

    def test_native_viewer_is_same_origin_and_assets_are_allowlisted(self):
        status, data, headers = self.request("GET", "/native/viewer.html")
        self.assertEqual(status, 200)
        self.assertIn("frame-ancestors 'self'", headers["Content-Security-Policy"])
        self.assertNotIn(b"https://cdn", data)
        self.assertNotIn("'unsafe-inline'", headers["Content-Security-Policy"])
        for path in ("/native/viewer.js", "/native/viewer.css", "/native/field.json",
                     "/native/three/build/three.module.js"):
            self.assertEqual(self.request("GET", path)[0], 200, path)
        for path in ("/native/../server.py", "/native/%2e%2e/server.py",
                     "/native/three/package.json", "/native/three/../../server.py",
                     "/native/model.fbx", "/state", "/input"):
            self.assertEqual(self.request("GET", path)[0], 404, path)
        self.assertEqual(self.request("POST", "/input", "{}",
                                     {"Content-Type": "application/json"})[0], 404)


class JobTests(unittest.TestCase):
    def test_live_publisher_is_wall_clock_throttled_and_atomic(self):
        with tempfile.TemporaryDirectory(prefix="mclib-live-test-") as directory:
            snapshot = mock.Mock(return_value=dict(trace=[dict(t=0)], live=True))
            publisher = LivePublisher(directory, snapshot)
            with mock.patch("runner.time.monotonic", return_value=10):
                publisher.publish()
                publisher.publish()
            self.assertEqual(snapshot.call_count, 1)
            with mock.patch("runner.time.monotonic", return_value=10.11):
                publisher.publish()
            self.assertEqual(snapshot.call_count, 2)
            publisher.publish(force=True)
            self.assertEqual(snapshot.call_count, 3)
            self.assertTrue(strict_json((Path(directory) / "partial.json").read_text())["live"])
            self.assertFalse((Path(directory) / "partial.json.tmp").exists())

    def test_drive_after_failed_turn_uses_actual_cpp_heading_target(self):
        observed = []
        class FakeBridge:
            def __init__(self, *args, **kwargs):
                self.sim = type("Sim", (), {"t": 0.0})()
                self.control_trace = []
                self.position = [0.0, 0.0, 0.0]
                self.heading = 0
            def pose(self):
                return list(self.position)
            truth = pose
            def commanded_heading(self):
                return self.heading
            def capture(self):
                return dict(t=self.sim.t, true_x=self.position[0], true_y=self.position[1],
                            true_heading=self.position[2], estimated_x=self.position[0],
                            estimated_y=self.position[1], estimated_heading=self.position[2],
                            odometry_error_in=0, left_volts=0, right_volts=0)
            def run(self, action, **kwargs):
                if action == 0:
                    self.sim.t += .05
                    self.heading = self.position[2] = 7
                else:
                    self.sim.t += .5
                    self.position[0] += kwargs["a"] * math.sin(math.radians(self.heading))
                    self.position[1] += kwargs["a"] * math.cos(math.radians(self.heading))
                self.control_trace.append(self.capture())
                self.on_capture(self.control_trace[-1])
                # This is inside the blocking action, before it returns.
                observed.append(strict_json((root / "partial.json").read_text()))
                return 0
            def advance_idle(self, ms):
                self.sim.t += ms / 1000
                self.control_trace.append(self.capture())
                self.on_capture(self.control_trace[-1])
                observed.append(strict_json((root / "partial.json").read_text()))
            def stopped(self):
                return True
        spec = dict(stop_on_failure=False, steps=[dict(type="turn", heading=90, timeout_ms=50),
                                                 dict(type="drive", distance=12)])
        with tempfile.TemporaryDirectory(prefix="mclib-builder-test-") as directory:
            root = Path(directory)
            def build(path):
                (path / "build.log").write_text("")
                return object()
            with mock.patch("runner.source_fingerprint", return_value="revision"), mock.patch("runner.harness.build", side_effect=build), mock.patch("runner.harness.PhysicsBridge", FakeBridge), mock.patch("runner.time.monotonic", side_effect=itertools.count(step=.2)):
                result = execute(spec, root, HERE)
            self.assertFalse(result["steps"][0]["passed"])
            self.assertTrue(result["steps"][1]["passed"])
            self.assertEqual(result["steps"][1]["target"]["heading"], 7)
            self.assertEqual([item["phase"] for item in observed], ["motion", "hold", "motion", "hold"])
            self.assertEqual([item["active_step"] for item in observed], [0, 0, 1, 1])
            self.assertEqual(observed[0]["steps"][-1]["status"], "running")
            self.assertEqual(observed[2]["steps"][-1]["target"]["heading"], 7)
            self.assertAlmostEqual(observed[0]["trace"][-1]["t"], .05)
            self.assertEqual(observed[1]["trace"][-1]["phase"], "hold")
            self.assertFalse(strict_json((root / "partial.json").read_text())["live"])

    def test_source_change_during_compile_never_publishes_cache(self):
        with tempfile.TemporaryDirectory(prefix="mclib-builder-test-") as directory:
            root = Path(directory)
            def fake_build(path):
                (path / "libmclib_vexsim.so").write_bytes(b"fixture")
                (path / "build.log").write_text("")
            with mock.patch("runner.source_fingerprint", side_effect=["before", "after"]), mock.patch("runner.harness.build", side_effect=fake_build):
                with self.assertRaisesRegex(RuntimeError, "changed during compilation"):
                    execute(DEFAULTS, root / "run", HERE, root / "cache")
            self.assertFalse((root / "cache/before/complete.json").exists())

    def test_bounded_queue_cancellation_and_restore(self):
        with tempfile.TemporaryDirectory(prefix="mclib-builder-test-") as directory:
            with mock.patch.object(JobManager, "_worker", return_value=None):
                manager = JobManager(directory, HERE)
                jobs = [manager.submit(DEFAULTS) for _ in range(LIMITS["queue_size"])]
                with self.assertRaises(BusyError):
                    manager.submit(DEFAULTS)
                self.assertEqual(manager.cancel(jobs[0]["id"])["status"], "cancelled")
                self.assertTrue((Path(jobs[0]["artifact_dir"]) / "spec.json").is_file())
                manager.close()
                restored = JobManager(directory, HERE)
                self.assertEqual(len(restored.list()), LIMITS["queue_size"])
                self.assertTrue(all(job["status"] == "cancelled" for job in restored.list()))
                restored.close()

    def test_worker_process_isolation_and_cancel_only_its_group(self):
        class Process:
            pid = 123456789
            returncode = None
            def poll(self):
                return self.returncode
        process = Process()
        started = threading.Event()
        def launch(*args, **kwargs):
            self.assertTrue(kwargs["start_new_session"])
            self.assertNotIn("shell", kwargs)
            started.set()
            return process
        def terminate(pid, signum):
            self.assertEqual(pid, process.pid)
            process.returncode = -signum
        with tempfile.TemporaryDirectory(prefix="mclib-builder-test-") as directory:
            with mock.patch("server.subprocess.Popen", side_effect=launch), mock.patch("server.os.killpg", side_effect=terminate):
                manager = JobManager(directory, HERE)
                try:
                    job = manager.submit(DEFAULTS)
                    self.assertTrue(started.wait(2))
                    self.assertEqual(manager.cancel(job["id"])["status"], "cancelled")
                    manager.pending.join()
                    self.assertEqual(manager.get(job["id"])["status"], "cancelled")
                finally:
                    manager.close()


class WorkflowTests(unittest.TestCase):
    def test_concurrent_workflow_refuses_without_mutating_journal(self):
        import workflow
        with tempfile.TemporaryDirectory(prefix="mclib-workflow-test-") as directory:
            journal = Path(directory) / "workflow.json"
            journal.write_text('{"version":1,"attempts":[]}')
            original = journal.read_bytes()
            with (Path(directory) / "workflow.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with mock.patch.object(sys, "argv", ["workflow", "baseline", "--output", directory]):
                    self.assertEqual(workflow.main(), 2)
            self.assertEqual(journal.read_bytes(), original)

    def test_focused_requires_current_passing_baseline(self):
        import workflow
        with tempfile.TemporaryDirectory(prefix="mclib-workflow-test-") as directory:
            with mock.patch.object(sys, "argv", ["workflow", "focused", "--output", directory]), mock.patch("workflow.source_fingerprint", return_value="revision"), mock.patch("workflow.command_check") as check:
                self.assertEqual(workflow.main(), 2)
                check.assert_not_called()
            state = strict_json((Path(directory) / "workflow.json").read_text())
            self.assertEqual(state["attempts"][-1]["status"], "blocked")

    def test_physics_failure_skips_controller_and_tracking_checks(self):
        import workflow
        def fake_check(name, command, destination, journal, persist, **kwargs):
            journal["checks"].append(dict(name=name, status="failed" if name == "physics-electrical" else "passed", returncode=1 if name == "physics-electrical" else 0, log="fixture.log"))
        with tempfile.TemporaryDirectory(prefix="mclib-workflow-test-") as directory:
            with mock.patch.object(sys, "argv", ["workflow", "baseline", "--output", directory]), mock.patch("workflow.source_fingerprint", return_value="revision"), mock.patch("workflow.command_check", side_effect=fake_check):
                self.assertEqual(workflow.main(), 1)
            state = strict_json((Path(directory) / "workflow.json").read_text())
            checks = {item["name"]: item["status"] for item in state["attempts"][0]["checks"]}
            self.assertEqual(checks["physics-electrical"], "failed")
            self.assertEqual(checks["physics-mechanical"], "passed")
            self.assertEqual(checks["host"], "skipped")
            self.assertEqual(checks["tracking-bridge"], "skipped")

    def test_verify_requires_boomerang_direction_regression(self):
        import workflow
        def fake_check(name, command, destination, journal, persist, **kwargs):
            journal["checks"].append(dict(name=name, command=list(map(str, command)),
                status="failed" if name == "boomerang-direction" else "passed",
                returncode=1 if name == "boomerang-direction" else 0, log="fixture.log"))
        with tempfile.TemporaryDirectory(prefix="mclib-workflow-test-") as directory:
            journal = Path(directory) / "workflow.json"
            journal.write_text(json.dumps(dict(version=1, attempts=[dict(
                id="baseline-fixture", stage="baseline", status="passed",
                source_fingerprint="revision", checks=[], results={})])))
            with mock.patch.object(sys, "argv", ["workflow", "verify", "--output", directory]), mock.patch("workflow.source_fingerprint", return_value="revision"), mock.patch("workflow.command_check", side_effect=fake_check):
                self.assertEqual(workflow.main(), 1)
            attempt = strict_json(journal.read_text())["attempts"][-1]
            check = next(item for item in attempt["checks"] if item["name"] == "boomerang-direction")
            self.assertTrue(any(path.endswith("boomerang_direction_audit.py") for path in check["command"]))
            self.assertIn("boomerang-direction", attempt["results"])
            chain = next(item for item in attempt["checks"] if item["name"] == "boomerang-chain")
            self.assertTrue(any(path.endswith("boomerang_chain_audit.py") for path in chain["command"]))
            self.assertIn("boomerang-chain", attempt["results"])
            self.assertEqual(attempt["status"], "failed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
