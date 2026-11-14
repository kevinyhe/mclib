"""The planner's HTTP server: what it accepts, what it refuses, how it
queues, cancels and restores runs, and how it writes an export into a robot
project. No C++ or physics runs here."""
import http.client
import json
import os
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
        self.field_drawings = dict(override=dict(half_in=70.2), open=None)
        self.submitted = []

    def engine_state(self, preset, tracking):
        return dict(preset=preset, tracking=tracking, ready=False, task=None)

    def prepare(self, preset, tracking):
        return dict(preset=preset, tracking=tracking, ready=False, task=dict(status="running"))

    def plan(self, raw):
        validate_project(raw)
        return dict(not_ready=True, error="not compiled")

    def write_export(self, body):
        folder = server.robot_project(body["folder"])
        return dict(folder=str(folder), function="runRoutine", files=[], mentioned_in=None)

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

    def submit_batch(self, body):
        if set(body) != {"project", "settings"}:
            raise ValueError("Send the routine and the run-many settings.")
        validate_project(body["project"])
        return dict(id="c" * 32, number=1, status="queued", settings=server.repeat_settings(body["settings"]))

    def get_batch(self, identity):
        return dict(id=identity, status="queued") if identity == "c" * 32 else None

    def list_batches(self):
        return []

    def cancel_batch(self, identity):
        return self.get_batch(identity)

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
                          "override_stack_to_goal", "override_roller", "override_match_load", "override_five_halves",
                          "points_arcs_and_swing", "keep_moving", "path_settings_and_gains", "worn_field",
                          "no_inertial_sensor", "override_wall_reset", "override_center_goal",
                          "override_lift_heights"})
        status, project = self.json("GET", "/api/examples/s_curve")
        self.assertEqual(status, 200)
        self.assertNotIn("description", project)
        self.assertEqual(self.json("GET", "/api/examples/..%2fserver")[0], 404)
        status, project = self.json("POST", "/api/validate", ONE_POINT)
        self.assertEqual((status, project["steps"][0]["volts"]), (200, 12))

    def test_run_many_checks_its_settings_and_routine(self):
        status, answer = self.json("POST", "/api/batches", dict(project=ONE_POINT, settings=dict(count=1)))
        self.assertEqual((status, answer["error"]), (400, "Run many: Runs must be between 2 and 50, not 1"))
        status, answer = self.json("POST", "/api/batches", dict(project=ONE_POINT, settings=dict(speed=3)))
        self.assertEqual((status, answer["error"]), (400, "Run many has fields the planner does not know: speed"))
        status, answer = self.json("POST", "/api/batches", dict(project=ONE_POINT, settings=dict(count=2.5)))
        self.assertEqual((status, answer["error"]), (400, "Run many: Runs must be a whole number"))
        status, answer = self.json("POST", "/api/batches", dict(project=dict(steps=[dict(type="turn", heading="left")]),
                                                               settings={}))
        self.assertEqual((status, answer["step"]), (400, 0))
        status, answer = self.json("POST", "/api/batches", dict(project=ONE_POINT, settings={}))
        self.assertEqual((status, answer["settings"]["count"]), (202, 20))
        self.assertEqual(self.json("GET", "/api/batches/" + "c" * 32)[0], 200)
        self.assertEqual(self.json("GET", "/api/batches/" + "d" * 32)[0], 404)
        self.assertEqual(self.json("POST", "/api/batches/" + "c" * 32 + "/cancel", {})[0], 200)
        self.assertEqual(self.json("GET", "/api/batches"), (200, dict(batches=[])))

    def test_bad_routines_come_back_with_the_step(self):
        status, answer = self.json("POST", "/api/runs", dict(steps=[dict(type="turn", heading="left")]))
        self.assertEqual(status, 400)
        self.assertEqual(answer, dict(error="Step 1 (Turn to heading): Heading must be a number", step=0))
        status, answer = self.json("POST", "/api/runs", dict(steps=[dict(type="path", points=[], sections=[])]))
        self.assertEqual((status, answer["step"]), (400, 0))
        self.assertIn("has no points yet", answer["error"])
        self.assertEqual(self.json("POST", "/api/runs", ONE_POINT)[0], 202)
        self.assertEqual(self.json("POST", "/api/plan", ONE_POINT)[0], 409)

    def test_export_write_says_what_is_wrong_with_the_folder(self):
        body = dict(project=ONE_POINT, folder="robot")
        status, answer = self.json("POST", "/api/export/write", body)
        self.assertEqual(status, 400)
        self.assertIn("full path", answer["error"])
        self.assertEqual(self.request("POST", "/api/export/write", body, {"Origin": "http://example.com"})[0], 403)
        with tempfile.TemporaryDirectory() as folder:
            status, answer = self.json("POST", "/api/export/write", dict(project=ONE_POINT, folder=folder))
            self.assertEqual(status, 400)
            self.assertIn("no project.pros", answer["error"])
            Path(folder, "project.pros").write_text("{}")
            status, answer = self.json("POST", "/api/export/write", dict(project=ONE_POINT, folder=folder))
            self.assertEqual((status, answer["folder"]), (200, folder))

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
            self.assertIn(f"git -C {folder} pull", payload["error"])
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
                while planner.processes and time.monotonic() - started < 5:
                    time.sleep(.05)
                self.assertEqual(planner.processes, {})
                self.assertLess(time.monotonic() - started, 3)
                self.assertEqual(planner.get(job["id"])["status"], "cancelled")
            finally:
                planner.close()

    def test_queue_is_bounded(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output, parallel_runs=2)
            try:
                with self.assertRaises(server.BusyError):
                    for _ in range(2 + LIMITS["queue_size"] + 1):
                        planner.submit(ONE_POINT)
            finally:
                planner.close()

    def test_up_to_parallel_runs_go_at_once_and_the_rest_wait(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output, parallel_runs=2)
            try:
                jobs = [planner.submit(ONE_POINT) for _ in range(3)]
                self.wait_for(planner, jobs[0]["id"], {"running"})
                self.wait_for(planner, jobs[1]["id"], {"running"})
                time.sleep(.5)
                self.assertEqual(planner.get(jobs[2]["id"])["status"], "queued")
                planner.cancel(jobs[0]["id"])
                self.wait_for(planner, jobs[2]["id"], {"running"})
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


FAKE_RUN = r"""
import json, sys
arguments = sys.argv[1:]
project = json.load(open(arguments[arguments.index("--project") + 1]))
output = arguments[arguments.index("--output") + 1]
conditions = project["conditions"]
row = dict(index=0, type="drive", executed=True, configuration_error=None, rule_breaks=[], coordinate_reset=False,
           move_result=None, duration_s=1.0, timeout_ms=4000, position_error_in=abs(conditions["placement_x_in"]) * 4,
           heading_error_deg=0.5)
trace = [dict(t=tenth / 10, true_x=conditions["placement_x_in"], true_y=tenth, true_heading=0) for tenth in range(13)]
placement = {key: conditions[key] for key in ("placement_x_in", "placement_y_in", "placement_heading_deg")}
json.dump(dict(legal=True, steps=[row], trace=trace, project=project, autonomous=None, score=None,
               placement=dict(placement, moved_back_from=None),
               summary=dict(steps=1, ran_steps=1, skipped_steps=0, broke_rule_in_step=None, simulated_s=1.2)),
          open(output + "/result.json", "w"))
"""
ONE_DRIVE = dict(steps=[dict(type="drive", distance=24)])


def fake_engine(*arguments, vexsim, cache):
    return [sys.executable, "-c", FAKE_RUN, *map(str, arguments)]


class RunManyTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.output = Path(self.folder.name)

    def tearDown(self):
        self.folder.cleanup()

    def wait_for(self, read, statuses):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            value = read()
            if value["status"] in statuses:
                return value
            time.sleep(.05)
        self.fail(f"never reached {statuses}")

    def test_each_run_varies_the_start_and_the_report_puts_the_worst_first(self):
        with mock.patch.object(server, "engine_command", fake_engine):
            planner = server.Planner(self.output, self.output, parallel_runs=3)
            try:
                batch = planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=6, placement_in=1)))
                batch = self.wait_for(lambda: planner.get_batch(batch["id"]), {"finished"})
                runs = batch["runs"]
                self.assertEqual([run["status"] for run in runs], ["finished"] * 6)
                placements = [run["conditions"]["placement_x_in"] for run in runs]
                self.assertTrue(all(abs(x) <= 1 for x in placements), placements)
                self.assertEqual(len(set(run["conditions"]["noise_seed"] for run in runs)), 6)
                step = batch["report"]["steps"][0]
                self.assertEqual((step["ran"], step["finished"]), (6, 6))
                self.assertEqual(step["on_target"], sum(abs(x) * 4 <= 2 for x in placements))
                worst = max(runs, key=lambda run: abs(run["conditions"]["placement_x_in"]))
                self.assertEqual(batch["report"]["worst_first"][0], worst["number"])
                child = planner.get(worst["id"])
                self.assertEqual(child["project"]["conditions"], worst["conditions"])
                self.assertEqual(child["batch"]["number"], batch["number"])
                self.assertEqual(planner.list(), [])
                self.assertEqual([item["id"] for item in planner.list_batches()], [batch["id"]])
            finally:
                planner.close()

    def test_one_run_many_at_a_time_and_cancel_stops_its_runs(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output, parallel_runs=2)
            try:
                batch = planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=4)))
                self.wait_for(lambda: planner.get_batch(batch["id"]), {"running"})
                with self.assertRaisesRegex(server.BusyError, "A run-many is still going"):
                    planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=2)))
                planner.cancel_batch(batch["id"])
                cancelled = self.wait_for(lambda: planner.get_batch(batch["id"]), {"cancelled"})
                self.assertEqual({run["status"] for run in cancelled["runs"]}, {"cancelled"})
                deadline = time.monotonic() + 5
                while planner.processes and time.monotonic() < deadline:
                    time.sleep(.05)
                self.assertEqual(planner.processes, {})
                self.assertEqual(planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=2)))["number"], 2)
            finally:
                planner.close()

    def test_a_single_run_goes_before_the_waiting_runs_of_a_run_many(self):
        with mock.patch.object(server, "engine_command", sleeping_engine):
            planner = server.Planner(self.output, self.output, parallel_runs=1)
            try:
                batch = planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=3)))
                first = self.wait_for(lambda: planner.get_batch(batch["id"]), {"running"})["runs"][0]
                single = planner.submit(ONE_POINT)
                planner.cancel(first["id"])
                self.wait_for(lambda: planner.get(single["id"]), {"running"})
                self.assertEqual(planner.get_batch(batch["id"])["runs"][1]["status"], "queued")
            finally:
                planner.close()

    def test_a_run_many_left_unfinished_is_restored_with_its_runs_as_errors(self):
        identity, done, waiting = "e" * 32, "f" * 32, "0" * 32
        folder = self.output / identity
        folder.mkdir()
        (folder / "project.json").write_text(json.dumps(validate_project(ONE_DRIVE)))
        runs = [dict(id=done, number=1, status="cancelled", outcome=None, conditions={}),
                dict(id=waiting, number=2, status="running", outcome=None, conditions={})]
        (folder / "batch.json").write_text(json.dumps(dict(id=identity, number=4, name="r", created_at=0, settings={},
                                                           seed=1, cancelled=False, runs=runs)))
        planner = server.Planner(self.output, self.output)
        try:
            batch = planner.get_batch(identity)
            self.assertEqual(batch["status"], "finished")
            self.assertEqual(batch["runs"][1]["error"], server.STOPPED_EARLY)
            self.assertEqual(planner.get(waiting)["status"], "error")
            with mock.patch.object(server, "engine_command", sleeping_engine):
                self.assertEqual(planner.submit_batch(dict(project=ONE_DRIVE, settings=dict(count=2)))["number"], 5)
        finally:
            planner.close()


LIFT_ROUTINE = dict(name="Lift check", field="override", preload="none",
                    robot=dict(preset="override_cascade", tracking="horizontal"),
                    steps=[dict(type="drive", distance=12), dict(type="lift", lift_preset="top")])
ROUTINE, ROBOT_CASCADE, CASCADE_HELPERS = ("src/autons/lift_check.cpp", "include/planner/robot_cascade.hpp",
                                           "include/planner/cascade_helpers.hpp")


class WriteExportTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.robot = Path(self.folder.name) / "robot"
        self.robot.mkdir()
        (self.robot / "project.pros").write_text("{}")
        self.output = Path(self.folder.name) / "output"
        self.output.mkdir()
        self.planner = server.Planner(self.output, self.output)

    def tearDown(self):
        self.planner.close()
        self.folder.cleanup()

    def write(self, project=LIFT_ROUTINE, folder=None):
        return self.planner.write_export(dict(project=project, folder=str(folder or self.robot)))

    def statuses(self, answer):
        return {file["path"]: file["status"] for file in answer["files"]}

    def test_first_write_puts_every_file_in_the_robot_project(self):
        answer = self.write()
        self.assertEqual((answer["folder"], answer["function"], answer["mentioned_in"]),
                         (str(self.robot), "runLiftCheck", None))
        self.assertEqual(self.statuses(answer),
                         {ROUTINE: "written", ROBOT_CASCADE: "written", CASCADE_HELPERS: "written"})
        self.assertIn("void runLiftCheck() {", (self.robot / ROUTINE).read_text())
        self.assertEqual(sorted(path.relative_to(self.robot).as_posix() for path in self.robot.rglob("*.?pp")),
                         sorted([ROUTINE, ROBOT_CASCADE, CASCADE_HELPERS]))

    def test_writing_again_replaces_the_routine_and_keeps_the_robots_own_file(self):
        self.write()
        (self.robot / ROBOT_CASCADE).write_text("// my ports\n")
        self.assertEqual(self.statuses(self.write()),
                         {ROUTINE: "unchanged", ROBOT_CASCADE: "kept", CASCADE_HELPERS: "unchanged"})
        changed = dict(LIFT_ROUTINE, steps=[dict(type="drive", distance=20), dict(type="lift", lift_preset="top")])
        self.assertEqual(self.statuses(self.write(changed)),
                         {ROUTINE: "replaced", ROBOT_CASCADE: "kept", CASCADE_HELPERS: "unchanged"})
        self.assertIn("driveTo(20_in", (self.robot / ROUTINE).read_text())
        self.assertEqual((self.robot / ROBOT_CASCADE).read_text(), "// my ports\n")

    def test_says_where_the_routine_is_already_hooked_up(self):
        (self.robot / "src").mkdir()
        (self.robot / "src" / "main.cpp").write_text("void runLiftCheckTwice();\n")
        self.assertIsNone(self.write()["mentioned_in"])
        (self.robot / "src" / "main.cpp").write_text("void runLiftCheck();\n")
        self.assertEqual(self.write()["mentioned_in"], "src/main.cpp")

    def test_writes_nothing_outside_a_pros_project(self):
        elsewhere = Path(self.folder.name) / "elsewhere"
        elsewhere.mkdir()
        cases = (("", "Type the folder"), ("robot", "full path"), (str(elsewhere / "missing"), "is not a folder"),
                 (str(elsewhere), "no project.pros"), (7, "Type the folder"))
        for folder, message in cases:
            with self.subTest(folder=folder), self.assertRaisesRegex(ValueError, f"^Nothing was written. .*{message}"):
                self.planner.write_export(dict(project=LIFT_ROUTINE, folder=folder))
        self.assertEqual(list(elsewhere.iterdir()), [])
        with self.assertRaisesRegex(ValueError, "Send the project and the robot project folder"):
            self.planner.write_export(dict(project=LIFT_ROUTINE))

    def test_a_bad_routine_writes_nothing(self):
        with self.assertRaises(ProjectError):
            self.write(dict(LIFT_ROUTINE, steps=[dict(type="lift", lift_preset="sky")]))
        self.assertEqual([path.name for path in self.robot.iterdir()], ["project.pros"])

    def test_a_file_that_cannot_be_written_is_named_with_what_was_written_before_it(self):
        (self.robot / "src").write_text("not a folder")
        with self.assertRaisesRegex(ValueError, r"Could not write .*lift_check\.cpp: .* Nothing was written\.$"):
            self.write()
        (self.robot / "src").unlink()
        (self.robot / "include").write_text("not a folder")
        with self.assertRaisesRegex(ValueError, r"Could not write .*robot_cascade\.hpp: .* Written before that: "
                                                r"src/autons/lift_check\.cpp\.$"):
            self.write()

    def test_a_routine_whose_paths_are_not_planned_yet_is_not_written(self):
        path_routine = dict(name="Path check", steps=ONE_POINT["steps"])
        still_compiling = dict(not_ready=True, error="Speeds are not planned yet.")
        with mock.patch.object(self.planner, "plan", return_value=still_compiling):
            with self.assertRaisesRegex(ValueError, "^Nothing was written. Until the paths are planned, their "
                                                    "speeds, time limits and the drive numbers would be "
                                                    "placeholders. The planner said: Speeds are not planned yet.$"):
                self.write(path_routine)
            exported = self.planner.export(path_routine)
        self.assertEqual([path.name for path in self.robot.iterdir()], ["project.pros"])
        self.assertEqual((exported["drive_numbers"], exported["plan_error"]),
                         ("placeholders", "Speeds are not planned yet."))

    def test_an_unknown_home_folder_is_refused(self):
        with self.assertRaisesRegex(ValueError, "names a home folder this computer does not have"):
            self.write(folder="~no-such-user-here/robot")

    def test_a_routine_with_only_drive_steps_writes_one_file_into_a_bare_project(self):
        answer = self.write(dict(name="Drive check", steps=[dict(type="drive", distance=12)]))
        self.assertEqual((self.statuses(answer), answer["mentioned_in"]),
                         ({"src/autons/drive_check.cpp": "written"}, None))
        self.assertFalse((self.robot / "include").exists())

    def test_a_project_file_that_is_not_utf8_is_still_searched(self):
        (self.robot / "src").mkdir()
        (self.robot / "src" / "main.cpp").write_bytes(b"// caf\xe9\nvoid runLiftCheck();\n")
        self.assertEqual(self.write()["mentioned_in"], "src/main.cpp")

    @unittest.skipUnless(os.name == "posix", "Windows paths are only translated under WSL")
    def test_a_windows_path_is_read_as_its_wsl_folder(self):
        self.assertEqual(server.local_path(r"C:\Users\you\robot"), Path("/mnt/c/Users/you/robot"))
        self.assertEqual(server.local_path("d:/robot"), Path("/mnt/d/robot"))
        self.assertEqual(server.local_path("/home/you/robot"), Path("/home/you/robot"))


if __name__ == "__main__":
    unittest.main()
