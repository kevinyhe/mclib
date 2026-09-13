"""Acceptance checks for a completed original-Metro native recording.

Usage: python3 -B tests/vexsim/test_metro_recording.py RECORDING.json
These check simulation/recording integrity, not successful seven-block scoring.
"""
import argparse
import hashlib
import math
from pathlib import Path
import sys
import unittest

from game_viewer import load_recording

parser = argparse.ArgumentParser()
parser.add_argument("recording", type=Path)
args, remaining = parser.parse_known_args()
recording = load_recording(args.recording)


class RecordedRunTests(unittest.TestCase):
    def test_completed_original_routine(self):
        self.assertTrue(recording["summary"]["completed"])
        self.assertIsNone(recording["summary"]["error"])
        self.assertTrue(recording["summary"]["routine_returned"])
        self.assertEqual(recording["metadata"]["routine"], "leftSideSevenMiddle")
        self.assertEqual(recording["metadata"]["source_commit"],
                         "0e13c4c6fc6aa7097f97074d6969c05d1d43dc05")

    def test_exact_capture_clock(self):
        frames = recording["frames"]
        self.assertEqual(len(frames), 1501)
        for index, frame in enumerate(frames):
            self.assertAlmostEqual(frame["t"], index / 100, places=8)
        self.assertEqual(recording["summary"]["simulated_ms"], 15000)

    def test_piece_identity_order_and_conservation(self):
        first = recording["frames"][0]["blocks"]
        self.assertEqual(len(first), 60 + recording["metadata"]["preloads"])
        colours = [block["c"] for block in first]
        for frame in recording["frames"]:
            self.assertEqual([b["c"] for b in frame["blocks"]], colours)
            self.assertEqual(sum(b["s"] == "held" for b in frame["blocks"]), frame["game"]["held"])
            for block in frame["blocks"]:
                self.assertAlmostEqual(sum(v * v for v in block["q"]), 1, places=6)

    def test_no_physical_teleport_during_coordinate_reset(self):
        resets = []
        frames = recording["frames"]
        for before, after in zip(frames, frames[1:]):
            distance = math.hypot(after["robot"]["x"] - before["robot"]["x"],
                                  after["robot"]["y"] - before["robot"]["y"])
            self.assertLess(distance, .1, "More than 10 cm physical movement in 10 ms")
            old, new = (f["metro"]["telemetry"]["local_pose_in_deg"] for f in (before, after))
            if abs(new[0]) + abs(new[1]) < 1e-10 and math.hypot(*old[:2]) > 1:
                resets.append(after["t"])
        self.assertEqual(len(resets), 1, "Expected delayed XY reset to be visible without moving truth")
        if "odometry_origin_in" in frames[0]["metro"]["telemetry"]:
            events = [e for e in recording["events"] if e["type"] == "odometry_origin_reset"]
            self.assertEqual(len(events), 1)
            for before, after in zip(frames, frames[1:]):
                distance = math.hypot(after["odom"]["x"] - before["odom"]["x"],
                                      after["odom"]["y"] - before["odom"]["y"])
                self.assertLess(distance, .1, "Display ghost must preserve its field origin across the local reset")

    def test_delayed_scraper_is_concurrent(self):
        events = recording["events"]
        scraper = [e for e in events if e["type"] == "pneumatic_output"
                   and e["name"] == "scraper" and e["value"]]
        self.assertEqual([e["t"] for e in scraper], [.5])
        delay = [e for e in events if e["type"] == "task_sleep" and e["t"] == .5 and e["a"] == 300]
        self.assertEqual(len(delay), 1)
        self.assertTrue(any(e["type"] == "task_ended" and e["task_id"] == delay[0]["task_id"]
                            and e["t"] == .8 for e in events))
        self.assertTrue(any(e["type"] == "drive_output" and .5 < e["t"] < .8 for e in events))

    def test_only_startup_encoder_tares(self):
        tares = [e for e in recording["events"] if e["type"] == "encoder_tare"]
        self.assertEqual(len(tares), 2)
        self.assertEqual([e["t"] for e in tares], [0, 0])

    def test_scraper_frames_follow_cpp_commands_until_explicit_retraction(self):
        events = [e for e in recording["events"] if e["type"] == "pneumatic_output"
                  and e.get("name") == "scraper"]
        self.assertEqual([e["value"] for e in events], [True, False])
        self.assertEqual(events[0]["t"], .5)
        self.assertGreater(events[1]["t"], .8,
                           "The delayed task's 300 ms sleep must not retract the scraper")
        for frame in recording["frames"]:
            deployed = False
            for event in events:
                if event["t"] <= frame["t"] + 1e-9:
                    deployed = event["value"]
            self.assertEqual(frame["metro"]["outputs"]["scraper"], deployed)

    def test_disabled_at_autonomous_end(self):
        last = recording["frames"][-1]
        self.assertEqual(last["cmd"]["left_volts"], 0)
        self.assertEqual(last["cmd"]["right_volts"], 0)
        self.assertEqual(last["metro"]["outputs"]["bottom"], 0)
        self.assertEqual(last["metro"]["outputs"]["top"], 0)
        self.assertEqual(last["metro"]["telemetry"]["intake_state"], "DISABLED")

    def test_visible_fidelity_limits(self):
        self.assertIn("not calibrated", recording["metadata"]["acceptance"])
        for frame in recording["frames"]:
            self.assertIn("custom_intake_geometry", frame["metro"]["fidelity"]["unsupported"])
            self.assertIn("scraper_collision", frame["metro"]["fidelity"]["unsupported"])

    def test_runtime_source_fingerprint(self):
        hashes = recording["metadata"]["runtime_source_sha256"]
        self.assertGreater(len(hashes), 10)
        for path, expected in hashes.items():
            self.assertEqual(hashlib.sha256(Path(path).read_bytes()).hexdigest(), expected, path)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
