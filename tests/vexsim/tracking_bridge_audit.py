#!/usr/bin/env python3
"""Check the Python/C++ sensor ABI, frames and real-physics sequence bridge.

Injected sensor tests deliberately leave body truth unchanged: they verify
measurement wiring independently, not physical fidelity. Live tests then run
the actual passive-wheel physics. No controller receives ground-truth pose.
"""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
from run import PhysicsBridge, Sample, build

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--vexsim", type=Path,
                    default=Path(__file__).resolve().parents[2].parent / "vexsim")
args, remaining = parser.parse_known_args()
sys.argv = [sys.argv[0], *remaining]
sys.path.insert(0, str(args.vexsim.resolve()))


class TrackingBridgeAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.artifacts = Path(tempfile.mkdtemp(prefix="mclib-tracking-bridge-"))
        cls.lib = build(cls.artifacts)
        print(f"Bridge build artifacts: {cls.artifacts}", flush=True)

    def bridge(self, **kwargs):
        return PhysicsBridge(self.lib, "six_motor_450", **kwargs)

    def assert_pose(self, actual, expected, tolerance=1e-9):
        self.assertAlmostEqual(actual[0], expected[0], delta=tolerance)
        self.assertAlmostEqual(actual[1], expected[1], delta=tolerance)
        self.assertAlmostEqual(math.remainder(actual[2] - expected[2], 360),
                               0, delta=tolerance)

    def test_sample_layout_matches_cpp_linux_abi(self):
        self.assertEqual(C.sizeof(Sample), 72)
        self.assertEqual(Sample.millis.offset, 40)
        self.assertEqual(Sample.vertical.offset, 56)
        self.assertEqual(Sample.horizontal.offset, 64)

    def test_nonzero_start_pose_and_heading_wrap(self):
        for mode in ("drive", "two"):
            for heading in (-450, -90, 0, 90, 450):
                with self.subTest(mode=mode, heading=heading):
                    bridge = self.bridge(tracking_mode=mode, start_pose=(12, -8, heading))
                    self.assert_pose(bridge.pose(), (12, -8, heading))
                    self.assert_pose(bridge.truth(), (12, -8, heading))
                    bridge.lib.sim_refresh(0)
                    self.assert_pose(bridge.pose(), (12, -8, heading))

    def test_sensor_degrees_circumference_and_sideways_sign(self):
        bridge = self.bridge(tracking_mode="two")
        radius_in = 2.75 / 2
        bridge.sim.tracking["track_parallel"].position = 10 / radius_in
        bridge.sim.tracking["track_perp"].position = -4 / radius_in
        bridge.lib.sim_refresh(0)
        self.assert_pose(bridge.pose(), (4, 10, 0))
        self.assert_pose(bridge.truth(), (0, 0, 0))

    def test_tracker_offset_removes_in_place_spin_translation(self):
        for heading in (-90, 90):
            with self.subTest(heading=heading):
                bridge = self.bridge(tracking_mode="two")
                angle = math.radians(heading)
                bridge.sim.imu.heading = -angle
                # Perpendicular wheel sits 3 inches behind the body origin.
                bridge.sim.tracking["track_perp"].position = 3 * angle / (2.75 / 2)
                bridge.lib.sim_refresh(0)
                self.assert_pose(bridge.pose(), (0, 0, heading))

    def test_drive_measurements_do_not_replace_enabled_trackers(self):
        bridge = self.bridge(tracking_mode="two")
        for sensor in bridge.sim.encoders.values():
            sensor.position = 100
        bridge.lib.sim_refresh(0)
        self.assert_pose(bridge.pose(), (0, 0, 0))

    def test_encoder_motor_to_wheel_ratio(self):
        bridge = self.bridge(tracking_mode="drive")
        wheel = bridge.sim.chassis.left_units()[0]
        for sensor in bridge.sim.encoders.values():
            sensor.position = 2 * math.pi
        bridge.lib.sim_refresh(0)
        expected = 2 * math.pi * wheel.radius / bridge.inch / wheel.gear_ratio
        self.assert_pose(bridge.pose(), (0, expected, 0))

    def test_initial_battery_and_seed_survive_pose_reset(self):
        bridge = self.bridge(soc=0, seed=52, start_pose=(2, 3, 45))
        self.assertEqual(bridge.sim.battery.voltage, 0)
        self.assertEqual(bridge.sim.imu.seed, 52)
        from random import Random
        self.assertEqual(bridge.sim.imu._rng.getstate(), Random(52).getstate())

    def test_idle_updates_sensor_based_odometry_and_enforces_guard(self):
        bridge = self.bridge(tracking_mode="two", sim_time_limit=.025)
        bridge.write(0, 0, 6)
        bridge.write(1, 0, 6)
        with self.assertRaisesRegex(RuntimeError, "simulated-time guard"):
            bridge.advance_idle(40)

    def test_actual_tracker_motion_from_nonzero_pose_and_sequential_tare(self):
        bridge = self.bridge(tracking_mode="two", start_pose=(12, -8, 90))
        bridge.run(1, a=12)
        bridge.advance_idle(300)
        first = bridge.capture()
        self.assertTrue(bridge.stopped())
        self.assertLess(first["odometry_error_in"], .5)
        self.assertGreater(first["true_x"], 20)
        self.assertAlmostEqual(first["true_y"], -8, delta=.3)
        bridge.run(1, a=-12)
        bridge.advance_idle(300)
        second = bridge.capture()
        self.assertTrue(bridge.stopped())
        self.assertGreater(second["t"], first["t"])
        self.assertLess(second["odometry_error_in"], .5)
        self.assertLess(second["true_x"], first["true_x"] - 8)
        self.assertTrue(all(a["t"] < b["t"] for a, b in
                            zip(bridge.control_trace, bridge.control_trace[1:])))

    def test_invalid_tuning_rejected(self):
        bridge = self.bridge()
        for tuning in ({"unknown": 1}, {"drive_kp": float("nan")},
                       {"turn_kd": -1}, {"heading_ki": True}, {"turn_kp": "1"}):
            with self.subTest(tuning=tuning), self.assertRaises(ValueError):
                bridge.set_tuning(tuning)

    def test_heading_target_after_timeout_comes_from_cpp_state(self):
        bridge = self.bridge()
        bridge.run(0, a=90, timeout=50)
        target = bridge.commanded_heading()
        self.assertLess(abs(target), 45)
        self.assertAlmostEqual(target, bridge.pose()[2], delta=1e-8)
        bridge.advance_idle(300)
        self.assertAlmostEqual(bridge.commanded_heading(), target, delta=1e-12)

    def test_debug_sensor_units_signs_and_motor_aggregates(self):
        bridge = self.bridge(tracking_mode="two")
        bridge.sim.state.u = 3 * bridge.inch
        bridge.sim.state.v = -4 * bridge.inch
        bridge.sim.state.r = -math.pi / 2
        bridge.sim.imu.heading = -math.pi / 3
        bridge.sim.imu.rate = -math.pi / 4
        for index, motors in enumerate(bridge.sides):
            for motor in motors:
                bridge.sim.encoders[motor.name].position = (index + 1) * math.pi
                bridge.sim.encoders[motor.name].velocity = (index + 1) * math.pi
                motor.current = -1.25
                motor.temperature = 40 + index
        for name, position in (("track_parallel", 7), ("track_perp", -9)):
            unit = next(u for u in bridge.sim.chassis.wheel_units if u.name == name)
            bridge.sim.tracking[name].position = position * bridge.inch / unit.radius
        for unit in bridge.sim.chassis.left_units():
            unit.angle = 2.25
        for unit in bridge.sim.chassis.right_units():
            unit.angle = -1.75
        frame = bridge.capture()
        for field, value in dict(forward_speed_ips=3, lateral_speed_ips=4, speed_ips=5,
                                 yaw_rate_dps=90, imu_heading_deg=60, imu_rate_dps=45,
                                 left_encoder_deg=180, right_encoder_deg=360,
                                 left_encoder_rpm=30, right_encoder_rpm=60,
                                 left_current_amps=3.75, right_current_amps=3.75,
                                 left_temp_c=40, right_temp_c=41,
                                 parallel_tracker_in=7, perpendicular_tracker_in=9,
                                 body_length_in=15, body_width_in=12.5,
                                 left_wheel_angle_rad=2.25, right_wheel_angle_rad=-1.75).items():
            self.assertAlmostEqual(frame[field], value, msg=field)
        json.dumps(frame, allow_nan=False)
        drive = self.bridge(tracking_mode="drive").capture()
        self.assertIsNone(drive["parallel_tracker_in"])
        self.assertIsNone(drive["perpendicular_tracker_in"])

    def test_capture_observer_does_not_change_motion_and_receives_actual_controller(self):
        baseline = self.bridge(tracking_mode="two")
        baseline.run(7, a=15.5, b=18.5, heading=-90)
        expected = baseline.control_trace
        observed = []
        bridge = self.bridge(tracking_mode="two")
        bridge.on_capture = lambda frame: observed.append(dict(frame))
        bridge.run(7, a=15.5, b=18.5, heading=-90)
        self.assertEqual(bridge.control_trace, expected)
        self.assertEqual(observed, expected)
        pursuit = [f for f in observed if f["controller_phase"] == "pursuit"]
        self.assertTrue(pursuit)
        self.assertGreater(pursuit[0]["controller_target_heading_deg"], 40)
        self.assertAlmostEqual(pursuit[0]["controller_carrot_y"], 18.5)
        self.assertEqual(bridge.capture()["controller_phase"], "idle")
        self.assertIsNone(bridge.capture()["controller_target_heading_deg"])
        bridge.run(0, a=0)
        self.assertTrue(any(f["controller_phase"] == "turn" for f in observed[len(expected):]))
        json.dumps(observed, allow_nan=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
