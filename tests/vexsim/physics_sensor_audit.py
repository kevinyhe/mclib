#!/usr/bin/env python3
"""Independent sensor/orchestration checks; intentionally fail on audit findings.

Run with PYTHONDONTWRITEBYTECODE=1. No simulator files are changed.
Hardware range references are recorded in PHYSICS_VALIDATION.md.
Range checks use a conservative in-range-or-invalid acceptance policy, not a
claim that the manufacturer specifies a unique overloaded sensor response.
"""
import argparse
import math
from pathlib import Path
import sys
import unittest

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--vexsim", type=Path,
                    default=Path(__file__).resolve().parents[2].parent / "vexsim")
args, remaining = parser.parse_known_args()
sys.argv = [sys.argv[0], *remaining]
sys.path.insert(0, str(args.vexsim.resolve()))
from vexsim import presets
from vexsim.sensors import DistanceSensor, Encoder, GPSSensor, IMU, RotationSensor
from vexsim.sim import Simulator, SimConfig
from vexsim.units import DEG, G, INCH, RPM, to_deg, to_inches, to_rpm


def ideal_imu():
    return IMU(bias=0, bias_walk=0, noise=0, scale_error=0, accel_noise=0)


class SensorAudit(unittest.TestCase):
    def test_unit_roundtrips(self):
        for value in (-720.5, -1, 0, 1, 1000):
            self.assertAlmostEqual(to_inches(value * INCH), value)
            self.assertAlmostEqual(to_deg(value * DEG), value)
            self.assertAlmostEqual(to_rpm(value * RPM), value)

    def test_encoder_quantization_bound_all_cartridges(self):
        for ticks in (1800, 900, 300, 4096):
            sensor = Encoder(ticks_per_rev=ticks)
            for i in range(1001):
                position = -7.7 + 0.0154 * i
                sensor.update(position, i * 0.02)
                self.assertLessEqual(abs(sensor.position - position), math.pi / ticks + 1e-12)

    def test_encoder_sample_and_hold(self):
        sensor = Encoder(ticks_per_rev=900)
        sensor.update(0, 0)
        sensor.update(1, 0.005)
        self.assertEqual(sensor.position, 0)
        sensor.update(1, 0.011)
        self.assertAlmostEqual(sensor.position, 1, delta=math.pi / 900)

    def test_seeded_fresh_sensors_repeat(self):
        for factory in (Encoder, RotationSensor, IMU, GPSSensor, DistanceSensor):
            a, b = factory(), factory()
            for i in range(100):
                t = i * 0.02
                if isinstance(a, Encoder):
                    a.update(t, t)
                    b.update(t, t)
                    self.assertEqual(a.position, b.position)
                elif isinstance(a, IMU):
                    a.update(0.3, 0.1, 0.2, t)
                    b.update(0.3, 0.1, 0.2, t)
                    self.assertEqual(a.heading, b.heading)
                else:
                    a.update(0.1, 0.2, 0.3, t)
                    b.update(0.1, 0.2, 0.3, t)
                    self.assertEqual(a.__dict__.get("distance", a.__dict__.get("x")),
                                     b.__dict__.get("distance", b.__dict__.get("x")))

    def test_imu_initial_sample_does_not_invent_elapsed_time(self):
        sensor = ideal_imu()
        sensor.update(1, 0, 0, 0)
        self.assertAlmostEqual(sensor.heading, 0, delta=1e-12,
                               msg="At t=0 no rotation has elapsed; current code adds 0.01 rad")

    def test_imu_constant_rate_matches_elapsed_time(self):
        sensor = ideal_imu()
        sensor.update(1, 0, 0, 0)
        for i in range(1, 21):
            sensor.update(1, 0, 0, i * 0.011)
        self.assertAlmostEqual(sensor.heading, 0.22, delta=1e-12)

    def test_imu_variable_rate_integral_independent_of_publication_period(self):
        # Exact integral of a triangular rate profile, supplied every 1 ms.
        # Assert each publication, not just the endpoint where errors can cancel.
        peak, ramp = 360 * DEG, .1
        for period in (.001, .010, .025):
            with self.subTest(period=period):
                sensor = ideal_imu()
                sensor.update_period = period
                for tick in range(201):
                    t = tick / 1000
                    rate = peak * (t / ramp if t <= ramp else (2 * ramp - t) / ramp)
                    before = sensor._last_update
                    previous_heading = sensor.heading
                    sensor.update(rate, 0, 0, t)
                    if sensor._last_update == before:
                        self.assertEqual(sensor.heading, previous_heading)
                        continue
                    if t <= ramp:
                        expected = peak * t * t / (2 * ramp)
                    else:
                        tail = t - ramp
                        expected = peak * ramp / 2 + peak * tail - peak * tail * tail / (2 * ramp)
                    self.assertAlmostEqual(sensor.heading, expected, delta=1e-10,
                        msg="Port publication must not discard intervening gyro integration samples")

    def test_imu_retains_rate_pulse_between_publications(self):
        sensor = ideal_imu()
        for tick in range(11):
            t = tick / 1000
            rate = max(0, 1 - abs(t - .004) / .002)
            sensor.update(rate, 0, 0, t)
            if tick < 10:
                self.assertEqual(sensor.heading, 0)
        self.assertAlmostEqual(sensor.heading, .002, delta=1e-12)

    def test_imu_reset_discards_unpublished_integral(self):
        sensor = ideal_imu()
        sensor.update(0, 0, 0, 0)
        sensor.update(1, 0, 0, .005)
        sensor.reset(1.2)
        sensor.update(0, 0, 0, 0)
        sensor.update(0, 0, 0, .01)
        self.assertAlmostEqual(sensor.heading, 1.2, delta=1e-12)

    def test_sensor_publication_cadence_has_no_float_extra_tick(self):
        for factory in (Encoder, RotationSensor, IMU, GPSSensor, DistanceSensor):
            with self.subTest(sensor=factory.__name__):
                sensor = factory()
                published = []
                for tick in range(1001):
                    before = sensor._last_update
                    t = tick / 1000
                    if isinstance(sensor, Encoder):
                        sensor.update(0, t)
                    else:
                        sensor.update(0, 0, 0, t)
                    if sensor._last_update != before:
                        published.append(tick)
                period_ms = round(sensor.update_period * 1000)
                self.assertEqual(published, list(range(0, 1001, period_ms)))

    def test_imu_accelerometer_respects_documented_range(self):
        sensor = ideal_imu()
        sensor.update(0, 8 * G, -8 * G, 0)
        self.assertTrue(not getattr(sensor, "valid", True) or
                        max(abs(sensor.accel_x), abs(sensor.accel_y)) <= 4 * G,
                        "Audit policy: out-of-range acceleration must saturate or be invalid")

    def test_imu_gyro_respects_documented_range(self):
        sensor = ideal_imu()
        sensor.update(2000 * DEG, 0, 0, 0)
        self.assertTrue(not getattr(sensor, "valid", True) or abs(sensor.rate) <= 1000 * DEG,
                        "Audit policy: out-of-range gyro rate must saturate or be invalid")

    def test_distance_interior_ray_matches_analytic_box(self):
        sensor = DistanceSensor(field_half=1, max_range=10, noise=0)
        for i, angle in enumerate((0, 0.3, math.pi / 2, 2, math.pi, 4, 5)):
            sensor.update(0, 0, angle, i * 0.03)
            expected = 1 / max(abs(math.sin(angle)), abs(math.cos(angle)))
            self.assertTrue(sensor.valid)
            self.assertAlmostEqual(sensor.distance, expected)

    def test_distance_beyond_max_is_invalid(self):
        sensor = DistanceSensor(field_half=3, max_range=2, noise=0)
        sensor.update(0, 0, 0, 0)
        self.assertFalse(sensor.valid)

    def test_distance_below_documented_min_is_invalid(self):
        sensor = DistanceSensor(field_half=1, noise=0)
        sensor.update(0.999, 0, 0, 0)
        self.assertFalse(sensor.valid, "1mm is below the V5 distance sensor's 20mm minimum")

    def test_distance_does_not_hit_infinite_wall_extensions(self):
        sensor = DistanceSensor(field_half=1, max_range=10, noise=0)
        sensor.update(2, 2, math.pi, 0)
        self.assertFalse(sensor.valid,
                         "Ray y=2 heading west misses all four segments of box [-1,1]^2")


class OrchestrationAudit(unittest.TestCase):
    def test_default_control_clock(self):
        sim = Simulator(presets.four_motor_200(), config=SimConfig(log_every=0))
        times = []
        sim.run(0.1, control=lambda s: times.append(s.t))
        self.assertEqual(len(times), 10)
        for a, b in zip(times, times[1:]):
            self.assertAlmostEqual(b - a, 0.01, delta=1e-12)

    def test_reset_resets_sensor_timebases(self):
        sim = Simulator(presets.odom_bot(), config=SimConfig(log_every=0))
        sim.run(0.1)
        sim.reset()
        stale = {name: sensor._last_update for name, sensor in
                 [("gps", sim.gps), ("distance", sim.distance), *sim.tracking.items()]
                 if sensor._last_update >= 0}
        self.assertFalse(stale, f"Sensors still on old clock after simulation time reset: {stale}")

    def test_reset_allows_immediate_fresh_gps_measurement(self):
        sim = Simulator(presets.four_motor_200(), config=SimConfig(log_every=0))
        sim.gps.position_noise = sim.gps.dropout_rate = 0
        sim.state.x = 1
        sim.run(0.1)
        sim.reset()
        sim.step()
        self.assertAlmostEqual(sim.gps.x, sim.state.x, delta=1e-12,
                               msg="Old GPS position persists until new time catches previous run")

    def test_reset_restarts_control_phase(self):
        sim = Simulator(presets.four_motor_200(), config=SimConfig(log_every=0))
        sim.run(0.005, control=lambda s: None)
        sim.reset()
        self.assertEqual(sim._control_accum, 0,
                         "Reset must not preserve the previous run's partial controller period")


if __name__ == "__main__":
    unittest.main(verbosity=2)
