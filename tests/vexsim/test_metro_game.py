#!/usr/bin/env python3
"""Bounded native game/coupling checks; no C++ builds, server or viewer jobs."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(os.environ.get("VEXSIM_PATH",
                      Path(__file__).resolve().parents[2].parent / "vexsim")).resolve()))

from vexsim import Simulator, presets
from vexsim.game import Block, Game, Intake
from vexsim.sim import SimConfig
from vexsim.field import HALF_X
from vexsim.units import INCH
from metro_game import MetroGame


class MetroGameTests(unittest.TestCase):
    def test_goal_sliding_duration_increases_150_percent_without_changing_floor_or_flight(self):
        g = self.coupling()
        g.game.goal_stopping_rate = 4.0
        stopped = []
        for rate in (4.0, 1.6):
            g.game.goal_stopping_rate = rate
            block = Block(x=0, y=0, colour="red", support="channel", vx=1.0)
            steps = 0
            while block.speed > 0 and steps < 10000:
                g.game._friction([block], .001)
                self.assertGreaterEqual(block.vx, 0)
                steps += 1
            self.assertLess(steps, 10000)
            stopped.append(steps * .001)
        self.assertAlmostEqual(stopped[1] / stopped[0], 2.5, delta=.02)
        for support in ("floor", "", "block", "loader"):
            block = Block(x=0, y=0, colour="red", support=support, vx=1.0)
            expected = deepcopy(block)
            g.game._friction([block], .001)
            Game._friction(g.game, [expected], .001)
            self.assertEqual(vars(block), vars(expected), support)
        print(f"Goal stop: {stopped[0]:.3f} s previous → {stopped[1]:.3f} s (+150% duration)")

    def test_competition_layout_and_mixed_loader_stacks(self):
        g = self.coupling()
        floor = [b for b in g.game.blocks if b.state == "field"]
        self.assertEqual(len(floor), 36)
        self.assertEqual(sum(b.colour == "red" for b in floor), 18)
        center = [b for b in floor if abs(b.x) < 30 * INCH and abs(b.y) < 30 * INCH]
        self.assertEqual(len(center), 12)
        under_goals = [b for b in floor if abs(b.x) < 6 * INCH and abs(b.y) > 40 * INCH]
        self.assertEqual(len(under_goals), 8)
        for i, loader in enumerate(g.game.field.loaders):
            stack = sorted((b for b in g.game.blocks if b.loader == i), key=lambda b: b.z)
            other = "blue" if loader.alliance == "red" else "red"
            self.assertEqual([b.colour for b in stack], [loader.alliance] * 3 + [other] * 3)

    def test_double_loader_rate_preserves_floor_intake_rate(self):
        g = self.coupling()
        g.game.matchload_rate = 2.0
        before = len(g.game.blocks)
        loader_block = next(b for b in g.game.blocks if b.state == "loader")
        g.game._take(loader_block)
        self.assertAlmostEqual(g.game.intake_timer, .165)
        floor_block = next(b for b in g.game.blocks if b.state == "field")
        g.game._take(floor_block)
        self.assertAlmostEqual(g.game.intake_timer, .33)
        self.assertEqual(len(g.game.blocks), before)

    def coupling(self, preloads=1, alliance="red", seed=7):
        sim = Simulator(presets.six_motor_450(), config=SimConfig(log_every=0))
        sim.set_pose(-46 * INCH, -13.5 * INCH, 0)
        return MetroGame(sim, alliance=alliance, seed=seed, preloads=preloads)

    def test_initialization_has_explicit_identity_preserving_preloads_and_no_tick(self):
        g = self.coupling()
        self.assertEqual(g.sim.t, 0)
        self.assertEqual(g.sim.steps, 0)
        self.assertEqual(g.game.elapsed, 0)
        self.assertEqual(len(g.game.blocks), 61)
        self.assertEqual(len({id(b) for b in g.game.blocks}), 61)
        self.assertEqual(len(g.game.held), 1)
        self.assertTrue(any(b is g.game.held[0] for b in g.game.blocks))
        self.assertEqual(g.game.held[0].colour, "red")
        held = g.game.held[0]
        forward, height = g.game.intake.at_distance(held.ride)
        self.assertAlmostEqual(held.x, g.sim.state.x + forward)
        self.assertAlmostEqual(held.y, g.sim.state.y)
        self.assertAlmostEqual(held.z, height)
        blue = self.coupling(preloads=["blue", "red"], alliance="blue")
        self.assertEqual([b.colour for b in blue.game.held], ["blue", "red"])
        self.assertEqual(len(blue.game.blocks), 62)
        self.assertEqual(len(self.coupling(preloads=0).game.blocks), 60)

    def test_tick_order_and_sensor_refresh_match_native_driver(self):
        g = self.coupling()
        order = []
        sim_step, perimeter, game_step = g.sim.step, g.game._perimeter, g.game.step
        with patch.object(g.sim, "step", side_effect=lambda dt: (order.append("sim"), sim_step(dt))), \
             patch.object(g.game, "_perimeter", side_effect=lambda: (order.append("wall"), perimeter())[1]), \
             patch.object(g.game, "step", side_effect=lambda dt: (order.append("game"), game_step(dt))), \
             patch.object(g.sim, "_update_sensors", wraps=g.sim._update_sensors) as sensors:
            g.step()
        self.assertEqual(order, ["sim", "wall", "game"])
        self.assertEqual(sensors.call_count, 1)
        self.assertEqual(g.sim.t, .001)
        self.assertEqual(g.game.elapsed, .001)
        self.assertEqual(g.game._accum, .001)
        for _ in range(3):
            g.step()
        self.assertAlmostEqual(g.game._accum, 0)
        self.assertEqual(g.sim.steps, 4)
        self.assertAlmostEqual(g.game.elapsed, g.sim.t)

    def test_powered_tick_matches_native_game_and_driver_order(self):
        actual = self.coupling()
        reference_sim = Simulator(presets.six_motor_450(), config=SimConfig(log_every=0))
        reference_sim.set_pose(-46 * INCH, -13.5 * INCH, 0)
        reference = Game(SimpleNamespace(sim=reference_sim, chassis=reference_sim.chassis),
                         alliance="red", seed=7, intake=deepcopy(Intake()))
        # Compare native stepping from the same corrected competition layout.
        reference.blocks = deepcopy(actual.game.blocks)
        reference.held = [b for b in reference.blocks if b.state == "held"]
        reference.held[0].ride = reference.intake.stop_at()
        reference._place_held()
        reference.intake_running = reference.scoring = True
        actual.set_outputs(127, 127, True, False)
        for _ in range(40):
            actual.step()
            reference_sim.step(.001)
            reference._perimeter()
            reference.step(.001)
        self.assertEqual(vars(actual.sim.state), vars(reference_sim.state))
        self.assertEqual(actual.sim.imu.heading, reference_sim.imu.heading)
        self.assertEqual([vars(b) for b in actual.game.blocks],
                         [vars(b) for b in reference.blocks])
        self.assertEqual(actual.game.snapshot(), reference.snapshot())

    def test_supplied_physical_track_geometry_and_pose_are_preserved(self):
        sim = Simulator(presets.six_motor_450(), config=SimConfig(log_every=0))
        for unit in sim.chassis.driven_units():
            unit.y = math.copysign(11.375 * INCH / 2, unit.y)
        sim.set_pose(3 * INCH, -7 * INCH, .4)
        g = MetroGame(sim, alliance="blue", seed=12, preloads=1)
        self.assertIs(g.sim, sim)
        self.assertIs(g.game.robot.sim, sim)
        self.assertIs(g.game.robot.chassis, sim.chassis)
        self.assertEqual((sim.state.x, sim.state.y, sim.state.theta),
                         (3 * INCH, -7 * INCH, .4))
        frame = g.snapshot((7, 3, -math.degrees(.4)))
        self.assertAlmostEqual(frame["metro"]["physical_configuration"]["track_width_in"], 11.375)
        self.assertEqual(sim.t, 0)

    def test_disabled_blocks_do_not_self_power_or_release(self):
        g = self.coupling()
        held = g.game.held[0]
        initial = held.ride
        for _ in range(80):
            g.step()
        self.assertFalse(g.game.rollers_on)
        self.assertFalse(g.game.intake_running)
        self.assertFalse(g.game.scoring)
        self.assertEqual(g.game.held, [held])
        self.assertGreaterEqual(held.ride, initial - 1e-9)
        self.assertTrue(all(r.rpm == 0 and r.grip == 0 for r in g.game.intake.rollers))
        self.assertEqual(g.game.gate_fraction(), 0)
        self.assertEqual(g.snapshot((13.5, -46, 0))["metro"]["native_stage_angles_rad"], [0] * 6)

    def test_signed_outputs_routes_and_reverse_only_are_honest(self):
        g = self.coupling()
        g.set_outputs(127, 0, False, False, state="INDEX")
        self.assertTrue(g.game.intake_running)
        self.assertFalse(g.game.scoring)
        self.assertEqual([r.rpm for r in g.game.intake.rollers], [560, 560, 420, 280, 0, 0])
        g.set_outputs(127, 127, True, False, state="SCORE")
        self.assertTrue(g.game.scoring)
        self.assertEqual(g.game.intake.route_name, "long")
        before = [(b.x, b.y, b.z, b.state, b.ride) for b in g.game.blocks]
        held = list(g.game.held)
        g.set_outputs(-60, -90, False, False, state="MIDDLE_GOAL_PRIME")
        self.assertEqual(before, [(b.x, b.y, b.z, b.state, b.ride) for b in g.game.blocks])
        self.assertEqual(held, g.game.held)
        self.assertFalse(g.game.scoring)
        self.assertFalse(g.reverse_transport_unsupported)
        self.assertTrue(all(r.rpm < 0 for r in g.game.intake.rollers))
        self.assertEqual(g.game.intake.route_name, "long")
        g.set_outputs(127, -65, False, True, state="MIDDLE_GOAL_SCORE")
        self.assertEqual(g.game.intake.route_name, "center")
        self.assertTrue(g.game.scoring)
        self.assertAlmostEqual(g.game.intake.rollers[-1].rpm, 815 * 65 / 127)
        self.assertEqual(g.outputs["top"], -65)
        self.assertFalse(g.reverse_transport_unsupported)
        self.assertIsNone(g.game.score_at)  # outputs do not themselves release

    def test_native_stage_defaults_are_not_mutated_between_games(self):
        nominal = [(r.rpm, r.grip) for r in Intake().rollers]
        a = self.coupling()
        b = self.coupling()
        a.set_outputs(127, -65, False, True)
        self.assertTrue(all(r.rpm == 0 for r in b.game.intake.rollers))
        self.assertEqual([(r.rpm, r.grip) for r in Intake().rollers], nominal)
        self.assertIsNot(a.game.intake.rollers[0], b.game.intake.rollers[0])

    def test_collect_only_native_mouth_no_arbitrary_transfer_and_conservation(self):
        g = self.coupling()
        identities = {id(b) for b in g.game.blocks}
        far = next(b for b in g.game.blocks if b.state == "field")
        far.x, far.y = 0, 0  # explicit test placement, remote from intake
        g.set_outputs(127, 0, False, False, state="INDEX")
        for _ in range(20):
            g.step()
        self.assertNotIn(far, g.game.held)
        near = next(b for b in g.game.blocks if b.state == "field" and b is not far)
        near.x = g.sim.state.x + g.sim.chassis.length / 2 + 2 * INCH
        near.y = g.sim.state.y
        near.vx = near.vy = near.vz = 0
        g.step()
        self.assertIn(near, g.game.held)
        self.assertEqual(identities, {id(b) for b in g.game.blocks})
        self.assertEqual(len(g.game.blocks), 61)
        for _ in range(40):
            g.step()
        self.assertEqual(identities, {id(b) for b in g.game.blocks})
        self.assertTrue(all(any(b is member for member in g.game.blocks) for b in g.game.held))
        # The deliberately remote block was placed over the native center goal
        # and may score under gravity; the collected block must not be awarded
        # a score merely because collection was requested.
        self.assertEqual(near.state, "held")
        self.assertEqual(near.goal, "")

    def test_perimeter_contact_uses_native_impulse_not_external_force_reapplication(self):
        g = self.coupling(preloads=0)
        g.sim.state.x = HALF_X - g.sim.chassis.length / 2 + .005
        g.sim.state.y = 0
        g.sim.state.u = 1
        g.step()
        self.assertTrue(g.wall_contact)
        self.assertLessEqual(g.sim.state.x, HALF_X - g.sim.chassis.length / 2 + 1e-12)
        self.assertLessEqual(g.sim.state.u, 1e-12)
        self.assertEqual((g.sim.dynamics.ext_fx, g.sim.dynamics.ext_fy,
                          g.sim.dynamics.ext_mz), (0, 0, 0))

    def test_native_block_contact_changes_chassis_and_preserves_identity(self):
        g = self.coupling(preloads=0)
        block = next(b for b in g.game.blocks if b.state == "field")
        block.x = g.sim.state.x + g.sim.chassis.length / 2 + 1.5 * INCH
        block.y = g.sim.state.y
        block.vx = block.vy = block.vz = 0
        g.sim.state.u = .5
        initial = {id(b) for b in g.game.blocks}
        for _ in range(4):
            g.step()
        self.assertGreater(block.vx, 0)
        self.assertLess(g.sim.state.u, .5)
        self.assertNotIn(block, g.game.held)
        self.assertEqual(initial, {id(b) for b in g.game.blocks})

    def test_snapshot_uses_supplied_cpp_pose_real_wheels_and_detaches_data(self):
        g = self.coupling()
        g.sim.odometry.x = 123456
        g.sim.odometry.y = -123456
        for u in g.sim.chassis.left_units():
            u.angle = 2.25
        for u in g.sim.chassis.right_units():
            u.angle = -1.75
        telemetry = {"target": 37, "missing": math.nan}
        before = (g.sim.t, g.sim.steps, g.game.elapsed, g.sim.imu._rng.getstate(),
                  [(b.x, b.y, b.z, b.ride) for b in g.game.blocks])
        frame = g.snapshot((12, -8, 90), telemetry, {"stop": "BRAKE"})
        self.assertAlmostEqual(frame["odom"]["x"], -8 * INCH)
        self.assertAlmostEqual(frame["odom"]["y"], -12 * INCH)
        self.assertAlmostEqual(frame["odom"]["theta"], -math.pi / 2)
        self.assertAlmostEqual(frame["robot"]["x"], -46 * INCH)
        self.assertAlmostEqual(frame["metro"]["actual_pose"]["y_in"], -46)
        self.assertEqual(frame["metro"]["odometry_pose"], {"x_in": 12, "y_in": -8, "heading_deg": 90})
        self.assertEqual(frame["metro"]["wheel_angles_rad"], {"left": 2.25, "right": -1.75})
        self.assertIsNone(frame["metro"]["telemetry"]["missing"])
        self.assertEqual(frame["cmd"]["stop"], "BRAKE")
        self.assertEqual(before[:3], (g.sim.t, g.sim.steps, g.game.elapsed))
        self.assertEqual(before[3], g.sim.imu._rng.getstate())
        self.assertEqual(before[4], [(b.x, b.y, b.z, b.ride) for b in g.game.blocks])
        self.assertTrue({"route", "aimed_at", "in_range", "goals", "red", "blue"} <= frame["game"].keys())
        json.dumps(frame, allow_nan=False)
        frame["blocks"][0]["x"] = 999
        frame["metro"]["outputs"]["bottom"] = 999
        telemetry["target"] = 999
        self.assertNotEqual(g.game.blocks[0].x, 999)
        self.assertEqual(g.outputs["bottom"], 0)
        self.assertEqual(frame["metro"]["telemetry"]["target"], 37)

    def test_missing_fidelity_and_scraper_wing_are_visible_not_collision_shapes(self):
        g = self.coupling()
        body = g.game._boxes()
        g.set_outputs(0, 0, False, False, scraper=True, wing=True)
        self.assertEqual(body, g.game._boxes())
        frame = g.snapshot((13.5, -46, 0))
        fidelity = frame["metro"]["fidelity"]
        self.assertIn("scraper_collision", fidelity["unsupported"])
        self.assertIn("intake_motor_electrical_load", fidelity["unsupported"])
        self.assertTrue(any("currently deployed" in w for w in frame["warnings"]))
        self.assertEqual(frame["metro"]["physical_configuration"]["drive_motors"], 6)
        self.assertEqual(frame["metro"]["physical_configuration"]["cartridges"], ["blue"] * 6)
        self.assertAlmostEqual(frame["metro"]["physical_configuration"]["mass_lb"], 16.5)

    def test_deterministic_game_seed_and_snapshot_observation_parity(self):
        a, b = self.coupling(seed=42), self.coupling(seed=42)
        for coupling in (a, b):
            coupling.start()
            coupling.set_outputs(127, 0, False, False)
        for _ in range(20):
            a.step()
            a.snapshot((13.5, -46, 0))
            b.step()
        self.assertEqual(a.snapshot((13.5, -46, 0)), b.snapshot((13.5, -46, 0)))
        self.assertEqual(a.game.period, "auton")
        self.assertAlmostEqual(a.game.t, a.sim.t)

    def test_invalid_inputs_fail_before_physics_or_output_mutation(self):
        for kwargs in ({"alliance": "purple"}, {"seed": True}, {"preloads": -1},
                       {"preloads": 10}, {"preloads": True}, {"preloads": ["purple"]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.coupling(**kwargs)
        g = self.coupling()
        original = dict(g.outputs)
        for args in ((128, 0, False, False), (math.nan, 0, False, False),
                     (0, 0, 1, False), (False, 0, False, False)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                g.set_outputs(*args)
            self.assertEqual(g.outputs, original)
        for dt in (0, .004, .01, math.nan, True):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                g.step(dt)
        self.assertEqual(g.sim.t, 0)
        with self.assertRaises(ValueError):
            g.snapshot((0, 0, math.nan))

    def test_optical_proxy_reads_actual_preload_and_prime_does_not_fake_clear(self):
        g = self.coupling(preloads=["blue"])
        reading = g.optical_sample()
        self.assertEqual(reading["proximity"], 255)
        self.assertEqual(reading["hue"], 210)
        self.assertEqual(reading["distance_mm"], 0)
        self.assertEqual(reading["detected_state"], "held")
        g.set_outputs(-60, -90, False, False, state="MIDDLE_GOAL_PRIME")
        for _ in range(80):
            g.step()
        # PRIME now physically backs the block away from the sensor rather
        # than suppressing both motors. Occupancy still follows geometry.
        after = g.optical_sample()
        self.assertEqual(after["proximity"], 0)
        self.assertGreater(after["distance_mm"], 0)
        self.assertEqual(len(g.game.held), 1, "PRIME must not eject the block")
        self.assertEqual(g.optical_sample()["body_position_in"], reading["body_position_in"])
        self.assertEqual(g.optical_sample(forward_in=30, height_in=40)["proximity"], 0)
        self.assertIn("uncalibrated", reading["model"])
        self.assertEqual(g.sim.steps, 80)
        with self.assertRaises(ValueError):
            g.optical_sample(detection_radius_in=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
