#!/usr/bin/env python3
"""Independent, read-only contact and 3D-driver checks for the sibling vexsim.

Run: PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_game_audit.py
The optional --vexsim PATH selects another checkout. Failures are unresolved
acceptance failures, not expected-failure passes. No hardware calibration is
implied. Synthetic isolated contacts deliberately remove unrelated geometry.
"""

import argparse
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--vexsim", type=Path, default=Path(__file__).resolve().parents[3] / "vexsim")
args, unittest_args = parser.parse_known_args()
sys.path.insert(0, str(args.vexsim.resolve()))

from vexsim import presets
from vexsim.api import Robot
from vexsim.field import BLOCK_MASS, BLOCK_RESTITUTION, HALF_X
from vexsim.game import BLOCK_R, GAME_DT, Block, Game
from vexsim.sim import SimConfig
from vexsim.units import G
from vexsim.viz3d import Driver


def isolated_game():
    robot = Robot(presets.six_motor_450(), config=SimConfig(log_every=0))
    game = Game(robot)
    game.blocks = []
    game.held = []
    game.solids = []
    game.platforms = []
    game.obstacles = []
    game.channels = []
    return robot, game


class GamePhysicsAudit(unittest.TestCase):
    def test_airborne_block_matches_ballistic_motion(self):
        _, game = isolated_game()
        block = Block(x=0.5, y=0.5, z=1.0, colour="red", vx=0.3, vy=-0.2)
        game.blocks = [block]
        duration = 0.2
        for _ in range(round(duration / GAME_DT)):
            game.step(GAME_DT)
        self.assertAlmostEqual(block.x, 0.5 + 0.3 * duration, places=12)
        self.assertAlmostEqual(block.y, 0.5 - 0.2 * duration, places=12)
        self.assertAlmostEqual(block.vz, -G * duration, places=12)
        # First-order integration must converge within half g*dt*t here.
        self.assertAlmostEqual(block.z, 1.0 - 0.5 * G * duration**2,
                               delta=0.5 * G * GAME_DT * duration + 1e-12)

    def test_free_flight_preserves_spin_and_normalized_orientation(self):
        _, game = isolated_game()
        block = Block(x=0.5, y=0.5, z=1.0, colour="red", spin=2.0, roll=0.4)
        game.blocks = [block]
        for _ in range(50):
            game.step(GAME_DT)
        self.assertEqual(block.spin, 2.0)
        self.assertEqual(block.roll, 0.4)
        self.assertAlmostEqual(sum(q*q for q in (block.qx, block.qy, block.qz, block.qw)),
                               1.0, places=12)

    def test_equal_mass_collision_preserves_momentum_and_restitution(self):
        _, game = isolated_game()
        a = Block(x=0.5, y=0.5, z=1.0, colour="red", vx=1.0)
        b = Block(x=0.5 + 2*BLOCK_R - 0.001, y=0.5, z=1.0, colour="blue")
        game._pairs([a, b], GAME_DT)
        self.assertAlmostEqual(BLOCK_MASS * (a.vx + b.vx), BLOCK_MASS, places=12)
        self.assertAlmostEqual(b.vx - a.vx, BLOCK_RESTITUTION, places=12)
        self.assertLessEqual(a.vx*a.vx + b.vx*b.vx, 1.0)

    def test_block_wall_reflects_normal_velocity(self):
        _, game = isolated_game()
        block = Block(x=HALF_X, y=0.5, z=0.1, colour="red", vx=1.0)
        game._walls([block])
        self.assertAlmostEqual(block.x, HALF_X - BLOCK_R)
        self.assertAlmostEqual(block.vx, -BLOCK_RESTITUTION)
        self.assertEqual(block.vy, 0.0)

    def test_3d_containment_resolves_wall_without_adding_energy(self):
        driver = Driver()
        state = driver.robot.sim.state
        state.x, state.y, state.theta = HALF_X, 0.5, 0.0
        state.u, state.v, state.r = 1.0, 0.25, 0.3
        chassis = driver.robot.chassis
        before = 0.5 * chassis.mass * (state.u**2 + state.v**2) + 0.5 * chassis.inertia_z * state.r**2
        driver._contain()
        self.assertAlmostEqual(state.x, HALF_X - driver.robot.chassis.length / 2)
        self.assertAlmostEqual(state.u, 0.0, places=12)
        # Wall friction is now shared with the game solver.  The original
        # translation-only driver left tangential speed and yaw unchanged.
        after = 0.5 * chassis.mass * (state.u**2 + state.v**2) + 0.5 * chassis.inertia_z * state.r**2
        self.assertLessEqual(after, before + 1e-12)

    def test_new_games_with_same_seed_have_same_initial_blocks(self):
        a, b = Game(Robot(presets.six_motor_450())), Game(Robot(presets.six_motor_450()))
        self.assertEqual([(x.x, x.y, x.yaw) for x in a.blocks],
                         [(x.x, x.y, x.yaw) for x in b.blocks])

    def test_driver_preserves_elapsed_wall_time(self):
        driver = Driver()
        driver.game.blocks = []
        driver.game.held = []
        times = iter([0.0] + [0.0029*i for i in range(1, 101)])
        loops = 0

        def sleep(_):
            nonlocal loops
            loops += 1
            if loops == 100:
                driver.quit = True

        with patch("vexsim.viz3d.time.perf_counter", side_effect=lambda: next(times)), \
             patch("vexsim.viz3d.time.sleep", side_effect=sleep):
            driver.run()
        print(f"TIMING wall_s=0.29 sim_s={driver.robot.sim.t:.9f} "
              f"game_s={driver.game.elapsed:.9f}", flush=True)
        self.assertAlmostEqual(driver.robot.sim.t, 0.29,
                               delta=driver.robot.sim.config.dt + 1e-12,
                               msg="fractional timestep remainders must survive between loops")

    def test_passive_static_contact_cannot_create_chassis_kinetic_energy(self):
        robot, game = isolated_game()
        state, chassis = robot.sim.state, robot.chassis
        state.r = 1.0
        # Ordinary chassis, one small fixed post meeting its front near a corner.
        # The post intersects the frame's height but not the tower above it.
        game.solids = [(chassis.length / 2 + 0.005, -0.99 * chassis.width / 2,
                        0.01, 0.03, 0.09)]
        before = 0.5 * chassis.inertia_z
        game._structures(GAME_DT)
        after = (0.5 * chassis.mass * (state.u**2 + state.v**2)
                 + 0.5 * chassis.inertia_z * state.r**2)
        print(f"STATIC_CONTACT before_J={before:.9f} after_J={after:.9f} "
              f"ratio={after/before:.6f}", flush=True)
        self.assertLessEqual(after, before + 1e-12,
                             "a stationary obstacle with friction must not add energy")

    def test_batching_does_not_discard_block_reaction_impulse(self):
        def collision():
            robot, game = isolated_game()
            robot.sim.state.u = 1.0
            game.blocks = [Block(x=robot.chassis.length / 2 + BLOCK_R - 0.001,
                                 y=0.0, z=0.05, colour="red")]
            return robot, game

        sequential_robot, sequential = collision()
        impulses = []
        for _ in range(2):
            sequential.step(GAME_DT)
            impulses.append(sequential.reaction[0] * GAME_DT)
        robot, batched = collision()
        robot.sim.dynamics.ext_fx = 7.0
        batched.step(2 * GAME_DT)
        # Originally this checked ext_fx * batch duration: the 8 ms call
        # exposed zero after overwriting its first -0.0568 Ns collision.
        # Contact impulses now belong directly to the two bodies.  Compare
        # actual momentum, and leave caller-owned external forces untouched.
        exposed = robot.chassis.mass * (robot.sim.state.u - 1.0)
        sequential_momentum = sequential_robot.chassis.mass * (sequential_robot.sim.state.u - 1.0)
        print(f"REACTION sequential_impulse_Ns={sum(impulses):.9f} "
              f"batch_exposed_impulse_Ns={exposed:.9f} "
              f"block_vx_mps={batched.blocks[0].vx:.9f}", flush=True)
        self.assertAlmostEqual(exposed, sum(impulses), places=12,
                               msg="all block substep impulses must reach the robot")
        self.assertAlmostEqual(exposed, sequential_momentum, places=12)
        self.assertAlmostEqual(exposed, batched.reaction[0] * (2 * GAME_DT), places=12)
        self.assertEqual(robot.sim.dynamics.ext_fx, 7.0)
        velocity_after_contact = robot.sim.state.u
        batched.step(GAME_DT)
        self.assertEqual(robot.sim.state.u, velocity_after_contact,
                         "old impulses must not be applied again after separation")

    def test_dashboard_battery_current_matches_pack_current(self):
        driver = Driver()
        driver.robot.sim.dynamics.locked = True
        driver.robot.tank(6.0, 6.0)
        for _ in range(100):
            driver.robot.sim.step()
        snapshot = driver.snapshot()
        battery = driver.robot.sim.battery
        print(f"DASHBOARD displayed_A={snapshot['battery']['amps']:.6f} "
              f"actual_pack_A={battery.current:.6f} "
              f"displayed_W={snapshot['battery']['watts']:.6f} "
              f"actual_pack_W={battery.current*battery.voltage:.6f}", flush=True)
        self.assertAlmostEqual(snapshot["battery"]["amps"], battery.current, delta=0.011,
                               msg="the battery panel must report pack current, not winding current")


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]] + unittest_args, verbosity=2)
