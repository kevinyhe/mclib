"""Exercise actual Metro output mappings against stationary aligned goals."""
import math
import os
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(os.environ.get("VEXSIM_PATH",
                      Path(__file__).resolve().parents[2].parent / "vexsim"))))
from vexsim import Simulator, presets
from vexsim.sim import SimConfig
from vexsim.units import INCH
from metro_game import MetroGame


class MetroScoringTests(unittest.TestCase):
    def exercise(self, goal, command, *, lateral=0, preloads=1, end=1):
        sim = Simulator(presets.six_motor_450(mass_lb=14), config=SimConfig(log_every=0))
        world = MetroGame(sim, alliance="red", seed=1, preloads=preloads)
        world.game.goal_stopping_rate = 1.6
        ch = next(c for c in world.game.channels if c.name == goal)
        approach = ch.heading + (math.pi if end < 0 else 0)
        heading = approach + (math.pi if goal == "center_lower" else 0)
        distance = ch.length / 2 + 6 * INCH
        sim.set_pose(ch.x + math.cos(approach) * distance - math.sin(approach) * lateral,
                     ch.y + math.sin(approach) * distance + math.cos(approach) * lateral,
                     heading)
        # Isolate the scoring mechanism from the rest of the match inventory.
        world.game.blocks = list(world.game.held)
        block = world.game.held[0]
        world.game._place_held()
        world.set_outputs(*command)
        for _ in range(4000):
            world.step()
        self.assertEqual(len(world.game.blocks), preloads)
        self.assertIs(world.game.blocks[0], block)
        return block, world

    def test_middle_outputs_score_upper_center_goal(self):
        block, world = self.exercise("center_upper", (127, -100, False, True))
        self.assertEqual(world.outputs['top'], -100)
        self.assertEqual(block.goal, "center_upper")
        self.assertEqual(block.state, "scored")
        self.assertEqual(len(world.game.held), 0)

    def test_reverse_outputs_score_lower_center_goal(self):
        block, world = self.exercise("center_lower", (-127, -127, False, False))
        self.assertEqual(world.outputs['bottom'], -127)
        self.assertEqual(block.goal, "center_lower")
        self.assertEqual(block.state, "scored")
        self.assertEqual(len(world.game.held), 0)

    def test_misaligned_middle_shot_is_not_forced_into_goal(self):
        block, world = self.exercise("center_upper", (127, -100, False, True), lateral=18*INCH)
        self.assertNotEqual(block.goal, "center_upper")
        self.assertEqual(len(world.game.held), 0, "Mechanism should still eject the block")

    def test_slow_commands_feed_multiple_blocks_from_both_ends(self):
        for goal, command in (("center_upper", (127, -65, False, True)),
                              ("center_lower", (-80, -80, False, False))):
            for end in (-1, 1):
                with self.subTest(goal=goal, end=end):
                    _, world = self.exercise(goal, command, preloads=3, end=end)
                    self.assertEqual(len(world.game.held), 0)
                    self.assertGreaterEqual(sum(b.goal == goal for b in world.game.blocks), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
