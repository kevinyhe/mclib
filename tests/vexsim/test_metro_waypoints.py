"""Verify corrections preserve Metro's routine rather than replacing its route."""
import argparse
from pathlib import Path
import re
import sys
import unittest

from metro_waypoints import POINTS, correct_waypoints

parser = argparse.ArgumentParser()
parser.add_argument('--metro', type=Path, required=True)
args, remaining = parser.parse_known_args()
original = (args.metro / 'src/autonomous.cpp').read_text()


class MetroWaypointTests(unittest.TestCase):
    def test_original_points_restore_original_autonomous_exactly(self):
        original_points = [(-9, 24), (-18, -10), (-19.5, -27.5), (-19.5, 22), (38, 23)]
        self.assertEqual(correct_waypoints(original, original_points), original)

    def test_active_user_points_are_applied(self):
        result = correct_waypoints(original)
        start = result.index('void leftSideSevenMiddle()')
        end = result.index('void rightSideSevenMiddle()', start)
        points = re.findall(r'moveToPoint\(([-\d.]+), ([-\d.]+),', result[start:end])
        self.assertEqual([tuple(map(float, p)) for p in points], list(POINTS))

    def test_edits_change_only_selected_point_coordinates(self):
        points = [(-20, 24), (-18, -10), (-19.5, -27.5), (-19.5, 22), (38, 23)]
        self.assertEqual(correct_waypoints(original, points),
                         original.replace('moveToPoint(-9, 24,', 'moveToPoint(-20, 24,', 1))

    def test_rejects_source_drift(self):
        with self.assertRaises(ValueError):
            correct_waypoints(original + original)
        with self.assertRaises(ValueError):
            correct_waypoints(original.replace('moveToPoint(-9, 24, 1, 500',
                                               'moveToPoint(-9, 24, 1, 501'))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
