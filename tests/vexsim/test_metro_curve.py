"""Compile Metro's corrected arc geometry and verify differential-drive radii.

Usage: python3 -B tests/vexsim/test_metro_curve.py --metro PATH
"""
import argparse
from pathlib import Path
import subprocess
import tempfile
import unittest

from metro_run import correct_curve_inner_arc

parser = argparse.ArgumentParser()
parser.add_argument("--metro", type=Path, required=True)
args, remaining = parser.parse_known_args()
source = (args.metro / "src/control.cpp").read_text()


class MetroCurveTests(unittest.TestCase):
    def test_compiled_geometry_preserves_requested_radius(self):
        corrected = correct_curve_inner_arc(source)
        start = corrected.index("  in_arc =", corrected.index("void curveCircle("))
        end = corrected.index("  stopChassis(", start)
        geometry = corrected[start:end]
        # Infer actual centre radius from wheel travel, independently of the
        # controller's radius-to-distance calculation. Both heading signs and
        # curve sides must work, including an in-place turn and a pivot wheel.
        cpp = r'''
#include <cmath>
#include <cassert>
#include <initializer_list>
int main() {
  const double distance_between_wheels = 11.375;
  for (double center_radius : {-12., -8.25, -5.6875, -2., 0., 2., 5.6875, 8.25, 12.}) {
    for (double result_angle : {-2., 2.}) {
      double in_arc, out_arc, ratio;
''' + geometry + r'''
      double inferred_radius = distance_between_wheels * (out_arc + in_arc)
                               / (2 * (out_arc - in_arc));
      assert(std::abs(inferred_radius - std::abs(center_radius)) < 1e-10);
      assert((in_arc < 0) == (std::abs(center_radius) < distance_between_wheels/2));
      assert(std::isfinite(ratio));
    }
  }
}
'''
        with tempfile.TemporaryDirectory(prefix="metro-curve-") as directory:
            root = Path(directory)
            (root / "curve.cpp").write_text(cpp)
            subprocess.run(["g++", "-std=c++17", str(root / "curve.cpp"),
                            "-o", str(root / "curve")], check=True, capture_output=True)
            subprocess.run([str(root / "curve")], check=True, capture_output=True)

    def test_correction_fails_closed_on_changed_or_duplicate_source(self):
        with self.assertRaises(ValueError):
            correct_curve_inner_arc(correct_curve_inner_arc(source))
        with self.assertRaises(ValueError):
            correct_curve_inner_arc(source + source)


if __name__ == "__main__":
    unittest.main(argv=[__file__, *remaining], verbosity=2)
