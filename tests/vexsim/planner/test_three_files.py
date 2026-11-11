import json
from pathlib import Path
import tempfile
import unittest

from three_files import THREE_VERSION, ThreeMissing, asset


def checkout(version):
    root = Path(tempfile.mkdtemp())
    package = root / "node_modules" / "three"
    (package / "build").mkdir(parents=True)
    (package / "package.json").write_text(json.dumps(dict(version=version)))
    (package / "build" / "three.module.js").write_text("export const REVISION = '160';")
    return root


class ThreeFilesTests(unittest.TestCase):
    def test_serves_the_module_the_3d_view_imports(self):
        self.assertEqual(asset("/three/three.module.js", checkout(THREE_VERSION)),
                         (b"export const REVISION = '160';", "text/javascript; charset=utf-8"))

    def test_other_paths_are_not_served(self):
        root = checkout(THREE_VERSION)
        for path in ("/three/package.json", "/three/../three/three.module.js", "/three/", "/three/build/three.module.js"):
            self.assertIsNone(asset(path, root), path)

    def test_another_version_is_refused_with_the_one_it_needs(self):
        with self.assertRaisesRegex(ThreeMissing, "needs Three.js 0.160.0; .* has 0.161.0"):
            asset("/three/three.module.js", checkout("0.161.0"))

    def test_a_missing_install_says_how_to_install_it(self):
        with self.assertRaisesRegex(ThreeMissing, "npm install --prefix .* --no-save three@0.160.0"):
            asset("/three/three.module.js", Path(tempfile.mkdtemp()))


if __name__ == "__main__":
    unittest.main()
