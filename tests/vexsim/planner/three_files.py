"""Three.js from the vexsim checkout's node_modules, for the 3D view."""
import json
from pathlib import Path

THREE_VERSION = "0.160.0"
ROUTES = {"/three/three.module.js": "build/three.module.js"}


class ThreeMissing(RuntimeError):
    pass


def asset(path, vexsim_root):
    """(bytes, content type) for a Three.js route, or None for any other path."""
    if path not in ROUTES:
        return None
    package = Path(vexsim_root).resolve() / "node_modules" / "three"
    try:
        version = json.loads((package / "package.json").read_text())["version"]
        body = (package / ROUTES[path]).read_bytes()
    except (OSError, ValueError, KeyError) as error:
        raise ThreeMissing(f"The 3D view needs Three.js {THREE_VERSION} in {package}. "
                           f"Run: npm install --prefix {Path(vexsim_root).resolve()} --no-save three@{THREE_VERSION}") from error
    if version != THREE_VERSION:
        raise ThreeMissing(f"The 3D view needs Three.js {THREE_VERSION}; {package} has {version}.")
    return body, "text/javascript; charset=utf-8"
