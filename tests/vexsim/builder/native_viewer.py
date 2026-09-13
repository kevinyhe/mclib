"""Serve the installed vexsim renderer as a passive, same-origin recording view.

The native HTML is read in place, never copied into this repository. Checked
transport substitutions preserve its scene, chassis placeholder and cameras.
No Driver, Game, simulation loop, remote dependency, or drive input is started.
"""
import base64
from functools import lru_cache
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import re
import sys
import threading


class NativeViewerError(RuntimeError):
    """The installed viewer is missing or differs from the supported source."""


THREE_FILES = (
    "build/three.module.js",
    "examples/jsm/loaders/FBXLoader.js",
    "examples/jsm/controls/OrbitControls.js",
    "examples/jsm/utils/BufferGeometryUtils.js",
    "examples/jsm/libs/fflate.module.js",
    "examples/jsm/curves/NURBSCurve.js",
    "examples/jsm/curves/NURBSUtils.js",
)
_FIELD_LOCK = threading.Lock()


def _replace_once(text, old, new):
    if text.count(old) != 1:
        raise NativeViewerError(f"Unsupported native viewer anchor: {old[:70]}")
    return text.replace(old, new, 1)


def _replace_region(text, start, end, replacement):
    if text.count(start) != 1 or text.count(end) != 1:
        raise NativeViewerError(f"Unsupported native viewer region: {start}")
    first, last = text.index(start), text.index(end)
    if last <= first:
        raise NativeViewerError("Native viewer sections are out of order")
    return text[:first] + replacement + text[last:]


_INPUT = """// Playback accepts camera keys only; it never sends drive inputs.
const VIEW_KEYS = { '1': 'operator', '2': 'audience', '3': 'overhead', '4': 'free' };
let view = 'overhead';
addEventListener('keydown', ev => {
  const v = VIEW_KEYS[ev.key];
  if (v) { ev.preventDefault(); setView(v); }
});

"""

_PLAYBACK = """
// The parent owns playback time. These are recorded physics coordinates,
// converted to vexsim's native metres/CCW convention, never a controller.
builtGoals.visible = false;
builtProps.visible = false;
for (const child of builtGround.children) child.visible = child === floor;
robot.visible = false;
const playbackStatus = document.getElementById('native-playback-status');
let playbackGeometry = null;
let lastFitBounds = null;
let pendingFitBounds = null;
const finite = value => typeof value === 'number' && Number.isFinite(value);
function tellParent(message) { parent.postMessage(message, location.origin); }
function clearRecording(reason, t = null) {
  state = null;
  robot.visible = false;
  playbackStatus.textContent = 'vexsim native viewer · open floor physics · ' + reason;
  tellParent({type: 'mclib-vexsim-applied', t, robot: null, available: false,
              reason, wheels_rendered: false});
}
function recordedFrame(frame, playing, live, checkViewport = false) {
  if (frame === null) { clearRecording('no recorded frame'); return; }
  if (!frame || typeof frame !== 'object' ||
      !['t', 'true_x', 'true_y', 'true_heading'].every(key => finite(frame[key])) ||
      frame.t < 0) {
    clearRecording('recorded pose unavailable');
    return;
  }
  if (![frame.body_length_in, frame.body_width_in].every(value => finite(value) && value > 0)) {
    clearRecording('recorded chassis geometry unavailable', frame.t);
    return;
  }
  const nativeRobot = {
    x: frame.true_y * IN, y: -frame.true_x * IN,
    theta: -frame.true_heading * Math.PI / 180,
    length: frame.body_length_in * IN, width: frame.body_width_in * IN,
  };
  if (!Object.values(nativeRobot).every(finite)) {
    clearRecording('recorded pose unavailable', frame.t);
    return;
  }
  const geometry = [nativeRobot.length, nativeRobot.width];
  if (body && (!playbackGeometry || geometry.some((value, i) => value !== playbackGeometry[i]))) {
    robot.remove(body);
    body.traverse(part => {
      if (part.isMesh) { part.geometry.dispose(); part.material.dispose(); }
    });
    body = null;
  }
  playbackGeometry = geometry;
  // The exact native placeholder is a shell and heading marker with no wheel
  // meshes. Preserve real recorded angles in the acknowledgement; do not
  // invent wheel animation or integrate a second clock in the browser.
  const wheelAngles = {
    left: finite(frame.left_wheel_angle_rad) ? frame.left_wheel_angle_rad : null,
    right: finite(frame.right_wheel_angle_rad) ? frame.right_wheel_angle_rad : null,
  };
  state = {t: frame.t, robot: nativeRobot, blocks: [], game: null,
           alliance: 'red', paused: !(playing === true || live === true), spin: null};
  robot.visible = true;
  apply(state);
  playbackStatus.textContent = 'vexsim native viewer · open floor physics · ' +
    (live === true ? 'live' : playing === true ? 'replay' : 'paused') +
    ' · ' + frame.t.toFixed(2) + ' s · 1–4 camera views';
  if (pendingFitBounds) {
    const bounds = pendingFitBounds;
    pendingFitBounds = null;
    fitRecording(bounds);
  }
  tellParent({type: 'mclib-vexsim-applied', t: frame.t,
              robot: {x: robot.position.x, y: -robot.position.z,
                      theta: robot.rotation.y - modelYaw,
                      length: playbackGeometry[0], width: playbackGeometry[1]},
              available: true, wheel_angles_rad: wheelAngles, wheels_rendered: false,
              ...(checkViewport ? {viewport: projectedBodyBounds()} : {})});
}
function projectedBodyBounds() {
  if (!body || !robot.visible) return null;
  robot.updateWorldMatrix(true, true);
  camera.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(body);
  const projected = {min_x: Infinity, max_x: -Infinity, min_y: Infinity, max_y: -Infinity,
                     min_z: Infinity, max_z: -Infinity};
  for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) {
    for (const z of [box.min.z, box.max.z]) {
      const point = new THREE.Vector3(x, y, z).project(camera);
      for (const axis of ['x', 'y', 'z']) {
        projected['min_' + axis] = Math.min(projected['min_' + axis], point[axis]);
        projected['max_' + axis] = Math.max(projected['max_' + axis], point[axis]);
      }
    }
  }
  return projected;
}
function fitRecording(bounds) {
  const keys = ['min_x', 'max_x', 'min_y', 'max_y'];
  if (!bounds || !keys.every(key => finite(bounds[key]) && Math.abs(bounds[key]) <= 2000) ||
      bounds.max_x < bounds.min_x || bounds.max_y < bounds.min_y) return;
  if (!body || !state || !robot.visible || !playbackGeometry) {
    pendingFitBounds = {...bounds};
    return;
  }
  robot.updateWorldMatrix(true, true);
  const box = new THREE.Box3().setFromObject(body);
  // Centre-path bounds do not include the robot. The recorded footprint and
  // native mesh bounds include its shell and protruding heading marker. This
  // radius is conservative for every heading, including a corner of the path.
  let radius = Math.hypot(...playbackGeometry) / 2;
  for (const x of [box.min.x, box.max.x]) for (const z of [box.min.z, box.max.z]) {
    radius = Math.max(radius, Math.hypot(x - robot.position.x, z - robot.position.z));
  }
  const top = Math.max(0, box.max.y);
  const centre = P((bounds.min_y + bounds.max_y) * IN / 2,
                   -(bounds.min_x + bounds.max_x) * IN / 2, 0);
  // Native overhead screen width spans mclib Y; screen height spans mclib X.
  const width = Math.max(24 * IN, (bounds.max_y - bounds.min_y) * IN + 2 * radius);
  const depth = Math.max(24 * IN, (bounds.max_x - bounds.min_x) * IN + 2 * radius);
  const overlay = playbackStatus.getBoundingClientRect().height + 16;
  const safeX = Math.max(.2, 1 - 32 / Math.max(1, innerWidth));
  const safeY = Math.max(.2, 1 - 2 * overlay / Math.max(1, innerHeight));
  // The top of the 3D body is closer to a perspective camera than the floor.
  // Reserve its actual height, then fit the expanded bounds with 20% padding.
  const height = top + Math.max(depth / safeY, width / (camera.aspect * safeX)) * .6 /
                       Math.tan(camera.fov * Math.PI / 360);
  setView('overhead');
  controls.target.copy(centre);
  camera.position.copy(centre).add(new THREE.Vector3(0, height, .001));
  camera.far = Math.max(100, height + 100);
  camera.updateProjectionMatrix();
  camera.lookAt(centre);
  lastFitBounds = {...bounds};
  tellParent({type: 'mclib-vexsim-fitted', bounds: {...bounds}, viewport: projectedBodyBounds(),
              safe_viewport: {min_x: -safeX, max_x: safeX, min_y: -safeY, max_y: safeY},
              footprint_radius_m: radius, body_top_m: top,
              camera: {position: camera.position.toArray(), target: controls.target.toArray(),
                       aspect: camera.aspect, fov: camera.fov, near: camera.near, far: camera.far}});
}
addEventListener('resize', () => {
  if (lastFitBounds && view === 'overhead') fitRecording(lastFitBounds);
});
addEventListener('message', event => {
  if (event.origin !== location.origin || event.source !== parent) return;
  const data = event.data;
  if (!data || typeof data !== 'object') return;
  if (data.type === 'mclib-vexsim-frame') recordedFrame(data.frame, data.playing, data.live,
                                                     data.check_viewport === true);
  if (data.type === 'mclib-vexsim-fit') fitRecording(data.bounds);
  if (data.type === 'mclib-vexsim-view' && ['operator', 'audience', 'overhead', 'free'].includes(data.view)) {
    setView(data.view);
  }
});
tellParent({type: 'mclib-vexsim-ready', renderer: 'vexsim/web/push_back.html',
            surface: 'open_floor', wheels_rendered: false});
"""


def transform(source):
    """Return HTML/module/CSS from a supported native template, or fail closed."""
    extracted = {}
    for name, pattern in (("css", r"<style>(.*?)</style>"),
                          ("imports", r'<script type="importmap">(.*?)</script>'),
                          ("js", r'<script type="module">(.*?)</script>')):
        matches = list(re.finditer(pattern, source, re.S))
        if len(matches) != 1:
            raise NativeViewerError(f"Expected one native {name} section")
        extracted[name] = matches[0].group(1)
    expected_imports = {"imports": {
        "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
        "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
    }}
    try:
        native_imports = json.loads(extracted["imports"])
    except ValueError as error:
        raise NativeViewerError("Native viewer import map is invalid") from error
    if native_imports != expected_imports:
        raise NativeViewerError("Unsupported native Three.js import map")
    js = extracted["js"]
    for token, value in (("__HAS_MODEL__", "false"), ("__HAS_FIELD__", "false"),
                         ("__HAS_BLOCK__", "false"), ("__MODEL_UP__", "z"),
                         ("__MODEL_FORWARD__", "+y")):
        js = _replace_once(js, token, value)
    js = _replace_once(js, "fetch('/field')", "fetch('/native/field.json')")
    js = _replace_region(js, "// ------------------------------------------------------------------- input",
                         "// ------------------------------------------------------------------ camera", _INPUT)
    js = _replace_region(js, "async function poll() {", "let lastHud = 0;", "")
    js = _replace_once(js, "    hud(s);", "    // HUD is supplied by the parent from the same recorded frame.")
    js = _replace_region(js, "// ---------------------------------------------------------------------- HUD",
                         "// ------------------------------------------------------------------ startup", "")
    js = _replace_once(js, "await poll();\nsetView('operator');", "setView('overhead');")
    js = _replace_once(js, "  turnWheels(dt);", "  // Playback never advances wheel angles using wall time.")
    js = _replace_once(js, "\nframe();", "\nframe();\n" + _PLAYBACK)
    if re.search(r"setInterval\s*\(|fetch\(['\"]/(state|input)['\"]", js):
        raise NativeViewerError("Native viewer still contains an active live transport")
    imports = json.dumps({"imports": {
        "three": "/native/three/build/three.module.js",
        "three/addons/": "/native/three/examples/jsm/",
    }}, separators=(",", ":"))
    html = _replace_once(source, "<style>" + extracted["css"] + "</style>",
                         '<link rel="stylesheet" href="/native/viewer.css">')
    html = _replace_once(html, '<script type="importmap">' + extracted["imports"] + '</script>',
                         '<script type="importmap">' + imports + '</script>')
    html = _replace_once(html, '<script type="module">' + extracted["js"] + '</script>',
                         '<script type="module" src="/native/viewer.js"></script>')
    # The original decorative inline style attributes are unnecessary here;
    # removing them lets the frame retain a self-only stylesheet policy.
    html = re.sub(r' style="[^"]*"', '', html)
    html = _replace_once(html, "</body>", '<div id="native-playback-status" role="status">'
                         'vexsim native viewer · open floor physics · no recorded frame</div>\n</body>')
    css = extracted["css"] + """
#score, #tele, #keys, #status, #warn { display: none; }
#native-playback-status { position: fixed; left: 8px; bottom: 8px; right: 8px;
  color: #d8dee9; background: #0d0f14cc; padding: 5px 8px; font-size: 11px;
  pointer-events: none; }
"""
    digest = base64.b64encode(hashlib.sha256(imports.encode()).digest()).decode()
    csp = ("default-src 'none'; script-src 'self' 'sha256-" + digest + "'; "
           "style-src 'self'; connect-src 'self'; img-src 'self' data: blob:; "
           "frame-ancestors 'self'; base-uri 'none'; object-src 'none'")
    return {"html": html.encode(), "js": js.encode(), "css": css.encode(), "csp": csp}


def _read(root, relative):
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise NativeViewerError("Native asset resolves outside the simulator") from error
    try:
        return path.read_bytes()
    except OSError as error:
        raise NativeViewerError(f"Native asset unavailable: {relative}") from error


@lru_cache(maxsize=4)
def _bundle(root, modified, size, package_modified, package_size):
    del modified, size, package_modified, package_size
    try:
        version = json.loads(_read(root, "node_modules/three/package.json"))["version"]
        if version != "0.160.0":
            raise NativeViewerError("The native viewer requires installed Three.js 0.160.0")
        return transform(_read(root, "vexsim/web/push_back.html").decode("utf-8"))
    except (ValueError, KeyError, UnicodeError) as error:
        raise NativeViewerError("Native viewer source or dependency metadata is invalid") from error


@lru_cache(maxsize=4)
def _field(root):
    # Load the native metadata function in an isolated package namespace. This
    # does not change sys.path or select a different vexsim for the worker.
    with _FIELD_LOCK:
        name = "_mclib_native_viewer_" + hashlib.sha256(str(root).encode()).hexdigest()[:16]
        if name not in sys.modules:
            package = root / "vexsim"
            _read(root, "vexsim/__init__.py")
            spec = importlib.util.spec_from_file_location(name, package / "__init__.py",
                                                          submodule_search_locations=[str(package)])
            if spec is None or spec.loader is None:
                raise NativeViewerError("Native simulator package is unavailable")
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            try:
                spec.loader.exec_module(module)
            except Exception:
                sys.modules.pop(name, None)
                raise
        native = importlib.import_module(name + ".viz3d")
        return json.dumps(native.field_json(), allow_nan=False).encode()


def asset(path, vexsim_root):
    """Return (bytes, content type, optional CSP), or None for a disallowed path."""
    routes = {"/native/viewer.html": ("html", "text/html; charset=utf-8"),
              "/native/viewer.js": ("js", "text/javascript; charset=utf-8"),
              "/native/viewer.css": ("css", "text/css; charset=utf-8")}
    three = {"/native/three/" + item: item for item in THREE_FILES}
    if path not in routes and path not in three and path != "/native/field.json":
        return None
    root = Path(vexsim_root).resolve()
    try:
        info = (root / "vexsim/web/push_back.html").stat()
        package = (root / "node_modules/three/package.json").stat()
        bundle = _bundle(root, info.st_mtime_ns, info.st_size,
                         package.st_mtime_ns, package.st_size)
        if path in routes:
            kind, content_type = routes[path]
            return bundle[kind], content_type, bundle["csp"] if kind == "html" else None
        if path in three:
            return _read(root, "node_modules/three/" + three[path]), "text/javascript; charset=utf-8", None
        return _field(root), "application/json; charset=utf-8", None
    except NativeViewerError:
        raise
    except (OSError, ImportError, ValueError) as error:
        raise NativeViewerError("Native viewer assets or field metadata are unavailable") from error
