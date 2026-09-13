#!/usr/bin/env python3
"""Passive full-game CAD replay: python3 -B tests/vexsim/game_viewer.py RECORDING.

Serves a single metro-game-recording-v1 file on loopback (default port 8766).
Reuses the installed vexsim native scene, FBX fitting and HUD in place. No
Driver/Game instance, physics clock, input endpoint, dependency install or
sibling source modification is involved. Native CAD level-of-detail remains
native; it is not a manufacturing-detail renderer or a hardware prediction.
"""
import argparse
import base64
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import sys
from urllib.parse import urlsplit

sys.dont_write_bytecode = True
from builder.native_viewer import (NativeViewerError, THREE_FILES, _field, _read,
                                   _replace_once, _replace_region)

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "metro-game-recording-v1"
MODEL_FILES = {"/model.fbx": "sexy s bot.fbx",
               "/field.fbx": "276-9142 V5RC Push Back.fbx",
               "/block.fbx": "276-9142-001_Instruction - Blue.fbx"}
MAX_RECORDING_BYTES = 256 * 1024 * 1024


def _finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def validate_recording(recording):
    """Validate display-critical native fields without synthesizing any state."""
    if not isinstance(recording, dict) or recording.get("schema") != SCHEMA:
        raise ValueError(f"Recording must use schema {SCHEMA}")
    for key in ("metadata", "summary"):
        if not isinstance(recording.get(key), dict):
            raise ValueError(f"Recording {key} must be an object")
    for key in ("routine", "source_commit"):
        if not isinstance(recording["metadata"].get(key), str) or not recording["metadata"][key]:
            raise ValueError(f"Recording metadata.{key} is required")
    if not isinstance(recording["metadata"].get("assumptions"), (list, dict, str)):
        raise ValueError("Recording metadata.assumptions is required")
    if not isinstance(recording.get("events"), list):
        raise ValueError("Recording events must be an array")
    frames = recording.get("frames")
    if not isinstance(frames, list) or not 1 <= len(frames) <= 100000:
        raise ValueError("Recording needs 1–100000 native frames")
    previous, dimensions = -1, None
    numeric = {"robot": ("x", "y", "theta", "length", "width", "speed_in", "v_in", "yaw_dps", "accel_g"),
               "odom": ("x", "y", "theta", "error_in"), "cmd": ("fwd", "turn"),
               "battery": ("volts", "amps", "watts", "soc"), "power": ("slip", "rolling"),
               "game": ("time_left", "held", "capacity", "gate", "exit_in", "exit_speed_in")}
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or not _finite(frame.get("t")) or frame["t"] < 0 or frame["t"] < previous:
            raise ValueError(f"Frame {index} must have nonnegative, ordered native time")
        previous = frame["t"]
        for group, keys in numeric.items():
            if not isinstance(frame.get(group), dict) or not all(_finite(frame[group].get(key)) for key in keys):
                raise ValueError(f"Frame {index} has incomplete/nonfinite native {group} data")
        body = frame["robot"]["length"], frame["robot"]["width"]
        if min(body) <= 0 or (dimensions is not None and body != dimensions):
            raise ValueError("A recording must retain one positive physical chassis size")
        dimensions = body
        for key in ("blocks", "motors", "wheels", "warnings"):
            if not isinstance(frame.get(key), list):
                raise ValueError(f"Frame {index} requires native {key}")
        if frame.get("alliance") not in ("red", "blue") or not isinstance(frame.get("preset"), str):
            raise ValueError(f"Frame {index} requires native alliance and preset")
        if not isinstance(frame.get("metro"), dict):
            raise ValueError(f"Frame {index} requires Metro metadata")
        game = frame["game"]
        if game.get("period") not in ("pre", "auton", "driver", "over"):
            raise ValueError(f"Frame {index} has unsupported game period")
        if not isinstance(game.get("in_range"), list) or not isinstance(game.get("goals"), dict):
            raise ValueError(f"Frame {index} requires native goals and in_range")
        for colour in ("red", "blue"):
            if not isinstance(game.get(colour), dict) or not all(_finite(game[colour].get(key)) for key in ("total", "blocks")):
                raise ValueError(f"Frame {index} requires native {colour} score")
            if sum(block.get("c") == colour for block in frame["blocks"] if isinstance(block, dict)) > 128:
                raise ValueError("Recording exceeds native block instance capacity")
        for block in frame["blocks"]:
            if not isinstance(block, dict) or block.get("c") not in ("red", "blue") or not all(_finite(block.get(key)) for key in ("x", "y", "z", "yaw")):
                raise ValueError(f"Frame {index} contains invalid native Block pose")
            if not isinstance(block.get("q"), list) or len(block["q"]) != 4 or not all(map(_finite, block["q"])):
                raise ValueError(f"Frame {index} requires recorded Block quaternions")
        for group, keys in (("motors", ("rpm", "amps", "temp")), ("wheels", ("slip", "fz", "fx"))):
            for item in frame[group]:
                if not isinstance(item, dict) or not isinstance(item.get("name"), str) or not all(_finite(item.get(key)) for key in keys):
                    raise ValueError(f"Frame {index} contains incomplete native {group}")
    # Includes nonfinite values nested in telemetry/metadata, not just HUD keys.
    json.dumps(recording, allow_nan=False)
    return recording


def load_recording(path):
    path = Path(path)
    if path.stat().st_size > MAX_RECORDING_BYTES:
        raise ValueError("Recording exceeds the 256 MiB viewer limit")
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result
    def constant(value):
        raise ValueError(f"Nonfinite JSON constant: {value}")
    return validate_recording(json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=pairs,
                                         parse_constant=constant))


_INPUT = """
// Local camera controls only. No drivetrain/game action is sent anywhere.
const VIEW_KEYS = {'1':'operator', '2':'audience', '3':'overhead', '4':'free'};
let view = 'overhead';
let hingeMarker = false;
addEventListener('keydown', event => {
  if (/INPUT|SELECT|TEXTAREA/.test(event.target.tagName)) return;
  if (VIEW_KEYS[event.key]) { event.preventDefault(); setView(VIEW_KEYS[event.key]); }
});
"""

# Selection is independently exercised by Node tests. The slider uses indices,
# so duplicated timestamps and a float-rounded last millisecond stay reachable.
_SELECTION = """
function recordedIndex(frames, time) {
  let low = 0, high = frames.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if (frames[middle].t <= time + 1e-9) low = middle + 1; else high = middle;
  }
  return Math.max(0, low - 1);
}
function escapeHud(value) {
  if (typeof value === 'string') return value.replace(/[&<>"']/g, char =>
    ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'}[char]));
  if (Array.isArray(value)) return value.map(escapeHud);
  if (value && typeof value === 'object') return Object.fromEntries(
    Object.entries(value).map(([key, item]) => [escapeHud(key), escapeHud(item)]));
  return value;
}
"""

_SCRAPER = """
// The scraper rig, CAD part list, hinge, 300 ms smoothstep and placeScraper()
// now live in the native vexsim page and are used from there unchanged. Only
// the recording-to-keys step is replay-specific, so only that is added here.
function indexScraper(data) {
  scraperKeys = [];
  const initial = data.frames[0]?.metro?.outputs?.scraper === true ? 1 : 0;
  if (data.frames[0]) {
    // placeScraper() reads 0 before its first key. Seat the recording's own
    // starting position as a key just ahead of the first frame so a run that
    // begins with the scraper already down draws it down.
    scraperKeys.push({t:data.frames[0].t - scraperDuration, from:initial, to:initial});
  }
  const events = (data.events || []).filter(e => e.type === 'pneumatic_output' &&
    e.name === 'scraper' && Number.isFinite(e.t) && typeof e.value === 'boolean');
  // Older recordings can supply frame states without the event log.
  const samples = events.length ? events : data.frames.map(f =>
    ({t:f.t, value:f.metro?.outputs?.scraper}));
  let target = initial;
  for (const event of [...samples].sort((a,b) => a.t-b.t)) {
    if (typeof event.value !== 'boolean' || Number(event.value) === target) continue;
    const previous = scraperKeys.at(-1);
    const from = previous ? scraperValue(previous, event.t) : initial;
    target = Number(event.value);
    scraperKeys.push({t:event.t, from, to:target});
  }
}
"""

_PLAYBACK = _SELECTION + _SCRAPER + """
let recording = null, selectedIndex = 0, replayPlaying = false, replayTime = 0, replayLast = null;
const assetsReady = {robot:false, field:false, block:false};
const fieldCadStats = {loaders:0, parkZones:0, embeddedBlocks:0};
let fatalError = '';
const wheelBases = new WeakMap();
// Amber ghost: the pose Metro's own C++ odometry reported for this frame
// (recording odom.x/y/theta, world frame). The CAD robot is simulated truth.
// Clone the loaded CAD hierarchy with independent amber materials. Geometry is
// shared read-only; the ghost follows recorded transforms, never physics.
const ghost = new THREE.Group();
const ghostParts = [];
function buildGhost() {
  if (ghost.children.length || !body) return;
  const sources = [], copies = [];
  for (const source of robot.children) {
    const copy = source.clone(true);
    source.traverse(part => sources.push(part));
    copy.traverse(part => copies.push(part));
    ghost.add(copy);
  }
  const glow = new THREE.MeshBasicMaterial({color:0xf0b429, transparent:true,
    opacity:.28, depthWrite:false});
  copies.forEach((part, index) => {
    if (part.isMesh) {
      part.material = glow;
      part.castShadow = part.receiveShadow = false;
    }
    ghostParts.push([sources[index], part]);
  });
}
ghost.visible = false;
scene.add(ghost);
const ghostLink = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
  new THREE.LineBasicMaterial({color:0xf0b429, transparent:true, opacity:.9}));
ghostLink.visible = false;
scene.add(ghostLink);
let ghostEnabled = true;
function placeGhost(frame) {
  const odom = frame.odom || {};
  const ok = ['x','y','theta'].every(key => Number.isFinite(odom[key]));
  ghost.visible = ghostLink.visible = ghostEnabled && ok && assetsReady.robot;
  if (!ok) return false;
  for (const [source, copy] of ghostParts) {
    copy.position.copy(source.position);
    copy.quaternion.copy(source.quaternion);
    copy.scale.copy(source.scale);
    copy.visible = source.visible;
  }
  ghost.position.copy(P(odom.x, odom.y, 0));
  ghost.rotation.y = odom.theta + modelYaw;
  const points = ghostLink.geometry.attributes.position;
  points.setXYZ(0, frame.robot.x, .2, -frame.robot.y);
  points.setXYZ(1, odom.x, .2, -odom.y);
  points.needsUpdate = true;
  ghostLink.geometry.computeBoundingSphere();
  return true;
}
function replayFailure(message) {
  fatalError = message; replayPlaying = false;
  el('loadmsg').textContent = message;
  el('loading').style.display = 'grid';
  el('replay-status').textContent = message;
  el('replay-play').disabled = true;
  el('replay-restart').disabled = true;
}
function markAsset(name) {
  assetsReady[name] = true;
  if (name === 'robot') buildGhost();
  if (fatalError) return;
  if (Object.values(assetsReady).every(Boolean)) {
    el('loading').style.display = 'none';
    el('replay-play').disabled = false;
    el('replay-restart').disabled = false;
  }
  if (recording) selectFrame(selectedIndex);
}
function recordedWheels(frame) {
  const angles = frame.metro?.wheel_angles_rad;
  const available = angles && ['left','right'].every(side => Number.isFinite(angles[side]));
  for (const wheel of spinners.drive) {
    if (!wheelBases.has(wheel.pivot)) wheelBases.set(wheel.pivot, wheel.pivot.quaternion.clone());
    wheel.pivot.quaternion.copy(wheelBases.get(wheel.pivot));
    if (available) wheel.pivot.rotateOnAxis(wheel.axis, -wheel.sign * angles[wheel.side]);
  }
  return !!available;
}
function selectFrame(index) {
  if (!recording) return;
  selectedIndex = Math.max(0, Math.min(recording.frames.length - 1, Math.trunc(index)));
  state = recording.frames[selectedIndex];
  apply(state);
  const wheels = recordedWheels(state);
  setIndexer((state.game.gate || 0) * (state.game.gate_angle || 0.36));
  placeScraper(state.t);
  robot.visible = assetsReady.robot;
  const ghostShown = placeGhost(state);
  for (const mesh of Object.values(swarm)) mesh.visible = assetsReady.block;
  el('replay-timeline').value = selectedIndex;
  el('replay-time').textContent = state.t.toFixed(3) + ' / ' + recording.frames.at(-1).t.toFixed(3) + ' s';
  el('replay-status').textContent = (fatalError || (replayPlaying ? 'Playing exact recorded frames' : 'Paused recorded frame')) +
    ' · ' + (selectedIndex + 1) + '/' + recording.frames.length +
    (wheels ? ' · recorded wheel angles' : ' · wheel angles unavailable (not extrapolated)') +
    ' · roller rotation unavailable (not extrapolated)';
  el('metro-frame').textContent = JSON.stringify(state.metro, null, 2);
  const mt = state.metro.telemetry || {}, local = mt.local_pose_in_deg;
  const number = value => Number.isFinite(value) ? value.toFixed(2) : 'unavailable';
  el('metro-metrics').textContent = [
    'Metro frame: +X right, +Y forward, CW°',
    'Truth XY in: ' + number(-state.robot.y / .0254) + ', ' + number(state.robot.x / .0254),
    'Truth CW°: ' + number(-state.robot.theta * 180 / Math.PI),
    'Odom XY in (ghost): ' + (ghostShown ? number(-state.odom.y / .0254) + ', ' + number(state.odom.x / .0254) : 'unavailable'),
    'Odom CW°: ' + (ghostShown ? number(-state.odom.theta * 180 / Math.PI) : 'unavailable'),
    'Odom error: ' + number(state.odom.error_in) + ' in',
    'Local odom: ' + (Array.isArray(local) ? local.map(number).join(', ') : 'unavailable'),
    'Heading target: ' + number(mt.heading_target_deg) + '°',
    'Intake: ' + (mt.intake_state || 'unavailable'),
    'Scraper command: ' + (state.metro.outputs?.scraper === true ? 'DOWN' : state.metro.outputs?.scraper === false ? 'UP' : 'unavailable'),
    'Scraper animation: ' + (scraperRig ? (scraperProgress * 90).toFixed(0) + '° (illustrative)' : scraperReason),
    'Scraper contact / inertia: not modeled',
    'Bottom / top: ' + (state.metro.outputs?.bottom ?? '?') + ' / ' + (state.metro.outputs?.top ?? '?'),
    'L / R volts: ' + number(state.cmd.left_volts) + ' / ' + number(state.cmd.right_volts),
    'Optical: ' + (mt.optical_proximity ?? '?'),
  ].join('\\n');
  el('replay-play').textContent = replayPlaying ? 'Pause' : 'Play';
  el('replay-timeline').dataset.frame = String(selectedIndex);
}
function setPlaying(value) {
  if (!recording) return;
  replayPlaying = value === true && !fatalError && Object.values(assetsReady).every(Boolean);
  if (replayPlaying && selectedIndex === recording.frames.length - 1) selectFrame(0);
  replayTime = state.t; replayLast = null;
  selectFrame(selectedIndex);
}
function advanceReplay(now) {
  if (!replayPlaying) { replayLast = null; return; }
  if (replayLast !== null) replayTime += Math.max(0, now - replayLast) / 1000 * Number(el('replay-speed').value);
  replayLast = now;
  const end = recording.frames.at(-1).t;
  const index = recordedIndex(recording.frames, Math.min(replayTime, end));
  if (index !== selectedIndex) selectFrame(index);
  if (replayTime >= end) setPlaying(false);
}
async function loadRecording() {
  const response = await fetch('/recording.json');
  if (!response.ok) throw new Error('Recording unavailable: HTTP ' + response.status);
  recording = await response.json();
  if (recording.schema !== 'metro-game-recording-v1' || !recording.frames?.length) throw new Error('Invalid game recording');
  indexScraper(recording);
  state = recording.frames[0];
  el('replay-title').textContent = recording.metadata.routine;
  el('replay-source').textContent = 'Source: ' + recording.metadata.source_commit + ' · passive recording, no live controls';
  el('replay-assumptions').textContent = JSON.stringify({metadata:recording.metadata, summary:recording.summary}, null, 2);
  el('replay-events').textContent = JSON.stringify(recording.events, null, 2);
  el('replay-timeline').max = recording.frames.length - 1;
  el('replay-timeline').disabled = false;
  el('replay-play').addEventListener('click', () => setPlaying(!replayPlaying));
  el('replay-restart').addEventListener('click', () => { setPlaying(false); selectFrame(0); setPlaying(true); });
  el('replay-timeline').addEventListener('input', event => { setPlaying(false); selectFrame(Number(event.target.value)); });
  el('replay-camera').addEventListener('change', event => setView(event.target.value));
  el('replay-telemetry').addEventListener('click', () => { el('tele').hidden = !el('tele').hidden; });
  el('replay-ghost').addEventListener('click', () => {
    ghostEnabled = !ghostEnabled;
    el('replay-ghost').setAttribute('aria-pressed', String(ghostEnabled));
    selectFrame(selectedIndex);
  });
  addEventListener('visibilitychange', () => { if (document.hidden) setPlaying(false); });
  // Read-only diagnostic projection; seeking only chooses an existing frame.
  window.metroReplay = Object.freeze({
    seek: index => { setPlaying(false); selectFrame(index); },
    snapshot: () => ({index:selectedIndex, t:state.t, playing:replayPlaying,
      robot:{x:robot.position.x, y:-robot.position.z, theta:robot.rotation.y-modelYaw},
      ghost:{visible:ghost.visible, x:ghost.position.x, y:-ghost.position.z, theta:ghost.rotation.y-modelYaw},
      scraper:{available:!!scraperRig, progress:scraperProgress, angle_deg:scraperProgress*90,
        duration_s:scraperDuration, visual_only:true, reason:scraperReason,
        hinge_in:scraperHinge?.toArray().map(v => v/.0254),
        parts:scraperRig?.children.map(part => part.name) || []},
      blocks:structuredClone(state.blocks), game:structuredClone(state.game),
      metro:structuredClone(state.metro), assets:{...assetsReady},
      fieldGeometry:{...fieldCadStats, simplifiedPropsVisible:builtProps.visible}, error:fatalError}),
  });
}
"""

_CONTROLS = """
<div class="panel" id="replay-info">
  <strong id="replay-title">Metro game recording</strong>
  <div id="replay-source">Loading source metadata…</div>
  <div class="warn">Simulation approximation — not certified scoring or a hardware prediction.</div>
  <pre id="metro-metrics"></pre>
  <div class="dim">Amber translucent robot = estimated pose Metro's C++ odometry reported (what the routine steers by). CAD robot = simulated truth. The line joins the two centres.</div>
  <details><summary>Source, assumptions &amp; summary</summary><pre id="replay-assumptions"></pre></details>
  <details><summary>Recorded Metro outputs &amp; fidelity</summary><pre id="metro-frame"></pre></details>
  <details><summary>Recorded events</summary><pre id="replay-events"></pre></details>
  <details><summary>Physics limitations &amp; warnings</summary><div id="warn"></div></details>
  <details><summary>Native CAD status</summary><div id="status"></div></details>
  <div class="dim">Official CAD goals, loaders, park zones and 18-sided Blocks; drawn perimeter and tiles. Competition block layout follows Push Back Appendix A. Intake routes approximate this robot; scraper/wing collisions and intake electrical load are not modeled.</div>
</div>
<div class="panel" id="replay-controls">
  <div class="replay-row"><button id="replay-play" disabled>Play</button>
    <button id="replay-restart" disabled title="Return to the first recorded frame and play">Restart</button>
    <input id="replay-timeline" type="range" min="0" max="1" step="1" value="0" disabled aria-label="Recorded frame">
    <select id="replay-speed" aria-label="Playback speed"><option value="0.25">¼×</option><option value="0.5">½×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
    <output id="replay-time">0 s</output></div>
  <div class="replay-row"><select id="replay-camera" aria-label="Native camera"><option value="overhead">3 · Overhead</option><option value="operator">1 · Operator</option><option value="audience">2 · Audience</option><option value="free">4 · Free orbit</option></select>
    <button id="replay-telemetry">Toggle native telemetry</button>
    <button id="replay-ghost" aria-pressed="true" title="Amber translucent robot at the pose Metro's odometry reported; the CAD robot is simulated truth">Toggle odometry ghost</button><span id="replay-status" role="status">Loading native CAD…</span></div>
</div>
"""


def transform(source):
    """Checked edits to the native renderer's transport only; fail on drift."""
    sections = {}
    for name, pattern in (("css", r"<style>(.*?)</style>"),
                          ("imports", r'<script type="importmap">(.*?)</script>'),
                          ("js", r'<script type="module">(.*?)</script>')):
        found = re.findall(pattern, source, re.S)
        if len(found) != 1:
            raise NativeViewerError(f"Expected one native {name} section")
        sections[name] = found[0]
    expected = {"imports": {"three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
                            "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}
    if json.loads(sections["imports"]) != expected:
        raise NativeViewerError("Unsupported native Three.js import map")
    js = sections["js"]
    for token, value in (("__HAS_MODEL__", "true"), ("__HAS_FIELD__", "true"), ("__HAS_BLOCK__", "true"),
                         ("__MODEL_UP__", "z"), ("__MODEL_FORWARD__", "+y")):
        js = _replace_once(js, token, value)
    js = _replace_region(js, "// ------------------------------------------------------------------- input",
                         "// ------------------------------------------------------------------ camera", _INPUT)
    js = _replace_region(js, "async function poll() {", "let lastHud = 0;", "")
    js = _replace_once(js, "  if (body === null) {\n    body = placeholder(s.robot.length, s.robot.width);\n    robot.add(body);\n  }", "  // Only the actual loaded CAD body is drawn; never fabricate a chassis.")
    js = _replace_once(js, "  const now = performance.now();\n  if (now - lastHud > 100) {\n    lastHud = now;\n    hud(s);\n  }", "  hud(escapeHud(s)); // Scrub updates the native HUD at this exact frame.")
    js = _replace_once(js, "(s.paused ? '<div class=\"warn\">PAUSED</div>' : '')", "(!replayPlaying ? '<div class=\"warn\">REPLAY PAUSED</div>' : '')")
    js = _replace_once(js, "// ------------------------------------------------------------------ startup", _PLAYBACK + "\n// ------------------------------------------------------------------ startup")
    js = _replace_once(js, "await poll();\nsetView('operator');\ndocument.getElementById('loading').style.display = 'none';", "try { await loadRecording(); } catch (error) { replayFailure(error.message); throw error; }\nsetView('overhead');")
    js = _replace_once(js, "    if (!found) return;", "    if (!found) { replayFailure('Block CAD contains no mesh'); return; }")
    js = _replace_once(js, "    modelStatus = 'Block model loaded';", "    modelStatus = 'Block model loaded'; markAsset('block');")
    js = _replace_once(js, "modelStatus = 'Block model failed, drawing prisms';", "replayFailure('Block CAD failed to load; replay unavailable');")
    js = _replace_once(js, "    applyDetail();\n  }, xhr => {", "    applyDetail(); markAsset('field');\n  }, xhr => {")
    js = _replace_once(js, "modelStatus = 'field model failed to load, drawing our own';", "replayFailure('Field CAD failed to load; replay unavailable');")
    js = _replace_once(js, "if (/^276-9142-(300|400)/.test(c.name)) {", "if (false) { // Preserve official CAD Loaders and Park Zones.")
    js = _replace_once(js, "    const box = new THREE.Box3();\n    // the printed centre tiles", "    const embeddedBlocks = [];\n    obj.traverse(o => { if (/Instruction/.test(o.name)) embeddedBlocks.push(o); });\n    for (const block of embeddedBlocks) block.removeFromParent(); // Includes blocks inside the loader assemblies.\n    const box = new THREE.Box3();\n    // the printed centre tiles")
    js = _replace_once(js, "    const fcut = lighten(obj, FIELD_MESH, CAD_GOAL);", "    fieldCadStats.loaders = obj.children.filter(o => /^276-9142-400/.test(o.name)).length;\n    fieldCadStats.parkZones = obj.children.filter(o => /^276-9142-300/.test(o.name)).length;\n    obj.traverse(o => { if (/Instruction/.test(o.name)) fieldCadStats.embeddedBlocks++; });\n    const fcut = lighten(obj, FIELD_MESH, CAD_GOAL);")
    js = _replace_once(js, "const fcut = lighten(obj, FIELD_MESH, CAD_GOAL);", "const fcut = lighten(obj, FIELD_MESH, /^276-9142-(10[0-4]|20[0-3]|300|400)/);")
    js = _replace_once(js, "  builtGoals.visible = detail === 'built';", "  builtGoals.visible = detail === 'built';\n  builtProps.visible = detail === 'built'; // CAD replaces simplified loaders and park pads.")
    js = _replace_once(js, "    dressField(obj);", "    dressField(obj);\n    for (const zone of obj.children.filter(o => /^276-9142-300_Red/.test(o.name))) {\n      zone.traverse(o => {\n        if (!o.isMesh) return;\n        const recolor = material => {\n          if (material?.color?.getHex() !== 0x00a3e0) return material;\n          const red = material.clone(); red.color.setHex(0xd92b36); return red;\n        };\n        o.material = Array.isArray(o.material) ? o.material.map(recolor) : recolor(o.material);\n      });\n    }")
    js = _replace_once(js, "    const s = state || { robot: { length: 0.381, width: 0.3175 } };", "    const s = state; // Validated recording supplies the physical chassis size.")
    js = _replace_once(js, "      `${got.indexer}-part indexer · ${modelDetail}`;", "      `${got.indexer}-part indexer · ${modelDetail}`;\n    markAsset('robot');")
    js = _replace_once(js, "modelStatus = 'model failed to load, drawing a box';", "replayFailure('Robot CAD failed to load; replay unavailable');")
    js = _replace_region(js, "function turnWheels(dt) {", "// Keep the frame rate", "// Wheel and roller RPM are never integrated by the viewer.\n\n")
    # The live page drives the scraper and the wing off the interpolated poll
    # clock; replay drives both by frame, so the whole block goes.
    js = _replace_once(js, "  turnWheels(dt);\n  if (clockOffset !== null) {\n"
                           "    const at = now / 1000 + clockOffset - RENDER_DELAY;\n"
                           "    placeScraper(at);\n    placeWing(at);\n  }",
                       "  advanceReplay(now); // Clock selects recorded samples only; no physics integration.")
    js = _replace_once(js, "\nframe();", "\nselectFrame(0);\nframe();")
    if re.search(r"setInterval\s*\(|fetch\(['\"]/(state|input)['\"]|ACTION_KEYS|turnWheels", js):
        raise NativeViewerError("Active native drive transport remains in replay")
    mapping = json.dumps({"imports": {"three": "/three/build/three.module.js", "three/addons/": "/three/examples/jsm/"}}, separators=(",", ":"))
    html = _replace_once(source, "<style>" + sections["css"] + "</style>", '<link rel="stylesheet" href="/viewer.css">')
    html = _replace_once(html, '<script type="importmap">' + sections["imports"] + '</script>', '<script type="importmap">' + mapping + '</script>')
    html = _replace_once(html, '<script type="module">' + sections["js"] + '</script>', '<script type="module" src="/viewer.js"></script>')
    html = _replace_once(html, '<div id="warn"></div>', '')
    html = _replace_once(html, '<div class="panel" id="status"></div>', '')
    html = _replace_region(html, '<div class="panel" id="keys">', '<script type="importmap">', _CONTROLS + '\n')
    html = _replace_once(html, "<title>Push Back — vexsim</title>", "<title>Metro recording — native vexsim Push Back</title>")
    css = sections["css"] + """
[hidden] { display:none !important; }
#replay-info { top:12px; left:12px; width:300px; max-height:calc(100vh - 160px); overflow:auto; }
#replay-info pre { white-space:pre-wrap; overflow-wrap:anywhere; font-size:11px; }
#replay-source { overflow-wrap:anywhere; }
#tele { max-height:calc(100vh - 160px); overflow:auto; }
#status,#warn { position:static; width:auto; max-width:100%; overflow-wrap:anywhere; }
#score { max-width:calc(100vw - 710px); min-width:260px; }
#replay-controls { bottom:12px; left:12px; right:12px; }
.replay-row { display:flex; align-items:center; gap:10px; flex-wrap:wrap; }
.replay-row + .replay-row { margin-top:6px; }
#replay-timeline { flex:1; min-width:100px; }
button,select { color:#d8dee9; background:#202735; border:1px solid #526075; padding:6px; font:inherit; }
button:disabled { opacity:.5; }
#replay-status { font-size:11px; }
@media(max-width:800px) {
  #replay-info { width:calc(100% - 24px); max-height:20vh; }
  #score { top:23vh; min-width:220px; }
  #tele { top:43vh; width:250px; max-height:28vh; }
  #status,#warn { display:none; }
  #replay-status { width:100%; }
}
"""
    digest = base64.b64encode(hashlib.sha256(mapping.encode()).digest()).decode()
    # Native HUD uses inline width styles for its battery/speed bars. Scripts
    # still require the exact import-map hash or local module; no inline JS.
    csp = ("default-src 'none'; script-src 'self' 'sha256-" + digest + "'; style-src 'self' 'unsafe-inline'; "
           "connect-src 'self'; img-src 'self' data: blob:; frame-ancestors 'none'; base-uri 'none'; object-src 'none'")
    return {"html": html.encode(), "js": js.encode(), "css": css.encode(), "csp": csp}


class ReplayAssets:
    """Fixed recording and allowlisted installed source assets, no directory API."""
    def __init__(self, recording, vexsim=ROOT.parent / "vexsim"):
        self.root = Path(vexsim).resolve()
        self.recording = json.dumps(validate_recording(recording), allow_nan=False, separators=(",", ":")).encode()
        if json.loads(_read(self.root, "node_modules/three/package.json"))["version"] != "0.160.0":
            raise NativeViewerError("Native replay requires installed Three.js 0.160.0")
        for relative in MODEL_FILES.values():
            target = (self.root / relative).resolve()
            try:
                target.relative_to(self.root)
            except ValueError as error:
                raise NativeViewerError(f"Native CAD resolves outside simulator: {relative}") from error
            if not target.is_file():
                raise NativeViewerError(f"Required native CAD unavailable: {relative}")
        self.bundle = transform(_read(self.root, "vexsim/web/push_back.html").decode())

    def asset(self, path):
        routes = {"/": ("html", "text/html; charset=utf-8"), "/viewer.js": ("js", "text/javascript; charset=utf-8"),
                  "/viewer.css": ("css", "text/css; charset=utf-8")}
        if path in routes:
            key, kind = routes[path]
            return self.bundle[key], kind, self.bundle["csp"] if key == "html" else None
        if path == "/recording.json":
            return self.recording, "application/json; charset=utf-8", None
        if path == "/field":
            return _field(self.root), "application/json; charset=utf-8", None
        if path in MODEL_FILES:
            return _read(self.root, MODEL_FILES[path]), "application/octet-stream", None
        three = {"/three/" + relative: relative for relative in THREE_FILES}
        if path in three:
            return _read(self.root, "node_modules/three/" + three[path]), "text/javascript; charset=utf-8", None
        return None


class ReplayHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _send(self, status, data, kind="text/plain; charset=utf-8", csp=None):
        self.send_response(status)
        for key, value in {"Content-Type": kind, "Content-Length": str(len(data)), "Cache-Control": "no-store",
                           "X-Content-Type-Options": "nosniff", "Referrer-Policy": "no-referrer",
                           "Content-Security-Policy": csp or "default-src 'none'; frame-ancestors 'none'",
                           "Connection": "close"}.items():
            self.send_header(key, value)
        self.end_headers()
        self.close_connection = True
        if self.command != "HEAD":
            self.wfile.write(data)

    def _local(self):
        hosts = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        received = self.headers.get_all("Host", [])
        origins = self.headers.get_all("Origin", [])
        if len(received) != 1 or received[0].lower() not in hosts or len(origins) > 1 or (origins and origins[0] not in {"http://" + host for host in hosts}):
            self._send(403, b"Only local same-origin requests are accepted")
            return False
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
            self._send(400, b"Only exact relative asset paths are accepted")
            return False
        return True

    def do_GET(self):
        if not self._local():
            return
        try:
            value = self.server.assets.asset(self.path)
            if value is None:
                self._send(404, b"No such replay asset")
            else:
                self._send(200, *value)
        except (NativeViewerError, OSError, ValueError) as error:
            self._send(503, str(error).encode())

    do_HEAD = do_GET

    def do_POST(self):
        if self._local():
            self._send(405, b"Passive replay has no input or simulation endpoints")

    do_PUT = do_DELETE = do_PATCH = do_OPTIONS = do_POST


class ReplayServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, port, assets):
        self.assets = assets
        super().__init__(("127.0.0.1", port), ReplayHandler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--vexsim", type=Path, default=ROOT.parent / "vexsim")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        assets = ReplayAssets(load_recording(args.recording), args.vexsim)
    except (OSError, ValueError, NativeViewerError) as error:
        parser.error(str(error))
    with ReplayServer(args.port, assets) as server:
        print(f"Passive native game replay: http://127.0.0.1:{args.port}\nRecording: {args.recording.resolve()}", flush=True)
        try:
            server.serve_forever(poll_interval=.2)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
