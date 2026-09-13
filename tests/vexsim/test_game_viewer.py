#!/usr/bin/env python3
"""Passive native game replay contracts; no simulation or server is started."""
import copy
from email.message import Message
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.dont_write_bytecode = True
from game_viewer import (MODEL_FILES, ROOT, SCHEMA, ReplayAssets, ReplayHandler,
                         NativeViewerError, THREE_FILES, _PLAYBACK, _SCRAPER, load_recording,
                         transform, validate_recording)

VEXSIM = Path(os.environ.get("VEXSIM_PATH", ROOT.parent / "vexsim")).resolve()


def fixture():
    """Synthetic native snapshots for UI-only tests, not physical evidence."""
    frame = {"t": 0, "paused": False, "preset": "fixture", "alliance": "red",
             "robot": {"x": .2, "y": -.3, "theta": 1.2, "length": .381, "width": .3175,
                       "speed_in": 1, "v_in": 0, "yaw_dps": 5, "accel_g": .1},
             "odom": {"x": .1, "y": -.05, "theta": .01, "error_in": .2}, "cmd": {"fwd": 1, "turn": -2, "stop": ""},
             "battery": {"volts": 12, "amps": 1, "watts": 12, "soc": 1},
             "power": {"slip": 1, "rolling": 2},
             "motors": [{"name": "left", "rpm": 10, "amps": 1, "temp": 22}],
             "wheels": [{"name": "left", "slip": .1, "fz": 1, "fx": -1}],
             "warnings": ["Fixture only"],
             "blocks": [{"c": "red", "s": "field", "x": .5, "y": -.6, "z": .07,
                         "yaw": .4, "q": [0, 0, .5, .8660254]}],
             "game": {"period": "auton", "time_left": 15, "held": 0, "capacity": 3,
                      "gate": .5, "gate_angle": .36, "route": "center", "exit_in": 4,
                      "exit_speed_in": 80, "in_range": [], "goals": {},
                      "red": {"total": 0, "blocks": 0}, "blue": {"total": 0, "blocks": 0}},
             "metro": {"wheel_angles_rad": {"left": 1, "right": -2}, "outputs": {"hood": False}}}
    second = copy.deepcopy(frame)
    second["t"] = 3.569999999999718
    second["robot"].update(x=-.8, y=.9, theta=-2)
    second["blocks"][0].update(x=-.4, z=.3, q=[.5, 0, 0, .8660254])
    second["metro"]["wheel_angles_rad"] = {"left": 5, "right": 7}
    return {"schema": SCHEMA, "metadata": {"routine": "UI fixture", "source_commit": "fixture-only",
                                           "assumptions": ["Not a simulation result"]},
            "frames": [frame, second], "events": [], "summary": {"fixture": True}}


class GameViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (VEXSIM / "vexsim/web/push_back.html").read_text()
        cls.bundle = transform(cls.source)
        cls.assets = ReplayAssets(fixture(), VEXSIM)

    def test_recording_validation_preserves_exact_data(self):
        data = fixture()
        before = copy.deepcopy(data)
        self.assertIs(validate_recording(data), data)
        self.assertEqual(data, before)
        self.assertEqual(json.loads(self.assets.recording), before)
        del data["frames"][0]["metro"]["wheel_angles_rad"]
        self.assertIs(validate_recording(data), data, "Missing angles are unavailable, not inferred from RPM")

    def test_invalid_recordings_are_rejected_before_rendering(self):
        for change in (
            lambda d: d.update(schema="wrong"), lambda d: d.update(frames=[]),
            lambda d: d["metadata"].pop("source_commit"), lambda d: d["metadata"].pop("assumptions"),
            lambda d: d["frames"][1].update(t=-1), lambda d: d["frames"][0]["robot"].update(x=float("nan")),
            lambda d: d["frames"][1]["robot"].update(length=.4),
            lambda d: d["frames"][0]["blocks"][0].pop("q"),
            lambda d: d["frames"][0]["blocks"][0].update(q=[0, 0, 1]),
            lambda d: d["frames"][0].update(blocks=d["frames"][0]["blocks"] * 129),
            lambda d: d["frames"][0]["metro"].update(invalid=float("inf")),
        ):
            data = fixture(); change(data)
            with self.subTest(data=data["schema"]), self.assertRaises(ValueError):
                validate_recording(data)

    def test_strict_json_file_loading(self):
        with tempfile.TemporaryDirectory(prefix="mclib-game-json-") as directory:
            path = Path(directory) / "recording.json"
            path.write_text(json.dumps(fixture()))
            self.assertEqual(load_recording(path), fixture())
            for text in ('{"schema":"a","schema":"b"}', '{"t":NaN}'):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    load_recording(path)

    def test_actual_cad_and_native_pose_application_are_retained(self):
        js = self.bundle["js"].decode()
        for start, end in (("function fitModel(", "// ------------------------------------------------------------------- input"),
                           ("function collectSpinners(", "// The stage the solver"),
                           ("// blocks\n", "  // The readout")):
            if start not in self.source:
                self.fail(f"Missing native test anchor {start}")
            excerpt = self.source[self.source.index(start):self.source.index(end, self.source.index(start))]
            self.assertIn(excerpt, js)
        for name in ("MODEL", "FIELD", "BLOCK"):
            self.assertIn(f"const HAS_{name} = true;", js)
        for path in MODEL_FILES:
            self.assertIn("load('" + path + "'", js)
        self.assertNotIn("body = placeholder(", js)
        self.assertNotIn("drawing a box", js)
        self.assertNotIn("drawing prisms", js)
        self.assertIn("replayFailure('Robot CAD failed", js)
        self.assertIn("replayFailure('Field CAD failed", js)
        self.assertIn("replayFailure('Block CAD failed", js)
        self.assertIn("hud(escapeHud(s));", js)

    def test_no_live_transport_or_visual_motion_integration(self):
        js = self.bundle["js"].decode()
        for value in ("setInterval", "ACTION_KEYS", "turnWheels", "fetch('/state')", "fetch('/input'", "rpmToRad"):
            self.assertNotIn(value, js)
        self.assertEqual(js.count("fetch('/recording.json')"), 1)
        self.assertIn("wheel.pivot.quaternion.copy", js)
        self.assertIn("angles[wheel.side]", js)
        self.assertIn("advanceReplay(now)", js)
        self.assertNotIn("await poll", js)

    def test_local_imports_native_hud_and_visible_fidelity(self):
        html, css = self.bundle["html"].decode(), self.bundle["css"].decode()
        self.assertEqual(html.count('<script type="importmap">'), 1)
        self.assertNotIn("cdn.jsdelivr", html)
        self.assertIn('src="/viewer.js"', html)
        for name in ("score", "tele", "motors", "wheels", "goals", "replay-assumptions", "metro-frame", "replay-events"):
            self.assertIn(f'id="{name}"', html)
        self.assertIn("not certified scoring", html)
        self.assertIn("scraper/wing", html)
        self.assertIn("18-sided Blocks", html)
        self.assertIn("overflow:auto", css)
        self.assertIn("frame-ancestors 'none'", self.bundle["csp"])
        script_policy = self.bundle["csp"].split("script-src", 1)[1].split(";", 1)[0]
        self.assertNotIn("unsafe-inline", script_policy)
        self.assertIn("'sha256-", script_policy)

    def test_native_source_changes_fail_closed(self):
        for old, replacement in (("const HAS_MODEL = __HAS_MODEL__;", "const HAS_MODEL = false;"),
                                  ("  turnWheels(dt);", "  turnWheels(2 * dt);"),
                                  ("await poll();", "await differentPoll();"),
                                  ("three@0.160.0", "three@0.161.0")):
            with self.subTest(old=old), self.assertRaises(NativeViewerError):
                transform(self.source.replace(old, replacement))

    def test_only_fixed_source_assets_are_served(self):
        for path in ("/", "/viewer.js", "/viewer.css", "/recording.json", "/field"):
            self.assertGreater(len(self.assets.asset(path)[0]), 100)
        for path, filename in MODEL_FILES.items():
            data, kind, _ = self.assets.asset(path)
            self.assertEqual(hashlib.sha256(data).digest(), hashlib.sha256((VEXSIM / filename).read_bytes()).digest())
            self.assertEqual(kind, "application/octet-stream")
        for relative in THREE_FILES:
            self.assertEqual(self.assets.asset("/three/" + relative)[0], (VEXSIM / "node_modules/three" / relative).read_bytes())
        for path in ("/state", "/input", "/api/runs", "/../README.md", "/%2e%2e/README.md", "/recording.json?x=1",
                     "/three/package.json", "/three/../package.json", "/etc/passwd", "/index.html"):
            with self.subTest(path=path):
                self.assertIsNone(self.assets.asset(path))

    def handler(self, path="/", host="127.0.0.1:8766", origin=None, command="GET"):
        handler = object.__new__(ReplayHandler)
        handler.path, handler.command = path, command
        handler.server = SimpleNamespace(server_port=8766, assets=self.assets)
        handler.headers = Message()
        if host is not None:
            handler.headers.add_header("Host", host)
        if origin is not None:
            handler.headers.add_header("Origin", origin)
        handler.sent = []
        handler._send = lambda *args: handler.sent.append(args)
        return handler

    def test_host_origin_and_fixed_route_guards_without_opening_a_port(self):
        for kwargs, status in (({}, 200), ({"path": "/recording.json"}, 200),
                               ({"path": "/input"}, 404), ({"path": "/state"}, 404),
                               ({"host": "evil.invalid:8766"}, 403), ({"host": None}, 403),
                               ({"origin": "http://evil.invalid"}, 403),
                               ({"path": "http://127.0.0.1:8766/"}, 400),
                               ({"path": "/?path=/etc/passwd"}, 400)):
            with self.subTest(kwargs=kwargs):
                handler = self.handler(**kwargs); handler.do_GET()
                self.assertEqual(handler.sent[-1][0], status)
        handler = self.handler(); handler.headers.add_header("Host", "127.0.0.1:8766"); handler.do_GET()
        self.assertEqual(handler.sent[-1][0], 403)
        handler = self.handler(path="/input", command="POST"); handler.do_POST()
        self.assertEqual(handler.sent[-1][0], 405)

    @unittest.skipUnless(shutil.which("node"), "Node is required for JavaScript contracts")
    def test_native_module_syntax(self):
        result = subprocess.run(["node", "--input-type=module", "--check"], input=self.bundle["js"].decode(), text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(shutil.which("node"), "Node and local Three.js are required")
    def test_scraper_rigid_hinge_and_reversible_recorded_timeline(self):
        span = self.source[self.source.index("function spanOf("):self.source.index("// ------------------------------------------------------------------ indexer")]
        script = "import * as THREE from " + json.dumps((VEXSIM / "node_modules/three/build/three.module.js").as_uri()) + ";\n"
        script += "import assert from 'node:assert/strict';\nconst robot=new THREE.Group(), root=new THREE.Group();robot.add(root);\n"
        # The rig, part list, hinge and placeScraper() are native to the
        # vexsim page now; _SCRAPER only adds the recording-to-keys step.
        native = self.source[self.source.index("// ------------------------------------------------------------------ scraper"):
                             self.source.index("// ------------------------------------------------------------------- input")]
        script += "const IN = 0.0254;\n"   # buildWing uses it; it is declared above the slice
        script += span + native + _SCRAPER + """
const shape=new THREE.BoxGeometry(.02,.04,.01),material=new THREE.MeshBasicMaterial();
for(const [i,name] of SCRAPER_PARTS.entries()) {
  const part=new THREE.Mesh(shape,material);part.name=name;
  part.position.set(.16+i*.001,.2,(i-2)*.04);root.add(part);
  assert(SCRAPER_KEEP.test(name));
}
for(const [i,name] of ['HS_Pillow_Bearing_Block_v18','HS_Pillow_Bearing_Block_v19'].entries()) {
  const bearing=new THREE.Mesh(shape,material);bearing.name=name;
  bearing.position.set(.15,.07,i===0?-.16:.16);root.add(bearing);
}
robot.position.set(2,0,-3);robot.rotation.y=.7;robot.updateMatrixWorld(true);
const before=SCRAPER_PARTS.map(name=>root.getObjectByName(name).getWorldPosition(new THREE.Vector3()));
buildScraper(root);robot.updateMatrixWorld(true);
assert.equal(scraperRig.children.length,6);
for(let i=0;i<6;i++)assert(scraperRig.children[i].getWorldPosition(new THREE.Vector3()).distanceTo(before[i])<1e-10);
assert(scraperHinge.distanceTo(new THREE.Vector3(.15,.07,0))<1e-10);
const hinge=scraperRig.getWorldPosition(new THREE.Vector3());
const d=scraperRig.children[0].getWorldPosition(new THREE.Vector3()).distanceTo(scraperRig.children[1].getWorldPosition(new THREE.Vector3()));
const data={frames:[{t:0,metro:{outputs:{scraper:false}}}],events:[
  {type:'pneumatic_output',name:'scraper',t:.5,value:true},
  {type:'pneumatic_output',name:'scraper',t:5.56,value:false}]};
const original=JSON.stringify(data);indexScraper(data);
for(const [t,expected] of [[0,0],[.5,0],[.65,.5],[.8,1],[5.56,1],[5.71,.5],[5.86,0],[.65,.5],[0,0]]) {
  placeScraper(t);robot.updateMatrixWorld(true);
  assert(Math.abs(scraperProgress-expected)<1e-10);
  assert(Math.abs(scraperRig.rotation.z+expected*Math.PI/2)<1e-10);
  assert(scraperRig.getWorldPosition(new THREE.Vector3()).distanceTo(hinge)<1e-10);
  assert(Math.abs(scraperRig.children[0].getWorldPosition(new THREE.Vector3()).distanceTo(scraperRig.children[1].getWorldPosition(new THREE.Vector3()))-d)<1e-10);
}
assert.equal(JSON.stringify(data),original);
// Reverse partway through deployment without jumping to a stroke endpoint.
data.events[1].t=.65;indexScraper(data);placeScraper(.65);assert(Math.abs(scraperProgress-.5)<1e-10);
placeScraper(.8);assert(Math.abs(scraperProgress-.25)<1e-10);
indexScraper({frames:[{t:0,metro:{outputs:{scraper:false}}},{t:1,metro:{outputs:{scraper:true}}}],events:[]});
placeScraper(1.15);assert(Math.abs(scraperProgress-.5)<1e-10);
console.log('PASS scraper hinge, rigid assembly, command timing, rewind, partial reversal and legacy frame fallback');
"""
        result = subprocess.run(["node", "--input-type=module"], input=script,
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(shutil.which("node"), "Node and local Three.js are required")
    def test_exact_frames_blocks_wheels_pause_and_scrub_on_native_scene(self):
        js = self.bundle["js"].decode()
        apply = js[js.index("let lastHud = 0;"):js.index("// ---------------------------------------------------------------------- HUD")]
        # The scraper rig and its state live in the native page above this
        # slice now, and _PLAYBACK's indexScraper writes to them.
        native_scraper = js[js.index("// ------------------------------------------------------------------ scraper"):
                            js.index("// ------------------------------------------------------------------ camera")]
        fixture_js = "import * as THREE from " + json.dumps((VEXSIM / "node_modules/three/build/three.module.js").as_uri()) + ";\n"
        fixture_js += "import assert from 'node:assert/strict';\nconst data=" + json.dumps(fixture()) + ";\n" + """
const P=(x,y,z=0)=>new THREE.Vector3(x,z,-y), robot=new THREE.Group(), scene=new THREE.Scene();
const builtProps=new THREE.Group();
const body=new THREE.Group(); body.add(new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshBasicMaterial())); robot.add(body);
const bank=new THREE.Object3D(), BLOCK_ROOM=128, swarm={};
for(const colour of ['red','blue']) swarm[colour]=new THREE.InstancedMesh(new THREE.BoxGeometry(),new THREE.MeshBasicMaterial(),128);
const spinners={drive:['left','right'].map(side=>({side,sign:side==='left'?1:-1,axis:new THREE.Vector3(0,1,0),pivot:new THREE.Group()}))};
const elements=new Map(), el=id=>{if(!elements.has(id))elements.set(id,{value:id==='replay-speed'?'1':'',style:{},dataset:{},addEventListener(type,fn){this[type]=fn;},setAttribute(key,value){this[key]=value;}});return elements.get(id);};
let state=null,modelYaw=0,gate=0,lastHudState=null;
function setIndexer(value){gate=value;}
function hud(value){lastHudState=value;}
function setView(){}
const window={},document={hidden:false},addEventListener=()=>{};
let gets=0; const fetch=async path=>{assert.equal(path,'/recording.json');gets++;return {ok:true,json:async()=>data};};
"""
        checks = """
await loadRecording(); selectFrame(0);
assert.equal(gets,1); assert.equal(robot.visible,false);
for(const name of ['robot','field','block'])markAsset(name);
assert.equal(robot.visible,true);assert.equal(el('replay-play').disabled,false);
assert.equal(ghost.visible,true);assert.equal(ghost.children.length,1);
assert.notEqual(ghost.children[0].children[0].material,body.children[0].material);
assert.equal(body.children[0].material.transparent,false);
assert.equal(ghost.children[0].children[0].material.transparent,true);
assert.equal(robot.position.x,data.frames[0].robot.x);
assert.equal(robot.position.z,-data.frames[0].robot.y);
assert.equal(robot.rotation.y,data.frames[0].robot.theta);
assert.equal(gate,.18);assert.deepEqual(lastHudState,data.frames[0]);
assert(el('metro-metrics').textContent.includes('Scraper command: '));
assert(el('metro-metrics').textContent.includes('Scraper contact / inertia: not modeled'));
const initial=spinners.drive.map(w=>w.pivot.quaternion.clone());
selectFrame(1);
assert.equal(state,data.frames[1]);
assert.equal(ghost.position.x,state.odom.x);assert.equal(ghost.position.z,-state.odom.y);
assert.equal(ghost.rotation.y,state.odom.theta+modelYaw);
el('replay-ghost').click();assert.equal(ghost.visible,false);assert.equal(el('replay-ghost')['aria-pressed'],'false');
el('replay-ghost').click();assert.equal(ghost.visible,true);
assert.equal(el('replay-timeline').value,1);
assert.equal(robot.position.x,-.8);assert.equal(robot.position.z,-.9);
const matrix=new THREE.Matrix4();swarm.red.getMatrixAt(0,matrix);
const position=new THREE.Vector3(),quaternion=new THREE.Quaternion(),scale=new THREE.Vector3();matrix.decompose(position,quaternion,scale);
assert(Math.abs(position.x+.4)<1e-7);assert(Math.abs(position.y-.3)<1e-7);assert(Math.abs(position.z-.6)<1e-7);
const expected=new THREE.Matrix4().compose(P(-.4,-.6,.3),new THREE.Quaternion(.5,0,0,.8660254),new THREE.Vector3(1,1,1));
for(let i=0;i<16;i++)assert(Math.abs(matrix.elements[i]-expected.elements[i])<1e-7);
selectFrame(0);spinners.drive.forEach((w,i)=>assert(w.pivot.quaternion.angleTo(initial[i])<1e-7));
selectFrame(1);const paused=robot.position.clone();advanceReplay(900000);assert(robot.position.equals(paused));
assert.equal(recordedIndex(data.frames,3.57),1);assert.equal(recordedIndex(data.frames,3.56),0);
assert.equal(recordedIndex([{t:0},{t:1},{t:1}],1),2);
setPlaying(true);assert.equal(selectedIndex,0);el('replay-speed').value='2';advanceReplay(1000);advanceReplay(2800);
assert.equal(selectedIndex,1);assert.equal(replayPlaying,false);assert.equal(gets,1);
for (const playing of [false,true]) {
  selectFrame(1);setPlaying(playing);selectFrame(1);
  el('replay-restart').click();
  assert.equal(selectedIndex,0);assert.equal(replayPlaying,true);assert.equal(replayTime,data.frames[0].t);
  assert.equal(replayLast,null);assert.equal(gets,1);
}
window.metroReplay.seek(0);assert.equal(window.metroReplay.snapshot().t,0);
const unsafe=escapeHud({warnings:['<img src=x onerror=alert(1)>'],game:{route:'"bad"'}});
assert.equal(unsafe.warnings[0],'&lt;img src=x onerror=alert(1)&gt;');
assert.equal(unsafe.game.route,'&quot;bad&quot;');
const before=JSON.stringify(data);selectFrame(1);selectFrame(0);assert.equal(JSON.stringify(data),before);
delete data.frames[0].metro.wheel_angles_rad;selectFrame(0);
assert(el('replay-status').textContent.includes('wheel angles unavailable'));
assert(spinners.drive.every(w=>w.pivot.quaternion.angleTo(new THREE.Quaternion())<1e-7));
replayFailure('CAD failed');setPlaying(true);assert.equal(replayPlaying,false);assert.equal(el('replay-play').disabled,true);
console.log('PASS exact native pose/quaternion application, recorded wheel angles, rewind, pause, sample selection and safe HUD');
"""
        result = subprocess.run(["node", "--input-type=module"], input=fixture_js + 'const IN = 0.0254;\n' + native_scraper + apply + _PLAYBACK + checks,
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
