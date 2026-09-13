#!/usr/bin/env python3
"""Native renderer transport, frame conversion, and exact asset boundaries."""
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
from native_viewer import NativeViewerError, THREE_FILES, _PLAYBACK, _read, asset, transform

ROOT = Path(__file__).resolve().parents[3]
VEXSIM = Path(os.environ.get("VEXSIM_PATH", ROOT.parent / "vexsim")).resolve()


class NativeViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (VEXSIM / "vexsim/web/push_back.html").read_text()
        cls.bundle = transform(cls.source)

    def test_native_scene_is_reused_and_live_transport_is_removed(self):
        js = self.bundle["js"].decode()
        # The actual chassis/field builders are retained from installed vexsim.
        for start, end in (("function placeholder(", "function orient("),
                           ("// floor tiles", "// perimeter")):
            self.assertIn(self.source[self.source.index(start):self.source.index(end)], js)
        self.assertNotIn("setInterval", js)
        self.assertNotIn("fetch('/input')", js)
        self.assertNotIn("fetch('/state')", js)
        self.assertNotIn("hud(s)", js)
        self.assertNotIn("turnWheels(dt);", js)
        self.assertNotIn("ACTION_KEYS", js)
        self.assertIn("setView('overhead');", js)
        self.assertIn("child.visible = child === floor", js)
        self.assertIn("builtGoals.visible = false", js)
        self.assertIn("builtProps.visible = false", js)
        self.assertIn("const HAS_MODEL = false", js)
        self.assertIn("const HAS_FIELD = false", js)
        self.assertIn("const HAS_BLOCK = false", js)

    def test_html_uses_local_assets_and_hashed_importmap(self):
        html = self.bundle["html"].decode()
        self.assertNotIn("cdn.jsdelivr", html)
        self.assertNotIn(" style=", html)
        self.assertIn('src="/native/viewer.js"', html)
        self.assertIn('href="/native/viewer.css"', html)
        mapping = re.search(r'<script type="importmap">(.*?)</script>', html, re.S).group(1)
        self.assertEqual(json.loads(mapping)["imports"]["three"], "/native/three/build/three.module.js")
        digest = base64.b64encode(hashlib.sha256(mapping.encode()).digest()).decode()
        self.assertIn("'sha256-" + digest + "'", self.bundle["csp"])
        self.assertIn("frame-ancestors 'self'", self.bundle["csp"])
        self.assertNotIn("unsafe-inline", self.bundle["csp"])

    def test_all_routes_and_dependencies_are_exact_local_assets(self):
        for path, content_type in (("viewer.html", "text/html"), ("viewer.js", "text/javascript"),
                                   ("viewer.css", "text/css"), ("field.json", "application/json")):
            data, actual_type, csp = asset("/native/" + path, VEXSIM)
            self.assertGreater(len(data), 100)
            self.assertTrue(actual_type.startswith(content_type))
            self.assertEqual(csp is not None, path == "viewer.html")
        field = json.loads(asset("/native/field.json", VEXSIM)[0])
        self.assertEqual(len(field["size"]), 2)
        self.assertTrue(all(3 < dimension < 4 for dimension in field["size"]))
        for relative in THREE_FILES:
            data, content_type, csp = asset("/native/three/" + relative, VEXSIM)
            self.assertEqual(data, (VEXSIM / "node_modules/three" / relative).read_bytes())
            self.assertTrue(content_type.startswith("text/javascript"))
            self.assertIsNone(csp)

    def test_traversal_and_unlisted_files_are_unavailable(self):
        for path in ("/native/../README.md", "/native/%2e%2e/README.md", "/native/viewer.html?x=1",
                     "/native/three/../../package.json", "/native/three/package.json",
                     "/native/three/build/../build/three.module.js", "/native/model.fbx",
                     "/native/input", "/native/state", "/etc/passwd"):
            with self.subTest(path=path):
                self.assertIsNone(asset(path, VEXSIM))
        with tempfile.TemporaryDirectory(prefix="mclib-native-assets-") as directory:
            root = Path(directory)
            (root / "escape").symlink_to(VEXSIM / "README.md")
            with self.assertRaises(NativeViewerError):
                _read(root, "escape")

    def test_changed_native_transport_fails_closed(self):
        for old, replacement in (("fetch('/field')", "fetch('/different-field')"),
                                  ("// ------------------------------------------------------------------- input", "// new input"),
                                  ("await poll();", "await pollState();"),
                                  ("three@0.160.0", "three@0.161.0"),
                                  ("  turnWheels(dt);", "  turnWheels(dt * 2);")):
            with self.subTest(anchor=old), self.assertRaises(NativeViewerError):
                transform(self.source.replace(old, replacement))
        with self.assertRaises(NativeViewerError):
            transform(self.source.replace("\nframe();", "\nsetInterval(poll, 1);\nframe();"))

    def test_changed_installed_three_version_invalidates_cached_bundle(self):
        with tempfile.TemporaryDirectory(prefix="mclib-native-version-") as directory:
            root = Path(directory)
            page = root / "vexsim/web/push_back.html"
            page.parent.mkdir(parents=True)
            page.write_text(self.source)
            package = root / "node_modules/three/package.json"
            package.parent.mkdir(parents=True)
            package.write_text('{"version":"0.160.0"}')
            self.assertIsNotNone(asset("/native/viewer.html", root))
            previous = package.stat()
            package.write_text('{"version":"0.161.0"}')
            os.utime(package, ns=(previous.st_atime_ns, previous.st_mtime_ns + 1000000))
            with self.assertRaises(NativeViewerError):
                asset("/native/viewer.html", root)

    @unittest.skipUnless(shutil.which("node"), "Node is required for JavaScript checks")
    def test_adapted_native_module_syntax(self):
        checked = subprocess.run(["node", "--input-type=module", "--check"],
                                 input=self.bundle["js"].decode(), text=True, capture_output=True)
        self.assertEqual(checked.returncode, 0, checked.stderr)

    def native_scene_fixture(self):
        js = self.bundle["js"].decode()
        apply = js[js.index("function apply(s) {"):js.index("// ------------------------------------------------------------------ startup")]
        placeholder = js[js.index("function placeholder("):js.index("function orient(")]
        set_view = js[js.index("function setView("):js.index("// -------------------------------------------------------------------- state")]
        fixture = "import * as THREE from " + json.dumps(
            (VEXSIM / "node_modules/three/build/three.module.js").as_uri()) + ";\n" + """
import assert from 'node:assert/strict';
const IN=.0254, P=(x,y,z=0)=>new THREE.Vector3(x,z,-y), FX=3.567,FY=3.567;
const floor={}, wall={}, builtGoals={}, builtProps={}, builtGround={children:[floor,wall]};
const scene=new THREE.Scene(), robot=new THREE.Group(); scene.add(robot);
const BLOCK_ROOM=128, swarm={red:{instanceMatrix:{}},blue:{instanceMatrix:{}}};
let body=null, modelYaw=0, state=null, lastHud=0, view='overhead';
let innerWidth=1366,innerHeight=600,statusHeight=26;
const status={getBoundingClientRect:()=>({height:statusHeight})}, document={getElementById:()=>status};
const location={origin:'http://127.0.0.1:8765'}, messages=[], handlers={};
const parent={postMessage:(message,origin)=>{assert.equal(origin,location.origin);messages.push(message);}};
const addEventListener=(type,callback)=>{handlers[type]=callback;};
const camera=new THREE.PerspectiveCamera(48,innerWidth/innerHeight,.05,100);
const controls={target:new THREE.Vector3(),update(){}};
function send(data,origin=location.origin,source=parent) { handlers.message({data,origin,source}); }
"""
        return fixture + placeholder + set_view + apply + _PLAYBACK

    @unittest.skipUnless(shutil.which("node"), "Node is required for JavaScript checks")
    def test_message_mapping_uses_native_apply_and_clears_missing_geometry(self):
        checks = """
assert.equal(messages.at(-1).type,'mclib-vexsim-ready');
assert.equal(wall.visible,false);
assert.equal(floor.visible,true);
const frame={t:2,true_x:12,true_y:-94,true_heading:90,body_length_in:15,body_width_in:12.5,
             left_wheel_angle_rad:7,right_wheel_angle_rad:-8};
const message={type:'mclib-vexsim-frame',frame,playing:true,live:false};
send(message,'http://evil.invalid');
send(message,location.origin,{});
assert.equal(messages.length,1);
send(message);
let ack=messages.at(-1);
assert.equal(ack.available,true);
assert.equal(ack.robot.x,-94*.0254);
assert.equal(ack.robot.y,-12*.0254);
assert.equal(ack.robot.theta,-Math.PI/2);
assert.equal(ack.robot.x,robot.position.x);
assert.equal(ack.robot.y,-robot.position.z);
assert.equal(ack.wheel_angles_rad.left,7);
assert.equal(ack.wheels_rendered,false);
assert.equal(body.children[0].geometry.parameters.width,15*.0254);
send({...message,frame:{...frame,body_length_in:18}});
assert.equal(body.children[0].geometry.parameters.width,18*.0254);
send({...message,frame:{...frame,body_length_in:null}});
assert.equal(messages.at(-1).robot,null);
assert.equal(robot.visible,false);
send({...message,frame:{...frame,true_heading:Infinity}});
assert.equal(messages.at(-1).robot,null);
send({...message,frame:null});
assert.equal(messages.at(-1).t,null);
assert.equal(messages.at(-1).robot,null);
send(message);
const bounds={min_x:-100,max_x:20,min_y:-94,max_y:24};
send({type:'mclib-vexsim-fit',bounds});
assert.equal(messages.at(-1).type,'mclib-vexsim-fitted');
assert.equal(controls.target.x,(-94+24)*.0254/2);
assert.equal(controls.target.z,(-100+20)*.0254/2);
assert(camera.position.y>0);
const count=messages.length;
send({type:'mclib-vexsim-fit',bounds:{...bounds,max_x:3000}});
assert.equal(messages.length,count);
"""
        result = subprocess.run(["node", "--input-type=module"],
                                input=self.native_scene_fixture() + checks,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(shutil.which("node"), "Node is required for JavaScript checks")
    def test_fit_projects_entire_native_body_at_path_corners_desktop_and_mobile(self):
        checks = """
let checks=0;
const paths=[
  {min_x:0,max_x:15.5,min_y:0,max_y:18.5},
  {min_x:-7,max_x:5,min_y:-94,max_y:0},
  {min_x:-94,max_x:0,min_y:-7,max_y:5},
  {min_x:12,max_x:12,min_y:-94,max_y:-94},
];
function record(x,y,heading) {
  send({type:'mclib-vexsim-frame',check_viewport:true,frame:{t:2,true_x:x,true_y:y,
    true_heading:heading,body_length_in:15,body_width_in:12.5}});
  return messages.at(-1).viewport;
}
function checkVisible(projected,safe) {
  for (const axis of ['x','y']) {
    assert(projected['min_'+axis] >= safe['min_'+axis],JSON.stringify({projected,safe}));
    assert(projected['max_'+axis] <= safe['max_'+axis],JSON.stringify({projected,safe}));
    checks+=2;
  }
  assert(projected.min_z>=-1 && projected.max_z<=1);
  checks++;
}
for (const [width,height,labelHeight] of [[1366,600,26],[390,260,42]]) {
  innerWidth=width; innerHeight=height; statusHeight=labelHeight;
  camera.aspect=width/height; camera.updateProjectionMatrix();
  for (const bounds of paths) {
    record(bounds.min_x,bounds.min_y,0);
    send({type:'mclib-vexsim-fit',bounds});
    const fitted=messages.at(-1);
    assert.equal(fitted.type,'mclib-vexsim-fitted');
    assert(fitted.footprint_radius_m>=Math.hypot(15,12.5)*.0254/2);
    assert(fitted.body_top_m>=.165-1e-7);
    assert.equal(fitted.camera.target[0],(bounds.min_y+bounds.max_y)*.0254/2);
    assert.equal(fitted.camera.target[2],(bounds.min_x+bounds.max_x)*.0254/2);
    const safe=fitted.safe_viewport;
    for (const x of [bounds.min_x,bounds.max_x]) for (const y of [bounds.min_y,bounds.max_y]) {
      for (const heading of [0,45,90,135,180,225,270,315]) {
        checkVisible(record(x,y,heading),safe);
      }
    }
    // Resizing an already fitted iframe must recompute its native perspective.
    innerWidth=width===1366?390:1366; innerHeight=width===1366?260:600;
    camera.aspect=innerWidth/innerHeight; camera.updateProjectionMatrix();
    handlers.resize();
    const resized=messages.at(-1);
    assert.equal(resized.type,'mclib-vexsim-fitted');
    checkVisible(resized.viewport,resized.safe_viewport);
    innerWidth=width; innerHeight=height;
    camera.aspect=width/height; camera.updateProjectionMatrix();
  }
}
assert.equal(checks,1320);
"""
        result = subprocess.run(["node", "--input-type=module"],
                                input=self.native_scene_fixture() + checks,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
