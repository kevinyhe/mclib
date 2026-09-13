// Headless checks of the actual browser transforms and animation functions.
// Usage: node tests/vexsim/physics_game_audit.mjs [vexsim-root]
// With --bake, also run the existing offline Rapier prototype in memory:
// intercept its output-file operations, leaving the sibling checkout untouched.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = path.resolve(process.argv.slice(2).find(x => !x.startsWith('--')) ||
  fileURLToPath(new URL('../../../vexsim', import.meta.url)));
const THREE = await import(pathToFileURL(path.join(root, 'node_modules/three/build/three.module.js')));
const page = fs.readFileSync(path.join(root, 'vexsim/web/push_back.html'), 'utf8');
function sourceBetween(start, end) {
  const lo = page.indexOf(start), hi = page.indexOf(end, lo);
  assert(lo >= 0 && hi > lo, `source markers missing: ${start}`);
  return page.slice(lo, hi);
}
const robot = new THREE.Group();
const bank = new THREE.Object3D();
const swarm = Object.fromEntries(['red', 'blue'].map(c => [c, {
  instanceMatrix: {}, setMatrixAt(i, matrix) { this.matrix = matrix.clone(); },
}]));
const state = { paused: false, spin: { left: 100, right: 100, intake: 0 } };
const pivot = new THREE.Object3D();
const spinners = { drive: [{ pivot, axis: new THREE.Vector3(0, 0, 1), sign: 1, side: 'left' }], rollers: [] };
const src = `${page.match(/^const P = .*;$/m)[0]}
let body = new THREE.Group(), modelYaw = 0, lastHud = Infinity;
const BLOCK_ROOM = 100;
${sourceBetween('function apply(s)', '// ---------------------------------------------------------------------- HUD')}
${sourceBetween('function turnWheels(dt)', '// Keep the frame rate')}
return { apply, turnWheels };`;
const api = new Function('THREE', 'robot', 'bank', 'swarm', 'state', 'spinners', src)
  (THREE, robot, bank, swarm, state, spinners);
let passes = 0, failures = 0;
function check(name, fn) {
  try { fn(); passes++; console.log(`PASS ${name}`); }
  catch (error) { failures++; console.log(`FAIL ${name}: ${error.message}`); }
}
const sample = { robot: { x: 0.4, y: -0.3, theta: Math.PI / 2 }, blocks: [
  { x: 0.2, y: 0.5, z: 0.3, c: 'red', q: [0, 0, Math.SQRT1_2, Math.SQRT1_2] },
] };
api.apply(sample);
check('robot translation and CCW heading map into three.js axes', () => {
  assert(robot.position.distanceTo(new THREE.Vector3(0.4, 0, 0.3)) < 1e-12);
  assert(new THREE.Vector3(1, 0, 0).applyQuaternion(robot.quaternion)
    .distanceTo(new THREE.Vector3(0, 0, -1)) < 1e-12);
});
check('block translation and quaternion agree in the same coordinate basis', () => {
  assert(new THREE.Vector3().setFromMatrixPosition(swarm.red.matrix)
    .distanceTo(new THREE.Vector3(0.2, 0.3, -0.5)) < 1e-12);
  assert(new THREE.Vector3(1, 0, 0).transformDirection(swarm.red.matrix)
    .distanceTo(new THREE.Vector3(0, 0, -1)) < 1e-12);
});
check('active drive animation follows supplied RPM', () => {
  api.turnWheels(0.01);
  assert(Math.abs(pivot.rotation.z + 100 * Math.PI * 2 / 60 * 0.01) < 1e-12);
});
check('paused state freezes wheel animation', () => {
  state.paused = true;
  const before = pivot.quaternion.clone();
  api.turnWheels(0.1);
  const angle = before.angleTo(pivot.quaternion);
  // angleTo uses acos(dot), which can report ~3e-8 rad for the exact same
  // floating-point quaternion.  A paused animation must leave all four
  // components unchanged, so compare those directly.
  assert(before.equals(pivot.quaternion), `wheel moved ${angle.toFixed(9)} rad during a 0.1 s pause`);
});
console.log(`BROWSER CHECKS: ${passes} passed, ${failures} failed`);

if (process.argv.includes('--bake')) {
  const bakePath = path.join(root, 'tools/bake_intake.mjs');
  const bake = fs.readFileSync(bakePath, 'utf8')
    .replace(/^import .*;$/gm, '')
    .replaceAll('import.meta.url', JSON.stringify(pathToFileURL(bakePath).href));
  const rapierDir = path.join(root, 'node_modules/@dimforge/rapier3d-compat');
  const rapierManifest = JSON.parse(fs.readFileSync(path.join(rapierDir, 'package.json'), 'utf8'));
  const rapierPath = path.join(rapierDir, rapierManifest.module);
  const RAPIER = (await import(pathToFileURL(rapierPath))).default;
  let bytes = 0;
  const memoryFs = { ...fs,
    writeFileSync(_path, data) { bytes = Buffer.byteLength(data); },
    statSync(_path) { return { size: bytes }; },
  };
  const AsyncFunction = Object.getPrototypeOf(async function() {}).constructor;
  await new AsyncFunction('RAPIER', 'fs', bake)(RAPIER, memoryFs);
  console.log(`RAPIER output captured in memory (${bytes} bytes); no bake files modified`);
}
process.exitCode = failures ? 1 : 0;
