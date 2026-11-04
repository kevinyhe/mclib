// The Override field in 3D, built from the field drawing the planner already
// has. Planner inches: +X right, +Y away from the red Alliance Station, +Z up.
// Three.js here is Y up, so a planner point (x, y, z) is (x, z, -y).
import * as THREE from "/native/three/build/three.module.js";

const COLORS = {
  red: 0xc8323a, blue: 0x2a62c9, yellow: 0xf0c02a, gray: 0x8d929a, clear: 0xd8ecff,
  neutral: 0x5b6068, center: 0x41454c, tile: 0x5a5e66, seam: 0x4a4e55, tape: 0xf2f2f2,
  wall: 0xbfd0e0, rail: 0xa9afb8, flange: 0x30343a, chassis: 0x3a3e46, robot: 0xb3262e,
  metal: 0xc4c9d0, wheel: 0x1c1e22, toggleEnd: 0x2b2e34, hole: 0x0e0f12,
};
const VIEWS = { "1": "operator", "2": "audience", "3": "overhead", "4": "free", "5": "robot" };

const host = document.getElementById("scene");
const statusLine = document.getElementById("status");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(2, devicePixelRatio));
host.append(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x15171c);
const camera = new THREE.PerspectiveCamera(40, 1, 1, 3000);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404048, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(60, 160, 90);
scene.add(sun);

const orbit = { target: new THREE.Vector3(0, 0, 0), radius: 210, azimuth: 0, elevation: 0.62 };
let view = "operator";
let solids = null;
let robot = null;
const toggleMeshes = [];
const pieceMeshes = new Map();

function at(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

function direction(x, y, z) {
  return new THREE.Vector3(x, z, -y).normalize();
}

function material(color, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.75, metalness: 0.05, ...extra });
}

function tellParent(message) {
  parent.postMessage(message, location.origin);
}

function placeCamera() {
  const flat = Math.cos(orbit.elevation) * orbit.radius;
  camera.position.set(orbit.target.x + Math.sin(orbit.azimuth) * flat, orbit.target.y + Math.sin(orbit.elevation) * orbit.radius,
    orbit.target.z + Math.cos(orbit.azimuth) * flat);
  camera.lookAt(orbit.target);
}

function setView(name) {
  view = name;
  orbit.target.set(0, 0, 0);
  if (name === "operator") Object.assign(orbit, { radius: 215, azimuth: 0, elevation: 0.62 });
  if (name === "audience") Object.assign(orbit, { radius: 215, azimuth: Math.PI / 2, elevation: 0.5 });
  if (name === "overhead") Object.assign(orbit, { radius: 250, azimuth: 0, elevation: Math.PI / 2 - 0.001 });
  if (name === "robot") followRobot();
  placeCamera();
}

function followRobot() {
  if (!robot?.group.visible) return;
  orbit.target.copy(robot.group.position).add(new THREE.Vector3(0, 7, 0));
  Object.assign(orbit, { radius: 46, azimuth: robot.group.rotation.y + 2.3, elevation: 0.33 });
}

function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
  camera.aspect = innerWidth / Math.max(1, innerHeight);
  camera.updateProjectionMatrix();
}

function addFloor(drawing) {
  const half = drawing.half_in;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(2 * half, 2 * half), material(COLORS.tile, { roughness: 1 }));
  floor.rotation.x = -Math.PI / 2;
  scene.add(floor);
  const seams = [];
  for (const value of solids.tile_seams_in) {
    seams.push(at(value, -half, 0.03), at(value, half, 0.03), at(-half, value, 0.03), at(half, value, 0.03));
  }
  scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(seams), new THREE.LineBasicMaterial({ color: COLORS.seam })));
  const [start, end] = drawing.tape.auton_line;
  const length = Math.hypot(end[0] - start[0], end[1] - start[1]);
  const line = new THREE.Mesh(new THREE.PlaneGeometry(length, drawing.tape.auton_line_width_in), material(COLORS.tape, { roughness: 1 }));
  line.rotation.set(-Math.PI / 2, 0, Math.atan2(end[1] - start[1], end[0] - start[0]));
  line.position.copy(at((start[0] + end[0]) / 2, (start[1] + end[1]) / 2, 0.05));
  scene.add(line);
  const midfield = drawing.tape.midfield.map(([x, y]) => at(x, y, 0.05));
  scene.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(midfield), new THREE.LineBasicMaterial({ color: COLORS.tape })));
}

function addWalls(drawing) {
  const half = drawing.half_in, height = solids.wall_height_in, thick = solids.wall_thickness_in;
  const panel = material(COLORS.wall, { transparent: true, opacity: 0.28, depthWrite: false });
  const rail = material(COLORS.rail, { metalness: 0.4 });
  for (const [x, y, alongX] of [[0, -half - thick / 2, true], [0, half + thick / 2, true], [-half - thick / 2, 0, false], [half + thick / 2, 0, false]]) {
    const width = alongX ? 2 * (half + thick) : thick, depth = alongX ? thick : 2 * (half + thick);
    const wall = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), panel);
    wall.position.copy(at(x, y, height / 2));
    scene.add(wall);
    const top = new THREE.Mesh(new THREE.BoxGeometry(width, 0.8, depth), rail);
    top.position.copy(at(x, y, height - 0.4));
    scene.add(top);
  }
}

function goalColor(kind) {
  return { red: COLORS.red, blue: COLORS.blue, neutral: COLORS.neutral, center: COLORS.center }[kind];
}

function addGoals(drawing) {
  const shapes = new Map(solids.goals.map((goal) => [goal.name, goal]));
  for (const goal of drawing.goals) {
    const solid = shapes.get(goal.name);
    const color = material(goalColor(goal.kind));
    const outline = new THREE.Shape(goal.outline.map(([x, y]) => new THREE.Vector2(x, y)));
    const base = new THREE.Mesh(new THREE.ExtrudeGeometry(outline, { depth: solid.top_of_base_in, bevelEnabled: false }), color);
    base.rotation.x = -Math.PI / 2;
    scene.add(base);
    const taper = solid.height_in - solid.top_of_base_in;
    const cone = new THREE.Mesh(new THREE.CylinderGeometry(solid.top_radius_in, solid.base_radius_in, taper, 32, 1, true), color);
    cone.position.copy(at(goal.center[0], goal.center[1], solid.top_of_base_in + taper / 2));
    scene.add(cone);
    const rim = new THREE.Mesh(new THREE.RingGeometry(solid.hole_radius_in, solid.top_radius_in, 32), color);
    rim.rotation.x = -Math.PI / 2;
    rim.position.copy(at(goal.center[0], goal.center[1], solid.height_in));
    scene.add(rim);
    const hole = new THREE.Mesh(new THREE.CircleGeometry(solid.hole_radius_in, 32), material(COLORS.hole));
    hole.rotation.x = -Math.PI / 2;
    hole.position.copy(at(goal.center[0], goal.center[1], solid.height_in - 0.4));
    scene.add(hole);
  }
}

const loaderMeshes = [];

function addLoaders(drawing) {
  const { opening_in: bottom, top_in: top } = solids.loader;
  for (const loader of drawing.loaders) {
    const xs = loader.outline.map((p) => p[0]), ys = loader.outline.map((p) => p[1]);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    const tube = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, top - bottom, y1 - y0),
      material(COLORS[loader.alliance], { transparent: true, opacity: 0.55 }));
    const lowered = at((x0 + x1) / 2, (y0 + y1) / 2, (top + bottom) / 2);
    tube.position.copy(lowered);
    scene.add(tube);
    loaderMeshes.push({ name: loader.name, tube, lowered });
  }
}

function showLoaders(frame) {
  const raised = new Set(frame?.raised_loaders ?? []);
  for (const loader of loaderMeshes) {
    loader.tube.position.copy(loader.lowered);
    if (raised.has(loader.name)) loader.tube.position.y += solids.loader.raised_by_in;
  }
}

function addToggles() {
  for (const toggle of solids.toggles) {
    const along = direction(-toggle.inward[1], toggle.inward[0], 0);
    const up = new THREE.Vector3(0, 1, 0);
    const inward = direction(toggle.inward[0], toggle.inward[1], 0);
    const side = new THREE.Vector3().crossVectors(along, up);
    const sign = Math.sign(side.dot(inward));
    const mount = new THREE.Group();
    mount.position.copy(at(toggle.axle[0], toggle.axle[1], toggle.axle_height_in));
    mount.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(along, up, side));
    const spin = new THREE.Group();
    mount.add(spin);
    const half = toggle.length_in / 2, radius = toggle.circumradius_in;
    const corner = (degrees) => {
      const angle = degrees * Math.PI / 180;
      return [radius * Math.cos(angle), radius * Math.sin(angle)];
    };
    const local = ([n, z], s) => new THREE.Vector3(s, z, sign * n);
    toggle.faces.forEach((color, index) => {
      const normal = toggle.face_angles_deg[index];
      const a = corner(normal - 60), b = corner(normal + 60);
      const points = [local(a, -half), local(b, -half), local(b, half), local(a, -half), local(b, half), local(a, half)];
      const face = new THREE.Mesh(new THREE.BufferGeometry().setFromPoints(points), material(COLORS[color], { side: THREE.DoubleSide }));
      face.geometry.computeVertexNormals();
      spin.add(face);
    });
    for (const s of [-half, half]) {
      const cap = new THREE.Mesh(new THREE.BufferGeometry().setFromPoints([90, 210, 330].map((d) => local(corner(d), s))),
        material(COLORS.toggleEnd, { side: THREE.DoubleSide }));
      spin.add(cap);
    }
    const axle = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.25, toggle.length_in + 3, 12), material(COLORS.metal, { metalness: 0.5 }));
    axle.rotation.z = Math.PI / 2;
    mount.add(axle);
    scene.add(mount);
    const faces = toggle.faces.map((color, index) => {
      const angle = toggle.face_angles_deg[index] * Math.PI / 180;
      return { color, normal: new THREE.Vector3(0, Math.sin(angle), sign * Math.cos(angle)) };
    });
    const intoTheField = inward.clone().multiplyScalar(Math.cos(Math.PI / 6)).addScaledVector(up, Math.sin(Math.PI / 6));
    toggleMeshes.push({ quadrant: toggle.quadrant, spin, sign, faces, intoTheField });
  }
}

function shownInnerColor(toggle) {
  const turned = toggle.spin.getWorldQuaternion(new THREE.Quaternion());
  const facing = (face) => face.normal.clone().applyQuaternion(turned).dot(toggle.intoTheField);
  return toggle.faces.reduce((best, face) => (facing(face) > facing(best) ? face : best)).color;
}

function lathe(profile, color, extra = {}) {
  const points = profile.map(([axial, radius]) => new THREE.Vector2(radius, axial));
  return new THREE.Mesh(new THREE.LatheGeometry(points, 28), material(color, { side: THREE.DoubleSide, ...extra }));
}

function pinMesh(colors) {
  const group = new THREE.Group();
  const upper = solids.pin_half_profile;
  const lower = upper.map(([axial, radius]) => [-axial, radius]).reverse();
  group.add(lathe([[lower[0][0], 0], ...lower], COLORS[colors[0]]));
  group.add(lathe([...upper, [upper[upper.length - 1][0], 0]], COLORS[colors[1]]));
  const flange = new THREE.Mesh(new THREE.CylinderGeometry(solids.pin_flange.radius_in, solids.pin_flange.radius_in,
    solids.pin_flange.thickness_in, 8), material(COLORS.flange));
  group.add(flange);
  return group;
}

function cupMesh() {
  const group = new THREE.Group();
  const upper = solids.cup_half_profile;
  const lower = upper.map(([axial, radius]) => [-axial, radius]).reverse();
  group.add(lathe(lower, COLORS.gray));
  group.add(lathe(upper, COLORS.clear, { transparent: true, opacity: 0.4, depthWrite: false }));
  return group;
}

function showPieces(pieces) {
  const seen = new Set();
  for (const piece of pieces) {
    if (!piece.pose) continue;
    let mesh = pieceMeshes.get(piece.id);
    if (!mesh) {
      mesh = piece.kind === "pin" ? pinMesh(piece.colors) : cupMesh();
      pieceMeshes.set(piece.id, mesh);
      scene.add(mesh);
    }
    const [x, y, z, ax, ay, az] = piece.pose;
    mesh.position.copy(at(x, y, z));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction(ax, ay, az));
    mesh.visible = true;
    seen.add(piece.id);
  }
  for (const [id, mesh] of pieceMeshes) if (!seen.has(id)) mesh.visible = false;
}

function box(width, height, length, color) {
  return new THREE.Mesh(new THREE.BoxGeometry(width, height, length), material(color));
}

// Robot-frame inches to the robot group's coordinates: forward is -Z, left
// is -X and up is +Y.
function placeBox(mesh, back, front, right, left, bottom, top) {
  mesh.scale.set(Math.max(0.01, left - right), Math.max(0.01, top - bottom), Math.max(0.01, front - back));
  mesh.position.set(-(left + right) / 2, (top + bottom) / 2, -(front + back) / 2);
}

function partBox(group, color) {
  const mesh = box(1, 1, 1, color);
  group.add(mesh);
  return mesh;
}

// A convex outline in the robot frame, extruded from `bottom` to `top`.
function extruded(outline, bottom, top, color) {
  const shape = new THREE.Shape(outline.map(([forward, left]) => new THREE.Vector2(-left, forward)));
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: top - bottom, bevelEnabled: false });
  geometry.rotateX(-Math.PI / 2);
  const mesh = new THREE.Mesh(geometry, material(color, { side: THREE.DoubleSide }));
  mesh.position.y = bottom;
  return mesh;
}

// The parts come from vexsim with the lift down and the claw open (see
// cascade_drawing() in fields.py): the bite's frame and the Toggle bars stay
// put, the rails telescope, the carriage and the plate rise with the claw,
// and the jaws also slide in as the claw closes.
function buildRobot(lengthIn, widthIn) {
  const shape = drawing.robot;
  const group = new THREE.Group();
  const half = lengthIn / 2, side = widthIn / 2;
  const rails = [], lift = [], jaws = [];
  for (const part of shape.parts) {
    if (part.moves === "fixed" && part.name === "drivetrain") {
      group.add(extruded(part.outline, part.bottom, part.top - 1.2, COLORS.chassis));
      group.add(extruded(part.outline, part.top - 1.2, part.top, COLORS.robot));
    } else if (part.moves === "fixed") {
      placeBox(partBox(group, COLORS.metal), part.back, part.front, part.right, part.left, part.bottom, part.top);
    } else if (part.moves === "rails") {
      const width = (part.left - part.right) / shape.stages;
      for (let stage = 0; stage < shape.stages; stage++) {
        const outer = part.left > 0 ? part.left - stage * width : part.right + stage * width;
        const inner = part.left > 0 ? outer - width : outer + width;
        rails.push({ stage, part, right: Math.min(inner, outer), left: Math.max(inner, outer),
          mesh: partBox(group, stage % 2 ? COLORS.metal : COLORS.rail) });
      }
    } else if (part.moves === "lift") {
      lift.push({ part, mesh: partBox(group, part.name === "carriage" ? COLORS.robot : COLORS.metal) });
    } else {
      jaws.push({ part, mesh: partBox(group, COLORS.robot) });
    }
  }
  for (const across of [-1, 1]) for (const end of [-1, 1]) {
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(1.625, 1.625, 1.0, 20), material(COLORS.wheel));
    wheel.rotation.z = Math.PI / 2;
    wheel.position.set(across * (side + 0.5), 1.625, end * (half - 2));
    group.add(wheel);
  }
  scene.add(group);
  return { group, lengthIn, widthIn, rails, lift, jaws };
}

function poseRobot(frame) {
  const shape = drawing.robot;
  const rise = frame.lift_height_in - shape.claw.lowest_in;
  const closing = (shape.claw.open_gap_in - frame.claw_gap_in) / 2;
  for (const rail of robot.rails) {
    const bottom = shape.rail_bottom_in + rise * rail.stage / shape.stages;
    placeBox(rail.mesh, rail.part.back, rail.part.front, rail.right, rail.left, bottom, bottom + shape.rail_length_in);
  }
  for (const { part, mesh } of robot.lift) placeBox(mesh, part.back, part.front, part.right, part.left, part.bottom + rise, part.top + rise);
  for (const { part, mesh } of robot.jaws) {
    const inward = part.left > 0 ? -closing : closing;
    placeBox(mesh, part.back, part.front, part.right + inward, part.left + inward, part.bottom + rise, part.top + rise);
  }
  robot.group.position.copy(at(frame.true_x, frame.true_y, 0));
  robot.group.rotation.y = -frame.true_heading * Math.PI / 180;
}

function showToggles(frame) {
  for (const toggle of toggleMeshes) {
    const angle = frame?.toggles?.[toggle.quadrant]?.angle_deg ?? 0;
    toggle.spin.rotation.x = -toggle.sign * angle * Math.PI / 180;
  }
}

function showFrame(frame, pieces, playing, live) {
  showPieces(pieces ?? []);
  showToggles(frame);
  showLoaders(frame);
  if (!frame) {
    if (robot) robot.group.visible = false;
    statusLine.textContent = "Override field · no recorded frame · 1–5 camera views, drag to turn, scroll to zoom";
    tellParent({ type: "mclib-vexsim-applied", t: null, robot: null, available: false, reason: "no recorded frame" });
    return;
  }
  if (!robot || robot.lengthIn !== frame.body_length_in || robot.widthIn !== frame.body_width_in) {
    if (robot) scene.remove(robot.group);
    robot = buildRobot(frame.body_length_in, frame.body_width_in);
  }
  robot.group.visible = true;
  poseRobot(frame);
  if (view === "robot") {
    followRobot();
    placeCamera();
  }
  const holding = frame.holding ? ` · holding ${frame.holding}` : "";
  statusLine.textContent = `Override field · ${live ? "live" : playing ? "replay" : "paused"} · ${frame.t.toFixed(2)} s${holding} · 1–5 camera views`;
  tellParent({ type: "mclib-vexsim-applied", t: frame.t, available: true,
    robot: { x: frame.true_x, y: frame.true_y, heading: frame.true_heading, lift_height_in: frame.lift_height_in,
      claw_gap_in: frame.claw_gap_in, raised_loaders: frame.raised_loaders },
    toggles: Object.fromEntries(toggleMeshes.map((toggle) => [toggle.quadrant, shownInnerColor(toggle)])) });
}

let drawing = null;
let dragging = null;
renderer.domElement.addEventListener("pointerdown", (event) => {
  dragging = { x: event.clientX, y: event.clientY };
  renderer.domElement.setPointerCapture(event.pointerId);
});
renderer.domElement.addEventListener("pointermove", (event) => {
  if (!dragging) return;
  orbit.azimuth -= (event.clientX - dragging.x) * 0.006;
  orbit.elevation = Math.min(Math.PI / 2 - 0.001, Math.max(0.05, orbit.elevation + (event.clientY - dragging.y) * 0.006));
  dragging = { x: event.clientX, y: event.clientY };
  view = "free";
  placeCamera();
});
renderer.domElement.addEventListener("pointerup", () => { dragging = null; });
renderer.domElement.addEventListener("wheel", (event) => {
  event.preventDefault();
  orbit.radius = Math.min(600, Math.max(30, orbit.radius * Math.exp(event.deltaY * 0.001)));
  placeCamera();
}, { passive: false });
addEventListener("keydown", (event) => {
  if (VIEWS[event.key]) {
    event.preventDefault();
    setView(VIEWS[event.key]);
  }
});
addEventListener("resize", resize);
addEventListener("error", (event) => tellParent({ type: "mclib-vexsim-error", message: String(event.message) }));
addEventListener("message", (event) => {
  if (event.origin !== location.origin || event.source !== parent || !drawing) return;
  const data = event.data;
  if (data?.type === "mclib-vexsim-frame") showFrame(data.frame, data.pieces, data.playing, data.live);
  if (data?.type === "mclib-vexsim-view" && Object.values(VIEWS).includes(data.view)) setView(data.view);
});

async function start() {
  const response = await fetch("/api/config");
  if (!response.ok) throw new Error(`The planner did not send the field (${response.status}).`);
  drawing = (await response.json()).field_drawings.override;
  solids = drawing.solid;
  addFloor(drawing);
  addWalls(drawing);
  addGoals(drawing);
  addLoaders(drawing);
  addToggles();
  showPieces(drawing.pieces);
  resize();
  setView("operator");
  renderer.setAnimationLoop(() => renderer.render(scene, camera));
  statusLine.textContent = "Override field · 1–5 camera views, drag to turn, scroll to zoom";
  tellParent({ type: "mclib-vexsim-ready", renderer: "override-field" });
}

start().catch((error) => {
  statusLine.textContent = `The 3D field did not start: ${error.message}`;
  tellParent({ type: "mclib-vexsim-error", message: error.message });
});
