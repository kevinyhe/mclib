// The Override field in 3D: the floor and walls from the planner's field
// drawing, the Goals, Loaders, Toggles, Pins and Cups from VEX's field CAD
// (the meshes vexsim's tools/override_cad.py makes). Planner inches: +X right, +Y away from the red Alliance Station, +Z up.
// Three.js here is Y up, so a planner point (x, y, z) is (x, z, -y).
import * as THREE from "/native/three/build/three.module.js";

const COLORS = {
  red: 0xc8323a, blue: 0x2a62c9, yellow: 0xf0c02a, gray: 0x8d929a, clear: 0xd8ecff,
  neutral: 0x5b6068, center: 0x41454c, tile: 0x5a5e66, seam: 0x4a4e55, tape: 0xf2f2f2,
  wall: 0xbfd0e0, rail: 0xa9afb8, flange: 0x30343a, chassis: 0x3a3e46, robot: 0xb3262e,
  metal: 0xc4c9d0, toggleEnd: 0x2b2e34, hole: 0x0e0f12, rubber: 0x1c1e22,
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

// vexsim field inches to three.js: planner (-y, x, z), then (x, z, -y).
const FROM_VEXSIM = new THREE.Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1);
// A piece frame (across, across, axis) to the piece group's, whose up (+Y) is the axis.
const FROM_PIECE = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);

function rotationAndOffset([a, b, c], offset) {
  return new THREE.Matrix4().set(a[0], a[1], a[2], offset[0], b[0], b[1], b[2], offset[1], c[0], c[1], c[2], offset[2],
    0, 0, 0, 1);
}

function partGeometry(cad, number, transform) {
  const mesh = cad.layout.meshes[number];
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(
    new Float32Array(cad.blob, mesh.vertex_offset, mesh.vertices * 3).slice(), 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(cad.blob, mesh.index_offset, mesh.triangles * 3), 1));
  geometry.applyMatrix4(transform);
  geometry.computeVertexNormals();
  return geometry;
}

async function loadCad() {
  const responses = await Promise.all([fetch("/cad/view.json"), fetch("/cad/view.bin")]);
  for (const response of responses) {
    if (!response.ok) throw new Error((await response.json()).error);
  }
  const cad = { layout: await responses[0].json(), blob: await responses[1].arrayBuffer() };
  const cadAxes = rotationAndOffset(cad.layout.cad_axes, [0, 0, 0]);
  const shared = new Map();
  cad.geometry = (number) => {
    if (!shared.has(number)) shared.set(number, partGeometry(cad, number, cadAxes));
    return shared.get(number);
  };
  cad.placement = (instance) => FROM_VEXSIM.clone().multiply(rotationAndOffset(instance.rotation, instance.position_in));
  cad.plannerPosition = ({ position_in: [x, y] }) => [-y, x];
  cad.threePosition = ({ position_in: [x, y, z] }) => new THREE.Vector3(-y, z, -x);
  return cad;
}

function placedMesh(geometry, matrix, materials) {
  const mesh = new THREE.Mesh(geometry, materials);
  mesh.matrixAutoUpdate = false;
  mesh.matrix.copy(matrix);
  scene.add(mesh);
  return mesh;
}

function nearest(items, [x, y], positionOf) {
  const distance = (item) => Math.hypot(positionOf(item)[0] - x, positionOf(item)[1] - y);
  return items.reduce((best, item) => (distance(item) < distance(best) ? item : best));
}

function goalColor(kind) {
  return { red: COLORS.red, blue: COLORS.blue, neutral: COLORS.neutral, center: COLORS.center }[kind];
}

function addGoals(cad) {
  for (const number of solids.cad.goal_parts) {
    for (const instance of cad.layout.instances[number]) {
      const goal = nearest(drawing.goals, cad.plannerPosition(instance), (g) => g.center);
      placedMesh(cad.geometry(number), cad.placement(instance), material(goalColor(goal.kind)));
    }
  }
}

const loaderMeshes = [];

// Each Loader's CAD parts, moved up or down so its tube's top is at the
// manual's height: the CAD has the red Loaders 0.31 in higher than the blue.
function addLoaders(cad) {
  const toManualHeight = new Map();
  for (const instance of cad.layout.instances[solids.cad.loader_tube]) {
    const loader = nearest(drawing.loaders, cad.plannerPosition(instance), (l) => l.center);
    const placed = cad.geometry(solids.cad.loader_tube).clone().applyMatrix4(cad.placement(instance));
    placed.computeBoundingBox();
    const top = placed.boundingBox.max.y;
    toManualHeight.set(loader.name, new THREE.Matrix4().makeTranslation(0, solids.loader.top_in - top, 0));
  }
  for (const number of solids.cad.loader_parts) {
    for (const instance of cad.layout.instances[number]) {
      const loader = nearest(drawing.loaders, cad.plannerPosition(instance), (l) => l.center);
      const look = number === solids.cad.loader_tube
        ? material(COLORS[loader.alliance], { transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false })
        : material(COLORS.metal, { metalness: 0.4 });
      const lowered = toManualHeight.get(loader.name).clone().multiply(cad.placement(instance));
      loaderMeshes.push({ name: loader.name, mesh: placedMesh(cad.geometry(number), lowered, look), lowered });
    }
  }
}

function showLoaders(frame) {
  const raised = new Set(frame?.raised_loaders ?? []);
  const lift = new THREE.Matrix4().makeTranslation(0, solids.loader.raised_by_in, 0);
  for (const loader of loaderMeshes) {
    loader.mesh.matrix.copy(raised.has(loader.name) ? lift.clone().multiply(loader.lowered) : loader.lowered);
  }
}

// A triangle of the bar belongs to the face whose normal at rest is nearest
// its own; the ends, whose normals run along the axle, are dark.
function barGeometry(cad, matrix, faceNormals, along) {
  const geometry = cad.geometry(solids.cad.toggle_bar).clone().applyMatrix4(matrix);
  const position = geometry.attributes.position, index = geometry.index.array;
  const groups = faceNormals.map(() => []).concat([[]]);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  for (let t = 0; t < index.length; t += 3) {
    a.fromBufferAttribute(position, index[t]);
    b.fromBufferAttribute(position, index[t + 1]);
    c.fromBufferAttribute(position, index[t + 2]);
    const normal = b.sub(a).cross(c.sub(a)).normalize();
    const facing = faceNormals.map((faceNormal) => normal.dot(faceNormal));
    const face = Math.abs(normal.dot(along)) > 0.7 ? faceNormals.length : facing.indexOf(Math.max(...facing));
    groups[face].push(index[t], index[t + 1], index[t + 2]);
  }
  geometry.setIndex(groups.flat());
  let start = 0;
  groups.forEach((triangles, face) => {
    geometry.addGroup(start, triangles.length, face);
    start += triangles.length;
  });
  return geometry;
}

// Each Toggle's CAD parts, moved so the CAD axle is where vexsim has it.
// The bar and its end caps turn about the axle.
function addToggles(cad) {
  const turning = new Set(solids.cad.toggle_turning_parts);
  const up = new THREE.Vector3(0, 1, 0);
  for (const toggle of solids.toggles) {
    const position = (instance) => cad.plannerPosition(instance);
    const bar = nearest(cad.layout.instances[solids.cad.toggle_bar], toggle.axle, position);
    const axle = at(toggle.axle[0], toggle.axle[1], toggle.axle_height_in);
    const toPhysicsAxle = new THREE.Matrix4().makeTranslation(axle.clone().sub(cad.threePosition(bar)));
    const along = direction(-toggle.inward[1], toggle.inward[0], 0);
    const inward = direction(toggle.inward[0], toggle.inward[1], 0);
    const sign = Math.sign(new THREE.Vector3().crossVectors(along, up).dot(inward));
    const faces = toggle.faces.map((color, index) => {
      const angle = toggle.face_angles_deg[index] * Math.PI / 180;
      return { color, normal: inward.clone().multiplyScalar(Math.cos(angle)).addScaledVector(up, Math.sin(angle)) };
    });
    const turned = [];
    for (const number of solids.cad.toggle_parts) {
      for (const instance of cad.layout.instances[number]) {
        if (nearest(solids.toggles, position(instance), (t) => t.axle) !== toggle) continue;
        const rest = toPhysicsAxle.clone().multiply(cad.placement(instance));
        if (number === solids.cad.toggle_bar) {
          const looks = toggle.faces.map((color) => material(COLORS[color])).concat([material(COLORS.toggleEnd)]);
          const geometry = barGeometry(cad, rest, faces.map((face) => face.normal), along);
          turned.push({ mesh: placedMesh(geometry, new THREE.Matrix4(), looks), rest: new THREE.Matrix4() });
        } else if (turning.has(number)) {
          turned.push({ mesh: placedMesh(cad.geometry(number), rest, material(COLORS.toggleEnd)), rest });
        } else {
          placedMesh(cad.geometry(number), rest, material(COLORS.metal, { metalness: 0.5 }));
        }
      }
    }
    const intoTheField = inward.clone().multiplyScalar(Math.cos(Math.PI / 6)).addScaledVector(up, Math.sin(Math.PI / 6));
    const slot = direction(toggle.inward[0] * toggle.slot[0], toggle.inward[1] * toggle.slot[0], toggle.slot[1]);
    toggleMeshes.push({ quadrant: toggle.quadrant, axle, along, sign, turned, faces, intoTheField, slot,
      turn: new THREE.Quaternion() });
  }
}

function shownInnerColor(toggle) {
  const facing = (face) => face.normal.clone().applyQuaternion(toggle.turn).dot(toggle.intoTheField);
  return toggle.faces.reduce((best, face) => (facing(face) > facing(best) ? face : best)).color;
}

function pieceMaker(cad) {
  const halves = {};
  for (const kind of ["pin", "cup"]) {
    halves[kind] = solids.cad.halves[kind].map(({ rotation, offset_in }) =>
      partGeometry(cad, solids.cad[`${kind}_half`], FROM_PIECE.clone().multiply(rotationAndOffset(rotation, offset_in))));
  }
  const looks = new Map([["clear", material(COLORS.clear, { transparent: true, opacity: 0.4, depthWrite: false })]]);
  const look = (color) => {
    if (!looks.has(color)) looks.set(color, material(COLORS[color]));
    return looks.get(color);
  };
  return (kind, colors) => {
    const group = new THREE.Group();
    halves[kind].forEach((geometry, end) => group.add(new THREE.Mesh(geometry, look(colors[end]))));
    return group;
  };
}

let makePiece = null;

function showPieces(pieces) {
  const seen = new Set();
  for (const piece of pieces) {
    if (!piece.pose) continue;
    let mesh = pieceMeshes.get(piece.id);
    if (!mesh) {
      mesh = makePiece(piece.kind, piece.colors);
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

// An outline in the robot frame, with an optional hole, extruded from
// `bottom` to `top`.
function extruded(outline, bottom, top, color, hole = null) {
  const toShape = (points, Kind) => new Kind(points.map(([forward, left]) => new THREE.Vector2(-left, forward)));
  const shape = toShape(outline, THREE.Shape);
  if (hole) shape.holes.push(toShape(hole, THREE.Path));
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: top - bottom, bevelEnabled: false });
  geometry.rotateX(-Math.PI / 2);
  geometry.translate(0, bottom, 0);
  return new THREE.Mesh(geometry, material(color, { side: THREE.DoubleSide }));
}

// A piece seen from the side, (forward, up), reaching from `right` to `left`.
function sideProfile(part, color) {
  const shape = new THREE.Shape(part.profile.map(([forward, up]) => new THREE.Vector2(forward, up)));
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: part.left - part.right, bevelEnabled: false });
  geometry.rotateY(Math.PI / 2);
  geometry.translate(-part.left, 0, 0);
  return new THREE.Mesh(geometry, material(color, { side: THREE.DoubleSide }));
}

function restingBox(part, color) {
  const geometry = new THREE.BoxGeometry(part.left - part.right, part.top - part.bottom, part.front - part.back);
  geometry.translate(-(part.left + part.right) / 2, (part.top + part.bottom) / 2, -(part.front + part.back) / 2);
  return new THREE.Mesh(geometry, material(color));
}

function pieceColor(part, stageShares) {
  if (part.jaw_side || part.name === "carriage") return COLORS.robot;
  if (part.name === "roller flap") return COLORS.rubber;
  if (part.name === "lift") return stageShares.indexOf(part.rise_share) % 2 ? COLORS.metal : COLORS.rail;
  return COLORS.metal;
}

// The pieces come from vexsim with the lift down and the claw open (see
// cascade_drawing() in fields.py), each drawn the size it collides at. The
// frame is drawn as FRC-style bumpers around its outline, into the bite,
// and the frame inside them; the wheels are hidden inside. Posing moves
// each piece as vexsim moves it: up by its share of the claw's rise, a jaw
// toward the middle as the claw closes, and a roller piece around each of
// its hinges.
function buildRobot(lengthIn, widthIn) {
  const shape = drawing.robot;
  const group = new THREE.Group();
  const frame = shape.frame;
  const bumpers = extruded(frame.outline, frame.bottom, frame.top, COLORS.robot, frame.bumper_inside);
  const inside = extruded(frame.bumper_inside, frame.bottom, frame.top, COLORS.chassis);
  group.add(bumpers, inside);
  const stageShares = [...new Set(shape.parts.filter((part) => part.name === "lift").map((part) => part.rise_share))].sort();
  const pieces = [];
  for (const part of shape.parts) {
    if (part.name === "drivetrain") continue;
    const color = pieceColor(part, stageShares);
    const mesh = part.profile.length ? sideProfile(part, color) : restingBox(part, color);
    mesh.matrixAutoUpdate = false;
    group.add(mesh);
    pieces.push({ part, mesh });
  }
  scene.add(group);
  return { group, lengthIn, widthIn, frame: [bumpers, inside], pieces };
}

// Turns `degrees` about a hinge across the robot. Positive takes what is
// behind the hinge up, as in vexsim; the 3D view's z points backward.
function hingeTurn(hinge, degrees) {
  const pivot = new THREE.Vector3(0, hinge.up_in, -hinge.forward_in);
  return new THREE.Matrix4().makeTranslation(pivot.x, pivot.y, pivot.z)
    .multiply(new THREE.Matrix4().makeRotationX(-degrees * Math.PI / 180))
    .multiply(new THREE.Matrix4().makeTranslation(-pivot.x, -pivot.y, -pivot.z));
}

function poseRobot(frame) {
  const claw = drawing.robot.claw;
  const closing = (claw.open_gap_in - frame.claw_gap_in) / 2;
  const stageRises = [0, ...frame.stage_rises_in];
  const turns = { roller: frame.roller_deg, ...frame.flap_bends_deg };
  for (const { part, mesh } of robot.pieces) {
    mesh.matrix.makeTranslation(part.jaw_side * closing, stageRises[part.stage], 0);
    for (const hinge of part.hinges) mesh.matrix.multiply(hingeTurn(hinge, turns[hinge.name]));
  }
  robot.group.position.copy(at(frame.true_x, frame.true_y, 0));
  robot.group.rotation.y = -frame.true_heading * Math.PI / 180;
}

// Each drawn piece's box in the robot frame, inches, as posed now.
function robotFrameBox(mesh) {
  const box = new THREE.Box3();
  const corner = new THREE.Vector3();
  const positions = mesh.geometry.attributes.position;
  for (let i = 0; i < positions.count; i++) box.expandByPoint(corner.fromBufferAttribute(positions, i).applyMatrix4(mesh.matrix));
  return { back: -box.max.z, front: -box.min.z, right: -box.max.x, left: -box.min.x, bottom: box.min.y, top: box.max.y };
}

window.drawnRobotPieces = () => robot && {
  frame: robot.frame.map(robotFrameBox),
  pieces: robot.pieces.map(({ part, mesh }) => ({ key: part.key, ...robotFrameBox(mesh) })),
};

function showToggles(frame) {
  for (const toggle of toggleMeshes) {
    const angle = frame?.toggles?.[toggle.quadrant]?.angle_deg ?? 0;
    const raised = toggle.slot.clone().multiplyScalar(frame?.toggles?.[toggle.quadrant]?.lift_in ?? 0);
    toggle.turn.setFromAxisAngle(toggle.along, -toggle.sign * angle * Math.PI / 180);
    const aboutTheAxle = new THREE.Matrix4().makeTranslation(toggle.axle.clone().add(raised))
      .multiply(new THREE.Matrix4().makeRotationFromQuaternion(toggle.turn))
      .multiply(new THREE.Matrix4().makeTranslation(toggle.axle.clone().negate()));
    for (const { mesh, rest } of toggle.turned) mesh.matrix.copy(aboutTheAxle.clone().multiply(rest));
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
      stage_rises_in: frame.stage_rises_in, claw_gap_in: frame.claw_gap_in, roller_deg: frame.roller_deg,
      flap_bends_deg: frame.flap_bends_deg, raised_loaders: frame.raised_loaders },
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
  const cad = await loadCad();
  makePiece = pieceMaker(cad);
  addFloor(drawing);
  addWalls(drawing);
  addGoals(cad);
  addLoaders(cad);
  addToggles(cad);
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
