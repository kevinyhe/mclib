"use strict";

// The page's copy of geometry.py: where each step leaves the robot and the
// curve of each path. test_geometry.py runs both and compares them.
// Inches, +X right, +Y forward; degrees, 0 facing +Y, clockwise positive.

const AUTO_HANDLE_FRACTION = 0.4;
const MIN_POINT_GAP_IN = 0.5;

function wrapDegrees(angle) {
  return ((angle + 180) % 360 + 360) % 360 - 180;
}

function bearing(a, b) {
  return Math.atan2(b.x - a.x, b.y - a.y) * 180 / Math.PI;
}

function along(heading, length) {
  const radians = heading * Math.PI / 180;
  return { x: length * Math.sin(radians), y: length * Math.cos(radians) };
}

function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function pathLayout(step, start) {
  const anchors = [{ x: start.x, y: start.y }, ...step.points.map((p) => ({ x: p.x, y: p.y }))];
  const count = step.points.length;
  const travel = step.sections.map((section) => (section.reverse ? 180 : 0));
  const headings = [start.heading];
  for (let i = 1; i <= count; i++) {
    const point = step.points[i - 1];
    if (point.heading !== null && point.heading !== undefined) {
      headings.push(point.heading);
      continue;
    }
    const offset = travel[i - 1];
    const smooth = i < count && travel[i] === offset && distance(anchors[i - 1], anchors[i + 1]) >= MIN_POINT_GAP_IN;
    const tangent = smooth ? bearing(anchors[i - 1], anchors[i + 1]) : bearing(anchors[i - 1], anchors[i]);
    headings.push(wrapDegrees(tangent - offset));
  }
  const handleOut = [step.start_handle ?? null, ...step.points.map((p) => p.handle_out ?? null)];
  const handleIn = [null, ...step.points.map((p) => p.handle_in ?? null)];
  const sections = [];
  for (let i = 1; i <= count; i++) {
    const a = anchors[i - 1], b = anchors[i];
    const chord = distance(a, b);
    const outLength = handleOut[i - 1] ?? AUTO_HANDLE_FRACTION * chord;
    const inLength = handleIn[i] ?? AUTO_HANDLE_FRACTION * chord;
    const out = along(headings[i - 1] + travel[i - 1], outLength);
    const into = along(headings[i] + travel[i - 1], inLength);
    sections.push({
      start: a, startHandle: { x: a.x + out.x, y: a.y + out.y },
      endHandle: { x: b.x - into.x, y: b.y - into.y }, end: b,
      reverse: step.sections[i - 1].reverse, speedPercent: step.sections[i - 1].speed_percent,
      handleLengths: [outLength, inLength],
    });
  }
  return { headings, sections };
}

function bezierPoint(section, t) {
  const u = 1 - t;
  const a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
  return {
    x: a * section.start.x + b * section.startHandle.x + c * section.endHandle.x + d * section.end.x,
    y: a * section.start.y + b * section.startHandle.y + c * section.endHandle.y + d * section.end.y,
  };
}

function sectionSamples(section, count = 48) {
  const points = [];
  for (let i = 0; i <= count; i++) points.push(bezierPoint(section, i / count));
  return points;
}

function pathSpacingProblem(step, start) {
  const anchors = [{ x: start.x, y: start.y }, ...step.points];
  for (let i = 1; i < anchors.length; i++) {
    const gap = distance(anchors[i - 1], anchors[i]);
    if (gap < MIN_POINT_GAP_IN) {
      const before = i === 1 ? "where the robot starts this step" : `point ${i - 1}`;
      return `point ${i} is ${gap.toFixed(2)} in from ${before}. Points must be at least ${MIN_POINT_GAP_IN} in apart`;
    }
  }
  return null;
}

function arcPoints(pose, endHeading, signedRadius, samples = 40) {
  const delta = wrapDegrees(endHeading - pose.heading);
  const heading = pose.heading * Math.PI / 180;
  const cx = pose.x + Math.cos(heading) * signedRadius;
  const cy = pose.y - Math.sin(heading) * signedRadius;
  const points = [];
  for (let part = 1; part <= samples; part++) {
    const angle = (pose.heading + delta * part / samples) * Math.PI / 180;
    points.push({ x: cx - Math.cos(angle) * signedRadius, y: cy + Math.sin(angle) * signedRadius });
  }
  return points;
}

function swingRadius(step, pose, trackWidth) {
  const delta = wrapDegrees(step.heading - pose.heading);
  return delta ? Math.sign(delta) * step.direction * trackWidth / 2 : 0;
}

function stepEndPose(step, pose, trackWidth) {
  switch (step.type) {
    case "drive": {
      const move = along(pose.heading, step.distance);
      return { x: pose.x + move.x, y: pose.y + move.y, heading: pose.heading };
    }
    case "turn": return { x: pose.x, y: pose.y, heading: step.heading };
    case "lift":
    case "claw":
    case "match_load":
      return { ...pose };
    case "turn_to_point":
      return { x: pose.x, y: pose.y, heading: wrapDegrees(bearing(pose, step) + (step.direction === -1 ? 180 : 0)) };
    case "point":
      return { x: step.x, y: step.y, heading: wrapDegrees(bearing(pose, step) + (step.direction === -1 ? 180 : 0)) };
    case "boomerang":
    case "wall_reset":
      return { x: step.x, y: step.y, heading: step.heading };
    case "arc":
    case "reverse_arc":
    case "swing": {
      const radius = step.type === "swing" ? swingRadius(step, pose, trackWidth) : step.radius;
      const end = arcPoints(pose, step.heading, radius).at(-1);
      return { x: end.x, y: end.y, heading: step.heading };
    }
    case "path": {
      if (!step.points.length) return { ...pose };
      const last = step.points.at(-1);
      return { x: last.x, y: last.y, heading: pathLayout(step, pose).headings.at(-1) };
    }
    default: throw new Error(`unknown step type ${step.type}`);
  }
}

// Start and end pose of every step, plus a message for any path whose
// points are too close together (the server refuses those).
function designedPoses(project, trackWidth) {
  let pose = { ...project.start };
  return project.steps.map((step) => {
    const problem = step.type === "path" ? pathSpacingProblem(step, pose) : null;
    const end = stepEndPose(step, pose, trackWidth);
    const item = { start: pose, end, problem };
    pose = end;
    return item;
  });
}

// What to draw for a step that is not a path: a line or arc of where it is
// meant to go. Boomerang is drawn as a curve toward its approach point; the
// real route depends on the controller.
function stepPreview(step, pose, trackWidth) {
  const end = stepEndPose(step, pose, trackWidth);
  switch (step.type) {
    case "drive": case "point": case "wall_reset":
      return [{ x: pose.x, y: pose.y }, { x: end.x, y: end.y }];
    case "arc": case "reverse_arc": case "swing": {
      const radius = step.type === "swing" ? swingRadius(step, pose, trackWidth) : step.radius;
      return [{ x: pose.x, y: pose.y }, ...arcPoints(pose, step.heading, radius)];
    }
    case "boomerang": {
      const reach = distance(pose, step) * step.lead;
      const approach = along(step.heading + (step.direction === -1 ? 180 : 0), reach);
      const control = { x: step.x - approach.x, y: step.y - approach.y };
      const points = [];
      for (let i = 0; i <= 32; i++) {
        const t = i / 32, u = 1 - t;
        points.push({ x: u * u * pose.x + 2 * u * t * control.x + t * t * step.x,
          y: u * u * pose.y + 2 * u * t * control.y + t * t * step.y });
      }
      return points;
    }
    default: return [];
  }
}

function mirrorHeading(heading, axis) {
  return axis === "x" ? -heading : 180 - heading;
}

// The routine for the other side of the field. "x" swaps left and right
// (x becomes -x), "y" swaps front and back (y becomes -y). Arcs bend the
// other way, so their radius changes sign.
function mirrorProject(project, axis) {
  const copy = JSON.parse(JSON.stringify(project));
  const flip = (item) => {
    if (axis === "x" && typeof item.x === "number") item.x = -item.x;
    if (axis === "y" && typeof item.y === "number") item.y = -item.y;
    if (typeof item.heading === "number") item.heading = wrapDegrees(mirrorHeading(item.heading, axis));
  };
  flip(copy.start);
  for (const step of copy.steps) {
    if (step.type === "path") step.points.forEach(flip);
    else flip(step);
    if (typeof step.radius === "number") step.radius = -step.radius;
  }
  return copy;
}

function frameIndexAt(trace, time) {
  if (!trace.length) return -1;
  let low = 0, high = trace.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (trace[mid].t <= time + 1e-9) low = mid; else high = mid - 1;
  }
  return low;
}

// The planned state at time @p t along one plan (states from the server:
// part, t, s, x, y, heading, speed, curvature), interpolated.
function planStateAt(states, t) {
  if (!states.length) return null;
  if (t <= states[0][1]) return states[0];
  if (t >= states.at(-1)[1]) return states.at(-1);
  let low = 0, high = states.length - 1;
  while (high - low > 1) {
    const mid = (low + high) >> 1;
    if (states[mid][1] <= t) low = mid; else high = mid;
  }
  const a = states[low], b = states[high];
  const f = b[1] > a[1] ? (t - a[1]) / (b[1] - a[1]) : 0;
  return [a[0], t, a[2] + (b[2] - a[2]) * f, a[3] + (b[3] - a[3]) * f, a[4] + (b[4] - a[4]) * f,
    a[5] + wrapDegrees(b[5] - a[5]) * f, a[6] + (b[6] - a[6]) * f, a[7]];
}

function unwrapSeries(values) {
  let previous = null;
  return values.map((value) => {
    if (!Number.isFinite(value)) return null;
    const angle = previous === null ? value : previous + wrapDegrees(value - previous);
    previous = angle;
    return angle;
  });
}

// Telemetry rows for one recorded frame. Missing values show as a dash,
// never as zero.
function telemetryRows(frame, target) {
  const f = frame || {};
  const num = (value, unit = "", digits = 1) => (Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : "—");
  const pair = (a, b, unit = "", digits = 1) => ([a, b].some(Number.isFinite) ? `${num(a, "", digits)} / ${num(b, "", digits)}${unit}` : "—");
  const pose = (x, y, h) => ([x, y, h].some(Number.isFinite) ? `${num(x)}, ${num(y)} in · ${num(h, "°")}` : "—");
  const mode = (value) => ({ 0: "voltage", 1: "coast", 2: "brake", 3: "hold" })[value] || "—";
  const flag = (value) => (value === true ? "yes" : value === false ? "no" : "—");
  const miss = frame && target && [target.x, target.y, f.true_x, f.true_y].every(Number.isFinite)
    ? Math.hypot(target.x - f.true_x, target.y - f.true_y) : null;
  const turnMiss = frame && target && [target.heading, f.true_heading].every(Number.isFinite)
    ? Math.abs(wrapDegrees(target.heading - f.true_heading)) : null;
  return [
    ["Position", [
      ["Simulated robot", pose(f.true_x, f.true_y, f.true_heading)],
      ["Odometry", pose(f.estimated_x, f.estimated_y, f.estimated_heading)],
      ["Odometry off by", f.coordinate_reset ? "frame moved by wall reset" : num(f.odometry_error_in, " in", 2)],
      ["Distance to step target", num(miss, " in", 2)],
      ["Heading off target", num(turnMiss, "°", 1)],
    ]],
    ["Motion", [
      ["Forward / sideways speed", pair(f.forward_speed_ips, f.lateral_speed_ips, " in/s")],
      ["Turn rate", num(f.yaw_rate_dps, " °/s")],
    ]],
    ...("contact" in f ? [["Field contact", [
      ["Touching", f.contact ?? "nothing"],
      ["Contact force", num(f.contact_force_n, " N", 0)],
    ]]] : []),
    ...("lift_height_in" in f ? [["Lift and claw", [
      ["Claw height", num(f.lift_height_in, " in", 2)],
      ["Lift motors", `${num(f.lift_deg, "°", 0)} · ${num(f.lift_amps, " A", 2)}`],
      ["Claw valve", f.claw_closed ? "closed" : "open"],
      ["Gap between the jaws", num(f.claw_gap_in, " in", 2)],
      ["Squeeze on the object", num(f.grip_force_n, " N", 1)],
      ["Holding", f.holding ?? "nothing"],
      ["Score red / blue", pair(f.score_red, f.score_blue, "", 0)],
    ]]] : []),
    ["Controller", [
      ["Phase", typeof f.controller_phase === "string" ? f.controller_phase : "—"],
      ["Target heading / error", pair(f.controller_target_heading_deg, f.controller_heading_error_deg, "°")],
      ["Distance left", num(f.controller_remaining_in, " in", 2)],
      ["Aim point", pair(f.controller_carrot_x, f.controller_carrot_y, " in", 1)],
      ["Drive / turn request", pair(f.controller_drive_volts, f.controller_yaw_volts, " V", 2)],
      ["Slew / voltage limited", f.controller_slew_limited === undefined ? "—" : `${flag(f.controller_slew_limited)} / ${flag(f.controller_voltage_limited)}`],
    ]],
    ["Motors and battery", [
      ["Left / right voltage", pair(f.left_volts, f.right_volts, " V", 2)],
      ["Left / right mode", frame ? `${mode(f.left_mode)} / ${mode(f.right_mode)}` : "—"],
      ["Left / right current", pair(f.left_current_amps, f.right_current_amps, " A", 2)],
      ["Hottest motor L / R", pair(f.left_temp_c, f.right_temp_c, " °C")],
      ["Battery", [f.battery_volts, f.battery_amps].some(Number.isFinite) ? `${num(f.battery_volts, " V", 2)} · ${num(f.battery_amps, " A")}` : "—"],
    ]],
    ["Sensors", [
      ["IMU heading / rate", [f.imu_heading_deg, f.imu_rate_dps].some(Number.isFinite) ? `${num(f.imu_heading_deg, "°")} · ${num(f.imu_rate_dps, " °/s")}` : "—"],
      ["Encoders L / R", pair(f.left_encoder_deg, f.right_encoder_deg, "°", 0)],
      ["Encoder speed L / R", pair(f.left_encoder_rpm, f.right_encoder_rpm, " RPM", 0)],
      ["Tracking wheels fwd / side", pair(f.parallel_tracker_in, f.perpendicular_tracker_in, " in", 2)],
    ]],
  ];
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = {
    wrapDegrees, bearing, along, pathLayout, bezierPoint, sectionSamples, pathSpacingProblem, arcPoints,
    stepEndPose, designedPoses, stepPreview, mirrorProject, frameIndexAt, planStateAt, unwrapSeries, telemetryRows,
  };
}
