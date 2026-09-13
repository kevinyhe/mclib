"use strict";

// Geometric intent only. Recorded robot movement always comes from the backend.
// Kept independent of the DOM so its coordinate conventions can be checked directly.
function motionReferencePath(spec, trackWidthIn) {
  const radians = (angle) => angle * Math.PI / 180;
  const wrap = (angle) => ((angle + 180) % 360 + 360) % 360 - 180;
  const trackWidth = trackWidthIn ?? (spec.preset === "speed_base" ? 11 : 11.5);
  let pose = { ...spec.start_pose };
  const paths = [];
  spec.steps.forEach((step, index) => {
    const start = { ...pose };
    const points = [{ x: pose.x, y: pose.y }];
    let target = null;
    if (step.type === "drive") {
      pose.x += Math.sin(radians(pose.heading)) * step.distance;
      pose.y += Math.cos(radians(pose.heading)) * step.distance;
    } else if (step.type === "turn") {
      pose.heading = step.heading;
    } else if (step.type === "turn_to_point") {
      pose.heading = Math.atan2(step.x - pose.x, step.y - pose.y) * 180 / Math.PI + (step.direction === -1 ? 180 : 0);
      target = { x: step.x, y: step.y };
    } else if (["arc", "reverse_arc", "swing"].includes(step.type)) {
      const delta = wrap(step.heading - pose.heading);
      // A swing holds one tread. The robot centre traces half the track width
      // around that tread; reversing travel changes which side remains fixed.
      const signedRadius = step.type === "swing"
        ? Math.sign(delta) * (step.direction ?? 1) * trackWidth / 2
        : step.radius;
      const heading = radians(pose.heading);
      const cx = pose.x + Math.cos(heading) * signedRadius;
      const cy = pose.y - Math.sin(heading) * signedRadius;
      for (let part = 1; part <= 40; part++) {
        const angle = radians(pose.heading + delta * part / 40);
        points.push({ x: cx - Math.cos(angle) * signedRadius, y: cy + Math.sin(angle) * signedRadius });
      }
      pose.x = points.at(-1).x;
      pose.y = points.at(-1).y;
      pose.heading = step.heading;
    } else {
      if (step.type === "point") pose.heading = Math.atan2(step.x - pose.x, step.y - pose.y) * 180 / Math.PI + (step.direction === -1 ? 180 : 0);
      pose.x = step.x;
      pose.y = step.y;
      if (step.heading !== undefined) pose.heading = step.heading;
    }
    if (points.length === 1) points.push({ x: pose.x, y: pose.y });
    paths.push({ index, type: step.type, start, end: { ...pose }, points, target: target || { ...pose } });
  });
  return paths;
}

// A result-row inspection is an exact frame selection, not a time seek. At a
// handoff, several frames share a timestamp; the following action owns the last.
const MOTION_TIME_EPSILON = 1e-9;

function motionTimelineMax(time) {
  // HTML range values must land on their 1 ms step grid. Floating integration
  // can put 3.570 s just below 3.57 and otherwise make the final frame unreachable.
  return Math.max(.001, Math.ceil((time - MOTION_TIME_EPSILON) * 1000) / 1000);
}

function motionFrameAt(trace, time, inspection = null) {
  if (inspection) {
    if (inspection.executed === false) return null;
    for (let index = trace.length - 1; index >= 0; index--) {
      if (trace[index].step === inspection.index) return trace[index];
    }
    return null;
  }
  if (!trace.length) return null;
  let low = 0, high = trace.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (trace[mid].t <= time + MOTION_TIME_EPSILON) low = mid; else high = mid - 1;
  }
  return trace[low];
}

function motionVisibleTrace(trace, time, inspection = null) {
  const frame = motionFrameAt(trace, time, inspection);
  return frame ? trace.slice(0, trace.indexOf(frame) + 1) : [];
}

function motionRecording(job) {
  if (["queued", "running"].includes(job?.status)) {
    return { ...(job.partial || {}), live: true, incomplete: false,
      trace: Array.isArray(job.partial?.trace) ? job.partial.trace : [],
      steps: Array.isArray(job.partial?.steps) ? job.partial.steps : [] };
  }
  if (!job?.result && ["cancelled", "error"].includes(job?.status)) {
    return { ...(job.partial || {}), live: false, incomplete: true, passed: null,
      trace: Array.isArray(job.partial?.trace) ? job.partial.trace : [],
      steps: Array.isArray(job.partial?.steps) ? job.partial.steps : [] };
  }
  return job?.result || { trace: [], steps: [] };
}

function motionAcceptance(recording) {
  return recording.incomplete ? "INCOMPLETE" : recording.live ? "RUNNING" : recording.passed === true ? "PASS" : recording.passed === false ? "FAIL" : "UNAVAILABLE";
}

function motionStepStatus(step, recording) {
  return step.executed === false ? "SKIPPED" : typeof step.passed !== "boolean" ? (recording.incomplete ? "INTERRUPTED" : "RUNNING") : step.passed ? "PASS" : "FAIL";
}

function motionUnwrapSeries(trace, key) {
  let previous = null;
  return trace.map(frame => {
    const value = frame[key];
    if (!Number.isFinite(value)) return null;
    const angle = previous === null ? value : previous + ((value - previous + 180) % 360 + 360) % 360 - 180;
    previous = angle;
    return angle;
  });
}

function motionTraceBounds(trace) {
  let bounds = null;
  for (const frame of trace) {
    if (![frame.true_x, frame.true_y].every(Number.isFinite)) continue;
    if (!bounds) bounds = { min_x: frame.true_x, max_x: frame.true_x, min_y: frame.true_y, max_y: frame.true_y };
    else {
      bounds.min_x = Math.min(bounds.min_x, frame.true_x); bounds.max_x = Math.max(bounds.max_x, frame.true_x);
      bounds.min_y = Math.min(bounds.min_y, frame.true_y); bounds.max_y = Math.max(bounds.max_y, frame.true_y);
    }
  }
  return bounds;
}

// Display only supplied measurements or errors against a recorded target.
// In particular, absent legacy sensor/controller fields must never become zero.
function motionTelemetry(frame, target) {
  const number = (value, unit = "", digits = 1) => Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : "—";
  const pair = (a, b, unit = "", digits = 1) => [a, b].some(Number.isFinite) ? `${number(a, "", digits)} / ${number(b, "", digits)}${unit}` : "—";
  const pose = (x, y, heading) => [x, y, heading].some(Number.isFinite) ? `${number(x)} / ${number(y)} / ${number(heading, "°")}` : "—";
  const flag = value => value === true || value === 1 ? "yes" : value === false || value === 0 ? "no" : "—";
  const mode = value => ({ 0: "voltage", 1: "coast", 2: "brake", 3: "hold" })[value] || "—";
  const f = frame || {};
  const positionError = frame && [target?.x, target?.y, f.true_x, f.true_y].every(Number.isFinite)
    ? Math.hypot(target.x - f.true_x, target.y - f.true_y) : null;
  const headingError = frame && [target?.heading, f.true_heading].every(Number.isFinite)
    ? Math.abs(((target.heading - f.true_heading + 180) % 360 + 360) % 360 - 180) : null;
  return {
    "truth-pose": pose(f.true_x, f.true_y, f.true_heading),
    "estimated-pose": pose(f.estimated_x, f.estimated_y, f.estimated_heading),
    "recorded-target": frame ? pose(target?.x, target?.y, target?.heading) : "—",
    "target-error": f.coordinate_reset ? "FRAME RESET" : Number.isFinite(positionError) ? number(positionError, " in", 2) : number(headingError, "°", 2),
    "final-heading-error": f.coordinate_reset ? "FRAME RESET" : number(headingError, "°", 2),
    "odometry-gap": f.coordinate_reset ? "FRAME RESET" : number(f.odometry_error_in, " in", 2),
    "body-speed": pair(f.forward_speed_ips, f.lateral_speed_ips, " in/s"),
    "yaw-rate": number(f.yaw_rate_dps, " °/s"),
    "motor-voltage": pair(f.left_volts, f.right_volts, " V"),
    "command-mode": frame ? `${mode(f.left_mode)} / ${mode(f.right_mode)}` : "—",
    "imu-reading": [f.imu_heading_deg, f.imu_rate_dps].some(Number.isFinite) ? `${number(f.imu_heading_deg, "°")} / ${number(f.imu_rate_dps, " °/s")}` : "—",
    "encoder-angle": pair(f.left_encoder_deg, f.right_encoder_deg, "°"),
    "encoder-speed": pair(f.left_encoder_rpm, f.right_encoder_rpm, " RPM"),
    "tracker-distance": pair(f.parallel_tracker_in, f.perpendicular_tracker_in, " in", 2),
    "battery-reading": [f.battery_volts, f.battery_amps].some(Number.isFinite) ? `${number(f.battery_volts, " V")} / ${number(f.battery_amps, " A")}` : "—",
    "motor-current": pair(f.left_current_amps, f.right_current_amps, " A", 2),
    "motor-temperature": pair(f.left_temp_c, f.right_temp_c, " °C"),
    "controller-phase": typeof f.controller_phase === "string" && f.controller_phase ? f.controller_phase : "—",
    "controller-heading": pair(f.controller_target_heading_deg, f.controller_heading_error_deg, "°"),
    "controller-remaining": number(f.controller_remaining_in, " in", 2),
    "controller-carrot": pair(f.controller_carrot_x, f.controller_carrot_y, " in", 2),
    "controller-demand": pair(f.controller_drive_volts, f.controller_yaw_volts, " V", 2),
    "controller-limits": [f.controller_slew_limited, f.controller_voltage_limited].some(value => [true, false, 0, 1].includes(value)) ? `${flag(f.controller_slew_limited)} / ${flag(f.controller_voltage_limited)}` : "—",
  };
}

if (typeof module !== "undefined" && module.exports) module.exports = { motionReferencePath, motionFrameAt, motionVisibleTrace, motionTimelineMax, motionRecording, motionAcceptance, motionStepStatus, motionTelemetry, motionUnwrapSeries, motionTraceBounds };

(() => {
  if (typeof document === "undefined") return;
  const $ = (id) => document.getElementById(id);
  const STORAGE_KEY = "mclib.motion-builder.v1";
  const TYPE_INFO = {
    drive: { label: "Drive distance", description: "Travel a relative distance along the current heading.", fields: ["distance"], defaults: { distance: 24 } },
    turn: { label: "Turn to angle", description: "Turn in place to an absolute field heading.", fields: ["heading"], defaults: { heading: 90 } },
    point: { label: "Move to point", description: "Drive to a field coordinate. Click or drag its target on the field.", fields: ["x", "y", "direction"], defaults: { x: 24, y: 24, direction: 1 } },
    turn_to_point: { label: "Turn to point", description: "Face a field coordinate without driving toward it.", fields: ["x", "y", "direction"], defaults: { x: 24, y: 24, direction: 1 } },
    boomerang: { label: "Boomerang", description: "Approach a field coordinate with a specified final heading.", fields: ["x", "y", "heading", "direction", "lead"], defaults: { x: 24, y: 24, heading: 90, direction: 1, lead: 0.5 } },
    arc: { label: "Arc (auto direction)", description: "Follow a constant-radius arc to an absolute heading. The radius sign picks the arc side; travel direction is automatic.", fields: ["heading", "radius"], defaults: { heading: 90, radius: 24 } },
    reverse_arc: { label: "Reverse arc", description: "Reverse along a constant-radius arc. The signed radius and target heading must describe reverse travel from the entry pose.", fields: ["heading", "radius"], defaults: { heading: -90, radius: 24 } },
    swing: { label: "Swing turn", description: "Turn around one stationary side of the drivetrain.", fields: ["heading", "direction"], defaults: { heading: 90, direction: 1 } },
    wall_reset: { label: "Wall reset", description: "Detect a sustained motor stall, then reset the estimated pose. Enable locked motors to model an ideal stall.", fields: ["x", "y", "heading", "current_ma"], defaults: { x: 0, y: 0, heading: 0, current_ma: 2500 } },
  };
  const FIELD_INFO = {
    x: ["X / in", -500, 500, 0.5], y: ["Y / in", -500, 500, 0.5], heading: ["Heading / deg", -3600, 3600, 1],
    distance: ["Distance / in", -500, 500, 1], radius: ["Signed radius / in", -250, 250, 0.5],
    timeout_ms: ["Timeout / ms", 50, 20000, 50], volts: ["Max output / V", 0.1, 12, 0.1],
    lead: ["Lead / ratio", 0, 0.99, 0.01], current_ma: ["Stall current / mA", 0, 10000, 50],
    soc: ["Battery / fraction", 0, 1, 0.05], friction: ["Friction / μ", 0.05, 2, 0.05], seed: ["Sensor noise seed", 0, 2147483647, 1],
  };
  const FALLBACK = {
    name: "Untitled autonomous", preset: "six_motor_450", tracking_mode: "two",
    start_pose: { x: 0, y: 0, heading: 0 }, environment: { soc: 1, friction: 1, encoder_heading: false, seed: 1 },
    tuning: { drive_kp: 0.4, drive_ki: 0, drive_kd: 3, heading_kp: 0.3, heading_ki: 0, heading_kd: 1.5, turn_kp: 0.3, turn_ki: 0, turn_kd: 1.5 },
    stop_on_failure: true,
    steps: [{ type: "drive", distance: 24, timeout_ms: 4000, volts: 12 }, { type: "turn", heading: 90, timeout_ms: 4000, volts: 12 }, { type: "point", x: 24, y: 24, direction: 1, timeout_ms: 4000, volts: 12 }],
  };
  const state = {
    config: null, spec: clone(FALLBACK), selected: 0, jobs: [], job: null, compare: null, activeId: null,
    trace: [], time: 0, playing: false, speed: 1, lastFrame: 0, dirty: false, connected: false,
    live: false, incomplete: false, inspection: null, viewMode: "edit", nativeReady: false, nativeError: "", nativeTimer: null,
    nativeUnavailable: "", nativeFitPending: null,
    historyRequest: 0, historyLoading: false, historyChoice: "", submitting: false,
    compareRequest: 0, compareLoading: false, compareChoice: "",
    view: { x: 0, y: 20, span: 100 }, transform: null, drag: null, saveTimer: null,
  };

  function clone(value) { return JSON.parse(JSON.stringify(value)); }
  function node(tag, text, className) { const element = document.createElement(tag); if (text !== undefined) element.textContent = text; if (className) element.className = className; return element; }
  function fmt(value, digits = 1) { return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—"; }
  function finite(value, fallback = 0) { return Number.isFinite(Number(value)) ? Number(value) : fallback; }
  function wrap(angle) { return ((angle + 180) % 360 + 360) % 360 - 180; }
  function radians(angle) { return angle * Math.PI / 180; }
  function selectedStep() { return state.spec.steps[state.selected]; }
  function labelFor(type) { return TYPE_INFO[type]?.label || type; }
  function notice(message = "") { $("notice").textContent = message; $("notice").hidden = !message; }
  function setConnection(connected) {
    state.connected = connected;
    $("connection").className = `connection ${connected ? "online" : "offline"}`;
    $("connection").lastElementChild.textContent = connected ? "LOCAL ENGINE CONNECTED" : "ENGINE UNAVAILABLE";
    $("run-sequence").disabled = !connected || !!state.activeId;
  }

  async function api(path, options = {}) {
    const response = await fetch(path, { ...options, headers: { "Content-Type": "application/json", ...options.headers } });
    let data;
    try { data = await response.json(); } catch { throw new Error(`Server returned ${response.status}; expected JSON.`); }
    if (!response.ok) throw new Error(typeof data.error === "string" ? data.error : data.message || `Request failed (${response.status}).`);
    return data;
  }

  function save() {
    state.dirty = true;
    $("save-status").textContent = "SAVING…";
    clearTimeout(state.saveTimer);
    state.saveTimer = setTimeout(() => {
      try { localStorage.setItem(STORAGE_KEY, JSON.stringify(state.spec)); $("save-status").textContent = "SAVED LOCALLY"; }
      catch { $("save-status").textContent = "LOCAL SAVE UNAVAILABLE"; }
    }, 200);
    updateFieldMode();
    updateReplayHighlight(frameAt(state.time));
  }

  function normalizeSpec(input) {
    if (!input || typeof input !== "object" || Array.isArray(input)) throw new Error("Import a routine JSON object.");
    const source = input.spec && typeof input.spec === "object" ? input.spec : input;
    const maxSteps = state.config?.limits?.max_steps || 16;
    if (!Array.isArray(source.steps) || source.steps.length === 0 || source.steps.length > maxSteps) throw new Error(`A routine must contain between 1 and ${maxSteps} motion steps.`);
    const defaults = state.config?.defaults || FALLBACK;
    const spec = clone(defaults);
    spec.name = typeof source.name === "string" ? source.name.slice(0, 80) : "Imported autonomous";
    if (typeof source.preset === "string") spec.preset = source.preset;
    if (source.tracking_mode !== undefined && !["drive", "two"].includes(source.tracking_mode)) throw new Error("Unknown odometry sensor configuration.");
    spec.tracking_mode = source.tracking_mode || defaults.tracking_mode;
    spec.stop_on_failure = source.stop_on_failure !== false;
    for (const key of ["x", "y", "heading"]) if (source.start_pose?.[key] !== undefined) spec.start_pose[key] = requireNumber(source.start_pose[key], `Start ${key}`);
    for (const key of ["soc", "friction", "seed"]) if (source.environment?.[key] !== undefined) spec.environment[key] = requireNumber(source.environment[key], key);
    for (const key of ["encoder_heading", "locked", "constrained"]) if (source.environment?.[key] !== undefined) spec.environment[key] = source.environment[key] === true;
    for (const key of Object.keys(FALLBACK.tuning)) if (source.tuning?.[key] !== undefined) spec.tuning[key] = requireNumber(source.tuning[key], key);
    spec.steps = source.steps.map((step, index) => {
      if (!step || !Object.hasOwn(TYPE_INFO, step.type)) throw new Error(`Step ${index + 1} has an unsupported motion type.`);
      const info = TYPE_INFO[step.type];
      const result = { type: step.type, ...info.defaults, timeout_ms: 4000 };
      if (step.type !== "turn_to_point") result.volts = step.type === "wall_reset" ? 6 : 12;
      for (const key of [...info.fields, "timeout_ms", ...(step.type !== "turn_to_point" ? ["volts"] : [])]) {
        if (step[key] !== undefined) result[key] = requireNumber(step[key], `Step ${index + 1} ${key}`);
      }
      if (result.direction !== undefined && ![1, -1].includes(result.direction)) throw new Error(`Step ${index + 1} direction must be 1 or -1.`);
      return result;
    });
    return spec;
  }

  function requireNumber(value, label) {
    if (typeof value !== "number" || !Number.isFinite(value)) throw new Error(`${label} must be a finite number.`);
    return value;
  }

  function makeNumber(key, value, onChange, customInfo) {
    const [label, min, max, step] = customInfo || FIELD_INFO[key] || [key, -10000, 10000, 0.1];
    const wrapper = node("label", label);
    const input = node("input"); input.type = "number"; input.value = value ?? 0; input.min = min; input.max = max; input.step = step; input.dataset.field = key;
    input.addEventListener("change", () => {
      const number = Number(input.value);
      if (input.value.trim() === "" || !Number.isFinite(number) || number < min || number > max || (key === "radius" && Math.abs(number) < 1) || (["seed", "timeout_ms"].includes(key) && !Number.isInteger(number))) {
        input.setCustomValidity(`Enter a number between ${min} and ${max}.`); input.reportValidity(); input.value = value ?? 0; input.setCustomValidity(""); return;
      }
      value = number; onChange(number);
    });
    wrapper.append(input); return wrapper;
  }

  function makeSelect(label, value, options, onChange) {
    const wrapper = node("label", label); const select = node("select");
    for (const [id, title] of options) { const option = node("option", title); option.value = id; select.append(option); }
    select.value = value; select.addEventListener("change", () => onChange(select.value)); wrapper.append(select); return wrapper;
  }

  function describeStep(step) {
    const timeout = `${fmt(step.timeout_ms / 1000, 1)} s`;
    if (step.type === "drive") return `${fmt(step.distance, 0)} in · ${timeout}`;
    if (["arc", "reverse_arc"].includes(step.type)) return `R ${fmt(step.radius, 0)} in → ${fmt(step.heading, 0)}° · ${timeout}`;
    if (["turn", "swing"].includes(step.type)) return `${fmt(step.heading, 0)}° · ${timeout}`;
    return `(${fmt(step.x, 0)}, ${fmt(step.y, 0)}) in${step.type === "boomerang" ? ` / ${fmt(step.heading, 0)}°` : ""} · ${timeout}`;
  }

  function renderSequence() {
    $("step-count").textContent = `${String(state.spec.steps.length).padStart(2, "0")} STEPS`;
    $("step-list").replaceChildren();
    state.spec.steps.forEach((step, index) => {
      const row = node("li", undefined, `step-row${index === state.selected ? " selected" : ""}`);
      row.dataset.step = index;
      const button = node("button", undefined, "step-select"); button.type = "button"; button.setAttribute("aria-pressed", String(index === state.selected)); button.setAttribute("aria-label", `Select step ${index + 1}, ${labelFor(step.type)}`);
      button.append(node("span", String(index + 1).padStart(2, "0"), "step-num"));
      const copy = node("span", undefined, "step-copy"); copy.append(node("strong", labelFor(step.type)), node("small", describeStep(step))); button.append(copy);
      const result = !state.dirty && motionRecording(state.job).steps?.find((item) => item.index === index);
      if (result && typeof result.passed === "boolean") button.append(node("span", result.executed === false ? "—" : result.passed ? "✓" : "!", `step-badge${result.executed !== false && !result.passed ? " failed" : ""}`));
      button.addEventListener("click", () => selectStep(index)); row.append(button);
      row.append(node("span", "", "replay-badge"));
      if (index === state.selected) {
        const actions = node("div", undefined, "step-actions");
        for (const [text, title, action, disabled] of [
          ["↑", "Move step up", () => moveStep(index, -1), index === 0],
          ["↓", "Move step down", () => moveStep(index, 1), index === state.spec.steps.length - 1],
          ["Copy", "Duplicate step", () => { state.spec.steps.splice(index + 1, 0, clone(step)); state.selected = index + 1; changed(); }, state.spec.steps.length >= (state.config?.limits?.max_steps || 16)],
          ["×", "Delete step", () => { state.spec.steps.splice(index, 1); state.selected = Math.min(index, state.spec.steps.length - 1); changed(); }, state.spec.steps.length <= 1],
        ]) {
          const control = node("button", text, title === "Delete step" ? "delete-step" : ""); control.title = title; control.setAttribute("aria-label", title); control.disabled = disabled; control.addEventListener("click", action); actions.append(control);
        }
        row.append(actions);
      }
      $("step-list").append(row);
    });
    $("add-step").disabled = state.spec.steps.length >= (state.config?.limits?.max_steps || 16);
  }

  function selectStep(index) { state.selected = index; renderSequence(); renderInspector(); draw(); }
  function moveStep(index, delta) { const [step] = state.spec.steps.splice(index, 1); state.spec.steps.splice(index + delta, 0, step); state.selected += delta; changed(); }
  function changed() { save(); renderSequence(); renderInspector(); draw(); }

  function renderInspector() {
    const step = selectedStep(); $("step-inspector").replaceChildren();
    $("selected-number").textContent = `EDITOR ${String(state.selected + 1).padStart(2, "0")}`;
    if (!step) return;
    const info = TYPE_INFO[step.type];
    $("step-inspector").append(node("h3", info.label, "motion-title"), node("p", info.description, "motion-description"));
    const form = node("div", undefined, "step-form");
    form.append(makeSelect("Motion type", step.type, Object.entries(TYPE_INFO).map(([id, data]) => [id, data.label]), (type) => {
      state.spec.steps[state.selected] = newStep(type); changed();
    }));
    const grid = node("div", undefined, "input-grid two");
    for (const key of [...info.fields, "timeout_ms", ...(step.type !== "turn_to_point" ? ["volts"] : [])]) {
      if (key === "direction") grid.append(makeSelect(step.type === "swing" ? "Drive direction" : "Travel direction", step.direction, [["1", "Forward (+1)"], ["-1", "Reverse (−1)"]], (value) => { step.direction = Number(value); save(); renderSequence(); draw(); }));
      else grid.append(makeNumber(key, step[key], (value) => { step[key] = value; save(); renderSequence(); draw(); }));
    }
    form.append(grid); $("step-inspector").append(form);
  }

  function newStep(type) { return { type, ...TYPE_INFO[type].defaults, timeout_ms: 4000, ...(type === "turn_to_point" ? {} : { volts: type === "wall_reset" ? 6 : 12 }) }; }

  function renderSettings() {
    $("routine-name").value = state.spec.name;
    $("stop-on-failure").checked = state.spec.stop_on_failure;
    const presets = state.config?.presets || [{ id: "four_motor_200", label: "4 motor / 200 RPM" }, { id: "six_motor_450", label: "6 motor / 450 RPM" }, { id: "speed_base", label: "Speed base / 4 in wheels" }];
    $("robot-preset").replaceChildren();
    for (const preset of presets) { const option = node("option", preset.label || preset.id); option.value = preset.id; $("robot-preset").append(option); }
    $("robot-preset").value = state.spec.preset;
    $("tracking-mode").value = state.spec.tracking_mode;
    $("heading-source").value = state.spec.environment.encoder_heading ? "encoders" : "imu";
    updatePresetDetail();
    $("start-pose-fields").replaceChildren();
    for (const key of ["x", "y", "heading"]) $("start-pose-fields").append(makeNumber(key, state.spec.start_pose[key], (value) => { state.spec.start_pose[key] = value; save(); draw(); }));
    $("environment-fields").replaceChildren();
    for (const key of ["soc", "friction", "seed"]) $("environment-fields").append(makeNumber(key, state.spec.environment[key], (value) => { state.spec.environment[key] = value; save(); }));
    for (const [key, label] of [["locked", "Lock motors"], ["constrained", "Pin robot body"]]) {
      const wrapper = node("label", undefined, "check-label"); const input = node("input"); input.type = "checkbox"; input.checked = state.spec.environment[key] === true;
      input.addEventListener("change", () => { state.spec.environment[key] = input.checked; save(); }); wrapper.append(input, document.createTextNode(label)); $("environment-fields").append(wrapper);
    }
    $("tuning-fields").replaceChildren();
    for (const [prefix, label] of [["drive", "DISTANCE / VOLTS PER INCH"], ["heading", "HEADING HOLD / VOLTS PER DEG"], ["turn", "TURN / VOLTS PER DEG"]]) {
      const group = node("div", undefined, "gain-group"); group.append(node("span", label)); const grid = node("div", undefined, "input-grid three");
      for (const suffix of ["kp", "ki", "kd"]) {
        const key = `${prefix}_${suffix}`; const metadata = state.config?.tuning?.[key];
        grid.append(makeNumber(key, state.spec.tuning[key], (value) => { state.spec.tuning[key] = value; save(); }, [suffix.toUpperCase(), metadata?.min ?? 0, metadata?.max ?? 100, 0.01]));
      }
      group.append(grid); $("tuning-fields").append(group);
    }
    $("tuning-fields").append(node("p", "These values are compiled mclib controller gains. Rerun after each change to compare the recorded response.", "micro-copy"));
  }

  function updatePresetDetail() {
    $("preset-detail").textContent = state.spec.tracking_mode === "two"
      ? "Adds two passive tracking wheels to the simulated robot. This is a hardware change from drive-encoder odometry."
      : "Uses drivetrain encoders. Wheel slip can separate odometry from ground truth.";
  }

  function buildTelemetry() {
    const groups = [
      ["Pose & target", "in / deg", [["truth-pose", "Truth X / Y / θ"], ["estimated-pose", "Odometry X / Y / θ"], ["recorded-target", "Recorded target X / Y / θ"], ["target-error", "Target error · truth"], ["final-heading-error", "Final heading error · truth"], ["odometry-gap", "Truth ↔ odometry"]]],
      ["Motion & commands", "L / R", [["body-speed", "Forward / rightward speed"], ["yaw-rate", "Yaw rate · truth"], ["motor-voltage", "Command"], ["command-mode", "Command mode"]]],
      ["Sensors", "L / R", [["imu-reading", "IMU heading / rate"], ["encoder-angle", "Raw drive encoder angle"], ["encoder-speed", "Drive encoder speed"], ["tracker-distance", "Parallel / rightward tracker"]]],
      ["Electrical", "L / R", [["battery-reading", "Battery voltage / pack current"], ["motor-current", "Total winding current"], ["motor-temperature", "Hottest motor temperature"]]],
      ["Controller", "Recorded, not inferred", [["controller-phase", "Phase"], ["controller-heading", "Steering target / error"], ["controller-remaining", "Remaining distance"], ["controller-carrot", "Carrot X / Y"], ["controller-demand", "Drive / yaw demand"], ["controller-limits", "Slew / voltage limited"]]],
    ];
    for (const [title, detail, fields] of groups) {
      const section = node("section", undefined, "telemetry-group");
      const heading = node("h3", title); heading.append(node("span", detail));
      const list = node("dl");
      for (const [id, label] of fields) {
        const row = node("div"), value = node("dd"), output = node("output", "—");
        output.id = id; value.append(output); row.append(node("dt", label), value); list.append(row);
      }
      section.append(heading, list); $("telemetry-groups").append(section);
    }
  }

  function referencePath(spec = state.spec) {
    const preset = state.config?.presets?.find((item) => item.id === spec.preset);
    return motionReferencePath(spec, preset?.track_width_in);
  }

  function fitView() {
    if (state.viewMode === "native") {
      const bounds = motionTraceBounds(state.trace);
      if (!bounds) return;
      if (state.nativeReady) $("native-viewer").contentWindow.postMessage({ type: "mclib-vexsim-fit", bounds }, window.location.origin);
      else state.nativeFitPending = bounds;
      return;
    }
    const points = [state.spec.start_pose, ...referencePath().flatMap((path) => [...path.points, path.target])];
    for (const frame of state.trace) points.push({ x: frame.true_x, y: frame.true_y }, { x: frame.estimated_x, y: frame.estimated_y });
    for (const frame of state.compare?.result?.trace || []) points.push({ x: frame.true_x, y: frame.true_y });
    const valid = points.filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
    if (!valid.length) return;
    const minX = Math.min(...valid.map((point) => point.x)), maxX = Math.max(...valid.map((point) => point.x));
    const minY = Math.min(...valid.map((point) => point.y)), maxY = Math.max(...valid.map((point) => point.y));
    state.view.x = (minX + maxX) / 2; state.view.y = (minY + maxY) / 2;
    const aspect = $("canvas-wrap").clientWidth / Math.max(1, $("canvas-wrap").clientHeight);
    state.view.span = Math.max(72, (maxY - minY + 30), (maxX - minX + 30) / aspect); draw();
  }

  function worldToScreen(point) { const t = state.transform; return { x: (point.x - state.view.x) * t.scale + t.width / 2, y: t.height / 2 - (point.y - state.view.y) * t.scale }; }
  function screenToWorld(x, y) { const t = state.transform; return { x: (x - t.width / 2) / t.scale + state.view.x, y: (t.height / 2 - y) / t.scale + state.view.y }; }
  function drawPolyline(ctx, points, color, width, dash = []) {
    if (!points.length) return; ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
    points.forEach((point, index) => { const screen = worldToScreen(point); if (index === 0) ctx.moveTo(screen.x, screen.y); else ctx.lineTo(screen.x, screen.y); }); ctx.stroke(); ctx.setLineDash([]);
  }

  function drawRobot(ctx, pose, estimated = false) {
    if (![pose.x, pose.y, pose.heading].every(Number.isFinite)) return;
    const screen = worldToScreen(pose); const size = Math.max(17, Math.min(38, state.transform.scale * 11));
    ctx.save(); ctx.translate(screen.x, screen.y); ctx.rotate(radians(pose.heading)); ctx.strokeStyle = estimated ? "#e61919" : "#111"; ctx.lineWidth = estimated ? 1.2 : 1.5; ctx.setLineDash(estimated ? [4, 3] : []);
    if (!estimated) { ctx.fillStyle = "#f4f4f0"; ctx.fillRect(-size / 2, -size / 2, size, size); }
    ctx.strokeRect(-size / 2, -size / 2, size, size);
    ctx.beginPath(); ctx.moveTo(0, -size / 2 - 6); ctx.lineTo(-4, -size / 2 + 3); ctx.lineTo(4, -size / 2 + 3); ctx.closePath(); ctx.fillStyle = estimated ? "#e61919" : "#111"; ctx.fill();
    if (!estimated) { ctx.setLineDash([]); for (const x of [-size / 2 - 2, size / 2 - 1]) { ctx.fillRect(x, -size / 2 + 4, 3, size / 3); ctx.fillRect(x, size / 6, 3, size / 3); } }
    ctx.restore();
  }

  function draw() {
    const selectedFrame = frameAt(state.time);
    syncNative(selectedFrame);
    if (state.viewMode === "native") { updateTelemetry(selectedFrame); return; }
    const canvas = $("field"), rect = canvas.getBoundingClientRect(); if (!rect.width || !rect.height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) { canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr); }
    const ctx = canvas.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, rect.width, rect.height);
    state.transform = { width: rect.width, height: rect.height, scale: Math.max(0.1, (rect.height - 52) / state.view.span) };
    const bounds = [screenToWorld(0, rect.height), screenToWorld(rect.width, 0)]; const grid = state.view.span > 220 ? 48 : state.view.span > 115 ? 24 : 12;
    ctx.font = "8px Consolas, monospace"; ctx.fillStyle = "#86867d"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    for (let x = Math.ceil(bounds[0].x / grid) * grid; x <= bounds[1].x; x += grid) {
      const screen = worldToScreen({ x, y: 0 }); ctx.beginPath(); ctx.moveTo(screen.x, 20); ctx.lineTo(screen.x, rect.height - 34); ctx.strokeStyle = x === 0 ? "#b1b1a7" : "#deded5"; ctx.lineWidth = 0.75; ctx.stroke(); if (screen.x > 22 && screen.x < rect.width - 22) ctx.fillText(String(x), screen.x, rect.height - 23);
    }
    for (let y = Math.ceil(bounds[0].y / grid) * grid; y <= bounds[1].y; y += grid) {
      const screen = worldToScreen({ x: 0, y }); if (screen.y < 20 || screen.y > rect.height - 34) continue;
      ctx.beginPath(); ctx.moveTo(30, screen.y); ctx.lineTo(rect.width - 18, screen.y); ctx.strokeStyle = y === 0 ? "#b1b1a7" : "#deded5"; ctx.lineWidth = 0.75; ctx.stroke(); ctx.fillText(String(y), 15, screen.y);
    }
    // The field is only a drawing surface: preview geometry never supplies physics data.
    const paths = referencePath();
    paths.forEach((path) => drawPolyline(ctx, path.points, "#99998f", 1, [3, 4]));
    const comparison = state.compare?.result?.trace || [];
    if (comparison.length) drawPolyline(ctx, comparison.map((frame) => ({ x: frame.true_x, y: frame.true_y })), "#b6b6ad", 3, [2, 5]);
    paths.forEach((path) => {
      const screen = worldToScreen(path.target), selected = path.index === state.selected;
      ctx.beginPath(); ctx.arc(screen.x, screen.y, selected ? 11 : 8, 0, Math.PI * 2); ctx.fillStyle = "#fff"; ctx.fill(); ctx.lineWidth = selected ? 2 : 1; ctx.strokeStyle = selected ? "#677789" : "#9ba5af"; ctx.stroke();
      ctx.font = "9px Consolas, monospace"; ctx.fillStyle = selected ? "#344252" : "#687482"; ctx.fillText(String(path.index + 1), screen.x, screen.y + 0.5);
      if (selected && path.type === "turn_to_point") drawPolyline(ctx, [path.start, path.target], "#99998f", 1, [2, 4]);
    });
    const start = worldToScreen(state.spec.start_pose); ctx.strokeStyle = "#111"; ctx.lineWidth = 1; ctx.strokeRect(start.x - 5, start.y - 5, 10, 10); ctx.font = "8px Consolas, monospace"; ctx.fillStyle = "#686862"; ctx.textAlign = "left"; ctx.fillText("START", start.x + 10, start.y + 15); ctx.textAlign = "center";
    // This canvas is the draft editor, never the native physical playback.
    drawRobot(ctx, state.spec.start_pose);
    updateTelemetry(selectedFrame);
  }

  function setFieldView(mode) {
    state.viewMode = mode === "native" ? "native" : "edit";
    $("field-view").value = state.viewMode;
    const native = state.viewMode === "native";
    $("canvas-wrap").classList.toggle("native-view", native);
    $("native-viewer").hidden = !native;
    $("native-status").hidden = !native;
    $("fit-view").textContent = native ? "Fit run" : "Fit draft";
    $("fit-view").disabled = native && !motionTraceBounds(state.trace);
    document.querySelector(".field-toolbar").hidden = native;
    $("field-help").textContent = native
      ? "Native vexsim viewer · click the view, then 1–4 for cameras. Playback uses recorded physics; it does not rerun simulation. Choose Edit targets to change the draft."
      : "Draft geometry only · outlined step = editor selection. Blue badge = current recorded step. Editing changes the draft, not the recording.";
    if (native && !$("native-viewer").getAttribute("src")) {
      $("native-viewer").src = $("native-viewer").dataset.src;
      state.nativeTimer = setTimeout(() => {
        if (!state.nativeReady) { state.nativeError = "Native viewer unavailable: no ready response. Check local viewer assets and WebGL support."; syncNative(frameAt(state.time)); }
      }, 10000);
    }
    draw();
  }

  function syncNative(frame, send = true) {
    const iframe = $("native-viewer");
    const status = $("native-status");
    status.classList.toggle("error", !!state.nativeError);
    status.textContent = state.nativeError || (!state.nativeReady ? "Loading native vexsim viewer…" : state.nativeUnavailable || (frame && state.viewMode === "native" ? `Vexsim native viewer · ${state.incomplete ? "INCOMPLETE " : ""}recorded physics` : "Vexsim native viewer · no frame selected"));
    $("fit-view").disabled = state.viewMode === "native" && !motionTraceBounds(state.trace);
    if (state.nativeReady && send) iframe.contentWindow.postMessage({
      type: "mclib-vexsim-frame", frame: state.viewMode === "native" ? frame : null,
      playing: state.playing, live: state.live,
    }, window.location.origin);
  }

  function frameAt(time) {
    return motionFrameAt(state.trace, time, state.inspection);
  }

  function visibleTrace() {
    return motionVisibleTrace(state.trace, state.time, state.inspection);
  }

  function updateTelemetry(frame) {
    $("current-time").textContent = `${fmt(state.time, 2)} s`;
    $("timeline").value = state.time;
    const target = motionRecording(state.job).steps?.find(step => step.index === frame?.step)?.target;
    for (const [id, value] of Object.entries(motionTelemetry(frame, target))) $(id).textContent = value;
    const stepLabel = Number.isInteger(frame?.step) ? `STEP ${String(frame.step + 1).padStart(2, "0")}` : "";
    const skipped = state.inspection?.executed === false;
    let label = "NO RECORDED RUN", empty = "";
    if (skipped) {
      label = `STEP ${String(state.inspection.index + 1).padStart(2, "0")} / SKIPPED`;
      empty = "This step was skipped. There is no recorded motion or telemetry for it.";
    } else if (frame) label = `${stepLabel} / ${frame.phase === "hold" ? "SETTLING" : "MOTION"}`;
    else if (state.live) {
      label = "LIVE / WAITING FOR DATA";
      empty = state.job?.progress?.phase === "build" ? "Building C++ — waiting for the first live sample." : "Waiting for live telemetry…";
    } else if (state.inspection) {
      label = `STEP ${state.inspection.index + 1} / NO RECORDING`; empty = "No recorded frame is available for this step.";
    }
    $("playback-step").textContent = `${label}${state.incomplete && frame ? " / INCOMPLETE" : ""}`;
    $("field-empty").textContent = empty; $("field-empty").hidden = !empty;
    $("telemetry-mode").textContent = skipped ? "SKIPPED / NO DATA" : state.live ? `${state.connected ? "LIVE" : "LIVE / DISCONNECTED"}${stepLabel ? ` · ${stepLabel}` : ""}` : state.incomplete ? `INCOMPLETE${stepLabel ? ` · ${stepLabel}` : " / NO DATA"}` : state.inspection ? `INSPECT ${state.inspection.index + 1}` : frame ? `REPLAY · ${stepLabel}` : "NO DATA";
    $("telemetry-mode").classList.toggle("live", state.live);
    updateReplayHighlight(frame);
    drawPlots();
  }

  function updateReplayHighlight(frame) {
    for (const row of $("step-list").children) {
      const current = !state.dirty && frame?.step === Number(row.dataset.step);
      row.classList.toggle("replay-current", current);
      row.querySelector(".replay-badge").textContent = state.live ? "LIVE NOW" : "REPLAY FRAME";
    }
    for (const row of $("step-results").children) {
      row.classList.toggle("replay-current", frame?.step === Number(row.dataset.step));
      row.classList.toggle("selected", state.inspection?.index === Number(row.dataset.step));
    }
  }

  function drawPlots() {
    if (!document.querySelector(".telemetry-plots").open) return;
    const trace = visibleTrace();
    for (const [id, keys, fixed] of [["command-plot", ["left_volts", "right_volts"], [-12, 12]], ["heading-plot", ["true_heading", "imu_heading_deg", "controller_target_heading_deg"], null]]) {
      const canvas = $(id), rect = canvas.getBoundingClientRect(); if (!rect.width) continue;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
      const ctx = canvas.getContext("2d"); ctx.scale(dpr, dpr);
      const seriesValues = keys.map(key => fixed ? trace.map(frame => frame[key]) : motionUnwrapSeries(trace, key));
      const finiteValues = seriesValues.flat().filter(Number.isFinite);
      if (!finiteValues.length) { ctx.fillStyle = "#596574"; ctx.font = "10px Consolas, monospace"; ctx.fillText("No recorded samples", 4, 20); continue; }
      const min = fixed?.[0] ?? Math.min(...finiteValues), max = fixed?.[1] ?? Math.max(...finiteValues);
      const span = Math.max(1, max - min), end = Math.max(.01, trace.at(-1).t - trace[0].t);
      const y = value => 8 + (max - value) / span * (rect.height - 16);
      ctx.strokeStyle = "#d3d9e0"; ctx.beginPath(); ctx.moveTo(0, y(Math.max(min, Math.min(max, 0)))); ctx.lineTo(rect.width, y(Math.max(min, Math.min(max, 0)))); ctx.stroke();
      seriesValues.forEach((values, series) => {
        ctx.strokeStyle = ["#175ea8", "#b96514", "#7e47a0"][series]; ctx.lineWidth = 1.2; ctx.beginPath(); let started = false;
        for (let index = 0; index < trace.length; index++) {
          const frame = trace[index], value = values[index];
          if (!Number.isFinite(value)) { started = false; continue; }
          const x = (frame.t - trace[0].t) / end * rect.width;
          if (started) ctx.lineTo(x, y(value)); else ctx.moveTo(x, y(value)); started = true;
        }
        ctx.stroke();
      });
      ctx.fillStyle = "#596574"; ctx.font = "8px Consolas, monospace"; ctx.fillText(`${fmt(max, 0)}`, 2, 7); ctx.fillText(`${fmt(min, 0)}`, 2, rect.height - 1);
    }
  }

  function updateFieldMode() {
    $("field-mode").textContent = state.live ? "Live simulation" : state.incomplete ? "Incomplete recording" : state.trace.length ? "Recorded run" : "Preview";
    $("field-caption").textContent = state.trace.length
      ? `${state.dirty ? "REFERENCE = EDITED DRAFT · " : ""}${state.live ? "LIVE PHYSICS" : state.incomplete ? "INCOMPLETE RECORDED PHYSICS" : "RECORDED PHYSICS"}${state.compare ? " · GRAY DOTS = PREVIOUS TRUTH" : ""}`
      : "DRAFT REFERENCE · NOT A RECORDING";
  }

  function play(playing) {
    state.playing = playing && !state.live && state.trace.length > 0;
    if (state.playing) state.inspection = null;
    $("play-pause").textContent = state.playing ? "Ⅱ" : "▶";
    $("play-pause").setAttribute("aria-label", state.playing ? "Pause recorded simulation" : "Play recorded simulation");
    if (state.playing) { if (state.time >= state.trace.at(-1).t) state.time = 0; state.lastFrame = performance.now(); requestAnimationFrame(animate); }
    syncNative(frameAt(state.time));
  }

  function animate(now) {
    if (!state.playing) return;
    state.time = Math.min(state.trace.at(-1).t, state.time + (now - state.lastFrame) / 1000 * state.speed); state.lastFrame = now; draw();
    if (state.time >= state.trace.at(-1).t) play(false); else requestAnimationFrame(animate);
  }

  function updateHistoryControls() {
    const busy = state.submitting || !!state.activeId || state.live;
    $("run-history").disabled = busy;
    $("compare-history").disabled = busy;
    $("load-recorded").disabled = busy || state.historyLoading || !state.job?.spec;
  }

  function invalidateRecordRequests() {
    state.historyRequest += 1; state.historyLoading = false; state.historyChoice = state.job?.id || "";
    state.compareRequest += 1; state.compareLoading = false; state.compareChoice = state.compare?.id || "";
    $("run-history").value = state.historyChoice;
    $("compare-history").value = state.compareChoice;
  }

  function updateJobStatus(job) {
    const busy = ["queued", "running"].includes(job.status); const phase = job.progress?.phase || (busy ? "build" : "complete");
    $("cancel-run").hidden = !busy; $("cancel-run").disabled = false;
    $("run-sequence").disabled = busy || !state.connected;
    updateHistoryControls();
    $("run-progress").hidden = !busy;
    $("run-progress").value = phase === "build" ? 10 : phase === "run" ? 20 + 70 * finite(job.progress?.step) / Math.max(1, finite(job.progress?.total, 1)) : 95;
    $("run-status-title").textContent = busy ? (phase === "build" ? "BUILDING CURRENT C++" : `RUNNING STEP ${finite(job.progress?.step) + 1} / ${job.progress?.total || job.spec?.steps?.length || "—"}`) : `${job.status.toUpperCase()} / ${job.spec?.name || state.spec.name}`;
    $("run-status-title").parentElement.classList.toggle("error", ["failed", "error"].includes(job.status));
    $("run-status-detail").textContent = busy ? (phase === "build" ? "Compiling the motion library. This may take a moment on the first run." : "Executing the ordered motions through the real C++ controller and physics adapter.") : job.error || (job.status === "cancelled" ? "The run was cancelled. The editor is ready for another run." : job.status === "failed" ? "Inspect the failed step below. Adjust the sequence or tuning, then run again." : "Execution complete. Scrub the recording and inspect each acceptance check.");
    if (state.incomplete) $("run-status-detail").textContent = `${job.error ? `${typeof job.error === "string" ? job.error : "The run errored."} ` : ""}Incomplete recording: ${state.trace.length ? "ends at the last captured sample; final stop and acceptance are unverified." : "no motion samples were captured."}`;
  }

  async function runSequence() {
    if (state.activeId || state.submitting) return;
    // A late history response must not replace a newly submitted live job.
    state.submitting = true; invalidateRecordRequests(); updateHistoryControls();
    notice(); play(false); $("run-sequence").disabled = true;
    try {
      const job = await api("/api/runs", { method: "POST", body: JSON.stringify(state.spec) });
      if (!state.compare && state.job?.result) state.compare = state.job;
      state.submitting = false; state.activeId = job.id; state.dirty = false; await loadJob(job);
      await pollRun(job.id);
    } catch (error) { notice(error.message); state.submitting = false; state.activeId = null; updateHistoryControls(); $("run-sequence").disabled = !state.connected; $("cancel-run").hidden = true; $("run-progress").hidden = true; }
  }

  async function pollRun(id) {
    if (state.activeId !== id) return;
    try {
      const job = await api(`/api/runs/${encodeURIComponent(id)}`); setConnection(true);
      if (["queued", "running"].includes(job.status)) { await loadJob(job, true); setTimeout(() => pollRun(id), 250); return; }
      state.activeId = null; await loadJob(job); await refreshHistory();
    } catch (error) {
      setConnection(false); notice(`Connection interrupted: ${error.message} The run may still be active; reconnecting…`);
      draw();
      setTimeout(() => pollRun(id), 1800);
    }
  }

  async function loadJob(job, liveUpdate = false) {
    const hadTrace = state.trace.length > 0;
    if (state.job?.id !== job.id) invalidateRecordRequests();
    play(false); state.job = job; state.inspection = null;
    const recording = motionRecording(job); state.live = recording.live === true; state.incomplete = recording.incomplete === true;
    state.trace = Array.isArray(recording.trace) ? recording.trace : [];
    state.time = state.trace.at(-1)?.t || 0;
    $("timeline").max = motionTimelineMax(state.time);
    $("timeline").disabled = state.live || !state.trace.length; $("play-pause").disabled = state.live || !state.trace.length;
    $("playback-speed").disabled = state.live || !state.trace.length;
    $("end-time").textContent = `${fmt(state.time, 2)} s`;
    if (!state.trace.length) $("playback-step").textContent = "NO RECORDED RUN";
    if (state.compare?.id === job.id) state.compare = null;
    if (!state.compareLoading) { state.compareChoice = state.compare?.id || ""; $("compare-history").value = state.compareChoice; }
    updateJobStatus(job);
    $("run-history").value = job.id; updateFieldMode(); renderResults(); renderSequence();
    if (!liveUpdate) setFieldView(state.live || state.trace.length ? "native" : "edit");
    if (!liveUpdate || (!hadTrace && state.trace.length)) fitView(); else draw();
  }

  async function refreshHistory() {
    try {
      const data = await api("/api/runs"); state.jobs = Array.isArray(data) ? data : data.runs || [];
      for (const id of ["run-history", "compare-history"]) {
        const current = id === "run-history" ? (state.historyLoading ? state.historyChoice : state.job?.id || "") : (state.compareLoading ? state.compareChoice : state.compare?.id || "");
        $(id).replaceChildren(); const blank = node("option", id === "run-history" ? "Choose a recorded run" : "No comparison"); blank.value = ""; $(id).append(blank);
        for (const job of state.jobs) {
          if (["queued", "running"].includes(job.status)) continue;
          if (id === "compare-history" && job.id === state.job?.id) continue;
          const date = job.created_at ? new Date(typeof job.created_at === "number" ? job.created_at * 1000 : job.created_at) : null;
          const time = date && !Number.isNaN(date.getTime()) ? date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) : String(job.id).slice(-6);
          const option = node("option", `${time} / ${job.status} / ${job.name || job.spec?.name || "Routine"}`); option.value = job.id; $(id).append(option);
        }
        $(id).value = current;
      }
    } catch (error) { notice(`Run history unavailable: ${error.message}`); }
  }

  function resultCell(label, value, detail, failed = false) {
    const cell = node("div", undefined, "summary-cell"); cell.append(node("span", label, "label"), node("strong", value, failed ? "failed" : "")); if (detail) cell.append(node("small", detail)); return cell;
  }

  function renderResults() {
    const job = state.job, result = job && (job.result || job.partial || state.incomplete ? motionRecording(job) : null); $("step-results").replaceChildren();
    if (!job) return;
    $("comparison-context").hidden = !state.compare;
    if (state.compare) {
      const previous = state.compare.spec || {}, current = job.spec || {};
      const changes = [];
      if (previous.tracking_mode !== current.tracking_mode) changes.push(`sensors ${previous.tracking_mode === "two" ? "tracking wheels" : "drive encoders"} → ${current.tracking_mode === "two" ? "tracking wheels" : "drive encoders"} (hardware change)`);
      if (previous.preset !== current.preset) changes.push(`drivetrain ${previous.preset} → ${current.preset}`);
      if (JSON.stringify(previous.tuning) !== JSON.stringify(current.tuning)) changes.push("controller gains changed");
      if (JSON.stringify(previous.environment) !== JSON.stringify(current.environment)) changes.push("environment changed");
      changes.push(JSON.stringify(previous.steps) === JSON.stringify(current.steps) ? "same motion sequence" : "motion sequence changed");
      if (result?.source_fingerprint && state.compare.result?.source_fingerprint !== result.source_fingerprint) changes.push("C++ / simulator source changed");
      $("comparison-context").textContent = `Compared with ${previous.name || "previous run"}: ${changes.join(" · ")}. The previous ground-truth path overlay is available in Edit targets; native playback shows the selected run only.`;
    }
    $("result-summary").replaceChildren();
    if (!result) {
      const empty = node("div", job.status === "error" ? "The run could not finish." : `Run ${job.status}.`, "empty-summary"); empty.append(node("span", job.error || "No completed motion trace is available.")); $("result-summary").append(empty);
    } else {
      const steps = result.steps || []; const executed = steps.filter((step) => step.executed !== false && typeof step.passed === "boolean"); const failed = executed.filter((step) => !step.passed);
      const summary = result.summary || {};
      const positionErrors = executed.map((step) => step.position_error_in).filter((value) => typeof value === "number" && Number.isFinite(value));
      const last = state.trace.at(-1); const drift = last && !last.coordinate_reset ? Math.hypot(last.true_x - last.estimated_x, last.true_y - last.estimated_y) : null;
      $("result-summary").append(
        resultCell("ACCEPTANCE", motionAcceptance(result), `${executed.length - failed.length}/${job.spec?.steps?.length || steps.length} STEPS PASSED${state.incomplete ? " · RUN INCOMPLETE" : ""}`, !state.live && result.passed !== true),
        resultCell(state.live ? "SIMULATED TIME" : "RECORDED TIME", `${fmt((summary.elapsed_ms ?? (state.trace.at(-1)?.t || 0) * 1000) / 1000, 2)} s`, `${summary.skipped_steps ?? steps.filter(step => step.executed === false).length} STEPS SKIPPED`),
        resultCell("MAX POSITION ERROR", positionErrors.length ? `${fmt(Math.max(...positionErrors), 2)} in` : "—", "GROUND TRUTH / STEP END"),
        resultCell(state.live ? "CURRENT ODOMETRY GAP" : state.incomplete ? "LAST CAPTURED ODOMETRY GAP" : "FINAL ODOMETRY GAP", last?.coordinate_reset ? "FRAME RESET" : drift === null ? "—" : `${fmt(drift, 2)} in`, last?.coordinate_reset ? "COORDINATES NO LONGER COMPARABLE" : "TRUTH ↔ ESTIMATED POSE"),
      );
      for (const step of steps) {
        const row = node("tr"); row.tabIndex = state.live ? -1 : 0; row.dataset.step = step.index; row.setAttribute("aria-label", `Inspect step ${step.index + 1}, ${labelFor(step.type)}`);
        row.append(node("td", `${String(step.index + 1).padStart(2, "0")} / ${labelFor(step.type)}`));
        const status = motionStepStatus(step, result); const tag = node("td"); tag.append(node("span", status, `result-tag${status === "FAIL" ? " failed" : ["SKIPPED", "INTERRUPTED"].includes(status) ? " skipped" : ""}`)); row.append(tag);
        row.append(node("td", step.executed === false ? "—" : `${fmt(step.duration_ms / 1000, 2)} s`), node("td", Number.isFinite(step.position_error_in) ? `${fmt(step.position_error_in, 2)} in` : "—"), node("td", Number.isFinite(step.heading_error_deg) ? `${fmt(step.heading_error_deg, 2)}°` : "—"));
        const previous = state.compare?.result?.steps?.find((other) => other.index === step.index && other.type === step.type);
        const delta = previous && Number.isFinite(previous.position_error_in) && Number.isFinite(step.position_error_in) ? step.position_error_in - previous.position_error_in : null;
        row.append(node("td", delta === null ? "—" : `${delta > 0 ? "+" : ""}${fmt(delta, 2)} in`, delta > 0 ? "error-text" : ""));
        const failedChecks = (step.checks || []).filter((check) => !check.passed).map((check) => check.name).join(", ");
        row.append(node("td", status === "INTERRUPTED" ? "Run ended before this action completed; only partial frames are available." : step.reason || failedChecks || (step.passed ? "All acceptance checks passed." : "No result."), status === "FAIL" ? "error-text" : ""));
        const inspect = () => {
          if (state.live) return;
          play(false); state.inspection = step;
          const frame = frameAt(state.time);
          state.time = frame?.t ?? (Number.isFinite(step.end_time_ms) ? step.end_time_ms / 1000 : 0);
          // Inspecting a recording must not change the independent editor selection.
          draw(); renderDiagnostics(step);
        };
        row.addEventListener("click", inspect); row.addEventListener("keydown", (event) => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); inspect(); } }); $("step-results").append(row);
      }
    }
    renderDiagnostics();
  }

  function renderDiagnostics(step) {
    const job = state.job; if (!job) return;
    $("diagnostic-details").hidden = false;
    const lines = [`Run: ${job.id}`, `Status: ${job.status}`, `Sensors: ${job.spec?.tracking_mode === "two" ? "two passive tracking wheels" : "drive encoders"}`, `Preset: ${job.spec?.preset || "—"}`];
    if (job.artifact_dir) lines.push(`Artifacts: ${job.artifact_dir}`);
    if (job.result?.source_fingerprint) lines.push(`Recorded source fingerprint: ${job.result.source_fingerprint}`);
    if (job.error) lines.push("", "Error:", typeof job.error === "string" ? job.error : JSON.stringify(job.error, null, 2));
    if (step) {
      const reason = state.incomplete && step.executed !== false && typeof step.passed !== "boolean"
        ? "INTERRUPTED: the run ended before this action completed. Final stop and acceptance are unverified." : step.reason || "";
      lines.push("", `Step ${step.index + 1}: ${labelFor(step.type)}`, reason, ...(step.diagnosis || []));
      for (const check of step.checks || []) lines.push(`${check.passed ? "PASS" : "FAIL"} ${check.name}: ${check.actual ?? "—"} (expected ${typeof check.expected === "object" ? JSON.stringify(check.expected) : check.expected ?? "—"})`);
      $("diagnostic-details").open = true;
    }
    if (job.result?.assumptions) lines.push("", ...(Array.isArray(job.result.assumptions) ? job.result.assumptions : [String(job.result.assumptions)]));
    $("diagnostic-output").textContent = lines.join("\n");
  }

  function bindEvents() {
    $("routine-name").addEventListener("input", (event) => { state.spec.name = event.target.value; save(); });
    $("stop-on-failure").addEventListener("change", (event) => { state.spec.stop_on_failure = event.target.checked; save(); });
    $("robot-preset").addEventListener("change", (event) => { state.spec.preset = event.target.value; save(); updatePresetDetail(); });
    $("tracking-mode").addEventListener("change", (event) => { state.spec.tracking_mode = event.target.value; save(); updatePresetDetail(); });
    $("heading-source").addEventListener("change", (event) => { state.spec.environment.encoder_heading = event.target.value === "encoders"; save(); });
    $("add-step").addEventListener("click", () => { state.spec.steps.push(newStep($("new-step-type").value)); state.selected = state.spec.steps.length - 1; changed(); $("step-list").lastElementChild?.scrollIntoView({ block: "nearest" }); });
    $("new-project").addEventListener("click", () => { state.spec = clone(state.config?.defaults || FALLBACK); state.selected = 0; save(); renderSettings(); renderSequence(); renderInspector(); setFieldView("edit"); fitView(); notice("A fresh routine is ready. Recorded runs remain available below."); });
    $("export-project").addEventListener("click", () => {
      const blob = new Blob([JSON.stringify(state.spec, null, 2) + "\n"], { type: "application/json" }); const link = node("a"); link.href = URL.createObjectURL(blob); link.download = `${state.spec.name.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "") || "autonomous"}.json`; document.body.append(link); link.click(); link.remove(); setTimeout(() => URL.revokeObjectURL(link.href), 1000);
    });
    $("import-project").addEventListener("click", () => $("import-file").click());
    $("import-file").addEventListener("change", async (event) => {
      const file = event.target.files?.[0]; if (!file) return;
      try { if (file.size > 65536) throw new Error("Routine JSON must be smaller than 64 KB."); state.spec = normalizeSpec(JSON.parse(await file.text())); state.selected = 0; save(); renderSettings(); renderSequence(); renderInspector(); setFieldView("edit"); fitView(); notice(`Imported ${state.spec.steps.length} steps from ${file.name}.`); }
      catch (error) { notice(`Import failed: ${error.message}`); } event.target.value = "";
    });
    $("fit-view").addEventListener("click", fitView);
    for (const [id, mode] of [["example-baseline", "drive"], ["example-trackers", "two"]]) $(id).addEventListener("click", () => {
      state.spec = clone(state.config?.defaults || FALLBACK); state.spec.name = mode === "drive" ? "Boomerang / slip baseline" : "Boomerang / tracking wheels"; state.spec.tracking_mode = mode; state.spec.preset = "six_motor_450";
      state.spec.steps = [{ type: "boomerang", x: 24, y: 24, heading: 90, direction: 1, lead: 0.5, timeout_ms: 4000, volts: 12 }]; state.selected = 0;
      save(); renderSettings(); renderSequence(); renderInspector(); setFieldView("edit"); fitView(); notice(mode === "drive" ? "Baseline loaded: drive encoders measure wheel motion, including slip. Build & run to record this case." : "Tracking-wheel variant loaded: two passive sensors are added to the robot. The motion and controller gains match the baseline.");
    });
    $("load-recorded").addEventListener("click", () => {
      if (state.submitting || state.activeId || state.historyLoading || state.live || !state.job?.spec || $("run-history").value !== state.job.id) return;
      state.spec = normalizeSpec(state.job.spec); state.selected = 0; save(); state.dirty = false; renderSettings(); renderSequence(); renderInspector(); updateFieldMode(); fitView(); notice("Recorded routine loaded into the editor. Change a parameter and rerun to compare.");
    });
    $("run-sequence").addEventListener("click", runSequence);
    $("cancel-run").addEventListener("click", async () => {
      if (!state.activeId) return; $("cancel-run").disabled = true;
      try { await api(`/api/runs/${encodeURIComponent(state.activeId)}/cancel`, { method: "POST", body: "{}" }); $("run-status-detail").textContent = "Cancellation requested. Waiting for the simulation process to stop."; }
      catch (error) { notice(error.message); $("cancel-run").disabled = false; }
    });
    $("play-pause").addEventListener("click", () => play(!state.playing));
    $("timeline").addEventListener("input", (event) => { if (state.live) return; play(false); state.inspection = null; state.time = Math.min(Number(event.target.value), state.trace.at(-1)?.t ?? 0); draw(); });
    $("playback-speed").addEventListener("change", (event) => { state.speed = Number(event.target.value); });
    $("field-view").addEventListener("change", event => setFieldView(event.target.value));
    $("native-viewer").addEventListener("error", () => { state.nativeError = "Native vexsim viewer failed to load."; syncNative(frameAt(state.time)); });
    window.addEventListener("message", event => {
      if (event.origin !== window.location.origin || event.source !== $("native-viewer").contentWindow) return;
      if (event.data?.type === "mclib-vexsim-ready") {
        clearTimeout(state.nativeTimer); state.nativeReady = true; state.nativeError = ""; syncNative(frameAt(state.time));
        if (state.nativeFitPending) {
          $("native-viewer").contentWindow.postMessage({ type: "mclib-vexsim-fit", bounds: state.nativeFitPending }, window.location.origin);
          state.nativeFitPending = null;
        }
      } else if (event.data?.type === "mclib-vexsim-applied") {
        $("native-viewer").dataset.applied = JSON.stringify({ t: event.data.t ?? null, robot: event.data.robot ?? null, available: event.data.available, reason: event.data.reason || "" });
        state.nativeUnavailable = event.data.available === false && event.data.t !== null
          ? (String(event.data.reason || "Recorded native data unavailable").includes("geometry") ? "Recording lacks chassis dimensions; rerun for native playback." : String(event.data.reason || "Recorded native data unavailable")) : "";
        state.nativeError = ""; syncNative(frameAt(state.time), false);
      } else if (event.data?.type === "mclib-vexsim-fitted") {
        $("native-viewer").dataset.fitted = JSON.stringify(event.data.bounds);
      } else if (event.data?.type === "mclib-vexsim-error") {
        state.nativeError = `Native vexsim viewer: ${String(event.data.message || "viewer error")}`; syncNative(frameAt(state.time), false);
      }
    });
    $("run-history").addEventListener("change", async (event) => {
      if (state.submitting || state.activeId) { $("run-history").value = state.job?.id || ""; return; }
      const requested = event.target.value, request = ++state.historyRequest;
      state.historyChoice = requested; state.historyLoading = !!requested;
      $("load-recorded").disabled = state.historyLoading || state.live || !state.job?.spec;
      if (!requested) { $("run-history").value = state.job?.id || ""; return; }
      notice("Loading recorded run… The editor remains unchanged.");
      try {
        const job = await api(`/api/runs/${encodeURIComponent(requested)}`);
        if (request !== state.historyRequest) return;
        state.historyLoading = false;
        state.dirty = JSON.stringify(job.spec) !== JSON.stringify(state.spec);
        await loadJob(job); notice(); await refreshHistory();
      } catch (error) {
        if (request !== state.historyRequest) return;
        state.historyLoading = false; state.historyChoice = state.job?.id || "";
        $("run-history").value = state.historyChoice;
        $("load-recorded").disabled = state.live || !state.job?.spec;
        notice(`Could not load recorded run: ${error.message} The previous recording is still selected; the editor is unchanged.`);
      }
    });
    $("compare-history").addEventListener("change", async (event) => {
      if (state.submitting || state.activeId) { $("compare-history").value = state.compareChoice; return; }
      const requested = event.target.value, request = ++state.compareRequest, currentId = state.job?.id;
      state.compareChoice = requested; state.compareLoading = !!requested && requested !== currentId;
      if (!state.compareLoading) {
        state.compare = null; state.compareChoice = ""; $("compare-history").value = "";
        renderResults(); updateFieldMode(); fitView(); notice(); return;
      }
      notice("Loading comparison…");
      try {
        const comparison = await api(`/api/runs/${encodeURIComponent(requested)}`);
        if (request !== state.compareRequest || state.job?.id !== currentId) return;
        state.compareLoading = false; state.compare = comparison.id === state.job?.id ? null : comparison;
        state.compareChoice = state.compare?.id || ""; $("compare-history").value = state.compareChoice;
        renderResults(); updateFieldMode(); fitView(); notice();
      } catch (error) {
        if (request !== state.compareRequest || state.job?.id !== currentId) return;
        state.compareLoading = false; state.compareChoice = state.compare?.id || "";
        $("compare-history").value = state.compareChoice;
        notice(`Could not load comparison: ${error.message} The previous comparison is still selected.`);
      }
    });
    bindCanvas(); new ResizeObserver(draw).observe($("canvas-wrap"));
    document.querySelector(".telemetry-plots").addEventListener("toggle", drawPlots);
    document.addEventListener("visibilitychange", () => { if (document.hidden) play(false); });
  }

  function bindCanvas() {
    const canvas = $("field");
    function coordinates(event) { const rect = canvas.getBoundingClientRect(); return { x: event.clientX - rect.left, y: event.clientY - rect.top }; }
    canvas.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || !state.transform) return;
      const position = coordinates(event); const paths = referencePath(); let hit = null;
      for (const path of [...paths].reverse()) {
        const screen = worldToScreen(path.target); if (Math.hypot(screen.x - position.x, screen.y - position.y) < 16 && ["point", "boomerang", "turn_to_point", "wall_reset"].includes(path.type)) { hit = { type: "target", index: path.index }; break; }
      }
      const start = worldToScreen(state.spec.start_pose);
      if (!hit && Math.hypot(start.x - position.x, start.y - position.y) < 18) hit = { type: "start" };
      if (!hit && ["point", "boomerang", "turn_to_point", "wall_reset"].includes(selectedStep()?.type)) hit = { type: "target", index: state.selected };
      if (hit) { state.drag = hit; if (hit.type === "target") selectStep(hit.index); canvas.setPointerCapture(event.pointerId); moveTarget(position); }
    });
    function moveTarget(position) {
      if (!state.drag) return;
      const point = screenToWorld(position.x, position.y); const target = state.drag.type === "start" ? state.spec.start_pose : state.spec.steps[state.drag.index];
      target.x = Math.max(-500, Math.min(500, Math.round(point.x * 2) / 2)); target.y = Math.max(-500, Math.min(500, Math.round(point.y * 2) / 2)); save(); draw();
    }
    canvas.addEventListener("pointermove", (event) => {
      if (!state.transform) return; const position = coordinates(event), point = screenToWorld(position.x, position.y);
      $("field-coordinate").textContent = `X ${fmt(point.x)} IN / Y ${fmt(point.y)} IN · CW+`; moveTarget(position);
    });
    const finishDrag = () => { if (state.drag) { state.drag = null; renderSequence(); renderInspector(); renderSettings(); } };
    canvas.addEventListener("pointerup", finishDrag); canvas.addEventListener("pointercancel", finishDrag);
    canvas.addEventListener("pointerleave", () => { if (!state.drag) $("field-coordinate").textContent = "+X RIGHT · +Y FORWARD · CW+"; });
    canvas.addEventListener("wheel", (event) => { event.preventDefault(); state.view.span = Math.max(24, Math.min(1000, state.view.span * (event.deltaY > 0 ? 1.12 : 0.89))); draw(); }, { passive: false });
    canvas.addEventListener("keydown", (event) => {
      if (event.key === " ") { event.preventDefault(); play(!state.playing); }
      if (["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(event.key)) { event.preventDefault(); const amount = state.view.span / 12; state.view.x += event.key === "ArrowRight" ? amount : event.key === "ArrowLeft" ? -amount : 0; state.view.y += event.key === "ArrowUp" ? amount : event.key === "ArrowDown" ? -amount : 0; draw(); }
    });
  }

  async function init() {
    buildTelemetry();
    for (const [type, info] of Object.entries(TYPE_INFO)) { const option = node("option", info.label); option.value = type; $("new-step-type").append(option); }
    bindEvents();
    try { state.config = await api("/api/config"); state.spec = normalizeSpec(state.config.defaults || FALLBACK); setConnection(true); }
    catch (error) { setConnection(false); notice(`The local simulation server is unavailable. ${error.message} Start tests/vexsim/builder/server.py to build and run motions.`); }
    try { const stored = localStorage.getItem(STORAGE_KEY); if (stored) state.spec = normalizeSpec(JSON.parse(stored)); }
    catch (error) { notice(`Could not restore the saved routine: ${error.message}`); }
    renderSettings(); renderSequence(); renderInspector(); fitView();
    if (state.connected) await refreshHistory();
  }
  init();
})();
