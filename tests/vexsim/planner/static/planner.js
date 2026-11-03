"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const DRAFT_KEY = "mclib.path-planner.draft.v1";
  const FIELD_HALF_IN = 72;
  const TILE_IN = 24;
  const HANDLE_RADIUS_PX = 6;
  const ANCHOR_RADIUS_PX = 8;
  const HEADING_KNOB_IN = 14;
  const SPEED_STOPS = [[0, [44, 95, 214]], [0.35, [20, 163, 163]], [0.7, [230, 180, 0]], [1, [216, 67, 47]]];
  const SHORTCUTS = [
    ["Click the field", "Add a point to the end of the selected path, or place the selected step's target"],
    ["Click the selected path's line", "Insert a point there"],
    ["Drag a point, handle or target", "Move it. Hold Shift to snap to a 6 in grid and headings to 15°"],
    ["Drag empty space · scroll", "Pan · zoom"],
    ["Double-click a point", "Put its heading and curve handles back on automatic"],
    ["Right-click a point", "Delete it"],
    ["Delete or Backspace", "Delete the selected point, or the selected step if no point is selected"],
    ["Arrow keys", "Nudge the selected point 0.5 in (Shift: 6 in)"],
    ["Ctrl+Z · Ctrl+Shift+Z", "Undo · redo"],
    ["Ctrl+Enter", "Run the simulation"],
    ["Space", "Play or pause"],
    [", and .", "Step playback 0.1 s back or forward"],
    ["[ and ]", "Select the previous or next step"],
    ["F", "Fit the routine in view"],
    ["Esc", "Deselect the point"],
    ["?", "Show the help"],
    ["1 to 4 (3D view)", "Change the 3D camera"],
  ];

  const state = {
    config: null,
    project: null,
    selected: 0,
    point: null,
    history: [],
    future: [],
    plan: null,
    planFor: null,
    planError: null,
    planInFlight: false,
    planAgain: false,
    planTimer: null,
    engine: null,
    engineTimer: null,
    runs: [],
    activeRun: null,
    shown: null,
    compare: null,
    frames: [],
    pollTimer: null,
    time: 0,
    playing: false,
    speed: 1,
    lastTick: 0,
    view: { x: 0, y: 0, scale: 4 },
    viewMovedByUser: false,
    size: { width: 0, height: 0 },
    layers: { plan: true, truth: true, odometry: true, compare: true },
    drag: null,
    hover: null,
    tab: "step",
    mode: "top",
    viewer: { ready: false, error: "" },
    log: [],
    graphCache: null,
    pieceCache: null,
  };

  const clone = (value) => JSON.parse(JSON.stringify(value));
  const fmt = (value, digits = 1) => (Number.isFinite(value) ? value.toFixed(digits) : "—");
  const trim = (value) => {
    const text = Number(value).toFixed(2).replace(/\.?0+$/, "");
    return text === "-0" ? "0" : text;
  };
  const round = (value, step) => Math.round(value / step) * step;
  function element(tag, attributes = {}, ...children) {
    const node = document.createElement(tag);
    for (const [key, value] of Object.entries(attributes)) {
      if (value === undefined || value === null || value === false) continue;
      if (key === "class") node.className = value;
      else if (key === "text") node.textContent = value;
      else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
      else node.setAttribute(key, value === true ? "" : value);
    }
    for (const child of children.flat()) {
      if (child === null || child === undefined || child === false) continue;
      node.append(child instanceof Node ? child : document.createTextNode(String(child)));
    }
    return node;
  }

  async function api(path, options = {}) {
    const response = await fetch(path, {
      ...options,
      headers: options.body ? { "Content-Type": "application/json" } : undefined,
      body: options.body ? JSON.stringify(options.body) : undefined,
    });
    let data;
    try {
      data = await response.json();
    } catch {
      throw new Error(`The server sent an unreadable answer (HTTP ${response.status}).`);
    }
    return { status: response.status, ok: response.ok, data };
  }

  function log(message, kind = "") {
    const now = new Date();
    state.log.unshift({ time: now.toTimeString().slice(0, 8), message, kind });
    state.log.length = Math.min(state.log.length, 60);
    renderLog();
  }

  const preset = () => state.config.presets[state.project.robot.preset];
  const trackWidth = () => preset().track_width_in;
  const stepInfo = (step) => state.config.step_types[step.type];
  const selectedStep = () => state.project.steps[state.selected];
  const poses = () => designedPoses(state.project, trackWidth());
  const sortedKeys = (value) => {
    if (Array.isArray(value)) return value.map(sortedKeys);
    if (value === null || typeof value !== "object") return value;
    return Object.fromEntries(Object.keys(value).sort().map((key) => [key, sortedKeys(value[key])]));
  };
  const projectKey = (project = state.project) => JSON.stringify(sortedKeys({ ...project, name: "", steps: project.steps.map((s) => ({ ...s, note: "" })) }));

  function saveDraft() {
    try {
      localStorage.setItem(DRAFT_KEY, JSON.stringify(state.project));
    } catch {
      // Private windows refuse storage; the draft then lives only in this tab.
    }
  }

  function changed({ replan = true } = {}) {
    saveDraft();
    state.graphCache = null;
    renderAll();
    if (replan) schedulePlan();
  }

  function edit(mutate, options) {
    const before = JSON.stringify(state.project);
    mutate(state.project);
    if (JSON.stringify(state.project) === before) return;
    state.history.push(before);
    state.history.length = Math.min(state.history.length, 300);
    state.future = [];
    changed(options);
  }

  function beginDrag() {
    return JSON.stringify(state.project);
  }

  function finishDrag(before) {
    if (JSON.stringify(state.project) === before) return;
    state.history.push(before);
    state.future = [];
    changed();
  }

  function undo() {
    if (!state.history.length) return log("Nothing to undo.");
    state.future.push(JSON.stringify(state.project));
    state.project = JSON.parse(state.history.pop());
    keepSelectionValid();
    changed();
  }

  function redo() {
    if (!state.future.length) return log("Nothing to redo.");
    state.history.push(JSON.stringify(state.project));
    state.project = JSON.parse(state.future.pop());
    keepSelectionValid();
    changed();
  }

  function keepSelectionValid() {
    state.selected = Math.max(0, Math.min(state.selected, state.project.steps.length - 1));
    const step = selectedStep();
    if (step.type !== "path" || state.point === null || state.point >= step.points.length) state.point = null;
  }

  function replaceProject(project, message) {
    state.history.push(JSON.stringify(state.project));
    state.future = [];
    state.project = project;
    state.selected = 0;
    state.point = null;
    state.plan = null;
    state.planFor = null;
    state.planError = null;
    if (!state.activeRun) {
      state.shown = null;
      state.compare = null;
      state.time = 0;
    }
    keepSelectionValid();
    fitView();
    changed();
    checkEngine();
    if (message) log(message, "ok");
  }

  function selectStep(index, point = null) {
    state.selected = index;
    state.point = point;
    state.tab = "step";
    renderAll();
  }

  function newStep(type, after) {
    const info = state.config.step_types[type];
    const step = { type, note: "", timeout_ms: 4000, volts: type === "wall_reset" ? 6 : 12, chain: false, actions: [], ...clone(info.defaults) };
    const pose = poses()[after]?.end ?? state.project.start;
    const ahead = along(pose.heading, 24);
    if ("x" in step && type !== "wall_reset") {
      step.x = round(pose.x + ahead.x, 0.5);
      step.y = round(pose.y + ahead.y, 0.5);
    }
    if (type === "turn") step.heading = wrapDegrees(Math.round(pose.heading + 90));
    if (type === "boomerang") step.heading = Math.round(pose.heading);
    if (type === "wall_reset") Object.assign(step, { x: round(pose.x, 0.5), y: round(pose.y, 0.5), heading: Math.round(pose.heading) });
    if (["arc", "swing"].includes(type)) step.heading = wrapDegrees(Math.round(pose.heading + 90));
    if (type === "reverse_arc") step.heading = wrapDegrees(Math.round(pose.heading - 90));
    return step;
  }

  function addStep(type) {
    const steps = state.project.steps;
    let index = state.selected + 1;
    if (steps.at(-1)?.type === "wall_reset" && index >= steps.length) index = steps.length - 1;
    if (type === "wall_reset" && steps.some((s) => s.type === "wall_reset")) {
      return log("There is already a wall reset. A routine can have one, as its last step.", "error");
    }
    if (steps.length >= state.config.limits.max_steps) return log(`A routine holds up to ${state.config.limits.max_steps} steps.`, "error");
    const step = newStep(type, index - 1);
    edit((project) => project.steps.splice(type === "wall_reset" ? project.steps.length : index, 0, step));
    state.selected = type === "wall_reset" ? steps.length - 1 : index;
    state.point = null;
    $("add-step").open = false;
    renderAll();
    log(`Added step ${state.selected + 1}: ${stepInfo(step).label}.${type === "path" ? " Click the field to add its points." : ""}`);
  }

  function deleteStep(index) {
    if (state.project.steps.length === 1) return log("A routine needs at least one step. Change this one, or start a new routine.", "error");
    const label = stepInfo(state.project.steps[index]).label;
    edit((project) => project.steps.splice(index, 1));
    state.selected = Math.min(index, state.project.steps.length - 1);
    state.point = null;
    renderAll();
    log(`Deleted step ${index + 1} (${label}). Ctrl+Z brings it back.`);
  }

  function moveStep(index, delta) {
    const target = index + delta;
    const steps = state.project.steps;
    if (target < 0 || target >= steps.length) return;
    if (steps[index].type === "wall_reset" || steps[target].type === "wall_reset") {
      return log("The wall reset has to stay last: it moves the odometry frame.", "error");
    }
    edit((project) => {
      const [step] = project.steps.splice(index, 1);
      project.steps.splice(target, 0, step);
    });
    state.selected = target;
    renderAll();
  }

  function addPoint(position) {
    const step = selectedStep();
    if (step.points.length >= state.config.limits.max_points) {
      return log(`A path holds up to ${state.config.limits.max_points} points. Add another path step for more.`, "error");
    }
    edit(() => {
      step.points.push({ x: position.x, y: position.y, heading: null, handle_in: null, handle_out: null });
      step.sections.push({ speed_percent: null, reverse: false });
    });
    state.point = step.points.length - 1;
    renderAll();
  }

  function insertPoint(sectionIndex, position) {
    const step = selectedStep();
    if (step.points.length >= state.config.limits.max_points) {
      return log(`A path holds up to ${state.config.limits.max_points} points.`, "error");
    }
    edit(() => {
      step.points.splice(sectionIndex, 0, { x: position.x, y: position.y, heading: null, handle_in: null, handle_out: null });
      step.sections.splice(sectionIndex, 0, clone(step.sections[sectionIndex]));
    });
    state.point = sectionIndex;
    renderAll();
    log(`Inserted point ${sectionIndex + 1} into step ${state.selected + 1}.`);
  }

  function deletePoint(index) {
    const step = selectedStep();
    edit(() => {
      step.points.splice(index, 1);
      step.sections.splice(index, 1);
    });
    state.point = step.points.length ? Math.min(index, step.points.length - 1) : null;
    renderAll();
  }

  function resetPoint(index) {
    const step = selectedStep();
    edit(() => Object.assign(step.points[index], { heading: null, handle_in: null, handle_out: null }));
    log(`Point ${index + 1}: heading and curve handles are automatic again.`);
  }

  function pathSteps() {
    return state.project.steps.filter((step) => step.type === "path" && step.points.length);
  }

  function currentPlan() {
    return state.plan && state.planFor === projectKey() ? state.plan : null;
  }

  function schedulePlan(delay = 200) {
    clearTimeout(state.planTimer);
    state.planTimer = setTimeout(requestPlan, delay);
  }

  async function requestPlan() {
    if (!pathSteps().length) {
      state.planError = null;
      renderAll();
      return;
    }
    if (!state.engine?.ready) return;
    if (state.planInFlight) {
      state.planAgain = true;
      return;
    }
    const key = projectKey();
    const project = clone(state.project);
    state.planInFlight = true;
    try {
      const answer = await api("/api/plan", { method: "POST", body: project });
      if (answer.status === 409) {
        checkEngine();
      } else if (!answer.ok) {
        state.planError = { message: answer.data.error, step: answer.data.step ?? null };
        state.plan = null;
      } else {
        const first = !state.plan;
        state.plan = answer.data;
        state.planFor = key;
        state.planError = null;
        if (first) log(`Speeds planned with mclib's trajectory planner (top speed ${fmt(answer.data.settings.top_speed_ips)} in/s).`, "ok");
      }
    } catch (error) {
      state.planError = { message: `Could not reach the planner: ${error.message}`, step: null };
    } finally {
      state.planInFlight = false;
      state.graphCache = null;
      renderAll();
      if (state.planAgain) {
        state.planAgain = false;
        schedulePlan(0);
      }
    }
  }

  function planFor(index) {
    const plan = currentPlan();
    return plan ? plan.plans[index] : null;
  }

  async function checkEngine() {
    clearTimeout(state.engineTimer);
    const robot = state.project.robot;
    try {
      let answer = await api(`/api/engine?preset=${robot.preset}&tracking=${robot.tracking}`);
      if (answer.ok && !answer.data.ready && answer.data.task?.status !== "running" && answer.data.task?.status !== "error") {
        answer = await api("/api/prepare", { method: "POST", body: { preset: robot.preset, tracking: robot.tracking } });
        if (answer.ok) log(`Preparing to plan for ${preset().label}: compiling mclib and measuring the robot.`);
      }
      if (!answer.ok) throw new Error(answer.data.error);
      const wasReady = state.engine?.ready && state.engine.preset === robot.preset && state.engine.tracking === robot.tracking;
      state.engine = answer.data;
      if (state.engine.ready && !wasReady) {
        const ff = state.engine.feedforward;
        log(`Robot measured: kS ${fmt(ff.ks, 2)} V, kV ${fmt(ff.kv, 4)} V per in/s, top speed ${fmt(ff.top_ips)} in/s, track width ${fmt(ff.track_in)} in.`, "ok");
        schedulePlan(0);
      }
      if (state.engine.task?.status === "error") log(`Could not prepare the planner: ${state.engine.task.error}`, "error");
      if (!state.engine.ready && state.engine.task?.status === "running") state.engineTimer = setTimeout(checkEngine, 1000);
    } catch (error) {
      state.engine = { ready: false, error: error.message };
      state.engineTimer = setTimeout(checkEngine, 3000);
    }
    renderPill();
    renderActivity();
  }

  async function runSimulation() {
    if (state.activeRun) return log("A run is already going. Wait for it, or cancel it.", "error");
    const answer = await api("/api/runs", { method: "POST", body: clone(state.project) }).catch((error) => ({ ok: false, data: { error: error.message } }));
    if (!answer.ok) {
      const step = answer.data.step;
      if (Number.isInteger(step)) selectStep(step);
      log(`Run not started. ${answer.data.error}`, "error");
      return;
    }
    const job = answer.data;
    state.activeRun = job.id;
    state.frames = [];
    state.shown = { ...job, live: { steps: [], frames: [] } };
    state.time = 0;
    state.graphCache = null;
    log(`Run ${job.number} started.`);
    pollRun();
    renderAll();
  }

  async function cancelRun() {
    if (!state.activeRun) return;
    await api(`/api/runs/${state.activeRun}/cancel`, { method: "POST", body: {} });
    log("Cancelling the run.");
  }

  async function pollRun() {
    clearTimeout(state.pollTimer);
    const id = state.activeRun;
    if (!id) return;
    let answer;
    try {
      answer = await api(`/api/runs/${id}?frames_from=${state.frames.length}`);
    } catch (error) {
      state.pollTimer = setTimeout(pollRun, 1000);
      return;
    }
    if (!answer.ok || state.activeRun !== id) return;
    const job = answer.data;
    if (job.live) {
      state.frames.push(...job.live.frames);
      job.live.frames = state.frames;
    }
    const finished = ["passed", "failed", "cancelled", "error"].includes(job.status);
    if (state.shown?.id === id) {
      state.shown = job;
      if (!finished) state.time = state.frames.at(-1)?.t ?? 0;
      state.graphCache = null;
    }
    if (finished) {
      state.activeRun = null;
      reportRun(job);
      if (state.shown?.id === id) state.time = trace().at(-1)?.t ?? 0;
      refreshRuns();
    } else {
      state.pollTimer = setTimeout(pollRun, 250);
    }
    renderAll();
  }

  function reportRun(job) {
    const summary = job.result?.summary ?? job.summary;
    if (job.status === "error") {
      log(`Run ${job.number} stopped with an error${job.error_stage ? ` while it was in "${stageLabel(job, job.error_stage)}"` : ""}: ${job.error}`, "error");
      if (Number.isInteger(job.error_step)) selectStep(job.error_step);
      return;
    }
    if (job.status === "cancelled") return log(`Run ${job.number} was cancelled.`);
    const wall = (job.finished_at - job.started_at).toFixed(1);
    log(`Run ${job.number}: ${summary.passed_steps} of ${summary.steps} steps passed, ${fmt(summary.simulated_s, 2)} s of robot time (took ${wall} s).`, job.status === "passed" ? "ok" : "error");
    for (const row of job.result.steps) {
      if (row.executed && !row.passed) log(`Step ${row.index + 1} (${state.config.step_types[row.type].label}) failed: ${row.reason}`, "error");
    }
  }

  function stageLabel(job, id) {
    return job.progress?.stages?.find((stage) => stage.id === id)?.label ?? id;
  }

  async function refreshRuns() {
    const answer = await api("/api/runs").catch(() => null);
    if (!answer?.ok) return;
    state.runs = answer.data.runs;
    renderRunPickers();
  }

  async function loadRun(id, into) {
    if (!id) {
      state[into] = null;
      state.graphCache = null;
      if (into === "shown") state.time = 0;
      renderAll();
      return;
    }
    const answer = await api(`/api/runs/${id}`);
    if (!answer.ok) return log(answer.data.error, "error");
    const job = answer.data;
    if (into === "shown" && job.live) state.frames = job.live.frames;
    state[into] = job;
    state.graphCache = null;
    if (into === "shown") state.time = trace().at(-1)?.t ?? 0;
    renderAll();
  }

  function trace(job = state.shown) {
    if (!job) return [];
    return job.result?.trace ?? job.live?.frames ?? [];
  }

  function stepRows(job = state.shown) {
    return job?.result?.steps ?? job?.live?.steps ?? [];
  }

  function timeline() {
    const frames = trace();
    if (state.shown) return { kind: "run", end: frames.at(-1)?.t ?? 0 };
    const plan = planFor(state.selected);
    if (plan) return { kind: "plan", end: plan.duration_s, plan };
    return { kind: "none", end: 0 };
  }

  function setTime(time) {
    const end = timeline().end;
    state.time = Math.max(0, Math.min(end, time));
    renderPlayback();
    drawField();
    drawGraph();
    renderTelemetry();
    sync3d();
  }

  function play(on = !state.playing) {
    const end = timeline().end;
    if (!end) return log("Nothing to play yet. Run a simulation, or select a planned path to preview it.");
    if (on && state.time >= end - 1e-6) state.time = 0;
    state.playing = on;
    state.lastTick = performance.now();
    if (on) requestAnimationFrame(tick);
    renderPlayback();
  }

  function tick(now) {
    if (!state.playing) return;
    const dt = (now - state.lastTick) / 1000;
    state.lastTick = now;
    const end = timeline().end;
    setTime(state.time + dt * state.speed);
    if (state.time >= end) {
      state.playing = false;
      renderPlayback();
      return;
    }
    requestAnimationFrame(tick);
  }

  function frameNow() {
    const frames = trace();
    const index = frameIndexAt(frames, state.time);
    return index >= 0 ? frames[index] : null;
  }

  function stepAtTime() {
    const frame = frameNow();
    return frame ? frame.step : null;
  }

  function renderAll() {
    $("routine-name").value = state.project.name;
    $("undo").disabled = !state.history.length;
    $("redo").disabled = !state.future.length;
    renderSteps();
    renderPane();
    renderPill();
    renderActivity();
    renderRunPickers();
    renderPlayback();
    renderHint();
    drawField();
    drawGraph();
    sync3d();
  }

  function stepSummary(step, index) {
    const p = (x, y) => `(${trim(x)}, ${trim(y)})`;
    switch (step.type) {
      case "path": {
        if (!step.points.length) return "No points yet. Click the field to add them.";
        const plan = planFor(index);
        const count = `${step.points.length} point${step.points.length === 1 ? "" : "s"}`;
        return plan ? `${count} · ${fmt(plan.duration_s, 2)} s · top ${fmt(plan.top_speed_ips)} in/s` : `${count} · speeds not planned yet`;
      }
      case "drive": return `${trim(Math.abs(step.distance))} in ${step.distance < 0 ? "backward" : "forward"}`;
      case "turn": return `to face ${trim(step.heading)}°`;
      case "turn_to_point": return `${step.direction === -1 ? "back toward" : "face"} ${p(step.x, step.y)}`;
      case "point": return `to ${p(step.x, step.y)}${step.direction === -1 ? ", backward" : ""}`;
      case "boomerang": return `to ${p(step.x, step.y)} facing ${trim(step.heading)}°`;
      case "arc": case "reverse_arc": return `to ${trim(step.heading)}°, radius ${trim(Math.abs(step.radius))} in ${step.radius < 0 ? "left" : "right"}`;
      case "swing": return `to ${trim(step.heading)}°${step.direction === -1 ? ", backward" : ""}`;
      case "wall_reset": return `set odometry to ${p(step.x, step.y)}, ${trim(step.heading)}°`;
      case "arm": return `to ${state.config.fields.arm_preset.choices[step.arm_preset]}`;
      case "claw": return state.config.fields.claw_preset.choices[step.claw_preset];
      default: return "";
    }
  }

  function actionsSummary(step) {
    const rules = state.config.actions;
    return (step.actions ?? []).map((action) => {
      const mechanism = rules.mechanisms[action.mechanism];
      const when = action.when === "start" ? "at the start" : action.when === "distance" ? `after ${trim(action.value)} in` : `after ${trim(action.value)} ms`;
      return `${mechanism.label} to ${mechanism.presets[action.preset]} ${when}`;
    }).join("; ");
  }

  function resultBadge(row) {
    if (!row) return null;
    if (row.executed === false) return element("span", { class: "badge skip", text: "skipped" });
    if (typeof row.passed !== "boolean") return element("span", { class: "badge run", text: "running" });
    const miss = row.position_error_in ?? null;
    const turn = row.heading_error_deg ?? null;
    const detail = Number.isFinite(miss) ? ` ${fmt(miss, 1)} in` : Number.isFinite(turn) ? ` ${fmt(turn, 1)}°` : "";
    return element("span", { class: `badge ${row.passed ? "pass" : "fail"}`, title: row.reason, text: `${row.passed ? "✓" : "✗"}${detail}` });
  }

  function renderSteps() {
    const start = state.project.start;
    const card = $("start-card");
    card.replaceChildren(element("b", { text: "Start" }), ` at (${trim(start.x)}, ${trim(start.y)}) facing ${trim(start.heading)}°`);
    $("step-count").textContent = `${state.project.steps.length} step${state.project.steps.length === 1 ? "" : "s"}`;
    const rows = stepRows();
    const live = stepAtTime();
    const items = state.project.steps.map((step, index) => {
      const selected = index === state.selected;
      const problem = poses()[index]?.problem || (state.planError?.step === index ? state.planError.message : null);
      const row = rows.find((item) => item.index === index);
      const actions = selected ? element("div", { class: "step-actions" },
        element("button", { class: "small", title: "Move this step earlier", onclick: (e) => { e.stopPropagation(); moveStep(index, -1); }, disabled: index === 0 }, "↑"),
        element("button", { class: "small", title: "Move this step later", onclick: (e) => { e.stopPropagation(); moveStep(index, 1); }, disabled: index === state.project.steps.length - 1 }, "↓"),
        element("button", { class: "small", title: "Add a copy of this step after it", onclick: (e) => { e.stopPropagation(); duplicateStep(index); }, disabled: step.type === "wall_reset" }, "Duplicate"),
        element("button", { class: "small danger", title: "Delete this step (Ctrl+Z undoes it)", onclick: (e) => { e.stopPropagation(); deleteStep(index); } }, "Delete")) : null;
      return element("li", {
        class: `step${selected ? " selected" : ""}${live === index && state.shown ? " playing" : ""}`,
        onclick: () => selectStep(index), "data-step": index,
      },
      element("div", { class: "step-top" },
        element("span", { class: "step-number", text: index + 1 }),
        element("span", { class: "step-title", text: step.note ? `${stepInfo(step).label}: ${step.note}` : stepInfo(step).label }),
        resultBadge(row)),
      element("div", { class: "step-sub", text: stepSummary(step, index) }),
      step.chain ? element("div", { class: "step-chain", text: `Keeps moving into step ${index + 2}` }) : null,
      step.actions?.length ? element("div", { class: "step-actions-list", text: actionsSummary(step) }) : null,
      problem ? element("div", { class: "step-problem", text: problem }) : null,
      actions);
    });
    $("step-list").replaceChildren(...items);
  }

  function duplicateStep(index) {
    const copy = clone(state.project.steps[index]);
    edit((project) => project.steps.splice(index + 1, 0, copy));
    state.selected = index + 1;
    renderAll();
  }

  function renderPill() {
    const pill = $("engine-pill");
    const engine = state.engine;
    let text = "Connecting to the planner…", kind = "";
    if (state.activeRun && state.shown?.progress) {
      const step = state.shown.progress.step;
      const active = state.shown.progress.stages.find((stage) => stage.status === "active");
      text = step ? `Run ${state.shown.number}: step ${step.index + 1} of ${step.total}` : `Run ${state.shown.number}: ${active?.label ?? "waiting"}`;
      kind = "busy";
    } else if (engine?.error) {
      text = `Planner not reachable: ${engine.error}`;
      kind = "bad";
    } else if (engine?.ready) {
      text = `Ready · robot top speed ${fmt(engine.feedforward.top_ips, 0)} in/s`;
      kind = "ok";
    } else if (engine?.task?.status === "running") {
      const active = engine.task.progress?.stages?.find((stage) => stage.status === "active");
      const seconds = Math.round(Date.now() / 1000 - engine.task.started_at);
      text = `${active?.id === "measure" ? "Measuring the robot" : "Compiling mclib"}… ${seconds} s`;
      kind = "busy";
    } else if (engine?.task?.status === "error") {
      text = "Could not prepare the planner. See Activity.";
      kind = "bad";
    }
    pill.textContent = text;
    pill.className = `pill ${kind}`;
    pill.title = text;
    $("run").hidden = !!state.activeRun;
    $("cancel-run").hidden = !state.activeRun;
  }

  function renderActivity() {
    const job = state.shown;
    const summary = $("run-summary");
    let stages = null;
    if (job) {
      stages = job.progress?.stages ?? [];
      const status = job.status;
      summary.hidden = false;
      const result = job.result?.summary;
      if (status === "passed" || status === "failed") {
        summary.className = `run-summary ${status === "passed" ? "pass" : "fail"}`;
        summary.replaceChildren(`Run ${job.number}: ${result.passed_steps} of ${result.steps} steps passed`,
          element("span", { text: `${fmt(result.simulated_s, 2)} s of robot time${result.skipped_steps ? `, ${result.skipped_steps} skipped after a failure` : ""}.` }));
      } else if (status === "error") {
        summary.className = "run-summary error";
        summary.replaceChildren(`Run ${job.number} stopped with an error`, element("span", { text: job.error }));
      } else if (status === "cancelled") {
        summary.className = "run-summary";
        summary.replaceChildren(`Run ${job.number} was cancelled.`);
      } else {
        summary.className = "run-summary";
        summary.replaceChildren(status === "queued" ? `Run ${job.number} is waiting to start…` : `Run ${job.number} is running…`);
      }
    } else {
      summary.hidden = true;
      if (state.engine?.task?.progress?.stages) stages = state.engine.task.progress.stages;
    }
    const icons = { waiting: "○", active: "●", done: "✓", skipped: "–", failed: "✗" };
    $("stage-list").replaceChildren(...(stages || []).map((stage) => {
      const seconds = stage.status === "active" ? Date.now() / 1000 - stage.started_at : stage.seconds;
      const shown = Number.isFinite(seconds) ? `${seconds.toFixed(seconds < 10 ? 2 : 1)} s` : "";
      return element("li", { class: `stage ${stage.status}` },
        element("span", { class: "icon", text: icons[stage.status] }),
        element("span", { class: "label", text: stage.label }),
        element("span", { class: "time", text: stage.status === "skipped" ? "skipped" : shown }),
        stage.detail ? element("span", { class: "detail", text: stage.detail }) : null);
    }));
    $("activity-clock").textContent = job ? `Run ${job.number}` : stages ? "Preparing" : "";
  }

  function renderLog() {
    $("log").replaceChildren(...state.log.map((item) => element("li", { class: item.kind },
      element("time", { text: item.time }), element("span", { text: item.message }))));
  }

  function runLabel(run) {
    const time = new Date(run.created_at * 1000).toTimeString().slice(0, 5);
    const outcome = run.summary ? `${run.summary.passed_steps}/${run.summary.steps} passed` : run.status;
    return `Run ${run.number} · ${outcome} · ${time}`;
  }

  function renderRunPickers() {
    const fill = (select, empty, current) => {
      const options = [element("option", { value: "", text: empty })];
      for (const run of state.runs) options.push(element("option", { value: run.id, text: runLabel(run) }));
      if (current && !state.runs.some((run) => run.id === current.id)) {
        options.splice(1, 0, element("option", { value: current.id, text: `Run ${current.number} · ${current.status}` }));
      }
      select.replaceChildren(...options);
      select.value = current?.id ?? "";
    };
    fill($("run-picker"), "the plan (no run)", state.shown);
    fill($("compare-picker"), "nothing", state.compare);
  }

  function renderPlayback() {
    const span = timeline();
    const slider = $("timeline");
    slider.max = String(Math.max(0.001, span.end));
    slider.value = String(state.time);
    slider.disabled = !span.end;
    $("play").disabled = !span.end;
    $("play").textContent = state.playing ? "❚❚" : "▶";
    $("play").setAttribute("aria-label", state.playing ? "Pause" : "Play");
    $("clock").textContent = `${fmt(state.time, 2)} / ${fmt(span.end, 2)} s`;
    const marks = [];
    if (span.kind === "run" && span.end) {
      for (const row of stepRows()) {
        if (!Number.isFinite(row.start_time_s)) continue;
        const mark = element("i", { class: row.executed === false ? "" : row.passed ? "pass" : typeof row.passed === "boolean" ? "fail" : "", title: `Step ${row.index + 1}` });
        mark.style.left = `${(row.start_time_s / span.end) * 100}%`;
        marks.push(mark);
      }
    }
    $("timeline-marks").replaceChildren(...marks);
    const stale = state.shown?.project && projectKey(state.shown.project) !== projectKey();
    $("stale-badge").hidden = !stale;
  }

  function renderHint() {
    const step = selectedStep();
    let hint = "";
    if (state.mode === "3d") hint = state.shown ? "vexsim's 3D view of the simulated robot. Click it, then press 1 to 4 to change the camera." : "";
    else if (state.drag?.kind === "pan") hint = "";
    else if (state.hover?.hint) hint = state.hover.hint;
    else if (step.type === "path" && !step.points.length) hint = `Click on the field to add points to step ${state.selected + 1}. The path starts where the robot is.`;
    else if (step.type === "path") hint = `Click to add a point to the end of step ${state.selected + 1}. Click its line to insert one. Drag points and handles to shape it.`;
    else if (["point", "turn_to_point", "boomerang", "wall_reset"].includes(step.type)) hint = `Click or drag to place the target of step ${state.selected + 1} (${stepInfo(step).label}).`;
    else if (["turn", "arc", "reverse_arc", "swing"].includes(step.type)) hint = `Drag the round knob to set the heading of step ${state.selected + 1}.`;
    else if (step.type === "drive") hint = `Drag the round knob to set how far step ${state.selected + 1} drives.`;
    $("field-hint").textContent = hint;
  }

  function numberInput({ label, unit = "", value, min, max, step = 1, help, onCommit, disabled = false, placeholder }) {
    const input = element("input", { type: "number", min, max, step, value: Number.isFinite(value) ? String(value) : "", disabled, placeholder });
    const wrapper = element("label", { class: "field-label" },
      element("span", {}, label, unit ? element("span", { class: "unit", text: ` (${unit})` }) : null),
      input, help ? element("span", { class: "help", text: help }) : null);
    const message = element("span", { class: "help" });
    input.addEventListener("change", () => {
      const parsed = Number(input.value);
      if (input.value.trim() === "" || !Number.isFinite(parsed) || parsed < min || parsed > max) {
        wrapper.classList.add("invalid");
        message.textContent = `${label} must be a number from ${min} to ${max}${unit ? ` ${unit}` : ""}. Not saved.`;
        wrapper.append(message);
        return;
      }
      wrapper.classList.remove("invalid");
      message.remove();
      onCommit(parsed);
    });
    return wrapper;
  }

  function fieldInput(step, key) {
    const info = state.config.fields[key];
    if (info.choices) {
      return selectInput(info.label, String(step[key]), Object.entries(info.choices), info.help,
        (value) => edit(() => { step[key] = key === "direction" ? Number(value) : value; }));
    }
    return numberInput({ label: info.label, unit: info.unit, value: step[key], min: info.low, max: info.high, step: info.step,
      help: info.help, onCommit: (value) => edit(() => { step[key] = key === "timeout_ms" ? Math.round(value) : value; }) });
  }

  function selectInput(label, value, choices, help, onChange) {
    const select = element("select", {}, ...choices.map(([key, text]) => element("option", { value: key, text })));
    select.value = value;
    select.addEventListener("change", () => onChange(select.value));
    return element("label", { class: "field-label" }, label, select, help ? element("span", { class: "help", text: help }) : null);
  }

  function checkInput(label, checked, help, onChange) {
    const input = element("input", { type: "checkbox" });
    input.checked = checked;
    input.addEventListener("change", () => onChange(input.checked));
    return element("label", { class: "check" }, input, element("span", {}, label, help ? element("span", { class: "help", text: help }) : null));
  }

  function chainReason(step, index) {
    const rules = state.config.chain;
    if (rules.not_allowed[step.type]) return rules.not_allowed[step.type];
    if (index === state.project.steps.length - 1) return rules.last_step;
    return null;
  }

  function chainInput(step, index) {
    const rules = state.config.chain;
    const reason = chainReason(step, index);
    const input = checkInput(rules.label, Boolean(step.chain), reason && !step.chain ? reason : rules.help,
      (on) => edit(() => { step.chain = on; }));
    input.querySelector("input").disabled = Boolean(reason) && !step.chain;
    return input;
  }

  function autoNumber({ label, unit, value, automatic, min, max, step, help, autoLabel, onCommit }) {
    const isAuto = value === null || value === undefined;
    const number = numberInput({ label, unit, value: isAuto ? Number(automatic.toFixed(1)) : value, min, max, step, help, disabled: isAuto, onCommit });
    const toggle = checkInput(autoLabel, isAuto, null, (on) => onCommit(on ? null : round(automatic, step)));
    return element("div", { class: "auto-row" }, number, toggle);
  }

  function renderPane() {
    for (const tab of ["step", "robot", "telemetry"]) {
      $(`tab-${tab}`).setAttribute("aria-selected", String(state.tab === tab));
      $(`pane-${tab}`).hidden = state.tab !== tab;
    }
    if (state.tab === "step") renderStepPane();
    if (state.tab === "robot") renderRobotPane();
    if (state.tab === "telemetry") renderTelemetry();
  }

  function resultCard(index) {
    const row = stepRows().find((item) => item.index === index);
    if (!row || typeof row.passed !== "boolean") return null;
    const kind = row.executed === false ? "" : row.passed ? "pass" : "fail";
    const title = row.executed === false ? "Skipped in this run" : row.passed ? `Passed in run ${state.shown.number}` : `Failed in run ${state.shown.number}`;
    return element("div", { class: `card ${kind}` },
      element("h3", { text: title }),
      row.executed === false ? element("p", { text: row.reason }) : null,
      row.checks?.length ? element("ul", { class: "checks" }, ...row.checks.map((check) =>
        element("li", { class: check.passed ? "pass" : "fail" }, element("span", { class: "mark", text: check.passed ? "✓" : "✗" }), element("span", { text: check.message })))) : null,
      row.info?.length ? element("ul", {}, ...row.info.map((text) => element("li", { text }))) : null,
      row.notes?.length ? element("ul", {}, ...row.notes.map((text) => element("li", { text: `Hint: ${text}` }))) : null,
      Number.isFinite(row.start_time_s) ? element("button", { class: "small", onclick: () => { play(false); setTime(row.start_time_s); } }, `Jump playback to step ${index + 1}`) : null);
  }

  function planCard(index) {
    const step = state.project.steps[index];
    if (!step.points.length) return element("div", { class: "card warn", text: "No points yet. Click on the field to add the first one. The path starts where the robot is at the start of this step." });
    if (state.planError?.step === index) return element("div", { class: "card fail" }, element("h3", { text: "Cannot plan this path" }), element("p", { text: state.planError.message }));
    const problem = poses()[index].problem;
    if (problem) return element("div", { class: "card fail", text: problem });
    const plan = planFor(index);
    if (!plan) {
      const waiting = !state.engine?.ready ? "Speeds are planned once mclib is compiled and the robot is measured. See Activity." : "Planning…";
      return element("div", { class: "card", text: waiting });
    }
    const settings = currentPlan().settings;
    const stats = [
      ["Planned time", `${fmt(plan.duration_s, 2)} s`],
      ["Length", `${fmt(plan.length_in)} in`],
      ["Top speed", `${fmt(plan.top_speed_ips)} in/s`],
      ["Tightest turn", plan.tightest_radius_in ? `${fmt(plan.tightest_radius_in)} in radius` : "straight"],
      ["Speed zones", `${plan.parts}`],
      ["Stops to reverse", `${plan.stops}`],
    ];
    if (plan.start_speed_ips > 0.05) stats.push(["Starts at", `${fmt(plan.start_speed_ips)} in/s, from step ${index}`]);
    if (plan.end_speed_ips > 0.05) stats.push(["Ends at", `${fmt(plan.end_speed_ips)} in/s, into step ${index + 2}`]);
    return element("div", { class: "card" },
      element("h3", { text: "Plan from mclib" }),
      element("dl", { class: "stats" }, ...stats.map(([label, value]) => element("div", {}, element("dt", { text: label }), element("dd", { text: value })))),
      element("p", { class: "muted small", text: `Planned with acceleration ${trim(settings.accel_ips2)} in/s², cornering ${trim(settings.cornering_ips2)} in/s², track width ${fmt(settings.track_width_in)} in. Robot top speed ${fmt(settings.top_speed_ips)} in/s.` }));
  }

  function renderStepPane() {
    const index = state.selected;
    const step = selectedStep();
    const info = stepInfo(step);
    const pane = $("pane-step");
    const children = [
      element("div", { class: "pane-title" }, element("h2", { text: `Step ${index + 1} · ${info.label}` }), element("span", { class: "call", text: info.call })),
      element("p", { class: "description", text: info.description }),
    ];
    const note = element("input", { type: "text", maxlength: 80, value: step.note || "", placeholder: "e.g. grab the goal" });
    note.addEventListener("change", () => edit(() => { step.note = note.value; }, { replan: false }));
    children.push(element("label", { class: "field-label" }, "Label (optional)", note, element("span", { class: "help", text: "Shown in the step list and as a comment in the exported C++." })));
    const result = resultCard(index);
    if (result) children.push(result);
    if (step.type === "path") children.push(...pathPane(step, index));
    else {
      const fields = info.fields.map((key) => fieldInput(step, key));
      children.push(element("div", { class: "form two" }, ...fields));
      children.push(element("h3", { text: "Limits" }));
      const limits = [fieldInput(step, "timeout_ms")];
      if (!info.mechanism && step.type !== "turn_to_point") limits.push(fieldInput(step, "volts"));
      children.push(element("div", { class: "form two" }, ...limits));
      if (!info.mechanism) {
        children.push(...actionsPane(step));
        children.push(element("h3", { text: "When it ends" }), chainInput(step, index));
      }
    }
    pane.replaceChildren(...children);
  }

  function pathPane(step, index) {
    const out = [planCard(index)];
    const layout = pathLayout(step, poses()[index].start);
    const defaults = state.project.path_defaults;
    const top = state.engine?.feedforward?.top_ips;
    out.push(element("h3", { text: `Points (${step.points.length})` }));
    if (!step.points.length) out.push(element("p", { class: "empty", text: "Click on the field to add points." }));
    const list = element("ol", { class: "points" });
    step.points.forEach((point, i) => {
      const section = step.sections[i];
      const selected = state.point === i;
      const heading = layout.headings[i + 1];
      const speed = section.speed_percent ?? defaults.speed_percent;
      const summary = `(${trim(point.x)}, ${trim(point.y)}) · ${trim(Math.round(heading))}°${point.heading === null ? " auto" : ""} · ${speed}%${section.reverse ? " · reverse" : ""}`;
      const item = element("li", { class: `point${selected ? " selected" : ""}` },
        element("button", { class: "point-head", onclick: () => { state.point = selected ? null : i; renderAll(); } },
          element("span", { class: "dot", text: i + 1 }), element("span", { class: "summary", text: summary })));
      if (selected) item.append(pointBody(step, i, layout, top));
      list.append(item);
    });
    out.push(list);
    out.push(element("h3", { text: "Whole path" }));
    out.push(element("p", { class: "muted small", text: `Sections without their own limit use the default speed limit, ${defaults.speed_percent}% of top speed${top ? ` (${fmt(defaults.speed_percent / 100 * top)} in/s)` : ""}. Change it under Robot & settings.` }));
    const plan = planFor(index);
    const limits = state.config.limits;
    const allowanceMs = step.chain ? limits.chain_allowance_ms : limits.settle_allowance_ms;
    const automaticTimeout = plan ? Math.ceil((plan.duration_s * 1000 + allowanceMs) / 100) * 100 : 8000;
    const timeoutHelp = step.chain
      ? `Automatic is the planned time plus ${trim(allowanceMs / 1000)} s. This path keeps moving, so it has no final turn or settle.`
      : `Automatic is the planned time plus ${trim(allowanceMs / 1000)} s for the final turn and settle.`;
    out.push(element("div", { class: "form" },
      step.points.length ? autoNumber({ label: "Curve handle at the start", unit: "in", value: step.start_handle, automatic: layout.sections[0].handleLengths[0], min: 0.5, max: 300, step: 0.5,
        help: "How far the path runs straight out of the robot's heading before it bends.", autoLabel: "Automatic", onCommit: (value) => edit(() => { step.start_handle = value; }) }) : null,
      autoNumber({ label: "Time limit", unit: "ms", value: step.timeout_ms, automatic: automaticTimeout, min: 50, max: 60000, step: 100,
        help: timeoutHelp, autoLabel: "Automatic", onCommit: (value) => edit(() => { step.timeout_ms = value === null ? null : Math.round(value); }) }),
      fieldInput(step, "volts")));
    out.push(...actionsPane(step));
    out.push(element("h3", { text: "When it ends" }), chainInput(step, index));
    return out;
  }

  function actionsPane(step) {
    if (state.project.field !== "override") return [];
    const rules = state.config.actions;
    const actions = step.actions ?? [];
    const firstValue = { start: 0, distance: 12, time: 500 };
    const rows = actions.map((action, i) => {
      const mechanism = rules.mechanisms[action.mechanism];
      const what = selectInput("Move", action.mechanism, Object.entries(rules.mechanisms).map(([key, item]) => [key, item.label]), null,
        (value) => edit(() => { action.mechanism = value; action.preset = Object.keys(rules.mechanisms[value].presets)[0]; }));
      const preset = selectInput("To", action.preset, Object.entries(mechanism.presets), null, (value) => edit(() => { action.preset = value; }));
      const when = selectInput("When", action.when, Object.entries(rules.times), null,
        (value) => edit(() => { action.when = value; action.value = firstValue[value]; }));
      const amount = action.when === "start" ? null : numberInput({
        label: action.when === "distance" ? "Distance" : "Time", unit: rules.units[action.when], value: action.value, min: 0,
        max: action.when === "distance" ? rules.max_distance_in : rules.max_time_ms, step: action.when === "distance" ? 1 : 50,
        onCommit: (value) => edit(() => { action.value = action.when === "time" ? Math.round(value) : value; }) });
      return element("div", { class: "section-box action-box", "data-action": i },
        element("div", { class: "form two" }, what, preset, when, amount),
        element("button", { class: "small danger", onclick: () => edit(() => { step.actions.splice(i, 1); }) }, "Remove"));
    });
    const add = element("button", { class: "small", id: "add-action", disabled: actions.length >= rules.max,
      onclick: () => edit(() => { step.actions = [...(step.actions ?? []), { when: "start", value: 0, mechanism: "arm", preset: "above_alliance_goal" }]; }) },
    "Add a lift or claw action");
    return [element("h3", { text: rules.label }), element("p", { class: "muted small", text: rules.help }), ...rows, add];
  }

  function pointBody(step, i, layout, top) {
    const point = step.points[i];
    const section = step.sections[i];
    const lengths = layout.sections[i].handleLengths;
    const next = layout.sections[i + 1];
    const defaults = state.project.path_defaults;
    const body = element("div", { class: "point-body" });
    body.append(element("div", { class: "form two" },
      numberInput({ label: "X", unit: "in", value: point.x, min: -500, max: 500, step: 0.5, onCommit: (value) => edit(() => { point.x = value; }) }),
      numberInput({ label: "Y", unit: "in", value: point.y, min: -500, max: 500, step: 0.5, onCommit: (value) => edit(() => { point.y = value; }) })));
    body.append(autoNumber({ label: "Heading here", unit: "°", value: point.heading, automatic: Math.round(layout.headings[i + 1]), min: -3600, max: 3600, step: 1,
      help: "Which way the robot faces at this point. Automatic follows the line from the point before to the point after.", autoLabel: "Automatic",
      onCommit: (value) => edit(() => { point.heading = value; }) }));
    body.append(autoNumber({ label: "Curve handle before", unit: "in", value: point.handle_in, automatic: lengths[1], min: 0.5, max: 300, step: 0.5,
      help: "Longer makes the curve arrive straighter.", autoLabel: "Automatic", onCommit: (value) => edit(() => { point.handle_in = value; }) }));
    if (next) {
      body.append(autoNumber({ label: "Curve handle after", unit: "in", value: point.handle_out, automatic: next.handleLengths[0], min: 0.5, max: 300, step: 0.5,
        help: "Longer makes the curve leave straighter.", autoLabel: "Automatic", onCommit: (value) => edit(() => { point.handle_out = value; }) }));
    }
    const speed = section.speed_percent;
    const box = element("div", { class: `section-box${section.reverse ? " reverse" : ""}` },
      element("strong", { text: `Section ${i + 1}: from ${i === 0 ? "the start" : `point ${i}`} to point ${i + 1}` }),
      autoNumber({ label: "Speed limit", unit: "% of top speed", value: speed, automatic: defaults.speed_percent, min: 5, max: 100, step: 1,
        help: top ? `100% is ${fmt(top)} in/s on this robot.` : "Top speed is measured in the simulator.", autoLabel: "Default",
        onCommit: (value) => edit(() => { section.speed_percent = value === null ? null : Math.round(value); }) }),
      checkInput("Reverse: back up along this section", section.reverse, "The robot stops before a section that changes direction.",
        (on) => edit(() => { section.reverse = on; })));
    body.append(box);
    body.append(element("div", { class: "row" },
      element("button", { class: "small", onclick: () => insertPoint(i, midpoint(layout.sections[i])) }, "Insert a point before"),
      element("button", { class: "small", onclick: () => resetPoint(i), disabled: point.heading === null && point.handle_in === null && point.handle_out === null }, "Make automatic"),
      element("button", { class: "small danger", onclick: () => deletePoint(i) }, "Delete point")));
    return body;
  }

  function midpoint(section) {
    const p = bezierPoint(section, 0.5);
    return { x: round(p.x, 0.5), y: round(p.y, 0.5) };
  }

  function renderRobotPane() {
    const project = state.project;
    const config = state.config;
    const robot = project.robot;
    const groupForm = (group, schema) => element("div", { class: "form two" }, ...Object.entries(schema).map(([key, info]) =>
      numberInput({ label: info.label, unit: info.unit, value: project[group][key], min: info.low ?? 0, max: info.high ?? 100, step: info.step ?? 0.01, help: info.help,
        onCommit: (value) => edit(() => { project[group][key] = key === "noise_seed" ? Math.round(value) : value; }) })));
    const robotChanged = () => { checkEngine(); };
    $("pane-robot").replaceChildren(
      element("h3", { text: "Field" }),
      element("div", { class: "form" },
        selectInput("Field", project.field, Object.entries(config.game_fields),
          "What the robot drives on. On a game field it hits the walls, Goals and Loaders, and the results say where and how hard.",
          (value) => edit(() => {
            project.field = value;
            if (value !== "override") project.preload = "none";
          }, { replan: false })),
        project.field === "override" ? selectInput(config.preload.label, project.preload ?? "none", Object.entries(config.preload.choices), config.preload.help,
          (value) => edit(() => { project.preload = value; }, { replan: false })) : null),
      element("h3", { text: "Robot" }),
      element("div", { class: "form" },
        selectInput("Drivetrain", robot.preset, Object.entries(config.presets).map(([key, value]) => [key, value.label]),
          "The simulated robot. Changing it measures the new robot once (a few seconds).", (value) => { edit(() => { robot.preset = value; }); robotChanged(); }),
        selectInput("Odometry sensors", robot.tracking, Object.entries(config.tracking), "Drive encoders slip; tracking wheels do not.",
          (value) => { edit(() => { robot.tracking = value; }); robotChanged(); }),
        selectInput("Heading from", robot.heading_source, Object.entries(config.heading_sources), "The IMU is the usual choice.",
          (value) => edit(() => { robot.heading_source = value; }))),
      element("h3", { text: "Start pose" }),
      element("p", { class: "muted small", text: "Where the robot starts. You can also drag the outlined robot and its knob on the field." }),
      element("div", { class: "form three" }, ...["x", "y", "heading"].map((key) => {
        const info = config.fields[key];
        return numberInput({ label: info.label, unit: info.unit, value: project.start[key], min: info.low, max: info.high, step: info.step, onCommit: (value) => edit(() => { project.start[key] = value; }) });
      })),
      element("h3", { text: "Paths" }),
      groupForm("path_defaults", config.path_defaults),
      element("h3", { text: "Simulated conditions" }),
      groupForm("conditions", config.conditions),
      element("h3", { text: "Controller gains (PID)" }),
      element("p", { class: "muted small", text: "Used by drive, turn, point, boomerang, arc and swing steps, and by the settle at the end of a path." }),
      element("div", { class: "form three" }, ...Object.entries(config.gains).map(([key, info]) =>
        numberInput({ label: info.label, value: project.gains[key], min: 0, max: 100, step: 0.05, onCommit: (value) => edit(() => { project.gains[key] = value; }) }))),
      element("h3", { text: "Run" }),
      checkInput("Stop at the first failed step", project.stop_on_failure, "Later steps are skipped, as the exported C++ returns early.",
        (on) => edit(() => { project.stop_on_failure = on; }, { replan: false })));
  }

  function renderTelemetry() {
    if (state.tab !== "telemetry") return;
    const pane = $("pane-telemetry");
    const frame = frameNow();
    if (!frame) {
      pane.replaceChildren(element("p", { class: "empty", text: "Telemetry shows the simulated robot at the playback time. Run a simulation first." }));
      return;
    }
    const row = stepRows().find((item) => item.index === frame.step);
    const step = state.shown?.project?.steps?.[frame.step];
    const heading = element("p", { class: "muted" }, `At ${fmt(frame.t, 2)} s · step ${frame.step + 1}${step ? ` (${state.config.step_types[step.type].label})` : ""} · ${frame.phase === "hold" ? "0.3 s pause after the step" : "moving"}`);
    const groups = telemetryRows(frame, row?.target).map(([title, rows]) => element("div", { class: "telemetry-group" },
      element("h3", { text: title }),
      element("dl", {}, ...rows.flatMap(([label, value]) => [element("dt", { text: label }), element("dd", { text: value })]))));
    pane.replaceChildren(heading, ...groups, element("p", { class: "muted small", text: "Simulated robot is the physics truth. Odometry is what mclib believed. A dash means the value was not recorded." }));
  }

  function resizeCanvas(canvas, cssHeight) {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(rect.width * ratio));
    const height = Math.max(1, Math.round((cssHeight ?? rect.height) * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { ctx, width: rect.width, height: cssHeight ?? rect.height };
  }

  function toScreen(point) {
    const { width, height } = state.size;
    return { x: (point.x - state.view.x) * state.view.scale + width / 2, y: height / 2 - (point.y - state.view.y) * state.view.scale };
  }

  function toField(x, y) {
    const { width, height } = state.size;
    return { x: (x - width / 2) / state.view.scale + state.view.x, y: (height / 2 - y) / state.view.scale + state.view.y };
  }

  function fitView() {
    const rect = $("field").getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const points = [{ x: -FIELD_HALF_IN, y: -FIELD_HALF_IN }, { x: FIELD_HALF_IN, y: FIELD_HALF_IN }, state.project.start];
    for (const item of poses()) points.push(item.end);
    for (const step of state.project.steps) if (step.type === "path") points.push(...step.points);
    for (const frame of trace()) points.push({ x: frame.true_x, y: frame.true_y });
    const xs = points.map((p) => p.x).filter(Number.isFinite), ys = points.map((p) => p.y).filter(Number.isFinite);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const side = 34, top = 44, bottom = 28;
    state.view.scale = Math.min((rect.width - 2 * side) / (maxX - minX), (rect.height - top - bottom) / (maxY - minY));
    state.view.x = (minX + maxX) / 2;
    state.view.y = (minY + maxY) / 2 + (top - bottom) / 2 / state.view.scale;
    state.viewMovedByUser = false;
    drawField();
  }

  function speedColor(fraction) {
    const f = Math.max(0, Math.min(1, fraction));
    for (let i = 1; i < SPEED_STOPS.length; i++) {
      const [b, cb] = SPEED_STOPS[i];
      const [a, ca] = SPEED_STOPS[i - 1];
      if (f <= b) {
        const t = (f - a) / (b - a);
        return `rgb(${ca.map((value, k) => Math.round(value + (cb[k] - value) * t)).join(",")})`;
      }
    }
    return "rgb(216,67,47)";
  }

  function fieldDrawing() {
    return state.config.field_drawings?.[state.project.field] ?? null;
  }

  function fillPolygon(ctx, points, fill, stroke = null, width = 1) {
    ctx.beginPath();
    points.forEach(([x, y], i) => {
      const p = toScreen({ x, y });
      if (i) ctx.lineTo(p.x, p.y); else ctx.moveTo(p.x, p.y);
    });
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke(); }
  }

  const ALLIANCE_FILL = { red: "#d9483b", blue: "#2f6bd8", yellow: "#e8b400", neutral: "#68707c", center: "#4b525c" };

  function drawOverrideField(ctx, drawing, frame) {
    const scale = state.view.scale;
    ctx.save();
    for (const quadrant of drawing.quadrants) {
      fillPolygon(ctx, quadrant.outline, quadrant.color === "red" ? "rgba(217,72,59,.07)" : "rgba(47,107,216,.07)");
    }
    const line = (points, width) => polyline(ctx, points.map(([x, y]) => ({ x, y })), "rgba(255,255,255,.95)", width);
    line(drawing.tape.auton_line, drawing.tape.auton_line_width_in * scale);
    line(drawing.tape.diagonal, Math.max(1, 0.5 * scale));
    line([...drawing.tape.midfield, drawing.tape.midfield[0]], Math.max(1.5, scale));
    for (const loader of drawing.loaders) {
      ctx.setLineDash([4, 3]);
      fillPolygon(ctx, loader.zone, null, ALLIANCE_FILL[loader.alliance], 1.2);
      ctx.setLineDash([]);
      fillPolygon(ctx, loader.outline, "#f4f5f7", ALLIANCE_FILL[loader.alliance], 2);
    }
    const toggleColors = frame?.toggles ?? {};
    for (const toggle of drawing.toggles) {
      const color = toggleColors[toggle.quadrant]?.inner ?? toggle.faces[0];
      fillPolygon(ctx, toggle.outline, ALLIANCE_FILL[color], "#c2408f", 2);
    }
    for (const goal of drawing.goals) {
      fillPolygon(ctx, goal.outline, ALLIANCE_FILL[goal.kind], "#20252c", 1.2);
      const centre = toScreen({ x: goal.center[0], y: goal.center[1] });
      ctx.beginPath();
      ctx.arc(centre.x, centre.y, 1.18 * scale, 0, 2 * Math.PI);
      ctx.fillStyle = "rgba(0,0,0,.35)";
      ctx.fill();
    }
    ctx.restore();
  }

  const PIECE_FILL = { red: "#d9483b", blue: "#2f6bd8", yellow: "#e8b400", gray: "#8b929c", clear: "rgba(214,232,244,.9)" };

  function runPieceList() {
    return state.shown?.result?.pieces ?? state.shown?.live?.pieces ?? null;
  }

  function designPreload(drawing) {
    const preload = state.project.preload ?? "none";
    if (preload === "none") return [];
    const start = state.project.start;
    const ahead = along(start.heading, preset().body_length_in / 2 + drawing.claw.ahead_in);
    const colors = ["red", "yellow"];
    const up = colors[1] === preload ? 1 : -1;
    return [{ id: "preload", kind: "pin", colors, pose: [start.x + ahead.x, start.y + ahead.y, drawing.piece_size.pin_length_in / 2, 0, 0, up] }];
  }

  // Every Pin and Cup at the playback time: the run's starting list moved by
  // each frame up to now. Without a run, the starting layout and the preload.
  function piecesNow() {
    const drawing = fieldDrawing();
    if (state.project.field !== "override" || !drawing?.pieces) return [];
    const list = runPieceList();
    if (!list) return [...drawing.pieces, ...designPreload(drawing)];
    const frames = trace();
    const index = frameIndexAt(frames, state.time);
    let cache = state.pieceCache;
    if (!cache || cache.frames !== frames || cache.list !== list || cache.index > index) {
      cache = { frames, list, index: -1, poses: new Map(list.map((piece) => [String(piece.id), piece.pose])) };
      state.pieceCache = cache;
    }
    for (let i = cache.index + 1; i <= index; i++) {
      for (const [id, pose] of Object.entries(frames[i].pieces ?? {})) cache.poses.set(id, pose);
    }
    cache.index = index;
    return list.map((piece) => ({ ...piece, pose: cache.poses.get(String(piece.id)) }));
  }

  function strokeBetween(ctx, a, b, color, widthIn) {
    const p = toScreen(a), q = toScreen(b);
    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1, widthIn * state.view.scale);
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(q.x, q.y);
    ctx.stroke();
  }

  function disc(ctx, centre, radiusIn, fill, stroke) {
    const p = toScreen(centre);
    ctx.beginPath();
    ctx.arc(p.x, p.y, Math.max(1, radiusIn * state.view.scale), 0, 2 * Math.PI);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  // Top view. An upright Pin shows its flange ring in the lower half's colour
  // around the upper half's cone. A lying piece shows both halves end to end.
  function drawPiece(ctx, piece, size) {
    const [x, y, , ax, ay, az] = piece.pose;
    const centre = { x, y };
    const upright = Math.abs(az) > 0.7;
    const length = piece.kind === "cup" ? size.cup_length_in : size.pin_length_in;
    const first = { x: x - ax * length / 2, y: y - ay * length / 2 };
    const second = { x: x + ax * length / 2, y: y + ay * length / 2 };
    const fill = (color) => PIECE_FILL[color] ?? "#888";
    ctx.save();
    ctx.lineCap = "butt";
    if (piece.kind === "cup") {
      if (upright) disc(ctx, centre, size.cup_rim_in / 2, fill("clear"), "#5d6570");
      else {
        strokeBetween(ctx, first, centre, fill(piece.colors[0]), size.cup_rim_in);
        strokeBetween(ctx, centre, second, fill(piece.colors[1]), size.cup_rim_in);
      }
    } else if (upright) {
      const [lower, upper] = az >= 0 ? piece.colors : [piece.colors[1], piece.colors[0]];
      disc(ctx, centre, size.pin_flange_in / 2, fill(lower), "#2b3038");
      disc(ctx, centre, size.pin_cone_in / 2, fill(upper), "#2b3038");
    } else {
      strokeBetween(ctx, first, centre, fill(piece.colors[0]), size.pin_cone_in);
      strokeBetween(ctx, centre, second, fill(piece.colors[1]), size.pin_cone_in);
      const across = Math.hypot(ax, ay) > 1e-6 ? { x: -ay / Math.hypot(ax, ay), y: ax / Math.hypot(ax, ay) } : { x: 1, y: 0 };
      const flange = size.pin_flange_in / 2;
      strokeBetween(ctx, { x: x - across.x * flange, y: y - across.y * flange }, { x: x + across.x * flange, y: y + across.y * flange }, "#2b3038", 0.5);
    }
    ctx.restore();
  }

  function drawPieces(ctx) {
    const drawing = fieldDrawing();
    const pieces = piecesNow().filter((piece) => piece.pose).sort((a, b) => a.pose[2] - b.pose[2]);
    for (const piece of pieces) drawPiece(ctx, piece, drawing.piece_size);
  }

  // The jaws in front of the robot. They move forward as the lift rises,
  // because the lift leans forward.
  function drawClaw(ctx, pose, armDeg, gapIn, size = null) {
    const claw = fieldDrawing()?.claw;
    if (state.project.field !== "override" || !claw) return;
    const body = size ?? preset();
    const lift = armDeg / claw.arm_full_deg * claw.lift_travel_in;
    const forward = body.body_length_in / 2 + claw.ahead_in + lift * Math.sin(claw.incline_deg * Math.PI / 180);
    const scale = state.view.scale;
    const p = toScreen(pose);
    const half = gapIn / 2 * scale, thick = claw.jaw_thickness_in * scale, length = claw.jaw_length_in * scale;
    const middle = -forward * scale;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(pose.heading * Math.PI / 180);
    ctx.fillStyle = "#3a414c";
    ctx.fillRect(-half - thick, middle - length / 2, thick, length);
    ctx.fillRect(half, middle - length / 2, thick, length);
    ctx.fillRect(-half - thick, middle + length / 2, 2 * half + 2 * thick, Math.max(1, thick));
    ctx.restore();
  }

  function renderScore() {
    const panel = $("score-panel");
    const frame = state.shown ? frameNow() : null;
    const show = state.mode === "top" && state.project.field === "override" && Number.isFinite(frame?.score_red);
    panel.hidden = !show;
    if (!show) return;
    const lines = [
      element("div", { class: "score-title", text: `Score at ${fmt(frame.t, 2)} s` }),
      element("div", { class: "score-line" },
        element("span", { class: "score-red", text: `Red ${frame.score_red}` }),
        element("span", { class: "score-blue", text: `Blue ${frame.score_blue}` })),
    ];
    if (frame.toggles) {
      const toggles = Object.entries(frame.toggles).map(([quadrant, toggle]) => `${quadrant} ${toggle.set}`).join(", ");
      lines.push(element("div", { class: "score-detail", text: `Toggles set: ${toggles}.` }));
    }
    const auton = state.shown?.result?.autonomous;
    if (auton && frame.t >= auton.t - 1e-6) {
      const point = auton.win_point;
      lines.push(
        element("div", { class: "score-detail", text: `Autonomous ended at ${fmt(auton.t, 2)} s: red ${auton.red}, blue ${auton.blue}. Autonomous Bonus: red ${auton.bonus.red}, blue ${auton.bonus.blue}.` }),
        element("div", { class: "score-detail", text: `Win Point: red owns ${point.pins} of ${point.pins_needed} Pin halves needed and has ${point.goals_with_two} of ${point.goals_needed} Goals with two${point.touching_perimeter ? ", and touches the wall" : ""}. ${point.earned ? "Earned." : "Not earned."}` }),
        element("div", { class: "score-detail", text: auton.robot_in_midfield ? "The robot ended in the Midfield. That counts only at the end of a match." : `The robot ended ${fmt(auton.robot_inches_from_midfield)} in outside the Midfield.` }));
    }
    panel.replaceChildren(...lines);
  }

  function drawPushBackField(ctx, drawing) {
    ctx.save();
    for (const zone of drawing.park_zones) fillPolygon(ctx, zone.outline, zone.alliance === "red" ? "rgba(217,72,59,.18)" : "rgba(47,107,216,.18)", ALLIANCE_FILL[zone.alliance], 1.5);
    for (const solid of drawing.solids) {
      const p = toScreen({ x: solid.center[0], y: solid.center[1] });
      ctx.beginPath();
      ctx.arc(p.x, p.y, solid.radius_in * state.view.scale, 0, 2 * Math.PI);
      ctx.fillStyle = "#68707c";
      ctx.fill();
    }
    ctx.restore();
  }

  function drawFieldBackground(ctx) {
    const drawing = fieldDrawing();
    const half = drawing?.half_in ?? FIELD_HALF_IN;
    const a = toScreen({ x: -half, y: half });
    const b = toScreen({ x: half, y: -half });
    ctx.fillStyle = "#e9ecf0";
    ctx.fillRect(0, 0, state.size.width, state.size.height);
    ctx.fillStyle = "#d9dde3";
    ctx.fillRect(a.x, a.y, b.x - a.x, b.y - a.y);
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        if ((i + j) % 2) continue;
        const p = toScreen({ x: -FIELD_HALF_IN + i * TILE_IN, y: FIELD_HALF_IN - j * TILE_IN });
        ctx.fillStyle = "#d4d8df";
        ctx.fillRect(p.x, p.y, TILE_IN * state.view.scale, TILE_IN * state.view.scale);
      }
    }
    ctx.strokeStyle = "#c3c9d1";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let v = -FIELD_HALF_IN; v <= FIELD_HALF_IN; v += TILE_IN) {
      const p = toScreen({ x: v, y: FIELD_HALF_IN }), q = toScreen({ x: v, y: -FIELD_HALF_IN });
      ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y);
      const r = toScreen({ x: -FIELD_HALF_IN, y: v }), s = toScreen({ x: FIELD_HALF_IN, y: v });
      ctx.moveTo(r.x, r.y); ctx.lineTo(s.x, s.y);
    }
    ctx.stroke();
    if (drawing && state.project.field === "override") drawOverrideField(ctx, drawing, frameNow());
    if (drawing && state.project.field === "push_back") drawPushBackField(ctx, drawing);
    ctx.strokeStyle = drawing ? "#2b3038" : "#5b6370";
    ctx.lineWidth = Math.max(2, (drawing ? 2 : 1.5) * state.view.scale);
    ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
    ctx.fillStyle = "#7d8694";
    ctx.font = "11px system-ui, sans-serif";
    ctx.textAlign = "center";
    for (let v = -FIELD_HALF_IN; v <= FIELD_HALF_IN; v += TILE_IN) {
      const p = toScreen({ x: v, y: -FIELD_HALF_IN });
      ctx.fillText(String(v), p.x, p.y + 16);
    }
    ctx.textAlign = "right";
    for (let v = -FIELD_HALF_IN; v <= FIELD_HALF_IN; v += TILE_IN) {
      const p = toScreen({ x: -FIELD_HALF_IN, y: v });
      ctx.fillText(String(v), p.x - 7, p.y + 4);
    }
    const origin = toScreen({ x: -FIELD_HALF_IN, y: -FIELD_HALF_IN });
    ctx.textAlign = "left";
    ctx.fillText("+X →", origin.x + 4, origin.y + 30);
    ctx.save();
    ctx.translate(origin.x - 30, origin.y - 4);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("+Y →", 0, 0);
    ctx.restore();
  }

  function polyline(ctx, points, color, width, dash = []) {
    if (points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    points.forEach((point, i) => {
      const p = toScreen(point);
      if (i) ctx.lineTo(p.x, p.y); else ctx.moveTo(p.x, p.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawRobot(ctx, pose, { fill = null, stroke = "#18202c", dash = [], width = 1.5, size = null } = {}) {
    const body = size ?? preset();
    const p = toScreen(pose);
    const length = body.body_length_in * state.view.scale, breadth = body.body_width_in * state.view.scale;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(pose.heading * Math.PI / 180);
    ctx.setLineDash(dash);
    ctx.lineWidth = width;
    ctx.strokeStyle = stroke;
    ctx.beginPath();
    ctx.roundRect(-breadth / 2, -length / 2, breadth, length, 3);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(-breadth * 0.28, -length / 2 + length * 0.22);
    ctx.lineTo(0, -length / 2 + 2);
    ctx.lineTo(breadth * 0.28, -length / 2 + length * 0.22);
    ctx.stroke();
    ctx.restore();
  }

  function drawKnob(ctx, point, color, filled = false, radius = HANDLE_RADIUS_PX) {
    const p = toScreen(point);
    ctx.save();
    ctx.beginPath();
    ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = filled ? color : "#fff";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = color;
    ctx.stroke();
    ctx.restore();
  }

  function drawLabel(ctx, point, text, color = "#18202c") {
    const p = toScreen(point);
    ctx.save();
    ctx.font = "600 11px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = color;
    ctx.fillText(text, p.x, p.y);
    ctx.restore();
  }

  function reverseMarker(ctx, section) {
    const a = toScreen(bezierPoint(section, 0.46)), b = toScreen(bezierPoint(section, 0.54));
    const angle = Math.atan2(b.y - a.y, b.x - a.x);
    ctx.save();
    ctx.translate((a.x + b.x) / 2, (a.y + b.y) / 2);
    ctx.rotate(angle);
    ctx.fillStyle = "#a16207";
    ctx.beginPath();
    ctx.moveTo(6, 0);
    ctx.lineTo(-4, -5);
    ctx.lineTo(-4, 5);
    ctx.closePath();
    ctx.fill();
    ctx.rotate(-angle);
    ctx.font = "600 10.5px system-ui, sans-serif";
    ctx.fillText("reverse", 9, -7);
    ctx.restore();
  }

  function stepTag(ctx, point, number, selected, placed) {
    const p = toScreen(point);
    const text = `Step ${number}`;
    ctx.save();
    ctx.font = "600 10.5px system-ui, sans-serif";
    const width = ctx.measureText(text).width + 10;
    let x = p.x + 14, y = p.y + 10;
    while (placed.some((other) => Math.abs(other.x - x) < width && Math.abs(other.y - y) < 16)) y += 17;
    placed.push({ x, y });
    ctx.fillStyle = selected ? "#2563eb" : "rgba(51,65,85,.88)";
    ctx.beginPath();
    ctx.roundRect(x, y - 8, width, 16, 8);
    ctx.fill();
    ctx.textBaseline = "middle";
    ctx.fillStyle = "#fff";
    ctx.fillText(text, x + 5, y + 0.5);
    ctx.restore();
  }

  function draggableKnobsInHitOrder() {
    const knobs = [];
    const all = poses();
    const index = state.selected;
    const step = selectedStep();
    const start = all[index].start;
    if (step.type === "path") {
      const layout = pathLayout(step, start);
      layout.sections.forEach((section, i) => {
        knobs.push({ kind: "handle-out", point: i - 1, at: section.startHandle, anchor: section.start });
        knobs.push({ kind: "handle-in", point: i, at: section.endHandle, anchor: section.end });
      });
      step.points.forEach((point, i) => knobs.push({ kind: "anchor", point: i, at: point }));
    } else if (["point", "turn_to_point", "boomerang", "wall_reset"].includes(step.type)) {
      if (["boomerang", "wall_reset"].includes(step.type)) {
        const tip = along(step.heading, HEADING_KNOB_IN);
        knobs.push({ kind: "target-heading", at: { x: step.x + tip.x, y: step.y + tip.y }, pivot: step });
      }
      knobs.push({ kind: "target", at: { x: step.x, y: step.y } });
    } else if (["turn", "arc", "reverse_arc", "swing"].includes(step.type)) {
      const pivot = step.type === "turn" ? start : all[index].end;
      const tip = along(step.heading, HEADING_KNOB_IN);
      knobs.push({ kind: "step-heading", at: { x: pivot.x + tip.x, y: pivot.y + tip.y }, pivot });
    } else if (step.type === "drive") {
      knobs.push({ kind: "drive-end", at: all[index].end, pivot: start });
    }
    const tip = along(state.project.start.heading, HEADING_KNOB_IN);
    knobs.push({ kind: "start-heading", at: { x: state.project.start.x + tip.x, y: state.project.start.y + tip.y } });
    knobs.push({ kind: "start", at: state.project.start, radius: Math.min(preset().body_width_in, preset().body_length_in) / 2 });
    return knobs;
  }

  function hitTest(sx, sy) {
    const knobs = draggableKnobsInHitOrder();
    for (const knob of knobs) {
      const p = toScreen(knob.at);
      const reach = knob.radius ? knob.radius * state.view.scale : (knob.kind === "anchor" ? ANCHOR_RADIUS_PX + 3 : HANDLE_RADIUS_PX + 4);
      if (Math.hypot(p.x - sx, p.y - sy) <= reach) return knob;
    }
    const all = poses();
    for (let s = 0; s < state.project.steps.length; s++) {
      const step = state.project.steps[s];
      if (s === state.selected || step.type !== "path") continue;
      for (let i = 0; i < step.points.length; i++) {
        const p = toScreen(step.points[i]);
        if (Math.hypot(p.x - sx, p.y - sy) <= ANCHOR_RADIUS_PX + 2) return { kind: "other-anchor", step: s, point: i };
      }
    }
    const step = selectedStep();
    if (step.type === "path" && step.points.length) {
      const layout = pathLayout(step, all[state.selected].start);
      let best = null;
      layout.sections.forEach((section, i) => {
        const samples = sectionSamples(section, 60);
        samples.forEach((point, k) => {
          const p = toScreen(point);
          const gap = Math.hypot(p.x - sx, p.y - sy);
          if (gap <= 7 && (!best || gap < best.gap) && k > 2 && k < samples.length - 3) best = { kind: "curve", section: i, at: point, gap };
        });
      });
      if (best) return best;
    }
    return null;
  }

  function drawPath(ctx, step, index, start, selected) {
    const layout = pathLayout(step, start);
    const plan = state.layers.plan ? planFor(index) : null;
    const dragging = state.drag && state.drag.kind !== "pan";
    if (plan && !dragging) {
      const top = currentPlan().settings.top_speed_ips;
      const states = plan.states;
      ctx.save();
      ctx.lineCap = "round";
      ctx.lineWidth = selected ? 5 : 3.5;
      ctx.globalAlpha = selected ? 1 : 0.75;
      for (let i = 1; i < states.length; i++) {
        const a = toScreen({ x: states[i - 1][3], y: states[i - 1][4] }), b = toScreen({ x: states[i][3], y: states[i][4] });
        ctx.strokeStyle = speedColor(Math.abs(states[i][6]) / top);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
      ctx.restore();
    } else {
      for (const section of layout.sections) {
        polyline(ctx, sectionSamples(section), selected ? "#2563eb" : "#6b7484", selected ? 3 : 2, section.reverse ? [7, 5] : []);
      }
    }
    layout.sections.forEach((section) => {
      if (section.reverse) reverseMarker(ctx, section);
    });
    if (!selected) {
      step.points.forEach((point) => drawKnob(ctx, point, "#6b7484", false, 4));
      return;
    }
    layout.sections.forEach((section, i) => {
      const emphasis = state.point === null || state.point === i || state.point === i - 1;
      ctx.save();
      ctx.globalAlpha = emphasis ? 1 : 0.45;
      polyline(ctx, [section.start, section.startHandle], "#2563eb", 1);
      polyline(ctx, [section.end, section.endHandle], "#2563eb", 1);
      drawKnob(ctx, section.startHandle, "#2563eb", true, 4.5);
      drawKnob(ctx, section.endHandle, "#2563eb", true, 4.5);
      ctx.restore();
    });
    step.points.forEach((point, i) => {
      const chosen = state.point === i;
      const reverse = step.sections[i].reverse;
      drawKnob(ctx, point, reverse ? "#a16207" : "#2563eb", chosen, ANCHOR_RADIUS_PX);
      drawLabel(ctx, point, String(i + 1), chosen ? "#fff" : reverse ? "#a16207" : "#1d4ed8");
    });
    if (state.point !== null && step.points[state.point]) {
      const end = layout.headings[state.point + 1];
      drawRobot(ctx, { ...step.points[state.point], heading: end }, { stroke: "rgba(37,99,235,.55)", dash: [4, 3], width: 1.2 });
    }
  }

  function drawOtherStep(ctx, step, index, start, end, selected) {
    const preview = stepPreview(step, start, trackWidth());
    polyline(ctx, preview, selected ? "#2563eb" : "#7b8494", selected ? 2.5 : 1.8, [6, 5]);
    const color = selected ? "#2563eb" : "#7b8494";
    if (["point", "turn_to_point", "boomerang", "wall_reset"].includes(step.type)) {
      const p = toScreen(step);
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(p.x - 7, p.y); ctx.lineTo(p.x + 7, p.y);
      ctx.moveTo(p.x, p.y - 7); ctx.lineTo(p.x, p.y + 7);
      ctx.stroke();
      ctx.restore();
    }
    if (selected) {
      for (const knob of draggableKnobsInHitOrder()) {
        if (["target-heading", "step-heading"].includes(knob.kind)) {
          polyline(ctx, [knob.pivot, knob.at], "#2563eb", 1.2);
          drawKnob(ctx, knob.at, "#2563eb", true, 5.5);
        }
        if (knob.kind === "target") drawKnob(ctx, knob.at, "#2563eb", false, 7);
        if (knob.kind === "drive-end") drawKnob(ctx, knob.at, "#2563eb", true, 5.5);
      }
      drawRobot(ctx, end, { stroke: "rgba(37,99,235,.6)", dash: [4, 3], width: 1.2 });
    }
  }

  function drawRunOverlay(ctx) {
    if (state.compare && state.layers.compare) {
      const frames = trace(state.compare);
      polyline(ctx, frames.map((f) => ({ x: f.true_x, y: f.true_y })), "#7c3aed", 2, [2, 4]);
    }
    const frames = trace();
    if (!frames.length) return;
    const index = frameIndexAt(frames, state.time);
    const shown = frames.slice(0, index + 1);
    if (state.layers.odometry) {
      const odometry = shown.filter((f) => !f.coordinate_reset).map((f) => ({ x: f.estimated_x, y: f.estimated_y }));
      polyline(ctx, odometry, "#e07b00", 2, [6, 4]);
    }
    if (state.layers.truth) polyline(ctx, shown.map((f) => ({ x: f.true_x, y: f.true_y })), "#18202c", 2.2);
    const frame = frames[index];
    const size = Number.isFinite(frame.body_length_in) ? { body_length_in: frame.body_length_in, body_width_in: frame.body_width_in } : null;
    if (state.layers.odometry && !frame.coordinate_reset) drawRobot(ctx, { x: frame.estimated_x, y: frame.estimated_y, heading: frame.estimated_heading }, { stroke: "#e07b00", dash: [4, 3], size });
    if (state.layers.truth) drawRobot(ctx, { x: frame.true_x, y: frame.true_y, heading: frame.true_heading }, { fill: "rgba(255,255,255,.82)", stroke: "#18202c", width: 2, size });
    if (state.layers.truth && Number.isFinite(frame.arm_deg)) drawClaw(ctx, { x: frame.true_x, y: frame.true_y, heading: frame.true_heading }, frame.arm_deg, frame.claw_gap_in, size);
    if (Number.isFinite(frame.controller_carrot_x)) {
      const p = toScreen({ x: frame.controller_carrot_x, y: frame.controller_carrot_y });
      ctx.save();
      ctx.strokeStyle = "#c2362f";
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(p.x - 4, p.y - 4); ctx.lineTo(p.x + 4, p.y + 4);
      ctx.moveTo(p.x + 4, p.y - 4); ctx.lineTo(p.x - 4, p.y + 4);
      ctx.stroke();
      ctx.restore();
    }
  }

  function drawField() {
    if (state.mode !== "top" || !state.config) return;
    const { ctx, width, height } = resizeCanvas($("field"));
    state.size = { width, height };
    $("field").dataset.view = JSON.stringify(state.view);
    drawFieldBackground(ctx);
    drawPieces(ctx);
    const all = poses();
    state.project.steps.forEach((step, index) => {
      if (index === state.selected) return;
      if (step.type === "path") drawPath(ctx, step, index, all[index].start, false);
      else drawOtherStep(ctx, step, index, all[index].start, all[index].end, false);
    });
    const step = selectedStep();
    if (step.type === "path") drawPath(ctx, step, state.selected, all[state.selected].start, true);
    else drawOtherStep(ctx, step, state.selected, all[state.selected].start, all[state.selected].end, true);
    if (state.project.steps.length > 1) {
      const placed = [];
      state.project.steps.forEach((item, index) => {
        if (stepInfo(item).mechanism) return;
        if (item.type !== "path" || item.points.length) stepTag(ctx, all[index].end, index + 1, index === state.selected, placed);
      });
    }
    const start = state.project.start;
    drawRobot(ctx, start, { stroke: "#2563eb", dash: [5, 3], width: 1.6, fill: "rgba(37,99,235,.06)" });
    if (!state.shown) drawClaw(ctx, start, 0, state.project.preload && state.project.preload !== "none" ? fieldDrawing().piece_size.pin_tip_in : 0);
    const startTip = along(start.heading, HEADING_KNOB_IN);
    polyline(ctx, [start, { x: start.x + startTip.x, y: start.y + startTip.y }], "rgba(37,99,235,.6)", 1.2);
    drawKnob(ctx, { x: start.x + startTip.x, y: start.y + startTip.y }, "#2563eb", false, 5);
    const span = timeline();
    if (span.kind === "plan" && state.time > 0) {
      const at = planStateAt(span.plan.states, state.time);
      drawRobot(ctx, { x: at[3], y: at[4], heading: at[5] }, { fill: "rgba(255,255,255,.75)", stroke: "#2563eb", width: 2 });
    }
    drawRunOverlay(ctx);
    if (state.hover?.kind === "curve") {
      drawKnob(ctx, state.hover.at, "#2563eb", false, 6);
      drawLabel(ctx, state.hover.at, "+", "#2563eb");
    }
    renderScore();
    const plan = currentPlan();
    $("speed-legend").hidden = !plan || !pathSteps().length;
    if (plan) $("speed-legend-max").textContent = `${fmt(plan.settings.top_speed_ips, 0)} in/s`;
  }

  function graphSeries() {
    const kind = $("graph-kind").value;
    const span = timeline();
    const series = [];
    const marks = [];
    let caption = "";
    if (span.kind === "plan") {
      if (kind === "speed") {
        series.push({ color: "#2563eb", points: span.plan.states.map((s) => [s[1], Math.abs(s[6])]), label: "Planned speed" });
        let part = 0;
        for (const s of span.plan.states) if (s[0] !== part) { marks.push({ t: s[1], label: `zone ${s[0] + 1}` }); part = s[0]; }
        caption = `Planned speed of step ${state.selected + 1}, in/s. Lines mark where the speed limit or direction changes.`;
      } else caption = "Run a simulation to see this graph. Before a run, only the planned speed is known.";
      return { series, marks, caption, end: span.end };
    }
    if (span.kind === "none") return { series, marks, caption: "Run a simulation, or select a planned path, to see a graph.", end: 0 };
    const frames = trace();
    const rows = stepRows();
    for (const row of rows) if (Number.isFinite(row.start_time_s)) marks.push({ t: row.start_time_s, label: `${row.index + 1}`, failed: row.executed !== false && row.passed === false });
    if (kind === "speed") {
      const plans = state.shown.result?.plans ?? [];
      rows.forEach((row) => {
        const plan = plans[row.index];
        if (plan && Number.isFinite(row.start_time_s)) series.push({ color: "#2563eb", dash: [5, 3], points: plan.states.map((s) => [row.start_time_s + s[1], Math.abs(s[6])]), label: "Planned" });
      });
      series.push({ color: "#18202c", points: frames.map((f) => [f.t, Math.abs(f.forward_speed_ips)]), label: "Simulated" });
      if (state.compare) series.push({ color: "#7c3aed", dash: [2, 3], points: trace(state.compare).map((f) => [f.t, Math.abs(f.forward_speed_ips)]), label: "Compare" });
      caption = "Speed in in/s. Dashed blue: planned for path steps. Black: simulated robot. Purple: compared run.";
    } else if (kind === "voltage") {
      series.push({ color: "#2563eb", points: frames.map((f) => [f.t, f.left_volts]), label: "Left" });
      series.push({ color: "#e07b00", points: frames.map((f) => [f.t, f.right_volts]), label: "Right" });
      caption = "Motor voltage in V. Blue: left side. Orange: right side.";
    } else if (kind === "heading") {
      const truth = unwrapSeries(frames.map((f) => f.true_heading));
      const odometry = unwrapSeries(frames.map((f) => f.estimated_heading));
      const target = unwrapSeries(frames.map((f) => f.controller_target_heading_deg));
      series.push({ color: "#18202c", points: frames.map((f, i) => [f.t, truth[i]]), label: "Simulated" });
      series.push({ color: "#e07b00", dash: [5, 3], points: frames.map((f, i) => [f.t, odometry[i]]), label: "Odometry" });
      series.push({ color: "#2563eb", points: frames.map((f, i) => [f.t, target[i]]), label: "Target" });
      caption = "Heading in degrees. Black: simulated. Orange: odometry. Blue: the controller's target.";
    } else {
      series.push({ color: "#e07b00", points: frames.map((f) => [f.t, f.odometry_error_in]), label: "Odometry error" });
      caption = "Distance between odometry and the simulated robot, in inches.";
    }
    return { series, marks, caption, end: span.end };
  }

  function drawGraph() {
    const canvas = $("graph");
    const { ctx, width, height } = resizeCanvas(canvas, 150);
    const key = `${$("graph-kind").value}|${state.shown?.id}|${state.shown?.status}|${state.frames.length}|${state.compare?.id}|${state.planFor}|${state.selected}|${width}|${timeline().kind}`;
    if (!state.graphCache || state.graphCache.key !== key) state.graphCache = { key, ...graphSeries() };
    const { series, marks, caption, end } = state.graphCache;
    $("graph-caption").textContent = caption;
    ctx.clearRect(0, 0, width, height);
    const left = 38, right = 8, top = 10, bottom = 20;
    const plotW = width - left - right, plotH = height - top - bottom;
    const values = series.flatMap((s) => s.points.map((p) => p[1])).filter(Number.isFinite);
    if (!end || !values.length) {
      ctx.fillStyle = "#8a93a1";
      ctx.font = "12px system-ui, sans-serif";
      ctx.fillText("No data to show yet.", left, top + 20);
      return;
    }
    let low = Math.min(0, ...values), high = Math.max(...values);
    if (high - low < 1e-6) high = low + 1;
    const pad = (high - low) * 0.08;
    low -= low < 0 ? pad : 0;
    high += pad;
    const x = (t) => left + (t / end) * plotW;
    const y = (v) => top + plotH - ((v - low) / (high - low)) * plotH;
    ctx.strokeStyle = "#eef0f4";
    ctx.fillStyle = "#8a93a1";
    ctx.font = "10.5px system-ui, sans-serif";
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    for (let i = 0; i <= 4; i++) {
      const v = low + (high - low) * i / 4;
      ctx.beginPath(); ctx.moveTo(left, y(v)); ctx.lineTo(width - right, y(v)); ctx.stroke();
      ctx.fillText(v.toFixed(Math.abs(high - low) < 10 ? 1 : 0), left - 4, y(v) + 3);
    }
    ctx.textAlign = "center";
    const stepSeconds = end > 20 ? 5 : end > 8 ? 2 : end > 3 ? 1 : 0.5;
    for (let t = 0; t <= end + 1e-6; t += stepSeconds) ctx.fillText(`${trim(t)} s`, x(t), height - 5);
    for (const mark of marks) {
      ctx.strokeStyle = mark.failed ? "rgba(194,54,47,.55)" : "#d5d9e0";
      ctx.beginPath(); ctx.moveTo(x(mark.t), top); ctx.lineTo(x(mark.t), top + plotH); ctx.stroke();
      ctx.fillStyle = mark.failed ? "#c2362f" : "#8a93a1";
      ctx.fillText(mark.label, x(mark.t) + 8, top + 9);
    }
    for (const s of series) {
      ctx.save();
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.setLineDash(s.dash ?? []);
      ctx.beginPath();
      let pen = false;
      for (const [t, v] of s.points) {
        if (!Number.isFinite(v)) { pen = false; continue; }
        if (pen) ctx.lineTo(x(t), y(v)); else ctx.moveTo(x(t), y(v));
        pen = true;
      }
      ctx.stroke();
      ctx.restore();
    }
    ctx.strokeStyle = "#2563eb";
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(x(state.time), top); ctx.lineTo(x(state.time), top + plotH); ctx.stroke();
    state.graphCache.geometry = { left, plotW, end };
  }

  function setMode(mode) {
    state.mode = mode;
    $("view-top").setAttribute("aria-selected", String(mode === "top"));
    $("view-3d").setAttribute("aria-selected", String(mode === "3d"));
    $("field").hidden = mode !== "top";
    const frame = $("viewer3d");
    frame.hidden = mode !== "3d";
    if (mode === "3d" && !frame.getAttribute("src")) {
      frame.src = frame.dataset.src;
      setTimeout(() => {
        if (!state.viewer.ready) {
          state.viewer.error = "The 3D view did not start. It needs Three.js 0.160.0 in the vexsim checkout's node_modules and WebGL in this browser.";
          sync3d();
        }
      }, 10000);
    }
    renderHint();
    drawField();
    sync3d();
  }

  function sync3d() {
    const status = $("viewer3d-status");
    if (state.mode !== "3d") {
      status.hidden = true;
      return;
    }
    const frame = frameNow();
    let message = "";
    if (state.viewer.error) message = state.viewer.error;
    else if (!state.viewer.ready) message = "Loading vexsim's 3D view…";
    else if (!state.shown) message = "Run a simulation to see the robot in 3D. The 3D view shows recorded physics only.";
    status.textContent = message;
    status.hidden = !message;
    if (state.viewer.ready) {
      $("viewer3d").contentWindow.postMessage({ type: "mclib-vexsim-frame", frame: state.shown ? frame : null, playing: state.playing, live: !!state.activeRun }, location.origin);
    }
  }

  function fit3d() {
    const frames = trace();
    if (!state.viewer.ready || !frames.length) return;
    const xs = frames.map((f) => f.true_x), ys = frames.map((f) => f.true_y);
    $("viewer3d").contentWindow.postMessage({ type: "mclib-vexsim-fit", bounds: { min_x: Math.min(...xs), max_x: Math.max(...xs), min_y: Math.min(...ys), max_y: Math.max(...ys) } }, location.origin);
  }

  function snap(point, shift) {
    const grid = shift ? 6 : 0.5;
    return { x: round(point.x, grid), y: round(point.y, grid) };
  }

  function snapAngle(angle, shift) {
    return wrapDegrees(round(angle, shift ? 15 : 1));
  }

  function canvasPoint(event) {
    const rect = $("field").getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function hoverHint(hit) {
    if (!hit) return null;
    const names = {
      anchor: `Point ${hit.point + 1}: drag to move, double-click for automatic heading, right-click to delete.`,
      "handle-in": `Curve handle of ${hit.point === -1 ? "the start" : `point ${hit.point + 1}`}: drag to set the heading there and how wide the curve is.`,
      "handle-out": hit.point === -1 ? "Start handle: drag to set how far the path runs straight out of the robot." : `Curve handle of point ${hit.point + 1}: drag to set the heading there and how wide the curve is.`,
      target: "Target: drag to move it.",
      "target-heading": "Heading knob: drag to set the heading at the target.",
      "step-heading": "Heading knob: drag to set the heading.",
      "drive-end": "Drag along the robot's heading to set the distance.",
      "start-heading": "Start heading: drag to turn the starting robot.",
      start: "Start pose: drag to move where the robot starts.",
      "other-anchor": `Point ${hit.point + 1} of step ${hit.step + 1}: click to select that step.`,
      curve: `Click to insert a point into section ${hit.section + 1}.`,
    };
    return names[hit.kind];
  }

  function applyDrag(drag, position, shift) {
    const project = state.project;
    const step = project.steps[state.selected];
    const snapped = snap(position, shift);
    switch (drag.kind) {
      case "anchor":
        Object.assign(step.points[drag.point], snapped);
        break;
      case "handle-out":
      case "handle-in": {
        const anchor = drag.point === -1 ? poses()[state.selected].start : step.points[drag.point];
        const length = Math.max(0.5, round(Math.hypot(position.x - anchor.x, position.y - anchor.y), 0.5));
        if (drag.point === -1) {
          const facing = along(anchor.heading + (step.sections[0].reverse ? 180 : 0), 1);
          step.start_handle = Math.max(0.5, round((position.x - anchor.x) * facing.x + (position.y - anchor.y) * facing.y, 0.5));
          break;
        }
        const sectionIndex = drag.kind === "handle-out" ? drag.point + 1 : drag.point;
        const offset = step.sections[sectionIndex].reverse ? 180 : 0;
        const travel = drag.kind === "handle-out" ? bearing(anchor, position) : bearing(position, anchor);
        step.points[drag.point].heading = snapAngle(travel - offset, shift);
        if (drag.kind === "handle-out") step.points[drag.point].handle_out = length;
        else step.points[drag.point].handle_in = length;
        break;
      }
      case "target":
        Object.assign(step, snapped);
        break;
      case "target-heading":
        step.heading = snapAngle(bearing(step, position), shift);
        break;
      case "step-heading": {
        const pivot = step.type === "turn" ? poses()[state.selected].start : poses()[state.selected].end;
        step.heading = snapAngle(bearing(pivot, position), shift);
        break;
      }
      case "drive-end": {
        const start = poses()[state.selected].start;
        const facing = along(start.heading, 1);
        step.distance = round((position.x - start.x) * facing.x + (position.y - start.y) * facing.y, shift ? 6 : 0.5);
        break;
      }
      case "start":
        Object.assign(project.start, snapped);
        break;
      case "start-heading":
        project.start.heading = snapAngle(bearing(project.start, position), shift);
        break;
      default:
        break;
    }
  }

  function bindField() {
    const canvas = $("field");
    canvas.addEventListener("pointerdown", (event) => {
      if (event.button === 2) return;
      canvas.focus();
      canvas.setPointerCapture(event.pointerId);
      const at = canvasPoint(event);
      const hit = event.button === 0 ? hitTest(at.x, at.y) : null;
      const draggable = hit && !["other-anchor", "curve"].includes(hit.kind);
      state.drag = {
        kind: draggable ? hit.kind : "pan", hit, point: hit?.point, start: at, moved: false,
        before: beginDrag(), view: { ...state.view },
      };
      if (hit?.kind === "anchor") { state.point = hit.point; renderPane(); }
    });
    canvas.addEventListener("pointermove", (event) => {
      const at = canvasPoint(event);
      const field = toField(at.x, at.y);
      $("cursor-readout").textContent = `x ${fmt(field.x)} in · y ${fmt(field.y)} in`;
      const drag = state.drag;
      if (!drag) {
        const hit = hitTest(at.x, at.y);
        state.hover = hit ? { ...hit, hint: hoverHint(hit) } : null;
        canvas.className = hit ? (hit.kind === "curve" || hit.kind === "other-anchor" ? "pointer" : "grab") : "";
        renderHint();
        drawField();
        return;
      }
      if (!drag.moved && Math.hypot(at.x - drag.start.x, at.y - drag.start.y) < 4) return;
      drag.moved = true;
      if (drag.kind === "pan") {
        state.view.x = drag.view.x - (at.x - drag.start.x) / state.view.scale;
        state.view.y = drag.view.y + (at.y - drag.start.y) / state.view.scale;
        state.viewMovedByUser = true;
        canvas.className = "grabbing";
      } else {
        applyDrag(drag, field, event.shiftKey);
        canvas.className = "grabbing";
        renderSteps();
      }
      state.hover = null;
      renderHint();
      drawField();
    });
    const release = (event) => {
      const drag = state.drag;
      if (!drag) return;
      state.drag = null;
      canvas.className = "";
      if (drag.moved && drag.kind !== "pan") {
        finishDrag(drag.before);
        return;
      }
      if (drag.moved) return drawField();
      clickField(drag.hit, toField(drag.start.x, drag.start.y), event.shiftKey);
    };
    canvas.addEventListener("pointerup", release);
    canvas.addEventListener("pointercancel", () => { state.drag = null; drawField(); });
    canvas.addEventListener("pointerleave", () => { $("cursor-readout").textContent = ""; state.hover = null; renderHint(); });
    canvas.addEventListener("dblclick", (event) => {
      const at = canvasPoint(event);
      const hit = hitTest(at.x, at.y);
      if (hit?.kind === "anchor") resetPoint(hit.point);
    });
    canvas.addEventListener("contextmenu", (event) => {
      const at = canvasPoint(event);
      const hit = hitTest(at.x, at.y);
      if (hit?.kind === "anchor") {
        event.preventDefault();
        deletePoint(hit.point);
        log(`Deleted point ${hit.point + 1} of step ${state.selected + 1}.`);
      }
    });
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      const at = canvasPoint(event);
      const before = toField(at.x, at.y);
      state.view.scale = Math.max(0.8, Math.min(60, state.view.scale * Math.exp(-event.deltaY * 0.0015)));
      const after = toField(at.x, at.y);
      state.view.x += before.x - after.x;
      state.view.y += before.y - after.y;
      state.viewMovedByUser = true;
      drawField();
    }, { passive: false });
  }

  function clickField(hit, position, shift) {
    const step = selectedStep();
    if (hit?.kind === "other-anchor") return selectStep(hit.step, hit.point);
    if (hit?.kind === "curve") return insertPoint(hit.section, snap(hit.at, shift));
    if (hit?.kind === "anchor") return;
    if (hit) return;
    const snapped = snap(position, shift);
    if (step.type === "path") return addPoint(snapped);
    if (["point", "turn_to_point", "boomerang", "wall_reset"].includes(step.type)) {
      edit(() => Object.assign(step, snapped));
      return;
    }
    if (state.point !== null) {
      state.point = null;
      renderAll();
    }
  }

  function nudge(dx, dy) {
    const step = selectedStep();
    if (step.type !== "path" || state.point === null) return false;
    edit(() => {
      step.points[state.point].x += dx;
      step.points[state.point].y += dy;
    });
    return true;
  }

  function bindKeys() {
    document.addEventListener("keydown", (event) => {
      const focused = document.activeElement;
      const typing = ["INPUT", "SELECT", "TEXTAREA"].includes(focused?.tagName) && !["range", "checkbox"].includes(focused.type);
      const mod = event.ctrlKey || event.metaKey;
      if (mod && event.key === "Enter") { event.preventDefault(); runSimulation(); return; }
      if (typing) return;
      if (document.querySelector("dialog[open]")) return;
      if (mod && event.key.toLowerCase() === "z") { event.preventDefault(); event.shiftKey ? redo() : undo(); return; }
      if (mod && event.key.toLowerCase() === "y") { event.preventDefault(); redo(); return; }
      if (mod) return;
      const big = event.shiftKey ? 6 : 0.5;
      switch (event.key) {
        case " ":
          event.preventDefault();
          if (focused instanceof HTMLElement && focused !== document.body) focused.blur();
          play();
          break;
        case "Delete": case "Backspace":
          event.preventDefault();
          if (state.point !== null) deletePoint(state.point); else deleteStep(state.selected);
          break;
        case "Escape": state.point = null; renderAll(); break;
        case "f": case "F": fitView(); break;
        case "?": $("help-dialog").showModal(); break;
        case "[": if (state.selected > 0) selectStep(state.selected - 1); break;
        case "]": if (state.selected < state.project.steps.length - 1) selectStep(state.selected + 1); break;
        case ",": play(false); setTime(state.time - 0.1); break;
        case ".": play(false); setTime(state.time + 0.1); break;
        case "ArrowLeft": if (nudge(-big, 0)) event.preventDefault(); break;
        case "ArrowRight": if (nudge(big, 0)) event.preventDefault(); break;
        case "ArrowUp": if (nudge(0, big)) event.preventDefault(); break;
        case "ArrowDown": if (nudge(0, -big)) event.preventDefault(); break;
        default: break;
      }
    });
  }

  function download(name, text, type) {
    const link = element("a", { href: URL.createObjectURL(new Blob([text], { type })), download: name });
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  function fileName(extension) {
    const slug = state.project.name.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "") || "routine";
    return `${slug}.${extension}`;
  }

  function fromBuilderFile(spec) {
    const steps = spec.steps.map((step) => {
      if (step.type !== "follow") {
        const { b, ...rest } = step;
        return rest;
      }
      const via = "via_x" in step ? [{ x: step.via_x, y: step.via_y ?? 0 }] : step.waypoints ?? [];
      const points = [...via, { x: step.x, y: step.y }].map((p) => ({ x: p.x, y: p.y, heading: null, handle_in: null, handle_out: null }));
      return { type: "path", points, sections: points.map(() => ({ speed_percent: null, reverse: step.direction === -1 })), start_handle: null,
        timeout_ms: step.timeout_ms ?? null, volts: step.volts ?? 12 };
    });
    const environment = spec.environment || {};
    return {
      format: state.config.format, version: state.config.version, name: spec.name || "Imported routine",
      robot: { preset: spec.preset, tracking: spec.tracking_mode || "two", heading_source: environment.encoder_heading ? "encoders" : "imu" },
      conditions: { battery_percent: (environment.soc ?? 1) * 100, grip_percent: (environment.friction ?? 1) * 100, noise_seed: environment.seed ?? 1 },
      gains: spec.tuning, start: spec.start_pose, stop_on_failure: spec.stop_on_failure ?? true, steps,
    };
  }

  async function openText(text, source) {
    let raw;
    try {
      raw = JSON.parse(text);
    } catch (error) {
      return log(`${source} is not valid JSON: ${error.message}`, "error");
    }
    let note = "";
    if (raw && raw.start_pose && raw.preset && Array.isArray(raw.steps)) {
      raw = fromBuilderFile(raw);
      note = " It came from the old motion builder: follow steps became paths, and their curves can differ slightly from the old splines.";
    }
    const answer = await api("/api/validate", { method: "POST", body: raw });
    if (!answer.ok) return log(`Could not open ${source}: ${answer.data.error}`, "error");
    replaceProject(answer.data, `Opened ${source}.${note}`);
  }

  async function exportCpp() {
    const answer = await api("/api/export", { method: "POST", body: clone(state.project) });
    if (!answer.ok) {
      if (Number.isInteger(answer.data.step)) selectStep(answer.data.step);
      return log(`Could not export: ${answer.data.error}`, "error");
    }
    $("cpp-code").value = answer.data.code;
    $("cpp-note").textContent = answer.data.measured
      ? "Paste this into your robot project. The path helpers at the top bake, plan and follow each path the same way the simulator did. kS, kV and track width were measured on the simulated robot: measure your own robot (docs/motion.md) and replace them."
      : `kS, kV and track width are placeholders${answer.data.note ? ` (${answer.data.note})` : ""}. Measure them on your robot (docs/motion.md).`;
    $("cpp-dialog").showModal();
    log("Exported the routine as C++.");
  }

  function bindControls() {
    $("routine-name").addEventListener("change", () => {
      const name = $("routine-name").value.trim();
      if (!name) { $("routine-name").value = state.project.name; return; }
      edit((project) => { project.name = name; }, { replan: false });
    });
    $("new-project").addEventListener("click", () => {
      const project = clone(state.config.default_project);
      project.robot = clone(state.project.robot);
      project.start = clone(state.project.start);
      replaceProject(project, "Started a new routine. Click on the field to add the path's points.");
    });
    $("open-project").addEventListener("click", () => $("open-file").click());
    $("open-file").addEventListener("change", async () => {
      const file = $("open-file").files[0];
      $("open-file").value = "";
      if (file) openText(await file.text(), file.name);
    });
    $("save-project").addEventListener("click", () => {
      download(fileName("json"), `${JSON.stringify(state.project, null, 2)}\n`, "application/json");
      log(`Saved ${fileName("json")} to your downloads.`, "ok");
    });
    $("example-picker").addEventListener("change", async () => {
      const id = $("example-picker").value;
      $("example-picker").value = "";
      if (!id) return;
      const answer = await api(`/api/examples/${id}`);
      if (!answer.ok) return log(answer.data.error, "error");
      replaceProject(answer.data, `Opened the example "${answer.data.name}". Press Run simulation to try it.`);
    });
    $("mirror-picker").addEventListener("change", () => {
      const axis = $("mirror-picker").value;
      $("mirror-picker").value = "";
      if (!axis) return;
      edit((project) => Object.assign(project, mirrorProject(project, axis)));
      log(axis === "x" ? "Mirrored the routine left ↔ right: x and headings flipped, arcs bend the other way." : "Mirrored the routine front ↔ back: y and headings flipped, arcs bend the other way.", "ok");
    });
    $("undo").addEventListener("click", undo);
    $("redo").addEventListener("click", redo);
    $("export-cpp").addEventListener("click", exportCpp);
    $("run").addEventListener("click", runSimulation);
    $("cancel-run").addEventListener("click", cancelRun);
    $("start-card").addEventListener("click", () => { state.tab = "robot"; renderAll(); });
    $("fit").addEventListener("click", () => (state.mode === "3d" ? fit3d() : fitView()));
    $("view-top").addEventListener("click", () => setMode("top"));
    $("view-3d").addEventListener("click", () => setMode("3d"));
    $("play").addEventListener("click", () => play());
    $("timeline").addEventListener("input", () => {
      const chosen = Number($("timeline").value);
      play(false);
      setTime(chosen);
    });
    $("speed").addEventListener("change", () => { state.speed = Number($("speed").value); });
    $("run-picker").addEventListener("change", () => loadRun($("run-picker").value, "shown"));
    $("compare-picker").addEventListener("change", () => loadRun($("compare-picker").value, "compare"));
    $("graph-kind").addEventListener("change", () => { state.graphCache = null; drawGraph(); });
    $("graph").addEventListener("click", (event) => {
      const geometry = state.graphCache?.geometry;
      if (!geometry) return;
      const rect = $("graph").getBoundingClientRect();
      play(false);
      setTime(((event.clientX - rect.left - geometry.left) / geometry.plotW) * geometry.end);
    });
    for (const tab of ["step", "robot", "telemetry"]) $(`tab-${tab}`).addEventListener("click", () => { state.tab = tab; renderPane(); });
    for (const input of document.querySelectorAll("#layer-toggles input")) {
      input.addEventListener("change", () => { state.layers[input.dataset.layer] = input.checked; drawField(); });
    }
    for (const dialog of document.querySelectorAll("dialog")) {
      for (const button of dialog.querySelectorAll("[data-close]")) button.addEventListener("click", () => dialog.close());
    }
    $("cpp-copy").addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText($("cpp-code").value);
        log("Copied the C++ to the clipboard.", "ok");
      } catch {
        $("cpp-code").select();
        log("The browser blocked the clipboard. The code is selected: press Ctrl+C.", "error");
      }
    });
    $("cpp-download").addEventListener("click", () => download("autonomous.cpp", $("cpp-code").value, "text/x-c++src"));
    window.addEventListener("resize", () => {
      state.graphCache = null;
      if (state.viewMovedByUser) drawField(); else fitView();
      drawGraph();
    });
    window.addEventListener("message", (event) => {
      if (event.origin !== location.origin || event.source !== $("viewer3d").contentWindow) return;
      const type = event.data?.type;
      if (type === "mclib-vexsim-ready") {
        state.viewer.ready = true;
        state.viewer.error = "";
        sync3d();
        fit3d();
      } else if (type === "mclib-vexsim-applied") {
        $("viewer3d").dataset.applied = JSON.stringify({ t: event.data.t ?? null, robot: event.data.robot ?? null, available: event.data.available });
      } else if (type === "mclib-vexsim-error") {
        state.viewer.error = `The 3D view stopped: ${String(event.data.message || "unknown error")}`;
        sync3d();
      }
    });
    const menu = $("add-step-menu");
    menu.replaceChildren(...Object.entries(state.config.step_types).map(([type, info]) =>
      element("button", { onclick: () => addStep(type), "data-type": type }, `${info.label}`, element("span", { text: `${info.call} · ${info.description.split(". ")[0]}.` }))));
    $("shortcut-strip").replaceChildren(
      element("kbd", { text: "Click" }), " add point · ", element("kbd", { text: "Drag" }), " move · ",
      element("kbd", { text: "Del" }), " delete · ", element("kbd", { text: "Ctrl+Z" }), " undo · ",
      element("kbd", { text: "Ctrl+Enter" }), " run · ", element("kbd", { text: "Space" }), " play · ",
      element("kbd", { text: "F" }), " fit · ", element("kbd", { text: "?" }), " all shortcuts");
    $("shortcut-table").replaceChildren(...SHORTCUTS.map(([keys, action]) => element("tr", {}, element("td", {}, element("kbd", { text: keys })), element("td", { text: action }))));
  }

  async function restoreDraft() {
    let text = null;
    try {
      text = localStorage.getItem(DRAFT_KEY);
    } catch {
      text = null;
    }
    if (!text) return false;
    const answer = await api("/api/validate", { method: "POST", body: JSON.parse(text) }).catch(() => null);
    if (!answer?.ok) {
      log(`Your saved draft could not be opened (${answer?.data?.error ?? "unreadable"}). Starting a new routine.`, "error");
      return false;
    }
    state.project = answer.data;
    log("Restored the routine you were editing in this browser.");
    return true;
  }

  async function init() {
    const answer = await api("/api/config");
    state.config = answer.data;
    state.project = clone(state.config.default_project);
    $("example-picker").append(...state.config.examples.map((item) => element("option", { value: item.id, text: item.name, title: item.description })));
    bindControls();
    bindField();
    bindKeys();
    if (!(await restoreDraft())) log("New routine. Click on the field to add points, then press Run simulation.");
    keepSelectionValid();
    renderAll();
    fitView();
    await checkEngine();
    await refreshRuns();
    schedulePlan(0);
  }

  init().catch((error) => {
    document.body.prepend(element("div", { class: "card fail", text: `The planner page could not start: ${error.message}. Is server.py still running?` }));
  });
})();
