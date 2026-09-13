// These checks cover geometric intent, not simulated motion or controller acceptance.
import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { readFileSync } from "node:fs";
import { runInNewContext } from "node:vm";

const { motionReferencePath, motionFrameAt, motionVisibleTrace, motionTimelineMax, motionRecording, motionAcceptance, motionStepStatus, motionTelemetry, motionUnwrapSeries, motionTraceBounds } = createRequire(import.meta.url)("./builder.js");
let checks = 0;
function near(actual, expected, label) {
  assert(Math.abs(actual - expected) < 1e-9, `${label}: expected ${expected}, got ${actual}`);
  checks += 1;
}
function scenario(steps, preset = "six_motor_450", start_pose = { x: 0, y: 0, heading: 0 }) {
  return { preset, start_pose, steps };
}

// Quarter-turn fixtures follow the stationary tread's physical location.
// The next drive must start at the translated centre, not the original origin.
for (const [heading, direction, x, y] of [
  [90, 1, 5.75, 5.75], [-90, 1, -5.75, 5.75],
  [90, -1, -5.75, -5.75], [-90, -1, 5.75, -5.75],
]) {
  const input = scenario([{ type: "swing", heading, direction }, { type: "drive", distance: 24 }]);
  const before = JSON.stringify(input);
  const [swing, drive] = motionReferencePath(input);
  near(swing.end.x, x, `Swing ${heading}/${direction} centre X`);
  near(swing.end.y, y, `Swing ${heading}/${direction} centre Y`);
  near(drive.start.x, x, "Following drive start X");
  near(drive.start.y, y, "Following drive start Y");
  near(drive.end.x, x + (heading === 90 ? 24 : -24), "Following drive endpoint X");
  near(drive.end.y, y, "Following drive endpoint Y");
  assert.equal(JSON.stringify(input), before, "Preview calculation must not mutate the routine");
  const pivotX = x;
  for (const point of swing.points) near(Math.hypot(point.x - pivotX, point.y), 5.75, "Centre stays half a track width from stationary tread");
}

const rotated = motionReferencePath(scenario(
  [{ type: "swing", heading: 180, direction: 1 }],
  "four_motor_200", { x: 10, y: -8, heading: 90 },
))[0];
near(rotated.end.x, 15.75, "Rotated swing endpoint X");
near(rotated.end.y, -13.75, "Rotated swing endpoint Y");

const wrapped = motionReferencePath(scenario(
  [{ type: "swing", heading: 90, direction: 1 }],
  "six_motor_450", { x: 0, y: 0, heading: 360 },
))[0];
near(wrapped.end.x, 5.75, "Unwrapped start heading X");
near(wrapped.end.y, 5.75, "Unwrapped start heading Y");

const zero = motionReferencePath(scenario([{ type: "swing", heading: 0, direction: -1 }]))[0];
near(zero.end.x, 0, "Zero angle swing X");
near(zero.end.y, 0, "Zero angle swing Y");

const speed = motionReferencePath(scenario([{ type: "swing", heading: 90, direction: 1 }], "speed_base"))[0];
near(speed.end.x, 5.5, "Speed preset half track width X");
near(speed.end.y, 5.5, "Speed preset half track width Y");
const metadata = motionReferencePath(scenario([{ type: "swing", heading: 90, direction: 1 }]), 14)[0];
near(metadata.end.x, 7, "Server preset metadata overrides fallback X");
near(metadata.end.y, 7, "Server preset metadata overrides fallback Y");

for (const type of ["arc", "reverse_arc"]) {
  for (const radius of [-24, 24]) {
    const arc = motionReferencePath(scenario([{ type, heading: -90, radius }]))[0];
    near(arc.end.x, radius, `${type} signed radius X`);
    near(arc.end.y, -radius, `${type} signed radius Y`);
  }
}
console.log(`PASS ${checks} geometric reference checks; no physics results are asserted.`);

let debuggerChecks = 0;
function equal(actual, expected, label) { assert.deepEqual(actual, expected, label); debuggerChecks += 1; }
const firstEnd = { t: 3.71, step: 0, phase: "hold", true_x: 15.5, true_y: 19.5, true_heading: -88,
  estimated_x: 15.4, estimated_y: 19.4, estimated_heading: -88.2, odometry_error_in: .14,
  left_volts: 0, right_volts: 0, left_mode: 3, right_mode: 3 };
const secondStart = { ...firstEnd, step: 1, phase: "motion", left_volts: 12, right_volts: -12, left_mode: 0, right_mode: 0 };
const recorded = [{ ...firstEnd, t: 0, phase: "motion" }, firstEnd, { ...secondStart, left_volts: 0, right_volts: 0 }, secondStart];
const beforeTrace = JSON.stringify(recorded);
equal(motionFrameAt(recorded, 3.71), secondStart, "Normal playback uses the latest state at a shared timestamp");
equal(motionFrameAt(recorded, 3.71, { index: 0, executed: true }), firstEnd, "Inspecting first result selects its exact stopped frame");
equal(motionFrameAt(recorded, 3.71, { index: 1, executed: false }), null, "Skipped inspection must not reuse a preceding frame");
equal(motionFrameAt(recorded, 3.71, { index: 5, executed: true }), null, "Missing step recording must not reuse a preceding frame");
equal(motionFrameAt([], 0), null, "Empty trace is unavailable");
equal(JSON.stringify(recorded), beforeTrace, "Playback selection does not mutate recorded data");
equal(motionTimelineMax(3.569999999999718), 3.57, "Negative floating drift cannot lower the final slider step");
equal(motionTimelineMax(3.570000000000282), 3.57, "Tiny positive floating drift does not invent an extra millisecond");
equal(motionTimelineMax(3.5705), 3.571, "A real sub-millisecond endpoint remains reachable on the UI grid");
equal(motionTimelineMax(0), .001, "An empty range retains a valid HTML maximum");
const floatFinal = { ...firstEnd, t: 3.570000000000282 };
const floatTrace = [{ ...firstEnd, t: 3.56 }, floatFinal];
equal(motionFrameAt(floatTrace, 3.57), floatFinal, "Nanosecond comparison tolerance reaches the exact raw final frame");
equal(motionFrameAt(floatTrace, 3.569), floatTrace[0], "Timing tolerance cannot admit a genuinely later frame");
equal(motionFrameAt(recorded, 3.71, { index: 0, executed: true }), firstEnd, "Grid correction does not change exact result-step ownership");
equal(motionVisibleTrace(floatTrace, 3.57).at(-1), floatFinal, "Plots include exactly the same last frame as native playback");
equal(motionVisibleTrace(recorded, 3.71, { index: 0, executed: true }), recorded.slice(0, 2), "Inspected plot excludes the next action's same-timestamp command");
equal(motionVisibleTrace(recorded, 3.71, { index: 1, executed: false }), [], "Skipped steps have no fabricated plot");
const stale = { trace: recorded, steps: [{ index: 0 }] };
equal(motionRecording({ status: "running", result: stale }).trace, [], "New live run cannot replay a stale completed trace");
const liveFrame = { ...secondStart, t: 4 };
const active = { index: 1, passed: null, status: "running", target: { x: 24, y: 25, heading: null } };
const live = motionRecording({ status: "running", result: stale, partial: { steps: [active], trace: [liveFrame], active_step: 1 } });
equal(live.trace, [liveFrame], "Live partial frame wins over stale result");
equal(live.steps[0].target, active.target, "Live targets come from the resolved backend step");
equal(live.live, true, "Running snapshots disable replay");
equal(motionRecording({ status: "passed", result: stale, partial: { trace: [liveFrame] } }), stale, "Final recorded result replaces partial snapshot");
equal(motionRecording({ status: "cancelled" }).trace, [], "Cancellation without a recording clears stale data");
for (const status of ["cancelled", "error"]) {
  const partial = { live: true, trace: [liveFrame], steps: [{ index: 0, passed: true, executed: true }, active] };
  const before = JSON.stringify(partial);
  const incomplete = motionRecording({ status, partial });
  equal(incomplete.trace, [liveFrame], `${status}: preserve this job's last captured frame`);
  equal(incomplete.live, false, `${status}: retained data is a recording, not live`);
  equal(incomplete.incomplete, true, `${status}: mark retained data incomplete`);
  equal(incomplete.passed, null, `${status}: no fabricated completed acceptance`);
  equal(motionAcceptance(incomplete), "INCOMPLETE", `${status}: overall acceptance cannot pass`);
  equal(motionStepStatus(active, incomplete), "INTERRUPTED", `${status}: active step is interrupted, not still running or skipped`);
  equal(motionStepStatus(incomplete.steps[0], incomplete), "PASS", `${status}: preserve genuine checks of previously completed steps`);
  equal(motionFrameAt(incomplete.trace, liveFrame.t, active), liveFrame, `${status}: interrupted step can inspect its last real controller frame`);
  equal(JSON.stringify(partial), before, `${status}: do not mutate the live snapshot when retaining it`);
  const empty = motionRecording({ status });
  equal(empty.trace, [], `${status}: without its own partial, there is no recording`);
  equal(motionAcceptance(empty), "INCOMPLETE", `${status}: absence of captured data cannot pass`);
}
const target = { x: 15.5, y: 18.5, heading: -90 };
const legacy = motionTelemetry(firstEnd, target);
equal(legacy["target-error"], "1.00 in", "Position error uses the inspected step's target");
equal(legacy["final-heading-error"], "2.00°", "Absolute heading error is separate from position");
equal(legacy["command-mode"], "hold / hold", "Recorded brake modes are explicit");
for (const key of ["controller-phase", "controller-carrot", "controller-limits", "controller-heading", "body-speed", "imu-reading", "motor-current", "encoder-angle", "tracker-distance"]) {
  equal(legacy[key], "—", `${key}: missing legacy data must not be fabricated`);
}
equal(Object.values(motionTelemetry(null, target)).every(value => value === "—"), true, "Skipped/missing frame clears every telemetry value");
const rich = motionTelemetry({ ...firstEnd, forward_speed_ips: 0, lateral_speed_ips: -2, imu_heading_deg: 0, imu_rate_dps: 0,
  left_current_amps: 2.5, right_current_amps: 3, controller_phase: "pivot", controller_target_heading_deg: -90,
  controller_heading_error_deg: -2, controller_carrot_x: 18.5, controller_carrot_y: 19.5,
  controller_slew_limited: false, controller_voltage_limited: true, controller_drive_volts: 0, controller_yaw_volts: -8,
  parallel_tracker_in: 2.5, perpendicular_tracker_in: -1.5 }, target);
equal(rich["body-speed"], "0.0 / -2.0 in/s", "Zero and signed velocity remain meaningful");
equal(rich["imu-reading"], "0.0° / 0.0 °/s", "Zero-valued gyro data is available");
equal(rich["controller-limits"], "no / yes", "False limitation flag is not missing");
equal(rich["controller-carrot"], "18.50 / 19.50 in", "Carrot is the recorded controller value");
equal(rich["controller-demand"], "0.00 / -8.00 V", "Drive/yaw requests remain separate from wheel commands");
equal(rich["tracker-distance"], "2.50 / -1.50 in", "Tracker signs and raw distances preserved");
equal(motionTelemetry({ ...firstEnd, coordinate_reset: true }, target)["odometry-gap"], "FRAME RESET", "Coordinate resets are not physical odometry gaps");
equal(motionUnwrapSeries([{ h: 179 }, { h: -179 }, { h: -175 }], "h"), [179, 181, 185], "Wrapped gyro does not produce a 360-degree plot spike");
equal(motionUnwrapSeries([{ h: -175 }, { h: 179 }, { h: 175 }], "h"), [-175, -181, -185], "Negative crossing unwraps symmetrically");
equal(motionUnwrapSeries([{ h: -207 }, { h: -90 }], "h"), [-207, -90], "Final-alignment target switch remains a real 117-degree discontinuity");
equal(motionUnwrapSeries([{ h: 179 }, { h: null }, { h: -179 }], "h"), [179, null, 181], "Unavailable samples stay unavailable while angle continuity is preserved");
equal(motionTraceBounds([{ true_x: -2, true_y: 4 }, { true_x: 5, true_y: -1 }, { true_x: null, true_y: 20 }]), { min_x: -2, max_x: 5, min_y: -1, max_y: 4 }, "Native Fit run bounds use only finite recorded positions");
equal(motionTraceBounds([]), null, "An empty run has no invented camera bounds");
console.log(`PASS ${debuggerChecks} debugger data/selection checks; synthetic UI contracts, not physics acceptance.`);

// Exercise the actual asynchronous handlers, with deferred transport and rendering
// stubs only. This validates request ownership, not browser layout or physics.
const source = readFileSync(new URL("./builder.js", import.meta.url), "utf8");
function section(start, end) { return source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start))); }
const asyncSource = section("  function updateHistoryControls()", "  function updateJobStatus(job)")
  + section("  async function runSequence()", "  async function pollRun(")
  + section('    $("run-history").addEventListener("change"', "    bindCanvas();");
function asyncFixture() {
  const elements = new Map(), pending = new Map(), calls = [];
  const state = { historyRequest: 0, historyLoading: false, historyChoice: "old", compareRequest: 0,
    compareLoading: false, compareChoice: "", submitting: false, activeId: null, live: false,
    connected: true, spec: { name: "draft" }, job: { id: "old", spec: { name: "old" }, result: {} }, compare: null };
  const $ = id => {
    if (!elements.has(id)) elements.set(id, { value: id === "run-history" ? "old" : "", disabled: false, hidden: false,
      addEventListener(type, callback) { this[type] = callback; } });
    return elements.get(id);
  };
  const context = { state, $, notice() {}, play() {}, renderResults() {}, updateFieldMode() {}, fitView() {},
    refreshHistory: async () => {}, pollRun: async () => {},
    api(path, options = {}) {
      const key = `${options.method || "GET"} ${path}`; calls.push(key);
      return new Promise((resolve, reject) => pending.set(key, { resolve, reject }));
    },
    loadJob: async job => {
      context.invalidateRecordRequests(); state.job = job; state.live = ["queued", "running"].includes(job.status);
      $("run-history").value = job.id; context.updateHistoryControls();
    },
  };
  runInNewContext(asyncSource, context);
  const change = (id, value) => { $(id).value = value; return $(id).change({ target: $(id) }); };
  return { context, state, $, pending, calls, change };
}
let requestChecks = 0;
function requestEqual(actual, expected, label) { assert.deepEqual(actual, expected, label); requestChecks += 1; }
{
  const f = asyncFixture();
  const comparison = f.change("compare-history", "comparison-a");
  await f.change("compare-history", "");
  f.pending.get("GET /api/runs/comparison-a").resolve({ id: "comparison-a" }); await comparison;
  requestEqual(f.state.compare, null, "Late comparison response cannot undo clear");
  requestEqual(f.$("compare-history").value, "", "Cleared dropdown agrees with comparison state");
}
{
  const f = asyncFixture();
  const history = f.change("run-history", "historical-b");
  const submission = f.context.runSequence();
  for (const id of ["run-history", "compare-history", "load-recorded", "run-sequence"]) requestEqual(f.$(id).disabled, true, `${id} disabled synchronously before POST resolves`);
  await f.change("run-history", "historical-c"); await f.change("compare-history", "comparison-a");
  await f.context.runSequence();
  requestEqual(f.calls, ["GET /api/runs/historical-b", "POST /api/runs"], "Dispatched events and duplicate submission cannot issue requests during POST");
  f.pending.get("POST /api/runs").resolve({ id: "live-c", status: "running", spec: { name: "live" } }); await submission;
  f.pending.get("GET /api/runs/historical-b").resolve({ id: "historical-b", status: "passed", spec: { name: "history" } }); await history;
  requestEqual(f.state.job.id, "live-c", "History started before POST cannot replace adopted live job");
  requestEqual(f.state.activeId, "live-c", "Live polling still owns the displayed job");
  requestEqual(f.state.live, true, "Late history cannot turn live telemetry into replay");
  requestEqual(f.$("run-history").value, "live-c", "Displayed dropdown retains live job ownership");
  requestEqual(f.state.spec.name, "draft", "Async requests never overwrite the editor");
  requestEqual(f.state.submitting, false, "Successful POST exits submission state");
}
{
  const f = asyncFixture();
  const submission = f.context.runSequence();
  f.pending.get("POST /api/runs").reject(new Error("Submission failed")); await submission;
  for (const id of ["run-history", "compare-history", "load-recorded", "run-sequence"]) requestEqual(f.$(id).disabled, false, `${id} restored after failed POST`);
  requestEqual(f.state.submitting, false, "Failed POST exits submission state");
  requestEqual(f.state.activeId, null, "Failed POST invents no active job");
  requestEqual(f.state.job.id, "old", "Failed POST preserves the previous recording");
  requestEqual(f.$("run-history").value, "old", "Failed POST preserves the previous selection");
}
console.log(`PASS ${requestChecks} actual-handler asynchronous ownership checks; stub transport, no simulation results asserted.`);
