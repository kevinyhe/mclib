"use strict";

// A robot log: the CSV mclib's telemetry Logger writes on the robot, read
// back and set against a simulated run of the same routine. The log holds
// what the robot's odometry believed, so it is compared with the simulated
// odometry, the plan and each step's target. test_robot_log.py runs these.

function robotLogFields(line) {
  return line.split(",").map((field) => field.trim());
}

function parseRobotLog(text) {
  const lines = text.split(/\r?\n/);
  const headerAt = lines.findIndex((line) => line.startsWith("t_ms,"));
  if (headerAt < 0) {
    throw new Error("No header line starting with t_ms. Use a CSV from mclib's telemetry Logger: its first line names the columns.");
  }
  const names = robotLogFields(lines[headerAt]);
  const at = (name) => names.indexOf(name);
  if (at("x_in") < 0 || at("y_in") < 0) {
    throw new Error(`The log needs x_in and y_in columns: log the pose with addLength("x") and addLength("y"). Its columns are ${names.join(", ")}.`);
  }
  const columns = { x: at("x_in"), y: at("y_in"), heading: at("heading_deg"), step: at("step") };
  const rows = [];
  let dropped = 0;
  for (let index = headerAt + 1; index < lines.length; index += 1) {
    const line = lines[index].trim();
    if (!line) continue;
    if (line.startsWith("#")) {
      const match = /dropped=(\d+)/.exec(line);
      if (match) dropped += Number(match[1]);
      continue;
    }
    const fields = robotLogFields(line);
    if (fields.length !== names.length || fields.some((field) => field === "")) {
      throw new Error(`Line ${index + 1} has ${fields.length} values where the header names ${names.length}: ${line.slice(0, 80)}`);
    }
    const values = fields.map(Number);
    if (!Number.isFinite(values[0])) throw new Error(`Line ${index + 1} has no time: ${line.slice(0, 80)}`);
    if (rows.length && values[0] < rows.at(-1).t_ms) {
      throw new Error(`Line ${index + 1} goes back in time, from ${rows.at(-1).t_ms} to ${values[0]} ms. Keep one run per file.`);
    }
    rows.push({
      t_ms: values[0], x: values[columns.x], y: values[columns.y],
      heading: columns.heading >= 0 ? values[columns.heading] : null,
      step: columns.step >= 0 ? values[columns.step] : null,
    });
  }
  if (rows.length < 2) throw new Error("The log has fewer than two rows.");
  const start = rows[0].t_ms;
  const samples = rows.map((row) => ({ t: (row.t_ms - start) / 1000, x: row.x, y: row.y, heading: row.heading, step: row.step }));
  return {
    columns: names, hasHeading: columns.heading >= 0, hasStep: columns.step >= 0, dropped,
    samples, duration: samples.at(-1).t,
  };
}

function lastAtOrBefore(items, t) {
  let low = 0;
  let high = items.length - 1;
  if (!items.length || items[0].t > t) return -1;
  while (low < high) {
    const middle = Math.ceil((low + high) / 2);
    if (items[middle].t <= t) low = middle;
    else high = middle - 1;
  }
  return low;
}

function robotLogPoseAt(log, offsetS, t) {
  const index = lastAtOrBefore(log.samples, t - offsetS);
  return index < 0 ? null : log.samples[index];
}

function headingGap(a, b) {
  if (!Number.isFinite(a) || !Number.isFinite(b)) return null;
  return Math.abs(((a - b) % 360 + 540) % 360 - 180);
}

function placed(sample) {
  return Number.isFinite(sample.x) && Number.isFinite(sample.y);
}

// The simulated odometry at time t, between the two frames around it, so a
// log written between frames is not set against a frame up to 10 ms away.
function simulatedOdometryAt(frames, t) {
  const index = Math.max(0, lastAtOrBefore(frames, t));
  const before = frames[index];
  const after = frames[Math.min(frames.length - 1, index + 1)];
  const share = after.t > before.t ? Math.min(1, Math.max(0, (t - before.t) / (after.t - before.t))) : 0;
  return {
    x: before.estimated_x + (after.estimated_x - before.estimated_x) * share,
    y: before.estimated_y + (after.estimated_y - before.estimated_y) * share,
    reset: before.coordinate_reset || after.coordinate_reset,
  };
}

// When each step ran in the log, in routine time: from the log's step column
// (the planner's step numbers, from 1) when it has one, else the simulated
// step's own times.
function robotLogWindows(log, offsetS, rows) {
  return rows.filter((row) => row.executed).map((row) => {
    if (!log.hasStep) return { index: row.index, from: row.start_time_s, to: row.return_time_s };
    const own = log.samples.filter((sample) => sample.step === row.index + 1);
    if (!own.length) return { index: row.index, from: null, to: null };
    return { index: row.index, from: own[0].t + offsetS, to: own.at(-1).t + offsetS };
  });
}

// For each step that ran: how far the logged pose got from the simulated
// odometry (at the same time into the step), from the planned path, and from
// the step's target when the step ended.
function compareRobotLog(log, offsetS, result) {
  const frames = result.trace;
  return robotLogWindows(log, offsetS, result.steps).map((window) => {
    const row = result.steps.find((item) => item.index === window.index);
    const samples = window.from === null ? [] : log.samples.filter((sample) => placed(sample) &&
      sample.t + offsetS >= window.from - 1e-9 && sample.t + offsetS <= window.to + 1e-9);
    if (!samples.length) return { index: window.index, samples: 0 };
    let largest = 0;
    let largestAt = null;
    for (const sample of samples) {
      const simulatedT = Math.min(row.return_time_s, row.start_time_s + (sample.t + offsetS - window.from));
      const simulated = simulatedOdometryAt(frames, simulatedT);
      if (simulated.reset) continue;
      const gap = Math.hypot(sample.x - simulated.x, sample.y - simulated.y);
      if (gap > largest) {
        largest = gap;
        largestAt = sample.t + offsetS;
      }
    }
    const last = samples.at(-1);
    const [endX, endY, endHeading] = row.estimated_pose_at_exit;
    const target = row.target ?? {};
    const plan = result.plans?.[window.index];
    return {
      index: window.index, samples: samples.length, from: window.from, to: window.to,
      simulated_largest_in: largest, simulated_largest_at_s: largestAt,
      simulated_end_in: row.coordinate_reset ? null : Math.hypot(last.x - endX, last.y - endY),
      simulated_end_heading_deg: headingGap(last.heading, endHeading),
      target_in: Number.isFinite(target.x) ? Math.hypot(last.x - target.x, last.y - target.y) : null,
      target_heading_deg: headingGap(last.heading, target.heading),
      plan_largest_in: plan ? Math.max(...samples.map((sample) =>
        Math.min(...plan.states.map((state) => Math.hypot(sample.x - state[3], sample.y - state[4]))))) : null,
    };
  });
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { parseRobotLog, robotLogPoseAt, robotLogWindows, compareRobotLog, headingGap };
}
