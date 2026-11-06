// Drives the real planner page in headless Chromium against a running
// server.py: place points, plan, run, play back, compare, export, mirror,
// run the Override example with its lift, claw, Toggle roller, score and 3D field, and
// run the Match Load example past 15 s.
// Installs nothing; pass an existing Playwright and browser.
import assert from "node:assert/strict";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";

const usage = "Usage: node browser_smoke.mjs --playwright /path/to/playwright/index.mjs --browser /path/to/chrome [--url http://127.0.0.1:8765] [--output folder-for-screenshots]";
const options = {};
for (let index = 2; index < process.argv.length; index += 2) {
  const key = process.argv[index];
  if (!["--playwright", "--browser", "--url", "--output"].includes(key) || !process.argv[index + 1]) throw new Error(usage);
  options[key.slice(2)] = process.argv[index + 1];
}
if (!options.playwright || !options.browser) throw new Error(usage);
const url = new URL(options.url ?? "http://127.0.0.1:8765");
assert(["127.0.0.1", "localhost"].includes(url.hostname), "Only a local planner is tested");
const output = resolve(options.output ?? "bin/path-planner-screenshots");
await mkdir(output, { recursive: true });

const { chromium } = await import(pathToFileURL(resolve(options.playwright)).href);
const browser = await chromium.launch({ executablePath: resolve(options.browser), headless: true, args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const pageErrors = [];
const requests = [];
let checks = 0;
let expectRefusal = false;
const pass = (label) => { checks += 1; console.log(`PASS ${label}`); };

try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  page.on("pageerror", (error) => pageErrors.push(error.message));
  page.on("console", (message) => {
    const expectedRefusal = /status of 400/.test(message.text()) && expectRefusal;
    if (message.type() === "error" && !expectedRefusal) pageErrors.push(message.text());
  });
  page.on("request", (request) => requests.push(request.url()));
  const shot = (name) => page.screenshot({ path: `${output}/${name}.png` });
  const draft = () => page.evaluate(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")));

  async function fieldPoint(x, y) {
    const box = await page.locator("#field").boundingBox();
    const view = JSON.parse(await page.locator("#field").getAttribute("data-view"));
    return { x: box.x + box.width / 2 + (x - view.x) * view.scale, y: box.y + box.height / 2 - (y - view.y) * view.scale };
  }
  async function clickField(x, y) {
    const at = await fieldPoint(x, y);
    await page.mouse.click(at.x, at.y);
  }
  async function dragField(from, to) {
    const a = await fieldPoint(from.x, from.y);
    const b = await fieldPoint(to.x, to.y);
    await page.mouse.move(a.x, a.y);
    await page.mouse.down();
    await page.mouse.move((a.x + b.x) / 2, (a.y + b.y) / 2, { steps: 4 });
    await page.mouse.move(b.x, b.y, { steps: 4 });
    await page.mouse.up();
  }
  const near = (a, b, tolerance = 1) => Math.abs(a - b) <= tolerance;

  await page.goto(url.href);
  await page.waitForFunction(() => document.querySelector("#step-list li"));
  assert.match(await page.locator("#log").textContent(), /Click on the field/);
  assert.match(await page.locator("#field-hint").innerText(), /Click on the field to add points/);
  assert.match(await page.locator("#shortcut-strip").innerText(), /Ctrl\+Z/);
  pass("A new routine opens with one empty path and tells the user to click the field");

  await page.waitForFunction(() => /Ready/.test(document.querySelector("#engine-pill").textContent), null, { timeout: 600000 });
  pass(`Planner ready: ${await page.locator("#engine-pill").innerText()}`);
  await shot("01-empty");

  await page.locator("#tab-robot").click();
  const fieldSelect = page.locator("#pane-robot label.field-label", { hasText: /^Field/ }).locator("select");
  assert.equal(await fieldSelect.inputValue(), "override");
  await fieldSelect.selectOption("open");
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).field === "open");
  assert.doesNotMatch(await page.locator("#pane-robot").textContent(), /failed step/);
  await page.locator("#tab-step").click();
  pass("A new routine starts on the Override field; these first checks switch it to the open floor");

  await clickField(24, -12);
  await clickField(0, 24);
  await clickField(-24, 48);
  let project = await draft();
  const points = project.steps[0].points;
  assert.equal(points.length, 3);
  assert(near(points[0].x, 24) && near(points[0].y, -12) && near(points[2].x, -24) && near(points[2].y, 48), JSON.stringify(points));
  pass("Three clicks place three points where they were clicked");

  await page.waitForFunction(() => /top \d/.test(document.querySelector("#step-list li").textContent), null, { timeout: 30000 });
  assert.equal(await page.locator("#speed-legend").isVisible(), true);
  const summary = await page.locator("#step-list li .step-sub").first().innerText();
  assert.match(summary, /3 points · \d+\.\d+ s · top \d+\.\d in\/s/);
  assert.match(await page.locator("#pane-step").innerText(), /Plan from mclib[\s\S]*Planned time[\s\S]*Top speed/i);
  pass(`mclib planned the path: ${summary}`);
  await page.locator("#graph-kind").selectOption("speed");
  assert.match(await page.locator("#graph-caption").innerText(), /Planned speed/);
  pass("The graph shows the planned speed before any run");
  await shot("02-planned");

  const probeAt = async (x, y) => {
    const at = await fieldPoint(x, y);
    await page.mouse.move(at.x, at.y);
    return (await page.locator("#path-probe").isVisible()) ? page.locator("#path-probe").innerText() : null;
  };
  const probeReading = (text) => {
    const [, along, length] = text.match(/(\d+\.\d) in of (\d+\.\d) in along the path/);
    const [, into, total] = text.match(/(\d+\.\d\d) s of (\d+\.\d\d) s into the path/);
    return { along: Number(along), length: Number(length), into: Number(into), total: Number(total) };
  };
  const nearStart = await probeAt(24, -12);
  const nearEnd = await probeAt(0, 24);
  await shot("02b-path-probe");
  assert.match(nearStart, /^Step 1 · /);
  assert.match(nearStart, /Planned speed \d+\.\d in\/s/);
  const early = probeReading(nearStart), late = probeReading(nearEnd);
  assert(0 < early.along && early.along < late.along && late.along < late.length, JSON.stringify({ early, late }));
  assert(0 < early.into && early.into < late.into && late.into < late.total, JSON.stringify({ early, late }));
  assert.equal(late.total.toFixed(2), summary.match(/(\d+\.\d+) s/)[1]);
  assert.equal(await probeAt(60, -60), null);
  pass(`Hovering over the path shows how far and how long along it: ${nearStart.split("\n").slice(1, 3).join(" · ")}`);

  await dragField({ x: 0, y: 24 }, { x: 6, y: 30 });
  project = await draft();
  assert(near(project.steps[0].points[1].x, 6) && near(project.steps[0].points[1].y, 30), JSON.stringify(project.steps[0].points[1]));
  await page.keyboard.press("Control+z");
  project = await draft();
  assert(near(project.steps[0].points[1].x, 0) && near(project.steps[0].points[1].y, 24));
  await page.keyboard.press("Control+Shift+z");
  project = await draft();
  assert(near(project.steps[0].points[1].x, 6));
  await page.keyboard.press("Control+z");
  pass("Dragging a point moves it; Ctrl+Z and Ctrl+Shift+Z undo and redo the drag");

  await clickField(0, 24);
  await page.locator("#pane-step .point.selected").waitFor();
  const sectionSpeed = page.locator("#pane-step .point.selected .section-box");
  await sectionSpeed.locator("label.check").first().click();
  const speedInput = sectionSpeed.locator("input[type=number]");
  await speedInput.fill("40");
  await speedInput.press("Enter");
  await page.waitForFunction(() => {
    const value = JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1"));
    return value.steps[0].sections[1].speed_percent === 40;
  });
  await page.waitForFunction(() => /Speed zones\s*3/.test(document.querySelector("#pane-step").innerText), null, { timeout: 30000 });
  pass("A 40% speed limit on section 2 splits the plan into 3 speed zones");
  await shot("03-speed-zone");

  await page.keyboard.press("Escape");
  await page.locator("#add-step summary").click();
  await page.locator('#add-step-menu button[data-type="turn"]').click();
  assert.equal(await page.locator("#step-list li").count(), 2);
  assert.match(await page.locator("#pane-step").innerText(), /Step 2 · Turn/);
  pass("Add a step puts a turn after the path");

  const keepMoving = (scope) => page.locator(`${scope} label.check`, { hasText: "Keep moving into the next step" });
  assert(await keepMoving("#pane-step").locator("input").isDisabled());
  assert.match(await keepMoving("#pane-step").innerText(), /The last step has to stop/);
  await page.locator("#step-list li").first().click();
  await keepMoving("#pane-step").locator("input").check();
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).steps[0].chain === true);
  assert.match(await page.locator("#step-list li").first().innerText(), /Keeps moving into step 2/);
  assert.match(await page.locator("#pane-step").textContent(), /planned time plus 0\.5 s/);
  await shot("03b-keep-moving");
  await page.keyboard.press("Control+z");
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).steps[0].chain === false);
  pass("Keep moving is refused on the last step with the reason, ticks on the path, and Ctrl+Z unticks it");
  await page.locator("#step-list li").nth(1).click();

  const finished = () => /Run \d+: (ran all \d+ steps|stopped after step \d+)/.test(document.querySelector("#run-summary").textContent);
  await page.locator("#run").click();
  await page.waitForFunction(() => /Run \d+ is/.test(document.querySelector("#run-summary").textContent));
  await page.waitForFunction(finished, null, { timeout: 300000 });
  const firstRun = await page.locator("#run-picker").inputValue();
  const stages = await page.locator("#stage-list").textContent();
  for (const label of ["Check the routine", "Compile mclib", "Measure the robot", "Plan path speeds", "Simulate in vexsim", "Report each step"]) assert.match(stages, new RegExp(label));
  assert.match(await page.locator("#run-summary").innerText(), /ran all 2 steps/);
  assert.equal(await page.locator("#step-list .badge.fail").count(), 0);
  assert.match(await page.locator("#step-list .badge.done").first().innerText(), /^\d+\.\d in off$/);
  assert.doesNotMatch(await page.locator("#step-list").innerText(), /[✓✗]/);
  pass(`Run simulation ran both steps in vexsim: ${(await page.locator("#run-summary").innerText()).replace(/\n/g, " ")}`);
  await shot("04-run-done");

  await page.locator("#step-list li").first().click();
  const result = await page.locator("#pane-step .card.result").textContent();
  assert.match(result, /Ended \d+\.\d+ in from the target\./);
  assert.match(result, /Largest gap from the planned path/);
  assert.doesNotMatch(result, /limit 2\.5 in|[✓✗]|Passed|Failed/);
  pass("The step pane shows what the step did, with no pass or fail");

  await page.locator("#step-list li").nth(1).click();
  const minVolts = page.locator("#pane-step .auto-row", { hasText: "Min voltage" });
  assert.equal(await minVolts.locator("input[type=checkbox]").isChecked(), true);
  await minVolts.locator("input[type=checkbox]").uncheck();
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).steps[1].min_volts === 1.5);
  await page.keyboard.press("Control+z");
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).steps[1].min_volts === null);
  pass("A turn has Min voltage on Automatic; unticking it sets mclib's 1.5 V and Ctrl+Z puts Automatic back");

  await page.locator("#timeline").evaluate((input) => { input.value = String(Number(input.max) / 2); input.dispatchEvent(new Event("input")); });
  const clock = await page.locator("#clock").innerText();
  assert.match(clock, /^\d+\.\d\d \/ \d+\.\d\d s$/);
  assert.notEqual(clock.split(" ")[0], "0.00");
  await page.locator("#tab-telemetry").click();
  const telemetry = await page.locator("#pane-telemetry").innerText();
  assert.match(telemetry, /Position/i);
  assert.match(telemetry, /Motors and battery/i);
  pass(`Scrubbing to ${clock} shows telemetry at that time`);
  await page.locator("#graph-kind").selectOption("voltage");
  await shot("05-playback-telemetry");

  await page.locator("#play").click();
  await page.waitForTimeout(400);
  const playing = await page.locator("#clock").innerText();
  assert.notEqual(playing, clock);
  await page.keyboard.press("Space");
  pass("Play advances the playback clock; Space pauses");

  await page.locator("#tab-step").click();
  await page.locator("#step-list li").first().click();
  await clickField(-12, 60);
  await page.keyboard.press("Control+Enter");
  await page.waitForFunction((previous) => document.querySelector("#run-picker").value !== previous, firstRun);
  await page.waitForFunction(finished, null, { timeout: 300000 });
  const secondSummary = (await page.locator("#run-summary").innerText()).replace(/\n/g, " ");
  await page.locator("#graph-kind").selectOption("speed");
  await page.locator("#compare-picker").selectOption(firstRun);
  await page.waitForFunction(() => /Purple: compared run/.test(document.querySelector("#graph-caption").textContent));
  pass(`A second run with an extra point (${secondSummary}) can be compared with the first`);
  await shot("06-compare");

  await page.locator("#export-cpp").click();
  await page.locator("#cpp-dialog[open]").waitFor();
  const code = await page.locator("#cpp-code").inputValue();
  assert.match(code, /void autonomous\(\)/);
  assert.match(code, /path_planner::planSections/);
  assert.match(code, /path_planner::followSections/);
  assert.match(code, /turnToAngle|turnTo/);
  assert.match(await page.locator("#cpp-note").innerText(), /measured on the simulated robot/);
  pass("Export C++ shows autonomous() with the same path helpers the simulator ran");
  await shot("07-export");
  await page.locator("#cpp-dialog .dialog-foot button.primary").click();

  const unmirrored = await draft();
  await page.locator("#mirror-picker").selectOption("x");
  project = await draft();
  assert.equal(project.steps[0].points[0].x, -unmirrored.steps[0].points[0].x);
  assert.equal(project.steps[1].heading, -unmirrored.steps[1].heading);
  await page.keyboard.press("Control+z");
  assert.deepEqual(await draft(), unmirrored);
  pass("Mirror left ↔ right negates x; Ctrl+Z puts it back");

  const builderFile = `${output}/old-builder-routine.json`;
  await writeFile(builderFile, JSON.stringify({
    name: "Old builder S-curve", preset: "six_motor_450", tracking_mode: "two", start_pose: { x: 0, y: 0, heading: 0 },
    environment: { soc: 1, friction: 1, seed: 1 }, stop_on_failure: true,
    steps: [{ type: "follow", via_x: 12, via_y: 24, x: 0, y: 48, direction: 1, b: 50, timeout_ms: 8000, volts: 12 },
      { type: "turn", heading: 90, timeout_ms: 4000, volts: 12 }],
  }));
  await page.locator("#open-file").setInputFiles(builderFile);
  await page.waitForFunction(() => /came from the old motion builder/.test(document.querySelector("#log").textContent));
  project = await draft();
  assert.equal(project.steps[0].type, "path");
  assert.deepEqual(project.steps[0].points.map((point) => [point.x, point.y]), [[12, 24], [0, 48]]);
  pass("Opening an old motion builder file turns its follow step into a path");

  await page.locator("#example-picker").selectOption("mixed_steps");
  await page.waitForFunction(() => /Opened the example/.test(document.querySelector("#log").textContent));
  await page.locator("#run").click();
  await page.locator("#cancel-run").waitFor({ state: "visible" });
  await page.waitForTimeout(500);
  await page.locator("#cancel-run").click();
  await page.waitForFunction(() => /was cancelled/.test(document.querySelector("#run-summary").textContent), null, { timeout: 30000 });
  assert.equal(await page.locator("#run").isVisible(), true);
  pass("Cancel stops a run that is going");

  await page.locator("#new-project").click();
  expectRefusal = true;
  await page.locator("#run").click();
  await page.waitForFunction(() => /has no points yet/.test(document.querySelector("#log").textContent));
  expectRefusal = false;
  pass("Running an empty path names the step and the fix");

  await page.keyboard.press("?");
  await page.locator("#help-dialog[open]").waitFor();
  assert((await page.locator("#shortcut-table tr").count()) >= 12);
  await shot("08-help");
  await page.locator("#help-dialog .dialog-foot button.primary").click();
  pass("? opens the help with the full shortcut list");

  await page.locator("#run-picker").selectOption({ index: 1 });
  await page.locator("#view-3d").click();
  await page.waitForFunction(() => {
    const status = document.querySelector("#viewer3d-status");
    return document.querySelector("#viewer3d").dataset.applied || (!status.hidden && /did not start|stopped/.test(status.textContent));
  }, null, { timeout: 30000 });
  const applied = await page.locator("#viewer3d").getAttribute("data-applied");
  if (applied) pass("The 3D view loaded and answered the page");
  else console.log(`NOTE 3D view not available here: ${await page.locator("#viewer3d-status").innerText()}`);
  await page.waitForTimeout(1500);
  await shot("09-3d");

  await page.locator("#view-top").click();
  await page.locator("#example-picker").selectOption("override_autonomous");
  await page.waitForFunction(() => /Opened the example/.test(document.querySelector("#log").textContent) &&
    /the left Toggle and a stack/.test(document.querySelector("#routine-name").value));
  project = await draft();
  assert.equal(project.field, "override");
  assert.equal(project.preload, "none");
  assert.match(await page.locator("#step-list li").first().innerText(), /470° at 12 V, pressing back at 3 V/);
  assert.match(await page.locator("#step-list li").nth(6).innerText(),
               /Lift to Over a stack on a Neutral Goal: claw 22\.66 in up after 3 in/);
  await page.locator("#tab-robot").click();
  const chosen = await page.locator("#pane-robot select").evaluateAll((selects) => selects.map((select) => select.selectedOptions[0]?.text));
  assert(chosen.includes("Override (2026-27)") && chosen.includes("No preload"), JSON.stringify(chosen));
  await page.locator("#tab-step").click();
  pass("The Override example opens on the Override field with no preload, a roller step and a lift action");

  await page.locator("#step-list li").first().click();
  await page.locator("#add-step summary").click();
  await page.locator('#add-step-menu button[data-type="claw"]').click();
  assert.match(await page.locator("#pane-step").innerText(), /Step 2 · Open or close the claw/);
  assert.match(await page.locator("#step-list li").nth(1).innerText(), /Closed/);
  await page.keyboard.press("Control+z");
  await page.waitForFunction(() => JSON.parse(localStorage.getItem("mclib.path-planner.draft.v1")).steps.length === 10);
  pass("A claw step can be added after the first step; Ctrl+Z removes it");

  await page.locator("#run").click();
  await page.waitForFunction(() => /Run \d+ is/.test(document.querySelector("#run-summary").textContent));
  await page.waitForFunction(finished, null, { timeout: 600000 });
  assert.match(await page.locator("#run-summary").innerText(), /ran all 10 steps/);
  await page.locator("#timeline").evaluate((input) => { input.value = input.max; input.dispatchEvent(new Event("input")); });
  assert.match(await page.locator("#score-panel").innerText(), /Red 20\s+Blue 0/);
  const score = await page.locator("#run-summary").innerText();
  assert.match(score, /Autonomous ended at 12\.\d\d s: red 20, blue 0/);
  assert.match(score, /Autonomous Bonus: red 12/);
  assert.match(score, /Win Point: red owns 2 of 6 Pin halves needed[\s\S]*Not earned/);
  pass(`The Override autonomous scores: ${score.split("\n").find((line) => line.startsWith("Autonomous ended"))}`);
  await shot("10-override-score");

  const turnedIn = async (index, pattern) => {
    await page.locator("#step-list li").nth(index).click();
    assert.match(await page.locator("#pane-step").textContent(), pattern);
  };
  await turnedIn(0, /Turned the left Toggle: red now faces into the field/);
  await turnedIn(0, /The roller turned 47\d° in 1\.\d\d s\./);
  await page.locator("#timeline").evaluate((input) => { input.value = "6.5"; input.dispatchEvent(new Event("input")); });
  await page.locator("#tab-telemetry").click();
  const lift = await page.locator("#pane-telemetry").innerText();
  assert.match(lift, /Lift and claw/i);
  assert.match(lift, /a Cup with a yellow Pin/);
  await page.locator("#tab-step").click();
  pass("The roller step says it turned the left Toggle to red, and telemetry shows the stack in the claw");
  await shot("11-override-top");

  const shown3d = async (check) => {
    await page.waitForFunction((name) => {
      const applied = document.querySelector("#viewer3d").dataset.applied;
      if (!applied || !JSON.parse(applied).available) return false;
      const { robot, toggles } = JSON.parse(applied);
      const checks = {
        carryingTheStack: robot.lift_height_in > 12.0 && robot.claw_gap_in < 4.2,
        leftToggleTurned: toggles.left === "red",
      };
      return checks[name];
    }, check, { timeout: 30000 });
    return JSON.parse(await page.locator("#viewer3d").getAttribute("data-applied"));
  };
  const scrub = (seconds) => page.locator("#timeline").evaluate((input, value) => {
    input.value = String(value); input.dispatchEvent(new Event("input"));
  }, seconds);
  const shownAt = async (seconds) => {
    await scrub(seconds);
    await page.waitForFunction((t) => {
      const applied = document.querySelector("#viewer3d").dataset.applied;
      return applied && JSON.parse(applied).available && Math.abs(JSON.parse(applied).t - t) < 0.006;
    }, seconds, { timeout: 30000 });
    return JSON.parse(await page.locator("#viewer3d").getAttribute("data-applied"));
  };
  const robotDrawing = await page.evaluate(async () => (await (await fetch("/api/config")).json()).field_drawings.override.robot);
  const outlineBox = (outline) => ({
    back: Math.min(...outline.map(([forward]) => forward)), front: Math.max(...outline.map(([forward]) => forward)),
    right: Math.min(...outline.map(([, left]) => left)), left: Math.max(...outline.map(([, left]) => left)),
    bottom: robotDrawing.frame.bottom, top: robotDrawing.frame.top,
  });
  const assertSameBox = (label, drawn, expected) => {
    for (const side of Object.keys(expected)) {
      assert(Math.abs(drawn[side] - expected[side]) < 1e-3, `${label} ${side}: drawn at ${drawn[side]} in, vexsim ${expected[side]} in`);
    }
  };
  const turnedAbout = ([forward, up], hinge, degrees) => {
    const c = Math.cos(degrees * Math.PI / 180), s = Math.sin(degrees * Math.PI / 180);
    const along = forward - hinge.forward_in, above = up - hinge.up_in;
    return [hinge.forward_in + along * c + above * s, hinge.up_in - along * s + above * c];
  };
  const drawnPiecesMatchVexsim = async (applied) => {
    const field3d = page.frames().find((frame) => frame.url().startsWith(`${url.origin}/field3d.html`));
    assert(field3d, "The Override 3D view's frame is on the page");
    const drawn = await field3d.evaluate(() => window.drawnRobotPieces());
    const stageRises = [0, ...applied.robot.stage_rises_in];
    const closing = (robotDrawing.claw.open_gap_in - applied.robot.claw_gap_in) / 2;
    const turns = { roller: applied.robot.roller_deg, ...applied.robot.flap_bends_deg };
    const solid = robotDrawing.parts.filter((part) => part.name !== "drivetrain");
    assert.deepEqual(drawn.pieces.map((piece) => piece.key), solid.map((part) => part.key));
    for (const [index, part] of solid.entries()) {
      const up = stageRises[part.stage];
      const left = -part.jaw_side * closing;
      let sideView = part.profile.length ? part.profile
        : [[part.back, part.bottom], [part.front, part.bottom], [part.front, part.top], [part.back, part.top]];
      for (const hinge of [...part.hinges].reverse()) sideView = sideView.map((point) => turnedAbout(point, hinge, turns[hinge.name]));
      const forwards = sideView.map(([forward]) => forward);
      const ups = sideView.map(([, height]) => height);
      assertSameBox(part.key, drawn.pieces[index], { back: Math.min(...forwards), front: Math.max(...forwards),
        right: part.right + left, left: part.left + left, bottom: Math.min(...ups) + up, top: Math.max(...ups) + up });
    }
    assertSameBox("bumpers", drawn.frame[0], outlineBox(robotDrawing.frame.outline));
    assertSameBox("frame inside the bumpers", drawn.frame[1], outlineBox(robotDrawing.frame.bumper_inside));
    return solid.length;
  };
  await page.locator("#view-3d").click();
  await scrub(8.5);
  let view = await shown3d("carryingTheStack");
  assert.deepEqual(view.toggles, { left: "red", bottom: "yellow", top: "yellow", right: "yellow" });
  await page.waitForTimeout(1500);
  await shot("12-override-3d");
  pass(`At 8.5 s the 3D robot carries the stack with the claw ${view.robot.lift_height_in.toFixed(2)} in up and the ` +
       `jaws ${view.robot.claw_gap_in.toFixed(2)} in apart`);
  let pieceCount = await drawnPiecesMatchVexsim(view);
  pass(`With the claw ${view.robot.lift_height_in.toFixed(2)} in up and the jaws closed, the 3D view draws all ` +
       `${pieceCount} pieces and the bumpers where vexsim has them, within 0.001 in`);
  await page.locator("#viewer3d").click({ position: { x: 400, y: 300 } });
  await page.keyboard.press("5");
  await page.waitForTimeout(1000);
  await shot("13-override-lift-up");
  view = await shownAt(0.8);
  const firstFace = view.robot;
  assert(Math.abs(view.robot.lift_height_in - robotDrawing.claw.lowest_in) < 0.2,
         `The lift is ${view.robot.lift_height_in} in up during the roller step, lowest ${robotDrawing.claw.lowest_in} in`);
  assert(view.robot.roller_deg > 250 && view.robot.roller_deg < 350, `The roller had turned ${view.robot.roller_deg}° at 0.8 s`);
  assert.equal(view.toggles.left, "blue");
  pieceCount = await drawnPiecesMatchVexsim(view);
  pass(`With the lift down and the roller ${view.robot.roller_deg.toFixed(0)}° round, the 3D view draws all ${pieceCount} ` +
       "pieces, flaps turned, and the bumpers where vexsim has them, within 0.001 in");
  await page.waitForTimeout(1000);
  await shot("14-override-lift-down");
  const viewerBox = await page.locator("#viewer3d").boundingBox();
  const middle = { x: viewerBox.x + viewerBox.width / 2, y: viewerBox.y + viewerBox.height / 2 };
  await page.mouse.move(middle.x, middle.y);
  await page.mouse.down();
  await page.mouse.move(middle.x + 200, middle.y + 100, { steps: 6 });
  await page.mouse.up();
  await page.mouse.wheel(0, -100);
  await page.waitForTimeout(1000);
  await shot("15-roller-first-face");
  view = await shownAt(1.2);
  const pushedOff = view.robot.y - firstFace.y;
  assert(Math.abs(pushedOff) < 0.3, `The robot moved ${pushedOff} in off the wall between 0.8 and 1.2 s`);
  assert.equal(view.toggles.left, "red");
  pieceCount = await drawnPiecesMatchVexsim(view);
  await page.waitForTimeout(1000);
  await shot("16-roller-second-face");
  pass(`At 0.8 s the 3D roller has turned the left Toggle to blue; at 1.2 s, ${view.robot.roller_deg.toFixed(0)}° round, ` +
       `to red, with the robot ${Math.abs(pushedOff).toFixed(2)} in from where it was and all ${pieceCount} pieces in place`);
  await scrub(15);
  view = await shown3d("leftToggleTurned");
  assert.deepEqual(view.toggles, { left: "red", bottom: "yellow", top: "yellow", right: "yellow" });
  pass("The 3D Toggles turn the way the simulator turned them: red faces into the field on the left");
  pass("The 3D view shows the Override field, the robot with its lift, claw and Toggle roller, and the Pins");
  await page.locator("#view-top").click();

  await page.locator("#export-cpp").click();
  await page.locator("#cpp-dialog[open]").waitFor();
  const cascadeCode = await page.locator("#cpp-code").inputValue();
  assert.match(cascadeCode, /cascade::Rig cascade_rig/);
  assert.match(cascadeCode, /cascade_rig\.beginActions\(/);
  assert.match(cascadeCode, /cascade::ClawPreset::Closed/);
  assert.match(cascadeCode, /cascade_rig\.moveLift\(cascade::LiftPreset::OnNeutralGoal, 1500\)/);
  assert.match(cascadeCode, /cascade::Action::When::AfterDistance/);
  assert.match(cascadeCode, /cascade_rig\.spinRoller\(\*drive, 470, 12, 3, 2000\)/);
  await page.locator("#cpp-dialog .dialog-foot button.primary").click();
  pass("Export C++ writes the lift, claw, roller and actions with the same helpers the simulator ran");

  await page.locator("#example-picker").selectOption("override_match_load");
  await page.waitForFunction(() => /Opened the example/.test(document.querySelector("#log").textContent) &&
    /take a Match Load/.test(document.querySelector("#routine-name").value));
  await page.locator("#step-list li").first().click();
  assert.match(await page.locator("#step-list li").first().innerText(), /Wait[\s\S]*15\.00 s/);
  const waitPane = await page.locator("#pane-step").innerText();
  assert.match(waitPane, /Step 1 · Wait/);
  assert.doesNotMatch(waitPane, /Time limit/);
  await page.locator("#run").click();
  await page.waitForFunction(() => /Run \d+ is/.test(document.querySelector("#run-summary").textContent));
  await page.waitForFunction(finished, null, { timeout: 600000 });
  assert.match(await page.locator("#run-summary").innerText(), /ran all 11 steps/);
  await page.locator("#timeline").evaluate((input) => { input.value = input.max; input.dispatchEvent(new Event("input")); });
  const matchLoadScore = await page.locator("#score-panel").innerText();
  assert.match(matchLoadScore, /Score at 2\d\.\d\d s/);
  assert.match(matchLoadScore, /Red 5/);
  assert.match(matchLoadScore, /Blue 0/);
  assert.match(await page.locator("#run-summary").innerText(), /Autonomous ended at 15\.\d\d s: red 0, blue 0/);
  await page.locator("#step-list li").nth(1).click();
  assert.match(await page.locator("#pane-step .card.result").textContent(), /The Drive Team stood a Cup under the red right Loader with a red\/yellow Pin in it, red half up, 15\.00 s into the match\./);
  await shot("17-override-match-load");
  await page.locator("#export-cpp").click();
  await page.locator("#cpp-dialog[open]").waitFor();
  assert.match(await page.locator("#cpp-code").inputValue(), /\n  pros::delay\(15000\);\n/);
  await page.locator("#cpp-dialog .dialog-foot button.primary").click();
  pass(`The Match Load example waits 15 s, loads at 15.00 s, runs to ${matchLoadScore.match(/Score at (\S+ s)/)[1]} ` +
       "and scores red 5; Export C++ writes the wait as pros::delay(15000)");

  const offsite = requests.filter((value) => /^https?:/.test(value) && new URL(value).origin !== url.origin);
  assert.deepEqual(offsite, [], "The page must not load anything from the internet");
  assert.deepEqual(pageErrors, []);
  pass("No page errors and no requests outside the local server");
  console.log(`${checks} checks passed. Screenshots in ${output}`);
} finally {
  await browser.close();
}
