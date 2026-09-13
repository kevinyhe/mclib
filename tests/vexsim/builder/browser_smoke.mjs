// Optional browser regression: reuse an existing Playwright/Chromium install.
// Start server.py first. No package installation or mocked physics responses.
import assert from "node:assert/strict";
import { mkdir, readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";

const options = {};
for (let index = 2; index < process.argv.length; index += 2) {
  const key = process.argv[index];
  if (!["--playwright", "--browser", "--url", "--output", "--mode"].includes(key) || !process.argv[index + 1]) {
    throw new Error("Usage: node browser_smoke.mjs --playwright /path/to/playwright/index.mjs --browser /path/to/chrome [--url http://127.0.0.1:8765] [--output /tmp/builder-browser] [--mode inspect]");
  }
  options[key.slice(2)] = process.argv[index + 1];
}
if (!options.playwright || !options.browser) throw new Error("Supply existing --playwright and --browser paths; this test installs nothing.");
assert(!options.mode || options.mode === "inspect", "Only optional --mode inspect is supported (GET-only existing-record checks)");
const url = new URL(options.url || "http://127.0.0.1:8765");
assert(["127.0.0.1", "localhost"].includes(url.hostname) && url.protocol === "http:", "Only the local builder is in scope");
const output = resolve(options.output || "/tmp/mclib-builder-browser");
await mkdir(output, { recursive: true });
const { chromium } = await import(pathToFileURL(resolve(options.playwright)).href);
const browser = await chromium.launch({ executablePath: resolve(options.browser), headless: true, args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const errors = [];
const requests = [];
let checks = 0;
function pass(label) { checks += 1; console.log(`PASS ${label}`); }
try {
  const page = await browser.newPage({ viewport: { width: 1366, height: 768 } });
  page.on("pageerror", error => errors.push(error.message));
  page.on("request", request => requests.push(request.url()));
  if (options.mode === "inspect") await page.route("**/*", route => ["GET", "HEAD"].includes(route.request().method()) ? route.continue() : route.abort());
  await page.goto(url.href);
  await page.waitForFunction(() => document.querySelector("#connection").textContent.includes("CONNECTED"));
  assert.equal(await page.locator("#routine-name").evaluate(input => getComputedStyle(input).minHeight), "30px");
  assert.equal(await page.locator("#play-pause").evaluate(button => getComputedStyle(button).cursor), "not-allowed");
  pass("Local server and current frontend connect without mocked responses");

  async function assertNativeFrame(frame) {
    assert.equal(await page.locator("#field-view").inputValue(), "native");
    assert.equal(await page.locator("#native-viewer").getAttribute("src"), "/native/viewer.html");
    await page.waitForFunction(t => {
      const value = document.querySelector("#native-viewer").dataset.applied;
      if (!value) return false;
      const applied = JSON.parse(value);
      return t === null ? applied.t === null && applied.robot === null : Math.abs(applied.t - t) < 1e-8;
    }, frame?.t ?? null, { timeout: 15000 });
    const applied = JSON.parse(await page.locator("#native-viewer").getAttribute("data-applied"));
    if (frame) {
      if (![frame.body_length_in, frame.body_width_in].every(value => Number.isFinite(value) && value > 0)) {
        assert.equal(applied.robot, null, "Legacy frames must not fabricate chassis geometry");
        assert.equal(applied.available, false);
        assert.match(await page.locator("#native-status").innerText(), /lacks chassis dimensions/);
        return;
      }
      assert.equal(applied.available, true);
      assert(Math.abs(applied.robot.x - frame.true_y * .0254) < 1e-8, "Native X is mclib forward Y in metres");
      assert(Math.abs(applied.robot.y + frame.true_x * .0254) < 1e-8, "Native Y is negative mclib right X in metres");
      assert(Math.abs(applied.robot.theta + frame.true_heading * Math.PI / 180) < 1e-8, "Native CCW heading matches the selected raw truth frame");
    }
  }

  function assertViewerNetwork() {
    const web = requests.filter(value => /^https?:/.test(value)).map(value => new URL(value));
    assert(web.every(value => value.origin === url.origin), "Native assets must be local; no CDN requests");
    assert(!web.some(value => ["/state", "/input"].includes(value.pathname)), "Recorded playback must not poll or drive a second simulator");
  }

  async function checkHistoryFetchOrdering(olderId, newerId) {
    // Delay a real history GET response; its JSON and all physics remain real.
    const olderUrl = new URL(`/api/runs/${olderId}`, url).href;
    let release, captured;
    const held = new Promise(resolve => { release = resolve; });
    const fetched = new Promise(resolve => { captured = resolve; });
    const delay = async route => {
      const response = await route.fetch(); captured(); await held;
      await route.fulfill({ response });
    };
    await page.locator("#routine-name").fill("Draft must survive history loading");
    const draftName = await page.locator("#routine-name").inputValue();
    await page.route(olderUrl, delay);
    try {
      await page.locator("#run-history").selectOption(olderId);
      await fetched;
      assert.equal(await page.locator("#load-recorded").isDisabled(), true, "Loading a recording disables copying stale inputs");
      await page.locator("#load-recorded").dispatchEvent("click");
      assert.equal(await page.locator("#routine-name").inputValue(), draftName, "Even a dispatched click cannot load the previous recording while fetching");
      await page.locator("#run-history").selectOption(newerId);
      await page.waitForFunction(id => document.querySelector("#diagnostic-output").textContent.includes(id) && !document.querySelector("#load-recorded").disabled, newerId);
      const delivered = page.waitForResponse(response => response.url() === olderUrl);
      release(); await delivered;
      await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
      assert.equal(await page.locator("#run-history").inputValue(), newerId, "A late older response cannot replace the latest choice");
      assert((await page.locator("#diagnostic-output").innerText()).includes(newerId));
      assert.equal(await page.locator("#routine-name").inputValue(), draftName, "History response ordering never edits the draft");
    } finally {
      release(); await page.unroute(olderUrl, delay);
    }
    const fail = route => route.abort("failed");
    await page.route(olderUrl, fail);
    try {
      await page.locator("#run-history").selectOption(olderId);
      await page.waitForFunction(() => document.querySelector("#notice").textContent.startsWith("Could not load recorded run:"));
      assert.equal(await page.locator("#run-history").inputValue(), newerId, "Failed fetch restores the displayed recording's selection");
      assert.equal(await page.locator("#load-recorded").isDisabled(), false, "Failed fetch restores usable controls for the prior recording");
      assert.equal(await page.locator("#routine-name").inputValue(), draftName, "Failed fetch does not alter draft inputs");
      await page.locator("#load-recorded").click();
      const newer = await (await page.request.get(new URL(`/api/runs/${newerId}`, url).href)).json();
      assert.equal(await page.locator("#routine-name").inputValue(), newer.spec.name);
      assert.equal(await page.locator("#step-list > li").count(), newer.spec.steps.length);
    } finally {
      await page.unroute(olderUrl, fail);
    }
    pass("Delayed, out-of-order and failed history GETs cannot load stale recording inputs");
  }

  async function inspectExistingRecords() {
    const ids = ["77eb016f43104299a760dcb9660d0260", "8c88b42312c94168b504668d0074377b", "584d6466a323499a91b89fff4b5dc6f8"];
    const history = await (await page.request.get(new URL("/api/runs", url).href)).json();
    const jobs = Array.isArray(history) ? history : history.runs || [];
    if (!ids.every(id => jobs.some(job => job.id === id))) {
      if (options.mode === "inspect") throw new Error("The three preserved multi-step recordings are required for --mode inspect.");
      console.log("SKIP preserved-record browser cases: this server does not contain all three historical fixtures.");
      return;
    }
    await page.waitForFunction(() => document.querySelectorAll("#run-history option").length > 1);
    for (const id of ids) {
      await page.locator("#run-history").selectOption(id);
      await page.waitForFunction(id => document.querySelector("#diagnostic-output").textContent.includes(id), id);
      await page.locator("#load-recorded").click();
      await page.locator("#step-list .step-select").nth(1).click();
      const editorSelection = await page.locator("#selected-number").innerText();
      await page.locator("#step-results tr").first().click();
      const recordedJob = await (await page.request.get(new URL(`/api/runs/${id}`, url).href)).json();
      await assertNativeFrame(recordedJob.result.trace.filter(frame => frame.step === 0).at(-1));
      assert.equal(await page.locator("#playback-step").innerText(), "STEP 01 / SETTLING");
      assert.equal(await page.locator("#motor-voltage").innerText(), "0.0 / 0.0 V");
      assert.equal(await page.locator("#target-error").innerText(), id === ids[2] ? "1.61 in" : "1.39 in");
      assert.equal(await page.locator("#selected-number").innerText(), editorSelection, "Inspecting a recording must not change editor selection");
      assert.equal(await page.locator("#controller-carrot").innerText(), "—", "Legacy records do not contain controller carrots");
      await page.locator("#timeline").evaluate(input => { input.value = "0.5"; input.dispatchEvent(new Event("input", { bubbles: true })); });
      assert.equal(await page.locator("#selected-number").innerText(), editorSelection, "Scrubbing must not select an editor step");
      // Wait for the explicit Load in editor save, then compare around another scrub.
      await page.waitForTimeout(250);
      const saved = await page.evaluate(() => localStorage.getItem("mclib.motion-builder.v1"));
      await page.locator("#timeline").evaluate(input => { input.value = "1"; input.dispatchEvent(new Event("input", { bubbles: true })); });
      assert.equal(await page.evaluate(() => localStorage.getItem("mclib.motion-builder.v1")), saved, "Scrubbing must not mutate the saved draft");
      if (id === ids[2]) {
        await page.locator("#step-results tr").nth(1).click();
        assert.equal(await page.locator("#playback-step").innerText(), "STEP 02 / SKIPPED");
        assert.equal(await page.locator("#truth-pose").innerText(), "—");
        assert.equal(await page.locator("#motor-voltage").innerText(), "—");
        assert(await page.locator("#field-empty").isVisible());
        await assertNativeFrame(null);
      }
    }
    pass("Shared-timestamp result inspection uses its own stopped frame; skipped steps have no fabricated telemetry");
    pass("Replay and editor selections remain separate, and legacy telemetry stays unavailable");
    pass("Native vexsim explicitly reports unavailable legacy chassis geometry; no shape is fabricated");
    await checkHistoryFetchOrdering(ids[0], ids[1]);
    await checkComparisonFetchOrdering(ids[0], ids[1], ids[2]);
  }

  async function checkComparisonFetchOrdering(olderId, currentId, newerId) {
    const olderUrl = new URL(`/api/runs/${olderId}`, url).href;
    const jobs = {};
    for (const id of [olderId, currentId, newerId]) jobs[id] = await (await page.request.get(new URL(`/api/runs/${id}`, url).href)).json();
    async function chooseCurrent(id) {
      await page.locator("#run-history").selectOption(id);
      await page.waitForFunction(id => document.querySelector("#diagnostic-output").textContent.includes(id) && !document.querySelector("#load-recorded").disabled, id);
      await page.waitForFunction(id => ![...document.querySelector("#compare-history").options].some(option => option.value === id), id);
    }
    async function chooseComparison(id) {
      await page.locator("#compare-history").selectOption(id);
      if (id) await page.waitForFunction(name => document.querySelector("#comparison-context").textContent.startsWith(`Compared with ${name}:`) && !document.querySelector("#comparison-context").hidden, jobs[id].spec.name);
      else assert.equal(await page.locator("#comparison-context").isHidden(), true);
    }
    async function withDelayedComparison(action) {
      let release, captured, intercepted = false;
      const held = new Promise(resolve => { release = resolve; });
      const fetched = new Promise(resolve => { captured = resolve; });
      const delay = async route => {
        if (intercepted) { await route.continue(); return; }
        intercepted = true;
        const response = await route.fetch(); captured(); await held; await route.fulfill({ response });
      };
      await page.route(olderUrl, delay);
      try {
        await page.locator("#compare-history").selectOption(olderId); await fetched;
        await action();
        const delivered = page.waitForResponse(response => response.url() === olderUrl);
        release(); await delivered;
        await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
      } finally { release(); await page.unroute(olderUrl, delay); }
    }
    await chooseCurrent(currentId);
    await withDelayedComparison(() => chooseComparison(""));
    assert.equal(await page.locator("#compare-history").inputValue(), "", "Cleared comparison stays cleared after its old GET arrives");
    assert.equal(await page.locator("#comparison-context").isHidden(), true, "A late GET cannot restore an invisible comparison or delta");
    assert.equal(await page.locator("#step-results tr td:nth-child(6)").first().innerText(), "—");

    await withDelayedComparison(() => chooseComparison(newerId));
    assert.equal(await page.locator("#compare-history").inputValue(), newerId, "Newer comparison wins over out-of-order response");
    assert((await page.locator("#comparison-context").innerText()).startsWith(`Compared with ${jobs[newerId].spec.name}:`));
    const fail = route => route.abort("failed");
    await page.route(olderUrl, fail);
    try {
      await page.locator("#compare-history").selectOption(olderId);
      await page.waitForFunction(() => document.querySelector("#notice").textContent.startsWith("Could not load comparison:"));
      assert.equal(await page.locator("#compare-history").inputValue(), newerId, "Failed comparison restores the displayed selection");
    } finally { await page.unroute(olderUrl, fail); }

    await chooseComparison("");
    await withDelayedComparison(() => chooseCurrent(newerId));
    assert.equal(await page.locator("#compare-history").inputValue(), "", "Changing current recording invalidates an outstanding comparison");
    assert.equal(await page.locator("#comparison-context").isHidden(), true);
    await chooseComparison(olderId);
    await chooseCurrent(olderId);
    assert.equal(await page.locator("#compare-history").inputValue(), "", "Current recording never compares with itself");
    assert.equal(await page.locator("#comparison-context").isHidden(), true);
    await chooseCurrent(currentId);
    pass("Comparison clear, out-of-order, failed and changed-record GETs keep dropdown, deltas and overlay consistent");
  }

  await inspectExistingRecords();
  if (options.mode === "inspect") {
    await page.locator("#step-results tr").first().click();
    await page.evaluate(() => scrollTo(0, 0));
    const field = await page.locator(".field-panel").boundingBox();
    const telemetry = await page.locator(".telemetry-panel").boundingBox();
    assert(telemetry.x >= field.x + field.width - 1, "Telemetry is beside the field on a laptop");
    const run = await page.locator("#run-sequence").boundingBox();
    assert(run.y >= 0 && run.y + run.height <= 768, "Run controls remain visible");
    await page.screenshot({ path: `${output}/motion-debugger-inspection.png`, fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.evaluate(() => scrollTo(0, 0));
    await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), "Mobile has no page overflow");
    await page.screenshot({ path: `${output}/motion-debugger-inspection-mobile.png`, fullPage: true });
    pass("Compact desktop telemetry and responsive mobile layout");
    assertViewerNetwork();
    assert.deepEqual(errors, []);
    console.log(`${checks}/${checks} read-only browser checks passed; artifacts: ${output}`);
  } else {

  async function demo(selector, expected) {
    await page.locator(selector).click();
    const submitted = page.waitForResponse(response => response.url().endsWith("/api/runs") && response.request().method() === "POST");
    await page.locator("#run-sequence").click();
    const response = await submitted;
    assert.equal(response.status(), 202);
    const job = await response.json();
    await page.waitForFunction(() => document.querySelector("#telemetry-mode").textContent.startsWith("LIVE"), null, { timeout: 10000 });
    assert.equal(await page.locator("#play-pause").isDisabled(), true, "Live runs cannot replay stale frames");
    assert.equal(await page.locator("#timeline").isDisabled(), true);
    await page.waitForFunction(({ id }) => {
      return document.querySelector("#run-history").value === id &&
        /^(PASSED|FAILED|ERROR|CANCELLED)\b/.test(document.querySelector("#run-status-title").textContent);
    }, { id: job.id, expected }, { timeout: 180000 });
    const completed = await (await page.request.get(new URL(`/api/runs/${job.id}`, url).href)).json();
    assert.equal(completed.status, expected);
    assert(completed.result.trace.length > 50);
    await assertNativeFrame(completed.result.trace.at(-1));
    await page.locator("#fit-view").click();
    await page.waitForFunction(() => !!document.querySelector("#native-viewer").dataset.fitted);
    assert.notEqual(await page.locator("#imu-reading").innerText(), "—", "Completed new runs display actual gyro samples");
    assert.notEqual(await page.locator("#encoder-angle").innerText(), "—", "Completed new runs display actual encoder samples");
    return completed;
  }

  const baseline = await demo("#example-baseline", "failed");
  assert(baseline.result.steps[0].position_error_in > 5);
  assert((await page.locator("#step-results").innerText()).includes("FAIL"));
  pass("Real drive-encoder slip failure remains visible with its recorded trace");
  const trackers = await demo("#example-trackers", "passed");
  assert.equal(trackers.spec.tracking_mode, "two");
  assert(trackers.result.steps[0].position_error_in <= 1.5);
  assert(trackers.result.steps[0].heading_error_deg <= 5);
  pass("Actual C++ boomerang passes unchanged endpoint checks with modeled trackers");

  await page.locator("#compare-history").selectOption(baseline.id);
  const positionDelta = trackers.result.steps[0].position_error_in - baseline.result.steps[0].position_error_in;
  const expectedDelta = `${positionDelta > 0 ? "+" : ""}${positionDelta.toFixed(2)} in`;
  await page.waitForFunction(expected => document.querySelector("#step-results tr td:nth-child(6)")?.textContent === expected, expectedDelta);
  assert.match(await page.locator("#comparison-context").innerText(), /overlay is available in Edit targets/);
  pass("Prior-run comparison shows endpoint delta and identifies its 2D-only path overlay");
  await page.locator("#play-pause").click();
  await page.waitForTimeout(450);
  const firstTime = Number(await page.locator("#timeline").inputValue());
  assert(firstTime > .1 && firstTime < 2);
  await page.locator("#play-pause").click();
  const pausedTime = await page.locator("#timeline").inputValue();
  await page.waitForTimeout(180);
  assert.equal(await page.locator("#timeline").inputValue(), pausedTime);
  await page.locator("#playback-speed").selectOption("2");
  await page.locator("#timeline").evaluate(input => { input.value = input.max; input.dispatchEvent(new Event("input", { bubbles: true })); });
  await assertNativeFrame(trackers.result.trace.at(-1));
  assert((await page.locator("#truth-pose").innerText()).includes("°"));
  pass("Playback advances, pauses, changes speed and scrubs the real recording");

  await page.locator("#step-results tr").first().click();
  assert((await page.locator("#diagnostic-output").innerText()).includes("PASS position"));
  await page.locator("#diagnostic-details").evaluate(element => { element.open = false; });
  await page.evaluate(() => scrollTo(0, 0));
  const runBounds = await page.locator("#run-sequence").boundingBox();
  assert(runBounds.y + runBounds.height <= 768, "Build & run must remain visible at laptop height");
  const workspace = await page.locator(".workspace").boundingBox();
  for (const selector of [".field-panel", ".editor-panel", ".telemetry-panel", ".playback"]) {
    const bounds = await page.locator(selector).boundingBox();
    assert(bounds.y + bounds.height <= workspace.y + workspace.height + 1, `${selector} must not overflow the workspace into the run bar`);
  }
  const fieldBounds = await page.locator(".field-panel").boundingBox();
  const telemetryBounds = await page.locator(".telemetry-panel").boundingBox();
  assert(telemetryBounds.x >= fieldBounds.x + fieldBounds.width - 1, "Telemetry belongs beside the field");
  await page.screenshot({ path: `${output}/motion-builder.png`, fullPage: true });
  pass("Readable laptop layout and inspectable per-step diagnostics");

  await page.locator("#field-view").selectOption("edit");
  const canvas = await page.locator("#field").boundingBox();
  const oldX = await page.locator('#step-inspector input[data-field="x"]').inputValue();
  await page.mouse.click(canvas.x + canvas.width * .66, canvas.y + canvas.height * .38);
  assert.notEqual(await page.locator('#step-inspector input[data-field="x"]').inputValue(), oldX);
  await page.locator("#new-step-type").selectOption("drive");
  await page.locator("#add-step").click();
  assert.equal(await page.locator("#step-list > li").count(), 2);
  await page.getByRole("button", { name: "Move step up", exact: true }).click();
  assert((await page.locator("#step-list > li").first().innerText()).includes("Drive distance"));
  await page.getByRole("button", { name: "Delete step", exact: true }).click();
  assert.equal(await page.locator("#step-list > li").count(), 1);
  pass("Field target placement and ordered sequence editing work");

  const downloadEvent = page.waitForEvent("download");
  await page.locator("#export-project").click();
  const download = await downloadEvent;
  await download.saveAs(`${output}/exported-routine.json`);
  const exported = JSON.parse(await readFile(`${output}/exported-routine.json`, "utf8"));
  assert.equal(exported.steps.length, 1);
  await page.locator("#new-project").click();
  await page.locator("#import-file").setInputFiles(`${output}/exported-routine.json`);
  await page.waitForFunction(() => document.querySelector("#notice").textContent.startsWith("Imported"));
  assert.equal(await page.locator('#step-inspector input[data-field="x"]').inputValue(), String(exported.steps[0].x));
  await page.waitForTimeout(250);
  await page.reload();
  await page.waitForFunction(() => document.querySelector("#connection").textContent.includes("CONNECTED"));
  assert.equal(await page.locator('#step-inspector input[data-field="x"]').inputValue(), String(exported.steps[0].x));
  pass("JSON export/import and local draft persistence survive reload");

  await page.locator("#example-baseline").click();
  const quickSubmitted = page.waitForResponse(response => response.url().endsWith("/api/runs") && response.request().method() === "POST");
  await page.locator("#run-sequence").click();
  const quickJob = await (await quickSubmitted).json();
  await page.locator("#cancel-run").waitFor({ state: "visible" });
  await page.locator("#cancel-run").click();
  await page.waitForFunction(() => document.querySelector("#run-status-title").textContent.startsWith("CANCELLED"), null, { timeout: 10000 });
  const quickCancelled = await (await page.request.get(new URL(`/api/runs/${quickJob.id}`, url).href)).json();
  assert.equal(await page.locator("#play-pause").isDisabled(), !(quickCancelled.partial?.trace?.length));
  assert.equal(await page.locator("#result-summary .summary-cell strong").first().innerText(), "INCOMPLETE");
  pass("Early cancellation never replays a previous job; its own frames are retained if already captured");

  // A pinned body cannot finish the turn, leaving time to cancel after a real
  // live sample. This is a bounded physical run, not a mocked response.
  const partialSpec = { ...trackers.spec, name: "Partial cancellation regression",
    environment: { ...trackers.spec.environment, constrained: true },
    steps: [{ type: "turn", heading: 90, timeout_ms: 20000, volts: 12 }] };
  await page.locator("#import-file").setInputFiles({ name: "partial-cancellation.json", mimeType: "application/json", buffer: Buffer.from(JSON.stringify(partialSpec)) });
  const partialSubmitted = page.waitForResponse(response => response.url().endsWith("/api/runs") && response.request().method() === "POST");
  await page.locator("#run-sequence").click();
  const partialJob = await (await partialSubmitted).json();
  await page.waitForFunction(() => document.querySelector("#telemetry-mode").textContent.startsWith("LIVE") &&
    Number(document.querySelector("#timeline").value) > .1, null, { timeout: 30000 });
  assert.equal(await page.locator("#play-pause").isDisabled(), true);
  await page.locator("#cancel-run").click();
  await page.waitForFunction(() => document.querySelector("#run-status-title").textContent.startsWith("CANCELLED"), null, { timeout: 10000 });
  const interrupted = await (await page.request.get(new URL(`/api/runs/${partialJob.id}`, url).href)).json();
  assert(interrupted.partial.trace.length > 0, "Cancellation preserves this worker's captured physics frames");
  assert.equal(await page.locator("#play-pause").isDisabled(), false);
  assert.equal(await page.locator("#result-summary .summary-cell strong").first().innerText(), "INCOMPLETE");
  assert.match(await page.locator("#step-results").innerText(), /INTERRUPTED/);
  assert.match(await page.locator("#telemetry-mode").innerText(), /INCOMPLETE/);
  await assertNativeFrame(interrupted.partial.trace.at(-1));
  await page.locator("#step-results tr").last().click();
  assert.match(await page.locator("#diagnostic-output").innerText(), /Final stop and acceptance are unverified/);
  pass("Mid-run cancellation retains real native playback and controller telemetry without claiming completed acceptance");

  await page.locator("#run-history").selectOption(trackers.id);
  await page.waitForFunction(id => document.querySelector("#diagnostic-output").textContent.includes(id) && !document.querySelector("#load-recorded").disabled, trackers.id);
  await page.locator("#load-recorded").click();
  assert.equal(await page.locator("#tracking-mode").inputValue(), "two");
  assert.equal(await page.locator("#routine-name").inputValue(), trackers.spec.name);
  assert.equal(await page.locator("#step-list > li").count(), trackers.spec.steps.length);
  assert.equal(await page.locator("#step-inspector select").first().inputValue(), trackers.spec.steps[0].type);
  await page.setViewportSize({ width: 390, height: 844 });
  await page.evaluate(() => scrollTo(0, 0));
  await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), "Mobile layout must not overflow horizontally");
  await page.screenshot({ path: `${output}/motion-builder-mobile.png`, fullPage: true });
  pass("Recorded inputs load correctly and mobile layout has no page overflow");
  assert.deepEqual(errors, []);
  assertViewerNetwork();
  console.log(`${checks}/${checks} browser checks passed; artifacts: ${output}`);
  console.log(`Baseline ${baseline.id}: ${baseline.result.steps[0].position_error_in.toFixed(3)} in`);
  console.log(`Trackers ${trackers.id}: ${trackers.result.steps[0].position_error_in.toFixed(3)} in`);
  }
} finally {
  await browser.close();
}
