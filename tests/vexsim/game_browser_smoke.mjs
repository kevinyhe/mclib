// Uses an existing Playwright/Chromium installation; installs nothing.
import assert from 'node:assert/strict';
import { readFile, mkdir, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const [playwright, executablePath, recordingPath, outputPath] = process.argv.slice(2);
if (!outputPath) throw Error('Usage: node game_browser_smoke.mjs PLAYWRIGHT CHROME RECORDING OUTPUT');
const recording = JSON.parse(await readFile(recordingPath, 'utf8'));
const output = resolve(outputPath);
await mkdir(output, {recursive:true});
const {chromium} = await import(pathToFileURL(resolve(playwright)).href);
const browser = await chromium.launch({executablePath:resolve(executablePath), headless:true,
  args:['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
const errors = [], requests = [];
let checks = 0;
try {
  const page = await browser.newPage({viewport:{width:1440,height:900}});
  page.on('pageerror', error => errors.push(error.message));
  page.on('request', request => requests.push({url:request.url(),method:request.method()}));
  await page.goto('http://127.0.0.1:8766', {timeout:60000});
  await page.waitForFunction(() => window.metroReplay &&
    Object.values(window.metroReplay.snapshot().assets).every(Boolean), null, {timeout:120000});
  console.log('Native robot, field and block CAD assets loaded');
  const fieldGeometry = await page.evaluate(() => window.metroReplay.snapshot().fieldGeometry);
  assert.deepEqual(fieldGeometry, {loaders:4, parkZones:2, embeddedBlocks:0, simplifiedPropsVisible:false});
  checks++;
  for (const index of [0,50,65,80,200,450,500,556,571,586,650,829,1000,1500,0,450]) {
    const actual = await page.evaluate(index => {
      window.metroReplay.seek(index); return window.metroReplay.snapshot();
    }, index);
    const expected = recording.frames[index];
    assert.equal(actual.index,index);
    assert.equal(actual.t,expected.t);
    assert.equal(actual.error,'');
    assert.equal(actual.scraper.available,true,actual.scraper.reason);
    assert.equal(actual.scraper.parts.length,6);
    assert.equal(actual.scraper.visual_only,true);
    for (const key of ['x','y','theta']) assert(Math.abs(actual.robot[key]-expected.robot[key])<1e-10,key);
    assert.equal(actual.ghost.visible,true);
    for (const key of ['x','y','theta']) assert(Math.abs(actual.ghost[key]-expected.odom[key])<1e-10,'ghost '+key);
    assert.deepEqual(actual.blocks,expected.blocks);
    assert.deepEqual(actual.metro,expected.metro);
    assert.deepEqual(actual.game,expected.game);
    checks += 9;
  }
  const commandEvents=recording.events.filter(e=>e.type==='pneumatic_output' && e.name==='scraper');
  const down=commandEvents.find(e=>e.value===true).t;
  const up=commandEvents.find(e=>e.value===false && e.t>down).t;
  for(const [time,progress] of [[down,0],[down+.15,.5],[down+.3,1],[up,1],[up+.15,.5],[up+.3,0],[down+.15,.5]]) {
    const index=recording.frames.reduce((best,f,i)=>Math.abs(f.t-time)<Math.abs(recording.frames[best].t-time)?i:best,0);
    const actual=await page.evaluate(i=>{window.metroReplay.seek(i);return window.metroReplay.snapshot();},index);
    assert(Math.abs(actual.scraper.progress-progress)<1e-7);
    checks++;
  }
  for(const [name,time] of [['scraper-up',0],['scraper-down',down+.3]]) {
    const index=recording.frames.findIndex(f=>f.t>=time-1e-8);
    await page.evaluate(i=>window.metroReplay.seek(i),index);
    await page.screenshot({path:resolve(output,name+'.png')});
  }
  await page.evaluate(()=>window.metroReplay.seek(450));
  await page.screenshot({path:resolve(output,'native-game.png')});
  await page.locator('#replay-play').click();
  await page.waitForFunction(() => window.metroReplay.snapshot().index > 450);
  await page.locator('#replay-play').click();
  const paused = await page.evaluate(() => window.metroReplay.snapshot().index);
  await page.waitForTimeout(150);
  assert.equal(await page.evaluate(() => window.metroReplay.snapshot().index),paused);
  checks++;
  await page.locator('#replay-ghost').click();
  assert.equal(await page.evaluate(() => window.metroReplay.snapshot().ghost.visible),false);
  await page.locator('#replay-ghost').click();
  for (const playing of [false,true]) {
    await page.evaluate(() => window.metroReplay.seek(450));
    if (playing) await page.locator('#replay-play').click();
    await page.locator('#replay-restart').click();
    const restarted = await page.evaluate(() => window.metroReplay.snapshot());
    assert.equal(restarted.playing,true);assert(restarted.index<450);
    await page.locator('#replay-play').click();
  }
  checks+=6;
  assert.equal(requests.some(r => r.method !== 'GET'),false);
  assert.equal(requests.some(r => new URL(r.url).hostname !== '127.0.0.1'),false);
  assert.deepEqual(errors,[]);
  checks += 3;
  await writeFile(resolve(output,'browser-results.json'),JSON.stringify({checks,errors,requests},null,2));
  console.log(`PASS ${checks} browser checks; native-game.png saved`);
} finally { await browser.close(); }
