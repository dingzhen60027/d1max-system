import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true});
const page=await browser.newPage();page.setDefaultTimeout(7000);
const errors=[];page.on('pageerror',e=>errors.push(e.message));const results=[];
try {
 for(const theme of ['dark','light'])for(const [width,height] of [[240,190],[300,220],[420,250]])for(const mode of ['demo','stop','stale','replay','empty']) {
  await page.setViewportSize({width,height});await page.goto('http://127.0.0.1:8768/?mode='+mode+'&theme='+theme);
  await page.locator('.d1-signals').waitFor();
  await page.waitForTimeout(250);
  const overflow=await page.evaluate(()=>[...document.querySelectorAll('.d1-panel,.d1-signals,.d1-signal,.d1-estop')].filter(e=>e.scrollWidth>e.clientWidth+2||e.scrollHeight>e.clientHeight+2).map(e=>e.className));
  assert.deepEqual(overflow,[],theme+' '+width+' '+mode);
  assert.equal(await page.locator('.d1-signal').count(),6);
  assert.equal(await page.locator('button').count(),1);
  assert.equal(await page.locator('nav,textarea,p,article').count(),0);
  assert.ok(await page.locator('.d1-estop').isDisabled());
  assert.equal(await page.locator('.d1-panel').getAttribute('data-telemetry'),['stale','empty'].includes(mode)?'unknown':'fresh');
  if(mode==='stop')assert.equal(await page.locator('.d1-signal.stop').count(),2);
  if(['stale','empty'].includes(mode))assert.equal(await page.locator('.d1-signal.unknown').count(),6);
  const stop=await page.locator('.d1-estop').boundingBox();assert.ok(stop.y+stop.height<=height+1);
  results.push({width,height,theme,mode,overflow:false,onlySafetyButton:true});
  if(width===300&&mode==='stop')await page.screenshot({path:'artifacts/instruments-widget-'+theme+'.png'});
 }
 assert.deepEqual(errors,[]);
 await writeFile('artifacts/instruments-ui-checks.json',JSON.stringify({results,pageErrors:errors},null,2));
 console.log('PASS: '+results.length+' graphic status checks; two themes, three sizes, fresh/stale/unknown/replay/stop; preview never connects to ROS.');
}finally{await browser.close();}
