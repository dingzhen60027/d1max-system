// Scoped native Foxglove checks. Never activates a robot control.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
const name='D1 Max · 实机工作台';
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try {
 let page;
 for(const p of browser.contexts().flatMap(c=>c.pages())) {
  if(await p.getByTestId('layout-menu-button').count() &&
     (await p.getByTestId('layout-menu-button').innerText()).trim()===name &&
     decodeURIComponent(p.url()).includes('127.0.0.1:8769')){page=p;break;}
 }
 assert(page,'Open the live workbench on port 8769 first.');
 page.setDefaultTimeout(8000);
 await page.evaluate(async()=>desktopBridge.activateTab(await desktopBridge.getTabId()));
 async function save() {
  await page.keyboard.press('Escape');
  await page.getByTestId('layout-menu-button').click();
  const row=page.getByRole('menuitem',{name,exact:true}).last();
  await row.hover();await row.getByTestId('layout-actions').click();
  const action=page.getByRole('menuitem',{name:'保存更改',exact:true});
  if(await action.isEnabled())await action.click();
  await page.keyboard.press('Escape');await page.keyboard.press('Escape');
 }
 await save();
 const report={time:new Date().toISOString(),layout:name,method:'Non-blank native canvas samples; no robot publications or service calls',windows:[],pageErrors:[]};
 page.on('pageerror',err=>report.pageErrors.push(err.message));
 const cdp=await page.context().newCDPSession(page);
 await page.reload({waitUntil:'domcontentloaded'});
 await page.locator('.d1-panel').waitFor();
 assert((await page.locator('body').innerText()).includes('视频/点云与 SDK 是独立链路'),'Updated extension not loaded');
 assert(await page.getByRole('button',{name:'软件急停',exact:true}).isDisabled());
 async function samples(count){
  // WebGL buffers can be cleared between paints: reject small/blank PNGs.
  const found=Array(count).fill(null);
  for(let attempt=0;attempt<30&&found.some(x=>!x);attempt++){
   const images=await page.locator('canvas').evaluateAll((cs,n)=>cs.slice(0,n).map(c=>c.toDataURL()),count);
   images.forEach((png,i)=>{if(png.length>100000)found[i]={bytes:png.length,hash:createHash('sha256').update(png).digest('hex')};});
   await page.waitForTimeout(150);
  }
  assert(found.every(Boolean),'Expected non-blank point cloud and camera canvases');return found;
 }
 async function observe(label,count=3){
  const before=await samples(count),start=Date.now();
  await page.waitForTimeout(3000);
  const after=await samples(count);
  const result={label,seconds:(Date.now()-start)/1000,changed:before.map((v,i)=>v.hash!==after[i].hash),before,after};
  report.windows.push(result);assert(result.changed.every(Boolean),'Native sensor display stopped changing: '+label);
 }
 async function capture(file){
  const result=await cdp.send('Page.captureScreenshot',{format:'png',fromSurface:false,captureBeyondViewport:false});
  await writeFile('artifacts/'+file,Buffer.from(result.data,'base64'));
 }
 await page.getByText('感知总览',{exact:true}).click();
 await observe('overview');
 await page.getByText('机器人模型',{exact:true}).click();
 await page.waitForTimeout(4000);
 await capture('live-model.png');
 await page.getByText('雷达与 IMU',{exact:true}).click();
 await observe('lidar-imu',1);
 await page.getByText('感知总览',{exact:true}).click();
 await observe('overview-after-tab-switch');
 await page.waitForTimeout(8000);
 await observe('overview-later');
 await capture('live-overview.png');
 await cdp.detach();
 await save();
 await writeFile('artifacts/live-ui-checks.json',JSON.stringify(report,null,2));
 console.log(JSON.stringify(report,null,2));
} finally {await browser.close();}
