// Native UI checks only. Never clicks a robot action or calls a ROS service.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {writeFile,readdir,readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const title='D1 Max · 实机工作台';
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try {
 let page;for(const p of browser.contexts().flatMap(c=>c.pages()))if(await p.getByTestId('layout-menu-button').count()&&(await p.getByTestId('layout-menu-button').innerText()).trim()===title&&decodeURIComponent(p.url()).includes('127.0.0.1:8769'))page=p;
 assert(page,'Open live workbench first');page.setDefaultTimeout(8000);
 await page.evaluate(async()=>desktopBridge.activateTab(await desktopBridge.getTabId()));
 const errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 async function save(){
  await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
  const row=page.getByRole('menuitem',{name:title,exact:true}).last();await row.hover();await row.getByTestId('layout-actions').click();
  const save=page.getByRole('menuitem',{name:'保存更改',exact:true});if(await save.isEnabled())await save.click();
  await page.keyboard.press('Escape');await page.keyboard.press('Escape');
 }
 await save();await page.reload({waitUntil:'domcontentloaded'});
 await page.getByText('SDK 在线',{exact:true}).waitFor();
 assert(await page.getByRole('button',{name:'软件急停',exact:true}).isDisabled());
 async function sample(count,minBytes){
  const found=Array(count).fill(null);
  for(let attempt=0;attempt<40&&found.some(x=>!x);attempt++){
   const images=await page.locator('canvas').evaluateAll((cs,n)=>cs.slice(0,n).map(c=>c.toDataURL()),count);
   images.forEach((png,i)=>{if(png.length>minBytes)found[i]={bytes:png.length,hash:createHash('sha256').update(png).digest('hex')};});
   await page.waitForTimeout(150);
  }
  assert(found.every(Boolean),'Expected textured, non-blank native canvases');return found;
 }
 const windows=[];
 async function observe(name,count,minBytes){
  await page.getByText(name,{exact:true}).click();
  const before=await sample(count,minBytes);await page.waitForTimeout(3000);const after=await sample(count,minBytes);
  const changed=before.map((x,i)=>x.hash!==after[i].hash);assert(changed.every(Boolean),'No live refresh: '+name);
  windows.push({name,changed,before,after});
 }
 assert.equal(await page.getByText('环绕感知',{exact:true}).count(),0);
 await observe('感知总览',3,100000);
 await page.getByText('机器人模型',{exact:true}).click();
 await page.waitForTimeout(1500);
 await observe('感知总览',3,100000);
 await page.waitForTimeout(5000);
 assert((await page.locator('.d1-panel').innerText()).includes('SDK 在线'));
 assert((await page.locator('.d1-panel').innerText()).includes('只读遥测'));
 const cdp=await page.context().newCDPSession(page);
 const shot=await cdp.send('Page.captureScreenshot',{format:'png',fromSurface:false,captureBeyondViewport:false});
 await writeFile('artifacts/merged-perception.png',Buffer.from(shot.data,'base64'));await cdp.detach();
 await save();
 const dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';let saved;
 for(const f of await readdir(dir)){const r=JSON.parse(await readFile(dir+'/'+f,'utf8'));if(r.name===title)saved=r;}
 assert(saved);assert(!errors.length,errors.join('\n'));
 const data=saved.working?.data??saved.baseline.data;
 const scene=data.configById['3D!d1scene'];
 for(const topic of ['/front_lidar','/rear_lidar','/front_camera/image_compressed','/rear_camera/image_compressed'])assert.equal(scene.topics[topic].visible,true);
 assert.equal(scene.layers['d1-urdf'].visible,true);
 assert.equal(data.configById['3D!d1surround'],undefined);
 assert.equal(data.configById['Tab!d1workspace'].tabs.length,4);
 await writeFile('artifacts/merged-layout-final.json',JSON.stringify(data,null,2));
 const report={time:new Date().toISOString(),layoutId:saved.id,windows,pageErrors:errors,sdkOnline:true,softwareEstopButtonDisabled:true,combinedScene:true,separateSurroundTab:false};
 await writeFile('artifacts/merged-ui-checks.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
}finally{await browser.close();}
