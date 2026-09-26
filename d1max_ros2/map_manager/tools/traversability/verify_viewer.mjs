import assert from 'node:assert/strict';
import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
const base=process.env.D1MAX_TRAV_VIEW_URL||'http://127.0.0.1:8766/traversability/sc_pgo_20260904_143406_width_aware/';
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const errors=[];
try {
 const page=await browser.newPage({viewport:{width:1600,height:1040}});
 page.on('pageerror',e=>errors.push(e.message));
 page.on('response',r=>{if(r.status()>=400)errors.push(`${r.status()} ${r.url()}`)});
 await page.goto(base);await page.waitForFunction(()=>window.__TRAV_READY__===true,{},{timeout:30000});
 const ground=await page.evaluate(()=>window.__TRAV_REPORT__.kind==='ground_only');
 const prefix=ground?'ground':'traversability';
 await page.locator('canvas').first().waitFor();await page.waitForTimeout(500);
 assert.equal(await page.locator('canvas').count(),2);
 assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth),1600);
 await page.screenshot({path:`/tmp/d1max-${prefix}-two-floors.png`});
 await page.getByRole('button',{name:'3D 总览',exact:true}).click();await page.waitForTimeout(600);
 assert.equal(await page.locator('canvas').count(),1);
 await page.screenshot({path:`/tmp/d1max-${prefix}-3d.png`});
 await page.getByLabel('原始结构',{exact:true}).check();
 await page.getByLabel('采集轨迹',{exact:true}).check();await page.waitForTimeout(250);
 await page.getByRole('button',{name:'楼梯特写',exact:true}).click();await page.waitForTimeout(500);
 if(ground){await page.getByLabel('原始结构',{exact:true}).uncheck();await page.getByLabel('采集轨迹',{exact:true}).uncheck();await page.waitForTimeout(250);}
 await page.screenshot({path:`/tmp/d1max-${prefix}-stairs.png`});
 await page.getByRole('button',{name:'配置与结果 ↗',exact:true}).click();
 assert(await page.getByRole('link',{name:ground?'完整地面 PCD':'彩色 PCD',exact:true}).isVisible());
 const files=ground?['ground.pcd','lower_ground.pcd','upper_ground.pcd','stairs_ground.pcd','ground_colored.ply','ground_points.npz']:['traversability.pcd','traversability.npz'];
 for(const file of [...files,'pipeline.yaml','report.json'])assert((await page.request.get(base+file)).ok());
 await page.getByRole('button',{name:'关闭',exact:true}).click();
 await page.getByRole('button',{name:'双层对照',exact:true}).click();
 if(ground){
   await page.getByLabel('点大小').fill('2');
   assert.equal(await page.locator('#point-size-value').textContent(),'2 px');
   await page.setViewportSize({width:390,height:844});await page.waitForTimeout(250);
   assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth),390);
   assert.equal(await page.locator('canvas').count(),2);
 }
 assert.deepEqual(errors,[]);
 console.log(JSON.stringify({ok:true,checks:['two floors','3D overview','stairs detail','structure overlay','recorded trajectory','downloads','no horizontal overflow'],errors}));
} finally {await browser.close()}
