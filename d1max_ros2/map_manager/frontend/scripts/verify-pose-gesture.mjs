// All lifecycle and seed writes are fixtures. Never send a real robot request.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {imageToMap,normalizeYaw} from '../src/lib/pose-estimate.js';
const angleDegrees=value=>normalizeYaw(value*Math.PI/180)*180/Math.PI;
await mkdir('artifacts',{recursive:true});
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true});
const page=await browser.newPage({viewport:{width:1600,height:1000}});page.setDefaultTimeout(8000);
const errors=[],requests=[],report={realWrites:0};page.on('pageerror',e=>errors.push(e.message));
const state={phase:'stopped',installed:true,connection:{active:false,phase:'stopped',health:{}},health:{},logs:[],navigation_ready:false};
let holdAck=false,lastId;
try{
 const grid=await (await page.request.get('http://127.0.0.1:8766/api/2d/overview')).json();
 const version=grid.versions.find(v=>v.selected);assert(version);
 await page.route('**/api/**',async route=>{
  const req=route.request(),url=new URL(req.url()),path=url.pathname.split('/').at(-1);
  if(!url.pathname.startsWith('/api/localization/')){
   if(req.method()!=='GET')throw Error('Non-fixture write prohibited: '+url.pathname);
   return route.continue();
  }
  if(req.method()==='POST'){
   const body=req.postDataJSON();requests.push({path,body});
   if(path==='connect')state.connection={active:true,phase:'connected',health:{sdk_fresh:true,lidar_fresh:true,replay:false}};
   else if(path==='start'){
    assert.equal(body.version_id,version.id);state.phase='running';state.version_id=version.id;
    state.health={state:'waiting_initial_pose',sensors:{sdk:true,imu:true,cloud:true},gyro_bias:0,local_ekf_fresh:true,global_ekf_fresh:true,sdk_source:'50hz_stream',fusion:{accepted:'0'}};
   }else if(path==='initial-pose'){
    assert.equal(body.reference,'body');lastId='fixture-'+requests.length;
    if(!holdAck)state.health.command_result={id:lastId,accepted:true,message:'fixture'};
    await new Promise(r=>setTimeout(r,120));
    return route.fulfill({status:202,json:{request_id:lastId,accepted:false,status:'queued'}});
   }else if(path==='stop'){state.phase='stopped';state.health={};}
   else throw Error('Unexpected fixture operation: '+path);
  }
  await route.fulfill({json:state});
 });
 await page.goto('http://127.0.0.1:8766/#/2d/navigation');
 await page.getByRole('heading',{name:'单楼层定位调试'}).waitFor();
 const submit=page.getByRole('button',{name:'提交定位初值',exact:true});
 assert(await page.getByRole('button',{name:'启动定位',exact:true}).isDisabled());assert(await submit.isDisabled());
 const img=page.getByAltText('初始定位地图'),arrow=page.getByTestId('initial-pose-arrow');
 async function drag(dx,dy,{cancel=false,click=false}={}){
  await img.scrollIntoViewIfNeeded();const box=await img.boundingBox();
  const x=box.x+box.width/2,y=box.y+box.height/2;
  await page.mouse.move(x,y);await page.mouse.down();
  if(!click)await page.mouse.move(x+dx,y+dy,{steps:5});
  if(cancel)await page.keyboard.press('Escape');
  await page.mouse.up();return box;
 }
 await drag(60,0);assert.equal(requests.length,0);await arrow.waitFor();
 await page.screenshot({path:'artifacts/localization-web-offline.png',fullPage:true});
 await page.getByRole('button',{name:'连接机器狗',exact:true}).click();
 await page.getByRole('button',{name:'启动定位',exact:true}).click();
 await page.getByText('等待初始位姿',{exact:true}).waitFor();
 const expectedXY=imageToMap(version,.5,.5);
 for(const [dx,dy,degrees] of [[60,0,0],[0,-60,90],[-60,0,180],[0,60,-90]]){
  const before=requests.length;await drag(dx,dy);
  await page.waitForFunction(()=>!document.querySelector('.pose-estimate-canvas[aria-disabled="true"]'));
  assert.equal(requests.length,before+1);
  const body=requests.at(-1).body;
  const expected=angleDegrees(degrees+Number(version.metadata.origin[2]||0)*180/Math.PI);
  assert(Math.abs(angleDegrees(body.yaw*180/Math.PI-expected))<.001);
  assert(Math.abs(body.x-expectedXY.x)<.001&&Math.abs(body.y-expectedXY.y)<.001);
  const line=await arrow.locator('line').evaluate(e=>({dx:Number(e.getAttribute('x2'))-Number(e.getAttribute('x1')),dy:Number(e.getAttribute('y2'))-Number(e.getAttribute('y1'))}));
  assert(Math.abs(angleDegrees(Math.atan2(-line.dy,line.dx)*180/Math.PI-degrees))<.001);
 }
 let before=requests.length;await drag(0,0,{click:true});await drag(50,20,{cancel:true});assert.equal(requests.length,before);
 const box=await img.boundingBox();await drag(box.width*.6,0);
 await page.waitForFunction(()=>!document.querySelector('.pose-estimate-canvas[aria-disabled="true"]'));
 assert.equal(requests.length,before+1);
 holdAck=true;
 const queued=page.waitForResponse(r=>r.url().endsWith('/initial-pose'));
 await drag(50,0);await queued;before=requests.length;
 await drag(0,50);assert.equal(requests.length,before);assert(await submit.isDisabled());
 state.health.command_result={id:lastId,accepted:true};holdAck=false;
 await page.waitForFunction(()=>!document.querySelector('.pose-estimate-canvas[aria-disabled="true"]'));
 await page.getByText('精确坐标 / 高级设置',{exact:true}).click();
 await page.getByLabel('机头朝向 · °',{exact:true}).fill('90');await submit.click();
 await page.waitForFunction(()=>!document.querySelector('.pose-estimate-canvas[aria-disabled="true"]'));
 assert(Math.abs(requests.at(-1).body.yaw-Math.PI/2)<1e-9);
 state.health={...state.health,state:'tracking',localized:true,correction_age:.08,fusion:{accepted:'14'}};
 await page.getByText('定位已锁定',{exact:true}).waitFor();
 await page.screenshot({path:'artifacts/localization-web-fixture-tracking.png',fullPage:true});
 report.widths=[];
 for(const width of [1920,1440,1100,800,390]){
  await page.setViewportSize({width,height:1080});await page.waitForTimeout(150);
  const overflow=await page.evaluate(()=>[...document.querySelectorAll('.localization-top [data-slot=card],.localization-seed,.localization-map,.localization-page')].filter(e=>e.scrollWidth>e.clientWidth+3).map(e=>e.className));
  assert.deepEqual(overflow,[],String(width));report.widths.push({width,overflow});
 }
 await page.setViewportSize({width:1600,height:1000});
 await page.getByRole('button',{name:'停止',exact:true}).click();await page.getByText('未启动',{exact:true}).waitFor();
 assert.deepEqual(errors,[]);report.passed=true;report.requests=requests;report.errors=errors;
 await writeFile('artifacts/localization-web-qa.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
}finally{await browser.close()}
