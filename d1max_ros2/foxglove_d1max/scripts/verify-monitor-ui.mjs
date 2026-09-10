// Native display-only QA. No service calls, publish tools, or robot-button clicks.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const title='D1 Max · 实机工作台',dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
const loadedMap=process.argv.includes('--loaded-map');
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const pages=browser.contexts().flatMap(c=>c.pages());
 let page;for(const p of pages)if(await p.getByTestId('layout-menu-button').count()&&(await p.getByTestId('layout-menu-button').textContent()).trim()===title)page=p;
 assert(page);page.setDefaultTimeout(12000);
 const report={time:new Date().toISOString(),url:page.url(),errors:[],assetRequests:[],layoutId:new URLSearchParams(page.url().split('?')[1]).get('layoutId')};
 page.on('pageerror',e=>report.errors.push(e.message));
 page.on('request',r=>{if(/127.0.0.1:8770|\.urdf(?:\?|$)|\.stl(?:\?|$)/i.test(r.url()))report.assetRequests.push(r.url());});
 await page.reload({waitUntil:'domcontentloaded'});await page.locator('.d1-signals').waitFor();
 await page.getByText('LiDAR + RGB · LIVE',{exact:true}).waitFor();await page.getByText('B2 · %',{exact:true}).waitFor();
 await page.getByText('PCD · MAP',{exact:true}).waitFor();
 await page.waitForTimeout(loadedMap?11000:1500);
 assert.equal(await page.locator('.d1-panel button').count(),1);
 assert.equal(await page.locator('.d1-signal').count(),6);
 assert.equal(await page.locator('.d1-panel nav,.d1-panel p,.d1-panel textarea').count(),0);
 for(const label of ['操作','申请控制权','站立','趴下','解除软件急停','准备低速运动'])assert.equal(await page.getByRole('button',{name:label,exact:true}).count(),0,label);
 report.telemetry=await page.locator('.d1-panel').getAttribute('data-telemetry');
 report.stateIcons=await page.locator('.d1-signal').evaluateAll(es=>es.map(e=>({state:e.dataset.state,label:e.getAttribute('aria-label')})));
 report.safetyDisabled=await page.locator('.d1-estop').isDisabled();
 report.overflow=await page.evaluate(()=>[...document.querySelectorAll('.d1-panel,.d1-signals,.d1-signal,.d1-estop')].filter(e=>e.scrollWidth>e.clientWidth+2||e.scrollHeight>e.clientHeight+2).map(e=>e.className));
 assert.deepEqual(report.overflow,[]);
 let data;
 for(const file of await readdir(dir)){let r;try{r=JSON.parse(await readFile(dir+'/'+file,'utf8'));}catch{continue;}if(r.id===report.layoutId){data=r.working?.data??r.baseline.data;break;}}
 assert(data,'Layout must be durably saved before reload');
 assert.equal(Object.keys(data.configById).length,10);
 if(loadedMap){
  assert.deepEqual(Object.keys(data.configById['3D!d1map'].topics),['/d1max/maps/floor1/points']);
  assert.equal(data.configById['3D!d1map'].topics['/d1max/maps/floor1/points'].visible,true);
  assert.equal(data.configById['3D!d1map'].followTf,'d1max_floor1_map');
 }else assert.deepEqual(data.configById['3D!d1map'].topics,{});
 assert.deepEqual(data.configById['3D!d1map'].layers,{});
 assert.doesNotMatch(JSON.stringify(data.configById),/\/d1max\/maps\/building\/points/);
 const before=JSON.parse(await readFile(loadedMap?'artifacts/layout-before-single-floor-load.json':'artifacts/layout-before-pcd-window-restore.json','utf8'));
 const original=JSON.parse(await readFile('artifacts/layout-before-single-floor.json','utf8'));
 assert.deepEqual(data.layout.first,original.layout.first);
 assert.deepEqual(data.layout.second,before.layout.second);
 assert.equal(data.layout.splitPercentage,before.layout.splitPercentage);
 const imported=JSON.parse(await readFile(loadedMap?'artifacts/single-floor-loaded-layout.json':'artifacts/pcd-window-restored-layout.json','utf8'));
 for(const [id,cfg] of Object.entries(before.configById))if(id!=='3D!d1map'){
  // The user can keep orbiting the live camera during QA; never undo that interaction.
  if(loadedMap&&id==='d1max-console.D1 状态监控!d1status')cfg.config.mapTopic='/d1max/maps/floor1/status';
  assert.deepEqual(imported.configById[id],cfg,id+' must be preserved by import');
  const actual=structuredClone(data.configById[id]),expected=structuredClone(cfg);
  if(id==='3D!d1scene'){
   report.liveCameraChangedSinceImport=JSON.stringify(actual.cameraState)!==JSON.stringify(expected.cameraState);
   delete actual.cameraState;delete expected.cameraState;
  }
  assert.deepEqual(actual,expected,id+' settings must remain unchanged');
 }
 assert.deepEqual(data.userNodes,before.userNodes);
 assert.equal(Object.keys(data.configById).filter(id=>id.startsWith('Gauge!')).length,2);
 for(const field of ['forward_speed','lateral_speed','yaw_speed']){
  const values=Object.values(data.configById).flatMap(c=>[c.path,...(c.paths??[]).map(p=>p.value)]).filter(Boolean);
  assert.equal(values.filter(v=>v.endsWith('.'+field)).length,1);
 }
 for(const id of ['d1b1','d1b2']){
  const c=data.configById['Gauge!'+id];assert.equal(c.colorMode,'colormap');assert.equal(c.colorMap,'red-yellow-green');assert.equal(c.reverse,false);
  assert.equal(c.style,'bar');
 }
 assert.ok(!Object.keys(data.configById).some(id=>/^(RawMessages|Markdown)!/.test(id)));
 assert.deepEqual(Object.keys(data.configById).filter(id=>id.startsWith('Tab!')),['Tab!d1batteries']);
 assert.equal(data.configById['Tab!d1batteries'].tabs.length,1);
 assert.equal(data.layout.direction,'column');
 const group=data.configById['Tab!d1batteries'].tabs[0].layout;
 assert.equal(group.first,'Gauge!d1b1');assert.equal(group.second,'Gauge!d1b2');
 assert.doesNotMatch(JSON.stringify(data),/foxglove.Urdf|d1max_model\/|127.0.0.1:8770/);
 assert.deepEqual(report.errors,[]);assert.deepEqual(report.assetRequests,[]);
 report.singlePage=true;report.nativeGauges=2;report.motionChartsNotDuplicated=true;report.batteryColorMap='red-yellow-green';report.noRobotCommands=true;
 report.compactBatteryGroup=true;
 report.staticPcdUnloaded=!loadedMap;report.multifloorPcdUnloaded=true;
 report.pcdWindowRestored=true;report.otherPanelsUnchanged=true;
 await writeFile('artifacts/monitor-layout-final.json',JSON.stringify(data,null,2)+'\n');
 const cdp=await page.context().newCDPSession(page),shot=await cdp.send('Page.captureScreenshot',{format:'png',fromSurface:false,captureBeyondViewport:false});
 await writeFile(loadedMap?'artifacts/single-floor-loaded-native.png':'artifacts/pcd-window-restored-native.png',Buffer.from(shot.data,'base64'));await cdp.detach();
 await writeFile(loadedMap?'artifacts/single-floor-loaded-checks.json':'artifacts/pcd-window-restored-checks.json',JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify(report,null,2));
}finally{await browser.close();}
