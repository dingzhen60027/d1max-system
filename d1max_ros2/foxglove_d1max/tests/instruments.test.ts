import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import ts from 'typescript';
import {monitorLayout,restoreMapWindow,selectConfiguredMap} from '../scripts/monitor-layout.mjs';
const layout=JSON.parse(readFileSync('layouts/D1Max-Monitor.json','utf8'));
test('motion appears once in plots; both batteries use the same charge-dependent colors',()=>{
 const cfg=layout.configById;
 assert.deepEqual(Object.keys(cfg).filter(id=>id.startsWith('Gauge!')),['Gauge!d1b1','Gauge!d1b2']);
 for(const field of ['forward_speed','lateral_speed','yaw_speed']){
  const values=Object.values(cfg).flatMap((c:any)=>[c.path,...(c.paths??[]).map((p:any)=>p.value)]).filter(Boolean);
  assert.equal(values.filter(v=>v.endsWith('.'+field)).length,1,field+' must not be duplicated');
 }
 for(const id of ['d1b1','d1b2']){
  const c=cfg['Gauge!'+id];assert.equal(c.minValue,0);assert.equal(c.maxValue,100);
  assert.equal(c.colorMode,'colormap');assert.equal(c.colorMap,'red-yellow-green');assert.equal(c.reverse,false);
  assert.equal(c.gradient,undefined);assert.ok(c.foxglovePanelTitle.endsWith('%'));
  assert.equal(c.style,'bar');assert.equal(c.tickInterval,50);
 }
 assert.ok(cfg['Plot!d1linear'].paths.every((p:any)=>!p.value.includes('yaw')));
 assert.equal(cfg['Plot!d1yaw'].paths.length,1);
});
test('both battery bars share one small group below the unchanged PCD and live split',()=>{
 const group=layout.configById['Tab!d1batteries'];
 assert.equal(group.tabs.length,1);assert.equal(group.activeTabIdx,0);
 assert.deepEqual(group.tabs[0].layout,{direction:'column',first:'Gauge!d1b1',second:'Gauge!d1b2',splitPercentage:50});
 assert.equal(layout.layout.direction,'column');
 assert.deepEqual(layout.layout.first,{direction:'row',first:'3D!d1map',second:'3D!d1scene',splitPercentage:50});
 const areas=new Map<string,number>();
 function walk(tree:any,area:number){
  if(typeof tree==='string'){areas.set(tree,area);return;}
  walk(tree.first,area*tree.splitPercentage/100);walk(tree.second,area*(1-tree.splitPercentage/100));
 }
 walk(layout.layout,1);
 assert.ok(areas.get('Tab!d1batteries')!<=.05,'Battery group must occupy no more than 5% of mosaic');
 assert.ok(areas.get('3D!d1scene')!+areas.get('3D!d1map')!>=.65);
 assert.ok(areas.has('d1max-console.D1 状态监控!d1status'));
});
test('single-screen migration preserves both user camera angles and is idempotent',()=>{
 const original=structuredClone(layout);original.configById['3D!d1map']={cameraState:{distance:87}};
 original.configById['3D!d1scene'].cameraState.distance=6;
 const meta={enabled:true,maps:[{frame_id:'d1max_pcd_map',topic:'/d1max/maps/building/points',min:[-20,-20,-8],max:[50,70,9]}]};
 const config=JSON.parse(readFileSync('config/panel.json','utf8'));
 const result=monitorLayout(original,meta,config);
 assert.equal(result.configById['3D!d1map'].cameraState.distance,87);
 assert.equal(result.configById['3D!d1scene'].cameraState.distance,6);
 assert.deepEqual(monitorLayout(result,meta,config),result);
});
test('disabled static map keeps an empty window without changing live sensor settings',()=>{
 const original=structuredClone(layout);
 original.configById['3D!d1map']={topics:{'/d1max/maps/building/points':{visible:true}}};
 const cfg=JSON.parse(readFileSync('config/panel.json','utf8'));
 const result=monitorLayout(original,{enabled:false,maps:[]},cfg);
 assert.deepEqual(result.configById['3D!d1map'].topics,{});
 assert.deepEqual(result.configById['3D!d1map'].layers,{});
 assert.equal(result.configById['3D!d1map'].followTf,undefined);
 assert.deepEqual(result.layout.first,layout.layout.first);
 assert.deepEqual(result.configById['3D!d1scene'].cameraState,original.configById['3D!d1scene'].cameraState);
 assert.ok(!JSON.stringify(result.configById).includes('/d1max/maps/building/points'));
});
test('restoring only the missing PCD window preserves all current panels and exact user split ratios',()=>{
 const previous=structuredClone(layout),current=structuredClone(layout);
 previous.configById['3D!d1map'].cameraState.distance=321;
 previous.configById['3D!d1map'].topics={'/d1max/maps/building/points':{visible:true}};
 current.layout.first='3D!d1scene';delete current.configById['3D!d1map'];
 current.layout.splitPercentage=67.92899408284023;
 current.layout.second.splitPercentage=41.2;
 current.configById['3D!d1scene'].cameraState.phi=73;
 const result=restoreMapWindow(current,previous);
 assert.deepEqual(result.layout.first,previous.layout.first);
 assert.deepEqual(result.layout.second,current.layout.second);
 assert.equal(result.layout.splitPercentage,current.layout.splitPercentage);
 assert.equal(result.configById['3D!d1map'].cameraState.distance,321);
 assert.deepEqual(result.configById['3D!d1map'].topics,{});
 for(const [id,cfg] of Object.entries(current.configById))assert.deepEqual(result.configById[id],cfg,id);
 assert.deepEqual(result.userNodes,current.userNodes);
 assert.deepEqual(restoreMapWindow(result,previous),result);
 assert.ok(!JSON.stringify(result).includes('/d1max/maps/building/points'));
});
test('selecting a single floor changes only map display and map status source',()=>{
 const before=structuredClone(layout);before.layout.splitPercentage=67.92899408284023;
 const meta={enabled:true,maps:[{frame_id:'d1max_floor1_map',topic:'/d1max/maps/floor1/points',min:[-20,-20,-3],max:[45,25,7]}]};
 const panel=JSON.parse(readFileSync('config/panel.json','utf8'));
 const result=selectConfiguredMap(before,meta,panel);
 assert.deepEqual(result.layout,before.layout);
 assert.deepEqual(result.userNodes,before.userNodes);
 for(const [id,cfg] of Object.entries(before.configById))if(id!=='3D!d1map')assert.deepEqual(result.configById[id],cfg);
 assert.deepEqual(Object.keys(result.configById['3D!d1map'].topics),['/d1max/maps/floor1/points']);
 assert.deepEqual(result.configById['3D!d1map'].cameraState.targetOffset,[12.5,2.5,2]);
 assert.equal(result.configById['3D!d1map'].synchronize,false);
 assert.ok(!JSON.stringify(result).includes('/d1max/maps/building/points'));
});
test('heartbeat clears stale native gauges, preserves missing values, and never synthesizes measurements',async()=>{
 const code=ts.transpileModule(layout.userNodes['d1-instruments'].sourceCode,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ES2022}}).outputText;
 const script=await import('data:text/javascript;base64,'+Buffer.from(code).toString('base64'));
 const robot=(obj:unknown)=>script.default({topic:'/d1max_sdk_bridge/robot_state',message:{data:JSON.stringify(obj)}});
 const beat=(wall_time:number)=>script.default({topic:'/d1max/monitor/status',message:{data:JSON.stringify({wall_time})}});
 assert.ok(Number.isNaN(beat(100).forward_speed));
 const initial=robot({received_at_unix:100,forward_speed:-.07,battery_power_1:72});
 assert.equal(initial.forward_speed,-.07);assert.ok(Number.isNaN(initial.battery_power_2));
 assert.equal(beat(101),undefined);
 assert.ok(Number.isNaN(beat(103).forward_speed));assert.equal(beat(104),undefined);
 assert.equal(robot({received_at_unix:105,forward_speed:0}).forward_speed,0);
 assert.ok(Number.isNaN(beat(99).forward_speed));
 assert.ok(Number.isNaN(robot(null).forward_speed));
});
