import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const monitor=JSON.parse(readFileSync('layouts/D1Max-Monitor.json','utf8'));
test('monitor layout has no URDF layers, model tab, assets, control panel or fake map-to-lidar TF',()=>{
 const text=JSON.stringify(monitor);
 assert.doesNotMatch(text,/foxglove.Urdf|8770|D1 状态与操作|d1max_model/);
 assert.ok(!Object.keys(monitor.configById).some(id=>/^(RawMessages|Markdown)!/.test(id)));
 assert.deepEqual(Object.keys(monitor.configById).filter(id=>id.startsWith('Tab!')),['Tab!d1batteries']);
 assert.deepEqual(Object.keys(monitor.configById['3D!d1map'].topics),['/d1max/maps/floor1/points']);
 assert.equal(monitor.configById['3D!d1map'].followTf,'d1max_floor1_map');
 assert.equal(monitor.configById['3D!d1scene'].followTf,'d1max_lidar');
 for(const script of Object.values(monitor.userNodes) as any[])assert.ok(!script.sourceCode.includes('d1max_pcd_map'));
 assert.ok(JSON.stringify(monitor.layout).includes('d1max-console.D1 状态监控!d1status'));
});
test('all native content is visible on one screen; only the single-page battery group is nested',()=>{
 const ids=new Set<string>();function walk(t:any){if(typeof t==='string'){assert.ok(monitor.configById[t],t);ids.add(t);}else if(t){walk(t.first);walk(t.second);}}
 walk(monitor.layout);for(const p of Object.values(monitor.configById) as any[])for(const t of p.tabs??[])walk(t.layout);
 assert.equal(ids.size,Object.keys(monitor.configById).length);
 assert.equal(ids.size,10);
 assert.deepEqual(monitor.layout.first,{direction:'row',first:'3D!d1map',second:'3D!d1scene',splitPercentage:50});
});
test('active startup excludes control gateway and model server; bridge has only estop service',()=>{
 const start=readFileSync('scripts/start_live_monitor.sh','utf8'),bridge=readFileSync('scripts/start_monitor_bridge.sh','utf8');
 assert.doesNotMatch(start,/start_sdk_console|start_gateway|serve-model/);
 assert.ok(start.includes('start_sdk_monitor'));assert.ok(start.includes('pcd_map_publisher'));
 assert.ok(bridge.includes("capabilities:='[services,connectionGraph]'"));
 assert.ok(bridge.includes("client_topic_whitelist:=\"['a^']\""));
 assert.match(bridge,/service_whitelist:.*soft_estop/);
 assert.doesNotMatch(bridge,/clientPublish|recover_estop|take_control|prepare_navigation|velocity\$'/);
});
test('monitor SDK only calls one literal true safety command; no motion or recovery API',()=>{
 const src=readFileSync('../sdk_bridge_ws/src/d1max_sdk_bridge/src/sdk_monitor_bridge.cpp','utf8');
 for(const name of [...src.matchAll(/sdk_->(\w+)\(/g)].map(m=>m[1]))
  assert.ok(['SoftEmergencyStop','Connect','Disconnect','IsConnected','SetDataCallback','SetControlCallback'].includes(name),name);
 assert.equal([...src.matchAll(/create_service</g)].length,1);
 assert.match(src,/SoftEmergencyStop\(true,0/);assert.doesNotMatch(src,/TakeControl\(|Move\(|StandUp\(|Recover|recover_estop|control_lease|guarded_velocity/);
});
