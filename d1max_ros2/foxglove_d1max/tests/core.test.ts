import {test} from 'node:test';
import assert from 'node:assert/strict';
import type {PanelExtensionContext} from '@foxglove/extension';
import {cleanConfig,newState,decode,measuredSpeed,validGateway,motionName} from '../src/model';
import {Controller} from '../src/controller';
const gateway=()=>({session:'0123456789abcdef',service_prefix:'/d1max/monitor/s_0123456789abcdef',mode:'monitor',motion_control_enabled:false,safety_available:true,wall_time:Date.now()/1000});
function mock(){
 const calls:{name:string;request:unknown}[]=[],published:unknown[]=[];
 const context={watch(){},subscribe(){},unsubscribeAll(){},saveState(){},setDefaultPanelTitle(){},async callService(name:string,request:unknown){calls.push({name,request});return{success:true,message:'mock only'};},advertise(){published.push('advertise');},publish(){published.push('publish');}} as unknown as PanelExtensionContext;
 return{c:new Controller(context),calls,published};
}
function feed(c:Controller,g= gateway()){c.ingest({topic:c.config.gatewayTopic,message:{data:JSON.stringify(g)}});}
test('unknown telemetry stays unknown; malformed JSON rejected',()=>{
 assert.equal(measuredSpeed(newState(),performance.now(),2.5).x,undefined);
 for(const data of ['garbage','[]','null'])assert.equal(decode({data}),undefined);
 assert.match(motionName(1),/未完成/);
});
test('velocity accepts only MC, never RobotState fallback; stale values clear',()=>{
 const s=newState(),now=performance.now();s.robot={at:now,value:{forward_speed:.3,lateral_speed:0,yaw_speed:.2}};
 s.velocity={at:now,value:{bad:true}};assert.equal(measuredSpeed(s,now,2.5).x,undefined);
 s.velocity.value={source:'sdk_mc',forward_speed:.5,lateral_speed:0,yaw_speed:.1};assert.equal(measuredSpeed(s,now,2.5).x,.5);
 s.velocity.value.source='sdk_speed_report';assert.equal(measuredSpeed(s,now,2.5).x,undefined);
 s.velocity.value.source='sdk_mc';assert.equal(measuredSpeed(s,now+301,2.5).x,undefined);
});
test('monitor status rejects old controls, forged namespace, stale timestamps and malformed flags',()=>{
 const at=performance.now();assert.ok(validGateway({at,value:gateway()},at));
 for(const extra of [{mode:'live'},{service_prefix:'/d1max_sdk_bridge'},{wall_time:0},{wall_time:NaN},{motion_control_enabled:true},{safety_available:'true'},{session:'old'}])
  assert.equal(validGateway({at,value:{...gateway(),...extra}},at),undefined);
 assert.equal(validGateway({at,value:gateway()},at+1600),undefined);
});
test('configuration drops motion parameters and migrates legacy gateway channel',()=>{
 const c=cleanConfig({linearCommand:20,lateralCommand:20,yawCommand:20,staleSeconds:99,gatewayTopic:'/d1max/console/status'});
 assert.equal(c.staleSeconds,5);assert.equal(c.gatewayTopic,'/d1max/monitor/status');
 assert.ok(!('linearCommand' in c));assert.ok(!('yawCommand' in c));
});
test('mount, telemetry, settings and disposal never call a service or publish',()=>{
 const {c,calls,published}=mock();feed(c);c.saveConfig({});c.destroy();assert.deepEqual(calls,[]);assert.deepEqual(published,[]);
 assert.ok(!('arm' in c));assert.ok(!('command' in c));assert.ok(!('startDrive' in c));
});
test('explicit emergency request uses the only Trigger endpoint',async()=>{
 const {c,calls,published}=mock();feed(c);await c.triggerEstop();c.destroy();
 assert.deepEqual(calls,[{name:'/d1max/monitor/s_0123456789abcdef/soft_estop',request:{}}]);assert.deepEqual(published,[]);
});
test('telemetry without live monitor heartbeat cannot issue estop',async()=>{
 const {c,calls}=mock();c.ingest({topic:c.config.stateTopic,message:{data:'{"software_emergency_status":1}'}});
 await c.triggerEstop();c.destroy();assert.deepEqual(calls,[]);
});
test('observed stop avoids duplicate requests; never turns stop into a release',async()=>{
 const {c,calls}=mock();feed(c);c.ingest({topic:c.config.stateTopic,message:{data:'{"software_emergency_status":2}'}});
 assert.equal(c.estopTriggered,true);await c.triggerEstop();c.destroy();assert.deepEqual(calls,[]);
});
test('bag /clock, monitor replay flag and preview all disable estop',async()=>{
 for(const kind of ['clock','gateway','preview']){
  const {c,calls}=mock();feed(c);
  if(kind==='clock')c.ingest({topic:'/clock',message:{}});
  if(kind==='gateway')feed(c,{...gateway(),mode:'replay'});
  if(kind==='preview')c.preview=true;
  await c.triggerEstop();c.destroy();assert.deepEqual(calls,[]);
 }
});
test('live reset is not bag playback; an existing replay lock survives seek',()=>{
 const {c}=mock();c.context.onRender!({topics:[{name:'/clock',schemaName:'rosgraph_msgs/msg/Clock'}]},()=>{});
 assert.equal(c.state.replay,false);c.context.onRender!({didSeek:true},()=>{});assert.equal(c.state.replay,false);
 c.ingest({topic:'/clock',message:{}});c.context.onRender!({didSeek:true},()=>{});assert.equal(c.state.replay,true);c.destroy();
});
test('pending request prevents double clicks and does not retry',async()=>{
 const {c,calls}=mock();feed(c);let finish:(v:unknown)=>void=()=>{};
 c.context.callService=async(name,request)=>{calls.push({name,request});return await new Promise(resolve=>{finish=resolve;});};
 const one=c.triggerEstop();await c.triggerEstop();assert.equal(calls.length,1);
 finish({success:false,message:'unconfirmed'});await one;c.destroy();assert.equal(calls.length,1);
 assert.ok(c.state.events.some(e=>e.text==='unconfirmed'));
});
