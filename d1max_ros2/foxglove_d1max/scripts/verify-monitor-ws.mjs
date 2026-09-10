// Read-only protocol handshake. Never advertises or publishes client channels.
import assert from 'node:assert/strict';
const report={capabilities:[],services:[],topics:[]};
await new Promise((resolve,reject)=>{
 const ws=new WebSocket('ws://127.0.0.1:8769',['foxglove.sdk.v1','foxglove.websocket.v1']);
 const timer=setTimeout(()=>{ws.close();resolve();},4000);
 ws.onerror=()=>{clearTimeout(timer);reject(Error('Monitor websocket failed'));};
 ws.onmessage=({data})=>{
  if(typeof data!=='string')return;const m=JSON.parse(data);
  if(m.op==='serverInfo')report.capabilities=m.capabilities;
  if(m.op==='advertiseServices')report.services.push(...m.services.map(s=>s.name));
  if(m.op==='advertise')report.topics.push(...m.channels.map(c=>c.topic));
 };
});
assert.deepEqual([...report.capabilities].sort(),['connectionGraph','services']);
assert.equal(report.services.length,1,JSON.stringify(report));
assert.match(report.services[0],/^\/d1max\/monitor\/s_[a-f0-9]{16}\/soft_estop$/);
assert.ok(report.topics.includes('/d1max/maps/building/points'));
console.log(JSON.stringify(report));
