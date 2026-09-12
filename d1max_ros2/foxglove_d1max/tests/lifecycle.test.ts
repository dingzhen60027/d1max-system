import {test} from "node:test";
import assert from "node:assert/strict";
import type {PanelExtensionContext} from "@foxglove/extension";
import {LifecycleClient,parseManager,type ManagerSnapshot} from "../src/lifecycle";
import {Controller} from "../src/controller";
const snapshot=():ManagerSnapshot=>({api:1,instance:"0123456789abcdef0123456789abcdef",wall_time:Date.now()/1000,monitor:{phase:"stopped",active:false,owned:false,busy:false,error:"",health:{}},web:{phase:"stopped",active:false,owned:false,busy:false,error:"",health:{}}});
test("manager parser fails closed for stale or malformed state",()=>{
 assert.ok(parseManager(snapshot()));
 for(const extra of [{api:0},{instance:"previous"},{wall_time:0},{monitor:{}},{web:{...snapshot().web,owned:"yes"}}])assert.equal(parseManager({...snapshot(),...extra}),undefined);
});
test("mount and destroy only read state; no implicit service lifecycle",async()=>{
 const calls:RequestInit[]=[];
 const client=new LifecycleClient("fake",()=>{},async(_url,init)=>{calls.push(init!);return Response.json(snapshot());});
 client.start();await new Promise(r=>setTimeout(r,10));client.destroy();
 assert.ok(calls.length>0);assert.ok(calls.every(c=>c.method==="GET"));
});
test("duplicate clicks send only one POST; response loss is never retried",async()=>{
 const calls:RequestInit[]=[];let reject!:(e:Error)=>void;
 const client=new LifecycleClient("fake",()=>{},async(_url,init)=>{
  calls.push(init!);if(init?.method==="POST")return await new Promise((_ok,bad)=>{reject=bad;});
  return Response.json(snapshot());
 });
 await client.refresh();
 const one=client.action("web","start");await client.action("web","start");
 assert.equal(calls.filter(c=>c.method==="POST").length,1);
 reject(Error("lost response"));await one;await new Promise(r=>setTimeout(r,10));client.destroy();
 assert.equal(calls.filter(c=>c.method==="POST").length,1);
});
test("unmount aborts HTTP but never sends a stop command",async()=>{
 let aborted=false;const client=new LifecycleClient("fake",()=>{},async(_url,init)=>await new Promise((_ok,bad)=>{
  init?.signal?.addEventListener("abort",()=>{aborted=true;bad(Error("abort"));});
 }));
 client.start();client.destroy();await new Promise(r=>setTimeout(r,5));assert.equal(aborted,true);
});
test("controller blocks preview, replay and pending emergency disconnect",async()=>{
 const context={watch(){},subscribe(){},unsubscribeAll(){},setDefaultPanelTitle(){}} as unknown as PanelExtensionContext;
 const c=new Controller(context),calls:RequestInit[]=[];
 c.lifecycle=new LifecycleClient("fake",()=>{},async(_url,init)=>{calls.push(init!);return Response.json(snapshot());});
 await c.lifecycle.refresh();
 c.preview=true;await c.toggleService("web");c.preview=false;
 c.state.replay=true;await c.toggleService("monitor");c.state.replay=false;
 c.pending=true;await c.toggleService("monitor");c.pending=false;
 c.state.behavior={at:performance.now(),value:{estop_pending:true}};await c.toggleService("monitor");
 assert.equal(calls.filter(c=>c.method==="POST").length,0);
 await c.toggleService("web");c.destroy();
 assert.equal(calls.filter(c=>c.method==="POST").length,1);
});
test("old GET cannot overwrite a newly acknowledged operation",async()=>{
 let finish!:(r:Response)=>void;let reads=0;
 const client=new LifecycleClient("fake",()=>{},async(_url,init)=>{
  if(init?.method==="POST")return Response.json({...snapshot(),web:{...snapshot().web,phase:"starting",busy:true}});
  reads++;if(reads===2)return await new Promise(resolve=>{finish=resolve;});
  return Response.json(snapshot());
 });
 await client.refresh();const old=client.refresh();
 await client.action("web","start");finish(Response.json(snapshot()));await old;
 assert.equal(client.value?.web.phase,"starting");client.destroy();
});
