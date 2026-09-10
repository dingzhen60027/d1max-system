import type {PanelExtensionContext,MessageEvent} from "@foxglove/extension";
import {cleanConfig,decode,fresh,logEvent,measuredSpeed,newState,object,validGateway,type Config} from "./model";
export class Controller {
 state=newState();config:Config;colorScheme:"light"|"dark"="dark";preview=false;pending=false;
 version=0;listeners=new Set<()=>void>();private alive=true;private timer:ReturnType<typeof setInterval>;
 constructor(public context:PanelExtensionContext){
  this.config=cleanConfig(object(context.initialState)?.config);
  context.setDefaultPanelTitle?.("D1 Max · 状态监控");
  for(const key of ["topics","currentFrame","didSeek","currentTime","colorScheme"] as const)context.watch(key);
  context.onRender=(render,done)=>{try{
   if(render.colorScheme)this.colorScheme=render.colorScheme;
   // Live reconnect/reset may also set didSeek. Drop stale samples, but only
   // actual /clock data or a backend replay flag proves ROS bag playback.
   if(render.didSeek){const replay=this.state.replay;this.state=newState();this.state.replay=replay;}
   if(render.topics)this.state.topics=new Set(render.topics.map(t=>t.name));
   for(const event of render.currentFrame??[])this.ingest(event);
  }finally{done();}};
  this.subscribe();this.timer=setInterval(()=>this.emit(),200);
 }
 subscribe(){const c=this.config;this.context.subscribe([...new Set([c.stateTopic,c.behaviorTopic,c.connectionTopic,c.faultTopic,c.eventTopic,c.velocityTopic,c.gatewayTopic,c.mapTopic,"/clock"])].filter(Boolean).map(topic=>({topic})));}
 saveConfig(value:unknown){this.config=cleanConfig(value);this.state=newState();this.context.saveState({config:this.config});this.subscribe();this.emit();}
 ingest({topic,message}:Pick<MessageEvent,"topic"|"message">){
  const now=performance.now(),c=this.config;this.state.lastSeen.set(topic,now);this.state.counts.set(topic,(this.state.counts.get(topic)??0)+1);
  if(topic==="/clock")this.state.replay=true;
  const key=topic===c.stateTopic?"robot":topic===c.behaviorTopic?"behavior":topic===c.gatewayTopic?"gateway":topic===c.velocityTopic?"velocity":topic===c.mapTopic?"map":undefined;
  if(key){const value=decode(message);if(value){this.state[key]={value,at:now};if(value.replay_latched===true||(key==="gateway"&&value.mode==="replay"))this.state.replay=true;}}
  if(topic===c.connectionTopic)this.state.connection={value:String(object(message)?.data??"unknown"),at:now};
  if(topic===c.eventTopic)logEvent(this.state,String(object(message)?.data??"事件"));
  if(topic===c.faultTopic)logEvent(this.state,String(object(message)?.data??"故障"),"error");
  if(key==="robot"){const s=measuredSpeed(this.state,now,c.staleSeconds);if(s.x!==undefined&&s.y!==undefined)this.state.speedHistory=[...this.state.speedHistory,Math.hypot(s.x,s.y)].slice(-80);}
 }
 onChange=(listener:()=>void)=>{this.listeners.add(listener);return()=>{this.listeners.delete(listener);};};
 snapshot=()=>this.version;
 emit(){if(!this.alive)return;this.version++;for(const listener of this.listeners)listener();}
 get gateway(){return validGateway(this.state.gateway,performance.now());}
 get safetyAvailable(){return !this.preview&&!this.state.replay&&this.gateway?.mode==="monitor"&&this.gateway.safety_available===true&&!!this.context.callService;}
 get estopTriggered(){return fresh(this.state.robot,performance.now(),this.config.staleSeconds)&&this.state.robot?.value.software_emergency_status===2;}
 // Sole outbound operation: no generic command, arm, advertise or velocity API.
 async triggerEstop(){
  const gateway=this.gateway;
  if(!this.safetyAvailable||!gateway||this.pending||this.estopTriggered)return;
  this.pending=true;this.emit();let timer:ReturnType<typeof setTimeout>|undefined;
  try{
   const result=object(await Promise.race([this.context.callService!(gateway.service_prefix+"/soft_estop",{}),new Promise((_,reject)=>{timer=setTimeout(()=>reject(Error("请求回执超时；请核对实机急停状态，不会自动重试")),4000);})]));
   if(result?.success!==true)throw Error(String(result?.message??"急停请求未确认"));
   logEvent(this.state,String(result.message??"请求已提交，等待实机状态"),"warn");
  }catch(error){logEvent(this.state,error instanceof Error?error.message:String(error),"error");}
  finally{if(timer)clearTimeout(timer);this.pending=false;this.emit();}
 }
 destroy(){this.alive=false;clearInterval(this.timer);this.context.onRender=undefined;this.context.unsubscribeAll();this.listeners.clear();}
}
