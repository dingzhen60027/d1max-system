export type Component="monitor"|"web";
export type Phase="stopped"|"starting"|"stopping"|"running"|"connected"|"degraded"|"failed"|"conflict";
export interface Service{phase:Phase;active:boolean;owned:boolean;busy:boolean;error:string;health:Record<string,unknown>}
export interface ManagerSnapshot{api:1;instance:string;wall_time:number;monitor:Service;web:Service}
export function parseManager(value:unknown):ManagerSnapshot|undefined{
 if(!value||typeof value!=="object")return;
 const v=value as ManagerSnapshot;
 if(v.api!==1||!/^[a-f0-9]{32}$/.test(v.instance)||!Number.isFinite(v.wall_time)||Math.abs(Date.now()/1000-v.wall_time)>4)return;
 for(const name of ["monitor","web"] as const){
  const s=v[name];
  if(!s||!["stopped","starting","stopping","running","connected","degraded","failed","conflict"].includes(s.phase)||typeof s.active!=="boolean"||typeof s.owned!=="boolean"||typeof s.busy!=="boolean"||typeof s.error!=="string"||!s.health||typeof s.health!=="object")return;
 }
 return v;
}
export class LifecycleClient{
 value?:ManagerSnapshot;at=0;error="";pending=new Set<Component>();uncertain=new Set<Component>();
 private alive=true;private polling=false;private epoch=0;private timer?:ReturnType<typeof setInterval>;private aborts=new Set<AbortController>();
 constructor(private token:string,private changed:()=>void,private transport:typeof fetch=fetch,private url="http://127.0.0.1:8771"){}
 get available(){return !!this.token&&!!this.value&&performance.now()-this.at<3500;}
 start(){
  if(!this.token){this.error="本机管理服务尚未配置";this.changed();return;}
  if(this.timer)return;
  void this.refresh();this.timer=setInterval(()=>void this.refresh(),1000);
 }
 private async send(path:string,method="GET",body?:unknown){
  const abort=new AbortController();this.aborts.add(abort);
  const timer=setTimeout(()=>abort.abort(),2500);
  try{
   const response=await this.transport(this.url+path,{method,signal:abort.signal,cache:"no-store",headers:{Authorization:"Bearer "+this.token,...(body?{"Content-Type":"application/json"}:{})},...(body?{body:JSON.stringify(body)}:{})});
   const data=await response.json();
   if(!response.ok)throw Error(String(data?.error??"管理服务请求失败"));
   const parsed=parseManager(data);if(!parsed)throw Error("管理服务状态无效或过期");
   return parsed;
  }finally{clearTimeout(timer);this.aborts.delete(abort);}
 }
 async refresh(){
  if(!this.alive||this.polling||!this.token)return;
  this.polling=true;const epoch=this.epoch;
  try{
   const value=await this.send("/v1/status");if(!this.alive||epoch!==this.epoch)return;
   this.value=value;this.at=performance.now();
   if(this.pending.size===0)this.error="";
   for(const name of this.uncertain)if(!this.pending.has(name))this.uncertain.delete(name);
  }catch(error){if(this.alive&&epoch===this.epoch){this.value=undefined;this.error="本机管理器未连接："+String(error instanceof Error?error.message:error);}}
  finally{this.polling=false;if(this.alive)this.changed();}
 }
 busy(name:Component){return this.pending.has(name)||this.uncertain.has(name)||this.value?.[name].busy===true;}
 async action(name:Component,desired:"start"|"stop"){
  if(!this.alive||!this.available||this.busy(name)||this.value?.[name].phase==="conflict")return;
  this.epoch++;this.pending.add(name);this.changed();
  try{
   const value=await this.send("/v1/"+name+"/"+desired,"POST",{request_id:crypto.randomUUID(),instance:this.value!.instance});
   if(this.alive){this.value=value;this.at=performance.now();this.error="";}
  }catch(error){if(this.alive){this.uncertain.add(name);this.error=String(error instanceof Error?error.message:error)+"；正在核对状态，不自动重试";}}
  finally{this.epoch++;this.pending.delete(name);if(this.alive){this.changed();void this.refresh();}}
 }
 destroy(){this.alive=false;if(this.timer)clearInterval(this.timer);for(const abort of this.aborts)abort.abort();this.aborts.clear();}
}
