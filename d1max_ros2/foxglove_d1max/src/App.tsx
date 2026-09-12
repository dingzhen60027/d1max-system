import {useSyncExternalStore} from "react";
import {Wifi,WifiOff,Globe,Power,ExternalLink,HelpCircle,ShieldAlert,OctagonX,Octagon,LoaderCircle,CheckCircle2,AlertTriangle,LocateFixed} from "lucide-react";
import type {Controller} from "./controller";
import type {Component} from "./lifecycle";
import {emergencyName,fresh} from "./model";

function LifecycleButton({c,name,sdkReady}:{c:Controller;name:Component;sdkReady:boolean}){
 const client=c.lifecycle,ready=client?.available===true,s=ready?client?.value?.[name]:undefined;
 const busy=client?.busy(name)===true,active=!!(s?.active||s?.owned);
 const healthy=name==="web"?s?.phase==="running":s?.phase==="connected"&&sdkReady;
 const warning=s&&["failed","degraded","conflict"].includes(s.phase);
 const stopping=s?.phase==="stopping";
 const loc=fresh(c.state.localization,performance.now(),1.5)&&typeof c.state.localization?.value.wall_time==="number"&&Math.abs(Date.now()/1000-c.state.localization.value.wall_time)<2?c.state.localization?.value:undefined;
 const locText=loc?.localized===true?'定位已锁定（调试）':loc?'定位未锁定 / 数据降级':'定位未运行或无新鲜状态';
 const pendingStop=c.pending||(fresh(c.state.behavior,performance.now())&&c.state.behavior?.value.estop_pending===true);
 const disabled=c.preview||!ready||busy||s?.phase==="conflict"||(name==="monitor"&&(c.state.replay||pendingStop));
 const label=name==="monitor"?(active?"Disconnect":"Connect"):(active?"Stop Web":"Start Web");
 const status=!ready?"管理器未连接":busy?(stopping?"停止中…":"处理中…"):healthy?(name==="web"?"运行中":"数据正常"):s?.phase==="conflict"?"端口占用":s?.phase==="failed"?(active?"清理未完成":"执行失败"):active?"等待数据":"未启动";
 const detail=(s?.error||client?.error||status)+"；"+(name==="web"?"停止时结束 Web 及其处理任务；不删除地图":"Disconnect 关闭本机通信服务，不会使机器人急停");
 return <div className={"d1-service "+(healthy?"is-healthy":warning?"is-warning":"")} data-service={name} data-phase={s?.phase??"unavailable"}>
  <button className="d1-service-button" disabled={disabled} title={detail} onClick={()=>void c.toggleService(name)}>
   {busy?<LoaderCircle className="d1-pending"/>:name==="web"?<Globe/>:healthy?<Wifi/>:<WifiOff/>}
   <span className="d1-service-copy"><strong>{label}</strong><small role="status">{status}</small></span>
   {name==="web"?<span title={locText} aria-label={locText} style={{color:loc?.localized===true?'#4ade80':loc?'#fbbf24':'#94a3b8'}}><LocateFixed className="d1-service-mark"/></span>:healthy?<CheckCircle2 className="d1-service-mark"/>:warning?<AlertTriangle className="d1-service-mark"/>:<Power className="d1-service-mark"/>}
  </button>
  {name==="web"&&s?.phase==="running"&&<a className="d1-web-open" href="http://127.0.0.1:8766" target="_blank" rel="noreferrer" aria-label="打开地图处理 Web" title="打开地图处理 Web"><ExternalLink/></a>}
 </div>;
}
export function App({controller:c}:{controller:Controller}){
 useSyncExternalStore(c.onChange,c.snapshot);
 const now=performance.now(),r=fresh(c.state.robot,now,c.config.staleSeconds)?c.state.robot?.value:undefined;
 const b=fresh(c.state.behavior,now)?c.state.behavior?.value:undefined;
 const sdkReady=fresh(c.state.connection,now)&&c.state.connection?.value==="connected"&&!!r;
 const pending=c.pending||b?.estop_pending===true;
 const label=c.estopTriggered?"已急停":pending?"待确认":!c.safetyAvailable?"不可用":"软件急停";
 const error=!!b?.estop_error&&!c.estopTriggered;
 return <div className={"d1-panel "+(c.colorScheme==="light"?"d1-light":"")} data-telemetry={sdkReady?"fresh":"unknown"} aria-label="D1 Max 连接、Web 与软件急停">
  <div className="d1-lifecycle"><LifecycleButton c={c} name="monitor" sdkReady={sdkReady}/><LifecycleButton c={c} name="web" sdkReady={sdkReady}/></div>
  <div className="d1-safety-row">
   <button className={"d1-estop "+(c.estopTriggered?"is-stopped":"")+(error?" has-error":"")} aria-label={c.estopTriggered?"软件急停已触发":pending?"等待急停状态反馈":"触发软件急停"} disabled={!c.safetyAvailable||pending||c.estopTriggered}
    title={(error?String(b?.estop_error)+"；":"")+"仅触发，不解除；依赖网络，不能替代机身急停。"+(!c.safetyAvailable?"当前链路不可用，请使用机身急停。":"")} onClick={()=>void c.triggerEstop()}>
    {pending?<LoaderCircle className="d1-pending"/>:<OctagonX/>}<span>{label}</span>{error&&<ShieldAlert aria-label="急停反馈异常"/>}
   </button>
   <div className={"d1-hardware "+(r?.hardware_emergency_status===2?"is-stopped":"")} role="img" aria-label={"硬件急停："+emergencyName(r?.hardware_emergency_status)} title={"硬件急停："+emergencyName(r?.hardware_emergency_status)}>
    {r?.hardware_emergency_status===2?<OctagonX/>:r?.hardware_emergency_status===1?<Octagon/>:<HelpCircle/>}
   </div>
  </div>
 </div>;
}
