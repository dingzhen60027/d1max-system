import {useSyncExternalStore, type ReactNode} from "react";
import {Wifi,WifiOff,Dog,Gamepad2,Braces,HelpCircle,ShieldCheck,ShieldAlert,OctagonX,Octagon,Layers,MapPinOff,LoaderCircle,Check,Minus} from "lucide-react";
import type {Controller} from "./controller";
import {controlName,emergencyName,fresh,motionName} from "./model";

function Signal({label,icon,state="unknown",detail}:{label:string;icon:ReactNode;state?:"good"|"stop"|"unknown";detail?:string}){
 return <div className={"d1-signal "+state} role="img" aria-label={label} title={detail??label} data-state={state}>
  {icon}<span className="d1-signal-mark">{state==="good"?<Check/>:state==="stop"?<Minus/>:<HelpCircle/>}</span>
 </div>;
}
export function App({controller:c}:{controller:Controller}){
 useSyncExternalStore(c.onChange,c.snapshot);
 const now=performance.now(),r=fresh(c.state.robot,now,c.config.staleSeconds)?c.state.robot?.value:undefined;
 const b=fresh(c.state.behavior,now)?c.state.behavior?.value:undefined;
 const connected=fresh(c.state.connection,now)&&c.state.connection?.value==="connected";
 const sdkReady=connected&&!!r;
 const mode=c.preview?"界面预览：不连接机器人":c.state.replay?"回放：只读":"只读监控：不申请控制权、不发送运动指令";
 const pending=c.pending||b?.estop_pending===true;
 const stopState=(v:unknown)=>v===2?"stop":v===1?"good":"unknown";
 const label=c.estopTriggered?"已急停":pending?"待确认":!c.safetyAvailable?"不可用":"软件急停";
 const error=!!b?.estop_error&&!c.estopTriggered;
 return <div className={"d1-panel "+(c.colorScheme==="light"?"d1-light":"")} data-telemetry={sdkReady?"fresh":"unknown"} aria-label="D1 Max 图形状态与软件急停">
  <div className="d1-signals">
   <Signal label={sdkReady?"SDK 状态新鲜":"SDK 状态未知或过期"} icon={sdkReady?<Wifi/>:<WifiOff/>} state={sdkReady?"good":"unknown"} detail={mode+"；"+(sdkReady?"SDK 状态新鲜":"遥测已过期：仪表最后值不可作当前状态")}/>
   <Signal label={"姿态："+motionName(r?.motion_status)} icon={<Dog className={r?.motion_status===2?"d1-crouched":""}/>} state={r&&motionName(r.motion_status)!=="未知"?"good":"unknown"} detail={"姿态："+motionName(r?.motion_status)+"；枚举示意图，不代表关节姿态"}/>
   <Signal label={"控制来源："+controlName(r?.control_source)} icon={r?.control_source===1?<Gamepad2/>:r?.control_source===2?<Braces/>:<HelpCircle/>} state={r?.control_source===1||r?.control_source===2?"good":"unknown"} detail={"控制来源："+controlName(r?.control_source)+"；本面板不申请控制权"}/>
   <Signal label={"软件急停："+emergencyName(r?.software_emergency_status)} icon={r?.software_emergency_status===2?<ShieldAlert/>:<ShieldCheck/>} state={stopState(r?.software_emergency_status)}/>
   <Signal label={"硬件急停："+emergencyName(r?.hardware_emergency_status)} icon={r?.hardware_emergency_status===2?<OctagonX/>:<Octagon/>} state={stopState(r?.hardware_emergency_status)}/>
   <Signal label="地图定位尚未接入" icon={<span className="d1-map-symbol"><Layers/><MapPinOff/></span>} detail="PCD 与实时感知独立坐标系，尚未接入定位；不表示已经配准"/>
  </div>
  <button className={"d1-estop "+(c.estopTriggered?"is-stopped":"")+(error?" has-error":"")} aria-label={c.estopTriggered?"软件急停已触发":pending?"等待急停状态反馈":"触发软件急停"} disabled={!c.safetyAvailable||pending||c.estopTriggered}
   title={(error?String(b?.estop_error)+"；":"")+"仅触发，不解除；依赖网络，不能替代机身急停。"+(!c.safetyAvailable?"当前链路不可用，请使用机身急停。":"")}
   onClick={()=>void c.triggerEstop()}>
   {pending?<LoaderCircle className="d1-pending"/>:<OctagonX/>}<span>{label}</span>{error&&<ShieldAlert aria-label="急停反馈异常"/>}
  </button>
 </div>;
}
