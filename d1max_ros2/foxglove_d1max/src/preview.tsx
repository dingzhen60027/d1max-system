// Component test harness only: no ROS transport, service or publisher.
import type { PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import { Controller } from "./controller";
import { App } from "./App";
import styles from "./styles.css";
const style=document.createElement("style");style.textContent=styles;document.head.append(style);
const root=document.getElementById("root")!;
const context={panelElement:root,watch(){},subscribe(){},unsubscribeAll(){},saveState(){},setDefaultPanelTitle(){}} as unknown as PanelExtensionContext;
const c=new Controller(context);c.preview=true;
const params=new URLSearchParams(location.search);c.colorScheme=params.get("theme")==="light"?"light":"dark";
createRoot(root).render(<App controller={c}/>);
const mode=params.get("mode");c.state.replay=mode==="replay";
const feed=()=>{
  c.ingest({topic:c.config.stateTopic,message:{data:JSON.stringify({forward_speed:.26,lateral_speed:.02,yaw_speed:.08,battery_power_1:86,battery_power_2:84,motion_status:5,control_source:2,software_emergency_status:1,hardware_emergency_status:1})}});
  c.ingest({topic:c.config.behaviorTopic,message:{data:JSON.stringify({fsm_state:"ready",fault_latched:false,ready_for_navigation:true})}});
  c.ingest({topic:c.config.connectionTopic,message:{data:"connected"}});
  if(mode==="stop")c.ingest({topic:c.config.stateTopic,message:{data:JSON.stringify({...c.state.robot!.value,software_emergency_status:2,hardware_emergency_status:2})}});
};
if(["demo","stop","stale","replay"].includes(mode??"")){
 feed();if(mode==="stale"){c.state.robot!.at-=4000;c.state.connection!.at-=4000;}
 else setInterval(feed,200);
}
window.addEventListener("beforeunload",()=>c.destroy(),{once:true});
