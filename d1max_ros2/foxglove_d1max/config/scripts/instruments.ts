// Display-only freshness guard. Never publishes ROS or interpolates new measurements.
import { Input } from "./types.ts";
export const inputs = ["__STATE_TOPIC__", "__MONITOR_TOPIC__"];
export const output = "/d1max/view/instruments";
type Values = {forward_speed:number;lateral_speed:number;yaw_speed:number;battery_power_1:number;battery_power_2:number};
let latest:Values|undefined;
let received=NaN;
let expired=true;
const empty=():Values=>({forward_speed:NaN,lateral_speed:NaN,yaw_speed:NaN,battery_power_1:NaN,battery_power_2:NaN});
const finite=(v:unknown):number=>typeof v==="number"&&Number.isFinite(v)?v:NaN;
export default function script(event:Input<"__STATE_TOPIC__">|Input<"__MONITOR_TOPIC__">):Values|undefined {
 try {
  const r=JSON.parse(event.message.data);
  if(!r||typeof r!=="object"||Array.isArray(r))throw Error("invalid");
  if(event.topic==="__STATE_TOPIC__"){
   received=finite(r.received_at_unix);expired=false;
   latest={forward_speed:finite(r.forward_speed),lateral_speed:finite(r.lateral_speed),yaw_speed:finite(r.yaw_speed),battery_power_1:finite(r.battery_power_1),battery_power_2:finite(r.battery_power_2)};
   return latest;
  }
  const age=finite(r.wall_time)-received;
  if(!Number.isFinite(age)||age<0||age>__STALE_SECONDS__){
   if(!expired||!latest){expired=true;latest=empty();return latest;}
  }
 } catch {received=NaN;expired=true;latest=empty();return latest;}
}
