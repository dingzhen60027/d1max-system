// Foxglove-local structured telemetry: does NOT publish ROS messages.
import { Input } from "./types.ts";
export const inputs = ["__STATE_TOPIC__"];
export const output = "/d1max/view/telemetry";
type Telemetry = {forward_speed:number;lateral_speed:number;yaw_speed:number;battery_power_1:number;battery_power_2:number;motion_status:number;control_source:number};
export default function script(event:Input<"__STATE_TOPIC__">):Telemetry|undefined {
  try {
    const r=JSON.parse(event.message.data);
    if (!r || typeof r!=="object" || Array.isArray(r)) return;
    const finite=(v:unknown):number=>typeof v==="number"&&Number.isFinite(v)?v:NaN;
    return {forward_speed:finite(r.forward_speed),lateral_speed:finite(r.lateral_speed),yaw_speed:finite(r.yaw_speed),battery_power_1:finite(r.battery_power_1),battery_power_2:finite(r.battery_power_2),motion_status:finite(r.motion_status),control_source:finite(r.control_source)};
  } catch { return; }
}
