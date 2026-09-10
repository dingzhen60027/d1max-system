// Display-only calibration copied from D1 mapping launch; no ROS /tf publisher.
// Sensor/TF header times differ from bag receive times; no verified base-to-lidar link.
// This view intentionally shows sensor-local perception, not SLAM odometry.
import { Input, Message } from "./types.ts";
export const inputs = ["__FRONT_CLOUD__","__REAR_CLOUD__"];
export const output = "/d1max/view/calibration";
const c = __CONFIG__;
export default function script(event:Input<"__FRONT_CLOUD__">|Input<"__REAR_CLOUD__">):Message<"tf2_msgs/msg/TFMessage"> {
  const stamp=event.message.header.stamp;
  return {transforms:[
    {header:{stamp,frame_id:c.displayFrame},child_frame_id:c.frontFrame,transform:{translation:{x:0,y:0,z:0},rotation:{x:c.frontRotation[0],y:c.frontRotation[1],z:c.frontRotation[2],w:c.frontRotation[3]}}},
    {header:{stamp,frame_id:c.frontFrame},child_frame_id:c.rearFrame,transform:{translation:{x:c.rearTranslation[0],y:c.rearTranslation[1],z:c.rearTranslation[2]},rotation:{x:c.rearRotation[0],y:c.rearRotation[1],z:c.rearRotation[2],w:c.rearRotation[3]}}}
  ]};
}
