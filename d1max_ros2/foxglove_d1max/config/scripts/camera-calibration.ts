// Foxglove-only nominal projection, NOT measured CameraInfo on the robot.
import { Input } from "./types.ts";
import { CameraCalibration } from "@foxglove/schemas";
export const inputs = ["__IMAGE_TOPIC__"];
export const output = "__CALIBRATION_TOPIC__";
type CameraConfig = {
  [key: string]: unknown;
  frame: string; horizontalFovDeg: number; verticalFovDeg: number;
  calibrated: {width: number; height: number; K: number[]; D: number[]; distortion_model: string} | null;
};
const c: CameraConfig = __CAMERA_CONFIG__;

function jpegSize(data: Uint8Array): [number, number] | undefined {
  if (data[0] !== 0xff || data[1] !== 0xd8) return;
  let offset = 2;
  while (offset + 3 < data.length) {
    if (data[offset] !== 0xff) return;
    while (data[offset] === 0xff) offset++;
    const marker = data[offset++]!;
    if (marker === 0xd9 || marker === 0xda) return;
    if (marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    const length = (data[offset]! << 8) | data[offset + 1]!;
    if (length < 2 || offset + length > data.length) return;
    if ([0xc0,0xc1,0xc2,0xc3,0xc5,0xc6,0xc7,0xc9,0xca,0xcb,0xcd,0xce,0xcf].includes(marker)) {
      if (length < 8) return;
      const height = (data[offset + 3]! << 8) | data[offset + 4]!;
      const width = (data[offset + 5]! << 8) | data[offset + 6]!;
      return width > 0 && height > 0 ? [width, height] : undefined;
    }
    offset += length;
  }
  return;
}

export default function script(event: Input<"__IMAGE_TOPIC__">): CameraCalibration | undefined {
  const size = jpegSize(event.message.data);
  if (!size || event.message.header.frame_id !== c.frame) return;
  const [width, height] = size;
  // Never silently apply a calibration at a different image resolution.
  if (c.calibrated && (c.calibrated.width !== width || c.calibrated.height !== height)) return;
  const fx = width / (2 * Math.tan(c.horizontalFovDeg * Math.PI / 360));
  const fy = height / (2 * Math.tan(c.verticalFovDeg * Math.PI / 360));
  const K = c.calibrated?.K ?? [fx,0,width/2,0,fy,height/2,0,0,1];
  return {
    timestamp: event.message.header.stamp,
    frame_id: c.frame, width, height,
    distortion_model: c.calibrated?.distortion_model ?? "plumb_bob",
    D: c.calibrated?.D ?? [0,0,0,0,0], K: new Float64Array(K),
    R: [1,0,0,0,1,0,0,0,1],
    P: [K[0]!,K[1]!,K[2]!,0,K[3]!,K[4]!,K[5]!,0,K[6]!,K[7]!,K[8]!,0]
  };
}
