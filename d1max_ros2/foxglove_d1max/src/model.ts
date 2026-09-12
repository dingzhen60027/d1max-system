import defaults from "../config/panel.json";
export type Config = typeof defaults;
export const defaultConfig: Config = defaults;
export type Dict = Record<string, unknown>;
export type Sample = { value: Dict; at: number };
export type EventRow = { id: number; at: number; text: string; level: "info" | "warn" | "error" };
export type Gateway = {
  session: string; service_prefix: string; mode: "monitor" | "replay";
  motion_control_enabled: false; wall_time: number; safety_available: boolean;
};
export type State = {
  robot?: Sample; behavior?: Sample; gateway?: Sample; velocity?: Sample; map?: Sample; localization?: Sample;
  connection?: { value: string; at: number }; topics: Set<string>;
  lastSeen: Map<string, number>; counts: Map<string, number>;
  events: EventRow[]; speedHistory: number[]; replay: boolean; frameTime?: number;
};
export function newState(): State {
  return { topics: new Set(), lastSeen: new Map(), counts: new Map(), events: [], speedHistory: [], replay: false };
}
export function object(value: unknown): Dict | undefined {
  return value != null && typeof value === "object" && !Array.isArray(value) ? value as Dict : undefined;
}
export function decode(message: unknown): Dict | undefined {
  const outer = object(message);
  if (!outer) return undefined;
  if (typeof outer.data === "string") {
    try { return object(JSON.parse(outer.data)); } catch { return undefined; }
  }
  return outer;
}
export const number = (value: unknown): number | undefined => typeof value === "number" && Number.isFinite(value) ? value : undefined;
export const fresh = (sample: { at: number } | undefined, now: number, seconds = 2.5): boolean => !!sample && now >= sample.at && now - sample.at <= seconds * 1000;
export function validGateway(sample: Sample | undefined, now: number): Gateway | undefined {
  if (!fresh(sample, now, 1.5)) return undefined;
  const g = sample!.value;
  if (typeof g.session !== "string" || !/^[a-f0-9]{16}$/.test(g.session)) return undefined;
  if (g.service_prefix !== `/d1max/monitor/s_${g.session}` || !["monitor", "replay"].includes(String(g.mode))) return undefined;
  if (number(g.wall_time) === undefined || Math.abs(Date.now() / 1000 - Number(g.wall_time)) > 3) return undefined;
  if (g.motion_control_enabled !== false || typeof g.safety_available !== "boolean") return undefined;
  return g as Gateway;
}
export function measuredSpeed(state: State, now: number, stale: number) {
  if (fresh(state.velocity, now, Math.min(stale,.3))) {
    const mc = state.velocity!.value;
    const x=number(mc.forward_speed), y=number(mc.lateral_speed), yaw=number(mc.yaw_speed);
    if(mc.source==='sdk_mc'&&x!==undefined&&y!==undefined&&yaw!==undefined) return { x, y, yaw, source: "OnMcData" };
  }
  return { x: undefined, y: undefined, yaw: undefined, source: "等待 OnMcData" };
}
export const motionName = (v: unknown) => ({ 1:"站立中（未完成）", 2:"趴下", 3:"匍匐", 4:"锁定", 5:"通用运动", 6:"原地模式", 7:"楼梯模式", 8:"攀爬", 9:"窄道", 10:"步态切换" }[Number(v)] || "未知");
export const controlName = (v: unknown) => ({ 1:"遥控器", 2:"SDK" }[Number(v)] || "未知");
export const emergencyName = (v: unknown) => Number(v) === 2 ? "已触发" : Number(v) === 1 ? "已解除" : "未知";
export function cleanConfig(input: unknown): Config {
  const value = object(input) ?? {};
  const result = { ...defaults };
  for (const key of Object.keys(defaults) as (keyof Config)[]) {
    if (typeof defaults[key] === "string" && typeof value[key] === "string") (result as unknown as Dict)[key] = value[key];
  }
  for (const key of ["staleSeconds"] as const) {
    const v = number(value[key]); if (v !== undefined && v > 0) result[key] = v;
  }
  result.staleSeconds = Math.min(5, Math.max(0.5, result.staleSeconds));
  result.gatewayTopic = defaults.gatewayTopic; // Migrate saved control layouts.
  return result;
}
export function logEvent(state: State, text: string, level: EventRow["level"] = "info") {
  state.events = [{ id: Date.now() + Math.random(), at: Date.now(), text: text.slice(0, 700), level }, ...state.events].slice(0, 60);
}
