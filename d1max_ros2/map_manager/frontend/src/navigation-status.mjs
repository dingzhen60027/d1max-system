// UI affordances only; the ROS command client repeats authoritative admission.
export function navigationControls(data, versionId) {
 const health=data?.health||{},gate=health.gate||{}
 const current=data?.phase==='running'&&data?.version_id===versionId&&!!data?.session_id
 const live=data?.mode==='live',fresh=health.runtime_available===true&&!!health.gate
 const moving=[1,2,3].includes(health.action?.status)
 const armReason=['not_armed','command_missing_or_stale'].includes(gate.reason)
 return {current,live,fresh,moving,gate,
  canArm:current&&live&&fresh&&!moving&&data.enable_motion===true&&!!gate.sdk_session&&!gate.armed&&!gate.sdk_arm_pending&&(gate.arm_ready===true||(gate.arm_ready===undefined&&armReason)),
  canDisarm:current&&live}
}

const reasons={not_armed:'待解锁 SDK 运动',live_motion_disabled:'当前会话只观察，不执行运动',command_missing_or_stale:'等待导航目标',
 sdk_motion_adapter_not_ready:'SDK 运动适配未就绪；核对后台运动能力配置',sdk_control_not_owned:'SDK 未取得控制权',sdk_motion_endpoint_unavailable:'SDK 后台处于只读模式，未开放运动能力',sdk_motion_endpoint_unavailable_or_stale:'SDK 后台只读或运动端点状态过期',
 localization_not_verified:'外参或时间同步尚未核验',localization_not_navigation_ready:'定位尚未满足导航条件',
 sdk_general_low_speed_head_forward_required:'机器人需处于通用低速、站立、机头向前状态',sdk_estop_active_or_unknown:'急停未解除或状态未知',
 simulation_only:'离线仿真 · 不连接机器狗',live_guard_passed:'实机运动已放行',sdk_mc_missing_or_stale:'MC 高频运动数据未就绪',
 odometry_missing_or_stale:'等待有效里程计',scan_missing_or_stale:'等待有效障碍扫描',localization_missing_or_stale:'等待定位状态'}
export function navigationReason(reason){return reasons[reason]||reason||'等待 Nav2 状态'}
