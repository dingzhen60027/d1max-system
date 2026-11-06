#pragma once
#include "execution_transport_core.hpp"
#include "navigation_motion_core.hpp"
#include "sdk_ownership_core.hpp"

namespace d1monitor::execution3 {
// The legacy navigation health combines missing telemetry and lost authority.
// Keep it unchanged for legacy callers; schema-3 separates their meanings.
// `owned` below is the explicit SDK ACK + ordered RobotState ownership fact,
// never motion permission by itself. The independent freshness field gates
// initial admission and EVERY nonzero writer selection.
// The raw RobotState and ownership event drain run independently. A newer
// explicit non-SDK raw state must veto the older confirmed ownership fact;
// an unchanged raw SDK state does not gain a new timestamp during a gap.
inline Health robotExecutionHealth(const NavigationHealth& robot,const Ownership& owner,
    double now,bool mc_source_fresh,int observed_control_source) {
  Health result;
  result.connected=robot.connected&&!robot.replay;
  result.owned=owner.connected&&!owner.replay&&owner.confirmed&&owner.control_source==2&&
    observed_control_source==2;
  result.general=robot.mode==1&&robot.motion==5&&robot.speed==1&&robot.head==1;
  result.mc_fresh=robot.mc_fresh&&mc_source_fresh;
  result.estop=robot.software!=1||robot.hardware!=1;
  result.robot_state_fresh=NavigationMotion{}.robot_fresh(robot,now)&&owner.fresh(now);
  return result;
}

// Select the configured stationary metric using the same explicit runtime
// profile contract as actuation. Fixed STAIR can reuse GENERAL parameters,
// but merely seeing mode=STAIR is not such a contract. Fresh RobotState must
// still confirm the selected standing/low-speed/head state; never combine
// unknown, transitioning, or differently calibrated modes into this window.
// This does not authorize motion or declare physical acceptance. The metric
// retains both the XY threshold and an independent body-Z threshold.
inline bool stationaryRuntimeProfileApplies(const Health& health,bool fixed_stair_general_profile) {
  if(!health.connected||!health.robot_state_fresh||health.mode_transition_active)return false;
  if(health.general)return health.required_motion_mode.empty()||health.required_motion_mode=="general";
  return fixed_stair_general_profile&&health.required_motion_mode=="stair"&&
    health.motion_mode=="stair"&&health.motion_mode_confirmed&&health.mode_runtime_parameters_ready;
}
}
