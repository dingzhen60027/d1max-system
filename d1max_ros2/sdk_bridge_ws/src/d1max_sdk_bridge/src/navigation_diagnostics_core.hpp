#pragma once
#include <nlohmann/json.hpp>
#include "navigation_motion_core.hpp"

namespace d1monitor {
// Passive snapshot only. This projection cannot authorize or submit motion.
struct ExecutionNavigationSnapshot {
  bool enabled=false,parameters_valid=false,grant_ready=false,fault_latched=false;
  uint64_t arm_generation=0;
  std::string execution_id,phase,reason,parameter_error,parameter_basis,kinematic_profile="forward_yaw",profile_sha256;
  double max_planar_mps=.3,max_yaw_radps=.5;
};

inline nlohmann::json navigationDiagnostics(const NavigationMotion&legacy,const std::string&legacy_reason,
    const ExecutionNavigationSnapshot&execution,bool mc_fresh) {
  const bool active=execution.enabled;
  std::string reason=legacy_reason,error=legacy.error;
  if(active) {
    reason=execution.reason=="ok"?std::string():execution.reason;
    error.clear();
    if(!execution.parameters_valid) {
      reason=execution.parameter_error.empty()?"runtime_motion_profile_unavailable":execution.parameter_error;
      error=reason;
    } else if(execution.fault_latched) {
      if(reason.empty())reason="execution_fault_latched";
      error=reason;
    } else if(execution.execution_id.empty())reason="waiting_execution_grant";
  }
  const nlohmann::json limits=active?nlohmann::json{
    {"limit_source",execution.parameter_basis=="physical_evidence"?"physical_acceptance_record":"runtime_motion_profile"},
    {"kinematic_profile",execution.kinematic_profile},{"runtime_profile_sha256",execution.profile_sha256},
    {"planar_mps",execution.max_planar_mps},{"forward_mps",execution.max_planar_mps},
    {"lateral_mps",execution.kinematic_profile=="holonomic_xyyaw"?execution.max_planar_mps:0.},
    {"yaw_radps",execution.max_yaw_radps},{"required_speed_level",1}}:nlohmann::json{
    {"limit_source","legacy_navigation_adapter"},{"hard_planar_mps",NavigationMotion::hard_planar_mps},
    {"forward_mps",legacy.max_x},{"lateral_mps",legacy.max_y},{"yaw_radps",legacy.max_yaw},{"required_speed_level",1}};
  // measurement() records accepted OnMcData even when the old adapter is
  // disabled. Label that real observation separately from its legacy-only
  // overspeed latch; an inactive latch is not a v3 protection verdict.
  const bool measured=legacy.measurement_at>=0.&&std::isfinite(legacy.measured_planar_mps)&&legacy.measured_planar_mps>=0.;
  return {{"navigation_armed",active?execution.grant_ready:legacy.armed},
    {"navigation_arm_generation",active?execution.arm_generation:legacy.generation},
    {"navigation_block_reason",reason},{"navigation_error",error},
    {"navigation_fault_latched",active?execution.fault_latched:legacy.fault},
    {"navigation_limits",limits},
    {"navigation_measured_planar_mps",measured?nlohmann::json(legacy.measured_planar_mps):nlohmann::json(nullptr)},
    {"navigation_measured_planar_source","sdk_mc"},{"navigation_measured_planar_fresh",measured&&mc_fresh},
    {"navigation_overspeed_latched",active?nlohmann::json(nullptr):nlohmann::json(legacy.overspeed_latched)},
    {"navigation_overspeed_guard_source",active?"not_applicable":"legacy_navigation_adapter"}};
}
}
