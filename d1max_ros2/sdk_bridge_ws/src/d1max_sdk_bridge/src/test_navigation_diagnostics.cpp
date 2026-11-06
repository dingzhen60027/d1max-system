#include "navigation_diagnostics_core.hpp"
#include <cassert>
#include <iostream>
using namespace d1monitor;
int main() {
  NavigationMotion legacy;
  ExecutionNavigationSnapshot execution;
  execution.enabled=execution.parameters_valid=true;
  execution.kinematic_profile="holonomic_xyyaw";
  execution.parameter_basis="vendor_nominal_and_engineering";
  execution.profile_sha256=std::string(64,'a');
  auto status=navigationDiagnostics(legacy,"disabled",execution,false);
  assert(!status.at("navigation_armed").get<bool>());
  assert(status.at("navigation_block_reason")=="waiting_execution_grant"&&status.at("navigation_error")=="");
  assert(status.at("navigation_measured_planar_mps").is_null());
  assert(!status.at("navigation_measured_planar_fresh").get<bool>());
  assert(status.at("navigation_overspeed_latched").is_null());
  assert(status.at("navigation_limits").at("lateral_mps")==.3);
  assert(!status.at("navigation_limits").contains("hard_planar_mps"));

  execution.execution_id="execution";execution.phase="tracking";execution.reason="ok";
  execution.grant_ready=true;execution.arm_generation=7;
  legacy.fault=legacy.overspeed_latched=true; // disabled writer must not leak its latches
  legacy.measurement(.12,.16,10.);
  status=navigationDiagnostics(legacy,"fault_latched",execution,true);
  assert(status.at("navigation_armed").get<bool>()&&status.at("navigation_arm_generation")==7);
  assert(status.at("navigation_block_reason")==""&&status.at("navigation_error")=="");
  assert(!status.at("navigation_fault_latched").get<bool>());
  assert(std::abs(status.at("navigation_measured_planar_mps").get<double>()-.2)<1e-12);
  assert(status.at("navigation_measured_planar_source")=="sdk_mc");
  assert(status.at("navigation_measured_planar_fresh").get<bool>());
  assert(status.at("navigation_overspeed_latched").is_null());
  assert(!navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_measured_planar_fresh").get<bool>());

  execution.reason="mc_stale";execution.grant_ready=false;execution.phase="holding";
  status=navigationDiagnostics(legacy,"disabled",execution,false);
  assert(status.at("navigation_block_reason")=="mc_stale"&&status.at("navigation_error")=="");
  execution.fault_latched=true;execution.reason="control_lost_no_auto_retake";
  status=navigationDiagnostics(legacy,"disabled",execution,false);
  assert(status.at("navigation_fault_latched").get<bool>());
  assert(status.at("navigation_error")==execution.reason&&status.at("navigation_block_reason")==execution.reason);
  execution.reason="";
  assert(navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_error")=="execution_fault_latched");
  execution.parameters_valid=false;execution.parameter_error="motion_profile_identity_mismatch";
  assert(navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_error")==execution.parameter_error);

  execution.parameters_valid=true;execution.fault_latched=false;execution.reason="";
  execution.max_planar_mps=.22;execution.max_yaw_radps=.44;
  auto limits=navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_limits");
  assert(limits.at("forward_mps")==.22&&limits.at("lateral_mps")==.22&&limits.at("yaw_radps")==.44);
  assert(limits.at("kinematic_profile")=="holonomic_xyyaw"&&limits.at("runtime_profile_sha256")==execution.profile_sha256);
  execution.kinematic_profile="forward_yaw";
  assert(navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_limits").at("lateral_mps")==0.);
  execution.parameter_basis="physical_evidence";
  assert(navigationDiagnostics(legacy,"disabled",execution,false).at("navigation_limits").at("limit_source")=="physical_acceptance_record");

  execution.enabled=false;legacy.armed=true;legacy.generation=11;legacy.max_y=.12;
  legacy.error="invalid_or_excessive_velocity";
  status=navigationDiagnostics(legacy,"fault_latched",execution,true);
  assert(status.at("navigation_armed").get<bool>()&&status.at("navigation_arm_generation")==11);
  assert(status.at("navigation_fault_latched").get<bool>()&&status.at("navigation_overspeed_latched").get<bool>());
  assert(status.at("navigation_error")==legacy.error&&status.at("navigation_block_reason")=="fault_latched");
  assert(status.at("navigation_limits").at("lateral_mps")==.12);
  assert(status.at("navigation_limits").at("hard_planar_mps")==NavigationMotion::hard_planar_mps);
  assert(status.at("navigation_limits").at("limit_source")=="legacy_navigation_adapter");
  std::cout<<"PASS passive navigation diagnostics: active state/profile, real MC source, no legacy latch leakage\n";
}
