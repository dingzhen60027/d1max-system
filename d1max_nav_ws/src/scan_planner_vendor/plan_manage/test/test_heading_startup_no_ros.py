#!/usr/bin/env python3
"""Compile the production heading initialization with passive parameter storage.

No ROS init, map replacement, SDK or graph. Collision geometry is exercised by
the separate native GridMap tests; this checks the startup declaration order.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <bspline_opt/spline_heading_contract.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <plan_env/voxel_collision.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <iostream>
struct Value {
  std::variant<bool,double,std::string> value;
  template<class T>T get()const{return std::get<T>(value);}
  bool as_bool()const{return get<bool>();}
  double as_double()const{return get<double>();}
  std::string as_string()const{return get<std::string>();}
};
struct Parameters {
  std::map<std::string,Value> overrides;
  const auto& get_parameter_overrides()const{return overrides;}
};
struct Node {
  std::map<std::string,Value> declared;
  Parameters startup;
  Node() {declared={{"grid_map.preview_only",{false}},
      {"grid_map.use_projected_rays",{true}},
      {"grid_map.double_cylinder_radius",{.29}},
      {"grid_map.double_cylinder_offset",{.2}}};}
  bool has_parameter(const std::string&n)const{return declared.count(n);}
  const Value& get_parameter(const std::string&n)const{return declared.at(n);}
  Parameters* get_node_parameters_interface(){return &startup;}
  template<class T>void declare_parameter(const std::string&n,T fallback) {
    auto it=startup.overrides.find(n);
    declared.emplace(n,it==startup.overrides.end()?Value{fallback}:it->second);
  }
};
struct Map {
  bool strict=false;
  scan_planner::BodyEnvelope shape{.29,.2,.45,.4};
  bool planningRequiresObservedFree()const{return strict;}
  scan_planner::BodyEnvelope planningBodyEnvelope()const{return shape;}
};
scan_planner::SplineHeadingContract initialize(Node*node_,Map*grid_map_,
    bool*planning_only=nullptr,
    scan_planner::PreviewCandidateAcceptancePolicy*policy=nullptr) {
  Node*node=node_;
  scan_planner::SplineHeadingContract preview_heading_contract_;
  bool planning_only_geometry_=false,rolling_candidate_connection_=false;
  scan_planner::PreviewCandidateAcceptancePolicy preview_candidate_policy_;
  const auto get_double=[&](const std::string&name,double fallback) {
    if(!node_->has_parameter(name))node_->declare_parameter<double>(name,fallback);
    return node_->get_parameter(name).as_double();
  };
  using namespace scan_planner;
__PRODUCTION__
  if(planning_only)*planning_only=planning_only_geometry_;
  if(policy)*policy=preview_candidate_policy_;
  return preview_heading_contract_;
}
int main() {
  int checks=0,failures=0;
  auto check=[&](bool value,const std::string&name){
    ++checks;
    std::cout<<(value?"PASS ":"FAIL ")<<name<<"\n";failures+=!value;};
  Map map;
  Node formal;formal.startup.overrides={{"fsm.execution_protocol",{true}},
      {"fsm.writer_handoff_enabled",{false}}};
  const auto preview=initialize(&formal,&map);
  check(preview.preview_only_enabled&&preview.measured_heading_enabled,"undeclared_formal_preview_overrides_enable_measured_yaw");
  check(preview.policy==scan_planner::BodyHeadingPolicy::PathTangent,"default_heading_policy_remains_path_tangent");
  Node held;held.startup.overrides=formal.startup.overrides;
  held.startup.overrides["manager.body_heading_policy"]={std::string("hold_heading")};
  check(initialize(&held,&map).policy==scan_planner::BodyHeadingPolicy::HoldHeading,
      "explicit_hold_heading_uses_measured_pose_contract");
  Node invalid_heading;invalid_heading.startup.overrides=formal.startup.overrides;
  invalid_heading.startup.overrides["manager.body_heading_policy"]={std::string("velocity_is_yaw")};
  bool invalid_heading_rejected=false;
  try{initialize(&invalid_heading,&map);}catch(const std::invalid_argument&){invalid_heading_rejected=true;}
  check(invalid_heading_rejected,"unknown_heading_policy_rejected");
  Node no_pose_hold;no_pose_hold.startup.overrides["manager.body_heading_policy"]={std::string("hold_heading")};
  bool no_pose_rejected=false;
  try{initialize(&no_pose_hold,&map);}catch(const std::invalid_argument&){no_pose_rejected=true;}
  check(no_pose_rejected,"hold_heading_cannot_invent_unmeasured_yaw");
  check(std::abs(preview.body_extent-.49)<1e-12,"legacy_rotation_extent_comes_from_native_shape");
  map.shape={0.,0.,.25,.25,scan_planner::CollisionModel::OrientedBox,.94,.50};
  Node box;box.startup.overrides=formal.startup.overrides;
  const auto box_contract=initialize(&box,&map);
  check(std::abs(box_contract.body_extent-std::hypot(.47,.25))<1e-12,
      "obb_rotation_extent_covers_actual_rectangle_corner");
  map.shape={.29,.2,.45,.4};
  check(!formal.has_parameter("fsm.execution_protocol") &&
      !formal.has_parameter("fsm.writer_handoff_enabled"),"reader_does_not_change_fsm_declaration_order");
  Node motion;motion.startup.overrides={{"fsm.execution_protocol",{true}},
      {"fsm.writer_handoff_enabled",{true}}};
  const auto writer=initialize(&motion,&map);
  check(!writer.preview_only_enabled&&writer.measured_heading_enabled,"writer_uses_same_geometry_without_preview_authority");
  Node missing;missing.startup.overrides={{"fsm.execution_protocol",{true}}};
  const auto absent=initialize(&missing,&map);
  check(!absent.preview_only_enabled&&absent.measured_heading_enabled,"missing_writer_override_not_assumed_preview");
  Node declared;declared.startup.overrides=formal.startup.overrides;
  declared.declared["fsm.execution_protocol"]={true};
  declared.declared["fsm.writer_handoff_enabled"]={true};
  const auto effective=initialize(&declared,&map);
  check(!effective.preview_only_enabled&&effective.measured_heading_enabled,"declared_effective_writer_takes_precedence");
  Node legacy;legacy.declared["grid_map.preview_only"]={true};
  legacy.startup.overrides["manager.preview_body_heading_contract"]={true};
  const auto legacy_contract=initialize(&legacy,&map);
  check(legacy_contract.preview_only_enabled&&legacy_contract.measured_heading_enabled,"legacy_explicit_map_preview_still_supported_without_fsm_flags");
  Node strict;strict.startup.overrides=formal.startup.overrides;map.strict=true;
  const auto strict_contract=initialize(&strict,&map);
  check(strict_contract.preview_only_enabled&&strict_contract.measured_heading_enabled,"strict_query_policy_is_not_changed_to_enable_heading");
  Node unmanaged;
  const auto no_pose_contract=initialize(&unmanaged,&map);
  check(!no_pose_contract.preview_only_enabled&&!no_pose_contract.measured_heading_enabled,"unmanaged_legacy_geometry_is_not_silently_enabled");
  bool planning_only=true;
  scan_planner::PreviewCandidateAcceptancePolicy policy;
  Node default_geometry;
  initialize(&default_geometry,&map,&planning_only,&policy);
  check(!planning_only&&policy.valid()&&
      policy.connector_m==scan_planner::NoMotionPreviewConnectionPolicy::max_connector_m&&
      policy.projection_window_m==.5,"planning_only_default_false_and_typed_geometry_policy_defaults");
  Node geometry;geometry.startup.overrides=formal.startup.overrides;
  geometry.startup.overrides["fsm.planning_only"]={true};
  const auto geometry_heading=initialize(&geometry,&map,&planning_only,&policy);
  check(planning_only&&geometry_heading.preview_only_enabled&&geometry_heading.measured_heading_enabled,
      "planning_only_startup_override_preserves_measured_geometry_contract");
  check(!geometry.has_parameter("fsm.planning_only"),"planning_only_reader_preserves_fsm_declaration_order");
  Node effective_geometry;effective_geometry.startup.overrides=geometry.startup.overrides;
  effective_geometry.declared["fsm.planning_only"]={false};
  initialize(&effective_geometry,&map,&planning_only);
  check(!planning_only,"declared_planning_only_value_takes_precedence");
  const auto rejected=[&](Node&n,const std::string&reason){
    try{initialize(&n,&map);return false;}
    catch(const std::invalid_argument&error){return error.what()==reason;}
  };
  const std::string writer_guard="planning-only candidate geometry cannot own writer handoff";
  Node writer_geometry;writer_geometry.startup.overrides=geometry.startup.overrides;
  writer_geometry.startup.overrides["fsm.writer_handoff_enabled"]={true};
  check(rejected(writer_geometry,writer_guard),"planning_only_rejects_writer_handoff");
  Node implicit_writer;implicit_writer.startup.overrides["fsm.planning_only"]={true};
  check(rejected(implicit_writer,writer_guard),"planning_only_requires_explicit_no_writer_override");
  Node declared_writer;declared_writer.startup.overrides=geometry.startup.overrides;
  declared_writer.declared["fsm.writer_handoff_enabled"]={true};
  check(rejected(declared_writer,writer_guard),"planning_only_effective_writer_guard_precedes_startup_override");
  const std::string invalid_policy="invalid preview candidate geometry policy";
  const std::pair<const char*,double> invalid_connectors[]={
      {"zero",0.},{"negative",-.01},{"over_typed_limit",.150001},
      {"infinite",std::numeric_limits<double>::infinity()},
      {"nan",std::numeric_limits<double>::quiet_NaN()}};
  for(const auto&item:invalid_connectors){
    Node invalid;invalid.startup.overrides=geometry.startup.overrides;
    invalid.startup.overrides["manager.preview_candidate_connector_m"]={item.second};
    check(rejected(invalid,invalid_policy),std::string("planning_only_rejects_connector_")+item.first);
  }
  Node bounded;bounded.startup.overrides=geometry.startup.overrides;
  bounded.startup.overrides["manager.preview_candidate_connector_m"]={.1};
  bounded.startup.overrides["manager.preview_candidate_projection_window_m"]={1.};
  initialize(&bounded,&map,&planning_only,&policy);
  check(planning_only&&policy.valid()&&policy.connector_m==.1&&policy.projection_window_m==1.,
      "planning_only_accepts_narrower_connector_and_maximum_arc_window");
  const std::pair<const char*,double> invalid_windows[]={
      {"zero",0.},{"negative",-.01},{"over_typed_limit",1.000001},
      {"nan",std::numeric_limits<double>::quiet_NaN()}};
  for(const auto&item:invalid_windows){
    Node invalid;invalid.startup.overrides=geometry.startup.overrides;
    invalid.startup.overrides["manager.preview_candidate_projection_window_m"]={item.second};
    check(rejected(invalid,invalid_policy),std::string("planning_only_rejects_projection_window_")+item.first);
  }
  Node no_yaw_configuration;
  initialize(&no_yaw_configuration,&map);
  check(!no_yaw_configuration.has_parameter("manager.max_yaw_rate")&&
      !no_yaw_configuration.has_parameter("manager.tangent_rate_diagnostic_limit"),
      "production_planner_declares_no_tangent_or_physical_yaw_limit");
__TIMING_CHECKS__
  std::cout<<"CHECKS "<<checks<<" FAILURES "<<failures<<"\n";
  return failures?1:0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('    if (!node_->has_parameter("manager.preview_body_heading_contract"))')
    end = source.index('    bspline_optimizer_rebound_.reset(new BsplineOptimizer);', start)
    include = args.source.parents[2] / 'bspline_opt/include'
    environment_include = args.source.parents[2] / 'plan_env/include'
    manager_include = args.source.parents[1] / 'include'
    timing_start = source.index('/*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/')
    final_check = source.index('    if (!flag_step_2_success || !checkDynamicFeasibility(pos))', timing_start)
    timing = source[timing_start:final_check]
    moving_start = timing.index('    if (guided && !stationary_guided)')
    moving_end = timing.index('    else if (!stationary_guided', moving_start)
    moving = timing[moving_start:moving_end]
    stationary_start = timing.index('    if (stationary_guided && flag_step_2_success)')
    stationary = timing[stationary_start:]
    wiring_checks = {
        'production_moving_timing_has_no_tangent_admission_constraint':
            'refine_interior,8,std::nullopt,' in moving and 'turning_rate_limits_' not in moving,
        'production_moving_refine_keeps_original_shared_solve_budget':
            'return solveAllowed(solve_budget_);' in moving,
        'production_stationary_timing_preserves_xyz_with_velocity_acceleration_only':
            'enforceDerivativeBounds(pos,pp_.max_vel_,pp_.max_acc_)' in stationary,
        'production_stationary_timing_checks_original_budget_before_and_after':
            stationary.count('if (!solveAllowed(solve_budget_)) return false;') >= 2,
        'production_neither_guided_lane_calls_turning_time_allocator':
            'retimeStationaryWithTurningLimits' not in timing and
            'turning_rate_limits_' not in timing,
        'production_exact_tangent_analysis_does_not_consume_core_deadline':
            'boundTurningRate(' not in source and 'turningRateDomain(' not in source,
        'production_no_new_tangent_or_physical_yaw_configuration':
            'manager.max_yaw_rate' not in source and
            'manager.tangent_rate_diagnostic_limit' not in source,
        'production_original_deadline_still_gates_before_final_curve_checks':
            timing.rstrip().endswith('if (!solveAllowed(solve_budget_)) return false;'),
        'production_actual_final_obb_collision_validation_still_required':
            'checkWholeTrajectoryCollision(pos,200000,0.,true,0.,attempt_heading_contract_' in
            source[final_check:source.index('    t_refine =', final_check)],
    }
    wiring_cpp = '\n'.join(
        f'  check({str(passed).lower()},"{name}");' for name, passed in wiring_checks.items())
    with tempfile.TemporaryDirectory(prefix='scan-heading-startup-') as temporary:
        directory = args.build_dir or Path(temporary)
        directory.mkdir(parents=True, exist_ok=True)
        cpp, binary = directory / 'startup.cpp', directory / 'startup'
        cpp.write_text(HARNESS.replace('__PRODUCTION__', source[start:end])
                       .replace('__TIMING_CHECKS__', wiring_cpp))
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra',
                        '-I' + str(include), '-I' + str(environment_include),
                        '-I' + str(manager_include), '-I/usr/include/eigen3',
                        str(cpp), '-o', str(binary)], check=True)
        return subprocess.run([str(binary)], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
