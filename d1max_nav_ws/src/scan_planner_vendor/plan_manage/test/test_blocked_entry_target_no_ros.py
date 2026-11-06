#!/usr/bin/env python3
"""Run the real getLocalTarget method and production GridMap without ROS.

Only message/clock delivery and snapshot acquisition are passive seams. The
complete production method, selectors, map queries and braking check are
compiled, not reimplemented. The in-memory geometry fixture is not a bag or
physical calibration. Source swapping supports identical-input regression.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/visible_reference_target.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/input_contract.hpp>
#include <plan_manage/collision_reference.hpp>
#include <memory>
#include <iostream>
#include <cstdint>
#include <optional>
#undef RCLCPP_INFO
#undef RCLCPP_ERROR
#undef RCLCPP_WARN_THROTTLE
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
struct ClockValue {
  double value=100.1;
  double seconds()const{return value;}
  std::int64_t nanoseconds()const{return std::llround(value*1e9);}
};
struct PassiveNode {ClockValue now()const{return {};}};
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto m=std::make_shared<GridMap>();auto& p=m->mp_;auto& d=m->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={120,100,60};
    p.map_origin_idx_={20,0,10};m->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.planning_collision_policy_="official";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;p.frame_id_="odom";
    const auto n=120*100*60;
    d.occupancy_buffer_.assign(n,-1.);
    d.occupancy_buffer_inflate_.assign(n,0);d.occupancy_buffer_inflate_cnt_.assign(n,0);
    d.planning_occupancy_buffer_inflate_.assign(n,0);d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);
    m->rebuildInflationOffsets();
    p.use_projected_rays_=true;p.cloud_pose_max_age_=.5;
    m->free_observation_stamps_.assign(n,100000000000LL);
    m->ray_integrated_stamps_={100000000000LL,100000000000LL};
    m->integrated_cloud_stamp_ns_=100000000000LL;m->snapshot_clock_ns_=100100000000LL;
    m->snapshot_captured_=std::chrono::steady_clock::now();m->collision_snapshot_=true;
    m->enforce_free_freshness_=true;m->ray_clock_fault_=false;return m;
  }
  static void walls(GridMap& m) {
    for(int x=-10;x<=70;++x)for(double y:{-.5,.5})m.setOccupied({x*.05,y,.55});
  }
  static void person(GridMap& m,bool present) {
    m.setOccupancy({.7,0.,.55},present?1.:0.);
  }
  static void unknown(GridMap& m) {
    for(auto& odds:m.md_.occupancy_buffer_)if(odds<=m.mp_.min_occupancy_log_)odds=-1.01;
  }
};
namespace scan_planner {
struct PassiveManager {
  GridMap::Ptr grid_map_=GridMapTestAccess::make();
  struct {double max_vel_=.3,max_acc_=.35;} pp_;
  struct {
    double global_duration_=4.,last_progress_time_=0.;
    Eigen::Vector3d getPosition(double t)const{return {t,0.,.55};}
    Eigen::Vector3d getVelocity(double)const{return {.3,0.,0.};}
  } global_data_;
  std::vector<Eigen::Vector3d> selected;
  void setLocalReference(const std::vector<Eigen::Vector3d>& p){selected=p;}
};
struct PassiveSupportOwner {
  std::shared_ptr<const MotionSupport> support;
  std::shared_ptr<const MotionSupport> planningSupport(int)const{return support;}
};
struct PassiveSnapshotPool {
  GridMap::Ptr snapshot;bool unavailable=false;
  GridMap::Ptr acquireSolver(GridMap&,std::int64_t,SolveBudget::Time) {
    return unavailable?GridMap::Ptr{}:snapshot;
  }
};
inline int executionVersion(int metadata){return metadata;}
class SCANReplanFSM {
public:
  bool reference_path_guidance_=true,force_visible_side_target_=false;
  DiscreteReference discrete_reference_;
  double discrete_progress_=0.,planning_horizon_=2.;
  double reference_target_forward_margin_=1.,reference_target_backward_margin_=2.;
  double reference_target_min_advance_=.3,reference_target_exit_margin_=.2;
  double self_double_cylinder_offset_=.2,odom_timeout_=.4;
  std::string self_inflation_frame_id_="odom",last_attempt_failure_phase_;
  Eigen::Vector3d start_pt_{0.,0.,.55},odom_pos_{0.,0.,.55},end_pt_{3.2,0.,.55};
  Eigen::Vector3d local_target_pt_{},local_target_vel_{},local_debug_projection_{};
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  ClockValue last_odom_time_;
  double local_debug_target_arc_=0.;
  ReferenceTargetResult local_target_query_debug_;
  std::vector<Eigen::Vector3d> local_debug_selected_reference_;
  int reference_metadata_=1;
  PassiveNode clock;PassiveNode* node_=&clock;
  std::shared_ptr<PassiveManager> planner_manager_=std::make_shared<PassiveManager>();
  std::shared_ptr<PassiveSupportOwner> execution_validator_=std::make_shared<PassiveSupportOwner>();
  PassiveSnapshotPool snapshot_pool_;
  SCANReplanFSM() {
    discrete_reference_.set({start_pt_,end_pt_});
    d1max_planning_interfaces::msg::SupportReference s;
    s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";s.frame_id="odom";
    s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
    s.support_xy_radius_m=.05;s.body_reference_height_m=.55;
    s.max_support_slope_rad=.15;s.max_support_step_m=.05;
    for(int x=-20;x<=70;++x)for(int y=-40;y<=40;++y) {
      geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;s.support_ground_xyz.push_back(p);
    }
    execution_validator_->support=std::make_shared<const MotionSupport>(s);
    snapshot_pool_.snapshot=planner_manager_->grid_map_;
  }
  bool getLocalTarget(const SolveBudget::Ptr&,GridMap::Ptr*);
  double estimateYawFromSegment(const Eigen::Vector3d&,const Eigen::Vector3d&)const;
  double getOdomYaw()const;
};
__METHOD__
}
int main() {
  using scan_planner::SCANReplanFSM;
  using scan_planner::SolveBudget;
  int failures=0,checks=0;
  const auto check=[&](bool passed,const char* name) {
    ++checks;if(!passed)++failures;
    std::cout<<(passed?"PASS ":"FAIL ")<<name<<"\n";
  };
  const auto solve=[](auto& f) {return f.getLocalTarget(std::make_shared<SolveBudget>(),nullptr);};
  {
    SCANReplanFSM f;GridMapTestAccess::walls(*f.planner_manager_->grid_map_);
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_nominal",
      "narrow_corridor_normal_target_is_free");
    const auto body=f.odom_pos_;const auto endpoint=f.discrete_reference_.points.back();
    GridMapTestAccess::person(*f.planner_manager_->grid_map_,true);
    f.force_visible_side_target_=true;
    check(!solve(f),"occupied_entry_side_attempt_has_no_narrow_corridor_alternative");
    check(!f.force_visible_side_target_,"actual_failed_side_attempt_consumes_hint");
    GridMapTestAccess::person(*f.planner_manager_->grid_map_,false);
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_nominal",
      "obstacle_removal_recovers_same_route_without_body_motion_or_new_reference");
    check(f.odom_pos_.isApprox(body)&&f.discrete_reference_.points.back().isApprox(endpoint)&&f.reference_metadata_==1,
      "recovery_preserves_measured_body_global_route_and_reference_identity");
  }
  {
    SCANReplanFSM f;f.force_visible_side_target_=true;f.snapshot_pool_.unavailable=true;
    check(!solve(f)&&f.force_visible_side_target_&&f.local_target_query_debug_.reason=="waiting_snapshot_slot",
      "snapshot_resource_wait_does_not_consume_side_request");
    f.snapshot_pool_.unavailable=false;
    check(solve(f)&&!f.force_visible_side_target_&&f.local_target_query_debug_.reason=="reference_target_visible_side",
      "successful_actual_side_search_consumes_hint");
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_nominal",
      "consumed_hint_cannot_permanently_forbid_normal_reference");
  }
  {
    SCANReplanFSM f;f.force_visible_side_target_=true;
    auto budget=std::make_shared<SolveBudget>();budget->cancel();
    check(!f.getLocalTarget(budget,nullptr)&&f.force_visible_side_target_,
      "cancel_before_search_does_not_claim_completed_side_attempt");
  }
  {
    SCANReplanFSM f;GridMapTestAccess::walls(*f.planner_manager_->grid_map_);
    GridMapTestAccess::person(*f.planner_manager_->grid_map_,true);f.force_visible_side_target_=true;
    check(!solve(f),"persistent_occupied_entry_side_attempt_cannot_make_free_certificate");
    // Endpoint discovery is not command permission. An actual native braking
    // sweep remains rejected after this planning hint has been consumed.
    const scan_planner::BrakingModel braking{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
    const auto proof=scan_planner::validateStraightBrakingCorridor(*f.planner_manager_->grid_map_,
      *f.execution_validator_->support,braking,f.start_pt_,{2.,0.,.55},"odom",.1);
    std::cout<<"occupied_counterexample_reason="<<proof.reason<<"\n";
    check(!proof.valid&&proof.reason=="motion_sweep_occupied",
      "persistent_real_obstacle_still_denies_native_motion_after_hint_consumption");
  }
  {
    SCANReplanFSM f;GridMapTestAccess::unknown(*f.planner_manager_->grid_map_);
    check(solve(f),"official_geometry_may_generate_candidate_in_unknown_space");
    const scan_planner::BrakingModel braking{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
    const auto proof=scan_planner::validateStraightBrakingCorridor(*f.planner_manager_->grid_map_,
      *f.execution_validator_->support,braking,f.start_pt_,{2.,0.,.55},"odom",.1);
    std::cout<<"unknown_counterexample_reason="<<proof.reason<<"\n";
    check(!proof.valid&&proof.reason=="motion_sweep_unknown_or_expired",
      "pure_unknown_motion_is_not_authorized_by_official_candidate_policy");
  }
  std::cout<<"SUMMARY checks="<<checks<<" failures="<<failures<<"\n";
  return failures?1:0;
}
'''


def method(source, signature):
    text = source.read_text()
    start = text.index(signature)
    opening = text.index('{', start)
    depth = 1
    i = opening + 1
    while depth:
        depth += (text[i] == '{') - (text[i] == '}')
        i += 1
    return text[start:i]


def main():
    parser = argparse.ArgumentParser()
    package = Path(__file__).resolve().parents[1]
    parser.add_argument('--source', type=Path, default=package/'src/scan_replan_fsm.cpp')
    parser.add_argument('--native-build', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    directory = args.build_dir or Path(tempfile.mkdtemp(prefix='scan-blocked-entry-'))
    directory.mkdir(parents=True, exist_ok=True)
    # Generated test artefacts only; production sources are not rewritten.
    cpp = directory/'blocked_entry_target.cpp'
    methods = '\n'.join(method(args.source, signature) for signature in
        ('  bool SCANReplanFSM::getLocalTarget(', '  double SCANReplanFSM::estimateYawFromSegment(',
         '  double SCANReplanFSM::getOdomYaw('))
    cpp.write_text(HARNESS.replace('__METHOD__', methods))
    metadata = args.native_build/'CMakeFiles/scan_planner_node.dir'
    variables = dict(re.findall(r'^(CXX_\w+) = (.*)$', (metadata/'flags.make').read_text(), re.M))
    include = args.source.resolve().parents[1]/'include'
    flags = ['-std=c++17', '-I'+str(include), '-I'+str(package.parent/'plan_env/include')]
    for name in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        flags += shlex.split(variables[name])
    # The same native library/message build is used for before/after. Swap the
    # exact FSM function and its package headers only, not a synthetic predicate.
    obj = directory/'blocked_entry_target.o'
    subprocess.run(['/usr/bin/c++', *flags, '-c', str(cpp), '-o', str(obj)], check=True)
    sweep = directory/'motion_sweep.o'
    subprocess.run(['/usr/bin/c++', *flags, '-c', str(args.source.resolve().with_name('motion_sweep.cpp')),
                    '-o', str(sweep)], check=True)
    original = shlex.split((metadata/'link.txt').read_text())
    libraries = original[original.index('-o')+2:]
    executable = directory/'blocked_entry_target'
    subprocess.run([original[0], str(obj), str(sweep), '-o', str(executable), *libraries], check=True)
    result = subprocess.run([str(executable)], text=True, capture_output=True)
    (directory/'result.log').write_text(result.stdout+result.stderr)
    print(result.stdout+result.stderr, end='')
    record = dict(source=str(args.source.resolve()), source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
                  executable=str(executable), result_code=result.returncode,
                  scope='production getLocalTarget, selectors, GridMap and motion sweep; passive delivery/acquisition; no ROS initialization')
    (directory/'result.json').write_text(json.dumps(record, indent=2)+'\n')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
