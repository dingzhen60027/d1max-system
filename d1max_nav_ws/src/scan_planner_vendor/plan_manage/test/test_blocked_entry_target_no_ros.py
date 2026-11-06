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
#include <plan_manage/forward_motion_boundary.hpp>
#include <plan_manage/execution_validator.hpp>
#include <d1max_planning_interfaces/msg/reference_path.hpp>
#include <future>
#include <memory>
#include <iostream>
#include <cstdint>
#include <optional>
#undef RCLCPP_INFO
#undef RCLCPP_ERROR
#undef RCLCPP_WARN_THROTTLE
#undef RCLCPP_WARN
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
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
    p.require_observed_free_=false;p.planning_collision_policy_="official";
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
  static void strictComparison(GridMap& m) {
    m.mp_.require_observed_free_=true;m.mp_.planning_collision_policy_="observed_free";
  }
  static void turningLanding(GridMap& m) {
    auto& p=m.mp_;
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    m.rebuildInflationOffsets();
    for(int y=4;y<49;++y)for(double x:{-.325,.325})
      m.setOccupied({x,(y+.5)*.05,.5});
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
  bool selected_has_projection_connector=false;
  std::shared_ptr<const PlanningSupport> selected_support;
  BodyHeadingPolicy configuredHeadingPolicy()const{return BodyHeadingPolicy::PathTangent;}
  void setLocalReference(const std::vector<Eigen::Vector3d>& p,bool=false,bool has_connector=false,
                         std::shared_ptr<const PlanningSupport> support={}){
    selected=p;selected_has_projection_connector=has_connector;
    selected_support=std::move(support);
  }
};
struct PassiveSupportOwner {
  std::shared_ptr<const PlanningSupport> support;
  std::optional<ew::MotionValidation> blocked;
  std::optional<ew::TrajectoryValidation> collision;
  int candidate_cancels=0;
  std::shared_ptr<const PlanningSupport> planningSupport(const ew::ExecutionVersion&)const{return support;}
  std::optional<Eigen::Vector3d> committedTaskGoal(const ew::ExecutionVersion&)const{return {};}
  bool finalEndpointConnectionNeedsReseed(const ew::ExecutionVersion&,std::int64_t)const{return false;}
  auto consumeBlockedEntry(){auto x=blocked;blocked.reset();return x;}
  auto consumeCommittedCollisionAt(std::int64_t){auto x=collision;collision.reset();return x;}
  void cancelPending(){++candidate_cancels;}
};
struct PassiveSnapshotPool {
  GridMap::Ptr snapshot;bool unavailable=false;
  GridMap::Ptr acquireSolver(GridMap&,std::int64_t,SolveBudget::Time) {
    return unavailable?GridMap::Ptr{}:snapshot;
  }
};
inline ew::ExecutionVersion executionVersion(int metadata){
  ew::ExecutionVersion v;v.schema_version=3;v.session_id="session";v.task_id="task";
  v.route_id="route";v.route_hash="hash";v.map_version_id="map";
  v.localization_epoch=1;v.localization_seed_id="seed";v.reference_generation=metadata;
  return v;
}
inline ew::ExecutionVersion executionVersion(const ew::ReferencePath& metadata){
  return executionVersion(static_cast<int>(metadata.generation));
}
class SCANReplanFSM {
public:
  // This fixture exercises collision ingress only, no queued reference.
  void processDeferredRollingProposal() {}
  // Independent body-inbox freshness is exercised by PlanningBodyInbox and
  // test_replan_handoff_no_ros. This fixture supplies its already paired pose.
  bool refreshPlanningBodyFromInbox(){return false;}
  enum State {EXEC_TRAJ,GEN_NEW_TRAJ};
  State exec_state_=EXEC_TRAJ;
  unsigned transitions=0;
  bool reference_path_guidance_=true,force_visible_side_target_=false,local_follower_=false;
  bool require_tagged_reference_=true;
  bool have_target_=true,blocked_entry_replan_pending_=false;
  int worker_cancels=0;
  std::future<int> solve_future_;
  bool solveWorkPending()const{return solve_future_.valid();}
  struct {bool pending()const{return false;}} pending_snapshot_request_;
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> collision_replan_incumbent_;
  Eigen::Vector3d blocked_entry_position_{};
  DiscreteReference discrete_reference_;
  double discrete_progress_=0.,planning_horizon_=2.;
  double reference_target_forward_margin_=1.,reference_target_backward_margin_=2.;
  double reference_target_min_advance_=.3,reference_target_exit_margin_=.2;
  double reference_goal_xy_tolerance_=.20,reference_goal_z_tolerance_=.15;
  double self_double_cylinder_offset_=.2,odom_timeout_=.4;
  std::string self_inflation_frame_id_="odom",last_attempt_failure_phase_;
  Eigen::Vector3d start_pt_{0.,0.,.55},odom_pos_{0.,0.,.55},end_pt_{3.2,0.,.55};
  Eigen::Vector3d local_target_pt_{},local_target_vel_{},local_debug_projection_{};
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  ClockValue last_odom_time_;
  double local_debug_target_arc_=0.;
  ReferenceTargetResult local_target_query_debug_;
  std::vector<Eigen::Vector3d> local_debug_selected_reference_;
  ew::ReferencePath reference_metadata_;
  PassiveNode clock;PassiveNode* node_=&clock;
  std::shared_ptr<PassiveManager> planner_manager_=std::make_shared<PassiveManager>();
  std::shared_ptr<PassiveSupportOwner> execution_validator_=std::make_shared<PassiveSupportOwner>();
  PassiveSnapshotPool snapshot_pool_;
  SCANReplanFSM() {
    reference_metadata_.schema_version=2;reference_metadata_.generation=1;
    reference_metadata_.required_mode="general";
    discrete_reference_.set({start_pt_,end_pt_});
    d1max_planning_interfaces::msg::SupportReference s;
    s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";s.frame_id="odom";
    s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
    s.support_xy_radius_m=.05;s.body_reference_height_m=.55;
    s.max_support_slope_rad=.15;s.max_support_step_m=.05;
    for(int x=-20;x<=70;++x)for(int y=-40;y<=40;++y) {
      geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;s.support_ground_xyz.push_back(p);
    }
    execution_validator_->support=std::make_shared<const PlanningSupport>(s);
    snapshot_pool_.snapshot=planner_manager_->grid_map_;
  }
  bool getLocalTarget(const SolveBudget::Ptr&,GridMap::Ptr*);
  double estimateYawFromSegment(const Eigen::Vector3d&,const Eigen::Vector3d&)const;
  double getOdomYaw()const;
  void cancelSolveWorker(){++worker_cancels;}
  void changeFSMExecState(State state,const std::string&){exec_state_=state;++transitions;}
  void obstacleEventIngress();
};
__METHOD__
__OBSTACLE_INGRESS__
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
    SCANReplanFSM f;
    f.end_pt_=Eigen::Vector3d(1.,0.,.55);
    f.discrete_reference_.set({f.start_pt_,f.end_pt_});
    f.start_pt_=f.odom_pos_=f.end_pt_;
    f.discrete_progress_=f.discrete_reference_.length();
    f.force_visible_side_target_=true;
    check(!solve(f)&&f.last_attempt_failure_phase_=="waiting_reference_extension",
      "exhausted_reference_is_not_reported_as_collision_or_side_target_wait");
    check(f.local_target_query_debug_.queries==0&&f.force_visible_side_target_,
      "exhaustion_does_not_run_collision_side_search_or_consume_its_pending_hint");
    // Rolling installation does NOT clear force_visible_side_target_. Exercise
    // that real state transition, including planGlobalTrajByWaypoints' native
    // body-to-reference binding, rather than silently dropping the old hint.
    const auto task=scan_planner::executionVersion(f.reference_metadata_);
    const auto body=f.odom_pos_;
    f.reference_metadata_.generation=2;
    f.end_pt_=Eigen::Vector3d(3.2,0.,.55);
    f.discrete_progress_=scan_planner::setReferenceFromBody(
      f.discrete_reference_,f.odom_pos_,{body,f.end_pt_});
    check(f.force_visible_side_target_&&f.reference_metadata_.generation==2&&
      scan_planner::executionVersion(f.reference_metadata_).task_id==task.task_id,
      "new_owner_window_preserves_pending_side_hint_and_task_identity");
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_visible_side"&&
      !f.force_visible_side_target_&&f.local_target_query_debug_.queries>0,
      "new_owner_window_runs_and_consumes_actual_pending_side_search");
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_nominal"&&
      f.local_target_pt_.x()>f.start_pt_.x()&&f.odom_pos_.isApprox(body)&&
      f.worker_cancels==0&&f.execution_validator_->candidate_cancels==0,
      "new_owner_window_resumes_nominal_target_without_motion_or_task_reset");
  }
  {
    SCANReplanFSM f;
    f.reference_metadata_.required_mode="stair";
    f.reference_metadata_.point_required_modes={"general","stair"};
    auto s=f.execution_validator_->support->source;
    s.version=scan_planner::executionVersion(f.reference_metadata_);
    s.floor_id="stairs";s.segment_kind="landing";s.required_mode="stair";
    s.reason="original_stair_surface_returns_associated";
    s.support_reference_id="delayed_support";
    s.max_support_step_m=s.max_support_slope_rad=0.;
    const auto exact=std::make_shared<const scan_planner::PlanningSupport>(s);
    check(exact->isStair(),"delayed_support_fixture_has_valid_native_stair_geometry");
    f.execution_validator_->support.reset();
    const std::vector<Eigen::Vector3d> incumbent{{0.,0.,.55},{.4,0.,.55}};
    f.planner_manager_->selected=incumbent;
    check(!solve(f)&&f.last_attempt_failure_phase_=="waiting_reference_support"&&
      f.local_target_query_debug_.queries==0,
      "typed_stair_reference_before_support_defers_before_any_collision_query");
    check(f.planner_manager_->selected==incumbent&&f.execution_validator_->candidate_cancels==0,
      "missing_stair_support_does_not_erase_incumbent_or_cancel_task");
    auto old=s;--old.version.reference_generation;
    f.execution_validator_->support=std::make_shared<const scan_planner::PlanningSupport>(old);
    check(!solve(f)&&f.last_attempt_failure_phase_=="waiting_reference_support",
      "old_support_cannot_fulfill_new_reference_geometry");
    auto foreign=s;foreign.frame_id="map";
    f.execution_validator_->support=std::make_shared<const scan_planner::PlanningSupport>(foreign);
    check(!solve(f)&&f.last_attempt_failure_phase_=="waiting_reference_support",
      "wrong_frame_support_cannot_fulfill_odom_reference_geometry");
    f.execution_validator_->support=exact;
    check(solve(f)&&f.planner_manager_->selected_support==exact,
      "late_exact_support_retries_real_selector_without_changed_map_or_body");
  }
  {
    SCANReplanFSM f;
    GridMapTestAccess::turningLanding(*f.planner_manager_->grid_map_);
    auto s=f.execution_validator_->support->source;
    s.floor_id="stair_lower";s.segment_kind="landing";s.required_mode="stair";
    s.reason="original_stair_surface_returns_associated";
    s.support_reference_id="landing_source";s.support_xy_radius_m=.2;
    s.body_reference_height_m=.5;s.max_support_slope_rad=s.max_support_step_m=0.;
    s.support_ground_xyz.clear();
    for(int x=-36;x<=36;++x)for(int y=-36;y<=36;++y) {
      geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;s.support_ground_xyz.push_back(p);
    }
    f.execution_validator_->support=std::make_shared<const scan_planner::PlanningSupport>(s);
    f.start_pt_=f.odom_pos_={-1.2,-1.2,.5};f.end_pt_={.025,1.225,.5};
    f.discrete_reference_.set({f.start_pt_,{.025,-1.2,.5},f.end_pt_});
    f.planning_horizon_=f.discrete_reference_.length();
    f.reference_target_forward_margin_=f.reference_target_backward_margin_=0.;
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_nominal"&&
      f.local_target_pt_.isApprox(f.end_pt_,1e-12),
      "formal_stair_scope_selects_native_incident_edge_endpoint_not_chord_pose");
    check(f.planner_manager_->selected_support==f.execution_validator_->support,
      "actual_target_selector_transfers_the_same_immutable_source_support_to_manager");
    check(f.start_pt_.isApprox(f.odom_pos_,0.)&&
      f.planner_manager_->grid_map_->getPlanningInflateOccupancy(f.end_pt_,M_PI*.5)==0&&
      f.planner_manager_->grid_map_->getPlanningInflateOccupancy(f.end_pt_,
        std::atan2(f.end_pt_.y()-f.start_pt_.y(),f.end_pt_.x()-f.start_pt_.x()))==1,
      "stair_target_selection_keeps_measured_start_and_native_occupied_chord");
    check(!f.planner_manager_->grid_map_->hasQuerySupportHeightProvider(),
      "formal_target_stair_scope_does_not_leak_into_later_flat_queries");
  }
  for(double lateral:{0.,.08}) {
    SCANReplanFSM f;
    f.discrete_reference_.set({{0.,0.,.55},{.5,0.,.55},{.5,1.,.55}});
    f.start_pt_=f.odom_pos_={0.,lateral,.55};
    check(solve(f),"real_getLocalTarget_accepts_on_route_and_offset_L_corner");
    const auto& selected=f.planner_manager_->selected;
    const bool connector=f.planner_manager_->selected_has_projection_connector;
    check(connector==(lateral!=0.),"real_slice_sets_projection_contract_only_for_distinct_actual_start");
    const Eigen::Vector3d velocity(.03,0.,0.);
    const auto boundary=scan_planner::selectForwardMotionBoundary(
      f.start_pt_,velocity,selected,true,false,0.,connector);
    check(boundary.reference_direction.isApprox(Eigen::Vector3d::UnitX())&&
      !boundary.regularized&&boundary.velocity.isApprox(velocity),
      "actual_forward_velocity_is_not_zeroed_by_next_corner_direction");
    const auto guidance=scan_planner::measuredForwardApproach(selected,.05,
      [&](const Eigen::Vector3d& p,double yaw){return f.planner_manager_->grid_map_->getPlanningInflateOccupancy(p,yaw);},
      velocity,0.,connector);
    check(std::any_of(guidance.begin(),guidance.end(),[](const Eigen::Vector3d& p){
      return p.isApprox(Eigen::Vector3d(.5,0.,.55));}),
      "smooth_initializer_preserves_first_real_corner_with_native_map_queries");
  }
  {
    SCANReplanFSM f;
    f.discrete_reference_.set({{0.,0.,.55},{.5,0.,.8},{1.,0.,.8}});
    check(solve(f)&&!f.planner_manager_->selected_has_projection_connector,
      "real_getLocalTarget_stair_start_has_no_artificial_projection_segment");
    const auto& selected=f.planner_manager_->selected;
    const Eigen::Vector3d velocity(.03,0.,.015);
    const auto boundary=scan_planner::selectForwardMotionBoundary(f.start_pt_,velocity,selected,true,false,0.,false);
    check(std::abs(boundary.reference_direction.z()-.5)<1e-12&&boundary.velocity.isApprox(velocity),
      "stair_initial_slope_not_replaced_by_landing_tangent");
    const auto guidance=scan_planner::measuredForwardApproach(selected,.05,
      [&](const Eigen::Vector3d& p,double yaw){return f.planner_manager_->grid_map_->getPlanningInflateOccupancy(p,yaw);},
      velocity,0.,false);
    bool unchanged=guidance.size()==selected.size();
    for(std::size_t i=0;i<guidance.size()&&unchanged;++i)unchanged=guidance[i].isApprox(selected[i]);
    check(unchanged,"real_stair_path_and_heights_not_flattened_by_floor_initializer");
  }
  // Exercise the actual FSM velocity-boundary wiring at a tread edge. The
  // fixture's raw 5 cm chord crosses a riser, whereas its classified support
  // spans several literal treads. This is synthetic geometry, not a measured
  // robot or a claim that a particular staircase is executable.
  for(bool down:{false,true}) {
    const auto configure=[&](SCANReplanFSM& f,const std::string& support_frame) {
      const auto height=[&](double rise){return .55+(down?.6-rise:rise);};
      const std::vector<Eigen::Vector3d> path{{0.,0.,height(0.)},
        {.89,0.,height(.24)},{.91,0.,height(.36)},{1.5,0.,height(.6)}};
      f.start_pt_=f.odom_pos_=path.front();f.end_pt_=path.back();
      f.discrete_reference_.set(path);
      f.planning_horizon_=(path[1]-path[0]).norm()+.5*(path[2]-path[1]).norm();
      d1max_planning_interfaces::msg::SupportReference s;
      s.verified=true;s.floor_id="stairs";s.segment_kind=down?"stair_down":"stair_up";
      s.required_mode="stair";s.reason="original_stair_surface_returns_associated";
      s.frame_id=support_frame;s.support_reference_id="tread_fixture";
      s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
      s.support_xy_radius_m=.2;s.body_reference_height_m=.55;
      for(int x=-20;x<=50;++x)for(int y=-12;y<=12;++y) {
        geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;
        const double rise=.12*std::floor((p.x+1e-9)/.3);
        p.z=down?.6-rise:rise;s.support_ground_xyz.push_back(p);
      }
      f.execution_validator_->support=std::make_shared<const scan_planner::PlanningSupport>(s);
    };
    const auto raw_tangent=[](const SCANReplanFSM& f) {
      const double arc=f.local_debug_target_arc_;
      return Eigen::Vector3d(f.discrete_reference_.sample(std::min(arc+.05,f.discrete_reference_.length()))-
        f.discrete_reference_.sample(std::max(0.,arc-.05)));
    };
    SCANReplanFSM stair;configure(stair,"odom");
    const auto actual_start=stair.start_pt_,actual_body=stair.odom_pos_;
    const auto original_path=stair.discrete_reference_.points;
    check(solve(stair),"real_getLocalTarget_selects_nonfinal_tread_edge");
    const auto raw=raw_tangent(stair);
    const auto fit=stair.execution_validator_->support->terrainFit(stair.local_target_pt_);
    const auto& velocity=stair.local_target_vel_;
    check(fit.valid&&std::abs(raw.z())>3.*raw.head<2>().norm(),
      "tread_edge_fixture_has_valid_mean_support_but_steep_raw_riser_chord");
    check(std::abs(velocity.norm()-.3)<1e-12&&velocity.head<2>().norm()>.26&&
      std::abs(velocity.dot(fit.normal))<1e-12&&velocity.z()*(down?-1.:1.)>0.,
      "real_getLocalTarget_terminal_velocity_follows_stair_surface_not_riser");
    bool path_unchanged=original_path.size()==stair.discrete_reference_.points.size();
    for(std::size_t i=0;i<original_path.size()&&path_unchanged;++i)
      path_unchanged=original_path[i].isApprox(stair.discrete_reference_.points[i]);
    check(path_unchanged&&stair.start_pt_.isApprox(actual_start)&&stair.odom_pos_.isApprox(actual_body),
      "stair_velocity_update_does_not_move_measured_start_body_or_global_reference");

    SCANReplanFSM flat;configure(flat,"odom");
    flat.execution_validator_->support=SCANReplanFSM().execution_validator_->support;
    check(solve(flat)&&flat.local_target_pt_.isApprox(stair.local_target_pt_)&&
      flat.local_target_vel_.isApprox(raw_tangent(flat).normalized()*.3),
      "flat_support_keeps_exact_previous_raw_terminal_velocity_and_target");
    SCANReplanFSM foreign;configure(foreign,"other_map");
    check(solve(foreign)&&foreign.local_target_vel_.isApprox(raw_tangent(foreign).normalized()*.3),
      "foreign_frame_support_cannot_change_odom_terminal_velocity");
    stair.planning_horizon_=10.;
    check(solve(stair)&&stair.local_target_pt_.isApprox(stair.end_pt_)&&stair.local_target_vel_.isZero(),
      "stair_final_endpoint_keeps_zero_terminal_velocity");
  }
  {
    SCANReplanFSM f;GridMapTestAccess::person(*f.planner_manager_->grid_map_,true);
    const auto native_query=f.planner_manager_->grid_map_->getPlanningInflateOccupancy({.7,0.,.55},0.);
    check(native_query==1,"native_map_confirms_real_route_obstacle");
    // Delivery of a native CURRENT committed proof is an explicit seam;
    // test_execution_ledger covers its real identity/source/edge acceptance.
    // No nonzero demand exists: the old curve's negative proof already holds
    // the tracker, which must not be required to command through the obstacle.
    d1max_planning_interfaces::msg::TrajectoryValidation p;
    p.version=scan_planner::executionVersion(f.reference_metadata_);p.trajectory_id=1;
    p.reason="occupied";p.map_snapshot_revision=7;p.sequence=1;
    f.execution_validator_->collision=p;
    f.obstacleEventIngress();
    check(f.force_visible_side_target_&&!f.blocked_entry_replan_pending_&&f.exec_state_==SCANReplanFSM::GEN_NEW_TRAJ&&
      f.worker_cancels==1&&f.execution_validator_->candidate_cancels==1,
      "confirmed_current_collision_immediately_enters_replan_not_periodic_or_half_second_wait");
    const auto first_transitions=f.transitions;
    check(solve(f)&&f.local_target_query_debug_.reason=="reference_target_visible_side",
      "real_native_side_selector_replaces_obstructed_nominal_entry");
    check(!f.planner_manager_->selected_has_projection_connector,
      "native_visible_side_path_first_segment_is_real_motion_not_projection");
    f.obstacleEventIngress();
    check(f.worker_cancels==1&&f.execution_validator_->candidate_cancels==1,
      "consumed_edge_does_not_cancel_new_solver_each_fsm_tick");
    check(f.transitions==first_transitions,"consumed_collision_does_not_reenter_or_reset_planning_each_poll");
    check(f.have_target_&&f.reference_metadata_.generation==1,
      "obstacle_event_keeps_original_task_and_global_route");
    std::promise<int> worker;f.solve_future_=worker.get_future();
    f.blocked_entry_replan_pending_=false; // Replacement worker was submitted.
    for(int i=0;i<8;++i) {
      // The ledger has witnessed clear then occupied again on the SAME
      // incumbent. New negative proofs remain unsafe; cancellation is not
      // permitted to starve the already running bounded replacement.
      ++p.sequence;f.execution_validator_->collision=p;f.obstacleEventIngress();
    }
    check(f.worker_cancels==1&&f.execution_validator_->candidate_cancels==1&&
      !f.blocked_entry_replan_pending_&&!f.force_visible_side_target_&&f.transitions==first_transitions,
      "rapid_clear_occupied_edges_coalesce_without_cancel_or_second_queued_solve");
    worker.set_value(0);f.solve_future_.get();f.blocked_entry_replan_pending_=false;
    p.trajectory_id=2;f.execution_validator_->collision=p;f.obstacleEventIngress();
    check(f.worker_cancels==2&&f.execution_validator_->candidate_cancels==2,
      "new_applied_curve_collision_is_not_suppressed_by_previous_curve_edge");
  }
  {
    SCANReplanFSM f;
    std::promise<int> worker;f.solve_future_=worker.get_future();
    d1max_planning_interfaces::msg::TrajectoryValidation p;
    p.version=scan_planner::executionVersion(f.reference_metadata_);p.trajectory_id=1;
    f.execution_validator_->collision=p;f.obstacleEventIngress();
    check(f.exec_state_==SCANReplanFSM::GEN_NEW_TRAJ&&f.worker_cancels==1&&
      !scan_planner::formalReseedSubmissionDue(std::chrono::steady_clock::now(),{},f.solve_future_.valid(),.1),
      "collision_has_priority_over_old_worker_but_does_not_start_second_worker");
    const auto last=std::chrono::steady_clock::time_point{}+std::chrono::seconds(10);
    worker.set_value(0);f.solve_future_.get();
    check(!scan_planner::formalReseedSubmissionDue(last+std::chrono::milliseconds(99),last,false,.1)&&
      scan_planner::formalReseedSubmissionDue(last+std::chrono::milliseconds(100),last,false,.1),
      "danger_event_preserves_configured_actual_solve_rate_without_extra_500ms_delay");
  }
  {
    SCANReplanFSM f;f.reference_metadata_.generation=2;
    d1max_planning_interfaces::msg::TrajectoryValidation p;
    p.version=scan_planner::executionVersion(1);p.trajectory_id=1;
    f.execution_validator_->collision=p;f.obstacleEventIngress();
    check(f.worker_cancels==1&&f.force_visible_side_target_,
      "actual_incumbent_collision_still_replans_after_normal_reference_generation_advance");
    p.version.task_id="retired_task";f.execution_validator_->collision=p;
    f.obstacleEventIngress();check(f.worker_cancels==1,
      "late_foreign_task_event_cannot_cancel_current_candidate");
    f.have_target_=false;p.version=scan_planner::executionVersion(2);
    f.execution_validator_->collision=p;f.obstacleEventIngress();
    check(f.worker_cancels==1,"canceled_task_collision_does_not_restart_solver");
  }
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
    check(f.odom_pos_.isApprox(body)&&f.discrete_reference_.points.back().isApprox(endpoint)&&f.reference_metadata_.generation==1,
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
      *f.execution_validator_->support->flatSupport(),braking,f.start_pt_,{2.,0.,.55},"odom",.1);
    std::cout<<"occupied_counterexample_reason="<<proof.reason<<"\n";
    check(!proof.valid&&proof.reason=="motion_sweep_occupied",
      "persistent_real_obstacle_still_denies_native_motion_after_hint_consumption");
  }
  {
    SCANReplanFSM f;GridMapTestAccess::unknown(*f.planner_manager_->grid_map_);
    check(solve(f),"official_geometry_may_generate_candidate_in_unknown_space");
    const scan_planner::BrakingModel braking{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
    const auto proof=scan_planner::validateStraightBrakingCorridor(*f.planner_manager_->grid_map_,
      *f.execution_validator_->support->flatSupport(),braking,f.start_pt_,{2.,0.,.55},"odom",.1);
    std::cout<<"unknown_counterexample_reason="<<proof.reason<<"\n";
    check(proof.valid&&proof.reason=="official_occupied_motion_sweep",
      "official_native_screening_uses_same_unknown_policy_not_an_execution_authority");
    GridMapTestAccess::strictComparison(*f.planner_manager_->grid_map_);
    const auto strict=scan_planner::validateStraightBrakingCorridor(*f.planner_manager_->grid_map_,
      *f.execution_validator_->support->flatSupport(),braking,f.start_pt_,{2.,0.,.55},"odom",.1);
    check(!strict.valid&&strict.reason=="motion_sweep_unknown_or_expired",
      "explicit_strict_comparison_still_requires_observed_free_evidence");
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
    # Compile the exact first production FSM callback block, before polling
    # the worker. This also works on the pre-fix source, giving same-input
    # evidence rather than treating an absent new method as the old bug.
    source=args.source.read_text()
    signature = ('execFSMOwnerCallback()' if 'void SCANReplanFSM::execFSMOwnerCallback()' in source
                 else 'execFSMCallback()')
    start=source.index('  void SCANReplanFSM::'+signature)
    end=source.index('    if (pollSolveWorker()) return;',start)
    ingress=source[start:end].replace(signature, 'obstacleEventIngress()',1)+'  }\n'
    cpp.write_text(HARNESS.replace('__METHOD__', methods).replace('__OBSTACLE_INGRESS__',ingress))
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
    subprocess.run(['/usr/bin/c++', *flags, '-c', str(package/'src/motion_sweep.cpp'),
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
                  harness_sha256=hashlib.sha256(HARNESS.encode()).hexdigest(),
                  production_methods_sha256=hashlib.sha256((methods+ingress).encode()).hexdigest(),
                  scope='production FSM obstacle ingress, getLocalTarget, selectors, GridMap and motion sweep; passive proof delivery/acquisition; no ROS initialization')
    (directory/'result.json').write_text(json.dumps(record, indent=2)+'\n')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
