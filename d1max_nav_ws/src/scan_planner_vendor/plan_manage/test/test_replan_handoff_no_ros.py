#!/usr/bin/env python3
"""Compile actual native replan/start-state methods against passive seams.

No ROS node or optimizer runs. This exercises the real mode/generation/time
decisions and boundary handed to the optimizer; it is not feasibility evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <future>
#include <deque>
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR_THROTTLE(...) ((void)0)
struct TimeValue { double value=100.; double seconds() const {return value;}
  TimeValue()=default;TimeValue(double seconds):value(seconds){}
  explicit TimeValue(std::int64_t ns,int=0):value(ns*1e-9){}
  std::int64_t nanoseconds() const {return static_cast<std::int64_t>(value*1e9);}
  TimeValue operator-(TimeValue b) const {return {value-b.value};} };
namespace rclcpp {using Time=TimeValue;}
namespace geometry_msgs {namespace msg {
struct Twist {struct Axis {double x=0.,y=0.,z=0.;};Axis linear,angular;};
}}
namespace nav_msgs {namespace msg {
struct Odometry {
  using ConstSharedPtr=std::shared_ptr<const Odometry>;
  struct {TimeValue stamp;std::string frame_id="odom";} header;
  struct {struct {geometry_msgs::msg::Twist::Axis position;
    struct {double w=1.,x=0.,y=0.,z=0.;} orientation;} pose;} pose;
  struct {geometry_msgs::msg::Twist twist;} twist;
};
}}
struct Node { struct Clock {int get_clock_type()const{return 0;}};Clock clock;
  Clock* get_clock(){return &clock;}TimeValue stamp; TimeValue now() const {return stamp;} };
struct Curve { Eigen::Vector3d value{.4,.2,.1};
  Eigen::Vector3d slope{.1,0.,0.};
  Eigen::Vector3d evaluateDeBoorT(double t) const {return value+t*slope;} };
namespace scan_planner {
__SOURCE_ORDER_RULE__
struct MeasuredBodyPose {Eigen::Vector3d position;Eigen::Quaterniond orientation;double source_stamp;std::string frame;};
struct Splice {struct Point {double x=0.,y=0.,z=0.;};
  struct {Point position{1.2,2.,.55},velocity{.12,.03,0.},acceleration{.02,0.,0.};} wire;};
struct LocalTrajData {
  TimeValue start_time_{99.}; double duration_=5.; int traj_id_=17;
  Curve velocity_traj_, acceleration_traj_;
  Curve position_traj_{{.9,2.,.55},{.1,0.,0.}};
};
struct Manager {
  struct Map {double getResolution() const{return .08;}};
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  LocalTrajData local_data_; int global_calls=0;
  std::optional<Splice> planned;void setPlannedSplice(std::optional<Splice> p){planned=std::move(p);}
  std::optional<MeasuredBodyPose> measured_body;Eigen::Vector3d measured_velocity;
  void setMeasuredBodyPose(const MeasuredBodyPose& p,const std::string&,double){measured_body=p;}
  void setMeasuredBodyVelocity(const Eigen::Vector3d& v){measured_velocity=v;}
  bool planGlobalTraj(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,
      const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&) {
    ++global_calls; return true;
  }
};
class SCANReplanFSM {
 public:
  enum NAVI_MODE {MANUAL_TARGET=1,PRESET_TARGET=2,REFERENCE_PATH=3};
  enum State {EXEC_TRAJ,WAIT_ENVIRONMENT,REPLAN_TRAJ,GEN_NEW_TRAJ};
  int exec_state_=EXEC_TRAJ;
  bool have_target_=true,have_odom_=true;
  std::string last_attempt_failure_phase_;
  std::future<int> solve_future_;
  std::optional<int> completed_solve_,retired_solve_failure_;
  bool solveWorkPending() const;
  std::chrono::steady_clock::time_point last_solve_submit_{};
  double min_replan_interval_=.1;
  void changeFSMExecState(int state,const char*) {exec_state_=state;}
  Node clock; Node *node_=&clock;
  struct Validator {
    using Twist=geometry_msgs::msg::Twist;
    struct BodySourceSample {MeasuredBodyPose pose;std::optional<Twist> twist;std::int64_t source_ns;};
    std::optional<BodySourceSample> latest;
    auto latestPlanningBodyAt(int,std::int64_t,double)const{return latest;}
    int body_writes=0;
    void body(const MeasuredBodyPose&,const Twist&,std::int64_t){++body_writes;}
    struct Reseed {int version=1,trajectory_id=7;TimeValue body_source_stamp;
      std::string reason="measured_candidate_entry_changed";bool terminal=false,actual_body=true;};
    std::optional<Reseed> reseed;
    std::optional<Reseed> consumeFormalReseedAt(std::int64_t,double) {
      auto out=reseed;reseed.reset();return out;
    }
    struct Boundary {Eigen::Vector3d velocity,acceleration;};
    std::optional<Boundary> value;
    std::optional<Splice> splice;
    auto plannedSpliceAtBody(int,std::int64_t,double,const MeasuredBodyPose&,std::int64_t)const{return splice;}
    std::optional<Boundary> committedBoundary(const Eigen::Vector3d&,const Eigen::Vector3d&,
        std::int64_t,double)const {return value;}
  };
  std::shared_ptr<Validator> execution_validator_;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  bool go2_execution_frozen_=false,reference_path_guidance_=true;
  bool planning_only_=false,local_follower_=false;int reference_metadata_=1;
  std::optional<int> actual_body_seed_request_;
  int executionVersion(int v)const{return v;}
  bool sameExecutionTask(int a,int b)const{return a==b;}
  bool require_tagged_reference_=true,have_new_target_=false;
  int navi_mode_=3,accepted_curve_id_=17;
  struct Progress {bool valid=true,holding=false; int trajectory_id=17;
    std::uint64_t generation=7;double curve_time=1.;struct {TimeValue stamp;} header;};
  std::optional<Progress> tracking_progress_{Progress{}};
  std::shared_ptr<Progress> pending_tracking_progress_;
  bool strict_input_frames_=true,odom_twist_in_body_frame_=false;
  bool rviz_height_ready_=false,preset_started_=false;double rviz_goal_height_=0.;
  int progress_consumptions=0,marker_publications=0,preset_starts=0;
  void consumeTrackingProgress(const std::shared_ptr<Progress>&){++progress_consumptions;}
  void publishSelfInflationMarker(){++marker_publications;}
  void planGlobalTrajbyGivenWps(){++preset_starts;}
  double odom_timeout_=.5;
  TimeValue last_odom_time_;
  std::uint64_t accepted_curve_generation_=7,reference_generation_=7;
  Eigen::Vector3d odom_pos_{1.,2.,.55},odom_vel_{.02,0.,0.},end_pt_{-10.,2.,.55};
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();std::string self_inflation_frame_id_="odom";
  struct OdomSourceSample {std::int64_t source_ns;Eigen::Vector3d position;std::string frame;};
  std::deque<OdomSourceSample> odom_source_history_;
  Eigen::Vector3d start_pt_,start_vel_,start_acc_;
  Eigen::Vector3d solved_position,solved_velocity,solved_acceleration;
  int solves=0; bool solve_result=true,target_valid=true;
  bool adjustGlobalTargetIfOccupied(){return target_valid;}
  bool callReboundReplan(bool,bool) {
    ++solves; solved_position=start_pt_;solved_velocity=start_vel_;solved_acceleration=start_acc_;
    return solve_result;
  }
  bool planFromCurrentTraj();
  bool refreshPlanningBodyFromInbox();
  void finishOwnerBodyUpdate();
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr&);
  void setStartStateFromOdomOrCurrentTraj();
  void reseedEventIngress(){__RESEED_INGRESS__}
};
__CALLBACKS__
}
int main() {
  using FSM=scan_planner::SCANReplanFSM;
  int checks=0,failures=0;
  auto expect=[&](bool ok,const char*name) {
    ++checks;failures+=!ok;std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";
  };
  auto measured=[](const FSM&f) {
    return f.start_pt_.isApprox(f.odom_pos_)&&f.start_vel_.isApprox(f.odom_vel_)&&f.start_acc_.isZero();
  };
  {
    FSM f;f.local_follower_=true;f.last_odom_time_.value=99.68;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->splice=scan_planner::Splice{};
    FSM::Validator::BodySourceSample source;
    source.pose={{1.12,2.08,.55},Eigen::Quaterniond::Identity(),99.98,"odom"};
    source.twist=FSM::Validator::Twist{};source.twist->linear={.31,.01,0.};source.source_ns=99980000000LL;
    f.execution_validator_->latest=source;
    expect(f.planFromCurrentTraj()&&f.solved_position.isApprox(source.pose.position)&&
      f.solved_velocity.isApprox(Eigen::Vector3d(.31,.01,0.))&&!f.planner_manager_->planned,
      "busy_map_owner_solves_from_latest_independent_original_acquisition_not_320ms_old_callback");
    expect(f.last_odom_time_.nanoseconds()==source.source_ns&&
      f.planner_manager_->measured_body->source_stamp==source.pose.source_stamp&&
      f.planner_manager_->measured_velocity.isApprox(f.solved_velocity),
      "fsm_optimizer_pose_velocity_and_source_stamp_remain_one_actual_sample");
    expect(f.odom_source_history_.size()==1&&!f.refreshPlanningBodyFromInbox(),
      "duplicate_inbox_read_does_not_renew_source_or_append_history");
    source.source_ns-=20000000LL;source.pose.source_stamp-=.02;source.pose.position.x()=9.;
    f.execution_validator_->latest=source;
    expect(!f.refreshPlanningBodyFromInbox()&&f.odom_pos_.x()==1.12,
      "out_of_order_owner_delivery_cannot_replace_the_newer_acquired_seed");
  }
  {
    FSM f;f.local_follower_=true;f.last_odom_time_.value=99.68;
    f.navi_mode_=FSM::MANUAL_TARGET;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    FSM::Validator::BodySourceSample source;
    source.pose={{1.12,2.08,.57},Eigen::Quaterniond::Identity(),99.98,"odom"};
    source.twist=FSM::Validator::Twist{};source.twist->linear={.31,.01,0.};
    source.source_ns=99980000000LL;f.execution_validator_->latest=source;
    f.pending_tracking_progress_=std::make_shared<FSM::Progress>();
    expect(f.refreshPlanningBodyFromInbox()&&f.progress_consumptions==1&&
      !f.pending_tracking_progress_&&f.marker_publications==1&&
      f.rviz_height_ready_&&f.rviz_goal_height_==.57,
      "independent_inbox_refresh_runs_pending_progress_marker_and_manual_height_owner_finish");
    expect(f.execution_validator_->body_writes==0&&f.last_odom_time_.nanoseconds()==source.source_ns,
      "owner_finish_does_not_echo_body_to_validator_or_restamp_acquisition");
    auto duplicate=std::make_shared<nav_msgs::msg::Odometry>();
    duplicate->header.stamp=TimeValue(source.source_ns);duplicate->pose.pose.position={9.,9.,9.};
    f.odometryCallback(duplicate);
    duplicate->header.stamp=TimeValue(source.source_ns-20000000LL,0);f.odometryCallback(duplicate);
    expect(f.marker_publications==1&&f.progress_consumptions==1&&
      f.execution_validator_->body_writes==0&&f.odom_pos_.isApprox(source.pose.position)&&
      f.last_odom_time_.nanoseconds()==source.source_ns,
      "duplicate_and_old_real_odometry_callbacks_neither_rollback_nor_repeat_owner_finish");
    auto next=std::make_shared<nav_msgs::msg::Odometry>();next->header.stamp=TimeValue(100000000000LL,0);
    next->pose.pose.position={1.13,2.08,.59};next->twist.twist.linear={.32,.02,0.};
    f.pending_tracking_progress_=std::make_shared<FSM::Progress>();f.odometryCallback(next);
    expect(f.marker_publications==2&&f.progress_consumptions==2&&
      f.execution_validator_->body_writes==0&&!f.pending_tracking_progress_&&
      f.odom_pos_.isApprox(Eigen::Vector3d(1.13,2.08,.59))&&f.rviz_goal_height_==.57,
      "newer_reference_relay_runs_owner_finish_without_overwriting_independent_typed_body");
  }
  {
    FSM f;f.local_follower_=true;f.last_odom_time_.value=99.68;f.navi_mode_=FSM::PRESET_TARGET;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    FSM::Validator::BodySourceSample source;
    source.pose={{1.,2.,.55},Eigen::Quaterniond::Identity(),99.98,"odom"};
    source.twist=FSM::Validator::Twist{};source.source_ns=99980000000LL;
    f.execution_validator_->latest=source;
    expect(f.refreshPlanningBodyFromInbox()&&f.preset_started_&&f.preset_starts==1,
      "preset_owner_start_is_preserved_when_independent_body_arrives_first");
    source.source_ns=100000000000LL;source.pose.source_stamp=100.;f.execution_validator_->latest=source;
    expect(f.refreshPlanningBodyFromInbox()&&!f.refreshPlanningBodyFromInbox()&&
      f.preset_starts==1&&f.marker_publications==2&&f.execution_validator_->body_writes==0,
      "newer_and_duplicate_inbox_samples_do_not_repeat_preset_task_or_echo_source");
  }
  {
    FSM f;expect(f.planFromCurrentTraj(),"executing_reference_replan_succeeds");
    expect(f.solved_position.isApprox(f.odom_pos_),"position_is_actual_body_not_virtual_spline");
    expect(f.solved_velocity.isApprox(Eigen::Vector3d(.5,.2,.1)) &&
           f.solved_acceleration.isApprox(Eigen::Vector3d(.5,.2,.1)),
           "executing_same_generation_preserves_incumbent_velocity_and_acceleration");
    expect(f.planner_manager_->global_calls==0,"local_retry_does_not_replace_global_route");
    expect(f.start_vel_.head<2>().dot((f.end_pt_-f.odom_pos_).head<2>())<0.,
           "routed_turn_away_from_final_goal_is_not_forced_to_zero_velocity");
    f.clock.stamp.value+=.02;
    expect(f.planFromCurrentTraj() && f.solved_velocity.isApprox(Eigen::Vector3d(.5,.2,.1)),
           "wall_clock_does_not_advance_handoff_progress");
    f.tracking_progress_->curve_time=1.1;f.odom_pos_.x()+=.01;
    expect(f.planFromCurrentTraj() && f.solved_velocity.isApprox(Eigen::Vector3d(.51,.2,.1)),
           "new_measured_progress_advances_handoff_derivatives");
  }
  {
    FSM f;f.go2_execution_frozen_=true;f.odom_vel_={-.25,.07,0.};
    f.planFromCurrentTraj();expect(measured(f),"frozen_preview_uses_measured_reverse_motion_not_spline_progress");
    f.clock.stamp.value+=1.;f.planFromCurrentTraj();
    expect(measured(f),"frozen_clock_does_not_create_virtual_body_motion");
  }
  for(int mode=0;mode<11;++mode) {
    FSM f;
    switch(mode) {
      case 0:f.have_new_target_=true;break;
      case 1:f.accepted_curve_generation_=6;break;
      case 2:f.accepted_curve_id_=-1;break;
      case 3:f.planner_manager_->local_data_.traj_id_=18;break;
      case 4:f.planner_manager_->local_data_.start_time_.value=0.;break;
      case 5:f.clock.stamp.value=110.;break;
      case 6:f.tracking_progress_.reset();break;
      case 7:f.planner_manager_->local_data_.acceleration_traj_.value.x()=
          std::numeric_limits<double>::quiet_NaN();break;
      case 8:f.tracking_progress_->holding=true;break;
      case 9:f.tracking_progress_->valid=false;break;
      case 10:f.odom_pos_.z()+=3.;break;
    }
    f.planFromCurrentTraj();expect(measured(f),"new_goal_unknown_old_expired_future_or_nonfinite_curve_uses_measured_boundary");
  }
  {
    FSM f;f.solve_result=false;
    expect(!f.planFromCurrentTraj()&&f.solves==1,"failed_guided_solve_is_not_success_or_random_second_attempt");
    f.target_valid=false;f.solves=0;
    expect(!f.planFromCurrentTraj()&&f.solves==0,"invalid_target_never_runs_optimizer");
  }
  {
    FSM f;f.planning_only_=true;f.go2_execution_frozen_=false;
    f.tracking_progress_.reset();
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->value=FSM::Validator::Boundary{{9.,9.,9.},{9.,9.,9.}};
    expect(f.planFromCurrentTraj()&&measured(f),
      "planning_only_uses_actual_odometry_without_tracker_frozen_or_controller_derivatives");
  }
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->value=FSM::Validator::Boundary{{.12,.03,0.},{.02,0.,0.}};
    f.planner_manager_->local_data_.velocity_traj_.value={9.,9.,9.};
    expect(f.planFromCurrentTraj()&&f.solved_velocity.isApprox(Eigen::Vector3d(.12,.03,0.)),
      "execution_handoff_uses_committed_ledger_not_latest_candidate");
    f.go2_execution_frozen_=true;f.planFromCurrentTraj();
    expect(f.solved_velocity.isApprox(Eigen::Vector3d(.12,.03,0.))&&
      f.solved_acceleration.isApprox(Eigen::Vector3d(.02,0.,0.)),
      "obsolete_frozen_flag_cannot_override_formal_committed_boundary");
    f.execution_validator_->value.reset();f.planFromCurrentTraj();
    expect(measured(f),"missing_committed_proof_keeps_real_measured_boundary");
  }
  {
    FSM f;f.local_follower_=true;f.have_new_target_=true;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->splice=scan_planner::Splice{};
    const auto actual=f.odom_pos_;f.planFromCurrentTraj();
    expect(!f.planner_manager_->planned&&measured(f)&&f.odom_pos_.isApprox(actual),
      "formal_successor_always_uses_current_body_not_available_old_future_seam");
    f.execution_validator_->splice.reset();f.planFromCurrentTraj();
    expect(!f.planner_manager_->planned&&measured(f),
      "missing_safe_common_prefix_returns_to_measured_seed_not_stale_future_boundary");
  }
  {
    FSM f;f.local_follower_=true;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->splice=scan_planner::Splice{};
    f.execution_validator_->reseed=FSM::Validator::Reseed{};
    f.reseedEventIngress();
    expect(f.exec_state_==FSM::GEN_NEW_TRAJ&&f.actual_body_seed_request_==f.reference_metadata_,
      "production_actual_body_event_records_task_scoped_seed_intent_not_only_state_label");
    f.odom_vel_={.41698,0.,0.};
    f.setStartStateFromOdomOrCurrentTraj();
    expect(!f.planner_manager_->planned&&measured(f),
      "actual_body_request_overrides_available_old_future_splice_and_preserves_measured_velocity");
    expect(f.actual_body_seed_request_.has_value(),
      "preparing_start_does_not_consume_reseed_before_actual_solver_submission");
    // A snapshot miss or submit-rate deferral runs this start-state callback
    // again after new original odometry. It must not retain the first seed.
    f.odom_pos_.x()+=.021;f.odom_vel_={.38,.02,0.};
    f.setStartStateFromOdomOrCurrentTraj();
    expect(!f.planner_manager_->planned&&measured(f),
      "deferred_actual_body_reseed_uses_latest_acquired_position_and_velocity");
    // committedBoundary's production contract retains CURRENT measured v;
    // acceleration is allowed only when it came from that exact source stamp.
    f.execution_validator_->value=FSM::Validator::Boundary{f.odom_vel_,{.01,0.,0.}};
    f.setStartStateFromOdomOrCurrentTraj();
    expect(!f.planner_manager_->planned&&f.start_pt_.isApprox(f.odom_pos_)&&
      f.start_vel_.isApprox(f.odom_vel_)&&f.start_acc_.isApprox(Eigen::Vector3d(.01,0.,0.)),
      "forced_actual_seed_can_keep_only_same_source_measured_acceleration");
    f.actual_body_seed_request_.reset();f.planFromCurrentTraj();
    expect(!f.planner_manager_->planned&&f.solved_position.isApprox(f.odom_pos_)&&
      f.solved_velocity.isApprox(f.odom_vel_),
      "ordinary_successor_keeps_actual_body_after_reseed_request_is_consumed");
  }
  {
    FSM f;f.local_follower_=true;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->splice=scan_planner::Splice{};
    f.actual_body_seed_request_=f.reference_metadata_+1;
    f.planFromCurrentTraj();
    expect(!f.actual_body_seed_request_&&!f.planner_manager_->planned&&measured(f),
      "old_task_reseed_cannot_override_new_task_measured_boundary_or_revive_old_curve");
  }
  {
    FSM f;f.local_follower_=true;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->reseed=FSM::Validator::Reseed{};
    f.execution_validator_->reseed->version=2;
    f.reseedEventIngress();
    expect(!f.actual_body_seed_request_&&f.exec_state_==FSM::EXEC_TRAJ,
      "late_other_task_event_cannot_request_actual_seed_or_restart_fsm");
    f.execution_validator_->reseed=FSM::Validator::Reseed{};
    f.last_solve_submit_=std::chrono::steady_clock::now();
    f.reseedEventIngress();
    expect(f.execution_validator_->reseed.has_value()&&!f.actual_body_seed_request_,
      "submission_throttle_does_not_consume_pending_native_reseed_event");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";
  return failures?1:0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1] / 'src/scan_replan_fsm.cpp')
    parser.add_argument('--include', type=Path, default=Path(__file__).resolve().parents[1] / 'include')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    helper='  bool SCANReplanFSM::refreshPlanningBodyFromInbox()'
    start = source.index(helper) if helper in source else source.index('  bool SCANReplanFSM::planFromCurrentTraj()')
    end = source.index('  void SCANReplanFSM::trackingProgressCallback(', start)
    production_methods = source[start:end]
    if helper not in source:
        production_methods='bool scan_planner::SCANReplanFSM::refreshPlanningBodyFromInbox(){return false;}\n'+production_methods
    owner_start=source.index('  void SCANReplanFSM::odometryCallback(')
    owner_end=source.index('  void SCANReplanFSM::go2ExecutionFrozenCallback(',owner_start)
    production_methods=source[owner_start:owner_end]+production_methods
    pending_start=source.index('  bool SCANReplanFSM::solveWorkPending() const')
    pending_end=source.index('  void SCANReplanFSM::collectSolveCompletion()',pending_start)
    production_methods+=source[pending_start:pending_end]
    rule_source=(args.include/'plan_manage/trajectory_collision.hpp').read_text()
    rule_start=rule_source.index('inline bool measuredPoseSourceAccepted(')
    rule_end=rule_source.index('inline bool measuredBodyYaw(',rule_start)
    source_order_rule=rule_source[rule_start:rule_end]
    event_start=source.index('    if(execution_validator_&&reference_path_guidance_&&have_target_&&have_odom_&&')
    event_end=source.index('    static int fsm_num',event_start)
    event_ingress=source[event_start:event_end]
    with tempfile.TemporaryDirectory(prefix='scan-replan-handoff-') as directory:
        cpp, executable = Path(directory)/'handoff.cpp', Path(directory)/'handoff'
        cpp.write_text(HARNESS.replace('__CALLBACKS__', production_methods).replace('__RESEED_INGRESS__',event_ingress)
            .replace('__SOURCE_ORDER_RULE__',source_order_rule))
        command = ['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
                   '-I'+str(args.include), '-I/usr/include/eigen3', str(cpp), '-o', str(executable)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(executable)], check=False, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(args.source),
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                production_method_sha256=hashlib.sha256(production_methods.encode()).hexdigest(),
                returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                ros_initialized=False, robot_connected=False,
                scope='actual start-state and replan methods; passive clock, known curve derivatives and solve sink'), indent=2)+'\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
