#!/usr/bin/env python3
"""Compile the real reference callbacks against passive transport/test doubles.

No rclcpp initialization, ROS graph, map substitute, optimizer or robot commands.
Only clock/messages/publication sinks are doubled. Both callback bodies and the
generation/waypoint helpers are production source, including for baseline runs.
The scope is reference admission and refusal notification, not path feasibility.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
struct Header { std::string frame_id; };
struct Position { double x=0., y=0., z=0.; };
struct PoseStamped { Header header; struct { Position position; } pose; };
namespace nav_msgs { namespace msg {
struct Path { using ConstSharedPtr=std::shared_ptr<const Path>; Header header;
  std::vector<PoseStamped> poses; };
}}
namespace d1max_planning_interfaces { namespace msg {
struct ReferencePath { using ConstSharedPtr=std::shared_ptr<const ReferencePath>;
  std::string session_id; std::uint64_t generation=0; nav_msgs::msg::Path path;
  std::uint32_t schema_version=0;
  std::string point_reference,task_id,route_id,route_hash,map_version_id,segment_id;
  std::vector<std::string> point_segment_ids,point_segment_kinds,point_required_modes;
};
}}
namespace builtin_interfaces { namespace msg { struct Time {}; }}
struct ClockValue { double value=0.; double seconds() const {return value;}
  ClockValue operator-(ClockValue b) const {return {value-b.value};} };
struct PassiveNode { double clock=100.; ClockValue now() const {return {clock};} };
namespace scan_planner {
struct ReferenceTargetResult {};
inline int executionVersion(const d1max_planning_interfaces::msg::ReferencePath& m){return m.generation;}
struct PassiveExecutionLedger {bool accepts=true,retired=false;
  bool cancelReference(int){if(accepts)retired=true;return accepts;}};
class SCANReplanFSM {
public:
  enum {WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_TRAJ};
  PassiveNode clock; PassiveNode *node_=&clock;
  std::string navigation_session_id_="session", self_inflation_frame_id_="map";
  bool have_reference_generation_=false, have_odom_=false;
  std::uint64_t reference_generation_=0, predecessor_id_=0;
  int accepted_curve_id_=-1, replan_fail_count_=0, exec_state_=WAIT_TARGET;
  bool predecessor_safe_=false, have_target_=false, have_new_target_=false;
  bool trigger_=false, need_hover_stop_=false, flag_escape_emergency_=false;
  bool strict_input_frames_=true;
  bool require_schema_v2_=false;
  bool force_visible_side_target_=false, blocked_entry_replan_pending_=false;
  d1max_planning_interfaces::msg::ReferencePath reference_metadata_;
  std::optional<int> tracking_progress_;
  std::shared_ptr<PassiveExecutionLedger> execution_validator_;
  std::optional<int> execution_proposal_;
  int cancellation_requests_=0;
  void cancelSolveWorker(){++cancellation_requests_;}
  double odom_timeout_=.5, reference_path_z_offset_=.55, reference_start_tolerance_=1.;
  ClockValue last_odom_time_{0.};
  builtin_interfaces::msg::Time predecessor_check_stamp_;
  ReferenceTargetResult local_target_query_debug_;
  Eigen::Vector3d odom_pos_{0.,0.,.55};
  std::vector<Eigen::Vector3d> local_debug_selected_reference_, active_waypoints_;
  struct Event {std::string phase,session; std::uint64_t generation; bool valid;};
  std::vector<Event> emitted;
  int accepted_calls=0, emergency_stop_calls=0;
  Header localPlanDebugHeader() {return {};}
  void publishAttemptDebug(Header,bool) {}
  void publishInvalidLocalPlanDebug(const std::string &phase,bool=false) {
    emitted.push_back({phase,navigation_session_id_,reference_generation_,false});
  }
  void changeFSMExecState(int state,const std::string&) {exec_state_=state;}
  void callEmergencyStop(const Eigen::Vector3d&) {++emergency_stop_calls;}
  bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d>&) {
    ++accepted_calls; have_target_=true; have_new_target_=true;return true;
  }
  void typedPathCallback(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr&);
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr&);
};
__CALLBACKS__
}
using scan_planner::SCANReplanFSM;
using Message=d1max_planning_interfaces::msg::ReferencePath;
std::shared_ptr<Message> path(std::uint64_t gen) {
  auto m=std::make_shared<Message>();m->session_id="session";m->generation=gen;
  m->path.header.frame_id="map";
  PoseStamped p;p.header.frame_id="map";m->path.poses.push_back(p);
  p.pose.position.x=1.;m->path.poses.push_back(p);return m;
}
void fresh(SCANReplanFSM &f) {f.have_odom_=true;f.last_odom_time_={f.clock.clock};}
int main() {
  int failures=0,checks=0;
  const auto check=[&](bool value,const char*name) {
    ++checks;std::cout<<(value?"PASS ":"FAIL ")<<name<<"\n";if(!value)++failures;
  };
  const auto phase=[](const SCANReplanFSM&f,const char*wanted) {
    return !f.emitted.empty() && f.emitted.back().phase==wanted &&
      !f.emitted.back().valid && f.emitted.back().session=="session" &&
      f.emitted.back().generation==f.reference_generation_;
  };
  {
    SCANReplanFSM f;f.typedPathCallback(path(1));
    check(phase(f,"reference_rejected_odometry"),"no_odometry_explicit_generation_bound_rejection");
    check(!f.have_target_ && f.exec_state_==f.WAIT_TARGET && f.accepted_calls==0,
          "rejected_generation_has_no_target_or_plan");
    fresh(f);check(f.accepted_calls==0 && !f.have_target_,"body_recovery_does_not_replay_reference");
    f.typedPathCallback(path(1));check(f.accepted_calls==0,"same_generation_remains_consumed");
    f.typedPathCallback(path(2));
    check(f.accepted_calls==1 && f.have_target_ && f.exec_state_==f.GEN_NEW_TRAJ,
          "new_owner_generation_with_fresh_body_can_be_accepted");
  }
  {
    SCANReplanFSM f;fresh(f);f.last_odom_time_.value-=.501;f.typedPathCallback(path(1));
    check(phase(f,"reference_rejected_odometry") && !f.have_target_,"stale_body_explicit_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.header.frame_id="foreign";f.typedPathCallback(m);
    check(phase(f,"reference_rejected_frame") && !f.have_target_,"path_frame_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[1].header.frame_id="foreign";f.typedPathCallback(m);
    check(phase(f,"reference_rejected_frame") && !f.have_target_,"pose_frame_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);
    m->path.poses[1].pose.position.x=std::numeric_limits<double>::quiet_NaN();f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"nan_geometry_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[0].pose.position.x=3.;f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"far_start_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[1].pose.position.x=0.;f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"collapsed_geometry_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);f.typedPathCallback(path(1));auto cancel=path(2);cancel->path.poses.clear();
    f.typedPathCallback(cancel);fresh(f);f.typedPathCallback(path(1));
    check(phase(f,"cancelled") && !f.have_target_ && f.accepted_calls==1,
          "explicit_cancel_and_old_generation_never_resurrect");
    auto foreign=path(3);foreign->session_id="foreign";f.typedPathCallback(foreign);
    check(f.reference_generation_==2 && !f.have_target_,"foreign_session_cannot_take_generation");
  }
  {
    SCANReplanFSM f;fresh(f);f.require_schema_v2_=true;f.typedPathCallback(path(1));
    check(phase(f,"invalid_reference_schema") && !f.have_target_,"v2_session_rejects_legacy_reference");
    auto m=path(2);m->schema_version=2;m->point_reference="ground";m->task_id="task";m->route_id="route";
    m->route_hash="hash";m->map_version_id="map-v";m->segment_id="floor1";
    m->point_segment_ids={"floor1","stairs"};
    m->point_segment_kinds={"floor","stairs"};m->point_required_modes={"walk","climb"};
    f.typedPathCallback(m);
    check(f.have_target_ && f.reference_metadata_.point_segment_ids[1]=="stairs",
          "v2_keeps_multilevel_semantic_point_arrays");
    auto body=std::make_shared<Message>(*m);body->generation=3;body->point_reference="body_center";
    f.typedPathCallback(body);
    check(!f.have_target_ && phase(f,"invalid_reference_schema"),"body_center_rejects_second_height_offset");
    body->generation=4;f.reference_path_z_offset_=0.;f.typedPathCallback(body);
    check(f.have_target_,"body_center_admitted_only_with_zero_offset");
    m=path(5);m->schema_version=2;f.typedPathCallback(m);
    check(!f.have_target_ && f.cancellation_requests_==5,
          "invalid_replacement_still_cancels_worker_and_revokes_prior_route");
  }
  {
    SCANReplanFSM f;fresh(f);f.typedPathCallback(path(1));
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto m=path(2);m->schema_version=2;m->path.poses.clear();
    f.execution_validator_->accepts=false;f.typedPathCallback(m);
    check(f.have_target_&&f.reference_generation_==1,"foreign_cancel_keeps_native_owner");
    f.execution_validator_->accepts=true;f.typedPathCallback(m);
    check(f.execution_validator_->retired&&phase(f,"cancelled")&&!f.have_target_,
          "execution_cancel_retires_ledger_before_fresh_cancel_ack");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";
  return failures?1:0;
}
'''


def callback(source, name, next_name):
    start = source.index('  void SCANReplanFSM::' + name + '(')
    end = source.index('  void SCANReplanFSM::' + next_name + '(', start)
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1] / 'src/scan_replan_fsm.cpp')
    parser.add_argument('--include', type=Path, default=Path(__file__).resolve().parents[1] / 'include')
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    extracted = callback(source, 'typedPathCallback', 'publishTrajectory')
    extracted += callback(source, 'pathCallback', 'odometryCallback')
    with tempfile.TemporaryDirectory(prefix='reference-callback-') as temporary:
        root = args.build_dir or Path(temporary)
        root.mkdir(parents=True, exist_ok=True)
        cpp, binary = root / 'reference_callback.cpp', root / 'reference_callback'
        cpp.write_text(HARNESS.replace('__CALLBACKS__', extracted))
        command = ['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
                   '-I' + str(args.include), '-I/usr/include/eigen3',
                   str(cpp), '-o', str(binary)]
        subprocess.run(command, check=True)
        print('Production callback source:', args.source, flush=True)
        result = subprocess.run([str(binary)], check=False, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            report = dict(schema=1, source=str(args.source),
                          source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                          production_helpers=str(args.include / 'plan_manage/input_contract.hpp'),
                          helpers_sha256=hashlib.sha256((args.include / 'plan_manage/input_contract.hpp').read_bytes()).hexdigest(),
                          callback_source=str(cpp), compile_command=command,
                          returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                          ros_initialized=False, robot_connected=False,
                          scope='actual extracted reference callbacks; clock, transport, publication and planning sink doubled; no map or optimizer validation')
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
