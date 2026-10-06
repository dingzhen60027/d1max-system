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
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#define RCLCPP_ERROR(...) ((void)0)
struct TimeValue { double value=100.; double seconds() const {return value;}
  std::int64_t nanoseconds() const {return std::llround(value*1e9);}
  TimeValue operator-(TimeValue b) const {return {value-b.value};} };
namespace rclcpp {using Time=TimeValue;}
struct Node { TimeValue stamp; TimeValue now() const {return stamp;} };
struct Curve { Eigen::Vector3d value{.4,.2,.1};
  Eigen::Vector3d slope{.1,0.,0.};
  Eigen::Vector3d evaluateDeBoorT(double t) const {return value+t*slope;} };
namespace scan_planner {
struct LocalTrajData {
  TimeValue start_time_{99.}; double duration_=5.; int traj_id_=17;
  Curve velocity_traj_, acceleration_traj_;
  Curve position_traj_{{.9,2.,.55},{.1,0.,0.}};
};
struct Manager {
  struct Map {double getResolution() const{return .08;}};
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  LocalTrajData local_data_; int global_calls=0;
  bool planGlobalTraj(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,
      const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&) {
    ++global_calls; return true;
  }
};
class SCANReplanFSM {
 public:
  enum NAVI_MODE {REFERENCE_PATH=3};
  Node clock; Node *node_=&clock;
  struct Validator {
    struct Boundary {Eigen::Vector3d velocity,acceleration;};
    std::optional<Boundary> value;
    std::optional<Boundary> committedBoundary(const Eigen::Vector3d&,double,std::int64_t)const {return value;}
  };
  std::shared_ptr<Validator> execution_validator_;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  bool go2_execution_frozen_=false,reference_path_guidance_=true;
  bool require_tagged_reference_=true,have_new_target_=false;
  int navi_mode_=3,accepted_curve_id_=17;
  struct Progress {bool valid=true,holding=false; int trajectory_id=17;
    std::uint64_t generation=7;double curve_time=1.;struct {TimeValue stamp;} header;};
  std::optional<Progress> tracking_progress_{Progress{}};
  double odom_timeout_=.5;
  std::uint64_t accepted_curve_generation_=7,reference_generation_=7;
  Eigen::Vector3d odom_pos_{1.,2.,.55},odom_vel_{.02,0.,0.},end_pt_{-10.,2.,.55};
  Eigen::Vector3d start_pt_,start_vel_,start_acc_;
  Eigen::Vector3d solved_position,solved_velocity,solved_acceleration;
  int solves=0; bool solve_result=true,target_valid=true;
  bool adjustGlobalTargetIfOccupied(){return target_valid;}
  bool callReboundReplan(bool,bool) {
    ++solves; solved_position=start_pt_;solved_velocity=start_vel_;solved_acceleration=start_acc_;
    return solve_result;
  }
  bool planFromCurrentTraj();
  void setStartStateFromOdomOrCurrentTraj();
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
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->value=FSM::Validator::Boundary{{.12,.03,0.},{.02,0.,0.}};
    f.planner_manager_->local_data_.velocity_traj_.value={9.,9.,9.};
    expect(f.planFromCurrentTraj()&&f.solved_velocity.isApprox(Eigen::Vector3d(.12,.03,0.)),
      "execution_handoff_uses_committed_ledger_not_latest_candidate");
    f.execution_validator_->value.reset();f.planFromCurrentTraj();
    expect(measured(f),"missing_committed_proof_keeps_real_measured_boundary");
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
    start = source.index('  bool SCANReplanFSM::planFromCurrentTraj()')
    end = source.index('  void SCANReplanFSM::trackingProgressCallback(', start)
    production_methods = source[start:end]
    with tempfile.TemporaryDirectory(prefix='scan-replan-handoff-') as directory:
        cpp, executable = Path(directory)/'handoff.cpp', Path(directory)/'handoff'
        cpp.write_text(HARNESS.replace('__CALLBACKS__', production_methods))
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
