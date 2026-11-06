#!/usr/bin/env python3
"""Compile the actual Native FSM arrival block, without ROS or robot motion.

Only immutable committed-goal lookup and settled-state evidence are passive
seams. The production condition and its terminal FSM mutations run verbatim.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <optional>
#include <memory>
#include <iostream>
struct Version { int task=1; };
Version executionVersion(Version v){return v;}
struct Node { struct Time {long nanoseconds()const{return 100000000000L;}};
  Time now()const{return {};}};
struct Ledger {
  std::optional<Eigen::Vector3d> goal{Eigen::Vector3d(0.,15.,.5)};
  bool settled=true;int task=1;
  std::optional<Eigen::Vector3d> committedTaskGoal(Version v)const{
    if(v.task!=task)return {};
    return goal;
  }
  bool measuredGoalSettled(Version v,long)const{return v.task==task&&settled;}
};
class FSM {
public:
  enum State {WAIT_TARGET,GEN_NEW_TRAJ,REPLAN_TRAJ,EXEC_TRAJ,WAIT_ENVIRONMENT};
  bool reference_path_guidance_=true,have_target_=true,have_odom_=true,local_follower_=true;
  State exec_state_=EXEC_TRAJ;
  Eigen::Vector3d odom_pos_{0.,4.,.5},end_pt_{0.,4.,.5};
  double reference_goal_xy_tolerance_=.2,reference_goal_z_tolerance_=.15;
  Version reference_metadata_;
  Node clock;Node* node_=&clock;
  std::shared_ptr<Ledger> execution_validator_=std::make_shared<Ledger>();
  int cancellations=0,completed=0;
  void cancelSolveWorker(){++cancellations;}
  void publishInvalidLocalPlanDebug(const char*){++completed;}
  int localPlanDebugHeader(){return 0;}
  void publishAttemptDebug(int,bool){}
  void changeFSMExecState(State s,const char*){exec_state_=s;}
  void arrival(){using scan_planner::measuredReferenceGoalReached;
__ARRIVAL__
  }
};
int main(){
  int checks=0,failures=0;
  auto expect=[&](bool v,const char*name){++checks;failures+=!v;
    std::cout<<(v?"PASS ":"FAIL ")<<name<<'\n';};
  for(auto state:{FSM::GEN_NEW_TRAJ,FSM::REPLAN_TRAJ,FSM::EXEC_TRAJ,FSM::WAIT_ENVIRONMENT}){
    FSM f;f.exec_state_=state;f.arrival();
    expect(f.have_target_&&f.exec_state_==state&&f.cancellations==0&&f.completed==0,
      "local_window_tail_is_not_task_completion");
  }
  {FSM f;f.execution_validator_->goal.reset();f.arrival();
    expect(f.have_target_&&f.completed==0,"missing_committed_goal_cannot_complete");}
  {FSM f;f.reference_metadata_.task=2;f.arrival();
    expect(f.have_target_&&f.completed==0,"old_task_goal_cannot_complete_new_task");}
  {FSM f;f.odom_pos_=f.end_pt_=Eigen::Vector3d(0.,15.,.5);f.arrival();
    expect(!f.have_target_&&f.exec_state_==FSM::WAIT_TARGET&&f.cancellations==1&&f.completed==1,
      "real_task_goal_and_settled_still_complete");}
  {FSM f;f.odom_pos_=f.end_pt_=Eigen::Vector3d(0.,15.,.5);
    f.execution_validator_->settled=false;f.arrival();
    expect(f.have_target_&&f.completed==0,"moving_at_task_goal_does_not_complete");}
  {FSM f;f.odom_pos_=f.end_pt_=Eigen::Vector3d(0.,15.,3.5);f.arrival();
    expect(f.have_target_&&f.completed==0,"same_xy_other_floor_does_not_complete");}
  {FSM f;f.local_follower_=false;f.arrival();
    expect(!f.have_target_&&f.completed==1,"legacy_point_goal_semantics_unchanged");}
  std::cout<<"checks="<<checks<<" failures="<<failures<<'\n';return failures?1:0;
}
'''


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'src/scan_replan_fsm.cpp')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('    // One measured XYZ criterion')
    end = source.index('    if (max_replan_interval_', start)
    block = source[start:end]
    with tempfile.TemporaryDirectory(prefix='scan-task-endpoint-') as directory:
        cpp, binary = Path(directory)/'arrival.cpp', Path(directory)/'arrival'
        cpp.write_text(HARNESS.replace('__ARRIVAL__', block))
        subprocess.run(['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
            '-I'+str(root/'include'), '-I/usr/include/eigen3', str(cpp), '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(args.source),
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                arrival_block_sha256=hashlib.sha256(block.encode()).hexdigest(),
                returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                ros_initialized=False, robot_connected=False), indent=2)+'\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
