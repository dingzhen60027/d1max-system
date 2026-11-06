#!/usr/bin/env python3
"""Compile the actual no-motion reseed FSM method without initializing ROS."""
import argparse
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
struct Time {
  long long value=10100000000LL;
  Time operator-(Time b)const{return {value-b.value};}
  double seconds()const{return value*1e-9;}
  long long nanoseconds()const{return value;}
};
struct Node {Time now()const{return {};}};
struct Map {bool fresh=true;bool integratedCloudFreshAt(long long)const{return fresh;}};
struct Manager {std::shared_ptr<Map> grid_map_=std::make_shared<Map>();};
namespace scan_planner {
struct SCANReplanFSM {
  enum State{EXEC_TRAJ,REPLAN_TRAJ,WAIT_ENVIRONMENT};
  bool planning_only_=true,reference_path_guidance_=true,have_target_=true,have_odom_=true;
  State exec_state_=EXEC_TRAJ;
  Time last_odom_time_{10050000000LL};Node clock;Node* node_=&clock;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  std::chrono::steady_clock::time_point last_solve_submit_{};std::future<int> solve_future_;
  double min_replan_interval_=.1;
  std::string last_attempt_failure_phase_="unchanged";unsigned changes=0;
  unsigned reference_generation_=7;std::string route_id="same_route",task_id="same_task";
  void changeFSMExecState(State next,const std::string&){exec_state_=next;++changes;}
  bool tryReplanPlanningOnlyConnection(const NoMotionPreviewProgress&);
};
__METHOD__
}
int main(){
  using F=scan_planner::SCANReplanFSM;using S=scan_planner::NoMotionPreviewStatus;
  scan_planner::NoMotionPreviewProgress p;p.status=S::ReplanFromMeasuredBody;
  int checks=0,failed=0;const auto expect=[&](bool ok,const char* name){
    ++checks;failed+=!ok;std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";};
  {
    F f;expect(f.tryReplanPlanningOnlyConnection(p)&&f.exec_state_==F::REPLAN_TRAJ&&f.changes==1,
      "out_of_range_preview_requests_actual_body_solve_not_endless_exec_return");
    expect(f.reference_generation_==7&&f.route_id=="same_route"&&f.task_id=="same_task",
      "reseed_preserves_existing_route_reference_task_and_owner");
    expect(f.last_attempt_failure_phase_=="preview_connection_requires_measured_reseed",
      "typed_reseed_reason_is_explicit_not_occupied_or_tracker_ack");
    expect(!f.tryReplanPlanningOnlyConnection(p)&&f.changes==1,"no_repeat_while_replan_is_pending");
  }
  for(int boundary=0;boundary<10;++boundary){
    F f;auto q=p;
    if(boundary==0)f.planning_only_=false;
    if(boundary==1)f.reference_path_guidance_=false;
    if(boundary==2)f.have_target_=false;
    if(boundary==3)f.have_odom_=false;
    if(boundary==4)q.status=S::Connected;
    if(boundary==5)q.status=S::Uncertified;
    if(boundary==6)f.last_odom_time_.value=9000000000LL;
    if(boundary==7)f.planner_manager_->grid_map_->fresh=false;
    if(boundary==8)f.last_solve_submit_=std::chrono::steady_clock::now();
    if(boundary==9){std::promise<int> pending;f.solve_future_=pending.get_future();}
    expect(!f.tryReplanPlanningOnlyConnection(q)&&f.exec_state_==F::EXEC_TRAJ&&f.changes==0,
      "execution_missing_sources_certified_or_budget_busy_cannot_trigger_preview_reseed");
  }
  std::cout<<"checks="<<checks<<" failures="<<failed<<"\n";return failed?1:0;
}
'''


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'src/scan_replan_fsm.cpp')
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('  bool SCANReplanFSM::tryReplanPlanningOnlyConnection(')
    end = source.index('  SCANReplanFSM::PreviewRecheck ', start)
    # Also prove the actual EXEC_TRAJ branch handles the typed failure; the
    # isolated method cannot become dead code behind the former bare return.
    exec_branch = source[source.index('    case EXEC_TRAJ:'):source.index('    case EMERGENCY_STOP:')]
    assert 'tryReplanPlanningOnlyConnection(measured);' in exec_branch
    assert 'if(!measured_time)return;' not in exec_branch
    with tempfile.TemporaryDirectory(prefix='scan-preview-reseed-') as directory:
        cpp, executable = Path(directory)/'reseed.cpp', Path(directory)/'reseed'
        cpp.write_text(HARNESS.replace('__METHOD__', source[start:end]))
        subprocess.run(['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
            '-I'+str(root/'include'), '-I'+str(root.parent/'bspline_opt/include'),
            '-I/usr/include/eigen3', str(cpp), '-o', str(executable)], check=True)
        return subprocess.run([str(executable)], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
