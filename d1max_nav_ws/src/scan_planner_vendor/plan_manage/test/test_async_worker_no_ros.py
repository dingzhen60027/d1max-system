#!/usr/bin/env python3
"""Compile production worker completion/cancel methods; no ROS or fake planner.

The candidate/adoption decision is a deliberately passive seam. These tests
prove generation, deadline, cancellation and publication ordering only; native
snapshot/AStar/collision tests separately validate geometry implementation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

HARNESS = r'''
#include <plan_env/solve_budget.hpp>
#include <future>
#include <optional>
#include <iostream>
#include <string>
#include <stdexcept>
#include <new>
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
struct Time {double value=100.;double seconds() const{return value;}
  Time operator-(Time rhs) const{return {value-rhs.value};}};
namespace builtin_interfaces {namespace msg {using Time=::Time;}}
struct Node {Time now() const{return {};}};
namespace scan_planner {
struct Manager {
  struct SolvedCandidate {int id=18;};
  bool safe=true;int imports=0,adoptions=0,curve=17;
  void importAttemptDiagnostics(const Manager&) {++imports;}
  bool adoptSolvedCandidate(SolvedCandidate candidate,const SolveBudget::Ptr &budget) {
    if(!safe || !budget->allowed())return false;
    ++adoptions;curve=candidate.id;return true;
  }
  bool recheckPredecessor(int,double){return safe;}
};
class SCANReplanFSM {
public:
  struct WorkerResult {std::optional<Manager::SolvedCandidate> candidate;std::string failure;};
  std::future<WorkerResult> solve_future_;
  SolveBudget::Ptr active_solve_budget_;
  std::uint64_t solve_generation_=7,reference_generation_=7;
  bool have_target_=true,have_odom_=true,flag_escape_emergency_=false;
  Node clock;Node *node_=&clock;
  std::unique_ptr<Manager> planner_manager_=std::make_unique<Manager>();
  std::unique_ptr<Manager> solve_worker_=std::make_unique<Manager>();
  std::string last_attempt_failure_phase_;
  Time last_odom_time_{99.9};double odom_timeout_=.5;
  int solve_attempt_header_=1,replan_fail_count_=0,attempts=0,waits=0,publishes=0,transitions=0;
  std::uint64_t predecessor_id_=0;
  bool predecessor_safe_=false;
  Time predecessor_check_stamp_;
  std::optional<int> solve_predecessor_;
  int solve_predecessor_id_=0;double solve_predecessor_measured_time_=0.;
  enum {EXEC_TRAJ};
  void publishAttemptDebug(int){++attempts;}
  void waitForChangedEnvironment(){++waits;}
  void publishCommittedTrajectory(){++publishes;}
  void changeFSMExecState(int,const std::string&){++transitions;}
  void cancelSnapshotAcquisition(){} // Separate production acquisition seam is tested in recovery harness.
  void cancelSolveWorker();bool pollSolveWorker();
};
__METHODS__
}
int main(){
  using F=scan_planner::SCANReplanFSM;using B=scan_planner::SolveBudget;
  int checks=0,failures=0;
  auto expect=[&](bool ok,const char *name){++checks;failures+=!ok;
    std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";};
  auto setup=[](F&f,std::promise<F::WorkerResult>&p){
    f.active_solve_budget_=std::make_shared<B>();f.solve_future_=p.get_future();};
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    expect(f.pollSolveWorker() && f.waits==0 && f.publishes==0,"pending_is_not_failure_or_adoption");
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    expect(f.pollSolveWorker() && f.publishes==1 && f.planner_manager_->curve==18,
           "ready_current_valid_candidate_commits_once");
    expect(!f.pollSolveWorker() && f.publishes==1,"completion_is_consumed_once");
  }
  for(int mode=0;mode<3;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    if(mode==0)f.cancelSolveWorker();
    if(mode==1)++f.reference_generation_;
    if(mode==2)f.have_target_=false;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    expect(!f.pollSolveWorker() && f.publishes==0 && f.planner_manager_->imports==0 && f.waits==0,
           "cancelled_old_generation_or_retired_task_has_no_result_side_effect");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);auto now=B::Clock::now();
    f.active_solve_budget_=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    now+=std::chrono::milliseconds(400);p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    f.pollSolveWorker();expect(f.waits==1 && f.publishes==0 && f.planner_manager_->curve==17 &&
      f.last_attempt_failure_phase_=="solve_deadline_exceeded","late_success_keeps_incumbent_and_waits");
  }
  for(int mode=0;mode<3;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    if(mode==0)f.planner_manager_->safe=false;
    if(mode==1)f.last_odom_time_.value=98.;
    p.set_value({mode==2?std::nullopt:std::optional<scan_planner::Manager::SolvedCandidate>(scan_planner::Manager::SolvedCandidate{}),"failed_optimization"});
    f.pollSolveWorker();expect(f.publishes==0 && f.waits==1 && f.planner_manager_->curve==17,
      "changed_map_stale_pose_or_failed_candidate_never_overwrites_incumbent");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);const auto budget=f.active_solve_budget_;
    p.set_exception(std::make_exception_ptr(std::runtime_error("native solve error")));
    expect(f.pollSolveWorker() && f.waits==1 && f.publishes==0 &&
      f.planner_manager_->curve==17 && !f.active_solve_budget_ && !f.solve_future_.valid() &&
      budget->allowed() && f.last_attempt_failure_phase_=="solve_exception",
      "future_exception_retires_request_and_uses_existing_incumbent_recheck_path");
  }
  for(int mode=0;mode<2;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.solve_predecessor_=17;
    if(mode==0)++f.reference_generation_;else f.cancelSolveWorker();
    p.set_exception(std::make_exception_ptr(std::runtime_error("late old exception")));
    expect(!f.pollSolveWorker() && f.waits==0 && f.publishes==0 &&
      f.planner_manager_->imports==0 && !f.active_solve_budget_ && !f.solve_predecessor_,
      "retired_generation_or_cancelled_exception_has_no_diagnostics_or_recovery_side_effect");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);const auto budget=f.active_solve_budget_;
    f.solve_predecessor_=17;p.set_exception(std::make_exception_ptr(std::bad_alloc{}));
    bool propagated=false;try{f.pollSolveWorker();}catch(const std::bad_alloc&){propagated=true;}
    expect(propagated && !f.active_solve_budget_ && !f.solve_future_.valid() &&
      !f.solve_predecessor_ && !budget->allowed() && f.publishes==0 && f.waits==0,
      "allocation_failure_retires_resources_and_fail_stops_without_claiming_safe_recovery");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";return failures?1:0;
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    source=package/'src/scan_replan_fsm.cpp'
    text=source.read_text()
    start=text.index('  void SCANReplanFSM::cancelSolveWorker()')
    end=text.index('  void SCANReplanFSM::publishCommittedTrajectory()',start)
    methods=text[start:end]
    with tempfile.TemporaryDirectory(prefix='native-worker-completion-') as directory:
        cpp,binary=Path(directory)/'worker.cpp',Path(directory)/'worker'
        cpp.write_text(HARNESS.replace('__METHODS__',methods))
        command=['c++','-std=c++17','-pthread','-Wall','-Wextra',
                 '-I'+str(package.parent/'plan_env/include'),str(cpp),'-o',str(binary)]
        subprocess.run(command,check=True)
        result=subprocess.run([str(binary)],capture_output=True,text=True)
        print(result.stdout,end='');print(result.stderr,end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(source),
                production_method_sha256=hashlib.sha256(methods.encode()).hexdigest(),
                compile_command=command,returncode=result.returncode,stdout=result.stdout,
                ros_initialized=False,scope='actual native worker completion/cancel; passive adoption sink, not feasibility evidence'),indent=2)+'\n')
        return result.returncode

if __name__=='__main__':
    raise SystemExit(main())
