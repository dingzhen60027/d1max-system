#!/usr/bin/env python3
"""Run exact production writer/hint methods with a deliberately held ledger mutex.

No map geometry is substituted for planning. The passive publication sink only
records ordering; native GridMap fixtures test the independent data invariants.
The one-second wait is a deadlock-test bound, never a performance measurement.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <iostream>
struct ValidationCycle {static bool admissionWorkWanted(bool,bool,bool pending){return pending;}};
struct Validator {
  struct Job{};mutable std::mutex mutex_;mutable std::atomic<std::uint64_t> slow_snapshot_hint_misses_{0};
  bool local_follower_{false},grant_retired_{false},grant_applied_{false};
  std::optional<Job> active_{Job{}},candidate_,prepared_;
  static bool sameJob(const Job&,const Job&){return true;}
  unsigned snapshot_wakes{0},admission_wakes{0};
  void snapshotPublished(){++snapshot_wakes;}
  void admissionSnapshotPublished(){++admission_wakes;}
__HINT_METHODS__
};
struct Node {struct Time{std::int64_t nanoseconds()const{return 100000000000LL;}};Time now()const{return {};}};
namespace scan_planner {
struct CollisionSnapshotPool {enum class SolverPublication {Deferred,Unchanged,Published};};
struct SCANReplanFSM {
  Node node;Node*node_=&node;
  std::unique_ptr<Validator> execution_validator_=std::make_unique<Validator>();
  std::shared_ptr<int> grid_map_writer_=std::make_shared<int>(17);
  bool map_writer_group_=true,local_follower_=true;
  bool map_validation_publication_pending_=false,map_solver_publication_pending_=false;
  std::future<void> solve_future_;void pollSolveWorker(){}
  struct Pool {
    std::promise<void> fast_published;unsigned heads{0},slow{0};int head_source=0,slow_source=0;
    bool publishValidation(int source,std::int64_t,std::chrono::steady_clock::time_point){
      head_source=source;if(++heads==1)fast_published.set_value();return true;}
    bool refreshOwnerReplica(int,std::int64_t,std::chrono::steady_clock::time_point){return false;}
    bool publishSolver(int source,std::int64_t,std::chrono::steady_clock::time_point){
      slow_source=source;++slow;return true;}
    CollisionSnapshotPool::SolverPublication refreshSolverPublication(int source,std::int64_t,std::chrono::steady_clock::time_point){
      if(slow_source==source)return CollisionSnapshotPool::SolverPublication::Unchanged;
      slow_source=source;++slow;return CollisionSnapshotPool::SolverPublication::Published;}
  }snapshot_pool_;
  void publishMapSnapshots();
  void completedMapWriterTransaction(){__FUSION_CALLBACK__}
};
__PUBLISH_METHOD__
}
int main(){
  scan_planner::SCANReplanFSM f;
  std::unique_lock<std::mutex> ledger(f.execution_validator_->mutex_);
  auto fast=f.snapshot_pool_.fast_published.get_future();std::promise<void> started;
  auto writer=std::async(std::launch::async,[&]{started.set_value();f.completedMapWriterTransaction();});
  started.get_future().wait();
  const bool fast_without_ledger=fast.wait_for(std::chrono::seconds(1))==std::future_status::ready;
  const bool finished_without_ledger=fast_without_ledger&&
    writer.wait_for(std::chrono::seconds(1))==std::future_status::ready;
  const bool retry_retained=finished_without_ledger&&f.map_solver_publication_pending_;
  // There is no sleep/racy observation of the slot. The writer sink fulfills
  // its promise only after the exact production publication branch runs.
  ledger.unlock();writer.get();
  f.publishMapSnapshots();
  const bool same_source=f.snapshot_pool_.head_source==17&&f.snapshot_pool_.slow_source==17;
  const bool completed_once=f.snapshot_pool_.heads==1&&f.snapshot_pool_.slow==1&&
    f.execution_validator_->snapshot_wakes==1&&f.execution_validator_->admission_wakes==1;
  // A new complete map arrived with no distinct admission work. The next
  // candidate arrives AFTER this callback, without another lidar frame.
  f.execution_validator_->active_.reset();*f.grid_map_writer_=18;
  f.completedMapWriterTransaction();
  const bool no_work_does_not_copy=f.snapshot_pool_.slow==1&&!f.map_solver_publication_pending_;
  f.execution_validator_->active_=Validator::Job{};
  f.publishMapSnapshots();
  const bool late_work_delivered=f.snapshot_pool_.slow==2&&f.snapshot_pool_.slow_source==18&&
    f.execution_validator_->admission_wakes==2;
  for(unsigned i=0;i<100;++i)f.publishMapSnapshots();
  const bool repeated_work_deduplicated=f.snapshot_pool_.slow==2&&f.execution_validator_->admission_wakes==2;
  std::cout<<"fast_published_while_ledger_held="<<fast_without_ledger
    <<" writer_returned_while_ledger_held="<<finished_without_ledger<<" retry_retained="<<retry_retained
    <<" same_source="<<same_source<<" completed_once="<<completed_once
    <<" no_work_does_not_copy="<<no_work_does_not_copy<<" late_work_delivered="<<late_work_delivered
    <<" repeated_work_deduplicated="<<repeated_work_deduplicated<<"\n";
  return fast_without_ledger&&finished_without_ledger&&retry_retained&&same_source&&completed_once&&
    no_work_does_not_copy&&late_work_delivered&&repeated_work_deduplicated?0:1;
}
'''


def function(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fsm', type=Path)
    parser.add_argument('--validator', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    fsm = args.fsm or package/'src/scan_replan_fsm.cpp'
    validator = args.validator or package/'include/plan_manage/execution_validator.hpp'
    source = fsm.read_text()
    header = validator.read_text()
    begin = source.index('setCollisionUpdateCallback([this]{')+len('setCollisionUpdateCallback([this]{')
    callback = source[begin:source.index('      });', begin)]
    publish = function(source, '  void SCANReplanFSM::publishMapSnapshots()')
    hints = function(header, '  bool slowSnapshotWanted() const')
    if '  std::optional<bool> trySlowSnapshotWanted() const' in header:
        hints += '\n'+function(header, '  std::optional<bool> trySlowSnapshotWanted() const')
        hints += '\n'+function(header, '  bool slowSnapshotWantedLocked() const')
    rendered = HARNESS.replace('__FUSION_CALLBACK__', callback).replace('__PUBLISH_METHOD__', publish).replace('__HINT_METHODS__', hints)
    with tempfile.TemporaryDirectory(prefix='native-writer-handoff-') as directory:
        cpp, binary = Path(directory)/'writer.cpp', Path(directory)/'writer'
        cpp.write_text(rendered)
        command = ['c++', '-std=c++17', '-pthread', str(cpp), '-o', str(binary)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
    print(result.stdout, end='')
    if args.report:
        args.report.write_text(json.dumps(dict(fsm=str(fsm), validator=str(validator),
            production_method_sha256=hashlib.sha256((callback+publish+hints).encode()).hexdigest(),
            compile_command=command, returncode=result.returncode, stdout=result.stdout,
            ros_initialized=False, scope='Exact production writer callback/publication and real ledger mutex hint methods; passive map publication sink; deterministic ordering, not latency evidence'), indent=2)+'\n')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
