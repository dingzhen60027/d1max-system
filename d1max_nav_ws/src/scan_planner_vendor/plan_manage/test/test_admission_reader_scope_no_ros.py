#!/usr/bin/env python3
"""Run the exact production admission loop with passive lease/sink instrumentation.

The paired C++ tests exercise actual GridMap/CollisionSnapshotPool geometry.
This seam checks that production wiring releases its lease before the output
sink, including exceptions and the archived two-query reader lifecycle.
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
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#define RCLCPP_ERROR_THROTTLE(...) ((void)0)
struct GridMap {using Ptr=std::shared_ptr<GridMap>;void requireObservedSnapshot(){}
  struct FusionTiming{};FusionTiming fusionTiming()const{return {};}
  std::uint64_t occupancyRevision()const{return 1;}
  std::int64_t integratedRaySourceStamp(int)const{return 100000000000LL;}
};
struct CollisionSnapshotPool {
  struct PublicationTiming {GridMap::FusionTiming fusion;std::uint64_t map_revision=1;};
  GridMap::Ptr head=std::make_shared<GridMap>();bool cancelled=false;
  GridMap::Ptr borrowAdmission(){return head;}
  void cancelAdmissionRequest(){cancelled=true;}
  bool solverCanAcquire()const{return head.use_count()==1;}
};
bool validationReadRequiresObservedSnapshot(const std::string&,bool){return false;}
struct MeasuredBodyPose {};
struct BoundedExecutionTrace {static std::int64_t steadyNowNs(){return 1;}};
struct CurveCheckObservation {
  struct Event {std::int64_t queue_steady_ns=0,source_ns=0,rear_source_ns=0;
    struct Check {std::int64_t borrow_end_steady_ns=0,lease_end_steady_ns=0;
      struct Disposition {void set(const char*){}} disposition;
    } check;
  } event;
  explicit CurveCheckObservation(bool){}
};
struct ValidationCycle {
  using Clock=std::chrono::steady_clock;
  static bool admissionWorkWanted(bool formal,bool active,bool distinct){return distinct||(!formal&&active);}
  template<class Check,class Now>static void formalAdmissionPass(Clock::time_point b,Check check,Now){check(.12,b+std::chrono::milliseconds(120));}
  template<class Active,class Pending,class Now>static void admissionPass(Clock::time_point b,Active a,Pending p,Now){
    a(.1,b+std::chrono::milliseconds(120));p(.02,b+std::chrono::milliseconds(120));}
};
struct Node {struct Time{std::int64_t nanoseconds()const{return 100000000000LL;}};Time now()const{return {};}};
struct Reader {
  struct Job{int id;};
  struct CurveCheckTraceScope {Reader& owner;const Job* job;CurveCheckObservation& observation;};
  void captureCheckObservationLocked(CurveCheckObservation&,const std::optional<Job>&){}
  static void observeSnapshot(CurveCheckObservation*,const CollisionSnapshotPool::PublicationTiming&){}
  std::atomic<bool> stop_{false};std::mutex mutex_,admission_wait_mutex_;
  std::condition_variable admission_wake_;bool admission_ready_=true;
  bool local_follower_=true,grant_retired_=false,grant_applied_=false;
  std::optional<Job>active_{Job{1}},candidate_{Job{2}},prepared_;
  MeasuredBodyPose body_;CollisionSnapshotPool pool_;
  std::string geometry_policy_="official_inflated_geometry";Node node;Node*node_=&node;
  bool throw_sink=false,queries_owned=true,sinks_free=true;
  int queries=0,sinks=0;bool legacy_both_queries_owned=true;
  static bool sameJob(const Job&a,const Job&b){return a.id==b.id;}
  void expireCandidateOfferLocked(std::int64_t){if(!candidate_)stop_=true;}
  void publishSelectedCandidate(){if(!candidate_)stop_=true;}
  bool check(const Job&,const MeasuredBodyPose&,GridMap&,double,bool,ValidationCycle::Clock::time_point,
      const std::function<void()>& finish={},CurveCheckObservation* =nullptr){
    queries_owned=queries_owned&&!pool_.solverCanAcquire();++queries;
    if(finish)finish();
    sinks_free=sinks_free&&pool_.solverCanAcquire();++sinks;
    if(local_follower_||queries==2)stop_=true;
    if(throw_sink)throw std::runtime_error("publisher failure");
    return true;
  }
__ADMISSION_METHODS__
};
int main(){
  bool ok=true;
  for(bool throwing:{false,true}) {
    Reader r;r.throw_sink=throwing;r.runAdmission();
    bool pass=r.queries_owned&&r.queries==1&&r.sinks_free&&r.pool_.solverCanAcquire();
    std::cout<<"formal=1 throwing="<<throwing<<" exclusive_query="<<r.queries_owned
      <<" sink_released="<<r.sinks_free<<" final_released="<<r.pool_.solverCanAcquire()<<"\n";
    ok=ok&&pass;
  }
  Reader legacy;legacy.local_follower_=false;legacy.runAdmission();
  // Archived active+pending share the original snapshot for both queries.
  const bool legacy_ok=legacy.queries==2&&legacy.queries_owned&&!legacy.sinks_free&&legacy.pool_.solverCanAcquire();
  std::cout<<"legacy_two_queries="<<legacy.queries<<" queries_owned="<<legacy.queries_owned
    <<" original_scope_preserved="<<legacy_ok<<"\n";ok=ok&&legacy_ok;
  Reader idle;idle.candidate_.reset();idle.runAdmission();
  const bool idle_ok=idle.queries==0&&idle.pool_.cancelled&&idle.pool_.solverCanAcquire();
  std::cout<<"idle_withdraws_request="<<idle_ok<<"\n";ok=ok&&idle_ok;
  return ok?0:1;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    source = args.source or package/'include/plan_manage/execution_validator.hpp'
    text = source.read_text()
    begin = text.index('  void runAdmission() {')
    end = text.index('  static std::int64_t ns(', begin)
    methods = text[begin:end]
    with tempfile.TemporaryDirectory(prefix='native-admission-scope-') as directory:
        cpp, binary = Path(directory)/'reader.cpp', Path(directory)/'reader'
        cpp.write_text(HARNESS.replace('__ADMISSION_METHODS__', methods))
        command = ['c++', '-std=c++17', '-pthread', str(cpp), '-o', str(binary)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
    print(result.stdout, end='')
    if args.report:
        args.report.write_text(json.dumps(dict(source=str(source),
            production_admission_sha256=hashlib.sha256(methods.encode()).hexdigest(),
            compile_command=command, returncode=result.returncode, stdout=result.stdout,
            ros_initialized=False,
            scope='Exact production admission loop; passive lease instrumentation, not geometry or physical timing evidence'), indent=2)+'\n')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
