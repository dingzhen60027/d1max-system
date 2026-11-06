#!/usr/bin/env python3
"""Execute the production current-reader loop against passive lease instrumentation.

The native candidate-admission tests separately query the actual GridMap and
CollisionSnapshotPool. This seam runs the exact complete reader-loop ordering,
without ROS, and asks its diagnostic sink to publish while it is being called.
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
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <string>
#include <type_traits>
#define RCLCPP_INFO_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR_THROTTLE(...) ((void)0)
static std::atomic<unsigned> allocations{0};
void* operator new(std::size_t size) {
  allocations.fetch_add(1,std::memory_order_relaxed);
  if(auto* p=std::malloc(size))return p;
  throw std::bad_alloc();
}
void operator delete(void* p)noexcept {std::free(p);}
void operator delete(void* p,std::size_t)noexcept {std::free(p);}
struct GridMap {
  using Ptr=std::shared_ptr<GridMap>;
  std::int64_t integratedRaySourceStamp(int)const{return 100000000000LL;}
  std::string planningCollisionQueryPolicy()const{return "official";}
  void requireObservedSnapshot(){}
};
struct CollisionSnapshotPool {
  struct PublicationTiming {std::uint64_t publication_sequence=1,map_revision=1;};
  struct ReaderDiagnostics {
    std::uint64_t latest_publication_sequence=1,validation_borrows=0,validation_misses=0,
      validation_fallback_borrows=0,owner_borrows=0,owner_misses=0,publication_misses=0,
      solver_newer_publication_reuses=0;
  };
  GridMap::Ptr head=std::make_shared<GridMap>();
  GridMap::Ptr borrowValidation(PublicationTiming*){return head;}
  ReaderDiagnostics readerDiagnostics()const{return {};}
  bool writerCanPublish()const{return head.use_count()==1;}
};
bool validationReadRequiresObservedSnapshot(const std::string&,bool,const std::string&){return false;}
struct MeasuredBodyPose {};
struct BoundedExecutionTrace {static std::int64_t steadyNowNs(){return 1;}};
struct CurveCheckObservation {
  struct Event {std::int64_t queue_steady_ns=0,source_ns=0,rear_source_ns=0;
    struct Check {std::int64_t borrow_end_steady_ns=0,lease_end_steady_ns=0;
      struct Disposition {std::string value;void set(const char* p){value=p;}} disposition;
    } check;
  } event;
  explicit CurveCheckObservation(bool){}
};
namespace ew {struct MotionDemand {};}
enum class MotionCheckStage {SnapshotUnavailable};
struct MotionDiagnostics {void record(MotionCheckStage,const std::optional<ew::MotionDemand>&,std::int64_t){}};
struct ValidationCycle {
  using Clock=std::chrono::steady_clock;
  static constexpr auto period=std::chrono::milliseconds(50);
  static inline bool skip_query=false;
  template<class Check,class Now> static void formalCurrentCurvePass(Clock::time_point begin,Check check,Now){
    if(!skip_query)check(.12,begin+std::chrono::milliseconds(120));
  }
};
struct Node {struct Time{std::int64_t nanoseconds()const{return 100000000000LL;}};Time now()const{return {};}};
struct Reader {
  struct Job{int version=0;};
  struct CurveCheckTraceScope {
    Reader& owner;const Job* job;CurveCheckObservation& observation;
    void finish(){if(job){owner.trace_free=owner.pool_.writerCanPublish();
      owner.disposition=observation.event.check.disposition.value;job=nullptr;}}
    ~CurveCheckTraceScope(){finish();}
  };
  void captureCheckObservationLocked(CurveCheckObservation&,const std::optional<Job>&){}
  bool nativePipelineReadyLocked(int,std::int64_t){
    if(!pool_.writerCanPublish())throw std::logic_error("health sampling pinned a snapshot");
    return true;
  }
  static void observeSnapshot(CurveCheckObservation*,const CollisionSnapshotPool::PublicationTiming&){}
  std::atomic<bool> stop_{false};bool snapshot_ready_{true};
  std::mutex wait_mutex_,mutex_;std::condition_variable wake_;
  CollisionSnapshotPool pool_;std::optional<Job> active_{Job{}};MeasuredBodyPose body_;
  std::optional<ew::MotionDemand> latest_demand_;MotionDiagnostics motion_diagnostics_;
  std::string geometry_policy_="official_inflated_geometry";Node node;Node* node_=&node;
  bool query_owned=false,diagnostic_free=false,trace_free=false;std::string disposition;
  int completed_queries=0,diagnostic_publications=0;
  bool check(const Job&,const MeasuredBodyPose&,GridMap&,double,bool,ValidationCycle::Clock::time_point,
      const std::function<void()>& finish={},CurveCheckObservation* observation=nullptr){
    if(observation)observation->event.check.disposition.set("checked");
    query_owned=!pool_.writerCanPublish();++completed_queries;if(finish)finish();stop_=true;return true;
  }
  std::uint64_t slowSnapshotHintMisses()const{return 0;}
  void publishMotionDiagnostics(){
    stop_=true;
    diagnostic_free=pool_.writerCanPublish();
    // Two attempted publications while the outer diagnostic sink is active.
    // The passive second slot is notionally pinned by the owner throughout.
    for(int i=0;i<2;++i)if(pool_.writerCanPublish())++diagnostic_publications;
  }
__READER_METHODS__
};
int main(){Reader r;r.runLocalFollowerValidation();
  ValidationCycle::skip_query=true;Reader skipped;skipped.runLocalFollowerValidation();
  const bool skipped_ok=skipped.completed_queries==0&&skipped.disposition=="check_not_started"&&skipped.trace_free;
  Reader allocation_reader;CurveCheckObservation observation(true);
  observation.event.check.disposition.value.reserve(64); // Passive string stub, outside measured production helper.
  const auto before=allocations.load();
  allocation_reader.withCurrentCurveSnapshot([](GridMap&,const std::function<void()>& finish){finish();},&observation);
  const auto helper_allocations=allocations.load()-before;
  const bool ok=r.query_owned&&r.completed_queries==1&&r.diagnostic_free&&r.diagnostic_publications==2&&r.trace_free&&skipped_ok&&helper_allocations==0;
  std::cout<<"query_exclusive="<<r.query_owned<<" completed_queries="<<r.completed_queries
    <<" diagnostics_released="<<r.diagnostic_free<<" publications_during_diagnostics="<<r.diagnostic_publications
    <<" check_trace_released="<<r.trace_free<<" budget_skip_has_distinct_terminal="<<skipped_ok
    <<" exact_helper_heap_allocations="<<helper_allocations<<"\n";
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
    begin = text.find('  struct CurrentCurveSnapshotRead {')
    if begin < 0:
        begin = text.index('  void runLocalFollowerValidation() {')
    end = text.index('  void runAdmission()', begin)
    methods = text[begin:end]
    with tempfile.TemporaryDirectory(prefix='native-reader-scope-') as directory:
        cpp, binary = Path(directory)/'reader.cpp', Path(directory)/'reader'
        cpp.write_text(HARNESS.replace('__READER_METHODS__', methods))
        command = ['c++', '-std=c++17', '-pthread', str(cpp), '-o', str(binary)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
    print(result.stdout, end='')
    if args.report:
        args.report.write_text(json.dumps(dict(source=str(source),
            production_reader_sha256=hashlib.sha256(methods.encode()).hexdigest(),
            compile_command=command, returncode=result.returncode, stdout=result.stdout,
            ros_initialized=False, scope='Exact production reader loop and lease helper; passive map/query instrumentation, not geometry or timing evidence'), indent=2)+'\n')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
