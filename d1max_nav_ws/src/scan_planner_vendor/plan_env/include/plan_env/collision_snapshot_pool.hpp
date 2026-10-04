#pragma once
#include <plan_env/grid_map.h>
#include <plan_env/snapshot_read_arbiter.hpp>
#include <array>
#include <mutex>

namespace scan_planner {
// Exactly three native-map slots. The writer owns fusion; readers own exclusive
// query leases. A retained shared_ptr pins its slot and may NOT be copied to a
// second querying thread. Busy readers cause a bounded miss, never map mutation.
class CollisionSnapshotPool {
public:
  struct PublicationTiming {
    GridMap::FusionTiming fusion;
    std::uint64_t map_revision{0},publication_sequence{0},publication_misses{0};
    std::int64_t begin_steady_ns{0},end_steady_ns{0};
  };
  CollisionSnapshotPool() {
    for (auto &slot:slots_) slot=std::make_shared<GridMap>();
  }
  bool publish(const GridMap &writer,std::int64_t source_now_ns,
               std::chrono::steady_clock::time_point captured) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t n=1;n<=slots_.size();++n) {
      const auto i=(latest_+n)%slots_.size();
      if (slots_[i].use_count()!=1) continue;
      writer.copyCollisionSnapshotTo(*slots_[i],source_now_ns,captured);
      latest_=i;published_=true;return true;
    }
    return false;
  }
  GridMap::Ptr borrowLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!published_ || slots_[latest_].use_count()!=1) return {};
    return slots_[latest_];
  }
  // Execution uses fixed roles: slots 0/1 are validator double buffers, slot
  // 2 is shared EXCLUSIVELY by bounded solver / slow admission. A stuck reader cannot consume a validator
  // slot. Reservation is short; the large native copy never holds the mutex.
  bool publishValidation(const GridMap& writer,std::int64_t now,
                         std::chrono::steady_clock::time_point captured) {
    return publishRole(writer,now,captured,false);
  }
  bool publishSolver(const GridMap& writer,std::int64_t now,
                     std::chrono::steady_clock::time_point captured) {
    return publishRole(writer,now,captured,true);
  }
  GridMap::Ptr borrowValidation(PublicationTiming* timing=nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(validation_latest_<0 || filling_[validation_latest_] || slots_[validation_latest_].use_count()!=1) return {};
    if(timing)*timing=publication_timing_[validation_latest_];
    return slots_[validation_latest_];
  }
  // Request before trying to publish: a pinned slot must retain the other
  // reader's intent so another immediate solver cannot repeatedly overtake it.
  // Solver intent expires with that ORIGINAL solve deadline, not a new budget.
  void requestSolver(std::chrono::steady_clock::time_point deadline) {
    std::lock_guard<std::mutex> lock(mutex_);
    slow_readers_.request(SnapshotReadArbiter::Reader::Solver,deadline,std::chrono::steady_clock::now());
  }
  void cancelSolverRequest() {
    std::lock_guard<std::mutex> lock(mutex_);
    slow_readers_.cancel(SnapshotReadArbiter::Reader::Solver);
  }
  // Owner-thread acquisition is one fair reservation/copy/lease transaction.
  // On contention do not copy 32 MB only to be denied by the arbiter. Copies
  // retain the writer's ORIGINAL ray/body source stamps, not acquisition time.
  GridMap::Ptr acquireSolver(const GridMap& writer,std::int64_t source_now_ns,
      std::chrono::steady_clock::time_point deadline) {
    const auto captured=std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slow_readers_.request(SnapshotReadArbiter::Reader::Solver,deadline,captured);
      if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Solver,captured,
          !filling_[2]&&slots_[2].use_count()==1))return {};
      filling_[2]=true;
    }
    try {writer.copyCollisionSnapshotTo(*slots_[2],source_now_ns,captured);}
    catch(...) {std::lock_guard<std::mutex> lock(mutex_);filling_[2]=false;throw;}
    std::lock_guard<std::mutex> lock(mutex_);
    filling_[2]=false;solver_ready_=true;
    if(std::chrono::steady_clock::now()>=deadline)return {};
    return slots_[2];
  }
  GridMap::Ptr borrowSolver(std::chrono::steady_clock::time_point deadline=
      std::chrono::steady_clock::now()+std::chrono::milliseconds(400)) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=std::chrono::steady_clock::now();
    slow_readers_.request(SnapshotReadArbiter::Reader::Solver,deadline,now);
    if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Solver,now,
          solver_ready_&&!filling_[2]&&slots_[2].use_count()==1))return {};
    return slots_[2];
  }
  GridMap::Ptr borrowAdmission() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=std::chrono::steady_clock::now();
    slow_readers_.request(SnapshotReadArbiter::Reader::Admission,
        std::chrono::steady_clock::time_point::max(),now);
    if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Admission,now,
          solver_ready_&&!filling_[2]&&slots_[2].use_count()==1))return {};
    return slots_[2];
  }
  void cancelAdmissionRequest() {
    std::lock_guard<std::mutex> lock(mutex_);
    slow_readers_.cancel(SnapshotReadArbiter::Reader::Admission);
  }
private:
  bool publishRole(const GridMap& writer,std::int64_t now,
                   std::chrono::steady_clock::time_point captured,bool solver) {
    int chosen=-1;
    const auto publication_begin=std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(solver) {if(!filling_[2] && slots_[2].use_count()==1) chosen=2;}
      else for(int n=1;n<=2;++n) {
        const int i=(std::max(0,validation_latest_)+n)%2;
        if(!filling_[i]&&slots_[i].use_count()==1) {chosen=i;break;}
      }
      if(chosen<0) {if(!solver)++validation_publication_misses_;return false;}
      filling_[chosen]=true;
    }
    try {writer.copyCollisionSnapshotTo(*slots_[chosen],now,captured);}
    catch(...) {std::lock_guard<std::mutex> lock(mutex_);filling_[chosen]=false;throw;}
    std::lock_guard<std::mutex> lock(mutex_);
    auto& timing=publication_timing_[chosen];
    timing.fusion=writer.fusionTiming();timing.map_revision=writer.occupancyRevision();
    timing.publication_sequence=++publication_sequence_;
    timing.publication_misses=validation_publication_misses_;
    timing.begin_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(publication_begin.time_since_epoch()).count();
    timing.end_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    filling_[chosen]=false;
    if(solver) solver_ready_=true;else validation_latest_=chosen;
    return true;
  }
  std::array<GridMap::Ptr,3> slots_;
  std::array<PublicationTiming,3> publication_timing_;
  std::uint64_t publication_sequence_{0},validation_publication_misses_{0};
  std::mutex mutex_;
  std::size_t latest_{2};
  bool published_{false};
  std::array<bool,3> filling_{{false,false,false}};
  int validation_latest_{-1};
  bool solver_ready_{false};
  SnapshotReadArbiter slow_readers_;
};
}  // namespace scan_planner
