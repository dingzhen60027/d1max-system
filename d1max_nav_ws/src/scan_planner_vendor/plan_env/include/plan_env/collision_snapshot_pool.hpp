#pragma once
#include <plan_env/grid_map.h>
#include <plan_env/snapshot_read_arbiter.hpp>
#include <array>
#include <mutex>
#include <optional>

namespace scan_planner {
// Exactly three native-map slots. The writer owns fusion; readers own exclusive
// query leases. A retained shared_ptr pins its slot and may NOT be copied to a
// second querying thread. Busy readers cause a bounded miss, never map mutation.
class CollisionSnapshotPool {
public:
  enum class SolverPublication { Deferred, Unchanged, Published };
  struct PublicationTiming {
    GridMap::FusionTiming fusion;
    std::uint64_t map_revision{0},publication_sequence{0},publication_misses{0};
    std::uint64_t context_sequence{0};
    std::int64_t cloud_source_ns{0};
    std::array<std::int64_t,2> ray_source_ns{{0,0}};
    bool clock_fault{false};
    std::int64_t begin_steady_ns{0},end_steady_ns{0};
  };
  struct ReaderDiagnostics {
    std::uint64_t validation_borrows{0},validation_misses{0},validation_fallback_borrows{0};
    std::uint64_t owner_borrows{0},owner_misses{0},publication_misses{0};
    std::uint64_t solver_newer_publication_reuses{0};
    std::uint64_t latest_publication_sequence{0};
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
  // Execution uses fixed roles: slots 0/1 are exclusive validator/owner views, slot
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
  // Work can arrive AFTER the latest fusion completed. The existing serial
  // writer retry delivers that completed state on demand, without requiring
  // another sensor frame or repeatedly copying the same 86 MB snapshot.
  SolverPublication refreshSolverPublication(const GridMap& writer,std::int64_t now,
                     std::chrono::steady_clock::time_point captured) {
    bool unchanged=false;
    if(publishRole(writer,now,captured,true,&unchanged))return SolverPublication::Published;
    return unchanged?SolverPublication::Unchanged:SolverPublication::Deferred;
  }
  // Deliver the same completed writer state to the owner's second validation
  // slot after a short reader lease releases it. Publishing the fast head is
  // not completion of this delivery: otherwise repeatedly overwriting that
  // head while the owner is busy strands the backup two sensor frames behind.
  // Called by the existing serial writer retry; no pending map or fourth slot.
  // A matching source/revision is a no-op, including when it is borrowed.
  bool refreshOwnerReplica(const GridMap& writer,std::int64_t now,
                           std::chrono::steady_clock::time_point captured) {
    int chosen=-1;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(validation_latest_<0 || filling_[validation_latest_] ||
         !publicationMatchesWriter(publication_timing_[validation_latest_],writer))return false;
      chosen=1-validation_latest_;
      if(publicationMatchesWriter(publication_timing_[chosen],writer) ||
         filling_[chosen] || slots_[chosen].use_count()!=1)return false;
      filling_[chosen]=true;
    }
    try {writer.copyCollisionSnapshotTo(*slots_[chosen],now,captured);}
    catch(...) {std::lock_guard<std::mutex> lock(mutex_);filling_[chosen]=false;throw;}
    std::lock_guard<std::mutex> lock(mutex_);
    recordPublication(chosen,writer,captured);
    // This copy completes the owner's delivery of the existing head; it is
    // not a newer observation. Promoting it swaps roles underneath a still
    // running fast reader and makes the owner compete for that reader's slot.
    filling_[chosen]=false;
    return true;
  }
  GridMap::Ptr borrowValidation(PublicationTiming* timing=nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Fusion completed while both readers held their slots. If the fast
    // reader finishes first, the still-borrowed backup cannot receive that
    // pending map. Reborrowing the old head here would consume the writer's
    // only free destination for another full check of the same old sources.
    // Reserve that head for the existing serial writer retry; publication
    // wakes this reader. When backup is free, it remains the writer's reserved
    // destination and the fast head stays available as before. No occupied
    // buffer is overwritten and no original proof/source lease is extended.
    if(validation_writer_pending_&&validation_latest_>=0) {
      const int backup=1-validation_latest_;
      if(filling_[backup]||slots_[backup].use_count()!=1) {
        ++reader_diagnostics_.validation_misses;
        return {};
      }
    }
    int chosen=-1;
    for(int offset=0;validation_latest_>=0&&offset<2;++offset) {
      const int i=(validation_latest_+offset)%2;
      if(filling_[i] || slots_[i].use_count()!=1 ||
         (validation_writer_pending_ && i!=validation_latest_) ||
         !matchesValidationIdentity(publication_timing_[i]))continue;
      // Prefer the real head. The backup can have a later COPY sequence
      // without newer geometry/rays; use it only while the head is busy.
      chosen=i;break;
    }
    if(chosen<0) {++reader_diagnostics_.validation_misses;return {};}
    ++reader_diagnostics_.validation_borrows;
    if(chosen!=validation_latest_)++reader_diagnostics_.validation_fallback_borrows;
    if(timing)*timing=publication_timing_[chosen];
    return slots_[chosen];
  }
  // The fast current-curve reader owns the newest completed validation head.
  // The 100 Hz FSM must not pin that head for a longer geometry transaction
  // and prevent the 20 Hz renewal from seeing it. Its exclusive query lease
  // uses only the other ALREADY PUBLISHED validation slot. Startup therefore
  // waits for two completed copies (which may share one real fusion source);
  // it never invents a second observation or borrows an uninitialized map.
  // Geometry/source ages still belong to the selected snapshot, not this lease.
  GridMap::Ptr borrowOwnerValidation(PublicationTiming* timing=nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(validation_latest_<0) {++reader_diagnostics_.owner_misses;return {};}
    int chosen=-1;
    for(int i=0;i<2;++i) {
      if(i==validation_latest_ || validation_writer_pending_ || filling_[i] || !publication_timing_[i].publication_sequence ||
         slots_[i].use_count()!=1)continue;
      // An older revision of the SAME map context is an honest completed
      // view; a pre-reset or pre-fault view is a different source identity.
      if(!matchesValidationIdentity(publication_timing_[i]))continue;
      if(chosen<0 || publication_timing_[i].publication_sequence>
                     publication_timing_[chosen].publication_sequence)chosen=i;
    }
    if(chosen<0) {++reader_diagnostics_.owner_misses;return {};}
    ++reader_diagnostics_.owner_borrows;
    if(timing)*timing=publication_timing_[chosen];
    return slots_[chosen];
  }
  ReaderDiagnostics readerDiagnostics() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result=reader_diagnostics_;
    result.publication_misses=validation_publication_misses_;
    if(validation_latest_>=0)result.latest_publication_sequence=
      publication_timing_[validation_latest_].publication_sequence;
    return result;
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
      if(!matchesValidationIdentity(writer.localizationContextSequence(),writer.rayEvidenceClockFault()))return {};
      if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Solver,captured,
          !filling_[2]&&slots_[2].use_count()==1))return {};
      // The owner's backup is deliberately not the fast validation head.
      // A writer may already have supplied a newer complete third-slot map.
      // Do not overwrite it with the older owner's copy merely to acquire a
      // solver lease. Reuse keeps the original clock/source/publication fact;
      // both geometry revision and every original source must be at least as
      // new, and the exclusive grant above still arbitrates admission fairly.
      if(solver_ready_&&matchesValidationIdentity(publication_timing_[2])&&
         slots_[2]->occupancyRevision()>=writer.occupancyRevision()&&
         slots_[2]->latestCloudStampNs()>=writer.latestCloudStampNs()&&
         slots_[2]->integratedRaySourceStamp(0)>=writer.integratedRaySourceStamp(0)&&
         slots_[2]->integratedRaySourceStamp(1)>=writer.integratedRaySourceStamp(1)) {
        ++reader_diagnostics_.solver_newer_publication_reuses;
        return slots_[2];
      }
      filling_[2]=true;
    }
    try {writer.copyCollisionSnapshotTo(*slots_[2],source_now_ns,captured);}
    catch(...) {std::lock_guard<std::mutex> lock(mutex_);filling_[2]=false;throw;}
    std::lock_guard<std::mutex> lock(mutex_);
    recordPublication(2,writer,captured);
    filling_[2]=false;solver_ready_=true;
    if(std::chrono::steady_clock::now()>=deadline || !matchesValidationIdentity(publication_timing_[2]))return {};
    return slots_[2];
  }
  GridMap::Ptr borrowSolver(std::chrono::steady_clock::time_point deadline=
      std::chrono::steady_clock::now()+std::chrono::milliseconds(400)) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=std::chrono::steady_clock::now();
    slow_readers_.request(SnapshotReadArbiter::Reader::Solver,deadline,now);
    if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Solver,now,
          solver_ready_&&!filling_[2]&&slots_[2].use_count()==1&&
          matchesValidationIdentity(publication_timing_[2])))return {};
    return slots_[2];
  }
  GridMap::Ptr borrowAdmission() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=std::chrono::steady_clock::now();
    // A finished solver releases its old input, not a current admission map.
    // Capture the already completed head once per pending reader turn. Do not
    // chase every subsequent fusion while waiting: a finite required version
    // preserves fair progress under continuous publication. The writer may
    // deliver a newer complete version; original source times are untouched.
    if(!admission_floor_ && validation_latest_>=0&&
       matchesValidationIdentity(publication_timing_[validation_latest_]))
      admission_floor_=publication_timing_[validation_latest_];
    slow_readers_.request(SnapshotReadArbiter::Reader::Admission,
        std::chrono::steady_clock::time_point::max(),now);
    if(!slow_readers_.grant(SnapshotReadArbiter::Reader::Admission,now,
          solver_ready_&&!filling_[2]&&slots_[2].use_count()==1&&
          matchesValidationIdentity(publication_timing_[2])&&
          (!admission_floor_||publicationAtLeast(publication_timing_[2],*admission_floor_))))return {};
    admission_floor_.reset();
    return slots_[2];
  }
  void cancelAdmissionRequest() {
    std::lock_guard<std::mutex> lock(mutex_);
    slow_readers_.cancel(SnapshotReadArbiter::Reader::Admission);
    admission_floor_.reset();
  }
private:
  bool matchesValidationIdentity(std::uint64_t context,bool fault)const {
    return !validation_identity_known_ ||
      (context==validation_context_sequence_&&fault==validation_clock_fault_);
  }
  bool matchesValidationIdentity(const PublicationTiming& publication)const {
    return publication.publication_sequence &&
      matchesValidationIdentity(publication.context_sequence,publication.clock_fault);
  }
  static bool publicationMatchesWriter(const PublicationTiming& publication,const GridMap& writer) {
    return publication.publication_sequence &&
      publication.map_revision==writer.occupancyRevision() &&
      publication.fusion.sequence==writer.fusionTiming().sequence &&
      publication.context_sequence==writer.localizationContextSequence() &&
      publication.clock_fault==writer.rayEvidenceClockFault() &&
      publication.cloud_source_ns==writer.latestCloudStampNs() &&
      publication.ray_source_ns[0]==writer.integratedRaySourceStamp(0) &&
      publication.ray_source_ns[1]==writer.integratedRaySourceStamp(1);
  }
  static bool publicationAtLeast(const PublicationTiming& current,const PublicationTiming& required) {
    return current.publication_sequence&&current.context_sequence==required.context_sequence&&
      current.clock_fault==required.clock_fault&&current.map_revision>=required.map_revision&&
      current.cloud_source_ns>=required.cloud_source_ns&&
      current.ray_source_ns[0]>=required.ray_source_ns[0]&&current.ray_source_ns[1]>=required.ray_source_ns[1];
  }
  // Caller holds the pool mutex, and the source is still exclusively owned by
  // the map writer or by the owner's already borrowed immutable query lease.
  void recordPublication(int chosen,const GridMap& writer,
                         std::chrono::steady_clock::time_point publication_begin) {
    auto& timing=publication_timing_[chosen];
    timing.fusion=writer.fusionTiming();timing.map_revision=writer.occupancyRevision();
    timing.context_sequence=writer.localizationContextSequence();
    timing.cloud_source_ns=writer.latestCloudStampNs();
    timing.ray_source_ns={{writer.integratedRaySourceStamp(0),writer.integratedRaySourceStamp(1)}};
    timing.clock_fault=writer.rayEvidenceClockFault();
    timing.publication_sequence=++publication_sequence_;
    timing.publication_misses=validation_publication_misses_;
    timing.begin_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(publication_begin.time_since_epoch()).count();
    timing.end_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool publishRole(const GridMap& writer,std::int64_t now,
                   std::chrono::steady_clock::time_point captured,bool solver,bool* unchanged=nullptr) {
    int chosen=-1;
    const auto publication_begin=std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!solver) {
        // Reset/fault identity is visible even when every validation buffer is
        // currently borrowed. Once those leases retire, no NEW reader may
        // reacquire a pre-reset/pre-fault snapshot just because its copy missed.
        if(validation_identity_known_&&
           (validation_context_sequence_!=writer.localizationContextSequence()||
            validation_clock_fault_!=writer.rayEvidenceClockFault()))admission_floor_.reset();
        validation_identity_known_=true;
        validation_context_sequence_=writer.localizationContextSequence();
        validation_clock_fault_=writer.rayEvidenceClockFault();
      } else if(!matchesValidationIdentity(writer.localizationContextSequence(),writer.rayEvidenceClockFault()))return false;
      if(solver&&unchanged&&solver_ready_&&!filling_[2]&&publicationMatchesWriter(publication_timing_[2],writer)) {
        *unchanged=true;return false;
      }
      if(solver) {if(!filling_[2] && slots_[2].use_count()==1) chosen=2;}
      else for(int n=1;n<=2;++n) {
        const int i=(std::max(0,validation_latest_)+n)%2;
        if(!filling_[i]&&slots_[i].use_count()==1) {chosen=i;break;}
      }
      if(chosen<0) {
        if(!solver) {
          ++validation_publication_misses_;
          // A complete new writer state must get a released destination,
          // rather than race repeated owner/fallback borrows every 10 ms.
          // Prefer backup; if owner still pins it, reserve the released fast
          // head. Existing proof/source deadlines remain unchanged throughout
          // the serial writer retry and the next actual publication wake.
          // Repeated publication of identical state has no delivery to reserve.
          if(validation_latest_<0 ||
             !publicationMatchesWriter(publication_timing_[validation_latest_],writer))
            validation_writer_pending_=true;
        }
        return false;
      }
      filling_[chosen]=true;
    }
    try {writer.copyCollisionSnapshotTo(*slots_[chosen],now,captured);}
    catch(...) {std::lock_guard<std::mutex> lock(mutex_);filling_[chosen]=false;throw;}
    std::lock_guard<std::mutex> lock(mutex_);
    recordPublication(chosen,writer,publication_begin);
    filling_[chosen]=false;
    if(solver) solver_ready_=true;
    else {validation_latest_=chosen;validation_writer_pending_=false;}
    return true;
  }
  std::array<GridMap::Ptr,3> slots_;
  std::array<PublicationTiming,3> publication_timing_;
  ReaderDiagnostics reader_diagnostics_;
  std::uint64_t publication_sequence_{0},validation_publication_misses_{0};
  std::mutex mutex_;
  std::size_t latest_{2};
  bool published_{false};
  std::array<bool,3> filling_{{false,false,false}};
  int validation_latest_{-1};
  // No fourth slot, callback wait or source renewal. The live writer remains
  // solely responsible for publishing its latest COMPLETE state on retry.
  bool validation_writer_pending_{false};
  bool validation_identity_known_{false},validation_clock_fault_{false};
  std::uint64_t validation_context_sequence_{0};
  bool solver_ready_{false};
  std::optional<PublicationTiming> admission_floor_;
  SnapshotReadArbiter slow_readers_;
};
}  // namespace scan_planner
