#pragma once
#include <plan_env/projected_rays.hpp>
#include <plan_env/native_pipeline_health.hpp>
#include <d1max_planning_interfaces/msg/projected_rays.hpp>
#include <memory>
#include <mutex>
#include <utility>

namespace scan_planner {
// Published only after the sole map writer has reset the old coordinate epoch.
// A shared immutable value is also a generation token: equal text does not
// allow an old in-flight decode to cross a reset/ACK boundary.
struct ProjectedRayContext {
  std::string session,seed,frame;
  std::uint64_t epoch{0},sequence{0},barrier{0};
  std::int64_t pair_barrier{0};
  double maximum_age{0.};
  bool diagnostics{false};
};
using ProjectedRayContextPtr=std::shared_ptr<const ProjectedRayContext>;
inline std::int64_t projectedRayStamp(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec<0||stamp.nanosec>=1000000000U?0:
      static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
}
inline bool projectedRayTimesMatch(std::int64_t begin,std::int64_t end,std::int64_t alignment,
    const ProjectedRayContext& context,std::int64_t now) {
  return begin>context.pair_barrier && alignment>context.pair_barrier && end>=begin && end-begin<=250000000LL &&
      rayStampFresh(begin,now,context.maximum_age) && rayStampFresh(end,now,context.maximum_age) &&
      rayStampFresh(alignment,now,context.maximum_age);
}
inline bool projectedRayContextMatches(const d1max_planning_interfaces::msg::ProjectedRays& message,
    const ProjectedRayContext& context,std::int64_t now) {
  return context.sequence && message.session_id==context.session && message.epoch==context.epoch &&
      message.seed_id==context.seed && message.context_sequence==context.sequence &&
      message.barrier_ns==context.barrier && message.projection_sequence &&
      projectedRayTimesMatch(projectedRayStamp(message.rays.header.stamp),
          projectedRayStamp(message.acquisition_end),projectedRayStamp(message.alignment_stamp),context,now);
}
struct PendingProjectedRays {
  PendingProjectedRays()=default;
  // Preserve memory-only/legacy callers that construct {batch, receipt, ready}.
  PendingProjectedRays(ProjectedRayBatch value,std::chrono::steady_clock::time_point at,bool pending)
      :batch(std::move(value)),received(at),writer_received(at),ready(pending) {}
  ProjectedRayBatch batch;
  std::chrono::steady_clock::time_point received;
  // Pair-wait begins at map-owner admission, not at the independent receiver.
  // received remains the original callback time for total-age evidence.
  std::chrono::steady_clock::time_point writer_received;
  bool ready{false};
  ProjectedRayContextPtr context;
  std::int64_t accepted_ns{0};
  std::int64_t acquisition_end_ns{0},alignment_stamp_ns{0};
  std::uint64_t projection_sequence{0};
};

// One serial ingress producer, one map-writer consumer. Decode never holds
// mutex_ or touches GridMap. Exactly two latest decoded slots are retained;
// taking them swaps ownership with the writer's two slots, never lends mutable
// vector storage to a concurrent reader. One producer scratch reuses storage.
class ProjectedRayMailbox {
public:
  struct Transfer {
    std::array<std::uint64_t,2> drops{{0,0}};
    std::array<bool,2> taken{{false,false}};
    std::uint64_t unattributed{0},callbacks{0},drain_taken{0},drain_accepted{0};
    ProjectedRayIngressDrain last_drain;
  };
  ProjectedRayContextPtr context()const {
    std::lock_guard<std::mutex> lock(mutex_);return context_;
  }
  NativePipelineHealth pipelineHealth()const {
    // The same bounded metadata lock makes reset, accepted sources and the
    // completion token coherent. No map, decode, allocation or ROS API here.
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    health_clock_fault_=health_clock_fault_ || now<=0 || now<health_sampled_ns_ ||
        (fusion_completed_ns_>0 && now<fusion_completed_ns_);
    health_sampled_ns_=now;
    return {context_,stamps_,fusion_sequence_,fusion_completed_ns_,now,health_clock_fault_};
  }
  // Writer-only, after a COMPLETE real fusion. Empty watchdog turns, snapshot
  // republishes and ingress receipt must never renew writer progress.
  void completedFusion(const ProjectedRayContextPtr& expected,std::uint64_t sequence,
      std::int64_t completed_steady_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!expected || expected!=context_)return;
    if(!sequence || sequence<=fusion_sequence_ || completed_steady_ns<=0 ||
       completed_steady_ns<fusion_completed_ns_) {health_clock_fault_=true;return;}
    fusion_sequence_=sequence;fusion_completed_ns_=completed_steady_ns;
  }
  // Writer only. Closing first invalidates both pending slots AND a producer
  // that took the old token before reset but is still decoding outside lock.
  void setContext(ProjectedRayContextPtr context) {
    std::lock_guard<std::mutex> lock(mutex_);
    context_=std::move(context);stamps_.fill(0);sequences_.fill(0);stats_={};
    fusion_sequence_=0;fusion_completed_ns_=0;health_sampled_ns_=0;health_clock_fault_=false;
    for(auto& slot:slots_) {slot.ready=false;slot.context.reset();}
  }
  void recordCallback() {
    std::lock_guard<std::mutex> lock(mutex_);++stats_.callbacks;
  }
  void recordDrain(ProjectedRayIngressDrain result) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_drain=result;stats_.drain_taken+=result.taken;stats_.drain_accepted+=result.accepted;
  }
  bool accept(const d1max_planning_interfaces::msg::ProjectedRays& message,std::int64_t now,
      std::chrono::steady_clock::time_point receipt) {
    const auto token=context();
    if(!token || !projectedRayContextMatches(message,*token,now))return reject(token);
    if(!decodeProjectedRaysInto(message.rays,token->frame,token->diagnostics,scratch_))return reject(token);
    for(auto& meta:scratch_.diagnostics) {
      meta.context_sequence=message.context_sequence;meta.projection_sequence=message.projection_sequence;
      meta.received_ns=now;meta.alignment_stamp_ns=projectedRayStamp(message.alignment_stamp);
      meta.acquisition_end_ns=projectedRayStamp(message.acquisition_end);
    }
    return offerDecoded(token,scratch_,message.projection_sequence,now,receipt,
        projectedRayStamp(message.acquisition_end),projectedRayStamp(message.alignment_stamp));
  }
  // Producer-only commit after decoding. Public for deterministic in-flight
  // reset tests; all production callers arrive through accept().
  bool offerDecoded(const ProjectedRayContextPtr& token,ProjectedRayBatch& batch,std::uint64_t sequence,
      std::int64_t accepted,std::chrono::steady_clock::time_point receipt,
      std::int64_t acquisition_end,std::int64_t alignment_stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!token || token!=context_)return false;
    const auto sensor=batch.sensor_id;
    if(sensor>=slots_.size() || batch.endpoints.empty() || !sequence) {++stats_.unattributed;return false;}
    if(batch.stamp_ns<=stamps_[sensor] || sequence<=sequences_[sensor]) {++stats_.drops[sensor];return false;}
    auto& slot=slots_[sensor];if(slot.ready)++stats_.drops[sensor];
    stamps_[sensor]=batch.stamp_ns;sequences_[sensor]=sequence;
    std::swap(slot.batch,batch);slot.context=token;slot.projection_sequence=sequence;
    slot.accepted_ns=accepted;slot.received=receipt;slot.ready=true;
    slot.acquisition_end_ns=acquisition_end;slot.alignment_stamp_ns=alignment_stamp;
    return true;
  }
  // Sole writer only, at a fusion boundary, never while reading these slots.
  // All allocations/decoding/integration occur outside the short swap lock.
  Transfer takeLatest(const ProjectedRayContextPtr& expected,std::array<PendingProjectedRays,2>& out,
      std::int64_t source_now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!expected || expected!=context_)return {};
    auto report=stats_;stats_.drops.fill(0);stats_.unattributed=0;
    for(std::size_t i=0;i<slots_.size();++i) {
      auto& slot=slots_[i];if(!slot.ready)continue;
      if(slot.context!=expected) {slot.ready=false;continue;}
      // Admission moved to an independent receiver, not to a younger clock.
      // Recheck the original public contract at writer admission, before an
      // invalid newer packet can replace an already valid writer-owned batch.
      if(!projectedRayTimesMatch(slot.batch.stamp_ns,slot.acquisition_end_ns,
          slot.alignment_stamp_ns,*expected,source_now)) {
        slot.ready=false;slot.context.reset();++report.drops[i];continue;
      }
      if(out[i].ready)++report.drops[i];
      std::swap(out[i],slot);slot.ready=false;slot.context.reset();
      report.taken[i]=true;
    }
    return report;
  }
private:
  bool reject(const ProjectedRayContextPtr& token) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(token==context_)++stats_.unattributed;
    return false;
  }
  mutable std::mutex mutex_;
  ProjectedRayContextPtr context_;
  std::array<PendingProjectedRays,2> slots_;
  std::array<std::int64_t,2> stamps_{{0,0}};
  std::array<std::uint64_t,2> sequences_{{0,0}};
  Transfer stats_;
  std::uint64_t fusion_sequence_{0};
  std::int64_t fusion_completed_ns_{0};
  mutable std::int64_t health_sampled_ns_{0};
  mutable bool health_clock_fault_{false};
  ProjectedRayBatch scratch_; // ingress only; reset never clears this storage
};
} // namespace scan_planner
