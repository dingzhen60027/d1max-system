#pragma once

#include "d1max_trajectory_tracker/command_timing_diagnostics.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <nlohmann/json.hpp>

namespace d1max_trajectory_tracker {

// Passive demand trace, not SDK transmission or a motion gate. Only immutable
// copies cross to the diagnostic callback. No ROS or controller dependencies.
struct ControlTraceIdentity {
  std::uint64_t id{0},epoch{0},anchor_revision{0};
  std::string session,task,route,route_hash,map,seed,segment,anchor,execution;
  bool bounded()const {
    for(const auto* s:{&session,&task,&route,&route_hash,&map,&seed,&segment,&anchor,&execution})
      if(s->size()>256)return false;
    return true;
  }
};

struct ControlTraceRecord {
  struct Source {
    std::int64_t received_ns{0},source_ns{0},posterior_ns{0},imu_ns{0},valid_until_ns{0};
    std::uint32_t schema{0};std::uint64_t epoch{0};double extrapolation_s{0.};
    bool present{false},usable{false},decoded{false},accepted{false};
    std::string seed,provider_reason,receiver_reason;
  };
  CommandTransitionDiagnostics::Transition tick;
  std::shared_ptr<const ControlTraceIdentity> identity;
  bool task_active{false}; // captured on the control lane, not inferred from a retained ID
  std::uint64_t permit_sequence{0},commit_sequence{0},proof_sequence{0},map_revision{0},
    context_sequence{0},map_geometry_revision{0};
  std::int64_t body_valid_until_ns{0},demand_valid_until_ns{0},proof_source_ns{0},
    proof_valid_until_ns{0},front_source_ns{0},rear_source_ns{0},proof_body_source_ns{0},
    check_begin_ns{0},check_end_ns{0};
  double tick_to_demand_publish_s{0.};
  std::string reason,phase,prepare_reason,zero_cause,proof_failure,native_reason;
  Source latest,accepted;
};

class ControlTraceBuffer {
 public:
  static constexpr std::size_t Capacity=64,MaxLineBytes=8192;
  struct Batch {
    std::array<std::optional<ControlTraceRecord>,Capacity> records;
    std::size_t size{0};
    std::uint64_t overflow{0},contention{0},failures{0};
  };
  // Never wait for a logger, formatter, or consumer. A dropped observation does
  // not affect the original command; counters make loss explicit.
  void push(ControlTraceRecord record) noexcept {
    try {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(!lock.owns_lock()){++contention_;return;}
      if(!records_.push(std::move(record)))++overflow_;
    } catch(...) {++failures_;}
  }
  Batch take() noexcept {
    Batch batch;
    try {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(lock.owns_lock()) {
        while(auto record=records_.pop())batch.records[batch.size++]=std::move(record);
      }
      batch.overflow=overflow_.load();batch.contention=contention_.load();batch.failures=failures_.load();
    } catch(...) {++failures_;batch.failures=failures_.load();}
    return batch;
  }
  void failure()noexcept {++failures_;}

  // Consumer-only formatting. Returns one physical log line <= 8 KiB,
  // as JSONL. Original source ns are reconstructed exactly from
  // base_source_ns + *_ns offsets. The steady clock is explicitly separate.
  static std::string format(const Batch& batch,std::int64_t flush_source_ns,double flush_steady_s) {
    using J=nlohmann::json;
    const auto dump=[](const J& j){return j.dump(-1,' ',false,J::error_handler_t::replace);};
    const std::int64_t base=batch.size?batch.records[0]->tick.current.source_ns:flush_source_ns;
    const double steady_base=batch.size?batch.records[0]->tick.current.steady_s:flush_steady_s;
    const auto offset=[base](std::int64_t value){return value-base;};
    const auto ns=[](double value){return std::llround(value*1e9);};
    J out={{"schema",1},{"component","local_follower_demand"},
      {"base_source_ns",base},{"base_steady_s",steady_base},
      {"flush_source_ns",flush_source_ns},{"flush_steady_s",flush_steady_s},
      {"task_active",batch.size&&batch.records[0]->task_active},
      {"overflow_total",batch.overflow},{"contention_total",batch.contention},{"failures_total",batch.failures},
      {"batch_samples",batch.size},{"omitted_ticks",0},{"omitted_details",0},
      {"columns",{"command_seq","identity","curve","ref","source_ns","body_ns","steady_ns","dt_ns",
        "vx","vy","wz","tick_to_demand_publish_ns","permit_seq","commit_seq","body_valid_ns","demand_valid_ns",
        "proof_seq","proof_source_ns","proof_valid_ns","front_ns","rear_ns","map_revision","event_bits",
        "context_seq","map_geometry_revision"}},
      {"contexts",J::object()},{"ticks",J::array()},{"events",J::array()}};
    std::size_t omitted_ticks=0,omitted_details=0;
    // Leave room for loss-counter digits and the shared rotating file sink's
    // outer metadata; the complete physical JSONL line stays below 8 KiB.
    const auto fits=[&](const J& candidate){return dump(candidate).size()+640<=MaxLineBytes;};
    std::array<bool,Capacity> included{};
    const auto source=[](const ControlTraceRecord::Source& s){return J{
      {"present",s.present},{"usable",s.usable},{"accepted",s.accepted},{"received_ns",s.received_ns},
      {"source_ns",s.source_ns},{"posterior_ns",s.posterior_ns},{"imu_ns",s.imu_ns},
      {"valid_until_ns",s.valid_until_ns},{"schema",s.schema},{"epoch",s.epoch},{"seed",s.seed},
      {"decoded",s.decoded},{"extrapolation_s",s.extrapolation_s},
      {"provider_reason",s.provider_reason},{"receiver_reason",s.receiver_reason}};};
    for(std::size_t index=0;index<batch.size;++index) {
      const auto& r=*batch.records[index];const auto& s=r.tick.current;
      if(!r.identity||!r.identity->bounded()){++omitted_ticks;continue;}
      J candidate=out;const auto& c=*r.identity;const auto key=std::to_string(c.id);
      if(!candidate["contexts"].contains(key))candidate["contexts"][key]={
        {"session",c.session},{"task",c.task},{"route",c.route},{"route_hash",c.route_hash},
        {"map",c.map},{"epoch",c.epoch},{"seed",c.seed},{"segment",c.segment},
        {"anchor",c.anchor},{"anchor_revision",c.anchor_revision},{"execution",c.execution}};
      candidate["ticks"].push_back({s.command_sequence,c.id,s.curve_id,s.reference_generation,
        offset(s.source_ns),offset(s.body_source_ns),ns(s.steady_s-steady_base),ns(r.tick.interval_s),
        s.forward,s.lateral,s.yaw,ns(r.tick_to_demand_publish_s),r.permit_sequence,r.commit_sequence,
        offset(r.body_valid_until_ns),offset(r.demand_valid_until_ns),r.proof_sequence,
        offset(r.proof_source_ns),offset(r.proof_valid_until_ns),offset(r.front_source_ns),
        offset(r.rear_source_ns),r.map_revision,r.tick.kind,r.context_sequence,r.map_geometry_revision});
      if(!fits(candidate)){++omitted_ticks;continue;}
      out=std::move(candidate);included[index]=true;
    }
    // Tick samples have priority over verbose transition details so a run of
    // changing reasons cannot erase the intervening acceleration history.
    for(std::size_t index=0;index<batch.size;++index) {
      const auto& r=*batch.records[index];if(!r.tick.kind)continue;
      if(!included[index]){++omitted_details;continue;}
      J candidate=out;const auto& p=r.tick.previous;
      candidate["events"].push_back({{"command_seq",r.tick.current.command_sequence},
        {"previous",{p.command_sequence,p.curve_id,p.reference_generation,p.source_ns,p.steady_s,p.forward,p.lateral,p.yaw}},
        {"reason",r.reason},{"phase",r.phase},{"prepare_reason",r.prepare_reason},{"zero_cause",r.zero_cause},
        {"proof_failure",r.proof_failure},{"native_reason",r.native_reason},
        {"check_begin_ns",r.check_begin_ns},{"check_end_ns",r.check_end_ns},
        {"proof_body_ns",r.proof_body_source_ns},{"latest_state",source(r.latest)},
        {"last_accepted_state",source(r.accepted)}});
      if(fits(candidate))out=std::move(candidate);else ++omitted_details;
    }
    out["omitted_ticks"]=omitted_ticks;out["omitted_details"]=omitted_details;
    return dump(out);
  }
 private:
  std::mutex mutex_;BoundedDiagnosticEvents<ControlTraceRecord,Capacity> records_;
  std::atomic<std::uint64_t> overflow_{0},contention_{0},failures_{0};
};
} // namespace d1max_trajectory_tracker
