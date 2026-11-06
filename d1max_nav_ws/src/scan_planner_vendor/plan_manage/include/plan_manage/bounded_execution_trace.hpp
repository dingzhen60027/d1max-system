#pragma once
#include <d1max_planning_interfaces/msg/execution_version.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>

namespace scan_planner {
// Diagnostics only. Producers never allocate, serialize, publish or wait for
// the drain. Losing diagnostic events must not delay a command or change its
// original source time. IDs are process-local, not execution authority.
class BoundedExecutionTrace {
  friend struct BoundedExecutionTraceTestAccess;
public:
  // Reserve 512 bytes for the rotating sink's record envelope.
  static constexpr std::size_t capacity=128, identity_capacity=64, max_batch_bytes=11776;
  using Version=d1max_planning_interfaces::msg::ExecutionVersion;
  enum class Kind {DemandRejected,CommandPublishFailed,CommandPublishReturned,CommitRequest,
    CommitDecision,CandidateSelected,CandidateQueued,CandidatePublishBegin,
    CandidatePublishReturned,ReferenceDecision,CurveProof,CurrentCheckFinished,AdmissionCheckFinished};
  template<std::size_t N> struct Text {
    std::array<char,N> value{};std::size_t original_size{0};
    void set(std::string_view s) noexcept {
      original_size=s.size();const auto n=std::min(s.size(),N);value.fill(0);
      if(n)std::memcpy(value.data(),s.data(),n);
    }
    bool matches(std::string_view s)const noexcept {
      return s.size()==original_size&&s.size()<=N&&
        (s.empty()||std::memcmp(value.data(),s.data(),s.size())==0);
    }
    std::string str()const {return {value.data(),std::min(original_size,N)};}
    bool complete()const noexcept{return original_size<=N;}
  };
  struct IdentityInput {
    const Version* version{nullptr};std::int64_t curve{-1};std::string_view proposal{};
  };
  // Context is copied while the caller holds its existing ledger lock. The
  // snapshot is not reconstructed later from independently acquired slots.
  struct Context {IdentityInput request,expected,active,prepared,candidate;};
  struct Retention {
    Text<160> session,task;
    bool active{false};std::int64_t steady_ns{0};
    void set(const Version* version,bool task_active)noexcept {
      if(version){session.set(version->session_id);task.set(version->task_id);}
      active=version&&task_active;
    }
    bool sameBucket(const Retention& other)const noexcept {
      return active==other.active&&session.original_size==other.session.original_size&&
        task.original_size==other.task.original_size&&session.value==other.session.value&&task.value==other.task.value;
    }
  };
  struct Event {
    std::uint64_t id{0};Kind kind{Kind::DemandRejected};
    std::int64_t ros_ns{0},steady_ns{0},source_ns{0},body_ns{0},valid_until_ns{0},
      native_body_ns{0},proof_until_ns{0},lease_until_ns{0},rear_source_ns{0};
    std::int64_t begin_steady_ns{0},queue_steady_ns{0};
    std::uint64_t sequence{0},proof{0},validation_floor{0},map_revision{0},stop_barrier{0},
      lease_sequence{0};
    std::array<std::uint64_t,5> identity{};
    std::array<std::uint64_t,5> before_identity{};
    std::array<double,3> velocity{};bool hold{false},accepted{false};
    Text<96> reason;
    Retention retention;
    // Present only for the independent check-completion stream. Source clocks
    // stay exact; steady clocks describe delivery/compute, never a new lease.
    struct CheckTiming {
      std::uint64_t fusion_sequence{0},publication_sequence{0},publication_misses{0};
      std::int64_t fusion_begin_ns{0},fusion_end_ns{0},fusion_begin_steady_ns{0},fusion_end_steady_ns{0},
        publication_begin_steady_ns{0},publication_end_steady_ns{0},borrow_end_steady_ns{0},
        lease_end_steady_ns{0},check_begin_ns{0},check_end_ns{0},ledger_steady_ns{0},
        output_begin_steady_ns{0},output_end_steady_ns{0},progress_source_ns{0};
      Text<48> disposition;
      bool valid{false};
    } check;
  };
  struct Identity {
    std::uint64_t id{0};std::uint32_t schema{0};
    std::array<Text<160>,9> text{}; // Exact version strings, then proposal.
    std::array<std::uint64_t,5> number{};
    std::int64_t curve{-1};
    bool matches(const IdentityInput& v)const noexcept {
      if(!v.version||schema!=v.version->schema_version||curve!=v.curve)return false;
      const auto& x=*v.version;
      const std::array<std::string_view,9> strings{{x.session_id,x.task_id,x.route_id,x.route_hash,
        x.map_version_id,x.localization_seed_id,x.segment_id,x.anchor_id,v.proposal}};
      const std::array<std::uint64_t,5> nums{{x.localization_epoch,x.reference_generation,
        x.anchor_revision,x.context_sequence,x.map_geometry_revision}};
      if(number!=nums)return false;
      for(std::size_t i=0;i<strings.size();++i)if(!text[i].matches(strings[i]))return false;
      return true;
    }
    void assign(std::uint64_t key,const IdentityInput& v) noexcept {
      id=key;schema=v.version->schema_version;curve=v.curve;const auto& x=*v.version;
      const std::array<std::string_view,9> strings{{x.session_id,x.task_id,x.route_id,x.route_hash,
        x.map_version_id,x.localization_seed_id,x.segment_id,x.anchor_id,v.proposal}};
      for(std::size_t i=0;i<strings.size();++i)text[i].set(strings[i]);
      number={{x.localization_epoch,x.reference_generation,x.anchor_revision,x.context_sequence,x.map_geometry_revision}};
    }
  };
  struct Snapshot {
    std::uint64_t last_event{0},last_identity{0},contention_drops{0},producer_failures{0},selected_overwrites{0};
    std::array<Event,capacity> events{};
    std::array<Identity,identity_capacity> identities{};
  };
  static std::int64_t steadyNowNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void selectedOverwritten() noexcept {selected_overwrites_.fetch_add(1,std::memory_order_relaxed);}
  // Adaptations of existing wire messages may allocate before record(). Keep
  // those optional operations from changing the action being observed.
  template<class F> void attempt(F&& operation) noexcept {
    try {operation();}
    catch(...){producer_failures_.fetch_add(1,std::memory_order_relaxed);}
  }
  bool record(Event e,const Context& context) noexcept {try {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()){contention_drops_.fetch_add(1,std::memory_order_relaxed);return false;}
    e.steady_ns=steadyNowNs();e.id=++state_.last_event;
    const std::array<IdentityInput,5> inputs{{context.request,context.expected,context.active,
      context.prepared,context.candidate}};
    for(std::size_t i=0;i<inputs.size();++i)e.identity[i]=intern(inputs[i]);
    state_.events[(e.id-1)%capacity]=e;return true;
  } catch(...){producer_failures_.fetch_add(1,std::memory_order_relaxed);return false;}}
  std::array<std::uint64_t,5> capture(const Context& context) noexcept {try {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()){contention_drops_.fetch_add(1,std::memory_order_relaxed);return {};}
    const std::array<IdentityInput,5> inputs{{context.request,context.expected,context.active,
      context.prepared,context.candidate}};
    std::array<std::uint64_t,5> result{};
    for(std::size_t i=0;i<inputs.size();++i)result[i]=intern(inputs[i]);
    return result;
  } catch(...){producer_failures_.fetch_add(1,std::memory_order_relaxed);return {};}}
  Snapshot snapshot()const {
    std::lock_guard<std::mutex> lock(mutex_);auto out=state_;
    out.contention_drops=contention_drops_.load(std::memory_order_relaxed);
    out.producer_failures=producer_failures_.load(std::memory_order_relaxed);
    out.selected_overwrites=selected_overwrites_.load(std::memory_order_relaxed);return out;
  }
  // One consumer; JSON and string allocation happen after releasing the ring
  // lock. Every batch carries the complete retained identities it references.
  // Missing/overlong identities are explicit and never treated as equal.
  std::string drain(std::uint64_t& cursor,std::size_t budget=max_batch_bytes,Retention* retention=nullptr)const {
    const auto s=snapshot();if(cursor>=s.last_event)return {};
    budget=std::min(budget,max_batch_bytes);if(budget<1024)return {};
    const auto first=s.last_event>capacity?s.last_event-capacity+1:1;
    const auto start=std::max(cursor+1,first);
    const auto& origin=s.events[(start-1)%capacity];
    const auto ros_base=origin.ros_ns?origin.ros_ns:origin.source_ns;
    const auto steady_base=origin.steady_ns;
    if(retention)*retention=origin.retention;
    nlohmann::json out{{"schema",1},{"diagnostic_only",true},{"capacity",capacity},
      {"clock_base_ns",{ros_base,steady_base}},
      {"identity_capacity",identity_capacity},{"last_available",s.last_event},
      {"retention",{{"session",origin.retention.session.str()},{"task",origin.retention.task.str()},
        {"active",origin.retention.active},{"complete",origin.retention.session.complete()&&origin.retention.task.complete()}}},
      {"overwritten",start-cursor-1},{"contention_drops",s.contention_drops},
      {"producer_failures",s.producer_failures},
      {"selected_publication_overwrites",s.selected_overwrites},
      {"identity_evictions",s.last_identity>identity_capacity?s.last_identity-identity_capacity:0},
      {"events",nlohmann::json::array()},{"identities",nlohmann::json::object()},
      {"tasks",nlohmann::json::object()},
      {"reasons",nlohmann::json::object()},
      {"missing_identities",nlohmann::json::array()}};
    if(origin.kind==Kind::CurrentCheckFinished||origin.kind==Kind::AdmissionCheckFinished)
      out["check_event_schema"]={{"version",1},{"detail_index",26},
        {"accepted_means","result_committed_to_ledger_not_positive_verdict"},
        {"proof_until_means","preceding_proof_original_deadline"},
        {"timestamps","ROS and steady deltas use clock_base_ns; null and publication_sequence=0 mean unavailable"}};
    std::string accepted;std::uint64_t accepted_cursor=cursor;
    for(auto id=start;id<=s.last_event;++id) {
      const auto& e=s.events[(id-1)%capacity];if(e.id!=id)continue;
      if(!origin.retention.sameBucket(e.retention))break;
      auto next=out;
      for(const auto* refs:{&e.identity,&e.before_identity})for(const auto key:*refs)if(key&&!next["identities"].contains(std::to_string(key))) {
        const auto& identity=s.identities[(key-1)%identity_capacity];
        if(identity.id==key)addIdentity(next,identity);
        else if(std::find(next["missing_identities"].begin(),next["missing_identities"].end(),key)==next["missing_identities"].end())
          next["missing_identities"].push_back(key);
      }
      const auto reason=addReason(next,e.reason);
      next["events"].push_back(eventJson(e,ros_base,steady_base,reason));next["through"]=id;
      const auto encoded=next.dump(-1,' ',false,nlohmann::json::error_handler_t::replace);
      if(encoded.size()>budget) {
        if(accepted.empty()) {
          // Even worst-case long identities cannot force oversized output or
          // permanently block draining subsequent events.
          next=out;const auto fallback_reason=addReason(next,e.reason);
          next["events"].push_back(eventJson(e,ros_base,steady_base,fallback_reason));next["through"]=id;
          next["identity_omitted_for_budget"]=true;
          for(const auto* refs:{&e.identity,&e.before_identity})for(const auto key:*refs)if(key)next["missing_identities"].push_back(key);
          accepted=next.dump(-1,' ',false,nlohmann::json::error_handler_t::replace);accepted_cursor=id;
          if(retention)retention->steady_ns=e.steady_ns;
          if(accepted.size()>budget)return {};
        }
        break;
      }
      out=std::move(next);accepted=encoded;accepted_cursor=id;
      if(retention)retention->steady_ns=e.steady_ns;
    }
    cursor=accepted_cursor;return accepted;
  }
  // One diagnostic consumer; a failed sink does not advance the durable
  // cursor. Page count, ring size and individual records remain bounded.
  template<class Emit> std::size_t drainPages(std::uint64_t& cursor,Emit&& emit,
      std::size_t page_limit=8)const {
    std::size_t pages=0;
    for(;pages<page_limit;++pages) {
      Retention retention;auto next=cursor;
      const auto batch=drain(next,max_batch_bytes,&retention);
      if(batch.empty()||!emit(batch,retention))break;
      cursor=next;
    }
    return pages;
  }
  static const char* name(Kind k) noexcept {
    switch(k) {
    case Kind::DemandRejected:return "demand_rejected";
    case Kind::CommandPublishFailed:return "command_publish_failed";
    case Kind::CommandPublishReturned:return "command_publish_returned";
    case Kind::CommitRequest:return "commit_request";
    case Kind::CommitDecision:return "commit_decision_not_ack_delivery";
    case Kind::CandidateSelected:return "candidate_selected";
    case Kind::CandidateQueued:return "candidate_queued";
    case Kind::CandidatePublishBegin:return "candidate_publish_begin";
    case Kind::CandidatePublishReturned:return "candidate_publish_returned";
    case Kind::ReferenceDecision:return "reference_decision";
    case Kind::CurveProof:return "curve_proof_committed";
    case Kind::CurrentCheckFinished:return "current_check_finished";
    case Kind::AdmissionCheckFinished:return "admission_check_finished";
    }return "unknown";
  }
private:
  static void addIdentity(nlohmann::json& batch,const Identity& i) {
    nlohmann::json task_text=nlohmann::json::array(),geometry_text=nlohmann::json::array(),
      task_lengths=nlohmann::json::array(),geometry_lengths=nlohmann::json::array();bool complete=true;
    for(std::size_t n=0;n<i.text.size();++n) {
      const auto& t=i.text[n];complete&=t.complete();
      (n<6?task_text:geometry_text).push_back(t.str());
      (n<6?task_lengths:geometry_lengths).push_back(t.original_size);
    }
    nlohmann::json task{{"schema",i.schema},{"strings",std::move(task_text)},
      {"localization_epoch",i.number[0]}};
    if(!complete)task["original_lengths"]=std::move(task_lengths);
    std::string task_key;
    for(auto it=batch["tasks"].begin();it!=batch["tasks"].end();++it)if(it.value()==task){task_key=it.key();break;}
    if(task_key.empty()){task_key=std::to_string(i.id);batch["tasks"][task_key]=task;}
    batch["identities"][std::to_string(i.id)]={{"task",task_key},{"strings",std::move(geometry_text)},
      {"numbers",{i.number[1],i.number[2],i.number[3],i.number[4]}},{"curve",i.curve},
      {"complete",complete}};
    if(!complete)batch["identities"][std::to_string(i.id)]["original_lengths"]=std::move(geometry_lengths);
  }
  static unsigned addReason(nlohmann::json& batch,const Text<96>& reason) {
    const nlohmann::json value={reason.str(),reason.original_size};
    for(auto it=batch["reasons"].begin();it!=batch["reasons"].end();++it)
      if(it.value()==value)return static_cast<unsigned>(std::stoul(it.key()));
    const auto id=static_cast<unsigned>(batch["reasons"].size()+1);
    batch["reasons"][std::to_string(id)]=value;return id;
  }
  static nlohmann::json eventJson(const Event& e,std::int64_t ros_base,std::int64_t steady_base,unsigned reason) {
    // Fixed field order is documented beside the producer, avoiding repeated
    // long JSON keys on every 20 Hz control sample. Integer deltas are lossless;
    // null is a missing original time, never a synthetic timestamp.
    const auto time=[&](std::int64_t t){return t?nlohmann::json(t-ros_base):nlohmann::json(nullptr);};
    auto out=nlohmann::json::array({e.id,static_cast<unsigned>(e.kind),time(e.ros_ns),e.steady_ns-steady_base,e.identity,e.sequence,
      time(e.source_ns),time(e.body_ns),time(e.valid_until_ns),time(e.native_body_ns),e.proof,e.validation_floor,
      e.map_revision,time(e.proof_until_ns),e.lease_sequence,time(e.lease_until_ns),e.stop_barrier,
      e.velocity,static_cast<unsigned>(e.hold),static_cast<unsigned>(e.accepted),reason,e.reason.original_size,time(e.rear_source_ns),
      e.begin_steady_ns?nlohmann::json(e.begin_steady_ns-steady_base):nlohmann::json(nullptr),
      std::any_of(e.before_identity.begin(),e.before_identity.end(),[](auto id){return id!=0;})?
        nlohmann::json(e.before_identity):nlohmann::json(nullptr),
      e.queue_steady_ns?nlohmann::json(e.queue_steady_ns-steady_base):nlohmann::json(nullptr)});
    if(e.kind==Kind::CurrentCheckFinished||e.kind==Kind::AdmissionCheckFinished) {
      const auto steady=[&](std::int64_t t){return t?nlohmann::json(t-steady_base):nlohmann::json(nullptr);};
      const auto& c=e.check;
      out.push_back({{"schema",1},{"disposition",c.disposition.str()},{"valid",c.valid},
        {"fusion_sequence",c.fusion_sequence},{"publication_sequence",c.publication_sequence},
        {"publication_misses",c.publication_misses},
        {"fusion_ros_ns",{time(c.fusion_begin_ns),time(c.fusion_end_ns)}},
        {"fusion_steady_ns",{steady(c.fusion_begin_steady_ns),steady(c.fusion_end_steady_ns)}},
        {"publication_steady_ns",{steady(c.publication_begin_steady_ns),steady(c.publication_end_steady_ns)}},
        {"borrow_end_steady_ns",steady(c.borrow_end_steady_ns)},
        {"lease_end_steady_ns",steady(c.lease_end_steady_ns)},
        {"check_ros_ns",{time(c.check_begin_ns),time(c.check_end_ns)}},
        {"ledger_steady_ns",steady(c.ledger_steady_ns)},
        {"progress_source_ns",time(c.progress_source_ns)},
        {"output_steady_ns",{steady(c.output_begin_steady_ns),steady(c.output_end_steady_ns)}}});
    }
    return out;
  }
  std::uint64_t intern(const IdentityInput& input) noexcept {
    if(!input.version)return 0;
    for(const auto& i:state_.identities)if(i.id&&i.matches(input))return i.id;
    const auto id=++state_.last_identity;state_.identities[(id-1)%identity_capacity].assign(id,input);return id;
  }
  mutable std::mutex mutex_;Snapshot state_;
  std::atomic<std::uint64_t> contention_drops_{0};
  std::atomic<std::uint64_t> producer_failures_{0};
  std::atomic<std::uint64_t> selected_overwrites_{0};
};
} // namespace scan_planner
