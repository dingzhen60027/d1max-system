#pragma once
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

namespace scan_planner {
// Waiting is observability, never collision evidence or execution permission.
// Fixed counters and one original nonzero request survive a later idle/stop
// tick without retaining an unbounded stream of control messages.
enum class MotionCheckStage : std::size_t {
  Idle, SnapshotUnavailable, PermitNotDelivered, CurveProofUnavailable,
  SourceLeaseExpired, BodyNotDelivered, BodyTwistMissing, BodyBehindDemand,
  BodySourceExpired, BrakingModelUnavailable, ActiveGeometryUnavailable,
  MeasuredProgressUnavailable, SupportUnavailable, ActiveGeometryMismatch,
  ActiveProofUnavailable, PermitBindingMismatch, CommandSourceExpired,
  SnapshotSourceUnavailable, BodyYawUnavailable, QueryBudgetExhausted,
  SweepBudgetExhausted, PublicationBudgetExhausted, EvidenceExpiredDuringCheck,
  CurveProofSuperseded, StopBarrierChanged, ActiveGeometryChanged,
  OwnerPermitChanged, ActiveProofExpired, VerdictPublished, LocalCommandRelayed,
  DemandContractInvalid, DemandAuthorityMismatch, DemandSequenceNotNew,
  CandidateCommitPending, CandidateProofUnavailable, CandidateSourceExpired,
  NativePipelineFault, Count
};

class MotionCheckDiagnostics {
public:
  static constexpr std::size_t stage_count=static_cast<std::size_t>(MotionCheckStage::Count);
  static constexpr std::size_t event_capacity=64;
  using Demand=d1max_planning_interfaces::msg::MotionDemand;
  struct Event {
    std::uint64_t event_id{0},command_sequence{0},proof_sequence{0},validation_floor{0};
    MotionCheckStage stage{MotionCheckStage::Idle};
    std::int64_t observed_at_ns{0},command_source_ns{0},body_source_ns{0},
      command_valid_until_ns{0},native_body_source_ns{0},curve{-1},active_curve{-1};
    std::string task_id;
    bool hold{false};
  };
  struct Snapshot {
    MotionCheckStage stage{MotionCheckStage::Idle};
    MotionCheckStage last_nonzero_stage{MotionCheckStage::Idle};
    std::array<std::uint64_t,stage_count> counts{};
    std::int64_t observed_at_ns{0},last_nonzero_observed_at_ns{0};
    std::optional<Demand> last_nonzero_demand;
    std::uint64_t proof_sequence{0},map_snapshot_revision{0};
    // Fixed storage: overload overwrites oldest diagnostic events only.
    // Event IDs make omissions explicit; this is never an execution queue.
    std::uint64_t event_sequence{0};
    std::array<Event,event_capacity> events{};
  };
  static const char* name(MotionCheckStage stage) {
    static constexpr std::array<const char*,stage_count> names{{
      "idle_no_nonzero_demand","waiting_collision_snapshot","waiting_exact_permit_delivery",
      "waiting_native_curve_proof","source_lease_expired","waiting_independent_body",
      "waiting_independent_body_twist","waiting_body_at_or_after_demand","body_source_expired",
      "waiting_braking_model","waiting_active_geometry","waiting_measured_curve_progress",
      "waiting_support_reference","active_geometry_mismatch","waiting_active_curve_proof",
      "permit_binding_mismatch","command_source_expired","waiting_fresh_matching_snapshot",
      "waiting_body_yaw","query_budget_exhausted","sweep_budget_exhausted",
      "publication_budget_exhausted","evidence_expired_during_check","curve_proof_superseded",
      "stop_barrier_changed","active_geometry_changed","owner_permit_changed",
      "active_curve_proof_expired","actual_verdict_published","local_command_relayed",
      "demand_contract_invalid","demand_authority_mismatch","demand_sequence_not_new",
      "waiting_candidate_commit","candidate_proof_unavailable","candidate_source_expired",
      "native_input_or_pipeline_fault"}};
    const auto i=static_cast<std::size_t>(stage);
    return i<names.size()?names[i]:"unknown_diagnostic_stage";
  }
  void record(MotionCheckStage stage,const std::optional<Demand>& demand,
      std::int64_t observed_at_ns,std::uint64_t proof_sequence=0,
      std::uint64_t map_snapshot_revision=0,std::int64_t active_curve=-1,
      std::int64_t native_body_source_ns=0) {
    const auto i=static_cast<std::size_t>(stage);if(i>=stage_count)return;
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.stage=stage;snapshot_.observed_at_ns=observed_at_ns;
    if(snapshot_.counts[i]!=std::numeric_limits<std::uint64_t>::max())++snapshot_.counts[i];
    if(demand&&stage!=MotionCheckStage::Idle&&stage!=MotionCheckStage::LocalCommandRelayed&&
        stage!=MotionCheckStage::VerdictPublished) {
      const auto stamp=[](const builtin_interfaces::msg::Time& t) {
        return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
      };
      const auto id=++snapshot_.event_sequence;
      auto& e=snapshot_.events[(id-1)%event_capacity];
      e={id,demand->sequence,proof_sequence,demand->validation_sequence,stage,observed_at_ns,
        stamp(demand->source_stamp),stamp(demand->body_source_stamp),stamp(demand->valid_until),
        native_body_source_ns,demand->trajectory_id,active_curve,demand->version.task_id.substr(0,128),
        demand->hold};
    }
    if(demand&&!demand->hold&&
       (demand->velocity.linear.x!=0.||demand->velocity.linear.y!=0.||demand->velocity.angular.z!=0.)) {
      snapshot_.last_nonzero_demand=*demand;
      snapshot_.last_nonzero_stage=stage;
      snapshot_.last_nonzero_observed_at_ns=observed_at_ns;
      snapshot_.proof_sequence=proof_sequence;
      snapshot_.map_snapshot_revision=map_snapshot_revision;
    }
  }
  Snapshot snapshot()const {std::lock_guard<std::mutex> lock(mutex_);return snapshot_;}
private:
  mutable std::mutex mutex_;
  Snapshot snapshot_;
};
} // namespace scan_planner
