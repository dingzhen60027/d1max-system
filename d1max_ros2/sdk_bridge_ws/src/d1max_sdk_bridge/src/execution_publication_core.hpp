#pragma once
#include "execution_transport_core.hpp"
#include <array>
#include <utility>

namespace d1monitor::execution3 {
// A commit result is a transaction, not a telemetry heartbeat. Identical
// retransmits and the one-way vendor ACK upgrade may occupy the same slot;
// distinct outcomes still consume one of eight slots, and overflow is a veto.
class CommitOutbox {
 public:
  static constexpr size_t capacity=8;
  enum class Result {Queued,Coalesced,Overflow};
  size_t size() const{return size_;}
  uint64_t coalesced() const{return coalesced_;}
  Result put(CommitAck value){
    for(size_t i=0;i<size_;++i){auto& old=data_[(head_+i)%capacity];
      if(old==value){++coalesced_;return Result::Coalesced;}
      if(value.sequence<=old.sequence&&old.write_acknowledged>=value.write_acknowledged&&sameOutcome(old,value)){
        ++coalesced_;return Result::Coalesced;}
      // All immutable fields must match. Never turn an applied/rejected result
      // into another outcome, change sources, or move a transaction backward.
      if(value.sequence>old.sequence&&!old.write_acknowledged&&value.write_acknowledged&&sameOutcome(old,value)){
        old=std::move(value);++coalesced_;return Result::Coalesced;}
    }
    if(size_==capacity)return Result::Overflow;
    data_[(head_+size_)%capacity]=std::move(value);++size_;return Result::Queued;
  }
  bool pop(CommitAck& value){
    if(!size_)return false;
    value=std::move(data_[head_]);head_=(head_+1)%capacity;--size_;return true;
  }
 private:
  static bool sameOutcome(const CommitAck&a,const CommitAck&b){
    return a.schema_version==b.schema_version&&a.handoff_id==b.handoff_id&&a.grant_sequence==b.grant_sequence&&
      a.previous_commit_sequence==b.previous_commit_sequence&&a.commit_sequence==b.commit_sequence&&
      a.incumbent_version==b.incumbent_version&&a.candidate_version==b.candidate_version&&
      a.incumbent_trajectory_id==b.incumbent_trajectory_id&&a.candidate_trajectory_id==b.candidate_trajectory_id&&
      a.execution_id==b.execution_id&&a.sdk_session==b.sdk_session&&a.transport_mode==b.transport_mode&&
      a.control_epoch==b.control_epoch&&a.sdk_arm_generation==b.sdk_arm_generation&&
      a.permit_sequence==b.permit_sequence&&a.demand_sequence==b.demand_sequence&&
      a.motion_validation_sequence==b.motion_validation_sequence&&a.entry_admission_sequence==b.entry_admission_sequence&&
      a.applied_at==b.applied_at&&a.demand_source_stamp==b.demand_source_stamp&&
      a.demand_body_source_stamp==b.demand_body_source_stamp&&a.entry_source_stamp==b.entry_source_stamp&&
      a.body_source_stamp==b.body_source_stamp&&a.valid_until==b.valid_until&&
      a.measured_pose==b.measured_pose&&a.measured_twist==b.measured_twist&&a.applied_velocity==b.applied_velocity&&
      a.curve_time==b.curve_time&&a.applied==b.applied&&a.write_submitted==b.write_submitted&&a.reason==b.reason;
  }
  std::array<CommitAck,capacity>data_;size_t head_=0,size_=0;uint64_t coalesced_=0;
};
}
