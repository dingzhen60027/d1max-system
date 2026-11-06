#pragma once
#include <d1max_planning_interfaces/msg/task_command_feedback.hpp>
#include <optional>
#include <cstdint>
#include <utility>

namespace d1monitor::execution3 {
using TaskFeedback=d1max_planning_interfaces::msg::TaskCommandFeedback;
// Owner-thread-only, bounded sending facts. This ledger never chooses a
// velocity, checks geometry, renews a source or claims physical stopping.
class TaskCommandFeedbackLedger {
public:
  void reset(){latest_.reset();last_nonzero_.reset();moving_=false;gap_write_=0;gap_stamp_={};
    interrupted_sequence_=0;interrupted_source_={};interrupted_deadline_={};}
  void record(TaskFeedback fact) {
    if(!fact.write_sequence||(latest_&&fact.write_sequence<=latest_->write_sequence))return;
    if(fact.write_submitted) {
      // Only a real submitted zero after a real nonzero can open a gap.
      // Explicit controller zero already has a matching conditioner history.
      if(moving_&&!fact.sdk_nonzero&&!fact.command_selected) {
        gap_write_=fact.write_sequence;gap_stamp_=fact.source_stamp;
        if(last_nonzero_){interrupted_sequence_=last_nonzero_->command_sequence;
          interrupted_source_=last_nonzero_->command_source_stamp;
          interrupted_deadline_=last_nonzero_->command_valid_until;}
      }
      moving_=fact.sdk_nonzero;
      if(moving_)last_nonzero_=fact;
    }
    fact.discontinuity_write_sequence=gap_write_;fact.discontinuity_stamp=gap_stamp_;
    fact.interrupted_command_sequence=interrupted_sequence_;
    fact.interrupted_command_source_stamp=interrupted_source_;
    fact.interrupted_command_valid_until=interrupted_deadline_;
    latest_=std::move(fact);
  }
  std::uint64_t discontinuity()const{return gap_write_;}
  const std::optional<TaskFeedback>& latest()const{return latest_;}
private:
  std::optional<TaskFeedback>latest_,last_nonzero_;bool moving_{false};
  std::uint64_t gap_write_{0},interrupted_sequence_{0};
  builtin_interfaces::msg::Time gap_stamp_,interrupted_source_,interrupted_deadline_;
};
} // namespace d1monitor::execution3
