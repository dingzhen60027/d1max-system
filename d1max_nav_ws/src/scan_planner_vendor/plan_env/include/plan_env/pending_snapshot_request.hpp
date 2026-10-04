#pragma once
#include <plan_env/solve_budget.hpp>
#include <cstdint>

namespace scan_planner {
// One acquisition intent, not a solve queue or a collision lease. A busy
// exclusive slot may be retried inside the SAME original budget. Neither a
// retry nor a fresh map publication can renew that budget.
class PendingSnapshotRequest {
public:
  SolveBudget::Ptr begin(std::uint64_t generation,
      SolveBudget::Now now=[] {return SolveBudget::Clock::now();}) {
    if(!budget_) {generation_=generation;budget_=std::make_shared<SolveBudget>(
        std::chrono::milliseconds(400),std::move(now));}
    return budget_;
  }
  bool pending() const {return static_cast<bool>(budget_);}
  bool current(std::uint64_t generation,bool source_fresh) const {
    return budget_&&generation_==generation&&source_fresh&&budget_->allowed();
  }
  SolveBudget::Ptr budget() const {return budget_;}
  void release() {budget_.reset();} // Transfer the original budget to one worker.
  void cancel() {if(budget_)budget_->cancel();budget_.reset();}
private:
  SolveBudget::Ptr budget_;
  std::uint64_t generation_{0};
};
} // namespace scan_planner
