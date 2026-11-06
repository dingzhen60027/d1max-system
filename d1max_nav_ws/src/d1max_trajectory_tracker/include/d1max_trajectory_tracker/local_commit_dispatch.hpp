#pragma once

#include <cmath>
#include <optional>
#include <d1max_planning_interfaces/msg/local_trajectory_commit.hpp>

namespace d1max_trajectory_tracker {
// Transport coalescing only. LocalFollower owns the transaction and supplies
// its original immutable request. An ACK/new phase is forwarded immediately;
// an identical reliable retry retains that request's source and deadline.
// This is independent of the 20 Hz velocity clock and never produces motion.
class LocalCommitDispatch {
 public:
  using Commit=d1max_planning_interfaces::msg::LocalTrajectoryCommit;
  template<class Emit>
  bool forward(const std::optional<Commit>& message,double receipt,Emit&& emit) {
    if(!message){last_.reset();return false;}
    if(!std::isfinite(receipt))return false;
    if(last_&&*last_==*message&&receipt-last_sent_<.05)return false;
    last_=*message;last_sent_=receipt;
    emit(*message);return true;
  }
 private:
  std::optional<Commit> last_;
  double last_sent_{0.};
};
} // namespace d1max_trajectory_tracker
