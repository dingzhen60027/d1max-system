#pragma once

#include "d1max_navigation_bt/task_contract.hpp"
#include <d1max_planning_interfaces/msg/sdk_execution_state.hpp>
#include <cstdint>
#include <string>

namespace d1max_navigation_bt {
// An operator stop belongs to the SDK session, including before its first
// task grant. Per-task authority matching alone cannot see that empty writer.
// This is an intent fence, not an SDK command or a physical stop certificate.
class SDKSoftwareEstopFence {
public:
  using State=d1max_planning_interfaces::msg::SDKExecutionState;
  bool observe(const State& state,const std::string& sdk_session,
      const std::string& transport,std::int64_t now_ns) {
    const auto&stamp=state.source_stamp;
    const auto source=static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
    if(sdk_session.empty()||state.sdk_session!=sdk_session||state.transport_mode!=transport||
       state.sequence<=sequence_||stamp.sec<0||stamp.nanosec>=1000000000U||source<=0||
       source<=source_||source>now_ns||now_ns-source>500000000LL)return false;
    sequence_=state.sequence;source_=source;
    const bool software_reason=state.reason=="software_estop_requested"||
      state.reason=="software_estop_observed"||
      state.reason=="software_estop_release_waiting_prior_retirement";
    const bool latch=state.phase=="software_estop_latched"&&software_reason&&
      state.fault_latched&&!state.grant_ready;
    if(latch){const bool edge=!blocked_;blocked_=true;return edge;}
    // Only a newer explicit nonfault writer state clears the new-intent block.
    // It does not clear an already revoked task, confirmation or goal intent.
    // A retired task keeps its original software-stop cause in reason; its
    // newer nonfault state is still an OFF/release fact, not another ON edge.
    if(!state.fault_latched&&state.phase!="software_estop_latched")
      blocked_=false;
    return false;
  }
  bool blocked()const{return blocked_;}
private:
  std::uint64_t sequence_=0;
  std::int64_t source_=0;
  bool blocked_=false;
};

inline bool revokeSoftwareEstopIntent(GoalExecutionIntent& intent,
    ConfirmationLedger& confirmation,bool sdk_bound,const std::string& reason) {
  if(!intent.requested()&&!confirmation.authorized()&&!sdk_bound)return false;
  intent.revoke(reason);confirmation.revoke();return true;
}
}  // namespace d1max_navigation_bt
