#pragma once

#include <d1max_planning_interfaces/msg/sdk_execution_state.hpp>
#include <charconv>
#include <cstdint>
#include <limits>
#include <string>

namespace d1max_navigation_bt {
// Manual operation and navigation share one SDK writer. This observes its
// ownership boundary; it neither writes velocities nor changes localization.
// The retained handoff event retires only pre-event intents. SDK state phases
// describe current availability, but cannot erase a newer goal when delayed.
class SDKManualControlFence {
public:
  using State=d1max_planning_interfaces::msg::SDKExecutionState;
  struct Handoff {bool accepted=false,duplicate=false;std::int64_t cutoff=0;};
  bool observeState(const State& state,const std::string& sdk_session,
      const std::string& transport,std::int64_t now_ns) {
    const auto& stamp=state.source_stamp;
    const auto source=static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
    if(sdk_session.empty()||state.sdk_session!=sdk_session||state.transport_mode!=transport||
       state.sequence<=sequence_||stamp.sec<0||stamp.nanosec>=1000000000U||source<=0||
       source<=source_||source>now_ns||now_ns-source>500000000LL)return false;
    sequence_=state.sequence;source_=source;
    const bool pending=state.phase=="manual_takeover_pending";
    const bool active=state.phase=="manual_active";
    const bool releasing=state.phase=="manual_releasing";
    if(pending||active||releasing) {
      // A manual state cannot also grant navigation velocity authority.
      if(state.grant_ready)return false;
      engaged_=true;transitioning_=pending||releasing;
      return true;
    }
    engaged_=false;transitioning_=false;
    return true;
  }
  Handoff observeHandoff(unsigned schema,const std::string& sdk_session,
      const std::string& expected_session,const std::string& lease,
      const std::string& stamp,std::int64_t now_ns) {
    const auto generation=decimal(lease),source=decimal(stamp);
    if(schema!=1||expected_session.empty()||sdk_session!=expected_session||
       !generation||!source||source>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())||
       now_ns<=0||source>static_cast<std::uint64_t>(now_ns)||
       generation<handoff_generation_||source<handoff_stamp_)return {};
    if(generation==handoff_generation_)
      return source==handoff_stamp_?Handoff{true,true,static_cast<std::int64_t>(source)}:Handoff{};
    handoff_generation_=generation;handoff_stamp_=source;
    return {true,false,static_cast<std::int64_t>(source)};
  }
  static bool precedesHandoff(std::int64_t goal_source,std::int64_t admitted_at,
      std::int64_t cutoff) {
    return cutoff>0&&(goal_source<=cutoff||admitted_at<=cutoff);
  }
  // A settled manual owner may receive a fresh RViz goal. The sole SDK writer
  // performs handback only after the jog is released and its zero has retired.
  bool transitioning()const{return transitioning_;}
  bool engaged()const{return engaged_;}
private:
  static std::uint64_t decimal(const std::string& text) {
    if(text.empty()||text.front()=='0')return 0;
    std::uint64_t value=0;const auto parsed=std::from_chars(text.data(),text.data()+text.size(),value);
    return parsed.ec==std::errc{}&&parsed.ptr==text.data()+text.size()?value:0;
  }
  std::uint64_t sequence_=0;
  std::uint64_t handoff_generation_=0,handoff_stamp_=0;
  std::int64_t source_=0;
  bool engaged_=false,transitioning_=false;
};
}  // namespace d1max_navigation_bt
