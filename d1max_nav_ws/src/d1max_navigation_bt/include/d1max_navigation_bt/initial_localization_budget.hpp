#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace d1max_navigation_bt {

struct InitialLocalizationTiming {
  bool preparing_index{false};
  std::optional<std::chrono::steady_clock::time_point> index_completed;
  std::string failure;
  std::string waiting_reason;
};

// This is component lifecycle evidence only. It can budget a genuine map
// index prewarm, but cannot admit a pose, refresh sensor source age or authorize
// execution. A publisher sequence also works while isolated ROS time is paused.
class InitialLocalizationEvidence {
public:
  using Clock=std::chrono::steady_clock;
  bool observe(unsigned schema,const std::string& session,const std::string& map,
      std::uint64_t sequence,std::int64_t epoch,const std::string& phase,
      const std::string& reason,bool index_ready,double report_time,double ros_now,Clock::time_point received,
      const std::string& expected_session,const std::string& expected_map) {
    const bool map_valid=expected_map.size()==64&&std::all_of(expected_map.begin(),expected_map.end(),
      [](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');});
    const bool finished=phase=="waiting_stationary"||phase=="waiting_lio_and_head_forward"||
      phase=="waiting_scan"||phase=="searching"||phase=="confirming"||phase=="ready"||
      phase=="manual_required_after_local_reset"||phase=="failed";
    if(schema!=1||session.empty()||session!=expected_session||!map_valid||map!=expected_map||
       sequence==0||sequence<=sequence_||epoch<epoch_||
       !std::isfinite(report_time)||!std::isfinite(ros_now)||report_time<=0||
       ros_now<report_time||ros_now-report_time>.6||
       (phase!="indexing"&&phase!="waiting_lio"&&!finished)||
       (phase=="failed"&&reason.empty()))return false;
    sequence_=sequence;epoch_=epoch;report_time_=report_time;received_=received;
    preparing_=phase=="indexing";
    waiting_reason_=reason;
    // Never restart the work clock on retry, index flashback or a new LIO epoch.
    if(index_ready&&!completed_)completed_=received;
    failure_=(phase=="failed"||phase=="manual_required_after_local_reset")?reason:std::string{};
    return true;
  }
  InitialLocalizationTiming timing(double ros_now,Clock::time_point now)const {
    const auto receipt_age=std::chrono::duration<double>(now-received_).count();
    const bool fresh=sequence_&&std::isfinite(ros_now)&&ros_now>=report_time_&&
      ros_now-report_time_<=.6&&receipt_age>=0&&receipt_age<=.6;
    return {fresh&&preparing_,completed_,fresh?failure_:std::string{},fresh?waiting_reason_:std::string{}};
  }
private:
  std::uint64_t sequence_=0;std::int64_t epoch_=0;
  double report_time_=0.;Clock::time_point received_{};
  bool preparing_=false;std::optional<Clock::time_point> completed_;
  std::string failure_,waiting_reason_;
};

class InitialLocalizationBudget {
public:
  using Clock=std::chrono::steady_clock;
  InitialLocalizationBudget(double warmup=120.,double work=60.):warmup_(warmup),work_(work) {
    if(!std::isfinite(warmup)||!std::isfinite(work)||warmup<=0||warmup>180.||work<=0||work>60.)
      throw std::invalid_argument("bounded_initial_localization_stage_deadlines_required");
  }
  void start(Clock::time_point now){started_=now;work_started_.reset();saw_index_=false;}
  std::string observe(Clock::time_point now,const InitialLocalizationTiming& timing) {
    if(now<started_)return "initial_localization_steady_clock_rollback";
    if(!timing.failure.empty())return "initial_localization_startup_failed:"+timing.failure;
    if(!work_started_&&timing.index_completed)work_started_=std::max(started_,*timing.index_completed);
    if(!work_started_&&timing.preparing_index)saw_index_=true;
    // Unknown/stale reports never start a timer, reset it or grant another
    // warmup. No valid index evidence means the original 60s startup budget.
    const auto since=work_started_.value_or(started_);
    const bool warmup=saw_index_&&!work_started_;
    if(std::chrono::duration<double>(now-since).count()>=(warmup?warmup_:work_))
      return warmup?"initial_localization_index_warmup_timeout":"initial_localization_wait_timeout";
    return {};
  }
  bool preparing()const{return saw_index_&&!work_started_;}
private:
  double warmup_,work_;Clock::time_point started_{};
  std::optional<Clock::time_point> work_started_;bool saw_index_=false;
};
}  // namespace d1max_navigation_bt
