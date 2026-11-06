#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <set>
#include <vector>

namespace d1max_navigation_bt {
struct FunctionalComponent {
  std::string name;
  bool required{false},seen{false};
  double callback_age{0.},source{0.};
};
// Functional heartbeat, not localization readiness or motion authorization.
// Sequence renewal is checked independently of receipt; replay cannot keep an
// executor alive. A short gap withdraws readiness, not the task identity.
// Only an explicit fatal report, invalid clock, or exhausted recovery episode
// isolates the session. Sensor/collision leases remain separate and shorter.
class DependencyHealthGuard {
public:
  explicit DependencyHealthGuard(bool required=false,double heartbeat_timeout=1.5,
      double recovery_timeout=30.,double stable_duration=.6)
    :required_(required),heartbeat_timeout_(heartbeat_timeout),
     recovery_timeout_(recovery_timeout),stable_duration_(stable_duration) {
    if(!std::isfinite(heartbeat_timeout_)||heartbeat_timeout_<=0.||heartbeat_timeout_>5.||
       !std::isfinite(recovery_timeout_)||recovery_timeout_<=heartbeat_timeout_||recovery_timeout_>120.||
       !std::isfinite(stable_duration_)||stable_duration_<.4||stable_duration_>3.||stable_duration_>=recovery_timeout_)
      throw std::invalid_argument("bounded_dependency_recovery_policy_required");
  }
  bool observe(unsigned schema,const std::string&session,uint64_t sequence,
      bool ready,bool fatal,const std::string&reason,double source,double now,
      double monotonic,const std::string&expected,
      const std::vector<FunctionalComponent>&components={},bool planning_required=true) {
    if(!required_||schema!=1||session!=expected||!sequence||sequence<=sequence_||
        !std::isfinite(source)||!std::isfinite(now)||!std::isfinite(monotonic)||
        source<=0.||now-source<-.05||now-source>heartbeat_timeout_||
        (seen_&&(monotonic<received_||source<=source_))||
        !validComponents(components,source))return false;
    // Account for a gap BEFORE receipt of its successor. A delayed callback
    // cannot skip the original recovery deadline or renew it with one packet.
    blocker(monotonic,planning_required);
    seen_=true;sequence_=sequence;source_=source;received_=monotonic;reported_ready_=ready;
    delivery_age_=std::max(0.,now-source);
    components_=components;
    reported_reason_=reason.empty()?"waiting_functional_dependencies":reason;
    selectReadiness(monotonic,planning_required);
    if(fatal&&fault_.empty())fault_="dependency_fault:"+reported_reason_;
    if(!fault_.empty())return true;
    accountReadiness(monotonic,true);
    return true;
  }
  std::string blocker(double monotonic,bool planning_required=true) {
    if(!required_)return {};
    if(fault_.empty()&&seen_) {
      if(!std::isfinite(monotonic)||monotonic<received_)fault_="dependency_clock_invalid";
      else if(monotonic-received_>heartbeat_timeout_) {
        if(admitted_)beginUnavailable(received_+heartbeat_timeout_,"dependency_heartbeat_unavailable");
        ready_=false;good_since_.reset();good_samples_=0;
      } else {selectReadiness(monotonic,planning_required);accountReadiness(monotonic,false);}
      if(fault_.empty()&&blocked_since_&&monotonic-*blocked_since_>=recovery_timeout_)
        fault_="dependency_recovery_timeout:"+blocked_reason_;
    }
    if(!fault_.empty())return fault_;
    if(!seen_)return "waiting_functional_dependencies";
    // The tree owns the measured-state stable recovery window. Report fresh
    // functional inputs immediately so its window runs in parallel. Keep our
    // episode deadline until sustained NEW monitor observations, so a brief
    // ready flash cannot keep resetting the session's recovery budget.
    if(blocked_since_&&!ready_)return blocked_reason_;
    return ready_?std::string{}:reason_;
  }
  bool fatal(double monotonic,bool planning_required=true){blocker(monotonic,planning_required);return !fault_.empty();}
private:
  static bool validComponents(const std::vector<FunctionalComponent>&values,double stamp) {
    std::set<std::string>names;bool required=false;
    for(const auto&v:values) {
      if(v.name.empty()||!names.insert(v.name).second||
         (v.seen&&(!std::isfinite(v.callback_age)||v.callback_age<0.||
          !std::isfinite(v.source)||v.source<=0.||v.source>stamp+.1)))return false;
      required|=v.required;
    }
    return values.empty()||required;
  }
  void selectReadiness(double monotonic,bool planning_required) {
    ready_=reported_ready_;reason_=reported_reason_;
    if(components_.empty())return; // Historical heartbeat contract.
    ready_=true;reason_.clear();bool selected=false;
    for(const auto&c:components_) {
      if(!c.required||(!planning_required&&c.name=="global"))continue;
      selected=true;
      if(!c.seen||c.callback_age+delivery_age_+monotonic-received_>heartbeat_timeout_) {
        ready_=false;
        if(reason_.empty())reason_=std::string(c.seen?"component_executor_stalled:":"waiting_components:")+c.name;
      }
    }
    // Never turn an empty/malformed projection into functional readiness.
    if(!selected){ready_=false;reason_="waiting_functional_dependencies";}
  }
  void accountReadiness(double monotonic,bool new_report) {
    if(!ready_) {
      if(admitted_)beginUnavailable(monotonic,reason_);
      good_since_.reset();good_samples_=0;
    } else if(!admitted_)admitted_=true;
    else if(blocked_since_) {
      if(!good_since_)good_since_=monotonic;
      if(new_report)++good_samples_;
      if(monotonic-*good_since_>=stable_duration_&&good_samples_>=3) {
        blocked_since_.reset();good_since_.reset();good_samples_=0;blocked_reason_.clear();
      }
    }
  }
  void beginUnavailable(double at,const std::string&reason) {
    if(!blocked_since_)blocked_since_=at;
    blocked_reason_=reason;
  }
  bool required_=false,seen_=false,ready_=false,reported_ready_=false,admitted_=false;
  uint64_t sequence_=0;unsigned good_samples_=0;
  double received_=0.,source_=0.,delivery_age_=0.,heartbeat_timeout_=1.5,recovery_timeout_=30.,stable_duration_=.6;
  std::optional<double>blocked_since_,good_since_;
  std::string reason_,reported_reason_,fault_,blocked_reason_;
  std::vector<FunctionalComponent>components_;
};
inline std::string taskAdmissionBlocker(bool lifecycle_active,const std::string&retirement_fault,
    bool retiring,const std::string&initial_pose,const std::string&dependency) {
  if(!lifecycle_active)return "lifecycle_not_active";
  if(!retirement_fault.empty())return retirement_fault;
  if(retiring)return "retiring_previous_task";
  if(!initial_pose.empty())return initial_pose;
  return dependency;
}

// A valid new target may replace the one waiting behind ordinary retirement.
// This does not authorize execution: confirmations and actual tree work keep
// the original retirement barrier above. Quarantine/pose-reset/drain remain
// admission blockers, so a queued intent cannot evade their identity fence.
// Stop evidence can arrive after its diagnostic budget. Retain a NEW target
// during that wait; inputsReady/confirmation still require actual retirement.
inline std::string replacementGoalAdmissionBlocker(bool lifecycle_active,
    const std::string& retirement_fault,const std::string& initial_pose,
    bool dependency_fatal,const std::string& dependency) {
  return taskAdmissionBlocker(lifecycle_active,
    retirement_fault=="previous_physical_stop_unconfirmed"?std::string{}:retirement_fault,false,initial_pose,
    dependency_fatal?dependency:std::string{});
}
} // namespace d1max_navigation_bt
