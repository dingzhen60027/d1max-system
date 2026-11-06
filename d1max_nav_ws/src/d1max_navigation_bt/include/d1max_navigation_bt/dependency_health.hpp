#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace d1max_navigation_bt {
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
      double monotonic,const std::string&expected) {
    if(!required_||schema!=1||session!=expected||!sequence||sequence<=sequence_||
        !std::isfinite(source)||!std::isfinite(now)||!std::isfinite(monotonic)||
        source<=0.||now-source<-.05||now-source>heartbeat_timeout_||
        (seen_&&(monotonic<received_||source<=source_)))return false;
    // Account for a gap BEFORE receipt of its successor. A delayed callback
    // cannot skip the original recovery deadline or renew it with one packet.
    blocker(monotonic);
    seen_=true;sequence_=sequence;source_=source;received_=monotonic;ready_=ready;
    reason_=reason.empty()?"waiting_functional_dependencies":reason;
    if(fatal&&fault_.empty())fault_="dependency_fault:"+reason_;
    if(!fault_.empty())return true;
    if(!ready_) {
      if(admitted_)beginUnavailable(monotonic,reason_);
      good_since_.reset();good_samples_=0;
    } else if(!admitted_) {
      // Cold preparation has the producer's startup budget. Do not create a
      // second 30-second startup deadline before the first healthy snapshot.
      admitted_=true;
    } else if(blocked_since_) {
      if(!good_since_)good_since_=monotonic;
      ++good_samples_; // observe() admitted a new sequence AND source time.
      if(monotonic-*good_since_>=stable_duration_&&good_samples_>=3) {
        blocked_since_.reset();good_since_.reset();good_samples_=0;
        blocked_reason_.clear();
      }
    }
    return true;
  }
  std::string blocker(double monotonic) {
    if(!required_)return {};
    if(fault_.empty()&&seen_) {
      if(!std::isfinite(monotonic)||monotonic<received_)fault_="dependency_clock_invalid";
      else if(monotonic-received_>heartbeat_timeout_) {
        if(admitted_)beginUnavailable(received_+heartbeat_timeout_,"dependency_heartbeat_unavailable");
        ready_=false;good_since_.reset();good_samples_=0;
      }
      if(fault_.empty()&&blocked_since_&&monotonic-*blocked_since_>=recovery_timeout_)
        fault_="dependency_recovery_timeout:"+blocked_reason_;
    }
    if(!fault_.empty())return fault_;
    if(!seen_)return "waiting_functional_dependencies";
    if(blocked_since_)return ready_?"confirming_dependency_recovery":blocked_reason_;
    return ready_?std::string{}:reason_;
  }
  bool fatal(double monotonic){blocker(monotonic);return !fault_.empty();}
private:
  void beginUnavailable(double at,const std::string&reason) {
    if(!blocked_since_)blocked_since_=at;
    blocked_reason_=reason;
  }
  bool required_=false,seen_=false,ready_=false,admitted_=false;
  uint64_t sequence_=0;unsigned good_samples_=0;
  double received_=0.,source_=0.,heartbeat_timeout_=1.5,recovery_timeout_=30.,stable_duration_=.6;
  std::optional<double>blocked_since_,good_since_;
  std::string reason_,fault_,blocked_reason_;
};
inline std::string taskAdmissionBlocker(bool lifecycle_active,const std::string&retirement_fault,
    bool retiring,const std::string&initial_pose,const std::string&dependency) {
  if(!lifecycle_active)return "lifecycle_not_active";
  if(!retirement_fault.empty())return retirement_fault;
  if(retiring)return "retiring_previous_task";
  if(!initial_pose.empty())return initial_pose;
  return dependency;
}
} // namespace d1max_navigation_bt
