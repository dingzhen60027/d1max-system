#pragma once
#include <cmath>
#include <cstdint>
#include <string>

namespace d1max_navigation_bt {
// Functional heartbeat, not localization readiness or motion authorization.
// Sequence renewal is checked independently of receipt; replay cannot keep an
// executor alive. A dependency fatal/timeout isolates the current session.
class DependencyHealthGuard {
public:
  explicit DependencyHealthGuard(bool required=false):required_(required){}
  bool observe(unsigned schema,const std::string&session,uint64_t sequence,
      bool ready,bool fatal,const std::string&reason,double source,double now,
      double monotonic,const std::string&expected) {
    if(!required_||schema!=1||session!=expected||!sequence||sequence<=sequence_||
        !std::isfinite(source)||!std::isfinite(now)||!std::isfinite(monotonic)||
        source<=0.||now-source<-.05||now-source>.6||
        (seen_&&monotonic<received_))return false;
    seen_=true;sequence_=sequence;received_=monotonic;ready_=ready;
    reason_=reason.empty()?"waiting_functional_dependencies":reason;
    if(fatal&&fault_.empty())fault_="dependency_fault:"+reason_;
    return true;
  }
  std::string blocker(double monotonic) {
    if(!required_)return {};
    if(fault_.empty()&&seen_&&(!std::isfinite(monotonic)||monotonic<received_||monotonic-received_>.6))
      fault_="dependency_heartbeat_expired";
    if(!fault_.empty())return fault_;
    if(!seen_)return "waiting_functional_dependencies";
    return ready_?std::string{}:reason_;
  }
  bool fatal(double monotonic){blocker(monotonic);return !fault_.empty();}
private:
  bool required_=false,seen_=false,ready_=false;
  uint64_t sequence_=0;double received_=0.;std::string reason_,fault_;
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
