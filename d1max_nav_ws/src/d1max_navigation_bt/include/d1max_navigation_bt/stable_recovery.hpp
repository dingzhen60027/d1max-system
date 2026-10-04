#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace d1max_navigation_bt {
// Safety withdraws on the first unavailable observation. Recovery is a separate
// operation: report heartbeats cannot manufacture new measurement evidence.
// Initial admission is owned by startup/arming, not by a motion-progress gate.
class StableRecoveryGate {
public:
  explicit StableRecoveryGate(double window=.6,unsigned samples=3)
    :window_(window),required_(samples) {
    if(!std::isfinite(window)||window<.4||window>3.||samples<3||samples>100)
      throw std::invalid_argument("bounded_source_evidenced_recovery_required");
  }
  bool observe(bool ready,double monotonic,std::int64_t source_ns=0,const std::string& identity={}) {
    if(!std::isfinite(monotonic)||(last_&&monotonic<*last_)) {
      withdraw();clock_fault_=true;return false;
    }
    last_=monotonic;
    if(clock_fault_)return false;
    if(!ready){if(admitted_)recovering_=true;clearWindow();return false;}
    // Do not require measured motion or a recovery window before first use.
    if(!admitted_){admitted_=true;identity_=identity;watermark_=source_ns;return true;}
    if(!identity_.empty()&&identity!=identity_){recovering_=true;clearWindow();return false;}
    if(!recovering_) {
      if(identity_.empty()&&!identity.empty())identity_=identity;
      watermark_=std::max(watermark_,source_ns);return true;
    }
    if(source_ns<=0||identity.empty()){clearWindow();return false;}
    if(identity_.empty())identity_=identity;
    if(!good_since_)good_since_=monotonic;
    if(source_ns>watermark_){watermark_=source_ns;++samples_;}
    if(monotonic-*good_since_>=window_&&samples_>=required_) {
      recovering_=false;clearWindow();return true;
    }
    return false;
  }
  bool recovering()const{return recovering_;}
  void withdraw(){if(admitted_)recovering_=true;clearWindow();}
private:
  void clearWindow(){good_since_.reset();samples_=0;}
  double window_;unsigned required_,samples_=0;std::int64_t watermark_=0;
  bool admitted_=false,recovering_=false,clock_fault_=false;
  std::string identity_;std::optional<double>last_,good_since_;
};
} // namespace d1max_navigation_bt
