#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace d1max_localization {
// Live, shared sensor epoch -> host epoch. Not NTP/PTP and not a latency estimator.
// Fit ONCE using the lowest observed IMU receipt delay; all scans/IMUs receive
// the same additive offset. Never restamp individual scans with receipt time.
class InputClock {
 public:
  InputClock(bool estimate=false, unsigned samples=100, double span=1.0)
      : estimate_(estimate), samples_(samples), span_(span), locked_(!estimate) {
    if(samples<2 || !std::isfinite(span) || span<=0.)throw std::invalid_argument("invalid clock calibration");
  }
  bool observe(double sensor, double host, double steady) {
    if(fault_ || !finite(sensor,host,steady))return false;
    if(count_ && sensor<=last_sensor_) {
      if(sensor<last_sensor_-.5)fault_="sensor clock jumped backwards; restart required";
      return false;
    }
    if(count_ && std::abs((host-last_host_)-(steady-last_steady_))>.1) {
      fault_="host clock jumped; restart required";return false;
    }
    // A delayed packet is not evidence of a changed sensor epoch. Drop it
    // without updating the accepted baseline or fitting a new offset. A fresh
    // packet can resume on the SAME time axis after a network/CPU stall.
    if(locked_ && host-(sensor+offset_)>.3)return false;
    if(count_ && std::abs((sensor-last_sensor_)-(steady-last_steady_))>.5) {
      fault_="sensor time discontinuity; restart required";return false;
    }
    if(!locked_) {
      if(!count_)first_sensor_=sensor;
      candidate_=std::min(candidate_,host-sensor);
    } else {
      const double age=host-(sensor+offset_);
      if(age<-.05 || age>.3)return false;
    }
    last_sensor_=sensor;last_host_=host;last_steady_=steady;++count_;
    if(!locked_ && count_>=samples_ && sensor-first_sensor_>=span_) {
      offset_=candidate_;locked_=true;
    }
    return locked_;
  }
  std::optional<double> offset(double steady) const {
    if(!locked_ || fault_ || !count_ || steady<last_steady_ || steady-last_steady_>.3)return std::nullopt;
    return offset_;
  }
  void disableForReplay(){fault_="ROS clock/replay detected; live epoch compensation disabled";}
  const char* state() const {return fault_?fault_:locked_?(estimate_?"estimated_shared_epoch":"strict"):"calibrating";}
  bool approximate() const{return estimate_;}
  bool faulted() const{return fault_!=nullptr;}
  unsigned count() const{return count_;}
 private:
  static bool finite(double a,double b,double c){return std::isfinite(a)&&a>0.&&std::isfinite(b)&&b>0.&&std::isfinite(c);}
  bool estimate_;unsigned samples_;double span_;bool locked_;
  const char* fault_=nullptr;unsigned count_=0;
  double first_sensor_=0.,last_sensor_=0.,last_host_=0.,last_steady_=0.;
  double offset_=0.,candidate_=std::numeric_limits<double>::infinity();
};
}
