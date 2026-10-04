#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace d1monitor::execution3 {
// Finite idempotent zero-write transaction. An ACK is only submission evidence;
// this helper can NEVER certify physical standstill or reacquire control.
class ShutdownStopTransaction {
 public:
  explicit ShutdownStopTransaction(double started,bool required):started_(started),required_(required) {
    if(!std::isfinite(started))throw std::invalid_argument("invalid_shutdown_clock");
    if(!required_)reason_="no_active_execution";
  }
  bool due(double now,bool stop_allowed) {
    if(done(now))return false;
    if(!stop_allowed){finished_=true;reason_="shutdown_stop_not_allowed_no_retake";return false;}
    if(attempts_>=3||now-last_attempt_<.05)return false;
    ++attempts_;last_attempt_=now;reason_="shutdown_zero_pending_ack";return true;
  }
  void acknowledge(bool success) {
    if(!required_||finished_)return;
    submitted_=success;finished_=true;
    reason_=success?"shutdown_zero_submitted_stop_unconfirmed":"shutdown_zero_write_outcome_unknown";
  }
  bool done(double now) {
    if(!required_||finished_)return true;
    if(!std::isfinite(now)||now<started_||now-started_>=.30){
      finished_=true;reason_="shutdown_zero_timeout_stop_unconfirmed";return true;
    }
    return false;
  }
  bool submitted()const{return submitted_;}
  unsigned attempts()const{return attempts_;}
  const std::string& reason()const{return reason_;}
 private:
  double started_=0,last_attempt_=-1e9;
  bool required_=false,finished_=false,submitted_=false;
  unsigned attempts_=0;std::string reason_;
};
}
