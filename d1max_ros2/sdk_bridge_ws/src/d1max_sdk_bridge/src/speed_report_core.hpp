#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

namespace d1monitor {
// Telemetry configuration only. No robot control or socket ownership here.
// Caller serializes access; monotonic time, never ROS/wall time, drives deadlines.
struct SpeedReport {
  unsigned frequency=50, max_attempts=3, attempts=0, ack_hz=0;
  double ready_delay=1., retry_sec=3., stale_sec=.3, window_sec=2.;
  bool connected=false, replay=false, acknowledged=false, ack_on=false, write_complete=false;
  uint64_t generation=0, samples=0, invalid_samples=0;
  double first_robot=-1., last_robot=-1., requested_at=-1.;
  std::string write_error;
  std::deque<double> arrivals;

  void validate() const {
    if(frequency<1||frequency>50||max_attempts<1||max_attempts>5||
       !std::isfinite(ready_delay)||ready_delay<0||!std::isfinite(retry_sec)||retry_sec<2.5||
       !std::isfinite(stale_sec)||stale_sec<=0||!std::isfinite(window_sec)||window_sec<1.)
      throw std::invalid_argument("invalid dedicated speed stream configuration");
  }
  void disconnect() {
    ++generation;connected=false;attempts=0;samples=0;invalid_samples=0;
    first_robot=last_robot=requested_at=-1.;acknowledged=ack_on=write_complete=false;
    ack_hz=0;write_error.clear();arrivals.clear();
  }
  void robot(double now) {if(first_robot<0)first_robot=now;last_robot=now;}
  void prune(double now) {while(!arrivals.empty()&&now-arrivals.front()>window_sec)arrivals.pop_front();}
  bool sample(double now,double x,double y,double yaw) {
    if(!connected||replay)return false;
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)){++invalid_samples;return false;}
    ++samples;arrivals.push_back(now);prune(now);return true;
  }
  bool fresh(double now) const {return connected&&!replay&&!arrivals.empty()&&now>=arrivals.back()&&now-arrivals.back()<stale_sec;}
  double observed_hz(double now) const {
    if(!fresh(now)||arrivals.size()<2)return 0.;
    const double span=arrivals.back()-arrivals.front();
    return span>=1.?(arrivals.size()-1)/span:0.;
  }
  bool rate_ok(double now) const {const auto hz=observed_hz(now);return hz>=frequency*.8&&hz<=frequency*1.2;}
  bool request_due(bool is_connected,bool is_replay,double now) {
    if(!is_connected){if(connected||first_robot>=0||attempts)disconnect();return false;}
    connected=true;replay=is_replay;prune(now);
    if(replay||first_robot<0||now-first_robot<ready_delay||now-last_robot>2.5)return false;
    // Stop retransmitting once the requested data rate is observed, even if an
    // ACK is missing. An ACK alone never substitutes for real speed samples.
    if(rate_ok(now)||attempts>=max_attempts)return false;
    if(attempts&&now-requested_at<retry_sec)return false;
    ++attempts;requested_at=now;write_complete=false;write_error.clear();return true;
  }
  void written(uint64_t epoch,unsigned attempt,const std::string& error) {
    if(epoch!=generation||attempt!=attempts)return;
    write_complete=true;if(!error.empty())write_error=error;
  }
  void ack(bool on,unsigned hz) {if(!connected)return;acknowledged=true;ack_on=on;ack_hz=hz;}
  std::string state(double now) const {
    if(!connected)return "disconnected";
    if(replay)return "replay_blocked";
    if(first_robot<0||now-last_robot>2.5)return "waiting_robot_state";
    if(rate_ok(now))return "streaming";
    if(fresh(now))return observed_hz(now)>0?"rate_mismatch":"measuring_rate";
    if(attempts>=max_attempts&&now-requested_at>=retry_sec)return "failed";
    if(!attempts)return "waiting_ready";
    if(!write_error.empty())return "write_failed";
    if(acknowledged&&(!ack_on||ack_hz!=frequency))return "ack_mismatch";
    return acknowledged?"waiting_stream":"waiting_ack";
  }
};
}
