#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

namespace d1monitor {
// Fixed storage, O(1) callback copy, discard oldest on overflow. Externally locked.
template<class T, size_t Capacity> struct Inbox {
  static_assert(Capacity > 0);
  std::array<T, Capacity> data{};
  size_t head=0, size=0;
  uint64_t dropped=0;
  void push(const T& value) {
    if(size==Capacity){head=(head+1)%Capacity;--size;++dropped;}
    data[(head+size)%Capacity]=value;++size;
  }
  bool pop(T& value) {
    if(!size)return false;
    value=data[head];head=(head+1)%Capacity;--size;return true;
  }
};

// SetMcConfig only switches reporting on/off. expected_hz is a diagnostic
// reference from this SDK's docs, NOT a frequency argument sent to the robot.
struct McReport {
  static constexpr double expected_hz=50.;
  // max_attempts is a per-burst budget, never a lifetime retry limit.
  unsigned max_attempts=3, attempts=0, retry_cycles=0;
  uint64_t total_attempts=0;
  double ready_delay=1., retry_sec=3., stale_sec=.3;
  double cooldown_sec=15., max_cooldown_sec=60., stream_timeout_sec=1.;
  bool connected=false,replay=false,acknowledged=false,ack_on=false,write_complete=false;
  uint64_t generation=0,samples=0,invalid_samples=0,timestamp_rejections=0,stale_samples=0;
  uint64_t last_source=0,anchor_source=0;
  double first_robot=-1.,last_robot=-1.,requested_at=-1.,last_arrival=-1.;
  double first_request_at=-1.,stable_since=-1.;
  double anchor_wall=0.,stamp_unix=0.;
  std::string write_error;
  struct Timing {double arrival;uint64_t source;};
  std::deque<Timing> timings;
  void validate() const {
    if(max_attempts<1||max_attempts>5||!std::isfinite(ready_delay)||ready_delay<0||
       !std::isfinite(retry_sec)||retry_sec<2.5||retry_sec>30||
       !std::isfinite(cooldown_sec)||cooldown_sec<retry_sec||
       !std::isfinite(max_cooldown_sec)||max_cooldown_sec<cooldown_sec||max_cooldown_sec>300||
       !std::isfinite(stream_timeout_sec)||stream_timeout_sec<stale_sec||stream_timeout_sec>10)
      throw std::invalid_argument("invalid MC report retry configuration");
  }
  void disconnect() {
    ++generation;connected=false;attempts=retry_cycles=0;total_attempts=0;
    samples=invalid_samples=timestamp_rejections=stale_samples=0;
    first_robot=last_robot=requested_at=last_arrival=-1.;
    first_request_at=stable_since=-1.;
    last_source=anchor_source=0;anchor_wall=stamp_unix=0.;timings.clear();
    acknowledged=ack_on=write_complete=false;write_error.clear();
  }
  void robot(double now){
    if(first_robot<0)first_robot=now;
    last_robot=now;
  }
  void prune(double now){while(!timings.empty()&&now-timings.front().arrival>2.)timings.pop_front();}
  bool fresh(double now) const {return connected&&!replay&&last_arrival>=0&&now>=last_arrival&&now-last_arrival<stale_sec;}
  double observed_hz(double now) const {
    if(!fresh(now)||timings.size()<2)return 0.;
    const auto span=timings.back().arrival-timings.front().arrival;
    return span>=1.?(timings.size()-1)/span:0.;
  }
  double source_hz(double now) const {
    if(!fresh(now)||timings.size()<2)return 0.;
    const double span=(timings.back().source-timings.front().source)*1e-9;
    return span>=1.?(timings.size()-1)/span:0.;
  }
  bool rate_ok(double now) const {const auto hz=observed_hz(now);return hz>=expected_hz*.8&&hz<=expected_hz*1.2;}
  double cooldown() const {
    return std::min(max_cooldown_sec,cooldown_sec*std::pow(2.,std::min(retry_cycles,8u)));
  }
  double next_retry_in(double now) const {
    if(!connected||replay||first_robot<0||now-last_robot>2.5||fresh(now))return -1.;
    double due=first_robot+ready_delay;
    if(total_attempts)due=std::max(due,requested_at+retry_sec+(attempts>=max_attempts?cooldown():0.));
    if(last_arrival>=0)due=std::max(due,last_arrival+stream_timeout_sec);
    return std::max(0.,due-now);
  }
  bool sample(double arrival,double received,uint64_t source,const float* v,const float* omega,double now) {
    if(!connected||replay||!total_attempts||arrival<first_request_at)return false;
    if(now<arrival||now-arrival>=stale_sec){++stale_samples;return false;}
    for(int i=0;i<3;++i)if(!std::isfinite(v[i])||!std::isfinite(omega[i])){++invalid_samples;return false;}
    if(!source||source<=last_source){++timestamp_rejections;return false;}
    // SDK timestamp epoch is undocumented: retain uint64 ns and use source
    // deltas anchored to the first host receipt. This is approximate, NOT PTP.
    const auto candidate=anchor_source?anchor_wall+(source-anchor_source)*1e-9:received;
    if(!std::isfinite(received)||!std::isfinite(candidate)||std::abs(candidate-received)>=stale_sec){++timestamp_rejections;return false;}
    if(!anchor_source){anchor_source=source;anchor_wall=received;}
    if(last_arrival<0||arrival-last_arrival>=stale_sec){stable_since=arrival;timings.clear();}
    last_source=source;last_arrival=arrival;stamp_unix=candidate;++samples;
    timings.push_back({arrival,source});prune(now);
    // Bound memory even under a faulty callback flood.
    while(timings.size()>512)timings.pop_front();
    return true;
  }
  bool request_due(bool is_connected,bool is_replay,double now,bool allow_request=true) {
    if(!is_connected){if(connected||first_robot>=0||attempts)disconnect();return false;}
    connected=true;replay=is_replay;prune(now);
    if(replay||first_robot<0||now-first_robot<ready_delay||now-last_robot>2.5)return false;
    // Actual data wins over a lost configuration ACK: never disturb a live stream.
    if(fresh(now)) {
      if(stable_since>=0&&now-stable_since>=2.&&rate_ok(now))attempts=retry_cycles=0;
      return false;
    }
    if(!allow_request||next_retry_in(now)>0)return false;
    if(attempts>=max_attempts){attempts=0;++retry_cycles;}
    ++attempts;++total_attempts;
    if(first_request_at<0)first_request_at=now;
    requested_at=now;write_complete=false;write_error.clear();return true;
  }
  void written(uint64_t epoch,uint64_t attempt,const std::string& error) {
    if(epoch!=generation||attempt!=total_attempts)return;
    write_complete=true;if(!error.empty())write_error=error;
  }
  void ack(bool on){if(!connected||replay||!total_attempts)return;acknowledged=true;ack_on=on;}
  std::string state(double now) const {
    if(!connected)return "disconnected";
    if(replay)return "replay_blocked";
    if(first_robot<0||now-last_robot>2.5)return "waiting_robot_state";
    if(fresh(now)) {
      if(!acknowledged)return "streaming_unconfirmed";
      if(!ack_on)return "ack_off";
      return rate_ok(now)?"streaming":observed_hz(now)>0?"rate_mismatch":"measuring_rate";
    }
    if(attempts>=max_attempts&&now-requested_at>=retry_sec)return "retry_cooldown";
    if(!total_attempts)return "waiting_ready";
    if(last_arrival>=0&&now-last_arrival>=stale_sec&&requested_at<last_arrival)return "stream_stale";
    if(!write_error.empty())return "write_failed";
    return acknowledged&&ack_on?"waiting_stream":"waiting_ack";
  }
};
}
