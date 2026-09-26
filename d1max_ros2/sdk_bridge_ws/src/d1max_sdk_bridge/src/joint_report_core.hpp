#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace d1monitor {
// The vendor JointStateData has NO acquisition timestamp, documented units or
// URDF mapping. Keep original names/values; receipt time is never source time.
struct JointPacket {
  static constexpr size_t max_joints=64, max_name=64;
  struct Joint {std::array<char,max_name+1> name{};double position=0,velocity=0,effort=0;};
  std::array<Joint,max_joints> joints{};
  size_t count=0;
  bool has_velocity=false,has_effort=false;
  double arrival=0,received=0;
};

// Check sizes BEFORE any copying/iteration. No SDK-owned dynamic container is
// retained. Cost and mailbox storage remain bounded even for malformed input.
template<class Data> bool copy_joint_packet(const Data& data,double arrival,
                                          double received,JointPacket& out) {
  const size_t n=data.names.size();
  if(!n||n>JointPacket::max_joints||data.positions.size()!=n||
     (!data.velocities.empty()&&data.velocities.size()!=n)||
     (!data.efforts.empty()&&data.efforts.size()!=n)||
     !std::isfinite(arrival)||arrival<0||!std::isfinite(received)||received<=0)return false;
  out.count=n;out.has_velocity=!data.velocities.empty();out.has_effort=!data.efforts.empty();
  out.arrival=arrival;out.received=received;
  for(size_t i=0;i<n;++i){
    const auto& name=data.names[i];
    if(name.empty()||name.size()>JointPacket::max_name)return false;
    for(const unsigned char c:name)
      if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='/'||c=='-'))return false;
    for(size_t j=0;j<i;++j)if(name==out.joints[j].name.data())return false;
    if(!std::isfinite(data.positions[i])||
       (out.has_velocity&&!std::isfinite(data.velocities[i]))||
       (out.has_effort&&!std::isfinite(data.efforts[i])))return false;
    auto& dst=out.joints[i];
    std::memcpy(dst.name.data(),name.data(),name.size());dst.name[name.size()]='\0';
    dst.position=data.positions[i];dst.velocity=out.has_velocity?data.velocities[i]:0;
    dst.effort=out.has_effort?data.efforts[i]:0;
  }
  return true;
}

// Independent from MC reporting and ownership. Configuration ACKs are advisory:
// neither an ACK nor a status heartbeat can manufacture a joint measurement.
// Three async requests per observed connection; no forced reconnect or control
// takeover. Reconnection clears all old sample/ACK/write admission evidence.
struct JointReport {
  static constexpr unsigned max_attempts=3;
  static constexpr double ready_delay=.5,retry_sec=3.,receipt_ttl=.25;
  bool enabled=false,connected=false,replay=false,acknowledged=false,ack_on=false;
  bool write_complete=false;
  unsigned attempts=0;
  uint64_t generation=0,total_attempts=0,samples=0,stale_samples=0;
  double connected_at=-1,first_request_at=-1,requested_at=-1,last_arrival=-1,last_received=0;
  std::string write_error;
  void connection(bool is_connected,bool is_replay,double now){
    const bool active=enabled&&is_connected&&!is_replay;
    if(active!=connected){
      ++generation;attempts=0;connected_at=active?now:-1;
      first_request_at=requested_at=last_arrival=-1;last_received=0;
      acknowledged=ack_on=write_complete=false;write_error.clear();
    }
    connected=active;replay=is_replay;
  }
  bool fresh(double now) const {
    return enabled&&connected&&!replay&&last_arrival>=0&&std::isfinite(now)&&
      now>=last_arrival&&now-last_arrival<receipt_ttl;
  }
  bool request_due(double now){
    if(!enabled||!connected||replay||!std::isfinite(now)||now<connected_at+ready_delay||
       fresh(now)||attempts>=max_attempts||(attempts&&now<requested_at+retry_sec))return false;
    ++attempts;++total_attempts;requested_at=now;
    if(first_request_at<0)first_request_at=now;
    write_complete=false;write_error.clear();return true;
  }
  bool sample(const JointPacket& packet,double now,double now_wall){
    if(!enabled||!connected||replay||!attempts||packet.arrival<=first_request_at)return false;
    if(!std::isfinite(now)||!std::isfinite(now_wall)||now<packet.arrival||
       now-packet.arrival>=receipt_ttl||now_wall<packet.received||
       now_wall-packet.received>=receipt_ttl||packet.arrival<=last_arrival||
       packet.received<=last_received){++stale_samples;return false;}
    last_arrival=packet.arrival;last_received=packet.received;++samples;return true;
  }
  void ack(bool on,double arrival){
    if(!connected||replay||!attempts||arrival<requested_at)return;
    acknowledged=true;ack_on=on;
  }
  void written(uint64_t epoch,uint64_t id,const std::string& error){
    if(!connected||epoch!=generation||id!=total_attempts)return;
    write_complete=true;write_error=error.substr(0,512);
  }
  std::string state(double now) const {
    if(!enabled)return "disabled";
    if(replay)return "replay_blocked";
    if(!connected)return "disconnected";
    if(fresh(now))return acknowledged&&ack_on?"streaming_receipt_only":"streaming_receipt_only_unconfirmed";
    if(attempts>=max_attempts&&now>=requested_at+retry_sec)return "retry_exhausted";
    if(!attempts)return "waiting_ready";
    if(!write_error.empty())return "write_failed";
    if(last_arrival>=0)return "stream_stale";
    return acknowledged?(ack_on?"waiting_stream":"ack_off"):"waiting_ack";
  }
};
}
