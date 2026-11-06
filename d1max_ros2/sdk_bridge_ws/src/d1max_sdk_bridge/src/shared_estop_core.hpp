#pragma once
// Local shared-panel protocol. Permission and the sole SDK writer remain in
// Monitor; parsing a request never submits a robot command.
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <vector>

namespace d1monitor::shared_estop {
using Json=nlohmann::json;
inline uint64_t decimal(const Json&j){
  if(!j.is_string())return 0;
  const auto s=j.get<std::string>();if(s.empty()||s.size()>20)return 0;
  uint64_t n=0;for(char c:s){if(c<'0'||c>'9'||n>(UINT64_MAX-uint64_t(c-'0'))/10)return 0;n=n*10+uint64_t(c-'0');}
  return n;
}
struct State {
  bool present=false,connected=false,owned=false,fatal=false,actual_speed_present=false;
  bool estop_pending=false,estop_target_on=true,estop_ack=false;
  uint64_t estop_transaction=0;
  std::string estop_result="IDLE";
  int mode=0,motion=0,source=0,speed=0,soft=0,hardware=0,head_direction=0,robot_mode=0;
  double received=0.,actual_forward=0.,actual_lateral=0.,actual_yaw=0.;
  uint64_t sequence=0,loss_sequence=0;
  Json manual={{"owner_client",""},{"lease",""},{"phase","idle"},{"pending",false}};
  Json json()const {
    return {{"present",present},{"connected",connected},{"owned",owned},{"fatal",fatal},
      {"mode",mode},{"motion",motion},{"source",source},{"speed",speed},{"soft",soft},{"hardware",hardware},
      {"head_direction",head_direction},{"robot_mode",robot_mode},{"received",received},
      {"sequence",std::to_string(sequence)},{"loss_sequence",std::to_string(loss_sequence)},
      {"estop_pending",estop_pending},{"estop_target_on",estop_target_on},
      {"estop_transaction",std::to_string(estop_transaction)},
      {"estop_result",estop_result},{"estop_ack",estop_ack},
      {"actual_speed_present",actual_speed_present},
      {"actual_forward",std::isfinite(actual_forward)?actual_forward:0.},
      {"actual_lateral",std::isfinite(actual_lateral)?actual_lateral:0.},
      {"actual_yaw",std::isfinite(actual_yaw)?actual_yaw:0.}};
  }
};
class Protocol {
public:
  enum class Kind {Ignore,Hello,Emergency,LocalOnly,Manual,Reject};
  struct Request {Kind kind=Kind::Ignore;std::string request,op,reason;bool on=true;Json arguments=Json::object();};
  explicit Protocol(std::string session,bool manual=false):session_(std::move(session)),manual_(manual){}
  Request receive(uint64_t client,const Json&j){
    std::lock_guard<std::mutex>lock(mutex_);
    if(!j.is_object())return {};
    if(j.value("type",std::string())=="hello"){
      if(j.value("version",0)!=1||!j.contains("client")||!j["client"].is_string()||j["client"].get<std::string>().empty()||j["client"].get<std::string>().size()>128||(!clients_.count(client)&&clients_.size()>=4))return {};
      // A repeated hello must not reset the command replay floor.
      clients_.emplace(client,0);names_.emplace(client,j["client"].get<std::string>());return {Kind::Hello,{},{},{}};
    }
    if(j.value("type",std::string())!="command")return {};
    Request r;r.request=j.contains("request")&&j["request"].is_string()?j["request"].get<std::string>():std::string();
    r.op=j.contains("op")&&j["op"].is_string()?j["op"].get<std::string>():std::string();
    const uint64_t sequence=j.contains("request")?decimal(j["request"]):0;
    auto found=clients_.find(client);
    if(found==clients_.end()||j.value("version",0)!=1||j.value("sdk_session",std::string())!=session_||!sequence){r.kind=Kind::Reject;r.reason="shared_estop_protocol_or_session_invalid";return r;}
    if(sequence<=found->second){r.kind=Kind::Reject;r.reason="shared_estop_request_replayed";return r;}
    found->second=sequence;
    if(r.op=="emergency"){
      if(j.contains("on")&&!j["on"].is_boolean()){r.kind=Kind::Reject;r.reason="software_estop_target_invalid";return r;}
      r.on=j.value("on",true); // Old emergency-only clients remain explicit ON.
      r.kind=Kind::Emergency;return r;
    }
    if(manual_&&(r.op=="manual_take"||r.op=="manual_release"||r.op=="posture"||r.op=="speed"||
       r.op=="jog"||r.op=="heartbeat"||r.op=="release_jog"||r.op=="stop"||r.op=="detach")){
      r.kind=Kind::Manual;r.arguments=j;return r;
    }
    if(r.op=="stop"||r.op=="release_jog"||r.op=="detach"){
      r.kind=Kind::LocalOnly;r.reason="local_detach_only_no_sdk_command";return r;
    }
    r.kind=Kind::Reject;r.reason="shared_session_emergency_only";return r;
  }
  // Registration and the first state broadcast run on different threads.
  // Expose a peer to broadcasters only AFTER its hello is queued, so the
  // receiver can establish the session before processing any state packet.
  void helloQueued(uint64_t client){std::lock_guard<std::mutex>lock(mutex_);if(clients_.count(client))ready_.insert(client);}
  void disconnect(uint64_t client){std::lock_guard<std::mutex>lock(mutex_);clients_.erase(client);names_.erase(client);ready_.erase(client);}
  std::string clientName(uint64_t client)const{std::lock_guard<std::mutex>lock(mutex_);auto i=names_.find(client);return i==names_.end()?std::string():i->second;}
  std::vector<uint64_t>clients()const {std::lock_guard<std::mutex>lock(mutex_);return {ready_.begin(),ready_.end()};}
  Json capabilities()const {auto out=Json::array({"emergency","emergency_release"});if(manual_)out.push_back("manual_control_v1");return out;}
  Json hello(const State&s)const{return {{"type","hello"},{"version",1},{"sdk_session",session_},{"capabilities",capabilities()},{"mode",manual_?"shared_manual":"emergency_only"},{"state",s.json()},{"manual",s.manual}};}
  Json state(const State&s)const{return {{"type","state"},{"sdk_session",session_},{"capabilities",capabilities()},{"state",s.json()},{"manual",s.manual}};}
  Json result(const Request&r,bool ok,const std::string&why)const{
    Json out={{"type","result"},{"sdk_session",session_},{"request",r.request},{"op",r.op},{"ok",ok},{"reason",why}};
    if(r.op=="emergency")out["on"]=r.on;
    return out;
  }
private:
  std::string session_;bool manual_=false;mutable std::mutex mutex_;std::map<uint64_t,uint64_t>clients_;std::map<uint64_t,std::string>names_;std::set<uint64_t>ready_;
};
// SDK callbacks only finish this small mailbox. They never serialize JSON,
// publish ROS data, perform socket I/O, or infer physical standstill.
class WriteResult {
public:
  void finish(const std::error_code&e){std::lock_guard<std::mutex>lock(mutex_);if(done_)return;error_=e;done_=true;}
  std::pair<bool,std::error_code>read()const{std::lock_guard<std::mutex>lock(mutex_);return {done_,error_};}
private:
  mutable std::mutex mutex_;bool done_=false;std::error_code error_;
};
// Shared lifetime is independent of Monitor. Disconnect or node destruction
// cannot turn a delayed vendor write callback into a dangling-node access.
struct WriteInbox {
  std::mutex mutex;std::error_code error;
  void finish(const std::error_code&e){if(e){std::lock_guard<std::mutex>lock(mutex);error=e;}}
  std::error_code take(){std::lock_guard<std::mutex>lock(mutex);const auto out=error;error.clear();return out;}
};
}
