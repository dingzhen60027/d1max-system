#pragma once
#include "shared_estop_core.hpp"
#include <optional>
#include <set>

namespace d1monitor::shared_manual {
using shared_estop::Json;
using shared_estop::State;
using Request=shared_estop::Protocol::Request;
enum class Action {None,Mode,Stand,Lie,Crawl,Lock,Speed};
struct Velocity {float lateral=0,forward=0,yaw=0;bool nonzero()const{return lateral!=0||forward!=0||yaw!=0;}};
struct Decision {bool accepted=false,pending=false;std::string reason;};
struct Pending {Request request;Action action=Action::None;int value=0;double started=0,sent=0;uint64_t state_before=0,ack_before=0;bool submitted=false;};
inline int standing(int mode){return mode>=1&&mode<=3?mode+4:0;}
inline bool fresh(const State&s,double now){return s.present&&now>=s.received&&now-s.received<=3.;}
inline bool matches(Action a,int value,const State&s){
  switch(a){case Action::Mode:return s.mode==value&&s.motion==standing(value);case Action::Stand:return s.motion==standing(s.mode)&&standing(s.mode);
    case Action::Lie:return s.motion==2;case Action::Crawl:return s.motion==3;case Action::Lock:return s.motion==4;
    case Action::Speed:return s.speed==value;default:return false;}
}
// No SDK/ROS dependency. All state is owned by Monitor's state mutex; only
// the existing velocity writer consumes it. IPC is input, never a sender.
class Core {
public:
  enum class Phase {Idle,Acquiring,Manual,Releasing};
  Phase phase=Phase::Idle;
  uint64_t client=0,lease_number=0,generation=0;
  std::string client_name,lease,key,reason;
  double heartbeat_until=0;
  int percent=0;
  bool zero_due=false,zero_confirmed=true,uncertain=false,take_requested=false;
  std::optional<Pending> pending;
  std::set<std::string> retired_tasks;
  static std::string taskKey(const std::string&session,const std::string&task){return session+":"+task;}
  bool active()const{return phase!=Phase::Idle;}
  // Estop/loss may retire the panel lease before its final zero can be sent.
  // This debt belongs to the sole writer, not to the now-retired UI owner.
  // Never erase it or infer an ACK merely because the phase became Idle.
  bool needsWriterService()const{return active()||zero_due;}
  bool owns()const{return phase==Phase::Manual;}
  bool held(double now)const{return !key.empty()&&now<heartbeat_until;}
  std::string phaseName()const{return phase==Phase::Acquiring?"acquiring":phase==Phase::Manual?"manual":phase==Phase::Releasing?"retiring":"idle";}
  std::string sdkPhase()const{return phase==Phase::Acquiring?"manual_takeover_pending":phase==Phase::Manual?"manual_active":phase==Phase::Releasing?"manual_releasing":"";}
  Json json()const{return {{"owner_client",client_name},{"lease",lease},{"phase",phaseName()},
    {"pending",bool(pending)||phase==Phase::Acquiring||phase==Phase::Releasing},{"reason",reason}};}
  bool current(uint64_t c,const Json&j)const{return c==client&&j.value("lease",std::string())==lease&&!lease.empty();}
  std::string deny(const State&s,double now,bool moving=false)const{
    if(!s.connected)return "SDK 未连接";
    if(!fresh(s,now))return "等待机器人状态";
    if(!s.owned||s.source!=2)return "等待 SDK 控制权确认";
    if(s.soft==2||s.hardware==2)return "急停已触发";
    if(s.fatal)return "机器人上报故障";
    if(uncertain)return "上一次 SDK 操作结果未知；请核对机器人状态";
    if(s.motion==1)return "机器人正在起身";
    if(moving&&!((s.mode==1&&s.motion==5)||(s.mode==3&&s.motion==7)))return "请先站立到通用或登阶模式";
    return {};
  }
  void stop(){if(!zero_confirmed||!key.empty()){zero_due=true;zero_confirmed=false;}key.clear();heartbeat_until=0;percent=0;}
  void expire(double now){if(!key.empty()&&now>=heartbeat_until){stop();reason="manual_heartbeat_expired";}}
  void detach(uint64_t c){if(c==client&&active()){stop();phase=Phase::Releasing;reason="manual_panel_detached";}}
  void veto(const std::string&why){if(active()){stop();phase=Phase::Releasing;reason=why;}}
  void releaseComplete(){phase=Phase::Idle;client=0;client_name.clear();lease.clear();pending.reset();key.clear();heartbeat_until=0;take_requested=false;}
  bool navigationReady(double now)const{return (!active()||owns())&&!pending&&!held(now)&&!zero_due&&zero_confirmed&&!uncertain;}
  bool allowsTask(const std::string&session,const std::string&task)const{return !retired_tasks.count(taskKey(session,task));}
  Decision receive(uint64_t c,const std::string&name,const Request&r,const State&s,double now,const std::string&old_task){
    const auto&j=r.arguments;
    if(r.op=="manual_take"){
      if(active())return {c==client,false,c==client?"manual_control_already_requested":"另一个面板正在手动控制"};
      if(!s.connected||!fresh(s,now))return {false,false,"等待机器人连接和状态"};
      if(s.source==1)return {false,false,"APP 正在控制，请先退出 APP 控制"};
      if(s.soft==2||s.hardware==2||s.fatal)return {false,false,"请先解除急停或处理机器人故障"};
      if(uncertain)return {false,false,"上一次 SDK 操作结果未知"};
      if(retired_tasks.size()>=1024)return {false,false,"手动任务退役记录已满，请重新建立 SDK 会话"};
      if(!old_task.empty())retired_tasks.insert(old_task);
      client=c;client_name=name;lease=std::to_string(++lease_number);generation=0;phase=Phase::Acquiring;reason.clear();take_requested=false;
      pending=Pending{r,Action::None,0,now};return {true,true,"manual_takeover_pending"};
    }
    if(r.op=="detach"){detach(c);return {true,false,"panel_detached"};}
    if(!current(c,j))return {false,false,"manual_lease_not_owned"};
    if(r.op=="stop"||r.op=="release_jog"){
      const auto g=j.contains("generation")?shared_estop::decimal(j["generation"]):0;
      if(r.op=="release_jog"&&g&&g!=generation)return {true,false,"old_key_release_ignored"};
      stop();return {true,false,"manual_stop_queued"};
    }
    if(r.op=="manual_release"){
      stop();phase=Phase::Releasing;reason="manual_return_to_navigation";return {true,false,"manual_release_queued"};
    }
    if(!owns())return {false,false,"manual_takeover_not_complete"};
    if(pending)return {false,false,"等待当前操作完成"};
    const bool moving=r.op=="jog"||r.op=="heartbeat";
    if(const auto why=deny(s,now,moving);!why.empty())return {false,false,why};
    if(moving){
      if(!j.contains("generation")||!j.contains("percent")||!j["percent"].is_number_integer()||!j.contains("key")||!j["key"].is_string())return {false,false,"manual_jog_fields_invalid"};
      const auto g=shared_estop::decimal(j["generation"]);const auto k=j["key"].get<std::string>();const int power=j["percent"].get<int>();
      if(!g||k.size()!=1||std::string("wsadjl").find(k)==std::string::npos||power<1||power>100)return {false,false,"manual_jog_fields_invalid"};
      if(r.op=="heartbeat"){
        if(g!=generation||k!=key||power!=percent||!held(now))return {false,false,"manual_jog_heartbeat_stale"};
      }else if(g<=generation)return {false,false,"manual_jog_generation_stale"};
      generation=g;key=k;percent=power;heartbeat_until=now+.30;zero_confirmed=false;
      return {true,false,"manual_jog_accepted"};
    }
    Action action=Action::None;int value=0;
    if(r.op=="posture"){
      if(!j.contains("action")||!j["action"].is_number_integer()||!j.contains("mode")||!j["mode"].is_number_integer())return {false,false,"manual_posture_fields_invalid"};
      const int a=j["action"].get<int>();value=j["mode"].get<int>();
      if(value<1||value>3||a<3||a>6)return {false,false,"manual_posture_fields_invalid"};
      action=a==3?(s.mode==value?Action::Stand:Action::Mode):a==4?Action::Lie:a==5?Action::Crawl:Action::Lock;
    }else if(r.op=="speed"){
      if(!j.contains("level")||!j["level"].is_number_integer())return {false,false,"manual_speed_invalid"};
      value=j["level"].get<int>();if(value<1||value>3)return {false,false,"manual_speed_invalid"};action=Action::Speed;
    }else return {false,false,"manual_operation_unknown"};
    if(matches(action,value,s))return {true,false,"当前已是所选状态"};
    stop();pending=Pending{r,action,value,now};return {true,true,"manual_operation_pending"};
  }
  Velocity velocity(double now)const{const float f=percent*.01f;if(!held(now))return {};
    if(key=="w")return {0,f,0};
    if(key=="s")return {0,-f,0};
    if(key=="a")return {f,0,0};
    if(key=="d")return {-f,0,0};
    if(key=="j")return {0,0,f};
    if(key=="l")return {0,0,-f};
    return {};
  }
};
}
