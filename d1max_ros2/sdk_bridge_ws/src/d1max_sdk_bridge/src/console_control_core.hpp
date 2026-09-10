#pragma once
// Side-effect-free control state machine. Tests use virtual time and no SDK/ROS.
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace d1console {
struct State {
  int motion=0, mode=0, control=0, speed=0, software=0, hardware=0;
  double x=0, y=0, yaw=0, at=-1e9;
  unsigned long sequence=0;
};
struct Config {
  double stale=2.5, lease_timeout=.6, velocity_timeout=.25, transition_timeout=15;
  double max_x=.30, max_y=.20, max_yaw=.50;
  int speed_level=1, confirmations=2;
};
struct Command { std::string name; double x=0,y=0,yaw=0; unsigned long generation=0; };
class Core {
 public:
  Config config;
  State state;
  bool connected=false, replay=false, fault=false, owned=false, prepared=false;
  bool uncertain=false, moving=false;
  std::string goal, active, result="IDLE", error;
  unsigned long goal_id=0, generation=0, sent_count=0;
  std::vector<std::string> events;
  explicit Core(Config c={}) : config(c) {}
  bool fresh(double now) const { return now-state.at>=0 && now-state.at<=config.stale; }
  static bool upright(int m) { return m==5 || m==6 || m==7; } // 1 means STILL STANDING UP.
  bool stationary() const { return std::isfinite(state.x)&&std::isfinite(state.y)&&std::isfinite(state.yaw)&&std::abs(state.x)<.03&&std::abs(state.y)<.03&&std::abs(state.yaw)<.03; }
  bool safe() const { return state.software==1 && state.hardware==1; }
  bool lease_valid(double now) const { return lease_ && now-lease_at_<=config.lease_timeout; }
  bool busy() const { return !active.empty() || !queue_.empty(); }
  bool ready(double now) const { return connected&&!replay&&!fault&&!uncertain&&lease_valid(now)&&fresh(now)&&safe()&&std::isfinite(state.x)&&std::isfinite(state.y)&&std::isfinite(state.yaw)&&owned&&state.control==2&&state.motion==5&&state.mode==1&&state.speed==config.speed_level&&prepared&&!busy(); }
  void lease(bool armed,const std::string& session,double now) {
    if (session_!=session) { cancel("控制会话改变",false); session_=session; }
    lease_=armed; lease_at_=now;
    if(!armed && goal!="soft_estop" && goal!="release_control") cancel("操作已锁定",false);
  }
  void connection(bool value) {
    if(connected&&!value) { cancel("SDK 断开；禁止自动续跑",true); owned=false; }
    connected=value;
  }
  void control_lost() { owned=false; cancel("SDK 控制权丢失",true); }
  void fault_report() { fault=true; cancel("Error/Fatal 故障已锁存；排障后重启",true); }
  void replay_detected() { replay=true; lease_=false; cancel("检测到回放，控制锁存禁用",true); }
  void update(State s,double now) {
    s.at=now; s.sequence=state.sequence+1; state=s;
    if(owned && state.control!=2 && active!="release_control") control_lost();
    if(active.empty()) return;
    // A command ACK is never completion. Require two NEW matching RobotStates.
    if(state.sequence>issued_sequence_ && complete(active)) ++matches_; else matches_=0;
  }
  void ack(const std::string& name,bool accepted=true) {
    if(name!=active) return;
    if(!accepted) { cancel("SDK 拒绝 "+name,true); return; }
    acked_=true;
  }
  std::string reason(const std::string& action,double now) const {
    if(!connected) return "SDK 未连接";
    if(replay) return "回放锁存禁用";
    if(action=="soft_estop" || action=="halt") return "";
    if(!lease_valid(now)) return "请解锁本次操作";
    if(!fresh(now)) return "机器人反馈过期";
    if(action=="release_control") return owned?"":"本接收器未确认持有控制权";
    if(fault) return "Error/Fatal 故障锁存，排障后重启";
    if(action=="reset_error") return uncertain&&stationary()&&(upright(state.motion)||state.motion==2||state.motion==3)?"":"仅可在稳定且静止反馈下确认异常";
    if(uncertain) return "上次执行状态需人工核对并复位";
    if(busy()) return "正在等待姿态/模式转换完成";
    if(state.hardware!=1) return "硬件急停未解除或状态未知";
    if(action=="recover_estop") return state.software==2?"":"软件急停未触发或状态未知";
    if(!safe()) return "软件急停未解除或状态未知";
    if(action=="take_control") return owned?"本接收器已确认持有控制权":"";
    if(action=="prepare_navigation") {
      return (state.motion==2||state.motion==3||upright(state.motion))?"":"锁定/站立中/特殊动作不能直接准备运动";
    }
    if(!owned||state.control!=2) return "请先申请本接收器的 SDK 控制权";
    if(action=="unlock_to_stand") return state.motion==4?"":"机器人不是锁定姿态";
    if(action=="stand") return state.motion==2||state.motion==3?"":"仅趴下/匍匐可站立，锁定需单独解锁站立";
    if(action=="lie_down") return upright(state.motion)?"":"先站稳，再趴下；不从匍匐隐式站起";
    if(action=="crawl") return state.motion==2||state.motion==5?"":"仅趴下/通用模式可匍匐";
    if(action=="general_mode"||action=="in_place_mode"||action=="stair_mode") return upright(state.motion)?"":"请先站稳，再切换模式";
    return "未开放的操作";
  }
  std::string request(const std::string& action,double now) {
    const auto denied=reason(action,now); if(!denied.empty()) return denied;
    if(action=="reset_error") { uncertain=false; error.clear(); result="IDLE"; prepared=false; return ""; }
    if(action=="halt") { cancel("已取消后续步骤；正在进行的姿态动作不保证中断",false); pending_stop_=owned; return ""; }
    if(action=="soft_estop"||action=="release_control") cancel("安全操作取消旧目标",false);
    prepared=false; velocity_at_=-1e9; ++goal_id; ++generation;
    goal=action; result="QUEUED"; error.clear();
    if(action=="prepare_navigation") {
      if(!owned) queue_.push_back("take_control");
      if(upright(state.motion)&&state.motion!=6) queue_.push_back("stop");
      if(state.motion==2||state.motion==3) queue_.push_back("stand");
      queue_.push_back("general_mode"); queue_.push_back("set_speed");
    } else {
      if((action=="lie_down"||action=="crawl"||action.find("_mode")!=std::string::npos||action=="release_control")&&!fault&&safe()&&owned&&(state.motion==5||state.motion==7)) queue_.push_back("stop");
      queue_.push_back(action=="unlock_to_stand"?"stand":action);
    }
    events.push_back("已受理 #"+std::to_string(goal_id)+" "+goal+"；等待状态确认");
    return "";
  }
  bool velocity(double x,double y,double yaw,double now) {
    if(!ready(now)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||std::abs(x)>config.max_x||std::abs(y)>config.max_y||std::abs(yaw)>config.max_yaw) return false;
    vx_=x; vy_=y; vyaw_=yaw; velocity_at_=now; return true;
  }
  std::vector<Command> tick(double now) {
    std::vector<Command> output;
    if(!connected) return output;
    if(!lease_valid(now) && goal!="soft_estop" && goal!="release_control") cancel("操作心跳失效；取消后续步骤",false);
    if(busy() && goal!="soft_estop" && goal!="release_control") {
      if(!fresh(now)||fault||(goal!="recover_estop"&&!safe())) cancel("反馈过期或安全条件改变",true);
    }
    if(moving && (!ready(now)||now-velocity_at_>config.velocity_timeout)) { pending_stop_=true; moving=false; velocity_at_=-1e9; }
    if(pending_stop_) {
      pending_stop_=false;
      // Do not interfere with other SDK clients or send Move in illegal modes.
      if(!replay&&owned&&state.control==2&&(state.motion==5||state.motion==7)) output.push_back({"velocity",0,0,0,generation});
    }
    if(!active.empty()) {
      if((acked_||active=="stop")&&matches_>=config.confirmations) {
        if(active=="take_control") owned=true;
        if(active=="release_control") owned=false;
        events.push_back("状态已确认："+active); active.clear();
        if(queue_.empty()) { result="SUCCEEDED"; prepared=goal=="prepare_navigation"; events.push_back("目标完成 #"+std::to_string(goal_id)+" "+goal); }
      } else if(now-issued_at_>config.transition_timeout) { cancel("转换超时，执行状态未知；禁止自动重试",true); }
    }
    if(active.empty()&&!queue_.empty()) {
      const std::string next=queue_.front();
      const bool safety=next=="soft_estop"||next=="release_control";
      if(!safety&&!legal(next,now)) { cancel("下一步前置状态不满足："+next,true); return output; }
      queue_.pop_front(); active=next; result="TRANSITIONING"; acked_=false; matches_=0;
      issued_at_=now; issued_sequence_=state.sequence;
      output.push_back({next,0,0,0,generation});
    }
    if(ready(now)&&now-velocity_at_<=config.velocity_timeout) {
      output.push_back({"velocity",vx_,vy_,vyaw_,generation}); moving=vx_!=0||vy_!=0||vyaw_!=0;
    }
    return output;
  }
  bool valid(const Command& c,double now) const {
    if(c.generation!=generation||!connected||replay) return false;
    if(c.name=="velocity") return c.x==0&&c.y==0&&c.yaw==0 ? owned&&state.control==2&&(state.motion==5||state.motion==7) : ready(now);
    return c.name==active && (c.name=="soft_estop"||c.name=="release_control"||legal(c.name,now));
  }
  void dispatch_failed(const std::string& why) { cancel("SDK 发送失败："+why,true); }
  void cancel(const std::string& why,bool ambiguous) {
    const bool running=busy();
    if(running||moving||prepared) { ++generation; pending_stop_=moving; moving=false; queue_.clear(); active.clear(); prepared=false; velocity_at_=-1e9; result=ambiguous?"FAILED":"CANCELLED"; error=why; events.push_back(why); }
    if(ambiguous) { uncertain=true; error=why; result="FAILED"; }
  }
 private:
  bool lease_=false, acked_=false, pending_stop_=false;
  std::string session_;
  double lease_at_=-1e9, issued_at_=0, velocity_at_=-1e9, vx_=0,vy_=0,vyaw_=0;
  unsigned long issued_sequence_=0;
  int matches_=0;
  std::deque<std::string> queue_;
  bool legal(const std::string& step,double now) const {
    // An already authorized release may finish stopping after the UI locks.
    if(step=="stop"&&goal=="release_control") return connected&&!replay&&!fault&&fresh(now)&&safe()&&owned&&state.control==2&&(state.motion==5||state.motion==7);
    if(!connected||replay||fault||!fresh(now)||!lease_valid(now)) return false;
    if(step=="recover_estop") return state.hardware==1&&state.software==2;
    if(!safe()) return false;
    if(step=="take_control") return true;
    if(!owned||state.control!=2) return false;
    if(step=="stand") return state.motion==2||state.motion==3||(goal=="unlock_to_stand"&&state.motion==4);
    if(step=="lie_down") return upright(state.motion)&&stationary();
    if(step=="crawl") return state.motion==2||(state.motion==5&&stationary());
    if(step=="stop") return state.motion==5||state.motion==7;
    if(step=="set_speed") return state.motion==5&&state.mode==1;
    return upright(state.motion)&&stationary();
  }
  bool complete(const std::string& step) const {
    if(step=="take_control") return state.control==2;
    if(step=="release_control") return state.control==1||state.control==3;
    if(step=="soft_estop") return state.software==2;
    if(step=="recover_estop") return state.software==1&&state.hardware==1;
    if(step=="stop") return stationary();
    if(step=="stand") return upright(state.motion)&&state.mode==state.motion-4;
    if(step=="lie_down") return state.motion==2;
    if(step=="crawl") return state.motion==3;
    if(step=="general_mode") return state.motion==5&&state.mode==1;
    if(step=="in_place_mode") return state.motion==6&&state.mode==2;
    if(step=="stair_mode") return state.motion==7&&state.mode==3;
    if(step=="set_speed") return state.speed==config.speed_level&&state.motion==5&&state.mode==1;
    return false;
  }
};
} // namespace d1console
