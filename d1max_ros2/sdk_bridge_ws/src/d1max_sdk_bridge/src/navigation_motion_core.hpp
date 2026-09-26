#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

// Optional, single-SDK-session navigation transport. No posture, mode, control
// acquisition, emergency-stop release or reconnect commands belong here.
namespace d1monitor {
struct NavigationHealth {
  bool connected=false,replay=false,owned=false,mc_fresh=false;
  int software=0,hardware=0,mode=0,motion=0,speed=0,head=0;
  double robot_at=-1.;
};
struct NavigationVelocity {
  double x=0,y=0,yaw=0;
};
struct NavigationMotion {
  // Not a parameter: the user's absolute planar command ceiling must survive
  // upstream profile changes and callers bypassing the ROS command gate.
  static constexpr double hard_planar_mps=1.5;
  static constexpr double low_forward_mps=1.0,low_lateral_mps=.5,low_yaw_radps=1.5;
  bool enabled=false,armed=false,fault=false,moving=false;
  bool overspeed_latched=false;
  double max_x=.30,max_y=.0,max_yaw=.50,command_timeout=.25;
  double measured_planar_mps=-1.,measurement_at=-1.;
  double command_at=-1.,armed_wall=-1.,armed_at=-1.;
  uint64_t generation=0,sequence=0,sent=0;
  unsigned stop_remaining=0;
  std::string navigation_session,map_version,command_source,error="disabled";
  NavigationVelocity command;

  void validate() const {
    if(!std::isfinite(max_x)||max_x<=0||max_x>low_forward_mps||!std::isfinite(max_y)||max_y<0||max_y>low_lateral_mps||
       !std::isfinite(max_yaw)||max_yaw<=0||max_yaw>1.||!std::isfinite(command_timeout)||command_timeout<=0||command_timeout>.25)
      throw std::invalid_argument("unsafe navigation SDK velocity limits");
  }
  bool robot_fresh(const NavigationHealth& h,double now) const {
    return std::isfinite(now)&&std::isfinite(h.robot_at)&&h.robot_at>=0&&now>=h.robot_at&&now-h.robot_at<=2.;
  }
  std::string reason(const NavigationHealth& h,double now) const {
    if(!enabled)return "disabled";
    if(fault)return "fault_latched";
    if(!h.connected)return "disconnected";
    if(h.replay)return "replay_blocked";
    if(!robot_fresh(h,now))return "robot_state_stale";
    if(!h.owned)return "sdk_control_not_confirmed";
    if(h.software!=1||h.hardware!=1)return "estop_active_or_unknown";
    if(h.mode!=1||h.motion!=5||h.speed!=1||h.head!=1)return "general_low_speed_head_forward_required";
    if(!h.mc_fresh)return "mc_missing_or_stale";
    if(measured_planar_mps>hard_planar_mps)return "measured_planar_overspeed";
    return {};
  }
  bool can_stop(const NavigationHealth& h,double now) const {
    return enabled&&h.connected&&!h.replay&&h.owned&&robot_fresh(h,now)&&
      h.software==1&&h.hardware==1&&h.mode==1&&h.motion==5;
  }
  void disarm(const std::string& why) {
    if(moving)stop_remaining=3;
    moving=armed=false;command_at=-1.;command={};error=why;
    navigation_session.clear();map_version.clear();command_source.clear();
  }
  bool arm(const NavigationHealth& h,double now,double wall) {
    if(armed){error="already_armed_disarm_before_rearming";return false;}
    disarm("rearming");
    const auto why=reason(h,now);
    if(!why.empty()||!std::isfinite(wall)){error=why.empty()?"invalid_time":why;return false;}
    ++generation;sequence=0;armed_wall=wall;armed_at=now;armed=true;error="waiting_command";return true;
  }
  void fail(const std::string& why) { fault=true;disarm(why); }
  // Call only after MC source time, receipt freshness and all fields have passed
  // McReport::sample. Never use OnSpeedData, predicted or timer-republished data.
  void measurement(double x,double y,double at) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(at)||at<measurement_at)return;
    measured_planar_mps=std::hypot(x,y);measurement_at=at;
    if(enabled&&armed&&measured_planar_mps>hard_planar_mps){
      stop_remaining=3;
      overspeed_latched=true;fail("measured_planar_overspeed_latched");
    }
  }
  static bool sdk_velocity_valid(const NavigationVelocity& si) {
    return std::isfinite(si.x)&&std::isfinite(si.y)&&std::isfinite(si.yaw)&&
      std::hypot(si.x,si.y)<=hard_planar_mps&&std::abs(si.x)<=low_forward_mps&&
      std::abs(si.y)<=low_lateral_mps&&std::abs(si.yaw)<=low_yaw_radps;
  }
  // Check the OLD lease before a new packet can overwrite command_at. This is
  // also used by tick(): a delayed timer must not let a late packet revive an
  // expired arm generation. Initial grace applies only before the first packet.
  bool expire_command_lease(double now) {
    if(!armed)return true;
    if(!std::isfinite(now)||now<armed_at){disarm("invalid_command_clock_rearm_required");return true;}
    if(command_at<0){
      if(now-armed_at>.6){disarm("initial_command_timeout");return true;}
    }else if(now<command_at||now-command_at>command_timeout){
      disarm("command_timeout_rearm_required");return true;
    }
    return false;
  }
  bool accept(uint64_t incoming_generation,uint64_t seq,const std::string& nav_session,
              const std::string& map,const NavigationVelocity& velocity,double stamp,double wall,double now,
              const std::string& source="") {
    if(expire_command_lease(now)||incoming_generation!=generation||seq<=sequence)return false;
    if(!std::isfinite(stamp)||!std::isfinite(wall)||!std::isfinite(now)||stamp<armed_wall||
       wall<stamp||wall-stamp>command_timeout)return false;
    if(nav_session.empty()||nav_session.size()>128||map.empty()||map.size()>128||source.size()!=32)return false;
    for(const char c:source)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;
    if(!command_source.empty()&&command_source!=source){disarm("command_source_changed");return false;}
    if((!navigation_session.empty()&&navigation_session!=nav_session)||(!map_version.empty()&&map_version!=map)){
      disarm("navigation_context_changed");return false;
    }
    if(!sdk_velocity_valid(velocity)||
       std::abs(velocity.x)>max_x||std::abs(velocity.y)>max_y||std::abs(velocity.yaw)>max_yaw){
      disarm("invalid_or_excessive_velocity");return false;
    }
    navigation_session=nav_session;map_version=map;command_source=source;sequence=seq;command=velocity;command_at=now;
    error.clear();return true;
  }
  std::optional<NavigationVelocity> tick(const NavigationHealth& h,double now) {
    if(armed&&(h.software==2||h.hardware==2))fail("emergency_stop_observed_latched");
    const auto why=reason(h,now);
    if(!why.empty())disarm(fault?error:why);
    if(!expire_command_lease(now)){
      if(command_at<0)return std::nullopt;
      moving=std::abs(command.x)+std::abs(command.y)+std::abs(command.yaw)>0;++sent;return command;
    }
    if(stop_remaining){
      if(!can_stop(h,now)){stop_remaining=0;return std::nullopt;}
      --stop_remaining;++sent;return NavigationVelocity{};
    }
    return std::nullopt;
  }
  // Vendor SDK docs: Move receives FRACTIONS, not SI velocity. Low speed in
  // general mode: +/-1 forward=1m/s, lateral=.5m/s, yaw=1.5rad/s.
  static NavigationVelocity sdk_percentages(const NavigationVelocity& si) {
    // Final boundary, independent of configurable acceptance limits. Reject;
    // silently scaling a corrupted command would conceal a control-path fault.
    if(!sdk_velocity_valid(si))throw std::invalid_argument("unsafe SDK boundary velocity");
    return {si.x/low_forward_mps,si.y/low_lateral_mps,si.yaw/low_yaw_radps};
  }
};
}
