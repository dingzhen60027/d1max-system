#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace d1monitor {
// Operator-opted-in APP -> SDK handoff. Availability is an explicit SDK event,
// NOT an inference from missing MC, control_source, or a closed phone UI.
// The vendor ACK has no request ID: an ambiguous attempt poisons this connection
// so a late ACK cannot be attributed to a later request. Never retry TakeControl.
struct Ownership {
  bool enabled=false,connected=false,replay=false,pending=false,acknowledged=false;
  bool confirmed=false,uncertain=false,attempted=false,waiting_app=false,reconfigure=false;
  double settle_sec=.5,ack_timeout_sec=3.,confirmation_timeout_sec=4.;
  double available_at=-1.,requested_at=-1.,ack_at=-1.,last_robot=-1.;
  int control_source=0;unsigned confirmations=0;
  uint64_t request_id=0,total_requests=0;
  std::string result="waiting_available",error;
  void validate() const {
    if(!std::isfinite(settle_sec)||settle_sec<.2||settle_sec>2.||
       !std::isfinite(ack_timeout_sec)||ack_timeout_sec<1.||ack_timeout_sec>10.||
       !std::isfinite(confirmation_timeout_sec)||confirmation_timeout_sec<2.||confirmation_timeout_sec>10.)
      throw std::invalid_argument("invalid SDK ownership handoff timing");
  }
  void connection(bool online,bool is_replay) {
    if(online!=connected) {
      ++request_id;pending=acknowledged=confirmed=uncertain=attempted=waiting_app=reconfigure=false;
      available_at=requested_at=ack_at=last_robot=-1.;control_source=0;confirmations=0;
      result="waiting_available";error.clear();
    }
    connected=online;replay=is_replay;
    if(replay){if(pending&&!acknowledged)uncertain=true;pending=confirmed=false;available_at=-1.;++request_id;}
  }
  void lost() {
    if(pending&&!acknowledged)uncertain=true;
    ++request_id;pending=acknowledged=confirmed=attempted=reconfigure=false;
    available_at=-1.;confirmations=0;waiting_app=true;result="waiting_app";
  }
  void available(double now) {
    if(!connected||replay||pending||confirmed||uncertain||attempted)return;
    // Repeated notifications must not keep postponing (or restart) one request.
    if(available_at<0)available_at=now;
    waiting_app=false;result="available";error.clear();
  }
  void robot(int source,double now) {
    if(!connected||replay||now<=last_robot)return;
    control_source=source;last_robot=now;
    if(confirmed&&source!=2)lost();
    if(!pending&&!confirmed&&available_at<0&&source==1)waiting_app=true;
    if(pending&&acknowledged&&now>ack_at) {
      confirmations=source==2?confirmations+1:0;
      if(confirmations>=2){pending=false;confirmed=true;reconfigure=true;result="confirmed";error.clear();}
    }
  }
  bool fresh(double now) const {return last_robot>=0&&now>=last_robot&&now-last_robot<=2.5;}
  bool owns(double now) const {return connected&&!replay&&confirmed&&control_source==2&&fresh(now);}
  void fail(const std::string& why,const std::string& detail,bool ambiguous) {
    pending=confirmed=false;uncertain=uncertain||ambiguous;result=why;error=detail;available_at=-1.;
  }
  void tick(double now) {
    if(pending&&now-(acknowledged?ack_at:requested_at)>=(acknowledged?confirmation_timeout_sec:ack_timeout_sec))
      fail("takeover_timeout",acknowledged?"接管回执成功，但机器人归属未确认；请核对后重连":"接管回执超时，状态未知；不自动重试，请核对后重连",true);
  }
  bool due(double now) {
    tick(now);
    if(!enabled||!connected||replay||pending||confirmed||uncertain||attempted||available_at<0||!fresh(now))return false;
    if(now-available_at>5.){available_at=-1.;result="waiting_available";return false;}
    if(now-available_at<settle_sec)return false;
    pending=attempted=true;acknowledged=false;confirmations=0;requested_at=now;
    ++request_id;++total_requests;result="takeover_pending";return true;
  }
  void ack(uint32_t code,const std::string& reason,double now) {
    tick(now);
    if(!connected||replay||!pending||acknowledged||uncertain||now<requested_at)return;
    if(code){fail("takeover_rejected",reason.empty()?"机器人拒绝接管，等待核对控制权":reason,false);return;}
    acknowledged=true;ack_at=now;result="takeover_verifying";
  }
  void written(uint64_t id,const std::string& detail) {
    if(id==request_id&&pending&&!detail.empty())fail("takeover_write_failed",detail,true);
  }
  bool allow_mc_config() const {
    return !enabled||(!waiting_app&&!pending&&!uncertain&&result!="takeover_rejected"&&(available_at<0||confirmed));
  }
  std::string state(double now) const {
    if(!enabled)return "disabled";
    if(!connected)return "disconnected";
    if(replay)return "replay_blocked";
    if(uncertain)return "takeover_timeout";
    if(pending||result=="takeover_rejected")return result;
    if(waiting_app)return "waiting_app";
    if(confirmed)return owns(now)?"confirmed":"waiting_robot_state";
    return result;
  }
};
}
