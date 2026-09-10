#pragma once
#include <string>

// Monitor-only safety state. No ownership, recovery, posture or velocity API.
namespace d1monitor {
struct Safety {
  bool connected=false, replay=false, pending=false, ack=false;
  int software=0, hardware=0, confirmations=0;
  double state_at=-1e9, requested_at=-1e9;
  unsigned long sent=0;
  std::string result="IDLE", error;
  bool fresh(double now) const { return now>=state_at && now-state_at<=2.5; }
  bool triggered(double now) const { return fresh(now)&&software==2; }
  bool available() const { return connected&&!replay; }
  // Return true ONLY when an explicit request needs one SDK transmission.
  bool request(double now) {
    if(!available()) { error=replay?"回放模式禁用急停请求":"SDK 未连接"; return false; }
    if(triggered(now)) { result="OBSERVED_STOPPED";error.clear();return false; }
    if(pending) return false;
    pending=true;ack=false;confirmations=0;requested_at=now;
    result="WAITING_STATE";error.clear();++sent;return true;
  }
  void update(int sw,int hw,double now) {
    software=sw;hardware=hw;state_at=now;
    if(pending) {
      confirmations=sw==2?confirmations+1:0;
      if(confirmations>=2) {pending=false;result="OBSERVED_STOPPED";error.clear();}
    }
    // Late telemetry is explicit evidence of current STOP, never permission to move.
    if(result=="UNCONFIRMED"&&sw==2) {result="OBSERVED_STOPPED";error.clear();}
  }
  void failed(const std::string& why) {pending=false;result="UNCONFIRMED";error=why;}
  void tick(double now) {
    if(pending&&(replay||!connected||now-requested_at>5))
      failed("急停请求尚未获得状态确认；请核对实机，必要时使用机身急停。不会自动重试。");
  }
};
}
