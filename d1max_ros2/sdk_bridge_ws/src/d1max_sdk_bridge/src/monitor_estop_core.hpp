#pragma once
#include <cmath>
#include <cstdint>
#include <string>

// One explicit software-estop transaction. SDK write/command ACKs are not
// completion; only ordered original RobotState receipts establish its target.
// This core owns no SDK client, ownership, posture or velocity API.
namespace d1monitor {
struct Safety {
  bool connected=false, replay=false, pending=false, ack=false;
  bool target_on=true, request_accepted=false;
  int software=0, hardware=0, confirmations=0;
  double state_at=-1e9, requested_at=-1e9;
  std::uint64_t transaction=0, confirmed_transaction=0;
  unsigned long sent=0;
  std::string result="IDLE", error, request_reason;
  bool fresh(double now) const { return std::isfinite(now)&&std::isfinite(state_at)&&now>=state_at&&now-state_at<=2.5; }
  bool triggered(double now) const { return fresh(now)&&software==2; }
  bool available() const { return connected&&!replay; }
  // Confirmation belongs to the CURRENT target, not to an earlier ON/OFF.
  // Starting OFF after a confirmed ON must not expose that old confirmation
  // as evidence of release before the new original RobotStates arrive.
  bool releaseConfirmed() const {
    return transaction&&confirmed_transaction==transaction&&!target_on&&!pending&&
      result=="OBSERVED_RECOVERED"&&software==1&&hardware==1;
  }
  // Return true ONLY when an explicit request needs one SDK transmission.
  // Same-target requests coalesce. An explicit ON can supersede a pending
  // OFF for safety; OFF never overtakes a pending ON. There are no retries.
  bool request(double now,bool on=true) {
    request_accepted=false;request_reason.clear();
    if(!available()||!std::isfinite(now)) {
      request_reason=replay?"software_estop_replay_disabled":
        !connected?"software_estop_sdk_disconnected":"software_estop_clock_invalid";
      error=request_reason;return false;
    }
    if(!on&&(!fresh(now)||hardware!=1||(software!=1&&software!=2))) {
      request_reason=!fresh(now)?"software_estop_state_stale":hardware!=1?
        "software_estop_hardware_not_recovered":"software_estop_state_unknown";
      error=request_reason;return false;
    }
    const bool superseding_release=on&&!target_on&&transaction>0;
    if(pending) {
      if(on==target_on) {request_accepted=true;request_reason="software_estop_request_pending";return false;}
      if(!on) {request_reason="software_estop_opposite_request_pending";return false;}
      // ON supersedes the release transaction; its late callback/state can
      // never complete this new target or restore a previous motion grant.
    }
    target_on=on;request_accepted=true;ack=false;confirmations=0;requested_at=now;
    ++transaction;confirmed_transaction=0;
    // An earlier OFF can change a currently reported STOP later (including
    // after timeout). A new ON target therefore always supersedes that OFF
    // on the SDK wire, even if the last state already reports STOP.
    if(!superseding_release&&fresh(now)&&software==(on?2:1)) {
      pending=false;result=on?"OBSERVED_STOPPED":"OBSERVED_RECOVERED";
      confirmed_transaction=transaction;error.clear();request_reason="software_estop_already_observed";return false;
    }
    pending=true;result="WAITING_STATE";error.clear();request_reason="software_estop_request_submitted";
    ++sent;return true;
  }
  bool update(int sw,int hw,double now) {
    // Republished heartbeats, equal receipts and reordered packets cannot
    // confirm a request, turn a toggle backwards or refresh the source lease.
    if(!std::isfinite(now)||now<=state_at)return false;
    software=sw;hardware=hw;state_at=now;
    if(transaction&&(pending||result=="UNCONFIRMED")&&now>requested_at) {
      if(!target_on&&hw!=1) {failed("software_estop_hardware_not_recovered");return true;}
      confirmations=sw==(target_on?2:1)?confirmations+1:0;
      if(confirmations>=2) {
        pending=false;result=target_on?"OBSERVED_STOPPED":"OBSERVED_RECOVERED";
        confirmed_transaction=transaction;error.clear();
      }
    }
    return true;
  }
  // Vendor OnSoftEmergencyStop has only a bool, not an operation ID. This is
  // uncorrelated ACK telemetry, never transaction completion/authorization.
  void acknowledge(bool on,double receipt) {
    if(pending&&on==target_on&&std::isfinite(receipt)&&receipt>=requested_at)ack=true;
  }
  void failed(const std::string& why,std::uint64_t id=0) {
    if(id&&(id!=transaction||confirmed_transaction==id))return;
    pending=false;result="UNCONFIRMED";error=why;
  }
  void tick(double now) {
    if(pending&&(replay||!connected||!std::isfinite(now)||now<requested_at||now-requested_at>5))
      failed("software_estop_state_unconfirmed_no_automatic_retry");
  }
};
}
