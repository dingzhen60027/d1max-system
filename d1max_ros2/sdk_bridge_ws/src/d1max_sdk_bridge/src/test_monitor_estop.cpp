#include <cassert>
#include <iostream>
#include "monitor_estop_core.hpp"
int main(){
  using d1monitor::Safety;
  Safety s;assert(!s.request(0));assert(s.sent==0);
  s.connected=true;s.replay=true;assert(!s.request(0));assert(s.sent==0);
  s.replay=false;s.update(1,1,0);assert(s.request(.1));assert(s.sent==1);
  assert(!s.request(.2));assert(s.sent==1); // Pending request never re-sends.
  s.update(2,1,.3);assert(s.pending);
  s.update(2,1,.5);assert(!s.pending);assert(!s.ack);
  assert(s.result=="OBSERVED_STOPPED"); // Feedback is evidence of STOP, not an ACK.
  assert(!s.request(.6));assert(s.sent==1);
  s.update(1,1,1);assert(s.request(1.1));
  s.ack=true;s.tick(7);assert(!s.pending);assert(s.result=="UNCONFIRMED");
  assert(s.sent==2);s.tick(20);assert(s.sent==2); // No automatic retry.
  s.update(2,1,21);assert(s.result=="OBSERVED_STOPPED");
  assert(s.triggered(22));assert(!s.triggered(25));
  s.update(1,1,26);assert(s.request(26.1));s.connected=false;s.tick(26.2);
  assert(!s.pending);assert(s.result=="UNCONFIRMED");
  std::cout<<"PASS monitor-only safety: connection/replay gating, deduplication, telemetry without ACK, timeout, stale state, no retries\n";
}
