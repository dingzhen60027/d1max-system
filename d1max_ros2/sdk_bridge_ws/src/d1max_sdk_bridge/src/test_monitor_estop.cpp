#include <cassert>
#include <iostream>
#include <limits>
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
  s.acknowledge(true,1.2);assert(s.ack);s.tick(7);assert(!s.pending);assert(s.result=="UNCONFIRMED");
  assert(s.sent==2);s.tick(20);assert(s.sent==2); // No automatic retry.
  s.update(2,1,21);assert(s.result=="UNCONFIRMED");
  s.update(2,1,21.1);assert(s.result=="OBSERVED_STOPPED");
  assert(s.triggered(22));assert(!s.triggered(25));
  s.update(1,1,26);assert(s.request(26.1));s.connected=false;s.tick(26.2);
  assert(!s.pending);assert(s.result=="UNCONFIRMED");
  {
    // The deployed monitor calls this exact predicate before notifying its
    // Transport. A previous successful ON is not a successful new OFF: neither
    // the request, the SDK bool ACK nor the first 1 Hz state may release it.
    Safety r;r.connected=true;r.update(1,1,100.);
    assert(r.request(100.1,true));r.update(2,1,101.);r.update(2,1,102.);
    const auto stopped=r.confirmed_transaction;assert(stopped==r.transaction&&!r.releaseConfirmed());
    assert(r.request(102.1,false));assert(r.transaction>stopped&&r.confirmed_transaction==0);
    assert(!r.releaseConfirmed());r.acknowledge(false,102.2);assert(!r.releaseConfirmed());
    r.update(1,1,103.);assert(r.pending&&!r.releaseConfirmed());
    r.update(1,1,104.);assert(r.releaseConfirmed());
    r.update(1,2,105.);assert(!r.releaseConfirmed()); // A new hardware STOP is never release proof.
    r.update(2,1,106.);assert(!r.releaseConfirmed()); // A later software STOP invalidates the current fact.
  }
  {
    // Sparse/missing real feedback reaches a finite UNCONFIRMED result; a
    // diagnostic heartbeat is not fed into update() and cannot finish it.
    Safety r;r.connected=true;r.update(2,1,10.);assert(r.request(10.1,false));
    r.update(1,1,11.);r.tick(15.2);assert(!r.pending&&r.result=="UNCONFIRMED"&&!r.releaseConfirmed());
    assert(!r.update(1,1,11.));assert(!r.releaseConfirmed());
    // Late second ORIGINAL state can establish the flag, but does not retry
    // a command or restore motion ownership (which this core never owns).
    r.update(1,1,16.);assert(r.releaseConfirmed()&&r.sent==1);
  }
  {
    Safety r;r.connected=true;r.update(2,1,100.);
    assert(r.request(100.1,false)&&r.request_accepted&&!r.target_on);
    const auto release=r.transaction;assert(release&&r.pending&&!r.ack);
    assert(!r.request(100.2,false)&&r.request_accepted&&r.transaction==release&&r.sent==1);
    r.acknowledge(true,100.3);assert(!r.ack); // Opposite ACK cannot confirm OFF.
    r.acknowledge(false,99.);assert(!r.ack);
    r.acknowledge(false,100.3);assert(r.ack&&r.pending);
    assert(!r.update(1,1,99.));assert(r.software==2&&r.state_at==100.);
    r.update(1,1,100.4);assert(r.pending&&r.confirmations==1);
    assert(!r.update(1,1,100.4));assert(r.pending&&r.confirmations==1);
    r.update(1,1,100.5);assert(!r.pending&&r.result=="OBSERVED_RECOVERED"&&r.confirmed_transaction==release);
    r.failed("late_current_write_error",release);assert(r.result=="OBSERVED_RECOVERED");
    assert(!r.request(100.6,false)&&r.request_accepted&&r.sent==1); // Already released: no SDK call.
    assert(!r.request(104.,false)&&!r.request_accepted&&r.request_reason=="software_estop_state_stale");
  }
  {
    Safety r;r.connected=true;r.update(2,1,10.);assert(r.request(10.1,false));
    const auto old=r.transaction;
    // Even while the real software flag still says STOP, pending OFF needs a
    // new ON write. Otherwise its delayed SDK(false) could win after this click.
    assert(r.request(10.2,true)&&r.target_on&&r.transaction>old&&r.sent==2);
    const auto newer=r.transaction;
    assert(!r.request(10.3,false)&&!r.request_accepted&&r.transaction==newer);
    r.failed("old_off_write_failed",old);assert(r.pending&&r.error.empty());
    r.acknowledge(false,10.4);assert(!r.ack);
    r.update(1,1,10.4);assert(r.pending&&r.confirmations==0); // Late OFF state is not ON completion.
    r.update(2,1,10.5);r.update(2,1,10.6);assert(!r.pending&&r.confirmed_transaction==newer);
    assert(!r.request(10.7,true)&&r.sent==2);
    assert(r.request(10.8,false));r.tick(16.);assert(r.result=="UNCONFIRMED");
    r.update(2,1,16.1);assert(r.request(16.2,true)&&r.sent==4); // Timed-out OFF is superseded too.
  }
  {
    Safety r;r.connected=true;r.update(2,2,1.);
    assert(!r.request(1.1,false)&&!r.request_accepted&&r.sent==0);
    assert(!r.request(1.2,true)&&r.request_accepted); // ON is safe/idempotent under hard STOP.
    r.update(2,0,1.3);assert(!r.request(1.4,false)&&!r.request_accepted);
    r.update(0,1,1.5);assert(!r.request(1.6,false)&&!r.request_accepted);
    r.update(2,1,1.7);assert(r.request(1.8,false));
    r.update(1,2,1.9);assert(!r.pending&&r.result=="UNCONFIRMED");
    assert(r.confirmed_transaction!=r.transaction); // Hardware STOP cannot be software-completed.
    assert(!r.update(1,1,std::numeric_limits<double>::quiet_NaN()));
    r.replay=true;assert(!r.request(2.,false));
  }
  std::cout<<"PASS monitor software-estop targets: ordered two-state ON/OFF confirmation, hard-stop rejection, supersession, correlated failures, no retries or motion permission\n";
}
