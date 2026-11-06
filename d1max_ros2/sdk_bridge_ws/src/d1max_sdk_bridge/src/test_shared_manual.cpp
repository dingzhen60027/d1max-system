#include "shared_manual_core.hpp"
#include <cassert>
#include <iostream>
using namespace d1monitor::shared_manual;
using Protocol=d1monitor::shared_estop::Protocol;
static Request request(const std::string&op,Json values=Json::object()) {Request r;r.op=op;r.arguments=std::move(values);return r;}
static State ready(){State s;s.present=s.connected=s.owned=true;s.source=2;s.mode=3;s.motion=7;s.soft=s.hardware=s.speed=1;s.received=10;s.sequence=3;return s;}
int main(){
  unsigned cases=0;
  Protocol protocol("sdk",true);auto hello=Json{{"type","hello"},{"version",1},{"client","qt"}};
  assert(protocol.receive(1,hello).kind==Protocol::Kind::Hello);
  assert(protocol.clientName(1)=="qt");auto state=ready();assert(protocol.hello(state)["manual"]["phase"]=="idle");
  assert(protocol.hello(state)["capabilities"].back()=="manual_control_v1");++cases;
  Core c;assert(!c.active());assert(c.navigationReady(10));
  auto r=c.receive(1,"qt",request("manual_take"),state,10,"session:old");assert(r.accepted&&r.pending&&c.phase==Core::Phase::Acquiring);
  assert(!c.navigationReady(10)&&!c.allowsTask("session","old"));assert(!c.zero_due);++cases;
  assert(!c.receive(2,"other",request("manual_take"),state,10,"").accepted);++cases;
  c.phase=Core::Phase::Manual;c.pending.reset();const std::string lease=c.lease;
  auto jog=request("jog",{{"lease",lease},{"generation","1"},{"key","w"},{"percent",30}});
  assert(c.receive(1,"qt",jog,state,10.1,"").accepted);assert(std::abs(c.velocity(10.15).forward-.30f)<1e-6f);
  assert(!c.navigationReady(10.15));++cases;
  auto beat=jog;beat.op="heartbeat";assert(c.receive(1,"qt",beat,state,10.2,"").accepted);
  assert(!c.receive(2,"other",beat,state,10.25,"").accepted);++cases;
  auto opposite=jog;opposite.arguments["generation"]="2";opposite.arguments["key"]="s";
  assert(c.receive(1,"qt",opposite,state,10.3,"").accepted);assert(std::abs(c.velocity(10.31).forward+.30f)<1e-6f);
  assert(c.receive(1,"qt",request("release_jog",{{"lease",lease},{"generation","1"}}),state,10.32,"").accepted);
  assert(c.held(10.33));++cases;
  c.expire(10.7);assert(!c.held(10.7)&&c.zero_due&&!c.zero_confirmed);
  opposite.op="heartbeat";assert(!c.receive(1,"qt",opposite,state,10.71,"").accepted);++cases;
  c.zero_due=false;c.zero_confirmed=true;assert(c.navigationReady(10.71));
  auto posture=request("posture",{{"lease",lease},{"mode",1},{"action",3}});
  assert(c.receive(1,"qt",posture,state,10.8,"").pending);assert(c.pending->action==Action::Mode);
  assert(!c.zero_due);assert(!c.navigationReady(10.8));++cases;
  c.pending.reset();posture.arguments["action"]=4;
  assert(c.receive(1,"qt",posture,state,10.9,"").pending);assert(c.pending->action==Action::Lie);++cases;
  c.pending.reset();auto lie=state;lie.motion=2;c.zero_confirmed=true;
  assert(c.receive(1,"qt",request("posture",{{"lease",lease},{"mode",3},{"action",3}}),lie,11,"").pending);
  assert(c.pending->action==Action::Stand&&!c.zero_due);++cases;
  c.pending.reset();c.detach(2);assert(c.owns());c.detach(1);assert(c.phase==Core::Phase::Releasing);++cases;
  c.releaseComplete();assert(c.navigationReady(11));assert(!c.receive(1,"qt",jog,state,11,"").accepted);
  assert(!c.allowsTask("session","old"));++cases;
  assert(c.receive(1,"qt",request("manual_take"),state,11,"").accepted);assert(c.lease!=lease);
  c.phase=Core::Phase::Manual;c.pending.reset();assert(!c.receive(1,"qt",jog,state,11,"").accepted);++cases;
  auto no_owner=state;no_owner.owned=false;assert(!c.deny(no_owner,11,true).empty());
  auto stale=state;stale.received=1;assert(!c.deny(stale,11,true).empty());++cases;
  auto on=state;on.soft=2;assert(!c.deny(on,11,true).empty());c.veto("estop");assert(!c.held(11));++cases;
  Core disconnected;auto disconnected_state=state;disconnected_state.connected=false;
  assert(!disconnected.receive(1,"qt",request("manual_take"),disconnected_state,11,"").accepted);++cases;
  for(const bool failed_zero:{false,true}){
    Core stopped;assert(stopped.receive(1,"qt",request("manual_take"),state,10,"session:old").accepted);
    stopped.phase=Core::Phase::Manual;stopped.pending.reset();
    const auto prior_lease=stopped.lease;
    assert(stopped.receive(1,"qt",request("jog",{{"lease",prior_lease},{"generation","1"},{"key","w"},{"percent",30}}),state,10.1,"").accepted);
    stopped.veto("software_estop_requested");stopped.releaseComplete();
    assert(!stopped.active()&&stopped.needsWriterService());
    assert(stopped.zero_due&&!stopped.zero_confirmed&&!stopped.navigationReady(10.2));
    assert(!stopped.held(10.2)&&!stopped.velocity(10.2).nonzero());
    assert(!stopped.receive(1,"qt",request("heartbeat",{{"lease",prior_lease},{"generation","1"},{"key","w"},{"percent",30}}),state,10.2,"").accepted);
    // Model ONLY the writer receipt, never a phase/UI/estop acknowledgement.
    stopped.zero_due=false;
    if(failed_zero)stopped.uncertain=true;else stopped.zero_confirmed=true;
    assert(stopped.navigationReady(10.3)==!failed_zero);
    assert(!stopped.allowsTask("session","old"));
    assert(stopped.receive(1,"qt",request("manual_take"),state,10.3,"").accepted==!failed_zero);
    ++cases;
  }
  std::cout<<"shared manual ownership: "<<cases<<" groups passed; no SDK linked\n";
}
