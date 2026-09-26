#include <cassert>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include "navigation_dispatch.hpp"
#include "navigation_motion_core.hpp"

using namespace d1monitor;
namespace {
const std::string source="1234567890abcdef1234567890abcdef";
NavigationHealth health(){return {true,false,true,true,1,1,1,5,1,1,10.};}
struct Harness {
  std::mutex mutex;
  NavigationMotion nav;
  NavigationEvents events;
  NavigationHealth h=health();
  std::vector<NavigationVelocity> submitted;
  Harness(){nav.enabled=true;assert(nav.arm(h,10.,100.));
    assert(nav.accept(nav.generation,1,"nav","map",{.2,0,.1},100.,100.,10.,source));}
  void drain(){
    const auto flags=events.take();
    if(flags&NavigationEvents::LostControl){h.owned=false;nav.disarm("lost");}
    if(flags&NavigationEvents::RobotFault)nav.fail("fault");
    if(flags&NavigationEvents::MoveWriteFailure)nav.fail("write");
  }
  template<class Sender>void tick(Sender&& sender){
    dispatch_navigation(mutex,[this]{drain();return nav.tick(h,10.1);},
      [this,&sender](const auto& command){
        drain(); // same final callback veto as the monitor's send function
        if(command.x!=0&&(!nav.armed||!nav.reason(h,10.1).empty()))return;
        if(command.x==0&&!nav.can_stop(h,10.1))return;
        submitted.push_back(command);sender();drain();
      });
  }
};
}
int main(){
  // Every event before dispatch suppresses nonzero output, even if a healthy
  // command was accepted earlier. The callback itself never takes state mutex.
  for(const auto kind:{NavigationEvents::LostControl,NavigationEvents::RobotFault,
                      NavigationEvents::MoveWriteFailure}){
    Harness a;std::thread callback([&]{a.events.raise(kind);});callback.join();
    a.tick([]{});for(const auto& v:a.submitted)assert(v.x==0&&v.y==0&&v.yaw==0);
    assert(!a.nav.armed);
    assert(!a.nav.accept(a.nav.generation,2,"nav","map",{.2,0,.1},100.1,100.1,10.1,source));
  }
  // All independent callback vetoes remain sticky; new events cannot overwrite
  // a lost-control event (no finite queue overflow can clear the veto).
  NavigationEvents bits;std::thread first([&]{for(int i=0;i<1000;++i)bits.raise(NavigationEvents::LostControl);});
  std::thread second([&]{for(int i=0;i<1000;++i)bits.raise(NavigationEvents::RobotFault);});
  first.join();second.join();assert(bits.take()==(NavigationEvents::LostControl|NavigationEvents::RobotFault));assert(bits.take()==0);
  // Worker fault / MC overspeed commits BEFORE selection => no stale send.
  for(bool overspeed:{false,true}){
    Harness a;
    {std::lock_guard<std::mutex> lock(a.mutex);
      if(overspeed)a.nav.measurement(2.,0.,10.05);else a.nav.fail("worker_fault");}
    a.tick([]{});for(const auto& v:a.submitted)assert(v.x==0);assert(!a.nav.armed);
  }
  // Deterministic reproduction of the old tick/unlock/Move race. Hold the
  // selected submission while a worker tries to revoke; worker cannot commit
  // in the middle, so ordering is submission -> revocation -> no next nonzero.
  {
    Harness a;std::promise<void> inside_send,worker_trying,release_send;
    auto release=release_send.get_future().share();
    std::thread sender([&]{a.tick([&]{inside_send.set_value();release.wait();});});
    inside_send.get_future().wait();
    std::thread worker([&]{worker_trying.set_value();std::lock_guard<std::mutex> lock(a.mutex);a.nav.fail("worker_fault");});
    worker_trying.get_future().wait();
    assert(!a.mutex.try_lock()); // sender still owns the actual state mutex
    release_send.set_value();sender.join();worker.join();
    assert(a.submitted.size()==1&&a.submitted.front().x==.2&&!a.nav.armed);
    a.tick([]{});assert(a.submitted.size()==2&&a.submitted.back().x==0);
  }
  // A vendor that synchronously invokes its error callback cannot deadlock:
  // callback has atomic/mailbox work only, consumed before dispatch returns.
  {
    Harness a;
    a.tick([&]{a.events.raise(NavigationEvents::MoveWriteFailure);});
    assert(a.nav.fault&&!a.nav.armed&&a.submitted.size()==1);
    a.tick([]{});assert(a.submitted.back().x==0);
  }
  // Also model a vendor waiting for a callback running on another thread.
  {
    Harness a;a.tick([&]{std::thread callback([&]{a.events.raise(NavigationEvents::LostControl);});callback.join();});
    assert(!a.nav.armed);a.tick([]{});assert(a.submitted.size()==1);
  }
  // Callback event after selection but before the final submission veto must
  // suppress the selected command, not defer it to a later watchdog timer.
  {
    Harness a;dispatch_navigation(a.mutex,[&]{const auto result=a.nav.tick(a.h,10.1);
      a.events.raise(NavigationEvents::LostControl);return result;},[&](const auto& command){
      a.drain();if(a.nav.armed)a.submitted.push_back(command);
    });assert(a.submitted.empty()&&!a.nav.armed);
  }
  std::cout<<"PASS serialized SDK dispatch: callback/worker revocations, MC overspeed, ordered submission, synchronous callback, sticky veto, no auto rearm\n";
}
