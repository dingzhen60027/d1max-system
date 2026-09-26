#include <cassert>
#include <iostream>
#include <limits>
#include "navigation_motion_core.hpp"
using namespace d1monitor;
static const std::string source="1234567890abcdef1234567890abcdef";
static NavigationHealth health(){return {true,false,true,true,1,1,1,5,1,1,10.};}
static NavigationMotion ready(){NavigationMotion n;n.enabled=true;n.validate();assert(n.arm(health(),10.,100.));return n;}
static bool command(NavigationMotion& n,uint64_t seq=1,double now=10.,double wall=100.){
  return n.accept(n.generation,seq,"navigation-a","grid-a",{.2,0.,.3},wall,wall,now,source);
}
int main(){
  NavigationMotion disabled;assert(!disabled.arm(health(),10.,100.));assert(!disabled.tick(health(),10.));
  auto n=ready();assert(!n.tick(health(),10.));assert(command(n));
  auto output=n.tick(health(),10.);assert(output&&output->x==.2);
  auto fractions=NavigationMotion::sdk_percentages({.2,.1,.3});
  assert(std::abs(fractions.x-.2)<1e-9&&std::abs(fractions.y-.2)<1e-9&&std::abs(fractions.yaw-.2)<1e-9);
  assert(!command(n)); // duplicate sequence
  assert(!n.accept(n.generation,2,"navigation-a","grid-a",{.2,0,0},99.,100.,10.,source));
  assert(!n.accept(n.generation,2,"navigation-a","grid-a",{.2,0,0},101.,100.,10.,source));
  output=n.tick(health(),10.3);assert(output&&output->x==0&&!n.armed);
  assert(n.tick(health(),10.35));assert(n.tick(health(),10.4));assert(!n.tick(health(),10.45));
  assert(!command(n,3));assert(n.arm(health(),10.5,100.5));
  assert(!n.accept(1,3,"navigation-a","grid-a",{.2,0,0},100.6,100.6,10.6,source)); // old arm generation
  assert(!n.accept(n.generation,3,"navigation-a","grid-a",{.2,0,0},100.4,100.6,10.6,source));
  assert(command(n,3,10.6,100.6));assert(n.tick(health(),10.6));
  assert(!n.accept(n.generation,4,"navigation-b","grid-a",{},100.6,100.6,10.6,source));assert(!n.armed);
  for(int i=0;i<12;++i){
    auto current=ready();assert(command(current));assert(current.tick(health(),10.));auto h=health();
    switch(i){case 0:h.connected=false;break;case 1:h.replay=true;break;case 2:h.owned=false;break;
      case 3:h.mc_fresh=false;break;case 4:h.software=0;break;case 5:h.hardware=2;break;
      case 6:h.mode=3;break;case 7:h.motion=7;break;case 8:h.speed=2;break;case 9:h.head=2;break;
      case 10:h.robot_at=0;break;case 11:current.fail("SDK write failed");break;}
    auto stop=current.tick(h,10.1);assert(!current.armed);assert(!stop||stop->x==0);
    assert(!command(current,2,10.2,100.2));
  }
  for(auto v:{NavigationVelocity{std::numeric_limits<double>::quiet_NaN(),0,0},NavigationVelocity{.31,0,0},NavigationVelocity{0,.01,0},NavigationVelocity{0,0,.51}}){
    auto current=ready();assert(!current.accept(current.generation,1,"nav","map",v,100.,100.,10.,source));assert(!current.armed);
  }
  n=ready();n.fail("fault");assert(!n.arm(health(),10.,100.));
  n=ready();assert(!n.arm(health(),10.,100.));assert(n.armed);assert(command(n));
  assert(!n.accept(n.generation,2,"navigation-a","grid-a",{},100.,100.,10.,"abcdef1234567890abcdef1234567890"));assert(!n.armed);
  n=ready();assert(!n.tick(health(),10.7));assert(!n.armed);
  // Receive path beats delayed watchdog timer: expired command must NOT renew.
  n=ready();assert(command(n));assert(n.tick(health(),10.));
  assert(!command(n,2,10.3,100.3));assert(!n.armed);
  assert(n.error=="command_timeout_rearm_required");
  assert(!command(n,3,10.31,100.31)); // fresh packets cannot implicitly rearm
  output=n.tick(health(),10.31);assert(output&&output->x==0);
  assert(n.arm(health(),10.4,100.4));assert(command(n,1,10.4,100.4));
  // Initial command grace has the same receive-side protection.
  n=ready();assert(!command(n,1,10.61,100.61));assert(!n.armed);
  assert(n.error=="initial_command_timeout");
  n=ready();assert(command(n,1,10.59,100.59));assert(n.armed);
  assert(!command(n,2,10.85,100.85));assert(!n.armed); // grace is not rolling
  n=ready();assert(command(n));
  assert(!command(n,2,9.99,100.01));assert(!n.armed); // monotonic clock regression
  n=ready();assert(!command(n,1,std::numeric_limits<double>::quiet_NaN(),100.));assert(!n.armed);
  n=ready();assert(command(n,900));const auto prior_generation=n.generation;
  n.disarm("operator_disarmed");assert(n.arm(health(),10.1,100.1));assert(n.sequence==0);
  assert(!n.accept(prior_generation,9999,"navigation-a","grid-a",{.2,0.,.3},100.1,100.1,10.1,source));
  const std::string restarted_source="abcdef1234567890abcdef1234567890";
  assert(n.accept(n.generation,1,"navigation-a","grid-a",{.2,0.,.3},100.1,100.1,10.1,restarted_source));
  assert(n.sequence==1&&n.command_source==restarted_source);
  // The final SDK call cannot bypass SI limits even if a future caller or a
  // corrupted parameter bypasses command acceptance / max_x validation.
  for(const auto v:{NavigationVelocity{1.51,0,0},NavigationVelocity{1.2,1.2,0},
      NavigationVelocity{1.01,0,0},NavigationVelocity{0,.51,0},NavigationVelocity{0,0,1.51},
      NavigationVelocity{std::numeric_limits<double>::infinity(),0,0}}){
    bool rejected=false;
    try{NavigationMotion::sdk_percentages(v);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  n=ready();n.max_x=n.max_y=n.max_yaw=100.;
  assert(!n.accept(n.generation,1,"nav","map",{1.2,1.2,0},100.,100.,10.,source));assert(!n.armed);
  n=ready();n.max_x=1.;n.max_y=.5;n.validate();
  assert(n.accept(n.generation,1,"nav","map",{1.,.5,1.},100.,100.,10.,source)==false); // configured yaw limit stays .5
  n=ready();n.max_x=1.;n.max_y=.5;
  assert(n.accept(n.generation,1,"nav","map",{1.,.5,.5},100.,100.,10.,source));
  fractions=NavigationMotion::sdk_percentages(*n.tick(health(),10.));
  assert(fractions.x==1.&&fractions.y==1.&&std::abs(fractions.yaw-1./3.)<1e-9);
  // Fresh MC body speed is a second, independent guard. A 1.5 m/s request cap
  // cannot promise a hardware speed cap; an observed exceedance latches STOP.
  n=ready();assert(command(n));assert(n.tick(health(),10.));
  n.measurement(1.5,0.,10.01);assert(!n.fault&&n.armed);
  n.measurement(1.1,1.1,10.02);assert(n.fault&&n.overspeed_latched&&!n.armed);
  const auto cause=n.error;
  for(int i=0;i<3;++i){output=n.tick(health(),10.03+i*.05);assert(output&&output->x==0&&output->y==0&&output->yaw==0);}
  assert(!n.tick(health(),10.20));assert(n.error==cause);
  n.measurement(0.,0.,10.21);assert(!n.arm(health(),10.22,100.22)); // no automatic or explicit fault bypass
  n=ready();n.measurement(.1,0.,10.1);
  n.measurement(9.,0.,10.0);assert(!n.fault&&n.measured_planar_mps==.1); // stale sample ignored
  n.measurement(std::numeric_limits<double>::quiet_NaN(),0.,10.2);assert(!n.fault);
  n.measurement(0.,2.,10.3);assert(n.overspeed_latched);
  output=n.tick(health(),10.3);assert(output&&output->x==0); // stopped command still triggers finite zero on actual overspeed
  n=ready();auto emergency=health();emergency.hardware=2;n.tick(emergency,10.);
  assert(n.fault&&!n.arm(health(),10.1,100.1)); // external release does not resume navigation
  NavigationMotion unarmed;unarmed.enabled=true;unarmed.measurement(2.,0.,10.);
  assert(!unarmed.arm(health(),10.,100.));assert(!unarmed.fault);
  unarmed.measurement(0.,0.,10.1);assert(unarmed.arm(health(),10.1,100.1));
  auto invalid=ready();invalid.max_x=1.01;bool config_rejected=false;
  try{invalid.validate();}catch(const std::invalid_argument&){config_rejected=true;}assert(config_rejected);
  std::cout<<"PASS optional SDK navigation: disabled default, explicit arm, SI conversion, generation/context/sequence guards, freshness, bounded stop, no automatic rearm\n";
}
