#include "sdk_ownership_core.hpp"
#include "mc_report_core.hpp"
#include <cassert>
#include <iostream>
using d1monitor::Ownership;
static Ownership ready(){Ownership o;o.enabled=true;o.validate();o.connection(true,false);o.robot(1,0.);return o;}
int main(){
  {auto o=ready(); // APP source or absent MC never authorizes a request.
   for(int i=1;i<100;++i){o.robot(1,i);assert(!o.due(i));}
   assert(o.state(99)=="waiting_app"&&!o.allow_mc_config());
   o.robot(2,100);assert(!o.due(100)&&!o.owns(100));}
  {auto o=ready();o.available(.1);assert(!o.due(.5));assert(o.due(.6));
   for(int i=0;i<10;++i){o.available(.7);assert(!o.due(.7));}
   o.written(o.request_id,"");assert(!o.acknowledged&&!o.owns(.7));
   o.robot(2,.8);assert(!o.owns(.8)); // enum alone is not an ACK
   o.ack(0,"",.9);assert(!o.owns(.9));o.robot(2,1.);assert(!o.owns(1.));
   o.robot(2,1.);assert(!o.owns(1.)); // duplicate RobotState not a confirmation
   o.robot(2,2.);assert(o.owns(2.)&&o.reconfigure&&o.total_requests==1);
   o.reconfigure=false;o.lost();assert(!o.owns(2.1)&&!o.allow_mc_config());
   o.robot(1,3.);assert(!o.due(3.));o.available(3.1);assert(o.due(3.7));
   o.ack(0,"",3.8);o.robot(2,4.);o.robot(2,5.);assert(o.owns(5.)&&o.total_requests==2);
   assert(!o.owns(8.));} // stale state cannot claim current control
  {auto o=ready();o.available(.1);assert(o.due(.7));const auto old=o.request_id;
   o.tick(4.);assert(o.state(4.)=="takeover_timeout");
   o.ack(0,"late",4.1);o.robot(2,4.2);o.robot(2,5.2);assert(!o.owns(5.2));
   o.lost();o.available(6.);o.robot(1,6.);assert(!o.due(7.));
   o.connection(false,false);o.connection(true,false);o.robot(1,8.);o.available(8.);
   assert(o.due(8.6));o.written(old,"old write error");assert(o.pending&&!o.uncertain);}
  {auto o=ready();o.available(.1);assert(o.due(.7));o.lost();
   o.ack(0,"",.8);o.available(.9);o.robot(2,1.);assert(!o.due(1.5)&&!o.owns(1.5));}
  {auto o=ready();o.available(.1);assert(o.due(.7));o.ack(10002,"APP owns control",.8);
   assert(o.state(.8)=="takeover_rejected"&&!o.allow_mc_config());
   o.available(1.);o.robot(1,1.);assert(!o.due(2.));}
  {auto o=ready();o.available(.1);assert(o.due(.7));o.ack(0,"",.8);
   o.robot(1,1.);o.robot(1,2.);o.tick(5.);assert(!o.owns(5.)&&o.uncertain);}
  {auto o=ready();o.available(.1);o.enabled=false;assert(!o.due(.7));
   o.enabled=true;o.connection(true,true);assert(!o.due(.7));o.ack(0,"",.8);assert(!o.owns(.8));}
  {auto o=ready();o.available(.1);assert(!o.due(3.));o.robot(1,6.);assert(!o.due(6.));}
  {auto o=ready();o.available(.1);assert(o.due(.7));o.written(o.request_id,"write failed");
   o.ack(0,"",.8);assert(!o.owns(.8)&&o.uncertain);}
  {auto o=ready();o.settle_sec=-1;bool refused=false;try{o.validate();}catch(const std::invalid_argument&){refused=true;}assert(refused);}
  { // MC requests pause while APP owns the session; handoff is not data readiness.
   auto o=ready();d1monitor::McReport mc;mc.robot(0.);
   assert(!mc.request_due(true,false,1.,o.allow_mc_config()));assert(!mc.total_attempts);
   o.available(1.);o.robot(1,1.);assert(o.due(1.6));o.ack(0,"",1.7);o.robot(2,2.);o.robot(2,3.);
   assert(o.owns(3.));mc.disconnect();mc.robot(o.last_robot);
   assert(!mc.request_due(true,false,3.,o.allow_mc_config()));mc.robot(4.);
   assert(mc.request_due(true,false,4.,o.allow_mc_config()));assert(!mc.fresh(4.)&&!mc.rate_ok(4.));
   mc.ack(true);assert(!mc.fresh(4.));const float v[3]={0,0,0};
   for(unsigned i=0;i<=100;++i)assert(mc.sample(4.+i*.02,100.+i*.02,1000000000ULL+i*20000000ULL,v,v,4.+i*.02));
   assert(mc.rate_ok(6.)&&mc.state(6.)=="streaming");}
  std::cout<<"SDK handoff: APP priority, repeated cycles, ACK/state/data confirmation, timeout, replay and MC gating passed\n";
}
