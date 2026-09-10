#include "console_control_core.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace d1console;
struct Fixture {
  Core c;double now=10;
  Fixture(){c.connection(true);State s;s.motion=2;s.mode=1;s.control=2;s.speed=1;s.software=s.hardware=1;c.update(s,now);}
  void lease(){c.lease(true,"test",now);}
  std::vector<Command> tick(double dt=.05){now+=dt;lease();return c.tick(now);}
  void state(int motion,int mode=1){auto s=c.state;s.motion=motion;s.mode=mode;c.update(s,now);}
  void complete(const std::string& a,int motion,int mode=1){c.ack(a);now+=.1;lease();state(motion,mode);c.tick(now);now+=.1;lease();state(motion,mode);c.tick(now);}
  void own(){lease();assert(c.request("take_control",now).empty());auto out=tick();assert(out.size()==1&&out[0].name=="take_control");complete("take_control",2);assert(c.owned);}
};
int main(){
  {Fixture f;assert(f.c.tick(f.now).empty());assert(!f.c.owned);assert(!f.c.ready(f.now));assert(!f.c.request("stand",f.now).empty());}
  {Fixture f;f.own();assert(f.c.request("stand",f.now).empty());auto out=f.tick();assert(out[0].name=="stand");f.complete("stand",1);assert(f.c.busy());assert(!f.c.request("general_mode",f.now).empty());f.complete("stand",5);assert(!f.c.busy());assert(f.c.result=="SUCCEEDED");}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.c.ack("stand");assert(f.c.busy());f.state(5);f.tick();assert(f.c.busy());f.state(5);f.tick();assert(!f.c.busy());}
  {Fixture f;f.own();f.c.request("stand",f.now);auto cmds=f.tick();f.c.lease(false,"test",f.now);assert(!f.c.valid(cmds[0],f.now));assert(!f.c.busy());assert(!f.c.prepared);assert(f.c.tick(f.now).empty());}
  {Fixture f;f.own();f.c.request("prepare_navigation",f.now);f.tick();f.c.connection(false);f.c.connection(true);assert(!f.c.owned&&!f.c.busy()&&!f.c.prepared);assert(f.tick().empty());}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.now+=16;f.lease();f.state(2);f.c.tick(f.now);assert(f.c.uncertain);assert(!f.c.request("stand",f.now).empty());f.c.ack("stand");assert(f.tick().empty());}
  {Fixture f;f.own();f.state(4);assert(!f.c.request("stand",f.now).empty());assert(!f.c.request("prepare_navigation",f.now).empty());assert(f.c.request("unlock_to_stand",f.now).empty());assert(f.tick()[0].name=="stand");}
  {Fixture f;f.own();f.state(3);assert(!f.c.request("lie_down",f.now).empty());assert(!f.c.request("general_mode",f.now).empty());}
  {Fixture f;f.own();f.c.request("prepare_navigation",f.now);assert(f.tick()[0].name=="stand");f.complete("stand",1);assert(f.c.active=="stand");f.complete("stand",5);assert(f.c.active=="general_mode");f.complete("general_mode",5);assert(f.c.active=="set_speed");f.complete("set_speed",5);assert(f.c.ready(f.now));assert(f.c.tick(f.now).empty());assert(f.c.velocity(.2,0,0,f.now));assert(!f.c.velocity(std::numeric_limits<double>::quiet_NaN(),0,0,f.now));assert(!f.c.velocity(.31,0,0,f.now));auto cmd=f.tick();assert(cmd[0].x==.2);f.now+=.3;f.lease();cmd=f.c.tick(f.now);assert(cmd.size()==1&&cmd[0].x==0);assert(!f.c.moving);}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.c.control_lost();assert(!f.c.owned&&!f.c.busy());assert(!f.c.request("prepare_navigation",f.now).empty());}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();assert(f.c.request("soft_estop",f.now).empty());auto out=f.tick();assert(out[0].name=="soft_estop");f.c.ack("stand");assert(f.c.active=="soft_estop");}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.c.fault_report();assert(f.c.fault&&!f.c.busy());assert(!f.c.request("reset_error",f.now).empty());assert(f.c.request("soft_estop",f.now).empty());}
  {Fixture f;f.own();f.c.replay_detected();for(const auto& a:{"stand","soft_estop","recover_estop","prepare_navigation"})assert(!f.c.request(a,f.now).empty());assert(f.tick().empty());}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.now+=.7;assert(f.c.tick(f.now).empty());assert(!f.c.busy());f.lease();assert(f.c.tick(f.now).empty());}
  {Fixture f;f.own();f.c.request("stand",f.now);f.tick();f.now+=3;f.lease();f.c.tick(f.now);assert(!f.c.busy()&&f.c.uncertain);}
  {Fixture f;f.own();f.state(5);f.c.request("release_control",f.now);f.c.lease(false,"test",f.now);auto out=f.c.tick(f.now);assert(out.size()==1&&out[0].name=="stop");f.now+=.1;f.state(5);f.c.tick(f.now);f.now+=.1;f.state(5);out=f.c.tick(f.now);assert(out.size()==1&&out[0].name=="release_control");assert(f.c.valid(out[0],f.now));}
  std::cout<<"16 guarded FSM scenarios passed (no ROS, no SDK, no robot commands)\n";
}
