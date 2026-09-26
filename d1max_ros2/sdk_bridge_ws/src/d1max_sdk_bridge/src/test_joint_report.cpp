#include "joint_report_core.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
struct Data {
  std::vector<std::string> names{"fl1","fl2"};
  std::vector<double> positions{.1,-.2},velocities{0.,.3},efforts{1.,2.};
};
static d1monitor::JointPacket packet(double t=1.,double wall=101.){
  d1monitor::JointPacket out;assert(d1monitor::copy_joint_packet(Data{},t,wall,out));return out;
}
static d1monitor::JointReport ready(){
  d1monitor::JointReport r;r.enabled=true;r.connection(true,false,0.);
  assert(!r.request_due(.49));assert(r.request_due(.5));return r;
}
int main(){
  {auto p=packet();assert(p.count==2&&p.has_velocity&&p.has_effort);
   assert(std::string(p.joints[0].name.data())=="fl1"&&p.joints[0].position==.1);
   assert(p.joints[1].velocity==.3&&p.joints[1].effort==2.);
   assert(p.arrival==1.&&p.received==101.);}
  {Data d;d.velocities.clear();d.efforts.clear();d1monitor::JointPacket p;
   assert(d1monitor::copy_joint_packet(d,1,101,p));assert(!p.has_velocity&&!p.has_effort);}
  {for(int shape=0;shape<5;++shape){Data d;d1monitor::JointPacket p;
    if(shape==0)d.names.clear();
    if(shape==1)d.positions.pop_back();
    if(shape==2)d.velocities.pop_back();
    if(shape==3)d.efforts.push_back(0);
    if(shape==4)d.names.resize(65,"x");
    assert(!d1monitor::copy_joint_packet(d,1,101,p));}}
  {for(const std::string& name:std::vector<std::string>{"","fl 1","fl\n1",std::string(65,'a'),std::string("a\0b",3),"fl2"}){
    Data d;d.names[0]=name;d1monitor::JointPacket p;assert(!d1monitor::copy_joint_packet(d,1,101,p));}}
  {Data d;d.names[0]=std::string(64,'a');d1monitor::JointPacket p;
   assert(d1monitor::copy_joint_packet(d,1,101,p));assert(p.joints[0].name[64]=='\0');}
  {Data d;d.names.clear();for(int i=0;i<64;++i)d.names.push_back("joint_"+std::to_string(i));
   d.positions.resize(64,0);d.velocities.clear();d.efforts.clear();d1monitor::JointPacket p;
   assert(d1monitor::copy_joint_packet(d,1,101,p)&&p.count==64);}
  {for(double bad:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
    for(int field=0;field<3;++field){Data d;d1monitor::JointPacket p;
      (field==0?d.positions:field==1?d.velocities:d.efforts)[0]=bad;
      assert(!d1monitor::copy_joint_packet(d,1,101,p));}}
  {Data d;d1monitor::JointPacket p;
   assert(!d1monitor::copy_joint_packet(d,-1,101,p));
   assert(!d1monitor::copy_joint_packet(d,1,0,p));
   assert(!d1monitor::copy_joint_packet(d,1,std::numeric_limits<double>::quiet_NaN(),p));}
  {d1monitor::JointReport r;r.connection(true,false,0);
   assert(!r.request_due(100)&&!r.sample(packet(),1,101)&&r.state(100)=="disabled");
   r.ack(true,100);assert(!r.acknowledged&&r.total_attempts==0);}
  {auto r=ready();r.ack(true,.6);assert(r.acknowledged&&!r.fresh(.6)&&r.state(.6)=="waiting_stream");
   assert(r.sample(packet(),1.01,101.01));assert(r.fresh(1.01));
   assert(r.last_received==101.&&r.samples==1&&r.state(1.01)=="streaming_receipt_only");
   assert(!r.fresh(1.25));assert(r.last_received==101.);}
  {auto r=ready();assert(r.sample(packet(),1,101));
   assert(r.state(1)=="streaming_receipt_only_unconfirmed");
   assert(!r.request_due(3.49)); // no retry in existing request window
   for(double t=1.02;t<9.;t+=.02){assert(r.sample(packet(t,100+t),t,100+t));assert(!r.request_due(t));}
   assert(r.total_attempts==1&&!r.acknowledged);}
  {auto r=ready();assert(!r.sample(packet(.49,100.49),.5,100.5));
   assert(!r.sample(packet(),1.25,101.25));assert(!r.sample(packet(),.99,101));
   assert(!r.sample(packet(),1.01,101.3));assert(!r.sample(packet(),1.01,100.99));
   assert(r.samples==0&&r.stale_samples==4);}
  {auto r=ready();assert(r.sample(packet(),1,101));
   assert(!r.sample(packet(),1.01,101.01));
   assert(!r.sample(packet(.99,101.01),1.01,101.01));
   assert(!r.sample(packet(1.01,100.99),1.01,101.01));
   assert(r.samples==1&&r.stale_samples==3);}
  {auto r=ready();assert(r.request_due(3.5));assert(r.request_due(6.5));
   for(double t=7;t<1000;++t)assert(!r.request_due(t));
   assert(r.total_attempts==3&&r.state(10)=="retry_exhausted");
   assert(r.sample(packet(10,110),10,110));assert(r.state(10)=="streaming_receipt_only_unconfirmed");}
  {auto r=ready();const auto old=r.generation;const auto id=r.total_attempts;
   assert(r.sample(packet(),1,101));r.connection(false,false,1.1);
   assert(!r.fresh(1.1)&&r.last_received==0&&r.state(1.1)=="disconnected");
   r.ack(true,1.2);r.written(old,id,"late");assert(!r.acknowledged&&!r.write_complete);
   r.connection(true,false,2);assert(!r.request_due(2.49));assert(r.request_due(2.5));
   assert(!r.sample(packet(),2.51,102.51));r.ack(true,2.4);assert(!r.acknowledged);
   r.written(old,id,"late");assert(!r.write_complete);
   assert(r.sample(packet(2.51,102.51),2.51,102.51));assert(r.total_attempts==2);}
  {auto r=ready();r.connection(true,true,.6);
   assert(!r.request_due(100)&&!r.sample(packet(),1,101));assert(r.state(1)=="replay_blocked");}
  {auto r=ready();r.written(r.generation,r.total_attempts,"denied");
   assert(r.write_complete&&r.write_error=="denied"&&r.state(.6)=="write_failed");
   r.written(r.generation+1,r.total_attempts,"bad");assert(r.write_error=="denied");
   assert(r.request_due(3.5));assert(!r.write_complete&&r.write_error.empty());
   r.written(r.generation,r.total_attempts-1,"old");assert(!r.write_complete);}
  {Data d;d1monitor::JointPacket p;assert(d1monitor::copy_joint_packet(d,1,101,p));
   d.names[0]="different";d.positions[0]=9;
   assert(std::string(p.joints[0].name.data())=="fl1"&&p.joints[0].position==.1);}
  std::cout<<"Joint receipt-only core: bounded copy, names, shapes, finite values, stale/order, default-off, ACK, retry and reconnect checks passed\n";
}
