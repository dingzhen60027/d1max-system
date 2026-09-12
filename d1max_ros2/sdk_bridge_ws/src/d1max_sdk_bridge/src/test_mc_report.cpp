#include "mc_report_core.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using d1monitor::McReport;
const float zero[3]={0,0,0};
static McReport ready() {
  McReport r;r.validate();r.robot(0);
  assert(!r.request_due(true,false,.5));assert(r.request_due(true,false,1.));
  return r;
}
int main() {
  {d1monitor::Inbox<int,2> q;q.push(1);q.push(2);q.push(3);int out=0;
   assert(q.dropped==1&&q.pop(out)&&out==2);assert(q.pop(out)&&out==3);assert(!q.pop(out));}
  {auto r=ready();r.ack(true);assert(!r.fresh(1.)&&r.state(1.)=="waiting_stream");
   for(int i=0;i<101;++i){const auto now=1.+i*.02;
     assert(r.sample(now,100.+i*.02,1000000000ULL+i*20000000ULL,zero,zero,now));}
   r.robot(3.);assert(r.rate_ok(3.));assert(std::abs(r.observed_hz(3.)-50.)<.001);
   assert(std::abs(r.source_hz(3.)-50.)<.001);assert(std::abs(r.stamp_unix-102.)<1e-8);
   assert(r.state(3.)=="streaming");assert(!r.request_due(true,false,3.));
   assert(!r.fresh(3.4));assert(r.observed_hz(3.4)==0.);}
  {auto r=ready();assert(r.sample(1.,100.,1000000000,zero,zero,1.));
   assert(r.state(1.)=="streaming_unconfirmed");  // data does not invent a configuration ACK
   assert(!r.sample(1.02,100.02,0,zero,zero,1.02));
   assert(!r.sample(1.02,100.02,1000000000,zero,zero,1.02));
   assert(!r.sample(1.02,100.02,999999999,zero,zero,1.02));
   assert(r.timestamp_rejections==3&&r.samples==1);
   assert(!r.sample(1.02,100.02,1020000000,zero,zero,1.5));assert(r.stale_samples==1);
   float bad[3]={0,std::numeric_limits<float>::quiet_NaN(),0};
   assert(!r.sample(1.02,100.02,1020000000,bad,zero,1.02));assert(r.invalid_samples==1);
   assert(!r.sample(1.02,110.,1020000000,zero,zero,1.02));assert(r.timestamp_rejections==4);}
  {auto r=ready();r.ack(true);r.robot(4.);assert(r.request_due(true,false,4.));
   r.robot(7.);assert(r.request_due(true,false,7.));r.robot(10.);
   assert(!r.request_due(true,false,10.)&&r.state(10.)=="retry_cooldown");
   const auto old=r.generation;r.request_due(false,false,10.1);
   r.written(old,3,"late error");assert(r.write_error.empty());
   r.robot(11.);assert(r.request_due(true,false,12.));assert(r.generation!=old);
   assert(!r.sample(7.,100.,1,zero,zero,12.));}
  {auto r=ready();assert(!r.request_due(true,true,1.1));
   assert(!r.sample(1.2,100.,1,zero,zero,1.2));assert(r.state(1.2)=="replay_blocked");}
  {auto r=ready(); // source intervals survive callback batching/jitter
   assert(r.sample(1.,100.,1000000000,zero,zero,1.));
   assert(r.sample(1.06,100.06,1020000000,zero,zero,1.06));
   assert(std::abs(r.stamp_unix-100.02)<1e-8);}
  {auto r=ready(); // No ACK/data: retry indefinitely but with a capped backoff.
   r.robot(4.);assert(r.request_due(true,false,4.));
   r.robot(7.);assert(r.request_due(true,false,7.));
   r.robot(10.);assert(!r.request_due(true,false,10.));
   assert(r.next_retry_in(10.)==15.);
   r.robot(24.9);assert(!r.request_due(true,false,24.9));
   r.robot(25.);assert(r.request_due(true,false,25.));
   assert(r.attempts==1&&r.total_attempts==4&&r.retry_cycles==1);
   r.written(r.generation,1,"old write callback");assert(!r.write_complete);
   r.written(r.generation,4,"");assert(r.write_complete&&r.write_error.empty());
   r.robot(28.);assert(r.request_due(true,false,28.));
   r.robot(31.);assert(r.request_due(true,false,31.));
   r.robot(34.);assert(r.next_retry_in(34.)==30.);
   r.robot(64.);assert(r.request_due(true,false,64.));
   assert(r.cooldown()==60.);r.retry_cycles=20;assert(r.cooldown()==60.);}
  {auto r=ready(); // Live stream with missing ACK must not be toggled/retried.
   for(int i=0;i<301;++i){double t=1.+i*.02;r.robot(t);
     assert(r.sample(t,100.+i*.02,1000000000ULL+i*20000000ULL,zero,zero,t));
     assert(!r.request_due(true,false,t));}
   assert(r.total_attempts==1&&r.attempts==0&&!r.acknowledged);
   assert(r.state(7.)=="streaming_unconfirmed");
   r.robot(7.4);assert(!r.request_due(true,false,7.4));assert(r.state(7.4)=="stream_stale");
   r.robot(8.);assert(r.request_due(true,false,8.));assert(r.attempts==1&&r.total_attempts==2);
   assert(r.sample(8.02,107.02,8020000000ULL,zero,zero,8.02));
   assert(r.observed_hz(8.02)==0.); // no pre-outage samples in the frequency window
   r.ack(true);assert(r.state(8.02)=="measuring_rate");}
  {auto r=ready(); // Robot becomes ready after retries exhausted, no socket restart.
   for(double t:{4.,7.}){r.robot(t);assert(r.request_due(true,false,t));}
   r.robot(10.);assert(r.state(10.)=="retry_cooldown");
   r.ack(true);
   for(int i=0;i<111;++i){double t=10.+i*.02;r.robot(t);
     assert(r.sample(t,100.+i*.02,1000000000ULL+i*20000000ULL,zero,zero,t));
     assert(!r.request_due(true,false,t));}
   assert(r.attempts==0&&r.state(12.2)=="streaming");}
  {auto r=ready(); // Never send telemetry while robot state is stale or replay latched.
   assert(!r.request_due(true,false,20.));assert(r.state(20.)=="waiting_robot_state");
   r.robot(20.);assert(r.request_due(true,false,20.));r.robot(21.);
   assert(!r.request_due(true,true,21.));assert(r.next_retry_in(21.)<0);}
  {auto r=ready(); // A retry must not discard valid queued samples of this session.
   r.robot(4.);assert(r.request_due(true,false,4.));
   assert(r.sample(3.99,103.,1000000000ULL,zero,zero,4.01));}
  {for(double bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}){
     McReport r;r.cooldown_sec=bad;bool threw=false;
     try{r.validate();}catch(const std::invalid_argument&){threw=true;}assert(threw);}}
  std::cout<<"MC core: queue, rate, ACK, retry, timestamp, stale, replay and clock checks passed\n";
}
