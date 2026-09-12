#include <cassert>
#include <iostream>
#include <limits>
#include "speed_report_core.hpp"
int main(){
  using d1monitor::SpeedReport;
  SpeedReport s;s.validate();assert(!s.request_due(false,false,0));
  assert(!s.request_due(true,false,0));s.robot(0);
  assert(!s.request_due(true,false,.9));assert(s.request_due(true,false,1));
  const auto epoch=s.generation;s.written(epoch,1,"");
  assert(s.write_complete&&s.state(1)=="waiting_ack");
  s.ack(true,50);assert(s.state(1)=="waiting_stream"&&!s.rate_ok(1));
  assert(!s.request_due(true,false,1.2));s.robot(3);
  assert(s.request_due(true,false,4));s.written(epoch,1,"late old attempt");assert(s.write_error.empty());
  s.written(epoch,2,"write failed");assert(s.state(4)=="write_failed");
  s.robot(6);assert(s.request_due(true,false,7));s.robot(9);
  assert(!s.request_due(true,false,10));assert(s.state(10)=="failed"&&s.attempts==3);
  s.disconnect();s.written(epoch,3,"late previous connection");assert(s.write_error.empty());
  s.robot(20);assert(!s.request_due(true,true,21));assert(s.attempts==0);
  assert(!s.sample(21,0,0,0));assert(s.request_due(true,false,21));
  for(int i=0;i<101;++i){const double t=21+i*.02;if(i%50==0)s.robot(t);assert(s.sample(t,.1,0,0));}
  assert(s.samples==101&&std::abs(s.observed_hz(23)-50)<1e-6);
  assert(s.state(23)=="streaming"&&!s.acknowledged);assert(!s.request_due(true,false,23.1));
  // A vanished stream uses the remaining finite retry budget, no restart loop.
  assert(!s.fresh(24));assert(s.request_due(true,false,24.1));
  assert(!s.sample(24.2,std::numeric_limits<double>::quiet_NaN(),0,0));assert(s.invalid_samples==1);
  SpeedReport slow;slow.robot(0);assert(slow.request_due(true,false,1));slow.ack(true,50);
  for(int i=0;i<31;++i)slow.sample(1+i*.05,0,0,0);
  assert(std::abs(slow.observed_hz(2.5)-20)<1e-6&&!slow.rate_ok(2.5));
  assert(slow.state(2.5)=="rate_mismatch");
  std::cout<<"PASS dedicated speed stream: finite retries, ACK != samples, measured Hz, replay, disconnect, stale stream and invalid data\n";
}
