#include "execution_publication_core.hpp"
#include "execution_scheduler.hpp"
#include <cassert>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
using namespace d1monitor::execution3;
static CommitAck outcome(){
  CommitAck a;a.schema_version=1;a.handoff_id="conditional-handoff";a.grant_sequence=7;
  a.sequence=1;a.previous_commit_sequence=3;a.commit_sequence=4;
  a.execution_id="execution";a.sdk_session="sdk";a.transport_mode="isolated_mock";
  a.control_epoch=9;a.sdk_arm_generation=5;a.applied=a.write_submitted=true;
  a.applied_at=stamp(10.);a.demand_source_stamp=stamp(9.99);a.demand_body_source_stamp=stamp(9.98);
  a.valid_until=stamp(10.08);a.applied_velocity.linear.x=.2;return a;
}
int main(){
  using Result=CommitOutbox::Result;
  {CommitOutbox q;const auto first=outcome();assert(q.put(first)==Result::Queued);
   for(int i=0;i<10000;++i)assert(q.put(first)==Result::Coalesced);
   auto acknowledged=first;acknowledged.sequence=2;acknowledged.write_acknowledged=true;
   assert(q.put(acknowledged)==Result::Coalesced&&q.size()==1);
   assert(q.put(first)==Result::Coalesced&&q.size()==1); // no delayed downgrade
   CommitAck out;assert(q.pop(out)&&out==acknowledged&&!q.pop(out));
   assert(out.demand_source_stamp==first.demand_source_stamp&&out.valid_until==first.valid_until);}
  {CommitOutbox q;const auto first=outcome();assert(q.put(first)==Result::Queued);
   for(unsigned field=0;field<7;++field){auto altered=first;altered.sequence=field+2;altered.write_acknowledged=true;
    switch(field){case 0:altered.sdk_session="other";break;case 1:++altered.control_epoch;break;
     case 2:++altered.commit_sequence;break;case 3:altered.demand_source_stamp=stamp(9.991);break;
     case 4:altered.valid_until=stamp(10.09);break;case 5:altered.applied=false;break;
     case 6:altered.candidate_version.route_hash=std::string(64,'b');break;}
    assert(q.put(altered)==Result::Queued);}
   auto ninth=first;ninth.handoff_id="different";
   assert(q.size()==8&&q.put(ninth)==Result::Overflow&&q.size()==8);
   CommitAck out;assert(q.pop(out)&&out==first);assert(q.put(ninth)==Result::Queued);}
  // Model a middleware call blocked after taking its sample. The consumer has
  // no state/outbox lock while publishing, so producer CAS results stay bounded
  // and a 20Hz deadline is still advanced without a catch-up command burst.
  {CommitOutbox q;d1monitor::LatestMailbox<CommitAck> telemetry;
   std::mutex m;std::condition_variable wake;std::atomic<bool>blocked{false};bool release=false;
   assert(q.put(outcome())==Result::Queued);
   std::thread consumer([&]{CommitAck in;
     {std::lock_guard<std::mutex>lock(m);assert(q.pop(in));}
     blocked=true;std::unique_lock<std::mutex>lock(m);wake.wait(lock,[&]{return release;});
     assert(in==outcome());});
   while(!blocked.load())std::this_thread::yield();
   d1monitor::PeriodicDeadline deadline(10.05,.05);
   for(unsigned i=1;i<=10000;++i){auto latest=outcome();latest.sequence=i;
     {std::lock_guard<std::mutex>lock(m);telemetry.put(std::move(latest));q.put(outcome());}
     deadline.complete(10.+i*.05+.001);}
   {std::lock_guard<std::mutex>lock(m);assert(q.size()==1&&q.coalesced()==9999);
     const auto latest=telemetry.take();assert(latest&&latest->sequence==10000&&latest->applied_at==stamp(10.));
     assert(telemetry.overwritten==9999&&deadline.skipped()==0);release=true;}
   wake.notify_one();consumer.join();
  }
  std::cout<<"Commit outbox: duplicate/ACK coalescing, immutable outcomes and overflow passed\n";
}
