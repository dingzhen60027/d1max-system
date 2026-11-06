#include "execution_scheduler.hpp"
#include "mc_report_core.hpp"
#include <cassert>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
int main(){
  d1monitor::PeriodicDeadline d(10.,.05);assert(d.next()==10.);d.complete(10.001);
  assert(std::abs(d.next()-10.05)<1e-9&&d.skipped()==0);
  d.complete(10.401);assert(d.next()>10.401&&d.skipped()==7); // no catch-up Move burst
  bool invalid=false;try{d.complete(9.);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  d1monitor::WriterDeadline writer(10.,.05);writer.complete(10.,10.001);
  assert(std::abs(writer.next()-10.05)<1e-9);
  writer.complete(10.06,10.061);assert(std::abs(writer.next()-10.11)<1e-9);
  writer.complete(writer.next(),10.111);assert(std::abs(writer.next()-10.16)<1e-9);
  writer.complete(10.50,10.701);assert(writer.next()>10.701&&writer.skipped()>0);
  invalid=false;try{writer.complete(10.6,10.8);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  // The watchdog can check an empty/HOLD input without spending the first
  // real command's normal 50 ms opportunity.
  d1monitor::CommandWriterDeadline commands(10.,.05);
  commands.complete(10.,10.001,0,true);
  commands.observe(1);assert(commands.next()<=10.001&&commands.normalDue(10.001));
  commands.complete(10.001,10.003,1,true,10.002);
  assert(!commands.pending()); // watchdog must not resend this old nonzero
  commands.observe(2);assert(std::abs(commands.next()-10.05)<1e-9);
  assert(!commands.normalDue(10.05)&&commands.normalDue(10.052001));
  commands.complete(10.05,10.0501,2,false);
  assert(std::abs(commands.next()-10.052)<1e-9);
  // The original source watchdog can stop earlier than the next nonzero
  // opportunity; it neither authorizes movement nor renews source evidence.
  assert(std::abs(commands.next(10.051)-10.051)<1e-9);
  commands.complete(10.051,10.0511,2,false);
  commands.complete(10.052,10.060,2,true,10.058);
  commands.observe(3);assert(!commands.normalDue(10.1019)&&commands.normalDue(10.102001));
  // A new command arriving during a blocked vendor call is still pending;
  // only the revision captured at selection is consumed.
  commands.observe(4);commands.complete(10.108,10.201,3,true,10.109);
  assert(commands.pending()&&!commands.normalDue(10.201)&&commands.normalDue(10.202001));
  invalid=false;try{commands.complete(10.201,10.202,4,false,10.201);}
    catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  // A burst coalesces to one newest opportunity rather than replaying 1000
  // notifications. Early event wakes never permit another nonzero admission.
  d1monitor::CommandWriterDeadline burst(20.,.05);burst.observe(1);
  burst.complete(20.,20.001,1,true,20.0005);
  for(uint64_t revision=2;revision<=1001;++revision)burst.observe(revision);
  assert(burst.pending()&&!burst.normalDue(20.04)&&burst.observed()==1001);
  burst.complete(20.051,20.052,1001,true,20.051);
  assert(!burst.pending()&&!burst.normalDue(20.10));
  // Small execution lateness never accumulates against a fixed producer.
  d1monitor::CommandWriterDeadline phase(30.,.05);phase.observe(1);
  phase.complete(30.,30.0002,1,true,30.0002);
  for(uint64_t i=1;i<=10000;++i){
    const auto due=30.0002+i*.05;phase.observe(i+1);
    const auto jitter=(i%4)*.0005;
    assert(phase.normalDue(due+jitter+1e-10));
    phase.complete(due+jitter+1e-10,due+jitter+.0002,i+1,true,due+jitter+.0001);
    assert(!phase.normalDue(due+.0499)&&phase.normalDue(due+.050001));
  }
  // A 40 ms late send must skip the near 10 ms catch-up slot. Even a
  // multi-second blocked vendor returns only a latest opportunity, no queue.
  for(double late:{.04,.15,.5}){
    d1monitor::CommandWriterDeadline skip(40.,.05);skip.observe(1);
    skip.complete(40.,40.,1,true,40.);
    skip.observe(100);const double actual=40.05+late;
    skip.complete(actual,actual+.0001,100,true,actual);
    assert(!skip.pending()&&!skip.normalDue(actual+.0249));
    skip.observe(101);assert(skip.pending());
    assert(skip.normalDue(actual+.0751));
  }
  // Force publication into the exact predicate-to-wait race window. The old
  // atomic+notify form completes while the waiter still owns the wait mutex;
  // the production helper must instead wait until condition_variable::wait
  // atomically releases it and registers the waiter. No vendor/ROS is used.
  for(bool paired:{false,true}){
    std::mutex wait_mutex;std::condition_variable changed;std::atomic<uint64_t>revision{0};
    std::promise<void>predicate_checked,release_predicate,notifier_entered,notifier_done,waiter_done;
    auto release=release_predicate.get_future().share();auto entered=notifier_entered.get_future();
    auto notified=notifier_done.get_future();auto finished=waiter_done.get_future();
    bool first=true;
    std::thread waiter([&]{std::unique_lock<std::mutex>lock(wait_mutex);
      const bool received=changed.wait_for(lock,std::chrono::seconds(2),[&]{
        const auto observed=revision.load(std::memory_order_acquire);
        if(first){first=false;predicate_checked.set_value();release.wait();}
        return observed!=0;});
      assert(received);waiter_done.set_value();});
    predicate_checked.get_future().wait();
    std::thread notifier([&]{notifier_entered.set_value();
      if(paired)d1monitor::notifyWriterRevision(wait_mutex,changed,revision);
      else {revision.fetch_add(1,std::memory_order_release);changed.notify_one();}
      notifier_done.set_value();});
    entered.wait();
    const auto publication=notified.wait_for(paired?std::chrono::milliseconds(20):std::chrono::milliseconds(1000));
    if(paired)assert(publication==std::future_status::timeout);
    else assert(publication==std::future_status::ready); // deterministic old race
    release_predicate.set_value();notified.wait();
    if(!paired){
      // Cleanup wake only: the old notification already ran before entering
      // wait. Repeated cleanup avoids relying on absence of spurious wakeups.
      while(finished.wait_for(std::chrono::milliseconds(1))!=std::future_status::ready)changed.notify_one();
    }else assert(finished.wait_for(std::chrono::seconds(1))==std::future_status::ready);
    notifier.join();waiter.join();assert(revision.load()==1);
  }
  struct Sample{uint64_t seq;double original_source;};
  d1monitor::LatestMailbox<Sample> mailbox;std::mutex mutex;std::condition_variable wake;
  std::atomic<bool> consumer_blocked{false};bool release=false;
  // The slow consumer shares only a short mailbox-copy lock. It deliberately
  // blocks after the copy, as real JSON serialization / publish must do.
  std::thread diagnostic([&]{std::unique_lock<std::mutex>lock(mutex);
    mailbox.put({1,1.});auto first=mailbox.take();assert(first&&first->seq==1);consumer_blocked=true;
    wake.wait(lock,[&]{return release;});});
  while(!consumer_blocked.load())std::this_thread::yield();
  for(uint64_t i=2;i<=10002;++i){std::lock_guard<std::mutex>lock(mutex);mailbox.put({i,i*.02});}
  {std::lock_guard<std::mutex>lock(mutex);auto latest=mailbox.take();assert(latest&&latest->seq==10002);
    assert(latest->original_source==10002*.02&&mailbox.overwritten==10000&&!mailbox.pending);
    release=true;}wake.notify_one();diagnostic.join();
  d1monitor::McReport rate;rate.configureAcceptedRate(100.,90.,110.);
  assert(rate.expected_hz==100.&&rate.minimum_hz==90.&&rate.maximum_hz==110.);
  invalid=false;try{rate.configureAcceptedRate(100.,120.,110.);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  std::cout<<"steady writer deadline / bounded latest mailbox / accepted MC rate assertions passed\n";
}
