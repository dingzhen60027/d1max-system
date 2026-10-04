#include "execution_scheduler.hpp"
#include "mc_report_core.hpp"
#include <cassert>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
int main(){
  d1monitor::PeriodicDeadline d(10.,.05);assert(d.next()==10.);d.complete(10.001);
  assert(std::abs(d.next()-10.05)<1e-9&&d.skipped()==0);
  d.complete(10.401);assert(d.next()>10.401&&d.skipped()==7); // no catch-up Move burst
  bool invalid=false;try{d.complete(9.);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
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
