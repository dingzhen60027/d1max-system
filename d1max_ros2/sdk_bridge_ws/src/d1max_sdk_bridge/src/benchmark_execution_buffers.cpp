// Pure production-core benchmark. No ROS executor, vendor library, SDK
// connection or movement. Compile this same file against either header tree;
// define D1MAX_BUFFER_BASELINE for the prior diagnostic/publication path.
#include <mc_report_core.hpp>
#include <execution_transport_core.hpp>
#ifndef D1MAX_BUFFER_BASELINE
#include <execution_publication_core.hpp>
#endif
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <new>
#include <string>

namespace allocation {
struct alignas(std::max_align_t) Header {size_t bytes;};
static size_t live=0,peak=0,count=0,requested=0;static bool measure=false;
static void reset(){peak=live;count=requested=0;measure=true;}
}
#ifndef D1MAX_BUFFER_TIMING_ONLY
void* operator new(std::size_t n){
  auto* h=static_cast<allocation::Header*>(std::malloc(sizeof(allocation::Header)+n));
  if(!h)throw std::bad_alloc();h->bytes=n;allocation::live+=n;
  if(allocation::measure){++allocation::count;allocation::requested+=n;allocation::peak=std::max(allocation::peak,allocation::live);}
  return h+1;
}
void operator delete(void* p) noexcept{if(!p)return;auto* h=static_cast<allocation::Header*>(p)-1;allocation::live-=h->bytes;std::free(h);}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void*p) noexcept{::operator delete(p);}
void operator delete(void*p,std::size_t) noexcept{::operator delete(p);}
void operator delete[](void*p,std::size_t) noexcept{::operator delete(p);}
#endif
static volatile double checksum=0.;
template<class F> void benchmark(const char* name,size_t operations,F body){
  allocation::reset();const size_t entry=allocation::live;
  const auto start=std::chrono::steady_clock::now();body();
  const auto duration=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count();
  allocation::measure=false;
  std::cout<<"{\"name\":\""<<name<<"\",\"operations\":"<<operations
    <<",\"ns_per_op\":"<<duration/operations<<",\"allocations\":"<<allocation::count
    <<",\"requested_bytes\":"<<allocation::requested<<",\"entry_heap_bytes\":"<<entry
    <<",\"peak_heap_bytes\":"<<allocation::peak<<",\"exit_heap_bytes\":"<<allocation::live<<"}\n";
}
static d1monitor::McReport ready(){d1monitor::McReport r;r.robot(0.);r.request_due(true,false,1.);return r;}
static const float zero[3]={0.,0.,0.};
// Preserve the callback boundary: a compile-time zero fixture must not let the
// optimizer specialize away the production core's buffer and validity work.
__attribute__((noinline,noclone)) static bool acceptedSample(d1monitor::McReport& r,
    double arrival,double received,uint64_t source,const float* velocity,const float* omega){
  return r.sample(arrival,received,source,velocity,omega,arrival);
}
static d1monitor::execution3::CommitAck ack(){
  using namespace d1monitor::execution3;CommitAck a;a.schema_version=1;
  a.handoff_id="benchmark-handoff-with-bound-identity";a.execution_id="benchmark-execution-transaction";
  a.sdk_session="0123456789abcdef0123456789abcdef";a.transport_mode="isolated_mock";
  a.incumbent_version.route_hash=a.candidate_version.route_hash=std::string(64,'a');
  a.incumbent_version.localization_seed_id=a.candidate_version.localization_seed_id="localization-seed-with-bound-identity";
  a.measured_pose.header.frame_id="d1max_loc_odom";a.applied=a.write_submitted=true;
  a.control_epoch=a.sdk_arm_generation=a.commit_sequence=a.sequence=1;return a;
}
int main(){
#ifdef D1MAX_BUFFER_BASELINE
  std::cout<<"{\"implementation\":\"baseline_b344f52\",\"mc_object_bytes\":"<<sizeof(d1monitor::McReport)<<"}\n";
#else
  std::cout<<"{\"implementation\":\"optimized_source\",\"mc_object_bytes\":"<<sizeof(d1monitor::McReport)
    <<",\"mc_snapshot_bytes\":"<<sizeof(d1monitor::McReport::Snapshot)<<"}\n";
#endif
#ifdef D1MAX_BUFFER_TIMING_ONLY
  std::cout<<"{\"allocation_instrumentation\":false}\n";
#else
  std::cout<<"{\"allocation_instrumentation\":true}\n";
#endif
  {auto r=ready();const size_t n=1000000;
   benchmark("mc_accepted_sample_50hz",n,[&]{for(size_t i=0;i<n;++i){const double t=1.+i*.02;
     checksum+=acceptedSample(r,t,100.+i*.02,1000000000ULL+i*20000000ULL,zero,zero);}});}
  {auto r=ready();for(size_t i=0;i<=100;++i){const double t=1.+i*.02;r.sample(t,100.+i*.02,1000000000ULL+i*20000000ULL,zero,zero,t);}
   const size_t n=500000;benchmark("mc_diagnostic_snapshot_50hz",n,[&]{for(size_t i=0;i<n;++i){
#ifdef D1MAX_BUFFER_BASELINE
     const auto copy=r;const auto state=copy.state(3.);
     checksum+=copy.samples+copy.observed_hz(3.)+copy.source_hz(3.)+state.size()+copy.fresh(3.)+copy.rate_ok(3.)+copy.next_retry_in(3.);
#else
     const auto copy=r.snapshot(3.);
     checksum+=copy.samples+copy.observed_hz+copy.source_hz+copy.state.size()+copy.fresh+copy.rate_ok+copy.next_retry_in;
#endif
   }});}
  {const auto original=ack();const size_t n=200000;
#ifdef D1MAX_BUFFER_BASELINE
   d1monitor::Inbox<d1monitor::execution3::CommitAck,8> q;
#else
   d1monitor::execution3::CommitOutbox q;
#endif
   benchmark("commit_enqueue_drain_distinct",n,[&]{for(size_t base=0;base<n;base+=8){
     for(size_t i=0;i<8;++i){auto value=original;value.commit_sequence=base+i+1;value.sequence=base+i+1;
#ifdef D1MAX_BUFFER_BASELINE
       q.push(value);
#else
       q.put(std::move(value));
#endif
     }
#ifdef D1MAX_BUFFER_BASELINE
     std::deque<d1monitor::execution3::CommitAck> batch;d1monitor::execution3::CommitAck value;
     while(q.pop(value))batch.push_back(std::move(value));
     for(const auto& item:batch)checksum+=item.sequence;
#else
     std::array<d1monitor::execution3::CommitAck,8>batch;size_t count=0;
     while(count<batch.size()&&q.pop(batch[count]))++count;
     for(size_t i=0;i<count;++i)checksum+=batch[i].sequence;
#endif
   }});}
  std::cout<<"{\"checksum\":"<<checksum<<"}\n";
}
