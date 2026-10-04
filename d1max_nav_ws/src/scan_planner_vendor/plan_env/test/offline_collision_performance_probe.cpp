#include <plan_env/grid_map.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <string>
#include <sys/resource.h>

// Offline measurement only: no node, executor, discovery or transport. Count
// C++ heap requests rather than inferring allocator cost from process RSS.
namespace measurement {
struct alignas(std::max_align_t) Header {std::size_t size;bool tracked;};
bool enabled=false;
std::size_t calls=0,bytes=0,live=0,peak=0;
void reset() {calls=bytes=live=peak=0;enabled=true;}
void* allocate(std::size_t size) {
  auto* header=static_cast<Header*>(std::malloc(sizeof(Header)+std::max(size,std::size_t(1))));
  if(!header)throw std::bad_alloc();
  header->size=size;header->tracked=enabled;
  if(enabled){++calls;bytes+=size;live+=size;peak=std::max(peak,live);}
  return header+1;
}
void release(void* memory) noexcept {
  if(!memory)return;
  auto* header=static_cast<Header*>(memory)-1;
  if(header->tracked)live-=header->size;
  std::free(header);
}
}
#ifndef D1MAX_PROBE_DISABLE_HEAP_MEASUREMENT
void* operator new(std::size_t n){return measurement::allocate(n);}
void* operator new[](std::size_t n){return measurement::allocate(n);}
void operator delete(void* p)noexcept{measurement::release(p);}
void operator delete[](void* p)noexcept{measurement::release(p);}
void operator delete(void* p,std::size_t)noexcept{measurement::release(p);}
void operator delete[](void* p,std::size_t)noexcept{measurement::release(p);}
#endif

struct GridMapTestAccess {
  static void configure(GridMap& map) {
    auto& p=map.mp_;auto& d=map.md_;
    p=MappingParameters{};d=MappingData{};
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={120,120,64};
    p.map_origin_idx_.setZero();map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.1;
    p.obstacles_inflation_z_up=.15;p.obstacles_inflation_z_down=.45;
    p.require_observed_free_=true;p.use_projected_rays_=true;p.cloud_pose_max_age_=.5;
    p.frame_id_="map";p.localization_session_id_="offline";
    const std::size_t count=120*120*64;
    d.occupancy_buffer_.assign(count,-1.);d.occupancy_buffer_inflate_.assign(count,0);
    d.occupancy_buffer_inflate_cnt_.assign(count,0);
    map.free_observation_stamps_.assign(count,100000000000);
    map.ray_tick_clock_ns_=map.ray_tick_effective_ns_=100100000000;
    map.ray_tick_receipt_=std::chrono::steady_clock::now();
    map.ray_query_clock_ns_=100100000000;map.integrated_cloud_stamp_ns_=100000000000;
    map.rebuildInflationOffsets();
    // Include measured occupied, insufficient and never-observed evidence.
    for(std::size_t i=0;i<count;++i) {
      if(i%137==0){d.occupancy_buffer_[i]=2.;d.occupancy_buffer_inflate_[i]=1;d.occupancy_buffer_inflate_cnt_[i]=3;}
      else if(i%139==0)d.occupancy_buffer_[i]=0.;
      else if(i%149==0)d.occupancy_buffer_[i]=-1.01;
    }
  }
  static std::uint64_t checksum(const GridMap& map) {
    std::uint64_t sum=0;
    for(std::size_t i=0;i<map.md_.occupancy_buffer_.size();++i)
      sum+=static_cast<std::uint64_t>(std::llround((map.md_.occupancy_buffer_[i]+2.)*100.))+
        17*static_cast<unsigned char>(map.md_.occupancy_buffer_inflate_[i])+
        static_cast<std::uint64_t>(map.free_observation_stamps_[i]);
    return sum;
  }
};
using Clock=std::chrono::steady_clock;
#ifdef D1MAX_PROBE_DISABLE_HEAP_MEASUREMENT
constexpr const char* heap_instrumented="false";
#else
constexpr const char* heap_instrumented="true";
#endif
static double us(Clock::duration elapsed){return std::chrono::duration<double,std::micro>(elapsed).count();}
static double percentile(std::vector<double> samples,double p){std::sort(samples.begin(),samples.end());return samples[static_cast<std::size_t>(p*(samples.size()-1))];}
static void cacheProbe(std::size_t capacity,unsigned rounds) {
  std::vector<double> timings;timings.reserve(rounds);
  std::uint64_t checksum=0,queries=0;
  measurement::reset();
  scan_planner::VoxelStatusCache cache(capacity);
  // Cold fill makes retained heap directly comparable. It is excluded from
  // hot wall time and hot allocation counters below.
  for(std::size_t i=0;i<capacity;++i)cache.getExact(i,-4,8,i*.0001,-static_cast<double>(i)*.0002,[&](){++queries;return static_cast<int>(i%4)-1;});
  const auto retained=measurement::live;
  const auto cold_calls=measurement::calls,cold_bytes=measurement::bytes;
  const auto begin_calls=measurement::calls,begin_bytes=measurement::bytes;
  for(unsigned round=0;round<rounds;++round) {
    const auto begin=Clock::now();cache.clear();
    for(std::size_t i=0;i<capacity;++i) {
      const double x=(i+round*.25)*.0001,y=-(i+round*.125)*.0002;
      auto query=[&](){++queries;return static_cast<int>((i+round)%4)-1;};
      checksum+=static_cast<std::uint64_t>(cache.getExact(i,-4,8,x,y,query)+1);
      checksum+=static_cast<std::uint64_t>(cache.getExact(i,-4,8,x,y,query)+1);
    }
    timings.push_back(us(Clock::now()-begin));
  }
  measurement::enabled=false;
  std::cout<<"{\"probe\":\"exact_cache\",\"heap_instrumented\":"<<heap_instrumented<<",\"capacity\":"<<capacity<<",\"rounds\":"<<rounds
    <<",\"misses\":"<<queries<<",\"checksum\":"<<checksum<<",\"retained_cpp_heap_bytes\":"<<retained
    <<",\"cold_allocation_calls\":"<<cold_calls<<",\"cold_allocation_bytes\":"<<cold_bytes
    <<",\"hot_allocation_calls\":"<<measurement::calls-begin_calls<<",\"hot_allocation_bytes\":"<<measurement::bytes-begin_bytes
    <<",\"round_p50_us\":"<<percentile(timings,.5)<<",\"round_p95_us\":"<<percentile(timings,.95)<<"}\n";
}
static void snapshotProbe(unsigned rounds) {
  GridMap writer;GridMapTestAccess::configure(writer);
  std::vector<double> timings;timings.reserve(rounds);
  measurement::reset();
  std::array<GridMap,3> slots;
  for(auto& slot:slots)writer.copyCollisionSnapshotTo(slot,100100000000,Clock::now());
  const auto retained=measurement::live,begin_calls=measurement::calls,begin_bytes=measurement::bytes;
  for(unsigned round=0;round<rounds;++round) {
    const auto begin=Clock::now();
    writer.copyCollisionSnapshotTo(slots[round%3],100100000000,begin);
    timings.push_back(us(Clock::now()-begin));
  }
  measurement::enabled=false;
  std::uint64_t checksum=0;std::size_t buffers=0;
  for(const auto& slot:slots){checksum+=GridMapTestAccess::checksum(slot);buffers+=slot.collisionSnapshotBytes();}
  rusage usage{};getrusage(RUSAGE_SELF,&usage);
  std::cout<<"{\"probe\":\"three_snapshot_slots\",\"heap_instrumented\":"<<heap_instrumented<<",\"voxels\":921600,\"rounds\":"<<rounds
    <<",\"checksum\":"<<checksum<<",\"three_slot_buffer_bytes\":"<<buffers
    <<",\"retained_cpp_heap_bytes\":"<<retained<<",\"hot_allocation_calls\":"<<measurement::calls-begin_calls
    <<",\"hot_allocation_bytes\":"<<measurement::bytes-begin_bytes<<",\"rss_high_water_kib\":"<<usage.ru_maxrss
    <<",\"copy_p50_us\":"<<percentile(timings,.5)<<",\"copy_p95_us\":"<<percentile(timings,.95)<<"}\n";
}
int main(int argc,char** argv) {
  std::cout<<std::fixed<<std::setprecision(3);
  const std::string mode=argc>1?argv[1]:"all";
  if(mode=="cache"||mode=="all"){cacheProbe(64,500);cacheProbe(1024,200);cacheProbe(32768,40);}
  if(mode=="snapshot"||mode=="all")snapshotProbe(120);
}
