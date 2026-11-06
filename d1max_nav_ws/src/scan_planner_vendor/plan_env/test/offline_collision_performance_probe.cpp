#include <plan_env/grid_map.h>
#include <plan_env/collision_snapshot_pool.hpp>
#include <plan_env/observed_ray.hpp>
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
  static void configure(GridMap& map,const Eigen::Vector3i& dimensions={120,120,64},
      bool require_observed_free=true) {
    auto& p=map.mp_;auto& d=map.md_;
    p=MappingParameters{};d=MappingData{};
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_=dimensions;
    p.map_origin_idx_.setZero();map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.1;
    p.obstacles_inflation_z_up=.15;p.obstacles_inflation_z_down=.45;
    p.require_observed_free_=require_observed_free;p.use_projected_rays_=true;p.cloud_pose_max_age_=.5;
    p.frame_id_="map";p.localization_session_id_="offline";
    const std::size_t count=static_cast<std::size_t>(dimensions.prod());
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
  static void advanceSource(GridMap& map,std::int64_t source) {
    map.integrated_cloud_stamp_ns_=source;map.ray_integrated_stamps_.fill(source);
    map.ray_tick_clock_ns_=map.ray_tick_effective_ns_=source+100000000LL;
    map.ray_tick_receipt_=std::chrono::steady_clock::now();++map.fusion_timing_.sequence;
  }
  static void configureFusion(GridMap& map) {
    configure(map,{240,240,60},false);
    auto& p=map.mp_;auto& d=map.md_;
    p.require_observed_free_=false;p.cloud_pose_max_age_=.6;
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    p.planning_obstacles_inflation_z_up=p.planning_obstacles_inflation_z_down=.25;
    p.local_update_range_={6.,6.,1.5};p.max_ray_length_=8.;p.ground_height_=-.8;
    p.map_sliding_en_=false;
    const auto probabilityLogOdds=[](double value){return std::log(value/(1.-value));};
    p.prob_hit_log_=probabilityLogOdds(.85);p.prob_miss_log_=probabilityLogOdds(.3);
    p.clamp_min_log_=probabilityLogOdds(.12);p.clamp_max_log_=probabilityLogOdds(.98);
    p.min_occupancy_log_=probabilityLogOdds(.8);
    const auto size=static_cast<std::size_t>(p.map_voxel_num_.prod());
    d.count_hit_.assign(size,0);d.count_hit_and_miss_.assign(size,0);
    d.flag_traverse_.assign(size,-1);d.flag_rayend_.assign(size,-1);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static void stageFusion(GridMap& map,std::int64_t source,unsigned round,unsigned points) {
    // Deterministic SYNTHETIC dual-LiDAR corridor, not bag replay. 120k rays
    // approximates the logged point count; per-point origins differ slightly.
    const auto now=std::chrono::steady_clock::now();
    for(unsigned sensor=0;sensor<2;++sensor) {
      scan_planner::ProjectedRayBatch batch;batch.stamp_ns=source;batch.sensor_id=sensor;
      batch.origins.reserve(points/2);batch.endpoints.reserve(points/2);
      for(unsigned i=0;i<points/2;++i) {
        const double x=-5.8+11.6*static_cast<double>((i*73+round*17)%60000)/59999.;
        const double y=-1.45+2.9*static_cast<double>((i*131+round*7)%60000)/59999.;
        const double z=-.45+1.85*static_cast<double>((i*251+round*3)%60000)/59999.;
        batch.origins.emplace_back(sensor?.375:-.375,0.,.001*static_cast<double>(i%11));
        batch.endpoints.emplace_back(x,i%3==0?y:(i%2?1.5:-1.5),i%3==0?-.5:z);
      }
      map.pending_rays_[sensor]=GridMap::PendingRays{std::move(batch),now,true};
    }
  }
  static void fuse(GridMap& map,std::int64_t source) {
    map.processProjectedRays(source+200000000LL,std::chrono::steady_clock::now());
    if(map.integratedRaySourceStamp(0)!=source||map.integratedRaySourceStamp(1)!=source)
      throw std::runtime_error("synthetic fusion incomplete");
  }
  static void inflationBatch(GridMap& map,bool legacy,unsigned round) {
    // Identical occupied-state changes for both implementations; no changed
    // geometry or ray selection. Alternate insert/remove to exercise counts,
    // clipping and saturation rather than timing a no-op saturated map.
    const auto& offsets=map.md_.inflate_offsets_;
    const scan_planner::ObservedRayMapIndex index(map.mp_.map_bound_min_idx_,map.mp_.map_voxel_num_);
    const int delta=round%2?-1:1;
    for(int i=0;i<4000;++i) {
      const Eigen::Vector3i cell((i*73)%241-120,(i*131)%241-120,(i*29)%61-30);
      if(!legacy){map.updateInflation(cell,delta,nullptr);continue;}
      for(const auto& offset:offsets) {
        const Eigen::Vector3i target=cell+offset;
        if(!map.isInMap(target))continue;
        const int address=index.address(target);
        auto& count=map.md_.occupancy_buffer_inflate_cnt_[address];
        count+=delta;if(count<0)count=0;
        map.md_.occupancy_buffer_inflate_[address]=count>0?1:0;
      }
    }
  }
  static bool sameInflation(const GridMap& a,const GridMap& b) {
    return a.md_.occupancy_buffer_inflate_cnt_==b.md_.occupancy_buffer_inflate_cnt_&&
      a.md_.occupancy_buffer_inflate_==b.md_.occupancy_buffer_inflate_&&
      a.md_.planning_occupancy_buffer_inflate_cnt_==b.md_.planning_occupancy_buffer_inflate_cnt_&&
      a.md_.planning_occupancy_buffer_inflate_==b.md_.planning_occupancy_buffer_inflate_&&
      a.md_.occupancy_buffer_==b.md_.occupancy_buffer_&&
      a.free_observation_stamps_==b.free_observation_stamps_;
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
static void ownerReplicaProbe(unsigned rounds) {
  // Same 12 x 12 x 3 m, 5 cm nominal voxel count as the Oct20 session.
  // Synthetic occupied/insufficient/unobserved mix above, not bag replay.
  GridMap writer;GridMapTestAccess::configure(writer,{240,240,60});
  GridMapTestAccess::advanceSource(writer,100000000000LL);
  scan_planner::CollisionSnapshotPool pool;
  pool.publishValidation(writer,100100000000LL,Clock::now());
  pool.refreshOwnerReplica(writer,100100000000LL,Clock::now());
  pool.publishSolver(writer,100100000000LL,Clock::now());
  auto solver=pool.borrowSolver(); // Third slot remains pinned throughout.
  std::vector<double> heads,replicas,noops;heads.reserve(rounds);replicas.reserve(rounds);noops.reserve(rounds);
  measurement::reset();
  const auto allocations=measurement::calls,allocated=measurement::bytes;
  for(unsigned round=0;round<rounds;++round) {
    const auto source=100100000000LL+static_cast<std::int64_t>(round)*100000000LL;
    GridMapTestAccess::advanceSource(writer,source);
    auto old=pool.borrowOwnerValidation();if(!old)throw std::runtime_error("owner unavailable");
    auto begin=Clock::now();
    if(!pool.publishValidation(writer,source+100000000LL,begin))throw std::runtime_error("head unavailable");
    heads.push_back(us(Clock::now()-begin));
    if(pool.refreshOwnerReplica(writer,source+100000000LL,Clock::now()))
      throw std::runtime_error("overwrote borrowed owner");
    old.reset();begin=Clock::now();
    if(!pool.refreshOwnerReplica(writer,source+100000000LL,begin))throw std::runtime_error("replica not completed");
    replicas.push_back(us(Clock::now()-begin));begin=Clock::now();
    for(unsigned retry=0;retry<100;++retry)
      if(pool.refreshOwnerReplica(writer,source+100000000LL,Clock::now()))
        throw std::runtime_error("identical retry recopied");
    noops.push_back(us(Clock::now()-begin)/100.);
  }
  measurement::enabled=false;
  auto owner=pool.borrowOwnerValidation();auto fast=pool.borrowValidation();
  if(!owner||!fast||owner.get()==fast.get()||owner.get()==solver.get()||fast.get()==solver.get())
    throw std::runtime_error("slot ownership violated");
  if(GridMapTestAccess::checksum(*owner)!=GridMapTestAccess::checksum(*fast))
    throw std::runtime_error("replica geometry differs");
  std::cout<<"{\"probe\":\"owner_replica_catchup\",\"synthetic\":true,\"heap_instrumented\":"<<heap_instrumented
    <<",\"voxels\":3456000,\"rounds\":"<<rounds
    <<",\"slot_buffer_bytes\":"<<owner->collisionSnapshotBytes()<<",\"allocated_slots\":3"
    <<",\"head_p50_us\":"<<percentile(heads,.5)<<",\"head_p95_us\":"<<percentile(heads,.95)
    <<",\"replica_p50_us\":"<<percentile(replicas,.5)<<",\"replica_p95_us\":"<<percentile(replicas,.95)
    <<",\"replica_max_us\":"<<*std::max_element(replicas.begin(),replicas.end())
    <<",\"noop_p50_us\":"<<percentile(noops,.5)<<",\"noop_p95_us\":"<<percentile(noops,.95)
    <<",\"hot_allocation_calls\":"<<measurement::calls-allocations
    <<",\"hot_allocation_bytes\":"<<measurement::bytes-allocated<<"}\n";
}
static void fusionSnapshotProbe(unsigned rounds,bool replica) {
  GridMap writer;GridMapTestAccess::configureFusion(writer);
  scan_planner::CollisionSnapshotPool pool;
  std::vector<double> fusion,copy,total,initialize,traverse,occupancy,steps;
  std::uint64_t source=100000000000LL;
  double copy_us=0.;
  writer.setCollisionUpdateCallback([&] {
    const auto begin=Clock::now();
    if(!pool.publishValidation(writer,source+200000000LL,Clock::now()))
      throw std::runtime_error("synthetic head unavailable");
    if(replica&&!pool.refreshOwnerReplica(writer,source+200000000LL,Clock::now()))
      throw std::runtime_error("synthetic replica unavailable");
    if(!pool.publishSolver(writer,source+200000000LL,Clock::now()))
      throw std::runtime_error("synthetic solver unavailable");
    copy_us=us(Clock::now()-begin);
  });
  for(unsigned round=0;round<rounds+2;++round) {
    source+=100000000LL;
    GridMapTestAccess::stageFusion(writer,source,round,120000);
    const auto begin=Clock::now();
    GridMapTestAccess::fuse(writer,source);
    const auto end=Clock::now();
    if(round>=2) {
      fusion.push_back(us(end-begin)-copy_us);copy.push_back(copy_us);total.push_back(us(end-begin));
      const auto work=writer.mapWorkTiming();
      initialize.push_back(work.initialize_ms*1000.);traverse.push_back(work.traverse_ms*1000.);
      occupancy.push_back(work.occupancy_ms*1000.);steps.push_back(work.ray_steps);
    }
  }
  const auto checked=pool.borrowValidation();
  std::cout<<"{\"probe\":\"native_fusion_plus_snapshots\",\"synthetic\":true"
    <<",\"sliding\":false,\"transport\":false,\"concurrent_readers\":false"
    <<",\"voxels\":3456000,\"rays\":120000,\"rounds\":"<<rounds
    <<",\"copies_per_fusion\":"<<(replica?3:2)
    <<",\"fusion_p50_us\":"<<percentile(fusion,.5)<<",\"fusion_p95_us\":"<<percentile(fusion,.95)
    <<",\"initialize_p50_us\":"<<percentile(initialize,.5)<<",\"initialize_p95_us\":"<<percentile(initialize,.95)
    <<",\"traverse_p50_us\":"<<percentile(traverse,.5)<<",\"traverse_p95_us\":"<<percentile(traverse,.95)
    <<",\"occupancy_p50_us\":"<<percentile(occupancy,.5)<<",\"occupancy_p95_us\":"<<percentile(occupancy,.95)
    <<",\"ray_steps_p50\":"<<percentile(steps,.5)
    <<",\"copy_p50_us\":"<<percentile(copy,.5)<<",\"copy_p95_us\":"<<percentile(copy,.95)
    <<",\"total_p50_us\":"<<percentile(total,.5)<<",\"total_p95_us\":"<<percentile(total,.95)
    <<",\"total_max_us\":"<<*std::max_element(total.begin(),total.end())
    <<",\"checksum\":"<<GridMapTestAccess::checksum(*checked)<<"}\n";
}
static void inflationColumnProbe(unsigned rounds) {
  GridMap legacy,optimized;GridMapTestAccess::configureFusion(legacy);
  GridMapTestAccess::configureFusion(optimized);
  std::vector<double> old_times,new_times;
  for(unsigned round=0;round<rounds+2;++round) {
    // Alternate measurement order to avoid consistently favouring warmed
    // CPU/cache conditions. Only original production updates are measured.
    const auto run=[&](bool old) {
      const auto begin=Clock::now();
      GridMapTestAccess::inflationBatch(old?legacy:optimized,old,round);
      if(round>=2)(old?old_times:new_times).push_back(us(Clock::now()-begin));
    };
    if(round%2){run(true);run(false);}else{run(false);run(true);}
    if(!GridMapTestAccess::sameInflation(legacy,optimized))
      throw std::runtime_error("inflation counters, flags or source evidence changed");
  }
  std::cout<<"{\"probe\":\"native_inflation_columns\",\"synthetic\":true,\"rounds\":"<<rounds
    <<",\"updates_per_round\":4000,\"voxels\":3456000"
    <<",\"legacy_p50_us\":"<<percentile(old_times,.5)<<",\"legacy_p95_us\":"<<percentile(old_times,.95)
    <<",\"columns_p50_us\":"<<percentile(new_times,.5)<<",\"columns_p95_us\":"<<percentile(new_times,.95)
    <<",\"exact_counters_flags_raw_sources_equal\":true"
    <<",\"checksum\":"<<GridMapTestAccess::checksum(optimized)<<"}\n";
}
int main(int argc,char** argv) {
  std::cout<<std::fixed<<std::setprecision(3);
  const std::string mode=argc>1?argv[1]:"all";
  if(mode=="cache"||mode=="all"){cacheProbe(64,500);cacheProbe(1024,200);cacheProbe(32768,40);}
  if(mode=="snapshot"||mode=="all")snapshotProbe(120);
  if(mode=="owner-replica")ownerReplicaProbe(120);
  if(mode=="fusion-snapshots"){fusionSnapshotProbe(30,false);fusionSnapshotProbe(30,true);}
  if(mode=="inflation-columns")inflationColumnProbe(30);
}
