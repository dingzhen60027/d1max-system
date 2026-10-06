#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "plan_env/grid_map.h"
#include "plan_env/observed_ray.hpp"
#include "voxel_cache_test_access.hpp"
#include <numeric>
#include <set>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <unistd.h>

// Actual map integration, memory only: no ROS context/node/executor/transport.
struct GridMapTestAccess {
  static void configure(GridMap &map,bool require_observed_free=true) {
    auto &p=map.mp_;auto &d=map.md_;
    p.use_projected_rays_=p.require_localization_context_=true;
    p.require_observed_free_=require_observed_free;
    p.localization_session_id_="session";p.sensor_type_="lidar";p.frame_id_="map";
    p.map_sliding_en_=false;p.local_update_range_={1.9,1.9,1.9};
    p.resolution_=.1;p.resolution_inv_=10.;p.map_voxel_num_={40,40,40};
    p.map_origin_idx_.setZero();map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.prob_hit_log_=3.;p.prob_miss_log_=-.5;p.max_ray_length_=4.;
    p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.1;p.obstacles_inflation_z_down=.1;
    p.ground_height_=0.;p.cloud_pose_max_age_=.5;p.cloud_pose_pair_wait_=.25;
    d.has_ray_pose_=false;d.ray_pos_.setZero();d.raycast_num_=0;
    const std::size_t size=64000;
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);
    d.count_hit_and_miss_.resize(size);d.flag_rayend_.resize(size);d.flag_traverse_.resize(size);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static bool accept(GridMap &map,const d1max_planning_interfaces::msg::ProjectedRays &msg,
      std::int64_t now=102000000000LL,double receipt=102.) {
    return map.acceptProjectedRays(msg,now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
        static_cast<std::int64_t>(receipt*1e9))));
  }
  static void integrate(GridMap &map,std::int64_t now=102000000000LL,double receipt=102.) {
    map.processProjectedRays(now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
        static_cast<std::int64_t>(receipt*1e9))));
  }
  static bool eventFusion(GridMap& map,std::int64_t now,bool watchdog=false) {
    return map.tryProjectedFusion(now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(now)),watchdog);
  }
  static auto nextWake(GridMap& map,std::int64_t now) {
    return map.projectedFusionWakeDelay(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(now))).count();
  }
  static double raw(GridMap &map,double x,double y=.05,double z=.05) {
    Eigen::Vector3i cell;map.posToIndex({x,y,z},cell);return map.md_.occupancy_buffer_[map.toAddress(cell)];
  }
  static auto counts(GridMap &map) {return map.ray_integrations_;}
  static auto drops(GridMap &map) {return map.ray_drops_;}
  static auto pending(GridMap &map) {return !!map.pending_rays_[0]+!!map.pending_rays_[1];}
  static auto drain(GridMap &map,const std::vector<d1max_planning_interfaces::msg::ProjectedRays> &queue,
      std::size_t &cursor,std::int64_t now=102000000000LL,double receipt=102.) {
    return scan_planner::drainProjectedRayIngress<d1max_planning_interfaces::msg::ProjectedRays>(
        [&queue,&cursor](auto &message) {
          if(cursor==queue.size()) return false;
          message=queue[cursor++];return true;
        },[&map,now,receipt](const auto &message) {return accept(map,message,now,receipt);});
  }
  static auto acceptedClock(const GridMap &map) {return map.ray_accept_clock_ns_;}
  static auto integrationStart(const GridMap &map) {return map.ray_integration_start_ns_;}
  static auto integratedPoints(const GridMap &map) {return map.md_.proj_points_cnt;}
  static void legacy(GridMap &map) {map.mp_.use_projected_rays_=false;}
  static void previewTiming(GridMap &map,double age) {
    auto &p=map.mp_;p.preview_only_=true;p.cloud_pose_max_age_=age;
    scan_planner::validateCloudPoseTiming(p.cloud_pose_pair_wait_,age,
        p.preview_only_,p.require_observed_free_,p.use_projected_rays_);
  }
  static void diagnostics(GridMap &map) {
    map.nearFieldDiagnostics().configure(true,Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(5.),.1);
  }
  static auto buffers(const GridMap &map) {return map.md_.occupancy_buffer_;}
  static void expectSameRayEvidence(const GridMap &a,const GridMap &b) {
    EXPECT_EQ(a.md_.occupancy_buffer_,b.md_.occupancy_buffer_);
    // Inflation intentionally follows a different query geometry in each policy.
    EXPECT_EQ(a.free_observation_stamps_,b.free_observation_stamps_);
    EXPECT_EQ(a.ray_integrations_,b.ray_integrations_);
    EXPECT_EQ(a.ray_integrated_stamps_,b.ray_integrated_stamps_);
    EXPECT_EQ(a.integrated_cloud_stamp_ns_,b.integrated_cloud_stamp_ns_);
  }
  static void pointQueryGeometry(GridMap &map) {
    map.mp_.double_cylinder_radius_=map.mp_.double_cylinder_offset_=0.;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=0.;
    map.rebuildInflationOffsets();
  }
  static void centerOnlyOfficialGeometry(GridMap &map) {
    map.mp_.double_cylinder_radius_=.01;map.mp_.double_cylinder_offset_=0.;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=0.;
    map.rebuildInflationOffsets();
  }
  static auto freeStamp(GridMap &map,const Eigen::Vector3i &cell) {
    return map.free_observation_stamps_.at(map.toAddress(cell));
  }
  static void simulationCollisionClock(GridMap &map) {map.mp_.simulation_collision_clock_=true;}
  static auto freeReceipt(GridMap &map,const Eigen::Vector3i &cell) {
    return map.free_observation_receipts_ns_.at(map.toAddress(cell));
  }
  static void queryTime(GridMap &map,std::int64_t stamp) {map.ray_query_clock_ns_=stamp;}
  static auto steadyClockAdvance(GridMap &map,std::int64_t source,std::int64_t receipt) {
    const auto at=std::chrono::steady_clock::time_point(std::chrono::nanoseconds(receipt));
    map.advanceRayEvidenceClock(source,at,true);
    map.ray_query_clock_ns_=map.collisionQueryClockAt(source,at);
    return map.ray_query_clock_ns_;
  }
  static void expectSameEvidence(const GridMap &a,const GridMap &b) {
    EXPECT_EQ(a.md_.occupancy_buffer_,b.md_.occupancy_buffer_);
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_,b.md_.occupancy_buffer_inflate_);
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_cnt_,b.md_.occupancy_buffer_inflate_cnt_);
    EXPECT_EQ(a.md_.count_hit_,b.md_.count_hit_);
    EXPECT_EQ(a.md_.count_hit_and_miss_,b.md_.count_hit_and_miss_);
    EXPECT_EQ(a.md_.flag_rayend_,b.md_.flag_rayend_);
    EXPECT_EQ(a.md_.flag_traverse_,b.md_.flag_traverse_);
    EXPECT_EQ(a.ray_integrations_,b.ray_integrations_);
    EXPECT_EQ(a.ray_integrated_stamps_,b.ray_integrated_stamps_);
    EXPECT_EQ(a.occupancyRevision(),b.occupancyRevision());
  }
  static void productionGeometry(GridMap &map) {
    // Exact current cylinder parameters; test map resolution remains .1 m.
    map.mp_.double_cylinder_radius_=.29;map.mp_.double_cylinder_offset_=.2;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=.45;
    map.rebuildInflationOffsets();
  }
  static void productionProbabilities(GridMap &map) {
    const auto odds=[](double p){return std::log(p/(1.-p));};
    map.mp_.clamp_min_log_=odds(.12);map.mp_.clamp_max_log_=odds(.98);
    map.mp_.min_occupancy_log_=odds(.8);map.mp_.prob_hit_log_=odds(.85);
    map.mp_.prob_miss_log_=odds(.3);map.resetAllMapData();
  }
  static Eigen::Vector3i index(GridMap &map,const Eigen::Vector3d &position) {
    Eigen::Vector3i result;map.posToIndex(position,result);return result;
  }
  static int address(GridMap &map,const Eigen::Vector3i &cell) {return map.toAddress(cell);}
  static void indexedUpdate(GridMap &map,const Eigen::Vector3i &cell,double value) {
    ASSERT_TRUE(map.isInMap(cell));
    const scan_planner::ObservedRayMapIndex index(map.mp_.map_bound_min_idx_,map.mp_.map_voxel_num_);
    map.applyOccupancyUpdateAtIndex(cell,index.address(cell),value);
  }
  // Frozen pre-optimization log-odds update: same map/buffers, no alternate
  // map model. Compare every state transition and inflation/reference count.
  static void legacyUpdate(GridMap &map,const Eigen::Vector3i &id,double value) {
    if(!map.isInMap(id)) return;
    const int addr=map.toAddress(id);
    const bool was_occ=map.md_.occupancy_buffer_[addr]>map.mp_.min_occupancy_log_;
    const bool now_occ=value>map.mp_.min_occupancy_log_;
    const bool was_known=map.md_.occupancy_buffer_[addr]>=map.mp_.clamp_min_log_;
    const bool now_known=value>=map.mp_.clamp_min_log_;
    const bool free_changed=scan_planner::strictRawVoxelStatus(map.md_.occupancy_buffer_[addr],
        map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_) !=
        scan_planner::strictRawVoxelStatus(value,map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_);
    if(was_known!=now_known || was_occ!=now_occ || (map.mp_.require_observed_free_ && free_changed)) {
      map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
    }
    map.md_.occupancy_buffer_[addr]=value;
    if(was_occ!=now_occ) map.updateInflation(id,now_occ?1:-1);
  }
  static scan_planner::RawVoxelDiagnostic category(GridMap &map,const Eigen::Vector3i &cell) {
    if(!map.isInMap(cell)) return scan_planner::RawVoxelDiagnostic::Outside;
    return scan_planner::diagnoseRawVoxel(map.md_.occupancy_buffer_[map.toAddress(cell)],
        map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_,map.mp_.unknown_flag_);
  }
  static void slide(GridMap &map,const Eigen::Vector3d &center) {
    map.mp_.map_sliding_en_=true;map.mp_.map_sliding_thresh_vox_=1;
    map.updateSlidingMap(center);
    map.mp_.map_sliding_en_=false; // Further packet integration holds this test window.
  }
  static std::string attachStaticPrior(GridMap &map,
      const std::vector<std::pair<Eigen::Vector3i,std::uint8_t>> &overrides={}) {
    static unsigned sequence=0;
    const auto directory=std::filesystem::temp_directory_path()/
        ("d1max-grid-prior-"+std::to_string(getpid())+"-"+std::to_string(++sequence));
    std::filesystem::create_directory(directory);
    const std::array<int,3> origin{{-20,-20,-20}},shape{{80,40,40}};
    std::vector<std::uint8_t> data(80*40*40,0);
    for(const auto &item:overrides) {
      const auto &cell=item.first;
      data.at(((cell.x()-origin[0])*shape[1]+cell.y()-origin[1])*shape[2]+cell.z()-origin[2])=item.second;
    }
    const auto hash=scan_planner::StaticOccupancyPrior::sha256(data.data(),data.size());
    const std::string geometry(64,'b');
    nlohmann::json manifest{{"schema",1},{"kind","certified_static_occupancy_prior"},
      {"provenance","isaac_closed_collision_geometry_v1"},{"frame_id","map"},{"map_version","map-version"},
      {"voxel_resolution",.1},{"origin_index",origin},{"shape",shape},{"storage_order","C_xyz_z_fastest"},
      {"state_codes",{{"free",0},{"occupied",1},{"unknown",2}}},{"dtype","uint8"},{"meters_per_unit",1.},{"up_axis","Z"},
      {"data_file","volume.bin"},{"data_sha256",hash},{"data_size_bytes",data.size()},
      {"collider_sha256",geometry},{"scene_sha256",std::string(64,'a')},{"spec_sha256",std::string(64,'c')},
      {"geometry_margin_m",0.},{"closed_world_bounds",{{"min",{-5.,-5.,-5.}},{"max",{10.,5.,5.}},
        {"semantics","whole_closed_cell_strictly_inside"}}},
      {"map_from_odom",{{"transform_contract","fixed_identity_map_from_odom_v1"},{"from_frame","map"},{"to_frame","map"},
        {"translation",{0.,0.,0.}},{"rotation_xyzw",{0.,0.,0.,1.}}}}};
    const auto text=manifest.dump();const auto manifest_hash=scan_planner::StaticOccupancyPrior::sha256(text.data(),text.size());
    {std::ofstream file(directory/"volume.bin",std::ios::binary);file.write(reinterpret_cast<const char*>(data.data()),data.size());}
    {std::ofstream file(directory/"volume.json");file<<text;}
    scan_planner::StaticOccupancyPrior::Expected expected;
    expected.manifest_sha256=manifest_hash;expected.geometry_sha256=geometry;expected.map_version="map-version";
    expected.frame_id=expected.odom_frame_id="map";expected.resolution=.1;
    map.static_prior_=scan_planner::StaticOccupancyPrior::load((directory/"volume.json").string(),expected);
    map.mp_.validated_static_prior_=true;map.mp_.simulation_collision_clock_=true;
    std::filesystem::remove_all(directory);
    return manifest_hash;
  }
  static int status(GridMap &map,const Eigen::Vector3i &cell) {
    map.beginCollisionQuery();return map.rawCollisionStatus(cell);
  }
  static int uncachedStatus(GridMap &map,const Eigen::Vector3i &cell) {
    map.beginCollisionQuery();return map.uncachedRawCollisionStatus(cell);
  }
  static std::size_t collisionCacheSize(const GridMap &map) {
    return map.observed_cylinder_cache_.size()+
        (map.raw_collision_cache_generation_==map.observed_cylinder_cache_.generationIdentifier()&&
         map.observed_cylinder_cache_.generationReusable()?std::count_if(map.raw_collision_cache_.begin(),map.raw_collision_cache_.end(),
           [](std::int8_t state){return state!=GridMap::uncached_raw_status_;}):0);
  }
  static std::size_t rawCacheStorage(const GridMap &map) {return map.raw_collision_cache_.size();}
  static std::size_t rawCacheCapacity(const GridMap &map) {return map.raw_collision_cache_.capacity();}
  static const void *rawCacheData(const GridMap &map) {return map.raw_collision_cache_.data();}
  static auto &cylinderCache(GridMap &map) {return map.observed_cylinder_cache_;}
  static std::size_t snapshotBytesWithoutRawCache(GridMap &map) {
    const auto prior=map.mp_.validated_static_prior_;map.mp_.validated_static_prior_=false;
    const auto bytes=map.collisionSnapshotBytes();map.mp_.validated_static_prior_=prior;return bytes;
  }
  static void beginIndependentMemoryProof(GridMap &map) {
    // The memory-only fixture is exclusively owned and has no ROS node. Use
    // the production proof-scope reset without replacing deterministic clocks
    // with wall time or duplicating its cache/deadline reset implementation.
    const auto snapshot=map.collision_snapshot_;map.collision_snapshot_=true;
    map.beginObservedProof();map.collision_snapshot_=snapshot;
  }
  static bool conflict(GridMap &map,const Eigen::Vector3i &cell) {
    return map.static_prior_live_hits_.count({cell.x(),cell.y(),cell.z()});
  }
  static bool priorContextValid(const GridMap &map) {return map.static_prior_context_valid_;}
  static bool cachedPriorLease(const GridMap &map) {return map.static_prior_query_lease_valid_;}
  static void freeWitness(GridMap &map,const Eigen::Vector3i &cell,std::int64_t source,std::int64_t receipt) {
    const auto address=map.toAddress(cell);map.free_observation_stamps_.at(address)=source;
    map.free_observation_receipts_ns_.at(address)=receipt;map.observed_cylinder_cache_.clear();
  }
};
static builtin_interfaces::msg::Time timeAt(std::int64_t ns) {
  builtin_interfaces::msg::Time out;out.sec=ns/1000000000LL;out.nanosec=ns%1000000000LL;return out;
}
static std::string context(unsigned epoch=1,unsigned sequence=1,const std::string &seed="seed") {
  return nlohmann::json{{"schema",1},{"session_id","session"},{"epoch",epoch},{"sequence",sequence},
    {"seed_id",seed},{"barrier_ns",100000000000ULL+sequence}}.dump();
}
static std::string priorContext(const std::string &hash,unsigned epoch=1,unsigned sequence=1) {
  auto value=nlohmann::json::parse(context(epoch,sequence));
  value["map_version"]="map-version";value["static_prior_manifest_sha256"]=hash;
  value["map_from_odom_contract"]="fixed_identity_map_from_odom_v1";
  value["map_from_odom_translation"]={0.,0.,0.};value["map_from_odom_rotation_xyzw"]={0.,0.,0.,1.};
  return value.dump();
}
static d1max_planning_interfaces::msg::ProjectedRays packet(std::uint16_t sensor=0,
    std::int64_t stamp=101900000000LL,std::uint64_t sequence=1,unsigned count=1,
    Eigen::Vector3d origin={.55,.05,.05},Eigen::Vector3d end={1.55,.05,.05}) {
  d1max_planning_interfaces::msg::ProjectedRays out;
  out.session_id="session";out.epoch=1;out.context_sequence=1;out.seed_id="seed";
  out.barrier_ns=100000000001ULL;out.projection_sequence=sequence;
  out.acquisition_end=timeAt(stamp+50000000);out.alignment_stamp=timeAt(stamp);
  auto &c=out.rays;c.header.frame_id="map";c.header.stamp=timeAt(stamp);
  c.width=count;c.height=1;c.point_step=64;c.row_step=count*64;c.data.resize(c.row_step);
  using F=sensor_msgs::msg::PointField;
  const std::pair<const char*,unsigned> fields[]={{"x",0},{"y",4},{"z",8},
    {"origin_x",16},{"origin_y",20},{"origin_z",24},{"sensor_id",28}};
  for (auto field:fields) {
    F f;f.name=field.first;f.offset=field.second;f.count=1;f.datatype=f.offset==28?F::UINT16:F::FLOAT32;
    c.fields.push_back(f);
  }
  const float xyz[]={float(end.x()),float(end.y()),float(end.z())};
  const float start[]={float(origin.x()),float(origin.y()),float(origin.z())};
  for(unsigned i=0;i<count;++i) {
    auto *bytes=c.data.data()+64*i;std::memcpy(bytes,xyz,12);std::memcpy(bytes+16,start,12);
    std::memcpy(bytes+28,&sensor,2);
  }
  return out;
}

TEST(StaticPriorIntegration, OptInFillsOnlyCertifiedNeverObservedWithoutManufacturingSourceStamp) {
  GridMap legacy,map;GridMapTestAccess::configure(legacy);GridMapTestAccess::configure(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map,{{{2,0,0},2}});
  ASSERT_TRUE(legacy.applyLocalizationContext(context()));ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor) {
    ASSERT_TRUE(GridMapTestAccess::accept(legacy,packet(sensor)));ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  }
  GridMapTestAccess::integrate(legacy);GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::status(legacy,{0,0,0}),2);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{0,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{0,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::status(map,{2,0,0}),2);
  GridMapTestAccess::indexedUpdate(map,{1,0,0},-.5); // Measured-but-insufficient is not certified away.
  EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
  const auto diagnostic=map.describeInflateOccupancy({.05,.05,.05},0.);
  EXPECT_NE(diagnostic.find("certified_static_free="),std::string::npos);
}

TEST(StaticPriorIntegration, StaticThinWallOccupiedWinsEvenOverActualFreshFreeRay) {
  GridMap map;GridMapTestAccess::configure(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map,{{{10,0,0},1}});
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::category(map,{10,0,0}),scan_planner::RawVoxelDiagnostic::Free);
  EXPECT_GT(GridMapTestAccess::freeStamp(map,{10,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::status(map,{10,0,0}),1);
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(GridMapTestAccess::status(map,{10,0,0}),1); // No occupied-to-free TTL conversion.
}

TEST(StaticPriorIntegration, ExpiredRealFreeMayUsePriorWithoutRenewalButInvalidWitnessCannot) {
  GridMap map;GridMapTestAccess::configure(map);const auto hash=GridMapTestAccess::attachStaticPrior(map);
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,101900000000LL,1,1,{-.55,.05,.05},{.35,.05,.05})));
  GridMapTestAccess::integrate(map);
  const auto original_source=GridMapTestAccess::freeStamp(map,{0,0,0});
  const auto original_receipt=GridMapTestAccess::freeReceipt(map,{0,0,0});
  ASSERT_GT(original_source,0);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102500000000LL,2,1,{1.05,.05,.05},{1.55,.05,.05}),102600000000LL,102.6));
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{0,0,0}),original_source);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{0,0,0}),original_receipt);
  EXPECT_EQ(map.observedProofDeadlineNs(),103000000000LL); // Bound by fresh both-ray witnesses, never prior infinity.
  for(const auto bad:std::vector<std::pair<std::int64_t,std::int64_t>>{
      {0,original_receipt},{103000000000LL,original_receipt},{original_source,0},{original_source,103000000000LL}}) {
    GridMapTestAccess::freeWitness(map,{0,0,0},bad.first,bad.second);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2); // Zero/future is invalid, not expired FREE.
  }
}

TEST(StaticPriorIntegration, FirstDynamicHitAndSameBatchHitMissConflictImmediatelyBlockPriorFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionProbabilities(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,1,1,{.05,.05,.05},{.35,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::Insufficient);
  EXPECT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
  EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
  // An unrelated fresh acquisition and elapsed TTL do not clear a real hit.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102500000000LL,2,1,{1.05,.05,.05},{1.55,.05,.05}),102600000000LL,102.6));
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
}

TEST(StaticPriorIntegration, ConflictClearsOnlyAfterNewCompleteRealMissReachesStrictFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionProbabilities(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  for(unsigned round=0;round<3;++round) {
    const auto source=102000000000LL+round*100000000LL;const double now=102.1+round*.1;
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
        packet(sensor,source,round+2,1,{.05,.05,.05},{.35,.05,.05}),source+100000000LL,now));
    GridMapTestAccess::integrate(map,source+100000000LL,now);
    if(round<2) {
      EXPECT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
    }
  }
  EXPECT_FALSE(GridMapTestAccess::conflict(map,{1,0,0}));EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),0);
  EXPECT_GT(GridMapTestAccess::freeStamp(map,{1,0,0}),101900000000LL);
}

TEST(StaticPriorIntegration, AsynchronousMissCannotBorrowOldCrossSensorFreeWitnessToClearConflict) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionProbabilities(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.35,.05,.05})));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101800000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);ASSERT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
  const auto conflicted_free_stamp=GridMapTestAccess::freeStamp(map,{1,0,0});
  ASSERT_EQ(conflicted_free_stamp,101900000000LL);
  for(unsigned round=0;round<3;++round) {
    // Monotonic at the rear source, but still older than the front witness in
    // the original conflicted batch. Actual new rear misses lower native odds.
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101850000000LL+round*10000000LL,
        round+2,1,{.05,.05,.05},{.35,.05,.05}),102000000000LL,102.+round*.01));
    GridMapTestAccess::integrate(map,102000000000LL,102.+round*.01);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{1,0,0}),conflicted_free_stamp);
    EXPECT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
    EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
  }
  EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::Free);
  // A newer actual front miss may now clear the conflict using its OWN source
  // and original callback receipt, after the complete native batch commits.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102000000000LL,2,1,{.05,.05,.05},{.35,.05,.05}),
      102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_FALSE(GridMapTestAccess::conflict(map,{1,0,0}));EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),0);
}

TEST(StaticPriorIntegration, HitContradictionSurvivesFullWindowSlideAndRingSlotReuse) {
  GridMap map;GridMapTestAccess::configure(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);ASSERT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
  GridMapTestAccess::slide(map,{4.,0.,0.});
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{5.05,.05,.05},{5.55,.05,.05}),102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_EQ(GridMapTestAccess::status(map,{41,0,0}),0); // Different world cell in old ring slot.
  GridMapTestAccess::slide(map,{0.,0.,0.});
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102100000000LL,3,1,{1.05,.05,.05},{1.55,.05,.05}),102200000000LL,102.2));
  GridMapTestAccess::integrate(map,102200000000LL,102.2);
  EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  EXPECT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
}

TEST(StaticPriorIntegration, MismatchedContextRevokesPreviousPriorAndAllIntegratedEvidence) {
  GridMap map;GridMapTestAccess::configure(map);const auto hash=GridMapTestAccess::attachStaticPrior(map);
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  auto bad=nlohmann::json::parse(priorContext(hash,2,2));bad["map_version"]="different-map";
  EXPECT_FALSE(map.applyLocalizationContext(bad.dump()));EXPECT_FALSE(GridMapTestAccess::priorContextValid(map));
  EXPECT_EQ(map.latestCloudStampNs(),0);EXPECT_EQ(map.integratedRaySourceStamp(0),0);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
  bad=nlohmann::json::parse(priorContext(hash,2,2));bad["map_from_odom_translation"]={.001,0.,0.};
  EXPECT_FALSE(map.applyLocalizationContext(bad.dump()));
  EXPECT_FALSE(map.applyLocalizationContext(priorContext(hash))); // Old latched context cannot resurrect prior.
  EXPECT_FALSE(map.applyLocalizationContext(priorContext(hash,2,2))); // Failed attestation's high-water barrier.
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash,2,3))); // Explicit fresh context transaction.
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2); // Valid identity alone is not a measurement lease.
  for(unsigned sensor=0;sensor<2;++sensor) {
    auto p=packet(sensor);p.epoch=2;p.context_sequence=3;p.barrier_ns=100000000003ULL;
    ASSERT_TRUE(GridMapTestAccess::accept(map,p));
  }
  GridMapTestAccess::integrate(map);EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  EXPECT_FALSE(map.applyLocalizationContext(priorContext(hash,1,1))); // Rollback deauthorizes, never keeps old FREE.
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
}

TEST(StaticPriorIntegration, EntirelyPriorProofStillExpiresWithBothRaysAndCannotRenewByTimerOrPause) {
  GridMap map;GridMapTestAccess::configure(map);const auto hash=GridMapTestAccess::attachStaticPrior(map);
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  const auto deadline=map.observedProofDeadlineNs();EXPECT_EQ(deadline,102400000000LL);
  GridMapTestAccess::integrate(map,102200000000LL,102.2);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);EXPECT_LE(map.observedProofDeadlineNs(),deadline);
  GridMapTestAccess::steadyClockAdvance(map,102200000000LL,102600000000LL);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2); // Source paused, actual callback receipt expired.
  EXPECT_EQ(map.observedProofDeadlineNs(),1);
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0),102200000000LL,102.6));
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
}

TEST(StaticPriorIntegration, CachedLeaseRechecksExactDeadlineAndRevocationInsteadOfReusingPreviousFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,.05},0.),0);EXPECT_TRUE(GridMapTestAccess::cachedPriorLease(map));
  GridMapTestAccess::steadyClockAdvance(map,102399999999LL,102399999999LL);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,.05},0.),0); // One ns before acquisition lease end.
  GridMapTestAccess::steadyClockAdvance(map,102400000000LL,102400000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,.05},0.),2);EXPECT_FALSE(GridMapTestAccess::cachedPriorLease(map));
  EXPECT_EQ(map.observedProofDeadlineNs(),1);
  auto invalid=nlohmann::json::parse(priorContext(hash,2,2));invalid["map_from_odom_translation"]={.01,0.,0.};
  EXPECT_FALSE(map.applyLocalizationContext(invalid.dump()));EXPECT_FALSE(GridMapTestAccess::cachedPriorLease(map));
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
}

TEST(StaticPriorIntegration, SnapshotAndNewProofScopeNeverInheritWritersCachedLease) {
  GridMap writer,snapshot;GridMapTestAccess::configure(writer);const auto hash=GridMapTestAccess::attachStaticPrior(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(priorContext(hash)));
  const auto now=std::chrono::steady_clock::now();
  const double receipt=std::chrono::duration<double>(now.time_since_epoch()).count();
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(writer,packet(sensor),102000000000LL,receipt));
  GridMapTestAccess::integrate(writer,102000000000LL,receipt);
  EXPECT_EQ(GridMapTestAccess::status(writer,{0,0,0}),0);ASSERT_TRUE(GridMapTestAccess::cachedPriorLease(writer));
  writer.copyCollisionSnapshotTo(snapshot,102000000000LL,now);
  EXPECT_FALSE(GridMapTestAccess::cachedPriorLease(snapshot));
  snapshot.beginObservedProof();EXPECT_EQ(snapshot.observedRawSnapshotStatus({0,0,0}),0);
  EXPECT_TRUE(GridMapTestAccess::cachedPriorLease(snapshot));
  snapshot.beginObservedProof();EXPECT_FALSE(GridMapTestAccess::cachedPriorLease(snapshot));
  EXPECT_EQ(snapshot.observedRawSnapshotStatus({0,0,0}),0);
  // Source at exact expiry is rejected on the first read of the new scope,
  // including a snapshot whose writer most recently checked a valid lease.
  writer.copyCollisionSnapshotTo(snapshot,102400000000LL,std::chrono::steady_clock::now());
  snapshot.beginObservedProof();EXPECT_EQ(snapshot.observedRawSnapshotStatus({0,0,0}),2);
  EXPECT_FALSE(GridMapTestAccess::cachedPriorLease(snapshot));EXPECT_EQ(snapshot.observedProofDeadlineNs(),1);
}

TEST(StaticPriorIntegration, RawCellCacheMatchesActualEvidenceAndCannotAliasCylinderCenter) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map,{{{1,0,0},1},{{2,0,0},2}});
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  const std::vector<Eigen::Vector3i> cells{{0,0,0},{1,0,0},{2,0,0},{9,0,0},{15,0,0},{30,0,0}};
  for(const auto &cell:cells) {
    EXPECT_EQ(GridMapTestAccess::status(map,cell),GridMapTestAccess::uncachedStatus(map,cell));
    const auto size=GridMapTestAccess::collisionCacheSize(map);
    for(unsigned repeat=0;repeat<5;++repeat)
      EXPECT_EQ(GridMapTestAccess::status(map,cell),GridMapTestAccess::uncachedStatus(map,cell));
    EXPECT_EQ(GridMapTestAccess::collisionCacheSize(map),size);
  }
  // Prime raw FREE in the front center cell; the exact cylinder includes a
  // distinct thin occupied neighbor. A shared positive key would hide it.
  GridMap cylinder;GridMapTestAccess::configure(cylinder);
  const auto second_hash=GridMapTestAccess::attachStaticPrior(cylinder,{{{1,0,0},1}});
  ASSERT_TRUE(cylinder.applyLocalizationContext(priorContext(second_hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(cylinder,packet(sensor)));
  GridMapTestAccess::integrate(cylinder);
  EXPECT_EQ(GridMapTestAccess::status(cylinder,{2,0,0}),0);
  EXPECT_EQ(cylinder.getInflateOccupancy({.05,.05,.05},0.),1);
}

TEST(StaticPriorIntegration, CachedRawFreeDoesNotHideNewWeakEvidenceOrActualHit) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  ASSERT_GT(GridMapTestAccess::collisionCacheSize(map),0U);
  GridMapTestAccess::indexedUpdate(map,{0,0,0},-.5);
  EXPECT_EQ(GridMapTestAccess::collisionCacheSize(map),0U);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2); // No semantic change for weak evidence.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{-.05,.05,.05},{.05,.05,.05}),102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_TRUE(GridMapTestAccess::conflict(map,{0,0,0}));
  EXPECT_NE(GridMapTestAccess::status(map,{0,0,0}),0);
}

TEST(StaticPriorIntegration, CachedRawLeaseCannotOutliveExactSourceOrReceiptDeadlineOrRevokedContext) {
  for(bool receipt_expires:{false,true}) {
    SCOPED_TRACE(receipt_expires);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
    GridMapTestAccess::integrate(map);
    ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
    const auto deadline=map.observedProofDeadlineNs();ASSERT_EQ(deadline,102400000000LL);
    GridMapTestAccess::steadyClockAdvance(map,102100000000LL,102100000000LL);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
    EXPECT_EQ(map.observedProofDeadlineNs(),deadline);
    GridMapTestAccess::steadyClockAdvance(map,receipt_expires?102100000000LL:deadline,
        receipt_expires?102500000000LL:102100000000LL);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);EXPECT_EQ(map.observedProofDeadlineNs(),1);
    auto invalid=nlohmann::json::parse(priorContext(hash,2,2));invalid["map_from_odom_translation"]={.01,0.,0.};
    EXPECT_FALSE(map.applyLocalizationContext(invalid.dump()));
    EXPECT_EQ(GridMapTestAccess::collisionCacheSize(map),0U);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
  }
}

TEST(StaticPriorIntegration, IndependentStaticFreeDoesNotInheritRedundantOldVoxelDeadline) {
  for(bool certified:{false,true}) {
    SCOPED_TRACE(certified);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    const auto hash=GridMapTestAccess::attachStaticPrior(map,{{{0,0,0},std::uint8_t(certified?0:2)}});
    ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
        packet(sensor,101900000000LL,1,1,{-.55,.05,.05},{.35,.05,.05})));
    GridMapTestAccess::integrate(map);
    const auto source=GridMapTestAccess::freeStamp(map,{0,0,0});
    const auto receipt=GridMapTestAccess::freeReceipt(map,{0,0,0});
    ASSERT_EQ(source,101900000000LL);ASSERT_EQ(receipt,102000000000LL);
    // Fresh remote real observations authorize the independent static map;
    // they never revisit or rewrite this voxel's original live observation.
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
        packet(sensor,102200000000LL,2,1,{1.05,.05,.05},{1.55,.05,.05}),102300000000LL,102.3));
    GridMapTestAccess::integrate(map,102300000000LL,102.3);
    EXPECT_EQ(map.observedProofDeadlineNs(),102400000000LL); // Old proof cannot be renewed by the new scan.
    // A new observation must not extend the preceding proof. The real reader
    // starts an independent scope before selecting the new both-ray evidence.
    GridMapTestAccess::beginIndependentMemoryProof(map);
    GridMapTestAccess::steadyClockAdvance(map,102399999999LL,102399999999LL);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
    EXPECT_EQ(map.observedProofDeadlineNs(),certified?102700000000LL:102400000000LL);
    GridMapTestAccess::steadyClockAdvance(map,102400000000LL,102400000000LL);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),certified?0:2);
    EXPECT_EQ(map.observedProofDeadlineNs(),certified?102700000000LL:102400000000LL);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{0,0,0}),source);
    EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{0,0,0}),receipt);
    GridMapTestAccess::steadyClockAdvance(map,102700000000LL,102700000000LL);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2); // Independent both-ray certificate also expires exactly.
    EXPECT_EQ(map.observedProofDeadlineNs(),1);
  }
}

TEST(StaticPriorIntegration, IndependentStaticFreeStillRejectsInvalidLiveWitnessAndHitVeto) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,101900000000LL,1,1,{-.55,.05,.05},{.35,.05,.05})));
  GridMapTestAccess::integrate(map);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  const auto source=GridMapTestAccess::freeStamp(map,{0,0,0});
  const auto receipt=GridMapTestAccess::freeReceipt(map,{0,0,0});
  for(const auto witness:std::vector<std::pair<std::int64_t,std::int64_t>>{
      {0,receipt},{102000000001LL,receipt},{source,0},{source,102000000001LL}}) {
    GridMapTestAccess::freeWitness(map,{0,0,0},witness.first,witness.second);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
  }
  GridMapTestAccess::freeWitness(map,{0,0,0},source,receipt);
  ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{-.05,.05,.05},{.05,.05,.05}),102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  ASSERT_TRUE(GridMapTestAccess::conflict(map,{0,0,0}));
  // A saturated log-odds FREE buffer by itself cannot erase the real hit.
  GridMapTestAccess::indexedUpdate(map,{0,0,0},-1.);
  GridMapTestAccess::freeWitness(map,{0,0,0},102000000000LL,102100000000LL);
  EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
}

TEST(StaticPriorIntegration, CapturedQueryLeaseDiagnosticCannotMutateEvidenceCacheOrDeadline) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
  const auto odds=GridMapTestAccess::buffers(map);
  const auto sources=GridMapTestAccess::counts(map);
  const auto cache_size=GridMapTestAccess::collisionCacheSize(map);
  const auto deadline=map.observedProofDeadlineNs();
  const auto description=nlohmann::json::parse(map.describeCollisionLease());
  EXPECT_EQ(description.at("query_source_ns"),102000000000LL);
  EXPECT_EQ(description.at("query_receipt_ns"),102000000000LL);
  EXPECT_EQ(description.at("source_deadline_ns"),102400000000LL);
  EXPECT_EQ(description.at("receipt_deadline_ns"),102500000000LL);
  EXPECT_EQ(description.at("both_ray_source_ns"),nlohmann::json({101900000000LL,101900000000LL}));
  EXPECT_EQ(description.at("both_ray_receipt_ns"),nlohmann::json({102000000000LL,102000000000LL}));
  EXPECT_EQ(GridMapTestAccess::buffers(map),odds);EXPECT_EQ(GridMapTestAccess::counts(map),sources);
  EXPECT_EQ(GridMapTestAccess::collisionCacheSize(map),cache_size);EXPECT_EQ(map.observedProofDeadlineNs(),deadline);
  EXPECT_EQ(map.describeCollisionLease(),description.dump());
}

TEST(StaticPriorIntegration, DirectRawCacheIsOptInAndItsReadSlotMemoryIsBudgeted) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
  EXPECT_EQ(GridMapTestAccess::rawCacheStorage(map),0U);EXPECT_EQ(GridMapTestAccess::rawCacheCapacity(map),0U);
  GridMap prior;GridMapTestAccess::configure(prior);const auto hash=GridMapTestAccess::attachStaticPrior(prior);
  ASSERT_TRUE(prior.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(prior,packet(sensor)));
  GridMapTestAccess::integrate(prior);ASSERT_EQ(GridMapTestAccess::status(prior,{0,0,0}),0);
  EXPECT_EQ(GridMapTestAccess::rawCacheStorage(prior),GridMapTestAccess::buffers(prior).size());
  EXPECT_EQ(prior.collisionSnapshotBytes()-GridMapTestAccess::snapshotBytesWithoutRawCache(prior),
      GridMapTestAccess::buffers(prior).size()*sizeof(std::int8_t));
}

TEST(StaticPriorIntegration, SnapshotKeepsDestinationStorageButNeverCopiesCachedWriterFree) {
  GridMap writer,reader;GridMapTestAccess::configure(writer);const auto hash=GridMapTestAccess::attachStaticPrior(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(priorContext(hash)));
  const auto captured=std::chrono::steady_clock::now();
  const double receipt=std::chrono::duration<double>(captured.time_since_epoch()).count();
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(writer,packet(sensor),102000000000LL,receipt));
  GridMapTestAccess::integrate(writer,102000000000LL,receipt);ASSERT_EQ(GridMapTestAccess::status(writer,{0,0,0}),0);
  writer.copyCollisionSnapshotTo(reader,102000000000LL,captured);
  EXPECT_EQ(GridMapTestAccess::collisionCacheSize(reader),0U);
  EXPECT_NE(GridMapTestAccess::rawCacheData(reader),GridMapTestAccess::rawCacheData(writer));
  reader.beginObservedProof();ASSERT_EQ(reader.observedRawSnapshotStatus({0,0,0}),0);
  const auto storage=GridMapTestAccess::rawCacheData(reader);
  const auto capacity=GridMapTestAccess::rawCacheCapacity(reader);
  GridMapTestAccess::indexedUpdate(writer,{0,0,0},-.5); // Actual weak evidence veto still applies.
  writer.copyCollisionSnapshotTo(reader,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_EQ(GridMapTestAccess::rawCacheData(reader),storage);EXPECT_EQ(GridMapTestAccess::rawCacheCapacity(reader),capacity);
  EXPECT_EQ(GridMapTestAccess::collisionCacheSize(reader),0U);
  reader.beginObservedProof();EXPECT_EQ(reader.observedRawSnapshotStatus({0,0,0}),2);
}

TEST(StaticPriorIntegration, DenseFreeCannotSurviveSlotGenerationWrapOrExhaustedClearSerial) {
  for(bool serial_saturates:{false,true}) {
    SCOPED_TRACE(serial_saturates);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
    GridMapTestAccess::integrate(map);ASSERT_EQ(GridMapTestAccess::status(map,{0,0,0}),0);
    auto &cache=GridMapTestAccess::cylinderCache(map);
    if(serial_saturates)scan_planner::VoxelStatusCacheTestAccess::forceNextSerialSaturation(cache);
    else scan_planner::VoxelStatusCacheTestAccess::forceNextSlotGenerationWrap(cache);
    cache.clear(); // The real invalidation path must be observed by the dense cache.
    GridMapTestAccess::indexedUpdate(map,{0,0,0},-.5);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),2);
    EXPECT_EQ(cache.generationReusable(),!serial_saturates);
    if(serial_saturates)EXPECT_EQ(GridMapTestAccess::rawCacheStorage(map),0U);
    GridMapTestAccess::indexedUpdate(map,{0,0,0},2.);
    EXPECT_EQ(GridMapTestAccess::status(map,{0,0,0}),1);
  }
}

TEST(StaticPriorIntegration, ImmutableSnapshotCopiesContradictionsAndContextWithoutMutatingWriterOrPrior) {
  GridMap map,before,after;GridMapTestAccess::configure(map);
  const auto hash=GridMapTestAccess::attachStaticPrior(map);ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  const auto captured=std::chrono::steady_clock::now();
  const double receipt=std::chrono::duration<double>(captured.time_since_epoch()).count();
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor),102000000000LL,receipt));
  GridMapTestAccess::integrate(map,102000000000LL,receipt);
  map.copyCollisionSnapshotTo(before,102000000000LL,captured);before.beginObservedProof();
  EXPECT_EQ(before.observedRawSnapshotStatus({0,0,0}),0);EXPECT_EQ(GridMapTestAccess::freeStamp(before,{0,0,0}),0);
  const double second_receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{-.05,.05,.05},{.05,.05,.05}),102100000000LL,second_receipt));
  GridMapTestAccess::integrate(map,102100000000LL,second_receipt);
  map.copyCollisionSnapshotTo(after,102100000000LL,std::chrono::steady_clock::now());
  after.beginObservedProof();EXPECT_NE(after.observedRawSnapshotStatus({0,0,0}),0);
  before.beginObservedProof();EXPECT_EQ(before.observedRawSnapshotStatus({0,0,0}),0);
  EXPECT_TRUE(GridMapTestAccess::conflict(after,{0,0,0}));EXPECT_FALSE(GridMapTestAccess::conflict(before,{0,0,0}));
}

TEST(StaticPriorIntegration, FixedWorldHitCannotBeForgottenByContextSequenceEpochOrSeedRebuild) {
  GridMap map;GridMapTestAccess::configure(map);const auto hash=GridMapTestAccess::attachStaticPrior(map);
  ASSERT_TRUE(map.applyLocalizationContext(priorContext(hash)));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);ASSERT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
  for(unsigned sequence=2;sequence<=3;++sequence) {
    const unsigned epoch=sequence==2?1:2;
    auto value=nlohmann::json::parse(priorContext(hash,epoch,sequence));
    if(sequence==3)value["seed_id"]="new-seed";
    ASSERT_TRUE(map.applyLocalizationContext(value.dump()));
    EXPECT_TRUE(GridMapTestAccess::conflict(map,{1,0,0}));
    for(unsigned sensor=0;sensor<2;++sensor) {
      auto p=packet(sensor,102000000000LL+sequence*100000000LL,sequence,1,{1.05,.05,.05},{1.55,.05,.05});
      p.epoch=epoch;p.context_sequence=sequence;p.barrier_ns=100000000000ULL+sequence;
      p.seed_id=sequence==3?"new-seed":"seed";
      ASSERT_TRUE(GridMapTestAccess::accept(map,p,102100000000LL+sequence*100000000LL,102.1+sequence*.1));
    }
    GridMapTestAccess::integrate(map,102100000000LL+sequence*100000000LL,102.1+sequence*.1);
    EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(GridMapTestAccess::status(map,{1,0,0}),2);
  }
}

TEST(ProjectedRayIntegration, CachedAddressLogOddsUpdateExactlyMatchesLegacyTransitions) {
  for(bool strict:{false,true}) {
    GridMap legacy,indexed;
    GridMapTestAccess::configure(legacy,strict);GridMapTestAccess::configure(indexed,strict);
    for(const Eigen::Vector3i &cell:{Eigen::Vector3i(-20,-20,-20),Eigen::Vector3i(-1,0,1),
        Eigen::Vector3i(19,19,19),Eigen::Vector3i(0,0,0)}) {
      for(double value:{-1.01,-1.,-1.,-.5,0.,-0.,0.,1.,1.01,2.,2.,1.,-.5,-1.,-1.01}) {
        GridMapTestAccess::legacyUpdate(legacy,cell,value);
        GridMapTestAccess::indexedUpdate(indexed,cell,value);
        GridMapTestAccess::expectSameEvidence(legacy,indexed);
        // Signed zero is preserved too, so a raw-byte map hash is unchanged.
        const auto a=GridMapTestAccess::buffers(legacy),b=GridMapTestAccess::buffers(indexed);
        ASSERT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(double)),0);
      }
    }
  }
}
TEST(ProjectedRayIntegration, SnapshotHookSeesCompletedBothSourceTransactionWithoutExtraTimer) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  int publications=0;std::array<std::int64_t,2> sources{};std::int64_t integrated=0;
  map.setCollisionUpdateCallback([&]{++publications;sources={map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1)};
    integrated=map.latestCloudStampNs();});
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0)));ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  EXPECT_EQ(publications,0);GridMapTestAccess::integrate(map);
  EXPECT_EQ(publications,1);EXPECT_EQ(sources[0],101900000000LL);EXPECT_EQ(sources[1],101900000000LL);
  EXPECT_EQ(integrated,101900000000LL);
  GridMapTestAccess::integrate(map,102500000000LL,102.5);
  EXPECT_EQ(publications,2);EXPECT_EQ(integrated,0); // Expiry is published too, never renewed.
}
TEST(ProjectedRayIntegration,NewDualSourceEventFusesImmediatelyButNeverExceedsFiveHz) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  int snapshots=0;map.setCollisionUpdateCallback([&]{++snapshots;});
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0)));EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102000000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  EXPECT_EQ(snapshots,1); // No 200 ms timer wait after the complete packet pair.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
    packet(sensor,102000000000LL,2),102100000000LL,102.1));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102100000000LL));
  // Both observations are newer than the integrated pair. Do not require
  // acquisition advance >=180 ms on top of the already elapsed 200 ms cap.
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102200000000LL,true));
  EXPECT_EQ(snapshots,2);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102000000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),102000000000LL);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,3),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,102100000000LL,3),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102399999999LL));
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102400000000LL));
  EXPECT_EQ(snapshots,3);EXPECT_EQ(GridMapTestAccess::counts(map)[0],3U);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102410000000LL,true));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102600000000LL,true));
  EXPECT_EQ(snapshots,3);EXPECT_EQ(map.latestCloudStampNs(),102100000000LL);
  EXPECT_FALSE(map.integratedCloudFreshAt(102600000001LL)); // No-data watchdog never refreshes a source.
}
TEST(ProjectedRayIntegration,MissingRearWatchdogCanAddFrontHitsWithoutRefreshingRear) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,2),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102310000000LL,true));
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
  EXPECT_FALSE(map.integratedCloudFreshAt(102410000000LL));
}

TEST(ProjectedRayIntegration,DeadlineWakeTargetsDueInsteadOfRoundingToPollingPhase) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102000000000LL),20000000);
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102180000000LL),20000000);
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102193000000LL),7000000);
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102199900000LL),1000000); // Bounded wake, never busy-spin.
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102200000000LL),20000000); // Due without data is not a poll storm.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102050000000LL,2),102150000000LL,102.15));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102207000000LL,true)); // A late executor callback.
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102393000000LL),14000000); // Relative to actual late start.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102200000000LL,3),102300000000LL,102.3));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102400000000LL,true)); // No 193 ms phase catch-up.
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102407000000LL,true));
  const auto before=map.fusionTiming();
  EXPECT_EQ(before.sequence,3U);EXPECT_EQ(before.begin_ns,102407000000LL);
  EXPECT_EQ(before.end_ns,102407000000LL); // Injected passive clock; no invented runtime duration.
  EXPECT_EQ(before.source_stamps[0],102200000000LL);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102607000000LL,true));
  EXPECT_EQ(map.fusionTiming().sequence,before.sequence);
  EXPECT_EQ(map.fusionTiming().source_stamps,before.source_stamps);
}

TEST(ProjectedRayIntegration, SaturatedFreeNoOpStillRefreshesOnlyActualSourceEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  std::uint64_t saturated_revision=0;
  for(unsigned sequence=1;sequence<=8;++sequence) {
    const std::int64_t stamp=101700000000LL+sequence*20000000LL;
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,stamp,sequence,128)));
    GridMapTestAccess::integrate(map);
    EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,.65),-1.);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{6,0,0}),stamp);
    EXPECT_EQ(GridMapTestAccess::counts(map)[0],sequence);
    EXPECT_EQ(GridMapTestAccess::counts(map)[1],sequence);
    EXPECT_DOUBLE_EQ(map.latestCloudStamp(),stamp*1e-9);
    if(sequence==1) saturated_revision=map.occupancyRevision();
    else EXPECT_EQ(map.occupancyRevision(),saturated_revision);
  }
}

TEST(ProjectedRayIngress, DrainIsBoundedEvenWhenProducerNeverEmpties) {
  unsigned takes=0,validations=0;
  const auto result=scan_planner::drainProjectedRayIngress<int>(
      [&takes](int &message) {message=++takes;return true;},
      [&validations](int message) {++validations;return message%2==0;});
  EXPECT_EQ(result.taken,5U);EXPECT_EQ(result.accepted,2U);
  EXPECT_EQ(takes,5U);EXPECT_EQ(validations,5U);
}

TEST(ProjectedRayIngress, SlowSourceSurvivesEveryPositionInMixedFiveMessageWindow) {
  for(unsigned rear_position=0;rear_position<5;++rear_position) {
    GridMap map;GridMapTestAccess::configure(map,false);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue;
    std::int64_t front_stamp=0;
    for(unsigned i=0;i<5;++i) {
      if(i==rear_position) queue.push_back(packet(1,101850000000LL,1));
      else {front_stamp=101700000000LL+i*40000000LL;queue.push_back(packet(0,front_stamp,i+1));}
    }
    std::size_t cursor=0;
    const auto result=GridMapTestAccess::drain(map,queue,cursor);
    ASSERT_EQ(result.taken,5U);ASSERT_EQ(result.accepted,5U);ASSERT_EQ(cursor,5U);
    ASSERT_EQ(GridMapTestAccess::pending(map),2);
    EXPECT_EQ(GridMapTestAccess::drops(map)[0],3U); // Whole older front batches only.
    EXPECT_EQ(GridMapTestAccess::drops(map)[1],0U);
    GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
    EXPECT_DOUBLE_EQ(map.latestCloudStamp(),std::min(front_stamp,std::int64_t{101850000000LL})*1e-9);
    EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],102000000000LL);
    EXPECT_EQ(GridMapTestAccess::integrationStart(map)[1],102000000000LL);
    ASSERT_TRUE(map.applyLocalizationContext(context(2,2)));
    EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],0);
    EXPECT_EQ(GridMapTestAccess::integrationStart(map)[1],0);
  }
}

TEST(ProjectedRayIngress, QueuedStaleSourceCannotRenewLeaseWhileOtherSourceContinues) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,101600000000LL,1)));
  GridMapTestAccess::integrate(map);
  ASSERT_GT(map.latestCloudStamp(),0.);
  const std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue{
    packet(1,101700000000LL,2),packet(0,102500000000LL,2),packet(0,102550000000LL,3)};
  std::size_t cursor=0;
  const auto result=GridMapTestAccess::drain(map,queue,cursor,102600000000LL,102.6);
  EXPECT_EQ(result.taken,3U);EXPECT_EQ(result.accepted,2U);
  EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],102000000000LL); // Not refreshed by rejected take.
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],2U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),0.);
}

TEST(ProjectedRayIngress, CompleteSelectedBatchesMatchNativeBuffersAndRejectPartialMalformedBatch) {
  GridMap actual,reference;
  GridMapTestAccess::configure(actual,false);GridMapTestAccess::configure(reference,false);
  ASSERT_TRUE(actual.applyLocalizationContext(context()));ASSERT_TRUE(reference.applyLocalizationContext(context()));
  auto front=packet(0,101900000000LL,3,3);
  auto rear=packet(1,101850000000LL,1,3,{-.55,.05,.05},{-1.55,.05,.05});
  for(auto *message:{&front,&rear}) for(unsigned i=0;i<3;++i) {
    const float lane=.05f+i*.3f;
    std::memcpy(message->rays.data.data()+64*i+4,&lane,4);
    std::memcpy(message->rays.data.data()+64*i+20,&lane,4);
  }
  auto malformed=packet(0,101950000000LL,4,3);
  const float nan=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(malformed.rays.data.data()+64*2,&nan,4); // Last ray invalid: accept none.
  const std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue{
    packet(0,101700000000LL,1),rear,packet(0,101800000000LL,2),front,malformed};
  std::size_t cursor=0;
  const auto result=GridMapTestAccess::drain(actual,queue,cursor);
  EXPECT_EQ(result.taken,5U);EXPECT_EQ(result.accepted,4U);
  ASSERT_TRUE(GridMapTestAccess::accept(reference,front));ASSERT_TRUE(GridMapTestAccess::accept(reference,rear));
  GridMapTestAccess::integrate(actual);GridMapTestAccess::integrate(reference);
  EXPECT_EQ(GridMapTestAccess::integratedPoints(actual),6);
  GridMapTestAccess::expectSameEvidence(actual,reference);
  GridMapTestAccess::expectSameRayEvidence(actual,reference);
  for(unsigned i=0;i<3;++i) {
    EXPECT_GT(GridMapTestAccess::raw(actual,1.55,.05+i*.3),1.);
    EXPECT_GT(GridMapTestAccess::raw(actual,-1.55,.05+i*.3),1.);
  }
}

TEST(ProjectedRaysPreviewTiming, ExplicitOfficialProjectedPreviewContractOnly) {
  using scan_planner::validateCloudPoseTiming;
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.5,false,true,true));
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.5,false,false,true));
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.75,true,false,true));
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,false,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,true,true,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.5,true,true,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,true,false,false),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.750001,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.250001,.75,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,NAN,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,0.,true,false,true),std::invalid_argument);
}

TEST(ProjectedRaysPreviewTiming, OriginalSourceClockAcceptedThrough750msThenRevoked) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::previewTiming(map,.75);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  constexpr std::int64_t source=101900000000LL;
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source),source+750000000LL,102.65));
  GridMapTestAccess::integrate(map,source+750000000LL,102.65);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),101.9); // Never restamped to receipt/integration.
  EXPECT_TRUE(map.integratedCloudFreshAt(source+600000000LL));
  EXPECT_TRUE(map.integratedCloudFreshAt(source+750000000LL));
  EXPECT_FALSE(map.integratedCloudFreshAt(source+750001000LL));
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);
  EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  GridMapTestAccess::integrate(map,source+750001000LL,102.650001);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),0.);
  EXPECT_FALSE(map.integratedCloudFreshAt(source+750001000LL));
  GridMap rejected;GridMapTestAccess::configure(rejected,false);
  GridMapTestAccess::previewTiming(rejected,.75);
  ASSERT_TRUE(rejected.applyLocalizationContext(context()));
  EXPECT_FALSE(GridMapTestAccess::accept(rejected,packet(0,source),source+750001000LL,102.650001));
  EXPECT_EQ(GridMapTestAccess::counts(rejected)[0],0U);
  GridMap normal;GridMapTestAccess::configure(normal,false);
  ASSERT_TRUE(normal.applyLocalizationContext(context()));
  EXPECT_FALSE(GridMapTestAccess::accept(normal,packet(0,source),source+500001000LL,102.400001));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(normal,packet(sensor,source),source+100000000LL,102.0));
  GridMapTestAccess::integrate(normal,source+100000000LL,102.0);
  EXPECT_TRUE(normal.integratedCloudFreshAt(source+500000000LL));
  EXPECT_FALSE(normal.integratedCloudFreshAt(source+500001000LL));
}

static d1max_planning_interfaces::msg::ProjectedRays withSourceFields(
    d1max_planning_interfaces::msg::ProjectedRays out) {
  using F=sensor_msgs::msg::PointField;
  struct Field {const char *name;unsigned offset;unsigned type;};
  for(const auto &field:std::initializer_list<Field>{{"ring",30,F::UINT16},
      {"offset_time",32,F::UINT32},{"source_index",36,F::UINT32},
      {"timestamp",40,F::FLOAT64},{"source_timestamp",48,F::FLOAT64},{"raw_timestamp",56,F::FLOAT64}}) {
    F f;f.name=field.name;f.offset=field.offset;f.datatype=field.type;f.count=1;
    out.rays.fields.push_back(f);
  }
  for(unsigned i=0;i<out.rays.width;++i) {
    auto *bytes=out.rays.data.data()+64*i;
    const std::uint16_t ring=22;
    const std::uint32_t offset=100+i,index=12000+i;
    const double timestamp=101.9000001+i*1e-9,source=101.4+i*1e-9,raw=101.3+i*1e-9;
    std::memcpy(bytes+30,&ring,2);std::memcpy(bytes+32,&offset,4);std::memcpy(bytes+36,&index,4);
    std::memcpy(bytes+40,&timestamp,8);std::memcpy(bytes+48,&source,8);std::memcpy(bytes+56,&raw,8);
  }
  return out;
}

TEST(ProjectedRaysOfficialScan, UnknownQueriesDoNotInventObservedFreeEvidence) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::productionGeometry(official);GridMapTestAccess::productionGeometry(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  const auto raw=GridMapTestAccess::buffers(official);
  for(double yaw:{0.,.63,1.57}) {
    EXPECT_EQ(official.getInflateOccupancy({.05,.05,.05},yaw),0);
    EXPECT_EQ(strict.getInflateOccupancy({.05,.05,.05},yaw),2);
  }
  EXPECT_EQ(GridMapTestAccess::buffers(official),raw);
  EXPECT_EQ(GridMapTestAccess::category(official,{0,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  EXPECT_EQ(GridMapTestAccess::freeStamp(official,{0,0,0}),0);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),0.);
}

TEST(ProjectedRaysOfficialScan, RealOccupiedInflationBlocksBothCentersAtRotatedHeadings) {
  for(double yaw:{0.,1.5707963267948966}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,
        {.05,.25,.05},{.55,.25,.05})));
    GridMapTestAccess::integrate(map);
    ASSERT_EQ(GridMapTestAccess::category(map,{5,2,0}),scan_planner::RawVoxelDiagnostic::Occupied);
    // The center itself has no hit. An actual hit within the inflation radius
    // must block either member of the official two-center query.
    EXPECT_EQ(GridMapTestAccess::category(map,{5,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
    const Eigen::Vector3d center(.55,.05,.05),heading(std::cos(yaw),std::sin(yaw),0.);
    for(int side:{-1,1}) EXPECT_EQ(map.getInflateOccupancy(center+side*.2*heading,yaw),1);
    EXPECT_EQ(map.getInflateOccupancy({-.75,-.75,.05},yaw),0);
  }
}

TEST(ProjectedRaysOfficialScan, OutsideAndNonfiniteQueriesRemainBlocked) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),-1); // Front center outside.
  EXPECT_EQ(map.getInflateOccupancy({-1.85,.05,.05},0.),-1); // Rear center outside.
  EXPECT_EQ(map.getInflateOccupancy({.05,1.85,.05},1.5707963267948966),-1);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,2.05},0.),-1);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(map.getInflateOccupancy({nan,.05,.05},0.),-1);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,.05},nan),-1);
}

TEST(ProjectedRaysOfficialScan, SamePerPointOriginsAndSourceTimesProduceIdenticalEvidence) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::diagnostics(official);GridMapTestAccess::diagnostics(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) {
    const double sign=sensor?-1.:1.;const auto stamp=101900000000LL+sensor*50000000LL;
    auto rays=withSourceFields(packet(sensor,stamp,1,2,{sign*.55,.05,.05},{sign*1.55,.05,.05}));
    // Distinct origins within each physical source must survive decoding too.
    const float lane=.75f;
    std::memcpy(rays.rays.data.data()+64+4,&lane,4);
    std::memcpy(rays.rays.data.data()+64+20,&lane,4);
    ASSERT_TRUE(GridMapTestAccess::accept(official,rays));ASSERT_TRUE(GridMapTestAccess::accept(strict,rays));
  }
  GridMapTestAccess::integrate(official);GridMapTestAccess::integrate(strict);
  GridMapTestAccess::expectSameRayEvidence(official,strict);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),101.9);
  EXPECT_EQ(GridMapTestAccess::counts(official)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(official)[1],1U);
  for(unsigned sensor=0;sensor<2;++sensor) {
    const double sign=sensor?-1.:1.;const auto stamp=101900000000LL+sensor*50000000LL;
    const Eigen::Vector3d traversed(sign*.95,.75,.05),end(sign*1.55,.75,.05);
    EXPECT_EQ(GridMapTestAccess::freeStamp(official,GridMapTestAccess::index(official,traversed)),stamp);
    const auto *history=official.nearFieldDiagnostics().find(GridMapTestAccess::index(official,end));
    ASSERT_NE(history,nullptr);ASSERT_TRUE(history->last[sensor*2]);
    const auto &w=*history->last[sensor*2];
    EXPECT_TRUE(w.origin.isApprox(Eigen::Vector3d(sign*.55,.75,.05),1e-6));
    EXPECT_TRUE(w.endpoint.isApprox(end,1e-6));
    EXPECT_EQ(w.metadata.sensor_id,sensor);EXPECT_EQ(w.metadata.scan_stamp_ns,stamp);
    EXPECT_EQ(w.metadata.offset_time_ns,101U);EXPECT_EQ(w.metadata.source_index,12001U);
    EXPECT_EQ(w.metadata.integration_ns,102000000000LL);
    EXPECT_DOUBLE_EQ(w.metadata.source_timestamp,101.4+1e-9);
    EXPECT_DOUBLE_EQ(w.metadata.raw_timestamp,101.3+1e-9);
  }
  for(const Eigen::Vector3d &position:{Eigen::Vector3d(.05,.75,.05),Eigen::Vector3d(1.75,.75,.05)})
    EXPECT_EQ(GridMapTestAccess::category(official,GridMapTestAccess::index(official,position)),
        scan_planner::RawVoxelDiagnostic::NeverObserved);
}

TEST(ProjectedRaysOfficialScan, FreeExpiryDoesNotChangeOfficialQueryOrRenewSourceLease) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::centerOnlyOfficialGeometry(official);GridMapTestAccess::pointQueryGeometry(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) {
    ASSERT_TRUE(GridMapTestAccess::accept(official,packet(sensor)));
    ASSERT_TRUE(GridMapTestAccess::accept(strict,packet(sensor)));
  }
  GridMapTestAccess::integrate(official);GridMapTestAccess::integrate(strict);
  ASSERT_GT(official.latestCloudStamp(),0.);
  GridMapTestAccess::integrate(official,102600000000LL,102.6);
  GridMapTestAccess::integrate(strict,102600000000LL,102.6);
  EXPECT_EQ(official.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(strict.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(official.getInflateOccupancy({1.55,.05,.05},0.),1);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),0.);
  EXPECT_EQ(GridMapTestAccess::freeStamp(official,{9,0,0}),101900000000LL);
  GridMapTestAccess::expectSameRayEvidence(official,strict);
}

TEST(ProjectedRaysFreshness, FreshTraversalExpiresWithoutMutatingOddsOrOccupied) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0); // cached positive
  const auto raw=GridMapTestAccess::buffers(map);
  GridMapTestAccess::integrate(map,102400000000LL,102.4); // exact half-second expiry
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
  EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),2); // occluded / never observed
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);
  const auto detail=map.inspectInflateOccupancy({.95,.05,.05},0.,true);
  ASSERT_EQ(detail.voxels.size(),1U);
  EXPECT_EQ(detail.voxels[0].raw_state,0);
  EXPECT_EQ(detail.voxels[0].native_state,2);
  EXPECT_EQ(detail.voxels[0].classification,scan_planner::RawVoxelDiagnostic::StaleFree);
  EXPECT_EQ(detail.voxels[0].free_observation_stamp_ns,101900000000LL);
}

TEST(ProjectedRaysFreshness, FreshRemoteSourcesDoNotRenewNearRegionAndRealReobservationDoes) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,103350000000LL,2,1,{.55,.75,.05},{1.55,.75,.05}),103450000000LL,103.45));
  GridMapTestAccess::integrate(map,103450000000LL,103.45);
  EXPECT_GT(map.latestCloudStamp(),103.);
  EXPECT_EQ(map.getInflateOccupancy({.95,.75,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,103450000000LL,3),103550000000LL,103.55));
  GridMapTestAccess::integrate(map,103550000000LL,103.55);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, ClockRollbackCannotRevivePreviouslyExpiredFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  GridMapTestAccess::integrate(map,102000000000LL,102.7);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  // Even fresh data in a rewound clock cannot bypass the required new context.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102700000000LL,2),102800000000LL,102.8));
  GridMapTestAccess::integrate(map,102800000000LL,102.8);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
}

TEST(ProjectedRaysFreshness, QueryClockExpiresCacheWithoutAnIntegrationHeartbeat) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::queryTime(map,102500000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  GridMapTestAccess::queryTime(map,102000000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
}

TEST(ProjectedRaysFreshness, ResetAndSlidingClearEvidenceWithoutAliasingNewWorldCells) {
  for(bool slide:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
    if(slide) {
      GridMapTestAccess::slide(map,{3.,0.,0.});
      EXPECT_EQ(GridMapTestAccess::freeStamp(map,{49,0,0}),0);
      EXPECT_EQ(map.getInflateOccupancy({4.95,.05,.05},0.),2);
    } else {
      map.resetBuffer();
      EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
      EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
    }
  }
}

TEST(ProjectedRaysFreshness, PausedAndSlowSourceClockCannotRenewLeaseViaRepeatedTimerTicks) {
  for(bool slow:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
    std::int64_t effective=0;
    for(int i=1;i<=8;++i) {
      effective=GridMapTestAccess::steadyClockAdvance(map,102000000000LL+(slow?i*1000000LL:0LL),
          102000000000LL+i*100000000LL);
    }
    EXPECT_GE(effective,102800000000LL);
    EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  }
}

TEST(ProjectedRaysSimulationFreshness, ThreeTimesSlowerRosClockKeepsActualNewTraversalFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sequence=1;sequence<=10;++sequence) {
    SCOPED_TRACE(sequence);
    const std::int64_t source=102000000000LL+(sequence-1)*100000000LL;
    const double integration_receipt=400.+(sequence-1)*.3;
    const double callback_receipt=integration_receipt-.06;
    for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
        packet(sensor,source-100000000LL,sequence),source,callback_receipt));
    GridMapTestAccess::integrate(map,source,integration_receipt);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),source-100000000LL);
    EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),
        static_cast<std::int64_t>(callback_receipt*1e9));
    // Source advances 50 ms while steady time advances 150 ms. Both actual
    // observation ages are fresh; wall elapsed must not become a ROS stamp.
    EXPECT_EQ(GridMapTestAccess::steadyClockAdvance(map,source+50000000LL,
        static_cast<std::int64_t>((integration_receipt+.15)*1e9)),source+50000000LL);
    EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
    EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
    EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),2);
  }
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],10U);
  EXPECT_EQ(GridMapTestAccess::counts(map)[1],10U);
}

TEST(ProjectedRaysSimulationFreshness, ReplayedSourceAndNoTraversalTicksCannotRenewReceipt) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(),102000000000LL,400.));
  GridMapTestAccess::integrate(map,102000000000LL,400.);
  ASSERT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  const auto raw=GridMapTestAccess::buffers(map);
  // A higher projection sequence does not turn the same acquisition into a
  // new observation, even though its original ROS stamp is still fresh.
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101900000000LL,2),102100000000LL,400.4));
  GridMapTestAccess::integrate(map,102100000000LL,400.4);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::integrate(map,102110000000LL,400.5);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400000000000LL);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);
}

TEST(ProjectedRaysSimulationFreshness, PausedQueryExpiresAtOriginalCallbackReceiptDeadline) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(),102000000000LL,400.));
  GridMapTestAccess::integrate(map,102020000000LL,400.2);
  ASSERT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  const auto raw=GridMapTestAccess::buffers(map);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400000000000LL);
  GridMapTestAccess::steadyClockAdvance(map,102040000000LL,400499999999LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  // No fusion or new packet: the positive cache must expire at receipt +.5 s,
  // rather than at the later integration +.5 s or only when ROS advances.
  GridMapTestAccess::steadyClockAdvance(map,102040000000LL,400500000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);
}

TEST(ProjectedRaysSimulationFreshness, SourceDeadlineExpiresEvenWhenReceiptRemainsFresh) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(),102000000000LL,400.));
  GridMapTestAccess::integrate(map,102000000000LL,400.);
  ASSERT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::steadyClockAdvance(map,102399999999LL,400199999999LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::steadyClockAdvance(map,102400000000LL,400200000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400000000000LL);
}

TEST(ProjectedRaysSimulationFreshness, RosRollbackCannotReviveReceiptExpiredFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(),102000000000LL,400.));
  GridMapTestAccess::integrate(map,102000000000LL,400.);
  GridMapTestAccess::integrate(map,102100000000LL,400.6);
  ASSERT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  GridMapTestAccess::integrate(map,102000000000LL,400.7);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,2),102200000000LL,400.8));
  GridMapTestAccess::integrate(map,102200000000LL,400.8);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),102100000000LL);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400800000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2); // New context is still required.
}

TEST(ProjectedRaysSimulationFreshness, FreshRemoteTraversalCannotReviveOldCellReceipt) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  GridMapTestAccess::simulationCollisionClock(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor),102000000000LL,400.));
  GridMapTestAccess::integrate(map,102000000000LL,400.);
  ASSERT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102100000000LL,2,1,{.55,.75,.05},{1.55,.75,.05}),102200000000LL,400.6));
  GridMapTestAccess::integrate(map,102200000000LL,400.6);
  EXPECT_GT(map.latestCloudStamp(),0.);
  EXPECT_EQ(map.getInflateOccupancy({.95,.75,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400000000000LL);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102200000000LL,3),102300000000LL,400.7));
  GridMapTestAccess::integrate(map,102300000000LL,400.7);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),102200000000LL);
  EXPECT_EQ(GridMapTestAccess::freeReceipt(map,{9,0,0}),400700000000LL);
  EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),2);
}

TEST(ProjectedRaysSimulationFreshness, SnapshotCopiesReceiptAndExpiresWithoutFreshFusion) {
  // Captures in the past exercise snapshot ageing immediately, without sleeps
  // or a ROS node. The expired case still has a fresh .45 s ROS acquisition age,
  // while its original callback receipt is .55 s old.
  for(bool expired:{false,true}) {
    SCOPED_TRACE(expired);
    GridMap map,snapshot;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    GridMapTestAccess::simulationCollisionClock(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    const auto captured=std::chrono::steady_clock::now()-
        std::chrono::milliseconds(expired?350:100);
    const double capture_s=std::chrono::duration<double>(captured.time_since_epoch()).count();
    const double callback_receipt=capture_s-(expired?.20:.05);
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(),102000000000LL,callback_receipt));
    GridMapTestAccess::integrate(map,102000000000LL,capture_s);
    map.copyCollisionSnapshotTo(snapshot,102000000000LL,captured);
    EXPECT_EQ(GridMapTestAccess::freeStamp(snapshot,{9,0,0}),101900000000LL);
    EXPECT_EQ(GridMapTestAccess::freeReceipt(snapshot,{9,0,0}),
        GridMapTestAccess::freeReceipt(map,{9,0,0}));
    EXPECT_EQ(snapshot.getInflateOccupancy({.95,.05,.05},0.),expired?2:0);
    if(expired) EXPECT_EQ(snapshot.observedProofDeadlineNs(),1);
    else {
      EXPECT_GT(snapshot.observedProofDeadlineNs(),102100000000LL);
      EXPECT_LE(snapshot.observedProofDeadlineNs(),102400000000LL);
    }
    EXPECT_EQ(snapshot.getInflateOccupancy({1.55,.05,.05},0.),1);
  }
}

TEST(ProjectedRaysFreshness, EndpointHitNeverRenewsFreeLease) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,103350000000LL,2,1,
      {.55,.05,.05},{.95,.05,.05}),103450000000LL,103.45));
  GridMapTestAccess::integrate(map,103450000000LL,103.45);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, PartialBatchCannotLendFreeEvidenceToLaterRemoteSuccess) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  // Front integrates near rays, rear has an origin outside the actual window.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,1,1,{3.,0.,0.},{3.5,0.,0.})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.latestCloudStamp(),0.);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{.55,.75,.05},{1.55,.75,.05}),102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_GT(map.latestCloudStamp(),0.);
  EXPECT_EQ(map.getInflateOccupancy({.95,.75,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, ActualFreeRecoveryWakesPlannerButFreshHeartbeatsDoNot) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  auto revision=map.occupancyRevision();
  GridMapTestAccess::integrate(map,102400000000LL,102.4);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.occupancyRevision(),revision);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102400000000LL,2),102500000000LL,102.5));
  GridMapTestAccess::integrate(map,102500000000LL,102.5);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_GT(map.occupancyRevision(),revision); // existing WAIT_ENVIRONMENT compares exactly this
  revision=map.occupancyRevision();
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102500000000LL,3),102600000000LL,102.6));
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(map.occupancyRevision(),revision); // no repeated search on steady fresh data
}

TEST(ProjectedRaysDiagnostics, EnablingDiagnosticsLeavesAllNativeBuffersRevisionAndCollisionUnchanged) {
  GridMap off,on;GridMapTestAccess::configure(off);GridMapTestAccess::configure(on);
  GridMapTestAccess::productionGeometry(off);GridMapTestAccess::productionGeometry(on);
  ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(on);
  for(unsigned iteration=0;iteration<3;++iteration) {
    const auto stamp=101900000000LL+iteration*10000000LL;
    for(unsigned sensor=0;sensor<2;++sensor) {
      const auto p=withSourceFields(packet(sensor,stamp,iteration+1,3,
          {.05,.05,.05},sensor?Eigen::Vector3d(1.55,.05,.05):Eigen::Vector3d(.55,.05,.05)));
      ASSERT_EQ(GridMapTestAccess::accept(off,p),GridMapTestAccess::accept(on,p));
    }
    GridMapTestAccess::integrate(off);GridMapTestAccess::integrate(on);
    GridMapTestAccess::expectSameEvidence(off,on);
    for(const Eigen::Vector3d &position: {Eigen::Vector3d(.05,.05,.05),Eigen::Vector3d(.55,.05,.05),
                                        Eigen::Vector3d(1.75,.05,.05)}) {
      for(double yaw:{0.,.71,1.57}) {
        const auto a=off.inspectInflateOccupancy(position,yaw,false);
        const auto b=on.inspectInflateOccupancy(position,yaw,true);
        EXPECT_EQ(a.counts,b.counts);
        EXPECT_EQ(off.getInflateOccupancy(position,yaw),on.getInflateOccupancy(position,yaw));
      }
    }
    GridMapTestAccess::expectSameEvidence(off,on); // A read-only query cannot mutate evidence.
  }
  EXPECT_EQ(off.nearFieldDiagnostics().size(),0U);
  EXPECT_GT(on.nearFieldDiagnostics().size(),0U);
}

TEST(ProjectedRaysDiagnostics, DenseNearReturnsDistinguishRayVisitsFromActualVotes) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,withSourceFields(packet(0,101900000000LL,1,3,
      {.05,.05,.05},{.15,.05,.05}))));
  GridMapTestAccess::integrate(map);
  const auto *hit=map.nearFieldDiagnostics().find({1,0,0});
  ASSERT_NE(hit,nullptr);ASSERT_TRUE(hit->last[0]);
  EXPECT_EQ(hit->visits[0],3U);EXPECT_EQ(hit->votes[0],1U);
  EXPECT_EQ(hit->last[0]->metadata.source_index,12002U);
  EXPECT_EQ(hit->last[0]->metadata.ring,22U);
  EXPECT_EQ(hit->last[0]->metadata.scan_stamp_ns,101900000000LL);
  EXPECT_EQ(hit->last[0]->metadata.integration_ns,102000000000LL);
  EXPECT_TRUE(hit->last[0]->metadata.source_fields_available);
  EXPECT_GT(GridMapTestAccess::raw(map,.15),1.);
  const auto *miss=map.nearFieldDiagnostics().find({0,0,0});
  ASSERT_NE(miss,nullptr);ASSERT_TRUE(miss->last[1]);
  EXPECT_EQ(miss->votes[1],1U);
  EXPECT_GE(miss->visits[1],1U);
}

TEST(ProjectedRaysDiagnostics, ConflictingDuplicateOptionalFieldDisablesWitnessMetadataNotAdmission) {
  for(const char *duplicate_name:{"ring","offset_time","source_index","timestamp","source_timestamp","raw_timestamp"}) {
    GridMap off,on;GridMapTestAccess::configure(off);GridMapTestAccess::configure(on);
    ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(on);
    auto p=withSourceFields(packet(0,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05}));
    auto found=std::find_if(p.rays.fields.begin(),p.rays.fields.end(),
        [&](const auto &field){return field.name==duplicate_name;});
    ASSERT_NE(found,p.rays.fields.end());
    auto duplicate=*found;duplicate.offset=0;duplicate.datatype=sensor_msgs::msg::PointField::UINT8;
    p.rays.fields.push_back(duplicate);
    ASSERT_TRUE(GridMapTestAccess::accept(off,p));
    ASSERT_TRUE(GridMapTestAccess::accept(on,p));
    GridMapTestAccess::integrate(off);GridMapTestAccess::integrate(on);
    GridMapTestAccess::expectSameEvidence(off,on);
    const auto *hit=on.nearFieldDiagnostics().find({1,0,0});
    ASSERT_NE(hit,nullptr);ASSERT_TRUE(hit->last[0]);
    EXPECT_FALSE(hit->last[0]->metadata.source_fields_available)<<duplicate_name;
    EXPECT_EQ(hit->last[0]->metadata.scan_stamp_ns,101900000000LL);
    EXPECT_GT(GridMapTestAccess::raw(on,.15),1.);
  }
}

TEST(ProjectedRaysDiagnostics, DetailedCylinderOverlapCountsUniqueWorldVoxelsExactlyOnce) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d body(.05,.05,.05);
  const auto simple=map.inspectInflateOccupancy(body,.63,false);
  const auto detail=map.inspectInflateOccupancy(body,.63,true);
  EXPECT_EQ(simple.counts,detail.counts);
  const auto raw=std::accumulate(detail.counts.begin(),detail.counts.end(),std::size_t{0});
  const auto unique=std::accumulate(detail.unique_counts.begin(),detail.unique_counts.end(),std::size_t{0});
  EXPECT_GT(raw,unique);EXPECT_EQ(detail.voxels.size(),unique);
  std::set<std::array<int,3>> indices;std::size_t memberships=0,both=0;
  for(const auto &voxel:detail.voxels) {
    EXPECT_TRUE(indices.insert({voxel.index.x(),voxel.index.y(),voxel.index.z()}).second);
    EXPECT_GT(voxel.cylinder_mask,0U);EXPECT_LE(voxel.cylinder_mask,3U);
    memberships+=bool(voxel.cylinder_mask&1U)+bool(voxel.cylinder_mask&2U);
    both+=voxel.cylinder_mask==3U;
    EXPECT_EQ(voxel.classification,scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(voxel.native_state,2);
  }
  EXPECT_GT(both,0U);EXPECT_EQ(memberships,raw);
  EXPECT_NE(map.getInflateOccupancy(body,.63),0);
}

TEST(ProjectedRaysDiagnostics, SlideClearsWorldWitnessBeforeCircularBufferSlotReuse) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,
      {-1.55,.05,.05},{-1.85,.05,.05})));
  GridMapTestAccess::integrate(map);
  const Eigen::Vector3i old(-19,0,0), replacement(21,0,0);
  const int address=GridMapTestAccess::address(map,old);
  ASSERT_NE(map.nearFieldDiagnostics().find(old),nullptr);
  GridMapTestAccess::slide(map,{.55,0.,0.});
  EXPECT_EQ(map.nearFieldDiagnostics().find(old),nullptr);
  EXPECT_EQ(map.nearFieldDiagnostics().find(replacement),nullptr);
  EXPECT_EQ(GridMapTestAccess::address(map,replacement),address);
  EXPECT_EQ(GridMapTestAccess::category(map,replacement),scan_planner::RawVoxelDiagnostic::NeverObserved);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101920000000LL,2,1,
      {1.65,.05,.05},{2.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  ASSERT_NE(map.nearFieldDiagnostics().find(replacement),nullptr);
  EXPECT_FALSE(map.nearFieldDiagnostics().find(replacement)->last[0]);
  EXPECT_TRUE(map.nearFieldDiagnostics().find(replacement)->last[2]);
  EXPECT_EQ(map.nearFieldDiagnostics().find(old),nullptr);
}

TEST(ProjectedRaysDiagnostics, LargeWindowJumpAndNewLocalizationIdentityClearAllWitnesses) {
  for(bool relocation:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(map);
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    ASSERT_GT(map.nearFieldDiagnostics().size(),0U);
    const auto generation=map.nearFieldDiagnostics().generation();
    if(relocation) ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new-seed")));
    else GridMapTestAccess::slide(map,{6.,0.,0.});
    EXPECT_EQ(map.nearFieldDiagnostics().size(),0U);
    EXPECT_GT(map.nearFieldDiagnostics().generation(),generation);
    EXPECT_EQ(map.nearFieldDiagnostics().find({15,0,0}),nullptr);
  }
}

TEST(ProjectedRaysDiagnostics, SubthresholdRealHitIsInsufficientNotNeverObservedAndStillBlocked) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionProbabilities(map);
  GridMapTestAccess::productionGeometry(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::Insufficient);
  EXPECT_EQ(GridMapTestAccess::category(map,{2,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  ASSERT_NE(map.nearFieldDiagnostics().find({1,0,0}),nullptr);
  EXPECT_TRUE(map.nearFieldDiagnostics().find({1,0,0})->last[0]);
  const auto detail=map.inspectInflateOccupancy({.05,.05,.05},0.,true);
  std::size_t insufficient=0;
  for(const auto &voxel:detail.voxels) insufficient+=voxel.classification==scan_planner::RawVoxelDiagnostic::Insufficient;
  EXPECT_GE(insufficient,1U);EXPECT_NE(map.getInflateOccupancy({.05,.05,.05},0.),0);
}

TEST(ProjectedRaysDiagnostics, LowOrThinExternalReturnRemainsOccupiedAndOccludedSpaceUnknown) {
  for(double height:{.12,.39}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));GridMapTestAccess::diagnostics(map);
    const Eigen::Vector3d end(.15,.05,height);
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{-.15,.05,height},end)));
    GridMapTestAccess::integrate(map);
    const auto cell=GridMapTestAccess::index(map,end);
    EXPECT_EQ(GridMapTestAccess::category(map,cell),scan_planner::RawVoxelDiagnostic::Occupied);
    const auto behind=GridMapTestAccess::index(map,{.25,.05,height});
    EXPECT_EQ(GridMapTestAccess::category(map,behind),scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(map.nearFieldDiagnostics().find(behind),nullptr);
    const auto detail=map.inspectInflateOccupancy({.05,.05,.35},0.,true);
    EXPECT_GT(detail.unique_counts[1],0U);
    EXPECT_NE(map.getInflateOccupancy({.05,.05,.35},0.),0);
  }
}

TEST(ProjectedRays, DualOriginsDoNotInventFreeSpaceBetweenSensorsOrBeyondHits) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  GridMapTestAccess::integrate(map);EXPECT_EQ(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101950000000LL,2,1,{-.55,.05,.05},{-1.55,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),101.9);
  EXPECT_EQ(GridMapTestAccess::raw(map,.75),-1.);EXPECT_EQ(GridMapTestAccess::raw(map,-.75),-1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.05),-1.);EXPECT_LT(GridMapTestAccess::raw(map,-.25),-1.);
  EXPECT_GT(GridMapTestAccess::raw(map,1.55),1.);EXPECT_GT(GridMapTestAccess::raw(map,-1.55),1.);
  EXPECT_LT(GridMapTestAccess::raw(map,1.75),-1.);EXPECT_LT(GridMapTestAccess::raw(map,-1.75),-1.);
}
TEST(ProjectedRays, NearReturnIsNotDiscardedAndDenseHitsCannotOverflowVotes) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,100000,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::raw(map,.05),-1.);EXPECT_GT(GridMapTestAccess::raw(map,.15),1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.25),-1.);
}
TEST(ProjectedRays, RealHitWinsAgainstCrossingFreeRayInSameIntegration) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.55,.05,.05})));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,2,1,{.05,.05,.05},{1.55,.05,.05})));
  GridMapTestAccess::integrate(map);EXPECT_GT(GridMapTestAccess::raw(map,.55),1.);
}
TEST(ProjectedRays, ContextMismatchAndOldPacketsCannotRefillReplacementMap) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto p=packet();ASSERT_TRUE(GridMapTestAccess::accept(map,p));
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"replacement")));
  EXPECT_EQ(GridMapTestAccess::pending(map),0);EXPECT_FALSE(GridMapTestAccess::accept(map,p));
  p.epoch=2;p.context_sequence=2;p.seed_id="replacement";p.barrier_ns=100000000002ULL;
  EXPECT_TRUE(GridMapTestAccess::accept(map,p));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::counts(map)[1],0U);
  for(unsigned field=0;field<5;++field) {
    auto bad=p;
    if(field==0) bad.session_id="other";
    if(field==1) --bad.epoch;
    if(field==2) --bad.context_sequence;
    if(field==3) --bad.barrier_ns;
    if(field==4) bad.seed_id="other";
    EXPECT_FALSE(GridMapTestAccess::accept(map,bad));
  }
}
TEST(ProjectedRays, BoundedIndependentQueuesRejectReplaysAndExpiredReceipts) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto p=packet();ASSERT_TRUE(GridMapTestAccess::accept(map,p));
  EXPECT_FALSE(GridMapTestAccess::accept(map,p));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101910000000LL,2)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101920000000LL,3)));
  EXPECT_EQ(GridMapTestAccess::pending(map),2);EXPECT_EQ(GridMapTestAccess::drops(map)[0],2U);
  GridMapTestAccess::integrate(map,102300000000LL,102.3);
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
}
TEST(ProjectedRays, OneSourceAndPausedSourceClockDoNotRenewBothSourceLease) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  GridMapTestAccess::integrate(map);ASSERT_GT(map.latestCloudStamp(),0.);
  GridMapTestAccess::integrate(map,102000000000LL,102.6);EXPECT_EQ(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102400000000LL,2),102500000000LL,102.6));
  GridMapTestAccess::integrate(map,102500000000LL,102.6);EXPECT_EQ(map.latestCloudStamp(),0.);
}
TEST(ProjectedRays, MalformedFieldsTimesAndUnboundedPayloadAreRejected) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned variant=0;variant<13;++variant) {
    auto p=packet();
    if(variant==0) p.rays.header.frame_id="odom";
    if(variant==1) p.rays.point_step=12;
    if(variant==2) p.rays.is_bigendian=true;
    if(variant==3) p.rays.fields[3].name="missing_origin";
    if(variant==4) p.rays.fields[3].datatype=sensor_msgs::msg::PointField::FLOAT64;
    if(variant==5) p.rays.fields.push_back(p.rays.fields[0]);
    if(variant==6) p.rays.row_step=1;
    if(variant==7) p.rays.header.stamp=timeAt(101000000000LL);
    if(variant==8) p.acquisition_end=timeAt(102500000000LL);
    if(variant==9) p.alignment_stamp=timeAt(99000000000LL);
    if(variant==10) p.rays.data[28]=2;
    if(variant==11) {const float nan=std::numeric_limits<float>::quiet_NaN();std::memcpy(p.rays.data.data(),&nan,4);}
    if(variant==12) p.rays.width=100001;
    EXPECT_FALSE(GridMapTestAccess::accept(map,p))<<variant;
  }
  GridMapTestAccess::legacy(map);EXPECT_FALSE(GridMapTestAccess::accept(map,packet()));
}
TEST(ProjectedRays, TraversalBudgetCannotReportIncompleteWorkAsComplete) {
  std::size_t budget=2;unsigned visits=0;
  EXPECT_FALSE(scan_planner::visitObservedRay({.05,.05,.05},{1.55,.05,.05},.1,false,budget,
      [&](const Eigen::Vector3i &){++visits;}));
  EXPECT_EQ(budget,0U);EXPECT_EQ(visits,2U);
}
TEST(ProjectedRays, ClippedDistantReturnPreservesNearFreeButNeverCreatesBoundaryHit) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{9.55,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::raw(map,.15),-1.);
  EXPECT_EQ(GridMapTestAccess::raw(map,1.85),-1.);
  EXPECT_EQ(GridMapTestAccess::raw(map,1.95),-1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.15,.25),-1.);
}
TEST(ProjectedRays, MixedSensorPacketAndNonfiniteOriginAreNotPartiallyApplied) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto mixed=packet(0,101900000000LL,1,2);mixed.rays.data[64+28]=1;
  EXPECT_FALSE(GridMapTestAccess::accept(map,mixed));
  auto corrupt=packet();const float inf=std::numeric_limits<float>::infinity();
  std::memcpy(corrupt.rays.data.data()+16,&inf,4);
  EXPECT_FALSE(GridMapTestAccess::accept(map,corrupt));
  GridMapTestAccess::integrate(map);EXPECT_LT(GridMapTestAccess::raw(map,.75),-1.);
}
TEST(ProjectedRays, SameCoordinateContextBarrierStillClearsPendingAndBothLeases) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  GridMapTestAccess::integrate(map);ASSERT_GT(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101910000000LL,2)));
  ASSERT_TRUE(map.applyLocalizationContext(context(1,2)));
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,1.55),-1.01); // New context ACK certifies actual rebuild.
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101920000000LL,3)));
}

static scan_planner::ProjectedRayStatusSchedule::Clock::time_point statusTime(double seconds) {
  return scan_planner::ProjectedRayStatusSchedule::Clock::time_point(
      std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(seconds*1e9))));
}
TEST(ProjectedRayStatus, CompletedIntegrationBypassesDiagnosticHeartbeatDelay) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  ASSERT_TRUE(schedule.due(statusTime(10.),9700000000LL,true,1));
  schedule.markPublished(statusTime(10.),9700000000LL,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.05),9700000000LL,true,1));
  // A newer actual completed integration must not wait until t=10.2.
  EXPECT_TRUE(schedule.due(statusTime(10.05),9800000000LL,true,1));
  schedule.markPublished(statusTime(10.05),9800000000LL,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.10),9800000000LL,true,1));
}
TEST(ProjectedRayStatus, HeartbeatRepeatsOriginalLeaseAndReportsExpiryImmediately) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  const auto source=10000000000LL;
  schedule.markPublished(statusTime(10.2),source,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.399),source,true,1));
  EXPECT_TRUE(schedule.due(statusTime(10.4),source,true,1));
  schedule.markPublished(statusTime(10.4),source,true,1);
  // The original source expires at .5 s, not .5 s after a status heartbeat.
  EXPECT_FALSE(scan_planner::rayStampFresh(source,10510000000LL,.5));
  EXPECT_TRUE(schedule.due(statusTime(10.51),0,false,1));
  schedule.markPublished(statusTime(10.51),0,false,1);
  EXPECT_FALSE(schedule.due(statusTime(10.55),0,false,1));
}
TEST(ProjectedRayStatus, FreshIntegrationDoesNotBecomeStaleOnlyBecauseHeartbeatWasThrottled) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  std::int64_t reported=9680000000LL;
  schedule.markPublished(statusTime(10.),reported,true,1);
  const auto integrated=9780000000LL; // just integrated at 10.10 s, age .32 s
  ASSERT_TRUE(schedule.due(statusTime(10.10),integrated,true,1));
  reported=integrated;schedule.markPublished(statusTime(10.10),reported,true,1);
  // At 10.19 s the old 5 Hz-only notification would have expired (.51 s)
  // although the current actual integration is only .41 s old.
  EXPECT_FALSE(scan_planner::rayStampFresh(9680000000LL,10190000000LL,.5));
  EXPECT_TRUE(scan_planner::rayStampFresh(reported,10190000000LL,.5));
}
TEST(ProjectedRayStatus, ContextChangeAndInvalidityCannotWaitForHeartbeat) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  schedule.markPublished(statusTime(20.),0,false,1);
  EXPECT_TRUE(schedule.due(statusTime(20.01),0,false,2));
  schedule.markPublished(statusTime(20.01),0,false,2);
  EXPECT_TRUE(schedule.due(statusTime(20.02),19900000000LL,true,2));
  schedule.markPublished(statusTime(20.02),19900000000LL,true,2);
  EXPECT_TRUE(schedule.due(statusTime(20.03),0,false,2));
}
