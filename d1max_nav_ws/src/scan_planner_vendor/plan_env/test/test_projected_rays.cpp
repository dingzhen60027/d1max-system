#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "plan_env/grid_map.h"
#include "plan_env/observed_ray.hpp"
#include <numeric>
#include <set>

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
};
static builtin_interfaces::msg::Time timeAt(std::int64_t ns) {
  builtin_interfaces::msg::Time out;out.sec=ns/1000000000LL;out.nanosec=ns%1000000000LL;return out;
}
static std::string context(unsigned epoch=1,unsigned sequence=1,const std::string &seed="seed") {
  return nlohmann::json{{"schema",1},{"session_id","session"},{"epoch",epoch},{"sequence",sequence},
    {"seed_id",seed},{"barrier_ns",100000000000ULL+sequence}}.dump();
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
