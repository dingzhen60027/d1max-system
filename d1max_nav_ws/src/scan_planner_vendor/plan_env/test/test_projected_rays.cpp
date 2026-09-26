#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "plan_env/grid_map.h"
#include "plan_env/observed_ray.hpp"

// Actual map integration, memory only: no ROS context/node/executor/transport.
struct GridMapTestAccess {
  static void configure(GridMap &map) {
    auto &p=map.mp_;auto &d=map.md_;
    p.use_projected_rays_=p.require_observed_free_=p.require_localization_context_=true;
    p.localization_session_id_="session";p.sensor_type_="lidar";p.frame_id_="map";
    p.map_sliding_en_=false;p.local_update_range_={1.9,1.9,1.9};
    p.resolution_=.1;p.resolution_inv_=10.;p.map_voxel_num_={40,40,40};
    p.map_origin_idx_.setZero();map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.prob_hit_log_=3.;p.prob_miss_log_=-.5;p.max_ray_length_=4.;
    p.double_cylinder_radius_=.1;p.obstacles_inflation_z_up=.1;p.obstacles_inflation_z_down=.1;
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
  static double raw(GridMap &map,double x,double y=.05,double z=.05) {
    Eigen::Vector3i cell;map.posToIndex({x,y,z},cell);return map.md_.occupancy_buffer_[map.toAddress(cell)];
  }
  static auto counts(GridMap &map) {return map.ray_integrations_;}
  static auto drops(GridMap &map) {return map.ray_drops_;}
  static auto pending(GridMap &map) {return !!map.pending_rays_[0]+!!map.pending_rays_[1];}
  static void legacy(GridMap &map) {map.mp_.use_projected_rays_=false;}
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
  EXPECT_GT(GridMapTestAccess::raw(map,1.55),1.); // Historic data is not current support.
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
