#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "plan_env/grid_map.h"

// No ROS init, node, executor, discovery or transport. Exercise the actual
// GridMap buffers/reset/cache methods against an in-memory map fixture.
struct GridMapTestAccess {
  static void configure(GridMap &map,bool strict=true) {
    auto &p=map.mp_; auto &d=map.md_;
    p.localization_session_id_="session"; p.require_localization_context_=true;
    p.require_observed_free_=strict;
    p.exact_cloud_pose_sync_=true; p.sensor_type_="lidar";
    p.resolution_=.1; p.resolution_inv_=10.; p.map_voxel_num_={8,8,8};
    p.map_origin_idx_.setZero(); map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.; p.clamp_max_log_=2.; p.min_occupancy_log_=1.; p.unknown_flag_=.01;
    p.double_cylinder_radius_=.1; p.obstacles_inflation_z_up=.1; p.obstacles_inflation_z_down=.1;
    p.frame_id_="map"; p.vis_height_=1.;
    d.has_ray_pose_=false; d.ray_pos_.setZero();
    d.occupancy_buffer_.resize(512); d.occupancy_buffer_inflate_.resize(512);
    d.occupancy_buffer_inflate_cnt_.resize(512); d.count_hit_.resize(512);
    d.count_hit_and_miss_.resize(512); d.flag_rayend_.resize(512); d.flag_traverse_.resize(512);
    map.rebuildInflationOffsets(); map.resetAllMapData();
  }
  static void seedEvidence(GridMap &map) {
    auto &d=map.md_;
    std::fill(d.occupancy_buffer_.begin(),d.occupancy_buffer_.end(),map.mp_.clamp_min_log_);
    map.applyOccupancyUpdate({2,2,2},map.mp_.clamp_max_log_);
    EXPECT_EQ(map.observedCylinderStatus({-.15,-.15,-.15}),0);
    EXPECT_GT(map.observed_cylinder_cache_.size(),0U);
    d.count_hit_[0]=1; d.count_hit_and_miss_[0]=2; d.cache_voxel_.push({0,0,0});
    d.proj_points_={{.1,.1,.1}}; d.proj_points_cnt=1;
    d.occ_need_update_=d.use_cloud_update_=d.has_cloud_=d.has_ray_pose_=true;
    map.integrated_cloud_stamp_ns_=map.projected_cloud_stamp_ns_=101000000000;
    map.last_paired_stamp_ns_=101000000000;
    map.pending_clouds_[101100000000]={std::make_shared<sensor_msgs::msg::PointCloud2>(),{}};
    map.pending_poses_[101100000000]={std::make_shared<nav_msgs::msg::Odometry>(),{}};
  }
  static void expectCleared(GridMap &map) {
    const auto &d=map.md_;
    EXPECT_TRUE(std::all_of(d.occupancy_buffer_.begin(),d.occupancy_buffer_.end(),
        [&](double x){return x==map.mp_.clamp_min_log_-map.mp_.unknown_flag_;}));
    for(auto x:d.occupancy_buffer_inflate_) EXPECT_EQ(x,0);
    for(auto x:d.occupancy_buffer_inflate_cnt_) EXPECT_EQ(x,0);
    for(auto x:d.count_hit_) EXPECT_EQ(x,0);
    for(auto x:d.count_hit_and_miss_) EXPECT_EQ(x,0);
    EXPECT_TRUE(d.cache_voxel_.empty()); EXPECT_TRUE(map.pending_clouds_.empty());
    EXPECT_TRUE(map.pending_poses_.empty()); EXPECT_EQ(d.proj_points_cnt,0);
    EXPECT_FALSE(d.occ_need_update_); EXPECT_FALSE(d.has_cloud_); EXPECT_FALSE(d.has_ray_pose_);
    EXPECT_EQ(map.latestCloudStamp(),0.); EXPECT_EQ(map.observed_cylinder_cache_.size(),0U);
    EXPECT_EQ(map.observedCylinderStatus({-.15,-.15,-.15}),2);
  }
  static auto raw(const GridMap &map) {return map.md_.occupancy_buffer_;}
  static void loseSensor(GridMap &map) {map.invalidateCloudPosePairs(102000000000);}
  static auto visual(GridMap &map,bool inflated) {return map.cachedVisualization(inflated);}
  static auto builds(const GridMap &map) {return map.visualization_builds_;}
  static int query(GridMap &map) {return map.observedCylinderStatus({-.15,-.15,-.15});}
  static void deliverDelayedOldPair(GridMap &map,std::int64_t stamp,bool cloud_first) {
    auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto pose=std::make_shared<nav_msgs::msg::Odometry>();
    cloud->header.frame_id=pose->header.frame_id="map";
    cloud->header.stamp.sec=static_cast<std::int32_t>(stamp/1000000000);
    cloud->header.stamp.nanosec=static_cast<std::uint32_t>(stamp%1000000000);
    pose->header.stamp=cloud->header.stamp;
    cloud->width=cloud->height=1;cloud->point_step=cloud->row_step=12;cloud->data.resize(12);
    pose->pose.pose.orientation.w=1.;
    if(cloud_first) {map.cloudCallback(cloud);map.sensorPoseCallback(pose);}
    else {map.sensorPoseCallback(pose);map.cloudCallback(cloud);}
  }
};

static std::string context(unsigned epoch,unsigned sequence,const std::string &seed="seed") {
  return nlohmann::json{{"schema",1},{"session_id","session"},{"epoch",epoch},
      {"sequence",sequence},{"seed_id",seed},{"barrier_ns",100000000000ULL+sequence*1000000000ULL}}.dump();
}

TEST(MapContext, EpochAndSeedResetActualOccupiedFreeInflatedAndPendingEvidence) {
  for(bool seed_only:{false,true}) {
    GridMap map; GridMapTestAccess::configure(map);
    ASSERT_TRUE(map.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::seedEvidence(map);
    const auto revision=map.occupancyRevision();
    ASSERT_TRUE(map.applyLocalizationContext(context(seed_only?1:2,2,"new-seed")));
    EXPECT_GT(map.occupancyRevision(),revision);
    GridMapTestAccess::expectCleared(map);
  }
}

TEST(MapContext, OrdinaryLossAndRepeatedAcknowledgementRetainHistoricalContent) {
  GridMap map; GridMapTestAccess::configure(map);
  const auto first=context(1,1); ASSERT_TRUE(map.applyLocalizationContext(first));
  GridMapTestAccess::seedEvidence(map);
  const auto old=GridMapTestAccess::raw(map);
  const auto revision=map.occupancyRevision();
  GridMapTestAccess::loseSensor(map);
  EXPECT_EQ(GridMapTestAccess::raw(map),old); EXPECT_EQ(map.latestCloudStamp(),0.);
  EXPECT_TRUE(map.applyLocalizationContext(first));
  EXPECT_EQ(map.occupancyRevision(),revision); EXPECT_EQ(GridMapTestAccess::raw(map),old);
}

TEST(MapContext, LegacyResetRetainsItsExplicitUnknownSpacePolicy) {
  GridMap map; GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::seedEvidence(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"replacement-seed")));
  // Only live strict queries demand measured-free evidence. The legacy preview
  // still regards the cleared unknown prior as non-occupied, not motion approval.
  EXPECT_EQ(GridMapTestAccess::query(map),0);
  EXPECT_EQ(map.latestCloudStamp(),0.);
}

TEST(MapContext, NonfinitePoseOrYawCannotBecomeObservedFree) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::seedEvidence(map);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(map.getInflateOccupancy({nan,0,0},0.),-1);
  EXPECT_EQ(map.getInflateOccupancy({0,nan,0},0.),-1);
  EXPECT_EQ(map.getInflateOccupancy({0,0,0},nan),-1);
}

TEST(MapContext, DelayedContextAndMalformedPayloadCannotRevertOrClearCurrentMap) {
  GridMap map; GridMapTestAccess::configure(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2)));
  GridMapTestAccess::seedEvidence(map); const auto old=GridMapTestAccess::raw(map);
  EXPECT_FALSE(map.applyLocalizationContext(context(1,3)));
  EXPECT_FALSE(map.applyLocalizationContext(context(2,1)));
  EXPECT_FALSE(map.applyLocalizationContext(context(2,2,"different-seed")));
  EXPECT_FALSE(map.applyLocalizationContext("{}"));
  EXPECT_EQ(GridMapTestAccess::raw(map),old);
}

TEST(MapContext, ResetBeforeDelayedFutureStampedOldPairCannotRefillNewMap) {
  for(bool cloud_first:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);
    ASSERT_TRUE(map.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::seedEvidence(map);
    // Bridge reset at 102 s had already published an old 102.1 s sample.
    // Context delivery overtakes both sensor topics; barrier includes that
    // future stamp instead of only the reset wall-clock time.
    auto replacement=nlohmann::json::parse(context(2,2,"new-seed"));
    replacement["barrier_ns"]=102100000001ULL;
    ASSERT_TRUE(map.applyLocalizationContext(replacement.dump()));
    GridMapTestAccess::deliverDelayedOldPair(map,102100000000LL,cloud_first);
    GridMapTestAccess::expectCleared(map);
  }
}

TEST(MapVisualization, ReusesBothCloudsUntilRealMapContentChangesThenClearsOnEpoch) {
  GridMap map; GridMapTestAccess::configure(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(1,1))); GridMapTestAccess::seedEvidence(map);
  const auto raw=GridMapTestAccess::visual(map,false), inflated=GridMapTestAccess::visual(map,true);
  EXPECT_GT(raw.width,0U); EXPECT_GT(inflated.width,0U);
  const auto builds=GridMapTestAccess::builds(map);
  for(int i=0;i<10;++i) {
    EXPECT_EQ(GridMapTestAccess::visual(map,false).data,raw.data);
    EXPECT_EQ(GridMapTestAccess::visual(map,true).data,inflated.data);
  }
  EXPECT_EQ(GridMapTestAccess::builds(map),builds);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2)));
  EXPECT_EQ(GridMapTestAccess::visual(map,false).width,0U);
  EXPECT_EQ(GridMapTestAccess::visual(map,true).width,0U);
  EXPECT_EQ(GridMapTestAccess::builds(map)[0],builds[0]+1);
  EXPECT_EQ(GridMapTestAccess::builds(map)[1],builds[1]+1);
}
