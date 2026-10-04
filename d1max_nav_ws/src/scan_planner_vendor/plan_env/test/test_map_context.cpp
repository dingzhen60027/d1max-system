#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "plan_env/grid_map.h"
#include "plan_env/collision_snapshot_pool.hpp"

TEST(CollisionSnapshotPool, EmptyPoolHasNoInventedMap) {
  scan_planner::CollisionSnapshotPool pool;
  EXPECT_FALSE(pool.borrowLatest());
}

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
    p.double_cylinder_radius_=.1; p.double_cylinder_offset_=.1;
    p.obstacles_inflation_z_up=.1; p.obstacles_inflation_z_down=.1;
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
  static void diagnosticFusion(GridMap& map,std::uint64_t sequence) {
    map.fusion_timing_={sequence,101000000000,101090000000,1000000000,1090000000,
      {{100900000000,100800000000}}};
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
  static void freeAges(GridMap &map,std::int64_t source,std::int64_t now) {
    map.mp_.use_projected_rays_=true;
    map.free_observation_stamps_.assign(map.md_.occupancy_buffer_.size(),source);
    map.ray_tick_clock_ns_=map.ray_tick_effective_ns_=now;
    map.ray_tick_receipt_=std::chrono::steady_clock::now();
    map.ray_query_clock_ns_=now;
  }
  static void loseSensor(GridMap &map) {map.invalidateCloudPosePairs(102000000000);}
  static auto visual(GridMap &map,bool inflated) {return map.cachedVisualization(inflated);}
  static auto builds(const GridMap &map) {return map.visualization_builds_;}
  static void visualizationFixture(GridMap &map,const Eigen::Vector3i &origin,
                                   double ray_z,bool have_ray) {
    map.mp_.map_origin_idx_=origin;map.updateMapBoundaryFromIndex();
    map.mp_.vis_height_=.15;
    map.md_.has_ray_pose_=have_ray;map.md_.ray_pos_.z()=ray_z;
    for(std::size_t i=0;i<map.md_.occupancy_buffer_.size();++i) {
      // Include equality at the raw threshold and different inflated content.
      map.md_.occupancy_buffer_[i]=static_cast<double>(i%4)-1.;
      map.md_.occupancy_buffer_inflate_[i]=i%3==0;
    }
    ++map.occupancy_revision_;
  }
  static std::vector<Eigen::Vector3f> legacyVisual(GridMap &map,bool inflated) {
    const int clip=map.md_.has_ray_pose_?static_cast<int>(std::floor(
        (map.md_.ray_pos_.z()+map.mp_.vis_height_)*map.mp_.resolution_inv_-.5)):
        std::numeric_limits<int>::max();
    std::vector<Eigen::Vector3f> result;
    for(int x=map.mp_.map_bound_min_idx_.x();x<=map.mp_.map_bound_max_idx_.x();++x)
      for(int y=map.mp_.map_bound_min_idx_.y();y<=map.mp_.map_bound_max_idx_.y();++y)
        for(int z=map.mp_.map_bound_min_idx_.z();z<=map.mp_.map_bound_max_idx_.z();++z) {
          const int address=map.toAddress(x,y,z);
          if(inflated?map.md_.occupancy_buffer_inflate_[address]==0:
             map.md_.occupancy_buffer_[address]<map.mp_.min_occupancy_log_) continue;
          Eigen::Vector3d point;map.indexToPos({x,y,z},point);
          if(z>clip) continue;
          result.push_back(point.cast<float>());
        }
    return result;
  }
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

TEST(CollisionSnapshotPool, NativeQueriesMatchWriterAndRemainIndependent) {
  for (bool strict:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer,strict);GridMapTestAccess::seedEvidence(writer);
    scan_planner::CollisionSnapshotPool pool;
    ASSERT_TRUE(pool.publish(writer,101000000000,std::chrono::steady_clock::now()));
    auto snapshot=pool.borrowLatest();ASSERT_TRUE(snapshot);
    EXPECT_FALSE(pool.borrowLatest()); // Native caches are not shared across readers.
    for (double x:{-.25,-.15,.05,.25}) for (double y:{-.25,-.05,.25})
      for (double yaw:{0.,.7,1.57}) {
        const Eigen::Vector3d p(x,y,.05);
        EXPECT_EQ(snapshot->getInflateOccupancy(p,yaw),writer.getInflateOccupancy(p,yaw));
        EXPECT_EQ(snapshot->inspectInflateOccupancy(p,yaw).state(),writer.inspectInflateOccupancy(p,yaw).state());
      }
    const auto saved=GridMapTestAccess::raw(*snapshot);
    writer.resetBuffer();
    EXPECT_EQ(GridMapTestAccess::raw(*snapshot),saved);
    EXPECT_EQ(snapshot->latestCloudStampNs(),101000000000);
  }
}
TEST(CollisionSnapshotPool, ThreeRetainedLeasesBoundMemoryAndNeverOverwriteReader) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;
  std::array<GridMap::Ptr,3> readers;
  for (auto &reader:readers) {
    ASSERT_TRUE(pool.publish(writer,101000000000,std::chrono::steady_clock::now()));
    reader=pool.borrowLatest();ASSERT_TRUE(reader);
  }
  EXPECT_FALSE(pool.publish(writer,101000000000,std::chrono::steady_clock::now()));
  readers[1].reset();
  EXPECT_TRUE(pool.publish(writer,101000000000,std::chrono::steady_clock::now()));
}
TEST(CollisionSnapshotPool, CopyDoesNotRenewExpiredFreeEvidence) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  GridMapTestAccess::freeAges(writer,100000000000,101000000000);
  GridMap snapshot;
  writer.copyCollisionSnapshotTo(snapshot,101000000000,std::chrono::steady_clock::now());
  EXPECT_EQ(snapshot.getInflateOccupancy({-.15,-.15,-.15},0.),2);
  EXPECT_EQ(snapshot.latestCloudStampNs(),writer.latestCloudStampNs());
}

TEST(CollisionSnapshotPool, SolverReservationCannotStarveTwoValidatorSlots) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000,captured));auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  EXPECT_FALSE(pool.publishSolver(writer,101000000000,captured));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000,captured));auto first=pool.borrowValidation();ASSERT_TRUE(first);
  ASSERT_TRUE(pool.publishValidation(writer,101000000000,captured));auto second=pool.borrowValidation();ASSERT_TRUE(second);
  EXPECT_NE(first.get(),second.get());EXPECT_NE(first.get(),solver.get());
  EXPECT_FALSE(pool.publishValidation(writer,101000000000,captured));
  first.reset();EXPECT_TRUE(pool.publishValidation(writer,101000000000,captured));
  EXPECT_FALSE(pool.publishSolver(writer,101000000000,captured));
}
TEST(CollisionSnapshotPool, SlowAdmissionAndSolverCannotShareMutableQueryCache) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000,captured));
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_FALSE(pool.borrowSolver());EXPECT_FALSE(pool.publishSolver(writer,101000000000,captured));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000,captured));
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);
  EXPECT_NE(fast.get(),admission.get());
  admission.reset();ASSERT_TRUE(pool.publishSolver(writer,101000000000,captured));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  EXPECT_NE(solver.get(),fast.get());
  // No copied querying pointer or fourth allocation is permitted by this API.
}

TEST(CollisionSnapshotPool, ContendingNativeReadersAlternateWithoutStealingFastSlots) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000,captured));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);const auto third=solver.get();
  for(unsigned cycle=0;cycle<100;++cycle) {
    // Worst arrival ordering: solver retries FIRST after holding its slot,
    // while admission registered real work during that hold.
    EXPECT_FALSE(pool.borrowAdmission());EXPECT_FALSE(pool.borrowSolver());
    solver.reset();
    EXPECT_FALSE(pool.borrowSolver()); // Cannot overtake waiting admission.
    auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);EXPECT_EQ(admission.get(),third);
    EXPECT_FALSE(pool.borrowSolver());
    ASSERT_TRUE(pool.publishValidation(writer,101000000000,captured));
    auto fast=pool.borrowValidation();ASSERT_TRUE(fast);EXPECT_NE(fast.get(),third);
    EXPECT_FALSE(pool.publishSolver(writer,101000000000,captured));
    admission.reset();
    EXPECT_FALSE(pool.borrowAdmission()); // Cannot starve queued solver either.
    solver=pool.borrowSolver();ASSERT_TRUE(solver);EXPECT_EQ(solver.get(),third);
  }
}

TEST(CollisionSnapshotPool, AcquisitionRetriesAfterReleaseWithoutCopyingOverBusyReaderOrRenewingSource) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  const auto deadline=captured+std::chrono::milliseconds(400);
  ASSERT_TRUE(pool.publishSolver(writer,101000000000,captured));
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);const auto third=admission.get();
  const auto original=GridMapTestAccess::raw(*admission);
  EXPECT_FALSE(pool.acquireSolver(writer,101200000000,deadline));
  EXPECT_EQ(GridMapTestAccess::raw(*admission),original);
  admission.reset();
  auto solver=pool.acquireSolver(writer,101200000000,deadline);ASSERT_TRUE(solver);
  EXPECT_EQ(solver.get(),third);EXPECT_EQ(solver->latestCloudStampNs(),101000000000);
  EXPECT_FALSE(pool.borrowAdmission()); // Query caches remain exclusive.
  ASSERT_TRUE(pool.publishValidation(writer,101200000000,captured));
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);EXPECT_NE(fast.get(),third);
}

TEST(CollisionSnapshotPool, AcquisitionRespectsAdmissionTurnBeforeCopyAndCancellationWithdrawsIntent) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  const auto deadline=captured+std::chrono::milliseconds(400);
  auto solver=pool.acquireSolver(writer,101000000000,deadline);ASSERT_TRUE(solver);
  EXPECT_FALSE(pool.borrowAdmission());solver.reset();
  writer.resetBuffer();
  EXPECT_FALSE(pool.acquireSolver(writer,101200000000,deadline)); // Admission owns next quota.
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_EQ(admission->latestCloudStampNs(),101000000000); // Denied solver did not copy newer writer.
  pool.cancelSolverRequest();admission.reset();
  auto next_admission=pool.borrowAdmission();ASSERT_TRUE(next_admission);
  EXPECT_FALSE(pool.acquireSolver(writer,101200000000,captured-std::chrono::milliseconds(1)));
  EXPECT_EQ(next_admission->latestCloudStampNs(),101000000000);
}

TEST(SnapshotReadArbiter, ExpiredOrCancelledRequestsDoNotReserveTheThirdSlot) {
  using Arbiter=scan_planner::SnapshotReadArbiter;using Reader=Arbiter::Reader;
  const auto start=Arbiter::Time{};Arbiter arbiter;
  arbiter.request(Reader::Solver,start+std::chrono::milliseconds(400),start);
  ASSERT_TRUE(arbiter.grant(Reader::Solver,start,true));
  arbiter.request(Reader::Admission,Arbiter::Time::max(),start);
  arbiter.request(Reader::Solver,start+std::chrono::milliseconds(400),start);
  EXPECT_FALSE(arbiter.grant(Reader::Admission,start,false));
  EXPECT_FALSE(arbiter.grant(Reader::Solver,start,true));
  ASSERT_TRUE(arbiter.grant(Reader::Admission,start,true));
  arbiter.request(Reader::Admission,Arbiter::Time::max(),start);
  // This same original solver deadline cannot be refreshed by cache release,
  // a map copy or admission finishing. Expiry withdraws it deterministically.
  const auto expired=start+std::chrono::milliseconds(401);
  EXPECT_FALSE(arbiter.pending(Reader::Solver,expired));
  ASSERT_TRUE(arbiter.grant(Reader::Admission,expired,true));
  arbiter.request(Reader::Admission,Arbiter::Time::max(),expired);
  arbiter.cancel(Reader::Admission);
  arbiter.request(Reader::Solver,expired+std::chrono::milliseconds(400),expired);
  ASSERT_TRUE(arbiter.grant(Reader::Solver,expired,true));
}

TEST(SnapshotReadArbiter, BoundedFsmRetryAfterSlotReleaseDoesNotRequireAChangedMapOrRenewOldDeadline) {
  using Arbiter=scan_planner::SnapshotReadArbiter;using Reader=Arbiter::Reader;
  const auto start=Arbiter::Time{};Arbiter arbiter;
  arbiter.request(Reader::Admission,Arbiter::Time::max(),start);
  ASSERT_TRUE(arbiter.grant(Reader::Admission,start,true)); // The slow reader holds slot 2.
  arbiter.request(Reader::Solver,start+std::chrono::milliseconds(400),start);
  EXPECT_FALSE(arbiter.grant(Reader::Solver,start,false));
  const auto retry=start+std::chrono::milliseconds(500);
  EXPECT_FALSE(arbiter.pending(Reader::Solver,retry)); // Old round has expired, not renewed.
  EXPECT_FALSE(arbiter.grant(Reader::Solver,retry,true)); // Release is not a fresh solve request.
  arbiter.request(Reader::Solver,retry+std::chrono::milliseconds(400),retry);
  EXPECT_FALSE(arbiter.grant(Reader::Solver,retry,false)); // Cannot bypass a still-pinned slot.
  EXPECT_TRUE(arbiter.grant(Reader::Solver,retry,true)); // Same map/pose; releasing it is sufficient.
}

TEST(CollisionSnapshotPool, StrictProofModeCannotModifyTheLiveWriter) {
  GridMap writer;GridMapTestAccess::configure(writer);
  EXPECT_THROW(writer.requireObservedSnapshot(),std::logic_error);
  scan_planner::CollisionSnapshotPool pool;
  ASSERT_TRUE(pool.publishValidation(writer,101000000000,std::chrono::steady_clock::now()));
  auto proof=pool.borrowValidation();ASSERT_TRUE(proof);proof->requireObservedSnapshot();
  EXPECT_TRUE(proof->requiresObservedFree());
}

TEST(CollisionSnapshotPool, PublicationAndBorrowDiagnosticsStayBoundToTheActualSnapshot) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  GridMapTestAccess::diagnosticFusion(writer,19);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishValidation(writer,101090000000,now));
  scan_planner::CollisionSnapshotPool::PublicationTiming first_timing,second_timing,last_timing;
  auto first=pool.borrowValidation(&first_timing);ASSERT_TRUE(first);
  EXPECT_EQ(first_timing.fusion.sequence,19U);
  EXPECT_EQ(first_timing.fusion.begin_ns,101000000000);
  EXPECT_EQ(first_timing.fusion.end_ns,101090000000);
  EXPECT_EQ(first_timing.fusion.source_stamps[1],100800000000);
  EXPECT_GE(first_timing.end_steady_ns,first_timing.begin_steady_ns);
  GridMapTestAccess::diagnosticFusion(writer,20);
  ASSERT_TRUE(pool.publishValidation(writer,101190000000,now));
  auto second=pool.borrowValidation(&second_timing);ASSERT_TRUE(second);
  EXPECT_EQ(second_timing.fusion.sequence,20U);
  EXPECT_EQ(first->fusionTiming().sequence,19U); // Writer update cannot relabel a reader.
  EXPECT_FALSE(pool.publishValidation(writer,101200000000,now));
  first.reset();ASSERT_TRUE(pool.publishValidation(writer,101210000000,now));
  auto last=pool.borrowValidation(&last_timing);ASSERT_TRUE(last);
  EXPECT_EQ(last_timing.publication_misses,1U);
  EXPECT_GT(last_timing.publication_sequence,second_timing.publication_sequence);
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

TEST(MapContext, HigherSequenceSameIdentityRebuildsBeforeAcknowledgement) {
  GridMap map;GridMapTestAccess::configure(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::seedEvidence(map);
  const auto revision=map.occupancyRevision();
  ASSERT_TRUE(map.applyLocalizationContext(context(1,2)));
  EXPECT_GT(map.occupancyRevision(),revision);
  GridMapTestAccess::expectCleared(map);
  const auto cleared_revision=map.occupancyRevision();
  ASSERT_TRUE(map.applyLocalizationContext(context(1,2)));
  EXPECT_EQ(map.occupancyRevision(),cleared_revision);
}

TEST(MapContext, LegacyResetRetainsItsExplicitUnknownSpacePolicy) {
  GridMap map; GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::seedEvidence(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"replacement-seed")));
  // Official collision queries read the cleared inflation buffer. Raw evidence
  // remains unknown; clearing the map does not manufacture observations.
  const auto raw=GridMapTestAccess::raw(map);
  EXPECT_EQ(map.getInflateOccupancy({-.15,-.15,-.15},0.),0);
  EXPECT_EQ(GridMapTestAccess::query(map),2);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
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

TEST(MapVisualization, ExactLegacyPointsAndOrderAcrossRingWrapsAndHeightClips) {
  GridMap map;GridMapTestAccess::configure(map);
  for(const Eigen::Vector3i &origin:std::vector<Eigen::Vector3i>{
      {0,0,0},{-17,9,-11},{21,-24,15},{-8,-8,-8}})
    for(double ray_z:{-20.,-.25,0.,.55,20.})
      for(bool have_ray:{false,true}) {
        GridMapTestAccess::visualizationFixture(map,origin,ray_z,have_ray);
        for(bool inflated:{false,true}) {
          const auto expected=GridMapTestAccess::legacyVisual(map,inflated);
          const auto actual=GridMapTestAccess::visual(map,inflated);
          ASSERT_EQ(actual.width,expected.size());
          EXPECT_EQ(actual.header.frame_id,"map");
          if(expected.empty()) continue;
          sensor_msgs::PointCloud2ConstIterator<float> x(actual,"x"),y(actual,"y"),z(actual,"z");
          for(const auto &point:expected) {
            EXPECT_EQ(*x,point.x());EXPECT_EQ(*y,point.y());EXPECT_EQ(*z,point.z());
            ++x;++y;++z;
          }
        }
      }
}
