#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <atomic>
#include <future>
#include <set>
#include <thread>
#include "plan_env/grid_map.h"
#include "plan_env/collision_snapshot_pool.hpp"

TEST(CollisionSnapshotPool, EmptyPoolHasNoInventedMap) {
  scan_planner::CollisionSnapshotPool pool;
  EXPECT_FALSE(pool.borrowLatest());
  EXPECT_FALSE(pool.borrowOwnerValidation());
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
  static void sourceEvidence(GridMap& map,std::int64_t source) {
    map.integrated_cloud_stamp_ns_=source;
    map.ray_integrated_stamps_={{source-1000,source}};
  }
  static void clockFault(GridMap& map,bool fault=true) {map.ray_clock_fault_=fault;}
  static void renderOnly(GridMap& map,bool enabled) {map.visualization_only_=enabled;}
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
  static std::size_t inflationCounterBytes(const GridMap& map) {
    return map.md_.occupancy_buffer_inflate_cnt_.capacity()*sizeof(int);
  }
  static std::size_t writerCounterBytes(const GridMap& map) {
    return (map.md_.occupancy_buffer_inflate_cnt_.size()+
        map.md_.planning_occupancy_buffer_inflate_cnt_.size())*sizeof(int);
  }
  static void freeAges(GridMap &map,std::int64_t source,std::int64_t now) {
    map.mp_.use_projected_rays_=true;
    map.free_observation_stamps_.assign(map.md_.occupancy_buffer_.size(),source);
    map.ray_tick_clock_ns_=map.ray_tick_effective_ns_=now;
    map.ray_tick_receipt_=std::chrono::steady_clock::now();
    map.ray_query_clock_ns_=now;
  }
  static void candidateAndProofEvidence(GridMap &map) {
    // A real native writer fixture with a measured-free start region, an
    // unobserved candidate region, and an independent occupied voxel. The
    // source begin predates query/copy time and follows the context barrier.
    map.mp_.double_cylinder_offset_=0.;
    for(int x=-4;x<0;++x)for(int y=-4;y<4;++y)for(int z=-4;z<4;++z)
      map.applyOccupancyUpdate({x,y,z},map.mp_.clamp_min_log_);
    map.applyOccupancyUpdate({2,2,2},map.mp_.clamp_max_log_);
    freeAges(map,101900000000LL,102000000000LL);
    map.integrated_cloud_stamp_ns_=101900000000LL;
    map.ray_integrated_stamps_.fill(101900000000LL);
  }
  static void freeStamp(GridMap &map,const Eigen::Vector3i &cell,std::int64_t source) {
    map.free_observation_stamps_.at(map.toAddress(cell))=source;
  }
  static void elapseSnapshot(GridMap &map,std::chrono::nanoseconds elapsed) {
    ASSERT_TRUE(map.collision_snapshot_);
    map.snapshot_captured_-=elapsed;
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

TEST(CollisionSnapshotPool, ThirdSlotSeparatesOfficialCandidateFromStrictUnknownExecutionProof) {
  for(const bool strict_writer:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer,strict_writer);
    ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::candidateAndProofEvidence(writer);
    const auto original=GridMapTestAccess::raw(writer);
    scan_planner::CollisionSnapshotPool pool;
    auto snapshot=pool.acquireSolver(writer,102000000000LL,
      std::chrono::steady_clock::now()+std::chrono::milliseconds(400));
    ASSERT_TRUE(snapshot);
    EXPECT_FALSE(pool.borrowAdmission()); // One exclusive native query lease.
    snapshot->requireObservedSnapshot();
    const Eigen::Vector3d start(-.25,-.15,-.15),unobserved(.15,-.15,-.15),occupied(.25,.25,.25);
    EXPECT_EQ(snapshot->getPlanningInflateOccupancy(start,0.),0);
    EXPECT_EQ(snapshot->getPlanningInflateOccupancy(unobserved,0.),0);
    EXPECT_EQ(snapshot->getPlanningInflateOccupancy(occupied,0.),1);
    snapshot->beginObservedProof();
    EXPECT_EQ(snapshot->getInflateOccupancy(start,0.),0);
    EXPECT_EQ(snapshot->getInflateOccupancy(unobserved,0.),2);
    EXPECT_EQ(snapshot->getInflateOccupancy(occupied,0.),1);
    EXPECT_EQ(snapshot->integratedRaySourceStamp(0),101900000000LL);
    EXPECT_EQ(snapshot->integratedRaySourceStamp(1),101900000000LL);
    EXPECT_EQ(snapshot->latestCloudStampNs(),101900000000LL);
    EXPECT_EQ(snapshot->observedProofDeadlineNs(),102400000000LL);
    EXPECT_EQ(GridMapTestAccess::raw(*snapshot),original);
    GridMapTestAccess::elapseSnapshot(*snapshot,std::chrono::milliseconds(600));
    snapshot->beginObservedProof();
    EXPECT_EQ(snapshot->getInflateOccupancy(start,0.),2);
    EXPECT_EQ(snapshot->getInflateOccupancy(unobserved,0.),2);
    EXPECT_EQ(snapshot->getInflateOccupancy(occupied,0.),1);
    EXPECT_EQ(snapshot->getPlanningInflateOccupancy(unobserved,0.),0);
    EXPECT_EQ(snapshot->integratedRaySourceStamp(0),101900000000LL);
    EXPECT_EQ(GridMapTestAccess::raw(*snapshot),original);
  }
}

TEST(CollisionSnapshotPool, OwnerSnapshotCanSeedSolverWithoutZeroClockOrRenewedEvidence) {
  GridMap writer;GridMapTestAccess::configure(writer,true);
  ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::candidateAndProofEvidence(writer);
  GridMap owner,solver,repeated;
  writer.copyCollisionSnapshotTo(owner,102000000000LL,std::chrono::steady_clock::now());
  ASSERT_TRUE(owner.collisionSnapshotSourceFresh());
  owner.copyCollisionSnapshotTo(solver,102000000000LL,std::chrono::steady_clock::now());
  ASSERT_TRUE(solver.collisionSnapshotSourceFresh());
  EXPECT_EQ(solver.localizationContextSequence(),owner.localizationContextSequence());
  EXPECT_EQ(solver.occupancyRevision(),owner.occupancyRevision());
  EXPECT_EQ(GridMapTestAccess::raw(solver),GridMapTestAccess::raw(owner));
  EXPECT_EQ(solver.integratedRaySourceStamp(0),101900000000LL);
  EXPECT_EQ(solver.integratedRaySourceStamp(1),101900000000LL);
  EXPECT_EQ(solver.latestCloudStampNs(),101900000000LL);
  solver.requireObservedSnapshot();solver.beginObservedProof();
  EXPECT_EQ(solver.getInflateOccupancy({-.25,-.15,-.15},0.),0);
  EXPECT_EQ(solver.getInflateOccupancy({.25,.25,.25},0.),1);
  const auto original_deadline=solver.observedProofDeadlineNs();
  EXPECT_EQ(original_deadline,102400000000LL);
  GridMapTestAccess::elapseSnapshot(owner,std::chrono::milliseconds(600));
  // A paused source clock and a new copy/lease cannot make these same rays
  // young again. The old native occupied evidence must nevertheless remain.
  owner.copyCollisionSnapshotTo(solver,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_FALSE(solver.collisionSnapshotSourceFresh());
  solver.requireObservedSnapshot();solver.beginObservedProof();
  EXPECT_EQ(solver.getInflateOccupancy({-.25,-.15,-.15},0.),2);
  EXPECT_EQ(solver.getInflateOccupancy({.25,.25,.25},0.),1);
  solver.copyCollisionSnapshotTo(repeated,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_FALSE(repeated.collisionSnapshotSourceFresh());
  EXPECT_EQ(repeated.latestCloudStampNs(),101900000000LL);
  EXPECT_EQ(repeated.integratedRaySourceStamp(0),101900000000LL);
  repeated.requireObservedSnapshot();repeated.beginObservedProof();
  EXPECT_EQ(repeated.getInflateOccupancy({-.25,-.15,-.15},0.),2);
  EXPECT_EQ(repeated.getInflateOccupancy({.25,.25,.25},0.),1);
}

TEST(CollisionSnapshotPool, OwnerSnapshotCloneKeepsClockFaultAndContextReset) {
  GridMap writer;GridMapTestAccess::configure(writer,true);
  ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::candidateAndProofEvidence(writer);
  GridMapTestAccess::clockFault(writer);
  GridMap owner,solver;
  writer.copyCollisionSnapshotTo(owner,102000000000LL,std::chrono::steady_clock::now());
  owner.copyCollisionSnapshotTo(solver,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_FALSE(solver.collisionSnapshotSourceFresh());
  ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"reset-seed")));
  writer.copyCollisionSnapshotTo(owner,102000000000LL,std::chrono::steady_clock::now());
  owner.copyCollisionSnapshotTo(solver,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_EQ(solver.localizationContextSequence(),2U);
  EXPECT_EQ(solver.latestCloudStampNs(),0);
  EXPECT_EQ(solver.integratedRaySourceStamp(0),0);
  EXPECT_EQ(solver.integratedRaySourceStamp(1),0);
  EXPECT_FALSE(solver.collisionSnapshotSourceFresh());
  EXPECT_EQ(GridMapTestAccess::raw(solver),GridMapTestAccess::raw(writer));
}

TEST(CollisionSnapshotPool, EachObservedProofUsesOnlyItsOwnQueriedSourceDeadline) {
  GridMap writer;GridMapTestAccess::configure(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::candidateAndProofEvidence(writer);
  GridMapTestAccess::freeStamp(writer,{-1,-2,-2},101950000000LL);
  scan_planner::CollisionSnapshotPool pool;
  auto snapshot=pool.acquireSolver(writer,102000000000LL,
    std::chrono::steady_clock::now()+std::chrono::milliseconds(400));
  ASSERT_TRUE(snapshot);snapshot->requireObservedSnapshot();
  snapshot->beginObservedProof();
  EXPECT_EQ(snapshot->observedRawSnapshotStatus({-3,-2,-2}),0);
  EXPECT_EQ(snapshot->observedProofDeadlineNs(),102400000000LL);
  snapshot->beginObservedProof();
  EXPECT_EQ(snapshot->observedRawSnapshotStatus({-1,-2,-2}),0);
  EXPECT_EQ(snapshot->observedProofDeadlineNs(),102450000000LL);
  EXPECT_EQ(snapshot->observedRawSnapshotStatus({1,-2,-2}),2);
  EXPECT_EQ(snapshot->observedProofDeadlineNs(),102450000000LL);
  EXPECT_EQ(snapshot->observedRawSnapshotStatus({2,2,2}),1);
  EXPECT_EQ(snapshot->observedProofDeadlineNs(),102450000000LL);
}

TEST(CollisionSnapshotPool, ReaderDoesNotRetainWriterOnlyInflationCounters) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  GridMap snapshot;GridMapTestAccess::configure(snapshot); // Reuse a memory-only destination.
  ASSERT_GT(GridMapTestAccess::inflationCounterBytes(writer),0U);
  ASSERT_GT(GridMapTestAccess::inflationCounterBytes(snapshot),0U);
  const auto writer_only_bytes=GridMapTestAccess::writerCounterBytes(writer);
  writer.copyCollisionSnapshotTo(snapshot,101000000000,std::chrono::steady_clock::now());
  EXPECT_EQ(GridMapTestAccess::inflationCounterBytes(snapshot),0U);
  EXPECT_EQ(GridMapTestAccess::writerCounterBytes(snapshot),0U);
  EXPECT_GT(GridMapTestAccess::inflationCounterBytes(writer),0U);
  // Strict evidence and official planning have separate writer-only counters;
  // snapshots retain BOTH committed flag layers but neither counter array.
  EXPECT_EQ(snapshot.collisionSnapshotBytes()+writer_only_bytes,writer.collisionSnapshotBytes());
  for(double x:{-.25,-.15,.05,.25})for(double y:{-.25,-.05,.25})for(double yaw:{0.,.7,1.57}) {
    const Eigen::Vector3d point(x,y,.05);
    EXPECT_EQ(snapshot.getInflateOccupancy(point,yaw),writer.getInflateOccupancy(point,yaw));
    EXPECT_EQ(snapshot.inspectInflateOccupancy(point,yaw,true).counts,writer.inspectInflateOccupancy(point,yaw,true).counts);
  }
  writer.resetBuffer();
  writer.copyCollisionSnapshotTo(snapshot,101000000000,std::chrono::steady_clock::now());
  EXPECT_EQ(GridMapTestAccess::inflationCounterBytes(snapshot),0U);
  EXPECT_EQ(snapshot.getInflateOccupancy({-.15,-.15,-.15},0.),2);
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

TEST(CollisionSnapshotPool, OwnerNeverBorrowsUnpublishedValidationOrSolverQuota) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,now));
  EXPECT_FALSE(pool.borrowOwnerValidation()); // A populated solver is not a validation publication.
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto validator=pool.borrowValidation();ASSERT_TRUE(validator);
  EXPECT_FALSE(pool.borrowOwnerValidation()); // The other validation slot has never been published.
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  EXPECT_NE(solver.get(),validator.get());
  validator.reset();
  EXPECT_FALSE(pool.borrowOwnerValidation()); // First publication remains the fast head even between checks.
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  EXPECT_NE(owner.get(),solver.get());
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);
  EXPECT_NE(fast.get(),owner.get()); // Separate query caches and a protected fast head.
}

TEST(CollisionSnapshotPool, OwnerCannotTakeFastHeadEvenWhenValidatorIsBetweenChecks) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101190000000LL,now));
  scan_planner::CollisionSnapshotPool::PublicationTiming latest_timing,older_timing;
  auto validator=pool.borrowValidation(&latest_timing);ASSERT_TRUE(validator);
  auto owner=pool.borrowOwnerValidation(&older_timing);ASSERT_TRUE(owner);
  EXPECT_NE(owner.get(),validator.get());
  EXPECT_LT(older_timing.publication_sequence,latest_timing.publication_sequence);
  EXPECT_EQ(owner->latestCloudStampNs(),101000000000LL);
  EXPECT_EQ(owner->integratedRaySourceStamp(0),100999999000LL);
  EXPECT_EQ(validator->latestCloudStampNs(),101100000000LL);
  const auto old_source=owner->latestCloudStampNs();
  EXPECT_FALSE(pool.publishValidation(writer,101290000000LL,now));
  EXPECT_EQ(owner->latestCloudStampNs(),old_source); // A publication miss cannot relabel a borrowed map.
  owner.reset();validator.reset();
  auto backup_owner=pool.borrowOwnerValidation();ASSERT_TRUE(backup_owner);
  EXPECT_EQ(backup_owner->latestCloudStampNs(),101000000000LL);
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);
  EXPECT_EQ(fast->latestCloudStampNs(),101100000000LL);
  EXPECT_NE(fast.get(),backup_owner.get());
  const auto diagnostics=pool.readerDiagnostics();
  EXPECT_EQ(diagnostics.owner_borrows,2U);
  EXPECT_EQ(diagnostics.validation_borrows,2U);
  EXPECT_EQ(diagnostics.validation_misses,0U);
  EXPECT_EQ(diagnostics.publication_misses,1U);
}

TEST(CollisionSnapshotPool, OwnerTransactionCannotHideNewerRayEvidenceFromCurrentRenewal) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  // Old-source backup has expired at query time; a genuinely newer complete
  // map is available. Reproduce the owner winning a borrow before renewal.
  GridMapTestAccess::sourceEvidence(writer,100500000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);
  EXPECT_EQ(owner->latestCloudStampNs(),100500000000LL);
  EXPECT_EQ(fast->latestCloudStampNs(),101000000000LL);
  EXPECT_FALSE(owner->integratedCloudFreshAt(101100000000LL));
  EXPECT_TRUE(fast->integratedCloudFreshAt(101100000000LL));
  const auto original_source=fast->integratedRaySourceStamp(0);
  fast.reset();
  for(unsigned i=0;i<10;++i) {
    auto renewal=pool.borrowValidation();ASSERT_TRUE(renewal);
    EXPECT_EQ(renewal->integratedRaySourceStamp(0),original_source);
  }
  // No arrival means no lease renewal: the protected head also expires at
  // its actual original ray deadline, rather than being kept alive by reads.
  auto expired=pool.borrowValidation();ASSERT_TRUE(expired);
  EXPECT_FALSE(expired->integratedCloudFreshAt(102000000000LL));
  EXPECT_EQ(pool.readerDiagnostics().validation_misses,0U);
}

TEST(CollisionSnapshotPool, PendingCompleteWriterGetsReleasedBackupWithoutStoppingFastReader) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  ASSERT_TRUE(pool.publishSolver(writer,101090000000LL,now));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation(),solver=pool.borrowSolver();
  ASSERT_TRUE(head&&owner&&solver);
  const auto head_address=head.get(),backup_address=owner.get(),solver_address=solver.get();
  const auto old_raw=GridMapTestAccess::raw(*head);
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  std::promise<void> missed,retry;auto missed_future=missed.get_future();auto retry_future=retry.get_future();
  bool initial=false,retried=false;
  std::thread writing([&]{
    initial=pool.publishValidation(writer,101190000000LL,now);
    missed.set_value();retry_future.wait();
    retried=pool.publishValidation(writer,101190000000LL,now);
  });
  missed_future.wait();
  EXPECT_FALSE(initial); // Both actual validation slots were borrowed at fusion completion.
  owner.reset();
  EXPECT_FALSE(pool.borrowValidation()); // A fast fallback cannot steal the pending writer's backup either.
  auto overtaking_owner=pool.borrowOwnerValidation();
  EXPECT_FALSE(overtaking_owner); // The owner must not repeatedly win before the 10 ms writer retry.
  // Current-curve checks remain available on their already-complete head.
  head.reset();
  head=pool.borrowValidation();
  EXPECT_TRUE(head);
  if(head) {EXPECT_EQ(head.get(),head_address);EXPECT_EQ(head->latestCloudStampNs(),101000000000LL);}
  retry.set_value();writing.join();
  EXPECT_TRUE(retried);
  if(head) {EXPECT_EQ(head->latestCloudStampNs(),101000000000LL);EXPECT_EQ(GridMapTestAccess::raw(*head),old_raw);}
  EXPECT_EQ(solver.get(),solver_address);EXPECT_EQ(solver->latestCloudStampNs(),101000000000LL);
  if(retried) {
    auto fresh=pool.borrowValidation();ASSERT_TRUE(fresh);
    EXPECT_EQ(fresh.get(),backup_address);EXPECT_EQ(fresh->latestCloudStampNs(),101100000000LL);
    EXPECT_EQ(fresh->integratedRaySourceStamp(0),101099999000LL);
    head.reset();auto resumed_owner=pool.borrowOwnerValidation();ASSERT_TRUE(resumed_owner);
    EXPECT_EQ(resumed_owner.get(),head_address); // Writer intent retired, with no fourth slot.
  }
}

TEST(CollisionSnapshotPool, PendingWriterDoesNotReserveFallbackForAnIdenticalPublishedMap) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
  EXPECT_FALSE(pool.publishValidation(writer,101000000000LL,now));
  owner.reset();
  EXPECT_TRUE(pool.borrowOwnerValidation()); // No new completed state is awaiting delivery.
}

TEST(CollisionSnapshotPool, PendingWriterCanUseFastHeadWhenOwnerIsLastReaderToFinish) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101300000000LL,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101300000000LL,now));
  ASSERT_TRUE(pool.publishSolver(writer,101300000000LL,now));
  auto fast=pool.borrowValidation(),owner=pool.borrowOwnerValidation(),solver=pool.borrowSolver();
  ASSERT_TRUE(fast&&owner&&solver);
  const auto fast_address=fast.get(),owner_address=owner.get(),solver_address=solver.get();
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  ASSERT_FALSE(pool.publishValidation(writer,101400000000LL,now));

  // Both readers overlapped fusion completion, but current-curve validation
  // finishes first. A second 100 ms check of its old head would consume the
  // only free destination and postpone delivery of the COMPLETE newer map.
  // Give that slot to the pending writer while the owner still holds backup.
  fast.reset();
  auto repeated_old_head=pool.borrowValidation();
  EXPECT_FALSE(repeated_old_head);
  EXPECT_FALSE(pool.borrowOwnerValidation());
  EXPECT_TRUE(pool.publishValidation(writer,101410000000LL,now));
  if(!repeated_old_head) {
    auto current=pool.borrowValidation();ASSERT_TRUE(current);
    EXPECT_EQ(current.get(),fast_address);
    EXPECT_EQ(current->latestCloudStampNs(),101100000000LL);
    EXPECT_EQ(current->integratedRaySourceStamp(0),101099999000LL);
    EXPECT_TRUE(current->integratedCloudFreshAt(101599000000LL));
    EXPECT_FALSE(current->integratedCloudFreshAt(101600000001LL));
    EXPECT_EQ(GridMapTestAccess::raw(*current),GridMapTestAccess::raw(writer));
    EXPECT_EQ(current->getInflateOccupancy(Eigen::Vector3d(.25,.25,.25),0.),1);
    EXPECT_EQ(owner.get(),owner_address);
    EXPECT_EQ(owner->latestCloudStampNs(),101000000000LL);
    EXPECT_EQ(solver.get(),solver_address);
    EXPECT_EQ(solver->latestCloudStampNs(),101000000000LL);
  }
}

TEST(CollisionSnapshotPool, PendingFastHeadReservationRetiresWhenAllReadersGoIdle) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishValidation(writer,101300000000LL,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101300000000LL,now));
  auto fast=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(fast&&owner);
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  ASSERT_FALSE(pool.publishValidation(writer,101400000000LL,now));
  fast.reset();EXPECT_FALSE(pool.borrowValidation());
  owner.reset();
  // With both slots free, backup alone is enough for writer delivery. Do not
  // turn the fast-reader yield into a global latch or wait for a reader wake.
  fast=pool.borrowValidation();ASSERT_TRUE(fast);
  ASSERT_TRUE(pool.publishValidation(writer,101410000000LL,now));
  auto newest=pool.borrowValidation();ASSERT_TRUE(newest);
  EXPECT_NE(newest.get(),fast.get());
  EXPECT_EQ(newest->latestCloudStampNs(),101100000000LL);
  fast.reset();newest.reset();
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101420000000LL,now));
  auto idle_fast=pool.borrowValidation(),idle_owner=pool.borrowOwnerValidation();
  ASSERT_TRUE(idle_fast&&idle_owner);EXPECT_NE(idle_fast.get(),idle_owner.get());
}

TEST(CollisionSnapshotPool, PendingWriterCopyExceptionReleasesFillFlagAndAllowsExactRetry) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  EXPECT_FALSE(pool.publishValidation(writer,101100000000LL,now));
  owner.reset();GridMapTestAccess::renderOnly(writer,true);
  EXPECT_THROW(pool.publishValidation(writer,101100000000LL,now),std::logic_error);
  EXPECT_FALSE(pool.borrowOwnerValidation());
  GridMapTestAccess::renderOnly(writer,false);
  ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
  auto fresh=pool.borrowValidation();ASSERT_TRUE(fresh);
  EXPECT_EQ(fresh->latestCloudStampNs(),101100000000LL);
  EXPECT_EQ(head->latestCloudStampNs(),101000000000LL);
  head.reset();EXPECT_TRUE(pool.borrowOwnerValidation());
}

TEST(CollisionSnapshotPool, PendingWriterCannotRenewAStaleHeadOrChangeResetIdentity) {
  for(bool clock_fault:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
    scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
    GridMapTestAccess::sourceEvidence(writer,101100000000LL);
    EXPECT_FALSE(pool.publishValidation(writer,101100000000LL,now));
    owner.reset();head.reset();
    auto old_head=pool.borrowValidation();ASSERT_TRUE(old_head);
    GridMapTestAccess::elapseSnapshot(*old_head,std::chrono::seconds(2));
    EXPECT_EQ(old_head->latestCloudStampNs(),101000000000LL);
    EXPECT_FALSE(old_head->integratedCloudFreshAt(103000000000LL));
    // A context or clock fault is visible even before a new copy can finish.
    if(clock_fault)GridMapTestAccess::clockFault(writer);
    else ASSERT_TRUE(writer.applyLocalizationContext(context(2,2)));
    owner=pool.borrowOwnerValidation();EXPECT_FALSE(owner);
    ASSERT_TRUE(pool.publishValidation(writer,103000000000LL,now));
    old_head.reset();
    EXPECT_FALSE(pool.borrowOwnerValidation()); // No reacquisition of the old identity.
    auto current=pool.borrowValidation();ASSERT_TRUE(current);
    EXPECT_EQ(current->rayEvidenceClockFault(),clock_fault);
    EXPECT_EQ(current->localizationContextSequence(),writer.localizationContextSequence());
  }
}

TEST(CollisionSnapshotPool, OlderOwnerCannotReplaceAlreadyPublishedFreshSolverSources) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,100500000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  ASSERT_TRUE(pool.publishSolver(writer,101090000000LL,now));
  auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  ASSERT_EQ(owner->latestCloudStampNs(),100500000000LL);
  auto solver=pool.acquireSolver(*owner,101100000000LL,
    now+std::chrono::milliseconds(400));ASSERT_TRUE(solver);
  EXPECT_EQ(solver->latestCloudStampNs(),101000000000LL);
  EXPECT_EQ(solver->integratedRaySourceStamp(0),100999999000LL);
  EXPECT_TRUE(solver->integratedCloudFreshAt(101100000000LL));
  EXPECT_FALSE(solver->integratedCloudFreshAt(102000000000LL)); // Original deadline still expires.
  EXPECT_EQ(pool.readerDiagnostics().solver_newer_publication_reuses,1U);
}

TEST(CollisionSnapshotPool, ReleasedOwnerReplicaCatchesUpWithoutWaitingForAnotherFusion) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,100500000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto old_owner=pool.borrowOwnerValidation();ASSERT_TRUE(old_owner);
  GridMapTestAccess::sourceEvidence(writer,100700000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  auto fast=pool.borrowValidation();ASSERT_TRUE(fast);
  EXPECT_TRUE(fast->integratedCloudFreshAt(101120000000LL));
  EXPECT_FALSE(old_owner->integratedCloudFreshAt(101120000000LL));
  EXPECT_FALSE(pool.refreshOwnerReplica(writer,101120000000LL,now));
  EXPECT_EQ(old_owner->latestCloudStampNs(),100500000000LL); // Pinned readers stay immutable.
  old_owner.reset();
  // Before this retry existed the next owner immediately reacquired the
  // same 620 ms backup although the protected fast head was only 420 ms old.
  auto stranded=pool.borrowOwnerValidation();ASSERT_TRUE(stranded);
  EXPECT_FALSE(stranded->integratedCloudFreshAt(101120000000LL));stranded.reset();
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101120000000LL,now));
  // The former fast head may still be borrowed. It is never stolen/overwritten.
  EXPECT_EQ(fast->latestCloudStampNs(),100700000000LL);fast.reset();
  auto repaired=pool.borrowOwnerValidation();ASSERT_TRUE(repaired);
  auto current=pool.borrowValidation();ASSERT_TRUE(current);
  EXPECT_NE(repaired.get(),current.get());
  EXPECT_EQ(repaired->latestCloudStampNs(),100700000000LL);
  EXPECT_EQ(repaired->integratedRaySourceStamp(0),100699999000LL);
  EXPECT_TRUE(repaired->integratedCloudFreshAt(101120000000LL));
  EXPECT_EQ(GridMapTestAccess::raw(*repaired),GridMapTestAccess::raw(*current));
  EXPECT_EQ(repaired->getInflateOccupancy({-.15,-.15,-.15},0.),
            current->getInflateOccupancy({-.15,-.15,-.15},0.));
  const auto sequence=pool.readerDiagnostics().latest_publication_sequence;
  for(int i=0;i<100;++i)EXPECT_FALSE(pool.refreshOwnerReplica(writer,101120000000LL+i,now));
  EXPECT_EQ(pool.readerDiagnostics().latest_publication_sequence,sequence);
  // Delivery/copy does not create new sensing evidence or renew ray leases.
  EXPECT_FALSE(repaired->integratedCloudFreshAt(102000000000LL));
  EXPECT_EQ(repaired->latestCloudStampNs(),100700000000LL);
}

TEST(CollisionSnapshotPool, ReplicaRetryDoesNotPublishAheadOfHeadOrAcrossReset) {
  for(bool reset:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer);
    ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::seedEvidence(writer);
    scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
    EXPECT_FALSE(pool.refreshOwnerReplica(writer,101000000000LL,now));
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    ASSERT_TRUE(pool.refreshOwnerReplica(writer,101000000000LL,now));
    auto old=pool.borrowOwnerValidation();ASSERT_TRUE(old);
    if(reset)ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"replica-reset")));
    else GridMapTestAccess::clockFault(writer);
    EXPECT_FALSE(pool.refreshOwnerReplica(writer,101100000000LL,now));
    ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
    EXPECT_FALSE(pool.refreshOwnerReplica(writer,101100000000LL,now));
    EXPECT_EQ(old->localizationContextSequence(),1U);
    EXPECT_FALSE(old->rayEvidenceClockFault());
    old.reset();
    EXPECT_FALSE(pool.borrowOwnerValidation()); // Old-source identity cannot be reacquired.
    ASSERT_TRUE(pool.refreshOwnerReplica(writer,101100000000LL,now));
    auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
    EXPECT_EQ(owner->localizationContextSequence(),reset?2U:1U);
    EXPECT_EQ(owner->rayEvidenceClockFault(),!reset);
    EXPECT_FALSE(pool.refreshOwnerReplica(writer,101100000001LL,now));
  }
}

TEST(CollisionSnapshotPool, SameSourceOwnerReplicaDoesNotPromoteOrStealFastHead) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  scan_planner::CollisionSnapshotPool::PublicationTiming head_timing;
  auto fast=pool.borrowValidation(&head_timing);ASSERT_TRUE(fast);
  const auto* head=fast.get();
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101100000000LL,now));
  // Completing another reader's copy is not a newer map observation. It must
  // not swap the protected head's role or make the owner wait for this reader.
  EXPECT_EQ(pool.readerDiagnostics().latest_publication_sequence,head_timing.publication_sequence);
  auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  EXPECT_NE(owner.get(),head);
  EXPECT_EQ(owner->latestCloudStampNs(),fast->latestCloudStampNs());
  owner.reset();fast.reset();
  for(unsigned i=0;i<10;++i) {
    // Even when both slots are free, renewal should not chase the replica's
    // later copy sequence. Reader roles stay stable until a real publication.
    auto renewal=pool.borrowValidation();ASSERT_TRUE(renewal);
    EXPECT_EQ(renewal.get(),head);
    auto admission=pool.borrowOwnerValidation();ASSERT_TRUE(admission);
    EXPECT_NE(admission.get(),head);
    EXPECT_FALSE(pool.refreshOwnerReplica(writer,101100000000LL+i,now));
  }
}

TEST(CollisionSnapshotPool, StableHeadAllowsSameSourceFallbackWithoutOverwritingReaders) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101000000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101090000000LL,now));
  auto head=pool.borrowValidation();ASSERT_TRUE(head);
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101100000000LL,now));
  auto fallback=pool.borrowValidation();ASSERT_TRUE(fallback);
  EXPECT_NE(head.get(),fallback.get());
  EXPECT_FALSE(pool.borrowOwnerValidation());
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  EXPECT_FALSE(pool.publishValidation(writer,101190000000LL,now));
  EXPECT_FALSE(pool.refreshOwnerReplica(writer,101190000000LL,now));
  EXPECT_EQ(head->latestCloudStampNs(),101000000000LL);
  EXPECT_EQ(fallback->latestCloudStampNs(),101000000000LL);
  fallback.reset();
  ASSERT_TRUE(pool.publishValidation(writer,101190000000LL,now));
  auto next=pool.borrowValidation();ASSERT_TRUE(next);
  EXPECT_EQ(next->latestCloudStampNs(),101100000000LL);
  EXPECT_NE(next.get(),head.get());
  head.reset();
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101200000000LL,now));
  auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
  EXPECT_EQ(owner->latestCloudStampNs(),101100000000LL);
  EXPECT_NE(owner.get(),next.get());
}

TEST(CollisionSnapshotPool, ReplicaCatchupKeepsThirdSlotAndIgnoresReceiptOnlyUpdates) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,now));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101000000000LL,now));
  const auto sequence=pool.readerDiagnostics().latest_publication_sequence;
  for(int i=0;i<100;++i)EXPECT_FALSE(pool.refreshOwnerReplica(writer,101000000000LL+i,now));
  EXPECT_EQ(pool.readerDiagnostics().latest_publication_sequence,sequence);
  GridMapTestAccess::sourceEvidence(writer,101100000000LL);
  // Replica retry is a continuation of a real completed head publication.
  EXPECT_FALSE(pool.refreshOwnerReplica(writer,101190000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101190000000LL,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(writer,101190000000LL,now));
  auto owner=pool.borrowOwnerValidation();auto fast=pool.borrowValidation();
  ASSERT_TRUE(owner);ASSERT_TRUE(fast);
  EXPECT_NE(owner.get(),fast.get());EXPECT_NE(owner.get(),solver.get());
  EXPECT_NE(fast.get(),solver.get());
  EXPECT_EQ(owner->latestCloudStampNs(),101100000000LL);
  EXPECT_EQ(fast->latestCloudStampNs(),101100000000LL);
  EXPECT_EQ(solver->latestCloudStampNs(),101000000000LL);
}

TEST(CollisionSnapshotPool, OwnerCannotFallBackAcrossPublishedContextOrClockFault) {
  for(bool context_reset:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
    ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::seedEvidence(writer);
    scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    if(context_reset)ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"new-seed")));
    else GridMapTestAccess::clockFault(writer);
    ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
    scan_planner::CollisionSnapshotPool::PublicationTiming timing;
    auto current=pool.borrowValidation(&timing);ASSERT_TRUE(current);
    EXPECT_EQ(timing.context_sequence,context_reset?2U:1U);
    EXPECT_EQ(timing.clock_fault,!context_reset);
    EXPECT_FALSE(pool.borrowOwnerValidation()); // Old valid-looking backup is not current context/fault evidence.
    current.reset();
    EXPECT_FALSE(pool.borrowOwnerValidation()); // Current fast head remains reserved.
    ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
    auto owner=pool.borrowOwnerValidation();ASSERT_TRUE(owner);
    EXPECT_EQ(owner->localizationContextSequence(),timing.context_sequence);
    EXPECT_EQ(owner->rayEvidenceClockFault(),timing.clock_fault);
    if(context_reset)EXPECT_EQ(owner->latestCloudStampNs(),0);
  }
}

TEST(CollisionSnapshotPool, MissedResetOrFaultPublicationBlocksNewOldIdentityLeases) {
  for(bool context_reset:{false,true}) {
    GridMap writer;GridMapTestAccess::configure(writer);
    ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
    GridMapTestAccess::seedEvidence(writer);
    scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    auto first=pool.borrowValidation();ASSERT_TRUE(first);
    ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
    auto second=pool.borrowValidation();ASSERT_TRUE(second);
    ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,now));
    auto third=pool.borrowSolver();ASSERT_TRUE(third);
    const auto old_source=first->latestCloudStampNs();
    if(context_reset)ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"reset-while-pinned")));
    else GridMapTestAccess::clockFault(writer);
    EXPECT_FALSE(pool.publishValidation(writer,101100000000LL,now));
    EXPECT_EQ(first->localizationContextSequence(),1U); // Never overwrite an outstanding reader.
    EXPECT_EQ(first->latestCloudStampNs(),old_source);
    EXPECT_EQ(second->latestCloudStampNs(),old_source);
    first.reset();second.reset();third.reset();
    // No ray arrived and no invalid-map copy could be stored. The writer
    // identity watermark nevertheless prevents all readers reacquiring old
    // evidence as the newest context after their previous leases retire.
    EXPECT_FALSE(pool.borrowValidation());
    EXPECT_FALSE(pool.borrowOwnerValidation());
    EXPECT_FALSE(pool.borrowSolver());
    EXPECT_FALSE(pool.borrowAdmission());
    pool.cancelSolverRequest();pool.cancelAdmissionRequest();
    ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
    ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,now));
    scan_planner::CollisionSnapshotPool::PublicationTiming timing;
    auto owner=pool.borrowOwnerValidation(&timing);ASSERT_TRUE(owner);
    EXPECT_EQ(owner->localizationContextSequence(),context_reset?2U:1U);
    EXPECT_EQ(owner->rayEvidenceClockFault(),!context_reset);
    if(context_reset)EXPECT_EQ(owner->latestCloudStampNs(),0);
    else EXPECT_EQ(owner->latestCloudStampNs(),old_source);
    EXPECT_EQ(timing.publication_misses,1U);
    ASSERT_TRUE(pool.publishSolver(writer,101100000000LL,now));
    auto current_solver=pool.borrowSolver();ASSERT_TRUE(current_solver);
    EXPECT_EQ(current_solver->localizationContextSequence(),owner->localizationContextSequence());
    EXPECT_EQ(current_solver->rayEvidenceClockFault(),owner->rayEvidenceClockFault());
    EXPECT_EQ(current_solver->latestCloudStampNs(),owner->latestCloudStampNs());
  }
}

TEST(CollisionSnapshotPool, WriterValidatorAndOwnerHaveOnlyThreeExclusiveSnapshotAddresses) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto now=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,now));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,now));
  auto validator=pool.borrowValidation();ASSERT_TRUE(validator);
  const auto validator_source=validator->latestCloudStampNs();
  const auto validator_raw=GridMapTestAccess::raw(*validator);
  std::set<const GridMap*> slots{solver.get(),validator.get()};
  std::atomic<bool> go{false},owner_has_read{false};
  std::atomic<unsigned> owner_reads{0},publications{0};
  std::promise<void> writer_done;auto done=writer_done.get_future();
  std::thread publishing([&] {
    while(!go.load())std::this_thread::yield();
    // Keep the writer active until at least one concurrent owner lease exists;
    // this is a synchronization seam, not a simulated ROS callback or actuator.
    for(unsigned i=0;i<1000 || !owner_has_read.load();++i) {
      GridMapTestAccess::sourceEvidence(writer,101000000000LL+static_cast<std::int64_t>(i)*1000);
      if(pool.publishValidation(writer,101200000000LL,std::chrono::steady_clock::now()))++publications;
      if(i>100000)break; // Failure remains finite even if a broken borrow API never succeeds.
    }
    writer_done.set_value();
  });
  go.store(true);
  for(unsigned i=0;i<1000 || done.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i) {
    auto owner=pool.borrowOwnerValidation();
    if(!owner){std::this_thread::yield();continue;}
    slots.insert(owner.get());
    EXPECT_NE(owner.get(),validator.get());EXPECT_NE(owner.get(),solver.get());
    const auto source=owner->latestCloudStampNs();
    owner->getPlanningInflateOccupancy({-.15,-.15,-.15},0.); // Mutates only this reader's query cache.
    EXPECT_EQ(owner->latestCloudStampNs(),source);
    EXPECT_GE(source,101000000000LL);
    owner_has_read.store(true);++owner_reads;
  }
  publishing.join();
  EXPECT_GT(owner_reads.load(),0U);EXPECT_GT(publications.load(),0U);
  EXPECT_EQ(slots.size(),3U); // No fourth map may be allocated under contention.
  EXPECT_EQ(validator->latestCloudStampNs(),validator_source);
  EXPECT_EQ(GridMapTestAccess::raw(*validator),validator_raw);
  EXPECT_FALSE(pool.publishSolver(writer,101200000000LL,now));
  solver.reset();validator.reset();
  ASSERT_TRUE(pool.publishValidation(writer,101200000000LL,now));
  auto released_owner=pool.borrowOwnerValidation();ASSERT_TRUE(released_owner);
  EXPECT_TRUE(slots.count(released_owner.get()));
  EXPECT_EQ(released_owner->latestCloudStampNs(),writer.latestCloudStampNs());
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

TEST(CollisionSnapshotPool, AdmissionDoesNotBorrowReleasedSolverInputBehindCompletedHead) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  GridMapTestAccess::sourceEvidence(writer,101417300000LL);
  ASSERT_TRUE(pool.publishSolver(writer,101900000000LL,captured));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);const auto third=solver.get();
  // Field ordering: new active proof at 102.020 uses a completed map while
  // candidate admission at 102.077 still sees the solver's released old input.
  GridMapTestAccess::sourceEvidence(writer,101617300000LL);
  ASSERT_TRUE(pool.publishValidation(writer,102020000000LL,captured));
  auto active=pool.borrowValidation();ASSERT_TRUE(active);
  solver.reset();
  EXPECT_FALSE(pool.borrowAdmission()); // Old code incorrectly returns the old third slot.
  EXPECT_EQ(active->integratedRaySourceStamp(1),101617300000LL);
  EXPECT_EQ(pool.refreshSolverPublication(writer,102078000000LL,captured),
      scan_planner::CollisionSnapshotPool::SolverPublication::Published);
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_EQ(admission.get(),third);EXPECT_NE(admission.get(),active.get());
  EXPECT_EQ(admission->integratedRaySourceStamp(1),101617300000LL);
  EXPECT_EQ(admission->latestCloudStampNs(),101617300000LL); // Not re-stamped to 102.078.
}

TEST(CollisionSnapshotPool, AdmissionRefreshDeduplicatesEvenWhenCurrentReaderPinsSlot) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  using Publication=scan_planner::CollisionSnapshotPool::SolverPublication;
  ASSERT_EQ(pool.refreshSolverPublication(writer,101000000000LL,captured),Publication::Published);
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  const auto source=admission->latestCloudStampNs();const auto raw=GridMapTestAccess::raw(*admission);
  for(unsigned i=0;i<100;++i)
    EXPECT_EQ(pool.refreshSolverPublication(writer,101100000000LL,captured),Publication::Unchanged);
  GridMapTestAccess::sourceEvidence(writer,101200000000LL);
  EXPECT_EQ(pool.refreshSolverPublication(writer,101300000000LL,captured),Publication::Deferred);
  EXPECT_EQ(admission->latestCloudStampNs(),source);EXPECT_EQ(GridMapTestAccess::raw(*admission),raw);
  admission.reset();
  EXPECT_EQ(pool.refreshSolverPublication(writer,101300000000LL,captured),Publication::Published);
}

TEST(CollisionSnapshotPool, PendingAdmissionHasFiniteVersionFloorUnderContinuouslyAdvancingHead) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  const auto deadline=captured+std::chrono::seconds(5);
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,captured));
  auto solver=pool.borrowSolver(deadline);ASSERT_TRUE(solver);
  GridMapTestAccess::sourceEvidence(writer,101200000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101250000000LL,captured));
  EXPECT_FALSE(pool.borrowAdmission()); // Pin the required, already complete 101.2 version.
  solver.reset();
  EXPECT_FALSE(pool.borrowSolver(deadline)); // Pending admission owns the next turn.
  ASSERT_EQ(pool.refreshSolverPublication(writer,101260000000LL,captured),
      scan_planner::CollisionSnapshotPool::SolverPublication::Published);
  GridMapTestAccess::sourceEvidence(writer,101300000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101350000000LL,captured));
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_EQ(admission->latestCloudStampNs(),101200000000LL); // Required floor does not chase 101.3.
  admission.reset();
  EXPECT_FALSE(pool.borrowAdmission()); // New turn must capture the newer head.
  solver=pool.borrowSolver(deadline);ASSERT_TRUE(solver); // Solver still gets its fair turn.
  solver.reset();pool.cancelSolverRequest();pool.cancelAdmissionRequest();
  EXPECT_TRUE(pool.borrowSolver(deadline)); // Cancel withdraws an unfulfilled admission turn.
}

TEST(CollisionSnapshotPool, ContextChangeClearsPendingAdmissionFloorWithoutRevivingOldMap) {
  GridMap writer;GridMapTestAccess::configure(writer);GridMapTestAccess::seedEvidence(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,captured));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  GridMapTestAccess::sourceEvidence(writer,101200000000LL);
  ASSERT_TRUE(pool.publishValidation(writer,101250000000LL,captured));
  EXPECT_FALSE(pool.borrowAdmission());
  ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"reset-during-admission-wait")));
  ASSERT_TRUE(pool.publishValidation(writer,101300000000LL,captured));
  solver.reset();EXPECT_FALSE(pool.borrowAdmission());
  ASSERT_EQ(pool.refreshSolverPublication(writer,101300000000LL,captured),
      scan_planner::CollisionSnapshotPool::SolverPublication::Published);
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_EQ(admission->localizationContextSequence(),2U);
  EXPECT_EQ(admission->latestCloudStampNs(),0); // Reset remains invalid/no evidence; never restored.
}

TEST(CollisionSnapshotPool, MissedResetPublicationCannotRecreateAnOldContextAdmissionFloor) {
  GridMap writer;GridMapTestAccess::configure(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(context(1,1)));
  GridMapTestAccess::seedEvidence(writer);
  scan_planner::CollisionSnapshotPool pool;const auto captured=std::chrono::steady_clock::now();
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,captured));
  auto first=pool.borrowValidation();ASSERT_TRUE(first);
  ASSERT_TRUE(pool.publishValidation(writer,101000000000LL,captured));
  auto second=pool.borrowValidation();ASSERT_TRUE(second);
  ASSERT_TRUE(pool.publishSolver(writer,101000000000LL,captured));
  auto solver=pool.borrowSolver();ASSERT_TRUE(solver);
  EXPECT_FALSE(pool.borrowAdmission());
  ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"missed-admission-reset")));
  EXPECT_FALSE(pool.publishValidation(writer,101100000000LL,captured));
  // The identity fence changed, but both physical head buffers are still old.
  // Do not capture that old physical head as the next pending source floor.
  EXPECT_FALSE(pool.borrowAdmission());
  first.reset();second.reset();solver.reset();
  ASSERT_TRUE(pool.publishValidation(writer,101100000000LL,captured));
  ASSERT_EQ(pool.refreshSolverPublication(writer,101100000000LL,captured),
      scan_planner::CollisionSnapshotPool::SolverPublication::Published);
  auto admission=pool.borrowAdmission();ASSERT_TRUE(admission);
  EXPECT_EQ(admission->localizationContextSequence(),2U);
  EXPECT_EQ(admission->latestCloudStampNs(),0);
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

TEST(MapContext, RealResetPublishesInvalidContextBeforeAckWithoutWaitingForAnotherRay) {
  GridMap writer;GridMapTestAccess::configure(writer);
  scan_planner::CollisionSnapshotPool pool;unsigned publications=0;
  writer.setCollisionUpdateCallback([&] {
    ++publications;
    EXPECT_GT(writer.localizationContextSequence(),0U);
    EXPECT_EQ(writer.latestCloudStampNs(),0);
    EXPECT_EQ(writer.integratedRaySourceStamp(0),0);
    EXPECT_EQ(writer.integratedRaySourceStamp(1),0);
    ASSERT_TRUE(pool.publishValidation(writer,102000000000LL,std::chrono::steady_clock::now()));
  });
  const auto first=context(1,1);
  ASSERT_TRUE(writer.applyLocalizationContext(first));
  EXPECT_EQ(publications,1U);
  auto first_view=pool.borrowValidation();ASSERT_TRUE(first_view);
  EXPECT_EQ(first_view->localizationContextSequence(),1U);
  first_view.reset();
  GridMapTestAccess::seedEvidence(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(first));
  EXPECT_EQ(publications,1U); // Identical reliable retry is not a second reset/publication.
  EXPECT_EQ(writer.latestCloudStampNs(),101000000000LL);
  EXPECT_FALSE(writer.applyLocalizationContext("not valid json"));
  EXPECT_FALSE(writer.applyLocalizationContext(context(1,0)));
  EXPECT_EQ(publications,1U);
  ASSERT_TRUE(writer.applyLocalizationContext(context(2,2,"new-seed")));
  EXPECT_EQ(publications,2U);
  auto reset_view=pool.borrowValidation();ASSERT_TRUE(reset_view);
  EXPECT_EQ(reset_view->localizationContextSequence(),2U);
  EXPECT_EQ(reset_view->latestCloudStampNs(),0);
  EXPECT_EQ(reset_view->integratedRaySourceStamp(0),0);
  EXPECT_EQ(reset_view->integratedRaySourceStamp(1),0);
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
