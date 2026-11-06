#include <gtest/gtest.h>
#include "plan_env/grid_map.h"

#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
#include <future>

// Memory-only production GridMap: no ROS node, SDK, bag playback, alternate
// occupancy implementation or near-body clearing. Each test writes explicit
// synthetic voxel evidence and exercises the real native query/snapshot path.
struct GridMapTestAccess {
  static void configure(GridMap &map,bool strict=true,double length=.93,double width=.30,
      double below=.25,double above=.25) {
    auto &p=map.mp_;auto &d=map.md_;
    p.require_observed_free_=strict;p.use_projected_rays_=false;
    p.collision_model_="oriented_box";p.box_length_=length;p.box_width_=width;
    p.body_below_=below;p.body_above_=above;
    p.frame_id_="odom";d.has_ray_pose_=false;
    p.resolution_=.05;p.resolution_inv_=20.;
    p.map_voxel_num_={32,32,64};p.map_origin_idx_.setZero();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.20;
    p.obstacles_inflation_z_up=.40;p.obstacles_inflation_z_down=.45;
    const std::size_t size=32U*32U*64U;
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);
    d.count_hit_and_miss_.resize(size);d.flag_rayend_.resize(size);d.flag_traverse_.resize(size);
    map.updateMapBoundaryFromIndex();map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static void fillBoxFree(GridMap &map,const Eigen::Vector3d &position,double yaw) {
    scan_planner::forEachOrientedBoxVoxel(position,yaw,map.getResolution(),map.bodyEnvelope(),
      [&](const Eigen::Vector3i &cell) {
        if (map.isInMap(cell)) {
          Eigen::Vector3d point;map.indexToPos(cell,point);map.setOccupancy(point,0.);
        }
        return true;
      });
  }
  static void measuredFreeTimes(GridMap &map,std::int64_t source,std::int64_t now) {
    map.mp_.use_projected_rays_=true;map.mp_.cloud_pose_max_age_=.5;
    map.ray_query_clock_ns_=now;map.integrated_cloud_stamp_ns_=source;
    map.ray_integrated_stamps_={source,source};
    map.free_observation_stamps_.assign(map.md_.occupancy_buffer_.size(),source);
  }
  static void queryTime(GridMap &map,std::int64_t now) {map.ray_query_clock_ns_=now;}
  static auto raw(const GridMap &map) {return map.md_.occupancy_buffer_;}
  static void weakHit(GridMap &map,const Eigen::Vector3d &point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);map.applyOccupancyUpdate(cell,.5);
  }
  static void inflatedOnes(GridMap &map) {
    std::fill(map.md_.occupancy_buffer_inflate_.begin(),map.md_.occupancy_buffer_inflate_.end(),1);
    std::fill(map.md_.planning_occupancy_buffer_inflate_.begin(),map.md_.planning_occupancy_buffer_inflate_.end(),1);
  }
  static void planningStrict(GridMap &map) {map.mp_.planning_collision_policy_="observed_free";}
  static void model(GridMap &map,const std::string &model) {map.mp_.collision_model_=model;map.rebuildInflationOffsets();}
  static void candidateBelow(GridMap &map,double below) {
    map.mp_.planning_body_below_=below;map.rebuildInflationOffsets();
  }
  static void bodyBelow(GridMap &map,double below) {
    map.mp_.body_below_=below;map.rebuildInflationOffsets();
  }
  static void stairClearance(GridMap& map,double clearance=.025,double nominal_length=.93,double nominal_width=.30) {
    map.mp_.box_nominal_length_=nominal_length;map.mp_.box_nominal_width_=nominal_width;
    map.mp_.stair_planar_clearance_=clearance;map.rebuildInflationOffsets();
  }
  static void subvoxels(GridMap& map,bool enable=true) {
    map.mp_.stair_subvoxel_collision_=enable;map.resetAllMapData();
  }
  static void allHitBounds(GridMap& map) {
    map.mp_.stair_contact_body_below_=.12;subvoxels(map);
  }
  static void discardHitBounds(GridMap& map) {
    map.md_.occupied_hit_bounds_.clear();map.invalidateCollisionQueryCaches();
  }
  static auto subvoxelStorage(const GridMap& map) {return map.md_.occupied_hit_subvoxels_;}
  static auto subvoxelEvidence(GridMap& map,const Eigen::Vector3d& point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);return map.md_.occupied_hit_subvoxels_.at(map.toAddress(cell));
  }
  static void integratedOdds(GridMap& map,const Eigen::Vector3d& point,double odds) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);map.applyOccupancyUpdateAtIndex(cell,map.toAddress(cell),odds);
  }
  static void slideEvidence(GridMap& map,const Eigen::Vector3d& center) {
    map.mp_.map_sliding_en_=true;map.mp_.map_sliding_thresh_vox_=1;map.updateSlidingMap(center);
  }
  static void ground(GridMap &map,double z) {map.mp_.ground_height_=z;}
  static auto visualization(GridMap &map,bool inflated) {return map.cachedVisualization(inflated);}
  static auto visualizationBuilds(const GridMap &map) {return map.visualization_builds_;}
  static void captureDisplay(const GridMap& map,GridMap& view,std::int64_t stamp=1000000000LL) {
    map.copyVisualizationTo(view,stamp,true,true,false);
  }
  static auto displayInputStorage(const GridMap& view) {
    return std::make_tuple(view.visualization_raw_flags_.size(),view.md_.occupancy_buffer_.size(),
      view.free_observation_stamps_.size(),view.md_.occupancy_buffer_inflate_.size());
  }
  static void startDisplay(GridMap& map,std::function<void(GridMap&)> sink) {
    map.visualization_sink_=std::move(sink);map.startVisualizationWorker();
  }
  static bool submitDisplay(GridMap& map,std::int64_t stamp) {
    return map.submitVisualization(stamp,true,true,false);
  }
  static bool submitDisplayWhenIdle(GridMap& map,std::int64_t stamp) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    do {
      if(submitDisplay(map,stamp))return true;
      std::this_thread::yield();
    } while(std::chrono::steady_clock::now()<deadline);
    return false;
  }
  static void stopDisplay(GridMap& map) {map.stopVisualizationWorker();}
  static bool displayCurrent(const GridMap& map) {return map.visualizationCurrent();}
  static std::int64_t displayStamp(const GridMap& map) {return map.visualizationStamp().nanoseconds();}
  static void bodyYaw(GridMap &map,double yaw,std::int64_t stamp=1000000000LL,
      const std::string &frame="odom") {
    auto message=std::make_shared<nav_msgs::msg::Odometry>();
    message->header.frame_id=frame;message->header.stamp.sec=stamp/1000000000LL;
    message->header.stamp.nanosec=stamp%1000000000LL;
    message->pose.pose.orientation.w=std::cos(yaw*.5);
    message->pose.pose.orientation.z=std::sin(yaw*.5);
    map.slidingMapFrameCallback(message);
  }
  static void invalidBody(GridMap &map,std::int64_t stamp) {
    auto message=std::make_shared<nav_msgs::msg::Odometry>();message->header.frame_id="odom";
    message->header.stamp.sec=stamp/1000000000LL;message->header.stamp.nanosec=stamp%1000000000LL;
    message->pose.pose.orientation.w=0.;
    map.slidingMapFrameCallback(message); // Explicit zero-norm quaternion.
  }
  static void overflowBodyQuaternion(GridMap &map,std::int64_t stamp) {
    auto message=std::make_shared<nav_msgs::msg::Odometry>();message->header.frame_id="odom";
    message->header.stamp.sec=stamp/1000000000LL;message->header.stamp.nanosec=stamp%1000000000LL;
    message->pose.pose.orientation.w=1e308;map.slidingMapFrameCallback(message);
  }
  static auto displayWork(const GridMap &map) {
    return std::make_tuple(map.visualization_obb_mask_builds_,map.visualization_obb_kernel_builds_,
        map.visualization_obb_interval_writes_,map.visualization_obb_difference_.size());
  }
  static void displayExtent(GridMap &map,const Eigen::Vector3i &count,const Eigen::Vector3i &origin) {
    map.mp_.map_voxel_num_=count;map.mp_.map_origin_idx_=origin;map.updateMapBoundaryFromIndex();
    const auto size=static_cast<std::size_t>(count.x())*count.y()*count.z();
    map.md_.occupancy_buffer_.resize(size);map.md_.occupancy_buffer_inflate_.resize(size);
    map.md_.occupancy_buffer_inflate_cnt_.resize(size);map.md_.count_hit_.resize(size);
    map.md_.count_hit_and_miss_.resize(size);map.md_.flag_rayend_.resize(size);map.md_.flag_traverse_.resize(size);
    map.resetAllMapData();
  }
  static void displayClip(GridMap &map,double z) {
    map.md_.has_ray_pose_=true;map.md_.ray_pos_.z()=z;map.mp_.vis_height_=0.;
  }
  static void reset(GridMap &map) {map.resetAllMapData();}
  static void displayBarrier(GridMap& map,std::int64_t barrier) {map.pair_barrier_ns_=barrier;}
  static auto cells(const sensor_msgs::msg::PointCloud2 &message,double resolution=.05) {
    std::set<std::array<int,3>> result;
    if(!message.width)return result;
    sensor_msgs::PointCloud2ConstIterator<float> x(message,"x"),y(message,"y"),z(message,"z");
    for(std::size_t i=0;i<message.width;++i,++x,++y,++z)
      result.insert({static_cast<int>(std::floor(*x/resolution)),static_cast<int>(std::floor(*y/resolution)),
          static_cast<int>(std::floor(*z/resolution))});
    return result;
  }
  static auto occupiedQueryCells(GridMap &map,double yaw,double pitch=0.,double roll=0.) {
    std::set<std::array<int,3>> result;
    for(int x=map.mp_.map_bound_min_idx_.x();x<=map.mp_.map_bound_max_idx_.x();++x)
      for(int y=map.mp_.map_bound_min_idx_.y();y<=map.mp_.map_bound_max_idx_.y();++y)
        for(int z=map.mp_.map_bound_min_idx_.z();z<=map.mp_.map_bound_max_idx_.z();++z) {
          Eigen::Vector3d point;map.indexToPos({x,y,z},point);
          if(map.getPlanningInflateOccupancy(point,yaw,pitch,roll)==1)result.insert({x,y,z});
        }
    return result;
  }
  static auto bounds(const GridMap &map) {return std::make_pair(map.mp_.map_bound_min_idx_,map.mp_.map_bound_max_idx_);}
  static void thresholdAndInvalid(GridMap &map,const Eigen::Vector3i &cell,double odds) {
    map.applyOccupancyUpdate(cell,odds);
  }
  static auto collisionCaches(const GridMap &map) {
    return std::make_tuple(map.observed_cylinder_cache_.size(),map.planning_box_cache_.size(),
        map.unknown_collision_queries_,map.occupancy_revision_,map.free_evidence_revision_);
  }
  static std::size_t displayStorage(const GridMap &map) {
    return map.visualization_obb_mask_.size()+map.visualization_obb_difference_.size()*sizeof(int);
  }
  static void denseHorizontalPlane(GridMap &map) {
    for(int x=map.mp_.map_bound_min_idx_.x();x<=map.mp_.map_bound_max_idx_.x();++x)
      for(int y=map.mp_.map_bound_min_idx_.y();y<=map.mp_.map_bound_max_idx_.y();++y)
        map.md_.occupancy_buffer_[map.toAddress(Eigen::Vector3i(x,y,0))]=2.;
    ++map.occupancy_revision_;map.invalidateCollisionQueryCaches();
  }
  static void contextSession(GridMap &map) {map.mp_.localization_session_id_="display";}
  static void comparisonInput(GridMap &map,const Eigen::Vector3i &origin,bool occupied) {
    map.mp_.map_origin_idx_=origin;map.updateMapBoundaryFromIndex();
    for (std::size_t i=0;i<map.md_.occupancy_buffer_.size();++i) {
      const auto key=(i*2654435761ULL+17)%997;
      map.md_.occupancy_buffer_[i]=occupied&&key<3?2.:
        key==31?std::numeric_limits<double>::quiet_NaN():key<11?.5:-1.;
    }
    map.invalidateCollisionQueryCaches();
  }
  static int originalOfficialBoxQuery(GridMap &map,const Eigen::Vector3d &p,double yaw) {
    if (!map.isInMap(p)) return -1;
    const auto body=map.planningBodyEnvelope();const double resolution=map.getResolution();
    const double c=std::cos(yaw),s=std::sin(yaw),ac=std::abs(c),as=std::abs(s);
    const double hx=body.length*.5,hy=body.width*.5,h=resolution*.5;
    const auto xs=scan_planner::verticalVoxelSpan(p.x(),hx*ac+hy*as,hx*ac+hy*as,resolution);
    const auto ys=scan_planner::verticalVoxelSpan(p.y(),hx*as+hy*ac,hx*as+hy*ac,resolution);
    const auto zs=scan_planner::verticalVoxelSpan(p.z(),body.below,body.above,resolution);
    int state=0;
    // Frozen pre-optimization scalar SAT and per-voxel ring query oracle;
    // tests compare exact outcomes, including negative global indices.
    for(int x=xs.low;x<=xs.high;++x)for(int y=ys.low;y<=ys.high;++y) {
      const double dx=(x+.5)*resolution-p.x(),dy=(y+.5)*resolution-p.y();
      if (!(std::abs(dx)<=hx*ac+hy*as+h+1e-12 &&
            std::abs(dy)<=hx*as+hy*ac+h+1e-12 &&
            std::abs(dx*c+dy*s)<=hx+h*(ac+as)+1e-12 &&
            std::abs(-dx*s+dy*c)<=hy+h*(ac+as)+1e-12)) continue;
      for(int z=zs.low;z<=zs.high;++z) {
        const Eigen::Vector3i cell(x,y,z);
        const double odds=map.isInMap(cell)?map.md_.occupancy_buffer_[map.toAddress(cell)]:0.;
        const int next=!map.isInMap(cell)?-1:!std::isfinite(odds)?2:odds>map.mp_.min_occupancy_log_?1:0;
        state=scan_planner::mergeCollisionStatus(state,next);
        if(state==1)return state;
      }
    }
    return state;
  }
  static int scalarSpatialQuery(GridMap& map,const Eigen::Vector3d& p,double yaw,double pitch,double roll) {
    if(!map.isInMap(p))return -1;
    return scan_planner::orientedBoxStatus(p,yaw,pitch,roll,map.getResolution(),map.planningBodyEnvelope(),
      [&](const Eigen::Vector3i& cell){
        if(!map.isInMap(cell))return -1;
        const auto odds=map.md_.occupancy_buffer_[map.toAddress(cell)];
        return !std::isfinite(odds)?2:odds>map.mp_.min_occupancy_log_?1:0;
      });
  }
};

namespace {
constexpr double kPi=3.14159265358979323846;
const Eigen::Vector3d kBody(.025,.025,.54);
scan_planner::BodyEnvelope box(double length=.93,double width=.30,double below=.25,double above=.25) {
  return {.29,.20,below,above,scan_planner::CollisionModel::OrientedBox,length,width};
}
}

TEST(OrientedBoxGeometry, LegacyAggregateAndReflectedDimensionsRemainCompatible) {
  const scan_planner::BodyEnvelope legacy{.29,.20,.40,.45};
  EXPECT_EQ(legacy.model,scan_planner::CollisionModel::DoubleCylinder);
  EXPECT_DOUBLE_EQ(scan_planner::bodyExtent(legacy),.49);
  const auto b=scan_planner::reflectedBodyEnvelope(
      scan_planner::ObstacleDilation{.29,.20,.25,.25,scan_planner::CollisionModel::OrientedBox,.93,.30});
  EXPECT_DOUBLE_EQ(b.length,.93);EXPECT_DOUBLE_EQ(b.width,.30);
  EXPECT_DOUBLE_EQ(b.below,.25);EXPECT_DOUBLE_EQ(b.above,.25);
  EXPECT_DOUBLE_EQ(scan_planner::bodyExtent(b),std::hypot(.465,.15));
}

TEST(OrientedBoxGeometry, SeparatingAxesRetainContactAndRejectOutsideCorners) {
  using scan_planner::orientedBoxIntersectsVoxelXY;
  const Eigen::Vector3d p(0.,0.,0.);
  EXPECT_TRUE(orientedBoxIntersectsVoxelXY(p,0.,{4,1,0},.1,.8,.2)); // Face/edge contact.
  EXPECT_FALSE(orientedBoxIntersectsVoxelXY(p,0.,{4,2,0},.1,.8,.2));
  EXPECT_TRUE(orientedBoxIntersectsVoxelXY(p,kPi/4.,{2,2,0},.1,.8,.2));
  EXPECT_FALSE(orientedBoxIntersectsVoxelXY(p,kPi/4.,{3,-3,0},.1,.8,.2));
  // A long thin crossing OBB intersects even though no OBB vertex lies in
  // the voxel and no square vertex lies inside the OBB.
  EXPECT_TRUE(orientedBoxIntersectsVoxelXY({.05,.05,0.},0.,{0,0,0},.1,1.,.01));
}

TEST(OrientedBoxGeometry, VoxelPhaseAndVerticalContactsUseActualBoxPose) {
  for (double phase:{-.049,-.001,0.,.001,.049}) {
    const Eigen::Vector3d p(phase,phase,.50+phase);
    std::size_t count=0;
    scan_planner::forEachOrientedBoxVoxel(p,kPi/5.,.05,box(),[&](const auto &cell) {
      const double lo=cell.z()*.05,hi=(cell.z()+1.)*.05;
      EXPECT_LE(lo,p.z()+.25+1e-12);EXPECT_GE(hi,p.z()-.25-1e-12);
      ++count;return true;
    });
    EXPECT_GT(count,0U);EXPECT_LT(count,5000U);
  }
}

TEST(OrientedBoxGeometry, InvalidAndUnboundedDimensionsFailExplicitly) {
  for(double width:{0.,-.1,std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(scan_planner::forEachOrientedBoxVoxel(kBody,0.,.05,box(.93,width),[](const auto&){return true;}),std::invalid_argument);
  EXPECT_THROW(scan_planner::forEachOrientedBoxVoxel(kBody,0.,.05,box(100.,100.),[](const auto&){return true;}),std::invalid_argument);
  EXPECT_THROW(scan_planner::forEachOrientedBoxVoxel(kBody,0.,0.,box(),[](const auto&){return true;}),std::invalid_argument);
}

TEST(OrientedBoxGeometry, OccupiedWitnessOutranksUnknownOrOutside) {
  int calls=0;
  EXPECT_EQ(scan_planner::orientedBoxStatus(kBody,0.,.05,box(),[&](const auto&) {
    return ++calls==3 ? 1 : calls==1 ? -1 : 2;
  }),1);
  EXPECT_EQ(calls,3);
}

TEST(OrientedBoxMap, ColumnTraversalExactlyMatchesOriginalVoxelQueryAcrossRingAndBoundary) {
  GridMap map;GridMapTestAccess::configure(map,false);
  std::size_t comparisons=0;
  for(const Eigen::Vector3i origin:{Eigen::Vector3i::Zero().eval(),Eigen::Vector3i(67,-39,-117)})
    for(bool occupied:{false,true}) {
      GridMapTestAccess::comparisonInput(map,origin,occupied);
      Eigen::Vector3d map_origin,map_size;map.getRegion(map_origin,map_size);
      for(int i=0;i<50;++i) {
        const double fx=(i*17%53)/52.,fy=(i*29%53)/52.,fz=(i*37%53)/52.;
        const Eigen::Vector3d p=map_origin+Eigen::Vector3d(fx*map_size.x(),fy*map_size.y(),fz*map_size.z());
        for(double yaw:{0.,kPi/4.,-kPi/2.,kPi,0.123456789,-2.987654321}) {
          EXPECT_EQ(map.getPlanningInflateOccupancy(p,yaw),GridMapTestAccess::originalOfficialBoxQuery(map,p,yaw));
          ++comparisons;
        }
      }
    }
  EXPECT_EQ(comparisons,1200U);
  ::testing::Test::RecordProperty("same_input_comparisons",std::to_string(comparisons));
}

TEST(OrientedBoxMap, CandidateAndExecutionAreExplicitlyDifferentEvidencePolicies) {
  GridMap map;GridMapTestAccess::configure(map);
  EXPECT_STREQ(map.collisionQueryPolicy(),"strict_observed_oriented_box");
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),2);
  const auto e=map.inspectInflateOccupancy(kBody,0.,true);
  EXPECT_EQ(e.counts,e.unique_counts);EXPECT_GT(e.counts[2],0U);
  EXPECT_EQ(e.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::NeverObserved)],e.counts[2]);
  for(const auto &v:e.voxels) EXPECT_EQ(v.cylinder_mask,4U);
}

TEST(OrientedBoxMap, OfficialPreviewDoesNotRewriteUnknownAndStrictReaderStillRejectsIt) {
  GridMap writer;GridMapTestAccess::configure(writer,false);
  const auto raw=GridMapTestAccess::raw(writer);
  EXPECT_FALSE(writer.requiresObservedFree());
  EXPECT_EQ(writer.getInflateOccupancy(kBody,0.),0);
  EXPECT_EQ(writer.getPlanningInflateOccupancy(kBody,0.),0);
  const auto diagnostic=writer.inspectInflateOccupancy(kBody,0.,true);
  EXPECT_GT(diagnostic.classification_counts[static_cast<std::size_t>(
      scan_planner::RawVoxelDiagnostic::NeverObserved)],0U);
  EXPECT_EQ(GridMapTestAccess::raw(writer),raw);

  GridMap snapshot;
  writer.copyCollisionSnapshotTo(snapshot,100000000000LL,std::chrono::steady_clock::now());
  snapshot.requireObservedSnapshot();
  EXPECT_TRUE(snapshot.requiresObservedFree());
  EXPECT_EQ(snapshot.getInflateOccupancy(kBody,0.),2);
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(kBody,0.),0);
  EXPECT_EQ(GridMapTestAccess::raw(snapshot),raw);
  EXPECT_FALSE(writer.requiresObservedFree());

  // A real hit is still blocked in the no-motion preview policy. Neither a
  // policy change nor a diagnostic query deletes obstacle evidence.
  writer.setOccupied(kBody);
  EXPECT_EQ(writer.getInflateOccupancy(kBody,0.),1);
  EXPECT_EQ(writer.getPlanningInflateOccupancy(kBody,0.),1);
}

TEST(OrientedBoxMap, NarrowWidthAndYawUseRawBoxNotLegacyCylinderOrInflatedBuffers) {
  GridMap map;GridMapTestAccess::configure(map);
  map.setOccupied({.025,.225,.54});
  const auto raw=GridMapTestAccess::raw(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0); // Outside the 30 cm box.
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,kPi/2.),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0); // Heading-sensitive cache.
  GridMapTestAccess::inflatedOnes(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0); // No stale radius proxy.
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(OrientedBoxMap, ObstacleVoxelFaceContactCannotDisappearBecauseCentreIsOutsideBox) {
  GridMap map;GridMapTestAccess::configure(map,true,.86);
  // Body x=.025, front x=.455. This voxel spans [.45,.50], intersects front,
  // although both the measured endpoint and voxel centre .475 are outside.
  map.setOccupied({.499,.025,.54});
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),1);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),1);
}

TEST(OrientedBoxMap, FloorIsNotRaisedIntoBoxAndRealLowObstacleStillRemains) {
  GridMap map;GridMapTestAccess::configure(map);
  map.setOccupied({.025,.025,0.});
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.025,.025,0.)),1);
  map.setOccupied({.025,.025,.31}); // Real obstacle top intersects box bottom .29.
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),1);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),1);
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.025,.025,0.)),1);
}

TEST(OrientedBoxMap, WeakHitIsInsufficientNotMeasuredFree) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::fillBoxFree(map,kBody,0.);
  ASSERT_EQ(map.getInflateOccupancy(kBody,0.),0);
  GridMapTestAccess::weakHit(map,kBody);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),2);
  const auto e=map.inspectInflateOccupancy(kBody,0.,true);
  EXPECT_EQ(e.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::Insufficient)],1U);
}

TEST(OrientedBoxMap, RawMutationInvalidatesCandidateAndStrictPoseCaches) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::fillBoxFree(map,kBody,0.);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);EXPECT_EQ(map.getInflateOccupancy(kBody,0.),0);
  map.setOccupied(kBody);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),1);EXPECT_EQ(map.getInflateOccupancy(kBody,0.),1);
  map.setOccupancy(kBody,0.);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);EXPECT_EQ(map.getInflateOccupancy(kBody,0.),0);
}

TEST(OrientedBoxMap, EvidenceExpiryDoesNotInventFreeOrRenewSourceTimes) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::fillBoxFree(map,kBody,0.);
  constexpr std::int64_t source=100000000000LL;
  GridMapTestAccess::measuredFreeTimes(map,source,source+100000000LL);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),0);
  GridMapTestAccess::queryTime(map,source+600000000LL);
  EXPECT_EQ(map.getInflateOccupancy(kBody,0.),2);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),0);
  const auto e=map.inspectInflateOccupancy(kBody,0.,true);
  EXPECT_GT(e.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::StaleFree)],0U);
  EXPECT_EQ(map.latestCloudStampNs(),source);EXPECT_EQ(map.integratedRaySourceStamp(0),source);
}

TEST(OrientedBoxMap, PrivateSnapshotRetainsShapeAndRawEvidenceWithoutAliasingWriter) {
  GridMap writer;GridMapTestAccess::configure(writer);
  writer.setOccupied(kBody);
  GridMap snapshot;
  writer.copyCollisionSnapshotTo(snapshot,100000000000LL,std::chrono::steady_clock::now());
  snapshot.requireObservedSnapshot();
  EXPECT_EQ(snapshot.bodyEnvelope().model,scan_planner::CollisionModel::OrientedBox);
  EXPECT_DOUBLE_EQ(snapshot.bodyEnvelope().length,.93);EXPECT_DOUBLE_EQ(snapshot.bodyEnvelope().width,.30);
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(kBody,0.),1);EXPECT_EQ(snapshot.getInflateOccupancy(kBody,0.),1);
  writer.setOccupancy(kBody,0.);
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(kBody,0.),1);
  EXPECT_EQ(writer.getPlanningInflateOccupancy(kBody,0.),0);
}

TEST(OrientedBoxMap, PartialOutsideFootprintIsRejectedButOccupiedWitnessTakesPriority) {
  GridMap map;GridMapTestAccess::configure(map);
  const Eigen::Vector3d p(.65,.025,.54);
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),-1);
  EXPECT_EQ(map.getInflateOccupancy(p,0.),-1);
  map.setOccupied({.70,.025,.54});
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);EXPECT_EQ(map.getInflateOccupancy(p,0.),1);
}

TEST(OrientedBoxMap, CandidateGeometryCannotWidenOrSilentlyWeakenRequestedStrictPolicy) {
  GridMap map;GridMapTestAccess::configure(map);
  EXPECT_THROW(GridMapTestAccess::candidateBelow(map,.26),std::invalid_argument);
  GridMapTestAccess::candidateBelow(map,.10);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().below,.25);EXPECT_DOUBLE_EQ(map.planningBodyEnvelope().below,.10);
  GridMapTestAccess::planningStrict(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),2);
  GridMap nonstrict;GridMapTestAccess::configure(nonstrict,false);
  EXPECT_THROW(GridMapTestAccess::candidateBelow(nonstrict,.10),std::invalid_argument);
}

TEST(OrientedBoxMap, BoxSupportAnchorIsIndependentOfPhysicalThickness) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::ground(map,2.);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().below,.25);EXPECT_DOUBLE_EQ(map.bodyEnvelope().above,.25);
  EXPECT_THROW(GridMapTestAccess::bodyBelow(map,-.2),std::invalid_argument);
  GridMap invalid;
  EXPECT_THROW(GridMapTestAccess::configure(invalid,true,.93,0.),std::invalid_argument);
  GridMap good;GridMapTestAccess::configure(good);
  EXPECT_THROW(GridMapTestAccess::model(good,"not_a_shape"),std::invalid_argument);
}

TEST(OrientedBoxMap, VisualizationUsesMeasuredYawAndDoesNotDrawOldCircularDilation) {
  GridMap map;GridMapTestAccess::configure(map);
  map.setOccupied(kBody);
  const auto before=GridMapTestAccess::raw(map);
  GridMapTestAccess::inflatedOnes(map); // Deliberately incompatible old display data.
  const auto raw=GridMapTestAccess::visualization(map,false);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U); // No invented yaw=0.
  GridMapTestAccess::bodyYaw(map,0.);
  const auto displayed=GridMapTestAccess::visualization(map,true);
  EXPECT_NE(displayed.data,raw.data);EXPECT_EQ(raw.width,1U);EXPECT_EQ(displayed.width,1463U);
  EXPECT_EQ(displayed.header.frame_id,"odom");
  EXPECT_EQ(GridMapTestAccess::visualizationBuilds(map)[0],1U);
  EXPECT_STREQ(map.inflatedVisualizationSemantics(),"occupied_oriented_box_current_body_attitude_coarse_voxels");
  EXPECT_EQ(GridMapTestAccess::raw(map),before);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,0.),1); // Display changes no decision.
}

TEST(OrientedBoxMap, InflatedDisplayMatchesActualOccupiedCentreQueriesAcrossYawAndRingWrap) {
  for(const Eigen::Vector3i &origin:std::vector<Eigen::Vector3i>{{0,0,0},{-21,17,-8},{25,-23,12}})
    for(double yaw:{0.,kPi*.25,kPi*.5,-.73}) {
      GridMap map;GridMapTestAccess::configure(map,false);
      GridMapTestAccess::displayExtent(map,{12,12,24},origin);
      const auto bounds=GridMapTestAccess::bounds(map);
      Eigen::Vector3d hit;map.indexToPos(origin,hit);map.setOccupied(hit);
      map.indexToPos(bounds.first+Eigen::Vector3i(1,2,3),hit);map.setOccupied(hit);
      GridMapTestAccess::bodyYaw(map,yaw);
      const auto before=GridMapTestAccess::raw(map);
      const auto expected=GridMapTestAccess::occupiedQueryCells(map,yaw);
      const auto actual=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
      EXPECT_EQ(actual,expected) << "origin="<<origin.transpose()<<" yaw="<<yaw;
      EXPECT_EQ(GridMapTestAccess::raw(map),before);
    }
}

TEST(OrientedBoxMap, InflatedDisplayReflectsAsymmetricHeightAndUsesHitsAboveOutputClip) {
  GridMap map;GridMapTestAccess::configure(map,false,.93,.30,.10,.30);
  map.setOccupied(kBody);GridMapTestAccess::bodyYaw(map,0.);
  auto cells=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
  EXPECT_EQ(cells.size(),1197U);
  int low=1000,high=-1000;
  for(const auto &cell:cells){low=std::min(low,cell[2]);high=std::max(high,cell[2]);}
  EXPECT_EQ(low,4);EXPECT_EQ(high,12); // hit.z cell10 minus query span[-2,6].
  GridMapTestAccess::displayClip(map,.275); // Body centres above cell5 are hidden.
  cells=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
  EXPECT_EQ(cells.size(),266U); // The higher obstacle still blocks lower centres.
  for(const auto &cell:cells)EXPECT_LE(cell[2],5);
}

TEST(OrientedBoxMap, InflatedDisplayCacheTracksRealRotationMapChangesAndInvalidBody) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,0.);
  const auto zero=GridMapTestAccess::visualization(map,true);
  const auto work=GridMapTestAccess::displayWork(map);
  for(int i=0;i<5;++i)EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,zero.data);
  EXPECT_EQ(GridMapTestAccess::displayWork(map),work);
  GridMapTestAccess::bodyYaw(map,kPi*.5,2000000000LL);
  const auto rotated=GridMapTestAccess::visualization(map,true);
  EXPECT_NE(rotated.data,zero.data);EXPECT_EQ(rotated.width,zero.width);
  const auto rotation_work=GridMapTestAccess::displayWork(map);
  EXPECT_EQ(std::get<0>(rotation_work),std::get<0>(work)+1);
  GridMapTestAccess::bodyYaw(map,0.,2000000000LL); // Duplicate cannot turn back.
  GridMapTestAccess::bodyYaw(map,0.,1500000000LL); // Nor reordered source.
  GridMapTestAccess::bodyYaw(map,0.,3000000000LL,"foreign");
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,rotated.data);
  EXPECT_EQ(GridMapTestAccess::displayWork(map),rotation_work);
  GridMapTestAccess::invalidBody(map,3000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U);
  GridMapTestAccess::bodyYaw(map,0.,4000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,zero.data);
  GridMapTestAccess::overflowBodyQuaternion(map,5000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U);
  GridMapTestAccess::bodyYaw(map,0.,6000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,zero.data);
  GridMapTestAccess::reset(map);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U);
}

TEST(OrientedBoxMap, InflatedDisplayDoesNotPromoteWeakUnknownOrNonfiniteEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::bodyYaw(map,.4);
  GridMapTestAccess::thresholdAndInvalid(map,{0,0,0},1.); // Exactly threshold is not occupied.
  GridMapTestAccess::thresholdAndInvalid(map,{1,0,0},.5);
  GridMapTestAccess::thresholdAndInvalid(map,{2,0,0},std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U);
  Eigen::Vector3d hit;map.indexToPos({3,0,0},hit);map.setOccupied(hit);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),
      GridMapTestAccess::occupiedQueryCells(map,.4));
}

TEST(OrientedBoxMap, DisplayEpochResetAndModelChangeCannotReuseOldOrientationCloud) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,0.);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,1463U);
  GridMapTestAccess::contextSession(map);
  ASSERT_TRUE(map.applyLocalizationContext(
      R"({"schema":1,"session_id":"display","epoch":1,"sequence":1,"seed_id":"seed","barrier_ns":3000000000})"));
  map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,kPi*.5,2000000000LL); // Delayed old context sample.
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,0U);
  GridMapTestAccess::bodyYaw(map,kPi*.5,4000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,1463U);
  GridMapTestAccess::inflatedOnes(map);
  GridMapTestAccess::model(map,"double_cylinder");
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,32U*32U*64U);
  GridMapTestAccess::model(map,"oriented_box");
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).width,1463U);
}

TEST(OrientedBoxMap, ActualYawWithIdenticalExactLatticeKernelReusesMaskWithoutQuantization) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,0.);
  const auto first=GridMapTestAccess::visualization(map,true);
  const auto work=GridMapTestAccess::displayWork(map);
  GridMapTestAccess::bodyYaw(map,1e-8,2000000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,first.data);
  const auto next=GridMapTestAccess::displayWork(map);
  EXPECT_EQ(std::get<0>(next),std::get<0>(work));
  EXPECT_EQ(std::get<1>(next),std::get<1>(work)+1); // Real yaw was inspected, not rounded to 0.
}

TEST(OrientedBoxMap, DisplayWorkDoesNotMutateCollisionCachesOrAllocateSnapshotMask) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,.6);
  EXPECT_EQ(map.getPlanningInflateOccupancy(kBody,.6),1);
  const auto before=GridMapTestAccess::collisionCaches(map);
  const auto raw=GridMapTestAccess::raw(map);
  EXPECT_GT(GridMapTestAccess::visualization(map,true).width,0U);
  EXPECT_EQ(GridMapTestAccess::collisionCaches(map),before);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,1000000000LL,std::chrono::steady_clock::now());
  EXPECT_EQ(GridMapTestAccess::displayStorage(snapshot),0U);
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(kBody,.6),1);
}

TEST(OrientedBoxMap, DefaultWindowDensePlaneDisplayHasBoundedRunWorkAndReusesStableMask) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::displayExtent(map,{240,240,60},{0,0,0});
  GridMapTestAccess::denseHorizontalPlane(map);GridMapTestAccess::bodyYaw(map,0.);
  const auto started=std::chrono::steady_clock::now();
  const auto first=GridMapTestAccess::visualization(map,true);
  const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  const auto work=GridMapTestAccess::displayWork(map);
  EXPECT_EQ(first.width,240U*240U*11U);
  EXPECT_LE(std::get<2>(work),240U*240U*133U);
  EXPECT_EQ(std::get<3>(work),240U*240U*61U);
  const auto cached_started=std::chrono::steady_clock::now();
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,first.data);
  const auto cached=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-cached_started).count();
  EXPECT_EQ(GridMapTestAccess::displayWork(map),work);
  // Observed software cost only, not a runtime deadline or physical claim.
  std::cout<<"OBB_DISPLAY_PERF cells=3456000 occupied=57600 points="<<first.width
      <<" intervals="<<std::get<2>(work)<<" writer_bytes="<<GridMapTestAccess::displayStorage(map)
      <<" build_ms="<<elapsed<<" cached_with_data_compare_ms="<<cached<<std::endl;
}

TEST(OrientedBoxMap, CompactRenderSnapshotPreservesBothDisplaysAndHasNoCollisionStorage) {
  for(const std::string& model:{std::string("oriented_box"),std::string("double_cylinder")})
    for(double yaw:{0.,.73,-1.2}) {
      GridMap map,view;GridMapTestAccess::configure(map,false,1.03,.4,.25,.25);
      GridMapTestAccess::displayExtent(map,{12,12,24},{-21,17,-8});
      GridMapTestAccess::comparisonInput(map,{-21,17,-8},true);
      GridMapTestAccess::model(map,model);GridMapTestAccess::bodyYaw(map,yaw);
      GridMapTestAccess::thresholdAndInvalid(map,{-21,17,-8},
        std::numeric_limits<double>::infinity());
      if(model!="oriented_box")GridMapTestAccess::inflatedOnes(map);
      const auto raw=GridMapTestAccess::visualization(map,false);
      const auto inflated=GridMapTestAccess::visualization(map,true);
      GridMapTestAccess::captureDisplay(map,view,1234567890LL);
      EXPECT_EQ(GridMapTestAccess::visualization(view,false).data,raw.data);
      EXPECT_EQ(GridMapTestAccess::visualization(view,true).data,inflated.data);
      EXPECT_EQ(GridMapTestAccess::displayStamp(view),1234567890LL);
      const auto storage=GridMapTestAccess::displayInputStorage(view);
      EXPECT_EQ(std::get<0>(storage),12U*12U*24U);
      EXPECT_EQ(std::get<1>(storage),0U);EXPECT_EQ(std::get<2>(storage),0U);
      EXPECT_EQ(std::get<3>(storage),model=="oriented_box"?0U:12U*12U*24U);
      EXPECT_THROW(view.getPlanningInflateOccupancy(kBody,yaw),std::logic_error);
      EXPECT_THROW(view.inspectInflateOccupancy(kBody,yaw),std::logic_error);
      GridMap collision;
      EXPECT_THROW(view.copyCollisionSnapshotTo(collision,1,std::chrono::steady_clock::now()),std::logic_error);
      EXPECT_THROW(map.copyCollisionSnapshotTo(view,1,std::chrono::steady_clock::now()),std::logic_error);
    }
}

TEST(OrientedBoxMap, BusyRenderDoesNotBlockFusionOwnerOrQueueAnotherFrame) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,.3);
  const auto expected=GridMapTestAccess::visualization(map,true).data;
  std::promise<void> entered,release;auto released=release.get_future().share();
  std::atomic<unsigned> frames{0};std::vector<unsigned char> actual;
  GridMapTestAccess::startDisplay(map,[&](GridMap& view) {
    actual=GridMapTestAccess::visualization(view,true).data;
    ++frames;entered.set_value();released.wait();
  });
  ASSERT_TRUE(GridMapTestAccess::submitDisplayWhenIdle(map,1000000000LL));
  entered.get_future().wait();
  const auto begin=std::chrono::steady_clock::now();
  EXPECT_FALSE(GridMapTestAccess::submitDisplay(map,2000000000LL));
  // Fusion can mutate the real map while its old display is being rendered.
  map.setOccupied({.7,.1,.5});
  const double owner_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-begin).count();
  EXPECT_LT(owner_ms,20.);
  release.set_value();GridMapTestAccess::stopDisplay(map);
  EXPECT_EQ(actual,expected);EXPECT_EQ(frames,1U);
}

TEST(OrientedBoxMap, ResetDiscardsAnInFlightDisplayWithoutWaitingOrRestamping) {
  GridMap map;GridMapTestAccess::configure(map,false);map.setOccupied(kBody);
  GridMapTestAccess::bodyYaw(map,.3);
  std::promise<void> rendered,release,finished;auto released=release.get_future().share();
  std::atomic<bool> may_publish{true};std::atomic<std::int64_t> capture{0};
  GridMapTestAccess::startDisplay(map,[&](GridMap& view) {
    GridMapTestAccess::visualization(view,true);rendered.set_value();released.wait();
    may_publish=GridMapTestAccess::displayCurrent(view);
    capture=GridMapTestAccess::displayStamp(view);
    finished.set_value();
  });
  ASSERT_TRUE(GridMapTestAccess::submitDisplayWhenIdle(map,1234567890LL));
  rendered.get_future().wait();GridMapTestAccess::reset(map);
  release.set_value();finished.get_future().wait();
  EXPECT_FALSE(may_publish);EXPECT_EQ(capture,1234567890LL);
  GridMapTestAccess::stopDisplay(map);
}

TEST(OrientedBoxMap, DisplayWorkerDestructorJoinsInFlightWorkWithoutMutexDeadlock) {
  auto map=std::make_unique<GridMap>();GridMapTestAccess::configure(*map,false);
  std::promise<void> entered,release;auto released=release.get_future().share();
  GridMapTestAccess::startDisplay(*map,[&](GridMap&) {entered.set_value();released.wait();});
  ASSERT_TRUE(GridMapTestAccess::submitDisplayWhenIdle(*map,1000000000LL));
  entered.get_future().wait();
  auto retiring=std::async(std::launch::async,[owned=std::move(map)]()mutable {owned.reset();});
  EXPECT_EQ(retiring.wait_for(std::chrono::milliseconds(10)),std::future_status::timeout);
  release.set_value();EXPECT_EQ(retiring.wait_for(std::chrono::seconds(1)),std::future_status::ready);
  retiring.get();
}

TEST(SpatialOrientedBox, ZeroAttitudeIsExactlyTheExistingVoxelSetAndMapPolicy) {
  for(double yaw:{0.,.31,1.2,2.8})for(double z:{.49,.5,.501}) {
    const Eigen::Vector3d p(.013,-.017,z);std::vector<Eigen::Vector3i> old_cells,new_cells;
    scan_planner::forEachOrientedBoxVoxel(p,yaw,.05,box(),[&](const auto& c){old_cells.push_back(c);return true;});
    scan_planner::forEachOrientedBoxVoxel(p,yaw,0.,-0.,.05,box(),[&](const auto& c){new_cells.push_back(c);return true;});
    ASSERT_EQ(old_cells.size(),new_cells.size());
    for(std::size_t i=0;i<old_cells.size();++i)EXPECT_EQ(old_cells[i],new_cells[i]);
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::comparisonInput(map,{-16,-16,-32},true);
    EXPECT_EQ(map.getInflateOccupancy(p,yaw),map.getInflateOccupancy(p,yaw,0.,0.));
    EXPECT_EQ(map.getPlanningInflateOccupancy(p,yaw),map.getPlanningInflateOccupancy(p,yaw,0.,0.));
  }
}

TEST(SpatialOrientedBox, SameFullBodyTiltsAboveTreadButNotThroughBodyRoofOrRailing) {
  const Eigen::Vector3d p(.025,.025,.50);const double pitch=-kPi/6.;
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4,.25,.25);
  const auto original=map.bodyEnvelope();map.setOccupied({.425,.025,.325});
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.,pitch,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(p,0.,pitch,0.),0);
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.,-pitch,0.),1); // Wrong pitch sign hits tread.
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().length,original.length);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().below,original.below);
  for(const Eigen::Vector3d obstacle:std::vector<Eigen::Vector3d>{{.025,.025,.525},{.175,.025,.875},{.025,.225,.525}}) {
    GridMap blocked;GridMapTestAccess::configure(blocked,false,1.03,.4,.25,.25);blocked.setOccupied(obstacle);
    EXPECT_EQ(blocked.getPlanningInflateOccupancy(p,0.,pitch,0.),1)<<obstacle.transpose();
    const auto evidence=blocked.inspectInflateOccupancy(p,0.,pitch,0.,true);
    EXPECT_GT(evidence.unique_counts[1],0U);EXPECT_EQ(evidence.counts,evidence.unique_counts);
  }
}

TEST(SpatialOrientedBox, UpDownMirrorRollAndAsymmetricVerticalOffsetAreGeometric) {
  const Eigen::Vector3d p(.013,.017,.5);const auto b=box(1.03,.4,.12,.25);
  scan_planner::OrientedBoxVoxel3DQuery up(p,.3,-.4,.15,.05,b);
  scan_planner::OrientedBoxVoxel3DQuery down(p,.3+kPi,.4,-.15,.05,b);
  // Reverse travel with opposite signed grade represents the same pitched
  // rigid body footprint; roll reverses as well, body Z stays unchanged.
  EXPECT_NEAR((up.center-down.center).norm(),0.,1e-12);
  for(int x=-12;x<=12;++x)for(int y=-12;y<=12;++y)for(int z=0;z<24;++z)
    EXPECT_EQ(up.intersects({x,y,z}),down.intersects({x,y,z}));
  const auto rotation=(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.15,Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_NEAR((up.center-(p+rotation.col(2)*(.25-.12)*.5)).norm(),0.,1e-12);
}

TEST(SpatialOrientedBox, RawUnknownInsufficientOutsideAndFreshnessArePreserved) {
  GridMap map;GridMapTestAccess::configure(map,true,1.03,.4);
  const Eigen::Vector3d p(.025,.025,.5);const double pitch=-.4;
  const auto raw=GridMapTestAccess::raw(map);
  EXPECT_EQ(map.getInflateOccupancy(p,0.,pitch,0.),2);
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.,pitch,0.),0);
  EXPECT_EQ(raw,GridMapTestAccess::raw(map)); // Query never converts unknown to free.
  scan_planner::forEachOrientedBoxVoxel(p,0.,pitch,0.,.05,map.bodyEnvelope(),[&](const auto& cell){
    Eigen::Vector3d q;map.indexToPos(cell,q);map.setOccupancy(q,0.);return true;
  });
  EXPECT_EQ(map.getInflateOccupancy(p,0.,pitch,0.),0);
  GridMapTestAccess::weakHit(map,p);EXPECT_EQ(map.getInflateOccupancy(p,0.,pitch,0.),2);
  map.setOccupied(p);EXPECT_EQ(map.getInflateOccupancy(p,0.,pitch,0.),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy({.79,.79,.5},0.,pitch,0.),-1);
  GridMap fresh;GridMapTestAccess::configure(fresh,true,1.03,.4);
  scan_planner::forEachOrientedBoxVoxel(p,0.,pitch,0.,.05,fresh.bodyEnvelope(),[&](const auto& cell){
    Eigen::Vector3d q;fresh.indexToPos(cell,q);fresh.setOccupancy(q,0.);return true;
  });
  GridMapTestAccess::measuredFreeTimes(fresh,1000000000LL,1100000000LL);
  EXPECT_EQ(fresh.getInflateOccupancy(p,0.,pitch,0.),0);
  GridMapTestAccess::queryTime(fresh,1600000000LL);EXPECT_EQ(fresh.getInflateOccupancy(p,0.,pitch,0.),2);
}

TEST(SpatialOrientedBox, ReaderProviderRestoresNestedScopeAndNeverLeaksAcrossSnapshots) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);
  const Eigen::Vector3d p(.025,.025,.5);map.setOccupied({.425,.025,.325});
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
  map.setQueryAttitudeProvider([](const auto&,double){return std::optional<Eigen::Vector2d>{{-kPi/6.,0.}};});
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.,0.,0.),1); // Explicit query cannot recurse into provider.
  auto previous=map.exchangeQueryAttitudeProvider([](const auto&,double){return std::optional<Eigen::Vector2d>{};});
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),2);map.exchangeQueryAttitudeProvider(std::move(previous));
  EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);
  GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,1000000000LL,std::chrono::steady_clock::now());
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(p,0.),1);
  map.clearQueryAttitudeProvider();EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
}

TEST(StairClearance, DisabledConfigIsExactNoOpAndInvalidNominalCannotShrinkBody) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);
  map.exchangeQueryStairClearanceProvider([](const auto&){return true;});
  EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(kBody).length,1.03);
  EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(kBody).width,.4);
  for(double clearance:{-.5,std::numeric_limits<double>::quiet_NaN(),.06})
    EXPECT_THROW(GridMapTestAccess::stairClearance(map,clearance),std::invalid_argument);
  EXPECT_THROW(GridMapTestAccess::stairClearance(map,.025,-1.,.30),std::invalid_argument);
  EXPECT_THROW(GridMapTestAccess::stairClearance(map,.025,.93,.4),std::invalid_argument);
  GridMapTestAccess::stairClearance(map);
  const auto body=map.queryBodyEnvelope(kBody);
  EXPECT_DOUBLE_EQ(body.length,.98);EXPECT_DOUBLE_EQ(body.width,.35);
  EXPECT_DOUBLE_EQ(body.below,.25);EXPECT_DOUBLE_EQ(body.above,.25);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().length,1.03); // Normal floor model is immutable.
}

TEST(StairClearance, ExactShapeCacheAndDiagnosticsAgreeWithoutMutatingRawEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);GridMapTestAccess::stairClearance(map);
  const Eigen::Vector3d low(0.,0.,.521),high(0.,0.,.522);
  map.setOccupied({.025,.225,.525}); // Full face touches y=.2; reduced y=.175 is clear.
  const auto raw=GridMapTestAccess::raw(map);
  map.exchangeQueryStairClearanceProvider([](const auto& p){return p.z()<.5215;});
  for(int repeat=0;repeat<2;++repeat) {
    EXPECT_EQ(map.getPlanningInflateOccupancy(low,0.),0);
    EXPECT_EQ(map.getPlanningInflateOccupancy(high,0.),1);
    EXPECT_EQ(map.getInflateOccupancy(low,0.),0);
    EXPECT_EQ(map.getInflateOccupancy(high,0.),1);
  }
  EXPECT_EQ(map.inspectInflateOccupancy(low,0.,true).counts[1],0U);
  EXPECT_GT(map.inspectInflateOccupancy(high,0.,true).counts[1],0U);
  EXPECT_NE(map.describeInflateOccupancy(low,0.).find("body_xy_box=[0.98,0.35]"),std::string::npos);
  EXPECT_NE(map.describeInflateOccupancy(high,0.).find("body_xy_box=[1.03,0.4]"),std::string::npos);
  // Pitch/roll path uses the exact same effective shape, not the yaw-only buffer.
  EXPECT_EQ(map.getPlanningInflateOccupancy(low,0.,-.1,0.),0);
  EXPECT_EQ(map.getPlanningInflateOccupancy(high,0.,-.1,0.),1);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairClearance, NestedScopeSnapshotAndUnknownPoliciesDoNotLeak) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);GridMapTestAccess::stairClearance(map);
  const Eigen::Vector3d body(0.,0.,.525);map.setOccupied({.025,.225,.525});
  map.exchangeQueryStairClearanceProvider([](const auto&){return true;});
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  auto previous=map.exchangeQueryStairClearanceProvider({});
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  map.exchangeQueryStairClearanceProvider(std::move(previous));
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,1000000000LL,std::chrono::steady_clock::now());
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(body,0.),1);
  EXPECT_EQ(GridMapTestAccess::raw(snapshot),GridMapTestAccess::raw(map));
  GridMap strict;GridMapTestAccess::configure(strict,true,1.03,.4);GridMapTestAccess::stairClearance(strict);
  strict.exchangeQueryStairClearanceProvider([](const auto&){return true;});
  EXPECT_EQ(strict.getInflateOccupancy(body,0.),2); // Never invent observed-free evidence.
  strict.setOccupied({.025,.025,.525});EXPECT_EQ(strict.getInflateOccupancy(body,0.),1);
}

TEST(StairClearance, VisualizationUsesMatchingImmutableBodySampleOnly) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);GridMapTestAccess::stairClearance(map);
  // At yaw zero both sizes may intersect the same 5 cm centre lattice owing
  // to touching contacts. Use a measured non-axis heading to expose the
  // changed kernel; no visual-only voxel removal is permitted.
  map.setOccupied({.025,.025,.525});GridMapTestAccess::bodyYaw(map,.31);
  const auto raw=GridMapTestAccess::raw(map);
  const auto normal=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
  auto reduced=map.planningBodyEnvelope();reduced.length=.98;reduced.width=.35;
  EXPECT_TRUE(map.setVisualizationBodyEnvelope(reduced,1000000000LL,"odom"));
  const auto small=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
  EXPECT_LT(small.size(),normal.size());
  EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(kBody).width,.4); // Display is non-authoritative.
  GridMap view;GridMapTestAccess::captureDisplay(map,view);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(view,true)),small);
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,999999999LL,"odom"));
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),small);
  EXPECT_TRUE(map.setVisualizationBodyEnvelope(reduced,1000000001LL,"odom"));
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),normal);
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,1000000002LL,"map"));
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),normal);
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,0,"odom"));
  reduced.width=-1.;EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,1000000000LL,"odom"));
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairClearance, AsynchronousDisplayKeepsShapeYawSourcePairedAndRejectsOldEpoch) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);GridMapTestAccess::stairClearance(map);
  map.setOccupied({.025,.025,.525});GridMapTestAccess::bodyYaw(map,.31,1000000000LL);
  auto reduced=map.planningBodyEnvelope();reduced.length=.98;reduced.width=.35;
  ASSERT_TRUE(map.setVisualizationBodyEnvelope(reduced,.31,1000000000LL,"odom"));
  const auto paired=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
  // New owner sample arrives before writer; its complete pair remains valid.
  ASSERT_TRUE(map.setVisualizationBodyEnvelope(reduced,.31,1020000000LL,"odom"));
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),paired);
  // Writer then runs ahead. Do not combine its new yaw with the old shape.
  GridMapTestAccess::bodyYaw(map,1.,1040000000LL);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),paired);
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,.1,1010000000LL,"odom"));
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,.1,1050000000LL,"map"));
  EXPECT_FALSE(map.setVisualizationBodyEnvelope(reduced,NAN,1050000000LL,"odom"));
  GridMap view;GridMapTestAccess::captureDisplay(map,view);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(view,true)),paired);
  // Existing max-pose-age rejects stale paired display, not a movement gate.
  GridMapTestAccess::bodyYaw(map,1.,1600000000LL);
  GridMap normal;GridMapTestAccess::configure(normal,false,1.03,.4);
  normal.setOccupied({.025,.025,.525});GridMapTestAccess::bodyYaw(normal,1.,1600000000LL);
  const auto fallback=GridMapTestAccess::cells(GridMapTestAccess::visualization(normal,true));
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),fallback);
  ASSERT_TRUE(map.setVisualizationBodyEnvelope(reduced,.31,1600000000LL,"odom"));
  GridMapTestAccess::displayBarrier(map,1600000000LL);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),fallback);
  GridMap epoch_view;GridMapTestAccess::captureDisplay(map,epoch_view);
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(epoch_view,true)),fallback);
  GridMapTestAccess::reset(map);map.setOccupied({.025,.025,.525});
  EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true)),fallback);
}

// The compatibility switch lets the same regression exercise the frozen
// pre-fix yaw-only publisher/library, without inventing another map kernel.
static bool setSpatialDisplay(GridMap& map,const scan_planner::BodyEnvelope& body,
    double yaw,double pitch,double roll,std::int64_t stamp=1000000000LL,
    const std::string& frame="odom") {
#ifdef D1MAX_TEST_PRE_SPATIAL_DISPLAY
  (void)pitch;(void)roll;
  return map.setVisualizationBodyEnvelope(body,yaw,stamp,frame);
#else
  return map.setVisualizationBodyEnvelope(body,yaw,pitch,roll,stamp,frame);
#endif
}

TEST(StairDisplayAttitude, InflatedCellsMatchRealTiltedCoarseQueriesAcrossSlopesRollAndWrap) {
  for(const auto& angle:std::vector<Eigen::Vector3d>{{0.,-.5,0.},{.7,.5,.2},{-.9,-.4,-.25}}) {
    GridMap map;GridMapTestAccess::configure(map,false,.98,.35,.12,.25);
    const Eigen::Vector3i origin(-13,9,-7);
    GridMapTestAccess::displayExtent(map,{18,18,28},origin);
    const auto bounds=GridMapTestAccess::bounds(map);
    for(const auto& cell:std::vector<Eigen::Vector3i>{origin,bounds.first+Eigen::Vector3i(2,3,4),
        bounds.second-Eigen::Vector3i(3,2,4)}) {
      Eigen::Vector3d hit;map.indexToPos(cell,hit);map.setOccupied(hit);
    }
    GridMapTestAccess::bodyYaw(map,angle.x());
    ASSERT_TRUE(setSpatialDisplay(map,map.planningBodyEnvelope(),angle.x(),angle.y(),angle.z()));
    const auto before=GridMapTestAccess::raw(map);
    const auto expected=GridMapTestAccess::occupiedQueryCells(map,angle.x(),angle.y(),angle.z());
    const auto actual=GridMapTestAccess::cells(GridMapTestAccess::visualization(map,true));
    EXPECT_EQ(actual,expected)<<"yaw pitch roll="<<angle.transpose();
    EXPECT_EQ(GridMapTestAccess::raw(map),before);
    GridMap view;GridMapTestAccess::captureDisplay(map,view);
    EXPECT_EQ(GridMapTestAccess::cells(GridMapTestAccess::visualization(view,true)),expected);
  }
}

TEST(StairDisplayAttitude, FlatBytesUnchangedAndAttitudeCacheKeepsWholeOriginalSample) {
  GridMap map;GridMapTestAccess::configure(map,false,.98,.35);
  map.setOccupied({.025,.025,.525});GridMapTestAccess::bodyYaw(map,.31);
  const auto flat=GridMapTestAccess::visualization(map,true).data;
  ASSERT_TRUE(setSpatialDisplay(map,map.planningBodyEnvelope(),.31,0.,0.));
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,flat);
  ASSERT_TRUE(setSpatialDisplay(map,map.planningBodyEnvelope(),.31,-.5,.15,1020000000LL));
  const auto tilted=GridMapTestAccess::visualization(map,true).data;
  EXPECT_NE(tilted,flat);
  const auto work=GridMapTestAccess::displayWork(map);
  for(int i=0;i<3;++i)EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,tilted);
  EXPECT_EQ(GridMapTestAccess::displayWork(map),work);
  GridMapTestAccess::bodyYaw(map,1.,1040000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,tilted);
  EXPECT_FALSE(setSpatialDisplay(map,map.planningBodyEnvelope(),.1,.5,0.,1010000000LL));
  EXPECT_FALSE(setSpatialDisplay(map,map.planningBodyEnvelope(),.1,.5,0.,1050000000LL,"map"));
#ifndef D1MAX_TEST_PRE_SPATIAL_DISPLAY
  EXPECT_FALSE(setSpatialDisplay(map,map.planningBodyEnvelope(),.1,NAN,0.,1050000000LL));
  EXPECT_FALSE(setSpatialDisplay(map,map.planningBodyEnvelope(),.1,.5,NAN,1050000000LL));
#endif
  // Stale metadata falls back to the existing upright/yaw-only display only;
  // it neither changes collisions nor extends the old pose-age deadline.
  GridMapTestAccess::bodyYaw(map,.31,1800000000LL);
  EXPECT_EQ(GridMapTestAccess::visualization(map,true).data,flat);
}

TEST(SpatialOrientedBox, SparseOfficialFastQueryExactlyMatchesFullSatWithRingIndices) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4,.12,.25);
  for(bool occupied:{false,true})
  for(const Eigen::Vector3i origin:std::vector<Eigen::Vector3i>{{0,0,0},{-16,-16,-32},{17,-7,13}}) {
    GridMapTestAccess::comparisonInput(map,origin,occupied);
    const auto bounds=GridMapTestAccess::bounds(map);
    const Eigen::Vector3d center=(bounds.first.cast<double>()+bounds.second.cast<double>())*.025;
    for(int i=0;i<80;++i) {
      const Eigen::Vector3d p=center+Eigen::Vector3d((i%7-3)*.13,(i%9-4)*.11,(i%11-5)*.08);
      const double yaw=i*.731,pitch=(i%13-6)*.09,roll=(i%5-2)*.08;
      EXPECT_EQ(map.getPlanningInflateOccupancy(p,yaw,pitch,roll),GridMapTestAccess::scalarSpatialQuery(map,p,yaw,pitch,roll))
          <<"occupied="<<occupied<<" ring_origin="<<origin.transpose()<<" case="<<i;
    }
  }
}

TEST(SpatialOrientedBox, ProviderDiagnosticsUseEffectiveAttitudeAndSamplingExtent) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);
  const Eigen::Vector3d p(.025,.025,.5);map.setOccupied({.425,.025,.325});
  const double flat_extent=scan_planner::bodyExtent(map.planningBodyEnvelope());
  EXPECT_DOUBLE_EQ(map.queryBodyExtent(),flat_extent);
  EXPECT_GT(map.inspectInflateOccupancy(p,0.,true).counts[1],0U);
  map.setQueryAttitudeProvider([](const auto&,double){return std::optional<Eigen::Vector2d>{{-kPi/6.,0.}};});
  EXPECT_DOUBLE_EQ(map.queryBodyExtent(),std::hypot(flat_extent,.25));
  EXPECT_DOUBLE_EQ(map.queryBodyExtent(false),std::hypot(flat_extent,.25));
  EXPECT_EQ(map.getInflateOccupancy(p,0.),0);
  EXPECT_EQ(map.inspectInflateOccupancy(p,0.,true).counts[1],0U);
  EXPECT_GT(map.inspectInflateOccupancy(p,0.,0.,0.,true).counts[1],0U);
  const auto diagnostic=map.describeInflateOccupancy(p,0.);
  EXPECT_NE(diagnostic.find("collision=0"),std::string::npos);
  EXPECT_NE(diagnostic.find("occupied=0"),std::string::npos);
  EXPECT_NE(diagnostic.find("pitch=-0.523598776 roll=0"),std::string::npos);
  EXPECT_NE(diagnostic.find("body_geometry=spatial_obb"),std::string::npos);
  EXPECT_EQ(diagnostic,map.describeInflateOccupancy(p,0.,-kPi/6.,0.));
  const auto raw_before=GridMapTestAccess::raw(map);
  const auto revision_before=map.occupancyRevision();
  const auto literal=map.describeInflateOccupancy(p,0.,0.,0.);
  EXPECT_NE(literal.find("collision=1"),std::string::npos);
  EXPECT_NE(literal.find("occupied=1"),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw_before);
  EXPECT_EQ(map.occupancyRevision(),revision_before);
  EXPECT_EQ(map.getInflateOccupancy(p,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(p,0.,0.,0.),1);
  EXPECT_EQ(diagnostic.find("body_z=[0.25,0.75]"),std::string::npos);
  map.setQueryAttitudeProvider([](const auto&,double){return std::optional<Eigen::Vector2d>{};});
  const auto unavailable=map.inspectInflateOccupancy(p,0.,true);
  EXPECT_FALSE(unavailable.attitude_resolved);EXPECT_EQ(unavailable.state(),2);
  EXPECT_TRUE(unavailable.voxels.empty());
  EXPECT_EQ(std::accumulate(unavailable.counts.begin(),unavailable.counts.end(),0U),0U);
  EXPECT_NE(map.describeInflateOccupancy(p,0.).find("attitude=unavailable evidence_counts=not_queried"),std::string::npos);
  EXPECT_NE(map.describeInflateOccupancy(p,0.,0.,0.).find("collision=1"),std::string::npos);
  EXPECT_EQ(map.describeInflateOccupancy(p,0.,std::numeric_limits<double>::quiet_NaN(),0.),
            "nonfinite_query_attitude");
  map.clearQueryAttitudeProvider();EXPECT_DOUBLE_EQ(map.queryBodyExtent(),flat_extent);
  EXPECT_GT(map.inspectInflateOccupancy(p,0.,true).counts[1],0U);
}

TEST(SpatialOrientedBox, DiagnosticWitnessListIsBoundedAndDoesNotChangeOccupancy) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);
  const Eigen::Vector3d p(.025,.025,.5);
  for(int x=-8;x<=8;++x)for(int y=-2;y<=2;++y)
    map.setOccupied({(x+.5)*.05,(y+.5)*.05,.525});
  const auto before=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
  const auto evidence=map.inspectInflateOccupancy(p,0.,0.,0.,true);
  ASSERT_GT(evidence.unique_counts[1],64U);
  const auto diagnostic=map.describeInflateOccupancy(p,0.,0.,0.);
  const auto begin=diagnostic.find(" occupied_witnesses=[");
  const auto end=diagnostic.find("] occupied_witnesses_omitted=",begin);
  ASSERT_NE(begin,std::string::npos);ASSERT_NE(end,std::string::npos);
  const auto list=diagnostic.substr(begin,end-begin);
  EXPECT_EQ(std::count(list.begin(),list.end(),'['),65); // Outer list plus 64 XYZ triples.
  EXPECT_NE(diagnostic.find("occupied_witnesses_omitted="+std::to_string(evidence.unique_counts[1]-64)),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::raw(map),before);EXPECT_EQ(map.occupancyRevision(),revision);
  EXPECT_EQ(map.getInflateOccupancy(p,0.,0.,0.),1);
}

TEST(SpatialOrientedBox, DiagnosticRetainsMillimetresAtFarMapCoordinates) {
  GridMap map;GridMapTestAccess::configure(map,false,1.03,.4);
  GridMapTestAccess::displayExtent(map,{32,32,64},{-800,820,-14});
  map.setOccupied({-40.175,40.775,-.975});
  const Eigen::Vector3d p(-39.9956,41.1693,-.7210);
  const auto diagnostic=map.describeInflateOccupancy(p,-1.5575,.01468,.01004);
  EXPECT_NE(diagnostic.find("query_xyz=[-39.9956,41.1693,-0.721]"),std::string::npos);
  EXPECT_NE(diagnostic.find("occupied_witnesses=[[-40.175,40.775,-0.975]]"),std::string::npos);
  EXPECT_NE(diagnostic.find("occupied_cell=(-40.175,40.775,-0.975)"),std::string::npos);
}

namespace {
const Eigen::Vector3d kLowHit(.001,.001,.001),kHighHit(.001,.001,.049);
const Eigen::Vector3d kFineBody(0.,0.,.2751);
void enableStairHitScope(GridMap& map) {
  map.exchangeQueryStairClearanceProvider([](const Eigen::Vector3d&){return true;});
}
}

TEST(StairHitSubvoxels, LiteralDown851EndpointRefinesVoxelWithoutChangingPoseOrRawOccupancy) {
  GridMap map;GridMapTestAccess::configure(map,false,.98,.35);
  GridMapTestAccess::subvoxels(map);
  GridMapTestAccess::displayExtent(map,{32,32,64},{-596,1010,-3});
  const Eigen::Vector3d point(-29.55028533935547,50.956756591796875,-.33488547801971436);
  const Eigen::Vector3d pose(-29.79066010599027,50.495704650878906,-.13168638944625854);
  const double yaw=-2.1278084365259176,pitch=.16227798675830324,roll=-.06378701475676254;
  map.setOccupiedHit(point);
  const auto raw=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
  EXPECT_EQ(map.getInflateOccupancy(pose,yaw,pitch,roll),1); // Ordinary floor unchanged.
  enableStairHitScope(map);
  EXPECT_EQ(map.getInflateOccupancy(pose,yaw,pitch,roll),0);
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),0);
  const auto evidence=map.inspectInflateOccupancy(pose,yaw,pitch,roll,true);
  EXPECT_EQ(evidence.unique_counts[1],1U);EXPECT_EQ(evidence.counts[1],1U);
  EXPECT_TRUE(evidence.subvoxel_narrowphase);EXPECT_EQ(evidence.subvoxels.refined_coarse,1U);
  EXPECT_EQ(evidence.subvoxels.tested_subcells,1U);EXPECT_EQ(evidence.subvoxels.intersecting_subcells,0U);
  EXPECT_EQ(evidence.subvoxels.coarse_fallback,0U);
  const auto diagnostic=map.describeInflateOccupancy(pose,yaw,pitch,roll);
  EXPECT_NE(diagnostic.find("hit_subcells_intersecting=0"),std::string::npos);
  EXPECT_EQ(raw,GridMapTestAccess::raw(map));EXPECT_EQ(revision,map.occupancyRevision());
  map.exchangeQueryStairClearanceProvider({});
  EXPECT_EQ(map.getInflateOccupancy(pose,yaw,pitch,roll),1);
}

TEST(StairHitSubvoxels, LiteralUp2068AllHitBoundsBelowRotatedBottomDoNotBecomeSideIntrusion) {
  GridMap map;GridMapTestAccess::configure(map,false,.98,.35);
  GridMapTestAccess::allHitBounds(map);
  GridMapTestAccess::displayExtent(map,{32,32,64},{-655,1002,20});
  // scan.log: 2026-11-04 20:01:10.189128, curve 2068. Use the original
  // complete ALL-hit endpoints, real pose and unchanged .98 x .35 x .50 OBB.
  const Eigen::Vector3d pose(-32.7467301,50.1403903,.98774152);
  const Eigen::Vector3d lo(-32.9207843,49.9466667,.562745098);
  const Eigen::Vector3d hi(-32.9017647,49.9498039,.580588235);
  const double yaw=1.39671771,pitch=-.459606157,roll=-.020741091;
  map.setOccupiedHit(lo);map.setOccupiedHit(hi);
  const auto raw=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1); // Flat/unscoped unchanged.
  enableStairHitScope(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),0);
  EXPECT_EQ(map.getInflateOccupancy(pose,yaw,pitch,roll),0);
  EXPECT_EQ(raw,GridMapTestAccess::raw(map));EXPECT_EQ(revision,map.occupancyRevision());
  EXPECT_EQ(map.getOccupancy(lo),1); // Never erase a support/obstacle voxel.
  GridMapTestAccess::discardHitBounds(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1); // Missing bounds stays coarse.
}

TEST(StairHitSubvoxels, RotatedBottomProofKeepsRailThickVoxelMixedAndTouchingHits) {
  const Eigen::Vector3d pose(0.,0.,.5);const double yaw=.3,pitch=-.45,roll=.08;
  const Eigen::Matrix3d r=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
  for(const Eigen::Vector3d local:{Eigen::Vector3d(0.,.174,0.), // true side rail
      Eigen::Vector3d(.3,0.,-.25),Eigen::Vector3d(.3,0.,-.249)}) { // touch / low obstacle
    GridMap map;GridMapTestAccess::configure(map,false,.98,.35);
    GridMapTestAccess::allHitBounds(map);enableStairHitScope(map);
    const auto hit=pose+r*local;map.setOccupiedHit(hit);
    EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1)<<local.transpose();
  }
  GridMap map;GridMapTestAccess::configure(map,false,.98,.35);
  GridMapTestAccess::allHitBounds(map);enableStairHitScope(map);
  const Eigen::Vector3d below=pose+r*Eigen::Vector3d(.3,0.,-.265);
  map.setOccupiedHit(below);
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),0);
  const Eigen::Vector3d inside=pose+r*Eigen::Vector3d(.3,0.,-.245);
  map.setOccupiedHit(inside); // Union must retain a higher intrusion in the same coarse voxel.
  EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1);
  map.setOccupiedHit(below);EXPECT_EQ(map.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1);
  GridMap thick;GridMapTestAccess::configure(thick,false,.98,.35);
  GridMapTestAccess::allHitBounds(thick);enableStairHitScope(thick);
  thick.setOccupancy(below,1.); // A coarse/thick obstacle is not an ALL-hit endpoint union.
  EXPECT_EQ(thick.getPlanningInflateOccupancy(pose,yaw,pitch,roll),1);
}

TEST(StairHitSubvoxels, AllHitBottomPlaneHandlesAsymmetricHeightAndConservativeAabb) {
  scan_planner::BodyEnvelope body;body.model=scan_planner::CollisionModel::OrientedBox;
  body.length=.98;body.width=.35;body.below=.12;body.above=.25;
  const Eigen::Vector3d p(.4,-.2,.7);const double yaw=.8,pitch=-.5,roll=.2;
  const Eigen::Matrix3d r=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
  scan_planner::OrientedBoxVoxel3DQuery query(p,yaw,pitch,roll,.05,body);
  const Eigen::Vector3d below=p-r.col(2)*.13,touch=p-r.col(2)*.12;
  EXPECT_TRUE(query.allHitBoundsBelowBody(below,below));
  EXPECT_FALSE(query.allHitBoundsBelowBody(touch,touch));
  EXPECT_FALSE(query.allHitBoundsBelowBody(below-Eigen::Vector3d::Constant(.02),
    below+Eigen::Vector3d::Constant(.02))); // Whole source union, not a selected low endpoint.
  EXPECT_FALSE(query.allHitBoundsBelowBody(below+Eigen::Vector3d::Ones(),below));
  EXPECT_FALSE(query.allHitBoundsBelowBody(Eigen::Vector3d::Constant(NAN),below));
}

TEST(StairHitSubvoxels, UnionRetainsOldHitsAndNewIntrusionInvalidatesPreviouslyClearQuery) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::subvoxels(map);enableStairHitScope(map);
  map.setOccupiedHit(kLowHit);EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),0);
  const auto revision=map.occupancyRevision();
  map.setOccupiedHit(kHighHit);
  EXPECT_GT(map.occupancyRevision(),revision);
  EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,kLowHit),3U);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
  map.setOccupiedHit(kLowHit);EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
  const auto evidence=map.inspectInflateOccupancy(kFineBody,0.,true);
  EXPECT_EQ(evidence.subvoxels.tested_subcells,2U);EXPECT_EQ(evidence.subvoxels.intersecting_subcells,1U);
}

TEST(StairHitSubvoxels, ExactBodyHeightMattersEvenWithinSameCoarseSpan) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::subvoxels(map);enableStairHitScope(map);map.setOccupiedHit(kLowHit);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),0);
  EXPECT_EQ(map.getInflateOccupancy({0.,0.,.2749},0.),1);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),0);
}

TEST(StairHitSubvoxels, VerticalHalfRetainsWholeHorizontalObstacleFootprint) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::subvoxels(map);enableStairHitScope(map);
  const Eigen::Vector3d body(0.,0.,.5),hit(.499,.001,.301);
  map.setOccupiedHit(hit);
  const Eigen::Vector3i fine=(hit/.025).array().floor().cast<int>();
  // The earlier full-XYZ refinement would discard this horizontal contact.
  const scan_planner::OrientedBoxVoxel3DQuery tiny(body,0.,0.,0.,.025,map.bodyEnvelope());
  EXPECT_FALSE(tiny.intersects(fine));
  EXPECT_EQ(map.getInflateOccupancy(body,0.),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  const auto evidence=map.inspectInflateOccupancy(body,0.,true);
  EXPECT_EQ(evidence.subvoxels.tested_subcells,1U);
  EXPECT_EQ(evidence.subvoxels.intersecting_subcells,1U);
  EXPECT_NE(map.describeInflateOccupancy(body,0.).find(
      "hit_subvoxel_geometry=coarse_xy_vertical_halves"),std::string::npos);
}

TEST(StairHitSubvoxels, HorizontalEndpointOctantsAreAggregatedWithoutLosingHistory) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::subvoxels(map);enableStairHitScope(map);
  for(const Eigen::Vector3d hit:{Eigen::Vector3d(.001,.001,.001),Eigen::Vector3d(.049,.001,.001),
      Eigen::Vector3d(.001,.049,.001),Eigen::Vector3d(.049,.049,.001)})map.setOccupiedHit(hit);
  EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,kLowHit),0x55U);
  const auto evidence=map.inspectInflateOccupancy(kFineBody,0.,true);
  EXPECT_EQ(evidence.subvoxels.refined_coarse,1U);
  EXPECT_EQ(evidence.subvoxels.tested_subcells,1U);
  EXPECT_EQ(evidence.subvoxels.intersecting_subcells,0U);
  map.setOccupiedHit({.049,.049,.049});
  EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,kLowHit),0xd5U);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
}

TEST(StairHitSubvoxels, ManualCoarseAndWeakHistoryCannotBeExplainedByLaterSingleHit) {
  for(bool weak:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);
    GridMapTestAccess::subvoxels(map);enableStairHitScope(map);
    if(weak)GridMapTestAccess::weakHit(map,kLowHit);else map.setOccupied(kLowHit);
    map.setOccupiedHit(kLowHit);
    EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
    EXPECT_NE(GridMapTestAccess::subvoxelEvidence(map,kLowHit)&scan_planner::kCoarseOnlyHitEvidence,0);
    const auto evidence=map.inspectInflateOccupancy(kFineBody,0.,true);
    EXPECT_EQ(evidence.subvoxels.coarse_fallback,1U);EXPECT_EQ(evidence.subvoxels.refined_coarse,0U);
  }
}

TEST(StairHitSubvoxels, OrdinaryFreeResetRetiresHistoryButWeakOddsDoNot) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::subvoxels(map);enableStairHitScope(map);
  map.setOccupiedHit(kHighHit);
  GridMapTestAccess::integratedOdds(map,kHighHit,.5); // Evidence insufficient, not fully free.
  map.setOccupiedHit(kLowHit);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
  map.setOccupancy(kHighHit,0.);
  EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,kLowHit),0U);
  map.setOccupiedHit(kLowHit);EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),0);
  map.setOccupied(kLowHit);EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
  map.setOccupancy(kLowHit,0.);map.setOccupiedHit(kLowHit);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),0);
}

TEST(StairHitSubvoxels, DisabledMissingScopeAndStrictPolicyKeepCoarseSemantics) {
  GridMap disabled;GridMapTestAccess::configure(disabled,false);enableStairHitScope(disabled);
  disabled.setOccupiedHit(kLowHit);EXPECT_EQ(disabled.getInflateOccupancy(kFineBody,0.),1);
  EXPECT_TRUE(GridMapTestAccess::subvoxelStorage(disabled).empty());
  GridMap strict;GridMapTestAccess::configure(strict,true);
  GridMapTestAccess::subvoxels(strict);enableStairHitScope(strict);
  strict.setOccupiedHit(kLowHit);EXPECT_EQ(strict.getInflateOccupancy(kFineBody,0.),1);
  GridMapTestAccess::planningStrict(strict);EXPECT_EQ(strict.getPlanningInflateOccupancy(kFineBody,0.),1);
  EXPECT_FALSE(strict.inspectInflateOccupancy(kFineBody,0.,true).subvoxel_narrowphase);
  GridMap unknown;GridMapTestAccess::configure(unknown,false);GridMapTestAccess::subvoxels(unknown);
  enableStairHitScope(unknown);EXPECT_EQ(unknown.getInflateOccupancy(kFineBody,0.),0);
  EXPECT_GT(unknown.inspectInflateOccupancy(kFineBody,0.,true).counts[2],0U);
  EXPECT_EQ(unknown.getInflateOccupancy({5.,5.,5.},0.),-1);
}

TEST(StairHitSubvoxels, SnapshotOwnsCompleteEvidenceAndDoesNotInheritScope) {
  GridMap writer;GridMapTestAccess::configure(writer,false);GridMapTestAccess::subvoxels(writer);
  enableStairHitScope(writer);writer.setOccupiedHit(kLowHit);
  GridMap snapshot;writer.copyCollisionSnapshotTo(snapshot,1,std::chrono::steady_clock::now());
  EXPECT_EQ(GridMapTestAccess::subvoxelStorage(snapshot),GridMapTestAccess::subvoxelStorage(writer));
  EXPECT_EQ(snapshot.getInflateOccupancy(kFineBody,0.),1); // No inherited region capability.
  enableStairHitScope(snapshot);EXPECT_EQ(snapshot.getInflateOccupancy(kFineBody,0.),0);
  writer.setOccupiedHit(kHighHit);EXPECT_EQ(writer.getInflateOccupancy(kFineBody,0.),1);
  EXPECT_EQ(snapshot.getInflateOccupancy(kFineBody,0.),0);
  GridMapTestAccess::reset(writer);EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(writer,kLowHit),0U);
  EXPECT_EQ(snapshot.getInflateOccupancy(kFineBody,0.),0);
}

TEST(StairHitSubvoxels, RingReuseAndLargeSlideCannotRetainHitMasks) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
  const Eigen::Vector3d departing(-.799,.001,.001),new_cell(.801,.001,.001);
  map.setOccupiedHit(departing);EXPECT_NE(GridMapTestAccess::subvoxelEvidence(map,departing),0U);
  GridMapTestAccess::slideEvidence(map,{.051,0.,0.});
  EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,new_cell),0U);
  map.setOccupiedHit(new_cell);EXPECT_EQ(GridMapTestAccess::subvoxelEvidence(map,new_cell),1U);
  GridMapTestAccess::slideEvidence(map,{10.,0.,0.});
  const auto evidence=GridMapTestAccess::subvoxelStorage(map);
  EXPECT_TRUE(std::all_of(evidence.begin(),evidence.end(),[](auto item){return item==0;}));
  EXPECT_EQ(evidence.size(),32U*32U*64U);
}

TEST(StairHitSubvoxels, OriginalPointInsideBodyAndTouchingSubcellAlwaysBlock) {
  for(const Eigen::Vector3d point:{Eigen::Vector3d(.001,.001,.30),Eigen::Vector3d(.40,.10,.4),
      Eigen::Vector3d(.001,.001,.049)}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
    enableStairHitScope(map);map.setOccupiedHit(point);
    EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
  }
  GridMap touching;GridMapTestAccess::configure(touching,false);GridMapTestAccess::subvoxels(touching);
  enableStairHitScope(touching);touching.setOccupiedHit(kLowHit);
  EXPECT_EQ(touching.getInflateOccupancy({0.,0.,.275},0.),1);
}

TEST(StairHitSubvoxels, NegativeCoordinatesAndHalfCellBoundaryRetainOriginalLattice) {
  using scan_planner::hitSubvoxelBit;
  EXPECT_EQ(hitSubvoxelBit({-.049,-.049,-.049},{-1,-1,-1},.05),1U);
  EXPECT_EQ(hitSubvoxelBit({-.001,-.001,-.001},{-1,-1,-1},.05),128U);
  EXPECT_EQ(hitSubvoxelBit({-.025,-.025,-.025},{-1,-1,-1},.05),128U);
  EXPECT_EQ(hitSubvoxelBit({.025,.025,.025},{0,0,0},.05),128U);
  EXPECT_EQ(hitSubvoxelBit({.05,.025,.025},{0,0,0},.05),0U); // Caller disagreement -> coarse.
  EXPECT_EQ(hitSubvoxelBit({.05,.025,.025},{1,0,0},.05),8U);
}

TEST(StairHitSubvoxels, InvalidOddsDoNotBecomeOrdinaryUnknownOrCompleteFineHistory) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
  enableStairHitScope(map);map.setOccupiedHit(kLowHit);
  GridMapTestAccess::integratedOdds(map,kLowHit,std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),2);
  map.setOccupiedHit(kLowHit);
  EXPECT_NE(GridMapTestAccess::subvoxelEvidence(map,kLowHit)&scan_planner::kCoarseOnlyHitEvidence,0);
  EXPECT_EQ(map.getInflateOccupancy(kFineBody,0.),1);
}

TEST(StairHitSubvoxels, BoundedStorageIsCountedForWriterAndEachIndependentSnapshot) {
  GridMap baseline,writer;GridMapTestAccess::configure(baseline,false);
  GridMapTestAccess::configure(writer,false);GridMapTestAccess::subvoxels(writer);
  const std::size_t additional=32U*32U*64U*sizeof(std::uint16_t);
  EXPECT_EQ(writer.collisionSnapshotBytes(),baseline.collisionSnapshotBytes()+additional);
  GridMap plain_reader,reader;
  baseline.copyCollisionSnapshotTo(plain_reader,1,std::chrono::steady_clock::now());
  writer.copyCollisionSnapshotTo(reader,1,std::chrono::steady_clock::now());
  EXPECT_EQ(reader.collisionSnapshotBytes(),plain_reader.collisionSnapshotBytes()+additional);
}
