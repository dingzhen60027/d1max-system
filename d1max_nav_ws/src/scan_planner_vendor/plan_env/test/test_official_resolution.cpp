#include <gtest/gtest.h>
#include "plan_env/grid_map.h"

#include <cmath>
#include <limits>
#include <vector>

// Exercise production indexing, inflation updates and two-center queries only.
// No ROS context, node, executor, sensor transport or synthetic free-space fill.
struct GridMapTestAccess {
  static void configure(GridMap &map, double resolution,bool strict=false,
      double planning_up=-1.,double planning_down=-1.,double strict_up=.45) {
    auto &p=map.mp_;auto &d=map.md_;
    p.require_observed_free_=strict;p.use_projected_rays_=false;
    p.resolution_=resolution;p.resolution_inv_=1./resolution;
    p.map_voxel_num_={32,32,64};p.map_origin_idx_.setZero();
    map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;
    p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    // D1 body dimensions are unchanged when selecting the official resolution.
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=strict_up;p.obstacles_inflation_z_down=.45;
    p.planning_obstacles_inflation_z_up=planning_up;
    p.planning_obstacles_inflation_z_down=planning_down;
    const std::size_t size=32U*32U*64U;
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);
    d.count_hit_and_miss_.resize(size);d.flag_rayend_.resize(size);
    d.flag_traverse_.resize(size);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static void reset(GridMap &map) {map.resetAllMapData();}
  static int zIndex(GridMap &map, double z) {
    Eigen::Vector3i cell;map.posToIndex({0.,0.,z},cell);return cell.z();
  }
  static int upperInflationStep(GridMap &map) {
    int result=0;
    for(const auto &offset:map.md_.inflate_offsets_) result=std::max(result,offset.z());
    return result;
  }
  static std::size_t kernelSize(GridMap &map) {return map.md_.inflate_offsets_.size();}
  static auto raw(const GridMap &map) {return map.md_.occupancy_buffer_;}
  static auto strictBuffer(const GridMap &map) {return map.md_.occupancy_buffer_inflate_;}
  static auto planningBuffer(const GridMap &map) {return map.md_.planning_occupancy_buffer_inflate_;}
  static void rebuildPlanning(GridMap &map,double up,double down) {
    map.mp_.planning_obstacles_inflation_z_up=up;
    map.mp_.planning_obstacles_inflation_z_down=down;
    map.rebuildInflationOffsets();
  }
  static void rawRayEvidence(GridMap &map) {
    map.mp_.use_projected_rays_=true;
    map.ray_query_clock_ns_=100000000000LL;
    map.integrated_cloud_stamp_ns_=99900000000LL;
    map.ray_integrated_stamps_={99900000000LL,99900000000LL};
  }
};

namespace {
constexpr double kBodyHeight=.55;
constexpr double kLowObstacleHeight=.2;
constexpr double kPi=3.14159265358979323846;

void expectGroundClearAndLowObstacleBlocked(GridMap &map,double floor_z,double yaw) {
  SCOPED_TRACE(::testing::Message()<<"floor_z="<<floor_z<<", yaw="<<yaw);
  GridMapTestAccess::reset(map);
  const Eigen::Vector3d body(.01,.01,floor_z+kBodyHeight);
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
  const Eigen::Vector3d center=body+.2*heading;
  const Eigen::Vector3d ground(center.x(),center.y(),floor_z);
  const Eigen::Vector3d low_top(center.x(),center.y(),floor_z+kLowObstacleHeight);
  ASSERT_TRUE(map.isInMap(ground));ASSERT_TRUE(map.isInMap(body));
  ASSERT_TRUE(map.isInMap(low_top));
  map.setOccupied(ground);
  ASSERT_EQ(map.getOccupancy(ground),1);
  EXPECT_EQ(map.getInflateOccupancy(body,yaw),0);
  EXPECT_TRUE(map.isUnknown(center)); // Collision zero does not invent raw free.
  // The floor stays occupied; adding a genuine low obstacle must block.
  map.setOccupied(low_top);
  EXPECT_EQ(map.getOccupancy(ground),1);
  EXPECT_EQ(map.getOccupancy(low_top),1);
  EXPECT_EQ(map.getInflateOccupancy(body,yaw),1);
}
} // namespace

TEST(OfficialResolution, EightCentimeterReproducesGroundQuantizationCollision) {
  GridMap map;GridMapTestAccess::configure(map,.08);
  EXPECT_EQ(map.collisionQueryPolicy(),"official_inflated_double_cylinder");
  EXPECT_EQ(GridMapTestAccess::upperInflationStep(map),6);
  EXPECT_EQ(GridMapTestAccess::zIndex(map,kBodyHeight)-GridMapTestAccess::zIndex(map,0.),6);
  map.setOccupied({.21,.01,0.});
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.21,.01,0.)),1);
  EXPECT_EQ(map.getInflateOccupancy({.01,.01,kBodyHeight},0.),1);
  EXPECT_TRUE(map.isUnknown(Eigen::Vector3d(.21,.01,kBodyHeight)));
}

TEST(OfficialResolution, FiveCentimeterKeepsGroundAndBlocksTwentyCentimeterObstacle) {
  GridMap map;GridMapTestAccess::configure(map,.05);
  EXPECT_EQ(GridMapTestAccess::upperInflationStep(map),9);
  EXPECT_EQ(GridMapTestAccess::zIndex(map,kBodyHeight)-GridMapTestAccess::zIndex(map,0.),11);
  for(double yaw:{0.,kPi/4.,kPi/2.}) expectGroundClearAndLowObstacleBlocked(map,0.,yaw);
}

TEST(OfficialResolution, FiveCentimeterResultIsStableAcrossFloorGridPhases) {
  GridMap map;GridMapTestAccess::configure(map,.05);
  std::vector<double> phases;
  for(int i=-80;i<=80;++i) phases.push_back(i*.005);
  // Include adjacent representable doubles on both sides of voxel boundaries.
  for(int i=-8;i<=8;++i) {
    const double boundary=i*.05;
    phases.push_back(std::nextafter(boundary,-std::numeric_limits<double>::infinity()));
    phases.push_back(boundary);
    phases.push_back(std::nextafter(boundary,std::numeric_limits<double>::infinity()));
  }
  for(double phase:phases)
    for(double yaw:{0.,kPi/4.,kPi/2.}) expectGroundClearAndLowObstacleBlocked(map,phase,yaw);
}

TEST(OfficialResolution, OfficialKernelCostRemainsExplicit) {
  GridMap coarse,fine;
  GridMapTestAccess::configure(coarse,.08);GridMapTestAccess::configure(fine,.05);
  // These are actual production offsets, not an approximate cylinder volume.
  EXPECT_EQ(GridMapTestAccess::kernelSize(coarse),585U);
  EXPECT_EQ(GridMapTestAccess::kernelSize(fine),1919U);
}

TEST(PlanningGeometry, NarrowCandidateDoesNotShrinkStrictEnvelopeOrDeleteLowOccupancy) {
  GridMap map;GridMapTestAccess::configure(map,.05,true,.12,.45,.40);
  const Eigen::Vector3d body(.01,.01,.04),low_hit(.21,.01,-.30);
  map.setOccupied(low_hit);
  const auto raw=GridMapTestAccess::raw(map);
  EXPECT_EQ(map.getOccupancy(low_hit),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(body,0.),1);
  EXPECT_DOUBLE_EQ(map.planningObstacleDilation().up,.12);
  EXPECT_DOUBLE_EQ(map.obstacleDilation().up,.40);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().below,.40);
  EXPECT_DOUBLE_EQ(map.bodyEnvelope().above,.45);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  // Candidate zero remains independent of unseen raw space and motion proof.
  EXPECT_EQ(map.getPlanningInflateOccupancy({.01,.40,.04},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.01,.40,.04},0.),2);
}

TEST(PlanningGeometry, SnapshotStrictProofKeepsOriginalLowHitAndSourceTimes) {
  GridMap writer;GridMapTestAccess::configure(writer,.05,true,.12,.45,.40);
  GridMapTestAccess::rawRayEvidence(writer);
  const Eigen::Vector3d body(.01,.01,.04),low_hit(.21,.01,-.30);
  writer.setOccupied(low_hit);
  const auto raw=GridMapTestAccess::raw(writer);
  GridMap snapshot;
  writer.copyCollisionSnapshotTo(snapshot,100000000000LL,std::chrono::steady_clock::now());
  snapshot.requireObservedSnapshot();
  EXPECT_EQ(snapshot.getPlanningInflateOccupancy(body,0.),0);
  snapshot.beginObservedProof();
  EXPECT_EQ(snapshot.getInflateOccupancy(body,0.),1);
  Eigen::Vector3i hit_cell;snapshot.posToIndex(low_hit,hit_cell);
  EXPECT_EQ(snapshot.observedRawSnapshotStatus(hit_cell),1);
  EXPECT_DOUBLE_EQ(snapshot.bodyEnvelope().below,.40);
  EXPECT_DOUBLE_EQ(snapshot.bodyEnvelope().above,.45);
  EXPECT_EQ(snapshot.latestCloudStampNs(),99900000000LL);
  EXPECT_EQ(snapshot.integratedRaySourceStamp(0),99900000000LL);
  EXPECT_EQ(snapshot.integratedRaySourceStamp(1),99900000000LL);
  EXPECT_EQ(GridMapTestAccess::raw(snapshot),raw);
}

TEST(PlanningGeometry, MissingParametersAndExplicitSameValuesPreserveLegacyBuffers) {
  for(const bool strict:{false,true}) {
    GridMap legacy,explicit_same;
    GridMapTestAccess::configure(legacy,.05,strict,-1.,-1.,.40);
    GridMapTestAccess::configure(explicit_same,.05,strict,.40,.45,.40);
    for(const Eigen::Vector3d &p:std::vector<Eigen::Vector3d>{{.21,.01,-.30},{-.21,.01,.20}}) {
      legacy.setOccupied(p);explicit_same.setOccupied(p);
    }
    EXPECT_EQ(GridMapTestAccess::strictBuffer(legacy),GridMapTestAccess::strictBuffer(explicit_same));
    EXPECT_EQ(GridMapTestAccess::planningBuffer(legacy),GridMapTestAccess::planningBuffer(explicit_same));
    EXPECT_EQ(GridMapTestAccess::raw(legacy),GridMapTestAccess::raw(explicit_same));
    for(double x:{-.4,0.,.4}) for(double y:{-.4,0.,.4}) for(double z:{-.4,0.,.4}) {
      EXPECT_EQ(legacy.getPlanningInflateOccupancy({x,y,z},.3),
          explicit_same.getPlanningInflateOccupancy({x,y,z},.3));
      EXPECT_EQ(legacy.getInflateOccupancy({x,y,z},.3),explicit_same.getInflateOccupancy({x,y,z},.3));
    }
  }
}

TEST(PlanningGeometry, NarrowingRequiresIndependentStrictEvidenceLayer) {
  GridMap map;
  EXPECT_THROW(GridMapTestAccess::configure(map,.05,false,.12,.45,.40),std::invalid_argument);
}

TEST(PlanningGeometry, ExplicitInvalidOrLargerCandidateExtentIsRejected) {
  for(double value:{-.2,.41,std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    GridMap up,down;
    EXPECT_THROW(GridMapTestAccess::configure(up,.05,true,value,.45,.40),std::invalid_argument);
    EXPECT_THROW(GridMapTestAccess::configure(down,.05,true,.12,
        value==.41 ? .46:value,.40),std::invalid_argument);
  }
}

TEST(PlanningGeometry, RebuildingCandidateKernelPreservesStrictBufferAndRawVotes) {
  GridMap map;GridMapTestAccess::configure(map,.05,true,-1.,-1.,.40);
  const Eigen::Vector3d body(.01,.01,.04),low_hit(.21,.01,-.30);
  map.setOccupied(low_hit);
  ASSERT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  const auto raw=GridMapTestAccess::raw(map);
  const auto strict_buffer=GridMapTestAccess::strictBuffer(map);
  GridMapTestAccess::rebuildPlanning(map,.12,.45);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  EXPECT_EQ(map.getInflateOccupancy(body,0.),1);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  EXPECT_EQ(GridMapTestAccess::strictBuffer(map),strict_buffer);
}
