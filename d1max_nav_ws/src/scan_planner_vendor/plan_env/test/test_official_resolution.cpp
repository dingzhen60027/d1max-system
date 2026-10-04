#include <gtest/gtest.h>
#include "plan_env/grid_map.h"

#include <cmath>
#include <limits>
#include <vector>

// Exercise production indexing, inflation updates and two-center queries only.
// No ROS context, node, executor, sensor transport or synthetic free-space fill.
struct GridMapTestAccess {
  static void configure(GridMap &map, double resolution) {
    auto &p=map.mp_;auto &d=map.md_;
    p.require_observed_free_=false;p.use_projected_rays_=false;
    p.resolution_=resolution;p.resolution_inv_=1./resolution;
    p.map_voxel_num_={32,32,64};p.map_origin_idx_.setZero();
    map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;
    p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    // D1 body dimensions are unchanged when selecting the official resolution.
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
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
