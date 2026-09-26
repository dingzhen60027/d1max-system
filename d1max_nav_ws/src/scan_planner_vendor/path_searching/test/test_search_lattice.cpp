#include <gtest/gtest.h>
#include <path_searching/search_lattice.hpp>
#include <limits>

using scan_planner::nearestSearchIndex;

TEST(SearchLattice, NegativeAndPositiveEndpointsRoundSymmetrically) {
  const Eigen::Vector3d center=Eigen::Vector3d::Zero();
  const Eigen::Vector3i middle(32,32,16),size(64,64,32);
  for (double cells:{.1,.49,.5,.51,.99,1.,1.49,1.5,12.51}) {
    Eigen::Vector3i positive,negative;
    ASSERT_TRUE(nearestSearchIndex(center+Eigen::Vector3d(cells*.08,0,0),center,.08,middle,size,positive));
    ASSERT_TRUE(nearestSearchIndex(center-Eigen::Vector3d(cells*.08,0,0),center,.08,middle,size,negative));
    EXPECT_EQ((positive-middle).x(),-(negative-middle).x());
    EXPECT_LE(std::abs((negative-middle).x()+cells),.500000001);
    EXPECT_LE(std::abs((positive-middle).x()-cells),.500000001);
  }
}

TEST(SearchLattice, NegativeCoordinateDoesNotMoveAnEntireExtraVoxel) {
  Eigen::Vector3i index;
  ASSERT_TRUE(nearestSearchIndex({-.119,0,0},Eigen::Vector3d::Zero(),.08,{16,16,16},{32,32,32},index));
  EXPECT_EQ(index.x(),15); // Old signed cast(.5-1.4875) incorrectly returned 16.
  EXPECT_NEAR((index.x()-16)*.08,-.08,1e-12);
}

TEST(SearchLattice, RejectsNonfiniteAndOutOfBoundsBeforeIntegerConversion) {
  Eigen::Vector3i index;
  const Eigen::Vector3d center=Eigen::Vector3d::Zero();
  for (double x:{1e300,-1e300,std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(nearestSearchIndex({x,0,0},center,.08,{16,16,16},{32,32,32},index));
  EXPECT_FALSE(nearestSearchIndex(center,center,0.,{16,16,16},{32,32,32},index));
}

TEST(SearchLattice, DiagnosticDoesNotLabelUnknownAsObstacleOrFree) {
  EXPECT_STREQ(scan_planner::occupancyStateName(0),"observed_free");
  EXPECT_STREQ(scan_planner::occupancyStateName(1),"occupied");
  EXPECT_STREQ(scan_planner::occupancyStateName(2),"unobserved_or_uncertain");
  EXPECT_STREQ(scan_planner::occupancyStateName(-1),"outside_map");
}
