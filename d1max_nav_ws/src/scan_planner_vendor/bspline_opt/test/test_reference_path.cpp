#include <gtest/gtest.h>
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/uniform_bspline.h>

using Eigen::Vector3d;
using scan_planner::DiscreteReference;

TEST(ReferencePath, InterpolationPreservesCornersAndNonlinearHeight) {
  DiscreteReference path;
  path.set({{0,0,0}, {2,0,.5}, {2,2,0}, {4,2,0}});
  for (double s=0; s<=path.length(); s+=.01) {
    const auto p=path.sample(s);
    EXPECT_TRUE(std::abs(p.y())<1e-9 || std::abs(p.x()-2)<1e-9 || std::abs(p.y()-2)<1e-9);
    EXPECT_GE(p.z(), -1e-9);
    EXPECT_LE(p.z(), .5+1e-9);
  }
  EXPECT_TRUE(path.sample(path.arc[1]).isApprox(Vector3d(2,0,.5)));
}

TEST(ReferencePath, ProjectionCannotSkipAcrossLoopsOrMoveBackward) {
  DiscreteReference path;
  path.set({{0,0,0}, {10,0,0}, {10,1,0}, {0,1,0}});
  EXPECT_NEAR(path.project({0,1,0},0,2),0,1e-9);
  EXPECT_GE(path.project({0,0,0},5,7),5);
  const auto part=path.slice(9,12,{9,0,0});
  ASSERT_EQ(part.size(),4U);
  EXPECT_TRUE(part[1].isApprox(Vector3d(10,0,0)));
  EXPECT_TRUE(part[2].isApprox(Vector3d(10,1,0)));
}

TEST(ReferencePath, TimedSeedKeepsOriginalGroundRiseAndVelocityBound) {
  DiscreteReference path;
  path.set({{0,0,0},{2,0,.5},{4,0,0}});
  const auto seed=scan_planner::sampleReferenceSeed(path,.3,.35,0,0,.2);
  ASSERT_GT(seed.samples.size(),6U);
  double max_z=0;
  for (std::size_t i=0;i<seed.samples.size();++i) {
    max_z=std::max(max_z,seed.samples[i].z());
    if(i) EXPECT_LE((seed.samples[i]-seed.samples[i-1]).norm()/seed.dt,.300001);
  }
  EXPECT_GT(max_z,.45);
  EXPECT_TRUE(seed.samples.front().isApprox(path.points.front()));
  EXPECT_TRUE(seed.samples.back().isApprox(path.points.back()));
}

TEST(ReferencePath, FitCostGradientMatchesFiniteDifferenceIncludingHeight) {
  Eigen::MatrixXd control(3,8);
  for(int i=0;i<8;++i) control.col(i)=Vector3d(i*.2,std::sin(i*.3),i*.05);
  const std::vector<Vector3d> reference{{0,0,0},{.2,0,0},{.5,.1,.2},{.8,.3,.1},{1,.5,.1},{1.4,.6,.1}};
  Eigen::MatrixXd grad;
  EXPECT_GT(scan_planner::referenceSampleCost(control,reference,grad),0);
  const double epsilon=1e-6;
  for(int r=0;r<3;++r) for(int c=0;c<8;++c) {
    Eigen::MatrixXd plus=control,minus=control,dummy;
    plus(r,c)+=epsilon;minus(r,c)-=epsilon;
    const double numerical=(scan_planner::referenceSampleCost(plus,reference,dummy)-
                            scan_planner::referenceSampleCost(minus,reference,dummy))/(2*epsilon);
    EXPECT_NEAR(grad(r,c),numerical,1e-7);
  }
}

TEST(ReferencePath, CubicSeedFollowsBentRouteRatherThanDirectShortcut) {
  DiscreteReference path;
  path.set({{0,0,0},{1,0,.1},{2,.4,.2},{3,1.4,.3},{3.4,2.4,.2},{3.4,3.4,0}});
  const auto seed=scan_planner::sampleReferenceSeed(path,.3,.35,0,0,.2);
  Eigen::MatrixXd control;
  std::vector<Vector3d> derivatives(4,Vector3d::Zero());
  scan_planner::UniformBspline::parameterizeToBspline(seed.dt,seed.samples,derivatives,control);
  scan_planner::UniformBspline curve(control,3,seed.dt);
  for(double t=0;t<=curve.getTimeSum();t+=.1) {
    const auto point=curve.evaluateDeBoorT(t);
    const double s=path.project(point,0,path.length());
    EXPECT_LT((point-path.sample(s)).norm(),.08);
  }
}
