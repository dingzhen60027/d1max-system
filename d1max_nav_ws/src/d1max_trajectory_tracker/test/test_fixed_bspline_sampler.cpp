#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <random>
#include "d1max_trajectory_tracker/tracker_core.hpp"

using namespace d1max_trajectory_tracker;
namespace {
scan_planner::UniformBspline vendor(int count,int degree,bool nonuniform=false) {
  Eigen::MatrixXd p(3,count);Eigen::VectorXd k(count+degree+1);
  for(int i=0;i<count;++i)p.col(i)=Eigen::Vector3d(std::sin(i*.13),std::cos(i*.31),i*.0123);
  k[0]=-1.3;
  for(int i=1;i<k.size();++i)k[i]=k[i-1]+(nonuniform?.01+.013*(i%7):.2);
  scan_planner::UniformBspline native(p,degree,.2);native.setKnot(k);return native;
}
void bitwiseSame(scan_planner::UniformBspline& native,const FixedBsplineSampler& sampler,double time) {
  const Eigen::VectorXd expected=native.evaluateDeBoorT(time);
  const Eigen::Vector3d actual=sampler.evaluateDeBoorT(time);
  for(int axis=0;axis<3;++axis) {
    std::uint64_t a,b;std::memcpy(&a,&expected[axis],sizeof(a));std::memcpy(&b,&actual[axis],sizeof(b));
    EXPECT_EQ(a,b)<<"axis="<<axis<<" time="<<time;
  }
}
void check(scan_planner::UniformBspline& native) {
  const auto sampler=FixedBsplineSampler::fromVendor(native);ASSERT_TRUE(sampler);
  const auto k=native.getKnot();const int degree=native.getOrder(),count=native.getControlPoint().cols();
  const double duration=k[count]-k[degree];
  const double infinity=std::numeric_limits<double>::infinity();
  for(double time:{-100.,-0.,+0.,duration,std::nextafter(duration,-infinity),
      std::nextafter(duration,infinity),duration+100.,infinity,-infinity,
      std::numeric_limits<double>::quiet_NaN()})bitwiseSame(native,*sampler,time);
  for(int i=degree;i<=count;++i) {
    const double time=k[i]-k[degree];
    bitwiseSame(native,*sampler,time);
    bitwiseSame(native,*sampler,std::nextafter(time,-INFINITY));
    bitwiseSame(native,*sampler,std::nextafter(time,INFINITY));
  }
  std::mt19937 generator(832520);std::uniform_real_distribution<double> distribution(-.1,duration+.1);
  for(int i=0;i<2000;++i)bitwiseSame(native,*sampler,distribution(generator));
}
}

TEST(FixedBsplineSampler, UniformCubicBitwiseIdenticalIncludingLeftKnotSpansAndClampedEnds) {
  auto native=vendor(23,3);check(native);
}
TEST(FixedBsplineSampler, NonuniformCubicBitwiseIdenticalAtEveryKnotAndNeighbours) {
  auto native=vendor(127,3,true);check(native);
}
TEST(FixedBsplineSampler, NativeVelocityAndAccelerationDerivativeKnotsPreserved) {
  auto native=vendor(83,3,true);auto velocity=native.getDerivative();auto acceleration=velocity.getDerivative();
  check(velocity);check(acceleration);
}
TEST(FixedBsplineSampler, MaximumAdmittedCurveMatchesNativeAcrossEntireDomain) {
  auto native=vendor(10000,3);check(native);
}
TEST(FixedBsplineSampler, DoesNotRelaxRepeatedClampedOrTooCloseKnotRejection) {
  for(int which=0;which<3;++which) {
    auto native=vendor(23,3);auto knots=native.getKnot();
    if(which==0)knots[5]=knots[4];
    if(which==1)knots[0]=knots[1]=knots[2]=knots[3];
    if(which==2)knots[5]=knots[4]+1e-10;
    native.setKnot(knots);EXPECT_FALSE(FixedBsplineSampler::fromVendor(native));
  }
}
TEST(FixedBsplineSampler, RejectsNonfiniteOrWrongDimensionAndUnsupportedDegree) {
  for(int which=0;which<5;++which) {
    Eigen::MatrixXd p=Eigen::MatrixXd::Zero(which==0?2:3,23);Eigen::VectorXd k(27);
    for(int i=0;i<27;++i)k[i]=i*.2;
    if(which==1)p(0,0)=std::numeric_limits<double>::infinity();
    if(which==2)k[4]=std::numeric_limits<double>::quiet_NaN();
    const int degree=which==3?0:which==4?4:3;
    scan_planner::UniformBspline native(p,degree,.2);native.setKnot(k);
    EXPECT_FALSE(FixedBsplineSampler::fromVendor(native));
  }
}
TEST(FixedBsplineSampler, ImmutableVendorSnapshotCannotBeChangedByLaterNativeMutation) {
  auto native=vendor(23,3);auto untouched=native;const auto sampler=FixedBsplineSampler::fromVendor(native);ASSERT_TRUE(sampler);
  auto knots=native.getKnot();knots*=2.;native.setKnot(knots);
  for(double time:{0.,.1,.2,.4,2.,4.})bitwiseSame(untouched,*sampler,time);
}
TEST(FixedBsplineSampler, ExactProjectionAdmissionUsesSameMathAndBoundary) {
  Trajectory trajectory;for(int i=0;i<23;++i)trajectory.points.emplace_back((i-1)*.05,0.,.55);
  for(int i=0;i<27;++i)trajectory.knots.push_back((i-3)*.2);
  auto cache=EntryCurveCache::build(trajectory);ASSERT_TRUE(cache);
  for(double error:{0.,.012499999,.0125,.012500001}) {
    const Eigen::Vector3d body(.075,error,.55);
    auto native=projectCurveAdmission(*cache->curve,cache->times,cache->arcs,cache->points,body,.2,.05,.05,.05);
    auto fixed=projectCurveAdmission(*cache->curve_sampler,cache->times,cache->arcs,cache->points,body,.2,.05,.05,.05);
    ASSERT_EQ(bool(native),bool(fixed));
    if(native) {
      EXPECT_EQ(native->time,fixed->time);EXPECT_EQ(native->arc,fixed->arc);
      EXPECT_TRUE((native->position.array()==fixed->position.array()).all());
    }
  }
}
