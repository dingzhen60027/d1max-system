#include <gtest/gtest.h>
#include <bspline_opt/near_curve_direction.hpp>
#include <limits>

using scan_planner::CurveDirectionPoint;
using scan_planner::nearCurveDirection;

TEST(NearCurveDirection,ZeroDerivativeUsesActualFirstMillimeterNotEndpointChord) {
  const auto p=[](double t){return CurveDirectionPoint{0.,.2*t*t,0.};};
  const auto v=[](double t){return CurveDirectionPoint{0.,.4*t,0.};};
  const auto direction=nearCurveDirection(p,v,0.,10.);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(p(direction->time)[1],.001,1e-10);
  EXPECT_DOUBLE_EQ(direction->velocity[0],0.);EXPECT_GT(direction->velocity[1],0.);
  const auto local_p=[](double t){return CurveDirectionPoint{.03*std::min(t,1.),std::max(0.,t-1.),0.};};
  const auto local_v=[](double t){return t<1.?CurveDirectionPoint{.03,0.,0.}:CurveDirectionPoint{0.,1.,0.};};
  const auto near=nearCurveDirection(local_p,local_v,0.,3.);
  ASSERT_TRUE(near);EXPECT_NEAR(near->time,1./30.,1e-8);EXPECT_GT(near->velocity[0],0.);
  EXPECT_DOUBLE_EQ(near->velocity[1],0.); // Distant +Y target is not the entry yaw.
}
TEST(NearCurveDirection,MeasuredCurveParameterSelectsLocalBranchNotElapsedClock) {
  const auto p=[](double t){return CurveDirectionPoint{.03*std::min(t,1.),.03*std::max(0.,t-1.),0.};};
  const auto v=[](double t){return t<1.?CurveDirectionPoint{.03,0.,0.}:CurveDirectionPoint{0.,.03,0.};};
  const auto direction=nearCurveDirection(p,v,1.1,3.);
  ASSERT_TRUE(direction);EXPECT_NEAR(direction->time,1.1+1./30.,1e-8);
  EXPECT_DOUBLE_EQ(direction->velocity[0],0.);EXPECT_GT(direction->velocity[1],0.);
}
TEST(NearCurveDirection,LowSpeedEntryNoiseCannotSelectOppositeTurnBeforeReliableGeometry) {
  for(double noise:{-.007,.007}) {
    const auto p=[&](double t){return CurveDirectionPoint{noise*t+.01*t*t,0.,0.};};
    const auto v=[&](double t){return CurveDirectionPoint{noise+.02*t,0.,0.};};
    const auto d=nearCurveDirection(p,v,0.,3.);
    ASSERT_TRUE(d);EXPECT_GE(d->velocity[0],.02);EXPECT_GT(d->direction[0],0.);
    EXPECT_LE(std::abs(p(d->time)[0]),.02+1e-9);
  }
}
TEST(NearCurveDirection,ContinuousSlowActualCurveHasGeometricDirectionWithoutInventedSpeed) {
  const auto p=[](double t){return CurveDirectionPoint{0.,.005*t,0.};};
  const auto v=[](double){return CurveDirectionPoint{0.,.005,0.};};
  const auto d=nearCurveDirection(p,v,0.,5.);ASSERT_TRUE(d);
  EXPECT_NEAR(d->time,4.,1e-8);EXPECT_DOUBLE_EQ(d->velocity[1],.005);
  EXPECT_NEAR(d->direction[1],.02,1e-10);EXPECT_DOUBLE_EQ(d->direction[0],0.);
}
TEST(NearCurveDirection,ShortSlowGeometryRetainsDirectionInsideFixedSearchBudget) {
  unsigned positions=0;
  const auto p=[&](double t){++positions;return CurveDirectionPoint{.002*t,0.,0.};};
  const auto v=[](double){return CurveDirectionPoint{.002,0.,0.};};
  const auto d=nearCurveDirection(p,v,0.,10.);ASSERT_TRUE(d);
  EXPECT_LE(d->time,4.096+1e-10);EXPECT_LE(positions,4097U+24U);
  EXPECT_GT(d->direction[0],.001);EXPECT_DOUBLE_EQ(d->direction[1],0.);
  EXPECT_DOUBLE_EQ(d->velocity[0],.002);
}
TEST(NearCurveDirection,BoundedFallbackCannotResurrectOneMillimeterReverseEntryNoise) {
  const double pi=std::acos(-1.);
  const auto p=[&](double t) {
    return CurveDirectionPoint{t<.3?-.001*std::pow(std::sin(pi*t/.6),2):
      -.001+.00005*(t-.3)*(t-.3),0.,0.};
  };
  const auto v=[&](double t) {
    return CurveDirectionPoint{t<.3?-.001*pi/.6*std::sin(pi*t/.3):.0001*(t-.3),0.,0.};
  };
  // A real 1 mm initial backward dip followed by very slow forward geometry
  // beyond the fixed budget is unresolved, never an invented backward turn.
  EXPECT_FALSE(nearCurveDirection(p,v,0.,10.));
}
TEST(NearCurveDirection,ReliableActualReverseIsNotReplacedByDistantForwardGoal) {
  const auto p=[](double t){return CurveDirectionPoint{-.03*std::min(t,1.),std::max(0.,t-1.),0.};};
  const auto v=[](double t){return t<1.?CurveDirectionPoint{-.03,0.,0.}:CurveDirectionPoint{0.,1.,0.};};
  const auto d=nearCurveDirection(p,v,0.,3.);ASSERT_TRUE(d);
  EXPECT_LT(d->direction[0],0.);EXPECT_DOUBLE_EQ(d->direction[1],0.);
}
TEST(NearCurveDirection,StationaryAndFarFutureDirectionStayBoundedAndUnresolved) {
  unsigned calls=0;
  const auto p=[&](double t){++calls;return CurveDirectionPoint{std::max(0.,t-5.),0.,0.};};
  const auto v=[](double t){return CurveDirectionPoint{t>5.?1.:0.,0.,0.};};
  EXPECT_FALSE(nearCurveDirection(p,v,0.,10.));
  EXPECT_LE(calls,4097U); // Cannot reach the far future motion after an idle start.
  calls=0;EXPECT_FALSE(nearCurveDirection(p,v,0.,10.,scan_planner::CurveDirectionAlwaysAllowed{},3));
  EXPECT_LE(calls,4U);
}
TEST(NearCurveDirection,TooShortOrVerticalGeometryCannotInventPlanarDirection) {
  const auto tiny=[](double t){return CurveDirectionPoint{.001*t,0.,0.};};
  const auto vx=[](double){return CurveDirectionPoint{.001,0.,0.};};
  EXPECT_FALSE(nearCurveDirection(tiny,vx,0.,.05));
  const auto vertical=[](double t){return CurveDirectionPoint{0.,0.,.3*t};};
  const auto vz=[](double){return CurveDirectionPoint{0.,0.,.3};};
  EXPECT_FALSE(nearCurveDirection(vertical,vz,0.,1.));
}
TEST(NearCurveDirection,InvalidGeometryAndBoundsRejectWithoutAlternativeDirection) {
  const auto p=[](double t){return CurveDirectionPoint{.1*t,0.,0.};};
  const auto v=[](double){return CurveDirectionPoint{.1,0.,0.};};
  for(double start:{-1.,1.,2.,std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(nearCurveDirection(p,v,start,1.));
  EXPECT_FALSE(nearCurveDirection(p,v,0.,121.));
  EXPECT_FALSE(nearCurveDirection(p,v,0.,1.,scan_planner::CurveDirectionAlwaysAllowed{},4097));
  EXPECT_FALSE(nearCurveDirection(p,v,0.,1.,scan_planner::CurveDirectionAlwaysAllowed{},0));
  const auto invalid=[](double){return CurveDirectionPoint{std::numeric_limits<double>::quiet_NaN(),0.,0.};};
  EXPECT_FALSE(nearCurveDirection(invalid,v,0.,1.));EXPECT_FALSE(nearCurveDirection(p,invalid,0.,1.));
}
TEST(NearCurveDirection,SharedDeadlineStopsProjectionWithoutReturningPartialDirection) {
  const auto p=[](double t){return CurveDirectionPoint{.1*t,0.,0.};};
  const auto v=[](double){return CurveDirectionPoint{.1,0.,0.};};
  unsigned allowed=0;
  EXPECT_FALSE(nearCurveDirection(p,v,0.,1.,[&]{return ++allowed<5;}));
  EXPECT_EQ(allowed,5U);
}
