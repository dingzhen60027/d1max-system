#include <gtest/gtest.h>
#include <bspline_opt/trajectory_timing.hpp>
#ifndef D1MAX_RECOVERY_BEFORE
#include <bspline_opt/initial_motion_limits.hpp>
#endif
namespace {
scan_planner::UniformBspline recovering(double v,double dt=.5) {
  Eigen::MatrixXd p=Eigen::MatrixXd::Zero(3,10);
  const double speeds[]={v,v,.3,.3,.3,.3,.15,0.,0.};
  for(int i=1;i<10;++i)p(0,i)=p(0,i-1)+dt*speeds[i-1];
  return {p,3,dt};
}
scan_planner::CubicMotionBoundary boundary(scan_planner::UniformBspline curve) {
  auto v=curve.getDerivative(),a=v.getDerivative();const double end=curve.getTimeSum();
  return {curve.evaluateDeBoorT(0.),v.evaluateDeBoorT(0.),a.evaluateDeBoorT(0.),
          curve.evaluateDeBoorT(end),v.evaluateDeBoorT(end),a.evaluateDeBoorT(end)};
}
TEST(InitialMotionLimits,RecordedAboveCruiseInitialStatesHaveFiniteExactRecovery) {
  for(double speed:{.304981,.416980}) {
    auto curve=recovering(speed);const auto original=boundary(curve);
#ifdef D1MAX_RECOVERY_BEFORE
    const auto result=scan_planner::refineTimingWithFixedBoundary(curve,original,.3,.35,
        [](Eigen::MatrixXd&,double){return true;});
#else
    const auto result=scan_planner::refineTimingWithFixedBoundary(curve,original,.3,.35,
        [](Eigen::MatrixXd&,double){return true;},8,std::nullopt,{},true);
#endif
    ASSERT_TRUE(result.success)<<result.reason<<" initial="<<speed;
    EXPECT_TRUE(scan_planner::cubicBoundaryMatches(curve,original));
    auto velocity=curve.getDerivative();
    EXPECT_NEAR(velocity.evaluateDeBoorT(0.).norm(),speed,1e-12);
    for(double t=1.;t<=curve.getTimeSum();t+=.001)EXPECT_LE(velocity.evaluateDeBoorT(t).norm(),.3+1e-9);
  }
}
#ifndef D1MAX_RECOVERY_BEFORE
TEST(InitialMotionLimits,SteadyOverspeedLateOverspeedAndUnboundedRetimingReject) {
  auto curve=recovering(.41698);
  ASSERT_TRUE(scan_planner::inspectInitialMotionLimits(curve,.3,.35).valid);
  auto p=curve.getControlPoint();for(int i=1;i<p.cols();++i)p(0,i)=p(0,i-1)+.5*.41698;
  EXPECT_FALSE(scan_planner::inspectInitialMotionLimits({p,3,.5},.3,.35).valid);
  curve=recovering(.304981,1.);EXPECT_STREQ(scan_planner::inspectInitialMotionLimits(curve,.3,.35).reason,"initial_speed_recovery_too_slow");
  curve=recovering(.41698);p=curve.getControlPoint();p(0,6)+=.01;
  EXPECT_FALSE(scan_planner::inspectInitialMotionLimits({p,3,.5},.3,.35).valid);
  p(0,6)=NAN;EXPECT_FALSE(scan_planner::inspectInitialMotionLimits({p,3,.5},.3,.35).valid);
}
TEST(InitialMotionLimits,ExcessAccelerationAndNominalStartsDoNotGainAnException) {
  EXPECT_FALSE(scan_planner::inspectInitialMotionLimits(recovering(.8),.3,.35).valid);
  EXPECT_FALSE(scan_planner::inspectInitialMotionLimits(recovering(.41698),.3,.35,false).valid);
  auto curve=recovering(.3);auto p=curve.getControlPoint();p(0,1)+=.02;
  EXPECT_FALSE(scan_planner::inspectInitialMotionLimits({p,3,.5},.3,.35).valid);
}
TEST(InitialMotionLimits,InteriorConstraintProjectionPreservesFullBoundaryAndCancelsWithoutMutation) {
  auto curve=recovering(.41698);auto p=curve.getControlPoint();p(0,4)+=.012;
  curve=scan_planner::UniformBspline(p,3,.5);const auto exact=boundary(curve);
  EXPECT_TRUE(scan_planner::projectInitialRecoveryInterior(curve,.3,.35));
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(curve,exact));
  EXPECT_TRUE(scan_planner::inspectInitialMotionLimits(curve,.3,.35).valid);
  curve=scan_planner::UniformBspline(p,3,.5);
  EXPECT_FALSE(scan_planner::projectInitialRecoveryInterior(curve,.3,.35,[]{return false;}));
  EXPECT_TRUE(curve.getControlPoint().isApprox(p,0.));
}
#endif
} // namespace
