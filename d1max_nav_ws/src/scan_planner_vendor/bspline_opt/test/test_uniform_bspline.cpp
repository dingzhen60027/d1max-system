#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/trajectory_timing.hpp>
#include <bspline_opt/reference_path.hpp>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, AffineTimeScalingPreservesCurveAndScalesAllDerivatives)
{
  Eigen::MatrixXd points(3, 8);
  points << 0, .1, .4, .9, 1.1, 1.4, 1.8, 2.0,
            0, .1, .2, .5, 1.0, 1.4, 1.5, 1.7,
            0, .0, .1, .1, .2, .0, .1, .2;
  scan_planner::UniformBspline original(points, 3, .3), scaled = original;
  scaled.scaleTime(1.7);
  EXPECT_NEAR(scaled.getTimeSum(), original.getTimeSum() * 1.7, 1e-12);
  EXPECT_NEAR(scaled.getInterval(), original.getInterval() * 1.7, 1e-12);
  auto v = original.getDerivative(), a = v.getDerivative();
  auto sv = scaled.getDerivative(), sa = sv.getDerivative();
  for (int i = 0; i <= 100; ++i) {
    const double t = original.getTimeSum() * i / 100.0;
    EXPECT_LT((scaled.evaluateDeBoorT(t*1.7) - original.evaluateDeBoorT(t)).norm(), 1e-10);
    EXPECT_LT((sv.evaluateDeBoorT(t*1.7) * 1.7 - v.evaluateDeBoorT(t)).norm(), 1e-10);
    EXPECT_LT((sa.evaluateDeBoorT(t*1.7) * 1.7*1.7 - a.evaluateDeBoorT(t)).norm(), 1e-10);
  }
}

TEST(UniformBspline, NormRetimingHandlesDiagonalAndEndpointAccelerationWithoutLooserLimits)
{
  Eigen::MatrixXd points(3, 8);
  points << 0, .1, .4, .9, 1.1, 1.4, 1.8, 2.0,
            0, .1, .2, .5, 1.0, 1.4, 1.5, 1.7,
            0, .0, .1, .1, .2, .0, .1, .2;
  scan_planner::UniformBspline original(points, 3, .3), retimed = original;
  const double scale = scan_planner::enforceDerivativeBounds(retimed, .3, .35);
  EXPECT_GT(scale, 1.0);
  auto v = retimed.getDerivative(), a = v.getDerivative();
  EXPECT_LE(scan_planner::derivativeControlBound(v), .3);
  EXPECT_LE(scan_planner::derivativeControlBound(a), .35);
  for (int i = 0; i <= 1000; ++i) {
    const double t = retimed.getTimeSum() * i / 1000.0;
    EXPECT_LE(v.evaluateDeBoorT(t).norm(), .3);
    EXPECT_LE(a.evaluateDeBoorT(t).norm(), .35);
    EXPECT_LT((retimed.evaluateDeBoorT(t) - original.evaluateDeBoorT(t/scale)).norm(), 1e-9);
  }
  EXPECT_NEAR(scan_planner::enforceDerivativeBounds(retimed, .3, .35), 1.0, 1e-12);
}

TEST(UniformBspline, StationaryBoundaryRetimingPreservesBentGeometry) {
  Eigen::MatrixXd points(3,8);
  points << 0,0,0,.5,1.,1.,1.,1.,
            0,0,0,.5,1.,1.,1.,1.,
            .55,.55,.55,.55,.55,.55,.55,.55;
  scan_planner::UniformBspline original(points,3,.2),candidate=original;
  const double scale=scan_planner::enforceDerivativeBoundsAtStart(
      candidate,.3,.35,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  EXPECT_GT(scale,1.);
  for(int i=0;i<=100;++i) {
    const double t=original.getTimeSum()*i/100.;
    EXPECT_LT((candidate.evaluateDeBoorT(t*scale)-original.evaluateDeBoorT(t)).norm(),1e-9);
  }
}

TEST(UniformBspline, MovingBoundaryCannotBeSilentlySlowedOrMutated) {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,6);
  for(int i=0;i<6;++i)points(0,i)=i*.2;
  scan_planner::UniformBspline candidate(points,3,.2);
  const auto before=candidate.getControlPoint();const double duration=candidate.getTimeSum();
  EXPECT_THROW(scan_planner::enforceDerivativeBoundsAtStart(
    candidate,.3,.35,Eigen::Vector3d(1.,0,0),Eigen::Vector3d::Zero()),std::invalid_argument);
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, MovingBoundaryMustMatchEvenWhenNoRetimingIsNeeded) {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,6);
  for(int i=0;i<6;++i)points(0,i)=i*.1;
  scan_planner::UniformBspline candidate(points,3,1.);
  EXPECT_DOUBLE_EQ(scan_planner::enforceDerivativeBoundsAtStart(
    candidate,.3,.35,Eigen::Vector3d(.1,0,0),Eigen::Vector3d::Zero()),1.);
  EXPECT_THROW(scan_planner::enforceDerivativeBoundsAtStart(
    candidate,.3,.35,Eigen::Vector3d(.2,0,0),Eigen::Vector3d::Zero()),std::invalid_argument);
}

namespace {
scan_planner::CubicMotionBoundary straightBoundary(double start_speed=.01) {
  return {Eigen::Vector3d::Zero(),Eigen::Vector3d(start_speed,0,0),Eigen::Vector3d::Zero(),
      Eigen::Vector3d(2.,0,0),Eigen::Vector3d(.3,0,0),Eigen::Vector3d::Zero()};
}
std::vector<Eigen::Vector3d> straightSamples() {
  std::vector<Eigen::Vector3d> samples;
  for (int i=0;i<=10;++i) samples.emplace_back(.2*i,0.,0.);
  return samples;
}
}

TEST(UniformBspline, ConstrainedFitPreservesFullMeasuredPositionVelocityAcceleration) {
  auto boundary=straightBoundary();
  boundary.start_position=Eigen::Vector3d(.2,.4,.6);
  boundary.start_velocity=Eigen::Vector3d(.11,-.04,.03);
  boundary.start_acceleration=Eigen::Vector3d(.03,.02,-.01);
  boundary.end_position=Eigen::Vector3d(2.,.6,1.1);
  boundary.end_velocity=Eigen::Vector3d(.2,.04,.08);
  boundary.end_acceleration=Eigen::Vector3d(-.02,.01,.015);
  for (double dt : {.1,.4,1.5}) {
    auto controls=scan_planner::fitCubicWithFixedBoundary(straightSamples(),dt,boundary);
    scan_planner::UniformBspline candidate(controls,3,dt);
    EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary,1e-10));
  }
}

TEST(UniformBspline, MovingTimingFindsStrictLimitsWithoutErasingSmallInitialMotion) {
  const auto boundary=straightBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  int refits=0;
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [&refits](Eigen::MatrixXd &,double) { ++refits; return true; });
  ASSERT_TRUE(result.success) << result.reason << " " << result.speed_bound << " " << result.acceleration_bound;
  EXPECT_GT(refits,0);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary));
  EXPECT_LE(result.speed_bound,.3+1e-9);
  EXPECT_LE(result.acceleration_bound,.35+1e-9);
  EXPECT_NEAR(candidate.getDerivative().evaluateDeBoorT(0.).x(),.01,1e-10);
}

TEST(UniformBspline, MovingTimingRejectsInfeasibleInitialSpeedWithoutCallingSolver) {
  const auto boundary=straightBoundary(.496);
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto before=candidate.getControlPoint();
  const double duration=candidate.getTimeSum();
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [](Eigen::MatrixXd &,double) { ADD_FAILURE()<<"infeasible start must not reach optimizer"; return true; });
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason,"initial_speed_exceeds_limit");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
}

TEST(UniformBspline, MovingTimingUsesNormNotPerAxisToRejectInitialBoundary) {
  auto boundary=straightBoundary(); boundary.start_velocity=Eigen::Vector3d(.25,.25,0.);
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [](Eigen::MatrixXd &,double) { return true; });
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"initial_speed_exceeds_limit");
}

TEST(UniformBspline, MovingTimingDoesNotAcceptOptimizerBoundaryMutation) {
  const auto boundary=straightBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto before=candidate.getControlPoint();
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [](Eigen::MatrixXd &controls,double) { controls(0,0)+=.01; return true; });
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"motion_boundary_mismatch");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, MovingTimingFailureAndIterationBudgetLeaveCandidateIntact) {
  const auto boundary=straightBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto before=candidate.getControlPoint();
  auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [](Eigen::MatrixXd &,double) { return false; });
  EXPECT_EQ(result.reason,"interior_refinement_failed");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
  result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.3,.35,
      [](Eigen::MatrixXd &,double) { return true; },0);
  EXPECT_EQ(result.reason,"constrained_timing_exhausted");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, MovingTimingRetainsTrueHalfMeterPerSecondStartUnderPointSixLimit) {
  const auto boundary=straightBoundary(.496);
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.6,.35,
      [](Eigen::MatrixXd &,double) { return true; });
  ASSERT_TRUE(result.success)<<result.reason<<" "<<result.speed_bound;
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary));
  EXPECT_LE(result.speed_bound,.6+1e-9);
  EXPECT_LE(result.acceleration_bound,.35+1e-9);
}

TEST(UniformBspline, FullXyzReferenceDomainAloneDoesNotGuaranteeFlatGuideFixedBoundaryFeasibility) {
  auto boundary=straightBoundary();boundary.start_velocity={.38,0.,.30};boundary.end_velocity={.15,0.,0.};
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(straightSamples(),.2,boundary),3,.2);
  const auto original=candidate.getControlPoint();
  auto strict=candidate;
  const auto rejected=scan_planner::refineTimingWithFixedBoundary(strict,boundary,.15,.35,
      [](Eigen::MatrixXd &,double) {return true;});
  EXPECT_FALSE(rejected.success);
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.5,.35,
      [](Eigen::MatrixXd &,double) {return true;});
  // Increasing a reference derivative domain cannot manufacture a feasible
  // passive-Z join. Without a successful interior refinement this original
  // curve remains rejected, including after all eight timing refinements.
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"constrained_timing_exhausted");
  EXPECT_GT(result.speed_bound,.5);EXPECT_TRUE(candidate.getControlPoint().isApprox(original,0.));
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary));
  EXPECT_NEAR(candidate.getDerivative().evaluateDeBoorT(0.).z(),.30,1e-10);
}

TEST(UniformBspline, SeparateXyzDomainAdmitsFeasibleCurveWithoutChangingGuideCruise) {
  const Eigen::Vector3d raw{.38,0.,.30};
  scan_planner::DiscreteReference guide;guide.set(straightSamples());
  const auto seed=scan_planner::sampleReferenceSeed(guide,.15,.35,raw.norm(),.15,.2);
  for(const auto& point:seed.samples)EXPECT_DOUBLE_EQ(point.z(),0.);
  EXPECT_NEAR(seed.dt,double(guide.length()/.15)/(seed.samples.size()-1),1e-10);
  scan_planner::CubicMotionBoundary boundary{Eigen::Vector3d::Zero(),raw,Eigen::Vector3d::Zero(),
      raw*2.,raw,Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> measured_reference;
  for(int i=0;i<=10;++i)measured_reference.push_back(raw*(i*.2));
  scan_planner::UniformBspline candidate(scan_planner::fitCubicWithFixedBoundary(measured_reference,.2,boundary),3,.2);
  auto strict=candidate;EXPECT_FALSE(scan_planner::refineTimingWithFixedBoundary(strict,boundary,.15,.35,
      [](Eigen::MatrixXd&,double){return true;}).success);
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.5,.35,
      [](Eigen::MatrixXd&,double){return true;});
  ASSERT_TRUE(result.success)<<result.reason;EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary));
  EXPECT_LE(result.speed_bound,.5+1e-9);EXPECT_LE(result.acceleration_bound,.35+1e-9);
}
