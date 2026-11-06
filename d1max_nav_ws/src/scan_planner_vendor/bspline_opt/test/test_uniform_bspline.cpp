#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/trajectory_timing.hpp>

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

namespace {
// Exact cubic polynomial on uniform spans, including an internal turning peak
// that endpoint-only sampling cannot see. Cubic B-spline controls are p(s) minus
// dt^2 p''(s)/6 at s=(i-1)dt.
scan_planner::UniformBspline polynomialCurve(const Eigen::Vector3d &c0,
    const Eigen::Vector3d &c1,const Eigen::Vector3d &c2,const Eigen::Vector3d &c3,
    double dt,int spans=1) {
  Eigen::MatrixXd controls(3,spans+3);
  for (int i=0;i<controls.cols();++i) {
    const double s=(i-1)*dt;
    controls.col(i)=c0+c1*s+c2*s*s+c3*s*s*s-dt*dt*(c2/3.+c3*s);
  }
  return scan_planner::UniformBspline(controls,3,dt);
}
double sampledTurningPeak(scan_planner::UniformBspline curve,double speed_threshold,
    int samples=20000) {
  auto velocity=curve.getDerivative(),acceleration=velocity.getDerivative();
  double peak=0.;
  for (int i=0;i<=samples;++i) {
    const double t=curve.getTimeSum()*i/samples;
    const Eigen::Vector2d v=velocity.evaluateDeBoorT(t).head<2>(),
        a=acceleration.evaluateDeBoorT(t).head<2>();
    if (v.norm()>=speed_threshold)
      peak=std::max(peak,std::abs(v.x()*a.y()-v.y()*a.x())/v.squaredNorm());
  }
  return peak;
}
scan_planner::UniformBspline stationaryCorner() {
  Eigen::MatrixXd controls(3,10);
  controls << 0,0,0,.4,.8,1.,1.,1.,1.,1.,
              0,0,0,0.,.1,.4,.8,1.,1.,1.,
              .55,.55,.55,.55,.55,.55,.55,.55,.55,.55;
  return scan_planner::UniformBspline(controls,3,.35);
}
scan_planner::CubicMotionBoundary turningBoundary() {
  return {Eigen::Vector3d(0,0,.55),Eigen::Vector3d(.1,0,0),Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.2,1.2,.55),Eigen::Vector3d(0,.1,0),Eigen::Vector3d::Zero()};
}
std::vector<Eigen::Vector3d> cornerSamples() {
  std::vector<Eigen::Vector3d> samples;
  for (int i=0;i<=10;++i) {
    const double angle=std::acos(-1.)*.5*i/10.;
    samples.emplace_back(1.2*std::sin(angle),1.2*(1.-std::cos(angle)),.55);
  }
  return samples;
}
}

TEST(UniformBspline, ContinuousTurningBoundFindsInternalPeakMissedAtBothEndpoints) {
  // v=(.021+.1*(t-.5)^2,.105*(t-.5)); endpoint omega=.086, middle=5.
  auto curve=polynomialCurve(Eigen::Vector3d::Zero(),Eigen::Vector3d(.046,-.0525,0),
      Eigen::Vector3d(-.05,.0525,0),Eigen::Vector3d(.1/3.,0,0),1.);
  const scan_planner::TurningRateLimits limits;
  const auto bound=scan_planner::boundTurningRate(curve,limits,scan_planner::turningRateDomain(curve,limits));
  ASSERT_TRUE(bound.complete());
  EXPECT_GE(bound.continuous_bound,5.);
  EXPECT_LT(bound.continuous_bound,5.10);
  EXPECT_NEAR(bound.sampled_peak,5.,1e-8);
  EXPECT_GE(bound.continuous_bound,sampledTurningPeak(curve,.02));
  EXPECT_LE(scan_planner::derivativeControlBound(curve.getDerivative()),.45);
  EXPECT_LE(scan_planner::derivativeControlBound(curve.getDerivative().getDerivative()),.35);
}

TEST(UniformBspline, FrozenDirectionDomainCannotEscapeTurningByDroppingBelowPhysicalThreshold) {
  auto curve=polynomialCurve(Eigen::Vector3d::Zero(),Eigen::Vector3d(.03,0,0),
      Eigen::Vector3d(0,.06,0),Eigen::Vector3d::Zero(),.25);
  const scan_planner::TurningRateLimits limits;
  const auto domain=scan_planner::turningRateDomain(curve,limits);
  const auto original=scan_planner::boundTurningRate(curve,limits,domain);
  curve.scaleTime(3.);
  EXPECT_LT(scan_planner::derivativeControlBound(curve.getDerivative()),.02);
  const auto slowed=scan_planner::boundTurningRate(curve,limits,domain);
  ASSERT_TRUE(original.complete()); ASSERT_TRUE(slowed.complete());
  EXPECT_GT(slowed.continuous_bound,limits.maximum_yaw_rate);
  EXPECT_NEAR(slowed.sampled_peak,4./3.,1e-9);
  EXPECT_NEAR(slowed.domain_speed_threshold,.02/3.,1e-12);
  EXPECT_NEAR(domain.reference_duration,.25,1e-12);
}

TEST(UniformBspline, StationaryTurningTimeAllocationPreservesXYZAndZeroBoundary) {
  auto original=stationaryCorner(),candidate=original;
  const scan_planner::TurningRateLimits limits;
  const auto result=scan_planner::retimeStationaryWithTurningLimits(candidate,.45,.35,limits);
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_GT(result.time_scale,1.);
  EXPECT_TRUE(candidate.getControlPoint().isApprox(original.getControlPoint(),0.));
  EXPECT_LE(result.speed_bound,.45+1e-9);
  EXPECT_LE(result.acceleration_bound,.35+1e-9);
  EXPECT_LE(result.turning.continuous_bound,.50+1e-9);
  EXPECT_LE(sampledTurningPeak(candidate,result.turning.domain_speed_threshold),.50+1e-9);
  EXPECT_NEAR(result.turning_domain.reference_duration,original.getTimeSum(),1e-12);
  EXPECT_NEAR(result.turning.domain_speed_threshold,.02/result.time_scale,1e-12);
  EXPECT_GT(result.turning.low_direction_spans,0u);
  for (int i=0;i<=200;++i) {
    const double t=original.getTimeSum()*i/200.;
    EXPECT_LT((candidate.evaluateDeBoorT(t*result.time_scale)-original.evaluateDeBoorT(t)).norm(),1e-10);
  }
  EXPECT_LT(candidate.getDerivative().evaluateDeBoorT(0.).norm(),1e-10);
  EXPECT_LT(candidate.getDerivative().getDerivative().evaluateDeBoorT(0.).norm(),1e-10);
}

TEST(UniformBspline, StraightTurningTimeAllocationDoesNotInventAngularDelay) {
  auto original=stationaryCorner();
  auto controls=original.getControlPoint(); controls.row(1).setZero();
  scan_planner::UniformBspline candidate(controls,3,2.);
  const double duration=candidate.getTimeSum();
  const auto result=scan_planner::retimeStationaryWithTurningLimits(candidate,.45,.35,{});
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_DOUBLE_EQ(result.time_scale,1.);
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
  EXPECT_LT(result.turning.continuous_bound,1e-8);
}

TEST(UniformBspline, MovingTurningAllocationPreservesMeasuredPVAAndRechecksActualInterior) {
  const auto boundary=turningBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  const double original_duration=candidate.getTimeSum();
  int refits=0;
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [&refits](Eigen::MatrixXd &,double) { ++refits; return true; },8,
      scan_planner::TurningRateLimits{});
  ASSERT_TRUE(result.success)<<result.reason<<" yaw="<<result.turning.continuous_bound;
  EXPECT_GT(refits,0);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary,1e-9));
  EXPECT_LE(result.speed_bound,.45+1e-9);
  EXPECT_LE(result.acceleration_bound,.35+1e-9);
  EXPECT_LE(result.turning.continuous_bound,.50+1e-9);
  EXPECT_LE(sampledTurningPeak(candidate,result.turning.domain_speed_threshold),.50+1e-9);
  EXPECT_DOUBLE_EQ(result.turning_domain.reference_duration,original_duration);
  EXPECT_NEAR(candidate.getDerivative().evaluateDeBoorT(0.).x(),.1,1e-10);
}

TEST(UniformBspline, MovingTurningRejectsImpossibleInstantaneousBoundaryWithoutRefit) {
  auto boundary=turningBoundary();
  boundary.start_velocity=Eigen::Vector3d(.03,0,0);
  boundary.start_acceleration=Eigen::Vector3d(0,.03,0);
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.4,boundary),3,.4);
  const auto before=candidate.getControlPoint(); const double duration=candidate.getTimeSum();
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [](Eigen::MatrixXd &,double) { ADD_FAILURE()<<"instantaneous rate cannot be refit"; return true; },8,
      scan_planner::TurningRateLimits{});
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"initial_turning_rate_exceeds_limit");
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, MovingTurningRechecksSmallRealBoundaryWhenFrozenDomainBecomesConfident) {
  auto boundary=turningBoundary();
  boundary.start_velocity=Eigen::Vector3d(.015,0,0);
  boundary.start_acceleration=Eigen::Vector3d(0,.015,0);
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.1,boundary),3,.1);
  const auto before=candidate.getControlPoint();
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [](Eigen::MatrixXd &,double) { return true; },8,scan_planner::TurningRateLimits{});
  EXPECT_FALSE(result.success); EXPECT_GT(result.refinements,0);
  EXPECT_EQ(result.reason,"initial_turning_rate_exceeds_limit");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, TurningAnalysisUsesNonuniformRealKnotsAndTranslatedKnotOrigin) {
  auto curve=stationaryCorner();
  auto knots=curve.getKnot();
  for (int i=0;i<knots.size();++i) knots[i]=10.+.09*i+.013*i*i;
  curve.setKnot(knots);
  const scan_planner::TurningRateLimits limits;
  const auto domain=scan_planner::turningRateDomain(curve,limits);
  const auto bound=scan_planner::boundTurningRate(curve,limits,domain);
  ASSERT_TRUE(bound.complete());
  EXPECT_GE(bound.continuous_bound,sampledTurningPeak(curve,.02));
  EXPECT_GT(bound.continuous_bound,.5);
  auto translated=curve;
  for (int i=0;i<knots.size();++i) knots[i]-=10.;
  translated.setKnot(knots);
  const auto other=scan_planner::boundTurningRate(translated,limits,domain);
  ASSERT_TRUE(other.complete());
  EXPECT_NEAR(bound.continuous_bound,other.continuous_bound,1e-8);
}

TEST(UniformBspline, LowDirectionVerticalStandstillAndCuspAreReportedNotPhysicalYawProof) {
  const scan_planner::TurningRateLimits limits;
  for (bool vertical : {false,true}) {
    Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,8);
    if (vertical) for(int i=0;i<controls.cols();++i) controls(2,i)=i*.1;
    scan_planner::UniformBspline curve(controls,3,.3);
    const auto bound=scan_planner::boundTurningRate(curve,limits,scan_planner::turningRateDomain(curve,limits));
    ASSERT_TRUE(bound.complete());
    EXPECT_DOUBLE_EQ(bound.continuous_bound,0.);
    EXPECT_EQ(bound.low_direction_spans,static_cast<std::size_t>(controls.cols()-3));
  }
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,9);
  controls.row(0)<<0,0,0,.5,1.,.5,0,0,0;
  scan_planner::UniformBspline cusp(controls,3,.3);
  const auto bound=scan_planner::boundTurningRate(cusp,limits,scan_planner::turningRateDomain(cusp,limits));
  ASSERT_TRUE(bound.complete());
  EXPECT_GT(bound.low_direction_spans,0u);
  // The tangent reverses across a zero-speed point: this result does NOT prove
  // finite body yaw or safe hold-yaw re-entry there.
  EXPECT_LT(bound.continuous_bound,1e-8);
}

TEST(UniformBspline, HugeFiniteKnotOriginCannotRoundMidpointOntoEndpointAndUnderestimateTurning) {
  // At 1e16, origin+1 rounds to origin, while every represented knot remains
  // strictly ordered. Sampling the unnormalized midpoint invents a different
  // velocity polynomial and incorrectly bounds this 2.5 rad/s peak below .5.
  Eigen::MatrixXd controls(3,4);
  controls << 0.,3.042,2.084,5.126,
              0.,-.21,-.21,0.,
              0.,0.,0.,0.;
  scan_planner::UniformBspline shifted(controls,3,2.),canonical=shifted;
  auto knots=shifted.getKnot();
  for (int i=0;i<knots.size();++i) knots[i]=1e16+2.*(i-3);
  shifted.setKnot(knots);
  const auto original_knots=shifted.getKnot();
  const scan_planner::TurningRateLimits limits;
  const auto bound=scan_planner::boundTurningRate(shifted,limits,
      scan_planner::turningRateDomain(shifted,limits));
  const auto reference=scan_planner::boundTurningRate(canonical,limits,
      scan_planner::turningRateDomain(canonical,limits));
  ASSERT_TRUE(bound.complete()); ASSERT_TRUE(reference.complete());
  EXPECT_GE(bound.continuous_bound,2.5);
  EXPECT_NEAR(bound.continuous_bound,reference.continuous_bound,1e-12);
  EXPECT_NEAR(bound.sampled_peak,2.5,1e-10);
  EXPECT_TRUE(shifted.getKnot().isApprox(original_knots,0.));
}

TEST(UniformBspline, UnrepresentableLateSpanInteriorIsExplicitInvalidNotFalseFiniteProof) {
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,5);
  controls.row(0)<<0.,.2,.4,.6,.8;
  scan_planner::UniformBspline curve(controls,3,1.);
  Eigen::VectorXd knots(9);
  knots << -3.,-2.,-1.,0.,1e16,1e16+2.,1e16+4.,1e16+6.,1e16+8.;
  curve.setKnot(knots);
  const scan_planner::TurningRateLimits limits;
  const auto bound=scan_planner::boundTurningRate(curve,limits,
      scan_planner::turningRateDomain(curve,limits));
  EXPECT_FALSE(bound.complete());
  EXPECT_EQ(bound.status,scan_planner::TurningRateAnalysisStatus::InvalidCurve);
}

TEST(UniformBspline, TurningAnalysisLimitsInvalidInputAndCancellationAreExplicit) {
  auto curve=stationaryCorner();
  scan_planner::TurningRateLimits limits;
  const auto domain=scan_planner::turningRateDomain(curve,limits);
  limits.maximum_analysis_nodes=1;
  EXPECT_EQ(scan_planner::boundTurningRate(curve,limits,domain).status,
      scan_planner::TurningRateAnalysisStatus::AnalysisLimit);
  limits=scan_planner::TurningRateLimits{};
  EXPECT_EQ(scan_planner::boundTurningRate(curve,limits,domain,[] { return false; }).status,
      scan_planner::TurningRateAnalysisStatus::BudgetExhausted);
  auto knots=curve.getKnot(); knots[4]=knots[3]; curve.setKnot(knots);
  EXPECT_EQ(scan_planner::boundTurningRate(curve,limits,domain).status,
      scan_planner::TurningRateAnalysisStatus::InvalidCurve);
  limits.maximum_yaw_rate=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(scan_planner::boundTurningRate(curve,limits,domain),std::invalid_argument);
  EXPECT_THROW(scan_planner::turningRateDomain(scan_planner::UniformBspline(),{}),std::invalid_argument);
}

TEST(UniformBspline, TurningTimingBudgetCannotPublishPartiallyScaledOrRefitInput) {
  auto candidate=stationaryCorner(); const auto before=candidate.getControlPoint();
  const double duration=candidate.getTimeSum(); int checks=0;
  const auto result=scan_planner::retimeStationaryWithTurningLimits(candidate,.45,.35,{},
      [&checks] { return ++checks<20; });
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"timing_budget_exhausted");
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
  const auto boundary=turningBoundary();
  candidate=scan_planner::UniformBspline(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  const auto moving_before=candidate.getControlPoint();
  bool cancelled=false;
  const auto moving=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [&cancelled](Eigen::MatrixXd &,double) { cancelled=true; return true; },8,
      scan_planner::TurningRateLimits{},[&cancelled] { return !cancelled; });
  EXPECT_FALSE(moving.success); EXPECT_EQ(moving.reason,"timing_budget_exhausted");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(moving_before,0.));
}

TEST(UniformBspline, TurningUniformAllocationCannotPretendMovingBoundaryWasStationary) {
  const auto boundary=turningBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  const auto before=candidate.getControlPoint();
  const auto result=scan_planner::retimeStationaryWithTurningLimits(candidate,.45,.35,{});
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"moving_initial_boundary_forbids_uniform_timing");
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
}

TEST(UniformBspline, RecordedReverseStartContinuousBoundIsTightDespiteAlreadyBeingBelowLimit) {
  // Saved actual 30-CDR candidate. Previous .49 early-stop returned .467984
  // from just 12 nodes and its 3-point diagnostic .0147 missed the confidence
  // threshold re-entry peak. The true domain maximum is about .1821691.
  Eigen::MatrixXd controls(3,15);
  controls << -.09471861318265631,-.0042188024749944125,.08628100823266749,
      .10513174361417274,.059487146321462785,-.02418805322703538,
      -.12670675097035636,-.23511509091932392,-.3366081393404992,
      -.41885269272780873,-.4743204407493045,-.5032284147058101,
      -.5125460447123965,-.5125460447123965,-.5125460447123965,
      -.2511615259950353,-.41503794865274757,-.5789143713104599,
      -.6210378875616903,-.5078075404419584,-.26238438540568154,
      .0645828137682228,.41855224621416226,.7539487924447307,
      1.037811683296432,1.2489799506621941,1.378039313782267,
      1.4300300231420027,1.4300300231420027,1.4300300231420027,
      .026616666832977903,.03846257079475542,.05030847475653294,
      -.009255136807393313,.07456873811851532,-.008780779670619097,
      .01726249369762354,.005085988890917405,.01489481967141304,
      .007825845271522497,.005494866983676054,.013496711616926503,
      .006693167500979901,.006693167500979901,.006693167500979901;
  scan_planner::UniformBspline curve(controls,3,9.96226755647541);
  const scan_planner::TurningRateDomain domain{7.471700667356558,.14943401334713116};
  const auto bound=scan_planner::boundTurningRate(curve,{},domain);
  ASSERT_TRUE(bound.complete());
  EXPECT_GE(bound.continuous_bound,.1821690);
  EXPECT_LT(bound.continuous_bound,.184);
  EXPECT_GT(bound.sampled_peak,.180);
  EXPECT_LE(bound.continuous_bound,bound.sampled_peak*1.01+1e-7);
  EXPECT_LT(bound.analysis_nodes,100u);
  EXPECT_NEAR(bound.domain_speed_threshold,.00125,1e-12);
}

TEST(UniformBspline, MovingTimingTightensDurationWithoutExceedingOriginalRefinementQuota) {
  const auto boundary=turningBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  const scan_planner::TurningRateLimits limits;
  const auto domain=scan_planner::turningRateDomain(candidate,limits);
  std::vector<scan_planner::UniformBspline> inspected_refits;
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [&inspected_refits](Eigen::MatrixXd &controls,double dt) {
        inspected_refits.emplace_back(controls,3,dt); return true;
      },8,limits);
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_GT(result.tightening_trials,0);
  EXPECT_LE(result.refinements,8);
  EXPECT_EQ(result.refinements,static_cast<int>(inspected_refits.size()));
  double shortest_verified=std::numeric_limits<double>::infinity(), longest_verified=0.;
  for (auto &trial:inspected_refits) {
    auto v=trial.getDerivative();
    const auto turn=scan_planner::boundTurningRate(trial,limits,domain);
    if (turn.complete() && turn.continuous_bound<=.5+1e-9 &&
        scan_planner::derivativeControlBound(v)<=.45+1e-9 &&
        scan_planner::derivativeControlBound(v.getDerivative())<=.35+1e-9 &&
        scan_planner::cubicBoundaryMatches(trial,boundary)) {
      shortest_verified=std::min(shortest_verified,trial.getTimeSum());
      longest_verified=std::max(longest_verified,trial.getTimeSum());
    }
  }
  EXPECT_NEAR(candidate.getTimeSum(),shortest_verified,1e-10);
  EXPECT_LT(candidate.getTimeSum(),longest_verified*.99);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary,1e-9));
  EXPECT_LE(result.turning.continuous_bound,.5+1e-9);
}

TEST(UniformBspline, OptionalTimingTighteningCancellationCannotPublishEarlierFeasibleCurve) {
  const auto boundary=turningBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  const auto before=candidate.getControlPoint(); const double duration=candidate.getTimeSum();
  bool cancelled=false; double previous_dt=0.; int refits=0;
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [&](Eigen::MatrixXd &,double dt) {
        ++refits; if (previous_dt>0. && dt<previous_dt) cancelled=true;
        previous_dt=dt; return true;
      },8,scan_planner::TurningRateLimits{},[&] { return !cancelled; });
  EXPECT_TRUE(cancelled);
  EXPECT_FALSE(result.success); EXPECT_EQ(result.reason,"timing_budget_exhausted");
  EXPECT_LE(refits,8);
  EXPECT_TRUE(candidate.getControlPoint().isApprox(before,0.));
  EXPECT_DOUBLE_EQ(candidate.getTimeSum(),duration);
}

TEST(UniformBspline, FailedOptionalTimingRefineRetainsOnlyPreviouslyAnalysedBoundaryCurve) {
  const auto boundary=turningBoundary();
  scan_planner::UniformBspline candidate(
      scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
  double previous_dt=0.; bool optional_failure=false; int refits=0;
  const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
      [&](Eigen::MatrixXd &controls,double dt) {
        ++refits;
        if (previous_dt>0. && dt<previous_dt) {
          optional_failure=true; controls(0,0)+=1.; return false;
        }
        previous_dt=dt; return true;
      },8,scan_planner::TurningRateLimits{});
  EXPECT_TRUE(optional_failure);
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_EQ(result.timing_search_stop_reason,"optional_interior_refinement_failed");
  EXPECT_LE(refits,8);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary,1e-9));
  EXPECT_LE(result.speed_bound,.45+1e-9);
  EXPECT_LE(result.acceleration_bound,.35+1e-9);
  EXPECT_LE(result.turning.continuous_bound,.5+1e-9);
}

TEST(UniformBspline, InvalidSuccessfulOptionalRefineCannotErasePreviouslyVerifiedCurve) {
  for (const bool wrong_shape:{false,true}) {
    const auto boundary=turningBoundary();
    scan_planner::UniformBspline candidate(
        scan_planner::fitCubicWithFixedBoundary(cornerSamples(),.15,boundary),3,.15);
    double previous_dt=0.; bool invalid_output=false; int refits=0;
    const auto result=scan_planner::refineTimingWithFixedBoundary(candidate,boundary,.45,.35,
        [&](Eigen::MatrixXd &controls,double dt) {
          ++refits;
          if (previous_dt>0. && dt<previous_dt) {
            invalid_output=true;
            if (wrong_shape) controls.conservativeResize(2,controls.cols());
            else controls(0,0)=std::numeric_limits<double>::quiet_NaN();
          }
          previous_dt=dt; return true;
        },8,scan_planner::TurningRateLimits{});
    EXPECT_TRUE(invalid_output);
    ASSERT_TRUE(result.success)<<result.reason;
    EXPECT_EQ(result.timing_search_stop_reason,"optional_invalid_refinement_output");
    EXPECT_LE(refits,8);
    EXPECT_TRUE(candidate.getControlPoint().allFinite());
    EXPECT_TRUE(scan_planner::cubicBoundaryMatches(candidate,boundary,1e-9));
    EXPECT_LE(result.turning.continuous_bound,.5+1e-9);
  }
}

TEST(UniformBspline, ActualReverseStartVelocityAccelerationAllocationDoesNotPretendTangentIsBodyYaw) {
  // Same actual recorded candidate before the tangent/body-yaw conflation.
  // Preserve its real small reverse velocity and input zero acceleration. A
  // >.5 tangent diagnostic must not stretch a v/a-feasible spatial trajectory.
  Eigen::MatrixXd controls(3,15);
  controls << -.013553169240385321,-.0042188024749944125,.005115564290396496,
      -.00009601846838012875,-.028138029347123953,-.08224739644438042,
      -.15972417033516983,-.25117787797148833,-.34238879962454416,
      -.4192798672723144,-.47305086560774334,-.5024105530497455,
      -.5125460447123965,-.5125460447123965,-.5125460447123965,
      -.39813534276522167,-.41503794865274757,-.43194055454027347,
      -.3889704499391295,-.2643147165363438,-.06250609539203161,
      .1963297486565971,.48498561048915534,.7741512012784048,
      1.0348966140874363,1.2410900453373088,1.3740265212798117,
      1.4300300231420027,1.4300300231420027,1.4300300231420027,
      .03724075594915302,.03846257079475542,.039684385640357825,
      -.0035617046219197056,.0722804123278143,-.007963239754135074,
      .01698867023075574,.005174032269281753,.014867301506947475,
      .007834257595147108,.005492371795012337,.013497353238353455,
      .006693167500979901,.006693167500979901,.006693167500979901;
  scan_planner::UniformBspline curve(controls,3,1.0275320849839449);
  auto velocity=curve.getDerivative(),acceleration=velocity.getDerivative();
  const double duration=curve.getTimeSum();
  const scan_planner::CubicMotionBoundary boundary{
      curve.evaluateDeBoorT(0.),velocity.evaluateDeBoorT(0.),acceleration.evaluateDeBoorT(0.),
      curve.evaluateDeBoorT(duration),velocity.evaluateDeBoorT(duration),
      acceleration.evaluateDeBoorT(duration)};
  int refits=0;
  const auto timing=scan_planner::refineTimingWithFixedBoundary(curve,boundary,.45,.35,
      [&](Eigen::MatrixXd &,double) { ++refits;return true; },8,std::nullopt);
  ASSERT_TRUE(timing.success)<<timing.reason;
  EXPECT_EQ(refits,0);
  EXPECT_DOUBLE_EQ(curve.getTimeSum(),duration);
  EXPECT_TRUE(curve.getControlPoint().isApprox(controls,0.));
  EXPECT_NEAR(boundary.start_velocity.x(),.00908425819670318,1e-12);
  EXPECT_NEAR(boundary.start_velocity.y(),-.01644971104507163,1e-12);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(curve,boundary,1e-9));
  const scan_planner::TurningRateLimits diagnostics;
  const auto domain=scan_planner::turningRateDomain(curve,diagnostics);
  const auto turn=scan_planner::boundTurningRate(curve,diagnostics,domain);
  ASSERT_TRUE(turn.complete());
  EXPECT_NEAR(turn.domain_speed_threshold,.02,1e-12);
  EXPECT_GT(turn.continuous_bound,.7676);
  EXPECT_LT(turn.continuous_bound,.78);
  EXPECT_GT(turn.continuous_bound,diagnostics.maximum_yaw_rate);
  EXPECT_TRUE(timing.success);  // independent diagnosis cannot mutate acceptance
}
