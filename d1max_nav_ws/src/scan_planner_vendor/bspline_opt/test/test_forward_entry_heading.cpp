#include <gtest/gtest.h>
#include <bspline_opt/forward_entry_heading.hpp>
#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/whole_spline_collision.hpp>
#include <chrono>
#include <vector>

using namespace scan_planner;
namespace {
UniformBspline acquiredReverseCurve() {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,23);
  const double prefix[]={.03,0.,-.03,-.03,-.015,.015,.06};
  for(int i=0;i<points.cols();++i)points(0,i)=i<7?prefix[i]:.06+.06*(i-6);
  return UniformBspline(points,3,.3);
}
SplineHeadingContract forwardContract() {
  SplineHeadingContract c;c.measured_heading_enabled=true;c.align_then_forward=true;return c;
}
}
TEST(ForwardEntryHeading, OriginalReverseVelocityAndCoordinatesRemainButHeadingFacesActualForwardGeometry) {
  auto curve=acquiredReverseCurve();auto velocity=curve.getDerivative();auto c=forwardContract();
  const auto original_controls=curve.getControlPoint();const auto v0=velocity.evaluateDeBoorT(0.);
  ASSERT_NEAR(v0.x(),-.1,1e-12);unsigned reads=0;
  ASSERT_TRUE(resolveForwardEntryHeading(c,curve.getTimeSum(),500,
    [&](double t){++reads;return curve.evaluateDeBoorT(t);},
    [&](double t){++reads;return velocity.evaluateDeBoorT(t);},[]{return true;}));
  ASSERT_GT(c.forward_entry_end,0.);EXPECT_LT(c.forward_entry_end,curve.getTimeSum());
  // This exceptional contract is bounded by the existing 0.4 m geometry
  // horizon, not a one-second first-positive-tangent implementation detail.
  double prefix_arc=0.;auto previous=curve.evaluateDeBoorT(0.);
  for(int i=1;i<=1000;++i) {
    const auto p=curve.evaluateDeBoorT(c.forward_entry_end*double(i)/1000.);
    prefix_arc+=(p-previous).head<2>().norm();previous=p;
  }
  EXPECT_LE(prefix_arc,.401);
  EXPECT_DOUBLE_EQ(c.forward_entry_yaw,0.);EXPECT_LT(reads,1003U);
  EXPECT_DOUBLE_EQ(bodyHeadingAtCurveTime(v0,.4,c,0.),0.);
  const auto release=velocity.evaluateDeBoorT(c.forward_entry_end);
  EXPECT_GT(release.x(),0.);EXPECT_NEAR(bodyHeadingAtCurveTime(release,.4,c,c.forward_entry_end),0.,1e-12);
  EXPECT_TRUE(curve.getControlPoint().isApprox(original_controls,0.));
  EXPECT_TRUE(velocity.evaluateDeBoorT(0.).isApprox(v0,0.));
}
TEST(ForwardEntryHeading, GenuineBehindRouteAndNormalBendAreNotReinterpretedAsForward) {
  for(bool bend:{false,true}) {
    auto c=forwardContract();
    const auto p=[=](double t)->Eigen::Vector3d {return bend?Eigen::Vector3d(std::sin(t),1.-std::cos(t),0.):Eigen::Vector3d(-.1*t,0.,0.);};
    const auto v=[=](double t)->Eigen::Vector3d {return bend?Eigen::Vector3d(std::cos(t),std::sin(t),0.):Eigen::Vector3d(-.1,0.,0.);};
    ASSERT_TRUE(resolveForwardEntryHeading(c,2.,100,p,v,[]{return true;}));
    EXPECT_DOUBLE_EQ(c.forward_entry_end,-1.);
    for(double at:{.5,M_PI/2.,1.9})
      EXPECT_NEAR(bodyHeadingAtCurveTime(v(at),0.,c,at),bend?at:M_PI,1e-12);
  }
}
TEST(ForwardEntryHeading, LongReversePrefixDoesNotLookPastTheBoundedSpatialHorizon) {
  auto c=forwardContract();
  const auto p=[](double t)->Eigen::Vector3d {return t<5.?Eigen::Vector3d(-.1*t,0.,0.):Eigen::Vector3d(-.5+.2*(t-5.),0.,0.);};
  const auto v=[](double t)->Eigen::Vector3d {return t<5.?Eigen::Vector3d(-.1,0.,0.):Eigen::Vector3d(.2,0.,0.);};
  ASSERT_TRUE(resolveForwardEntryHeading(c,10.,500,p,v,[]{return true;}));
  EXPECT_DOUBLE_EQ(c.forward_entry_end,-1.);
}
TEST(ForwardEntryHeading, HoldHeadingDisabledStrategyCancellationAndInvalidSourceStayDistinct) {
  auto curve=acquiredReverseCurve();auto v=curve.getDerivative();auto c=forwardContract();
  const auto p=[&](double t){return curve.evaluateDeBoorT(t);};const auto velocity=[&](double t){return v.evaluateDeBoorT(t);};
  c.policy=BodyHeadingPolicy::HoldHeading;c.measured_yaw=.7;
  EXPECT_TRUE(resolveForwardEntryHeading(c,6.,500,p,velocity,[]{return true;}));EXPECT_DOUBLE_EQ(c.forward_entry_end,-1.);
  EXPECT_DOUBLE_EQ(bodyHeadingAtCurveTime(velocity(0.),0.,c,0.),.7);
  c=forwardContract();c.align_then_forward=false;
  EXPECT_TRUE(resolveForwardEntryHeading(c,6.,500,p,velocity,[]{return true;}));EXPECT_DOUBLE_EQ(c.forward_entry_end,-1.);
  c=forwardContract();unsigned reads=0;
  EXPECT_FALSE(resolveForwardEntryHeading(c,6.,500,p,velocity,[&]{return ++reads<5;}));
  EXPECT_FALSE(resolveForwardEntryHeading(c,6.,50000,p,velocity,[]{return true;}));
  EXPECT_FALSE(resolveForwardEntryHeading(c,6.,500,p,[](double)->Eigen::Vector3d {return {NAN,0.,0.};},[]{return true;}));
}
TEST(ForwardEntryHeading, BoundedPreparationCostNotPerCollisionQueryOrControlTick) {
  auto curve=acquiredReverseCurve();auto velocity=curve.getDerivative();std::vector<double> times;
  for(int i=0;i<200;++i) {
    auto c=forwardContract();const auto begin=std::chrono::steady_clock::now();
    ASSERT_TRUE(resolveForwardEntryHeading(c,curve.getTimeSum(),500,
      [&](double t){return curve.evaluateDeBoorT(t);},[&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
    times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count());
  }
  std::sort(times.begin(),times.end());
  RecordProperty("prefix_preparation_p95_us",std::to_string(times[190]));
  RecordProperty("prefix_preparation_max_us",std::to_string(times.back()));
}

namespace {
UniformBspline lateralAcquiredReverse(double transverse,double rotation=0.) {
  auto original=acquiredReverseCurve();auto points=original.getControlPoint();
  for(int i=0;i<points.cols();++i) {
    const double x=points(0,i),y=transverse*.3*(i-1);
    points(0,i)=std::cos(rotation)*x-std::sin(rotation)*y;
    points(1,i)=std::sin(rotation)*x+std::cos(rotation)*y;
  }
  return UniformBspline(points,3,.3);
}
double headingDelta(double a,double b) {return std::atan2(std::sin(a-b),std::cos(a-b));}
}

TEST(ForwardEntryHeading, TransverseAcquisitionDoesNotSelectTheFirstPositiveLongitudinalSingularity) {
  for(double transverse:{-.03,.03})for(double rotation:{0.,179.*M_PI/180.,-179.*M_PI/180.}) {
    SCOPED_TRACE(::testing::Message()<<"vy="<<transverse<<" seed="<<rotation);
    auto curve=lateralAcquiredReverse(transverse,rotation);auto velocity=curve.getDerivative();
    const auto original_points=curve.getControlPoint(),original_v=velocity.getControlPoint();
    auto c=forwardContract();c.measured_yaw=rotation;
    ASSERT_TRUE(resolveForwardEntryHeading(c,curve.getTimeSum(),500,
      [&](double t){return curve.evaluateDeBoorT(t);},
      [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
    auto canonical_curve=lateralAcquiredReverse(transverse);auto canonical_velocity=canonical_curve.getDerivative();
    auto canonical=forwardContract();
    ASSERT_TRUE(resolveForwardEntryHeading(canonical,canonical_curve.getTimeSum(),500,
      [&](double t){return canonical_curve.evaluateDeBoorT(t);},
      [&](double t){return canonical_velocity.evaluateDeBoorT(t);},[]{return true;}));
    ASSERT_GT(c.forward_entry_end,0.);
    EXPECT_LT(std::abs(headingDelta(bodyHeadingAtCurveTime(velocity.evaluateDeBoorT(0.),rotation,c,0.),rotation)),.30);
    EXPECT_TRUE(curve.getControlPoint().isApprox(original_points,0.));
    EXPECT_TRUE(velocity.getControlPoint().isApprox(original_v,0.));
    // Native sampling and independent bounded replay agree at the same source
    // curve coordinates, even when a consumer queries backwards in progress.
    for(double at:{c.forward_entry_end+1e-7,c.forward_entry_end-1e-7,.46,.50,.46,.50}) {
      const auto one=sampledBodyHeadingAt(at,curve.getTimeSum(),500,c,
        [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
      const auto two=sampledBodyHeadingAt(at,curve.getTimeSum(),500,c,
        [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
      ASSERT_TRUE(one);ASSERT_TRUE(two);EXPECT_DOUBLE_EQ(*one,*two);
      const auto unrotated=sampledBodyHeadingAt(at,canonical_curve.getTimeSum(),500,canonical,
        [&](double t){return canonical_velocity.evaluateDeBoorT(t);},[]{return true;});
      ASSERT_TRUE(unrotated);EXPECT_NEAR(headingDelta(*one,*unrotated+rotation),0.,1e-10);
    }
  }
}

TEST(ForwardEntryHeading, NativeCollisionKeepsTheIntermediateBodyRotationSweep) {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,23);
  for(int i=0;i<points.cols();++i)points(0,i)=(i-1)*.06;
  UniformBspline curve(points,3,.3);auto c=forwardContract();c.measured_yaw=-M_PI/2.;
  // A point is outside the 0.93 x 0.30 m body at both endpoint orientations,
  // but inside the box during its intermediate -45 degree rotation. Keep
  // the production whole-spline rotation sweep, not just endpoint occupancy.
  const Eigen::Vector2d obstacle(.31,-.31);
  const auto occupied=[&](const Eigen::Vector3d& p,double yaw) {
    const Eigen::Vector2d d=obstacle-p.head<2>();
    const double x=std::cos(yaw)*d.x()+std::sin(yaw)*d.y();
    const double y=-std::sin(yaw)*d.x()+std::cos(yaw)*d.y();
    return std::abs(x)<=.465&&std::abs(y)<=.15?1:0;
  };
  const auto start=curve.evaluateDeBoorT(0.);
  ASSERT_EQ(occupied(start,c.measured_yaw),0);ASSERT_EQ(occupied(start,0.),0);
  const auto blocked=checkWholeSplineCollision(curve,.05,occupied,50000,c);
  EXPECT_EQ(blocked.state,SplineCollisionState::Collision);
  EXPECT_GT(blocked.queries,2u);EXPECT_LT(blocked.yaw,0.);EXPECT_GT(blocked.yaw,-M_PI/2.);
  EXPECT_EQ(checkWholeSplineCollision(curve,.05,
    [](const Eigen::Vector3d&,double){return 0;},50000,c).state,SplineCollisionState::Clear);
}

TEST(ForwardEntryHeading, ShortForwardGoalRetainsOriginalStopWithoutDiscardingReverseEntryHeading) {
  auto original=acquiredReverseCurve();auto points=original.getControlPoint();
  for(int i=7;i<points.cols();++i)points(0,i)=.06+.01*std::min(i-6,13);
  UniformBspline curve(points,3,.3);auto velocity=curve.getDerivative();
  const auto original_controls=curve.getControlPoint();const double end=curve.getTimeSum();
  ASSERT_NEAR(velocity.evaluateDeBoorT(0.).x(),-.1,1e-12);
  ASSERT_LT((curve.evaluateDeBoorT(end)-curve.evaluateDeBoorT(0.)).norm(),.4);
  ASSERT_NEAR(velocity.evaluateDeBoorT(end).norm(),0.,1e-12);
  auto c=forwardContract();
  ASSERT_TRUE(resolveForwardEntryHeading(c,end,500,
    [&](double t){return curve.evaluateDeBoorT(t);},
    [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
  ASSERT_GT(c.forward_entry_end,0.)<<"a real zero-speed terminal boundary is not a backwards route";
  for(double at:{0.,.46,.50,end-.02,end}) {
    const auto yaw=sampledBodyHeadingAt(at,end,500,c,
      [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
    ASSERT_TRUE(yaw);EXPECT_NEAR(headingDelta(*yaw,0.),0.,1e-12);
  }
  EXPECT_TRUE(curve.getControlPoint().isApprox(original_controls,0.));
  EXPECT_DOUBLE_EQ(velocity.evaluateDeBoorT(end).norm(),0.);
}

TEST(ForwardEntryHeading, ReliableReverseTerminalTangentIsNotTrimmedAsStationaryTail) {
  auto original=acquiredReverseCurve();auto points=original.getControlPoint();
  for(int i=7;i<points.cols();++i)points(0,i)=.06+.01*std::min(i-6,10);
  points(0,20)=.15;points(0,21)=.14;points(0,22)=.13;
  UniformBspline curve(points,3,.3);auto velocity=curve.getDerivative();const double end=curve.getTimeSum();
  ASSERT_LT(velocity.evaluateDeBoorT(end).x(),-.02);auto c=forwardContract();
  ASSERT_TRUE(resolveForwardEntryHeading(c,end,500,
    [&](double t){return curve.evaluateDeBoorT(t);},
    [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
  const auto yaw=sampledBodyHeadingAt(end,end,500,c,
    [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
  ASSERT_TRUE(yaw);EXPECT_NEAR(headingDelta(*yaw,M_PI),0.,1e-12);
}

TEST(ForwardEntryHeading, NativeCollisionUsesResolvedReversePrefixOrientationNotOnlyEndpointYaw) {
  const auto original=lateralAcquiredReverse(.03).getControlPoint();
  Eigen::MatrixXd points(3,43);points.leftCols(original.cols())=original;
  for(int i=original.cols();i<points.cols();++i)
    points.col(i)=original.col(original.cols()-1)+
      double(i-original.cols()+1)*(original.col(original.cols()-1)-original.col(original.cols()-2));
  UniformBspline curve(points,3,.3);auto velocity=curve.getDerivative();auto c=forwardContract();
  ASSERT_TRUE(resolveForwardEntryHeading(c,curve.getTimeSum(),500,
    [&](double t){return curve.evaluateDeBoorT(t);},
    [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
  const double at=.70*c.forward_entry_end;const auto p=curve.evaluateDeBoorT(at);
  const auto yaw=sampledBodyHeadingAt(at,curve.getTimeSum(),500,c,
    [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
  ASSERT_TRUE(yaw);ASSERT_GT(*yaw,0.);
  // Physical point near the body corner: clear if one wrongly keeps the
  // original yaw here, occupied for the shared resolved orientation. The
  // whole production query must check this intermediate prefix rotation.
  const Eigen::Vector2d corner(.46,.145);
  const Eigen::Vector2d obstacle=p.head<2>()+Eigen::Vector2d(
    std::cos(*yaw)*corner.x()-std::sin(*yaw)*corner.y(),
    std::sin(*yaw)*corner.x()+std::cos(*yaw)*corner.y());
  const auto occupied=[&](const Eigen::Vector3d& at_position,double at_yaw) {
    const Eigen::Vector2d d=obstacle-at_position.head<2>();
    const double x=std::cos(at_yaw)*d.x()+std::sin(at_yaw)*d.y();
    const double y=-std::sin(at_yaw)*d.x()+std::cos(at_yaw)*d.y();
    return std::abs(x)<=.465&&std::abs(y)<=.15?1:0;
  };
  ASSERT_EQ(occupied(p,*yaw),1);
  ASSERT_EQ(occupied(p,c.measured_yaw),0);
  ASSERT_EQ(occupied(curve.evaluateDeBoorT(0.),c.measured_yaw),0);
  const auto terminal_v=velocity.evaluateDeBoorT(curve.getTimeSum());
  ASSERT_EQ(occupied(curve.evaluateDeBoorT(curve.getTimeSum()),std::atan2(terminal_v.y(),terminal_v.x())),0);
  const auto result=checkWholeSplineCollision(curve,.05,occupied,50000,c);
  EXPECT_EQ(result.state,SplineCollisionState::Collision);EXPECT_GT(result.yaw,0.);
}
