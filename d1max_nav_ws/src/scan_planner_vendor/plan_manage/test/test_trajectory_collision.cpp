#include <gtest/gtest.h>
#include <thread>
#include <limits>
#include "plan_manage/trajectory_collision.hpp"

static scan_planner::UniformBspline line(double y=0.) {
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i) points.col(i)=Eigen::Vector3d((i-1)*.05,y,.55);
  return scan_planner::UniformBspline(points,3,.2);
}

static scan_planner::MeasuredBodyPose bodyAt(scan_planner::UniformBspline &curve, double yaw=0.) {
  return {curve.evaluateDeBoorT(0.),Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())),10.,"map"};
}

TEST(TrajectoryCollision, ActualRearConnectionChecksBothBranchesWithoutChangingCurveInternalHeadings) {
  auto curve=line();scan_planner::SplineHeadingContract h;
  h.measured_heading_enabled=true;h.align_then_forward=true;
  const double pi=std::acos(-1.);auto body=bodyAt(curve,100.*pi/180.);
  // Path heading is 0, so the old shortest actual-body turn is -100 deg.
  // The retained positive branch crosses 180 deg and cannot be signed clear.
  std::size_t connection_queries=0,curve_queries=0;
  const auto obstacle=[&](const Eigen::Vector3d& p,double yaw) {
    if(p.x()<1e-10)++connection_queries;else {++curve_queries;EXPECT_NEAR(yaw,0.,1e-10);}
    return p.x()<1e-10&&yaw>pi&&yaw<pi+.2?1:0;
  };
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,obstacle,body,"map",10.,.4,50000,0.,0.,h));
  EXPECT_GT(connection_queries,10U);EXPECT_EQ(curve_queries,0U);
  connection_queries=curve_queries=0;
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [&](const Eigen::Vector3d& p,double yaw) {
      if(p.x()<1e-10)++connection_queries;else {++curve_queries;EXPECT_NEAR(yaw,0.,1e-10);}
      return 0;
    },body,"map",10.,.4,50000,0.,0.,h));
  EXPECT_GT(connection_queries,500U);EXPECT_GT(curve_queries,0U);
}

TEST(TrajectoryCollision, RepeatedExactClearPoseKeepsLogicalBudgetAndFreshProofScope) {
  auto curve=line();scan_planner::SplineHeadingContract h;h.measured_heading_enabled=true;
  auto body=bodyAt(curve);std::size_t queries=0;
  scan_planner::CurveCheckCompletion completion;
  // The sampling lattice itself fits 203 samples, but its endpoint/heading
  // sweeps exceed that logical budget even after exact callbacks are reused.
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [&](const Eigen::Vector3d&,double){++queries;return 0;},body,"map",10.,.4,203,0.,0.,h,0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,&completion,
    std::numeric_limits<double>::infinity(),{},true));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::BudgetExhausted);
  EXPECT_GT(queries,1U);EXPECT_LT(queries,203U);
  // A new proof/map must ask again, and occupied/unknown still stop at the
  // first exact pose. No clear result survives the function invocation.
  for(int state:{1,2,-1}) {
    queries=0;
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
      [&](const Eigen::Vector3d&,double){++queries;return state;},body,"map",10.,.4,50000,0.,0.,h,0.,
      scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,&completion,
      std::numeric_limits<double>::infinity(),{},true));
    EXPECT_EQ(queries,1U);
  }
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [](const Eigen::Vector3d&,double){std::this_thread::sleep_for(std::chrono::milliseconds(2));return 0;},
    body,"map",10.,.4,50000,.001,0.,h,0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,&completion,
    std::numeric_limits<double>::infinity(),{},true));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::BudgetExhausted);
}

TEST(TrajectoryCollision, ExactClearPoseReuseNeverMergesNearbyPositionOrYaw) {
  auto curve=line();scan_planner::SplineHeadingContract h;h.measured_heading_enabled=true;
  auto body=bodyAt(curve,1e-10);body.position.y()=1e-10;
  bool actual_position=false,curve_position=false,actual_heading=false,curve_heading=false;
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [&](const Eigen::Vector3d& p,double yaw) {
      actual_position|=p.y()==1e-10;curve_position|=p.y()==0.;
      actual_heading|=yaw==1e-10;curve_heading|=yaw==0.;return 0;
    },body,"map",10.,.4,50000,0.,0.,h,0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,nullptr,
    std::numeric_limits<double>::infinity(),{},true));
  EXPECT_TRUE(actual_position);EXPECT_TRUE(curve_position);
  EXPECT_TRUE(actual_heading);EXPECT_TRUE(curve_heading);
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [](const Eigen::Vector3d& p,double yaw){return p.y()==0.&&yaw==0.?1:0;},
    body,"map",10.,.4,50000,0.,0.,h,0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,nullptr,
    std::numeric_limits<double>::infinity(),{},true));
}

TEST(TrajectoryCollision, GenericQueriesDoNotReuseExpiringOrCancelledExactPoses) {
  auto curve=line();scan_planner::SplineHeadingContract h;h.measured_heading_enabled=true;
  for(const int expired_state:{2,-1}) {
    std::size_t queries=0;Eigen::Vector3d first_position;double first_yaw=0.;
    scan_planner::CurveCheckCompletion completion;
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
      [&](const Eigen::Vector3d& p,double yaw) {
        if(++queries==1) {first_position=p;first_yaw=yaw;return 0;}
        EXPECT_EQ(p,first_position);EXPECT_EQ(yaw,first_yaw);
        // The immutable map can still have a dynamic observed-free deadline
        // or a callback-level cancellation check. The duplicate must run it.
        return expired_state;
      },bodyAt(curve),"map",10.,.4,50000,0.,0.,h,0.,
      scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,&completion));
    EXPECT_EQ(queries,2U);
    EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::Uncertified);
  }
}

TEST(TrajectoryCollision, RearConnectionUsesExistingRoundQueryAndWallBudgets) {
  auto curve=line();scan_planner::SplineHeadingContract h;
  h.measured_heading_enabled=true;h.align_then_forward=true;
  auto body=bodyAt(curve,std::acos(-1.));std::size_t queries=0;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [&](const Eigen::Vector3d&,double){++queries;return 0;},body,"map",10.,.4,100,0.,0.,h));
  EXPECT_LE(queries,100U);
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.55,
    [](const Eigen::Vector3d&,double){std::this_thread::sleep_for(std::chrono::milliseconds(2));return 0;},
    body,"map",10.,.4,50000,.001,0.,h));
}

TEST(TrajectoryCollision, AcquiredReverseEntryUsesSameForwardHeadingInWholeAndRemainingChecks) {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,23);
  const double prefix[]={.03,0.,-.03,-.03,-.015,.015,.06};
  for(int i=0;i<points.cols();++i){points(0,i)=i<7?prefix[i]:.06+.06*(i-6);points(2,i)=.55;}
  scan_planner::UniformBspline curve(points,3,.3);
  scan_planner::SplineHeadingContract h;h.measured_heading_enabled=true;h.align_then_forward=true;
  const auto narrow_heading=[](const Eigen::Vector3d&,double yaw){return std::abs(yaw)>.1?1:0;};
  EXPECT_EQ(scan_planner::checkWholeSplineCollision(curve,.05,narrow_heading,50000,h).state,
    scan_planner::SplineCollisionState::Clear);
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,narrow_heading,bodyAt(curve),"map",10.,.4,50000,0.,0.,h));
  // The original backwards geometry remains checked, including an obstacle
  // in the tiny prefix; this is not a near-body clearing or curve shortcut.
  const auto prefix_obstacle=[](const Eigen::Vector3d& p,double){return p.x()<-.02?1:0;};
  EXPECT_EQ(scan_planner::checkWholeSplineCollision(curve,.05,prefix_obstacle,50000,h).state,
    scan_planner::SplineCollisionState::Collision);
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,prefix_obstacle,bodyAt(curve),"map",10.,.4,50000,0.,0.,h));
  auto actual=bodyAt(curve,.6);h.measured_yaw=.6;
  const auto rotation_obstacle=[](const Eigen::Vector3d&,double yaw){return yaw>.2&&yaw<.4?1:0;};
  EXPECT_EQ(scan_planner::checkWholeSplineCollision(curve,.05,rotation_obstacle,50000,h).state,
    scan_planner::SplineCollisionState::Collision);
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,rotation_obstacle,actual,"map",10.,.4,50000,0.,0.,h));
}

template<class Query> static bool check(scan_planner::UniformBspline &curve, Query query,
    std::size_t budget=200000,double wall_budget=0.) {
  return scan_planner::wholeCurveCollisionFree(curve,.08,.49,query,bodyAt(curve),"map",10.,.5,
                                               budget,wall_budget);
}

TEST(TrajectoryCollision, PredecessorFullCurveIncludesFinalThirdAndEndpoint) {
  auto old=line(),replacement=line(1.);
  const auto occupied=[](const Eigen::Vector3d &p,double) {return p.x()>.85 && p.y()<.2 ? 1:0;};
  EXPECT_TRUE(check(replacement,occupied));
  EXPECT_FALSE(check(old,occupied));
}
TEST(TrajectoryCollision, MeasuredSuffixSkipsOnlyIrrelevantPrefixAndRetainsReverseSweep) {
  auto curve=line();auto body=bodyAt(curve);body.position=curve.evaluateDeBoorT(2.);
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_arc,.5,1e-6);
  EXPECT_LE(curve.evaluateDeBoorT(domain->checked_from_time).x(),.35+1e-6);
  const auto old_unknown=[](const Eigen::Vector3d& p,double){return p.x()<.2?2:0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,old_unknown,body,"map",10.,.4,5000,0.,2.));
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,old_unknown,body,"map",10.,.4,5000,0.,2.,{},domain->checked_from_time));
  for(int state:{-1,1,2}) {
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [state](const Eigen::Vector3d& p,double){return p.x()>.39&&p.x()<.41?state:0;},
      body,"map",10.,.4,5000,0.,2.,{},domain->checked_from_time));
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [state](const Eigen::Vector3d& p,double){return p.x()>.85?state:0;},
      body,"map",10.,.4,5000,0.,2.,{},domain->checked_from_time));
  }
}
TEST(TrajectoryCollision, FormalMeasuredHeadingUsesRemainingDomainWithoutRecheckingIrrelevantHistory) {
  auto curve=line();auto body=bodyAt(curve);body.position=curve.evaluateDeBoorT(2.);
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5);
  ASSERT_TRUE(domain);
  scan_planner::SplineHeadingContract heading;heading.measured_heading_enabled=true;
  const auto behind=[](const Eigen::Vector3d& p,double){return p.x()<.2?2:0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,behind,body,"map",10.,.4,
      5000,0.,2.,heading));
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,behind,body,"map",10.,.4,
      5000,0.,2.,heading,domain->checked_from_time));
  for(int state:{-1,1,2}) {
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,
        [state](const Eigen::Vector3d& p,double){return p.x()>.39&&p.x()<.41?state:0;},
        body,"map",10.,.4,5000,0.,2.,heading,domain->checked_from_time));
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,
        [state](const Eigen::Vector3d& p,double){return p.x()>.85?state:0;},
        body,"map",10.,.4,5000,0.,2.,heading,domain->checked_from_time));
  }
  // Rotation of the actual body into the original curve remains checked even
  // when the historical prefix has deliberately left the rolling map.
  body.orientation=Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [](const Eigen::Vector3d& p,double yaw){return p.x()>.49&&p.x()<.51&&yaw>.3&&yaw<1.?1:0;},
      body,"map",10.,.4,5000,0.,2.,heading,domain->checked_from_time));
}
TEST(TrajectoryCollision, MeasuredSuffixAllowsActualShortRetreatNotFloorOrRemoteJump) {
  auto curve=line();const auto retreated=curve.evaluateDeBoorT(1.6);
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,retreated,2.,.4,.5);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_arc,.4,1e-6);
  EXPECT_LE(curve.evaluateDeBoorT(domain->checked_from_time).x(),.25+1e-6);
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,retreated+Eigen::Vector3d(0,0,3.),2.,.4,.5));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,curve.evaluateDeBoorT(.8),2.,.4,.5));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,retreated,2.,.5,.5));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,retreated,2.,.4,.5,.3,.1));
}
TEST(TrajectoryCollision, CommittedTrackingRequiresEveryActualConnectorFootprintNotCandidateTolerance) {
  auto curve=line();auto body=bodyAt(curve);body.position=curve.evaluateDeBoorT(2.);
  body.position.y()=.06;
  using Policy=scan_planner::MeasuredConnectionPolicy;
  // Unchanged candidate admission cannot use execution's swept connection.
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5));
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5,.3,.15,Policy::CommittedSweptConnection);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_time,2.,1e-6);
  const auto free=[](const Eigen::Vector3d&,double){return 0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,free,body,"map",10.,.4,5000,0.,2.));
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,free,body,"map",10.,.4,5000,0.,2.,{},domain->checked_from_time,Policy::CommittedSweptConnection));
  // The complete original curve is free, but the actual lateral connector is
  // occupied, unknown or outside. None may be masked by the active identity.
  for(int state:{-1,1,2}) {
    const auto obstacle=[state](const Eigen::Vector3d& p,double){return p.y()>.02&&p.y()<.05?state:0;};
    EXPECT_TRUE(check(curve,obstacle));
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,obstacle,body,"map",10.,.4,5000,0.,2.,{},domain->checked_from_time,Policy::CommittedSweptConnection));
  }
  body.position.y()=.151;
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5,.3,.15,Policy::CommittedSweptConnection));
  body.position.y()=0.;body.position.z()+=3.;
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,body.position,2.,.02,.5,.3,.15,Policy::CommittedSweptConnection));
}

TEST(TrajectoryCollision, UnknownAndOutsideCannotProduceContinuityProof) {
  auto old=line();
  for(int state:{-1,1,2}) {
    EXPECT_FALSE(check(old,
        [state](const Eigen::Vector3d &p,double){return p.x()>.9?state:0;}));
  }
}

TEST(TrajectoryCollision, QueryBudgetNeverConvertsPartialCheckToSuccess) {
  auto old=line(); std::size_t queries=0;
  const auto free=[&](const Eigen::Vector3d &,double){++queries;return 0;};
  EXPECT_FALSE(check(old,free,20));
  EXPECT_LE(queries,20U);
  EXPECT_TRUE(check(old,free,5000));
  EXPECT_GT(queries,100U);
}

TEST(TrajectoryCollision, WallBudgetNeverConvertsPartialCheckToSuccess) {
  auto old=line(); std::size_t queries=0;
  EXPECT_FALSE(check(old,
      [&](const Eigen::Vector3d &,double){++queries;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));return 0;},5000,.001));
  EXPECT_LE(queries,1U);
}

TEST(TrajectoryCollision, MissingStaleFutureWrongFrameAndInvalidOrientationFailClosed) {
  auto curve=line();
  const auto free=[](const Eigen::Vector3d &,double){return 0;};
  const auto valid=bodyAt(curve);
  for (int failure=0;failure<8;++failure) {
    auto pose=valid;
    switch(failure) {
      case 0:pose.source_stamp=0.;break;
      case 1:pose.source_stamp=9.;break;
      case 2:pose.source_stamp=10.2;break;
      case 3:pose.frame="odom";break;
      case 4:pose.orientation.coeffs().setZero();break;
      case 5:pose.orientation.x()=std::numeric_limits<double>::quiet_NaN();break;
      case 6:pose.orientation=Eigen::AngleAxisd(std::acos(-1.)/2.,Eigen::Vector3d::UnitY());break;
      case 7:pose.position.x()+=.1;break;
    }
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,pose,"map",10.,.5))<<failure;
  }
}

TEST(TrajectoryCollision, CallbackSourceLeaseCannotBeRenewedByRepeatedOrDelayedReceipt) {
  using scan_planner::measuredPoseSourceAccepted;
  EXPECT_TRUE(measuredPoseSourceAccepted(10000000000LL,10010000000LL,.5,9999999999LL));
  EXPECT_FALSE(measuredPoseSourceAccepted(10000000000LL,10010000000LL,.5,10000000000LL));
  EXPECT_FALSE(measuredPoseSourceAccepted(9999999999LL,10010000000LL,.5,10000000000LL));
  EXPECT_FALSE(measuredPoseSourceAccepted(9000000000LL,10010000000LL,.5));
  EXPECT_FALSE(measuredPoseSourceAccepted(10200000000LL,10000000000LL,.5));
  EXPECT_FALSE(measuredPoseSourceAccepted(0LL,0LL,.5));
  EXPECT_FALSE(measuredPoseSourceAccepted(10000000000LL,10010000000LL,0.));
  EXPECT_FALSE(measuredPoseSourceAccepted(10000000000LL,10010000000LL,
                                         std::numeric_limits<double>::quiet_NaN()));
}

TEST(TrajectoryCollision, InitialMeasuredRotationSharesQueryBudgetAndRejectsEveryNonFreeState) {
  auto curve=line();const auto measured=bodyAt(curve,std::acos(-1.)/2.);
  for (int state:{-1,1,2}) {
    std::size_t intermediate=0;
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
        [&](const Eigen::Vector3d &,double yaw) {
          if(yaw>.35 && yaw<1.2){++intermediate;return state;}return 0;
        },measured,"map",10.,.5));
    EXPECT_GT(intermediate,0U);
  }
  std::size_t calls=0;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
      [&](const Eigen::Vector3d &,double){++calls;return 0;},measured,"map",10.,.5,210));
  EXPECT_LE(calls,210U);
}

TEST(TrajectoryCollision, ShortestSignedTurnCrossesPiNotTheOppositeArc) {
  auto curve=line();
  auto points=curve.getControlPoint();points.row(0)=-points.row(0);curve=scan_planner::UniformBspline(points,3,.2);
  const auto measured=bodyAt(curve,-std::acos(-1.)+.02);
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
      [](const Eigen::Vector3d &,double yaw){return std::abs(yaw)<2. ? 1:0;},measured,"map",10.,.5));
}

TEST(TrajectoryCollision, PredecessorUsesMeasuredCurrentTimeNotHistoricalStart) {
  auto curve=line();auto measured=bodyAt(curve,std::acos(-1.)/2.);
  measured.position=curve.evaluateDeBoorT(2.);
  const auto free=[](const Eigen::Vector3d &,double){return 0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,measured,"map",10.,.5));
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,measured,"map",10.,.5,5000,0.,2.));
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,measured,"map",10.,.5,5000,0.,5.));
}

TEST(TrajectoryCollision, StationaryCurveUsesMeasuredYawInsteadOfInventingZeroHeading) {
  Eigen::MatrixXd points(3,5);for(int i=0;i<5;++i)points.col(i)=Eigen::Vector3d(0.,0.,.55);
  scan_planner::UniformBspline curve(points,3,.2);const auto measured=bodyAt(curve,1.);
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
      [](const Eigen::Vector3d &,double yaw){return std::abs(yaw-1.)>.001 ? 1:0;},measured,"map",10.,.5));
}

TEST(TrajectoryCollision, MeasuredProgressKeepsOldCurveAfterTwentyCentimeterMove) {
  auto curve=line();auto measured=bodyAt(curve);
  measured.position=curve.evaluateDeBoorT(.8);  // 0.20 m, independent of wall time.
  const auto free=[](const Eigen::Vector3d &,double){return 0;};
  scan_planner::SplineHeadingContract heading;heading.preview_only_enabled=true;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,
      measured,"map",10.,.5,5000,0.,0.,heading));  // Previous production behavior.
  const auto progress=scan_planner::measuredPreviewCurveTime(curve,measured.position,0.,.08);
  ASSERT_TRUE(progress);
  EXPECT_NEAR(*progress,.8,.03);
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,free,
      measured,"map",10.,.5,5000,0.,*progress,heading));
}

TEST(TrajectoryCollision, PreviewProgressDoesNotSkipCollisionUnknownOrRotationProof) {
  auto curve=line();auto measured=bodyAt(curve,1.2);
  measured.position=curve.evaluateDeBoorT(.8);
  const auto progress=scan_planner::measuredPreviewCurveTime(curve,measured.position,0.,.08);
  ASSERT_TRUE(progress);
  scan_planner::SplineHeadingContract heading;heading.preview_only_enabled=true;
  for (int state:{-1,1,2}) {
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
        [state](const Eigen::Vector3d &p,double){return p.x()>.85?state:0;},
        measured,"map",10.,.5,5000,0.,*progress,heading));
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
        [state](const Eigen::Vector3d &p,double yaw){
          return p.x()>.18&&p.x()<.22&&yaw>.3&&yaw<1.?state:0;},
        measured,"map",10.,.5,5000,0.,*progress,heading));
  }
  measured.source_stamp=9.;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
      [](const Eigen::Vector3d &,double){return 0;},measured,"map",10.,.5,5000,0.,*progress,heading));
}

TEST(TrajectoryCollision, StationaryPreviewCannotAdvanceByRepeatedChecks) {
  auto curve=line();const auto body=curve.evaluateDeBoorT(0.);
  double progress=0.;
  for (int i=0;i<50;++i) {
    const auto next=scan_planner::measuredPreviewCurveTime(curve,body,progress,.08);
    ASSERT_TRUE(next); EXPECT_DOUBLE_EQ(*next,0.); progress=*next;
  }
}

TEST(TrajectoryCollision, MeasuredProgressCannotRewindTeleportOrJumpFloors) {
  auto curve=line();
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,curve.evaluateDeBoorT(.2),.8,.08));
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{.2,.3,.55},0.,.08));
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{.2,0.,3.55},0.,.08));
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{5.,0.,.55},0.,.08));
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{.2,0.,.55},0.,.08,2));
  const auto close=scan_planner::measuredPreviewCurveTime(curve,curve.evaluateDeBoorT(.78),.8,.08);
  ASSERT_TRUE(close);EXPECT_GE(*close,.8);  // Tiny reverse noise does not rewind progress.
}

TEST(TrajectoryCollision, SpatialCrossingDoesNotSelectDistantLaterBranch) {
  Eigen::MatrixXd points(3,83);
  for (int i=0;i<83;++i) points.col(i)=Eigen::Vector3d(
      i<=41 ? (i-1)*.05 : (81-i)*.05,0.,.55);
  scan_planner::UniformBspline curve(points,3,.2);
  const auto actual=curve.evaluateDeBoorT(.4);
  const auto progress=scan_planner::measuredPreviewCurveTime(curve,actual,0.,.08);
  ASSERT_TRUE(progress); EXPECT_LT(*progress,1.);
  EXPECT_NEAR(*progress,.4,.03);
}

static scan_planner::NoMotionPreviewProgress previewSeed(scan_planner::UniformBspline& curve,
    double time=0.,double arc=0.) {
  return {scan_planner::NoMotionPreviewStatus::Connected,time,arc,arc,0.,10.,curve.evaluateDeBoorT(time)};
}
TEST(NoMotionPreviewConnection, CommittedOffsetDoesNotRelaxStrictCandidateAdmission) {
  auto curve=line();auto pose=bodyAt(curve);pose.position.y()=.06;pose.source_stamp=10.1;
  const auto progress=scan_planner::projectCommittedNoMotionPreview(curve,pose.position,10.1,previewSeed(curve),.05);
  ASSERT_TRUE(progress.connected());EXPECT_NEAR(progress.residual,.06,1e-8);
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,pose.position,0.,.05));
  const auto free=[](const Eigen::Vector3d&,double){return 0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,free,pose,"map",10.1,.4));
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,free,pose,"map",10.1,.4,
    10000,0.,progress.curve_time,{},0.,scan_planner::MeasuredConnectionPolicy::CommittedNoMotionPreview));
}
TEST(NoMotionPreviewConnection, EveryActualConnectorAndWholeCurveStillRejectsBlockedStates) {
  auto curve=line();auto pose=bodyAt(curve);pose.position.y()=.10;pose.source_stamp=10.1;
  const auto p=scan_planner::projectCommittedNoMotionPreview(curve,pose.position,10.1,previewSeed(curve),.05);
  ASSERT_TRUE(p.connected());
  for(int state:{-1,1,2})for(int where:{0,1}) {
    const auto query=[=](const Eigen::Vector3d& point,double) {
      return (where==0?point.y()>.03&&point.y()<.07:point.x()>.85)?state:0;
    };
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.05,.49,query,pose,"map",10.1,.4,
      10000,0.,p.curve_time,{},0.,scan_planner::MeasuredConnectionPolicy::CommittedNoMotionPreview));
  }
}
TEST(NoMotionPreviewConnection, SmallReverseIsRealProgressWithoutReverseRatchet) {
  auto curve=line();auto seed=previewSeed(curve,2.,.5);
  auto p=scan_planner::projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(1.6),10.1,seed,.05);
  ASSERT_TRUE(p.connected());EXPECT_NEAR(p.arc_length,.4,1e-6);EXPECT_NEAR(p.curve_time,1.6,1e-6);
  EXPECT_DOUBLE_EQ(p.high_water_arc,.5);
  // The residual bound does not turn repeated reverse samples into an
  // arbitrary new route/floor domain: the high-water limit remains fixed.
  p=scan_planner::projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(1.4),10.2,p,.05);
  ASSERT_TRUE(p.connected());EXPECT_NEAR(p.arc_length,.35,1e-6);EXPECT_DOUBLE_EQ(p.high_water_arc,.5);
  auto lateral=curve.evaluateDeBoorT(1.4);lateral.y()=.06;
  EXPECT_TRUE(scan_planner::projectCommittedNoMotionPreview(curve,lateral,10.3,p,.05).connected());
  // A 15 cm connector cannot add another 15 cm behind the reverse boundary.
  const auto farther=scan_planner::projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(1.1),10.3,p,.05);
  EXPECT_EQ(farther.status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
  const auto distant=scan_planner::projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(.2),10.3,p,.05);
  EXPECT_EQ(distant.status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
}
TEST(NoMotionPreviewConnection, ArcStartConnectorUsesOnlyUnusedReverseAllowance) {
  auto curve=line();const auto seed=previewSeed(curve,.4,.1);
  EXPECT_TRUE(scan_planner::projectCommittedNoMotionPreview(curve,{-.04,0.,.55},10.1,seed,.05).connected());
  EXPECT_EQ(scan_planner::projectCommittedNoMotionPreview(curve,{-.06,0.,.55},10.1,seed,.05).status,
    scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
}
TEST(NoMotionPreviewConnection, StationaryChecksDoNotAdvanceOrRenewAcquiredTime) {
  auto curve=line();auto seed=previewSeed(curve);
  for(int i=0;i<30;++i) {
    const auto p=scan_planner::projectCommittedNoMotionPreview(curve,seed.measured_position,seed.source_stamp,seed,.05);
    ASSERT_TRUE(p.connected());EXPECT_DOUBLE_EQ(p.curve_time,0.);EXPECT_DOUBLE_EQ(p.source_stamp,10.);seed=p;
  }
  const auto changed=scan_planner::projectCommittedNoMotionPreview(curve,{.05,0.,.55},10.,seed,.05);
  EXPECT_EQ(changed.status,scan_planner::NoMotionPreviewStatus::Uncertified);
}
TEST(NoMotionPreviewConnection, IndependentObservationAcceptanceNeverBecomesCertificateOrReverseRatchet) {
  using namespace scan_planner;
  auto curve=line();auto prior=previewSeed(curve);
  auto next=projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(.3),10.1,prior,.05);
  ASSERT_TRUE(next.connected());EXPECT_TRUE(advanceNoMotionPreviewObservation(prior,next));
  EXPECT_FALSE(advanceNoMotionPreviewObservation(next,next));
  for(int mode=0;mode<8;++mode) {
    auto invalid=next;
    if(mode==0)invalid.source_stamp=prior.source_stamp;
    if(mode==1)invalid.source_stamp=prior.source_stamp-.01;
    if(mode==2)invalid.status=NoMotionPreviewStatus::ReplanFromMeasuredBody;
    if(mode==3)invalid.status=NoMotionPreviewStatus::BudgetExhausted;
    if(mode==4)invalid.high_water_arc=prior.high_water_arc-.01;
    if(mode==5)invalid.arc_length=invalid.high_water_arc-.16;
    if(mode==6)invalid.residual=.151;
    if(mode==7)invalid.source_stamp=prior.source_stamp+.401;
    EXPECT_FALSE(advanceNoMotionPreviewObservation(prior,invalid))<<mode;
  }
}
TEST(NoMotionPreviewConnection, RemoteFloorOrConnectorRequiresMeasuredReseedNotSafetyClaim) {
  auto curve=line();const auto seed=previewSeed(curve);
  for(const Eigen::Vector3d& body:std::vector<Eigen::Vector3d>{{0.,.151,.55},{0.,0.,3.55},{5.,0.,.55}}) {
    const auto p=scan_planner::projectCommittedNoMotionPreview(curve,body,10.1,seed,.05);
    EXPECT_EQ(p.status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
    EXPECT_FALSE(p.connected());
  }
  EXPECT_EQ(scan_planner::projectCommittedNoMotionPreview(curve,seed.measured_position,10.5,seed,.05).status,
    scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
}
TEST(NoMotionPreviewConnection, InvalidOrExhaustedSearchIsNotAnOutOfRangeCertificate) {
  auto curve=line();const auto seed=previewSeed(curve);const auto body=curve.evaluateDeBoorT(.8);
  EXPECT_EQ(scan_planner::projectCommittedNoMotionPreview(curve,body,10.1,seed,.05,2).status,
    scan_planner::NoMotionPreviewStatus::BudgetExhausted);
  EXPECT_EQ(scan_planner::projectCommittedNoMotionPreview(curve,body,9.9,seed,.05).status,
    scan_planner::NoMotionPreviewStatus::Uncertified);
  EXPECT_EQ(scan_planner::projectCommittedNoMotionPreview(curve,body,10.1,seed,0.).status,
    scan_planner::NoMotionPreviewStatus::Uncertified);
}
TEST(NoMotionPreviewConnection, AbsoluteDeadlineDoesNotBecomeOutOfRangeOrRenewTheSeed) {
  auto curve=line();const auto seed=previewSeed(curve);
  const auto p=scan_planner::projectCommittedNoMotionPreview(curve,seed.measured_position,10.1,
      seed,.05,10000,.005,std::chrono::steady_clock::now());
  EXPECT_EQ(p.status,scan_planner::NoMotionPreviewStatus::BudgetExhausted);
  EXPECT_FALSE(p.connected());EXPECT_DOUBLE_EQ(seed.source_stamp,10.);
}
TEST(TrajectoryCollision, CompletionDistinguishesPartialBudgetFromWitnessedCollisionAndInvalidSource) {
  using C=scan_planner::CurveCheckCompletion;
  auto curve=line();auto body=bodyAt(curve);C completion=C::Clear;
  const auto checkCompletion=[&](auto query,std::size_t budget,double wall=0.) {
    return scan_planner::wholeCurveCollisionFree(curve,.05,.49,query,body,"map",10.,.4,
        budget,wall,0.,{},0.,scan_planner::MeasuredConnectionPolicy::CandidateAdmission,.15,&completion);
  };
  EXPECT_FALSE(checkCompletion([](const auto&,double){return 0;},2));
  EXPECT_EQ(completion,C::BudgetExhausted);
  EXPECT_FALSE(checkCompletion([](const auto&,double){return 1;},200000));
  EXPECT_EQ(completion,C::Blocked);
  EXPECT_FALSE(checkCompletion([](const auto&,double){return 2;},200000));
  EXPECT_EQ(completion,C::Uncertified);
  EXPECT_FALSE(checkCompletion([](const auto&,double){return 0;},200000,1e-12));
  EXPECT_EQ(completion,C::BudgetExhausted);
  body.source_stamp=9.;
  EXPECT_FALSE(checkCompletion([](const auto&,double){return 0;},200000,1e-12));
  EXPECT_EQ(completion,C::InvalidInput); // A deadline cannot hide a stale source.
  body=bodyAt(curve);EXPECT_TRUE(checkCompletion([](const auto&,double){return 0;},200000));
  EXPECT_EQ(completion,C::Clear);
}
TEST(TrajectoryCollision, ProjectionResourceCompletionIsNotAConnectionFailureWitness) {
  using C=scan_planner::CurveCheckCompletion;auto curve=line();C completion=C::Clear;
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,curve.evaluateDeBoorT(.8),0.,.05,2,0.,&completion));
  EXPECT_EQ(completion,C::BudgetExhausted);
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{.2,1.,.55},0.,.05,10000,0.,&completion));
  EXPECT_EQ(completion,C::Uncertified);
  EXPECT_FALSE(scan_planner::measuredPreviewCurveTime(curve,{0.,0.,.55},0.,0.,10000,0.,&completion));
  EXPECT_EQ(completion,C::InvalidInput);
}
TEST(NoMotionPreviewConnection, CrossingKeepsLocalArcAndCannotSelectDistantReturnBranch) {
  Eigen::MatrixXd points(3,83);
  for(int i=0;i<83;++i)points.col(i)=Eigen::Vector3d(i<=41?(i-1)*.05:(81-i)*.05,0.,.55);
  scan_planner::UniformBspline curve(points,3,.2);const auto seed=previewSeed(curve);
  const auto p=scan_planner::projectCommittedNoMotionPreview(curve,curve.evaluateDeBoorT(.4),10.1,seed,.05);
  ASSERT_TRUE(p.connected());EXPECT_NEAR(p.curve_time,.4,1e-6);EXPECT_LT(p.arc_length,.2);
}
