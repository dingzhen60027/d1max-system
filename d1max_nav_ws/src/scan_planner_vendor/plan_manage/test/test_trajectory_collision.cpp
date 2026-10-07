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

TEST(TrajectoryCollision, OriginalUnixNanosecondsSurviveBodyCopyAndFreshnessBoundaries) {
  constexpr std::int64_t ns=1791124691902856036LL;
  auto curve=line();auto pose=bodyAt(curve);
  pose.source_stamp=static_cast<double>(ns)*1e-9;pose.source_stamp_ns=ns;
  const auto copy=pose;EXPECT_EQ(scan_planner::measuredBodySourceNs(copy),ns);
  double yaw=0.;
  EXPECT_TRUE(scan_planner::measuredBodyYaw(copy,"map",pose.source_stamp,.4,yaw,ns));
  EXPECT_TRUE(scan_planner::measuredBodyYaw(copy,"map",pose.source_stamp+.4,.4,yaw,ns+400000000LL));
  EXPECT_FALSE(scan_planner::measuredBodyYaw(copy,"map",pose.source_stamp+.4,.4,yaw,ns+400000001LL));
  EXPECT_TRUE(scan_planner::measuredBodyYaw(copy,"map",pose.source_stamp-.1,.4,yaw,ns-100000000LL));
  EXPECT_FALSE(scan_planner::measuredBodyYaw(copy,"map",pose.source_stamp-.1,.4,yaw,ns-100000001LL));
  EXPECT_TRUE(scan_planner::measuredPoseSourceAccepted(ns+1,ns+1,.4,ns));
  EXPECT_FALSE(scan_planner::measuredPoseSourceAccepted(ns,ns+1,.4,ns));
}

TEST(TrajectoryCollision, WholeCurveProofDoesNotRenewOriginalUnixBodyFreshness) {
  constexpr std::int64_t ns=1791124691902856036LL;
  auto curve=line();auto pose=bodyAt(curve);
  pose.source_stamp=static_cast<double>(ns)*1e-9;pose.source_stamp_ns=ns;
  const auto check_at=[&](std::int64_t now) {
    return scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [](const Eigen::Vector3d&,double){return 0;},pose,"map",static_cast<double>(now)*1e-9,.4,
      200000,0.,0.,{},0.,scan_planner::MeasuredConnectionPolicy::CandidateAdmission,now);
  };
  EXPECT_TRUE(check_at(ns+400000000LL));
  EXPECT_FALSE(check_at(ns+400000001LL));
  EXPECT_EQ(pose.source_stamp_ns,ns);
}

template<class Query> static bool check(scan_planner::UniformBspline &curve, Query query,
    std::size_t budget=200000,double wall_budget=0.,scan_planner::CurveCheckTrace* trace=nullptr) {
  return scan_planner::wholeCurveCollisionFree(curve,.08,.49,query,bodyAt(curve),"map",10.,.5,
      budget,wall_budget,0.,{},0.,scan_planner::MeasuredConnectionPolicy::CandidateAdmission,0,trace);
}

TEST(TrajectoryCollision, TraceCapturesOriginalFirstNonFreeActualBodyQuery) {
  auto curve=line();const auto measured=bodyAt(curve,.7);
  for(int state:{-1,1,2}) {
    scan_planner::CurveCheckTrace trace;std::size_t queries=0;
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
      [&](const Eigen::Vector3d&,double){++queries;return state;},
      measured,"map",10.,.5,200000,0.,0.,{},0.,
      scan_planner::MeasuredConnectionPolicy::CandidateAdmission,0,&trace));
    ASSERT_TRUE(trace.first_non_free);
    EXPECT_EQ(queries,1U);EXPECT_EQ(trace.first_non_free->query_index,1U);
    EXPECT_EQ(trace.first_non_free->state,state);
    EXPECT_TRUE(trace.first_non_free->position.isApprox(measured.position,1e-12));
    EXPECT_NEAR(trace.first_non_free->yaw,.7,1e-12);
  }
}

TEST(TrajectoryCollision, TracePreservesQuerySequenceAndCapturesLateUnknown) {
  auto curve=line();scan_planner::CurveCheckTrace trace;
  std::vector<scan_planner::CurveQueryWitness> original,traced;
  const auto query=[&](std::vector<scan_planner::CurveQueryWitness>& seen) {
    return [&](const Eigen::Vector3d& p,double yaw) {
      const int state=p.x()>.85?2:0;
      seen.push_back({p,yaw,state,seen.size()+1});return state;
    };
  };
  EXPECT_FALSE(check(curve,query(original)));
  EXPECT_FALSE(check(curve,query(traced),200000,0.,&trace));
  ASSERT_TRUE(trace.first_non_free);ASSERT_GT(original.size(),1U);
  ASSERT_EQ(traced.size(),original.size());
  for(std::size_t i=0;i<original.size();++i) {
    EXPECT_TRUE(traced[i].position.isApprox(original[i].position,1e-12));
    EXPECT_DOUBLE_EQ(traced[i].yaw,original[i].yaw);EXPECT_EQ(traced[i].state,original[i].state);
  }
  EXPECT_EQ(trace.first_non_free->state,2);
  EXPECT_EQ(trace.first_non_free->query_index,original.size());
  EXPECT_TRUE(trace.first_non_free->position.isApprox(original.back().position,1e-12));
  EXPECT_DOUBLE_EQ(trace.first_non_free->yaw,original.back().yaw);
}

TEST(TrajectoryCollision, TraceCapturesPreviewRotationUnknownWithoutSkippingSweep) {
  auto curve=line();const auto measured=bodyAt(curve,1.2);
  scan_planner::SplineHeadingContract heading;heading.preview_only_enabled=true;
  scan_planner::CurveCheckTrace trace;std::size_t queries=0;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,
    [&](const Eigen::Vector3d&,double yaw){++queries;return yaw>.3&&yaw<1.?2:0;},
    measured,"map",10.,.5,200000,0.,0.,heading,0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,0,&trace));
  ASSERT_TRUE(trace.first_non_free);
  EXPECT_GT(queries,1U);EXPECT_EQ(trace.first_non_free->query_index,queries);
  EXPECT_EQ(trace.first_non_free->state,2);
  EXPECT_TRUE(trace.first_non_free->position.isApprox(measured.position,1e-12));
  EXPECT_GT(trace.first_non_free->yaw,.3);EXPECT_LT(trace.first_non_free->yaw,1.);
}

TEST(TrajectoryCollision, TraceCannotInventWitnessForClearBudgetOrStaleSource) {
  auto curve=line();scan_planner::CurveCheckTrace trace;
  const auto unknown=[](const Eigen::Vector3d&,double){return 2;};
  const auto free=[](const Eigen::Vector3d&,double){return 0;};
  EXPECT_FALSE(check(curve,unknown,200000,0.,&trace));ASSERT_TRUE(trace.first_non_free);
  EXPECT_TRUE(check(curve,free,200000,0.,&trace));EXPECT_FALSE(trace.first_non_free);
  EXPECT_FALSE(check(curve,unknown,200000,0.,&trace));ASSERT_TRUE(trace.first_non_free);
  std::size_t queries=0;
  const auto counted_free=[&](const Eigen::Vector3d&,double){++queries;return 0;};
  EXPECT_FALSE(check(curve,counted_free,20,0.,&trace));
  EXPECT_EQ(queries,0U);EXPECT_FALSE(trace.first_non_free);
  EXPECT_FALSE(check(curve,unknown,200000,0.,&trace));ASSERT_TRUE(trace.first_non_free);
  auto stale=bodyAt(curve);stale.source_stamp=9.;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(curve,.08,.49,counted_free,
    stale,"map",10.,.5,200000,0.,0.,{},0.,
    scan_planner::MeasuredConnectionPolicy::CandidateAdmission,0,&trace));
  EXPECT_EQ(queries,0U);EXPECT_FALSE(trace.first_non_free);
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
TEST(TrajectoryCollision, ExplicitIsolatedPlantBoundDoesNotChangeDefaultOrReverseSweep) {
  auto curve=line();const auto advanced=curve.evaluateDeBoorT(2.8);
  using Speed=scan_planner::MeasuredSpeedProfile;
  using Connection=scan_planner::MeasuredConnectionPolicy;
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.6));
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.6,.15,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_arc,.7,1e-6);
  EXPECT_LE(curve.evaluateDeBoorT(domain->checked_from_time).x(),.55+1e-6);
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.6500001,.15,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.5,.5,.6,.15,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.6,.1,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot));
}
TEST(TrajectoryCollision, IsolatedProjectionAcceptsExactCalibratedCapWithoutChangingDefault) {
  auto curve=line();const auto advanced=curve.evaluateDeBoorT(2.8);
  using Speed=scan_planner::MeasuredSpeedProfile;
  using Connection=scan_planner::MeasuredConnectionPolicy;
  double residual=std::numeric_limits<double>::quiet_NaN();
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.65,.15,
      Connection::CandidateAdmission,&residual,Speed::IsolatedOfficialSpot);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_arc,.7,1e-6);
  EXPECT_NEAR(residual,0.,1e-12);
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,
      std::nextafter(.65,1.),.15,Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot));
  // The unmarked production profile retains its exact .3 maximum.
  const auto nearby=curve.evaluateDeBoorT(2.3);
  EXPECT_TRUE(scan_planner::measuredRemainingCurveDomain(curve,nearby,2.,.4,.5,.3));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,nearby,2.,.4,.5,std::nextafter(.3,1.)));
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,nearby,2.,.4,.5,.65));
}
TEST(TrajectoryCollision, OldProjectionRecordRetainsItsOwnTravelDomainAfterCalibrationCapChange) {
  auto curve=line();const auto advanced=curve.evaluateDeBoorT(3.08);
  using Speed=scan_planner::MeasuredSpeedProfile;
  using Connection=scan_planner::MeasuredConnectionPolicy;
  // Measured motion is .270m. The unchanged search budgets are .2525m for
  // the old .6 record and .2725m for the new .65 record, with the original
  // .0125m candidate residual bound. The old record cannot borrow new reach.
  EXPECT_FALSE(scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.6,.15,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot));
  const auto domain=scan_planner::measuredRemainingCurveDomain(curve,advanced,2.,.4,.5,.65,.15,
      Connection::CandidateAdmission,nullptr,Speed::IsolatedOfficialSpot);
  ASSERT_TRUE(domain);EXPECT_NEAR(domain->measured_arc,.77,1e-6);
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
