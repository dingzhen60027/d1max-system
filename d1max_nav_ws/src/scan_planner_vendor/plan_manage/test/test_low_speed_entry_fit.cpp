// Pure candidate-boundary math and native measured adoption: no ROS init,
// stationary evidence, writer, motion authority or simulation participant.
#include <gtest/gtest.h>
#include <plan_manage/low_speed_entry_fit.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <limits>

namespace {
using namespace scan_planner;
const Eigen::Vector3d raw_velocity{-.0001,.00002,.007};
EntryDerivativeFit fit(bool enabled=true,bool guided=true,std::optional<double> floor=0.,
    std::int64_t source=1791348500000000001LL,
    const Eigen::Vector3d& velocity=raw_velocity,const Eigen::Vector3d& acceleration=Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& measured=raw_velocity) {
  return fitLowSpeedEntryDerivatives(enabled,guided,floor,source,velocity,acceleration,measured);
}
UniformBspline fittedCurve() {
  const Eigen::Vector3d start{-8.,-5.,.481},goal{-7.4,-5.,.52};
  std::vector<Eigen::Vector3d> samples;
  for(int i=0;i<=10;++i)samples.push_back(start+(goal-start)*(i/10.));
  const auto chosen=fit();
  const CubicMotionBoundary boundary{start,chosen.solver_velocity,chosen.solver_acceleration,
      goal,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  return UniformBspline(fitCubicWithFixedBoundary(samples,2.,boundary),3,2.);
}
struct Adoption {
  UniformBspline curve=fittedCurve();
  MeasuredBodyPose body{{-8.,-5.,.481},Eigen::Quaterniond::Identity(),10.,"odom",10000000000LL};
  SolveBudget::Ptr budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
};
}

TEST(LowSpeedEntryFit, CandidateDerivativeChoiceDoesNotModifyOriginalXYZBoundaryInputs) {
  const Eigen::Vector3d v=raw_velocity,a=Eigen::Vector3d::Zero();
  const auto result=fit(true,true,0.,10000000000LL,v,a,v);
  ASSERT_TRUE(result.low_speed_fitted);
  EXPECT_EQ(result.solver_velocity,Eigen::Vector3d::Zero());
  EXPECT_EQ(result.solver_acceleration,Eigen::Vector3d::Zero());
  EXPECT_EQ(v,raw_velocity);EXPECT_EQ(a,Eigen::Vector3d::Zero());
}

TEST(LowSpeedEntryFit, DefaultsAndUncertifiedNonGuidedFloorPreserveOriginalBoundary) {
  for(const auto& result:{fit(false),fit(true,false),fit(true,true,std::nullopt),
      fit(true,true,std::numeric_limits<double>::quiet_NaN()),
      fit(true,true,std::numeric_limits<double>::infinity())}) {
    EXPECT_FALSE(result.low_speed_fitted);EXPECT_EQ(result.solver_velocity,raw_velocity);
  }
}

TEST(LowSpeedEntryFit, OriginalMeasurementSourceIsRequired) {
  for(const auto source:{0LL,-1LL}) {
    const auto result=fit(true,true,0.,source);EXPECT_FALSE(result.low_speed_fitted);
    EXPECT_EQ(result.solver_velocity,raw_velocity);
  }
}

TEST(LowSpeedEntryFit, UsesFullXYZNormForBothIncomingAndOriginalMeasuredVelocity) {
  const Eigen::Vector3d over{.02,.02,.02};
  for(bool high_start:{false,true}) {
    const auto result=fit(true,true,0.,10000000000LL,high_start?over:raw_velocity,
        Eigen::Vector3d::Zero(),high_start?raw_velocity:over);
    EXPECT_FALSE(result.low_speed_fitted);
    EXPECT_EQ(result.solver_velocity,high_start?over:raw_velocity);
  }
  EXPECT_FALSE(fit(true,true,0.,10000000000LL,raw_velocity,Eigen::Vector3d::Zero(),{0.,0.,.030001}).low_speed_fitted);
}

TEST(LowSpeedEntryFit, ExistingLowSpeedBoundaryHasNoNewToleranceOrMinimumSpeed) {
  const Eigen::Vector3d at{.03,0.,0.},above{std::nextafter(.03,1.),0.,0.};
  EXPECT_TRUE(fit(true,true,0.,10000000000LL,at,Eigen::Vector3d::Zero(),at).low_speed_fitted);
  EXPECT_FALSE(fit(true,true,0.,10000000000LL,above,Eigen::Vector3d::Zero(),at).low_speed_fitted);
  EXPECT_FALSE(fit(true,true,0.,10000000000LL,at,Eigen::Vector3d::Zero(),above).low_speed_fitted);
  EXPECT_TRUE(fit(true,true,0.,10000000000LL,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()).low_speed_fitted);
}

TEST(LowSpeedEntryFit, EveryNonzeroAccelerationBoundaryRetainsItsOldDerivativeEvenWhenTiny) {
  for(const auto& acceleration:{Eigen::Vector3d(.001,0.,0.),Eigen::Vector3d(0.,0.,1e-300)}) {
    const auto result=fit(true,true,0.,10000000000LL,raw_velocity,acceleration);
    EXPECT_FALSE(result.low_speed_fitted);EXPECT_EQ(result.solver_velocity,raw_velocity);
    EXPECT_EQ(result.solver_acceleration,acceleration);
  }
}

TEST(LowSpeedEntryFit, NonfiniteInputsAreNeverConvertedToPlausibleZeroMeasurements) {
  const double bad=std::numeric_limits<double>::quiet_NaN();
  for(int field=0;field<3;++field) {
    auto v=raw_velocity,a=Eigen::Vector3d::Zero().eval(),measured=raw_velocity;
    if(field==0)v.z()=bad;
    if(field==1)a.z()=bad;
    if(field==2)measured.z()=bad;
    EXPECT_FALSE(fit(true,true,0.,10000000000LL,v,a,measured).low_speed_fitted);
  }
}

TEST(LowSpeedEntryFit, FullXYZFixedBoundaryKeepsStartAndFutureZAndPublishesOriginalRawJoin) {
  Adoption f;auto v=f.curve.getDerivative();
  EXPECT_LT(v.evaluateDeBoorT(0.).norm(),1e-12);
  EXPECT_NEAR(f.curve.evaluateDeBoorT(0.).z(),.481,1e-14);
  EXPECT_NEAR(f.curve.evaluateDeBoorT(f.curve.getTimeSum()).z(),.52,1e-14);
  const auto join=measuredCandidateJoin(f.curve,f.body,f.body,raw_velocity,.05,.15,.35,true,f.budget);
  ASSERT_TRUE(join);EXPECT_EQ(join->measured.source_stamp_ns,f.body.source_stamp_ns);
  EXPECT_EQ(join->velocity,raw_velocity);EXPECT_GT(join->velocity.z(),0.);
  EXPECT_EQ(join->measured.position,f.body.position);EXPECT_FALSE(join->acceleration_valid);
  EXPECT_EQ(join->acceleration,Eigen::Vector3d::Zero());
}

TEST(LowSpeedEntryFit, NativeAdoptionStillRejectsLatestVelocityOutsideOriginalC1Tolerance) {
  Adoption f;
  EXPECT_FALSE(measuredCandidateJoin(f.curve,f.body,f.body,{0.,0.,.051},.05,.15,.35,true,f.budget));
  const auto join=measuredCandidateJoin(f.curve,f.body,f.body,raw_velocity,.05,.15,.35,true,f.budget);
  ASSERT_TRUE(join);EXPECT_EQ(join->velocity,raw_velocity);
}

TEST(LowSpeedEntryFit, ChoosingZeroBoundaryNeverBypassesWholeCurveCollisionOrFinalSourceGate) {
  for(bool collision_clear:{false,true}) {
    Adoption f;int checks=0;bool changed=false;
    auto read=[&]{return CandidateSourceLease{1,7,1,10.,changed?10000000001LL:10000000000LL};};
    const auto result=certifyCandidateAdoption(f.curve,f.body,f.body,raw_velocity,.05,.15,.35,true,7,f.budget,
        read,[]{return std::make_pair(true,true);},[&](double){++checks;changed=true;return collision_clear;});
    EXPECT_FALSE(result);EXPECT_EQ(checks,1); // Obstacle or original-source change denies adoption.
  }
}
