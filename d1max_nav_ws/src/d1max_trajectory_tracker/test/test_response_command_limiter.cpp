#include <gtest/gtest.h>
#include "d1max_trajectory_tracker/response_command_limiter.hpp"

using namespace d1max_trajectory_tracker;
namespace {
CommandResponseCurve forwardCurve() { return {.30,.31328213472822414,.40,.10}; }
CommandResponseCurve yawCurve() { return {.50,.30,.40,.10}; }
constexpr double kDt=.05,kRate=1.6,kAcceleration=12.;
}

TEST(CommandResponseCurve, ExplicitDisabledCurveDoesNotChangeSiContract) {
  CommandResponseCurve curve;EXPECT_NO_THROW(curve.validate());
  EXPECT_FALSE(curve.enabled());EXPECT_DOUBLE_EQ(curve.map(.12),.12);
  EXPECT_DOUBLE_EQ(curve.inverse(-.15),-.15);
  ResponseCommandLimiter limiter;
  const auto result=limiter.step(.20,.10,.08,.12,curve,0.,0.,kDt);
  EXPECT_DOUBLE_EQ(result.value,.12);EXPECT_TRUE(result.limited);
  EXPECT_FALSE(result.priority_override);EXPECT_DOUBLE_EQ(result.mapped_rate,0.);
}

TEST(CommandResponseCurve, SignedDenseRoundTripIsMonotoneAndExactAtZero) {
  for (const auto curve : {forwardCurve(),yawCurve()}) {
    double previous=-curve.maximum_fraction;
    for (int i=-2000;i<=2000;++i) {
      const double demand=curve.reference*i/2000.;
      const double mapped=curve.map(demand);
      EXPECT_GE(mapped,previous-1e-14);
      EXPECT_NEAR(curve.inverse(mapped),demand,2e-10);
      EXPECT_NEAR(curve.map(-demand),-mapped,1e-14);previous=mapped;
    }
    EXPECT_DOUBLE_EQ(curve.map(0.),0.);EXPECT_DOUBLE_EQ(curve.inverse(0.),0.);
    EXPECT_DOUBLE_EQ(curve.map(curve.reference*2.),curve.maximum_fraction);
    EXPECT_DOUBLE_EQ(curve.inverse(-curve.maximum_fraction*2.),-curve.reference);
  }
}

TEST(CommandResponseCurve, RejectsInvalidConfigurationAndNonfiniteInput) {
  for (double invalid : {-1.,double(NAN),double(INFINITY)}) {
    auto curve=forwardCurve();curve.reference=invalid;
    EXPECT_THROW(curve.validate(),std::invalid_argument);
  }
  for (double invalid : {0.,1.,double(NAN)}) {
    auto curve=forwardCurve();curve.blend_ratio=invalid;
    EXPECT_THROW(curve.validate(),std::invalid_argument);
  }
  auto curve=forwardCurve();curve.effective_fraction=curve.maximum_fraction+.01;
  EXPECT_THROW(curve.validate(),std::invalid_argument);
  EXPECT_THROW(forwardCurve().map(NAN),std::invalid_argument);
  EXPECT_THROW(forwardCurve().inverse(INFINITY),std::invalid_argument);
}

TEST(CommandResponseCurve, ConstantWorkingFractionHasTheLowestPlateauPreimage) {
  const CommandResponseCurve curve{.50,.20,.20,.10};EXPECT_NO_THROW(curve.validate());
  EXPECT_DOUBLE_EQ(curve.inverse(.20),.05);EXPECT_DOUBLE_EQ(curve.inverse(-.20),-.05);
  EXPECT_DOUBLE_EQ(curve.inverse(.40),.05);
  for(int i=-1000;i<=1000;++i) {
    const double demand=.5*i/1000.,mapped=curve.map(demand);
    EXPECT_NEAR(curve.map(curve.inverse(mapped)),mapped,1e-13);
    if(std::abs(demand)>=.05) { EXPECT_NEAR(std::abs(mapped),.20,1e-14); }
  }
}

TEST(ResponseCommandLimiter, PlateauNeverReplacesAnAlreadyReachableSiTargetWithItsOtherPreimage) {
  const CommandResponseCurve curve{.50,.20,.20,.10};ResponseCommandLimiter limiter;
  for(double sign:{-1.,1.}) {
    limiter.reset();
    const auto out=limiter.step(sign*.25,sign*.20,sign>0?.16:-.24,sign>0?.24:-.16,
      curve,kRate,kAcceleration,kDt);
    EXPECT_DOUBLE_EQ(out.value,sign*.24);EXPECT_DOUBLE_EQ(out.mapped_fraction,sign*.20);
    EXPECT_DOUBLE_EQ(out.mapped_rate,0.);EXPECT_FALSE(out.priority_override);
  }
}

TEST(ResponseCommandLimiter, OneConditionerBoundsActualMappedDifferencesAtStartupAndSignedTurn) {
  for (const auto curve : {forwardCurve(),yawCurve()}) {
    ResponseCommandLimiter limiter;double previous=0.,previous_fraction=0.,previous_rate=0.;
    unsigned overrides=0;
    for (int i=1;i<=120;++i) {
      const double target=(i<=40?1.:(i<=80?-1.:.5))*curve.reference*.6;
      const auto result=limiter.step(target,previous,-curve.reference,curve.reference,
        curve,kRate,kAcceleration,kDt);
      EXPECT_GE(result.value,-curve.reference);EXPECT_LE(result.value,curve.reference);
      const double rate=(curve.map(result.value)-previous_fraction)/kDt;
      if (!result.priority_override) {
        EXPECT_LE(std::abs(rate),kRate+1e-10);
        EXPECT_LE(std::abs(rate-previous_rate),kAcceleration*kDt+1e-9);
      } else ++overrides;
      EXPECT_NEAR(result.mapped_fraction,curve.map(result.value),1e-14);
      EXPECT_NEAR(result.mapped_rate,rate,1e-12);
      previous=result.value;previous_fraction=result.mapped_fraction;previous_rate=rate;
    }
    EXPECT_EQ(overrides,0u);
  }
}

TEST(ResponseCommandLimiter, SameTaskRefreshDoesNotRequireAResetOrASecondSdkFilter) {
  const auto curve=forwardCurve();ResponseCommandLimiter unchanged,refreshed;
  double a=0.,b=0.;
  for (int i=1;i<=80;++i) {
    // There is deliberately no curve identity in the conditioner: ordinary
    // curve replacement cannot reset its rate history or restart an actuator.
    const double target=i<35?.20:.12;
    const auto expected=unchanged.step(target,a,0.,curve.reference,curve,kRate,kAcceleration,kDt);
    const auto actual=refreshed.step(target,b,0.,curve.reference,curve,kRate,kAcceleration,kDt);
    EXPECT_DOUBLE_EQ(actual.value,expected.value);
    EXPECT_DOUBLE_EQ(actual.mapped_rate,expected.mapped_rate);
    EXPECT_EQ(actual.priority_override,expected.priority_override);
    a=expected.value;b=actual.value;
  }
}

TEST(ResponseCommandLimiter, StationaryTargetsConvergeWithoutAnOutputDomainLimitCycle) {
  for (const auto curve : {forwardCurve(),yawCurve()}) {
    for (double sign : {-1.,1.}) {
      ResponseCommandLimiter limiter;double previous=0.,previous_fraction=0.,previous_rate=0.;
      const double target=sign*curve.reference*.6,target_fraction=curve.map(target);
      double last_error=std::abs(target_fraction);
      for (int i=0;i<120;++i) {
        const auto out=limiter.step(target,previous,-curve.reference,curve.reference,
          curve,kRate,kAcceleration,kDt);
        EXPECT_FALSE(out.priority_override);
        EXPECT_GE(sign*out.mapped_fraction,-1e-12);
        EXPECT_LE(sign*out.mapped_fraction,sign*target_fraction+1e-12);
        const double error=std::abs(target_fraction-out.mapped_fraction);
        EXPECT_LE(error,last_error+1e-12);
        EXPECT_LE(std::abs(out.mapped_rate),kRate+1e-10);
        EXPECT_LE(std::abs(out.mapped_rate-previous_rate),kAcceleration*kDt+1e-8);
        EXPECT_NEAR(out.mapped_rate,(out.mapped_fraction-previous_fraction)/kDt,1e-12);
        previous=out.value;previous_fraction=out.mapped_fraction;
        previous_rate=out.mapped_rate;last_error=error;
      }
      EXPECT_NEAR(previous,target,1e-10);EXPECT_NEAR(previous_rate,0.,1e-9);
    }
  }
}

TEST(ResponseCommandLimiter, OrdinaryReleaseBrakesContinuouslyButAnExplicitResetStartsCleanly) {
  const auto curve=forwardCurve();ResponseCommandLimiter limiter;double previous=0.,previous_rate=0.;
  for (int i=0;i<80;++i) {
    const auto out=limiter.step(.20,previous,0.,curve.reference,curve,kRate,kAcceleration,kDt);
    previous=out.value;previous_rate=out.mapped_rate;
  }
  ASSERT_GT(curve.map(previous),.30);
  double prior_fraction=curve.map(previous);
  for (int i=0;i<80;++i) {
    const auto out=limiter.step(0.,previous,0.,curve.reference,curve,kRate,kAcceleration,kDt);
    EXPECT_FALSE(out.priority_override);EXPECT_LE(out.mapped_fraction,prior_fraction+1e-12);
    EXPECT_GE(out.mapped_fraction,0.);
    EXPECT_LE(std::abs(out.mapped_rate-previous_rate),kAcceleration*kDt+1e-8);
    if (i==0) { EXPECT_GT(out.value,0.); } // Ordinary braking is not an emergency stop.
    previous=out.value;previous_rate=out.mapped_rate;prior_fraction=out.mapped_fraction;
  }
  EXPECT_NEAR(previous,0.,1e-10);EXPECT_NEAR(previous_rate,0.,1e-9);
  limiter.reset();const auto restart=limiter.step(.20,0.,0.,curve.reference,curve,kRate,kAcceleration,kDt);
  EXPECT_NEAR(restart.mapped_rate,kAcceleration*kDt,1e-9);
}

TEST(ResponseCommandLimiter, ConflictingSiBrakingBoundsWinAndAreExplicitlyReported) {
  const auto curve=yawCurve();ResponseCommandLimiter limiter;
  const auto startup=limiter.step(.4,0.,-.5,.5,curve,kRate,kAcceleration,kDt);
  ASSERT_GT(startup.value,0.);
  // A higher-priority hard SI bound must not become an unbounded navigation
  // hold merely because the optional output-domain acceleration is infeasible.
  const auto forced=limiter.step(0.,startup.value,0.,0.,curve,kRate,kAcceleration,kDt);
  EXPECT_TRUE(forced.priority_override);EXPECT_DOUBLE_EQ(forced.value,0.);
  EXPECT_DOUBLE_EQ(forced.mapped_fraction,0.);
  limiter.reset();
  const auto after_stop=limiter.step(.4,0.,-.5,.5,curve,kRate,kAcceleration,kDt);
  EXPECT_NEAR(after_stop.value,startup.value,1e-14);
  EXPECT_NEAR(after_stop.mapped_rate,startup.mapped_rate,1e-12);
}

TEST(ResponseCommandLimiter, InvalidTimeOrBoundsCannotInventIntegrationCredit) {
  ResponseCommandLimiter limiter;const auto curve=forwardCurve();
  for (double bad_dt : {0.,-.05,double(NAN),double(INFINITY)})
    EXPECT_THROW(limiter.step(.2,0.,0.,.3,curve,kRate,kAcceleration,bad_dt),std::invalid_argument);
  EXPECT_THROW(limiter.step(.2,0.,.3,0.,curve,kRate,kAcceleration,kDt),std::invalid_argument);
  EXPECT_THROW(limiter.step(.2,0.,0.,.3,curve,0.,kAcceleration,kDt),std::invalid_argument);
  EXPECT_THROW(limiter.step(.2,0.,0.,.3,curve,kRate,NAN,kDt),std::invalid_argument);
}
