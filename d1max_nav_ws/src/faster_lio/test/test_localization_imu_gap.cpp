#include <gtest/gtest.h>
#include <limits>
#include "localization_imu_gap.hpp"
#include "localization_gap_coast.hpp"

using faster_lio::AssessLocalizationImuGap;

TEST(LocalizationImuGap, NormalSamplesKeepOriginalUncertainty) {
    const auto q=AssessLocalizationImuGap(.005, .003, .015, .1, .08);
    EXPECT_TRUE(q.accepted); EXPECT_FALSE(q.degraded); EXPECT_EQ(q.noise_scale, 1.0);
}
TEST(LocalizationImuGap, FiftyToHundredMillisecondsAreDegradedNotReset) {
    for(double gap : {.0500009, .055, .07, .08, .1}) {
        const auto q=AssessLocalizationImuGap(gap, .01, .015, .1, .08);
        EXPECT_TRUE(q.accepted); EXPECT_TRUE(q.degraded); EXPECT_GT(q.noise_scale, 1.0);
    }
}
TEST(LocalizationImuGap, ToleranceOnlyCoversEpochArithmetic) {
    constexpr double t=1790242500.0;
    EXPECT_TRUE(AssessLocalizationImuGap((t+.1)-t,.01,.015,.1,.08).accepted);
    EXPECT_TRUE(AssessLocalizationImuGap(.1000009,.01,.015,.1,.08).accepted);
    EXPECT_FALSE(AssessLocalizationImuGap(.10001,.01,.015,.1,.08).accepted);
}
TEST(LocalizationImuGap, LongOutagesStillRequireRecovery) {
    for(double gap : {.11, .15, .3, 1.0})
        EXPECT_FALSE(AssessLocalizationImuGap(gap,.001,.015,.1,.08).accepted);
}
TEST(LocalizationImuGap, FastRotationIsNotHiddenByLargerTimeLimit) {
    EXPECT_FALSE(AssessLocalizationImuGap(.08,.081,.015,.1,.08).accepted);
    EXPECT_TRUE(AssessLocalizationImuGap(.08,.079,.015,.1,.08).accepted);
}
TEST(LocalizationImuGap, NormalFrameAfterGapDoesNotKeepDegradedNoise) {
    const auto a=AssessLocalizationImuGap(.08,.01,.015,.1,.08);
    const auto b=AssessLocalizationImuGap(.005,.001,.015,.1,.08);
    EXPECT_GT(a.noise_scale,b.noise_scale); EXPECT_FALSE(b.degraded);
    EXPECT_EQ(b.noise_scale,1.0);
}
TEST(LocalizationImuGap, ConfigurationAndInvalidTimeFailClearly) {
    EXPECT_FALSE(AssessLocalizationImuGap(-.01,.01,.015,.1,.08).accepted);
    EXPECT_FALSE(AssessLocalizationImuGap(.05,.01,.015,.2,.08).accepted);
    EXPECT_FALSE(AssessLocalizationImuGap(.05,.01,.0,.1,.08).accepted);
    EXPECT_FALSE(AssessLocalizationImuGap(.05,std::numeric_limits<double>::quiet_NaN(),.015,.1,.08).accepted);
    EXPECT_FALSE(AssessLocalizationImuGap(.055,.01,.015,.05,.08).accepted);
}

namespace {
struct CoastState {
    Eigen::Vector3d pos=Eigen::Vector3d::Zero(),vel{.3,0.,0.};
    Eigen::Quaterniond rot=Eigen::Quaterniond::Identity();
};
using CoastCov=Eigen::Matrix<double,23,23>;
}
TEST(LocalizationGapCoast, Real359994msGapModelsVelocityWithoutMissingAcceleration) {
    CoastState s;CoastCov p=CoastCov::Identity();const auto prior=p;
    ASSERT_TRUE(faster_lio::CoastLocalizationFilter(s,p,.359994,Eigen::Vector3d::Zero()));
    EXPECT_NEAR(s.pos.x(),.1079982,1e-12);EXPECT_DOUBLE_EQ(s.vel.x(),.3);
    EXPECT_GT(p.trace(),prior.trace());EXPECT_NEAR((p-p.transpose()).norm(),0.,1e-12);
    EXPECT_EQ(s.rot.coeffs(),Eigen::Quaterniond::Identity().coeffs());
}
TEST(LocalizationGapCoast, LongGapFastMotionClockAndNumericFaultCannotMutatePrior) {
    for(int kind=0;kind<5;++kind) {
        CoastState s;CoastCov p=CoastCov::Identity();double dt=.36;Eigen::Vector3d w=Eigen::Vector3d::Zero();
        if(kind==0)dt=.50001;if(kind==1)s.vel.x()=2.;if(kind==2)w.z()=2.;
        if(kind==3)dt=-.1;if(kind==4)p(0,0)=std::numeric_limits<double>::infinity();
        const auto before=s;const auto old=p;
        EXPECT_FALSE(faster_lio::CoastLocalizationFilter(s,p,dt,w));
        EXPECT_EQ(s.pos,before.pos);EXPECT_EQ(s.vel,before.vel);EXPECT_EQ(p,old);
    }
}
