#include <gtest/gtest.h>
#include <plan_manage/final_endpoint_reseed.hpp>
#include <limits>

namespace {
using P=Eigen::Vector3d;
using scan_planner::finalEndpointShortConnectionAllowed;
TEST(FinalEndpointShortConnection, TrueFinalReseedClosesTheExistingFifteenToTwentyCmGap) {
  const P endpoint(2.,0.,.55);
  for(double residual:{.1501,.16,.19,.1999}) {
    const P actual=endpoint+P(residual,0.,0.);
    EXPECT_TRUE(finalEndpointShortConnectionAllowed(true,{actual,endpoint},actual,endpoint,false));
    EXPECT_FALSE(finalEndpointShortConnectionAllowed(false,{actual,endpoint},actual,endpoint,false));
  }
}
TEST(FinalEndpointShortConnection, NormalHoverNearZeroAndSplicesRetainTheirOldBounds) {
  const P endpoint(2.,0.,.55);
  for(double residual:{0.,.0001,.1,.15,.2,.25}) {
    const P actual=endpoint+P(residual,0.,0.);
    EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual,endpoint},actual,endpoint,false));
  }
  const P actual(2.16,0.,.55);
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual,endpoint},actual,endpoint,true));
}
TEST(FinalEndpointShortConnection, TypedTargetFlagDoesNotAuthorizeDifferentOrMalformedReference) {
  const P actual(2.16,0.,.55),endpoint(2.,0.,.55);
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{},actual,endpoint,false));
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual},actual,endpoint,false));
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual,P(2.08,.1,.55),endpoint},actual,endpoint,false));
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual+P(.01,0.,0.),endpoint},actual,endpoint,false));
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual,endpoint+P(.01,0.,0.)},actual,endpoint,false));
  const P bad(std::numeric_limits<double>::quiet_NaN(),0.,.55);
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{bad,endpoint},bad,endpoint,false));
  EXPECT_FALSE(finalEndpointShortConnectionAllowed(true,{actual,bad},actual,bad,false));
}
} // namespace
