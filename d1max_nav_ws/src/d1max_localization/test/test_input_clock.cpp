#include <gtest/gtest.h>
#include "d1max_localization/input_clock.hpp"
using d1max_localization::InputClock;
TEST(InputClock, StrictRejectsWrongEpoch){InputClock c;EXPECT_FALSE(c.observe(100.,17320311.,1.));EXPECT_FALSE(c.offset(1.));}
TEST(InputClock, OneSharedConstantPreservesIntervals){
 InputClock c(true,3,.02);
 EXPECT_FALSE(c.observe(100.,17320311.020,1.));
 EXPECT_FALSE(c.observe(100.01,17320311.025,1.005));
 EXPECT_TRUE(c.observe(100.03,17320311.047,1.027));
 const double offset=*c.offset(1.027);EXPECT_NEAR(offset,17320211.015,1e-6);
 EXPECT_TRUE(c.observe(100.04,17320311.080,1.06));
 EXPECT_EQ(offset,*c.offset(1.06));
 EXPECT_NEAR((100.14+offset)-(100.04+offset),.10,1e-6);
}
TEST(InputClock, DuplicateCannotFinishCalibration){InputClock c(true,3,.02);for(int i=0;i<20;++i)c.observe(100.,100.,1.);EXPECT_EQ(c.count(),1U);EXPECT_FALSE(c.offset(1.));}
TEST(InputClock, StaleCannotBeMadeFresh){InputClock c;EXPECT_TRUE(c.observe(100.,100.,1.));EXPECT_FALSE(c.observe(100.01,100.41,1.41));EXPECT_FALSE(c.offset(1.41));}
TEST(InputClock, SensorRollbackLatches){InputClock c;EXPECT_TRUE(c.observe(100.,100.,1.));EXPECT_FALSE(c.observe(98.,100.1,1.1));EXPECT_TRUE(c.faulted());EXPECT_FALSE(c.observe(100.2,100.2,1.2));}
TEST(InputClock, HostStepLatches){InputClock c;EXPECT_TRUE(c.observe(100.,100.,1.));EXPECT_FALSE(c.observe(100.1,101.,1.1));EXPECT_TRUE(c.faulted());}
TEST(InputClock, SensorForwardStepLatches){InputClock c;EXPECT_TRUE(c.observe(100.,100.,1.));EXPECT_FALSE(c.observe(102.,100.1,1.1));EXPECT_TRUE(c.faulted());}
TEST(InputClock, ReplayLatches){InputClock c(true);c.disableForReplay();EXPECT_FALSE(c.observe(100.,100.,1.));EXPECT_TRUE(c.faulted());}
TEST(InputClock, OutageDoesNotRecalibrate){InputClock c;EXPECT_TRUE(c.observe(100.,100.,1.));EXPECT_FALSE(c.offset(2.));EXPECT_TRUE(c.observe(101.,101.,2.));EXPECT_DOUBLE_EQ(*c.offset(2.),0.);}
TEST(InputClock, DelayedPacketDropsWithoutFalseClockFault){
 InputClock c;ASSERT_TRUE(c.observe(100.,100.,1.));
 EXPECT_FALSE(c.observe(100.01,101.,2.));EXPECT_FALSE(c.faulted());
 EXPECT_FALSE(c.offset(2.));EXPECT_EQ(c.count(),1U);
 EXPECT_TRUE(c.observe(101.01,101.01,2.01));EXPECT_DOUBLE_EQ(*c.offset(2.01),0.);
}
TEST(InputClock, DelayedPacketStillChecksHostClock){
 InputClock c;ASSERT_TRUE(c.observe(100.,100.,1.));
 EXPECT_FALSE(c.observe(100.01,102.,2.));EXPECT_TRUE(c.faulted());
}
