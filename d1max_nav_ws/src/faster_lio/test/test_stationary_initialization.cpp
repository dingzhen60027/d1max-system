#include <gtest/gtest.h>
#include "stationary_initialization.hpp"

TEST(StationaryInitialization, RequiresTimeAndStableMeasurements) {
  faster_lio::StationaryInitialization gate;
  for (int i=0;i<400;++i) gate.add(i*.005,{0,0,9.81},{.005,-.002,.001});
  EXPECT_FALSE(gate.ready(300));
  gate.add(2.0,{0,0,9.81},{.005,-.002,.001});
  EXPECT_TRUE(gate.ready(300));
  for (int i=401;i<900;++i) gate.add(i*.005,{double(i%2),0,9.81},{0,0,0});
  EXPECT_FALSE(gate.ready(300));
}
TEST(StationaryInitialization, RejectsMovingRotationAndGaps) {
  faster_lio::StationaryInitialization gate;
  for (int i=0;i<500;++i) gate.add(i*.005,{0,0,9.81},{0,0,.2});
  EXPECT_FALSE(gate.ready(300));
  gate.add(5.0,{0,0,9.81},{0,0,0});
  EXPECT_EQ(gate.samples.size(),1U);
  EXPECT_FALSE(gate.ready(300));
  gate.add(5.01,{NAN,0,9.81},{0,0,0});
  EXPECT_TRUE(gate.samples.empty());
}
