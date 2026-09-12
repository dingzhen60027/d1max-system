#include <gtest/gtest.h>
#include "d1max_localization/registration_information.hpp"
using namespace d1max_localization;
TEST(Information, FullyConstrained) {
  EXPECT_NEAR(informationRatio(Information6::Identity(),Eigen::Vector3d::Zero(),1.),1.,1e-12);
}
TEST(Information, WeakAxesAreNotWhitenedAway) {
  Information6 h=Information6::Identity();h(3,3)=1e-5;
  EXPECT_LT(informationRatio(h,Eigen::Vector3d::Zero(),1.),.002);
  h(3,3)=0;EXPECT_EQ(informationRatio(h,Eigen::Vector3d::Zero(),1.),0.);
}
TEST(Information, MapOriginDoesNotChangeObservability) {
  Information6 h=Information6::Identity(),a=Information6::Identity();
  Eigen::Vector3d center(100.,-50.,7.);a.block<3,3>(3,0)=skew(center);
  Information6 shifted=a.inverse().transpose()*h*a.inverse();
  EXPECT_NEAR(informationRatio(shifted,center,1.),1.,1e-8);
}
TEST(Information, InvalidFailsClosed) {
  Information6 h=Information6::Identity();h(0,0)=-1;
  EXPECT_EQ(informationRatio(h,Eigen::Vector3d::Zero(),1.),0.);
}
