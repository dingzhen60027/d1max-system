#include "execution_mock_telemetry.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace d1monitor::execution3;
using Local=d1max_planning_interfaces::msg::LocalNavigationState;
using Global=d1max_planning_interfaces::msg::NavigationState;
template<class Message> static Message sample(double time) {
  Message m;m.schema_version=std::is_same_v<Message,Local>?1:2;
  m.session_id="session";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
  m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(time);m.usable=true;
  m.local_odometry.header.stamp=m.source_stamp;m.local_odometry.header.frame_id="d1max_loc_odom";
  m.local_odometry.child_frame_id="d1max_loc_base_link";
  m.local_odometry.twist.twist.linear.x=.12;m.local_odometry.twist.twist.angular.z=.23;return m;
}
int main() {
  MockTelemetry isolated("session","map",true);
  auto decoded=isolated.local(sample<Local>(10.),10.01);
  assert(decoded&&decoded->raw_ns==10000000000ULL&&decoded->epoch==1&&decoded->seed=="seed");
  assert(decoded->source==10.&&decoded->linear==.12&&decoded->angular==.23);
  // A full second without a global pair leaves the independent 50 Hz local
  // telemetry alive. Global frames/TF/pose never supply mock MC samples.
  for(int i=1;i<=50;++i) {
    double time=10.+i*.02;
    assert(isolated.local(sample<Local>(time),time+.001));
    assert(!isolated.global(sample<Global>(10.),time+.001));
    auto correction=sample<Global>(time);correction.local_odometry.twist.twist.linear.x=99.;
    correction.global_odometry.pose.pose.position.x=1000.;
    assert(!isolated.global(correction,time+.001));
  }
  // Explicit legacy config still works, but cannot mix two telemetry owners.
  MockTelemetry legacy("session","map",false);
  assert(legacy.global(sample<Global>(10.),10.01));assert(!legacy.local(sample<Local>(10.),10.01));
  assert(!isolated.local(sample<Local>(10.),10.251));
  assert(!isolated.local(sample<Local>(10.1),10.));
  for(int variant=0;variant<13;++variant) {
    auto bad=sample<Local>(10.);
    switch(variant) {
      case 0:bad.schema_version=2;break;case 1:bad.session_id="foreign";break;
      case 2:bad.map_version_id="foreign";break;case 3:bad.usable=false;break;
      case 4:bad.localization_epoch=0;break;case 5:bad.localization_seed_id.clear();break;
      case 6:bad.local_odometry.header.frame_id="map";break;
      case 7:bad.local_odometry.child_frame_id="lidar";break;
      case 8:bad.local_odometry.header.stamp=stamp(9.99);break;
      case 9:bad.posterior_stamp=stamp(9.);break;case 10:bad.imu_stamp=stamp(10.1);break;
      case 11:bad.extrapolation_sec=.05;break;
      case 12:bad.local_odometry.twist.twist.linear.z=std::numeric_limits<double>::quiet_NaN();break;
    }
    assert(!isolated.local(bad,10.01));
  }
  // Norms come from actual local twist, never the requested command.
  auto actual=sample<Local>(10.);actual.local_odometry.twist.twist.linear.x=.03;
  actual.local_odometry.twist.twist.linear.y=.04;
  auto motion=isolated.local(actual,10.01);assert(motion&&std::abs(motion->linear-.05)<1e-12);
  std::cout<<"mock telemetry local independence and source contracts passed\n";
}
