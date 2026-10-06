#pragma once
#include "execution_transport_core.hpp"
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <d1max_planning_interfaces/msg/navigation_state.hpp>

namespace d1monitor::execution3 {
// Control-driven test plant telemetry, not a replacement for live OnMcData.
// A global correction cannot invent velocity or renew the local source time.
struct MockMcSample {
  uint64_t raw_ns=0,epoch=0;
  std::string seed;
  SourceClock source;double linear=0,angular=0;
};
class MockTelemetry {
public:
  MockTelemetry(std::string session,std::string map,bool local_enabled)
    :session_(std::move(session)),map_(std::move(map)),local_enabled_(local_enabled){}
  std::optional<MockMcSample> local(const d1max_planning_interfaces::msg::LocalNavigationState&m,SourceClock now) {
    if(!local_enabled_||m.schema_version!=1)return {};
    return admit(m,now);
  }
  std::optional<MockMcSample> global(const d1max_planning_interfaces::msg::NavigationState&m,SourceClock now) {
    if(local_enabled_||m.schema_version!=2)return {};
    return admit(m,now);
  }
private:
  template<class Message> std::optional<MockMcSample> admit(const Message&m,SourceClock now) {
    if(m.session_id!=session_||m.map_version_id!=map_||!m.usable||m.localization_epoch==0||m.localization_seed_id.empty())return {};
    const auto source=SourceClock::fromNanoseconds(nanoseconds(m.source_stamp));
    const auto&o=m.local_odometry;
    if(!fresh(m.source_stamp,now,.25)||!fresh(m.posterior_stamp,now,.25)||!fresh(m.imu_stamp,now,.25)||
       nanoseconds(m.posterior_stamp)>nanoseconds(m.source_stamp)||nanoseconds(m.imu_stamp)>nanoseconds(m.source_stamp)||
       !std::isfinite(m.extrapolation_sec)||m.extrapolation_sec<0||m.extrapolation_sec>.1||
       std::abs(elapsed(m.source_stamp,m.imu_stamp)-m.extrapolation_sec)>1e-5||o.header.stamp!=m.source_stamp||
       o.header.frame_id!="d1max_loc_odom"||o.child_frame_id!="d1max_loc_base_link")return {};
    const auto norm=[](const auto&v){return std::hypot(v.x,v.y,v.z);};
    const auto&v=o.twist.twist;const double linear=norm(v.linear),angular=norm(v.angular);
    if(!std::isfinite(linear)||!std::isfinite(angular))return {};
    return MockMcSample{static_cast<uint64_t>(m.source_stamp.sec)*1000000000ULL+m.source_stamp.nanosec,
      m.localization_epoch,m.localization_seed_id,source,linear,angular};
  }
  std::string session_,map_;bool local_enabled_;
};
}
