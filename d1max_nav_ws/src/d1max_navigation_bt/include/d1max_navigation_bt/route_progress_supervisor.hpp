#pragma once
#include <d1max_navigation_bt_interfaces/msg/route_snapshot.hpp>
#include <d1max_planning_interfaces/msg/route_progress.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include "d1max_navigation_bt/execution_identity.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

namespace d1max_navigation_bt::exec3 {
// This verifier consumes the production ReferenceCore projection, rather than
// inventing another nearest-neighbour/global route manager. It independently
// checks that projection against the fixed full route, its fixed anchor and a
// literal source-matched local observation. It controls the blocked budget only.
class RouteProgressSupervisor {
  using Progress=d1max_planning_interfaces::msg::RouteProgress;
  using Snapshot=d1max_navigation_bt_interfaces::msg::RouteSnapshot;
  using Pose=geometry_msgs::msg::Pose;
  struct Body {int64_t source;Pose pose;std::string frame;};
public:
  bool bind(const Snapshot&s,const Version&v,const std::string&transport) {
    if(route_||s.schema_version!=2||s.session_id!=v.session_id||s.task_id!=v.task_id||
       s.route_id!=v.route_id||s.route_hash!=v.route_hash||s.map_version_id!=v.map_version_id||
       s.localization_epoch!=static_cast<int64_t>(v.localization_epoch)||s.localization_seed_id!=v.localization_seed_id||
       s.point_reference!="ground"||s.frame_id.empty()||s.path.header.frame_id!=s.frame_id||
       s.path.poses.size()<2||s.edge_segments.size()+1!=s.path.poses.size()||s.segments.empty())return false;
    std::vector<double>arc{0.};
    for(size_t i=0;i<s.path.poses.size();++i) {
      if(!finite(s.path.poses[i].pose.position))return false;
      if(i)arc.push_back(arc.back()+distance(s.path.poses[i-1].pose.position,s.path.poses[i].pose.position));
    }
    if(arc.back()<=1e-9)return false;
    size_t previous_end=0;
    for(const auto&segment:s.segments) {
      if(segment.segment_id.empty()||segment.begin_index!=previous_end||segment.end_index<=segment.begin_index||
         segment.end_index>=s.path.poses.size())return false;
      for(size_t edge=segment.begin_index;edge<segment.end_index;++edge)
        if(s.edge_segments[edge]!=segment.segment_id)return false;
      previous_end=segment.end_index;
    }
    if(previous_end+1!=s.path.poses.size())return false;
    route_=s;arc_=std::move(arc);task_=v;transport_=transport;return true;
  }
  template<class State>void observeBody(const State&s) {
    constexpr unsigned schema=std::is_same_v<State,d1max_planning_interfaces::msg::LocalNavigationState>?1u:2u;
    if(!route_||s.schema_version!=schema||!s.usable||s.session_id!=task_.session_id||s.map_version_id!=task_.map_version_id||
       s.localization_epoch!=task_.localization_epoch||s.localization_seed_id!=task_.localization_seed_id||
       !canonical(s.source_stamp)||s.local_odometry.header.stamp!=s.source_stamp||
       s.local_odometry.child_frame_id!="d1max_loc_base_link"||!finite(s.local_odometry.pose.pose))return;
    const auto source=ns(s.source_stamp);if(source<=0)return;
    // Arrival order on independent topics is not a source-time ordering rule.
    // Repeated bodies may match a pending projection, but cannot renew time.
    for(const auto&b:bodies_)if(b.source==source)return;
    if(bodies_.size()==32)bodies_.erase(bodies_.begin());
    bodies_.push_back({source,s.local_odometry.pose.pose,s.local_odometry.header.frame_id});
  }
  bool observe(const Progress&p,double now) {
    if(!route_||p.schema_version!=1||!sameTask(task_,p.version)||!complete(p.version)||
       p.sequence<=sequence_||p.transport_mode!=transport_||p.frame_id!=route_->frame_id||
       p.odom_body_pose.header.frame_id.empty()||p.odom_body_pose.header.stamp!=p.body_source_stamp||
       !canonical(p.body_source_stamp)||!canonical(p.anchor_source_stamp)||
       !fresh(seconds(p.body_source_stamp),now,.4)||ns(p.anchor_source_stamp)<=0||
       ns(p.anchor_source_stamp)>ns(p.body_source_stamp)||!finite(p.odom_body_pose.pose)||
       !finite(p.map_from_odom)||!finite(p.source_map_body_xyz)||
       !std::isfinite(p.body_reference_height_m)||p.body_reference_height_m<.05||p.body_reference_height_m>1.5||
       !std::isfinite(p.measured_arc_m)||!std::isfinite(p.confirmed_arc_m)||!std::isfinite(p.cross_track_m)||
       static_cast<size_t>(p.edge_index)+1>=arc_.size()||p.measured_arc_m<arc_[p.edge_index]-1e-5||
       p.measured_arc_m>arc_[p.edge_index+1]+1e-5||p.confirmed_arc_m<p.measured_arc_m-1e-5||
       p.confirmed_arc_m>arc_.back()+1e-5||p.cross_track_m<0||p.cross_track_m>1.5||
       route_->edge_segments[p.edge_index]!=p.version.segment_id)return false;
    const auto transformed=transform(p.map_from_odom,p.odom_body_pose.pose.position);
    if(distance(transformed,p.source_map_body_xyz)>1e-5)return false;
    auto ground=p.source_map_body_xyz;ground.z-=p.body_reference_height_m;
    const auto&a=route_->path.poses[p.edge_index].pose.position;
    const auto&b=route_->path.poses[p.edge_index+1].pose.position;
    const double span=arc_[p.edge_index+1]-arc_[p.edge_index];if(span<=1e-9)return false;
    const double t=std::clamp((p.measured_arc_m-arc_[p.edge_index])/span,0.,1.);
    geometry_msgs::msg::Point on;on.x=a.x+(b.x-a.x)*t;on.y=a.y+(b.y-a.y)*t;on.z=a.z+(b.z-a.z)*t;
    const double lateral=std::hypot(ground.x-on.x,ground.y-on.y);
    const double free=std::clamp(((ground.x-a.x)*(b.x-a.x)+(ground.y-a.y)*(b.y-a.y)+
      (ground.z-a.z)*(b.z-a.z))/(span*span),0.,1.);
    if(std::abs(lateral-p.cross_track_m)>1e-5||std::abs(ground.z-on.z)>.3||std::abs(free-t)*span>.3)return false;
    sequence_=p.sequence;
    for(auto&old:pending_)if(sameVersion(old.version,p.version)) {
      if(ns(p.body_source_stamp)<=ns(old.body_source_stamp))return false;
      old=p;return true;
    }
    if(pending_.size()==8)pending_.erase(pending_.begin());pending_.push_back(p);return true;
  }
  bool advance(const Version&v,const std::string&frame,double now) {
    const Progress*p=nullptr;
    for(auto i=pending_.rbegin();i!=pending_.rend();++i)if(sameVersion(i->version,v)){p=&*i;break;}
    if(!p||!fresh(seconds(p->body_source_stamp),now,.4)||p->odom_body_pose.header.frame_id!=frame)return false;
    const auto source=ns(p->body_source_stamp);if(source<=last_source_)return false;
    const Body*body=nullptr;for(const auto&b:bodies_)if(b.source==source&&b.frame==frame){body=&b;break;}
    if(!body||body->pose!=p->odom_body_pose.pose)return false;
    if(!previous_) {baseline(*p);return false;}
    const auto&old=*previous_;
    const double dt=(source-last_source_)*1e-9;
    const double travel=distance(p->odom_body_pose.pose.position,old.odom_body_pose.pose.position);
    if(travel>1.5*dt+.05)return false;
    const bool anchor_changed=p->version.anchor_id!=old.version.anchor_id||
      p->version.anchor_revision!=old.version.anchor_revision;
    const bool segment_changed=p->version.segment_id!=old.version.segment_id;
    if(anchor_changed||segment_changed) {
      // Coordinate corrections and portal transitions are not robot motion.
      // Rebase without clearing an existing blocked episode or adding samples.
      if(anchor_changed&&p->version.anchor_revision<=old.version.anchor_revision)return false;
      if(!segment_changed&&std::abs(p->measured_arc_m-old.measured_arc_m)>.35+travel+1e-5)return false;
      if(segment_changed&&!adjacent(old,*p,travel))return false;
      baseline(*p);return false;
    }
    if(p->map_from_odom!=old.map_from_odom||p->anchor_source_stamp!=old.anchor_source_stamp||
       p->body_reference_height_m!=old.body_reference_height_m)return false;
    if(p->measured_arc_m<old.measured_arc_m-.35-1e-5||
       p->measured_arc_m>old.measured_arc_m+std::min(1.,1.5*dt+.05)+1e-5||
       p->confirmed_arc_m<old.confirmed_arc_m-1e-5||
       p->confirmed_arc_m>std::max(old.confirmed_arc_m,p->measured_arc_m)+1e-5)return false;
    local_travel_+=travel;previous_=*p;last_source_=source;
    const double gain=p->confirmed_arc_m-progress_arc_;
    if(gain<.03||local_travel_+1e-5<gain)return false;
    progress_arc_=p->confirmed_arc_m;local_travel_=0.;return true;
  }
private:
  static int64_t ns(const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
  static bool canonical(const builtin_interfaces::msg::Time&t){return t.sec>=0&&t.nanosec<1000000000u&&ns(t)>0;}
  static bool finite(const geometry_msgs::msg::Point&p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
  static bool finite(const Pose&p){const auto&q=p.orientation;const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return finite(p.position)&&std::isfinite(n)&&std::abs(n-1.)<=.001;}
  static double distance(const geometry_msgs::msg::Point&a,const geometry_msgs::msg::Point&b){
    return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));}
  static geometry_msgs::msg::Point transform(const Pose&t,const geometry_msgs::msg::Point&p){
    const auto&q=t.orientation;const double tx=2*(q.y*p.z-q.z*p.y),ty=2*(q.z*p.x-q.x*p.z),tz=2*(q.x*p.y-q.y*p.x);
    geometry_msgs::msg::Point o;o.x=t.position.x+p.x+q.w*tx+q.y*tz-q.z*ty;
    o.y=t.position.y+p.y+q.w*ty+q.z*tx-q.x*tz;o.z=t.position.z+p.z+q.w*tz+q.x*ty-q.y*tx;return o;
  }
  bool adjacent(const Progress&a,const Progress&b,double observed_travel)const {
    for(size_t i=0;i+1<route_->segments.size();++i) {
      const auto&old=route_->segments[i];const auto&next=route_->segments[i+1];
      if(old.segment_id!=a.version.segment_id||next.segment_id!=b.version.segment_id)continue;
      if(old.end_index!=next.begin_index||old.exit_portal_id.empty()||old.exit_portal_id!=next.entry_portal_id||
         std::abs(a.measured_arc_m-arc_[old.end_index])>.12+observed_travel||std::abs(b.measured_arc_m-arc_[next.begin_index])>.12)return false;
      auto ground=b.source_map_body_xyz;ground.z-=b.body_reference_height_m;
      return distance(ground,route_->path.poses[old.end_index].pose.position)<=.12;
    }return false;
  }
  void baseline(const Progress&p){previous_=p;last_source_=ns(p.body_source_stamp);progress_arc_=p.confirmed_arc_m;local_travel_=0.;}
  Version task_;std::string transport_;std::optional<Snapshot>route_;std::vector<double>arc_;
  std::vector<Body>bodies_;std::vector<Progress>pending_;std::optional<Progress>previous_;
  uint64_t sequence_=0;int64_t last_source_=0;double progress_arc_=0.,local_travel_=0.;
};
} // namespace d1max_navigation_bt::exec3
