#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace d1monitor::execution3 {
using EntryVector=std::array<double,3>;
struct ActualCurveEntry {
  double curve_time=0.,arc=0.;
  EntryVector position{},velocity{};
  double position_error=0.,velocity_error=0.;
  std::int64_t body_source_stamp_ns=0;
};

// Pure geometry, not another admission or collision authority. Construction
// owns a bounded immutable copy; observation allocates nothing. This is the
// vendor/FixedBsplineSampler De Boor recurrence and derivative knot contract,
// not a clamped-knot approximation or a table-interpolated motion prediction.
class MeasuredCurveEntryCache {
 public:
  template<class Tagged>static std::optional<MeasuredCurveEntryCache> build(const Tagged& m) {
    const auto& s=m.trajectory;
    if(m.schema_version!=2||m.point_reference!="body_center"||m.frame_id.empty()||
       m.session_id.empty()||m.task_id.empty()||m.route_id.empty()||m.route_hash.size()!=64||
       m.map_version_id.empty()||!m.localization_epoch||m.localization_seed_id.empty()||
       !m.generation||m.segment_id.empty()||m.anchor_id.empty()||!m.anchor_revision||!m.context_sequence||
       s.order!=3||s.traj_id<=0||s.pos_pts.size()<4||s.pos_pts.size()>10000||
       s.knots.size()!=s.pos_pts.size()+4||!std::isfinite(s.yaw_dt))return {};
    MeasuredCurveEntryCache c;c.identity_=Identity::fromTagged(m);c.trajectory_id_=s.traj_id;
    c.segment_kind_=m.segment_kind;c.required_mode_=m.required_mode;c.yaw_dt_=s.yaw_dt;
    c.knots_=s.knots;c.yaw_=s.yaw_pts;c.points_.reserve(s.pos_pts.size());
    for(double y:c.yaw_)if(!std::isfinite(y))return {};
    for(const auto& p:s.pos_pts){const auto v=vector(p);if(!finite(v))return {};c.points_.push_back(v);}
    for(std::size_t i=0;i<c.knots_.size();++i)if(!std::isfinite(c.knots_[i])||
        (i&&c.knots_[i]-c.knots_[i-1]<=1e-9))return {};
    c.duration_=c.knots_[c.points_.size()]-c.knots_[3];
    if(!std::isfinite(c.duration_)||c.duration_<=.01||c.duration_>120.)return {};
    c.velocity_knots_.assign(c.knots_.begin()+1,c.knots_.end()-1);
    c.velocity_points_.reserve(c.points_.size()-1);
    for(std::size_t i=0;i+1<c.points_.size();++i){
      EntryVector p{};const double divisor=c.knots_[i+4]-c.knots_[i+1];
      for(unsigned a=0;a<3;++a)p[a]=3.*(c.points_[i+1][a]-c.points_[i][a])/divisor;
      if(!finite(p))return {};
      c.speed_bound_=std::max(c.speed_bound_,norm(p));c.velocity_points_.push_back(p);
    }
    const unsigned n=std::clamp(static_cast<unsigned>(std::ceil(c.duration_/.02)),40u,6000u);
    c.times_.resize(n+1);c.arcs_.resize(n+1);c.samples_.reserve(n+1);
    for(unsigned i=0;i<=n;++i){
      c.times_[i]=c.duration_*static_cast<double>(i)/n;
      c.samples_.push_back(c.positionAt(c.times_[i]));if(!finite(c.samples_.back()))return {};
      if(i)c.arcs_[i]=c.arcs_[i-1]+distance(c.samples_[i],c.samples_[i-1]);
    }
    if(!std::isfinite(c.arcs_.back())||c.arcs_.back()<1e-6)return {};
    return c;
  }
  template<class Version>bool versionMatches(const Version& v)const {
    return v.schema_version==3&&identity_.matchesVersion(v);
  }
  template<class Tagged>bool matches(const Tagged& m)const {
    const auto& s=m.trajectory;
    if(m.schema_version!=2||m.point_reference!="body_center"||!identity_.matchesTagged(m)||
       m.segment_kind!=segment_kind_||m.required_mode!=required_mode_||s.order!=3||
       s.traj_id!=trajectory_id_||s.knots!=knots_||s.yaw_pts!=yaw_||s.yaw_dt!=yaw_dt_||
       s.pos_pts.size()!=points_.size())return false;
    for(std::size_t i=0;i<points_.size();++i)if(vector(s.pos_pts[i])!=points_[i])return false;
    return true;
  }
  double duration()const{return duration_;}
  EntryVector positionAt(double time)const{return evaluate(points_,knots_,3,time);}
  EntryVector velocityAt(double time)const{return evaluate(velocity_points_,velocity_knots_,2,time);}

  // The caller supplies its original end-to-end command budget. Current body
  // acquisition remains independently fresh (100 ms); a geometry observation
  // cannot refresh the prepared demand, admission or native collision proof.
  template<class Prepared,class Local>std::optional<ActualCurveEntry> observe(
      const Prepared& p,const Local& m,double maximum_speed,double now,double command_pipeline_bound_s=.1)const {
    const auto entry_ns=sourceNs(p.entry_source_stamp),body_ns=sourceNs(m.source_stamp);
    const double body_time=body_ns*1e-9;
    if(!std::isfinite(now)||!std::isfinite(maximum_speed)||maximum_speed<=0.||
       !std::isfinite(command_pipeline_bound_s)||command_pipeline_bound_s<=0.||
       speed_bound_>maximum_speed+1e-6||!versionMatches(p.demand.version)||p.demand.trajectory_id!=trajectory_id_||
       m.schema_version!=2||!m.usable||!identity_.matchesLocal(m)||entry_ns<=0||body_ns<entry_ns||
       m.valid_until.nanosec>=1000000000U||sourceNs(m.valid_until)<=body_ns||
       sourceNs(m.valid_until)-body_ns>100000000LL||now>sourceNs(m.valid_until)*1e-9||
       body_ns!=sourceNs(m.local_odometry.header.stamp)||m.local_odometry.header.frame_id!=identity_.frame||
       m.local_odometry.child_frame_id!="d1max_loc_base_link"||now<body_time||now-body_time>.1||
       now-entry_ns*1e-9>command_pipeline_bound_s||
       sourceNs(p.demand.body_source_stamp)!=entry_ns||sourceNs(p.measured_pose.header.stamp)!=entry_ns||
       p.measured_pose.header.frame_id!=identity_.frame||!std::isfinite(p.curve_time)||
       p.curve_time<0.||p.curve_time>duration_)return {};
    const auto original=positionAt(p.curve_time),original_velocity=velocityAt(p.curve_time);
    // First bind the certificate's fixed entry to THIS exact production curve.
    // Its identity, original source/entry and native sweep are never rewritten.
    if(distance(original,vector(p.curve_entry_pose.position))>1e-6||
       distance(original_velocity,vector(p.curve_entry_twist.linear))>1e-6||
       distance(original,vector(p.measured_pose.pose.position))>.0125||
       distance(original_velocity,vector(p.measured_twist.linear))>.05)return {};
    const auto actual=vector(m.local_odometry.pose.pose.position);
    const auto actual_velocity=worldVelocity(m);
    if(!finite(actual)||!actual_velocity)return {};
    const double seed_arc=arcAt(p.curve_time);
    // Acquisition delta bounds the connected local arc. It NEVER chooses a
    // curve parameter or extrapolates a pose; measured XYZ selects the entry.
    const double travel=std::min(.5,maximum_speed*((body_ns-entry_ns)*1e-9)+.0125);
    const double from=std::max(0.,seed_arc-travel),to=std::min(arcs_.back(),seed_arc+travel);
    ActualCurveEntry result;result.curve_time=p.curve_time;result.arc=seed_arc;result.position=original;
    double nearest=squareDistance(original,actual);
    const auto lower=std::lower_bound(arcs_.begin(),arcs_.end(),from);
    const std::size_t begin=lower==arcs_.begin()?0:static_cast<std::size_t>(lower-arcs_.begin()-1);
    // Binary-search the local ARC neighborhood, then evaluate the exact spline
    // at chord-selected parameters. No whole-curve rebuild or per-tick heap.
    for(std::size_t i=begin;i+1<arcs_.size()&&arcs_[i]<=to;++i){
      const double span=arcs_[i+1]-arcs_[i];const auto chord=subtract(samples_[i+1],samples_[i]);
      const double length_squared=dot(chord,chord);if(span<1e-12||length_squared<1e-12)continue;
      const double lo=std::clamp((from-arcs_[i])/span,0.,1.),hi=std::clamp((to-arcs_[i])/span,0.,1.);
      const double fraction=std::clamp(dot(subtract(actual,samples_[i]),chord)/length_squared,lo,hi);
      const double t=times_[i]+fraction*(times_[i+1]-times_[i]);const auto point=positionAt(t);
      const double error=squareDistance(point,actual);
      if(error+1e-12<nearest){nearest=error;result.curve_time=t;result.arc=arcs_[i]+fraction*span;result.position=point;}
    }
    result.velocity=velocityAt(result.curve_time);result.position_error=std::sqrt(nearest);
    result.velocity_error=distance(result.velocity,*actual_velocity);result.body_source_stamp_ns=body_ns;
    if(!finite(result.position)||!finite(result.velocity)||result.position_error>.0125||result.velocity_error>.05)return {};
    return result;
  }
 private:
  struct Identity {
    std::string session,task,route,route_hash,map,seed,segment,anchor,frame;
    std::uint64_t epoch=0,generation=0,anchor_revision=0,context=0,geometry_revision=0;
    template<class T>static Identity fromTagged(const T& t){return {t.session_id,t.task_id,t.route_id,t.route_hash,
      t.map_version_id,t.localization_seed_id,t.segment_id,t.anchor_id,t.frame_id,t.localization_epoch,
      t.generation,t.anchor_revision,t.context_sequence,t.map_geometry_revision};}
    template<class V>bool matchesVersion(const V& v)const{return session==v.session_id&&task==v.task_id&&
      route==v.route_id&&route_hash==v.route_hash&&map==v.map_version_id&&seed==v.localization_seed_id&&
      segment==v.segment_id&&anchor==v.anchor_id&&epoch==v.localization_epoch&&generation==v.reference_generation&&
      anchor_revision==v.anchor_revision&&context==v.context_sequence&&geometry_revision==v.map_geometry_revision;}
    template<class T>bool matchesTagged(const T& t)const{const auto i=fromTagged(t);return session==i.session&&
      task==i.task&&route==i.route&&route_hash==i.route_hash&&map==i.map&&seed==i.seed&&segment==i.segment&&
      anchor==i.anchor&&frame==i.frame&&epoch==i.epoch&&generation==i.generation&&anchor_revision==i.anchor_revision&&
      context==i.context&&geometry_revision==i.geometry_revision;}
    template<class L>bool matchesLocal(const L& l)const{return session==l.session_id&&map==l.map_version_id&&
      seed==l.localization_seed_id&&epoch==l.localization_epoch;}
  };
  template<class Time>static std::int64_t sourceNs(const Time& t){return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;}
  template<class Point>static EntryVector vector(const Point& p){return {p.x,p.y,p.z};}
  static bool finite(const EntryVector& p){return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]);}
  static EntryVector subtract(const EntryVector& a,const EntryVector& b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
  static double dot(const EntryVector& a,const EntryVector& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
  static double norm(const EntryVector& p){return std::hypot(p[0],p[1],p[2]);}
  static double distance(const EntryVector& a,const EntryVector& b){return norm(subtract(a,b));}
  static double squareDistance(const EntryVector& a,const EntryVector& b){const auto d=subtract(a,b);return dot(d,d);}
  static EntryVector evaluate(const std::vector<EntryVector>& points,const std::vector<double>& knots,int degree,double time){
    const double u=time+knots[degree];
    const double ub=std::min(std::max(knots[degree],u),knots[knots.size()-degree-1]);
    const int k=static_cast<int>(std::lower_bound(knots.begin()+degree+1,knots.begin()+points.size()+1,ub)-knots.begin())-1;
    std::array<EntryVector,4> d{};for(int i=0;i<=degree;++i)d[i]=points[k-degree+i];
    for(int r=1;r<=degree;++r)for(int i=degree;i>=r;--i){
      const double alpha=(ub-knots[i+k-degree])/(knots[i+1+k-r]-knots[i+k-degree]);
      for(unsigned a=0;a<3;++a)d[i][a]=(1-alpha)*d[i-1][a]+alpha*d[i][a];
    }
    return d[degree];
  }
  double arcAt(double time)const{
    const auto upper=std::upper_bound(times_.begin(),times_.end(),time);
    if(upper==times_.begin())return arcs_.front();
    if(upper==times_.end())return arcs_.back();
    const auto i=static_cast<std::size_t>(upper-times_.begin()-1);
    return arcs_[i]+(time-times_[i])/(times_[i+1]-times_[i])*(arcs_[i+1]-arcs_[i]);
  }
  template<class Local>static std::optional<EntryVector> worldVelocity(const Local& m){
    const auto& q=m.local_odometry.pose.pose.orientation;const auto v=vector(m.local_odometry.twist.twist.linear);
    const double length=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
    if(!finite(v)||!std::isfinite(length)||std::abs(length-1.)>.001)return {};
    // Exactly the SDK's measured child->odom quaternion rotation convention.
    const double tx=2.*(q.y*v[2]-q.z*v[1]),ty=2.*(q.z*v[0]-q.x*v[2]),tz=2.*(q.x*v[1]-q.y*v[0]);
    const EntryVector out{v[0]+q.w*tx+q.y*tz-q.z*ty,v[1]+q.w*ty+q.z*tx-q.x*tz,v[2]+q.w*tz+q.x*ty-q.y*tx};
    return finite(out)?std::optional<EntryVector>{out}:std::nullopt;
  }
  Identity identity_;std::int64_t trajectory_id_=0;std::string segment_kind_,required_mode_;
  double duration_=0.,speed_bound_=0.,yaw_dt_=0.;
  std::vector<double> knots_,velocity_knots_,yaw_,times_,arcs_;
  std::vector<EntryVector> points_,velocity_points_,samples_;
};
} // namespace d1monitor::execution3
