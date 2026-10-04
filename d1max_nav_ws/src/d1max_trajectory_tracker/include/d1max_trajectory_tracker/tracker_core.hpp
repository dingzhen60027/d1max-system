#pragma once

// ROS-independent admission/control core. Trajectory evaluation is SCAN's own
// implementation, not a second interpretation of its knot/time conventions.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bspline_opt/uniform_bspline.h>
#include "d1max_trajectory_tracker/curve_admission.hpp"

namespace d1max_trajectory_tracker
{
constexpr double HARD_PLANAR_SPEED = 1.5;
inline bool finite(double value) { return std::isfinite(value); }
inline double angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

// SCAN's limits are spatial norms, whereas SDK Move accepts planar velocity.
// Keep the real vertical component: dz per horizontal metre consumes part of
// the same spatial budget. Vertical/undefined directions cannot justify XY.
inline double spatialToPlanarScale(const Eigen::Vector3d& tangent) {
  if(!tangent.allFinite()) return 0.;
  const double planar=std::hypot(tangent.x(),tangent.y());
  const double spatial=std::hypot(planar,tangent.z());
  if(planar<=1e-12 || !finite(spatial) || spatial<=1e-12) return 0.;
  if(tangent.z()==0.) return 1.;
  return std::nextafter(std::clamp(planar/spatial,0.,1.),0.);
}

// Forward+yaw execution is not holonomic XY tracking. This uses the admitted
// vendor spline's actual derivatives; no lateral-acceleration model is guessed.
inline double curvatureForwardLimit(const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration,double max_yaw,double nominal) {
  if(!velocity.allFinite()||!acceleration.allFinite()||!finite(max_yaw)||max_yaw<=0.||
     !finite(nominal)||nominal<=0.)return 0.;
  const double speed=std::hypot(velocity.x(),velocity.y());
  if(speed<1e-5)return nominal; // zero tangent does not define a curvature
  const double cross=std::abs(velocity.x()*acceleration.y()-velocity.y()*acceleration.x());
  if(cross<=1e-12)return nominal;
  return std::min(nominal,max_yaw*speed*speed*speed/cross);
}

// Spatially indexed forward-speed envelope. A future low curvature speed is
// a braking target, not an instantaneous limit at the present curve entry.
// Arc is XYZ distance; conservative_planar_scale converts it to a guaranteed
// lower bound on horizontal travel before using the planar braking budget.
inline std::vector<double> brakingSpeedEnvelope(const std::vector<double>& arcs,
    const std::vector<double>& local_limits,double planar_acceleration,double conservative_planar_scale) {
  if(arcs.size()<2||arcs.size()!=local_limits.size()||!finite(planar_acceleration)||
     planar_acceleration<0.||!finite(conservative_planar_scale)||
     conservative_planar_scale<0.||conservative_planar_scale>1.||
     (planar_acceleration==0.&&conservative_planar_scale>0.))return {};
  auto out=local_limits;
  for(std::size_t i=0;i<arcs.size();++i)if(!finite(arcs[i])||!finite(out[i])||out[i]<0.||
    (i&&arcs[i]<arcs[i-1]))return {};
  for(std::size_t i=arcs.size()-1;i>0;--i) {
    const double reachable=std::sqrt(out[i]*out[i]+2.*planar_acceleration*
      conservative_planar_scale*(arcs[i]-arcs[i-1]));
    out[i-1]=std::min(out[i-1],reachable);
  }
  return out;
}
inline double brakingEnvelopeAt(const std::vector<double>& arcs,const std::vector<double>& limits,
    double arc,double planar_acceleration,double conservative_planar_scale) {
  if(arcs.size()<2||arcs.size()!=limits.size()||!finite(arc)||!finite(planar_acceleration)||
     planar_acceleration<=0.||!finite(conservative_planar_scale)||conservative_planar_scale<=0.)return 0.;
  const auto hi=std::upper_bound(arcs.begin(),arcs.end(),arc);
  if(hi==arcs.begin())return limits.front();
  if(hi==arcs.end())return limits.back();
  const auto i=static_cast<std::size_t>(hi-arcs.begin());
  // Never interpolate above the sampled local cap. The right hand braking
  // cone is the amount that can still be shed before the next sample.
  return std::min(limits[i-1],std::sqrt(limits[i]*limits[i]+2.*planar_acceleration*
    conservative_planar_scale*std::max(0.,arcs[i]-arc)));
}
inline std::optional<double> normalForwardStep(double desired,double previous,double speed_cap,
    double acceleration,double dt) {
  if(!finite(desired)||!finite(previous)||!finite(speed_cap)||!finite(acceleration)||!finite(dt)||
     previous<0.||speed_cap<0.||acceleration<0.||dt<0.||dt>.25)return {};
  if(acceleration==0.)return previous==0.&&speed_cap==0.?std::optional<double>(0.):std::nullopt;
  const double lower=std::max(0.,previous-acceleration*dt);
  if(lower>speed_cap+1e-12)return {};
  return std::clamp(std::clamp(desired,0.,speed_cap),lower,previous+acceleration*dt);
}

struct Config
{
  std::string session_id, planning_frame{"d1max_loc_odom"}, base_frame{"d1max_loc_base_link"};
  std::string map_version_id, map_frame{"d1max_loc_map"};
  double max_speed{0.30}, max_yaw_rate{0.50};
  double max_acceleration{0.35}, max_yaw_acceleration{0.80};
  double task_timeout{0.75}, odom_timeout{0.40}, trajectory_timeout{5.0};
  double lookahead{0.8}, kp_position{0.8}, kp_yaw{1.5};
  double heading_threshold{0.6}, goal_tolerance{0.20}, position_freeze_distance{0.60};
  double goal_height_tolerance{0.15}, single_floor_max_height_change{0.25};
  bool require_versioned_identity{true};
  bool require_support_reference{false}; // mandatory in schema-3 execution node
  bool external_goal_completion{false};
  double join_limit{.0125}, recovery_span{.6};
  // Shared with BT/SDK from the physically accepted stationary record.
  // These thresholds do not alter the spline velocity join bound (.05 m/s).
  double stationary_linear_threshold_mps{.03}, stationary_angular_threshold_radps{.05};
  double stationary_reentry_duration_s{.6};
  unsigned stationary_minimum_samples{3};
  double projection_backtrack_m{.15}, projection_forward_m{.30}, projection_max_forward_m{1.0};
  double posterior_timeout{.40}, imu_timeout{.10}, max_extrapolation{.10};
  void validate() const
  {
    if (session_id.empty() || planning_frame.empty() || base_frame.empty() ||
        (require_versioned_identity && (map_version_id.empty() || map_frame.empty() ||
         planning_frame == map_frame)))
      throw std::invalid_argument("session and frames must be explicit");
    const auto bound = [](double value, double ceiling) {
      return finite(value) && value > 0.0 && value <= ceiling;
    };
    if (!bound(max_speed, HARD_PLANAR_SPEED) || !bound(max_yaw_rate, 1.0) ||
        !bound(max_acceleration, 0.8) || !bound(max_yaw_acceleration, 1.5) ||
        !bound(task_timeout, 1.0) || !bound(odom_timeout, 0.5) ||
        !bound(trajectory_timeout, 10.0) || !bound(lookahead, 2.0) ||
        !bound(kp_position, 3.0) || !bound(kp_yaw, 3.0) ||
        !bound(heading_threshold, 1.0) || !bound(goal_tolerance, 0.3) ||
        !bound(goal_height_tolerance, 0.20) ||
        !bound(single_floor_max_height_change, 0.30) ||
        goal_height_tolerance > single_floor_max_height_change ||
        !bound(position_freeze_distance, 1.0) || !bound(projection_backtrack_m, .15) ||
        !bound(projection_forward_m, .30) || !bound(projection_max_forward_m, 1.0) ||
        projection_forward_m > projection_max_forward_m || !bound(posterior_timeout,.40) ||
        !bound(imu_timeout,.10) || !bound(max_extrapolation,.10) ||
        !bound(join_limit,.0125) || !bound(recovery_span,1.) ||
        !bound(stationary_linear_threshold_mps,.05) ||
        !bound(stationary_angular_threshold_radps,.1) ||
        !finite(stationary_reentry_duration_s) || stationary_reentry_duration_s<.6 ||
        stationary_reentry_duration_s>5. || stationary_minimum_samples<3 ||
        stationary_minimum_samples>512)
      throw std::invalid_argument("unsafe tracker configuration");
  }
};

struct ControlIdentity {
  std::uint32_t schema_version{0};
  std::string task_id, route_id, route_hash, segment_id, map_version_id, anchor_id;
  std::uint64_t anchor_revision{0}, context_sequence{0}, localization_epoch{0},map_geometry_revision{0};
  std::string localization_seed_id;
  bool valid() const {
    return schema_version==2 && !task_id.empty() && !route_id.empty() && route_hash.size()==64 &&
      std::all_of(route_hash.begin(),route_hash.end(),[](char c){return (c>='0' && c<='9') || (c>='a' && c<='f');}) &&
      !segment_id.empty() && !map_version_id.empty() && !anchor_id.empty() && anchor_revision>0 &&
      context_sequence>0 && localization_epoch>0 && map_geometry_revision>0 && !localization_seed_id.empty();
  }
  bool operator==(const ControlIdentity& o) const {
    return schema_version==o.schema_version && task_id==o.task_id && route_id==o.route_id &&
      route_hash==o.route_hash && segment_id==o.segment_id && map_version_id==o.map_version_id &&
      anchor_id==o.anchor_id && anchor_revision==o.anchor_revision && context_sequence==o.context_sequence &&
      localization_epoch==o.localization_epoch && localization_seed_id==o.localization_seed_id&&
      map_geometry_revision==o.map_geometry_revision;
  }
  bool sameTask(const ControlIdentity& o) const {
    return schema_version==o.schema_version && task_id==o.task_id && route_id==o.route_id &&
      route_hash==o.route_hash && map_version_id==o.map_version_id &&
      localization_epoch==o.localization_epoch && localization_seed_id==o.localization_seed_id;
  }
};

struct SupportEvidence {
  ControlIdentity identity;
  std::uint64_t generation{0};
  std::string id, hash, map_hash, floor_id, segment_kind, mode, frame;
  double source_stamp{0.}, xy_radius{0.}, body_height{0.}, max_slope{0.}, max_step{0.};
  bool verified{false};
  std::vector<Eigen::Vector3d> ground;
};
inline bool sameSupportEvidenceGeometry(const SupportEvidence& a,const SupportEvidence& b) {
  if(&a==&b)return true;
  if(!(a.identity==b.identity)||a.generation!=b.generation||a.id!=b.id||a.hash!=b.hash||a.map_hash!=b.map_hash||
     a.floor_id!=b.floor_id||a.segment_kind!=b.segment_kind||a.mode!=b.mode||a.frame!=b.frame||
     a.verified!=b.verified||a.xy_radius!=b.xy_radius||a.body_height!=b.body_height||
     a.max_slope!=b.max_slope||a.max_step!=b.max_step||a.ground.size()!=b.ground.size())return false;
  for(std::size_t i=0;i<a.ground.size();++i)if(!(a.ground[i].array()==b.ground[i].array()).all())return false;
  return true;
}
struct SupportIndex {
  std::unordered_map<std::uint64_t,std::vector<Eigen::Vector3d>> bins;
  bool valid{true};
  static std::uint64_t key(std::int32_t x,std::int32_t y) {
    return (std::uint64_t(static_cast<std::uint32_t>(x))<<32)|static_cast<std::uint32_t>(y);
  }
  explicit SupportIndex(const std::vector<Eigen::Vector3d>& points) {
    for(const auto& p:points) {
      if(!p.allFinite()||p.cwiseAbs().maxCoeff()>1e6){valid=false;return;}
      bins[key(static_cast<std::int32_t>(std::floor(p.x()/.1)),static_cast<std::int32_t>(std::floor(p.y()/.1)))].push_back(p);
    }
  }
};

struct Task
{
  std::string session_id, frame_id;
  std::uint64_t generation{0};
  bool active{false};
  double issued_at{0.0};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  ControlIdentity identity;
};

struct PreparedGeometry;
struct Trajectory
{
  std::string session_id, frame_id, point_reference;
  std::uint64_t generation{0};
  std::int64_t id{0};
  double start_time{0.0};
  int order{3};
  std::vector<Eigen::Vector3d> points;
  std::vector<double> knots;
  ControlIdentity identity;
  // Certified join into the original (untrimmed) curve. This is measured
  // evidence, not elapsed wall time and not a shifted trajectory start clock.
  double valid_start_time{0.}, valid_start_arc_length{0.}, join_source_stamp{0.};
  std::int64_t join_source_stamp_ns{0}; // Original wire time, diagnostics only.
  // A fresh entry observation is separate from the worker's immutable source.
  // It is made only under a matching fresh whole-curve proof, never by copying
  // the old sample and changing its timestamp.
  bool entry_reobserved{false};
  std::int64_t original_join_source_stamp_ns{0};
  Eigen::Vector3d join_position{Eigen::Vector3d::Zero()}, join_velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond join_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d join_acceleration{Eigen::Vector3d::Zero()};
  bool join_acceleration_valid{false};
  // Immutable worker preparation; never contains measured control state or a
  // motion permit. The control owner still performs the actual entry check.
  std::shared_ptr<const PreparedGeometry> prepared;
};

struct Odom
{
  std::string frame_id, child_frame_id;
  double stamp{0.0}, yaw{0.0};
  // Preserve the original transport source for exact evidence cloning. The
  // controller still uses seconds for arithmetic, never to re-date this stamp.
  std::int64_t source_stamp_ns{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double planar_speed{0.0};
  std::uint64_t localization_epoch{0};
  std::string localization_seed_id;
  Eigen::Vector3d velocity_in_frame{Eigen::Vector3d::Zero()}, angular_velocity_in_frame{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  std::uint32_t schema_version{0};
  std::string session_id, map_version_id;
  double posterior_stamp{0.}, imu_stamp{0.}, extrapolation_sec{0.};
};

// Exact admission predicates with separate diagnostic names. These helpers
// neither enlarge source leases nor alter curve/collision admission policy.
inline const char* sourceEvidenceFailure(const Odom& o,double now,const Config& c) {
  if(!(o.posterior_stamp>0.))return "body_posterior_source_missing";
  if(!(o.imu_stamp>0.))return "body_imu_source_missing";
  if(!finite(o.extrapolation_sec))return "body_extrapolation_nonfinite";
  if(o.extrapolation_sec<0.)return "body_extrapolation_negative";
  if(o.extrapolation_sec>c.max_extrapolation)return "body_extrapolation_limit";
  if(!(o.posterior_stamp<=o.stamp))return "body_posterior_after_state";
  if(!(o.imu_stamp<=o.stamp))return "body_imu_after_state";
  if(!(std::abs(o.stamp-o.imu_stamp-o.extrapolation_sec)<1e-5))return "body_extrapolation_source_mismatch";
  if(!finite(now)||!finite(o.posterior_stamp)||now-o.posterior_stamp<-.02||now-o.posterior_stamp>c.posterior_timeout)
    return "body_posterior_not_fresh";
  if(!finite(o.imu_stamp)||now-o.imu_stamp<-.02||now-o.imu_stamp>c.imu_timeout)return "body_imu_not_fresh";
  return nullptr;
}
inline const char* joinEvidenceFailure(const Trajectory& t,bool have_body,const Odom& o,
                                      double now,double duration,const Config& c) {
  if(!have_body)return "join_body_missing";
  if(!finite(now)||!finite(o.stamp)||now-o.stamp<-.02||now-o.stamp>c.odom_timeout)return "join_body_not_fresh";
  if(const auto why=sourceEvidenceFailure(o,now,c))return why;
  if(!finite(t.valid_start_time))return "join_curve_time_nonfinite";
  if(t.valid_start_time<0.||t.valid_start_time>duration)return "join_curve_time_out_of_range";
  if(!finite(t.valid_start_arc_length))return "join_arc_nonfinite";
  if(t.valid_start_arc_length<0.)return "join_arc_negative";
  if(!finite(t.join_source_stamp))return "join_source_nonfinite";
  if(t.join_source_stamp<=0.)return "join_source_missing";
  if(now-t.join_source_stamp<0.)return "join_source_in_future";
  if(now-t.join_source_stamp>c.odom_timeout)return "join_source_expired";
  if(t.join_source_stamp>o.stamp+1e-6)return "join_source_after_body";
  if(!t.join_position.allFinite())return "join_position_nonfinite";
  if(!t.join_velocity.allFinite())return "join_velocity_nonfinite";
  if(!t.join_orientation.coeffs().allFinite())return "join_orientation_nonfinite";
  if(std::abs(t.join_orientation.norm()-1.)>.001)return "join_orientation_not_unit";
  return nullptr;
}
struct JoinDiagnostic {
  bool body_present{false};
  std::int64_t trajectory_id{-1},body_source_stamp_ns{0},join_source_stamp_ns{0};
  bool entry_reobserved{false};
  std::int64_t original_join_source_stamp_ns{0};
  double now{0.},body_stamp{0.},posterior_stamp{0.},imu_stamp{0.},extrapolation_sec{0.},
    join_stamp{0.},curve_time{0.},curve_duration{0.},arc{0.};
  std::string reason;
};

struct EntryCurveCache {
  std::shared_ptr<const scan_planner::UniformBspline> curve,velocity;
  std::vector<double> times,arcs;
  std::vector<Eigen::Vector3d> points;
  double duration{0.};
  static std::optional<EntryCurveCache> build(const Trajectory& t,
      const std::function<bool()>& allowed=[] {return true;},std::string* failure=nullptr) {
    if(failure)*failure="trajectory_evaluation_nonfinite";
    if(t.order!=3||t.points.size()<4||t.points.size()>10000||
       t.knots.size()!=t.points.size()+static_cast<std::size_t>(t.order)+1)return {};
    for(const auto& p:t.points)if(!p.allFinite())return {};
    for(std::size_t i=0;i<t.knots.size();++i)if(!finite(t.knots[i])||
      (i&&t.knots[i]-t.knots[i-1]<=1e-9))return {};
    EntryCurveCache c;c.duration=t.knots[t.points.size()]-t.knots[t.order];
    if(!finite(c.duration)||c.duration<=.01||c.duration>120.)return {};
    Eigen::MatrixXd p(3,t.points.size());Eigen::VectorXd k(t.knots.size());
    for(std::size_t i=0;i<t.points.size();++i)p.col(i)=t.points[i];
    for(std::size_t i=0;i<t.knots.size();++i)k(i)=t.knots[i];
    auto curve=std::make_shared<scan_planner::UniformBspline>(p,t.order,.1);curve->setKnot(k);
    c.velocity=std::make_shared<const scan_planner::UniformBspline>(curve->getDerivative());
    c.curve=std::move(curve);
    const unsigned n=std::clamp(static_cast<unsigned>(std::ceil(c.duration/.02)),40u,6000u);
    c.times.resize(n+1);c.arcs.resize(n+1);c.points.reserve(n+1);
    for(unsigned i=0;i<=n;++i) {
      if(!allowed())return {};
      c.times[i]=c.duration*static_cast<double>(i)/n;
      c.points.push_back(c.curve->evaluateDeBoorT(c.times[i]));
      if(!c.points.back().allFinite())return {};
      if(i)c.arcs[i]=c.arcs[i-1]+(c.points[i]-c.points[i-1]).norm();
    }
    if(!finite(c.arcs.back())||c.arcs.back()<1e-6) {
      if(failure)*failure="trajectory_arc_degenerate";
      return {};
    }
    return c;
  }
  std::optional<Trajectory> observe(const Trajectory& original,const Odom& body,const Config& config,
      double now,double proof_body_stamp,double seed_time,double seed_arc,std::string& reason)const {
    const auto reject=[&](const char* why)->std::optional<Trajectory>{reason=why;return {};};
    if(!finite(now)||!finite(body.stamp)||now-body.stamp<-.02||now-body.stamp>.1)
      return reject("entry_body_not_fresh");
    if(const auto why=sourceEvidenceFailure(body,now,config))return reject(why);
    if(body.localization_epoch!=original.identity.localization_epoch||
       body.localization_seed_id!=original.identity.localization_seed_id||
       body.session_id!=original.session_id||body.map_version_id!=original.identity.map_version_id||
       body.frame_id!=original.frame_id)return reject("entry_body_identity_mismatch");
    if(!finite(proof_body_stamp)||proof_body_stamp<=0.||proof_body_stamp>body.stamp+1e-6||
       now-proof_body_stamp>.4)return reject("entry_body_precedes_or_exceeds_proof");
    if(!finite(seed_time)||seed_time<0.||seed_time>duration||!finite(seed_arc)||seed_arc<0.||
       std::abs(curveArcAt(times,arcs,seed_time)-seed_arc)>.01)return reject("entry_proof_curve_domain_invalid");
    const double travel=std::min(config.projection_max_forward_m,
      config.max_speed*std::max(0.,body.stamp-proof_body_stamp)+config.join_limit);
    const auto p=projectCurveAdmission(*curve,times,arcs,points,body.position,seed_time,seed_arc,
      std::min(config.projection_backtrack_m,travel),travel,config.join_limit);
    if(!p)return reject("entry_measured_body_not_on_candidate");
    if((velocity->evaluateDeBoorT(p->time)-body.velocity_in_frame).norm()>.05)
      return reject("entry_measured_velocity_off_candidate");
    auto t=original;t.valid_start_time=p->time;t.valid_start_arc_length=p->arc;
    t.entry_reobserved=true;t.original_join_source_stamp_ns=original.join_source_stamp_ns;
    t.join_source_stamp=body.stamp;t.join_source_stamp_ns=body.source_stamp_ns;
    t.join_position=body.position;t.join_velocity=body.velocity_in_frame;t.join_orientation=body.orientation;
    // No trustworthy acceleration measurement is available in LocalState.
    t.join_acceleration_valid=false;t.join_acceleration.setZero();
    reason.clear();return t;
  }
};

inline bool supportEvidenceValid(const SupportEvidence& s,const SupportIndex& index,
    const ControlIdentity& identity,std::uint64_t generation,const Config& config) {
  return index.valid&&s.verified&&s.identity==identity&&s.generation==generation&&
    !s.id.empty()&&s.hash.size()==64&&s.map_hash.size()==64&&s.floor_id=="floor1"&&
    s.segment_kind=="floor"&&s.mode=="general"&&s.frame==config.planning_frame&&
    finite(s.source_stamp)&&s.source_stamp>0.&&s.ground.size()>=2&&s.ground.size()<=20000&&
    finite(s.xy_radius)&&s.xy_radius>0.&&s.xy_radius<=.5&&finite(s.body_height)&&
    s.body_height>=.2&&s.body_height<=.8&&finite(s.max_slope)&&s.max_slope>0.&&
    s.max_slope<=.176&&finite(s.max_step)&&s.max_step>0.&&s.max_step<=.08;
}
inline bool pointSupported(const SupportEvidence& support,const SupportIndex& index,
    const Eigen::Vector3d& body,double height_tolerance) {
  if(!index.valid||!body.allFinite()||body.cwiseAbs().maxCoeff()>1e6)return false;
  double nearest=std::numeric_limits<double>::infinity(),ground_z=0.;
  Eigen::Vector3d anchor=Eigen::Vector3d::Zero();
  const int ix=static_cast<int>(std::floor(body.x()/.1)),iy=static_cast<int>(std::floor(body.y()/.1));
  const int span=static_cast<int>(std::ceil(2.*support.xy_radius/.1))+1;
  // Two passes over fixed bins; no per-control-tick heap allocation.
  for(int dx=-span;dx<=span;++dx)for(int dy=-span;dy<=span;++dy) {
    const auto found=index.bins.find(SupportIndex::key(ix+dx,iy+dy));
    if(found==index.bins.end())continue;
    for(const auto& p:found->second) {
      const double d=(body.head<2>()-p.head<2>()).norm();
      if(d<nearest){nearest=d;ground_z=p.z();anchor=p;}
    }
  }
  if(nearest>support.xy_radius)return false;
  for(int dx=-span;dx<=span;++dx)for(int dy=-span;dy<=span;++dy) {
    const auto found=index.bins.find(SupportIndex::key(ix+dx,iy+dy));
    if(found==index.bins.end())continue;
    for(const auto& p:found->second) {
      const double distance=(p.head<2>()-anchor.head<2>()).norm();
      if(distance>support.xy_radius||distance<1e-6)continue;
      if(std::abs(p.z()-anchor.z())>support.max_step+std::tan(support.max_slope)*distance)return false;
    }
  }
  return std::abs(body.z()-ground_z-support.body_height)<=height_tolerance;
}

struct PreparedGeometry {
  EntryCurveCache entry;
  std::shared_ptr<const SupportEvidence> support;
  std::shared_ptr<const SupportIndex> support_index;
  std::shared_ptr<const scan_planner::UniformBspline> acceleration;
  ControlIdentity identity;
  std::uint64_t generation{0};std::int64_t trajectory_id{-1};
  std::vector<double> speed_envelope;
  double planar_scale{1.};bool derivatives_valid{false};
  std::string failure;
  std::vector<Eigen::Vector3d> original_controls;
  std::vector<double> original_knots;
  std::string frame,point_reference;
  double max_speed{0.},max_acceleration{0.},max_yaw{0.},height_tolerance{0.};
  bool requires_support{false};
  bool matches(const Config& c,const Trajectory& t)const {
    if(!(identity==t.identity)||generation!=t.generation||trajectory_id!=t.id||frame!=t.frame_id||
       point_reference!=t.point_reference||max_speed!=c.max_speed||max_acceleration!=c.max_acceleration||
       max_yaw!=c.max_yaw_rate||height_tolerance!=c.goal_height_tolerance||requires_support!=c.require_support_reference||
       original_knots!=t.knots||original_controls.size()!=t.points.size())return false;
    for(std::size_t i=0;i<t.points.size();++i)if(!(original_controls[i].array()==t.points[i].array()).all())return false;
    return true;
  }
  static std::shared_ptr<const PreparedGeometry> build(const Config& config,const Trajectory& t,
      SupportEvidence support,const std::function<bool()>& allowed=[] {return true;}) {
    auto out=std::make_shared<PreparedGeometry>();
    out->identity=t.identity;out->generation=t.generation;out->trajectory_id=t.id;
    out->original_controls=t.points;out->original_knots=t.knots;out->frame=t.frame_id;out->point_reference=t.point_reference;
    out->max_speed=config.max_speed;out->max_acceleration=config.max_acceleration;out->max_yaw=config.max_yaw_rate;
    out->height_tolerance=config.goal_height_tolerance;out->requires_support=config.require_support_reference;
    auto entry=EntryCurveCache::build(t,allowed,&out->failure);
    if(!entry){if(!allowed())out->failure="preparation_cancelled";return out;}
    out->failure.clear();
    out->entry=std::move(*entry);
    auto velocity=*out->entry.velocity;
    auto acceleration=std::make_shared<scan_planner::UniformBspline>(velocity.getDerivative());
    out->derivatives_valid=curveDerivativeBound(velocity,config.max_speed)&&
      curveDerivativeBound(*acceleration,config.max_acceleration);
    out->acceleration=std::move(acceleration);
    if(config.require_support_reference) {
      out->support=std::make_shared<const SupportEvidence>(std::move(support));
      out->support_index=std::make_shared<const SupportIndex>(out->support->ground);
      if(!supportEvidenceValid(*out->support,*out->support_index,t.identity,t.generation,config)) {
        out->failure="support_invalid";return out;
      }
    }
    for(std::size_t i=0;i<out->entry.points.size();++i) {
      if(!allowed()){out->failure="preparation_cancelled";return out;}
      if(i){const auto tangent=out->entry.points[i]-out->entry.points[i-1];
        if(tangent.norm()>1e-10)out->planar_scale=std::min(out->planar_scale,spatialToPlanarScale(tangent));}
      if(config.require_support_reference&&!pointSupported(*out->support,*out->support_index,
          out->entry.points[i],config.goal_height_tolerance)){out->failure="curve_outside_support";return out;}
    }
    if(config.require_support_reference)out->planar_scale=std::min(out->planar_scale,
      spatialToPlanarScale(Eigen::Vector3d(1.,0.,std::tan(out->support->max_slope))));
    std::vector<double> limits;limits.reserve(out->entry.times.size());
    for(const auto time:out->entry.times) {
      if(!allowed()){out->failure="preparation_cancelled";return out;}
      limits.push_back(curvatureForwardLimit(out->entry.velocity->evaluateDeBoorT(time),
        out->acceleration->evaluateDeBoorT(time),config.max_yaw_rate,config.max_speed*out->planar_scale));
    }
    out->speed_envelope=brakingSpeedEnvelope(out->entry.arcs,limits,
      config.max_acceleration*out->planar_scale,out->planar_scale);
    if(out->speed_envelope.empty())out->failure="curvature_braking_envelope_invalid";
    return out;
  }
};

struct Progress {
  ControlIdentity identity;
  double source_stamp{0.}, curve_time{0.}, arc_length{0.}, s_committed{0.};
  bool valid{false}, holding{true};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.};
  Eigen::Vector3d velocity_in_frame{Eigen::Vector3d::Zero()}, angular_velocity_in_frame{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  std::string reason;
};

struct Output
{
  double forward{0.0}, yaw_rate{0.0};
  bool frozen{true}, finished{false};
  std::string reason{"idle"};
};

class TrackerCore
{
public:
  explicit TrackerCore(Config config) : config_(std::move(config)) { config_.validate(); }

  bool receiveTask(const Task &task, double ros_now, double received)
  {
    if (task.session_id != config_.session_id || task.generation < task_.generation ||
        task.generation == 0 || !finite(received) || !finite(ros_now)) return false;
    if (!task.active) {
      if (config_.require_versioned_identity && (!task.identity.valid() ||
          task.generation != task_.generation || !(task.identity == task_.identity))) return false;
      task_ = task;
      cancel("task_stopped");
      return true;
    }
    if (task.frame_id != config_.planning_frame || !finite(task.issued_at) ||
        task.issued_at <= 0.0 || task.issued_at > ros_now + 0.2 || !task.goal.allFinite() ||
        (config_.require_versioned_identity && (!task.identity.valid() ||
         task.identity.map_version_id != config_.map_version_id))) {
      cancel("invalid_task");
      return false;
    }
    if (task.generation == task_.generation) {
      // A terminal generation can never be revived by a delayed heartbeat.
      if (!active_) return false;
      if (task.issued_at != task_.issued_at || task.frame_id != task_.frame_id ||
          (task.goal - task_.goal).norm() > 1e-9 || !(task.identity == task_.identity)) {
        cancel("task_context_changed_without_generation");
        return false;
      }
    } else {
      if (ros_now - task.issued_at > config_.task_timeout) return false;
      task_ = task;
      trajectory_.reset();
      last_trajectory_id_ = -1;
      active_ = true;
      reason_ = "waiting_trajectory";
      finished_ = false;
      holding_=false; recovery_samples_=0;
      braking_reentry_required_=false;
      floor_anchor_valid_ = false;
      measured_arc_ = committed_arc_ = execution_time_ = 0.;
      last_projected_stamp_ = 0.;
      if (config_.require_versioned_identity && (!have_odom_ ||
          odom_.localization_epoch != task.identity.localization_epoch ||
          odom_.localization_seed_id != task.identity.localization_seed_id)) have_odom_ = false;
      last_output_ = Output{};
      turn_phase_=TurnPhase::Following;turn_first_engaged_=false;
      resetTurnStationaryWindow();turn_sample_stamp_=last_turn_feedback_stamp_=0.;
      aligned_entry_curve_id_=-1;aligned_entry_stamp_=0.;
      last_step_ = received;
    }
    task_received_ = received;
    return true;
  }

  bool receiveOdom(const Odom &odom, double ros_now, double received)
  {
    if(config_.require_versioned_identity&&(odom.schema_version!=2||
       odom.session_id!=config_.session_id||odom.map_version_id!=config_.map_version_id))return false;
    // Order the original observation BEFORE interpreting faults/context. An
    // old unusable/expired packet must not erase a newer measured state or
    // cancel a task admitted under a later localization epoch.
    const auto epoch_floor=std::max({odom_.localization_epoch,source_order_epoch_,
      active_?task_.identity.localization_epoch:std::uint64_t{0}});
    if(config_.require_versioned_identity&&odom.localization_epoch>0&&
       odom.localization_epoch<epoch_floor)return false;
    const bool expected_stream=!config_.require_versioned_identity||
      (odom.schema_version==2&&odom.session_id==config_.session_id&&
       odom.map_version_id==config_.map_version_id&&odom.localization_epoch>0&&!odom.localization_seed_id.empty());
    const bool new_epoch=config_.require_versioned_identity&&expected_stream&&
      odom.localization_epoch>source_order_epoch_;
    const auto source_ns=odom.source_stamp_ns>0?odom.source_stamp_ns:
      (finite(odom.stamp)&&odom.stamp>0.&&odom.stamp<1e10?
        static_cast<std::int64_t>(std::llround(odom.stamp*1e9)):std::int64_t{0});
    if(!new_epoch&&source_ns>0&&source_ns<=last_odom_source_ns_)return false;
    // Fresh in-range faults establish a barrier too. A delayed valid sample
    // cannot undo a later fault merely because that fault was not installed.
    if(source_ns>0&&finite(ros_now)&&odom.stamp<=ros_now+.02) {
      if(new_epoch) {last_odom_source_ns_=0;last_odom_source_stamp_=0.;source_order_epoch_=odom.localization_epoch;}
      last_odom_source_ns_=source_ns;
    }
    if(config_.require_versioned_identity&&finite(odom.stamp)&&finite(ros_now)&&odom.stamp>ros_now+.02) {
      have_odom_=false;hold("odometry_source_evidence_invalid",received);return false;
    }
    if(config_.require_versioned_identity&&(odom.localization_epoch==0||odom.localization_seed_id.empty())) {
      have_odom_=false;hold("odometry_identity_invalid",received);return false;
    }
    if (config_.require_versioned_identity &&
        (active_ && (odom.localization_epoch != task_.identity.localization_epoch ||
                    odom.localization_seed_id != task_.identity.localization_seed_id))) {
      have_odom_ = false;
      cancel("odometry_context_changed");
      return false;
    }
    if (config_.require_versioned_identity && (!sourceEvidenceFresh(odom,ros_now) ||
        odom.stamp > ros_now+.02)) {
      have_odom_ = false;
      hold("odometry_source_evidence_invalid",received);
      return false;
    }
    if (!finite(ros_now) || !finite(received) || !finite(odom.stamp) ||
        odom.frame_id != config_.planning_frame || odom.child_frame_id != config_.base_frame ||
        !odom.position.allFinite() || !finite(odom.yaw) || !finite(odom.planar_speed) ||
        !odom.velocity_in_frame.allFinite() || !odom.angular_velocity_in_frame.allFinite() ||
        !odom.orientation.coeffs().allFinite() || std::abs(odom.orientation.norm()-1.)>.001 ||
        odom.planar_speed > HARD_PLANAR_SPEED || odom.planar_speed < 0.0 ||
        ros_now - odom.stamp > config_.odom_timeout || odom.stamp > ros_now + 0.1) {
      have_odom_ = false;
      hold("invalid_odometry",received);
      return false;
    }
    // A delayed sample within the TTL is not a new physical observation. It
    // must neither rewind the control pose nor renew its receipt-time lease.
    if (odom.stamp <= last_odom_source_stamp_) return false;
    if (config_.require_versioned_identity && have_odom_ &&
        (odom.posterior_stamp < odom_.posterior_stamp || odom.imu_stamp < odom_.imu_stamp)) {
      have_odom_ = false;
      hold("odometry_evidence_reordered",received);
      return false;
    }
    odom_ = odom;
    last_odom_source_stamp_ = odom.stamp;
    odom_received_ = received;
    have_odom_ = true;
    if(holding_) {
      if(recovery_samples_==0) recovery_first_source_=odom.stamp;
      ++recovery_samples_;
      if(recovery_samples_>=3 && odom.stamp-recovery_first_source_>=config_.recovery_span) {
        holding_=false; reason_=trajectory_?"tracking":"waiting_trajectory";
      }
    }
    return true;
  }

  bool receiveTrajectory(const Trajectory &trajectory, double ros_now, double received)
  {
    recordJoinDiagnostic(trajectory,ros_now,0.,{});
    // Unrelated or obsolete context is not allowed to replace a current plan.
    if (!active_ || trajectory.session_id != config_.session_id ||
        trajectory.generation != task_.generation || trajectory.id <= last_trajectory_id_)
      return candidateReject("stale_or_foreign_trajectory");
    // Unrelated anchors/segments and old solves cannot replace an accepted
    // curve. An explicit task/segment handoff must establish new ownership.
    if (config_.require_versioned_identity &&
        (!trajectory.identity.valid() || !(trajectory.identity == task_.identity) ||
         trajectory.point_reference != "body_center")) return candidateReject("identity_mismatch");
    if (trajectory_ && trajectory.start_time < trajectory_start_) return candidateReject("older_start_time");
    // A rejected candidate is not a withdrawal of the independently validated
    // incumbent. Explicit cancellation/proof expiry are separate channels.
    // Distinguish input format, source age and evaluator failures without
    // changing any admission predicate or withdrawing the incumbent. A generic
    // "invalid" reason previously hid an expired pending candidate as geometry.
    if (!finite(ros_now) || !finite(received) || !finite(trajectory.start_time))
      return candidateReject("trajectory_clock_nonfinite");
    if (trajectory.frame_id != config_.planning_frame) return candidateReject("trajectory_frame_mismatch");
    if (trajectory.start_time + 1e-6 < task_.issued_at) return candidateReject("trajectory_predates_task");
    if (trajectory.start_time > ros_now + 0.2) return candidateReject("trajectory_start_in_future");
    if (ros_now - trajectory.start_time > config_.trajectory_timeout) return candidateReject("trajectory_start_expired");
    if (trajectory.order != 3) return candidateReject("trajectory_order_unsupported");
    if (trajectory.points.size() < 4 || trajectory.points.size() > 10000)
      return candidateReject("trajectory_control_point_count");
    if (trajectory.knots.size() != trajectory.points.size() + trajectory.order + 1)
      return candidateReject("trajectory_knot_count");
    for (const auto &point : trajectory.points)
      if (!point.allFinite()) return candidateReject("trajectory_control_point_nonfinite");
    // The convex hull of the control points bounds the entire cubic's height.
    // This is a single-floor controller, not an automatic stair controller.
    double min_z = trajectory.points.front().z(), max_z = min_z;
    for (const auto &point : trajectory.points) {
      min_z = std::min(min_z, point.z());
      max_z = std::max(max_z, point.z());
    }
    const double anchor = floor_anchor_valid_ ? floor_anchor_z_ : task_.goal.z();
    if (!config_.require_support_reference && (max_z - min_z > config_.single_floor_max_height_change ||
        min_z < anchor - config_.single_floor_max_height_change ||
        max_z > anchor + config_.single_floor_max_height_change)) {
      candidate_reason_="trajectory_outside_single_floor_envelope";
      return false;
    }
    // SCAN emits strictly increasing, non-clamped knots. Reject repeated knots
    // before calling the vendor evaluator, which divides by their differences.
    for (std::size_t i = 0; i < trajectory.knots.size(); ++i) {
      if (!finite(trajectory.knots[i])) return candidateReject("trajectory_knot_nonfinite");
      if (i && trajectory.knots[i] - trajectory.knots[i - 1] <= 1e-9)
        return candidateReject("trajectory_knots_not_increasing");
    }
    const double duration = trajectory.knots[trajectory.points.size()] -
                            trajectory.knots[trajectory.order];
    if (!finite(duration)) return candidateReject("trajectory_duration_nonfinite");
    if (duration <= 0.01 || duration > 120.0) return candidateReject("trajectory_duration_out_of_range");
    if (ros_now - trajectory.start_time > duration) return candidateReject("trajectory_duration_elapsed");
    auto prepared=trajectory.prepared;
    if(prepared&&!prepared->matches(config_,trajectory))return candidateReject("prepared_geometry_identity_mismatch");
    if(!prepared)prepared=PreparedGeometry::build(config_,trajectory,support_?*support_:SupportEvidence{});
    if(!prepared->failure.empty())return candidateReject(prepared->failure.c_str());
    const auto& arc=prepared->entry.arcs;
    const auto& times=prepared->entry.times;
    const auto& samples=prepared->entry.points;
    const auto& speed_envelope=prepared->speed_envelope;
    const auto curve=prepared->entry.curve,velocity=prepared->entry.velocity;
    const double planar_scale=prepared->planar_scale;
    double initial_time=0., initial_arc=0.;
    if (config_.require_versioned_identity) {
      const double join_time=trajectory.valid_start_time;
      const auto acceleration=prepared->acceleration;
      join_diagnostic_.curve_duration=duration;
      if(const auto why=joinEvidenceFailure(trajectory,have_odom_,odom_,ros_now,duration,config_))return candidateReject(why);
      if ((curve->evaluateDeBoorT(join_time)-trajectory.join_position).norm()>config_.join_limit)
        return candidateReject("join_position_off_curve");
      if ((velocity->evaluateDeBoorT(join_time)-trajectory.join_velocity).norm()>.05)
        return candidateReject("join_velocity_off_curve");
      if (!prepared->derivatives_valid) return candidateReject("curve_derivative_bound");
      if (trajectory.join_acceleration_valid && (!trajectory.join_acceleration.allFinite() ||
          (acceleration->evaluateDeBoorT(join_time)-trajectory.join_acceleration).norm()>.10))
        return candidateReject("join_acceleration_off_curve");
      const double seed_arc=curveArcAt(times,arc,join_time);
      // Both ends integrate the same original curve; discretization error is
      // bounded, not an opportunity to skip to an unrelated branch.
      if (std::abs(seed_arc-trajectory.valid_start_arc_length)>.01) return candidateReject("join_arc_mismatch");
      const double source_dt=std::max(0.,odom_.stamp-trajectory.join_source_stamp);
      const double travel=std::min(config_.projection_max_forward_m,
          config_.max_speed*source_dt+config_.join_limit);
      const auto projected=projectCurveAdmission(*curve,times,arc,samples,odom_.position,
          join_time,seed_arc,std::min(config_.projection_backtrack_m,travel),travel,config_.join_limit);
      if (!projected) return candidateReject("measured_body_not_on_candidate");
      if ((velocity->evaluateDeBoorT(projected->time)-odom_.velocity_in_frame).norm()>.05)
        return candidateReject("measured_velocity_off_candidate");
      initial_time=projected->time; initial_arc=projected->arc;
    }
    // Strictly preserve the limiter history. Only the reachable limit at the
    // actual entry matters, not the minimum over unrelated future geometry.
    const double entry_limit=brakingEnvelopeAt(arc,speed_envelope,initial_arc,
      config_.max_acceleration*planar_scale,planar_scale);
    if(last_output_.forward>entry_limit+1e-12)
      return candidateReject("curvature_braking_limit_below_current_command");
    trajectory_ = std::move(curve);
    velocity_ = std::move(velocity);
    prepared_geometry_=std::move(prepared);
    curve_identity_ = trajectory.identity;
    trajectory_received_ = received;
    trajectory_start_ = trajectory.start_time;
    duration_ = duration;
    trajectory_min_z_ = min_z;
    trajectory_max_z_ = max_z;
    planar_scale_ = planar_scale;
    if(turn_phase_==TurnPhase::WaitingEntry) {
      // A genuinely admitted replacement may try its new entry, but does not
      // jump directly from a blocked old entry into forward motion.
      turn_phase_=TurnPhase::Decelerating;resetTurnStationaryWindow();turn_sample_stamp_=turnSourceStamp();
      aligned_entry_curve_id_=-1;aligned_entry_stamp_=0.;
    }
    // Enter the original spline at the measured body projection, which can
    // be nonzero after asynchronous solving/delivery. Do not restart at zero
    // or extrapolate a position from elapsed wall time.
    // Its progress will be measured spatially in step(), within a bounded
    // forward window. Original start/receipt times remain the lease clocks.
    execution_time_ = initial_time;
    measured_arc_ = initial_arc;
    // This is the LOCAL curve high water, scoped by trajectory ID. It is not
    // global-route progress: obstacle detours have a different arc length.
    committed_arc_ = initial_arc;
    last_projected_stamp_ = config_.require_versioned_identity ? odom_.stamp : 0.;
    last_projected_position_ = odom_.position;
    last_trajectory_id_ = trajectory.id;
    local_endpoint_waiting_=false;
    braking_reentry_required_=false;
    reason_ = "tracking";
    candidate_reason_.clear();
    return true;
  }

  // The typed execution wrapper must first match this to one of OUR exact
  // nonzero demands and a fresh native occupied (not unknown/stale) proof.
  // This requests a control maneuver only; it grants no collision authority.
  bool notifyBlockedForwardTurn(std::int64_t trajectory_id,double demand_source_stamp,
      double now,double forward,double yaw_rate) {
    const bool in_place=turn_phase_==TurnPhase::Aligning;
    if(!active_||!trajectory_||!have_odom_||trajectory_id!=last_trajectory_id_||
       !finite(forward)||forward<0.||(forward<=1e-6&&!in_place)||!finite(yaw_rate)||std::abs(yaw_rate)<=1e-6||
       !fresh(now,demand_source_stamp,.1)||demand_source_stamp<=last_turn_feedback_stamp_||
       !fresh(now,odom_.stamp,.1,.02)||turn_phase_==TurnPhase::WaitingEntry)return false;
    last_turn_feedback_stamp_=demand_source_stamp;
    turn_first_engaged_=true;
    if(in_place&&forward<=1e-6) {
      // The in-place turn toward this curve itself collides. Re-aligning would
      // repeat the same rejected rotation; native must choose another entry.
      turn_phase_=TurnPhase::WaitingEntry;resetTurnStationaryWindow();
      last_output_=Output{};last_output_.frozen=true;
      reason_="waiting_executable_entry";return true;
    }
    if(aligned_entry_curve_id_==trajectory_id&&demand_source_stamp>=aligned_entry_stamp_&&
       (odom_.position-aligned_entry_position_).norm()<.05) {
      // Alignment is not an executable-entry proof. If an actual forward
      // attempt still collided after alignment, do not cycle zero-yaw align
      // and the same rejected forward command. Native must replace this entry.
      turn_phase_=TurnPhase::WaitingEntry;resetTurnStationaryWindow();
      last_output_=Output{};last_output_.frozen=true;
      reason_="waiting_executable_entry";return true;
    }
    if(turn_phase_==TurnPhase::Following) {
      turn_phase_=TurnPhase::Decelerating;resetTurnStationaryWindow();turn_sample_stamp_=turnSourceStamp();
    }
    return true;
  }
  const char* maneuverPhase()const {
    return turn_phase_==TurnPhase::Decelerating?"decelerating_for_turn":
      turn_phase_==TurnPhase::Aligning?"aligning_to_curve":
      turn_phase_==TurnPhase::WaitingEntry?"waiting_executable_entry":"following";
  }

  // The formal execution gate may supply a fresh, exact-curve collision lease.
  // This changes no curve timestamp/progress and never relaxes body freshness.
  // Legacy callers have no external proof and retain publication-age timeout.
  Output step(double ros_now, double received, bool fresh_external_curve_lease=false)
  {
    // Idle has no execution clock to guard. Establish a baseline instead of
    // reporting a huge "clock jump" on the first timer after process startup.
    if (!active_ && finite(ros_now) && finite(received)) {
      last_step_ = received;
      last_ros_time_ = ros_now;
      return stop();
    }
    const double dt = received - last_step_;
    const double trajectory_dt = last_ros_time_ > 0.0 ? ros_now - last_ros_time_ : dt;
    last_step_ = received;
    if (!finite(received) || !finite(ros_now) || !finite(dt) || dt < 0.0 || dt > 0.25 ||
        !finite(trajectory_dt) || trajectory_dt < 0.0 || trajectory_dt > 0.25) {
      last_ros_time_ = ros_now;
      hold("clock_or_executor_discontinuity",received);
      return stop();
    }
    last_ros_time_ = ros_now;
    if (!active_) return stop();
    if (!fresh(received, task_received_, config_.task_timeout)) {
      cancel("task_heartbeat_stale");
      return stop();
    }
    if (!have_odom_ || !fresh(received, odom_received_, config_.odom_timeout) ||
        !fresh(ros_now, odom_.stamp, config_.odom_timeout, 0.1) ||
        (config_.require_versioned_identity && !sourceEvidenceFresh(odom_,ros_now))) {
      hold("odometry_stale",received);
      return stop();
    }
    if(holding_) return stop();
    if(config_.require_support_reference &&
       (!supportValid(task_.identity,task_.generation) || !supported(odom_.position))) {
      reason_="support_evidence_missing_or_body_outside_support"; return stop();
    }
    if (!config_.require_support_reference && !floor_anchor_valid_) {
      floor_anchor_z_ = odom_.position.z();
      floor_anchor_valid_ = true;
    }
    // Pin height to this generation's first fresh measured body pose. Replans
    // cannot walk this anchor up a staircase one small local segment at a time.
    if (!config_.require_support_reference && (std::abs(task_.goal.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        std::abs(odom_.position.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        (trajectory_ && (trajectory_min_z_ < floor_anchor_z_ - config_.single_floor_max_height_change ||
                         trajectory_max_z_ > floor_anchor_z_ + config_.single_floor_max_height_change)))) {
      cancel("task_outside_single_floor_envelope");
      return stop();
    }
    if ((task_.goal.head<2>() - odom_.position.head<2>()).norm() <= config_.goal_tolerance &&
        std::abs(task_.goal.z() - odom_.position.z()) <= config_.goal_height_tolerance) {
      if(config_.external_goal_completion) { reason_="goal_position_reached"; return stop(); }
      finished_ = true;
      cancel("goal_reached");
      return stop();
    }
    if (!trajectory_) return stop();
    if(turn_phase_==TurnPhase::WaitingEntry) {
      reason_="waiting_executable_entry";return stop();
    }
    if (trajectory_dt == 0.0 || ros_now < trajectory_start_) {
      reason_ = "waiting_trajectory_clock";
      return stop();
    }
    if (!fresh_external_curve_lease&&(!fresh(received, trajectory_received_, config_.trajectory_timeout) ||
        ros_now - trajectory_start_ > duration_ + config_.trajectory_timeout)) {
      invalidateTrajectory("trajectory_stale");
      return stop();
    }
    // Only a new measured body sample updates progress. XYZ bounded projection
    // stays within the admitted segment and permits small physical backtracking;
    // a separate high water mark must not hide the robot's actual position.
    if (odom_.stamp > last_projected_stamp_) {
      const double source_dt = last_projected_stamp_ > 0 ? odom_.stamp-last_projected_stamp_ : 0.;
      const bool first = last_projected_stamp_ <= 0.;
      if (first || (odom_.position-last_projected_position_).norm() > 1e-6) {
        if (!projectMeasured(std::min(config_.projection_max_forward_m,
          config_.projection_forward_m + odom_.planar_speed*std::max(0.,source_dt)))) {
          invalidateTrajectory("measured_projection_outside_window");
          return stop();
        }
        last_projected_position_ = odom_.position;
      }
      last_projected_stamp_ = odom_.stamp;
    }
    const auto pos = trajectory_->evaluateDeBoorT(execution_time_);
    const Eigen::Vector2d pos_error = pos.head<2>() - odom_.position.head<2>();
    if (pos_error.norm() > 2.0 || std::abs(pos.z() - odom_.position.z()) > 0.5) {
      invalidateTrajectory("tracking_error_outside_single_floor_envelope");
      return stop();
    }
    const double look_time = std::min(duration_, execution_time_ + config_.lookahead);
    const Eigen::Vector3d desired = trajectory_->evaluateDeBoorT(look_time);
    const Eigen::Vector3d velocity = velocity_->evaluateDeBoorT(look_time);
    const Eigen::Vector2d look_error = desired.head<2>() - odom_.position.head<2>();
    // The upstream adapter can translate sideways. Ours cannot: the complete
    // positional feedback must steer yaw, not be thrown away after projecting
    // velocity onto body X. This is essential on a straight, parallel offset.
    // At the endpoint remove feed-forward so it cannot pull us past the end.
    const Eigen::Vector2d world = config_.kp_position * look_error +
        (look_time < duration_ ? Eigen::Vector2d(velocity.head<2>()) : Eigen::Vector2d::Zero());
    if (!world.allFinite() || !pos.allFinite() || !desired.allFinite()) {
      invalidateTrajectory("nonfinite_controller_output");
      return stop();
    }
    const double desired_yaw = world.norm() > 1e-5 ?
        std::atan2(world.y(), world.x()) : odom_.yaw;
    const double yaw_error = angle(desired_yaw - odom_.yaw);
    Output out;
    out.reason = "tracking";
    out.frozen = std::abs(yaw_error) > config_.heading_threshold ||
                 pos_error.norm() > config_.position_freeze_distance;
    double forward = 0.0;
    const double planar_speed_limit=std::min(config_.max_speed*planar_scale_,
      brakingEnvelopeAt(arcTable(),speedEnvelope(),measured_arc_,
        config_.max_acceleration*planar_scale_,planar_scale_));
    const double planar_acceleration_limit=config_.max_acceleration*planar_scale_;
    if(braking_reentry_required_) {
      reason_="braking_envelope_reentry_required";return stop();
    }
    // A pose discontinuity or unmodelled physical overspeed can leave less
    // braking distance than the accepted curve's normal speed profile permits.
    // Never disguise an emergency stop as a normally slew-limited command.
    if(!normalForwardStep(0.,last_output_.forward,planar_speed_limit,
                          planar_acceleration_limit,dt)) {
      braking_reentry_required_=true;reason_="braking_envelope_reentry_required";return stop();
    }
    if (std::abs(yaw_error) <= config_.heading_threshold) {
      forward = std::clamp(std::cos(odom_.yaw) * world.x() + std::sin(odom_.yaw) * world.y(),
                           0.0, planar_speed_limit);
    }
    if (local_endpoint_waiting_ || (look_time >= duration_ && look_error.norm() <= config_.goal_tolerance)) {
      // A local endpoint is not task retirement. Preserve the applied curve
      // and measured progress while the next candidate is prepared. Native
      // must still validate its real connector/support and latest raw map;
      // only a matching writer ACK may replace this geometry.
      local_endpoint_waiting_=true;reason_="local_segment_finished_waiting_replan";
      Output waiting;waiting.reason=reason_;waiting.frozen=true;
      waiting.forward=std::max(0.,last_output_.forward-planar_acceleration_limit*dt);
      const double yaw_step=config_.max_yaw_acceleration*dt;
      waiting.yaw_rate=std::clamp(0.,last_output_.yaw_rate-yaw_step,last_output_.yaw_rate+yaw_step);
      last_output_=waiting;return waiting;
    }
    double yaw = std::clamp(config_.kp_yaw * yaw_error,
                                 -config_.max_yaw_rate, config_.max_yaw_rate);
    // Entered only by a verified occupied forward+turn request. The entry and
    // exit bands differ so replan/heading noise cannot alternate forward/yaw.
    if(turn_first_engaged_&&turn_phase_==TurnPhase::Following&&std::abs(yaw_error)>.20) {
      turn_phase_=TurnPhase::Decelerating;resetTurnStationaryWindow();turn_sample_stamp_=turnSourceStamp();
    }
    if(turn_phase_!=TurnPhase::Following) {
      forward=0.;out.frozen=true;
      if(turn_phase_==TurnPhase::Decelerating) {
        yaw=0.;out.reason="turn_first_decelerating";
        const bool stopped=std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm())<=
          config_.stationary_linear_threshold_mps&&
          std::abs(odom_.angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps&&
          last_output_.forward<=1e-9&&std::abs(last_output_.yaw_rate)<=1e-9;
        if(observeTurnStationary(stopped)) {
          turn_phase_=TurnPhase::Aligning;resetTurnStationaryWindow();
        }
      } else {
        out.reason="turn_first_aligning";
        if(std::abs(yaw_error)<=.10)yaw=0.;
        const bool aligned=std::abs(yaw_error)<=.10&&
          std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm())<=
            config_.stationary_linear_threshold_mps&&
          std::abs(odom_.angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps&&
          std::abs(last_output_.yaw_rate)<=1e-9;
        if(observeTurnStationary(aligned)) {
          turn_phase_=TurnPhase::Following;resetTurnStationaryWindow();
          aligned_entry_curve_id_=last_trajectory_id_;aligned_entry_stamp_=odom_.stamp;
          aligned_entry_position_=odom_.position;
        }
      }
    }
    const auto normal_forward=normalForwardStep(forward,last_output_.forward,planar_speed_limit,
      planar_acceleration_limit,dt);
    if(!normal_forward) {
      braking_reentry_required_=true;reason_="braking_envelope_reentry_required";return stop();
    }
    out.forward=*normal_forward;
    out.yaw_rate = std::clamp(yaw, last_output_.yaw_rate - config_.max_yaw_acceleration * dt,
                             last_output_.yaw_rate + config_.max_yaw_acceleration * dt);
    if (!finite(out.forward) || !finite(out.yaw_rate)) {
      invalidateTrajectory("nonfinite_controller_output");
      return stop();
    }
    last_output_ = out;
    reason_ = out.reason;
    return out;
  }

  void cancel(const std::string &reason) {
    active_ = false;
    turn_phase_=TurnPhase::Following;turn_first_engaged_=false;resetTurnStationaryWindow();
    aligned_entry_curve_id_=-1;aligned_entry_stamp_=0.;
    if (reason != "goal_reached") finished_ = false;
    invalidateTrajectory(reason);
  }
  void hold(const std::string& reason,double received) {
    (void)received;
    holding_=true; recovery_samples_=0; recovery_first_source_=0.; reason_=reason;
    resetTurnStationaryWindow();
    last_output_=Output{};
  }
  bool holding() const { return holding_; }
  Output align(double desired_yaw,double tolerance,double now,double receipt,bool fresh_external_curve_lease=false) {
    const double dt=receipt-last_step_;
    const double source_dt=now-last_ros_time_;
    const double previous_yaw_command=last_output_.yaw_rate;
    const auto guarded=step(now,receipt,fresh_external_curve_lease);
    if(!active_ || holding_ || !have_odom_ || !trajectory_ || dt<=0. || dt>.1 ||source_dt<=0.||source_dt>.1||
       !fresh(now,odom_.stamp,config_.odom_timeout,.02) || !sourceEvidenceFresh(odom_,now) ||
       (config_.require_support_reference && (!supportValid(task_.identity,task_.generation) || !supported(odom_.position))) ||
       !finite(desired_yaw) || !finite(tolerance) || tolerance<.01 || tolerance>.5) return stop();
    (void)guarded;
    Output out;
    out.reason="aligning";out.frozen=true;
    const double error=angle(desired_yaw-odom_.yaw);
    const double requested=std::abs(error)<=tolerance?0.:std::clamp(config_.kp_yaw*error,-config_.max_yaw_rate,config_.max_yaw_rate);
    out.yaw_rate=std::clamp(requested,previous_yaw_command-config_.max_yaw_acceleration*dt,
                          previous_yaw_command+config_.max_yaw_acceleration*dt);
    last_output_=out; reason_=out.reason; return out;
  }
  void refreshTaskLease(double receipt) { if(active_ && finite(receipt)) task_received_=receipt; }
  void suspendOutput(double now,double receipt,const std::string& reason,bool recovery) {
    resetTurnStationaryWindow();
    if(recovery)hold(reason,receipt);
    if(!recovery&&active_&&trajectory_&&have_odom_&&!holding_&&
       fresh(now,odom_.stamp,config_.odom_timeout,.02)&&sourceEvidenceFresh(odom_,now)&&
       (!config_.require_support_reference||(supportValid(task_.identity,task_.generation)&&supported(odom_.position)))&&
       odom_.stamp>last_projected_stamp_) {
      const double dt=last_projected_stamp_>0.?odom_.stamp-last_projected_stamp_:0.;
      if(projectMeasured(std::min(config_.projection_max_forward_m,
          config_.projection_forward_m+odom_.planar_speed*std::max(0.,dt)))) {
        last_projected_stamp_=odom_.stamp;last_projected_position_=odom_.position;
      } else hold("measured_projection_outside_window",receipt);
    }
    last_step_=receipt;last_ros_time_=now;last_output_=Output{};reason_=reason;
  }
  void setSupport(const SupportEvidence& support) {
    if(!support_index_||!support_||!sameSupportEvidenceGeometry(support,*support_))
      support_index_=std::make_shared<SupportIndex>(support.ground);
    support_=std::make_shared<const SupportEvidence>(support);
  }
  // Preparation is pure with respect to the current controller. At commit we
  // repeat admission against the latest body and retain the limiter history.
  bool admitRevision(const Task& task,const Trajectory& trajectory,const SupportEvidence& support,
                     double now,double receipt,bool commit) {
    if(active_ && !task.identity.sameTask(task_.identity)) return candidateReject("foreign_task");
    if(task.session_id!=config_.session_id || !task.active || !task.identity.valid() ||
       task.frame_id!=config_.planning_frame || !task.goal.allFinite() ||
       trajectory.generation!=task.generation || !(trajectory.identity==task.identity)) return candidateReject("task_identity_mismatch");
    TrackerCore trial(*this);
    if(!active_){trial.last_step_=receipt;trial.last_ros_time_=now;trial.holding_=false;}
    trial.task_=task; trial.active_=true; trial.task_received_=receipt;
    if(trajectory.prepared&&trajectory.prepared->support&&
       !sameSupportEvidenceGeometry(*trajectory.prepared->support,support))
      return candidateReject("prepared_support_geometry_mismatch");
    if(trajectory.prepared&&trajectory.prepared->support) {
      trial.support_=trajectory.prepared->support;
      trial.support_index_=trajectory.prepared->support_index;
    } else trial.setSupport(support);
    if(trajectory.generation!=task_.generation) trial.last_trajectory_id_=-1;
    if(!trial.receiveTrajectory(trajectory,now,receipt)) {
      candidate_reason_=trial.candidate_reason_.empty()?"candidate_rejected":trial.candidate_reason_;
      join_diagnostic_=trial.join_diagnostic_;
      return false;
    }
    if(commit) *this=std::move(trial); else candidate_reason_.clear();
    return true;
  }
  const Task& task() const { return task_; }
  const Odom& odometry() const { return odom_; }
  struct EntryBoundary {Eigen::Vector3d position,velocity;double time,position_error,velocity_error;};
  // Keep prepared geometry private, but refresh its boundary from the SAME
  // measured state and limiter history as the incumbent. No elapsed-wall-clock
  // position prediction and no spline rebuild on the 50 Hz control lane.
  std::optional<EntryBoundary> refreshPreparedState(const TrackerCore& current,double now,double receipt) {
    if(!trajectory_||!velocity_||!current.have_odom_||current.holding_||
       !fresh(now,current.odom_.stamp,.1,.02)||!sourceEvidenceFresh(current.odom_,now))return {};
    const double dt=std::max(0.,current.odom_.stamp-last_projected_stamp_);
    const double travel=std::min(config_.projection_max_forward_m,config_.max_speed*dt+config_.join_limit);
    const auto p=projectCurveAdmission(*trajectory_,timeTable(),arcTable(),pointTable(),current.odom_.position,
      execution_time_,measured_arc_,std::min(config_.projection_backtrack_m,travel),travel,config_.join_limit);
    if(!p)return {};
    const auto position=trajectory_->evaluateDeBoorT(p->time),velocity=velocity_->evaluateDeBoorT(p->time);
    const double pe=(position-current.odom_.position).norm(),ve=(velocity-current.odom_.velocity_in_frame).norm();
    if(pe>config_.join_limit||ve>.05)return {};
    odom_=current.odom_;have_odom_=current.have_odom_;odom_received_=current.odom_received_;
    last_odom_source_stamp_=current.last_odom_source_stamp_;last_output_=current.last_output_;
    last_step_=current.last_step_;last_ros_time_=current.last_ros_time_;
    holding_=current.holding_;recovery_samples_=current.recovery_samples_;recovery_first_source_=current.recovery_first_source_;
    execution_time_=p->time;measured_arc_=p->arc;committed_arc_=std::max(committed_arc_,p->arc);
    last_projected_stamp_=odom_.stamp;last_projected_position_=odom_.position;task_received_=receipt;
    return EntryBoundary{position,velocity,p->time,pe,ve};
  }
  void recordAppliedOutput(double forward,double yaw,double now,double receipt) {
    last_output_.forward=forward;last_output_.yaw_rate=yaw;last_step_=receipt;last_ros_time_=now;
  }
  // A writer-applied identity is a fact even if its ACK arrives late. Retire
  // old geometry without pretending the new curve has fresh admission proof.
  void recordAppliedHold(const Task& task,std::int64_t id,double receipt,const std::string& reason) {
    task_=task;active_=true;task_received_=receipt;last_trajectory_id_=id;
    trajectory_.reset();velocity_.reset();hold(reason,receipt);
  }
  bool active() const { return active_; }
  bool hasInstalledGeometry()const {return active_&&trajectory_&&velocity_&&curve_identity_==task_.identity;}
  bool requiresMeasuredReentry()const {return braking_reentry_required_;}
  std::uint64_t generation() const { return task_.generation; }
  std::int64_t trajectoryId() const { return last_trajectory_id_; }
  double progressTime() const { return execution_time_; }
  double progressArc() const { return measured_arc_; }
  double committedArc() const { return committed_arc_; }
  bool remainingProofCovers(double from,double measured,double arc,double to,double duration,double reverse) const {
    if(!active_||!trajectory_||timeTable().size()<2||!finite(from)||!finite(measured)||!finite(arc)||
       !finite(to)||!finite(duration)||!finite(reverse)||from<0.||from>measured||measured>to||
       std::abs(duration-duration_)>1e-6||std::abs(to-duration_)>1e-6||reverse<.15||reverse>.15+1e-9||
       execution_time_<from-1e-6||execution_time_>to+1e-6)return false;
    const auto arcAt=[&](double t) {
      const auto it=std::lower_bound(timeTable().begin(),timeTable().end(),t);
      if(it==timeTable().begin())return 0.;
      if(it==timeTable().end())return arcTable().back();
      const auto hi=static_cast<std::size_t>(it-timeTable().begin()),lo=hi-1;
      return arcTable()[lo]+(t-timeTable()[lo])/(timeTable()[hi]-timeTable()[lo])*(arcTable()[hi]-arcTable()[lo]);
    };
    const double measured_arc=arcAt(measured),from_arc=arcAt(from);
    return std::abs(arc-measured_arc)<=.01&&from_arc<=std::max(0.,measured_arc-reverse)+.001;
  }
  Progress progress(double ros_now) const {
    Progress p;
    p.identity=curve_identity_; p.source_stamp=odom_.stamp; p.curve_time=execution_time_;
    p.arc_length=progressArc(); p.s_committed=committed_arc_;
    p.position=odom_.position; p.yaw=odom_.yaw; p.reason=reason_;
    p.velocity_in_frame=odom_.velocity_in_frame; p.angular_velocity_in_frame=odom_.angular_velocity_in_frame;
    p.orientation=odom_.orientation;
    p.valid=active_ && !holding_ && trajectory_ && have_odom_ && curve_identity_.valid() &&
      fresh(ros_now,odom_.stamp,config_.odom_timeout,.1) &&
      (!config_.require_versioned_identity || sourceEvidenceFresh(odom_,ros_now));
    p.holding=last_output_.frozen;
    return p;
  }
  const std::string &reason() const { return reason_; }
  // Diagnostic only: why the most recent candidate admission failed.
  const std::string &candidateReason() const { return candidate_reason_; }
  const JoinDiagnostic& joinDiagnostic()const {return join_diagnostic_;}
  void recordJoinDiagnostic(const Trajectory& t,double now,double duration,const std::string& reason) {
    join_diagnostic_={have_odom_,t.id,odom_.source_stamp_ns,t.join_source_stamp_ns,
      t.entry_reobserved,t.original_join_source_stamp_ns,now,odom_.stamp,odom_.posterior_stamp,
      odom_.imu_stamp,odom_.extrapolation_sec,t.join_source_stamp,t.valid_start_time,duration,
      t.valid_start_arc_length,reason};
  }

private:
  bool candidateReject(const char* reason) { candidate_reason_=reason;join_diagnostic_.reason=reason;return false; }
  bool supportValid(const ControlIdentity& identity,std::uint64_t generation) const {
    return support_&&support_index_&&supportEvidenceValid(*support_,*support_index_,identity,generation,config_);
  }
  bool supported(const Eigen::Vector3d& body) const {
    return support_&&support_index_&&pointSupported(*support_,*support_index_,body,config_.goal_height_tolerance);
  }
  bool sourceEvidenceFresh(const Odom& odom, double now) const {
    return sourceEvidenceFailure(odom,now,config_)==nullptr;
  }
  bool projectMeasured(double forward_window) {
    const double from=std::max(0., committed_arc_-config_.projection_backtrack_m);
    const double to=std::min(arcTable().back(), measured_arc_+forward_window);
    double best_arc=measured_arc_, best_time=execution_time_;
    bool outside_window=false;
    double best_dist=(trajectory_->evaluateDeBoorT(execution_time_)-odom_.position).squaredNorm();
    auto lower=std::lower_bound(arcTable().begin(),arcTable().end(),from);
    std::size_t begin=lower==arcTable().begin()?0:static_cast<std::size_t>(lower-arcTable().begin()-1);
    for (std::size_t i=begin; i+1<arcTable().size() && arcTable()[i]<=to; ++i) {
      const double span=arcTable()[i+1]-arcTable()[i];
      if (span<1e-12) continue;
      const double lo=std::clamp((from-arcTable()[i])/span,0.,1.);
      const double hi=std::clamp((to-arcTable()[i])/span,0.,1.);
      const Eigen::Vector3d chord=pointTable()[i+1]-pointTable()[i];
      const double raw_f=(odom_.position-pointTable()[i]).dot(chord)/chord.squaredNorm();
      const double f=std::clamp(raw_f,lo,hi);
      const double distance=(pointTable()[i]+f*chord-odom_.position).squaredNorm();
      if(distance+1e-12<best_dist) {
        best_dist=distance; best_arc=arcTable()[i]+f*span;
        best_time=timeTable()[i]+f*(timeTable()[i+1]-timeTable()[i]);
        outside_window=(from>0. && best_arc<=from+1e-9 && (lo-raw_f)*span>.02) ||
          (to<arcTable().back() && best_arc>=to-1e-9 && (raw_f-hi)*span>.02);
      }
    }
    if (outside_window) return false;
    measured_arc_=best_arc; execution_time_=best_time;
    committed_arc_=std::max(committed_arc_,measured_arc_);
    return true;
  }
  static bool fresh(double now, double then, double limit, double future = 0.0) {
    return finite(now) && finite(then) && now - then >= -future && now - then <= limit;
  }
  void invalidateTrajectory(const std::string &reason) {
    resetTurnStationaryWindow();
    trajectory_.reset(); velocity_.reset(); reason_ = reason;
    last_output_ = Output{};
  }
  Output stop() {
    resetTurnStationaryWindow();
    last_output_ = Output{};
    last_output_.reason = reason_;
    last_output_.finished = finished_;
    return last_output_;
  }
  void resetTurnStationaryWindow() {turn_samples_=0;turn_stationary_first_source_=0.;}
  double turnSourceStamp()const {
    // Predicted state headers are not an independent measurement of rest.
    return config_.require_versioned_identity?std::min(odom_.posterior_stamp,odom_.imu_stamp):odom_.stamp;
  }
  bool observeTurnStationary(bool qualifies) {
    // Only distinct measured state sources count. Timer ticks, old packets and
    // a source gap outside the ordinary odom lease cannot establish rest.
    const double source=turnSourceStamp();
    // A currently trusted abnormal state clears rest even when one slower
    // evidence stream has not advanced. It must never preserve the old window.
    if(!qualifies) {
      resetTurnStationaryWindow();turn_sample_stamp_=std::max(turn_sample_stamp_,source);return false;
    }
    if(source<=turn_sample_stamp_)return false;
    if(source-turn_sample_stamp_>config_.odom_timeout)resetTurnStationaryWindow();
    turn_sample_stamp_=source;
    if(turn_samples_==0)turn_stationary_first_source_=source;
    ++turn_samples_;
    return turn_samples_>=config_.stationary_minimum_samples&&
      source-turn_stationary_first_source_+1e-9>=config_.stationary_reentry_duration_s;
  }
  Config config_;
  std::shared_ptr<const SupportEvidence> support_;
  std::shared_ptr<const SupportIndex> support_index_;
  Task task_;
  Odom odom_;
  bool active_{false}, finished_{false}, have_odom_{false};
  bool holding_{false};
  bool local_endpoint_waiting_{false};
  bool braking_reentry_required_{false};
  unsigned recovery_samples_{0};
  double recovery_first_source_{0.};
  std::string candidate_reason_;
  JoinDiagnostic join_diagnostic_;
  bool floor_anchor_valid_{false};
  double floor_anchor_z_{0.0}, trajectory_min_z_{0.0}, trajectory_max_z_{0.0};
  double task_received_{-1e10}, odom_received_{-1e10}, trajectory_received_{-1e10};
  double last_odom_source_stamp_{0.0};
  std::int64_t last_odom_source_ns_{0};
  std::uint64_t source_order_epoch_{0};
  double trajectory_start_{0.0}, duration_{0.0}, execution_time_{0.0};
  double planar_scale_{1.};

  enum class TurnPhase {Following,Decelerating,Aligning,WaitingEntry};
  TurnPhase turn_phase_{TurnPhase::Following};
  bool turn_first_engaged_{false};
  unsigned turn_samples_{0};
  double turn_sample_stamp_{0.},turn_stationary_first_source_{0.},last_turn_feedback_stamp_{0.};
  std::int64_t aligned_entry_curve_id_{-1};
  double aligned_entry_stamp_{0.};
  Eigen::Vector3d aligned_entry_position_{Eigen::Vector3d::Zero()};
  double measured_arc_{0.}, committed_arc_{0.}, last_projected_stamp_{0.};
  Eigen::Vector3d last_projected_position_{Eigen::Vector3d::Zero()};
  std::shared_ptr<const PreparedGeometry> prepared_geometry_;
  const std::vector<double>& arcTable() const {return prepared_geometry_->entry.arcs;}
  const std::vector<double>& timeTable() const {return prepared_geometry_->entry.times;}
  const std::vector<Eigen::Vector3d>& pointTable() const {return prepared_geometry_->entry.points;}
  const std::vector<double>& speedEnvelope() const {return prepared_geometry_->speed_envelope;}
  ControlIdentity curve_identity_;
  double last_step_{0.0}, last_ros_time_{0.0};
  std::int64_t last_trajectory_id_{-1};
  std::string reason_{"idle"};
  std::shared_ptr<const scan_planner::UniformBspline> trajectory_, velocity_;
  Output last_output_;
};
}  // namespace d1max_trajectory_tracker
