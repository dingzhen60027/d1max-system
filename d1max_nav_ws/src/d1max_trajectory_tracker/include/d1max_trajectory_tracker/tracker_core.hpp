#pragma once

// ROS-independent admission/control core. Curves and derivatives are built and
// admitted by SCAN; the immutable fixed-3D evaluator preserves its exact math.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bspline_opt/uniform_bspline.h>
#include <bspline_opt/spline_heading_contract.hpp>
#include <bspline_opt/near_curve_direction.hpp>
#include <bspline_opt/forward_entry_heading.hpp>
#include <bspline_opt/initial_motion_limits.hpp>
#include "d1max_trajectory_tracker/curve_admission.hpp"
#include "d1max_trajectory_tracker/fixed_bspline_sampler.hpp"
#include "d1max_trajectory_tracker/velocity_smoother.hpp"
#include "d1max_trajectory_tracker/forward_steering_controller.hpp"
#include "d1max_trajectory_tracker/stair_steering_preview.hpp"
#include "d1max_trajectory_tracker/response_command_limiter.hpp"
#include "d1max_trajectory_tracker/motion_command_conditioner.hpp"

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

// Holonomic translation has its own normal acceleration v^2*kappa. Body yaw
// is an independent output and cannot be used to infer sideslip or XY curvature.
inline double curvaturePlanarLimit(const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration,double max_acceleration,double nominal) {
  if(!velocity.allFinite()||!acceleration.allFinite()||!finite(max_acceleration)||
     max_acceleration<=0.||!finite(nominal)||nominal<=0.)return 0.;
  const double speed=velocity.head<2>().norm();
  if(speed<1e-5)return nominal;
  const double cross=std::abs(velocity.x()*acceleration.y()-velocity.y()*acceleration.x());
  return cross<=1e-12?nominal:std::min(nominal,std::sqrt(max_acceleration*speed*speed*speed/cross));
}

// Total planar norm and total vector slew, not separate per-axis limits. The
// caller must express both vectors in ONE frame before taking their difference.
// A newly tighter cap is admitted only if reachable this tick.
// If the straight slew toward the target misses the feasible speed disk, use
// radial braking; never clip afterwards and hide an acceleration violation.
inline std::optional<Eigen::Vector2d> normalPlanarStep(Eigen::Vector2d desired,
    const Eigen::Vector2d& previous,double speed_cap,double acceleration,double dt) {
  if(!desired.allFinite()||!previous.allFinite()||!finite(speed_cap)||!finite(acceleration)||
     !finite(dt)||speed_cap<0.||acceleration<0.||dt<0.||dt>.25)return {};
  const double step=acceleration*dt,old_speed=previous.norm();
  if(old_speed>speed_cap+step+1e-12)return {};
  if(desired.norm()>speed_cap)desired*=speed_cap/desired.norm();
  Eigen::Vector2d delta=desired-previous;
  if(delta.norm()>step)delta*=step/delta.norm();
  Eigen::Vector2d result=previous+delta;
  if(result.norm()>speed_cap+1e-12)result=previous*(speed_cap/old_speed);
  return result;
}

// Geometric translation weight, not a timed filter or an alignment state.
// Keep the author's full FF+P for small errors and its zero XY at/above the
// existing large-heading threshold. A C2 taper over the latter half removes
// the finite target jump at that threshold without changing certified yaw.
// A spatial error band, not a latch or a prediction of continued body motion.
// Small heading noise requests no correction; the SAME feedback law reaches
// full strength continuously at the upper angle, including across replans.
// Keep the old .04-rad curvature-noise scale as the centre of a C2 blend,
// rather than switching an entire bend's yaw feed-forward on in one tick.

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

// This is a construction-time component contract, not a tunable permission.
// Formal LocalFollower relies on Native SCAN's matching trajectory proof;
// legacy standalone callers retain their historical raw-PCD support policy.
enum class GeometryAuthority { LegacyTracker, NativeScan };

struct Config
{
  std::string session_id, planning_frame{"d1max_loc_odom"}, base_frame{"d1max_loc_base_link"};
  std::string map_version_id, map_frame{"d1max_loc_map"};
  // Explicit certificate contract. Legacy callers keep observed-free curves;
  // the mainline consumes Native SCAN's matching geometry verdict, without a
  // second follower/SDK occupancy sweep.
  std::string trajectory_geometry_policy{"observed_free"};
  std::string motion_model{"forward_yaw"}, holonomic_yaw_policy{"path_tangent"};
  // Navigation strategy, independent of the platform's holonomic capability.
  // Legacy parameter name for forward + continuous steering on the admitted
  // path (never sideways SDK drift), not a stop/fully-align/go state machine.
  bool align_then_forward{false};
  double max_speed{0.30}, max_yaw_rate{0.50};
  double max_acceleration{0.35}, max_yaw_acceleration{0.80};
  // Zero keeps saved/legacy controller profiles unchanged. These are command
  // derivatives, not promises about SDK fractions or physical body dynamics.
  double max_jerk{0.0}, max_yaw_jerk{0.0};
  // The same final conditioner accounts for the selected static actuator
  // response. These describe INPUT fractions, never measured body dynamics.
  CommandResponseCurve forward_response,yaw_response;
  double max_input_rate{1.6},max_input_acceleration{12.0};
  double task_timeout{0.75}, odom_timeout{0.40}, trajectory_timeout{5.0};
  double lookahead{0.8}, kp_position{0.8}, kp_yaw{1.5};
  // Measured-state damping. Zero preserves the legacy proportional controller.
  double velocity_damping_gain{0.0}, yaw_damping_gain{0.0};
  // Holonomic preview is a speed-scaled spatial window, not a wall-clock
  // advance of measured progress. `lookahead` retains its seconds unit.
  double min_lookahead_distance_m{.05}, max_lookahead_distance_m{.30};
  double heading_threshold{0.6}, goal_tolerance{0.20}, position_freeze_distance{0.60};
  // Historical saved-configuration fields only. Continuous forward steering
  // does not use their former engage/release or zero/full-feedback semantics.
  double steering_enter_angle_rad{.10}, steering_exit_angle_rad{.04};
  // Continuous error softness, not an engage/release angle. The historical
  // two angle fields above are read only for saved-configuration compatibility.
  double steering_error_scale_rad{.04};
  double goal_height_tolerance{0.15}, single_floor_max_height_change{0.25};
  bool require_versioned_identity{true};
  bool require_support_reference{false}; // mandatory in schema-3 execution node
  GeometryAuthority geometry_authority{GeometryAuthority::LegacyTracker};
  bool checksSupportGeometry() const {
    return require_support_reference&&geometry_authority==GeometryAuthority::LegacyTracker;
  }
  bool usesLegacyHeightAnchor() const {
    return !require_support_reference&&geometry_authority==GeometryAuthority::LegacyTracker;
  }
  bool followsNativeGeometry() const { return geometry_authority==GeometryAuthority::NativeScan; }
  bool usesNativeRollingConnection(bool strict_splice=false) const {
    return followsNativeGeometry()&&!strict_splice;
  }
  CurveAdmissionPolicy admissionPolicy(bool strict_splice=false) const {
    return usesNativeRollingConnection(strict_splice)?CurveAdmissionPolicy::NativeRolling:CurveAdmissionPolicy::StrictState;
  }
  double admissionPositionLimit(bool strict_splice=false) const {
    return usesNativeRollingConnection(strict_splice)?scan_planner::RollingCurveConnectionPolicy::max_connector_m:join_limit;
  }
  double admissionForwardLimit(bool strict_splice=false) const {
    return usesNativeRollingConnection(strict_splice)?std::min(projection_max_forward_m,
      scan_planner::RollingCurveConnectionPolicy::max_forward_arc_m):projection_max_forward_m;
  }
  // Formal nodes require Native's immutable heading seed. Old pure geometry
  // fixtures/saved nonformal callers can still use their historical default.
  bool require_body_heading_seed{false};
  bool external_goal_completion{false};
  double join_limit{.0125}, recovery_span{.6};
  // Shared with BT/SDK from the physically accepted stationary record.
  // These thresholds do not alter legacy strict-state spline admission.
  double stationary_linear_threshold_mps{.03}, stationary_angular_threshold_radps{.05};
  std::string stationary_velocity_metric{"spatial_norm_3d"};
  double stationary_reentry_duration_s{.6};
  unsigned stationary_minimum_samples{3};
  double projection_backtrack_m{.15}, projection_forward_m{.30}, projection_max_forward_m{1.0};
  double posterior_timeout{.40}, imu_timeout{.10}, max_extrapolation{.10};
  void validate() const
  {
    if((geometry_authority!=GeometryAuthority::LegacyTracker&&geometry_authority!=GeometryAuthority::NativeScan)||
       (geometry_authority==GeometryAuthority::NativeScan&&
        (!require_support_reference||!require_versioned_identity)))
      throw std::invalid_argument("native_geometry_requires_versioned_support_contract");
    forward_response.validate();yaw_response.validate();
    if(forward_response.enabled()!=yaw_response.enabled()||
       (forward_response.enabled()&&!align_then_forward)||
       !finite(max_input_rate)||max_input_rate<=0.||max_input_rate>8.||
       !finite(max_input_acceleration)||max_input_acceleration<=0.||max_input_acceleration>80.)
      throw std::invalid_argument("invalid_command_response_conditioner");
    if (session_id.empty() || planning_frame.empty() || base_frame.empty() ||
        (require_versioned_identity && (map_version_id.empty() || map_frame.empty() ||
         planning_frame == map_frame)))
      throw std::invalid_argument("session and frames must be explicit");
    if(trajectory_geometry_policy!="observed_free"&&
       trajectory_geometry_policy!="official_inflated_geometry")
      throw std::invalid_argument("unknown trajectory geometry policy");
    if((motion_model!="forward_yaw"&&motion_model!="holonomic")||
       (holonomic_yaw_policy!="path_tangent"&&holonomic_yaw_policy!="hold_heading"))
      throw std::invalid_argument("unknown tracker motion model or yaw policy");
    if(align_then_forward&&(motion_model!="holonomic"||holonomic_yaw_policy!="path_tangent"))
      throw std::invalid_argument("align_then_forward_requires_path_tangent_geometry");
    const auto bound = [](double value, double ceiling) {
      return finite(value) && value > 0.0 && value <= ceiling;
    };
    if (!bound(max_speed, HARD_PLANAR_SPEED) || !bound(max_yaw_rate, 1.0) ||
        !bound(max_acceleration, 0.8) || !bound(max_yaw_acceleration, 1.5) ||
        !finite(max_jerk) || max_jerk<0. || max_jerk>2. ||
        !finite(max_yaw_jerk) || max_yaw_jerk<0. || max_yaw_jerk>4. ||
        !bound(task_timeout, 1.0) || !bound(odom_timeout, 0.5) ||
        !bound(trajectory_timeout, 10.0) || !bound(lookahead, 2.0) ||
        !bound(min_lookahead_distance_m, 1.0) || !bound(max_lookahead_distance_m, 1.0) ||
        min_lookahead_distance_m > max_lookahead_distance_m ||
        !bound(kp_position, 3.0) || !bound(kp_yaw, 3.0) ||
        !finite(velocity_damping_gain) || velocity_damping_gain<0. || velocity_damping_gain>2. ||
        !finite(yaw_damping_gain) || yaw_damping_gain<0. || yaw_damping_gain>2. ||
        !bound(heading_threshold, 1.0) ||
        !bound(steering_enter_angle_rad,.5) || !bound(steering_exit_angle_rad,.5) ||
        steering_exit_angle_rad>=steering_enter_angle_rad || !bound(steering_error_scale_rad,.5) || !bound(goal_tolerance, 0.3) ||
        !bound(goal_height_tolerance, 0.20) ||
        !bound(single_floor_max_height_change, 0.30) ||
        goal_height_tolerance > single_floor_max_height_change ||
        !bound(position_freeze_distance, 1.0) || !bound(projection_backtrack_m, .15) ||
        !bound(projection_forward_m, .30) || !bound(projection_max_forward_m, 1.0) ||
        projection_forward_m > projection_max_forward_m || !bound(posterior_timeout,.40) ||
        !bound(imu_timeout,.10) || !bound(max_extrapolation,.10) ||
        !bound(join_limit,.0125) || !bound(recovery_span,1.) ||
        (stationary_velocity_metric!="spatial_norm_3d"&&stationary_velocity_metric!="planar_xy_with_vertical_guard_v1") ||
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
  // Separate planning-prior identity. Never set literal verified on its behalf.
  bool model_geometry_valid{false};
  std::string model_hash;
  // Decoder-owned source-region structure, not a gait/collision permission.
  bool transition_regions_present{false}, transition_regions_valid{false};
  std::vector<StairSteeringSurface> steering_surfaces;
  std::vector<Eigen::Vector3d> ground;
};
inline bool sameSupportEvidenceGeometry(const SupportEvidence& a,const SupportEvidence& b) {
  if(&a==&b)return true;
  if(!(a.identity==b.identity)||a.generation!=b.generation||a.id!=b.id||a.hash!=b.hash||a.map_hash!=b.map_hash||
     a.floor_id!=b.floor_id||a.segment_kind!=b.segment_kind||a.mode!=b.mode||a.frame!=b.frame||
     a.verified!=b.verified||a.transition_regions_present!=b.transition_regions_present||
     a.transition_regions_valid!=b.transition_regions_valid||
     a.model_geometry_valid!=b.model_geometry_valid||a.model_hash!=b.model_hash||
     a.xy_radius!=b.xy_radius||a.body_height!=b.body_height||
     a.max_slope!=b.max_slope||a.max_step!=b.max_step||a.ground.size()!=b.ground.size()||
     a.steering_surfaces!=b.steering_surfaces)return false;
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
  // Explicit native candidate heading for hold_heading. This is immutable
  // geometry metadata, not a heading sampled later by the controller.
  std::optional<double> frozen_yaw;
  // Native's immutable path-heading seed, independent of the actual reobserved
  // join pose. It survives entry observation and moving same-task handoffs.
  std::optional<double> body_heading_seed_yaw;
  std::uint32_t body_heading_sampling_intervals{0};
  ControlIdentity identity;
  // Certified join into the original (untrimmed) curve. This is measured
  // evidence, not elapsed wall time and not a shifted trajectory start clock.
  double valid_start_time{0.}, valid_start_arc_length{0.}, join_source_stamp{0.};
  std::int64_t join_source_stamp_ns{0}; // Original wire time, diagnostics only.
  // A fresh entry observation is separate from the worker's immutable source.
  // It is made only under a matching fresh whole-curve proof, never by copying
  // the old sample and changing its timestamp.
  bool entry_reobserved{false};
  // Immutable TaggedBspline splice contract. A scheduled exact-state splice
  // is not a rolling geometry replacement, even in the Native component.
  bool strict_splice_admission{false};
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
  // Set only by the formal LocalNavigationState decoder. The estimator owns
  // usability; consumers own bounded delivery and the ORIGINAL source expiry.
  // Legacy raw/pair fixtures retain the independent evidence checks below.
  bool provider_contract{false}, provider_usable{false};
  std::int64_t provider_valid_until_ns{0};
  std::int64_t provider_posterior_stamp_ns{0}, provider_imu_stamp_ns{0};
};

// Exact admission predicates with separate diagnostic names. These helpers
// neither enlarge source leases nor alter curve/collision admission policy.
inline const char* sourceEvidenceFailure(const Odom& o,double now,const Config& c) {
  if(o.provider_contract) {
    if(!o.provider_usable)return "local_state_provider_unusable";
    if(!finite(now)||!finite(o.stamp)||o.source_stamp_ns<=0||
       std::abs(o.stamp-double(o.source_stamp_ns)*1e-9)>1e-6)
      return "local_state_source_invalid";
    if(o.stamp>now+.02)return "local_state_source_in_future";
    const auto maximum=std::min<std::int64_t>(100000000LL,
      static_cast<std::int64_t>(std::llround(c.odom_timeout*1e9)));
    if(o.provider_valid_until_ns<=o.source_stamp_ns||
       o.provider_valid_until_ns-o.source_stamp_ns>maximum)
      return "local_state_provider_deadline_invalid";
    if(now-o.stamp>c.odom_timeout||
       static_cast<std::int64_t>(std::llround(now*1e9))>=o.provider_valid_until_ns)
      return "local_state_provider_expired";
    // Provenance remains structurally checked; it is not a second estimator
    // health decision computed at a later consumer clock.
    if(!finite(o.posterior_stamp)||!finite(o.imu_stamp)||
       o.posterior_stamp<=0.||o.imu_stamp<=0.||o.posterior_stamp>o.stamp||o.imu_stamp>o.stamp||
       o.provider_posterior_stamp_ns<=0||o.provider_imu_stamp_ns<=0||
       o.provider_posterior_stamp_ns>o.source_stamp_ns||o.provider_imu_stamp_ns>o.source_stamp_ns||
       std::abs(o.posterior_stamp-double(o.provider_posterior_stamp_ns)*1e-9)>1e-6||
       std::abs(o.imu_stamp-double(o.provider_imu_stamp_ns)*1e-9)>1e-6||
       !finite(o.extrapolation_sec)||o.extrapolation_sec<0.||
       std::abs(o.stamp-o.imu_stamp-o.extrapolation_sec)>=1e-5)
      return "local_state_provider_provenance_invalid";
    // Raw estimator evidence is judged at its ORIGINAL state source. Delivery
    // consumes the signed lease; it is not added again to the coast/IMU age.
    if(o.source_stamp_ns-o.provider_posterior_stamp_ns>std::llround(c.posterior_timeout*1e9)||
       o.source_stamp_ns-o.provider_imu_stamp_ns>std::llround(c.imu_timeout*1e9)||
       o.extrapolation_sec>c.max_extrapolation)
      return "local_state_provider_source_evidence_invalid";
    return nullptr;
  }
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
  std::shared_ptr<const FixedBsplineSampler> curve_sampler,velocity_sampler;
  std::vector<double> times,arcs;
  std::vector<Eigen::Vector3d> points;
  double duration{0.};
  // Immutable TaggedBspline entry domain, independent of later Native proof
  // refreshes. A new proof cannot ratchet an uncommitted candidate onto a
  // distant branch. These are local cached source fields, not new wire state.
  double origin_time{0.},origin_arc{0.},origin_source_stamp{0.};
  bool strict_splice_admission{false};
  static std::optional<EntryCurveCache> build(const Trajectory& t,
      const std::function<bool()>& allowed=[] {return true;},std::string* failure=nullptr) {
    if(failure)*failure="trajectory_evaluation_nonfinite";
    if(t.order!=3||t.points.size()<4||t.points.size()>10000||
       t.knots.size()!=t.points.size()+static_cast<std::size_t>(t.order)+1)return {};
    for(const auto& p:t.points)if(!p.allFinite())return {};
    for(std::size_t i=0;i<t.knots.size();++i)if(!finite(t.knots[i])||
      (i&&t.knots[i]-t.knots[i-1]<=1e-9))return {};
    EntryCurveCache c;c.duration=t.knots[t.points.size()]-t.knots[t.order];
    c.origin_time=t.valid_start_time;c.origin_arc=t.valid_start_arc_length;
    c.origin_source_stamp=t.join_source_stamp;
    c.strict_splice_admission=t.strict_splice_admission;
    if(!finite(c.duration)||c.duration<=.01||c.duration>120.)return {};
    Eigen::MatrixXd p(3,t.points.size());Eigen::VectorXd k(t.knots.size());
    for(std::size_t i=0;i<t.points.size();++i)p.col(i)=t.points[i];
    for(std::size_t i=0;i<t.knots.size();++i)k(i)=t.knots[i];
    auto curve=std::make_shared<scan_planner::UniformBspline>(p,t.order,.1);curve->setKnot(k);
    auto velocity=std::make_shared<scan_planner::UniformBspline>(curve->getDerivative());
    c.curve_sampler=FixedBsplineSampler::fromVendor(*curve);
    c.velocity_sampler=FixedBsplineSampler::fromVendor(*velocity);
    if(!c.curve_sampler||!c.velocity_sampler)return {};
    c.velocity=std::move(velocity);
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
    // The caller binds a live SCAN verdict whose original deadline includes
    // its source leases. Keep the source as the geometric projection anchor,
    // not another perception-age veto; the current measured body TTL above
    // and source ordering still constrain this new entry observation.
    if(!finite(proof_body_stamp)||proof_body_stamp<=0.||proof_body_stamp>body.stamp+1e-6)
      return reject("entry_body_precedes_or_exceeds_proof");
    if(!finite(seed_time)||seed_time<0.||seed_time>duration||!finite(seed_arc)||seed_arc<0.||
       std::abs(curveArcAt(times,arcs,seed_time)-seed_arc)>.01)return reject("entry_proof_curve_domain_invalid");
    const bool rolling=config.usesNativeRollingConnection(strict_splice_admission);
    if(rolling) {
      if(body.stamp-proof_body_stamp>scan_planner::RollingCurveConnectionPolicy::max_source_gap_s)
        return reject("entry_proof_body_gap_exceeded");
      if(!finite(origin_source_stamp)||origin_source_stamp<=0.||body.stamp<origin_source_stamp-1e-6)
        return reject("entry_body_precedes_candidate_source");
      if(!finite(origin_time)||origin_time<0.||origin_time>duration||!finite(origin_arc)||origin_arc<0.||
         std::abs(curveArcAt(times,arcs,origin_time)-origin_arc)>.01)
        return reject("entry_candidate_curve_domain_invalid");
    }
    const auto dynamics=scan_planner::inspectInitialMotionLimits(*curve,config.max_speed,config.max_acceleration);
    const double travel_speed=dynamics.valid&&(rolling||dynamics.recovering)?
      dynamics.peak_speed:config.max_speed;
    const double connection_limit=config.admissionPositionLimit(strict_splice_admission);
    const double travel=std::min(config.admissionForwardLimit(strict_splice_admission),
      travel_speed*std::max(0.,body.stamp-proof_body_stamp)+connection_limit);
    const auto p=projectCurveAdmission(*curve_sampler,times,arcs,points,body.position,seed_time,seed_arc,
      std::min(config.projection_backtrack_m,travel),travel,connection_limit,
      config.admissionPolicy(strict_splice_admission),origin_arc);
    if(!p)return reject("entry_measured_body_not_on_candidate");
    if(!rolling&&
       (velocity_sampler->evaluateDeBoorT(p->time)-body.velocity_in_frame).norm()>.05)
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
  // Formal follower consumes SCAN's accepted geometry, not a second terrain
  // classifier. A bounded model may supply support where literal samples are
  // sparse; legacy tracker geometry cannot use this independent prior.
  if(config.followsNativeGeometry()&&s.model_geometry_valid) {
    return index.valid&&s.identity==identity&&s.generation==generation&&!s.id.empty()&&
      s.hash.size()==64&&s.map_hash.size()==64&&s.model_hash.size()==64&&
      s.frame==config.planning_frame&&finite(s.source_stamp)&&s.source_stamp>0.&&
      s.ground.size()<=20000&&finite(s.body_height)&&s.body_height>=.2&&s.body_height<=.8;
  }
  const bool floor=(s.floor_id=="floor1"||s.floor_id=="floor2")&&
    (s.segment_kind=="floor"||s.segment_kind=="landing")&&s.mode=="general";
  const bool stair=(s.floor_id=="stairs"||s.floor_id=="stair_lower"||s.floor_id=="stair_upper")&&
    (s.segment_kind=="stair_up"||s.segment_kind=="stair_down"||s.segment_kind=="landing")&&s.mode=="stair";
  // A native SCAN verdict is not limited to flat floors. For stair packets,
  // zero legacy slope/step fields explicitly mean "not a gait certificate";
  // accepting their versioned source geometry must not claim physical stair
  // capability or reintroduce the retired tracker terrain collision check.
  const bool transition=s.transition_regions_present&&s.transition_regions_valid&&config.followsNativeGeometry();
  const bool domain=s.transition_regions_present?transition:(floor||(config.followsNativeGeometry()&&stair));
  const bool metadata=index.valid&&s.verified&&s.identity==identity&&s.generation==generation&&
    !s.id.empty()&&s.hash.size()==64&&s.map_hash.size()==64&&domain&&s.frame==config.planning_frame&&
    finite(s.source_stamp)&&s.source_stamp>0.&&s.ground.size()>=2&&s.ground.size()<=20000&&
    finite(s.xy_radius)&&s.xy_radius>0.&&s.xy_radius<=.5&&finite(s.body_height)&&
    s.body_height>=.2&&s.body_height<=.8&&finite(s.max_slope)&&finite(s.max_step)&&
    (s.transition_regions_present?(transition&&s.max_slope==0.&&s.max_step==0.):
      ((floor&&s.max_slope>0.&&s.max_step>0.)||(stair&&s.max_slope==0.&&s.max_step==0.)));
  // The packet remains an immutable, versioned source record. Its historical
  // flat-floor thresholds are not a second Native SCAN geometry verdict.
  return metadata&&(config.geometry_authority==GeometryAuthority::NativeScan||
    (s.max_slope<=.176&&s.max_step<=.08));
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
  std::shared_ptr<const FixedBsplineSampler> acceleration_sampler;
  ControlIdentity identity;
  std::uint64_t generation{0};std::int64_t trajectory_id{-1};
  std::vector<double> speed_envelope;
  // XYZ progress remains the immutable projection coordinate. Horizontal
  // travel and the integral of locally available planar braking are separate
  // metrics: a far-future vertical micro-segment cannot shrink the whole path.
  std::vector<double> horizontal_arcs,braking_arcs,local_planar_scales;
  // Prepared off the control thread from this exact spline/model/anchor.
  std::vector<StairFlightRange> stair_flights;
  double support_planar_scale{1.};
  double planar_scale{1.};bool derivatives_valid{false};
  scan_planner::InitialMotionLimits initial_motion_limits;
  std::string failure;
  std::vector<Eigen::Vector3d> original_controls;
  std::vector<double> original_knots;
  std::string frame,point_reference;
  std::string motion_model,holonomic_yaw_policy;
  std::optional<double> frozen_yaw;
  std::optional<double> body_heading_seed_yaw;
  std::uint32_t body_heading_sampling_intervals{0};
  std::vector<double> body_heading_times,body_heading_samples;
  scan_planner::SplineHeadingContract body_heading_contract;
  double max_speed{0.},max_acceleration{0.},max_yaw{0.},height_tolerance{0.};
  bool requires_support{false},requires_body_heading_seed{false},align_then_forward{false};
  GeometryAuthority geometry_authority{GeometryAuthority::LegacyTracker};
  static double metricAt(const std::vector<double>& from,const std::vector<double>& to,double value) {
    if(from.empty()||from.size()!=to.size())return 0.;
    const auto hi=std::upper_bound(from.begin(),from.end(),value);
    if(hi==from.begin())return to.front();
    if(hi==from.end())return to.back();
    const auto i=static_cast<std::size_t>(hi-from.begin());const double span=from[i]-from[i-1];
    return span<=1e-12?to[i]:to[i-1]+std::clamp((value-from[i-1])/span,0.,1.)*(to[i]-to[i-1]);
  }
  double horizontalAt(double arc)const {return metricAt(entry.arcs,horizontal_arcs,arc);}
  double arcAtHorizontal(double horizontal)const {return metricAt(horizontal_arcs,entry.arcs,horizontal);}
  double scaleAt(double time)const {
    const auto v=entry.velocity_sampler->evaluateDeBoorT(time);
    if(v.norm()>1e-10)return std::min(support_planar_scale,spatialToPlanarScale(v));
    return metricAt(entry.times,local_planar_scales,time);
  }
  double brakingLimitAt(double arc,double acceleration)const {
    if(motion_model!="holonomic")return brakingEnvelopeAt(entry.arcs,speed_envelope,arc,
      acceleration*planar_scale,planar_scale);
    return brakingEnvelopeAt(braking_arcs,speed_envelope,metricAt(entry.arcs,braking_arcs,arc),acceleration,1.);
  }
  double bodyHeadingAt(double time)const {
    if(body_heading_samples.empty()||!finite(time)||time<0.||time>entry.duration)
      return std::numeric_limits<double>::quiet_NaN();
    // Prefix memory is prepared once on Native's EXACT full lattice. The
    // last exact-time derivative is the same final step as its shared helper;
    // no elapsed progress, yaw interpolation or per-tick full replay.
    const auto hi=std::upper_bound(body_heading_times.begin(),body_heading_times.end(),time);
    const std::size_t i=hi==body_heading_times.begin()?0:static_cast<std::size_t>(hi-body_heading_times.begin()-1);
    return scan_planner::bodyHeadingAtCurveTime(entry.velocity_sampler->evaluateDeBoorT(time),
      body_heading_samples[i],body_heading_contract,time);
  }
  double bodyHeadingCurvatureAt(double time)const {
    const auto v=entry.velocity_sampler->evaluateDeBoorT(time);
    const auto a=acceleration_sampler->evaluateDeBoorT(time);
    if(const auto sample=scan_planner::forwardEntryHeadingGeometryAt(time,v,a,body_heading_contract))
      return sample->curvature;
    if(body_heading_contract.policy==scan_planner::BodyHeadingPolicy::HoldHeading)return 0.;
    const double speed=v.head<2>().norm();
    if(speed<body_heading_contract.minimum_direction_speed)return 0.;
    return (v.x()*a.y()-v.y()*a.x())/(speed*speed*speed);
  }
  bool matches(const Config& c,const Trajectory& t)const {
    if(!(identity==t.identity)||generation!=t.generation||trajectory_id!=t.id||frame!=t.frame_id||
       point_reference!=t.point_reference||max_speed!=c.max_speed||max_acceleration!=c.max_acceleration||
       max_yaw!=c.max_yaw_rate||height_tolerance!=c.goal_height_tolerance||requires_support!=c.require_support_reference||
       geometry_authority!=c.geometry_authority||
       entry.strict_splice_admission!=t.strict_splice_admission||
       motion_model!=c.motion_model||holonomic_yaw_policy!=c.holonomic_yaw_policy||
       align_then_forward!=c.align_then_forward||
       requires_body_heading_seed!=c.require_body_heading_seed||body_heading_seed_yaw!=t.body_heading_seed_yaw||
       body_heading_sampling_intervals!=t.body_heading_sampling_intervals||
       frozen_yaw!=t.frozen_yaw||
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
    out->motion_model=config.motion_model;out->holonomic_yaw_policy=config.holonomic_yaw_policy;
    out->align_then_forward=config.align_then_forward;
    out->geometry_authority=config.geometry_authority;
    out->frozen_yaw=t.frozen_yaw;
    out->body_heading_seed_yaw=t.body_heading_seed_yaw;out->requires_body_heading_seed=config.require_body_heading_seed;
    out->body_heading_sampling_intervals=t.body_heading_sampling_intervals;
    if((t.body_heading_seed_yaw&&!finite(*t.body_heading_seed_yaw))||
       (t.body_heading_seed_yaw&&(!t.body_heading_sampling_intervals||t.body_heading_sampling_intervals>49999))||
       (!t.body_heading_seed_yaw&&t.body_heading_sampling_intervals)||
       (config.require_body_heading_seed&&!t.body_heading_seed_yaw)) {
      out->failure="body_heading_seed_missing_or_invalid";return out;
    }
    if((t.frozen_yaw&&!finite(*t.frozen_yaw))||
       (config.motion_model=="holonomic"&&config.holonomic_yaw_policy=="hold_heading"&&!t.frozen_yaw)) {
      out->failure="frozen_heading_missing_or_invalid";return out;
    }
    if(config.holonomic_yaw_policy=="hold_heading"&&t.body_heading_seed_yaw&&t.frozen_yaw&&
       std::abs(*t.body_heading_seed_yaw-*t.frozen_yaw)>1e-12) {
      // Native's constant held-yaw wire encoding uses the exact same scalar;
      // the seed table and controller must not represent two different poses.
      out->failure="body_heading_seed_frozen_yaw_mismatch";return out;
    }
    out->height_tolerance=config.goal_height_tolerance;out->requires_support=config.require_support_reference;
    auto entry=EntryCurveCache::build(t,allowed,&out->failure);
    if(!entry){if(!allowed())out->failure="preparation_cancelled";return out;}
    out->failure.clear();
    out->entry=std::move(*entry);
    if(t.body_heading_seed_yaw) {
      out->body_heading_contract.measured_yaw=*t.body_heading_seed_yaw;
      out->body_heading_contract.align_then_forward=config.align_then_forward;
      out->body_heading_contract.measured_heading_enabled=true;
      if(config.holonomic_yaw_policy=="hold_heading")
        out->body_heading_contract.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
      if(!scan_planner::resolveForwardEntryHeading(out->body_heading_contract,out->entry.duration,
           t.body_heading_sampling_intervals,
           [&](double at){return out->entry.curve_sampler->evaluateDeBoorT(at);},
           [&](double at){return out->entry.velocity_sampler->evaluateDeBoorT(at);},allowed)) {
        out->failure="body_heading_geometry_invalid";return out;
      }
      if(config.align_then_forward&&config.holonomic_yaw_policy=="path_tangent"&&
         out->entry.velocity_sampler->evaluateDeBoorT(0.).head<2>().norm()<out->body_heading_contract.minimum_direction_speed) {
        const auto direction=scan_planner::nearCurveDirection(
          [&](double at){const auto p=out->entry.curve_sampler->evaluateDeBoorT(at);return std::array<double,3>{p.x(),p.y(),p.z()};},
          [&](double at){const auto v=out->entry.velocity_sampler->evaluateDeBoorT(at);return std::array<double,3>{v.x(),v.y(),v.z()};},
          0.,out->entry.duration,allowed);
        if(direction)out->body_heading_contract.measured_yaw=std::atan2(direction->direction[1],direction->direction[0]);
      }
      double yaw=out->body_heading_contract.measured_yaw;
      out->body_heading_times.reserve(t.body_heading_sampling_intervals+1);
      out->body_heading_samples.reserve(t.body_heading_sampling_intervals+1);
      for(std::uint32_t i=0;i<=t.body_heading_sampling_intervals;++i) {
        if(!allowed()){out->failure="preparation_cancelled";return out;}
        const double at=out->entry.duration*static_cast<double>(i)/t.body_heading_sampling_intervals;
        const auto v=out->entry.velocity_sampler->evaluateDeBoorT(at);
        if(!v.allFinite()){out->failure="body_heading_sample_invalid";return out;}
        yaw=scan_planner::bodyHeadingAtCurveTime(v,yaw,out->body_heading_contract,at);
        out->body_heading_times.push_back(at);out->body_heading_samples.push_back(yaw);
      }
    }
    auto velocity=*out->entry.velocity;
    auto acceleration=std::make_shared<scan_planner::UniformBspline>(velocity.getDerivative());
    out->initial_motion_limits=scan_planner::inspectInitialMotionLimits(
      *out->entry.curve,config.max_speed,config.max_acceleration);
    out->derivatives_valid=out->initial_motion_limits.valid;
    out->acceleration_sampler=FixedBsplineSampler::fromVendor(*acceleration);
    if(!out->acceleration_sampler){out->failure="curve_acceleration_sampler_invalid";return out;}
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
      if(config.checksSupportGeometry()&&!pointSupported(*out->support,*out->support_index,
          out->entry.points[i],config.goal_height_tolerance)){out->failure="curve_outside_support";return out;}
    }
    if(config.checksSupportGeometry())out->support_planar_scale=
      spatialToPlanarScale(Eigen::Vector3d(1.,0.,std::tan(out->support->max_slope)));
    out->planar_scale=std::min(out->planar_scale,out->support_planar_scale);
    out->horizontal_arcs.assign(out->entry.arcs.size(),0.);
    out->braking_arcs.assign(out->entry.arcs.size(),0.);
    out->local_planar_scales.reserve(out->entry.arcs.size());
    for(std::size_t i=0;i<out->entry.times.size();++i) {
      if(!allowed()){out->failure="preparation_cancelled";return out;}
      const auto v=out->entry.velocity_sampler->evaluateDeBoorT(out->entry.times[i]);
      const auto chord=i+1<out->entry.points.size()?out->entry.points[i+1]-out->entry.points[i]:
        out->entry.points[i]-out->entry.points[i-1];
      out->local_planar_scales.push_back(std::min(out->support_planar_scale,
        spatialToPlanarScale(v.norm()>1e-10?v:Eigen::Vector3d(chord))));
      if(i)out->horizontal_arcs[i]=out->horizontal_arcs[i-1]+chordHorizontal(out->entry.points[i]-out->entry.points[i-1]);
    }
    if(config.followsNativeGeometry()&&out->support&&out->support->model_geometry_valid)
      out->stair_flights=stairCurveFlightRanges(out->support->steering_surfaces,
        out->entry.points,out->horizontal_arcs,out->support->body_height,allowed);
    if(!allowed()){out->failure="preparation_cancelled";return out;}
    for(std::size_t i=1;i<out->entry.times.size();++i) {
      if(!allowed()){out->failure="preparation_cancelled";return out;}
      const auto chord=out->entry.points[i]-out->entry.points[i-1];
      const double q=std::min({out->local_planar_scales[i-1],out->local_planar_scales[i],
        spatialToPlanarScale(chord)});
      // dv_planar^2 <= 2*a_planar*dh, with a_planar=A*q HERE,
      // not A times the minimum q of an unrelated future chord.
      out->braking_arcs[i]=out->braking_arcs[i-1]+q*(out->horizontal_arcs[i]-out->horizontal_arcs[i-1]);
    }
    std::vector<double> limits;limits.reserve(out->entry.times.size());
    for(std::size_t i=0;i<out->entry.times.size();++i) {
      if(!allowed()){out->failure="preparation_cancelled";return out;}
      const auto time=out->entry.times[i];
      const auto velocity=out->entry.velocity->evaluateDeBoorT(time);
      const auto acceleration=out->acceleration->evaluateDeBoorT(time);
      double limit=config.motion_model=="holonomic"?
        curvaturePlanarLimit(velocity,acceleration,.5*config.max_acceleration*out->local_planar_scales[i],
          config.max_speed*out->local_planar_scales[i]):
        curvatureForwardLimit(velocity,acceleration,config.max_yaw_rate,config.max_speed*out->planar_scale);
      // A holonomic-capable platform can select a no-sideways navigation
      // strategy. That strategy still needs |kappa|*v <= max_yaw_rate. Apply
      // it to the EXISTING future braking envelope, rather than driving too
      // fast into a bend and repeatedly saturating yaw/zeroing translation.
      if(config.align_then_forward) {
        // Turn authority belongs to the SAME frozen body-heading field used
        // by Native and tracking. A backwards acquired prefix is not the raw
        // position tangent's almost-singular sideways rotation. The separate
        // translational-acceleration limit above remains unchanged.
        const double curvature=out->bodyHeadingCurvatureAt(time);
        if(!finite(curvature)){out->failure="body_heading_curvature_invalid";return out;}
        if(std::abs(curvature)>1e-12)limit=std::min(limit,config.max_yaw_rate/std::abs(curvature));
      }
      limits.push_back(limit);
    }
    // A committed local curve ends at a real stopping boundary until another
    // curve is actually admitted. Reserve braking distance before that end.
    if(config.motion_model=="holonomic"&&!limits.empty())limits.back()=0.;
    out->speed_envelope=config.motion_model=="holonomic"?
      brakingSpeedEnvelope(out->braking_arcs,limits,.5*config.max_acceleration,1.):
      brakingSpeedEnvelope(out->entry.arcs,limits,config.max_acceleration*out->planar_scale,out->planar_scale);
    if(out->speed_envelope.empty())out->failure="curvature_braking_envelope_invalid";
    return out;
  }
 private:
  static double chordHorizontal(const Eigen::Vector3d& chord){return chord.head<2>().norm();}
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
  double lateral{0.0};
  bool frozen{true}, finished{false};
  std::string reason{"idle"};
};

struct SteeringPreviewSample {
  bool valid{false};
  std::int64_t curve{-1},body_source_ns{0};
  double projection_time{0.},projection_arc{0.},committed_arc{0.},preview_time{0.},preview_arc{0.};
  Eigen::Vector2d target{Eigen::Vector2d::Zero()};
};
// Inspectable command calculation, never motion authorization or sensor data.
struct ControlDiagnostic {
  bool forward_entry_heading_active{false};
  bool valid{false},final_endpoint{false};
  double curve_speed{0.},remaining_arc{0.},goal_distance{0.},speed_cap{0.},jerk_target_cap{0.};
  double measured_forward{0.},forward_target{0.},heading_error{0.},heading_weight{0.},yaw_target{0.};
  double steering_curvature{0.},unallocated_forward_target{0.},unallocated_yaw_target{0.},turn_allocation_scale{1.};
  double steering_current_curvature{0.},steering_preview_distance{0.};
  SteeringPreviewSample steering_sample,previous_steering_sample;
  double steering_tangent_yaw{0.},steering_cross_error{0.},steering_flight_weight{0.};
  double steering_yaw_feedforward{0.},steering_yaw_feedback{0.},steering_yaw_damping{0.};
  std::string steering_curvature_source;
  double forward_input_fraction{0.},forward_input_rate{0.},yaw_input_fraction{0.},yaw_input_rate{0.};
  bool input_limit_override{false};
  bool working_level_steering{false};
  double yaw_working_fraction{0.};
  std::string steering_phase;
  std::string zero_target_cause;
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
      terminal_settling_=false;
      floor_anchor_valid_ = false;
      holonomic_heading_valid_=false;goal_stationary_samples_=0;goal_stationary_first_=goal_stationary_source_=0.;
      path_turn_intent_.reset();goal_turn_intent_.reset();
      stair_steering_history_.reset();stair_steering_scope_valid_=false;
      last_steering_sample_={};
      measured_arc_ = committed_arc_ = execution_time_ = 0.;
      last_projected_stamp_ = 0.;
      if (config_.require_versioned_identity && (!have_odom_ ||
          odom_.localization_epoch != task.identity.localization_epoch ||
          odom_.localization_seed_id != task.identity.localization_seed_id)) have_odom_ = false;
      last_output_ = Output{};
      resetCommandAcceleration();
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
      if(recovery_samples_>=3 && odom.stamp-recovery_first_source_+1e-9>=config_.recovery_span) {
        holding_=false; reason_=trajectory_?"tracking":"waiting_trajectory";
      }
    }
    return true;
  }

  bool receiveTrajectory(const Trajectory &trajectory, double ros_now, double received,
                         bool fresh_external_curve_lease=false)
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
    // A rejected candidate is not a withdrawal of the independently validated
    // incumbent. Explicit cancellation/proof expiry are separate channels.
    // Distinguish input format, source age and evaluator failures without
    // changing any admission predicate or withdrawing the incumbent. A generic
    // "invalid" reason previously hid an expired pending candidate as geometry.
    if (!finite(ros_now) || !finite(received) || !finite(trajectory.start_time))
      return candidateReject("trajectory_clock_nonfinite");
    if (trajectory.frame_id != config_.planning_frame) return candidateReject("trajectory_frame_mismatch");
    if (trajectory.start_time > ros_now + 0.2) return candidateReject("trajectory_start_in_future");
    // A typed caller can recertify immutable geometry against the current map
    // and reobserve its entry from actual state. That live certificate, not
    // elapsed time since the original solve, supplies its collision lifetime.
    // Keep original timestamps and all measured-entry predicates; legacy
    // callers (or a candidate without an actual reobserved entry) retain TTL.
    const bool externally_recertified=fresh_external_curve_lease&&
      config_.require_versioned_identity&&trajectory.entry_reobserved;
    // Typed execution has already bound the exact current native proof and
    // reobserved the actual entry. Task ownership and replacement order are
    // the complete identity/generation/curve ID above; an immutable solve
    // clock is not another veto on that current admission.
    if (!externally_recertified&&trajectory_&&trajectory.start_time<trajectory_start_)
      return candidateReject("older_start_time");
    if (!externally_recertified&&trajectory.start_time+1e-6<task_.issued_at)
      return candidateReject("trajectory_predates_task");
    if (!externally_recertified&&ros_now-trajectory.start_time>config_.trajectory_timeout)
      return candidateReject("trajectory_start_expired");
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
    if (config_.usesLegacyHeightAnchor() && (max_z - min_z > config_.single_floor_max_height_change ||
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
    if (!externally_recertified&&ros_now-trajectory.start_time>duration)
      return candidateReject("trajectory_duration_elapsed");
    auto prepared=trajectory.prepared;
    if(prepared&&!prepared->matches(config_,trajectory))return candidateReject("prepared_geometry_identity_mismatch");
    if(!prepared)prepared=PreparedGeometry::build(config_,trajectory,support_?*support_:SupportEvidence{});
    if(!prepared->failure.empty())return candidateReject(prepared->failure.c_str());
    const auto& arc=prepared->entry.arcs;
    const auto& times=prepared->entry.times;
    const auto& samples=prepared->entry.points;
    const auto curve=prepared->entry.curve,velocity=prepared->entry.velocity;
    double initial_time=0., initial_arc=0.;
    if (config_.require_versioned_identity) {
      const double join_time=trajectory.valid_start_time;
      const auto acceleration=prepared->acceleration;
      join_diagnostic_.curve_duration=duration;
      if(const auto why=joinEvidenceFailure(trajectory,have_odom_,odom_,ros_now,duration,config_))return candidateReject(why);
      const bool strict_splice=prepared->entry.strict_splice_admission;
      const bool rolling=config_.usesNativeRollingConnection(strict_splice);
      const double connection_limit=config_.admissionPositionLimit(strict_splice);
      if ((curve->evaluateDeBoorT(join_time)-trajectory.join_position).norm()>connection_limit)
        return candidateReject("join_position_off_curve");
      if (!rolling&&
          (velocity->evaluateDeBoorT(join_time)-trajectory.join_velocity).norm()>.05)
        return candidateReject("join_velocity_off_curve");
      if (!prepared->derivatives_valid) return candidateReject("curve_derivative_bound");
      if (trajectory.join_acceleration_valid && (!trajectory.join_acceleration.allFinite() ||
          (!rolling&&
           (acceleration->evaluateDeBoorT(join_time)-trajectory.join_acceleration).norm()>.10)))
        return candidateReject("join_acceleration_off_curve");
      const double seed_arc=curveArcAt(times,arc,join_time);
      // Both ends integrate the same original curve; discretization error is
      // bounded, not an opportunity to skip to an unrelated branch.
      if (std::abs(seed_arc-trajectory.valid_start_arc_length)>.01) return candidateReject("join_arc_mismatch");
      const double source_dt=std::max(0.,odom_.stamp-trajectory.join_source_stamp);
      const double travel_speed=(rolling||prepared->initial_motion_limits.recovering)?
          prepared->initial_motion_limits.peak_speed:config_.max_speed;
      const double travel=std::min(config_.admissionForwardLimit(strict_splice),
          travel_speed*source_dt+connection_limit);
      const auto projected=projectCurveAdmission(*curve,times,arc,samples,odom_.position,
          join_time,seed_arc,std::min(config_.projection_backtrack_m,travel),travel,connection_limit,
          config_.admissionPolicy(strict_splice),prepared->entry.origin_arc);
      if (!projected) return candidateReject("measured_body_not_on_candidate");
      if (!rolling&&
          (velocity->evaluateDeBoorT(projected->time)-odom_.velocity_in_frame).norm()>.05)
        return candidateReject("measured_velocity_off_candidate");
      initial_time=projected->time; initial_arc=projected->arc;
    }
    // Strictly preserve the limiter history. Only the reachable limit at the
    // actual entry matters, not the minimum over unrelated future geometry.
    const double planar_scale=holonomic()?prepared->scaleAt(initial_time):prepared->planar_scale;
    const double entry_limit=std::min(config_.max_speed*planar_scale,
      prepared->brakingLimitAt(initial_arc,(holonomic()?.5:1.)*config_.max_acceleration));
    if((holonomic()?std::hypot(last_output_.forward,last_output_.lateral):last_output_.forward)>entry_limit+1e-12)
      return candidateReject("curvature_braking_limit_below_current_command");
    trajectory_ = std::move(curve);
    if((trajectory_->evaluateDeBoorT(duration)-task_.goal).norm()>config_.join_limit)terminal_settling_=false;
    if(holonomic()) {
      if(config_.holonomic_yaw_policy=="hold_heading")holonomic_heading_=*trajectory.frozen_yaw;
      else if(prepared->body_heading_seed_yaw)holonomic_heading_=prepared->bodyHeadingAt(initial_time);
      else if(!holonomic_heading_valid_)holonomic_heading_=odom_.yaw;
      holonomic_heading_valid_=true;
    }
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
      turn_phase_=holonomic()?TurnPhase::Following:TurnPhase::Decelerating;
      resetTurnStationaryWindow();turn_sample_stamp_=turnSourceStamp();
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
  // Original command age follows the bounded shared pipeline record. Latest
  // actual body still has its independent 100 ms lease; neither is re-stamped.
  bool notifyBlockedForwardTurn(std::int64_t trajectory_id,double demand_source_stamp,
      double now,double forward,double yaw_rate,double original_command_max_age=.1) {
    if(holonomic())return notifyBlockedMotion(trajectory_id,demand_source_stamp,now,forward,0.,yaw_rate,
      original_command_max_age);
    const bool in_place=turn_phase_==TurnPhase::Aligning;
    if(!active_||!trajectory_||!have_odom_||trajectory_id!=last_trajectory_id_||
       !finite(forward)||forward<0.||(forward<=1e-6&&!in_place)||!finite(yaw_rate)||
       (forward<=1e-6&&std::abs(yaw_rate)<=1e-6)||
       !finite(original_command_max_age)||original_command_max_age<.1||original_command_max_age>.5||
       !fresh(now,demand_source_stamp,original_command_max_age)||demand_source_stamp<=last_turn_feedback_stamp_||
       !fresh(now,odom_.stamp,.1,.02)||turn_phase_==TurnPhase::WaitingEntry)return false;
    last_turn_feedback_stamp_=demand_source_stamp;
    turn_first_engaged_=true;
    if(forward>1e-6&&std::abs(yaw_rate)<=1e-6) {
      // A blocked straight entry cannot be repaired by aligning to the same
      // heading. Retain task/geometry, stop this entry, and request a different
      // native local curve. The wrapper has already matched real occupied
      // evidence; unknown, reverse and zero commands never reach this branch.
      turn_phase_=TurnPhase::WaitingEntry;resetTurnStationaryWindow();
      last_output_=Output{};last_output_.frozen=true;
      reason_="waiting_executable_entry";return true;
    }
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
  bool notifyBlockedMotion(std::int64_t trajectory_id,double source,double now,
      double forward,double lateral,double yaw_rate,double original_command_max_age=.1) {
    if(!holonomic())return lateral==0.&&notifyBlockedForwardTurn(trajectory_id,source,now,forward,yaw_rate,
      original_command_max_age);
    if(!active_||!trajectory_||!have_odom_||trajectory_id!=last_trajectory_id_||
       !finite(forward)||!finite(lateral)||!finite(yaw_rate)||
       (std::hypot(forward,lateral)<=1e-6&&std::abs(yaw_rate)<=1e-6)||
       !finite(original_command_max_age)||original_command_max_age<.1||original_command_max_age>.5||
       !fresh(now,source,original_command_max_age)||source<=last_turn_feedback_stamp_||
       !fresh(now,odom_.stamp,.1,.02)||turn_phase_==TurnPhase::WaitingEntry)return false;
    // An occupied holonomic sweep is not evidence that rotating toward the
    // tangent repairs it. Ask the same native planner for a different entry.
    last_turn_feedback_stamp_=source;turn_phase_=TurnPhase::WaitingEntry;
    resetTurnStationaryWindow();last_output_=Output{};
    reason_="waiting_executable_entry";return true;
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
    control_diagnostic_=ControlDiagnostic{};
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
    if(!supportContractAllowsBody()) {
      reason_="support_evidence_missing_or_body_outside_support"; return stop();
    }
    if (config_.usesLegacyHeightAnchor() && !floor_anchor_valid_) {
      floor_anchor_z_ = odom_.position.z();
      floor_anchor_valid_ = true;
    }
    // Pin height to this generation's first fresh measured body pose. Replans
    // cannot walk this anchor up a staircase one small local segment at a time.
    if (config_.usesLegacyHeightAnchor() && (std::abs(task_.goal.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        std::abs(odom_.position.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        (trajectory_ && (trajectory_min_z_ < floor_anchor_z_ - config_.single_floor_max_height_change ||
                         trajectory_max_z_ > floor_anchor_z_ + config_.single_floor_max_height_change)))) {
      cancel("task_outside_single_floor_envelope");
      return stop();
    }
    const bool position_at_goal=(task_.goal.head<2>()-odom_.position.head<2>()).norm()<=config_.goal_tolerance&&
      std::abs(task_.goal.z()-odom_.position.z())<=config_.goal_height_tolerance;
    const bool measured_goal_stationary=position_at_goal&&
      std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm())<=config_.stationary_linear_threshold_mps&&
      std::abs(odom_.angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps;
    if(holonomic()&&trajectory_) {
      refreshMeasuredProgress(ros_now);
      // A real global-goal pose owns the deceleration/settling stage below.
      // Reaching it while still moving is not a request for another local
      // route. This changes neither the limiter nor measured-rest criteria;
      // an intermediate finite endpoint still requests its normal successor.
      if(needsTerminalReplan(ros_now)&&!position_at_goal) {
        local_endpoint_waiting_=true;
        // Outside SCAN's bounded actual-body connector, only a fresh solve
        // can supply a valid return trajectory. Keep the task and immutable
        // incumbent; this reason already drives the Native reseed channel.
        return holonomicLimited(Eigen::Vector2d::Zero(),0.,config_.max_speed*planar_scale_,
          config_.max_acceleration*planar_scale_,dt,"local_segment_finished_waiting_replan",true);
      }
    }
    if(position_at_goal) {
      if(holonomic()) {
        const bool measured_stationary=measured_goal_stationary;
        Output out;
        if(measured_stationary) {
          // The same fresh measured pose/rest facts already satisfy the
          // follower's completion status. Do not emit an old approach or yaw
          // tail while asking its owner to stop. This is exact zero, not
          // fabricated rest evidence or retirement of the task.
          reason_="goal_position_reached";out=stop();
        } else out=!config_.align_then_forward&&atTerminalProjection()&&isFinalCurveEndpoint()?
          finalEndpointCapture(dt,"goal_position_settling",true):
          holonomicLimited(Eigen::Vector2d::Zero(),0.,config_.max_speed*planar_scale_,
            config_.max_acceleration*planar_scale_,dt,"goal_position_settling",true);
        // Being inside position tolerance is a request to decelerate, not a
        // physical-rest observation. Keep the SAME XY/yaw limiter until fresh
        // actual rest permits exact zero above. No special forward-only cut.
        if(config_.external_goal_completion||braking_reentry_required_)return out;
        const bool stationary=std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm())<=
            config_.stationary_linear_threshold_mps&&
          std::abs(odom_.angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps&&
          std::hypot(out.forward,out.lateral)<=1e-9&&std::abs(out.yaw_rate)<=1e-9;
        const double source=turnSourceStamp();
        if(!stationary){goal_stationary_samples_=0;goal_stationary_first_=0.;}
        if(source>goal_stationary_source_) {
          if(source-goal_stationary_source_>config_.odom_timeout)goal_stationary_samples_=0;
          goal_stationary_source_=source;
          if(stationary) {
            if(goal_stationary_samples_++==0)goal_stationary_first_=source;
            if(goal_stationary_samples_>=config_.stationary_minimum_samples&&
               source-goal_stationary_first_+1e-9>=config_.stationary_reentry_duration_s) {
              finished_=true;cancel("goal_reached");return stop();
            }
          }
        }
        return out;
      }
      if(config_.external_goal_completion) { reason_="goal_position_reached"; return stop(); }
      finished_ = true;
      cancel("goal_reached");
      return stop();
    }
    goal_stationary_samples_=0;goal_stationary_first_=0.;
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
    const auto pos = positionAt(execution_time_);
    const Eigen::Vector2d pos_error = pos.head<2>() - odom_.position.head<2>();
    if (pos_error.norm() > 2.0 || std::abs(pos.z() - odom_.position.z()) > 0.5) {
      invalidateTrajectory("tracking_error_outside_single_floor_envelope");
      return stop();
    }
    if (!pos.allFinite()) {
      invalidateTrajectory("nonfinite_controller_output");
      return stop();
    }
    if(holonomic()) {
      if(!config_.align_then_forward&&atTerminalProjection()&&isFinalCurveEndpoint())
        return finalEndpointCapture(dt,"final_endpoint_capture",false);
      const Eigen::Vector3d velocity=velocityAt(execution_time_);
      const double speed=velocity.head<2>().norm();
      Eigen::Vector2d tangent=speed>1e-9 ? Eigen::Vector2d(velocity.head<2>()/speed) :
        Eigen::Vector2d::Zero();
      double command_speed=speed;
      // A native curve joined to a stationary measured body has v(0)=0.
      // Measured-only progress plus v(s) feed-forward would then self-lock:
      // no command -> no body motion -> no new s -> no command. Use a bounded
      // spatial preview on THIS immutable curve only to bootstrap translation.
      // Do not advance execution_time_/arc, add a future chord to normal
      // tracking, or pretend the preview is the current measured state.
      // Startup is a near-zero tangent condition, NOT the lateral steering
      // lookahead. Sharing that length made every ordinary slow curve a
      // "startup" and raised its requested speed for its entire lifetime.
      // Direction confidence and actuator breakaway are different facts.
      // Reach the configured static response's effective point when enabled;
      // otherwise a zero-derivative curve can remain stuck below SDK response.
      const double startup_speed=std::min(config_.max_speed,std::max(
        scan_planner::SplineHeadingContract{}.minimum_direction_speed,
        config_.forward_response.enabled()?
          config_.forward_response.reference*config_.forward_response.blend_ratio:0.));
      const double startup_distance=startup_speed/config_.kp_position;
      if(speed<startup_speed&&arcTable().back()>measured_arc_) {
        // Sample before the stopping endpoint: its exact zero derivative is
        // a terminal boundary, not evidence that a short forward curve has
        // no start direction. Endpoint braking itself remains authoritative.
        const double current_horizontal=prepared_geometry_->horizontalAt(measured_arc_);
        const double preview_arc=prepared_geometry_->arcAtHorizontal(current_horizontal+std::min(
          startup_distance,.5*(prepared_geometry_->horizontal_arcs.back()-current_horizontal)));
        const auto preview_velocity=velocityAt(timeAtArc(preview_arc));
        const double preview_speed=preview_velocity.head<2>().norm();
        const double reachable_preview=config_.kp_position*(prepared_geometry_->horizontalAt(preview_arc)-current_horizontal);
        // A spatial P startup must not be capped back to an almost-zero
        // future derivative, or measured-only progress can still self-lock.
        // The SAME current-curve speed/acceleration/stopping envelope below
        // remains authoritative; no curve clock or sensor source is advanced.
        command_speed=std::max(speed,reachable_preview);
        if(speed<=1e-9&&command_speed>1e-9) {
          // The exact zero tangent has no direction. Prefer the first local
          // forward tangent (within 1 mm), not a chord across a future bend.
          const auto local_velocity=velocityAt(timeAtArc(std::min(preview_arc,measured_arc_+.001)));
          const double local_speed=local_velocity.head<2>().norm();
          if(local_speed>1e-9)tangent=local_velocity.head<2>()/local_speed;
          else if(preview_speed>1e-9)tangent=preview_velocity.head<2>()/preview_speed;
        }
      }
      if(speed<scan_planner::SplineHeadingContract{}.minimum_direction_speed) {
        // Both body-XY tracking and saved forward-only profiles need the
        // same bounded translation direction at an almost-stationary entry.
        // Do not amplify a mm/s derivative sign into a breakaway request.
        // This changes no native twist or yaw certificate: below the shared
        // direction-speed threshold yaw still retains the previous heading.
        const auto direction=scan_planner::nearCurveDirection(
          [this](double t){const auto p=positionAt(t);return std::array<double,3>{p.x(),p.y(),p.z()};},
          [this](double t){const auto v=velocityAt(t);return std::array<double,3>{v.x(),v.y(),v.z()};},
          execution_time_,duration_);
        if(direction) {
          const Eigen::Vector2d v(direction->direction[0],direction->direction[1]);tangent=v.normalized();
        } else tangent.setZero();
      }
      if(config_.align_then_forward&&
         execution_time_<prepared_geometry_->body_heading_contract.forward_entry_end) {
        const double heading=prepared_geometry_->bodyHeadingAt(execution_time_);
        tangent={std::cos(heading),std::sin(heading)};
      }
      // Native and control share the same curve-derived body heading;
      // the existing curvature/braking cap and vector limiter bound startup.
      const Eigen::Vector2d cross_error=pos_error-tangent*pos_error.dot(tangent);
      const Eigen::Vector2d reference_velocity=command_speed*tangent;
      // SCAN author's closed-loop law: velocity feed-forward + FULL XY
      // position feedback, then rotate into the measured body frame. XYZ
      // projection can leave an along-XY residual on sloped curves; discarding
      // it as "cross-track only" would silently discard valid position error.
      // Explicit historical damping remains configurable for saved profiles;
      // the mainline author-like profile sets both damping gains to zero.
      Eigen::Vector2d world=reference_velocity+config_.kp_position*pos_error+
        config_.velocity_damping_gain*(reference_velocity-odom_.velocity_in_frame.head<2>());
      if(!world.allFinite()) {invalidateTrajectory("nonfinite_controller_output");return stop();}
      double cap=std::min(config_.max_speed*planar_scale_,
        prepared_geometry_->brakingLimitAt(measured_arc_,.5*config_.max_acceleration));
      // XY overlap with a goal on another floor is not arrival. Retain the
      // exact flat-goal braking law, but do not apply it on an intermediate
      // curve while the actual support level is still different. The finite
      // current curve's own stopping envelope above remains unchanged.
      if(isFinalCurveEndpoint()||std::abs(task_.goal.z()-odom_.position.z())<=config_.goal_height_tolerance)
        cap=std::min(cap,std::sqrt(config_.max_acceleration*planar_scale_*
          (task_.goal.head<2>()-odom_.position.head<2>()).norm()));
      control_diagnostic_.valid=true;control_diagnostic_.final_endpoint=isFinalCurveEndpoint();
      control_diagnostic_.curve_speed=speed;
      control_diagnostic_.remaining_arc=std::max(0.,prepared_geometry_->horizontal_arcs.back()-
        prepared_geometry_->horizontalAt(measured_arc_));
      control_diagnostic_.goal_distance=(task_.goal.head<2>()-odom_.position.head<2>()).norm();
      control_diagnostic_.speed_cap=control_diagnostic_.jerk_target_cap=cap;
      if(config_.max_jerk>0.) {
        const Eigen::Vector2d previous(last_output_.forward,last_output_.lateral);
        const double along_acceleration=previous.norm()>1e-9?
          command_planar_acceleration_.dot(previous/previous.norm()):command_planar_acceleration_.norm();
        const double remaining=control_diagnostic_.remaining_arc;
        // This OPTIONAL smoother supplies an early deceleration TARGET, not
        // another hard motion cap. A replacement accepted by the existing
        // curve envelope must not immediately hard-zero merely because the
        // carried command cannot satisfy a newly introduced S-stop target.
        // The original curvature/acceleration cap remains authoritative and
        // can still require normal braking or real measured reentry.
        const double jerk_target_cap=std::min(cap,jerkBrakingSpeedLimit(remaining,along_acceleration,
          .5*config_.max_acceleration*planar_scale_,.5*config_.max_jerk*planar_scale_,
          config_.max_speed*planar_scale_));
        control_diagnostic_.jerk_target_cap=jerk_target_cap;
        if(world.norm()>jerk_target_cap)world*=jerk_target_cap/world.norm();
        if(config_.align_then_forward&&isFinalCurveEndpoint()) {
          const double actual_speed=std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm());
          const double stop_distance=jerkStoppingDistance(std::max(actual_speed,previous.norm()),
            along_acceleration,config_.max_acceleration*planar_scale_,config_.max_jerk*planar_scale_);
          if(remaining<=stop_distance&&actual_speed>config_.stationary_linear_threshold_mps)terminal_settling_=true;
          if(terminal_settling_) {
            if(actual_speed<=config_.stationary_linear_threshold_mps&&previous.norm()<=config_.stationary_linear_threshold_mps)
              terminal_settling_=false;
            else {
              control_diagnostic_.zero_target_cause="terminal_s_curve_deceleration";
              auto out=alignThenForward(tangent,cross_error,Eigen::Vector2d::Zero(),velocity,cap,dt);
              if(out.reason=="tracking_forward"||out.reason=="tracking_forward_yaw_coasting") {
                out.reason="goal_position_settling";last_output_=out;reason_=out.reason;
              }
              return out;
            }
          }
        }
      }
      const bool at_local_end=arcTable().back()-measured_arc_<=.02&&
        (positionAt(duration_).head<2>()-odom_.position.head<2>()).norm()<=.02&&
        cap<=config_.stationary_linear_threshold_mps;
      if(local_endpoint_waiting_||at_local_end) {
        local_endpoint_waiting_=true;
        return holonomicLimited(Eigen::Vector2d::Zero(),0.,cap,config_.max_acceleration*planar_scale_,
          dt,"local_segment_finished_waiting_replan",true);
      }
      if(config_.align_then_forward)
        return alignThenForward(tangent,cross_error,world,velocity,cap,dt);
      if(config_.holonomic_yaw_policy=="path_tangent")
        holonomic_heading_=prepared_geometry_->body_heading_seed_yaw?
          prepared_geometry_->bodyHeadingAt(execution_time_):
          scan_planner::bodyHeadingFromVelocity(velocity,holonomic_heading_,scan_planner::SplineHeadingContract{});
      // The author uses a look-ahead chord. Our CURRENT Native certificate
      // instead binds body yaw to the current spline tangent, so use that same
      // heading here until both sides explicitly support a different contract.
      // P yaw has no measured-rate prediction, coasting state or yaw deadband.
      // Current-tangent kinematic feed-forward below compensates turn rate
      // without changing that certified heading or assuming robot inertia.
      // A large error asks for XY deceleration, not task retirement.
      const double heading_error=angle(holonomic_heading_-odom_.yaw);
      const double yaw=config_.kp_yaw*heading_error-
        config_.yaw_damping_gain*projectedBodyHeadingRate(odom_.orientation,odom_.angular_velocity_in_frame);
      const double c=std::cos(odom_.yaw),s=std::sin(odom_.yaw);
      const double heading_weight=config_.holonomic_yaw_policy=="path_tangent"?
        headingTranslationWeight(heading_error,config_.heading_threshold):1.;
      const bool large_heading=heading_weight==0.;
      world*=heading_weight;
      const Eigen::Vector2d body(c*world.x()+s*world.y(),-s*world.x()+c*world.y());
      const Eigen::Vector2d body_tangent(c*tangent.x()+s*tangent.y(),-s*tangent.x()+c*tangent.y());
      const double curvature=config_.holonomic_yaw_policy=="path_tangent"?headingCurvature(velocity):0.;
      if(!finite(curvature)){invalidateTrajectory("nonfinite_controller_output");return stop();}
      control_diagnostic_.heading_error=heading_error;
      control_diagnostic_.heading_weight=heading_weight;
      control_diagnostic_.forward_target=body.x();
      control_diagnostic_.measured_forward=c*odom_.velocity_in_frame.x()+s*odom_.velocity_in_frame.y();
      control_diagnostic_.yaw_target=std::clamp(yaw,-config_.max_yaw_rate,config_.max_yaw_rate);
      if(large_heading)control_diagnostic_.zero_target_cause="heading_feasibility";
      return holonomicLimited(body,yaw,cap,config_.max_acceleration*planar_scale_,dt,
        large_heading?"heading_alignment":"tracking",false,curvature,body_tangent);
    }
    // Keep the legacy forward-only controller's compatibility semantics.
    const double look_time = std::min(duration_, execution_time_ + config_.lookahead);
    const Eigen::Vector3d desired = positionAt(look_time);
    const Eigen::Vector3d velocity = velocityAt(look_time);
    const Eigen::Vector2d look_error = desired.head<2>() - odom_.position.head<2>();
    const Eigen::Vector2d world = config_.kp_position * look_error +
        (look_time < duration_ ? Eigen::Vector2d(velocity.head<2>()) : Eigen::Vector2d::Zero());
    if (!world.allFinite() || !desired.allFinite()) {
      invalidateTrajectory("nonfinite_controller_output");return stop();
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
    path_turn_intent_.reset();goal_turn_intent_.reset();
    stair_steering_history_.reset();stair_steering_scope_valid_=false;
    turn_phase_=TurnPhase::Following;turn_first_engaged_=false;resetTurnStationaryWindow();
    aligned_entry_curve_id_=-1;aligned_entry_stamp_=0.;
    if (reason != "goal_reached") finished_ = false;
    invalidateTrajectory(reason);
  }
  void hold(const std::string& reason,double received) {
    (void)received;
    holding_=true; recovery_samples_=0; recovery_first_source_=0.; reason_=reason;
    resetTurnStationaryWindow();
    resetCommandAcceleration();
    last_output_=Output{};
  }
  bool holding() const { return holding_; }
  Output align(double desired_yaw,double tolerance,double now,double receipt,bool fresh_external_curve_lease=false) {
    control_diagnostic_=ControlDiagnostic{};
    const double dt=receipt-last_step_;
    const double source_dt=now-last_ros_time_;
    const double previous_yaw_command=last_output_.yaw_rate;
    const auto previous_output=last_output_;
    const auto previous_planar_acceleration=command_planar_acceleration_;
    const auto previous_forward_response=forward_response_limiter_;
    const auto previous_yaw_response=yaw_response_limiter_;
    const auto previous_forward_conditioner=forward_conditioner_;
    const auto previous_yaw_conditioner=yaw_conditioner_;
    const double previous_output_frame_yaw=output_frame_yaw_;
    const bool previous_output_frame_valid=output_frame_yaw_valid_;
    const double previous_yaw_acceleration=command_yaw_acceleration_;
    const auto guarded=step(now,receipt,fresh_external_curve_lease);
    if(!active_ || holding_ || !have_odom_ || !trajectory_ || dt<=0. || dt>.1 ||source_dt<=0.||source_dt>.1||
       !fresh(now,odom_.stamp,config_.odom_timeout,.02) || !sourceEvidenceFresh(odom_,now) ||
       !supportContractAllowsBody() ||
       !finite(desired_yaw) || !finite(tolerance) || tolerance<.01 || tolerance>.5) return stop();
    (void)guarded;
    Output out;
    out.reason="aligning";out.frozen=true;
    const double shortest_error=angle(desired_yaw-odom_.yaw);
    const double error=config_.align_then_forward?goal_turn_intent_.resolve(shortest_error):shortest_error;
    double requested=std::abs(error)<=tolerance?0.:
      std::clamp(config_.kp_yaw*error-config_.yaw_damping_gain*
        projectedBodyHeadingRate(odom_.orientation,odom_.angular_velocity_in_frame),
        -config_.max_yaw_rate,config_.max_yaw_rate);
    if(holonomic()) {
      last_output_=previous_output;
      command_planar_acceleration_=previous_planar_acceleration;
      forward_response_limiter_=previous_forward_response;
      yaw_response_limiter_=previous_yaw_response;
      forward_conditioner_=previous_forward_conditioner;
      yaw_conditioner_=previous_yaw_conditioner;
      output_frame_yaw_=previous_output_frame_yaw;
      output_frame_yaw_valid_=previous_output_frame_valid;
      command_yaw_acceleration_=previous_yaw_acceleration;
      // Once terminal translation is already physically settled, alignment
      // must not resurrect the saved approach command after step() stopped it.
      // Keep yaw history, but its ordinary XY deceleration ramp is no longer
      // a requested motion at this completed position. Nonterminal alignment
      // retains the existing planar limiter behavior.
      const bool terminal_translation_settled=
        (task_.goal.head<2>()-odom_.position.head<2>()).norm()<=config_.goal_tolerance&&
        std::abs(task_.goal.z()-odom_.position.z())<=config_.goal_height_tolerance&&
        std::max(odom_.planar_speed,odom_.velocity_in_frame.head<2>().norm())<=
          config_.stationary_linear_threshold_mps;
      if(terminal_translation_settled) {
        last_output_.forward=last_output_.lateral=0.;command_planar_acceleration_.setZero();
        forward_response_limiter_.reset();
        forward_conditioner_.reset();
      }
      if(terminal_translation_settled&&std::abs(error)<=tolerance&&
         std::abs(odom_.angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps) {
        // Both explicit goal pose and actual rest are already true. A saved
        // path-yaw command must not resume after completion or demand an
        // unchecked final rotation solely to drain its limiter history.
        reason_="aligning";return stop();
      }
      // Terminal heading uses the explicit goal tolerance, not the path's
      // noise band or an assumed .8-second turn coast. A new signed target
      // crosses zero through the same normal yaw limiter as ordinary tracking.
      auto result=holonomicLimited(Eigen::Vector2d::Zero(),requested,config_.max_speed*planar_scale_,
        config_.max_acceleration*planar_scale_,dt,"aligning",true);
      return result;
    }
    out.yaw_rate=std::clamp(requested,previous_yaw_command-config_.max_yaw_acceleration*dt,
                          previous_yaw_command+config_.max_yaw_acceleration*dt);
    last_output_=out; reason_=out.reason; return out;
  }
  void refreshTaskLease(double receipt) { if(active_ && finite(receipt)) task_received_=receipt; }
  // A query of fresh physical state/progress, never a collision verdict or
  // permission. The formal follower also uses it while Native's proof is
  // unavailable, retaining zero output and the existing reseed reason.
  bool needsTerminalReplan(double now)const {
    if(!active_||!trajectory_||!have_odom_||!fresh(now,odom_.stamp,config_.odom_timeout,.02)||
       !sourceEvidenceFresh(odom_,now))return false;
    if(local_endpoint_waiting_||braking_reentry_required_)return true;
    if(!atTerminalProjection())return false;
    const double residual=(positionAt(duration_)-odom_.position).norm();
    if(residual>.15)return true;
    return !isFinalCurveEndpoint()&&residual<=.02&&
      prepared_geometry_->brakingLimitAt(measured_arc_,.5*config_.max_acceleration)<=config_.stationary_linear_threshold_mps;
  }
  // Update only spatial progress from the latest real sample, before a local
  // splice decision. This neither advances a timer nor changes limiter state.
  // Without it, a 20 Hz executor would test the preceding sample's arc and
  // miss a 1.25 cm seam when travelling 0.3 m/s (1.5 cm per tick).
  bool refreshMeasuredProgress(double now) {
    if(!active_||!trajectory_||!have_odom_||holding_||
       !fresh(now,odom_.stamp,config_.odom_timeout,.02)||!sourceEvidenceFresh(odom_,now)||
       !supportContractAllowsBody())return false;
    if(odom_.stamp<=last_projected_stamp_)return true;
    const double dt=last_projected_stamp_>0.?odom_.stamp-last_projected_stamp_:0.;
    if(!projectMeasured(std::min(config_.projection_max_forward_m,
         config_.projection_forward_m+odom_.planar_speed*std::max(0.,dt))))return false;
    last_projected_stamp_=odom_.stamp;last_projected_position_=odom_.position;return true;
  }
  void suspendOutput(double now,double receipt,const std::string& reason,bool recovery) {
    control_diagnostic_=ControlDiagnostic{};
    resetTurnStationaryWindow();
    if(recovery)hold(reason,receipt);
    if(!recovery&&active_&&trajectory_&&have_odom_&&!holding_&&
       fresh(now,odom_.stamp,config_.odom_timeout,.02)&&sourceEvidenceFresh(odom_,now)&&
       supportContractAllowsBody()&&
       odom_.stamp>last_projected_stamp_) {
      const double dt=last_projected_stamp_>0.?odom_.stamp-last_projected_stamp_:0.;
      if(projectMeasured(std::min(config_.projection_max_forward_m,
          config_.projection_forward_m+odom_.planar_speed*std::max(0.,dt)))) {
        last_projected_stamp_=odom_.stamp;last_projected_position_=odom_.position;
      } else hold("measured_projection_outside_window",receipt);
    }
    last_step_=receipt;last_ros_time_=now;last_output_=Output{};reason_=reason;resetCommandAcceleration();
  }
  // Rebase only command history after the sole writer reports an actual
  // forced-zero send. Keep real measured progress, geometry, task, clock and
  // stationary observations; a submitted command is not a physical stop.
  void reconcileSubmittedZero() {
    last_output_=Output{};
    resetCommandAcceleration();
  }
  void reconcileSubmittedVelocity(double forward,double lateral,double yaw) {
    reconcileSubmittedZero();
    last_output_.forward=forward;last_output_.lateral=lateral;last_output_.yaw_rate=yaw;
    forward_conditioner_.reset(forward);yaw_conditioner_.reset(yaw);rememberOutputFrame();
  }
  struct CommandHistory {
    Output output;
    Eigen::Vector2d planar_acceleration;
    double frame_yaw,yaw_acceleration;
    bool frame_yaw_valid;
    ResponseCommandLimiter forward_response,yaw_response;
    MotionCommandConditioner forward,yaw;
  };
  CommandHistory commandHistory()const {
    return {last_output_,command_planar_acceleration_,output_frame_yaw_,command_yaw_acceleration_,
      output_frame_yaw_valid_,forward_response_limiter_,yaw_response_limiter_,forward_conditioner_,yaw_conditioner_};
  }
  void reconcileSubmittedCommand(const CommandHistory& h) {
    last_output_=h.output;command_planar_acceleration_=h.planar_acceleration;
    output_frame_yaw_=h.frame_yaw;command_yaw_acceleration_=h.yaw_acceleration;
    output_frame_yaw_valid_=h.frame_yaw_valid;forward_response_limiter_=h.forward_response;
    yaw_response_limiter_=h.yaw_response;forward_conditioner_=h.forward;yaw_conditioner_=h.yaw;
  }
  // An interrupted output is not a measured stop. Keep the last continuous
  // history only to the extent that the CURRENT original-source body motion
  // supports it. This never increases its magnitude, grants geometry, or
  // alters progress/clock/stationary evidence. Ordinary ticks never call it.
  std::optional<Eigen::Vector3d> measuredContinuityBoundary(double now)const {
    if(!active_||!trajectory_||!have_odom_||holding_||
       !fresh(now,odom_.stamp,config_.odom_timeout,.02)||!sourceEvidenceFresh(odom_,now))return {};
    Eigen::Vector2d previous(last_output_.forward,last_output_.lateral);
    if(output_frame_yaw_valid_) {
      const double delta=output_frame_yaw_-odom_.yaw,c=std::cos(delta),s=std::sin(delta);
      previous={c*previous.x()-s*previous.y(),s*previous.x()+c*previous.y()};
    }
    const double c=std::cos(odom_.yaw),s=std::sin(odom_.yaw);
    const Eigen::Vector2d measured(c*odom_.velocity_in_frame.x()+s*odom_.velocity_in_frame.y(),
      -s*odom_.velocity_in_frame.x()+c*odom_.velocity_in_frame.y());
    Eigen::Vector2d bounded=Eigen::Vector2d::Zero();
    if(config_.align_then_forward||!holonomic()) {
      bounded.x()=std::min({std::max(0.,previous.x()),std::max(0.,measured.x()),config_.max_speed*planar_scale_});
    } else if(previous.norm()>0.) {
      const auto direction=previous.normalized();
      bounded=direction*std::min({previous.norm(),std::max(0.,measured.dot(direction)),config_.max_speed*planar_scale_});
    }
    const double measured_yaw=projectedBodyHeadingRate(odom_.orientation,odom_.angular_velocity_in_frame);
    const double old_yaw=last_output_.yaw_rate;
    const double yaw=std::copysign(std::min({std::abs(old_yaw),
      std::max(0.,std::copysign(1.,old_yaw)*measured_yaw),config_.max_yaw_rate}),old_yaw);
    return Eigen::Vector3d{bounded.x(),bounded.y(),yaw};
  }
  bool commandWithinMeasuredContinuity(double now)const {
    const auto boundary=measuredContinuityBoundary(now);
    return boundary&&std::abs(boundary->x()-last_output_.forward)<=1e-12&&
      std::abs(boundary->y()-last_output_.lateral)<=1e-12&&std::abs(boundary->z()-last_output_.yaw_rate)<=1e-12;
  }
  bool reconcileMeasuredContinuity(double now) {
    const auto boundary=measuredContinuityBoundary(now);
    if(!boundary||commandWithinMeasuredContinuity(now))return false;
    // A measured correction of one axis is not a discontinuity of every
    // command axis. In particular, a yaw boundary after a short send gap must
    // not discard a still-supported forward acceleration or its SDK-image
    // derivative. Only the actually rebased axis loses those histories;
    // no derivative here is a measurement of physical acceleration.
    if(boundary->x()!=last_output_.forward||boundary->y()!=last_output_.lateral) {
      const double projection=output_frame_yaw_valid_?
        std::max(0.,std::cos(output_frame_yaw_-odom_.yaw)):0.;
      const bool supported_frame_change=config_.align_then_forward&&output_frame_yaw_valid_&&
        last_output_.lateral==0.&&boundary->y()==0.&&boundary->x()>0.&&
        boundary->x()==last_output_.forward*projection&&
        forward_conditioner_.state().command==last_output_.forward;
      if(supported_frame_change) {
        forward_conditioner_.projectForwardHistory(projection,config_.forward_response);
        command_planar_acceleration_={forward_conditioner_.state().acceleration,0.};
      } else {
        command_planar_acceleration_.setZero();forward_response_limiter_.reset();
        forward_conditioner_.reset(boundary->x());
      }
      last_output_.forward=boundary->x();last_output_.lateral=boundary->y();
    }
    if(boundary->z()!=last_output_.yaw_rate) {
      last_output_.yaw_rate=boundary->z();command_yaw_acceleration_=0.;
      yaw_response_limiter_.reset();yaw_conditioner_.reset(boundary->z());
    }
    rememberOutputFrame();return true;
  }
  void setSupport(const SupportEvidence& support) {
    if(!support_index_||!support_||!sameSupportEvidenceGeometry(support,*support_))
      support_index_=std::make_shared<SupportIndex>(support.ground);
    support_=std::make_shared<const SupportEvidence>(support);
  }
  // Preparation is pure with respect to the current controller. At commit we
  // repeat admission against the latest body and retain the limiter history.
  bool admitRevision(const Task& task,const Trajectory& trajectory,const SupportEvidence& support,
                     double now,double receipt,bool commit,bool fresh_external_curve_lease=false) {
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
    if(!trial.receiveTrajectory(trajectory,now,receipt,fresh_external_curve_lease)) {
      candidate_reason_=trial.candidate_reason_.empty()?"candidate_rejected":trial.candidate_reason_;
      join_diagnostic_=trial.join_diagnostic_;
      return false;
    }
    if(commit) *this=std::move(trial); else candidate_reason_.clear();
    return true;
  }
  // A rejected asynchronous commit restores its incumbent GEOMETRY, never a
  // snapshot of the running controller. New observations/faults, source-order
  // floors, HOLD recovery, cancellation, output/conditioner history and control
  // clocks can all advance while Native considers the transaction.
  bool rollbackGeometry(const TrackerCore& incumbent,double now,bool preserve_live_stair_history=false) {
    if(!active_||(incumbent.active_&&!task_.identity.sameTask(incumbent.task_.identity)))return false;
    if(incumbent.active_)task_=incumbent.task_;
    support_=incumbent.support_;support_index_=incumbent.support_index_;
    trajectory_=incumbent.trajectory_;velocity_=incumbent.velocity_;
    prepared_geometry_=incumbent.prepared_geometry_;curve_identity_=incumbent.curve_identity_;
    trajectory_received_=incumbent.trajectory_received_;trajectory_start_=incumbent.trajectory_start_;
    duration_=incumbent.duration_;trajectory_min_z_=incumbent.trajectory_min_z_;
    trajectory_max_z_=incumbent.trajectory_max_z_;planar_scale_=incumbent.planar_scale_;
    holonomic_heading_valid_=incumbent.holonomic_heading_valid_;holonomic_heading_=incumbent.holonomic_heading_;
    last_trajectory_id_=incumbent.last_trajectory_id_;execution_time_=incumbent.execution_time_;
    measured_arc_=incumbent.measured_arc_;committed_arc_=incumbent.committed_arc_;
    last_projected_stamp_=incumbent.last_projected_stamp_;last_projected_position_=incumbent.last_projected_position_;
    // Finishing the rejected candidate's finite window says nothing about
    // the restored curve's endpoint. Keep this geometry-owned latch with its
    // curve; otherwise needsTerminalReplan() strands a still-long incumbent
    // forever despite fresh body/Native evidence. The projection below still
    // detects an actually exhausted incumbent from the latest measured pose.
    // Do not restore live HOLD, braking faults, goal settling or limiter state.
    local_endpoint_waiting_=incumbent.local_endpoint_waiting_;
    // Rejection restores the previously accepted exit context. Promotion of
    // an accepted candidate is different: the robot may have crossed that
    // exit while its ACK was in flight. Keep the LIVE context in that case;
    // the normal task/model/anchor scope check still rejects an obsolete one.
    if(!preserve_live_stair_history) {
      stair_steering_history_=incumbent.stair_steering_history_;
      stair_steering_identity_=incumbent.stair_steering_identity_;
      stair_steering_model_hash_=incumbent.stair_steering_model_hash_;
      stair_steering_scope_valid_=incumbent.stair_steering_scope_valid_;
    }
    // First-admission rollback removes geometry, not the task or its newly
    // acquired HOLD. Making it idle would let the next admission reset that
    // recovery window. Latest actual state is not re-fed (the same timestamp
    // is rightly rejected by receiveOdom).
    refreshMeasuredProgress(now);
    return true;
  }
  // Restore only the exact identity already selected by the writer, after its
  // expired ACK forced HOLD and cleared geometry. The caller binds that fact;
  // this still performs the normal fresh support, derivative, actual-entry
  // and limiter admission. It neither resumes authority nor re-dates a source.
  bool admitAppliedGeometry(const Task& task,const Trajectory& trajectory,const SupportEvidence& support,
      double now,double receipt,bool commit) {
    if(!active_||hasInstalledGeometry()||trajectory.id!=last_trajectory_id_||
       trajectory.generation!=task_.generation||!(task.identity==task_.identity)||
       !trajectory.entry_reobserved)return candidateReject("applied_geometry_recovery_identity_mismatch");
    TrackerCore trial(*this);trial.last_trajectory_id_=-1;
    if(!trial.admitRevision(task,trajectory,support,now,receipt,true,true)) {
      candidate_reason_=trial.candidate_reason_;join_diagnostic_=trial.join_diagnostic_;return false;
    }
    if(commit)*this=std::move(trial);else candidate_reason_.clear();
    return true;
  }
  const Task& task() const { return task_; }
  const Odom& odometry() const { return odom_; }
  struct EntryBoundary {Eigen::Vector3d position,velocity;double time,position_error,velocity_error;};
  // Keep prepared geometry private, but refresh its boundary from the SAME
  // measured state and limiter history as the incumbent. No elapsed-wall-clock
  // position prediction and no spline rebuild on the control lane.
  std::optional<EntryBoundary> refreshPreparedState(const TrackerCore& current,double now,double receipt) {
    if(!trajectory_||!velocity_||!current.have_odom_||current.holding_||
       !fresh(now,current.odom_.stamp,.1,.02)||!sourceEvidenceFresh(current.odom_,now))return {};
    const double dt=std::max(0.,current.odom_.stamp-last_projected_stamp_);
    const bool strict_splice=prepared_geometry_->entry.strict_splice_admission;
    const bool rolling=config_.usesNativeRollingConnection(strict_splice);
    const double connection_limit=config_.admissionPositionLimit(strict_splice);
    const double travel_speed=rolling?prepared_geometry_->initial_motion_limits.peak_speed:config_.max_speed;
    const double travel=std::min(config_.admissionForwardLimit(strict_splice),travel_speed*dt+connection_limit);
    const auto p=projectCurveAdmission(*prepared_geometry_->entry.curve_sampler,timeTable(),arcTable(),pointTable(),current.odom_.position,
      execution_time_,measured_arc_,std::min(config_.projection_backtrack_m,travel),travel,connection_limit,
      config_.admissionPolicy(strict_splice),prepared_geometry_->entry.origin_arc);
    if(!p)return {};
    const auto position=positionAt(p->time),velocity=velocityAt(p->time);
    const double pe=(position-current.odom_.position).norm(),ve=(velocity-current.odom_.velocity_in_frame).norm();
    if(pe>connection_limit||(!rolling&&ve>.05))return {};
    odom_=current.odom_;have_odom_=current.have_odom_;odom_received_=current.odom_received_;
    last_odom_source_stamp_=current.last_odom_source_stamp_;last_output_=current.last_output_;
    command_planar_acceleration_=current.command_planar_acceleration_;
    forward_response_limiter_=current.forward_response_limiter_;
    yaw_response_limiter_=current.yaw_response_limiter_;
    forward_conditioner_=current.forward_conditioner_;
    yaw_conditioner_=current.yaw_conditioner_;
    last_steering_sample_=current.last_steering_sample_;
    path_turn_intent_=current.path_turn_intent_;goal_turn_intent_=current.goal_turn_intent_;
    stair_steering_history_=current.stair_steering_history_;
    stair_steering_identity_=current.stair_steering_identity_;
    stair_steering_model_hash_=current.stair_steering_model_hash_;
    stair_steering_scope_valid_=current.stair_steering_scope_valid_;
    output_frame_yaw_=current.output_frame_yaw_;
    output_frame_yaw_valid_=current.output_frame_yaw_valid_;
    command_yaw_acceleration_=current.command_yaw_acceleration_;
    terminal_settling_=current.terminal_settling_;
    last_step_=current.last_step_;last_ros_time_=current.last_ros_time_;
    holding_=current.holding_;recovery_samples_=current.recovery_samples_;recovery_first_source_=current.recovery_first_source_;
    execution_time_=p->time;measured_arc_=p->arc;committed_arc_=std::max(committed_arc_,p->arc);
    if(holonomic())planar_scale_=prepared_geometry_->scaleAt(execution_time_);
    last_projected_stamp_=odom_.stamp;last_projected_position_=odom_.position;task_received_=receipt;
    return EntryBoundary{position,velocity,p->time,pe,ve};
  }
  void recordAppliedOutput(double forward,double yaw,double now,double receipt) {
    recordAppliedOutput(forward,0.,yaw,now,receipt);
  }
  void recordAppliedOutput(double forward,double lateral,double yaw,double now,double receipt) {
    if(forward!=last_output_.forward||lateral!=last_output_.lateral||yaw!=last_output_.yaw_rate) {
      resetCommandAcceleration();
      forward_conditioner_.reset(forward);yaw_conditioner_.reset(yaw);
    }
    last_output_.forward=forward;last_output_.lateral=lateral;last_output_.yaw_rate=yaw;
    rememberOutputFrame();
    last_step_=receipt;last_ros_time_=now;
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
  const ControlDiagnostic& controlDiagnostic()const {return control_diagnostic_;}
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
  bool holonomic()const {return config_.motion_model=="holonomic";}
  Output alignThenForward(const Eigen::Vector2d& tangent,const Eigen::Vector2d& cross_error,
      const Eigen::Vector2d& world,const Eigen::Vector3d& curve_velocity,double cap,double dt) {
    if(tangent.norm()<1e-9) {
      control_diagnostic_.zero_target_cause="direction_unresolved";
      return holonomicLimited(Eigen::Vector2d::Zero(),0.,cap,
        config_.max_acceleration*planar_scale_,dt,"waiting_curve_direction",true);
    }
    const double current_curvature=headingCurvature(curve_velocity);
    const double preview=std::clamp(std::max(odom_.planar_speed,last_output_.forward)*config_.lookahead,
      config_.min_lookahead_distance_m,config_.max_lookahead_distance_m);
    const double horizontal=prepared_geometry_->horizontalAt(measured_arc_);
    const double preview_arc=prepared_geometry_->arcAtHorizontal(horizontal+preview);
    const double preview_time=timeAtArc(preview_arc);
    const Eigen::Vector2d preview_position=positionAt(preview_time).head<2>();
    SteeringPreviewSample steering_sample{true,last_trajectory_id_,odom_.source_stamp_ns,
      execution_time_,measured_arc_,committed_arc_,preview_time,preview_arc,preview_position};
    const Eigen::Vector2d chord=preview_position-
      positionAt(execution_time_).head<2>();
    // Repeated measured-start solves legitimately have zero initial lateral
    // acceleration. Looking only at that point would keep postponing every
    // bend as the candidate is replaced. Preview THIS curve's geometry while
    // keeping actual progress, current cross-track feedback and one limiter.
    // The short reverse-boundary profile is a different immutable body-heading
    // geometry, not a raw velocity tangent. Keep its established derivative
    // for the whole original curve; never switch laws at its internal boundary.
    const bool resolved_entry=bool(prepared_geometry_->body_heading_contract.forward_entry_profile);
    double curvature=resolved_entry?current_curvature:
      previewReferenceCurvature(tangent,chord,current_curvature);
    Eigen::Vector2d steering_tangent=tangent,steering_cross=cross_error;
    double lateral_preview=preview,stair_weight=0.,steering_preview_distance=chord.norm();
    // No SDK mode or broad segment label enables this law. Only finite XYZ
    // flight ranges of the ACCEPTED curve can select it. The original spline,
    // Native body-heading certificate and measured progress stay unchanged.
    const auto& model=prepared_geometry_->support;
    const std::string no_model;
    const std::string& model_hash=model&&model->model_geometry_valid?model->model_hash:no_model;
    if(!stair_steering_scope_valid_||!stair_steering_identity_.sameTask(curve_identity_)||
        stair_steering_identity_.anchor_id!=curve_identity_.anchor_id||
        stair_steering_identity_.anchor_revision!=curve_identity_.anchor_revision||
        stair_steering_identity_.map_geometry_revision!=curve_identity_.map_geometry_revision||
        stair_steering_model_hash_!=model_hash) {
      stair_steering_history_.reset();stair_steering_identity_=curve_identity_;
      stair_steering_model_hash_=model_hash;stair_steering_scope_valid_=true;
    }
    if(!resolved_entry&&!prepared_geometry_->stair_flights.empty()) {
      const double body_length=2.*prepared_geometry_->body_heading_contract.body_extent;
      const double preceding_exit_distance=stair_steering_history_.precedingExitDistance(
        prepared_geometry_->stair_flights,horizontal,body_length,positionAt(execution_time_));
      const auto stair=stairSteeringPreview(prepared_geometry_->stair_flights,horizontal,
        preview,body_length,preceding_exit_distance);
      const auto point=positionAt(timeAtArc(prepared_geometry_->arcAtHorizontal(stair.horizontal)));
      const Eigen::Vector2d ahead=point.head<2>()-positionAt(execution_time_).head<2>();
      if(stair.weight>0.&&ahead.norm()>1e-6) {
        steering_sample.preview_arc=prepared_geometry_->arcAtHorizontal(stair.horizontal);
        steering_sample.preview_time=timeAtArc(steering_sample.preview_arc);
        steering_sample.target=point.head<2>();
        const double path_yaw=std::atan2(tangent.y(),tangent.x());
        const double preview_yaw=std::atan2(ahead.y(),ahead.x());
        const double yaw=path_yaw+stair.weight*angle(preview_yaw-path_yaw);
        steering_tangent={std::cos(yaw),std::sin(yaw)};
        const Eigen::Vector2d error=positionAt(execution_time_).head<2>()-odom_.position.head<2>();
        steering_cross=error-steering_tangent*error.dot(steering_tangent);
        lateral_preview=preview+stair.weight*(stair.lateral_distance-preview);
        // The long chord already anticipates this accepted curve's bend.
        // Adding its short-tangent curvature again caused alternating large
        // turns on small spline ripples. Fade the old feedforward at entry
        // and restore it continuously before the flight's real exit.
        curvature*=1.-stair.weight;stair_weight=stair.weight;
        steering_preview_distance+=stair.weight*(ahead.norm()-steering_preview_distance);
      }
    }
    if(!finite(curvature)){invalidateTrajectory("nonfinite_controller_output");return stop();}
    auto unconstrained=forwardSteeringTarget({lateral_preview,config_.steering_error_scale_rad,
      config_.kp_yaw,config_.yaw_damping_gain},steering_tangent,steering_cross,
      world.dot(tangent.normalized()),odom_.yaw,
      projectedBodyHeadingRate(odom_.orientation,odom_.angular_velocity_in_frame),curvature);
    // Retain preview anticipation, but do not amplify it again through the
    // damping reference. The measured rate is compared with THIS accepted
    // curve's current heading derivative. Below the existing direction-speed
    // contract the heading is held, so retain the old law there; a C2 weight
    // admits the derivative only as that same direction becomes observable.
    // Full flight and resolved entry keep their sealed laws.
    if(!resolved_entry) {
      const double direction_weight=smoothUnitInterval(curve_velocity.head<2>().norm()/
        prepared_geometry_->body_heading_contract.minimum_direction_speed-1.);
      unconstrained.damping_curvature_offset=direction_weight*
        (1.-smoothUnitInterval(stair_weight))*(current_curvature-curvature);
    }
    // Native certifies either rear turn at this immutable PATH heading, not
    // at a heading modified by lateral feedback. A front-facing path releases
    // the remembered branch even if cross-track correction crosses 90 deg.
    // Neither release nor a curve replacement resets the command conditioner.
    if(scan_planner::rearBodyHeadingTransition(odom_.yaw,unconstrained.path_heading))
      unconstrained.heading_error=path_turn_intent_.resolve(unconstrained.heading_error);
    else path_turn_intent_.reset();
    unconstrained.yaw_feedback=config_.kp_yaw*softTrackingError(
      unconstrained.heading_error,config_.steering_error_scale_rad);
    // Use the same acquired odometry as projection/feedback. SDK input
    // percentage is not a physical speed calibration; no new source clock,
    // response model, or second output filter is introduced here.
    unconstrained.measured_forward=std::cos(odom_.yaw)*odom_.velocity_in_frame.x()+
      std::sin(odom_.yaw)*odom_.velocity_in_frame.y();
    const auto target=unconstrained.withTwistLimits(cap,config_.max_yaw_rate);
    if(!target.valid){invalidateTrajectory("nonfinite_controller_output");return stop();}
    holonomic_heading_=target.path_heading;
    control_diagnostic_.steering_curvature=curvature;
    control_diagnostic_.steering_current_curvature=current_curvature;
    control_diagnostic_.steering_preview_distance=steering_preview_distance;
    control_diagnostic_.steering_curvature_source=resolved_entry?"resolved_entry_heading":
      (stair_weight>0.?"accepted_stair_flight_preview":"accepted_curve_spatial_preview");
    control_diagnostic_.forward_entry_heading_active=
      execution_time_<prepared_geometry_->body_heading_contract.forward_entry_end;
    control_diagnostic_.unallocated_forward_target=std::min(unconstrained.forward,cap);
    control_diagnostic_.unallocated_yaw_target=unconstrained.yawAtForward(control_diagnostic_.unallocated_forward_target);
    control_diagnostic_.turn_allocation_scale=target.correction_scale;
    control_diagnostic_.heading_error=target.heading_error;control_diagnostic_.heading_weight=target.heading_weight;
    control_diagnostic_.forward_target=target.forward;
    control_diagnostic_.measured_forward=unconstrained.measured_forward;
    if(target.forward==0.&&control_diagnostic_.zero_target_cause.empty())
      control_diagnostic_.zero_target_cause=target.heading_weight<=1e-9?"heading_feasibility":
        "nonpositive_forward_after_velocity_feedback";
    const double previous_yaw_command=last_output_.yaw_rate;
    auto out=holonomicLimited({target.forward,0.},0.,cap,config_.max_acceleration*planar_scale_,dt,
      "tracking_forward",false,0.,Eigen::Vector2d::Zero(),false);
    if(out.reason!="tracking_forward")return out;
    // Same continuous law at every tick, including curve replacement. The
    // sole conditioner preserves its state; no correction/settling latch.
    const double yaw_target=target.yawAtForward(out.forward);
    // Passive inputs and exact components of THIS command, not a second
    // steering law. Retain the preceding control sample with its own original
    // body source/curve so a 1 Hz report can expose a target or progress jump.
    control_diagnostic_.steering_sample=steering_sample;
    control_diagnostic_.previous_steering_sample=last_steering_sample_;
    last_steering_sample_=steering_sample;
    control_diagnostic_.steering_tangent_yaw=target.path_heading;
    control_diagnostic_.steering_cross_error=steering_tangent.normalized().x()*steering_cross.y()-
      steering_tangent.normalized().y()*steering_cross.x();
    control_diagnostic_.steering_flight_weight=stair_weight;
    const double steering_speed=out.forward>0.?
      std::max(out.forward,std::max(0.,target.measured_forward))*target.heading_weight:0.;
    control_diagnostic_.steering_yaw_feedforward=target.curvatureYawAtSpeed(steering_speed);
    control_diagnostic_.steering_yaw_feedback=target.correction_scale*target.yaw_feedback;
    control_diagnostic_.steering_yaw_damping=-target.correction_scale*target.yaw_damping*target.measured_yaw_rate;
    control_diagnostic_.steering_phase="continuous_path_tracking";
    control_diagnostic_.yaw_target=yaw_target;
    out.yaw_rate=limitedYaw(yaw_target,previous_yaw_command,dt);
    if(yaw_target==0.&&out.yaw_rate!=0.)out.reason="tracking_forward_yaw_coasting";
    reason_=out.reason;
    last_output_=out;
    return out;
  }
  bool atTerminalProjection()const {
    return trajectory_&&prepared_geometry_&&arcTable().back()-measured_arc_<=.02;
  }
  bool isFinalCurveEndpoint()const {
    return trajectory_&&prepared_geometry_&&(positionAt(duration_)-task_.goal).norm()<=config_.join_limit;
  }
  Output finalEndpointCapture(double dt,const std::string& reason,bool frozen) {
    // This is the CURRENT immutable curve endpoint, not an arbitrary task
    // goal shortcut. SCAN renews the actual body-to-curve connector and its
    // support/collision evidence within .15 m; larger residuals reseed above.
    const Eigen::Vector2d error=(positionAt(duration_)-odom_.position).head<2>();
    const Eigen::Vector2d world=config_.kp_position*error-
      config_.velocity_damping_gain*odom_.velocity_in_frame.head<2>();
    const double c=std::cos(odom_.yaw),s=std::sin(odom_.yaw);
    const Eigen::Vector2d body(c*world.x()+s*world.y(),-s*world.x()+c*world.y());
    const double acceleration=config_.max_acceleration*planar_scale_;
    const double reachable_floor=std::max(0.,std::hypot(last_output_.forward,last_output_.lateral)-acceleration*dt);
    // Endpoint envelope zero is a stopping boundary, not a zero-radius disk
    // that disables a measured return correction. Use bounded position and
    // measured braking demand, retaining the ordinary vector slew budget.
    const double cap=std::min(config_.max_speed*planar_scale_,std::max(reachable_floor,
      config_.kp_position*error.norm()+config_.velocity_damping_gain*odom_.velocity_in_frame.head<2>().norm()));
    const double yaw=config_.kp_yaw*angle(holonomic_heading_-odom_.yaw)-
      config_.yaw_damping_gain*projectedBodyHeadingRate(odom_.orientation,odom_.angular_velocity_in_frame);
    return holonomicLimited(body,yaw,cap,acceleration,dt,reason,frozen);
  }
  Output holonomicLimited(const Eigen::Vector2d& desired,double yaw,double cap,double acceleration,
      double dt,const std::string& reason,bool frozen,double curvature=0.,
      const Eigen::Vector2d& body_tangent=Eigen::Vector2d::Zero(),bool update_yaw=true) {
    if(braking_reentry_required_){reason_="braking_envelope_reentry_required";return stop();}
    if(config_.align_then_forward) {
      if(dt<=0.) {auto out=last_output_;out.reason=reason;out.frozen=frozen;return out;}
      const auto forward=forward_conditioner_.step(std::max(0.,desired.x()),0.,cap,
        acceleration,config_.max_jerk*planar_scale_,config_.forward_response,
        config_.max_input_rate,config_.max_input_acceleration,dt);
      Output out;out.forward=forward.value;out.lateral=0.;out.reason=reason;out.frozen=frozen;
      command_planar_acceleration_={forward.acceleration,0.};
      control_diagnostic_.forward_input_fraction=forward.mapped_fraction;
      control_diagnostic_.forward_input_rate=forward.mapped_rate;
      control_diagnostic_.input_limit_override|=forward.priority_override;
      out.yaw_rate=update_yaw?limitedYaw(yaw,last_output_.yaw_rate,dt):last_output_.yaw_rate;
      last_output_=out;rememberOutputFrame();reason_=reason;return out;
    }
    // Desired is in the CURRENT measured body frame. Transport the preceding
    // command (and optional derivative history) into that same basis first.
    // Otherwise body yaw alone can consume or evade the XY acceleration
    // budget. This preserves command history, not a predicted physical state.
    // Saved forward-only steering profiles intentionally constrain BODY vx
    // with vy=0; retain that legacy component contract there.
    const Eigen::Vector2d previous=planarHistoryInCurrentBody({last_output_.forward,last_output_.lateral});
    command_planar_acceleration_=planarHistoryInCurrentBody(command_planar_acceleration_);
    auto velocity=normalPlanarStep(desired,previous,cap,acceleration,dt);
    if(!velocity) {
      braking_reentry_required_=true;reason_="braking_envelope_reentry_required";return stop();
    }
    if(config_.max_jerk>0.&&dt>0.) {
      Eigen::Vector2d target=desired;
      if(target.norm()>cap)target*=cap/target.norm();
      const auto shaped=smoothVelocityTarget<2>(target,previous,command_planar_acceleration_,
        acceleration,config_.max_jerk*planar_scale_,dt);
      if(shaped.allFinite()&&shaped.norm()<=cap+1e-12&&
         (shaped-previous).norm()<=acceleration*dt+1e-12&&
         (!config_.align_then_forward||(shaped.x()>=0.&&std::abs(shaped.y())<=1e-12)))velocity=shaped;
      else command_planar_acceleration_=(*velocity-previous)/dt;
    }
    if(config_.forward_response.enabled()&&dt>0.) {
      const auto response=forward_response_limiter_.step(velocity->x(),previous.x(),
        std::max(0.,previous.x()-acceleration*dt),std::min(cap,previous.x()+acceleration*dt),
        config_.forward_response,config_.max_input_rate,config_.max_input_acceleration,dt);
      velocity->x()=response.value;
      command_planar_acceleration_=(*velocity-previous)/dt;
      control_diagnostic_.forward_input_fraction=response.mapped_fraction;
      control_diagnostic_.forward_input_rate=response.mapped_rate;
      control_diagnostic_.input_limit_override|=response.priority_override;
    }
    Output out;out.forward=velocity->x();out.lateral=velocity->y();out.reason=reason;out.frozen=frozen;
    // Feed-forward follows the ACTUALLY slew-limited along-path speed, not
    // an unachievable future native speed during startup or endpoint braking.
    yaw+=curvature*std::max(0.,velocity->dot(body_tangent));
    if(update_yaw&&control_diagnostic_.valid)
      control_diagnostic_.yaw_target=std::clamp(yaw,-config_.max_yaw_rate,config_.max_yaw_rate);
    out.yaw_rate=update_yaw?limitedYaw(yaw,last_output_.yaw_rate,dt):last_output_.yaw_rate;
    last_output_=out;rememberOutputFrame();reason_=reason;return out;
  }
  double limitedYaw(double target,double previous,double dt) {
    if(config_.align_then_forward) {
      if(dt<=0.)return previous;
      const auto result=yaw_conditioner_.step(target,-config_.max_yaw_rate,config_.max_yaw_rate,
        config_.max_yaw_acceleration,config_.max_yaw_jerk,config_.yaw_response,
        config_.max_input_rate,config_.max_input_acceleration,dt);
      command_yaw_acceleration_=result.acceleration;
      control_diagnostic_.yaw_input_fraction=result.mapped_fraction;
      control_diagnostic_.yaw_input_rate=result.mapped_rate;
      control_diagnostic_.input_limit_override|=result.priority_override;
      return result.value;
    }
    target=std::clamp(target,-config_.max_yaw_rate,config_.max_yaw_rate);
    const double normal=std::clamp(target,previous-config_.max_yaw_acceleration*dt,
      previous+config_.max_yaw_acceleration*dt);
    if(dt<=0.)return normal;
    double result=target==0.&&std::abs(normal)<=1e-12?0.:normal;
    if(config_.max_yaw_jerk>0.) {
    Eigen::Matrix<double,1,1> t,p,a;t<<target;p<<previous;a<<command_yaw_acceleration_;
    const double shaped=smoothVelocityTarget<1>(t,p,a,config_.max_yaw_acceleration,
      config_.max_yaw_jerk,dt).x();
    if(std::abs(shaped)<=config_.max_yaw_rate+1e-12&&
       std::abs(shaped-previous)<=config_.max_yaw_acceleration*dt+1e-12) {
      command_yaw_acceleration_=a.x();result=shaped;
    } else {
      command_yaw_acceleration_=(normal-previous)/dt;
    }
    }
    if(config_.yaw_response.enabled()) {
      const auto response=yaw_response_limiter_.step(result,previous,
        std::max(-config_.max_yaw_rate,previous-config_.max_yaw_acceleration*dt),
        std::min(config_.max_yaw_rate,previous+config_.max_yaw_acceleration*dt),
        config_.yaw_response,config_.max_input_rate,config_.max_input_acceleration,dt);
      result=response.value;command_yaw_acceleration_=(result-previous)/dt;
      control_diagnostic_.yaw_input_fraction=response.mapped_fraction;
      control_diagnostic_.yaw_input_rate=response.mapped_rate;
      control_diagnostic_.input_limit_override|=response.priority_override;
    }
    return result;
  }
  Eigen::Vector2d planarHistoryInCurrentBody(const Eigen::Vector2d& value)const {
    if(!output_frame_yaw_valid_||config_.align_then_forward)return value;
    const double delta=output_frame_yaw_-odom_.yaw,c=std::cos(delta),s=std::sin(delta);
    return {c*value.x()-s*value.y(),s*value.x()+c*value.y()};
  }
  void rememberOutputFrame() {output_frame_yaw_=odom_.yaw;output_frame_yaw_valid_=have_odom_;}
  void resetCommandAcceleration() {
    command_planar_acceleration_.setZero();command_yaw_acceleration_=0.;output_frame_yaw_valid_=false;
    forward_response_limiter_.reset();yaw_response_limiter_.reset();
    forward_conditioner_.reset();yaw_conditioner_.reset();
  }
  Eigen::Vector3d positionAt(double time)const {
    return prepared_geometry_->entry.curve_sampler->evaluateDeBoorT(time);
  }
  Eigen::Vector3d velocityAt(double time)const {
    return prepared_geometry_->entry.velocity_sampler->evaluateDeBoorT(time);
  }
  double timeAtArc(double arc)const {
    const auto hi=std::upper_bound(arcTable().begin(),arcTable().end(),arc);
    if(hi==arcTable().begin())return timeTable().front();
    if(hi==arcTable().end())return timeTable().back();
    const auto i=static_cast<std::size_t>(hi-arcTable().begin());
    const double span=arcTable()[i]-arcTable()[i-1];
    return timeTable()[i-1]+std::clamp((arc-arcTable()[i-1])/span,0.,1.)*
      (timeTable()[i]-timeTable()[i-1]);
  }
  double headingCurvature(const Eigen::Vector3d& velocity)const {
    if(config_.align_then_forward&&prepared_geometry_->body_heading_contract.forward_entry_profile)
      return prepared_geometry_->bodyHeadingCurvatureAt(execution_time_);
    const scan_planner::SplineHeadingContract heading;
    const double speed=velocity.head<2>().norm();
    if(speed<heading.minimum_direction_speed)return 0.;
    const auto acceleration=prepared_geometry_->acceleration_sampler->evaluateDeBoorT(execution_time_);
    // Native certifies the current tangent. Its instantaneous geometric turn
    // rate needs no measured-inertia prediction or future-chord low-pass.
    return (velocity.x()*acceleration.y()-velocity.y()*acceleration.x())/(speed*speed*speed);
  }
  bool candidateReject(const char* reason) { candidate_reason_=reason;join_diagnostic_.reason=reason;return false; }
  bool supportValid(const ControlIdentity& identity,std::uint64_t generation) const {
    return support_&&support_index_&&supportEvidenceValid(*support_,*support_index_,identity,generation,config_);
  }
  bool supported(const Eigen::Vector3d& body) const {
    return support_&&support_index_&&pointSupported(*support_,*support_index_,body,config_.goal_height_tolerance);
  }
  bool supportContractAllowsBody() const {
    if(!config_.require_support_reference)return true;
    if(!supportValid(task_.identity,task_.generation))return false;
    // Formal execution already binds the exact current Native SCAN proof.
    // Do not turn noisy historical PCD returns into another per-tick veto.
    return config_.geometry_authority==GeometryAuthority::NativeScan||supported(odom_.position);
  }
  bool sourceEvidenceFresh(const Odom& odom, double now) const {
    return sourceEvidenceFailure(odom,now,config_)==nullptr;
  }
  bool projectMeasured(double forward_window) {
    const double from=std::max(0., committed_arc_-config_.projection_backtrack_m);
    const double to=std::min(arcTable().back(), measured_arc_+forward_window);
    double best_arc=measured_arc_, best_time=execution_time_;
    bool outside_window=false;
    double best_dist=(positionAt(execution_time_)-odom_.position).squaredNorm();
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
    if(holonomic())planar_scale_=prepared_geometry_->scaleAt(execution_time_);
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
    resetCommandAcceleration();
  }
  Output stop() {
    resetTurnStationaryWindow();
    resetCommandAcceleration();
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
  bool terminal_settling_{false};
  ControlDiagnostic control_diagnostic_;
  unsigned recovery_samples_{0};
  double recovery_first_source_{0.};
  std::string candidate_reason_;
  JoinDiagnostic join_diagnostic_;
  bool floor_anchor_valid_{false};
  bool holonomic_heading_valid_{false};
  RearTurnIntent path_turn_intent_,goal_turn_intent_;
  StairSteeringHistory stair_steering_history_;
  ControlIdentity stair_steering_identity_;
  std::string stair_steering_model_hash_;
  bool stair_steering_scope_valid_{false};
  double holonomic_heading_{0.},goal_stationary_first_{0.},goal_stationary_source_{0.};
  unsigned goal_stationary_samples_{0};
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
  Eigen::Vector2d command_planar_acceleration_{Eigen::Vector2d::Zero()};
  double output_frame_yaw_{0.};
  bool output_frame_yaw_valid_{false};
  double command_yaw_acceleration_{0.};
  ResponseCommandLimiter forward_response_limiter_,yaw_response_limiter_;
  MotionCommandConditioner forward_conditioner_,yaw_conditioner_;
  SteeringPreviewSample last_steering_sample_;
};
}  // namespace d1max_trajectory_tracker
