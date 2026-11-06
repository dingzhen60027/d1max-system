#pragma once

#include <bspline_opt/reference_path.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace scan_planner {

using OccupancyQuery = std::function<int(const Eigen::Vector3d &, double)>;
// Native A* checks a rectangular body at its start-to-end heading. An anchor
// that is free along the route tangent can therefore still be rejected by A*.
// Preserve that distinction so the caller can choose another reference anchor;
// generic search failure/deadline must not trigger a fresh search budget.
enum class DetourSearchStatus { Failure, Success, StartEndpointOccupied, TargetEndpointOccupied };

// A native SUCCESS can legitimately contain one lattice point: exact start
// and end may share a cell. Point count must not replace the search status.
// Such a result is only a seed; both exact endpoint connectors are still
// checked using the caller's collision policy below.
struct DetourSearchResult {
  DetourSearchStatus status{DetourSearchStatus::Failure};
  std::vector<Eigen::Vector3d> points;
};
using DetourSearch = std::function<DetourSearchResult(
    const Eigen::Vector3d &, const Eigen::Vector3d &)>;
using LegacyDetourSearch = std::function<std::vector<Eigen::Vector3d>(
    const Eigen::Vector3d &, const Eigen::Vector3d &)>;

// Nonzero occupancy queries (including out-of-map=-1) are blocked. Native
// unobserved in-map cells retain GridMap's existing policy; this helper does
// not prove ground support or turn unobserved space into measured free space.
// Check connectors too: free endpoints do not justify cutting a corner.
inline bool collisionFreeSegment(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                                 double spacing, const OccupancyQuery &occupied) {
  if (!a.allFinite() || !b.allFinite() || !std::isfinite(spacing) || spacing <= 0.) return false;
  const double length = (b-a).norm();
  if (length/spacing > 20000.) return false;
  const int steps = std::max(1, static_cast<int>(std::ceil(length/spacing)));
  const double yaw = std::atan2(b.y()-a.y(), b.x()-a.x());
  for (int i=0; i<=steps; ++i)
    if (occupied(a + (b-a)*(static_cast<double>(i)/steps), yaw) != 0) return false;
  return true;
}

// A route projection measures progress; it is not an intermediate motion goal.
// Join the eligible level/straight prefix ahead with a smooth geometric seed,
// rather than first travelling sideways to its nearest point. The seed starts
// in the measured forward direction and joins the route tangent. Its zero
// endpoint geometric second derivatives do NOT replace measured acceleration:
// fitCubicWithFixedBoundary still fixes the real P/V/A after this initializer.
// No offset threshold changes the path shape at one voxel. Bends/stairs bound
// the join, their original vertices/Z remain, and an occupied connector falls
// back to the original guidance and existing native A* (not a failed task).
inline std::vector<Eigen::Vector3d> measuredForwardApproach(
    const std::vector<Eigen::Vector3d>& reference,double resolution,
    const OccupancyQuery& occupied,const Eigen::Vector3d& start_velocity,
    double body_yaw,bool has_start_projection_connector=false) {
  // slice() removes a duplicate projection when the acquired body is already
  // on the route. Only its producer knows whether point 1 is a projection or
  // the first REAL corner. Never infer that from front()==actual_start.
  const std::size_t route_begin=has_start_projection_connector?1:0;
  if(reference.size()<route_begin+2||!std::isfinite(resolution)||resolution<=0.||
     !start_velocity.allFinite()||!std::isfinite(body_yaw))return reference;
  for(const auto& p:reference)if(!p.allFinite())return reference;
  const Eigen::Vector3d direction=reference[route_begin+1]-reference[route_begin];
  const double length=direction.head<2>().norm();
  if(!direction.allFinite()||!std::isfinite(length)||length<1e-6)return reference;
  // A millimetric source-height vertex on a short level edge must not turn
  // an off-route forward approach into a compulsory sideways projection.
  // The existing half-voxel HEIGHT band stays unchanged. Only this bounded
  // height-profile case relaxes the per-edge slope heuristic; literal Z
  // vertices are retained below, not flattened or treated as measured free.
  const bool bounded_height_profile=has_start_projection_connector&&
      std::abs(direction.z())<=resolution*.5;
  bool literal_height_profile=std::abs(direction.z())>length*.03;
  if(literal_height_profile&&!bounded_height_profile)return reference;
  const Eigen::Vector2d tangent=direction.head<2>()/length;
  const Eigen::Vector2d offset=(reference.front()-reference[route_begin]).head<2>();
  const double lateral=std::abs(offset.x()*tangent.y()-offset.y()*tangent.x());
  const double advance=std::clamp(.8+2.*lateral,.8,1.5);
  double walked=0.;Eigen::Vector3d join=reference[route_begin],join_direction=direction/length;
  std::vector<double> height_arc{0.},height_z{reference[route_begin].z()};
  std::size_t next=route_begin+1;
  for(std::size_t i=route_begin+1;i<reference.size();++i) {
    const Eigen::Vector3d delta=reference[i]-reference[i-1];
    const double planar=delta.head<2>().norm();
    const bool steep_edge=std::abs(delta.z())>planar*.03;
    if(planar<1e-6||(steep_edge&&!bounded_height_profile)||
       delta.head<2>().dot(tangent)<planar*.995||
       std::abs(reference[i].z()-reference[route_begin].z())>resolution*.5)break;
    const double take=std::min(planar,advance-walked);
    join=reference[i-1]+delta*(take/planar);join_direction=delta/planar;
    walked+=take;next=i;
    literal_height_profile=literal_height_profile||steep_edge;
    height_arc.push_back(walked);height_z.push_back(join.z());
    if(take>=planar-1e-9)next=i+1;
    if(walked>=advance-1e-9)break;
  }
  if(walked<.2)return reference;

  // At rest use the body-forward direction; blend continuously toward a
  // measured forward velocity as it becomes informative. A backwards MC
  // transient must not turn the geometric seed's head by pi. The exact
  // negative measured velocity, if any, is still retained by the P/V/A fit.
  const Eigen::Vector2d heading(std::cos(body_yaw),std::sin(body_yaw));
  const Eigen::Vector2d velocity=start_velocity.head<2>();
  const double speed=velocity.norm();
  if(!std::isfinite(speed))return reference;
  Eigen::Vector2d entry=.05*heading;
  if(speed>1e-12)entry+=std::max(0.,velocity.dot(heading))*velocity/speed;
  entry.normalize();
  const Eigen::Vector3d d0(walked*entry.x(),walked*entry.y(),0.);
  const Eigen::Vector3d d1=walked*join_direction;
  // Quintic Bezier: endpoint position/tangent/second derivative are exact.
  const std::array<Eigen::Vector3d,6> controls{
      reference.front(),reference.front()+d0/5.,reference.front()+2.*d0/5.,
      join-2.*d1/5.,join-d1/5.,join};
  double derivative_bound=0.;
  for(std::size_t i=1;i<controls.size();++i) {
    const Eigen::Vector3d delta=controls[i]-controls[i-1];
    // The profile branch replaces Bezier Z with literal piecewise heights.
    // A short source-height edge at the join must not inflate the unused
    // Bezier Z derivative and exhaust an otherwise simple planar seed.
    derivative_bound=std::max(derivative_bound,5.*(literal_height_profile?
        delta.head<2>().norm():delta.norm()));
  }
  // A Bezier derivative is in the convex hull of its derivative controls.
  // Therefore each sampled connector is at most half a voxel long. Work is
  // bounded even for a malformed or very distant actual-start/reference pair.
  const double sample_count=std::ceil(derivative_bound/(resolution*.5));
  if(!std::isfinite(sample_count)||sample_count>2048.)return reference;
  const int count=std::max(2,static_cast<int>(sample_count));
  if(literal_height_profile) {
    // Include every source Z vertex explicitly, even a very short spike that
    // uniform sampling could miss. u remains geometric seed parameter only;
    // real start P/V/A is fixed later by fitCubicWithFixedBoundary.
    std::vector<double> parameters;
    parameters.reserve(count+height_arc.size());
    for(int i=1;i<=count;++i)parameters.push_back(double(i)/count);
    for(std::size_t i=1;i<height_arc.size();++i)parameters.push_back(height_arc[i]/walked);
    std::sort(parameters.begin(),parameters.end());
    parameters.erase(std::unique(parameters.begin(),parameters.end()),parameters.end());
    std::vector<Eigen::Vector3d> shaped{reference.front()};
    for(const double u:parameters) {
      auto values=controls;
      for(std::size_t order=1;order<controls.size();++order)
        for(std::size_t j=0;j<controls.size()-order;++j)
          values[j]=(1.-u)*values[j]+u*values[j+1];
      Eigen::Vector3d point=u==1.?join:values[0];
      const double arc=u*walked;
      const auto upper=std::upper_bound(height_arc.begin(),height_arc.end(),arc);
      const auto k=std::min(height_arc.size()-2,static_cast<std::size_t>(upper-height_arc.begin()-1));
      const double ratio=std::clamp((arc-height_arc[k])/(height_arc[k+1]-height_arc[k]),0.,1.);
      const double offset_blend=1.-u*u*u*(10.+u*(-15.+6.*u));
      point.z()=(1.-ratio)*height_z[k]+ratio*height_z[k+1]+
          offset_blend*(reference.front().z()-reference[route_begin].z());
      if(u==1.)point=join;
      const Eigen::Vector3d a=shaped.back();
      const int pieces=std::max(1,static_cast<int>(std::ceil((point-a).norm()/(resolution*.5))));
      if(shaped.size()+pieces>2049)return reference;
      for(int j=1;j<=pieces;++j) {
        const Eigen::Vector3d p=j==pieces?point:a+(point-a)*(double(j)/pieces);
        if(!collisionFreeSegment(shaped.back(),p,resolution*.5,occupied))return reference;
        shaped.push_back(p);
      }
    }
    shaped.insert(shaped.end(),reference.begin()+next,reference.end());
    return shaped;
  }
  std::vector<Eigen::Vector3d> result;
  result.reserve(static_cast<std::size_t>(count)+1+reference.size()-next);
  result.push_back(reference.front());
  for(int i=1;i<=count;++i) {
    const double u=static_cast<double>(i)/count;
    auto values=controls;
    for(std::size_t order=1;order<controls.size();++order)
      for(std::size_t j=0;j<controls.size()-order;++j)
        values[j]=(1.-u)*values[j]+u*values[j+1];
    const Eigen::Vector3d point=i==count?join:values[0];
    if(!collisionFreeSegment(result.back(),point,resolution*.5,occupied))return reference;
    result.push_back(point);
  }
  result.insert(result.end(),reference.begin()+next,reference.end());
  return result;
}

// Geometry-only callers describe a real path starting at point 0. Production
// sliced references supply the explicit projection-connector flag above.
inline std::vector<Eigen::Vector3d> measuredForwardApproach(
    const std::vector<Eigen::Vector3d>& reference,double resolution,
    const OccupancyQuery& occupied) {
  const Eigen::Vector3d direction=reference.size()>=2?
      Eigen::Vector3d(reference[1]-reference[0]):Eigen::Vector3d::UnitX();
  return measuredForwardApproach(reference,resolution,occupied,Eigen::Vector3d::Zero(),
      std::atan2(direction.y(),direction.x()));
}

// Repair only obstructed subsegments of the selected PCT reference. This is
// a local seed, not a replacement global route or a free-flying 3-D search.
// Native A* supplies XY detours on its original interpolated height plane.
inline bool buildCollisionAwareReference(const std::vector<Eigen::Vector3d> &reference,
    double resolution, const OccupancyQuery &occupied, const DetourSearch &search,
    std::vector<Eigen::Vector3d> &out, std::string &reason, double anchor_margin=.5) {
  out.clear();
  DiscreteReference path;
  try { path.set(reference); } catch (const std::exception &) {
    reason="failed_reference_geometry"; return false;
  }
  if (!std::isfinite(resolution) || resolution<=0. || path.length()/resolution > 10000. ||
      !std::isfinite(anchor_margin) || anchor_margin<0. || anchor_margin>2.) {
    reason="failed_reference_geometry"; return false;
  }
  const double spacing=resolution*.5;
  std::vector<Eigen::Vector3d> samples{path.points.front()};
  // Keep every original corner; uniform resampling must not cut across it.
  for (std::size_t i=1; i<path.points.size(); ++i) {
    const auto a=path.points[i-1], b=path.points[i];
    const int count=std::max(1, static_cast<int>(std::ceil((b-a).norm()/spacing)));
    for (int j=1; j<=count; ++j) samples.push_back(a+(b-a)*(static_cast<double>(j)/count));
  }
  out.push_back(samples.front());
  std::size_t i=1;
  int repairs=0;
  while (i<samples.size()) {
    if (collisionFreeSegment(out.back(), samples[i], spacing, occupied)) {
      out.push_back(samples[i++]); continue;
    }
    // Retain a short free approach/departure for turning; starting exactly on
    // an obstacle boundary can prevent A* from taking even its first side step.
    double retreat=0.;
    while (out.size()>1 && retreat<anchor_margin) {
      retreat+=(out.back()-out[out.size()-2]).norm();
      out.pop_back();
    }
    auto anchor=out.back();
    if (++repairs>16) { reason="failed_reference_search_budget"; return false; }
    std::size_t exit=i;
    for (; exit<samples.size(); ++exit) {
      const auto before=exit+1<samples.size() ? samples[exit] : samples[exit-1];
      const auto after=samples[std::min(exit+1, samples.size()-1)];
      if (collisionFreeSegment(before, after, spacing, occupied)) break;
    }
    if (exit==samples.size()) { reason="failed_reference_target_occupied"; return false; }
    double departure=0.;
    while (exit+1<samples.size() && departure<anchor_margin &&
           collisionFreeSegment(samples[exit], samples[exit+1], spacing, occupied)) {
      departure+=(samples[exit+1]-samples[exit]).norm();
      ++exit;
    }
    auto search_result=search(anchor, samples[exit]);
    // A free route-tangent endpoint is not necessarily free at the native
    // search heading. Do not retry that identical rejected pair forever on
    // every planner cycle. Move only the rejected intermediate anchor, using
    // the same native occupancy and search. Indices move monotonically and are
    // bounded by this reference; the production search shares its one deadline.
    while (search_result.status==DetourSearchStatus::StartEndpointOccupied ||
           search_result.status==DetourSearchStatus::TargetEndpointOccupied) {
      if (search_result.status==DetourSearchStatus::StartEndpointOccupied) {
        if (out.size()<=1) break;  // Never move the actual acquired start.
        out.pop_back();
        anchor=out.back();
      } else {
        bool found=false;
        while (++exit<samples.size()) {
          const auto before=exit+1<samples.size()?samples[exit]:samples[exit-1];
          const auto after=samples[std::min(exit+1,samples.size()-1)];
          if (collisionFreeSegment(before,after,spacing,occupied)) {
            found=true;
            break;
          }
        }
        if (!found) break;  // Never move beyond the selected reference goal.
      }
      search_result=search(anchor,samples[exit]);
    }
    const auto &detour=search_result.points;
    if (search_result.status!=DetourSearchStatus::Success || detour.empty() ||
        detour.size()>20000) { reason="failed_reference_search"; return false; }
    std::vector<Eigen::Vector3d> connected{anchor};
    connected.insert(connected.end(), detour.begin(), detour.end());
    connected.push_back(samples[exit]);
    for (std::size_t j=1; j<connected.size(); ++j) {
      if ((connected[j]-connected[j-1]).norm()<=1e-6) continue;
      if (!collisionFreeSegment(connected[j-1], connected[j], spacing, occupied)) {
        reason="failed_reference_search_collision"; return false;
      }
      if ((connected[j]-out.back()).norm()>1e-6) out.push_back(connected[j]);
    }
    i=exit+1;
  }
  reason=repairs ? "reference_detour_ready" : "reference_clear";
  return true;
}

// Compatibility for geometry-only callers predating explicit native search
// status. Production adapters should supply DetourSearchResult. A nonempty
// legacy result still receives exactly the same endpoint/segment checks.
inline bool buildCollisionAwareReference(const std::vector<Eigen::Vector3d> &reference,
    double resolution, const OccupancyQuery &occupied, const LegacyDetourSearch &search,
    std::vector<Eigen::Vector3d> &out, std::string &reason, double anchor_margin=.5) {
  const DetourSearch typed_search=[&search](const auto &a, const auto &b) {
    auto points=search(a,b);
    const auto status=points.empty() ? DetourSearchStatus::Failure : DetourSearchStatus::Success;
    return DetourSearchResult{status,std::move(points)};
  };
  return buildCollisionAwareReference(reference,resolution,occupied,typed_search,out,reason,anchor_margin);
}

}  // namespace scan_planner
