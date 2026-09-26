#pragma once

#include <bspline_opt/reference_path.hpp>
#include <cmath>
#include <functional>
#include <string>

namespace scan_planner {

using OccupancyQuery = std::function<int(const Eigen::Vector3d &, double)>;
using DetourSearch = std::function<std::vector<Eigen::Vector3d>(
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
    const auto anchor=out.back();
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
    auto detour=search(anchor, samples[exit]);
    if (detour.size()<2 || detour.size()>20000) { reason="failed_reference_search"; return false; }
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

}  // namespace scan_planner
