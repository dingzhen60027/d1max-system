#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <bspline_opt/spline_heading_contract.hpp>

namespace scan_planner {
using CurveDirectionPoint=std::array<double,3>;
struct NearCurveDirection {
  double time=0.;
  CurveDirectionPoint velocity{};
  // A direction of immutable geometry, NOT a fabricated velocity sample.
  // Usually the confident local derivative; only mm/s geometry uses the
  // bounded near-curve secant below.
  CurveDirectionPoint direction{};
};
struct CurveDirectionAlwaysAllowed {bool operator()()const{return true;}};

// Actual immutable curve geometry, not a future wall-time prediction or a
// chord to a distant target. Both Native collision and control use this SAME
// near-curve observation when the current derivative is not directional.
// Keep the first 1 mm derivative when it clears the SAME .02 m/s confidence
// contract as the rest of Native heading checks. A tiny measured entry jitter
// must not choose an arbitrary 180 degree turn: search no farther than 2 cm
// for a confident derivative, otherwise use that short geometry's secant.
// This never uses an endpoint/global-goal chord or alters the curve/velocity.
// Evaluation callbacks consume curve-parameter time and return XYZ arrays.
template<class Position,class Velocity,class Allowed=CurveDirectionAlwaysAllowed>
std::optional<NearCurveDirection> nearCurveDirection(Position position,Velocity velocity,
    double start_time,double duration,Allowed allowed={},std::size_t maximum_steps=4096) {
  if(!std::isfinite(start_time)||!std::isfinite(duration)||start_time<0.||
     duration<=start_time||duration>120.||!maximum_steps||maximum_steps>4096)return {};
  const auto finite=[](const CurveDirectionPoint&p){
    return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]);};
  const auto distance=[](const CurveDirectionPoint&a,const CurveDirectionPoint&b){
    return std::hypot(std::hypot(a[0]-b[0],a[1]-b[1]),a[2]-b[2]);};
  if(!allowed())return {};
  auto previous=position(start_time);if(!finite(previous))return {};
  const auto origin=previous;
  constexpr double first_arc=.001,near_arc=.02;
  const double minimum_speed=SplineHeadingContract{}.minimum_direction_speed;
  double time=start_time,arc=0.;
  for(std::size_t step=0;step<maximum_steps&&time<duration;++step) {
    if(!allowed())return {};
    const double next_time=std::min(duration,time+.001);
    if(next_time<=time)return {};
    const auto next=position(next_time);if(!finite(next))return {};
    const double segment=distance(previous,next);
    if(!std::isfinite(segment))return {};
    const bool first_crossing=arc<first_arc&&arc+segment>=first_arc;
    const bool near_crossing=arc+segment>=near_arc;
    if(first_crossing||near_crossing||arc>=first_arc) {
      const double crossing=first_crossing?first_arc:near_crossing?near_arc:arc+segment;
      const double remaining=crossing-arc;double low=time,high=next_time;
      // Refine only the crossing millisecond. Both callers retain the exact
      // same De Boor curve; this does not move, retime or re-admit it.
      for(unsigned i=0;i<24&&(first_crossing||near_crossing);++i) {
        if(!allowed())return {};
        const double mid=(low+high)*.5;const auto p=position(mid);
        if(!finite(p))return {};
        if(distance(previous,p)<remaining)low=mid;else high=mid;
      }
      if(!allowed())return {};
      const auto tangent=velocity(high);
      if(!finite(tangent))return {};
      if(std::hypot(tangent[0],tangent[1])>=minimum_speed)
        return NearCurveDirection{high,tangent,tangent};
      if(near_crossing) {
        const auto p=position(high);if(!finite(p))return {};
        const CurveDirectionPoint chord{p[0]-origin[0],p[1]-origin[1],p[2]-origin[2]};
        // A folded/vertical/undefined short curve supplies no orientation.
        // Native keeps measured heading; the tracker waits for real geometry.
        if(std::hypot(chord[0],chord[1])<near_arc*.5)return {};
        return NearCurveDirection{high,tangent,chord};
      }
    }
    arc+=segment;previous=next;time=next_time;
  }
  // A short legitimate slow curve may not accumulate 2 cm within the bounded
  // search. Its coherent first >=1 mm secant is still actual geometry (not
  // its noisy derivative or a distant target). Deadline interruption above
  // NEVER takes this fallback. Stationary, folded and vertical remain absent.
  const CurveDirectionPoint chord{previous[0]-origin[0],previous[1]-origin[1],previous[2]-origin[2]};
  if(arc>=first_arc&&std::hypot(chord[0],chord[1])>=std::max(first_arc,arc*.5)) {
    if(!allowed())return {};
    const auto tangent=velocity(time);if(!finite(tangent))return {};
    return NearCurveDirection{time,tangent,chord};
  }
  // Constant/undefined/too-short geometry supplies no new turn direction.
  return {};
}
} // namespace scan_planner
