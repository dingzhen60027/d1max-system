#pragma once

#include <Eigen/Geometry>
#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace d1max_trajectory_tracker {

struct CurveProjection {
  double time{0.}, arc{0.};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

// Bounded same-curve XYZ projection. Source time only limits how far the
// measured body could have moved; elapsed time never chooses the curve time.
template<class CurveEvaluator>
inline std::optional<CurveProjection> projectCurveAdmission(
    const CurveEvaluator& curve,
    const std::vector<double>& times, const std::vector<double>& arcs,
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& body,
    double seed_time, double seed_arc, double backward, double forward, double join_limit=.0125) {
  if (!body.allFinite() || !std::isfinite(seed_time) || !std::isfinite(seed_arc) ||
      !std::isfinite(backward) || !std::isfinite(forward) || backward<0. || forward<0. ||
      !std::isfinite(join_limit) || join_limit<=0. || join_limit>.0125 ||
      times.size()<2 || times.size()!=arcs.size() || times.size()!=points.size()) return {};
  const double from=std::max(0.,seed_arc-backward), to=std::min(arcs.back(),seed_arc+forward);
  CurveProjection result{seed_time,seed_arc,curve.evaluateDeBoorT(seed_time)};
  double nearest=(result.position-body).squaredNorm();
  const auto lower=std::lower_bound(arcs.begin(),arcs.end(),from);
  const std::size_t begin=lower==arcs.begin()?0:static_cast<std::size_t>(lower-arcs.begin()-1);
  for(std::size_t i=begin;i+1<arcs.size() && arcs[i]<=to;++i) {
    const double span=arcs[i+1]-arcs[i];
    const Eigen::Vector3d chord=points[i+1]-points[i];
    if(span<1e-12 || chord.squaredNorm()<1e-12) continue;
    const double lo=std::clamp((from-arcs[i])/span,0.,1.);
    const double hi=std::clamp((to-arcs[i])/span,0.,1.);
    const double fraction=std::clamp((body-points[i]).dot(chord)/chord.squaredNorm(),lo,hi);
    const double t=times[i]+fraction*(times[i+1]-times[i]);
    const Eigen::Vector3d p=curve.evaluateDeBoorT(t);
    const double square=(p-body).squaredNorm();
    if(square+1e-12<nearest) {
      nearest=square; result={t,arcs[i]+fraction*span,p};
    }
  }
  if(!result.position.allFinite() || nearest>join_limit*join_limit) return {};
  return result;
}

inline double curveArcAt(const std::vector<double>& times, const std::vector<double>& arcs, double time) {
  const auto upper=std::upper_bound(times.begin(),times.end(),time);
  if(upper==times.begin()) return arcs.front();
  if(upper==times.end()) return arcs.back();
  const auto i=static_cast<std::size_t>(upper-times.begin()-1);
  const double ratio=(time-times[i])/(times[i+1]-times[i]);
  return arcs[i]+ratio*(arcs[i+1]-arcs[i]);
}

inline bool curveDerivativeBound(scan_planner::UniformBspline& derivative,double maximum) {
  const auto controls=derivative.getControlPoint();
  if(!controls.allFinite() || controls.cols()<1) return false;
  for(int i=0;i<controls.cols();++i) if(controls.col(i).norm()>maximum+1e-6) return false;
  return true;
}
} // namespace d1max_trajectory_tracker
