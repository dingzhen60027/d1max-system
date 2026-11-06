#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace d1max_trajectory_tracker {
// A decoded, finite planning prior. This is steering context only: it cannot
// establish support, clear occupancy, authorize motion, or replace the spline.
struct StairSteeringSurface {
  std::string id,segment,kind;
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  std::vector<Eigen::Vector2d> polygon;
  double grade{0.},intercept{0.},height_association{0.};
  bool operator==(const StairSteeringSurface& o)const {
    if(id!=o.id||segment!=o.segment||kind!=o.kind||grade!=o.grade||intercept!=o.intercept||
       height_association!=o.height_association||polygon.size()!=o.polygon.size()||
       !(origin.array()==o.origin.array()).all()||!(rotation.array()==o.rotation.array()).all())return false;
    for(std::size_t i=0;i<polygon.size();++i)
      if(!(polygon[i].array()==o.polygon[i].array()).all())return false;
    return true;
  }
};

// Same finite XYZ association and landing tie break as Native's model query.
// Only the selected index is returned; no per-control-tick heap allocation.
inline std::optional<std::size_t> stairSteeringSurfaceAt(
    const std::vector<StairSteeringSurface>& surfaces,const Eigen::Vector3d& body,double height) {
  if(!body.allFinite()||!std::isfinite(height)||height<0.||height>1.)return {};
  std::optional<std::size_t> result;double distance=INFINITY,result_z=0.;bool ambiguous=false;
  for(std::size_t i=0;i<surfaces.size();++i) {
    const auto& s=surfaces[i];if(s.polygon.size()<3)continue;
    const Eigen::Vector3d offset=s.rotation.col(2)*height;
    Eigen::Vector3d ground=body-offset;
    const Eigen::Vector3d n=s.rotation*Eigen::Vector3d(-s.grade,0.,1.);
    if(n.z()<=1e-6||std::abs(n.dot(ground-s.origin)-s.intercept)>s.height_association)continue;
    ground.z()=(s.intercept+n.dot(s.origin)-n.x()*ground.x()-n.y()*ground.y())/n.z();
    const Eigen::Vector2d p=(s.rotation.transpose()*(ground-s.origin)).head<2>();
    bool contains=true;
    for(std::size_t k=0;k<s.polygon.size();++k) {
      const Eigen::Vector2d edge=s.polygon[(k+1)%s.polygon.size()]-s.polygon[k],delta=p-s.polygon[k];
      if(edge.x()*delta.y()-edge.y()*delta.x()<-1e-10){contains=false;break;}
    }
    if(!contains)continue;
    const double z=ground.z()+offset.z(),d=std::abs(z-body.z());
    if(!result||d<distance-1e-10){result=i;distance=d;result_z=z;ambiguous=false;}
    else if(std::abs(d-distance)<=1e-10) {
      if(std::abs(result_z-z)>1e-8)ambiguous=true;
      else if(s.kind=="landing"){result=i;result_z=z;}
    }
  }
  return ambiguous?std::nullopt:result;
}

struct StairFlightRange {
  std::string segment;
  double begin{0.},end{0.}; // Horizontal distance on THIS accepted curve.
  bool ends_at_curve_tail{false};
  double tail_exit_distance{INFINITY}; // Semantic boundary only, never a target.
  Eigen::Vector3d exit_point{Eigen::Vector3d::Zero()};
  Eigen::Vector2d direction{Eigen::Vector2d::Zero()};
  bool has_exit_point{false};
};
inline double stairSurfaceExitDistance(const StairSteeringSurface& s,
    const Eigen::Vector3d& body,double height,const Eigen::Vector3d& tangent) {
  if(tangent.head<2>().norm()<1e-9)return INFINITY;
  const Eigen::Vector3d n=s.rotation*Eigen::Vector3d(-s.grade,0.,1.);
  if(n.z()<=1e-6)return INFINITY;
  Eigen::Vector3d ground=body-s.rotation.col(2)*height;
  ground.z()=(s.intercept+n.dot(s.origin)-n.x()*ground.x()-n.y()*ground.y())/n.z();
  Eigen::Vector3d direction=tangent/tangent.head<2>().norm();
  direction.z()=-(n.x()*direction.x()+n.y()*direction.y())/n.z();
  const Eigen::Vector2d p=(s.rotation.transpose()*(ground-s.origin)).head<2>();
  const Eigen::Vector2d v=(s.rotation.transpose()*direction).head<2>();
  double exit=INFINITY;
  for(std::size_t k=0;k<s.polygon.size();++k) {
    const Eigen::Vector2d edge=s.polygon[(k+1)%s.polygon.size()]-s.polygon[k],delta=p-s.polygon[k];
    const double derivative=edge.x()*v.y()-edge.y()*v.x();
    if(derivative<-1e-12)
      exit=std::min(exit,std::max(0.,(edge.x()*delta.y()-edge.y()*delta.x())/-derivative));
  }
  return exit;
}
inline std::vector<StairFlightRange> stairCurveFlightRanges(
    const std::vector<StairSteeringSurface>& surfaces,const std::vector<Eigen::Vector3d>& points,
    const std::vector<double>& horizontal,double height,
    const std::function<bool()>& allowed=[] {return true;}) {
  std::vector<StairFlightRange> ranges;
  if(points.size()!=horizontal.size())return ranges;
  bool previous_flight=false;
  for(std::size_t i=0;i<points.size();++i) {
    if(!allowed())return {};
    const auto sample=stairSteeringSurfaceAt(surfaces,points[i],height);
    const bool flight=sample&&surfaces[*sample].kind=="continuous_support_envelope";
    if(flight) {
      const auto& segment=surfaces[*sample].segment;
      if(!previous_flight||ranges.back().segment!=segment)
        ranges.push_back({segment,horizontal[i],horizontal[i],false});
      else ranges.back().end=horizontal[i];
      if(ranges.back().direction.squaredNorm()<1e-12&&points.size()>1) {
        const Eigen::Vector2d chord=(i+1<points.size()?points[i+1]-points[i]:points[i]-points[i-1]).head<2>();
        if(chord.norm()>1e-9)ranges.back().direction=chord.normalized();
      }
      ranges.back().exit_point=points[i];ranges.back().has_exit_point=true;
      ranges.back().ends_at_curve_tail=i+1==points.size();
      if(ranges.back().ends_at_curve_tail&&i)
        ranges.back().tail_exit_distance=stairSurfaceExitDistance(surfaces[*sample],points[i],height,points[i]-points[i-1]);
    }
    previous_flight=flight;
  }
  return ranges;
}

struct StairSteeringPreview {double weight{0.},horizontal{0.},lateral_distance{0.};};
// A physical exit point, not an old trajectory target or an elapsed-time
// latch. The owner scopes it to task/map/model/anchor identity. A sliding
// prefix may remove the previous flight but cannot erase its already seen
// exit. Preserve that point only for the same next flight and travel direction;
// all preview positions still come from the currently accepted finite spline.
class StairSteeringHistory {
public:
  void reset(){*this=StairSteeringHistory{};}
  double precedingExitDistance(const std::vector<StairFlightRange>& ranges,
      double horizontal,double body_length,const Eigen::Vector3d& body) {
    if(!body.allFinite())return INFINITY;
    for(const auto& r:ranges) {
      if(r.end<horizontal||r.end-r.begin<1e-6) {
        if(r.end<=horizontal)rememberExit(r);
        continue;
      }
      if(r.begin>horizontal+body_length)return INFINITY;
      // A window may stop at the physical exit before its following landing
      // or next flight enters the window. Remember that finite boundary, but
      // never interpret an ordinary interior sliding tail as a passed exit.
      if(r.ends_at_curve_tail&&r.tail_exit_distance<=1e-6&&horizontal>=r.end-1e-6) {
        rememberExit(r);return INFINITY;
      }
      if(valid_&&next_segment_.empty()&&r.segment!=previous_segment_&&r.direction.squaredNorm()>.5) {
        next_segment_=r.segment;direction_=r.direction;
      }
      if(valid_&&(next_segment_!=r.segment||direction_.dot(r.direction)<=0.))reset();
      return valid_?(body.head<2>()-exit_.head<2>()).norm():INFINITY;
    }
    return INFINITY;
  }
private:
  void rememberExit(const StairFlightRange& r) {
    if(!r.has_exit_point||!r.exit_point.allFinite()||
        (r.ends_at_curve_tail&&(!std::isfinite(r.tail_exit_distance)||r.tail_exit_distance>1e-6))||
        (valid_&&previous_segment_==r.segment))return;
    // No following flight is required yet. Otherwise a finite window that
    // ends on the platform loses this context before the next flight appears.
    previous_segment_=r.segment;next_segment_.clear();exit_=r.exit_point;
    direction_.setZero();valid_=true;
  }
  bool valid_{false};
  std::string previous_segment_,next_segment_;
  Eigen::Vector3d exit_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d direction_{Eigen::Vector2d::Zero()};
};
inline double stairBlend(double x) {
  x=std::clamp(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x));
}
inline StairSteeringPreview stairSteeringPreview(const std::vector<StairFlightRange>& ranges,
    double horizontal,double ordinary_preview,double body_length,
    double preceding_exit_distance=INFINITY) {
  StairSteeringPreview out{0.,horizontal+ordinary_preview,ordinary_preview};
  if(!std::isfinite(horizontal)||!std::isfinite(ordinary_preview)||ordinary_preview<=0.||
      !std::isfinite(body_length)||body_length<=0.)return out;
  // One body length gives an entrance runway, while a 1.25-body-length chord
  // avoids repeatedly correcting a large body toward a 40 cm local tangent.
  const double preview=std::max(ordinary_preview,1.25*body_length);
  double preceding_exit=-INFINITY;
  for(const auto& r:ranges) {
    if(r.end<horizontal||r.end-r.begin<1e-6){preceding_exit=r.end;continue;}
    if(r.begin>horizontal+body_length)break;
    const double entry=stairBlend(1.-std::max(0.,r.begin-horizontal)/body_length);
    // A finite window ending exactly on a physical flight exit must not be
    // mistaken for an interior sliding tail. The polygon/terminal tangent
    // estimate affects this blend only; the preview stays on the old curve.
    const double remaining=r.end-horizontal+(r.ends_at_curve_tail?r.tail_exit_distance:0.);
    const double exit=stairBlend(remaining/body_length);
    // A short turning landing must not jump directly from the preceding
    // flight's faded-out law to the next flight's opposite heading.
    const double since_exit=std::isfinite(preceding_exit_distance)?preceding_exit_distance:
      horizontal-preceding_exit;
    out.weight=entry*exit*stairBlend(since_exit/body_length);
    out.horizontal=std::min(horizontal+preview,r.end);
    out.lateral_distance=preview;
    return out; // Never look through a landing into the next flight.
  }
  return out;
}
} // namespace d1max_trajectory_tracker
