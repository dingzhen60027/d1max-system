#pragma once

#include <bspline_opt/reference_path.hpp>
#include <iterator>
#include <optional>
#include <set>

namespace scan_planner {

struct ReferenceTerrainRun {
  std::string id,kind,mode;
  double begin{0.},end{0.};
  bool operator==(const ReferenceTerrainRun& b)const {
    return id==b.id&&kind==b.kind&&mode==b.mode;
  }
};
// A scheduling contract for fixed locomotion, not a terrain-free or motion
// certificate. Labels remain literal route labels; a portal may not erase or
// reorder them simply because its actuator mode stays STAIR.
template<typename Reference>
inline std::optional<std::vector<ReferenceTerrainRun>> referenceTerrainRuns(const Reference& r) {
  const auto count=r.path.poses.size();
  if(count<2||r.point_segment_ids.size()!=count||r.point_segment_kinds.size()!=count||
     r.point_required_modes.size()!=count)return {};
  std::vector<ReferenceTerrainRun> result;std::set<std::string> seen;
  double arc=0.;bool header=false;
  for(std::size_t i=0;i<count;++i) {
    if(i) {
      const auto& a=r.path.poses[i-1].pose.position;const auto& b=r.path.poses[i].pose.position;
      const double d=std::hypot(b.x-a.x,b.y-a.y,b.z-a.z);if(!std::isfinite(d))return {};
      arc+=d;
    }
    ReferenceTerrainRun label{r.point_segment_ids[i],r.point_segment_kinds[i],r.point_required_modes[i],arc,arc};
    const bool floor=label.mode=="general"&&(label.kind=="floor"||label.kind=="landing");
    const bool stair=label.mode=="stair"&&(label.kind=="stair_up"||label.kind=="stair_down"||label.kind=="landing");
    if(label.id.empty()||(!floor&&!stair))return {};
    if(label.id==r.segment_id) {
      if(label.kind!=r.segment_kind||label.mode!=r.required_mode)return {};
      header=true;
    }
    if(result.empty()||!(result.back()==label)) {
      if(!seen.insert(label.id).second)return {}; // no return to an earlier semantic branch
      if(!result.empty())result.back().end=arc;
      result.push_back(label);
    }
    result.back().end=arc;
  }
  if(!header)return {};
  return result;
}
template<typename Reference>
inline bool continuousStairReferenceWindow(const Reference& r) {
  const auto runs=referenceTerrainRuns(r);
  return runs&&runs->size()>1&&std::any_of(runs->begin(),runs->end(),
    [](const auto& run){return run.mode=="stair";});
}

// A geometry scheduling distinction, not a collision/permission shortcut.
// The reference owner may slide an unchanged route under one anchor while an
// immutable, already prepared tail is still approaching its measured seam.
template<typename Reference>
inline bool rollingReferenceExtension(const Reference& old,const Reference& next,bool fixed_stair=false) {
  if(old.schema_version!=2||next.schema_version!=2||next.generation<=old.generation||
     old.session_id!=next.session_id||old.task_id!=next.task_id||old.task_id.empty()||
     old.route_id!=next.route_id||old.route_hash!=next.route_hash||
     old.map_version_id!=next.map_version_id||
     old.anchor_id!=next.anchor_id||old.anchor_revision!=next.anchor_revision||
     old.map_geometry_revision!=next.map_geometry_revision||old.context_sequence!=next.context_sequence||
     old.localization_epoch!=next.localization_epoch||old.localization_seed_id!=next.localization_seed_id||
     old.point_reference!="body_center"||next.point_reference!=old.point_reference||
     old.path.header.frame_id!=next.path.header.frame_id||old.path.poses.size()<2||
     next.path.poses.size()<2||old.path.poses.size()>1000||next.path.poses.size()>1000)return false;
  const auto same_segment=[](const Reference& ref) {
    const auto size=ref.path.poses.size();
    if(ref.point_segment_ids.size()!=size||ref.point_segment_kinds.size()!=size||
       ref.point_required_modes.size()!=size)return false;
    for(std::size_t i=0;i<size;++i)
      if(ref.point_segment_ids[i]!=ref.segment_id||ref.point_segment_kinds[i]!=ref.segment_kind||
         ref.point_required_modes[i]!=ref.required_mode)return false;
    return true;
  };
  const bool same_header=old.segment_id==next.segment_id&&old.segment_kind==next.segment_kind&&
      old.required_mode==next.required_mode;
  const bool original_single_segment=same_header&&same_segment(old)&&same_segment(next);
  auto old_runs=referenceTerrainRuns(old);const auto next_runs=referenceTerrainRuns(next);
  const bool has_stair=(old_runs&&std::any_of(old_runs->begin(),old_runs->end(),[](const auto& r){return r.mode=="stair";}))||
      (next_runs&&std::any_of(next_runs->begin(),next_runs->end(),[](const auto& r){return r.mode=="stair";}));
  const bool terrain_continuation=!original_single_segment&&fixed_stair&&old_runs&&next_runs&&has_stair;
  if(!original_single_segment&&!terrain_continuation)return false;
  if(terrain_continuation&&!same_header) {
    const auto current=std::find_if(old_runs->begin(),old_runs->end(),[&](const auto& r){return r.id==old.segment_id;});
    const auto successor=current==old_runs->end()?current:std::next(current);
    // The old admitted window must already name the immediately adjacent
    // terrain. A distant same-task segment is not a sliding-window refresh.
    if(successor==old_runs->end()||successor->id!=next.segment_id||
       successor->kind!=next.segment_kind||successor->mode!=next.required_mode)return false;
  }
  const auto geometry=[](const auto& ref) {
    std::vector<Eigen::Vector3d> points;points.reserve(ref.path.poses.size());
    for(const auto& pose:ref.path.poses) {
      const auto& p=pose.pose.position;points.emplace_back(p.x,p.y,p.z);
    }
    DiscreteReference result;result.set(points);return result;
  };
  try {
    const auto a=geometry(old),b=geometry(next);
    // Classification must remain bounded on malformed-but-finite transport.
    // Longer routes use the ordinary input path, never an unbounded sampler.
    if(!std::isfinite(a.length())||!std::isfinite(b.length())||a.length()>10.||b.length()>10.)return false;
    // Only classifies polyline overlap. This tolerance grants no trajectory
    // join or map free space; Native's original full check remains unchanged.
    // ReferenceCallbacks resamples the same route at window-local spacing;
    // shifted samples can clip different sides of a corner. Half a sample
    // bounds that representation error. Never use the 1.25 cm measured curve
    // join limit here: no curve is adopted by this scheduling classification.
    const auto spacing=[](const DiscreteReference& r) {
      double gap=0.;for(std::size_t i=1;i<r.arc.size();++i)gap=std::max(gap,r.arc[i]-r.arc[i-1]);
      return gap;
    };
    const double tolerance=std::clamp(.5*std::max(spacing(a),spacing(b)),.0125,.075);
    const double begin=a.project(b.points.front(),0.,a.length());
    const double end=b.project(a.points.back(),0.,b.length());
    const bool same_front=(a.points.front()-b.points.front()).norm()<=1e-9;
    const bool same_end=(a.points.back()-b.points.back()).norm()<=1e-9;
    const bool shared_portal=terrain_continuation&&!same_header&&same_front&&
      old.point_segment_ids.front()==old.segment_id&&old.point_segment_ids[1]==next.segment_id&&
      next.point_segment_ids.front()==next.segment_id;
    // A prefetched final tail cannot grow beyond the route endpoint. Likewise
    // its literal portal point can change semantic owner after actual crossing
    // without moving at all. These fixed-STAIR cases retain the same geometry;
    // old flat rolling behavior still requires forward advance and new tail.
    if((begin<=1e-5&&!shared_portal)||a.length()-begin<.1||end<.1||
       (b.length()-end<=.02&&!(terrain_continuation&&same_end))||
       (a.sample(begin)-b.points.front()).norm()>tolerance||
       (b.sample(end)-a.points.back()).norm()>tolerance)return false;
    if(shared_portal) {
      // ReferenceWindow keeps just the shared first vertex under the old
      // measured owner until its transition commits. The following edge is
      // already the successor terrain. Normalize that one tie for comparison
      // only; never rewrite source labels, pose, support or active geometry.
      old_runs->front().end=0.;(*old_runs)[1].begin=0.;
    }
    // The extra tail must advance out of the overlap, not double back into
    // the old corridor after having touched its endpoint. A perpendicular
    // continuation can remain projected at the old endpoint and is allowed.
    double tail_projection=a.length();
    const int tail_samples=std::max(1,static_cast<int>(std::ceil((b.length()-end)/.05)));
    for(int i=1;i<=tail_samples;++i) {
      const auto p=b.sample(end+(b.length()-end)*double(i)/tail_samples);
      const double projected=a.project(p,begin,a.length());
      if(projected+tolerance<tail_projection)return false;
      tail_projection=std::max(tail_projection,projected);
    }
    // Ordered, bidirectional overlap prevents shortcuts at corners, reversed
    // windows, another floor or a nearby branch from becoming a soft update.
    const auto matches=[&](const DiscreteReference& from,double lo,double hi,
                           const DiscreteReference& to,double other_lo,double other_hi,
                           const auto& from_runs,const auto& to_runs) {
      const int n=std::max(1,static_cast<int>(std::ceil((hi-lo)/.05)));
      double progress=other_lo;
      for(int i=0;i<=n;++i) {
        const auto point=from.sample(lo+(hi-lo)*double(i)/n);
        const double s=to.project(point,progress,other_hi);
        if((to.sample(s)-point).norm()>tolerance)return false;
        if(terrain_continuation) {
          const double here=lo+(hi-lo)*double(i)/n;bool label_match=false;
          // Shifted sampling may place a portal at either adjacent sample.
          // Only local half-sample ambiguity is allowed, never a label swap
          // farther inside an otherwise geometrically overlapping corridor.
          for(const auto& f:*from_runs)if(here>=f.begin-tolerance&&here<=f.end+tolerance)
            for(const auto& t:*to_runs)if(s>=t.begin-tolerance&&s<=t.end+tolerance&&f==t)label_match=true;
          if(!label_match)return false;
        }
        progress=s;
      }
      return true;
    };
    return matches(a,begin,a.length(),b,0.,end,old_runs,next_runs)&&
      matches(b,0.,end,a,begin,a.length(),next_runs,old_runs);
  } catch(const std::invalid_argument&) {return false;}
}

// Proactive work scheduling only. This is not the controller's braking law
// and never extends a finite accepted curve. Include the existing future seam,
// full solver budget and one control period BEFORE the finite stopping tail.
inline double continuationPlanningDistance(double actual_speed,double positive_acceleration,
    double acceleration_limit,double jerk_limit=.7,double solve_budget_s=.4,double splice_lead_s=.8,
    double control_period_s=.05) {
  if(!std::isfinite(actual_speed)||actual_speed<0.||!std::isfinite(positive_acceleration)||
     !std::isfinite(acceleration_limit)||acceleration_limit<=0.||!std::isfinite(jerk_limit)||jerk_limit<0.||
     !std::isfinite(solve_budget_s)||solve_budget_s<=0.||
     !std::isfinite(splice_lead_s)||splice_lead_s<=0.||
     !std::isfinite(control_period_s)||control_period_s<=0.)return 0.;
  const double lead=solve_budget_s+splice_lead_s+control_period_s;
  const double a_limit=.5*acceleration_limit,j_limit=.5*jerk_limit;
  double speed=actual_speed,acceleration=std::clamp(positive_acceleration,0.,a_limit);
  double stop=actual_speed*actual_speed/(2.*a_limit);
  if(j_limit>0.) {
    const double settle=acceleration/j_limit;
    stop=speed*settle+acceleration*settle*settle/3.;
    speed+=acceleration*acceleration/(2.*j_limit);
    const double duration=speed<=a_limit*a_limit/j_limit?
      2.*std::sqrt(speed/j_limit):speed/a_limit+a_limit/j_limit;
    stop+=.5*speed*duration;
  }
  return actual_speed*lead+stop;
}

} // namespace scan_planner
