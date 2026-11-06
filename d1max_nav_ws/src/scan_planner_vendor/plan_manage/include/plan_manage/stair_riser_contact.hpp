#pragma once

#include <plan_manage/stair_contact_query.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

namespace scan_planner {
// Explain an already observed riser hit from BOTH adjacent qualified treads
// and either their actual local common edge or independent original riser
// observations. This never generates occupancy/free space or stair ability.
// The caller must first exclude every intersection with its rigid chassis.
class StairRiserContactQuery {
  struct Key {int x,y;bool operator==(const Key& b)const{return x==b.x&&y==b.y;}};
  struct Hash {std::size_t operator()(const Key& k)const{return std::hash<int>{}(k.x)^
    (std::hash<int>{}(k.y)<<1);}};
  struct Sample {Eigen::Vector3d point;int owner;bool qualified;};
  struct Patch {
    int owner{-1};Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
    Eigen::Vector2d grade{Eigen::Vector2d::Zero()};std::vector<const Sample*> points;
    std::vector<const Sample*> literal_constraints;
    bool fitted{false};
    double height(const Eigen::Vector2d& xy)const{return mean.z()+grade.dot(xy-mean.head<2>());}
  };
  std::shared_ptr<const PlanningSupport> support_;
  std::unordered_map<Key,std::vector<Sample>,Hash> bins_;
  std::unordered_map<Key,std::vector<Sample>,Hash> riser_bins_;
  double resolution_{0.};bool valid_{false};
  Key key(const Eigen::Vector3d& p)const {
    return {int(std::floor(p.x()/resolution_)),int(std::floor(p.y()/resolution_))};
  }
  static bool bounded(const Eigen::Vector3d& p) {
    return p.allFinite()&&(p.array().abs()<=1e5).all();
  }
  bool fit(Patch& patch)const {
    if(patch.points.size()<6)return false;
    for(const auto* p:patch.points)patch.mean+=p->point;
    patch.mean/=static_cast<double>(patch.points.size());
    Eigen::Matrix2d matrix=Eigen::Matrix2d::Zero();Eigen::Vector2d rhs=Eigen::Vector2d::Zero();
    for(const auto* p:patch.points) {
      const auto d=(p->point-patch.mean).eval();
      matrix.noalias()+=d.head<2>()*d.head<2>().transpose();rhs.noalias()+=d.head<2>()*d.z();
    }
    const double trace=matrix.trace();
    if(!(trace>0.)||matrix.determinant()<=1e-5*trace*trace)return false;
    patch.grade=matrix.inverse()*rhs;
    if(!patch.grade.allFinite()||patch.grade.norm()>.35)return false;
    double sq=0.;
    for(const auto* p:patch.points) {
      const double error=p->point.z()-patch.height(p->point.head<2>());
      if(std::abs(error)>resolution_*.25)return false;
      sq+=error*error;
    }
    // Exclusion from contact qualification is not permission to erase a
    // contradicting observed elevation inside this height cohort. These rows
    // constrain the qualified fit; they never participate in fitting it.
    for(const auto* p:patch.literal_constraints)
      if(std::abs(p->point.z()-patch.height(p->point.head<2>()))>resolution_*.25)return false;
    return std::sqrt(sq/patch.points.size())<=resolution_*.1;
  }
  bool originalRiserExplains(const Patch& lower,const Patch& upper,
      const Eigen::Vector2d& normal,const Eigen::Vector2d& lateral,
      double lower_edge,double upper_edge,const Eigen::Vector3d& lo,const Eigen::Vector3d& hi,
      double side_low,double side_high,const std::vector<const Sample*>& original)const {
    const double quant=resolution_/255.,epsilon=quant+1e-9;
    const double side_center=(side_low+side_high)*.5,z_center=(lo.z()+hi.z())*.5;
    const double n_center=(lower_edge+upper_edge)*.5;
    std::vector<Eigen::Vector3d> points; // literal (lateral, height, normal)
    for(const auto* sample:original) {
      const auto& p=sample->point;const auto xy=p.head<2>();
      const double n=normal.dot(xy),side=lateral.dot(xy);
      if(n<lower_edge-epsilon||n>upper_edge+epsilon||
          side<side_low-resolution_||side>side_high+resolution_||
          p.z()<=lower.height(xy)+resolution_*.25||
          p.z()>=upper.height(xy)-resolution_*.25)continue;
      // Raw original candidates also contain horizontal tread returns. Fit
      // the vertical interior first, then use only actual collinear original
      // endpoints for full-height coverage. Do not rotate a riser to fit its
      // neighbouring horizontal treads or synthesize an unsampled edge.
      points.emplace_back(side-side_center,p.z()-z_center,n-n_center);
    }
    if(points.size()<6)return false;
    Eigen::Matrix3d matrix=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
    Eigen::Vector2d mean=Eigen::Vector2d::Zero();
    for(const auto& p:points) {
      const Eigen::Vector3d feature(p.x(),p.y(),1.);
      matrix.noalias()+=feature*feature.transpose();rhs.noalias()+=feature*p.z();mean+=p.head<2>();
    }
    mean/=static_cast<double>(points.size());Eigen::Matrix2d extent=Eigen::Matrix2d::Zero();
    for(const auto& p:points){const auto d=(p.head<2>()-mean).eval();extent.noalias()+=d*d.transpose();}
    if(extent.trace()<=0.||extent.determinant()<=1e-5*extent.trace()*extent.trace())return false;
    const Eigen::LDLT<Eigen::Matrix3d> solve(matrix);
    if(solve.info()!=Eigen::Success||!solve.isPositive())return false;
    const Eigen::Vector3d plane=solve.solve(rhs);
    if(!plane.allFinite()||plane.head<2>().norm()>.35)return false;
    double square=0.,residual_low=INFINITY,residual_high=-INFINITY;
    for(const auto& p:points) {
      const double error=p.z()-plane.dot(Eigen::Vector3d(p.x(),p.y(),1.));
      if(std::abs(error)>resolution_*.25)return false;
      square+=error*error;
      residual_low=std::min(residual_low,error);residual_high=std::max(residual_high,error);
    }
    if(std::sqrt(square/points.size())>resolution_*.1)return false;
    const auto face=[&](double side,double z) {
      return n_center+plane.dot(Eigen::Vector3d(side-side_center,z-z_center,1.));
    };
    // Source candidates are not pre-labelled as risers and can contain walls.
    // Do not crop a known wall to the two tread elevations and call its lower
    // fragment a step face. Same observed face above the upper tread is
    // contrary original evidence, not an extra permission surface.
    for(const auto* sample:original) {
      const auto& p=sample->point;const double side=lateral.dot(p.head<2>());
      const double residual=normal.dot(p.head<2>())-face(side,p.z());
      if(side>=side_low-epsilon&&side<=side_high+epsilon&&
          p.z()>upper.height(p.head<2>())+resolution_*.25&&
          residual>=residual_low-epsilon&&residual<=residual_high+epsilon)return false;
    }
    double bottom=INFINITY,top=-INFINITY;
    for(int corner=0;corner<4;++corner) {
      const Eigen::Vector2d xy(corner&2?hi.x():lo.x(),corner&1?hi.y():lo.y());
      bottom=std::min(bottom,lower.height(xy));top=std::max(top,upper.height(xy));
    }
    // Keep only literal points, selecting a spatially distributed bounded
    // subset before the existing 64-vertex triangulator. No synthetic riser
    // endpoint or predicted ground point enters this observed-surface mesh.
    std::vector<Eigen::Vector2d> all;all.reserve(original.size());
    for(const auto* sample:original) {
      const auto& p=sample->point;const double side=lateral.dot(p.head<2>());
      const double n=normal.dot(p.head<2>()),error=n-face(side,p.z());
      if(n>=lower_edge-epsilon&&n<=upper_edge+epsilon&&
          side>=side_low-resolution_&&side<=side_high+resolution_&&
          p.z()>=lower.height(p.head<2>())-epsilon&&p.z()<=upper.height(p.head<2>())+epsilon&&
          error>=residual_low-epsilon&&error<=residual_high+epsilon)all.emplace_back(side,p.z());
    }
    std::vector<Eigen::Vector2d> mesh;
    if(all.size()<=64)mesh=all;
    else {
      std::vector<double> distance(all.size(),INFINITY);std::size_t selected=0;
      for(std::size_t count=0;count<64;++count) {
        mesh.push_back(all[selected]);distance[selected]=-1.;double farthest=-1.;std::size_t next=selected;
        for(std::size_t i=0;i<all.size();++i)if(distance[i]>=0.) {
          distance[i]=std::min(distance[i],(all[i]-all[selected]).squaredNorm());
          if(distance[i]>farthest){farthest=distance[i];next=i;}
        }
        selected=next;
      }
    }
    // Full vertical source coverage is required, not just the hit's small
    // patch: a short box observed between two treads cannot stand in for an
    // entire riser. Ground height/owner remain the original adjacent pair.
    // PCD float32 endpoints and a float64 fitted tread can differ by a few
    // nanometres. Use the same outward endpoint quantisation already required
    // by the raw ALL-hit representation, not an enlarged mesh edge/radius.
    // Only align to actually observed extremes; coverage across their full
    // width still has to be supplied by literal triangles below.
    double observed_bottom=INFINITY,observed_top=-INFINITY;
    for(const auto& p:mesh){observed_bottom=std::min(observed_bottom,p.y());observed_top=std::max(observed_top,p.y());}
    if(observed_bottom-bottom>epsilon||top-observed_top>epsilon)return false;
    const double covered_bottom=std::max(bottom,observed_bottom),covered_top=std::min(top,observed_top);
    if(!stairObservedRectangleCovered(mesh,{side_low,covered_bottom},{side_high,covered_top},2.*resolution_))return false;
    bool thin=true,solid=true;double upper_error=INFINITY;
    for(int corner=0;corner<8;++corner) {
      const Eigen::Vector3d p(corner&4?hi.x():lo.x(),corner&2?hi.y():lo.y(),corner&1?hi.z():lo.z());
      const double side=lateral.dot(p.head<2>()),n=normal.dot(p.head<2>()),expected=face(side,p.z());
      if(expected<lower_edge-epsilon||expected>upper_edge+epsilon)return false;
      // Fit tolerance decides whether the original patch is coherent; it is
      // NOT spare free thickness for a new object. Current all-hit bounds must
      // stay inside the actually observed normal envelope plus quantisation.
      thin=thin&&n>=expected+residual_low-epsilon&&n<=expected+residual_high+epsilon;
      solid=solid&&n>=expected+residual_low-epsilon&&n<=upper_edge+resolution_;
      upper_error=std::min(upper_error,std::abs(hi.z()-upper.height(p.head<2>())));
    }
    if(thin)return true;
    if(!solid||upper_error>epsilon)return false;
    // A mixed riser/top AABB may describe the step solid only when actual
    // original riser-top returns and qualified upper-tread points together
    // cover its complete horizontal footprint. No ideal top edge is added.
    std::vector<Eigen::Vector2d> top_xy;
    for(const auto* p:upper.points)top_xy.emplace_back(normal.dot(p->point.head<2>()),lateral.dot(p->point.head<2>()));
    for(const auto* sample:original) {
      const auto& p=sample->point;const double n=normal.dot(p.head<2>()),side=lateral.dot(p.head<2>());
      if(n>=lower_edge-epsilon&&n<=upper_edge+epsilon&&
          std::abs(p.z()-upper.height(p.head<2>()))<=epsilon&&
          n-face(side,p.z())>=residual_low-epsilon&&
          n-face(side,p.z())<=residual_high+epsilon)top_xy.emplace_back(n,side);
    }
    double normal_low=INFINITY,normal_high=-INFINITY;
    for(int corner=0;corner<4;++corner) {
      const Eigen::Vector2d xy(corner&2?hi.x():lo.x(),corner&1?hi.y():lo.y());
      normal_low=std::min(normal_low,normal.dot(xy));normal_high=std::max(normal_high,normal.dot(xy));
    }
    double actual_front=INFINITY;
    for(const auto& p:top_xy)actual_front=std::min(actual_front,p.x());
    if(actual_front-normal_low>epsilon)return false;
    normal_low=std::max(normal_low,actual_front);
    return stairObservedRectangleCovered(std::move(top_xy),
        {normal_low,side_low},{normal_high,side_high},2.*resolution_);
  }
  bool pairExplains(const Patch& lower,const Patch& upper,
      const Eigen::Vector3d& lo,const Eigen::Vector3d& hi,
      const std::vector<const Sample*>& source_risers)const {
    if(lower.owner!=upper.owner)return false;
    const Eigen::Vector2d delta=upper.mean.head<2>()-lower.mean.head<2>();
    if(delta.norm()<resolution_)return false;
    Eigen::Vector2d normal=delta.normalized();
    // Centroids only locate the facing edges. Different sampling densities
    // shift their lateral centroids and must not rotate the physical edge.
    // Fit a common direction to the two separately centred observed strips.
    double initial_lower=-INFINITY,initial_upper=INFINITY;
    for(const auto* p:lower.points)initial_lower=std::max(initial_lower,normal.dot(p->point.head<2>()));
    for(const auto* p:upper.points)initial_upper=std::min(initial_upper,normal.dot(p->point.head<2>()));
    std::array<std::vector<Eigen::Vector2d>,2> strips;
    for(const auto* p:lower.points)if(initial_lower-normal.dot(p->point.head<2>())<=resolution_*.5)
      strips[0].push_back(p->point.head<2>());
    for(const auto* p:upper.points)if(normal.dot(p->point.head<2>())-initial_upper<=resolution_*.5)
      strips[1].push_back(p->point.head<2>());
    Eigen::Matrix2d edge_covariance=Eigen::Matrix2d::Zero();
    std::array<Eigen::Vector2d,2> means{Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    for(std::size_t side=0;side<2;++side) {
      if(strips[side].size()<2)return false;
      for(const auto& p:strips[side])means[side]+=p;
      means[side]/=static_cast<double>(strips[side].size());
      for(const auto& p:strips[side]) {
        const auto d=(p-means[side]).eval();edge_covariance.noalias()+=d*d.transpose();
      }
    }
    if(edge_covariance.trace()<=0.)return false;
    const double tangent=.5*std::atan2(2.*edge_covariance(0,1),edge_covariance(0,0)-edge_covariance(1,1));
    normal={-std::sin(tangent),std::cos(tangent)};
    if(normal.dot(delta)<0.)normal=-normal;
    for(std::size_t side=0;side<2;++side)for(const auto& p:strips[side])
      if(std::abs(normal.dot(p-means[side]))>resolution_*.25)return false;
    const Eigen::Vector2d lateral(-normal.y(),normal.x());
    double lower_edge=-INFINITY,upper_edge=INFINITY,lower_back=INFINITY,upper_back=-INFINITY;
    for(const auto* p:lower.points) {
      const double n=normal.dot(p->point.head<2>());lower_edge=std::max(lower_edge,n);lower_back=std::min(lower_back,n);
    }
    for(const auto* p:upper.points) {
      const double n=normal.dot(p->point.head<2>());upper_edge=std::min(upper_edge,n);upper_back=std::max(upper_back,n);
    }
    // Two separated tread samples do not locate the intervening riser: a new
    // thin object on the unseen lower-tread extension can give the same hits.
    // Without independent original riser evidence, their locally observed
    // fronts must coincide within endpoint outward quantisation. The 1 nm roundoff
    // matches the Z comparisons below, not an additional spatial clearance.
    // At least one voxel of qualified tread must still back each actual edge.
    const double quant=resolution_/255.,edge_epsilon=quant+1e-9,gap=upper_edge-lower_edge;
    if(gap<-edge_epsilon||gap>2.*resolution_||lower_edge-lower_back<resolution_||
        upper_back-upper_edge<resolution_)return false;
    std::vector<double> low_side,high_side;
    // Only real front samples, never horizontal backing, establish a shared
    // edge. Separate original riser observations may establish a missing edge.
    for(const auto* p:lower.points)if(lower_edge-normal.dot(p->point.head<2>())<=edge_epsilon)
      low_side.push_back(lateral.dot(p->point.head<2>()));
    for(const auto* p:upper.points)if(normal.dot(p->point.head<2>())-upper_edge<=edge_epsilon)
      high_side.push_back(lateral.dot(p->point.head<2>()));
    if(low_side.size()<2||high_side.size()<2)return false;
    std::sort(low_side.begin(),low_side.end());std::sort(high_side.begin(),high_side.end());
    const double side_low=std::max(low_side.front(),high_side.front());
    const double side_high=std::min(low_side.back(),high_side.back());
    if(side_high-side_low<resolution_)return false;
    // Qualified gap/backing/edge-strip/lateral evidence above must stand on
    // its own. All observed same-cohort points can only SHRINK that permitted
    // gap. In particular a real lower tread extending into it is not a riser,
    // even when the extension has deliberately no contact qualification.
    double observed_lower_edge=lower_edge,observed_upper_edge=upper_edge;
    for(const auto* p:lower.literal_constraints)
      observed_lower_edge=std::max(observed_lower_edge,normal.dot(p->point.head<2>()));
    for(const auto* p:upper.literal_constraints)
      observed_upper_edge=std::min(observed_upper_edge,normal.dot(p->point.head<2>()));
    if(observed_upper_edge-observed_lower_edge<-edge_epsilon)return false;
    double n_low=INFINITY,n_high=-INFINITY,s_low=INFINITY,s_high=-INFINITY;
    double upper_endpoint_error=INFINITY;
    for(int corner=0;corner<4;++corner) {
      const Eigen::Vector2d xy(corner&2?hi.x():lo.x(),corner&1?hi.y():lo.y());
      const double n=normal.dot(xy),side=lateral.dot(xy);
      n_low=std::min(n_low,n);n_high=std::max(n_high,n);
      s_low=std::min(s_low,side);s_high=std::max(s_high,side);
      if(n<observed_lower_edge-edge_epsilon||side<side_low-quant||side>side_high+quant)return false;
      const double low_z=lower.height(xy),high_z=upper.height(xy),rise=high_z-low_z;
      // 25 cm is only the documented geometric ceiling, not a physical
      // acceptance flag. No new step is inferred when the neighbouring source
      // is missing or when the measured rise exceeds that ceiling.
      if(rise<=resolution_*.5||rise>.25||lo.z()<low_z-quant-1e-9||hi.z()>high_z+quant+1e-9)return false;
      upper_endpoint_error=std::min(upper_endpoint_error,std::abs(hi.z()-high_z));
    }
    const auto edge_covers=[&](const std::vector<double>& edges) {
      double covered=s_low;
      for(std::size_t i=1;i<edges.size();++i) {
        const double a=edges[i-1],b=edges[i];
        if(b-a>2.*resolution_+1e-9)continue;
        if(a>covered+edge_epsilon)return false;
        if(b>=covered-edge_epsilon) {
          covered=std::max(covered,b);
          if(covered>=s_high-edge_epsilon)return true;
        }
      }
      return false;
    };
    const bool common_edge=gap<=edge_epsilon&&edge_covers(low_side)&&edge_covers(high_side);
    if(!common_edge) {
      // The caller supplies independently bound ORIGINAL observations, never
      // the current occupied map. They must cover the entire riser height and
      // query width; a short current object cannot provide its own evidence.
      if(!originalRiserExplains(lower,upper,normal,lateral,observed_lower_edge,
          observed_upper_edge,lo,hi,s_low,s_high,source_risers))return false;
      return true;
    }
    if(n_high<=observed_upper_edge+edge_epsilon&&n_high-n_low<=resolution_*.5)return true;
    // A coarse voxel can retain both a measured riser and its top tread. Its
    // all-hit AABB is then not a thin sheet. The corner is contact geometry of
    // the already evidenced STEP SOLID, not a fictitious free-space surface:
    // it must reach the unchanged upper tread and its part behind the edge
    // must be entirely covered by that tread's measured triangulated support.
    // Keep the extension to this one voxel; never fill a whole ramp/ROI.
    if(n_low>observed_upper_edge+edge_epsilon||n_high>observed_upper_edge+resolution_||
        upper_endpoint_error>quant+1e-9)return false;
    std::vector<Eigen::Vector2d> upper_xy;upper_xy.reserve(upper.points.size());
    for(const auto* p:upper.points)upper_xy.emplace_back(normal.dot(p->point.head<2>()),lateral.dot(p->point.head<2>()));
    return stairObservedRectangleCovered(std::move(upper_xy),
        {std::max(n_low,observed_upper_edge),s_low},
        {std::max(n_high,observed_upper_edge+quant),s_high},2.*resolution_);
  }
public:
  StairRiserContactQuery(std::shared_ptr<const PlanningSupport> support,
      const std::string& frame,double resolution):support_(std::move(support)) {
    StairContactQuery qualification(support_,frame,resolution);
    if(!qualification.valid())return;
    resolution_=resolution;const auto& s=support_->source;
    std::vector<int> owner(s.support_ground_xyz.size(),-1);
    if(s.regions.empty()) {
      if(s.segment_kind=="stair_up"||s.segment_kind=="stair_down")std::fill(owner.begin(),owner.end(),0);
    } else {
      int id=0;for(const auto& region:s.regions) {
        if(region.required_mode=="stair"&&(region.segment_kind=="stair_up"||region.segment_kind=="stair_down"))
          for(std::size_t i=region.begin_index;i<region.end_index;++i)owner[i]=id;
        ++id;
      }
    }
    std::vector<bool> qualified(s.support_ground_xyz.size(),false);
    for(const auto i:s.stair_contact_indices)qualified[i]=owner[i]>=0;
    bins_.reserve(s.support_ground_xyz.size());
    for(std::size_t i=0;i<s.support_ground_xyz.size();++i) {
      const auto& p=s.support_ground_xyz[i];const Eigen::Vector3d xyz(p.x,p.y,p.z);
      bins_[key(xyz)].push_back({xyz,owner[i],qualified[i]});valid_|=qualified[i];
    }
    for(std::size_t i=0;i<s.stair_riser_xyz.size();++i) {
      const auto& p=s.stair_riser_xyz[i];const Eigen::Vector3d xyz(p.x,p.y,p.z);
      riser_bins_[key(xyz)].push_back({xyz,static_cast<int>(s.stair_riser_owner[i]),false});
    }
  }
  bool valid()const{return valid_;}
  bool matchesObservedBounds(const Eigen::Vector3d& lo,const Eigen::Vector3d& hi)const {
    if(!valid_||!bounded(lo)||!bounded(hi)||(lo.array()>hi.array()).any()||
        ((hi-lo).array()>resolution_+1e-9).any())return false;
    const Eigen::Vector3d center=(lo+hi)*.5;const auto cell=key(center);
    std::vector<const Sample*> local;const double radius=4.*resolution_;
    for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y) {
      const auto it=bins_.find({cell.x+x,cell.y+y});if(it==bins_.end())continue;
      for(const auto& p:it->second)if((p.point.head<2>()-center.head<2>()).squaredNorm()<=radius*radius&&
          std::abs(p.point.z()-center.z())<=.25+resolution_*.25)local.push_back(&p);
    }
    // Do not turn a closer unqualified return or an ownership boundary into a
    // riser by finding two convenient qualified treads elsewhere in the patch.
    int owner=-1;
    for(const auto* p:local)if(p->qualified) {
      if(owner<0)owner=p->owner;else if(owner!=p->owner)return false;
    }
    if(owner<0)return false;
    std::vector<const Sample*> source_risers;
    for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y) {
      const auto it=riser_bins_.find({cell.x+x,cell.y+y});if(it==riser_bins_.end())continue;
      for(const auto& p:it->second)if((p.point.head<2>()-center.head<2>()).squaredNorm()<=radius*radius&&
          std::abs(p.point.z()-center.z())<=.25+resolution_*.25) {
        if(p.owner!=owner) {
          const auto nearest=p.point.cwiseMax(lo).cwiseMin(hi);
          if((p.point-nearest).squaredNorm()<=resolution_*resolution_*.0625)return false;
        } else source_risers.push_back(&p);
      }
    }
    for(const auto* p:local) {
      const auto nearest=p->point.cwiseMax(lo).cwiseMin(hi);
      if((p->point-nearest).squaredNorm()<=resolution_*resolution_*.0625&&
          (!p->qualified||p->owner!=owner))return false;
    }
    // All literal elevations delimit adjacency. Filtering out an unqualified
    // or foreign-owner middle surface here would let the two qualified outer
    // treads incorrectly excuse an object above that known middle surface.
    // Such points are separators only, never contact-plane fit evidence.
    std::vector<const Sample*> points=local;
    std::sort(points.begin(),points.end(),[](const auto* a,const auto* b){return a->point.z()<b->point.z();});
    std::vector<Patch> patches;
    for(std::size_t begin=0;begin<points.size();) {
      std::size_t end=begin+1;
      while(end<points.size()&&points[end]->point.z()-points[begin]->point.z()<=resolution_*.5)++end;
      Patch patch;patch.owner=owner;
      patch.literal_constraints.assign(points.begin()+begin,points.begin()+end);
      for(std::size_t i=begin;i<end;++i)
        if(points[i]->qualified&&points[i]->owner==owner)patch.points.push_back(points[i]);
      // Keep every observed elevation cluster, including one that cannot fit
      // a plane. Otherwise two non-adjacent treads could invent a tall riser
      // across a real middle tread and excuse an object above that tread.
      patch.fitted=fit(patch);patches.push_back(std::move(patch));
      begin=end;
    }
    for(std::size_t i=1;i<patches.size();++i)
      if(patches[i-1].fitted&&patches[i].fitted&&
          pairExplains(patches[i-1],patches[i],lo,hi,source_risers))return true;
    return false;
  }
};
} // namespace scan_planner
