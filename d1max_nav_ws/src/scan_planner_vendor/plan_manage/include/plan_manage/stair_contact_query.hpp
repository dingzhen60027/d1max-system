#pragma once

#include <plan_manage/planning_support.hpp>
#include <plan_env/voxel_collision.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace scan_planner {

// A bounded local Delaunay mesh made only from literal XY returns. Sum the
// non-overlapping clipped triangle areas to cover the whole rectangle, not
// only sampled corners. Long triangles cannot bridge an unobserved hole.
inline bool stairObservedRectangleCovered(std::vector<Eigen::Vector2d> points,
    Eigen::Vector2d low,Eigen::Vector2d high,double max_edge) {
  if(points.size()<3||!low.allFinite()||!high.allFinite()||
      (low.array()>high.array()).any()||!std::isfinite(max_edge)||max_edge<=0.)return false;
  const Eigen::Vector2d origin=(low+high)*.5;
  for(auto& p:points)p-=origin;
  low-=origin;high-=origin;
  std::sort(points.begin(),points.end(),[](const auto& a,const auto& b) {
    if(a.squaredNorm()!=b.squaredNorm())return a.squaredNorm()<b.squaredNorm();
    return a.x()<b.x()||(a.x()==b.x()&&a.y()<b.y());
  });
  points.erase(std::unique(points.begin(),points.end(),[](const auto& a,const auto& b){return a==b;}),points.end());
  // Bounded cost even for malformed over-dense input. Omitting farther points
  // can only refuse a patch; it never expands the measured support mesh.
  if(points.size()>64)points.resize(64);
  const std::size_t count=points.size();if(count<3)return false;
  double radius=max_edge;for(const auto& p:points)radius=std::max(radius,p.norm());
  points.push_back({-16.*radius,-16.*radius});points.push_back({16.*radius,-16.*radius});
  points.push_back({0.,16.*radius});
  struct Triangle{std::size_t a,b,c;};
  std::vector<Triangle> triangles{{count,count+1,count+2}};
  const auto cross=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b){return a.x()*b.y()-a.y()*b.x();};
  for(std::size_t index=0;index<count;++index) {
    std::vector<Triangle> kept;std::vector<std::pair<std::size_t,std::size_t>> boundary;
    const auto edge=[&](std::size_t a,std::size_t b) {
      const auto reverse=std::make_pair(b,a);const auto it=std::find(boundary.begin(),boundary.end(),reverse);
      if(it==boundary.end())boundary.emplace_back(a,b);else boundary.erase(it);
    };
    for(const auto& t:triangles) {
      const auto a=(points[t.a]-points[index]).eval(),b=(points[t.b]-points[index]).eval(),c=(points[t.c]-points[index]).eval();
      const double determinant=a.squaredNorm()*cross(b,c)-b.squaredNorm()*cross(a,c)+c.squaredNorm()*cross(a,b);
      if(determinant>1e-15*radius*radius*radius*radius) {edge(t.a,t.b);edge(t.b,t.c);edge(t.c,t.a);}
      else kept.push_back(t);
    }
    for(const auto& e:boundary) {
      if(cross(points[e.second]-points[e.first],points[index]-points[e.first])>1e-16*radius*radius)
        kept.push_back({e.first,e.second,index});
    }
    triangles=std::move(kept);
  }
  // Tiny regularisation is numerical only; a degenerate point/line normally
  // already uses the stricter literal-source branch.
  for(int d=0;d<2;++d)if(high[d]-low[d]<1e-9){low[d]-=5e-10;high[d]+=5e-10;}
  double covered_area=0.;
  for(const auto& t:triangles) {
    if(t.a>=count||t.b>=count||t.c>=count)continue;
    const auto& a=points[t.a];const auto& b=points[t.b];const auto& c=points[t.c];
    const double bound=max_edge*max_edge*(1.+1e-10);
    if((a-b).squaredNorm()>bound||(b-c).squaredNorm()>bound||(c-a).squaredNorm()>bound)continue;
    std::vector<Eigen::Vector2d> polygon{a,b,c};
    for(int axis=0;axis<2;++axis)for(int side=0;side<2;++side) {
      std::vector<Eigen::Vector2d> clipped;if(polygon.empty())break;
      const double threshold=side?high[axis]:low[axis];
      const auto inside=[&](const auto& p){return side?p[axis]<=threshold:p[axis]>=threshold;};
      for(std::size_t i=0;i<polygon.size();++i) {
        const auto& start=polygon[i];const auto& end=polygon[(i+1)%polygon.size()];
        const bool first=inside(start),second=inside(end);
        if(first)clipped.push_back(start);
        if(first!=second)clipped.push_back(start+(end-start)*((threshold-start[axis])/(end[axis]-start[axis])));
      }
      polygon=std::move(clipped);
    }
    double area=0.;for(std::size_t i=0;i<polygon.size();++i)area+=cross(polygon[i],polygon[(i+1)%polygon.size()]);
    covered_area+=std::abs(area)*.5;
  }
  const double expected=(high.x()-low.x())*(high.y()-low.y());
  return covered_area>=expected*(1.-1e-8);
}

// Source association only. This does not change occupancy, move the robot,
// certify a gait, or excuse a hit in the rigid torso. The caller owns those
// decisions and the immutable source/version binding for its query scope.
class StairContactQuery {
  struct Key {
    int x,y,z;
    bool operator==(const Key& other)const {
      return x==other.x&&y==other.y&&z==other.z;
    }
  };
  struct Hash {
    std::size_t operator()(const Key& k)const {
      return std::hash<int>{}(k.x)^(std::hash<int>{}(k.y)<<1)^
          (std::hash<int>{}(k.z)<<2);
    }
  };
  struct Sample {Eigen::Vector3d point;bool qualified;std::size_t owner;};
  std::shared_ptr<const PlanningSupport> support_;
  std::unordered_map<Key,std::vector<Sample>,Hash> bins_;
  double resolution_{0.},bin_size_{0.},match_radius_{0.};
  std::size_t qualified_count_{0};
  bool valid_{false};

  Key key(const Eigen::Vector3d& p)const {
    return {int(std::floor(p.x()/bin_size_)),int(std::floor(p.y()/bin_size_)),
            int(std::floor(p.z()/bin_size_))};
  }
  static bool bounded(const Eigen::Vector3d& p) {
    return p.allFinite()&&(p.array().abs()<=1e5).all();
  }
  static double cross(const Eigen::Vector2d& a,const Eigen::Vector2d& b,
      const Eigen::Vector2d& c) {
    return (b.x()-a.x())*(c.y()-a.y())-(b.y()-a.y())*(c.x()-a.x());
  }
  // Local interpolation is permitted only within the convex hull of actual
  // qualified returns. This never extrapolates a tread over its observed edge.
  static std::vector<Eigen::Vector2d> hull(std::vector<Eigen::Vector2d> points) {
    std::sort(points.begin(),points.end(),[](const auto& a,const auto& b) {
      return a.x()<b.x()||(a.x()==b.x()&&a.y()<b.y());
    });
    points.erase(std::unique(points.begin(),points.end(),[](const auto& a,const auto& b) {
      return a==b;
    }),points.end());
    if(points.size()<3)return {};
    std::vector<Eigen::Vector2d> result;result.reserve(points.size()*2);
    for(const auto& p:points) {
      while(result.size()>1&&cross(result[result.size()-2],result.back(),p)<=0.)result.pop_back();
      result.push_back(p);
    }
    const auto lower=result.size();
    for(auto it=points.rbegin()+1;it!=points.rend();++it) {
      while(result.size()>lower&&cross(result[result.size()-2],result.back(),*it)<=0.)result.pop_back();
      result.push_back(*it);
    }
    result.pop_back();return result;
  }
public:
  // Both maps use the same global lattice as hitSubvoxelBit. At 5 cm map
  // resolution this matches at most 12.5 mm in 3D, not the much wider route
  // support lookup radius (normally 12 cm). There is no plane extrapolation.
  StairContactQuery(std::shared_ptr<const PlanningSupport> support,
      const std::string& frame,double resolution):support_(std::move(support)) {
    if(!support_||!support_->literalStairValid()||frame.empty()||support_->source.frame_id!=frame||
        !std::isfinite(resolution)||resolution<.001||resolution>1.)return;
    resolution_=resolution;bin_size_=resolution;match_radius_=resolution*.25;
    const auto& source=support_->source;
    std::vector<bool> stair(source.support_ground_xyz.size(),source.regions.empty());
    std::vector<std::size_t> owners(source.support_ground_xyz.size(),0);
    std::size_t owner=0;
    for(const auto& region:source.regions) {
      const bool owned=region.required_mode=="stair"&&
        (region.segment_kind=="stair_up"||region.segment_kind=="stair_down"||
         region.segment_kind=="landing");
      if(region.end_index>stair.size()||region.begin_index>region.end_index)return;
      for(std::size_t i=region.begin_index;i<region.end_index;++i) {
        stair[i]=owned;owners[i]=owner;
      }
      ++owner;
    }
    // Ordinary PCT-associated source points are not independently certified
    // tread returns. Only the explicit literal-source subset may establish a
    // contact plane. Missing metadata leaves the original collision intact.
    if(source.stair_contact_indices.empty())return;
    std::vector<bool> qualified(stair.size(),false);
    std::uint32_t previous=0;bool first=true;
    for(const auto index:source.stair_contact_indices) {
      if(index>=qualified.size()||qualified[index]||!stair[index]||(!first&&index<=previous))return;
      qualified[index]=true;++qualified_count_;
      previous=index;first=false;
    }
    bins_.reserve(source.support_ground_xyz.size());
    bool any=false;
    for(std::size_t i=0;i<source.support_ground_xyz.size();++i) {
      const auto& p=source.support_ground_xyz[i];const Eigen::Vector3d point(p.x,p.y,p.z);
      // PlanningSupport validates these coordinates. Retain a defensive bound
      // here so no malformed packet can overflow the finer spatial index.
      if(!bounded(point)) {bins_.clear();return;}
      bins_[key(point)].push_back({point,qualified[i],owners[i]});any|=qualified[i];
    }
    valid_=any;
  }
  bool valid()const {return valid_;}
  double matchRadius()const {return match_radius_;}
  std::size_t sourcePointCount()const {
    return valid_?support_->source.support_ground_xyz.size():0;
  }
  std::size_t qualifiedPointCount()const {return valid_?qualified_count_:0;}

  // Accept only a nearest literal stair return. At a floor/stair ownership
  // tie, the ordinary floor wins. A source point on a different height/floor
  // cannot match merely because its XY or a fitted ramp agrees.
  bool matchesObservedEndpoint(const Eigen::Vector3d& endpoint)const {
    if(!valid_||!bounded(endpoint))return false;
    const auto center=key(endpoint);
    double best=match_radius_*match_radius_;bool found=false,stair=false;
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z) {
      const auto it=bins_.find({center.x+x,center.y+y,center.z+z});
      if(it==bins_.end())continue;
      for(const auto& sample:it->second) {
        const double d=(sample.point-endpoint).squaredNorm();
        if(d>match_radius_*match_radius_)continue;
        if(!found||d<best) {found=true;best=d;stair=sample.qualified;}
        else if(d==best)stair=stair&&sample.qualified;
      }
    }
    return found&&stair;
  }

  // Explain the ENTIRE conservative endpoint AABB by a small observed tread.
  // lo/hi must include every retained hit in a voxel, not just the latest hit
  // or a union octant mask. This function never decides whether the robot's
  // rigid torso intersects: that independent hard-volume test must run first.
  //
  // At 5 cm map resolution: source collection <=10 cm XY / 2.5 cm Z;
  // <=5 mm fit RMS; <=12.5 mm source and endpoint plane residual; measured
  // triangles with edges <=10 cm cover the entire XY bounds. The collection
  // radius is NOT a matching allowance.
  // No stair-wide ramp, unobserved riser, box top or unknown hole is inferred.
  bool matchesObservedBounds(const Eigen::Vector3d& lo,const Eigen::Vector3d& hi)const {
    if(!valid_||!bounded(lo)||!bounded(hi)||(lo.array()>hi.array()).any()||
        ((hi-lo).array()>resolution_+1e-9).any())return false;
    const Eigen::Vector3d center=(lo+hi)*.5;
    const auto cell=key(center);
    std::vector<const Sample*> candidates;candidates.reserve(64);
    const double reach=2.*resolution_,z_reach=resolution_*.5;
    const Sample* nearest=nullptr;double nearest_distance=INFINITY;
    // Three bins of Z cover the bounded cohort. Any additional step/riser
    // heights remain separate rather than bending a plane through two treads.
    for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int z=-1;z<=1;++z) {
      const auto it=bins_.find({cell.x+x,cell.y+y,cell.z+z});
      if(it==bins_.end())continue;
      for(const auto& sample:it->second) {
        if((sample.point.head<2>()-center.head<2>()).squaredNorm()>reach*reach||
            std::abs(sample.point.z()-center.z())>z_reach)continue;
        candidates.push_back(&sample);
        if(!sample.qualified)continue;
        const double distance=(sample.point-center).squaredNorm();
        if(distance<nearest_distance) {nearest=&sample;nearest_distance=distance;}
      }
    }
    if(!nearest)return false;
    std::vector<const Sample*> patch;patch.reserve(candidates.size());
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();
    for(const auto* sample:candidates) {
      // The full conservative hit bounds, not a sampling stencil, own every
      // interior/boundary point. Explicitly excluded literal evidence inside
      // them cannot be bypassed by either a qualified ball or a fitted plane.
      if(!sample->qualified&&(sample->point.array()>=lo.array()).all()&&
          (sample->point.array()<=hi.array()).all())return false;
      // A nearby ownership boundary cannot borrow another floor/flight's
      // qualification to bridge the transition.
      if(sample->owner!=nearest->owner&&
          (sample->point.head<2>()-center.head<2>()).squaredNorm()<=resolution_*resolution_)
        return false;
      if(sample->qualified&&sample->owner==nearest->owner) {
        patch.push_back(sample);mean+=sample->point;
      }
    }
    // A sparse original return still has independently classified provenance.
    // It needs no six-point plane if the ENTIRE accumulated endpoint extent
    // fits its original 12.5 mm (at 5 cm) 3D association ball. A convex ball
    // containing all eight corners contains the whole AABB, not just one hit.
    // This cannot distinguish a new object thinner than that resolution band;
    // the caller's hard torso SAT remains mandatory in every case.
    for(const auto* sample:patch) {
      bool fits=true;
      for(int corner=0;corner<8&&fits;++corner) {
        const Eigen::Vector3d p(corner&4?hi.x():lo.x(),corner&2?hi.y():lo.y(),corner&1?hi.z():lo.z());
        fits=(p-sample->point).squaredNorm()<=match_radius_*match_radius_&&matchesObservedEndpoint(p);
      }
      if(fits)return true;
    }
    if(patch.size()<6)return false;
    mean/=static_cast<double>(patch.size());
    Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
    Eigen::Vector2d correlation=Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> xy;xy.reserve(patch.size());
    for(const auto* sample:patch) {
      const auto delta=(sample->point-mean).eval();
      covariance.noalias()+=delta.head<2>()*delta.head<2>().transpose();
      correlation.noalias()+=delta.head<2>()*delta.z();xy.push_back(sample->point.head<2>());
    }
    // Scale-relative non-collinearity avoids accepting a line of wall/riser
    // returns as a two-dimensional support patch.
    const double determinant=covariance.determinant(),trace=covariance.trace();
    if(!(trace>0.)||determinant<=1e-5*trace*trace)return false;
    const Eigen::Vector2d grade=covariance.inverse()*correlation;
    if(!grade.allFinite()||grade.norm()>.35)return false;
    double squared_error=0.;
    for(const auto* sample:patch) {
      const double error=sample->point.z()-mean.z()-grade.dot(sample->point.head<2>()-mean.head<2>());
      if(std::abs(error)>match_radius_)return false;
      squared_error+=error*error;
    }
    if(std::sqrt(squared_error/patch.size())>resolution_*.1)return false;
    const auto polygon=hull(xy);if(polygon.size()<3)return false;
    if(!stairObservedRectangleCovered(xy,lo.head<2>(),hi.head<2>(),resolution_*2.))return false;
    // Check the full bounding rectangle and its centre/edge midpoints. Hull
    // containment stops edge extrapolation; local measured coverage stops a
    // sparse convex hull from filling a larger unobserved hole.
    for(int x=0;x<3;++x)for(int y=0;y<3;++y) {
      const Eigen::Vector2d p(lo.x()+(hi.x()-lo.x())*.5*x,lo.y()+(hi.y()-lo.y())*.5*y);
      for(std::size_t i=0;i<polygon.size();++i)
        if(cross(polygon[i],polygon[(i+1)%polygon.size()],p)<-1e-12)return false;
      const double height=mean.z()+grade.dot(p-mean.head<2>());
      if(std::abs(lo.z()-height)>match_radius_||std::abs(hi.z()-height)>match_radius_)return false;
      // Interpolation cannot override closer contradictory literal evidence
      // from the same owner (e.g. an old PCT-associated but unqualified point).
      // No nearby literal is required here: hull/coverage still govern a small
      // measured gap, but a present unqualified nearest return is not ignored.
      for(const double z:{lo.z(),(lo.z()+hi.z())*.5,hi.z()}) {
        const Eigen::Vector3d point(p.x(),p.y(),z);
        bool found=false,qualified=false;double best=match_radius_*match_radius_;
        for(const auto* sample:candidates) {
          const double d=(sample->point-point).squaredNorm();
          if(d>match_radius_*match_radius_)continue;
          if(!found||d<best) {found=true;best=d;qualified=sample->qualified;}
          else if(d==best)qualified=qualified&&sample->qualified;
        }
        if(found&&!qualified)return false;
      }
    }
    return true;
  }

  struct EndpointClassification {
    std::uint16_t observed{0},matched{0},unmatched{0};
    bool complete{true};
    // A stair point and an unexplained object in the SAME subcell must not
    // turn that subcell into terrain. Unmatched evidence always wins.
    std::uint16_t contactMask()const {
      return complete?static_cast<std::uint16_t>(observed&matched&~unmatched&0xffU):0;
    }
  };
  // This helper requires actual endpoints, not only a historical union mask.
  // A union bit does not prove every contributing hit was a tread. Consumers
  // aggregating across scans must preserve unmatched evidence until genuine
  // occupancy clearing/reset; missing/coarse-only history cannot be excused.
  EndpointClassification classifyObservedEndpoints(const Eigen::Vector3i& coarse,
      const std::vector<Eigen::Vector3d>& endpoints)const {
    EndpointClassification result;
    if(!valid_||endpoints.empty()) {result.complete=false;return result;}
    for(const auto& endpoint:endpoints) {
      const auto bit=hitSubvoxelBit(endpoint,coarse,resolution_);
      if(!bit) {result.complete=false;continue;}
      result.observed|=bit;
      if(matchesObservedEndpoint(endpoint))result.matched|=bit;
      else result.unmatched|=bit;
    }
    return result;
  }
};
} // namespace scan_planner
