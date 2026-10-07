#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace scan_planner {

// Explicit isolated-simulation truth, separate from all laser odds/stamps.
// A complete registered actor snapshot is an additional veto, never FREE
// evidence. Copying this object copies its original receipt and finite lease.
class DynamicOccupancyOracle {
 public:
  static constexpr std::int64_t kMaximumAgeNs=200000000LL;
  static constexpr std::int64_t kMaximumHorizonNs=300000000LL;
  static constexpr std::int64_t kMaximumReachableHorizonNs=8000000000LL;
  struct Context { std::string session,seed; std::uint64_t epoch=0,sequence=0; };
  struct Region {
    std::array<int,3> lower,upper; int state=2;
    std::uint16_t actor_id=0; // One-based ordinal of the exact sorted registry.
    bool sphere=false; std::array<double,3> center{}; double radius=0.;
  };
  void configure(std::string registry,std::string frame,std::vector<std::string> actors,double resolution) {
    if(registry.size()!=64||registry.find_first_not_of("0123456789abcdef")!=std::string::npos||frame.empty()||
        !std::isfinite(resolution)||resolution<=0.||actors.size()>64)
      throw std::invalid_argument("invalid dynamic occupancy oracle configuration");
    std::set<std::string> unique;
    for(const auto &actor:actors)if(actor.empty()||actor.size()>128||!unique.insert(actor).second)
      throw std::invalid_argument("invalid dynamic occupancy actor registry");
    registry_=std::move(registry);frame_=std::move(frame);actors_=std::move(unique);resolution_=resolution;
    enabled_=true;sequence_=seen_sequence_=0;source_ns_=seen_source_ns_=0;context_={};revoke();
  }
  bool enabled() const {return enabled_;}
  std::uint16_t actorCount() const {return static_cast<std::uint16_t>(actors_.size());}
  std::vector<std::string> actorIds() const {return {actors_.begin(),actors_.end()};}
  void revoke() {valid_=false;regions_.clear();}
  bool apply(const std::string &payload,const Context &expected,std::int64_t receipt_ns) {
    if(!enabled_)return false;
    try {
      if(payload.empty()||payload.size()>65536||receipt_ns<=0||expected.session.empty()||expected.seed.empty()||
          !expected.epoch||!expected.sequence)throw std::invalid_argument("oracle missing bounded context");
      const auto value=nlohmann::json::parse(payload);
      const auto unsigned_value=[](const nlohmann::json &v)->std::uint64_t {
        if(!v.is_number_unsigned())throw std::invalid_argument("oracle unsigned identity required");
        return v.get<std::uint64_t>();
      };
      const auto positive_ns=[](const nlohmann::json &v)->std::int64_t {
        if(!v.is_number_integer()||v.is_boolean()||(v.is_number_unsigned()&&v.get<std::uint64_t>()>
            static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())))
          throw std::invalid_argument("oracle integer source required");
        const auto n=v.get<std::int64_t>();if(n<=0)throw std::invalid_argument("oracle nonpositive source");return n;
      };
      if(value.at("schema")!=1||value.at("kind")!="isaac_dynamic_occupancy_oracle_v1"||
          value.at("session_id")!=expected.session||value.at("seed_id")!=expected.seed||
          unsigned_value(value.at("epoch"))!=expected.epoch||
          unsigned_value(value.at("context_sequence"))!=expected.sequence||
          value.at("registry_sha256")!=registry_||value.at("frame_id")!=frame_||
          !value.at("complete").is_boolean()||value.at("complete")!=true)
        throw std::invalid_argument("oracle identity or completeness mismatch");
      const auto sequence=unsigned_value(value.at("sequence"));
      const auto source=positive_ns(value.at("source_stamp_ns"));
      const auto deadline=positive_ns(value.at("valid_until_ns"));
      const auto reachable_horizon=positive_ns(value.at("reachable_horizon_ns"));
      const auto reachable_until=positive_ns(value.at("reachable_until_ns"));
      if(reachable_horizon>kMaximumReachableHorizonNs||reachable_until<source||
          reachable_until-source!=reachable_horizon)throw std::invalid_argument("oracle reachable horizon mismatch");
      if(context_.session!=expected.session||context_.seed!=expected.seed||context_.epoch!=expected.epoch||
          context_.sequence!=expected.sequence) {
        // Only an exact active identity above may open a new replay domain.
        revoke();sequence_=seen_sequence_=0;source_ns_=seen_source_ns_=0;context_=expected;
      }
      if(!sequence||deadline<=source||deadline-source>kMaximumHorizonNs)
        throw std::invalid_argument("oracle invalid finite horizon");
      // A reliable exact duplicate cannot refresh its actual receipt lease.
      if(valid_&&sequence==sequence_&&payload==payload_)return true;
      if(sequence<=seen_sequence_||source<=seen_source_ns_)
        throw std::invalid_argument("oracle replay or source rollback");
      seen_sequence_=sequence;seen_source_ns_=source; // Malformed actor snapshots cannot later replay as valid.
      const auto &actor_values=value.at("actors");
      if(!actor_values.is_array()||actor_values.size()!=actors_.size())
        throw std::invalid_argument("oracle missing registered actors");
      std::set<std::string> actual;
      std::vector<Region> regions;
      for(const auto &actor:actor_values) {
        const auto id=actor.at("actor_id").get<std::string>();
        if(!actors_.count(id)||!actual.insert(id).second)throw std::invalid_argument("oracle unknown/duplicate actor");
        const auto &boxes=actor.at("regions");
        if(!boxes.is_array()||boxes.empty()||boxes.size()>64)throw std::invalid_argument("oracle absent/oversize actor volume");
        for(const auto &box:boxes) {
          Region r;
          r.actor_id=static_cast<std::uint16_t>(std::distance(actors_.begin(),actors_.find(id))+1);
          if(!box.at("state").is_number_integer())throw std::invalid_argument("oracle state required");
          r.state=box.at("state").get<int>();if(r.state!=1&&r.state!=2)throw std::invalid_argument("oracle cannot certify free");
          const auto &lo=box.at("min"),&hi=box.at("max");
          if(!lo.is_array()||lo.size()!=3||!hi.is_array()||hi.size()!=3)throw std::invalid_argument("oracle malformed region");
          if(box.contains("enclosure")) {
            if(box.at("enclosure")!="sphere_v1"||!box.contains("center")||!box.contains("radius"))
              throw std::invalid_argument("oracle unsupported or incomplete sphere");
            const auto &center=box.at("center"),&radius=box.at("radius");
            if(!center.is_array()||center.size()!=3||!radius.is_number())
              throw std::invalid_argument("oracle malformed sphere");
            r.radius=radius.get<double>();
            if(!std::isfinite(r.radius)||r.radius<=0.)throw std::invalid_argument("oracle invalid sphere radius");
            for(std::size_t d=0;d<3;++d) {
              if(!center[d].is_number())throw std::invalid_argument("oracle numeric sphere center required");
              r.center[d]=center[d].get<double>();
              if(!std::isfinite(r.center[d]))throw std::invalid_argument("oracle invalid sphere center");
            }
            r.sphere=true;
          } else if(box.contains("center")||box.contains("radius")) {
            throw std::invalid_argument("oracle incomplete sphere declaration");
          }
          for(std::size_t d=0;d<3;++d) {
            if(!lo[d].is_number()||!hi[d].is_number())throw std::invalid_argument("oracle numeric bounds required");
            const auto a=lo[d].get<double>(),b=hi[d].get<double>();
            if(!std::isfinite(a)||!std::isfinite(b)||a>=b)throw std::invalid_argument("oracle invalid closed bounds");
            if(r.sphere&&(a!=r.center[d]-r.radius||b!=r.center[d]+r.radius))
              throw std::invalid_argument("oracle sphere broad phase mismatch");
            const auto lower=std::ceil(a/resolution_-1e-10)-1.;
            const auto upper=std::floor(b/resolution_+1e-10);
            if(lower<std::numeric_limits<int>::min()||upper>std::numeric_limits<int>::max())
              throw std::invalid_argument("oracle world index overflow");
            r.lower[d]=static_cast<int>(lower);r.upper[d]=static_cast<int>(upper);
          }
          regions.push_back(r);
        }
      }
      regions_=std::move(regions);sequence_=sequence;source_ns_=source;deadline_ns_=deadline;
      reachable_until_ns_=reachable_until;receipt_ns_=receipt_ns;context_=expected;payload_=payload;valid_=true;return true;
    } catch(const std::exception &) {revoke();return false;}
  }
  bool live(const Context &context,std::int64_t source_now,std::int64_t receipt_now) const {
    return enabled_&&valid_&&context.session==context_.session&&context.seed==context_.seed&&
        context.epoch==context_.epoch&&context.sequence==context_.sequence&&source_now>=source_ns_&&
        source_now<sourceDeadlineNs()&&receipt_now>=receipt_ns_&&receipt_now<receiptDeadlineNs();
  }
  int status(const std::array<int,3> &cell) const {
    if(!valid_)return 2;
    int state=0;
    for(const auto &r:regions_) {
      bool intersects=true;
      for(std::size_t d=0;d<3;++d)if(cell[d]<r.lower[d]||cell[d]>r.upper[d]){intersects=false;break;}
      if(intersects&&(!r.sphere||sphereIntersectsCell(r,cell,3))){if(r.state==1)return 1;state=2;}
    }
    return state; // Zero only means no dynamic veto; callers still need static/live FREE.
  }
  int actorStatus(std::uint16_t actor_id,const std::array<int,3>& cell) const {
    if(!valid_||!actor_id||actor_id>actorCount())return 2;
    int state=0;
    for(const auto& r:regions_)if(r.actor_id==actor_id) {
      bool intersects=true;
      for(std::size_t d=0;d<3;++d)if(cell[d]<r.lower[d]||cell[d]>r.upper[d]){intersects=false;break;}
      if(intersects&&(!r.sphere||sphereIntersectsCell(r,cell,3))){if(r.state==1)return 1;state=2;}
    }
    return state; // Only withdrawal of this actor's veto, never independent FREE.
  }
  bool intersectsColumnXY(int x,int y) const {
    if(!valid_)return true; // Never a disjointness proof from revoked data.
    for(const auto& r:regions_)
      if(x>=r.lower[0]&&x<=r.upper[0]&&y>=r.lower[1]&&y<=r.upper[1]&&
          (!r.sphere||sphereIntersectsCell(r,{x,y,0},2)))return true;
    return false; // Exact closed XY disjointness excludes every Z of this column.
  }
  std::int64_t sourceDeadlineNs() const {
    if(source_ns_>std::numeric_limits<std::int64_t>::max()-kMaximumAgeNs)return 1;
    return std::min(deadline_ns_,source_ns_+kMaximumAgeNs);
  }
  std::int64_t receiptDeadlineNs() const {
    return receipt_ns_>std::numeric_limits<std::int64_t>::max()-kMaximumAgeNs?1:receipt_ns_+kMaximumAgeNs;
  }
  std::uint64_t sequence() const {return sequence_;}
  std::int64_t sourceNs() const {return source_ns_;}
  std::int64_t reachableUntilNs() const {return reachable_until_ns_;}
  const std::string &registry() const {return registry_;}
  std::size_t bytes() const {return regions_.capacity()*sizeof(Region)+payload_.capacity();}
 private:
  bool sphereIntersectsCell(const Region& r,const std::array<int,3>& cell,std::size_t dimensions) const {
    long double distance_squared=0.;
    for(std::size_t d=0;d<dimensions;++d) {
      // Closed, outward-rounded complete voxel faces. In particular, neither
      // a center-only test nor an XY-only test may authorize a foot/top cell.
      const auto low=std::nextafter(static_cast<double>(cell[d])*resolution_,
          -std::numeric_limits<double>::infinity());
      const auto high=std::nextafter((static_cast<double>(cell[d])+1.)*resolution_,
          std::numeric_limits<double>::infinity());
      const long double gap=std::max({static_cast<long double>(low)-r.center[d],0.L,
          static_cast<long double>(r.center[d])-high});
      distance_squared+=gap*gap;
    }
    const long double radius_squared=static_cast<long double>(r.radius)*r.radius;
    const auto scale=static_cast<double>(std::max({1.L,distance_squared,radius_squared}));
    if(!std::isfinite(scale))return true; // Unsupported numeric range cannot prove disjointness.
    const long double guard=16.L*(std::nextafter(scale,std::numeric_limits<double>::infinity())-scale);
    return distance_squared<=radius_squared+guard;
  }
  bool enabled_=false,valid_=false;
  std::string registry_,frame_,payload_;
  std::set<std::string> actors_;
  double resolution_=0.;
  Context context_;
  std::vector<Region> regions_;
  std::uint64_t sequence_=0;
  std::uint64_t seen_sequence_=0;
  std::int64_t seen_source_ns_=0;
  std::int64_t source_ns_=0,deadline_ns_=0,receipt_ns_=0,reachable_until_ns_=0;
};
} // namespace scan_planner
