#pragma once

#include <plan_manage/collision_reference.hpp>
#include <plan_manage/planning_support.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/solve_budget.hpp>

namespace scan_planner {

struct ModelReferenceInitializerResult {
  bool attempted{false};
  bool accepted{false};
  std::string reason;
};

// A lattice's temporary diagonal body heading can prevent its initial detour
// even when the final continuous guide fits the modeled stair corridor. Only
// after that precise failure, generate ONE initializer proposal without the
// model's hard-boundary callback. Raw occupancy, unknown-space policy, support,
// attitude and the original absolute solve deadline all remain active.
// This proposal is not a collision proof: restore the complete model, then
// check every final guide edge before returning it to the normal optimizer and
// whole-curve admission. The reader belongs exclusively to this solve lane.
inline ModelReferenceInitializerResult tryModelReferenceInitializer(
    GridMap& map, const std::shared_ptr<const PlanningSupport>& support,
    const SolveBudget::Ptr& budget, const std::vector<Eigen::Vector3d>& reference,
    const OccupancyQuery& occupied, const DetourSearch& search,
    const std::string& first_reference_reason, const std::string& first_native_reason,
    std::vector<Eigen::Vector3d>& output, std::string& output_reason,
    double anchor_margin=.5) {
  ModelReferenceInitializerResult result;
  if(first_reference_reason!="failed_reference_search" ||
     first_native_reason!="open_set_exhausted" || !budget || !budget->allowed() ||
     !support || !support->valid || !support->isStair() || !support->model() ||
     support->source.frame_id!=map.collisionFrame() || reference.size()<2)
    return result;
  for(const auto& point:reference)if(!point.allFinite())return result;

  const OccupancyQuery checked_occupancy=[&](const Eigen::Vector3d& p,double yaw) {
    return budget->allowed()?occupied(p,yaw):-1;
  };
  const DetourSearch checked_search=[&](const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    return budget->allowed()?search(a,b):DetourSearchResult{};
  };
  std::vector<Eigen::Vector3d> proposal;
  bool ready=false;
  {
    struct RestoreBoundary {
      GridMap& map;
      GridMap::QueryHardBoundaryProvider previous;
      ~RestoreBoundary(){map.exchangeQueryHardBoundaryProvider(std::move(previous));}
    } restore{map,map.exchangeQueryHardBoundaryProvider({})};
    // A model in the packet is insufficient: its same-frame reader binding
    // must actually exist. Never produce a second geometry interpretation for
    // an unbound/foreign reader. The RAII guard also restores on exceptions.
    if(!restore.previous)return result;
    result.attempted=true;
    ready=buildCollisionAwareReference(reference,map.getResolution(),checked_occupancy,
        checked_search,proposal,result.reason,anchor_margin);
  }
  if(!budget->allowed()){result.reason=budget->reason();return result;}
  if(!ready)return result;
  if(proposal.size()<2 || proposal.size()>20000 ||
     !proposal.front().isApprox(reference.front(),1e-12) ||
     !proposal.back().isApprox(reference.back(),1e-12)) {
    result.reason="failed_model_proposal_geometry";return result;
  }
  for(const auto& point:proposal)if(!point.allFinite()) {
    result.reason="failed_model_proposal_geometry";return result;
  }
  for(std::size_t i=1;i<proposal.size();++i) {
    if(!collisionFreeSegment(proposal[i-1],proposal[i],map.getResolution()*.5,
                             checked_occupancy)) {
      result.reason=budget->allowed()?"failed_model_proposal_collision":budget->reason();
      return result;
    }
  }
  if(!budget->allowed()){result.reason=budget->reason();return result;}
  output=std::move(proposal);
  output_reason="reference_model_checked_initializer";
  result.reason=output_reason;
  result.accepted=true;
  return result;
}

} // namespace scan_planner
