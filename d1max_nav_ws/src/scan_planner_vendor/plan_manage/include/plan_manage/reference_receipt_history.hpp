#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace scan_planner {

// Immutable reference admission facts, not renewable motion/collision leases.
// Re-delivery must not clear the pending solve or issue a freshly stamped ACK.
// Keep the FSM's current reference and the validator's committed version
// pinned; an evicted old generation is ambiguous and must not be called a
// never-admitted negative by the ingress callback.
template<class Proposal, class Receipt>
class ReferenceReceiptHistory {
public:
  using Version=decltype(std::declval<Proposal>().version);
  enum class Match {Missing, Exact, PayloadConflict};
  struct Lookup {Match match;const Receipt* receipt;};
  Lookup lookup(const Proposal& proposal) const {
    for(const auto& item:items_) {
      if(item.first==proposal)return {Match::Exact,&item.second};
      if(item.first.proposal_id==proposal.proposal_id||item.first.version==proposal.version)
        return {Match::PayloadConflict,nullptr};
    }
    return {Match::Missing,nullptr};
  }
  bool remember(const Proposal& proposal,const Receipt& receipt,
      const std::optional<Version>& current,const std::optional<Version>& committed) {
    if(lookup(proposal).match!=Match::Missing)return false;
    if(items_.size()==capacity) {
      const auto unpinned=std::find_if(items_.begin(),items_.end(),[&](const auto& item){
        return (!current||item.first.version!=*current)&&
          (!committed||item.first.version!=*committed);
      });
      if(unpinned==items_.end())return false;
      items_.erase(unpinned);
    }
    items_.emplace_back(proposal,receipt);return true;
  }
  std::size_t size()const{return items_.size();}
  static constexpr std::size_t capacity=8;
private:
  std::vector<std::pair<Proposal,Receipt>> items_;
};

} // namespace scan_planner
