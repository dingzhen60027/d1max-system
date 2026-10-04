#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace d1max_navigation_bt {
enum class PreparationStage {Pose,Map,Reference,Trajectory,Ready,Dormant};
inline bool preparationStarted(bool route_ready,bool follow_sent) {
  // InitialLocalization and ComputeRoute have their own BT deadlines. Do not
  // charge their time to the later FollowRoute recovery episode.
  return route_ready&&follow_sent;
}
inline PreparationStage preparationStage(bool inputs_ready,bool route_ready,
    const std::string& follow_phase,std::int64_t evidence_ns) {
  if(!inputs_ready)return PreparationStage::Pose;
  if(!route_ready)return PreparationStage::Dormant;
  if(follow_phase=="waiting_localization")return PreparationStage::Pose;
  if(follow_phase=="waiting_local_map")return PreparationStage::Map;
  if(follow_phase=="waiting_reference")return PreparationStage::Reference;
  return follow_phase=="following"&&evidence_ns>0?PreparationStage::Ready:PreparationStage::Trajectory;
}
struct PreparationLimits {
  std::array<double,4> waits{30.,15.,5.,20.};
  double episode{30.},stable{.6};
  void validate()const {
    for(const auto value:waits)if(!std::isfinite(value)||value<1.||value>120.)
      throw std::invalid_argument("bounded_preparation_waits_required");
    if(!std::isfinite(episode)||episode<1.||episode>120.||!std::isfinite(stable)||stable<.4||stable>3.||stable>=episode)
      throw std::invalid_argument("bounded_preparation_episode_required");
  }
};
// Owned by the same BT task as Compute/Follow. Phase chatter accumulates, not
// renews, each wait and its episode. Action heartbeat receipt is NOT recovery
// evidence: reset needs distinct source times from genuine native body proofs.
class PreparationBudget {
public:
  explicit PreparationBudget(PreparationLimits limits={}):limits_(limits){limits_.validate();}
  std::string observe(PreparationStage stage,double now,std::int64_t evidence_ns=0) {
    if(!failure_.empty())return failure_;
    if(!std::isfinite(now)||(last_&&now<*last_))return failure_="preparation_clock_invalid";
    if(stage==PreparationStage::Ready&&evidence_ns<=0)stage=PreparationStage::Trajectory;
    if(last_&&previous_!=PreparationStage::Dormant) {
      const double dt=now-*last_;
      if(isWait(previous_))waits_[index(previous_)]+=dt;
      if(episode_active_)episode_elapsed_+=dt;
    }
    last_=now;previous_=stage;
    // A newly arrived good sample cannot resurrect a task whose original
    // deadline has already elapsed (including a delayed owner callback).
    static constexpr const char*names[]{"pose","map","reference","trajectory"};
    for(std::size_t i=0;i<waits_.size();++i)if(waits_[i]>=limits_.waits[i])
      return failure_=std::string("follow_")+names[i]+"_timeout";
    if(episode_active_&&episode_elapsed_>=limits_.episode)
      return failure_="follow_preparation_episode_timeout";
    if(isWait(stage)) {episode_active_=true;good_since_.reset();good_samples_=0;}
    else if(stage==PreparationStage::Ready&&episode_active_) {
      if(!good_since_)good_since_=now;
      if(evidence_ns>last_evidence_ns_){++good_samples_;last_evidence_ns_=evidence_ns;}
      if(now-*good_since_>=limits_.stable&&good_samples_>=3) {
        waits_.fill(0.);episode_elapsed_=0.;episode_active_=false;good_since_.reset();good_samples_=0;
      }
    }
    return {};
  }
private:
  static std::size_t index(PreparationStage stage){return static_cast<std::size_t>(stage);}
  static bool isWait(PreparationStage stage){return index(stage)<4;}
  PreparationLimits limits_;std::array<double,4> waits_{};
  std::optional<double>last_,good_since_;PreparationStage previous_{PreparationStage::Dormant};
  double episode_elapsed_{};std::int64_t last_evidence_ns_{};unsigned good_samples_{};bool episode_active_{};
  std::string failure_;
};
} // namespace d1max_navigation_bt
