#!/usr/bin/env python3
"""Run the production preview-proof callback with passive lifecycle doubles.

No ROS graph, robot connection, optimizer or substitute collision model. The
full-curve checker is an explicit boolean test seam: this validates when its
result may be published, not geometry feasibility (covered by native tests).
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <cstdint>
#include <memory>
#include <optional>
#include <functional>
#include <iostream>
struct ClockValue {
  std::int64_t ns=500000000000LL;
  double seconds() const {return ns*1e-9;}
  std::int64_t nanoseconds() const {return ns;}
  ClockValue operator-(ClockValue b) const {return {ns-b.ns};}
};
struct Node {ClockValue time; ClockValue now() const {return time;}};
struct Debug {std::uint64_t plan_id=17;};
struct Map {
  std::int64_t source=499800000000LL;
  std::uint64_t context=2, revision=100;
  bool integratedCloudFreshAt(std::int64_t now) {return source>0 && now-source>=0 && now-source<=750000000LL;}
  std::int64_t latestCloudStampNs() {return source;}
  std::uint64_t localizationContextSequence() {return context;}
  std::uint64_t occupancyRevision() {return revision;}
};
struct Manager {
  bool preview=true; int result=0,checks=0;
  struct {int traj_id_=17;} local_data_;
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  std::function<void()> on_check;
  bool hasPreviewHeadingContract() {return preview;}
  int recheckPreviewTrajectory() {++checks; if(on_check)on_check(); return result;}
};
struct Publisher {int count=0; void publish(const Debug &) {++count;}};
namespace scan_planner {
struct CurveCheckEvidence {static constexpr int Clear=0,Occupied=1,Uncertified=2;};
bool measuredBodyYaw(bool fresh,const std::string &,double,double,double &) {return fresh;}
class SCANPlannerManager {
public:
  Node clock;Node *node_=&clock;
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
  struct Candidate {int curve,heading;ClockValue solved_at;std::uint64_t context_sequence;};
  std::optional<Candidate> pending_preview_candidate_=Candidate{18,2,{499990000000LL},2};
  bool measured_body_pose_=true;
  std::string measured_body_frame_="map";
  double measured_body_maximum_age_=.5;
  int accepted_heading_contract_=1,committed_curve=17,commits=0;
  ClockValue committed_time{499000000000LL};
  void updateTrajInfo(int curve,ClockValue when) {committed_curve=curve;committed_time=when;++commits;}
  bool commitPreviewCandidate();
};
std::optional<Debug> makeRevalidatedLocalPlanDebug(const Debug&d,ClockValue,
  std::int64_t,std::int64_t,std::uint64_t,std::uint64_t context) {
  return context ? std::optional<Debug>(d) : std::nullopt;
}
class SCANReplanFSM {
public:
  enum class PreviewRecheck {Unavailable,Stale,Safe,Unsafe,Uncertified};
  Node clock; Node *node_=&clock;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  std::shared_ptr<Publisher> local_plan_debug_pub_=std::make_shared<Publisher>();
  bool reference_path_guidance_=true,require_tagged_reference_=true,have_target_=true,have_odom_=true;
  std::optional<Debug> accepted_preview_debug_=Debug{};
  std::uint64_t accepted_curve_generation_=7,reference_generation_=7,last_local_debug_generation_=0;
  std::uint64_t accepted_curve_context_sequence_=2;
  std::int64_t accepted_curve_id_=17;
  ClockValue last_odom_time_{499900000000LL};
  double odom_timeout_=.5;
  bool have_local_debug_state_=false;
  std::string last_local_debug_phase_;
  ClockValue localPlanDebugHeader() {return node_->now();}
  PreviewRecheck revalidatePreviewIncumbent();
};
__CALLBACK__
__COMMIT__
}
int main() {
  using FSM=scan_planner::SCANReplanFSM; using R=FSM::PreviewRecheck;
  int failures=0,checks=0;
  auto expect=[&](bool result,const char *name) {
    ++checks; std::cout<<(result ? "PASS " : "FAIL ")<<name<<"\n"; failures+=!result;
  };
  {
    FSM f; expect(f.revalidatePreviewIncumbent()==R::Safe,"fresh_real_check_publishes");
    expect(f.local_plan_debug_pub_->count==1 && f.planner_manager_->checks==1,"one_full_check_one_proof");
    expect(f.accepted_preview_debug_->plan_id==17 && f.accepted_curve_id_==17,"no_new_trajectory_identity");
    expect(f.revalidatePreviewIncumbent()==R::Safe && f.local_plan_debug_pub_->count==2,"same_accepted_curve_can_be_rechecked");
  }
  for(int mode=0;mode<8;++mode) {
    FSM f;
    switch(mode) {
      case 0:f.reference_path_guidance_=false;break;
      case 1:f.require_tagged_reference_=false;break;
      case 2:f.have_target_=false;break;
      case 3:f.planner_manager_->preview=false;break;
      case 4:f.accepted_preview_debug_.reset();break;
      case 5:f.reference_generation_=8;break;
      case 6:f.planner_manager_->local_data_.traj_id_=18;break;
      case 7:f.accepted_preview_debug_->plan_id=18;break;
    }
    expect(f.revalidatePreviewIncumbent()==R::Unavailable && f.planner_manager_->checks==0 &&
      f.local_plan_debug_pub_->count==0,"default_cancel_or_identity_change_never_proves");
  }
  {
    FSM f;f.planner_manager_->result=1;
    expect(f.revalidatePreviewIncumbent()==R::Unsafe && f.local_plan_debug_pub_->count==0,
      "confirmed_collision_never_publishes_proof");
  }
  {
    FSM f;f.planner_manager_->result=2;
    expect(f.revalidatePreviewIncumbent()==R::Uncertified && f.local_plan_debug_pub_->count==0,
      "incomplete_budget_or_join_proof_keeps_original_geometry_only");
    f.planner_manager_->result=0;
    expect(f.revalidatePreviewIncumbent()==R::Safe,"later_complete_recheck_can_certify_after_incomplete_proof");
  }
  {
    FSM f;f.planner_manager_->grid_map_->context=3;
    expect(f.revalidatePreviewIncumbent()==R::Unavailable && !f.accepted_preview_debug_ &&
      f.local_plan_debug_pub_->count==0,"context_changed_before_check_revokes_original_acceptance");
  }
  {
    FSM f;f.last_odom_time_.ns-=1000000000LL;
    expect(f.revalidatePreviewIncumbent()==R::Stale && f.planner_manager_->checks==0,"stale_body_not_checked");
  }
  {
    FSM f;f.planner_manager_->grid_map_->source-=1000000000LL;
    expect(f.revalidatePreviewIncumbent()==R::Stale && f.planner_manager_->checks==0,"stale_map_not_checked");
  }
  for(int mode=0;mode<5;++mode) {
    FSM f;
    f.planner_manager_->on_check=[&]() {
      switch(mode) {
        case 0:f.clock.time.ns+=1000000000LL;break;
        case 1:f.planner_manager_->grid_map_->source+=1;break;
        case 2:f.last_odom_time_.ns+=1;break;
        case 3:f.planner_manager_->grid_map_->context+=1;break;
        case 4:f.planner_manager_->grid_map_->revision+=1;break;
      }
    };
    expect(f.revalidatePreviewIncumbent()==R::Stale && f.local_plan_debug_pub_->count==0,
      "postcheck_age_or_changed_snapshot_rejects_even_clear_geometry");
  }
  for(int mode=0;mode<3;++mode) {
    scan_planner::SCANPlannerManager manager;
    if(mode==0)manager.grid_map_->source=498000000000LL;
    if(mode==1)manager.measured_body_pose_=false;
    if(mode==2)manager.grid_map_->context=3;
    expect(!manager.commitPreviewCandidate() && !manager.pending_preview_candidate_ &&
      manager.commits==0 && manager.committed_curve==17 && manager.accepted_heading_contract_==1 &&
      manager.committed_time.ns==499000000000LL,"late_rejection_keeps_incumbent_curve_heading_time");
  }
  {
    scan_planner::SCANPlannerManager manager;
    expect(manager.commitPreviewCandidate() && !manager.pending_preview_candidate_ &&
      manager.commits==1 && manager.committed_curve==18 && manager.accepted_heading_contract_==2 &&
      manager.committed_time.ns==499990000000LL,"candidate_commits_only_after_fresh_gate_with_original_solve_time");
  }
  {
    scan_planner::SCANPlannerManager manager;manager.pending_preview_candidate_.reset();
    expect(manager.commitPreviewCandidate() && manager.commits==0 && manager.committed_curve==17,
      "default_nontransactional_path_unchanged");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";
  return failures ? 1 : 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    manager = (args.source.parent / 'planner_manager.cpp').read_text()
    start = source.index('  SCANReplanFSM::PreviewRecheck SCANReplanFSM::revalidatePreviewIncumbent()')
    end = source.index('  void SCANReplanFSM::publishAttemptDebug(', start)
    commit_start=manager.index('  bool SCANPlannerManager::commitPreviewCandidate()')
    commit_end=manager.index('  bool SCANPlannerManager::checkDynamicFeasibility(',commit_start)
    with tempfile.TemporaryDirectory(prefix='preview-recheck-') as temporary:
        root = args.build_dir or Path(temporary)
        root.mkdir(parents=True, exist_ok=True)
        cpp, binary = root / 'callback.cpp', root / 'callback'
        cpp.write_text(HARNESS.replace('__CALLBACK__', source[start:end]).replace(
            '__COMMIT__',manager[commit_start:commit_end]))
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', str(cpp), '-o', str(binary)], check=True)
        return subprocess.run([str(binary)], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
