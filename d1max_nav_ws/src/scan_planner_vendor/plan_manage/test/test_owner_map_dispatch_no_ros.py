#!/usr/bin/env python3
"""Compile the exact owner scope/ingress methods, without a ROS context.

The map seam supplies exclusive immutable leases, not substitute collision
decisions. Production GridMap/pool tests separately cover the real map/cache.
The 90/20 ms case is a scheduling counterexample, not a robot performance claim.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def method(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


HARNESS = r'''
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>
namespace scan_planner {
struct GridMap {using Ptr=std::shared_ptr<GridMap>;int value=7;};
struct Pool {
  struct PublicationTiming {};
  GridMap::Ptr view=std::make_shared<GridMap>();bool available=true;unsigned borrows=0;
  GridMap::Ptr borrowOwnerValidation(PublicationTiming* =nullptr){++borrows;return available?view:nullptr;}
};
struct CollisionSnapshotPool {using PublicationTiming=Pool::PublicationTiming;};
struct Manager {GridMap::Ptr grid_map_=std::make_shared<GridMap>();};
class SCANReplanFSM {
public:
  Pool snapshot_pool_;std::unique_ptr<Manager> planner_manager_=std::make_unique<Manager>();
  std::shared_ptr<int> map_writer_group_=std::make_shared<int>(1);unsigned owner_map_depth_=0;
  CollisionSnapshotPool::PublicationTiming owner_snapshot_timing_{};
  enum class OwnerInput:std::size_t {Odom,Progress,Proposal,TypedPath,Path,Goal,Count};
  std::array<std::function<void()>,static_cast<std::size_t>(OwnerInput::Count)> owner_inputs_;
  std::array<std::uint64_t,static_cast<std::size_t>(OwnerInput::Count)> owner_input_orders_{};
  class OwnerMapScope {
    SCANReplanFSM& owner_;GridMap::Ptr previous_;bool entered_=false,borrowed_=false;
  public:
    explicit OwnerMapScope(SCANReplanFSM& owner);~OwnerMapScope();
    explicit operator bool()const{return entered_;}
    OwnerMapScope(const OwnerMapScope&)=delete;OwnerMapScope& operator=(const OwnerMapScope&)=delete;
  };
  void deferOwnerInput(OwnerInput,std::uint64_t,std::function<void()>);
  void dispatchOwnerInput(OwnerInput,std::uint64_t,std::function<void()>);
  void drainOwnerInputs();
  unsigned unavailable_debug=0,executed=0;GridMap::Ptr queried;
  unsigned completion_polls=0;void collectSolveCompletion(){++completion_polls;}
  void publishExecutionLedgerDebug(){++unavailable_debug;}
  void execFSMOwnerCallback(){++executed;queried=planner_manager_->grid_map_;}
  void execFSMCallback();
};
__METHODS__
bool formalPrivateOwnerEnabled(bool local_follower_,bool solve_worker_enabled_,
    bool reference_path_guidance_,bool execution_protocol) {return __FORMAL_CONDITION__;}
}

int main() {
  using F=scan_planner::SCANReplanFSM;using I=F::OwnerInput;int checks=0;
  auto expect=[&](bool yes,const char* label){++checks;if(!yes)throw std::runtime_error(label);};
  {
    // Evaluate the production init condition, not a separately maintained
    // test-only choice of which execution modes own private map readers.
    for(unsigned flags=0;flags<16;++flags)
      expect(scan_planner::formalPrivateOwnerEnabled(flags&1,flags&2,flags&4,flags&8)==(flags==15),
        "formal_follower_worker_reference_protocol_enables_private_owner");
    F f;const auto writer=f.planner_manager_->grid_map_;
    f.execFSMCallback();
    expect(f.executed==1&&f.queried==f.snapshot_pool_.view&&f.queried!=writer,
      "formal_fsm_target_selection_uses_private_snapshot");
    expect(f.planner_manager_->grid_map_==writer&&f.owner_map_depth_==0,
      "formal_fsm_restores_writer_after_query");
    f.snapshot_pool_.available=false;f.execFSMCallback();
    expect(f.executed==1&&f.unavailable_debug==1,
      "missing_private_snapshot_never_falls_back_to_live_writer_query");
  }
  {
    F f;const auto writer=f.planner_manager_->grid_map_;const auto leases=f.snapshot_pool_.view.use_count();
    {F::OwnerMapScope outer(f);expect(bool(outer),"owner_can_borrow_complete_snapshot");
      expect(f.planner_manager_->grid_map_==f.snapshot_pool_.view&&f.owner_map_depth_==1,"manager_uses_private_query_view");
      {F::OwnerMapScope inner(f);expect(bool(inner)&&f.snapshot_pool_.borrows==1&&f.owner_map_depth_==2,
          "nested_reference_callback_reuses_same_view");}
      expect(f.owner_map_depth_==1,"nested_scope_releases_depth_only");}
    expect(f.planner_manager_->grid_map_==writer&&f.owner_map_depth_==0&&
      f.snapshot_pool_.view.use_count()==leases,"scope_restores_pointer_and_releases_slot");
    try {F::OwnerMapScope scope(f);throw std::runtime_error("test_exception");}catch(const std::runtime_error&){}
    expect(f.planner_manager_->grid_map_==writer&&f.owner_map_depth_==0&&
      f.snapshot_pool_.view.use_count()==leases,"exception_cannot_pin_query_slot");
  }
  {
    F f;f.snapshot_pool_.available=false;int used=0;std::vector<int> order;
    f.dispatchOwnerInput(I::Proposal,10,[&]{used=10;order.push_back(2);});
    f.dispatchOwnerInput(I::Proposal,12,[&]{used=12;order.push_back(2);});
    f.dispatchOwnerInput(I::Proposal,11,[&]{used=11;});
    f.dispatchOwnerInput(I::Proposal,12,[&]{used=-12;});
    f.dispatchOwnerInput(I::Odom,100,[&]{order.push_back(1);});
    expect(used==0&&order.empty(),"contention_does_not_consume_input_or_mutate_task");
    std::size_t count=0;for(const auto& pending:f.owner_inputs_)if(pending)++count;
    expect(count==2,"latest_only_fixed_capacity_not_callback_backlog");
    f.snapshot_pool_.available=true;
    {F::OwnerMapScope scope(f);f.drainOwnerInputs();}
    expect(used==12&&order==std::vector<int>({1,2}),"latest_original_input_and_body_before_reference");
    count=0;for(const auto& pending:f.owner_inputs_)if(pending)++count;
    expect(count==0,"completed_inputs_retire_once");
    const std::uint64_t original_source=123456789;
    f.dispatchOwnerInput(I::Odom,original_source,[&]{expect(original_source==123456789,"original_source_not_renewed");});
  }
  {
    F f;f.snapshot_pool_.available=false;std::promise<int> promise;auto ready=promise.get_future();promise.set_value(18);
    {F::OwnerMapScope scope(f);if(scope)ready.get();}
    expect(ready.valid()&&ready.wait_for(std::chrono::seconds(0))==std::future_status::ready,
      "no_view_does_not_consume_ready_result_or_clear_incumbent");
    f.map_writer_group_.reset();bool called=false;
    f.dispatchOwnerInput(I::Proposal,1,[&]{called=true;});
    expect(called&&f.snapshot_pool_.borrows==1,"compatibility_keeps_original_direct_owner");
  }
  {
    F f;const auto writer=f.planner_manager_->grid_map_;std::atomic<bool> writing{false},finished{false};
    std::promise<void> begun;auto entered=begun.get_future();
    std::thread fusion([&]{writing=true;begun.set_value();std::this_thread::sleep_for(std::chrono::milliseconds(90));
      writer->value=99;finished=true;});
    entered.wait();auto solve=std::async(std::launch::async,[]{std::this_thread::sleep_for(std::chrono::milliseconds(20));return 18;});
    const auto started=std::chrono::steady_clock::now();bool harvested=false;
    while(std::chrono::steady_clock::now()-started<std::chrono::milliseconds(200)) {
      F::OwnerMapScope scope(f);
      if(scope&&solve.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        expect(solve.get()==18,"completed_worker_result_preserved");harvested=true;
        expect(writing&&!finished,"short_result_taken_before_long_fusion_finishes");
        expect(f.planner_manager_->grid_map_->value==7,"query_complete_snapshot_not_partial_writer");break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    fusion.join();expect(harvested&&finished&&writer->value==99,"writer_and_owner_both_finish");
    std::cout<<"synthetic_fusion_ms=90 synthetic_solve_ms=20 observed_harvest_ms="<<elapsed<<'\n';
  }
  std::cout<<checks<<" owner scope/dispatch checks passed; no ROS transport or collision substitution\n";
}
'''


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=package/'src/scan_replan_fsm.cpp')
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    text = args.source.read_text()
    exact = '\n'.join(method(text, signature) for signature in (
        '  SCANReplanFSM::OwnerMapScope::OwnerMapScope(',
        '  SCANReplanFSM::OwnerMapScope::~OwnerMapScope()',
        '  void SCANReplanFSM::deferOwnerInput(',
        '  void SCANReplanFSM::dispatchOwnerInput(',
        '  void SCANReplanFSM::drainOwnerInputs()',
        '  void SCANReplanFSM::execFSMCallback()'))
    formal_condition = re.search(
        r'if\(([^\n]+)\)\s*\n\s*map_writer_group_=node_->create_callback_group', text)
    assert formal_condition, 'formal owner private-reader initialization missing'
    fusion = method(text, '      grid_map_writer_->setCollisionUpdateCallback(')
    assert '*planner_manager_->grid_map_' not in fusion
    publication = method(text, '  void SCANReplanFSM::publishMapSnapshots()')
    assert '*grid_map_writer_' in publication
    assert '*planner_manager_->grid_map_' not in publication
    assert 'publishMapSnapshots()' in fusion
    assert '[this]{publishMapSnapshots();},map_writer_group_' in text
    assert 'if(!map_writer_group_' in fusion, 'formal map writer cannot harvest FSM future'
    owner = method(text, '  void SCANReplanFSM::execFSMCallback()')
    assert owner.index('collectSolveCompletion()') < owner.index('OwnerMapScope scope(*this)')
    assert 'OwnerMapScope scope(*this)' in owner and owner.index('drainOwnerInputs()') < owner.index('execFSMOwnerCallback()')
    node = (package/'src/scan_planner_node.cpp').read_text()
    assert 'RoleExecutors executors(node->get_node_base_interface(),planner.executorRoles())' in node
    roles = method(text, '  std::vector<ExecutorRole> SCANReplanFSM::executorRoles()const')
    assert '{"planning_owner",node_->get_node_base_interface()->get_default_callback_group()}' in roles
    assert '{"map_writer",map_writer_group_}' in roles
    assert '{"map_ingress",grid_map_writer_?grid_map_writer_->projectedIngressGroup():nullptr}' in roles
    assert '{"control_inbox",execution_control_group_}' in roles
    assert 'execution_validator_->selectedPublicationGroup()' in roles
    assert 'execution_validator_->committedDisplayGroup()' in roles
    assert 'MultiThreadedExecutor' not in node
    output = args.build_dir or Path(tempfile.mkdtemp(prefix='scan-owner-scope-'))
    output.mkdir(parents=True, exist_ok=True)
    cpp, binary = output/'owner_scope.cpp', output/'owner_scope'
    cpp.write_text(HARNESS.replace('__METHODS__', exact)
                   .replace('__FORMAL_CONDITION__', formal_condition.group(1)))
    subprocess.run(['c++', '-std=c++17', '-O2', '-pthread', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
