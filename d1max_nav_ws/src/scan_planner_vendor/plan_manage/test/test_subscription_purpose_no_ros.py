#!/usr/bin/env python3
"""Compile the production purpose-gated inbox registration without ROS.

Only registration is exercised: a passive factory records topic names and never
invokes callbacks, initializes ROS, publishes messages, or connects a robot.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/subscription_purpose.hpp>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <chrono>
#include <vector>
namespace d1max_planning_interfaces::msg {
struct PassiveHeader {struct {std::int64_t sec=0;std::uint32_t nanosec=0;} stamp;};
#define MESSAGE(T) struct T {using ConstSharedPtr=std::shared_ptr<const T>;PassiveHeader header;};
MESSAGE(ExecutionHandoffGrant) MESSAGE(PreparedMotionDemand) MESSAGE(ExecutionCommitAck)
MESSAGE(TrackingProgress) MESSAGE(MotionDemand) MESSAGE(MotionValidation) MESSAGE(TrajectoryAdmission)
MESSAGE(LocalTrajectoryCommit) MESSAGE(TaskExecutionLease)
#undef MESSAGE
}
namespace std_msgs::msg {struct Bool {using ConstSharedPtr=std::shared_ptr<const Bool>;};}
namespace rclcpp {
struct QoS {
  int depth;bool reliable_value=false,durable_value=false;
  explicit QoS(int d):depth(d){} QoS& best_effort(){return *this;}
  QoS& reliable(){reliable_value=true;return *this;}
  QoS& transient_local(){durable_value=true;return *this;}
};
struct SensorDataQoS:QoS {
  SensorDataQoS():QoS(5){} SensorDataQoS& keep_last(int d){depth=d;return *this;}
};
}
struct PassiveFactory {
  std::vector<std::int64_t> timer_ms;
  std::map<std::string,int> topics;
  std::map<std::string,std::tuple<int,bool,bool>> qualities;
  struct Publisher {template<class T>void publish(const T&) {}};
  template<class Message,class QoS>std::shared_ptr<Publisher> create_publisher(const std::string&,QoS) {
    return std::make_shared<Publisher>();
  }
  static std::tuple<int,bool,bool> quality(const rclcpp::QoS& q){return {q.depth,q.reliable_value,q.durable_value};}
  static std::tuple<int,bool,bool> quality(int depth){return {depth,false,false};}
  template<class Duration,class Callback> std::shared_ptr<int> create_wall_timer(Duration d,Callback) {
    timer_ms.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(d).count());
    return std::make_shared<int>(1); // Registration only; never invoke a callback.
  }
  template<class Message,class QoS,class Callback,class... Options>
  std::shared_ptr<int> create_subscription(const std::string& topic,QoS qos,Callback,Options...) {
    ++topics[topic];qualities[topic]=quality(qos);return std::make_shared<int>(1);
  }
};
namespace scan_planner {
namespace ew=d1max_planning_interfaces::msg;
template<class Stamp> std::uint64_t ownerSourceOrder(const Stamp& stamp) {
  return static_cast<std::uint64_t>(stamp.sec)*1000000000ULL+stamp.nanosec;
}
struct PassiveValidator {
  template<class T> void handoff(const T&){}
  template<class T> void preparedDemand(const T&){}
  template<class T> void commitAck(const T&){}
  template<class T> void localCommit(const T&){}
  template<class T> T localCommitExchange(const T& value){return value;}
  template<class T> void taskLease(const T&){}
  template<class T> void progress(const T&){}
  template<class T> void demand(const T&){}
  template<class T> void submitBlockedEntry(const T&){}
  template<class T> void submitCandidateEntryRejection(const T&){}
};
struct SCANReplanFSM {
  enum class OwnerInput {Odom,Progress,Proposal,TypedPath,Path,Goal,Count};
  void dispatchOwnerInput(OwnerInput,std::uint64_t,std::function<void()> callback){callback();}
  bool planning_only_=false,local_follower_=false;
  PassiveFactory factory;PassiveFactory* node_=&factory;
  std::shared_ptr<PassiveValidator> execution_validator_=std::make_shared<PassiveValidator>();
  std::shared_ptr<int> execution_handoff_sub_,execution_prepared_sub_,execution_commit_ack_sub_;
  std::shared_ptr<int> execution_local_commit_sub_,execution_task_lease_sub_;
  std::shared_ptr<PassiveFactory::Publisher> execution_local_commit_result_pub_;
  std::shared_ptr<int> execution_progress_sub_,execution_demand_sub_,execution_blocked_entry_sub_;
  std::shared_ptr<int> execution_admission_sub_,tracking_progress_sub_,go2_execution_frozen_sub_;
  std::shared_ptr<int> exec_timer_,safety_timer_;
  void execFSMCallback(){}void checkCollisionCallback(){}
  void trackingProgressCallback(ew::TrackingProgress::ConstSharedPtr){}
  void go2ExecutionFrozenCallback(std_msgs::msg::Bool::ConstSharedPtr){}
  rclcpp::QoS bodySourceQos() {return local_follower_?__TYPED_BODY_QOS__:__BODY_QOS__;}
  void bindPurposeInboxes() {
    const int control_options=1;
__TIMERS__
__BLOCKS__
  }
};
}
int main() {
  const std::map<std::string,int> execution={
    {"/d1max/live_planning/execution/handoff_grant",1},
    {"/d1max/live_planning/execution/prepared_demand",1},
    {"/d1max/live_planning/execution/commit_ack",1},
    {"/d1max/live_planning/execution/demand",1},
    {"/d1max/live_planning/execution/blocked_entry",1},
    {"/d1max/live_planning/execution/admission",1},
    {"planning/tracking_progress",2}};
  scan_planner::SCANReplanFSM preview;preview.planning_only_=true;preview.bindPurposeInboxes();
  if(preview.factory.timer_ms!=std::vector<std::int64_t>{10,50})return 10;
  std::cout<<"PASS production_FSM_100Hz_and_collision_20Hz_are_independent_registrations_not_solver_frequency\n";
  if(!preview.factory.topics.empty())return 1;
  std::cout<<"PASS planning_only_creates_no_controller_writer_tracker_or_frozen_inboxes\n";
  scan_planner::SCANReplanFSM actual;actual.bindPurposeInboxes();
  if(actual.factory.topics!=execution)return 2;
  std::cout<<"PASS formal_execution_uses_typed_writer_and_tracker_without_legacy_freeze_topic\n";
  auto local_execution=execution;
  local_execution.erase("/d1max/live_planning/execution/handoff_grant");
  local_execution.erase("/d1max/live_planning/execution/prepared_demand");
  local_execution.erase("/d1max/live_planning/execution/commit_ack");
  local_execution.erase("/d1max/live_planning/execution/blocked_entry");
  local_execution["/d1max/live_planning/execution/local_commit"]=1;
  local_execution["/d1max/live_planning/execution/task_lease"]=1;
  scan_planner::SCANReplanFSM local;local.local_follower_=true;local.bindPurposeInboxes();
  if(local.factory.topics!=local_execution)return 5;
  if(local.factory.qualities.at("/d1max/live_planning/execution/local_commit")!=std::make_tuple(8,true,true))return 6;
  if(local.factory.qualities.at("/d1max/live_planning/execution/task_lease")!=std::make_tuple(2,true,false))return 7;
  std::cout<<"PASS local_follower_consumes_durable_geometry_fact_and_task_lease_without_curve_writer_or_braking_volume_inboxes\n";
  if(PassiveFactory::quality(local.bodySourceQos())!=std::make_tuple(1,true,false))return 8;
  if(PassiveFactory::quality(actual.bodySourceQos())!=std::make_tuple(1,false,false))return 9;
  std::cout<<"PASS local_follower_exact_body_evidence_is_reliable_bounded_and_legacy_sensor_qos_is_unchanged\n";
  auto compatibility=execution;compatibility["planning/go2_execution_frozen"]=1;
  scan_planner::SCANReplanFSM legacy;legacy.execution_validator_.reset();legacy.bindPurposeInboxes();
  if(legacy.factory.topics!=compatibility)return 4;
  std::cout<<"PASS compatibility_execution_retains_legacy_freeze_topic\n";
  if(scan_planner::nativeMotionSubscriptionsEnabled(true)||
     !scan_planner::nativeMotionSubscriptionsEnabled(false))return 3;
  std::cout<<"PASS explicit_purpose_gate_excludes_preview_and_allows_execution_inboxes\n";
}
'''


def purpose_blocks(source):
    marker = 'if (nativeMotionSubscriptionsEnabled(planning_only_)) {'
    result = []
    cursor = 0
    while (start := source.find(marker, cursor)) >= 0:
        brace = source.index('{', start)
        depth = 1
        cursor = brace + 1
        while depth:
            if cursor >= len(source):
                raise AssertionError('unterminated native subscription purpose block')
            depth += (source[cursor] == '{') - (source[cursor] == '}')
            cursor += 1
        result.append((start, cursor, source[start:cursor]))
    return result


def braced_block(source, start):
    """Restrict a contract assertion to its actual C++ callback/method body."""
    brace = source.index('{', start)
    depth = 1
    cursor = brace + 1
    while depth:
        if cursor >= len(source):
            raise AssertionError('unterminated C++ contract block')
        depth += (source[cursor] == '{') - (source[cursor] == '}')
        cursor += 1
    return source[brace:cursor]


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'src/scan_replan_fsm.cpp')
    parser.add_argument('--include', type=Path, default=root/'include')
    args = parser.parse_args()
    source = args.source.read_text()
    timer_start=source.index('    exec_timer_ = node_->create_wall_timer(')
    timer_end=source.index('    odom_sub_ = ',timer_start)
    timers=source[timer_start:timer_end]
    blocks = purpose_blocks(source)
    assert len(blocks) == 3, 'all three controller/writer/tracker-purpose blocks required'
    guarded = ''.join(block[2] for block in blocks)
    forbidden = ('execution_handoff_sub_', 'execution_prepared_sub_', 'execution_commit_ack_sub_',
                 'execution_local_commit_sub_', 'execution_task_lease_sub_',
                 'execution_demand_sub_', 'execution_blocked_entry_sub_', 'execution_admission_sub_',
                 'execution_progress_sub_', 'tracking_progress_sub_', 'go2_execution_frozen_sub_')
    for member in forbidden:
        assignment = re.compile(r'\b'+member+r'\s*=\s*node_->create_subscription<')
        assert len(assignment.findall(source)) == len(assignment.findall(guarded)) == 1, member
    for member in ('execution_permit_sub_', 'execution_support_sub_', 'execution_body_sub_',
                   'execution_proposal_sub_', 'odom_sub_'):
        assignment = re.compile(r'\b'+member+r'\s*=\s*node_->create_subscription<')
        assert len(assignment.findall(source)) == 1 and not assignment.search(guarded), member
    assert 'execution_control_group_=node_->create_callback_group' in source
    proposal_start = source.index('execution_proposal_sub_=')
    callback_start = source.index('[this,mode](ew::ReferenceProposal::ConstSharedPtr m)', proposal_start)
    proposal = braced_block(source, callback_start)
    if 'receiveReferenceProposal(m,mode)' in proposal:
        proposal = braced_block(source, source.index('void SCANReplanFSM::receiveReferenceProposal('))
    # Ingress now checks AND replaces under the validator's one lock rather
    # than acquiring an identity outside it and racing a later writer commit.
    assert proposal.count('replacePendingReference(*m,rolling,') == 1
    assert 'const bool rolling=local_follower_&&have_target_&&' in proposal
    assert 'rollingReferenceExtension(reference_metadata_,m->reference,fixed_stair_reference_continuity_)' in proposal
    assert 'rolling&&fixed_stair_reference_continuity_&&reference_metadata_.segment_id!=m->reference.segment_id' in proposal
    assert 'std::optional<ew::ExecutionVersion>(executionVersion(reference_metadata_))' in proposal
    # A post-admission immutable ACK cache may read the committed identity
    # ONLY to pin its bounded history. It cannot qualify or replace a pending
    # reference from that out-of-lock observation.
    admission = proposal[:proposal.index('const auto committed=execution_validator_->committedIdentity()')]
    assert all(getter not in admission for getter in
               ('committedIdentity()', 'committedVersion()', 'committedTrajectory()'))
    assert proposal.count('committedIdentity()') == 1
    assert 'execution_receipt_history_.remember(*m,ack,' in proposal
    assert 'std::optional<ew::ExecutionVersion>(committed->version):std::nullopt' in proposal
    validator = (args.include/'plan_manage/execution_validator.hpp').read_text()
    assert 'measuredConnectionSupported(' not in validator, 'do not restore a second raw-floor connector classifier'
    assert 'wholeCurveCollisionFree(' in validator and 'getPlanningInflateOccupancy(' in validator
    assert 'job.support->frame_id==frame_' in validator
    support_contract = braced_block(validator, validator.index('static bool usablePlanningGeometry(const Job& job)'))
    assert '!job.support||job.support->version!=job.version' in support_contract
    assert 'return s.verified;' in support_contract, 'literal-only contract must remain independent'
    assert 'geometry->source.version==job.version' in support_contract
    assert 'geometry->source.support_hash==s.support_hash' in support_contract
    assert 'geometry->source.frame_id==s.frame_id' in support_contract
    assert 'geometry->source.stair_model_hash==s.stair_model_hash' in support_contract
    body_qos = re.search(r'execution_body_sub_\s*=\s*node_->create_subscription<[^>]+>\(\s*'
                        r'"body_pose",\s*([^\n]+),\s*\n', source)
    assert body_qos is not None, 'production independent body evidence registration required'
    typed_body = source[source.index('execution_local_body_sub_='):source.index('} else execution_body_sub_=')]
    typed_body_qos = re.search(r'"/d1max/localization/navigation/local_state"\),([^\n]+),\s*\n',typed_body)
    assert typed_body_qos is not None, 'formal Native must consume the direct provider source'
    assert 'execution_validator_->localBody(*state)' in typed_body and 'control_options' in typed_body
    assert 'configureLocalBodyInput(navigation_session_id_' in source
    assert 'if(execution_validator_&&!local_follower_) execution_validator_->body(' in source
    replacement = braced_block(validator, validator.index(
        'bool replacePendingReference(const ew::ReferenceProposal& proposal,'))
    assert 'bool rolling_reference=false' in validator
    lock = 'std::lock_guard<std::mutex> l(mutex_);'
    reset = 'clearCandidatesLocked();'
    assert replacement.count(lock) == replacement.count(reset) == 1
    for gate in ('referenceTaskOpenLocked(proposal.version)',
                 'active_->version!=proposal.expected_version',
                 '!sameExecutionTask(active_->version,proposal.version)',
                 'current!=proposal.expected_trajectory_id',
                 'proposal.version!=rolling_version',
                 'proposal.expected_trajectory_id>0',
                 'proposal.expected_trajectory_id<current',
                 '!proposal.expected_version.task_id.empty()'):
        assert replacement.index(lock) < replacement.index(gate) < replacement.index(reset)
    clearing = braced_block(validator, validator.index('void clearCandidatesLocked()'))
    for retired in ('candidate_.reset();', 'queued_candidate_.reset();',
                    'selected_candidate_publication_.reset();', 'candidate_offer_deadline_ns_=0;'):
        # candidate_ is also a suffix of queued_candidate_; match the actual
        # identifier, not two substring occurrences of the same reset.
        assert len(re.findall(r'\b'+re.escape(retired), clearing)) == 1, retired
    print('PASS reference_input_rechecks_locked_identity_and_allows_only_classified_same_reference_curve_advance')
    print('PASS permit_support_body_reference_and_independent_control_group_remain_geometry_inputs')
    with tempfile.TemporaryDirectory(prefix='scan-subscription-purpose-') as directory:
        cpp = Path(directory)/'purpose.cpp'
        executable = Path(directory)/'purpose'
        cpp.write_text(HARNESS.replace('__BLOCKS__', guarded).replace('__BODY_QOS__', body_qos.group(1))
                       .replace('__TYPED_BODY_QOS__',typed_body_qos.group(1)).replace('__TIMERS__',timers))
        subprocess.run(['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
                        '-I'+str(args.include), str(cpp), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=False)
        print(result.stdout, end='')
        print(result.stderr, end='')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
