#include "d1max_navigation_bt/task_execution_authority.hpp"
#include "d1max_navigation_bt/runtime_policy.hpp"
#include "d1max_navigation_bt/task_contract.hpp"
#include "d1max_navigation_bt/sdk_software_estop_fence.hpp"
#include "d1max_navigation_bt/sdk_manual_control_fence.hpp"
#include <gtest/gtest.h>

using namespace d1max_navigation_bt;
namespace {
exec3::Version taskVersion(){exec3::Version v;v.schema_version=3;v.session_id="session";v.task_id="task";
  v.route_id="route";v.route_hash="hash";v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";return v;}
TaskExecutionAuthority::Status status(const TaskExecutionAuthority&a,double now,uint64_t seq=1){
  TaskExecutionAuthority::Status s;s.schema_version=1;s.version=a.grantVersion();s.execution_id=a.executionId();
  s.control_epoch=a.controlEpoch();s.sequence=seq;s.source_stamp=s.body_source_stamp=exec3::stamp(now);
  const auto deadline=int64_t(s.body_source_stamp.sec)*1000000000LL+s.body_source_stamp.nanosec+100000000LL;
  s.body_valid_until.sec=deadline/1000000000LL;s.body_valid_until.nanosec=deadline%1000000000LL;
  s.trajectory_id=1;s.commit_sequence=1;s.ready=true;s.phase="following";s.transport_mode="isolated_mock";return s;}
exec3::Grant::Request request(const TaskExecutionAuthority&a){exec3::Grant::Request r;r.version=a.grantVersion();
  r.execution_id=a.executionId();r.control_epoch=a.controlEpoch();r.confirmation_id=r.request_id="confirmation";
  r.sdk_session="sdk";r.transport_mode="isolated_mock";r.source_stamp=exec3::stamp(10.);r.activate=true;return r;}
exec3::Grant::Response reply(const exec3::Grant::Request&r,bool accepted){exec3::Grant::Response s;s.version=r.version;
  s.execution_id=r.execution_id;s.control_epoch=r.control_epoch;s.request_id=r.request_id;s.sdk_session=r.sdk_session;
  s.transport_mode=r.transport_mode;s.accepted=accepted;s.definitely_not_admitted=!accepted;s.sdk_arm_generation=accepted?2:0;return s;}
exec3::SDKState sdk(const TaskExecutionAuthority&a,double now,uint64_t sequence=1){exec3::SDKState s;
  s.version=a.grantVersion();s.execution_id=a.executionId();s.control_epoch=a.controlEpoch();s.sdk_session="sdk";
  s.sdk_arm_generation=2;s.sequence=sequence;s.source_stamp=exec3::stamp(now);s.grant_ready=s.control_owned=s.general_low_speed_confirmed=true;
  s.transport_mode="isolated_mock";s.phase="ready";return s;}
void arm(TaskExecutionAuthority&a){ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
  auto r=request(a);ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.));ASSERT_TRUE(a.observe(sdk(a,10.),10.));
  ASSERT_TRUE(a.observe(status(a,10.01,2),10.01));}
}
namespace {
exec3::SDKState softwareLatch(double time,uint64_t sequence=1) {
  exec3::SDKState state;state.sdk_session="sdk";state.transport_mode="isolated_mock";
  state.sequence=sequence;state.source_stamp=exec3::stamp(time);
  state.phase="software_estop_latched";state.reason="software_estop_requested";
  state.fault_latched=true;return state;
}
exec3::SDKState softwareReleased(double time,uint64_t sequence=2) {
  auto state=softwareLatch(time,sequence);state.fault_latched=false;state.phase="idle";
  state.reason="software_estop_released_new_grant_required";return state;
}
}

TEST(SDKSoftwareEstopFence,EmptyWriterRevokesPreGrantGoalAndConfirmationWithoutChangingRoute) {
  SDKSoftwareEstopFence fence;
  GoalExecutionIntent intent(true,"goal_execute:1");ConfirmationLedger confirmation;
  const RouteBinding route{"session","task","route","hash"};
  ASSERT_TRUE(intent.prepare(route,true));
  ASSERT_TRUE(confirmation.confirm(2,"click",route,route,true,true,true,false,true).authorized);
  TaskExecutionAuthority authority(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(authority.observe(status(authority,10.),10.));
  const auto event=softwareLatch(10.01);
  EXPECT_TRUE(event.execution_id.empty());EXPECT_EQ(event.sdk_arm_generation,0U);
  // Existing per-task matching alone misses this real operator stop.
  EXPECT_FALSE(authority.observe(event,10.01));
  ASSERT_TRUE(fence.observe(event,"sdk","isolated_mock",10010000000LL));
  ASSERT_TRUE(revokeSoftwareEstopIntent(intent,confirmation,authority.confirmed(),"sdk_software_emergency_stop"));
  authority.stop("sdk_software_emergency_stop",10.01);
  EXPECT_FALSE(intent.prepare(route,true));EXPECT_FALSE(confirmation.authorized());
  EXPECT_FALSE(confirmation.confirm(2,"click",route,route,true,true,true,false,true).authorized);
  EXPECT_FALSE(authority.begin("execution","confirmation",1,10.02,"sdk"));
  EXPECT_EQ(authority.grantVersion().route_id,route.route);
  EXPECT_EQ(authority.grantVersion().route_hash,route.hash);
  EXPECT_FALSE(authority.requiresPhysicalStop());EXPECT_FALSE(authority.stopped());
  ASSERT_FALSE(fence.observe(softwareReleased(10.03),"sdk","isolated_mock",10030000000LL));
  EXPECT_FALSE(fence.blocked());EXPECT_FALSE(intent.consume(route));EXPECT_FALSE(confirmation.authorized());
  // OFF permits an explicitly new task intent, never resurrection of the old.
  const RouteBinding next{"session","next","next-route","next-hash"};
  GoalExecutionIntent new_intent(true,"goal_execute:2");ConfirmationLedger new_confirmation;
  EXPECT_TRUE(new_intent.prepare(next,true));
  EXPECT_TRUE(new_confirmation.confirm(2,"new-click",next,next,true,true,true,false,true).authorized);
}
TEST(TaskExecutionAuthority,StairModeTransactionHoldsWithoutRetiringTheTask) {
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  auto state=sdk(a,10.02,2);state.general_low_speed_confirmed=false;
  state.motion_mode="stair";state.mode_transition_active=true;state.grant_ready=false;
  state.phase="holding";state.reason="waiting_mode_ack";
  ASSERT_TRUE(a.observe(state,10.02));auto hold=a.tick(10020000000LL,true,true);
  EXPECT_FALSE(hold.allowed);EXPECT_FALSE(hold.revoked);EXPECT_EQ(hold.version.route_id,"route");
  EXPECT_EQ(hold.reason,"mode_transition:waiting_mode_ack");
  // Truthful verified STAIR state replaces GENERAL; it does not fake GENERAL.
  for(int i=0;i<9;++i){const auto time=10.1+.1*i;state=sdk(a,time,3+i);
    state.general_low_speed_confirmed=false;state.motion_mode="stair";state.motion_mode_confirmed=true;
    ASSERT_TRUE(a.observe(state,time));hold=a.tick(int64_t(time*1e9),true,true);}
  EXPECT_TRUE(hold.allowed);EXPECT_FALSE(hold.revoked);
  state=sdk(a,11.,20);state.general_low_speed_confirmed=false;state.motion_mode="stair";
  EXPECT_TRUE(a.observe(state,11.));EXPECT_TRUE(a.tick(11000000000LL,true,true).revoked);
}
TEST(SDKSoftwareEstopFence,LateFirstGrantAfterOperatorStopIsOnlyARetirementObligation) {
  SDKSoftwareEstopFence fence;TaskExecutionAuthority authority(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(authority.observe(status(authority,10.),10.));
  ASSERT_TRUE(authority.begin("execution","confirmation",1,10.,"sdk"));
  const auto original_request=request(authority);
  GoalExecutionIntent intent;ConfirmationLedger confirmation;
  ASSERT_TRUE(fence.observe(softwareLatch(10.01),"sdk","isolated_mock",10010000000LL));
  ASSERT_TRUE(revokeSoftwareEstopIntent(intent,confirmation,authority.confirmed(),"sdk_software_emergency_stop"));
  authority.stop("sdk_software_emergency_stop",10.01);
  ASSERT_TRUE(authority.observeGrant(original_request,reply(original_request,true),10.02));
  ASSERT_TRUE(authority.observe(sdk(authority,10.03),10.03));
  ASSERT_TRUE(authority.observe(status(authority,10.03,2),10.03));
  const auto revoked=authority.tick(10030000000LL,true,true);
  EXPECT_FALSE(revoked.allowed);EXPECT_TRUE(revoked.revoked);EXPECT_TRUE(authority.requiresPhysicalStop());
  EXPECT_FALSE(authority.stopped());EXPECT_FALSE(authority.executionSoftwareRetired());
  EXPECT_FALSE(authority.begin("execution","confirmation",1,10.04,"sdk"));
}
TEST(SDKSoftwareEstopFence,PreviewHasNoImplicitExecutionIntentToCancel) {
  GoalExecutionIntent intent;ConfirmationLedger confirmation;SDKSoftwareEstopFence fence;
  ASSERT_TRUE(fence.observe(softwareLatch(10.),"sdk","isolated_mock",10000000000LL));
  EXPECT_TRUE(fence.blocked());
  EXPECT_FALSE(revokeSoftwareEstopIntent(intent,confirmation,false,"sdk_software_emergency_stop"));
  const RouteBinding route{"session","task","route","hash"};
  EXPECT_TRUE(confirmation.confirm(2,"later-click",route,route,true,true,true,false,true).authorized);
  // The navigator separately rejects confirmation while fence.blocked().
}
TEST(SDKSoftwareEstopFence,OnlyAnExplicitFreshSameSDKSoftwareLatchIsAnOperatorEvent) {
  for(int fault=0;fault<11;++fault) {
    SDKSoftwareEstopFence fence;auto state=softwareLatch(10.);
    switch(fault) {
      case 0:state.sdk_session="other-sdk";break;
      case 1:state.transport_mode="live";break;
      case 2:state.source_stamp=exec3::stamp(9.499);break;
      case 3:state.source_stamp=exec3::stamp(10.001);break;
      case 4:state.sequence=0;break;
      case 5:state.source_stamp.nanosec=1000000000U;break;
      case 6:state.phase="holding";state.reason="mc_stale";state.fault_latched=false;break;
      case 7:state.phase="stopping";state.reason="hardware_estop_observed";break;
      case 8:state.fault_latched=false;break;
      case 9:state.grant_ready=true;break;
      case 10:state.reason="owner_revoked";break;
    }
    EXPECT_FALSE(fence.observe(state,"sdk","isolated_mock",10000000000LL))<<fault;
    EXPECT_FALSE(fence.blocked())<<fault;
  }
}
TEST(SDKSoftwareEstopFence,LatchHeartbeatsAreIdempotentAndOldEventsCannotRelockAfterRelease) {
  SDKSoftwareEstopFence fence;
  ASSERT_TRUE(fence.observe(softwareLatch(10.),"sdk","isolated_mock",10000000000LL));
  auto observed=softwareLatch(10.02,2);observed.reason="software_estop_observed";
  EXPECT_FALSE(fence.observe(observed,"sdk","isolated_mock",10020000000LL));
  auto pending=softwareLatch(10.03,3);pending.reason="software_estop_release_waiting_prior_retirement";
  EXPECT_FALSE(fence.observe(pending,"sdk","isolated_mock",10030000000LL));EXPECT_TRUE(fence.blocked());
  EXPECT_FALSE(fence.observe(softwareReleased(10.04,4),"sdk","isolated_mock",10040000000LL));
  EXPECT_FALSE(fence.blocked());
  EXPECT_FALSE(fence.observe(softwareLatch(10.03,5),"sdk","isolated_mock",10050000000LL));
  EXPECT_FALSE(fence.observe(softwareLatch(10.05,4),"sdk","isolated_mock",10050000000LL));
  EXPECT_FALSE(fence.blocked());
  EXPECT_TRUE(fence.observe(softwareLatch(10.06,6),"sdk","isolated_mock",10060000000LL));
}
TEST(SDKSoftwareEstopFence,GenericFaultCannotClearSoftwareBlockOrFakeAnOperatorEvent) {
  SDKSoftwareEstopFence fence;
  ASSERT_TRUE(fence.observe(softwareLatch(10.),"sdk","isolated_mock",10000000000LL));
  auto hardware=softwareReleased(10.01,2);hardware.fault_latched=true;
  hardware.phase="stopping";hardware.reason="hardware_estop_observed";
  EXPECT_FALSE(fence.observe(hardware,"sdk","isolated_mock",10010000000LL));EXPECT_TRUE(fence.blocked());
  auto contradictory=softwareLatch(10.02,3);contradictory.fault_latched=false;
  EXPECT_FALSE(fence.observe(contradictory,"sdk","isolated_mock",10020000000LL));EXPECT_TRUE(fence.blocked());
  EXPECT_FALSE(fence.observe(softwareReleased(10.01,4),"sdk","isolated_mock",10030000000LL));
  EXPECT_TRUE(fence.blocked()); // Source order, not callback receipt, is authoritative.
}
TEST(SDKSoftwareEstopFence,ReleasedOldTaskRetainsStopCauseWithoutPermanentlyBlockingNewTask) {
  SDKSoftwareEstopFence fence;
  ASSERT_TRUE(fence.observe(softwareLatch(10.),"sdk","isolated_mock",10000000000LL));
  auto stopped=softwareLatch(10.1,2);stopped.fault_latched=false;stopped.phase="stopped";
  stopped.execution_id="retired-execution";stopped.control_epoch=1;stopped.sdk_arm_generation=2;
  EXPECT_FALSE(fence.observe(stopped,"sdk","isolated_mock",10100000000LL));
  EXPECT_FALSE(fence.blocked());
}

TEST(SDKManualControlFence,PhasesOnlyDescribeAvailabilityAndSettledManualAllowsNewGoal) {
  SDKManualControlFence fence;
  auto state=softwareReleased(10.,1);state.phase="manual_takeover_pending";
  ASSERT_TRUE(fence.observeState(state,"sdk","isolated_mock",10000000000LL));
  EXPECT_TRUE(fence.engaged());EXPECT_TRUE(fence.transitioning());
  state.sequence=2;state.source_stamp=exec3::stamp(10.01);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10010000000LL));
  state.phase="manual_active";state.sequence=3;state.source_stamp=exec3::stamp(10.02);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10020000000LL));
  EXPECT_TRUE(fence.engaged());EXPECT_FALSE(fence.transitioning());
  // A fresh navigation goal is allowed while the released manual panel stays
  // open. Subsequent manual heartbeats must not keep cancelling that goal.
  state.sequence=4;state.source_stamp=exec3::stamp(10.03);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10030000000LL));
  state.phase="manual_releasing";state.sequence=5;state.source_stamp=exec3::stamp(10.04);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10040000000LL));
  EXPECT_TRUE(fence.transitioning());
  state.phase="idle";state.sequence=6;state.source_stamp=exec3::stamp(10.05);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10050000000LL));
  EXPECT_FALSE(fence.engaged());EXPECT_FALSE(fence.transitioning());
  state.phase="manual_takeover_pending";state.sequence=7;state.source_stamp=exec3::stamp(10.06);
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10060000000LL));
}

TEST(SDKManualControlFence,RetainedEventWorksAfterQuickReleaseAndLateJoinWithoutCancelingNewGoal) {
  SDKManualControlFence fence;GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(11000000000LL,11000000000LL,"new-goal").empty());
  // The whole manual operation finished before this navigator started. The
  // retained original event remains authoritative, not a fresh velocity fact.
  const auto event=fence.observeHandoff(1,"sdk","sdk","1","10000000000",20000000000LL);
  ASSERT_TRUE(event.accepted);EXPECT_FALSE(event.duplicate);
  admission.fenceThrough(event.cutoff);
  EXPECT_TRUE(admission.commit(11000000000LL,11010000000LL,"new-goal").empty());
  EXPECT_TRUE(SDKManualControlFence::precedesHandoff(9900000000LL,9950000000LL,event.cutoff));
  EXPECT_FALSE(SDKManualControlFence::precedesHandoff(11000000000LL,11000000000LL,event.cutoff));
  // Accepted before takeover but a slightly future-skewed goal header is old.
  EXPECT_TRUE(SDKManualControlFence::precedesHandoff(10040000000LL,9950000000LL,event.cutoff));
  const auto duplicate=fence.observeHandoff(1,"sdk","sdk","1","10000000000",21000000000LL);
  EXPECT_TRUE(duplicate.accepted);EXPECT_TRUE(duplicate.duplicate);
}

TEST(SDKManualControlFence,TakeoverFencesQueuedGoalsAndLateGrantCannotResumeOldTask) {
  SDKManualControlFence fence;GoalAdmission admission;
  GoalExecutionIntent intent(true,"old-click");ConfirmationLedger confirmation;
  const RouteBinding route{"session","task","route","hash"};
  ASSERT_TRUE(intent.prepare(route,true));
  ASSERT_TRUE(confirmation.confirm(2,"old-click",route,route,true,true,true,false,true).authorized);
  ASSERT_TRUE(admission.reserve(10000000000LL,10000000000LL,"pending-action").empty());
  TaskExecutionAuthority authority(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(authority.observe(status(authority,10.),10.));
  ASSERT_TRUE(authority.begin("execution","old-click",1,10.,"sdk"));
  auto in_flight=request(authority);in_flight.confirmation_id=in_flight.request_id="old-click";
  const auto event=fence.observeHandoff(1,"sdk","sdk","1","10010000000",10020000000LL);
  ASSERT_TRUE(event.accepted);
  // This is the navigator's same callback-boundary contract, before it
  // acknowledges the SDK handoff or accepts another goal.
  admission.fenceThrough(event.cutoff);intent.revoke("sdk_manual_takeover");
  confirmation.revoke();authority.stop("sdk_manual_takeover",10.01);
  EXPECT_EQ(admission.commit(10000000000LL,10020000000LL,"pending-action"),
    "goal_source_precedes_admission_barrier");
  ASSERT_TRUE(authority.observeGrant(in_flight,reply(in_flight,true),10.02));
  ASSERT_TRUE(authority.observe(sdk(authority,10.03),10.03));
  const auto lease=authority.tick(10030000000LL,true,true);
  EXPECT_TRUE(lease.revoked);EXPECT_FALSE(lease.allowed);
  EXPECT_TRUE(authority.requiresPhysicalStop());
  auto state=softwareReleased(10.04,2);state.phase="manual_active";
  EXPECT_TRUE(fence.observeState(state,"sdk","isolated_mock",10040000000LL));
  EXPECT_FALSE(fence.transitioning());
  EXPECT_FALSE(intent.prepare(route,true));EXPECT_FALSE(confirmation.authorized());
  EXPECT_FALSE(authority.begin("execution","old-click",1,10.04,"sdk"));
  EXPECT_TRUE(admission.reserve(10050000000LL,10050000000LL,"fresh-goal").empty());
  EXPECT_TRUE(admission.commit(10050000000LL,10050000000LL,"fresh-goal").empty());
}

TEST(SDKManualControlFence,ForeignStaleReplayedOrContradictoryStatusCannotChangeAvailability) {
  for(int fault=0;fault<8;++fault) {
    SDKManualControlFence fence;auto state=softwareReleased(10.,1);
    state.phase="manual_takeover_pending";
    switch(fault) {
      case 0:state.sdk_session="other-sdk";break;
      case 1:state.transport_mode="live";break;
      case 2:state.source_stamp=exec3::stamp(9.499);break;
      case 3:state.source_stamp=exec3::stamp(10.001);break;
      case 4:state.sequence=0;break;
      case 5:state.source_stamp.nanosec=1000000000U;break;
      case 6:state.source_stamp.sec=-1;break;
      case 7:state.grant_ready=true;break;
    }
    EXPECT_FALSE(fence.observeState(state,"sdk","isolated_mock",10000000000LL))<<fault;
    EXPECT_FALSE(fence.engaged())<<fault;
  }
  SDKManualControlFence fence;auto state=softwareReleased(10.,1);
  state.phase="manual_active";
  ASSERT_TRUE(fence.observeState(state,"sdk","isolated_mock",10000000000LL));
  auto released=state;released.phase="idle";released.sequence=2;released.source_stamp=exec3::stamp(10.01);
  EXPECT_TRUE(fence.observeState(released,"sdk","isolated_mock",10010000000LL));
  state.sequence=3;
  EXPECT_FALSE(fence.observeState(state,"sdk","isolated_mock",10020000000LL));
  EXPECT_FALSE(fence.engaged());
}

TEST(SDKManualControlFence,HandoffRejectsForeignMalformedConflictingAndOlderEvents) {
  SDKManualControlFence fence;
  EXPECT_FALSE(fence.observeHandoff(1,"foreign","sdk","1","10000000000",10000000000LL).accepted);
  EXPECT_FALSE(fence.observeHandoff(2,"sdk","sdk","1","10000000000",10000000000LL).accepted);
  for(const auto& invalid:{"","0","01","-1","+1","1.0","18446744073709551616"})
    EXPECT_FALSE(fence.observeHandoff(1,"sdk","sdk",invalid,"10000000000",10000000000LL).accepted);
  for(const auto& invalid:{"","0","01","-1","1.0","9223372036854775808","10000000001"})
    EXPECT_FALSE(fence.observeHandoff(1,"sdk","sdk","1",invalid,10000000000LL).accepted);
  ASSERT_TRUE(fence.observeHandoff(1,"sdk","sdk","2","10000000000",10000000000LL).accepted);
  EXPECT_FALSE(fence.observeHandoff(1,"sdk","sdk","1","9900000000",10000000000LL).accepted);
  EXPECT_FALSE(fence.observeHandoff(1,"sdk","sdk","2","10000000001",10000000001LL).accepted);
  EXPECT_FALSE(fence.observeHandoff(1,"sdk","sdk","3","9900000000",10000000001LL).accepted);
  const auto next=fence.observeHandoff(1,"sdk","sdk","3","10000000001",10000000001LL);
  EXPECT_TRUE(next.accepted);EXPECT_FALSE(next.duplicate);
}

TEST(TaskExecutionAuthority,ManualHandbackWaitPreservesNewRouteButFencedTaskCannotRetry) {
  for(const auto&reason:{"manual_handoff_waiting_task_fence","manual_control_busy"}) {
    TaskExecutionAuthority authority(taskVersion(),"isolated_mock",30.);
    ASSERT_TRUE(authority.observe(status(authority,10.),10.));
    ASSERT_TRUE(authority.begin("execution","confirmation",1,10.,"sdk"));
    const auto submitted=request(authority);auto wait=reply(submitted,false);wait.reason=reason;
    ASSERT_TRUE(authority.observeGrant(submitted,wait,10.01));
    EXPECT_TRUE(authority.failureReason().empty());EXPECT_FALSE(authority.stopping());
    EXPECT_EQ(authority.grantVersion().route_hash,"hash");
    ASSERT_TRUE(authority.observe(status(authority,10.02,2),10.02));
    EXPECT_TRUE(authority.begin("execution","confirmation",1,10.02,"sdk"));
  }
  TaskExecutionAuthority old(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(old.observe(status(old,10.),10.));ASSERT_TRUE(old.begin("execution","confirmation",1,10.,"sdk"));
  const auto submitted=request(old);auto reject=reply(submitted,false);reject.reason="manual_takeover_retired_task";
  ASSERT_TRUE(old.observeGrant(submitted,reject,10.01));
  EXPECT_TRUE(old.stopping());EXPECT_FALSE(old.failureReason().empty());
  EXPECT_FALSE(old.begin("execution","confirmation",1,10.02,"sdk"));
}

TEST(TaskExecutionAuthority,PreviewCommitsNeedNoSDKOrDynamicVersion){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  auto s=status(a,10.);s.version.reference_generation=47;s.version.anchor_id="new-anchor";s.version.segment_id="landing";
  ASSERT_TRUE(a.observe(s,10.));EXPECT_TRUE(a.canConfirm(10.));
  const auto p=a.tick(10000000001LL,true,false);EXPECT_FALSE(p.allowed);EXPECT_FALSE(p.revoked);
  EXPECT_EQ(p.control_epoch,0U);EXPECT_EQ(p.version.reference_generation,0U);
  EXPECT_EQ(int64_t(p.valid_until.sec)*1000000000LL+p.valid_until.nanosec-(int64_t(p.source_stamp.sec)*1000000000LL+p.source_stamp.nanosec),350000000LL);
}
TEST(TaskExecutionAuthority,ConditionedMapConfirmationUsesActualLocalReadinessWithoutCreatingSdkAuthority){
  TaskExecutionAuthority authority(taskVersion(),"isolated_mock",30.);
  const auto version=authority.grantVersion();
  const RouteBinding binding{version.session_id,version.task_id,version.route_id,version.route_hash};
  const bool map_geometry_eligible=false;
  ConfirmationLedger before,after;
  EXPECT_FALSE(before.confirm(2,"intent",binding,binding,true,true,
    map_geometry_eligible,false,true).authorized);
  EXPECT_FALSE(after.confirm(2,"intent",binding,binding,true,true,
    confirmationGeometryReady("task_follower_v1",map_geometry_eligible,authority.canConfirm(10.)),
    false,true).authorized);
  ASSERT_TRUE(authority.observe(status(authority,10.),10.));
  ASSERT_TRUE(authority.canConfirm(10.));
  const auto accepted=after.confirm(2,"intent",binding,binding,true,true,
    confirmationGeometryReady("task_follower_v1",map_geometry_eligible,authority.canConfirm(10.)),
    false,true);
  ASSERT_TRUE(accepted.authorized);
  ASSERT_TRUE(authority.begin("execution",accepted.confirmation_id,1,10.,"sdk"));
  EXPECT_FALSE(authority.admitted()); // User intent is not an SDK reply or velocity.
  EXPECT_FALSE(authority.tick(10000000001LL,true,true).allowed);
  EXPECT_FALSE(authority.canConfirm(10.101)); // The original body lease was not renewed.
  authority.stop("cancel",10.02);after.revoke();
  EXPECT_FALSE(after.confirm(2,"intent",binding,binding,true,true,true,false,true).authorized);
  EXPECT_FALSE(map_geometry_eligible);
}
TEST(TaskExecutionAuthority,TaskLeaseSurvivesOrdinaryCurveAndAnchorChanges){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  auto s=status(a,10.03,3);s.version.reference_generation=99;s.version.anchor_id="corrected";s.version.segment_id="next";s.commit_sequence=800;s.trajectory_id=99;
  ASSERT_TRUE(a.observe(s,10.03));const auto lease=a.tick(10030000000LL,true,true);
  EXPECT_TRUE(lease.allowed);EXPECT_EQ(lease.execution_id,"execution");EXPECT_EQ(lease.control_epoch,1U);EXPECT_EQ(lease.version,taskVersion());
}
TEST(TaskExecutionAuthority,TemporaryUnadmittedTelemetryRetryUsesSameUserIdentity){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);ASSERT_TRUE(a.observe(status(a,10.),10.));
  ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));auto r=request(a);auto rejection=reply(r,false);rejection.reason="mc_capture_unavailable";
  ASSERT_TRUE(a.observeGrant(r,rejection,10.));EXPECT_FALSE(a.requiresPhysicalStop());EXPECT_TRUE(a.failureReason().empty());
  ASSERT_TRUE(a.observe(status(a,10.1,2),10.1));ASSERT_TRUE(a.begin("execution","confirmation",1,10.1,"sdk"));
  EXPECT_FALSE(a.begin("execution","confirmation",2,10.1,"sdk"));EXPECT_EQ(a.controlEpoch(),1U);
}
TEST(TaskExecutionAuthority,PriorStopWaitRetainsRouteAndRetriesWithinOriginalBudget){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  GrantAdmissionWait wait;
  ASSERT_TRUE(a.observe(status(a,10.),10.));
  ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
  ASSERT_TRUE(wait.attempted(100.));
  auto r=request(a);auto rejection=reply(r,false);
  rejection.reason="prior_stop_unconfirmed";
  ASSERT_TRUE(a.observeGrant(r,rejection,10.01));
  EXPECT_FALSE(a.stopping());EXPECT_FALSE(a.admitted());EXPECT_FALSE(a.requiresPhysicalStop());
  EXPECT_TRUE(a.failureReason().empty());
  EXPECT_FALSE(a.tick(10010000000LL,true,true).allowed);
  EXPECT_EQ(a.grantVersion(),taskVersion());EXPECT_EQ(a.executionId(),"execution");
  EXPECT_FALSE(wait.retryDue(100.49));
  ASSERT_TRUE(wait.attempted(100.5));
  ASSERT_TRUE(a.observe(status(a,10.5,2),10.5));
  ASSERT_TRUE(a.begin("execution","confirmation",1,10.5,"sdk"));
  auto retry=request(a);retry.source_stamp=exec3::stamp(10.5);
  ASSERT_TRUE(a.observeGrant(retry,reply(retry,true),10.51));
  ASSERT_TRUE(a.observe(sdk(a,10.52),10.52));
  ASSERT_TRUE(a.observe(status(a,10.52,3),10.52));
  EXPECT_TRUE(a.tick(10520000000LL,true,true).allowed);
  EXPECT_EQ(a.grantVersion(),taskVersion());EXPECT_EQ(a.controlEpoch(),1U);
  // Attempts never restart the original wait budget.
  EXPECT_TRUE(wait.expired(130.,30.));
}
TEST(TaskExecutionAuthority,PriorStopReplyCannotUndoCancelOrClaimAmbiguousAdmissionWasSafe){
  for(const bool canceled:{false,true}) {
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
    ASSERT_TRUE(a.observe(status(a,10.),10.));
    ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
    auto r=request(a);auto rejection=reply(r,false);rejection.reason="prior_stop_unconfirmed";
    if(canceled)a.stop("new_goal",10.01);
    else rejection.definitely_not_admitted=false;
    ASSERT_TRUE(a.observeGrant(r,rejection,10.02));
    EXPECT_TRUE(a.stopping());EXPECT_FALSE(a.tick(10020000000LL,true,true).allowed);
    EXPECT_FALSE(a.begin("execution","confirmation",1,10.02,"sdk"));
  }
}
TEST(TaskExecutionAuthority,UnretiredTransactionsAndDisconnectedWriterRemainTerminal){
  for(const auto reason:{"prior_sdk_transactions_unretired","writer_disconnected","robot_mode_unsupported"}){
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);ASSERT_TRUE(a.observe(status(a,10.),10.));
    ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));auto r=request(a);auto rejection=reply(r,false);rejection.reason=reason;
    ASSERT_TRUE(a.observeGrant(r,rejection,10.));EXPECT_TRUE(a.stopping());EXPECT_FALSE(a.failureReason().empty());
  }
}
TEST(TaskExecutionAuthority,StaleTelemetryHoldsWithoutNewTaskOrPermanentLoss){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);auto stale=sdk(a,10.05,2);
  stale.phase="holding";stale.reason="robot_state_stale";stale.general_low_speed_confirmed=stale.grant_ready=false;
  ASSERT_TRUE(a.observe(stale,10.05));EXPECT_FALSE(a.tick(10050000000LL,true,true).allowed);EXPECT_FALSE(a.stopping());
  ASSERT_TRUE(a.observe(sdk(a,10.1,3),10.1));ASSERT_TRUE(a.observe(status(a,10.1,3),10.1));
  EXPECT_TRUE(a.tick(10100000000LL,true,true).allowed);EXPECT_EQ(a.controlEpoch(),1U); // first arming, not a recovery
}
TEST(TaskExecutionAuthority,ForeignIdleAndPreviewCannotEraseAdmittedExecution){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);auto idle=sdk(a,10.05,100);idle.execution_id.clear();idle.control_epoch=0;
  EXPECT_FALSE(a.observe(idle,10.05));auto preview=status(a,10.05,100);preview.execution_id.clear();preview.control_epoch=0;
  EXPECT_FALSE(a.observe(preview,10.05));EXPECT_TRUE(a.tick(10050000000LL,true,true).allowed);
}
TEST(TaskExecutionAuthority,FirstAcceptedTakeoverWaitIsNotAnOwnedControlLoss){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
  auto r=request(a);ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.));
  auto taking=sdk(a,10.01);taking.phase="taking_control";taking.control_owned=taking.grant_ready=false;
  ASSERT_TRUE(a.observe(taking,10.01));EXPECT_FALSE(a.stopping());
  EXPECT_FALSE(a.tick(10010000000LL,true,true).allowed);EXPECT_EQ(a.controlEpoch(),1U);
  taking.sequence=2;taking.source_stamp=exec3::stamp(10.03);ASSERT_TRUE(a.observe(taking,10.03));
  EXPECT_FALSE(a.stopping());ASSERT_TRUE(a.observe(sdk(a,10.05,3),10.05));
  ASSERT_TRUE(a.observe(status(a,10.05,2),10.05));EXPECT_TRUE(a.tick(10050000000LL,true,true).allowed);
  // The same phase is never a way to disguise loss after confirmed ownership.
  taking.sequence=4;taking.source_stamp=exec3::stamp(10.06);ASSERT_TRUE(a.observe(taking,10.06));
  EXPECT_TRUE(a.stopping());EXPECT_FALSE(a.tick(10060000000LL,true,true).allowed);
}
TEST(TaskExecutionAuthority,InitialTakeoverCannotMaskFaultBadModeOrOtherLossPhase){
  for(const auto reason:{"fault","bad_mode","other_phase","grant_ready","explicit_reason"}){
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
    ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
    auto r=request(a);ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.));
    auto s=sdk(a,10.01);s.phase="taking_control";s.control_owned=s.grant_ready=false;
    if(std::string(reason)=="fault")s.fault_latched=true;
    else if(std::string(reason)=="bad_mode")s.general_low_speed_confirmed=false;
    else if(std::string(reason)=="other_phase")s.phase="holding";
    else if(std::string(reason)=="grant_ready")s.grant_ready=true;
    else s.reason="ownership_lost";
    ASSERT_TRUE(a.observe(s,10.01));EXPECT_TRUE(a.stopping());EXPECT_FALSE(a.tick(10010000000LL,true,true).allowed);
  }
}
TEST(TaskExecutionAuthority,FixedConfirmedStairTakeoverPreservesTaskWithoutClaimingGeneralMode){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
  auto r=request(a);ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.));
  auto taking=sdk(a,10.01);taking.phase="taking_control";
  taking.control_owned=taking.grant_ready=taking.general_low_speed_confirmed=false;
  taking.motion_mode="stair";taking.motion_mode_confirmed=true;
  for(int i=0;i<3;++i) {
    const double now=10.01+i*.01;taking.sequence=i+1;taking.source_stamp=exec3::stamp(now);
    ASSERT_TRUE(a.observe(taking,now));
    const auto lease=a.tick(static_cast<int64_t>(now*1e9),true,true);
    EXPECT_FALSE(a.stopping());EXPECT_FALSE(lease.allowed);EXPECT_FALSE(lease.revoked);
    EXPECT_TRUE(a.failureReason().empty());EXPECT_EQ(lease.version.route_id,"route");
    EXPECT_FALSE(taking.general_low_speed_confirmed);
  }
  auto ready=taking;ready.sequence=4;ready.source_stamp=exec3::stamp(10.05);
  ready.phase="ready";ready.control_owned=ready.grant_ready=true;
  ASSERT_TRUE(a.observe(ready,10.05));ASSERT_TRUE(a.observe(status(a,10.05,2),10.05));
  const auto running=a.tick(10050000000LL,true,true);
  EXPECT_TRUE(running.allowed);EXPECT_FALSE(running.revoked);EXPECT_EQ(a.controlEpoch(),1U);
  // Already-owned control cannot masquerade as another first takeover.
  taking.sequence=5;taking.source_stamp=exec3::stamp(10.06);
  ASSERT_TRUE(a.observe(taking,10.06));EXPECT_TRUE(a.stopping());
  EXPECT_TRUE(a.tick(10060000000LL,true,true).revoked);
}
TEST(TaskExecutionAuthority,FixedStairTakeoverStillRejectsUnconfirmedOrUnexpectedMode){
  for(const std::string mode:{"stair","general","unknown"}) {
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
    ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));
    auto r=request(a);ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.));
    auto taking=sdk(a,10.01);taking.phase="taking_control";
    taking.control_owned=taking.grant_ready=taking.general_low_speed_confirmed=false;
    taking.motion_mode=mode;taking.motion_mode_confirmed=mode!="stair";
    ASSERT_TRUE(a.observe(taking,10.01));EXPECT_TRUE(a.stopping());
    EXPECT_FALSE(a.tick(10010000000LL,true,true).allowed);
  }
}
TEST(TaskExecutionAuthority,LatePositiveGrantAfterCancelCreatesOnlyStopObligation){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);ASSERT_TRUE(a.observe(status(a,10.),10.));
  ASSERT_TRUE(a.begin("execution","confirmation",1,10.,"sdk"));auto r=request(a);a.stop("new_goal",10.01);
  ASSERT_TRUE(a.observeGrant(r,reply(r,true),10.02));EXPECT_TRUE(a.requiresPhysicalStop());
  const auto lease=a.tick(10020000000LL,true,true);EXPECT_TRUE(lease.revoked);EXPECT_FALSE(lease.allowed);
  EXPECT_FALSE(a.begin("execution","confirmation",1,10.03,"sdk"));
}
TEST(TaskExecutionAuthority,LocalRetirementBeforeBTStopIsRetainedForReplacement){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(a.observe(status(a,10.),10.));ASSERT_TRUE(a.canConfirm(10.));
  // The task-correlated FollowRoute terminal result can arrive before the
  // next BT tick calls finishTask/retire. Its worker retirement proof remains
  // valid regardless of that callback order.
  a.noteLocalSoftwareRetired();
  EXPECT_FALSE(a.canConfirm(10.));
  EXPECT_FALSE(a.begin("execution","confirmation",1,10.,"sdk"));
  a.stop("follow_terminal",10.01);
  EXPECT_TRUE(a.executionSoftwareRetired());
  EXPECT_FALSE(a.requiresPhysicalStop());
  const auto lease=a.tick(10020000000LL,true,true);
  EXPECT_TRUE(lease.revoked);EXPECT_FALSE(lease.allowed);
  EXPECT_FALSE(a.canConfirm(10.02));
}
TEST(TaskExecutionAuthority,EarlyLocalRetirementDoesNotClearSDKStopObligation){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);
  a.noteLocalSoftwareRetired();
  EXPECT_FALSE(a.tick(10010000000LL,true,true).allowed);
  a.stop("follow_terminal",10.02);
  EXPECT_FALSE(a.executionSoftwareRetired());
  EXPECT_TRUE(a.requiresPhysicalStop());EXPECT_FALSE(a.stopped());
  exec3::Retirement fact;fact.schema_version=1;fact.version=a.grantVersion();fact.execution_id="execution";
  fact.control_epoch=1;fact.sdk_session="sdk";fact.sdk_arm_generation=2;fact.grant_request_id="confirmation";
  fact.sequence=1;fact.source_stamp=exec3::stamp(10.03);fact.transport_mode="isolated_mock";
  fact.nonzero_blocked=fact.software_retired=true;
  ASSERT_TRUE(a.observe(fact,10.03));EXPECT_TRUE(a.executionSoftwareRetired());
  EXPECT_TRUE(a.requiresPhysicalStop());EXPECT_FALSE(a.stopped());
  EXPECT_TRUE(a.tick(10030000000LL,true,true).revoked);
}
TEST(TaskExecutionAuthority,GoalAndSDKSubmissionAloneAreNotMeasuredCompletion){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);auto goal=status(a,10.1,3);goal.at_goal=goal.request_stop=true;
  ASSERT_TRUE(a.observe(goal,10.1));EXPECT_TRUE(a.stopping());EXPECT_FALSE(a.goalStopped());
  exec3::Stop stop;stop.version=a.grantVersion();stop.execution_id="execution";stop.control_epoch=1;stop.sdk_session="sdk";stop.sdk_arm_generation=2;
  stop.stop_request_id="confirmation";stop.sequence=1;stop.source_stamp=exec3::stamp(11.2);stop.nonzero_blocked=stop.stop_submitted=true;
  stop.stationary_samples=3;stop.stationary_duration_sec=1.;stop.mc_raw_stamp_ns=1;stop.mc_clock_epoch="clock";
  stop.time_basis="isolated_simulated_source_clock";stop.transport_mode="isolated_mock";
  EXPECT_FALSE(a.observe(stop,11.2));stop.measured_stop_confirmed=true;ASSERT_TRUE(a.observe(stop,11.2));EXPECT_TRUE(a.goalStopped());
  auto retired=status(a,11.3,4);retired.software_retired=true;retired.ready=false;
  ASSERT_TRUE(a.observe(retired,15.));EXPECT_FALSE(a.executionSoftwareRetired());
  exec3::Retirement fact;fact.schema_version=1;fact.version=a.grantVersion();fact.execution_id="execution";fact.control_epoch=1;fact.sdk_session="sdk";
  fact.sdk_arm_generation=2;fact.grant_request_id="confirmation";fact.sequence=1;fact.source_stamp=exec3::stamp(11.3);fact.transport_mode="isolated_mock";
  fact.nonzero_blocked=fact.software_retired=true;ASSERT_TRUE(a.observe(fact,15.));EXPECT_TRUE(a.executionSoftwareRetired());
}
TEST(TaskExecutionAuthority,RetainedManualStopCompletesOnlyTheMatchedOldTask) {
  auto stopFor=[](const TaskExecutionAuthority&a){
    exec3::Stop s;s.version=a.grantVersion();s.execution_id="execution";s.control_epoch=1;
    s.sdk_session="sdk";s.sdk_arm_generation=2;s.stop_request_id="confirmation";
    s.sequence=1;s.source_stamp=exec3::stamp(11.2);
    s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;
    s.stationary_samples=50;s.stationary_duration_sec=1.;s.mc_raw_stamp_ns=1;s.mc_clock_epoch="clock";
    s.time_basis="isolated_simulated_source_clock";s.transport_mode="isolated_mock";return s;
  };
  TaskExecutionAuthority old(taskVersion(),"isolated_mock",30.);arm(old);
  old.stopForManualTakeover(10.1);const auto terminal=stopFor(old);
  ASSERT_TRUE(old.observe(terminal,20.));EXPECT_TRUE(old.stopped());EXPECT_FALSE(old.goalStopped());
  EXPECT_FALSE(old.observe(terminal,20.1)); // Original sequence cannot replay.
  EXPECT_TRUE(old.tick(20000000000LL,true,true).revoked);
  EXPECT_FALSE(old.begin("execution","confirmation",1,20.,"sdk"));
  TaskExecutionAuthority ordinary(taskVersion(),"isolated_mock",30.);arm(ordinary);
  ordinary.stop("cancel",10.1);EXPECT_TRUE(ordinary.observe(terminal,20.));
  EXPECT_TRUE(ordinary.stopped());EXPECT_TRUE(ordinary.tick(20000000000LL,true,true).revoked);
  auto next_version=taskVersion();next_version.task_id="new-task";
  TaskExecutionAuthority next(next_version,"isolated_mock",30.);arm(next);
  EXPECT_FALSE(next.observe(terminal,20.));EXPECT_FALSE(next.stopped());
  for(int fault=0;fault<8;++fault) {
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);a.stopForManualTakeover(10.1);
    auto s=stopFor(a);
    switch(fault) {
      case 0:s.sdk_session="other";break;
      case 1:s.source_stamp=exec3::stamp(10.);break;
      case 2:s.source_stamp=exec3::stamp(21.);break;
      case 3:s.measured_stop_confirmed=false;break;
      case 4:s.measured_linear_mps=.5;break;
      case 5:s.stationary_duration_sec=.1;break;
      case 6:s.mc_raw_stamp_ns=0;break;
      case 7:s.sdk_arm_generation=3;break;
    }
    EXPECT_FALSE(a.observe(s,20.))<<fault;EXPECT_FALSE(a.stopped())<<fault;
  }
}
TEST(TaskExecutionAuthority,DelayedGoalStopIsTerminalEvidenceNotFreshMotionAuthority) {
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);
  auto goal=status(a,10.1,3);goal.at_goal=goal.request_stop=true;
  ASSERT_TRUE(a.observe(goal,10.1));
  exec3::Stop stop;stop.version=a.grantVersion();stop.execution_id="execution";stop.control_epoch=1;
  stop.sdk_session="sdk";stop.sdk_arm_generation=2;stop.stop_request_id="confirmation";
  stop.sequence=1;stop.source_stamp=exec3::stamp(11.2);
  stop.nonzero_blocked=stop.stop_submitted=stop.measured_stop_confirmed=true;
  stop.stationary_samples=50;stop.stationary_duration_sec=1.;stop.mc_raw_stamp_ns=123;
  stop.mc_clock_epoch="clock";stop.time_basis="isolated_simulated_source_clock";stop.transport_mode="isolated_mock";
  ASSERT_TRUE(a.observe(stop,20.));EXPECT_TRUE(a.goalStopped());
  EXPECT_EQ(stop.source_stamp,exec3::stamp(11.2));EXPECT_EQ(stop.mc_raw_stamp_ns,123U);
  EXPECT_FALSE(a.observe(stop,20.1)); // Retained event is idempotent.
  EXPECT_TRUE(a.tick(20000000000LL,true,true).revoked);
  EXPECT_FALSE(a.begin("execution","confirmation",1,20.,"sdk"));
  auto next_version=taskVersion();next_version.task_id="next-task";
  TaskExecutionAuthority next(next_version,"isolated_mock",30.);arm(next);
  EXPECT_FALSE(next.observe(stop,20.));EXPECT_FALSE(next.goalStopped());
  EXPECT_FALSE(next.tick(20000000000LL,true,true).allowed); // Live SDK is stale.
  TaskExecutionAuthority later_stop(taskVersion(),"isolated_mock",30.);arm(later_stop);
  later_stop.stop("cancel",11.);
  EXPECT_FALSE(later_stop.observe(stop,20.)); // Window began before this stop.
}
TEST(TaskExecutionAuthority,ManualTakeoverDuringExistingDrainKeepsOriginalStopBoundary) {
  TaskExecutionAuthority old(taskVersion(),"isolated_mock",30.);arm(old);
  old.stop("cancel",10.0);
  old.stopForManualTakeover(10.2);
  exec3::Stop stop;stop.version=old.grantVersion();stop.execution_id="execution";stop.control_epoch=1;
  stop.sdk_session="sdk";stop.sdk_arm_generation=2;stop.stop_request_id="confirmation";
  stop.sequence=1;stop.source_stamp=exec3::stamp(11.1);
  stop.nonzero_blocked=stop.stop_submitted=stop.measured_stop_confirmed=true;
  stop.stationary_samples=50;stop.stationary_duration_sec=1.05;
  stop.mc_raw_stamp_ns=1;stop.mc_clock_epoch="clock";
  stop.time_basis="isolated_simulated_source_clock";stop.transport_mode="isolated_mock";
  // SDK stationary interval starts at 10.05, after cancel but before manual
  // takeover. Its original terminal message arrives late from the cache.
  EXPECT_TRUE(old.observe(stop,12.));EXPECT_TRUE(old.stopped());EXPECT_FALSE(old.goalStopped());
  EXPECT_TRUE(old.tick(12000000000LL,true,true).revoked);
  auto next_version=taskVersion();next_version.task_id="next";
  TaskExecutionAuthority next(next_version,"isolated_mock",30.);arm(next);
  EXPECT_FALSE(next.observe(stop,12.));EXPECT_FALSE(next.stopped());
  TaskExecutionAuthority before_cancel(taskVersion(),"isolated_mock",30.);arm(before_cancel);
  before_cancel.stop("cancel",10.0);before_cancel.stopForManualTakeover(10.2);
  stop.stationary_duration_sec=1.2; // Starts 9.9, before the real stop request.
  EXPECT_FALSE(before_cancel.observe(stop,12.));EXPECT_FALSE(before_cancel.stopped());
}
TEST(TaskExecutionAuthority,WheelStopMetricMatchesWriterWithoutDiscardingRaw3D){
  for(int invalid=0;invalid<8;++invalid){
    exec3::StationaryLimits limits;limits.velocity_metric="planar_xy_with_vertical_guard_v1";
    if(invalid==1)limits.velocity_metric="spatial_norm_3d";
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,limits);arm(a);a.stop("cancel",10.1);
    exec3::Stop s;s.version=a.grantVersion();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";
    s.sdk_arm_generation=2;s.stop_request_id="confirmation";s.sequence=1;s.source_stamp=exec3::stamp(11.2);
    s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.stationary_samples=50;s.stationary_duration_sec=1.;
    s.mc_raw_stamp_ns=1;s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";s.transport_mode="isolated_mock";
    s.velocity_metric="planar_xy_with_vertical_guard_v1";s.measured_planar_mps=std::hypot(.028366,.001744);
    s.measured_vertical_mps=.013489;s.measured_linear_mps=std::hypot(s.measured_planar_mps,s.measured_vertical_mps);
    if(invalid==2){s.measured_vertical_mps=.031;s.measured_linear_mps=std::hypot(s.measured_planar_mps,s.measured_vertical_mps);}
    if(invalid==3)s.measured_angular_radps=.051;
    if(invalid==4)s.measured_linear_mps=.001;
    if(invalid==5)s.velocity_metric="guess_ignore_z";
    if(invalid==6)s.stop_submitted=false;
    if(invalid==7)s.mc_raw_stamp_ns=0;
    EXPECT_EQ(a.observe(s,11.2),invalid==0);EXPECT_EQ(a.stopped(),invalid==0);
  }
}
TEST(TaskExecutionAuthority,StatusHeartbeatDoesNotRenewOriginalBodyDeadline){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  auto s=status(a,10.);ASSERT_TRUE(a.observe(s,10.));EXPECT_TRUE(a.canConfirm(10.09));
  EXPECT_FALSE(a.canConfirm(10.11));EXPECT_EQ(a.evidence(10.11),0);
  s.sequence=2;s.source_stamp=exec3::stamp(10.2);
  EXPECT_FALSE(a.observe(s,10.2));EXPECT_FALSE(a.canConfirm(10.2));
}
TEST(TaskExecutionAuthority,SdkRecoveryRequiresDistinctOriginalSdkSourcesAndConfiguredWindow){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",.1,{},.02);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  auto unavailable=sdk(a,10.02,2);unavailable.phase="holding";unavailable.reason="mc_stale";
  unavailable.grant_ready=false;ASSERT_TRUE(a.observe(unavailable,10.02));
  ASSERT_FALSE(a.tick(10020000000LL,false,true).allowed);
  // Local control availability no longer renews or resets an SDK recovery.
  for(uint64_t i=3;i<=5;++i){const double now=10.05+(i-3)*.02;
    ASSERT_TRUE(a.observe(sdk(a,now,i),now));
    const auto lease=a.tick(int64_t(now*1e9),false,true);
    EXPECT_EQ(lease.allowed,i==5);}
  EXPECT_FALSE(a.stopping());EXPECT_TRUE(a.failureReason().empty());
  TaskExecutionAuthority repeated(taskVersion(),"isolated_mock",.1,{},.02);arm(repeated);
  ASSERT_TRUE(repeated.tick(10010000000LL,true,true).allowed);
  unavailable=sdk(repeated,10.02,2);unavailable.phase="holding";unavailable.reason="mc_stale";
  unavailable.grant_ready=false;ASSERT_TRUE(repeated.observe(unavailable,10.02));
  repeated.tick(10020000000LL,true,true);
  for(uint64_t i=3;i<=7;++i){const double now=10.05+(i-3)*.02;
    ASSERT_TRUE(repeated.observe(sdk(repeated,10.05,i),now));
    ASSERT_TRUE(repeated.observe(status(repeated,now,i),now));
    EXPECT_FALSE(repeated.tick(int64_t(now*1e9),true,true).allowed);}
  EXPECT_TRUE(repeated.stopping());
  EXPECT_EQ(repeated.failureReason(),"execution_local_recovery_timeout");
}
TEST(TaskExecutionAuthority,PersistentLocalHoldDoesNotRevokeTaskAuthorityOrStartSdkRecovery){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
  const auto original=a.tick(10010000000LL,true,true);ASSERT_TRUE(original.allowed);
  // The worker and SDK are still reporting. Only local planning is holding;
  // crossing the old 30-second budget must not retire its fixed task/grant.
  for(uint64_t i=1;i<=61;++i){
    const double now=10.+i;
    ASSERT_TRUE(a.observe(sdk(a,now,i+1),now));
    auto held=status(a,now,i+2);held.holding=true;held.phase="waiting_replan";
    ASSERT_TRUE(a.observe(held,now));
    const auto lease=a.tick(int64_t(now*1e9),true,true);
    EXPECT_TRUE(lease.allowed);EXPECT_FALSE(lease.revoked);EXPECT_FALSE(a.stopping());
    EXPECT_TRUE(a.failureReason().empty());EXPECT_EQ(lease.version,original.version);
    EXPECT_EQ(lease.execution_id,original.execution_id);EXPECT_EQ(lease.control_epoch,original.control_epoch);
    EXPECT_EQ(lease.confirmation_id,original.confirmation_id);EXPECT_EQ(lease.sdk_arm_generation,original.sdk_arm_generation);
  }
  struct RecoverySample{double event,body;bool allowed;};
  const RecoverySample samples[]{{72.,72.,true},{72.05,72.,true},{72.09,72.,true},
    {72.3,72.3,true},{72.61,72.61,true}};
  uint64_t local_sequence=64,sdk_sequence=63;
  for(const auto& sample:samples){
    ASSERT_TRUE(a.observe(sdk(a,sample.event,sdk_sequence++),sample.event));
    auto measured=status(a,sample.event,local_sequence++);
    const auto original_body=status(a,sample.body);
    measured.body_source_stamp=original_body.body_source_stamp;
    measured.body_valid_until=original_body.body_valid_until;
    ASSERT_TRUE(a.observe(measured,sample.event));
    const auto lease=a.tick(int64_t(sample.event*1e9),true,true);
    EXPECT_EQ(lease.allowed,sample.allowed);EXPECT_FALSE(lease.revoked);
    EXPECT_EQ(lease.version,original.version);EXPECT_EQ(lease.execution_id,original.execution_id);
    EXPECT_EQ(lease.control_epoch,original.control_epoch);EXPECT_EQ(lease.confirmation_id,original.confirmation_id);
    EXPECT_EQ(lease.sdk_session,original.sdk_session);EXPECT_EQ(lease.sdk_arm_generation,original.sdk_arm_generation);
  }
  EXPECT_TRUE(a.admitted());EXPECT_FALSE(a.stopping());EXPECT_TRUE(a.failureReason().empty());
}
TEST(TaskExecutionAuthority,PersistentLocalPolicyCannotMaskSdkOwnershipFaultOrBadMode){
  for(const auto failure:{"ownership_lost","fault","bad_mode"}){
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
    ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
    auto lost=sdk(a,10.02,2);lost.reason=failure;
    if(std::string(failure)=="ownership_lost")lost.control_owned=false;
    else if(std::string(failure)=="fault")lost.fault_latched=true;
    else lost.general_low_speed_confirmed=false;
    ASSERT_TRUE(a.observe(lost,10.02));EXPECT_TRUE(a.stopping());
    EXPECT_EQ(a.failureReason(),std::string("sdk_authority_lost:")+failure);
    EXPECT_TRUE(a.requiresPhysicalStop());EXPECT_FALSE(a.stopped());
    ASSERT_TRUE(a.observe(sdk(a,10.1,3),10.1));
    ASSERT_TRUE(a.observe(status(a,10.1,3),10.1));
    const auto lease=a.tick(10100000000LL,true,true);
    EXPECT_FALSE(lease.allowed);EXPECT_TRUE(lease.revoked);EXPECT_FALSE(a.canConfirm(10.1));
    EXPECT_FALSE(a.begin("execution","confirmation",1,10.1,"sdk"));
  }
}
TEST(TaskExecutionAuthority,PersistentLocalPolicyCannotResumeCancelledOrRetiredWorker){
  for(bool local_retirement:{false,true}){
    TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
    const auto original=a.tick(10010000000LL,true,true);ASSERT_TRUE(original.allowed);
    EXPECT_FALSE(a.tick(70100000000LL,false,true).allowed);EXPECT_FALSE(a.stopping());
    if(local_retirement)a.noteLocalSoftwareRetired();
    a.stop(local_retirement?"follow_terminal":"operator_cancel",70.1);
    for(uint64_t i=0;i<3;++i){
      const double now=70.2+i*.4;
      ASSERT_TRUE(a.observe(sdk(a,now,i+2),now));
      ASSERT_TRUE(a.observe(status(a,now,i+3),now));
      const auto lease=a.tick(int64_t(now*1e9),true,true);
      EXPECT_FALSE(lease.allowed);EXPECT_TRUE(lease.revoked);EXPECT_TRUE(a.stopping());
      EXPECT_EQ(lease.version,original.version);EXPECT_EQ(lease.execution_id,original.execution_id);
      EXPECT_EQ(lease.control_epoch,original.control_epoch);EXPECT_FALSE(a.canConfirm(now));
      EXPECT_FALSE(a.begin("execution","confirmation",1,now,"sdk"));
    }
    EXPECT_TRUE(a.requiresPhysicalStop());EXPECT_FALSE(a.stopped());EXPECT_FALSE(a.goalStopped());
    EXPECT_FALSE(a.executionSoftwareRetired()); // Fresh status is not an SDK retirement/stop ACK.
  }
}
TEST(TaskExecutionAuthority,FirstAdmissionStillRequiresReadyLocalGeometryWithinOriginalBudget){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);arm(a);
  // A grant is admitted, but no initially ready task lease has been issued.
  const double times[]{10.02,40.01,40.03};uint64_t sequence=2;
  for(const double now:times){
    ASSERT_TRUE(a.observe(sdk(a,now,sequence),now));
    auto held=status(a,now,sequence+1);held.holding=true;
    ASSERT_TRUE(a.observe(held,now));
    const auto lease=a.tick(int64_t(now*1e9),true,true);
    EXPECT_FALSE(lease.allowed);
    if(now<40.02){EXPECT_FALSE(a.stopping());EXPECT_FALSE(lease.revoked);}
    else{EXPECT_TRUE(a.stopping());EXPECT_TRUE(lease.revoked);}
    ++sequence;
  }
  EXPECT_EQ(a.failureReason(),"execution_local_recovery_timeout");
}

TEST(TaskExecutionAuthority,TenHzTaskLeaseDoesNotResampleTwentyHzBodyConsumptionDeadline){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
  const auto first=a.tick(10010000000LL,true,true);ASSERT_TRUE(first.allowed);
  for(uint64_t i=1;i<=30;++i){
    const double now=10.1+i*.1;
    // Tracker publishes a valid 20 Hz fact; another topic/timer delivers it
    // 71 ms later, after the unchanged body lease (100 ms from body source).
    auto local=status(a,now-.03,i+2);
    const auto original_body=status(a,now-.045);
    local.body_source_stamp=original_body.body_source_stamp;
    local.body_valid_until=original_body.body_valid_until;
    if(i%3==0){local.ready=false;local.holding=true;local.reason="waiting_fresh_local_map";}
    if(i%4==0){local.version.reference_generation=i;local.version.anchor_id="new-candidate";
      local.trajectory_id=i;local.commit_sequence=i;}
    ASSERT_TRUE(a.observe(local,now+.071));
    ASSERT_TRUE(a.observe(sdk(a,now,i+1),now+.071));
    EXPECT_FALSE(a.canConfirm(now+.071)); // No restamp, no renewed body lease.
    EXPECT_EQ(a.evidence(now+.071),0);
    const auto lease=a.tick(int64_t((now+.08)*1e9),false,true);
    EXPECT_TRUE(lease.allowed);EXPECT_FALSE(lease.revoked);EXPECT_EQ(lease.phase,"tracking");
    EXPECT_EQ(lease.version,first.version);EXPECT_EQ(lease.execution_id,first.execution_id);
    EXPECT_EQ(lease.control_epoch,first.control_epoch);EXPECT_EQ(lease.confirmation_id,first.confirmation_id);
    EXPECT_TRUE(a.failureReason().empty());
  }
}

TEST(TaskExecutionAuthority,LateReadyFactCannotBootstrapOrExtendItsOriginalBodyLease){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.);
  const auto original=status(a,10.);
  ASSERT_TRUE(a.observe(original,10.11)); // Genuine coarse fact, not permission.
  EXPECT_FALSE(a.canConfirm(10.11));EXPECT_EQ(a.evidence(10.11),0);
  EXPECT_FALSE(a.begin("execution","confirmation",1,10.11,"sdk"));
  EXPECT_FALSE(a.tick(10110000000LL,true,true).allowed);
  auto malformed=status(a,10.2,2);malformed.body_valid_until=exec3::stamp(10.4);
  EXPECT_FALSE(a.observe(malformed,10.2)); // No extension beyond original 100 ms.
}

TEST(TaskExecutionAuthority,EstablishedTaskNeverRevivesAfterContextStopOrUnconfirmedSdkGrant){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  a.stop("localization_context_replaced",10.02);
  ASSERT_TRUE(a.observe(sdk(a,10.1,2),10.1));ASSERT_TRUE(a.observe(status(a,10.1,3),10.1));
  const auto stopped=a.tick(10100000000LL,true,true);
  EXPECT_FALSE(stopped.allowed);EXPECT_TRUE(stopped.revoked);
  EXPECT_FALSE(a.begin("execution","confirmation",1,10.1,"sdk"));
  EXPECT_EQ(stopped.version.route_hash,taskVersion().route_hash);
  TaskExecutionAuthority preview(taskVersion(),"isolated_mock",30.);
  ASSERT_TRUE(preview.observe(status(preview,10.),10.));
  EXPECT_FALSE(preview.tick(10000000000LL,true,true).allowed); // No SDK admission.
}

TEST(TaskExecutionAuthority,SdkRecoveryDeadlineWinsOverThirdGoodFactAtDeadline){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",.2,{},.05);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  auto held=sdk(a,10.02,2);held.phase="holding";held.reason="mc_stale";held.grant_ready=false;
  ASSERT_TRUE(a.observe(held,10.02));ASSERT_FALSE(a.tick(10020000000LL,true,true).allowed);
  for(const auto sample:std::vector<std::pair<double,uint64_t>>{{10.1,3},{10.15,4}}){
    ASSERT_TRUE(a.observe(sdk(a,sample.first,sample.second),sample.first));
    EXPECT_FALSE(a.tick(int64_t(sample.first*1e9),true,true).allowed);
  }
  ASSERT_TRUE(a.observe(sdk(a,10.22,5),10.22));
  const auto deadline=a.tick(10220000000LL,true,true);
  EXPECT_FALSE(deadline.allowed);EXPECT_TRUE(deadline.revoked);EXPECT_TRUE(a.stopping());
  EXPECT_EQ(a.failureReason(),"execution_local_recovery_timeout");
}

TEST(TaskExecutionAuthority,ConfiguredSixHundredMillisecondSdkRecoveryIsIndependentOfLocalGaps){
  TaskExecutionAuthority a(taskVersion(),"isolated_mock",30.,{},.6,true);arm(a);
  ASSERT_TRUE(a.tick(10010000000LL,true,true).allowed);
  auto held=sdk(a,10.02,2);held.phase="holding";held.reason="mc_stale";held.grant_ready=false;
  ASSERT_TRUE(a.observe(held,10.02));EXPECT_FALSE(a.tick(10020000000LL,true,true).allowed);
  struct Sample{double time;uint64_t sequence;bool allowed;};
  for(const auto s:std::vector<Sample>{{10.1,3,false},{10.3,4,false},{10.69,5,false},{10.71,6,true}}){
    ASSERT_TRUE(a.observe(sdk(a,s.time,s.sequence),s.time));
    const auto lease=a.tick(int64_t(s.time*1e9),false,true);
    EXPECT_EQ(lease.allowed,s.allowed);EXPECT_FALSE(lease.revoked);
    EXPECT_EQ(lease.control_epoch,1U);EXPECT_EQ(lease.version.route_hash,taskVersion().route_hash);
  }
  EXPECT_FALSE(a.stopping());EXPECT_TRUE(a.failureReason().empty());
}
