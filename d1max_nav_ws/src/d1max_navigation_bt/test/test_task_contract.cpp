#include "d1max_navigation_bt/task_contract.hpp"
#include "d1max_navigation_bt/route_contract.hpp"
#include "d1max_navigation_bt/runtime_policy.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <rclcpp_action/types.hpp>

using namespace d1max_navigation_bt;

TEST(TransitionAdmission, CrossSessionOrMissingSessionCannotBeginDrain) {
  EXPECT_EQ(transitionAdmission(2,"","current"),"transition_session_mismatch");
  EXPECT_EQ(transitionAdmission(2,"old","current"),"transition_session_mismatch");
  EXPECT_EQ(transitionAdmission(1,"current","current"),"unsupported_schema_version");
  EXPECT_TRUE(transitionAdmission(2,"current","current").empty());
}

TEST(GoalAdmission, CancelBeforeDelayedRequestRejectsOldSourceButAllowsNewIntent) {
  GoalAdmission admission;
  admission.fenceAll(100000000000LL);
  EXPECT_EQ(admission.reserve(99900000000LL,100100000000LL),"goal_source_precedes_admission_barrier");
  EXPECT_TRUE(admission.reserve(100100000000LL,100100000000LL).empty());
  EXPECT_TRUE(admission.commit(100100000000LL,100110000000LL).empty());
}

TEST(GoalAdmission, CancelInitialPoseOrDrainBetweenAcceptanceAndDispatchPreventsSubmit) {
  GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(100000000000LL,100000000000LL).empty());
  admission.fenceAll(100010000000LL);
  EXPECT_EQ(admission.commit(100000000000LL,100020000000LL),"goal_source_precedes_admission_barrier");
}

TEST(GoalAdmission, TargetedOldActionCancelCannotRevokeNewAcceptedGoal) {
  GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(100000000000LL,100000000000LL).empty());
  ASSERT_TRUE(admission.commit(100000000000LL,100000000000LL).empty());
  ASSERT_TRUE(admission.reserve(100100000000LL,100100000000LL).empty());
  admission.fenceThrough(100000000000LL);
  EXPECT_TRUE(admission.commit(100100000000LL,100110000000LL).empty());
}

TEST(GoalAdmission, ActionAndTopicShareOneSourceWatermark) {
  GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(100000000000LL,100000000000LL).empty());
  ASSERT_TRUE(admission.reserve(100100000000LL,100100000000LL).empty());
  EXPECT_EQ(admission.commit(100000000000LL,100110000000LL),"goal_request_superseded_before_submission");
  ASSERT_TRUE(admission.commit(100100000000LL,100120000000LL).empty());
  EXPECT_EQ(admission.reserve(100100000000LL,100130000000LL),"goal_source_replayed_or_reordered");
}

TEST(GoalAdmission, ZeroFutureStaleAndClockRollbackDoNotBecomeNewTasks) {
  GoalAdmission admission;
  EXPECT_EQ(admission.reserve(0,100000000000LL),"goal_source_stamp_missing");
  EXPECT_EQ(admission.reserve(100100000000LL,100000000000LL),"goal_source_stamp_in_future");
  EXPECT_EQ(admission.reserve(94000000000LL,100000000000LL),"goal_source_stamp_stale");
  ASSERT_TRUE(admission.reserve(100000000000LL,100000000000LL).empty());
  EXPECT_EQ(admission.commit(100000000000LL,99999999999LL),"goal_admission_clock_rollback");
}

TEST(GoalAdmission, CancelAllInvalidatesAlreadyReservedSlightlyFutureIntent) {
  GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(100040000000LL,100000000000LL).empty());
  admission.fenceAll(100010000000LL);
  EXPECT_EQ(admission.commit(100040000000LL,100050000000LL),"goal_source_precedes_admission_barrier");
}

TEST(GoalAdmission, DelayedManualTakeoverRetiresPreEventFutureSkewReservationOnly) {
  GoalAdmission admission;
  ASSERT_TRUE(admission.reserve(100040000000LL,100000000000LL,"old").empty());
  admission.fenceAcceptedBefore(100010000000LL);
  EXPECT_EQ(admission.commit(100040000000LL,100050000000LL,"old"),
    "goal_request_superseded_before_submission");
  ASSERT_TRUE(admission.reserve(100060000000LL,100060000000LL,"new").empty());
  // A retained duplicate old boundary never cancels the new operator intent.
  admission.fenceAcceptedBefore(100010000000LL);
  EXPECT_TRUE(admission.commit(100060000000LL,100070000000LL,"new").empty());
}
namespace {
std::string actionRequest(unsigned char value) {
  rclcpp_action::GoalUUID uuid{};uuid.back()=value;
  return rclcpp_action::to_string(uuid);
}
}
TEST(GoalAdmission, SameRosSourceDifferentActionUuidsCanReplaceWithoutInventingTime) {
  GoalAdmission admission;constexpr std::int64_t at=100000000000LL;
  const auto a=actionRequest(1),b=actionRequest(2),c=actionRequest(3);
  ASSERT_TRUE(admission.reserve(at,at,a).empty());
  ASSERT_TRUE(admission.commit(at,at,a).empty());
  ASSERT_TRUE(admission.reserve(at,at,b).empty());
  ASSERT_TRUE(admission.reserve(at,at,c).empty());
  EXPECT_EQ(admission.commit(at,at,b),"goal_request_superseded_before_submission");
  EXPECT_EQ(admission.commit(at,at,a),"goal_request_superseded_before_submission");
  EXPECT_TRUE(admission.commit(at,at,c).empty());
  EXPECT_EQ(admission.reserve(at,at,c),"goal_request_replayed");
  EXPECT_EQ(admission.commit(at,at,c),"goal_request_superseded_before_submission");
}
TEST(GoalAdmission, TargetedOldUuidCancelDoesNotFenceSameStampNewAction) {
  GoalAdmission admission;constexpr std::int64_t at=100000000000LL;
  const auto a=actionRequest(1),b=actionRequest(2);
  ASSERT_TRUE(admission.reserve(at,at,a).empty());
  ASSERT_TRUE(admission.commit(at,at,a).empty());
  ASSERT_TRUE(admission.reserve(at,at,b).empty());
  admission.fenceRequest(a);
  EXPECT_TRUE(admission.commit(at,at,b).empty());
}
TEST(GoalAdmission, CancelAllPoseResetAndSourceAgeStillFenceActionIdentity) {
  GoalAdmission admission;constexpr std::int64_t at=100000000000LL;
  ASSERT_TRUE(admission.reserve(at,at,actionRequest(1)).empty());
  admission.fenceAll(at);
  EXPECT_EQ(admission.commit(at,at,actionRequest(1)),"goal_source_precedes_admission_barrier");
  EXPECT_EQ(admission.reserve(at,at,actionRequest(2)),"goal_source_precedes_admission_barrier");
  EXPECT_EQ(admission.reserve(at-6000000000LL,at,actionRequest(3)),"goal_source_precedes_admission_barrier");
  EXPECT_TRUE(admission.reserve(at+100000000LL,at+100000000LL,actionRequest(4)).empty());
  EXPECT_TRUE(admission.commit(at+100000000LL,at+100000000LL,actionRequest(4)).empty());
  EXPECT_EQ(admission.reserve(at+100000000LL,at+6100000000LL,actionRequest(5)),"goal_source_stamp_stale");
  EXPECT_EQ(admission.reserve(at+6200000000LL,at+6100000000LL,actionRequest(6)),"goal_source_stamp_in_future");
}
TEST(GoalAdmission, TopicCompatibilityRemainsTimeOrderedWhenActionsShareAStamp) {
  GoalAdmission admission;constexpr std::int64_t at=100000000000LL;
  ASSERT_TRUE(admission.reserve(at,at,actionRequest(1)).empty());
  ASSERT_TRUE(admission.commit(at,at,actionRequest(1)).empty());
  EXPECT_EQ(admission.reserve(at,at),"goal_source_replayed_or_reordered");
  ASSERT_TRUE(admission.reserve(at+1,at+1).empty());
  EXPECT_EQ(admission.commit(at,at+1,actionRequest(1)),"goal_request_superseded_before_submission");
  EXPECT_TRUE(admission.commit(at+1,at+1).empty());
}
namespace {
const RouteBinding binding{"session", "task", "route", std::string(64,'a')};
auto confirm(ConfirmationLedger& ledger, const std::string& request="request", bool preview=true,
             bool available=false, bool eligible=true, bool ready=true) {
  return ledger.confirm(2,request,binding,binding,true,ready,eligible,preview,available);
}
auto snapshot() {
  d1max_navigation_bt_interfaces::msg::RouteSnapshot s;
  s.schema_version=2; s.session_id="session"; s.task_id="task"; s.route_id="route";
  s.route_hash=s.source_map_sha256=s.tomogram_sha256=s.conditioning_sha256=std::string(64,'a');
  s.map_version_id="map"; s.frame_id="d1max_loc_map"; s.path.header.frame_id=s.frame_id;
  s.point_reference="ground"; s.preview_ready=true; s.direction="forward";
  s.localization_epoch=1; s.localization_seed_id="seed"; s.geometry_evidence_json="{}";
  s.path.poses.resize(3);
  for (unsigned i=0;i<3;++i) { s.path.poses[i].pose.position.x=i; s.path.poses[i].pose.orientation.w=1.; }
  s.layer_ids={0,0,1}; s.source_layer_ids={0,0,1}; s.point_floor_ids={"floor1","stair","floor2"};
  s.edge_segments={"a","b"};
  d1max_navigation_bt_interfaces::msg::RouteSegment a;
  a.segment_id="a"; a.kind="floor"; a.begin_index=0; a.end_index=1; a.required_mode="general"; a.floor_id="floor1";
  auto b=a; b.segment_id="b"; b.kind="stair_up"; b.begin_index=1; b.end_index=2; b.required_mode="stair"; b.floor_id="stair"; b.target_layer_id=1;
  s.segments={a,b};
  return s;
}
std::string validate(const d1max_navigation_bt_interfaces::msg::RouteSnapshot& s) {
  return validateSnapshot(s,binding,1,"seed","d1max_loc_map","map",s.path);
}
}

TEST(Confirmation, PreviewRejectsWithoutConsumingPermission) {
  ConfirmationLedger l;
  auto result=confirm(l);
  EXPECT_FALSE(result.accepted); EXPECT_FALSE(result.authorized); EXPECT_FALSE(l.authorized());
  EXPECT_EQ(result.reason,"physical_execution_not_implemented_preview_only");
  result=confirm(l); EXPECT_TRUE(result.duplicate); EXPECT_FALSE(result.authorized);
  // Pure policy test only: a separately implemented real backend would need
  // all explicit capabilities, not changing a ROS preview flag alone.
  result=confirm(l,"request",false,true);
  EXPECT_TRUE(result.accepted); EXPECT_FALSE(result.duplicate);
}
TEST(Confirmation, CapacityFailureDoesNotConsumeConfirmationIdentity) {
  ConfirmationLedger l;
  for(unsigned i=0;i<256;++i)EXPECT_FALSE(confirm(l,"rejected"+std::to_string(i),true,false).authorized);
  const auto overflow=confirm(l,"overflow",false,true);
  EXPECT_EQ(overflow.reason,"confirmation_request_capacity_exceeded");EXPECT_FALSE(l.authorized());
  // Existing admitted request may still move from preview rejection to a real
  // confirmation. The failed new request did not poison route_already_confirmed.
  EXPECT_TRUE(confirm(l,"rejected0",false,true).authorized);
}
TEST(Confirmation, IdempotentConsumptionAndNoSecondGrant) {
  ConfirmationLedger l;
  auto first=confirm(l,"request",false,true);
  auto retry=confirm(l,"request",false,true);
  EXPECT_TRUE(first.authorized); EXPECT_TRUE(retry.duplicate);
  EXPECT_EQ(first.confirmation_id,retry.confirmation_id);
  EXPECT_FALSE(confirm(l,"different",false,true).accepted);
}
TEST(Confirmation, BindingAndSchemaCannotBeReplayed) {
  ConfirmationLedger l;
  EXPECT_FALSE(l.confirm(1,"a",binding,binding,true,true,true,false,true).accepted);
  EXPECT_TRUE(confirm(l,"a",false,true).accepted);
  auto other=binding; other.hash=std::string(64,'b');
  EXPECT_EQ(l.confirm(2,"a",other,other,true,true,true,false,true).reason,"request_id_binding_conflict");
  EXPECT_EQ(l.confirm(2,"b",other,binding,true,true,true,false,true).reason,"route_snapshot_mismatch");
}
TEST(Confirmation, CancelRevokesEvenAnIdenticalRetry) {
  ConfirmationLedger l; ASSERT_TRUE(confirm(l,"a",false,true).authorized);
  l.revoke(); EXPECT_FALSE(l.authorized());
  EXPECT_FALSE(confirm(l,"a",false,true).authorized);
}
TEST(Confirmation, StaleContextAndUnverifiedGeometryNeverAuthorize) {
  ConfirmationLedger l;
  EXPECT_FALSE(confirm(l,"a",false,true,true,false).authorized);
  EXPECT_FALSE(confirm(l,"a",false,true,false,true).authorized);
  EXPECT_FALSE(confirm(l,"a",false,false,true,true).authorized);
  EXPECT_FALSE(l.authorized());
}
TEST(Confirmation, ConditionedRouteFlagCannotVetoPreparedTaskFollowerOrGrantWithoutPreparation) {
  ConfirmationLedger ledger;
  const bool original_route_flag=false;
  auto missing=confirm(ledger,"same-intent",false,true,
    confirmationGeometryReady("task_follower_v1",original_route_flag,false));
  EXPECT_FALSE(missing.authorized);
  auto prepared=confirm(ledger,"same-intent",false,true,
    confirmationGeometryReady("task_follower_v1",original_route_flag,true));
  ASSERT_TRUE(prepared.authorized);
  auto duplicate=confirm(ledger,"same-intent",false,true,
    confirmationGeometryReady("task_follower_v1",original_route_flag,true));
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_EQ(duplicate.confirmation_id,prepared.confirmation_id);
  EXPECT_FALSE(original_route_flag); // No map/route acceptance fact was rewritten.
  ledger.revoke();
  EXPECT_FALSE(confirm(ledger,"same-intent",false,true,true).authorized);
}
TEST(GoalExecutionIntent, StartupAndPreviewGoalsHaveNoImplicitIntent) {
  GoalExecutionIntent idle;
  EXPECT_FALSE(idle.requested());EXPECT_FALSE(idle.prepare(binding,true));
  EXPECT_FALSE(idle.consume(binding));EXPECT_FALSE(idle.consumed());
}
TEST(GoalExecutionIntent, MissingRouteAndInactiveTaskDoNotConsumeRequest) {
  GoalExecutionIntent intent(true,"goal-request");
  auto incomplete=binding;incomplete.route.clear();
  EXPECT_FALSE(intent.prepare(incomplete,true));EXPECT_FALSE(intent.prepare(binding,false));
  intent.wait("waiting_matching_native_and_tracker_preparation");
  EXPECT_FALSE(intent.consumed());EXPECT_TRUE(intent.prepare(binding,true));
  EXPECT_EQ(intent.requestId(),"goal-request");
}
TEST(GoalExecutionIntent, PinsExactCommittedRouteBeforeOneExistingConfirmation) {
  GoalExecutionIntent intent(true,"goal-request");ConfirmationLedger ledger;
  ASSERT_TRUE(intent.prepare(binding,true));
  auto waiting=ledger.confirm(2,intent.requestId(),binding,binding,true,false,true,false,true);
  EXPECT_FALSE(waiting.authorized);intent.wait(waiting.reason);EXPECT_FALSE(intent.consumed());
  auto accepted=ledger.confirm(2,intent.requestId(),binding,binding,true,true,true,false,true);
  ASSERT_TRUE(accepted.authorized);EXPECT_TRUE(intent.consume(binding));EXPECT_TRUE(intent.consumed());
  EXPECT_FALSE(intent.prepare(binding,true)); // Recovery cannot consume a second request.
  EXPECT_TRUE(intent.consume(binding)); // Idempotent fact, not another grant.
  EXPECT_TRUE(ledger.confirm(2,intent.requestId(),binding,binding,true,true,true,false,true).duplicate);
}
TEST(GoalExecutionIntent, RouteHashOrTaskChangeRevokesRatherThanRebinding) {
  for(bool change_task:{false,true}) {
    GoalExecutionIntent intent(true,"goal-request");ASSERT_TRUE(intent.prepare(binding,true));
    auto other=binding;if(change_task)other.task="new-task";else other.hash=std::string(64,'b');
    EXPECT_FALSE(intent.prepare(other,true));EXPECT_FALSE(intent.prepare(binding,true));
    EXPECT_FALSE(intent.consume(other));EXPECT_FALSE(intent.consumed());
    EXPECT_EQ(intent.reason(),"goal_execution_route_changed");
  }
}
TEST(GoalExecutionIntent, CancelAndLocalizationResetDoNotAutoAuthorizeAfterRecovery) {
  for(bool consumed:{false,true}) {
    GoalExecutionIntent intent(true,"goal-request");ASSERT_TRUE(intent.prepare(binding,true));
    if(consumed)ASSERT_TRUE(intent.consume(binding));
    intent.revoke("localization_context_replaced");
    EXPECT_FALSE(intent.prepare(binding,true));EXPECT_FALSE(intent.consume(binding));
    EXPECT_EQ(intent.reason(),"localization_context_replaced");
  }
}
TEST(GoalExecutionIntent, ExplicitIntentCannotBypassPreviewOrPhysicalEvidence) {
  for(bool preview:{false,true}) {
    GoalExecutionIntent intent(true,"goal-request");ConfirmationLedger ledger;
    ASSERT_TRUE(intent.prepare(binding,true));
    const auto decision=ledger.confirm(2,intent.requestId(),binding,binding,true,true,true,preview,false);
    EXPECT_FALSE(decision.authorized);EXPECT_FALSE(intent.consumed());EXPECT_FALSE(ledger.authorized());
  }
}
TEST(GoalExecutionIntent, RequestIdentityMustBeBounded) {
  EXPECT_THROW(GoalExecutionIntent(true,""),std::invalid_argument);
  EXPECT_THROW(GoalExecutionIntent(true,std::string(129,'a')),std::invalid_argument);
  EXPECT_NO_THROW(GoalExecutionIntent(false,""));
}
TEST(LifecycleDrain, WaitsAsynchronouslyBeforeReactivationAndCleanup) {
  LifecycleDrain l;
  EXPECT_FALSE(l.activate()); ASSERT_TRUE(l.configure()); ASSERT_TRUE(l.activate());
  l.begin(); EXPECT_FALSE(l.accepting()); EXPECT_TRUE(l.draining());
  EXPECT_FALSE(l.activate()); EXPECT_FALSE(l.cleanup());
  l.observe(false,false); EXPECT_TRUE(l.draining());
  l.observe(true,false); EXPECT_FALSE(l.draining()); ASSERT_TRUE(l.cleanup());
  EXPECT_FALSE(l.activate()); ASSERT_TRUE(l.configure()); EXPECT_TRUE(l.activate());
}
TEST(LifecycleDrain, SoftwareRetirementIsNotPhysicalStop) {
  LifecycleDrain l; l.configure(); l.activate(); l.begin();
  l.observe(true,false,true,false); EXPECT_TRUE(l.draining());
  EXPECT_FALSE(l.activate());
  l.observe(true,false,true,true); EXPECT_FALSE(l.draining()); EXPECT_TRUE(l.activate());
}
TEST(LifecycleDrain, UnconfirmedRetirementCannotBeClearedByLateReply) {
  LifecycleDrain l; l.configure(); l.activate(); l.begin();
  l.observe(false,true); l.observe(true,false);
  EXPECT_TRUE(l.quarantined()); EXPECT_FALSE(l.activate()); EXPECT_FALSE(l.cleanup());
}
TEST(PhysicalStop, FreshPostRequestMeasurementsAndDwellAreRequired) {
  StopVerifier verifier; verifier.begin("task","mode-1",10.);
  EXPECT_FALSE(verifier.confirmed(10.5));
  EXPECT_FALSE(verifier.observe({"task","mode-1",9.9,10.1,0,0,0},10.1));
  EXPECT_FALSE(verifier.observe({"task","mode-1",10.1,10.1,0,0,0},10.1));
  EXPECT_FALSE(verifier.observe({"task","mode-1",10.25,10.25,0,0,0},10.25));
  EXPECT_TRUE(verifier.observe({"task","mode-1",10.45,10.45,0,0,0},10.45));
  EXPECT_FALSE(verifier.confirmed(10.8));
}
TEST(PhysicalStop, WrongIdentityRepeatedTimestampsAndMovementAreNotProof) {
  StopVerifier verifier; verifier.begin("task","mode-1",10.);
  EXPECT_FALSE(verifier.observe({"old-task","mode-1",10.1,10.1,0,0,0},10.1));
  EXPECT_FALSE(verifier.observe({"task","old-mode",10.1,10.1,0,0,0},10.1));
  for(int i=0;i<20;++i) EXPECT_FALSE(verifier.observe({"task","mode-1",10.1,10.1,0,0,0},10.1));
  EXPECT_FALSE(verifier.observe({"task","mode-1",10.2,10.2,.1,0,0},10.2));
  EXPECT_FALSE(verifier.observe({"task","mode-1",10.3,10.3,0,0,0},10.3));
  EXPECT_FALSE(verifier.observe({"task","mode-1",10.7,10.7,0,0,0},10.7));
}
TEST(RouteSnapshot, TypedPreviewAndLegacyDisplayAgree) {
  EXPECT_TRUE(validate(snapshot()).empty());
}
TEST(RouteSnapshot, RejectsProtocolAndContextMismatch) {
  auto s=snapshot(); s.schema_version=1; EXPECT_EQ(validate(s),"route_schema_mismatch");
  s=snapshot(); s.localization_epoch=2; EXPECT_EQ(validate(s),"route_snapshot_identity_mismatch");
  s=snapshot(); s.map_version_id="other"; EXPECT_EQ(validate(s),"route_snapshot_identity_mismatch");
  s=snapshot(); s.task_id="old"; EXPECT_EQ(validate(s),"route_snapshot_identity_mismatch");
}
TEST(RouteSnapshot, RejectsMissingOrChangedGeometryAndProvenance) {
  auto s=snapshot(); s.source_map_sha256="unknown"; EXPECT_EQ(validate(s),"route_provenance_hash_invalid");
  s=snapshot(); s.path.poses[1].pose.position.z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate(s),"route_point_invalid");
  s=snapshot(); auto legacy=s.path; legacy.poses[1].pose.position.x+=.1;
  EXPECT_EQ(validateSnapshot(s,binding,1,"seed",s.frame_id,"map",legacy),"legacy_route_disagrees_with_snapshot");
}
TEST(RouteSnapshot, SegmentCoverageAndLayerEvidenceCannotBeLost) {
  auto s=snapshot(); s.segments.pop_back(); EXPECT_EQ(validate(s),"route_segments_incomplete");
  s=snapshot(); s.edge_segments[1]="a"; EXPECT_EQ(validate(s),"route_edge_segment_mismatch");
  s=snapshot(); s.segments[1].begin_index=0; EXPECT_EQ(validate(s),"route_segment_invalid");
  s=snapshot(); s.source_layer_ids.pop_back(); EXPECT_EQ(validate(s),"route_snapshot_shape_invalid");
}
TEST(RouteSnapshot, PreviewIsNotExecutionCertification) {
  auto s=snapshot(); s.execution_eligible=true;
  EXPECT_EQ(validate(s),"route_eligibility_inconsistent");
  s=snapshot(); s.has_goal_yaw=true; s.goal_yaw_tolerance_rad=0;
  EXPECT_EQ(validate(s),"route_goal_yaw_invalid");
}
