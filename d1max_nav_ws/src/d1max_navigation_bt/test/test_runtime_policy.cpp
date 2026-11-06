#include "d1max_navigation_bt/runtime_policy.hpp"
#include "d1max_navigation_bt/stable_recovery.hpp"
#include "d1max_navigation_bt/dependency_health.hpp"

#include <gtest/gtest.h>
#include <limits>

using namespace d1max_navigation_bt;

TEST(InitialTaskIdentity, FreshHistoricalHeartbeatDoesNotBindNewTaskDuringRecovery) {
  EXPECT_FALSE(initialTaskIdentityReady(true,true,false,false));
  EXPECT_FALSE(initialTaskIdentityReady(true,true,false,true));
  EXPECT_FALSE(initialTaskIdentityReady(false,true,true,false));
  EXPECT_FALSE(initialTaskIdentityReady(true,false,true,false));
  EXPECT_FALSE(initialTaskIdentityReady(true,true,true,true));
  EXPECT_TRUE(initialTaskIdentityReady(true,true,true,false));
}
TEST(InitialTaskIdentity, NewUnboundIntentMayWaitButOldBoundAuthorityNeverRecoversAcrossLoss) {
  EXPECT_TRUE(taskLocalizationFault(false,true,true,"atomic_navigation_hard_loss").empty());
  EXPECT_FALSE(initialTaskIdentityReady(true,true,false,true));
  EXPECT_EQ(taskLocalizationFault(true,true,true,"atomic_navigation_hard_loss"),"atomic_navigation_hard_loss");
  EXPECT_EQ(taskLocalizationFault(true,true,true,""),"hard_localization_lost");
  EXPECT_TRUE(taskLocalizationFault(true,true,false,"ready").empty());
  EXPECT_TRUE(taskLocalizationFault(true,false,true,"historical").empty());
}

TEST(ConfirmationGeometry, TaskFollowerUsesCurrentLocalPreparationNotGlobalMapQualification) {
  EXPECT_TRUE(confirmationGeometryReady("task_follower_v1",false,true));
  EXPECT_TRUE(confirmationGeometryReady("task_follower_v1",true,true));
  EXPECT_FALSE(confirmationGeometryReady("task_follower_v1",false,false));
  EXPECT_FALSE(confirmationGeometryReady("task_follower_v1",true,false));
}
TEST(ConfirmationGeometry, LegacyWriterAndUnknownArchitectureCannotBorrowLocalPreparation) {
  EXPECT_FALSE(confirmationGeometryReady("curve_writer_v3",false,true));
  EXPECT_TRUE(confirmationGeometryReady("curve_writer_v3",true,false));
  EXPECT_FALSE(confirmationGeometryReady("unknown",true,true));
}

TEST(GrantAdmissionWait, RetriesAtTwoHzWithoutRenewingTheOriginalPolicyBudget) {
  GrantAdmissionWait wait;
  ASSERT_TRUE(wait.start(100.));ASSERT_TRUE(wait.attempted(100.));
  EXPECT_FALSE(wait.retryDue(100.499));EXPECT_FALSE(wait.attempted(100.499));
  EXPECT_TRUE(wait.retryDue(100.5));ASSERT_TRUE(wait.attempted(100.5));
  for(double now:{101.,110.,120.,129.5}) {
    ASSERT_TRUE(wait.start(now));ASSERT_TRUE(wait.attempted(now));
    EXPECT_FALSE(wait.expired(now,30.));
  }
  EXPECT_DOUBLE_EQ(wait.age(129.5),29.5);
  EXPECT_TRUE(wait.expired(130.,30.));EXPECT_TRUE(wait.expired(140.,30.));
}
TEST(GrantAdmissionWait, CancelOrWriterAcceptanceClearsOnlyTheCurrentRpcEpisode) {
  GrantAdmissionWait old;ASSERT_TRUE(old.attempted(100.));old.reset();
  EXPECT_FALSE(old.pending());EXPECT_DOUBLE_EQ(old.age(101.),0.);
  EXPECT_FALSE(old.expired(131.,30.));
  GrantAdmissionWait next;ASSERT_TRUE(next.attempted(101.));
  EXPECT_TRUE(next.pending());EXPECT_FALSE(next.retryDue(101.49));
  EXPECT_DOUBLE_EQ(next.age(101.5),.5);
}
TEST(GrantAdmissionWait, InvalidOrRolledBackMonotonicTimeCannotRenewTheEpisode) {
  GrantAdmissionWait wait;ASSERT_TRUE(wait.attempted(100.));
  EXPECT_FALSE(wait.start(99.));EXPECT_FALSE(wait.retryDue(99.));
  EXPECT_FALSE(wait.attempted(99.));EXPECT_TRUE(wait.expired(99.,30.));
  EXPECT_TRUE(wait.expired(std::numeric_limits<double>::quiet_NaN(),30.));
  EXPECT_TRUE(wait.expired(100.,0.));EXPECT_DOUBLE_EQ(wait.age(101.),1.);
}
TEST(GrantAdmissionWait, OriginalDeadlineWinsLateAcceptanceButPriorExactStatusEndsAdmissionWait) {
  GrantAdmissionWait late;ASSERT_TRUE(late.attempted(100.));
  EXPECT_FALSE(late.admitted(130.,30.));EXPECT_TRUE(late.pending());
  EXPECT_TRUE(late.expired(130.,30.));
  GrantAdmissionWait status_first;ASSERT_TRUE(status_first.attempted(100.));
  ASSERT_TRUE(status_first.admitted(100.1,30.));EXPECT_FALSE(status_first.pending());
  // A delayed RPC cannot invent a fresh admission episode after a genuine
  // exact writer-status ACK already completed the original one.
  EXPECT_TRUE(status_first.admitted(140.,30.));EXPECT_DOUBLE_EQ(status_first.age(140.),0.);
  EXPECT_FALSE(status_first.pending());
}

TEST(StableRecovery, FirstAdmissionDoesNotRequireRobotMotionOrARecoveryEpisode) {
  StableRecoveryGate gate;
  EXPECT_FALSE(gate.observe(false,0.));
  EXPECT_TRUE(gate.observe(true,.01,100,"epoch1:seed1"));
}
TEST(StableRecovery, WithdrawalIsImmediateAndOneGoodPacketCannotResume) {
  StableRecoveryGate gate;ASSERT_TRUE(gate.observe(true,0.,100,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(false,.1,100,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,.2,101,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,.5,102,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,.79,103,"epoch1:seed1"));
  EXPECT_TRUE(gate.observe(true,.81,104,"epoch1:seed1"));
}
TEST(StableRecovery, RepeatedAndReorderedSourcesCannotManufactureRecovery) {
  StableRecoveryGate gate;gate.observe(true,0.,100,"epoch1:seed1");gate.observe(false,.1);
  for(double now:{.2,.4,.6,.9,1.2})EXPECT_FALSE(gate.observe(true,now,101,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,1.3,99,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,1.4,102,"epoch1:seed1"));
  EXPECT_TRUE(gate.observe(true,1.5,103,"epoch1:seed1"));
}
TEST(StableRecovery, AnotherIdentityOrMissingSourceNeverResumesOldTask) {
  StableRecoveryGate gate;gate.observe(true,0.,100,"epoch1:seed1");gate.observe(false,.1);
  for(int i=0;i<10;++i)EXPECT_FALSE(gate.observe(true,.2+i*.2,200+i,"epoch2:seed2"));
  EXPECT_FALSE(gate.observe(true,3.,0,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,3.2,300,""));
  EXPECT_FALSE(gate.observe(true,3.3,301,"epoch1:seed1"));
}
TEST(StableRecovery, EveryBadSampleRestartsOnlyStabilityNotTheOuterWaitBudget) {
  StableRecoveryGate gate;gate.observe(true,0.,100,"epoch1:seed1");gate.observe(false,.1);
  gate.observe(true,.2,101,"epoch1:seed1");gate.observe(true,.6,102,"epoch1:seed1");
  EXPECT_FALSE(gate.observe(false,.7));
  EXPECT_FALSE(gate.observe(true,.8,103,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,1.2,104,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,1.39,105,"epoch1:seed1"));
  EXPECT_TRUE(gate.observe(true,1.41,106,"epoch1:seed1"));
}
TEST(StableRecovery, MonotonicRollbackNeverRestoresAuthority) {
  StableRecoveryGate gate;ASSERT_TRUE(gate.observe(true,1.,100,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,.9,101,"epoch1:seed1"));
  EXPECT_FALSE(gate.observe(true,2.,102,"epoch1:seed1"));
}
TEST(RouteDeadline, IncludesPauseAndDoesNotConfuseTerminalWorkWithPendingWork) {
  EXPECT_FALSE(routeDeadlineExpired(true,9.999,10.));
  EXPECT_TRUE(routeDeadlineExpired(true,10.,10.));
  EXPECT_TRUE(routeDeadlineExpired(true,30.,10.)); // no active-time renewal
  EXPECT_FALSE(routeDeadlineExpired(false,30.,10.));
  EXPECT_TRUE(routeDeadlineExpired(true,std::numeric_limits<double>::quiet_NaN(),10.));
}

TEST(ArrivalCapability, TypedLocalExecutionDoesNotReenterGlobalStartupAtFinalStop) {
  EXPECT_TRUE(measuredArrivalCapabilityReady(true,true,true,false));
  EXPECT_TRUE(measuredArrivalCapabilityReady(true,true,true,true));
  EXPECT_FALSE(measuredArrivalCapabilityReady(true,false,true,true));
  EXPECT_FALSE(measuredArrivalCapabilityReady(true,true,false,true));
  EXPECT_FALSE(measuredArrivalCapabilityReady(true,false,false,true));
}
TEST(ArrivalCapability, LegacyVisualizationStillRequiresItsOriginalGlobalReadiness) {
  EXPECT_FALSE(measuredArrivalCapabilityReady(false,true,true,false));
  EXPECT_TRUE(measuredArrivalCapabilityReady(false,false,false,true));
}

TEST(ExecutionPurpose, LivePlanningOnlyRetainsGeometryOwnerWithoutAnyMotionAuthority) {
  EXPECT_TRUE(validExecutionPurpose("planning_only",true,"live"));
  EXPECT_TRUE(usesGeometryOwner("planning_only",true));
  EXPECT_FALSE(permitsMotionAuthority("planning_only",true));
  EXPECT_EQ(executionConfirmationBlocker("planning_only"),"planning_only_execution_disabled");
}
TEST(ExecutionPurpose, PlanningOnlyCannotBeMisconfiguredAsExecutionOrMock) {
  EXPECT_FALSE(validExecutionPurpose("planning_only",false,"live"));
  EXPECT_FALSE(validExecutionPurpose("planning_only",true,"isolated_mock"));
  EXPECT_FALSE(validExecutionPurpose("unknown",true,"live"));
  EXPECT_FALSE(permitsMotionAuthority("planning_only",false));
}
TEST(ExecutionPurpose, ExistingExecutionAndLegacyPreviewContractsAreUnchanged) {
  EXPECT_TRUE(validExecutionPurpose("execution",false,"live"));
  EXPECT_TRUE(validExecutionPurpose("execution",false,"isolated_mock"));
  EXPECT_TRUE(usesGeometryOwner("execution",false));
  EXPECT_TRUE(permitsMotionAuthority("execution",false));
  EXPECT_FALSE(usesGeometryOwner("execution",true));
  EXPECT_FALSE(permitsMotionAuthority("execution",true));
  EXPECT_TRUE(executionConfirmationBlocker("execution").empty());
}

TEST(PlanningScope, GlobalRouteOwnsOnlyGlobalWorkAndRequiresLivePlanningOnly) {
  EXPECT_TRUE(validPlanningScope("global_route","planning_only",true,"live"));
  EXPECT_FALSE(usesLocalExecutionAuthority("global_route"));
  EXPECT_FALSE(validPlanningScope("global_route","execution",true,"live"));
  EXPECT_FALSE(validPlanningScope("global_route","planning_only",false,"live"));
  EXPECT_FALSE(validPlanningScope("global_route","planning_only",true,"isolated_mock"));
  EXPECT_FALSE(validPlanningScope("unknown","planning_only",true,"live"));
  // No local/SDK grant has ever been created: after Compute terminates the
  // existing software-only completion and replacement predicates need no ACK.
  EXPECT_TRUE(completedTaskReclaimable(false,true,true,false,false,true,false,false));
  EXPECT_EQ(publicCompletion(true,false,false,false,true,false),PublicCompletion::Succeeded);
  EXPECT_EQ(publicCompletion(true,false,false,false,false,true),PublicCompletion::Canceled);
  EXPECT_FALSE(completedTaskReclaimable(false,true,true,true,false,true,false,false));
}

TEST(PlanningScope, NavigationRetainsExistingLocalOwnerInBothPurposes) {
  EXPECT_TRUE(validPlanningScope("navigation","execution",false,"live"));
  EXPECT_TRUE(validPlanningScope("navigation","execution",false,"isolated_mock"));
  EXPECT_TRUE(validPlanningScope("navigation","planning_only",true,"live"));
  EXPECT_TRUE(usesLocalExecutionAuthority("navigation"));
  EXPECT_FALSE(usesLocalExecutionAuthority("unknown"));
  EXPECT_TRUE(usesGeometryOwner("planning_only",true));
  EXPECT_TRUE(permitsMotionAuthority("execution",false));
  EXPECT_FALSE(permitsMotionAuthority("planning_only",true));
}

TEST(TaskIdentity, GeneratedIdsStayInsideAdapterContract) {
  EXPECT_TRUE(validSessionId("session-name_01.abcdef"));
  EXPECT_FALSE(validSessionId(""));
  EXPECT_FALSE(validSessionId("session:bad"));
  EXPECT_FALSE(validSessionId(std::string(97, 'a')));
  // A maximum 20-digit uint64 generation still fits the 128-byte action limit.
  EXPECT_LT(std::string(96, 'a').size() + 1 + 20, 128U);
}

TEST(GoalFrame, RvizThreeDimensionalSelectionPreservesItsPlanningFrame) {
  EXPECT_TRUE(validGoalFrame("3d", "planning", "map", "planning"));
  EXPECT_TRUE(validGoalFrame("3d", "map", "map", "planning"));
  EXPECT_TRUE(validGoalFrame("2d", "map", "map", "planning"));
  EXPECT_FALSE(validGoalFrame("2d", "planning", "map", "planning"));
  EXPECT_FALSE(validGoalFrame("3d", "unrelated", "map", "planning"));
  EXPECT_FALSE(validGoalFrame("invalid", "map", "map", "planning"));
}

TEST(AsyncRequest, DispatchIsExactlyOnce) {
  AsyncRequestState request;
  EXPECT_FALSE(request.pending());
  EXPECT_TRUE(request.begin());
  EXPECT_FALSE(request.begin());
  EXPECT_TRUE(request.pending());
}

TEST(AsyncRequest, CancelBeforeAcceptanceWaitsForHandle) {
  AsyncRequestState request;
  request.begin();
  EXPECT_FALSE(request.shouldCancel(false));
  EXPECT_TRUE(request.pending());
  EXPECT_TRUE(request.shouldCancel(true));
  request.cancel_sent = true;
  EXPECT_FALSE(request.shouldCancel(true));
  // Even a cancel ACK does not release the old worker before its result.
  EXPECT_TRUE(request.pending());
}

TEST(AsyncRequest, RetiredResultDrainsWorkerButCannotCommitOldRoute) {
  AsyncRequestState request;
  request.begin();
  EXPECT_FALSE(request.acceptTerminal(true));
  EXPECT_FALSE(request.pending());
  EXPECT_FALSE(request.shouldCancel(true));
}

TEST(AsyncRequest, DuplicateResultCannotReplaceCommittedResult) {
  AsyncRequestState request;
  request.begin();
  EXPECT_TRUE(request.acceptTerminal(false));
  EXPECT_FALSE(request.acceptTerminal(false));
  EXPECT_FALSE(request.begin());
}

TEST(AsyncRequest, UnrequestedResultIsNotAccepted) {
  AsyncRequestState request;
  EXPECT_FALSE(request.acceptTerminal(false));
  EXPECT_FALSE(request.terminal);
}

TEST(AsyncRequest, IndependentGenerationsDoNotShareTerminalState) {
  AsyncRequestState old_request, new_request;
  old_request.begin();
  new_request.begin();
  EXPECT_FALSE(old_request.acceptTerminal(true));
  EXPECT_TRUE(new_request.pending());
  EXPECT_TRUE(new_request.acceptTerminal(false));
}

TEST(ActionAdmission, GlobalDiscoveryIsNotALocalTrackingDependency) {
  EXPECT_EQ(plannerAdmissionWait(false, false, false, false, true), "waiting_compute_route_action");
  EXPECT_TRUE(plannerAdmissionWait(false, false, false, true, false).empty());
  EXPECT_TRUE(plannerAdmissionWait(true, false, false, false, false).empty());
  EXPECT_EQ(plannerAdmissionWait(true, true, false, false, false), "waiting_follow_route_action");
  EXPECT_TRUE(plannerAdmissionWait(true, true, false, false, true).empty());
  EXPECT_TRUE(plannerAdmissionWait(true, true, true, false, false).empty());
}

TEST(ActionTransport, MissingAcceptanceHasBoundedFailure) {
  EXPECT_TRUE(actionTransportTimeout(true, false, 2.9, 100, 3).empty());
  EXPECT_EQ(actionTransportTimeout(true, false, 3, 100, 3), "action_acceptance_timeout");
  EXPECT_TRUE(actionTransportTimeout(false, false, 100, 100, 3).empty());
}

TEST(ActionTransport, AliveBlockedWorkerMayRunWithoutReachingAnArtificialTotalDuration) {
  EXPECT_TRUE(actionTransportTimeout(true, true, 3600, 0.2, 3).empty());
  EXPECT_EQ(actionTransportTimeout(true, true, 3600, 3, 3), "action_feedback_timeout");
  EXPECT_EQ(actionTransportTimeout(true, true, 1,
    std::numeric_limits<double>::quiet_NaN(), 3), "action_feedback_timeout");
}

TEST(ActionTransport, WatchdogDoesNotPretendTheWorkerHasTerminated) {
  AsyncRequestState request;
  request.begin();
  EXPECT_EQ(actionTransportTimeout(request.pending(), true, 4, 4, 3), "action_feedback_timeout");
  EXPECT_TRUE(request.pending());
  EXPECT_TRUE(request.shouldCancel(true));
  EXPECT_FALSE(request.acceptTerminal(true));
  EXPECT_FALSE(request.pending());
}

TEST(Cancellation, CleanupFailureQuarantinesEvenAfterActionHasTerminated) {
  EXPECT_TRUE(unresolvedRetirement(true, true, ""));
  EXPECT_TRUE(unresolvedRetirement(true, false, "native_worker_stop_unconfirmed"));
  EXPECT_FALSE(unresolvedRetirement(true, false, ""));
  EXPECT_FALSE(unresolvedRetirement(false, false, ""));
}

TEST(Cancellation, AcknowledgedPlannerFailureAndUnconfirmedCleanupAreDifferent) {
  EXPECT_FALSE(retirementResultUnconfirmed(false, false, true));
  EXPECT_TRUE(retirementResultUnconfirmed(false, false, false));
  EXPECT_FALSE(retirementResultUnconfirmed(false, true, true));
  EXPECT_FALSE(retirementResultUnconfirmed(true, false, false));
}

TEST(HealthReport, BothReceiptAndSourceTimesMustBeFresh) {
  EXPECT_TRUE(reportFresh(0.3, 0.2, 0.6));
  EXPECT_FALSE(reportFresh(1.0, 0.0, 0.6));
  EXPECT_FALSE(reportFresh(0.1, 1.0, 0.6));
}

TEST(HealthReport, FutureNaNAndInvalidBudgetsRejected) {
  EXPECT_FALSE(reportFresh(-0.101, 0, 0.6));
  EXPECT_FALSE(reportFresh(std::numeric_limits<double>::quiet_NaN(), 0, 0.6));
  EXPECT_FALSE(reportFresh(0, -0.1, 0.6));
  EXPECT_FALSE(reportFresh(0, 0, 0));
}

TEST(ArrivalProof, AcknowledgedCleanupDelayDoesNotEraseARealMeasuredEvent) {
  EXPECT_TRUE(reportFresh(2.8, 0.1, 4.0));
  // A queued successful result must keep its original sensor timestamp.
  EXPECT_FALSE(reportFresh(10.0, 0.0, 4.0));
  // Reject a stale event at RECEIPT; consumption of an accepted completion
  // uses its immutable task identity, not another freshness deadline.
  EXPECT_FALSE(reportFresh(4.1, 4.1, 4.0));
  EXPECT_FALSE(reportFresh(-1.0, 0.0, 4.0));
}

TEST(ArrivalProof, AcceptedCompletionSurvivesReadinessPauseWithoutRestamping) {
  ArrivalIdentity key{"task","session","map","route","hash","seed",7};
  MeasuredArrivalEvent event;
  ASSERT_TRUE(event.accept(key,100000000000LL,100100000000LL,4.,true));
  // There is deliberately no 'now' input: completing a suspended tree must
  // not invent another measured source sample at 105/110 seconds.
  EXPECT_TRUE(event.matches(key,true,true));
  EXPECT_EQ(event.sourceStamp(),100000000000LL);
  EXPECT_FALSE(event.accept(key,110000000000LL,110100000000LL,4.,true));
  EXPECT_EQ(event.sourceStamp(),100000000000LL);
  EXPECT_FALSE(event.matches(key,true,false));
  EXPECT_FALSE(event.matches(key,false,true));
}
TEST(ArrivalProof, StaleAtReceiptOrRetiredSuccessCannotBecomeCompletion) {
  ArrivalIdentity key{"task","session","map","route","hash","seed",7};
  for(const auto source:{0LL,90000000000LL,102000000000LL}) {
    MeasuredArrivalEvent event;
    EXPECT_FALSE(event.accept(key,source,100000000000LL,4.,true));
    EXPECT_FALSE(event.matches(key,true,true));
  }
  MeasuredArrivalEvent event;
  EXPECT_FALSE(event.accept(key,100000000000LL,100100000000LL,4.,false));
  EXPECT_FALSE(event.matches(key,true,true));
}
TEST(ArrivalProof, NewTaskRouteMapOrLocalizationCannotInheritCompletion) {
  const ArrivalIdentity key{"task","session","map","route","hash","seed",7};
  MeasuredArrivalEvent event;
  ASSERT_TRUE(event.accept(key,100000000000LL,100100000000LL,4.,true));
  for(unsigned index=0;index<7;++index) {
    auto other=key;
    if(index==0)other.task="other";
    if(index==1)other.context="other";
    if(index==2)other.map="other";
    if(index==3)other.route="other";
    if(index==4)other.hash="other";
    if(index==5)other.seed="other";
    if(index==6)other.epoch=8;
    EXPECT_FALSE(event.matches(other,true,true));
  }
}

TEST(Cancellation, BoundedErrorNeverConvertsOldPendingIntoReady) {
  EXPECT_FALSE(retirementExpired(true, true, 2.9, 3));
  EXPECT_TRUE(retirementExpired(true, true, 3.1, 3));
  EXPECT_FALSE(retirementExpired(true, false, 100, 3));
  EXPECT_FALSE(retirementExpired(false, true, 100, 3));
}

TEST(GoalReplacement, ReceivingIntentAndStartingWorkAreDifferentBarriers) {
  EXPECT_FALSE(replacementTaskMayTick(false,""));
  EXPECT_TRUE(replacementTaskMayTick(true,""));
  // A late genuine stop must release the SAME waiting target, not require
  // another click after CheckTaskIdentity permanently failed it.
  EXPECT_FALSE(replacementTaskMayTick(false,"previous_physical_stop_unconfirmed"));
  EXPECT_TRUE(replacementTaskMayTick(false,"previous_worker_cancellation_timeout"));
}

TEST(GoalReplacement, FormalPlanningWaitsOnlyForSoftwareButLegacyStillWaitsForStop) {
  const std::string physical="previous_physical_stop_unconfirmed";
  EXPECT_TRUE(planningRetirementReady(true,true,false));
  EXPECT_FALSE(planningRetirementReady(true,false,false));
  EXPECT_FALSE(planningRetirementReady(false,true,false));
  EXPECT_TRUE(planningRetirementReady(false,true,true));
  EXPECT_TRUE(planningRetirementReason(true,physical).empty());
  EXPECT_EQ(planningRetirementReason(false,physical),physical);
  EXPECT_EQ(planningRetirementReason(true,"previous_worker_cancellation_timeout"),
    "previous_worker_cancellation_timeout");
  // Neither readiness nor the filtered planning reason is used for granting
  // execution: the full physical-stop reason remains an admission blocker.
  EXPECT_EQ(taskAdmissionBlocker(true,physical,false,"",""),physical);
}

TEST(GoalReplacement, LateOldCancelCannotApplyToNewerCurrentTask) {
  EXPECT_TRUE(targetedCancelApplies("A","A"));
  EXPECT_FALSE(targetedCancelApplies("A","B"));
  EXPECT_FALSE(targetedCancelApplies("A","C"));
  EXPECT_FALSE(targetedCancelApplies("",""));
}

TEST(GoalReplacement, IntermediateIntentIsReclaimedButRealRetirementIsNot) {
  EXPECT_TRUE(completedTaskReclaimable(false,true,true,false,false,true,false,false));
  EXPECT_FALSE(completedTaskReclaimable(true,true,true,false,false,true,false,false));
  EXPECT_FALSE(completedTaskReclaimable(false,true,false,false,false,true,false,false));
  EXPECT_FALSE(completedTaskReclaimable(false,true,true,true,false,true,false,false));
  EXPECT_FALSE(completedTaskReclaimable(false,true,true,false,true,true,false,false));
  EXPECT_FALSE(completedTaskReclaimable(false,true,true,false,false,false,false,false));
  EXPECT_FALSE(completedTaskReclaimable(false,true,true,false,false,true,true,false));
  EXPECT_TRUE(completedTaskReclaimable(false,true,true,false,false,true,true,true));
}

TEST(PublicAction, CancelWaitsForWorkerTerminalNotJustCancelAck) {
  EXPECT_EQ(publicCompletion(true, true, false, false, false, true), PublicCompletion::None);
  EXPECT_EQ(publicCompletion(true, false, false, false, false, true), PublicCompletion::Canceled);
}

TEST(PublicAction, CancelTimeoutAbortsInsteadOfReportingCanceled) {
  EXPECT_EQ(publicCompletion(true, true, true, false, false, true), PublicCompletion::Aborted);
  EXPECT_EQ(publicCompletion(true, false, false, true, false, true), PublicCompletion::Aborted);
}

TEST(PublicAction, PreemptionRetainsOldTaskUntilChildIsRetired) {
  EXPECT_EQ(publicCompletion(true, true, false, false, false, false), PublicCompletion::None);
  EXPECT_EQ(publicCompletion(true, false, false, false, false, false), PublicCompletion::Aborted);
}

TEST(PublicAction, SuccessCannotPrecedeWorkerResult) {
  EXPECT_EQ(publicCompletion(false, false, false, false, true, false), PublicCompletion::None);
  EXPECT_EQ(publicCompletion(true, true, false, false, true, false), PublicCompletion::None);
  EXPECT_EQ(publicCompletion(true, false, false, false, true, false), PublicCompletion::Succeeded);
}

TEST(Diagnostics, RunningActionsExposeActualFeedbackAndTerminalReasonIsExact) {
  EXPECT_EQ(operationExplanation(true, "not_requested", "waiting_local_map", "following"),
    "waiting_local_map");
  EXPECT_EQ(operationExplanation(true, "not_requested", "", "computing_route"), "computing_route");
  EXPECT_EQ(operationExplanation(false, "no_route", "waiting_local_map", "following"), "no_route");
  EXPECT_EQ(operationExplanation(false, "measured_arrival", "previous_feedback", "following"),
    "measured_arrival");
}

TEST(Diagnostics, IntentionalCancellationIsNotReportedAsPlannerFailure) {
  EXPECT_EQ(terminalPhase(false, true, false), "canceled");
  EXPECT_EQ(terminalPhase(false, false, false), "failed");
  EXPECT_EQ(terminalPhase(false, true, true), "failed");
  EXPECT_EQ(terminalPhase(true, false, false), "succeeded");
}
