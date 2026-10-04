#include "d1max_navigation_bt/runtime_policy.hpp"
#include "d1max_navigation_bt/stable_recovery.hpp"

#include <gtest/gtest.h>
#include <limits>

using namespace d1max_navigation_bt;

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
  // Nor can cached proof be resurrected after a long pause.
  EXPECT_FALSE(reportFresh(4.1, 4.1, 4.0));
  EXPECT_FALSE(reportFresh(-1.0, 0.0, 4.0));
}

TEST(Cancellation, BoundedErrorNeverConvertsOldPendingIntoReady) {
  EXPECT_FALSE(retirementExpired(true, true, 2.9, 3));
  EXPECT_TRUE(retirementExpired(true, true, 3.1, 3));
  EXPECT_FALSE(retirementExpired(true, false, 100, 3));
  EXPECT_FALSE(retirementExpired(false, true, 100, 3));
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
