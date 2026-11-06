#pragma once

#include <cmath>
#include <cstdint>
#include <cctype>
#include <optional>
#include <string>

namespace d1max_navigation_bt {

// A diagnostic heartbeat may still name the last seed during recovery. Only
// current verified global capability may bind a NEW task. An unbound user's
// intent can wait for recovery without owning a route or any motion authority;
// the same loss on a bound task must revoke that old identity.
inline bool initialTaskIdentityReady(bool report_fresh,bool identity_valid,
    bool global_planning_ready,bool hard_fault) {
  return report_fresh&&identity_valid&&global_planning_ready&&!hard_fault;
}
inline std::string taskLocalizationFault(bool bound,bool health_seen,
    bool hard_fault,const std::string& reason) {
  return bound&&health_seen&&hard_fault?
    (reason.empty()?"hard_localization_lost":reason):std::string{};
}

// Geometry preparation and movement authority are separate capabilities. The
// live planning-only profile uses the real schema-3 commit owner, but cannot
// create an SDK grant client or authorize a confirmation.
inline bool validExecutionPurpose(const std::string& purpose,bool preview,const std::string& transport) {
  return purpose=="execution" || (purpose=="planning_only"&&preview&&transport=="live");
}
inline bool usesGeometryOwner(const std::string& purpose,bool preview) {
  return purpose=="planning_only" || !preview;
}
inline bool permitsMotionAuthority(const std::string& purpose,bool preview) {
  return purpose=="execution"&&!preview;
}
inline std::string executionConfirmationBlocker(const std::string& purpose) {
  return purpose=="planning_only"?"planning_only_execution_disabled":"";
}

// A global route is guidance, not the current collision verdict. The formal
// task follower consumes the local executor's accepted SCAN trajectory; a
// conditioned planning map must not veto it again through a whole-route flag.
// Keep the historical curve-writer contract unchanged.
inline bool confirmationGeometryReady(const std::string& architecture,
    bool route_geometry_eligible, bool local_executor_ready) {
  if(architecture=="task_follower_v1")return local_executor_ready;
  return architecture=="curve_writer_v3"&&route_geometry_eligible;
}

// A global-route task owns only ComputeRoute. Installed controller choice is
// independent of task scope: no fictitious local owner or SDK retirement may
// be created for a graph that never dispatches FollowRoute.
inline bool validPlanningScope(const std::string& scope,const std::string& purpose,
    bool preview,const std::string& transport) {
  return scope=="navigation" || (scope=="global_route"&&purpose=="planning_only"&&preview&&transport=="live");
}
inline bool usesLocalExecutionAuthority(const std::string& scope) {
  return scope=="navigation";
}

inline bool validSessionId(const std::string& session) {
  if (session.empty() || session.size() > 96) return false;
  for (const unsigned char ch : session) {
    if (ch > 127 || (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.')) return false;
  }
  return true;
}

inline bool validGoalFrame(const std::string& kind, const std::string& frame,
    const std::string& map_frame, const std::string& planning_frame) {
  return (kind == "2d" && frame == map_frame) ||
    (kind == "3d" && (frame == map_frame || frame == planning_frame));
}

// Transport-independent bookkeeping used by the actual ROS callbacks. It is
// deliberately not a planner, alternate map or a second task state machine.
struct AsyncRequestState {
  bool sent{false};
  bool terminal{false};
  bool cancel_sent{false};
  bool cancel_error{false};

  bool pending() const { return sent && !terminal; }
  bool begin() {
    if (sent) return false;
    sent = true;
    return true;
  }
  bool acceptTerminal(bool retired) {
    if (!sent || terminal) return false;
    terminal = true;
    return !retired;
  }
  bool shouldCancel(bool has_handle) const {
    return pending() && has_handle && !cancel_sent;
  }
};

// Bookkeeping for a single explicitly authorized SDK admission episode. It is
// not control ownership: only the writer's exact definitely-not-admitted reply
// permits another RPC. Keep the first monotonic start across preparation and
// discovery changes, and never turn periodic retries into renewed budgets.
class GrantAdmissionWait {
public:
  bool start(double now) {
    if(!std::isfinite(now)||now<0)return false;
    if(!started_)started_=now;
    return now>=*started_;
  }
  bool attempted(double now) {
    if(!start(now)||!retryDue(now))return false;
    attempted_=now;return true;
  }
  bool retryDue(double now) const {
    return std::isfinite(now)&&(!started_||now>=*started_)&&
      (!attempted_||now-*attempted_>=.5);
  }
  bool expired(double now,double budget) const {
    return started_&&(!std::isfinite(now)||!std::isfinite(budget)||budget<=0||
      now<*started_||now-*started_>=budget);
  }
  double age(double now) const {return started_?now-*started_:0.;}
  bool pending() const {return started_.has_value();}
  bool admitted(double now,double budget) {
    // Called only after an exact writer RPC/status admission fact. Arrival of
    // that fact cannot win against an elapsed original wait deadline.
    if(expired(now,budget))return false;
    reset();return true;
  }
  void reset(){started_.reset();attempted_.reset();}
private:
  std::optional<double>started_,attempted_;
};

inline bool initialPreparationBudgetApplies(bool owner_confirmed,bool user_authorized,bool sdk_admission_pending) {
  // Successful preparation is already required to record this authorization.
  // A never-installed SDK attempt rolling back does not restart that initial
  // phase; its discovery/telemetry episode has one separate policy budget.
  return !owner_confirmed&&!(user_authorized&&sdk_admission_pending);
}

// Discovery is only an admission check. Once an action is in flight its own
// response/feedback/result is authoritative; unrelated server discovery must
// not interrupt a committed route's local tracking.
inline std::string plannerAdmissionWait(bool compute_sent, bool compute_succeeded,
    bool follow_sent, bool compute_available, bool follow_available) {
  if (!compute_sent && !compute_available) return "waiting_compute_route_action";
  if (compute_succeeded && !follow_sent && !follow_available) return "waiting_follow_route_action";
  return {};
}

inline std::string actionTransportTimeout(bool pending, bool accepted,
    double request_age, double feedback_age, double timeout) {
  if (!pending) return {};
  if (!std::isfinite(timeout) || timeout <= 0) return "action_watchdog_invalid";
  const double age = accepted ? feedback_age : request_age;
  if (!std::isfinite(age) || age < 0 || age >= timeout) {
    return accepted ? "action_feedback_timeout" : "action_acceptance_timeout";
  }
  return {};
}

inline bool routeDeadlineExpired(bool pending,double request_age,double deadline) {
  return pending&&(!std::isfinite(request_age)||request_age<0||
    !std::isfinite(deadline)||deadline<=0||request_age>=deadline);
}

inline bool unresolvedRetirement(bool retired, bool pending, const std::string& error) {
  // An abort saying that cleanup failed is not equivalent to a proven stop.
  // Keep the session quarantined even when the Action result has arrived.
  return retired && (pending || !error.empty());
}

// Native task following separates geometry preparation from SDK execution.
// New geometry may use retired workers while the old writer still waits for
// measured stopping. Legacy curve ownership keeps its original full barrier.
inline bool planningRetirementReady(bool task_follower, bool software_retired, bool fully_retired) {
  return task_follower ? software_retired : fully_retired;
}
inline std::string planningRetirementReason(bool task_follower, const std::string& retirement_fault) {
  return task_follower && retirement_fault=="previous_physical_stop_unconfirmed" ?
    std::string{} : retirement_fault;
}

// The caller supplies the barrier for its operation: planning uses the helpers
// above, while execution always retains physical stopping. Do not spend the
// preparation budget while the old workers are actually still retiring.
// Worker retirement faults must reach the tree's fault guard.
inline bool replacementTaskMayTick(bool previous_retired, const std::string& retirement_fault) {
  return previous_retired || (!retirement_fault.empty() &&
    retirement_fault!="previous_physical_stop_unconfirmed");
}

// Action cancellation is targeted. In particular, a cancel accepted for A and
// dispatched after B/C arrived must not become a cancel-all of the new task.
inline bool targetedCancelApplies(const std::string& requested_task, const std::string& current_task) {
  return !requested_task.empty() && requested_task == current_task;
}

inline bool completedTaskReclaimable(bool current, bool finished, bool public_terminal,
    bool worker_pending, bool worker_error, bool execution_software_retired,
    bool physical_stop_required, bool measured_stop) {
  return !current && finished && public_terminal && !worker_pending && !worker_error &&
    execution_software_retired && (!physical_stop_required || measured_stop);
}

inline bool retirementResultUnconfirmed(bool succeeded, bool canceled, bool retirement_confirmed) {
  // A normal, acknowledged planner failure is not a transport quarantine.
  // Conversely an Action ABORT alone does not prove that its native worker
  // acknowledged cancellation; that is an explicit adapter contract field.
  return !succeeded && !canceled && !retirement_confirmed;
}

inline bool reportFresh(double wall_age, double receipt_age, double timeout) {
  return std::isfinite(wall_age) && std::isfinite(receipt_age) &&
    std::isfinite(timeout) && timeout > 0 && wall_age >= -0.1 &&
    wall_age <= timeout && receipt_age >= 0 && receipt_age <= timeout;
}

// Completion is a source-time-validated EVENT, not a renewable state lease.
// FollowRoute emits it only after measured arrival/stop. Validate its age on
// receipt, then retain the original identity and timestamp through a BT pause.
// Reusing it for another task, route, seed or localization epoch is forbidden.
struct ArrivalIdentity {
  std::string task,context,map,route,hash,seed;
  std::int64_t epoch{0};
  bool operator==(const ArrivalIdentity& other) const {
    return task==other.task&&context==other.context&&map==other.map&&route==other.route&&
      hash==other.hash&&seed==other.seed&&epoch==other.epoch;
  }
};
class MeasuredArrivalEvent {
public:
  bool accept(const ArrivalIdentity& identity,std::int64_t source_ns,std::int64_t received_ns,
      double timeout,bool active) {
    if(event_||!active||identity.task.empty()||identity.context.empty()||identity.map.empty()||
       identity.route.empty()||identity.hash.empty()||identity.seed.empty()||identity.epoch<0||
       source_ns<=0||!reportFresh((received_ns-source_ns)*1e-9,0.,timeout))return false;
    event_=identity;source_ns_=source_ns;return true;
  }
  bool matches(const ArrivalIdentity& identity,bool active,bool measured_stop) const {
    return active&&measured_stop&&event_&&*event_==identity;
  }
  std::int64_t sourceStamp() const{return source_ns_;}
private:
  std::optional<ArrivalIdentity>event_;
  std::int64_t source_ns_{0};
};

// Arrival is evaluated in the committed route's continuous local execution
// domain. A coarse/global startup capability is not an additional final-stop
// prerequisite once that route has been bound. Legacy visualization backends
// without the typed geometry owner retain their original global capability.
// This does not replace the source-bound arrival/physical-stop facts.
inline bool measuredArrivalCapabilityReady(bool typed_geometry, bool local_task_ready,
    bool local_control_ready, bool global_startup_ready) {
  return typed_geometry ? local_task_ready && local_control_ready : global_startup_ready;
}

inline bool retirementExpired(bool retired, bool pending, double age, double timeout) {
  return retired && pending && (!std::isfinite(age) || age > timeout);
}

enum class PublicCompletion { None, Succeeded, Canceled, Aborted };

inline PublicCompletion publicCompletion(bool finished, bool worker_pending,
    bool quarantined, bool worker_error, bool success, bool action_canceling) {
  if (!finished || (worker_pending && !quarantined)) return PublicCompletion::None;
  if (quarantined || worker_error) return PublicCompletion::Aborted;
  if (action_canceling) return PublicCompletion::Canceled;
  return success ? PublicCompletion::Succeeded : PublicCompletion::Aborted;
}

inline std::string operationExplanation(bool running, const std::string& operation_reason,
    const std::string& feedback_reason, const std::string& phase) {
  if (!running) return operation_reason;
  if (!feedback_reason.empty()) return feedback_reason;
  return phase.empty() ? operation_reason : phase;
}

inline std::string terminalPhase(bool success, bool cancellation_requested, bool retirement_fault) {
  if (retirement_fault) return "failed";
  if (success) return "succeeded";
  return cancellation_requested ? "canceled" : "failed";
}

}  // namespace d1max_navigation_bt
