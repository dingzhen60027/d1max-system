#pragma once

#include <cmath>
#include <cctype>
#include <string>

namespace d1max_navigation_bt {

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
