#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <stdexcept>

namespace d1max_navigation_bt {
inline constexpr std::uint32_t task_schema_version = 2;

inline std::string transitionAdmission(std::uint32_t schema,const std::string& requested_session,
                                       const std::string& current_session) {
  if(schema!=task_schema_version) return "unsupported_schema_version";
  if(requested_session.empty() || requested_session!=current_session) return "transition_session_mismatch";
  return {};
}

// Shared source-time admission for ROS action and topic goals. Acceptance and
// accepted-callback submission are separate events; a cancel/pose reset/drain
// between them must invalidate a previously accepted but undispatched request.
class GoalAdmission {
public:
  explicit GoalAdmission(std::int64_t max_age_ns=5000000000LL) : max_age_(max_age_ns) {
    if (max_age_ns<=0 || max_age_ns>30000000000LL) throw std::invalid_argument("invalid_goal_source_timeout");
  }
  std::string reserve(std::int64_t source, std::int64_t now) {
    const auto reason=check(source,now);
    if (!reason.empty()) return reason;
    if (source<=reserved_) return "goal_source_replayed_or_reordered";
    reserved_=source;
    return {};
  }
  std::string commit(std::int64_t source, std::int64_t now) {
    const auto reason=check(source,now);
    if (!reason.empty()) return reason;
    if (source!=reserved_ || source<=committed_) return "goal_request_superseded_before_submission";
    committed_=source;
    return {};
  }
  void fenceAll(std::int64_t now) {
    clock_high_water_=std::max(clock_high_water_,now);
    barrier_=std::max({barrier_,now,reserved_,committed_});
  }
  // A targeted action cancel is not a cancel-all. A newer accepted request
  // retains its own admission while the older task drains by task identity.
  void fenceThrough(std::int64_t source) { barrier_=std::max(barrier_,source); }
  std::int64_t barrier() const { return barrier_; }
private:
  std::string check(std::int64_t source, std::int64_t now) {
    if (now<=0 || source<=0) return "goal_source_stamp_missing";
    if (now<clock_high_water_) return "goal_admission_clock_rollback";
    clock_high_water_=now;
    if (source<=barrier_) return "goal_source_precedes_admission_barrier";
    if (source-now>50000000LL) return "goal_source_stamp_in_future";
    if (now-source>max_age_) return "goal_source_stamp_stale";
    return {};
  }
  std::int64_t max_age_, clock_high_water_{0}, barrier_{0}, reserved_{0}, committed_{0};
};

struct RouteBinding {
  std::string session, task, route, hash;
  bool operator==(const RouteBinding& other) const {
    return session == other.session && task == other.task && route == other.route && hash == other.hash;
  }
};

struct ConfirmationDecision {
  bool accepted{false}, authorized{false}, duplicate{false};
  std::string confirmation_id, reason;
};

// ROS-independent authorization contract. A remembered rejected request is NOT
// an execution lease. The preview runtime always supplies physical_available=false.
class ConfirmationLedger {
public:
  ConfirmationDecision confirm(std::uint32_t schema, const std::string& request,
      const RouteBinding& asked, const RouteBinding& current, bool active,
      bool context_ready, bool eligible, bool preview, bool physical_available) {
    if (schema != task_schema_version) return denied("unsupported_schema_version");
    if (request.empty() || request.size() > 128) return denied("invalid_request_id");
    const auto prior = entries_.find(request);
    if (prior != entries_.end() && !(prior->second.binding == asked)) return denied("request_id_binding_conflict");
    if (!(asked == current) || asked.task.empty() || asked.route.empty() || asked.hash.empty())
      return denied("route_snapshot_mismatch");
    if (!active) return denied("task_not_active");
    if (!context_ready) return denied("navigation_context_not_ready");
    if (preview || !physical_available) return remember(request, asked, denied("physical_execution_not_implemented_preview_only"));
    if (!eligible) return remember(request, asked, denied("route_not_execution_eligible"));
    if (revoked_) return denied("execution_confirmation_revoked");
    if (prior != entries_.end() && prior->second.decision.accepted) {
      auto result = prior->second.decision;
      result.duplicate = true;
      return result;
    }
    // One immutable route may consume only one confirmation, regardless of RPC
    // retries. This is an admission token, never an SDK control command.
    if (!confirmation_id_.empty()) return denied("route_already_confirmed");
    const auto identifier=asked.task + ":" + request;
    auto result=remember(request, asked, {true, true, false, identifier, "execution_confirmed"});
    // Capacity/commit failure must not consume an authorization identity.
    if(result.authorized)confirmation_id_=identifier;
    return result;
  }
  void revoke() { revoked_ = true; }
  bool authorized() const { return !revoked_ && !confirmation_id_.empty(); }
private:
  struct Entry { RouteBinding binding; ConfirmationDecision decision; };
  static ConfirmationDecision denied(const std::string& reason) { return {false, false, false, "", reason}; }
  ConfirmationDecision remember(const std::string& request, const RouteBinding& binding,
                               ConfirmationDecision decision) {
    const auto old = entries_.find(request);
    decision.duplicate = old != entries_.end() && old->second.decision.reason == decision.reason;
    if (old == entries_.end() && entries_.size() >= 256) return denied("confirmation_request_capacity_exceeded");
    entries_[request] = {binding, decision};
    return decision;
  }
  std::unordered_map<std::string, Entry> entries_;
  std::string confirmation_id_;
  bool revoked_{false};
};

// A lifecycle callback starts draining and returns; executor ticks and action
// replies finish it. No synchronous wait and no communication teardown while pending.
class LifecycleDrain {
public:
  bool configure() {
    if (draining_ || quarantined_) return false;
    configured_ = true; return true;
  }
  bool activate() {
    if (!configured_ || draining_ || quarantined_) return false;
    accepting_ = true; return true;
  }
  void begin() { accepting_ = false; draining_ = true; }
  void observe(bool software_retired, bool failed, bool physical_required=false, bool stopped=false) {
    if (!draining_) return;
    if (failed) quarantined_ = true;
    if (software_retired && !failed && (!physical_required || stopped)) draining_ = false;
  }
  bool cleanup() {
    if (accepting_ || draining_ || quarantined_) return false;
    configured_ = false; return true;
  }
  bool accepting() const { return accepting_; }
  bool draining() const { return draining_; }
  bool quarantined() const { return quarantined_; }
private:
  bool configured_{false}, accepting_{false}, draining_{false}, quarantined_{false};
};

struct StopObservation {
  std::string task, mode_epoch;
  double source_time{0}, received_time{0}, vx{0}, vy{0}, yaw_rate{0};
};

// Physical stop proof is separate from worker retirement. No caller may turn a
// cancel ACK, sent zero, displayed stationary pose or elapsed time into proof.
class StopVerifier {
public:
  void begin(std::string task, std::string mode_epoch, double requested_at) {
    task_ = std::move(task); mode_ = std::move(mode_epoch); requested_ = requested_at;
    first_ = last_ = -1; samples_ = 0;
  }
  bool observe(const StopObservation& o, double now) {
    if (!std::isfinite(now) || !std::isfinite(o.source_time) || !std::isfinite(o.received_time) ||
        !std::isfinite(o.vx) || !std::isfinite(o.vy) || !std::isfinite(o.yaw_rate) ||
        task_.empty() || mode_.empty() || o.task != task_ || o.mode_epoch != mode_ ||
        o.source_time <= requested_ || o.source_time <= last_ ||
        now < o.source_time || now-o.source_time > .25 ||
        now < o.received_time || now-o.received_time > .25) return false;
    if ((last_ >= 0 && o.source_time-last_ > .25) || std::hypot(o.vx,o.vy) > .03 || std::abs(o.yaw_rate) > .03) {
      first_ = -1; samples_ = 0;
    }
    last_ = o.source_time;
    if (std::hypot(o.vx,o.vy) > .03 || std::abs(o.yaw_rate) > .03) return false;
    if (first_ < 0) first_ = o.source_time;
    ++samples_;
    return confirmed(now);
  }
  bool confirmed(double now) const {
    return std::isfinite(now) && samples_ >= 3 && last_-first_ >= .3 && now >= last_ && now-last_ <= .25;
  }
private:
  std::string task_, mode_;
  double requested_{0}, first_{-1}, last_{-1};
  unsigned samples_{0};
};
}  // namespace d1max_navigation_bt
