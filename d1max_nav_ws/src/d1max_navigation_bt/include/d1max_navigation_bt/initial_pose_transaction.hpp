#pragma once
#include <d1max_navigation_bt_interfaces/srv/prepare_initial_pose.hpp>
#include <d1max_navigation_bt_interfaces/msg/initial_pose_outcome.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace d1max_navigation_bt {
// Admission/retirement handshake inside the existing task owner. This owns no
// goal, estimator, SDK connection or motion grant. An authorized application is
// kept closed until the exact localization-owner outcome AND new ready identity.
class InitialPoseTransaction {
public:
  using Service=d1max_navigation_bt_interfaces::srv::PrepareInitialPose;
  using Request=Service::Request;
  using Outcome=d1max_navigation_bt_interfaces::msg::InitialPoseOutcome;
  struct Decision {
    bool accepted=false,ready=false,application_authorized=false;
    bool physical_stop_confirmed=false,begin_retirement=false;
    std::string reason;
  };
  explicit InitialPoseTransaction(std::string session,std::string frame,std::string map_sha)
    :session_(std::move(session)),frame_(std::move(frame)),map_sha_(std::move(map_sha)) {}

  Decision request(const Request&r,int64_t now_ns,double monotonic,bool active,
      bool retired,bool physical_required,bool physical_stopped,bool live,
      uint64_t epoch,const std::string&seed) {
    expire(monotonic);
    if(r.schema_version!=1)return denied("unsupported_initial_pose_transaction_schema");
    if(r.session_id!=session_||map_sha_.size()!=64||r.source_map_sha256!=map_sha_)
      return denied("initial_pose_context_mismatch");
    if(!identifier(r.request_id))return denied("initial_pose_request_id_invalid");
    if(r.operation!=Request::PREPARE&&r.operation!=Request::COMMIT&&r.operation!=Request::ABORT)
      return denied("initial_pose_operation_invalid");
    if(!std::isfinite(monotonic)||!std::isfinite(now_ns*1e-9)||now_ns<=0)
      return denied("initial_pose_clock_invalid");
    if(pending_) {
      if(r.request_id!=pending_->request.request_id)return denied("initial_pose_transaction_busy");
      if(!sameIntent(r,pending_->request))return denied("initial_pose_request_binding_conflict");
      if(r.operation==Request::ABORT) {
        if(pending_->authorized)return denied("initial_pose_application_outcome_unknown");
        remember(r.request_id,"initial_pose_request_aborted");pending_.reset();
        return {true,false,false,false,false,"initial_pose_request_aborted"};
      }
      if(pending_->failed)return denied(pending_->reason);
      pending_->ready=retired&&(!pending_->physical_required||physical_stopped);
      if(r.operation==Request::COMMIT) {
        if(!pending_->ready)return denied("initial_pose_retirement_pending");
        if(!pending_->authorized) {
          pending_->authorized=true;pending_->authorized_at=monotonic;
          pending_->authorized_ns=now_ns;
        }
      }
      return {true,pending_->ready,pending_->authorized,
        pending_->physical_required&&physical_stopped&&live,false,
        pending_->authorized?"initial_pose_application_authorized_waiting_matching_outcome":
          pending_->ready?"initial_pose_retired_ready_to_apply":"initial_pose_retirement_pending"};
    }
    for(const auto&old:history_)if(old.first==r.request_id)return denied(old.second);
    if(r.operation!=Request::PREPARE)return denied("initial_pose_request_not_prepared");
    if(!active)return denied("navigation_lifecycle_not_active");
    if(history_.size()>=128)return denied("initial_pose_request_capacity_requires_new_session");
    const auto problem=validatePose(r.body_pose,now_ns,frame_);
    if(!problem.empty())return denied(problem);
    Pending p;p.request=r;p.started=monotonic;p.physical_required=physical_required;
    p.baseline_epoch=epoch;p.baseline_seed=seed;
    p.ready=retired&&(!physical_required||physical_stopped);
    pending_=std::move(p);
    return {true,pending_->ready,false,physical_required&&physical_stopped&&live,true,
      pending_->ready?"initial_pose_retired_ready_to_apply":"initial_pose_retirement_pending"};
  }

  bool outcome(const Outcome&o,int64_t now_ns,double monotonic) {
    expire(monotonic);
    if(!pending_||pending_->failed||!pending_->authorized||o.schema_version!=1||
        o.session_id!=session_||o.request_id!=pending_->request.request_id||
        o.intent_source_stamp!=pending_->request.body_pose.header.stamp||
        o.source_stamp.sec<0||o.source_stamp.nanosec>=1000000000u||
        ns(o.source_stamp)<pending_->authorized_ns||
        !fresh(ns(o.source_stamp),now_ns,.6)||o.map_version_id.empty())return false;
    if(pending_->applied) {
      return o.accepted&&o.applied&&o.localization_epoch==pending_->applied_epoch&&
        o.localization_seed_id==pending_->applied_seed&&o.map_version_id==pending_->map_version;
    }
    if(!o.accepted&&!o.applied) {
      // Exact rejection is a known unapplied outcome. No old grant is restored.
      remember(o.request_id,"initial_pose_application_rejected:"+o.reason);pending_.reset();return true;
    }
    if(!o.accepted||!o.applied) {
      pending_->failed=true;pending_->reason="initial_pose_partial_application_requires_review";return false;
    }
    if(o.localization_epoch==0||o.localization_seed_id.empty()||
        (o.localization_epoch==pending_->baseline_epoch&&o.localization_seed_id==pending_->baseline_seed))return false;
    pending_->applied=true;pending_->failed=false;pending_->reason.clear();
    pending_->applied_epoch=o.localization_epoch;pending_->applied_seed=o.localization_seed_id;
    pending_->applied_ns=ns(o.source_stamp);pending_->map_version=o.map_version_id;
    return true;
  }

  bool readyIdentity(uint64_t epoch,const std::string&seed,const std::string&map_version,
      int64_t source_ns,int64_t now_ns,double monotonic,bool ready) {
    expire(monotonic);
    if(!pending_||!pending_->applied||pending_->failed)return false;
    if(!ready||
        epoch!=pending_->applied_epoch||seed!=pending_->applied_seed||map_version!=pending_->map_version||
        source_ns<pending_->applied_ns||!fresh(source_ns,now_ns,.4)) {
      pending_->ready_since.reset();pending_->ready_samples=0;return false;
    }
    if(source_ns>pending_->last_ready_source) {
      pending_->last_ready_source=source_ns;++pending_->ready_samples;
      if(!pending_->ready_since)pending_->ready_since=monotonic;
    }
    if(pending_->ready_since&&monotonic-*pending_->ready_since>=.6&&pending_->ready_samples>=3) {
      remember(pending_->request.request_id,"initial_pose_applied_new_identity_ready");
      pending_.reset();return true;
    }
    return false;
  }
  void unavailable(double monotonic) {
    expire(monotonic);
    if(pending_){pending_->ready_since.reset();pending_->ready_samples=0;}
  }
  bool blocked()const{return pending_.has_value();}
  std::string blocker()const {
    if(!pending_)return {};
    return pending_->failed?pending_->reason:pending_->authorized?
      "initial_pose_awaiting_matching_localization":"initial_pose_retirement_pending";
  }
  void expire(double monotonic) {
    if(!pending_||pending_->failed)return;
    if(!std::isfinite(monotonic)||monotonic<pending_->started) {
      pending_->failed=true;pending_->reason="initial_pose_clock_invalid";return;
    }
    if(!pending_->authorized&&monotonic-pending_->started>=10.) {
      remember(pending_->request.request_id,"initial_pose_prepare_timeout");pending_.reset();return;
    }
    if(pending_->authorized&&monotonic-pending_->authorized_at>=30.) {
      pending_->failed=true;pending_->reason="initial_pose_matching_outcome_or_readiness_timeout_requires_review";
    }
  }

  static std::string validatePose(const geometry_msgs::msg::PoseWithCovarianceStamped&p,
      int64_t now_ns,const std::string&frame) {
    const auto&v=p.pose.pose.position;const auto&q=p.pose.pose.orientation;
    if(p.header.frame_id!=frame)return "initial_pose_frame_mismatch";
    if(p.header.stamp.sec<0||p.header.stamp.nanosec>=1000000000u||!fresh(ns(p.header.stamp),now_ns,2.))
      return "initial_pose_source_stamp_invalid";
    if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z)||
        std::max(std::abs(v.x),std::abs(v.y))>10000.||std::abs(v.z)>100.)return "initial_pose_position_invalid";
    const double n=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
    if(!std::isfinite(n)||std::abs(n-1.)>.01)return "initial_pose_orientation_invalid";
    for(const auto value:p.pose.covariance)if(!std::isfinite(value))return "initial_pose_covariance_invalid";
    for(int i=0;i<6;++i)if(p.pose.covariance[i*7]<0.)return "initial_pose_covariance_invalid";
    return {};
  }
private:
  struct Pending {
    Request request;double started=0.,authorized_at=0.;
    int64_t authorized_ns=0,applied_ns=0,last_ready_source=0;
    bool physical_required=false,ready=false,authorized=false,applied=false,failed=false;
    uint64_t baseline_epoch=0,applied_epoch=0;unsigned ready_samples=0;
    std::string baseline_seed,applied_seed,map_version,reason;
    std::optional<double>ready_since;
  };
  static int64_t ns(const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
  static bool fresh(int64_t source,int64_t now,double age) {
    return source>0&&now>0&&source-now<=50000000LL&&now-source<=int64_t(age*1e9);
  }
  static bool identifier(const std::string&id) {
    if(id.empty()||id.size()>128)return false;
    for(unsigned char c:id)if(c>127||(!std::isalnum(c)&&c!='-'&&c!='_'&&c!='.'))return false;
    return true;
  }
  static bool sameIntent(const Request&a,const Request&b) {
    return a.session_id==b.session_id&&a.request_id==b.request_id&&a.source_map_sha256==b.source_map_sha256&&a.body_pose==b.body_pose;
  }
  static Decision denied(const std::string&reason){Decision d;d.reason=reason;return d;}
  void remember(const std::string&id,const std::string&reason){history_.emplace_back(id,reason);}
  std::string session_,frame_,map_sha_;
  std::optional<Pending>pending_;
  std::vector<std::pair<std::string,std::string>>history_;
};
} // namespace d1max_navigation_bt
