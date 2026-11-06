#include "d1max_trajectory_tracker/local_follower.hpp"
#include "d1max_trajectory_tracker/navigation_state_contract.hpp"
#include "d1max_trajectory_tracker/navigation_state_diagnostics.hpp"
#include "d1max_trajectory_tracker/local_commit_dispatch.hpp"
#include "d1max_trajectory_tracker/command_timing_diagnostics.hpp"
#include "d1max_trajectory_tracker/control_trace_diagnostics.hpp"
#include <d1max_planning_interfaces/bounded_diagnostic_file.hpp>
#include <rclcpp/rclcpp.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace d1max_trajectory_tracker {
class LocalFollowerNode final:public rclcpp::Node {
 public:
  LocalFollowerNode():Node("trajectory_tracker") {
    Config c;c.session_id=declare_parameter<std::string>("session_id","");
    c.map_version_id=declare_parameter<std::string>("map_version_id","");
    c.map_frame=declare_parameter<std::string>("map_frame",c.map_frame);
    c.planning_frame=declare_parameter<std::string>("planning_frame",c.planning_frame);
    c.base_frame=declare_parameter<std::string>("base_frame",c.base_frame);
    c.trajectory_geometry_policy=declare_parameter<std::string>("trajectory_geometry_policy",c.trajectory_geometry_policy);
    c.motion_model=declare_parameter<std::string>("motion_model",c.motion_model);
    c.holonomic_yaw_policy=declare_parameter<std::string>("holonomic_yaw_policy",c.holonomic_yaw_policy);
    c.align_then_forward=declare_parameter<bool>("align_then_forward",c.align_then_forward);
    c.require_body_heading_seed=declare_parameter<bool>("require_body_heading_seed",true);
    c.max_speed=declare_parameter<double>("max_speed",c.max_speed);
    c.max_yaw_rate=declare_parameter<double>("max_yaw_rate",c.max_yaw_rate);
    c.max_acceleration=declare_parameter<double>("max_acceleration",c.max_acceleration);
    c.max_yaw_acceleration=declare_parameter<double>("max_yaw_acceleration",c.max_yaw_acceleration);
    c.task_timeout=declare_parameter<double>("task_timeout",c.task_timeout);
    c.odom_timeout=declare_parameter<double>("odom_timeout",c.odom_timeout);
    c.trajectory_timeout=declare_parameter<double>("trajectory_timeout",c.trajectory_timeout);
    c.lookahead=declare_parameter<double>("lookahead",c.lookahead);
    c.min_lookahead_distance_m=declare_parameter<double>("min_lookahead_distance_m",c.min_lookahead_distance_m);
    c.max_lookahead_distance_m=declare_parameter<double>("max_lookahead_distance_m",c.max_lookahead_distance_m);
    c.kp_position=declare_parameter<double>("kp_position",c.kp_position);c.kp_yaw=declare_parameter<double>("kp_yaw",c.kp_yaw);
    c.heading_threshold=declare_parameter<double>("heading_threshold",c.heading_threshold);
    c.steering_enter_angle_rad=declare_parameter<double>("steering_enter_angle_rad",c.steering_enter_angle_rad);
    c.steering_exit_angle_rad=declare_parameter<double>("steering_exit_angle_rad",c.steering_exit_angle_rad);
    c.steering_error_scale_rad=declare_parameter<double>("steering_error_scale_rad",c.steering_error_scale_rad);
    c.velocity_damping_gain=declare_parameter<double>("velocity_damping_gain",c.velocity_damping_gain);
    c.yaw_damping_gain=declare_parameter<double>("yaw_damping_gain",c.yaw_damping_gain);
    c.max_jerk=declare_parameter<double>("max_jerk",c.max_jerk);
    c.max_yaw_jerk=declare_parameter<double>("max_yaw_jerk",c.max_yaw_jerk);
    c.max_input_rate=declare_parameter<double>("max_input_rate",c.max_input_rate);
    c.max_input_acceleration=declare_parameter<double>("max_input_acceleration",c.max_input_acceleration);
    if(declare_parameter<bool>("command_response_enabled",false)) {
      const auto response=[this](const std::string& prefix) {
        CommandResponseCurve r;
        r.reference=declare_parameter<double>(prefix+"_reference",0.);
        r.effective_fraction=declare_parameter<double>(prefix+"_effective_fraction",0.);
        r.maximum_fraction=declare_parameter<double>(prefix+"_maximum_fraction",0.);
        r.blend_ratio=declare_parameter<double>(prefix+"_blend_ratio",.1);
        return r;
      };
      c.forward_response=response("forward_response");c.yaw_response=response("yaw_response");
      if(!c.forward_response.enabled()||!c.yaw_response.enabled())
        throw std::invalid_argument("enabled_command_response_requires_both_axes");
    }
    c.goal_tolerance=declare_parameter<double>("goal_tolerance",c.goal_tolerance);
    c.goal_height_tolerance=declare_parameter<double>("goal_height_tolerance",c.goal_height_tolerance);
    c.stationary_velocity_metric=declare_parameter<std::string>("stationary_velocity_metric",c.stationary_velocity_metric);
    c.stationary_linear_threshold_mps=declare_parameter<double>(
      "stationary_linear_threshold_mps",c.stationary_linear_threshold_mps);
    c.stationary_angular_threshold_radps=declare_parameter<double>(
      "stationary_angular_threshold_radps",c.stationary_angular_threshold_radps);
    c.stationary_reentry_duration_s=declare_parameter<double>(
      "stationary_reentry_duration_s",c.stationary_reentry_duration_s);
    const int stationary_samples=declare_parameter<int>("stationary_minimum_samples",3);
    if(stationary_samples<3||stationary_samples>512)
      throw std::invalid_argument("bounded_stationary_minimum_samples_required");
    c.stationary_minimum_samples=static_cast<unsigned>(stationary_samples);
    // LocalFollower fixes Native SCAN as geometry authority in its constructor;
    // this source-record requirement is not a raw-PCD traversability gate.
    c.require_support_reference=true;c.external_goal_completion=true;config_=c;c.validate();
    mode_=declare_parameter<std::string>("transport_mode","live");
    const double hz=declare_parameter<double>("controller_frequency_hz",20.);
    if(!finite(hz)||hz<10.||hz>50.)throw std::invalid_argument("invalid_control_frequency");
    follower_=std::make_unique<LocalFollower>(c,mode_,declare_parameter<std::string>("execution_braking_model_sha256",""),true,
      1./hz,declare_parameter<double>("command_pipeline_bound_s",.1),declare_parameter<double>("writer_period_s",.05));
    const auto topic=[this](const char*name,const char*value){return declare_parameter<std::string>(name,value);};
    const auto qos=rclcpp::QoS(4).reliable();
    const auto durable=rclcpp::QoS(8).reliable().transient_local();
    commit_=create_publisher<wire::LocalTrajectoryCommit>(topic("local_commit_topic","/d1max/live_planning/execution/local_commit"),durable);
    commit_result_=create_subscription<wire::LocalTrajectoryCommit>(
      topic("local_commit_result_topic","/d1max/live_planning/execution/local_commit_result"),durable,
      [this](wire::LocalTrajectoryCommit::ConstSharedPtr result){
        if(follower_->commitResult(*result,now().seconds(),receipt()))pumpGeometryExchange();});
    status_=create_publisher<wire::LocalExecutionStatus>(topic("local_status_topic","/d1max/live_planning/execution/local_status"),durable);
    geometry_=create_publisher<wire::ExecutionPermit>(topic("permit_topic","/d1max/live_planning/execution/permit"),rclcpp::QoS(2));
    demand_=create_publisher<wire::MotionDemand>(topic("demand_topic","/d1max/live_planning/execution/demand_unchecked"),rclcpp::QoS(1));
    progress_=create_publisher<wire::TrackingProgress>(topic("progress_topic","/d1max/live_planning/tracking_progress"),rclcpp::QoS(1));
    diagnostic_=create_publisher<std_msgs::msg::String>(topic("status_topic","/d1max/pct_scan/tracker_status"),rclcpp::QoS(1));
    proposal_=create_subscription<wire::ReferenceProposal>(topic("proposal_topic","/d1max/live_planning/execution/reference_proposal"),durable,
      [this](wire::ReferenceProposal::ConstSharedPtr p){
        if(follower_->proposal(*p,now().seconds()))pumpGeometryExchange();});
    support_=create_subscription<wire::SupportReference>(topic("support_topic","/d1max/live_planning/execution/support"),durable,
      [this](wire::SupportReference::ConstSharedPtr s){if(follower_->support(*s))pumpGeometryExchange();});
    candidate_=create_subscription<wire::TaggedBspline>(topic("trajectory_topic","/d1max/live_planning/scan_tagged_bspline"),durable,
      [this](wire::TaggedBspline::ConstSharedPtr t){if(follower_->candidate(*t))pumpGeometryExchange();});
    validation_=create_subscription<wire::TrajectoryValidation>(topic("validation_topic","/d1max/live_planning/execution/validation"),qos,
      [this](wire::TrajectoryValidation::ConstSharedPtr v){if(follower_->validation(*v))pumpGeometryExchange();});
    lease_=create_subscription<wire::TaskExecutionLease>(topic("task_lease_topic","/d1max/live_planning/execution/task_lease"),qos,
      [this](wire::TaskExecutionLease::ConstSharedPtr l){
        if(follower_->lease(*l,now().seconds()))pumpGeometryExchange();});
    task_feedback_=create_subscription<wire::TaskCommandFeedback>(
      topic("task_command_feedback_topic","/d1max/live_planning/execution/task_command_feedback"),rclcpp::QoS(1).reliable(),
      [this](wire::TaskCommandFeedback::ConstSharedPtr f){follower_->commandFeedback(*f,now().seconds());});
    retirement_=create_subscription<wire::RouteDeliveryReceipt>(topic("route_retirement_topic","/d1max/live_planning/execution/route_delivery_receipt"),durable,
      [this](wire::RouteDeliveryReceipt::ConstSharedPtr r){
        if(follower_->retireRoute(*r,now().seconds()))pumpGeometryExchange();});
    state_=create_subscription<wire::LocalNavigationState>(topic("local_navigation_state_topic","/d1max/localization/navigation/local_state"),rclcpp::QoS(1),
      [this](wire::LocalNavigationState::ConstSharedPtr state) {
        const auto received=now();const double time=received.seconds();auto& core=follower_->controller();
        if(!ingress_.inspect(*state,config_,time,core.task().identity.localization_epoch))return;
        Odom o;std::string reason;
        if(decodeLocalNavigationState(*state,config_,o,reason)) {
          const bool accepted=follower_->body(o,time,receipt());
          state_diagnostics_.record(*state,received.nanoseconds(),true,accepted,
            accepted?reason:core.reason());
          if(accepted)pumpGeometryExchange();
        }
        else {
          state_diagnostics_.record(*state,received.nanoseconds(),false,false,reason);
          if(core.active()&&navigationContextReset(*state,config_,core.task().identity,time))follower_->retire("localization_identity_changed");
          else core.hold(reason,receipt());
        }
      });
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1./hz)),[this]{tick();});
    // The bounded preparation worker can finish between ROS deliveries. Poll
    // only geometry preparation/ACK delivery; do not run control or renew a
    // sensor stamp. Final activation belongs to the next control tick so its
    // first demand is published with it, not one velocity period later.
    geometry_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{pumpGeometryExchange();});
    // Diagnostics must not alter the executor scheduling of the existing
    // mutually-exclusive input/control graph. The passive worker consumes
    // immutable ring copies only; a slow file never owns an executor thread.
    startControlTraceWorker();
  }
  ~LocalFollowerNode() override {stopControlTraceWorker();}
 private:
  void startControlTraceWorker() noexcept {
    try {trace_worker_=std::thread([this]{runControlTraceWorker();});}
    catch(...) {RCLCPP_WARN(get_logger(),"ControlTrace worker unavailable; navigation unchanged");}
  }
  void runControlTraceWorker() noexcept {
    try {
      std::unique_lock<std::mutex> lock(trace_wait_mutex_);
      while(!trace_stop_) {
        if(trace_wakeup_.wait_for(lock,std::chrono::seconds(1),[this]{return trace_stop_;}))break;
        lock.unlock();
        flushControlTrace();
        lock.lock();
      }
      // Orderly node shutdown may arrive before the next one-second wake.
      // Drain the last stop/retirement records on this diagnostic worker,
      // never on the control callback or directly in the destructor.
      lock.unlock();
      flushControlTrace();
    } catch(...) {RCLCPP_WARN(get_logger(),"ControlTrace worker stopped; navigation unchanged");}
  }
  void stopControlTraceWorker() {
    {std::lock_guard<std::mutex> lock(trace_wait_mutex_);trace_stop_=true;}
    trace_wakeup_.notify_all();
    if(trace_worker_.joinable())trace_worker_.join();
  }
  void flushControlTrace() noexcept {
    try {
      auto batch=control_trace_.take(); // Unlock before encoding or disk I/O.
      // Preserve a task boundary even when retirement/new-goal records share
      // one drain. Idle cannot rotate out the movement that preceded it.
      for(std::size_t begin=0;begin<batch.size;) {
        const auto identity=batch.records[begin]->identity;
        const bool active=batch.records[begin]->task_active;
        ControlTraceBuffer::Batch group;
        group.overflow=batch.overflow;group.contention=batch.contention;group.failures=batch.failures;
        std::size_t end=begin;
        for(;end<batch.size;++end) {
          const auto& item=*batch.records[end];
          const bool same=identity&&item.identity&&identity->session==item.identity->session&&
            identity->task==item.identity->task;
          if(end>begin&&(!same||item.task_active!=active))break;
          group.records[group.size++]=std::move(batch.records[end]);
        }
        const auto key=identity?d1max_diagnostics::BoundedTaskDiagnosticFile::taskKey(
          identity->session,identity->task):std::string{};
        const auto event_ns=std::llround(group.records[group.size-1]->tick.current.steady_s*1e9);
        if(!trace_file_.write(ControlTraceBuffer::format(group,now().nanoseconds(),receipt()),key,active,event_ns)) {
          control_trace_.failure();
          if(!trace_failure_reported_) {
            trace_failure_reported_=true;
            RCLCPP_WARN(get_logger(),"ControlTraceFile unavailable; navigation unchanged");
          }
        }
        else if(!trace_path_logged_) {
          trace_path_logged_=true;
          RCLCPP_INFO(get_logger(),"ControlTraceFile directory=%s task_buckets=4 task_mib=4 idle_kib=256",trace_file_.directory().string().c_str());
        }
        begin=end;
      }
    } catch(...) {control_trace_.failure();}
  }
  static std::string traceLabel(const std::string& label){return label.substr(0,160);}
  void recordControlTrace(const wire::MotionDemand& command,const wire::LocalExecutionStatus& status,
      double t,double r,double tick_to_demand_publish_elapsed) noexcept {
    try {
      const auto& v=command.version;
      // Ordinary rolling reference versions share the immutable task context.
      if(!trace_identity_||trace_identity_->session!=v.session_id||trace_identity_->task!=v.task_id||
         trace_identity_->route!=v.route_id||trace_identity_->route_hash!=v.route_hash||
         trace_identity_->map!=v.map_version_id||trace_identity_->epoch!=v.localization_epoch||
         trace_identity_->seed!=v.localization_seed_id||trace_identity_->segment!=v.segment_id||
         trace_identity_->anchor!=v.anchor_id||trace_identity_->anchor_revision!=v.anchor_revision||
         trace_identity_->execution!=command.execution_id) {
        for(const auto* value:{&v.session_id,&v.task_id,&v.route_id,&v.route_hash,&v.map_version_id,
             &v.localization_seed_id,&v.segment_id,&v.anchor_id,&command.execution_id})
          if(value->size()>256){control_trace_.failure();return;}
        auto c=std::make_shared<ControlTraceIdentity>();c->id=++trace_identity_sequence_;
        c->session=v.session_id;c->task=v.task_id;c->route=v.route_id;c->route_hash=v.route_hash;
        c->map=v.map_version_id;c->epoch=v.localization_epoch;c->seed=v.localization_seed_id;
        c->segment=v.segment_id;c->anchor=v.anchor_id;c->anchor_revision=v.anchor_revision;
        c->execution=command.execution_id;
        if(!c->bounded()){control_trace_.failure();return;}
        trace_identity_=std::move(c);
      }
      CommandTransitionDiagnostics::Sample sample;
      sample.steady_s=r;sample.source_ns=timeNs(command.source_stamp);sample.body_source_ns=timeNs(command.body_source_stamp);
      sample.command_sequence=command.sequence;sample.curve_id=command.trajectory_id;
      sample.reference_generation=v.reference_generation;
      sample.forward=command.velocity.linear.x;sample.lateral=command.velocity.linear.y;sample.yaw=command.velocity.angular.z;
      const auto tick=control_transitions_.observeTick(sample);
      if(!tick){control_trace_.failure();return;}
      ControlTraceRecord record;record.tick=*tick;record.identity=trace_identity_;
      record.task_active=!status.software_retired&&!v.task_id.empty();
      record.context_sequence=v.context_sequence;record.map_geometry_revision=v.map_geometry_revision;
      record.permit_sequence=command.permit_sequence;record.commit_sequence=status.commit_sequence;
      record.body_valid_until_ns=timeNs(status.body_valid_until);record.demand_valid_until_ns=timeNs(command.valid_until);
      record.tick_to_demand_publish_s=tick_to_demand_publish_elapsed;
      const auto proof=follower_->validationDiagnostic(t);
      record.proof_sequence=proof.sequence;record.map_revision=proof.map_revision;
      record.proof_source_ns=proof.source_ns;record.proof_valid_until_ns=proof.valid_until_ns;
      record.front_source_ns=proof.front_source_ns;record.rear_source_ns=proof.rear_source_ns;
      if(tick->kind) {
        record.reason=traceLabel(command.reason);record.phase=traceLabel(status.phase);
        record.prepare_reason=traceLabel(follower_->candidateReason());
        record.zero_cause=traceLabel(follower_->controller().controlDiagnostic().zero_target_cause);
        record.proof_failure=traceLabel(proof.failure);record.native_reason=traceLabel(proof.native_reason);
        record.proof_body_source_ns=proof.body_source_ns;record.check_begin_ns=proof.check_begin_ns;record.check_end_ns=proof.check_end_ns;
        const auto source=[](const NavigationSourceRecord& s) {
          ControlTraceRecord::Source out;out.received_ns=s.received_ns;out.source_ns=s.source_ns;
          out.posterior_ns=s.posterior_ns;out.imu_ns=s.imu_ns;out.valid_until_ns=s.valid_until_ns;
          out.schema=s.schema;out.epoch=s.epoch;out.seed=s.seed;out.extrapolation_s=s.extrapolation_sec;
          out.present=s.present;out.usable=s.usable;out.decoded=s.decoded;out.accepted=s.accepted;
          out.provider_reason=s.provider_reason;out.receiver_reason=s.receiver_reason;return out;
        };
        record.latest=source(state_diagnostics_.latest());record.accepted=source(state_diagnostics_.lastAccepted());
      }
      control_trace_.push(std::move(record));
    } catch(...) {control_trace_.failure();}
  }
  static double receipt(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void pumpGeometryExchange() {
    const auto clock=now();pumpGeometryExchange(clock,receipt());
  }
  void pumpGeometryExchange(const rclcpp::Time& clock,double r,
      LocalFollower::ExchangeClock trigger=LocalFollower::ExchangeClock::InputDelivery) {
    const builtin_interfaces::msg::Time event_time=clock;
    commit_dispatch_.forward(follower_->exchangeCommit(clock.seconds(),r,event_time,trigger),r,
      [this](const auto& fact){commit_->publish(fact);});
  }
  void tick() {
    const auto clock=now();const builtin_interfaces::msg::Time event_time=clock;
    const double t=clock.seconds(),r=receipt();
    pumpGeometryExchange(clock,r,LocalFollower::ExchangeClock::ControlTick);
    geometry_->publish(follower_->geometryPermit(t,event_time));
    const auto command=follower_->step(t,r,event_time);demand_->publish(command);
    const double tick_to_demand_publish_elapsed=receipt()-r; // Includes exchange/step, not just ROS publish.
    const auto s=follower_->status(t,event_time);status_->publish(s);
    const auto& core=follower_->controller();progress_->publish(trackingProgress(core,config_,t,s));
    const auto& control=core.controlDiagnostic();
    command_timing_.observe(r,command.velocity.linear.x,command.velocity.angular.z,
      control.valid&&!s.holding,control.input_limit_override);
    const bool nonzero=command.velocity.linear.x!=0.||command.velocity.linear.y!=0.||command.velocity.angular.z!=0.;
    recordControlTrace(command,s,t,r,tick_to_demand_publish_elapsed);
    if(nonzero)++nonzero_command_ticks_;else {++zero_command_ticks_;last_zero_reason_=command.reason;}
    if(have_command_classification_&&nonzero!=last_command_nonzero_)++zero_edges_;
    have_command_classification_=true;last_command_nonzero_=nonzero;
    const int yaw_sign=command.velocity.angular.z>0.?1:command.velocity.angular.z<0.?-1:0;
    if(yaw_sign&&last_nonzero_yaw_sign_&&yaw_sign!=last_nonzero_yaw_sign_)++yaw_reversals_;
    if(yaw_sign)last_nonzero_yaw_sign_=yaw_sign;
    // One bounded summary per second, not a command-rate logger. These are
    // command facts + original body feedback, never a physical-speed claim.
    if((core.active()||trace_was_active_)&&r-last_command_trace_>=1.) {
      last_command_trace_=r;trace_was_active_=core.active();
      const auto timing=command_timing_.takeWindow();
      RCLCPP_INFO(get_logger(),"ControlTiming samples=%lu intervals=%lu normal_intervals=%lu nonmonotonic=%lu stop_edges=%lu constraint_overrides=%lu max_interval_ms=%.3f max_forward_acceleration=%.6f max_yaw_acceleration=%.6f max_forward_jerk=%.6f max_yaw_jerk=%.6f normal_forward_acceleration=%.6f normal_yaw_acceleration=%.6f normal_forward_jerk=%.6f normal_yaw_jerk=%.6f",
        timing.samples,timing.intervals,timing.normal_intervals,timing.nonmonotonic_samples,timing.stop_edges,
        timing.constraint_overrides,
        timing.max_interval_s*1000.,timing.max_forward_acceleration,timing.max_yaw_acceleration,
        timing.max_forward_jerk,timing.max_yaw_jerk,timing.normal_max_forward_acceleration,
        timing.normal_max_yaw_acceleration,timing.normal_max_forward_jerk,timing.normal_max_yaw_jerk);
      const auto validation=follower_->validationDiagnostic(t);
      const auto& feedback=follower_->feedbackDiagnostic();
      RCLCPP_INFO(get_logger(),"WriterFeedback received=%lu rejected=%lu write=%lu gap=%lu gaps=%lu rebases=%lu body_source_ns=%ld before_forward=%.6f after_forward=%.6f",
        feedback.received,feedback.rejected,feedback.write_sequence,feedback.discontinuity,
        feedback.gaps,feedback.rebases,feedback.body_source_ns,feedback.before_forward,feedback.after_forward);
      RCLCPP_INFO(get_logger(),"ControlTrace curve=%ld phase=%s reason=%s command=[%.6f,%.6f,%.6f] body_v=[%.6f,%.6f] body_yaw=%.6f body_w=%.6f valid=%d final=%d goal_m=%.4f remaining_m=%.4f curve_v=%.6f cap=%.6f jerk_target=%.6f forward_target=%.6f heading_error=%.6f heading_weight=%.6f zero_cause=%s zero_ticks=%lu nonzero_ticks=%lu zero_edges=%lu yaw_reversals=%lu last_zero=%s steering=%s yaw_working_fraction=%.3f validation_failure=%s native_reason=%s validation_sequence=%lu proof_age_ms=%.3f proof_remaining_ms=%.3f prepare_reason=%s map_revision=%lu check_ms=%.3f proof_body_age_ms=%.3f front_ray_age_ms=%.3f rear_ray_age_ms=%.3f steering_curvature=%.6f raw_forward_target=%.6f raw_yaw_target=%.6f turn_allocation=%.6f yaw_target=%.6f",
        static_cast<long>(s.trajectory_id),s.phase.c_str(),command.reason.c_str(),
        command.velocity.linear.x,command.velocity.linear.y,command.velocity.angular.z,
        core.odometry().velocity_in_frame.x(),core.odometry().velocity_in_frame.y(),core.odometry().yaw,
        core.odometry().angular_velocity_in_frame.z(),control.valid,control.final_endpoint,
        control.goal_distance,control.remaining_arc,control.curve_speed,control.speed_cap,control.jerk_target_cap,
        control.forward_target,control.heading_error,control.heading_weight,control.zero_target_cause.c_str(),
        zero_command_ticks_,nonzero_command_ticks_,zero_edges_,yaw_reversals_,last_zero_reason_.c_str(),
        control.steering_phase.c_str(),control.yaw_working_fraction,validation.failure.c_str(),
        validation.native_reason.c_str(),validation.sequence,validation.age_s*1000.,validation.remaining_s*1000.,
        follower_->candidateReason().c_str(),validation.map_revision,validation.check_duration_s*1000.,
        validation.body_age_s*1000.,validation.front_age_s*1000.,validation.rear_age_s*1000.,
        control.steering_curvature,control.unallocated_forward_target,control.unallocated_yaw_target,
        control.turn_allocation_scale,control.yaw_target);
      const auto& preview=control.steering_sample;const auto& preceding=control.previous_steering_sample;
      RCLCPP_INFO(get_logger(),"SteeringGeometry curve=%ld source=%s current_curvature=%.6f applied_curvature=%.6f preview_chord_m=%.6f command_seq=%lu reference_generation=%lu sample_valid=%d body_source_ns=%ld projection_t=%.6f projection_arc=%.6f committed_arc=%.6f preview_t=%.6f preview_arc=%.6f preview_target=[%.6f,%.6f] previous_valid=%d previous_curve=%ld previous_body_source_ns=%ld previous_projection_t=%.6f previous_projection_arc=%.6f previous_preview_t=%.6f previous_preview_arc=%.6f previous_preview_target=[%.6f,%.6f] tangent_yaw=%.6f cross_error=%.6f flight_weight=%.6f yaw_feedforward=%.6f yaw_feedback=%.6f yaw_damping=%.6f",
        static_cast<long>(s.trajectory_id),control.steering_curvature_source.c_str(),
        control.steering_current_curvature,control.steering_curvature,control.steering_preview_distance,
        command.sequence,command.version.reference_generation,preview.valid,preview.body_source_ns,
        preview.projection_time,preview.projection_arc,preview.committed_arc,preview.preview_time,preview.preview_arc,
        preview.target.x(),preview.target.y(),preceding.valid,preceding.curve,preceding.body_source_ns,
        preceding.projection_time,preceding.projection_arc,preceding.preview_time,preceding.preview_arc,
        preceding.target.x(),preceding.target.y(),control.steering_tangent_yaw,control.steering_cross_error,
        control.steering_flight_weight,control.steering_yaw_feedforward,control.steering_yaw_feedback,
        control.steering_yaw_damping);
    }
    if(r-last_diagnostic_>=.2||s.phase!=last_phase_) {
      last_diagnostic_=r;last_phase_=s.phase;std_msgs::msg::String message;
      const auto validation=follower_->validationDiagnostic(t);
      message.data=nlohmann::json{{"session_id",config_.session_id},{"callback_wall_time",t},{"stamp",t},
        {"active",core.active()},{"trajectory_id",s.trajectory_id},{"generation",core.generation()},
        {"reason",s.reason},{"prepare_reason",follower_->candidateReason()},
        {"execution_architecture","task_follower_v1"},{"commit_sequence",s.commit_sequence},
        {"phase",s.phase},{"finished",s.at_goal},{"execution_frozen",s.holding},
        {"command_reason",command.reason},
        {"validation",{{"semantics","persistent_scan_collision_state"},
          {"persistent_state",validation.persistent_state},
          {"failure",validation.failure},{"native_reason",validation.native_reason},
          {"sequence",validation.sequence},{"age_s",validation.age_s},{"remaining_s",validation.remaining_s},
          {"map_revision",validation.map_revision},{"check_duration_s",validation.check_duration_s},
          {"body_age_s",validation.body_age_s},{"front_ray_age_s",validation.front_age_s},
          {"rear_ray_age_s",validation.rear_age_s}}},
        {"command",{{"forward",command.velocity.linear.x},
          {"lateral",command.velocity.linear.y},{"yaw_rate",command.velocity.angular.z}}},
        {"control",{{"valid",control.valid},{"final_endpoint",control.final_endpoint},
          {"remaining_arc_m",control.remaining_arc},{"goal_distance_m",control.goal_distance},
          {"curve_speed_mps",control.curve_speed},{"speed_cap_mps",control.speed_cap},
          {"jerk_target_cap_mps",control.jerk_target_cap},{"forward_target_mps",control.forward_target},
          {"heading_error_rad",control.heading_error},{"heading_weight",control.heading_weight},
          {"steering_curvature_per_m",control.steering_curvature},
          {"steering_current_curvature_per_m",control.steering_current_curvature},
          {"steering_preview_chord_m",control.steering_preview_distance},
          {"steering_curvature_source",control.steering_curvature_source},
          {"forward_entry_heading_active",control.forward_entry_heading_active},
          {"unallocated_forward_target_mps",control.unallocated_forward_target},
          {"unallocated_yaw_target_radps",control.unallocated_yaw_target},
          {"turn_allocation_scale",control.turn_allocation_scale},{"yaw_target_radps",control.yaw_target},
          {"forward_input_fraction",control.forward_input_fraction},{"forward_input_rate",control.forward_input_rate},
          {"yaw_input_fraction",control.yaw_input_fraction},{"yaw_input_rate",control.yaw_input_rate},
          {"input_limit_override",control.input_limit_override},
          {"working_level_steering",control.working_level_steering},
          {"yaw_working_fraction",control.yaw_working_fraction},{"steering_phase",control.steering_phase},
          {"zero_target_cause",control.zero_target_cause}}}}.dump();diagnostic_->publish(message);
    }
  }
  Config config_;std::string mode_;std::unique_ptr<LocalFollower> follower_;NavigationStateIngress ingress_;
  NavigationStateDiagnostics state_diagnostics_;
  LocalCommitDispatch commit_dispatch_;
  CommandTimingDiagnostics command_timing_;
  CommandTransitionDiagnostics control_transitions_;
  ControlTraceBuffer control_trace_;
  std::shared_ptr<const ControlTraceIdentity> trace_identity_;
  std::uint64_t trace_identity_sequence_{0};
  d1max_diagnostics::BoundedTaskDiagnosticFile trace_file_{"tracker"};
  bool trace_path_logged_{false},trace_failure_reported_{false}; // Passive diagnostic worker only.
  std::mutex trace_wait_mutex_;std::condition_variable trace_wakeup_;
  bool trace_stop_{false};std::thread trace_worker_;
  double last_diagnostic_{0.};std::string last_phase_;
  double last_command_trace_{0.};bool trace_was_active_{false},have_command_classification_{false},last_command_nonzero_{false};
  int last_nonzero_yaw_sign_{0};std::uint64_t zero_command_ticks_{0},nonzero_command_ticks_{0},zero_edges_{0},yaw_reversals_{0};
  std::string last_zero_reason_;
  rclcpp::Publisher<wire::LocalTrajectoryCommit>::SharedPtr commit_;
  rclcpp::Subscription<wire::LocalTrajectoryCommit>::SharedPtr commit_result_;
  rclcpp::Publisher<wire::LocalExecutionStatus>::SharedPtr status_;
  rclcpp::Publisher<wire::ExecutionPermit>::SharedPtr geometry_;
  rclcpp::Publisher<wire::MotionDemand>::SharedPtr demand_;
  rclcpp::Publisher<wire::TrackingProgress>::SharedPtr progress_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_;
  rclcpp::Subscription<wire::ReferenceProposal>::SharedPtr proposal_;
  rclcpp::Subscription<wire::SupportReference>::SharedPtr support_;
  rclcpp::Subscription<wire::TaggedBspline>::SharedPtr candidate_;
  rclcpp::Subscription<wire::TrajectoryValidation>::SharedPtr validation_;
  rclcpp::Subscription<wire::TaskExecutionLease>::SharedPtr lease_;
  rclcpp::Subscription<wire::TaskCommandFeedback>::SharedPtr task_feedback_;
  rclcpp::Subscription<wire::RouteDeliveryReceipt>::SharedPtr retirement_;
  rclcpp::Subscription<wire::LocalNavigationState>::SharedPtr state_;
  rclcpp::TimerBase::SharedPtr timer_,geometry_timer_;
};
} // namespace d1max_trajectory_tracker
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  try{
    rclcpp::spin(std::make_shared<d1max_trajectory_tracker::LocalFollowerNode>());
  }
  catch(const std::exception& e){std::fprintf(stderr,"local_follower startup: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
