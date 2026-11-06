// Status + handoff + explicit software e-stop targets. Optional navigation
// velocity transport reuses this SAME SDK session and requires explicit arming.
#include <chrono>
#include <atomic>
#include <array>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <thread>
#include "task_feedback_publication.hpp"
#include <random>
#include <iomanip>
#include <limits>
#include <sstream>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "robot_sdk/sdk_client.hpp"
#include "monitor_estop_core.hpp"
#include "mc_report_core.hpp"
#include "joint_report_core.hpp"
#include "sdk_session_core.hpp"
#include "sdk_ownership_core.hpp"
#include "navigation_motion_core.hpp"
#include "navigation_diagnostics_core.hpp"
#include "navigation_dispatch.hpp"
#include "execution_transport_core.hpp"
#include "mode_transition_core.hpp"
#include "execution_robot_health.hpp"
#include "execution_acceptance.hpp"
#include "execution_shutdown_core.hpp"
#include "execution_scheduler.hpp"
#include "execution_publication_core.hpp"
#include "sdk_write_transactions.hpp"
#include "sdk_command_trace.hpp"
#include "d1max_planning_interfaces/bounded_diagnostic_file.hpp"
#include "shared_estop_core.hpp"
#include "shared_estop_ipc.hpp"
#include "shared_manual_core.hpp"
using Json=nlohmann::json;
using String=std_msgs::msg::String;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double wall(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
class Monitor;
class Data final:public robot_sdk::IDataCallback {
 public:explicit Data(Monitor* node):node_(node){}
  void OnRobotStateData(const robot_sdk::RobotState&) override;
  void OnMcData(const robot_sdk::MotionData&) override;
  void OnJointStateData(const robot_sdk::JointStateData&) override;
  void OnFaultData(const robot_sdk::FaultDatas&) override;
  void OnControlLost(const robot_sdk::ControlLostInfo&) override;
  void OnControlAvailable(const robot_sdk::ControlAvailableInfo&) override;
 private:Monitor* node_;
};
class Ack final:public robot_sdk::IControlCallback {
 public:explicit Ack(Monitor* node):node_(node){}
  void OnSoftEmergencyStop(bool on) override;
  void OnMcConfig(bool on) override;
  void OnJointStateConfig(bool on) override;
  void OnTakeControlAck(const robot_sdk::TakeControlAck&) override;
  void OnMode(int mode) override;
  void OnStandUp() override;
  void OnLieDown() override;
  void OnCrawl() override;
  void OnLocked() override;
  void OnSpeed(int speed) override;
 private:Monitor* node_;
};
class Monitor final:public rclcpp::Node {
 public:
  std::mutex mutex;
  d1monitor::Safety safety;
  Monitor():Node("d1max_sdk_monitor",rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false)) {
    ip_=declare_parameter<std::string>("robot_ip","192.168.168.168");
    port_=std::to_string(declare_parameter<int>("robot_port",8081));
    mc_.max_attempts=declare_parameter<int>("mc_report_max_attempts",3);
    mc_.ready_delay=declare_parameter<double>("mc_report_ready_delay_sec",1.);
    mc_.retry_sec=declare_parameter<double>("mc_report_retry_sec",3.);
    mc_.cooldown_sec=declare_parameter<double>("mc_report_cooldown_sec",15.);
    mc_.max_cooldown_sec=declare_parameter<double>("mc_report_max_cooldown_sec",60.);
    mc_.stream_timeout_sec=declare_parameter<double>("mc_report_stream_timeout_sec",1.);
    mc_.validate();
    joints_.enabled=declare_parameter<bool>("joint_state_enabled",false);
    ownership_.enabled=declare_parameter<bool>("auto_take_control_on_available",false);
    ownership_.settle_sec=declare_parameter<double>("takeover_settle_sec",.5);
    ownership_.ack_timeout_sec=declare_parameter<double>("takeover_ack_timeout_sec",3.);
    ownership_.confirmation_timeout_sec=declare_parameter<double>("takeover_confirmation_timeout_sec",4.);
    ownership_.validate();
    navigation_.enabled=declare_parameter<bool>("navigation_control_enabled",false);
    navigation_.max_x=declare_parameter<double>("navigation_max_forward_mps",.30);
    navigation_.max_y=declare_parameter<double>("navigation_max_lateral_mps",.0);
    navigation_.max_yaw=declare_parameter<double>("navigation_max_yaw_radps",.50);
    navigation_.command_timeout=declare_parameter<double>("navigation_command_timeout_sec",.25);
    navigation_.validate();
    std::random_device random;std::ostringstream token;
    token<<std::hex<<std::setfill('0')<<std::setw(8)<<random()<<std::setw(8)<<random();session_=token.str();
    shared_estop_enabled_=declare_parameter<bool>("shared_estop_ipc_enabled",true);
    shared_estop_path_=declare_parameter<std::string>("shared_estop_ipc_path",d1monitor::shared_estop::IPC::defaultPath());
    const bool exec3_enabled=declare_parameter<bool>("execution_v3_enabled",false);
    const bool task_writer_enabled=declare_parameter<bool>("execution_task_writer_enabled",false);
    shared_estop_protocol_=std::make_unique<d1monitor::shared_estop::Protocol>(session_,task_writer_enabled);
    execution_mode_policy_=d1monitor::execution3::executionModePolicy(
      declare_parameter<std::string>("execution_mode_policy","segment_modes"));
    const auto stair_runtime_source=d1monitor::execution3::stairRuntimeParameterSource(
      declare_parameter<std::string>("execution_stair_runtime_parameter_source","evidence_record"));
    if(exec3_enabled&&execution_mode_policy_==d1monitor::execution3::ExecutionModePolicy::FixedStair)
      throw std::invalid_argument("fixed_stair_requires_task_follower_v1");
    const auto exec3_record=declare_parameter<std::string>("execution_acceptance_record","");
    const auto exec3_robot=declare_parameter<std::string>("execution_robot_id","");
    const auto exec3_sdk=declare_parameter<std::string>("execution_sdk_version","");
    const auto exec3_calibration=declare_parameter<std::string>("execution_calibration_sha256","");
    const auto exec3_profile=declare_parameter<std::string>("execution_robot_profile_sha256","");
    if((exec3_enabled?1:0)+(task_writer_enabled?1:0)+(navigation_.enabled?1:0)>1)
      throw std::invalid_argument("task_schema3_and_legacy_motion_writers_are_mutually_exclusive");
    if(exec3_enabled||task_writer_enabled) {
      ownership_.enabled=false; // Only an explicit grant may request one takeover.
      execution_acceptance_=d1monitor::execution3::validateAcceptance(exec3_record,exec3_robot,exec3_sdk,exec3_calibration,exec3_profile);
      if(execution_acceptance_.sdk_input_mapping.enabled&&!task_writer_enabled)
        throw std::invalid_argument("nonlinear_sdk_input_mapping_requires_task_follower_v1");
      const auto accepted_record_hash=execution_acceptance_.valid?d1monitor::execution3::fileSha256(exec3_record):std::string();
      execution_profile_sha256_=accepted_record_hash;
      // Reusing an engineering profile is explicitly authorized runtime
      // configuration, never a certificate of STAIR physical acceptance.
      execution_physical_acceptance_verified_=execution_acceptance_.physically_verified&&
        stair_runtime_source!=d1monitor::execution3::StairRuntimeParameterSource::GeneralRuntimeReuse;
      execution3_=std::make_unique<d1monitor::execution3::Transport>(session_,"live",execution_acceptance_.valid,
        execution_acceptance_.max_speed,execution_acceptance_.max_yaw,accepted_record_hash,execution_acceptance_.kinematic_profile,
        execution_physical_acceptance_verified_);
      execution3_->configureTaskFollower(task_writer_enabled);
      mode_transition_=std::make_unique<d1monitor::execution3::ModeTransition>(session_,"live",
        d1monitor::execution3::loadModeCapabilities(
          declare_parameter<std::string>("execution_mode_capability_record",""),
          declare_parameter<std::string>("execution_mode_capability_sha256",""),
          exec3_robot,exec3_sdk,exec3_profile,execution_mode_policy_),execution_mode_policy_,stair_runtime_source,
          execution_acceptance_.valid);
      if(execution_acceptance_.valid){
        execution3_->configureMcCaptureBound(execution_acceptance_.mc_delay_bound,"source_delta_host_anchor_approximate");
        execution3_->configureExecutionPolicy(execution_acceptance_.policy,execution_acceptance_.reaction_bound);
        mc_.configureAcceptedRate(execution_acceptance_.policy.mc_expected_hz,execution_acceptance_.policy.mc_min_hz,
          execution_acceptance_.policy.mc_max_hz);
      }
      // Never attach this group to the diagnostic executor. Core input and
      // revoke callbacks get their own bounded queues and executor thread.
      execution_callbacks_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
      rclcpp::SubscriptionOptions execution_options;execution_options.callback_group=execution_callbacks_;
      const std::string p="/d1max/live_planning/execution/";
      if(task_writer_enabled){
        manual_handoff_event_=create_publisher<String>("/d1max_sdk_bridge/manual_handoff",rclcpp::QoS(1).reliable().transient_local());
        manual_handoff_ack_=create_subscription<String>("/d1max_sdk_bridge/manual_handoff_ack",rclcpp::QoS(1).reliable().transient_local(),
          [this](String::ConstSharedPtr m){
            try{const auto j=Json::parse(m->data);std::lock_guard<std::mutex>lock(mutex);
              if(j.is_object()&&j.value("schema",0)==1&&j.value("sdk_session",std::string())==session_&&
                 !manual_handoff_lease_.empty()&&j.value("lease",std::string())==manual_handoff_lease_)
                manual_handoff_acknowledged_=true;
            }catch(const Json::exception&){}
          },execution_options);
      }
      execution_state_=create_publisher<d1monitor::execution3::State>(p+"sdk_state",1);
      execution_task_feedback_=create_publisher<d1monitor::execution3::TaskFeedback>(p+"task_command_feedback",rclcpp::QoS(1).reliable());
      execution_stop_=create_publisher<d1monitor::execution3::Stop>(p+"stop_report",1);
      execution_retirement_=create_publisher<d1monitor::execution3::Retirement>(p+"retirement",1);
      execution_commit_=create_publisher<d1monitor::execution3::CommitAck>(p+"commit_ack",5);
      execution_stationary_=create_publisher<d1monitor::execution3::Stationary>(p+"stationary_evidence",8);
      mode_state_=create_publisher<d1monitor::execution3::ModeState>(p+"mode_transition_state",1);
      execution_grant_=create_service<d1monitor::execution3::Grant>(p+"grant",
        [this,accepted_record_hash,exec3_sdk](const std::shared_ptr<d1monitor::execution3::Grant::Request>r,std::shared_ptr<d1monitor::execution3::Grant::Response>s){
          std::lock_guard<std::mutex> lock(mutex);
          sync_execution_transactions();
          apply_navigation_events();
          if(execution_write_failed_.exchange(false))execution3_->submissionFailed(wall());
          if(r->activate&&(!sdk_||sdk_version_!=exec3_sdk)){
            *s=execution3_->rejectGrant(*r,"actual_sdk_version_not_bound");return;}
          if(r->activate&&(accepted_record_hash.empty()||r->acceptance_record_sha256!=accepted_record_hash)){
            *s=execution3_->rejectGrant(*r,"runtime_motion_profile_not_bound");return;}
          if(r->activate&&!shared_manual_.allowsTask(r->version.session_id,r->version.task_id)){
            *s=execution3_->rejectGrant(*r,"manual_takeover_retired_task");return;}
          if(r->activate&&!manual_handoff_lease_.empty()&&
             (!manual_handoff_acknowledged_||d1monitor::execution3::sourceNs(r->source_stamp)<=manual_handoff_stamp_ns_)){
            *s=execution3_->rejectGrant(*r,"manual_handoff_waiting_task_fence");return;}
          if(r->activate&&!shared_manual_.navigationReady(mono())){
            *s=execution3_->rejectGrant(*r,"manual_control_busy");return;}
          *s=execution3_->grant(*r,wall(),execution_health());
          if(!r->activate&&s->accepted)wake_execution_stop();
          if(r->activate&&s->accepted&&!s->duplicate){
            shared_manual_.releaseComplete();manual_retired_stop_.reset();manual_retired_report_.reset();
            execution_stop_written_epoch_.store(0);execution_stop_written_at_.store(0.);
          }
        },rmw_qos_profile_services_default,execution_callbacks_);
      if(task_writer_enabled) {
        if(!mode_transition_->fixedStair())mode_request_=create_subscription<d1monitor::execution3::ModeRequest>(p+"mode_transition_request",5,
          [this](d1monitor::execution3::ModeRequest::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);
            apply_navigation_events();
            const auto previous=mode_transition_->requestId();
            if(mode_transition_->begin(*m,*execution3_,execution3_->taskLeaseSnapshot(),wall())){
              if(previous!=mode_transition_->requestId()&&mode_transition_->holding())execution3_->beginModeHold();
              wake_execution_stop();}},execution_options);
        execution_task_lease_=create_subscription<d1monitor::execution3::TaskLease>(p+"task_lease",10,
          [this](d1monitor::execution3::TaskLease::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);
            const auto before=execution3_->lastDemand().sequence;
            const bool accepted=execution3_->taskLease(*m,wall());
            if(execution3_->stopping())ownership_.enabled=false;
            if(accepted&&(!m->allowed||m->revoked||m->phase=="stopping"||m->phase=="terminal"))wake_execution_stop();
            else if(accepted&&execution3_->lastDemand().sequence!=before)wake_execution_command();},execution_options);
        execution_local_command_=create_subscription<d1monitor::execution3::LocalCommand>(p+"local_command",1,
          [this](d1monitor::execution3::LocalCommand::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);
            const auto barrier=execution3_->taskCommandStopBarrier();
            const bool accepted=execution3_->localCommand(*m,wall());
            if(execution3_->taskCommandStopBarrier()!=barrier||
               (accepted&&(m->hold||!d1monitor::execution3::planarMotionNonzero(m->velocity))))wake_execution_stop();
            // A pure Native evidence update changes no controller sample and
            // must not generate an extra Move or rephase the 20 Hz writer.
            // Its existing earlier deadline wake re-reads the new lease.
            else if(accepted&&execution3_->taskCommandAdmission().reason!=std::string_view("evidence_updated"))
              wake_execution_command();},execution_options);
      } else {
      execution_permit_=create_subscription<d1monitor::execution3::Permit>(p+"permit",10,
        [this](d1monitor::execution3::Permit::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->permit(*m,wall());
          if(execution3_->stopping())ownership_.enabled=false;},execution_options);
      execution_demand_=create_subscription<d1monitor::execution3::Demand>(p+"safe_demand",1,
        [this](d1monitor::execution3::Demand::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->demand(*m,wall());},execution_options);
      execution_motion_proof_=create_subscription<d1monitor::execution3::MotionProof>(p+"motion_validation",20,
        [this](d1monitor::execution3::MotionProof::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->motionValidation(*m,wall());},execution_options);
      execution_handoff_=create_subscription<d1monitor::execution3::Handoff>(p+"handoff_grant",5,
        [this](d1monitor::execution3::Handoff::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->handoff(*m,wall());},execution_options);
      execution_prepared_=create_subscription<d1monitor::execution3::Prepared>(p+"safe_prepared_demand",1,
        [this](d1monitor::execution3::Prepared::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->preparedDemand(*m,wall());},execution_options);
      execution_curve_=create_subscription<d1monitor::execution3::CurveProof>(p+"validation",20,
        [this](d1monitor::execution3::CurveProof::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->trajectoryValidation(*m,wall());},execution_options);
      execution_progress_=create_subscription<d1monitor::execution3::Progress>("/d1max/live_planning/tracking_progress",2,
        [this](d1monitor::execution3::Progress::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->trackingProgress(*m,wall());},execution_options);
      execution_local_=create_subscription<d1monitor::execution3::Local>("/d1max/localization/navigation/local_state",2,
        [this](d1monitor::execution3::Local::SharedPtr m){std::lock_guard<std::mutex>lock(mutex);execution3_->localState(*m,wall());},execution_options);
      execution_geometry_=create_subscription<d1max_planning_interfaces::msg::TaggedBspline>(
        "/d1max/live_planning/scan_tagged_bspline",2,
        [this](d1max_planning_interfaces::msg::TaggedBspline::SharedPtr m){
          std::lock_guard<std::mutex>lock(mutex);execution3_->trajectoryGeometry(*m,wall());},execution_options);
      }
    }
    for(const auto& topic:{"robot_state","velocity","speed_report_status","faults","behavior_state","connection_state_text","transition_event"})
      pubs_[topic]=create_publisher<String>(std::string("/d1max_sdk_bridge/")+topic,1);
    if(joints_.enabled){
      pubs_["joint_states"]=create_publisher<String>("/d1max_sdk_bridge/joint_states",1);
      pubs_["joint_state_status"]=create_publisher<String>("/d1max_sdk_bridge/joint_state_status",1);
    }
    status_=create_publisher<String>("/d1max/monitor/status",10);
    rclcpp::SubscriptionOptions clock_options;if(execution_callbacks_)clock_options.callback_group=execution_callbacks_;
    clock_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;},clock_options);
    graph_=create_wall_timer(std::chrono::seconds(1),[this]{
      for(const auto& name:get_node_names())if(name.find("rosbag2_player")!=std::string::npos){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;}
    });
    stop_=create_service<std_srvs::srv::Trigger>("/d1max/monitor/s_"+session_+"/soft_estop",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      const auto submission=request_soft_estop();
      response->success=submission.success;response->message=submission.message;
      String event;event.data=response->message;pubs_.at("transition_event")->publish(event);
    });
    if(navigation_.enabled){
      navigation_velocity_=create_subscription<String>("/d1max/monitor/s_"+session_+"/navigation_velocity",1,[this](String::ConstSharedPtr msg){
        if(msg->data.size()>4096)return;
        try {
          const auto j=Json::parse(msg->data);
          if(j.at("sdk_session")!=session_||!j.at("seq").is_number_unsigned()||!j.at("arm_generation").is_number_unsigned())return;
          const d1monitor::NavigationVelocity v{j.at("x").get<double>(),j.at("y").get<double>(),j.at("yaw").get<double>()};
          std::lock_guard<std::mutex> lock(mutex);
          apply_navigation_events();
          navigation_.accept(j.at("arm_generation").get<uint64_t>(),j.at("seq").get<uint64_t>(),
            j.at("navigation_session").get<std::string>(),j.at("map_version_id").get<std::string>(),v,j.at("stamp").get<double>(),wall(),mono(),j.at("command_source").get<std::string>());
        }catch(const Json::exception&){std::lock_guard<std::mutex> lock(mutex);navigation_.disarm("malformed_navigation_command");}
      });
      navigation_arm_=create_service<std_srvs::srv::SetBool>("/d1max/monitor/s_"+session_+"/navigation_arm",[this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        std::lock_guard<std::mutex> lock(mutex);
        apply_navigation_events();
        if(req->data)response->success=navigation_.arm(navigation_health(mono()),mono(),wall());
        else {navigation_.disarm("operator_disarmed");response->success=true;}
        response->message=Json{{"armed",navigation_.armed},{"arm_generation",navigation_.generation},{"sdk_session",session_},{"reason",navigation_.error}}.dump();
      });
      navigation_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{navigation_tick();});
    }
    robot_sdk::ConnectionConfig config;config.auto_reconnect=true;config.reconnect_interval_ms=2000;
    sdk_=std::make_unique<robot_sdk::SDKClient>([this](const std::error_code& e){if(e){raise_navigation_event(d1monitor::NavigationEvents::ConnectionError);std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}},config);
    sdk_version_=sdk_->Version(); // Local immutable version; never queried in a service/critical lock.
    data_=std::make_shared<Data>(this);ack_=std::make_shared<Ack>(this);
    sdk_->SetDataCallback(data_);sdk_->SetControlCallback(ack_);
    // Async even for the first connection: an initially offline robot must not
    // block the ROS executor or require a user to restart this process.
    connect_retry_.due(1,mono());connect_async();
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{tick();});
    RCLCPP_INFO(get_logger(),"MONITOR: OnMcData; APP-release handoff %s; optional navigation %s (startup DISARMED). No posture/mode/e-stop-release commands.",ownership_.enabled?"enabled":"disabled",navigation_.enabled?"enabled":"disabled");
  }
  ~Monitor() override {
    shutting_down_.store(true);
    // Stop and join local IPC before the SDK/node lifetime ends. Detaching a
    // panel has no SDK side effect and cannot release the latched emergency.
    if(shared_estop_ipc_)shared_estop_ipc_->stop();
    shared_estop_ipc_.reset();
    stop_execution_workers();
    timer_->cancel();graph_->cancel();
    if(navigation_timer_)navigation_timer_->cancel();
    try{shutdown_execution();}
    catch(const std::exception&error){RCLCPP_ERROR(get_logger(),"SDK v3 shutdown outcome unknown: %s",error.what());}
    d1monitor::dispatch_navigation(mutex,[this]()->std::optional<d1monitor::NavigationVelocity>{
      apply_navigation_events();
      const bool send=navigation_.moving&&navigation_.can_stop(navigation_health(mono()),mono());
      navigation_.disarm("bridge_exit");
      return send?std::optional<d1monitor::NavigationVelocity>{d1monitor::NavigationVelocity{}}:std::nullopt;
    },[this](const auto& command){send_navigation(command);});
    {std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;}
    wake_.notify_all();joint_wake_.notify_all();
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);telemetry_stopping_=true;}
    telemetry_wake_.notify_all();
    if(worker_.joinable())worker_.join();
    if(telemetry_worker_.joinable())telemetry_worker_.join();
    if(joint_worker_.joinable())joint_worker_.join();
    // Only close the telemetry stream this process requested. Never release estop.
    if(mc_.total_attempts&&!safety.replay&&ownership_.allow_mc_config()&&sdk_->IsConnected())sdk_->SetMcConfig(false,250);
    if(joints_.enabled&&joints_.attempts&&!safety.replay&&sdk_->IsConnected())sdk_->SetJointStateConfig(false,250);
    sdk_->Disconnect(true);sdk_.reset();
    // All Move producers are joined and SDK callbacks have ended. Only now
    // request the passive worker's final drain, after real stop/disconnect;
    // a slow file sink must never delay those operations or outlive this node.
    execution_trace_stopping_.store(true);execution_trace_wake_.notify_all();
    if(execution_trace_worker_.joinable())execution_trace_worker_.join();
    if(execution_feedback_worker_)execution_feedback_worker_->close();
  }
  void start_execution_workers(){
    // Start only after the complete object exists. A thread allocation failure
    // can then unwind through the normal join/stop destructor, not terminate
    // from a partially constructed object holding joinable threads.
    if(worker_.joinable()||telemetry_worker_.joinable())throw std::logic_error("monitor_workers_already_started");
    if(shared_estop_enabled_){
      try{
        shared_estop_ipc_=std::make_unique<d1monitor::shared_estop::IPC>(shared_estop_path_,
          [this](uint64_t client,const Json&message){shared_estop_request(client,message);},
          [this](uint64_t client){{std::lock_guard<std::mutex>lock(mutex);shared_manual_.detach(client);}shared_estop_protocol_->disconnect(client);wake_execution_stop();});
        shared_estop_ipc_->start();
      }catch(const std::exception&error){
        // The local panel is optional. A bad path, occupied socket or failed
        // I/O thread must not close the existing SDK session or ROS estop.
        if(shared_estop_ipc_)shared_estop_ipc_->stop();
        shared_estop_ipc_.reset();shared_estop_enabled_=false;
        RCLCPP_WARN(get_logger(),"Shared emergency panel IPC unavailable: %s; ROS emergency stop and navigation SDK session remain enabled",error.what());
      }
    }
    worker_=std::thread([this]{work();});
    telemetry_worker_=std::thread([this]{work_telemetry();});
    if(joints_.enabled)joint_worker_=std::thread([this]{work_joint();});
    if(!execution3_)return;
    execution_feedback_worker_=std::make_unique<d1monitor::execution3::TaskFeedbackPublication>(
      [this](const auto& fact){execution_task_feedback_->publish(fact);},
      [this]{raise_navigation_event(d1monitor::NavigationEvents::WorkerStopped);});
    execution_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    execution_executor_->add_callback_group(execution_callbacks_,get_node_base_interface());
    execution_input_worker_=std::thread([this]{
      try{execution_executor_->spin();}
      catch(const std::exception&error){raise_navigation_event(d1monitor::NavigationEvents::WorkerStopped);
        RCLCPP_ERROR(get_logger(),"Execution input worker stopped: %s",error.what());}
    });
    execution_writer_=std::thread([this]{
      const bool task_writer=execution3_->taskFollower();
      const auto start=mono();d1monitor::WriterDeadline deadline(start,.05);
      d1monitor::PeriodicDeadline manual_deadline(start,.05);
      d1monitor::CommandWriterDeadline command_deadline(start,.05);uint64_t urgent_seen=0,command_seen=0;
      while(!execution_workers_stopping_.load()){
        std::optional<double> original_deadline;
        if(task_writer){std::lock_guard<std::mutex>lock(mutex);
          if(const auto end=execution3_->taskCommandDeadline()){
            // Original wall deadline translated to this wait's steady clock,
            // not rewritten into the command. nextafter preserves inclusive
            // source validity at the exact original boundary.
            original_deadline=mono()+std::max(0.,std::nextafter(*end,std::numeric_limits<double>::infinity())-wall());}}
        command_seen=execution_command_revision_.load(std::memory_order_acquire);
        if(task_writer)command_deadline.observe(command_seen);
        bool manual_active=false;{std::lock_guard<std::mutex>lock(mutex);manual_active=shared_manual_.needsWriterService();}
        const auto next=manual_active?manual_deadline.next():task_writer?command_deadline.next(original_deadline):deadline.next();
        {std::unique_lock<std::mutex>lock(execution_wait_mutex_);
          const auto due=std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(next)));
          execution_wake_.wait_until(lock,due,[this,urgent_seen,command_seen,task_writer]{return execution_workers_stopping_.load()||
            execution_urgent_revision_.load()!=urgent_seen||
            (task_writer&&execution_command_revision_.load(std::memory_order_acquire)!=command_seen);});}
        if(execution_workers_stopping_.load())break;
        const auto urgent=execution_urgent_revision_.load();const bool urgent_wake=urgent!=urgent_seen;
        urgent_seen=urgent;const auto started=mono();
        command_seen=execution_command_revision_.load(std::memory_order_acquire);
        if(task_writer)command_deadline.observe(command_seen);
        // Urgent zero is independent of the absolute nominal 20 Hz send
        // phase. Ordinary OS jitter must not rebase that phase every cycle.
        {std::lock_guard<std::mutex>lock(mutex);manual_active=shared_manual_.needsWriterService();}
        const bool urgent_only=manual_active?(urgent_wake||started<manual_deadline.next()):
          task_writer?(urgent_wake||!command_deadline.normalDue(started)):started<deadline.next();
        const bool no_fresh_command=task_writer&&!command_deadline.pending()&&!manual_active;
        uint64_t selected_revision=command_seen;
        std::optional<double> nonzero_submitted;
        // Health/watchdog polls must not resend an unchanged nonzero and
        // consume the opportunity just before the next producer sample.
        // Initial TakeControl and all zero/stop work still progress normally.
        try{nonzero_submitted=execution_tick(urgent_only,no_fresh_command,&selected_revision);}
        catch(const std::exception&error){
          // This writer is already awake. Do not wake itself recursively on
          // a repeatedly throwing vendor: retry only at the next normal slot.
          navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
          RCLCPP_ERROR(get_logger(),"Execution writer tick failed, motion veto latched: %s",error.what());}
        const auto completed=mono();execution_last_duration_s_.store(completed-started);
        if(completed-started>.05)++execution_writer_overruns_;
        if(manual_active){if(completed>=manual_deadline.next())manual_deadline.complete(completed);}
        else if(task_writer){command_deadline.complete(started,completed,selected_revision,!urgent_only&&!no_fresh_command,nonzero_submitted);
          execution_writer_skipped_.store(command_deadline.skipped());}
        else {if(!urgent_only)deadline.complete(started,completed);execution_writer_skipped_.store(deadline.skipped());}
      }
    });
    start_command_trace_worker();
  }
  void start_command_trace_worker() noexcept {
    try {execution_trace_worker_=std::thread([this]{work_command_trace();});}
    catch(const std::exception& error){RCLCPP_WARN(get_logger(),"SDK command trace unavailable (diagnostics only): %s",error.what());}
    catch(...){RCLCPP_WARN(get_logger(),"SDK command trace unavailable (diagnostics only)");}
  }
  void work_command_trace() noexcept {
    // This thread owns only the passive ring consumer and file sink. It is
    // deliberately outside BOTH ROS executors and the ordered SDK writer.
    // Slow disk cannot delay ROS soft_estop, shared_estop_tick or state updates.
    try {
      const auto flush=[this](size_t snapshots) {
        for(size_t snapshot=0;snapshot<snapshots;++snapshot) {
          if(!execution_trace_.beginDrain())break;
          // At least one record is consumed per page; a counters-only snapshot
          // needs one page. Never chase unlimited new producer input here.
          for(size_t page=0;page<=d1monitor::command_trace::Trace::capacity&&execution_trace_.pending();++page) {
            const auto& command_batch=execution_trace_.preparePage();
            if(command_batch.empty())return;
            const bool written=execution_trace_file_.write(command_batch);
            if(written)execution_trace_.commitPage();
            if(written&&!execution_trace_location_logged_){
              execution_trace_location_logged_=true;
              RCLCPP_INFO(get_logger(),"SDKCommandTraceLocation directory=%s schema=sdk_move_v1 command_and_ACK_are_not_measured_motion",
                execution_trace_file_.directory().c_str());
            }
            if(!written&&!execution_trace_failure_logged_){
              execution_trace_failure_logged_=true;
              RCLCPP_WARN(get_logger(),"SDKCommandTraceWriteFailed diagnostics_only failed=%lu rejected=%lu",
                execution_trace_file_.failedWrites(),execution_trace_file_.rejectedBatches());
            }
            if(!written)return; // Keep the identical page pending for retry.
          }
        }
      };
      while(!execution_trace_stopping_.load()){
        {std::unique_lock<std::mutex> lock(execution_trace_wait_mutex_);
          if(execution_trace_wake_.wait_for(lock,std::chrono::seconds(1),
              [this]{return execution_trace_stopping_.load();}))break;}
        flush(1);
      }
      // Shutdown publishes the stop flag only after producer/callback lifetime
      // ends. Finish a pending snapshot, then capture the final bounded ring.
      flush(2);
    }catch(const std::exception& error){RCLCPP_WARN(get_logger(),"SDK command trace stopped (diagnostics only): %s",error.what());}
    catch(...){RCLCPP_WARN(get_logger(),"SDK command trace stopped (diagnostics only)");}
  }
  void stop_execution_workers(){
    execution_workers_stopping_.store(true);execution_wake_.notify_all();
    if(execution_executor_)execution_executor_->cancel();
    if(execution_input_worker_.joinable())execution_input_worker_.join();
    if(execution_writer_.joinable())execution_writer_.join();
    // Joining a vendor-blocked Move is not a bounded physical stop. Do not
    // detach the writer, submit a competing command, or claim a 250 ms guard.
  }
  void publish(const std::string& name,const Json& json){String msg;msg.data=json.dump();pubs_.at(name)->publish(msg);}
  void robot(const robot_sdk::RobotState& d){
    using Events=d1monitor::NavigationEvents;
    const auto arrival=mono(),received=wall();
    const int software=static_cast<int>(d.software_emergency_status),hardware=static_cast<int>(d.hardware_emergency_status);
    if(static_cast<int>(d.control_source)!=2)raise_navigation_event(Events::LostControl);
    if(software==2||hardware==2){
      {std::lock_guard<std::mutex>lock(estop_observation_mutex_);
       if(software==2)software_estop_observed_at_=std::max(software_estop_observed_at_,arrival);
       if(hardware==2)hardware_estop_observed_at_=std::max(hardware_estop_observed_at_,arrival);}
      raise_navigation_event(Events::EmergencyStop);
    }
    // A real software STOP has its own irreversible old-task retirement latch.
    // Do not also turn that one fact into an unrelated, unreleasable fault.
    const int mode=static_cast<int>(d.sport_mode),motion=static_cast<int>(d.motion_status);
    const bool fixed_stair=execution3_&&execution_mode_policy_==d1monitor::execution3::ExecutionModePolicy::FixedStair;
    const bool ordinary=!fixed_stair&&mode==1&&motion==5;
    const bool stair=mode==3&&motion==7&&
      (fixed_stair||mode_expected_.load(std::memory_order_acquire)==3||mode_active_.load(std::memory_order_acquire)==3);
    if((software!=1&&software!=2)||hardware!=1||(!ordinary&&!stair)||
       static_cast<int>(d.speed_level)!=1||static_cast<int>(d.head_direction)!=1)
      raise_navigation_event(Events::UnsafeRobotState);
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     robot_in_=RobotPacket{d,arrival,received,++mode_robot_sequence_};
     owner_in_.push({OwnerKind::Robot,arrival,static_cast<uint32_t>(d.control_source),0,{}});}
    wake_.notify_one();
  }
  void faults(const robot_sdk::FaultDatas& d){
    // The sticky veto is independent of the diagnostic queue capacity.
    for(const auto& f:d)if(static_cast<int>(f.level)==1||static_cast<int>(f.level)==2){
      shared_estop_fatal_.store(true);
      raise_navigation_event(d1monitor::NavigationEvents::RobotFault);break;
    }
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     for(const auto& f:d){if(faults_in_.size()>=64)break;auto copy=f;copy.message=copy.message.substr(0,1024);faults_in_.push_back(std::move(copy));}}
    wake_.notify_one();
  }
  void mc(const robot_sdk::MotionData& d){
    const McPacket packet{d,mono(),wall()};
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;mc_in_.push(packet);}
    wake_.notify_one();
  }
  void mc_config(bool on){if(!on)raise_navigation_event(d1monitor::NavigationEvents::McDisabled);std::lock_guard<std::mutex> lock(inbox_mutex_);mc_ack_=on;}
  void mode_ack(int mode){
    manual_ack(d1monitor::shared_manual::Action::Mode,mode);
    // Vendor callback only copies its original receipt. No SDK call or state
    // transition runs on the callback thread.
    {std::lock_guard<std::mutex>lock(mode_ack_mutex_);mode_ack_in_=std::make_pair(mode,wall());}
    wake_execution_stop();
  }
  void manual_ack(d1monitor::shared_manual::Action action,int value=0){
    std::lock_guard<std::mutex>lock(manual_ack_mutex_);
    manual_acks_[static_cast<unsigned>(action)]={++manual_ack_sequence_,value};
  }
  void joint(const robot_sdk::JointStateData& d){
    if(!joints_.enabled)return;
    d1monitor::JointPacket packet;
    const bool valid=d1monitor::copy_joint_packet(d,mono(),wall(),packet);
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_||joint_worker_failed_)return;
     if(!valid){++joint_invalid_;return;}
     if(joint_in_)++joint_dropped_;
     joint_in_=packet;}
    joint_wake_.notify_one();
  }
  void joint_config(bool on){
    if(!joints_.enabled)return;
    std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
    joint_ack_=JointAck{on,mono()};
  }
  void estop_ack(bool on){std::lock_guard<std::mutex>lock(inbox_mutex_);if(!stopping_)estop_ack_=EstopAck{on,mono()};}
  // SDK callback thread only copies small, bounded events. Network requests,
  // state transitions, JSON and ROS publication run on the ROS executor.
  void ownership_event(bool available){
    if(!available)raise_navigation_event(d1monitor::NavigationEvents::LostControl);
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     owner_in_.push({available?OwnerKind::Available:OwnerKind::Lost,mono(),0,0,{}});}
  }
  void ownership_ack(const robot_sdk::TakeControlAck& ack){
    std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
    owner_in_.push({OwnerKind::Ack,mono(),ack.error_code,0,ack.reason.substr(0,512)});
  }
 private:
  struct EstopSubmission {
    bool success=false,sent=false;std::string message;
    uint64_t transaction=0;
    std::shared_ptr<d1monitor::shared_estop::WriteResult>write;
  };
  // Called with the state mutex held. Confirmation is a software flag fact,
  // not physical stopping or renewed navigation permission. Transport retains
  // its old stopping identity until its existing retirement/MC evidence closes.
  void confirm_soft_estop_release(){
    if(!safety.releaseConfirmed()||
       safety.confirmed_transaction==software_release_notified_)return;
    software_release_notified_=safety.confirmed_transaction;
    software_release_receipt_=safety.state_at;
    if(execution3_)execution3_->confirmSoftwareEstopRelease(wall());
  }
  // Log transitions, not each 1 Hz RobotState or IPC heartbeat. Preserve the
  // raw state age so a later report can distinguish SDK feedback from an ACK.
  // Called with the state mutex held; no SDK operation is performed here.
  void log_soft_estop_target(const char* event){
    RCLCPP_INFO(get_logger(),"Software estop %s: transaction=%lu target=%s pending=%s result=%s confirmed=%lu software=%d hardware=%d state_age=%.3f error=%s",
      event,static_cast<unsigned long>(safety.transaction),safety.target_on?"ON":"OFF",
      safety.pending?"true":"false",safety.result.c_str(),static_cast<unsigned long>(safety.confirmed_transaction),
      safety.software,safety.hardware,mono()-safety.state_at,safety.error.c_str());
  }
  EstopSubmission request_soft_estop(bool on=true){
    EstopSubmission out;
    {std::lock_guard<std::mutex>lock(mutex);
      if(shutting_down_.load()){out.message="SDK 接收器正在退出";return out;}
      safety.connected=sdk_&&sdk_->IsConnected();
      if(!safety.available()){out.message=safety.replay?"检测到回放，急停请求禁用":"SDK 未连接，急停请求未发送";return out;}
      const auto prior_transaction=safety.transaction;
      out.sent=safety.request(mono(),on);out.success=safety.request_accepted;out.transaction=safety.transaction;
      if(safety.transaction!=prior_transaction)log_soft_estop_target("requested");
      if(!out.success){out.message=safety.request_reason;return out;}
      // This lock orders final Move selection. ON retires permission BEFORE
      // entering the SDK. OFF never arms, takes control or restores a demand.
      if(on){
        shared_manual_.veto("software_estop_requested");
        if(execution3_)execution3_->softwareEstopRequested(wall());
        if(navigation_.enabled)navigation_.fail("software_estop_requested_latched");
        else navigation_.disarm("software_estop_requested");
        wake_execution_stop();
      }else confirm_soft_estop_release();
      out.message=out.sent?(on?"软件急停请求已提交；等待机器人状态，不代表实测停稳":
        "解除软件急停请求已提交；等待机器人状态，不恢复旧导航权限或速度"):
        safety.pending?"软件急停目标正在等待状态确认；未重复发送":
        on?"机器人已回报软件急停触发；未重复发送，不代表实测停稳":
        "机器人已回报软件急停解除；未重复发送，不恢复旧导航权限或速度";
      if(out.sent){
        out.write=std::make_shared<d1monitor::shared_estop::WriteResult>();
        estop_write_=out.write;estop_write_transaction_=out.transaction;estop_write_checked_=false;
      }
    }
    if(!out.sent)return out;
    const auto write=out.write;
    // Serialize actual SDK submissions, not just state selection. ON still
    // latches permission above immediately while an earlier async call is
    // finishing. An OFF superseded before entering the SDK is never sent;
    // an already-entered OFF is followed by ON, never the other way around.
    std::lock_guard<std::mutex>submission_lock(estop_submission_mutex_);
    {std::lock_guard<std::mutex>lock(mutex);
      if(out.transaction!=safety.transaction){
        write->finish(std::make_error_code(std::errc::operation_canceled));
        out.sent=false;out.success=false;out.message="software_estop_request_superseded";return out;
      }
      if(safety.confirmed_transaction==out.transaction){
        write->finish({});out.sent=false;
        out.message="软件急停目标已由机器人状态确认；未重复发送，不恢复旧导航权限或速度";return out;
      }
    }
    // No captured Node: late completion only finishes this transaction's
    // lifetime-safe mailbox. A superseded write cannot fail a newer target.
    const auto error=sdk_->SoftEmergencyStop(on,0,[write](const std::error_code&e,std::size_t){write->finish(e);});
    if(error){write->finish(error);std::lock_guard<std::mutex>lock(mutex);
      if(out.transaction==safety.transaction&&execution3_)execution3_->fault("software_estop_sdk_write_failed",wall());
      safety.failed("软件急停发送异常："+error.message(),out.transaction);out.success=false;out.message=error.message();}
    return out;
  }
  d1monitor::shared_estop::State shared_estop_state(){
    std::lock_guard<std::mutex>lock(mutex);return shared_estop_state_locked();
  }
  d1monitor::shared_estop::State shared_estop_state_locked(){
    auto state=shared_estop_robot_;
    state.connected=sdk_&&sdk_->IsConnected();
    state.owned=state.connected&&ownership_.confirmed&&navigation_control_source_==2;
    state.fatal=state.fatal||shared_estop_fatal_.load();
    state.estop_pending=safety.pending;state.estop_target_on=safety.target_on;
    state.estop_transaction=safety.transaction;state.estop_result=safety.result;state.estop_ack=safety.ack;
    state.manual=shared_manual_.json();
    if(!shared_manual_.active()&&execution3_&&!execution3_->executionId().empty()&&!execution3_->stopping())state.manual["phase"]="navigation";
    return state;
  }
  void shared_estop_request(uint64_t client,const Json&message){
    using Protocol=d1monitor::shared_estop::Protocol;
    const auto request=shared_estop_protocol_->receive(client,message);
    auto*server=shared_estop_ipc_.get();if(!server)return;
    if(request.kind==Protocol::Kind::Hello){
      server->send(client,shared_estop_protocol_->hello(shared_estop_state()));
      shared_estop_protocol_->helloQueued(client);return;
    }
    if(request.kind==Protocol::Kind::Ignore)return;
    if(request.kind==Protocol::Kind::Manual){
      d1monitor::shared_manual::Decision decision;
      std::optional<String> handoff;
      {std::lock_guard<std::mutex>lock(mutex);
        const auto prior=shared_manual_.active();
        const auto old_task=execution3_&&!execution3_->executionId().empty()?
          d1monitor::shared_manual::Core::taskKey(execution3_->binding().session_id,execution3_->binding().task_id):std::string();
        decision=shared_manual_.receive(client,shared_estop_protocol_->clientName(client),request,shared_estop_state_locked(),mono(),old_task);
        if(decision.accepted&&!prior&&shared_manual_.active()){
          execution3_->stop("manual_takeover",wall());ownership_.enabled=false;
          manual_handoff_lease_=shared_manual_.lease;manual_handoff_acknowledged_=false;
          manual_handoff_stamp_ns_=d1monitor::execution3::sourceNs(d1monitor::execution3::stamp(wall()));
          String event;event.data=Json{{"schema",1},{"sdk_session",session_},{"lease",manual_handoff_lease_},
            {"takeover_stamp_ns",std::to_string(manual_handoff_stamp_ns_)}}.dump();handoff=std::move(event);
        }
      }
      if(handoff&&manual_handoff_event_)manual_handoff_event_->publish(*handoff);
      if(request.op=="jog"||request.op=="heartbeat")wake_execution_command();else wake_execution_stop();
      server->send(client,shared_estop_protocol_->state(shared_estop_state()));
      if(!decision.pending)server->send(client,shared_estop_protocol_->result(request,decision.accepted,decision.reason));
      return;
    }
    if(request.kind==Protocol::Kind::Reject||request.kind==Protocol::Kind::LocalOnly){
      server->send(client,shared_estop_protocol_->result(request,request.kind==Protocol::Kind::LocalOnly,request.reason));return;}
    const auto submission=request_soft_estop(request.on);
    server->send(client,shared_estop_protocol_->state(shared_estop_state()));
    if(!submission.sent||!submission.success){server->send(client,shared_estop_protocol_->result(request,submission.success,submission.message));return;}
    std::lock_guard<std::mutex>lock(shared_estop_replies_mutex_);
    if(shared_estop_replies_.size()>=16){
      // The safety request remains submitted; only reply waiting is bounded.
      server->send(client,shared_estop_protocol_->result(request,true,"急停请求已提交；写结果尚未确认，不代表实测停稳"));return;}
    shared_estop_replies_.push_back({client,request,submission.write,mono()});
  }
  void shared_estop_tick(){
    auto*server=shared_estop_ipc_.get();if(!server)return;
    const auto state=shared_estop_state();
    for(const auto client:shared_estop_protocol_->clients())server->send(client,shared_estop_protocol_->state(state));
    std::lock_guard<std::mutex>lock(shared_estop_replies_mutex_);
    for(auto it=shared_estop_replies_.begin();it!=shared_estop_replies_.end();){
      const auto outcome=it->write->read();
      if(!outcome.first&&mono()-it->started<=5.){++it;continue;}
      const bool ok=outcome.first&&!outcome.second;
      const auto reason=!outcome.first?"急停请求写结果未知；请核对实机，不代表实测停稳":
        outcome.second?"急停请求写失败："+outcome.second.message():"急停请求写完成；等待机器人急停状态，不代表实测停稳";
      server->send(it->client,shared_estop_protocol_->result(it->request,ok,reason));
      it=shared_estop_replies_.erase(it);
    }
  }
  // This runs ONLY in execution_writer_, alongside navigation Move. IPC and
  // vendor callbacks never call posture, speed or Move themselves.
  void shared_manual_tick(std::unique_lock<std::mutex>&lock,bool urgent_only,std::optional<double>&nonzero_submitted){
    using namespace d1monitor::shared_manual;
    auto& m=shared_manual_;const double now=mono();m.expire(now);
    const auto reply=[this](const Pending&p,bool ok,const std::string&why){
      if(shared_estop_ipc_)shared_estop_ipc_->send(shared_manual_.client,
        shared_estop_protocol_->result(p.request,ok,why));
    };
    if(manual_move_write_){
      const auto outcome=manual_move_write_->read();
      if(outcome.first){
        if(outcome.second){m.uncertain=true;m.veto("manual_move_write_failed");}
        else if(manual_move_zero_&&!m.held(now)&&!m.zero_due)m.zero_confirmed=true;
        manual_move_write_.reset();
      }else if(now-manual_move_sent_>1.){m.uncertain=true;m.veto("manual_move_write_unconfirmed");}
    }
    // A software estop may have retired the manual owner while suppressing
    // its zero write. Once OFF is observed, drain that original zero even in
    // Idle; only its real callback clears the debt. No held intent survives.
    if(!m.needsWriterService())return;
    auto state=shared_estop_state_locked();
    if(!state.connected||safety.replay||state.soft==2||state.hardware==2||state.fatal){
      if(m.pending){reply(*m.pending,false,"机器人连接、急停或故障状态阻止操作");m.pending.reset();}
      m.veto("manual_robot_unavailable");
    }
    if(manual_action_write_&&!m.pending){
      // An emergency/detach may retire the UI request, but its vendor write
      // still has to drain before another posture or owner is admitted.
      const auto outcome=manual_action_write_->read();
      if(outcome.first){if(outcome.second)m.uncertain=true;manual_action_write_.reset();}
    }
    if(m.phase==Core::Phase::Acquiring){
      if(m.pending&&now-m.pending->started>=10.){
        reply(*m.pending,false,"手动接管超时；没有重新连接或自动重试");m.pending.reset();m.veto("manual_takeover_timeout");
      }else{
        // Keep the actual last task's reports before the terminal identity
        // transition. Its ACK and measured stop are never a manual lease.
        bool retired=false;
        if(!execution3_->executionId().empty()){
          const auto stop=execution3_->report(wall());const auto retirement=execution3_->retirement(wall());
          if(execution3_->completeManualHandoff(wall())){manual_retired_stop_=stop;manual_retired_report_=retirement;retired=true;}
        }else retired=execution3_->completeManualHandoff(wall());
        if(retired){
          if(ownership_.owns(now)&&navigation_control_source_==2){
            m.phase=Core::Phase::Manual;m.reason.clear();
            if(m.pending){reply(*m.pending,true,"manual_control_ready");m.pending.reset();}
          }else if(!m.take_requested){
            m.take_requested=true;
            if(!ownership_.beginAuthorizedAttempt(now)){
              if(m.pending){reply(*m.pending,false,"SDK 控制权事务不可用");m.pending.reset();}m.veto("manual_takeover_unavailable");
            }
          }
          if(m.phase==Core::Phase::Acquiring&&!urgent_only&&ownership_.due(now)){
            ownership_.enabled=false;const auto id=ownership_.request_id;
            const auto transaction=execution_writes_.beginCall();
            if(!transaction){m.uncertain=true;m.veto("sdk_pending_write_limit");}
            else{d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
              const auto submit=[this,id,transaction]{return sdk_->TakeControl(0,[this,id,transaction](const std::error_code&e,std::size_t){
                transaction->finish([&]{if(e){std::lock_guard<std::mutex>in(inbox_mutex_);if(!stopping_)owner_in_.push({OwnerKind::Written,mono(),0,id,e.message().substr(0,512)});}});});};
              const auto error=d1monitor::invokeSdkOutsideState(lock,submit);
              if(error){ownership_.written(id,error.message());transaction->complete();}
            }
          }
        }
      }
    }
    state=shared_estop_state_locked();
    if(m.owns()&&!m.deny(state,now,true).empty()&&m.held(now)){m.stop();m.reason=m.deny(state,now,true);}
    if(manual_action_write_&&m.pending){
      const auto outcome=manual_action_write_->read();auto&p=*m.pending;
      std::pair<uint64_t,int>ack;
      {std::lock_guard<std::mutex>ack_lock(manual_ack_mutex_);ack=manual_acks_[static_cast<unsigned>(p.action)];}
      const bool value_match=(p.action!=Action::Mode&&p.action!=Action::Speed)||ack.second==p.value;
      const bool confirmed=outcome.first&&!outcome.second&&ack.first>p.ack_before&&value_match&&
        fresh(state,now)&&state.sequence>p.state_before&&state.received>=p.sent&&matches(p.action,p.value,state);
      if(confirmed||(outcome.first&&outcome.second)||now-p.started>=10.){
        const bool ambiguous=!confirmed;reply(p,confirmed,confirmed?"机器人已确认所选状态":
          outcome.second?"SDK 操作发送失败："+outcome.second.message():"SDK 操作状态未确认，请核对机器人");
        if(ambiguous)m.uncertain=true;
        manual_action_write_.reset();m.pending.reset();
      }
    }
    // Zero follows an actual manual nonzero, not every posture button. In a
    // lying posture SetMode/Stand are legal while Move(0) is not guaranteed.
    const bool can_zero=state.connected&&state.owned&&state.source==2&&state.soft!=2&&state.hardware!=2;
    if(m.zero_due&&!manual_move_write_&&!manual_action_write_&&can_zero){
      m.zero_due=false;manual_move_zero_=true;manual_move_sent_=now;
      manual_move_write_=std::make_shared<d1monitor::shared_estop::WriteResult>();const auto result=manual_move_write_;
      const auto transaction=execution_writes_.beginCall();
      if(!transaction){result->finish(std::make_error_code(std::errc::no_buffer_space));}
      else{d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
        const auto submit=[this,result,transaction]{return sdk_->Move(0.f,0.f,0.f,0,[result,transaction](const std::error_code&e,std::size_t){transaction->finish([&]{result->finish(e);});});};
        const auto error=d1monitor::invokeSdkOutsideState(lock,submit);if(error){result->finish(error);transaction->complete();}
      }
      return;
    }
    if(m.phase==Core::Phase::Releasing&&!manual_move_write_&&!manual_action_write_&&!ownership_.pending&&execution_writes_.pending()==0){
      if(ownership_.uncertain)m.uncertain=true;
      if(m.zero_confirmed||!state.connected||!state.owned||state.soft==2||state.hardware==2){
        if(m.pending){reply(*m.pending,false,"manual_operation_canceled");m.pending.reset();}
        m.releaseComplete();
      }
      return;
    }
    if(!m.owns()||manual_move_write_||manual_action_write_||urgent_only)return;
    if(m.pending&&m.pending->action!=Action::None&&m.zero_confirmed){
      auto&p=*m.pending;const auto why=m.deny(state,now);
      if(!why.empty()){reply(p,false,why);m.pending.reset();return;}
      p.state_before=state.sequence;p.sent=mono();p.submitted=true;
      {std::lock_guard<std::mutex>ack_lock(manual_ack_mutex_);p.ack_before=manual_acks_[static_cast<unsigned>(p.action)].first;}
      const auto action=p.action;const auto value=p.value;
      manual_action_write_=std::make_shared<d1monitor::shared_estop::WriteResult>();const auto result=manual_action_write_;
      const auto transaction=execution_writes_.beginCall();
      if(!transaction){result->finish(std::make_error_code(std::errc::no_buffer_space));return;}
      d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
      const auto submit=[this,result,transaction,action,value]{
        const auto cb=[result,transaction](const std::error_code&e,std::size_t){transaction->finish([&]{result->finish(e);});};
        switch(action){case Action::Mode:return sdk_->SetMode(value,0,cb);case Action::Stand:return sdk_->StandUp(0,cb);
          case Action::Lie:return sdk_->LieDown(0,cb);case Action::Crawl:return sdk_->Crawl(0,cb);case Action::Lock:return sdk_->Locked(0,cb);
          case Action::Speed:return sdk_->SetSpeed(value,0,cb);default:return std::make_error_code(std::errc::invalid_argument);}
      };
      const auto error=d1monitor::invokeSdkOutsideState(lock,submit);if(error){result->finish(error);transaction->complete();}
      return;
    }
    if(!m.pending&&m.held(now)&&m.deny(state,now,true).empty()){
      const auto velocity=m.velocity(now);manual_move_zero_=false;manual_move_sent_=now;
      manual_move_write_=std::make_shared<d1monitor::shared_estop::WriteResult>();const auto result=manual_move_write_;
      const auto transaction=execution_writes_.beginCall();if(!transaction){result->finish(std::make_error_code(std::errc::no_buffer_space));return;}
      d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
      nonzero_submitted=mono();
      const auto submit=[this,result,transaction,velocity]{return sdk_->Move(velocity.lateral,velocity.forward,velocity.yaw,0,
        [result,transaction](const std::error_code&e,std::size_t){transaction->finish([&]{result->finish(e);});});};
      const auto error=d1monitor::invokeSdkOutsideState(lock,submit);if(error){result->finish(error);transaction->complete();}
    }
  }
  // Only with mutex held. Callback vetoes are consumed before arm/accept/send;
  // their ACKs cannot silently restore an old authorization.
  void apply_navigation_events(){
    using Events=d1monitor::NavigationEvents;
    const auto events=navigation_events_.take();
    double software_stop=-1.,hardware_stop=-1.;
    {std::lock_guard<std::mutex>lock(estop_observation_mutex_);
     std::swap(software_stop,software_estop_observed_at_);std::swap(hardware_stop,hardware_estop_observed_at_);}
    if(execution3_){
      if(software_stop>software_release_receipt_)execution3_->softwareEstopObserved(wall());
      if(hardware_stop>=0.)execution3_->fault("hardware_emergency_stop_observed_latched",wall());
    }
    // Before the first grant, an ordinary posture/mode/gear mismatch is
    // admission readiness, not a failed execution. execution_health() still
    // rejects grants until a fresh normal/standing/low-speed state is present.
    // Once an execution was admitted, the same event remains irreversible:
    // do not clear a real veto or let a new task reuse an uncertain writer.
    const bool execution_state_veto=execution3_&&!execution3_->executionId().empty()&&
      (events&Events::UnsafeRobotState);
    if(execution3_&&(execution_state_veto||(events&(Events::ConnectionError|Events::MoveWriteFailure|
        Events::RobotFault|Events::WorkerStopped))))
      execution3_->fault("sdk_callback_safety_veto",wall());
    const bool was_armed=navigation_.armed;
    if(events&Events::ConnectionError)navigation_.disarm("SDK_connection_error");
    if(events&Events::LostControl){
      if(shared_manual_.owns())shared_manual_.veto("manual_control_lost");
      navigation_control_source_=0;
      // A not-yet-confirmed handoff may legitimately still report APP ownership.
      // Keep its existing ordered ACK/state FSM; invalidate an actual old owner
      // immediately, before a delayed robot packet could restore its source.
      if(ownership_.confirmed)ownership_.lost();
      navigation_.disarm("SDK_control_lost");
    }
    if(events&Events::UnsafeRobotState){navigation_robot_.robot_at=-1.;navigation_.disarm("robot_state_changed_rearm_required");}
    if(events&Events::McDisabled)navigation_.disarm("MC_disabled_rearm_required");
    if(events&Events::MoveWriteFailure)navigation_.fail("SDK_velocity_write_failed_latched");
    if(events&Events::RobotFault)navigation_.fail("robot_fault_latched");
    if(events&Events::WorkerStopped)navigation_.fail("telemetry_worker_stopped_latched");
    if((events&Events::EmergencyStop)&&navigation_.enabled&&was_armed)
      navigation_.fail("emergency_stop_observed_latched");
  }
  // Invoked on the ROS executor with mutex held. SDK callbacks only copy data;
  // no network operation runs inside any SDK data callback.
  d1monitor::NavigationHealth navigation_health(double now){
    auto h=navigation_robot_;
    h.connected=sdk_&&sdk_->IsConnected();h.replay=safety.replay;
    h.owned=ownership_.owns(now)&&navigation_control_source_==2;
    h.mc_fresh=mc_.fresh(now)&&mc_.rate_ok(now);
    return h;
  }
  d1monitor::execution3::Health execution_health(){
    const double receipt=mono();
    const auto h=navigation_health(receipt);
    auto result=d1monitor::execution3::robotExecutionHealth(h,ownership_,receipt,
      execution3_&&execution3_->mcSourceFresh(wall(),mc_clock_epoch()),navigation_control_source_);
    // Classification of the original known emergency fact is independent
    // of its freshness. Stale feedback remains a HOLD, not a new hard fault.
    result.software_estop_only=h.software==2&&h.hardware==1;
    if(mode_transition_)mode_transition_->applyHealth(result,wall());
    return result;
  }
  // Access only with the state mutex held. Preserve the exact epoch label,
  // rebuilding its string only when MC disconnect/reconfiguration advances it.
  const std::string& mc_clock_epoch(){
    if(mc_clock_epoch_.empty()||mc_clock_generation_!=mc_.generation){
      mc_clock_epoch_=session_+":"+std::to_string(mc_.generation);mc_clock_generation_=mc_.generation;}
    return mc_clock_epoch_;
  }
  void sync_execution_transactions(){
    if(execution3_)execution3_->setSDKTransactions(execution_writes_.pending()+(ownership_.pending?1u:0u),ownership_.uncertain);
  }
  void wake_execution_stop() noexcept {
    d1monitor::notifyWriterRevision(execution_wait_mutex_,execution_wake_,execution_urgent_revision_);
  }
  void wake_execution_command() noexcept {
    // ROS input callback only (never a vendor IDataCallback). Pair revision
    // publication with the wait mutex so a fresh command cannot lose its wake
    // between the writer's predicate check and condition-variable sleep.
    d1monitor::notifyWriterRevision(execution_wait_mutex_,execution_wake_,execution_command_revision_);
  }
  void raise_navigation_event(d1monitor::NavigationEvents::Event event) noexcept {
    navigation_events_.raise(event);wake_execution_stop();
  }
  std::optional<double> execution_tick(bool urgent_only=false,bool no_fresh_command=false,uint64_t*selected_revision=nullptr){
    if(shutting_down_.load())return {};
    std::optional<double> nonzero_submitted;
    d1monitor::execution3::State state;d1monitor::execution3::Stop stop;
    d1monitor::execution3::Retirement retirement;
    d1monitor::execution3::Stationary stationary;std::optional<d1monitor::execution3::CommitAck>ack;
    std::optional<d1monitor::execution3::TaskFeedback>task_feedback;
    std::optional<d1monitor::execution3::ModeState>mode_state;
    {std::unique_lock<std::mutex>lock(mutex);
    apply_navigation_events();
    const double t=wall();
    sync_execution_transactions();
    if(execution_write_failed_.exchange(false))execution3_->submissionFailed(t);
    {std::lock_guard<std::mutex>ack_lock(execution_ack_mutex_);
      if(execution_write_ack_epoch_==execution3_->epoch()&&execution_write_ack_generation_==execution3_->armGeneration())
        execution3_->writeAcknowledged(execution_write_ack_commit_,t);
      if(execution_zero_ack_epoch_==execution3_->epoch()&&execution_zero_ack_generation_==execution3_->armGeneration())
        execution3_->zeroAcknowledged(execution_zero_ack_write_,execution_zero_ack_at_,t);}
    if(execution_stop_written_epoch_.load()==execution3_->epoch())execution3_->stopWritten(execution_stop_written_at_.load());
    shared_manual_tick(lock,urgent_only,nonzero_submitted);
    if(mode_transition_&&!mode_transition_->fixedStair()){
      {std::lock_guard<std::mutex>ack_lock(mode_ack_mutex_);
        if(mode_ack_in_){mode_transition_->acknowledge(mode_ack_in_->first,mode_ack_in_->second,t);mode_ack_in_.reset();}}
      if(mode_write_failed_.exchange(false))mode_transition_->submitted(false);
      mode_transition_->tick(*execution3_,execution3_->taskLeaseSnapshot(),execution3_->stationaryEvidence(t),t);
      if(!urgent_only&&!execution3_->stopping())if(const auto mode=mode_transition_->takeDispatch(t)){
        mode_expected_.store(*mode,std::memory_order_release);
        const auto transaction=execution_writes_.beginCall();
        if(!transaction)mode_transition_->submitted(false);
        else {d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
          const auto submit=[this,mode,transaction]{return sdk_->SetMode(*mode,0,
            [this,transaction](const std::error_code&e,std::size_t){transaction->finish([&]{if(e){
              mode_write_failed_.store(true);execution_write_failed_.store(true);}});});};
          const auto error=d1monitor::invokeSdkOutsideState(lock,submit);
          if(error){mode_transition_->submitted(false);execution3_->submissionFailed(wall());transaction->complete();}
          apply_navigation_events();
          // Cancellation arriving during SetMode retires the request before
          // any later ACK/feedback can confirm or re-authorize it.
          mode_transition_->tick(*execution3_,execution3_->taskLeaseSnapshot(),execution3_->stationaryEvidence(wall()),wall());
        }
      }
      mode_expected_.store(mode_transition_->expectedSdkMode(),std::memory_order_release);
      const auto*active=mode_transition_->activeModeObserved()?mode_transition_->activeCapability():nullptr;
      mode_active_.store(active?active->sdk_mode:0,std::memory_order_release);
    }
    if(!shared_manual_.active()&&!urgent_only&&execution3_->takeControlOnce()) {
      // Existing ownership core correlates ACK + two RobotState callbacks.
      if(!ownership_.beginAuthorizedAttempt(mono()))
        execution3_->stop("sdk_control_transaction_unavailable",t);
    }
    // Admission under state is the ordering point for this one attempt. The
    // task writer releases state only after registering the in-flight ledger;
    // revocation stays responsive even if the vendor blocks. No auto-retake.
    if(!shared_manual_.active()&&!urgent_only&&!execution3_->stopping()&&ownership_.due(mono())){
      const auto id=ownership_.request_id;ownership_.enabled=false;
      const auto transaction=execution_writes_.beginCall();
      if(!transaction){ownership_.fail("takeover_write_limit","SDK pending write limit",true);
        execution3_->fault("sdk_pending_write_limit",t);}
      else {d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
        const auto submit=[this,id,transaction]{return sdk_->TakeControl(0,[this,id,transaction](const std::error_code&e,std::size_t){
        transaction->finish([&]{if(e){std::lock_guard<std::mutex>lock(inbox_mutex_);if(!stopping_)
          owner_in_.push({OwnerKind::Written,mono(),0,id,e.message().substr(0,512)});}});
        });};
        const auto error=execution3_->taskFollower()?d1monitor::invokeSdkOutsideState(lock,submit):submit();
      if(error){ownership_.written(id,error.message());transaction->complete();}}
      apply_navigation_events();
    }
    // TakeControl can block or invoke callbacks inline. Select speed only
    // AFTER it returns, against the latest revoke, telemetry and source time.
    const double selected_at=wall();
    auto command=execution3_->tick(selected_at,execution_health());
    // The same state lock protects command acceptance and its wake revision.
    // Consume the revision actually selected, not an earlier pre-lock read;
    // a newer command arriving during the vendor call remains pending.
    if(selected_revision)*selected_revision=execution_command_revision_.load(std::memory_order_acquire);
    if(execution3_->stopping())ownership_.enabled=false;
    if(!command||execution3_->stopping())execution_fraction_slew_.reset();
    if((urgent_only||no_fresh_command)&&command&&d1monitor::execution3::planarMotionNonzero(*command))command.reset();
    if(command) {
      const auto mapped=d1monitor::execution3::executionMoveFractions(mode_transition_.get(),execution_acceptance_,
        execution3_->taskFollower(),*command,execution_fraction_slew_,
        {execution3_->executionId(),execution3_->epoch(),execution3_->armGeneration()},mono(),
        execution3_->ordinaryGoalDecelerationSelected(),execution3_->ordinaryYawCoastingSelected());
      if(!mapped){execution3_->fault("fixed_stair_runtime_mapping_unavailable",t);execution_fraction_slew_.reset();return {};}
      const auto fraction=*mapped;
      // Classify the exact Move arguments after mapping and float conversion.
      // The original SI demand remains in Transport and diagnostics; SDK input
      // fractions cannot stand in for measured body velocity or stop evidence.
      const float sdk_lateral=static_cast<float>(fraction.left_right);
      const float sdk_forward=static_cast<float>(fraction.forward_back);
      const float sdk_yaw=static_cast<float>(fraction.yaw);
      const bool actual_nonzero=sdk_lateral!=0.f||sdk_forward!=0.f||sdk_yaw!=0.f;
      const bool actual_zero=!actual_nonzero;
      const bool stop_zero=actual_zero&&execution3_->stopping();
      const auto epoch=execution3_->epoch();
      const auto generation=execution3_->armGeneration();
      const auto commit=execution3_->selectedCommitSequence();
      const auto write=execution3_->nextWriteSequence();
      const auto transaction=execution_writes_.beginCall();
      if(!transaction){execution3_->fault("sdk_pending_write_limit",t);execution_fraction_slew_.reset();}
      else {d1monitor::SDKWriteTransactions::CallReturn call_return(transaction);
      execution_last_mapping_demand_=*command;
      execution_last_mapping_fraction_={sdk_lateral,sdk_forward,sdk_yaw};
      execution_last_mapping_attempt_=wall();
      execution_last_mapping_timing_=execution3_->selectedTaskCommandTiming();
      auto mapping_state=execution3_->state(selected_at);
      execution_last_mapping_reason_=std::move(mapping_state.reason);
      // Bind diagnostics to THIS attempted Move, not a future granted task.
      // These fields and the raw/fraction/time snapshot share the state mutex.
      execution_last_mapping_execution_id_=execution3_->executionId();
      execution_last_mapping_epoch_=epoch;execution_last_mapping_generation_=generation;
      const bool raw_nonzero=command->linear.x!=0.||command->linear.y!=0.||command->angular.z!=0.;
      if(actual_nonzero)++execution_mapped_nonzero_ticks_;else ++execution_mapped_zero_ticks_;
      if(!raw_nonzero){++execution_raw_zero_ticks_;execution_last_raw_zero_reason_=execution_last_mapping_reason_;}
      if(execution_have_raw_classification_&&raw_nonzero&&!execution_last_raw_nonzero_)++execution_raw_restarts_;
      execution_have_raw_classification_=true;execution_last_raw_nonzero_=raw_nonzero;
      const int yaw_sign=sdk_yaw>0.f?1:sdk_yaw<0.f?-1:0;
      if(yaw_sign&&execution_last_nonzero_yaw_sign_&&yaw_sign!=execution_last_nonzero_yaw_sign_)
        ++execution_mapped_yaw_reversals_;
      if(yaw_sign)execution_last_nonzero_yaw_sign_=yaw_sign;
      execution3_->moveAttempted(actual_nonzero);
      if(actual_nonzero)nonzero_submitted=mono();
      const auto captured_execution=execution3_->executionId();
      // Fixed-size passive capture under the existing state lock. No new
      // authorization, queue of commands, formatting or I/O is introduced.
      d1monitor::command_trace::Record trace;
      trace.attempt=execution_trace_.nextAttempt();trace.write=write;trace.commit=commit;
      trace.curve=mapping_state.applied_trajectory_id;
      trace.selected_ns=static_cast<int64_t>(selected_at*1e9);
      trace.selected=execution_last_mapping_timing_.present;
      trace.sequence=trace.selected?execution_last_mapping_timing_.sequence:0;
      trace.source_ns=trace.selected?d1monitor::execution3::sourceNs(execution_last_mapping_timing_.source):0;
      trace.body_ns=trace.selected?d1monitor::execution3::sourceNs(execution_last_mapping_timing_.body_source):0;
      trace.deadline_ns=trace.selected?d1monitor::execution3::sourceNs(execution_last_mapping_timing_.valid_until):0;
      const auto& expiry=execution3_->taskCommandExpiryView();
      if(!trace.selected&&execution_last_mapping_reason_=="waiting_local_command"&&expiry.execution_id==captured_execution){
        trace.last_expiry_sequence=expiry.sequence;trace.expiry_count=expiry.count;
        trace.expiry_ns=static_cast<int64_t>(expiry.observed_at*1e9);
        // Only this tick's exact expiry is causal. Later waiting ticks retain
        // last-expiry context, without inventing a new expiry for every zero.
        trace.new_expiry=expiry.observed_at==selected_at;
        trace.cause_sequence=trace.new_expiry?expiry.sequence:0;
        trace.source_ns=d1monitor::execution3::sourceNs(expiry.source);
        trace.body_ns=d1monitor::execution3::sourceNs(expiry.body_source);
        trace.deadline_ns=d1monitor::execution3::sourceNs(expiry.valid_until);
      }
      trace.identity.truncated=d1monitor::command_trace::copy(trace.identity.session,mapping_state.version.session_id);
      trace.identity.truncated|=d1monitor::command_trace::copy(trace.identity.task,mapping_state.version.task_id);
      trace.identity.truncated|=d1monitor::command_trace::copy(trace.identity.execution,captured_execution);
      trace.identity.truncated|=d1monitor::command_trace::copy(trace.identity.sdk,session_);
      trace.identity.epoch=epoch;trace.identity.arm=generation;
      trace.truncated=d1monitor::command_trace::copy(trace.reason,execution_last_mapping_reason_);
      trace.raw_forward=command->linear.x;trace.raw_lateral=command->linear.y;trace.raw_yaw=command->angular.z;
      trace.forward=sdk_forward;trace.lateral=sdk_lateral;trace.yaw=sdk_yaw;
      trace.measured=execution_trace_mc_; // original MC source/receipt, never restamped
      auto write_feedback=execution3_->captureTaskWrite(*command,wall(),
        d1monitor::execution3::SdkMoveFractions{sdk_lateral,sdk_forward,sdk_yaw});
      auto submit=[this,sdk_lateral,sdk_forward,sdk_yaw,stop_zero,actual_zero,epoch,generation,commit,write,transaction,trace,&write_feedback]()mutable{
        trace.begin_ns=d1monitor::command_trace::wallNs();trace.begin_steady_ns=d1monitor::command_trace::steadyNs();
        write_feedback.source_stamp.sec=trace.begin_ns/1000000000LL;
        write_feedback.source_stamp.nanosec=trace.begin_ns%1000000000LL;
        const auto attempt=trace.attempt;
        try {const auto error=sdk_->Move(sdk_lateral,sdk_forward,sdk_yaw,0,
        [this,stop_zero,actual_zero,epoch,generation,commit,write,transaction,attempt](const std::error_code&e,std::size_t){
          const auto ack_at=d1monitor::command_trace::wallNs(),ack_steady=d1monitor::command_trace::steadyNs();
          const bool applied=transaction->finish([&]{
          // Write outcome uncertainty belongs to this unique SDK session,
          // not a recoverable trajectory rejection. The grant service drains
          // this flag BEFORE admitting another execution; the ledger publishes
          // the flag before pending can become zero. A duplicate late callback
          // loses token finalization and cannot fault a future task.
          if(e)execution_write_failed_.store(true);
          else {{std::lock_guard<std::mutex>ack_lock(execution_ack_mutex_);
              if(generation>execution_write_ack_generation_||(generation==execution_write_ack_generation_&&commit>=execution_write_ack_commit_)){
                execution_write_ack_generation_=generation;execution_write_ack_epoch_=epoch;execution_write_ack_commit_=commit;}
              if(actual_zero&&(generation>execution_zero_ack_generation_||(generation==execution_zero_ack_generation_&&write>execution_zero_ack_write_))){
                execution_zero_ack_generation_=generation;execution_zero_ack_epoch_=epoch;execution_zero_ack_write_=write;execution_zero_ack_at_=wall();}}
            if(stop_zero){execution_stop_written_at_.store(wall());execution_stop_written_epoch_.store(epoch);}}
          });
          // ACK is an SDK callback fact, NOT measured physical motion or stop.
          execution_trace_.ack(attempt,write,ack_at,ack_steady,e.value(),e.category().name(),applied);
        });
        trace.end_ns=d1monitor::command_trace::wallNs();trace.end_steady_ns=d1monitor::command_trace::steadyNs();
        trace.error=error.value();trace.truncated|=d1monitor::command_trace::copy(trace.error_category,error.category().name());
        execution_trace_.push(trace);return error;
        }catch(...){trace.end_ns=d1monitor::command_trace::wallNs();trace.end_steady_ns=d1monitor::command_trace::steadyNs();
          trace.exception=true;execution_trace_.push(trace);throw;}
      };
      const auto error=execution3_->taskFollower()?d1monitor::invokeSdkOutsideState(lock,submit):submit();
      // The ledger also holds the vendor stack frame, so a new grant cannot
      // retire this identity on an inline ACK while Move is still blocked.
      // Results never overwrite a newer execution or restore a cleared lease.
      if(captured_execution==execution3_->executionId()&&epoch==execution3_->epoch()&&generation==execution3_->armGeneration())
        execution3_->writeCalled(wall(),!error,actual_nonzero,std::move(write_feedback));
      ++execution_submitted_moves_;
      // An immediate SDK error also faults this writer session. A different
      // execution cannot be admitted while this CallReturn frame is pending.
      if(error){execution3_->submissionFailed(wall());execution_fraction_slew_.reset();transaction->complete();}
      apply_navigation_events();}
    }
    sync_execution_transactions();
    const auto published_at=wall();
    state=execution3_->state(published_at);stop=execution3_->report(published_at);
    retirement=execution3_->retirement(published_at);
    if(execution3_->executionId().empty()&&manual_retired_stop_&&manual_retired_report_){
      stop=*manual_retired_stop_;retirement=*manual_retired_report_;
    }
    // The session-wide manual owner is separate from the last task's final
    // stop/retirement report. Never mask an emergency/fault with a UI phase.
    if(shared_manual_.active()&&!state.fault_latched){
      state.phase=shared_manual_.sdkPhase();state.reason=shared_manual_.reason;
      state.grant_ready=false;
    }
    stationary=execution3_->stationaryEvidence(published_at);ack=execution3_->takeCommitAck();
    // The feedback mailbox deduplicates within the complete writer domain.
    // A scalar write high-water mark here would suppress a new task whose
    // execution-local write sequence has restarted at one.
    if(const auto& latest=execution3_->taskCommandFeedback();latest)task_feedback=*latest;
    if(mode_transition_&&mode_transition_->hasRequest())mode_state=mode_transition_->state(published_at);}
    // Publication cannot delay the writer, not just the state mutex. Latest
    // status is lossy; commit results use a separate fixed queue. An overflow
    // is a safety veto, never silent authorization loss/revival.
    if(task_feedback&&execution_feedback_worker_)execution_feedback_worker_->put(std::move(*task_feedback));
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);
      execution_telemetry_.put(ExecutionTelemetry{std::move(state),std::move(stop),std::move(stationary),std::move(retirement),std::move(mode_state)});
      if(ack&&execution_ack_out_.put(std::move(*ack))==d1monitor::execution3::CommitOutbox::Result::Overflow)
        raise_navigation_event(d1monitor::NavigationEvents::WorkerStopped);}
    telemetry_wake_.notify_one();
    return nonzero_submitted;
  }
  void shutdown_execution(){
    if(!execution3_)return;
    bool required=false;
    {std::lock_guard<std::mutex>lock(mutex);apply_navigation_events();ownership_.enabled=false;
      required=execution3_->beginShutdown(wall())||!shared_manual_.zero_confirmed;shared_manual_.stop();}
    struct Pending {
      std::mutex mutex;std::condition_variable wake;
      d1monitor::execution3::ShutdownStopTransaction transaction;
      double acknowledged_at=0;
      Pending(double at,bool required):transaction(at,required){}
    };
    // ACK lifetime is independent of Monitor: timeout/Disconnect cannot leave
    // a late shutdown callback dereferencing an already destroyed ROS node.
    const auto pending=std::make_shared<Pending>(mono(),required);
    while(true){
      bool allowed=false;
      {std::lock_guard<std::mutex>lock(mutex);const auto h=execution_health();
        const auto remote=shared_estop_state_locked();
        allowed=h.connected&&h.owned&&!h.estop&&(d1monitor::execution3::zeroModeAllowed(h)||
          (shared_manual_.active()&&((remote.mode==1&&remote.motion==5)||(remote.mode==3&&remote.motion==7))));}
      bool send=false;
      {std::lock_guard<std::mutex>lock(pending->mutex);
        if(pending->transaction.done(mono()))break;
        send=pending->transaction.due(mono(),allowed);}
      if(send){
        const auto error=sdk_->Move(0.f,0.f,0.f,0,[pending](const std::error_code&e,std::size_t){
          {std::lock_guard<std::mutex>lock(pending->mutex);pending->transaction.acknowledge(!e);
            if(!e)pending->acknowledged_at=wall();}
          pending->wake.notify_all();});
        if(error){std::lock_guard<std::mutex>lock(pending->mutex);pending->transaction.acknowledge(false);}
      }
      std::unique_lock<std::mutex>lock(pending->mutex);
      pending->wake.wait_for(lock,std::chrono::milliseconds(10));
    }
    bool submitted=false;double at=0;std::string reason;unsigned attempts=0;
    {std::lock_guard<std::mutex>lock(pending->mutex);submitted=pending->transaction.submitted();
      at=pending->acknowledged_at;reason=pending->transaction.reason();attempts=pending->transaction.attempts();}
    d1monitor::execution3::State state;d1monitor::execution3::Stop report;
    {std::lock_guard<std::mutex>lock(mutex);if(submitted)execution3_->stopWritten(at);
      state=execution3_->state(wall());report=execution3_->report(wall());}
    if(rclcpp::ok()){execution_state_->publish(state);execution_stop_->publish(report);}
    // No wait for standstill, automatic re-connect/takeover, posture or mode
    // changes. Normal BT cancellation owns the longer measured-stop drain.
    RCLCPP_WARN(get_logger(),"SDK v3 shutdown: %s; zero_attempts=%u; physical stop is not confirmed by this transaction",
      reason.c_str(),attempts);
  }
  void navigation_tick(){
    d1monitor::dispatch_navigation(mutex,[this]{
      apply_navigation_events();
      return navigation_.tick(navigation_health(mono()),mono());
    },[this](const auto& command){send_navigation(command);});
  }
  // Keep final admission and async Move submission inside the same lock as
  // worker MC/fault/ownership mutations. Every SDK callback is mailbox-only.
  void send_navigation(const d1monitor::NavigationVelocity& command){
    apply_navigation_events();
    const auto h=navigation_health(mono());
    const bool nonzero=command.x!=0||command.y!=0||command.yaw!=0;
    if(nonzero){
      const auto why=navigation_.reason(h,mono());
      if(!why.empty()){navigation_.disarm(why);return;}
      if(navigation_.expire_command_lease(mono())||!navigation_.armed)return;
    }else if(!navigation_.can_stop(h,mono()))return;
    d1monitor::NavigationVelocity fractions;
    try {fractions=d1monitor::NavigationMotion::sdk_percentages(command);}
    catch(const std::invalid_argument& e){navigation_.fail(e.what());return;}
    const auto error=sdk_->Move(static_cast<float>(fractions.y),static_cast<float>(fractions.x),static_cast<float>(fractions.yaw),0,
      [this](const std::error_code& e,std::size_t){if(e)raise_navigation_event(d1monitor::NavigationEvents::MoveWriteFailure);});
    if(error)navigation_.fail("SDK velocity rejected: "+error.message());
    apply_navigation_events(); // inline error/ownership callbacks cannot deadlock
  }
  enum class OwnerKind {Robot,Available,Lost,Ack,Written};
  struct OwnerEvent {OwnerKind kind;double arrival;uint32_t code;uint64_t id;std::string detail;};
  void connect_async(){
    if(shutting_down_.load())return;
    const auto error=sdk_->Connect(ip_,port_,false,[this](const std::error_code& e){
      if(e){raise_navigation_event(d1monitor::NavigationEvents::ConnectionError);std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}
    });
    if(error){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=error;}
  }
  struct McPacket {robot_sdk::MotionData data{};double arrival=0,received=0;};
  struct RobotPacket {robot_sdk::RobotState data;double arrival,received;uint64_t sequence;};
  struct McTelemetry {McPacket packet;uint64_t sequence=0,generation=0;double stamp=0;};
  struct ExecutionTelemetry {d1monitor::execution3::State state;d1monitor::execution3::Stop stop;d1monitor::execution3::Stationary stationary;d1monitor::execution3::Retirement retirement;std::optional<d1monitor::execution3::ModeState> mode;};
  struct DiagnosticTelemetry {String connected,behavior,speed,status;};
  void work_telemetry(){
    double last_mc_trace_received=0.;
    try{for(;;){
      std::optional<RobotPacket> robot;std::optional<McTelemetry> mc;robot_sdk::FaultDatas faults;
      std::optional<ExecutionTelemetry>execution;std::array<d1monitor::execution3::CommitAck,8>commits;size_t commit_count=0;
      std::optional<DiagnosticTelemetry>diagnostic;std::optional<String>joint_diagnostic;
      {std::unique_lock<std::mutex>lock(telemetry_mutex_);
        telemetry_wake_.wait(lock,[this]{return telemetry_stopping_||robot_telemetry_.pending||mc_telemetry_.pending||
          execution_telemetry_.pending||execution_ack_out_.size()||diagnostic_telemetry_.pending||
          joint_diagnostic_telemetry_.pending||!fault_telemetry_.empty();});
        if(telemetry_stopping_)return;
        robot=robot_telemetry_.take();mc=mc_telemetry_.take();faults.swap(fault_telemetry_);execution=execution_telemetry_.take();
        diagnostic=diagnostic_telemetry_.take();joint_diagnostic=joint_diagnostic_telemetry_.take();
        while(commit_count<commits.size()&&execution_ack_out_.pop(commits[commit_count]))++commit_count;}
      // Send transaction results before lossy status/JSON on this consumer.
      // A blocked middleware call still has no software deadline guarantee.
      for(size_t i=0;i<commit_count;++i)execution_commit_->publish(commits[i]);
      if(execution){execution_state_->publish(execution->state);execution_stop_->publish(execution->stop);
        execution_stationary_->publish(execution->stationary);execution_retirement_->publish(execution->retirement);
        if(execution->mode)mode_state_->publish(*execution->mode);}
      if(robot){const auto&d=robot->data;
        publish("robot_state",{{"source",execution3_?"sdk_monitor_execution_v3":navigation_.enabled?"sdk_monitor_navigation":"sdk_monitor_estop"},
          {"read_only",!navigation_.enabled&&!execution3_},{"motion_control_enabled",navigation_.enabled||static_cast<bool>(execution3_)},
          {"received_at_unix",robot->received},{"motion_status",static_cast<int>(d.motion_status)},
          {"sport_mode",static_cast<int>(d.sport_mode)},{"control_source",static_cast<int>(d.control_source)},
          {"speed_level",static_cast<int>(d.speed_level)},{"software_emergency_status",static_cast<int>(d.software_emergency_status)},
          {"hardware_emergency_status",static_cast<int>(d.hardware_emergency_status)},
          {"battery_power_1",d.battery.power1},{"battery_power_2",d.battery.power2},
          {"head_angle",d.head_angle},{"head_direction",static_cast<int>(d.head_direction)},{"mileage",d.mile_data},{"joint_temperatures",d.joint_temps}});}
      if(!faults.empty()){auto output=Json::array();for(const auto&f:faults)
        output.push_back({{"level",static_cast<int>(f.level)},{"code",static_cast<int>(f.code)},{"message",f.message}});
        publish("faults",output);}
      if(mc){const auto&d=mc->packet.data;
        // Retain original acquisition/arrival metadata even if this consumer
        // stalls. This mailbox is lossy telemetry, not a clock or state source.
        publish("velocity",{{"source","sdk_mc"},{"read_only",true},{"frame","sdk_body"},
          {"received_at_unix",mc->packet.received},{"stamp_unix",mc->stamp},{"source_timestamp_ns",std::to_string(d.time_stamp)},
          {"clock_mode","source_delta_host_anchor"},{"clock_approximate",true},{"session",session_},{"generation",mc->generation},
          {"v_body",d.v_body},{"omega_body",d.omega_body},
          {"forward_speed",d.v_body[0]},{"lateral_speed",d.v_body[1]},{"yaw_speed",d.omega_body[2]},{"sequence",mc->sequence}});}
      if(mc&&mc->packet.received-last_mc_trace_received>=1.) {
        last_mc_trace_received=mc->packet.received;const auto& d=mc->packet.data;
        RCLCPP_INFO(get_logger(),"MCTrace sequence=%lu generation=%lu source_ns=%lu received_unix=%.9f v_body=[%.6f,%.6f,%.6f] omega_body=[%.6f,%.6f,%.6f]",
          mc->sequence,mc->generation,static_cast<unsigned long>(d.time_stamp),mc->packet.received,
          d.v_body[0],d.v_body[1],d.v_body[2],d.omega_body[0],d.omega_body[1],d.omega_body[2]);
      }
      // Periodic status belongs to this lossy consumer too. Middleware
      // backpressure cannot block the main executor's ownership/event drain.
      if(diagnostic){pubs_.at("connection_state_text")->publish(diagnostic->connected);
        pubs_.at("behavior_state")->publish(diagnostic->behavior);
        pubs_.at("speed_report_status")->publish(diagnostic->speed);status_->publish(diagnostic->status);}
      if(joint_diagnostic)pubs_.at("joint_state_status")->publish(*joint_diagnostic);
    }}catch(const std::exception&error){
      raise_navigation_event(d1monitor::NavigationEvents::WorkerStopped);
      RCLCPP_ERROR(get_logger(),"Telemetry publication worker stopped, motion veto latched: %s",error.what());
    }
  }
  void process_joint(const d1monitor::JointPacket& packet){
    uint64_t sequence,generation;
    {std::lock_guard<std::mutex> lock(mutex);if(safety.replay)return;}
    {std::lock_guard<std::mutex> lock(joint_mutex_);
     if(!sdk_->IsConnected()||!joints_.sample(packet,mono(),wall()))return;
     sequence=joints_.samples;generation=joints_.generation;}
    auto names=Json::array(),positions=Json::array(),velocities=Json::array(),efforts=Json::array();
    for(size_t i=0;i<packet.count;++i){
      const auto& joint=packet.joints[i];names.push_back(joint.name.data());positions.push_back(joint.position);
      if(packet.has_velocity)velocities.push_back(joint.velocity);
      if(packet.has_effort)efforts.push_back(joint.effort);
    }
    // No stamp_unix/header: the SDK supplies no acquisition time. Consumers
    // cannot claim time-aligned URDF geometry from this receipt-only evidence.
    const Json output={{"source","sdk_joint_state"},{"callback","OnJointStateData"},{"read_only",true},
      {"received_at_unix",packet.received},{"clock_mode","receipt_only"},{"source_timestamp_available",false},
      {"source_timestamp_ns",nullptr},{"units_verified",false},{"urdf_mapping_verified",false},
      {"session",session_},{"generation",generation},{"sequence",sequence},{"receipt_ttl_sec",d1monitor::JointReport::receipt_ttl},
      {"names",names},{"positions",positions},{"velocities",velocities},{"efforts",efforts}};
    // Do not publish a packet retired by disconnect/replay while constructing
    // JSON, or one that exhausted its original receipt lease in the worker.
    {std::lock_guard<std::mutex> lock(mutex);if(safety.replay)return;}
    {std::lock_guard<std::mutex> lock(joint_mutex_);
     const auto now=mono(),now_wall=wall();
     if(!sdk_->IsConnected()||generation!=joints_.generation||
        !joints_.fresh(now)||now_wall<packet.received||
        now_wall-packet.received>=d1monitor::JointReport::receipt_ttl)return;
     publish("joint_states",output);}
  }
  void work_joint(){
    try{for(;;){
      std::optional<d1monitor::JointPacket> packet;
      {std::unique_lock<std::mutex> lock(inbox_mutex_);
       joint_wake_.wait(lock,[this]{return stopping_||joint_in_;});
       if(stopping_)return;
       packet.swap(joint_in_);}
      if(packet)process_joint(*packet);
    }}catch(const std::exception& error){
      RCLCPP_ERROR(get_logger(),"Joint telemetry worker stopped: %s",error.what());
      std::lock_guard<std::mutex> lock(inbox_mutex_);joint_worker_failed_=true;joint_in_.reset();
    }
  }
  void process_robot(const RobotPacket& packet){
    const auto& d=packet.data;
    {std::lock_guard<std::mutex> lock(mutex);
     const auto prior_result=safety.result;const bool prior_pending=safety.pending;
     if(!safety.update(static_cast<int>(d.software_emergency_status),static_cast<int>(d.hardware_emergency_status),packet.arrival))return;
     if(safety.result!=prior_result||safety.pending!=prior_pending)log_soft_estop_target("feedback");
     mc_.robot(packet.arrival);
     navigation_robot_.robot_at=packet.arrival;
     navigation_robot_.software=static_cast<int>(d.software_emergency_status);navigation_robot_.hardware=static_cast<int>(d.hardware_emergency_status);
     navigation_robot_.mode=static_cast<int>(d.sport_mode);navigation_robot_.motion=static_cast<int>(d.motion_status);
     navigation_robot_.speed=static_cast<int>(d.speed_level);navigation_robot_.head=static_cast<int>(d.head_direction);
     navigation_control_source_=static_cast<int>(d.control_source);
     // A view of the same raw RobotState, with its ORIGINAL receipt. Neither
     // IPC hello nor a diagnostic heartbeat refreshes this source timestamp.
     auto&remote=shared_estop_robot_;remote.present=true;remote.received=packet.arrival;++remote.sequence;
     if(remote.source==2&&navigation_control_source_!=2)++remote.loss_sequence;
     remote.source=navigation_control_source_;remote.mode=int(d.sport_mode);remote.motion=int(d.motion_status);
     remote.speed=int(d.speed_level);remote.soft=int(d.software_emergency_status);remote.hardware=int(d.hardware_emergency_status);
     remote.head_direction=int(d.head_direction);remote.robot_mode=int(d.robot_mode);
     remote.actual_forward=d.speed.line;remote.actual_lateral=d.speed.translation;remote.actual_yaw=d.speed.angle;
     remote.actual_speed_present=std::isfinite(remote.actual_forward)&&std::isfinite(remote.actual_lateral)&&std::isfinite(remote.actual_yaw);
     if(mode_transition_)mode_transition_->observeRobot(remote.mode,remote.motion,remote.speed,remote.head_direction,
       remote.source==2,remote.soft==1&&remote.hardware==1,packet.sequence,packet.received,wall());
     confirm_soft_estop_release();}
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);robot_telemetry_.put(packet);}
    telemetry_wake_.notify_one();
  }
  void work() {
    try {for(;;){
      McPacket packet;bool have_mc=false;std::optional<RobotPacket> robot;robot_sdk::FaultDatas faults;
      {std::unique_lock<std::mutex> lock(inbox_mutex_);
       wake_.wait(lock,[this]{return stopping_||mc_in_.size||robot_in_||!faults_in_.empty();});
       if(stopping_)return;
       have_mc=mc_in_.pop(packet);robot.swap(robot_in_);faults.swap(faults_in_);}
      if(robot)process_robot(*robot);
      if(!faults.empty()){
        {std::lock_guard<std::mutex> lock(mutex);for(const auto& f:faults)
          if(static_cast<int>(f.level)==1||static_cast<int>(f.level)==2)navigation_.fail("robot_fault_latched");}
        {std::lock_guard<std::mutex>lock(telemetry_mutex_);
          for(const auto&f:faults){if(fault_telemetry_.size()>=64)break;fault_telemetry_.push_back(f);}}
        telemetry_wake_.notify_one();
      }
      if(!have_mc)continue;
      const auto& d=packet.data;uint64_t sequence,generation;double stamp;
      {std::lock_guard<std::mutex> lock(mutex);
       // The reporter keeps its original accepted clock floor, but must not
       // swallow a backward-source event before the unique writer sees it.
       // An equal duplicate only remains a diagnostic rejection, not a gap.
       if(execution3_&&mc_.sourceOrder(d.time_stamp)==d1monitor::McReport::SourceOrder::Backward)
         execution3_->mcSourceRegressed(wall(),mc_clock_epoch());
       if(safety.replay||!mc_.sample(packet.arrival,packet.received,d.time_stamp,d.v_body,d.omega_body,mono()))continue;
       navigation_.measurement(d.v_body[0],d.v_body[1],packet.arrival);
       sequence=mc_.samples;generation=mc_.generation;stamp=mc_.stamp_unix;
       // Passive scalar cache, using the SAME already-held state lock. No
       // extra SDK request, subscription, source-clock mapping or MC acceptance.
       execution_trace_mc_={d.time_stamp,sequence,generation,static_cast<int64_t>(packet.received*1e9),
         static_cast<int64_t>(packet.arrival*1e9),d.v_body[0],d.omega_body[2]};
       if(execution3_) {
         double v2=0,w2=0;for(int i=0;i<3;++i){v2+=double(d.v_body[i])*d.v_body[i];w2+=double(d.omega_body[i])*d.omega_body[i];}
         const auto h=execution_health();
         execution3_->mc(d.time_stamp,stamp,packet.received,std::sqrt(v2),std::sqrt(w2),wall(),mc_clock_epoch(),
           std::hypot(double(d.v_body[0]),double(d.v_body[1])),std::abs(double(d.v_body[2])),
           d1monitor::execution3::stationaryRuntimeProfileApplies(h,
             mode_transition_&&mode_transition_->reusesGeneralRuntime()));
       }}
      {std::lock_guard<std::mutex>lock(telemetry_mutex_);mc_telemetry_.put(McTelemetry{packet,sequence,generation,stamp});}
      telemetry_wake_.notify_one();
    }} catch(const std::exception& error) {
      // Fail closed: no timer republishing of the last velocity measurement.
      RCLCPP_ERROR(get_logger(),"MC publisher worker stopped: %s",error.what());
      raise_navigation_event(d1monitor::NavigationEvents::WorkerStopped);
      std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;
    }
  }
  void tick(){
    if(shutting_down_.load())return;
    shared_estop_tick();
    const auto connection_state=sdk_->GetConnectionState();
    const bool connected_now=connection_state==robot_sdk::ConnectionState::CONNECTED;
    bool replay;{std::lock_guard<std::mutex> lock(mutex);replay=safety.replay;}
    if(!replay&&connect_retry_.due(static_cast<int>(connection_state),mono()))connect_async();
    if(static_cast<int>(connection_state)!=last_connection_state_){
      RCLCPP_INFO(get_logger(),"SDK connection state: %d -> %d",last_connection_state_,static_cast<int>(connection_state));
      last_connection_state_=static_cast<int>(connection_state);
    }
    bool request_mc=false,request_takeover=false;
    uint64_t request_generation=0,request_id=0;unsigned request_attempt=0;
    std::optional<bool> ack;std::optional<McWrite> mc_write;std::error_code sdk_error;std::optional<EstopAck> estop_ack;uint64_t dropped,owner_dropped;
    std::deque<OwnerEvent> owner_events;
    {std::lock_guard<std::mutex> lock(inbox_mutex_);ack.swap(mc_ack_);mc_write.swap(mc_write_);sdk_error=sdk_error_;sdk_error_.clear();dropped=mc_in_.dropped;
     estop_ack.swap(estop_ack_);
     OwnerEvent event;while(owner_in_.pop(event))owner_events.push_back(std::move(event));owner_dropped=owner_in_.dropped;}
    if(sdk_error){
      last_sdk_error_=sdk_error.message();last_sdk_error_code_=sdk_error.value();last_sdk_error_at_=wall();
      RCLCPP_WARN(get_logger(),"SDK monitor: %s (%d, %s)",sdk_error.message().c_str(),sdk_error.value(),sdk_error.category().name());
    }
    uint64_t takeover_id=0;
    {std::lock_guard<std::mutex> lock(mutex);
    apply_navigation_events();
    if(estop_ack)safety.acknowledge(estop_ack->on,estop_ack->arrival);
    if(estop_write_&&!estop_write_checked_&&estop_write_transaction_==safety.transaction){
      const auto outcome=estop_write_->read();
      if(outcome.first){estop_write_checked_=true;if(outcome.second){
        if(execution3_)execution3_->fault("software_estop_sdk_write_failed",wall());
        safety.failed("软件急停发送异常："+outcome.second.message(),estop_write_transaction_);}}
    }
    if(mc_write)mc_.written(mc_write->generation,mc_write->id,mc_write->error?mc_write->error.message():"");
    ownership_.connection(connected_now,replay);
    if(!connected_now)owner_disconnected_at_=mono();
    for(const auto& event:owner_events){
      if(!connected_now||replay||event.arrival<=owner_disconnected_at_)continue;
      switch(event.kind){
        case OwnerKind::Robot:ownership_.robot(event.code,event.arrival);break;
        case OwnerKind::Available:ownership_.available(event.arrival);break;
        case OwnerKind::Lost:ownership_.lost();break;
        case OwnerKind::Ack:{const bool pending=ownership_.pending;
          ownership_.ack(event.code,event.detail,event.arrival);
          if(execution3_&&pending&&ownership_.result=="takeover_rejected")
            execution3_->stop("sdk_takeover_rejected:"+ownership_.error,wall());
          break;}
        case OwnerKind::Written:ownership_.written(event.id,event.detail);break;
      }
    }
    if(owner_dropped!=last_owner_dropped_){
      ownership_.fail("takeover_timeout","控制权事件队列溢出，状态未知；请核对后重连",true);
      last_owner_dropped_=owner_dropped;
    }
    request_takeover=!execution3_&&ownership_.due(mono());
    takeover_id=ownership_.request_id;
    }
    if(request_takeover){
      if(execution3_){std::lock_guard<std::mutex>lock(mutex);ownership_.enabled=false;}
      const auto id=takeover_id;
      RCLCPP_INFO(get_logger(),"APP released control: requesting TakeControl once, id %lu",id);
      const auto error=sdk_->TakeControl(0,[this,id](const std::error_code& e,std::size_t){
        if(e){std::lock_guard<std::mutex> lock(inbox_mutex_);if(!stopping_)
          owner_in_.push({OwnerKind::Written,mono(),0,id,e.message().substr(0,512)});}
      });
      if(error){std::lock_guard<std::mutex> lock(mutex);ownership_.written(id,error.message());}
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      if(ownership_.reconfigure){
        // A confirmed handoff starts a new telemetry generation. Old queued
        // samples are excluded by the next MC request's arrival-time cutoff.
        if(execution3_)execution3_->telemetryRestartBeforeFirstMotion(wall());
        mc_.disconnect();mc_.robot(ownership_.last_robot);ownership_.reconfigure=false;ack.reset();
      }
      if(ack)mc_.ack(*ack);
      request_mc=mc_.request_due(connected_now,safety.replay,mono(),ownership_.allow_mc_config());
      request_generation=mc_.generation;request_attempt=mc_.attempts;
      request_id=mc_.total_attempts;
    }
    if(request_mc){
      RCLCPP_INFO(get_logger(),"Enabling MC telemetry, attempt %u/%u",request_attempt,mc_.max_attempts);
      const auto error=sdk_->SetMcConfig(true,0,[this,request_generation,request_id](const std::error_code& e,std::size_t){
        std::lock_guard<std::mutex> lock(inbox_mutex_);mc_write_=McWrite{request_generation,request_id,e};
      });
      if(error){std::lock_guard<std::mutex> lock(mutex);mc_.written(request_generation,request_id,error.message());}
    }
    if(joints_.enabled)tick_joint(connected_now,replay);
    d1monitor::Safety safe;d1monitor::McReport::Snapshot report{};d1monitor::Ownership owner_report;
    {std::lock_guard<std::mutex> lock(mutex);safety.connected=connected_now;
      const auto prior_result=safety.result;const bool prior_pending=safety.pending;safety.tick(mono());
      if(safety.result!=prior_result||safety.pending!=prior_pending)log_soft_estop_target("wait_finished");
      safe=safety;report=mc_.snapshot(mono());owner_report=ownership_;}
    String connected;connected.data=safe.connected?"connected":"disconnected";
    d1monitor::NavigationMotion nav;std::string nav_reason,sdk_version;d1monitor::execution3::State execution_state;
    d1monitor::execution3::Stop stop_trace;uint64_t v3_moves=0;
    double mc_capture_lower=0,mc_capture_upper=0,mc_capture_bound=0,mapping_attempt=0;bool mc_source_fresh=false;
    geometry_msgs::msg::Twist mapping_demand;d1monitor::execution3::SdkMoveFractions mapping_fraction;
    d1monitor::execution3::TaskCommandTiming mapping_timing;
    d1monitor::execution3::TaskCommandReorderStats command_reorder;
    d1monitor::execution3::TaskCommandExpiry command_expiry;
    d1monitor::execution3::TaskCommandAdmission command_admission;
    std::string mapping_reason,last_raw_zero_reason,mapping_execution_id,mode_parameter_basis;
    bool mode_runtime_ready=false,mode_measurements_verified=false;
    uint64_t mapping_epoch=0,mapping_generation=0;
    uint64_t mapped_zero_ticks=0,mapped_nonzero_ticks=0,raw_zero_ticks=0,raw_restarts=0,mapped_yaw_reversals=0;
    {std::lock_guard<std::mutex> lock(mutex);nav=navigation_;nav_reason=navigation_.reason(navigation_health(mono()),mono());
      if(execution3_){execution_state=execution3_->state(wall());stop_trace=execution3_->report(wall());mc_capture_lower=execution3_->mcCaptureLowerBound();
        command_reorder=execution3_->taskCommandReorderStats();
        command_expiry=execution3_->taskCommandExpiry();
        command_admission=execution3_->taskCommandAdmission();
        mc_capture_upper=execution3_->mcCaptureUpperBound();mc_capture_bound=execution3_->mcCaptureDelayBound();
        mc_source_fresh=execution3_->mcSourceFresh(wall(),mc_clock_epoch());}
      if(mode_transition_){mode_parameter_basis=mode_transition_->runtimeParameterBasis();
        mode_runtime_ready=mode_transition_->runtimeParametersReady();mode_measurements_verified=mode_transition_->measuredEvidenceVerified();}
      v3_moves=execution_submitted_moves_;sdk_version=sdk_version_;
      mapping_demand=execution_last_mapping_demand_;mapping_fraction=execution_last_mapping_fraction_;mapping_attempt=execution_last_mapping_attempt_;
      mapping_timing=execution_last_mapping_timing_;
      mapping_reason=execution_last_mapping_reason_;last_raw_zero_reason=execution_last_raw_zero_reason_;
      mapping_execution_id=execution_last_mapping_execution_id_;mapping_epoch=execution_last_mapping_epoch_;
      mapping_generation=execution_last_mapping_generation_;
      mapped_zero_ticks=execution_mapped_zero_ticks_;mapped_nonzero_ticks=execution_mapped_nonzero_ticks_;
      raw_zero_ticks=execution_raw_zero_ticks_;raw_restarts=execution_raw_restarts_;mapped_yaw_reversals=execution_mapped_yaw_reversals_;}
    // Logging runs on the existing diagnostic executor, NEVER the ordered SDK
    // writer. A held last sample retains its original attempt timestamp.
    if(command_expiry.count>last_command_expiry_trace_count_&&mono()-last_command_expiry_trace_at_>=1.) {
      last_command_expiry_trace_count_=command_expiry.count;last_command_expiry_trace_at_=mono();
      RCLCPP_INFO(get_logger(),
        "SDKCommandExpiry count=%lu execution=%s command_seq=%lu tick_unix=%.9f source_to_tick_ms=%.3f body_to_tick_ms=%.3f body_remaining_ms=%.3f command_remaining_ms=%.3f last_admission_seq=%lu last_admission_unix=%.9f last_admission_reason=%s",
        command_expiry.count,command_expiry.execution_id.c_str(),command_expiry.sequence,command_expiry.observed_at,
        1000.*(command_expiry.observed_at-d1monitor::execution3::secs(command_expiry.source)),
        1000.*(command_expiry.observed_at-d1monitor::execution3::secs(command_expiry.body_source)),
        1000.*(d1monitor::execution3::secs(command_expiry.body_valid_until)-command_expiry.observed_at),
        1000.*(d1monitor::execution3::secs(command_expiry.valid_until)-command_expiry.observed_at),
        command_expiry.admission_sequence,command_expiry.admission_at,command_expiry.admission_reason);
    }
    if(command_admission.attempts>last_command_admission_trace_count_&&mono()-last_command_admission_trace_at_>=1.) {
      last_command_admission_trace_count_=command_admission.attempts;last_command_admission_trace_at_=mono();
      RCLCPP_INFO(get_logger(),
        "SDKCommandAdmission attempts=%lu accepted=%lu rejected=%lu deferred=%lu command_seq=%lu admission_unix=%.9f reason=%s source_to_admission_ms=%.3f body_to_admission_ms=%.3f remaining_original_deadline_ms=%.3f",
        command_admission.attempts,command_admission.accepted,command_admission.rejected,command_admission.deferred,
        command_admission.sequence,command_admission.at,command_admission.reason,
        1000.*(command_admission.at-d1monitor::execution3::secs(command_admission.source)),
        1000.*(command_admission.at-d1monitor::execution3::secs(command_admission.body_source)),
        1000.*(d1monitor::execution3::secs(command_admission.valid_until)-command_admission.at));
    }
    if(stop_trace.mc_raw_stamp_ns&&stop_trace.stop_submitted) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
        "SDKStopTrace execution=%s metric=%s raw3d=%.6f planar=%.6f vertical=%.6f angular3d=%.6f samples=%u duration=%.6f confirmed=%s physical_verified=%s mc_raw_ns=%lu mc_source_age=%.6f",
        stop_trace.execution_id.c_str(),stop_trace.velocity_metric.c_str(),stop_trace.measured_linear_mps,
        stop_trace.measured_planar_mps,stop_trace.measured_vertical_mps,stop_trace.measured_angular_radps,
        stop_trace.stationary_samples,stop_trace.stationary_duration_sec,stop_trace.measured_stop_confirmed?"true":"false",
        stop_trace.physical_acceptance_verified?"true":"false",stop_trace.mc_raw_stamp_ns,
        wall()-d1monitor::execution3::secs(stop_trace.source_stamp));
    }
    const double unavailable=std::numeric_limits<double>::quiet_NaN();
    const double source_to_attempt_ms=mapping_timing.present?
      1000.*(mapping_attempt-d1monitor::execution3::secs(mapping_timing.source)):unavailable;
    const double body_to_attempt_ms=mapping_timing.present?
      1000.*(mapping_attempt-d1monitor::execution3::secs(mapping_timing.body_source)):unavailable;
    const double remaining_original_deadline_ms=mapping_timing.present?
      1000.*(d1monitor::execution3::secs(mapping_timing.valid_until)-mapping_attempt):unavailable;
    if(mapping_attempt>last_mapping_trace_attempt_&&mono()-last_mapping_trace_receipt_>=1.) {
      last_mapping_trace_attempt_=mapping_attempt;last_mapping_trace_receipt_=mono();
      RCLCPP_INFO(get_logger(),"SDKCommandTrace execution=%s epoch=%lu arm=%lu attempt_unix=%.9f reason=%s raw=[%.6f,%.6f,%.6f] fraction=[%.6f,%.6f,%.6f] mapped_zero_ticks=%lu mapped_nonzero_ticks=%lu raw_zero_ticks=%lu raw_restarts=%lu mapped_yaw_reversals=%lu last_raw_zero=%s command_timing_present=%s command_seq=%lu source_to_attempt_ms=%.3f body_to_attempt_ms=%.3f remaining_original_deadline_ms=%.3f lease_deferred=%lu lease_completed=%lu lease_discarded=%lu lease_pending=%s",
        mapping_execution_id.c_str(),mapping_epoch,mapping_generation,
        mapping_attempt,mapping_reason.c_str(),mapping_demand.linear.x,mapping_demand.linear.y,mapping_demand.angular.z,
        mapping_fraction.forward_back,mapping_fraction.left_right,mapping_fraction.yaw,
        mapped_zero_ticks,mapped_nonzero_ticks,raw_zero_ticks,raw_restarts,mapped_yaw_reversals,last_raw_zero_reason.c_str(),
        mapping_timing.present?"true":"false",mapping_timing.sequence,source_to_attempt_ms,body_to_attempt_ms,remaining_original_deadline_ms,
        command_reorder.deferred,command_reorder.completed,command_reorder.discarded,command_reorder.pending?"true":"false");
    }
    const bool v3=static_cast<bool>(execution3_);
    d1monitor::ExecutionNavigationSnapshot navigation_execution;
    navigation_execution.enabled=v3;navigation_execution.parameters_valid=execution_acceptance_.valid;
    navigation_execution.grant_ready=execution_state.grant_ready;
    navigation_execution.fault_latched=execution_state.fault_latched;
    navigation_execution.arm_generation=execution_state.sdk_arm_generation;
    navigation_execution.execution_id=execution_state.execution_id;navigation_execution.phase=execution_state.phase;
    navigation_execution.reason=execution_state.reason;navigation_execution.parameter_error=execution_acceptance_.reason;
    navigation_execution.parameter_basis=execution_acceptance_.parameter_basis;
    navigation_execution.kinematic_profile=execution_acceptance_.kinematic_profile;
    navigation_execution.profile_sha256=execution_profile_sha256_;
    navigation_execution.max_planar_mps=execution_acceptance_.max_speed;
    navigation_execution.max_yaw_radps=execution_acceptance_.max_yaw;
    const Json navigation_diag=d1monitor::navigationDiagnostics(nav,nav_reason,navigation_execution,
      report.fresh&&report.rate_ok&&(!v3||mc_source_fresh));
    const bool task_writer=execution3_&&execution3_->taskFollower();
    const auto timing_value=[&](double value){return mapping_timing.present?Json(value):Json(nullptr);};
    const Json attempt_timing={{"present",mapping_timing.present},
      {"basis","software_attempt_admission_not_network_write"},
      {"sequence",mapping_timing.present?Json(mapping_timing.sequence):Json(nullptr)},
      {"source_stamp_ns",mapping_timing.present?Json(d1monitor::execution3::sourceNs(mapping_timing.source)):Json(nullptr)},
      {"body_source_stamp_ns",mapping_timing.present?Json(d1monitor::execution3::sourceNs(mapping_timing.body_source)):Json(nullptr)},
      {"original_valid_until_ns",mapping_timing.present?Json(d1monitor::execution3::sourceNs(mapping_timing.valid_until)):Json(nullptr)},
      {"source_to_attempt_ms",timing_value(source_to_attempt_ms)},
      {"body_to_attempt_ms",timing_value(body_to_attempt_ms)},
      {"remaining_original_deadline_ms",timing_value(remaining_original_deadline_ms)}};
    const Json execution_diag={{"enabled",v3},{"protocol",task_writer?"task_follower_v1":"curve_cas_v3"},{"execution_policy_schema",1},{"phase",execution_state.phase},{"reason",execution_state.reason},
      {"grant_ready",execution_state.grant_ready},{"control_owned",execution_state.control_owned},
      {"execution_id",execution_state.execution_id},{"control_epoch",execution_state.control_epoch},
      {"sdk_arm_generation",execution_state.sdk_arm_generation},{"move_submissions",v3_moves},
      {"command_lease_reorder",{{"deferred",command_reorder.deferred},{"completed",command_reorder.completed},
        {"discarded",command_reorder.discarded},{"pending",command_reorder.pending},{"capacity",1}}},
      {"acceptance_record_valid",execution_acceptance_.valid},{"runtime_parameters_valid",execution_acceptance_.valid},
      {"physical_acceptance_verified",execution_physical_acceptance_verified_},
      {"actuation_runtime_parameter_basis",mode_parameter_basis},{"actuation_runtime_parameters_ready",mode_runtime_ready},
      {"actuation_measured_evidence_verified",mode_measurements_verified},
      {"runtime_profile_sha256",execution_profile_sha256_},
      {"parameter_basis",execution_acceptance_.parameter_basis},{"acceptance_reason",execution_acceptance_.reason},
      {"sdk_input_mapping",{{"kind",execution_acceptance_.sdk_input_mapping.enabled?d1monitor::execution3::sdkInputMappingKindFor(execution_acceptance_.sdk_input_mapping):"legacy_linear_scale"},
        {"physical_speed_calibrated",execution_physical_acceptance_verified_},
        {"last_attempt_stamp",mapping_attempt},
        {"last_attempt_identity",{{"execution_id",mapping_execution_id},{"control_epoch",mapping_epoch},
          {"sdk_arm_generation",mapping_generation}}},
        {"last_attempt_timing",attempt_timing},
        {"last_command_reason",mapping_reason},{"last_raw_zero_reason",last_raw_zero_reason},
        {"mapped_zero_ticks",mapped_zero_ticks},{"mapped_nonzero_ticks",mapped_nonzero_ticks},
        {"raw_zero_ticks",raw_zero_ticks},{"raw_restarts",raw_restarts},{"mapped_yaw_reversals",mapped_yaw_reversals},
        {"last_original_demand",{{"vx_mps",mapping_demand.linear.x},{"vy_mps",mapping_demand.linear.y},{"yaw_radps",mapping_demand.angular.z}}},
        {"last_sdk_fraction",{{"left_right",mapping_fraction.left_right},{"forward_back",mapping_fraction.forward_back},{"yaw",mapping_fraction.yaw}}}}},
      {"mc_clock_basis","source_delta_host_anchor_approximate"},{"mc_clock_approximate",true},
      {"mc_capture_lower_bound_unix",mc_capture_lower},{"mc_capture_upper_bound_unix",mc_capture_upper},
      {"mc_capture_delay_bound_s",mc_capture_bound},
      {"mc_accepted_source_fresh",mc_source_fresh},{"mc_arrival_fresh",report.fresh},
      {"writer_period_s",.05},{"writer_last_duration_s",execution_last_duration_s_.load()},
      {"writer_deadlines_skipped",execution_writer_skipped_.load()},{"writer_overruns",execution_writer_overruns_.load()},
      {"scheduling_isolation","dedicated_input_executor_steady_writer_latest_telemetry"},
      {"independent_physical_stop_verified",false},
      {"fault_latched",execution_state.fault_latched}};
    Json behavior_data={{"fsm_state",v3?execution_state.phase:nav.enabled?(nav.armed?"NAVIGATION_ARMED":"NAVIGATION_LOCKED"):"MONITOR_ONLY"},{"telemetry_only",!nav.enabled&&!v3},{"motion_control_enabled",nav.enabled||v3},{"control_adapter",task_writer?"monitor_task_writer_v1":v3?"monitor_execution_v3":nav.enabled?"monitor_navigation_v1":"monitor_estop_v1"},
      {"execution_v3",execution_diag},{"sdk_version",sdk_version},
      {"ready_for_navigation",v3?Json(execution_state.grant_ready):nav.enabled?Json(nav_reason.empty()):Json(nullptr)},{"sdk_has_control",owner_report.owns(mono())},{"ownership_state",owner_report.state(mono())},{"replay_latched",safe.replay},{"sdk_commands_sent",safe.sent+nav.sent+v3_moves},
      {"fault_latched",navigation_diag.at("navigation_fault_latched")},{"requires_review",navigation_diag.at("navigation_fault_latched")},
      {"estop_result",safe.result},{"estop_ack",safe.ack},{"estop_error",safe.error},{"estop_pending",safe.pending}};
    behavior_data.update(navigation_diag);String behavior;behavior.data=behavior_data.dump();
    const auto now=mono();const auto owner_state=owner_report.state(now);
    const auto mc_state=!report.fresh&&connected_now&&!replay&&!owner_report.allow_mc_config()?owner_state:report.state;
    const Json owner={{"enabled",owner_report.enabled},{"state",owner_state},{"confirmed",owner_report.owns(now)},
      {"acknowledged",owner_report.acknowledged},{"state_confirmations",owner_report.confirmations},{"control_source",owner_report.control_source},
      {"requests",owner_report.total_requests},{"error",owner_report.error},{"automatic_retry",false}};
    if(owner_state!=last_owner_state_){
      RCLCPP_INFO(get_logger(),"SDK ownership: %s",owner_state.c_str());last_owner_state_=owner_state;
    }
    if(mc_state!=last_mc_state_){
      RCLCPP_INFO(get_logger(),"MC stream: %s, received %.1f Hz, samples %lu",mc_state.c_str(),report.observed_hz,report.samples);
      last_mc_state_=mc_state;
    }
    // Stable ROS topic name for existing Web/Foxglove subscriptions; source is explicit.
    String speed;speed.data=Json{{"source","sdk_mc"},{"callback","OnMcData"},{"requested",report.total_attempts>0},
      {"expected_hz",report.expected_hz},{"minimum_hz",report.minimum_hz},{"maximum_hz",report.maximum_hz},
      {"acknowledged",report.acknowledged},{"ack_on",report.ack_on},
      {"samples",report.samples},{"invalid_samples",report.invalid_samples},{"timestamp_rejections",report.timestamp_rejections},
      {"stale_samples",report.stale_samples},{"queue_dropped",dropped},
      {"stream_fresh",report.fresh},{"observed_hz",report.observed_hz},{"source_hz",report.source_hz},{"rate_ok",report.rate_ok},
      {"clock_mode","source_delta_host_anchor"},{"clock_approximate",true},
      {"state",mc_state},{"attempts",report.attempts},{"max_attempts",report.max_attempts},
      {"total_attempts",report.total_attempts},{"retry_cycles",report.retry_cycles},{"next_retry_sec",owner_report.allow_mc_config()?report.next_retry_in:-1.},
      {"ownership",owner},
      {"connection_state",static_cast<int>(connection_state)},{"connect_attempts",connect_retry_.attempts},
      {"last_sdk_error",last_sdk_error_},{"last_sdk_error_code",last_sdk_error_code_},{"last_sdk_error_at",last_sdk_error_at_},
      {"write_complete",report.write_complete},{"write_error",report.write_error},{"received_at_unix",wall()}}.dump();
    Json status_data={{"session",session_},{"service_prefix","/d1max/monitor/s_"+session_},{"mode",safe.replay?"replay":"monitor"},
      {"motion_control_enabled",nav.enabled||v3},
      {"execution_v3",execution_diag},{"sdk_version",sdk_version},
      {"joint_state_enabled",joints_.enabled},
      {"ownership",owner},{"safety_available",safe.available()},{"wall_time",wall()}};
    status_data.update(navigation_diag);String status;status.data=status_data.dump();
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);diagnostic_telemetry_.put(
      DiagnosticTelemetry{std::move(connected),std::move(behavior),std::move(speed),std::move(status)});}
    telemetry_wake_.notify_one();
  }
  void tick_joint(bool connected,bool replay){
    std::optional<JointAck> ack;std::optional<McWrite> written;uint64_t invalid,dropped;bool failed;
    {std::lock_guard<std::mutex> lock(inbox_mutex_);
     ack.swap(joint_ack_);written.swap(joint_write_);invalid=joint_invalid_;dropped=joint_dropped_;failed=joint_worker_failed_;}
    bool request;uint64_t generation,id;
    {std::lock_guard<std::mutex> lock(joint_mutex_);
     joints_.connection(connected&&!failed,replay,mono());
     if(ack)joints_.ack(ack->on,ack->arrival);
     if(written)joints_.written(written->generation,written->id,written->error?written->error.message():"");
     request=joints_.request_due(mono());generation=joints_.generation;id=joints_.total_attempts;}
    if(request){
      // Telemetry configuration only, independent of ownership and MC config.
      const auto error=sdk_->SetJointStateConfig(true,0,[this,generation,id](const std::error_code& e,std::size_t){
        std::lock_guard<std::mutex> lock(inbox_mutex_);if(!stopping_)joint_write_=McWrite{generation,id,e};
      });
      if(error){std::lock_guard<std::mutex> lock(joint_mutex_);joints_.written(generation,id,error.message());}
    }
    d1monitor::JointReport report;{std::lock_guard<std::mutex> lock(joint_mutex_);report=joints_;}
    const auto now=mono();
    String status;status.data=Json{{"source","sdk_joint_state"},{"callback","OnJointStateData"},{"read_only",true},
      {"enabled",true},{"state",failed?"worker_failed":report.state(now)},{"stream_fresh",report.fresh(now)},{"worker_failed",failed},
      {"clock_mode","receipt_only"},{"source_timestamp_available",false},{"units_verified",false},{"urdf_mapping_verified",false},
      {"session",session_},{"generation",report.generation},{"samples",report.samples},
      {"last_sample_received_at_unix",report.last_received>0?Json(report.last_received):Json(nullptr)},
      {"receipt_ttl_sec",d1monitor::JointReport::receipt_ttl},{"invalid_samples",invalid},{"stale_samples",report.stale_samples},{"queue_dropped",dropped},
      {"attempts",report.attempts},{"max_attempts",d1monitor::JointReport::max_attempts},{"total_attempts",report.total_attempts},
      {"acknowledged",report.acknowledged},{"ack_on",report.ack_on},{"write_complete",report.write_complete},{"write_error",report.write_error},
      {"received_at_unix",wall()}}.dump();
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);joint_diagnostic_telemetry_.put(std::move(status));}
    telemetry_wake_.notify_one();
  }
  std::string session_,last_mc_state_,ip_,port_,last_sdk_error_,sdk_version_;d1monitor::McReport mc_;
  d1monitor::ConnectRetry connect_retry_;int last_connection_state_=-1,last_sdk_error_code_=0;double last_sdk_error_at_=0.;
  std::atomic<bool>shutting_down_{false};
  bool shared_estop_enabled_=true;std::string shared_estop_path_;
  std::unique_ptr<d1monitor::shared_estop::Protocol>shared_estop_protocol_;
  d1monitor::shared_manual::Core shared_manual_;
  std::mutex manual_ack_mutex_;
  std::array<std::pair<uint64_t,int>,7>manual_acks_{};
  uint64_t manual_ack_sequence_=0;
  std::shared_ptr<d1monitor::shared_estop::WriteResult>manual_move_write_,manual_action_write_;
  bool manual_move_zero_=false;double manual_move_sent_=0;
  std::optional<d1monitor::execution3::Stop>manual_retired_stop_;
  std::optional<d1monitor::execution3::Retirement>manual_retired_report_;
  rclcpp::Publisher<String>::SharedPtr manual_handoff_event_;
  rclcpp::Subscription<String>::SharedPtr manual_handoff_ack_;
  std::string manual_handoff_lease_;int64_t manual_handoff_stamp_ns_=0;
  bool manual_handoff_acknowledged_=true;
  std::unique_ptr<d1monitor::shared_estop::IPC>shared_estop_ipc_;
  d1monitor::shared_estop::State shared_estop_robot_;std::atomic<bool>shared_estop_fatal_{false};
  std::mutex estop_observation_mutex_;double software_estop_observed_at_=-1.,hardware_estop_observed_at_=-1.;
  std::mutex estop_submission_mutex_;
  uint64_t software_release_notified_=0;double software_release_receipt_=-1.;
  struct SharedEstopReply {uint64_t client;d1monitor::shared_estop::Protocol::Request request;
    std::shared_ptr<d1monitor::shared_estop::WriteResult>write;double started;};
  std::mutex shared_estop_replies_mutex_;std::deque<SharedEstopReply>shared_estop_replies_;
  d1monitor::Ownership ownership_;std::string last_owner_state_;
  d1monitor::NavigationMotion navigation_;d1monitor::NavigationHealth navigation_robot_;int navigation_control_source_=0;
  d1monitor::NavigationEvents navigation_events_;
  struct McWrite {uint64_t generation,id;std::error_code error;};
  struct JointAck {bool on;double arrival;};
  d1monitor::JointReport joints_;std::optional<d1monitor::JointPacket> joint_in_;
  std::optional<JointAck> joint_ack_;std::optional<McWrite> joint_write_;
  uint64_t joint_invalid_=0,joint_dropped_=0;
  std::mutex joint_mutex_;std::condition_variable joint_wake_;std::thread joint_worker_;bool joint_worker_failed_=false;
  struct EstopAck {bool on;double arrival;};
  std::optional<McWrite> mc_write_;std::optional<EstopAck>estop_ack_;
  std::shared_ptr<d1monitor::shared_estop::WriteResult>estop_write_;
  uint64_t estop_write_transaction_=0;bool estop_write_checked_=false;
  d1monitor::Inbox<OwnerEvent,32> owner_in_;uint64_t last_owner_dropped_=0;double owner_disconnected_at_=-1.;
  std::mutex inbox_mutex_;std::condition_variable wake_;std::thread worker_;bool stopping_=false;
  std::mutex telemetry_mutex_;std::condition_variable telemetry_wake_;std::thread telemetry_worker_;bool telemetry_stopping_=false;
  d1monitor::LatestMailbox<RobotPacket>robot_telemetry_;d1monitor::LatestMailbox<McTelemetry>mc_telemetry_;
  d1monitor::LatestMailbox<ExecutionTelemetry>execution_telemetry_;
  d1monitor::LatestMailbox<DiagnosticTelemetry>diagnostic_telemetry_;
  d1monitor::LatestMailbox<String>joint_diagnostic_telemetry_;
  d1monitor::execution3::CommitOutbox execution_ack_out_;
  std::string mc_clock_epoch_;uint64_t mc_clock_generation_=0;
  robot_sdk::FaultDatas fault_telemetry_;
  d1monitor::Inbox<McPacket,128> mc_in_;std::optional<RobotPacket> robot_in_;
  robot_sdk::FaultDatas faults_in_;std::optional<bool> mc_ack_;std::error_code sdk_error_;
  std::map<std::string,rclcpp::Publisher<String>::SharedPtr> pubs_;
  rclcpp::Publisher<String>::SharedPtr status_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Subscription<String>::SharedPtr navigation_velocity_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr navigation_arm_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;
  std::unique_ptr<d1monitor::execution3::Transport>execution3_;
  std::unique_ptr<d1monitor::execution3::ModeTransition>mode_transition_;
  d1monitor::execution3::ExecutionModePolicy execution_mode_policy_=d1monitor::execution3::ExecutionModePolicy::SegmentModes;
  rclcpp::Publisher<d1monitor::execution3::ModeState>::SharedPtr mode_state_;
  rclcpp::Subscription<d1monitor::execution3::ModeRequest>::SharedPtr mode_request_;
  std::mutex mode_ack_mutex_;std::optional<std::pair<int,double>>mode_ack_in_;
  std::atomic<bool>mode_write_failed_{false};std::atomic<int>mode_expected_{0},mode_active_{0};
  std::atomic<uint64_t>mode_robot_sequence_{0};
  d1monitor::execution3::Acceptance execution_acceptance_;
  bool execution_physical_acceptance_verified_=false;
  d1monitor::execution3::SdkFractionSlew execution_fraction_slew_;
  d1monitor::command_trace::Trace execution_trace_;
  d1monitor::command_trace::Measurement execution_trace_mc_;
  d1max_diagnostics::BoundedDiagnosticFile execution_trace_file_{"sdk"};
  bool execution_trace_location_logged_=false,execution_trace_failure_logged_=false;
  std::atomic<bool>execution_trace_stopping_{false};
  std::mutex execution_trace_wait_mutex_;std::condition_variable execution_trace_wake_;
  std::thread execution_trace_worker_;
  geometry_msgs::msg::Twist execution_last_mapping_demand_;
  d1monitor::execution3::TaskCommandTiming execution_last_mapping_timing_;
  d1monitor::execution3::SdkMoveFractions execution_last_mapping_fraction_;
  double execution_last_mapping_attempt_=0.;
  std::string execution_last_mapping_reason_,execution_last_raw_zero_reason_;
  std::string execution_last_mapping_execution_id_;
  uint64_t execution_last_mapping_epoch_=0,execution_last_mapping_generation_=0;
  uint64_t execution_mapped_zero_ticks_=0,execution_mapped_nonzero_ticks_=0,execution_raw_zero_ticks_=0,
    execution_raw_restarts_=0,execution_mapped_yaw_reversals_=0;
  bool execution_have_raw_classification_=false,execution_last_raw_nonzero_=false;
  int execution_last_nonzero_yaw_sign_=0;
  double last_mapping_trace_attempt_=0.,last_mapping_trace_receipt_=0.;
  uint64_t last_command_expiry_trace_count_=0;double last_command_expiry_trace_at_=0.;
  uint64_t last_command_admission_trace_count_=0;double last_command_admission_trace_at_=0.;
  std::string execution_profile_sha256_; // Immutable profile actually admitted by this writer.
  uint64_t execution_submitted_moves_=0;
  d1monitor::SDKWriteTransactions execution_writes_;
  std::atomic<bool>execution_write_failed_{false};
  std::atomic<uint64_t>execution_stop_written_epoch_{0};std::atomic<double>execution_stop_written_at_{0};
  std::mutex execution_ack_mutex_;uint64_t execution_write_ack_commit_=0,execution_write_ack_epoch_=0,execution_write_ack_generation_=0;
  uint64_t execution_zero_ack_write_=0,execution_zero_ack_epoch_=0,execution_zero_ack_generation_=0;double execution_zero_ack_at_=0.;
  rclcpp::Publisher<d1monitor::execution3::State>::SharedPtr execution_state_;
  rclcpp::Publisher<d1monitor::execution3::TaskFeedback>::SharedPtr execution_task_feedback_;
  std::unique_ptr<d1monitor::execution3::TaskFeedbackPublication> execution_feedback_worker_;
  rclcpp::Publisher<d1monitor::execution3::Stop>::SharedPtr execution_stop_;
  rclcpp::Publisher<d1monitor::execution3::Retirement>::SharedPtr execution_retirement_;
  rclcpp::Publisher<d1monitor::execution3::CommitAck>::SharedPtr execution_commit_;
  rclcpp::Publisher<d1monitor::execution3::Stationary>::SharedPtr execution_stationary_;
  rclcpp::Service<d1monitor::execution3::Grant>::SharedPtr execution_grant_;
  rclcpp::Subscription<d1monitor::execution3::Permit>::SharedPtr execution_permit_;
  rclcpp::Subscription<d1monitor::execution3::TaskLease>::SharedPtr execution_task_lease_;
  rclcpp::Subscription<d1monitor::execution3::LocalCommand>::SharedPtr execution_local_command_;
  rclcpp::Subscription<d1monitor::execution3::Demand>::SharedPtr execution_demand_;
  rclcpp::Subscription<d1monitor::execution3::MotionProof>::SharedPtr execution_motion_proof_;
  rclcpp::Subscription<d1monitor::execution3::Handoff>::SharedPtr execution_handoff_;
  rclcpp::Subscription<d1monitor::execution3::Prepared>::SharedPtr execution_prepared_;
  rclcpp::Subscription<d1monitor::execution3::CurveProof>::SharedPtr execution_curve_;
  rclcpp::Subscription<d1monitor::execution3::Progress>::SharedPtr execution_progress_;
  rclcpp::Subscription<d1monitor::execution3::Local>::SharedPtr execution_local_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::TaggedBspline>::SharedPtr execution_geometry_;
  rclcpp::CallbackGroup::SharedPtr execution_callbacks_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>execution_executor_;
  std::thread execution_input_worker_,execution_writer_;std::atomic<bool>execution_workers_stopping_{false};
  std::mutex execution_wait_mutex_;std::condition_variable execution_wake_;
  std::atomic<uint64_t>execution_urgent_revision_{0};
  std::atomic<uint64_t>execution_command_revision_{0};
  std::atomic<uint64_t>execution_writer_skipped_{0},execution_writer_overruns_{0};std::atomic<double>execution_last_duration_s_{0};
  rclcpp::TimerBase::SharedPtr timer_,graph_;
  std::shared_ptr<Data> data_;std::shared_ptr<Ack> ack_;std::unique_ptr<robot_sdk::SDKClient> sdk_;
};
void Data::OnRobotStateData(const robot_sdk::RobotState& d){node_->robot(d);}
void Data::OnMcData(const robot_sdk::MotionData& d){node_->mc(d);}
void Data::OnJointStateData(const robot_sdk::JointStateData& d){node_->joint(d);}
void Data::OnFaultData(const robot_sdk::FaultDatas& d){node_->faults(d);}
void Data::OnControlLost(const robot_sdk::ControlLostInfo&){node_->ownership_event(false);}
void Data::OnControlAvailable(const robot_sdk::ControlAvailableInfo&){node_->ownership_event(true);}
void Ack::OnSoftEmergencyStop(bool on){node_->estop_ack(on);}
void Ack::OnMcConfig(bool on){node_->mc_config(on);}
void Ack::OnJointStateConfig(bool on){node_->joint_config(on);}
void Ack::OnTakeControlAck(const robot_sdk::TakeControlAck& ack){node_->ownership_ack(ack);}
void Ack::OnMode(int mode){node_->mode_ack(mode);}
void Ack::OnStandUp(){node_->manual_ack(d1monitor::shared_manual::Action::Stand);}
void Ack::OnLieDown(){node_->manual_ack(d1monitor::shared_manual::Action::Lie);}
void Ack::OnCrawl(){node_->manual_ack(d1monitor::shared_manual::Action::Crawl);}
void Ack::OnLocked(){node_->manual_ack(d1monitor::shared_manual::Action::Lock);}
void Ack::OnSpeed(int speed){node_->manual_ack(d1monitor::shared_manual::Action::Speed,speed);}
#ifndef D1MAX_SHARED_MONITOR_TEST_DRIVER
int main(int argc,char** argv){
  try {
    d1monitor::SessionLease lease;
    rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();node->start_execution_workers();rclcpp::spin(node);node.reset();
    if(rclcpp::ok())rclcpp::shutdown();
  }catch(const std::exception& error){fprintf(stderr,"SDK monitor: %s\n",error.what());return 1;}
}
#endif
