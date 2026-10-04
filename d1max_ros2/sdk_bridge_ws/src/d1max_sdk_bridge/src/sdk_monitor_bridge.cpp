// Status + handoff + one-way e-stop. Optional, disabled-by-default navigation
// velocity transport reuses this SAME SDK session and requires explicit arming.
#include <chrono>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <thread>
#include <random>
#include <iomanip>
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
#include "navigation_dispatch.hpp"
#include "execution_transport_core.hpp"
#include "execution_acceptance.hpp"
#include "execution_shutdown_core.hpp"
#include "execution_scheduler.hpp"
#include "execution_publication_core.hpp"
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
    const bool exec3_enabled=declare_parameter<bool>("execution_v3_enabled",false);
    const auto exec3_record=declare_parameter<std::string>("execution_acceptance_record","");
    const auto exec3_robot=declare_parameter<std::string>("execution_robot_id","");
    const auto exec3_sdk=declare_parameter<std::string>("execution_sdk_version","");
    const auto exec3_calibration=declare_parameter<std::string>("execution_calibration_sha256","");
    const auto exec3_profile=declare_parameter<std::string>("execution_robot_profile_sha256","");
    if(exec3_enabled) {
      if(navigation_.enabled)throw std::invalid_argument("legacy_and_schema3_motion_writers_are_mutually_exclusive");
      ownership_.enabled=false; // Only an explicit grant may request one takeover.
      execution_acceptance_=d1monitor::execution3::validateAcceptance(exec3_record,exec3_robot,exec3_sdk,exec3_calibration,exec3_profile);
      const auto accepted_record_hash=execution_acceptance_.valid?d1monitor::execution3::fileSha256(exec3_record):std::string();
      execution3_=std::make_unique<d1monitor::execution3::Transport>(session_,"live",execution_acceptance_.valid,
        execution_acceptance_.max_speed,execution_acceptance_.max_yaw,accepted_record_hash);
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
      execution_state_=create_publisher<d1monitor::execution3::State>(p+"sdk_state",1);
      execution_stop_=create_publisher<d1monitor::execution3::Stop>(p+"stop_report",1);
      execution_commit_=create_publisher<d1monitor::execution3::CommitAck>(p+"commit_ack",5);
      execution_stationary_=create_publisher<d1monitor::execution3::Stationary>(p+"stationary_evidence",8);
      execution_grant_=create_service<d1monitor::execution3::Grant>(p+"grant",
        [this,accepted_record_hash,exec3_sdk](const std::shared_ptr<d1monitor::execution3::Grant::Request>r,std::shared_ptr<d1monitor::execution3::Grant::Response>s){
          std::lock_guard<std::mutex> lock(mutex);
          if(r->activate&&(!sdk_||sdk_version_!=exec3_sdk)){s->reason="actual_sdk_version_not_bound";return;}
          if(r->activate&&(accepted_record_hash.empty()||r->acceptance_record_sha256!=accepted_record_hash)){
            s->reason="physical_acceptance_record_not_bound";return;}
          *s=execution3_->grant(*r,wall(),execution_health());
        },rmw_qos_profile_services_default,execution_callbacks_);
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
      bool send=false;
      {std::lock_guard<std::mutex> lock(mutex);safety.connected=sdk_->IsConnected();
       if(!safety.available()){response->success=false;response->message="SDK 不可用或已检测到回放";return;}
       if(execution3_)execution3_->fault("software_estop_requested_latched",wall());
       if(navigation_.enabled)navigation_.fail("software_estop_requested_latched");
       else navigation_.disarm("software_estop_requested");
       send=safety.request(mono());response->success=true;
       response->message=safety.triggered(mono())?"机器人已回报软件急停触发；未重复发送":send?"急停请求已提交；等待机器人状态，不代表已确认停止":"正在等待急停状态；未重复发送";
      }
      if(send){
        // The literal true is intentional. There is no release/reset endpoint.
        const auto error=sdk_->SoftEmergencyStop(true,0,[this](const std::error_code& e,std::size_t){if(e){std::lock_guard<std::mutex> lock(inbox_mutex_);estop_write_error_=e;}});
        if(error){std::lock_guard<std::mutex> lock(mutex);safety.failed("急停发送异常："+error.message());response->success=false;response->message=safety.error;}
      }
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
    sdk_=std::make_unique<robot_sdk::SDKClient>([this](const std::error_code& e){if(e){navigation_events_.raise(d1monitor::NavigationEvents::ConnectionError);std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}},config);
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
  }
  void start_execution_workers(){
    // Start only after the complete object exists. A thread allocation failure
    // can then unwind through the normal join/stop destructor, not terminate
    // from a partially constructed object holding joinable threads.
    if(worker_.joinable()||telemetry_worker_.joinable())throw std::logic_error("monitor_workers_already_started");
    worker_=std::thread([this]{work();});
    telemetry_worker_=std::thread([this]{work_telemetry();});
    if(joints_.enabled)joint_worker_=std::thread([this]{work_joint();});
    if(!execution3_)return;
    execution_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    execution_executor_->add_callback_group(execution_callbacks_,get_node_base_interface());
    execution_input_worker_=std::thread([this]{
      try{execution_executor_->spin();}
      catch(const std::exception&error){navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
        RCLCPP_ERROR(get_logger(),"Execution input worker stopped: %s",error.what());}
    });
    execution_writer_=std::thread([this]{
      d1monitor::PeriodicDeadline deadline(mono(),.05);
      while(!execution_workers_stopping_.load()){
        {std::unique_lock<std::mutex>lock(execution_wait_mutex_);
          const auto due=std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(deadline.next())));
          execution_wake_.wait_until(lock,due,[this]{return execution_workers_stopping_.load();});}
        if(execution_workers_stopping_.load())break;
        const auto started=mono();
        try{execution_tick();}
        catch(const std::exception&error){navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
          RCLCPP_ERROR(get_logger(),"Execution writer tick failed, motion veto latched: %s",error.what());}
        const auto completed=mono();execution_last_duration_s_.store(completed-started);
        if(completed-started>.05)++execution_writer_overruns_;
        deadline.complete(completed);execution_writer_skipped_.store(deadline.skipped());
      }
    });
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
    if(static_cast<int>(d.control_source)!=2)navigation_events_.raise(Events::LostControl);
    if(static_cast<int>(d.software_emergency_status)==2||static_cast<int>(d.hardware_emergency_status)==2)
      navigation_events_.raise(Events::EmergencyStop);
    if(static_cast<int>(d.software_emergency_status)!=1||static_cast<int>(d.hardware_emergency_status)!=1||
       static_cast<int>(d.sport_mode)!=1||static_cast<int>(d.motion_status)!=5||
       static_cast<int>(d.speed_level)!=1||static_cast<int>(d.head_direction)!=1)
      navigation_events_.raise(Events::UnsafeRobotState);
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     const auto now=mono();robot_in_=RobotPacket{d,now,wall()};
     owner_in_.push({OwnerKind::Robot,now,static_cast<uint32_t>(d.control_source),0,{}});}
    wake_.notify_one();
  }
  void faults(const robot_sdk::FaultDatas& d){
    // The sticky veto is independent of the diagnostic queue capacity.
    for(const auto& f:d)if(static_cast<int>(f.level)==1||static_cast<int>(f.level)==2){
      navigation_events_.raise(d1monitor::NavigationEvents::RobotFault);break;
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
  void mc_config(bool on){if(!on)navigation_events_.raise(d1monitor::NavigationEvents::McDisabled);std::lock_guard<std::mutex> lock(inbox_mutex_);mc_ack_=on;}
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
  void estop_ack(bool on){if(!on)return;navigation_events_.raise(d1monitor::NavigationEvents::EmergencyStop);std::lock_guard<std::mutex> lock(inbox_mutex_);estop_ack_=true;}
  // SDK callback thread only copies small, bounded events. Network requests,
  // state transitions, JSON and ROS publication run on the ROS executor.
  void ownership_event(bool available){
    if(!available)navigation_events_.raise(d1monitor::NavigationEvents::LostControl);
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     owner_in_.push({available?OwnerKind::Available:OwnerKind::Lost,mono(),0,0,{}});}
  }
  void ownership_ack(const robot_sdk::TakeControlAck& ack){
    std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
    owner_in_.push({OwnerKind::Ack,mono(),ack.error_code,0,ack.reason.substr(0,512)});
  }
 private:
  // Only with mutex held. Callback vetoes are consumed before arm/accept/send;
  // their ACKs cannot silently restore an old authorization.
  void apply_navigation_events(){
    using Events=d1monitor::NavigationEvents;
    const auto events=navigation_events_.take();
    if(execution3_&&(events&(Events::ConnectionError|Events::UnsafeRobotState|Events::MoveWriteFailure|
        Events::RobotFault|Events::WorkerStopped|Events::EmergencyStop)))
      execution3_->fault("sdk_callback_safety_veto",wall());
    const bool was_armed=navigation_.armed;
    if(events&Events::ConnectionError)navigation_.disarm("SDK_connection_error");
    if(events&Events::LostControl){
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
    const auto h=navigation_health(mono());
    return {h.connected&&!h.replay,h.owned,
      navigation_.robot_fresh(h,mono())&&h.mode==1&&h.motion==5&&h.speed==1&&h.head==1,
      h.mc_fresh&&execution3_&&execution3_->mcSourceFresh(wall(),mc_clock_epoch()),
      h.software!=1||h.hardware!=1};
  }
  // Access only with the state mutex held. Preserve the exact epoch label,
  // rebuilding its string only when MC disconnect/reconfiguration advances it.
  const std::string& mc_clock_epoch(){
    if(mc_clock_epoch_.empty()||mc_clock_generation_!=mc_.generation){
      mc_clock_epoch_=session_+":"+std::to_string(mc_.generation);mc_clock_generation_=mc_.generation;}
    return mc_clock_epoch_;
  }
  void execution_tick(){
    if(shutting_down_.load())return;
    d1monitor::execution3::State state;d1monitor::execution3::Stop stop;
    d1monitor::execution3::Stationary stationary;std::optional<d1monitor::execution3::CommitAck>ack;
    {std::lock_guard<std::mutex>lock(mutex);
    apply_navigation_events();
    const double t=wall();
    if(execution_write_failed_.exchange(false))execution3_->submissionFailed(t);
    {std::lock_guard<std::mutex>ack_lock(execution_ack_mutex_);
      if(execution_write_ack_epoch_==execution3_->epoch())execution3_->writeAcknowledged(execution_write_ack_commit_,t);
      if(execution_zero_ack_epoch_==execution3_->epoch())execution3_->zeroAcknowledged(execution_zero_ack_write_,execution_zero_ack_at_,t);}
    if(execution_stop_written_epoch_.load()==execution3_->epoch())execution3_->stopWritten(execution_stop_written_at_.load());
    if(execution3_->takeControlOnce()) {
      // Existing ownership core correlates ACK + two RobotState callbacks.
      ownership_.enabled=true;ownership_.available(mono());
    }
    const auto command=execution3_->tick(t,execution_health());
    if(execution3_->stopping())ownership_.enabled=false;
    // Schema-3 takeover shares the writer's final ordered state check. The
    // diagnostic tick cannot select TakeControl, unlock, then submit it after
    // a concurrent revoke. This remains a single attempt, never auto-retake.
    if(!execution3_->stopping()&&ownership_.due(mono())){
      const auto id=ownership_.request_id;ownership_.enabled=false;
      const auto error=sdk_->TakeControl(0,[this,id](const std::error_code&e,std::size_t){
        if(e){std::lock_guard<std::mutex>lock(inbox_mutex_);if(!stopping_)
          owner_in_.push({OwnerKind::Written,mono(),0,id,e.message().substr(0,512)});}
      });
      if(error)ownership_.written(id,error.message());
      apply_navigation_events();
    }
    if(command) {
      const d1monitor::NavigationVelocity fraction{command->linear.x/execution_acceptance_.forward_scale,0.,
        command->angular.z/execution_acceptance_.yaw_scale};
      const bool stop_zero=command->linear.x==0.&&command->angular.z==0.&&execution3_->stopping();
      const bool actual_zero=command->linear.x==0.&&command->angular.z==0.;
      const auto epoch=execution3_->epoch();
      const auto commit=execution3_->selectedCommitSequence();
      const auto write=execution3_->nextWriteSequence();
      const auto error=sdk_->Move(static_cast<float>(fraction.y),static_cast<float>(fraction.x),static_cast<float>(fraction.yaw),0,
        [this,stop_zero,actual_zero,epoch,commit,write](const std::error_code&e,std::size_t){
          if(e)execution_write_failed_.store(true);
          else {{std::lock_guard<std::mutex>ack_lock(execution_ack_mutex_);
              if(epoch>execution_write_ack_epoch_||(epoch==execution_write_ack_epoch_&&commit>=execution_write_ack_commit_)){
                execution_write_ack_epoch_=epoch;execution_write_ack_commit_=commit;}
              if(actual_zero&&(epoch>execution_zero_ack_epoch_||(epoch==execution_zero_ack_epoch_&&write>execution_zero_ack_write_))){
                execution_zero_ack_epoch_=epoch;execution_zero_ack_write_=write;execution_zero_ack_at_=wall();}}
            if(stop_zero){execution_stop_written_at_.store(wall());execution_stop_written_epoch_.store(epoch);}}
        });
      execution3_->writeCalled(t,!error);
      ++execution_submitted_moves_;
      if(error)execution3_->submissionFailed(t);
    }
    state=execution3_->state(t);stop=execution3_->report(t);
    stationary=execution3_->stationaryEvidence(t);ack=execution3_->takeCommitAck();}
    // Publication cannot delay the writer, not just the state mutex. Latest
    // status is lossy; commit results use a separate fixed queue. An overflow
    // is a safety veto, never silent authorization loss/revival.
    {std::lock_guard<std::mutex>lock(telemetry_mutex_);
      execution_telemetry_.put(ExecutionTelemetry{std::move(state),std::move(stop),std::move(stationary)});
      if(ack&&execution_ack_out_.put(std::move(*ack))==d1monitor::execution3::CommitOutbox::Result::Overflow)
        navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);}
    telemetry_wake_.notify_one();
  }
  void shutdown_execution(){
    if(!execution3_)return;
    bool required=false;
    {std::lock_guard<std::mutex>lock(mutex);apply_navigation_events();ownership_.enabled=false;
      required=execution3_->beginShutdown(wall());}
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
        allowed=h.connected&&h.owned&&h.general&&!h.estop;}
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
      [this](const std::error_code& e,std::size_t){if(e)navigation_events_.raise(d1monitor::NavigationEvents::MoveWriteFailure);});
    if(error)navigation_.fail("SDK velocity rejected: "+error.message());
    apply_navigation_events(); // inline error/ownership callbacks cannot deadlock
  }
  enum class OwnerKind {Robot,Available,Lost,Ack,Written};
  struct OwnerEvent {OwnerKind kind;double arrival;uint32_t code;uint64_t id;std::string detail;};
  void connect_async(){
    if(shutting_down_.load())return;
    const auto error=sdk_->Connect(ip_,port_,false,[this](const std::error_code& e){
      if(e){navigation_events_.raise(d1monitor::NavigationEvents::ConnectionError);std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}
    });
    if(error){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=error;}
  }
  struct McPacket {robot_sdk::MotionData data{};double arrival=0,received=0;};
  struct RobotPacket {robot_sdk::RobotState data;double arrival,received;};
  struct McTelemetry {McPacket packet;uint64_t sequence=0,generation=0;double stamp=0;};
  struct ExecutionTelemetry {d1monitor::execution3::State state;d1monitor::execution3::Stop stop;d1monitor::execution3::Stationary stationary;};
  struct DiagnosticTelemetry {String connected,behavior,speed,status;};
  void work_telemetry(){
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
        execution_stationary_->publish(execution->stationary);}
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
      // Periodic status belongs to this lossy consumer too. Middleware
      // backpressure cannot block the main executor's ownership/event drain.
      if(diagnostic){pubs_.at("connection_state_text")->publish(diagnostic->connected);
        pubs_.at("behavior_state")->publish(diagnostic->behavior);
        pubs_.at("speed_report_status")->publish(diagnostic->speed);status_->publish(diagnostic->status);}
      if(joint_diagnostic)pubs_.at("joint_state_status")->publish(*joint_diagnostic);
    }}catch(const std::exception&error){
      navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
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
     safety.update(static_cast<int>(d.software_emergency_status),static_cast<int>(d.hardware_emergency_status),packet.arrival);
     mc_.robot(packet.arrival);
     navigation_robot_.robot_at=packet.arrival;
     navigation_robot_.software=static_cast<int>(d.software_emergency_status);navigation_robot_.hardware=static_cast<int>(d.hardware_emergency_status);
     navigation_robot_.mode=static_cast<int>(d.sport_mode);navigation_robot_.motion=static_cast<int>(d.motion_status);
     navigation_robot_.speed=static_cast<int>(d.speed_level);navigation_robot_.head=static_cast<int>(d.head_direction);
     navigation_control_source_=static_cast<int>(d.control_source);}
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
       if(execution3_) {
         double v2=0,w2=0;for(int i=0;i<3;++i){v2+=d.v_body[i]*d.v_body[i];w2+=d.omega_body[i]*d.omega_body[i];}
         execution3_->mc(d.time_stamp,stamp,packet.received,std::sqrt(v2),std::sqrt(w2),wall(),mc_clock_epoch());
       }}
      {std::lock_guard<std::mutex>lock(telemetry_mutex_);mc_telemetry_.put(McTelemetry{packet,sequence,generation,stamp});}
      telemetry_wake_.notify_one();
    }} catch(const std::exception& error) {
      // Fail closed: no timer republishing of the last velocity measurement.
      RCLCPP_ERROR(get_logger(),"MC publisher worker stopped: %s",error.what());
      navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
      std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;
    }
  }
  void tick(){
    if(shutting_down_.load())return;
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
    std::optional<bool> ack;std::optional<McWrite> mc_write;std::error_code sdk_error,estop_error;bool estop_ack=false;uint64_t dropped,owner_dropped;
    std::deque<OwnerEvent> owner_events;
    {std::lock_guard<std::mutex> lock(inbox_mutex_);ack.swap(mc_ack_);mc_write.swap(mc_write_);sdk_error=sdk_error_;sdk_error_.clear();dropped=mc_in_.dropped;
     estop_ack=estop_ack_;estop_ack_=false;estop_error=estop_write_error_;estop_write_error_.clear();
     OwnerEvent event;while(owner_in_.pop(event))owner_events.push_back(std::move(event));owner_dropped=owner_in_.dropped;}
    if(sdk_error){
      last_sdk_error_=sdk_error.message();last_sdk_error_code_=sdk_error.value();last_sdk_error_at_=wall();
      RCLCPP_WARN(get_logger(),"SDK monitor: %s (%d, %s)",sdk_error.message().c_str(),sdk_error.value(),sdk_error.category().name());
    }
    uint64_t takeover_id=0;
    {std::lock_guard<std::mutex> lock(mutex);
    apply_navigation_events();
    if(estop_ack&&safety.pending)safety.ack=true;
    if(estop_error)safety.failed("急停发送异常："+estop_error.message());
    if(mc_write)mc_.written(mc_write->generation,mc_write->id,mc_write->error?mc_write->error.message():"");
    ownership_.connection(connected_now,replay);
    if(!connected_now)owner_disconnected_at_=mono();
    for(const auto& event:owner_events){
      if(!connected_now||replay||event.arrival<=owner_disconnected_at_)continue;
      switch(event.kind){
        case OwnerKind::Robot:ownership_.robot(event.code,event.arrival);break;
        case OwnerKind::Available:ownership_.available(event.arrival);break;
        case OwnerKind::Lost:ownership_.lost();break;
        case OwnerKind::Ack:ownership_.ack(event.code,event.detail,event.arrival);break;
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
    {std::lock_guard<std::mutex> lock(mutex);safety.connected=connected_now;safety.tick(mono());safe=safety;report=mc_.snapshot(mono());owner_report=ownership_;}
    String connected;connected.data=safe.connected?"connected":"disconnected";
    d1monitor::NavigationMotion nav;std::string nav_reason,sdk_version;d1monitor::execution3::State execution_state;uint64_t v3_moves=0;
    double mc_capture_lower=0,mc_capture_upper=0,mc_capture_bound=0;bool mc_source_fresh=false;
    {std::lock_guard<std::mutex> lock(mutex);nav=navigation_;nav_reason=navigation_.reason(navigation_health(mono()),mono());
      if(execution3_){execution_state=execution3_->state(wall());mc_capture_lower=execution3_->mcCaptureLowerBound();
        mc_capture_upper=execution3_->mcCaptureUpperBound();mc_capture_bound=execution3_->mcCaptureDelayBound();
        mc_source_fresh=execution3_->mcSourceFresh(wall(),mc_clock_epoch());}
      v3_moves=execution_submitted_moves_;sdk_version=sdk_version_;}
    const bool v3=static_cast<bool>(execution3_);
    const Json execution_diag={{"enabled",v3},{"execution_policy_schema",1},{"phase",execution_state.phase},{"reason",execution_state.reason},
      {"grant_ready",execution_state.grant_ready},{"control_owned",execution_state.control_owned},
      {"execution_id",execution_state.execution_id},{"control_epoch",execution_state.control_epoch},
      {"sdk_arm_generation",execution_state.sdk_arm_generation},{"move_submissions",v3_moves},
      {"acceptance_record_valid",execution_acceptance_.valid},{"acceptance_reason",execution_acceptance_.reason},
      {"mc_clock_basis","source_delta_host_anchor_approximate"},{"mc_clock_approximate",true},
      {"mc_capture_lower_bound_unix",mc_capture_lower},{"mc_capture_upper_bound_unix",mc_capture_upper},
      {"mc_capture_delay_bound_s",mc_capture_bound},
      {"mc_accepted_source_fresh",mc_source_fresh},{"mc_arrival_fresh",report.fresh},
      {"writer_period_s",.05},{"writer_last_duration_s",execution_last_duration_s_.load()},
      {"writer_deadlines_skipped",execution_writer_skipped_.load()},{"writer_overruns",execution_writer_overruns_.load()},
      {"scheduling_isolation","dedicated_input_executor_steady_writer_latest_telemetry"},
      {"independent_physical_stop_verified",false},
      {"fault_latched",execution_state.fault_latched}};
    String behavior;behavior.data=Json{{"fsm_state",v3?execution_state.phase:nav.enabled?(nav.armed?"NAVIGATION_ARMED":"NAVIGATION_LOCKED"):"MONITOR_ONLY"},{"telemetry_only",!nav.enabled&&!v3},{"motion_control_enabled",nav.enabled||v3},{"control_adapter",v3?"monitor_execution_v3":nav.enabled?"monitor_navigation_v1":"monitor_estop_v1"},
      {"execution_v3",execution_diag},{"sdk_version",sdk_version},
      {"ready_for_navigation",v3?Json(execution_state.grant_ready):nav.enabled?Json(nav_reason.empty()):Json(nullptr)},{"sdk_has_control",owner_report.owns(mono())},{"ownership_state",owner_report.state(mono())},{"replay_latched",safe.replay},{"sdk_commands_sent",safe.sent+nav.sent+v3_moves},
      {"navigation_armed",nav.armed},{"navigation_arm_generation",nav.generation},{"navigation_block_reason",nav_reason},{"navigation_error",nav.error},{"fault_latched",nav.fault},{"requires_review",nav.fault},
      {"navigation_measured_planar_mps",nav.measured_planar_mps},{"navigation_overspeed_latched",nav.overspeed_latched},
      {"estop_result",safe.result},{"estop_ack",safe.ack},{"estop_error",safe.error},{"estop_pending",safe.pending}}.dump();
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
    String status;status.data=Json{{"session",session_},{"service_prefix","/d1max/monitor/s_"+session_},{"mode",safe.replay?"replay":"monitor"},
      {"motion_control_enabled",nav.enabled||v3},{"navigation_armed",v3?execution_state.grant_ready:nav.armed},{"navigation_arm_generation",v3?execution_state.sdk_arm_generation:nav.generation},
      {"execution_v3",execution_diag},{"sdk_version",sdk_version},
      {"navigation_block_reason",nav_reason},{"navigation_error",nav.error},{"navigation_fault_latched",nav.fault},
      {"navigation_measured_planar_mps",nav.measured_planar_mps},{"navigation_overspeed_latched",nav.overspeed_latched},
      {"navigation_limits",{{"hard_planar_mps",d1monitor::NavigationMotion::hard_planar_mps},{"forward_mps",nav.max_x},{"lateral_mps",nav.max_y},{"yaw_radps",nav.max_yaw},{"required_speed_level",1}}},
      {"joint_state_enabled",joints_.enabled},
      {"ownership",owner},{"safety_available",safe.available()},{"wall_time",wall()}}.dump();
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
  d1monitor::Ownership ownership_;std::string last_owner_state_;
  d1monitor::NavigationMotion navigation_;d1monitor::NavigationHealth navigation_robot_;int navigation_control_source_=0;
  d1monitor::NavigationEvents navigation_events_;
  struct McWrite {uint64_t generation,id;std::error_code error;};
  struct JointAck {bool on;double arrival;};
  d1monitor::JointReport joints_;std::optional<d1monitor::JointPacket> joint_in_;
  std::optional<JointAck> joint_ack_;std::optional<McWrite> joint_write_;
  uint64_t joint_invalid_=0,joint_dropped_=0;
  std::mutex joint_mutex_;std::condition_variable joint_wake_;std::thread joint_worker_;bool joint_worker_failed_=false;
  std::optional<McWrite> mc_write_;bool estop_ack_=false;std::error_code estop_write_error_;
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
  d1monitor::execution3::Acceptance execution_acceptance_;
  uint64_t execution_submitted_moves_=0;
  std::atomic<bool>execution_write_failed_{false};
  std::atomic<uint64_t>execution_stop_written_epoch_{0};std::atomic<double>execution_stop_written_at_{0};
  std::mutex execution_ack_mutex_;uint64_t execution_write_ack_commit_=0,execution_write_ack_epoch_=0;
  uint64_t execution_zero_ack_write_=0,execution_zero_ack_epoch_=0;double execution_zero_ack_at_=0.;
  rclcpp::Publisher<d1monitor::execution3::State>::SharedPtr execution_state_;
  rclcpp::Publisher<d1monitor::execution3::Stop>::SharedPtr execution_stop_;
  rclcpp::Publisher<d1monitor::execution3::CommitAck>::SharedPtr execution_commit_;
  rclcpp::Publisher<d1monitor::execution3::Stationary>::SharedPtr execution_stationary_;
  rclcpp::Service<d1monitor::execution3::Grant>::SharedPtr execution_grant_;
  rclcpp::Subscription<d1monitor::execution3::Permit>::SharedPtr execution_permit_;
  rclcpp::Subscription<d1monitor::execution3::Demand>::SharedPtr execution_demand_;
  rclcpp::Subscription<d1monitor::execution3::MotionProof>::SharedPtr execution_motion_proof_;
  rclcpp::Subscription<d1monitor::execution3::Handoff>::SharedPtr execution_handoff_;
  rclcpp::Subscription<d1monitor::execution3::Prepared>::SharedPtr execution_prepared_;
  rclcpp::Subscription<d1monitor::execution3::CurveProof>::SharedPtr execution_curve_;
  rclcpp::Subscription<d1monitor::execution3::Progress>::SharedPtr execution_progress_;
  rclcpp::Subscription<d1monitor::execution3::Local>::SharedPtr execution_local_;
  rclcpp::CallbackGroup::SharedPtr execution_callbacks_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>execution_executor_;
  std::thread execution_input_worker_,execution_writer_;std::atomic<bool>execution_workers_stopping_{false};
  std::mutex execution_wait_mutex_;std::condition_variable execution_wake_;
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
int main(int argc,char** argv){
  try {
    d1monitor::SessionLease lease;
    rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();node->start_execution_workers();rclcpp::spin(node);node.reset();
    if(rclcpp::ok())rclcpp::shutdown();
  }catch(const std::exception& error){fprintf(stderr,"SDK monitor: %s\n",error.what());return 1;}
}
