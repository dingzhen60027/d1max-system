// Status + handoff + one-way e-stop. Optional, disabled-by-default navigation
// velocity transport reuses this SAME SDK session and requires explicit arming.
#include <chrono>
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
    for(const auto& topic:{"robot_state","velocity","speed_report_status","faults","behavior_state","connection_state_text","transition_event"})
      pubs_[topic]=create_publisher<String>(std::string("/d1max_sdk_bridge/")+topic,10);
    if(joints_.enabled){
      pubs_["joint_states"]=create_publisher<String>("/d1max_sdk_bridge/joint_states",1);
      pubs_["joint_state_status"]=create_publisher<String>("/d1max_sdk_bridge/joint_state_status",1);
    }
    status_=create_publisher<String>("/d1max/monitor/status",10);
    clock_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;});
    graph_=create_wall_timer(std::chrono::seconds(1),[this]{
      for(const auto& name:get_node_names())if(name.find("rosbag2_player")!=std::string::npos){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;}
    });
    stop_=create_service<std_srvs::srv::Trigger>("/d1max/monitor/s_"+session_+"/soft_estop",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      bool send=false;
      {std::lock_guard<std::mutex> lock(mutex);safety.connected=sdk_->IsConnected();
       if(!safety.available()){response->success=false;response->message="SDK 不可用或已检测到回放";return;}
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
    data_=std::make_shared<Data>(this);ack_=std::make_shared<Ack>(this);
    sdk_->SetDataCallback(data_);sdk_->SetControlCallback(ack_);
    // Async even for the first connection: an initially offline robot must not
    // block the ROS executor or require a user to restart this process.
    connect_retry_.due(1,mono());connect_async();
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{tick();});
    worker_=std::thread([this]{work();});
    if(joints_.enabled)joint_worker_=std::thread([this]{work_joint();});
    RCLCPP_INFO(get_logger(),"MONITOR: OnMcData; APP-release handoff %s; optional navigation %s (startup DISARMED). No posture/mode/e-stop-release commands.",ownership_.enabled?"enabled":"disabled",navigation_.enabled?"enabled":"disabled");
  }
  ~Monitor() override {
    timer_->cancel();graph_->cancel();
    if(navigation_timer_)navigation_timer_->cancel();
    d1monitor::dispatch_navigation(mutex,[this]()->std::optional<d1monitor::NavigationVelocity>{
      apply_navigation_events();
      const bool send=navigation_.moving&&navigation_.can_stop(navigation_health(mono()),mono());
      navigation_.disarm("bridge_exit");
      return send?std::optional<d1monitor::NavigationVelocity>{d1monitor::NavigationVelocity{}}:std::nullopt;
    },[this](const auto& command){send_navigation(command);});
    {std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;}
    wake_.notify_all();joint_wake_.notify_all();
    if(worker_.joinable())worker_.join();
    if(joint_worker_.joinable())joint_worker_.join();
    // Only close the telemetry stream this process requested. Never release estop.
    if(mc_.total_attempts&&!safety.replay&&ownership_.allow_mc_config()&&sdk_->IsConnected())sdk_->SetMcConfig(false,250);
    if(joints_.enabled&&joints_.attempts&&!safety.replay&&sdk_->IsConnected())sdk_->SetJointStateConfig(false,250);
    sdk_->Disconnect(true);sdk_.reset();
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
    const auto error=sdk_->Connect(ip_,port_,false,[this](const std::error_code& e){
      if(e){navigation_events_.raise(d1monitor::NavigationEvents::ConnectionError);std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}
    });
    if(error){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=error;}
  }
  struct McPacket {robot_sdk::MotionData data{};double arrival=0,received=0;};
  struct RobotPacket {robot_sdk::RobotState data;double arrival,received;};
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
    publish("robot_state",{{"source",navigation_.enabled?"sdk_monitor_navigation":"sdk_monitor_estop"},{"read_only",!navigation_.enabled},{"motion_control_enabled",navigation_.enabled},{"received_at_unix",packet.received},
      {"motion_status",static_cast<int>(d.motion_status)},{"sport_mode",static_cast<int>(d.sport_mode)},{"control_source",static_cast<int>(d.control_source)},
      {"speed_level",static_cast<int>(d.speed_level)},{"software_emergency_status",static_cast<int>(d.software_emergency_status)},
      {"hardware_emergency_status",static_cast<int>(d.hardware_emergency_status)},
      {"battery_power_1",d.battery.power1},{"battery_power_2",d.battery.power2},
      {"head_angle",d.head_angle},{"head_direction",static_cast<int>(d.head_direction)},{"mileage",d.mile_data},{"joint_temperatures",d.joint_temps}});
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
        auto output=Json::array();for(const auto& f:faults)output.push_back({{"level",static_cast<int>(f.level)},{"code",static_cast<int>(f.code)},{"message",f.message}});
        publish("faults",output);
      }
      if(!have_mc)continue;
      const auto& d=packet.data;uint64_t sequence,generation;double stamp;
      {std::lock_guard<std::mutex> lock(mutex);
       if(safety.replay||!mc_.sample(packet.arrival,packet.received,d.time_stamp,d.v_body,d.omega_body,mono()))continue;
       navigation_.measurement(d.v_body[0],d.v_body[1],packet.arrival);
       sequence=mc_.samples;generation=mc_.generation;stamp=mc_.stamp_unix;}
      publish("velocity",{{"source","sdk_mc"},{"read_only",true},{"frame","sdk_body"},
        {"received_at_unix",packet.received},{"stamp_unix",stamp},{"source_timestamp_ns",std::to_string(d.time_stamp)},
        {"clock_mode","source_delta_host_anchor"},{"clock_approximate",true},{"session",session_},{"generation",generation},
        {"v_body",d.v_body},{"omega_body",d.omega_body},
        {"forward_speed",d.v_body[0]},{"lateral_speed",d.v_body[1]},{"yaw_speed",d.omega_body[2]},{"sequence",sequence}});
    }} catch(const std::exception& error) {
      // Fail closed: no timer republishing of the last velocity measurement.
      RCLCPP_ERROR(get_logger(),"MC publisher worker stopped: %s",error.what());
      navigation_events_.raise(d1monitor::NavigationEvents::WorkerStopped);
      std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;
    }
  }
  void tick(){
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
    request_takeover=ownership_.due(mono());
    takeover_id=ownership_.request_id;
    }
    if(request_takeover){
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
    d1monitor::Safety safe;d1monitor::McReport report;
    {std::lock_guard<std::mutex> lock(mutex);safety.connected=connected_now;safety.tick(mono());safe=safety;report=mc_;}
    String connected;connected.data=safe.connected?"connected":"disconnected";pubs_.at("connection_state_text")->publish(connected);
    d1monitor::NavigationMotion nav;std::string nav_reason;
    {std::lock_guard<std::mutex> lock(mutex);nav=navigation_;nav_reason=navigation_.reason(navigation_health(mono()),mono());}
    publish("behavior_state",{{"fsm_state",nav.enabled?(nav.armed?"NAVIGATION_ARMED":"NAVIGATION_LOCKED"):"MONITOR_ONLY"},{"telemetry_only",!nav.enabled},{"motion_control_enabled",nav.enabled},{"control_adapter",nav.enabled?"monitor_navigation_v1":"monitor_estop_v1"},
      {"ready_for_navigation",nav.enabled?Json(nav_reason.empty()):Json(nullptr)},{"sdk_has_control",ownership_.owns(mono())},{"ownership_state",ownership_.state(mono())},{"replay_latched",safe.replay},{"sdk_commands_sent",safe.sent+nav.sent},
      {"navigation_armed",nav.armed},{"navigation_arm_generation",nav.generation},{"navigation_block_reason",nav_reason},{"navigation_error",nav.error},{"fault_latched",nav.fault},{"requires_review",nav.fault},
      {"navigation_measured_planar_mps",nav.measured_planar_mps},{"navigation_overspeed_latched",nav.overspeed_latched},
      {"estop_result",safe.result},{"estop_ack",safe.ack},{"estop_error",safe.error},{"estop_pending",safe.pending}});
    const auto now=mono();const auto owner_state=ownership_.state(now);
    const auto mc_state=!report.fresh(now)&&connected_now&&!replay&&!ownership_.allow_mc_config()?owner_state:report.state(now);
    const Json owner={{"enabled",ownership_.enabled},{"state",owner_state},{"confirmed",ownership_.owns(now)},
      {"acknowledged",ownership_.acknowledged},{"state_confirmations",ownership_.confirmations},{"control_source",ownership_.control_source},
      {"requests",ownership_.total_requests},{"error",ownership_.error},{"automatic_retry",false}};
    if(owner_state!=last_owner_state_){
      RCLCPP_INFO(get_logger(),"SDK ownership: %s",owner_state.c_str());last_owner_state_=owner_state;
    }
    if(mc_state!=last_mc_state_){
      RCLCPP_INFO(get_logger(),"MC stream: %s, received %.1f Hz, samples %lu",mc_state.c_str(),report.observed_hz(now),report.samples);
      last_mc_state_=mc_state;
    }
    // Stable ROS topic name for existing Web/Foxglove subscriptions; source is explicit.
    publish("speed_report_status",{{"source","sdk_mc"},{"callback","OnMcData"},{"requested",report.total_attempts>0},
      {"expected_hz",report.expected_hz},{"acknowledged",report.acknowledged},{"ack_on",report.ack_on},
      {"samples",report.samples},{"invalid_samples",report.invalid_samples},{"timestamp_rejections",report.timestamp_rejections},
      {"stale_samples",report.stale_samples},{"queue_dropped",dropped},
      {"stream_fresh",report.fresh(now)},{"observed_hz",report.observed_hz(now)},{"source_hz",report.source_hz(now)},{"rate_ok",report.rate_ok(now)},
      {"clock_mode","source_delta_host_anchor"},{"clock_approximate",true},
      {"state",mc_state},{"attempts",report.attempts},{"max_attempts",report.max_attempts},
      {"total_attempts",report.total_attempts},{"retry_cycles",report.retry_cycles},{"next_retry_sec",ownership_.allow_mc_config()?report.next_retry_in(now):-1.},
      {"ownership",owner},
      {"connection_state",static_cast<int>(connection_state)},{"connect_attempts",connect_retry_.attempts},
      {"last_sdk_error",last_sdk_error_},{"last_sdk_error_code",last_sdk_error_code_},{"last_sdk_error_at",last_sdk_error_at_},
      {"write_complete",report.write_complete},{"write_error",report.write_error},{"received_at_unix",wall()}});
    String status;status.data=Json{{"session",session_},{"service_prefix","/d1max/monitor/s_"+session_},{"mode",safe.replay?"replay":"monitor"},
      {"motion_control_enabled",nav.enabled},{"navigation_armed",nav.armed},{"navigation_arm_generation",nav.generation},
      {"navigation_block_reason",nav_reason},{"navigation_error",nav.error},{"navigation_fault_latched",nav.fault},
      {"navigation_measured_planar_mps",nav.measured_planar_mps},{"navigation_overspeed_latched",nav.overspeed_latched},
      {"navigation_limits",{{"hard_planar_mps",d1monitor::NavigationMotion::hard_planar_mps},{"forward_mps",nav.max_x},{"lateral_mps",nav.max_y},{"yaw_radps",nav.max_yaw},{"required_speed_level",1}}},
      {"joint_state_enabled",joints_.enabled},
      {"ownership",owner},{"safety_available",safe.available()},{"wall_time",wall()}}.dump();status_->publish(status);
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
    publish("joint_state_status",{{"source","sdk_joint_state"},{"callback","OnJointStateData"},{"read_only",true},
      {"enabled",true},{"state",failed?"worker_failed":report.state(now)},{"stream_fresh",report.fresh(now)},{"worker_failed",failed},
      {"clock_mode","receipt_only"},{"source_timestamp_available",false},{"units_verified",false},{"urdf_mapping_verified",false},
      {"session",session_},{"generation",report.generation},{"samples",report.samples},
      {"last_sample_received_at_unix",report.last_received>0?Json(report.last_received):Json(nullptr)},
      {"receipt_ttl_sec",d1monitor::JointReport::receipt_ttl},{"invalid_samples",invalid},{"stale_samples",report.stale_samples},{"queue_dropped",dropped},
      {"attempts",report.attempts},{"max_attempts",d1monitor::JointReport::max_attempts},{"total_attempts",report.total_attempts},
      {"acknowledged",report.acknowledged},{"ack_on",report.ack_on},{"write_complete",report.write_complete},{"write_error",report.write_error},
      {"received_at_unix",wall()}});
  }
  std::string session_,last_mc_state_,ip_,port_,last_sdk_error_;d1monitor::McReport mc_;
  d1monitor::ConnectRetry connect_retry_;int last_connection_state_=-1,last_sdk_error_code_=0;double last_sdk_error_at_=0.;
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
  d1monitor::Inbox<McPacket,128> mc_in_;std::optional<RobotPacket> robot_in_;
  robot_sdk::FaultDatas faults_in_;std::optional<bool> mc_ack_;std::error_code sdk_error_;
  std::map<std::string,rclcpp::Publisher<String>::SharedPtr> pubs_;
  rclcpp::Publisher<String>::SharedPtr status_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Subscription<String>::SharedPtr navigation_velocity_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr navigation_arm_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;
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
    rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();rclcpp::spin(node);node.reset();
    if(rclcpp::ok())rclcpp::shutdown();
  }catch(const std::exception& error){fprintf(stderr,"SDK monitor: %s\n",error.what());return 1;}
}
