// Status + authorized ownership handoff + one-way e-stop. No motion/recovery API.
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
#include "rosgraph_msgs/msg/clock.hpp"
#include "robot_sdk/sdk_client.hpp"
#include "monitor_estop_core.hpp"
#include "mc_report_core.hpp"
#include "sdk_session_core.hpp"
#include "sdk_ownership_core.hpp"
using Json=nlohmann::json;
using String=std_msgs::msg::String;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double wall(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
class Monitor;
class Data final:public robot_sdk::IDataCallback {
 public:explicit Data(Monitor* node):node_(node){}
  void OnRobotStateData(const robot_sdk::RobotState&) override;
  void OnMcData(const robot_sdk::MotionData&) override;
  void OnFaultData(const robot_sdk::FaultDatas&) override;
  void OnControlLost(const robot_sdk::ControlLostInfo&) override;
  void OnControlAvailable(const robot_sdk::ControlAvailableInfo&) override;
 private:Monitor* node_;
};
class Ack final:public robot_sdk::IControlCallback {
 public:explicit Ack(Monitor* node):node_(node){}
  void OnSoftEmergencyStop(bool on) override;
  void OnMcConfig(bool on) override;
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
    ownership_.enabled=declare_parameter<bool>("auto_take_control_on_available",false);
    ownership_.settle_sec=declare_parameter<double>("takeover_settle_sec",.5);
    ownership_.ack_timeout_sec=declare_parameter<double>("takeover_ack_timeout_sec",3.);
    ownership_.confirmation_timeout_sec=declare_parameter<double>("takeover_confirmation_timeout_sec",4.);
    ownership_.validate();
    std::random_device random;std::ostringstream token;
    token<<std::hex<<std::setfill('0')<<std::setw(8)<<random()<<std::setw(8)<<random();session_=token.str();
    for(const auto& topic:{"robot_state","velocity","speed_report_status","faults","behavior_state","connection_state_text","transition_event"})
      pubs_[topic]=create_publisher<String>(std::string("/d1max_sdk_bridge/")+topic,10);
    status_=create_publisher<String>("/d1max/monitor/status",10);
    clock_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;});
    graph_=create_wall_timer(std::chrono::seconds(1),[this]{
      for(const auto& name:get_node_names())if(name.find("rosbag2_player")!=std::string::npos){std::lock_guard<std::mutex> lock(mutex);safety.replay=true;}
    });
    stop_=create_service<std_srvs::srv::Trigger>("/d1max/monitor/s_"+session_+"/soft_estop",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      bool send=false;
      {std::lock_guard<std::mutex> lock(mutex);safety.connected=sdk_->IsConnected();
       if(!safety.available()){response->success=false;response->message="SDK 不可用或已检测到回放";return;}
       send=safety.request(mono());response->success=true;
       response->message=safety.triggered(mono())?"机器人已回报软件急停触发；未重复发送":send?"急停请求已提交；等待机器人状态，不代表已确认停止":"正在等待急停状态；未重复发送";
      }
      if(send){
        // The literal true is intentional. There is no release/reset endpoint.
        const auto error=sdk_->SoftEmergencyStop(true,0,[this](const std::error_code& e,std::size_t){if(e){std::lock_guard<std::mutex> lock(mutex);safety.failed("急停发送异常："+e.message());}});
        if(error){std::lock_guard<std::mutex> lock(mutex);safety.failed("急停发送异常："+error.message());response->success=false;response->message=safety.error;}
      }
      String event;event.data=response->message;pubs_.at("transition_event")->publish(event);
    });
    robot_sdk::ConnectionConfig config;config.auto_reconnect=true;config.reconnect_interval_ms=2000;
    sdk_=std::make_unique<robot_sdk::SDKClient>([this](const std::error_code& e){if(e){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}},config);
    data_=std::make_shared<Data>(this);ack_=std::make_shared<Ack>(this);
    sdk_->SetDataCallback(data_);sdk_->SetControlCallback(ack_);
    // Async even for the first connection: an initially offline robot must not
    // block the ROS executor or require a user to restart this process.
    connect_retry_.due(1,mono());connect_async();
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{tick();});
    worker_=std::thread([this]{work();});
    RCLCPP_INFO(get_logger(),"MONITOR: OnMcData only; APP-release handoff %s. No motion/posture/mode/e-stop-release commands.",ownership_.enabled?"enabled":"disabled");
  }
  ~Monitor() override {
    timer_->cancel();graph_->cancel();
    {std::lock_guard<std::mutex> lock(inbox_mutex_);stopping_=true;}
    wake_.notify_all();if(worker_.joinable())worker_.join();
    // Only close the telemetry stream this process requested. Never release estop.
    if(mc_.total_attempts&&!safety.replay&&ownership_.allow_mc_config()&&sdk_->IsConnected())sdk_->SetMcConfig(false,250);
    sdk_->Disconnect(true);sdk_.reset();
  }
  void publish(const std::string& name,const Json& json){String msg;msg.data=json.dump();pubs_.at(name)->publish(msg);}
  void robot(const robot_sdk::RobotState& d){
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     const auto now=mono();robot_in_=RobotPacket{d,now,wall()};
     owner_in_.push({OwnerKind::Robot,now,static_cast<uint32_t>(d.control_source),0,{}});}
    wake_.notify_one();
  }
  void faults(const robot_sdk::FaultDatas& d){
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
     for(const auto& f:d){if(faults_in_.size()>=64)break;auto copy=f;copy.message=copy.message.substr(0,1024);faults_in_.push_back(std::move(copy));}}
    wake_.notify_one();
  }
  void mc(const robot_sdk::MotionData& d){
    const McPacket packet{d,mono(),wall()};
    {std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;mc_in_.push(packet);}
    wake_.notify_one();
  }
  void mc_config(bool on){std::lock_guard<std::mutex> lock(inbox_mutex_);mc_ack_=on;}
  // SDK callback thread only copies small, bounded events. Network requests,
  // state transitions, JSON and ROS publication run on the ROS executor.
  void ownership_event(bool available){
    std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
    owner_in_.push({available?OwnerKind::Available:OwnerKind::Lost,mono(),0,0,{}});
  }
  void ownership_ack(const robot_sdk::TakeControlAck& ack){
    std::lock_guard<std::mutex> lock(inbox_mutex_);if(stopping_)return;
    owner_in_.push({OwnerKind::Ack,mono(),ack.error_code,0,ack.reason.substr(0,512)});
  }
 private:
  enum class OwnerKind {Robot,Available,Lost,Ack,Written};
  struct OwnerEvent {OwnerKind kind;double arrival;uint32_t code;uint64_t id;std::string detail;};
  void connect_async(){
    const auto error=sdk_->Connect(ip_,port_,false,[this](const std::error_code& e){
      if(e){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=e;}
    });
    if(error){std::lock_guard<std::mutex> lock(inbox_mutex_);sdk_error_=error;}
  }
  struct McPacket {robot_sdk::MotionData data{};double arrival=0,received=0;};
  struct RobotPacket {robot_sdk::RobotState data;double arrival,received;};
  void process_robot(const RobotPacket& packet){
    const auto& d=packet.data;
    {std::lock_guard<std::mutex> lock(mutex);
     safety.update(static_cast<int>(d.software_emergency_status),static_cast<int>(d.hardware_emergency_status),packet.arrival);
     mc_.robot(packet.arrival);}
    publish("robot_state",{{"source","sdk_monitor_estop"},{"read_only",true},{"motion_control_enabled",false},{"received_at_unix",packet.received},
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
        auto output=Json::array();for(const auto& f:faults)output.push_back({{"level",static_cast<int>(f.level)},{"code",static_cast<int>(f.code)},{"message",f.message}});
        publish("faults",output);
      }
      if(!have_mc)continue;
      const auto& d=packet.data;uint64_t sequence,generation;double stamp;
      {std::lock_guard<std::mutex> lock(mutex);
       if(safety.replay||!mc_.sample(packet.arrival,packet.received,d.time_stamp,d.v_body,d.omega_body,mono()))continue;
       sequence=mc_.samples;generation=mc_.generation;stamp=mc_.stamp_unix;}
      publish("velocity",{{"source","sdk_mc"},{"read_only",true},{"frame","sdk_body"},
        {"received_at_unix",packet.received},{"stamp_unix",stamp},{"source_timestamp_ns",std::to_string(d.time_stamp)},
        {"clock_mode","source_delta_host_anchor"},{"clock_approximate",true},{"session",session_},{"generation",generation},
        {"v_body",d.v_body},{"omega_body",d.omega_body},
        {"forward_speed",d.v_body[0]},{"lateral_speed",d.v_body[1]},{"yaw_speed",d.omega_body[2]},{"sequence",sequence}});
    }} catch(const std::exception& error) {
      // Fail closed: no timer republishing of the last velocity measurement.
      RCLCPP_ERROR(get_logger(),"MC publisher worker stopped: %s",error.what());
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
    std::optional<bool> ack;std::error_code sdk_error;uint64_t dropped,owner_dropped;
    std::deque<OwnerEvent> owner_events;
    {std::lock_guard<std::mutex> lock(inbox_mutex_);ack.swap(mc_ack_);sdk_error=sdk_error_;sdk_error_.clear();dropped=mc_in_.dropped;
     OwnerEvent event;while(owner_in_.pop(event))owner_events.push_back(std::move(event));owner_dropped=owner_in_.dropped;}
    if(sdk_error){
      last_sdk_error_=sdk_error.message();last_sdk_error_code_=sdk_error.value();last_sdk_error_at_=wall();
      RCLCPP_WARN(get_logger(),"SDK monitor: %s (%d, %s)",sdk_error.message().c_str(),sdk_error.value(),sdk_error.category().name());
    }
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
    if(request_takeover){
      const auto id=ownership_.request_id;
      RCLCPP_INFO(get_logger(),"APP released control: requesting TakeControl once, id %lu",id);
      const auto error=sdk_->TakeControl(0,[this,id](const std::error_code& e,std::size_t){
        if(e){std::lock_guard<std::mutex> lock(inbox_mutex_);if(!stopping_)
          owner_in_.push({OwnerKind::Written,mono(),0,id,e.message().substr(0,512)});}
      });
      if(error)ownership_.written(id,error.message());
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
        std::lock_guard<std::mutex> lock(mutex);mc_.written(request_generation,request_id,e?e.message():"");
      });
      if(error){std::lock_guard<std::mutex> lock(mutex);mc_.written(request_generation,request_id,error.message());}
    }
    d1monitor::Safety safe;d1monitor::McReport report;
    {std::lock_guard<std::mutex> lock(mutex);safety.connected=connected_now;safety.tick(mono());safe=safety;report=mc_;}
    String connected;connected.data=safe.connected?"connected":"disconnected";pubs_.at("connection_state_text")->publish(connected);
    publish("behavior_state",{{"fsm_state","MONITOR_ONLY"},{"telemetry_only",true},{"motion_control_enabled",false},{"control_adapter","monitor_estop_v1"},
      {"ready_for_navigation",nullptr},{"sdk_has_control",ownership_.owns(mono())},{"ownership_state",ownership_.state(mono())},{"replay_latched",safe.replay},{"sdk_commands_sent",safe.sent},
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
      {"motion_control_enabled",false},{"ownership",owner},{"safety_available",safe.available()},{"wall_time",wall()}}.dump();status_->publish(status);
  }
  std::string session_,last_mc_state_,ip_,port_,last_sdk_error_;d1monitor::McReport mc_;
  d1monitor::ConnectRetry connect_retry_;int last_connection_state_=-1,last_sdk_error_code_=0;double last_sdk_error_at_=0.;
  d1monitor::Ownership ownership_;std::string last_owner_state_;
  d1monitor::Inbox<OwnerEvent,32> owner_in_;uint64_t last_owner_dropped_=0;double owner_disconnected_at_=-1.;
  std::mutex inbox_mutex_;std::condition_variable wake_;std::thread worker_;bool stopping_=false;
  d1monitor::Inbox<McPacket,128> mc_in_;std::optional<RobotPacket> robot_in_;
  robot_sdk::FaultDatas faults_in_;std::optional<bool> mc_ack_;std::error_code sdk_error_;
  std::map<std::string,rclcpp::Publisher<String>::SharedPtr> pubs_;
  rclcpp::Publisher<String>::SharedPtr status_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::TimerBase::SharedPtr timer_,graph_;
  std::shared_ptr<Data> data_;std::shared_ptr<Ack> ack_;std::unique_ptr<robot_sdk::SDKClient> sdk_;
};
void Data::OnRobotStateData(const robot_sdk::RobotState& d){node_->robot(d);}
void Data::OnMcData(const robot_sdk::MotionData& d){node_->mc(d);}
void Data::OnFaultData(const robot_sdk::FaultDatas& d){node_->faults(d);}
void Data::OnControlLost(const robot_sdk::ControlLostInfo&){node_->ownership_event(false);}
void Data::OnControlAvailable(const robot_sdk::ControlAvailableInfo&){node_->ownership_event(true);}
void Ack::OnSoftEmergencyStop(bool on){if(on){std::lock_guard<std::mutex> lock(node_->mutex);if(node_->safety.pending)node_->safety.ack=true;}}
void Ack::OnMcConfig(bool on){node_->mc_config(on);}
void Ack::OnTakeControlAck(const robot_sdk::TakeControlAck& ack){node_->ownership_ack(ack);}
int main(int argc,char** argv){
  try {
    d1monitor::SessionLease lease("/run/user/"+std::to_string(getuid())+"/d1max-sdk-monitor.lock");
    rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();rclcpp::spin(node);node.reset();
    if(rclcpp::ok())rclcpp::shutdown();
  }catch(const std::exception& error){fprintf(stderr,"SDK monitor: %s\n",error.what());return 1;}
}
