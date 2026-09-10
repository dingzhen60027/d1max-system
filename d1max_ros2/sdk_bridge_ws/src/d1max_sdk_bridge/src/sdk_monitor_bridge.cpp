// Status observer + one-way soft emergency stop. No motion/control/recovery API.
#include <chrono>
#include <memory>
#include <mutex>
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
using Json=nlohmann::json;
using String=std_msgs::msg::String;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double wall(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
class Monitor;
class Data final:public robot_sdk::IDataCallback {
 public:explicit Data(Monitor* node):node_(node){}
  void OnRobotStateData(const robot_sdk::RobotState&) override;
  void OnFaultData(const robot_sdk::FaultDatas&) override;
 private:Monitor* node_;
};
class Ack final:public robot_sdk::IControlCallback {
 public:explicit Ack(Monitor* node):node_(node){}
  void OnSoftEmergencyStop(bool on) override;
 private:Monitor* node_;
};
class Monitor final:public rclcpp::Node {
 public:
  std::mutex mutex;
  d1monitor::Safety safety;
  Monitor():Node("d1max_sdk_monitor",rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false)) {
    const auto ip=declare_parameter<std::string>("robot_ip","192.168.168.168");
    const auto port=declare_parameter<int>("robot_port",8081);
    std::random_device random;std::ostringstream token;
    token<<std::hex<<std::setfill('0')<<std::setw(8)<<random()<<std::setw(8)<<random();session_=token.str();
    for(const auto& topic:{"robot_state","faults","behavior_state","connection_state_text","transition_event"})
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
    sdk_=std::make_unique<robot_sdk::SDKClient>([logger=get_logger()](const std::error_code& e){if(e)RCLCPP_WARN(logger,"SDK monitor: %s",e.message().c_str());},config);
    data_=std::make_shared<Data>(this);ack_=std::make_shared<Ack>(this);
    sdk_->SetDataCallback(data_);sdk_->SetControlCallback(ack_);
    const auto error=sdk_->Connect(ip,std::to_string(port),true);
    if(error)RCLCPP_WARN(get_logger(),"SDK connect: %s",error.message().c_str());
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{tick();});
    RCLCPP_INFO(get_logger(),"MONITOR + one-way soft emergency stop ONLY. No ownership, motion or recovery; startup sends no commands.");
  }
  ~Monitor() override {timer_->cancel();sdk_->Disconnect(true);sdk_.reset();}
  void publish(const std::string& name,const Json& json){String msg;msg.data=json.dump();pubs_.at(name)->publish(msg);}
  void robot(const robot_sdk::RobotState& d){
    std::lock_guard<std::mutex> lock(mutex);
    safety.update(static_cast<int>(d.software_emergency_status),static_cast<int>(d.hardware_emergency_status),mono());
    robot_json_={{"source","sdk_monitor_estop"},{"read_only",true},{"motion_control_enabled",false},{"received_at_unix",wall()},
      {"motion_status",static_cast<int>(d.motion_status)},{"sport_mode",static_cast<int>(d.sport_mode)},{"control_source",static_cast<int>(d.control_source)},
      {"speed_level",static_cast<int>(d.speed_level)},{"software_emergency_status",safety.software},{"hardware_emergency_status",safety.hardware},
      {"battery_power_1",d.battery.power1},{"battery_power_2",d.battery.power2},{"forward_speed",d.speed.line},{"lateral_speed",d.speed.translation},
      {"yaw_speed",d.speed.angle},{"head_angle",d.head_angle},{"head_direction",static_cast<int>(d.head_direction)},{"mileage",d.mile_data},{"joint_temperatures",d.joint_temps}};
    robot_pending_=true;
  }
  void faults(const robot_sdk::FaultDatas& faults){std::lock_guard<std::mutex> lock(mutex);for(const auto& f:faults)faults_.push_back({{"level",static_cast<int>(f.level)},{"code",static_cast<int>(f.code)},{"message",f.message}});}
 private:
  void tick(){
    std::lock_guard<std::mutex> lock(mutex);safety.connected=sdk_->IsConnected();safety.tick(mono());
    if(robot_pending_){publish("robot_state",robot_json_);robot_pending_=false;}
    if(!faults_.empty()){publish("faults",faults_);faults_.clear();}
    String connected;connected.data=safety.connected?"connected":"disconnected";pubs_.at("connection_state_text")->publish(connected);
    publish("behavior_state",{{"fsm_state","MONITOR_ONLY"},{"telemetry_only",true},{"motion_control_enabled",false},{"control_adapter","monitor_estop_v1"},
      {"ready_for_navigation",nullptr},{"sdk_has_control",false},{"replay_latched",safety.replay},{"sdk_commands_sent",safety.sent},
      {"estop_result",safety.result},{"estop_ack",safety.ack},{"estop_error",safety.error},{"estop_pending",safety.pending}});
    String status;status.data=Json{{"session",session_},{"service_prefix","/d1max/monitor/s_"+session_},{"mode",safety.replay?"replay":"monitor"},
      {"motion_control_enabled",false},{"safety_available",safety.available()},{"wall_time",wall()}}.dump();status_->publish(status);
  }
  std::string session_;
  Json robot_json_,faults_=Json::array();bool robot_pending_=false;
  std::map<std::string,rclcpp::Publisher<String>::SharedPtr> pubs_;
  rclcpp::Publisher<String>::SharedPtr status_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::TimerBase::SharedPtr timer_,graph_;
  std::shared_ptr<Data> data_;std::shared_ptr<Ack> ack_;std::unique_ptr<robot_sdk::SDKClient> sdk_;
};
void Data::OnRobotStateData(const robot_sdk::RobotState& d){node_->robot(d);}
void Data::OnFaultData(const robot_sdk::FaultDatas& d){node_->faults(d);}
void Ack::OnSoftEmergencyStop(bool on){if(on){std::lock_guard<std::mutex> lock(node_->mutex);if(node_->safety.pending)node_->safety.ack=true;}}
int main(int argc,char** argv){rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();rclcpp::spin(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();}
