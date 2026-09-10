// Protected console adapter. Startup only connects; every SDK action is gated.
#include <chrono>
#include <memory>
#include <mutex>
#include <regex>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "robot_sdk/sdk_client.hpp"
#include "console_control_core.hpp"

using Json=nlohmann::json;
using String=std_msgs::msg::String;
using Trigger=std_srvs::srv::Trigger;
using SetBool=std_srvs::srv::SetBool;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double wall(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
static const std::vector<std::string> actions={"take_control","release_control","stand","lie_down","crawl","general_mode","in_place_mode","stair_mode","prepare_navigation","unlock_to_stand","reset_error","halt","soft_estop","recover_estop"};
class Bridge;
class Data final:public robot_sdk::IDataCallback {
 public: explicit Data(Bridge* b):b_(b){}
  void OnRobotStateData(const robot_sdk::RobotState& s) override;
  void OnFaultData(const robot_sdk::FaultDatas& f) override;
  void OnControlLost(const robot_sdk::ControlLostInfo&) override;
 private: Bridge* b_;
};
class Acks final:public robot_sdk::IControlCallback {
 public: explicit Acks(Bridge* b):b_(b){}
  void OnStandUp() override; void OnLieDown() override; void OnCrawl() override;
  void OnMode(int mode) override; void OnSpeed(int speed) override;
  void OnSoftEmergencyStop(bool on) override;
  void OnTakeControlAck(const robot_sdk::TakeControlAck& a) override;
  void OnReleaseControlAck(const robot_sdk::ReleaseControlAck& a) override;
 private: Bridge* b_;
};
class Bridge final:public rclcpp::Node {
 public:
  std::mutex mutex;
  d1console::Core core;
  Bridge():Node("d1max_sdk_bridge",rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false)) {
    auto ip=declare_parameter<std::string>("robot_ip","192.168.168.168");
    auto port=declare_parameter<int>("robot_port",8081);
    core.config.stale=declare_parameter<double>("state_stale_seconds",2.5);
    core.config.lease_timeout=declare_parameter<double>("lease_timeout_seconds",.6);
    core.config.velocity_timeout=declare_parameter<double>("command_timeout_seconds",.25);
    core.config.transition_timeout=declare_parameter<double>("transition_timeout_seconds",15.);
    core.config.max_x=declare_parameter<double>("max_forward",.30);
    core.config.max_y=declare_parameter<double>("max_lateral",.20);
    core.config.max_yaw=declare_parameter<double>("max_yaw",.50);
    if(!(core.config.stale>0&&core.config.stale<=2.5&&core.config.lease_timeout>0&&core.config.lease_timeout<=.6&&core.config.velocity_timeout>0&&core.config.velocity_timeout<=.25&&core.config.transition_timeout>=3&&core.config.transition_timeout<=30&&core.config.max_x>0&&core.config.max_x<=.30&&core.config.max_y>0&&core.config.max_y<=.20&&core.config.max_yaw>0&&core.config.max_yaw<=.50)) throw std::runtime_error("Unsafe console configuration");
    for(const auto& name:{"robot_state","behavior_state","connection_state_text","faults","transition_event"}) pubs_[name]=create_publisher<String>(std::string("~/")+name,10);
    lease_sub_=create_subscription<String>("/d1max/console/control_lease",1,[this](String::ConstSharedPtr m){
      try {const auto j=Json::parse(m->data);const auto session=j.at("session").get<std::string>();const double stamp=j.at("stamp").get<double>();const auto seq=j.at("seq").get<unsigned long>();
        if(!std::regex_match(session,std::regex("[a-f0-9]{16}"))||!std::isfinite(stamp)||wall()-stamp<-.1||wall()-stamp>.6||!j.at("armed").is_boolean()) return;
        std::lock_guard<std::mutex> lock(mutex);
        if(session==lease_session_&&seq<=lease_seq_) return;
        if(session!=lease_session_&&core.lease_valid(mono())) return; // no takeover of an active UI lease
        lease_session_=session;lease_seq_=seq;core.lease(j.at("armed").get<bool>(),session,mono());
      }catch(const Json::exception&){}
    });
    velocity_sub_=create_subscription<String>("/d1max/console/guarded_velocity",1,[this](String::ConstSharedPtr m){
      try {const auto j=Json::parse(m->data);const double stamp=j.at("stamp").get<double>();const auto seq=j.at("seq").get<unsigned long>();
        if(!std::isfinite(stamp)||wall()-stamp<-.1||wall()-stamp>core.config.velocity_timeout) return;
        std::lock_guard<std::mutex> lock(mutex);
        if(j.at("session")!=lease_session_||seq<=velocity_seq_) return;
        if(core.velocity(j.at("x").get<double>(),j.at("y").get<double>(),j.at("yaw").get<double>(),mono())) velocity_seq_=seq;
      }catch(const Json::exception&){}
    });
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr){std::lock_guard<std::mutex> lock(mutex);core.replay_detected();});
    for(const auto& a:actions){
      if(a=="soft_estop"||a=="recover_estop") bool_services_.push_back(create_service<SetBool>("~/"+a,[this,a](std::shared_ptr<SetBool::Request> req,std::shared_ptr<SetBool::Response> res){
        if(req->data!=(a=="soft_estop")){res->success=false;res->message="急停/解除端点与参数不匹配";return;} accept(a,res);
      }));
      else services_.push_back(create_service<Trigger>("~/"+a,[this,a](std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> res){accept(a,res);}));
    }
    robot_sdk::ConnectionConfig c;c.auto_reconnect=true;c.reconnect_interval_ms=2000;
    sdk_=std::make_unique<robot_sdk::SDKClient>([this](const std::error_code& e){if(e){std::lock_guard<std::mutex> lock(mutex);core.dispatch_failed(e.message());}},c);
    data_=std::make_shared<Data>(this);acks_=std::make_shared<Acks>(this);
    sdk_->SetDataCallback(data_);sdk_->SetControlCallback(acks_);
    const auto ec=sdk_->Connect(ip,std::to_string(port),true);
    if(ec) RCLCPP_WARN(get_logger(),"Connect: %s",ec.message().c_str());
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    graph_timer_=create_wall_timer(std::chrono::seconds(1),[this]{for(const auto& n:get_node_names())if(n.find("rosbag2_player")!=std::string::npos){std::lock_guard<std::mutex> lock(mutex);core.replay_detected();}});
    RCLCPP_INFO(get_logger(),"Console controller connected LOCKED; no automatic TakeControl, recovery or movement");
  }
  ~Bridge() override {
    if(timer_)timer_->cancel();
    // Only stop this process's own movement. Never release someone else's control.
    bool stop=false;
    {std::lock_guard<std::mutex> lock(mutex);stop=core.moving&&core.owned&&core.state.control==2&&(core.state.motion==5||core.state.motion==7);core.cancel("控制桥退出",false);}
    if(stop&&sdk_->IsConnected()) sdk_->Move(0,0,0,0);
    sdk_->Disconnect(true);sdk_.reset();
  }
  void publish(const std::string& name,const Json& data){String m;m.data=data.dump();pubs_.at(name)->publish(m);}
  void robot(const robot_sdk::RobotState& d){
    d1console::State s;s.motion=static_cast<int>(d.motion_status);s.mode=static_cast<int>(d.sport_mode);s.control=static_cast<int>(d.control_source);s.speed=static_cast<int>(d.speed_level);s.software=static_cast<int>(d.software_emergency_status);s.hardware=static_cast<int>(d.hardware_emergency_status);s.x=d.speed.line;s.y=d.speed.translation;s.yaw=d.speed.angle;
    std::lock_guard<std::mutex> lock(mutex);core.update(s,mono());
    robot_json_={{"source","sdk_guarded_console"},{"read_only",false},{"received_at_unix",wall()},{"motion_status",s.motion},{"sport_mode",s.mode},{"control_source",s.control},{"speed_level",s.speed},{"software_emergency_status",s.software},{"hardware_emergency_status",s.hardware},{"battery_power_1",d.battery.power1},{"battery_power_2",d.battery.power2},{"forward_speed",s.x},{"lateral_speed",s.y},{"yaw_speed",s.yaw},{"head_angle",d.head_angle},{"head_direction",static_cast<int>(d.head_direction)},{"mileage",d.mile_data},{"joint_temperatures",d.joint_temps}};robot_pending_=true;
  }
  void ack(const std::string& name,bool success=true){std::lock_guard<std::mutex> lock(mutex);core.ack(name,success);}
  void faults(const robot_sdk::FaultDatas& data){std::lock_guard<std::mutex> lock(mutex);for(const auto& f:data){fault_queue_.push_back({{"level",static_cast<int>(f.level)},{"code",static_cast<int>(f.code)},{"message",f.message}});if(static_cast<int>(f.level)==1||static_cast<int>(f.level)==2)core.fault_report();}}
 private:
  template<class T> void accept(const std::string& a,const std::shared_ptr<T>& response){
    std::lock_guard<std::mutex> lock(mutex);const auto why=core.request(a,mono());response->success=why.empty();response->message=why.empty()?Json{{"accepted",true},{"goal_id",core.goal_id},{"message","请求受理；动作完成以状态机反馈为准"}}.dump():why;
  }
  void tick(){
    std::vector<d1console::Command> commands;
    {std::lock_guard<std::mutex> lock(mutex);core.connection(sdk_->IsConnected());commands=core.tick(mono());}
    for(const auto& c:commands){
      {std::lock_guard<std::mutex> lock(mutex);if(!core.valid(c,mono()))continue;++core.sent_count;}
      // All motion APIs are asynchronous, so ACK waits cannot starve the stop timer.
      std::error_code e;
      if(c.name=="take_control")e=sdk_->TakeControl(0);
      else if(c.name=="release_control")e=sdk_->ReleaseControl(0);
      else if(c.name=="stand")e=sdk_->StandUp(0);
      else if(c.name=="lie_down")e=sdk_->LieDown(0);
      else if(c.name=="crawl")e=sdk_->Crawl(0);
      else if(c.name=="general_mode")e=sdk_->SetMode(1,0);
      else if(c.name=="in_place_mode")e=sdk_->SetMode(2,0);
      else if(c.name=="stair_mode")e=sdk_->SetMode(3,0);
      else if(c.name=="set_speed")e=sdk_->SetSpeed(core.config.speed_level,0);
      else if(c.name=="soft_estop"||c.name=="recover_estop")e=sdk_->SoftEmergencyStop(c.name=="soft_estop",0);
      else if(c.name=="stop"||c.name=="velocity")e=sdk_->Move(static_cast<float>(c.y),static_cast<float>(c.x),static_cast<float>(c.yaw),0);
      if(e){std::lock_guard<std::mutex> lock(mutex);core.dispatch_failed(e.message());}
    }
    std::lock_guard<std::mutex> lock(mutex);
    if(robot_pending_){publish("robot_state",robot_json_);robot_pending_=false;}
    if(!fault_queue_.empty()){publish("faults",fault_queue_);fault_queue_.clear();}
    for(const auto& e:core.events){String m;m.data=e;pubs_.at("transition_event")->publish(m);}core.events.clear();
    if(++ticks_%4!=0)return;
    String connection;connection.data=core.connected?"connected":"disconnected";pubs_.at("connection_state_text")->publish(connection);
    Json reasons=Json::object();for(const auto& a:actions)reasons[a]=core.reason(a,mono());
    publish("behavior_state",{{"fsm_state",core.busy()?"TRANSITIONING":core.fault?"FAULT":core.uncertain?"ERROR":core.lease_valid(mono())?"IDLE":"LOCKED"},{"telemetry_only",false},{"control_adapter","guarded_console_v1"},{"goal",core.goal},{"goal_id",core.goal_id},{"goal_status",core.result},{"active_transition",core.active},{"fault_latched",core.fault},{"ready_for_navigation",core.ready(mono())},{"sdk_has_control",core.owned},{"last_error",core.error},{"requires_review",core.uncertain},{"available_actions",reasons},{"lease_valid",core.lease_valid(mono())},{"replay_latched",core.replay},{"sdk_commands_sent",core.sent_count}});
  }
  std::map<std::string,rclcpp::Publisher<String>::SharedPtr> pubs_;
  Json robot_json_,fault_queue_=Json::array();bool robot_pending_=false;
  std::string lease_session_;unsigned long lease_seq_=0,velocity_seq_=0;int ticks_=0;
  std::shared_ptr<Data> data_;std::shared_ptr<Acks> acks_;std::unique_ptr<robot_sdk::SDKClient> sdk_;
  rclcpp::Subscription<String>::SharedPtr lease_sub_,velocity_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  std::vector<rclcpp::Service<SetBool>::SharedPtr> bool_services_;
  rclcpp::TimerBase::SharedPtr timer_,graph_timer_;
};
void Data::OnRobotStateData(const robot_sdk::RobotState& d){b_->robot(d);}
void Data::OnFaultData(const robot_sdk::FaultDatas& d){b_->faults(d);}
void Data::OnControlLost(const robot_sdk::ControlLostInfo&){std::lock_guard<std::mutex> lock(b_->mutex);b_->core.control_lost();}
void Acks::OnStandUp(){b_->ack("stand");} void Acks::OnLieDown(){b_->ack("lie_down");} void Acks::OnCrawl(){b_->ack("crawl");}
void Acks::OnMode(int m){if(m>=1&&m<=3)b_->ack(m==1?"general_mode":m==2?"in_place_mode":"stair_mode");}
void Acks::OnSpeed(int s){if(s==b_->core.config.speed_level)b_->ack("set_speed");}
void Acks::OnSoftEmergencyStop(bool on){b_->ack(on?"soft_estop":"recover_estop");}
void Acks::OnTakeControlAck(const robot_sdk::TakeControlAck& a){b_->ack("take_control",a.error_code==0);}
void Acks::OnReleaseControlAck(const robot_sdk::ReleaseControlAck& a){b_->ack("release_control",a.error_code==0);}
int main(int argc,char** argv){rclcpp::init(argc,argv);auto node=std::make_shared<Bridge>();rclcpp::spin(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();}
