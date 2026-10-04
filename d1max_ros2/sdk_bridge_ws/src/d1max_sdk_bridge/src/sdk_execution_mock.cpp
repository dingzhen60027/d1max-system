// Deliberately SDK-free transport. Same admission core as the live writer.
#include "execution_transport_core.hpp"
#include "execution_acceptance.hpp"
#include "execution_mock_telemetry.hpp"
#include <d1max_planning_interfaces/msg/navigation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <stdexcept>
namespace ex=d1monitor::execution3;
class Mock final:public rclcpp::Node {
public:
  Mock():Node("sdk_execution_mock"),core_(declare_parameter<std::string>("sdk_session",""),"isolated_mock") {
    auto env=[](const char*n){const auto*v=std::getenv(n);return v?std::string(v):std::string();};
    if(env("D1MAX_OFFLINE_ZENOH_TEST")!="1"||env("D1MAX_NAV_ISOLATED")!="1"||
       env("D1MAX_NAV_TRANSPORT")!="isolated_mock"||env("D1MAX_NAV_ISOLATION_TOKEN").size()!=32)
      throw std::invalid_argument("private_isolated_mock_environment_required");
    session_=declare_parameter<std::string>("session_id","");map_=declare_parameter<std::string>("map_version_id","");
    if(session_.empty()||map_.empty()||get_parameter("sdk_session").as_string().empty())throw std::invalid_argument("explicit_mock_context_required");
    const bool local_enabled=declare_parameter<bool>("local_state_enabled",false);
    telemetry_=std::make_unique<ex::MockTelemetry>(session_,map_,local_enabled);
    const auto local_topic=declare_parameter<std::string>("local_navigation_state_topic","/d1max/localization/navigation/local_state");
    core_.configureMcCaptureBound(declare_parameter<double>("execution_mc_delay_bound_s",-1.),
      "isolated_simulated_source_clock");
    const auto model=ex::loadBrakingModel(declare_parameter<std::string>("execution_braking_model_record",""),
      declare_parameter<std::string>("execution_braking_model_sha256",""),"isolated_mock");
    if(!model.valid)throw std::invalid_argument(model.reason);
    core_.configureBrakingModel(model.sha256,model.max_speed,model.max_yaw);
    core_.configureExecutionPolicy(model.policy,model.reaction_bound);
    const std::string p="/d1max/live_planning/execution/";
    state_=create_publisher<ex::State>(p+"sdk_state",1);stop_=create_publisher<ex::Stop>(p+"stop_report",1);
    applied_=create_publisher<ex::Demand>(p+"applied_motion",1);
    commit_=create_publisher<ex::CommitAck>(p+"commit_ack",5);
    stationary_=create_publisher<ex::Stationary>(p+"stationary_evidence",8);
    grant_=create_service<ex::Grant>(p+"grant",[this](const std::shared_ptr<ex::Grant::Request>req,std::shared_ptr<ex::Grant::Response>res){
      if(req->version.session_id!=session_||req->version.map_version_id!=map_||
         req->version.localization_epoch!=body_epoch_||req->version.localization_seed_id!=body_seed_){res->reason="mock_context_mismatch";return;}
      *res=core_.grant(*req,now().seconds(),health());
    });
    permit_=create_subscription<ex::Permit>(p+"permit",10,[this](ex::Permit::SharedPtr m){core_.permit(*m,now().seconds());});
    demand_=create_subscription<ex::Demand>(p+"safe_demand",1,[this](ex::Demand::SharedPtr m){core_.demand(*m,now().seconds());});
    motion_proof_=create_subscription<ex::MotionProof>(p+"motion_validation",20,[this](ex::MotionProof::SharedPtr m){core_.motionValidation(*m,now().seconds());});
    handoff_=create_subscription<ex::Handoff>(p+"handoff_grant",5,[this](ex::Handoff::SharedPtr m){core_.handoff(*m,now().seconds());});
    prepared_=create_subscription<ex::Prepared>(p+"safe_prepared_demand",1,[this](ex::Prepared::SharedPtr m){core_.preparedDemand(*m,now().seconds());});
    curve_=create_subscription<ex::CurveProof>(p+"validation",20,[this](ex::CurveProof::SharedPtr m){core_.trajectoryValidation(*m,now().seconds());});
    progress_=create_subscription<ex::Progress>("/d1max/live_planning/tracking_progress",2,[this](ex::Progress::SharedPtr m){core_.trackingProgress(*m,now().seconds());});
    if(local_enabled) local_body_=create_subscription<d1max_planning_interfaces::msg::LocalNavigationState>(local_topic,1,
      [this](d1max_planning_interfaces::msg::LocalNavigationState::SharedPtr m){
        const double received=now().seconds();const auto sample=telemetry_->local(*m,received);
        if(sample)core_.localState(*m,received);
        acceptTelemetry(sample,received);});
    else body_=create_subscription<d1max_planning_interfaces::msg::NavigationState>("/d1max/localization/navigation/state",1,
      [this](d1max_planning_interfaces::msg::NavigationState::SharedPtr m){
        const double received=now().seconds();acceptTelemetry(telemetry_->global(*m,received),received);});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const double t=now().seconds();if(core_.takeControlOnce())owned_=true;
      const auto command=core_.tick(t,health());
      if(command){const auto commit=core_.selectedCommitSequence();const auto write=core_.nextWriteSequence();
        core_.writeCalled(t,true);core_.writeAcknowledged(commit,t);
        if(command->linear.x==0.&&command->angular.z==0.)core_.zeroAcknowledged(write,t,t);
        auto out=core_.lastDemand();
        if(!ex::task(out.version,core_.binding())||out.execution_id!=core_.executionId()) {
          out=ex::Demand{};out.version=core_.binding();out.execution_id=core_.executionId();out.control_epoch=core_.epoch();
          out.sdk_session=get_parameter("sdk_session").as_string();out.sdk_arm_generation=core_.armGeneration();
          // Actual mock zero-write event, not a new motion/safety authorization.
          out.source_stamp=ex::stamp(t);out.body_source_stamp=ex::stamp(source_);out.valid_until=ex::stamp(t+.05);
        }
        out.velocity=*command;out.hold=command->linear.x==0&&command->angular.z==0;
        out.transport_mode="isolated_mock";applied_->publish(out);if(out.hold)core_.stopWritten(t);}
      state_->publish(core_.state(t));stop_->publish(core_.report(t));
      stationary_->publish(core_.stationaryEvidence(t));
      if(const auto ack=core_.takeCommitAck())commit_->publish(*ack);
    });
  }
private:
  void acceptTelemetry(const std::optional<ex::MockMcSample>&sample,double received) {
    if(!sample)return;
    if(!core_.executionId().empty()&&(sample->epoch!=core_.binding().localization_epoch||
       sample->seed!=core_.binding().localization_seed_id)){core_.fault("mock_body_context_changed",received);return;}
    if(sample->source<source_){core_.mcSourceRegressed(received,"isolated-simulated-clock");return;}
    if(sample->source==source_)return;
    source_=sample->source;body_epoch_=sample->epoch;body_seed_=sample->seed;
    core_.mc(sample->raw_ns,sample->source,received,sample->linear,sample->angular,received,"isolated-simulated-clock");
  }
  ex::Health health()const{return {true,owned_,true,core_.mcSourceFresh(now().seconds(),"isolated-simulated-clock"),false};}
  ex::Transport core_;std::string session_,map_,body_seed_;uint64_t body_epoch_=0;double source_=0;bool owned_=false;
  rclcpp::Publisher<ex::State>::SharedPtr state_;rclcpp::Publisher<ex::Stop>::SharedPtr stop_;
  rclcpp::Publisher<ex::Demand>::SharedPtr applied_;rclcpp::Service<ex::Grant>::SharedPtr grant_;
  rclcpp::Publisher<ex::CommitAck>::SharedPtr commit_;
  rclcpp::Publisher<ex::Stationary>::SharedPtr stationary_;
  rclcpp::Subscription<ex::Permit>::SharedPtr permit_;rclcpp::Subscription<ex::Demand>::SharedPtr demand_;
  rclcpp::Subscription<ex::MotionProof>::SharedPtr motion_proof_;
  rclcpp::Subscription<ex::Handoff>::SharedPtr handoff_;rclcpp::Subscription<ex::Prepared>::SharedPtr prepared_;
  rclcpp::Subscription<ex::CurveProof>::SharedPtr curve_;rclcpp::Subscription<ex::Progress>::SharedPtr progress_;
  std::unique_ptr<ex::MockTelemetry> telemetry_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::LocalNavigationState>::SharedPtr local_body_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::NavigationState>::SharedPtr body_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Mock>());}catch(const std::exception&e){
  std::fprintf(stderr,"sdk_execution_mock refused: %s\n",e.what());rclcpp::shutdown();return 2;}rclcpp::shutdown();return 0;}
