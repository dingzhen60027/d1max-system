#pragma once
#include "execution_transport_core.hpp"
#include "execution_acceptance.hpp"
#include <d1max_planning_interfaces/msg/mode_transition_request.hpp>
#include <d1max_planning_interfaces/msg/mode_transition_state.hpp>
#include <array>

namespace d1monitor::execution3 {
using ModeRequest=d1max_planning_interfaces::msg::ModeTransitionRequest;
using ModeState=d1max_planning_interfaces::msg::ModeTransitionState;
enum class ExecutionModePolicy { SegmentModes, FixedStair };
inline ExecutionModePolicy executionModePolicy(const std::string&name){
  if(name=="segment_modes")return ExecutionModePolicy::SegmentModes;
  if(name=="fixed_stair")return ExecutionModePolicy::FixedStair;
  throw std::invalid_argument("unknown_execution_mode_policy");
}
enum class StairRuntimeParameterSource { EvidenceRecord, GeneralRuntimeReuse };
inline StairRuntimeParameterSource stairRuntimeParameterSource(const std::string&name){
  if(name=="evidence_record")return StairRuntimeParameterSource::EvidenceRecord;
  if(name=="general_runtime_reuse")return StairRuntimeParameterSource::GeneralRuntimeReuse;
  throw std::invalid_argument("unknown_stair_runtime_parameter_source");
}
struct ModeCapability {
  std::string name,record_sha256;
  int sdk_mode=0,motion_status=0;
  bool verified=false;
  double forward_scale=0.,yaw_scale=0.,max_speed=0.,max_yaw=0.;
};
struct ModeCapabilities {
  std::array<ModeCapability,2> modes{{{"general","",1,5},{"stair","",3,7}}};
  std::string reason="stair_move_speed_and_hold_unverified";
  const ModeCapability*find(const std::string&name)const {
    for(const auto& m:modes)if(m.name==name)return &m;
    return nullptr;
  }
};
// This is a different, evidence-backed locomotion record, NOT the general
// runtime tuning profile. Missing records never manufacture measured scales or
// zero-hold evidence. User-authorized runtime reuse is separate below. Hash binding preserves the supplied evidence;
// software validation itself does not perform or certify a physical test.
inline ModeCapabilities loadModeCapabilities(const std::string&path,const std::string&expected,
    const std::string&robot,const std::string&sdk,const std::string&profile,
    ExecutionModePolicy policy=ExecutionModePolicy::SegmentModes) {
  ModeCapabilities out;if(path.empty()&&expected.empty())return out;
  try {
    if(path.empty()||expected.size()!=64||fileSha256(path)!=expected)
      throw std::runtime_error("mode_capability_record_hash_mismatch");
    std::ifstream in(path);nlohmann::json j;in>>j;
    if(j.at("schema_version")!=1||j.at("record_kind")!="locomotion_mode_measurements"||
       j.at("transport_mode")!="live"||j.at("fixture_only")!=false||
       j.at("robot_id")!=robot||j.at("sdk_version")!=sdk||j.at("robot_profile_sha256")!=profile||
       j.at("firmware_version").get<std::string>().empty()||!j.at("modes").is_object()||
       (policy==ExecutionModePolicy::SegmentModes&&j.at("modes").size()!=2)||
       (policy==ExecutionModePolicy::FixedStair&&(!j.at("modes").contains("stair")||j.at("modes").size()>2)))
      throw std::runtime_error("mode_capability_record_identity_invalid");
    for(auto&m:out.modes){
      // Fixed STAIR never switches modes and never sends GENERAL commands.
      // Do not demand unrelated GENERAL or mode-switch measurements.
      if(policy==ExecutionModePolicy::FixedStair&&m.name!="stair")continue;
      const auto&p=j.at("modes").at(m.name);
      if(p.at("sdk_mode")!=m.sdk_mode||p.at("motion_status")!=m.motion_status)
        throw std::runtime_error("mode_capability_vendor_enum_mismatch");
      for(const auto*kind:{"mode_switch","move","zero_hold","speed_mapping"}) {
        if(policy==ExecutionModePolicy::FixedStair&&std::string_view(kind)=="mode_switch")continue;
        const auto&e=p.at("measurements").at(kind);
        auto evidence=std::filesystem::path(e.at("path").get<std::string>());
        if(evidence.is_relative())evidence=std::filesystem::path(path).parent_path()/evidence;
        if(e.at("passed")!=true||e.at("sha256").get<std::string>().size()!=64||
           fileSha256(evidence.string())!=e.at("sha256").get<std::string>())
          throw std::runtime_error("mode_capability_measurement_missing");
      }
      m.forward_scale=p.at("forward_scale_mps");m.yaw_scale=p.at("yaw_scale_radps");
      m.max_speed=p.at("maximum_speed_mps");m.max_yaw=p.at("maximum_yaw_radps");
      if(!std::isfinite(m.forward_scale)||m.forward_scale<=0||!std::isfinite(m.yaw_scale)||m.yaw_scale<=0||
         !std::isfinite(m.max_speed)||m.max_speed<=0||m.max_speed>.3||
         !std::isfinite(m.max_yaw)||m.max_yaw<=0||m.max_yaw>.5||
         m.max_speed>m.forward_scale||m.max_yaw>m.yaw_scale)
        throw std::runtime_error("mode_capability_motion_scale_invalid");
      m.verified=true;m.record_sha256=expected;
    }
    out.reason.clear();return out;
  }catch(const std::exception&e){out=ModeCapabilities{};out.reason=e.what();return out;}
}

// One bounded transaction in the existing writer. This core emits one SetMode
// intent; it never calls SDK, acquires control, unsets an estop or grants motion.
class ModeTransition {
 public:
  ModeTransition(std::string sdk,std::string transport,ModeCapabilities capabilities={},
      ExecutionModePolicy policy=ExecutionModePolicy::SegmentModes,
      StairRuntimeParameterSource runtime_source=StairRuntimeParameterSource::EvidenceRecord,
      bool general_runtime_valid=false)
    :sdk_(std::move(sdk)),transport_(std::move(transport)),capabilities_(std::move(capabilities)),policy_(policy),
     runtime_source_(runtime_source),general_runtime_valid_(general_runtime_valid){
      if(runtime_source_==StairRuntimeParameterSource::GeneralRuntimeReuse&&!fixedStair())
        throw std::invalid_argument("general_runtime_reuse_requires_fixed_stair");
      if(fixedStair())active_mode_="stair";
    }
  bool fixedStair()const{return policy_==ExecutionModePolicy::FixedStair;}
  bool reusesGeneralRuntime()const{return fixedStair()&&runtime_source_==StairRuntimeParameterSource::GeneralRuntimeReuse;}
  bool runtimeParametersReady()const{return reusesGeneralRuntime()||(!fixedStair()&&active_mode_=="general")?
    general_runtime_valid_:activeCapability()!=nullptr;}
  bool measuredEvidenceVerified()const{return activeCapability()!=nullptr;}
  const char*runtimeParameterBasis()const{return reusesGeneralRuntime()?
    "user_authorized_general_runtime_reuse_not_stair_calibration":
    (!fixedStair()&&active_mode_=="general"?"general_runtime_profile":"mode_measurement_record");}
  const ModeCapability*activeCapability()const {
    const auto*m=capabilities_.find(active_mode_);return m&&m->verified?m:nullptr;
  }
  bool begin(const ModeRequest&r,const Transport&writer,const TaskLease&lease,double now) {
    if(fixedStair())return false; // No request, stop transaction or SetMode in this policy.
    if(r.schema_version!=1||!complete(r.version)||r.request_id.empty()||r.request_id.size()>200||!r.sequence||
       r.target_segment_id.empty()||r.target_segment_id==r.version.segment_id||r.preview_only||
       r.sdk_session!=sdk_||r.transport_mode!=transport_||
       !task(r.version,writer.binding())||r.execution_id!=writer.executionId()||
       r.control_epoch!=writer.epoch()||r.sdk_arm_generation!=writer.armGeneration()||writer.stopping()||
       !leaseMatches(r,lease,now)||!canonicalTime(r.source_stamp)||!canonicalTime(r.valid_until)||
       !fresh(secs(r.source_stamp),now,10.)||secs(r.valid_until)<=now||
       secs(r.valid_until)-secs(r.source_stamp)>15.||!canonicalTime(r.body_source_stamp)||
       sourceNs(r.body_source_stamp)<=0||sourceNs(r.body_source_stamp)>sourceNs(r.source_stamp)||
       !capabilities_.find(r.current_mode)||!capabilities_.find(r.target_mode))return false;
    if(request_&&r.request_id==request_->request_id)return r==*request_;
    if(request_&&r.sequence<=request_->sequence&&task(r.version,request_->version))return false;
    if(request_&&phase_!="ready"&&phase_!="cancelled")return false;
    // SDK OnMode has no request id. After an uncertain dispatched attempt,
    // a late same-mode ACK cannot be distinguished from a new transaction.
    // Require a new SDK session rather than recycling that ambiguity.
    if(uncertain_dispatch_)return false;
    request_=r;sent_=ack_=0.;confirm_count_=0;confirm_first_=0.;phase_="waiting_stationary";
    reason_="waiting_zero_and_measured_stationary";
    const auto*from=capabilities_.find(r.current_mode);const auto*to=capabilities_.find(r.target_mode);
    if(!from->verified||!to->verified){phase_="capability_unverified";reason_=capabilities_.reason.empty()?
      "stair_move_speed_and_hold_unverified":capabilities_.reason;return true;}
    if(!robotFresh(now)||!robot_.owned||!robot_.clear||!robotMatches(*from)) {
      fail("current_robot_mode_not_confirmed");return true;}
    if(r.current_mode==r.target_mode){active_mode_=r.target_mode;phase_="ready";reason_="existing_mode_observed";}
    return true;
  }
  void observeRobot(int mode,int motion,int speed,int head,bool owned,bool clear,uint64_t sequence,double receipt,double now) {
    if(!sequence||sequence<=robot_.sequence||!fresh(receipt,now,2.5)||receipt<robot_.receipt)return;
    robot_={mode,motion,speed,head,owned,clear,sequence,receipt};
    if(!request_||phase_=="capability_unverified"||phase_=="cancelled"||phase_=="failed")return;
    if(!owned||!clear){fail("mode_transition_control_or_estop_changed");return;}
    const auto*target=capabilities_.find(request_->target_mode);
    if(phase_=="ready") {if(!target||!robotMatches(*target))fail("confirmed_robot_mode_changed");return;}
    if(phase_=="confirming_state"&&receipt>ack_&&sequence>ack_robot_sequence_) {
      if(!target||!robotMatches(*target)){confirm_count_=0;confirm_first_=0.;return;}
      if(!confirm_count_)confirm_first_=receipt;
      if(++confirm_count_>=2&&receipt-confirm_first_>=.1){active_mode_=request_->target_mode;
        phase_="ready";reason_="mode_ack_and_new_robot_states_confirmed";}
    }
  }
  void tick(const Transport&writer,const TaskLease&lease,const Stationary&stationary,double now) {
    if(fixedStair())return;
    if(!request_)return;
    if(writer.stopping()||writer.executionId()!=request_->execution_id||writer.epoch()!=request_->control_epoch||
       writer.armGeneration()!=request_->sdk_arm_generation||!leaseMatches(*request_,lease,now)){
      if(awaitingModeChange())uncertain_dispatch_=true;
      phase_="cancelled";reason_="mode_request_task_or_lease_retired";return;}
    if(phase_=="ready")return; // callback evidence independently expires; never restamped as source
    if(phase_=="capability_unverified"||phase_=="failed"||phase_=="cancelled")return;
    if(now>=secs(request_->valid_until)){fail("mode_transition_deadline_expired");return;}
    if(!robotFresh(now)){fail("mode_robot_feedback_stale");return;}
    if(phase_=="waiting_ack"&&now-sent_>2.){fail("mode_ack_timeout");return;}
    if(phase_=="confirming_state"&&now-ack_>4.){fail("mode_feedback_timeout");return;}
    if(phase_!="waiting_stationary")return;
    const auto&s=stationary;
    if(s.schema_version!=1||!s.usable||!s.nonzero_blocked||!task(s.version,request_->version)||
       s.execution_id!=request_->execution_id||s.control_epoch!=request_->control_epoch||
       s.sdk_session!=sdk_||s.sdk_arm_generation!=request_->sdk_arm_generation||s.transport_mode!=transport_||
       !s.zero_write_sequence||!s.mc_raw_stamp_ns||s.mc_clock_epoch.empty()||
       (transport_=="live"&&!s.physical_acceptance_verified)||
       secs(s.zero_ack_at)<secs(request_->source_stamp)||secs(s.capture_lower_bound)<=secs(s.zero_ack_at)||
       secs(s.valid_until)<now||s.stationary_samples<3||s.stationary_duration_sec<.2||
       !std::isfinite(s.measured_linear_mps)||s.measured_linear_mps>.04||
       !std::isfinite(s.measured_angular_radps)||s.measured_angular_radps>.06)return;
    phase_="dispatch_ready";reason_="stationary_set_mode_once";
  }
  std::optional<int> takeDispatch(double now) {
    if(fixedStair())return {};
    if(!request_||phase_!="dispatch_ready"||now>=secs(request_->valid_until))return {};
    phase_="waiting_ack";reason_="waiting_mode_ack";sent_=now;return capabilities_.find(request_->target_mode)->sdk_mode;
  }
  void submitted(bool success){if(!success&&sent_>0&&phase_!="cancelled"){
    uncertain_dispatch_=true;fail("set_mode_write_outcome_unknown");}}
  void acknowledge(int mode,double receipt,double now) {
    if(!request_||phase_!="waiting_ack"||mode!=capabilities_.find(request_->target_mode)->sdk_mode||
       !fresh(receipt,now,2.)||receipt<sent_)return;
    ack_=receipt;ack_robot_sequence_=robot_.sequence;phase_="confirming_state";reason_="waiting_new_mode_states";
  }
  bool holding()const{return request_&&phase_!="ready"&&phase_!="cancelled";}
  bool awaitingModeChange()const{return phase_=="waiting_ack"||phase_=="confirming_state";}
  int expectedSdkMode()const{return request_&&awaitingModeChange()?capabilities_.find(request_->target_mode)->sdk_mode:0;}
  // Mode identity is a last observed fact; its freshness is a separate HOLD
  // input. A callback gap must not invent a different mode and cancel task.
  bool activeModeObserved()const {const auto*m=activeCapability();return m&&robot_.sequence&&robotMatches(*m);}
  // Record a real selected mode even when its Move scale/hold capability has
  // not been measured. Observed != permitted; neither fact fabricates the other.
  bool selectedModeObserved()const {const auto*m=capabilities_.find(active_mode_);
    return m&&robot_.sequence&&robotMatches(*m);}
  void applyHealth(Health&h,double now)const {
    h.motion_mode=robot_.mode==1?"general":robot_.mode==3?"stair":"unknown";
    h.motion_mode_confirmed=fixedStair()?selectedModeObserved():activeModeObserved();
    h.mode_transition_active=holding();h.mode_zero_hold_allowed=zeroHoldAllowed(now);
    h.mode_transition_reason=reason_;
    if(fixedStair()){
      h.required_motion_mode="stair";h.mode_runtime_parameters_ready=runtimeParametersReady();
      h.mode_transition_reason=!selectedModeObserved()?"fixed_stair_requires_selected_stair_mode":
        !h.mode_runtime_parameters_ready?(reusesGeneralRuntime()?"general_runtime_parameters_invalid":
          (capabilities_.reason.empty()?"stair_move_speed_and_hold_unverified":capabilities_.reason)):"";
    }
  }
  std::optional<SdkMoveFractions> stairFractions(double forward,double yaw)const {
    if(reusesGeneralRuntime())return {}; // The entire normal mapping, not these measured-mode linear scales.
    const auto*m=activeModeObserved()?activeCapability():nullptr;
    if(!m||m->name!="stair"||!std::isfinite(forward)||!std::isfinite(yaw))return {};
    return sdkMoveFractions(std::clamp(forward,0.,m->max_speed),0.,
      std::clamp(yaw,-m->max_yaw,m->max_yaw),"forward_yaw",m->forward_scale,m->forward_scale,m->yaw_scale);
  }
  bool activeConfirmed(double now)const {return activeModeObserved()&&robotFresh(now)&&robot_.owned&&robot_.clear;}
  bool zeroHoldAllowed(double now)const {if(!robotFresh(now)||!robot_.owned||!robot_.clear)return false;
    // Explicit reuse includes ordinary Move(0,0,0) submission semantics, but
    // never certifies measured stair holding or physical stop behaviour.
    if(reusesGeneralRuntime()&&general_runtime_valid_&&selectedModeObserved())return true;
    for(const auto&m:capabilities_.modes)if(m.verified&&robotMatches(m))return true;
    return false;}
  bool hasRequest()const{return request_.has_value();}
  std::string requestId()const{return request_?request_->request_id:std::string();}
  const std::string&phase()const{return phase_;}
  const std::string&reason()const{return reason_;}
  ModeState state(double now) {
    ModeState s;if(!request_)return s;const auto&r=*request_;
    s.schema_version=1;s.version=r.version;s.request_id=r.request_id;s.request_sequence=r.sequence;s.sequence=++state_sequence_;
    s.target_segment_id=r.target_segment_id;s.target_mode=r.target_mode;s.transport_mode=transport_;
    s.execution_id=r.execution_id;s.control_epoch=r.control_epoch;s.sdk_session=sdk_;s.sdk_arm_generation=r.sdk_arm_generation;
    s.phase=phase_;s.reason=reason_;s.time_basis=transport_=="live"?"sdk_callback_receipt_no_vendor_stamp":"isolated_mock_callback_receipt";
    s.robot_callback_sequence=robot_.sequence;s.robot_callback_stamp=stamp(robot_.receipt);
    s.source_stamp=stamp(now);s.valid_until=stamp(std::max(0.,std::min(now+.5,robot_.receipt+2.5)));
    s.sdk_mode=robot_.mode;s.motion_status=robot_.motion;s.speed_level=robot_.speed;
    s.control_owned=robot_.owned;s.emergency_clear=robot_.clear;
    const auto*target=capabilities_.find(r.target_mode);s.capability_verified=target&&target->verified;
    s.confirmed=phase_=="ready"&&active_mode_==r.target_mode&&activeConfirmed(now);return s;
  }
 private:
  struct Robot {int mode=0,motion=0,speed=0,head=0;bool owned=false,clear=false;uint64_t sequence=0;double receipt=0.;};
  static bool leaseMatches(const ModeRequest&r,const TaskLease&p,double now){return p.schema_version==1&&!p.revoked&&
    p.phase!="stopping"&&p.phase!="terminal"&&task(p.version,r.version)&&p.execution_id==r.execution_id&&
    p.control_epoch==r.control_epoch&&p.sdk_session==r.sdk_session&&p.sdk_arm_generation==r.sdk_arm_generation&&
    p.transport_mode==r.transport_mode&&fresh(secs(p.source_stamp),now,.35)&&secs(p.valid_until)>=now;}
  bool robotFresh(double now)const{return robot_.sequence&&fresh(robot_.receipt,now,2.5);}
  bool robotMatches(const ModeCapability&m)const{return robot_.mode==m.sdk_mode&&robot_.motion==m.motion_status&&robot_.speed==1&&robot_.head==1;}
  void fail(const char*why){if(awaitingModeChange())uncertain_dispatch_=true;phase_="failed";reason_=why;}
  std::string sdk_,transport_,active_mode_="general",phase_="idle",reason_;ModeCapabilities capabilities_;
  ExecutionModePolicy policy_=ExecutionModePolicy::SegmentModes;
  StairRuntimeParameterSource runtime_source_=StairRuntimeParameterSource::EvidenceRecord;
  bool general_runtime_valid_=false;
  std::optional<ModeRequest>request_;Robot robot_;double sent_=0.,ack_=0.,confirm_first_=0.;
  uint64_t state_sequence_=0,ack_robot_sequence_=0;unsigned confirm_count_=0;
  bool uncertain_dispatch_=false;
};

// The live unique writer and isolated checks use this SAME mapping function.
// A user-authorized fixed-STAIR runtime reuse preserves the entire established
// normal-mode map, including nonlinear axis mapping, slew state and exact zero
// handling; it does not copy/mark a physical capability record as verified.
inline std::optional<SdkMoveFractions> executionMoveFractions(const ModeTransition*mode,
    const Acceptance&runtime,bool task_writer,const geometry_msgs::msg::Twist&command,
    SdkFractionSlew&slew,const SdkMappingIdentity&identity,double at,
    bool goal_deceleration=false,bool yaw_coasting=false) {
  if(mode){
    if(const auto measured=mode->stairFractions(command.linear.x,command.angular.z))return measured;
    if(mode->fixedStair()&&!(mode->reusesGeneralRuntime()&&mode->runtimeParametersReady()&&runtime.valid&&mode->selectedModeObserved())) {
      if(planarMotionNonzero(command))return {};
      return SdkMoveFractions{};
    }
  }
  if(task_writer&&runtime.sdk_input_mapping.enabled){
    const auto target=mappedSdkFractions(command.linear.x,command.linear.y,command.angular.z,
      runtime.kinematic_profile,runtime.sdk_input_mapping,runtime.max_speed,runtime.max_yaw);
    return slew.apply(target,command.linear.x,command.linear.y,command.angular.z,runtime.sdk_input_mapping,
      identity,at,goal_deceleration,yaw_coasting);
  }
  return sdkMoveFractions(command.linear.x,command.linear.y,command.angular.z,runtime.kinematic_profile,
    runtime.forward_scale,runtime.lateral_scale,runtime.yaw_scale);
}
}
