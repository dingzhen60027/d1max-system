#include "mode_transition_core.hpp"
#include "test_execution_fixture.hpp"
#include <cassert>
#include <iostream>
#include <unistd.h>
using namespace d1monitor::execution3;
static Version v(){Version v;v.schema_version=3;v.session_id="nav";v.task_id="task";v.route_id="route";
  v.route_hash=std::string(64,'a');v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";
  v.reference_generation=1;v.segment_id="lower";v.anchor_id="anchor";v.anchor_revision=1;v.context_sequence=1;return v;}
static Health health(){return {true,true,true,true,false,true};}
static Transport writer(Health initial=health()){Transport w("sdk","isolated_mock");w.configureTaskFollower(true);
  w.configureMcCaptureBound(.02,"isolated_simulated_source_clock");w.configureExecutionPolicy(testExecutionPolicy(),.8);
  w.configureBrakingModel(std::string(64,'c'),.3,.5,"forward_yaw");w.setSDKTransactions(0,false);
  Grant::Request g;g.version=v();g.sdk_session="sdk";g.execution_id="exec";g.control_epoch=1;
  g.confirmation_id="confirm";g.request_id="grant";g.source_stamp=stamp(10.);g.activate=true;g.transport_mode="isolated_mock";
  assert(w.grant(g,10.,initial).accepted);return w;}
static TaskLease lease(double t){TaskLease l;l.schema_version=1;l.version=v();l.execution_id="exec";l.control_epoch=1;
  l.confirmation_id="confirm";l.sdk_session="sdk";l.sdk_arm_generation=1;l.sequence=uint64_t(t*1000);
  l.source_stamp=stamp(t);l.valid_until=stamp(t+.35);l.allowed=false;l.phase="holding";l.transport_mode="isolated_mock";return l;}
static ModeRequest request(){ModeRequest r;r.schema_version=1;r.version=v();r.request_id="stairs";r.sequence=1;
  r.target_segment_id="stair_lower";r.current_mode="general";r.target_mode="stair";r.transport_mode="isolated_mock";
  r.execution_id="exec";r.control_epoch=1;r.sdk_session="sdk";r.sdk_arm_generation=1;
  r.source_stamp=stamp(10.);r.valid_until=stamp(20.);r.body_source_stamp=stamp(9.99);return r;}
static ModeCapabilities fixture(){ModeCapabilities p;p.reason.clear();for(auto&m:p.modes){m.verified=true;
  m.record_sha256="isolated_fixture_only";m.forward_scale=m.yaw_scale=1.;m.max_speed=.3;m.max_yaw=.5;}return p;}
static Stationary stopped(){Stationary s;s.schema_version=1;s.version=v();s.execution_id="exec";s.control_epoch=1;
  s.sdk_session="sdk";s.sdk_arm_generation=1;s.transport_mode="isolated_mock";s.usable=s.nonzero_blocked=true;
  s.zero_write_sequence=1;s.mc_raw_stamp_ns=10100000000ULL;s.mc_clock_epoch="clock";
  s.zero_ack_at=stamp(10.01);s.capture_lower_bound=stamp(10.2);s.valid_until=stamp(10.45);
  s.stationary_samples=4;s.stationary_duration_sec=.2;return s;}
static void dispatch(ModeTransition&m,Transport&w){m.observeRobot(1,5,1,1,true,true,1,10.,10.);
  assert(m.begin(request(),w,lease(10.),10.));m.tick(w,lease(10.3),stopped(),10.3);
  assert(m.takeDispatch(10.3)==3);assert(!m.takeDispatch(10.3));}
int main(){
  auto w=writer();
  // Fixed STAIR observes the mode already selected by the operator. Floors
  // and stair segments cannot request a SetMode, including stale old traffic.
  const auto fixed_policy=executionModePolicy("fixed_stair");
  assert(executionModePolicy("segment_modes")==ExecutionModePolicy::SegmentModes);
  bool bad_policy=false;try{(void)executionModePolicy("automatic");}catch(const std::invalid_argument&){bad_policy=true;}assert(bad_policy);
  ModeTransition fixed("sdk","isolated_mock",fixture(),fixed_policy);
  fixed.observeRobot(3,7,1,1,false,true,1,10.,10.);
  auto fh=health();fh.owned=false;fh.general=false;fixed.applyHealth(fh,10.);
  assert(fh.motion_mode=="stair"&&fh.motion_mode_confirmed&&motionModeAllowed(fh)&&!fh.general);
  assert(!fh.mode_transition_active&&fh.required_motion_mode=="stair");
  assert(!fixed.begin(request(),w,lease(10.),10.));fixed.tick(w,lease(10.),stopped(),10.);
  assert(!fixed.takeDispatch(10.)&&!fixed.hasRequest());
  fixed.acknowledge(1,10.01,10.01); // an unrelated/manual ACK cannot select GENERAL
  assert(fixed.activeModeObserved());
  auto fractions=fixed.stairFractions(.2,-.15);assert(fractions);
  assert(std::abs(fractions->forward_back-.2)<1e-12&&std::abs(fractions->yaw+.15)<1e-12&&fractions->left_right==0.);
  // Observe STAIRS before control acquisition, then acquire once as usual.
  auto fw=writer(fh);assert(fw.takeControlOnce()&&!fw.takeControlOnce());
  fw.tick(10.005,fh);assert(fw.state(10.005).phase=="taking_control"&&!fw.stopping());
  auto fixed_owned=fh;fixed_owned.owned=true;
  fw.tick(10.01,fixed_owned);auto fs=fw.state(10.01);
  assert(!fs.general_low_speed_confirmed&&fs.motion_mode=="stair"&&fs.motion_mode_confirmed&&fs.grant_ready);
  // A source gap is still a temporary HOLD, not an invented mode change.
  fixed_owned.robot_state_fresh=false;fw.tick(10.02,fixed_owned);
  assert(!fw.stopping()&&!fw.state(10.02).grant_ready);
  fixed_owned.robot_state_fresh=true;fw.tick(10.03,fixed_owned);assert(!fw.stopping());
  fixed_owned.owned=false;fw.tick(10.04,fixed_owned);assert(fw.stopping()); // no auto reacquisition
  // No GENERAL scale cache or stale mode fact can leak into a new task.
  fixed.observeRobot(1,5,1,1,true,true,2,10.1,10.1);fh=health();fixed.applyHealth(fh,10.1);
  assert(fh.general&&!motionModeAllowed(fh)&&!fixed.stairFractions(.2,0.));
  assert(fh.mode_transition_reason=="fixed_stair_requires_selected_stair_mode");
  fixed.observeRobot(3,7,1,1,true,true,3,10.2,10.2);fh=health();fh.general=false;fixed.applyHealth(fh,10.2);
  assert(motionModeAllowed(fh)&&fixed.stairFractions(.2,0.));
  auto emergency=writer();fh.estop=true;emergency.tick(10.21,fh);assert(emergency.stopping());
  // Actual mode is reported even without a measured Move capability. It is
  // not a grant, does not fake physical acceptance and never maps via GENERAL.
  ModeTransition fixed_missing("sdk","live",{},fixed_policy);
  fixed_missing.observeRobot(3,7,1,1,true,true,1,10.,10.);
  fh=health();fh.general=false;fixed_missing.applyHealth(fh,10.);
  assert(fh.motion_mode_confirmed&&!motionModeAllowed(fh)&&!fixed_missing.stairFractions(.2,0.));
  assert(fh.mode_transition_reason=="stair_move_speed_and_hold_unverified");
  Transport unarmed("sdk","isolated_mock");unarmed.tick(10.,fh);
  assert(unarmed.state(10.).motion_mode_confirmed&&!unarmed.state(10.).grant_ready);
  assert(!fixed_missing.begin(request(),w,lease(10.),10.)&&!fixed_missing.takeDispatch(10.));
  // Explicit USER runtime reuse is a separate source, not a fictitious STAIR
  // measurement. Exercise the live Transport core without any ROS/SDK object:
  // actual-STAir feedback -> grant -> lease -> local command -> shared mapping
  // -> nonzero write intent, preserving exact normal mapping over all segments.
  {
    const auto reuse=stairRuntimeParameterSource("general_runtime_reuse");
    assert(stairRuntimeParameterSource("evidence_record")==StairRuntimeParameterSource::EvidenceRecord);
    bool invalid_source=false;try{(void)stairRuntimeParameterSource("measured_by_user_request");}
      catch(const std::invalid_argument&){invalid_source=true;}assert(invalid_source);
    ModeTransition runtime_mode("sdk","live",{},fixed_policy,reuse,true);
    runtime_mode.observeRobot(3,7,1,1,false,true,1,10.,10.);
    auto rh=health();rh.general=false;rh.owned=false;runtime_mode.applyHealth(rh,10.);
    assert(motionModeAllowed(rh)&&rh.mode_runtime_parameters_ready&&!runtime_mode.measuredEvidenceVerified());
    assert(!runtime_mode.zeroHoldAllowed(10.)&&runtime_mode.reusesGeneralRuntime());
    assert(std::string(runtime_mode.runtimeParameterBasis())=="user_authorized_general_runtime_reuse_not_stair_calibration");
    assert(!runtime_mode.stairFractions(.2,.1)); // do not take the measured linear scale branch
    Transport live("sdk","live",true,.3,.5,std::string(64,'c'),"forward_yaw",false);
    live.configureTaskFollower(true);live.configureMcCaptureBound(.02,"source_delta_host_anchor_approximate");
    live.configureExecutionPolicy(testExecutionPolicy(),.8);live.setSDKTransactions(0,false);
    Grant::Request g;g.version=v();g.sdk_session="sdk";g.execution_id="exec";g.control_epoch=1;
    g.confirmation_id="confirm";g.request_id="grant";g.source_stamp=stamp(10.);g.activate=true;g.transport_mode="live";
    // Identical callback/profile/grant input: only the declared runtime source
    // changes. Empty evidence_record must remain unverified, while explicit
    // user reuse may take ownership once without falsely certifying evidence.
    auto baseline=live;auto prior_health=rh;fixed_missing.applyHealth(prior_health,10.);
    const auto prior=baseline.grant(g,10.,prior_health);
    assert(!prior.accepted&&prior.reason=="stair_move_speed_and_hold_unverified");
    assert(!baseline.tick(10.001,prior_health)&&!baseline.takeControlOnce());
    assert(live.grant(g,10.,rh).accepted);
    assert(live.takeControlOnce()&&!live.takeControlOnce());
    assert(!live.tick(10.005,rh)&&live.state(10.005).phase=="taking_control");
    runtime_mode.observeRobot(3,7,1,1,true,true,2,10.01,10.01);rh.owned=true;runtime_mode.applyHealth(rh,10.01);
    live.tick(10.01,rh);assert(live.state(10.01).grant_ready&&!live.stopping());
    assert(runtime_mode.zeroHoldAllowed(10.01)&&!live.state(10.01).general_low_speed_confirmed);
    Acceptance a;a.valid=true;a.physically_verified=false;a.kinematic_profile="forward_yaw";
    a.sdk_input_mapping.enabled=true;a.sdk_input_mapping.component_cap=true;a.sdk_input_mapping.continuous_command_ratio=true;
    a.sdk_input_mapping.validate();SdkFractionSlew stair_slew,general_slew;SdkMappingIdentity identity{"exec",1,1};
    unsigned nonzero_writes=0;
    for(unsigned i=1;i<=60;++i){const double now=10.+i*.05;
      runtime_mode.observeRobot(3,7,1,1,true,true,i+1,now,now);runtime_mode.applyHealth(rh,now);
      auto permission=lease(now);permission.transport_mode="live";permission.allowed=true;permission.phase="tracking";
      assert(live.taskLease(permission,now));
      LocalCommand c;c.schema_version=1;c.version=v();
      c.version.segment_id=i<15?"floor1":i<30?"stair_lower":i<45?"stair_upper":"floor2";
      c.execution_id="exec";c.control_epoch=1;c.sdk_session="sdk";c.sdk_arm_generation=1;
      c.lease_sequence=permission.sequence;c.sequence=i;c.source_stamp=c.body_source_stamp=stamp(now);
      c.body_valid_until=c.valid_until=stamp(now+.1);c.body_frame="d1max_loc_base_link";
      c.velocity.linear.x=.12+double(i%10)*.015;c.velocity.angular.z=.05;c.transport_mode="live";c.trajectory_id=i;
      assert(live.localCommand(c,now));live.mc(uint64_t(now*1e9),now,now,.2,0.,now,"clock");
      const auto selected=live.tick(now,rh);assert(selected&&planarMotionNonzero(*selected)&&!live.stopping());
      const auto stair=executionMoveFractions(&runtime_mode,a,true,*selected,stair_slew,identity,now);
      const auto general=executionMoveFractions(nullptr,a,true,*selected,general_slew,identity,now);
      assert(stair&&general&&stair->left_right==0.&&stair->forward_back>0.);
      assert(stair->forward_back==general->forward_back&&stair->yaw==general->yaw);
      live.moveAttempted(true);live.writeCalled(now,true,true);++nonzero_writes;
      assert(!runtime_mode.takeDispatch(now)&&!runtime_mode.hasRequest());
      assert(!live.report(now).physical_acceptance_verified);
    }
    assert(nonzero_writes==60&&!runtime_mode.measuredEvidenceVerified());
    // All existing runtime boundaries still apply to this explicit reuse
    // source. Clone the genuinely moving core, not a measured-capability
    // fixture, so each fault starts from the same accepted command/ownership.
    for(unsigned fault=0;fault<3;++fault){auto affected=live;auto unhealthy=rh;
      if(fault==0)unhealthy.owned=false;
      if(fault==1)unhealthy.estop=true;
      if(fault==2){affected.moveAttempted(true);affected.writeCalled(13.005,false,true);}
      const auto stopped_output=affected.tick(13.01,unhealthy);
      assert(!stopped_output||!planarMotionNonzero(*stopped_output));
      assert(affected.stopping()&&affected.state(13.01).fault_latched&&!affected.takeControlOnce());
      assert(!affected.report(13.01).physical_acceptance_verified&&!affected.report(13.01).measured_stop_confirmed);
      if(fault==2)assert(affected.state(13.01).reason=="sdk_write_outcome_unknown");
    }
    {auto expired=live;
      const auto expired_command=expired.tick(13.11,rh);
      assert(expired_command&&!planarMotionNonzero(*expired_command)&&!expired.stopping());
      assert(expired.state(13.11).reason=="waiting_local_command");
      const auto expired_lease=expired.tick(13.36,rh);
      assert(expired_lease&&!planarMotionNonzero(*expired_lease)&&!expired.stopping());
      assert(expired.state(13.36).reason=="task_lease_unavailable"&&!expired.report(13.36).physical_acceptance_verified);
    }
    // Zero/stop uses the same writer and becomes EXACT zero, not a minimum
    // breakaway fraction; no physical stop certificate is invented by ACK.
    live.stop("owner_cancelled",13.01);const auto selected_zero=live.tick(13.01,rh);
    assert(selected_zero&&!planarMotionNonzero(*selected_zero));
    const auto zero=executionMoveFractions(&runtime_mode,a,true,*selected_zero,stair_slew,identity,13.01);
    assert(zero&&zero->left_right==0.&&zero->forward_back==0.&&zero->yaw==0.);
    assert(!live.report(13.01).physical_acceptance_verified&&!live.report(13.01).measured_stop_confirmed);
    live.moveAttempted(false);live.writeCalled(13.01,true,false);live.stopWritten(13.02);
    assert(live.report(13.02).stop_submitted&&!live.report(13.02).measured_stop_confirmed);
    for(unsigned i=0;i<=24;++i){const double now=13.1+i*.05;
      runtime_mode.observeRobot(3,7,1,1,true,true,100+i,now,now);runtime_mode.applyHealth(rh,now);
      live.mc(uint64_t(now*1e9),now,now,0.,0.,now,"clock");
      const auto after_zero=live.tick(now,rh);assert(!after_zero||!planarMotionNonzero(*after_zero));
    }
    const auto terminal_stop=live.report(14.301);
    assert(terminal_stop.measured_stop_confirmed&&!terminal_stop.physical_acceptance_verified);
    assert(terminal_stop.velocity_metric=="spatial_norm_3d");
    assert(live.report(14.6)==terminal_stop); // immutable past event, no timer refresh
    assert(!live.currentStopReport(14.6).measured_stop_confirmed);
    assert(!live.mcSourceFresh(14.6,"clock")&&!live.stationaryEvidence(14.6).usable);
    // Wrong real mode cannot use even explicitly authorized runtime reuse.
    runtime_mode.observeRobot(1,5,1,1,true,true,200,14.31,14.31);rh.general=true;runtime_mode.applyHealth(rh,14.31);
    assert(!motionModeAllowed(rh));geometry_msgs::msg::Twist attempt;attempt.linear.x=.2;
    assert(!executionMoveFractions(&runtime_mode,a,true,attempt,stair_slew,identity,14.31));
    ModeTransition invalid_runtime("sdk","live",{},fixed_policy,reuse,false);
    invalid_runtime.observeRobot(3,7,1,1,true,true,1,10.,10.);rh.general=false;invalid_runtime.applyHealth(rh,10.);
    assert(!motionModeAllowed(rh)&&rh.mode_transition_reason=="general_runtime_parameters_invalid");
  }
  // The loader's required evidence matches the actual policy: no switch or
  // GENERAL evidence for fixed STAIR. These files are isolated unit fixtures,
  // never installed acceptance records or claimed physical measurements.
  {
    char directory[]="/tmp/d1max-fixed-mode-test-XXXXXX";const auto*created=mkdtemp(directory);assert(created);
    const auto dir=std::filesystem::path(created),evidence=dir/"synthetic-unit-fixture.txt",record=dir/"mode.json";
    {std::ofstream out(evidence);out<<"synthetic unit-test evidence, not robot acceptance";}
    const auto evidence_hash=fileSha256(evidence.string());
    nlohmann::json p={{"sdk_mode",3},{"motion_status",7},{"forward_scale_mps",.8},{"yaw_scale_radps",1.2},
      {"maximum_speed_mps",.3},{"maximum_yaw_radps",.5},{"measurements",nlohmann::json::object()}};
    for(const auto*kind:{"move","zero_hold","speed_mapping"})p["measurements"][kind]=
      {{"path",evidence.filename().string()},{"sha256",evidence_hash},{"passed",true}};
    nlohmann::json j={{"schema_version",1},{"record_kind","locomotion_mode_measurements"},{"transport_mode","live"},
      {"fixture_only",false},{"robot_id","unit-fixture"},{"sdk_version","fixture"},{"robot_profile_sha256","fixture"},
      {"firmware_version","fixture"},{"modes",{{"stair",p}}}};
    auto write_record=[&]{std::ofstream out(record);out<<j.dump();};write_record();
    auto loaded=loadModeCapabilities(record.string(),fileSha256(record.string()),"unit-fixture","fixture","fixture",fixed_policy);
    assert(loaded.find("stair")->verified&&!loaded.find("general")->verified);
    assert(!loadModeCapabilities(record.string(),fileSha256(record.string()),"unit-fixture","fixture","fixture").find("stair")->verified);
    ModeTransition scaled("sdk","isolated_mock",loaded,fixed_policy);scaled.observeRobot(3,7,1,1,true,true,1,10.,10.);
    const auto mapped=scaled.stairFractions(.24,.24);assert(mapped&&std::abs(mapped->forward_back-.3)<1e-12&&std::abs(mapped->yaw-.2)<1e-12);
    j["modes"]["stair"]["measurements"].erase("zero_hold");write_record();
    assert(!loadModeCapabilities(record.string(),fileSha256(record.string()),"unit-fixture","fixture","fixture",fixed_policy).find("stair")->verified);
    std::filesystem::remove(record);std::filesystem::remove(evidence);std::filesystem::remove(dir);
  }
  // Missing physical evidence is specific, stable, and cannot issue SetMode.
  ModeTransition missing("sdk","isolated_mock");missing.observeRobot(1,5,1,1,true,true,1,10.,10.);
  assert(missing.begin(request(),w,lease(10.),10.));assert(missing.phase()=="capability_unverified");
  assert(!missing.takeDispatch(10.1));assert(!missing.state(10.1).confirmed);
  assert(!loadModeCapabilities("","","robot","sdk","profile").find("stair")->verified);
  // Complete pure transaction: stop evidence, one intent, ACK, then NEW states.
  ModeTransition m("sdk","isolated_mock",fixture());dispatch(m,w);m.submitted(true);
  m.observeRobot(3,7,1,1,true,true,2,10.31,10.31);assert(m.phase()=="waiting_ack");
  m.acknowledge(1,10.32,10.32);assert(m.phase()=="waiting_ack");
  m.acknowledge(3,10.32,10.32);m.observeRobot(3,7,1,1,true,true,3,10.4,10.4);
  assert(!m.state(10.4).confirmed);m.observeRobot(3,7,1,1,true,true,4,10.51,10.51);
  auto s=m.state(10.52);assert(s.confirmed&&s.capability_verified&&s.sdk_mode==3);
  assert(s.time_basis=="isolated_mock_callback_receipt"&&s.robot_callback_stamp==stamp(10.51));
  assert(!m.holding()&&m.activeConfirmed(10.52));assert(!m.state(13.1).confirmed&&m.activeModeObserved());
  // Same STAIR mode at a platform binds new segment without pause or SDK write.
  auto next=request();next.request_id="upper";next.sequence=2;next.version.segment_id="stair_lower";
  next.target_segment_id="stair_upper";next.current_mode="stair";next.source_stamp=stamp(10.52);
  next.body_source_stamp=stamp(10.51);next.valid_until=stamp(20.52);
  assert(m.begin(next,w,lease(10.52),10.52));assert(m.state(10.52).confirmed&&!m.holding());assert(!m.takeDispatch(10.53));
  auto exit=next;exit.request_id="exit";exit.sequence=3;exit.version.segment_id="stair_upper";
  exit.target_segment_id="floor2";exit.target_mode="general";exit.source_stamp=stamp(10.6);
  exit.body_source_stamp=stamp(10.59);exit.valid_until=stamp(20.6);
  m.observeRobot(3,7,1,1,true,true,5,10.6,10.6);assert(m.begin(exit,w,lease(10.6),10.6));
  auto exit_stop=stopped();exit_stop.zero_ack_at=stamp(10.61);exit_stop.capture_lower_bound=stamp(10.85);exit_stop.valid_until=stamp(11.1);
  m.tick(w,lease(10.9),exit_stop,10.9);assert(m.takeDispatch(10.9)==1);
  m.acknowledge(1,10.91,10.91);m.observeRobot(1,5,1,1,true,true,6,11.,11.);
  m.observeRobot(1,5,1,1,true,true,7,11.12,11.12);assert(m.state(11.12).confirmed&&m.activeCapability()->name=="general");
  // Cached capability is not current telemetry: manual mode changes must
  // not carry the previous mode's SDK mapping into a subsequent task.
  ModeTransition manual("sdk","isolated_mock",fixture());dispatch(manual,w);
  manual.acknowledge(3,10.31,10.31);manual.observeRobot(3,7,1,1,true,true,2,10.4,10.4);
  manual.observeRobot(3,7,1,1,true,true,3,10.51,10.51);assert(manual.activeModeObserved());
  manual.observeRobot(1,5,1,1,true,true,4,10.6,10.6);assert(!manual.activeModeObserved());
  // Late ACK cannot revive a canceled task or be reused in a new transaction.
  ModeTransition canceled("sdk","isolated_mock",fixture());dispatch(canceled,w);auto revoked=lease(10.31);revoked.revoked=true;
  canceled.tick(w,revoked,stopped(),10.31);canceled.acknowledge(3,10.32,10.32);
  canceled.observeRobot(3,7,1,1,true,true,2,10.4,10.4);assert(!canceled.state(10.4).confirmed);
  assert(!canceled.begin(next,w,lease(10.52),10.52));
  ModeTransition timeout("sdk","isolated_mock",fixture());dispatch(timeout,w);
  timeout.observeRobot(1,5,1,1,true,true,2,12.31,12.31);timeout.tick(w,lease(12.31),stopped(),12.31);
  assert(timeout.reason()=="mode_ack_timeout");timeout.acknowledge(3,12.32,12.32);assert(!timeout.state(12.32).confirmed);
  ModeTransition failed("sdk","isolated_mock",fixture());dispatch(failed,w);failed.submitted(false);
  assert(failed.reason()=="set_mode_write_outcome_unknown");assert(!failed.takeDispatch(10.4));
  ModeTransition bad("sdk","isolated_mock",fixture());auto r=request();r.sdk_arm_generation=2;
  assert(!bad.begin(r,w,lease(10.),10.));r=request();r.preview_only=true;assert(!bad.begin(r,w,lease(10.),10.));
  r=request();r.target_mode="auto";assert(!bad.begin(r,w,lease(10.),10.));
  // No zero/old MC evidence can dispatch, and repeated request is idempotent.
  ModeTransition wait("sdk","isolated_mock",fixture());wait.observeRobot(1,5,1,1,true,true,1,10.,10.);
  assert(wait.begin(request(),w,lease(10.),10.));assert(wait.begin(request(),w,lease(10.01),10.01));
  auto wrong=stopped();wrong.capture_lower_bound=stamp(9.);wait.tick(w,lease(10.3),wrong,10.3);assert(!wait.takeDispatch(10.3));
  wrong=stopped();wrong.sdk_arm_generation=2;wait.tick(w,lease(10.3),wrong,10.3);assert(!wait.takeDispatch(10.3));
  // Exercise real Transport's task-level zero/MC evidence producer, not only
  // a hand-built Stationary message. Mode hold does not retire task identity.
  auto integrated_writer=writer();ModeTransition integrated("sdk","isolated_mock",fixture());
  integrated.observeRobot(1,5,1,1,true,true,1,10.,10.);
  assert(integrated.begin(request(),integrated_writer,lease(10.),10.));integrated_writer.beginModeHold();
  bool dispatched=false;
  for(unsigned i=1;i<30;++i){const double now=10.+i*.05;const auto permission=lease(now);
    assert(integrated_writer.taskLease(permission,now));
    integrated_writer.mc(uint64_t(now*1e9),now,now,0.,0.,now,"clock");
    auto h=health();h.mode_transition_active=true;h.mode_transition_reason=integrated.reason();
    const auto zero=integrated_writer.tick(now,h);assert(zero&&!planarMotionNonzero(*zero));
    const auto write=integrated_writer.nextWriteSequence();integrated_writer.moveAttempted(false);
    integrated_writer.writeCalled(now,true,false);integrated_writer.zeroAcknowledged(write,now,now);
    integrated.observeRobot(1,5,1,1,true,true,i+1,now,now);
    integrated.tick(integrated_writer,permission,integrated_writer.stationaryEvidence(now),now);
    if(integrated.takeDispatch(now)){dispatched=true;assert(!integrated_writer.stopping());break;}
  }
  assert(dispatched);
  std::cout<<"mode policy: fixed STAIR/no dispatch/actual observation; same-input evidence_record: grant rejected/0 writes; explicit runtime reuse: one takeover/60 live-core nonzero intents/normal mapping parity/zero cancel/measured stop, physical flags false; reuse estop/control loss/write failure/expiry and transaction cases passed\n";
}
