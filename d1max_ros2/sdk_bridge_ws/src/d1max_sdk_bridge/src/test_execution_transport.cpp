#include "execution_transport_core.hpp"
#include "test_execution_fixture.hpp"
#include "sdk_write_transactions.hpp"
#include "sdk_ownership_core.hpp"
#include "execution_robot_health.hpp"
#include "mc_report_core.hpp"
#include "execution_scheduler.hpp"
#include <array>
#include <cassert>
#include <iostream>
#include <limits>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <functional>
using namespace d1monitor::execution3;
static Version v(){Version x;x.schema_version=3;x.session_id="nav";x.task_id="task";x.route_id="route";
 x.route_hash=std::string(64,'a');x.map_version_id="map";x.localization_epoch=1;x.localization_seed_id="seed";
 x.reference_generation=1;x.segment_id="floor";x.anchor_id="anchor";x.anchor_revision=x.context_sequence=x.map_geometry_revision=1;return x;}
static Health h(){return {true,true,true,true,false,true};}
static Grant::Request g(){Grant::Request x;x.version=v();x.sdk_session="sdk";x.execution_id="execution";x.control_epoch=1;
 x.confirmation_id="confirmation";x.request_id="request";x.source_stamp=stamp(10);x.activate=true;x.transport_mode="isolated_mock";return x;}
static Permit p(){Permit x;x.version=v();x.execution_id="execution";x.control_epoch=1;x.sdk_session="sdk";x.sdk_arm_generation=1;
 x.sequence=x.validation_sequence=1;x.trajectory_id=1;x.source_stamp=stamp(10);x.valid_until=stamp(10.25);x.allowed=true;
 x.phase="tracking";x.transport_mode="isolated_mock";x.geometry_committed=true;x.frame_id="d1max_loc_odom";return x;}
static Demand d(){Demand x;x.version=v();x.execution_id="execution";x.control_epoch=1;x.sdk_session="sdk";x.sdk_arm_generation=1;
 x.sequence=x.permit_sequence=x.validation_sequence=1;x.trajectory_id=1;x.source_stamp=x.body_source_stamp=x.safety_source_stamp=stamp(10);
 x.valid_until=stamp(10.2);x.velocity.linear.x=.2;x.safety_checked=true;x.transport_mode="isolated_mock";
 x.motion_validation_sequence=1;x.braking_model_sha256=std::string(64,'c');return x;}
static MotionProof proof(const Demand&d,double now=10.,uint64_t sequence=1){MotionProof p;
 p.version=d.version;p.execution_id=d.execution_id;p.control_epoch=d.control_epoch;p.sdk_session=d.sdk_session;p.sdk_arm_generation=d.sdk_arm_generation;
 p.trajectory_id=d.trajectory_id;p.permit_sequence=d.permit_sequence;p.trajectory_validation_sequence=d.validation_sequence;
 p.demand_sequence=d.sequence;p.demand_source_stamp=d.source_stamp;p.demand_body_source_stamp=d.body_source_stamp;p.demand_valid_until=d.valid_until;
 p.sequence=sequence;p.check_begin=p.check_end=p.body_source_stamp=p.front_ray_source_stamp=p.rear_ray_source_stamp=stamp(now);
 p.valid_until=d.valid_until;p.velocity=d.velocity;p.frame_id="d1max_loc_odom";p.valid=true;p.transport_mode=d.transport_mode;p.braking_model_sha256=d.braking_model_sha256;return p;}
static Transport ready(const ExecutionPolicy&policy=testExecutionPolicy(),const std::string&profile="forward_yaw"){Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
 t.configureExecutionPolicy(policy,std::max(.8,policy.reactionBudget()));
 t.configureBrakingModel(std::string(64,'c'),.3,.5,profile);assert(t.grant(g(),10,h()).accepted);
 t.tick(10,h());assert(t.permit(p(),10));assert(t.motionValidation(proof(d()),10));
 Local local;local.schema_version=2;local.session_id="nav";local.map_version_id="map";local.localization_epoch=1;local.localization_seed_id="seed";
 local.source_stamp=local.posterior_stamp=local.imu_stamp=stamp(10.);local.valid_until=stamp(10.1);local.usable=true;
 local.local_odometry.header.stamp=local.source_stamp;local.local_odometry.header.frame_id="d1max_loc_odom";
 local.local_odometry.child_frame_id="d1max_loc_base_link";local.local_odometry.pose.pose.orientation.w=1.;assert(t.localState(local,10.));
 CurveProof curve;curve.version=v();curve.trajectory_id=curve.sequence=1;curve.transport_mode="isolated_mock";
 curve.check_begin=curve.check_end=curve.source_stamp=curve.body_source_stamp=curve.front_ray_source_stamp=curve.rear_ray_source_stamp=stamp(10.);
 curve.valid_until=stamp(10.25);curve.valid=curve.whole_curve=true;curve.curve_duration=curve.checked_to_time=1.;
 curve.frame_id="d1max_loc_odom";curve.collision_policy="observed_free";curve.map_snapshot_revision=1;
 curve.support_reference_id="support";curve.support_hash=std::string(64,'d');assert(t.trajectoryValidation(curve,10.));return t;}
static void refreshBody(Transport&t,double now) {
 Local m;m.schema_version=2;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(now);m.valid_until=stamp(now+.1);m.usable=true;m.local_odometry.header.stamp=m.source_stamp;
 m.local_odometry.header.frame_id="d1max_loc_odom";m.local_odometry.child_frame_id="d1max_loc_base_link";
 m.local_odometry.pose.pose.orientation.w=1.;assert(t.localState(m,now));
 CurveProof c;c.version=v();c.trajectory_id=1;c.sequence=static_cast<uint64_t>(now*1e6);c.transport_mode="isolated_mock";
 c.check_begin=c.check_end=c.source_stamp=c.body_source_stamp=c.front_ray_source_stamp=c.rear_ray_source_stamp=stamp(now);
 c.valid_until=stamp(now+.25);c.valid=c.whole_curve=true;c.curve_duration=c.checked_to_time=1.;c.map_snapshot_revision=1;
 c.frame_id="d1max_loc_odom";c.collision_policy="observed_free";c.support_reference_id="support";c.support_hash=std::string(64,'d');
 assert(t.trajectoryValidation(c,now));
}
static CurveProof suffix(uint64_t sequence=2) {
 CurveProof c;c.version=v();c.trajectory_id=1;c.sequence=sequence;c.transport_mode="isolated_mock";
 c.check_begin=c.check_end=c.source_stamp=c.body_source_stamp=c.front_ray_source_stamp=c.rear_ray_source_stamp=stamp(10.);
 c.valid_until=stamp(10.08);c.valid=c.remaining_curve=true;c.curve_duration=c.checked_to_time=1.;
 c.checked_from_time=.05;c.valid_start_time=.2;c.reverse_margin_m=.15;
 c.frame_id="d1max_loc_odom";c.collision_policy="observed_free";c.map_snapshot_revision=2;
 c.support_reference_id="support";c.support_hash=std::string(64,'d');return c;
}
// The LocalFollower protocol is distinct from legacy SafeDemand. These tests
// exercise the production writer with an unchanged 20 Hz control sample whose
// SCAN environmental evidence improves before its first short deadline.
static TaskLease taskLeaseForEvidence(uint64_t sequence=1,double at=10.) {
 TaskLease x;x.schema_version=1;x.version=v();x.execution_id="execution";x.control_epoch=1;
 x.confirmation_id="confirmation";x.sdk_session="sdk";x.sdk_arm_generation=1;
 x.sequence=sequence;x.source_stamp=stamp(at);x.valid_until=stamp(at+.35);
 x.allowed=true;x.phase="tracking";x.transport_mode="isolated_mock";return x;
}
static LocalCommand localEvidenceCommand(uint64_t sequence=1,double at=10.) {
 LocalCommand x;x.schema_version=1;x.version=v();x.execution_id="execution";x.control_epoch=1;
 x.sdk_session="sdk";x.sdk_arm_generation=1;x.lease_sequence=1;x.sequence=sequence;
 x.source_stamp=x.body_source_stamp=stamp(at);x.body_valid_until=stamp(at+.1);
 x.valid_until=stamp(at+.041);x.body_frame="d1max_loc_base_link";x.velocity.linear.x=.2;
 x.trajectory_id=1;x.reason="tracking";x.transport_mode="isolated_mock";return x;
}
static Transport evidenceWriter(const LocalCommand&command=localEvidenceCommand()) {
 Transport t("sdk","isolated_mock");t.configureTaskFollower(true);
 t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
 t.configureExecutionPolicy(testExecutionPolicy(),.8);
 t.configureBrakingModel(std::string(64,'c'),.3,.5,"holonomic_xyyaw");
 t.setSDKTransactions(0,false);assert(t.grant(g(),10.,h()).accepted);
 assert(t.taskLease(taskLeaseForEvidence(),10.));assert(t.localCommand(command,10.001));return t;
}
static bool evidenceMoving(const std::optional<geometry_msgs::msg::Twist>&out) {
 return out&&planarMotionNonzero(*out);
}
static void testLocalEvidenceUpdates() {
 unsigned cases=0;
 const auto run=[&](const char*name,const std::function<void()>&test) {
   test();++cases;std::cout<<"PASS local evidence "<<name<<'\n';
 };
 run("only_deadline_increases_and_original_sources_expire",[]{
  auto t=evidenceWriter();auto c=localEvidenceCommand();const auto original=c;
  c.valid_until=stamp(10.08);assert(t.localCommand(c,10.02));
  assert(std::string(t.taskCommandAdmission().reason)=="evidence_updated");
  assert(t.lastDemand().sequence==original.sequence&&t.lastDemand().source_stamp==original.source_stamp&&
    t.lastDemand().body_source_stamp==original.body_source_stamp&&t.lastDemand().velocity==original.velocity);
  assert(evidenceMoving(t.tick(10.045,h())));const auto selected=t.selectedTaskCommandTiming();
  assert(selected.source==original.source_stamp&&selected.body_source==original.body_source_stamp&&selected.valid_until==c.valid_until);
  assert(!t.localCommand(c,10.046));c.valid_until=stamp(10.06);assert(!t.localCommand(c,10.047));
  assert(evidenceMoving(t.tick(10.048,h())));c.valid_until=stamp(10.1);assert(t.localCommand(c,10.05));
  assert(evidenceMoving(t.tick(10.099999,h())));
  assert(!evidenceMoving(t.tick(10.100001,h()))&&!t.localCommand(c,10.100002));
 });
 run("all_other_wire_fields_are_immutable",[]{
  const std::array<std::function<void(LocalCommand&)>,24> changes={{
   [](auto&c){++c.schema_version;},[](auto&c){c.version.task_id="other";},
   [](auto&c){++c.version.reference_generation;},[](auto&c){c.version.anchor_id="other";},
   [](auto&c){++c.version.context_sequence;},[](auto&c){++c.version.map_geometry_revision;},
   [](auto&c){c.execution_id="other";},[](auto&c){++c.control_epoch;},
   [](auto&c){c.sdk_session="other";},[](auto&c){++c.sdk_arm_generation;},
   [](auto&c){++c.lease_sequence;},[](auto&c){c.source_stamp=stamp(10.005);},
   [](auto&c){c.body_source_stamp=stamp(9.999);},[](auto&c){c.body_valid_until=stamp(10.099);},
   [](auto&c){c.body_frame="base_link";},[](auto&c){c.velocity.linear.x=.21;},
   [](auto&c){c.velocity.linear.y=.01;},[](auto&c){c.velocity.linear.z=.01;},
   [](auto&c){c.velocity.angular.x=.01;},[](auto&c){c.velocity.angular.y=.01;},
   [](auto&c){c.velocity.angular.z=.01;},[](auto&c){++c.trajectory_id;},
   [](auto&c){c.reason="changed";},[](auto&c){c.transport_mode="live";}
  }};
  for(const auto&change:changes){auto t=evidenceWriter();auto c=localEvidenceCommand();
   c.valid_until=stamp(10.09);change(c);assert(!t.localCommand(c,10.02));
   assert(evidenceMoving(t.tick(10.021,h()))&&t.lastDemand().sequence==1);
   assert(t.lastDemand().valid_until==stamp(10.041));}
 });
 run("same_sequence_cannot_replace_motion_with_zero_or_hold",[]{
  for(bool hold:{false,true}){auto t=evidenceWriter();auto c=localEvidenceCommand();c.valid_until=stamp(10.09);
   c.velocity=geometry_msgs::msg::Twist{};c.hold=hold;assert(!t.localCommand(c,10.02));assert(evidenceMoving(t.tick(10.021,h())));}
 });
 run("controller_and_body_deadlines_cannot_be_extended",[]{
  auto t=evidenceWriter();auto c=localEvidenceCommand();c.valid_until=stamp(10.100001);
  assert(!t.localCommand(c,10.02));c.body_valid_until=stamp(10.2);c.valid_until=stamp(10.15);
  assert(!t.localCommand(c,10.021));assert(!evidenceMoving(t.tick(10.042,h())));
 });
 run("expired_before_callback_cannot_revive_even_without_writer_poll",[]{
  for(double at:{10.041,10.041001,10.1}){auto t=evidenceWriter();auto c=localEvidenceCommand();
   c.valid_until=stamp(10.1);assert(!t.localCommand(c,at));assert(!evidenceMoving(t.tick(at+.000001,h())));}
 });
 run("expired_and_cleared_cannot_revive",[]{
  auto t=evidenceWriter();assert(!evidenceMoving(t.tick(10.042,h())));
  auto c=localEvidenceCommand();c.valid_until=stamp(10.1);assert(!t.localCommand(c,10.043));
 });
 run("already_zero_or_hold_cannot_be_renewed",[]{
  for(bool hold:{false,true}){auto c=localEvidenceCommand();c.velocity=geometry_msgs::msg::Twist{};c.hold=hold;auto t=evidenceWriter(c);
   c.valid_until=stamp(10.1);assert(!t.localCommand(c,10.02));assert(!evidenceMoving(t.tick(10.021,h())));}
 });
 run("newer_command_wins_and_delayed_renewal_does_not_clear_it",[]{
  auto t=evidenceWriter();auto newer=localEvidenceCommand(2,10.02);newer.velocity.linear.x=.25;
  assert(t.localCommand(newer,10.021));auto old=localEvidenceCommand();old.valid_until=stamp(10.1);
  assert(!t.localCommand(old,10.022));auto out=t.tick(10.023,h());assert(out&&out->linear.x==.25);
  assert(t.lastDemand().sequence==2&&t.lastDemand().valid_until==newer.valid_until);
 });
 run("newer_zero_including_unknown_lease_is_irreversible_for_old_command",[]{
  for(bool delayed_lease:{false,true}){auto t=evidenceWriter();auto zero=localEvidenceCommand(2,10.02);
   zero.velocity=geometry_msgs::msg::Twist{};zero.hold=true;if(delayed_lease)zero.lease_sequence=2;
   assert(t.localCommand(zero,10.021)!=delayed_lease);auto old=localEvidenceCommand();old.valid_until=stamp(10.1);
   assert(!t.localCommand(old,10.022));assert(!evidenceMoving(t.tick(10.023,h())));}
 });
 run("hold_revoke_disconnect_loss_of_control_and_estop_cannot_renew",[]{
  for(unsigned why=0;why<8;++why){auto t=evidenceWriter();auto stop=taskLeaseForEvidence(2,10.01);
   if(why==0){stop.allowed=false;assert(t.taskLease(stop,10.011));}
   if(why==1){stop.allowed=false;stop.revoked=true;assert(t.taskLease(stop,10.011));}
   if(why>=2&&why<=5){auto health=h();
    if(why==2)health.owned=false;
    if(why==3)health.connected=false;
    if(why==4)health.mc_fresh=false;
    if(why==5)health.robot_state_fresh=false;
    assert(!evidenceMoving(t.tick(10.011,health)));}
   if(why==6)t.softwareEstopRequested(10.011);
   if(why==7)t.beginShutdown(10.011);
   auto c=localEvidenceCommand();c.valid_until=stamp(10.1);assert(!t.localCommand(c,10.02));
   assert(!evidenceMoving(t.tick(10.021,h())));}
 });
 run("hold_then_new_lease_does_not_resurrect_same_sample",[]{
  auto t=evidenceWriter();auto held=taskLeaseForEvidence(2,10.01);held.allowed=false;
  assert(t.taskLease(held,10.011));assert(t.taskLease(taskLeaseForEvidence(3,10.015),10.016));
  auto c=localEvidenceCommand();c.valid_until=stamp(10.1);assert(!t.localCommand(c,10.02));
  auto next=localEvidenceCommand(2,10.025);next.lease_sequence=3;
  assert(t.localCommand(next,10.026)&&evidenceMoving(t.tick(10.027,h())));
 });
 run("oversized_original_intent_compares_before_writer_saturation",[]{
  auto c=localEvidenceCommand();c.velocity.linear.x=.6;c.velocity.angular.z=1.;auto t=evidenceWriter(c);
  auto out=t.tick(10.005,h());assert(out&&out->linear.x==.3&&out->angular.z==.5);
  auto forged=c;forged.valid_until=stamp(10.09);forged.velocity=*out;
  assert(!t.localCommand(forged,10.02));c.valid_until=stamp(10.09);assert(t.localCommand(c,10.021));
  out=t.tick(10.045,h());assert(out&&out->linear.x==.3&&out->angular.z==.5);
 });
 run("evidence_upgrade_does_not_spend_or_rephase_the_20Hz_slot",[]{
  auto t=evidenceWriter();d1monitor::CommandWriterDeadline schedule(10.,.05);uint64_t revision=1;
  schedule.observe(revision);assert(evidenceMoving(t.tick(10.001,h())));
  schedule.complete(10.001,10.0011,revision,true,10.001);t.writeCalled(10.0011,true,true);
  auto c=localEvidenceCommand();c.valid_until=stamp(10.1);assert(t.localCommand(c,10.026));
  // This is the exact monitor callback policy: evidence changes no sample.
  if(std::string(t.taskCommandAdmission().reason)!="evidence_updated")++revision;
  schedule.observe(revision);assert(revision==1&&!schedule.pending());
  assert(!schedule.normalDue(10.041001));assert(evidenceMoving(t.tick(10.041001,h())));
  schedule.complete(10.041001,10.041002,revision,false);
  auto next=localEvidenceCommand(2,10.05);assert(t.localCommand(next,10.0501));
  ++revision;schedule.observe(revision);assert(schedule.pending());
  assert(!schedule.normalDue(10.0509)&&schedule.normalDue(10.051));
  assert(evidenceMoving(t.tick(10.051,h())));schedule.complete(10.051,10.0511,revision,true,10.051);
  assert(!schedule.pending()&&t.taskCommandExpiry().count==0);
 });
 std::cout<<"local evidence upgrade: "<<cases<<" groups passed\n";
}
int main(){
 // The stationary metric follows explicit runtime-profile reuse, not a raw
 // mode number. SDK state remains truthful: STAIR never reports GENERAL.
 {auto general=h();assert(stationaryRuntimeProfileApplies(general,false));
  auto stair=general;stair.general=false;stair.required_motion_mode="stair";
  stair.motion_mode="stair";stair.motion_mode_confirmed=true;
  assert(stationaryRuntimeProfileApplies(stair,true));
  assert(!stationaryRuntimeProfileApplies(stair,false)); // separate evidence-record profile
  for(int failure=0;failure<8;++failure){auto bad=stair;
   if(failure==0)bad.connected=false;
   if(failure==1)bad.robot_state_fresh=false;
   if(failure==2)bad.mode_transition_active=true;
   if(failure==3)bad.motion_mode_confirmed=false;
   if(failure==4)bad.mode_runtime_parameters_ready=false;
   if(failure==5)bad.motion_mode="unknown";
   if(failure==6)bad.required_motion_mode="";
   if(failure==7){bad.motion_mode="general";bad.general=true;}
   assert(!stationaryRuntimeProfileApplies(bad,true));}
 }
 testLocalEvidenceUpdates();
 // Software-only release is a distinct operator transaction. Healthy
 // telemetry/false ACK alone never revives the old grant, lease or demand.
 {auto t=ready();t.setSDKTransactions(0,false);t.softwareEstopRequested(10.01);
  auto soft=h();soft.estop=soft.software_estop_only=true;t.tick(10.02,soft);
  assert(t.softwareEstopLatched()&&t.state(10.02).fault_latched&&t.stopping());
  assert(t.state(10.02).phase=="software_estop_latched"&&t.state(10.02).reason=="software_estop_requested");
  assert(!t.permit(p(),10.02)&&!t.demand(d(),10.02));
  t.tick(10.03,h());assert(t.softwareEstopLatched());
  assert(t.confirmSoftwareEstopRelease(10.04));t.tick(10.05,h());
  assert(!t.softwareEstopLatched()&&!t.state(10.05).fault_latched&&t.stopping());
  assert(!t.grant(g(),10.05,h()).accepted&&!t.takeControlOnce());
  auto next=g();next.version.task_id="next";next.execution_id="next";next.control_epoch=2;
  next.request_id="new-request";next.confirmation_id="new-confirmation";next.source_stamp=stamp(10.06);
  assert(t.grant(next,10.06,h()).accepted&&t.armGeneration()==2);
  assert(!t.permit(p(),10.07)&&!t.demand(d(),10.07));
 }
 // A session-wide software event remains explicit even when the old task
 // already records another stopping cause; stop/retirement facts stay intact.
 {auto t=ready();t.setSDKTransactions(1,false);t.stop("owner_revoked",10.005);
  t.softwareEstopObserved(10.01);const auto latched=t.state(10.02);
  assert(latched.phase=="software_estop_latched"&&latched.reason=="software_estop_observed");
  assert(latched.fault_latched&&!latched.grant_ready);
  assert(t.report(10.02).reason=="owner_revoked"&&t.retirement(10.02).reason=="owner_revoked");
  assert(t.confirmSoftwareEstopRelease(10.03));t.tick(10.04,h());
  assert(t.state(10.04).phase=="software_estop_latched");
  assert(t.state(10.04).reason=="software_estop_release_waiting_prior_retirement");
  assert(!t.retirement(10.04).software_retired&&!t.report(10.04).measured_stop_confirmed);
 }
 // A hardware/unknown estop classification is not the new recoverable seam.
 {auto t=ready();t.setSDKTransactions(0,false);t.softwareEstopRequested(10.01);
  t.fault("hardware_estop_observed",10.02);
  assert(!t.confirmSoftwareEstopRelease(10.03));t.tick(10.04,h());
  assert(t.state(10.04).fault_latched&&t.softwareEstopLatched());
 }
 // Real reporter -> writer seam: the live clock estimate can lead original
 // host receipt by 60 ms without raw timestamp regression or stale arrivals.
 // This was accepted as 50-Hz telemetry, but rejected by the former writer's
 // zero-future point-time test. Preserve the very same raw/source/receipt.
 {Transport t("sdk","live",true,.3,.5,std::string(64,'c'),"forward_yaw",false);
  t.configureMcCaptureBound(.25,"source_delta_host_anchor_approximate");
  t.configureExecutionPolicy(testExecutionPolicy(),.8);
  d1monitor::McReport reporter;reporter.robot(0.);assert(reporter.request_due(true,false,1.));reporter.ack(true);
  const float zero[3]={0,0,0};uint64_t raw=0;double source=0,received=0;
  for(unsigned i=0;i<=100;++i){const double arrival=1.+i*.02;received=10.+i*.02;raw=1000000000ULL+i*20600000ULL;
    assert(reporter.sample(arrival,received,raw,zero,zero,arrival));source=reporter.stamp_unix;
    t.mc(raw,source,received,0,0,received,"sdk:0");assert(t.mcSourceFresh(received,"sdk:0"));}
  assert(reporter.samples==101&&reporter.timestamp_rejections==0&&reporter.rate_ok(3.));
  assert(std::abs(source-received-.060)<1e-10);
  assert(!fresh(source,received,.25)&&fresh(received,received,.25)); // old point-time verdict
  assert(t.report(received).mc_raw_stamp_ns==raw&&t.report(received).source_stamp==stamp(received));
  assert(t.mcCaptureUpperBound()==received&&std::abs(t.mcCaptureLowerBound()-(received-.25))<1e-10);
  assert(t.mcSourceFresh(received+.249,"sdk:0")&&!t.mcSourceFresh(received+.251,"sdk:0"));
  assert(!t.mcSourceFresh(received,"sdk:1"));
  auto request=g();request.transport_mode="live";request.source_stamp=stamp(received);
  auto healthy=h();healthy.mc_fresh=t.mcSourceFresh(received,"sdk:0");
  const auto granted=t.grant(request,received,healthy);assert(granted.accepted&&!t.report(received).physical_acceptance_verified);
  // Exact duplicates cannot restamp either bound. Excessive positive drift,
  // delayed capture, future receipt and nonfinite values cannot freshen it.
  t.mc(raw,received+.1,received+.1,0,0,received+.1,"sdk:0");
  for(int bad=0;bad<5;++bad){const double at=received+.1;
    const double mapped=bad==0?at+.251:bad==1?at-.251:bad==2?at+.06:
      bad==3?std::numeric_limits<double>::quiet_NaN():at;
    const double receipt=bad==2?at+.001:bad==4?at-.251:at;
    t.mc(raw+20000000ULL,mapped,receipt,0,0,at,"sdk:0");
    assert(t.report(at).mc_raw_stamp_ns==raw&&t.mcCaptureUpperBound()==received);}
  // Actual backwards raw order still fences evidence; accepting future
  // estimates did not erase its floor or manufacture a new clock identity.
  t.mc(raw-1,received+.12,received+.12,0,0,received+.12,"sdk:0");
  assert(!t.mcSourceFresh(received+.12,"sdk:0")&&t.report(received+.12).mc_raw_stamp_ns==raw);
 }
 // A live approximate estimate is not a waiver for an exact mock clock.
 {Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.25,"isolated_simulated_source_clock");
  t.mc(1,10.06,10.,0,0,10.,"mock");assert(!t.mcSourceFresh(10.,"mock"));
  assert(t.report(10.).source_stamp==stamp(0.)&&t.report(10.).mc_raw_stamp_ns==0);}
 // Rejection reasons describe THIS attempted grant. Never mutate the replay
 // ledger or start takeover when a first telemetry sample is unavailable.
  for(int cause=0;cause<6;++cause){Transport t("sdk","isolated_mock");
  t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");t.configureExecutionPolicy(testExecutionPolicy(),.8);
  auto unhealthy=h();const char* expected="";
  if(cause==0){unhealthy.connected=false;expected="writer_disconnected";}
  if(cause==1){unhealthy.robot_state_fresh=false;expected="robot_state_unavailable";}
  if(cause==2){unhealthy.mc_fresh=false;expected="mc_capture_unavailable";}
  if(cause==3){unhealthy.estop=true;expected="robot_estop_active";}
  if(cause==4){unhealthy.general=false;expected="robot_mode_unsupported";}
  if(cause==5){t.fault("test_fault",10.);expected="writer_fault_latched";}
  const auto rejected=t.grant(g(),10.,unhealthy);
  assert(!rejected.accepted&&!rejected.duplicate&&rejected.definitely_not_admitted&&rejected.reason==expected);
  assert(t.executionId().empty()&&t.armGeneration()==0&&!t.takeControlOnce());
  if(cause!=5)assert(t.grant(g(),10.,h()).accepted); // same original request can really retry
 }
 // Forward approximate mapping must not allow buffered pre-zero samples or
 // a shorter capture interval to masquerade as measured post-zero standstill.
 {Transport t("sdk","live",true,.3,.5,std::string(64,'c'),"forward_yaw",false);
  t.configureMcCaptureBound(.25,"source_delta_host_anchor_approximate");t.configureExecutionPolicy(testExecutionPolicy(),.8);
  auto request=g();request.transport_mode="live";assert(t.grant(request,10.,h()).accepted);
  assert(t.report(10.).source_stamp==stamp(0.));t.stop("cancel",10.1);t.stopWritten(10.1);
  for(unsigned i=0;i<10;++i){const double receipt=10.105+i*.02;
    t.mc(1000000000ULL+i*20000000ULL,receipt+.06,receipt,0,0,receipt,"sdk:0");
    assert(t.report(receipt).stationary_samples==0&&!t.report(receipt).measured_stop_confirmed);}
  for(unsigned i=0;i<=20;++i){const double receipt=10.36+i*.05;
    t.mc(1300000000ULL+i*50000000ULL,receipt+.06,receipt,0,0,receipt,"sdk:0");
    assert(!t.report(receipt).measured_stop_confirmed);}
  assert(t.report(11.36).stationary_duration_sec<1.); // raw/receipt span alone is insufficient
  for(unsigned i=21;i<=26;++i){const double receipt=10.36+i*.05;
    t.mc(1300000000ULL+i*50000000ULL,receipt+.06,receipt,0,0,receipt,"sdk:0");}
  const auto report=t.report(11.66);assert(report.measured_stop_confirmed&&report.stationary_duration_sec>=1.);
  assert(report.source_stamp==stamp(11.66)&&!report.physical_acceptance_verified);
  assert(t.mcCaptureUpperBound()==11.66&&std::abs(t.mcCaptureLowerBound()-11.41)<1e-10);
  const auto expired=t.report(11.911);assert(!expired.measured_stop_confirmed&&expired.source_stamp==report.source_stamp);
  t.mc(2650000000ULL,11.77,11.71,.1,0,11.71,"sdk:0");assert(!t.report(11.71).measured_stop_confirmed);
 }
 // The actual live health assembler must not confuse an expired slow
 // RobotState with a fresh observed ownership or mode change. Keep owner
 // heartbeats live throughout this gap, so an absent owner cannot explain it.
 {auto t=ready();auto before=ready();assert(t.demand(d(),10.));const auto initial=t.tick(10.01,h());assert(initial&&initial->linear.x==.2);
  t.moveAttempted(*initial);t.writeCalled(10.01,true);
  d1monitor::NavigationHealth robot;robot.connected=robot.mc_fresh=true;
  robot.software=robot.hardware=robot.mode=robot.speed=robot.head=1;robot.motion=5;robot.robot_at=10.;
  d1monitor::Ownership owner;owner.connected=owner.confirmed=true;owner.control_source=2;owner.last_robot=10.;
  uint64_t sequence=1;
  for(int i=1;i<=61;++i){const double now=10.+i*.05;
    auto heartbeat=p();heartbeat.sequence=++sequence;heartbeat.source_stamp=stamp(now);heartbeat.valid_until=stamp(now+.25);
    if(i>=42){heartbeat.allowed=false;heartbeat.phase="holding";}
    assert(t.permit(heartbeat,now));const auto health=robotExecutionHealth(robot,owner,now,true,2);
    // Exact previous live-node assembly: both mode and owned conflate source
    // expiry with an adverse observation. Same input / clocks / heartbeat.
    auto previous=health;previous.robot_state_fresh=true;
    previous.general=d1monitor::NavigationMotion{}.robot_fresh(robot,now)&&health.general;
    previous.owned=owner.owns(now);before.permit(heartbeat,now);before.tick(now,previous);
    const auto output=t.tick(now,health);
    if(i>=41){assert(!health.robot_state_fresh&&health.owned&&health.general);
      const auto state=t.state(now);
      assert(!t.stopping()&&!state.fault_latched&&!state.grant_ready);
      assert(state.phase=="holding"&&state.reason=="robot_state_stale");
      assert(!state.general_low_speed_confirmed&&state.control_owned&&output&&!planarMotionNonzero(*output));
      assert(t.report(now).nonzero_blocked&&!t.report(now).stop_submitted);
      assert(t.executionId()=="execution"&&t.epoch()==1&&t.armGeneration()==1);
      assert(!t.takeControlOnce());}
  }
  assert(before.stopping()&&before.state(13.05).fault_latched);
  assert(before.state(13.05).reason=="robot_state_or_connection_lost");
  robot.robot_at=13.06;owner.robot(2,13.06);
  const auto recovered=robotExecutionHealth(robot,owner,13.06,true,2);assert(recovered.robot_state_fresh);
  assert(!planarMotionNonzero(*t.tick(13.06,recovered))&&!t.stopping());
  assert(t.state(13.06).grant_ready&&!t.state(13.06).fault_latched);
  // Even still-live pre-gap source messages cannot restart motion after the
  // telemetry barrier. A later proof cannot re-date their body or permission.
  auto old=p();old.sequence=sequence+1;old.source_stamp=stamp(13.04);old.valid_until=stamp(13.25);
  assert(!t.permit(old,13.07));
  auto queued=d();queued.sequence=2;queued.permit_sequence=old.sequence;
  queued.source_stamp=queued.body_source_stamp=stamp(13.04);queued.valid_until=stamp(13.14);
  queued.motion_validation_sequence=2;assert(t.motionValidation(proof(queued,13.07,2),13.07));
  assert(!t.demand(queued,13.07));assert(!planarMotionNonzero(*t.tick(13.07,recovered)));
  auto current=p();current.sequence=sequence+1;current.source_stamp=stamp(13.08);current.valid_until=stamp(13.30);
  assert(t.permit(current,13.08));auto command=d();command.sequence=2;command.permit_sequence=current.sequence;
  command.source_stamp=command.body_source_stamp=stamp(13.08);command.valid_until=stamp(13.18);command.motion_validation_sequence=3;
  refreshBody(t,13.08);assert(t.motionValidation(proof(command,13.08,3),13.08));assert(t.demand(command,13.08));
  assert(t.tick(13.09,recovered)->linear.x==.2&&t.state(13.09).writer_commit_sequence==1);
  assert(t.executionId()=="execution"&&t.armGeneration()==1&&!t.stopping());
  before.tick(13.06,recovered);assert(before.stopping()&&!before.state(13.06).grant_ready);
 }
 // A stale initial RobotState cannot obtain a grant, including the interval
 // where legacy owns() already expired but the explicit owner fact remains.
 {Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
  t.configureExecutionPolicy(testExecutionPolicy(),.8);t.configureBrakingModel(std::string(64,'c'));
  auto stale=h();stale.robot_state_fresh=false;
  assert(!t.grant(g(),10.,stale).accepted&&t.executionId().empty());
  assert(t.grant(g(),10.,h()).accepted);
 }
 // Every explicit adverse fact is still irreversible, even during a known
 // telemetry gap. No later healthy callback rearms or replays old commands.
 for(int kind=0;kind<4;++kind){auto t=ready();auto gap=h();gap.robot_state_fresh=false;
  assert(!planarMotionNonzero(*t.tick(10.05,gap))&&!t.stopping());
  auto lost=gap;
  if(kind==0)lost.connected=false;
  if(kind==1)lost.owned=false;
  if(kind==2)lost.estop=true;
  if(kind==3)lost.general=false;
  t.tick(10.1,lost);assert(t.stopping()&&t.state(10.1).fault_latched);
  t.tick(10.15,h());assert(!t.state(10.15).grant_ready&&!t.demand(d(),10.15));
  assert(!t.takeControlOnce());
 }
 // Ownership consumes ordered original RobotState receipts. A queued older
 // APP-owned packet cannot overwrite a newer confirmed SDK ownership fact.
 {d1monitor::NavigationHealth robot;robot.connected=robot.mc_fresh=true;
  robot.software=robot.hardware=robot.mode=robot.speed=robot.head=1;robot.motion=5;robot.robot_at=10.;
  d1monitor::Ownership owner;owner.connected=owner.confirmed=true;owner.control_source=2;owner.last_robot=10.;
  owner.robot(1,9.99);assert(robotExecutionHealth(robot,owner,10.01,true,2).owned);
  const auto stale=robotExecutionHealth(robot,owner,13.,true,2);
  assert(stale.owned&&!stale.robot_state_fresh&&!owner.owns(13.));
  owner.robot(1,13.01);robot.robot_at=13.01;
  const auto lost=robotExecutionHealth(robot,owner,13.01,true,1);
  assert(lost.robot_state_fresh&&!lost.owned);
 }
 // Production raw RobotState and ownership-event processing have separate
 // queues. A fresh APP-owned raw state must veto a still-confirmed old SDK
 // owner before its matching ownership event drains. Source expiry alone is
 // already covered above and must not be confused with this adverse fact.
 {d1monitor::NavigationHealth robot;robot.connected=robot.mc_fresh=true;
  robot.software=robot.hardware=robot.mode=robot.speed=robot.head=1;robot.motion=5;robot.robot_at=10.02;
  d1monitor::Ownership owner;owner.connected=owner.confirmed=true;owner.control_source=2;owner.last_robot=10.;
  for(int observed:{0,1,3}){auto t=ready();auto before=ready();
    assert(t.demand(d(),10.)&&before.demand(d(),10.));
    const auto guarded=robotExecutionHealth(robot,owner,10.03,true,observed);
    assert(guarded.robot_state_fresh&&!guarded.owned&&owner.confirmed&&owner.control_source==2);
    // The former factory only used the old explicit ownership object.
    auto unguarded=guarded;unguarded.owned=true;
    assert(planarMotionNonzero(*before.tick(10.03,unguarded))&&!before.stopping());
    t.tick(10.03,guarded);assert(t.stopping()&&t.state(10.03).fault_latched);
    t.tick(10.04,h());assert(t.stopping()&&!t.state(10.04).grant_ready);
  }
 }
 // A source-measured suffix can bootstrap the initial writer without a
 // second TrackingProgress message. Only the exact native actual-command
 // verdict binds the curve evidence; no timestamps/leases are renewed here.
 {auto t=ready();auto command=d();command.sequence=command.motion_validation_sequence=2;
  command.validation_sequence=2;auto c=suffix();assert(t.trajectoryValidation(c,10.));
  auto native=proof(command,10.,2);assert(t.motionValidation(native,10.));
  assert(t.demand(command,10.));const auto output=t.tick(10.01,h());
  assert(output&&planarMotionNonzero(*output));t.writeCalled(10.01,true);
  const auto ack=t.takeCommitAck();assert(ack&&ack->applied&&ack->commit_sequence==1);
  assert(ack->motion_validation_sequence==native.sequence&&ack->valid_until==c.valid_until);
 }
 // A later same-curve positive does not substitute its longer expiry for
 // the immutable certificate named by the exact command verdict.
 {auto t=ready();auto command=d();command.sequence=command.motion_validation_sequence=2;
  command.validation_sequence=2;auto c=suffix();assert(t.trajectoryValidation(c,10.));
  auto newer=suffix(3);newer.valid_until=stamp(10.25);assert(t.trajectoryValidation(newer,10.));
  auto native=proof(command,10.,2);assert(t.motionValidation(native,10.)&&t.demand(command,10.));
  assert(planarMotionNonzero(*t.tick(10.01,h())));
  assert(!planarMotionNonzero(*t.tick(10.081,h())));
  assert(t.state(10.081).writer_commit_sequence==0);
 }
 // Missing/negative/foreign/expired native evidence and missing or invalid
 // named curve evidence all remain fail closed before the first SDK call.
 for(int fault=0;fault<9;++fault){auto t=ready();auto command=d();
  command.sequence=command.motion_validation_sequence=2;command.validation_sequence=2;
  auto c=suffix();if(fault==4)c.valid_until=stamp(10.005);
  if(fault==6)c.valid=false;
  if(fault==7)c.reverse_margin_m=.149;
  if(fault==8)c.checked_from_time=.201;
  assert(t.trajectoryValidation(c,10.));auto native=proof(command,10.,2);
  if(fault==1)native.valid=false;
  if(fault==2)native.version.context_sequence=2;
  if(fault==3)native.valid_until=stamp(10.005);
  if(fault==5)native.trajectory_validation_sequence=3;
  if(fault!=0)t.motionValidation(native,10.);
  t.demand(command,10.);const auto output=t.tick(10.01,h());
  assert(output&&!planarMotionNonzero(*output));
  assert(t.state(10.01).writer_commit_sequence==0);
 }
 // A bare zero cannot borrow suffix geometry as first-write authorization.
 {auto t=ready();auto c=suffix();assert(t.trajectoryValidation(c,10.));
  auto zero=d();zero.sequence=2;zero.validation_sequence=2;zero.motion_validation_sequence=0;
  zero.velocity=geometry_msgs::msg::Twist{};assert(t.demand(zero,10.));
  assert(!planarMotionNonzero(*t.tick(10.01,h())));t.writeCalled(10.01,true);
  assert(!t.takeCommitAck()&&t.state(10.01).writer_commit_sequence==0);
 }
 // Consume the same provider-certified state as the tracker. Transport does
 // not reapply a raw IMU age limit after delivery, nor extend source/expiry.
 {auto t=ready();Local m;m.schema_version=2;m.session_id="nav";m.map_version_id="map";
  m.localization_epoch=1;m.localization_seed_id="seed";m.usable=true;
  m.source_stamp=stamp(10.02);m.posterior_stamp=stamp(9.8);m.imu_stamp=stamp(9.9254);
  m.extrapolation_sec=.0946;m.valid_until=stamp(10.12);
  m.local_odometry.header.stamp=m.source_stamp;m.local_odometry.header.frame_id="d1max_loc_odom";
  m.local_odometry.child_frame_id="d1max_loc_base_link";m.local_odometry.pose.pose.orientation.w=1.;
  assert(t.localState(m,10.0459)); // raw IMU age 120.5 ms; provider lease alive.
  assert(!t.localState(m,10.06)); // repeated delivery never renews evidence.
  auto bad=m;bad.source_stamp=bad.local_odometry.header.stamp=stamp(10.04);
  bad.valid_until=stamp(10.140000001);assert(!t.localState(bad,10.05));
  assert(t.demand(d(),10.05));assert(planarMotionNonzero(*t.tick(10.05,h())));
  assert(!planarMotionNonzero(*t.tick(10.120001,h()))); // actual provider lease exhausted.
 }
 // The write ledger is global across grants and bounded even if the vendor
 // never calls back. Saturation rejects BEFORE any SDK send can take place.
 {d1monitor::SDKWriteTransactions ledger;
  std::array<std::shared_ptr<d1monitor::SDKWriteTransactions::Token>,64> calls;
  for(auto&call:calls){call=ledger.begin();assert(call);}
  assert(ledger.pending()==64&&!ledger.begin());
  assert(calls.front()->complete()&&ledger.pending()==63);
  assert(!calls.front()->complete()&&ledger.pending()==63);
  auto next=ledger.begin();assert(next&&ledger.pending()==64&&!ledger.begin());
  for(auto&call:calls)call->complete();
  assert(ledger.pending()==1);
  assert(next->complete()&&ledger.pending()==0);
  auto abandoned=ledger.begin();assert(abandoned);abandoned.reset();
  assert(ledger.pending()==1); // token destruction cannot certify a lost call
 }
 // Concurrent registrations share the same fixed allowance; there is no
 // check-then-increment race that can admit a sixty-fifth outstanding call.
 {d1monitor::SDKWriteTransactions ledger;
  std::array<std::array<std::shared_ptr<d1monitor::SDKWriteTransactions::Token>,16>,8> calls;
  std::array<std::thread,8> workers;
  for(size_t i=0;i<workers.size();++i)workers[i]=std::thread([&,i]{
    for(auto&call:calls[i])call=ledger.begin();});
  for(auto&worker:workers)worker.join();
  size_t admitted=0;for(const auto&row:calls)for(const auto&call:row)admitted+=bool(call);
  assert(admitted==64&&ledger.pending()==64&&!ledger.begin());
  for(auto&row:calls)for(auto&call:row)if(call)assert(call->complete());
  assert(ledger.pending()==0);
 }
 // An inline callback and an immediate error compete for ONE outcome. A
 // delayed duplicate cannot mutate the ACK/error state of a newer grant.
 {d1monitor::SDKWriteTransactions ledger;auto call=ledger.begin();int outcome=0;
  assert(call->finish([&]{outcome=1;}));
  assert(!call->finish([&]{outcome=2;})&&outcome==1&&ledger.pending()==0);
  auto failed=ledger.begin();assert(failed->complete());
  assert(!failed->finish([&]{outcome=3;})&&outcome==1&&ledger.pending()==0);
 }
 // Claiming finalization does not release the slot before its ACK/error
 // mutation finishes. This deterministic barrier covers duplicate callbacks
 // arriving DURING finalization, without relying on sleeps or the SDK.
 {d1monitor::SDKWriteTransactions ledger;auto call=ledger.begin();
  std::mutex mutex;std::condition_variable cv;bool entered=false,release=false;int outcome=0;
  std::thread callback([&]{assert(call->finish([&]{
    std::unique_lock<std::mutex>lock(mutex);entered=true;cv.notify_one();
    cv.wait(lock,[&]{return release;});outcome=1;}));});
  {std::unique_lock<std::mutex>lock(mutex);cv.wait(lock,[&]{return entered;});}
  assert(ledger.pending()==1&&!call->finish([&]{outcome=99;}));
  {std::lock_guard<std::mutex>lock(mutex);release=true;}cv.notify_one();
  while(ledger.pending()!=0)std::this_thread::yield();
  assert(outcome==1);callback.join();
 }
 // The last acquire observation publishes every callback's effect, not just
 // the last callback. Shared tokens also remain valid after owner destruction.
 {d1monitor::SDKWriteTransactions ledger;std::array<int,8> outcomes{};
  std::array<std::thread,8> workers;std::array<std::shared_ptr<d1monitor::SDKWriteTransactions::Token>,8> calls;
  for(auto&call:calls){call=ledger.begin();assert(call);}
  for(size_t i=0;i<workers.size();++i)workers[i]=std::thread([&,i]{assert(calls[i]->finish([&,i]{outcomes[i]=int(i)+1;}));});
  while(ledger.pending()!=0)std::this_thread::yield();
  for(size_t i=0;i<outcomes.size();++i)assert(outcomes[i]==int(i)+1);
  for(auto&worker:workers)worker.join();
  std::shared_ptr<d1monitor::SDKWriteTransactions::Token> late;
  {d1monitor::SDKWriteTransactions temporary;late=temporary.begin();assert(late);}
  assert(late->complete()&&!late->complete());
 }
 // No-motion retirement is a separate typed software fact, NEVER an MC
 // standstill assertion. It requires tracked and fully settled SDK work.
 {auto t=ready();t.stop("cancel_before_motion",10.01);
  assert(t.motionNeverSubmitted()&&!t.noMotionRetired());
  assert(!t.retirement(10.01).software_retired);t.setSDKTransactions(0,false);
  const auto retired=t.retirement(10.02);
  assert(retired.schema_version==1&&retired.version==v()&&retired.execution_id=="execution");
  assert(retired.control_epoch==1&&retired.sdk_session=="sdk"&&retired.sdk_arm_generation==1);
  assert(retired.grant_request_id=="request"&&retired.nonzero_blocked&&retired.software_retired);
  assert(retired.motion_never_submitted&&retired.pending_sdk_transactions==0);
  assert(retired.reason=="cancel_before_motion"&&retired.source_stamp==stamp(10.02));
  assert(!t.report(10.02).stop_submitted&&!t.report(10.02).measured_stop_confirmed);
  assert(!t.tick(10.03,h())); // no zero/TakeControl work is revived after retirement
 }
 for(const auto&outstanding:{std::pair<uint32_t,bool>{1,false},{0,true},{64,true}}){
  auto t=ready();t.setSDKTransactions(outstanding.first,outstanding.second);t.stop("cancel",10.01);
  auto retired=t.retirement(10.02);assert(retired.motion_never_submitted&&!retired.software_retired);
  assert(retired.pending_sdk_transactions==outstanding.first&&!t.noMotionRetired());
  t.setSDKTransactions(0,false);assert(t.retirement(10.03).software_retired);
 }
 // An attempted nonzero call with an immediate error has an unknown physical
 // outcome. Empty pending work must never turn it into "never moved".
 {auto t=ready();t.moveAttempted(d().velocity);t.setSDKTransactions(0,false);t.submissionFailed(10.01);
  assert(!t.motionNeverSubmitted()&&!t.noMotionRetired());
  const auto retired=t.retirement(10.02);assert(!retired.motion_never_submitted&&retired.software_retired);
  assert(!t.report(10.02).measured_stop_confirmed&&t.state(10.02).fault_latched);
 }
 {auto t=ready();geometry_msgs::msg::Twist zero;t.moveAttempted(zero);t.setSDKTransactions(0,false);
  t.stop("cancel",10.01);assert(t.motionNeverSubmitted()&&t.noMotionRetired());
 }
 {auto t=ready();assert(t.demand(d(),10.));assert(t.tick(10.01,h())->linear.x==.2);
  assert(t.motionNeverSubmitted()); // selected, but never passed to the SDK
  t.stop("cancel_before_call",10.02);t.setSDKTransactions(0,false);assert(t.noMotionRetired());
 }
 // After actual motion, one acknowledged zero finishes the software write
 // obligation. Do not manufacture a new async zero every tick and keep the
 // ledger permanently pending; MC standstill remains independently unproven.
 {auto t=ready();assert(t.demand(d(),10.));const auto moving=t.tick(10.01,h());assert(moving);
  t.moveAttempted(*moving);t.writeCalled(10.01,true);t.stop("cancel",10.02);
  const auto zero=t.tick(10.03,h());assert(zero&&!planarMotionNonzero(*zero));
  d1monitor::SDKWriteTransactions ledger;auto call=ledger.begin();assert(call);
  t.setSDKTransactions(ledger.pending(),false);t.moveAttempted(*zero);t.writeCalled(10.03,true);
  assert(!t.retirement(10.03).software_retired);
  assert(call->finish([&]{t.stopWritten(10.04);assert(ledger.pending()==1);}));
  t.setSDKTransactions(ledger.pending(),false);assert(!t.tick(10.05,h()));
  const auto retired=t.retirement(10.05);assert(retired.software_retired&&!retired.motion_never_submitted);
  assert(retired.pending_sdk_transactions==0&&t.report(10.05).stop_submitted);
  assert(!t.report(10.05).measured_stop_confirmed);
 }
 // A new BT process legitimately starts at epoch one. SDK generation stays
 // monotonic, while retired navigation sessions and prior request IDs cannot
 // be resurrected even after another never-moved task exits.
 {auto t=ready();t.setSDKTransactions(0,false);t.stop("cancel",10.01);
  auto next=g();next.version.session_id="nav-next";next.execution_id="execution-next";
  next.request_id="request-next";next.source_stamp=stamp(10.02);
  auto accepted=t.grant(next,10.02,h());assert(accepted.accepted&&accepted.sdk_arm_generation==2);
  assert(t.epoch()==1&&t.binding().session_id=="nav-next");
  t.stop("cancel",10.03);
  auto old=g();old.execution_id="execution-old-revival";old.request_id="request-old-revival";
  old.source_stamp=stamp(10.04);old.control_epoch=2;
  assert(!t.grant(old,10.04,h()).accepted&&t.grant(old,10.04,h()).reason=="grant_replay");
  assert(t.binding().session_id=="nav-next"&&t.armGeneration()==2);
 }
 {auto t=ready();d1monitor::SDKWriteTransactions ledger;auto old=ledger.begin();assert(old);
  t.stop("cancel",10.01);t.setSDKTransactions(ledger.pending(),false);
  auto next=g();next.version.session_id="nav-next";next.execution_id="execution-next";
  next.request_id="request-next";next.source_stamp=stamp(10.02);
  assert(!t.grant(next,10.02,h()).accepted&&t.binding().session_id=="nav");
  assert(old->complete());t.setSDKTransactions(ledger.pending(),false);
  assert(t.grant(next,10.02,h()).accepted&&t.armGeneration()==2);
  // A duplicate old completion cannot decrement current-generation work.
  auto current=ledger.begin();assert(current&&!old->complete()&&ledger.pending()==1);
  assert(current->complete()&&ledger.pending()==0);
 }
 // A refusal proves only the NEW request was never installed; an unresolved
 // incumbent still owns its own retirement and physical-stop obligations.
 {auto t=ready();t.moveAttempted(d().velocity);t.stop("cancel",10.01);t.setSDKTransactions(0,false);
  auto next=g();next.version.session_id="nav-next";next.execution_id="execution-next";
  next.request_id="request-next";next.source_stamp=stamp(10.02);
  const auto rejected=t.grant(next,10.02,h());
  assert(!rejected.accepted&&rejected.definitely_not_admitted&&rejected.reason=="prior_stop_unconfirmed");
  const auto prior=t.retirement(10.02);
  assert(t.executionId()=="execution"&&prior.software_retired&&!prior.motion_never_submitted);
  assert(!t.report(10.02).measured_stop_confirmed);
  assert(!t.rejectGrant(g(),"failed").definitely_not_admitted);
  auto same=g();same.request_id="new-name-same-binding";
  assert(!t.rejectGrant(same,"failed").definitely_not_admitted);
 }
 // APP availability cannot rearm a rejected one-shot TakeControl. Only a
 // NEW explicit authorization can, and pending/ambiguous outcomes cannot.
 {d1monitor::Ownership owner;owner.connection(true,false);owner.enabled=true;
  owner.robot(1,10.);owner.available(10.);assert(owner.due(10.5));
  const auto first=owner.request_id;owner.ack(1,"rejected",10.6);
  owner.available(10.7);assert(!owner.due(11.3)&&owner.total_requests==1);
  assert(owner.beginAuthorizedAttempt(11.3));owner.robot(1,11.4);
  assert(owner.due(11.8)&&owner.request_id>first&&owner.total_requests==2);
  assert(!owner.beginAuthorizedAttempt(11.9)); // still awaiting vendor ACK
  owner.tick(14.9);assert(owner.uncertain&&!owner.pending);
  assert(!owner.beginAuthorizedAttempt(15.)&&!owner.due(15.)&&owner.total_requests==2);
 }
 {d1monitor::Ownership owner;assert(!owner.beginAuthorizedAttempt(10.));
  owner.connection(true,true);assert(!owner.beginAuthorizedAttempt(10.));
  owner.connection(false,false);owner.connection(true,false);owner.enabled=true;
  owner.robot(1,10.);owner.available(10.);assert(owner.due(10.5));owner.ack(0,"",10.6);
  assert(!owner.beginAuthorizedAttempt(10.7));owner.robot(2,10.7);owner.robot(2,10.8);
  const auto requests=owner.total_requests;assert(owner.beginAuthorizedAttempt(10.9));
  assert(owner.owns(10.9)&&!owner.due(11.4)&&owner.total_requests==requests);
 }
 // Original command acquisition has one record-bound pipeline budget.
 // Latest actual body/IMU and the exact native swept proof still have their
 // own shorter deadlines. No callback grants another source lease.
 for(int fault=0;fault<6;++fault){auto policy=testExecutionPolicy();policy.command_pipeline_bound_s=.2;
  auto t=ready(policy);auto command=d();command.motion_validation_sequence=2;
  refreshBody(t,10.065);auto native=proof(command,10.065,2);native.valid_until=stamp(10.165);
  if(fault==1)native.valid_until=stamp(10.124);
  if(fault==2)native.valid=false;
  if(fault==3)command.body_source_stamp=stamp(9.924);
  if(fault==4){auto revoke=p();revoke.sequence=2;revoke.revoked=true;assert(t.permit(revoke,10.125));}
  if(fault==5)native.braking_model_sha256=std::string(64,'d');
  t.motionValidation(native,10.125);const bool accepted=t.demand(command,10.125);
  assert(accepted==(fault==0));const auto output=t.tick(10.125,h());assert(output);
  if(fault==0){assert(output->linear.x==.2);t.writeCalled(10.125,true);const auto ack=t.takeCommitAck();
   assert(ack&&ack->demand_source_stamp==command.source_stamp&&ack->demand_body_source_stamp==command.body_source_stamp);
   assert(ack->valid_until==native.valid_until&&ack->body_source_stamp==stamp(10.065));
   assert(!planarMotionNonzero(*t.tick(10.166,h())));assert(!t.demand(command,10.201));}
  else assert(!planarMotionNonzero(*output));
 }
 {auto policy=testExecutionPolicy();policy.command_pipeline_bound_s=.2;auto t=ready(policy);
  auto command=d();command.motion_validation_sequence=2;auto native=proof(command,10.12,2);
  assert(t.motionValidation(native,10.125)&&t.demand(command,10.125));
  assert(!planarMotionNonzero(*t.tick(10.125,h()))); // actual Local remained at 10.0, stale by .125.
 }
 // The configured runtime model can grant, but never reports physical
 // acceptance. A real new post-zero MC window is still required to stop.
 for(bool physical:{false,true}) {
   Transport t("sdk","live",true,.3,.5,std::string(64,'c'),"holonomic_xyyaw",physical);
   t.configureMcCaptureBound(.02,"source_delta_host_anchor_approximate");
   t.configureExecutionPolicy(testExecutionPolicy(),.8);
   auto request=g();request.transport_mode="live";assert(t.grant(request,10.,h()).accepted);
   t.tick(10.,h());t.stop("cancel",10.1);t.stopWritten(10.1);
   assert(!t.report(10.1).measured_stop_confirmed);
   t.mc(10090000000ULL,10.09,10.15,0.,0.,10.15,"runtime-clock");
   assert(!t.report(10.15).measured_stop_confirmed); // pre-stop capture is not evidence
   for(int i=0;i<=24;++i){const double now=10.2+i*.05;
     t.mc(10200000000ULL+i*50000000ULL,now,now,0.,0.,now,"runtime-clock");}
   const auto stopped=t.report(11.401);
   assert(stopped.measured_stop_confirmed&&stopped.physical_acceptance_verified==physical);
   assert(!t.report(11.7).measured_stop_confirmed); // no source/receipt TTL relaxation
 }
 {Transport legacy("sdk","live",true);assert(legacy.report(10.).physical_acceptance_verified);}
 {Transport invalid("sdk","live",false,.3,.5,{},"forward_yaw",true);
   assert(!invalid.report(10.).physical_acceptance_verified);
   auto request=g();request.transport_mode="live";
   assert(invalid.grant(request,10.,h()).reason=="runtime_motion_profile_unavailable");}
 // Signed XY is opt-in. A legacy record still rejects sideways/reverse;
 // planar norm, all finite axes and non-planar rejection are common bounds.
 for(const auto&xy:{std::pair<double,double>{0.,.2},{-.2,0.}}) {
   auto legacy=ready();auto command=d();command.velocity.linear.x=xy.first;command.velocity.linear.y=xy.second;
   assert(!legacy.demand(command,10.));
   auto omni=ready(testExecutionPolicy(),"holonomic_xyyaw");
   command.sequence=command.motion_validation_sequence=2;
   assert(omni.motionValidation(proof(command,10.,2),10.));assert(omni.demand(command,10.));
   const auto output=omni.tick(10.01,h());assert(output&&output->linear.x==xy.first&&output->linear.y==xy.second);
   omni.writeCalled(10.01,true);const auto ack=omni.takeCommitAck();
   assert(ack&&ack->applied&&ack->applied_velocity==command.velocity&&ack->motion_validation_sequence==2);
   auto upgraded=command;upgraded.motion_validation_sequence=3;upgraded.safety_source_stamp=stamp(10.02);
   assert(omni.motionValidation(proof(upgraded,10.02,3),10.02)&&omni.demand(upgraded,10.02));
   assert(omni.tick(10.03,h())->linear.y==xy.second);
   assert(!planarMotionNonzero(*omni.tick(10.101,h()))); // Original source TTL, no lateral exemption.
 }
 for(int bad_axis=0;bad_axis<6;++bad_axis) {
   auto t=ready(testExecutionPolicy(),"holonomic_xyyaw");auto command=d();command.sequence=2;
   switch(bad_axis){case 0:command.velocity.linear.x=.25;command.velocity.linear.y=.25;break;
     case 1:command.velocity.linear.y=std::numeric_limits<double>::quiet_NaN();break;
     case 2:command.velocity.linear.z=.01;break;case 3:command.velocity.angular.x=.01;break;
     case 4:command.velocity.angular.y=.01;break;case 5:command.velocity.angular.z=.501;break;}
   assert(!t.demand(command,10.));
 }
 for(bool hold:{false,true}) {
   auto t=ready(testExecutionPolicy(),"holonomic_xyyaw");auto command=d();command.sequence=command.motion_validation_sequence=2;
   command.velocity.linear.x=0.;command.velocity.linear.y=.2;command.hold=hold;command.safety_checked=hold;
   assert(!t.demand(command,10.)); // Neither HOLD nor unchecked lateral motion is a zero.
 }
 {auto t=ready(testExecutionPolicy(),"holonomic_xyyaw");auto command=d();command.sequence=command.motion_validation_sequence=2;
   command.velocity.linear.x=0.;command.velocity.linear.y=.2;
   assert(!t.demand(command,10.));assert(!planarMotionNonzero(*t.tick(10.01,h())));
   assert(t.motionValidation(proof(command,10.02,2),10.02));assert(t.tick(10.03,h())->linear.y==.2);
   assert(!planarMotionNonzero(*t.tick(10.101,h())));
 }
 {auto t=ready(testExecutionPolicy(),"holonomic_xyyaw");auto command=d();command.sequence=command.motion_validation_sequence=2;
   command.velocity.linear.x=0.;command.velocity.linear.y=.2;
   assert(t.motionValidation(proof(command,10.,2),10.)&&t.demand(command,10.));
   assert(t.tick(10.01,h())->linear.y==.2);t.writeCalled(10.01,true);
   assert(t.beginShutdown(10.02));assert(!planarMotionNonzero(*t.tick(10.03,h())));
   assert(!t.demand(command,10.03));assert(!t.report(10.03).measured_stop_confirmed);
 }
 {auto t=ready(testExecutionPolicy(),"holonomic_xyyaw");auto command=d();command.sequence=command.motion_validation_sequence=2;
   command.velocity.linear.x=0.;command.velocity.linear.y=.2;
   assert(t.motionValidation(proof(command,10.,2),10.)&&t.demand(command,10.));
   assert(t.tick(10.01,h())->linear.y==.2);const auto lateral_write=t.nextWriteSequence();t.writeCalled(10.01,true);
   t.mc(10020000000ULL,10.02,10.02,0.,0.,10.02,"mc-clock");
   auto hold=p();hold.sequence=2;hold.source_stamp=stamp(10.02);hold.allowed=false;hold.phase="holding";
   assert(t.permit(hold,10.02));assert(!planarMotionNonzero(*t.tick(10.02,h())));
   const auto zero_write=t.nextWriteSequence();t.writeCalled(10.02,true);
   // A lateral write must fence old/mislabelled zero ACKs just like forward.
   t.zeroAcknowledged(lateral_write,10.01,10.02);assert(t.stationaryEvidence(10.02).zero_write_sequence==0);
   t.zeroAcknowledged(zero_write,10.02,10.02);assert(t.stationaryEvidence(10.02).zero_write_sequence==zero_write);
 }
 try {Transport invalid("sdk","isolated_mock",false,.3,.5,{},"invalid");assert(false);}
 catch(const std::invalid_argument&){}
 // SCAN owns the physical source-age policy. The writer binds its unchanged
 // effective deadline and does not veto the same verdict by metadata ages.
 for(bool expired:{false,true}) {auto policy=testExecutionPolicy();policy.sensor_source_age_bound_s=.2;auto t=ready(policy);
  auto command=d();command.sequence=command.motion_validation_sequence=2;command.safety_source_stamp=stamp(9.7);
  auto verdict=proof(command,10.,2);verdict.body_source_stamp=verdict.front_ray_source_stamp=verdict.rear_ray_source_stamp=stamp(9.7);
  verdict.valid_until=stamp(expired?9.99:10.04);
  assert(t.motionValidation(verdict,10.));
  assert(t.demand(command,10.)==!expired);
  assert(t.tick(10.,h())->linear.x==(expired?0.:.2));
  assert(t.tick(10.041,h())->linear.x==0.);} // Never extend SCAN's original deadline.
 // Production admission core + a deliberately blocked diagnostic consumer.
 // The writer still expires the ORIGINAL proof and processes revoke; no ROS
 // executor, SDK connection, fabricated heartbeat or sleep-based timing.
 {auto t=ready();assert(t.demand(d(),10.));assert(t.tick(10.01,h())->linear.x==.2);
  std::mutex diagnostic_mutex;std::condition_variable wake;bool release=false;std::atomic<bool>blocked{false};
  std::thread diagnostic([&]{std::unique_lock<std::mutex>lock(diagnostic_mutex);blocked=true;
    wake.wait(lock,[&]{return release;});});
  while(!blocked.load())std::this_thread::yield();
  assert(t.tick(10.06,h())->linear.x==.2);
  const auto expired=t.tick(10.12,h());assert(expired&&expired->linear.x==0.);
  auto revoked=p();revoked.sequence=2;revoked.revoked=true;revoked.allowed=false;revoked.phase="stopping";
  revoked.source_stamp=stamp(10.13);revoked.valid_until=stamp(10.38);assert(t.permit(revoked,10.13));
  assert(t.tick(10.13,h())->linear.x==0.&&t.state(10.13).phase=="stopping");
  auto late=d();late.sequence=2;late.source_stamp=late.body_source_stamp=stamp(10.13);late.valid_until=stamp(10.23);
  assert(!t.demand(late,10.13));assert(!t.report(10.13).measured_stop_confirmed);
  {std::lock_guard<std::mutex>lock(diagnostic_mutex);release=true;}wake.notify_one();diagnostic.join();}
 // Native curve renewal can produce a fresher sweep of the exact original
 // command. Upgrade only its proof binding, not its control or source lease.
 {auto t=ready();auto original=d();auto early=proof(original,10.,2);
  original.motion_validation_sequence=2;early.valid_until=stamp(10.04);
  assert(t.motionValidation(early,10.));assert(t.demand(original,10.));
  auto upgraded=original;upgraded.motion_validation_sequence=3;
  upgraded.safety_source_stamp=stamp(10.03);auto renewed=proof(upgraded,10.03,3);
  renewed.valid_until=stamp(10.095);assert(t.motionValidation(renewed,10.03));
  assert(t.demand(upgraded,10.035));assert(t.tick(10.05,h())->linear.x==.2);
  assert(!t.demand(upgraded,10.06)); // an exact duplicate remains replay
  assert(t.tick(10.096,h())->linear.x==0.); // new proof's real deadline
 }
 {auto t=ready();assert(t.demand(d(),10.));auto upgraded=d();upgraded.motion_validation_sequence=2;
  assert(t.motionValidation(proof(upgraded,10.05,2),10.05));assert(t.demand(upgraded,10.05));
  assert(t.tick(10.09,h())->linear.x==.2);assert(t.tick(10.101,h())->linear.x==0.);
 } // even an otherwise live new proof cannot extend original 100ms source
 for(int changed=0;changed<8;++changed){auto t=ready();assert(t.demand(d(),10.));
  auto altered=d();altered.motion_validation_sequence=2;
  switch(changed){case 0:altered.source_stamp=stamp(10.01);break;
   case 1:altered.body_source_stamp=stamp(10.01);break;case 2:altered.valid_until=stamp(10.21);break;
   case 3:altered.velocity.linear.x=.21;break;case 4:++altered.permit_sequence;break;
   case 5:++altered.validation_sequence;break;case 6:altered.hold=true;break;
   case 7:altered.safety_source_stamp=stamp(9.99);break;}
  t.motionValidation(proof(altered,10.02,2),10.02);assert(!t.demand(altered,10.02));
 }
 {auto t=ready();assert(t.demand(d(),10.));auto invalid=proof(d(),10.02,2);invalid.valid=false;
  assert(t.motionValidation(invalid,10.02));auto replay=d();replay.motion_validation_sequence=3;
  assert(t.motionValidation(proof(replay,10.03,3),10.03));assert(!t.demand(replay,10.03));
  assert(t.tick(10.04,h())->linear.x==0.);
 }
 // Invalid arriving before any acceptance cannot be bypassed by explicitly
 // binding a still later positive result for that same original command.
 {auto t=ready();auto original=d();original.sequence=2;original.motion_validation_sequence=2;
  auto invalid=proof(original,10.01,2);invalid.valid=false;assert(t.motionValidation(invalid,10.01));
  original.motion_validation_sequence=3;assert(t.motionValidation(proof(original,10.02,3),10.02));
  assert(!t.demand(original,10.03));assert(t.tick(10.04,h())->linear.x==0.);
 }
 for(int barrier=0;barrier<3;++barrier){auto t=ready();assert(t.demand(d(),10.));
  auto next=p();next.sequence=2;next.source_stamp=stamp(10.02);
  if(barrier==0)next.allowed=false;else if(barrier==1)next.revoked=true;else next.phase="aligning";
  assert(t.permit(next,10.02));auto replay=d();replay.motion_validation_sequence=2;
  t.motionValidation(proof(replay,10.03,2),10.03);assert(!t.demand(replay,10.03));
  assert(t.tick(10.04,h())->linear.x==0.);
 }
 // Two independent positive checks of one command can arrive around a writer
 // tick. Preserve the exact still-live verdict signed into SafeDemand; never
 // implicitly bind the later verdict, borrow its deadline, or drop to zero.
 {auto t=ready();assert(t.demand(d(),10.));
  auto positive=proof(d(),10.02,2);positive.valid_until=stamp(10.04);
  assert(t.motionValidation(positive,10.02));assert(t.tick(10.025,h())->linear.x==.2);
  assert(t.tick(10.09,h())->linear.x==.2); // later positive's expiry is unrelated
  assert(t.tick(10.101,h())->linear.x==0.); // bound original source TTL unchanged
 }
 // A later invalid verdict cannot be concealed by another later positive.
 // This also covers invalid arriving before the delayed safe demand.
 {auto t=ready();auto command=d();command.sequence=2;command.motion_validation_sequence=2;
  auto bound=proof(command,10.01,2);assert(t.motionValidation(bound,10.01));
  auto invalid=proof(command,10.02,3);invalid.valid=false;assert(t.motionValidation(invalid,10.02));
  assert(t.motionValidation(proof(command,10.03,4),10.03));
  assert(!t.demand(command,10.03));assert(t.tick(10.04,h())->linear.x==0.);
 }
 // A newer positive cannot lend its longer TTL to an expired exact binding.
 {auto t=ready();auto command=d();command.sequence=2;command.motion_validation_sequence=2;
  auto bound=proof(command,10.01,2);bound.valid_until=stamp(10.02);assert(t.motionValidation(bound,10.01));
  assert(t.motionValidation(proof(command,10.03,3),10.03));
  assert(!t.demand(command,10.03));assert(t.tick(10.04,h())->linear.x==0.);
 }
 // Same command number in a different execution version cannot replace or
 // authorize this command; it cannot become an unrelated invalid fence.
 for(bool valid:{true,false}) {auto t=ready();assert(t.demand(d(),10.));
  auto foreign=proof(d(),10.02,2);++foreign.version.anchor_revision;foreign.valid=valid;
  assert(t.motionValidation(foreign,10.02));assert(t.tick(10.03,h())->linear.x==.2);
  auto command=d();command.sequence=2;command.motion_validation_sequence=3;
  auto wrong=proof(command,10.03,3);++wrong.version.anchor_revision;
  assert(t.motionValidation(wrong,10.03));assert(!t.demand(command,10.04));
 }
 // The writer's MC health and its stop evidence share one accepted raw sample
 // and SDK telemetry epoch. Frequent arrivals cannot disguise rejected source
 // timestamps; all original times and the conservative interval stay intact.
 {Transport t("sdk","live",true);t.configureMcCaptureBound(.02,"source_delta_host_anchor_approximate");
  assert(!t.mcSourceFresh(10.,"sdk:7"));
  t.mc(10030000000ULL,10.03,10.,0,0,10.,"sdk:7"); // exceeds the declared live uncertainty
  assert(!t.mcSourceFresh(10.,"sdk:7"));assert(t.report(10.).mc_raw_stamp_ns==0);
  t.mc(9000000000ULL,9.,10.,0,0,10.,"sdk:7"); // fresh arrival, expired capture
  assert(!t.mcSourceFresh(10.,"sdk:7"));assert(t.report(10.).mc_raw_stamp_ns==0);
  t.mc(10070000000ULL,10.07,10.1,0,0,10.1,"sdk:7");
  assert(t.mcSourceFresh(10.1,"sdk:7"));assert(!t.mcSourceFresh(10.1,"sdk:8"));
  assert(!t.mcSourceFresh(10.1,""));
  assert(t.report(10.1).source_stamp==stamp(10.1)&&t.report(10.1).mc_raw_stamp_ns==10070000000ULL);
  assert(std::abs(t.mcCaptureLowerBound()-10.05)<1e-12&&t.mcCaptureUpperBound()==10.1);
  // Excessive future/late data and replayed raw identities do not refresh it.
  t.mc(10430000000ULL,10.43,10.4,0,0,10.4,"sdk:7");
  t.mc(10100000000ULL,10.1,10.4,0,0,10.4,"sdk:7");
  t.mc(10070000000ULL,10.4,10.4,0,0,10.4,"sdk:7");
  assert(!t.mcSourceFresh(10.4,"sdk:7"));assert(t.report(10.4).mc_raw_stamp_ns==10070000000ULL);
  t.mc(10420000000ULL,10.42,10.42,0,0,10.42,"sdk:7");
  assert(t.mcSourceFresh(10.42,"sdk:7"));
 }
 // Short telemetry unavailability is a genuine zero-speed HOLD, not loss
 // of the independently live owner/control session. No old demand survives.
 {auto t=ready();assert(t.demand(d(),10.));assert(t.tick(10.01,h())->linear.x==.2);
  auto stale=h();stale.mc_fresh=false;
  assert(t.tick(10.05,stale)->linear.x==0.);auto held=t.state(10.05);
  assert(held.phase=="holding"&&held.reason=="mc_stale"&&!held.grant_ready&&held.control_owned&&!held.fault_latched);
  assert(!t.stopping()&&t.report(10.05).nonzero_blocked&&!t.report(10.05).stop_submitted);
  assert(t.executionId()=="execution"&&t.epoch()==1&&t.armGeneration()==1);
  auto waiting=p();waiting.sequence=2;waiting.allowed=false;waiting.source_stamp=stamp(10.1);waiting.valid_until=stamp(10.3);
  assert(t.permit(waiting,10.1));assert(t.tick(10.1,stale)->linear.x==0.);
  assert(!t.demand(d(),10.1));assert(!t.takeControlOnce());
  assert(t.tick(10.12,h())->linear.x==0.);auto recovered=t.state(10.12);
  assert(recovered.grant_ready&&recovered.reason.empty()&&!recovered.fault_latched);
  assert(t.executionId()=="execution"&&t.armGeneration()==1&&!t.stopping());
  // Both delayed old authorization and delayed old nonzero evidence fail.
  auto old=p();old.sequence=3;assert(!t.permit(old,10.13));assert(!t.demand(d(),10.13));
  auto current=p();current.sequence=3;current.source_stamp=stamp(10.14);current.valid_until=stamp(10.34);
  assert(t.permit(current,10.14));auto command=d();command.sequence=2;command.permit_sequence=3;
  command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.14);
  command.valid_until=stamp(10.3);command.motion_validation_sequence=2;
  assert(t.motionValidation(proof(command,10.14,2),10.14));assert(t.demand(command,10.14));
  refreshBody(t,10.14);
  assert(t.tick(10.15,h())->linear.x==.2);
 }
 // No owner heartbeat means terminal retirement, even if MC is missing.
 {auto t=ready();auto stale=h();stale.mc_fresh=false;t.tick(10.1,stale);t.tick(10.2,stale);
  assert(t.tick(10.36,stale)->linear.x==0.);assert(t.stopping());assert(t.state(10.36).reason=="permission_expired");}
 // A genuine control/mode/connection loss must not inherit MC HOLD recovery.
 for(int kind=0;kind<3;++kind) {auto t=ready();auto stale=h();stale.mc_fresh=false;t.tick(10.05,stale);
  auto lost=stale;if(kind==0)lost.owned=false;if(kind==1)lost.connected=false;if(kind==2)lost.general=false;
  t.tick(10.1,lost);assert(t.stopping()&&t.state(10.1).fault_latched);
  t.tick(10.15,h());assert(!t.state(10.15).grant_ready);assert(!t.demand(d(),10.15));assert(!t.takeControlOnce());}
 {Transport t("sdk","live");auto a=g();a.transport_mode="live";assert(!t.grant(a,10,h()).accepted);}
 {Transport t("sdk","isolated_mock");auto a=g();a.sdk_session="foreign";assert(!t.grant(a,10,h()).accepted);
  a=g();a.version.task_id.clear();assert(!t.grant(a,10,h()).accepted);a=g();a.source_stamp=stamp(9);assert(!t.grant(a,10,h()).accepted);}
 {auto t=ready();assert(t.grant(g(),10,h()).duplicate);auto a=g();a.version.route_hash=std::string(64,'b');assert(!t.grant(a,10,h()).accepted);}
 {Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");t.configureExecutionPolicy(testExecutionPolicy(),.8);
  auto health=h();health.owned=false;assert(t.grant(g(),10,health).accepted);
  assert(t.takeControlOnce());assert(!t.takeControlOnce());assert(!t.tick(10,health));
  t.telemetryRestartBeforeFirstMotion(10.1);health.owned=true;health.mc_fresh=false;t.tick(10.1,health);
  assert(!t.stopping());health.mc_fresh=true;t.tick(10.2,health);assert(t.state(10.2).grant_ready);assert(!t.takeControlOnce());}
 {auto t=ready();assert(t.demand(d(),10));assert(t.tick(10.05,h())->linear.x==.2);assert(!t.demand(d(),10.05));
  assert(t.tick(10.21,h())->linear.x==0);assert(!t.stopping());}
 {auto t=ready();auto a=d();a.safety_checked=false;assert(!t.demand(a,10));a.velocity.linear.x=0;a.hold=true;
  assert(t.demand(a,10));assert(t.tick(10.05,h())->linear.x==0);}
 for(int variant=0;variant<9;++variant){auto t=ready();auto a=d();switch(variant){
  case 0:a.version.anchor_revision=2;break;case 1:a.trajectory_id=2;break;case 2:a.sdk_arm_generation=2;break;
  case 3:a.transport_mode="live";break;case 4:a.source_stamp=stamp(11);break;case 5:a.velocity.linear.x=.31;break;
  case 6:a.velocity.linear.y=.01;break;case 7:a.velocity.angular.z=std::numeric_limits<double>::quiet_NaN();break;
  case 8:a.validation_sequence=2;break;}assert(!t.demand(a,10));}
 {auto t=ready();assert(t.demand(d(),10));auto health=h();health.owned=false;assert(!t.tick(10.1,health));
  assert(t.stopping());assert(t.state(10.1).fault_latched);assert(!t.takeControlOnce());assert(!t.demand(d(),10.1));}
 {auto t=ready();auto a=p();a.sequence=2;a.revoked=true;a.source_stamp=stamp(10.1);a.valid_until=stamp(10.3);
  assert(t.permit(a,10.1));assert(t.tick(10.1,h())->linear.x==0);assert(!t.report(10.1).measured_stop_confirmed);
  t.stopWritten(10.11);assert(t.report(10.11).stop_submitted);
  for(int i=0;i<=21;++i){const double time=10.16+i*.05;t.mc(10160000000ULL+i*50000000ULL,time,time,0,0,time,"epoch");}
  const auto r=t.report(11.21);assert(r.measured_stop_confirmed);assert(r.stationary_duration_sec>=1.);assert(!r.physical_acceptance_verified);
  assert(!t.report(12.).measured_stop_confirmed);
  auto next=g();next.request_id="next";next.execution_id="next-execution";next.control_epoch=2;next.source_stamp=stamp(12.);
  assert(!t.grant(next,12.,h()).accepted); // stale stop cannot arm another task
 }
 {auto t=ready();t.stop("cancel",10.1);t.stopWritten(10.1);for(int i=0;i<30;++i){double time=10.15+i*.05;
  t.mc(10150000000ULL+i*50000000ULL,time,time,i==20?.1:0.,0,time,"epoch");}
  assert(!t.report(11.6).measured_stop_confirmed);}
 {auto t=ready();t.mc(10000000000ULL,10,10,0,0,10,"epoch");t.mc(1,10.1,10.1,0,0,10.1,"restarted");
  assert(t.stopping());assert(t.state(10.1).fault_latched);}
 {auto t=ready();auto a=p();a.sequence=2;a.source_stamp=stamp(10.1);a.valid_until=stamp(10.11);assert(t.permit(a,10.1));
  assert(t.tick(10.12,h())->linear.x==0);assert(!t.stopping());
  a.sequence=3;a.source_stamp=stamp(10.13);a.valid_until=stamp(10.3);assert(t.permit(a,10.13));auto command=d();
  command.sequence=2;command.permit_sequence=3;command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.13);
  command.valid_until=stamp(10.25);assert(t.motionValidation(proof(command,10.13),10.13));assert(t.demand(command,10.13));
  refreshBody(t,10.13);assert(t.tick(10.13,h())->linear.x==.2);}
 {auto t=ready();t.stop("cancel",10.1);t.stopWritten(10.1);
  t.mc(10200000000ULL,10.2,10.200001,0,0,10.2,"clock");assert(t.report(10.2).mc_raw_stamp_ns==0);
  t.mc(10200000000ULL,10.2,10.200001,0,0,10.200001,"clock");assert(t.report(10.21).mc_raw_stamp_ns==10200000000ULL);}
 {auto t=ready();assert(t.demand(d(),10));assert(t.tick(10.4,h())->linear.x==0);assert(t.state(10.4).fault_latched);}
 {auto t=ready();t.stop("cancel_during_takeover",10.1);t.telemetryRestartBeforeFirstMotion(10.11);
  assert(!t.state(10.11).fault_latched);assert(t.stopping());}
 {auto t=ready();assert(t.demand(d(),10));const auto command=t.tick(10.01,h());assert(command);
  t.moveAttempted(*command);t.writeCalled(10.01,true);t.telemetryRestartBeforeFirstMotion(10.1);
  assert(t.state(10.1).fault_latched);assert(t.stopping());}
 // Same-geometry heartbeat preserves bounded valid commands, not their expiry.
 {auto t=ready();assert(t.demand(d(),10));auto a=p();a.sequence=2;a.validation_sequence=2;
  a.source_stamp=stamp(10.05);a.valid_until=stamp(10.3);assert(t.permit(a,10.05));
  refreshBody(t,10.05);
  assert(t.tick(10.05,h())->linear.x==.2);assert(t.tick(10.21,h())->linear.x==0);assert(!t.stopping());}
 // Original native proof may expire before the command's declared lifetime.
 {auto t=ready();auto a=p();a.sequence=2;a.source_stamp=stamp(10.01);a.valid_until=stamp(10.06);assert(t.permit(a,10.01));
  auto command=d();command.permit_sequence=2;command.motion_validation_sequence=2;
  assert(t.motionValidation(proof(command,10.01,2),10.01));assert(t.demand(command,10.01));
  a.sequence=3;a.source_stamp=stamp(10.05);a.valid_until=stamp(10.3);assert(t.permit(a,10.05));
  assert(t.tick(10.05,h())->linear.x==.2);assert(t.tick(10.07,h())->linear.x==0);assert(!t.stopping());}
 // New curve/anchor/phase cannot borrow a cached nonzero command.
 for(int variant=0;variant<4;++variant){auto t=ready();assert(t.demand(d(),10));auto a=p();a.sequence=2;
  a.source_stamp=stamp(10.01);a.valid_until=stamp(10.2);
  switch(variant){case 0:++a.trajectory_id;break;case 1:++a.version.anchor_revision;break;
    case 2:a.phase="aligning";break;case 3:a.allowed=false;break;}
  assert(t.permit(a,10.01));assert(t.tick(10.02,h())->linear.x==0);assert(!t.stopping());}
 // N+1 may arrive before the safe demand for N; preserve the exact N proof.
 {auto t=ready();auto newer=p();newer.sequence=2;newer.validation_sequence=2;newer.source_stamp=stamp(10.02);
  newer.valid_until=stamp(10.27);assert(t.permit(newer,10.02));assert(t.demand(d(),10.03));
  refreshBody(t,10.03);
  assert(t.tick(10.04,h())->linear.x==.2);assert(!t.stopping());
  auto wrong=d();wrong.sequence=2;wrong.validation_sequence=2;assert(!t.demand(wrong,10.05));}
 // New permission cannot revive an earlier proof whose own deadline ended.
 {auto t=ready();auto older=p();older.sequence=2;older.source_stamp=stamp(10.01);older.valid_until=stamp(10.04);
  assert(t.permit(older,10.01));auto newer=older;newer.sequence=3;newer.source_stamp=stamp(10.03);newer.valid_until=stamp(10.28);
  assert(t.permit(newer,10.03));auto command=d();command.permit_sequence=2;assert(!t.demand(command,10.05));}
 for(int variant=0;variant<5;++variant){auto t=ready();auto newer=p();newer.sequence=2;newer.source_stamp=stamp(10.01);
  switch(variant){case 0:newer.phase="aligning";break;case 1:newer.allowed=false;break;
   case 2:newer.revoked=true;break;case 3:++newer.trajectory_id;break;case 4:++newer.version.reference_generation;break;}
  assert(t.permit(newer,10.01));assert(!t.demand(d(),10.02));assert(t.tick(10.02,h())->linear.x==0);
  if(variant==1){newer.sequence=3;newer.allowed=true;newer.source_stamp=stamp(10.03);assert(t.permit(newer,10.03));
    assert(!t.demand(d(),10.04));} // HOLD is an irreversible fence for older N.
 }
 // Independent motion proof is mandatory; a safety_checked bool alone is not evidence.
 {auto t=ready();auto command=d();command.sequence=2;command.motion_validation_sequence=2;
  assert(!t.demand(command,10.01));assert(t.tick(10.01,h())->linear.x==0);
  assert(t.motionValidation(proof(command,10.02,2),10.02));assert(t.tick(10.03,h())->linear.x==.2);}
 // A later explicit invalid verdict cannot be hidden by earlier good proof.
 {auto t=ready();assert(t.demand(d(),10));auto bad=proof(d(),10.01,2);bad.valid=false;
  assert(t.motionValidation(bad,10.01));assert(t.tick(10.02,h())->linear.x==0);assert(!t.demand(d(),10.02));}
 for(int variant=0;variant<10;++variant){auto t=ready();auto command=d();command.sequence=2;command.motion_validation_sequence=2;
  auto bad=proof(command,10.02,2);switch(variant){case 0:bad.velocity.angular.z=.1;break;
   case 1:bad.demand_body_source_stamp=stamp(10.01);break;case 2:bad.demand_source_stamp=stamp(10.01);break;
   case 3:bad.braking_model_sha256=std::string(64,'d');break;case 4:bad.version.anchor_revision=9;break;
   case 5:bad.sdk_arm_generation=9;break;case 6:bad.transport_mode="live";break;
   case 7:bad.valid_until=stamp(10.01);break;case 8:bad.frame_id="map";break;case 9:bad.demand_valid_until=stamp(10.03);break;}
  t.motionValidation(bad,10.02);assert(!t.demand(command,10.02));assert(t.tick(10.03,h())->linear.x==0);}
 {auto t=ready();assert(t.demand(d(),10));assert(t.tick(10.1001,h())->linear.x==0);}
 {auto t=ready();auto command=d();command.sequence=2;command.motion_validation_sequence=2;
  assert(!t.demand(command,10.01));auto hold=p();hold.sequence=2;hold.allowed=false;hold.source_stamp=stamp(10.02);
  assert(t.permit(hold,10.02));assert(t.motionValidation(proof(command,10.03,2),10.03));assert(t.tick(10.04,h())->linear.x==0);}
 // An unexpired exact owner lease permits a newer same-curve proof, avoiding
 // a heartbeat-sized stop when the proof available at signing has expired.
 {auto t=ready();auto command=d();command.sequence=2;command.validation_sequence=2;command.motion_validation_sequence=2;
  command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.03);command.valid_until=stamp(10.11);
  auto newer=proof(command,10.03,2);assert(t.motionValidation(newer,10.03));
  assert(t.demand(command,10.03));assert(t.tick(10.04,h())->linear.x==.2);
  assert(t.tick(10.111,h())->linear.x==0); // actual proof deadline unchanged
 }
 // Floor relaxation is not permission to choose an earlier proof, a foreign
 // curve/version, or to conceal the latest explicit invalid actual demand.
 for(int variant=0;variant<5;++variant){auto t=ready();auto owner=p();owner.sequence=2;owner.validation_sequence=3;
  owner.source_stamp=stamp(10.01);assert(t.permit(owner,10.01));
  auto command=d();command.sequence=2;command.permit_sequence=2;command.validation_sequence=4;command.motion_validation_sequence=2;
  command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.02);
  if(variant==0)command.validation_sequence=2;
  if(variant==1)++command.trajectory_id;
  if(variant==2)++command.version.anchor_revision;
  auto evidence=proof(command,10.02,2);
  if(variant==3)evidence.trajectory_validation_sequence=3;
  if(variant==4)evidence.valid=false;
  t.motionValidation(evidence,10.02);assert(!t.demand(command,10.02));assert(t.tick(10.03,h())->linear.x==0);
 }
 {auto t=ready();auto command=d();command.sequence=2;command.validation_sequence=2;command.motion_validation_sequence=2;
  assert(t.motionValidation(proof(command,10.02,2),10.02));assert(t.demand(command,10.02));
  auto invalid=proof(command,10.03,3);invalid.valid=false;assert(t.motionValidation(invalid,10.03));
  assert(t.tick(10.04,h())->linear.x==0);
 }
 // Native may replace an ageing curve proof while retaining the original
 // demand floor, deadline and velocity. Vary callback phase across 50 ms.
 for(const double phase:{.005,.015,.025,.035,.045}){auto t=ready();auto command=d();
  command.sequence=2;command.motion_validation_sequence=2;command.validation_sequence=1;
  command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.02);
  command.valid_until=stamp(10.17);assert(!t.demand(command,10.02));
  auto evidence=proof(command,10.02+phase,2);evidence.trajectory_validation_sequence=3;
  evidence.valid_until=stamp(10.09);
  assert(t.motionValidation(evidence,10.02+phase));assert(t.tick(10.025+phase,h())->linear.x==.2);
  assert(t.tick(10.091,h())->linear.x==0); // actual sweep TTL was not renewed
 }
 // A newer curve sequence cannot hide an invalid actual-command certificate,
 // extend the source time, change geometry, or revive an expired demand.
 for(int variant=0;variant<5;++variant){auto t=ready();auto command=d();command.sequence=2;
  command.motion_validation_sequence=2;command.source_stamp=command.body_source_stamp=command.safety_source_stamp=stamp(10.02);
  command.valid_until=stamp(10.17);auto evidence=proof(command,10.03,2);evidence.trajectory_validation_sequence=9;
  if(variant==0)evidence.valid=false;
  if(variant==1)evidence.demand_source_stamp=stamp(10.021);
  if(variant==2)evidence.trajectory_id=2;
  if(variant==3)evidence.version.anchor_revision=2;
  if(variant==4)evidence.valid_until=stamp(10.025);
  t.motionValidation(evidence,10.03);assert(!t.demand(command,10.03));assert(t.tick(10.04,h())->linear.x==0);
 }
 // A historical stop does not certify a new MC sample that shows motion.
 for(int variant=0;variant<4;++variant){auto t=ready();t.stop("cancel",10.1);t.stopWritten(10.1);
  uint64_t raw=10150000000ULL;double time=10.15;
  for(int i=0;i<22;++i){time=10.15+i*.05;raw=10150000000ULL+i*50000000ULL;t.mc(raw,time,time,0,0,time,"epoch");}
  assert(t.report(time).measured_stop_confirmed);
  double next=time+.05;uint64_t next_raw=raw+50000000ULL;
  if(variant==2){next+=.25;next_raw+=250000000ULL;}
  t.mc(next_raw,next,next,variant==0?.1:0.,variant==1?.1:0.,next,variant==3?"changed":"epoch");
  assert(!t.report(next).measured_stop_confirmed);
  auto request=g();request.request_id="next";request.execution_id="next-execution";request.control_epoch=2;
  request.source_stamp=stamp(next);assert(!t.grant(request,next,h()).accepted);
  if(variant==3){assert(t.state(next).fault_latched);continue;}
  for(int i=1;i<=19;++i){const auto at=next+i*.05;t.mc(next_raw+i*50000000ULL,at,at,0,0,at,"epoch");
   assert(!t.report(at).measured_stop_confirmed);}
  for(int i=20;i<=22;++i){const auto at=next+i*.05;t.mc(next_raw+i*50000000ULL,at,at,0,0,at,"epoch");}
  assert(t.report(next+1.1).measured_stop_confirmed);
 }
 // A record without a bounded capture-time contract cannot arm either mode.
 {Transport t("sdk","live",true);auto request=g();request.transport_mode="live";
  assert(t.grant(request,10,h()).reason=="mc_capture_time_bound_unavailable");
  bool invalid=false;try{t.configureMcCaptureBound(0.,"source_delta_host_anchor_approximate");}
  catch(const std::invalid_argument&){invalid=true;}assert(invalid);}
 {Transport t("sdk","isolated_mock");assert(t.grant(g(),10,h()).reason=="mc_capture_time_bound_unavailable");}
 {Transport t("sdk","live",true);t.configureMcCaptureBound(.1,"source_delta_host_anchor_approximate");
  auto request=g();request.transport_mode="live";assert(t.grant(request,10.,h()).reason=="execution_policy_unavailable");}
 // Static MC noise must be accepted by the measured record, not a hardcoded
 // epsilon. Above-threshold true motion still invalidates the stop proof.
 {Transport t("sdk","live",true);t.configureMcCaptureBound(.02,"source_delta_host_anchor_approximate");
  auto policy=testExecutionPolicy();policy.linear_threshold_mps=.04;policy.measured_static_linear_bound_mps=.038;
  t.configureExecutionPolicy(policy,.8);auto request=g();request.transport_mode="live";assert(t.grant(request,10.,h()).accepted);
  t.stop("cancel",10.1);t.stopWritten(10.1);
  for(int i=0;i<=24;++i){const double at=10.15+i*.05;t.mc(10150000000ULL+i*50000000ULL,at,at,.038,.02,at,"epoch");}
  assert(t.report(11.351).measured_stop_confirmed);
  t.mc(11400000000ULL,11.4,11.4,.041,.02,11.4,"epoch");assert(!t.report(11.4).measured_stop_confirmed);}
 {Transport t("sdk","isolated_mock");assert(!t.beginShutdown(10.));
  assert(t.grant(g(),10.,h()).reason=="writer_shutting_down");assert(!t.takeControlOnce());}
 // Buffered samples with raw time after the cutoff but whose possible capture
 // is before the zero ACK must not contribute to stationary evidence.
 {Transport t("sdk","live",true);t.configureMcCaptureBound(.10,"source_delta_host_anchor_approximate");
  t.configureExecutionPolicy(testExecutionPolicy(),.8);
  auto request=g();request.transport_mode="live";assert(t.grant(request,10,h()).accepted);
  t.stop("cancel",10.1);t.stopWritten(10.1);
  for(int i=0;i<10;++i){const double at=10.105+i*.009;t.mc(10105000000ULL+i*9000000ULL,at,at,0,0,at,"epoch");
    assert(t.report(at).stationary_samples==0);assert(!t.report(at).measured_stop_confirmed);}
  for(int i=0;i<25;++i){const double at=10.22+i*.05;t.mc(10220000000ULL+i*50000000ULL,at,at,0,0,at,"epoch");}
  const auto report=t.report(11.421);assert(report.measured_stop_confirmed);assert(report.stationary_duration_sec>=1.);
  assert(report.mc_raw_stamp_ns==11420000000ULL);assert(report.time_basis=="source_delta_host_anchor_approximate");
  assert(t.mcCaptureLowerBound()<=t.mcCaptureUpperBound());}
 // Approximate timestamps spanning one second are insufficient when the
 // conservative capture interval spans less than one second.
 {auto t=ready();t.stop("cancel",10.1);t.stopWritten(10.1);
  for(int i=0;i<=20;++i){const double at=10.15+i*.05;t.mc(10150000000ULL+i*50000000ULL,at,at,0,0,at,"epoch");}
  assert(!t.report(11.15).measured_stop_confirmed);assert(t.report(11.15).stationary_duration_sec<1.);
  t.mc(11200000000ULL,11.2,11.2,0,0,11.2,"epoch");assert(t.report(11.2).measured_stop_confirmed);}
 // Direct writer shutdown permanently fences nonzero data and new grants,
 // but a zero ACK alone still cannot certify standstill.
 {auto t=ready();assert(t.demand(d(),10));assert(t.tick(10.01,h())->linear.x==.2);
  assert(t.beginShutdown(10.02));assert(t.tick(10.03,h())->linear.x==0);
  assert(!t.demand(d(),10.03));assert(!t.takeControlOnce());t.stopWritten(10.04);
  assert(t.report(10.04).stop_submitted);assert(!t.report(10.04).measured_stop_confirmed);
  auto request=g();request.source_stamp=stamp(10.04);assert(t.grant(request,10.04,h()).reason=="writer_shutting_down");}
 // SDK-transaction-backed terminal completion is a historical event. A later
 // MC clock fault cannot erase that history, but MUST prevent every new owner.
 {auto t=ready();t.setSDKTransactions(0,false);t.moveAttempted(true);
  t.stop("goal_reached",10.1);t.stopWritten(10.11);
  for(unsigned i=0;i<26;++i){const auto at=10.15+i*.05;
    t.mc(10150000000ULL+i*50000000ULL,at,at,.022,.008,at,"epoch");t.tick(at,h());}
  const auto terminal=t.report(11.4);assert(terminal.measured_stop_confirmed);
  assert(t.retirement(11.4).software_retired&&secs(terminal.source_stamp)<11.4);
  t.mc(11450000000ULL,11.45,11.45,.0316,.008,11.45,"changed-clock");
  assert(t.report(11.45)==terminal&&!t.currentStopReport(11.45).measured_stop_confirmed);
  assert(t.state(11.45).fault_latched&&!t.completeManualHandoff(11.45));
  auto next=g();next.source_stamp=stamp(11.45);next.request_id="next";next.execution_id="next";next.control_epoch=2;
  assert(t.grant(next,11.45,h()).reason=="writer_fault_latched");}
 // An estop arriving AFTER a proven terminal task cannot reopen that task,
 // nor turn harmless later MC drift into an eternal old-retirement barrier.
 {auto t=ready();t.setSDKTransactions(0,false);t.moveAttempted(true);
  t.stop("goal_reached",10.1);t.stopWritten(10.11);
  for(unsigned i=0;i<26;++i){const auto at=10.15+i*.05;
    t.mc(10150000000ULL+i*50000000ULL,at,at,.022,.008,at,"epoch");t.tick(at,h());}
  const auto terminal=t.report(11.4);assert(terminal.measured_stop_confirmed);
  t.mc(11450000000ULL,11.45,11.45,.0316,.008,11.45,"epoch");
  t.softwareEstopRequested(11.45);assert(!t.completeManualHandoff(11.45));
  assert(t.confirmSoftwareEstopRelease(11.46));t.tick(11.47,h());
  assert(!t.softwareEstopLatched()&&t.report(11.47)==terminal);
  assert(!t.permit(p(),11.47)&&!t.demand(d(),11.47));
  assert(t.completeManualHandoff(11.47)&&t.executionId().empty());}
 std::cout<<"schema3 execution transport positive/negative assertions passed\n";
}
