#include "execution_transport_core.hpp"
#include "execution_publication_core.hpp"
#include "mc_report_core.hpp"
#include "test_execution_fixture.hpp"
#include <cassert>
#include <iostream>
using namespace d1monitor::execution3;
static Version version1(){Version v;v.schema_version=3;v.session_id="nav";v.task_id="task";v.route_id="route";
 v.route_hash=std::string(64,'a');v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";
 v.reference_generation=v.anchor_revision=v.context_sequence=v.map_geometry_revision=1;v.segment_id="floor";v.anchor_id="anchor";return v;}
static Health health(){return {true,true,true,true,false};}
static Permit permit1(){Permit p;p.version=version1();p.execution_id="execution";p.control_epoch=1;p.sdk_session="sdk";p.sdk_arm_generation=1;
 p.sequence=p.validation_sequence=1;p.trajectory_id=1;p.source_stamp=stamp(10.);p.valid_until=stamp(10.25);
 p.allowed=p.geometry_committed=true;p.phase="tracking";p.frame_id="d1max_loc_odom";p.transport_mode="isolated_mock";return p;}
static Demand command(const Permit&p,double time,uint64_t seq){Demand d;d.version=p.version;d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;
 d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.sequence=seq;d.permit_sequence=p.sequence;d.validation_sequence=p.validation_sequence;
 d.trajectory_id=p.trajectory_id;d.source_stamp=d.body_source_stamp=d.safety_source_stamp=stamp(time);d.valid_until=stamp(time+.1);
 d.velocity.linear.x=.2;d.safety_checked=true;d.transport_mode=p.transport_mode;d.motion_validation_sequence=seq;d.braking_model_sha256=std::string(64,'c');return d;}
static Local body(double time,double x=0.,double speed=.2){Local m;m.schema_version=1;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(time);m.usable=true;m.local_odometry.header.stamp=m.source_stamp;
 m.local_odometry.header.frame_id="d1max_loc_odom";m.local_odometry.child_frame_id="d1max_loc_base_link";
 m.local_odometry.pose.pose.position.x=x;m.local_odometry.pose.pose.orientation.w=1.;m.local_odometry.twist.twist.linear.x=speed;return m;}
static CurveProof curve(const Permit&p,double time){CurveProof v;v.version=p.version;v.trajectory_id=p.trajectory_id;v.sequence=p.validation_sequence;
 v.source_stamp=v.check_begin=v.check_end=v.body_source_stamp=v.front_ray_source_stamp=v.rear_ray_source_stamp=stamp(time);
 v.valid_until=stamp(time+.2);v.valid=v.whole_curve=true;v.curve_duration=v.checked_to_time=1.;v.map_snapshot_revision=1;
 v.support_reference_id="support";v.support_hash=std::string(64,'d');v.frame_id="d1max_loc_odom";v.collision_policy="observed_free";v.transport_mode=p.transport_mode;return v;}
static MotionProof sweep(const Demand&d,double time){MotionProof p;p.version=d.version;p.execution_id=d.execution_id;p.control_epoch=d.control_epoch;
 p.sdk_session=d.sdk_session;p.sdk_arm_generation=d.sdk_arm_generation;p.trajectory_id=d.trajectory_id;p.permit_sequence=d.permit_sequence;
 p.trajectory_validation_sequence=d.validation_sequence;p.demand_sequence=d.sequence;p.demand_source_stamp=d.source_stamp;
 p.demand_valid_until=d.valid_until;p.demand_body_source_stamp=d.body_source_stamp;p.body_source_stamp=stamp(time);p.sequence=d.motion_validation_sequence;
 p.front_ray_source_stamp=p.rear_ray_source_stamp=p.check_begin=p.check_end=stamp(time);p.map_snapshot_revision=1;p.valid_until=d.valid_until;
 p.velocity=d.velocity;p.frame_id="d1max_loc_odom";p.valid=true;p.transport_mode=d.transport_mode;p.braking_model_sha256=d.braking_model_sha256;return p;}
static Transport armed(){Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");t.configureBrakingModel(std::string(64,'c'));
 t.configureExecutionPolicy(testExecutionPolicy(),.8);
 Grant::Request r;r.version=version1();r.sdk_session="sdk";r.execution_id="execution";r.control_epoch=1;r.request_id="grant";
 r.source_stamp=stamp(10.);r.activate=true;r.transport_mode="isolated_mock";assert(t.grant(r,10.,health()).accepted);t.tick(10.,health());
 assert(t.localState(body(10.),10.));assert(t.permit(permit1(),10.));assert(t.trajectoryValidation(curve(permit1(),10.),10.));
 const auto d=command(permit1(),10.,1);assert(t.motionValidation(sweep(d,10.),10.));assert(t.demand(d,10.));return t;}
static Transport active(){auto t=armed();assert(t.localState(body(10.005),10.005));assert(t.tick(10.005,health())->linear.x==.2);
 t.writeCalled(10.005,true);const auto a=t.takeCommitAck();assert(a&&a->applied&&a->commit_sequence==1&&a->previous_commit_sequence==0);
 assert(a->body_source_stamp==stamp(10.005)&&a->measured_pose.header.stamp==a->body_source_stamp);
 assert(a->demand_body_source_stamp==stamp(10.)&&a->demand_source_stamp==stamp(10.)&&!a->write_acknowledged);
 CommitOutbox outbox;assert(outbox.put(*a)==CommitOutbox::Result::Queued);
 t.writeAcknowledged(1,10.01);const auto ack=t.takeCommitAck();assert(ack&&ack->write_acknowledged&&ack->applied_at==stamp(10.005));
 assert(outbox.put(*ack)==CommitOutbox::Result::Coalesced&&outbox.size()==1);
 CommitAck emitted;assert(outbox.pop(emitted)&&emitted==*ack);return t;}
static Handoff grant(){Handoff g;g.schema_version=2;g.transition_mode=Handoff::CONTINUOUS_REPLACE;g.handoff_id="handoff-1";g.sequence=g.expected_commit_sequence=1;
 g.incumbent=permit1();g.candidate=permit1();g.candidate.version.reference_generation=2;g.candidate.version.anchor_revision=2;
 g.candidate.trajectory_id=2;g.candidate.sequence=g.candidate.validation_sequence=2;g.candidate.geometry_committed=false;
 g.candidate.source_stamp=g.source_stamp=stamp(10.02);g.valid_until=g.transition_deadline=g.candidate.valid_until=stamp(10.22);
 g.retain_incumbent_until=stamp(10.2);return g;}
static Prepared prepared(const Handoff&g,double time=10.03){Prepared p;p.schema_version=1;p.handoff_id=g.handoff_id;
 p.grant_sequence=g.sequence;p.expected_commit_sequence=g.expected_commit_sequence;p.demand=command(g.candidate,time,2);
 p.entry_source_stamp=stamp(time);p.measured_pose.header.stamp=p.entry_source_stamp;p.measured_pose.header.frame_id="d1max_loc_odom";
 p.measured_pose.pose.orientation.w=p.curve_entry_pose.orientation.w=1.;p.measured_twist.linear.x=p.curve_entry_twist.linear.x=.2;
 p.position_tolerance_m=.0125;p.velocity_tolerance_mps=.05;p.curve_time=.1;
 auto&a=p.entry_admission;a.sequence=1;a.version=p.demand.version;a.trajectory_id=p.demand.trajectory_id;
 a.validation_sequence=p.demand.validation_sequence;a.body_source_stamp=p.entry_source_stamp;a.checked_at=stamp(time);
 a.valid_until=p.demand.valid_until;a.accepted=true;a.transport_mode=p.demand.transport_mode;return p;}
static MotionProof sweep(const Prepared&p,double time){auto v=sweep(p.demand,time);v.handoff_id=p.handoff_id;v.entry_admission_sequence=p.entry_admission.sequence;
 v.entry_curve_pose=p.curve_entry_pose;v.entry_curve_twist=p.curve_entry_twist;v.entry_curve_time=p.curve_time;
 v.entry_position_error_m=p.position_error_m;v.entry_velocity_error_mps=p.velocity_error_mps;return v;}
static void evidence(Transport&t,const Handoff&g,const Prepared&p,double now){assert(t.localState(body(now),now));
 assert(t.trajectoryValidation(curve(g.candidate,now),now));assert(t.motionValidation(sweep(p,now),now));}
static Transport stationary(bool acknowledge=true){auto t=active();
 for(unsigned i=0;i<=37;++i){const double now=10.06+i*.02;auto hold=permit1();hold.sequence=i+2;hold.allowed=false;hold.phase="holding";
  hold.source_stamp=stamp(now);hold.valid_until=stamp(now+.25);assert(t.permit(hold,now));
  t.mc(1000000000ULL+i*20000000ULL,now,now,0.,0.,now,"mc-clock");
  assert(t.tick(now,health())->linear.x==0.);const auto write=t.nextWriteSequence();t.writeCalled(now,true);
  if(acknowledge)t.zeroAcknowledged(write,now,now);
 }
 return t;
}
static Handoff reentry(const Stationary&s){auto g=grant();g.transition_mode=Handoff::STATIONARY_REENTRY;g.stationary_evidence=s;
 g.incumbent=permit1();g.incumbent.sequence=39;g.incumbent.allowed=false;g.incumbent.phase="holding";
 g.incumbent.source_stamp=g.source_stamp=stamp(10.80);g.incumbent.valid_until=g.valid_until=g.transition_deadline=stamp(11.04);
 g.retain_incumbent_until=g.source_stamp;g.candidate.sequence=40;g.candidate.source_stamp=g.source_stamp;g.candidate.valid_until=g.valid_until;
 return g;}
int main(){
 // Real-epoch regression: double seconds cannot represent all nanoseconds.
 // The first nonzero ACK must copy the exact minimum original command,
 // permission, motion-sweep and geometry deadlines, never round one forward.
 for(int limiting=0;limiting<4;++limiting)for(bool expired:{false,true}) {
  const double base=1791011008.;const double source=base+.02;
  builtin_interfaces::msg::Time exact;exact.sec=1791011008;exact.nanosec=75261729;
  assert(stamp(secs(exact))!=exact); // the old export loses original bytes
  Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");t.configureExecutionPolicy(testExecutionPolicy(),.8);
  t.configureBrakingModel(std::string(64,'c'));
  Grant::Request r;r.version=version1();r.sdk_session="sdk";r.execution_id="execution";r.control_epoch=1;r.request_id="grant";
  r.source_stamp=stamp(base);r.activate=true;r.transport_mode="isolated_mock";assert(t.grant(r,base,health()).accepted);t.tick(base,health());
  auto p=permit1();p.source_stamp=stamp(base);p.valid_until=stamp(base+.25);
  if(limiting==1)p.valid_until=exact;
  assert(t.permit(p,source));assert(t.localState(body(source),source));
  auto c=curve(p,source);if(limiting==3)c.valid_until=exact;
  assert(t.trajectoryValidation(c,source));auto d=command(p,source,1);
  if(limiting==0)d.valid_until=exact;
  auto m=sweep(d,source);m.valid_until=limiting==2?exact:d.valid_until;
  assert(t.motionValidation(m,source));assert(t.demand(d,source));
  const double at=base+(expired?.08:.03);assert(t.localState(body(at),at));
  const auto velocity=t.tick(at,health());t.writeCalled(at,true);const auto ack=t.takeCommitAck();
  if(expired)assert(velocity->linear.x==0.&&!ack&&t.state(at).writer_commit_sequence==0);
  else {assert(velocity->linear.x==.2&&ack&&ack->valid_until==exact);
   assert(ack->demand_source_stamp==d.source_stamp&&ack->demand_body_source_stamp==d.body_source_stamp);
   assert(ack->permit_sequence==p.sequence&&ack->motion_validation_sequence==m.sequence);
   t.writeAcknowledged(1,at+.001);const auto later=t.takeCommitAck();
   assert(later&&later->valid_until==exact&&later->applied_at==ack->applied_at);}
 }
 // The same precise sweep cap applies to a conditional new-geometry call.
 {auto t=active();const auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));
  assert(t.localState(body(10.03),10.03));assert(t.trajectoryValidation(curve(g.candidate,10.03),10.03));
  auto m=sweep(p,10.03);m.valid_until.sec=10;m.valid_until.nanosec=75261729;
  assert(t.motionValidation(m,10.03));assert(t.preparedDemand(p,10.03));
  assert(t.tick(10.04,health())->linear.x==.2);t.writeCalled(10.04,true);const auto ack=t.takeCommitAck();
  assert(ack&&ack->commit_sequence==2&&ack->valid_until==m.valid_until);
 }
 // Whole geometry/real body are prerequisites even for the first zero-call
 // commit; the actual ACK distinguishes demand, entry and latest body times.
 {auto t=active();assert(t.state(10.01).writer_commit_sequence==1&&t.state(10.01).applied_trajectory_id==1);}
 // Zero submission facts keep the selected native geometry deadline. A
 // later positive renewal cannot silently lengthen the already-selected ACK,
 // and expiry never creates an initial commit through the zero fast path.
 for(bool expired:{false,true}) {auto t=armed();auto zero=command(permit1(),10.01,2);
  zero.velocity.linear.x=0.;zero.hold=true;zero.motion_validation_sequence=0;
  assert(t.demand(zero,10.01));assert(t.localState(body(10.01,0.,0.),10.01));
  auto original=curve(permit1(),10.01);original.sequence=2;original.valid_until=stamp(10.04);
  assert(t.trajectoryValidation(original,10.01));const double at=expired?10.041:10.02;
  assert(t.tick(at,health())->linear.x==0.);
  if(!expired){auto later=curve(permit1(),10.0205);later.sequence=3;
   assert(t.trajectoryValidation(later,10.0205));}
  t.writeCalled(expired?at:10.021,true);const auto ack=t.takeCommitAck();
  if(expired)assert(!ack&&t.state(at).writer_commit_sequence==0);
  else {assert(ack&&ack->applied&&ack->valid_until==original.valid_until);
   assert(ack->demand_source_stamp==zero.source_stamp&&ack->demand_body_source_stamp==zero.body_source_stamp);
   assert(ack->permit_sequence==zero.permit_sequence&&ack->demand_sequence==zero.sequence);
   assert(ack->motion_validation_sequence==0&&ack->applied_velocity.linear.x==0.);}
 }
 // A remaining-curve zero initial commit is allowed only with its exact
 // measured XYZ progress; its shorter original proof still caps the ACK.
 for(bool wrong_progress:{false,true}) {auto t=armed();auto zero=command(permit1(),10.02,2);
  zero.velocity.linear.x=0.;zero.hold=true;zero.motion_validation_sequence=0;
  assert(t.demand(zero,10.02));assert(t.localState(body(10.02,0.,0.),10.02));
  auto c=curve(permit1(),10.01);c.sequence=2;c.valid_until=stamp(10.035);
  c.whole_curve=false;c.remaining_curve=true;c.checked_from_time=.1;c.valid_start_time=.2;c.reverse_margin_m=.15;
  assert(t.trajectoryValidation(c,10.02));
  Progress p;p.schema_version=2;p.header=body(10.02).local_odometry.header;p.session_id="nav";p.task_id="task";p.route_id="route";
  p.route_hash=version1().route_hash;p.map_version_id="map";p.localization_epoch=1;p.localization_seed_id="seed";
  p.generation=p.anchor_revision=p.context_sequence=1;p.trajectory_id=wrong_progress?2:1;p.segment_id="floor";p.anchor_id="anchor";
  p.curve_time=p.arc_length=.3;p.pose.orientation.w=1.;p.valid=true;assert(t.trackingProgress(p,10.02));
  assert(t.tick(10.03,health())->linear.x==0.);t.writeCalled(10.03,true);const auto ack=t.takeCommitAck();
  if(wrong_progress)assert(!ack&&t.state(10.03).writer_commit_sequence==0);
  else assert(ack&&ack->valid_until==c.valid_until&&ack->motion_validation_sequence==0);
 }
 // Conditional zero replacement never borrows a remaining incumbent proof.
 // It still needs the full NEW geometry and exact native command sweep.
 for(int kind=0;kind<3;++kind) {auto t=active();const auto g=grant();auto p=prepared(g);
  p.demand.velocity.linear.x=0.;p.demand.hold=true;assert(t.handoff(g,10.02));
  assert(t.localState(body(10.03),10.03));auto c=curve(g.candidate,10.03);c.valid_until=stamp(10.045);
  if(kind==1){c.whole_curve=false;c.remaining_curve=true;c.checked_from_time=.1;c.valid_start_time=.2;c.reverse_margin_m=.15;}
  assert(t.trajectoryValidation(c,10.03));assert(t.motionValidation(sweep(p,10.03),10.03));
  t.preparedDemand(p,10.03);const double at=kind==2?10.046:10.04;
  const auto out=t.tick(at,health());t.writeCalled(at,true);const auto ack=t.takeCommitAck();
  if(kind!=0)assert(out->linear.x==.2&&!ack&&t.state(at).writer_commit_sequence==1);
  else assert(out->linear.x==0.&&ack&&ack->commit_sequence==2&&ack->valid_until==c.valid_until);
 }
 // Nonterminal MC evidence is not inferred from BT/local velocity or a zero
 // submission. It needs the actual zero ACK and new conservative captures.
 {auto t=stationary(false);assert(!t.stationaryEvidence(10.80).usable);
  t.zeroAcknowledged(t.nextWriteSequence(),10.80,10.80);assert(!t.stationaryEvidence(10.80).usable);}
 {auto t=stationary();const auto s=t.stationaryEvidence(10.80);assert(s.usable&&s.nonzero_blocked);
  assert(s.zero_ack_at==stamp(10.06)&&s.stationary_samples>=3&&s.stationary_duration_sec>=.6);
  assert(secs(s.capture_lower_bound)>secs(s.zero_ack_at)&&s.valid_until==stamp(11.05));
  assert(s.time_basis=="isolated_simulated_source_clock"&&!s.physical_acceptance_verified);
  const auto samples=s.stationary_samples;t.mc(s.mc_raw_stamp_ns,10.80,10.80,0.,0.,10.80,"mc-clock");
  assert(t.stationaryEvidence(10.80).stationary_samples==samples); // duplicate raw is no new evidence
  assert(!t.stationaryEvidence(11.06).usable); // source/receipt expiry cannot be restamped
 }
 // Unsafe/expired old geometry can remain only an identity anchor during a
 // stationary re-entry. Real MC is stopped, and a NEW full curve and exact
 // measured entry are still independently mandatory for the writer CAS.
 {auto t=stationary();const auto s=t.stationaryEvidence(10.80);const auto g=reentry(s);assert(t.handoff(g,10.80));
  auto heartbeat=g.incumbent;heartbeat.sequence=41;heartbeat.source_stamp=stamp(10.81);heartbeat.valid_until=stamp(11.06);
  assert(t.permit(heartbeat,10.81));assert(t.tick(10.81,health())->linear.x==0.); // repeated HOLD does not retire prepared
  auto p=prepared(g,10.82);p.measured_twist.linear.x=p.curve_entry_twist.linear.x=0.;
  assert(t.localState(body(10.82,0.,0.),10.82));assert(t.trajectoryValidation(curve(g.candidate,10.82),10.82));
  assert(t.motionValidation(sweep(p,10.82),10.82));assert(t.preparedDemand(p,10.82));
  assert(t.tick(10.83,health())->linear.x==.2);t.writeCalled(10.83,true);const auto ack=t.takeCommitAck();
  assert(ack&&ack->applied&&ack->commit_sequence==2&&ack->previous_commit_sequence==1);
  assert(!t.stationaryEvidence(10.83).usable); // old stationary identity cannot authorize another CAS
 }
 for(int invalid=0;invalid<8;++invalid){auto t=stationary();auto s=t.stationaryEvidence(10.80);auto g=reentry(s);
  if(invalid==0)g.stationary_evidence.sequence++;
  if(invalid==1)g.stationary_evidence.mc_raw_stamp_ns++;
  if(invalid==2)g.stationary_evidence.zero_ack_at=stamp(10.07);
  if(invalid==3)g.incumbent.allowed=true;
  if(invalid==4)g.retain_incumbent_until=stamp(10.81);
  if(invalid==5)g.candidate.valid_until=stamp(11.06);
  if(invalid==6)g.schema_version=1;
  if(invalid==7){t.mc(s.mc_raw_stamp_ns+20000000ULL,10.82,10.82,.04,0.,10.82,"mc-clock");}
  assert(!t.handoff(g,invalid==7?10.82:10.80));
 }
 {auto t=stationary();const auto s=t.stationaryEvidence(10.80);const auto g=reentry(s);assert(t.handoff(g,10.80));
  auto h=health();h.mc_fresh=false;assert(t.tick(10.81,h)->linear.x==0.);
  assert(!t.stationaryEvidence(10.81).usable);assert(t.handoff(g,10.82)); // only immutable negative ACK replay
  const auto ack=t.takeCommitAck();assert(ack&&!ack->applied&&ack->reason=="mc_stale");
  assert(!t.preparedDemand(prepared(g,10.82),10.82)&&t.state(10.82).writer_commit_sequence==1);
 }
 // The live reporter rejects bad raw timestamps, but regression must still
 // reach the unique writer. Duplicate keeps the original pinned evidence;
 // true backward order clears it immediately without guessing a new epoch.
 {auto t=stationary();const auto s=t.stationaryEvidence(10.80);const auto g=reentry(s);
  d1monitor::McReport reporter;reporter.validate();reporter.robot(0.);assert(reporter.request_due(true,false,1.));
  const float zero[3]={0.,0.,0.};assert(reporter.sample(1.,10.80,s.mc_raw_stamp_ns,zero,zero,1.));
  assert(reporter.sourceOrder(s.mc_raw_stamp_ns)==d1monitor::McReport::SourceOrder::Duplicate);
  assert(!reporter.sample(1.02,10.82,s.mc_raw_stamp_ns,zero,zero,1.02));
  assert(t.stationaryEvidence(10.82).usable); // ignored duplicate did not refresh source or break control
  assert(reporter.sourceOrder(s.mc_raw_stamp_ns-20000000ULL)==d1monitor::McReport::SourceOrder::Backward);
  t.mcSourceRegressed(10.84,"mc-clock"); // live worker forwards this event BEFORE sample rejects it
  assert(!reporter.sample(1.04,10.84,s.mc_raw_stamp_ns-20000000ULL,zero,zero,1.04));
  assert(!t.stationaryEvidence(10.84).usable&&!t.handoff(g,10.84)&&!t.mcSourceFresh(10.84,"mc-clock"));
  assert(t.report(10.84).mc_raw_stamp_ns==s.mc_raw_stamp_ns&&t.report(10.84).source_stamp==s.source_stamp);
  // Receipt is genuinely later than acquisition. Do not let round-off in
  // independently calculated equal doubles accidentally turn this recovery
  // fixture into a future-source test (future sources must remain rejected).
  for(unsigned i=0;i<=32;++i){const double now=10.861+i*.02;const uint64_t raw=s.mc_raw_stamp_ns+(i+3)*20000000ULL;
   assert(reporter.sample(1.06+i*.02,now,raw,zero,zero,1.06+i*.02));
   t.mc(raw,reporter.stamp_unix,now,0.,0.,now,"mc-clock");
   auto hold=permit1();hold.sequence=40+i;hold.allowed=false;hold.phase="holding";hold.source_stamp=stamp(now);hold.valid_until=stamp(now+.25);
   assert(t.permit(hold,now));auto h=health();h.mc_fresh=t.mcSourceFresh(now,"mc-clock");assert(t.tick(now,h)->linear.x==0.);
   if(i<30)assert(!h.mc_fresh);
  }
  const double after_receipt=11.502;
  assert(t.mcSourceFresh(after_receipt,"mc-clock"));assert(!t.stationaryEvidence(after_receipt).usable);
  const auto state=t.state(after_receipt);assert(state.writer_commit_sequence==1&&state.sdk_arm_generation==1&&state.control_owned&&!state.fault_latched);
 }
 // Transport also enforces raw order for mock/direct MC input; equality is
 // harmless, whereas a different clock identity remains a hard fault.
 {auto t=stationary();const auto s=t.stationaryEvidence(10.80);
  t.mc(s.mc_raw_stamp_ns,10.82,10.82,0.,0.,10.82,"mc-clock");assert(t.stationaryEvidence(10.82).usable);
  t.mc(s.mc_raw_stamp_ns-1,10.83,10.83,0.,0.,10.83,"mc-clock");assert(!t.mcSourceFresh(10.83,"mc-clock"));
  assert(!t.stationaryEvidence(10.83).usable&&!t.stopping());
  t.mc(s.mc_raw_stamp_ns+1,10.84,10.84,0.,0.,10.84,"different-clock");assert(t.stopping()&&t.state(10.84).fault_latched);
 }
 // Old command is not retired merely because a conditional grant arrives.
 {auto t=active();const auto g=grant();assert(t.handoff(g,10.02));assert(t.tick(10.025,health())->linear.x==.2);
  const auto p=prepared(g);evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  assert(t.localState(body(10.035,.005),10.035));assert(t.tick(10.04,health())->linear.x==.2);
  assert(t.state(10.04).writer_commit_sequence==1); // selected is not yet called
  t.writeCalled(10.04,true);const auto a=t.takeCommitAck();assert(a&&a->applied&&a->commit_sequence==2&&a->previous_commit_sequence==1);
  assert(a->handoff_id==g.handoff_id&&a->motion_validation_sequence==p.demand.motion_validation_sequence);
  assert(a->entry_admission_sequence==p.entry_admission.sequence&&a->entry_source_stamp==p.entry_source_stamp);
  assert(a->demand_body_source_stamp==p.demand.body_source_stamp&&a->body_source_stamp==stamp(10.035));
  assert(a->measured_pose.pose.position.x==.005&&a->measured_pose.header.stamp==a->body_source_stamp);
  assert(t.state(10.04).applied_trajectory_id==2&&t.state(10.04).version==g.candidate.version);
  auto old=permit1();old.sequence=3;old.source_stamp=stamp(10.05);assert(!t.permit(old,10.05));
  assert(t.tick(10.05,health())->linear.x==.2);assert(t.handoff(g,10.06));const auto duplicate=t.takeCommitAck();
  assert(duplicate&&duplicate->applied_at==a->applied_at&&duplicate->valid_until==a->valid_until);
  t.writeAcknowledged(2,10.07);const auto confirmed=t.takeCommitAck();assert(confirmed&&confirmed->write_acknowledged);
  assert(confirmed->sequence>a->sequence&&confirmed->applied_at==a->applied_at&&confirmed->valid_until==a->valid_until);
 }
 // Real graph ordering: tracker emits candidate 356 and incumbent 357 in
 // the same cycle. Native candidate proof arrives AFTER old command 357.
 // The fixed grant floor and each role's own ordering must remain separate.
 for(bool incumbent_first:{false,true}){auto t=active();auto g=grant();g.candidate.version=g.incumbent.version;
  assert(t.handoff(g,10.02));
  auto p=prepared(g);p.demand.sequence=356;p.demand.motion_validation_sequence=60;
  const auto incumbent=command(permit1(),10.03,357);
  const auto accept_old=[&](){assert(t.motionValidation(sweep(incumbent,10.03),10.03));assert(t.demand(incumbent,10.03));};
  if(incumbent_first)accept_old();
  evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  if(!incumbent_first)accept_old();
  assert(t.preparedDemand(p,10.031)); // exact duplicate, no deadline renewal
  auto older=p;older.demand.sequence=355;assert(!t.preparedDemand(older,10.031));
  assert(t.tick(10.04,health())->linear.x==.2);t.writeCalled(10.04,true);const auto a=t.takeCommitAck();
  assert(a&&a->applied&&a->commit_sequence==2&&a->demand_sequence==356&&a->motion_validation_sequence==60);
  assert(!t.demand(incumbent,10.041)); // old role cannot revive after CAS
  auto upgrade=p.demand;upgrade.motion_validation_sequence=61;auto improved=sweep(p,10.045);improved.sequence=61;
  assert(t.motionValidation(improved,10.045));assert(t.demand(upgrade,10.045));
  assert(!t.preparedDemand(p,10.045)); // retired transaction is not reusable
 }
 // A candidate at or below the frozen grant floor remains a replay. Later
 // incumbent work cannot move the frozen floor, but expired grants still do.
 {auto t=active();const auto old=command(permit1(),10.02,355);assert(t.motionValidation(sweep(old,10.02),10.02));
  assert(t.demand(old,10.02));const auto g=grant();assert(t.handoff(g,10.02));auto p=prepared(g);
  p.demand.sequence=355;p.demand.motion_validation_sequence=60;evidence(t,g,p,10.03);assert(!t.preparedDemand(p,10.03));
  p.demand.sequence=356;assert(t.localState(body(10.031),10.031));
  assert(t.motionValidation(sweep(p,10.031),10.031));assert(t.preparedDemand(p,10.031));
  t.tick(10.23,health());const auto a=t.takeCommitAck();assert(a&&!a->applied&&a->permit_sequence==g.candidate.sequence);
  assert(a->commit_sequence==1&&a->previous_commit_sequence==1&&!t.preparedDemand(p,10.23));
 }
 // Missing command proof does not become permission; old proof/source still
 // expires naturally. A fixed same-id retransmission cannot renew grant TTL.
 {auto t=active();auto g=grant();g.transition_deadline=g.valid_until=stamp(10.08);assert(t.handoff(g,10.02));
  assert(t.handoff(g,10.04));auto changed=g;changed.valid_until=stamp(10.2);assert(!t.handoff(changed,10.04));
  assert(t.tick(10.05,health())->linear.x==.2);t.tick(10.09,health());const auto a=t.takeCommitAck();
  assert(a&&!a->applied&&a->commit_sequence==1&&a->reason=="handoff_deadline_expired"&&a->permit_sequence==g.candidate.sequence);
  assert(t.handoff(g,10.1));assert(t.takeCommitAck()->applied_at==a->applied_at);
 }
 // Latest actual entry, not the old preparation's zero-error declaration,
 // decides whether the candidate can replace the still-safe incumbent.
 for(int kind=0;kind<3;++kind){auto t=active();const auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);
  assert(t.preparedDemand(p,10.03));auto latest=body(10.04);if(kind==0)latest.local_odometry.pose.pose.position.x=.02;
  if(kind==1)latest.local_odometry.pose.pose.position.z=.02;if(kind==2)latest.local_odometry.twist.twist.linear.x=.26;
  assert(t.localState(latest,10.04));assert(t.tick(10.05,health())->linear.x==.2);t.writeCalled(10.05,true);
  assert(t.state(10.05).writer_commit_sequence==1&&!t.takeCommitAck());
 }
 for(int kind=0;kind<8;++kind){auto t=active();const auto g=grant();auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);
  if(kind==0)++p.expected_commit_sequence;if(kind==1)++p.grant_sequence;if(kind==2)++p.entry_admission.sequence;
  if(kind==3)p.entry_admission.accepted=false;if(kind==4)p.demand.velocity.linear.x=.31;
  if(kind==5)p.curve_entry_pose.position.x=.03;if(kind==6)p.measured_pose.header.stamp=stamp(10.031);
  if(kind==7)p.demand.body_source_stamp=stamp(10.031);
  const bool admitted=t.preparedDemand(p,10.03);if(kind!=2)assert(!admitted);
  t.tick(10.04,health());t.writeCalled(10.04,true);assert(t.state(10.04).writer_commit_sequence==1);
 }
 // A later invalid sweep is a fence, even if another later positive appears.
 {auto t=active();const auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  auto bad=sweep(p,10.035);bad.sequence=3;bad.valid=false;assert(t.motionValidation(bad,10.035));
  auto positive=sweep(p,10.04);positive.sequence=4;assert(t.motionValidation(positive,10.04));
  assert(t.tick(10.05,health())->linear.x==.2);t.writeCalled(10.05,true);assert(t.state(10.05).writer_commit_sequence==1);
 }
 // An intervening invalid geometry revision fences the fixed conditional
 // authorization, even if the later positive arrives before the negative.
 for(bool delayed_invalid:{false,true}){auto t=active();const auto g=grant();const auto p=prepared(g);
  assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  auto bad=curve(g.candidate,10.035);bad.sequence=3;bad.valid=false;
  auto positive=curve(g.candidate,10.04);positive.sequence=4;
  if(delayed_invalid){assert(t.trajectoryValidation(positive,10.04));assert(t.trajectoryValidation(bad,10.045));}
  else{assert(t.trajectoryValidation(bad,10.035));assert(t.trajectoryValidation(positive,10.04));}
  t.tick(10.05,health());t.writeCalled(10.05,true);assert(t.state(10.05).writer_commit_sequence==1);
 }
 // Fixed-grant revocation is processed before duplicate-sequence rejection.
 {auto t=active();auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  g.revoked=true;g.reason="owner_hold";assert(t.handoff(g,10.035));const auto a=t.takeCommitAck();assert(a&&!a->applied);
  assert(t.tick(10.04,health())->linear.x==0.); // owner's HOLD/revoke is an immediate authority fence
 }
 for(int kind=0;kind<4;++kind){auto t=active();const auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  if(kind==0){auto hold=permit1();hold.sequence=3;hold.allowed=false;hold.source_stamp=stamp(10.035);assert(t.permit(hold,10.035));}
  if(kind==1){auto h=health();h.owned=false;t.tick(10.035,h);}
  if(kind==2){auto h=health();h.mc_fresh=false;t.tick(10.035,h);}
  if(kind==3)t.beginShutdown(10.035);
  t.tick(10.04,health());t.writeCalled(10.04,true);assert(t.state(10.04).writer_commit_sequence==1);
 }
 // Calling the writer is irreversible identity advancement; write failure
 // stops the NEW identity and cannot roll back to its incumbent command.
 {auto t=active();const auto g=grant();const auto p=prepared(g);assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  t.tick(10.04,health());t.writeCalled(10.04,false);const auto a=t.takeCommitAck();
  assert(a&&a->applied&&!a->write_submitted&&!a->write_acknowledged&&a->commit_sequence==2);
  assert(t.stopping()&&t.state(10.04).applied_trajectory_id==2);assert(t.tick(10.05,health())->linear.x==0.);
 }
 // A zero candidate still needs real native sweep + full geometry + actual
 // boundary. It must not use the ordinary zero-demand safety fast path.
 {auto t=active();const auto g=grant();auto p=prepared(g);p.demand.velocity.linear.x=0;p.demand.hold=true;
  assert(t.handoff(g,10.02));evidence(t,g,p,10.03);assert(t.preparedDemand(p,10.03));
  assert(t.tick(10.04,health())->linear.x==0.);t.writeCalled(10.04,true);assert(t.takeCommitAck()->commit_sequence==2);
 }
 // A remaining-curve first commit needs exact fresh measured XYZ progress;
 // it cannot borrow the zero-demand fast path or an unrelated floor/branch.
 for(int invalid=0;invalid<5;++invalid){auto t=armed();auto c=curve(permit1(),10.01);c.sequence=2;
  c.whole_curve=false;c.remaining_curve=true;c.checked_from_time=.1;c.valid_start_time=.2;c.reverse_margin_m=.15;
  assert(t.trajectoryValidation(c,10.01));assert(t.localState(body(10.02),10.02));
  Progress p;p.schema_version=2;p.header=body(10.02).local_odometry.header;p.session_id="nav";p.task_id="task";p.route_id="route";
  p.route_hash=version1().route_hash;p.map_version_id="map";p.localization_epoch=1;p.localization_seed_id="seed";
  p.generation=p.anchor_revision=p.context_sequence=1;p.trajectory_id=1;p.segment_id="floor";p.anchor_id="anchor";
  p.curve_time=p.arc_length=.3;p.pose.orientation.w=1.;p.twist.linear.x=.2;p.valid=true;
  if(invalid==1)p.trajectory_id=2;if(invalid==2)p.pose.position.z=.02;if(invalid==3)p.curve_time=.01;
  if(invalid==4)p.header.stamp=stamp(9.9);
  const bool received=t.trackingProgress(p,10.02);if(invalid==4)assert(!received);else assert(received);
  const auto out=t.tick(10.03,health());t.writeCalled(10.03,true);
  if(invalid==0){assert(out->linear.x==.2);assert(t.takeCommitAck()->commit_sequence==1);}
  else {assert(out->linear.x==0.);assert(!t.takeCommitAck()&&t.state(10.03).writer_commit_sequence==0);}
 }
 // SDK compares an odom entry velocity to a correctly rotated child-frame
 // twist, not to the unrotated body's forward axis.
 {auto t=active();const auto g=grant();auto p=prepared(g);p.measured_twist.linear.x=p.curve_entry_twist.linear.x=0.;
  p.measured_twist.linear.y=p.curve_entry_twist.linear.y=.2;assert(t.handoff(g,10.02));
  auto b=body(10.03);b.local_odometry.pose.pose.orientation.w=b.local_odometry.pose.pose.orientation.z=std::sqrt(.5);
  assert(t.localState(b,10.03));assert(t.trajectoryValidation(curve(g.candidate,10.03),10.03));
  assert(t.motionValidation(sweep(p,10.03),10.03));assert(t.preparedDemand(p,10.03));
  assert(t.tick(10.04,health())->linear.x==.2);t.writeCalled(10.04,true);assert(t.takeCommitAck()->commit_sequence==2);
 }
 std::cout<<"bounded writer handoff positive/negative assertions passed\n";
}
