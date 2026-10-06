#include "execution_transport_core.hpp"
#include "test_execution_fixture.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
using namespace d1monitor::execution3;
static Version v(){Version x;x.schema_version=3;x.session_id="nav";x.task_id="task";x.route_id="route";
 x.route_hash=std::string(64,'a');x.map_version_id="map";x.localization_epoch=1;x.localization_seed_id="seed";
 x.reference_generation=1;x.segment_id="floor";x.anchor_id="anchor";x.anchor_revision=x.context_sequence=x.map_geometry_revision=1;return x;}
static Health h(){return {true,true,true,true,false};}
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
static Transport ready(const ExecutionPolicy&policy=testExecutionPolicy(),double command_speed=.3,double command_yaw=.5){Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
 t.configureExecutionPolicy(policy,.8);
 t.configureBrakingModel(std::string(64,'c'),command_speed,command_yaw);assert(t.grant(g(),10,h()).accepted);
 t.tick(10,h());assert(t.permit(p(),10));assert(t.motionValidation(proof(d()),10));
 Local local;local.schema_version=1;local.session_id="nav";local.map_version_id="map";local.localization_epoch=1;local.localization_seed_id="seed";
 local.source_stamp=local.posterior_stamp=local.imu_stamp=stamp(10.);local.usable=true;
 local.local_odometry.header.stamp=local.source_stamp;local.local_odometry.header.frame_id="d1max_loc_odom";
 local.local_odometry.child_frame_id="d1max_loc_base_link";local.local_odometry.pose.pose.orientation.w=1.;assert(t.localState(local,10.));
 CurveProof curve;curve.version=v();curve.trajectory_id=curve.sequence=1;curve.transport_mode="isolated_mock";
 curve.check_begin=curve.check_end=curve.source_stamp=curve.body_source_stamp=curve.front_ray_source_stamp=curve.rear_ray_source_stamp=stamp(10.);
 curve.valid_until=stamp(10.25);curve.valid=curve.whole_curve=true;curve.curve_duration=curve.checked_to_time=1.;
 curve.frame_id="d1max_loc_odom";curve.collision_policy="observed_free";curve.map_snapshot_revision=1;
 curve.support_reference_id="support";curve.support_hash=std::string(64,'d');assert(t.trajectoryValidation(curve,10.));return t;}
static void refreshBody(Transport&t,double now) {
 Local m;m.schema_version=1;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(now);m.usable=true;m.local_odometry.header.stamp=m.source_stamp;
 m.local_odometry.header.frame_id="d1max_loc_odom";m.local_odometry.child_frame_id="d1max_loc_base_link";
 m.local_odometry.pose.pose.orientation.w=1.;assert(t.localState(m,now));
 CurveProof c;c.version=v();c.trajectory_id=1;c.sequence=static_cast<uint64_t>(now*1e6);c.transport_mode="isolated_mock";
 c.check_begin=c.check_end=c.source_stamp=c.body_source_stamp=c.front_ray_source_stamp=c.rear_ray_source_stamp=stamp(now);
 c.valid_until=stamp(now+.25);c.valid=c.whole_curve=true;c.curve_duration=c.checked_to_time=1.;c.map_snapshot_revision=1;
 c.frame_id="d1max_loc_odom";c.collision_policy="observed_free";c.support_reference_id="support";c.support_hash=std::string(64,'d');
 assert(t.trajectoryValidation(c,now));
}
int main(){
 // A plant's larger measured reachable bound must never reach writer caps.
 {auto t=ready(testExecutionPolicy(),.15,.3);assert(!t.demand(d(),10.));
  auto within=d();within.sequence=within.motion_validation_sequence=2;within.velocity.linear.x=.15;
  assert(t.motionValidation(proof(within,10.,2),10.));assert(t.demand(within,10.));
  assert(t.tick(10.01,h())->linear.x==.15);
  auto above=within;above.sequence=above.motion_validation_sequence=3;above.velocity.linear.x=.1501;
  assert(t.motionValidation(proof(above,10.,3),10.));assert(!t.demand(above,10.));
  above.velocity.linear.x=.1;above.velocity.angular.z=.3001;above.sequence=above.motion_validation_sequence=4;
  assert(t.motionValidation(proof(above,10.,4),10.));assert(!t.demand(above,10.));
  assert(!t.report(10.01).physical_acceptance_verified);
  for(const auto*mode:{"isolated_mock","live"}) {
   Transport guarded("sdk",mode);bool rejected=false;
   try{guarded.configureBrakingModel(std::string(64,'c'),.6,.8);}catch(const std::invalid_argument&){rejected=true;}
   assert(rejected);
  }}
 // A real Unix-epoch source equal to ROS time used to fail grant admission:
 // sec + nanosec*1e-9 rounded above ns/1e9. Exact clocks also distinguish a
 // genuinely future or expired sample by one nanosecond, without tolerance.
 {constexpr std::int64_t ns=1791124691902856036LL;
  const auto now=SourceClock::fromNanoseconds(ns);const auto exact=stamp(now);
  const double old_source=exact.sec+exact.nanosec*1e-9;
  assert(old_source>static_cast<double>(now));
  assert(stamp(secs(exact))!=exact);assert(nanoseconds(exact)==ns);
  for(const auto delta:{0LL,500000000LL,500000001LL,-1LL}){
   Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
   t.configureExecutionPolicy(testExecutionPolicy(),.8);t.configureBrakingModel(std::string(64,'c'));
   auto request=g();request.source_stamp=stamp(SourceClock::fromNanoseconds(ns-delta));
   const auto reply=t.grant(request,now,h());
   assert(reply.accepted==(delta>=0&&delta<=500000000));
   if(!reply.accepted)assert(reply.reason=="grant_binding_or_time_invalid");
  }
  auto exact_now=SourceClock::fromNanoseconds(ns);
  assert(nanoseconds(stamp(after(exact_now,.25)))==ns+250000000);
  Transport t("sdk","isolated_mock");t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
  t.configureExecutionPolicy(testExecutionPolicy(),.8);t.configureBrakingModel(std::string(64,'c'));
  auto request=g();request.source_stamp=exact;assert(t.grant(request,exact_now,h()).accepted);
  auto permit=p();permit.source_stamp=exact;permit.valid_until=stamp(after(exact_now,.25));
  assert(t.permit(permit,exact_now));assert(t.state(exact_now).source_stamp==exact);
  Local m;m.schema_version=1;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;
  m.localization_seed_id="seed";m.usable=true;m.source_stamp=m.posterior_stamp=m.imu_stamp=exact;
  m.local_odometry.header.stamp=exact;m.local_odometry.header.frame_id="d1max_loc_odom";
  m.local_odometry.child_frame_id="d1max_loc_base_link";m.local_odometry.pose.pose.orientation.w=1.;
  assert(t.localState(m,exact_now));
  ++m.source_stamp.nanosec;m.posterior_stamp=m.imu_stamp=m.local_odometry.header.stamp=m.source_stamp;
  assert(!t.localState(m,exact_now));
  assert(!fresh(exact,SourceClock::fromNanoseconds(ns+100000001),.1));
  assert(fresh(exact,SourceClock::fromNanoseconds(ns+100000000),.1));
 }
 // A tighter physical source-age record must reach the FINAL writer, not
 // merely the Python/native validators. A later proof check cannot renew it.
 {auto policy=testExecutionPolicy();policy.sensor_source_age_bound_s=.2;auto t=ready(policy);
  auto command=d();command.sequence=command.motion_validation_sequence=2;command.safety_source_stamp=stamp(9.7);
  auto old=proof(command,10.,2);old.front_ray_source_stamp=old.rear_ray_source_stamp=stamp(9.7);
  assert(t.motionValidation(old,10.));assert(!t.demand(command,10.));assert(t.tick(10.,h())->linear.x==0.);}
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
  t.mc(10010000000ULL,10.01,10.,0,0,10.,"sdk:7"); // future mapped source
  assert(!t.mcSourceFresh(10.,"sdk:7"));assert(t.report(10.).mc_raw_stamp_ns==0);
  t.mc(9000000000ULL,9.,10.,0,0,10.,"sdk:7"); // fresh arrival, expired capture
  assert(!t.mcSourceFresh(10.,"sdk:7"));assert(t.report(10.).mc_raw_stamp_ns==0);
  t.mc(10070000000ULL,10.07,10.1,0,0,10.1,"sdk:7");
  assert(t.mcSourceFresh(10.1,"sdk:7"));assert(!t.mcSourceFresh(10.1,"sdk:8"));
  assert(!t.mcSourceFresh(10.1,""));
  assert(secs(t.report(10.1).source_stamp)==10.07&&t.report(10.1).mc_raw_stamp_ns==10070000000ULL);
  assert(std::abs(t.mcCaptureLowerBound()-10.05)<1e-12&&t.mcCaptureUpperBound()==10.1);
  // Neither future/late data nor replayed raw identities refresh the watermark.
  t.mc(10410000000ULL,10.41,10.4,0,0,10.4,"sdk:7");
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
 {auto t=ready();assert(t.demand(d(),10));t.tick(10.01,h());t.telemetryRestartBeforeFirstMotion(10.1);
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
  auto request=g();request.transport_mode="live";assert(t.grant(request,10.,h()).reason=="measured_execution_policy_unavailable");}
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
 std::cout<<"schema3 execution transport positive/negative assertions passed\n";
}
