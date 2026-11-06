#include <gtest/gtest.h>
#include <array>
#include <limits>
#include "d1max_navigation_bt/execution_owner.hpp"
#include "d1max_navigation_bt/body_pair_admission.hpp"
#include "d1max_navigation_bt/preparation_budget.hpp"
#include "d1max_navigation_bt/engine.hpp"
#include "d1max_navigation_bt/task_contract.hpp"
#include "d1max_navigation_bt/runtime_policy.hpp"
using d1max_navigation_bt::PreparationBudget;
using d1max_navigation_bt::PreparationLimits;
using d1max_navigation_bt::PreparationStage;
using d1max_navigation_bt::preparationStage;
using namespace d1max_navigation_bt::exec3;
namespace {
Version ver(){Version v;v.schema_version=3;v.session_id="nav";v.task_id="task";v.route_id="route";v.route_hash=std::string(64,'a');
 v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";v.reference_generation=1;v.segment_id="floor";
 v.anchor_id="anchor";v.anchor_revision=1;v.context_sequence=1;return v;}
Validation proof(double t=10.,uint64_t seq=1,int64_t curve=1){Validation p;p.version=ver();p.proposal_id="proposal";p.trajectory_id=curve;p.sequence=seq;
 p.frame_id="d1max_loc_odom";p.collision_policy="official_inflated_geometry";
 p.source_stamp=p.check_begin=p.check_end=p.body_source_stamp=p.front_ray_source_stamp=p.rear_ray_source_stamp=stamp(t);
 p.valid_until=stamp(t+.4);
 p.checked_to_time=p.curve_duration=1.;
 p.valid=p.whole_curve=true;p.support_reference_id="support";p.support_hash="hash";p.transport_mode="isolated_mock";return p;}
Admission ack(double t=10.,uint64_t seq=1,int64_t curve=1){Admission a;a.version=ver();a.trajectory_id=curve;a.validation_sequence=seq;
 a.sequence=seq;
 a.body_source_stamp=a.checked_at=stamp(t);a.valid_until=stamp(t+.4);a.accepted=true;a.transport_mode="isolated_mock";return a;}
SDKState sdk(double t=10.1,uint64_t seq=1){SDKState s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";
 s.sdk_arm_generation=1;s.sequence=seq;s.source_stamp=stamp(t);s.grant_ready=s.control_owned=s.general_low_speed_confirmed=true;
 s.transport_mode="isolated_mock";return s;}
Receipt receipt(){Receipt r;r.version=ver();r.proposal_id="proposal";r.expected_trajectory_id=-1;
 r.source_stamp=stamp(10.);r.valid_until=stamp(10.5);r.accepted=true;r.transport_mode="isolated_mock";return r;}
d1max_navigation_bt_interfaces::msg::RouteSnapshot routeSnapshot(std::vector<std::array<double,3>> points={{0.,0.,0.},{100.,0.,0.}}){
 d1max_navigation_bt_interfaces::msg::RouteSnapshot r;r.schema_version=2;r.session_id="nav";r.task_id="task";
 r.route_id="route";r.route_hash=std::string(64,'a');r.map_version_id="map";r.localization_epoch=1;r.localization_seed_id="seed";
 r.frame_id=r.path.header.frame_id="d1max_loc_map";r.point_reference="ground";
 for(const auto&p:points){geometry_msgs::msg::PoseStamped s;s.pose.position.x=p[0];s.pose.position.y=p[1];s.pose.position.z=p[2];
  s.pose.orientation.w=1.;r.path.poses.push_back(s);}
 r.edge_segments.assign(points.size()-1,"floor");d1max_navigation_bt_interfaces::msg::RouteSegment segment;
 segment.segment_id="floor";segment.begin_index=0;segment.end_index=points.size()-1;r.segments.push_back(segment);return r;
}
void prepare(Owner&o,const std::string&transport="isolated_mock"){
 o.bind(ver());ASSERT_TRUE(o.bindRoute(routeSnapshot()));auto r=receipt();r.transport_mode=transport;
 auto v=proof();v.transport_mode=transport;auto a=ack();a.transport_mode=transport;
 ASSERT_TRUE(o.observe(r,10.));ASSERT_TRUE(o.observe(v,10.));ASSERT_TRUE(o.observe(a,10.));}
void beginUnapplied(Owner&o,const std::string&transport="isolated_mock"){
 prepare(o,transport);ASSERT_TRUE(o.begin("execution","confirm",1,10.));
 auto s=sdk();s.transport_mode=transport;ASSERT_TRUE(o.observe(s,10.1));}
MotionDemand movingDemand(double t,uint64_t seq){MotionDemand d;d.version=ver();d.execution_id="execution";
 d.control_epoch=1;d.sdk_session="sdk";d.sdk_arm_generation=1;d.sequence=seq;d.permit_sequence=seq;
 d.validation_sequence=seq;d.motion_validation_sequence=seq;d.braking_model_sha256=std::string(64,'b');
 d.trajectory_id=1;d.source_stamp=d.body_source_stamp=stamp(t);d.valid_until=stamp(t+.2);
 d.velocity.linear.x=.2;d.safety_checked=true;d.transport_mode="isolated_mock";return d;}
MotionValidation motionProof(const MotionDemand&d){MotionValidation m;m.version=d.version;m.execution_id=d.execution_id;
 m.control_epoch=d.control_epoch;m.sdk_session=d.sdk_session;m.sdk_arm_generation=d.sdk_arm_generation;
 m.trajectory_id=d.trajectory_id;m.permit_sequence=d.permit_sequence;m.trajectory_validation_sequence=d.validation_sequence;
 m.demand_sequence=d.sequence;m.demand_source_stamp=d.source_stamp;m.demand_valid_until=d.valid_until;
 m.demand_body_source_stamp=m.body_source_stamp=d.body_source_stamp;m.sequence=d.motion_validation_sequence;
 m.front_ray_source_stamp=m.rear_ray_source_stamp=m.check_begin=m.check_end=d.source_stamp;m.valid_until=d.valid_until;
 m.velocity=d.velocity;m.frame_id="d1max_loc_odom";m.valid=true;m.transport_mode=d.transport_mode;
 m.braking_model_sha256=d.braking_model_sha256;return m;}
NavigationState body(double t,double x=0.,double yaw=0.){NavigationState b;b.schema_version=2;b.session_id="nav";
 b.map_version_id="map";b.localization_epoch=1;b.localization_seed_id="seed";b.usable=true;b.source_stamp=stamp(t);
 b.local_odometry.header.stamp=b.source_stamp;b.local_odometry.header.frame_id="d1max_loc_odom";
 b.local_odometry.child_frame_id="d1max_loc_base_link";
 b.local_odometry.pose.pose.position.x=x;b.local_odometry.pose.pose.orientation.w=std::cos(yaw*.5);
 b.local_odometry.pose.pose.orientation.z=std::sin(yaw*.5);return b;}
RouteProgress routeProgress(double t,uint64_t seq,double x=0.,double y=0.) {
 RouteProgress r;r.schema_version=1;r.version=ver();r.sequence=seq;r.frame_id="d1max_loc_map";r.transport_mode="isolated_mock";
 r.body_source_stamp=stamp(t);r.odom_body_pose.header=body(t,x).local_odometry.header;
 r.odom_body_pose.pose=body(t,x).local_odometry.pose.pose;r.odom_body_pose.pose.position.y=y;
 r.anchor_source_stamp=stamp(9.);r.map_from_odom.position.z=.5;r.map_from_odom.orientation.w=1.;
 r.source_map_body_xyz.x=x;r.source_map_body_xyz.y=y;r.source_map_body_xyz.z=.5;r.body_reference_height_m=.5;
 r.measured_arc_m=r.confirmed_arc_m=x;r.cross_track_m=std::abs(y);return r;
}
void feedback(Owner&o,double t,uint64_t seq,double x=0.){const auto d=movingDemand(t,seq);
 ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(motionProof(d),t));ASSERT_TRUE(o.observe(routeProgress(t,seq,x),t));}
CommitAck initialCommit(Owner&o,Permit p,double time=10.11) {
 p.frame_id="d1max_loc_odom";o.notePublication(p,seconds(p.source_stamp));
 CommitAck a;a.schema_version=1;a.sequence=1;a.commit_sequence=1;
 a.candidate_version=p.version;a.candidate_trajectory_id=p.trajectory_id;
 a.execution_id=p.execution_id;a.control_epoch=p.control_epoch;a.sdk_session=p.sdk_session;
 a.sdk_arm_generation=p.sdk_arm_generation;a.transport_mode=p.transport_mode;a.permit_sequence=p.sequence;
 a.demand_sequence=a.motion_validation_sequence=1;
 a.applied_at=stamp(time);a.demand_source_stamp=a.body_source_stamp=stamp(time-.01);
 a.demand_body_source_stamp=a.body_source_stamp;
 a.valid_until=stamp(time+.09);a.measured_pose.header.frame_id=p.frame_id;
 a.measured_pose.header.stamp=a.body_source_stamp;a.measured_pose.pose.orientation.w=1.;
 a.applied=a.write_submitted=a.write_acknowledged=true;return a;
}
void begin(Owner&o,const std::string&transport="isolated_mock") {
 beginUnapplied(o,transport);const auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p,10.1),10.1));
}
GeometryReceipt geometryReceipt(const Permit&p,double at=10.1,uint64_t sequence=1,
    uint64_t installation=1,double source=10.1) {
 GeometryReceipt r;r.schema_version=1;r.version=p.version;r.frame_id=p.frame_id;
 r.transport_mode=p.transport_mode;r.trajectory_id=p.trajectory_id;r.sequence=sequence;
 r.installation_sequence=installation;r.permit_sequence=p.sequence;
 r.admission_sequence=r.validation_sequence=p.validation_sequence;r.installed_at=stamp(at);
 r.body_source_stamp=stamp(source);r.installed=true;return r;
}
GeometryReceipt installedActual(Owner&o) {
 beginUnapplied(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 EXPECT_TRUE(o.observe(initialCommit(o,p,10.1),10.1));
 auto r=geometryReceipt(p,10.1,1,1,10.12);EXPECT_TRUE(o.observe(r,10.12));return r;
}
GoalLedger::Proposal candidateGoal(const Version&v) {
 GoalLedger::Proposal p;p.version=v;p.reference.path.header.frame_id="d1max_loc_odom";
 p.goal_position.x=4.;return p;
}
CommitAck handoffCommit(const Handoff&h,double time,uint64_t sequence=2,bool applied=true) {
 CommitAck a;a.schema_version=1;a.handoff_id=h.handoff_id;a.grant_sequence=h.sequence;a.sequence=sequence;
 a.previous_commit_sequence=h.expected_commit_sequence;
 a.commit_sequence=h.expected_commit_sequence+(applied?1:0);
 a.incumbent_version=h.incumbent.version;a.incumbent_trajectory_id=h.incumbent.trajectory_id;
 a.candidate_version=h.candidate.version;a.candidate_trajectory_id=h.candidate.trajectory_id;
 a.execution_id=h.candidate.execution_id;a.control_epoch=h.candidate.control_epoch;
 a.sdk_session=h.candidate.sdk_session;a.sdk_arm_generation=h.candidate.sdk_arm_generation;
 a.transport_mode=h.candidate.transport_mode;a.permit_sequence=h.candidate.sequence;
 a.demand_sequence=20;a.motion_validation_sequence=30;a.entry_admission_sequence=40;
 a.applied_at=stamp(time);a.body_source_stamp=a.demand_source_stamp=stamp(time-.01);
 a.entry_source_stamp=a.demand_body_source_stamp=a.body_source_stamp;
 a.valid_until=stamp(time+.09);a.measured_pose.header.frame_id=h.candidate.frame_id;
 a.measured_pose.header.stamp=a.body_source_stamp;a.measured_pose.pose.orientation.w=1.;
 a.applied=applied;a.write_submitted=a.write_acknowledged=applied;return a;
}
Stationary stationaryWitness(double t=11.,uint64_t seq=1,const std::string&transport="isolated_mock") {
 Stationary s;s.schema_version=1;s.version=ver();s.execution_id="execution";s.control_epoch=1;
 s.sdk_session="sdk";s.sdk_arm_generation=1;s.transport_mode=transport;
 s.sequence=seq;s.writer_commit_sequence=1;s.applied_trajectory_id=1;s.zero_write_sequence=3;
 s.zero_ack_at=stamp(10.2);s.mc_raw_stamp_ns=static_cast<uint64_t>(t*1e9);s.mc_clock_epoch="mc-clock";
 s.time_basis=transport=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
 s.source_stamp=stamp(t);s.received_stamp=stamp(t+.01);
 s.capture_lower_bound=stamp(t-.02);s.capture_upper_bound=s.received_stamp;s.mc_capture_delay_bound_sec=.02;
 s.valid_until=stamp(t+.25);s.stationary_samples=35;s.stationary_duration_sec=.65;
 s.measured_linear_mps=.005;s.measured_angular_radps=.002;s.nonzero_blocked=s.usable=true;return s;
}
Stop stopWitness(const std::string&transport="isolated_mock") {
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";
 s.sdk_arm_generation=1;s.transport_mode=transport;s.stop_request_id="confirm";s.sequence=1;
 s.mc_raw_stamp_ns=11300000000ULL;s.mc_clock_epoch="clock";
 s.time_basis=transport=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
 s.source_stamp=stamp(11.3);s.stationary_samples=50;s.stationary_duration_sec=1.;
 s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;return s;
}
Retirement retirementWitness(double t=10.3,uint64_t seq=1) {
 Retirement r;r.schema_version=1;r.version=ver();r.execution_id="execution";r.control_epoch=1;
 r.sdk_session="sdk";r.sdk_arm_generation=1;r.grant_request_id="confirm";r.sequence=seq;
 r.source_stamp=stamp(t);r.transport_mode="isolated_mock";
 r.nonzero_blocked=r.software_retired=r.motion_never_submitted=true;r.reason="take_control_failed";return r;
}
Grant::Request grantRequest() {
 Grant::Request r;r.version=ver();r.execution_id="execution";r.control_epoch=1;
 r.confirmation_id=r.request_id="confirm";r.sdk_session="sdk";r.source_stamp=stamp(10.);
 r.transport_mode="isolated_mock";r.activate=true;return r;
}
Grant::Response rejectedGrant(const Grant::Request& r) {
 Grant::Response out;out.version=r.version;out.request_id=r.request_id;out.execution_id=r.execution_id;
 out.control_epoch=r.control_epoch;out.sdk_session=r.sdk_session;out.transport_mode=r.transport_mode;
 out.definitely_not_admitted=true;out.reason="writer_not_ready_or_prior_not_stopped";return out;
}
void stationaryCandidate(Owner&o,double t=11.,const std::string&transport="isolated_mock") {
 auto s=sdk(t+.01,2);s.transport_mode=transport;ASSERT_TRUE(o.observe(s,t+.01));
 auto invalid=proof(t,2);invalid.transport_mode=transport;invalid.valid=false;invalid.reason="occupied";
 invalid.valid_until=stamp(0.);ASSERT_TRUE(o.observe(invalid,t+.01));
 auto r=receipt();r.version.reference_generation=2;r.proposal_id="stationary-reference";
 r.expected_version=ver();r.expected_trajectory_id=1;r.source_stamp=stamp(t);r.valid_until=stamp(t+.3);r.transport_mode=transport;
 ASSERT_TRUE(o.observe(r,t+.01));auto next=proof(t,3,2);next.version=r.version;next.proposal_id=r.proposal_id;next.transport_mode=transport;
 ASSERT_TRUE(o.observe(next,t+.01));auto a=ack(t,3,2);a.version=r.version;a.transport_mode=transport;ASSERT_TRUE(o.observe(a,t+.01));
}
// The production navigator uses this same execution-failure contract in its
// outer TaskContextValid gate, independent of FollowRoute's worker feedback.
struct ExecutionTaskBackend final : d1max_navigation_bt::Backend {
 explicit ExecutionTaskBackend(Owner& owner):owner(owner){}
 Owner& owner;bool inputs_ready=true;int routes=0,follows=0,follow_halts=0,finished=0;
 bool success=false;
 d1max_navigation_bt::ReadyState contextValid(const d1max_navigation_bt::TaskIdentity&) override {
  const auto failure=owner.failureReason();return {failure.empty(),failure};
 }
 d1max_navigation_bt::ReadyState inputsReady(const d1max_navigation_bt::TaskIdentity&) override {
  return {inputs_ready,inputs_ready?"ready":"waiting_navigation"};
 }
 void requestRoute(const d1max_navigation_bt::TaskIdentity&) override {++routes;}
 d1max_navigation_bt::Result pollRoute(const d1max_navigation_bt::TaskIdentity&) override {return {BT::NodeStatus::SUCCESS,"route"};}
 void haltRoute(const d1max_navigation_bt::TaskIdentity&) override {}
 void requestFollow(const d1max_navigation_bt::TaskIdentity&) override {++follows;}
 d1max_navigation_bt::Result pollFollow(const d1max_navigation_bt::TaskIdentity&) override {return {BT::NodeStatus::RUNNING,"worker_running"};}
 void haltFollow(const d1max_navigation_bt::TaskIdentity&) override {++follow_halts;}
 bool measuredGoalReached(const d1max_navigation_bt::TaskIdentity&) override {return false;}
 void setPaused(const d1max_navigation_bt::TaskIdentity&,bool,const std::string&) override {}
 void finishTask(const d1max_navigation_bt::TaskIdentity&,bool ok,const std::string&) override {++finished;success=ok;}
 void cancelTask(const d1max_navigation_bt::TaskIdentity&,const std::string&) override {}
};
}
TEST(ExecutionOwner, PreviewCommitsGeometryButNeverPermission){Owner o("isolated_mock");prepare(o);auto p=o.tick(10.1,true);
 EXPECT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.phase,"preview");EXPECT_TRUE(o.canConfirm(10.1));}

TEST(ExecutionInitialGeometry, OriginalShortLeaseCanPrepareAtCallbackButNotAtNextTenHzTimer) {
 for(bool admission_first:{false,true}) {
  Owner callback("isolated_mock"),timer_only("isolated_mock");
  for(auto*o:{&callback,&timer_only}) {
   o->bind(ver());ASSERT_TRUE(o->bindRoute(routeSnapshot()));ASSERT_TRUE(o->observe(receipt(),10.));
  }
  auto v=proof(10.02);v.valid_until=stamp(10.07);
  auto a=ack(10.03);a.valid_until=v.valid_until;
  for(auto*o:{&callback,&timer_only}) {
   if(admission_first)ASSERT_TRUE(o->observe(a,10.031));else ASSERT_TRUE(o->observe(v,10.031));
   EXPECT_FALSE(o->canConfirm(10.031));
   if(admission_first)ASSERT_TRUE(o->observe(v,10.032));else ASSERT_TRUE(o->observe(a,10.032));
  EXPECT_TRUE(o->canConfirm(10.032));
  }
  // Production's fast callback shares this same intent/Owner sequence. It
  // neither ticks the tree nor assigns another timestamp to either proof.
  d1max_navigation_bt::GoalExecutionIntent intent(true,"rviz-goal-request");
  const d1max_navigation_bt::RouteBinding binding{"nav","task","route",std::string(64,'a')};
  ASSERT_TRUE(intent.prepare(binding,true));ASSERT_TRUE(intent.consume(binding));
  EXPECT_TRUE(callback.initialGeometryNeedsPublication(10.032));
  ASSERT_TRUE(callback.begin("execution","confirm",1,10.032,"sdk"));
  EXPECT_FALSE(callback.canPrepareInitialGeometry(10.032));
  EXPECT_TRUE(intent.consumed()); // The fast path declines consumed intents before confirm.
  EXPECT_FALSE(callback.begin("execution","repeated",2,10.032,"sdk"));
  auto p=callback.tick(10.032,true);p.frame_id="d1max_loc_odom";
  EXPECT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.phase,"arming");
  EXPECT_EQ(p.version,v.version);EXPECT_EQ(p.validation_sequence,v.sequence);
  callback.notePublication(p,10.032);
  const auto installed=geometryReceipt(p,10.033,1,1,10.034);
  ASSERT_TRUE(callback.observe(installed,10.034));
  EXPECT_EQ(installed.validation_sequence,v.sequence);
  EXPECT_EQ(v.valid_until,stamp(10.07));EXPECT_EQ(a.valid_until,stamp(10.07));
  EXPECT_FALSE(timer_only.canConfirm(10.1));
  EXPECT_FALSE(timer_only.begin("execution","too-late",1,10.1));
  EXPECT_FALSE(timer_only.tick(10.1,true).geometry_committed);
  // Capturing installation is not permission after the original proof expires.
  ASSERT_TRUE(callback.observe(sdk(10.1),10.1));
  EXPECT_FALSE(callback.tick(10.1,true).allowed);
 }
}
TEST(ExecutionInitialGeometry, PlanningOnlyOriginalProofCanPrepareImmediatelyWithoutAnIntentOrGrant) {
 Owner callback("live",30.,1.,{},true),timer_only("live",30.,1.,{},true);
 for(auto*o:{&callback,&timer_only}) {
  o->bind(ver());ASSERT_TRUE(o->bindRoute(routeSnapshot()));
  auto r=receipt();r.transport_mode="live";ASSERT_TRUE(o->observe(r,10.));
  auto v=proof(10.02);v.transport_mode="live";v.valid_until=stamp(10.07);
  ASSERT_TRUE(o->observe(v,10.032));
  EXPECT_TRUE(o->canPrepareInitialGeometry(10.032));EXPECT_FALSE(o->canConfirm(10.032));
  EXPECT_FALSE(o->begin("execution","forbidden",1,10.032));
 }
 ASSERT_TRUE(callback.initialGeometryNeedsPublication(10.032));
 auto p=callback.tick(10.032,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.phase,"preview");
 callback.notePublication(p,10.032);
 EXPECT_FALSE(callback.initialGeometryNeedsPublication(10.04));
 EXPECT_FALSE(callback.confirmed());EXPECT_TRUE(callback.executionId().empty());
 EXPECT_EQ(callback.controlEpoch(),0u);
 // A genuinely new native proof is an event, not repeated local heartbeats.
 auto newer=proof(10.04,2);newer.transport_mode="live";newer.valid_until=stamp(10.09);
 ASSERT_TRUE(callback.observe(newer,10.04));
 EXPECT_TRUE(callback.initialGeometryNeedsPublication(10.04));
 EXPECT_FALSE(timer_only.canPrepareInitialGeometry(10.1));
 EXPECT_FALSE(timer_only.tick(10.1,true).geometry_committed);
 EXPECT_FALSE(callback.tick(10.1,true).allowed);
}
TEST(ExecutionInitialGeometry, FastPreparationCannotUseExpiredForeignNegativeOrIncompletePair) {
 for(int kind=0;kind<7;++kind) {
  Owner o("isolated_mock");o.bind(ver());ASSERT_TRUE(o.bindRoute(routeSnapshot()));
  auto r=receipt();auto v=proof(10.02);auto a=ack(10.03);
  if(kind==0)v.valid_until=stamp(10.031);
  if(kind==1)a.version.task_id="old-task";
  if(kind==2)v.valid=false;
  if(kind==3)a.accepted=false;
  if(kind==4)r.accepted=false;
  if(kind==5)a.validation_sequence=2;
  if(kind==6)v.transport_mode="live";
  o.observe(r,10.);o.observe(v,10.032);o.observe(a,10.032);
  EXPECT_FALSE(o.canConfirm(10.032))<<kind;
  EXPECT_FALSE(o.canPrepareInitialGeometry(10.032))<<kind;
  EXPECT_FALSE(o.initialGeometryNeedsPublication(10.032))<<kind;
  EXPECT_FALSE(o.begin("execution","confirm",1,10.032))<<kind;
  EXPECT_FALSE(o.tick(10.032,true).geometry_committed)<<kind;
 }
}
TEST(PlanningGeometryOwner, NativeWholeProofAndReferenceReceiptCommitWithoutTracker) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.bindRoute(routeSnapshot()));
 ASSERT_TRUE(o.observe(receipt(),10.));ASSERT_TRUE(o.observe(proof(),10.));
 const auto p=o.tick(10.1,true);EXPECT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);
 EXPECT_EQ(p.phase,"preview");EXPECT_EQ(p.trajectory_id,1);EXPECT_EQ(p.validation_sequence,1u);
 EXPECT_EQ(p.version,ver());EXPECT_EQ(o.preparationEvidence(10.1),10000000000LL);
 EXPECT_FALSE(o.canConfirm(10.1));EXPECT_FALSE(o.begin("execution","confirm",1,10.1));
 EXPECT_FALSE(o.confirmed());EXPECT_TRUE(o.executionId().empty());EXPECT_EQ(o.controlEpoch(),0u);
 EXPECT_FALSE(o.observe(sdk(),10.1));EXPECT_EQ(o.appliedCommitSequence(),0u);
}
TEST(ExecutionReferenceReceipt, RecoveredHistoricalAdmissionNeedsFreshExactPreparation) {
 Owner o("isolated_mock");o.bind(ver());ASSERT_TRUE(o.bindRoute(routeSnapshot()));
 const auto original=receipt();ASSERT_TRUE(o.observe(original,14.));
 EXPECT_FALSE(o.canPrepareInitialGeometry(14.)); // Historical ACK alone is never current geometry.
 ASSERT_TRUE(o.observe(proof(14.),14.));
 EXPECT_FALSE(o.canPrepareInitialGeometry(14.)); // Motion purpose still needs actual tracker entry.
 ASSERT_TRUE(o.observe(ack(14.),14.));EXPECT_TRUE(o.canPrepareInitialGeometry(14.));
 ASSERT_TRUE(o.observe(original,14.1)); // Exact replay remains immutable/idempotent.
 EXPECT_TRUE(o.canPrepareInitialGeometry(14.1));
 EXPECT_FALSE(o.canPrepareInitialGeometry(14.41)); // Replay did not renew either current proof lease.
}
TEST(ExecutionReferenceReceipt, HistoricalFactCannotOverrideNegativeProofOrRetiredTask) {
 Owner o("isolated_mock");o.bind(ver());ASSERT_TRUE(o.observe(receipt(),14.));
 ASSERT_TRUE(o.observe(proof(14.),14.));ASSERT_TRUE(o.observe(ack(14.),14.));
 EXPECT_TRUE(o.canPrepareInitialGeometry(14.));
 auto blocked=proof(14.1,2);blocked.valid=false;blocked.reason="trajectory_occupied";
 ASSERT_TRUE(o.observe(blocked,14.1));ASSERT_TRUE(o.observe(receipt(),14.1));
 EXPECT_FALSE(o.canPrepareInitialGeometry(14.1));
 o.stop("cancelled",14.2);EXPECT_FALSE(o.observe(receipt(),14.2));
 EXPECT_FALSE(o.canPrepareInitialGeometry(14.2));
}
TEST(ExecutionReferenceReceipt, OriginalAdmissionIdentityTimeAndPredecessorStayBinding) {
 for(int kind=0;kind<7;++kind) {
  Owner o("isolated_mock");o.bind(ver());auto r=receipt();
  if(kind==0)r.version.task_id="old-task";
  if(kind==1)++r.version.localization_epoch;
  if(kind==2)r.version.localization_seed_id="old-seed";
  if(kind==3)r.source_stamp=stamp(14.01);
  if(kind==4)r.valid_until=stamp(9.9);
  if(kind==5)r.valid_until=stamp(10.501);
  if(kind==6)r.accepted=false;
  EXPECT_FALSE(o.observe(r,14.))<<kind;
  ASSERT_TRUE(o.observe(proof(14.),14.));ASSERT_TRUE(o.observe(ack(14.),14.));
  EXPECT_FALSE(o.canPrepareInitialGeometry(14.))<<kind;
 }
 Owner wrong("isolated_mock");wrong.bind(ver());auto predecessor=receipt();
 predecessor.expected_version=ver();predecessor.expected_trajectory_id=17;
 ASSERT_TRUE(wrong.observe(predecessor,14.));ASSERT_TRUE(wrong.observe(proof(14.),14.));
 ASSERT_TRUE(wrong.observe(ack(14.),14.));EXPECT_FALSE(wrong.canPrepareInitialGeometry(14.));
 Owner o("isolated_mock");o.bind(ver());ASSERT_TRUE(o.observe(receipt(),14.));
 auto negative=receipt();negative.accepted=false;negative.source_stamp=stamp(14.);negative.valid_until=stamp(14.5);
 EXPECT_FALSE(o.observe(negative,14.)); // A later negative cannot erase the genuine positive fact.
 auto conflict=receipt();conflict.expected_trajectory_id=18;EXPECT_FALSE(o.observe(conflict,14.));
 ASSERT_TRUE(o.observe(proof(14.),14.));ASSERT_TRUE(o.observe(ack(14.),14.));
 EXPECT_TRUE(o.canPrepareInitialGeometry(14.));
}
TEST(ExecutionIdentityTime, RecordedStampUsesIdenticalRosClockArithmetic) {
 builtin_interfaces::msg::Time t;t.sec=1774197216;t.nanosec=858544988;
 // rclcpp::Time::seconds uses integer total nanoseconds divided by 1e9.
 // The linked production rclcpp reproduction is retained in experiments.
 const auto ns=static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
 const double ros_clock=static_cast<double>(ns)/1000000000.;
 EXPECT_EQ(seconds(t),ros_clock);
 EXPECT_TRUE(fresh(seconds(t),ros_clock,.5));
}
TEST(PlanningGeometryOwner, RecordedReceiptAndProofAcceptTheirExactRosClock) {
 const auto exact=[](int32_t sec,uint32_t nsec){builtin_interfaces::msg::Time t;t.sec=sec;t.nanosec=nsec;return t;};
 const auto ros_seconds=[](const auto&t){return static_cast<double>(static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec)/1e9;};
 Owner o("live",30.,.6,{},true);o.bind(ver());
 auto r=receipt();r.transport_mode="live";
 r.source_stamp=exact(1774197216,858544988);r.valid_until=exact(1774197217,358544988);
 ASSERT_TRUE(o.observe(r,ros_seconds(r.source_stamp)));
 auto v=proof();v.transport_mode="live";
 v.source_stamp=v.check_end=exact(1774197217,165072128);
 v.check_begin=exact(1774197217,157045220);v.body_source_stamp=exact(1774197217,141511680);
 v.front_ray_source_stamp=exact(1774197216,799983104);v.rear_ray_source_stamp=exact(1774197216,799994880);
 v.valid_until=exact(1774197217,299983104);v.checked_to_time=v.curve_duration=12.29509947311361;
 ASSERT_TRUE(o.observe(v,ros_seconds(v.source_stamp)));
 const auto p=o.tick(ros_seconds(exact(1774197217,201806068)),false);
 EXPECT_TRUE(p.geometry_committed);EXPECT_EQ(p.trajectory_id,1);EXPECT_EQ(p.validation_sequence,1u);
 EXPECT_FALSE(p.allowed);EXPECT_FALSE(o.confirmed());
}
TEST(ExecutionIdentityTime, NoFutureToleranceWasAdded) {
 builtin_interfaces::msg::Time future;future.sec=1;future.nanosec=1;
 // At this epoch a 1 ns difference is representable. Unix-epoch double
 // seconds cannot resolve individual nanoseconds; do not claim otherwise.
 EXPECT_GT(seconds(future),1.);EXPECT_FALSE(fresh(seconds(future),1.,.35));
 Owner o("isolated_mock",30.,.6,{},true);o.bind(ver());
 auto r=receipt();r.source_stamp=future;r.valid_until=stamp(1.2);
 EXPECT_FALSE(o.observe(r,1.));
 auto v=proof(1.);v.source_stamp=v.check_begin=v.check_end=future;
 EXPECT_FALSE(o.observe(v,1.));
}
TEST(PlanningGeometryOwner, RecordedEpochTrueFutureExpiryAndIdentityStillReject) {
 builtin_interfaces::msg::Time t;t.sec=1774197216;t.nanosec=858544988;
 const auto ns=static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec;
 const double now=static_cast<double>(ns)/1e9;
 for(int kind=0;kind<3;++kind) {
  Owner o("isolated_mock",30.,.6,{},true);o.bind(ver());
  auto r=receipt();r.source_stamp=t;r.valid_until=stamp(now+.2);
  auto v=proof(now);v.source_stamp=v.check_begin=v.check_end=t;
  if(kind==0){r.source_stamp.nanosec+=1000;v.source_stamp.nanosec+=1000;v.check_end=v.source_stamp;}
  if(kind==1){r.valid_until=t;r.valid_until.nanosec-=1000;v.valid_until=r.valid_until;}
  if(kind==2){r.version.task_id="wrong-task";v.version.task_id="wrong-task";}
  EXPECT_FALSE(o.observe(r,now))<<kind;EXPECT_FALSE(o.observe(v,now))<<kind;
  EXPECT_FALSE(o.tick(now,false).geometry_committed)<<kind;
 }
}
TEST(PlanningGeometryOwner, ExecutionStillNeedsItsExactTrackerPreparation) {
 Owner o("isolated_mock");o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));ASSERT_TRUE(o.observe(proof(),10.));
 EXPECT_FALSE(o.tick(10.1,true).geometry_committed);EXPECT_FALSE(o.canConfirm(10.1));
 EXPECT_FALSE(o.begin("execution","confirm",1,10.1));
 ASSERT_TRUE(o.observe(ack(),10.));EXPECT_TRUE(o.tick(10.1,true).geometry_committed);
 EXPECT_TRUE(o.canConfirm(10.1));
}
TEST(PlanningGeometryOwner, TrackerFactsCannotInstallOrVetoPlanningGeometry) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
 ASSERT_TRUE(o.observe(proof(),10.));auto negative=ack();negative.accepted=false;
 EXPECT_FALSE(o.observe(negative,10.));auto p=o.tick(10.1,true);ASSERT_TRUE(p.geometry_committed);
 p.frame_id="d1max_loc_odom";o.notePublication(p,10.1);
 EXPECT_FALSE(o.observe(geometryReceipt(p),10.1));
 EXPECT_FALSE(o.observe(initialCommit(o,p),10.11));
 EXPECT_TRUE(o.tick(10.12,true).geometry_committed);EXPECT_EQ(o.appliedCommitSequence(),0u);
}
TEST(PlanningGeometryOwner, MissingRejectedAndWrongPredecessorReceiptNeverCommit) {
 for(int kind=0;kind<5;++kind) {
  Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(proof(),10.));
  auto r=receipt();
  if(kind==1)r.accepted=false;
  if(kind==2)r.expected_version=ver();
  if(kind==3)r.expected_trajectory_id=7;
  if(kind==4)r.proposal_id="other-proposal";
  if(kind)ASSERT_TRUE(o.observe(r,10.));
  EXPECT_FALSE(o.tick(10.1,true).geometry_committed)<<kind;
  EXPECT_EQ(o.preparationEvidence(10.1),0)<<kind;
 }
}
TEST(PlanningGeometryOwner, WholeDomainFrameDeadlineAndSupportAreStillRequired) {
 for(int kind=0;kind<9;++kind) {
  Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
  auto v=proof();
  if(kind==0)v.whole_curve=false;
  if(kind==1){v.whole_curve=false;v.remaining_curve=true;v.checked_from_time=v.valid_start_time=.5;v.reverse_margin_m=.2;}
  if(kind==2)v.checked_to_time=.9;
  if(kind==3)v.valid_start_time=1.1;
  if(kind==4)v.frame_id="d1max_loc_map";
  if(kind==5)v.valid_until=stamp(10.05);
  if(kind==6)v.support_reference_id.clear();
  if(kind==7)v.support_hash.clear();
  if(kind==8)v.collision_policy="unrecognized";
  ASSERT_TRUE(o.observe(v,10.));
  EXPECT_FALSE(o.tick(10.1,true).geometry_committed)<<kind;
 }
}
TEST(PlanningGeometryOwner, ScanDeadlineOwnsPerceptionFreshness) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
 auto v=proof();v.body_source_stamp=v.front_ray_source_stamp=v.rear_ray_source_stamp=stamp(9.);
 v.valid_until=stamp(10.15);ASSERT_TRUE(o.observe(v,10.));
 EXPECT_TRUE(o.tick(10.1,true).geometry_committed);
 EXPECT_FALSE(o.tick(10.16,true).geometry_committed); // No downstream deadline extension.
}
TEST(PlanningGeometryOwner, LatestNegativeAndSourceExpiryAreNotRenewedByPreviewTicks) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
 ASSERT_TRUE(o.observe(proof(),10.));ASSERT_TRUE(o.tick(10.1,true).geometry_committed);
 auto negative=proof(10.12,2);negative.valid=false;ASSERT_TRUE(o.observe(negative,10.12));
 EXPECT_FALSE(o.tick(10.13,true).geometry_committed);EXPECT_EQ(o.preparationEvidence(10.13),0);
 ASSERT_TRUE(o.observe(proof(10.14,3),10.14));ASSERT_TRUE(o.tick(10.15,true).geometry_committed);
 const auto expired=o.tick(10.55,true);EXPECT_FALSE(expired.geometry_committed);EXPECT_FALSE(expired.allowed);
 EXPECT_EQ(o.preparationEvidence(10.55),0);EXPECT_EQ(expired.phase,"preview");
}
TEST(PlanningGeometryOwner, FreshWholeProofRestagesGeometryWithExactVersionCAS) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
 ASSERT_TRUE(o.observe(proof(),10.));const auto first=o.tick(10.1,true);ASSERT_TRUE(first.geometry_committed);
 ASSERT_TRUE(o.observe(proof(10.12,2),10.12));EXPECT_EQ(o.tick(10.13,true).validation_sequence,2u);
 auto next=proof(10.14,3,2);next.version.reference_generation=2;next.proposal_id="next";
 auto r=receipt();r.version=next.version;r.proposal_id=next.proposal_id;
 r.expected_version=first.version;r.expected_trajectory_id=first.trajectory_id;
 ASSERT_TRUE(o.observe(r,10.14));ASSERT_TRUE(o.observe(next,10.14));
 const auto p=o.tick(10.15,true);EXPECT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);
 EXPECT_EQ(p.version,next.version);EXPECT_EQ(p.trajectory_id,2);EXPECT_EQ(p.validation_sequence,3u);
 EXPECT_EQ(o.appliedCommitSequence(),0u);EXPECT_FALSE(o.begin("wrong-caller","confirm",1,10.15));
}
TEST(PlanningGeometryOwner, NativeRenewalDoesNotNeedTrackerOrRedateReferenceReceipt) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());const auto original=receipt();
 ASSERT_TRUE(o.observe(original,10.));ASSERT_TRUE(o.observe(proof(),10.));
 ASSERT_TRUE(o.tick(10.1,true).geometry_committed);
 // The original delivery lease is over. Fresh real native whole-curve
 // evidence can still validate the already received exact reference.
 const auto renewed=proof(11.,2);ASSERT_TRUE(o.observe(renewed,11.));
 const auto p=o.tick(11.1,true);EXPECT_TRUE(p.geometry_committed);EXPECT_FALSE(p.allowed);
 EXPECT_EQ(p.validation_sequence,2u);EXPECT_EQ(p.version,original.version);
 EXPECT_EQ(o.preparationEvidence(11.1),11000000000LL);
 EXPECT_EQ(o.preparationEvidence(11.2),11000000000LL); // Heartbeat is not a new body sample.
 EXPECT_FALSE(o.tick(11.41,true).geometry_committed);
}
TEST(PlanningGeometryOwner, FollowPreparationBecomesReadyWithRealNativeBodyEvidence) {
 Owner o("isolated_mock",30.,1.,{},true);o.bind(ver());ASSERT_TRUE(o.observe(receipt(),10.));
 PreparationLimits limits;limits.waits={1.,1.,1.,1.};limits.episode=2.;limits.stable=.4;
 PreparationBudget budget(limits);
 ASSERT_TRUE(budget.observe(PreparationStage::Trajectory,9.95).empty());
 for(int i=0;i<3;++i) {
  const auto t=10.+i*.3;ASSERT_TRUE(o.observe(proof(t,i+1),t));
  ASSERT_TRUE(o.tick(t+.1,true).geometry_committed);
  const auto evidence=o.preparationEvidence(t+.1);
  EXPECT_EQ(preparationStage(true,true,"following",evidence),PreparationStage::Ready);
  ASSERT_TRUE(budget.observe(preparationStage(true,true,"following",evidence),t+.1,evidence).empty());
 }
 // Three distinct acquired body stamps reset the real preparation episode;
 // no tracker/admission/SDK callback was needed to leave the wait stage.
 ASSERT_TRUE(o.observe(proof(12.,4),12.));ASSERT_TRUE(o.tick(12.1,true).geometry_committed);
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,12.1,o.preparationEvidence(12.1)).empty());
}
TEST(ExecutionInitialGeometry, LatestUnadmittedProofCannotAdvanceFirstGeometryFloor) {
 Owner o("isolated_mock");beginUnapplied(o);
 auto first=o.tick(10.1,true);ASSERT_TRUE(first.allowed);EXPECT_EQ(first.validation_sequence,1u);
 ASSERT_TRUE(o.observe(proof(10.12,2),10.12));
 auto rejected=ack(10.12,2);rejected.accepted=false;rejected.reason="candidate:join_evidence_invalid_or_stale";
 ASSERT_TRUE(o.observe(rejected,10.12));
 auto p=o.tick(10.13,true);EXPECT_FALSE(p.allowed);EXPECT_FALSE(p.geometry_committed);
 EXPECT_EQ(p.validation_sequence,1u);EXPECT_EQ(p.reason,"waiting_initial_tracker_admission");
 EXPECT_EQ(o.appliedCommitSequence(),0u);
}
TEST(ExecutionInitialGeometry, FreshCandidateRestagesBeforeAnyWriterCommit) {
 Owner o("isolated_mock");beginUnapplied(o);const auto first=o.tick(10.1,true);
 const auto late=initialCommit(o,first,10.11); // Issued, not observed/applied yet.
 ASSERT_TRUE(o.observe(sdk(10.4,2),10.4));
 ASSERT_TRUE(o.observe(proof(10.4,2,2),10.4));ASSERT_TRUE(o.observe(ack(10.4,2,2),10.4));
 auto p=o.tick(10.41,true);ASSERT_TRUE(p.allowed);EXPECT_EQ(p.trajectory_id,2);EXPECT_EQ(p.validation_sequence,2u);
 EXPECT_EQ(o.appliedCommitSequence(),0u);EXPECT_EQ(o.pendingVersion(10.41),nullptr);
 // A real delayed old submission wins over a newer desired initial geometry.
 ASSERT_TRUE(o.observe(late,10.42));EXPECT_EQ(o.appliedCommitSequence(),1u);
 p=o.tick(10.42,true);EXPECT_EQ(p.trajectory_id,1);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.phase,"holding");
 EXPECT_FALSE(o.observe(late,10.43));EXPECT_EQ(o.tick(10.43,true).trajectory_id,1);
}
TEST(ExecutionInitialGeometry, WholeCollisionProofWithoutTrackerReceiptCannotRenewInitialEntry) {
 Owner o("isolated_mock");prepare(o);o.tick(10.01,true);
 ASSERT_TRUE(o.observe(proof(12.,20),12.));
 auto p=o.tick(12.01,true);EXPECT_FALSE(p.geometry_committed);EXPECT_FALSE(o.canConfirm(12.01));
 EXPECT_FALSE(o.begin("execution","missing-install",1,12.01));
}
TEST(ExecutionInitialGeometry, SDKAppliedStatusIsNotAnAckAndCannotAuthorizeDesiredRestaging) {
 Owner o("isolated_mock");beginUnapplied(o);const auto first=o.tick(10.1,true);
 const auto actual=initialCommit(o,first,10.11);
 auto s=sdk(10.12,2);s.writer_commit_sequence=1;s.applied_trajectory_id=1;
 ASSERT_TRUE(o.observe(s,10.12));
 ASSERT_TRUE(o.observe(proof(10.12,2,2),10.12));ASSERT_TRUE(o.observe(ack(10.12,2,2),10.12));
 auto p=o.tick(10.13,true);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.reason,"waiting_writer_commit_ack");
 EXPECT_EQ(o.appliedCommitSequence(),0u);
 ASSERT_TRUE(o.observe(actual,10.14));EXPECT_EQ(o.appliedCommitSequence(),1u);
 p=o.tick(10.15,true);EXPECT_EQ(p.trajectory_id,1);EXPECT_TRUE(p.allowed);
}
TEST(ExecutionInitialGeometry, TrackerInstallationIsBoundToPublishedOriginalEvidence) {
 for(int kind=0;kind<8;++kind) {
  Owner o("isolated_mock");prepare(o);auto p=o.tick(10.01,true);p.frame_id="d1max_loc_odom";
  o.notePublication(p,10.01);auto r=geometryReceipt(p,10.02,1,1,10.03);
  if(kind==0)r.version.anchor_revision++;if(kind==1)r.permit_sequence++;
  if(kind==2)r.validation_sequence++;if(kind==3)r.admission_sequence++;
  if(kind==4)r.installed_at=stamp(10.4);if(kind==5)r.frame_id="map";
  if(kind==6)r.transport_mode="live";if(kind==7)r.version.map_geometry_revision++;
  EXPECT_FALSE(o.observe(r,10.41));
  ASSERT_TRUE(o.observe(proof(10.41,20),10.41));EXPECT_FALSE(o.tick(10.42,true).geometry_committed);
 }
}
TEST(ExecutionInitialGeometry, SoftwareReceiptDoesNotRestampInstallationOrCollisionLease) {
 Owner o("isolated_mock");prepare(o);auto p=o.tick(10.01,true);p.frame_id="d1max_loc_odom";
 o.notePublication(p,10.01);auto r=geometryReceipt(p,10.02,1,1,10.03);
 ASSERT_TRUE(o.observe(r,10.03));auto repeated=r;repeated.sequence=2;
 EXPECT_FALSE(o.observe(repeated,10.2));
 auto altered=r;altered.sequence=2;altered.installed_at=stamp(10.04);altered.body_source_stamp=stamp(10.2);
 EXPECT_FALSE(o.observe(altered,10.2));
 ASSERT_TRUE(o.observe(proof(10.45,2),10.45));
 EXPECT_FALSE(o.tick(10.46,true).geometry_committed); // Current body fact expired.
 r.sequence=2;r.body_source_stamp=stamp(10.47);ASSERT_TRUE(o.observe(r,10.47));
 EXPECT_TRUE(o.tick(10.48,true).geometry_committed); // Fresh body + fresh proof, no SDK permit.
 EXPECT_EQ(o.appliedCommitSequence(),0u);
 r.sequence=3;r.body_source_stamp=stamp(10.49);r.installed=false;ASSERT_TRUE(o.observe(r,10.49));
 EXPECT_FALSE(o.tick(10.5,true).geometry_committed);
}
TEST(ExecutionInitialGeometry, SameBodyNegativeReceiptFencesAndCannotReviveByRepeatingSource) {
 Owner o("isolated_mock");prepare(o);auto p=o.tick(10.01,true);p.frame_id="d1max_loc_odom";
 o.notePublication(p,10.01);auto r=geometryReceipt(p,10.02,1,1,10.03);
 ASSERT_TRUE(o.observe(r,10.03));r.sequence=2;r.installed=false;ASSERT_TRUE(o.observe(r,10.04));
 ASSERT_TRUE(o.observe(proof(10.4,2),10.4));r.sequence=3;r.installed=true;
 EXPECT_FALSE(o.observe(r,10.4));EXPECT_FALSE(o.tick(10.4,true).geometry_committed);
}
TEST(ExecutionOwner, SourceEvidencedRecoveryAllowsStationaryRobotWithoutMotionDeadlock) {
 Owner o("isolated_mock");begin(o);
 ASSERT_TRUE(o.tick(10.1,true,101,"epoch1:seed1").allowed);
 auto p=o.tick(10.2,false,101,"epoch1:seed1");EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.geometry_committed);
 for(int i=0;i<=7;++i) {
  const double t=10.3+i*.1;const auto seq=static_cast<uint64_t>(i+2);
  ASSERT_TRUE(o.observe(sdk(t,seq),t));ASSERT_TRUE(o.observe(proof(t,seq),t));
  p=o.tick(t,true,102+i,"epoch1:seed1");
  if(i<6)EXPECT_FALSE(p.allowed);
 }
 EXPECT_TRUE(p.allowed);EXPECT_EQ(p.trajectory_id,1);
 // No MotionDemand or measured displacement was needed for recovery admission.
 EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, RepeatedSourceCannotResumeEvenWithFreshSDKAndCollisionProof) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true,101,"epoch1:seed1").allowed);
 EXPECT_FALSE(o.tick(10.2,false,101,"epoch1:seed1").allowed);
 for(int i=0;i<20;++i) {
  const double t=10.3+i*.1;const auto seq=static_cast<uint64_t>(i+2);
  o.observe(sdk(t,seq),t);o.observe(proof(t,seq),t);
  EXPECT_FALSE(o.tick(t,true,102,"epoch1:seed1").allowed);
 }
}
TEST(ExecutionOwner, McEvidenceHoldPreservesTaskAndRequiresStableSourceRecovery) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true,101,"identity").allowed);
 auto unavailable=sdk(10.2,2);unavailable.grant_ready=false;unavailable.phase="holding";unavailable.reason="mc_stale";
 ASSERT_TRUE(o.observe(unavailable,10.2));auto p=o.tick(10.2,true,102,"identity");
 EXPECT_FALSE(p.allowed);EXPECT_FALSE(p.revoked);EXPECT_TRUE(p.geometry_committed);
 EXPECT_EQ(p.reason,"mc_stale");EXPECT_FALSE(o.stopping());EXPECT_TRUE(o.failureReason().empty());
 for(int i=0;i<=7;++i) {
  const double t=10.3+i*.1;const auto seq=static_cast<uint64_t>(i+3);
  ASSERT_TRUE(o.observe(sdk(t,seq),t));ASSERT_TRUE(o.observe(proof(t,seq),t));
  p=o.tick(t,true,103+i,"identity");
  if(i<6)EXPECT_FALSE(p.allowed);
 }
 EXPECT_TRUE(p.allowed);EXPECT_FALSE(o.stopping());EXPECT_TRUE(o.failureReason().empty());
}
TEST(ExecutionOwner, McEvidenceHoldCannotRecoverFromRepeatedLocalEvidenceOrRenewBudget) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true,101,"identity").allowed);
 feedback(o,10.2,1); // The motion task had actually admitted a nonzero demand.
 auto unavailable=sdk(10.2,2);unavailable.grant_ready=false;unavailable.phase="holding";unavailable.reason="mc_stale";
 ASSERT_TRUE(o.observe(unavailable,10.2));auto p=o.tick(10.2,true,102,"identity");
 p.frame_id="d1max_loc_odom";
 EXPECT_TRUE(o.supervise(p,10.2).empty());
 for(int i=1;i<=301;++i) {
  const double t=10.2+i*.1;const auto seq=static_cast<uint64_t>(i+2);
  auto state=sdk(t,seq);
  // Brief SDK good blips and fresh geometry cannot renew a failed recovery.
  if(i%5) {state.grant_ready=false;state.phase="holding";state.reason="mc_stale";}
  o.observe(state,t);o.observe(proof(t,seq),t);p=o.tick(t,true,103,"identity");
  const auto failure=o.supervise(p,t);
  EXPECT_FALSE(p.allowed);
  if(i<300)EXPECT_TRUE(failure.empty());
  if(!failure.empty())break;
 }
 EXPECT_TRUE(o.stopping());EXPECT_NE(o.failureReason().find("execution_blocked_timeout:"),std::string::npos);
}
TEST(ExecutionOwner, McStaleReasonCannotSoftenActualSDKWithdrawal) {
 for(int variant=0;variant<3;++variant) {
  Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
  auto s=sdk(10.2,2);s.grant_ready=false;s.phase="holding";s.reason="mc_stale";
  if(variant==0)s.control_owned=false;
  if(variant==1)s.fault_latched=true;
  if(variant==2)s.phase="stopping";
  ASSERT_TRUE(o.observe(s,10.2));EXPECT_TRUE(o.stopping());EXPECT_FALSE(o.failureReason().empty());
  EXPECT_FALSE(o.tick(10.2,true).allowed);
 }
}
TEST(ExecutionOwner, RobotStateEvidenceHoldRetainsExactTaskWithoutClaimingFreshMode) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true,101,"identity").allowed);
 auto unavailable=sdk(10.2,2);unavailable.grant_ready=false;
 unavailable.general_low_speed_confirmed=false;unavailable.phase="holding";
 unavailable.reason="robot_state_stale";
 ASSERT_TRUE(o.observe(unavailable,10.2));auto p=o.tick(10.2,true,102,"identity");
 EXPECT_FALSE(p.allowed);EXPECT_FALSE(p.revoked);EXPECT_TRUE(p.geometry_committed);
 EXPECT_EQ(p.reason,"robot_state_stale");EXPECT_FALSE(o.stopping());
 EXPECT_TRUE(o.failureReason().empty());EXPECT_EQ(p.version.task_id,"task");
 EXPECT_EQ(p.version.route_hash,ver().route_hash);EXPECT_EQ(p.trajectory_id,1);
 // A fresh local body and native map proof are not substitutes for SDK mode
 // state. They cannot unblock nonzero while that producer is still holding.
 for(int i=0;i<8;++i) {
  const double t=10.3+i*.1;const auto seq=static_cast<uint64_t>(i+3);
  unavailable.source_stamp=stamp(t);unavailable.sequence=seq;
  ASSERT_TRUE(o.observe(unavailable,t));ASSERT_TRUE(o.observe(proof(t,seq),t));
  p=o.tick(t,true,103+i,"identity");
  EXPECT_FALSE(p.allowed);EXPECT_FALSE(o.stopping());
 }
 // Fresh SDK state does not immediately revive an old command: stable new
 // localization evidence and original new map proofs must complete recovery.
 for(int i=0;i<=7;++i) {
  const double t=11.1+i*.1;const auto seq=static_cast<uint64_t>(i+11);
  ASSERT_TRUE(o.observe(sdk(t,seq),t));ASSERT_TRUE(o.observe(proof(t,seq),t));
  p=o.tick(t,true,111+i,"identity");
  if(i<6)EXPECT_FALSE(p.allowed);
 }
 EXPECT_TRUE(p.allowed);EXPECT_EQ(p.version.task_id,"task");
 EXPECT_EQ(p.version.route_hash,ver().route_hash);EXPECT_EQ(p.trajectory_id,1);
 EXPECT_FALSE(o.stopping());EXPECT_TRUE(o.failureReason().empty());
}
TEST(ExecutionOwner, RobotStateStaleCannotSoftenLossFaultBadModeOrChangedSDKSession) {
 for(int variant=0;variant<7;++variant) {
  Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
  auto s=sdk(10.2,2);s.grant_ready=false;s.general_low_speed_confirmed=false;
  s.phase="holding";s.reason="robot_state_stale";
  if(variant==0)s.control_owned=false;
  if(variant==1)s.fault_latched=true;
  if(variant==2)s.phase="stopping";
  if(variant==3)s.reason="robot_mode_invalid";
  if(variant==4)s.sdk_session="reconnected-sdk";
  if(variant==5)s.sdk_arm_generation=2;
  if(variant==6)s.general_low_speed_confirmed=true; // Inconsistent disposition.
  if(variant==4)EXPECT_FALSE(o.observe(s,10.2));
  else ASSERT_TRUE(o.observe(s,10.2));
  EXPECT_TRUE(o.stopping());EXPECT_FALSE(o.failureReason().empty());
  EXPECT_FALSE(o.tick(10.2,true).allowed);
  EXPECT_TRUE(o.observe(sdk(10.3,3),10.3)); // Same old session may report fresh state, not revive authorization.
  EXPECT_FALSE(o.tick(10.3,true).allowed); // Fresh late status cannot revive a withdrawal.
 }
}
TEST(ExecutionOwner, RobotStateStaleStatusStillRequiresExactExecutionAndOriginalSource) {
 for(int variant=0;variant<7;++variant) {
  Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
  auto s=sdk(10.2,2);s.grant_ready=false;s.general_low_speed_confirmed=false;
  s.phase="holding";s.reason="robot_state_stale";
  if(variant==0)s.execution_id="old-execution";
  if(variant==1)s.control_epoch=2;
  if(variant==2)s.version.task_id="old-task";
  if(variant==3)s.transport_mode="live";
  if(variant==4)s.source_stamp=stamp(9.);
  if(variant==5)s.sequence=1;
  if(variant==6)s.sdk_session.clear();
  EXPECT_FALSE(o.observe(s,10.2));EXPECT_FALSE(o.stopping());
  EXPECT_TRUE(o.failureReason().empty());
  // The ignored disposition does not make the previously original SDK and
  // native proof leases immortal; source expiry still holds the exact task.
  EXPECT_FALSE(o.tick(10.61,true).allowed);EXPECT_FALSE(o.stopping());
 }
}
TEST(ExecutionTask, HardSDKWithdrawalRetiresRunningFollowAndCannotReviveOnLateGrant) {
 Owner owner("isolated_mock");begin(owner);ASSERT_TRUE(owner.tick(10.1,true).allowed);
 auto backend=std::make_shared<ExecutionTaskBackend>(owner);
 d1max_navigation_bt::Engine tree(backend,DEFAULT_BT_XML);tree.submit({"task","nav"});
 for(int i=0;i<3;++i)ASSERT_EQ(tree.tick(),BT::NodeStatus::RUNNING);
 ASSERT_EQ(backend->follows,1);
 auto withdrawn=sdk(10.2,2);withdrawn.control_owned=false;withdrawn.grant_ready=false;withdrawn.reason="control_lost";
 ASSERT_TRUE(owner.observe(withdrawn,10.2));
 EXPECT_EQ(tree.tick(),BT::NodeStatus::FAILURE);EXPECT_FALSE(tree.snapshot().active);
 EXPECT_EQ(backend->finished,1);EXPECT_FALSE(backend->success);EXPECT_EQ(backend->follow_halts,1);
 EXPECT_EQ(tree.snapshot().reason,"sdk_grant_withdrawn:control_lost");
 ASSERT_TRUE(owner.observe(sdk(10.3,3),10.3));
 for(int i=0;i<20;++i)EXPECT_EQ(tree.tick(),BT::NodeStatus::FAILURE);
 EXPECT_EQ(backend->routes,1);EXPECT_EQ(backend->follows,1);EXPECT_EQ(backend->finished,1);
 EXPECT_FALSE(owner.stopped()); // Algorithm failure is not physical-stop proof.
}
TEST(ExecutionTask, SDKHardFailureCannotHideBehindPausedInputDecorator) {
 Owner owner("isolated_mock");begin(owner);ASSERT_TRUE(owner.tick(10.1,true).allowed);
 auto backend=std::make_shared<ExecutionTaskBackend>(owner);
 d1max_navigation_bt::Engine tree(backend,DEFAULT_BT_XML);tree.submit({"task","nav"});
 for(int i=0;i<3;++i)ASSERT_EQ(tree.tick(),BT::NodeStatus::RUNNING);
 backend->inputs_ready=false;ASSERT_EQ(tree.tick(),BT::NodeStatus::RUNNING);
 owner.stop("sdk_session_changed",10.2);
 EXPECT_EQ(tree.tick(),BT::NodeStatus::FAILURE);EXPECT_FALSE(tree.snapshot().active);
 EXPECT_EQ(backend->follow_halts,1);EXPECT_EQ(backend->finished,1);
}
TEST(ExecutionTask, EvidenceHoldAndNormalGoalStopDoNotAbortRunningTask) {
 Owner owner("isolated_mock");begin(owner);ASSERT_TRUE(owner.tick(10.1,true).allowed);
 auto backend=std::make_shared<ExecutionTaskBackend>(owner);
 d1max_navigation_bt::Engine tree(backend,DEFAULT_BT_XML);tree.submit({"task","nav"});
 for(int i=0;i<3;++i)ASSERT_EQ(tree.tick(),BT::NodeStatus::RUNNING);
 auto held=sdk(10.2,2);held.grant_ready=false;held.phase="holding";held.reason="mc_stale";
 ASSERT_TRUE(owner.observe(held,10.2));EXPECT_FALSE(owner.tick(10.2,true).allowed);
 EXPECT_EQ(tree.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(backend->follow_halts,0);
 owner.stop("goal_reached",10.3,true);EXPECT_TRUE(owner.failureReason().empty());
 EXPECT_EQ(tree.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(backend->finished,0);
 owner.tick(15.4,true);EXPECT_TRUE(owner.needsReview());
 EXPECT_EQ(tree.tick(),BT::NodeStatus::FAILURE);EXPECT_EQ(backend->follow_halts,1);
}
TEST(LocalStateAdmission, IndependentLocalEvidenceDoesNotRequireASyntheticGlobalPose) {
 using State=d1max_planning_interfaces::msg::LocalNavigationState;State old,m;
 m.schema_version=2;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(10.);m.usable=true;
 m.valid_until=stamp(10.1);
 m.local_odometry.header.stamp=m.source_stamp;m.local_odometry.header.frame_id="d1max_loc_odom";
 m.local_odometry.child_frame_id="d1max_loc_base_link";m.local_odometry.pose.pose.orientation.w=1.;
 auto accept=[&](const State&s){return d1max_navigation_bt::acceptLocalState(s,old,"nav","map",10.05);};
 ASSERT_TRUE(accept(m));old=m;EXPECT_FALSE(accept(m));
 m.source_stamp=m.local_odometry.header.stamp=stamp(10.02);m.usable=false;m.reason="waiting_imu";
 EXPECT_TRUE(accept(m));auto bad=m;bad.session_id="other";EXPECT_FALSE(accept(bad));
 bad=m;bad.usable=true;bad.local_odometry.header.frame_id="map";EXPECT_FALSE(accept(bad));
 bad=m;bad.usable=true;bad.imu_stamp=stamp(9.8);EXPECT_FALSE(accept(bad));
 bad=m;bad.imu_stamp=bad.posterior_stamp=bad.source_stamp;
 bad.localization_epoch=2;bad.localization_seed_id="reset";ASSERT_TRUE(accept(bad));old=bad;
 bad=m;bad.source_stamp=bad.local_odometry.header.stamp=stamp(10.03);EXPECT_FALSE(accept(bad));
}
namespace {
d1max_planning_interfaces::msg::LocalNavigationState taskLocalState() {
 d1max_planning_interfaces::msg::LocalNavigationState m;
 m.schema_version=2;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=stamp(10.08);m.imu_stamp=stamp(10.);m.extrapolation_sec=.08;m.usable=true;
 m.valid_until=stamp(10.18);
 m.local_odometry.header.stamp=m.source_stamp;m.local_odometry.header.frame_id="d1max_loc_odom";
 m.local_odometry.child_frame_id="d1max_loc_base_link";m.local_odometry.pose.pose.orientation.w=1.;
 return m;
}
bool taskLocalFresh(const d1max_planning_interfaces::msg::LocalNavigationState&m,double now=10.14) {
 return d1max_navigation_bt::localTaskInputsFresh(m,"nav","map",1,"seed",now);
}
}
TEST(LocalStateAdmission, UsableEntryRequiresOriginalPropagationAndCausalTimestamps) {
 const auto m=taskLocalState();d1max_planning_interfaces::msg::LocalNavigationState old;
 auto accept=[&](const auto&s){return d1max_navigation_bt::acceptLocalState(s,old,"nav","map",10.09);};
 ASSERT_TRUE(accept(m));
 for(int kind=0;kind<7;++kind) {
  auto bad=m;
  switch(kind) {
   case 0:bad.extrapolation_sec=.101;break;
   case 1:bad.imu_stamp=stamp(10.085);break;
   case 2:bad.posterior_stamp=stamp(10.085);break;
   case 3:bad.extrapolation_sec=.01;break;
   case 4:bad.posterior_stamp.nanosec=1000000000U;break;
   case 5:bad.imu_stamp=stamp(0.);break;
   case 6:bad.local_odometry.pose.pose.orientation.w=0.;break;
  }
  EXPECT_FALSE(accept(bad))<<kind;
 }
 // A newer explicit loss must withdraw immediately, even when it has no
 // usable posterior/IMU or pose. It is not an alternative positive contract.
 old=m;auto unavailable=m;unavailable.usable=false;unavailable.reason="waiting_imu";
 unavailable.source_stamp=unavailable.local_odometry.header.stamp=stamp(10.085);
 unavailable.imu_stamp=unavailable.posterior_stamp=stamp(0.);unavailable.extrapolation_sec=.2;
 unavailable.local_odometry.pose.pose.orientation.w=0.;
 EXPECT_TRUE(accept(unavailable));EXPECT_FALSE(taskLocalFresh(unavailable,10.09));
}
TEST(LocalStateAdmission, NewEpochWithSlightClockRollbackWithdrawsOldExecutionIdentity) {
 auto old=taskLocalState();auto reset=old;reset.localization_epoch=2;reset.localization_seed_id="new";
 reset.source_stamp=reset.posterior_stamp=reset.imu_stamp=reset.local_odometry.header.stamp=stamp(10.03);
 reset.extrapolation_sec=0.;reset.valid_until=reset.source_stamp;
 reset.valid_until.nanosec+=100000000U; // exact original-source + 100 ms, no double reconstruction.
 ASSERT_TRUE(d1max_navigation_bt::acceptLocalState(reset,old,"nav","map",10.1));
 EXPECT_FALSE(d1max_navigation_bt::localTaskInputsFresh(reset,"nav","map",1,"seed",10.1));
 // New-epoch loss must be observed even without a synthetic usable pose.
 reset.usable=false;reset.reason="lio_reset";EXPECT_TRUE(d1max_navigation_bt::acceptLocalState(reset,old,"nav","map",10.1));
 old=reset;auto late=taskLocalState();late.source_stamp=late.local_odometry.header.stamp=stamp(10.09);
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(late,old,"nav","map",10.1));
 reset.usable=true;reset.reason="";EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(reset,old,"nav","map",10.1));
}
TEST(LocalStateAdmission, ProductionUnavailableEnvelopeRequiresNoSyntheticGeometry) {
 // navigation_output.publish_local_unavailable constructs this from scratch,
 // not by copying the last valid odometry and changing usable to false.
 const auto old=taskLocalState();
 for(const auto* reason:{"local_navigation_unavailable","hard_localization_lost","lio_reset","localization_reset"}) {
  d1max_planning_interfaces::msg::LocalNavigationState event;
  event.schema_version=2;event.session_id="nav";event.map_version_id="map";
  event.localization_epoch=1;event.localization_seed_id="seed";
  event.source_stamp=stamp(10.081);event.usable=false;event.reason=reason;
  ASSERT_TRUE(d1max_navigation_bt::acceptLocalState(event,old,"nav","map",10.09))<<reason;
  EXPECT_FALSE(taskLocalFresh(event,10.09));
  auto fake_positive=event;fake_positive.usable=true;
  EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(fake_positive,old,"nav","map",10.09));
 }
}
TEST(LocalStateAdmission, UnavailableEnvelopeCannotEraseNewerOrForeignIdentity) {
 const auto old=taskLocalState();
 d1max_planning_interfaces::msg::LocalNavigationState event;
 event.schema_version=2;event.session_id="nav";event.map_version_id="map";
 event.localization_epoch=1;event.localization_seed_id="seed";event.source_stamp=stamp(10.081);
 event.usable=false;event.reason="hard_localization_lost";
 const auto accept=[&](const auto&m){return d1max_navigation_bt::acceptLocalState(m,old,"nav","map",10.09);};
 ASSERT_TRUE(accept(event));
 for(int kind=0;kind<10;++kind) {
  auto bad=event;
  switch(kind) {
   case 0:bad.session_id="other";break;
   case 1:bad.map_version_id="other";break;
   case 2:bad.schema_version=1;break;
   case 3:bad.localization_epoch=0;break;
   case 4:bad.localization_seed_id.clear();break;
   case 5:bad.source_stamp=old.source_stamp;break;
   case 6:bad.source_stamp=stamp(9.);break;
   case 7:bad.source_stamp=stamp(10.1);break;
   case 8:bad.source_stamp.nanosec=1000000000U;break;
   case 9:bad.reason.assign(1025,'x');break;
  }
  EXPECT_FALSE(accept(bad))<<kind;
 }
 EXPECT_TRUE(old.usable);EXPECT_EQ(old.source_stamp,stamp(10.08));
 auto reset=event;reset.localization_epoch=2;reset.localization_seed_id="new";reset.reason="localization_reset";
 reset.source_stamp=stamp(10.03);
 ASSERT_TRUE(accept(reset));
 EXPECT_FALSE(d1max_navigation_bt::localTaskInputsFresh(reset,"nav","map",1,"seed",10.09));
 auto late=old;late.source_stamp=late.local_odometry.header.stamp=stamp(10.085);
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(late,reset,"nav","map",10.09));
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(reset,reset,"nav","map",10.09));
}
TEST(LocalTaskContinuity, TimerPhaseCannotTearDownAdmittedTaskOrAuthorizeMotion) {
 const auto m=taskLocalState();d1max_planning_interfaces::msg::LocalNavigationState old;
 ASSERT_TRUE(d1max_navigation_bt::acceptLocalState(m,old,"nav","map",10.09));
 ASSERT_TRUE(taskLocalFresh(m));
 // Provider validated original IMU age at source; a consumer must not reopen
 // that check against its later timer phase. Its source-bound lease is intact.
 EXPECT_FALSE(fresh(seconds(m.imu_stamp),10.14,.1));
 EXPECT_TRUE(d1max_navigation_bt::acceptLocalState(m,old,"nav","map",10.14));
 // An expired control lease still cannot authorize motion. Coarse task
 // continuity is not a substitute for that deadline or a restamped source.
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(m,old,"nav","map",10.181));
 EXPECT_TRUE(taskLocalFresh(m,10.181));
 EXPECT_EQ(m.source_stamp,stamp(10.08));EXPECT_EQ(m.imu_stamp,stamp(10.));
}
TEST(LocalControlContinuity, LaterTimerConsumesOnlyOriginalProviderLease) {
 const auto m=taskLocalState();
 // The same source packet is valid at admission and at a later BT/SDK phase.
 // Its IMU is 140 ms old at use but was only 80 ms old at the state source.
 EXPECT_FALSE(fresh(seconds(m.imu_stamp),10.14,.1));
 EXPECT_TRUE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",10.14));
 const double lease_end=seconds(m.valid_until); // consume the original typed deadline, not a reconstructed double.
 EXPECT_TRUE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",lease_end));
 EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",
   std::nextafter(lease_end,std::numeric_limits<double>::infinity())));
 EXPECT_TRUE(taskLocalFresh(m,10.180001)); // task liveness is NOT motion authority.
 EXPECT_EQ(m.source_stamp,stamp(10.08));EXPECT_EQ(m.valid_until,stamp(10.18));
}
TEST(LocalControlContinuity, PosteriorAgeIsValidatedAtSourceNotChargedAgainAtDelivery) {
 auto m=taskLocalState();m.posterior_stamp=stamp(9.69); // 390 ms at the 10.08 state source.
 d1max_planning_interfaces::msg::LocalNavigationState old;
 const double delivered=10.106; // A legitimate 26 ms delivery phase.
 ASSERT_FALSE(fresh(seconds(m.posterior_stamp),delivered,.4));
 EXPECT_TRUE(d1max_navigation_bt::acceptLocalState(m,old,"nav","map",delivered));
 EXPECT_TRUE(taskLocalFresh(m,delivered));
 EXPECT_TRUE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",delivered));
 const double deadline=seconds(m.valid_until);
 EXPECT_TRUE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",deadline));
 EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",
   std::nextafter(deadline,std::numeric_limits<double>::infinity())));
 EXPECT_TRUE(taskLocalFresh(m,10.479)); // Task continuity does not grant nonzero control.
 EXPECT_FALSE(taskLocalFresh(m,10.481));
 auto too_old=m;too_old.posterior_stamp=stamp(9.679);
 EXPECT_FALSE(taskLocalFresh(too_old,delivered));
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(too_old,old,"nav","map",delivered));
 auto future=m;future.posterior_stamp=stamp(10.081);
 EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(future,"nav","map",1,"seed",delivered));
 EXPECT_EQ(m.posterior_stamp,stamp(9.69));EXPECT_EQ(m.source_stamp,stamp(10.08));
 EXPECT_EQ(m.valid_until,stamp(10.18)); // Never re-date the original producer lease.
}
TEST(LocalControlContinuity, InvalidSourceContractCannotBorrowTaskFreshness) {
 const auto m=taskLocalState();
 for(int kind=0;kind<9;++kind) {
  auto bad=m;
  switch(kind) {
   case 0:bad.usable=false;bad.reason="waiting_imu";break;
   case 1:bad.usable=false;bad.reason="lio_reset";break;
   case 2:bad.imu_stamp=stamp(9.9);bad.extrapolation_sec=.18;break;
   case 3:bad.valid_until=stamp(10.180001);break;
   case 4:bad.valid_until=bad.source_stamp;break;
   case 5:bad.localization_epoch=2;break;
   case 6:bad.localization_seed_id="reset";break;
   case 7:bad.local_odometry.header.frame_id="map";break;
   case 8:bad.posterior_stamp=stamp(10.09);break;
  }
  EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(bad,"nav","map",1,"seed",10.14))<<kind;
 }
 EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(m,"nav","map",1,"seed",10.079));
 EXPECT_FALSE(d1max_navigation_bt::localControlInputsFresh(m,"foreign","map",1,"seed",10.14));
}
TEST(LocalTaskContinuity, RepeatedSnapshotCannotRenewSourceOrPosteriorPropagationBudget) {
 const auto m=taskLocalState();
 EXPECT_TRUE(taskLocalFresh(m,10.35));
 EXPECT_FALSE(taskLocalFresh(m,10.49));
 EXPECT_FALSE(d1max_navigation_bt::acceptLocalState(m,m,"nav","map",10.14));
 auto old_posterior=m;old_posterior.posterior_stamp=stamp(9.679);
 EXPECT_FALSE(taskLocalFresh(old_posterior));
 auto future=m;future.source_stamp=future.local_odometry.header.stamp=stamp(10.15);
 future.extrapolation_sec=.15;EXPECT_FALSE(taskLocalFresh(future));
 EXPECT_FALSE(taskLocalFresh(m,10.07));
}
TEST(LocalTaskContinuity, ExplicitLossHardIdentityAndFramesRemainUnavailable) {
 const auto m=taskLocalState();
 for(int kind=0;kind<10;++kind) {
  auto bad=m;
  switch(kind) {
   case 0:bad.usable=false;bad.reason="waiting_imu";break;
   case 1:bad.usable=false;bad.reason="lio_reset";break;
   case 2:bad.session_id="foreign";break;
   case 3:bad.map_version_id="other-map";break;
   case 4:bad.localization_epoch=2;break;
   case 5:bad.localization_seed_id="reset";break;
   case 6:bad.local_odometry.header.frame_id="map";break;
   case 7:bad.local_odometry.child_frame_id="lidar";break;
   case 8:bad.local_odometry.header.stamp=stamp(10.07);break;
   case 9:bad.schema_version=1;break;
  }
  EXPECT_FALSE(taskLocalFresh(bad))<<kind;
 }
}
TEST(LocalTaskContinuity, OriginalPropagationAndFiniteGeometryCannotBeRelaxed) {
 const auto m=taskLocalState();
 for(int kind=0;kind<10;++kind) {
  auto bad=m;
  switch(kind) {
   case 0:bad.imu_stamp=stamp(10.09);break;
   case 1:bad.posterior_stamp=stamp(10.09);break;
   case 2:bad.imu_stamp=stamp(9.9);bad.extrapolation_sec=.18;break;
   case 3:bad.extrapolation_sec=.01;break;
   case 4:bad.extrapolation_sec=std::numeric_limits<double>::quiet_NaN();break;
   case 5:bad.source_stamp.nanosec=1000000000U;break;
   case 6:bad.imu_stamp=stamp(0.);break;
   case 7:bad.local_odometry.pose.pose.orientation.w=0.;break;
   case 8:bad.local_odometry.pose.pose.position.z=std::numeric_limits<double>::infinity();break;
   case 9:bad.local_odometry.twist.twist.angular.z=std::numeric_limits<double>::quiet_NaN();break;
  }
  EXPECT_FALSE(taskLocalFresh(bad))<<kind;
 }
 EXPECT_FALSE(d1max_navigation_bt::localTaskInputsFresh(m,"nav","map",0,"seed",10.14));
 EXPECT_FALSE(d1max_navigation_bt::localTaskInputsFresh(m,"nav","map",1,"",10.14));
}
TEST(ExecutionTrackerFence, ExactActualInstallationHoldsWithoutCancellingOrBorrowingNewPositiveProof) {
 Owner o("isolated_mock");auto r=installedActual(o);
 r.sequence=2;r.body_source_stamp=stamp(10.13);r.reason="braking_envelope_reentry_required";
 ASSERT_TRUE(o.observe(r,10.13));auto p=o.tick(10.13,true);
 EXPECT_FALSE(p.allowed);EXPECT_FALSE(p.revoked);EXPECT_TRUE(p.geometry_committed);
 EXPECT_EQ(p.phase,"holding");EXPECT_EQ(p.reason,r.reason);EXPECT_FALSE(o.stopping());
 ASSERT_TRUE(o.observe(proof(10.14,2),10.14));ASSERT_TRUE(o.observe(sdk(10.14,2),10.14));
 EXPECT_FALSE(o.tick(10.15,true).allowed);
 r.sequence=3;r.body_source_stamp=stamp(10.16);r.reason="tracker_geometry_installed";
 ASSERT_TRUE(o.observe(r,10.16));EXPECT_FALSE(o.tick(10.16,true).allowed);
 EXPECT_EQ(o.appliedCommitSequence(),1u);EXPECT_EQ(o.pendingVersion(10.16),nullptr);
}
TEST(ExecutionTrackerFence, ForeignExpiredRepeatedAndMutatedInstallationCannotFence) {
 for(int kind=0;kind<9;++kind) {
  Owner o("isolated_mock");auto r=installedActual(o);
  r.sequence=2;r.body_source_stamp=stamp(10.13);r.reason="braking_envelope_reentry_required";
  switch(kind) {
   case 0:r.trajectory_id=2;break;
   case 1:r.version.anchor_revision=2;break;
   case 2:r.body_source_stamp=stamp(10.12);break;
   case 3:r.body_source_stamp=stamp(9.7);break;
   case 4:r.permit_sequence++;break;
   case 5:r.installed_at=stamp(10.11);break;
   case 6:r.reason="unknown_tracker_wait";break;
   case 7:r.installed=false;break;
   case 8:r.transport_mode="live";break;
  }
  if(kind==6||kind==7)EXPECT_TRUE(o.observe(r,10.13));else EXPECT_FALSE(o.observe(r,10.13));
  auto p=o.tick(10.14,true);EXPECT_TRUE(p.allowed)<<kind;EXPECT_FALSE(p.revoked)<<kind;
 }
}
TEST(ExecutionTrackerFence, LaterActualCASInstallationKeepsFixedFactsAndCanFenceItsOwnGeometry) {
 Owner o("isolated_mock");installedActual(o);
 auto r=receipt();r.proposal_id="next-reference";r.version.reference_generation=2;r.expected_version=ver();r.expected_trajectory_id=1;
 r.source_stamp=stamp(10.2);r.valid_until=stamp(10.7);ASSERT_TRUE(o.observe(r,10.2));
 auto v=proof(10.2,2,2);v.version=r.version;v.proposal_id=r.proposal_id;ASSERT_TRUE(o.observe(v,10.2));
 auto a=ack(10.2,2,2);a.version=r.version;ASSERT_TRUE(o.observe(a,10.2));
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(r.version),100,10.21));const auto grant=*o.handoff();
 const auto call=handoffCommit(grant,10.22);ASSERT_TRUE(o.observe(call,10.22));
 auto installed=geometryReceipt(grant.candidate,10.23,2,2,10.24);
 installed.admission_sequence=call.entry_admission_sequence;
 ASSERT_TRUE(o.observe(installed,10.24));ASSERT_TRUE(o.tick(10.25,true).allowed);
 auto bad=installed;bad.sequence=3;bad.body_source_stamp=stamp(10.26);bad.installed_at=stamp(10.24);
 bad.reason="braking_envelope_reentry_required";EXPECT_FALSE(o.observe(bad,10.26));
 installed.sequence=3;installed.body_source_stamp=stamp(10.26);installed.reason=bad.reason;
 ASSERT_TRUE(o.observe(installed,10.26));EXPECT_FALSE(o.tick(10.26,true).allowed);
 v=proof(10.27,3,2);v.version=r.version;v.proposal_id=r.proposal_id;ASSERT_TRUE(o.observe(v,10.27));
 EXPECT_FALSE(o.tick(10.28,true).allowed);EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionTrackerFence, PreparedFloorExpiryDoesNotEraseInstallationOrRenewActualCollisionEvidence) {
 Owner o("isolated_mock");installedActual(o);
 auto r=receipt();r.proposal_id="next-reference";r.version.reference_generation=2;r.expected_version=ver();r.expected_trajectory_id=1;
 r.source_stamp=stamp(10.2);r.valid_until=stamp(10.7);ASSERT_TRUE(o.observe(r,10.2));
 auto v=proof(10.2,2,2);v.version=r.version;v.proposal_id=r.proposal_id;v.valid_until=stamp(10.31);ASSERT_TRUE(o.observe(v,10.2));
 auto a=ack(10.2,2,2);a.version=r.version;a.valid_until=v.valid_until;ASSERT_TRUE(o.observe(a,10.2));
 auto p=o.tick(10.205,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(r.version),100,10.205));const auto grant=*o.handoff();
 // An actual later native revision supports the SDK call. The software entry
 // installation still reports its unchanged original preparation floor.
 auto actual=proof(10.315,3,2);actual.version=r.version;actual.proposal_id=r.proposal_id;actual.valid_until=stamp(10.37);
 ASSERT_TRUE(o.observe(actual,10.315));auto call=handoffCommit(grant,10.32);call.valid_until=stamp(10.36);
 ASSERT_TRUE(o.observe(call,10.32));auto installed=geometryReceipt(grant.candidate,10.33,2,2,10.34);
 installed.admission_sequence=call.entry_admission_sequence;
 ASSERT_TRUE(o.observe(installed,10.34));EXPECT_TRUE(o.tick(10.35,true).allowed);
 EXPECT_EQ(installed.validation_sequence,2u);EXPECT_EQ(installed.installed_at,stamp(10.33));
 installed.sequence=3;installed.body_source_stamp=stamp(10.41);
 ASSERT_TRUE(o.observe(installed,10.41));auto expired=o.tick(10.41,true);
 EXPECT_FALSE(expired.allowed);EXPECT_EQ(expired.reason,"waiting_current_collision_and_tracker_proof");
 EXPECT_EQ(expired.trajectory_id,2);EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionTrackerFence, OnlyVerifiedStationaryReentryAppliedAckClearsTheGeometryFence) {
 Owner o("isolated_mock");auto r=installedActual(o);
 r.sequence=2;r.body_source_stamp=stamp(10.13);r.reason="braking_envelope_reentry_required";
 ASSERT_TRUE(o.observe(r,10.13));EXPECT_FALSE(o.tick(10.13,true).allowed);
 ASSERT_TRUE(o.observe(sdk(11.01,2),11.01));ASSERT_TRUE(o.observe(proof(11.,2),11.01));
 auto reference=receipt();reference.proposal_id="next-reference";reference.version.reference_generation=2;reference.expected_version=ver();reference.expected_trajectory_id=1;
 reference.source_stamp=stamp(11.);reference.valid_until=stamp(11.3);ASSERT_TRUE(o.observe(reference,11.01));
 auto v=proof(11.,3,2);v.version=reference.version;v.proposal_id=reference.proposal_id;ASSERT_TRUE(o.observe(v,11.01));
 auto a=ack(11.,3,2);a.version=reference.version;ASSERT_TRUE(o.observe(a,11.01));
 auto p=o.tick(11.01,true);EXPECT_FALSE(p.allowed);EXPECT_EQ(o.pendingVersion(11.01),nullptr);
 ASSERT_TRUE(o.observe(stationaryWitness(),11.01));p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(reference.version),100,11.02));const auto grant=*o.handoff();
 ASSERT_EQ(grant.transition_mode,Handoff::STATIONARY_REENTRY);
 EXPECT_FALSE(o.tick(11.025,true).allowed);EXPECT_EQ(o.appliedCommitSequence(),1u);
 ASSERT_TRUE(o.observe(handoffCommit(grant,11.03),11.03));
 auto active=o.tick(11.04,true);EXPECT_TRUE(active.allowed);EXPECT_EQ(active.trajectory_id,2);
 EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionTrackerFence, HealthyHeartbeatsCannotRenewItsBoundedBlockedEpisode) {
 Owner o("isolated_mock");auto r=installedActual(o);
 r.sequence=2;r.body_source_stamp=stamp(10.13);r.reason="braking_envelope_reentry_required";
 ASSERT_TRUE(o.observe(r,10.13));auto p=o.tick(10.13,true);EXPECT_TRUE(o.supervise(p,10.13).empty());
 std::string failure;
 for(int i=1;i<=301;++i) {
  const double t=10.13+i*.1;ASSERT_TRUE(o.observe(sdk(t,i+2),t));ASSERT_TRUE(o.observe(proof(t,i+2),t));
  r.sequence=i+2;r.body_source_stamp=stamp(t);ASSERT_TRUE(o.observe(r,t));
  p=o.tick(t,true);failure=o.supervise(p,t);
  if(!failure.empty())break;
 }
 EXPECT_EQ(failure,"execution_blocked_timeout:braking_envelope_reentry_required");
 EXPECT_TRUE(o.stopping());EXPECT_FALSE(o.stopped());
}
TEST(ExecutionOwner, LiveGeometryPreparationDoesNotRequireSDKAuthorityOrFakeAcceptance){
 Owner o("live");o.bind(ver());auto r=receipt();auto p=proof();auto a=ack();
 r.transport_mode=p.transport_mode=a.transport_mode="live";
 ASSERT_TRUE(o.observe(r,10.));ASSERT_TRUE(o.observe(p,10.));ASSERT_TRUE(o.observe(a,10.));
 const auto geometry=o.tick(10.1,true);
 EXPECT_TRUE(geometry.geometry_committed);EXPECT_FALSE(geometry.allowed);
 EXPECT_EQ(geometry.phase,"preview");EXPECT_FALSE(o.confirmed());
 EXPECT_EQ(geometry.trajectory_id,1);EXPECT_TRUE(geometry.execution_id.empty());
 // Native evidence expiry never authorizes motion or pretends arrival.
 EXPECT_FALSE(o.tick(10.4,true).allowed);EXPECT_FALSE(o.goalStopped());
}
TEST(ExecutionOwner, SustainedBlockedExecutionStopsAtBoundAndStillRequiresMCProof){
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
 std::string failure;Permit emitted;
 for(int i=0;i<=301;++i){const double time=10.2+i*.1;
  ASSERT_TRUE(o.observe(sdk(time,i+2),time));
  auto invalid=proof(time,i+2);invalid.valid=false;invalid.reason="occupied";ASSERT_TRUE(o.observe(invalid,time));
  emitted=o.tick(time,true);failure=o.supervise(emitted,time);
  if(i<300){EXPECT_TRUE(failure.empty());EXPECT_FALSE(emitted.revoked);}
  if(!failure.empty())break;
 }
 EXPECT_EQ(failure,"execution_blocked_timeout:waiting_current_collision_and_tracker_proof");
 EXPECT_TRUE(o.stopping());EXPECT_FALSE(o.stopped());EXPECT_FALSE(o.goalStopped());
 EXPECT_TRUE(emitted.revoked);EXPECT_FALSE(emitted.allowed);
 auto late=proof(40.4,900);o.observe(late,40.4);EXPECT_FALSE(o.tick(40.4,true).allowed);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=1;s.stop_request_id="confirm";s.sequence=1;s.source_stamp=stamp(41.5);
 s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.stationary_samples=50;
 s.stationary_duration_sec=1.;s.mc_raw_stamp_ns=41500000000ULL;s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";
 EXPECT_TRUE(o.observe(s,41.5));EXPECT_TRUE(o.stopped());EXPECT_FALSE(o.goalStopped());
 EXPECT_EQ(o.tick(41.5,true).phase,"terminal");
}
TEST(ExecutionOwner, ShortGoodProofCannotRenewBlockedBudget){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.allowed=false;p.phase="holding";p.reason="occupied";
 auto negative=proof(10.1,2);negative.valid=false;negative.reason="occupied";ASSERT_TRUE(o.observe(negative,10.1));
 EXPECT_TRUE(o.supervise(p,10.1).empty());
 for(int i=1;i<300;++i){p.allowed=i%5==0;p.phase=p.allowed?"tracking":"holding";
  EXPECT_TRUE(o.supervise(p,10.1+i*.1).empty());}
 p.allowed=true;p.phase="tracking";EXPECT_FALSE(o.supervise(p,40.2).empty());
 EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);
}
TEST(ExecutionOwner, ContinuousPermissionRecoveryStartsANewBlockedEpisode){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.allowed=false;p.phase="holding";p.reason="occupied";
 auto negative=proof(10.1,2);negative.valid=false;negative.reason="occupied";ASSERT_TRUE(o.observe(negative,10.1));
 p.frame_id="d1max_loc_odom";auto b=body(10.1);o.supervise(p,10.1,&b);p.allowed=true;p.phase="tracking";
 for(int i=0;i<=15;++i){const double t=20.+i*.1;feedback(o,t,i+1,i*.02);b=body(t,i*.02);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());}
 p.allowed=false;p.phase="holding";EXPECT_TRUE(o.supervise(p,40.2).empty());
 EXPECT_TRUE(o.supervise(p,70.1).empty());EXPECT_FALSE(o.supervise(p,70.3).empty());
}
TEST(ExecutionOwner, PreviewAndArmingDoNotConsumeExecutionBlockedBudget){
 Owner o("isolated_mock");prepare(o);auto p=o.tick(10.1,true);
 EXPECT_TRUE(o.supervise(p,100.).empty());EXPECT_FALSE(o.stopping());
 ASSERT_TRUE(o.begin("execution","confirm",1,10.2));p=o.tick(10.2,true);
 EXPECT_TRUE(o.supervise(p,100.).empty());EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, FinalYawOrGoalHoldCannotBypassBlockedBudget){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);
 p.allowed=false;p.phase="aligning";p.reason="waiting_goal_yaw_swept_volume_proof";
 EXPECT_TRUE(o.supervise(p,10.1).empty());
 EXPECT_EQ(o.supervise(p,40.2),"execution_blocked_timeout:waiting_goal_yaw_swept_volume_proof");
 EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);
}
TEST(ExecutionOwner, UnboundedBlockedBudgetConfigurationIsRejected){
 EXPECT_THROW(Owner("live",0.,1.),std::invalid_argument);
 EXPECT_THROW(Owner("live",std::numeric_limits<double>::quiet_NaN(),1.),std::invalid_argument);
 EXPECT_THROW(Owner("live",30.,30.),std::invalid_argument);
 EXPECT_THROW(Owner("live",30.,0.),std::invalid_argument);
}
TEST(GoalLedger, PendingChurnCannotEvictCommittedFinalGoal){
 GoalLedger ledger;GoalLedger::Proposal p;p.version=ver();p.reference.path.header.frame_id="odom";p.goal_position.x=4.;
 ASSERT_TRUE(ledger.observe(p));ASSERT_NE(ledger.committed(ver()),nullptr);
 for(int i=2;i<50;++i){p.version.reference_generation=i;p.version.anchor_revision=i;ASSERT_TRUE(ledger.observe(p));}
 auto active=ledger.committed(ver());ASSERT_NE(active,nullptr);EXPECT_EQ(active->goal_position.x,4.);
 auto other=ver();other.task_id="other";EXPECT_EQ(ledger.committed(other),nullptr);
}
TEST(GoalLedger, FinalGoalIsImmutableWithinAnExactAnchorVersion){
 GoalLedger ledger;GoalLedger::Proposal p;p.version=ver();p.reference.path.header.frame_id="odom";p.goal_position.x=4.;
 ASSERT_TRUE(ledger.observe(p));ASSERT_NE(ledger.committed(ver()),nullptr);p.goal_position.x=8.;
 EXPECT_FALSE(ledger.observe(p));EXPECT_EQ(ledger.committed(ver())->goal_position.x,4.);
 ++p.version.reference_generation;++p.version.anchor_revision;ASSERT_TRUE(ledger.observe(p));
 ASSERT_NE(ledger.committed(p.version),nullptr);EXPECT_EQ(ledger.committed(p.version)->goal_position.x,8.);
}
TEST(GoalLedger, HistoricalDeliveryIsRouteBoundAndFullPayloadImmutable) {
 GoalLedger ledger;auto p=candidateGoal(ver());p.proposal_id="proposal";p.transport_mode="isolated_mock";
 p.source_stamp=stamp(10.);p.valid_until=stamp(10.4);
 geometry_msgs::msg::PoseStamped point;point.pose.orientation.w=1.;p.reference.path.poses.push_back(point);
 ASSERT_TRUE(historicalProposalMatchesRoute(p,routeSnapshot(),"isolated_mock",14.));
 ASSERT_TRUE(ledger.observe(p));ASSERT_TRUE(ledger.observe(p));
 ASSERT_NE(ledger.committed(p.version),nullptr);
 for(int mutation=0;mutation<8;++mutation) {
  auto wrong=p;
  if(mutation==0)wrong.reference.path.poses[0].pose.position.x+=.01;
  if(mutation==1)wrong.reference.path.poses[0].pose.orientation.z=.01;
  if(mutation==2)wrong.expected_trajectory_id++;
  if(mutation==3)wrong.proposal_id="same-version-different-id";
  if(mutation==4)wrong.source_stamp=stamp(10.1);
  if(mutation==5)wrong.valid_until=stamp(10.5);
  if(mutation==6)wrong.reference.point_segment_kinds.push_back("stairs");
  if(mutation==7)wrong.goal_yaw_tolerance_rad+=.01;
  EXPECT_FALSE(ledger.observe(wrong))<<mutation;
  EXPECT_EQ(*ledger.committed(p.version),p)<<mutation;
 }
 for(int mismatch=0;mismatch<9;++mismatch) {
  auto wrong=p;
  if(mismatch==0)wrong.version.task_id="retired-task";
  if(mismatch==1)wrong.version.route_hash=std::string(64,'b');
  if(mismatch==2)wrong.version.map_version_id="another-map";
  if(mismatch==3)wrong.version.localization_epoch++;
  if(mismatch==4)wrong.version.localization_seed_id="old-seed";
  if(mismatch==5)wrong.source_stamp=stamp(14.01);
  if(mismatch==6)wrong.valid_until=stamp(9.99);
  if(mismatch==7)wrong.valid_until=stamp(10.501);
  if(mismatch==8)wrong.transport_mode="live";
  EXPECT_FALSE(historicalProposalMatchesRoute(wrong,routeSnapshot(),"isolated_mock",14.))<<mismatch;
 }
}
TEST(BodyPairAdmission, OrderedUnusableWithdrawsButBadOrOldCannotErase){
 using State=d1max_planning_interfaces::msg::NavigationState;State old,m;
 m.schema_version=2;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(10);m.usable=true;
 m.local_odometry.header.stamp=m.global_odometry.header.stamp=m.source_stamp;
 m.local_odometry.header.frame_id="d1max_loc_odom";m.global_odometry.header.frame_id="d1max_loc_map";
 m.local_odometry.child_frame_id=m.global_odometry.child_frame_id="d1max_loc_base_link";
 m.local_odometry.pose.pose.orientation.w=m.global_odometry.pose.pose.orientation.w=1.;
 auto accept=[&](const State&s){return d1max_navigation_bt::acceptBodyPair(s,old,"nav","map",10.2);};
 ASSERT_TRUE(accept(m));old=m;EXPECT_FALSE(accept(m));
 m.source_stamp=m.local_odometry.header.stamp=m.global_odometry.header.stamp=stamp(10.1);m.usable=false;
 EXPECT_TRUE(accept(m));auto bad=m;bad.usable=true;bad.local_odometry.header.frame_id="map";EXPECT_FALSE(accept(bad));
 bad=m;bad.usable=true;bad.global_odometry.pose.pose.position.z=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(accept(bad));
 bad=m;bad.usable=true;bad.local_odometry.header.stamp=stamp(10.11);EXPECT_FALSE(accept(bad));
 bad=m;bad.localization_epoch=2;bad.localization_seed_id="reset";EXPECT_TRUE(accept(bad));
 bad=m;bad.session_id="foreign";EXPECT_FALSE(accept(bad));
}
TEST(BodyPairAdmission, EpochOrderPrecedesSourceWatermarkAndNeverReturnsToOldEpoch) {
 using State=d1max_planning_interfaces::msg::NavigationState;State old,m;
 m.schema_version=2;m.session_id="nav";m.map_version_id="map";m.localization_epoch=1;m.localization_seed_id="seed";
 m.source_stamp=m.posterior_stamp=m.imu_stamp=stamp(10.08);m.usable=true;
 m.local_odometry.header.stamp=m.global_odometry.header.stamp=m.source_stamp;
 m.local_odometry.header.frame_id="d1max_loc_odom";m.global_odometry.header.frame_id="d1max_loc_map";
 m.local_odometry.child_frame_id=m.global_odometry.child_frame_id="d1max_loc_base_link";
 m.local_odometry.pose.pose.orientation.w=m.global_odometry.pose.pose.orientation.w=1.;old=m;
 auto reset=m;reset.localization_epoch=2;reset.localization_seed_id="new";
 reset.source_stamp=reset.local_odometry.header.stamp=reset.global_odometry.header.stamp=stamp(10.03);
 ASSERT_TRUE(d1max_navigation_bt::acceptBodyPair(reset,old,"nav","map",10.1));old=reset;
 m.source_stamp=m.local_odometry.header.stamp=m.global_odometry.header.stamp=stamp(10.09);
 EXPECT_FALSE(d1max_navigation_bt::acceptBodyPair(m,old,"nav","map",10.1));
 EXPECT_FALSE(d1max_navigation_bt::acceptBodyPair(reset,old,"nav","map",10.1));
}
TEST(BodyPairAdmission, ProductionUnavailableEnvelopeIsNegativeEvidenceOnly) {
 using State=d1max_planning_interfaces::msg::NavigationState;
 auto old=body(10.08);old.global_odometry=old.local_odometry;old.global_odometry.header.frame_id="d1max_loc_map";
 State event;event.schema_version=2;event.session_id="nav";event.map_version_id="map";
 event.localization_epoch=1;event.localization_seed_id="seed";event.source_stamp=stamp(10.081);
 event.usable=false;event.reason="atomic_navigation_unavailable";
 const auto accept=[&](const State&m){return d1max_navigation_bt::acceptBodyPair(m,old,"nav","map",10.09);};
 ASSERT_TRUE(accept(event));
 event.reason="hard_localization_lost";ASSERT_TRUE(accept(event));
 auto positive=event;positive.usable=true;EXPECT_FALSE(accept(positive));
 for(int kind=0;kind<8;++kind) {
  auto bad=event;
  switch(kind) {
   case 0:bad.session_id="other";break;
   case 1:bad.map_version_id="other";break;
   case 2:bad.schema_version=1;break;
   case 3:bad.localization_epoch=0;break;
   case 4:bad.source_stamp=old.source_stamp;break;
   case 5:bad.source_stamp=stamp(9.);break;
   case 6:bad.source_stamp=stamp(10.1);break;
   case 7:bad.reason.assign(1025,'x');break;
  }
  EXPECT_FALSE(accept(bad))<<kind;
 }
 EXPECT_TRUE(old.usable);
 auto reset=event;reset.localization_epoch=2;reset.localization_seed_id="new";reset.reason="localization_reset";
 reset.source_stamp=stamp(10.03);ASSERT_TRUE(accept(reset));
 auto late=old;late.source_stamp=late.local_odometry.header.stamp=late.global_odometry.header.stamp=stamp(10.085);
 EXPECT_FALSE(d1max_navigation_bt::acceptBodyPair(late,reset,"nav","map",10.09));
 EXPECT_FALSE(d1max_navigation_bt::acceptBodyPair(reset,reset,"nav","map",10.09));
}
TEST(ExecutionOwner, RequiresBothFreshMatchingProofs){Owner o("isolated_mock");o.bind(ver());o.observe(proof(),10.);
 EXPECT_FALSE(o.canConfirm(10.));auto a=ack();a.validation_sequence=2;o.observe(a,10.);EXPECT_FALSE(o.canConfirm(10.));
 EXPECT_FALSE(o.begin("x","y",1,10.));}
TEST(ExecutionOwner, IsolatedAndLiveProofsNeverMix){Owner o("live");o.bind(ver());EXPECT_FALSE(o.observe(proof(),10.));
 EXPECT_FALSE(o.observe(ack(),10.));EXPECT_FALSE(o.begin("x","y",1,10.));}
TEST(ExecutionOwner, CommitNeedsGrantAndCurrentCollisionProof){Owner o("isolated_mock");prepare(o);o.begin("execution","confirm",1,10.);
 EXPECT_FALSE(o.tick(10.05,true).allowed);o.observe(sdk(),10.1);EXPECT_TRUE(o.tick(10.1,true).allowed);
 EXPECT_FALSE(o.tick(10.36,true).allowed);EXPECT_EQ(o.phase(),"holding");}
TEST(ExecutionOwner, BadCandidateDoesNotReplaceValidIncumbent){Owner o("isolated_mock");begin(o);EXPECT_EQ(o.tick(10.1,true).trajectory_id,1);
 auto p=proof(10.2,2,2);o.observe(p,10.2);auto a=ack(10.2,2,2);a.accepted=false;o.observe(a,10.2);
 EXPECT_EQ(o.tick(10.2,true).trajectory_id,1);a.accepted=true;a.checked_at=stamp(10.21);o.observe(a,10.21);
 EXPECT_EQ(o.tick(10.21,true).trajectory_id,1); // Preparation is not writer application.
 EXPECT_EQ(o.appliedCommitSequence(),1u);EXPECT_NE(o.pendingVersion(10.21),nullptr);}
TEST(ExecutionOwner, RevocationCannotBeUndoneByLateGrant){Owner o("isolated_mock");begin(o);o.stop("cancel",10.2);
 auto s=sdk(10.3,2);o.observe(s,10.3);auto p=o.tick(10.3,true);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);
 EXPECT_FALSE(o.begin("other","late",2,10.3));}
TEST(ExecutionOwner, StopSubmissionAndSoftwareResultAreNotStopProof){Owner o("isolated_mock");begin(o);o.stop("goal_reached",10.2,true);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=1;s.stop_request_id="confirm";s.sequence=1;s.mc_raw_stamp_ns=11300000000ULL;s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";
 s.source_stamp=stamp(11.3);s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.stationary_samples=50;
 s.stationary_duration_sec=.9;EXPECT_FALSE(o.observe(s,11.3));s.stationary_duration_sec=1.;EXPECT_TRUE(o.observe(s,11.3));
 EXPECT_FALSE(o.goalStopped());EXPECT_EQ(o.tick(11.3,true).phase,"verifying_arrival");
 o.measuredGoal(true,false,true,11.31,11.31);EXPECT_TRUE(o.goalStopped());EXPECT_EQ(o.tick(11.31,true).phase,"terminal");}
TEST(ExecutionOwner, NoStopProofBecomesNeedsReviewAndNoSuccess){Owner o("isolated_mock");begin(o);o.stop("cancel",10.2);
 auto p=o.tick(15.3,true);EXPECT_EQ(p.phase,"needs_review");EXPECT_FALSE(p.allowed);EXPECT_FALSE(o.stopped());}
TEST(ExecutionRetirement, FailedTakeControlBeforeFirstStateDoesNotInventMeasuredStop) {
 Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
 o.stop("sdk_grant_transport_failure",10.2);
 EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.executionSoftwareRetired());
 EXPECT_EQ(o.tick(15.3,true).phase,"needs_review");
 ASSERT_TRUE(o.observe(retirementWitness(15.35),15.35));
 EXPECT_TRUE(o.executionSoftwareRetired());EXPECT_FALSE(o.requiresPhysicalStop());
 EXPECT_FALSE(o.stopped());EXPECT_FALSE(o.goalStopped());EXPECT_FALSE(o.needsReview());
 EXPECT_EQ(o.failureReason(),"sdk_grant_transport_failure");
 const auto p=o.tick(30.,true);EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);
 EXPECT_EQ(p.phase,"terminal_no_motion");
 EXPECT_FALSE(o.begin("other","other-confirm",2,30.,"sdk"));
}
TEST(ExecutionRetirement, ExactLateWriterStateCannotResurrectRetiredPermission) {
 Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
 ASSERT_TRUE(o.observe(retirementWitness(),10.3));
 ASSERT_TRUE(o.observe(sdk(10.31),10.31));
 const auto p=o.tick(10.32,true);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);
 EXPECT_FALSE(o.requiresPhysicalStop());EXPECT_FALSE(o.stopped());
 EXPECT_EQ(p.phase,"terminal_no_motion");
}
TEST(ExecutionRetirement, RetiredWriterStateRevokesEvenIfRetirementTopicHasNotArrived) {
 Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
 auto s=sdk(10.3);s.phase="retired_without_motion";s.reason="take_control_failed";
 ASSERT_TRUE(o.observe(s,10.3));EXPECT_TRUE(o.stopping());
 EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.executionSoftwareRetired());
 EXPECT_FALSE(o.tick(10.3,true).allowed);EXPECT_TRUE(o.tick(10.3,true).revoked);
 ASSERT_TRUE(o.observe(retirementWitness(10.31),10.31));
 EXPECT_FALSE(o.requiresPhysicalStop());EXPECT_TRUE(o.executionSoftwareRetired());
 EXPECT_FALSE(o.stopped());
}
TEST(ExecutionRetirement, MeasuredStopAloneDoesNotClaimOutstandingTransactionsFinished) {
 Owner o("isolated_mock");begin(o);o.stop("cancel",10.2);
 ASSERT_TRUE(o.observe(stopWitness(),11.3));EXPECT_TRUE(o.stopped());
 EXPECT_FALSE(o.executionSoftwareRetired());
 auto r=retirementWitness(11.31);r.motion_never_submitted=false;
 ASSERT_TRUE(o.observe(r,11.31));EXPECT_TRUE(o.executionSoftwareRetired());EXPECT_TRUE(o.stopped());
}
TEST(ExecutionRetirement, WriterSoftwareExitDoesNotEraseActualMotionStopObligation) {
 Owner o("isolated_mock");begin(o);o.stop("cancel",10.2);
 auto r=retirementWitness();r.motion_never_submitted=false;
 ASSERT_TRUE(o.observe(r,10.3));EXPECT_TRUE(o.executionSoftwareRetired());
 EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.stopped());
 d1max_navigation_bt::LifecycleDrain drain;drain.configure();drain.activate();drain.begin();
 // Shutdown software drain and admission for the next motion are distinct.
 drain.observe(o.executionSoftwareRetired(),false);EXPECT_FALSE(drain.draining());
 EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.stopped());
 ASSERT_TRUE(o.observe(stopWitness(),11.3));EXPECT_TRUE(o.stopped());
}
TEST(ExecutionRetirement, RejectsForeignIncompletePendingAndStaleFacts) {
 for(int kind=0;kind<18;++kind) {
  Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
  o.stop("cancel",10.2);auto r=retirementWitness();double now=10.3;
  if(kind==0)r.schema_version=2;if(kind==1)r.version.task_id="other";
  if(kind==2)r.version.route_hash=std::string(64,'b');if(kind==3)r.version.localization_epoch=2;
  if(kind==4)r.version.anchor_id.clear();if(kind==5)r.execution_id="other";
  if(kind==6)r.control_epoch=2;if(kind==7)r.sdk_session="other";
  if(kind==8)r.sdk_arm_generation=0;if(kind==9)r.grant_request_id="old-confirm";
  if(kind==10)r.sequence=0;if(kind==11)r.transport_mode="live";
  if(kind==12)r.nonzero_blocked=false;if(kind==13)r.software_retired=false;
  if(kind==14)r.pending_sdk_transactions=1;if(kind==15)now=10.81;
  if(kind==16)r.source_stamp=stamp(9.9);if(kind==17)r.source_stamp=stamp(10.6);
  EXPECT_FALSE(o.observe(r,now))<<kind;EXPECT_TRUE(o.requiresPhysicalStop())<<kind;
  EXPECT_FALSE(o.executionSoftwareRetired())<<kind;EXPECT_FALSE(o.stopped())<<kind;
 }
}
TEST(ExecutionRetirement, RequiresKnownSessionAndNeverChangesRetiredGeneration) {
 Owner unknown("isolated_mock");prepare(unknown);ASSERT_TRUE(unknown.begin("execution","confirm",1,10.));
 EXPECT_FALSE(unknown.observe(retirementWitness(),10.3));
 Owner o("isolated_mock");beginUnapplied(o);o.stop("cancel",10.2);
 auto r=retirementWitness();r.sdk_arm_generation=2;EXPECT_FALSE(o.observe(r,10.3));
 r.sdk_arm_generation=1;ASSERT_TRUE(o.observe(r,10.3));
 EXPECT_FALSE(o.observe(r,10.3));r.sequence=2;r.motion_never_submitted=false;
 EXPECT_FALSE(o.observe(r,10.3));EXPECT_FALSE(o.requiresPhysicalStop());
}
TEST(ExecutionRetirement, ActualNonzeroSubmissionRejectsNoMotionCertificate) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 auto a=initialCommit(o,p,10.1);a.applied_velocity.linear.x=.2;
 ASSERT_TRUE(o.observe(a,10.1));o.stop("cancel",10.2);
 EXPECT_FALSE(o.observe(retirementWitness(),10.3));EXPECT_TRUE(o.requiresPhysicalStop());
 auto r=retirementWitness();r.motion_never_submitted=false;
 EXPECT_TRUE(o.observe(r,10.3));EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.stopped());
}
TEST(ExecutionRetirement, DelayedActualWriteContradictsNoMotionRatherThanReauthorizing) {
 Owner o("isolated_mock");beginUnapplied(o);const auto p=o.tick(10.1,true);
 auto a=initialCommit(o,p,10.1);a.applied_velocity.linear.x=.2;
 o.stop("cancel",10.2);ASSERT_TRUE(o.observe(retirementWitness(),10.3));
 ASSERT_TRUE(o.observe(a,10.31));EXPECT_TRUE(o.requiresPhysicalStop());
 EXPECT_FALSE(o.executionSoftwareRetired());EXPECT_FALSE(o.stopped());
 EXPECT_EQ(o.failureReason(),"writer_retirement_contradicted_by_nonzero_submission");
 EXPECT_FALSE(o.tick(10.32,true).allowed);
}
TEST(ExecutionGrantRejection, ExactLateNotAdmittedReplyReleasesOnlyThisAttempt) {
 Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
 o.stop("cancel",10.2);EXPECT_EQ(o.tick(15.3,true).phase,"needs_review");
 const auto req=grantRequest();ASSERT_TRUE(o.rejectUnadmittedGrant(req,rejectedGrant(req)));
 EXPECT_FALSE(o.confirmed());EXPECT_FALSE(o.requiresPhysicalStop());EXPECT_TRUE(o.executionSoftwareRetired());
 EXPECT_FALSE(o.stopped());EXPECT_FALSE(o.goalStopped());
 EXPECT_EQ(o.failureReason(),"sdk_grant_rejected:writer_not_ready_or_prior_not_stopped");
 EXPECT_FALSE(o.observe(sdk(15.4),15.4));
 EXPECT_FALSE(o.tick(15.4,true).allowed);EXPECT_TRUE(o.tick(15.4,true).revoked);
}
TEST(ExecutionGrantRejection, RefusalCannotRetireAnObservedOrDifferentGrant) {
 for(int kind=0;kind<8;++kind) {
  Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
  auto req=grantRequest();auto reply=rejectedGrant(req);
  if(kind==0)ASSERT_TRUE(o.observe(sdk(),10.1));
  if(kind==1)reply.definitely_not_admitted=false;if(kind==2)reply.duplicate=true;
  if(kind==3)reply.accepted=true;if(kind==4)reply.execution_id="old-execution";
  if(kind==5)reply.control_epoch=2;if(kind==6)reply.request_id="old-confirm";
  if(kind==7){req.sdk_session="foreign";reply.sdk_session="foreign";}
  EXPECT_FALSE(o.rejectUnadmittedGrant(req,reply))<<kind;
  EXPECT_TRUE(o.confirmed())<<kind;EXPECT_TRUE(o.requiresPhysicalStop())<<kind;
  EXPECT_FALSE(o.stopped())<<kind;
 }
}
TEST(ExecutionGrantAdmissionRetry, ExactUnadmittedTelemetryKeepsRouteAndFollowForManualAndGoalAuthorization) {
 for(bool goal_bound:{false,true})for(const std::string reason:{"robot_state_unavailable","mc_capture_unavailable"}) {
  Owner o("isolated_mock");prepare(o);
  const auto backend_owner=std::make_shared<ExecutionTaskBackend>(o);auto&backend=*backend_owner;
  d1max_navigation_bt::Engine engine(backend_owner,DEFAULT_BT_XML);
  engine.submit({"task","context"});for(int i=0;i<3;++i)ASSERT_EQ(engine.tick(),BT::NodeStatus::RUNNING);
  ASSERT_EQ(backend.routes,1);ASSERT_EQ(backend.follows,1);
  const d1max_navigation_bt::RouteBinding route{"nav","task","route",std::string(64,'a')};
  d1max_navigation_bt::GoalExecutionIntent intent(goal_bound,goal_bound?"clicked-goal":"");
  const std::string original_request=goal_bound?intent.requestId():"manual-execute";
  d1max_navigation_bt::ConfirmationLedger authorization;
  const auto approved=authorization.confirm(d1max_navigation_bt::task_schema_version,original_request,
    route,route,true,true,true,false,true);
  ASSERT_TRUE(approved.authorized);
  if(goal_bound)ASSERT_TRUE(intent.consume(route));
  ASSERT_TRUE(o.begin("execution",approved.confirmation_id,1,10.,"sdk"));
  auto req=grantRequest();req.confirmation_id=req.request_id=approved.confirmation_id;
  auto rejected=rejectedGrant(req);rejected.reason=reason;
  ASSERT_TRUE(o.waitForUnadmittedTelemetryGrant(req,rejected));
  EXPECT_TRUE(authorization.authorized());EXPECT_EQ(intent.consumed(),goal_bound);
  EXPECT_FALSE(o.confirmed());EXPECT_FALSE(o.requiresPhysicalStop());EXPECT_FALSE(o.stopped());
  EXPECT_TRUE(o.waitingUnadmittedGrant());EXPECT_TRUE(o.failureReason().empty());
  auto preview=o.tick(10.1,true);EXPECT_FALSE(preview.allowed);EXPECT_FALSE(preview.revoked);
  EXPECT_TRUE(preview.geometry_committed);EXPECT_EQ(preview.reason,"waiting_sdk_telemetry:"+reason);
  EXPECT_EQ(engine.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(backend.follow_halts,0);
  EXPECT_EQ(backend.routes,1);EXPECT_EQ(backend.follows,1);EXPECT_EQ(backend.finished,0);
  // Retry uses the already-authorized original request, not a fresh click or
  // a new route. Its new epoch excludes replies/status from the prior attempt.
  const auto duplicate=authorization.confirm(d1max_navigation_bt::task_schema_version,original_request,
    route,route,true,true,true,false,true);
  ASSERT_TRUE(duplicate.authorized);ASSERT_TRUE(duplicate.duplicate);
  EXPECT_EQ(duplicate.confirmation_id,approved.confirmation_id);
  EXPECT_FALSE(o.canConfirm(10.5));EXPECT_FALSE(o.begin("execution",approved.confirmation_id,2,10.5,"sdk"));
  ASSERT_TRUE(o.observe(proof(10.6,2),10.6));ASSERT_TRUE(o.observe(ack(10.6,2),10.6));
  ASSERT_TRUE(o.begin("execution",approved.confirmation_id,2,10.6,"sdk"));
  EXPECT_FALSE(o.waitingUnadmittedGrant());
  EXPECT_FALSE(o.waitForUnadmittedTelemetryGrant(req,rejected));EXPECT_TRUE(o.confirmed());
  EXPECT_FALSE(o.observe(sdk(10.61),10.61));
  auto idle=sdk(10.61);idle.execution_id.clear();idle.control_epoch=0;idle.phase="idle";
  idle.grant_ready=idle.control_owned=false;EXPECT_FALSE(o.observe(idle,10.61));
  EXPECT_FALSE(o.stopping());EXPECT_TRUE(o.failureReason().empty());
  auto current_sdk=sdk(10.61);current_sdk.control_epoch=2;ASSERT_TRUE(o.observe(current_sdk,10.61));
  EXPECT_TRUE(o.tick(10.61,true).allowed);EXPECT_TRUE(o.failureReason().empty());
  EXPECT_EQ(engine.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(backend.follow_halts,0);
  EXPECT_EQ(backend.routes,1);EXPECT_EQ(backend.follows,1);
 }
}
TEST(ExecutionGrantAdmissionRetry, OnlyPreciseSideEffectFreeTelemetryRejectionCanWait) {
 for(const std::string reason:{"writer_not_ready_or_prior_not_stopped","writer_disconnected",
   "prior_sdk_transactions_unretired","prior_stop_unconfirmed","robot_mode_unsupported",
   "robot_estop_active","writer_fault_latched","take_control_timeout",""}) {
  Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
  const auto req=grantRequest();auto rejected=rejectedGrant(req);rejected.reason=reason;
  EXPECT_FALSE(o.waitForUnadmittedTelemetryGrant(req,rejected))<<reason;
  EXPECT_TRUE(o.confirmed());EXPECT_TRUE(o.requiresPhysicalStop());EXPECT_FALSE(o.waitingUnadmittedGrant());
 }
}
TEST(ExecutionGrantAdmissionRetry, ForeignAmbiguousOrObservedGrantCannotBeRolledBack) {
 for(int kind=0;kind<23;++kind) {
  Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
  auto req=grantRequest();auto rejected=rejectedGrant(req);rejected.reason="mc_capture_unavailable";
  if(kind==0)ASSERT_TRUE(o.observe(sdk(),10.1));
  if(kind==1)rejected.definitely_not_admitted=false;if(kind==2)rejected.duplicate=true;
  if(kind==3)rejected.accepted=true;if(kind==4)rejected.execution_id="foreign";
  if(kind==5)rejected.control_epoch=2;if(kind==6)rejected.request_id="foreign";
  if(kind==7){req.sdk_session="foreign";rejected.sdk_session="foreign";}
  if(kind==8)rejected.version.task_id="foreign";if(kind==9)rejected.sdk_session="foreign";
  if(kind==10)rejected.transport_mode="live";if(kind==11)req.activate=false;
  if(kind==12)req.confirmation_id="foreign";if(kind==13)req.request_id="foreign";
  if(kind==14)req.execution_id="foreign";if(kind==15)req.control_epoch=2;
  if(kind==16){req.version.localization_epoch=2;rejected.version=req.version;}
  if(kind==17){req.version.localization_seed_id="reset";rejected.version=req.version;}
  if(kind==18){req.version.anchor_revision=2;rejected.version=req.version;}
  if(kind==19){req.version.reference_generation=2;rejected.version=req.version;}
  if(kind==20){req.version.route_hash=std::string(64,'b');rejected.version=req.version;}
  if(kind==21){req.version.context_sequence=2;rejected.version=req.version;}
  if(kind==22){req.version.anchor_id.clear();rejected.version=req.version;}
  EXPECT_FALSE(o.waitForUnadmittedTelemetryGrant(req,rejected))<<kind;
  EXPECT_TRUE(o.confirmed())<<kind;EXPECT_TRUE(o.requiresPhysicalStop())<<kind;
  EXPECT_FALSE(o.waitingUnadmittedGrant())<<kind;
 }
}
TEST(ExecutionGrantAdmissionRetry, CancelAndReplacementFenceLateTelemetryReplies) {
 for(const std::string reason:{"cancel","task_retired","localization_context_replaced"}) {
  Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
  const auto req=grantRequest();auto rejected=rejectedGrant(req);rejected.reason="robot_state_unavailable";
  o.stop(reason,10.1);EXPECT_FALSE(o.waitForUnadmittedTelemetryGrant(req,rejected));
  EXPECT_TRUE(o.stopping());EXPECT_FALSE(o.tick(10.1,true).allowed);
 }
 Owner old("isolated_mock");prepare(old);ASSERT_TRUE(old.begin("execution","confirm",1,10.,"sdk"));
 const auto req=grantRequest();auto rejected=rejectedGrant(req);rejected.reason="mc_capture_unavailable";
 ASSERT_TRUE(old.waitForUnadmittedTelemetryGrant(req,rejected));old.stop("task_retired",10.1);
 EXPECT_FALSE(old.waitingUnadmittedGrant());EXPECT_FALSE(old.begin("execution","confirm",2,10.1,"sdk"));
 Owner next("isolated_mock");auto identity=ver();identity.task_id="next-task";next.bind(identity);
 EXPECT_FALSE(next.waitForUnadmittedTelemetryGrant(req,rejected));EXPECT_FALSE(next.confirmed());
 EXPECT_FALSE(next.tick(10.1,true).allowed);
}
TEST(ExecutionGrantAdmissionRetry, SdkWaitDoesNotRestartInitialReferenceOrTrajectoryBudget) {
 for(const auto stage:{PreparationStage::Reference,PreparationStage::Trajectory}) {
  d1max_navigation_bt::GrantAdmissionWait admission;ASSERT_TRUE(admission.start(100.));
  PreparationBudget initial;std::string problem;
  for(int i=0;i<60;++i) {
   const double now=100.+i*.5;
   if(d1max_navigation_bt::initialPreparationBudgetApplies(false,true,admission.pending()))
     problem=initial.observe(stage,now,0);
   EXPECT_TRUE(problem.empty());EXPECT_FALSE(admission.expired(now,30.));
   ASSERT_TRUE(admission.attempted(now));
  }
  // A good last packet cannot renew the shared admission deadline. The caller
  // must retire at this policy boundary, not fabricate a prepared/ready phase.
  EXPECT_TRUE(admission.expired(130.,30.));
  EXPECT_TRUE(d1max_navigation_bt::initialPreparationBudgetApplies(false,false,true));
  EXPECT_TRUE(d1max_navigation_bt::initialPreparationBudgetApplies(false,true,false));
  EXPECT_FALSE(d1max_navigation_bt::initialPreparationBudgetApplies(true,false,false));
  // Genuine initial preparation (no prior authorization) retains its original
  // shorter phase budgets; this change is not a broad removal of timeouts.
  PreparationBudget never_prepared;EXPECT_TRUE(never_prepared.observe(stage,100.).empty());
  EXPECT_FALSE(never_prepared.observe(stage,stage==PreparationStage::Reference?105.:120.).empty());
 }
}
TEST(ExecutionGrantAdmissionRetry, LateAdmittedSdkStateAfterCancelKeepsStopObligationWithoutResuming) {
 Owner o("isolated_mock");prepare(o);ASSERT_TRUE(o.begin("execution","confirm",1,10.,"sdk"));
 o.stop("cancel",10.1);ASSERT_TRUE(o.observe(sdk(10.11),10.11));
 EXPECT_TRUE(o.stopping());EXPECT_TRUE(o.requiresPhysicalStop());
 auto p=o.tick(10.12,true);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);
 EXPECT_FALSE(o.begin("execution","confirm",2,10.12,"sdk"));
}
TEST(ExecutionOwner, LiveMeasuredStopDoesNotDependOnPhysicalAcceptanceReport) {
 for(bool reported:{false,true}) {
  Owner o("live");begin(o,"live");o.stop("goal_reached",10.2,true);
  auto s=stopWitness("live");s.physical_acceptance_verified=reported;
  ASSERT_TRUE(o.observe(s,11.3));EXPECT_EQ(s.physical_acceptance_verified,reported);
  EXPECT_TRUE(o.stopped());EXPECT_FALSE(o.goalStopped());
  o.measuredGoal(true,false,true,11.31,11.31);EXPECT_TRUE(o.goalStopped());
 }
}
TEST(ExecutionOwner, LiveRuntimeStopStillRejectsStaleForeignAndUnmeasuredEvidence) {
 for(int kind=0;kind<10;++kind) {
  Owner o("live");begin(o,"live");o.stop("cancel",10.2);auto s=stopWitness("live");
  ASSERT_FALSE(s.physical_acceptance_verified);double now=11.3;
  if(kind==0)now=11.61;if(kind==1)s.sdk_session="foreign";
  if(kind==2)s.version.localization_epoch+=1;if(kind==3)s.mc_raw_stamp_ns=0;
  if(kind==4)s.measured_stop_confirmed=false;if(kind==5)s.measured_linear_mps=.031;
  if(kind==6)s.stationary_duration_sec=.9;if(kind==7)s.stationary_samples=2;
  if(kind==8)s.stop_request_id="foreign";if(kind==9)s.time_basis="isolated_simulated_source_clock";
  EXPECT_FALSE(o.observe(s,now))<<kind;EXPECT_FALSE(o.stopped());
 }
}
TEST(ExecutionOwner, NoReferenceReceiptDoesNotCommit){Owner o("isolated_mock");o.bind(ver());o.observe(proof(),10.);o.observe(ack(),10.);
 EXPECT_FALSE(o.canConfirm(10.1));EXPECT_FALSE(o.tick(10.1,true).geometry_committed);}
TEST(ExecutionOwner, IncorrectExpectedRevisionDoesNotCommit){Owner o("isolated_mock");o.bind(ver());auto r=receipt();r.expected_version=ver();
 r.expected_trajectory_id=12;o.observe(r,10.);o.observe(proof(),10.);o.observe(ack(),10.);EXPECT_FALSE(o.canConfirm(10.1));}
TEST(ExecutionOwner, LaterInvalidProofCannotBeHiddenByOlderValidProof){Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
 auto p=proof(10.2,2);p.valid=false;ASSERT_TRUE(o.observe(p,10.2));EXPECT_FALSE(o.tick(10.2,true).allowed);}
TEST(ExecutionOwner, AuthorityLeaseIsIndependentButSigningRequiresFreshEvidence){Owner o("isolated_mock");prepare(o);auto p=proof(10.02,2);p.valid_until=stamp(10.12);
 o.observe(p,10.02);auto a=ack(10.02,2);o.observe(a,10.02);ASSERT_TRUE(o.begin("execution","confirm",1,10.03));o.observe(sdk(),10.1);
 auto lease=o.tick(10.1,true);EXPECT_NEAR(seconds(lease.valid_until),10.35,1e-8);EXPECT_EQ(lease.validation_sequence,2u);
 ASSERT_TRUE(o.observe(initialCommit(o,lease,10.11),10.11));
 // No fresh proof still prevents another allowed lease; the already signed
 // lease cannot bypass consumers' independent proof checks.
 EXPECT_FALSE(o.tick(10.13,true).allowed);
 auto newer=proof(10.14,3);ASSERT_TRUE(o.observe(newer,10.14));
 lease=o.tick(10.14,true);EXPECT_TRUE(lease.allowed);EXPECT_EQ(lease.validation_sequence,3u);
 EXPECT_NEAR(seconds(lease.valid_until),10.39,1e-8);}
TEST(ExecutionOwner, AligningRequiresMatchingNativeSweepEvidence){Owner o("isolated_mock");begin(o);o.tick(10.1,true);
 EXPECT_FALSE(o.yawProof(.5,10.1));auto p=proof(10.2,2);p.goal_yaw_checked=true;p.checked_goal_yaw=.5;o.observe(p,10.2);
 EXPECT_TRUE(o.yawProof(.5,10.2));EXPECT_FALSE(o.yawProof(.6,10.2));}
TEST(ExecutionOwner, SDKGrantWithdrawalStopsEvenWhileStillOwned){Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
 auto s=sdk(10.2,2);s.grant_ready=false;s.control_owned=true;s.phase="stopping";s.reason="mc_stale";
 ASSERT_TRUE(o.observe(s,10.2));auto p=o.tick(10.2,true);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);}
TEST(ExecutionOwner, ForeignStopGenerationAndMissingRawClockAreRejected){Owner o("isolated_mock");begin(o);o.stop("cancel",10.2);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=2;s.stop_request_id="confirm";s.sequence=1;s.source_stamp=stamp(11.3);s.stationary_duration_sec=1.;
 s.stationary_samples=50;s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.mc_raw_stamp_ns=11300000000ULL;
 s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";EXPECT_FALSE(o.observe(s,11.3));
 s.sdk_arm_generation=1;s.mc_raw_stamp_ns=0;EXPECT_FALSE(o.observe(s,11.3));s.mc_raw_stamp_ns=11300000000ULL;
 s.stop_request_id="earlier";EXPECT_FALSE(o.observe(s,11.3));s.stop_request_id="confirm";EXPECT_TRUE(o.observe(s,11.3));}
TEST(ExecutionOwner, CalibratedStationaryNoiseUsesSameConfiguredBoundWithoutChangingDefault) {
 StationaryLimits calibrated;calibrated.linear=.04;calibrated.angular=.08;
 Owner strict("isolated_mock"),measured("isolated_mock",30.,1.,calibrated);
 begin(strict);begin(measured);strict.stop("cancel",10.2);measured.stop("cancel",10.2);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=1;s.stop_request_id="confirm";s.sequence=1;s.source_stamp=stamp(11.3);s.stationary_duration_sec=1.;
 s.stationary_samples=50;s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.mc_raw_stamp_ns=11300000000ULL;
 s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";s.measured_linear_mps=.035;s.measured_angular_radps=.06;
 EXPECT_FALSE(strict.observe(s,11.3));EXPECT_TRUE(measured.observe(s,11.3));
}
TEST(ExecutionOwner, RecordStricterDurationAndSamplesCannotBeBypassed) {
 StationaryLimits calibrated;calibrated.stop_duration=1.5;calibrated.minimum_samples=80;
 Owner o("isolated_mock",30.,1.,calibrated);begin(o);o.stop("cancel",10.2);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=1;s.stop_request_id="confirm";s.sequence=1;s.source_stamp=stamp(12.);s.stationary_duration_sec=1.;
 s.stationary_samples=50;s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.mc_raw_stamp_ns=12000000000ULL;
 s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";
 EXPECT_FALSE(o.observe(s,12.));s.stationary_duration_sec=1.5;EXPECT_FALSE(o.observe(s,12.));
 s.stationary_samples=80;EXPECT_TRUE(o.observe(s,12.));
}
TEST(ExecutionOwner, StationaryCalibrationBoundsDoNotPermitUnboundedTolerance) {
 StationaryLimits invalid;invalid.linear=.051;EXPECT_THROW((Owner("isolated_mock",30.,1.,invalid)),std::invalid_argument);
 invalid={};invalid.angular=.101;EXPECT_THROW((Owner("isolated_mock",30.,1.,invalid)),std::invalid_argument);
 invalid={};invalid.minimum_samples=2;EXPECT_THROW((Owner("isolated_mock",30.,1.,invalid)),std::invalid_argument);
}
TEST(ExecutionOwner, StoppedOutsideFinalToleranceIsNotGoalSuccess){Owner o("isolated_mock");begin(o);o.stop("goal_reached",10.2,true);
 Stop s;s.version=ver();s.execution_id="execution";s.control_epoch=1;s.sdk_session="sdk";s.transport_mode="isolated_mock";
 s.sdk_arm_generation=1;s.stop_request_id="confirm";s.sequence=1;s.source_stamp=stamp(11.3);s.stationary_duration_sec=1.;
 s.stationary_samples=50;s.nonzero_blocked=s.stop_submitted=s.measured_stop_confirmed=true;s.mc_raw_stamp_ns=11300000000ULL;
 s.mc_clock_epoch="clock";s.time_basis="isolated_simulated_source_clock";ASSERT_TRUE(o.observe(s,11.3));
 o.measuredGoal(true,false,true,11.31,11.2);EXPECT_FALSE(o.goalStopped()); // Pre-stop pose is not proof.
 o.measuredGoal(false,false,true,11.32,11.32);EXPECT_FALSE(o.goalStopped());EXPECT_TRUE(o.needsReview());EXPECT_TRUE(o.stopped());}
TEST(ExecutionOwner, NewCurveCommitUsesExactlyPreparedNativeSequence){Owner o("isolated_mock");beginUnapplied(o);ASSERT_EQ(o.tick(10.1,true).trajectory_id,1);
 ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.15,2,2),10.15);o.observe(ack(10.15,2,2),10.15);o.observe(proof(10.16,3,2),10.16);
 auto p=o.tick(10.17,true);p.frame_id="d1max_loc_odom";EXPECT_EQ(p.trajectory_id,1);
 ASSERT_NE(o.pendingVersion(10.17),nullptr);ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.17)),100,10.17));
 ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(o.handoff()->candidate.trajectory_id,2);
 EXPECT_EQ(o.handoff()->candidate.validation_sequence,2u);EXPECT_FALSE(o.handoff()->candidate.geometry_committed);
 EXPECT_TRUE(o.handoff()->candidate.allowed);
 ASSERT_TRUE(o.observe(handoffCommit(*o.handoff(),10.18),10.18));
 EXPECT_EQ(o.tick(10.19,true).trajectory_id,2);EXPECT_EQ(o.appliedCommitSequence(),2u);}
TEST(ExecutionHandoff, InitialCommitRequiresAnActuallyPublishedLeaseAndWriterSubmission) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 auto a=initialCommit(o,p);auto foreign=a;foreign.permit_sequence+=100;
 EXPECT_FALSE(o.observe(foreign,10.11));foreign=a;foreign.control_epoch++;
 EXPECT_FALSE(o.observe(foreign,10.11));foreign=a;foreign.measured_pose.header.stamp=stamp(10.11);
 EXPECT_FALSE(o.observe(foreign,10.11));foreign=a;foreign.previous_commit_sequence=1;
 EXPECT_FALSE(o.observe(foreign,10.11));ASSERT_TRUE(o.observe(a,10.11));
 EXPECT_EQ(o.appliedCommitSequence(),1u);EXPECT_FALSE(o.observe(a,10.12));
}
TEST(ExecutionOwner, NativeNegativeWithZeroDeadlineImmediatelyFencesPriorPositive) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
 auto invalid=proof(10.11,2);invalid.valid=false;invalid.valid_until=stamp(0.);
 ASSERT_TRUE(o.observe(invalid,10.11));
 auto p=o.tick(10.11,true);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.geometry_committed);
 EXPECT_EQ(p.reason,"waiting_current_collision_and_tracker_proof");
 EXPECT_FALSE(o.stopping());
 // Duplicate or late older positive evidence cannot undo a newer negative.
 EXPECT_FALSE(o.observe(proof(10.12,1),10.12));EXPECT_FALSE(o.tick(10.12,true).allowed);
 auto newer=proof(10.13,3);ASSERT_TRUE(o.observe(newer,10.13));EXPECT_TRUE(o.tick(10.13,true).allowed);
}
TEST(ExecutionOwner, NegativeEvidenceStillRequiresFreshMatchingDecision) {
 for(int kind=0;kind<4;++kind) {
  Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
  auto invalid=proof(10.11,2);invalid.valid=false;invalid.valid_until=stamp(0.);
  if(kind==0)invalid.source_stamp=stamp(9.);
  if(kind==1)invalid.version.localization_epoch++;
  if(kind==2)invalid.check_end=stamp(9.);
  if(kind==3)invalid.sequence=0;
  EXPECT_FALSE(o.observe(invalid,10.11));EXPECT_TRUE(o.tick(10.11,true).allowed);
 }
}
TEST(ExecutionOwner, ScanNegativeImmediatelyFencesWithoutRecheckingSensorAges) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.tick(10.1,true).allowed);
 auto invalid=proof(10.11,2);invalid.valid=false;invalid.valid_until=stamp(0.);
 invalid.body_source_stamp=invalid.front_ray_source_stamp=invalid.rear_ray_source_stamp=stamp(9.);
 ASSERT_TRUE(o.observe(invalid,10.11));EXPECT_FALSE(o.tick(10.11,true).allowed);
}
TEST(ExecutionOwner, MotionProofDelegatesPerceptionAgeButRetainsCommandBodyTTL) {
 Owner o("isolated_mock");begin(o);const auto d=movingDemand(10.2,1);auto m=motionProof(d);
 m.body_source_stamp=m.front_ray_source_stamp=m.rear_ray_source_stamp=stamp(9.);
 ASSERT_TRUE(o.observe(d,10.2));ASSERT_TRUE(o.observe(m,10.2));
 auto expired=motionProof(movingDemand(10.21,2));expired.demand_body_source_stamp=stamp(9.);
 EXPECT_FALSE(o.observe(expired,10.21));
 expired=motionProof(movingDemand(10.22,2));expired.valid_until=stamp(10.21);
 EXPECT_FALSE(o.observe(expired,10.22));
}
TEST(ExecutionOwner, CandidateSelectionIsLexicographicNotFastLaneSequencePriority) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));
 auto r=receipt();r.version.reference_generation=2;r.proposal_id="next-reference";
 r.expected_version=ver();r.expected_trajectory_id=1;r.source_stamp=stamp(10.2);r.valid_until=stamp(10.5);
 ASSERT_TRUE(o.observe(r,10.2));
 auto next=proof(10.2,2,2);next.version=r.version;next.proposal_id=r.proposal_id;
 ASSERT_TRUE(o.observe(next,10.2));
 auto admitted=ack(10.2,2,2);admitted.version=r.version;ASSERT_TRUE(o.observe(admitted,10.2));
 // The old curve's fast safety lane can have a higher global validation seq.
 ASSERT_TRUE(o.observe(proof(10.21,100,1),10.21));ASSERT_TRUE(o.observe(ack(10.21,100,1),10.21));
 p=o.tick(10.22,true);ASSERT_TRUE(p.allowed);p.frame_id="d1max_loc_odom";
 const auto* chosen=o.pendingVersion(10.22);ASSERT_NE(chosen,nullptr);
 EXPECT_EQ(chosen->reference_generation,2u);
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*chosen),500,10.22));
 ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(o.handoff()->candidate.trajectory_id,2);
 EXPECT_EQ(o.handoff()->candidate.validation_sequence,2u);
}
TEST(ExecutionHandoff, StationaryReentryRequiresSDKWitnessAndNeverRestoresUnsafeIncumbentPermission) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);
 p=o.tick(11.01,true);ASSERT_FALSE(p.allowed);ASSERT_TRUE(p.geometry_committed);
 EXPECT_EQ(o.pendingVersion(11.01),nullptr); // LIO velocity/zero output alone cannot unlock.
 ASSERT_TRUE(o.observe(stationaryWitness(),11.01));
 const auto* next=o.pendingVersion(11.01);ASSERT_NE(next,nullptr);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*next),100,11.01));
 const auto grant=*o.handoff();EXPECT_EQ(grant.schema_version,2u);
 EXPECT_EQ(grant.transition_mode,Handoff::STATIONARY_REENTRY);EXPECT_FALSE(grant.incumbent.allowed);
 EXPECT_EQ(grant.incumbent.phase,"holding");EXPECT_EQ(grant.retain_incumbent_until,grant.source_stamp);
 EXPECT_TRUE(grant.candidate.allowed);EXPECT_FALSE(grant.candidate.geometry_committed);
 EXPECT_EQ(grant.stationary_evidence.source_stamp,stationaryWitness().source_stamp);
 EXPECT_LE(seconds(grant.valid_until),seconds(grant.stationary_evidence.valid_until));
 auto holding=o.tick(11.02,true);EXPECT_FALSE(holding.allowed);EXPECT_EQ(holding.trajectory_id,1);
 o.notePublication(holding,11.02);ASSERT_NE(o.handoff(),nullptr);EXPECT_FALSE(o.handoff()->revoked);
 ASSERT_TRUE(o.observe(handoffCommit(grant,11.05),11.05));
 auto switched=o.tick(11.06,true);EXPECT_TRUE(switched.allowed);EXPECT_EQ(switched.trajectory_id,2);
 EXPECT_EQ(o.appliedCommitSequence(),2u);EXPECT_FALSE(o.stopping());
}
TEST(ExecutionHandoff, StationaryEvidenceFailuresCannotAuthorizeNewGeometry) {
 for(int kind=0;kind<11;++kind) {
  Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
  ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
  auto witness=stationaryWitness();
  if(kind==0)witness.stationary_samples=2;if(kind==1)witness.stationary_duration_sec=.599;
  if(kind==2)witness.measured_linear_mps=.031;if(kind==3)witness.measured_angular_radps=.051;
  if(kind==4)witness.nonzero_blocked=false;if(kind==5)witness.writer_commit_sequence=2;
  if(kind==6)witness.mc_raw_stamp_ns=0;if(kind==7)witness.mc_clock_epoch.clear();
  if(kind==8)witness.capture_lower_bound=witness.zero_ack_at;
  if(kind==9)witness.valid_until=stamp(11.5);
  if(kind==10)witness.zero_ack_at=stamp(10.);
  EXPECT_FALSE(o.observe(witness,11.01));EXPECT_EQ(o.pendingVersion(11.01),nullptr);
  EXPECT_FALSE(o.tick(11.02,true).allowed);EXPECT_FALSE(o.stopping());
 }
}
TEST(ExecutionHandoff, LiveRuntimeStationaryWitnessCanReenterButCannotRestoreUnsafeOldCurve) {
 Owner o("live");beginUnapplied(o,"live");auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o,11.,"live");
 p=o.tick(11.01,true);ASSERT_FALSE(p.allowed);ASSERT_TRUE(p.geometry_committed);
 auto witness=stationaryWitness(11.,1,"live");ASSERT_FALSE(witness.physical_acceptance_verified);
 ASSERT_TRUE(o.observe(witness,11.01));const auto* next=o.pendingVersion(11.01);ASSERT_NE(next,nullptr);
 p.frame_id="d1max_loc_odom";ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*next),100,11.01));
 const auto grant=*o.handoff();EXPECT_EQ(grant.transition_mode,Handoff::STATIONARY_REENTRY);
 EXPECT_FALSE(grant.stationary_evidence.physical_acceptance_verified);
 EXPECT_EQ(grant.stationary_evidence.source_stamp,witness.source_stamp);
 EXPECT_EQ(grant.stationary_evidence.valid_until,witness.valid_until);
 EXPECT_FALSE(grant.incumbent.allowed);EXPECT_FALSE(o.tick(11.02,true).allowed);
 ASSERT_TRUE(o.observe(handoffCommit(grant,11.05),11.05));
 EXPECT_TRUE(o.tick(11.06,true).allowed);EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionHandoff, LiveRuntimeStationaryReentryStillRejectsStaleAndForeignEvidence) {
 for(int kind=0;kind<9;++kind) {
  Owner o("live");begin(o,"live");stationaryCandidate(o,11.,"live");o.tick(11.01,true);
  auto s=stationaryWitness(11.,1,"live");ASSERT_FALSE(s.physical_acceptance_verified);
  if(kind==0)s.source_stamp=stamp(10.7);if(kind==1)s.received_stamp=stamp(10.7);
  if(kind==2)s.sdk_session="foreign";if(kind==3)s.version.localization_epoch+=1;
  if(kind==4)s.usable=false;if(kind==5)s.writer_commit_sequence+=1;
  if(kind==6)s.measured_linear_mps=.031;if(kind==7)s.zero_ack_at=s.capture_lower_bound;
  if(kind==8)s.mc_raw_stamp_ns=0;
  EXPECT_FALSE(o.observe(s,11.01))<<kind;EXPECT_EQ(o.pendingVersion(11.01),nullptr);
  EXPECT_FALSE(o.tick(11.02,true).allowed);
 }
}
TEST(ExecutionHandoff, LiveMappedFutureMcSourceKeepsOriginalReceiptCaptureAndDeadline) {
 Owner o("live");begin(o,"live");stationaryCandidate(o,11.,"live");auto p=o.tick(11.02,true);
 auto s=stationaryWitness(11.,1,"live");
 s.source_stamp=stamp(11.07);s.received_stamp=stamp(11.01);s.mc_capture_delay_bound_sec=.25;
 s.capture_lower_bound=stamp(10.76);s.capture_upper_bound=s.received_stamp;s.valid_until=stamp(11.26);
 ASSERT_TRUE(o.observe(s,11.02));const auto*next=o.pendingVersion(11.02);ASSERT_NE(next,nullptr);
 p.frame_id="d1max_loc_odom";ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*next),100,11.02));
 const auto& kept=o.handoff()->stationary_evidence;
 EXPECT_EQ(kept.source_stamp,s.source_stamp);EXPECT_EQ(kept.received_stamp,s.received_stamp);
 EXPECT_EQ(kept.capture_lower_bound,s.capture_lower_bound);EXPECT_EQ(kept.capture_upper_bound,s.capture_upper_bound);
 EXPECT_EQ(kept.valid_until,s.valid_until);EXPECT_LE(seconds(o.handoff()->valid_until),11.26);
 EXPECT_FALSE(o.tick(11.03,true).allowed); // no unsafe incumbent revival
}
TEST(ExecutionHandoff, ApproximateMappingDoesNotForgiveExpiredOrPostZeroUnprovenWitnesses) {
 for(int kind=0;kind<8;++kind) {
  Owner o("live");begin(o,"live");stationaryCandidate(o,11.,"live");o.tick(11.02,true);
  auto s=stationaryWitness(11.,1,"live");
  s.source_stamp=stamp(11.07);s.received_stamp=stamp(11.01);s.mc_capture_delay_bound_sec=.25;
  s.capture_lower_bound=stamp(10.76);s.capture_upper_bound=s.received_stamp;s.valid_until=stamp(11.26);
  double at=11.02;
  if(kind==0)s.source_stamp=stamp(11.261);if(kind==1)at=11.261;
  if(kind==2)s.capture_lower_bound=s.zero_ack_at;if(kind==3)s.capture_upper_bound=s.source_stamp;
  if(kind==4)s.valid_until=stamp(11.32);if(kind==5)s.time_basis="isolated_simulated_source_clock";
  if(kind==6)s.mc_capture_delay_bound_sec=.251;if(kind==7)s.mc_raw_stamp_ns=0;
  EXPECT_FALSE(o.observe(s,at))<<kind;EXPECT_EQ(o.pendingVersion(at),nullptr)<<kind;
  EXPECT_FALSE(o.tick(at,true).allowed)<<kind;
 }
 Owner mock("isolated_mock");begin(mock);stationaryCandidate(mock);mock.tick(11.02,true);
 auto exact_clock=stationaryWitness();exact_clock.source_stamp=stamp(11.07);
 exact_clock.mc_capture_delay_bound_sec=.25;exact_clock.valid_until=stamp(11.26);
 EXPECT_FALSE(mock.observe(exact_clock,11.02));
}
TEST(ExecutionOwner, StopReportUsesOriginalMcReceiptWithoutMappedFutureAllowance) {
 Owner o("live");begin(o,"live");o.stop("cancel",10.2);
 auto s=stopWitness("live");s.source_stamp=stamp(11.36);
 EXPECT_FALSE(o.observe(s,11.3));EXPECT_FALSE(o.stopped());
 s.source_stamp=stamp(11.3);ASSERT_TRUE(o.observe(s,11.3));EXPECT_TRUE(o.stopped());
}
TEST(ExecutionHandoff, LiveRuntimeNewNegativeWitnessWithdrawsReentryWithoutOldProofResurrection) {
 Owner o("live");begin(o,"live");stationaryCandidate(o,11.,"live");auto p=o.tick(11.01,true);
 auto s=stationaryWitness(11.,1,"live");ASSERT_TRUE(o.observe(s,11.01));
 p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(11.01)),100,11.01));
 auto lost=stationaryWitness(11.02,2,"live");lost.usable=false;
 EXPECT_FALSE(o.observe(lost,11.03));ASSERT_NE(o.handoff(),nullptr);EXPECT_TRUE(o.handoff()->revoked);
 EXPECT_FALSE(o.observe(s,11.03));EXPECT_FALSE(o.tick(11.03,true).allowed);
}
TEST(ExecutionHandoff, LiveRuntimeWitnessCannotReplaceNegativeCandidateCollisionProof) {
 Owner o("live");begin(o,"live");stationaryCandidate(o,11.,"live");o.tick(11.01,true);
 ASSERT_TRUE(o.observe(stationaryWitness(11.,1,"live"),11.01));
 const auto* version=o.pendingVersion(11.01);ASSERT_NE(version,nullptr);
 auto invalid=proof(11.02,4,2);invalid.version=*version;invalid.transport_mode="live";
 invalid.proposal_id="stationary-reference";invalid.valid=false;
 ASSERT_TRUE(o.observe(invalid,11.02));EXPECT_EQ(o.pendingVersion(11.02),nullptr);
 EXPECT_FALSE(o.tick(11.02,true).allowed);EXPECT_EQ(o.appliedCommitSequence(),1u);
}
TEST(ExecutionHandoff, NewUnusableSDKWitnessImmediatelyWithdrawsStationaryTransaction) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
 ASSERT_TRUE(o.observe(stationaryWitness(),11.01));p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(11.01)),100,11.01));
 auto witness=stationaryWitness(11.02,2);witness.usable=false;
 EXPECT_FALSE(o.observe(witness,11.03));ASSERT_NE(o.handoff(),nullptr);EXPECT_TRUE(o.handoff()->revoked);
 EXPECT_FALSE(o.observe(stationaryWitness(),11.03)); // No older positive resurrection.
 EXPECT_FALSE(o.tick(11.03,true).allowed);
}
TEST(ExecutionHandoff, MatchingRawRollbackWithdrawsPendingAndCannotResetTheAcceptedRawFloor) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
 ASSERT_TRUE(o.observe(stationaryWitness(),11.01));p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(11.01)),100,11.01));
 const auto original=*o.handoff();auto regressed=stationaryWitness(11.02,2);
 regressed.mc_raw_stamp_ns=stationaryWitness().mc_raw_stamp_ns-1;
 EXPECT_FALSE(o.observe(regressed,11.03));ASSERT_NE(o.handoff(),nullptr);
 EXPECT_TRUE(o.handoff()->revoked);EXPECT_EQ(o.handoff()->reason,"stationary_raw_time_rollback");
 EXPECT_EQ(o.handoff()->valid_until,original.valid_until);EXPECT_FALSE(o.stopping());
 auto second=stationaryWitness(11.04,3);second.mc_raw_stamp_ns=regressed.mc_raw_stamp_ns;
 EXPECT_FALSE(o.observe(second,11.05)); // Resetting cache did not reset raw floor.
 auto held=o.tick(11.05,true);o.notePublication(held,11.05);
 EXPECT_FALSE(held.allowed);EXPECT_EQ(held.trajectory_id,1);EXPECT_EQ(o.appliedCommitSequence(),1u);
 EXPECT_TRUE(o.handoff()->revoked);EXPECT_EQ(o.pendingVersion(11.05),nullptr);
 // Only an exact confirmed negative writer outcome releases the pending slot.
 auto rejected=handoffCommit(original,11.06,2,false);rejected.permit_sequence=0;
 EXPECT_FALSE(o.observe(rejected,11.06));ASSERT_NE(o.handoff(),nullptr);
 rejected.permit_sequence=original.candidate.sequence;ASSERT_TRUE(o.observe(rejected,11.06));
 EXPECT_EQ(o.handoff(),nullptr);EXPECT_FALSE(o.observe(second,11.07));
 ASSERT_TRUE(o.observe(stationaryWitness(11.08,4),11.09));
 EXPECT_NE(o.pendingVersion(11.09),nullptr);EXPECT_FALSE(o.tick(11.09,true).allowed);
}
TEST(ExecutionHandoff, MatchingClockEpochChangeHardRevokesEvenAnUnusableWitness) {
 for(bool usable:{false,true}) {
  Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
  ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
  ASSERT_TRUE(o.observe(stationaryWitness(),11.01));p.frame_id="d1max_loc_odom";
  ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(11.01)),100,11.01));
  auto reset=stationaryWitness(11.02,2);reset.mc_clock_epoch="reset-clock";reset.usable=usable;
  EXPECT_FALSE(o.observe(reset,11.03));EXPECT_TRUE(o.stopping());
  EXPECT_EQ(o.failureReason(),"sdk_stationary_clock_epoch_changed");
  ASSERT_NE(o.handoff(),nullptr);EXPECT_TRUE(o.handoff()->revoked);
  EXPECT_FALSE(o.observe(stationaryWitness(11.04,3),11.05));
  auto held=o.tick(11.05,true);EXPECT_TRUE(held.revoked);EXPECT_FALSE(held.allowed);
 }
}
TEST(ExecutionHandoff, StationaryReentryCannotBypassHardWithdrawalOrIdentityChange) {
 for(int kind=0;kind<3;++kind) {
  Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
  ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
  auto witness=stationaryWitness();
  if(kind==0){auto state=sdk(11.01,3);state.control_owned=false;state.grant_ready=false;o.observe(state,11.01);}
  if(kind==1)witness.version.localization_epoch++;
  if(kind==2)witness.sdk_session="reconnected-sdk";
  EXPECT_FALSE(o.observe(witness,11.01));EXPECT_EQ(o.pendingVersion(11.01),nullptr);
  EXPECT_FALSE(o.tick(11.02,true).allowed);
 }
}
TEST(ExecutionHandoff, StationaryWitnessCannotRenewPinnedDeadlineOrBlockedBudget) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
 ASSERT_TRUE(o.observe(initialCommit(o,p),10.11));stationaryCandidate(o);p=o.tick(11.01,true);
 EXPECT_TRUE(o.supervise(p,11.01).empty());ASSERT_TRUE(o.observe(stationaryWitness(),11.01));p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(11.01)),100,11.01));const auto original=*o.handoff();
 ASSERT_TRUE(o.observe(stationaryWitness(11.1,2),11.11));p=o.tick(11.11,true);o.notePublication(p,11.11);
 ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(o.handoff()->valid_until,original.valid_until);
 for(int i=1;i<310;++i) {
  const double t=11.01+i*.1;auto state=sdk(t,i+3);o.observe(state,t);
  auto witness=stationaryWitness(t,i+3);o.observe(witness,t+.01);
  p=o.tick(t+.01,true);o.notePublication(p,t+.01);
  const auto failure=o.supervise(p,t+.01);
  if(t<41.)EXPECT_TRUE(failure.empty());
  if(!failure.empty())break;
 }
 EXPECT_TRUE(o.stopping());EXPECT_NE(o.failureReason().find("execution_blocked_timeout:"),std::string::npos);
}
TEST(ExecutionHandoff, ConditionalGrantKeepsIncumbentAndDoesNotRenewItsOriginalLease) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 ASSERT_TRUE(o.observe(proof(10.2,2,2),10.2));ASSERT_TRUE(o.observe(ack(10.2,2,2),10.2));
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 const auto grant=*o.handoff();EXPECT_EQ(grant.expected_commit_sequence,1u);
 EXPECT_EQ(grant.incumbent.trajectory_id,1);EXPECT_EQ(grant.candidate.trajectory_id,2);
 EXPECT_FALSE(grant.candidate.geometry_committed);EXPECT_TRUE(grant.candidate.allowed);
 EXPECT_EQ(grant.retain_incumbent_until,p.valid_until);
 ASSERT_TRUE(o.observe(proof(10.25,3),10.25));ASSERT_TRUE(o.observe(sdk(10.25,2),10.25));
 auto again=o.tick(10.3,true);EXPECT_EQ(again.trajectory_id,1);EXPECT_TRUE(again.allowed);
 EXPECT_EQ(again.valid_until,grant.retain_incumbent_until);EXPECT_EQ(o.handoff()->source_stamp,grant.source_stamp);
 EXPECT_FALSE(o.prepareHandoff(again,candidateGoal(grant.candidate.version),101,10.3));
 ASSERT_TRUE(o.observe(handoffCommit(grant,10.31),10.31));
 EXPECT_EQ(o.tick(10.32,true).trajectory_id,2);EXPECT_EQ(o.appliedCommitSequence(),2u);
 EXPECT_EQ(o.handoff(),nullptr);EXPECT_FALSE(o.observe(handoffCommit(grant,10.31),10.32));
}
TEST(ExecutionHandoff, CrossTopicPairCanPrepareBeforeNextTreeTickWithoutRenewingIncumbent) {
 for(bool admission_first:{false,true}) {
  Owner o("isolated_mock");begin(o);
  ASSERT_TRUE(o.observe(proof(10.17,2),10.17));ASSERT_TRUE(o.observe(sdk(10.17,2),10.17));
  auto original=o.tick(10.18,true);original.frame_id="d1max_loc_odom";o.notePublication(original,10.18);
  auto r=receipt();r.source_stamp=stamp(10.20);r.valid_until=stamp(10.7);
  r.version.reference_generation=2;r.version.anchor_revision=2;r.version.context_sequence=2;
  r.proposal_id="ordinary-map-correction";r.expected_version=ver();r.expected_trajectory_id=1;
  ASSERT_TRUE(o.observe(r,10.20));
  auto v=proof(10.201,3,2);v.version=r.version;v.proposal_id=r.proposal_id;v.valid_until=stamp(10.403);
  auto a=ack(10.20,3,2);a.version=r.version;a.valid_until=v.valid_until;
  if(admission_first)ASSERT_TRUE(o.observe(a,10.202));else ASSERT_TRUE(o.observe(v,10.202));
  EXPECT_EQ(o.pendingVersion(10.202),nullptr);
  if(admission_first)ASSERT_TRUE(o.observe(v,10.203));else ASSERT_TRUE(o.observe(a,10.203));
  ASSERT_NE(o.pendingVersion(10.203),nullptr);
  EXPECT_EQ(o.pendingVersion(10.203)->context_sequence,2u);
  const auto* signed_old=o.publishedHandoffIncumbent(10.203);ASSERT_NE(signed_old,nullptr);
  EXPECT_EQ(*signed_old,original);
  ASSERT_TRUE(o.prepareHandoff(*signed_old,candidateGoal(r.version),100,10.203));
  const auto fixed=*o.handoff();EXPECT_EQ(fixed.incumbent,original);
  // The conversion used to build the grant may conservatively truncate one
  // nanosecond; it must never extend the original signed incumbent deadline.
  EXPECT_LE(seconds(fixed.retain_incumbent_until),seconds(original.valid_until));
  EXPECT_GE(seconds(fixed.retain_incumbent_until),seconds(original.valid_until)-2e-9);
  EXPECT_EQ(fixed.candidate.validation_sequence,3u);EXPECT_EQ(o.appliedCommitSequence(),1u);
  // The fast path did not tick the tree, apply the candidate, or modify either
  // original native evidence timestamp. Repeated callbacks cannot create a
  // second transaction or renew this one.
  EXPECT_EQ(o.pendingVersion(10.204),nullptr);
  EXPECT_EQ(o.publishedHandoffIncumbent(10.204),nullptr);
  EXPECT_FALSE(o.prepareHandoff(original,candidateGoal(r.version),101,10.204));
  EXPECT_EQ(o.handoff()->source_stamp,fixed.source_stamp);
  EXPECT_EQ(o.handoff()->valid_until,fixed.valid_until);
 }
}
TEST(ExecutionHandoff, FastPreparationCannotInventALeaseOrBorrowExpiredOrUnsafeIncumbent) {
 for(int kind=0;kind<7;++kind) {
  Owner o("isolated_mock");begin(o);
  ASSERT_TRUE(o.observe(proof(10.32,2),10.32));ASSERT_TRUE(o.observe(sdk(10.32,2),10.32));
  auto p=o.tick(10.36,true);p.frame_id="d1max_loc_odom";
  if(kind==1)p.valid_until=stamp(10.366);
  if(kind!=0)o.notePublication(p,10.36);
  auto v=proof(10.365,3,2);v.valid_until=stamp(10.39);auto a=ack(10.365,3,2);a.valid_until=v.valid_until;
  ASSERT_TRUE(o.observe(v,10.365));ASSERT_TRUE(o.observe(a,10.365));
  if(kind==2){auto invalid=proof(10.366,4);invalid.valid=false;invalid.valid_until=stamp(0.);ASSERT_TRUE(o.observe(invalid,10.366));}
  if(kind==3){auto hold=sdk(10.366,3);hold.grant_ready=false;hold.phase="holding";hold.reason="mc_stale";ASSERT_TRUE(o.observe(hold,10.366));}
  if(kind==4){auto unknown=sdk(10.366,3);unknown.writer_commit_sequence=2;unknown.applied_trajectory_id=2;ASSERT_TRUE(o.observe(unknown,10.366));}
  if(kind==5)o.stop("cancel",10.366);
  if(kind==6)EXPECT_FALSE(o.tick(10.366,false).allowed);
  EXPECT_EQ(o.publishedHandoffIncumbent(10.367),nullptr);
  EXPECT_EQ(o.appliedCommitSequence(),1u);EXPECT_EQ(o.handoff(),nullptr);
 }
}
TEST(ExecutionHandoff, ShortBirthWindowWaitsForNewEvidenceWithoutRenewingAnyDeadline) {
 Owner o("isolated_mock");begin(o);ASSERT_TRUE(o.observe(proof(10.17,2),10.17));ASSERT_TRUE(o.observe(sdk(10.17,2),10.17));
 auto incumbent=o.tick(10.18,true);incumbent.frame_id="d1max_loc_odom";o.notePublication(incumbent,10.18);
 auto v=proof(10.201,3,2);v.valid_until=stamp(10.232);auto a=ack(10.201,3,2);a.valid_until=v.valid_until;
 ASSERT_TRUE(o.observe(v,10.203));ASSERT_TRUE(o.observe(a,10.203));ASSERT_NE(o.pendingVersion(10.203),nullptr);
 EXPECT_FALSE(o.prepareHandoff(incumbent,candidateGoal(v.version),100,10.203));EXPECT_EQ(o.handoff(),nullptr);
 EXPECT_EQ(o.handoffReason(),"waiting_candidate_usable_window");EXPECT_NEAR(o.handoffReadyWindow(),.029,1e-8);
 EXPECT_EQ(*o.publishedHandoffIncumbent(10.203),incumbent); // old signed lease stays unmodified/safe
 // A genuinely new whole proof and actual entry become a new opportunity.
 // Neither the old 31 ms proof nor the old admission is re-dated.
 auto newer=proof(10.21,4,2);newer.valid_until=stamp(10.42);auto entered=ack(10.21,4,2);entered.valid_until=newer.valid_until;
 ASSERT_TRUE(o.observe(newer,10.212));ASSERT_TRUE(o.observe(entered,10.212));
 ASSERT_TRUE(o.prepareHandoff(incumbent,candidateGoal(newer.version),101,10.212));
 EXPECT_EQ(o.handoff()->candidate.validation_sequence,4u);EXPECT_EQ(o.handoff()->incumbent,incumbent);
 EXPECT_EQ(v.valid_until,stamp(10.232));EXPECT_EQ(a.valid_until,v.valid_until);
 EXPECT_LE(seconds(o.handoff()->retain_incumbent_until),seconds(incumbent.valid_until));
}
TEST(ExecutionHandoff, EachOriginalReadyWindowCapMustFitTheSoftwareBudget) {
 for(int cap=0;cap<3;++cap){Owner o("isolated_mock");begin(o);
  ASSERT_TRUE(o.observe(proof(10.17,2),10.17));ASSERT_TRUE(o.observe(sdk(10.17,2),10.17));
  auto incumbent=o.tick(10.18,true);incumbent.frame_id="d1max_loc_odom";
  auto v=proof(10.20,3,2);auto a=ack(10.20,3,2);
  if(cap==0)v.valid_until=stamp(10.247);if(cap==1)a.valid_until=stamp(10.247);
  if(cap==2)incumbent.valid_until=stamp(10.247);
  ASSERT_TRUE(o.observe(v,10.21));ASSERT_TRUE(o.observe(a,10.21));
  EXPECT_FALSE(o.prepareHandoff(incumbent,candidateGoal(v.version),100,10.21));
  EXPECT_EQ(o.handoff(),nullptr);EXPECT_NEAR(o.handoffReadyWindow(),.037,1e-8);
 }
}
TEST(ExecutionHandoff, OriginalNanosecondCapsAreNotReencodedThroughEpochDouble) {
 builtin_interfaces::msg::Time raw,rounded,later;raw.sec=rounded.sec=later.sec=1791011008;
 raw.nanosec=626172908;rounded.nanosec=626173019;later.nanosec=876172908;
 EXPECT_EQ(earliestStamp({rounded,raw,later}),raw);
 EXPECT_EQ(earliestStamp({later,rounded,raw}),raw);
}
TEST(ExecutionHandoff, FastPreparationRequiresTheExactFreshWholeProofAdmissionPair) {
 for(int kind=0;kind<4;++kind) {
  Owner o("isolated_mock");begin(o);
  ASSERT_TRUE(o.observe(proof(10.17,2),10.17));ASSERT_TRUE(o.observe(sdk(10.17,2),10.17));
  auto p=o.tick(10.18,true);p.frame_id="d1max_loc_odom";o.notePublication(p,10.18);
  auto v=proof(10.20,3,2);v.valid_until=stamp(10.23);auto a=ack(10.20,3,2);a.valid_until=v.valid_until;
  if(kind==0)a.validation_sequence=4;
  if(kind==1){a.version.context_sequence++;a.version.anchor_revision++;}
  ASSERT_TRUE(o.observe(v,10.20));ASSERT_TRUE(o.observe(a,10.20));
  double now=10.21;
  if(kind==2)now=10.231;
  if(kind==3){auto invalid=v;invalid.sequence=4;invalid.valid=false;invalid.source_stamp=invalid.check_end=stamp(10.205);invalid.valid_until=stamp(0.);ASSERT_TRUE(o.observe(invalid,10.205));}
  EXPECT_EQ(o.pendingVersion(now),nullptr);
  EXPECT_NE(o.publishedHandoffIncumbent(now),nullptr); // Old current curve remains independently safe.
  EXPECT_EQ(o.handoff(),nullptr);
 }
}
TEST(ExecutionHandoff, ForeignOrZeroCASCannotAdvanceAnActiveWriter) {
 for(int kind=0;kind<9;++kind) {
  Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
  o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
  auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
  ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
  auto a=handoffCommit(*o.handoff(),10.22);
  if(kind==0)a.previous_commit_sequence=0;if(kind==1)a.commit_sequence=4;
  if(kind==2)a.handoff_id="foreign";if(kind==3)a.grant_sequence++;
  if(kind==4)a.incumbent_trajectory_id=99;if(kind==5)a.candidate_version.anchor_revision++;
  if(kind==6)a.permit_sequence++;if(kind==7)a.entry_admission_sequence=0;
  if(kind==8)a.sdk_arm_generation++;
  EXPECT_FALSE(o.observe(a,10.22));EXPECT_EQ(o.tick(10.23,true).trajectory_id,1);
  EXPECT_EQ(o.appliedCommitSequence(),1u);ASSERT_NE(o.handoff(),nullptr);
 }
}
TEST(ExecutionHandoff, ExpiredProofStillReportsActualAppliedIdentityButDoesNotAuthorizeMotion) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 auto a=handoffCommit(*o.handoff(),10.3);a.valid_until=stamp(10.31);
 ASSERT_TRUE(o.observe(a,10.55));EXPECT_EQ(o.appliedCommitSequence(),2u);
 o.observe(sdk(10.55,2),10.55);p=o.tick(10.55,true);
 EXPECT_EQ(p.trajectory_id,2);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.phase,"holding");
 EXPECT_FALSE(o.observe(a,10.56));
}
TEST(ExecutionHandoff, NoAckCannotAssumeFailureAndRepeatedTicksCannotRenewTransition) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 const auto grant=*o.handoff();
 for(int i=0;i<5;++i) {
  const double time=10.5+i*.1;o.observe(sdk(time,i+2),time);o.observe(proof(time,i+3),time);
  p=o.tick(time,true);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.trajectory_id,1);
  EXPECT_EQ(o.pendingVersion(time),nullptr);ASSERT_NE(o.handoff(),nullptr);
  EXPECT_EQ(o.handoff()->transition_deadline,grant.transition_deadline);
 }
 auto negative=handoffCommit(grant,11.01,2,false);negative.reason="handoff_expired";
 ASSERT_TRUE(o.observe(negative,11.01));EXPECT_EQ(o.handoff(),nullptr);EXPECT_EQ(o.appliedCommitSequence(),1u);
 EXPECT_FALSE(o.stopping());EXPECT_FALSE(o.observe(handoffCommit(grant,10.3),11.02));
}
TEST(ExecutionHandoff, LateNoWriteRejectionIsSoftwareFactNotRenewedMotionEvidence) {
 for(const double delay:{.349,.351,.6,3.}) {
  SCOPED_TRACE(delay);
  Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
  ASSERT_TRUE(o.observe(proof(10.2,2,2),10.2));ASSERT_TRUE(o.observe(ack(10.2,2,2),10.2));
  auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
  ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
  const auto grant=*o.handoff();
  auto negative=handoffCommit(grant,10.22,2,false);negative.reason="no_write_admission_rejected";
  const auto original=negative;
  ASSERT_TRUE(o.observe(negative,10.22+delay));
  EXPECT_EQ(negative,original);EXPECT_EQ(o.handoff(),nullptr);EXPECT_EQ(o.appliedCommitSequence(),1u);
  EXPECT_EQ(o.handoffReason(),"writer_handoff_rejected:no_write_admission_rejected");
  const auto held=o.tick(10.22+delay,true);
  EXPECT_EQ(held.trajectory_id,1);EXPECT_FALSE(held.allowed);EXPECT_FALSE(held.revoked);
  EXPECT_FALSE(o.stopping());EXPECT_FALSE(o.observe(negative,10.23+delay));
 }
}
TEST(ExecutionHandoff, DelayedExpiredNoWriteFactReleasesOnlyItsOldTransaction) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 ASSERT_TRUE(o.observe(proof(10.2,2,2),10.2));ASSERT_TRUE(o.observe(ack(10.2,2,2),10.2));
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 const auto old=*o.handoff();auto negative=handoffCommit(old,11.01,2,false);negative.reason="handoff_expired";
 ASSERT_GT(seconds(negative.applied_at),seconds(old.transition_deadline));
 ASSERT_TRUE(o.observe(negative,11.61));EXPECT_EQ(o.handoff(),nullptr);EXPECT_EQ(o.appliedCommitSequence(),1u);
 EXPECT_FALSE(o.tick(11.61,true).allowed); // No current SDK/curve lease was created by the fact.
 // The old ACK is only a fact. Renew all actual SDK/curve evidence and satisfy
 // the original source-evidenced recovery window after the stale-state HOLD.
 uint64_t sequence=2;
 for(const double t:{11.7,12.,12.4}) {
  ASSERT_TRUE(o.observe(sdk(t,sequence),t));
  ASSERT_TRUE(o.observe(proof(t,sequence*2-1,1),t));ASSERT_TRUE(o.observe(ack(t,sequence*2-1,1),t));
  ASSERT_TRUE(o.observe(proof(t,sequence*2,3),t));ASSERT_TRUE(o.observe(ack(t,sequence*2,3),t));
  p=o.tick(t+.01,true,static_cast<int64_t>(t*1e9),"epoch:1");++sequence;
 }
 p.frame_id="d1max_loc_odom";const auto* pending=o.pendingVersion(12.41);ASSERT_NE(pending,nullptr);
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*pending),101,12.41));
 const auto current=*o.handoff();ASSERT_NE(current.handoff_id,old.handoff_id);
 negative.sequence=1000;
 EXPECT_FALSE(o.observe(negative,12.42));
 ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(*o.handoff(),current);EXPECT_EQ(o.appliedCommitSequence(),1u);
 // A foreign old refusal cannot advance the global ACK floor and swallow
 // the current transaction's genuine lower-sequence applied result.
 ASSERT_TRUE(o.observe(handoffCommit(current,12.42,3),12.42));
 EXPECT_EQ(o.handoff(),nullptr);EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionHandoff, NoWriteFactRequiresExactPendingIdentityCASAndOriginalLegalTime) {
 for(const double delay:{0.,.6})for(int kind=0;kind<24;++kind) {
  SCOPED_TRACE(delay);
  SCOPED_TRACE(kind);
  Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
  ASSERT_TRUE(o.observe(proof(10.2,2,2),10.2));ASSERT_TRUE(o.observe(ack(10.2,2,2),10.2));
  auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
  ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
  const auto grant=*o.handoff();auto negative=handoffCommit(grant,10.22,2,false);
  if(kind==0)negative.schema_version=2;if(kind==1)negative.sequence=1;
  if(kind==2)negative.execution_id="foreign";if(kind==3)negative.control_epoch++;
  if(kind==4)negative.sdk_session="foreign";if(kind==5)negative.sdk_arm_generation++;
  if(kind==6)negative.transport_mode="live";if(kind==7)negative.candidate_version.task_id="new-task";
  if(kind==8)negative.candidate_version.route_hash=std::string(64,'f');
  if(kind==9)negative.candidate_version.reference_generation++;
  if(kind==10)negative.incumbent_trajectory_id++;if(kind==11)negative.candidate_trajectory_id++;
  if(kind==12)negative.handoff_id="foreign";if(kind==13)negative.grant_sequence++;
  if(kind==14)negative.previous_commit_sequence++;if(kind==15)negative.commit_sequence++;
  if(kind==16)negative.permit_sequence++;if(kind==17)negative.applied_at=stamp(10.209999);
  if(kind==18)negative.applied_at=stamp(10.220001+delay);
  if(kind==19)negative.applied_at.nanosec=1000000000u;if(kind==20)negative.applied_at.sec=-1;
  if(kind==21)negative.write_submitted=true;if(kind==22)negative.write_acknowledged=true;
  if(kind==23)negative.incumbent_version.anchor_revision++;
  EXPECT_FALSE(o.observe(negative,10.22+delay));
  ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(*o.handoff(),grant);EXPECT_EQ(o.appliedCommitSequence(),1u);
 }
}
TEST(ExecutionHandoff, LateNoWriteFactCannotClearCancellationOrResurrectExecution) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 ASSERT_TRUE(o.observe(proof(10.2,2,2),10.2));ASSERT_TRUE(o.observe(ack(10.2,2,2),10.2));
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 auto negative=handoffCommit(*o.handoff(),10.22,2,false);o.stop("new_target",10.23);
 const auto revoked=*o.handoff();ASSERT_TRUE(revoked.revoked);
 EXPECT_FALSE(o.observe(negative,10.82));
 ASSERT_NE(o.handoff(),nullptr);EXPECT_EQ(*o.handoff(),revoked);
 p=o.tick(10.82,true);EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);
 EXPECT_TRUE(o.stopping());EXPECT_EQ(o.appliedCommitSequence(),1u);
}
TEST(ExecutionHandoff, RevocationAndLateAckCannotResurrectAnOldExecution) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 const auto grant=*o.handoff();o.stop("cancel",10.22);
 EXPECT_TRUE(o.handoff()->revoked);EXPECT_FALSE(o.observe(handoffCommit(grant,10.23),10.23));
 p=o.tick(10.24,true);EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.trajectory_id,1);
 EXPECT_EQ(o.appliedCommitSequence(),1u);
}
TEST(ExecutionHandoff, FailedWriterSubmissionAdvancesIdentityIrreversiblyAndThenStops) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 auto a=handoffCommit(*o.handoff(),10.22);a.write_submitted=a.write_acknowledged=false;a.reason="move_error";
 ASSERT_TRUE(o.observe(a,10.22));EXPECT_EQ(o.appliedCommitSequence(),2u);
 p=o.tick(10.23,true);EXPECT_EQ(p.trajectory_id,2);EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);
 EXPECT_EQ(o.failureReason(),"writer_submission_failed:move_error");
}
TEST(ExecutionHandoff, OriginalDemandAndEntrySourceAreDistinctFromActualWriterMeasurement) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);auto first=initialCommit(o,p);
 first.body_source_stamp=stamp(10.11);first.measured_pose.header.stamp=first.body_source_stamp;
 ASSERT_TRUE(o.observe(first,10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 auto actual=handoffCommit(*o.handoff(),10.24);
 actual.entry_source_stamp=actual.demand_body_source_stamp=stamp(10.21);
 actual.demand_source_stamp=stamp(10.22);actual.body_source_stamp=stamp(10.24);
 actual.measured_pose.header.stamp=actual.body_source_stamp;
 auto restamped=actual;restamped.entry_source_stamp=restamped.body_source_stamp;
 EXPECT_FALSE(o.observe(restamped,10.24));
 ASSERT_TRUE(o.observe(actual,10.24));EXPECT_EQ(o.appliedCommitSequence(),2u);
}
TEST(ExecutionHandoff, LateAppliedFactIsRecordedWithoutRestampingExpiredEvidence) {
 for(const double delay:{.349,.351}) {
  Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
  o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
  auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
  ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
  auto actual=handoffCommit(*o.handoff(),10.22);
  EXPECT_TRUE(o.observe(actual,10.22+delay));
  EXPECT_EQ(o.appliedCommitSequence(),2u);
  const auto held=o.tick(10.22+delay,true);
  EXPECT_EQ(held.trajectory_id,2);EXPECT_FALSE(held.allowed);
  EXPECT_EQ(o.handoff(),nullptr); // Recorded fact, never a refreshed motion lease.
 }
}
TEST(ExecutionHandoff, BeforeRevocationAppliedFactCanArriveWhileStoppingWithoutResuming) {
 Owner o("isolated_mock");beginUnapplied(o);ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 o.observe(proof(10.2,2,2),10.2);o.observe(ack(10.2,2,2),10.2);
 auto p=o.tick(10.21,true);p.frame_id="d1max_loc_odom";
 ASSERT_TRUE(o.prepareHandoff(p,candidateGoal(*o.pendingVersion(10.21)),100,10.21));
 const auto actual=handoffCommit(*o.handoff(),10.22);
 o.stop("cancel",10.23);
 ASSERT_TRUE(o.observe(actual,11.));EXPECT_EQ(o.appliedCommitSequence(),2u);
 p=o.tick(11.01,true);EXPECT_TRUE(p.revoked);EXPECT_FALSE(p.allowed);EXPECT_EQ(p.trajectory_id,2);
 EXPECT_EQ(o.failureReason(),"cancel");
}
TEST(ExecutionInitialGeometry, ActualZeroOutputPinsInitialFactBeyondPreparationHeartbeats) {
 Owner o("isolated_mock");beginUnapplied(o);const auto first=o.tick(10.1,true);
 auto actual=initialCommit(o,first,10.11);
 actual.demand_source_stamp=actual.demand_body_source_stamp=actual.body_source_stamp=first.source_stamp;
 actual.measured_pose.header.stamp=actual.body_source_stamp;
 auto demand=movingDemand(10.1,1);demand.permit_sequence=first.sequence;
 demand.validation_sequence=first.validation_sequence;demand.velocity=actual.applied_velocity;
 demand.safety_checked=false; // Zero is a write fact, not a free-space assertion.
 ASSERT_TRUE(o.observe(demand,10.1));
 ++demand.sequence;++demand.motion_validation_sequence;
 ASSERT_TRUE(o.observe(demand,10.1));
 actual.demand_sequence=demand.sequence;actual.motion_validation_sequence=demand.motion_validation_sequence;
 for(int i=0;i<12;++i) {
  const double time=10.2+.1*i;
  ASSERT_TRUE(o.observe(sdk(time,i+2),time));
  ASSERT_TRUE(o.observe(proof(time,i+2),time));ASSERT_TRUE(o.observe(ack(time,i+2),time));
  auto renewal=o.tick(time,true);renewal.frame_id="d1max_loc_odom";o.notePublication(renewal,time);
 }
 o.stop("cancel",11.4);
 ASSERT_TRUE(o.observe(actual,11.5));EXPECT_EQ(o.appliedCommitSequence(),1u);
 const auto held=o.tick(11.51,true);EXPECT_TRUE(held.revoked);EXPECT_FALSE(held.allowed);
 EXPECT_LT(seconds(actual.valid_until),11.5);
 auto malformed=actual;++malformed.sequence;malformed.demand_sequence+=100;
 EXPECT_FALSE(o.observe(malformed,11.52));
}
TEST(ExecutionHandoff, VendorAcknowledgementFactCannotRegressOrRenewItsOriginalLease) {
 Owner o("isolated_mock");beginUnapplied(o);auto actual=initialCommit(o,o.tick(10.1,true),10.11);
 actual.write_acknowledged=false;ASSERT_TRUE(o.observe(actual,10.11));
 auto acknowledged=actual;++acknowledged.sequence;acknowledged.write_acknowledged=true;
 ASSERT_TRUE(o.observe(acknowledged,10.8));EXPECT_EQ(o.appliedCommitSequence(),1u);
 auto rollback=acknowledged;++rollback.sequence;rollback.write_acknowledged=false;
 EXPECT_FALSE(o.observe(rollback,10.81));
 auto redated=acknowledged;redated.sequence+=2;redated.valid_until=stamp(11.);
 EXPECT_FALSE(o.observe(redated,10.82));
 const auto held=o.tick(10.83,true);EXPECT_FALSE(held.allowed);
}
TEST(ExecutionOwner, RemainingProofOnlyForAlreadyCommittedCurve){Owner o("isolated_mock");prepare(o);
 auto p=proof(10.1,2);p.whole_curve=false;p.remaining_curve=true;p.checked_from_time=.1;p.valid_start_time=.2;
 p.checked_to_time=p.curve_duration=1.;p.reverse_margin_m=.15;o.observe(p,10.1);o.observe(ack(10.1,2),10.1);
 EXPECT_FALSE(o.canConfirm(10.1)); // latest proof cannot certify initial commit
 Owner running("isolated_mock");begin(running);running.tick(10.1,true);running.observe(p,10.1);
 EXPECT_TRUE(running.tick(10.15,true).allowed);p.sequence=3;p.trajectory_id=2;p.source_stamp=p.check_end=stamp(10.2);
 p.valid_until=stamp(10.4);running.observe(p,10.2);running.observe(ack(10.2,3,2),10.2);
 EXPECT_EQ(running.tick(10.2,true).trajectory_id,1);}
TEST(ExecutionOwner, RemainingProofCannotOmitTailOrReverseMargin){
 for(int mode=0;mode<3;++mode){Owner o("isolated_mock");begin(o);o.tick(10.1,true);auto p=proof(10.2,2);
  p.whole_curve=false;p.remaining_curve=true;p.checked_from_time=.1;p.valid_start_time=.2;
  p.checked_to_time=p.curve_duration=1.;p.reverse_margin_m=.15;
  if(mode==0)p.checked_to_time=.9;if(mode==1)p.reverse_margin_m=.1;if(mode==2)p.checked_from_time=.3;
  o.observe(p,10.2);EXPECT_FALSE(o.tick(10.2,true).allowed);}
}
TEST(ExecutionOwner, ArmingAndHoldPreserveCommittedGeometryWithoutPermission){
 Owner o("isolated_mock");prepare(o);auto preview=o.tick(10.01,true);ASSERT_TRUE(preview.geometry_committed);
 ASSERT_TRUE(o.begin("execution","confirm",1,10.02));auto arming=o.tick(10.03,true);
 EXPECT_TRUE(arming.geometry_committed);EXPECT_FALSE(arming.allowed);EXPECT_EQ(arming.phase,"arming");
 EXPECT_EQ(arming.version,preview.version);EXPECT_EQ(arming.trajectory_id,preview.trajectory_id);
 o.observe(sdk(),10.1);ASSERT_TRUE(o.tick(10.1,true).allowed);
 ASSERT_TRUE(o.observe(initialCommit(o,o.tick(10.1,true)),10.11));
 auto holding=o.tick(10.36,true);EXPECT_TRUE(holding.geometry_committed);EXPECT_FALSE(holding.allowed);
 EXPECT_EQ(holding.trajectory_id,preview.trajectory_id);
 auto newer=proof(10.37,2);ASSERT_TRUE(o.observe(newer,10.37));
 auto restored=o.tick(10.38,true);EXPECT_TRUE(restored.allowed);EXPECT_EQ(restored.trajectory_id,preview.trajectory_id);
 EXPECT_EQ(restored.validation_sequence,2u); // no new preparation for unchanged committed curve
}
TEST(ExecutionPublication, SequenceDoesNotResetAtSecondTask){
 PermitPublication publisher;Permit first;first.version=ver();first.sequence=15;
 ASSERT_TRUE(publisher.prepare(first,"task",true,false));EXPECT_EQ(first.sequence,1u);
 first.sequence=16;ASSERT_TRUE(publisher.prepare(first,"task",true,false));EXPECT_EQ(first.sequence,2u);
 Permit second;second.version=ver();second.version.task_id="task2";second.sequence=1;
 ASSERT_TRUE(publisher.prepare(second,"task2",false,false));EXPECT_EQ(second.sequence,3u);
 first.revoked=true;EXPECT_FALSE(publisher.prepare(first,"task2",true,true));
 EXPECT_FALSE(publisher.prepare(first,"task2",false,false));
 second.sequence=2;ASSERT_TRUE(publisher.prepare(second,"task2",false,false));EXPECT_EQ(second.sequence,4u);
}
TEST(ExecutionPublication, PriorTaskCanOnlyPublishStopWhileStillDraining){
 PermitPublication publisher;Permit old;old.version=ver();old.allowed=true;
 EXPECT_FALSE(publisher.prepare(old,"newtask",true,false));
 old.allowed=false;old.revoked=true;EXPECT_TRUE(publisher.prepare(old,"newtask",true,false));
 EXPECT_FALSE(publisher.prepare(old,"newtask",true,true));
}
TEST(ExecutionPublication, RetiredPreviewNotifiesExactOldGeometryOnceWithoutCompetingWithNewTask) {
 PermitPublication publisher;Owner old("isolated_mock");prepare(old);
 auto preview=old.tick(10.1,true);ASSERT_TRUE(preview.geometry_committed);
 ASSERT_TRUE(publisher.prepare(preview,"task",false,false));
 old.stop("superseded_by_new_task",10.11);
 auto revoke=old.tick(10.12,false);
 EXPECT_TRUE(revoke.revoked);EXPECT_FALSE(revoke.allowed);EXPECT_EQ(revoke.phase,"retired_geometry");
 EXPECT_EQ(revoke.version,preview.version);EXPECT_EQ(revoke.trajectory_id,preview.trajectory_id);
 EXPECT_TRUE(publisher.prepare(revoke,"newtask",false,false));
 const auto sequence=revoke.sequence;
 auto new_task=preview;new_task.version.task_id="newtask";
 ASSERT_TRUE(publisher.prepare(new_task,"newtask",false,false));EXPECT_GT(new_task.sequence,sequence);
 EXPECT_FALSE(publisher.prepare(revoke,"newtask",false,false));
 ASSERT_TRUE(publisher.prepare(new_task,"newtask",false,false));EXPECT_EQ(new_task.sequence,4u);
 EXPECT_FALSE(old.canConfirm(10.12));
 EXPECT_FALSE(old.begin("late_execution","late_confirm",2,10.12));
 EXPECT_FALSE(old.confirmed());EXPECT_FALSE(old.requiresPhysicalStop());EXPECT_FALSE(old.stopped());
 // A fresh late result is geometry evidence, never resurrection of the task.
 auto late=proof(10.13,2,2);ASSERT_TRUE(old.observe(late,10.13));
 auto after=old.tick(10.14,true);
 EXPECT_TRUE(after.revoked);EXPECT_FALSE(after.allowed);EXPECT_EQ(after.version,preview.version);
 EXPECT_EQ(after.trajectory_id,preview.trajectory_id);EXPECT_EQ(old.preparationEvidence(10.14),0);
}
TEST(ExecutionPublication, ForeignPreviewHeartbeatCannotPretendToBeGeometryRetirement) {
 PermitPublication publisher;auto stale=Permit{};stale.version=ver();stale.phase="preview";
 EXPECT_FALSE(publisher.prepare(stale,"newtask",false,false));
 stale.revoked=true;EXPECT_FALSE(publisher.prepare(stale,"newtask",false,false));
 stale.phase="retired_geometry";stale.allowed=true;
 EXPECT_FALSE(publisher.prepare(stale,"newtask",false,false));
}
TEST(ExecutionOwner, HumanMayConfirmActuallyInstalledPreviewAfterInitialAdmissionExpires){
 Owner o("isolated_mock");prepare(o);auto first=o.tick(10.01,true);
 first.frame_id="d1max_loc_odom";o.notePublication(first,10.01);
 auto installed=geometryReceipt(first,10.02,1,1,10.02);ASSERT_TRUE(o.observe(installed,10.02));
 auto newer=proof(12.,20);ASSERT_TRUE(o.observe(newer,12.));
 installed.sequence=2;installed.body_source_stamp=stamp(12.);ASSERT_TRUE(o.observe(installed,12.));
 // Only the actual typed software-installation fact permits renewal without
 // re-preparing. Its original receipt/proof times are not refreshed.
 auto preview=o.tick(12.01,true);EXPECT_FALSE(preview.allowed);EXPECT_EQ(preview.validation_sequence,20u);
 EXPECT_TRUE(o.canConfirm(12.02));EXPECT_TRUE(complete(o.grantVersion(12.02)));
 EXPECT_EQ(o.grantVersion(12.02),ver());EXPECT_TRUE(o.begin("execution","late-human",1,12.02));
}
TEST(ExecutionOwner, CommittedPreviewDoesNotAuthorizeWithoutCurrentCollisionEvidence){
 Owner o("isolated_mock");prepare(o);o.tick(10.01,true);EXPECT_FALSE(o.canConfirm(12.));
 auto invalid=proof(12.,20);invalid.valid=false;ASSERT_TRUE(o.observe(invalid,12.));
 EXPECT_FALSE(o.tick(12.01,true).geometry_committed);EXPECT_FALSE(o.canConfirm(12.01));
 EXPECT_FALSE(o.begin("execution","invalid-human",1,12.01));
 auto foreign=proof(12.02,21);foreign.version.anchor_revision=2;
 ASSERT_TRUE(o.observe(foreign,12.02));EXPECT_FALSE(o.canConfirm(12.02));
}
TEST(PreparationBudget, EveryWaitHasItsOwnBoundAndFailureLatches){
 const PreparationStage stages[]{PreparationStage::Pose,PreparationStage::Map,
   PreparationStage::Reference,PreparationStage::Trajectory};
 const double bounds[]{30.,15.,5.,20.};
 const char* reasons[]{"follow_pose_timeout","follow_map_timeout","follow_reference_timeout","follow_trajectory_timeout"};
 for(unsigned i=0;i<4;++i){PreparationBudget budget;
  EXPECT_TRUE(budget.observe(stages[i],100.).empty());
  EXPECT_TRUE(budget.observe(stages[i],100.+bounds[i]-.01).empty());
  EXPECT_EQ(budget.observe(stages[i],100.+bounds[i]),reasons[i]);
  EXPECT_EQ(budget.observe(PreparationStage::Ready,200.,200000000000LL),reasons[i]);
 }
}
TEST(PreparationBudget, FeedbackAndAlternatingWaitPhasesCannotRenewBudget){
 PreparationBudget budget;
 // A 250 ms map/reference status flicker is not two independent attempts.
 for(int i=0;i<39;++i)EXPECT_TRUE(budget.observe(i%2?PreparationStage::Map:PreparationStage::Reference,i*.25).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Map,9.75),"follow_reference_timeout");
}
TEST(PreparationBudget, MixedSubthresholdWaitsStillHaveATotalEpisodeBound){
 PreparationBudget budget;
 EXPECT_TRUE(budget.observe(PreparationStage::Pose,0.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Map,10.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Trajectory,20.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Reference,29.).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Ready,30.,30000000000LL),"follow_preparation_episode_timeout");
}
TEST(PreparationBudget, RepeatedSourceEvidenceIsNotRecovery){
 PreparationBudget budget;budget.observe(PreparationStage::Map,0.);
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,1.,123456789).empty());
 for(int i=2;i<30;++i)EXPECT_TRUE(budget.observe(PreparationStage::Ready,i,123456789).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Ready,30.,123456789),"follow_preparation_episode_timeout");
}
TEST(PreparationBudget, ThreeDistinctSourceSamplesAndStableReadinessResetEpisode){
 PreparationBudget budget;budget.observe(PreparationStage::Map,0.);
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,14.,14000000000LL).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,14.3,14300000000LL).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,14.7,14700000000LL).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Map,100.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Map,114.9).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Map,115.),"follow_map_timeout");
}
TEST(PreparationBudget, MissingSourceProofCannotMarkReady){
 PreparationBudget budget;EXPECT_TRUE(budget.observe(PreparationStage::Ready,0.).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Ready,20.),"follow_trajectory_timeout");
}
TEST(PreparationBudget, ComputeHasItsOwnBoundButCannotResetAPriorWait){
 PreparationBudget budget;budget.observe(PreparationStage::Map,0.);
 EXPECT_TRUE(budget.observe(PreparationStage::Dormant,10.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Map,70.).empty());
 EXPECT_EQ(budget.observe(PreparationStage::Map,75.),"follow_map_timeout");
}
TEST(PreparationBudget, HealthyPreviewDoesNotTimeOutWaitingForHumanConfirmation){
 PreparationBudget budget;
 for(int i=0;i<1000;++i)EXPECT_TRUE(budget.observe(PreparationStage::Ready,i+1.,(i+1)*1000000000LL).empty());
}
TEST(PreparationBudget, RecoveryAtExpiredDeadlineCannotResurrectTask){
 PreparationBudget budget;budget.observe(PreparationStage::Trajectory,0.);
 budget.observe(PreparationStage::Ready,19.);
 budget.observe(PreparationStage::Ready,19.2,19200000000LL);
 budget.observe(PreparationStage::Ready,19.4,19400000000LL);
 EXPECT_EQ(budget.observe(PreparationStage::Ready,30.,30000000000LL),"follow_preparation_episode_timeout");
}
TEST(PreparationBudget, InvalidConfigurationAndBackwardTimeFailClosed){
 PreparationLimits limits;
 for(const double invalid:{0.,-1.,121.,std::numeric_limits<double>::quiet_NaN()}){
  limits.waits[0]=invalid;EXPECT_THROW(PreparationBudget{limits},std::invalid_argument);
 }
 limits=PreparationLimits{};limits.episode=0.;EXPECT_THROW(PreparationBudget{limits},std::invalid_argument);
 limits=PreparationLimits{};limits.stable=0.;EXPECT_THROW(PreparationBudget{limits},std::invalid_argument);
 PreparationBudget budget;budget.observe(PreparationStage::Map,10.);
 EXPECT_EQ(budget.observe(PreparationStage::Map,9.),"preparation_clock_invalid");
}
TEST(PreparationBudget, ProductionFeedbackMapsToCorrectBudgetNotGenericReference){
 EXPECT_EQ(preparationStage(false,false,"following",123),PreparationStage::Pose);
 EXPECT_EQ(preparationStage(true,false,"waiting_reference",0),PreparationStage::Dormant);
 EXPECT_EQ(preparationStage(true,true,"waiting_local_map",0),PreparationStage::Map);
 EXPECT_EQ(preparationStage(true,true,"waiting_reference",0),PreparationStage::Reference);
 EXPECT_EQ(preparationStage(true,true,"recovering_local_trajectory",0),PreparationStage::Trajectory);
 EXPECT_EQ(preparationStage(true,true,"following",0),PreparationStage::Trajectory);
 EXPECT_EQ(preparationStage(true,true,"following",123),PreparationStage::Ready);
}
TEST(PreparationBudget, PersistentLocalStagesSurviveSixtySecondsAndAcceptLateMeasuredRecovery){
 const PreparationStage stages[]{PreparationStage::Pose,PreparationStage::Map,
   PreparationStage::Reference,PreparationStage::Trajectory};
 for(const auto stage:stages){
  SCOPED_TRACE(static_cast<unsigned>(stage));
  PreparationBudget budget(PreparationLimits{},true);
  EXPECT_TRUE(budget.observe(stage,100.).empty());
  EXPECT_TRUE(budget.observe(stage,130.).empty());
  EXPECT_TRUE(budget.observe(stage,161.).empty());
  // The caller keeps its same Follow task; this budget creates no route,
  // SDK authority or substitute measurement while the local stage is waiting.
  EXPECT_TRUE(budget.observe(PreparationStage::Ready,162.,162000000000LL).empty());
  EXPECT_TRUE(budget.observe(PreparationStage::Ready,162.3,162300000000LL).empty());
  EXPECT_TRUE(budget.observe(PreparationStage::Ready,162.61,162610000000LL).empty());
  EXPECT_TRUE(budget.observe(stage,200.).empty());
  EXPECT_TRUE(budget.observe(stage,261.).empty());
 }
}
TEST(PreparationBudget, PersistentMixedWaitsAndMissingBodyProofDoNotLatchEpisodeFailure){
 PreparationBudget budget(PreparationLimits{},true);
 const PreparationStage stages[]{PreparationStage::Pose,PreparationStage::Map,
   PreparationStage::Reference,PreparationStage::Trajectory};
 for(unsigned i=0;i<=8;++i)
  EXPECT_TRUE(budget.observe(stages[i%4],100.+i*10.).empty());
 // Ready without a source remains a trajectory wait, not fabricated readiness.
 EXPECT_EQ(preparationStage(true,true,"following",0),PreparationStage::Trajectory);
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,190.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,251.).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,252.,252000000000LL).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,252.3,252300000000LL).empty());
 EXPECT_TRUE(budget.observe(PreparationStage::Ready,252.61,252610000000LL).empty());
}
TEST(PreparationBudget, PersistentPolicyStillLatchesNonFiniteAndBackwardClockFailures){
 const double invalid[]{99.,std::numeric_limits<double>::quiet_NaN(),
   std::numeric_limits<double>::infinity()};
 for(const double now:invalid){
  PreparationBudget budget(PreparationLimits{},true);
  ASSERT_TRUE(budget.observe(PreparationStage::Map,100.).empty());
  EXPECT_EQ(budget.observe(PreparationStage::Ready,now,101000000000LL),"preparation_clock_invalid");
  EXPECT_EQ(budget.observe(PreparationStage::Ready,200.,200000000000LL),"preparation_clock_invalid");
 }
}
TEST(PreparationBudget, PersistentPolicyCannotAdmitMalformedLimits){
 for(const double invalid:{0.,-1.,121.,std::numeric_limits<double>::quiet_NaN(),
   std::numeric_limits<double>::infinity()}){
  auto limits=PreparationLimits{};limits.waits[0]=invalid;
  EXPECT_THROW((PreparationBudget{limits,true}),std::invalid_argument);
 }
 auto limits=PreparationLimits{};limits.episode=0.;
 EXPECT_THROW((PreparationBudget{limits,true}),std::invalid_argument);
 limits=PreparationLimits{};limits.stable=0.;
 EXPECT_THROW((PreparationBudget{limits,true}),std::invalid_argument);
 limits=PreparationLimits{};limits.episode=1.;limits.stable=1.;
 EXPECT_THROW((PreparationBudget{limits,true}),std::invalid_argument);
}
TEST(ExecutionOwner, PreparationEvidenceRequiresCommittedAndFreshActualSource){
 Owner o("isolated_mock");prepare(o);EXPECT_EQ(o.preparationEvidence(10.01),0);
 o.tick(10.01,true);EXPECT_EQ(o.preparationEvidence(10.01),10000000000LL);
 EXPECT_EQ(o.preparationEvidence(11.),0); // receipt/feedback does not renew proof.
 auto p=proof(11.,2);p.body_source_stamp=stamp(10.9);ASSERT_TRUE(o.observe(p,11.));
 EXPECT_EQ(o.preparationEvidence(11.),0); // Whole collision proof alone is not a prepared entry.
 auto prepared=ack(11.,2);prepared.body_source_stamp=p.body_source_stamp;ASSERT_TRUE(o.observe(prepared,11.));
 EXPECT_EQ(o.preparationEvidence(11.),10900000000LL); // not check_end == 11.
 p=proof(11.1,3);p.valid=false;ASSERT_TRUE(o.observe(p,11.1));EXPECT_EQ(o.preparationEvidence(11.1),0);
}
TEST(ExecutionOwner, PermissionAndZeroMotionProofNeverCountAsProgress){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<=601;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);
  d.velocity.linear.x=0.;d.hold=true;auto m=motionProof(d);
  ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(m,t));auto b=body(t);
  const auto reason=o.supervise(p,t,&b);
  EXPECT_TRUE(reason.empty());EXPECT_EQ(o.progressReason(),"waiting_admitted_nonzero_command");
  EXPECT_EQ(o.blockedAge(t),0.);
 }
 EXPECT_FALSE(o.stopping());EXPECT_TRUE(o.confirmed());EXPECT_FALSE(p.revoked);
 EXPECT_EQ(p.version.route_hash,ver().route_hash); // pure zero is neither progress nor task failure.
}
TEST(ExecutionBootstrap, InstalledCurveWithoutAdmittedCommandRetainsConfirmedFixedRoute) {
 // Real f2a452... evidence: tracker installation=1, writer commit=0,
 // Safety accepted=0; the old supervisor killed this bootstrap at 30 s.
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 o.notePublication(p,10.1);auto installed=geometryReceipt(p);
 ASSERT_TRUE(o.observe(installed,10.1));
 for(int i=0;i<=601;++i) {
  const double t=10.2+i*.1;ASSERT_TRUE(o.observe(sdk(t,i+2),t));
  ASSERT_TRUE(o.observe(proof(t,i+2),t));
  installed.sequence=i+2;installed.body_source_stamp=stamp(t);ASSERT_TRUE(o.observe(installed,t));
  p=o.tick(t,true);p.frame_id="d1max_loc_odom";const auto b=body(t);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());EXPECT_FALSE(o.stopping());
  EXPECT_EQ(o.blockedAge(t),0.);EXPECT_EQ(o.progressReason(),"waiting_admitted_nonzero_command");
  EXPECT_EQ(p.version.task_id,ver().task_id);EXPECT_EQ(p.version.route_id,ver().route_id);
  EXPECT_EQ(p.version.route_hash,ver().route_hash);EXPECT_EQ(o.appliedCommitSequence(),0u);
 }
 EXPECT_TRUE(o.confirmed());EXPECT_TRUE(o.failureReason().empty());
}
TEST(ExecutionBootstrap, MissingWorkerEvidenceWaitsButDoesNotManufacturePermission) {
 for(const auto* reason:{"missing_fresh_raw_ray_support_or_body_evidence","source_expired_during_check",
     "curve_check_budget_exhausted","candidate:join_source_expired"}) {
  Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);
  for(int i=0;i<=301;++i) {
   const double t=10.2+i*.1;ASSERT_TRUE(o.observe(sdk(t,i+2),t));
   auto unavailable=proof(t,i+2);unavailable.valid=false;unavailable.valid_until=stamp(0.);unavailable.reason=reason;
   ASSERT_TRUE(o.observe(unavailable,t));p=o.tick(t,true);p.frame_id="d1max_loc_odom";auto b=body(t);
   EXPECT_FALSE(p.allowed);EXPECT_TRUE(o.supervise(p,t,&b).empty());
   EXPECT_FALSE(p.revoked);EXPECT_FALSE(o.stopping());EXPECT_EQ(o.blockedAge(t),0.);
  }
  EXPECT_TRUE(o.confirmed());EXPECT_EQ(p.version.route_hash,ver().route_hash);
 }
}
TEST(ExecutionBootstrap, CurrentNegativeEnvironmentEvidenceIsStillBoundedAndUnknownIsNotOccupied) {
 for(const auto* reason:{"occupied","unknown_or_unobserved","measured_connection_support_invalid"}) {
  Owner o("isolated_mock");beginUnapplied(o);o.tick(10.1,true);Permit p;std::string failure;
  for(int i=0;i<=301;++i) {
   const double t=10.2+i*.1;ASSERT_TRUE(o.observe(sdk(t,i+2),t));
   auto rejected=proof(t,i+2);rejected.valid=false;rejected.reason=reason;rejected.valid_until=stamp(0.);
   ASSERT_TRUE(o.observe(rejected,t));p=o.tick(t,true);p.frame_id="d1max_loc_odom";auto b=body(t);
   failure=o.supervise(p,t,&b);EXPECT_FALSE(p.allowed);
   if(!failure.empty())break;
  }
  EXPECT_EQ(failure,"execution_blocked_timeout:waiting_initial_tracker_admission");
  EXPECT_TRUE(o.stopping());EXPECT_TRUE(p.revoked);EXPECT_FALSE(o.stopped());
  EXPECT_EQ(p.version.route_hash,ver().route_hash);
 }
}
TEST(ExecutionBootstrap, UnrelatedCandidateNegativeCannotBecomeCurrentObstruction) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<=301;++i) {
  const double t=10.2+i*.1;ASSERT_TRUE(o.observe(sdk(t,i+2),t));
  auto rejected=proof(t,i+2,2);rejected.valid=false;rejected.reason="occupied";rejected.valid_until=stamp(0.);
  ASSERT_TRUE(o.observe(rejected,t));auto b=body(t);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());EXPECT_EQ(o.blockedAge(t),0.);
 }
 EXPECT_FALSE(o.stopping());EXPECT_EQ(p.trajectory_id,1);
}
TEST(ExecutionBootstrap, ExplicitCancellationAndSDKWithdrawalNeverRecoverOnLateGoodEvidence) {
 for(int kind=0;kind<2;++kind) {
  Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
  auto b=body(10.2);ASSERT_TRUE(o.supervise(p,10.2,&b).empty());
  if(kind==0)o.stop("explicit_cancel",10.3);
  else {auto lost=sdk(10.3,2);lost.control_owned=false;lost.reason="control_lost";ASSERT_TRUE(o.observe(lost,10.3));}
  for(int i=0;i<10;++i) {
   const double t=10.4+i*.1;o.observe(sdk(t,i+3),t);o.observe(proof(t,i+3),t);
   p=o.tick(t,true,104+i,"same-local-epoch");EXPECT_FALSE(p.allowed);EXPECT_TRUE(p.revoked);
   b=body(t);EXPECT_TRUE(o.supervise(p,t,&b).empty());EXPECT_TRUE(o.stopping());
  }
  EXPECT_FALSE(o.begin("another_execution","confirm",2,12.));
 }
}
TEST(ExecutionBootstrap, ExactNonzeroWriterSubmissionStartsProgressBudgetEvenIfFeedbackIsLost) {
 Owner o("isolated_mock");beginUnapplied(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 auto actual=initialCommit(o,p,10.1);actual.applied_velocity.linear.x=.2;
 ASSERT_TRUE(o.observe(actual,10.1));std::string failure;
 for(int i=0;i<=301;++i) {
  const double t=10.2+i*.1;auto b=body(t);failure=o.supervise(p,t,&b);
  if(!failure.empty())break;
 }
 EXPECT_EQ(failure,"execution_blocked_timeout:waiting_admitted_nonzero_command");
 EXPECT_TRUE(o.stopping());EXPECT_TRUE(p.revoked);EXPECT_FALSE(o.stopped());
}
TEST(ExecutionOwner, NonzeroAdmittedButBodyStationaryStillTimesOut){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<=301;++i){const double t=10.2+i*.1;feedback(o,t,i+1);auto b=body(t);
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_EQ(reason,"execution_blocked_timeout:waiting_measured_motion_progress");break;}
 }
}
TEST(ExecutionOwner, DeadlineWinsOverLastStableRecoveryCallback) {
 for(bool before_deadline:{false,true}) {
  Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
  feedback(o,10.2,1);auto b=body(10.2);ASSERT_TRUE(o.supervise(p,10.2,&b).empty());
  for(int i=1;i<=280;++i) {
   const double t=10.2+i*.1;feedback(o,t,i+1);b=body(t);ASSERT_TRUE(o.supervise(p,t,&b).empty());
  }
  feedback(o,39.1,282,.04);b=body(39.1,.04);ASSERT_TRUE(o.supervise(p,39.1,&b).empty());
  feedback(o,39.6,283,.08);b=body(39.6,.08);ASSERT_TRUE(o.supervise(p,39.6,&b).empty());
  const double final=before_deadline?40.199:40.2;
  feedback(o,final,284,.12);b=body(final,.12);const auto failure=o.supervise(p,final,&b);
  if(before_deadline) {EXPECT_TRUE(failure.empty());EXPECT_FALSE(o.stopping());EXPECT_EQ(o.blockedAge(final),0.);}
  else {EXPECT_EQ(failure,"execution_blocked_timeout:waiting_measured_motion_progress");EXPECT_TRUE(o.stopping());EXPECT_TRUE(p.revoked);}
 }
}
TEST(ExecutionOwner, ActualMotionInvalidIsDiagnosticAndCannotBeErasedByZeroProof){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 auto d=movingDemand(10.2,1);auto m=motionProof(d);m.valid=false;m.reason="motion_sweep_occupied";m.valid_until=stamp(0.);
 ASSERT_TRUE(o.observe(m,10.2));auto b=body(10.2);EXPECT_TRUE(o.supervise(p,10.2,&b).empty());
 for(int i=1;i<=301;++i){const double t=10.2+i*.1;d=movingDemand(t,i+1);d.velocity.linear.x=0.;d.hold=true;
  ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(motionProof(d),t));b=body(t);
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_EQ(reason,"execution_blocked_timeout:actual_command_blocked:motion_sweep_occupied");break;}
 }
}
TEST(ExecutionOwner, ShortPhysicalMovementCannotRenewBlockedEpisode){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 feedback(o,10.2,1);
 auto b=body(10.2);o.supervise(p,10.2,&b);
 for(int i=1;i<=301;++i){const double t=10.2+i*.1;b=body(t,i>10?.04:0.);
  if(i==10||i==11)feedback(o,t,i+1);
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_FALSE(reason.empty());break;}
 }
}
TEST(ExecutionOwner, OnePositionStepWithContinuousCommandsIsNotStableRecovery){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 feedback(o,10.2,1);
 auto b=body(10.2);o.supervise(p,10.2,&b);
 for(int i=1;i<=301;++i){const double t=10.2+i*.1;feedback(o,t,i+1);b=body(t,i>=10?.04:0.);
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_FALSE(reason.empty());break;}
 }
}
TEST(ExecutionOwner, BodySourceReplayCannotManufactureProgress){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 feedback(o,10.2,1);
 auto b=body(10.2);o.supervise(p,10.2,&b);
 for(int i=1;i<=301;++i){const double t=10.2+i*.1;feedback(o,t,i+1);b.local_odometry.pose.pose.position.x=i*.1;
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_FALSE(reason.empty());break;}
 }
}
TEST(ExecutionOwner, MatchingAdmissionAndContinuousMeasuredProgressSustainTask){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<1000;++i){const double t=10.2+i*.1;feedback(o,t,i+1,i*.02);auto b=body(t,i*.02);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());}
 EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, RealLateralTravelCannotRenewFixedRouteBlockedBudget){
 // blocked_safe_distance moved ~0.4 m in Y on an X-only global route. The old
 // hypot progress rule reset 30 s even though no task arc was gained.
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 std::string failure;
 for(int i=0;i<=301;++i){const double t=10.2+i*.1;const double y=.004*i;
  auto d=movingDemand(t,i+1);ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(motionProof(d),t));
  ASSERT_TRUE(o.observe(routeProgress(t,i+1,0.,y),t));auto b=body(t);b.local_odometry.pose.pose.position.y=y;
  failure=o.supervise(p,t,&b);if(!failure.empty())break;
 }
 EXPECT_EQ(failure,"execution_blocked_timeout:waiting_measured_motion_progress");EXPECT_TRUE(p.revoked);
}
TEST(RouteProgressSupervisor, GenuineArcCanIncreaseWhileEuclideanGoalDistanceIncreases){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot({{0.,0.,0.},{0.,1.,0.},{1.,1.,0.},{1.,0.,0.}}),ver(),"isolated_mock"));
 int advances=0;
 for(int i=0;i<=20;++i){const double y=i*.04;auto p=routeProgress(10.+i*.1,i+1,0.,y);
  p.measured_arc_m=p.confirmed_arc_m=y;p.cross_track_m=0.;auto b=body(10.+i*.1);b.local_odometry.pose.pose.position.y=y;
  s.observeBody(b);ASSERT_TRUE(s.observe(p,10.+i*.1));advances+=s.advance(ver(),"d1max_loc_odom",10.+i*.1);
 }
 EXPECT_GE(advances,15); // first leg of a U: moves away from final (1,0), but follows the route.
}
TEST(RouteProgressSupervisor, CornerUsesFullRouteArcNotTheLastLocalCurve){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot({{0.,0.,0.},{1.,0.,0.},{1.,1.,0.}}),ver(),"isolated_mock"));
 for(int i=0;i<=10;++i){const double y=i*.05;auto p=routeProgress(10.+i*.1,i+1,1.,y);
  p.edge_index=i?1:0;p.measured_arc_m=p.confirmed_arc_m=1.+y;p.cross_track_m=0.;
  auto b=body(10.+i*.1,1.);b.local_odometry.pose.pose.position.y=y;s.observeBody(b);ASSERT_TRUE(s.observe(p,10.+i*.1));
  EXPECT_EQ(s.advance(ver(),"d1max_loc_odom",10.+i*.1),i>0);
 }
}
TEST(RouteProgressSupervisor, CorrectionRebasesButNeverManufacturesMotion){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot(),ver(),"isolated_mock"));
 for(int i=0;i<50;++i){const double t=10.+i*.1;auto p=routeProgress(t,i+1);auto b=body(t);
  p.version.anchor_id="anchor"+std::to_string(i);p.version.anchor_revision=i+1;
  p.map_from_odom.position.x=i*.04;p.source_map_body_xyz.x=i*.04;
  p.measured_arc_m=p.confirmed_arc_m=i*.04;p.anchor_source_stamp=stamp(t);
  s.observeBody(b);ASSERT_TRUE(s.observe(p,t));EXPECT_FALSE(s.advance(p.version,"d1max_loc_odom",t));
 }
}
TEST(RouteProgressSupervisor, SameAnchorTransformCannotBeSilentlyChanged){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot(),ver(),"isolated_mock"));
 auto p=routeProgress(10.,1);s.observeBody(body(10.));ASSERT_TRUE(s.observe(p,10.));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.));
 p=routeProgress(10.1,2);p.map_from_odom.position.x=.04;p.source_map_body_xyz.x=.04;
 p.measured_arc_m=p.confirmed_arc_m=.04;s.observeBody(body(10.1));ASSERT_TRUE(s.observe(p,10.1));
 EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.1));
}
TEST(RouteProgressSupervisor, CrossingCannotJumpToAnotherNearbyRouteBranch){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot({{0.,0.,0.},{2.,0.,0.},{2.,.1,0.},{0.,.1,0.}}),ver(),"isolated_mock"));
 auto p=routeProgress(10.,1);s.observeBody(body(10.));ASSERT_TRUE(s.observe(p,10.));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.));
 p=routeProgress(10.1,2,0.,.1);p.edge_index=2;p.measured_arc_m=p.confirmed_arc_m=4.1;p.cross_track_m=0.;
 auto b=body(10.1);b.local_odometry.pose.pose.position.y=.1;s.observeBody(b);ASSERT_TRUE(s.observe(p,10.1));
 EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.1));
}
TEST(RouteProgressSupervisor, OtherFloorOrForeignIdentityCannotProvideProgress){
 auto route=routeSnapshot({{0.,0.,0.},{1.,0.,0.},{1.,0.,3.},{0.,0.,3.}});
 route.edge_segments[2]="floor2";
 route.segments[0].end_index=2;auto second=route.segments[0];second.segment_id="floor2";
 second.begin_index=2;second.end_index=3;route.segments.push_back(second);
 for(int mode=0;mode<5;++mode){RouteProgressSupervisor s;ASSERT_TRUE(s.bind(route,ver(),"isolated_mock"));
  auto p=routeProgress(10.,1,.1);if(mode==0){p.edge_index=2;p.measured_arc_m=p.confirmed_arc_m=4.9;p.source_map_body_xyz.z=3.5;}
  if(mode==1)p.version.task_id="old";if(mode==2)p.version.localization_seed_id="old";
  if(mode==3)p.version.localization_epoch=2;if(mode==4)p.frame_id="wrong";
  EXPECT_FALSE(s.observe(p,10.));
 }
}
TEST(RouteProgressSupervisor, OriginalBodyPoseAndSourceMustMatchLocalObservation){
 for(int mode=0;mode<4;++mode){RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot(),ver(),"isolated_mock"));
  auto p=routeProgress(10.,1);s.observeBody(body(10.));ASSERT_TRUE(s.observe(p,10.));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.));
  p=routeProgress(10.1,2,.04);auto b=body(10.1,.04);
  if(mode==0)b.source_stamp=b.local_odometry.header.stamp=stamp(10.09);
  if(mode==1)b.local_odometry.pose.pose.position.x=.08;
  if(mode==2)b.usable=false;if(mode==3)b.localization_seed_id="old";
  s.observeBody(b);ASSERT_TRUE(s.observe(p,10.1));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.1));
 }
}
TEST(RouteProgressSupervisor, DuplicateAndOutOfOrderProgressDoesNotRenewSource){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot(),ver(),"isolated_mock"));
 auto p=routeProgress(10.,1);s.observeBody(body(10.));ASSERT_TRUE(s.observe(p,10.));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.));
 EXPECT_FALSE(s.observe(p,10.1));p.sequence=2;EXPECT_FALSE(s.observe(p,10.1));
 p=routeProgress(9.99,3,.04);s.observeBody(body(9.99,.04));EXPECT_FALSE(s.observe(p,10.1));
 EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.1));
 p=routeProgress(10.2,4,.04);s.observeBody(body(10.2,.04));ASSERT_TRUE(s.observe(p,10.2));
 EXPECT_TRUE(s.advance(ver(),"d1max_loc_odom",10.2));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.2));
 EXPECT_FALSE(s.observe(routeProgress(10.2,5,.08),10.7));
}
TEST(RouteProgressSupervisor, ClaimedArcWithoutActualLocalTravelCannotAdvance){
 RouteProgressSupervisor s;ASSERT_TRUE(s.bind(routeSnapshot(),ver(),"isolated_mock"));
 for(int i=0;i<4;++i){auto p=routeProgress(10.+i*.1,i+1);p.measured_arc_m=p.confirmed_arc_m=i*.04;p.cross_track_m=i*.04;
  s.observeBody(body(10.+i*.1));ASSERT_TRUE(s.observe(p,10.+i*.1));EXPECT_FALSE(s.advance(ver(),"d1max_loc_odom",10.+i*.1));
 }
}
TEST(ExecutionOwner, FinalYawProgressUsesErrorReductionNotForwardDistance){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 p.phase="aligning";p.goal_yaw=2.;
 for(int i=0;i<100;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);d.velocity.linear.x=0.;d.velocity.angular.z=.2;
  ASSERT_TRUE(o.observe(motionProof(d),t));ASSERT_TRUE(o.observe(d,t));auto b=body(t,0.,i*.02);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());}
 EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, ForeignVersionAndMismatchedMotionPairCannotRecover){
 for(int mode=0;mode<7;++mode){Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
  feedback(o,10.2,1);
  auto b=body(10.2);o.supervise(p,10.2,&b);
  for(int i=1;i<=301;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);auto m=motionProof(d);
   if(mode==0)d.execution_id="old";if(mode==1)d.version.anchor_revision=2;if(mode==2)m.demand_body_source_stamp=stamp(t-.01);
   if(mode==3)m.demand_sequence++;if(mode==4)m.velocity.linear.x=.25;if(mode==5)m.sdk_arm_generation++;
   if(mode==6)m.braking_model_sha256=std::string(64,'c');
   o.observe(d,t);o.observe(m,t);b=body(t,i*.02);const auto reason=o.supervise(p,t,&b);
   if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_FALSE(reason.empty());break;}
  }
 }
}
TEST(ExecutionOwner, LiveRuntimeModelHashMismatchCannotClaimAdmittedMotion) {
 Owner o("live");begin(o,"live");auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 auto admitted=movingDemand(10.2,1);admitted.transport_mode="live";
 ASSERT_TRUE(o.observe(admitted,10.2));ASSERT_TRUE(o.observe(motionProof(admitted),10.2));
 auto b=body(10.2);o.supervise(p,10.2,&b);
 for(int i=1;i<=301;++i) {
  const double t=10.2+i*.1;auto d=movingDemand(t,i+1);d.transport_mode="live";
  auto m=motionProof(d);m.braking_model_sha256=std::string(64,'c');
  ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(m,t));b=body(t,i*.02);
  const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {
   EXPECT_EQ(reason,"execution_blocked_timeout:waiting_admitted_nonzero_command");
   EXPECT_TRUE(o.stopping());break;
  }
 }
}
TEST(ExecutionOwner, ActualSweepMayUseNewerCurveEvidenceThanOriginalDemandFloor){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<500;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);auto m=motionProof(d);
  m.trajectory_validation_sequence=d.validation_sequence+2;
  ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(m,t));ASSERT_TRUE(o.observe(routeProgress(t,i+1,i*.02),t));auto b=body(t,i*.02);
  EXPECT_TRUE(o.supervise(p,t,&b).empty());}
 EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, LateralTrackingIsAdmittedMotionNotAFalseHold){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 for(int i=0;i<500;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);
  d.velocity.linear.x=0.;d.velocity.linear.y=.2;
  ASSERT_TRUE(o.observe(d,t));ASSERT_TRUE(o.observe(motionProof(d),t));
  auto b=body(t,i*.02,-1.5707963267948966);
  auto progress=routeProgress(t,i+1,i*.02);progress.odom_body_pose.pose=b.local_odometry.pose.pose;
  ASSERT_TRUE(o.observe(progress,t));
  EXPECT_TRUE(o.supervise(p,t,&b).empty());}
 EXPECT_FALSE(o.stopping());
}
TEST(ExecutionOwner, EarlierCurveEvidenceCannotSatisfyDemandFloor){
 Owner o("isolated_mock");begin(o);auto p=o.tick(10.1,true);p.frame_id="d1max_loc_odom";
 feedback(o,10.2,1);auto b=body(10.2);o.supervise(p,10.2,&b);
 for(int i=1;i<=301;++i){const double t=10.2+i*.1;auto d=movingDemand(t,i+1);auto m=motionProof(d);
  m.trajectory_validation_sequence=d.validation_sequence-1;
  o.observe(d,t);o.observe(m,t);auto b=body(t,i*.02);const auto reason=o.supervise(p,t,&b);
  if(i<300)EXPECT_TRUE(reason.empty());else {EXPECT_FALSE(reason.empty());break;}}
}
TEST(CommandPipelineOwner, SubmittedOriginalDemandAgeUsesBoundRecordButActualBodyDoesNot) {
 for(int fault=0;fault<5;++fault){Owner o("isolated_mock");o.configureCommandPipeline(.2);beginUnapplied(o);
  const auto p=o.tick(10.1,true);auto actual=initialCommit(o,p,10.225);
  actual.demand_source_stamp=actual.demand_body_source_stamp=stamp(10.1);
  actual.body_source_stamp=actual.measured_pose.header.stamp=stamp(10.215);
  if(fault==1)actual.demand_source_stamp=actual.demand_body_source_stamp=stamp(10.024);
  if(fault==2)actual.body_source_stamp=actual.measured_pose.header.stamp=stamp(10.124);
  if(fault==3)actual.valid_until=stamp(10.224);
  if(fault==4)actual.demand_body_source_stamp=stamp(10.101);
  EXPECT_EQ(o.observe(actual,10.225),fault==0)<<fault;
  if(fault==0)EXPECT_EQ(o.appliedCommitSequence(),1u);
 }
 Owner legacy("isolated_mock");beginUnapplied(legacy);auto actual=initialCommit(legacy,legacy.tick(10.1,true),10.225);
 actual.demand_source_stamp=actual.demand_body_source_stamp=stamp(10.1);
 actual.body_source_stamp=actual.measured_pose.header.stamp=stamp(10.215);
 EXPECT_FALSE(legacy.observe(actual,10.225));
}
TEST(CommandPipelineOwner, CannotChangeRecordBudgetAfterExecutionConfirmation) {
 Owner o("isolated_mock");
 for(double invalid:{.099,.501,std::numeric_limits<double>::quiet_NaN()})
  EXPECT_THROW(o.configureCommandPipeline(invalid),std::invalid_argument);
 o.configureCommandPipeline(.2);beginUnapplied(o);
 EXPECT_THROW(o.configureCommandPipeline(.3),std::invalid_argument);
}
