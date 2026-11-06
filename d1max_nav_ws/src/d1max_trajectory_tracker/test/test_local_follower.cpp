#include <gtest/gtest.h>
#include "d1max_trajectory_tracker/local_follower.hpp"
#include "d1max_trajectory_tracker/local_commit_dispatch.hpp"
using namespace d1max_trajectory_tracker;

namespace d1max_trajectory_tracker {
struct LocalFollowerTestAccess {
  static TrackerCore& core(LocalFollower& f){return f.core_;}
  static bool recovering(const LocalFollower& f){return f.writer_recovering_;}
  static void enableProductionWorker(LocalFollower& f) {
    f.worker_=std::make_unique<LocalFollower::Worker>([](const auto& p,const auto& allowed) {
      return PreparedGeometry::build(p.config,p.curve,supportEvidence(p.support),allowed);
    });
  }
  static bool completionReady(LocalFollower& f) {
    std::lock_guard<std::mutex> lock(f.worker_->mutex_);return f.worker_->completed_.has_value();
  }
  static std::optional<wire::LocalTrajectoryCommit> nestedReservedCommit(LocalFollower& f,double now,double receipt) {
    // Reproduce the actual exchangeCommit interleaving: newer preparation
    // finishes after its outer poll, before tryCommit polls the temporary
    // reserved-old view. No controller callback or fake geometry is injected.
    auto newer=std::move(f.pending_);f.pending_=f.exchange_->candidate;
    auto commit=f.tryCommit(now,receipt);f.pending_=std::move(newer);return commit;
  }
  static void pollWithoutPending(LocalFollower& f) {
    auto pending=std::move(f.pending_);f.pending_.reset();f.poll();f.pending_=std::move(pending);
  }
  static bool pollPrepared(LocalFollower& f,std::int64_t id) {
    f.poll();return f.pending_&&f.pending_->curve.id==id&&f.pending_->curve.prepared&&
      f.pending_->curve.prepared->failure.empty();
  }
};
}
TEST(LocalCommitDispatch, NewPhaseIsImmediateButIdenticalRetriesAreBoundedAndUnrestamped) {
 LocalCommitDispatch dispatch;wire::LocalTrajectoryCommit request;
 request.commit_sequence=1;request.source_stamp=stamp(100.);request.disposition="prepare";
 std::vector<wire::LocalTrajectoryCommit> sent;
 const auto send=[&](const auto& m){sent.push_back(m);};
 ASSERT_TRUE(dispatch.forward(request,10.,send));
 EXPECT_FALSE(dispatch.forward(request,10.001,send));
 EXPECT_FALSE(dispatch.forward(request,10.049,send));
 EXPECT_TRUE(dispatch.forward(request,10.051,send));
 ASSERT_EQ(sent.size(),2u);EXPECT_EQ(sent[0],sent[1]);
 auto commit=request;commit.disposition="committed";
 EXPECT_TRUE(dispatch.forward(commit,10.052,send));
 EXPECT_EQ(sent.back().source_stamp,request.source_stamp);
 EXPECT_FALSE(dispatch.forward(commit,10.053,send));
 EXPECT_FALSE(dispatch.forward(commit,9.9,send));
 EXPECT_FALSE(dispatch.forward(commit,std::numeric_limits<double>::quiet_NaN(),send));
 EXPECT_FALSE(dispatch.forward(std::nullopt,10.06,send));
 commit.commit_sequence=2;commit.source_stamp=stamp(100.06);
 EXPECT_TRUE(dispatch.forward(commit,10.06,send));EXPECT_EQ(sent.size(),4u);
}
namespace {
Config cfg(){Config c;c.session_id="session";c.map_version_id="map";c.motion_model="holonomic";
 c.require_support_reference=true;c.external_goal_completion=true;return c;}
wire::ExecutionVersion ver(unsigned generation=1){wire::ExecutionVersion v;v.schema_version=3;
 v.session_id="session";v.task_id="task";v.route_id="route";v.route_hash=std::string(64,'a');v.map_version_id="map";
 v.localization_epoch=v.anchor_revision=v.context_sequence=v.map_geometry_revision=1;v.localization_seed_id="seed";
 v.reference_generation=generation;v.segment_id="corridor";v.anchor_id="anchor";return v;}
wire::ReferenceProposal proposal(wire::ExecutionVersion v=ver(),double now=100.) {
 wire::ReferenceProposal p;p.version=v;p.proposal_id="p"+std::to_string(v.reference_generation);
 p.expected_trajectory_id=-1;p.source_stamp=stamp(now);p.valid_until=stamp(now+.9);
 p.reference.path.header.frame_id="d1max_loc_odom";p.reference.point_reference="body_center";
 p.goal_position.x=3.;p.goal_position.z=.55;p.transport_mode="isolated_mock";return p;
}
wire::SupportReference support(wire::ExecutionVersion v=ver()) {
 wire::SupportReference s;s.version=v;s.support_reference_id="support";s.support_hash=std::string(64,'b');
 s.support_map_sha256=std::string(64,'c');s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
 s.frame_id="d1max_loc_odom";s.source_stamp=stamp(100.);s.support_xy_radius_m=.3;s.body_reference_height_m=.55;
 s.max_support_slope_rad=.15;s.max_support_step_m=.05;s.verified=true;
 for(int i=-3;i<=100;i++){geometry_msgs::msg::Point p;p.x=i*.05;s.support_ground_xyz.push_back(p);}return s;
}
wire::SupportReference modelSupport() {
 auto s=support();s.verified=false;s.support_ground_xyz.clear();s.stair_model_hash=std::string(64,'d');
 wire::StairSurfaceModel surface;surface.surface_id="flight";surface.segment_id="corridor";
 surface.kind="continuous_support_envelope";surface.pose.orientation.w=1.;surface.support_grade=.5;
 surface.height_association_m=.25;
 for(const auto& xy:std::vector<std::pair<double,double>>{{-1.,-.7},{3.,-.7},{3.,.7},{-1.,.7}}) {
   geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;surface.polygon.push_back(p);
 }
 s.stair_model_surfaces.push_back(surface);return s;
}
wire::TaggedBspline tag(wire::ExecutionVersion v=ver(),std::int64_t id=1,double x=0.,double now=100.) {
 wire::TaggedBspline t;t.schema_version=2;t.session_id=v.session_id;t.task_id=v.task_id;t.route_id=v.route_id;
 t.route_hash=v.route_hash;t.map_version_id=v.map_version_id;t.localization_epoch=v.localization_epoch;
 t.localization_seed_id=v.localization_seed_id;t.generation=v.reference_generation;t.segment_id=v.segment_id;
 t.anchor_id=v.anchor_id;t.anchor_revision=v.anchor_revision;t.context_sequence=v.context_sequence;
 t.map_geometry_revision=v.map_geometry_revision;t.frame_id="d1max_loc_odom";t.point_reference="body_center";
 t.trajectory.order=3;t.trajectory.traj_id=id;t.trajectory.start_time=stamp(now);
 for(int i=0;i<23;i++){geometry_msgs::msg::Point p;p.x=x+(i-1)*.05;p.z=.55;t.trajectory.pos_pts.push_back(p);}
 for(int i=0;i<27;i++)t.trajectory.knots.push_back((i-3)*.2);
 t.join_source_stamp=stamp(now);t.join_pose.position.x=x;t.join_pose.position.z=.55;t.join_pose.orientation.w=1.;
 t.join_twist.linear.x=.25;return t;
}
wire::TrajectoryValidation proof(wire::ExecutionVersion v=ver(),std::int64_t id=1,double now=100.,unsigned seq=1) {
 wire::TrajectoryValidation p;p.version=v;p.proposal_id=proposal(v).proposal_id;p.trajectory_id=id;p.sequence=seq;
 p.check_begin=p.check_end=p.source_stamp=p.body_source_stamp=stamp(now);
 p.valid_until=stamp(0.); // Formal persistent SCAN state, never a timed motion lease.
 p.valid=p.whole_curve=true;p.checked_to_time=p.curve_duration=4.;p.map_snapshot_revision=1;
 p.support_reference_id="support";p.support_hash=std::string(64,'b');p.frame_id="d1max_loc_odom";
 p.collision_policy="observed_free";p.transport_mode="isolated_mock";return p;
}
Odom body(double now=100.,double x=0.,double vx=.25) {
 Odom o;o.schema_version=2;o.session_id="session";o.map_version_id="map";o.localization_epoch=1;o.localization_seed_id="seed";
 o.frame_id="d1max_loc_odom";o.child_frame_id="d1max_loc_base_link";o.stamp=o.posterior_stamp=o.imu_stamp=now;
 o.position={x,0.,.55};o.velocity_in_frame={vx,0.,0.};o.planar_speed=std::abs(vx);o.source_stamp_ns=timeNs(stamp(now));
 o.provider_contract=o.provider_usable=true;o.provider_valid_until_ns=o.source_stamp_ns+100000000LL;
 o.provider_posterior_stamp_ns=o.provider_imu_stamp_ns=o.source_stamp_ns;return o;
}
wire::TaskExecutionLease lease(double now=100.,unsigned seq=1) {
 wire::TaskExecutionLease p;p.schema_version=1;p.version=ver();p.execution_id="execution";p.control_epoch=1;
 p.confirmation_id="confirm";p.sdk_session="sdk";p.sdk_arm_generation=1;p.sequence=seq;
 p.source_stamp=stamp(now);p.valid_until=stamp(now+.35);p.allowed=true;p.phase="tracking";p.transport_mode="isolated_mock";return p;
}
void stage(LocalFollower& f,wire::ReferenceProposal p=proposal()) {
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(p,100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));ASSERT_TRUE(f.validation(proof()));
 ASSERT_TRUE(f.tryCommit(100.,10.));
}
void stageExchange(LocalFollower& f) {
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));ASSERT_TRUE(f.validation(proof()));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);ASSERT_EQ(request->disposition,"prepare");
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.,10.));
 const auto commit=f.exchangeCommit(100.,10.);ASSERT_TRUE(commit);ASSERT_EQ(commit->disposition,"committed");
 ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,100.,10.));
}
TEST(LocalFollowerNoticeDelivery, CommittedAuthorizedCurveKeepsFreshDemandsWhilePositiveNoticeIsDelayed) {
 auto config=cfg();config.max_jerk=1.4;LocalFollower f(config,"isolated_mock",{},false);
 stageExchange(f);ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
 double previous=0.,acceleration=0.;
 for(unsigned i=1;i<=9;++i) {
  const double now=100.+.05*i;ASSERT_TRUE(f.body(body(now,.25*(now-100.)),now,now-90.));
  ASSERT_TRUE(f.lease(lease(now,i+1),now));f.geometryPermit(now);
  const auto demand=f.step(now,now-90.);
  ASSERT_FALSE(demand.hold)<<i<<" "<<demand.reason;EXPECT_GT(demand.velocity.linear.x,0.);
  EXPECT_EQ(demand.validation_sequence,1u);EXPECT_EQ(demand.trajectory_id,1);
  EXPECT_EQ(demand.body_source_stamp,stamp(now));EXPECT_EQ(demand.source_stamp,stamp(now));
  EXPECT_LE(seconds(demand.valid_until),now+.100000001); // The real body deadline, not the old map proof.
  const double next_acceleration=(demand.velocity.linear.x-previous)/.05;
  EXPECT_LE(std::abs(next_acceleration),config.max_acceleration+1e-8);
  EXPECT_LE(std::abs(next_acceleration-acceleration)/.05,config.max_jerk+1e-7);
  previous=demand.velocity.linear.x;acceleration=next_acceleration;
  const auto status=f.status(now);
  if(i>=4) {
   EXPECT_TRUE(f.validationDiagnostic(now).failure.empty());
   EXPECT_TRUE(status.ready);EXPECT_FALSE(status.holding);EXPECT_EQ(status.phase,"following");
   EXPECT_FALSE(status.request_stop);EXPECT_FALSE(status.software_retired);
   EXPECT_FALSE(f.controller().holding()); // Notification wait is not a controller recovery event.
   const auto progress=trackingProgress(f.controller(),config,now,status);
   EXPECT_TRUE(progress.valid);EXPECT_FALSE(progress.holding);EXPECT_EQ(progress.header.stamp,stamp(now));
  }
 }
 ASSERT_TRUE(f.validation(proof(ver(),1,100.46,2)));ASSERT_TRUE(f.body(body(100.50,.125),100.50,10.50));
 ASSERT_TRUE(f.lease(lease(100.50,11),100.50));f.geometryPermit(100.50);
 const auto next=f.step(100.50,10.50);EXPECT_FALSE(next.hold);EXPECT_GT(next.velocity.linear.x,0.);
 EXPECT_EQ(f.status(100.50).phase,"following");EXPECT_FALSE(f.status(100.50).holding);
 EXPECT_EQ(f.commitSequence(),1u);EXPECT_EQ(f.controller().trajectoryId(),1);
}
TEST(LocalFollowerPersistentState, FiveSecondsWithoutAnotherCollisionNoticeExactlyMatchesRepeatedState) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;c.max_yaw_jerk=4.;
 LocalFollower persistent(c,"isolated_mock",{},false),repeated(c,"isolated_mock",{},false);
 stageExchange(persistent);stageExchange(repeated);
 for(unsigned i=1;i<=100;++i) {
  const double now=100.+i*.05;
  for(auto* f:{&persistent,&repeated}) {
   ASSERT_TRUE(f->body(body(now,.1,0.),now,now-90.));ASSERT_TRUE(f->lease(lease(now,i),now));
   f->geometryPermit(now);
  }
  ASSERT_TRUE(repeated.validation(proof(ver(),1,now,i+1)));
  const auto a=persistent.step(now,now-90.),b=repeated.step(now,now-90.);
  EXPECT_EQ(a.velocity,b.velocity);EXPECT_EQ(a.hold,b.hold);EXPECT_EQ(a.reason,b.reason);
  EXPECT_EQ(a.body_source_stamp,stamp(now));EXPECT_EQ(a.source_stamp,stamp(now));
  EXPECT_EQ(a.valid_until,stamp(now+.1));
  EXPECT_FALSE(LocalFollowerTestAccess::recovering(persistent));
  EXPECT_FALSE(persistent.status(now).holding);EXPECT_EQ(persistent.status(now).phase,"following");
 }
 EXPECT_EQ(persistent.feedbackDiagnostic().rebases,0u);
 const auto diagnostic=persistent.validationDiagnostic(105.);
 EXPECT_EQ(diagnostic.sequence,1u);EXPECT_EQ(diagnostic.source_ns,timeNs(stamp(100.)));
 EXPECT_EQ(diagnostic.valid_until_ns,0);EXPECT_TRUE(diagnostic.persistent_state);
 EXPECT_TRUE(diagnostic.failure.empty());EXPECT_DOUBLE_EQ(diagnostic.age_s,5.);
 EXPECT_TRUE(std::isnan(diagnostic.remaining_s));
}
TEST(LocalFollowerPersistentState, NegativeRemainsUntilANewerCompletedSameCurveState) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
 auto negative=proof(ver(),1,100.01,2);negative.valid=false;negative.reason="occupied";
 ASSERT_TRUE(f.validation(negative));
 for(unsigned i=1;i<=80;++i) {
  const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.1,0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  EXPECT_FALSE(f.validation(proof())); // A delayed former positive is never current again.
  ASSERT_TRUE(f.validation(proof(ver(),100+i,now,100+i))); // Unrelated candidate cache pressure.
  f.geometryPermit(now);EXPECT_TRUE(f.step(now,now-90.).hold);
  EXPECT_EQ(f.validationDiagnostic(now).sequence,2u);
  EXPECT_EQ(f.validationDiagnostic(now).native_reason,"occupied");
 }
 auto same_sequence=proof(ver(),1,104.01,2);EXPECT_FALSE(f.validation(same_sequence));
 ASSERT_TRUE(f.validation(proof(ver(),1,104.05,3)));
 ASSERT_TRUE(f.body(body(104.05,.1,0.),104.05,14.05));ASSERT_TRUE(f.lease(lease(104.05,81),104.05));
 f.geometryPermit(104.05);EXPECT_FALSE(f.step(104.05,14.05).hold);
 EXPECT_EQ(f.commitSequence(),1u);EXPECT_EQ(f.controller().trajectoryId(),1);
}
TEST(LocalFollowerPersistentState, CandidateCompletedStateOlderThan250msStillRequiresActualJoin) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));ASSERT_TRUE(f.validation(proof()));
 ASSERT_TRUE(f.body(body(100.3,.075),100.3,10.3));
 const auto request=f.exchangeCommit(100.3,10.3);ASSERT_TRUE(request)<<f.candidateReason();
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.3,10.3));
 const auto commit=f.exchangeCommit(100.3,10.3);ASSERT_TRUE(commit)<<f.candidateReason();
 EXPECT_EQ(commit->disposition,"committed");EXPECT_EQ(commit->native_validation_sequence,1u);
 EXPECT_EQ(commit->body_source_stamp,stamp(100.3));EXPECT_GT(commit->arc_length,.05);
}
wire::TaskCommandFeedback sendFact(double now,unsigned write,unsigned gap=0) {
 wire::TaskCommandFeedback f;f.schema_version=1;f.version=ver();f.execution_id="execution";
 f.control_epoch=1;f.sdk_session="sdk";f.sdk_arm_generation=1;f.transport_mode="isolated_mock";
 f.write_sequence=write;f.source_stamp=stamp(now);f.write_submitted=true;
 f.discontinuity_write_sequence=gap;
 if(gap){f.discontinuity_stamp=stamp(100.);f.interrupted_command_sequence=1;}
 return f;
}
TEST(LocalFollowerWriterFeedback, HealthyFactsAreExactlyTransparentAndForeignFactsCannotReset) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;c.max_yaw_jerk=4.;
 c.forward_response={.3,.31328213472822414,.5,.1};c.yaw_response={.5,.2,.3,.2};
 c.max_input_rate=1.6;c.max_input_acceleration=12.;
 LocalFollower a(c,"isolated_mock",{},false),b(c,"isolated_mock",{},false);stageExchange(a);stageExchange(b);
 for(unsigned i=1;i<=30;++i) {
  const double now=100.+i*.05;
  for(auto* f:{&a,&b}){ASSERT_TRUE(f->body(body(now,.2*i*.05,.2),now,now-90.));
   ASSERT_TRUE(f->lease(lease(now,i),now));ASSERT_TRUE(f->validation(proof(ver(),1,now,i+1)));f->geometryPermit(now);}
  auto fact=sendFact(now,i);fact.command_selected=fact.sdk_nonzero=true;
  ASSERT_TRUE(b.commandFeedback(fact,now));
  auto foreign=sendFact(now,100+i,1);foreign.sdk_session="retired-sdk";EXPECT_FALSE(b.commandFeedback(foreign,now));
  const auto x=a.step(now,now-90.),y=b.step(now,now-90.);
  EXPECT_EQ(x,y); // All fields, not just nonzero counts.
 }
}
TEST(LocalFollowerWriterFeedback, FreshEvidenceRecoversMovingBodyWithoutWaitingForFeedbackAck) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 wire::MotionDemand demand;
 for(unsigned i=1;i<=30;++i){const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,std::min(.2*i*.05,.15),i<15?.2:0.),now,now-90.));
  ASSERT_TRUE(f.lease(lease(now,i),now));ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));f.geometryPermit(now);
  if(i==10)ASSERT_TRUE(f.commandFeedback(sendFact(now,20,20),now));
  // Same-task different anchor/reference does not hide the sticky gap.
  if(i==20){auto fact=sendFact(now,30,20);fact.version.anchor_id="old-anchor";
   fact.version.reference_generation=99;ASSERT_TRUE(f.commandFeedback(fact,now));}
  demand=f.step(now,now-90.);
  if(i>=10)EXPECT_EQ(demand.acknowledged_discontinuity_write,20u);
  if(i==10)EXPECT_GT(demand.velocity.linear.x,.04); // Actual moving body, not an invented stop.
 }
 ASSERT_GT(demand.velocity.linear.x,0.);const auto before=f.controller().progress(101.5);
 auto sent=sendFact(101.51,40,20);sent.sdk_nonzero=sent.command_selected=true;
 sent.command_sequence=demand.sequence;sent.command_source_stamp=demand.source_stamp;sent.velocity=demand.velocity;
 ASSERT_TRUE(f.commandFeedback(sent,101.51));EXPECT_FALSE(f.commandFeedback(sendFact(101.5,39,20),101.51));
 ASSERT_TRUE(f.body(body(101.55,.15,0.),101.55,11.55));ASSERT_TRUE(f.lease(lease(101.55,31),101.55));
 ASSERT_TRUE(f.validation(proof(ver(),1,101.55,32)));f.geometryPermit(101.55);
 const auto resumed=f.step(101.55,11.55);EXPECT_GE(resumed.velocity.linear.x,demand.velocity.linear.x);
 EXPECT_LE(resumed.velocity.linear.x-demand.velocity.linear.x,c.max_acceleration*.05+1e-8);
 EXPECT_GE(f.controller().progress(101.55).s_committed,before.s_committed);
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 // A later latest-only repeat of this watermark does not reset a recovered command.
 sent.write_sequence=41;sent.source_stamp=stamp(101.56);ASSERT_TRUE(f.commandFeedback(sent,101.56));
 ASSERT_TRUE(f.body(body(101.60,.15,0.),101.60,11.60));ASSERT_TRUE(f.lease(lease(101.60,32),101.60));
 ASSERT_TRUE(f.validation(proof(ver(),1,101.60,33)));f.geometryPermit(101.60);
 const auto continuing=f.step(101.60,11.60);EXPECT_GE(continuing.velocity.linear.x,resumed.velocity.linear.x);
 EXPECT_LE(continuing.velocity.linear.x-resumed.velocity.linear.x,c.max_acceleration*.05+1e-8);
 f.retire("operator_cancel");EXPECT_FALSE(f.commandFeedback(sendFact(101.65,42,20),101.65));
 EXPECT_TRUE(f.step(101.65,11.65).hold);
}
TEST(LocalFollowerWriterFeedback, InvalidWatermarksDoNotPoisonMonotonicReceipt) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);ASSERT_TRUE(f.lease(lease(),100.));
 for(unsigned kind=0;kind<7;++kind){auto fact=sendFact(100.,100,20);
  if(kind==0)fact.execution_id="old";if(kind==1)fact.control_epoch=2;
  if(kind==2)fact.version.task_id="other";if(kind==3)fact.sdk_arm_generation=2;
  if(kind==4)fact.discontinuity_write_sequence=101;
  if(kind==5)fact.discontinuity_stamp=stamp(100.1);
  if(kind==6)fact.source_stamp.nanosec=1000000000U;
  EXPECT_FALSE(f.commandFeedback(fact,100.));
 }
 EXPECT_TRUE(f.commandFeedback(sendFact(100.,20,20),100.));
 EXPECT_FALSE(f.commandFeedback(sendFact(100.,21,0),100.));
}
TEST(LocalFollowerWriterFeedback, DelayedClosedGapFactDoesNotRewindCurrentController) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 wire::MotionDemand first,last;
 for(unsigned i=1;i<=25;++i){const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.1,0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));f.geometryPermit(now);
  if(i==1)ASSERT_TRUE(f.commandFeedback(sendFact(now,20,20),now));
  last=f.step(now,now-90.);if(i==3)first=last;
 }
 ASSERT_GT(first.velocity.linear.x,0.);
 auto actual=sendFact(100.16,21,20);actual.sdk_nonzero=actual.command_selected=true;
 actual.command_sequence=first.sequence;actual.command_source_stamp=first.source_stamp;actual.velocity=first.velocity;
 ASSERT_TRUE(f.commandFeedback(actual,101.25));
 ASSERT_TRUE(f.body(body(101.3,.1,0.),101.3,11.3));ASSERT_TRUE(f.lease(lease(101.3,26),101.3));
 ASSERT_TRUE(f.validation(proof(ver(),1,101.3,27)));f.geometryPermit(101.3);
 const auto recovered=f.step(101.3,11.3);
 EXPECT_DOUBLE_EQ(recovered.velocity.linear.x,last.velocity.linear.x);
 EXPECT_GT(recovered.velocity.linear.x,first.velocity.linear.x+.02);
}
TEST(LocalFollowerWriterFeedback, FieldNineteenMillisecondGapPreservesFreshMovingBoundary) {
 // Field 1067 -> SDK write166 zero -> 1068: same curve, 19.22ms gap,
 // measured body still moving. Synthetic straight geometry, original tick
 // offsets; this is NOT a replay of the unrecorded field point cloud.
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;c.max_yaw_jerk=4.;
 c.forward_response={.3,.31328213472822414,.5,.1};c.yaw_response={.5,.2,.3,.2};
 c.max_input_rate=1.6;c.max_input_acceleration=12.;
 LocalFollower uninterrupted(c,"isolated_mock",{},false),interrupted(c,"isolated_mock",{},false);
 stageExchange(uninterrupted);stageExchange(interrupted);wire::MotionDemand last;
 for(unsigned i=1;i<=25;++i) {
  const double now=100.+i*.05;
  for(auto* f:{&uninterrupted,&interrupted}) {
   ASSERT_TRUE(f->body(body(now,.25*i*.05,.494385),now,now-90.));
   ASSERT_TRUE(f->lease(lease(now,i),now));auto p=proof(ver(),1,now,i+1);
   ASSERT_TRUE(f->validation(p));f->geometryPermit(now);
  }
  const auto normal=uninterrupted.step(now,now-90.);last=interrupted.step(now,now-90.);
  EXPECT_EQ(normal.velocity,last.velocity);
 }
 ASSERT_GT(last.velocity.linear.x,.15);
 auto zero=sendFact(101.28158,166,166);zero.discontinuity_stamp=zero.source_stamp;
 zero.interrupted_command_sequence=last.sequence;zero.interrupted_command_source_stamp=last.source_stamp;
 ASSERT_TRUE(interrupted.commandFeedback(zero,101.28158));
 for(unsigned i=26;i<=30;++i) {
  const double now=100.+i*.05;
  for(auto* f:{&uninterrupted,&interrupted}) {
   ASSERT_TRUE(f->body(body(now,.25*i*.05,.494385),now,now-90.));
   ASSERT_TRUE(f->lease(lease(now,i),now));ASSERT_TRUE(f->validation(proof(ver(),1,now,i+1)));f->geometryPermit(now);
  }
  const auto a=uninterrupted.step(now,now-90.),b=interrupted.step(now,now-90.);
  EXPECT_EQ(a.velocity,b.velocity); // No nonzero feedback is needed to progress.
  EXPECT_GT(b.velocity.linear.x,.15);
 }
}
TEST(LocalFollowerWriterFeedback, RealNegativeKeepsUnsentIntentUnchargedAndRestRestartsFromRest) {
  auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
  LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);wire::MotionDemand last;
  for(unsigned i=1;i<=15;++i) {const double now=100.+i*.05;
   ASSERT_TRUE(f.body(body(now,.2*i*.05,.2),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
   auto p=proof(ver(),1,now,i+1);
   ASSERT_TRUE(f.validation(p));f.geometryPermit(now);last=f.step(now,now-90.);
  }
  ASSERT_GT(last.velocity.linear.x,.05);
  auto zero=sendFact(100.79,80,80);zero.discontinuity_stamp=zero.source_stamp;
  zero.interrupted_command_sequence=last.sequence;zero.interrupted_command_source_stamp=last.source_stamp;
  ASSERT_TRUE(f.commandFeedback(zero,100.79));
  // A genuine latest negative state, not notification age, stops intent.
  auto negative=proof(ver(),1,100.80,90);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(f.validation(negative));
  for(unsigned i=16;i<=35;++i) {const double now=100.+i*.05;
   ASSERT_TRUE(f.body(body(now,.15,0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));f.geometryPermit(now);
   const auto d=f.step(now,now-90.);
   EXPECT_TRUE(d.hold);
  }
  ASSERT_TRUE(f.validation(proof(ver(),1,101.8,91)));ASSERT_TRUE(f.body(body(101.8,.15,0.),101.8,11.8));
  ASSERT_TRUE(f.lease(lease(101.8,36),101.8));f.geometryPermit(101.8);
  const auto resumed=f.step(101.8,11.8);EXPECT_GT(resumed.velocity.linear.x,0.);
  EXPECT_LE(resumed.velocity.linear.x,c.max_jerk*.05*.05+1e-8);
  // Lost feedback must not create a permanent first-step floor after fresh
  // Native authority returns. Its next ordinary tick may accelerate normally.
  ASSERT_TRUE(f.validation(proof(ver(),1,101.85,92)));ASSERT_TRUE(f.body(body(101.85,.15,0.),101.85,11.85));
  ASSERT_TRUE(f.lease(lease(101.85,37),101.85));f.geometryPermit(101.85);
  EXPECT_GT(f.step(101.85,11.85).velocity.linear.x,resumed.velocity.linear.x);
}
class LocalFollowerInterruptedReplacement : public ::testing::Test {
 protected:
  Config c=[] {auto config=cfg();config.align_then_forward=true;config.max_jerk=.7;return config;}();
  LocalFollower f{c,"isolated_mock",{},false};
  wire::MotionDemand previous;
  void SetUp() override {
    stageExchange(f);
    for(unsigned i=1;i<=25;++i) {
      const double now=100.+i*.05;
      ASSERT_TRUE(f.body(body(now,.1,.3),now,now-90.));
      ASSERT_TRUE(f.lease(lease(now,i),now));ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));
      f.geometryPermit(now);previous=f.step(now,now-90.);
    }
    ASSERT_GT(previous.velocity.linear.x,.1);
    // Same ordering as the stair field stop: a completed negative incumbent
    // verdict, while body input and task authority remain fresh.
    ASSERT_TRUE(f.body(body(101.3,.1,.3),101.3,11.3));
    auto negative=proof(ver(),1,101.3,100);negative.valid=false;negative.reason="occupied";
    ASSERT_TRUE(f.validation(negative));ASSERT_TRUE(f.lease(lease(101.3,26),101.3));f.geometryPermit(101.3);
    ASSERT_TRUE(f.step(101.3,11.3).hold);
    ASSERT_TRUE(LocalFollowerTestAccess::recovering(f));
    ASSERT_DOUBLE_EQ(f.controller().commandHistory().output.forward,previous.velocity.linear.x);
  }
  void reserve(double now,std::int64_t id=2) {
    // This short, newly safe curve cannot be entered at the old cruising
    // command, but is feasible from measured rest. Production spline/entry
    // preparation and the real reserve/commit protocol are used unchanged.
    auto candidate=tag(ver(),id,.1,now);
    for(auto& p:candidate.trajectory.pos_pts)p.x=.1+(p.x-.1)*.01;
    ASSERT_TRUE(f.candidate(candidate));ASSERT_TRUE(f.validation(proof(ver(),id,now,100+id)));
    const auto request=f.exchangeCommit(now,now-90.);ASSERT_TRUE(request)<<f.candidateReason();
    ASSERT_EQ(request->disposition,"prepare");auto ack=*request;ack.accepted=true;ack.disposition="prepared";
    ASSERT_TRUE(f.commitResult(ack,now,now-90.));
  }
};
TEST_F(LocalFollowerInterruptedReplacement, FreshMeasuredRestAdmitsReplacementWithoutRevivingNegativeIncumbent) {
 ASSERT_TRUE(f.body(body(101.35,.1,0.),101.35,11.35));reserve(101.35);
 const auto commit=f.exchangeCommit(101.35,11.35);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed")<<commit->reason;
 EXPECT_EQ(f.controller().trajectoryId(),1); // Final ACK, not preparation, changes the owner.
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,0.);
 EXPECT_EQ(f.validationDiagnostic(101.35).native_reason,"occupied");
 auto ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,101.35,11.35));
 EXPECT_EQ(f.controller().trajectoryId(),2);
 EXPECT_EQ(f.activeVersion()->task_id,ver().task_id);EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);
 ASSERT_TRUE(f.lease(lease(101.35,27),101.35));f.geometryPermit(101.35);
 const auto resumed=f.step(101.35,11.35);EXPECT_FALSE(resumed.hold)<<resumed.reason;
 EXPECT_GT(resumed.velocity.linear.x,0.);EXPECT_LE(resumed.velocity.linear.x,c.max_jerk*.05*.05+1e-8);
 EXPECT_EQ(f.feedbackDiagnostic().rebases,1u);
}
TEST_F(LocalFollowerInterruptedReplacement, StillMovingCannotUseTheZeroDemandAsAStop) {
 ASSERT_TRUE(f.body(body(101.35,.1,.3),101.35,11.35));reserve(101.35);
 const auto rejected=f.exchangeCommit(101.35,11.35);ASSERT_TRUE(rejected);
 EXPECT_EQ(rejected->disposition,"abort");EXPECT_EQ(rejected->reason,"curvature_braking_limit_below_current_command");
 EXPECT_EQ(f.controller().trajectoryId(),1);
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,previous.velocity.linear.x);
 EXPECT_EQ(f.feedbackDiagnostic().rebases,0u);
}
TEST_F(LocalFollowerInterruptedReplacement, NoNewBodySampleCannotInventMeasuredRest) {
 // Do not deliver an after-stop body sample. The original moving state is
 // still within its lease, but a repeated source is not new stop evidence.
 EXPECT_FALSE(f.body(body(101.3,.1,0.),101.35,11.35));reserve(101.35);
 const auto rejected=f.exchangeCommit(101.35,11.35);
 if(rejected)EXPECT_EQ(rejected->disposition,"abort");
 EXPECT_NE(f.controller().trajectoryId(),2);
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,previous.velocity.linear.x);
 EXPECT_EQ(f.feedbackDiagnostic().rebases,0u);
}
TEST_F(LocalFollowerInterruptedReplacement, StaleStateCannotAdmitAndLateWriterCannotRestorePreStopSpeed) {
 reserve(101.35);
 ASSERT_TRUE(f.validation(proof(ver(),2,101.45,103)));
 EXPECT_FALSE(f.exchangeCommit(101.45,11.45));
 EXPECT_EQ(f.candidateReason(),"entry_body_not_fresh");
 EXPECT_EQ(f.controller().trajectoryId(),1);
 ASSERT_TRUE(f.lease(lease(101.45,27),101.45));f.geometryPermit(101.45);
 EXPECT_TRUE(f.step(101.45,11.45).hold);
 ASSERT_TRUE(f.body(body(101.5,.1,0.),101.5,11.5));ASSERT_TRUE(f.validation(proof(ver(),2,101.5,104)));
 const auto commit=f.exchangeCommit(101.5,11.5);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed")<<commit->reason;
 auto ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,101.5,11.5));
 auto late=sendFact(101.26,1);late.sdk_nonzero=late.command_selected=true;
 late.command_sequence=previous.sequence;late.command_source_stamp=previous.source_stamp;
 late.command_valid_until=previous.valid_until;late.velocity=previous.velocity;
 ASSERT_TRUE(f.commandFeedback(late,101.5));
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,0.);
 ASSERT_TRUE(f.lease(lease(101.5,28),101.5));f.geometryPermit(101.5);
 EXPECT_LE(f.step(101.5,11.5).velocity.linear.x,c.max_jerk*.05*.05+1e-8);
}
TEST_F(LocalFollowerInterruptedReplacement, RejectedFinalAckAndCancelCannotRestoreOldMovingHistory) {
 ASSERT_TRUE(f.body(body(101.35,.1,0.),101.35,11.35));reserve(101.35);
 const auto commit=f.exchangeCommit(101.35,11.35);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed")<<commit->reason;
 auto rejected=*commit;rejected.accepted=false;rejected.disposition="rejected";
 rejected.reason="native_candidate_geometry_changed";
 ASSERT_TRUE(f.commitResult(rejected,101.35,11.35));
 EXPECT_EQ(f.controller().trajectoryId(),1); // Old negative curve stays stopped.
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,0.);
 ASSERT_TRUE(f.lease(lease(101.35,27),101.35));f.geometryPermit(101.35);
 EXPECT_TRUE(f.step(101.35,11.35).hold);
 EXPECT_DOUBLE_EQ(f.controller().commandHistory().output.forward,0.);
 auto late=*commit;late.accepted=true;
 EXPECT_FALSE(f.commitResult(late,101.36,11.36));
 f.retire("operator_cancel");
 EXPECT_FALSE(f.commitResult(late,101.37,11.37));
 auto sent=sendFact(101.26,1);sent.sdk_nonzero=sent.command_selected=true;
 sent.command_sequence=previous.sequence;sent.command_source_stamp=previous.source_stamp;
 sent.command_valid_until=previous.valid_until;sent.velocity=previous.velocity;
 EXPECT_FALSE(f.commandFeedback(sent,101.37));
 EXPECT_TRUE(f.step(101.4,11.4).hold);EXPECT_TRUE(f.status(101.4).software_retired);
}
TEST(LocalFollowerWriterFeedback, DelayedZeroCannotRewindAnAlreadyResumedFreshProofBoundary) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower a(c,"isolated_mock",{},false),b(c,"isolated_mock",{},false);stageExchange(a);stageExchange(b);
 wire::MotionDemand interrupted;
 for(unsigned i=1;i<=25;++i) {const double now=100.+i*.05;
  for(auto* f:{&a,&b}) {ASSERT_TRUE(f->body(body(now,.25*i*.05,.3),now,now-90.));
   ASSERT_TRUE(f->lease(lease(now,i),now));ASSERT_TRUE(f->validation(proof(ver(),1,now,i+1)));f->geometryPermit(now);}
  if(i==20) {
   auto late=sendFact(100.79,40,40);late.discontinuity_stamp=late.source_stamp;
   late.interrupted_command_sequence=interrupted.sequence;late.interrupted_command_source_stamp=interrupted.source_stamp;
   ASSERT_TRUE(b.commandFeedback(late,now));
  }
  const auto x=a.step(now,now-90.),y=b.step(now,now-90.);
  EXPECT_EQ(x.velocity,y.velocity);if(i==15)interrupted=y;
 }
}
TEST(LocalFollowerWriterFeedback, FreshCallbackAloneDoesNotHideAnUnreconciledSendingGap) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);wire::MotionDemand last;
 for(unsigned i=1;i<=19;++i) {const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.15,i<16?.2:0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  if(i<=15){ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));}
  // Explicit completed negative state keeps this a real interruption while
  // the zero notification is delayed; absence of newer positives alone does not.
  if(i==16){auto negative=proof(ver(),1,now,25);negative.valid=false;negative.reason="occupied";
    ASSERT_TRUE(f.validation(negative));}
  f.geometryPermit(now);const auto d=f.step(now,now-90.);if(i==15)last=d;
 }
 ASSERT_TRUE(f.body(body(101.,.15,0.),101.,11.));ASSERT_TRUE(f.lease(lease(101.,20),101.));
 ASSERT_TRUE(f.validation(proof(ver(),1,101.,30)));f.geometryPermit(101.);
 auto late=sendFact(100.79,40,40);late.discontinuity_stamp=late.source_stamp;
 late.interrupted_command_sequence=last.sequence;late.interrupted_command_source_stamp=last.source_stamp;
 ASSERT_TRUE(f.commandFeedback(late,101.));
 EXPECT_LE(f.step(101.,11.).velocity.linear.x,c.max_jerk*.05*.05+1e-8);
}
TEST(LocalFollowerWriterFeedback, DelayedPositiveNotificationWithActualSendingPreservesEveryHealthyTick) {
 for(const bool one_tick_delivery_delay:{false,true}) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower a(c,"isolated_mock",{},false),b(c,"isolated_mock",{},false);stageExchange(a);stageExchange(b);
 std::optional<wire::TaskCommandFeedback> queued;
 for(unsigned i=1;i<=35;++i) {const double now=100.+i*.05;
  for(auto* f:{&a,&b}) {ASSERT_TRUE(f->body(body(now,.15,0.),now,now-90.));
   ASSERT_TRUE(f->lease(lease(now,i),now));}
  ASSERT_TRUE(a.validation(proof(ver(),1,now,i+1)));
  if(i<=15||i>=31){ASSERT_TRUE(b.validation(proof(ver(),1,now,i+1)));}
  if(queued){ASSERT_TRUE(b.commandFeedback(*queued,now));queued.reset();}
  a.geometryPermit(now);b.geometryPermit(now);
  const auto x=a.step(now,now-90.),y=b.step(now,now-90.);EXPECT_EQ(x.velocity,y.velocity);
  // The real sender fact closes only a command-history boundary; Native's
  // fresh proof lives at the sender although its positive notification is late.
  auto actual=sendFact(now+.001,i);actual.command_selected=actual.sdk_nonzero=true;
  actual.command_sequence=y.sequence;actual.command_source_stamp=y.source_stamp;actual.velocity=y.velocity;
  actual.command_valid_until=y.valid_until;
  if(one_tick_delivery_delay)queued=actual;else ASSERT_TRUE(b.commandFeedback(actual,now+.001));
 }
 }
}
TEST(LocalFollowerWriterFeedback, NewerActualZeroClearsSubmittedHistoryBeforeNextControlTick) {
 for(const bool new_gap:{false,true}) {
  auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
  LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);wire::MotionDemand last;
  for(unsigned i=1;i<=16;++i) {const double now=100.+i*.05;
   ASSERT_TRUE(f.body(body(now,.15,.3),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
   if(i<=15){ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));}
   f.geometryPermit(now);last=f.step(now,now-90.);
  }
  auto actual=sendFact(100.801,30);actual.command_selected=actual.sdk_nonzero=true;
  auto gap=sendFact(100.8005,29,29);gap.discontinuity_stamp=gap.source_stamp;
  gap.interrupted_command_sequence=last.sequence;gap.interrupted_command_source_stamp=last.source_stamp;
  ASSERT_TRUE(f.commandFeedback(gap,100.8005));
  actual.discontinuity_write_sequence=29;actual.discontinuity_stamp=gap.source_stamp;
  actual.interrupted_command_sequence=last.sequence;actual.interrupted_command_source_stamp=last.source_stamp;
  actual.command_sequence=last.sequence;actual.command_source_stamp=last.source_stamp;
  actual.command_valid_until=last.valid_until;actual.velocity=last.velocity;
  ASSERT_TRUE(f.commandFeedback(actual,100.801));
  auto zero=sendFact(100.802,31,new_gap?31:29);zero.discontinuity_stamp=new_gap?zero.source_stamp:gap.source_stamp;
  zero.interrupted_command_sequence=last.sequence;zero.interrupted_command_source_stamp=last.source_stamp;
  ASSERT_TRUE(f.commandFeedback(zero,100.802));
  ASSERT_TRUE(f.body(body(100.85,.15,0.),100.85,10.85));ASSERT_TRUE(f.lease(lease(100.85,17),100.85));
  ASSERT_TRUE(f.validation(proof(ver(),1,100.85,30)));f.geometryPermit(100.85);
  EXPECT_LE(f.step(100.85,10.85).velocity.linear.x,c.max_jerk*.05*.05+1e-8);
 }
}
TEST(LocalFollowerWriterFeedback, StillLiveSubmittedHistoryCanPrecedeMoreThanOneGeneratedSequence) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);wire::MotionDemand sent;
 for(unsigned i=1;i<=25;++i) {const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.15,0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  auto p=proof(ver(),1,now,i+1);
  ASSERT_TRUE(f.validation(p));f.geometryPermit(now);sent=f.step(now,now-90.);
 }
 ASSERT_GT(sent.velocity.linear.x,.15);
 // A second legal fresh demand was generated before the sending fact arrives.
 // The last actual sending fact is two sequences behind, but its ORIGINAL
 // body/command deadline 101.35 is still live at the 101.30 control step.
 ASSERT_TRUE(f.body(body(101.275,.15,0.),101.275,11.275));ASSERT_TRUE(f.lease(lease(101.275,26),101.275));
 f.geometryPermit(101.275);f.step(101.275,11.275);
 auto actual=sendFact(101.251,30);actual.command_selected=actual.sdk_nonzero=true;
 actual.command_sequence=sent.sequence;actual.command_source_stamp=sent.source_stamp;
 actual.command_valid_until=sent.valid_until;actual.velocity=sent.velocity;
 ASSERT_TRUE(f.commandFeedback(actual,101.29));
 ASSERT_TRUE(f.body(body(101.3,.15,0.),101.3,11.3));ASSERT_TRUE(f.lease(lease(101.3,27),101.3));
 f.geometryPermit(101.3);EXPECT_GT(f.step(101.3,11.3).velocity.linear.x,.15);
}
TEST(LocalFollowerWriterFeedback, MissingNotificationsCannotInventAStopFromMeasuredRest) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 for(unsigned i=1;i<=35;++i) {const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.15,i<16?.2:0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  if(i<=15){ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));}
  f.geometryPermit(now);f.step(now,now-90.); // no negative proof or feedback reaches this follower
 }
 ASSERT_TRUE(f.body(body(101.8,.15,0.),101.8,11.8));ASSERT_TRUE(f.lease(lease(101.8,36),101.8));
 ASSERT_TRUE(f.validation(proof(ver(),1,101.8,40)));f.geometryPermit(101.8);
 EXPECT_GT(f.step(101.8,11.8).velocity.linear.x,.15);
 EXPECT_EQ(f.feedbackDiagnostic().rebases,0u);
}
TEST(LocalFollowerWriterFeedback, RepeatedActualZeroWithOneStickyGapCannotChargeUnsentHistory) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 wire::MotionDemand last,interrupted;
 for(unsigned i=1;i<=60;++i) {
  const double now=100.+i*.05;
  ASSERT_TRUE(f.body(body(now,.15,i<20?.2:0.),now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
  if(i>=20) {
   if(i==20)interrupted=last;
   auto zero=sendFact(now-.001,i,20);zero.discontinuity_stamp=stamp(100.999);
   zero.interrupted_command_sequence=interrupted.sequence;
   zero.interrupted_command_source_stamp=interrupted.source_stamp;
   ASSERT_TRUE(f.commandFeedback(zero,now));
  }
  f.geometryPermit(now);last=f.step(now,now-90.);
  if(i>=20) {
   EXPECT_GT(last.velocity.linear.x,0.); // Current intent is not gated on a sender ACK.
   EXPECT_LE(last.velocity.linear.x,c.max_jerk*.05*.05+1e-8);
   EXPECT_EQ(last.validation_sequence,1u);EXPECT_FALSE(f.status(now).holding);
  }
 }
 EXPECT_EQ(f.feedbackDiagnostic().gaps,1u);EXPECT_GE(f.feedbackDiagnostic().rebases,40u);
 // An actual submitted nonzero closes the history boundary. A late older
 // zero, despite retaining the same sticky gap, cannot rewind that send.
 auto sent=sendFact(103.001,61,20);sent.discontinuity_stamp=stamp(100.999);
 sent.interrupted_command_sequence=interrupted.sequence;sent.interrupted_command_source_stamp=interrupted.source_stamp;
 sent.command_selected=sent.sdk_nonzero=true;sent.command_sequence=last.sequence;
 sent.command_source_stamp=last.source_stamp;sent.command_valid_until=last.valid_until;sent.velocity=last.velocity;
 ASSERT_TRUE(f.commandFeedback(sent,103.001));
 auto late=sent;late.write_sequence=60;late.source_stamp=stamp(102.999);
 late.command_selected=late.sdk_nonzero=false;late.velocity=geometry_msgs::msg::Twist{};
 EXPECT_FALSE(f.commandFeedback(late,103.002));
 ASSERT_TRUE(f.body(body(103.05,.15,0.),103.05,13.05));ASSERT_TRUE(f.lease(lease(103.05,61),103.05));
 f.geometryPermit(103.05);EXPECT_GT(f.step(103.05,13.05).velocity.linear.x,last.velocity.linear.x);
}
TEST(LocalFollowerWriterFeedback, ExplicitAuthorityHoldFencesLatePreHoldHistory) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower a(c,"isolated_mock",{},false),b(c,"isolated_mock",{},false);stageExchange(a);stageExchange(b);
 wire::MotionDemand old;
 for(unsigned i=1;i<=23;++i) {const double now=100.+i*.05;
  for(auto* f:{&a,&b}) {ASSERT_TRUE(f->body(body(now,.15,i<21?.3:.1),now,now-90.));
   auto grant=lease(now,i);if(i==21)grant.allowed=false;
   ASSERT_TRUE(f->lease(grant,now));ASSERT_TRUE(f->validation(proof(ver(),1,now,i+1)));f->geometryPermit(now);}
  if(i==23) {auto late=sendFact(100.99,40,40);late.discontinuity_stamp=late.source_stamp;
   late.interrupted_command_sequence=old.sequence;late.interrupted_command_source_stamp=old.source_stamp;
   ASSERT_TRUE(b.commandFeedback(late,now));}
  const auto x=a.step(now,now-90.),y=b.step(now,now-90.);EXPECT_EQ(x.velocity,y.velocity);
  if(i==20)old=y;if(i==21)EXPECT_TRUE(y.hold);
  if(i==22)EXPECT_LE(y.velocity.linear.x,c.max_jerk*.05*.05+1e-8);
 }
}
TEST(LocalFollowerWriterFeedback, MeasuredRecoveryUsesCurrentBodyFrameAndCannotRestoreStaleOrHoldingMotion) {
 for(const double yaw:{0.,M_PI/2,M_PI})for(const double measured:{-.2,0.,.2}) {
  auto c=cfg();c.align_then_forward=true;LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
  auto& core=LocalFollowerTestAccess::core(f);core.reconcileSubmittedVelocity(.2,0.,0.);
  auto o=body(100.05,0.,measured);o.yaw=yaw;
  o.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(f.body(o,100.05,10.05));core.reconcileMeasuredContinuity(100.05);
  const double expected=std::min(std::max(0.,.2*std::cos(yaw)),std::max(0.,measured*std::cos(yaw)));
  EXPECT_NEAR(core.commandHistory().output.forward,expected,1e-12);
  EXPECT_DOUBLE_EQ(core.commandHistory().output.lateral,0.);
 }
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);auto& core=LocalFollowerTestAccess::core(f);
 core.reconcileSubmittedVelocity(.2,0.,0.);const auto old=core.commandHistory().output.forward;
 EXPECT_FALSE(core.reconcileMeasuredContinuity(101.));EXPECT_DOUBLE_EQ(core.commandHistory().output.forward,old);
 core.suspendOutput(100.05,10.05,"explicit_stop",true);
 EXPECT_FALSE(core.reconcileMeasuredContinuity(100.05));EXPECT_DOUBLE_EQ(core.commandHistory().output.forward,0.);
}
TEST(LocalFollowerWriterFeedback, MeasuredRecoveryOnlyResetsTheAxisWhoseBoundaryChanged) {
 for(const bool correct_yaw:{false,true}) {
  auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;c.max_yaw_jerk=4.;
  c.forward_response={.3,.31328213472822414,.5,.1};c.yaw_response={.5,.2,.3,.2};
  c.max_input_rate=1.6;c.max_input_acceleration=12.;
  LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
  auto& core=LocalFollowerTestAccess::core(f);auto history=core.commandHistory();
  // Establish both axes through the production conditioner, then install the
  // actual submitted history through the same production recovery method.
  for(int i=0;i<15;++i) {
   history.forward.step(.25,0.,.3,.35,.7,c.forward_response,1.6,12.,.05);
   if(i<3)history.yaw.step(.3,-.5,.5,.8,4.,c.yaw_response,1.6,12.,.05);
  }
  history.output.forward=history.forward.state().command;
  history.output.yaw_rate=history.yaw.state().command;
  history.planar_acceleration={history.forward.state().acceleration,0.};
  history.yaw_acceleration=history.yaw.state().acceleration;
  core.reconcileSubmittedCommand(history);
  auto o=body(100.05,0.,correct_yaw?.5:0.);
  o.angular_velocity_in_frame.z()=correct_yaw?0.:.5;
  ASSERT_TRUE(f.body(o,100.05,10.05));ASSERT_TRUE(core.reconcileMeasuredContinuity(100.05));
  const auto actual=core.commandHistory();
  const auto& before=correct_yaw?history.forward.state():history.yaw.state();
  const auto& after=correct_yaw?actual.forward.state():actual.yaw.state();
  EXPECT_DOUBLE_EQ(after.command,before.command);
  EXPECT_DOUBLE_EQ(after.acceleration,before.acceleration);
  EXPECT_DOUBLE_EQ(after.mapped_rate,before.mapped_rate);
  EXPECT_DOUBLE_EQ(correct_yaw?actual.output.yaw_rate:actual.output.forward,0.);
 }
}
TEST(LocalFollowerWriterFeedback, YawRebaseAfterShortGapDoesNotRestartForwardAcceleration) {
 for(const double frame_change:{0.,.0025}) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;c.max_yaw_jerk=4.;
 c.kp_yaw=.9;c.yaw_damping_gain=.4;c.steering_error_scale_rad=.02;
 c.lookahead=1.2;c.min_lookahead_distance_m=.4;c.max_lookahead_distance_m=.8;
 c.forward_response={.3,.31328213472822414,.5,.1};c.yaw_response={.5,.2,.3,.2};
 c.max_input_rate=1.6;c.max_input_acceleration=12.;
 LocalFollower normal(c,"isolated_mock",{},false),gap(c,"isolated_mock",{},false);
 stageExchange(normal);stageExchange(gap);wire::MotionDemand last;
 constexpr unsigned warm_ticks=12;
 const auto update=[&](LocalFollower& f,unsigned i) {
  const double now=100.+i*.05;auto o=body(now,.25*i*.05,.5);o.yaw=-.02;
  if(i>warm_ticks)o.yaw+=frame_change;
  o.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(f.body(o,now,now-90.));EXPECT_TRUE(f.lease(lease(now,i),now));
  EXPECT_TRUE(f.validation(proof(ver(),1,now,i+1)));f.geometryPermit(now);
 };
 for(unsigned i=1;i<=warm_ticks;++i) {
  update(normal,i);update(gap,i);
  const double now=100.+i*.05;
  const auto a=normal.step(now,now-90.);last=gap.step(now,now-90.);
  ASSERT_EQ(a.velocity,last.velocity);
 }
 const auto before=gap.controller().commandHistory();
 ASSERT_GT(before.forward.state().acceleration,.05)<<last.reason<<" vx="<<last.velocity.linear.x
   <<" yaw="<<last.velocity.angular.z<<" core="<<gap.controller().reason();
 ASSERT_GT(last.velocity.angular.z,0.);
 const double last_time=100.+warm_ticks*.05,next_time=100.+(warm_ticks+1)*.05;
 auto zero=sendFact(last_time+.031,166,166);zero.discontinuity_stamp=zero.source_stamp;
 zero.interrupted_command_sequence=last.sequence;zero.interrupted_command_source_stamp=last.source_stamp;
 ASSERT_TRUE(gap.commandFeedback(zero,last_time+.031));update(normal,warm_ticks+1);update(gap,warm_ticks+1);
 const auto a=normal.step(next_time,next_time-90.),b=gap.step(next_time,next_time-90.);
 ASSERT_FALSE(a.hold)<<a.reason;ASSERT_FALSE(b.hold)<<b.reason;
 const double projection=std::cos(frame_change);
 const double recovered_acceleration=(b.velocity.linear.x-last.velocity.linear.x*projection)/.05;
 RecordProperty("before_forward",std::to_string(last.velocity.linear.x));
 RecordProperty("before_acceleration",std::to_string(before.forward.state().acceleration));
 RecordProperty("normal_next_forward",std::to_string(a.velocity.linear.x));
 RecordProperty("recovered_next_forward",std::to_string(b.velocity.linear.x));
 RecordProperty("recovery_forward_jerk",std::to_string((recovered_acceleration-before.forward.state().acceleration)/.05));
 if(frame_change==0.)EXPECT_DOUBLE_EQ(a.velocity.linear.x,b.velocity.linear.x);
 else EXPECT_NEAR(a.velocity.linear.x,b.velocity.linear.x,
   (last.velocity.linear.x+c.max_acceleration*.05)*(1.-projection)+1e-8);
 EXPECT_LE(std::abs(recovered_acceleration-before.forward.state().acceleration*projection),c.max_jerk*.05+1e-8);
 EXPECT_DOUBLE_EQ(b.velocity.linear.y,0.);
 }
}
TEST(LocalFollowerWriterFeedback, CoordinateTransportRequiresFullySupportedForwardMotion) {
 for(const double yaw:{.0025,.4,M_PI})for(const double measured:{-.5,0.,.5}) {
  SCOPED_TRACE(::testing::Message()<<"yaw="<<yaw<<" measured="<<measured);
  auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
  c.forward_response={.3,.31328213472822414,.5,.1};c.yaw_response={.5,.2,.3,.2};c.max_yaw_jerk=4.;
  LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
  auto& core=LocalFollowerTestAccess::core(f);core.reconcileSubmittedVelocity(0.,0.,0.);
  auto h=core.commandHistory();
  for(int i=0;i<15;++i)h.forward.step(.25,0.,.3,.35,.7,c.forward_response,1.6,12.,.05);
  h.output.forward=h.forward.state().command;h.planar_acceleration={h.forward.state().acceleration,0.};
  core.reconcileSubmittedCommand(h);
  auto o=body(100.05,0.,measured);o.yaw=yaw;
  o.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(f.body(o,100.05,10.05));ASSERT_TRUE(core.reconcileMeasuredContinuity(100.05));
  const auto after=core.commandHistory();const double factor=std::max(0.,std::cos(yaw));
  if(measured>0.&&factor>0.) {
   EXPECT_DOUBLE_EQ(after.output.forward,h.output.forward*factor);
   EXPECT_DOUBLE_EQ(after.forward.state().acceleration,h.forward.state().acceleration*factor);
   const double old=h.forward.state().command-h.forward.state().acceleration*.05;
   EXPECT_NEAR(after.forward.state().mapped_rate,
     (c.forward_response.map(h.output.forward*factor)-c.forward_response.map(old*factor))/.05,1e-12);
  } else {
   EXPECT_DOUBLE_EQ(after.output.forward,0.);EXPECT_DOUBLE_EQ(after.forward.state().acceleration,0.);
   EXPECT_DOUBLE_EQ(after.forward.state().mapped_rate,0.);
  }
  EXPECT_DOUBLE_EQ(after.output.lateral,0.);
 }
}
TEST(LocalFollowerWriterFeedback, HealthyActualCurveReplacementIsTransparentAndNewTaskRejectsOldWriter) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.7;
 LocalFollower a(c,"isolated_mock",{},false),b(c,"isolated_mock",{},false);stageExchange(a);stageExchange(b);
 auto active=ver();unsigned id=1;
 for(unsigned i=1;i<=20;++i){const double now=100.+i*.05;
  for(auto* f:{&a,&b}){ASSERT_TRUE(f->body(body(now,.25*i*.05),now,now-90.));ASSERT_TRUE(f->lease(lease(now,i),now));}
  if(i==11){auto next=ver(2);next.anchor_id="new-anchor";next.anchor_revision=2;
   auto p=proposal(next,now);p.expected_version=active;p.expected_trajectory_id=id;
   for(auto* f:{&a,&b}){ASSERT_TRUE(f->proposal(p,now));ASSERT_TRUE(f->support(support(next)));
    ASSERT_TRUE(f->candidate(tag(next,2,.25*i*.05,now)));ASSERT_TRUE(f->validation(proof(next,2,now,i+1)));
    ASSERT_TRUE(f->tryCommit(now,now-90.))<<f->candidateReason();}
   active=next;id=2;
  }else for(auto* f:{&a,&b})ASSERT_TRUE(f->validation(proof(active,id,now,i+1)));
  auto fact=sendFact(now,i);fact.command_selected=fact.sdk_nonzero=true;
  ASSERT_TRUE(b.commandFeedback(fact,now)); // SDK binding retains original anchor.
  for(auto* f:{&a,&b})f->geometryPermit(now);
  EXPECT_EQ(a.step(now,now-90.),b.step(now,now-90.));
 }
 EXPECT_EQ(a.controller().trajectoryId(),2);EXPECT_EQ(b.controller().trajectoryId(),2);
 b.retire("new_goal");auto next=ver(3);next.task_id="new-task";
 ASSERT_TRUE(b.proposal(proposal(next,101.05),101.05));auto grant=lease(101.05,21);
 grant.version=next;grant.execution_id="new-execution";grant.control_epoch=2;
 ASSERT_TRUE(b.lease(grant,101.05));EXPECT_FALSE(b.commandFeedback(sendFact(101.05,100,50),101.05));
 EXPECT_TRUE(b.step(101.05,11.05).hold); // New task has no admitted geometry.
}
TEST(LocalFollowerNoticeDelivery, PersistentPreviewDoesNotInventAuthorityOrCommitAcknowledgement) {
 for(const bool pending_ack:{false,true}) {
  SCOPED_TRACE(pending_ack);LocalFollower f(cfg(),"isolated_mock",{},false);
  ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
  ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));ASSERT_TRUE(f.validation(proof()));
  const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.,10.));
  const auto commit=f.exchangeCommit(100.,10.);ASSERT_TRUE(commit);
  if(pending_ack){ASSERT_TRUE(f.lease(lease(),100.));}
  else {ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,100.,10.));}
  ASSERT_TRUE(f.body(body(100.21,.0525),100.21,10.21));f.geometryPermit(100.21);
  EXPECT_TRUE(f.step(100.21,10.21).hold);EXPECT_EQ(f.status(100.21).ready,!pending_ack);
  EXPECT_FALSE(f.status(100.21).request_stop);
 }
}
TEST(LocalFollowerNoticeDelivery, NegativeContractCoverageAndFutureTimeStillStopOldPositiveDemand) {
 for(unsigned fault=0;fault<7;++fault) {
  SCOPED_TRACE(fault);LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
  auto v=proof(ver(),1,100.01,2);
  if(fault==0){v.valid=false;v.reason="occupied";}
  if(fault==1){v.remaining_curve=true;v.whole_curve=false;v.checked_from_time=2.;v.valid_start_time=2.;v.valid_start_arc_length=.5;}
  if(fault==2)v.proposal_id="another_proposal";
  if(fault==3)v.frame_id="map";
  if(fault==4)v.check_end=v.source_stamp=stamp(101.);
  if(fault==5)v.valid_until=stamp(101.); // Originally malformed lease is not an expired notification.
  if(fault==6)v.valid_until=stamp(100.005); // Expired before its own query finished.
  ASSERT_TRUE(f.validation(v));ASSERT_TRUE(f.body(body(100.25,.0625),100.25,10.25));
  f.geometryPermit(100.25);const auto demand=f.step(100.25,10.25);
  EXPECT_TRUE(demand.hold);EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);
  EXPECT_FALSE(f.status(100.25).ready);
 }
}
TEST(LocalFollowerNoticeDelivery, MissingBodyAuthorityAndCancellationStillStopDuringNoticeDelay) {
 for(unsigned fault=0;fault<5;++fault) {
  SCOPED_TRACE(fault);LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
  const double now=fault==1?100.36:100.25;
  if(fault!=0){ASSERT_TRUE(f.body(body(now,.25*(now-100.)),now,now-90.));}
  if(fault==2){auto p=lease(now,2);p.allowed=false;ASSERT_TRUE(f.lease(p,now));}
  if(fault==3)f.retire("new_goal");
  if(fault==4)f.controller().hold("real_source_fault",now-90.);
  f.geometryPermit(now);const auto demand=f.step(now,now-90.);
  EXPECT_TRUE(demand.hold);EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);
  EXPECT_TRUE(f.status(now).holding);EXPECT_FALSE(f.status(now).at_goal);
  EXPECT_TRUE(trackingProgress(f.controller(),cfg(),now,f.status(now)).holding);
  if(fault==4){EXPECT_NE(f.status(now).reason,"waiting_native_validation");}
 }
}
TEST(LocalFollowerNoticeDelivery, CompletedTerminalYawStateDoesNotExpireWithNotificationAge) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
 p.has_goal_yaw=true;p.goal_yaw=.3;p.goal_yaw_tolerance_rad=.05;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));auto v=proof(ver(),1,100.01,2);v.goal_yaw_checked=true;v.checked_goal_yaw=.3;
 ASSERT_TRUE(f.validation(v));
 for(unsigned i=1;i<=4;++i){const double now=100.+.05*i;
   ASSERT_TRUE(f.body(body(now,.05,0.),now,now-90.));f.geometryPermit(now);f.step(now,now-90.);}
 ASSERT_TRUE(f.body(body(100.25,.05,0.),100.25,10.25));
 f.geometryPermit(100.25);const auto demand=f.step(100.25,10.25);
 EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.angular.z,0.);
 EXPECT_EQ(demand.reason,"aligning");
 EXPECT_EQ(f.status(100.25).phase,"aligning");EXPECT_FALSE(f.status(100.25).at_goal);
 EXPECT_FALSE(trackingProgress(f.controller(),cfg(),100.25,f.status(100.25)).holding);
 EXPECT_FALSE(f.status(100.25).request_stop);
}
// Exercise the production two-phase exchange using the real follower/core.
// Curve 1's original staging proof has expired, while its independently
// renewed active proof is fresh when curve 2 receives a rejected commit ACK.
class LocalFollowerRollback : public ::testing::Test {
 protected:
  LocalFollower follower{cfg(),"isolated_mock","",false};
  wire::LocalTrajectoryCommit committed;
  void SetUp() override {
    stageExchange(follower);ASSERT_FALSE(HasFatalFailure());
    ASSERT_TRUE(follower.lease(lease(),100.));follower.geometryPermit(100.);follower.step(100.,10.);
    ASSERT_TRUE(follower.body(body(100.05,.0125),100.05,10.05));
    follower.geometryPermit(100.05);ASSERT_FALSE(follower.step(100.05,10.05).hold);
    ASSERT_TRUE(follower.validation(proof(ver(),1,100.18,10)));
    ASSERT_TRUE(follower.body(body(100.18,.045),100.18,10.18));
    follower.geometryPermit(100.18);ASSERT_FALSE(follower.step(100.18,10.18).hold);
    ASSERT_TRUE(follower.body(body(100.21,.0525),100.21,10.21));
    ASSERT_TRUE(follower.candidate(tag(ver(),2,.0525,100.21)));
    ASSERT_TRUE(follower.validation(proof(ver(),2,100.21,20)));
    const auto request=follower.exchangeCommit(100.21,10.21);ASSERT_TRUE(request);
    ASSERT_EQ(request->disposition,"prepare");
    auto ack=*request;ack.accepted=true;ack.disposition="prepared";
    ASSERT_TRUE(follower.commitResult(ack,100.21,10.21));
    const auto result=follower.exchangeCommit(100.21,10.21);ASSERT_TRUE(result)<<follower.candidateReason();
    committed=*result;ASSERT_EQ(committed.disposition,"committed");
    ASSERT_EQ(follower.controller().trajectoryId(),1); // final ACK not received
    ASSERT_EQ(follower.commitSequence(),1u); // version and commit identity switch together
  }
  wire::LocalTrajectoryCommit rejected() const {
    auto ack=committed;ack.accepted=false;ack.disposition="rejected";
    ack.reason="native_measured_join_rejected";return ack;
  }
  void rollback() {
    ASSERT_TRUE(follower.body(body(100.22,.055),100.22,10.22));
    ASSERT_TRUE(follower.commitResult(rejected(),100.22,10.22));
    ASSERT_EQ(follower.controller().trajectoryId(),1);
  }
  void expectFresh(std::uint64_t sequence,double deadline) {
    (void)deadline; // Historical fixture time is no longer a motion permission.
    const auto diagnostic=follower.validationDiagnostic(100.22);
    EXPECT_EQ(diagnostic.sequence,sequence);EXPECT_TRUE(diagnostic.failure.empty());
    EXPECT_TRUE(diagnostic.persistent_state);EXPECT_TRUE(std::isnan(diagnostic.remaining_s));
    EXPECT_TRUE(follower.status(100.22).ready);
    follower.geometryPermit(100.22);const auto demand=follower.step(100.22,10.22);
    EXPECT_EQ(demand.validation_sequence,sequence);EXPECT_FALSE(demand.hold);
    EXPECT_GT(demand.velocity.linear.x,0.);
  }
};
TEST_F(LocalFollowerRollback, OldStagingStateCannotReplaceNewerIncumbentAfterRejectedCommit) {
  rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(10,100.38);
  ASSERT_TRUE(follower.body(body(100.381,.09525),100.381,10.381));
  ASSERT_TRUE(follower.lease(lease(100.381,2),100.381));follower.geometryPermit(100.381);
  const auto demand=follower.step(100.381,10.381);
  // The controller computes a new demand, not a fresh collision certificate.
  // Native alone decides whether its latest actual evidence can send it.
  EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
  EXPECT_EQ(demand.validation_sequence,10u);
  EXPECT_EQ(follower.status(100.381).phase,"following");
  EXPECT_TRUE(follower.validationDiagnostic(100.381).failure.empty());
}
TEST_F(LocalFollowerRollback, OlderDelayedNegativeCannotOverrideRenewedIncumbent) {
  auto old=proof(ver(),1,100.01,2);old.valid=false;old.reason="occupied";
  ASSERT_FALSE(follower.validation(old));rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(10,100.38);
}
TEST_F(LocalFollowerRollback, NewerNegativeDuringCommitSupersedesIncumbentAndStops) {
  auto negative=proof(ver(),1,100.215,11);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(follower.validation(negative));rollback();ASSERT_FALSE(HasFatalFailure());
  const auto diagnostic=follower.validationDiagnostic(100.22);
  EXPECT_EQ(diagnostic.sequence,11u);EXPECT_EQ(diagnostic.failure,"native_negative");
  EXPECT_EQ(diagnostic.native_reason,"occupied");EXPECT_FALSE(follower.status(100.22).ready);
  follower.geometryPermit(100.22);const auto demand=follower.step(100.22,10.22);
  EXPECT_EQ(demand.validation_sequence,11u);EXPECT_TRUE(demand.hold);
  EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);EXPECT_EQ(demand.reason,"local_collision_state_unavailable");
}
TEST_F(LocalFollowerRollback, NewerPositiveDuringCommitSupersedesIncumbentWithOriginalLease) {
  ASSERT_TRUE(follower.validation(proof(ver(),1,100.215,11)));
  rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(11,100.415);
}
TEST_F(LocalFollowerRollback, EqualSequenceCannotReplaceIncumbentPayloadOrRenewItsLease) {
  auto duplicate=proof(ver(),1,100.215,10);duplicate.valid=false;duplicate.reason="conflicting_duplicate";
  ASSERT_FALSE(follower.validation(duplicate));rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(10,100.38);
  // Once active again, the ordinary sequence check must also reject this tie.
  EXPECT_FALSE(follower.validation(duplicate));
}
TEST_F(LocalFollowerRollback, EvictedStagingProofRetainsRenewedIncumbent) {
  for(std::int64_t id=100;id<109;++id)ASSERT_TRUE(follower.validation(proof(ver(),id,100.215,30)));
  rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(10,100.38);
}
TEST_F(LocalFollowerRollback, NewerIncumbentNegativeSurvivesUnrelatedStagingCacheEviction) {
  auto negative=proof(ver(),1,100.215,11);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(follower.validation(negative));
  for(std::int64_t id=100;id<109;++id)ASSERT_TRUE(follower.validation(proof(ver(),id,100.216,30)));
  rollback();ASSERT_FALSE(HasFatalFailure());
  EXPECT_EQ(follower.validationDiagnostic(100.22).sequence,11u);
  EXPECT_EQ(follower.validationDiagnostic(100.22).failure,"native_negative");
  follower.geometryPermit(100.22);const auto demand=follower.step(100.22,10.22);
  EXPECT_TRUE(demand.hold);EXPECT_EQ(demand.validation_sequence,11u);
}
TEST_F(LocalFollowerRollback, NewerIncumbentPositiveSurvivesUnrelatedStagingCacheEviction) {
  ASSERT_TRUE(follower.validation(proof(ver(),1,100.215,11)));
  for(std::int64_t id=100;id<109;++id)ASSERT_TRUE(follower.validation(proof(ver(),id,100.216,30)));
  rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(11,100.415);
}
TEST_F(LocalFollowerRollback, SuccessfulCommitDoesNotApplyIncumbentVerdictToReplacement) {
  auto negative=proof(ver(),1,100.215,11);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(follower.validation(negative));
  auto accepted=committed;accepted.accepted=true;
  ASSERT_TRUE(follower.commitResult(accepted,100.22,10.22));
  EXPECT_FALSE(follower.commitResult(rejected(),100.22,10.22));
  EXPECT_EQ(follower.controller().trajectoryId(),2);
  EXPECT_EQ(follower.validationDiagnostic(100.22).sequence,20u);
  EXPECT_TRUE(follower.validationDiagnostic(100.22).failure.empty());
  EXPECT_TRUE(follower.status(100.22).ready);
}
TEST_F(LocalFollowerRollback, NewerCandidateNegativeBeforeFinalAckCannotBeReplacedByStagedPositive) {
  auto negative=proof(ver(),2,100.215,21);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(follower.validation(negative));
  // Bounded staging-cache eviction cannot remove the reserved verdict.
  for(std::int64_t id=100;id<109;++id)ASSERT_TRUE(follower.validation(proof(ver(),id,100.216,30)));
  auto accepted=committed;accepted.accepted=true;
  ASSERT_TRUE(follower.commitResult(accepted,100.22,10.22));
  EXPECT_EQ(follower.controller().trajectoryId(),2);
  EXPECT_EQ(follower.validationDiagnostic(100.22).sequence,21u);
  EXPECT_EQ(follower.validationDiagnostic(100.22).failure,"native_negative");
  follower.geometryPermit(100.22);const auto demand=follower.step(100.22,10.22);
  EXPECT_TRUE(demand.hold);EXPECT_EQ(demand.reason,"local_collision_state_unavailable");
}
TEST_F(LocalFollowerRollback, DifferentAnchorProofCannotReplaceIncumbent) {
  auto other=ver();other.reference_generation=2;other.anchor_revision=2;other.anchor_id="other_anchor";
  ASSERT_TRUE(follower.validation(proof(other,1,100.215,99)));
  rollback();ASSERT_FALSE(HasFatalFailure());expectFresh(10,100.38);
}
TEST_F(LocalFollowerRollback, RetiredTaskCannotBeRevivedByLateCommitOrRollbackAcknowledgment) {
  follower.retire("operator_cancel");
  EXPECT_FALSE(follower.commitResult(rejected(),100.22,10.22));
  auto accepted=committed;accepted.accepted=true;
  EXPECT_FALSE(follower.commitResult(accepted,100.22,10.22));
  // Retirement retains task/curve identity for status, but never its executable
  // geometry or authority. A late ACK must not restore either.
  EXPECT_FALSE(follower.controller().hasInstalledGeometry());
  EXPECT_FALSE(follower.geometryPermit(100.22).allowed);
  EXPECT_TRUE(follower.status(100.22).software_retired);
  follower.geometryPermit(100.22);EXPECT_TRUE(follower.step(100.22,10.22).hold);
}

// An unacknowledged candidate never replaces the incumbent. Its short endpoint
// must not affect a still-long incumbent while the final reply is in flight.
class LocalFollowerEndpointRollback : public ::testing::Test {
 protected:
  static Config config() {auto c=cfg();c.align_then_forward=true;c.max_jerk=1.4;return c;}
  Config c=config();LocalFollower f{c,"isolated_mock",{},false};
  wire::LocalTrajectoryCommit committed;
  double now=100.,x=.0125,v=.25,previous=.0,acceleration=0.;unsigned sequence=4;
  virtual bool shortIncumbent()const {return false;}
  wire::TrajectoryValidation incumbentProof(double time,unsigned serial)const {
    auto p=proof(ver(),1,time,serial);
    if(shortIncumbent())p.checked_to_time=p.curve_duration=.4;
    return p;
  }
  void SetUp() override {
    if(shortIncumbent()) {
      ASSERT_TRUE(f.body(body(),now,now-90.));ASSERT_TRUE(f.proposal(proposal(),now));
      ASSERT_TRUE(f.support(support()));auto original=tag();
      for(auto& point:original.trajectory.pos_pts)point.x*=.1;
      for(auto& knot:original.trajectory.knots)knot*=.1;
      ASSERT_TRUE(f.candidate(original));ASSERT_TRUE(f.validation(incumbentProof(now,1)));
      const auto request=f.exchangeCommit(now,now-90.);ASSERT_TRUE(request);
      auto ack=*request;ack.accepted=true;ack.disposition="prepared";
      ASSERT_TRUE(f.commitResult(ack,now,now-90.));const auto installed=f.exchangeCommit(now,now-90.);
      ASSERT_TRUE(installed);ack=*installed;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,now,now-90.));
    } else stageExchange(f);
    ASSERT_FALSE(HasFatalFailure());ASSERT_TRUE(f.lease(lease(),now));
    f.geometryPermit(now);f.step(now,now-90.);now+=.05;
    ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
    ASSERT_TRUE(f.validation(incumbentProof(now,2)));f.geometryPermit(now);
    previous=f.step(now,now-90.).velocity.linear.x;
    auto candidate=tag(ver(),2,x,now);
    for(std::size_t i=0;i<candidate.trajectory.pos_pts.size();++i)
      candidate.trajectory.pos_pts[i].x=x+(static_cast<double>(i)-1.)*.005;
    for(std::size_t i=0;i<candidate.trajectory.knots.size();++i)
      candidate.trajectory.knots[i]=(static_cast<double>(i)-3.)*.02;
    ASSERT_TRUE(f.candidate(candidate));auto validation=proof(ver(),2,now,3);
    validation.checked_to_time=validation.curve_duration=.4;ASSERT_TRUE(f.validation(validation));
    const auto request=f.exchangeCommit(now,now-90.);ASSERT_TRUE(request)<<f.candidateReason();
    auto ack=*request;ack.accepted=true;ack.disposition="prepared";
    ASSERT_TRUE(f.commitResult(ack,now,now-90.));
    const auto installed=f.exchangeCommit(now,now-90.);ASSERT_TRUE(installed)<<f.candidateReason();
    committed=*installed;ASSERT_EQ(f.controller().trajectoryId(),1);
    bool ended=false;
    for(unsigned i=1;i<=300;++i) {
      now=100.05+i*.05;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
      validation=proof(ver(),2,now,sequence++);validation.checked_to_time=validation.curve_duration=.4;
      ASSERT_TRUE(f.validation(validation));ASSERT_TRUE(f.validation(incumbentProof(now,sequence++)));
      ASSERT_TRUE(f.lease(lease(now,sequence++),now));f.geometryPermit(now);
      const auto demand=f.step(now,now-90.);v=demand.velocity.linear.x;
      acceleration=(v-previous)/.05;previous=v;x+=v*.05;
      if(demand.reason=="local_segment_finished_waiting_replan"){ended=true;break;}
      if(!shortIncumbent()&&i==10)break;
    }
    ASSERT_EQ(ended,shortIncumbent())<<f.controller().reason();ASSERT_LT(x,.2);
    ASSERT_FALSE(f.controller().holding());ASSERT_TRUE(f.status(now).ready);
  }
  void reject() {
    auto reply=committed;reply.accepted=false;reply.disposition="rejected";
    reply.reason="native_measured_join_rejected";
    ASSERT_TRUE(f.commitResult(reply,now,now-90.));ASSERT_EQ(f.controller().trajectoryId(),1);
  }
};
TEST_F(LocalFollowerEndpointRollback, RejectedShortEndpointCannotStrandLongIncumbentOrResetLimiter) {
  const auto old_body=f.controller().odometry();reject();ASSERT_FALSE(HasFatalFailure());
  EXPECT_DOUBLE_EQ(f.controller().odometry().stamp,old_body.stamp);
  EXPECT_EQ(f.controller().odometry().source_stamp_ns,old_body.source_stamp_ns);
  for(unsigned i=0;i<20;++i) {
    now+=.05;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
    ASSERT_TRUE(f.validation(proof(ver(),1,now,sequence++)));
    ASSERT_TRUE(f.lease(lease(now,sequence++),now));f.geometryPermit(now);
    const auto demand=f.step(now,now-90.);v=demand.velocity.linear.x;
    EXPECT_NE(demand.reason,"local_segment_finished_waiting_replan");EXPECT_GT(v,0.);
    const double next_acceleration=(v-previous)/.05;
    EXPECT_LE(std::abs(next_acceleration),c.max_acceleration+1e-9);
    EXPECT_LE(std::abs(next_acceleration-acceleration),c.max_jerk*.05+1e-8);
    acceleration=next_acceleration;previous=v;x+=v*.05;
    EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.status(now).version.task_id,"task");
    EXPECT_FALSE(f.status(now).request_stop);
  }
}
TEST_F(LocalFollowerEndpointRollback, NewerIncumbentCollisionStillStopsAfterEndpointRollback) {
  auto negative=proof(ver(),1,now,sequence++);negative.valid=false;negative.reason="occupied";
  ASSERT_TRUE(f.validation(negative));reject();ASSERT_FALSE(HasFatalFailure());
  now+=.05;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
  ASSERT_TRUE(f.lease(lease(now,sequence++),now));f.geometryPermit(now);
  const auto demand=f.step(now,now-90.);EXPECT_TRUE(demand.hold);
  EXPECT_EQ(demand.reason,"local_collision_state_unavailable");
  EXPECT_EQ(f.validationDiagnostic(now).native_reason,"occupied");
}
TEST_F(LocalFollowerEndpointRollback, CurrentSourceFaultAndRecoveryAreNotRolledBackWithEndpoint) {
  now+=.01;auto invalid=body(now,x,v);invalid.provider_usable=false;
  ASSERT_FALSE(f.body(invalid,now,now-90.));ASSERT_TRUE(f.controller().holding());
  reject();ASSERT_FALSE(HasFatalFailure());EXPECT_TRUE(f.controller().holding());
  EXPECT_FALSE(f.body(body(now-.001,x,v),now,now-90.));
  now+=.04;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
  ASSERT_TRUE(f.validation(proof(ver(),1,now,sequence++)));
  ASSERT_TRUE(f.lease(lease(now,sequence++),now));f.geometryPermit(now);
  EXPECT_TRUE(f.step(now,now-90.).hold);EXPECT_TRUE(f.controller().holding());
}
class LocalFollowerShortEndpointRollback : public LocalFollowerEndpointRollback {
 protected:
  bool shortIncumbent()const override {return true;}
};
TEST_F(LocalFollowerShortEndpointRollback, ActuallyExhaustedIncumbentStillStopsAfterFreshProjection) {
  reject();ASSERT_FALSE(HasFatalFailure());
  for(unsigned i=0;i<20;++i) {
    now+=.05;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
    ASSERT_TRUE(f.validation(incumbentProof(now,sequence++)));
    ASSERT_TRUE(f.lease(lease(now,sequence++),now));f.geometryPermit(now);
    const auto demand=f.step(now,now-90.);v=demand.velocity.linear.x;x+=v*.05;
    EXPECT_EQ(demand.reason,"local_segment_finished_waiting_replan");
    EXPECT_FALSE(f.status(now).at_goal);EXPECT_FALSE(f.status(now).request_stop);
  }
  EXPECT_DOUBLE_EQ(v,0.);EXPECT_EQ(f.controller().trajectoryId(),1);
}
TEST(LocalFollower, ParametricSupportIsIndependentOfLiteralQualification) {
 LocalFollower follower(cfg(),"isolated_mock","",false);
 ASSERT_TRUE(follower.proposal(proposal(),100.));
 auto s=modelSupport();ASSERT_TRUE(follower.support(s));
 EXPECT_FALSE(s.verified);EXPECT_TRUE(s.support_ground_xyz.empty());
 auto e=supportEvidence(s);EXPECT_FALSE(e.verified);ASSERT_TRUE(e.model_geometry_valid);
 SupportIndex index(e.ground);auto c=cfg();c.geometry_authority=GeometryAuthority::NativeScan;
 EXPECT_TRUE(supportEvidenceValid(e,index,identity(s.version),s.version.reference_generation,c));
 c.geometry_authority=GeometryAuthority::LegacyTracker;
 EXPECT_FALSE(supportEvidenceValid(e,index,identity(s.version),s.version.reference_generation,c));
 // Repeating a version cannot silently move its model, even with a stale hash.
 s.stair_model_surfaces[0].pose.position.x=.1;EXPECT_FALSE(follower.support(s));
}

TEST(LocalFollower, MalformedModelDoesNotReplaceAbsentLiteralSupport) {
 LocalFollower follower(cfg(),"isolated_mock","",false);
 ASSERT_TRUE(follower.proposal(proposal(),100.));
 auto s=modelSupport();s.stair_model_surfaces[0].pose.orientation.w=0.;
 EXPECT_FALSE(follower.support(s));
 s=modelSupport();s.stair_model_surfaces[0].polygon[0].x=10.;
 EXPECT_FALSE(follower.support(s));
 s=modelSupport();s.stair_model_hash="not_a_model_identity";EXPECT_FALSE(follower.support(s));
}

TEST(LocalFollower, ModelOnlyGeometryCommitsOnlyAfterNativeProofAndDoesNotAuthorizeMotion) {
 LocalFollower f(cfg(),"isolated_mock","",false);
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(modelSupport()));ASSERT_TRUE(f.candidate(tag()));
 EXPECT_FALSE(f.tryCommit(100.,10.));
 ASSERT_TRUE(f.validation(proof()));ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_TRUE(f.status(100.01).ready);
 EXPECT_FALSE(f.geometryPermit(100.01).allowed);
}

TEST(LocalFollowerClock, InputDeliveryReservesImmediatelyButCannotInstallBetweenControlTicks) {
 LocalFollower f(cfg(),"isolated_mock","",false);stageExchange(f);
 ASSERT_TRUE(f.body(body(100.04,.01),100.04,10.04));
 ASSERT_TRUE(f.candidate(tag(ver(),2,.01,100.04)));
 ASSERT_TRUE(f.validation(proof(ver(),2,100.04,2)));
 using Clock=LocalFollower::ExchangeClock;
 const auto request=f.exchangeCommit(100.04,10.04,stamp(100.04),Clock::InputDelivery);
 ASSERT_TRUE(request);ASSERT_EQ(request->disposition,"prepare");
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.041,10.041));
 for(const double time:{100.042,100.045,100.049}) {
  EXPECT_FALSE(f.exchangeCommit(time,time-90.,stamp(time),Clock::InputDelivery));
  EXPECT_EQ(f.controller().trajectoryId(),1);
 }
 ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));
 const auto commit=f.exchangeCommit(100.05,10.05,stamp(100.05),Clock::ControlTick);
 ASSERT_TRUE(commit)<<f.candidateReason();EXPECT_EQ(commit->trajectory_id,2);
 EXPECT_EQ(f.controller().trajectoryId(),1);
 ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,100.051,10.051));
 EXPECT_EQ(f.controller().trajectoryId(),2);
 EXPECT_EQ(commit->source_stamp,stamp(100.05));
 EXPECT_EQ(commit->body_source_stamp,stamp(100.05));
}
TEST(LocalFollowerClock, InputDeliveryStillAbortsNegativeProofAndHonorsCancellationImmediately) {
 using Clock=LocalFollower::ExchangeClock;
 for(const bool cancel:{false,true}) {
  LocalFollower f(cfg(),"isolated_mock","",false);stageExchange(f);
  ASSERT_TRUE(f.body(body(100.04,.01),100.04,10.04));
  ASSERT_TRUE(f.candidate(tag(ver(),2,.01,100.04)));
  ASSERT_TRUE(f.validation(proof(ver(),2,100.04,2)));
  const auto request=f.exchangeCommit(100.04,10.04,stamp(100.04),Clock::InputDelivery);
  ASSERT_TRUE(request);auto ack=*request;ack.accepted=true;ack.disposition="prepared";
  ASSERT_TRUE(f.commitResult(ack,100.041,10.041));
  if(cancel) {
   f.retire("operator_cancel");
   EXPECT_FALSE(f.exchangeCommit(100.042,10.042,stamp(100.042),Clock::InputDelivery));
   EXPECT_FALSE(f.exchangeCommit(100.05,10.05,stamp(100.05),Clock::ControlTick));
  } else {
   auto negative=proof(ver(),2,100.042,3);negative.valid=false;negative.reason="occupied";
   ASSERT_TRUE(f.validation(negative));
   const auto abort=f.exchangeCommit(100.043,10.043,stamp(100.043),Clock::InputDelivery);
   ASSERT_TRUE(abort);EXPECT_EQ(abort->disposition,"abort");
   EXPECT_EQ(f.controller().trajectoryId(),1);
  }
 }
}
// Literal adjacent floor returns from the 2026-10-15 runtime diagnosis. These
// are provenance, not a substitute production map or a synthetic SCAN verdict.
constexpr double recorded_x=-.020464,recorded_y=1.121268,recorded_body_z=-.002283;
wire::SupportReference recordedRoughSupport() {
 auto s=support();s.support_ground_xyz.clear();s.body_reference_height_m=.5;s.support_xy_radius_m=.2;
 geometry_msgs::msg::Point a;a.x=recorded_x;a.y=recorded_y;a.z=-.502283;
 geometry_msgs::msg::Point b;b.x=-.002781;b.y=1.126652;b.z=-.443233;
 s.support_ground_xyz={a,b};return s;
}
wire::TaggedBspline recordedCurve() {
 auto t=tag(ver(),1,recorded_x);
 for(auto& p:t.trajectory.pos_pts){p.y=recorded_y;p.z=recorded_body_z;}
 t.join_pose.position.y=recorded_y;t.join_pose.position.z=recorded_body_z;return t;
}
Odom recordedBody(double now=100.,double advance=0.,double vx=.25) {
 auto o=body(now,recorded_x+advance,vx);o.position.y()=recorded_y;
 o.position.z()=recorded_body_z;return o;
}
wire::ReferenceProposal recordedProposal() {
 auto p=proposal();p.goal_position.x=recorded_x+3.;p.goal_position.y=recorded_y;
 p.goal_position.z=recorded_body_z;return p;
}
void stageRecorded(LocalFollower& f,wire::ReferenceProposal p=recordedProposal()) {
 ASSERT_TRUE(f.body(recordedBody(),100.,10.));ASSERT_TRUE(f.proposal(p,100.));
 ASSERT_TRUE(f.support(recordedRoughSupport()));ASSERT_TRUE(f.candidate(recordedCurve()));
 ASSERT_TRUE(f.validation(proof()));ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();
}
}
TEST(LocalFollower, NativeAuthorityAcceptsWholeCurveWithoutRepeatingRecordedFloorRoughness) {
 auto c=cfg();c.align_then_forward=true;
 const auto source=recordedRoughSupport();const auto evidence=supportEvidence(source);
 const SupportIndex index(evidence.ground);
 ASSERT_NEAR((evidence.ground[1]-evidence.ground[0]).head<2>().norm(),.018484,1e-6);
 ASSERT_NEAR(evidence.ground[1].z()-evidence.ground[0].z(),.05905,1e-12);
 ASSERT_FALSE(pointSupported(evidence,index,recordedBody().position,c.goal_height_tolerance));
 // Same-input comparison: the old tracker-side PCD predicate rejects this
 // already Native-certified curve, including raw-return gaps farther ahead.
 const auto legacy=PreparedGeometry::build(c,decodeLocalCurve(recordedCurve()),evidence);
 ASSERT_EQ(legacy->failure,"curve_outside_support");
 LocalFollower f(c,"isolated_mock",{},false);stageRecorded(f);
 EXPECT_TRUE(f.status(100.01).ready);EXPECT_EQ(f.commitSequence(),1u);
 ASSERT_TRUE(f.activeVersion());
 EXPECT_EQ(f.activeVersion()->task_id,"task");EXPECT_EQ(f.controller().trajectoryId(),1);
 EXPECT_FALSE(f.geometryPermit(100.01).allowed); // Geometry alone is not a task execution lease.
}
TEST(LocalFollower, NativeAuthorityStillRequiresMatchingFreshPositiveWholeCurveEvidence) {
 for(unsigned bad_case=0;bad_case<4;++bad_case) {
  SCOPED_TRACE(bad_case);LocalFollower f(cfg(),"isolated_mock",{},false);
  ASSERT_TRUE(f.body(recordedBody(),100.,10.));ASSERT_TRUE(f.proposal(recordedProposal(),100.));
  ASSERT_TRUE(f.support(recordedRoughSupport()));ASSERT_TRUE(f.candidate(recordedCurve()));
  auto bad=proof();
  if(bad_case==0){bad.valid=false;bad.reason="occupied";}
  if(bad_case==1)bad.valid_until=stamp(99.99);
  if(bad_case==2)bad.support_hash=std::string(64,'d');
  if(bad_case==3)bad.version.task_id="foreign-task";
  if(bad_case==3)EXPECT_FALSE(f.validation(bad));else ASSERT_TRUE(f.validation(bad));
  EXPECT_FALSE(f.tryCommit(100.,10.));EXPECT_EQ(f.commitSequence(),0u);
  EXPECT_FALSE(f.controller().hasInstalledGeometry());EXPECT_FALSE(f.status(100.).ready);
  ASSERT_TRUE(f.validation(proof(ver(),1,100.,2)));
  ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();
 }
}
TEST(LocalFollower, NativeAuthorityKeepsActualJoinAndCancellationFences) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 ASSERT_TRUE(f.body(recordedBody(100.,.4),100.,10.));
 ASSERT_TRUE(f.proposal(recordedProposal(),100.));ASSERT_TRUE(f.support(recordedRoughSupport()));
 ASSERT_TRUE(f.candidate(recordedCurve()));ASSERT_TRUE(f.validation(proof()));
 EXPECT_FALSE(f.tryCommit(100.,10.));EXPECT_FALSE(f.controller().hasInstalledGeometry());
 EXPECT_EQ(f.commitSequence(),0u);
 f.retire("operator_cancel");EXPECT_TRUE(f.status(100.).software_retired);
 EXPECT_FALSE(f.proposal(recordedProposal(),100.));EXPECT_FALSE(f.candidate(recordedCurve()));
 EXPECT_FALSE(f.validation(proof(ver(),1,100.,2)));
 const auto stopped=f.step(100.01,10.01);EXPECT_TRUE(stopped.hold);
 EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);
}
TEST(LocalFollower, NativeAuthorityRunsAndRecoversWithoutASecondRawFloorGate) {
 auto c=cfg();c.align_then_forward=true;LocalFollower f(c,"isolated_mock",{},false);stageRecorded(f);
 double advance=0.,velocity=.25,previous=0.;
 for(unsigned i=1;i<=60;++i) {
  const double now=100.+i*.05;
  ASSERT_TRUE(f.body(recordedBody(now,advance,velocity),now,now-90.));
  ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));ASSERT_TRUE(f.lease(lease(now,i),now));
  ASSERT_TRUE(f.geometryPermit(now).allowed);const auto command=f.step(now,now-90.);
  EXPECT_FALSE(command.hold)<<command.reason;EXPECT_DOUBLE_EQ(command.velocity.linear.y,0.);
  EXPECT_GE(command.velocity.linear.x,0.);
  EXPECT_LE(std::abs(command.velocity.linear.x-previous),c.max_acceleration*.05+1e-8);
  previous=velocity=command.velocity.linear.x;advance+=velocity*.05;
 }
 ASSERT_GT(advance,.2); // Past the sparse recorded returns, not just admission at the first point.
 auto negative=proof(ver(),1,103.05,62);negative.valid=false;negative.reason="occupied";
 ASSERT_TRUE(f.body(recordedBody(103.05,advance,0.),103.05,13.05));
 ASSERT_TRUE(f.validation(negative));ASSERT_TRUE(f.lease(lease(103.05,61),103.05));
 f.geometryPermit(103.05);const auto stopped=f.step(103.05,13.05);
 EXPECT_TRUE(stopped.hold);EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);
 EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);
 EXPECT_FALSE(f.validation(proof())); // Superseded positive evidence cannot undo the stop.
 bool resumed=false;
 for(unsigned i=1;i<=20;++i) {
  const double now=103.05+i*.05;ASSERT_TRUE(f.body(recordedBody(now,advance,0.),now,now-90.));
  ASSERT_TRUE(f.validation(proof(ver(),1,now,62+i)));ASSERT_TRUE(f.lease(lease(now,61+i),now));
  f.geometryPermit(now);const auto command=f.step(now,now-90.);
  EXPECT_DOUBLE_EQ(command.velocity.linear.y,0.);resumed|=!command.hold&&command.velocity.linear.x>0.;
 }
 EXPECT_TRUE(resumed)<<f.controller().reason()<<" advance="<<advance;
 EXPECT_EQ(f.commitSequence(),1u);EXPECT_EQ(f.activeVersion()->task_id,"task");
 EXPECT_FALSE(f.status(104.05).software_retired);
}
TEST(LocalFollower, NativeAuthorityTerminalYawAndMeasuredRestUseNativeProofNotRawFloorNoise) {
 auto c=cfg();c.align_then_forward=true;LocalFollower f(c,"isolated_mock",{},false);
 auto p=recordedProposal();p.goal_position.x=recorded_x+.05;p.has_goal_yaw=true;
 p.goal_yaw=.2;p.goal_yaw_tolerance_rad=.07;stageRecorded(f,p);
 auto o=recordedBody(100.05,.05,0.);ASSERT_TRUE(f.body(o,100.05,10.05));
 auto valid=proof(ver(),1,100.05,2);valid.goal_yaw_checked=true;valid.checked_goal_yaw=.2;
 ASSERT_TRUE(f.validation(valid));ASSERT_TRUE(f.lease(lease(100.05,1),100.05));
 ASSERT_EQ(f.geometryPermit(100.05).phase,"aligning");const auto turning=f.step(100.05,10.05);
 EXPECT_FALSE(turning.hold);EXPECT_GT(turning.velocity.angular.z,0.);
 EXPECT_DOUBLE_EQ(turning.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(turning.velocity.linear.y,0.);
 EXPECT_FALSE(f.status(100.05).at_goal);
 o=recordedBody(100.1,.05,0.);o.yaw=.2;o.orientation=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ());
 ASSERT_TRUE(f.body(o,100.1,10.1));valid=proof(ver(),1,100.1,3);
 valid.goal_yaw_checked=true;valid.checked_goal_yaw=.2;ASSERT_TRUE(f.validation(valid));
 ASSERT_TRUE(f.lease(lease(100.1,2),100.1));f.geometryPermit(100.1);
 const auto stopped=f.step(100.1,10.1);EXPECT_TRUE(stopped.hold);
 EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);EXPECT_TRUE(f.status(100.1).at_goal);
 EXPECT_TRUE(f.status(100.1).request_stop);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, NewTaskPreparationReportsItsCurrentReasonNotPreviousRetirement) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);f.retire("route_retired");
 auto v=ver(3);v.task_id="next-task";v.route_id="next-route";
 ASSERT_TRUE(f.proposal(proposal(v,100.02),100.02));ASSERT_TRUE(f.support(support(v)));
 auto broken=tag(v,2,0.,100.02);broken.trajectory.knots[4]=broken.trajectory.knots[3];
 ASSERT_TRUE(f.candidate(broken));ASSERT_TRUE(f.validation(proof(v,2,100.02,2)));
 EXPECT_FALSE(f.tryCommit(100.02,10.02));ASSERT_FALSE(f.candidateReason().empty());
 const auto state=f.status(100.02);EXPECT_FALSE(state.software_retired);
 EXPECT_FALSE(state.ready);EXPECT_EQ(state.phase,"waiting_local_trajectory");
 EXPECT_EQ(state.reason,f.candidateReason());EXPECT_NE(state.reason,"route_retired");
 EXPECT_EQ(state.version.task_id,"next-task");
}
TEST(LocalFollower, PreviewCommitsGeometryWithoutSdkOrPerCurveOwner) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 EXPECT_FALSE(f.geometryPermit(100.01).allowed);EXPECT_TRUE(f.step(100.01,10.01).hold);
  EXPECT_TRUE(f.status(100.01).ready);
}
TEST(LocalFollower, ValidationWaitDiagnosticsDoNotRenewOrReplaceNativeEvidence) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 EXPECT_EQ(f.validationDiagnostic(100.).failure,"no_current_proof");stage(f);
 const auto commit=f.commitSequence();
 EXPECT_TRUE(f.validationDiagnostic(100.01).failure.empty());
 EXPECT_TRUE(f.validationDiagnostic(100.21).failure.empty());
 EXPECT_EQ(f.commitSequence(),commit);EXPECT_EQ(f.controller().trajectoryId(),1);
 auto negative=proof(ver(),1,100.02,2);negative.valid=false;negative.reason="occupied";
 ASSERT_TRUE(f.validation(negative));const auto blocked=f.validationDiagnostic(100.03);
 EXPECT_EQ(blocked.failure,"native_negative");EXPECT_EQ(blocked.native_reason,"occupied");
 EXPECT_EQ(blocked.sequence,2u);EXPECT_TRUE(std::isnan(blocked.remaining_s));
 auto uncovered=proof(ver(),1,100.04,3);uncovered.whole_curve=false;uncovered.remaining_curve=true;
 uncovered.checked_from_time=.1;uncovered.valid_start_time=.2;uncovered.valid_start_arc_length=.05;
 uncovered.reverse_margin_m=.15;ASSERT_TRUE(f.validation(uncovered));
 EXPECT_EQ(f.validationDiagnostic(100.05).failure,"remaining_curve_coverage");
 EXPECT_EQ(f.commitSequence(),commit);EXPECT_TRUE(f.controller().hasInstalledGeometry());
}
TEST(LocalFollower, ExchangeDistinguishesPreparedGeometryFromMissingNativeProof) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));
 EXPECT_FALSE(f.exchangeCommit(100.,10.));
 EXPECT_EQ(f.candidateReason(),"waiting_current_native_proof:missing");
 EXPECT_EQ(f.status(100.).reason,f.candidateReason());
 EXPECT_EQ(f.commitSequence(),0u);EXPECT_FALSE(f.controller().hasInstalledGeometry());
 ASSERT_TRUE(f.validation(proof()));const auto request=f.exchangeCommit(100.,10.);
 ASSERT_TRUE(request);EXPECT_EQ(request->disposition,"prepare");
 EXPECT_EQ(f.candidateReason(),"waiting_native_candidate_reservation");
}

TEST(LocalFollower, ReservedOldViewAndEmptyPollPreserveNewerProductionPreparation) {
 for(const bool reserved_old:{false,true}) {
  SCOPED_TRACE(reserved_old);
  LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
  ASSERT_TRUE(f.candidate(tag(ver(),2)));ASSERT_TRUE(f.validation(proof(ver(),2,100.,2)));
  const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
  auto prepared=*request;prepared.accepted=true;prepared.disposition="prepared";
  ASSERT_TRUE(f.commitResult(prepared,100.,10.));
  // Candidate2 is already reserved/prepared. Candidate3 uses the real bounded
  // worker and real PreparedGeometry builder while that reservation persists.
  LocalFollowerTestAccess::enableProductionWorker(f);
  ASSERT_TRUE(f.candidate(tag(ver(),3)));
  for(int i=0;i<1000&&!LocalFollowerTestAccess::completionReady(f);++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(LocalFollowerTestAccess::completionReady(f));
  if(reserved_old) {
    const auto commit=LocalFollowerTestAccess::nestedReservedCommit(f,100.,10.);
    ASSERT_TRUE(commit)<<f.candidateReason();EXPECT_EQ(commit->trajectory_id,2);
  } else LocalFollowerTestAccess::pollWithoutPending(f);
  EXPECT_TRUE(LocalFollowerTestAccess::completionReady(f));
  EXPECT_TRUE(LocalFollowerTestAccess::pollPrepared(f,3));
  EXPECT_FALSE(LocalFollowerTestAccess::completionReady(f));
  EXPECT_EQ(f.controller().trajectoryId(),reserved_old?2:1);
  f.retire("new_target");
  EXPECT_FALSE(LocalFollowerTestAccess::pollPrepared(f,3));
 }
}
TEST(LocalFollower, ExchangeRejectsLegacyTimedProofAndRequiresExplicitPersistentState) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));
 // Producer regression: two calls to an advancing clock create a lease
 // longer than its own check_end contract. This is not sensor grace time.
 std::int64_t clock_ns=100000000000LL;
 const auto advancing_clock=[&](){const auto value=clock_ns;clock_ns+=25000LL;return value;};
 auto bad=proof();const auto original_end=advancing_clock();
 bad.check_end=bad.source_stamp=stampNs(original_end);
 bad.valid_until=stampNs(advancing_clock()+250000000LL);
 ASSERT_TRUE(f.validation(bad));EXPECT_FALSE(f.exchangeCommit(100.,10.));
 EXPECT_EQ(f.candidateReason(),"waiting_current_native_proof:persistent_collision_state_required");
 EXPECT_EQ(f.commitSequence(),0u);EXPECT_EQ(timeNs(bad.valid_until)-timeNs(bad.check_end),250025000LL);
 // The new contract does not extend the lease: zero explicitly removes it.
 auto valid=bad;valid.sequence=2;valid.valid_until=stamp(0.);
 ASSERT_TRUE(f.validation(valid));const auto request=f.exchangeCommit(100.,10.);
 ASSERT_TRUE(request);EXPECT_EQ(request->disposition,"prepare");
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";ack.reason="native_prepared";
 ASSERT_TRUE(f.commitResult(ack,100.,10.));
 ASSERT_TRUE(f.body(body(100.01),100.01,10.01));
 const auto committed=f.exchangeCommit(100.01,10.01);
 ASSERT_TRUE(committed)<<f.candidateReason();EXPECT_EQ(committed->disposition,"committed");
 ack=*committed;ack.accepted=true;ack.reason="native_committed";
 ASSERT_TRUE(f.commitResult(ack,100.01,10.01));
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 EXPECT_TRUE(f.validationDiagnostic(100.251).failure.empty());
 EXPECT_EQ(timeNs(valid.valid_until),0);EXPECT_EQ(timeNs(valid.check_end),original_end);
}
TEST(LocalFollower, ExchangeProofFailuresExplainWaitWithoutDiscardingActiveCurve) {
 const std::vector<std::string> reasons={"native_negative:occupied","persistent_collision_state_required",
   "whole_curve_required","proof_time_invalid","persistent_collision_state_required"};
 for(unsigned which=0;which<reasons.size();++which) {
  SCOPED_TRACE(which);LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
  ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.geometryPermit(100.).allowed);
  f.step(100.,10.);ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));
  f.geometryPermit(100.05);ASSERT_FALSE(f.step(100.05,10.05).hold);
  ASSERT_TRUE(f.candidate(tag(ver(),2)));auto bad=proof(ver(),2,100.,2);
  if(which==0){bad.valid=false;bad.reason="occupied";}
  if(which==1)bad.valid_until=stamp(100.01);
  if(which==2){bad.whole_curve=false;bad.remaining_curve=true;}
  if(which==3)bad.check_end=bad.source_stamp=stamp(100.1);
  if(which==4)bad.valid_until=stamp(100.250025);
  ASSERT_TRUE(f.validation(bad));EXPECT_FALSE(f.exchangeCommit(100.07,10.07));
  EXPECT_EQ(f.candidateReason(),"waiting_current_native_proof:"+reasons[which]);
  EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
  EXPECT_TRUE(f.status(100.07).ready);EXPECT_FALSE(f.status(100.07).software_retired);
  EXPECT_NE(f.status(100.07).reason,f.candidateReason());
  ASSERT_TRUE(f.body(body(100.07,.0175),100.07,10.07));f.geometryPermit(100.07);
  EXPECT_FALSE(f.step(100.07,10.07).hold);
 }
}
TEST(LocalFollower, ReservedActualEntryReobservationKeepsOriginalAcquisitionBudget) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));
 auto old_body=proof();old_body.body_source_stamp=stamp(99.8);ASSERT_TRUE(f.validation(old_body));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.,10.));
 ASSERT_TRUE(f.body(body(100.21),100.21,10.21));
 const auto refresh=f.exchangeCommit(100.21,10.21);ASSERT_TRUE(refresh);
 EXPECT_EQ(*refresh,*request);
 EXPECT_EQ(f.candidateReason(),"entry_proof_body_gap_exceeded");
 EXPECT_EQ(f.commitSequence(),0u);
 EXPECT_FALSE(f.controller().hasInstalledGeometry());EXPECT_FALSE(f.status(100.21).software_retired);
 // A duplicate ACK and genuine but TOO LATE fresh evidence cannot resurrect
 // this reservation or restart its finite budget.
 EXPECT_FALSE(f.commitResult(ack,100.39,10.39));
 ASSERT_TRUE(f.body(body(100.39),100.39,10.39));
 EXPECT_EQ(*f.exchangeCommit(100.39,10.39),*request);
 ASSERT_TRUE(f.validation(proof(ver(),1,100.401,2)));
 ASSERT_TRUE(f.body(body(100.401),100.401,10.401));
 const auto aborted=f.exchangeCommit(100.401,10.401);ASSERT_TRUE(aborted);
 EXPECT_EQ(aborted->disposition,"abort");EXPECT_EQ(aborted->source_stamp,request->source_stamp);
 EXPECT_EQ(aborted->reason,"native_candidate_entry_refresh_expired");
 EXPECT_FALSE(f.controller().hasInstalledGeometry());EXPECT_EQ(f.commitSequence(),0u);
 EXPECT_FALSE(f.exchangeCommit(100.402,10.402));
 // A later solve is allowed under the same task, without recycling the old
 // candidate or the expired request's identity.
 ASSERT_TRUE(f.candidate(tag(ver(),2,0.,100.41)));
 ASSERT_TRUE(f.validation(proof(ver(),2,100.41,3)));
 const auto next=f.exchangeCommit(100.41,10.41);ASSERT_TRUE(next);
 EXPECT_EQ(next->disposition,"prepare");EXPECT_GT(next->commit_sequence,request->commit_sequence);
 EXPECT_EQ(next->version.task_id,request->version.task_id);
 EXPECT_FALSE(f.commitResult(ack,100.41,10.41)); // Late predecessor ACK cannot bind the new reservation.
}
TEST(LocalFollower, RecordedNearExpiryOffsetsNoLongerCreateACandidateRunwayGate) {
 // e60622ea actual stop episodes: at the other process's commit observation,
 // the new proof had these remaining original deadlines (not renewed TTLs).
 // Two were already expired by that observation; six had only 1--35 ms left.
 const std::vector<double> recorded_remaining={-.002086,.034101,.002855,-.000701,
   .029476,.009254,.014198,.001319};
 for(const double remaining:recorded_remaining) {
  SCOPED_TRACE(remaining);LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
  ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));f.geometryPermit(100.05);
  ASSERT_FALSE(f.step(100.05,10.05).hold);
  ASSERT_TRUE(f.body(body(100.10,.025),100.10,10.10));
  ASSERT_TRUE(f.validation(proof(ver(),1,100.10,2)));f.geometryPermit(100.10);
  const auto before=f.step(100.10,10.10);ASSERT_FALSE(before.hold);
  auto version=ver(2);auto p=proposal(version,100.10);p.expected_version=ver();p.expected_trajectory_id=1;
  ASSERT_TRUE(f.proposal(p,100.10));ASSERT_TRUE(f.support(support(version)));
  ASSERT_TRUE(f.candidate(tag(version,2,.025,100.10)));
  auto expiring=proof(version,2,100.10,3);
  EXPECT_LT(remaining,.1); // Original trace values; not copied into the state wire contract.
  ASSERT_TRUE(f.validation(expiring));const auto request=f.exchangeCommit(100.10,10.10);
  ASSERT_TRUE(request);ASSERT_EQ(request->disposition,"prepare");
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.15,10.15));
  ASSERT_TRUE(f.body(body(100.15,.0375),100.15,10.15));
  ASSERT_TRUE(f.validation(proof(ver(),1,100.15,4)));
  const auto commit=f.exchangeCommit(100.15,10.15);ASSERT_TRUE(commit);
  ASSERT_EQ(commit->disposition,"committed");EXPECT_EQ(commit->native_validation_sequence,3u);
  EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
  f.geometryPermit(100.15);const auto during=f.step(100.15,10.15);
  EXPECT_FALSE(during.hold);EXPECT_GT(during.velocity.linear.x,0.);
  EXPECT_LE(std::abs(during.velocity.linear.x-before.velocity.linear.x),cfg().max_acceleration*.05+1e-9);
  // A newer completed state can arrive before the commit ACK without
  // changing the original transaction identity or hiding a negative state.
  ASSERT_TRUE(f.candidate(tag(version,3,.03775,100.151)));
  const auto refreshed=proof(version,2,100.151,5);ASSERT_TRUE(f.validation(refreshed));
  ASSERT_TRUE(f.body(body(100.151,.03775),100.151,10.151));
  EXPECT_EQ(*f.exchangeCommit(100.151,10.151),*commit);EXPECT_EQ(commit->trajectory_id,2);
  EXPECT_EQ(commit->commit_sequence,request->commit_sequence);
  ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,100.152,10.152));
  ASSERT_TRUE(f.body(body(100.20,.05),100.20,10.20));f.geometryPermit(100.20);
  const auto after=f.step(100.20,10.20);EXPECT_FALSE(after.hold);EXPECT_GT(after.velocity.linear.x,0.);
  EXPECT_LE(std::abs(after.velocity.linear.x-during.velocity.linear.x),cfg().max_acceleration*.05+1e-9);
  EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);EXPECT_EQ(f.controller().trajectoryId(),2);
  EXPECT_EQ(f.commitSequence(),2u);EXPECT_TRUE(std::isnan(f.validationDiagnostic(100.20).remaining_s));
  EXPECT_EQ(f.validationDiagnostic(100.20).sequence,refreshed.sequence);
 }
}
TEST(LocalFollower, ReservedPersistentStateDoesNotMaskOccupiedCurrentCurveOrCancellation) {
 for(const bool cancel:{false,true}) {
  LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
  ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));f.geometryPermit(100.05);
  ASSERT_FALSE(f.step(100.05,10.05).hold);
  ASSERT_TRUE(f.candidate(tag(ver(),2,.0125,100.05)));auto expiring=proof(ver(),2,100.05,2);
  ASSERT_TRUE(f.validation(expiring));
  const auto request=f.exchangeCommit(100.05,10.05);ASSERT_TRUE(request);
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.05,10.05));
  EXPECT_FALSE(f.exchangeCommit(100.05,10.05,{},LocalFollower::ExchangeClock::InputDelivery));
  if(cancel) {
   f.retire("operator_cancel");EXPECT_FALSE(f.exchangeCommit(100.06,10.06));
   EXPECT_FALSE(f.validation(proof(ver(),2,100.06,3)));EXPECT_TRUE(f.status(100.06).software_retired);
  }else {
   auto occupied=proof(ver(),2,100.06,3);occupied.valid=false;occupied.reason="occupied";
   ASSERT_TRUE(f.validation(occupied));const auto aborted=f.exchangeCommit(100.06,10.06);ASSERT_TRUE(aborted);
   EXPECT_EQ(aborted->disposition,"abort");EXPECT_EQ(f.controller().trajectoryId(),1);
   // Rejecting a candidate does not stop a still-safe incumbent. An actual
   // negative CURRENT verdict, however, must stop it immediately.
   EXPECT_TRUE(f.status(100.06).ready);
   occupied=proof(ver(),1,100.06,4);occupied.valid=false;occupied.reason="occupied";
   ASSERT_TRUE(f.validation(occupied));
  }
  f.geometryPermit(100.06);const auto zero=f.step(100.06,10.06);
  EXPECT_TRUE(zero.hold);EXPECT_DOUBLE_EQ(zero.velocity.linear.x,0.);
 }
}
TEST(LocalFollower, ReservedNegativeStateCannotBeRenewedByDuplicateAcknowledgement) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
 ASSERT_TRUE(f.candidate(tag(ver(),2)));ASSERT_TRUE(f.validation(proof(ver(),2,100.,2)));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.,10.));
 auto expired=proof(ver(),2,100.01,3);expired.valid=false;expired.reason="occupied";
 ASSERT_TRUE(f.validation(expired));const auto retry=f.exchangeCommit(100.01,10.01);ASSERT_TRUE(retry);
 EXPECT_EQ(retry->disposition,"abort");EXPECT_EQ(f.controller().trajectoryId(),1);
 EXPECT_FALSE(f.commitResult(ack,100.02,10.02));EXPECT_FALSE(f.exchangeCommit(100.02,10.02));
 EXPECT_FALSE(f.validation(proof(ver(),2,100.,2))); // Old positive cannot undo the completed negative.
}
TEST(LocalFollower, WaitingForReplacementDoesNotExpireCommittedIncumbentState) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
 ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);f.step(100.,10.);
 ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));f.geometryPermit(100.05);
 ASSERT_FALSE(f.step(100.05,10.05).hold);
 ASSERT_TRUE(f.candidate(tag(ver(),2,.0125,100.05)));auto expiring=proof(ver(),2,100.05,2);
 ASSERT_TRUE(f.validation(expiring));
 const auto request=f.exchangeCommit(100.05,10.05);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.05,10.05));
 ASSERT_EQ(f.exchangeCommit(100.05,10.05)->disposition,"committed");
 ASSERT_TRUE(f.body(body(100.21,.0525),100.21,10.21));
 ASSERT_EQ(f.exchangeCommit(100.21,10.21)->disposition,"committed");
 f.geometryPermit(100.21);const auto current=f.step(100.21,10.21);
 EXPECT_FALSE(current.hold);EXPECT_GT(current.velocity.linear.x,0.);
 EXPECT_EQ(current.validation_sequence,1u);EXPECT_EQ(f.controller().trajectoryId(),1);
 EXPECT_EQ(f.status(100.21).phase,"following");
 EXPECT_TRUE(f.validationDiagnostic(100.21).failure.empty());
}
TEST(LocalFollower, CollisionStateAdmissionIsIndependentOfControllerAndWriterRunway) {
 for(const double hz:{10.,20.,50.})for(const double pipeline:{.1,.2,.5}) {
  SCOPED_TRACE(hz);
  SCOPED_TRACE(pipeline);
  LocalFollower f(cfg(),"isolated_mock",{},false,1./hz,pipeline,.05);
  ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
  ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));
  auto initial=proof();ASSERT_TRUE(f.validation(initial));
  const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.,10.));
  const auto commit=f.exchangeCommit(100.,10.);ASSERT_TRUE(commit)<<f.candidateReason();
  EXPECT_EQ(commit->disposition,"committed");EXPECT_EQ(commit->native_validation_sequence,1u);
  EXPECT_EQ(timeNs(initial.valid_until),0);
 }
}
TEST(LocalFollower, PersistentStateDiagnosticsRetainOriginalSamplesWithoutAMotionExpiry) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 const auto initial=f.validationDiagnostic(100.01);
 EXPECT_TRUE(std::isnan(initial.front_age_s));EXPECT_TRUE(std::isnan(initial.rear_age_s));
 auto v=proof(ver(),1,100.10,2);
 v.check_begin=stamp(100.09);v.body_source_stamp=stamp(100.02);
 v.front_ray_source_stamp=stamp(99.65);v.rear_ray_source_stamp=stamp(99.83);
 v.map_snapshot_revision=7;
 ASSERT_TRUE(f.validation(v));
 const auto before=f.validationDiagnostic(100.14);
 EXPECT_TRUE(before.failure.empty());EXPECT_EQ(before.map_revision,7u);
 EXPECT_NEAR(before.check_duration_s,.01,1e-9);
 EXPECT_NEAR(before.body_age_s,.12,1e-9);EXPECT_NEAR(before.front_age_s,.49,1e-9);
 EXPECT_NEAR(before.rear_age_s,.31,1e-9);EXPECT_TRUE(std::isnan(before.remaining_s));
 EXPECT_EQ(before.source_ns,100100000000LL);EXPECT_EQ(before.check_begin_ns,100090000000LL);
 EXPECT_EQ(before.check_end_ns,100100000000LL);EXPECT_EQ(before.valid_until_ns,0);
 EXPECT_EQ(before.body_source_ns,100020000000LL);EXPECT_EQ(before.front_source_ns,99650000000LL);
 EXPECT_EQ(before.rear_source_ns,99830000000LL);
 const auto after=f.validationDiagnostic(100.16);
 EXPECT_TRUE(after.failure.empty());EXPECT_TRUE(std::isnan(after.remaining_s));
 EXPECT_NEAR(after.front_age_s,.51,1e-9);EXPECT_EQ(after.sequence,2u);
 EXPECT_EQ(after.source_ns,before.source_ns);EXPECT_EQ(after.valid_until_ns,before.valid_until_ns);
 EXPECT_EQ(f.commitSequence(),1u);EXPECT_EQ(f.controller().trajectoryId(),1);
}
TEST(LocalFollower,VerifiedFloorTwoGeneralGeometryCanPrepareWithoutSdkAuthorization) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 auto measured=body();measured.position.z()=3.55;
 auto p=proposal();p.goal_position.z=3.55;
 auto s=support();s.floor_id="floor2";for(auto& point:s.support_ground_xyz)point.z=3.;
 auto candidate=tag();for(auto& point:candidate.trajectory.pos_pts)point.z=3.55;
 candidate.join_pose.position.z=3.55;
 ASSERT_TRUE(f.body(measured,100.,10.));ASSERT_TRUE(f.proposal(p,100.));ASSERT_TRUE(f.support(s));
 ASSERT_TRUE(f.candidate(candidate));ASSERT_TRUE(f.validation(proof()));
 ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();
 EXPECT_TRUE(f.status(100.01).ready);EXPECT_TRUE(f.controller().hasInstalledGeometry());
 EXPECT_FALSE(f.geometryPermit(100.01).allowed);EXPECT_TRUE(f.step(100.01,10.01).hold);
}
TEST(LocalFollower, BadCandidateNeverClearsAcceptedController) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 auto bad=tag(ver(),2);bad.trajectory.knots[4]=bad.trajectory.knots[3];
 EXPECT_TRUE(f.candidate(bad));auto p=proof(ver(),2);EXPECT_TRUE(f.validation(p));
 EXPECT_FALSE(f.tryCommit(100.01,10.01));EXPECT_TRUE(f.controller().hasInstalledGeometry());
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, CurveAndAnchorChangeDoNotNeedAnotherTaskGrant) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 auto v=ver(2);v.anchor_id="corrected";v.anchor_revision=2;
 auto p=proposal(v,100.02);p.expected_version=ver();p.expected_trajectory_id=1;
 ASSERT_TRUE(f.proposal(p,100.02));ASSERT_TRUE(f.support(support(v)));
 ASSERT_TRUE(f.candidate(tag(v,2,0.,100.02)));ASSERT_TRUE(f.body(body(100.02),100.02,10.02));
 ASSERT_TRUE(f.validation(proof(v,2,100.02,2)));ASSERT_TRUE(f.tryCommit(100.02,10.02));
 EXPECT_EQ(f.controller().trajectoryId(),2);EXPECT_TRUE(f.geometryPermit(100.02).allowed);
 EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);
}
TEST(LocalFollower, SameTaskCurveHandoffCarriesCommonFrameLimiterAndHardExpiryStillZeros) {
 auto c=cfg();c.heading_threshold=.8;c.require_body_heading_seed=true;
 LocalFollower f(c,"isolated_mock",{},false);
 auto first_tag=tag();first_tag.body_heading_seed_valid=true;first_tag.body_heading_seed_yaw=0.;
 first_tag.body_heading_sampling_intervals=200;
 ASSERT_TRUE(f.body(body(),100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));ASSERT_TRUE(f.support(support()));
 ASSERT_TRUE(f.candidate(first_tag));ASSERT_TRUE(f.validation(proof()));ASSERT_TRUE(f.tryCommit(100.,10.));
 Eigen::Vector2d previous_world=Eigen::Vector2d::Zero();
 for(unsigned i=1;i<=14;++i) {
   const double now=100.+i*.05,receipt=10.+i*.05;auto o=body(now,.25*i*.05);
   if(i>=13){o.yaw=.025;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());o.position.y()=-.005;}
   ASSERT_TRUE(f.body(o,now,receipt));ASSERT_TRUE(f.lease(lease(now,i),now));
   if(i==13) {
     auto replacement=tag(ver(),2,o.position.x(),now);replacement.body_heading_seed_valid=true;
     replacement.body_heading_seed_yaw=0.;replacement.body_heading_sampling_intervals=200;
     ASSERT_TRUE(f.candidate(replacement));ASSERT_TRUE(f.validation(proof(ver(),2,now,i+1)));
     ASSERT_TRUE(f.tryCommit(now,receipt))<<f.candidateReason();
     EXPECT_EQ(f.controller().trajectoryId(),2);EXPECT_EQ(f.commitSequence(),2u);
   } else ASSERT_TRUE(f.validation(proof(ver(),i>13?2:1,now,i+1)));
   ASSERT_TRUE(f.geometryPermit(now).allowed);const auto d=f.step(now,receipt);
   const Eigen::Vector2d world=Eigen::Rotation2Dd(o.yaw)*Eigen::Vector2d(d.velocity.linear.x,d.velocity.linear.y);
   EXPECT_LE((world-previous_world).norm(),c.max_acceleration*.05+1e-12);
   EXPECT_FALSE(d.hold);EXPECT_EQ(d.reason,"tracking");EXPECT_EQ(d.execution_id,"execution");
   if(i==13)EXPECT_GT(world.norm(),.2); // Not a startup restart at the new curve.
   previous_world=world;
 }
 // A current-map/source/permission gap is still immediate zero; no transported
 // previous motion leaks past original deadlines or resumes a retired task.
 f.geometryPermit(101.10);const auto zero=f.step(101.10,11.10);
 EXPECT_TRUE(zero.hold);EXPECT_DOUBLE_EQ(zero.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(zero.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(zero.velocity.angular.z,0.);EXPECT_EQ(f.controller().trajectoryId(),2);
 f.retire("operator_cancel");EXPECT_TRUE(f.step(101.15,11.15).hold);
 EXPECT_FALSE(f.candidate(first_tag));EXPECT_FALSE(f.controller().hasInstalledGeometry());
}
TEST(LocalFollower, CommonPrefixSwitchUsesMeasuredProgressNotElapsedWallClock) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 auto tail=tag(ver(),2,.05,100.02);tail.splice.enabled=true;tail.splice.incumbent_version=ver();
 tail.splice.incumbent_trajectory_id=1;tail.splice.measured_progress_stamp=stamp(100.);
 tail.splice.splice_curve_time=.2;tail.splice.splice_arc_length=.05;tail.splice.position.x=.05;
 tail.splice.position.z=.55;tail.splice.velocity.x=.25;
 ASSERT_TRUE(f.candidate(tail));auto vp=proof(ver(),2,100.02,2);vp.splice=tail.splice;ASSERT_TRUE(f.validation(vp));
 ASSERT_TRUE(f.body(body(100.02),100.02,10.02));
 EXPECT_FALSE(f.tryCommit(100.02,10.02));EXPECT_EQ(f.controller().trajectoryId(),1);
 // Advancing time without physical displacement never switches the tail.
 ASSERT_TRUE(f.body(body(100.07),100.07,10.07));f.geometryPermit(100.07);f.step(100.07,10.07);
 EXPECT_FALSE(f.tryCommit(100.07,10.07));EXPECT_EQ(f.controller().trajectoryId(),1);
 ASSERT_TRUE(f.body(body(100.20,.05),100.20,10.20));
 auto fresh=proof(ver(),1,100.20,3);f.validation(fresh);f.geometryPermit(100.20);f.step(100.20,10.20);
 vp=proof(ver(),2,100.20,4);vp.splice=tail.splice;ASSERT_TRUE(f.validation(vp));
 ASSERT_TRUE(f.tryCommit(100.20,10.20))<<f.candidateReason();EXPECT_EQ(f.controller().trajectoryId(),2);
}
TEST(LocalFollower, LeaseExpiryHoldsAndFreshSameTaskResumesWithoutNewCurve) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 f.geometryPermit(100.36);EXPECT_TRUE(f.step(100.36,10.36).hold);
 ASSERT_TRUE(f.body(body(100.40),100.40,10.40));ASSERT_TRUE(f.validation(proof(ver(),1,100.40,2)));
 ASSERT_TRUE(f.lease(lease(100.40,2),100.40));EXPECT_TRUE(f.geometryPermit(100.40).allowed);
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, UnboundLeaseCanBindOnceButForeignSdkCannotReplaceIt) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 auto p=lease();p.allowed=false;p.sdk_session.clear();p.sdk_arm_generation=0;ASSERT_TRUE(f.lease(p,100.));
 ASSERT_TRUE(f.lease(lease(100.02,2),100.02));p=lease(100.03,3);p.sdk_session="other";
 EXPECT_FALSE(f.lease(p,100.03));EXPECT_TRUE(f.geometryPermit(100.03).allowed);
}
TEST(LocalFollower, NewInvalidNativeVerdictCannotFallBackToOldProof) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 auto p=proof(ver(),1,100.02,2);p.valid=false;ASSERT_TRUE(f.validation(p));
 f.geometryPermit(100.03);EXPECT_TRUE(f.step(100.03,10.03).hold);
 EXPECT_TRUE(f.controller().hasInstalledGeometry());EXPECT_FALSE(f.status(100.03).ready);
}
TEST(LocalFollower, NewGoalWaitsForOldSoftwareRetirementAndDoesNotReviveOldCurve) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 auto p=proposal(ver(3),100.02);p.version.task_id="new";EXPECT_FALSE(f.proposal(p,100.02));
 f.retire("cancelled");ASSERT_TRUE(f.proposal(p,100.02));
 EXPECT_FALSE(f.candidate(tag()));EXPECT_FALSE(f.controller().hasInstalledGeometry());
}
TEST(LocalFollower, NewGoalStagesSupportCandidateAndProofBeforeProposal) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);f.retire("cancelled");
 auto v=ver(3);v.task_id="new";v.route_id="new-route";
 ASSERT_TRUE(f.support(support(v)));ASSERT_TRUE(f.candidate(tag(v,2,0.,100.02)));
 ASSERT_TRUE(f.validation(proof(v,2,100.02,2)));
 EXPECT_FALSE(f.tryCommit(100.02,10.02));
 EXPECT_FALSE(f.support(support()));EXPECT_FALSE(f.validation(proof()));
 ASSERT_TRUE(f.proposal(proposal(v,100.02),100.02));
 ASSERT_TRUE(f.body(body(100.02),100.02,10.02));
 ASSERT_TRUE(f.tryCommit(100.02,10.02))<<f.candidateReason();
 EXPECT_EQ(f.activeVersion()->task_id,"new");
 EXPECT_FALSE(f.geometryPermit(100.02).allowed);
 EXPECT_FALSE(f.candidate(tag()));EXPECT_FALSE(f.validation(proof()));
}
TEST(LocalFollower, DurableProposalIsGeometryButNeverFreshMotionEvidence) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 const auto original=proposal();ASSERT_TRUE(f.proposal(original,102.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));
 ASSERT_TRUE(f.body(body(102.),102.,12.));
 ASSERT_TRUE(f.validation(proof()));EXPECT_FALSE(f.tryCommit(102.,12.));
 ASSERT_TRUE(f.validation(proof(ver(),1,102.,2)));ASSERT_TRUE(f.tryCommit(102.,12.));
 EXPECT_FALSE(f.geometryPermit(102.).allowed);
 auto future=original;future.version=ver(2);future.source_stamp=stamp(103.);future.valid_until=stamp(103.9);
 EXPECT_FALSE(f.proposal(future,102.));
 auto malformed=original;malformed.version=ver(2);malformed.valid_until=malformed.source_stamp;
 EXPECT_FALSE(f.proposal(malformed,102.));
 malformed=original;malformed.version=ver(2);malformed.source_stamp.nanosec=1000000000U;
 EXPECT_FALSE(f.proposal(malformed,102.));
 f.retire("cancelled");EXPECT_FALSE(f.proposal(original,103.));
}
TEST(LocalFollower, SourceExpiryCannotBeRenewedByAuthorityOrDisplayHeartbeat) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 ASSERT_TRUE(f.validation(proof(ver(),1,100.12,2)));f.geometryPermit(100.12);
 EXPECT_TRUE(f.step(100.12,10.12).hold);EXPECT_FALSE(f.status(100.12).at_goal);
 EXPECT_EQ(f.controller().odometry().provider_valid_until_ns,100100000000LL);
}
TEST(LocalFollower, GoalPoseAtHighMeasuredLinearSpeedSettlesWithoutRequestingSdkStop) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
 p.has_goal_yaw=true;p.goal_yaw=0.;p.goal_yaw_tolerance_rad=.1;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));
 for(unsigned i=1;i<=3;i++) {
   const double now=100.+i*.01;auto o=body(now,.05,.25);
   if(i==2)o.velocity_in_frame.setZero(); // The scalar speed alone still prevents arrival.
   if(i==3){o.planar_speed=0.;o.velocity_in_frame={0.,.25,0.};} // The XY norm also prevents arrival.
   ASSERT_TRUE(f.body(o,now,10.+i*.01));
   EXPECT_EQ(f.geometryPermit(now).phase,"tracking");
   const auto s=f.status(now);EXPECT_TRUE(s.ready);EXPECT_FALSE(s.at_goal);
   EXPECT_FALSE(s.request_stop);EXPECT_FALSE(s.software_retired);EXPECT_EQ(s.phase,"settling");
 }
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, SettledGoalStatusAndDemandAgreeOnExactZeroDespiteSavedCommand) {
 for(bool explicit_yaw:{false,true}) {
   LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
   p.has_goal_yaw=explicit_yaw;p.goal_yaw=0.;p.goal_yaw_tolerance_rad=.07;stage(f,p);
   ASSERT_TRUE(f.lease(lease(),100.));
   f.controller().recordAppliedOutput(.10,.04,.20,100.,10.);
   auto o=body(100.05,.05,0.);o.yaw=explicit_yaw?.06:0.;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
   ASSERT_TRUE(f.body(o,100.05,10.05));ASSERT_TRUE(f.validation(proof(ver(),1,100.05,2)));
   // Already within the explicit goal-yaw tolerance needs no new rotation
   // proof; no nonzero yaw tail may leak through that zero-motion branch.
   EXPECT_TRUE(f.geometryPermit(100.05).allowed);const auto d=f.step(100.05,10.05);
   const auto s=f.status(100.05);EXPECT_TRUE(s.ready);EXPECT_TRUE(s.at_goal);EXPECT_TRUE(s.request_stop);
   EXPECT_FALSE(s.software_retired);EXPECT_EQ(s.phase,"goal_reached");
   EXPECT_TRUE(d.hold);EXPECT_DOUBLE_EQ(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);
   EXPECT_DOUBLE_EQ(d.velocity.angular.z,0.);EXPECT_TRUE(f.controller().active());EXPECT_EQ(f.commitSequence(),1u);
   EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(d.execution_id,"execution");
 }
}
TEST(LocalFollower, ForwardSteeringMeasuredRateDoesNotPredictAForcedPause) {
 auto c=cfg();c.align_then_forward=true;c.kp_yaw=.8;c.yaw_damping_gain=.35;c.lookahead=.8;
 LocalFollower f(c,"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 auto o=body(100.01);o.yaw=-.18;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
 o.angular_velocity_in_frame.z()=.20;ASSERT_TRUE(f.body(o,100.01,10.01));
 const auto permit=f.geometryPermit(100.01);ASSERT_TRUE(permit.allowed);EXPECT_EQ(permit.phase,"tracking");
 const auto demand=f.step(100.01,10.01);EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
 EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);
 // Continuous soft-error feedback plus the measured rate remains positive.
 // No assumed .8-s coasting horizon may turn this ready task into a pause.
 const double soft_error=.18-c.steering_error_scale_rad*std::tanh(.18/c.steering_error_scale_rad);
 EXPECT_NEAR(f.controller().controlDiagnostic().yaw_target,c.kp_yaw*soft_error-c.yaw_damping_gain*.20,1e-12);
 EXPECT_GT(demand.velocity.angular.z,0.);
 EXPECT_LE(demand.velocity.angular.z,c.max_yaw_acceleration*.01+1e-12);
 EXPECT_EQ(demand.reason,"tracking_forward");
 const auto status=f.status(100.01);EXPECT_TRUE(status.ready);EXPECT_FALSE(status.holding);
 EXPECT_EQ(status.phase,"following");EXPECT_FALSE(status.request_stop);EXPECT_FALSE(status.software_retired);
 EXPECT_EQ(status.version.task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, TwentyHzContinuousYawFeedbackKeepsTypedFollowingAndCancelStillZeros) {
 auto c=cfg();c.align_then_forward=true;c.holonomic_yaw_policy="path_tangent";
 c.max_acceleration=.2;c.max_yaw_acceleration=.4;c.max_jerk=.4;c.max_yaw_jerk=.8;
 c.kp_yaw=.8;c.yaw_damping_gain=.35;
 LocalFollower f(c,"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 double previous_yaw=0.,previous_yaw_acceleration=0.,previous_forward=0.;
 for(unsigned i=1;i<=50;++i) {
   const double now=100.+i*.05;auto o=body(now,.25*i*.05);
   o.yaw=i<=10?-.22:-.18;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
   if(i>10)o.angular_velocity_in_frame.z()=.30; // Independent body observation, not echoed command.
   ASSERT_TRUE(f.body(o,now,now-90.));ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));
   ASSERT_TRUE(f.lease(lease(now,i+1),now));EXPECT_TRUE(f.geometryPermit(now).allowed);
   const auto demand=f.step(now,now-90.);const auto status=f.status(now);
   EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_FALSE(demand.hold);EXPECT_TRUE(status.ready);
   EXPECT_FALSE(status.holding);EXPECT_EQ(status.phase,"following");EXPECT_FALSE(status.request_stop);
   EXPECT_EQ(status.version.task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
   EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);
   EXPECT_LE(std::abs(demand.velocity.linear.x-previous_forward),c.max_acceleration*.05+1e-12);
   EXPECT_LE(std::abs(demand.velocity.angular.z-previous_yaw),c.max_yaw_acceleration*.05+1e-12);
   const double acceleration=(demand.velocity.angular.z-previous_yaw)/.05;
   EXPECT_LE(std::abs(acceleration-previous_yaw_acceleration),c.max_yaw_jerk*.05+1e-10);
   previous_forward=demand.velocity.linear.x;previous_yaw=demand.velocity.angular.z;
   previous_yaw_acceleration=acceleration;
   if(i>10) {
     const double soft_error=.18-c.steering_error_scale_rad*std::tanh(.18/c.steering_error_scale_rad);
     EXPECT_NEAR(f.controller().controlDiagnostic().yaw_target,c.kp_yaw*soft_error-c.yaw_damping_gain*.30,1e-12);
     EXPECT_GT(demand.velocity.angular.z,0.);
     EXPECT_EQ(demand.reason,"tracking_forward");
   }
 }
 f.retire("operator_cancel");const auto stopped=f.step(102.55,12.55);
 EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);EXPECT_TRUE(stopped.hold);
 EXPECT_TRUE(f.status(102.55).software_retired);
}
TEST(LocalFollower, ContinuousSmallHeadingNoiseDoesNotPauseOrEmitLargeAlternatingInputs) {
 auto c=cfg();c.align_then_forward=true;c.kp_yaw=1.2;c.yaw_damping_gain=.15;
 c.steering_error_scale_rad=.04;c.max_jerk=.70;c.max_yaw_jerk=2.;
 c.forward_response={c.max_speed,.31328213472822414,.40,.10};
 c.yaw_response={c.max_yaw_rate,.20,.30,.20};
 LocalFollower f(c,"isolated_mock",{},false);stage(f);
 double previous_forward=0.,previous_yaw=0.,previous_forward_acceleration=0.,previous_yaw_acceleration=0.;
 double previous_forward_input_rate=0.,previous_yaw_input_rate=0.;
 for(unsigned i=1;i<=30;++i) {
   const double now=100.+i*.05;auto o=body(now,.25*i*.05);
   const double error=(i%2?-1.:1.)*M_PI/180.;o.yaw=-error;
   o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
   ASSERT_TRUE(f.body(o,now,now-90.));ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));
   ASSERT_TRUE(f.lease(lease(now,i),now));ASSERT_TRUE(f.geometryPermit(now).allowed);
   const auto d=f.step(now,now-90.);const auto s=f.status(now);
   EXPECT_GT(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);
   EXPECT_LT(std::abs(c.yaw_response.map(d.velocity.angular.z)),.02);
   EXPECT_LE(std::abs(d.velocity.linear.x-previous_forward),c.max_acceleration*.05+1e-12);
   EXPECT_LE(std::abs(d.velocity.angular.z-previous_yaw),c.max_yaw_acceleration*.05+1e-12);
   const double acceleration=(d.velocity.linear.x-previous_forward)/.05;
   const double yaw_acceleration=(d.velocity.angular.z-previous_yaw)/.05;
   EXPECT_LE(std::abs(acceleration-previous_forward_acceleration),c.max_jerk*.05+1e-8);
   EXPECT_LE(std::abs(yaw_acceleration-previous_yaw_acceleration),c.max_yaw_jerk*.05+1e-8);
   const double input_rate=(c.forward_response.map(d.velocity.linear.x)-c.forward_response.map(previous_forward))/.05;
   const double yaw_input_rate=(c.yaw_response.map(d.velocity.angular.z)-c.yaw_response.map(previous_yaw))/.05;
   EXPECT_LE(std::abs(input_rate),c.max_input_rate+1e-8);
   EXPECT_LE(std::abs(yaw_input_rate),c.max_input_rate+1e-8);
   EXPECT_LE(std::abs(input_rate-previous_forward_input_rate),c.max_input_acceleration*.05+1e-8);
   EXPECT_LE(std::abs(yaw_input_rate-previous_yaw_input_rate),c.max_input_acceleration*.05+1e-8);
   EXPECT_FALSE(d.hold);EXPECT_TRUE(s.ready);EXPECT_FALSE(s.holding);EXPECT_EQ(s.phase,"following");
   EXPECT_FALSE(s.request_stop);EXPECT_EQ(s.version.task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
   previous_forward=d.velocity.linear.x;previous_yaw=d.velocity.angular.z;
   previous_forward_acceleration=acceleration;previous_yaw_acceleration=yaw_acceleration;
   previous_forward_input_rate=input_rate;previous_yaw_input_rate=yaw_input_rate;
 }
 // Soft steering is not permission to renew expired sensor evidence with a
 // fresh task lease or map verdict. The original provider deadline still wins.
 const double now=101.62;ASSERT_TRUE(f.validation(proof(ver(),1,now,40)));
 ASSERT_TRUE(f.lease(lease(now,40),now));f.geometryPermit(now);const auto expired=f.step(now,now-90.);
 EXPECT_TRUE(expired.hold);EXPECT_DOUBLE_EQ(expired.velocity.linear.x,0.);
 EXPECT_DOUBLE_EQ(expired.velocity.linear.y,0.);EXPECT_DOUBLE_EQ(expired.velocity.angular.z,0.);
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, FormalResponseConditionerSurvivesCommittedReplansButNotExpiredSource) {
 auto c=cfg();c.align_then_forward=true;c.kp_yaw=1.5;c.heading_threshold=.8;
 c.steering_enter_angle_rad=8.*M_PI/180.;c.steering_exit_angle_rad=5.*M_PI/180.;
 c.forward_response={c.max_speed,.31328213472822414,.40,.10};
 c.yaw_response={c.max_yaw_rate,.30,.40,.10};
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 double previous_forward=0.,previous_yaw=0.,previous_forward_rate=0.,previous_yaw_rate=0.;
 std::int64_t id=1;
 for(unsigned i=1;i<=20;++i) {
   const double now=100.+i*.05;auto o=body(now,.25*i*.05);
   o.yaw=-.20;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
   ASSERT_TRUE(f.body(o,now,now-90.));ASSERT_TRUE(f.lease(lease(now,i),now));
   if(i%4==0) {
     ++id;ASSERT_TRUE(f.candidate(tag(ver(),id,o.position.x(),now)));
     ASSERT_TRUE(f.validation(proof(ver(),id,now,i+1)));
     const auto request=f.exchangeCommit(now,now-90.);ASSERT_TRUE(request);
     ASSERT_EQ(request->disposition,"prepare");auto ack=*request;ack.accepted=true;ack.disposition="prepared";
     ASSERT_TRUE(f.commitResult(ack,now,now-90.));
     const auto commit=f.exchangeCommit(now,now-90.);ASSERT_TRUE(commit)<<f.candidateReason();
     ASSERT_EQ(commit->disposition,"committed");ack=*commit;ack.accepted=true;
     ASSERT_TRUE(f.commitResult(ack,now,now-90.));
   } else ASSERT_TRUE(f.validation(proof(ver(),id,now,i+1)));
   ASSERT_TRUE(f.geometryPermit(now).allowed);const auto d=f.step(now,now-90.);
   ASSERT_FALSE(d.hold);EXPECT_GT(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);
   EXPECT_GT(d.velocity.angular.z,0.);EXPECT_EQ(d.version.task_id,"task");
   const double forward=c.forward_response.map(d.velocity.linear.x),yaw=c.yaw_response.map(d.velocity.angular.z);
   const double forward_rate=(forward-previous_forward)/.05,yaw_rate=(yaw-previous_yaw)/.05;
   ASSERT_FALSE(f.controller().controlDiagnostic().input_limit_override);
   EXPECT_LE(std::abs(forward_rate),c.max_input_rate+1e-8);
   EXPECT_LE(std::abs(yaw_rate),c.max_input_rate+1e-8);
   EXPECT_LE(std::abs(forward_rate-previous_forward_rate),c.max_input_acceleration*.05+1e-7);
   EXPECT_LE(std::abs(yaw_rate-previous_yaw_rate),c.max_input_acceleration*.05+1e-7);
   previous_forward=forward;previous_yaw=yaw;previous_forward_rate=forward_rate;previous_yaw_rate=yaw_rate;
 }
 EXPECT_EQ(f.commitSequence(),6u);EXPECT_GT(previous_forward,.30);
 // Fresh lease/map heartbeats cannot repair the original provider source.
 const double expired_time=101.16;ASSERT_TRUE(f.validation(proof(ver(),id,expired_time,30)));
 ASSERT_TRUE(f.lease(lease(expired_time,30),expired_time));f.geometryPermit(expired_time);
 const auto expired=f.step(expired_time,expired_time-90.);
 EXPECT_TRUE(expired.hold);EXPECT_DOUBLE_EQ(expired.velocity.linear.x,0.);
 EXPECT_DOUBLE_EQ(expired.velocity.linear.y,0.);EXPECT_DOUBLE_EQ(expired.velocity.angular.z,0.);
 EXPECT_EQ(f.commitSequence(),6u);EXPECT_EQ(f.activeVersion()->task_id,"task");
 // Resuming the same task after a real zero starts the ONE conditioner from
 // zero, rather than restoring the pre-expiry SDK amplitude/rate history.
 bool recovered=false;
 for(unsigned i=0;i<16;++i) {
   const double now=101.21+i*.05;auto resumed_body=body(now,.25);
   resumed_body.yaw=-.20;resumed_body.orientation=Eigen::AngleAxisd(-.20,Eigen::Vector3d::UnitZ());
   ASSERT_TRUE(f.body(resumed_body,now,now-90.));ASSERT_TRUE(f.validation(proof(ver(),id,now,31+i)));
   ASSERT_TRUE(f.lease(lease(now,31+i),now));ASSERT_TRUE(f.geometryPermit(now).allowed);
   const auto resumed=f.step(now,now-90.);
   if(resumed.hold)continue; // Preserve the existing distinct-source recovery window.
   EXPECT_GT(resumed.velocity.linear.x,0.);
   EXPECT_LE(c.forward_response.map(resumed.velocity.linear.x),c.max_input_acceleration*.05*.05+1e-8);
   EXPECT_LE(c.yaw_response.map(resumed.velocity.angular.z),c.max_input_acceleration*.05*.05+1e-8);
   recovered=true;break;
 }
 EXPECT_TRUE(recovered);
}
TEST(LocalFollower, ForwardGoalEntryDeceleratesTypedDemandUntilMeasuredRest) {
 auto c=cfg();c.align_then_forward=true;c.max_acceleration=.35;c.max_yaw_acceleration=.8;
 LocalFollower f(c,"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;stage(f,p);
 f.controller().recordAppliedOutput(.20,0.,.10,100.,10.);
 double previous_forward=.20,previous_yaw=.10;
 for(unsigned i=1;i<=12;++i) {
   const double now=100.+i*.05;auto o=body(now,.05,.20);o.angular_velocity_in_frame.z()=.10;
   ASSERT_TRUE(f.body(o,now,now-90.));ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));
   ASSERT_TRUE(f.lease(lease(now,i),now));ASSERT_TRUE(f.geometryPermit(now).allowed);
   const auto d=f.step(now,now-90.);const auto s=f.status(now);
   EXPECT_EQ(d.reason,"goal_position_settling");EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);
   EXPECT_GE(d.velocity.linear.x,0.);EXPECT_GE(d.velocity.angular.z,0.);
   EXPECT_LE(std::abs(d.velocity.linear.x-previous_forward),c.max_acceleration*.05+1e-12);
   EXPECT_LE(std::abs(d.velocity.angular.z-previous_yaw),c.max_yaw_acceleration*.05+1e-12);
   if(i==1){
     // SCAN owns geometry; an unmeasured raw-PCD slope limit must not rescale
     // the controller's acceleration on an already approved flat curve.
     EXPECT_NEAR(d.velocity.linear.x,.20-c.max_acceleration*.05,1e-12);
     EXPECT_NEAR(d.velocity.angular.z,.06,1e-12);
   }
   EXPECT_TRUE(s.ready);EXPECT_FALSE(s.at_goal);EXPECT_FALSE(s.request_stop);
   EXPECT_EQ(s.phase,"settling");EXPECT_EQ(s.version.task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
   previous_forward=d.velocity.linear.x;previous_yaw=d.velocity.angular.z;
 }
 // Actual fresh rest, not elapsed settling time or command zero, permits the
 // original completion status. It does not retire the task or forge SDK rest.
 ASSERT_TRUE(f.body(body(100.65,.05,0.),100.65,10.65));
 ASSERT_TRUE(f.validation(proof(ver(),1,100.65,14)));ASSERT_TRUE(f.lease(lease(100.65,13),100.65));
 ASSERT_TRUE(f.geometryPermit(100.65).allowed);const auto stopped=f.step(100.65,10.65);
 EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);EXPECT_TRUE(stopped.hold);
 const auto s=f.status(100.65);EXPECT_TRUE(s.at_goal);EXPECT_TRUE(s.request_stop);EXPECT_FALSE(s.software_retired);
}
TEST(LocalFollower, ForwardGoalYawUsesDirectFeedbackAndStillRequiresNewestNativeProof) {
 auto c=cfg();c.align_then_forward=true;c.kp_yaw=1.5;c.max_yaw_acceleration=.8;
 LocalFollower f(c,"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
 p.has_goal_yaw=true;p.goal_yaw=.15;p.goal_yaw_tolerance_rad=.07;stage(f,p);
 f.controller().recordAppliedOutput(0.,0.,.10,100.,10.);
 auto o=body(100.05,.05,0.);o.angular_velocity_in_frame.z()=.20;
 auto valid=proof(ver(),1,100.05,2);valid.goal_yaw_checked=true;valid.checked_goal_yaw=p.goal_yaw;
 ASSERT_TRUE(f.body(o,100.05,10.05));ASSERT_TRUE(f.validation(valid));ASSERT_TRUE(f.lease(lease(100.05,1),100.05));
 ASSERT_EQ(f.geometryPermit(100.05).phase,"aligning");const auto d=f.step(100.05,10.05);
 // Old 0.8-s prediction computed .15-.20*.8<0 and forced this yaw to zero.
 EXPECT_EQ(d.reason,"aligning");EXPECT_NEAR(d.velocity.angular.z,.14,1e-12);
 EXPECT_DOUBLE_EQ(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);EXPECT_FALSE(d.hold);
 EXPECT_FALSE(f.status(100.05).request_stop);EXPECT_FALSE(f.status(100.05).at_goal);
 auto blocked=proof(ver(),1,100.10,3);blocked.valid=false;ASSERT_TRUE(f.validation(blocked));
 o=body(100.10,.05,0.);o.angular_velocity_in_frame.z()=.20;ASSERT_TRUE(f.body(o,100.10,10.10));
 ASSERT_TRUE(f.lease(lease(100.10,2),100.10));f.geometryPermit(100.10);const auto zero=f.step(100.10,10.10);
 EXPECT_TRUE(zero.hold);EXPECT_DOUBLE_EQ(zero.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(zero.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(zero.velocity.angular.z,0.);EXPECT_FALSE(f.status(100.10).ready);
 EXPECT_TRUE(f.controller().hasInstalledGeometry());EXPECT_EQ(f.commitSequence(),1u);
 EXPECT_FALSE(f.validation(valid)); // No fallback to old positive orientation evidence.
 valid=proof(ver(),1,100.15,4);valid.goal_yaw_checked=true;valid.checked_goal_yaw=p.goal_yaw;
 o=body(100.15,.05,0.);o.angular_velocity_in_frame.z()=.20;
 ASSERT_TRUE(f.body(o,100.15,10.15));ASSERT_TRUE(f.validation(valid));ASSERT_TRUE(f.lease(lease(100.15,3),100.15));
 EXPECT_EQ(f.geometryPermit(100.15).phase,"aligning");const auto resumed=f.step(100.15,10.15);
 EXPECT_GT(resumed.velocity.angular.z,0.);EXPECT_FALSE(resumed.hold);
 EXPECT_EQ(f.activeVersion()->task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, AuthorBodyXYLawAndMovingReplacementKeepTypedTaskAndLimiterHistory) {
 auto c=cfg();c.align_then_forward=false;c.holonomic_yaw_policy="path_tangent";
 c.kp_position=.8;c.kp_yaw=1.5;c.heading_threshold=.8;
 c.max_acceleration=.35;c.max_yaw_acceleration=.8;
 c.velocity_damping_gain=c.yaw_damping_gain=c.max_jerk=c.max_yaw_jerk=0.;
 LocalFollower f(c,"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 auto active=ver();std::int64_t id=1;Eigen::Vector2d previous=Eigen::Vector2d::Zero();
 double previous_yaw=0.;
 for(unsigned i=1;i<=30;++i) {
   const double now=100.+i*.05;auto o=body(now,.25*i*.05);
   o.yaw=-.2;o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
   o.angular_velocity_in_frame.z()=.3; // No predicted coast in the selected profile.
   ASSERT_TRUE(f.body(o,now,now-90.));
   if(i==10) {
     auto next=ver(2);auto p=proposal(next,now);p.expected_version=active;p.expected_trajectory_id=id;
     ASSERT_TRUE(f.proposal(p,now));ASSERT_TRUE(f.support(support(next)));
     auto replacement=tag(next,2,o.position.x(),now);replacement.join_pose.orientation.z=o.orientation.z();
     replacement.join_pose.orientation.w=o.orientation.w();
     ASSERT_TRUE(f.candidate(replacement));ASSERT_TRUE(f.validation(proof(next,2,now,i+1)));
     ASSERT_TRUE(f.tryCommit(now,now-90.))<<f.candidateReason();active=next;id=2;
   }else ASSERT_TRUE(f.validation(proof(active,id,now,i+1)));
   ASSERT_TRUE(f.lease(lease(now,i+1),now));ASSERT_TRUE(f.geometryPermit(now).allowed);
   const auto demand=f.step(now,now-90.);const auto status=f.status(now);
   const Eigen::Vector2d command(demand.velocity.linear.x,demand.velocity.linear.y);
   EXPECT_GT(command.x(),0.);EXPECT_GT(command.y(),0.);EXPECT_GT(demand.velocity.angular.z,0.);
   EXPECT_EQ(demand.reason,"tracking");EXPECT_FALSE(demand.hold);
   EXPECT_LE((command-previous).norm(),c.max_acceleration*.05+1e-12);
   EXPECT_LE(std::abs(demand.velocity.angular.z-previous_yaw),c.max_yaw_acceleration*.05+1e-12);
   EXPECT_TRUE(status.ready);EXPECT_FALSE(status.holding);EXPECT_EQ(status.phase,"following");
   EXPECT_EQ(status.version.task_id,ver().task_id);EXPECT_FALSE(status.request_stop);
   previous=command;previous_yaw=demand.velocity.angular.z;
 }
 EXPECT_EQ(f.commitSequence(),2u);EXPECT_EQ(f.controller().trajectoryId(),2);
 // An actual newest negative proof still stops all axes immediately, without
 // replacing/clearing the committed task or inventing a local re-grant.
 auto blocked=proof(active,id,101.55,40);blocked.valid=false;ASSERT_TRUE(f.validation(blocked));
 ASSERT_TRUE(f.body(body(101.55,.3875),101.55,11.55));ASSERT_TRUE(f.lease(lease(101.55,40),101.55));
 const auto zero=f.step(101.55,11.55);EXPECT_TRUE(zero.hold);
 EXPECT_DOUBLE_EQ(zero.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(zero.velocity.linear.y,0.);
 EXPECT_DOUBLE_EQ(zero.velocity.angular.z,0.);EXPECT_TRUE(f.controller().hasInstalledGeometry());
 EXPECT_EQ(f.commitSequence(),2u);
 f.retire("operator_cancel");EXPECT_TRUE(f.status(101.55).software_retired);
}
TEST(LocalFollower, FormalLowSpeedHeadingSeedIsIndependentOfActualJoinAndRequiredOnReplacement) {
 auto c=cfg();c.align_then_forward=false;c.require_body_heading_seed=true;
 c.kp_position=.8;c.kp_yaw=1.5;c.heading_threshold=.8;c.max_acceleration=.35;c.max_yaw_acceleration=.8;
 c.velocity_damping_gain=c.yaw_damping_gain=c.max_jerk=c.max_yaw_jerk=0.;
 LocalFollower f(c,"isolated_mock",{},false);
 auto measured=body(100.,0.,.01);measured.yaw=1.2;
 measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
 auto candidate=tag();for(auto& knot:candidate.trajectory.knots)knot*=25.;
 candidate.join_twist.linear.x=.01;candidate.join_pose.orientation.z=measured.orientation.z();
 candidate.join_pose.orientation.w=measured.orientation.w();candidate.body_heading_seed_valid=true;
 candidate.body_heading_seed_yaw=1.2;candidate.body_heading_sampling_intervals=5000;
 auto whole=proof();whole.checked_to_time=whole.curve_duration=100.;
 ASSERT_TRUE(f.body(measured,100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));ASSERT_TRUE(f.support(support()));
 ASSERT_TRUE(f.candidate(candidate));ASSERT_TRUE(f.validation(whole));ASSERT_TRUE(f.tryCommit(100.,10.));
 ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.05);const auto first=f.step(100.05,10.05);
 EXPECT_GT(std::hypot(first.velocity.linear.x,first.velocity.linear.y),0.);EXPECT_DOUBLE_EQ(first.velocity.angular.z,0.);
 auto v=ver(2);auto p=proposal(v,100.1);p.expected_version=ver();p.expected_trajectory_id=1;
 ASSERT_TRUE(f.proposal(p,100.1));ASSERT_TRUE(f.support(support(v)));
 auto next=tag(v,2,0.,100.1);for(auto& knot:next.trajectory.knots)knot*=25.;
 next.join_twist.linear.x=.01;next.join_pose.orientation=candidate.join_pose.orientation;
 next.body_heading_seed_valid=true;next.body_heading_seed_yaw=-.4;next.body_heading_sampling_intervals=5000;
 measured.stamp=measured.posterior_stamp=measured.imu_stamp=100.1;measured.source_stamp_ns=timeNs(stamp(100.1));
 measured.provider_valid_until_ns=measured.source_stamp_ns+100000000LL;
 measured.provider_posterior_stamp_ns=measured.provider_imu_stamp_ns=measured.source_stamp_ns;
 ASSERT_TRUE(f.body(measured,100.1,10.1));ASSERT_TRUE(f.candidate(next));
 whole=proof(v,2,100.1,2);whole.checked_to_time=whole.curve_duration=100.;
 ASSERT_TRUE(f.validation(whole));ASSERT_TRUE(f.tryCommit(100.1,10.1))<<f.candidateReason();
 ASSERT_TRUE(f.lease(lease(100.1,2),100.1));ASSERT_TRUE(f.geometryPermit(100.1).allowed);
 const auto out=f.step(100.1,10.1);EXPECT_EQ(out.reason,"heading_alignment");EXPECT_LT(out.velocity.angular.z,0.);
 EXPECT_NEAR(f.controller().controlDiagnostic().heading_error,-1.6,1e-12);
 EXPECT_TRUE(f.status(100.1).ready);EXPECT_FALSE(f.status(100.1).holding);EXPECT_EQ(f.commitSequence(),2u);
 auto missing=next;missing.trajectory.traj_id=3;missing.body_heading_seed_valid=false;missing.body_heading_sampling_intervals=0;
 ASSERT_TRUE(f.candidate(missing));whole.trajectory_id=3;whole.sequence=3;ASSERT_TRUE(f.validation(whole));
 EXPECT_FALSE(f.tryCommit(100.1,10.1));EXPECT_EQ(f.controller().trajectoryId(),2);EXPECT_EQ(f.commitSequence(),2u);
 EXPECT_EQ(f.candidateReason(),"body_heading_seed_missing_or_invalid");
}
TEST(LocalFollower, HighSpeedGoalCrossingKeepsTranslationBeforeYawAlignment) {
 auto c=cfg();c.goal_tolerance=.02;LocalFollower f(c,"isolated_mock",{},false);
 auto p=proposal();p.goal_position.x=.05;p.has_goal_yaw=true;p.goal_yaw=1.;p.goal_yaw_tolerance_rad=.1;
 stage(f,p);ASSERT_TRUE(f.lease(lease(),100.));
 ASSERT_TRUE(f.body(body(100.01,.05,.25),100.01,10.01));
 EXPECT_NE(f.geometryPermit(100.01).phase,"aligning");
 EXPECT_NE(f.step(100.01,10.01).reason,"aligning");
 EXPECT_FALSE(f.status(100.01).request_stop);
 ASSERT_TRUE(f.body(body(100.02,.05,c.stationary_linear_threshold_mps),100.02,10.02));
 auto yaw_proof=proof(ver(),1,100.02,2);yaw_proof.goal_yaw_checked=true;yaw_proof.checked_goal_yaw=p.goal_yaw;
 ASSERT_TRUE(f.validation(yaw_proof)); // Explicit terminal rotation, not just the path, has Native evidence.
 EXPECT_EQ(f.geometryPermit(100.02).phase,"aligning");
 EXPECT_EQ(f.step(100.02,10.02).reason,"aligning");
 const auto s=f.status(100.02);EXPECT_EQ(s.phase,"aligning");EXPECT_FALSE(s.at_goal);EXPECT_FALSE(s.request_stop);
}
TEST(LocalFollower, ExplicitFarGoalYawDoesNotGateOrdinaryContinuousTrackingWithoutYawProof) {
 auto c=cfg();c.align_then_forward=true;LocalFollower f(c,"isolated_mock",{},false);
 auto p=proposal();p.has_goal_yaw=true;p.goal_yaw=1.;p.goal_yaw_tolerance_rad=.07;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.body(body(100.01),100.01,10.01));
 EXPECT_EQ(f.geometryPermit(100.01).phase,"tracking");
 const auto demand=f.step(100.01,10.01);EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_FALSE(demand.hold);
 EXPECT_EQ(demand.reason,"tracking_forward");const auto status=f.status(100.01);
 EXPECT_TRUE(status.ready);EXPECT_FALSE(status.holding);EXPECT_EQ(status.phase,"following");
 EXPECT_FALSE(status.request_stop);EXPECT_FALSE(status.software_retired);
}
TEST(LocalFollower, TerminalYawConsumesLatestNativeProofAndResumesSameTaskAfterRollingHold) {
 auto c=cfg();c.align_then_forward=true;LocalFollower f(c,"isolated_mock",{},false);
 auto p=proposal();p.goal_position.x=.05;p.has_goal_yaw=true;p.goal_yaw=1.;p.goal_yaw_tolerance_rad=.07;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.body(body(100.01,.05,0.),100.01,10.01));
 EXPECT_NE(f.geometryPermit(100.01).phase,"aligning");
 auto demand=f.step(100.01,10.01);EXPECT_TRUE(demand.hold);EXPECT_EQ(demand.reason,"waiting_native_goal_yaw_proof");
 auto status=f.status(100.01);EXPECT_TRUE(status.ready);EXPECT_TRUE(status.holding);
 EXPECT_EQ(status.phase,"waiting_goal_yaw_proof");EXPECT_FALSE(status.at_goal);EXPECT_FALSE(status.request_stop);
 EXPECT_TRUE(f.controller().active());EXPECT_TRUE(f.controller().hasInstalledGeometry());
 const auto progress=trackingProgress(f.controller(),c,100.01,status);EXPECT_TRUE(progress.valid);EXPECT_TRUE(progress.holding);
 // Proof delivery stays independent of an allowed motion lease. A newer
 // body than this fresh proof must not introduce an exact-stamp deadlock.
 auto held=lease(100.02,2);held.allowed=false;held.phase="holding";ASSERT_TRUE(f.lease(held,100.02));
 auto good=proof(ver(),1,100.02,2);good.goal_yaw_checked=true;good.checked_goal_yaw=p.goal_yaw+2.*M_PI;
 ASSERT_TRUE(f.validation(good));ASSERT_TRUE(f.body(body(100.02,.05,0.),100.02,10.02));
 EXPECT_FALSE(f.geometryPermit(100.02).allowed);EXPECT_TRUE(f.step(100.02,10.02).hold);
 status=f.status(100.02);EXPECT_TRUE(status.holding);EXPECT_EQ(status.phase,"waiting_task_authority");
 EXPECT_FALSE(status.at_goal);EXPECT_FALSE(status.request_stop);
 ASSERT_TRUE(f.lease(lease(100.03,3),100.03));ASSERT_TRUE(f.body(body(100.03,.05,0.),100.03,10.03));
 EXPECT_EQ(f.geometryPermit(100.03).phase,"aligning");demand=f.step(100.03,10.03);
 EXPECT_GT(demand.velocity.angular.z,0.);EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);
 EXPECT_EQ(demand.reason,"aligning");EXPECT_EQ(f.status(100.03).phase,"aligning");
 for(unsigned i=4;i<=6;++i) {
   const double now=100.+i*.01;auto bad=proof(ver(),1,now,i-1);
   bad.goal_yaw_checked=i!=4;bad.checked_goal_yaw=i==5?-1.:std::numeric_limits<double>::quiet_NaN();
   ASSERT_TRUE(f.validation(bad));ASSERT_TRUE(f.body(body(now,.05,0.),now,now-90.));
   EXPECT_NE(f.geometryPermit(now).phase,"aligning");demand=f.step(now,now-90.);
   EXPECT_TRUE(demand.hold);EXPECT_DOUBLE_EQ(demand.velocity.angular.z,0.);
   EXPECT_EQ(demand.reason,"waiting_native_goal_yaw_proof");EXPECT_TRUE(f.status(now).holding);
   EXPECT_FALSE(f.validation(good)); // No fallback to a superseded positive target proof.
   EXPECT_TRUE(f.controller().active());EXPECT_EQ(f.commitSequence(),1u);
 }
 good=proof(ver(),1,100.07,6);good.goal_yaw_checked=true;good.checked_goal_yaw=p.goal_yaw-2.*M_PI;
 ASSERT_TRUE(f.validation(good));ASSERT_TRUE(f.body(body(100.07,.05,0.),100.07,10.07));
 EXPECT_EQ(f.geometryPermit(100.07).phase,"aligning");demand=f.step(100.07,10.07);
 EXPECT_GT(demand.velocity.angular.z,0.);EXPECT_FALSE(f.status(100.07).holding);
 EXPECT_EQ(f.activeVersion()->task_id,ver().task_id);EXPECT_EQ(f.commitSequence(),1u);
 EXPECT_FALSE(f.status(100.07).request_stop);EXPECT_FALSE(f.status(100.07).software_retired);
}
TEST(LocalFollower, GoalPoseAtHighMeasuredAngularSpeedSettlesWithoutRequestingSdkStop) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
 p.has_goal_yaw=true;p.goal_yaw=0.;p.goal_yaw_tolerance_rad=.1;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));auto o=body(100.01,.05,0.);
 o.angular_velocity_in_frame.z()=-.051;ASSERT_TRUE(f.body(o,100.01,10.01));
 EXPECT_EQ(f.geometryPermit(100.01).phase,"tracking");
 const auto s=f.status(100.01);EXPECT_TRUE(s.ready);EXPECT_FALSE(s.at_goal);
 EXPECT_FALSE(s.request_stop);EXPECT_EQ(s.phase,"settling");
}
TEST(LocalFollower, LowMeasuredSpeedAtGoalPoseRequestsStopWithoutAnotherConfirmationTimer) {
 const auto c=cfg();LocalFollower f(c,"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;
 p.has_goal_yaw=true;p.goal_yaw=0.;p.goal_yaw_tolerance_rad=.1;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));auto o=body(100.01,.05,c.stationary_linear_threshold_mps);
 o.angular_velocity_in_frame.z()=c.stationary_angular_threshold_radps;
 ASSERT_TRUE(f.body(o,100.01,10.01));
 EXPECT_EQ(f.geometryPermit(100.01).phase,"tracking");
 const auto s=f.status(100.01);EXPECT_TRUE(s.ready);EXPECT_TRUE(s.at_goal);EXPECT_TRUE(s.request_stop);
 EXPECT_EQ(s.phase,"goal_reached");EXPECT_FALSE(s.software_retired);
}
TEST(LocalFollower, MissingGoalStateCannotRequestStopAndFreshStateRestoresSameTask) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.body(body(100.01,.05,0.),100.01,10.01));
 ASSERT_TRUE(f.status(100.01).at_goal);
 ASSERT_TRUE(f.validation(proof(ver(),1,100.12,2)));f.geometryPermit(100.12);
 EXPECT_TRUE(f.step(100.12,10.12).hold);const auto missing=f.status(100.12);
 EXPECT_FALSE(missing.ready);EXPECT_FALSE(missing.at_goal);EXPECT_FALSE(missing.request_stop);
 EXPECT_FALSE(missing.software_retired);EXPECT_TRUE(f.controller().active());
 EXPECT_EQ(timeNs(missing.body_source_stamp),100010000000LL);
 EXPECT_EQ(timeNs(missing.body_valid_until),100110000000LL);
 ASSERT_TRUE(f.body(body(100.15,.05,0.),100.15,10.15));
 ASSERT_TRUE(f.validation(proof(ver(),1,100.15,3)));f.geometryPermit(100.15);
 f.step(100.15,10.15);const auto restored=f.status(100.15);EXPECT_TRUE(restored.ready);EXPECT_FALSE(restored.at_goal);
 EXPECT_FALSE(restored.request_stop);EXPECT_FALSE(restored.software_retired);
 EXPECT_EQ(restored.version.task_id,"task");EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 for(unsigned i=1;i<=12;++i) {
   const double now=100.15+i*.05;ASSERT_TRUE(f.body(body(now,.05,0.),now,now-90.));
   ASSERT_TRUE(f.validation(proof(ver(),1,now,i+3)));ASSERT_TRUE(f.lease(lease(now,i+1),now));
   f.geometryPermit(now);f.step(now,now-90.);
 }
 EXPECT_TRUE(f.status(100.75).at_goal);EXPECT_TRUE(f.status(100.75).request_stop);
}
TEST(LocalFollower, FreshProviderFaultCannotCompleteCachedGoalAndExistingHoldRecoveryKeepsTask) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.body(body(100.01,.05,0.),100.01,10.01));
 ASSERT_TRUE(f.status(100.01).at_goal);
 auto fault=body(100.02,.05,0.);fault.provider_usable=false;EXPECT_FALSE(f.body(fault,100.02,10.02));
 const auto held=f.status(100.02);EXPECT_TRUE(held.ready);EXPECT_TRUE(held.holding);
 EXPECT_FALSE(held.at_goal);EXPECT_FALSE(held.request_stop);EXPECT_FALSE(held.software_retired);
 EXPECT_EQ(timeNs(held.body_source_stamp),100010000000LL);
 for(unsigned i=1;i<=13;++i) {
   const double now=100.05+(i-1)*.05;ASSERT_TRUE(f.body(body(now,.05,0.),now,now-90.));
   ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));ASSERT_TRUE(f.lease(lease(now,i+1),now));
   f.geometryPermit(now);f.step(now,now-90.);const auto status=f.status(now);
   if(i<13){EXPECT_FALSE(status.at_goal);EXPECT_FALSE(status.request_stop);}
   EXPECT_FALSE(status.software_retired);EXPECT_EQ(status.version.task_id,"task");
 }
 EXPECT_TRUE(f.status(100.65).at_goal);EXPECT_TRUE(f.status(100.65).request_stop);
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
}
TEST(LocalFollower, LocalEndpointWaitExposesHoldingForExistingReplanWithoutRetiringTask) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);double now=100.,x=0.,v=.25;unsigned sample=0;
 bool reached=false;
 for(unsigned i=1;i<=500;i++) {
   now=100.+i*.05;sample=i;
   ASSERT_TRUE(f.body(body(now,x,v),now,10.+i*.05));
   ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));ASSERT_TRUE(f.lease(lease(now,i),now));
   f.geometryPermit(now);const auto d=f.step(now,10.+i*.05);v=d.velocity.linear.x;x+=v*.05;
   if(d.reason=="local_segment_finished_waiting_replan"){reached=true;break;}
 }
 ASSERT_TRUE(reached)<<f.controller().reason();
 ASSERT_EQ(f.controller().reason(),"local_segment_finished_waiting_replan");
 EXPECT_FALSE(f.controller().holding());const auto s=f.status(now);
 EXPECT_TRUE(s.ready);EXPECT_TRUE(s.holding);EXPECT_FALSE(s.at_goal);EXPECT_FALSE(s.request_stop);
 EXPECT_FALSE(s.software_retired);EXPECT_EQ(s.phase,"waiting_replan");
 EXPECT_EQ(f.geometryPermit(now).phase,"tracking");
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 // A newer negative proof still holds output, but cannot hide the measured
 // endpoint wait from the existing Native replan consumer.
 now+=.05;ASSERT_TRUE(f.body(body(now,x,v),now,now-90.));
 auto negative=proof(ver(),1,now,sample+2);negative.valid=false;ASSERT_TRUE(f.validation(negative));
 ASSERT_TRUE(f.lease(lease(now,sample+1),now));f.geometryPermit(now);
 const auto d=f.step(now,now-90.);EXPECT_TRUE(d.hold);
 EXPECT_EQ(d.reason,"local_segment_finished_waiting_replan");
 const auto waiting=f.status(now);EXPECT_FALSE(waiting.ready);EXPECT_TRUE(waiting.holding);
 EXPECT_FALSE(waiting.request_stop);EXPECT_FALSE(waiting.software_retired);EXPECT_EQ(waiting.phase,"waiting_replan");
 const auto progress=trackingProgress(f.controller(),cfg(),now,waiting);
 EXPECT_TRUE(progress.valid);EXPECT_TRUE(progress.holding);EXPECT_EQ(progress.reason,d.reason);
}
TEST(LocalFollower, DurableRetirementBeforeQueuedProposalFencesOldGeneration) {
 LocalFollower f(cfg(),"isolated_mock",{},false);wire::RouteDeliveryReceipt r;
 r.schema_version=1;r.session_id="session";r.task_id="task";r.route_id="route";r.route_hash=std::string(64,'a');
 r.delivery_sequence=2;r.source_stamp=stamp(100.01);r.receipt_stamp=stamp(100.02);
 r.status=wire::RouteDeliveryReceipt::RETIRED;r.native_retirement_required=r.native_retirement_confirmed=true;r.native_generation=2;
 ASSERT_TRUE(f.retireRoute(r,101.));EXPECT_FALSE(f.proposal(proposal(),101.));EXPECT_FALSE(f.candidate(tag()));
 auto p=proposal(ver(3),101.);p.version.task_id="next";ASSERT_TRUE(f.proposal(p,101.));
 EXPECT_FALSE(f.retireRoute(r,101.));EXPECT_FALSE(f.candidate(tag()));
}
TEST(LocalFollower, PreviewGeometryReadyDoesNotClaimExecutionOrCompleteAtGoal) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;stage(f,p);
 ASSERT_TRUE(f.body(body(100.01,.05,0.),100.01,10.01));
 EXPECT_FALSE(f.geometryPermit(100.01).allowed);EXPECT_TRUE(f.step(100.01,10.01).hold);
 const auto preview=f.status(100.01);
 EXPECT_TRUE(preview.ready);EXPECT_FALSE(preview.holding); // Initial BT admission, not output.
 EXPECT_EQ(preview.phase,"preview");EXPECT_EQ(preview.reason,"local_geometry_preview");
 EXPECT_FALSE(preview.at_goal);EXPECT_FALSE(preview.request_stop);
 EXPECT_TRUE(preview.execution_id.empty());EXPECT_FALSE(preview.software_retired);
 auto arming=lease(100.02);arming.allowed=false;arming.phase="holding";arming.sdk_arm_generation=0;
 ASSERT_TRUE(f.lease(arming,100.02));ASSERT_TRUE(f.body(body(100.02,.05,0.),100.02,10.02));
 EXPECT_FALSE(f.geometryPermit(100.02).allowed);EXPECT_TRUE(f.step(100.02,10.02).hold);
 const auto preparing=f.status(100.02);EXPECT_TRUE(preparing.ready);EXPECT_FALSE(preparing.holding);
 EXPECT_EQ(preparing.phase,"preview");EXPECT_FALSE(preparing.request_stop);
 ASSERT_TRUE(f.lease(lease(100.03,2),100.03));ASSERT_TRUE(f.body(body(100.03,.05,0.),100.03,10.03));
 EXPECT_TRUE(f.geometryPermit(100.03).allowed);f.step(100.03,10.03);
 const auto goal=f.status(100.03);EXPECT_TRUE(goal.at_goal);EXPECT_TRUE(goal.request_stop);
 EXPECT_EQ(goal.phase,"goal_reached");
}
TEST(LocalFollower, ExpiredTaskLeaseReportsHoldWithFreshGeometryAndRecoversSameCurveImmediately) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));f.geometryPermit(100.05);
 ASSERT_FALSE(f.step(100.05,10.05).hold);ASSERT_EQ(f.status(100.05).phase,"following");
 ASSERT_TRUE(f.validation(proof(ver(),1,100.34,2)));
 ASSERT_TRUE(f.body(body(100.35,.0875),100.35,10.35));
 EXPECT_FALSE(f.geometryPermit(100.35).allowed);const auto demand=f.step(100.35,10.35);
 EXPECT_TRUE(demand.hold);EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);
 EXPECT_EQ(demand.reason,"waiting_task_authority");
 const auto held=f.status(100.35);EXPECT_TRUE(held.ready);EXPECT_TRUE(held.holding);
 EXPECT_EQ(held.phase,"waiting_task_authority");EXPECT_EQ(held.reason,demand.reason);
 EXPECT_FALSE(held.request_stop);EXPECT_FALSE(held.software_retired);
 EXPECT_EQ(held.version,ver());EXPECT_EQ(held.commit_sequence,1u);EXPECT_EQ(held.trajectory_id,1);
 EXPECT_EQ(timeNs(held.body_source_stamp),100350000000LL);
 EXPECT_EQ(timeNs(held.body_valid_until),100450000000LL);
 EXPECT_TRUE(trackingProgress(f.controller(),cfg(),100.35,held).holding);
 // An old sequence and a wrong task cannot reauthorize the retained curve.
 EXPECT_FALSE(f.lease(lease(100.36,1),100.36));auto wrong=lease(100.36,2);wrong.version.task_id="other";
 EXPECT_FALSE(f.lease(wrong,100.36));EXPECT_TRUE(f.status(100.36).holding);
 ASSERT_TRUE(f.lease(lease(100.36,2),100.36));ASSERT_TRUE(f.body(body(100.36,.09),100.36,10.36));
 EXPECT_TRUE(f.geometryPermit(100.36).allowed);EXPECT_FALSE(f.step(100.36,10.36).hold);
 const auto resumed=f.status(100.36);EXPECT_TRUE(resumed.ready);EXPECT_FALSE(resumed.holding);
 EXPECT_EQ(resumed.phase,"following");EXPECT_EQ(resumed.commit_sequence,1u);EXPECT_EQ(resumed.trajectory_id,1);
}
TEST(LocalFollower, WithheldLeaseIsNotPreviewAfterTaskWasAuthorized) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 ASSERT_TRUE(f.body(body(100.05,.0125),100.05,10.05));f.geometryPermit(100.05);ASSERT_FALSE(f.step(100.05,10.05).hold);
 auto withheld=lease(100.06,2);withheld.allowed=false;withheld.phase="holding";
 ASSERT_TRUE(f.lease(withheld,100.06));ASSERT_TRUE(f.body(body(100.06,.015),100.06,10.06));
 f.geometryPermit(100.06);EXPECT_TRUE(f.step(100.06,10.06).hold);
 auto status=f.status(100.06);EXPECT_TRUE(status.ready);EXPECT_TRUE(status.holding);
 EXPECT_EQ(status.phase,"waiting_task_authority");EXPECT_FALSE(status.at_goal);EXPECT_FALSE(status.request_stop);
 // A bad replacement is still only a candidate failure, not task retirement.
 auto bad=tag(ver(),2,.015,100.06);bad.trajectory.knots.clear();ASSERT_TRUE(f.candidate(bad));
 ASSERT_TRUE(f.validation(proof(ver(),2,100.06,2)));EXPECT_FALSE(f.tryCommit(100.06,10.06));
 status=f.status(100.06);EXPECT_TRUE(status.holding);EXPECT_EQ(status.phase,"waiting_task_authority");
 EXPECT_EQ(status.trajectory_id,1);EXPECT_FALSE(status.software_retired);
 ASSERT_TRUE(f.lease(lease(100.07,3),100.07));ASSERT_TRUE(f.body(body(100.07,.0175),100.07,10.07));
 f.geometryPermit(100.07);EXPECT_FALSE(f.step(100.07,10.07).hold);
 EXPECT_FALSE(f.status(100.07).holding);EXPECT_EQ(f.status(100.07).phase,"following");
}
TEST(LocalFollower, ExpiredAuthorityCannotReportGoalCompletionFromCachedAllowedFlag) {
 LocalFollower f(cfg(),"isolated_mock",{},false);auto p=proposal();p.goal_position.x=.05;stage(f,p);
 ASSERT_TRUE(f.lease(lease(),100.));ASSERT_TRUE(f.validation(proof(ver(),1,100.34,2)));
 ASSERT_TRUE(f.body(body(100.35,.05,0.),100.35,10.35));f.geometryPermit(100.35);
 EXPECT_TRUE(f.step(100.35,10.35).hold);const auto held=f.status(100.35);
 EXPECT_TRUE(held.ready);EXPECT_TRUE(held.holding);EXPECT_EQ(held.phase,"waiting_task_authority");
 EXPECT_FALSE(held.at_goal);EXPECT_FALSE(held.request_stop);
 ASSERT_TRUE(f.lease(lease(100.36,2),100.36));f.geometryPermit(100.36);f.step(100.36,10.36);
 EXPECT_TRUE(f.status(100.36).at_goal);EXPECT_TRUE(f.status(100.36).request_stop);
}
TEST(LocalFollower, RevocationRemainsTerminalAndNewTaskStartsAsUnexecutedPreview) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);ASSERT_TRUE(f.lease(lease(),100.));
 f.geometryPermit(100.05);f.step(100.05,10.05);
 auto revoked=lease(100.06,2);revoked.allowed=false;revoked.revoked=true;revoked.phase="stopping";
 ASSERT_TRUE(f.lease(revoked,100.06));f.geometryPermit(100.06);EXPECT_TRUE(f.step(100.06,10.06).hold);
 const auto stopped=f.status(100.06);EXPECT_TRUE(stopped.holding);EXPECT_TRUE(stopped.request_stop);
 EXPECT_TRUE(stopped.software_retired);EXPECT_EQ(stopped.phase,"retired");EXPECT_FALSE(stopped.at_goal);
 EXPECT_FALSE(f.proposal(proposal(ver(),100.07),100.07));
 auto next=ver(2);next.task_id="next_task";next.route_id="next_route";next.route_hash=std::string(64,'d');
 ASSERT_TRUE(f.body(body(100.08,0.),100.08,10.08));ASSERT_TRUE(f.proposal(proposal(next,100.08),100.08));
 ASSERT_TRUE(f.support(support(next)));ASSERT_TRUE(f.candidate(tag(next,2,0.,100.08)));
 ASSERT_TRUE(f.validation(proof(next,2,100.08,3)));ASSERT_TRUE(f.tryCommit(100.08,10.08));
 EXPECT_FALSE(f.geometryPermit(100.08).allowed);EXPECT_TRUE(f.step(100.08,10.08).hold);
 const auto preview=f.status(100.08);EXPECT_TRUE(preview.ready);EXPECT_FALSE(preview.holding);
 EXPECT_EQ(preview.phase,"preview");EXPECT_TRUE(preview.execution_id.empty());EXPECT_FALSE(preview.request_stop);
 EXPECT_FALSE(preview.software_retired);EXPECT_EQ(preview.version,next);
 EXPECT_FALSE(f.lease(lease(100.09,3),100.09)); // Late old task cannot revive authorization.
 auto admitted=lease(100.09,1);admitted.version=next;admitted.execution_id="new_execution";
 ASSERT_TRUE(f.lease(admitted,100.09));ASSERT_TRUE(f.body(body(100.09,.0025),100.09,10.09));
 EXPECT_TRUE(f.geometryPermit(100.09).allowed);EXPECT_FALSE(f.step(100.09,10.09).hold);
 EXPECT_EQ(f.status(100.09).phase,"following");EXPECT_EQ(f.status(100.09).execution_id,"new_execution");
}
TEST(LocalFollower, ControlDrivenMovingBodyKeepsProgressAndBoundedCommands) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stage(f);
 double x=0.,vx=.25,last=0.;
 for(unsigned i=1;i<=60;i++) {
   const double t=100.+i*.05;ASSERT_TRUE(f.body(body(t,x,vx),t,10.+i*.05));
   ASSERT_TRUE(f.validation(proof(ver(),1,t,i+1)));ASSERT_TRUE(f.lease(lease(t,i),t));
   f.geometryPermit(t);const auto d=f.step(t,10.+i*.05);
   EXPECT_GE(d.velocity.linear.x,0.);EXPECT_LE(std::hypot(d.velocity.linear.x,d.velocity.linear.y),.3+1e-9);
   EXPECT_LE(std::abs(d.velocity.linear.x-last),.35*.05+1e-8);
   last=vx=d.velocity.linear.x;x+=vx*.05;
   EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);
 }
 EXPECT_GT(x,.2);EXPECT_GT(f.controller().committedArc(),.2);
}
TEST(LocalFollower, ZeroStartControlDrivenPlantReachesPositionWithoutTimerProgress) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 auto t=tag();for(std::size_t i=0;i<t.trajectory.pos_pts.size();i++)t.trajectory.pos_pts[i].x=i<3?0.:.001*(i-2)*(i-2);
 t.join_twist.linear.x=0.;auto p=proposal();p.goal_position.x=.25;
 ASSERT_TRUE(f.body(body(100.,0.,0.),100.,10.));ASSERT_TRUE(f.proposal(p,100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(t));ASSERT_TRUE(f.validation(proof()));
 ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();double x=0.,v=0.;bool reached=false;
 for(unsigned i=1;i<=300;i++) {
   const double now=100.+i*.05;ASSERT_TRUE(f.body(body(now,x,v),now,10.+i*.05));
   ASSERT_TRUE(f.validation(proof(ver(),1,now,i+1)));ASSERT_TRUE(f.lease(lease(now,i),now));
   f.geometryPermit(now);const auto d=f.step(now,10.+i*.05);
   if(d.reason=="goal_position_reached") {
     // The original fresh measured input has already met both arrival/rest
     // predicates: its completion demand is exact zero, not a slew tail.
     EXPECT_LE(std::abs(p.goal_position.x-x),cfg().goal_tolerance);
     EXPECT_LE(std::abs(v),cfg().stationary_linear_threshold_mps);
     EXPECT_TRUE(f.status(now).at_goal);EXPECT_TRUE(d.hold);
     EXPECT_DOUBLE_EQ(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.linear.y,0.);
     EXPECT_DOUBLE_EQ(d.velocity.angular.z,0.);
   } else EXPECT_LE(std::abs(d.velocity.linear.x-v),.35*.05+1e-8);
   EXPECT_LE(std::hypot(d.velocity.linear.x,d.velocity.linear.y),.3+1e-9);
   v=d.velocity.linear.x;x+=v*.05;
   if(f.status(now).at_goal&&std::abs(v)<1e-9){reached=true;break;}
 }
 EXPECT_TRUE(reached);EXPECT_GT(x,.05);EXPECT_GT(f.controller().committedArc(),.03);
}

TEST(LocalFollower, NativeGeometryExchangeAcceptsRealOffsetAndVelocityWithoutInventingState) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 auto measured=body(100.,.04,.10);measured.position.y()=.08;
 ASSERT_TRUE(f.body(measured,100.,10.));ASSERT_TRUE(f.proposal(proposal(),100.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag()));ASSERT_TRUE(f.validation(proof()));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);ASSERT_EQ(request->disposition,"prepare");
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.,10.));
 const auto commit=f.exchangeCommit(100.,10.);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed");
 EXPECT_NEAR(commit->curve_time,.16,1e-9);EXPECT_NEAR(commit->arc_length,.04,1e-9);
 // Native receives the actual connector start and actual velocity, not a
 // pose snapped onto the spline nor its .25 m/s derivative as fake odometry.
 EXPECT_NEAR(commit->measured_pose.position.x,.04,1e-12);
 EXPECT_NEAR(commit->measured_pose.position.y,.08,1e-12);
 EXPECT_NEAR(commit->measured_twist.linear.x,.10,1e-12);
 EXPECT_EQ(commit->body_source_stamp,stamp(100.));
 ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,100.,10.));
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_TRUE(f.status(100.).ready);
 EXPECT_NEAR(f.controller().odometry().position.y(),.08,1e-12);
 EXPECT_NEAR(f.controller().odometry().velocity_in_frame.x(),.10,1e-12);
}

TEST(LocalFollower, NativeGeometryCannotUseDistantWrongHeightOrRatchetedCandidateDomain) {
 for(unsigned kind=0;kind<3;++kind) {
  SCOPED_TRACE(kind);LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  auto measured=body(100.05,.0125,.10);
  if(kind==0)measured.position.y()=.151;
  if(kind==1)measured.position.z()+=.151;
  if(kind==2)measured.position.x()=.60;
  ASSERT_TRUE(f.body(measured,100.05,10.05));
  ASSERT_TRUE(f.validation(proof(ver(),1,100.05,2)));
  ASSERT_TRUE(f.candidate(tag(ver(),2,0.,100.)));
  auto current=proof(ver(),2,100.05,3);
  if(kind==2) {current.valid_start_time=2.4;current.valid_start_arc_length=.60;}
  ASSERT_TRUE(f.validation(current));
  const auto request=f.exchangeCommit(100.05,10.05);ASSERT_TRUE(request);
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.05,10.05));
  const auto rejected=f.exchangeCommit(100.05,10.05);
  ASSERT_TRUE(rejected);EXPECT_EQ(rejected->disposition,"abort");
  EXPECT_EQ(rejected->reason,"entry_measured_body_not_on_candidate");
  EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);
 }
}

TEST(LocalFollower, GeometryFollowingDoesNotChangeStrictLegacyAdmission) {
 auto original=decodeLocalCurve(tag());auto cache=EntryCurveCache::build(original);ASSERT_TRUE(cache);
 auto legacy=cfg();legacy.validate();
 EXPECT_EQ(legacy.geometry_authority,GeometryAuthority::LegacyTracker);
 EXPECT_EQ(legacy.admissionPolicy(),CurveAdmissionPolicy::StrictState);
 EXPECT_DOUBLE_EQ(legacy.admissionPositionLimit(),.0125);
 std::string reason;auto measured=body();measured.position.y()=.08;
 EXPECT_FALSE(cache->observe(original,measured,legacy,100.,100.,0.,0.,reason));
 EXPECT_EQ(reason,"entry_measured_body_not_on_candidate");
 measured=body(100.,0.,.10);
 EXPECT_FALSE(cache->observe(original,measured,legacy,100.,100.,0.,0.,reason));
 EXPECT_EQ(reason,"entry_measured_velocity_off_candidate");
}

TEST(LocalFollower, NativeGeometryRefreshUsesRealProofSourceNotOriginalCurveExpiry) {
 auto original=decodeLocalCurve(tag());auto cache=EntryCurveCache::build(original);ASSERT_TRUE(cache);
 auto c=cfg();c.geometry_authority=GeometryAuthority::NativeScan;c.validate();
 auto measured=body(101.,.04,.10);measured.position.y()=.08;std::string reason;
 EXPECT_FALSE(cache->observe(original,measured,c,101.,100.59,0.,0.,reason));
 EXPECT_EQ(reason,"entry_proof_body_gap_exceeded");
 // The immutable original source remains 100. A genuinely new Native proof
 // permits its same bounded geometry; neither source timestamp is re-dated.
 const auto refreshed=cache->observe(original,measured,c,101.,101.,0.,0.,reason);
 ASSERT_TRUE(refreshed)<<reason;EXPECT_DOUBLE_EQ(original.join_source_stamp,100.);
 EXPECT_DOUBLE_EQ(cache->origin_source_stamp,100.);EXPECT_DOUBLE_EQ(refreshed->join_source_stamp,101.);
 EXPECT_EQ(refreshed->original_join_source_stamp_ns,timeNs(stamp(100.)));
}

TEST(LocalFollower, PreparedThenAppliedProjectionKeepsImmutableCandidateOrigin) {
 LocalFollower f(cfg(),"isolated_mock",{},false);
 auto measured=body(101.,.59,.10);measured.position.y()=.08;
 ASSERT_TRUE(f.body(measured,101.,11.));ASSERT_TRUE(f.proposal(proposal(ver(),101.),101.));
 ASSERT_TRUE(f.support(support()));ASSERT_TRUE(f.candidate(tag(ver(),1,0.,100.)));
 auto refreshed=proof(ver(),1,101.);refreshed.valid_start_time=1.96;refreshed.valid_start_arc_length=.49;
 ASSERT_TRUE(f.validation(refreshed));const auto request=f.exchangeCommit(101.,11.);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,101.,11.));
 const auto commit=f.exchangeCommit(101.,11.);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed");
 // A fresh proof near the edge of the original domain must NOT move that
 // domain forward. Both observe() and core admission retain original arc 0.
 EXPECT_NEAR(commit->arc_length,.5,1e-9);EXPECT_NEAR(commit->curve_time,2.,1e-9);
 EXPECT_NEAR(commit->measured_pose.position.x,.59,1e-12);
 EXPECT_NEAR(commit->measured_pose.position.y,.08,1e-12);
 ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,101.,11.));
 EXPECT_NEAR(f.controller().progressArc(),.5,1e-9);
 EXPECT_EQ(f.controller().joinDiagnostic().original_join_source_stamp_ns,timeNs(stamp(100.)));
 EXPECT_EQ(f.controller().joinDiagnostic().join_source_stamp_ns,timeNs(stamp(101.)));
}

TEST(LocalFollower, ScheduledSpliceKeepsExactStateAdmissionInNativeComponent) {
 for(const bool velocity_mismatch:{false,true}) {
  LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
  auto measured=body(100.2,.05,velocity_mismatch?.19:.25);
  if(!velocity_mismatch)measured.position.y()=.02;
  ASSERT_TRUE(f.body(measured,100.2,10.2));ASSERT_TRUE(f.validation(proof(ver(),1,100.2,2)));
  auto tail=tag(ver(),2,.05,100.2);tail.splice.enabled=true;tail.splice.incumbent_version=ver();
  tail.splice.incumbent_trajectory_id=1;tail.splice.measured_progress_stamp=stamp(100.2);
  tail.splice.splice_curve_time=.2;tail.splice.splice_arc_length=.05;tail.splice.position.x=.05;
  tail.splice.position.z=.55;tail.splice.velocity.x=.25;
  ASSERT_TRUE(decodeLocalCurve(tail).strict_splice_admission);
  ASSERT_TRUE(f.candidate(tail));auto candidate_proof=proof(ver(),2,100.2,3);candidate_proof.splice=tail.splice;
  ASSERT_TRUE(f.validation(candidate_proof));const auto request=f.exchangeCommit(100.2,10.2);ASSERT_TRUE(request);
  auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,100.2,10.2));
  const auto rejected=f.exchangeCommit(100.2,10.2);ASSERT_TRUE(rejected);
  EXPECT_EQ(rejected->disposition,"abort");EXPECT_EQ(rejected->reason,velocity_mismatch?
    "entry_measured_velocity_off_candidate":"entry_measured_body_not_on_candidate");
  EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_EQ(f.commitSequence(),1u);
 }
}

TEST(LocalFollower, GeometryReplacementPreservesCommandVelocityAccelerationAndTask) {
 for(bool terrain_portals:{false,true}) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.8;c.max_yaw_jerk=1.;
 if(terrain_portals)c.geometry_authority=GeometryAuthority::NativeScan;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 Eigen::Vector2d previous=Eigen::Vector2d::Zero(),previous_acceleration=Eigen::Vector2d::Zero();
 auto active=ver();std::int64_t id=1;
 for(unsigned i=1;i<=24;++i) {
  const double now=100.+i*.05;auto measured=body(now,.12*i*.05,.12);
  measured.position.y()=.03;
  ASSERT_TRUE(f.body(measured,now,now-90.));
  ASSERT_TRUE(f.validation(proof(active,id,now,i*2)));
  if(i==8||i==16) {
   auto next=ver(i/8+1);
   if(terrain_portals)next.segment_id=i==8?"stair_lower":"floor2";
   auto p=proposal(next,now);p.expected_version=active;p.expected_trajectory_id=id;
   auto source=support(next);
   // Exercise the production handoff/limiter on semantic entry and exit.
   // Geometry here is deliberately unchanged: this is not stair dynamics.
   if(terrain_portals&&i==8) {
    source.floor_id="stair_lower";source.segment_kind="stair_up";source.required_mode="stair";
    source.max_support_step_m=source.max_support_slope_rad=0.;
   } else if(terrain_portals)source.floor_id="floor2";
   ASSERT_TRUE(f.proposal(p,now));ASSERT_TRUE(f.support(source));
   ASSERT_TRUE(f.candidate(tag(next,id+1,measured.position.x(),now)));
   ASSERT_TRUE(f.validation(proof(next,id+1,now,i*2+1)));
   const auto request=f.exchangeCommit(now,now-90.);ASSERT_TRUE(request);
   auto ack=*request;ack.accepted=true;ack.disposition="prepared";ASSERT_TRUE(f.commitResult(ack,now,now-90.));
   const auto commit=f.exchangeCommit(now,now-90.);ASSERT_TRUE(commit)<<f.candidateReason();
   ASSERT_EQ(commit->disposition,"committed");ack=*commit;ack.accepted=true;
   ASSERT_TRUE(f.commitResult(ack,now,now-90.));active=next;++id;
  }
  ASSERT_TRUE(f.lease(lease(now,i),now));ASSERT_TRUE(f.geometryPermit(now).allowed);
  const auto demand=f.step(now,now-90.);ASSERT_FALSE(demand.hold)<<demand.reason;
  const Eigen::Vector2d command(demand.velocity.linear.x,demand.velocity.angular.z);
  const Eigen::Vector2d acceleration=(command-previous)/.05;
  EXPECT_GT(command.x(),0.);EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);
  EXPECT_LE(std::abs(acceleration.x()),c.max_acceleration+1e-9);
  EXPECT_LE(std::abs(acceleration.y()),c.max_yaw_acceleration+1e-9);
  EXPECT_LE(std::abs(acceleration.x()-previous_acceleration.x()),c.max_jerk*.05+1e-8);
  EXPECT_LE(std::abs(acceleration.y()-previous_acceleration.y()),c.max_yaw_jerk*.05+1e-8);
  EXPECT_EQ(f.activeVersion()->route_hash,ver().route_hash);EXPECT_EQ(f.activeVersion()->task_id,ver().task_id);
  previous=command;previous_acceleration=acceleration;
 }
 EXPECT_EQ(f.commitSequence(),3u);EXPECT_EQ(f.controller().trajectoryId(),3);
 }
}

TEST(LocalFollower, UnusableReservedEntryCannotStarveNewerCandidateOrResetMovingIncumbent) {
 auto c=cfg();c.align_then_forward=true;c.max_jerk=.8;c.max_yaw_jerk=1.;
 LocalFollower f(c,"isolated_mock",{},false);stageExchange(f);
 // The body is stationary (e.g. actuator breakaway), while the command ramps
 // normally. A short candidate cannot accommodate the current command; a
 // newer, ordinary-length candidate can. All geometry is built by production.
 double previous=0.,previous_acceleration=0.;unsigned sequence=1;
 const auto tick=[&](double now,bool deliver_body=true) {
   if(deliver_body)EXPECT_TRUE(f.body(body(now,0.,0.),now,now-90.));
   EXPECT_TRUE(f.validation(proof(ver(),1,now,++sequence)));
   EXPECT_TRUE(f.lease(lease(now,sequence),now));f.geometryPermit(now);
   const auto demand=f.step(now,now-90.);
   EXPECT_FALSE(demand.hold)<<demand.reason;EXPECT_GT(demand.velocity.linear.x,0.);
   const double acceleration=(demand.velocity.linear.x-previous)/.05;
   EXPECT_LE(std::abs(acceleration),c.max_acceleration+1e-9);
   EXPECT_LE(std::abs(acceleration-previous_acceleration),c.max_jerk*.05+1e-8);
   previous=demand.velocity.linear.x;previous_acceleration=acceleration;
 };
 for(unsigned i=1;i<=20;++i)tick(100.+i*.05);
 ASSERT_GT(previous,.1);
 auto short_curve=tag(ver(),2,0.,101.);
 for(auto& point:short_curve.trajectory.pos_pts)point.x*=.01;
 ASSERT_TRUE(f.candidate(short_curve));ASSERT_TRUE(f.validation(proof(ver(),2,101.,++sequence)));
 const auto request=f.exchangeCommit(101.,11.);ASSERT_TRUE(request);
 ASSERT_EQ(request->disposition,"prepare");
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,101.,11.));
 ASSERT_TRUE(f.candidate(tag(ver(),3,0.,101.)));
 ASSERT_TRUE(f.validation(proof(ver(),3,101.,++sequence)));
 std::optional<wire::LocalTrajectoryCommit> rejected;
 // Repeated NEW proofs remove expiry as an escape hatch. Previously this
 // reserved curve kept the exchange forever and blocked the ready successor.
 for(unsigned i=1;i<=12&&!rejected;++i) {
   const double now=101.+i*.05;
   EXPECT_TRUE(f.body(body(now,0.,0.),now,now-90.));
   EXPECT_TRUE(f.validation(proof(ver(),2,now,++sequence)));
   rejected=f.exchangeCommit(now,now-90.);
   EXPECT_EQ(f.controller().trajectoryId(),1);
   if(!rejected)tick(now,false);
 }
 ASSERT_TRUE(rejected)<<f.candidateReason();
 EXPECT_EQ(rejected->disposition,"abort");
 EXPECT_EQ(rejected->reason,"curvature_braking_limit_below_current_command");
 EXPECT_EQ(rejected->trajectory_id,2);
 EXPECT_EQ(f.controller().trajectoryId(),1);
 const double now=seconds(rejected->source_stamp);
 EXPECT_FALSE(f.commitResult(ack,now,now-90.)); // late prepared ACK cannot revive curve 2
 EXPECT_TRUE(f.validation(proof(ver(),1,now,++sequence)));
 EXPECT_TRUE(f.validation(proof(ver(),3,now,++sequence)));
 const auto next=f.exchangeCommit(now,now-90.);ASSERT_TRUE(next);
 ASSERT_EQ(next->disposition,"prepare");EXPECT_EQ(next->trajectory_id,3);
 ack=*next;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,now,now-90.));
 const auto commit=f.exchangeCommit(now,now-90.);ASSERT_TRUE(commit)<<f.candidateReason();
 ASSERT_EQ(commit->disposition,"committed");EXPECT_EQ(commit->trajectory_id,3);
 ack=*commit;ack.accepted=true;ASSERT_TRUE(f.commitResult(ack,now,now-90.));
 EXPECT_TRUE(f.lease(lease(now,++sequence),now));f.geometryPermit(now);
 const auto demand=f.step(now,now-90.);
 EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
 const double acceleration=(demand.velocity.linear.x-previous)/.05;
 EXPECT_LE(std::abs(acceleration),c.max_acceleration+1e-9);
 EXPECT_LE(std::abs(acceleration-previous_acceleration),c.max_jerk*.05+1e-8);
 EXPECT_EQ(f.activeVersion()->task_id,ver().task_id);
}

TEST(LocalFollower, ReservedEntryWaitsForActualFreshBodyThenCommitsSameCandidate) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
 ASSERT_TRUE(f.candidate(tag(ver(),2)));ASSERT_TRUE(f.validation(proof(ver(),2)));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.,10.));
 ASSERT_TRUE(f.validation(proof(ver(),2,100.11,2)));
 EXPECT_FALSE(f.exchangeCommit(100.11,10.11));
 EXPECT_EQ(f.candidateReason(),"entry_body_not_fresh");
 EXPECT_EQ(f.controller().trajectoryId(),1);
 ASSERT_TRUE(f.body(body(100.12,.03),100.12,10.12));
 const auto committed=f.exchangeCommit(100.12,10.12);ASSERT_TRUE(committed)<<f.candidateReason();
 EXPECT_EQ(committed->disposition,"committed");EXPECT_EQ(committed->trajectory_id,2);
 EXPECT_EQ(committed->commit_sequence,request->commit_sequence);
}

TEST(LocalFollower, InvalidImmutableDerivativesReleaseReservationWithoutChangingIncumbent) {
 LocalFollower f(cfg(),"isolated_mock",{},false);stageExchange(f);
 auto fast=tag(ver(),2);for(auto& p:fast.trajectory.pos_pts)p.x*=4.;
 ASSERT_TRUE(f.candidate(fast));ASSERT_TRUE(f.validation(proof(ver(),2)));
 const auto request=f.exchangeCommit(100.,10.);ASSERT_TRUE(request);
 auto ack=*request;ack.accepted=true;ack.disposition="prepared";
 ASSERT_TRUE(f.commitResult(ack,100.,10.));
 const auto rejected=f.exchangeCommit(100.,10.);ASSERT_TRUE(rejected)<<f.candidateReason();
 EXPECT_EQ(rejected->disposition,"abort");EXPECT_EQ(rejected->reason,"curve_derivative_bound");
 EXPECT_EQ(f.controller().trajectoryId(),1);EXPECT_TRUE(f.status(100.).ready);
 auto negative=proof(ver(),1,100.,2);negative.valid=false;negative.reason="occupied";
 ASSERT_TRUE(f.validation(negative));ASSERT_TRUE(f.lease(lease(),100.));f.geometryPermit(100.);
 const auto stopped=f.step(100.,10.);
 EXPECT_TRUE(stopped.hold);EXPECT_EQ(stopped.reason,"local_collision_state_unavailable");
}

TEST(LocalFollower, PartialStairAndLandingEndpointsUseSelectedBodyGoalWithoutFloorAlias) {
 // Callback/contract test only: the curve is supplied, not a native SCAN
 // stair-dynamics simulation. The production follower must accept the same
 // typed terrain metadata emitted by SourceRouteBuilder/ReferenceCallbacks.
 for(const std::string kind:{"stair_up","stair_down","landing"}) {
  SCOPED_TRACE(kind);
  auto c=cfg();c.geometry_authority=GeometryAuthority::NativeScan;
  LocalFollower f(c,"isolated_mock",{},false);
  auto v=ver();v.segment_id=kind=="landing"?"stair_landing":"stair_lower";
  const double ground=1.8,slope=kind=="stair_up"?.15:kind=="stair_down"?-.15:0.;
  auto p=proposal(v);p.goal_position.x=.6;p.goal_position.z=ground+.55+slope*.6;
  p.reference.segment_id=v.segment_id;p.reference.segment_kind=kind;p.reference.required_mode="stair";
  auto s=support(v);s.floor_id=kind=="landing"?"stairs":"stair_lower";
  s.segment_kind=kind;s.required_mode="stair";s.max_support_step_m=s.max_support_slope_rad=0.;
  for(auto& point:s.support_ground_xyz)point.z=ground+slope*point.x;
  auto t=tag(v);for(auto& point:t.trajectory.pos_pts)point.z=ground+.55+slope*point.x;
  t.join_pose.position.z=ground+.55;t.join_twist.linear.z=slope*.25;
  auto o=body();o.position.z()=ground+.55;o.velocity_in_frame.z()=slope*.25;
  ASSERT_TRUE(f.body(o,100.,10.));ASSERT_TRUE(f.proposal(p,100.));ASSERT_TRUE(f.support(s));
  ASSERT_TRUE(f.candidate(t));ASSERT_TRUE(f.validation(proof(v)));
  ASSERT_TRUE(f.tryCommit(100.,10.))<<f.candidateReason();
  auto permit=lease();permit.version=v;ASSERT_TRUE(f.lease(permit,100.));
  auto actual=sendFact(100.,1);actual.version=v;actual.command_selected=actual.sdk_nonzero=true;
  ASSERT_TRUE(f.commandFeedback(actual,100.)); // Healthy stair facts do not introduce a hold.
  ASSERT_TRUE(f.geometryPermit(100.).allowed);
  const auto demand=f.step(100.01,10.01);
  EXPECT_FALSE(demand.hold)<<demand.reason;EXPECT_GT(demand.velocity.linear.x,0.);
  EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);
  // Same XY on a different surface must not complete the partial target.
  o=body(100.05,.6,0.);o.position.z()=p.goal_position.z+.4;
  ASSERT_TRUE(f.body(o,100.05,10.05));ASSERT_TRUE(f.validation(proof(v,1,100.05,2)));
  EXPECT_FALSE(f.status(100.05).at_goal);EXPECT_FALSE(f.status(100.05).request_stop);
  // The actual selected intermediate tread/landing is the task endpoint;
  // no floor2 portal, next flight or mode change is required to report it.
  o=body(100.10,.6,0.);o.position.z()=p.goal_position.z;
  ASSERT_TRUE(f.body(o,100.10,10.10));ASSERT_TRUE(f.validation(proof(v,1,100.10,3)));
  ASSERT_TRUE(f.geometryPermit(100.10).allowed);
  const auto status=f.status(100.10);
  EXPECT_TRUE(status.at_goal);EXPECT_TRUE(status.request_stop);EXPECT_EQ(status.phase,"goal_reached");
  EXPECT_EQ(status.version.segment_id,v.segment_id);EXPECT_FALSE(status.software_retired);
  const auto stopped=f.step(100.10,10.10);EXPECT_TRUE(stopped.hold);
  EXPECT_DOUBLE_EQ(stopped.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(stopped.velocity.linear.y,0.);
  EXPECT_DOUBLE_EQ(stopped.velocity.angular.z,0.);
 }
}
