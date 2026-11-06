// Production GridMap + Rebound/Refine + Native validator + typed LocalFollower
// + production SDK amplitude map. Acquired robot response and scheduling are
// simulated. No ROS node, context, graph, SDK client or actuator writer is
// constructed. This exercises the C++ handoff contract, not the Python
// reference scheduler or the complete production FSM.
#include "native_local_follower_test_fixture.hpp"
#include "sdk_input_mapping.hpp"
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& optimizer,const GridMap::Ptr& map) {
    optimizer.order_=3;optimizer.lambda1_=optimizer.lambda2_=optimizer.lambda3_=optimizer.lambda4_=1.;
    optimizer.dist0_=.2;optimizer.max_vel_=.3;optimizer.max_acc_=.35;optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,{80,40,32});
  }
};
}

namespace {
namespace sdk=d1monitor::execution3;
constexpr double dt=.05;
struct RollingPolicy {t::Config control;sdk::SdkInputMapping mapping;};
RollingPolicy rollingPolicy() {
  const char* root=std::getenv("D1MAX_ROLLING_SOURCE");
  if(!root)throw std::runtime_error("D1MAX_ROLLING_SOURCE is required");
  const auto config_dir=std::filesystem::path(root)/"src/d1max_pct_scan/config";
  const auto y=YAML::LoadFile((config_dir/"motion_tracking.yaml").string());
  std::ifstream input(config_dir/"runtime_motion_profile.json");nlohmann::json j;input>>j;
  RollingPolicy p;auto& c=p.control;c=config();
  c.motion_model=y["motion_model"].as<std::string>();c.holonomic_yaw_policy=y["heading_policy"].as<std::string>();
  c.align_then_forward=y["align_then_forward"].as<bool>();
  c.max_speed=y["max_speed_mps"].as<double>();c.max_yaw_rate=y["max_yaw_radps"].as<double>();
  c.max_acceleration=y["max_acceleration_mps2"].as<double>();c.max_yaw_acceleration=y["max_yaw_acceleration_radps2"].as<double>();
  c.max_jerk=y["max_jerk_mps3"].as<double>();c.max_yaw_jerk=y["max_yaw_jerk_radps3"].as<double>();
  c.lookahead=y["preview_time_s"].as<double>();c.kp_position=y["position_gain"].as<double>();c.kp_yaw=y["yaw_gain"].as<double>();
  c.min_lookahead_distance_m=y["min_lookahead_distance_m"].as<double>();c.max_lookahead_distance_m=y["max_lookahead_distance_m"].as<double>();
  c.steering_error_scale_rad=y["steering_error_scale_rad"].as<double>();
  c.velocity_damping_gain=y["velocity_damping_gain"].as<double>();c.yaw_damping_gain=y["yaw_damping_gain"].as<double>();
  c.max_input_rate=y["max_input_rate_per_s"].as<double>();c.max_input_acceleration=y["max_input_acceleration_per_s2"].as<double>();
  const auto& m=j.at("sdk_input_mapping");
  if(m.at("schema_version")!=4)throw std::runtime_error("production V4 mapping required");
  p.mapping.enabled=p.mapping.component_cap=p.mapping.continuous_command_ratio=true;
  const auto axis=[](const auto& from,auto& to){to.effective_fraction=from.at("effective_fraction");
    to.maximum_fraction=from.at("maximum_fraction");to.blend_demand_ratio=from.at("blend_demand_ratio");};
  axis(m.at("planar"),p.mapping.planar);axis(m.at("yaw"),p.mapping.yaw);
  c.forward_response={c.max_speed,p.mapping.planar.effective_fraction,p.mapping.planar.maximum_fraction,p.mapping.planar.blend_demand_ratio};
  c.yaw_response={c.max_yaw_rate,p.mapping.yaw.effective_fraction,p.mapping.yaw.maximum_fraction,p.mapping.yaw.blend_demand_ratio};
  c.validate();p.mapping.validate();return p;
}
w::TaggedBspline rollingTag(unsigned id,const w::ExecutionVersion& v,double x,double now) {
  auto q=tag(id,{x,0.,.55},{.3,0.,0.},Eigen::Vector3d::Zero());
  q.generation=v.reference_generation;q.trajectory.start_time=q.join_source_stamp=t::stamp(now);
  q.trajectory.pos_pts.clear();q.trajectory.yaw_pts.clear();q.trajectory.knots.clear();
  // Exactly 4 m of uniform native cubic, with .3 m/s derivative. The tracker
  // still constructs its own finite zero-speed endpoint envelope.
  constexpr unsigned controls=43;constexpr double h=1./3.;
  for(unsigned i=0;i<controls;++i){geometry_msgs::msg::Point pt;pt.x=x+(double(i)-1.)*.1;pt.z=.55;
    q.trajectory.pos_pts.push_back(pt);}
  for(unsigned i=0;i<controls+4;++i)q.trajectory.knots.push_back((double(i)-3.)*h);
  q.trajectory.yaw_dt=0.;freezeHeading(q);return q;
}
struct RollingFixture {
  RollingPolicy policy=rollingPolicy();scan_planner::CollisionSnapshotPool pool;
  Native native{Native::PassiveLedger{},pool,"odom","isolated_mock",false,false,true};
  t::LocalFollower follower{policy.control,"isolated_mock",std::string(64,'d'),false};GridMap map;
  scan_planner::SplineHeadingContract heading;w::ReferenceProposal reference=proposal();
  unsigned generation=1,curve_id=1;double x=0.,vx=.3;
  bool heading_noise=false;
  std::optional<w::LocalTrajectoryCommit> delivery;
  std::optional<w::TaggedBspline> solving;double solve_ready_at=0.;
  bool pending=false;double last_issued_x=0.,last_issued_time=100.;
  unsigned actual_reseeds=0;
  RollingFixture() {
    scan_planner::BrakingModel b{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'d')};
    b.kinematic_profile=scan_planner::KinematicProfile::HolonomicXYYaw;b.lateral_distance=.08;b.command_pipeline_bound=.1;
    Access::braking(native,b);Access::geometryPolicy(native,true);GridMapTestAccess::configure(map,true,600);
    heading.measured_heading_enabled=true;heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;
    heading.measured_yaw=0.;heading.body_extent=.53;
    reference.goal_position.x=7.;installReference(100.);acquire(100.);
    const auto first=rollingTag(1,reference.version,0.,100.);native.candidate(first,reference,heading);
    EXPECT_TRUE(follower.candidate(first));const auto p=Access::checkCurve(native,map,100000000000LL);
    EXPECT_TRUE(p&&p->valid)<<(p?p->reason:"no proof");if(p)EXPECT_TRUE(follower.validation(*p));
    const auto committed=follower.tryCommit(100.,10.);EXPECT_TRUE(committed)<<follower.candidateReason();
    if(committed)EXPECT_TRUE(native.localCommitAt(*committed,100000000000LL));
  }
  void installReference(double now) {
    reference.version=version();reference.version.reference_generation=generation;
    reference.proposal_id="rolling-"+std::to_string(generation);
    reference.source_stamp=t::stamp(now);reference.valid_until=t::stamp(now+.9);
    reference.expected_version=follower.activeVersion().value_or(w::ExecutionVersion{});
    reference.expected_trajectory_id=follower.controller().trajectoryId();
    EXPECT_TRUE(native.replacePendingReference(reference));EXPECT_TRUE(follower.proposal(reference,now));
    auto s=support();s.version=reference.version;s.support_ground_xyz.clear();
    for(int i=-20;i<=380;++i)for(int j=-4;j<=4;++j){geometry_msgs::msg::Point p;p.x=i*.025;p.y=j*.025;s.support_ground_xyz.push_back(p);}
    native.support(s);EXPECT_TRUE(follower.support(s));
  }
  void acquire(double now) {
    auto o=body(now,x,vx);
    if(heading_noise){o.yaw=.012*std::sin((now-100.)*17.);o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());}
    EXPECT_TRUE(follower.body(o,now,now-90.));
    geometry_msgs::msg::Twist twist;twist.linear.x=vx;native.body({o.position,o.orientation,now,"odom"},twist,o.source_stamp_ns);
    GridMapTestAccess::acquire(map,o.source_stamp_ns);follower.controller().refreshMeasuredProgress(now);
    native.progressAt(t::trackingProgress(follower.controller(),policy.control,now,follower.status(now)),now);
  }
  bool prepare(double now,double solve_delay=.4) {
    ++generation;++curve_id;installReference(now);
    const auto splice=native.plannedSpliceAt(reference.version,t::timeNs(t::stamp(now)),.05,.8);
    if(!splice)return false;
    const auto& w=splice->wire;
    auto q=boundaryTag(now,{w.position.x,w.position.y,w.position.z},
        {w.velocity.x,w.velocity.y,w.velocity.z},
        {w.acceleration.x,w.acceleration.y,w.acceleration.z});q.splice=w;
    q.join_source_stamp={};q.join_pose={};q.join_twist={};
    solving=q;solve_ready_at=now+solve_delay;
    if(solve_delay==0.)publishSolved(now);
    pending=true;last_issued_x=x;last_issued_time=now;return true;
  }
  void publishSolved(double now) {
    if(!solving)return;
    // Actual publication time, not a fabricated future acquired body stamp.
    solving->trajectory.start_time=t::stamp(now);native.candidate(*solving,reference,heading);
    EXPECT_TRUE(follower.candidate(*solving));solving.reset();
  }
  w::TaggedBspline boundaryTag(double now,const Eigen::Vector3d& position,
      const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration) {
    const Eigen::Vector3d end=position+Eigen::Vector3d{4.,0,0};
    scan_planner::DiscreteReference path;path.set({position,end});
    const auto seed=scan_planner::sampleReferenceSeed(path,.3,.35,velocity.norm(),0.,.2);
    const scan_planner::CubicMotionBoundary boundary{position,velocity,acceleration,end,{0,0,0},{0,0,0}};
    auto cp=scan_planner::fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
    auto owned_map=GridMap::Ptr(&map,[](GridMap*){});scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::configure(optimizer,owned_map);
    auto budget=std::make_shared<scan_planner::SolveBudget>();optimizer.setSolveBudget(budget);
    optimizer.reference_path_samples_=seed.samples;optimizer.initControlPoints(cp);
    EXPECT_TRUE(optimizer.controlPointsInitialized());Eigen::MatrixXd optimized;
    EXPECT_TRUE(optimizer.BsplineOptimizeTrajRebound(optimized,seed.dt));cp=optimized;
    scan_planner::UniformBspline curve(cp,3,seed.dt);
    const auto timed=scan_planner::refineTimingWithFixedBoundary(curve,boundary,.3,.35,
      [&](Eigen::MatrixXd& points,double dt) {
        scan_planner::UniformBspline reference(points,3,dt);optimizer.ref_pts_.clear();
        for(int i=0;i<=points.cols()-3;++i)optimizer.ref_pts_.push_back(reference.evaluateDeBoorT(i*dt));
        Eigen::MatrixXd out;const bool ok=optimizer.BsplineOptimizeTrajRefine(points,dt,out);
        if(ok)points=out;return ok;
      },8,std::nullopt,[&]{return budget->allowed();},true);
    EXPECT_TRUE(timed.success)<<timed.reason;
    auto q=tag(curve_id,position,velocity,acceleration);
    q.generation=reference.version.reference_generation;q.trajectory.start_time=q.join_source_stamp=t::stamp(now);
    q.trajectory.pos_pts.clear();q.trajectory.knots.clear();q.trajectory.yaw_pts.clear();q.trajectory.yaw_dt=0.;
    cp=curve.getControlPoint();const auto knots=curve.getKnot();
    for(int i=0;i<cp.cols();++i){geometry_msgs::msg::Point p;p.x=cp(0,i);p.y=cp(1,i);p.z=cp(2,i);q.trajectory.pos_pts.push_back(p);}
    for(int i=0;i<knots.size();++i)q.trajectory.knots.push_back(knots[i]);freezeHeading(q);return q;
  }
  void reseedFromActual(double now) {
    // Model the owner's existing GEN_NEW_TRAJ branch after a typed Native
    // reseed event. Keep the non-ideal plant and acquired p/v exactly; only
    // production Rebound/Refine + finite-boundary recovery supplies geometry.
    ++curve_id;++actual_reseeds;
    auto q=boundaryTag(now,{x,0,.55},{vx,0,0},Eigen::Vector3d::Zero());
    native.candidate(q,reference,heading);ASSERT_TRUE(follower.candidate(q));pending=true;
  }
  std::optional<w::LocalMotionCommand> tick(double now,unsigned sequence,bool& did_commit) {
    const auto ns=t::timeNs(t::stamp(now));did_commit=false;
    const auto active=Access::checkCurve(native,map,ns,true);EXPECT_TRUE(active&&active->valid)<<(active?active->reason:"none");
    if(active)EXPECT_TRUE(follower.validation(*active));
    if(delivery){EXPECT_TRUE(follower.commitResult(*delivery,now,now-90.));delivery.reset();}
    if(solving&&now+1e-9>=solve_ready_at)publishSolved(now);
    if(pending&&!solving) {
      auto proof=Access::checkReserved(native,map,ns);if(!proof)proof=Access::checkCurve(native,map,ns);
      const auto pending_curve=Access::pendingCurve(native);
      EXPECT_TRUE(proof&&proof->valid)<<(proof?proof->reason:"none")
        <<" body_x="<<x<<" body_vx="<<vx<<" now="<<now
        <<" progress="<<follower.controller().progressArc()
        <<" splice_arc="<<(pending_curve?pending_curve->splice.splice_arc_length:-1.)
        <<" splice_x="<<(pending_curve?pending_curve->splice.position.x:-1.);
      if(proof)EXPECT_TRUE(follower.validation(*proof));
      if(const auto event=follower.exchangeCommit(now,now-90.)) {
        const auto reply=native.localCommitExchangeAt(*event,ns);EXPECT_TRUE(reply.accepted)<<reply.reason;
        if(event->disposition=="abort") {
          const auto reseed=native.consumeFormalReseedAt(ns);EXPECT_TRUE(reseed.has_value());
          if(reseed){EXPECT_TRUE(reseed->actual_body);reseedFromActual(now);}
        }else delivery=reply;
        if(event->disposition=="committed"){did_commit=true;pending=false;}
      }
    }
    auto grant=lease(now,sequence);grant.version=reference.version;
    EXPECT_TRUE(follower.lease(grant,now));EXPECT_TRUE(native.taskLeaseAt(grant,ns));
    EXPECT_TRUE(native.commit(follower.geometryPermit(now)));const auto demand=follower.step(now,now-90.);
    std::optional<w::LocalMotionCommand> output;native.demandAt(demand,ns,[&](const auto& command){output=command;});
    native.progressAt(t::trackingProgress(follower.controller(),policy.control,now,follower.status(now)),now);
    return output;
  }
  void advance(const sdk::SdkMoveFractions& input) {
    // Declared response assumption: .30 SDK fraction breaks away at .20m/s;
    // .40 corresponds to .30m/s. First-order 50ms response, NOT calibration.
    const double target=input.forward_back>=.30?.20+(input.forward_back-.30):0.;
    vx+=(target-vx)*(-std::expm1(-dt/.05));x+=vx*dt;
  }
};
void checkHealthyWindows(bool rolling) {
  RollingFixture f;f.heading_noise=true;const auto& c=f.policy.control;
  const unsigned expected_commits=rolling?6U:2U;
  unsigned commits=0,internal_zero=0;double previous=0.,previous_acc=0.,previous_fraction=0.,previous_fraction_rate=0.;
  double previous_yaw=0.,previous_yaw_acc=0.,previous_sdk_yaw=0.,previous_sdk_yaw_rate=0.;
  double minimum=1.,max_dv=0.,max_da=0.,max_jerk=0.,max_sdk_rate=0.,max_sdk_acc=0.;
  double minimum_sdk=1.,minimum_measured=1.;
  double max_yaw_delta=0.,max_yaw_jerk=0.,max_sdk_yaw_rate=0.,max_sdk_yaw_acc=0.;
  double current_end=4.;
  for(unsigned i=1;i<=600&&commits<expected_commits;++i) {
    const double now=100.+i*dt;f.acquire(now);
    // Acquired window request stream mirrors the real rolling policy's
    // .25m / >=.5s conditions. Its Python callback has separate tests; this
    // fixture does NOT claim to execute that Python callback or the whole FSM.
    const bool due=rolling?(f.x-f.last_issued_x>=.25&&now-f.last_issued_time>=.5):current_end-f.x<=2.;
    if(!f.pending&&!f.delivery&&i>40&&due)
      ASSERT_TRUE(f.prepare(now))<<f.follower.candidateReason();
    bool committed=false;const auto output=f.tick(now,i,committed);ASSERT_TRUE(output)<<f.follower.controller().reason();
    const auto& v=output->velocity;const auto mapped=sdk::mappedSdkFractions(v.linear.x,v.linear.y,v.angular.z,
      "holonomic_xyyaw",f.policy.mapping,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(v.linear.y,0.);EXPECT_DOUBLE_EQ(mapped.left_right,0.);
    EXPECT_EQ(output->body_source_stamp,t::stamp(now));EXPECT_EQ(output->source_stamp,t::stamp(now));
    const double acceleration=(v.linear.x-previous)/dt,jerk=(acceleration-previous_acc)/dt;
    const double fraction_rate=(mapped.forward_back-previous_fraction)/dt;
    const double yaw_acc=(v.angular.z-previous_yaw)/dt,yaw_jerk=(yaw_acc-previous_yaw_acc)/dt;
    const double sdk_yaw_rate=(mapped.yaw-previous_sdk_yaw)/dt,sdk_yaw_acc=(sdk_yaw_rate-previous_sdk_yaw_rate)/dt;
    EXPECT_LE(std::abs(acceleration),c.max_acceleration+1e-8);
    EXPECT_LE(std::abs(jerk),c.max_jerk+1e-7);
    EXPECT_LE(std::abs(fraction_rate),c.max_input_rate+1e-8);
    EXPECT_LE(std::abs(fraction_rate-previous_fraction_rate)/dt,c.max_input_acceleration+1e-7);
    EXPECT_LE(std::abs(yaw_acc),c.max_yaw_acceleration+1e-8);EXPECT_LE(std::abs(yaw_jerk),c.max_yaw_jerk+1e-7);
    EXPECT_LE(std::abs(sdk_yaw_rate),c.max_input_rate+1e-8);EXPECT_LE(std::abs(sdk_yaw_acc),c.max_input_acceleration+1e-7);
    EXPECT_LT(std::abs(mapped.yaw),.20); // This noise does not cross the assumed yaw working threshold.
    if(i>40){minimum=std::min(minimum,v.linear.x);internal_zero+=output->hold||v.linear.x<=.01;
      minimum_sdk=std::min(minimum_sdk,mapped.forward_back);minimum_measured=std::min(minimum_measured,f.vx);}
    if(committed){++commits;const auto view=f.native.committedView();ASSERT_TRUE(view);
      auto curve=scan_planner::decodeSpliceCurve(view->spline.trajectory);ASSERT_TRUE(curve);
      current_end=curve->evaluateDeBoorT(curve->getTimeSum()).x();
      EXPECT_GT(v.linear.x,.20);max_dv=std::max(max_dv,std::abs(v.linear.x-previous));
      max_da=std::max(max_da,std::abs(acceleration-previous_acc));max_jerk=std::max(max_jerk,std::abs(jerk));
      max_sdk_rate=std::max(max_sdk_rate,std::abs(fraction_rate));
      max_sdk_acc=std::max(max_sdk_acc,std::abs(fraction_rate-previous_fraction_rate)/dt);
      max_yaw_delta=std::max(max_yaw_delta,std::abs(v.angular.z-previous_yaw));max_yaw_jerk=std::max(max_yaw_jerk,std::abs(yaw_jerk));
      max_sdk_yaw_rate=std::max(max_sdk_yaw_rate,std::abs(sdk_yaw_rate));max_sdk_yaw_acc=std::max(max_sdk_yaw_acc,std::abs(sdk_yaw_acc));}
    f.follower.controller().recordAppliedOutput(v.linear.x,v.linear.y,v.angular.z,now,now-90.);
    f.advance(mapped);previous=v.linear.x;previous_acc=acceleration;previous_fraction=mapped.forward_back;previous_fraction_rate=fraction_rate;
    previous_yaw=v.angular.z;previous_yaw_acc=yaw_acc;previous_sdk_yaw=mapped.yaw;previous_sdk_yaw_rate=sdk_yaw_rate;
  }
  EXPECT_EQ(commits,expected_commits);EXPECT_EQ(internal_zero,0U);
  if(rolling)EXPECT_GT(minimum,.20);
  else {
    // SI demand is not measured speed with the V4 nonlinear SDK mapping.
    // The old half-window comparator must verify actual modeled motion and
    // SDK working amplitude, not require SI >= the plant's .20m/s breakaway.
    EXPECT_GT(minimum_measured,.20);EXPECT_GE(minimum_sdk,.30);
  }
  EXPECT_GT(max_da,1e-7); // History preservation is checked with nonzero derivatives, not only steady output.
  EXPECT_EQ(f.follower.activeVersion()->route_hash,version().route_hash);
  std::cout<<std::setprecision(10)<<"ROLLING_RESULT schedule="<<(rolling?"rolling":"healthy_half_window")
    <<" commits="<<commits<<" actual_reseeds="<<f.actual_reseeds<<" min_v="<<minimum
    <<" min_sdk="<<minimum_sdk<<" min_measured_v="<<minimum_measured
    <<" internal_stops="<<internal_zero<<" handoff_delta_v="<<max_dv<<" handoff_delta_a="<<max_da
    <<" handoff_jerk="<<max_jerk<<" sdk_rate="<<max_sdk_rate<<" sdk_acc="<<max_sdk_acc
    <<" handoff_delta_yaw="<<max_yaw_delta<<" handoff_yaw_jerk="<<max_yaw_jerk
    <<" sdk_yaw_rate="<<max_sdk_yaw_rate<<" sdk_yaw_acc="<<max_sdk_yaw_acc<<"\n";
}
TEST(RollingFollowerNative, SixFiniteWindowsCommitAtActualSplicesWithoutRestartingControl) {checkHealthyWindows(true);}
TEST(RollingFollowerNative, HealthyFormerHalfWindowScheduleWasAlreadyContinuous) {checkHealthyWindows(false);}
TEST(RollingFollowerNative, NoSuccessorOrOnlyReservedOrRejectedSuccessorNeverExtendsFiniteCurve) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    SCOPED_TRACE(scenario);RollingFixture f;const auto& c=f.policy.control;
    bool injected=false,settled=false;unsigned zero_samples=0;
    for(unsigned i=1;i<=600;++i) {
      const double now=100.+i*dt;f.acquire(now);
      if(scenario&&!injected&&f.x>=1.) {
        ASSERT_TRUE(f.prepare(now,0.));
        if(scenario==2)GridMapTestAccess::voxel(f.map,{5.,0.,.55},2.); // Beyond incumbent body's entire 4m curve.
        const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));ASSERT_TRUE(proof);
        ASSERT_EQ(proof->valid,scenario==1)<<proof->reason;ASSERT_TRUE(f.follower.validation(*proof));
        if(scenario==1) {
          const auto request=f.follower.exchangeCommit(now,now-90.);ASSERT_TRUE(request);
          EXPECT_EQ(request->disposition,"prepare");
          const auto reply=f.native.localCommitExchangeAt(*request,t::timeNs(t::stamp(now)));
          ASSERT_TRUE(reply.accepted)<<reply.reason;ASSERT_TRUE(f.follower.commitResult(reply,now,now-90.));
          // A prepared future tail is intentionally NEVER committed. It
          // cannot change the active curve's finite braking envelope.
        }
        f.pending=false;injected=true;
      }
      bool committed=false;const auto output=f.tick(now,i,committed);ASSERT_TRUE(output);
      EXPECT_FALSE(committed);EXPECT_EQ(f.follower.controller().trajectoryId(),1);
      EXPECT_EQ(f.native.committedTrajectory(),1);
      const auto& v=output->velocity;EXPECT_DOUBLE_EQ(v.linear.y,0.);
      const auto mapped=sdk::mappedSdkFractions(v.linear.x,v.linear.y,v.angular.z,
        "holonomic_xyyaw",f.policy.mapping,c.max_speed,c.max_yaw_rate);
      EXPECT_LE(f.x,4.03); // Physical response assumption remains within the finite endpoint.
      if(i>40&&v.linear.x==0.&&f.vx<=.03)++zero_samples;else zero_samples=0;
      f.follower.controller().recordAppliedOutput(v.linear.x,v.linear.y,v.angular.z,now,now-90.);
      f.advance(mapped);if(zero_samples>=20){settled=true;break;}
    }
    EXPECT_TRUE(settled)<<f.follower.controller().reason()<<" x="<<f.x<<" vx="<<f.vx;
    EXPECT_TRUE(f.follower.controller().active());EXPECT_FALSE(f.follower.status(130.).software_retired);
    if(scenario)EXPECT_TRUE(injected);
    std::cout<<"FINITE_BOUNDARY scenario="<<scenario<<" stopped="<<settled<<" x="<<f.x<<"\n";
  }
}
} // namespace

namespace {
// Scheduling probe, not another planner. Every curve is a production uniform
// cubic; Native checks the real GridMap and performs the actual typed reserve /
// measured-entry / commit exchange. A 10 Hz result stream and ACK transport are
// injected independently of 50 Hz acquisition and 20 Hz control. Map source
// updates remain 5 Hz. The source time is never replaced by a delivery time.
struct TenHzMetrics {
  unsigned offered{0},committed{0},interior_stops{0},bad_candidates{0},rollbacks{0};
  double min_forward{1.},max_acc{0.},max_jerk{0.},max_yaw_acc{0.},max_yaw_jerk{0.};
  double max_sdk_rate{0.},max_sdk_acc{0.},max_body_age{0.},max_commit_delta{0.};
};
void checkTenHzResults(double candidate_phase,double ack_delay,bool inject_failures=false) {
  SCOPED_TRACE(::testing::Message()<<"candidate_phase="<<candidate_phase<<" ack_delay="<<ack_delay);
  RollingFixture f;const auto& c=f.policy.control;TenHzMetrics m;
  struct Ack {double at;w::LocalTrajectoryCommit message;};std::deque<Ack> replies;
  sdk::SdkMoveFractions input{};
  double last_forward=0.,last_acc=0.,last_yaw=0.,last_yaw_acc=0.;
  double last_fraction=0.,last_fraction_rate=0.,last_yaw_fraction=0.,last_yaw_fraction_rate=0.;
  double next_candidate=101.+candidate_phase;
  unsigned sequence=0,curve_id=1;double last_committed_at=100.;
  constexpr double event_dt=.005;
  for(unsigned event=1;event<=1600;++event) {
    const double now=100.+event*event_dt;
    const double target=input.forward_back>=.30?.20+(input.forward_back-.30):0.;
    f.vx+=(target-f.vx)*(-std::expm1(-event_dt/.05));f.x+=f.vx*event_dt;
    if(event%4==0) { // Actual 50 Hz acquired state, not a controller heartbeat.
      auto acquired=body(now,f.x,f.vx);
      acquired.yaw=.012*std::sin((now-100.)*17.);
      acquired.orientation=Eigen::AngleAxisd(acquired.yaw,Eigen::Vector3d::UnitZ());
      ASSERT_TRUE(f.follower.body(acquired,now,now-90.));
      geometry_msgs::msg::Twist twist;twist.linear.x=f.vx;
      f.native.body({acquired.position,acquired.orientation,now,"odom"},twist,acquired.source_stamp_ns);
      f.follower.controller().refreshMeasuredProgress(now);
      f.native.progressAt(t::trackingProgress(f.follower.controller(),c,now,f.follower.status(now)),now);
      if(event%40==0)GridMapTestAccess::acquire(f.map,acquired.source_stamp_ns);
    }
    while(!replies.empty()&&replies.front().at<=now+1e-9) {
      const auto reply=replies.front().message;replies.pop_front();
      ASSERT_TRUE(f.follower.commitResult(reply,now,now-90.));
    }
    if(now+1e-9>=next_candidate) {
      next_candidate+=.1;++m.offered;
      const auto& acquired=f.follower.controller().odometry();
      f.curve_id=++curve_id;
      // The V4 plant may exceed nominal SI speed. Use the existing production
      // Rebound/Refine fixed-boundary recovery, NOT an indefinitely over-speed
      // constant-velocity test curve. Four-metre tail / original P,V remain.
      ASSERT_GT(acquired.velocity_in_frame.x(),.05);
      auto candidate=f.boundaryTag(now,acquired.position,acquired.velocity_in_frame,Eigen::Vector3d::Zero());
      candidate.join_source_stamp=t::bodySourceStamp(acquired);
      candidate.join_pose.orientation.x=acquired.orientation.x();candidate.join_pose.orientation.y=acquired.orientation.y();
      candidate.join_pose.orientation.z=acquired.orientation.z();candidate.join_pose.orientation.w=acquired.orientation.w();
      freezeHeading(candidate,acquired.yaw);auto heading=f.heading;heading.measured_yaw=acquired.yaw;
      if(inject_failures&&m.offered%5==0) {
        // A malformed arriving candidate must never clear an independently
        // valid incumbent. Native never publishes a fake positive for it.
        candidate.trajectory.knots[5]=candidate.trajectory.knots[4];
        ASSERT_TRUE(f.follower.candidate(candidate));++m.bad_candidates;
      }else {
        f.native.candidate(candidate,f.reference,heading);
        ASSERT_TRUE(f.follower.candidate(candidate));
        // The production event-driven reader's result, not a test-built proof.
        const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));
        ASSERT_TRUE(proof&&proof->valid)<<(proof?proof->reason:"no candidate proof");
        ASSERT_TRUE(f.follower.validation(*proof));
      }
    }
    if(event%10!=0)continue; // The controller still runs at exactly 20 Hz.
    const auto ns=t::timeNs(t::stamp(now));
    const auto active=Access::checkCurve(f.native,f.map,ns,true);
    ASSERT_TRUE(active&&active->valid)<<(active?active->reason:"no active proof");
    ASSERT_TRUE(f.follower.validation(*active));
    if(const auto reserved=Access::checkReserved(f.native,f.map,ns))
      ASSERT_TRUE(f.follower.validation(*reserved));
    bool changed=false;
    if(const auto request=f.follower.exchangeCommit(now,now-90.)) {
      const bool reject=inject_failures&&request->disposition=="committed"&&request->trajectory_id%7==0;
      const auto incumbent=f.native.committedTrajectory();
      if(reject)Access::forgetOriginalProofs(f.native);
      const auto reply=f.native.localCommitExchangeAt(*request,ns);
      if(reject) {
        ASSERT_FALSE(reply.accepted);
        ASSERT_TRUE(f.follower.commitResult(reply,now,now-90.));++m.rollbacks;
        EXPECT_EQ(f.follower.controller().trajectoryId(),incumbent);
        EXPECT_EQ(f.native.committedTrajectory(),incumbent);
      }else {
        ASSERT_TRUE(reply.accepted)<<reply.reason<<" request="<<request->disposition;
        replies.push_back({now+ack_delay,reply});
      }
      if(request->disposition=="committed"&&!reject) {
        ++m.committed;changed=true;last_committed_at=now;
        ASSERT_EQ(f.native.committedTrajectory(),request->trajectory_id);
      }
    }
    auto permission=lease(now,++sequence);permission.version=f.reference.version;
    ASSERT_TRUE(f.follower.lease(permission,now));ASSERT_TRUE(f.native.taskLeaseAt(permission,ns));
    ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(now)));
    const auto demand=f.follower.step(now,now-90.);
    std::optional<w::LocalMotionCommand> command;
    f.native.demandAt(demand,ns,[&](const auto& message){command=message;});
    ASSERT_TRUE(command)<<demand.reason;
    const auto& v=command->velocity;
    const auto mapped=sdk::mappedSdkFractions(v.linear.x,v.linear.y,v.angular.z,
      "holonomic_xyyaw",f.policy.mapping,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(v.linear.y,0.);EXPECT_DOUBLE_EQ(mapped.left_right,0.);
    const auto acquired_ns=f.follower.controller().odometry().source_stamp_ns;
    EXPECT_EQ(t::timeNs(command->body_source_stamp),acquired_ns);
    EXPECT_EQ(command->source_stamp,t::stamp(now));
    const double source_age=now-double(acquired_ns)*1e-9;
    EXPECT_GE(source_age,-1e-9);EXPECT_LE(source_age,.010000001);
    m.max_body_age=std::max(m.max_body_age,source_age);
    const double acceleration=(v.linear.x-last_forward)/dt,jerk=(acceleration-last_acc)/dt;
    const double yaw_acc=(v.angular.z-last_yaw)/dt,yaw_jerk=(yaw_acc-last_yaw_acc)/dt;
    const double fraction_rate=(mapped.forward_back-last_fraction)/dt;
    const double yaw_fraction_rate=(mapped.yaw-last_yaw_fraction)/dt;
    EXPECT_LE(std::abs(acceleration),c.max_acceleration+1e-8);EXPECT_LE(std::abs(jerk),c.max_jerk+1e-7);
    EXPECT_LE(std::abs(yaw_acc),c.max_yaw_acceleration+1e-8);EXPECT_LE(std::abs(yaw_jerk),c.max_yaw_jerk+1e-7);
    EXPECT_LE(std::abs(fraction_rate),c.max_input_rate+1e-8);EXPECT_LE(std::abs(yaw_fraction_rate),c.max_input_rate+1e-8);
    EXPECT_LE(std::abs(fraction_rate-last_fraction_rate)/dt,c.max_input_acceleration+1e-7);
    EXPECT_LE(std::abs(yaw_fraction_rate-last_yaw_fraction_rate)/dt,c.max_input_acceleration+1e-7);
    m.max_acc=std::max(m.max_acc,std::abs(acceleration));m.max_jerk=std::max(m.max_jerk,std::abs(jerk));
    m.max_yaw_acc=std::max(m.max_yaw_acc,std::abs(yaw_acc));m.max_yaw_jerk=std::max(m.max_yaw_jerk,std::abs(yaw_jerk));
    m.max_sdk_rate=std::max(m.max_sdk_rate,std::max(std::abs(fraction_rate),std::abs(yaw_fraction_rate)));
    m.max_sdk_acc=std::max(m.max_sdk_acc,std::max(std::abs(fraction_rate-last_fraction_rate),
      std::abs(yaw_fraction_rate-last_yaw_fraction_rate))/dt);
    if(now>102.){m.min_forward=std::min(m.min_forward,v.linear.x);m.interior_stops+=command->hold||v.linear.x<=.01;}
    if(changed)m.max_commit_delta=std::max(m.max_commit_delta,std::abs(v.linear.x-last_forward));
    f.follower.controller().recordAppliedOutput(v.linear.x,v.linear.y,v.angular.z,now,now-90.);
    input=mapped;last_forward=v.linear.x;last_acc=acceleration;last_yaw=v.angular.z;last_yaw_acc=yaw_acc;
    last_fraction=mapped.forward_back;last_fraction_rate=fraction_rate;
    last_yaw_fraction=mapped.yaw;last_yaw_fraction_rate=yaw_fraction_rate;
  }
  EXPECT_GE(m.offered,69U);EXPECT_LE(m.committed,m.offered);
  if(inject_failures)EXPECT_EQ(m.committed,m.offered-m.bad_candidates-m.rollbacks)<<f.follower.candidateReason();
  else EXPECT_EQ(m.committed,70U)<<f.follower.candidateReason();
  EXPECT_EQ(m.interior_stops,0U);EXPECT_GT(m.min_forward,.10);
  EXPECT_GT(last_committed_at,107.7); // Latest-only work cannot silently strand an early reservation.
  EXPECT_GT(m.max_commit_delta,.01); // Include actual startup acceleration, not only steady output.
  if(inject_failures){EXPECT_GT(m.bad_candidates,0U);EXPECT_GT(m.rollbacks,0U);}
  EXPECT_EQ(f.follower.activeVersion()->route_hash,version().route_hash);
  std::cout<<std::setprecision(10)<<"TEN_HZ_RESULT phase="<<candidate_phase<<" ack_delay="<<ack_delay
    <<" offered="<<m.offered<<" committed="<<m.committed<<" internal_stops="<<m.interior_stops
    <<" bad_candidates="<<m.bad_candidates<<" rollbacks="<<m.rollbacks
    <<" min_forward="<<m.min_forward<<" commit_dv="<<m.max_commit_delta
    <<" max_acc="<<m.max_acc<<" max_jerk="<<m.max_jerk<<" max_yaw_acc="<<m.max_yaw_acc
    <<" max_yaw_jerk="<<m.max_yaw_jerk<<" sdk_rate="<<m.max_sdk_rate<<" sdk_acc="<<m.max_sdk_acc
    <<" max_body_age="<<m.max_body_age<<'\n';
}
TEST(RollingFollowerNative, TenHzCandidatesTwentyHzControlPreserveHistoryAcrossTransportPhases) {
  for(double phase:{0.,.025,.05})for(double delay:{0.,.025,.05})checkTenHzResults(phase,delay);
}
TEST(RollingFollowerNative, TenHzBadCandidatesAndRejectedCommitsDoNotResetIncumbentControl) {
  checkTenHzResults(.025,.025,true);
}
} // namespace
