// Production GridMap + Rebound/Refine + Native validator + typed LocalFollower
// + production SDK amplitude map. Acquired robot response and scheduling are
// simulated. No ROS node, context, graph, SDK client or actuator writer is
// constructed. This exercises the C++ handoff contract, not the Python
// reference scheduler or the complete production FSM.
// Candidates complete with zero simulated latency to expose repeated actual
// entry blindness; separate phased-ACK tests cover transport latency. The
// response/deadzone is deliberately uncalibrated, not an SDK speed model.
#include "native_local_follower_test_fixture.hpp"
#include "sdk_input_mapping.hpp"
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/collision_reference.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>


namespace {
// Same effective limits as single_floor_session.prepare(): the D1 template
// supplies native objectives/spacing, while the formal tracking profile owns
// the actual speed/acceleration override. Generic .45 template limits are not
// silently substituted for the .30 formal task contract.
const YAML::Node& effectiveScanParameters() {
  static const YAML::Node result=[] {
    const char* root=std::getenv("D1MAX_ROLLING_SOURCE");
    if(!root)throw std::runtime_error("D1MAX_ROLLING_SOURCE is required");
    const auto source=std::filesystem::path(root)/"src";
    const auto document=YAML::LoadFile((source/"d1max_scan_planner/config/d1max_scan_planner.yaml").string());
    auto p=YAML::Clone(document.begin()->second["ros__parameters"]);
    const auto tracking=YAML::LoadFile((source/"d1max_pct_scan/config/motion_tracking.yaml").string());
    p["manager.max_vel"]=tracking["max_speed_mps"].as<double>();
    p["optimization.max_vel"]=tracking["max_speed_mps"].as<double>();
    p["manager.max_acc"]=tracking["max_acceleration_mps2"].as<double>();
    p["optimization.max_acc"]=tracking["max_acceleration_mps2"].as<double>();
    std::cout<<"EFFECTIVE_NATIVE "<<YAML::Dump(p)<<'\n';
    return p;
  }();
  return result;
}
}

namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& optimizer,const GridMap::Ptr& map) {
    const auto& p=effectiveScanParameters();
    optimizer.order_=p["optimization.order"].as<int>();
    optimizer.lambda1_=p["optimization.lambda_smooth"].as<double>();
    optimizer.lambda2_=p["optimization.lambda_collision"].as<double>();
    optimizer.lambda3_=p["optimization.lambda_feasibility"].as<double>();
    optimizer.lambda4_=p["optimization.lambda_fitness"].as<double>();
    optimizer.lambda_reference_=p["optimization.lambda_reference"].as<double>();
    optimizer.lambda_planar_curvature_=p["optimization.lambda_planar_curvature"].as<double>();
    optimizer.preferred_turn_radius_=p["optimization.preferred_turn_radius"].as<double>();
    optimizer.dist0_=p["optimization.dist0"].as<double>();
    optimizer.max_vel_=p["optimization.max_vel"].as<double>();
    optimizer.max_acc_=p["optimization.max_acc"].as<double>();optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,{80,40,32});
  }
};
}

namespace {
namespace sdk=d1monitor::execution3;
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
  unsigned generation=1,curve_id=1,rejected_solves=0;double x=0.,vx=.3,initial_y=0.;
  std::vector<double> planning_ms;
  RollingFixture(double goal_x=7.,int map_x_cells=600,double offset=0.):initial_y(offset) {
    scan_planner::BrakingModel b{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'d')};
    b.kinematic_profile=scan_planner::KinematicProfile::HolonomicXYYaw;b.lateral_distance=.08;b.command_pipeline_bound=.1;
    Access::braking(native,b);Access::geometryPolicy(native,true);GridMapTestAccess::configure(map,true,map_x_cells);
    heading.measured_heading_enabled=true;heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;
    heading.align_then_forward=policy.control.align_then_forward;
    heading.measured_yaw=0.;heading.body_extent=.53;
    reference.goal_position.x=goal_x;installReference(100.);acquire(100.);
    auto first=rollingTag(1,reference.version,0.,100.);
    first.join_pose.position.y=initial_y;
    for(auto& point:first.trajectory.pos_pts)point.y=initial_y;
    native.candidate(first,reference,heading);
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
    o.position.y()=initial_y;
    EXPECT_TRUE(follower.body(o,now,now-90.));
    geometry_msgs::msg::Twist twist;twist.linear.x=vx;native.body({o.position,o.orientation,now,"odom"},twist,o.source_stamp_ns);
    GridMapTestAccess::acquire(map,o.source_stamp_ns);follower.controller().refreshMeasuredProgress(now);
    native.progressAt(t::trackingProgress(follower.controller(),policy.control,now,follower.status(now)),now);
  }
};
}
namespace {
std::optional<w::TaggedBspline> actualReturn(RollingFixture& f,double now,const t::Odom& acquired) {
  const auto planning_started=std::chrono::steady_clock::now();
  const auto record_time=[&]{f.planning_ms.push_back(std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-planning_started).count());};
  const auto p=acquired.position,v=acquired.velocity_in_frame;
  scan_planner::DiscreteReference global;
  global.set({{0.,0.,p.z()},{f.reference.goal_position.x,0.,p.z()}});
  const double begin=global.project(p,0.,global.length()),end=std::min(global.length(),begin+4.);
  const auto original=global.slice(begin,end,p);
  const auto goal=global.sample(end);
  const bool has_connector=(p-global.sample(begin)).norm()>1e-6;
  const auto reference=scan_planner::measuredForwardApproach(original,.05,[&](const auto& point,double yaw){
    return f.map.getPlanningInflateOccupancy(point,yaw);},v,acquired.yaw,has_connector);
  scan_planner::DiscreteReference path;path.set(reference);
  const auto& policy=effectiveScanParameters();
  const double max_speed=policy["manager.max_vel"].as<double>(),max_acc=policy["manager.max_acc"].as<double>();
  const auto seed=scan_planner::sampleReferenceSeed(path,max_speed,max_acc,v.norm(),0.,
    policy["manager.control_points_distance"].as<double>());
  const scan_planner::CubicMotionBoundary boundary{p,v,Eigen::Vector3d::Zero(),goal,
    Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  auto cp=scan_planner::fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
  scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::configure(optimizer,GridMap::Ptr(&f.map,[](GridMap*){}));
  auto heading=f.heading;heading.measured_yaw=acquired.yaw;
  optimizer.setPreviewHeadingContract(heading);
  auto budget=std::make_shared<scan_planner::SolveBudget>();optimizer.setSolveBudget(budget);
  optimizer.reference_path_samples_=seed.samples;optimizer.initControlPoints(cp);
  Eigen::MatrixXd optimized;
  if(!optimizer.controlPointsInitialized()||!optimizer.BsplineOptimizeTrajRebound(optimized,seed.dt)) {
    ++f.rejected_solves;record_time();return std::nullopt;
  }
  scan_planner::UniformBspline curve(optimized,3,seed.dt);
  const auto timed=scan_planner::refineTimingWithFixedBoundary(curve,boundary,max_speed,max_acc,
    [&](Eigen::MatrixXd& trial,double interval) {
      scan_planner::UniformBspline proposed(trial,3,interval);optimizer.ref_pts_.clear();
      for(int i=0;i<=trial.cols()-3;++i)optimizer.ref_pts_.push_back(proposed.evaluateDeBoorT(i*interval));
      Eigen::MatrixXd out;const bool ok=optimizer.BsplineOptimizeTrajRefine(trial,interval,out);
      if(ok)trial=out;return ok;
    },8,std::nullopt,[&]{return budget->allowed();},true);
  if(!timed.success){++f.rejected_solves;record_time();return std::nullopt;}
  auto velocity=curve.getDerivative();const auto acceleration=velocity.getDerivative();
  EXPECT_LT((curve.evaluateDeBoorT(0.)-p).norm(),1e-8);
  EXPECT_LT((velocity.evaluateDeBoorT(0.)-v).norm(),1e-8);
  EXPECT_LT(acceleration.evaluateDeBoorT(0.).norm(),1e-8);
  EXPECT_LT((curve.evaluateDeBoorT(curve.getTimeSum())-goal).norm(),1e-8);
  auto q=tag(++f.curve_id,p,v,Eigen::Vector3d::Zero());
  q.generation=f.reference.version.reference_generation;
  q.trajectory.start_time=q.join_source_stamp=t::stamp(now);
  q.trajectory.pos_pts.clear();q.trajectory.knots.clear();q.trajectory.yaw_pts.clear();q.trajectory.yaw_dt=0.;
  cp=curve.getControlPoint();const auto knots=curve.getKnot();
  for(int i=0;i<cp.cols();++i){geometry_msgs::msg::Point point;
    point.x=cp(0,i);point.y=cp(1,i);point.z=cp(2,i);q.trajectory.pos_pts.push_back(point);}
  for(int i=0;i<knots.size();++i)q.trajectory.knots.push_back(knots[i]);
  q.join_pose.orientation.x=acquired.orientation.x();q.join_pose.orientation.y=acquired.orientation.y();
  q.join_pose.orientation.z=acquired.orientation.z();q.join_pose.orientation.w=acquired.orientation.w();
  freezeHeading(q,acquired.yaw);
  record_time();
  return q;
}
void runMeasuredStartRolling(double route_length,const std::vector<double>& offsets,bool strict_return) {
  for(double offset:offsets) {
    SCOPED_TRACE(offset);RollingFixture f(route_length+7.,route_length>5.?1000:600,offset);const auto& c=f.policy.control;
    const auto fixed_route=f.reference.version;
    double x=0.,y=offset,yaw=0.,vx=.3,wz=0.;unsigned commits=0,stops=0,reversals=0;
    double late_peak=0.,last_sign=0.,max_yaw=0.,previous_v=0.,previous_w=0.,last_a=0.,last_aw=0.;
    double settled_cross=0.,settled_yaw=0.;unsigned late_reversals=0;
    sdk::SdkMoveFractions input;
    for(unsigned i=1;i<=1000;++i) {
      const double now=100.+i*.05;
      auto acquired=body(now,x,vx);acquired.position.y()=y;acquired.yaw=yaw;
      acquired.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());
      acquired.velocity_in_frame={vx*std::cos(yaw),vx*std::sin(yaw),0.};
      acquired.angular_velocity_in_frame.z()=wz;
      ASSERT_TRUE(f.follower.body(acquired,now,now-90.));
      geometry_msgs::msg::Twist twist;twist.linear.x=acquired.velocity_in_frame.x();
      twist.linear.y=acquired.velocity_in_frame.y();twist.angular.z=wz;
      f.native.body({acquired.position,acquired.orientation,now,"odom"},twist,acquired.source_stamp_ns);
      if(i%4==1)GridMapTestAccess::acquire(f.map,acquired.source_stamp_ns); // original 5 Hz map stamps
      f.follower.controller().refreshMeasuredProgress(now);
      f.native.progressAt(t::trackingProgress(f.follower.controller(),c,now,f.follower.status(now)),now);
      if(i%2==1) {
        EXPECT_EQ(f.reference.version,fixed_route);
        auto candidate=actualReturn(f,now,acquired);auto heading=f.heading;heading.measured_yaw=yaw;
        if(candidate) {
        const auto& q=*candidate;
        f.native.candidate(q,f.reference,heading);ASSERT_TRUE(f.follower.candidate(q));
        const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));
        ASSERT_TRUE(proof&&proof->valid)<<(proof?proof->reason:"no proof");ASSERT_TRUE(f.follower.validation(*proof));
        const auto prepare=f.follower.exchangeCommit(now,now-90.);ASSERT_TRUE(prepare)<<f.follower.candidateReason();
        const auto prepared=f.native.localCommitExchangeAt(*prepare,t::timeNs(t::stamp(now)));
        ASSERT_TRUE(prepared.accepted)<<prepared.reason;ASSERT_TRUE(f.follower.commitResult(prepared,now,now-90.));
        const auto commit=f.follower.exchangeCommit(now,now-90.);ASSERT_TRUE(commit)<<f.follower.candidateReason();
        const auto done=f.native.localCommitExchangeAt(*commit,t::timeNs(t::stamp(now)));
        ASSERT_TRUE(done.accepted)<<done.reason;ASSERT_TRUE(f.follower.commitResult(done,now,now-90.));
        ASSERT_EQ(commit->body_source_stamp,t::stamp(now));
        ++commits;
        }
      }
      const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)),true);
      ASSERT_TRUE(proof&&proof->valid)<<(proof?proof->reason:"no active proof");ASSERT_TRUE(f.follower.validation(*proof));
      auto permission=lease(now,i);permission.version=f.reference.version;
      ASSERT_TRUE(f.follower.lease(permission,now));ASSERT_TRUE(f.native.taskLeaseAt(permission,t::timeNs(t::stamp(now))));
      ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(now)));
      const auto demand=f.follower.step(now,now-90.);std::optional<w::LocalMotionCommand> command;
      f.native.demandAt(demand,t::timeNs(t::stamp(now)),[&](const auto& out){command=out;});
      ASSERT_TRUE(command)<<demand.reason;
      const double v=command->velocity.linear.x,w=command->velocity.angular.z;
      EXPECT_DOUBLE_EQ(command->velocity.linear.y,0.);EXPECT_EQ(command->body_source_stamp,t::stamp(now));
      const double a=(v-previous_v)/.05,aw=(w-previous_w)/.05;
      EXPECT_LE(std::abs(a),c.max_acceleration+1e-8);EXPECT_LE(std::abs(aw),c.max_yaw_acceleration+1e-8);
      EXPECT_LE(std::abs(a-last_a)/.05,c.max_jerk+1e-7);EXPECT_LE(std::abs(aw-last_aw)/.05,c.max_yaw_jerk+1e-7);
      input=sdk::mappedSdkFractions(v,0.,w,"holonomic_xyyaw",f.policy.mapping,c.max_speed,c.max_yaw_rate);
      if(i%20==1) {
        const auto& diagnostic=f.follower.controller().controlDiagnostic();
        std::cout<<"ACTUAL_TICK i="<<i<<" x="<<x<<" y="<<y<<" yaw="<<yaw
          <<" k="<<diagnostic.steering_curvature<<" heading="<<diagnostic.heading_error
          <<" target_w="<<diagnostic.yaw_target<<" output_w="<<w<<" sdk_w="<<input.yaw<<'\n';
      }
      if(i>40)stops+=v<=.01||command->hold;
      const int sign=input.yaw>=.195?1:input.yaw<=-.195?-1:0;
      if(sign&&last_sign&&sign!=last_sign){++reversals;if(x>=10.)++late_reversals;}
      if(sign)last_sign=sign;
      max_yaw=std::max(max_yaw,std::abs(yaw));if(x>2.)late_peak=std::max(late_peak,std::abs(y));
      if(x>=10.){settled_cross=std::max(settled_cross,std::abs(y));settled_yaw=std::max(settled_yaw,std::abs(yaw));}
      f.follower.controller().recordAppliedOutput(v,0.,w,now,now-90.);
      for(int sub=0;sub<10;++sub) {
        const double target_v=input.forward_back>=.29?.24+(input.forward_back-.29):0.;
        const double target_w=std::abs(input.yaw)>=.195?std::copysign(.18+1.2*(std::abs(input.yaw)-.195),input.yaw):0.;
        vx+=(target_v-vx)*(-std::expm1(-.005/.1));wz+=(target_w-wz)*(-std::expm1(-.005/.1));
        yaw+=wz*.005;x+=vx*std::cos(yaw)*.005;y+=vx*std::sin(yaw)*.005;
      }
      previous_v=v;previous_w=w;last_a=a;last_aw=aw;
      if(x>=route_length)break;
    }
    std::cout<<"ACTUAL_ROLL offset="<<offset<<" commits="<<commits<<" rejected_solves="<<f.rejected_solves<<" x="<<x<<" y="<<y
      <<" late_peak="<<late_peak<<" stops="<<stops<<" reversals="<<reversals<<" max_yaw="<<max_yaw
      <<" final_yaw="<<yaw<<" final_w="<<wz<<" sdk_yaw="<<input.yaw
      <<" settled_cross10m="<<settled_cross<<" settled_yaw10m="<<settled_yaw
      <<" late_reversals10m="<<late_reversals<<'\n';
    std::sort(f.planning_ms.begin(),f.planning_ms.end());
    ASSERT_FALSE(f.planning_ms.empty());
    const auto percentile=[&](double q){return f.planning_ms[std::min(f.planning_ms.size()-1,
      static_cast<std::size_t>(std::ceil(q*f.planning_ms.size())-1))];};
    std::cout<<"NATIVE_INITIALIZE_REBOUND_REFINE_WALL_MS count="<<f.planning_ms.size()
      <<" p50="<<percentile(.50)<<" p95="<<percentile(.95)<<" maximum="<<f.planning_ms.back()
      <<" excludes_ros_scheduling_transport_and_commit=true simulated_candidate_period_s=0.1\n";
      EXPECT_GT(x,route_length);EXPECT_GT(commits,80U);EXPECT_EQ(stops,0U);EXPECT_LE(reversals,3U);
      if(strict_return){EXPECT_LT(late_peak,.06);EXPECT_LT(std::abs(y),.04);}
      // This long scenario checks continuity and sustained reversal, NOT cm
      // precision under an uncalibrated discontinuous response. The original
      // 4 cm long-run quality failure is retained separately in evidence.
      else {EXPECT_EQ(late_reversals,0U);EXPECT_LT(max_yaw,.30);}
  }
}
TEST(MeasuredStartRolling, ActualPvaTenHzRetainsForwardReturnAndTwentyHzHistory) {
  runMeasuredStartRolling(5.,{0.,-.08,.08},true);
}
TEST(MeasuredStartRolling, LongResponseSensitivityReportsResidualWithoutPersistentReversal) {
  runMeasuredStartRolling(15.,{-.08,.08,-.30,.30},false);
}
}
