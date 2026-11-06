// SDK-free sensitivity fixture, NOT a D1 Max physical calibration/acceptance.
// Compiles the real TrackerCore, SCAN spline math and sdk_input_mapping.hpp;
// does not instantiate ROS, vendor clients, a socket, or a robot writer.
//
// Both closed loops use the same trajectory/controller settings, but their
// controller demands necessarily diverge after their physical feedback does.
// A separate paired replay applies BOTH mappings to each exact demand sample.
// Old linear scales 1/1/2 come from runtime_motion_profile.json; the units are
// nominal compatibility values and not a measured physical response curve.
//
// Explicit ASSUMPTIONS: 20 Hz controller, exact/noiseless world pose feedback;
// command transport delay is 0.05--0.60 s; each body-axis has a first-order
// response tau 0.25--1.8 s, including coasting after raw zero. The equilibrium
// response is odd in input: below `knee`, creep_at_knee * |u| / knee; above it,
// linear interpolation from creep_at_knee to speed_at_30 at 30% input. These
// are invented sensitivity models, consistent only qualitatively with the
// user's '<20% little, 20% slow, 30% more normal' observation. One model has
// strict 20% stiction; another raises it to 24%; high gain intentionally allows
// >0.3 m/s so a demand cap cannot be mistaken for a physical velocity bound.
// Actual pose/twist below come ONLY from independently integrating the plant.
// We never feed the requested velocity back as a supposed measurement.
//
// Version/proof admission is disabled ONLY for this geometry/control fixture,
// as in test_holonomic_tracker.cpp. Fresh immutable geometry is retained during
// the test; this does not provide a production SCAN proof or new task gate.
// Arrival failures and reversals are reported; they are not suppressed to make
// the test pass. Contract assertions test math/invariants, NOT navigation QA.
//
// Current acceptance groups: SdkV4ContinuousControl and SdkCurrentProfile load
// BOTH live source configuration files and use independent supplied states.
// SdkMappingInertia is an opt-in HISTORICAL compatibility/sensitivity suite:
// its V1/V2/V3 mapping, holonomic control and invented inertial plants are NOT
// current defaults. In particular, its plant arrival counts are experiments,
// not claims about measured D1 dynamics or formal deployment acceptance.

#include <gtest/gtest.h>
#include <array>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/response_command_limiter.hpp"
#include "sdk_input_mapping.hpp"

namespace {
using namespace d1max_trajectory_tracker;
namespace sdk = d1monitor::execution3;
constexpr double kDt=.05, kRosStart=100., kReceiptStart=10.;
constexpr double kPi=3.14159265358979323846;

Config config() {
  Config c;c.session_id="sdk-inertia-fixture";c.require_versioned_identity=false;
  c.motion_model="holonomic";c.holonomic_yaw_policy="hold_heading";
  c.kp_position=.6;c.kp_yaw=.8;c.max_acceleration=.2;c.max_yaw_acceleration=.4;
  c.velocity_damping_gain=.35;c.yaw_damping_gain=.35;
  c.lookahead=.8;c.min_lookahead_distance_m=.07;
  c.max_speed=.3;c.max_yaw_rate=.5;return c;
}
sdk::SdkInputMapping mapping() {sdk::SdkInputMapping m;m.enabled=true;return m;}
Eigen::Vector2d planar(const Output&o) {return {o.forward,o.lateral};}
Eigen::Vector2d fractions(const sdk::SdkMoveFractions&f) {return {f.forward_back,f.left_right};}
bool rawZero(const Output&o) {return o.forward==0.&&o.lateral==0.&&o.yaw_rate==0.;}

struct PlantParameters {
  std::string name;
  double knee,creep_at_knee,speed_at_30,yaw_at_30,translation_tau,yaw_tau,delay;
};
const std::array<PlantParameters,5> kPlants{{
  {"soft_short_delay",.18,.015,.30,.50,.25,.30,.05},
  {"slow_large_body",.20,.008,.30,.50,.80,.90,.20},
  {"strict_deadzone",.20,0.,.30,.50,1.20,1.40,.35},
  {"high_friction",.24,0.,.25,.40,1.20,1.40,.35},
  {"high_gain_long_delay",.20,.004,.45,.75,1.80,1.80,.60}
}};
struct DelayedInput {double time;sdk::SdkMoveFractions value;};
struct Plant {
  PlantParameters p;
  Eigen::Vector3d position{0.,0.,.55};Eigen::Vector2d body_velocity{0.,0.};
  double yaw=0.,yaw_velocity=0.;std::deque<DelayedInput> pipeline;
  sdk::SdkMoveFractions applied;
  explicit Plant(PlantParameters parameters):p(std::move(parameters)){}
  double response(double fraction,double maximum)const {
    const double input=std::abs(fraction);
    const double creep=p.creep_at_knee*(maximum/p.speed_at_30);
    const double magnitude=input<=p.knee?creep*input/p.knee:
      creep+(maximum-creep)*(input-p.knee)/(.30-p.knee);
    return std::copysign(magnitude,fraction);
  }
  void advance(const sdk::SdkMoveFractions&input,double time) {
    pipeline.push_back({time+p.delay,input});
    constexpr int substeps=10;constexpr double h=kDt/substeps;
    for(int i=0;i<substeps;++i) {
      const double t=time+(i+1)*h;
      while(!pipeline.empty()&&pipeline.front().time<=t+1e-10) {
        applied=pipeline.front().value;pipeline.pop_front();
      }
      const Eigen::Vector2d equilibrium(response(applied.forward_back,p.speed_at_30),
        response(applied.left_right,p.speed_at_30));
      body_velocity+=(equilibrium-body_velocity)*(1.-std::exp(-h/p.translation_tau));
      yaw_velocity+=(response(applied.yaw,p.yaw_at_30)-yaw_velocity)*(1.-std::exp(-h/p.yaw_tau));
      yaw=angle(yaw+yaw_velocity*h);
      position.head<2>()+=Eigen::Rotation2Dd(yaw)*body_velocity*h;
    }
  }
  Odom odom(double now)const {
    Odom o;o.frame_id="d1max_loc_odom";o.child_frame_id="d1max_loc_base_link";
    o.stamp=o.posterior_stamp=o.imu_stamp=now;o.source_stamp_ns=std::llround(now*1e9);
    o.position=position;o.yaw=yaw;o.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());
    const Eigen::Vector2d world=Eigen::Rotation2Dd(yaw)*body_velocity;
    o.velocity_in_frame={world.x(),world.y(),0.};o.planar_speed=body_velocity.norm();
    o.angular_velocity_in_frame={0.,0.,yaw_velocity};return o;
  }
};

struct Scenario {
  std::string name;Eigen::Vector2d direction;double cross_offset=0.,initial_yaw=0.,frozen_yaw=0.;
  bool pure_yaw=false,replace=false,cancel=false,estop=false;
};
const std::array<Scenario,10> kScenarios{{
  {"zero_tangent_start",{1.,0.}},
  {"straight_cross_error",{1.,0.},.25},
  {"diagonal_45",{std::sqrt(.5),std::sqrt(.5)}},
  {"pure_lateral",{0.,1.}},
  {"reverse",{-1.,0.}},
  {"translate_turn_90",{1.,0.},0.,0.,kPi/2.},
  {"pure_yaw_90",{1.,0.},0.,0.,kPi/2.,true},
  {"curve_replacement",{1.,0.},0.,0.,0.,false,true},
  {"cancel_raw_zero",{1.,0.},0.,0.,0.,false,false,true},
  {"estop_raw_zero",{1.,0.},0.,0.,0.,false,false,false,true}
}};

// Increasing non-clamped SCAN knots and native derivative controls within
// .3 m/s/.2 m/s^2; three repeated endpoint controls yield zero start/end speed.
Trajectory curve(const Scenario&s,std::int64_t id=1,double now=kRosStart) {
  Trajectory t;t.session_id="sdk-inertia-fixture";t.frame_id="d1max_loc_odom";
  t.generation=1;t.id=id;t.start_time=now;t.frozen_yaw=s.frozen_yaw;
  double distance=0.;const auto point=[&](double d) {
    t.points.emplace_back(s.direction.x()*d,s.direction.y()*d,.55);
  };
  point(0.);point(0.);point(0.);
  for(double increment:{.02,.04,.06,.08}){distance+=increment;point(distance);}
  for(int i=0;i<16;++i){distance+=.10;point(distance);}
  for(double increment:{.08,.06,.04,.02,0.,0.}){distance+=increment;point(distance);}
  for(std::size_t i=0;i<t.points.size()+4;++i)t.knots.push_back((double(i)-3.)*.4);
  return t;
}
Task task(const Trajectory&t) {
  Task task;task.session_id=t.session_id;task.frame_id=t.frame_id;task.generation=1;
  task.active=true;task.issued_at=kRosStart;task.goal=t.points.back();return task;
}

struct SignCounter {
  int last=0,reversals=0;
  void update(double value,double threshold) {
    const int sign=value>threshold?1:value<-threshold?-1:0;
    if(sign&&last&&sign!=last)++reversals;
    if(sign)last=sign;
  }
};
struct Metrics {
  std::string plant,scenario,mode,final_reason,first_fault;
  bool arrived=false,replacement_accepted=false,zero_immediate=false;
  double arrival_time=-1.,stop_time=-1.,final_error=0.,final_yaw_error=0.;
  double max_cross_error=0.,final_cross_error=0.,max_speed=0.,max_yaw_speed=0.;
  double max_demand=0.,max_yaw_demand=0.,max_fraction=0.,max_yaw_fraction=0.;
  double max_slew=0.,max_yaw_slew=0.,max_zero_jump=0.,post_zero_distance=0.;
  double same_input_old_max=0.,same_input_new_max=0.,same_input_rms_delta=0.;
  int samples=0,request_reversals=0,sdk_reversals=0,physical_reversals=0;
  int yaw_request_reversals=0,yaw_sdk_reversals=0,yaw_physical_reversals=0;
  int cross_sign_changes=0;
};
std::ofstream summary,trace;
void writeSummary(const Metrics&m) {
  summary<<m.plant<<','<<m.scenario<<','<<m.mode<<','<<m.arrived<<','<<m.arrival_time<<','<<m.stop_time
    <<','<<m.final_error<<','<<m.final_yaw_error<<','<<m.max_cross_error<<','<<m.final_cross_error
    <<','<<m.max_speed<<','<<m.max_yaw_speed<<','<<m.max_demand<<','<<m.max_yaw_demand
    <<','<<m.max_fraction<<','<<m.max_yaw_fraction<<','<<m.max_slew<<','<<m.max_yaw_slew
    <<','<<m.max_zero_jump<<','<<m.post_zero_distance<<','<<m.request_reversals<<','<<m.sdk_reversals
    <<','<<m.physical_reversals<<','<<m.yaw_request_reversals<<','<<m.yaw_sdk_reversals
    <<','<<m.yaw_physical_reversals<<','<<m.cross_sign_changes<<','<<m.replacement_accepted
    <<','<<m.zero_immediate<<','<<m.same_input_old_max<<','<<m.same_input_new_max
    <<','<<m.same_input_rms_delta<<','<<m.final_reason<<','<<m.first_fault<<'\n';
}

Metrics run(const PlantParameters&parameters,const Scenario&s,bool nonlinear) {
  const auto c=config();TrackerCore core(c);auto t=curve(s);auto goal=task(t);Plant body(parameters);
  body.yaw=s.initial_yaw;body.position.head<2>()+=Eigen::Vector2d(-s.direction.y(),s.direction.x())*s.cross_offset;
  Metrics m;m.plant=parameters.name;m.scenario=s.name;m.mode=nonlinear?"mapped":"linear";
  EXPECT_TRUE(core.receiveTask(goal,kRosStart,kReceiptStart));
  EXPECT_TRUE(core.receiveOdom(body.odom(kRosStart),kRosStart,kReceiptStart));
  EXPECT_TRUE(core.receiveTrajectory(t,kRosStart,kReceiptStart))<<core.candidateReason();
  const auto geometry=PreparedGeometry::build(c,t,{});EXPECT_TRUE(geometry->derivatives_valid);
  sdk::SdkFractionSlew slew,replay_slew;const auto map=mapping();
  const sdk::SdkMappingIdentity identity{"same-writer-execution",1,1};
  sdk::SdkMoveFractions previous_fraction;
  std::array<SignCounter,3> requested_sign,sdk_sign,physical_sign;SignCounter cross_sign;
  Eigen::Vector2d zero_position{0.,0.};bool zero_triggered=false,yaw_complete=false;
  double squared_replay_delta=0.,stationary_since=-1.;
  for(int i=1;i<=1600;++i) {
    const double elapsed=i*kDt,now=kRosStart+elapsed,receipt=kReceiptStart+elapsed;
    const auto measured=body.odom(now);EXPECT_EQ(measured.position,body.position);
    EXPECT_NEAR(measured.planar_speed,body.body_velocity.norm(),1e-15);
    EXPECT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    if(s.replace&&i==16) {
      t=curve(s,2,now);m.replacement_accepted=core.receiveTrajectory(t,now,receipt);
      EXPECT_TRUE(m.replacement_accepted)<<core.candidateReason();
    }
    if((s.cancel||s.estop)&&elapsed>=6.&&!zero_triggered) {
      if(s.cancel)core.cancel("fixture_cancel");else core.hold("fixture_estop",receipt);
      zero_triggered=true;zero_position=body.position.head<2>();
    }
    // An emergency stop stays asserted at the existing writer until cleared.
    // TrackerCore::hold alone is a recoverable input hold, not that latch.
    if(s.estop&&zero_triggered)core.hold("fixture_estop",receipt);
    Output output=s.pure_yaw?core.align(s.frozen_yaw,.07,now,receipt,true):core.step(now,receipt,true);
    if(s.pure_yaw&&!yaw_complete&&std::abs(angle(s.frozen_yaw-body.yaw))<=.07&&
       std::abs(body.yaw_velocity)<=c.stationary_angular_threshold_radps&&std::abs(output.yaw_rate)<1e-12) {
      yaw_complete=true;core.cancel("fixture_yaw_complete");output=core.step(now,receipt,true);
    }
    // Feed the tracker the last accepted SI demand, NOT SDK fractions and NOT
    // an invented inverse mapping. Measured pose/twist arrive independently.
    core.recordAppliedOutput(output.forward,output.lateral,output.yaw_rate,now,receipt);
    const auto old=sdk::sdkMoveFractions(output.forward,output.lateral,output.yaw_rate,
      "holonomic_xyyaw",1.,1.,2.);
    const auto target=sdk::mappedSdkFractions(output.forward,output.lateral,output.yaw_rate,
      "holonomic_xyyaw",map,c.max_speed,c.max_yaw_rate);
    const auto replay=replay_slew.apply(target,output.forward,output.lateral,output.yaw_rate,map,identity,elapsed);
    const auto sent=nonlinear?slew.apply(target,output.forward,output.lateral,output.yaw_rate,map,identity,elapsed):old;
    squared_replay_delta+=(fractions(old)-fractions(replay)).squaredNorm()+std::pow(old.yaw-replay.yaw,2);
    m.same_input_old_max=std::max(m.same_input_old_max,fractions(old).norm());
    m.same_input_new_max=std::max(m.same_input_new_max,fractions(replay).norm());
    EXPECT_LE(planar(output).norm(),c.max_speed+1e-10);EXPECT_LE(std::abs(output.yaw_rate),c.max_yaw_rate+1e-10);
    EXPECT_LE(fractions(sent).norm(),.3+1e-10);EXPECT_LE(std::abs(sent.yaw),.3+1e-10);
    const double delta=(fractions(sent)-fractions(previous_fraction)).norm()/kDt;
    const double yaw_delta=std::abs(sent.yaw-previous_fraction.yaw)/kDt;
    if(rawZero(output)) {
      m.max_zero_jump=std::max(m.max_zero_jump,(fractions(sent)-fractions(previous_fraction)).norm());
      EXPECT_DOUBLE_EQ(fractions(sent).norm(),0.);EXPECT_DOUBLE_EQ(sent.yaw,0.);
      if(zero_triggered&&!m.zero_immediate)m.zero_immediate=true;
    } else {
      m.max_slew=std::max(m.max_slew,delta);m.max_yaw_slew=std::max(m.max_yaw_slew,yaw_delta);
      if(nonlinear){EXPECT_LE(delta,.45+1e-8);EXPECT_LE(yaw_delta,.45+1e-8);}
    }
    const Eigen::Vector2d cross_normal(-s.direction.y(),s.direction.x());
    const double cross=body.position.head<2>().dot(cross_normal);
    m.max_cross_error=std::max(m.max_cross_error,std::abs(cross));cross_sign.update(cross,.005);
    const std::array<double,3> requested{{output.forward,output.lateral,output.yaw_rate}};
    const std::array<double,3> command{{sent.forward_back,sent.left_right,sent.yaw}};
    const std::array<double,3> physical{{body.body_velocity.x(),body.body_velocity.y(),body.yaw_velocity}};
    for(int axis=0;axis<3;++axis) {
      requested_sign[axis].update(requested[axis],.005);sdk_sign[axis].update(command[axis],.01);
      physical_sign[axis].update(physical[axis],axis==2?.01:.005);
    }
    m.max_speed=std::max(m.max_speed,body.body_velocity.norm());
    m.max_yaw_speed=std::max(m.max_yaw_speed,std::abs(body.yaw_velocity));
    m.max_demand=std::max(m.max_demand,planar(output).norm());m.max_yaw_demand=std::max(m.max_yaw_demand,std::abs(output.yaw_rate));
    m.max_fraction=std::max(m.max_fraction,fractions(sent).norm());m.max_yaw_fraction=std::max(m.max_yaw_fraction,std::abs(sent.yaw));
    if(output.reason!="tracking"&&output.reason!="goal_position_reached"&&output.reason!="goal_position_settling"&&
       output.reason!="final_endpoint_capture"&&output.reason!="goal_reached"&&
       output.reason!="aligning"&&output.reason!="fixture_cancel"&&output.reason!="fixture_estop"&&
       output.reason!="fixture_yaw_complete"&&m.first_fault.empty())m.first_fault=output.reason;
    const double error=(body.position-goal.goal).head<2>().norm();
    const double yaw_error=std::abs(angle(s.frozen_yaw-body.yaw));
    const bool arrival=s.pure_yaw?yaw_complete:output.finished;
    if(arrival&&!m.arrived){m.arrived=true;m.arrival_time=elapsed;}
    const bool stopped=zero_triggered||m.arrived;
    const bool stationary=stopped&&rawZero(output)&&body.body_velocity.norm()<=c.stationary_linear_threshold_mps&&
      std::abs(body.yaw_velocity)<=c.stationary_angular_threshold_radps;
    if(!stationary){stationary_since=-1.;m.stop_time=-1.;}
    else {
      if(stationary_since<0.)stationary_since=elapsed;
      if(m.stop_time<0.&&elapsed-stationary_since+1e-9>=c.stationary_reentry_duration_s)m.stop_time=elapsed;
    }
    if(trace.is_open())trace<<m.plant<<','<<s.name<<','<<m.mode<<','<<elapsed<<','<<body.position.x()<<','<<body.position.y()
      <<','<<body.yaw<<','<<physical[0]<<','<<physical[1]<<','<<physical[2]<<','<<requested[0]<<','<<requested[1]
      <<','<<requested[2]<<','<<command[0]<<','<<command[1]<<','<<command[2]<<','<<error<<','<<yaw_error<<','<<output.reason
      <<','<<old.forward_back<<','<<old.left_right<<','<<old.yaw
      <<','<<replay.forward_back<<','<<replay.left_right<<','<<replay.yaw<<'\n';
    body.advance(sent,elapsed);previous_fraction=sent;++m.samples;
    m.final_reason=output.reason;
    // Keep several seconds of coasting after completion/cancel; a zero SDK
    // command is not a physical stop. Failed scenes receive a full 80 s run.
    if((m.arrived&&elapsed>=m.arrival_time+8.)||(zero_triggered&&elapsed>=14.))break;
  }
  m.final_error=(body.position-goal.goal).head<2>().norm();m.final_yaw_error=std::abs(angle(s.frozen_yaw-body.yaw));
  m.final_cross_error=std::abs(body.position.head<2>().dot(Eigen::Vector2d(-s.direction.y(),s.direction.x())));
  m.request_reversals=requested_sign[0].reversals+requested_sign[1].reversals;
  m.sdk_reversals=sdk_sign[0].reversals+sdk_sign[1].reversals;m.physical_reversals=physical_sign[0].reversals+physical_sign[1].reversals;
  m.yaw_request_reversals=requested_sign[2].reversals;m.yaw_sdk_reversals=sdk_sign[2].reversals;
  m.yaw_physical_reversals=physical_sign[2].reversals;m.cross_sign_changes=cross_sign.reversals;
  m.same_input_rms_delta=std::sqrt(squared_replay_delta/std::max(1,m.samples));
  if(zero_triggered)m.post_zero_distance=(body.position.head<2>()-zero_position).norm();
  EXPECT_TRUE(body.position.allFinite());EXPECT_TRUE(body.body_velocity.allFinite());
  return m;
}

TEST(SdkMappingInertia, SignedAllAxesAndDiagonalPreserveDirectionUnderNormCap) {
  const auto map=mapping();
  for(double sign:{-1.,1.})for(int axis=0;axis<3;++axis)for(double ratio:{.001,.05,.1,.3,1.}) {
    const double vx=axis==0?sign*.3*ratio:0.,vy=axis==1?sign*.3*ratio:0.,yaw=axis==2?sign*.5*ratio:0.;
    const auto f=sdk::mappedSdkFractions(vx,vy,yaw,"holonomic_xyyaw",map,.3,.5);
    EXPECT_LE(fractions(f).norm(),.3+1e-12);EXPECT_LE(std::abs(f.yaw),.3+1e-12);
    if(axis==0){EXPECT_GT(f.forward_back*sign,0.);EXPECT_DOUBLE_EQ(f.left_right,0.);}
    if(axis==1){EXPECT_GT(f.left_right*sign,0.);EXPECT_DOUBLE_EQ(f.forward_back,0.);}
    if(axis==2){EXPECT_GT(f.yaw*sign,0.);EXPECT_DOUBLE_EQ(fractions(f).norm(),0.);}
  }
  for(double x:{-1.,1.})for(double y:{-1.,1.}) {
    const auto f=sdk::mappedSdkFractions(x*.3/std::sqrt(2.),y*.3/std::sqrt(2.),0.,"holonomic_xyyaw",map,.3,.5);
    EXPECT_NEAR(fractions(f).norm(),.3,1e-12);EXPECT_NEAR(f.forward_back*x,f.left_right*y,1e-12);
    EXPECT_GT(std::abs(f.forward_back),.20); // capped 30% diagonal is 21.21%/axis.
  }
}

TEST(SdkMappingInertia, ReplacementDoesNotResetWriterSlewButRawZeroDoes) {
  auto m=mapping();sdk::SdkFractionSlew continuous,reference;
  const sdk::SdkMappingIdentity id{"writer",3,5};
  const auto target=sdk::mappedSdkFractions(.3,.0,.5,"holonomic_xyyaw",m,.3,.5);
  sdk::SdkMoveFractions last;
  for(int i=0;i<20;++i) {
    last=continuous.apply(target,.3,0.,.5,m,id,i*kDt);
    reference.apply(target,.3,0.,.5,m,id,i*kDt);
  }
  // Curve identity is deliberately absent from SdkMappingIdentity. A new
  // curve in the same execution retains all three fraction-history axes.
  const auto replacement=continuous.apply(target,.3,0.,.5,m,id,1.);
  const auto expected=reference.apply(target,.3,0.,.5,m,id,1.);
  EXPECT_DOUBLE_EQ(replacement.forward_back,expected.forward_back);EXPECT_DOUBLE_EQ(replacement.yaw,expected.yaw);
  EXPECT_GT(replacement.forward_back,.25);EXPECT_GT(replacement.yaw,.25);
  const auto stop=continuous.apply({},0.,0.,0.,m,id,1.05);
  EXPECT_DOUBLE_EQ(fractions(stop).norm(),0.);EXPECT_DOUBLE_EQ(stop.yaw,0.);
  const auto restart=continuous.apply(target,.3,0.,.5,m,id,1.1);
  EXPECT_DOUBLE_EQ(fractions(restart).norm(),0.);EXPECT_DOUBLE_EQ(restart.yaw,0.);
  const auto next=continuous.apply(target,.3,0.,.5,m,id,1.15);
  EXPECT_NEAR(next.forward_back,.015,1e-12);EXPECT_NEAR(next.yaw,.015,1e-12);
  // Independent zero axes cannot start sideways/yaw motion; reversals spend
  // real writer ticks braking through zero input before the opposite sign.
  for(int i=0;i<40;++i)continuous.apply(target,.3,0.,.5,m,id,1.2+i*kDt);
  auto reversed=sdk::mappedSdkFractions(-.3,0.,-.5,"holonomic_xyyaw",m,.3,.5);
  double previous=.3;bool reached_negative=false;
  for(int i=0;i<80;++i) {
    const auto f=continuous.apply(reversed,-.3,0.,-.5,m,id,3.2+i*kDt);
    EXPECT_DOUBLE_EQ(f.left_right,0.);EXPECT_LE(std::abs(f.forward_back-previous),.0225+1e-12);
    if(f.forward_back<0.){EXPECT_LE(previous,0.);reached_negative=true;break;}previous=f.forward_back;
  }
  EXPECT_TRUE(reached_negative);
}

// Focused V2 contract checks only: no simulated plant, no arrival assertion.
// Keep the original V1 sensitivity cases above/below unchanged. These values
// mirror motion_tracking.yaml and runtime_motion_profile.json; production
// profile parsing/version discrimination is tested by the SDK acceptance tests.
sdk::SdkInputMapping mappingV2() {
  auto m=mapping();m.component_cap=true;
  for(auto *axis:{&m.planar,&m.yaw}) {
    axis->effective_fraction=.30;axis->maximum_fraction=.40;
    axis->rise_rate_per_s=.60;axis->fall_rate_per_s=.60;
  }
  return m;
}
Odom fixedBody(double now,double cross_offset=0.,double cross_velocity=0.) {
  Odom body;body.frame_id="d1max_loc_odom";body.child_frame_id="d1max_loc_base_link";
  body.stamp=body.posterior_stamp=body.imu_stamp=now;body.source_stamp_ns=std::llround(now*1e9);
  body.position={0.,cross_offset,.55};body.orientation=Eigen::Quaterniond::Identity();
  body.velocity_in_frame={0.,cross_velocity,0.};body.planar_speed=std::abs(cross_velocity);
  return body;
}

TEST(SdkMappingInertia, V2DefaultTrackerStationaryStartupAndCurveReplacementKeepContinuousFractions) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";const auto m=mappingV2();
  const double preview_demand=c.kp_position*c.min_lookahead_distance_m;
  ASSERT_NEAR(preview_demand,.042,1e-12);
  const auto at_preview=sdk::mappedSdkFractions(preview_demand,0.,0.,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  EXPECT_GT(at_preview.forward_back,.30);EXPECT_LT(at_preview.forward_back,.31);
  for(const Eigen::Vector2d &direction:{Eigen::Vector2d(1.,0.),Eigen::Vector2d(std::sqrt(.5),std::sqrt(.5))}) {
    SCOPED_TRACE(direction.y()==0.?"straight":"diagonal_45");
    const Scenario scenario{"v2_stationary_start",direction};
    TrackerCore core(c);auto trajectory=curve(scenario);
    ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
    ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart),kRosStart,kReceiptStart));
    ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart))<<core.candidateReason();
    sdk::SdkFractionSlew slew;const sdk::SdkMappingIdentity identity{"v2-writer",1,1};
    Output previous_output; sdk::SdkMoveFractions previous_fraction;
    // The measured body deliberately stays stationary. Requested SI velocity
    // is never fed back as measured speed and no physical movement is claimed.
    for(int i=1;i<=24;++i) {
      const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
      ASSERT_TRUE(core.receiveOdom(fixedBody(now),now,receipt));core.refreshTaskLease(receipt);
      if(i==10) {
        trajectory=curve(scenario,2,now);
        ASSERT_TRUE(core.receiveTrajectory(trajectory,now,receipt))<<core.candidateReason();
      }
      const auto output=core.step(now,receipt,true);ASSERT_EQ(output.reason,"tracking");
      EXPECT_FALSE(output.finished);EXPECT_DOUBLE_EQ(core.progressArc(),0.);
      EXPECT_GE(planar(output).dot(direction)+1e-12,planar(previous_output).dot(direction));
      EXPECT_LE((planar(output)-planar(previous_output)).norm(),c.max_acceleration*kDt+1e-10);
      EXPECT_NEAR(output.yaw_rate,0.,1e-12);
      const auto target=sdk::mappedSdkFractions(output.forward,output.lateral,output.yaw_rate,
        "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
      const auto sent=slew.apply(target,output.forward,output.lateral,output.yaw_rate,m,identity,i*kDt);
      EXPECT_GE(sent.forward_back+1e-12,previous_fraction.forward_back);
      EXPECT_GE(sent.left_right+1e-12,previous_fraction.left_right);
      EXPECT_LE((fractions(sent)-fractions(previous_fraction)).norm(),.60*kDt+1e-10);
      EXPECT_LE(std::max(std::abs(sent.forward_back),std::abs(sent.left_right)),.40+1e-12);
      EXPECT_NEAR(sent.forward_back*direction.y()-sent.left_right*direction.x(),0.,1e-12);
      if(i==1){EXPECT_DOUBLE_EQ(fractions(sent).norm(),0.);} // writer initialization, not motion
      if(i==10){EXPECT_GT(fractions(sent).norm(),.20);} // actual curve replacement did not reset slew
      core.recordAppliedOutput(output.forward,output.lateral,output.yaw_rate,now,receipt);
      previous_output=output;previous_fraction=sent;
    }
    // .042 is the preview term, not a fabricated final controller output:
    // the existing measured-state damping adds its reference error at rest.
    EXPECT_NEAR(planar(previous_output).norm(),preview_demand*(1.+c.velocity_damping_gain),1e-10);
    EXPECT_GE(previous_fraction.forward_back,.30);
    if(direction.y()!=0.){EXPECT_GE(previous_fraction.left_right,.30);} // no radial cap to .283/axis
    else {EXPECT_DOUBLE_EQ(previous_fraction.left_right,0.);}
    core.cancel("v2_fixture_cancel");
    const auto stop=core.step(kRosStart+25*kDt,kReceiptStart+25*kDt,true);ASSERT_TRUE(rawZero(stop));
    const auto zero=slew.apply({},stop.forward,stop.lateral,stop.yaw_rate,m,identity,25*kDt);
    EXPECT_DOUBLE_EQ(fractions(zero).norm(),0.);EXPECT_DOUBLE_EQ(zero.yaw,0.);
    // A new explicit demand after zero has no history from the cancelled task.
    const auto target=sdk::mappedSdkFractions(preview_demand,0.,0.,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(fractions(slew.apply(target,preview_demand,0.,0.,m,identity,26*kDt)).norm(),0.);
  }
}

TEST(SdkMappingInertia, V2DefaultTrackerTinyCrossCorrectionsDoNotGetIndependentAxisFloors) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";const auto m=mappingV2();TrackerCore core(c);
  const auto trajectory=curve(Scenario{"v2_tiny_correction",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  sdk::SdkFractionSlew slew;const sdk::SdkMappingIdentity identity{"v2-noise-writer",1,1};
  for(int i=1;i<=30;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    // Small, explicitly supplied pose/twist perturbations, not command-fed
    // measurements or a robot noise model. Alternate signs after warm-up.
    const double perturbation=i<=20?0.:(i%2?1e-5:-1e-5);
    auto body=fixedBody(now,perturbation,perturbation);body.angular_velocity_in_frame.z()=perturbation;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto output=core.step(now,receipt,true);ASSERT_EQ(output.reason,"tracking");
    const auto target=sdk::mappedSdkFractions(output.forward,output.lateral,output.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,output.forward,output.lateral,output.yaw_rate,m,identity,i*kDt);
    if(i>20) {
      EXPECT_GT(std::abs(output.lateral),0.);EXPECT_LT(std::abs(output.lateral),1e-4);
      EXPECT_LT(std::abs(target.left_right),1e-3);EXPECT_LT(std::abs(sent.left_right),1e-3);
      EXPECT_LT(std::abs(target.yaw),1e-6);EXPECT_LT(std::abs(sent.yaw),1e-6);
      EXPECT_GE(sent.forward_back,.30);
      EXPECT_NEAR(target.left_right*output.forward-target.forward_back*output.lateral,0.,1e-12);
    }
    core.recordAppliedOutput(output.forward,output.lateral,output.yaw_rate,now,receipt);
  }
}

// V3 control-boundary checks use explicitly supplied body observations only.
// They do not integrate a plant or provide a physical steering acceptance.
sdk::SdkInputMapping mappingV3() {
  auto m=mappingV2();m.direct_breakaway=true;return m;
}
struct CurrentRuntimeProfile {Config control;sdk::SdkInputMapping mapping;};
CurrentRuntimeProfile currentRuntimeProfile() {
  const auto directory=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/
    "src/d1max_pct_scan/config";
  auto path=directory/"runtime_motion_profile.json";
  if(const char* configured=std::getenv("D1MAX_RUNTIME_MOTION_PROFILE"))path=configured;
  std::ifstream input(path);if(!input)throw std::runtime_error("runtime motion profile unavailable: "+path.string());
  nlohmann::json profile;input>>profile;const auto& mapping=profile.at("sdk_input_mapping");
  if(mapping.at("schema_version")!=4||mapping.at("kind")!="continuous_amplitude_ratio_v4")
    throw std::runtime_error("current fixture requires the actual V4 mapping");
  auto control_path=directory/"motion_tracking.yaml";
  if(const char* configured=std::getenv("D1MAX_MOTION_TRACKING"))control_path=configured;
  const auto policy=YAML::LoadFile(control_path.string());
  CurrentRuntimeProfile out;out.control=config();auto& c=out.control;
  c.motion_model=policy["motion_model"].as<std::string>();
  c.holonomic_yaw_policy=policy["heading_policy"].as<std::string>();
  c.align_then_forward=policy["align_then_forward"].as<bool>();
  c.lookahead=policy["preview_time_s"].as<double>();
  c.min_lookahead_distance_m=policy["min_lookahead_distance_m"].as<double>();
  c.max_lookahead_distance_m=policy["max_lookahead_distance_m"].as<double>();
  c.kp_position=policy["position_gain"].as<double>();c.kp_yaw=policy["yaw_gain"].as<double>();
  c.heading_threshold=policy["heading_error_threshold_rad"].as<double>();
  c.steering_error_scale_rad=policy["steering_error_scale_rad"].as<double>();
  c.velocity_damping_gain=policy["velocity_damping_gain"].as<double>();
  c.yaw_damping_gain=policy["yaw_damping_gain"].as<double>();
  c.max_speed=policy["max_speed_mps"].as<double>();c.max_yaw_rate=policy["max_yaw_radps"].as<double>();
  c.max_acceleration=policy["max_acceleration_mps2"].as<double>();
  c.max_yaw_acceleration=policy["max_yaw_acceleration_radps2"].as<double>();
  c.max_jerk=policy["max_jerk_mps3"].as<double>();c.max_yaw_jerk=policy["max_yaw_jerk_radps3"].as<double>();
  c.max_input_rate=policy["max_input_rate_per_s"].as<double>();
  c.max_input_acceleration=policy["max_input_acceleration_per_s2"].as<double>();
  if(std::abs(policy["controller_frequency_hz"].as<double>()*kDt-1.)>1e-12||
      c.max_speed!=profile.at("measurements").at("max_speed_mps").get<double>()||
      c.max_yaw_rate!=profile.at("measurements").at("max_yaw_radps").get<double>())
    throw std::runtime_error("current controller and SDK profile timing/scale mismatch");
  out.mapping.enabled=out.mapping.component_cap=out.mapping.continuous_command_ratio=true;
  const auto load=[&](const auto& source,auto& axis) {
    axis.effective_fraction=source.at("effective_fraction");axis.maximum_fraction=source.at("maximum_fraction");
    axis.blend_demand_ratio=source.at("blend_demand_ratio");
  };
  load(mapping.at("planar"),out.mapping.planar);load(mapping.at("yaw"),out.mapping.yaw);
  out.control.forward_response={out.control.max_speed,out.mapping.planar.effective_fraction,
    out.mapping.planar.maximum_fraction,out.mapping.planar.blend_demand_ratio};
  out.control.yaw_response={out.control.max_yaw_rate,out.mapping.yaw.effective_fraction,
    out.mapping.yaw.maximum_fraction,out.mapping.yaw.blend_demand_ratio};
  out.mapping.validate();out.control.validate();return out;
}
Odom measuredHeading(double now,double yaw,double yaw_rate=0.) {
  auto body=fixedBody(now);body.yaw=yaw;
  body.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());
  body.angular_velocity_in_frame.z()=yaw_rate;return body;
}
struct CurrentInputHistory {
  Output command;
  sdk::SdkMoveFractions mapped;
  double forward_acceleration=0.,yaw_acceleration=0.,forward_rate=0.,yaw_rate=0.;
};
void expectCurrentContinuousStep(const CurrentRuntimeProfile& profile,const TrackerCore& core,
    const Output& out,const sdk::SdkMoveFractions& sent,CurrentInputHistory& old) {
  const auto& c=profile.control;
  EXPECT_FALSE(core.controlDiagnostic().input_limit_override);
  EXPECT_FALSE(out.frozen);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(sent.left_right,0.);
  const double a=(out.forward-old.command.forward)/kDt,w=(out.yaw_rate-old.command.yaw_rate)/kDt;
  const double fr=(sent.forward_back-old.mapped.forward_back)/kDt,wr=(sent.yaw-old.mapped.yaw)/kDt;
  EXPECT_LE(std::abs(a),c.max_acceleration+1e-9);EXPECT_LE(std::abs(w),c.max_yaw_acceleration+1e-9);
  EXPECT_LE(std::abs(a-old.forward_acceleration),c.max_jerk*kDt+1e-8);
  EXPECT_LE(std::abs(w-old.yaw_acceleration),c.max_yaw_jerk*kDt+1e-8);
  EXPECT_LE(std::abs(fr),c.max_input_rate+1e-8);EXPECT_LE(std::abs(wr),c.max_input_rate+1e-8);
  EXPECT_LE(std::abs(fr-old.forward_rate),c.max_input_acceleration*kDt+1e-7);
  EXPECT_LE(std::abs(wr-old.yaw_rate),c.max_input_acceleration*kDt+1e-7);
  EXPECT_NEAR(sent.forward_back,c.forward_response.map(out.forward),1e-14);
  EXPECT_NEAR(sent.yaw,c.yaw_response.map(out.yaw_rate),1e-14);
  old={out,sent,a,w,fr,wr};
}

TEST(SdkV4ContinuousControl, ControllerResponseExactlyMatchesProductionStaticSdkMapping) {
  const auto profile=currentRuntimeProfile();const auto c=profile.control;const auto m=profile.mapping;
  for (int i=-2000;i<=2000;++i) {
    const double ratio=i/2000.,forward=ratio*c.max_speed,yaw=ratio*c.max_yaw_rate;
    const auto sent=sdk::mappedSdkFractions(forward,0.,yaw,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    EXPECT_NEAR(c.forward_response.map(forward),sent.forward_back,1e-14);
    EXPECT_NEAR(c.yaw_response.map(yaw),sent.yaw,1e-14);
    EXPECT_DOUBLE_EQ(sent.left_right,0.);
  }
}

TEST(SdkV4ContinuousControl, SameTaskReplansPreserveTheOnlyConditionerAndNoWriterHistoryIsInvented) {
  const auto profile=currentRuntimeProfile();const auto c=profile.control;const auto m=profile.mapping;
  const Scenario scenario{"v4_same_task_forward",{1.,0.}};
  auto trajectory=curve(scenario);TrackerCore unchanged(c),refreshed(c);
  for (auto *core : {&unchanged,&refreshed}) {
    ASSERT_TRUE(core->receiveTask(task(trajectory),kRosStart,kReceiptStart));
    ASSERT_TRUE(core->receiveOdom(measuredHeading(kRosStart,-.20),kRosStart,kReceiptStart));
    ASSERT_TRUE(core->receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  }
  sdk::SdkFractionSlew writer;const sdk::SdkMappingIdentity owner{"same-task-v4",1,1};
  sdk::SdkMoveFractions previous_sent;Output previous_output;
  double previous_forward_rate=0.,previous_yaw_rate=0.,max_step=0.,max_acceleration=0.;
  unsigned overrides=0,zero_edges=0;bool first=true;
  std::ofstream evidence;
  if (const char *directory=std::getenv("D1MAX_INERTIA_OUTPUT")) {
    evidence.open(std::string(directory)+"/continuous_forward_v4.csv");
    evidence<<std::setprecision(17)<<"tick,curve_id,si_forward,si_yaw,sdk_forward,sdk_yaw,sdk_forward_rate,sdk_yaw_rate,input_override\n";
  }
  for (int i=1;i<=80;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    // Deliberately fixed pose and independently specified measured rates. No
    // physical body model or navigation-arrival claim is hidden in this test.
    const auto measured=measuredHeading(now,-.20,i%2?.30:0.);
    for (auto *core : {&unchanged,&refreshed}) {
      ASSERT_TRUE(core->receiveOdom(measured,now,receipt));core->refreshTaskLease(receipt);
    }
    if (i%4==0) {
      auto replacement=curve(scenario,1+i/4,now);
      ASSERT_TRUE(refreshed.receiveTrajectory(replacement,now,receipt))<<refreshed.candidateReason();
    }
    const auto expected=unchanged.step(now,receipt,true),out=refreshed.step(now,receipt,true);
    ASSERT_FALSE(out.frozen);ASSERT_FALSE(out.finished);ASSERT_TRUE(refreshed.active());
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_GT(out.yaw_rate,0.);
    EXPECT_NEAR(out.forward,expected.forward,1e-12);EXPECT_NEAR(out.yaw_rate,expected.yaw_rate,1e-12);
    EXPECT_LE(std::abs(out.forward-previous_output.forward),c.max_acceleration*kDt+1e-12);
    EXPECT_LE(std::abs(out.yaw_rate-previous_output.yaw_rate),c.max_yaw_acceleration*kDt+1e-12);
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=writer.apply(target,out.forward,out.lateral,out.yaw_rate,m,owner,i*kDt);
    EXPECT_DOUBLE_EQ(sent.forward_back,target.forward_back);EXPECT_DOUBLE_EQ(sent.yaw,target.yaw);
    EXPECT_DOUBLE_EQ(sent.left_right,0.); // The writer is a static mapper, not a second smoother.
    const double forward_rate=(sent.forward_back-previous_sent.forward_back)/kDt;
    const double yaw_rate=(sent.yaw-previous_sent.yaw)/kDt;
    const auto diagnostic=refreshed.controlDiagnostic();
    if (!diagnostic.input_limit_override) {
      EXPECT_LE(std::abs(forward_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(yaw_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(forward_rate-previous_forward_rate),c.max_input_acceleration*kDt+1e-7);
      EXPECT_LE(std::abs(yaw_rate-previous_yaw_rate),c.max_input_acceleration*kDt+1e-7);
    } else ++overrides;
    if (!first && (sent.yaw==0.)!=(previous_sent.yaw==0.)) ++zero_edges;
    first=false;max_step=std::max({max_step,std::abs(forward_rate*kDt),std::abs(yaw_rate*kDt)});
    max_acceleration=std::max({max_acceleration,std::abs(forward_rate-previous_forward_rate)/kDt,
      std::abs(yaw_rate-previous_yaw_rate)/kDt});
    evidence<<i<<','<<refreshed.trajectoryId()<<','<<out.forward<<','<<out.yaw_rate<<','
      <<sent.forward_back<<','<<sent.yaw<<','<<forward_rate<<','<<yaw_rate<<','<<diagnostic.input_limit_override<<'\n';
    previous_forward_rate=forward_rate;previous_yaw_rate=yaw_rate;
    previous_sent=sent;previous_output=out;
  }
  EXPECT_EQ(overrides,0u);EXPECT_EQ(zero_edges,0u);
  EXPECT_GT(previous_sent.forward_back,.30);EXPECT_GT(previous_sent.yaw,m.yaw.effective_fraction);
  EXPECT_LT(previous_sent.yaw,m.yaw.maximum_fraction);
  std::cout<<"V4 actual controller + static SDK mapping: max_fraction_step="<<max_step
    <<" max_fraction_acceleration="<<max_acceleration<<" overrides="<<overrides
    <<" yaw_zero_edges="<<zero_edges<<"; fixed observations, NOT physical acceptance\n";
  refreshed.cancel("v4_cancel");const auto stop=refreshed.step(kRosStart+81*kDt,kReceiptStart+81*kDt,true);
  ASSERT_TRUE(rawZero(stop));const auto zero=writer.apply({},0.,0.,0.,m,owner,81*kDt);
  EXPECT_DOUBLE_EQ(zero.forward_back,0.);EXPECT_DOUBLE_EQ(zero.yaw,0.);
}

TEST(SdkV4ContinuousControl, SmallStraightHeadingNoiseKeepsForwardWithoutLateralOrLargeAlternatingYaw) {
  const auto profile=currentRuntimeProfile();const auto c=profile.control;const auto m=profile.mapping;TrackerCore core(c);
  const auto trajectory=curve(Scenario{"v4_small_noise",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  for (int i=1;i<=80;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,(i%2?-1.:1.)*kPi/180.),now,receipt));
    core.refreshTaskLease(receipt);const auto out=core.step(now,receipt,true);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    const auto sent=sdk::mappedSdkFractions(out.forward,0.,out.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(sent.left_right,0.);EXPECT_LT(std::abs(sent.yaw),.02);
  }
}

TEST(SdkV4ContinuousControl, GoalBrakingUsesSameConditionerAndFreshActualRestIsAnExplicitZeroPriority) {
  const auto profile=currentRuntimeProfile();auto c=profile.control;c.external_goal_completion=true;
  const auto m=profile.mapping;const auto trajectory=curve(Scenario{"v4_goal_settling",{1.,0.}});
  auto goal=task(trajectory);goal.goal={.05,0.,.55};TrackerCore core(c);
  auto measured=measuredHeading(kRosStart,0.,.10);measured.position=goal.goal;
  measured.planar_speed=.20;measured.velocity_in_frame.x()=.20;
  ASSERT_TRUE(core.receiveTask(goal,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measured,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  core.recordAppliedOutput(.20,0.,.10,kRosStart,kReceiptStart);
  auto previous=sdk::mappedSdkFractions(.20,0.,.10,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  double previous_vx=.20,previous_yaw=.10,previous_forward_rate=0.,previous_yaw_rate=0.;
  for(int i=1;i<=20;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
    measured.source_stamp_ns=std::llround(now*1e9);
    ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);EXPECT_EQ(out.reason,"goal_position_settling");
    EXPECT_FALSE(out.finished);EXPECT_TRUE(core.active());EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_LE(std::abs(out.forward-previous_vx),c.max_acceleration*kDt+1e-10);
    EXPECT_LE(std::abs(out.yaw_rate-previous_yaw),c.max_yaw_acceleration*kDt+1e-10);
    if(i==1) { EXPECT_GT(out.forward,0.);EXPECT_GT(out.yaw_rate,0.); }
    const auto sent=sdk::mappedSdkFractions(out.forward,0.,out.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const double forward_rate=(sent.forward_back-previous.forward_back)/kDt;
    const double yaw_rate=(sent.yaw-previous.yaw)/kDt;
    EXPECT_FALSE(core.controlDiagnostic().input_limit_override);
    if(!core.controlDiagnostic().input_limit_override) {
      EXPECT_LE(std::abs(forward_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(yaw_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(forward_rate-previous_forward_rate),c.max_input_acceleration*kDt+1e-7);
      EXPECT_LE(std::abs(yaw_rate-previous_yaw_rate),c.max_input_acceleration*kDt+1e-7);
    }
    previous=sent;previous_vx=out.forward;previous_yaw=out.yaw_rate;
    previous_forward_rate=forward_rate;previous_yaw_rate=yaw_rate;
  }
  // A command reaching zero is not a fresh actual-rest observation.
  EXPECT_GT(measured.planar_speed,c.stationary_linear_threshold_mps);
  measured.planar_speed=0.;measured.velocity_in_frame.setZero();measured.angular_velocity_in_frame.setZero();
  measured.stamp=measured.posterior_stamp=measured.imu_stamp=kRosStart+21*kDt;
  measured.source_stamp_ns=std::llround((kRosStart+21*kDt)*1e9);
  ASSERT_TRUE(core.receiveOdom(measured,kRosStart+21*kDt,kReceiptStart+21*kDt));
  core.refreshTaskLease(kReceiptStart+21*kDt);
  const auto stopped=core.step(kRosStart+21*kDt,kReceiptStart+21*kDt,true);
  EXPECT_EQ(stopped.reason,"goal_position_reached");EXPECT_TRUE(rawZero(stopped));
  EXPECT_FALSE(stopped.finished);EXPECT_TRUE(core.active());
}

TEST(SdkV4ContinuousControl, CurvedNativeGeometryKeepsFeedforwardAndMappedDerivativeHistory) {
  const auto profile=currentRuntimeProfile();const auto c=profile.control;const auto m=profile.mapping;TrackerCore core(c);
  auto trajectory=curve(Scenario{"v4_actual_curve",{1.,0.}});
  for(auto& point:trajectory.points)point.y()=.20*point.x()*point.x();
  const auto geometry=PreparedGeometry::build(c,trajectory,{});ASSERT_TRUE(geometry->failure.empty());
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  auto measured=fixedBody(kRosStart);
  measured.position=geometry->entry.curve_sampler->evaluateDeBoorT(1.);
  const auto initial_velocity=geometry->entry.velocity_sampler->evaluateDeBoorT(1.);
  measured.yaw=std::atan2(initial_velocity.y(),initial_velocity.x());
  measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
  measured.velocity_in_frame=initial_velocity;measured.planar_speed=initial_velocity.head<2>().norm();
  ASSERT_TRUE(core.receiveOdom(measured,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  const auto initial_acceleration=geometry->acceleration_sampler->evaluateDeBoorT(1.);
  const double initial_yaw=(initial_velocity.x()*initial_acceleration.y()-initial_velocity.y()*initial_acceleration.x())/
    initial_velocity.head<2>().squaredNorm();
  core.recordAppliedOutput(initial_velocity.head<2>().norm(),0.,initial_yaw,kRosStart,kReceiptStart);
  auto previous=sdk::mappedSdkFractions(initial_velocity.head<2>().norm(),0.,initial_yaw,
    "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  double previous_forward_rate=0.,previous_yaw_rate=0.;
  for(int i=1;i<=50;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt,at=1.+i*kDt;
    const auto velocity=geometry->entry.velocity_sampler->evaluateDeBoorT(at);
    const auto acceleration=geometry->acceleration_sampler->evaluateDeBoorT(at);
    measured.position=geometry->entry.curve_sampler->evaluateDeBoorT(at);
    measured.yaw=std::atan2(velocity.y(),velocity.x());
    measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
    measured.velocity_in_frame=velocity;measured.planar_speed=velocity.head<2>().norm();
    measured.angular_velocity_in_frame.z()=(velocity.x()*acceleration.y()-velocity.y()*acceleration.x())/
      velocity.head<2>().squaredNorm();
    measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
    measured.source_stamp_ns=std::llround(now*1e9);
    ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);ASSERT_FALSE(out.frozen);ASSERT_FALSE(out.finished);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_GT(out.yaw_rate,0.);
    const auto sent=sdk::mappedSdkFractions(out.forward,0.,out.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const double forward_rate=(sent.forward_back-previous.forward_back)/kDt,yaw_rate=(sent.yaw-previous.yaw)/kDt;
    EXPECT_FALSE(core.controlDiagnostic().input_limit_override);
    if(!core.controlDiagnostic().input_limit_override) {
      EXPECT_LE(std::abs(forward_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(yaw_rate),c.max_input_rate+1e-8);
      EXPECT_LE(std::abs(forward_rate-previous_forward_rate),c.max_input_acceleration*kDt+1e-7);
      EXPECT_LE(std::abs(yaw_rate-previous_yaw_rate),c.max_input_acceleration*kDt+1e-7);
    }
    previous=sent;previous_forward_rate=forward_rate;previous_yaw_rate=yaw_rate;
  }
}

TEST(SdkCurrentProfile, ActualContinuousProfileMatchesProductionMappingWithoutFixedYawPlateau) {
  const auto profile=currentRuntimeProfile();const auto& c=profile.control;const auto& m=profile.mapping;
  ASSERT_TRUE(c.align_then_forward);ASSERT_GT(m.yaw.maximum_fraction,m.yaw.effective_fraction);
  double previous=-m.yaw.maximum_fraction;
  for(int i=-2000;i<=2000;++i) {
    const double demand=c.max_yaw_rate*i/2000.;
    const auto sent=sdk::mappedSdkFractions(.20,0.,demand,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    EXPECT_NEAR(sent.yaw,c.yaw_response.map(demand),1e-14);
    EXPECT_DOUBLE_EQ(sent.left_right,0.);EXPECT_LE(std::abs(sent.yaw),m.yaw.maximum_fraction+1e-14);
    EXPECT_GE(sent.yaw,previous);EXPECT_LT(sent.yaw-previous,.01);previous=sent.yaw;
  }
}

TEST(SdkCurrentProfile, NoiseDoesNotCreateHighAmplitudeReversalsAndPersistentErrorIsCorrected) {
  const auto profile=currentRuntimeProfile();const auto& c=profile.control;const auto& m=profile.mapping;
  TrackerCore core(c);const auto trajectory=curve(Scenario{"continuous_actual_profile",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,0.),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  sdk::SdkMoveFractions sent;CurrentInputHistory history;SignCounter high_amplitude;
  for(int i=1;i<=90;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    const double error=i<=30?(i%2?-1.:1.):10.;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,-error*kPi/180.),now,receipt));core.refreshTaskLease(receipt);
    const auto demand=core.step(now,receipt,true);EXPECT_GT(demand.forward,0.);EXPECT_DOUBLE_EQ(demand.lateral,0.);
    sent=sdk::mappedSdkFractions(demand.forward,0.,demand.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    expectCurrentContinuousStep(profile,core,demand,sent,history);high_amplitude.update(sent.yaw,.02);
    if(i<=30) {EXPECT_LT(std::abs(sent.yaw),.02);}
    EXPECT_LE(std::abs(sent.yaw),m.yaw.maximum_fraction+1e-14);
  }
  EXPECT_EQ(high_amplitude.reversals,0);EXPECT_GT(sent.yaw,m.yaw.effective_fraction);
  EXPECT_LT(sent.yaw,m.yaw.maximum_fraction);EXPECT_TRUE(core.active());
}

TEST(SdkCurrentProfile, OppositeRequestHasOneContinuousReversalAndCancellationIsExactZero) {
  const auto profile=currentRuntimeProfile();const auto& c=profile.control;const auto& m=profile.mapping;
  TrackerCore core(c);const auto trajectory=curve(Scenario{"continuous_yaw_reverse",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,-30.*kPi/180.),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  sdk::SdkMoveFractions sent;Output demand;CurrentInputHistory history;SignCounter high_amplitude;
  for(int i=1;i<=120;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,(i<=40?-30.:10.)*kPi/180.,.25),now,receipt));
    core.refreshTaskLease(receipt);
    demand=core.step(now,receipt,true);
    sent=sdk::mappedSdkFractions(demand.forward,0.,demand.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    EXPECT_GT(demand.forward,0.);expectCurrentContinuousStep(profile,core,demand,sent,history);
    high_amplitude.update(sent.yaw,.02);
    if(i==40){EXPECT_GT(sent.yaw,m.yaw.effective_fraction);}
    if(i>40){EXPECT_LT(core.controlDiagnostic().yaw_target,0.);} // No old-rate settling lock.
  }
  EXPECT_EQ(high_amplitude.reversals,1);EXPECT_LT(sent.yaw,-m.yaw.effective_fraction);
  core.cancel("actual_profile_cancel");const auto stopped=core.step(kRosStart+121*kDt,kReceiptStart+121*kDt,true);
  EXPECT_TRUE(rawZero(stopped));
  const auto zero=sdk::mappedSdkFractions(stopped.forward,stopped.lateral,stopped.yaw_rate,
    "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  EXPECT_DOUBLE_EQ(zero.forward_back,0.);EXPECT_DOUBLE_EQ(zero.left_right,0.);EXPECT_DOUBLE_EQ(zero.yaw,0.);
}

TEST(SdkMappingInertia, V3ContinuousForwardRetainsHistoryAndSlewsReleasedYaw) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  TrackerCore core(c);const Scenario scenario{"v3_align",{1.,0.}};
  auto trajectory=curve(scenario);const auto m=mappingV3();
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,kPi/2.),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  sdk::SdkFractionSlew slew;const sdk::SdkMappingIdentity identity{"v3-writer",1,1};
  const std::array<double,4> yaw{{kPi/2.,.08,.02,.20}};
  for(std::size_t i=0;i<yaw.size();++i) {
    const double now=kRosStart+(i+1)*kDt,receipt=kReceiptStart+(i+1)*kDt;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,yaw[i]),now,receipt));core.refreshTaskLease(receipt);
    if(i==3) {
      trajectory=curve(scenario,2,now);
      ASSERT_TRUE(core.receiveTrajectory(trajectory,now,receipt))<<core.candidateReason();
    }
    const auto out=core.step(now,receipt,true);
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,(i+1)*kDt);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(sent.left_right,0.);
    EXPECT_TRUE(out.reason=="tracking_forward"||out.reason=="tracking_forward_yaw_coasting");EXPECT_FALSE(out.frozen);
    if(i==0) {
      // Perpendicular body has zero forward projection, but it is not a
      // separate frozen alignment phase. The actual yaw command still uses
      // V3's signed breakaway, never a mock controller result.
      EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(sent.forward_back,0.);
      EXPECT_DOUBLE_EQ(sent.yaw,-.30);
    } else {
      EXPECT_GT(out.forward,0.);EXPECT_GE(sent.forward_back,.30);
      if(i==1){EXPECT_DOUBLE_EQ(sent.forward_back,.30);}
      if(i==2){
        // A normal heading release keeps the previous yaw's deceleration
        // history; the legacy V3 amplitude floor is not a reason to hard-cut SI.
        EXPECT_DOUBLE_EQ(core.controlDiagnostic().yaw_target,0.);
        EXPECT_LE(std::abs(out.yaw_rate),c.max_yaw_acceleration*kDt+1e-12);
        EXPECT_LE(out.yaw_rate,0.);EXPECT_LE(sent.yaw,0.);
      }
      // A same-task curve refresh at .20 rad must keep moving, not require
      // full alignment or reset the SDK fraction history back to breakaway.
      if(i==3){EXPECT_GT(sent.forward_back,.30);}
    }
  }
  core.cancel("v3_fixture_cancel");
  const auto out=core.step(kRosStart+5*kDt,kReceiptStart+5*kDt,true);ASSERT_TRUE(rawZero(out));
  const auto stop=slew.apply({},out.forward,out.lateral,out.yaw_rate,m,identity,5*kDt);
  EXPECT_DOUBLE_EQ(stop.forward_back,0.);EXPECT_DOUBLE_EQ(stop.left_right,0.);EXPECT_DOUBLE_EQ(stop.yaw,0.);
}

TEST(SdkMappingInertia, V3ShortLocalSegmentUsesInteriorDirectionBeforeZeroEndpoint) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  TrackerCore core(c);auto trajectory=curve(Scenario{"v3_short_entry",{1.,0.}});
  const double scale=.04/trajectory.points.back().x();
  for(auto& p:trajectory.points){p.x()*=scale;p.y()*=scale;}
  auto goal=task(trajectory);goal.goal.x()=2.; // a local window, not final arrival
  ASSERT_TRUE(core.receiveTask(goal,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart))<<core.candidateReason();
  ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart+kDt),kRosStart+kDt,kReceiptStart+kDt));
  core.refreshTaskLease(kReceiptStart+kDt);
  const auto out=core.step(kRosStart+kDt,kReceiptStart+kDt,true);
  EXPECT_EQ(out.reason,"tracking_forward");EXPECT_GT(out.forward,0.);
  EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
}

TEST(SdkMappingInertia, SoftJerkSlowdownAcrossAcceptedCurveCannotRestartThirtyPercentWriter) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;c.max_jerk=.4;
  TrackerCore core(c);Trajectory original;original.session_id=c.session_id;original.frame_id=c.planning_frame;
  original.generation=1;original.id=1;original.start_time=kRosStart;
  for(int i=0;i<23;++i)original.points.emplace_back((i-1)*.05,0.,.55);
  for(int i=0;i<27;++i)original.knots.push_back((i-3)*.2);
  auto goal=task(original);goal.goal.x()=2.;
  ASSERT_TRUE(core.receiveTask(goal,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(fixedBody(kRosStart),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(original,kRosStart,kReceiptStart));
  auto m=mappingV3();m.startup_transition_s=1.;m.fraction_acceleration_per_s2=2.;
  sdk::SdkFractionSlew writer;const sdk::SdkMappingIdentity owner{"same-task-writer",1,1};
  // Already-running writer history is an explicit command fixture, not body
  // feedback. The actual production writer identity does NOT include curve.
  sdk::SdkMoveFractions sent;
  for(int i=0;i<=24;++i) {
    const auto target=sdk::mappedSdkFractions(.24,0.,0.,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    sent=writer.apply(target,.24,0.,0.,m,owner,i*kDt);
  }
  ASSERT_GE(sent.forward_back,.30-1e-12);
  core.recordAppliedOutput(.24,0.,0.,kRosStart,kReceiptStart);
  auto replacement=original;replacement.id=2;for(auto& p:replacement.points)p.x()*=.3;
  ASSERT_TRUE(core.receiveTrajectory(replacement,kRosStart,kReceiptStart));
  const auto out=core.step(kRosStart+kDt,kReceiptStart+kDt,true);
  ASSERT_EQ(out.reason,"tracking_forward");ASSERT_GT(out.forward,0.);
  const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  const auto continued=writer.apply(target,out.forward,out.lateral,out.yaw_rate,m,owner,25*kDt);
  EXPECT_GE(continued.forward_back,.30-1e-12);EXPECT_LE(continued.forward_back,.40+1e-12);
  EXPECT_NEAR(continued.forward_back,sent.forward_back,m.planar.fall_rate_per_s*kDt);
  EXPECT_FALSE(core.requiresMeasuredReentry());EXPECT_FALSE(out.frozen);
  core.cancel("explicit_cancel");const auto zero=core.step(kRosStart+2*kDt,kReceiptStart+2*kDt,true);
  ASSERT_TRUE(rawZero(zero));
  const auto stopped=writer.apply({},0.,0.,0.,m,owner,26*kDt);
  EXPECT_DOUBLE_EQ(stopped.forward_back,0.);EXPECT_DOUBLE_EQ(stopped.yaw,0.);
}

TEST(SdkMappingInertia, V3ContinuousForwardDoesNotWaitForMeasuredAngularCoast) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  TrackerCore core(c);const auto trajectory=curve(Scenario{"v3_angular_coast",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,.08,.4),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  const auto m=mappingV3();sdk::SdkFractionSlew slew;
  const sdk::SdkMappingIdentity identity{"v3-forward-coast",1,1};
  for(int i=1;i<=2;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,.08,i==1?.4:.05),now,receipt));
    core.refreshTaskLease(receipt);const auto out=core.step(now,receipt,true);
    EXPECT_EQ(out.reason,"tracking_forward");EXPECT_GT(out.forward,0.);EXPECT_FALSE(out.frozen);
    EXPECT_DOUBLE_EQ(out.lateral,0.);
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,i*kDt);
    EXPECT_GE(sent.forward_back,.30);EXPECT_DOUBLE_EQ(sent.left_right,0.);
    // Actual rate does not invent a .8-second turn prediction or an extra
    // wait. Both samples request the corrective sign through the normal slew.
    EXPECT_LT(out.yaw_rate,0.);EXPECT_LE(sent.yaw,-.30);
  }
}

TEST(SdkMappingInertia, V3CurrentHeadingSteeringKeepsForwardAndSlewsYawRelease) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  TrackerCore core(c);const Scenario scenario{"v3_predictive_coast",{1.,0.}};
  auto trajectory=curve(scenario);const auto m=mappingV3();
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,-.22),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  sdk::SdkFractionSlew slew;sdk::SdkMappingIdentity identity{"v3-predictive-writer",1,1};
  const std::array<double,5> yaw{{-.22,-.18,.03,-.03,.20}};
  const std::array<double,5> measured_rate{{0.,.30,.20,-.20,0.}};
  sdk::SdkMoveFractions previous;
  for(std::size_t i=0;i<yaw.size();++i) {
    const double now=kRosStart+(i+1)*kDt,receipt=kReceiptStart+(i+1)*kDt;
    // Fresh independent observations, not output fed back as sensor data.
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,yaw[i],measured_rate[i]),now,receipt));
    core.refreshTaskLease(receipt);
    if(i==1) {
      trajectory=curve(scenario,2,now);ASSERT_TRUE(core.receiveTrajectory(trajectory,now,receipt));
    }
    const auto out=core.step(now,receipt,true);
    ASSERT_TRUE(out.reason=="tracking_forward"||out.reason=="tracking_forward_yaw_coasting");EXPECT_FALSE(out.frozen);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.finished);
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,(i+1)*kDt);
    EXPECT_GE(sent.forward_back,.30);EXPECT_LE(sent.forward_back,.40);EXPECT_DOUBLE_EQ(sent.left_right,0.);
    if(i==0){EXPECT_GT(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,.30);}
    if(i==1){EXPECT_GT(out.yaw_rate,0.);EXPECT_GE(sent.yaw,.30);}
    if(i==2){
      EXPECT_DOUBLE_EQ(core.controlDiagnostic().yaw_target,0.);
      EXPECT_GE(out.yaw_rate,0.);EXPECT_LE(out.yaw_rate,c.max_yaw_acceleration*kDt+1e-12);
    }
    if(i==3){EXPECT_DOUBLE_EQ(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,0.);}
    if(i>=1&&i<=3) {
      EXPECT_GE(sent.forward_back+1e-12,previous.forward_back);
    }
    if(i==4){EXPECT_LT(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,-.30);} // Genuine new error after measured coast.
    previous=sent;
  }
  core.cancel("v3_predictive_cancel");const auto zero=core.step(kRosStart+6*kDt,kReceiptStart+6*kDt,true);
  ASSERT_TRUE(rawZero(zero));const auto stopped=slew.apply({},zero.forward,zero.lateral,zero.yaw_rate,m,identity,6*kDt);
  EXPECT_DOUBLE_EQ(stopped.forward_back,0.);EXPECT_DOUBLE_EQ(stopped.left_right,0.);EXPECT_DOUBLE_EQ(stopped.yaw,0.);
  // A new writer identity cannot resurrect either direction from the retired
  // history. This uses the real V3 helper, with an explicit new requested sign.
  identity={"new-v3-predictive-writer",2,2};
  const auto new_target=sdk::mappedSdkFractions(.04,0.,.1,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
  const auto restarted=slew.apply(new_target,.04,0.,.1,m,identity,7*kDt);
  EXPECT_DOUBLE_EQ(restarted.forward_back,.30);EXPECT_DOUBLE_EQ(restarted.left_right,0.);EXPECT_DOUBLE_EQ(restarted.yaw,.30);
}

TEST(SdkMappingInertia, V3FinalHeadingUsesActualErrorAndNormalYawDeceleration) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;c.external_goal_completion=true;
  TrackerCore core(c);auto trajectory=curve(Scenario{"v3_goal_predictive",{1.,0.}});
  const double scale=.30/trajectory.points.back().x();for(auto& point:trajectory.points)point.x()*=scale;
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  auto body=measuredHeading(kRosStart,-.22);body.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart,kReceiptStart));ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  const auto m=mappingV3();sdk::SdkFractionSlew slew;const sdk::SdkMappingIdentity identity{"v3-goal-predictive",1,1};
  // Preserve the existing original-clock guard, exactly as the terminal-yaw
  // case below: a first call primes its source clock and may not send motion.
  body=measuredHeading(kRosStart+kDt,-.22);body.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart+kDt,kReceiptStart+kDt));core.refreshTaskLease(kReceiptStart+kDt);
  const auto primed=core.align(0.,.07,kRosStart+kDt,kReceiptStart+kDt,true);ASSERT_TRUE(rawZero(primed));
  EXPECT_FALSE(primed.finished);EXPECT_TRUE(core.active());
  const std::array<double,4> yaw{{-.22,-.18,.12,.06}},measured_rate{{0.,.30,.25,-.25}};
  for(std::size_t i=0;i<yaw.size();++i) {
    const double now=kRosStart+(i+2)*kDt,receipt=kReceiptStart+(i+2)*kDt;
    body=measuredHeading(now,yaw[i],measured_rate[i]);body.position.x()=.29;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.align(0.,.07,now,receipt,true);ASSERT_EQ(out.reason,"aligning");
    EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.finished);EXPECT_TRUE(core.active());
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,(i+1)*kDt);
    if(i==0){EXPECT_GT(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,.30);}
    if(i==1){EXPECT_GT(out.yaw_rate,0.);EXPECT_GE(sent.yaw,.30);}
    if(i==2){EXPECT_GE(out.yaw_rate,0.);EXPECT_LE(out.yaw_rate,c.max_yaw_acceleration*kDt+1e-12);}
    if(i==3){EXPECT_DOUBLE_EQ(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,0.);}
    if(i==1||i==2){EXPECT_GT(std::abs(yaw[i]),.07);} // Actual orientation remains outside tolerance.
    if(i==3){EXPECT_LT(std::abs(yaw[i]),.07);} // Inside tolerance still requires true stationary evidence above this core.
  }
}

TEST(SdkMappingInertia, V3FinalSettlingDoesNotReenterLateralOrStaleHeadingController) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  TrackerCore core(c);auto trajectory=curve(Scenario{"v3_terminal_settling",{1.,0.}});
  const double scale=.30/trajectory.points.back().x();
  for(auto& p:trajectory.points){p.x()*=scale;p.y()*=scale;}
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  auto initial=measuredHeading(kRosStart,kPi/2.);initial.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(initial,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart))<<core.candidateReason();
  auto body=measuredHeading(kRosStart+kDt,0.);body.position.x()=.29;
  body.velocity_in_frame.y()=.10;body.planar_speed=.10;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart+kDt,kReceiptStart+kDt));core.refreshTaskLease(kReceiptStart+kDt);
  const auto out=core.step(kRosStart+kDt,kReceiptStart+kDt,true);
  ASSERT_EQ(out.reason,"goal_position_settling");
  // The goal is already within tolerance. Coasting must not reactivate the
  // old holonomic lateral correction or rotate back to the admission yaw.
  EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
}

TEST(SdkMappingInertia, V3MovingAtGoalDeceleratesPriorSteeringWithoutHardCut) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  c.external_goal_completion=true;
  TrackerCore core(c);auto trajectory=curve(Scenario{"v3_goal_position",{1.,0.}});
  const double scale=.30/trajectory.points.back().x();
  for(auto& p:trajectory.points){p.x()*=scale;p.y()*=scale;}
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  auto body=measuredHeading(kRosStart,0.);body.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  // Explicit prior applied controller output, not a measured body velocity.
  core.recordAppliedOutput(0.,0.,.20,kRosStart,kReceiptStart);
  body=measuredHeading(kRosStart+kDt,0.,.08);body.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart+kDt,kReceiptStart+kDt));core.refreshTaskLease(kReceiptStart+kDt);
  const auto out=core.step(kRosStart+kDt,kReceiptStart+kDt,true);
  ASSERT_EQ(out.reason,"goal_position_settling");
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  EXPECT_NEAR(out.yaw_rate,.20-c.max_yaw_acceleration*kDt,1e-12);
}

#ifndef D1MAX_BASELINE_CONTROLLER
TEST(SdkMappingInertia, SmoothedV3HeadingBoundaryNoiseCannotCreateThirtyPercentOnOffPulses) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;c.max_jerk=.4;c.max_yaw_jerk=.8;
  TrackerCore core(c);const auto t=curve(Scenario{"heading_boundary_noise",{1.,0.}});
  ASSERT_TRUE(core.receiveTask(task(t),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveOdom(measuredHeading(kRosStart,-.59),kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(t,kRosStart,kReceiptStart));
  auto m=mappingV3();m.startup_transition_s=1.;m.fraction_acceleration_per_s2=2.;sdk::SdkFractionSlew writer;
  const sdk::SdkMappingIdentity identity{"threshold-noise",1,1};double max_forward=0.;
  for(int i=1;i<=40;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    ASSERT_TRUE(core.receiveOdom(measuredHeading(now,i%2?-.59:-.61),now,receipt));core.refreshTaskLease(receipt);
    auto next=t;next.id=i+1;next.start_time=now;ASSERT_TRUE(core.receiveTrajectory(next,now,receipt));
    const auto out=core.step(now,receipt,true);EXPECT_FALSE(out.frozen);EXPECT_EQ(out.reason,"tracking_forward");
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto input=writer.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,i*kDt);
    max_forward=std::max(max_forward,std::abs(input.forward_back));
    EXPECT_LT(std::abs(input.forward_back),.001); // Any neutral/start noise stays far below 30%, not a full pulse.
    EXPECT_GE(input.yaw,0.); // The accepted correcting turn is not repeatedly reversed or re-armed.
  }
  EXPECT_GT(max_forward,0.);
}
#endif

TEST(SdkMappingInertia, V3ExplicitGoalYawToleranceStopsSdkYawWithoutLimiterTail) {
  auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;
  c.external_goal_completion=true;
  TrackerCore core(c);auto trajectory=curve(Scenario{"v3_goal_yaw",{1.,0.}});
  const double scale=.30/trajectory.points.back().x();
  for(auto& p:trajectory.points){p.x()*=scale;p.y()*=scale;}
  ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
  auto body=measuredHeading(kRosStart,0.);body.position.x()=.29;
  ASSERT_TRUE(core.receiveOdom(body,kRosStart,kReceiptStart));
  ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart));
  const auto m=mappingV3();sdk::SdkFractionSlew slew;
  const sdk::SdkMappingIdentity identity{"v3-goal-yaw",1,1};
  for(int i=1;i<=5;++i) {
    const double now=kRosStart+i*kDt,receipt=kReceiptStart+i*kDt;
    body=measuredHeading(now,i==5?1.:0.);body.position.x()=.29;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.align(1.,.10,now,receipt,true);
    const auto target=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,
      "holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
    const auto sent=slew.apply(target,out.forward,out.lateral,out.yaw_rate,m,identity,i*kDt);
    // The first alignment call primes the original clock guard. Subsequent
    // fresh calls establish an actual yaw command before testing its stop.
    if(i>1&&i<5){EXPECT_GT(out.yaw_rate,0.);EXPECT_GE(sent.yaw,.30);}
    if(i==5){EXPECT_DOUBLE_EQ(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(sent.yaw,0.);}
  }
}

TEST(SdkMappingInertia, RealTrackerWithIndependentInertiaAndDelayFeedbackReportsFailures) {
  const char*directory=std::getenv("D1MAX_INERTIA_OUTPUT");ASSERT_NE(directory,nullptr);
  summary.open(std::string(directory)+"/summary.csv");trace.open(std::string(directory)+"/trace.csv");
  ASSERT_TRUE(summary.is_open());ASSERT_TRUE(trace.is_open());summary<<std::setprecision(9);trace<<std::setprecision(9);
  summary<<"plant,scenario,mode,arrived,arrival_s,stationary_confirmed_s,final_position_error_m,final_yaw_error_rad,peak_cross_error_m,final_cross_error_m,actual_speed_peak_mps,actual_yaw_peak_radps,demand_norm_peak_mps,demand_yaw_peak_radps,sdk_norm_peak,sdk_yaw_peak,normal_sdk_slew_per_s,normal_yaw_slew_per_s,raw_zero_fraction_jump,coast_after_raw_zero_m,request_xy_reversals,sdk_xy_reversals,physical_xy_reversals,request_yaw_reversals,sdk_yaw_reversals,physical_yaw_reversals,cross_error_sign_changes,replacement_accepted,raw_zero_immediate,same_input_linear_peak,same_input_mapped_peak,same_input_rms_delta,final_reason,first_fault\n";
  trace<<"plant,scenario,mode,t_s,x_m,y_m,yaw_rad,actual_vx_body_mps,actual_vy_body_mps,actual_yaw_radps,demand_vx_mps,demand_vy_mps,demand_yaw_radps,sdk_forward,sdk_lateral,sdk_yaw,position_error_m,yaw_error_rad,reason,same_input_linear_forward,same_input_linear_lateral,same_input_linear_yaw,same_input_mapped_forward,same_input_mapped_lateral,same_input_mapped_yaw\n";
  int reported_arrival_failures=0;
  for(const auto&p:kPlants) {
    std::array<int,2> arrived{{0,0}};
    std::array<double,2> peak{{0.,0.}};
    for(const auto&s:kScenarios)for(bool nonlinear:{false,true}) {
      SCOPED_TRACE(p.name+"/"+s.name+"/"+(nonlinear?"mapped":"linear"));
      const auto result=run(p,s,nonlinear);writeSummary(result);
      if(s.cancel||s.estop){EXPECT_TRUE(result.zero_immediate);}
      else {
        if(result.arrived)++arrived[nonlinear?1:0];else ++reported_arrival_failures;
        peak[nonlinear?1:0]=std::max(peak[nonlinear?1:0],result.max_speed);
      }
    }
    std::cout<<"ASSUMED_PLANT "<<p.name<<": linear arrival "<<arrived[0]<<"/8, mapped "
      <<arrived[1]<<"/8; actual simulated speed peaks "<<peak[0]<<" / "<<peak[1]<<" m/s\n";
  }
  RecordProperty("simulation_arrival_failures",reported_arrival_failures);
  std::cout<<"Arrival failures retained in CSV: "<<reported_arrival_failures
    <<". A passing contract test is NOT physical navigation acceptance.\n";
  summary.close();trace.close();
}

// Small, reproducible control sensitivity experiment, separate from the old
// expensive sweep above. Actual feedback is independently integrated Plant;
// no requested velocity is ever substituted for a measured body response.
// The same recorded SI input is ALSO replayed through both writer mappings
// and both plants, so nonlinear input differences are not hidden by feedback.
TEST(SdkMappingInertia, ContinuousCommandShapingReportsSameInputPhysicalSensitivity) {
  const char* destination=std::getenv("D1MAX_CONTINUOUS_OUTPUT");
  std::ofstream csv,commands;
  if(destination) {
    csv.open(std::string(destination)+"/continuous_summary.csv");
    commands.open(std::string(destination)+"/continuous_commands.csv");
    ASSERT_TRUE(csv.is_open());ASSERT_TRUE(commands.is_open());
    csv<<"controller,plant,scenario,mapping,model_settled_at_end,final_error,peak_actual_speed,peak_actual_yaw,peak_actual_accel,peak_actual_yaw_accel,peak_actual_jerk,peak_actual_yaw_jerk,yaw_sign_reversals,normal_fraction_rate,normal_fraction_accel,all_fraction_rate,all_fraction_accel,zero_to_nonzero_count,last_reason\n";
    commands<<"controller,plant,scenario,mapping,t,forward,lateral,yaw,SDK_forward,SDK_yaw,x,y,heading,actual_speed,actual_yaw\n";
  }
  const auto legacy=mappingV3();auto smooth=legacy;smooth.startup_transition_s=1.;smooth.fraction_acceleration_per_s2=2.;
  for(const auto&parameters:{kPlants[0],kPlants[1],kPlants[4]})for(int scenario=0;scenario<3;++scenario) {
    const std::string name=scenario==0?"steady_bend":scenario==1?"heading_replan":"yaw_90_start";
    const Scenario spec{name,{1.,0.},0.,scenario==2?-kPi/2.:scenario==1?-.22:0.};
    for(bool smoothed:{false,true}) {
      auto c=config();c.holonomic_yaw_policy="path_tangent";c.align_then_forward=true;c.external_goal_completion=true;
#ifndef D1MAX_BASELINE_CONTROLLER
      c.max_jerk=.4;c.max_yaw_jerk=.8;
      const char* controller="continuous_shaped";
#else
      const char* controller="frozen_before";
#endif
      TrackerCore core(c);auto trajectory=curve(spec);
      if(scenario==0)for(auto&point:trajectory.points)point.y()=.12*point.x()*point.x();
      Plant body(parameters);body.yaw=spec.initial_yaw;
      ASSERT_TRUE(core.receiveTask(task(trajectory),kRosStart,kReceiptStart));
      ASSERT_TRUE(core.receiveOdom(body.odom(kRosStart),kRosStart,kReceiptStart));
      ASSERT_TRUE(core.receiveTrajectory(trajectory,kRosStart,kReceiptStart))<<core.candidateReason();
      const auto policy=smoothed?smooth:legacy;sdk::SdkFractionSlew slew;
      const sdk::SdkMappingIdentity identity{"continuous-physical-fixture",1,1};
      double peak_speed=0.,peak_yaw=0.,peak_accel=0.,peak_yaw_accel=0.,peak_jerk=0.,peak_yaw_jerk=0.;
      double fraction_rate=0.,fraction_accel=0.,normal_rate=0.,normal_accel=0.;
      double previous_speed=0.,previous_yaw=0.,previous_accel=0.,previous_yaw_accel=0.,previous_input_rate=0.;
      sdk::SdkMoveFractions previous_input;SignCounter sign;int starts=0,settled_samples=0;bool arrived=false;
      std::string reason;std::vector<Output> recorded;
      for(int i=1;i<=440;++i) {
        const double elapsed=i*kDt,now=kRosStart+elapsed,receipt=kReceiptStart+elapsed;
        ASSERT_TRUE(core.receiveOdom(body.odom(now),now,receipt));core.refreshTaskLease(receipt);
        if(scenario==1&&i%20==0) {
          trajectory.id++;trajectory.start_time=now;
          // Repeating unchanged immutable geometry is enough to test history:
          // the independent body pose does NOT restart at the curve origin.
          EXPECT_TRUE(core.receiveTrajectory(trajectory,now,receipt,true))<<core.candidateReason();
        }
        const auto demand=core.step(now,receipt,true);recorded.push_back(demand);reason=demand.reason;
        const auto target=sdk::mappedSdkFractions(demand.forward,demand.lateral,demand.yaw_rate,"holonomic_xyyaw",policy,c.max_speed,c.max_yaw_rate);
        const auto input=slew.apply(target,demand.forward,demand.lateral,demand.yaw_rate,policy,identity,elapsed,
          demand.reason=="goal_position_settling"&&!rawZero(demand));
        EXPECT_LE(std::abs(input.forward_back),.40+1e-12);EXPECT_DOUBLE_EQ(input.left_right,0.);
        EXPECT_LE(std::abs(input.yaw),.40+1e-12);
        if(rawZero(demand)){EXPECT_DOUBLE_EQ(input.forward_back,0.);EXPECT_DOUBLE_EQ(input.yaw,0.);}
        const double input_rate=(input.forward_back-previous_input.forward_back)/kDt;
        const double input_accel=(input_rate-previous_input_rate)/kDt;
        fraction_rate=std::max(fraction_rate,std::abs(input_rate));fraction_accel=std::max(fraction_accel,std::abs(input_accel));
        if(!rawZero(demand)&&input.forward_back*previous_input.forward_back>0.) {
          normal_rate=std::max(normal_rate,std::abs(input_rate));normal_accel=std::max(normal_accel,std::abs(input_accel));
        }
        if(previous_input.yaw==0.&&input.yaw!=0.)++starts;
        sign.update(input.yaw,.001);body.advance(input,elapsed);
        const double speed=body.body_velocity.norm(),yaw=body.yaw_velocity;
        const double accel=(speed-previous_speed)/kDt,yaw_accel=(yaw-previous_yaw)/kDt;
        peak_speed=std::max(peak_speed,speed);peak_yaw=std::max(peak_yaw,std::abs(yaw));
        peak_accel=std::max(peak_accel,std::abs(accel));peak_yaw_accel=std::max(peak_yaw_accel,std::abs(yaw_accel));
        peak_jerk=std::max(peak_jerk,std::abs(accel-previous_accel)/kDt);
        peak_yaw_jerk=std::max(peak_yaw_jerk,std::abs(yaw_accel-previous_yaw_accel)/kDt);
        const bool settled=(body.position-task(trajectory).goal).norm()<=c.goal_tolerance&&
          speed<=c.stationary_linear_threshold_mps&&std::abs(yaw)<=c.stationary_angular_threshold_radps;
        settled_samples=settled?settled_samples+1:0;
        arrived=settled_samples>=static_cast<int>(std::ceil(c.stationary_reentry_duration_s/kDt))+1;
        if(commands)commands<<controller<<','<<parameters.name<<','<<name<<','<<(smoothed?"smooth":"direct")<<','<<elapsed<<','<<demand.forward<<','<<demand.lateral<<','<<demand.yaw_rate<<','<<input.forward_back<<','<<input.yaw<<','<<body.position.x()<<','<<body.position.y()<<','<<body.yaw<<','<<speed<<','<<yaw<<'\n';
        previous_speed=speed;previous_yaw=yaw;previous_accel=accel;previous_yaw_accel=yaw_accel;
        previous_input=input;previous_input_rate=input_rate;
      }
      const double error=(body.position-task(trajectory).goal).norm();
      if(csv)csv<<controller<<','<<parameters.name<<','<<name<<','<<(smoothed?"smooth":"direct")<<','<<arrived<<','<<error<<','<<peak_speed<<','<<peak_yaw<<','<<peak_accel<<','<<peak_yaw_accel<<','<<peak_jerk<<','<<peak_yaw_jerk<<','<<sign.reversals<<','<<normal_rate<<','<<normal_accel<<','<<fraction_rate<<','<<fraction_accel<<','<<starts<<','<<reason<<'\n';
      // Identical demand samples, identical assumed initial state: a paired
      // open-loop replay isolates actual response to the two writer mappings.
      std::array<Plant,2> replay{{Plant(parameters),Plant(parameters)}};
      std::array<sdk::SdkFractionSlew,2> writers;
      std::array<double,2> last_speed{},last_acceleration{},actual_peak{},actual_jerk{};
      for(std::size_t i=0;i<recorded.size();++i)for(int mode=0;mode<2;++mode) {
        const auto&o=recorded[i];const auto&m=mode?smooth:legacy;
        const auto target=sdk::mappedSdkFractions(o.forward,o.lateral,o.yaw_rate,"holonomic_xyyaw",m,c.max_speed,c.max_yaw_rate);
        const auto input=writers[mode].apply(target,o.forward,o.lateral,o.yaw_rate,m,identity,(i+1)*kDt,
          o.reason=="goal_position_settling"&&!rawZero(o));
        replay[mode].advance(input,(i+1)*kDt);
        const double v=replay[mode].body_velocity.norm(),a=(v-last_speed[mode])/kDt;
        actual_peak[mode]=std::max(actual_peak[mode],std::abs(a));actual_jerk[mode]=std::max(actual_jerk[mode],std::abs(a-last_acceleration[mode])/kDt);
        last_speed[mode]=v;last_acceleration[mode]=a;
      }
      std::cout<<controller<<' '<<parameters.name<<' '<<name<<" trace="<<(smoothed?"smooth":"direct")
        <<" paired_actual_accel "<<actual_peak[0]<<" -> "<<actual_peak[1]
        <<" paired_actual_jerk "<<actual_jerk[0]<<" -> "<<actual_jerk[1]
        <<" arrival="<<arrived<<" final_error="<<error<<" yaw_starts="<<starts<<" reason="<<reason<<'\n';
    }
  }
}
} // namespace
