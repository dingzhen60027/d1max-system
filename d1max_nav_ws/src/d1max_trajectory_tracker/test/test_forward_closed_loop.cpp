// Deterministic software closed loop: production TrackerCore + native SCAN
// spline math + the production SDK static mapping. Only the robot response is
// simulated. These response families are sensitivity assumptions, NOT D1 Max
// calibration or field acceptance. No ROS graph, SDK client, or robot writer.
// Version/proof tests remain in the existing typed execution test suite; this
// fixture isolates control and preserves original measurement timestamps.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "sdk_input_mapping.hpp"

namespace {
using namespace d1max_trajectory_tracker;
namespace sdk=d1monitor::execution3;
constexpr double kDt=.05,kRos=100.,kReceipt=10.,kPi=3.14159265358979323846;

std::filesystem::path configuredPath(const char* name,const char* fallback) {
  if(const char* value=std::getenv(name))return value;
  return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/fallback;
}
struct Policy {Config control;sdk::SdkInputMapping mapping;};
template<class C> auto errorScale(C& c,double value,int)->decltype(c.steering_error_scale_rad=value,void()) {
  c.steering_error_scale_rad=value;
}
template<class C> void errorScale(C&,double,long) {}
Policy policy() {
  const auto yaml=YAML::LoadFile(configuredPath("D1MAX_TRACKING_CONFIG","src/d1max_pct_scan/config/motion_tracking.yaml").string());
  std::ifstream input(configuredPath("D1MAX_RUNTIME_MOTION_PROFILE","src/d1max_pct_scan/config/runtime_motion_profile.json"));
  if(!input)throw std::runtime_error("runtime motion profile missing");
  nlohmann::json profile;input>>profile;
  Policy p;auto& c=p.control;auto& m=p.mapping;
  c.session_id="forward-closed-loop";c.require_versioned_identity=false;
  c.motion_model=yaml["motion_model"].as<std::string>();
  c.holonomic_yaw_policy=yaml["heading_policy"].as<std::string>();
  c.align_then_forward=yaml["align_then_forward"].as<bool>();
  c.max_speed=yaml["max_speed_mps"].as<double>();c.max_yaw_rate=yaml["max_yaw_radps"].as<double>();
  c.max_acceleration=yaml["max_acceleration_mps2"].as<double>();
  c.max_yaw_acceleration=yaml["max_yaw_acceleration_radps2"].as<double>();
  c.max_jerk=yaml["max_jerk_mps3"].as<double>();c.max_yaw_jerk=yaml["max_yaw_jerk_radps3"].as<double>();
  c.lookahead=yaml["preview_time_s"].as<double>();c.kp_position=yaml["position_gain"].as<double>();
  c.kp_yaw=yaml["yaw_gain"].as<double>();c.heading_threshold=yaml["heading_error_threshold_rad"].as<double>();
  c.min_lookahead_distance_m=yaml["min_lookahead_distance_m"].as<double>();
  c.max_lookahead_distance_m=yaml["max_lookahead_distance_m"].as<double>();
  if(yaml["steering_enter_angle_rad"])c.steering_enter_angle_rad=yaml["steering_enter_angle_rad"].as<double>();
  if(yaml["steering_exit_angle_rad"])c.steering_exit_angle_rad=yaml["steering_exit_angle_rad"].as<double>();
  if(yaml["steering_error_scale_rad"])errorScale(c,yaml["steering_error_scale_rad"].as<double>(),0);
  c.velocity_damping_gain=yaml["velocity_damping_gain"].as<double>();
  c.yaw_damping_gain=yaml["yaw_damping_gain"].as<double>();
  c.max_input_rate=yaml["max_input_rate_per_s"].as<double>();
  c.max_input_acceleration=yaml["max_input_acceleration_per_s2"].as<double>();
  const auto& mapping=profile.at("sdk_input_mapping");
  if(mapping.at("schema_version")!=4||mapping.at("kind")!="continuous_amplitude_ratio_v4")
    throw std::runtime_error("fixture requires production V4 static mapping");
  m.enabled=m.component_cap=m.continuous_command_ratio=true;
  const auto axis=[](const auto& source,auto& target) {
    target.effective_fraction=source.at("effective_fraction");
    target.maximum_fraction=source.at("maximum_fraction");target.blend_demand_ratio=source.at("blend_demand_ratio");
  };
  axis(mapping.at("planar"),m.planar);axis(mapping.at("yaw"),m.yaw);
  c.forward_response={c.max_speed,m.planar.effective_fraction,m.planar.maximum_fraction,m.planar.blend_demand_ratio};
  c.yaw_response={c.max_yaw_rate,m.yaw.effective_fraction,m.yaw.maximum_fraction,m.yaw.blend_demand_ratio};
  m.validate();c.validate();return p;
}

struct PlantSpec {
  const char* name;
  double forward_knee,yaw_knee,forward_working,yaw_working;
  double forward_gain,yaw_gain,translation_tau,yaw_tau,delay;
};
// Finite response AT the working level matters: a linear model with zero
// response at 20% would silently turn the current plateau into a stuck robot.
const std::array<PlantSpec,3> kPlants{{
  {"nominal",.29,.195,.24,.18,1.0,1.2,.10,.10,.05},
  {"responsive",.27,.18,.30,.30,1.2,1.8,.05,.05,.05},
  {"delayed",.30,.20,.20,.24,1.0,1.5,.20,.25,.15}
}};
struct Plant {
  PlantSpec spec;Eigen::Vector3d position{0.,0.,.55};double heading=0.,speed=0.,yaw_rate=0.;
  struct Queued {double when;sdk::SdkMoveFractions input;};std::deque<Queued> queue;
  sdk::SdkMoveFractions applied;
  explicit Plant(PlantSpec s):spec(s){}
  static double response(double input,double knee,double working,double gain) {
    const double magnitude=std::abs(input);
    return magnitude+1e-10<knee?0.:std::copysign(working+gain*(magnitude-knee),input);
  }
  void advance(const sdk::SdkMoveFractions& input,double time) {
    queue.push_back({time+spec.delay,input});
    constexpr int steps=10;constexpr double h=kDt/steps;
    for(int i=0;i<steps;++i) {
      const double subtime=time+(i+1)*h;
      while(!queue.empty()&&queue.front().when<=subtime+1e-10){applied=queue.front().input;queue.pop_front();}
      const double target_v=response(applied.forward_back,spec.forward_knee,spec.forward_working,spec.forward_gain);
      const double target_w=response(applied.yaw,spec.yaw_knee,spec.yaw_working,spec.yaw_gain);
      speed+=(target_v-speed)*(-std::expm1(-h/spec.translation_tau));
      yaw_rate+=(target_w-yaw_rate)*(-std::expm1(-h/spec.yaw_tau));
      heading=angle(heading+yaw_rate*h);
      position.x()+=speed*std::cos(heading)*h;position.y()+=speed*std::sin(heading)*h;
    }
  }
  Odom measurement(double now,bool noisy,int tick,double heading_noise_degrees=.8)const {
    Odom o;o.frame_id="d1max_loc_odom";o.child_frame_id="d1max_loc_base_link";
    o.stamp=o.posterior_stamp=o.imu_stamp=now;o.source_stamp_ns=std::llround(now*1e9);
    o.position=position;o.yaw=heading;
    if(noisy){o.position.y()+=.006*std::sin(tick*1.7);o.yaw=angle(o.yaw+heading_noise_degrees*kPi/180.*std::sin(tick*1.1));}
    o.orientation=Eigen::AngleAxisd(o.yaw,Eigen::Vector3d::UnitZ());
    o.velocity_in_frame={speed*std::cos(heading),speed*std::sin(heading),0.};
    o.planar_speed=std::abs(speed);o.angular_velocity_in_frame={0.,0.,yaw_rate};return o;
  }
};

enum class Shape {Straight,QuarterCircle,TwoTurns,AlternatingTurns};
struct Scene {const char* name;Shape shape;double offset=0.,heading=0.;bool noise=false,replace=false,reject=false;
  double heading_noise_degrees=.8;};
const std::array<Scene,9> kScenes{{
  {"straight_noise",Shape::Straight,0.,0.,true},
  {"offset_recovery",Shape::Straight,.20,10.*kPi/180.},
  {"quarter_circle",Shape::QuarterCircle},
  {"continuous_turns",Shape::TwoTurns},
  {"alternating_turns",Shape::AlternatingTurns},
  {"same_task_replacement",Shape::Straight,0.,0.,false,true},
  {"curved_task_replacement",Shape::QuarterCircle,0.,0.,false,true},
  {"bad_candidate_keeps_incumbent",Shape::Straight,0.,0.,false,false,true},
  {"small_heading_noise",Shape::Straight,0.,2.*kPi/180.,true}
}};
// Separate heading-acquisition sensitivity cases. These do not claim that a
// native collision/support certificate admitted an arbitrary in-place turn.
const std::array<Scene,4> kHeadingScenes{{
  {"heading_120_noise",Shape::Straight,0.,120.*kPi/180.,true,false,false,1.},
  {"heading_179_noise",Shape::Straight,0.,179.*kPi/180.,true,false,false,1.},
  {"heading_minus179_noise",Shape::Straight,0.,-179.*kPi/180.,true,false,false,1.},
  {"heading_180_noise",Shape::Straight,0.,kPi,true,false,false,1.}
}};
double length(Shape shape) {return shape==Shape::Straight?4.:(shape==Shape::QuarterCircle?4.+.5*kPi:6.+kPi);}
Eigen::Vector3d pathPoint(Shape shape,double s) {
  if(shape==Shape::Straight||s<=2.)return {s,0.,.55};
  s-=2.;
  if(s<=.5*kPi)return {2.+std::sin(s),1.-std::cos(s),.55};
  s-=.5*kPi;
  if(shape==Shape::QuarterCircle||s<=2.)return {3.,1.+s,.55};
  s-=2.;
  if(shape==Shape::AlternatingTurns) {
    if(s<=.5*kPi)return {4.-std::cos(s),3.+std::sin(s),.55};
    return {4.+s-.5*kPi,4.,.55};
  }
  if(s<=.5*kPi)return {2.+std::cos(s),3.+std::sin(s),.55};
  return {2.-(s-.5*kPi),4.,.55};
}
Trajectory trajectory(const Scene& scene,std::int64_t id=1) {
  Trajectory t;t.session_id="forward-closed-loop";t.frame_id="d1max_loc_odom";
  t.generation=1;t.id=id;t.start_time=kRos;t.frozen_yaw=0.;
  const double total=length(scene.shape);std::vector<double> stations{0.,0.,0.,.02,.06,.12,.20};
  for(double s=.30;s<total-.20-1e-8;s+=.10)stations.push_back(s);
  // Uniform stations through the final deceleration region, then repeated
  // final controls give the native cubic zero terminal velocity/acceleration.
  const double base=stations.back();
  for(double s:{total-.20,total-.12,total-.06,total-.02,total,total,total})if(s>=base-1e-8)stations.push_back(s);
  for(double s:stations)t.points.push_back(pathPoint(scene.shape,s));
  for(std::size_t i=0;i<t.points.size()+4;++i)t.knots.push_back((static_cast<double>(i)-3.)*.60);
  return t;
}
Task task(const Trajectory& t) {Task q;q.session_id=t.session_id;q.frame_id=t.frame_id;q.generation=1;q.active=true;q.issued_at=kRos;q.goal=t.points.back();return q;}
double pathError(const PreparedGeometry& geometry,const Eigen::Vector3d& p) {
  double best=std::numeric_limits<double>::infinity();
  for(std::size_t i=1;i<geometry.entry.points.size();++i) {
    const Eigen::Vector3d a=geometry.entry.points[i-1],d=geometry.entry.points[i]-a;
    const double f=d.squaredNorm()<1e-14?0.:std::clamp((p-a).dot(d)/d.squaredNorm(),0.,1.);
    best=std::min(best,(p-a-f*d).head<2>().norm());
  }
  return best;
}
Trajectory cropped(const Trajectory& original,const PreparedGeometry& geometry,const Plant& body,std::int64_t id,double now) {
  double best=std::numeric_limits<double>::infinity(),time=0.;
  for(std::size_t i=0;i<geometry.entry.points.size();++i) {
    const double distance=(geometry.entry.points[i]-body.position).squaredNorm();
    if(distance<best){best=distance;time=geometry.entry.times[i];}
  }
  const int cut=std::max(0,std::min(static_cast<int>(std::floor(time/.60)),static_cast<int>(original.points.size())-4));
  Trajectory t=original;t.id=id;t.start_time=now;
  t.points.erase(t.points.begin(),t.points.begin()+cut);t.knots.erase(t.knots.begin(),t.knots.begin()+cut);return t;
}
struct Metrics {
  bool arrived=false;double stop_time=0.,final_error=0.,peak_cross=0.,final_cross=0.,worst_midstop=0.;
  double max_si_acc=0.,max_yaw_acc=0.,max_si_jerk=0.,max_yaw_jerk=0.;
  double max_fraction_rate=0.,max_fraction_acc=0.,peak_actual_acc=0.,peak_actual_yaw_acc=0.;
  int yaw_reversals=0,fast_reversals=0,mid_starts=0,replacements=0,rejections=0,overrides=0;
  std::string first_fault,final_reason;
};
bool normalReason(const std::string& r) {return r=="tracking"||r=="tracking_forward"||r=="tracking_forward_yaw_coasting"||r=="goal_position_settling"||r=="goal_position_reached"||r=="goal_reached"||r=="final_endpoint_capture";}
Metrics run(const PlantSpec& spec,const Scene& scene) {
  const auto p=policy();const auto& c=p.control;TrackerCore core(c);Plant body(spec);
  body.position.y()=scene.offset;body.heading=scene.heading;
  const auto original=trajectory(scene);const auto goal=task(original);
  const auto geometry=PreparedGeometry::build(c,original,{});
  EXPECT_TRUE(geometry->failure.empty())<<geometry->failure;
  EXPECT_TRUE(geometry->derivatives_valid);
  EXPECT_TRUE(core.receiveTask(goal,kRos,kReceipt));
  EXPECT_TRUE(core.receiveOdom(body.measurement(kRos,scene.noise,0,scene.heading_noise_degrees),kRos,kReceipt));
  EXPECT_TRUE(core.receiveTrajectory(original,kRos,kReceipt))<<core.candidateReason();
  std::ofstream trace;const char* dir=std::getenv("D1MAX_FORWARD_OUTPUT");
  static bool summary_initialized=false;
  if(dir&&!summary_initialized) {
    std::ofstream summary(std::filesystem::path(dir)/"summary.csv");
    summary<<"scene,plant,arrived,stop_time_s,final_error_m,peak_cross_m,final_cross_m,worst_midstop_s,mid_starts,yaw_reversals,subsecond_reversals,max_sdk_fraction_rate,max_sdk_fraction_acceleration,peak_assumed_body_accel,peak_assumed_yaw_accel,accepted_replacements,rejected_candidates,conditioner_priority_overrides,first_fault,final_reason,max_si_acceleration,max_si_yaw_acceleration,max_si_jerk,max_si_yaw_jerk\n";
    summary_initialized=true;
  }
  if(dir){trace.open(std::filesystem::path(dir)/(std::string(scene.name)+"_"+spec.name+".csv"));trace<<std::setprecision(12)<<"t,x,y,yaw,v,w,request_v,request_w,sdk_v,sdk_w,cross,goal_error,curve_id,reason\n";}
  Metrics result;sdk::SdkMoveFractions previous_input;Output previous_output;
  double previous_rate_v=0.,previous_rate_w=0.,last_sign_time=-100.,stationary_since=-1.,midstop=0.;
  double previous_si_acc=0.,previous_si_yaw_acc=0.;
  int last_sign=0;bool previous_mid_moving=false,ever_mid_moving=false;
  std::int64_t candidate_id=1;
  for(int i=1;i<=2400;++i) {
    const double elapsed=i*kDt,now=kRos+elapsed,receipt=kReceipt+elapsed;
    const auto measured=body.measurement(now,scene.noise,i,scene.heading_noise_degrees);
    EXPECT_EQ(measured.source_stamp_ns,std::llround(now*1e9));
    EXPECT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    if(core.active()&&i%4==0&&(scene.replace||scene.reject)) {
      auto candidate=cropped(original,*geometry,body,++candidate_id,now);
      if(scene.reject)candidate.points[2].x()=std::numeric_limits<double>::quiet_NaN();
      const auto old_id=core.trajectoryId();const bool accepted=core.receiveTrajectory(candidate,now,receipt,true);
      if(accepted)++result.replacements;else ++result.rejections;
      if(scene.reject){EXPECT_FALSE(accepted);EXPECT_EQ(core.trajectoryId(),old_id);EXPECT_TRUE(core.active());}
    }
    const auto out=core.step(now,receipt,true);
    const auto input=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,"holonomic_xyyaw",p.mapping,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(input.left_right,0.);
    EXPECT_TRUE(std::isfinite(out.forward)&&std::isfinite(out.yaw_rate));
    EXPECT_GE(out.forward,-1e-12);EXPECT_LE(out.forward,c.max_speed+1e-10);
    EXPECT_LE(std::abs(out.yaw_rate),c.max_yaw_rate+1e-10);
    EXPECT_LE(std::abs(input.forward_back),p.mapping.planar.maximum_fraction+1e-10);
    EXPECT_LE(std::abs(input.yaw),p.mapping.yaw.maximum_fraction+1e-10);
    const double rate_v=(input.forward_back-previous_input.forward_back)/kDt;
    const double rate_w=(input.yaw-previous_input.yaw)/kDt;
    const double si_acc=(out.forward-previous_output.forward)/kDt,si_yaw_acc=(out.yaw_rate-previous_output.yaw_rate)/kDt;
    const bool ordinary=out.reason.find("reached")==std::string::npos&&!out.frozen;
    if(ordinary) {
      result.max_si_acc=std::max(result.max_si_acc,std::abs(out.forward-previous_output.forward)/kDt);
      result.max_yaw_acc=std::max(result.max_yaw_acc,std::abs(out.yaw_rate-previous_output.yaw_rate)/kDt);
      result.max_si_jerk=std::max(result.max_si_jerk,std::abs(si_acc-previous_si_acc)/kDt);
      result.max_yaw_jerk=std::max(result.max_yaw_jerk,std::abs(si_yaw_acc-previous_si_yaw_acc)/kDt);
      EXPECT_LE(std::abs(out.forward-previous_output.forward),c.max_acceleration*kDt+1e-9);
      EXPECT_LE(std::abs(out.yaw_rate-previous_output.yaw_rate),c.max_yaw_acceleration*kDt+1e-9);
      result.max_fraction_rate=std::max({result.max_fraction_rate,std::abs(rate_v),std::abs(rate_w)});
      result.max_fraction_acc=std::max({result.max_fraction_acc,std::abs(rate_v-previous_rate_v)/kDt,std::abs(rate_w-previous_rate_w)/kDt});
      if(core.controlDiagnostic().input_limit_override)++result.overrides;
      else {
        if(c.max_jerk>0.)EXPECT_LE(std::abs(si_acc-previous_si_acc)/kDt,c.max_jerk+1e-7);
        if(c.max_yaw_jerk>0.)EXPECT_LE(std::abs(si_yaw_acc-previous_si_yaw_acc)/kDt,c.max_yaw_jerk+1e-7);
        EXPECT_LE(std::abs(rate_v),c.max_input_rate+1e-8);EXPECT_LE(std::abs(rate_w),c.max_input_rate+1e-8);
        EXPECT_LE(std::abs(rate_v-previous_rate_v)/kDt,c.max_input_acceleration+1e-7);
        EXPECT_LE(std::abs(rate_w-previous_rate_w)/kDt,c.max_input_acceleration+1e-7);
      }
    }
    const double goal_error=(body.position-goal.goal).head<2>().norm(),cross=pathError(*geometry,body.position);
    result.peak_cross=std::max(result.peak_cross,cross);result.final_cross=cross;
    if(elapsed>2.&&goal_error>.6) {
      const bool moving=input.forward_back+1e-9>=spec.forward_knee;
      if(moving&&!previous_mid_moving&&ever_mid_moving)++result.mid_starts;
      if(moving){midstop=0.;ever_mid_moving=true;}else if(ever_mid_moving)midstop+=kDt;
      result.worst_midstop=std::max(result.worst_midstop,midstop);previous_mid_moving=moving;
    }
    const int sign=input.yaw>=spec.yaw_knee-1e-9?1:input.yaw<=-spec.yaw_knee+1e-9?-1:0;
    if(sign&&last_sign&&sign!=last_sign) {
      ++result.yaw_reversals;if(elapsed-last_sign_time<1.)++result.fast_reversals;last_sign_time=elapsed;
    }
    if(sign&&sign!=last_sign){last_sign=sign;if(last_sign_time<0.)last_sign_time=elapsed;}
    if(!normalReason(out.reason)&&result.first_fault.empty())result.first_fault=out.reason;
    const bool stationary=goal_error<=c.goal_tolerance&&std::abs(body.speed)<=c.stationary_linear_threshold_mps&&
      std::abs(body.yaw_rate)<=c.stationary_angular_threshold_radps&&out.forward==0.&&out.yaw_rate==0.;
    if(!stationary)stationary_since=-1.;else if(stationary_since<0.)stationary_since=elapsed;
    if(stationary_since>=0.&&elapsed-stationary_since>=1.-1e-9){result.arrived=true;result.stop_time=elapsed;}
    if(trace)trace<<elapsed<<','<<body.position.x()<<','<<body.position.y()<<','<<body.heading<<','<<body.speed<<','<<body.yaw_rate<<','<<out.forward<<','<<out.yaw_rate<<','<<input.forward_back<<','<<input.yaw<<','<<cross<<','<<goal_error<<','<<core.trajectoryId()<<','<<out.reason<<'\n';
    core.recordAppliedOutput(out.forward,out.lateral,out.yaw_rate,now,receipt);
    const double old_v=body.speed,old_w=body.yaw_rate;body.advance(input,elapsed);
    result.peak_actual_acc=std::max(result.peak_actual_acc,std::abs(body.speed-old_v)/kDt);
    result.peak_actual_yaw_acc=std::max(result.peak_actual_yaw_acc,std::abs(body.yaw_rate-old_w)/kDt);
    previous_input=input;previous_output=out;previous_rate_v=rate_v;previous_rate_w=rate_w;
    previous_si_acc=si_acc;previous_si_yaw_acc=si_yaw_acc;
    result.final_reason=out.reason;result.final_error=(body.position-goal.goal).head<2>().norm();
    if(result.arrived)break;
  }
  if(dir) {
    std::ofstream summary(std::filesystem::path(dir)/"summary.csv",std::ios::app);
    summary<<std::setprecision(10)<<scene.name<<','<<spec.name<<','<<result.arrived<<','<<result.stop_time<<','<<result.final_error<<','<<result.peak_cross<<','<<result.final_cross<<','<<result.worst_midstop<<','<<result.mid_starts<<','<<result.yaw_reversals<<','<<result.fast_reversals<<','<<result.max_fraction_rate<<','<<result.max_fraction_acc<<','<<result.peak_actual_acc<<','<<result.peak_actual_yaw_acc<<','<<result.replacements<<','<<result.rejections<<','<<result.overrides<<','<<result.first_fault<<','<<result.final_reason<<','<<result.max_si_acc<<','<<result.max_yaw_acc<<','<<result.max_si_jerk<<','<<result.max_yaw_jerk<<'\n';
  }
  return result;
}
class ForwardClosedLoop:public ::testing::TestWithParam<std::tuple<int,int>> {};
TEST(ForwardControllerContract, SteeringPreviewDoesNotRaiseANormallyMovingCurvesForwardSpeed) {
  auto small=policy().control,large=small;
  small.min_lookahead_distance_m=.07;large.min_lookahead_distance_m=.40;
  small.max_lookahead_distance_m=large.max_lookahead_distance_m=.80;
  TrackerCore a(small),b(large);
  Trajectory path;path.session_id=small.session_id;path.frame_id=small.planning_frame;
  path.generation=1;path.id=1;path.start_time=kRos;path.frozen_yaw=0.;
  for(int i=0;i<30;++i)path.points.emplace_back((i-1)*.1,0.,.55);
  for(std::size_t i=0;i<path.points.size()+4;++i)path.knots.push_back((static_cast<double>(i)-3.)*.60);
  const auto goal=task(path);Plant measured(kPlants[0]);measured.speed=1./6.;
  for(auto* core:{&a,&b}) {
    ASSERT_TRUE(core->receiveTask(goal,kRos,kReceipt));
    ASSERT_TRUE(core->receiveOdom(measured.measurement(kRos,false,0),kRos,kReceipt));
    ASSERT_TRUE(core->receiveTrajectory(path,kRos,kReceipt))<<core->candidateReason();
  }
  for(int tick=1;tick<=40;++tick) {
    const double now=kRos+tick*kDt,receipt=kReceipt+tick*kDt;
    // Explicit independent observations isolate the geometry parameter. This
    // is a contract test, separate from the integrated physical loops above.
    measured.position.x()=tick*kDt/6.;const auto observation=measured.measurement(now,false,tick);
    for(auto* core:{&a,&b}){ASSERT_TRUE(core->receiveOdom(observation,now,receipt));core->refreshTaskLease(receipt);}
    const auto x=a.step(now,receipt,true),y=b.step(now,receipt,true);
    EXPECT_FALSE(x.frozen);EXPECT_FALSE(y.frozen);EXPECT_GT(x.forward,0.);
    EXPECT_NEAR(x.forward,y.forward,1e-12)<<"steering preview changed normal forward tracking";
    EXPECT_DOUBLE_EQ(x.lateral,0.);EXPECT_DOUBLE_EQ(y.lateral,0.);
    a.recordAppliedOutput(x.forward,x.lateral,x.yaw_rate,now,receipt);
    b.recordAppliedOutput(y.forward,y.lateral,y.yaw_rate,now,receipt);
  }
}
TEST_P(ForwardClosedLoop, SmoothProgressAndActualArrival) {
  const auto [scene_index,plant_index]=GetParam();const auto& scene=kScenes[scene_index];const auto& plant=kPlants[plant_index];
  SCOPED_TRACE(std::string(scene.name)+"/"+plant.name);const auto result=run(plant,scene);
  EXPECT_TRUE(result.arrived)<<"error="<<result.final_error<<" reason="<<result.final_reason;
  EXPECT_TRUE(result.first_fault.empty())<<result.first_fault;
  EXPECT_LE(result.final_error,.20);EXPECT_LE(result.peak_cross,scene.offset+.18);
  EXPECT_EQ(result.fast_reversals,0)<<"sub-second left/right physical working commands";
  EXPECT_LE(result.worst_midstop,.25)<<"midroute forward command fell below actuator working range";
  EXPECT_LE(result.mid_starts,2)<<"repeated forward breakaway cycles";
  if(scene.replace)EXPECT_GT(result.replacements,5);
  if(scene.reject)EXPECT_GT(result.rejections,5);
}
INSTANTIATE_TEST_SUITE_P(ProductionController,ForwardClosedLoop,
  ::testing::Combine(::testing::Range(0,static_cast<int>(kScenes.size())),::testing::Range(0,static_cast<int>(kPlants.size()))),
  [](const auto& info){return std::string(kScenes[std::get<0>(info.param)].name)+"_"+kPlants[std::get<1>(info.param)].name;});
class ForwardHeadingAcquisition:public ::testing::TestWithParam<std::tuple<int,int>> {};
TEST_P(ForwardHeadingAcquisition, NoisyOppositeHeadingRotatesThenMakesSmoothProgress) {
  const auto [scene_index,plant_index]=GetParam();const auto& scene=kHeadingScenes[scene_index];
  const auto& plant=kPlants[plant_index];SCOPED_TRACE(std::string(scene.name)+"/"+plant.name);
  const auto result=run(plant,scene);
  EXPECT_TRUE(result.arrived)<<"error="<<result.final_error<<" reason="<<result.final_reason;
  EXPECT_TRUE(result.first_fault.empty())<<result.first_fault;
  EXPECT_LE(result.final_error,.20);EXPECT_LE(result.peak_cross,.18);
  EXPECT_EQ(result.fast_reversals,0)<<"sub-second left/right physical working commands";
  EXPECT_LE(result.worst_midstop,.25)<<"forward fell below working range after initial acquisition";
  EXPECT_LE(result.mid_starts,2)<<"repeated forward breakaway cycles";
}
INSTANTIATE_TEST_SUITE_P(ProductionController,ForwardHeadingAcquisition,
  ::testing::Combine(::testing::Range(0,static_cast<int>(kHeadingScenes.size())),::testing::Range(0,static_cast<int>(kPlants.size()))),
  [](const auto& info){return std::string(kHeadingScenes[std::get<0>(info.param)].name)+"_"+kPlants[std::get<1>(info.param)].name;});
} // namespace

// Extended long-route/feedback-delay sensitivity. The original 40 tests above
// remain unchanged. These stronger response families are not physical calibration.
namespace {
struct SensitivityScene { const char* name; double radius, offset, heading; };
const std::array<SensitivityScene,3> kSensitivityScenes{{
  {"long_straight_noise",0.,0.,0.},
  {"long_straight_offset",0.,.20,10.*kPi/180.},
  {"wide_curve",8.,0.,0.}
}};
// Keep every original family verbatim. The added families alter only the yaw
// working speed, retaining the original delayed family's knee, gains and lag.
const std::array<PlantSpec,5> kSensitivityPlants{{
  kPlants[0],kPlants[1],kPlants[2],
  {"unmeasured_yaw45",.30,.20,.20,.45,1.0,1.5,.20,.25,.15},
  {"unmeasured_yaw60",.30,.20,.20,.60,1.0,1.5,.20,.25,.15}
}};
constexpr double kSensitivityLength=25.;
Eigen::Vector3d sensitivityPoint(const SensitivityScene& scene,double station) {
  if(scene.radius==0.)return {station,0.,.55};
  return {scene.radius*std::sin(station/scene.radius),
    scene.radius*(1.-std::cos(station/scene.radius)),.55};
}
Trajectory sensitivityTrajectory(const SensitivityScene& scene) {
  auto t=trajectory(kScenes[0]);t.points.clear();t.knots.clear();
  std::vector<double> stations{0.,0.,0.,.02,.06,.12,.20};
  for(double s=.30;s<kSensitivityLength-.20-1e-8;s+=.10)stations.push_back(s);
  for(double s:{24.80,24.88,24.94,24.98,25.,25.,25.})stations.push_back(s);
  for(double s:stations)t.points.push_back(sensitivityPoint(scene,s));
  // 0.10 m / 0.40 s = 0.25 m/s reference; the 25 m finite curve remains
  // inside the production 120 s representation bound without changing it.
  for(std::size_t i=0;i<t.points.size()+4;++i)t.knots.push_back((double(i)-3.)*.40);
  return t;
}
struct SensitivityMetrics {
  bool arrived=false;std::string first_fault,final_reason;
  int stable_samples=0,body_reversals=0,command_reversals=0,mid_starts=0;
  int body_subsecond_reversals=0,sdk_subsecond_reversals=0;
  double fastest_body_reversal=std::numeric_limits<double>::infinity();
  double fastest_sdk_reversal=std::numeric_limits<double>::infinity();
  double fastest_body_cycle=std::numeric_limits<double>::infinity();
  double fastest_sdk_cycle=std::numeric_limits<double>::infinity();
  double final_error=0.,stable_heading_min=100.,stable_heading_max=-100.;
  double stable_peak_cross=0.,stable_cross_squared=0.,late_cross_squared=0.;
  int late_samples=0;
  double max_body_yaw=0.,max_forward_delta=0.,max_yaw_delta=0.,max_yaw_jerk=0.;
  double max_sdk_fraction_rate=0.,max_sdk_fraction_acceleration=0.;
  int conditioner_priority_overrides=0;
  double longest_midstop=0.,max_source_age=0.;
};
SensitivityMetrics runSensitivity(const SensitivityScene& scene,const PlantSpec& spec,int delay_ticks) {
  const auto p=policy();const auto& c=p.control;TrackerCore core(c);Plant body(spec);
  body.position.y()=scene.offset;body.heading=scene.heading;
  const auto curve=sensitivityTrajectory(scene);const auto goal=task(curve);
  const auto geometry=PreparedGeometry::build(c,curve,{});
  EXPECT_TRUE(geometry->failure.empty())<<geometry->failure;
  EXPECT_TRUE(core.receiveTask(goal,kRos,kReceipt));
  // Each history entry is acquired at its own original source time. The
  // transport delivers that very entry later; no old pose is stamped as now.
  std::deque<Odom> history;
  for(int i=-delay_ticks;i<=0;++i)history.push_back(body.measurement(kRos+i*kDt,true,i));
  EXPECT_TRUE(core.receiveOdom(history.front(),kRos,kReceipt));history.pop_front();
  EXPECT_TRUE(core.receiveTrajectory(curve,kRos,kReceipt))<<core.candidateReason();
  const std::string name=std::string(scene.name)+"_"+spec.name+"_feedback"+std::to_string(delay_ticks*50)+"ms";
  const char* data_dir=std::getenv("D1MAX_SENSITIVITY_OUTPUT");
  if(!data_dir)data_dir=std::getenv("D1MAX_FORWARD_OUTPUT");
  const auto output=data_dir?std::filesystem::path(data_dir):std::filesystem::path{};
  std::ofstream trace;if(data_dir)trace.open(output/(name+".csv"));
  trace<<std::setprecision(12)<<"t,source_age,x,y,body_heading,body_v,body_w,request_v,request_w,sdk_v,sdk_w,station,cross,heading_residual,reason\n";
  SensitivityMetrics m;Output previous;double previous_yaw_acc=0.,previous_forward_acc=0.;
  sdk::SdkMoveFractions previous_input;double previous_sdk_v_rate=0.,previous_sdk_w_rate=0.;
  int previous_body_sign=0,previous_command_sign=0;
  std::deque<double> body_sign_times,sdk_sign_times;
  const auto recordSign=[&](int sign,int& previous_sign,std::deque<double>& sign_times,
      double elapsed,int& reversals,int& fast,double& minimum_interval,double& minimum_cycle) {
    if(!sign||sign==previous_sign)return;
    if(previous_sign) {
      ++reversals;const double interval=elapsed-sign_times.back();
      minimum_interval=std::min(minimum_interval,interval);
      if(interval<1.-1e-9)++fast;
      if(sign_times.size()==2)minimum_cycle=std::min(minimum_cycle,elapsed-sign_times.front());
    }
    sign_times.push_back(elapsed);if(sign_times.size()>2)sign_times.pop_front();previous_sign=sign;
  };
  double stationary_since=-1.,midstop=0.;bool moving_seen=false,previous_moving=false;
  for(int i=1;i<=4000;++i) {
    const double elapsed=i*kDt,now=kRos+elapsed,receipt=kReceipt+elapsed;
    history.push_back(body.measurement(now,true,i));const auto measured=history.front();history.pop_front();
    const double source_age=now-measured.stamp;
    EXPECT_NEAR(source_age,delay_ticks*kDt,1e-10);
    EXPECT_EQ(measured.source_stamp_ns,std::llround(measured.stamp*1e9));
    EXPECT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);
    const auto input=sdk::mappedSdkFractions(out.forward,out.lateral,out.yaw_rate,"holonomic_xyyaw",p.mapping,c.max_speed,c.max_yaw_rate);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(input.left_right,0.);
    EXPECT_TRUE(std::isfinite(out.forward)&&std::isfinite(out.yaw_rate));
    EXPECT_GE(out.forward,-1e-12);EXPECT_LE(out.forward,c.max_speed+1e-10);
    EXPECT_LE(std::abs(out.yaw_rate),c.max_yaw_rate+1e-10);
    EXPECT_LE(std::abs(input.forward_back),p.mapping.planar.maximum_fraction+1e-10);
    EXPECT_LE(std::abs(input.yaw),p.mapping.yaw.maximum_fraction+1e-10);
    const double forward_acc=(out.forward-previous.forward)/kDt;
    const double yaw_acc=(out.yaw_rate-previous.yaw_rate)/kDt;
    const double sdk_v_rate=(input.forward_back-previous_input.forward_back)/kDt;
    const double sdk_w_rate=(input.yaw-previous_input.yaw)/kDt;
    const bool ordinary=out.reason.find("reached")==std::string::npos&&!out.frozen;
    if(ordinary) {
      EXPECT_LE(std::abs(forward_acc),c.max_acceleration+1e-8);
      EXPECT_LE(std::abs(yaw_acc),c.max_yaw_acceleration+1e-8);
      m.max_sdk_fraction_rate=std::max({m.max_sdk_fraction_rate,std::abs(sdk_v_rate),std::abs(sdk_w_rate)});
      m.max_sdk_fraction_acceleration=std::max({m.max_sdk_fraction_acceleration,
        std::abs(sdk_v_rate-previous_sdk_v_rate)/kDt,std::abs(sdk_w_rate-previous_sdk_w_rate)/kDt});
      // Match the original 40's priority policy: actual collision/stop and
      // explicit conditioner feasibility overrides are not hidden as smooth
      // tracking. Acceleration remains checked; every override is reported.
      if(core.controlDiagnostic().input_limit_override)++m.conditioner_priority_overrides;
      else {
        if(c.max_jerk>0.)EXPECT_LE(std::abs(forward_acc-previous_forward_acc)/kDt,c.max_jerk+1e-7);
        if(c.max_yaw_jerk>0.)EXPECT_LE(std::abs(yaw_acc-previous_yaw_acc)/kDt,c.max_yaw_jerk+1e-7);
        EXPECT_LE(std::abs(sdk_v_rate),c.max_input_rate+1e-8);
        EXPECT_LE(std::abs(sdk_w_rate),c.max_input_rate+1e-8);
        EXPECT_LE(std::abs(sdk_v_rate-previous_sdk_v_rate)/kDt,c.max_input_acceleration+1e-7);
        EXPECT_LE(std::abs(sdk_w_rate-previous_sdk_w_rate)/kDt,c.max_input_acceleration+1e-7);
      }
    }
    const double station=scene.radius==0.?body.position.x():
      scene.radius*std::atan2(body.position.x(),scene.radius-body.position.y());
    const double heading_residual=angle(body.heading-(scene.radius==0.?0.:station/scene.radius));
    const double cross=scene.radius==0.?std::abs(body.position.y()):
      std::abs(std::hypot(body.position.x(),body.position.y()-scene.radius)-scene.radius);
    const double goal_error=(body.position-goal.goal).head<2>().norm();
    if(station>=5.&&station<=22.) {
      ++m.stable_samples;m.stable_heading_min=std::min(m.stable_heading_min,heading_residual);
      m.stable_heading_max=std::max(m.stable_heading_max,heading_residual);
      m.stable_peak_cross=std::max(m.stable_peak_cross,cross);m.stable_cross_squared+=cross*cross;
      m.max_body_yaw=std::max(m.max_body_yaw,std::abs(body.yaw_rate));
      // Significant *physical* yaw reversals, including slow limit cycles.
      // Legitimate changes in route curvature are reported, not prohibited.
      const int sign=body.yaw_rate>=.15?1:body.yaw_rate<=-.15?-1:0;
      recordSign(sign,previous_body_sign,body_sign_times,elapsed,m.body_reversals,
        m.body_subsecond_reversals,m.fastest_body_reversal,m.fastest_body_cycle);
      const int command_sign=input.yaw>=spec.yaw_knee-1e-9?1:input.yaw<=-spec.yaw_knee+1e-9?-1:0;
      recordSign(command_sign,previous_command_sign,sdk_sign_times,elapsed,m.command_reversals,
        m.sdk_subsecond_reversals,m.fastest_sdk_reversal,m.fastest_sdk_cycle);
    }
    if(station>=15.&&station<=22.) {m.late_cross_squared+=cross*cross;++m.late_samples;}
    if(station>=2.&&station<=22.) {
      const bool moving=input.forward_back+1e-9>=spec.forward_knee;
      if(moving&&!previous_moving&&moving_seen)++m.mid_starts;
      if(moving){midstop=0.;moving_seen=true;}else if(moving_seen)midstop+=kDt;
      m.longest_midstop=std::max(m.longest_midstop,midstop);previous_moving=moving;
    }
    m.max_forward_delta=std::max(m.max_forward_delta,std::abs(out.forward-previous.forward));
    m.max_yaw_delta=std::max(m.max_yaw_delta,std::abs(out.yaw_rate-previous.yaw_rate));
    m.max_yaw_jerk=std::max(m.max_yaw_jerk,std::abs(yaw_acc-previous_yaw_acc)/kDt);
    m.max_source_age=std::max(m.max_source_age,source_age);
    if(!normalReason(out.reason)&&m.first_fault.empty())m.first_fault=out.reason;
    const bool stopped=goal_error<=c.goal_tolerance&&std::abs(body.speed)<=c.stationary_linear_threshold_mps&&
      std::abs(body.yaw_rate)<=c.stationary_angular_threshold_radps&&out.forward==0.&&out.yaw_rate==0.;
    if(!stopped)stationary_since=-1.;else if(stationary_since<0.)stationary_since=elapsed;
    if(stationary_since>=0.&&elapsed-stationary_since>=1.-1e-9)m.arrived=true;
    trace<<elapsed<<','<<source_age<<','<<body.position.x()<<','<<body.position.y()<<','<<body.heading<<','
      <<body.speed<<','<<body.yaw_rate<<','<<out.forward<<','<<out.yaw_rate<<','<<input.forward_back<<','
      <<input.yaw<<','<<station<<','<<cross<<','<<heading_residual<<','<<out.reason<<'\n';
    m.final_error=goal_error;m.final_reason=out.reason;
    core.recordAppliedOutput(out.forward,out.lateral,out.yaw_rate,now,receipt);
    body.advance(input,elapsed);previous=out;previous_yaw_acc=yaw_acc;previous_forward_acc=forward_acc;
    previous_input=input;previous_sdk_v_rate=sdk_v_rate;previous_sdk_w_rate=sdk_w_rate;
    if(m.arrived)break;
  }
  if(data_dir) {
  static bool header=false;
  std::ofstream summary(output/"sensitivity_summary.csv",header?std::ios::app:std::ios::out);
  if(!header){summary<<"scene,plant,feedback_delay_ms,arrived,stable_heading_p2p_deg,body_reversals,sdk_working_reversals,stable_cross_rms,late_cross_rms,stable_cross_peak,stable_body_yaw_peak,mid_starts,max_midstop,final_error,first_fault,final_reason,max_source_age,max_command_v_delta,max_command_w_delta,max_command_yaw_jerk,body_subsecond_reversals,sdk_subsecond_reversals,fastest_body_reversal_interval_s,fastest_sdk_reversal_interval_s,fastest_body_fullcycle_s,fastest_sdk_fullcycle_s,max_sdk_fraction_rate,max_sdk_fraction_acceleration,conditioner_priority_overrides\n";header=true;}
  const double heading_range=(m.stable_heading_max-m.stable_heading_min)*180./kPi;
  const double cross_rms=std::sqrt(m.stable_cross_squared/std::max(1,m.stable_samples));
  const double late_rms=std::sqrt(m.late_cross_squared/std::max(1,m.late_samples));
  summary<<std::setprecision(10)<<scene.name<<','<<spec.name<<','<<delay_ticks*50<<','<<m.arrived<<','<<heading_range<<','
    <<m.body_reversals<<','<<m.command_reversals<<','<<cross_rms<<','<<late_rms<<','<<m.stable_peak_cross<<','<<m.max_body_yaw<<','
    <<m.mid_starts<<','<<m.longest_midstop<<','<<m.final_error<<','<<m.first_fault<<','<<m.final_reason<<','<<m.max_source_age<<','
    <<m.max_forward_delta<<','<<m.max_yaw_delta<<','<<m.max_yaw_jerk<<','<<m.body_subsecond_reversals<<','
    <<m.sdk_subsecond_reversals<<','<<m.fastest_body_reversal<<','<<m.fastest_sdk_reversal<<','
    <<m.fastest_body_cycle<<','<<m.fastest_sdk_cycle<<','<<m.max_sdk_fraction_rate<<','
    <<m.max_sdk_fraction_acceleration<<','<<m.conditioner_priority_overrides<<'\n';
  }
  return m;
}
class YawSensitivity:public ::testing::TestWithParam<std::tuple<int,int,int>> {};
TEST_P(YawSensitivity, LongRouteHasNoPersistentStraightLimitCycle) {
  const auto [s,p,d]=GetParam();const auto& scene=kSensitivityScenes[s];
  const auto result=runSensitivity(scene,kSensitivityPlants[p],d);
  EXPECT_TRUE(result.arrived)<<result.first_fault<<" / "<<result.final_reason;
  EXPECT_TRUE(result.first_fault.empty())<<result.first_fault;
  EXPECT_GT(result.stable_samples,100);
  EXPECT_LE(result.stable_peak_cross,.18)<<"unchanged original cross-track envelope";
  EXPECT_LE(result.longest_midstop,.25)<<"unchanged original midroute stop envelope";
  EXPECT_LE(result.mid_starts,2)<<"unchanged original restart envelope";
  EXPECT_GT(result.late_samples,100);
  EXPECT_LE(std::sqrt(result.late_cross_squared/std::max(1,result.late_samples)),.06)
    <<"cannot trade reduced steering for persistent lateral displacement";
  // A diagnostic criterion fixed before observing this experiment: a straight
  // route should not repeatedly reverse a substantial physical yaw response
  // while traversing >12 degrees peak-to-peak after the first five metres.
  // This is NOT a calibrated field acceptance threshold or a production gate.
  if(scene.radius==0.) {
    EXPECT_EQ(result.body_subsecond_reversals,0)<<"do not trade low-frequency sway for fast physical reversal";
    EXPECT_EQ(result.sdk_subsecond_reversals,0)<<"do not alternate effective left/right SDK commands in <1s";
    EXPECT_FALSE(result.body_reversals>=6&&result.stable_heading_max-result.stable_heading_min>12.*kPi/180.)
      <<"persistent straight-line yaw cycle: reversals="<<result.body_reversals
      <<" heading_p2p_deg="<<(result.stable_heading_max-result.stable_heading_min)*180./kPi;
  }
}
INSTANTIATE_TEST_SUITE_P(ProductionCore,YawSensitivity,
  ::testing::Combine(::testing::Range(0,3),::testing::Range(0,5),::testing::Range(0,3)),
  [](const auto& info){return std::string(kSensitivityScenes[std::get<0>(info.param)].name)+"_"+
    kSensitivityPlants[std::get<1>(info.param)].name+"_feedback"+std::to_string(std::get<2>(info.param)*50)+"ms";});
} // namespace
