#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include "execution_kinematics.hpp"

namespace d1monitor::execution3 {
// This is an SDK-input engineering policy, not a physical velocity calibration.
struct SdkInputAxisMapping {
  double effective_fraction=.20,maximum_fraction=.30,blend_demand_ratio=.10;
  double rise_rate_per_s=.30,fall_rate_per_s=.45;
  void validate(double fraction_cap=.40)const {
    for(const double v:{effective_fraction,maximum_fraction,blend_demand_ratio,rise_rate_per_s,fall_rate_per_s})
      if(!std::isfinite(v))throw std::invalid_argument("sdk_input_mapping_nonfinite");
    if(effective_fraction<=0.||effective_fraction>maximum_fraction||maximum_fraction>fraction_cap||
       blend_demand_ratio<=0.||blend_demand_ratio>=1.||rise_rate_per_s<=0.||rise_rate_per_s>1.||
       fall_rate_per_s<=0.||fall_rate_per_s>1.)
      throw std::invalid_argument("sdk_input_mapping_bounds_invalid");
  }
};
struct SdkInputMapping {
  bool enabled=false;
  bool component_cap=false;
  bool direct_breakaway=false;
  // V4 follows each current typed command. Its near-zero ratio blend is a
  // static input response, not a startup timer or an assumed robot inertia.
  bool continuous_command_ratio=false;
  // Optional V3 command-domain shaping. Zero retains the sealed old V3
  // contract; enabled profiles reach the same effective floor in finite time.
  double startup_transition_s=0.,fraction_acceleration_per_s2=0.;
  bool smoothTransitions()const{return startup_transition_s>0.;}
  SdkInputAxisMapping planar,yaw;
  void validate()const {
    // Only the current V4 planar response acquires the operator's 50% cap.
    // The yaw and archived contracts retain their original bounds.
    planar.validate(continuous_command_ratio?.50:.40);yaw.validate();
    if(!component_cap&&(planar.maximum_fraction>.30||yaw.maximum_fraction>.30))
      throw std::invalid_argument("sdk_input_mapping_v1_fraction_cap_exceeded");
    if(direct_breakaway&&!component_cap)
      throw std::invalid_argument("sdk_direct_breakaway_requires_component_cap");
    if(continuous_command_ratio&&(!component_cap||direct_breakaway||smoothTransitions()))
      throw std::invalid_argument("sdk_continuous_ratio_policy_mixed");
    if(!std::isfinite(startup_transition_s)||!std::isfinite(fraction_acceleration_per_s2)||
       startup_transition_s<0.||startup_transition_s>2.||fraction_acceleration_per_s2<0.||
       fraction_acceleration_per_s2>4.||
       (startup_transition_s==0.)!=(fraction_acceleration_per_s2==0.)||
       (smoothTransitions()&&!direct_breakaway))
      throw std::invalid_argument("sdk_input_smoothing_invalid");
    if(smoothTransitions())for(const auto&axis:{planar,yaw}) {
      // Quintic smootherstep peak first/second derivatives: 1.875 / 5.774.
      if(1.875*axis.effective_fraction/startup_transition_s>axis.rise_rate_per_s+1e-12||
         5.774*axis.effective_fraction/(startup_transition_s*startup_transition_s)>
           fraction_acceleration_per_s2+1e-12)
        throw std::invalid_argument("sdk_input_startup_transition_exceeds_derivative_bounds");
    }
  }
};
inline double mappingSmoothstep(double x){x=std::clamp(x,0.,1.);return x*x*(3.-2.*x);}
inline double mappingSmootherstep(double x){x=std::clamp(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x));}
inline double sdkDemandFraction(double ratio,const SdkInputAxisMapping&axis,bool direct_breakaway=false) {
  if(!std::isfinite(ratio)||ratio<0.)throw std::invalid_argument("sdk_input_demand_ratio_invalid");
  ratio=std::clamp(ratio,0.,1.);
  if(ratio==0.)return 0.;
  // The operator measured a substantial input dead zone. In this explicit
  // policy, a requested movement begins at the effective SDK fraction; zero
  // remains exactly zero. This is NOT a physical speed calibration.
  if(direct_breakaway)return axis.effective_fraction+
    (axis.maximum_fraction-axis.effective_fraction)*mappingSmoothstep(ratio);
  if(ratio<=axis.blend_demand_ratio)
    return axis.effective_fraction*mappingSmoothstep(ratio/axis.blend_demand_ratio);
  return axis.effective_fraction+(axis.maximum_fraction-axis.effective_fraction)*
    mappingSmoothstep((ratio-axis.blend_demand_ratio)/(1.-axis.blend_demand_ratio));
}
// V4's explicit rounded-direction normalization selects this C2 scalar shape.
// The legacy cubic shape remains attached to the old V1--V3 contracts.
inline double sdkDemandFractionV4(double ratio,const SdkInputAxisMapping&axis) {
  if(!std::isfinite(ratio)||ratio<0.)throw std::invalid_argument("sdk_input_demand_ratio_invalid");
  ratio=std::clamp(ratio,0.,1.);
  if(ratio<=axis.blend_demand_ratio)
    return axis.effective_fraction*mappingSmootherstep(ratio/axis.blend_demand_ratio);
  return axis.effective_fraction+(axis.maximum_fraction-axis.effective_fraction)*
    mappingSmootherstep((ratio-axis.blend_demand_ratio)/(1.-axis.blend_demand_ratio));
}
inline double roundedDominantPnorm16(double x,double y) {
  const double dominant=std::max(std::abs(x),std::abs(y));
  if(dominant==0.)return 0.;
  // Scaling avoids overflow/underflow. Algebraically this is the smooth
  // 16-norm, not max(|x|,|y|): crossing a diagonal has one shared derivative.
  const double minor=std::min(std::abs(x),std::abs(y))/dominant;
  return dominant*std::pow(1.+std::pow(minor,16.),1./16.);
}
inline SdkMoveFractions mappedSdkFractions(double vx,double vy,double omega,const std::string&profile,
    const SdkInputMapping&mapping,double planar_reference,double yaw_reference) {
  validateKinematicProfile(profile);mapping.validate();
  if(!mapping.enabled||!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(omega)||
     !std::isfinite(planar_reference)||planar_reference<=0.||!std::isfinite(yaw_reference)||yaw_reference<=0.||
     (profile=="forward_yaw"&&(vx<0.||vy!=0.)))
    throw std::invalid_argument("sdk_input_mapping_demand_invalid");
  const double norm=std::hypot(vx,vy);
  if(!std::isfinite(norm))throw std::invalid_argument("sdk_input_mapping_demand_invalid");
  SdkMoveFractions out;
  if(norm>0.) {
    // SDK Move fractions are per axis. V2 preserves the requested direction
    // and its dominant-axis response, including diagonal travel; a radial
    // cap would push both axes back below the observed response region.
    const auto peak=mapping.continuous_command_ratio?sdkDemandFractionV4(norm/planar_reference,mapping.planar):
      sdkDemandFraction(norm/planar_reference,mapping.planar,mapping.direct_breakaway);
    const double dominant=mapping.continuous_command_ratio?roundedDominantPnorm16(vx,vy):
      std::max(std::abs(vx),std::abs(vy));
    out.forward_back=peak*(vx/dominant);out.left_right=peak*(vy/dominant);
    const auto mapped_norm=std::hypot(out.forward_back,out.left_right);
    if(!mapping.component_cap&&mapped_norm>mapping.planar.maximum_fraction) {
      const double factor=mapping.planar.maximum_fraction/mapped_norm;
      out.forward_back*=factor;out.left_right*=factor;
    }
  }
  if(omega!=0.)out.yaw=std::copysign(mapping.continuous_command_ratio?
    sdkDemandFractionV4(std::abs(omega)/yaw_reference,mapping.yaw):
    sdkDemandFraction(std::abs(omega)/yaw_reference,mapping.yaw,mapping.direct_breakaway),omega);
  return out;
}
struct SdkMappingIdentity {
  std::string execution_id;
  uint64_t epoch=0,generation=0;
  bool operator==(const SdkMappingIdentity&other)const {
    return execution_id==other.execution_id&&epoch==other.epoch&&generation==other.generation;
  }
};
// Owned by the unique SDK writer. Curve changes deliberately do not reset it.
// V4 applies the current ratio directly, including exact zero components and
// signed turning. The historical V1--V3 shaping remains only for old profiles.
// Whole raw-zero commands stop immediately for every version.
class SdkFractionSlew {
 public:
  void reset(){previous_={};identity_={};last_time_=0.;have_history_=false;
    planar_elapsed_=yaw_elapsed_=planar_rate_=yaw_rate_=0.;planar_starting_=yaw_starting_=false;
    planar_start_value_=0.;
    goal_decelerating_=planar_resuming_=false;
    yaw_coasting_=yaw_resuming_=false;yaw_start_value_=0.;}
  SdkMoveFractions apply(SdkMoveFractions target,double vx,double vy,double omega,
      const SdkInputMapping&mapping,const SdkMappingIdentity&identity,double now,bool ordinary_goal_deceleration=false,
      bool ordinary_yaw_coasting=false) {
    mapping.validate();
    for(const double v:{target.left_right,target.forward_back,target.yaw,vx,vy,omega,now})
      if(!std::isfinite(v))throw std::invalid_argument("sdk_input_slew_nonfinite");
    if(vx==0.&&vy==0.&&omega==0.){reset();return {};}
    if(identity.execution_id.empty())throw std::invalid_argument("sdk_input_slew_identity_missing");
    const double planar_magnitude=mapping.component_cap?
      std::max(std::abs(target.forward_back),std::abs(target.left_right)):
      std::hypot(target.forward_back,target.left_right);
    if(planar_magnitude>mapping.planar.maximum_fraction+1e-12||
       std::abs(target.yaw)>mapping.yaw.maximum_fraction+1e-12)
      throw std::invalid_argument("sdk_input_slew_fraction_cap_exceeded");
    if(mapping.continuous_command_ratio) {
      // Original typed-command freshness and authority are decided before
      // mapping by Transport. Do not add a second dynamics model, replay old
      // amplitude on a new curve, or turn an ordinary sign change into a
      // fabricated zero command. Zero is exactly the current zero demand.
      if(have_history_&&identity_==identity&&now<last_time_){reset();return {};}
      if(vx==0.)target.forward_back=0.;
      if(vy==0.)target.left_right=0.;
      if(omega==0.)target.yaw=0.;
      reset();identity_=identity;last_time_=now;have_history_=true;previous_=target;
      return target;
    }
    double dt=.05;
    if(!have_history_||!(identity_==identity)) {
      reset();identity_=identity;have_history_=true;last_time_=now;
      if(!mapping.direct_breakaway)return {};
      if(mapping.smoothTransitions())dt=0.;
    }
    else {
      if(now<last_time_){reset();return {};}
      dt=std::clamp(now-last_time_,0.,mapping.smoothTransitions()?.10:.05);
    }
    last_time_=now;
    // A zero axis cannot introduce motion from rest; an existing fraction is
    // allowed to ramp down. Safety HOLD/STOP use the complete zero fast path.
    if(vx==0.)target.forward_back=0.;
    if(vy==0.)target.left_right=0.;
    if(omega==0.)target.yaw=0.;
    if(mapping.smoothTransitions())return smoothDirect(target,mapping,dt,ordinary_goal_deceleration,ordinary_yaw_coasting);
    if(mapping.direct_breakaway) {
      // Slew only ABOVE the dead zone. Ramping 0 -> 30% on every restart
      // spends the whole short-lived command in a region that cannot move
      // this robot. A zero component stops immediately (not lingering yaw
      // after alignment); a sign reversal crosses one zero command first.
      const bool reversing=(previous_.forward_back*target.forward_back<0.)||
        (previous_.left_right*target.left_right<0.);
      if(reversing) {target.forward_back=0.;target.left_right=0.;}
      else {
        if(vx==0.)previous_.forward_back=0.;
        if(vy==0.)previous_.left_right=0.;
        const double old_peak=std::max(std::abs(previous_.forward_back),std::abs(previous_.left_right));
        const double peak=std::max(std::abs(target.forward_back),std::abs(target.left_right));
        if(old_peak==0.&&peak>0.) {
          const double scale=std::min(peak,mapping.planar.effective_fraction)/peak;
          target.forward_back*=scale;target.left_right*=scale;
        } else {
          const double dx=target.forward_back-previous_.forward_back,dy=target.left_right-previous_.left_right;
          const double delta=std::hypot(dx,dy);
          const double rate=peak>old_peak?mapping.planar.rise_rate_per_s:mapping.planar.fall_rate_per_s;
          if(delta>rate*dt&&delta>0.) {
            target.forward_back=previous_.forward_back+dx*(rate*dt/delta);
            target.left_right=previous_.left_right+dy*(rate*dt/delta);
          }
          // Direction changes can otherwise cut through the dead-zone disk.
          // Keep the dominant input effective; the rate above limits excess
          // amplitude, not a claimed physical acceleration during a turn.
          const double result_peak=std::max(std::abs(target.forward_back),std::abs(target.left_right));
          const double floor=std::min(peak,mapping.planar.effective_fraction);
          if(result_peak>0.&&result_peak<floor) {
            target.forward_back*=floor/result_peak;target.left_right*=floor/result_peak;
          }
        }
      }
      if(target.yaw==0.||previous_.yaw*target.yaw<0.)target.yaw=0.;
      else if(previous_.yaw==0.)target.yaw=std::copysign(std::min(std::abs(target.yaw),mapping.yaw.effective_fraction),target.yaw);
      else {
        const double rate=std::abs(target.yaw)>std::abs(previous_.yaw)?mapping.yaw.rise_rate_per_s:mapping.yaw.fall_rate_per_s;
        target.yaw=previous_.yaw+std::clamp(target.yaw-previous_.yaw,-rate*dt,rate*dt);
      }
      previous_=target;return target;
    }
    const auto reverse=[](double previous,double&desired){
      if((previous>0.&&desired<0.)||(previous<0.&&desired>0.))desired=0.;
    };
    reverse(previous_.forward_back,target.forward_back);reverse(previous_.left_right,target.left_right);
    reverse(previous_.yaw,target.yaw);
    const double dx=target.forward_back-previous_.forward_back,dy=target.left_right-previous_.left_right;
    const double delta=std::hypot(dx,dy);
    const double rate=std::hypot(target.forward_back,target.left_right)>std::hypot(previous_.forward_back,previous_.left_right)?
      mapping.planar.rise_rate_per_s:mapping.planar.fall_rate_per_s;
    if(delta>rate*dt&&delta>0.) {
      const double factor=rate*dt/delta;
      target.forward_back=previous_.forward_back+dx*factor;target.left_right=previous_.left_right+dy*factor;
    }
    const double yaw_rate=std::abs(target.yaw)>std::abs(previous_.yaw)?mapping.yaw.rise_rate_per_s:mapping.yaw.fall_rate_per_s;
    target.yaw=previous_.yaw+std::clamp(target.yaw-previous_.yaw,-yaw_rate*dt,yaw_rate*dt);
    previous_=target;return target;
  }
 private:
  static double startupFraction(double elapsed,double duration,double floor) {
    const double x=std::clamp(elapsed/duration,0.,1.);
    return floor*x*x*x*(10.+x*(-15.+6.*x));
  }
  static double startupRate(double elapsed,double duration,double floor) {
    const double x=std::clamp(elapsed/duration,0.,1.);
    return floor*30.*x*x*(1.-x)*(1.-x)/duration;
  }
  struct AmplitudeStep {double value,rate;};
  // Finite second-order release from the carried amplitude AND rate. Positive
  // input momentum is first shed before reversing its rate; startup/coast and
  // coast/resume cannot silently restart a derivative at zero.
  static AmplitudeStep stopAmplitude(double value,double rate,const SdkInputAxisMapping&axis,
      double acceleration,double dt) {
    if(dt<=0.)return {value,rate};
    if(value<=1e-12)return {0.,0.};
    rate=std::clamp(rate,-axis.fall_rate_per_s,axis.rise_rate_per_s);
    const double peak=std::min(axis.fall_rate_per_s,std::sqrt(acceleration*value+.5*rate*rate));
    const double down=std::max(0.,(rate+peak)/acceleration);
    const double dwell=peak>0.?std::max(0.,(value+.5*rate*rate/acceleration-
      peak*peak/acceleration)/peak):0.;
    const double up=peak/acceleration;
    if(dt+1e-12>=down+dwell+up)return {0.,0.};
    double left=dt;
    const double h1=std::min(left,down);value+=rate*h1-.5*acceleration*h1*h1;
    rate-=acceleration*h1;left-=h1;
    const double h2=std::min(left,dwell);value-=peak*h2;left-=h2;
    if(left>0.){value+=rate*left+.5*acceleration*left*left;rate+=acceleration*left;}
    // The existing component cap still outranks the soft derivative history.
    if(value<=0.)return {0.,0.};
    if(value>=axis.maximum_fraction)return {axis.maximum_fraction,0.};
    return {value,rate};
  }
  static double smoothAmplitude(double value,double target,double&rate,
      const SdkInputAxisMapping&axis,double acceleration,double dt,bool allow_subfloor=false) {
    if(dt<=0.)return value;
    const int count=std::max(1,static_cast<int>(std::ceil(dt/.005)));
    const double h=dt/count,omega=acceleration/std::max(axis.rise_rate_per_s,axis.fall_rate_per_s);
    for(int i=0;i<count;++i) {
      const double change=std::clamp(omega*omega*(target-value)-2.*omega*rate,-acceleration,acceleration);
      const double next=std::clamp(rate+change*h,-axis.fall_rate_per_s,axis.rise_rate_per_s);
      value+=(rate+next)*(.5*h);rate=next;
      // Effective-domain and cap boundaries outrank soft acceleration state.
      const double floor=allow_subfloor?0.:axis.effective_fraction;
      if(value<floor){value=floor;rate=0.;}
      if(value>axis.maximum_fraction){value=axis.maximum_fraction;rate=0.;}
    }
    return value;
  }
  SdkMoveFractions smoothDirect(SdkMoveFractions target,const SdkInputMapping&m,double dt,bool goal_deceleration,
      bool yaw_coasting) {
    const auto peak=[](const SdkMoveFractions&v){return std::max(std::abs(v.forward_back),std::abs(v.left_right));};
    const bool reverse=previous_.forward_back*target.forward_back<0.||previous_.left_right*target.left_right<0.;
    if(reverse||peak(target)==0.) {
      target.forward_back=target.left_right=0.;planar_starting_=false;planar_elapsed_=planar_rate_=0.;
      planar_start_value_=0.;
      goal_decelerating_=planar_resuming_=false;
    } else if(goal_deceleration) {
      // Only an independently checked, STILL NONZERO terminal command may
      // opt into this finite normal S-down. Any raw zero above remains exact
      // zero, so collision/lease/source expiry cannot acquire a motion tail.
      goal_decelerating_=true;planar_resuming_=false;
      const auto release=stopAmplitude(peak(previous_),planar_rate_,m.planar,
        m.fraction_acceleration_per_s2,dt);
      const double p=peak(target);target.forward_back*=release.value/p;target.left_right*=release.value/p;
      planar_rate_=release.rate;planar_starting_=false;planar_elapsed_=0.;
    } else {
      const bool resuming_goal_deceleration=goal_decelerating_;
      goal_decelerating_=false;
      // Exact zero components stay zero, including during a direction change.
      if(target.forward_back==0.)previous_.forward_back=0.;
      if(target.left_right==0.)previous_.left_right=0.;
      const double target_peak=peak(target),old_peak=peak(previous_);
      if(resuming_goal_deceleration&&old_peak!=0.)planar_resuming_=true;
      if(old_peak==0.) {
        planar_starting_=true;planar_elapsed_=planar_rate_=0.;planar_start_value_=old_peak;
        planar_resuming_=false;
      }
      double amplitude;
      if(planar_starting_) {
        planar_elapsed_=std::min(m.startup_transition_s,planar_elapsed_+dt);
        amplitude=planar_start_value_+startupFraction(planar_elapsed_,m.startup_transition_s,
          m.planar.effective_fraction-planar_start_value_);
        planar_rate_=startupRate(planar_elapsed_,m.startup_transition_s,m.planar.effective_fraction-planar_start_value_);
        if(planar_elapsed_+1e-12>=m.startup_transition_s){planar_starting_=false;planar_rate_=0.;}
      } else amplitude=smoothAmplitude(old_peak,target_peak,planar_rate_,m.planar,m.fraction_acceleration_per_s2,
        dt,planar_resuming_);
      if(amplitude>=m.planar.effective_fraction)planar_resuming_=false;
      // Planar direction continuity shares the existing rate budget. The
      // dominant floor remains the engineering domain, NOT a norm/SI claim.
      if(!planar_starting_&&old_peak>=m.planar.effective_fraction) {
        SdkMoveFractions directed=target;directed.forward_back*=amplitude/target_peak;directed.left_right*=amplitude/target_peak;
        const double dx=directed.forward_back-previous_.forward_back,dy=directed.left_right-previous_.left_right;
        const double delta=std::hypot(dx,dy),step=std::min(m.planar.rise_rate_per_s,m.planar.fall_rate_per_s)*dt;
        if(delta>step&&delta>0.) {
          directed.forward_back=previous_.forward_back+dx*step/delta;
          directed.left_right=previous_.left_right+dy*step/delta;
        }
        const double p=peak(directed);
        if(p>0.){directed.forward_back*=amplitude/p;directed.left_right*=amplitude/p;}
        target.forward_back=directed.forward_back;target.left_right=directed.left_right;
      } else {target.forward_back*=amplitude/target_peak;target.left_right*=amplitude/target_peak;}
    }
    if(target.yaw==0.||previous_.yaw*target.yaw<0.) {
      target.yaw=0.;yaw_starting_=false;yaw_elapsed_=yaw_rate_=0.;
      yaw_coasting_=yaw_resuming_=false;yaw_start_value_=0.;
    } else if(yaw_coasting) {
      // Only a selected, fresh Native command can request normal steering
      // release. Retain its sign and decay the SDK INPUT amplitude, instead
      // of turning every tiny residual yaw demand back into 30 percent.
      // Raw zero, reversal, expiry and task retirement retain their exact-zero
      // priority. The input transition is not a physical yaw-rate calibration.
      yaw_coasting_=true;yaw_resuming_=false;
      const auto release=stopAmplitude(std::abs(previous_.yaw),yaw_rate_,m.yaw,
        m.fraction_acceleration_per_s2,dt);
      target.yaw=std::copysign(release.value,target.yaw);yaw_rate_=release.rate;
      yaw_starting_=false;yaw_elapsed_=0.;
    } else {
      const bool resuming_coast=yaw_coasting_;
      yaw_coasting_=false;
      const double requested=std::abs(target.yaw);
      if(resuming_coast&&previous_.yaw!=0.)yaw_resuming_=true;
      if(previous_.yaw==0.) {
        yaw_starting_=true;yaw_elapsed_=yaw_rate_=0.;yaw_start_value_=std::abs(previous_.yaw);
        yaw_resuming_=false;
      }
      double amplitude;
      if(yaw_starting_) {
        yaw_elapsed_=std::min(m.startup_transition_s,yaw_elapsed_+dt);
        amplitude=yaw_start_value_+startupFraction(yaw_elapsed_,m.startup_transition_s,
          m.yaw.effective_fraction-yaw_start_value_);
        yaw_rate_=startupRate(yaw_elapsed_,m.startup_transition_s,m.yaw.effective_fraction-yaw_start_value_);
        if(yaw_elapsed_+1e-12>=m.startup_transition_s){yaw_starting_=false;yaw_rate_=0.;}
      } else amplitude=smoothAmplitude(std::abs(previous_.yaw),requested,yaw_rate_,m.yaw,
        m.fraction_acceleration_per_s2,dt,yaw_resuming_);
      if(amplitude>=m.yaw.effective_fraction)yaw_resuming_=false;
      target.yaw=std::copysign(amplitude,target.yaw);
    }
    previous_=target;return target;
  }
  SdkMoveFractions previous_;
  SdkMappingIdentity identity_;
  double last_time_=0.;bool have_history_=false;
  double planar_elapsed_=0.,yaw_elapsed_=0.,planar_rate_=0.,yaw_rate_=0.;
  bool planar_starting_=false,yaw_starting_=false;
  bool goal_decelerating_=false,planar_resuming_=false;
  double planar_start_value_=0.;
  bool yaw_coasting_=false,yaw_resuming_=false;
  double yaw_start_value_=0.;
};
}
