#include "sdk_input_mapping.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace d1monitor::execution3;
static bool near(double a,double b,double tolerance=1e-12){return std::abs(a-b)<=tolerance;}
static double norm(const SdkMoveFractions&v){return std::hypot(v.forward_back,v.left_right);}
template<class F>static bool rejects(F fn){try{fn();return false;}catch(const std::invalid_argument&){return true;}}
int main() {
  SdkInputMapping mapping;mapping.enabled=true;mapping.validate();
  const auto mapped=[&](double x,double y,double yaw){return mappedSdkFractions(x,y,yaw,"holonomic_xyyaw",mapping,.3,.5);};
  const auto zero=mapped(0.,0.,0.);assert(zero.forward_back==0.&&zero.left_right==0.&&zero.yaw==0.);
  const auto tiny=mapped(1e-12,0.,-1e-12);assert(tiny.forward_back>0.&&tiny.forward_back<1e-18&&tiny.left_right==0.&&tiny.yaw<0.);
  assert(near(sdkDemandFraction(.1,mapping.planar),.2));
  assert(near(sdkDemandFraction(1.,mapping.planar),.3)&&near(sdkDemandFraction(2.,mapping.planar),.3));
  double previous=0.;
  for(int i=0;i<=20000;++i) {
    const auto value=sdkDemandFraction(i/20000.,mapping.planar);
    assert(value>=previous&&value<=.3+1e-12);previous=value;
  }
  // Both scalar smoothstep pieces meet with zero first derivative.
  const double epsilon=1e-7;
  for(double knot:{0.,.1,1.}) {
    const double value=sdkDemandFraction(knot,mapping.planar);
    if(knot>0.)assert(std::abs((value-sdkDemandFraction(knot-epsilon,mapping.planar))/epsilon)<1e-4);
    assert(std::abs((sdkDemandFraction(knot+epsilon,mapping.planar)-value)/epsilon)<1e-4);
  }
  for(double angle:{-3.,-2.,-1.,0.,.3,.8,1.5,2.8})for(double demand:{.001,.03,.04,.1,.3,.6}) {
    const auto value=mapped(demand*std::cos(angle),demand*std::sin(angle),.5);
    assert(norm(value)<=.3+1e-12&&std::abs(value.yaw)<=.3+1e-12);
    assert(near(value.forward_back*std::sin(angle)-value.left_right*std::cos(angle),0.));
  }
  const auto diagonal=mapped(.04/std::sqrt(2.),.04/std::sqrt(2.),0.);
  assert(diagonal.forward_back>=.2&&near(diagonal.forward_back,diagonal.left_right)&&norm(diagonal)<=.3);
  const auto top_diagonal=mapped(.3/std::sqrt(2.),.3/std::sqrt(2.),0.);
  assert(near(norm(top_diagonal),.3)&&near(top_diagonal.forward_back,.3/std::sqrt(2.)));
  const auto back=mapped(-.3,0.,-.5),side=mapped(0.,-.3,.5);
  assert(near(back.forward_back,-.3)&&back.left_right==0.&&near(back.yaw,-.3));
  assert(side.forward_back==0.&&near(side.left_right,-.3)&&near(side.yaw,.3));
  assert(rejects([&]{mapped(std::numeric_limits<double>::quiet_NaN(),0.,0.);}));
  assert(rejects([&]{mappedSdkFractions(-.1,0.,0.,"forward_yaw",mapping,.3,.5);}));
  assert(rejects([&]{mappedSdkFractions(0.,.1,0.,"forward_yaw",mapping,.3,.5);}));
  assert(rejects([&]{mappedSdkFractions(0.,0.,0.,"unknown",mapping,.3,.5);}));
  auto invalid=mapping;invalid.planar.maximum_fraction=.31;assert(rejects([&]{invalid.validate();}));
  invalid=mapping;invalid.yaw.rise_rate_per_s=std::numeric_limits<double>::infinity();assert(rejects([&]{invalid.validate();}));

  SdkFractionSlew slew;SdkMappingIdentity identity{"task",1,1};double time=1.;
  const auto step=[&](double x,double y,double yaw,double dt=.05){
    time+=dt;return slew.apply(mapped(x,y,yaw),x,y,yaw,mapping,identity,time);
  };
  auto value=step(.24,.18,.5);assert(norm(value)==0.&&value.yaw==0.);
  for(int i=0;i<25;++i) {
    const auto last=value;value=step(.24,.18,.5);
    assert(std::hypot(value.forward_back-last.forward_back,value.left_right-last.left_right)<=.015+1e-12);
    assert(value.yaw-last.yaw<=.015+1e-12&&norm(value)<=.3+1e-12);
    assert(near(value.forward_back*.18-value.left_right*.24,0.));
  }
  assert(near(norm(value),.3)&&near(value.yaw,.3));
  // Ordinary zero components ramp down while another axis is requested.
  auto first=step(.3,0.,0.);assert(first.left_right>0.&&first.yaw>0.);
  for(int i=0;i<25;++i) {
    const auto last=first;first=step(.3,0.,0.);
    assert(std::hypot(first.forward_back-last.forward_back,first.left_right-last.left_right)<=.0225+1e-12);
    assert(last.yaw-first.yaw<=.0225+1e-12);
  }
  assert(first.left_right==0.&&first.yaw==0.&&near(first.forward_back,.3));
  // Opposite input cannot cross either SDK sign without a zero output first.
  for(int i=0;i<25;++i)step(.3,0.,.5);
  double old_forward=.3,old_yaw=.3;bool saw_forward_zero=false,saw_yaw_zero=false;
  for(int i=0;i<40;++i) {
    value=step(-.3,0.,-.5);
    if(old_forward>0.)assert(value.forward_back>=0.);
    if(old_yaw>0.)assert(value.yaw>=0.);
    if(value.forward_back==0.)saw_forward_zero=true;
    if(value.yaw==0.)saw_yaw_zero=true;
    if(value.forward_back<0.)assert(saw_forward_zero);
    if(value.yaw<0.)assert(saw_yaw_zero);
    old_forward=value.forward_back;old_yaw=value.yaw;
  }
  assert(near(value.forward_back,-.3)&&near(value.yaw,-.3));
  value=step(0.,0.,0.);assert(norm(value)==0.&&value.yaw==0.);
  value=step(.3,0.,0.);assert(norm(value)==0.&&value.yaw==0.);
  value=step(.3,0.,0.,10.);assert(near(value.forward_back,.015)); // long gap cannot buy a larger step
  value=step(.3,0.,0.,0.);assert(near(value.forward_back,.015));
  // Same task identity survives curve changes (no curve identifier is stored).
  value=step(.3,0.,0.);assert(near(value.forward_back,.030));
  for(int field=0;field<3;++field) {
    if(field==0)identity.execution_id="other_task";
    else if(field==1)++identity.epoch;
    else ++identity.generation;
    value=step(.3,0.,.5);assert(norm(value)==0.&&value.yaw==0.);
    value=step(.3,0.,.5);assert(near(value.forward_back,.015)&&near(value.yaw,.015));
  }
  // No-command/fault reset uses this exact helper and discards all history.
  slew.reset();value=step(0.,.3,0.);assert(norm(value)==0.);
  value=step(0.,.3,0.);assert(value.forward_back==0.&&near(value.left_right,.015)&&value.yaw==0.);
  value=step(0.,.3,0.,-.01);assert(norm(value)==0.&&value.yaw==0.);
  assert(rejects([&]{slew.apply({0.,.31,0.},.3,0.,0.,mapping,identity,time);}));
  // The real tracker startup demand is .60 * .07 m/s. The 30--40% V2
  // observation must reach all dominant directions without boosting noise.
  auto responsive=mapping;responsive.component_cap=true;
  responsive.planar={.30,.40,.10,.60,.60};responsive.yaw=responsive.planar;
  responsive.validate();
  const auto response=[&](double x,double y,double yaw){
    return mappedSdkFractions(x,y,yaw,"holonomic_xyyaw",responsive,.3,.5);
  };
  const auto startup=response(.042,0.,0.);
  assert(startup.forward_back>=.30&&startup.forward_back<.31);
  const auto diagonal_start=response(.042/std::sqrt(2.),.042/std::sqrt(2.),0.);
  assert(diagonal_start.forward_back>=.30&&near(diagonal_start.forward_back,diagonal_start.left_right));
  for(double direction:{-3.,-2.,-1.,0.,.3,.8,1.5,2.8})for(double demand:{.001,.03,.042,.1,.3,.6}) {
    const auto value=response(demand*std::cos(direction),demand*std::sin(direction),-.5);
    assert(std::max(std::abs(value.forward_back),std::abs(value.left_right))<=.40+1e-12);
    assert(std::abs(value.yaw)<=.40+1e-12);
    assert(near(value.forward_back*std::sin(direction)-value.left_right*std::cos(direction),0.));
  }
  assert(response(.042,1e-7,1e-7).left_right<1e-5);
  assert(response(0.,0.,1e-7).yaw<1e-8);
  SdkFractionSlew responsive_slew;SdkMoveFractions prior;
  for(int i=0;i<=20;++i){
    const auto value=responsive_slew.apply(startup,.042,0.,0.,responsive,identity,10.+i*.05);
    assert(value.forward_back+1e-12>=prior.forward_back);
    assert(value.forward_back-prior.forward_back<=.030+1e-12);
    prior=value;
    if(i==11)assert(prior.forward_back>=.30);
  }
  assert(near(prior.forward_back,startup.forward_back));
  const auto stopped=responsive_slew.apply({},0.,0.,0.,responsive,identity,11.05);
  assert(norm(stopped)==0.&&stopped.yaw==0.);
  // V3 is an explicit engineering input policy, not measured physical speed.
  // Its FIRST nonzero dominant-axis/yaw input begins directly at signed 30%;
  // unlike V1/V2, neither tiny demand nor a fresh identity spends time below
  // that breakaway floor. Minor planar axes preserve the requested direction.
  auto direct=responsive;direct.direct_breakaway=true;direct.validate();
  const auto direct_map=[&](double x,double y,double yaw){
    return mappedSdkFractions(x,y,yaw,"holonomic_xyyaw",direct,.3,.5);
  };
  const auto peak=[](const SdkMoveFractions&v){return std::max(std::abs(v.forward_back),std::abs(v.left_right));};
  assert(sdkDemandFraction(0.,direct.planar,true)==0.);
  for(double demand:{1e-12,.001,.042,.3,.6}) {
    for(double sign:{-1.,1.}) {
      for(int axis=0;axis<3;++axis) {
        const double x=axis==0?sign*demand:0.,y=axis==1?sign*demand:0.,yaw=axis==2?sign*demand:0.;
        SdkFractionSlew initial;
        const auto first=initial.apply(direct_map(x,y,yaw),x,y,yaw,direct,identity,30.);
        assert(near(first.forward_back,axis==0?sign*.30:0.));
        assert(near(first.left_right,axis==1?sign*.30:0.));
        assert(near(first.yaw,axis==2?sign*.30:0.));
      }
    }
  }
  for(double angle:{-3.,-2.,-1.,-.2,.3,.8,1.5,2.8}) {
    const double x=.042*std::cos(angle),y=.042*std::sin(angle);
    SdkFractionSlew initial;const auto first=initial.apply(direct_map(x,y,-.001),x,y,-.001,direct,identity,30.);
    assert(near(peak(first),.30)&&near(first.yaw,-.30));
    assert(near(first.forward_back*y-first.left_right*x,0.));
  }
  SdkFractionSlew direct_slew;SdkMappingIdentity direct_identity{"direct",1,1};double direct_time=40.;
  const auto direct_step=[&](double x,double y,double yaw,double dt=.05){
    direct_time+=dt;return direct_slew.apply(direct_map(x,y,yaw),x,y,yaw,direct,direct_identity,direct_time);
  };
  value=direct_step(.3,0.,.5);assert(near(value.forward_back,.30)&&near(value.yaw,.30));
  for(int i=0;i<10;++i) {
    const auto last=value;value=direct_step(.3,0.,.5);
    assert(value.forward_back>=.30&&value.forward_back<=.40&&value.yaw>=.30&&value.yaw<=.40);
    assert(value.forward_back-last.forward_back<=.030+1e-12&&value.yaw-last.yaw<=.030+1e-12);
  }
  assert(near(value.forward_back,.40)&&near(value.yaw,.40));
  for(int i=0;i<10;++i) {
    const auto last=value;value=direct_step(1e-12,0.,1e-12);
    assert(value.forward_back>=.30&&value.yaw>=.30);
    assert(last.forward_back-value.forward_back<=.030+1e-12&&last.yaw-value.yaw<=.030+1e-12);
  }
  assert(near(value.forward_back,.30)&&near(value.yaw,.30));
  // Every raw zero component is immediately zero, not just a whole HOLD.
  value=direct_step(.3,.3,.5);assert(value.left_right>0.&&value.yaw>0.);
  value=direct_step(.3,0.,.5);assert(value.left_right==0.&&value.forward_back>0.&&value.yaw>0.);
  value=direct_step(0.,.3,0.);assert(value.forward_back==0.&&value.yaw==0.&&near(value.left_right,.30));
  value=direct_step(.3,.3,.5);assert(value.forward_back>0.&&value.left_right>0.&&value.yaw>0.);
  value=direct_step(0.,0.,.5);assert(value.forward_back==0.&&value.left_right==0.&&value.yaw>0.);
  // Full raw zero also ignores a stale nonzero mapped target and clears all
  // history; restart has no legacy ramp through 0--30%.
  value=direct_slew.apply(direct_map(.3,.3,.5),0.,0.,0.,direct,direct_identity,direct_time+.05);
  assert(peak(value)==0.&&value.yaw==0.);
  value=direct_step(-.3,0.,-.5);assert(near(value.forward_back,-.30)&&near(value.yaw,-.30));
  value=direct_step(.3,0.,.5);assert(peak(value)==0.&&value.yaw==0.); // One exact zero before opposite sign.
  value=direct_step(.3,0.,.5);assert(near(value.forward_back,.30)&&near(value.yaw,.30));
  value=direct_step(.3,-.3,.5);assert(value.left_right<0.);
  value=direct_step(.3,.3,.5);assert(peak(value)==0.); // A single planar reversal zeros the planar vector.
  value=direct_step(.3,.3,.5);assert(near(value.forward_back,.30)&&near(value.left_right,.30));
  for(int field=0;field<3;++field) {
    if(field==0)direct_identity.execution_id="new_direct_task";
    else if(field==1)++direct_identity.epoch;
    else ++direct_identity.generation;
    value=direct_step(-.3,0.,-.5);
    assert(near(value.forward_back,-.30)&&value.left_right==0.&&near(value.yaw,-.30));
    value=direct_step(-.3,0.,-.5);assert(near(value.forward_back,-.33)&&near(value.yaw,-.33));
  }
  direct_slew.reset();value=direct_step(0.,-.3,0.);assert(value.forward_back==0.&&near(value.left_right,-.30)&&value.yaw==0.);
  value=direct_step(0.,-.3,0.,10.);assert(near(value.left_right,-.33));
  value=direct_step(0.,-.3,0.,0.);assert(near(value.left_right,-.33));
  value=direct_step(0.,-.3,0.,-.01);assert(peak(value)==0.&&value.yaw==0.); // Clock regression fails zero.
  value=direct_step(0.,-.3,0.);assert(near(value.left_right,-.30));
  assert(rejects([&]{direct_slew.apply({.401,0.,0.},0.,.3,0.,direct,direct_identity,direct_time);}));
  assert(rejects([&]{direct_slew.apply({0.,0.,-.401},0.,0.,-.5,direct,direct_identity,direct_time);}));
  auto mixed=direct;mixed.component_cap=false;assert(rejects([&]{mixed.validate();}));
  // Same-sign continuous XY turning must not bring the dominant SDK input
  // back under the breakaway floor. No reversal or raw zero occurs here.
  SdkFractionSlew turning;
  auto turn=turning.apply(direct_map(.3,.03,0.),.3,.03,0.,direct,direct_identity,60.);
  assert(near(peak(turn),.30));
  for(int i=1;i<=20;++i) {
    turn=turning.apply(direct_map(.03,.3,0.),.03,.3,0.,direct,direct_identity,60.+i*.05);
    assert(peak(turn)>=.30-1e-12&&peak(turn)<=.40+1e-12);
    assert(turn.forward_back>0.&&turn.left_right>0.&&turn.yaw==0.);
  }
  // Optional smooth V3 leaves the original direct V3 fixtures intact. A true
  // start is finite (one second), not persistent low input; curve IDs are not
  // present in the writer's task identity and cannot restart this transition.
  auto smooth=direct;smooth.startup_transition_s=1.;smooth.fraction_acceleration_per_s2=2.;smooth.validate();
  const auto smooth_map=[&](double x,double y,double yaw){return mappedSdkFractions(x,y,yaw,"holonomic_xyyaw",smooth,.3,.5);};
  for(double sign:{-1.,1.}) {
    SdkFractionSlew shaped;SdkMoveFractions previous{};double previous_rate=0.;
    for(int i=0;i<=20;++i) {
      const auto f=shaped.apply(smooth_map(sign*.3,0.,sign*.5),sign*.3,0.,sign*.5,smooth,identity,80.+i*.05);
      const double rate=(f.forward_back-previous.forward_back)/.05;
      assert(std::abs(rate)<=.6+1e-12);
      if(i>0)assert(std::abs(rate-previous_rate)/.05<=2.+1e-9);
      assert(f.forward_back*sign>=0.&&std::abs(f.forward_back)<=.30+1e-12);
      assert(f.left_right==0.&&near(f.forward_back,f.yaw));previous=f;previous_rate=rate;
      if(i==0)assert(f.forward_back==0.&&f.yaw==0.);
    }
    assert(near(std::abs(previous.forward_back),.30));
    for(int i=21;i<=120;++i) {
      const auto f=shaped.apply(smooth_map(sign*.3,0.,sign*.5),sign*.3,0.,sign*.5,smooth,identity,80.+i*.05);
      const double rate=(f.forward_back-previous.forward_back)/.05;
      assert(std::abs(rate)<=.6+1e-12&&std::abs(rate-previous_rate)/.05<=2.+1e-8);
      assert(std::abs(f.forward_back)>=.30-1e-12&&std::abs(f.forward_back)<=.40+1e-12);
      previous=f;previous_rate=rate;
    }
    assert(std::abs(previous.forward_back)>.3999);
    const auto yaw_zero=shaped.apply(smooth_map(sign*.3,0.,0.),sign*.3,0.,0.,smooth,identity,86.05);
    assert(yaw_zero.yaw==0.&&std::abs(yaw_zero.forward_back)>=.30);
    const auto reverse_zero=shaped.apply(smooth_map(-sign*.3,0.,-.5),-sign*.3,0.,-.5,smooth,identity,86.10);
    assert(reverse_zero.forward_back==0.);
    const auto reversed=shaped.apply(smooth_map(-sign*.3,0.,-.5),-sign*.3,0.,-.5,smooth,identity,86.15);
    assert(reversed.forward_back*sign<=0.&&std::abs(reversed.forward_back)<.01);
    const auto safety_zero=shaped.apply(smooth_map(sign*.3,0.,.5),0.,0.,0.,smooth,identity,86.20);
    assert(peak(safety_zero)==0.&&safety_zero.yaw==0.);
    auto new_identity=identity;++new_identity.generation;
    const auto fresh=shaped.apply(smooth_map(sign*.3,0.,sign*.5),sign*.3,0.,sign*.5,smooth,new_identity,86.25);
    assert(peak(fresh)==0.&&fresh.yaw==0.);
  }
  // Same elapsed input time, not number of calls, determines startup shape.
  SdkFractionSlew s20,s40;
  for(int i=0;i<=20;++i) {
    const auto a=s20.apply(smooth_map(.3,0.,.5),.3,0.,.5,smooth,identity,90.+i*.05);
    auto b=s40.apply(smooth_map(.3,0.,.5),.3,0.,.5,smooth,identity,90.+i*.05);
    assert(near(a.forward_back,b.forward_back));
    if(i<20)s40.apply(smooth_map(.3,0.,.5),.3,0.,.5,smooth,identity,90.+i*.05+.025);
  }
  auto invalid_smooth=smooth;invalid_smooth.startup_transition_s=.1;
  assert(rejects([&]{invalid_smooth.validate();}));
  invalid_smooth=smooth;invalid_smooth.direct_breakaway=false;
  assert(rejects([&]{invalid_smooth.validate();}));
  // Only an explicit checked nonzero terminal command requests a finite
  // normal S-down. The bit cannot override raw zero, reversal, or identity.
  SdkFractionSlew terminal;SdkMoveFractions last;
  for(int i=0;i<=120;++i)last=terminal.apply(smooth_map(.3,0.,0.),.3,0.,0.,smooth,identity,100.+i*.05);
  assert(last.forward_back>.3999);double previous_stop_rate=0.;
  for(int i=1;i<=25;++i) {
    const auto value=terminal.apply(smooth_map(.001,0.,0.),.001,0.,0.,smooth,identity,106.+i*.05,true);
    const double rate=(value.forward_back-last.forward_back)/.05;
    assert(value.forward_back>=0.&&value.forward_back<=last.forward_back+1e-12);
    assert(std::abs(rate)<=.6+1e-12&&std::abs(rate-previous_stop_rate)/.05<=2.+1e-8);
    last=value;previous_stop_rate=rate;
  }
  assert(near(last.forward_back,0.));
  // A normal nonzero restart is S-shaped, not a resurrection of old amplitude.
  last=terminal.apply(smooth_map(.3,0.,0.),.3,0.,0.,smooth,identity,107.30,false);
  assert(last.forward_back<.01);
  last=terminal.apply(smooth_map(.3,0.,0.),0.,0.,0.,smooth,identity,107.35,true);
  assert(peak(last)==0.&&last.yaw==0.);
  SdkFractionSlew replanned_goal;
  for(int i=0;i<=120;++i)last=replanned_goal.apply(smooth_map(.3,0.,0.),.3,0.,0.,smooth,identity,110.+i*.05);
  for(int i=1;i<=15;++i)last=replanned_goal.apply(smooth_map(.001,0.,0.),.001,0.,0.,smooth,identity,116.+i*.05,true);
  assert(last.forward_back>0.&&last.forward_back<.30);const double partial=last.forward_back;
  last=replanned_goal.apply(smooth_map(.3,0.,0.),.3,0.,0.,smooth,identity,116.80,false);
  assert(last.forward_back>=0.&&std::abs(last.forward_back-partial)<=.6*.05+1e-12); // Carry the release rate, no jump to the floor.
  // Normal steering release reaches zero without repeated 30-percent yaw
  // pulses while the same task's translation continues. No curve ID resets
  // this writer history, and the bit does not change exact-zero precedence.
  for(double sign:{-1.,1.}) {
    SdkFractionSlew coast;
    for(int i=0;i<=120;++i)last=coast.apply(smooth_map(.3,0.,sign*.5),.3,0.,sign*.5,
      smooth,identity,120.+i*.05);
    double prior_rate=0.;
    for(int i=1;i<=25;++i) {
      const auto next=coast.apply(smooth_map(.3,0.,sign*.001),.3,0.,sign*.001,
        smooth,identity,126.+i*.05,false,true);
      const double rate=(next.yaw-last.yaw)/.05;
      assert(next.yaw*sign>=0.&&std::abs(next.yaw)<=std::abs(last.yaw)+1e-12);
      assert(std::abs(rate)<=.6+1e-12&&std::abs(rate-prior_rate)/.05<=2.+1e-8);
      assert(next.forward_back>=.30);last=next;prior_rate=rate;
    }
    assert(last.yaw==0.);
    last=coast.apply(smooth_map(.3,0.,sign*.5),.3,0.,sign*.5,
      smooth,identity,127.30,false,false);
    assert(std::abs(last.yaw)<.01&&last.forward_back>=.30);
    last=coast.apply(smooth_map(.3,0.,sign*.5),0.,0.,0.,
      smooth,identity,127.35,false,true);
    assert(last.yaw==0.&&peak(last)==0.);
    SdkFractionSlew interrupted;
    for(int i=0;i<=120;++i)last=interrupted.apply(smooth_map(.3,0.,sign*.5),.3,0.,sign*.5,
      smooth,identity,130.+i*.05);
    for(int i=1;i<=15;++i)last=interrupted.apply(smooth_map(.3,0.,sign*.001),.3,0.,sign*.001,
      smooth,identity,136.+i*.05,false,true);
    const double old_yaw=std::abs(last.yaw);assert(old_yaw>0.&&old_yaw<.30);
    last=interrupted.apply(smooth_map(.3,0.,sign*.5),.3,0.,sign*.5,
      smooth,identity,136.80,false,false);
    assert(last.yaw*sign>=0.&&std::abs(std::abs(last.yaw)-old_yaw)<=.6*.05+1e-12);
    last=interrupted.apply(smooth_map(.3,0.,0.),.3,0.,0.,
      smooth,identity,136.85,false,true);
    assert(last.yaw==0.&&last.forward_back>=.30);
  }
  // Interrupt the rising startup itself, and resume while the release is
  // still moving. Preserve the SDK input's actual rate at BOTH transitions.
  // Resetting that derivative formerly produced an 11.24/s^2 peak at .5 s.
  for(double sign:{-1.,1.})for(unsigned release_at:{4U,10U,16U,24U}) {
    for(unsigned resume_after:{0U,2U,7U}) {
      SdkFractionSlew transition;SdkMoveFractions previous{};double previous_rate=0.;
      bool reached_release_zero=false;
      for(unsigned i=0;i<=release_at+60U;++i) {
        const bool release=i>release_at&&(resume_after==0U||i<=release_at+resume_after);
        const double request=sign*(release?.001:.5);
        const auto next=transition.apply(smooth_map(.3,0.,request),.3,0.,request,
          smooth,identity,150.+i*.05,false,release);
        const double rate=(next.yaw-previous.yaw)/.05;
        assert(next.yaw*sign>=0.&&std::abs(next.yaw)<=.40+1e-12);
        assert(std::abs(rate)<=.60+1e-9);
        if(i>0)assert(std::abs(rate-previous_rate)/.05<=2.+1e-8);
        if(release&&next.yaw==0.)reached_release_zero=true;
        previous=next;previous_rate=rate;
      }
      if(resume_after==0U)assert(reached_release_zero&&previous.yaw==0.);
      else assert(std::abs(previous.yaw)>=.30);
    }
  }
  // The same amplitude/rate mechanism governs normal goal deceleration;
  // a nearby goal during startup or a renewed approach cannot reset its rate.
  for(double sign:{-1.,1.})for(unsigned release_at:{4U,10U,16U,24U}) {
    for(unsigned resume_after:{0U,2U,7U}) {
      SdkFractionSlew transition;SdkMoveFractions previous{};double previous_rate=0.;
      bool reached_release_zero=false;
      for(unsigned i=0;i<=release_at+60U;++i) {
        const bool release=i>release_at&&(resume_after==0U||i<=release_at+resume_after);
        const double request=sign*(release?.001:.3);
        const auto next=transition.apply(smooth_map(request,0.,0.),request,0.,0.,
          smooth,identity,160.+i*.05,release,false);
        const double rate=(next.forward_back-previous.forward_back)/.05;
        assert(next.forward_back*sign>=0.&&std::abs(next.forward_back)<=.40+1e-12);
        assert(std::abs(rate)<=.60+1e-9);
        if(i>0)assert(std::abs(rate-previous_rate)/.05<=2.+1e-8);
        if(release&&next.forward_back==0.)reached_release_zero=true;
        previous=next;previous_rate=rate;
      }
      if(resume_after==0U)assert(reached_release_zero&&previous.forward_back==0.);
      else assert(std::abs(previous.forward_back)>=.30);
    }
  }
  responsive.yaw.maximum_fraction=.41;assert(rejects([&]{responsive.validate();}));
  // V4 is a static current-command response, not the old 1 s startup,
  // amplitude PD or a robot inertia estimate. It uses the same near-zero
  // blend deliberately: tiny tracking yaw must not jump to +/-30 percent.
  auto continuous=mapping;continuous.component_cap=true;continuous.continuous_command_ratio=true;
  continuous.planar={.30,.40,.10,.60,.60};continuous.yaw=continuous.planar;continuous.validate();
  auto fifty=continuous;fifty.planar.maximum_fraction=.50;fifty.yaw={.20,.30,.20,.60,.60};fifty.validate();
  const auto fifty_value=mappedSdkFractions(.3,0.,.5,"holonomic_xyyaw",fifty,.3,.5);
  assert(near(fifty_value.forward_back,.50)&&fifty_value.left_right==0.&&near(fifty_value.yaw,.30));
  const auto fifty_zero=mappedSdkFractions(0.,0.,0.,"holonomic_xyyaw",fifty,.3,.5);
  assert(peak(fifty_zero)==0.&&fifty_zero.yaw==0.);
  auto invalid_fifty=fifty;invalid_fifty.planar.maximum_fraction=.500001;
  assert(rejects([&]{invalid_fifty.validate();}));
  invalid_fifty=fifty;invalid_fifty.yaw.maximum_fraction=.400001;
  assert(rejects([&]{invalid_fifty.validate();}));
  invalid_fifty=fifty;invalid_fifty.continuous_command_ratio=false;
  assert(rejects([&]{invalid_fifty.validate();}));
  const auto continuous_map=[&](double x,double y,double yaw){
    return mappedSdkFractions(x,y,yaw,"holonomic_xyyaw",continuous,.3,.5);
  };
  SdkFractionSlew current_command;SdkMappingIdentity current_identity{"continuous-task",1,1};
  const auto current=[&](double x,double y,double yaw,double at,bool goal=false,bool coast=false){
    const auto target=continuous_map(x,y,yaw);
    const auto result=current_command.apply(target,x,y,yaw,continuous,current_identity,at,goal,coast);
    assert(near(result.forward_back,target.forward_back)&&near(result.left_right,target.left_right)&&near(result.yaw,target.yaw));
    return result;
  };
  value=current(.042,0.,.25,200.);
  assert(value.forward_back>=.30&&value.forward_back<.31&&value.yaw>.30&&value.yaw<.40);
  assert(continuous_map(.3,0.,1e-7).yaw>0.&&continuous_map(.3,0.,1e-7).yaw<1e-8);
  // Each 20 Hz sample follows the current direction and fraction immediately;
  // sign changes do not insert an invented neutral Move command.
  for(unsigned i=1;i<=80;++i) {
    const double angle=-1.+2.*i/80.,x=.12*std::cos(angle),y=.12*std::sin(angle);
    value=current(x,y,(i%2?1.:-1.)*.001,200.+i*.05);
    assert(near(value.forward_back*y-value.left_right*x,0.));
    assert(value.yaw*(i%2?1.:-1.)>0.&&std::abs(value.yaw)<.001);
  }
  current(.3,.1,.5,204.1);
  value=current(0.,-.3,0.,204.15);
  assert(value.forward_back==0.&&near(value.left_right,-.40)&&value.yaw==0.);
  // Decreasing real demands continue below the response floor. Repeated
  // selected terminal commands do not reach zero on a hidden release timer
  // and then start another ramp. Resume is likewise the current ratio only.
  for(double request:{.03,.02,.01,.005,.001}) {
    value=current(request,0.,-.001,204.2+(.03-request),true,true);
    assert(value.forward_back>0.&&value.forward_back<=.30&&value.yaw<0.);
  }
  const auto held=current(.005,0.,-.001,204.3,true,true);
  value=current(.005,0.,-.001,204.35,true,true);assert(near(value.forward_back,held.forward_back));
  value=current(.008,0.,-.002,204.4);assert(value.forward_back>held.forward_back&&value.forward_back<.30);
  value=current(.3,0.,.5,204.45);assert(near(value.forward_back,.40)&&near(value.yaw,.40));
  value=current_command.apply(continuous_map(.3,.3,.5),0.,0.,0.,continuous,current_identity,204.5,true,true);
  assert(peak(value)==0.&&value.yaw==0.); // Whole raw zero outranks any cached target/reason.
  value=current(.042,0.,.001,204.55);assert(value.forward_back>=.30&&std::abs(value.yaw)<.001);
  current_command.reset();value=current(.005,0.,0.,204.6);assert(value.forward_back>0.&&value.forward_back<.30);
  for(int field=0;field<3;++field) {
    if(field==0)current_identity.execution_id="new-continuous-task";
    else if(field==1)++current_identity.epoch;
    else ++current_identity.generation;
    value=current(-.042,.021,-.001,204.7+field*.05);
    assert(value.forward_back<-.30&&value.left_right>0.&&value.yaw<0.);
  }
  value=current_command.apply(continuous_map(.3,0.,.5),.3,0.,.5,continuous,current_identity,204.79);
  assert(peak(value)==0.&&value.yaw==0.); // Retain the existing backwards-clock zero.
  value=current(.042,0.,.001,204.9);assert(value.forward_back>=.30);
  auto mixed_continuous=continuous;mixed_continuous.direct_breakaway=true;
  assert(rejects([&]{mixed_continuous.validate();}));
  mixed_continuous=continuous;mixed_continuous.component_cap=false;
  assert(rejects([&]{mixed_continuous.validate();}));
  mixed_continuous=continuous;mixed_continuous.startup_transition_s=1.;mixed_continuous.fraction_acceleration_per_s2=2.;
  assert(rejects([&]{mixed_continuous.validate();}));
  assert(rejects([&]{current_command.apply({0.,.401,0.},.3,0.,0.,continuous,current_identity,205.);}));
  assert(rejects([&]{continuous_map(std::numeric_limits<double>::quiet_NaN(),0.,0.);}));
  assert(rejects([&]{mappedSdkFractions(-.1,0.,0.,"forward_yaw",continuous,.3,.5);}));
  // A smooth input must not acquire the old max-normalization derivative
  // cusp when its dominant axis changes. Test actual production mapping,
  // not an independently simplified curve or a physical plant assumption.
  const double diagonal_angle=std::acos(-1.)/4.,epsilon_angle=1e-6;
  const auto circle=[&](double angle){return continuous_map(.2*std::cos(angle),.2*std::sin(angle),0.);};
  for(double crossing:{diagonal_angle,3.*diagonal_angle,5.*diagonal_angle,7.*diagonal_angle}) {
    const auto l=circle(crossing-epsilon_angle),c=circle(crossing),r=circle(crossing+epsilon_angle);
    const double jump=std::hypot((r.forward_back-c.forward_back)-(c.forward_back-l.forward_back),
      (r.left_right-c.left_right)-(c.left_right-l.left_right))/epsilon_angle;
    assert(jump<2e-5); // Old actual max mapping gives ~1.04 here.
  }
  // The scalar shape is C2 at zero, the response blend and saturation. Signed
  // tiny yaw passes continuously through zero rather than acquiring a floor.
  const double difference=1e-7;
  for(double boundary:{0.,continuous.planar.blend_demand_ratio,1.}) {
    const auto f=[&](double r){return r<0.?0.:sdkDemandFractionV4(r,continuous.planar);};
    const double dl=(f(boundary)-f(boundary-difference))/difference;
    const double dr=(f(boundary+difference)-f(boundary))/difference;
    assert(std::abs(dl)<1e-5&&std::abs(dr)<1e-5&&std::abs(dl-dr)<1e-5);
    const double ddl=(f(boundary)-2.*f(boundary-difference)+f(boundary-2.*difference))/(difference*difference);
    const double ddr=(f(boundary+2.*difference)-2.*f(boundary+difference)+f(boundary))/(difference*difference);
    assert(std::abs(ddl-ddr)<.25); // Finite-difference round-off at saturation.
  }
  auto observed_continuous=continuous;
  observed_continuous.planar.effective_fraction=.30*std::pow(2.,1./16.);
  observed_continuous.yaw.effective_fraction=observed_continuous.planar.effective_fraction;
  observed_continuous.validate();
  const double start_norm=.3*observed_continuous.planar.blend_demand_ratio;
  const auto observed_diagonal=mappedSdkFractions(start_norm/std::sqrt(2.),start_norm/std::sqrt(2.),0.,
    "holonomic_xyyaw",observed_continuous,.3,.5);
  assert(near(observed_diagonal.forward_back,.30)&&near(observed_diagonal.left_right,.30));
  const auto observed_axial=mappedSdkFractions(start_norm,0.,0.,"holonomic_xyyaw",observed_continuous,.3,.5);
  assert(near(observed_axial.forward_back,.31328213472822414));
  for(unsigned i=0;i<=720;++i)for(double demand:{1e-9,.005,.015,.03,.042,.1,.2,.3,.6}) {
    const double angle=i*std::acos(-1.)/360.,x=demand*std::cos(angle),y=demand*std::sin(angle);
    const auto v=mappedSdkFractions(x,y,-.001,"holonomic_xyyaw",observed_continuous,.3,.5);
    assert(std::max(std::abs(v.forward_back),std::abs(v.left_right))<=.40+1e-12);
    assert(std::abs(v.yaw)<=.40&&near(v.forward_back*y-v.left_right*x,0.,1e-10));
    assert(v.forward_back*x>=0.&&v.left_right*y>=0.&&v.yaw<0.);
  }
  std::cout<<"SDK engineering V1/V2/V3 legacy and V4 current-command ratio, zero, cap, direction and identity assertions passed\n";
}
