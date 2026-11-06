#include "execution_transport_core.hpp"
#include "test_execution_fixture.hpp"
#include "sdk_input_mapping.hpp"
#include "sdk_write_transactions.hpp"
#include "execution_scheduler.hpp"
#include "execution_robot_health.hpp"
#include <cassert>
#include <functional>
#include <iostream>
#include <limits>
#include <future>
#include <chrono>
#include <thread>
#include <vector>

using namespace d1monitor::execution3;
static Version taskVersion(){Version v;v.schema_version=3;v.session_id="nav";v.task_id="task";
  v.route_id="route";v.route_hash=std::string(64,'a');v.map_version_id="map";
  v.localization_epoch=1;v.localization_seed_id="seed";return v;}
static Health healthy(){return {true,true,true,true,false,true};}
static builtin_interfaces::msg::Time afterNs(const builtin_interfaces::msg::Time&source,int64_t delta){
  const auto ns=sourceNs(source)+delta;builtin_interfaces::msg::Time out;
  out.sec=static_cast<int32_t>(ns/1000000000LL);out.nanosec=static_cast<uint32_t>(ns%1000000000LL);return out;}
static Grant::Request request(double at=10.){Grant::Request r;r.version=taskVersion();r.sdk_session="sdk";
  r.execution_id="execute";r.control_epoch=1;r.confirmation_id="confirmed";r.request_id="grant";
  r.source_stamp=stamp(at);r.activate=true;r.transport_mode="isolated_mock";return r;}
static TaskLease lease(double at=10.,uint64_t seq=1){TaskLease p;p.schema_version=1;p.version=taskVersion();p.execution_id="execute";
  p.control_epoch=1;p.confirmation_id="confirmed";p.sdk_session="sdk";p.sdk_arm_generation=1;
  p.sequence=seq;p.source_stamp=stamp(at);p.valid_until=afterNs(p.source_stamp,350000000LL);p.allowed=true;
  p.phase="tracking";p.transport_mode="isolated_mock";return p;}
static LocalCommand command(double at=10.,uint64_t seq=1,uint64_t lease_seq=1){LocalCommand c;c.schema_version=1;
  c.version=taskVersion();c.execution_id="execute";c.control_epoch=1;c.sdk_session="sdk";c.sdk_arm_generation=1;
  c.lease_sequence=lease_seq;c.sequence=seq;c.source_stamp=c.body_source_stamp=stamp(at);
  c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);c.body_frame="d1max_loc_base_link";
  c.velocity.linear.x=.2;c.transport_mode="isolated_mock";c.trajectory_id=1;return c;}
static Transport writer(bool planar_stop=false,double at=10.){Transport t("sdk","isolated_mock");t.configureTaskFollower(true);
  t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");
  auto policy=testExecutionPolicy();if(planar_stop)policy.velocity_metric="planar_xy_with_vertical_guard_v1";
  t.configureExecutionPolicy(policy,.8);t.configureBrakingModel(std::string(64,'c'),.3,.5,"holonomic_xyyaw");
  t.setSDKTransactions(0,false);assert(t.grant(request(at),at,healthy()).accepted);return t;}
static void ready(Transport&t,double at=10.,uint64_t seq=1){assert(t.taskLease(lease(at,seq),at));assert(t.localCommand(command(at,seq,seq),at));}
static bool moving(const std::optional<geometry_msgs::msg::Twist>&v){return v&&planarMotionNonzero(*v);}
static bool sdkMoving(const SdkMoveFractions&v){return static_cast<float>(v.forward_back)!=0.f||
  static_cast<float>(v.left_right)!=0.f||static_cast<float>(v.yaw)!=0.f;}
// Reproduce the live SDK sequence: actual motion, acknowledged terminal zero,
// one full post-zero stationary interval, then small 3D MC drift. The drift is
// recorded input, not a relaxed threshold or a new "still stationary" claim.
static Transport terminalWriter(bool drain=true,bool zero_ack=true,bool first_stop=true){
  auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));
  t.moveAttempted(true);t.writeCalled(10.,true,true);t.stop("goal_reached",10.05);
  if(zero_ack)t.stopWritten(10.05);
  if(!drain)t.setSDKTransactions(1,false);
  for(unsigned i=1;i<=140;++i){const double at=10.05+i*.02;
    const double planar=i>70||!first_stop?.021325:.020497;
    const double vertical=i>70||!first_stop?.023331:.009689;
    t.mc(i*20000000ULL,at,at,std::hypot(planar,vertical),.008,at,"mock-clock",planar,vertical,false);
    t.tick(at,healthy());}
  return t;
}
static Grant::Request nextRequest(double at=12.85){auto next=request(at);
  next.version.task_id="next-task";next.execution_id="next-execution";
  next.request_id="next-grant";next.control_epoch=2;return next;}

struct DeliverySweep {
  unsigned nonzero=0,zero_edges=0,expired=0,accepted=0;
  double minimum_spacing=std::numeric_limits<double>::infinity(),maximum_spacing=0.,maximum_body_age=0.,maximum_source_age=0.;
};
// Drive the PRODUCTION scheduler and Transport at every input, watchdog and
// ORIGINAL command deadline, not only at producer receipts. The environment
// below supplies clock/callback events; it does not replace admission or copy
// any task, freshness, SDK mapping or scheduler decision into a toy transport.
static DeliverySweep sweepDelivery(double phase,double body_age,const std::vector<double>&delays,
    double lease_delay=0.,bool callbacks_first=true,unsigned samples=160,
    double command_horizon=.1,unsigned skip_every=0,
    const std::vector<double>&wake_lateness={0.},double admission_delay=0.){
  struct Event{double at;bool owner;uint64_t seq;};
  std::vector<Event> events;events.reserve(samples+samples/2+2);
  const auto end=10.+phase+samples*.05-.001;
  for(uint64_t i=0;i<=samples/2+1;++i)events.push_back({10.+i*.1+lease_delay,true,i+1});
  for(uint64_t i=0;i<samples;++i)if(!skip_every||(i+1)%skip_every)
    events.push_back({10.+phase+.001+i*.05+delays[i%delays.size()],false,i+1});
  std::stable_sort(events.begin(),events.end(),[callbacks_first](const Event&a,const Event&b){
    if(a.at!=b.at)return a.at<b.at;
    return callbacks_first?a.owner>b.owner:a.owner<b.owner;});
  auto t=writer();d1monitor::CommandWriterDeadline schedule(10.,.05);
  DeliverySweep result;uint64_t revision=0;size_t next_event=0;double now=10.,last_nonzero=-1.;
  bool moving_before=false,primed=false;unsigned iterations=0;
  while(now<=end){
    assert(++iterations<100000); // a repeated expired hint must not spin forever
    schedule.observe(revision);
    auto deadline=t.taskCommandDeadline();
    if(deadline)*deadline=std::nextafter(*deadline,std::numeric_limits<double>::infinity());
    const double wake=schedule.next(deadline);
    const double input=next_event<events.size()?events[next_event].at:std::numeric_limits<double>::infinity();
    now=std::max(now,std::min(wake,input))+wake_lateness[(iterations-1)%wake_lateness.size()];if(now>end)break;
    // Both orderings matter when a deadline and callback share a clock value.
    const bool consume_inputs=callbacks_first||input<wake;
    const auto deliver=[&]{while(next_event<events.size()&&events[next_event].at<=now){
      const auto event=events[next_event++];
      if(event.owner){const auto before=t.lastDemand().sequence;
        assert(t.taskLease(lease(10.+(event.seq-1)*.1,event.seq),event.at));
        if(t.lastDemand().sequence!=before)++revision;
      }else{const auto generated=10.+phase+.001+(event.seq-1)*.05;
        auto c=command(generated,event.seq,(event.seq-1)/2+1);
        // This sender scheduling sweep models immediate controller receipt
        // and echo. Delayed/missing feedback is tested separately below.
        if(t.taskCommandFeedback())c.acknowledged_discontinuity_write=
          t.taskCommandFeedback()->discontinuity_write_sequence;
        c.body_source_stamp=afterNs(c.source_stamp,-static_cast<int64_t>(std::llround(body_age*1e9)));
        c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
        c.valid_until=earlier(c.valid_until,afterNs(c.source_stamp,
          static_cast<int64_t>(std::llround(command_horizon*1e9))));
        if(t.localCommand(c,event.at)){++revision;++result.accepted;}
      }
    }};
    if(consume_inputs)deliver();
    schedule.observe(revision);
    const bool urgent_only=!schedule.normalDue(now),no_fresh_command=!schedule.pending();
    const auto started=now;
    // State/input callbacks are independently scheduled while the writer is
    // late or preparing its vendor call. Select the latest original sample,
    // exactly as execution_tick does after taking the state lock.
    now+=admission_delay;
    if(consume_inputs)deliver();
    const auto selected_revision=revision;
    auto output=t.tick(now,healthy());
    if((urgent_only||no_fresh_command)&&moving(output))output.reset();
    std::optional<double> submitted;
    if(output){const bool positive=moving(output);
      if(positive){
        const auto timing=t.selectedTaskCommandTiming();assert(timing.present);
        result.maximum_body_age=std::max(result.maximum_body_age,now-secs(timing.body_source));
        result.maximum_source_age=std::max(result.maximum_source_age,now-secs(timing.source));
        if(last_nonzero>=0){result.minimum_spacing=std::min(result.minimum_spacing,now-last_nonzero);
          result.maximum_spacing=std::max(result.maximum_spacing,now-last_nonzero);}
        last_nonzero=now;submitted=now;++result.nonzero;primed=true;
      }else if(primed&&moving_before)++result.zero_edges;
      moving_before=positive;t.moveAttempted(positive);t.writeCalled(now,true,positive);
    }
    schedule.complete(started,now,selected_revision,!urgent_only&&!no_fresh_command,submitted);
    if(!consume_inputs)deliver();
  }
  result.expired=t.taskCommandExpiry().count;assert(!t.stopping());return result;
}

int main(){unsigned cases=0;
  const auto run=[&](const char*name,const std::function<void()>&test){test();++cases;std::cout<<"PASS "<<name<<'\n';};
  run("actual_forced_zero_sticky_feedback_is_observation_not_resume_authority",[]{
    auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true,true);
    assert(t.taskCommandFeedback()&&t.taskCommandFeedback()->write_sequence==1);
    assert(t.taskCommandFeedback()->command_sequence==1&&t.taskCommandFeedback()->discontinuity_write_sequence==0);
    assert(!moving(t.tick(10.101,healthy())));t.writeCalled(10.101,true,false);
    auto f=*t.taskCommandFeedback();assert(f.discontinuity_write_sequence==2&&!f.command_selected);
    assert(f.interrupted_command_sequence==1&&f.interrupted_command_source_stamp==stamp(10.));
    assert(f.interrupted_command_valid_until==command().valid_until);
    assert(!moving(t.tick(10.15,healthy())));t.writeCalled(10.15,true,false);
    assert(t.taskCommandFeedback()->discontinuity_write_sequence==2); // One gap, not each zero tick.
    assert(t.taskLease(lease(10.16,2),10.16));auto c=command(10.16,2,2);
    assert(c.acknowledged_discontinuity_write==0); // Feedback lost; fresh Native command remains authoritative.
    assert(t.localCommand(c,10.16));
    assert(moving(t.tick(10.16,healthy())));t.writeCalled(10.16,true,true);
    f=*t.taskCommandFeedback();assert(f.discontinuity_write_sequence==2&&f.command_sequence==2);
    assert(f.command_source_stamp==c.source_stamp&&f.command_valid_until==c.valid_until);
    assert(!moving(t.tick(10.261,healthy())));t.writeCalled(10.261,true,false);
    assert(t.taskCommandFeedback()->discontinuity_write_sequence==5);
  });
  run("feedback_reports_no_discontinuity_for_initial_or_explicit_zero_or_unsent_zero",[]{
    auto t=writer();assert(!moving(t.tick(10.,healthy())));t.writeCalled(10.,true,false);
    assert(t.taskCommandFeedback()->discontinuity_write_sequence==0);
    ready(t,10.01);assert(moving(t.tick(10.01,healthy())));t.writeCalled(10.01,true,true);
    auto zero=command(10.02,2);zero.velocity={};zero.hold=true;
    assert(t.localCommand(zero,10.02));assert(!moving(t.tick(10.02,healthy())));t.writeCalled(10.02,true,false);
    assert(t.taskCommandFeedback()->command_selected&&t.taskCommandFeedback()->discontinuity_write_sequence==0);
    auto failed=writer();ready(failed);assert(moving(failed.tick(10.,healthy())));failed.writeCalled(10.,true,true);
    assert(!moving(failed.tick(10.101,healthy())));failed.writeCalled(10.101,false,false);
    assert(!failed.taskCommandFeedback()->write_submitted&&failed.taskCommandFeedback()->discontinuity_write_sequence==0);
  });
  run("inflight_zero_capture_keeps_original_identity_without_vetoing_new_native_command",[]{
    auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true,true);
    const auto zero=t.tick(10.101,healthy());assert(zero&&!moving(zero));
    const auto capture=t.captureTaskWrite(*zero,10.101,SdkMoveFractions{});
    auto next=command(10.102,2);assert(t.localCommand(next,10.102)); // Arrived while Move is unlocked.
    t.writeCalled(10.103,true,false,capture);
    const auto f=*t.taskCommandFeedback();assert(!f.command_selected&&f.command_sequence==0);
    assert(f.source_stamp==stamp(10.101)&&f.discontinuity_write_sequence==2);
    assert(moving(t.tick(10.104,healthy()))); // Fresh original evidence, not ACK awareness, selects it.
    assert(t.taskCommandFeedback()->write_sequence==2); // tick alone is not a send fact.
    next=command(10.105,3);next.acknowledged_discontinuity_write=2;
    assert(t.localCommand(next,10.105));assert(moving(t.tick(10.105,healthy())));
  });
  run("proof_only_refresh_cannot_change_writer_acknowledgement",[]{
    auto t=writer();assert(t.taskLease(lease(),10.));auto c=command();
    c.valid_until=afterNs(c.source_stamp,40000000LL);assert(t.localCommand(c,10.));
    auto refresh=c;refresh.valid_until=afterNs(c.source_stamp,80000000LL);
    refresh.acknowledged_discontinuity_write=1;assert(!t.localCommand(refresh,10.01));
    assert(t.taskCommandAdmission().reason=="evidence_update_changed_intent");
    refresh.acknowledged_discontinuity_write=0;assert(t.localCommand(refresh,10.01));
    assert(moving(t.tick(10.05,healthy())));const auto timing=t.selectedTaskCommandTiming();
    assert(timing.source==c.source_stamp&&timing.body_source==c.body_source_stamp);
  });
  run("writer_persistent_wake_and_admission_lateness_does_not_accumulate_phase",[]{
    for(const auto&late:std::vector<std::vector<double>>{{.0002},{.0003},{.002},{.0002,.001,.002,.0005}}){
      const auto r=sweepDelivery(0.,.022,{.001},0.,true,1000,.1,0,late,.0002);
      std::cout<<"persistent writer jitter: max_source_age_ms="<<r.maximum_source_age*1000.
        <<", min_interval_ms="<<r.minimum_spacing*1000.<<", max_interval_ms="<<r.maximum_spacing*1000.
        <<", sent="<<r.nonzero<<", expiries="<<r.expired<<'\n';
      assert(r.accepted==1000&&r.nonzero==1000&&r.zero_edges==0&&r.expired==0);
      assert(r.maximum_source_age<.005&&r.minimum_spacing>.048&&r.maximum_spacing<.054);
    }
  });
  run("same_original_input_fixed_poll_phase_expires_but_fresh_command_delivery_does_not",[]{
    auto before=writer(),after=writer();
    assert(before.taskLease(lease(),10.)&&after.taskLease(lease(),10.));
    d1monitor::WriterDeadline old_schedule(10.,.05);
    d1monitor::CommandWriterDeadline schedule(10.,.05);
    // Both writers first perform the identical empty-input health check.
    assert(!moving(before.tick(10.,healthy()))&&!moving(after.tick(10.,healthy())));
    old_schedule.complete(10.,10.0001);schedule.complete(10.,10.0001,0,true);
    // Production typed LocalCommand retains a 100 ms original body lease.
    // A valid command arrives 1 ms AFTER the old writer's poll. Its paired
    // actual body sample was acquired 60 ms before command generation.
    auto c=command(10.001);c.body_source_stamp=afterNs(c.source_stamp,-60000000LL);
    c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
    const auto receipt=10.002;
    assert(before.localCommand(c,receipt)&&after.localCommand(c,receipt));
    assert(std::abs(*after.taskCommandDeadline()-10.041)<1e-8);
    schedule.observe(c.sequence);assert(schedule.next()<=receipt&&schedule.normalDue(receipt));
    assert(moving(after.tick(receipt,healthy())));schedule.complete(receipt,receipt+.0001,c.sequence,true,receipt);
    assert(after.selectedTaskCommandTiming().source==c.source_stamp&&
      after.selectedTaskCommandTiming().body_source==c.body_source_stamp&&
      after.selectedTaskCommandTiming().valid_until==c.valid_until);
    assert(!moving(before.tick(old_schedule.next(),healthy())));
    assert(before.taskCommandExpiry().count==1&&after.taskCommandExpiry().count==0);
    // Removing producer input still expires at the exact ORIGINAL bound.
    assert(!moving(after.tick(10.041001,healthy()))&&after.taskCommandExpiry().count==1);
    assert(!after.taskCommandDeadline()&&!after.stopping());
  });
  run("command_driven_sender_preserves_20Hz_spacing_without_a_second_poll_phase",[]{
    auto t=writer();d1monitor::CommandWriterDeadline schedule(10.,.05);
    double last_attempt=-1.;unsigned sends=0;
    for(uint64_t i=1;i<=40;++i){
      const auto generated=10.001+static_cast<double>(i-1)*.05,receipt=generated+.001;
      assert(t.taskLease(lease(generated-.0005,i),receipt));
      auto c=command(generated,i,i);c.body_source_stamp=afterNs(c.source_stamp,-60000000LL);
      c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
      assert(t.localCommand(c,receipt));schedule.observe(i);
      auto attempted=std::max(receipt,last_attempt<0?receipt:last_attempt+.05);
      assert(schedule.normalDue(attempted)&&moving(t.tick(attempted,healthy())));
      const auto selected=t.selectedTaskCommandTiming();assert(selected.sequence==i&&selected.source==c.source_stamp&&
        selected.body_source==c.body_source_stamp&&selected.valid_until==c.valid_until);
      assert(last_attempt<0||attempted-last_attempt>=.05-1e-10);
      schedule.complete(attempted,attempted+.0001,i,true,attempted);
      t.writeCalled(attempted,true,true);last_attempt=attempted;++sends;
    }
    assert(sends==40&&t.taskCommandExpiry().count==0);
    assert(!moving(t.tick(last_attempt+.041,healthy()))&&t.taskCommandExpiry().count==1);
  });
  run("complete_writer_event_watchdog_deadline_sweep_has_no_avoidable_zero",[]{
    unsigned streams=0,minimum_sends=160;double maximum_spacing=0.,maximum_body_age=0.;
    for(double phase:{0.,.001,.009,.024,.039,.049})for(bool callbacks_first:{false,true})
      for(const auto& scenario:std::vector<std::pair<double,std::vector<double>>>{
          {.004,{.001,.005,.013,.035,.045}}, {.020,{.001,.005,.013,.022}},
          {.040,{.001,.003,.005}}}){
        const auto result=sweepDelivery(phase,scenario.first,scenario.second,0.,callbacks_first);
        assert(result.accepted==160&&result.nonzero>100&&result.zero_edges==0&&result.expired==0);
        // Nominal 20 Hz permits phase jitter, but a delayed send may never
        // be followed by a sub-half-period catch-up call.
        assert(result.minimum_spacing>=.025-1e-10&&result.maximum_body_age<=.100);
        minimum_sends=std::min(minimum_sends,result.nonzero);
        maximum_spacing=std::max(maximum_spacing,result.maximum_spacing);
        maximum_body_age=std::max(maximum_body_age,result.maximum_body_age);
        ++streams;
      }
    // Native may receive an owner lease before SDK receives that independent
    // publication. The actual one-slot defer + lease callback admission wakes
    // the same latest writer; it must not add an extra 50 ms send phase.
    for(bool order:{false,true}){const auto result=sweepDelivery(.001,.004,{.001},.030,order);
      assert(result.accepted<160&&result.nonzero>100&&result.zero_edges==0&&result.expired==0);++streams;}
    std::cout<<"complete writer scheduling sweep: "<<streams<<" feasible streams, no zero edges; minimum_nonzero_sends="
      <<minimum_sends<<"/160, maximum_send_interval_ms="<<maximum_spacing*1000.
      <<", maximum_original_body_age_ms="<<maximum_body_age*1000.<<'\n';
  });
  run("complete_writer_deadline_sweep_exposes_infeasible_source_budget",[]{
    // E10's receipt-only positive test omitted these real deadline wakes.
    // With 60 ms body age + a 50 ms producer period, every previous ORIGINAL
    // 100 ms deadline ends before the next source arrives. Event delivery can
    // remove avoidable polling latency, but cannot honestly erase that gap.
    const auto result=sweepDelivery(0.,.060,{.001},0.,true,40);
    assert(result.accepted==40&&result.nonzero==40&&result.zero_edges>=39&&result.expired>=39);
    assert(result.minimum_spacing>=.05-1e-10&&result.maximum_body_age<=.100);
    std::cout<<"infeasible source budget: "<<result.zero_edges<<" true deadline zero edges / "
      <<result.expired<<" original expiries; TTL unchanged\n";
  });
  run("recorded_short_lease_and_missing_input_produce_independent_zero_edges",[]{
    // Latest real run: body is only ~22 ms old, but Native sometimes binds a
    // command to a geometry deadline ~42 ms after its source. A 50 ms producer
    // cannot bridge that interval even with an immediate event-driven writer.
    const auto complete=sweepDelivery(0.,.022,{.001},0.,true,80);
    const auto short_lease=sweepDelivery(0.,.022,{.001},0.,true,80,.042);
    // An omitted intermediate command instead leaves the last 100 ms body
    // deadline to expire at command source +78 ms. These are distinct causes.
    const auto missing=sweepDelivery(0.,.022,{.001},0.,true,80,.1,8);
    assert(complete.accepted==80&&complete.zero_edges==0&&complete.expired==0);
    assert(short_lease.accepted==80&&short_lease.zero_edges>=79&&short_lease.expired>=79);
    assert(missing.accepted==70&&missing.zero_edges>=9&&missing.expired>=9);
    std::cout<<"recorded timing regimes: complete_expiries="<<complete.expired
      <<", 42ms_lease_expiries="<<short_lease.expired<<", omitted_input_expiries="<<missing.expired<<'\n';
  });
  run("new_original_command_replaces_short_lease_before_next_send_slot",[]{
    auto t=writer();d1monitor::CommandWriterDeadline schedule(10.,.05);
    assert(t.taskLease(lease(),10.));auto c=command();c.valid_until=stamp(10.042);
    assert(t.localCommand(c,10.001));schedule.observe(1);
    assert(moving(t.tick(10.001,healthy())));schedule.complete(10.001,10.001,1,true,10.001);
    auto newer=command(10.04,2);newer.valid_until=stamp(10.082);
    assert(t.localCommand(newer,10.041));schedule.observe(2);
    assert(!schedule.normalDue(10.042)); // Wait for the normal slot, not a zero.
    assert(moving(t.tick(10.042,healthy()))&&t.taskCommandExpiry().count==0);
    assert(schedule.normalDue(10.051)&&moving(t.tick(10.051,healthy())));
    assert(t.selectedTaskCommandTiming().sequence==2&&t.taskCommandExpiry().count==0);
  });
  run("blocked_vendor_and_long_idle_keep_only_latest_without_replaying_or_resetting_phase",[]{
    auto t=writer();ready(t);d1monitor::CommandWriterDeadline schedule(10.,.05);
    schedule.observe(1);assert(moving(t.tick(10.,healthy())));
    // The first call is blocked outside state. Its real ROS input callback
    // accepts a later command while the vendor has not returned.
    assert(t.localCommand(command(10.06,2),10.061));schedule.observe(2);
    schedule.complete(10.,10.12,1,true,10.);
    assert(schedule.pending()&&!schedule.normalDue(10.12));
    assert(schedule.normalDue(10.150001)&&moving(t.tick(10.150001,healthy())));
    assert(t.selectedTaskCommandTiming().sequence==2);
    schedule.complete(10.150001,10.150001,2,true,10.150001);
    assert(!schedule.pending());
    // Source expiry still submits zero immediately, not at the next phase.
    assert(!moving(t.tick(10.160001,healthy()))&&t.taskCommandExpiry().count==1);
    schedule.complete(10.160001,10.160001,2,false);
    assert(!t.taskCommandDeadline()&&!schedule.pending());
    // Long idle does not enqueue old opportunities. A genuinely new granted
    // writer identity and fresh input can use the idle opportunity once.
    auto replacement=writer(false,20.001);ready(replacement,20.001);
    schedule.observe(100);assert(schedule.normalDue(20.001));
    assert(moving(replacement.tick(20.001,healthy())));
    schedule.complete(20.001,20.002,100,true,20.001);
    assert(!schedule.pending()&&!schedule.normalDue(20.025));
    schedule.observe(101);assert(!schedule.normalDue(20.025)&&schedule.normalDue(20.051));
  });
  run("admission_diagnostics_preserve_rejection_and_capture_expiry_context",[]{
    auto t=writer();ready(t);auto a=t.taskCommandAdmission();
    assert(a.attempts==1&&a.accepted==1&&a.rejected==0&&std::string(a.reason)=="accepted");
    auto late=command(10.02,2);assert(!t.localCommand(late,10.121));
    assert(std::string(t.taskCommandAdmission().reason)=="original_time_contract");
    assert(!moving(t.tick(10.122,healthy())));
    const auto expiry=t.taskCommandExpiry();assert(expiry.sequence==1&&expiry.admission_sequence==2&&
      expiry.admission_at==10.121&&std::string(expiry.admission_reason)=="original_time_contract");
    auto fresh=command(10.13,3);assert(t.localCommand(fresh,10.131));
    auto foreign=command(10.14,4);foreign.version.task_id="other";
    assert(!t.localCommand(foreign,10.141));assert(std::string(t.taskCommandAdmission().reason)=="task_identity_or_retired");
    assert(moving(t.tick(10.142,healthy()))&&t.selectedTaskCommandTiming().sequence==3);
    assert(t.taskCommandExpiry().admission_sequence==2); // Immutable expiry fact.
    auto waiting=command(10.15,5,2);assert(!t.localCommand(waiting,10.151));
    assert(std::string(t.taskCommandAdmission().reason)=="lease_deferred");
    assert(t.taskLease(lease(10.15,2),10.152));
    a=t.taskCommandAdmission();assert(a.attempts==6&&a.accepted==3&&a.rejected==2&&a.deferred==1);
    assert(std::string(a.reason)=="accepted"&&moving(t.tick(10.153,healthy())));
  });
  run("new_event_does_not_bypass_expiry_revoke_estop_or_foreign_task",[]{
    for(int kind=0;kind<4;++kind){auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));
      auto c=command(10.02,2);
      if(kind==0){assert(!t.localCommand(c,10.121));assert(!moving(t.tick(10.122,healthy())));}
      if(kind==1){auto p=lease(10.01,2);p.revoked=true;assert(t.taskLease(p,10.01));
        assert(!t.localCommand(c,10.02)&&!moving(t.tick(10.021,healthy())));}
      if(kind==2){t.softwareEstopRequested(10.01);assert(!t.localCommand(c,10.02));
        assert(!moving(t.tick(10.021,healthy())));}
      if(kind==3){c.version.task_id="foreign";assert(!t.localCommand(c,10.02));
        assert(t.lastDemand().sequence==1&&t.lastDemand().version.task_id=="task");}
    }
  });
  run("wheel_stop_keeps_raw3d_but_checks_xy_and_vertical_separately",[]{
    for(bool enabled:{false,true}){auto t=writer(enabled);ready(t);assert(moving(t.tick(10.,healthy())));
      t.writeCalled(10.,true);auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));t.stopWritten(10.05);
      const double xy=std::hypot(.028366,.001744),z=.013489,raw=std::hypot(xy,z);
      for(unsigned i=1;i<=90;++i){const double at=10.05+i*.02;
        t.mc(i*20000000ULL,at,at,raw,.005,at,"mock-clock",xy,z,true);}
      const auto report=t.report(10.05+90*.02);assert(raw>.03&&report.measured_linear_mps==raw);
      assert(report.measured_planar_mps==xy&&report.measured_vertical_mps==z);
      assert(report.measured_stop_confirmed==enabled&&!report.physical_acceptance_verified);
      assert(report.velocity_metric==(enabled?"planar_xy_with_vertical_guard_v1":"spatial_norm_3d"));}
  });
  run("wheel_stop_preserves_vertical_planar_angular_mode_and_zero_ack_guards",[]{
    for(int invalid=0;invalid<6;++invalid){auto t=writer(true);ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true);
      auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));if(invalid!=4)t.stopWritten(10.05);
      const double xy=invalid==0?.031:.0284,z=invalid==1?.031:.0135,angular=invalid==2?.051:.005;
      for(unsigned i=1;i<=90;++i){const double at=10.05+i*.02;
        t.mc(i*20000000ULL,at,at,std::hypot(xy,z)+(invalid==5?.01:0.),angular,at,"mock-clock",xy,z,invalid!=3);}
      assert(!t.report(10.05+90*.02).measured_stop_confirmed);}
  });
  run("fixed_stair_explicit_runtime_reuse_uses_same_bounded_stop_metric",[]{
    // Reproduce the 21:22 capture. Neither MC component breaches its configured
    // limit, but silently replacing this profile with a 3D sphere does.
    for(int invalid=0;invalid<9;++invalid){auto t=writer(true);ready(t);
      assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true);
      auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));
      if(invalid!=7)t.stopWritten(10.05);
      auto mode=healthy();mode.general=false;mode.required_motion_mode="stair";
      mode.motion_mode="stair";mode.motion_mode_confirmed=true;
      if(invalid==4)mode.robot_state_fresh=false;
      if(invalid==5)mode.motion_mode_confirmed=false;
      const double xy=invalid==1?.031:.0217,z=invalid==2?.031:.0287,w=invalid==3?.051:.007;
      for(unsigned i=1;i<=90;++i){const double at=10.05+i*.02;
        t.mc(i*20000000ULL,at,at,std::hypot(xy,z)+(invalid==8?.01:0.),w,at,"mock-clock",xy,z,
          stationaryRuntimeProfileApplies(mode,invalid!=6));}
      const auto report=t.report(11.85);
      assert(report.measured_stop_confirmed==(invalid==0));
      if(invalid==0){assert(report.measured_linear_mps>.03&&report.measured_planar_mps<.03&&report.measured_vertical_mps<.03);
        assert(report.velocity_metric=="planar_xy_with_vertical_guard_v1"&&report.stationary_duration_sec>=1.);
        assert(t.completeManualHandoff(11.85));}
      else assert(!t.completeManualHandoff(11.85));
    }
  });
  run("wheel_stop_mode_change_restarts_window_terminal_report_does_not_renew_live_mc",[]{
    auto t=writer(true);ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true);
    auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));t.stopWritten(10.05);
    for(unsigned i=1;i<=70;++i){const double at=10.05+i*.02;
      t.mc(i*20000000ULL,at,at,.01,.005,at,"mock-clock",.01,0.,i>60);}
    assert(!t.currentStopReport(10.05+70*.02).measured_stop_confirmed);
    for(unsigned i=71;i<=130;++i){const double at=10.05+i*.02;
      t.mc(i*20000000ULL,at,at,.01,.005,at,"mock-clock",.01,0.,true);}
    const auto terminal=t.report(10.05+130*.02);assert(terminal.measured_stop_confirmed);
    assert(t.report(13.65)==terminal);assert(!t.currentStopReport(13.65).measured_stop_confirmed);
    assert(!t.mcSourceFresh(13.65,"mock-clock"));
  });
  run("terminal_stop_first_mc_window_survives_noise_without_report_polling",[]{
    auto t=terminalWriter();const auto terminal=t.report(12.85);
    assert(terminal.measured_stop_confirmed&&terminal.execution_id=="execute");
    assert(terminal.stationary_duration_sec>=1.&&terminal.measured_linear_mps<.03);
    assert(secs(terminal.source_stamp)<11.45); // original first proven window
    assert(!t.currentStopReport(12.85).measured_stop_confirmed);
    assert(t.currentStopReport(12.85).measured_linear_mps>.03);
    assert(t.report(30.)==terminal); // no sequence, timestamp, or value refresh
    assert(!t.mcSourceFresh(30.,"mock-clock")&&!t.stationaryEvidence(30.).usable);
    assert(t.state(12.85).phase=="stopped"&&!t.state(12.85).grant_ready);
  });
  run("terminal_stop_allows_fresh_next_task_without_inheriting_stop_or_commands",[]{
    auto t=terminalWriter();const auto old=t.report(12.85);const auto next=nextRequest();
    assert(t.grant(next,12.85,healthy()).accepted&&t.armGeneration()==2);
    assert(!t.report(12.85).measured_stop_confirmed&&t.report(12.85).execution_id=="next-execution");
    assert(t.report(12.85).source_stamp!=old.source_stamp);
    assert(!t.taskLease(lease(12.85,20),12.85)&&!t.localCommand(command(12.85,20,20),12.85));
    auto p=lease(12.85);p.version=next.version;p.execution_id=next.execution_id;p.control_epoch=2;p.sdk_arm_generation=2;
    auto c=command(12.85);c.version=next.version;c.execution_id=next.execution_id;c.control_epoch=2;c.sdk_arm_generation=2;
    assert(t.taskLease(p,12.85)&&t.localCommand(c,12.85)&&moving(t.tick(12.85,healthy())));
    t.moveAttempted(true);t.writeCalled(12.85,true,true);t.stop("next_goal",12.86);t.stopWritten(12.87);
    assert(!t.report(12.87).measured_stop_confirmed&&!t.completeManualHandoff(12.87));
  });
  run("terminal_stop_allows_manual_takeover_after_old_goal_noise",[]{
    auto t=terminalWriter();const auto old=t.report(12.85);assert(old.measured_stop_confirmed);
    assert(t.completeManualHandoff(12.85)&&t.executionId().empty());
    assert(!t.taskLease(lease(12.86,20),12.86)&&!t.localCommand(command(12.86,20,20),12.86));
    auto posture=healthy();posture.general=false;posture.motion_mode="manual_posture";
    assert(!t.tick(12.90,posture)&&!t.state(12.90).fault_latched);
    assert(t.grant(nextRequest(12.95),12.95,healthy()).accepted&&t.armGeneration()==2);
    assert(!t.report(12.95).measured_stop_confirmed);
  });
  run("terminal_stop_needs_real_window_zero_ack_and_retired_vendor_calls",[]{
    for(int invalid=0;invalid<3;++invalid){
      auto t=terminalWriter(invalid!=0,invalid!=1,invalid!=2);
      if(invalid==0)t.setSDKTransactions(0,false); // old window is gone, cannot backdate it
      assert(!t.report(12.85).measured_stop_confirmed);
      assert(!t.grant(nextRequest(),12.85,healthy()).accepted&&!t.completeManualHandoff(12.85));
    }
  });
  run("terminal_stop_does_not_bypass_current_health_or_uncertain_vendor_calls",[]{
    for(int invalid=0;invalid<7;++invalid){auto t=terminalWriter();auto h=healthy();
      const auto old=t.report(12.85);
      if(invalid==0)t.setSDKTransactions(1,false);
      if(invalid==1)t.setSDKTransactions(0,true);
      if(invalid==2)h.mc_fresh=false;
      if(invalid==3)h.robot_state_fresh=false;
      if(invalid==4)h.connected=false;
      if(invalid==5)t.softwareEstopRequested(12.85);
      if(invalid==6)t.fault("control_lost_no_auto_retake",12.85);
      assert(!t.grant(nextRequest(),12.85,h).accepted);
      assert(t.report(12.85)==old); // retained fact is not a current motion authorization
      if(invalid<2||invalid>4)assert(!t.completeManualHandoff(12.85));
    }
  });
  run("unexpected_nonzero_attempt_invalidates_terminal_certificate_even_if_not_acknowledged",[]{
    for(bool use_attempt:{false,true}){auto t=terminalWriter();assert(t.report(12.85).measured_stop_confirmed);
      if(use_attempt)t.moveAttempted(true);else t.writeCalled(12.85,true,true);
      assert(!t.report(12.85).measured_stop_confirmed&&!t.report(12.85).stop_submitted);
      assert(!t.grant(nextRequest(),12.85,healthy()).accepted&&!t.completeManualHandoff(12.85));
      assert(!moving(t.tick(12.85,healthy())));t.stopWritten(12.86);
      assert(!t.report(12.86).measured_stop_confirmed); // fresh zero is not enough
    }
  });
  run("zero_awaiting_lease_fences_delayed_pre_zero_nonzero_without_poisoning_sequence",[]{
    auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true,true);
    auto zero=command(10.08,3,2);zero.velocity={};zero.hold=true;
    assert(!t.localCommand(zero,10.081));
    // This packet is newer than the accepted floor, but its original source
    // precedes the already observed zero. Its older known lease cannot revive
    // pre-zero motion merely because the zero's own lease is still in transit.
    auto delayed=command(10.075,2,1);assert(!t.localCommand(delayed,10.082));
    assert(!moving(t.tick(10.083,healthy())));
    assert(t.taskLease(lease(10.07,2),10.09));assert(!moving(t.tick(10.091,healthy())));
    // Unknown/deferred sequence 3 never poisoned the accepted sequence floor;
    // a genuinely later source with its real causal lease resumes normally.
    auto later=command(10.095,4,2);assert(t.localCommand(later,10.096));
    assert(moving(t.tick(10.097,healthy()))&&t.lastDemand().source_stamp==later.source_stamp);
    assert(!moving(t.tick(10.196,healthy())));
  });
  run("unknown_zero_sequence_cannot_poison_later_genuine_positive",[]{
    auto t=writer();ready(t);auto zero=command(10.08,std::numeric_limits<uint64_t>::max(),2);
    zero.velocity={};zero.hold=true;assert(!t.localCommand(zero,10.081));
    auto delayed=command(10.075,99,4);assert(!t.localCommand(delayed,10.082));
    auto later=command(10.09,2,1);assert(t.localCommand(later,10.091));
    assert(moving(t.tick(10.092,healthy()))&&t.lastDemand().sequence==2);
    assert(!t.taskCommandReorderStats().pending);
  });
  run("zero_stops_older_current_motion_while_preserving_a_genuinely_later_pending_source",[]{
    auto t=writer();ready(t);auto later=command(10.09,4,4);assert(!t.localCommand(later,10.091));
    auto zero=command(10.085,3,3);zero.velocity={};zero.hold=true;
    assert(!t.localCommand(zero,10.092));assert(!moving(t.tick(10.093,healthy())));
    auto delayed=command(10.08,2,1);assert(!t.localCommand(delayed,10.094));
    assert(t.taskLease(lease(10.084,3),10.095));assert(!moving(t.tick(10.096,healthy())));
    assert(t.taskLease(lease(10.088,4),10.097));assert(moving(t.tick(10.098,healthy())));
    assert(t.lastDemand().sequence==4&&t.lastDemand().source_stamp==later.source_stamp);
  });
  run("future_stale_foreign_and_invalid_zero_cannot_create_a_source_barrier",[]{
    for(int kind=0;kind<4;++kind){auto t=writer();ready(t);auto bad=command(10.08,3,2);
      bad.velocity={};bad.hold=true;
      if(kind==0)bad=command(10.20,3,2);
      if(kind==1)bad=command(9.90,3,2);
      if(kind==2)bad.sdk_session="foreign";
      if(kind==3)bad.body_valid_until=afterNs(bad.body_source_stamp,100000001LL);
      bad.velocity={};bad.hold=true;assert(!t.localCommand(bad,10.081));
      auto real=command(10.075,2,1);assert(t.localCommand(real,10.082));
      assert(moving(t.tick(10.083,healthy()))&&t.lastDemand().sequence==2);
    }
  });
  run("command_before_its_causal_lease_survives_cross_topic_reordering_without_restamp",[]{
    auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));t.writeCalled(10.,true,true);
    auto later=command(10.075,2,2);assert(!t.localCommand(later,10.076));
    // Native already received lease 2 before producing this command, while
    // SDK receives the same original lease slightly later on another topic.
    assert(t.taskLease(lease(10.07,2),10.08));
    // Old command is now expired. The new command's ORIGINAL body deadline
    // is 10.175, so middleware reordering alone must not insert an SDK zero.
    assert(moving(t.tick(10.11,healthy())));
    assert(t.lastDemand().source_stamp==later.source_stamp&&t.lastDemand().body_source_stamp==later.body_source_stamp);
    assert(!moving(t.tick(10.176,healthy()))&&!t.stopping());
  });
  run("first_command_can_wait_for_first_real_positive_lease_but_not_move_before_it",[]{
    auto t=writer();auto c=command();assert(!t.localCommand(c,10.001));
    assert(t.taskCommandReorderStats().pending&&!moving(t.tick(10.002,healthy())));
    assert(t.taskLease(lease(),10.003));assert(moving(t.tick(10.004,healthy())));
    const auto stats=t.taskCommandReorderStats();assert(stats.deferred==1&&stats.completed==1&&!stats.pending);
  });
  run("missing_lease_never_advances_accepted_command_or_sequence_floor",[]{
    auto t=writer();ready(t);auto unknown=command(10.02,std::numeric_limits<uint64_t>::max(),999);
    assert(!t.localCommand(unknown,10.021)&&t.taskCommandReorderStats().pending);
    assert(t.lastDemand().sequence==1);auto real=command(10.03,2,1);
    assert(t.localCommand(real,10.031));assert(!t.taskCommandReorderStats().pending);
    assert(moving(t.tick(10.032,healthy()))&&t.lastDemand().sequence==2);
    // A missing older/history-evicted lease is not a future reorder fact.
    assert(t.taskLease(lease(10.04,2),10.04));auto old=command(10.05,3,0);
    assert(!t.localCommand(old,10.051)&&!t.taskCommandReorderStats().pending);
  });
  run("deferred_command_is_one_latest_source_not_a_replay_queue",[]{
    auto t=writer();ready(t);auto a=command(10.02,2,2),b=command(10.03,3,3);
    assert(!t.localCommand(a,10.021));assert(!t.localCommand(b,10.031));
    assert(t.taskCommandReorderStats().deferred==2&&t.taskCommandReorderStats().pending);
    assert(t.taskLease(lease(10.01,2),10.04));assert(t.lastDemand().sequence==1);
    assert(t.taskLease(lease(10.025,3),10.05));assert(moving(t.tick(10.051,healthy())));
    assert(t.lastDemand().sequence==3&&t.lastDemand().source_stamp==b.source_stamp);
    assert(t.taskCommandReorderStats().completed==1&&!t.taskCommandReorderStats().pending);
  });
  run("late_or_noncausal_positive_lease_does_not_refresh_deferred_original_deadline",[]{
    for(bool late:{false,true}){auto t=writer();ready(t);auto c=command(10.02,2,2);
      assert(!t.localCommand(c,10.021));
      const auto at=late?10.13:10.04;const auto owner_source=late?10.01:10.03;
      assert(t.taskLease(lease(owner_source,2),at));assert(!t.taskCommandReorderStats().pending);
      assert(t.lastDemand().sequence==1);assert(!moving(t.tick(late?10.131:10.101,healthy())));
      assert(t.taskCommandReorderStats().completed==0&&!t.stopping());
    }
  });
  run("new_zero_for_a_missing_lease_clears_old_nonzero_and_pending_immediately",[]{
    auto t=writer();ready(t);auto c=command(10.02,2,2);assert(!t.localCommand(c,10.021));
    auto zero=command(10.03,3,3);zero.velocity={};zero.hold=true;zero.reason="operator_hold";
    assert(!t.localCommand(zero,10.031));assert(!moving(t.tick(10.032,healthy())));
    assert(t.taskLease(lease(10.01,2),10.04));assert(!moving(t.tick(10.041,healthy())));
    assert(t.taskLease(lease(10.025,3),10.05));assert(!moving(t.tick(10.051,healthy())));
    assert(t.lastDemand().sequence==3&&t.lastDemand().hold);
  });
  run("owner_hold_revoke_mc_gap_and_new_task_purge_deferred_nonzero",[]{
    for(int barrier=0;barrier<4;++barrier){auto t=writer();ready(t);auto c=command(10.02,2,2);
      assert(!t.localCommand(c,10.021)&&t.taskCommandReorderStats().pending);
      if(barrier==0){auto hold=lease(10.03,3);hold.allowed=false;hold.phase="holding";assert(t.taskLease(hold,10.03));}
      if(barrier==1){auto revoke=lease(10.03,3);revoke.revoked=true;assert(t.taskLease(revoke,10.03));}
      if(barrier==2){auto h=healthy();h.mc_fresh=false;assert(!moving(t.tick(10.03,h)));}
      if(barrier==3){t.stop("replace",10.03);t.setSDKTransactions(0,false);
        auto next=request(10.04);next.execution_id="next";next.request_id="next";next.control_epoch=2;
        next.version.task_id="next";assert(t.grant(next,10.04,healthy()).accepted);}
      assert(!t.taskCommandReorderStats().pending);
      t.taskLease(lease(10.01,2),10.05);assert(!moving(t.tick(10.051,healthy())));
      assert(!t.localCommand(c,10.052));
    }
  });
  run("invalid_or_foreign_zero_does_not_erase_a_valid_deferred_command",[]{
    for(int kind=0;kind<4;++kind){auto t=writer();ready(t);auto c=command(10.02,2,2);
      assert(!t.localCommand(c,10.021));auto bad=command(10.03,3,3);bad.velocity={};bad.hold=true;
      if(kind==0)bad.sdk_session="foreign";if(kind==1)bad.version.task_id="foreign";
      if(kind==2)bad.body_valid_until=afterNs(bad.body_source_stamp,100000001LL);
      if(kind==3)bad.velocity.linear.x=std::numeric_limits<double>::quiet_NaN();
      assert(!t.localCommand(bad,10.031)&&t.taskCommandReorderStats().pending);
      assert(t.taskLease(lease(10.01,2),10.04));assert(moving(t.tick(10.041,healthy())));
      assert(t.lastDemand().sequence==2);
    }
  });
  run("selected_command_timing_preserves_original_wire_stamps_and_selection",[]{
    auto t=writer();assert(!t.selectedTaskCommandTiming().present);assert(t.taskLease(lease(),10.));
    auto c=command();c.body_source_stamp=afterNs(c.source_stamp,-4000000LL);
    c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
    assert(t.localCommand(c,10.001));assert(!t.selectedTaskCommandTiming().present);
    assert(moving(t.tick(10.002,healthy())));const auto first=t.selectedTaskCommandTiming();
    assert(first.present&&first.sequence==c.sequence&&first.source==c.source_stamp&&
      first.body_source==c.body_source_stamp&&first.valid_until==c.valid_until);
    assert(std::abs(1000.*(10.002-secs(first.source))-2.)<1e-6);
    assert(std::abs(1000.*(10.002-secs(first.body_source))-6.)<1e-6);
    assert(std::abs(1000.*(secs(first.valid_until)-10.002)-94.)<1e-6);
    auto duplicate=command(10.003,1);assert(!t.localCommand(duplicate,10.003));
    auto older=command(10.004,0);assert(!t.localCommand(older,10.004));
    auto selected=t.selectedTaskCommandTiming();assert(selected.sequence==first.sequence&&selected.source==first.source);
    auto newer=command(10.01,2);assert(t.localCommand(newer,10.01));
    // Receipt is not selection: provenance stays with the immutable attempted
    // command until the next tick, rather than drifting to the newest inbox.
    selected=t.selectedTaskCommandTiming();assert(selected.sequence==first.sequence&&selected.source==first.source);
    assert(moving(t.tick(10.011,healthy())));selected=t.selectedTaskCommandTiming();
    assert(selected.present&&selected.sequence==2&&selected.source==newer.source_stamp);
    t.writeCalled(10.011,true,true);assert(!t.selectedTaskCommandTiming().present);
    auto zero=command(10.02,3);zero.velocity={};zero.hold=true;zero.reason="goal_position_settling";
    assert(t.localCommand(zero,10.02));assert(!moving(t.tick(10.021,healthy())));
    selected=t.selectedTaskCommandTiming();assert(selected.present&&selected.sequence==3&&selected.source==zero.source_stamp);
    assert(!moving(t.tick(10.121,healthy()))&&!t.selectedTaskCommandTiming().present);
    auto revoke=lease(10.122,2);revoke.revoked=true;assert(t.taskLease(revoke,10.122));
    assert(!moving(t.tick(10.123,healthy()))&&!t.selectedTaskCommandTiming().present);
  });
  run("expiry_diagnostic_retains_original_deadlines_but_cannot_replay_velocity",[]{
    auto t=writer();assert(t.taskCommandExpiry().count==0);
    assert(t.taskLease(lease(),10.));
    auto c=command();c.body_source_stamp=afterNs(c.source_stamp,-4000000LL);
    c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
    assert(t.localCommand(c,10.001)&&moving(t.tick(10.002,healthy())));
    assert(t.taskCommandExpiry().count==0);
    assert(!moving(t.tick(10.100,healthy())));
    const auto gap=t.taskCommandExpiry();
    assert(gap.count==1&&gap.sequence==c.sequence&&gap.execution_id==t.executionId());
    assert(gap.source==c.source_stamp&&gap.body_source==c.body_source_stamp&&
      gap.body_valid_until==c.body_valid_until&&gap.valid_until==c.valid_until);
    assert(gap.observed_at==10.100&&secs(gap.valid_until)<gap.observed_at);
    for(double now:{10.11,10.12,10.13})assert(!moving(t.tick(now,healthy())));
    assert(t.taskCommandExpiry().count==1&&!t.selectedTaskCommandTiming().present);
    auto fresh_command=command(10.14,2);
    assert(t.localCommand(fresh_command,10.14)&&moving(t.tick(10.141,healthy())));
    assert(t.taskCommandExpiry().count==1); // Read-only history, not a current input.
  });
  run("vendor_inline_ack_block_does_not_block_revoke_or_retire_old_execution",[]{
    auto t=writer();ready(t);std::mutex state;d1monitor::SDKWriteTransactions ledger;
    std::promise<void>inside,release;auto released=release.get_future().share();
    bool revoked=false,new_grant=false,retired=false;unsigned submitted_nonzero=0;
    std::thread sender([&]{std::unique_lock<std::mutex>lock(state);
      const auto selected=t.tick(10.,healthy());assert(moving(selected));
      const auto execution=t.executionId();const auto epoch=t.epoch(),arm=t.armGeneration();
      auto token=ledger.beginCall();d1monitor::SDKWriteTransactions::CallReturn frame(token);
      t.moveAttempted(true);
      const auto result=d1monitor::invokeSdkOutsideState(lock,[&]{
        ++submitted_nonzero;token->complete();inside.set_value();released.wait();return std::error_code{};});
      if(execution==t.executionId()&&epoch==t.epoch()&&arm==t.armGeneration())t.writeCalled(10.02,!result,true);
    });
    inside.get_future().wait();
    auto cancel=std::async(std::launch::async,[&]{std::lock_guard<std::mutex>lock(state);
      revoked=t.taskLease(lease(10.01,2),10.01); // A newer ordinary lease remains accepted.
      auto stop=lease(10.015,3);stop.revoked=true;revoked=revoked&&t.taskLease(stop,10.015);
      t.setSDKTransactions(ledger.pending(),false);retired=t.retirement(10.015).software_retired;
      auto next=request(10.016);next.execution_id="next";next.request_id="next";next.control_epoch=2;
      next.version.task_id="next";new_grant=t.grant(next,10.016,healthy()).accepted;
      return ledger.pending();});
    const bool responsive=cancel.wait_for(std::chrono::milliseconds(200))==std::future_status::ready;
    release.set_value();sender.join();const auto pending=cancel.get();
    assert(responsive&&pending==1&&revoked&&!new_grant&&!retired&&ledger.pending()==0);
    assert(submitted_nonzero==1&&t.stopping()&&!t.report(10.02).measured_stop_confirmed);
    assert(!t.localCommand(command(10.025,2,3),10.025));
    t.setSDKTransactions(ledger.pending(),false);assert(!moving(t.tick(10.025,healthy())));
    assert(t.executionId()=="execute");
  });
  run("after_slow_takeover_speed_is_selected_from_latest_sources_not_old_selection",[]{
    auto t=writer();ready(t);std::mutex state;d1monitor::SDKWriteTransactions ledger;
    std::unique_lock<std::mutex>lock(state);const auto old=t.tick(10.,healthy());assert(moving(old));
    auto token=ledger.beginCall();{
      d1monitor::SDKWriteTransactions::CallReturn frame(token);
      const auto result=d1monitor::invokeSdkOutsideState(lock,[&]{token->complete();return std::error_code{};});
      assert(!result&&ledger.pending()==1);
      // Same command after a 120 ms vendor delay is expired at its original
      // body acquisition deadline. Vendor return never restamps it as fresh.
      assert(!moving(t.tick(10.12,healthy()))&&!t.stopping());
    }assert(ledger.pending()==0);
  });
  run("inline_zero_ack_cannot_admit_next_task_before_vendor_stack_returns",[]{
    auto t=writer();ready(t);auto c=command(10.001,2);c.velocity={};c.hold=true;
    assert(t.localCommand(c,10.001));const auto zero=t.tick(10.001,healthy());assert(zero&&!moving(zero));
    d1monitor::SDKWriteTransactions ledger;auto token=ledger.beginCall();{
      d1monitor::SDKWriteTransactions::CallReturn frame(token);t.moveAttempted(false);
      assert(token->complete()); // Inline successful zero ACK, still inside SDK.
      auto revoke=lease(10.002,2);revoke.revoked=true;assert(t.taskLease(revoke,10.002));
      t.setSDKTransactions(ledger.pending(),false);assert(!t.noMotionRetired());
      auto next=request(10.003);next.execution_id="next";next.request_id="next";next.control_epoch=2;
      next.version.task_id="next";const auto rejected=t.grant(next,10.003,healthy());
      assert(!rejected.accepted&&rejected.reason=="prior_sdk_transactions_unretired");
      t.writeCalled(10.004,true,false);
    }
    t.setSDKTransactions(ledger.pending(),false);assert(t.noMotionRetired());
    auto next=request(10.005);next.execution_id="next";next.request_id="next";next.control_epoch=2;
    next.version.task_id="next";assert(t.grant(next,10.005,healthy()).accepted);
  });
  run("old_vendor_error_is_session_uncertainty_consumed_before_next_grant",[]{
    auto t=writer();ready(t);d1monitor::SDKWriteTransactions ledger;
    std::atomic<bool>write_failed{false};auto token=ledger.beginCall();token->returned();
    auto revoke=lease(10.001,2);revoke.revoked=true;assert(t.taskLease(revoke,10.001));
    assert(token->finish([&]{write_failed.store(true);}));assert(ledger.pending()==0);
    // Exact service ordering: synchronize actual ledger, consume uncertainty,
    // then consider the next grant. Error cannot leak across a successful grant.
    t.setSDKTransactions(ledger.pending(),false);if(write_failed.exchange(false))t.submissionFailed(10.002);
    auto next=request(10.003);next.execution_id="next";next.request_id="next";next.control_epoch=2;
    next.version.task_id="next";const auto rejected=t.grant(next,10.003,healthy());
    assert(!rejected.accepted&&rejected.reason=="writer_fault_latched"&&t.executionId()=="execute");
    assert(!token->finish([&]{write_failed.store(true);})&&!write_failed.load());
  });
  run("continuous_ratio_follows_selected_typed_commands_without_startup_or_reason_timer",[]{auto t=writer();
    SdkInputMapping m;m.enabled=true;m.component_cap=true;m.continuous_command_ratio=true;
    m.planar={.30,.40,.10,.60,.60};m.yaw=m.planar;m.validate();SdkFractionSlew mapper;
    const auto send=[&](double at){const auto raw=t.tick(at,healthy());assert(raw);
      const auto target=mappedSdkFractions(raw->linear.x,raw->linear.y,raw->angular.z,"holonomic_xyyaw",m,.3,.5);
      const auto out=mapper.apply(target,raw->linear.x,raw->linear.y,raw->angular.z,m,
        {t.executionId(),t.epoch(),t.armGeneration()},at,
        t.ordinaryGoalDecelerationSelected(),t.ordinaryYawCoastingSelected());
      assert(out.forward_back==target.forward_back&&out.left_right==target.left_right&&out.yaw==target.yaw);
      t.moveAttempted(sdkMoving(out));t.writeCalled(at,true,sdkMoving(out));return out;};
    assert(t.taskLease(lease(),10.));auto c=command();c.velocity.linear.x=.042;c.velocity.angular.z=.001;
    c.reason="tracking";assert(t.localCommand(c,10.));auto out=send(10.);
    assert(out.forward_back>=.30&&out.forward_back<.31&&out.yaw>0.&&out.yaw<.001);
    assert(!t.motionNeverSubmitted());
    c=command(10.05,2);c.velocity.linear.x=.005;c.velocity.angular.z=-.001;c.reason="goal_position_settling";
    c.trajectory_id=2;assert(t.localCommand(c,10.05));out=send(10.05);
    assert(out.forward_back>0.&&out.forward_back<.30&&out.yaw<0.); // No forced reversal/curve zero.
    out=send(10.10);assert(out.forward_back>0.&&out.forward_back<.30); // No release timer to zero.
    out=send(10.151);assert(!sdkMoving(out)); // Original body/command lease still ends motion.
    assert(t.taskLease(lease(10.16,2),10.16));c=command(10.16,3,2);c.velocity.linear.x=.042;
    c.acknowledged_discontinuity_write=t.taskCommandFeedback()->discontinuity_write_sequence;
    assert(t.localCommand(c,10.16));out=send(10.16);assert(out.forward_back>=.30); // New source, not stale revival.
    auto revoke=lease(10.17,3);revoke.revoked=true;assert(t.taskLease(revoke,10.17));
    const auto stopped=t.tick(10.17,healthy());assert(stopped&&!planarMotionNonzero(*stopped));
    const auto zero=mapper.apply({},0.,0.,0.,m,{t.executionId(),t.epoch(),t.armGeneration()},10.17);
    assert(!sdkMoving(zero));assert(!t.localCommand(command(10.18,4,3),10.18));
    assert(!t.report(10.18).measured_stop_confirmed);});
  run("independent_20Hz_writer_phase_has_no_normal_lease_or_curve_zero",[]{
    for(double offset:{.000001,.001,.024,.049}) {
      auto t=writer();SdkInputMapping m;m.enabled=m.component_cap=m.continuous_command_ratio=true;
      m.planar={.31328213472822414,.40,.10,.6,.6};m.yaw=m.planar;SdkFractionSlew mapper;
      uint64_t lease_sequence=0;
      for(uint64_t i=0;i<160;++i) {
        const double source=10.+i*.05,receipt=source+.0000001,writer_at=source+offset;
        if(i%2==0){++lease_sequence;assert(t.taskLease(lease(source,lease_sequence),receipt));}
        auto c=command(source,i+1,lease_sequence);
        const double angle=.2+static_cast<double>(i)*.005;
        c.velocity.linear.x=.15*std::cos(angle);c.velocity.linear.y=.15*std::sin(angle);
        c.velocity.angular.z=(i%2?1.:-1.)*.001;c.reason="tracking";c.trajectory_id=i/9+1;
        assert(t.localCommand(c,receipt));const auto raw=t.tick(writer_at,healthy());assert(moving(raw));
        const auto fraction=mappedSdkFractions(raw->linear.x,raw->linear.y,raw->angular.z,"holonomic_xyyaw",m,.3,.5);
        const auto out=mapper.apply(fraction,raw->linear.x,raw->linear.y,raw->angular.z,m,
          {t.executionId(),t.epoch(),t.armGeneration()},writer_at);
        assert(sdkMoving(out)&&out.forward_back==fraction.forward_back&&out.left_right==fraction.left_right&&out.yaw==fraction.yaw);
        t.moveAttempted(sdkMoving(out));t.writeCalled(writer_at,true,sdkMoving(out));
      }
      const double source=10.+160*.05;
      assert(t.taskLease(lease(source,++lease_sequence),source));
      const auto expired=t.tick(source+.101,healthy());assert(expired&&!moving(expired));
      assert(!sdkMoving(mapper.apply({},0.,0.,0.,m,{t.executionId(),t.epoch(),t.armGeneration()},source+.101)));
      auto stale=command(source+.102,161,lease_sequence);stale.body_source_stamp=stamp(source-.05);
      stale.body_valid_until=stale.valid_until=stamp(source+.05);assert(!t.localCommand(stale,source+.102));
      auto revoke=lease(source+.103,++lease_sequence);revoke.revoked=true;assert(t.taskLease(revoke,source+.103));
      assert(!t.localCommand(command(source+.104,162,lease_sequence),source+.104));
      assert(!t.report(source+.104).measured_stop_confirmed);
    }
  });
  run("independent_20Hz_delivery_jitter_preserves_original_body_deadline",[]{
    // Deliver the real typed command and owner lease on independent clocks.
    // This is not a latest-message timestamp rewrite: the body was sampled
    // 4 ms before command generation, and its original 100 ms deadline stays.
    for(double phase:{.0,.000001,.001,.009,.019,.024,.039,.045,.049,.049999}) {
      auto t=writer();SdkInputMapping m;m.enabled=m.component_cap=m.continuous_command_ratio=true;
      m.planar={.31328213472822414,.40,.10,.6,.6};m.yaw={.30,.40,.10,.6,.6};SdkFractionSlew mapper;
      uint64_t next_lease=0,next_command=0,lease_sequence=0;bool primed=false;
      const double jitter[]={.001,.005,.013,.035,.045};
      for(uint64_t i=0;i<200;++i) {
        const double writer_at=10.+static_cast<double>(i)*.05+phase;
        for(;;) {
          const double lease_at=10.+static_cast<double>(next_lease)*.1,lease_receipt=lease_at+.0000001;
          const double source=10.+static_cast<double>(next_command)*.05;
          const double command_receipt=source+jitter[next_command%5];
          if(std::min(lease_receipt,command_receipt)>writer_at)break;
          // Merge callbacks by actual arrival, not message type. In
          // particular a future owner heartbeat cannot precede an already
          // received older local command merely due to fixture batching.
          if(lease_receipt<=command_receipt) {
            ++lease_sequence;assert(t.taskLease(lease(lease_at,lease_sequence),lease_receipt));++next_lease;continue;
          }
          auto c=command(source,next_command+1,next_command/2+1);
          c.body_source_stamp=afterNs(c.source_stamp,-4000000LL);
          c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
          const double angle=.6+static_cast<double>(next_command)*.006;
          c.velocity.linear.x=.15*std::cos(angle);c.velocity.linear.y=.15*std::sin(angle);
          c.velocity.angular.z=(next_command%2?1.:-1.)*.001;c.reason="tracking";c.trajectory_id=next_command/7+1;
          const bool accepted=t.localCommand(c,command_receipt);
          if(!accepted)std::cerr<<"jitter command rejected phase="<<phase<<" index="<<next_command
            <<" source="<<source<<" lease="<<c.lease_sequence<<" writer="<<writer_at<<'\n';
          assert(accepted);++next_command;primed=true;
        }
        const auto raw=t.tick(writer_at,healthy());
        if(!primed){assert(raw&&!moving(raw));continue;}
        assert(moving(raw));
        const auto target=mappedSdkFractions(raw->linear.x,raw->linear.y,raw->angular.z,"holonomic_xyyaw",m,.3,.5);
        const auto out=mapper.apply(target,raw->linear.x,raw->linear.y,raw->angular.z,m,
          {t.executionId(),t.epoch(),t.armGeneration()},writer_at);
        assert(sdkMoving(out)&&out.forward_back==target.forward_back&&out.left_right==target.left_right&&out.yaw==target.yaw);
        t.moveAttempted(sdkMoving(out));t.writeCalled(writer_at,true,sdkMoving(out));
      }
      assert(primed&&!t.stopping()&&!t.state(20.).fault_latched);
    }
  });
  run("pipeline_jitter_cannot_extend_original_body_or_revoked_task",[]{auto t=writer();
    assert(t.taskLease(lease(),10.));auto c=command();
    c.body_source_stamp=afterNs(c.source_stamp,-4000000LL);
    c.body_valid_until=c.valid_until=afterNs(c.body_source_stamp,100000000LL);
    assert(t.localCommand(c,10.045));assert(moving(t.tick(10.095999,healthy())));
    assert(!moving(t.tick(10.096001,healthy()))&&!t.stopping());
    // Delivery after the true body deadline is rejected even though the
    // command pipeline budget alone would have accepted a 97 ms-old source.
    c.sequence=2;assert(!t.localCommand(c,10.097));assert(!moving(t.tick(10.097,healthy())));
    c=command(10.098,3);assert(t.localCommand(c,10.098));assert(moving(t.tick(10.098,healthy())));
    auto revoke=lease(10.099,2);revoke.revoked=true;assert(t.taskLease(revoke,10.099));
    assert(!t.localCommand(command(10.100,4,2),10.100));assert(!moving(t.tick(10.100,healthy())));
    assert(!t.report(10.100).measured_stop_confirmed);
  });
  run("normal_yaw_coasting_is_only_a_current_selected_tracking_command",[]{auto t=writer();
    assert(!t.ordinaryYawCoastingSelected());assert(t.taskLease(lease(),10.));
    auto c=command();c.reason="tracking_forward_yaw_coasting";c.velocity.angular.z=.01;
    assert(t.localCommand(c,10.));assert(!t.ordinaryYawCoastingSelected());
    assert(moving(t.tick(10.,healthy()))&&t.ordinaryYawCoastingSelected());
    t.writeCalled(10.,true);assert(!t.ordinaryYawCoastingSelected());
    assert(!moving(t.tick(10.11,healthy()))&&!t.ordinaryYawCoastingSelected());
    assert(t.taskLease(lease(10.12,2),10.12));c=command(10.12,2,2);
    c.reason="tracking_forward_yaw_coasting";assert(t.localCommand(c,10.12));
    assert(moving(t.tick(10.12,healthy()))&&!t.ordinaryYawCoastingSelected()); // Exact yaw zero.
    c=command(10.13,3,2);c.reason="tracking_forward_yaw_coasting";c.velocity.linear.x=0.;c.velocity.angular.z=.01;
    assert(t.localCommand(c,10.13));assert(moving(t.tick(10.13,healthy()))&&!t.ordinaryYawCoastingSelected());
    c=command(10.14,4,2);c.reason="tracking_forward_yaw_coasting";c.velocity.angular.z=.01;
    assert(t.localCommand(c,10.14));assert(moving(t.tick(10.14,healthy()))&&t.ordinaryYawCoastingSelected());
    t.fault("fixture_sdk_fault",10.15);assert(!t.ordinaryYawCoastingSelected());});
  run("normal_goal_deceleration_is_only_a_current_selected_typed_nonzero_fact",[]{auto t=writer();
    assert(!t.ordinaryGoalDecelerationSelected());assert(t.taskLease(lease(),10.));
    auto c=command();c.reason="goal_position_settling";assert(t.localCommand(c,10.));
    assert(!t.ordinaryGoalDecelerationSelected());assert(moving(t.tick(10.,healthy())));
    assert(t.ordinaryGoalDecelerationSelected());t.writeCalled(10.,true);assert(!t.ordinaryGoalDecelerationSelected());
    assert(!moving(t.tick(10.11,healthy())));assert(!t.ordinaryGoalDecelerationSelected());
    auto p=lease(10.12,2);assert(t.taskLease(p,10.12));c=command(10.12,2,2);c.reason="goal_position_settling";
    c.velocity={};c.hold=true;assert(t.localCommand(c,10.12));assert(!moving(t.tick(10.12,healthy())));
    assert(!t.ordinaryGoalDecelerationSelected());
    c=command(10.13,3,2);c.reason="tracking_forward";assert(t.localCommand(c,10.13));
    assert(moving(t.tick(10.13,healthy()))&&!t.ordinaryGoalDecelerationSelected());
    c=command(10.14,4,2);c.reason="goal_position_settling";assert(t.localCommand(c,10.14));
    assert(moving(t.tick(10.14,healthy()))&&t.ordinaryGoalDecelerationSelected());
    t.fault("fixture_sdk_fault",10.15);assert(!t.ordinaryGoalDecelerationSelected());});
  run("first_task_moves_without_curve_CAS_or_proofs",[]{auto t=writer();ready(t);
    assert(moving(t.tick(10.,healthy())));assert(t.selectedCommitSequence()==0);t.writeCalled(10.,true);
    assert(!t.takeCommitAck());assert(t.state(10.).writer_commit_sequence==0);assert(!t.motionNeverSubmitted());});
  run("mapped_initial_zero_retires_without_faking_measured_stop",[]{auto t=writer();ready(t);
    t.mc(1,10.,10.,.12,.1,10.,"mock-clock"); // actual body can still be moving
    const auto raw=t.tick(10.,healthy());assert(moving(raw));SdkInputMapping m;m.enabled=true;SdkFractionSlew slew;
    const auto target=mappedSdkFractions(raw->linear.x,raw->linear.y,raw->angular.z,"holonomic_xyyaw",m,.3,.5);
    const auto fraction=slew.apply(target,raw->linear.x,raw->linear.y,raw->angular.z,m,
      {t.executionId(),t.epoch(),t.armGeneration()},10.);assert(!sdkMoving(fraction));
    t.setSDKTransactions(1,false);t.moveAttempted(sdkMoving(fraction));t.writeCalled(10.,true,sdkMoving(fraction));
    assert(t.motionNeverSubmitted()&&t.lastDemand().velocity.linear.x==.2);
    assert(!t.report(10.).measured_stop_confirmed&&!t.stationaryEvidence(10.).usable);
    auto revoke=lease(10.01,2);revoke.revoked=true;assert(t.taskLease(revoke,10.01));
    assert(!t.noMotionRetired()&&!t.retirement(10.01).software_retired);
    t.zeroAcknowledged(1,10.01,10.01);t.setSDKTransactions(0,false); // vendor callback retired
    assert(t.noMotionRetired()&&t.retirement(10.01).software_retired&&t.retirement(10.01).motion_never_submitted);
    assert(!t.report(10.01).measured_stop_confirmed&&!t.tick(10.01,healthy()));
    auto next=request(10.02);next.execution_id="next";next.request_id="next-grant";
    next.control_epoch=2;next.version.task_id="next-task";assert(t.grant(next,10.02,healthy()).accepted);});
  run("mapped_nonzero_keeps_stop_obligation_until_new_MC_proof",[]{auto t=writer();ready(t);
    SdkInputMapping m;m.enabled=true;SdkFractionSlew slew;const auto send=[&](double at){
      const auto raw=t.tick(at,healthy());assert(moving(raw));
      const auto target=mappedSdkFractions(raw->linear.x,raw->linear.y,raw->angular.z,"holonomic_xyyaw",m,.3,.5);
      const auto fraction=slew.apply(target,raw->linear.x,raw->linear.y,raw->angular.z,m,
        {t.executionId(),t.epoch(),t.armGeneration()},at);
      t.moveAttempted(sdkMoving(fraction));t.writeCalled(at,true,sdkMoving(fraction));return sdkMoving(fraction);};
    assert(!send(10.)&&send(10.05)&&!t.motionNeverSubmitted());
    auto revoke=lease(10.06,2);revoke.revoked=true;assert(t.taskLease(revoke,10.06));
    assert(!t.noMotionRetired()&&!t.report(10.06).measured_stop_confirmed);
    t.stopWritten(10.06);assert(!t.report(10.06).measured_stop_confirmed);
    for(unsigned i=1;i<=90;++i){const auto at=10.06+i*.02;t.mc(i*20000000ULL,at,at,0.,0.,at,"mock-clock");}
    const auto last_at=10.06+90*.02;
    assert(t.report(last_at).measured_stop_confirmed&&!t.report(last_at).physical_acceptance_verified);});
  run("failed_actual_nonzero_call_keeps_uncertain_stop_obligation",[]{auto t=writer();ready(t);
    assert(moving(t.tick(10.,healthy())));t.moveAttempted(true);t.writeCalled(10.,false,true);
    assert(t.stopping()&&t.state(10.).fault_latched&&!t.motionNeverSubmitted()&&!t.noMotionRetired());
    assert(!t.report(10.).measured_stop_confirmed);});
  run("legacy_attempt_and_write_interfaces_keep_nonzero_semantics",[]{auto t=writer();ready(t);
    const auto raw=t.tick(10.,healthy());assert(moving(raw));t.moveAttempted(*raw);t.writeCalled(10.,true);
    assert(!t.motionNeverSubmitted());auto revoke=lease(10.01,2);revoke.revoked=true;
    assert(t.taskLease(revoke,10.01)&&!t.noMotionRetired()&&!t.report(10.01).measured_stop_confirmed);});
  run("task_identity_ignores_normal_anchor_reference_updates",[]{auto t=writer();ready(t);
    auto c=command(10.05,2);c.version.reference_generation=999;c.version.anchor_id="new-anchor";
    c.version.anchor_revision=c.version.context_sequence=99;c.trajectory_id=2;
    assert(t.localCommand(c,10.05));assert(moving(t.tick(10.05,healthy())));
    assert(t.state(10.05).applied_trajectory_id==2&&t.state(10.05).writer_commit_sequence==0);});
  run("continuous_20Hz_commands_curve_changes_do_not_handshake",[]{auto t=writer();
    for(uint64_t i=0;i<100;++i){const double at=10.+i*.05;const auto p=lease(at,i+1);
      assert(t.taskLease(p,at));auto c=command(at,i+1,i+1);c.trajectory_id=static_cast<int64_t>(i/10+1);
      assert(t.localCommand(c,at));assert(moving(t.tick(at,healthy())));t.writeCalled(at,true);
      assert(!t.takeCommitAck()&&t.selectedCommitSequence()==0&&!t.stopping());}});
  run("lease_heartbeat_does_not_retire_current_command",[]{auto t=writer();ready(t);
    assert(t.taskLease(lease(10.05,2),10.05));assert(moving(t.tick(10.05,healthy())));
    auto c=command(10.06,2,1);assert(t.localCommand(c,10.06));assert(moving(t.tick(10.06,healthy())));});
  run("async_native_verdict_must_reference_lease_at_original_command_source",[]{auto t=writer();ready(t);
    // The command was produced at 10.04. During its Native check, the
    // independent 10 Hz owner publishes lease 2 at 10.05. Mapping this old
    // source onto the latest lease invents causality and is rejected. The
    // original still-live lease is valid without any source/TTL rewrite.
    assert(t.taskLease(lease(10.05,2),10.05));auto c=command(10.04,2,2);
    assert(!t.localCommand(c,10.06));c.lease_sequence=1;
    assert(t.localCommand(c,10.06));assert(moving(t.tick(10.06,healthy())));
    auto hold=lease(10.07,3);hold.allowed=false;hold.phase="holding";
    assert(t.taskLease(hold,10.07));c.sequence=3;
    assert(!t.localCommand(c,10.08));assert(!moving(t.tick(10.08,healthy())));});
  run("task_lease_expiry_is_hold_and_fresh_input_recovers",[]{auto t=writer();ready(t);
    for(int i=0;i<8;++i)t.tick(10.+i*.05,healthy());
    assert(!moving(t.tick(10.36,healthy()))&&!t.stopping()&&!t.state(10.36).fault_latched);
    auto c=command(10.37,2,1);assert(!t.localCommand(c,10.37));
    ready(t,10.38,2);assert(moving(t.tick(10.38,healthy())));});
  run("body_original_deadline_is_not_command_renewable",[]{auto t=writer();ready(t);
    assert(moving(t.tick(10.09,healthy())));assert(!moving(t.tick(10.101,healthy()))&&!t.stopping());
    auto c=command(10.11,2);c.body_source_stamp=stamp(10.);c.body_valid_until=stamp(10.1);c.valid_until=c.body_valid_until;
    assert(!t.localCommand(c,10.11));ready(t,10.12,2);assert(moving(t.tick(10.12,healthy())));});
  run("explicit_hold_fences_previous_lease_and_commands",[]{auto t=writer();ready(t);
    auto hold=lease(10.05,2);hold.allowed=false;hold.phase="holding";assert(t.taskLease(hold,10.05));
    assert(!moving(t.tick(10.05,healthy()))&&!t.stopping());
    assert(t.taskLease(lease(10.06,3),10.06));assert(!t.localCommand(command(10.06,2,1),10.06));
    assert(t.localCommand(command(10.07,3,3),10.07));assert(moving(t.tick(10.07,healthy())));});
  for(int kind=0;kind<2;++kind)run(kind?"robot_state_gap_hold":"MC_gap_hold",[kind]{auto t=writer();ready(t);
    t.tick(10.,healthy());auto h=healthy();if(kind)h.robot_state_fresh=false;else h.mc_fresh=false;
    assert(!moving(t.tick(10.05,h))&&!t.stopping());
    assert(!t.localCommand(command(10.06,2),10.06));t.tick(10.07,healthy());
    assert(!t.localCommand(command(10.08,2),10.08));ready(t,10.09,2);assert(moving(t.tick(10.09,healthy())));});
  for(int kind=0;kind<4;++kind)run("explicit_hardware_or_owner_loss_is_terminal",[kind]{auto t=writer();ready(t);
    t.tick(10.,healthy());auto h=healthy();if(kind==0)h.connected=false;if(kind==1)h.owned=false;
    if(kind==2)h.estop=true;if(kind==3)h.general=false;t.tick(10.05,h);
    assert(t.stopping()&&t.state(10.05).fault_latched);assert(!t.taskLease(lease(10.06,2),10.06));
    assert(!t.localCommand(command(10.06,2,2),10.06));});
  run("explicit_revoke_cannot_be_revived_by_late_command",[]{auto t=writer();ready(t);
    auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05)&&t.stopping());
    assert(!t.localCommand(command(10.06,2),10.06));assert(!t.taskLease(lease(10.06,3),10.06));
    assert(t.retirement(10.06).software_retired&&t.retirement(10.06).motion_never_submitted);
    assert(!t.report(10.06).measured_stop_confirmed);});
  run("all_stable_foreign_identities_rejected_without_incumbent_effect",[]{auto t=writer();ready(t);
    for(int kind=0;kind<13;++kind){auto c=command(10.01,2);if(kind==0)c.version.session_id="other";
      if(kind==1)c.version.task_id="other";if(kind==2)c.version.route_id="other";if(kind==3)c.version.route_hash=std::string(64,'b');
      if(kind==4)c.version.map_version_id="other";if(kind==5)c.version.localization_epoch=2;if(kind==6)c.version.localization_seed_id="other";
      if(kind==7)c.execution_id="other";if(kind==8)c.control_epoch=2;if(kind==9)c.sdk_session="other";
      if(kind==10)c.sdk_arm_generation=2;if(kind==11)c.transport_mode="live";if(kind==12)c.version.schema_version=2;
      assert(!t.localCommand(c,10.01));assert(moving(t.tick(10.01,healthy())));}});
  run("command_invalid_clock_frame_sequence_and_deadline_do_not_clear_current",[]{auto t=writer();ready(t);
    for(int kind=0;kind<10;++kind){auto c=command(10.01,2);if(kind==0)c.source_stamp=stamp(10.02);
      if(kind==1)c.body_source_stamp=stamp(10.02);if(kind==2)c.body_valid_until=stamp(10.2);
      if(kind==3)c.valid_until=stamp(10.2);if(kind==4)c.body_frame="map";if(kind==5)c.sequence=1;
      if(kind==6)c.lease_sequence=100;if(kind==7)c.velocity.linear.x=std::numeric_limits<double>::quiet_NaN();
      if(kind==8)c.velocity.linear.z=.1;if(kind==9)c.hold=true;
      assert(!t.localCommand(c,10.01));assert(moving(t.tick(10.01,healthy())));}});
  run("writer_saturates_planar_request_without_task_failure",[]{auto t=writer();assert(t.taskLease(lease(),10.));
    auto c=command();c.velocity.linear.x=c.velocity.linear.y=1.;c.velocity.angular.z=1.;assert(t.localCommand(c,10.));
    const auto v=t.tick(10.,healthy());assert(v&&std::abs(std::hypot(v->linear.x,v->linear.y)-.3)<1e-12);
    assert(v->angular.z==.5&&!t.stopping());});
  run("legacy_geometric_inputs_cannot_enter_task_writer",[]{auto t=writer();
    assert(!t.permit(Permit{},10.)&&!t.demand(Demand{},10.)&&!t.motionValidation(MotionProof{},10.));
    assert(!t.localState(Local{},10.)&&!t.trajectoryGeometry(Tagged{},10.)&&!t.trajectoryValidation(CurveProof{},10.));
    assert(!t.trackingProgress(Progress{},10.)&&!t.handoff(Handoff{},10.)&&!t.preparedDemand(Prepared{},10.));
    ready(t);assert(moving(t.tick(10.,healthy())));});
  run("software_retirement_does_not_fake_physical_stop",[]{auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));
    t.writeCalled(10.,true);auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));
    assert(t.retirement(10.05).software_retired&&!t.retirement(10.05).motion_never_submitted);
    assert(!t.report(10.05).measured_stop_confirmed);t.stopWritten(10.05);
    for(unsigned i=1;i<=90;++i){const auto at=10.05+i*.02;t.mc(i*20000000ULL,at,at,0.,0.,at,"mock-clock");}
    const auto last_at=10.05+90*.02;
    assert(t.report(last_at).measured_stop_confirmed&&!t.report(last_at).physical_acceptance_verified);});
  run("new_target_requires_explicit_new_grant_and_old_command_stays_retired",[]{auto t=writer();ready(t);
    auto p=lease(10.05,2);p.revoked=true;assert(t.taskLease(p,10.05));auto r=request(10.06);
    r.version.task_id="new-task";r.execution_id="new-execute";r.control_epoch=2;r.request_id="new-grant";
    assert(t.grant(r,10.06,healthy()).accepted);assert(!t.localCommand(command(10.07,2),10.07));
    auto np=lease(10.07,1);np.version=r.version;np.execution_id=r.execution_id;np.control_epoch=2;np.sdk_arm_generation=2;
    assert(t.taskLease(np,10.07));auto nc=command(10.07,1);nc.version=r.version;nc.execution_id=r.execution_id;
    nc.control_epoch=2;nc.sdk_arm_generation=2;assert(t.localCommand(nc,10.07));assert(moving(t.tick(10.07,healthy())));});
  run("software_estop_before_first_grant_can_release_without_reviving_old_arm",[]{
    Transport t("sdk","isolated_mock");t.configureTaskFollower(true);
    t.configureMcCaptureBound(.02,"isolated_simulated_source_clock");t.configureExecutionPolicy(testExecutionPolicy(),.8);
    t.setSDKTransactions(0,false);t.softwareEstopRequested(10.);
    assert(t.softwareEstopLatched()&&t.state(10.).fault_latched);
    assert(!t.grant(request(),10.,healthy()).accepted&&!t.takeControlOnce());
    assert(t.confirmSoftwareEstopRelease(10.1)&&t.softwareEstopReleasePending());
    auto stale=healthy();stale.robot_state_fresh=false;t.tick(10.11,stale);
    assert(t.softwareEstopLatched());t.tick(10.12,healthy());
    assert(!t.softwareEstopLatched()&&!t.state(10.12).fault_latched&&!t.takeControlOnce());
    assert(!t.grant(request(),10.12,healthy()).accepted); // Original pending arm remains fenced.
    assert(t.grant(request(10.13),10.13,healthy()).accepted);
  });
  run("software_estop_release_waits_exact_pending_software_retirement",[]{auto t=writer();ready(t);
    t.tick(10.,healthy());t.setSDKTransactions(1,false);t.softwareEstopRequested(10.01);
    auto stopped_health=healthy();stopped_health.estop=stopped_health.software_estop_only=true;
    assert(!moving(t.tick(10.02,stopped_health)));assert(!t.taskLease(lease(10.03,2),10.03));
    assert(!t.localCommand(command(10.03,2,2),10.03));
    assert(t.confirmSoftwareEstopRelease(10.04));t.tick(10.05,healthy());
    assert(t.softwareEstopLatched()&&t.stopping());
    t.setSDKTransactions(0,true);t.tick(10.06,healthy());assert(t.softwareEstopLatched());
    t.setSDKTransactions(0,false);t.tick(10.07,healthy());
    assert(!t.softwareEstopLatched()&&t.noMotionRetired()&&t.stopping());
    assert(!t.grant(request(),10.07,healthy()).accepted); // Old duplicate not re-armed.
    auto r=request(10.08);r.request_id="after-estop";r.execution_id="after-estop-execution";
    r.confirmation_id="after-estop-confirmation";r.control_epoch=2;r.version.task_id="after-estop-task";
    assert(t.grant(r,10.08,healthy()).accepted);assert(!t.localCommand(command(10.09,3,3),10.09));
    assert(!moving(t.tick(10.09,healthy()))); // Fresh task still needs its own lease/command.
  });
  run("software_estop_OFF_precedes_zero_ACK_and_actual_stop_without_old_motion",[]{auto t=writer();ready(t);
    assert(moving(t.tick(10.,healthy())));t.moveAttempted(true);t.writeCalled(10.,true,true);
    t.softwareEstopRequested(10.05);auto on=healthy();on.estop=on.software_estop_only=true;
    assert(!t.tick(10.05,on));assert(!t.report(10.05).measured_stop_confirmed);
    assert(t.confirmSoftwareEstopRelease(10.1));
    const auto zero=t.tick(10.1,healthy());assert(zero&&!planarMotionNonzero(*zero));
    assert(t.softwareEstopLatched()&&!t.report(10.1).measured_stop_confirmed);
    t.stopWritten(10.1); // Real writer ACK barrier, never synthesized by OFF.
    for(unsigned i=1;i<=70;++i){const auto at=10.1+i*.02;
      t.mc(i*20000000ULL,at,at,0.,0.,at,"mock-clock");assert(!moving(t.tick(at,healthy())));
      if(i<=40)assert(t.softwareEstopLatched());}
    const auto at=11.5;assert(t.report(at).measured_stop_confirmed&&!t.softwareEstopLatched());
    assert(t.stopping()&&!t.taskLease(lease(at,50),at));
    auto r=request(11.52);r.control_epoch=2;r.request_id="next";r.confirmation_id="next-confirmation";
    r.execution_id="next-execution";r.version.task_id="next-task";assert(t.grant(r,11.52,healthy()).accepted);
    assert(!t.localCommand(command(11.53,60),11.53));
  });
  for(int cause=0;cause<4;++cause)run("software_release_does_not_clear_other_fault",[cause]{auto t=writer();ready(t);
    t.tick(10.,healthy());t.softwareEstopRequested(10.01);
    if(cause==0)t.fault("hardware_emergency",10.02);
    if(cause==1)t.submissionFailed(10.02);
    if(cause==2){auto lost=healthy();lost.owned=false;t.tick(10.02,lost);}
    if(cause==3){auto lost=healthy();lost.connected=false;t.tick(10.02,lost);t.fault("SDK_connection_error",10.02);}
    assert(!t.confirmSoftwareEstopRelease(10.03));t.tick(10.04,healthy());
    assert(t.softwareEstopLatched()&&t.state(10.04).fault_latched);
    auto r=request(10.05);r.request_id="new";r.control_epoch=2;r.execution_id="new";r.version.task_id="new";
    assert(!t.grant(r,10.05,healthy()).accepted);
  });
  run("new_software_ON_revokes_pending_OFF_confirmation",[]{auto t=writer();ready(t);
    t.tick(10.,healthy());t.softwareEstopRequested(10.01);assert(t.confirmSoftwareEstopRelease(10.02));
    t.softwareEstopObserved(10.03);t.tick(10.04,healthy());
    assert(t.softwareEstopLatched()&&!t.softwareEstopReleasePending());
    assert(!t.confirmSoftwareEstopRelease(10.02));assert(t.confirmSoftwareEstopRelease(10.05));
    t.tick(10.06,healthy());assert(!t.softwareEstopLatched());
  });
  run("known_software_STOP_does_not_become_hardware_fault_on_RobotState_gap",[]{auto t=writer();ready(t);
    assert(moving(t.tick(10.,healthy())));t.moveAttempted(true);t.writeCalled(10.,true,true);
    t.softwareEstopRequested(10.05);auto gap=healthy();
    gap.estop=gap.software_estop_only=true;gap.robot_state_fresh=false;
    for(unsigned i=1;i<=60;++i)assert(!t.tick(10.05+i*.05,gap));
    assert(t.softwareEstopLatched()&&t.confirmSoftwareEstopRelease(13.06));
    gap.estop=false;t.tick(13.07,gap);
    assert(t.softwareEstopReleasePending()&&!t.report(13.07).measured_stop_confirmed);
    assert(!t.taskLease(lease(13.08,80),13.08)&&!t.localCommand(command(13.08,80,80),13.08));
  });
  run("manual_handoff_retires_old_identity_without_resetting_epoch_or_replay",[]{
    auto t=writer();ready(t);assert(!t.completeManualHandoff(10.01));
    t.stop("manual_takeover",10.01);t.setSDKTransactions(1,false);
    assert(!t.completeManualHandoff(10.02));t.setSDKTransactions(0,false);
    const auto final=t.retirement(10.02);assert(final.software_retired&&final.motion_never_submitted);
    assert(t.completeManualHandoff(10.02)&&t.executionId().empty());
    t.setSDKTransactions(1,false);assert(!t.completeManualHandoff(10.025));t.setSDKTransactions(0,true);
    assert(!t.completeManualHandoff(10.026));t.setSDKTransactions(0,false);
    assert(!t.taskLease(lease(10.03,2),10.03)&&!t.localCommand(command(10.03,2,2),10.03));
    auto changed=healthy();changed.general=false;changed.motion_mode="manual_posture";
    assert(!t.tick(11.,changed)&&!t.state(11.).fault_latched);
    auto replay=request(11.);assert(!t.grant(replay,11.,healthy()).accepted);
    auto next=request(11.);next.request_id="new";next.execution_id="new";next.version.task_id="new";next.control_epoch=2;
    assert(t.grant(next,11.,healthy()).accepted&&t.armGeneration()==2);
  });
  run("manual_handoff_cannot_erase_prior_motion_or_fault",[]{
    auto t=writer();ready(t);assert(moving(t.tick(10.,healthy())));t.moveAttempted(true);t.writeCalled(10.,true,true);
    t.stop("manual_takeover",10.01);assert(!t.completeManualHandoff(10.02));
    t.stopWritten(10.03);assert(!t.completeManualHandoff(10.04));
    auto failed=writer();failed.fault("SDK_failed",10.01);assert(!failed.completeManualHandoff(10.02));
    auto emergency=writer();emergency.softwareEstopRequested(10.01);assert(!emergency.completeManualHandoff(10.02));
  });
  std::cout<<"task writer: "<<cases<<" cases passed\n";
}
