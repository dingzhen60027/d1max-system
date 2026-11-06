#include "sdk_command_trace.hpp"
#include <nlohmann/json.hpp>
#include <cassert>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <thread>
#include <vector>

using namespace d1monitor::command_trace;
using Json=nlohmann::json;
namespace d1monitor::command_trace {
struct TraceTestAccess {static std::unique_lock<std::mutex> lock(Trace& t){return std::unique_lock<std::mutex>(t.mutex_);}};
}
constexpr int64_t base=1793022800000000000LL,steady=456000000000000LL;
Record move(Trace& trace,int i){
  Record r;r.attempt=trace.nextAttempt();r.write=i+1;r.sequence=5000+i;r.curve=321;r.commit=123;
  copy(r.identity.session,"6855a7dac76a431786f1a213313a208f");copy(r.identity.task,"6855a7dac76a431786f1a213313a208f.2");
  copy(r.identity.execution,"exec.6855a7dac76a431786f1a213313a208f.2.1793022560117");copy(r.identity.sdk,"aabbccdd77889900");
  r.identity.epoch=5;r.identity.arm=3;copy(r.reason,"tracking_geometry_only");
  r.selected=true;r.selected_ns=base+i*50000000LL;r.begin_ns=r.selected_ns+18000;r.end_ns=r.begin_ns+230000;
  r.begin_steady_ns=steady+i*50000000LL;r.end_steady_ns=r.begin_steady_ns+230000;
  r.source_ns=r.selected_ns-1600000;r.body_ns=r.selected_ns-7400000;r.deadline_ns=r.selected_ns+80000000;
  r.raw_forward=.297495+.00015*std::sin(i*.2);r.raw_yaw=.0023*std::sin(i*.1);
  r.forward=.35f+.0005f*std::sin(i*.2);r.yaw=0.f;
  r.measured={998877660000000ULL+static_cast<uint64_t>(i)*50000000,10000+static_cast<uint64_t>(i),7,
    r.selected_ns-8000000,r.begin_steady_ns-8018000,.28678f,.001945f};
  return r;
}
size_t countKind(const Json& j,const std::string& kind){size_t count=0;for(const auto& e:j.at("events"))if(e.at(0)==kind)++count;return count;}
std::vector<std::string> pages(Trace& trace,size_t budget=Trace::byte_budget){
  std::vector<std::string> result;if(!trace.beginDrain())return result;
  while(trace.pending()) {
    const auto page=trace.preparePage(budget);assert(!page.empty()&&page.size()<=budget);
    result.push_back(page);assert(trace.commitPage());assert(result.size()<=Trace::capacity+1);
  }
  return result;
}
Json take(Trace& trace){
  const auto batches=pages(trace);assert(!batches.empty());
  auto result=Json::parse(batches.back());result["events"]=Json::array();
  for(const auto& page:batches){const auto parsed=Json::parse(page);
    for(const auto& event:parsed["events"])result["events"].push_back(event);}
  return result;
}
int main(){
  // Full 20 Hz output plus ACK, including variable velocities and unmodified
  // measurement stamps. No ROS initialization or vendor SDK is involved.
  size_t maximum=0,total=0;
  for(int second=0;second<60;++second){
    Trace trace;
    for(int i=0;i<20;++i){auto r=move(trace,second*20+i);trace.push(r);
      trace.ack(r.attempt,r.write,r.end_ns+300000,r.end_steady_ns+300000,0,"system",true);}
    size_t moves=0,acks=0;for(const auto& page:pages(trace)) {
      const auto j=Json::parse(page);maximum=std::max(maximum,page.size());total+=page.size();
      moves+=countKind(j,"M");acks+=countKind(j,"A");assert(j["drop"]["budget"]==0);
    }
    assert(moves==20&&acks==20&&!trace.beginDrain());
  }
  // Diagnostic timer jitter of one 200 ms period still fits without loss.
  {
    Trace trace;for(int i=0;i<24;++i){auto r=move(trace,i);trace.push(r);
      trace.ack(r.attempt,r.write,r.end_ns+300000,r.end_steady_ns+300000,0,"system",true);}
    const auto j=take(trace);
    assert(countKind(j,"M")==24&&countKind(j,"A")==24&&j["drop"]["budget"]==0);
  }
  // ACK can arrive inline, before Move returns. Event order does not redefine
  // source/call timestamps; attempt ID disambiguates write_seq reused by a task.
  {
    Trace trace;auto r=move(trace,0);trace.ack(r.attempt,r.write,r.begin_ns+10,r.begin_steady_ns+10,0,"system",true);trace.push(r);
    auto second=move(trace,1);second.write=1;copy(second.identity.task,"next_task");second.identity.arm=4;trace.push(second);
    trace.ack(r.attempt,r.write,r.end_ns+50,r.end_steady_ns+50,5,"system",false);
    const auto j=take(trace);assert(countKind(j,"I")==2&&countKind(j,"M")==2&&countKind(j,"A")==2);
    assert(j["events"][0][0]=="A"&&j["events"][0][2]==r.attempt);
    assert(j["events"].back()[8]==0); // duplicate/late callback is observation only
  }
  // Expiry is its own cause, never a falsely selected expired velocity.
  {
    Trace trace;auto r=move(trace,0);r.selected=false;r.cause_sequence=r.sequence;r.sequence=0;
    r.last_expiry_sequence=r.cause_sequence;r.expiry_count=1;r.new_expiry=true;
    r.raw_forward=r.raw_yaw=0.;r.forward=r.yaw=0.f;r.expiry_ns=r.selected_ns;copy(r.reason,"waiting_local_command");trace.push(r);
    const auto j=take(trace);const auto e=j["events"].back();
    assert(e[7]==0&&e[8]==0&&e[9]==5000&&e[18]==0&&e[22]==0);
    assert(e[30]==r.measured.raw_stamp_ns&&e[33]==r.measured.sequence);
    r.new_expiry=false;r.cause_sequence=0;r.selected_ns+=50000000;trace.push(r);
    const auto again=take(trace)["events"].back();
    assert(again[9]==0&&again[37]==5000&&again[38]==1&&again[39]==0);
  }
  // Each batch is independently decodable after file rotation. Failed writes
  // cannot make subsequent batches refer to an absent dictionary.
  {
    Trace trace;trace.push(move(trace,0));const auto first=take(trace);
    trace.push(move(trace,1));const auto second=take(trace);
    assert(countKind(first,"I")==1&&countKind(second,"I")==1);
  }
  // Contended capture returns without waiting. Only producer overflow loses
  // records; splitting a fixed snapshot into pages must not lose another one.
  {
    Trace trace;auto held=TraceTestAccess::lock(trace);auto writer=std::async(std::launch::async,[&]{trace.push(move(trace,0));return true;});
    const bool returned=writer.wait_for(std::chrono::milliseconds(200))==std::future_status::ready;
    held.unlock();assert(returned&&writer.get());auto j=take(trace);assert(j["drop"]["contention"]==1);
    for(int i=0;i<400;++i)trace.push(move(trace,i));
    j=take(trace);
    assert(j["drop"]["overflow"]==400-Trace::capacity&&j["drop"]["budget"]==0);
    assert(countKind(j,"M")==Trace::capacity);
  }
  // Oversized/escaped identities are flagged, not ambiguously presented as a
  // complete identity. SDK immediate failures/throws stay distinct from ACK.
  {
    Trace trace;auto r=move(trace,0);r.identity.truncated=copy(r.identity.task,std::string(600,'q'));
    copy(r.reason,"reason\"with\ncontrol");r.error=5;copy(r.error_category,"system");r.exception=true;trace.push(r);
    const auto j=take(trace);assert(j["events"][0][8]==1);
    assert(j["events"].back()[25]==5&&j["events"].back()[27]==1&&j["events"][1][2]=="reason\"with\ncontrol");
    Trace distinct;distinct.push(r);r.attempt=distinct.nextAttempt();distinct.push(r);
    assert(countKind(take(distinct),"I")==2);
  }
  // A diagnostic sink stalled after drain owns no producer lock.
  {
    // Decode JSON scalars U+0000..U+00FF back into original bytes. No invalid
    // input or array-boundary UTF-8 truncation can invalidate the whole line.
    const auto bytes=[](const Json& value){
      const auto s=value.get<std::string>();std::string out;
      for(size_t i=0;i<s.size();++i){const auto c=static_cast<unsigned char>(s[i]);
        if(c<128)out+=s[i];
        else {assert((c==0xc2||c==0xc3)&&i+1<s.size());const auto tail=static_cast<unsigned char>(s[++i]);
          assert((tail&0xc0)==0x80);out+=static_cast<char>(((c&0x1f)<<6)|(tail&0x3f));}}
      return out;
    };
    for(const auto& text:std::vector<std::string>{u8"楼梯任务",std::string(253,'x')+u8"楼",
        std::string("bad")+static_cast<char>(0xff)+static_cast<char>(0xc0)+"tail"}){
      Trace trace;auto r=move(trace,0);r.identity.truncated=copy(r.identity.task,text);trace.push(r);
      const auto j=take(trace);assert(j["text_encoding"]=="byte_escaped");
      assert(bytes(j["events"][0][3])==text.substr(0,255));
      assert(j["events"][0][8]==(text.size()>=256?1:0));
    }
  }
  // A diagnostic sink stalled after drain owns no producer lock.
  {
    Trace trace;trace.push(move(trace,0));std::promise<void> entered,release;auto gate=release.get_future().share();
    auto sink=std::async(std::launch::async,[&]{assert(trace.beginDrain());const auto batch=trace.preparePage();entered.set_value();gate.wait();assert(trace.commitPage());return batch.size();});
    entered.get_future().wait();for(int i=1;i<21;++i)trace.push(move(trace,i));release.set_value();assert(sink.get()>0);
    const auto j=take(trace);assert(countKind(j,"M")==20&&j["drop"]["contention"]==0);
  }
  // Reproduce the field load: two writer wakeups per 50 ms plus every ACK.
  // All M/A survive page boundaries, including inline and duplicate ACKs,
  // task/write-number reuse, original source ns, and the final stop reason.
  {
    Trace trace;std::map<uint64_t,Record> expected;size_t expected_acks=0;
    std::vector<std::pair<std::string,uint64_t>> order;
    for(int i=0;i<40;++i){auto r=move(trace,i);r.write=i%20+1;
      if(i>=20){copy(r.identity.task,"second_task");copy(r.identity.execution,"second_execution");r.identity.arm=4;}
      if(i==39){r.raw_forward=r.raw_yaw=0.;r.forward=r.yaw=0.f;copy(r.reason,"manual_takeover");}
      expected[r.attempt]=r;
      if(i%2==0){trace.ack(r.attempt,r.write,r.begin_ns+10,r.begin_steady_ns+10,0,"system",true);++expected_acks;order.emplace_back("A",r.attempt);}
      trace.push(r);order.emplace_back("M",r.attempt);
      if(i%2){trace.ack(r.attempt,r.write,r.end_ns+50,r.end_steady_ns+50,0,"system",true);++expected_acks;order.emplace_back("A",r.attempt);}
    }
    trace.ack(1,1,base+3000000000LL,steady+3000000000LL,5,"system",false);++expected_acks;order.emplace_back("A",1);
    assert(trace.beginDrain());const auto first=trace.preparePage();assert(!first.empty());
    // A failed sink has no commit. The page is stable even as new input arrives.
    auto future=move(trace,99);trace.push(future);
    assert(trace.preparePage()==first&&trace.beginDrain());
    const auto batches=pages(trace);assert(batches.size()>1);
    size_t moves=0,acks=0,offset=0,page_index=0,event_index=0;bool final_zero=false,next_task_nonzero=false;
    for(const auto& page:batches){const auto j=Json::parse(page);
      assert(j["snapshot"]==1&&j["page"]==page_index++&&j["record_offset"]==offset);
      offset+=j["record_count"].get<size_t>();
      assert(j["observed"]==81&&j["snapshot_records"]==81&&j["drop"]["budget"]==0);
      std::map<size_t,Json> identities;std::map<size_t,std::string> reasons;
      for(const auto& e:j["events"]){
        if(e[0]=="M"||e[0]=="A") {
          assert(e[0]==order.at(event_index).first&&e[2]==order.at(event_index).second);
          assert(e[1]==++event_index);
        }
        if(e[0]=="I")identities[e[1].get<size_t>()]=e;
        else if(e[0]=="R")reasons[e[1].get<size_t>()]=e[2].get<std::string>();
        else if(e[0]=="A"){++acks;assert(expected.count(e[2].get<uint64_t>()));}
        else if(e[0]=="M"){++moves;const auto& r=expected.at(e[2].get<uint64_t>());
          const auto& identity=identities.at(e[4].get<size_t>());
          assert(e[3]==r.write&&e[8]==r.sequence&&identity[3]==r.identity.task.data());
          assert(j["base_ns"].get<int64_t>()+e[15].get<int64_t>()==r.source_ns);
          assert(j["base_ns"].get<int64_t>()+e[16].get<int64_t>()==r.body_ns);
          assert(j["base_ns"].get<int64_t>()+e[17].get<int64_t>()==r.deadline_ns);
          assert(e[30]==r.measured.raw_stamp_ns&&e[33]==r.measured.sequence);
          if(r.attempt==21){next_task_nonzero=true;assert(identity[3]=="second_task"&&identity[7]==4&&e[22]!=0);}
          if(reasons.at(e[28].get<size_t>())=="manual_takeover"){final_zero=true;assert(e[22]==0);}
        }
      }
    }
    assert(moves==40&&acks==expected_acks&&offset==81&&event_index==81&&final_zero&&next_task_nonzero);
    const auto next=take(trace);assert(countKind(next,"M")==1&&next["snapshot"]==2);
    assert(next["events"].back()[2]==future.attempt);
  }
  // Malformed floating-point values and impossible single-record budgets are
  // explicit losses. They cannot strand later valid records or repeat counts
  // when a sink retries the same prepared page.
  {
    Trace trace;auto bad=move(trace,0);bad.raw_forward=std::numeric_limits<double>::quiet_NaN();trace.push(bad);
    auto huge=move(trace,1);copy(huge.identity.task,std::string(255,static_cast<char>(0xff)));
    copy(huge.identity.execution,std::string(255,static_cast<char>(0xff)));trace.push(huge);
    trace.push(move(trace,2));assert(trace.beginDrain());const auto first=trace.preparePage(2048);
    assert(first==trace.preparePage(2048));auto j=Json::parse(first);
    assert(j["drop"]["malformed"]==1&&j["drop"]["oversized"]==1&&countKind(j,"M")==1);
    assert(trace.commitPage()&&!trace.pending()&&!trace.commitPage());
    trace.push(move(trace,3));j=take(trace);
    assert(j["drop"]["malformed"]==1&&j["drop"]["oversized"]==1&&countKind(j,"M")==1);
  }
  // Capture cost benchmark is diagnostic, not an SDK/robot real-time claim.
  Trace trace;auto r=move(trace,0);const auto start=steadyNs();for(unsigned i=0;i<100000;++i)trace.push(r);
  const auto elapsed=steadyNs()-start;
  std::cout<<"PASS sdk trace: bounded pages preserve 40Hz+ACK; retry/commit; final zero; original identities/source; malformed/oversized; overflow/contention; blocked sink\n";
  std::cout<<"{\"seconds\":60,\"maximum_payload_bytes_per_page\":"<<maximum<<",\"mean_payload_bytes_per_second\":"<<total/60
    <<",\"record_bytes\":"<<sizeof(Record)<<",\"ring_capacity\":"<<Trace::capacity
    <<",\"producer_100000_ns\":"<<elapsed<<"}\n";
}
