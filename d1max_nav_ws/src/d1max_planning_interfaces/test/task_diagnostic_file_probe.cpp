#include "d1max_planning_interfaces/bounded_diagnostic_file.hpp"
#include <cassert>
#include <iostream>
#include <map>
#include <iterator>

using Sink=d1max_diagnostics::BoundedTaskDiagnosticFile;
using Files=std::map<std::string,std::string>;
Files records(const std::filesystem::path& root,std::string_view component) {
  Files result;
  for(const auto& dir:std::filesystem::directory_iterator(root))
    if(dir.is_directory()&&dir.path().filename().string().find(component)==0)
      for(const auto& file:std::filesystem::directory_iterator(dir.path()))
        if(file.path().extension()==".jsonl") {
          std::ifstream input(file.path());
          result[file.path().string()]={std::istreambuf_iterator<char>(input),{}};
        }
  return result;
}
int main(int argc,char**argv) {
  assert(argc==2);const std::filesystem::path root(argv[1]);
  Sink sink("retention",root,512,512);
  d1max_diagnostics::BoundedDiagnosticFile old("legacy",root,512);
  assert(sink.directory().empty());
  const auto first=Sink::taskKey("session","task1");
  assert(first!=Sink::taskKey("sess","iontask1"));
  for(int i=0;i<100;++i) {
    const auto batch="{\"seq\":"+std::to_string(i)+",\"source_ns\":123456789}";
    assert(sink.write(batch,first,true,1000000000LL+i));assert(old.write(batch));
  }
  assert(sink.write("{\"stop_ack\":true}",first,false,2000000000LL));
  assert(old.write("{\"stop_ack\":true}"));
  const auto frozen=records(sink.directory(),"task0-");assert(!frozen.empty());
  // Forty minutes of idle callbacks, with both retained and cleared task IDs.
  // They must not evict or rewrite a byte of the completed movement trace.
  for(int i=0;i<4800;++i) {
    const auto batch="{\"idle\":true,\"seq\":"+std::to_string(i)+"}";
    assert(sink.write(batch,i%2?first:std::string{},false,12000000000LL+i*500000000LL));
    assert(old.write(batch));
  }
  const auto legacy=records(root,"legacy-");assert(!legacy.empty());
  for(const auto& item:legacy)assert(item.second.find("stop_ack")==std::string::npos);
  assert(records(sink.directory(),"task0-")==frozen);
  assert(sink.evictedTasks()==0);
  // A late inactive observation for an unseen task cannot allocate/evict a task bucket.
  assert(sink.write("{\"late\":true}",Sink::taskKey("session","unseen"),false,3000000000000LL));
  assert(sink.write("{\"late_active\":true}",Sink::taskKey("session","out-of-order"),true,2000000000LL));
  assert(sink.write("{\"late_stop\":true}",first,false,2000000000LL));
  assert(records(sink.directory(),"task0-")==frozen);
  assert(sink.evictedTasks()==0);
  for(int i=2;i<=4;++i)assert(sink.write("{\"task\":"+std::to_string(i)+"}",
      Sink::taskKey("session","task"+std::to_string(i)),true,4000000000000LL+i));
  assert(records(sink.directory(),"task0-")==frozen);
  const auto task2=records(sink.directory(),"task1-");
  assert(sink.write("{\"task\":5}",Sink::taskKey("session","task5"),true,5000000000000LL));
  assert(sink.evictedTasks()==1);
  assert(records(sink.directory(),"task0-")!=frozen);
  assert(records(sink.directory(),"task1-")==task2);
  // Rapid new goals reuse exactly four private buckets, not unbounded archives.
  for(int i=6;i<30;++i)assert(sink.write("{\"task\":"+std::to_string(i)+"}",
      Sink::taskKey("session","task"+std::to_string(i)),true,6000000000000LL+i));
  assert(sink.evictedTasks()==25);
  std::size_t dirs=0,bytes=0;
  for(const auto& path:std::filesystem::recursive_directory_iterator(sink.directory())) {
    dirs+=path.is_directory();if(path.path().extension()==".jsonl")bytes+=path.file_size();
  }
  assert(dirs==5&&bytes<=5*4*512);
  assert(sink.failedWrites()==0&&!sink.disabled());
  assert(!sink.write("{}",std::string(1025,'x'),true,1));
  assert(sink.rejectedBatches()==1);
  Sink retry("retention",root,512,512);
  assert(retry.write("{}",Sink::taskKey("session","../../not-a-path"),true,1));
  assert(retry.directory()!=sink.directory());
  Sink invalid("../escape",root);
  assert(!invalid.write("{}","task",true,1)&&invalid.disabled());
  std::cout<<sink.directory().string()<<'\n';
}
