#pragma once
// Passive, bounded evidence only. This queue never supplies motion or source
// freshness. Producers do not format, allocate, wait for a consumer or do I/O.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

namespace d1monitor::command_trace {
inline int64_t wallNs() noexcept {return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::system_clock::now().time_since_epoch()).count();}
inline int64_t steadyNs() noexcept {return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::steady_clock::now().time_since_epoch()).count();}
template<size_t N> bool copy(std::array<char,N>& dst,std::string_view text) noexcept {
  const auto n=text.size()<N?text.size():N-1;if(n)std::memcpy(dst.data(),text.data(),n);
  dst[n]=0;return text.size()>=N;
}
struct Identity {
  std::array<char,128> session{};
  std::array<char,256> task{},execution{};
  std::array<char,64> sdk{};
  uint64_t epoch=0,arm=0;
  bool truncated=false;
  bool operator==(const Identity& o)const noexcept {
    return !truncated&&!o.truncated&&session==o.session&&task==o.task&&execution==o.execution&&sdk==o.sdk&&
      epoch==o.epoch&&arm==o.arm&&truncated==o.truncated;
  }
};
struct Measurement {
  uint64_t raw_stamp_ns=0,sequence=0,generation=0;
  int64_t received_ns=0,arrival_steady_ns=0;
  float forward=0.f,yaw=0.f;
};
struct Record {
  enum class Kind {Move,Ack};Kind kind=Kind::Move;
  uint64_t event=0,attempt=0,write=0,sequence=0,cause_sequence=0,commit=0;
  uint64_t last_expiry_sequence=0,expiry_count=0;
  int64_t curve=-1,selected_ns=0,begin_ns=0,end_ns=0,begin_steady_ns=0,end_steady_ns=0;
  int64_t source_ns=0,body_ns=0,deadline_ns=0,expiry_ns=0;
  Identity identity;
  Measurement measured;
  std::array<char,96> reason{};
  std::array<char,32> error_category{};
  double raw_forward=0.,raw_lateral=0.,raw_yaw=0.;
  float forward=0.f,lateral=0.f,yaw=0.f;
  int error=0;
  bool selected=false,exception=false,ack_applied=false,truncated=false,new_expiry=false;
};
static_assert(std::is_trivially_copyable<Record>::value,"producer record must not own dynamic memory");

class Trace {
public:
  // Leave 512 bytes for the common file sink's timestamp/rotation envelope.
  static constexpr size_t capacity=256,byte_budget=7680;
  uint64_t nextAttempt() noexcept {return attempts_.fetch_add(1,std::memory_order_relaxed)+1;}
  void push(Record record) noexcept {
    record.event=events_.fetch_add(1,std::memory_order_relaxed)+1;
    try {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(!lock.owns_lock()){contention_.fetch_add(1,std::memory_order_relaxed);return;}
      if(count_==capacity){head_=(head_+1)%capacity;--count_;overflow_.fetch_add(1,std::memory_order_relaxed);}
      records_[(head_+count_)%capacity]=record;++count_;
    } catch(...) {errors_.fetch_add(1,std::memory_order_relaxed);}
  }
  void ack(uint64_t attempt,uint64_t write,int64_t at,int64_t steady,
      int error,std::string_view category,bool applied) noexcept {
    Record record;record.kind=Record::Kind::Ack;record.attempt=attempt;record.write=write;
    record.end_ns=at;record.end_steady_ns=steady;record.error=error;record.ack_applied=applied;
    record.truncated=copy(record.error_category,category);push(record);
  }
  // Consumer-only fixed snapshot. A failed page remains pending while producers
  // keep using their original bounded try-lock ring. A caller drains at most
  // one snapshot per ordinary wake, never chases an unbounded incoming stream.
  bool beginDrain() noexcept {
    if(pending_active_)return true;
    try {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(!lock.owns_lock())return false;
      const Counters counters{events_.load(),contention_.load(),overflow_.load(),errors_.load()};
      if(!count_&&counters==reported_&&format_failures_==reported_format_failures_)return false;
      pending_count_=count_;
      for(size_t i=0;i<count_;++i)pending_[i]=records_[(head_+i)%capacity];
      head_=count_=0;
      pending_counters_=counters;pending_cursor_=pending_page_index_=0;
      ++snapshot_;pending_active_=true;return true;
    }catch(...){++format_failures_;return false;}
  }
  bool pending()const noexcept{return pending_active_;}

  // Called ONLY by the passive consumer, after beginDrain(). Each <=8 KiB
  // page owns its dictionaries. Budget exhaustion ends a page, not a record.
  // The cached page is retried byte-for-byte until its sink reports success.
  const std::string& preparePage(size_t budget=byte_budget) noexcept {
    if(!page_.empty()||!pending_active_)return page_;
    if(budget>byte_budget)budget=byte_budget;
    if(budget<1024)return page_;
    try {
      const auto* first_record=pending_cursor_<pending_count_?&pending_[pending_cursor_]:nullptr;
      const int64_t base=first_record?(first_record->kind==Record::Kind::Move?first_record->selected_ns:first_record->end_ns):wallNs();
      const int64_t steady=first_record?(first_record->kind==Record::Kind::Move?first_record->begin_steady_ns:first_record->end_steady_ns):steadyNs();
      std::ostringstream out;out.imbue(std::locale::classic());
      out<<std::setprecision(9)<<"{\"v\":1,\"source\":\"sdk_move\",\"text_encoding\":\"byte_escaped\",\"base_ns\":"<<base
        <<",\"steady_ns\":"<<steady<<",\"events\":[";
      bool first=true;std::array<Identity,capacity> identities{};size_t identity_count=0;
      std::array<std::array<char,96>,capacity> reasons{};size_t reason_count=0;
      const auto relative=[](int64_t value,int64_t origin){return value?std::to_string(value-origin):"null";};
      size_t next=pending_cursor_,emitted=0,malformed=0,oversized=0;
      for(;next<pending_count_;++next) {
        const auto& r=pending_[next];
        if(!encodable(r)){++malformed;continue;}
        std::ostringstream row;row.imbue(std::locale::classic());row<<std::setprecision(9);
        size_t identity=0,reason=0;bool new_identity=false,new_reason=false;
        if(r.kind==Record::Kind::Move){
          for(;identity<identity_count;++identity)if(identities[identity]==r.identity)break;
          new_identity=identity==identity_count;
          if(new_identity){row<<"[\"I\","<<identity<<','<<quote(text(r.identity.session))<<','
            <<quote(text(r.identity.task))<<','<<quote(text(r.identity.execution))<<','<<quote(text(r.identity.sdk))
            <<','<<r.identity.epoch<<','<<r.identity.arm<<','<<r.identity.truncated<<"],";}
          for(;reason<reason_count;++reason)if(reasons[reason]==r.reason)break;
          new_reason=reason==reason_count;
          if(new_reason)row<<"[\"R\","<<reason<<','<<quote(text(r.reason))<<"],";
          row<<"[\"M\","<<r.event<<','<<r.attempt<<','<<r.write<<','<<identity<<','<<r.curve<<','<<r.commit<<','
            <<r.selected<<','<<r.sequence<<','<<r.cause_sequence<<','<<relative(r.selected_ns,base)<<','
            <<relative(r.begin_ns,base)<<','<<relative(r.end_ns,base)<<','<<relative(r.begin_steady_ns,steady)<<','
            <<relative(r.end_steady_ns,steady)<<','<<relative(r.source_ns,base)<<','<<relative(r.body_ns,base)<<','
            <<relative(r.deadline_ns,base)<<','<<relative(r.expiry_ns,base)<<','
            <<r.raw_forward<<','<<r.raw_lateral<<','<<r.raw_yaw<<','<<r.forward<<','<<r.lateral<<','<<r.yaw<<','
            <<r.error<<','<<quote(r.error?text(r.error_category):std::string_view{})<<','<<r.exception<<','<<reason<<','<<r.truncated<<','
            <<r.measured.raw_stamp_ns<<','<<relative(r.measured.received_ns,base)<<','
            <<relative(r.measured.arrival_steady_ns,steady)<<','<<r.measured.sequence<<','<<r.measured.generation<<','
            <<r.measured.forward<<','<<r.measured.yaw<<','<<r.last_expiry_sequence<<','<<r.expiry_count<<','<<r.new_expiry<<']';
        }else row<<"[\"A\","<<r.event<<','<<r.attempt<<','<<r.write<<','<<relative(r.end_ns,base)<<','
          <<relative(r.end_steady_ns,steady)<<','<<r.error<<','<<quote(r.error?text(r.error_category):std::string_view{})<<','<<r.ack_applied<<','<<r.truncated<<']';
        if(!row)throw std::runtime_error("trace_row_encoding_failed");
        const auto encoded=row.str();
        // Reserve metadata and counter digits. One impossible record is an
        // explicit loss; it must not strand every subsequent diagnostic page.
        if(static_cast<size_t>(out.tellp())+encoded.size()+768>budget){
          if(emitted)break;
          ++oversized;continue;
        }
        if(!first)out<<',';
        first=false;out<<encoded;++emitted;
        if(new_identity)identities[identity_count++]=r.identity;
        if(new_reason)reasons[reason_count++]=r.reason;
      }
      // observed is a production high-water observation, NOT a contiguous
      // write cursor: event IDs are assigned before try_lock and ACK may be
      // observed before Move. Only this snapshot's array offset is committed.
      out<<"],\"snapshot\":"<<snapshot_<<",\"page\":"<<pending_page_index_
        <<",\"record_offset\":"<<pending_cursor_<<",\"record_count\":"<<next-pending_cursor_
        <<",\"emitted_records\":"<<emitted<<",\"snapshot_records\":"<<pending_count_
        <<",\"remaining_records\":"<<pending_count_-next
        <<",\"observed\":"<<pending_counters_.observed<<",\"format_failures\":"<<format_failures_
        <<",\"drop\":{\"contention\":"<<pending_counters_.contention
        <<",\"overflow\":"<<pending_counters_.overflow<<",\"budget\":0,\"error\":"<<pending_counters_.error
        <<",\"malformed\":"<<malformed_+malformed<<",\"oversized\":"<<oversized_+oversized<<"}}";
      if(!out)throw std::runtime_error("trace_page_encoding_failed");
      auto encoded=out.str();
      if(encoded.size()>budget)throw std::runtime_error("trace_page_metadata_exceeded_budget");
      prepared_end_=next;prepared_malformed_=malformed;prepared_oversized_=oversized;
      prepared_format_failures_=format_failures_;page_=std::move(encoded);return page_;
    }catch(...) {++format_failures_;page_.clear();return page_;}
  }
  // Call only after the file sink accepts the complete cached page. Failed
  // writes do not consume pending records or renumber events/attempts/commands.
  bool commitPage() noexcept {
    if(page_.empty())return false;
    pending_cursor_=prepared_end_;++pending_page_index_;
    malformed_+=prepared_malformed_;oversized_+=prepared_oversized_;
    reported_=pending_counters_;reported_format_failures_=prepared_format_failures_;
    if(pending_cursor_==pending_count_)pending_active_=false;
    page_.clear();return true;
  }
private:
  struct Counters {
    uint64_t observed=0,contention=0,overflow=0,error=0;
    bool operator==(const Counters& o)const noexcept{return observed==o.observed&&
      contention==o.contention&&overflow==o.overflow&&error==o.error;}
  };
  static bool encodable(const Record& r)noexcept {
    if(r.kind==Record::Kind::Ack)return true;
    return r.kind==Record::Kind::Move&&std::isfinite(r.raw_forward)&&std::isfinite(r.raw_lateral)&&
      std::isfinite(r.raw_yaw)&&std::isfinite(r.forward)&&std::isfinite(r.lateral)&&std::isfinite(r.yaw)&&
      std::isfinite(r.measured.forward)&&std::isfinite(r.measured.yaw);
  }
  template<size_t N> static std::string_view text(const std::array<char,N>& value)noexcept {
    size_t size=0;while(size<N&&value[size])++size;return {value.data(),size};
  }
  static std::string quote(std::string_view text){
    std::ostringstream out;out<<'"';for(const auto c:text){
      if(c=='"'||c=='\\')out<<'\\'<<c;
      // Fixed capture arrays may cut through UTF-8, and SDK strings need not
      // be valid UTF-8 at all. Preserve every byte as a reversible JSON scalar
      // rather than emit malformed JSON or alias distinct bytes via U+FFFD.
      else if(static_cast<unsigned char>(c)<32||static_cast<unsigned char>(c)>=127)
        out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(static_cast<unsigned char>(c))<<std::dec;
      else out<<c;
    }out<<'"';return out.str();
  }
  friend struct TraceTestAccess;
  std::mutex mutex_;
  std::array<Record,capacity> records_{};size_t head_=0,count_=0;
  std::atomic<uint64_t>attempts_{0},events_{0},contention_{0},overflow_{0},errors_{0};
  // All fields below are owned exclusively by the existing diagnostic worker.
  std::array<Record,capacity> pending_{};
  size_t pending_count_=0,pending_cursor_=0,pending_page_index_=0,prepared_end_=0,
    prepared_malformed_=0,prepared_oversized_=0;
  uint64_t snapshot_=0,malformed_=0,oversized_=0,format_failures_=0,
    reported_format_failures_=0,prepared_format_failures_=0;
  Counters pending_counters_,reported_;
  bool pending_active_=false;
  std::string page_;
};
} // namespace d1monitor::command_trace
