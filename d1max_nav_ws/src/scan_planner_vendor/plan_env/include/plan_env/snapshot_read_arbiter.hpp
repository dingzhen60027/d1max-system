#pragma once
#include <array>
#include <chrono>

namespace scan_planner {
// Fixed two-request arbitration for the ONE slow native snapshot. This is a
// scheduler, not a source lease: granting a reader never refreshes evidence.
// The owning CollisionSnapshotPool serializes every call under its mutex.
class SnapshotReadArbiter {
public:
  using Clock=std::chrono::steady_clock;
  using Time=Clock::time_point;
  enum class Reader : unsigned { Solver=0, Admission=1 };
  void request(Reader reader,Time deadline,Time now) {
    const auto i=index(reader);pending_[i]=deadline>now;deadline_[i]=deadline;
  }
  void cancel(Reader reader) {pending_[index(reader)]=false;}
  bool grant(Reader reader,Time now,bool slot_available) {
    expire(now);
    const auto i=index(reader),other=1U-i;
    if(!slot_available||!pending_[i]||(pending_[other]&&next_!=i))return false;
    pending_[i]=false;next_=other;return true;
  }
  bool pending(Reader reader,Time now) {expire(now);return pending_[index(reader)];}
private:
  static unsigned index(Reader reader) {return static_cast<unsigned>(reader);}
  void expire(Time now) {
    for(unsigned i=0;i<2;++i)if(pending_[i]&&deadline_[i]<=now)pending_[i]=false;
  }
  std::array<bool,2> pending_{{false,false}};
  std::array<Time,2> deadline_{};
  unsigned next_{1};
};
} // namespace scan_planner
