#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace d1max_trajectory_tracker {

// A ROS clock observation keeps its integer source. The double conversion is
// only for control arithmetic and legacy offline callers that supply seconds.
// Production callers construct this from the clock's original nanoseconds.
struct SourceTime {
  SourceTime(double seconds) : seconds_(seconds) {}
  static SourceTime fromNanoseconds(std::int64_t ns) {
    SourceTime result(static_cast<double>(ns)*1e-9);
    result.nanoseconds_=ns;
    return result;
  }
  operator double() const {return seconds_;}
  std::int64_t nanoseconds() const {
    if(nanoseconds_!=0)return nanoseconds_;
    const long double ns=static_cast<long double>(seconds_)*1000000000.L;
    if(!std::isfinite(seconds_)||ns<=0.||ns>=std::numeric_limits<std::int64_t>::max())return 0;
    return static_cast<std::int64_t>(std::llround(ns));
  }
private:
  double seconds_;
  std::int64_t nanoseconds_{0};
};

inline std::int64_t durationNs(double seconds) {
  return static_cast<std::int64_t>(std::llround(seconds*1e9));
}
inline std::int64_t originalSourceNs(std::int64_t original,double seconds) {
  return original>0?original:SourceTime(seconds).nanoseconds();
}
inline bool sourceFresh(SourceTime now,std::int64_t source,double limit,double future=0.) {
  const auto ns=now.nanoseconds();
  return ns>0&&source>0&&source-ns<=durationNs(future)&&ns-source<=durationNs(limit);
}
inline double sourceDeltaSeconds(std::int64_t newer,std::int64_t older) {
  return static_cast<double>(newer-older)*1e-9;
}
}  // namespace d1max_trajectory_tracker
