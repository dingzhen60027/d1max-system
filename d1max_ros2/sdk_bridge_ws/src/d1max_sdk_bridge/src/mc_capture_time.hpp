#pragma once
#include <algorithm>
#include <cmath>

namespace d1monitor {
// The raw SDK epoch is not host time. Its receipt-anchored estimate may lie
// ahead of receipt within the declared live capture uncertainty. Keep both
// original numbers: the evidence lease always starts at the EARLIER one, so a
// positive mapping offset cannot buy extra freshness or prove a later capture.
inline bool mcCaptureTimeFresh(double source,double received,double now,
                               double age,double future_bound) {
  return std::isfinite(source)&&std::isfinite(received)&&std::isfinite(now)&&
    std::isfinite(age)&&std::isfinite(future_bound)&&source>0&&received>0&&
    age>=0&&future_bound>=0&&future_bound<=.25&&received<=now&&
    source-received<=future_bound&&now-std::min(source,received)<=age;
}

// Reporter consistency is deliberately separate from evidence freshness:
// arrival-rate diagnostics must not turn an approximate clock into exact time.
inline bool mcMappedTimeConsistent(double source,double received,double bound) {
  return std::isfinite(source)&&std::isfinite(received)&&std::isfinite(bound)&&
    bound>0&&std::abs(source-received)<bound;
}
}
