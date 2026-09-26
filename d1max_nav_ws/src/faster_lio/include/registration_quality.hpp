#pragma once

#include <cmath>

namespace faster_lio {
// Never let a rejected correspondence retain a previous iteration's residual.
// This preserves Faster-LIO's range-dependent gate, not a new map constraint.
inline bool SelectSurfaceResidual(float range, float distance, float& residual) {
    residual = 0.0F;
    if (!std::isfinite(range) || !std::isfinite(distance) ||
        range <= 0.0F || range <= 81.0F * distance * distance) return false;
    residual = distance;
    return true;
}
}  // namespace faster_lio
