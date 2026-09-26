#pragma once
#include <algorithm>
#include <cmath>

namespace faster_lio {
// Only localization opts into this policy. A soft gap is not a new IMU sample,
// a new local epoch, or proof of motion safety. Timestamp arithmetic uses doubles
// at Unix epoch magnitude, hence the same 1 us tolerance as the mapping guard.
struct LocalizationImuGap {
    bool accepted{false};
    bool degraded{false};
    double noise_scale{1.0};
};
inline LocalizationImuGap AssessLocalizationImuGap(double gap, double rotation,
                                                   double warning, double maximum,
                                                   double maximum_rotation) {
    if (!std::isfinite(gap) || gap <= 0 || !std::isfinite(rotation) || rotation < 0 ||
        !std::isfinite(warning) || warning <= 0 || !std::isfinite(maximum) ||
        maximum < warning || maximum > .1 || !std::isfinite(maximum_rotation) ||
        maximum_rotation <= 0) return {};
    constexpr double tolerance = 1e-6;
    if (gap > maximum + tolerance || (gap > warning + tolerance && rotation > maximum_rotation))
        return {};
    const bool degraded = gap > warning + tolerance;
    return {true, degraded, degraded ? std::min(100.0, std::pow(gap / warning, 2)) : 1.0};
}
}  // namespace faster_lio
