#pragma once

#include <cmath>

namespace faster_lio {
enum class LocalizationRegistrationDisposition { Accepted, WeakGeometry, NonfiniteFilter };

// A finite posterior with too few accepted surface constraints is an unusable
// measurement, not proof that the filter/clock has irreversibly broken. Keep
// inspecting later scans, but do not publish this posterior or add its points
// to the map. Numeric corruption is a different, genuinely hard disposition.
// This is the Run worker's exact state/covariance decision; source leases and
// task recovery remain with their existing consumers, not another age gate.
template<class State, class Covariance>
inline bool FiniteLocalizationFilterState(const State& state, const Covariance& covariance) {
    return state.pos.allFinite() && state.vel.allFinite() && state.rot.coeffs().allFinite() &&
        state.bg.allFinite() && state.ba.allFinite() && state.offset_T_L_I.allFinite() &&
        state.offset_R_L_I.coeffs().allFinite() && std::isfinite(state.grav[0]) &&
        std::isfinite(state.grav[1]) && std::isfinite(state.grav[2]) && covariance.allFinite();
}
template<class State, class Covariance>
inline LocalizationRegistrationDisposition AssessLocalizationRegistration(
    int features, int required, const State& state, const Covariance& covariance) {
    if (!FiniteLocalizationFilterState(state,covariance))
        return LocalizationRegistrationDisposition::NonfiniteFilter;
    return features < required ? LocalizationRegistrationDisposition::WeakGeometry
                               : LocalizationRegistrationDisposition::Accepted;
}

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
