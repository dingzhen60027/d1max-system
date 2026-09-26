#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace faster_lio {

// A scan-admission check, not an IMU interpolator or filter reset mechanism.
// The caller owns synchronization and must check BEFORE IMU propagation.
enum class ImuContinuityMode { Disabled, Observe, FailClosed };

inline ImuContinuityMode ParseImuContinuityMode(const std::string& value) {
    if (value == "disabled") return ImuContinuityMode::Disabled;
    if (value == "observe") return ImuContinuityMode::Observe;
    if (value == "fail_closed") return ImuContinuityMode::FailClosed;
    throw std::invalid_argument("IMU continuity mode must be disabled, observe, or fail_closed");
}

struct ImuContinuityConfig {
    ImuContinuityMode mode{ImuContinuityMode::Disabled};
    double warning_gap_sec{0.015};
    double maximum_gap_sec{0.030};
};

struct ImuContinuitySample {
    double stamp{0.0};
    std::array<double, 3> acceleration{};
    std::array<double, 3> angular_velocity{};
};

struct ImuContinuityResult {
    // accepted authorizes admission only, NOT that a resulting pose is valid.
    // Observe mode intentionally admits unhealthy input for explicit diagnostics.
    bool accepted{true};
    bool checked{false};
    bool healthy{true};
    bool latched{false};
    std::string reason{"disabled"};
    double begin{0.0};
    double end{0.0};
    double maximum_gap_sec{0.0};
    double offending_begin{0.0};
    double offending_end{0.0};
    std::size_t sample_count{0};
    std::size_t warning_intervals{0};
};

// ImuProcess initializes from IMU samples without integrating a lidar interval.
// Its first propagation starts at the final initialization sample; only after
// that does last_lidar_end_time_ represent the actual state propagation epoch.
inline double MappingImuContinuityBegin(bool has_propagated, double previous_scan_end,
                                        double first_support_stamp) {
    return has_propagated ? previous_scan_end : first_support_stamp;
}

class ImuContinuityGuard {
  public:
    explicit ImuContinuityGuard(ImuContinuityConfig config = {}) : config_(config) {
        if (!std::isfinite(config_.warning_gap_sec) || config_.warning_gap_sec <= 0.0 ||
            !std::isfinite(config_.maximum_gap_sec) || config_.maximum_gap_sec <= 0.0 ||
            config_.maximum_gap_sec > 1.0 || config_.warning_gap_sec > config_.maximum_gap_sec) {
            throw std::invalid_argument("Invalid IMU continuity gap thresholds");
        }
    }

    const ImuContinuityConfig& config() const { return config_; }
    bool faulted() const { return faulted_; }
    const ImuContinuityResult& fault() const { return fault_; }

    // begin is the previous propagated scan end (or the first scan start), NOT
    // always the current scan start: missing lidar frames do not erase IMU gaps.
    // samples must include the latest real sample at/before begin and the first
    // real sample at/after end. The right support sample need not be integrated
    // by the caller; it verifies that the final extrapolation spans a bounded dt.
    // Keep original timestamps and values. Equal values with advancing stamps
    // are allowed; that does not assert independent physical sensor sampling.
    ImuContinuityResult Check(double begin, double end,
                              const std::vector<ImuContinuitySample>& samples) {
        if (faulted_) return fault_;
        ImuContinuityResult result;
        result.begin = begin;
        result.end = end;
        result.sample_count = samples.size();
        if (config_.mode == ImuContinuityMode::Disabled) return result;
        result.checked = true;
        result.reason = "ok";
        if (!std::isfinite(begin) || !std::isfinite(end) || end <= begin) {
            return Reject(result, "invalid_scan_interval");
        }
        if (samples.size() < 2) return Reject(result, "insufficient_imu");
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto& sample = samples[i];
            bool finite = std::isfinite(sample.stamp);
            for (std::size_t axis = 0; axis < 3; ++axis) {
                finite = finite && std::isfinite(sample.acceleration[axis]) &&
                         std::isfinite(sample.angular_velocity[axis]);
            }
            if (!finite) {
                result.offending_begin = result.offending_end = sample.stamp;
                return Reject(result, "imu_nonfinite");
            }
            if (i > 0 && sample.stamp <= samples[i - 1].stamp) {
                result.offending_begin = samples[i - 1].stamp;
                result.offending_end = sample.stamp;
                return Reject(result, "imu_nonmonotonic");
            }
        }
        // 1 us only covers double-precision ROS epoch timestamp rounding, not a
        // sensor sampling period. A real missing bracket cannot be excused.
        constexpr double kTimestampTolerance = 1e-6;
        if (samples.front().stamp > begin + kTimestampTolerance) {
            result.offending_begin = begin;
            result.offending_end = samples.front().stamp;
            return Reject(result, "imu_start_uncovered");
        }
        if (samples.back().stamp < end - kTimestampTolerance) {
            result.offending_begin = samples.back().stamp;
            result.offending_end = end;
            return Reject(result, "imu_end_uncovered");
        }
        bool excessive_gap = false;
        for (std::size_t i = 1; i < samples.size(); ++i) {
            const double left = samples[i - 1].stamp, right = samples[i].stamp;
            // Ignore intervals fully outside the integration interval, but do
            // not clip a straddling gap to the scan boundary and hide its size.
            if (right <= begin || left >= end) continue;
            const double gap = right - left;
            result.maximum_gap_sec = std::max(result.maximum_gap_sec, gap);
            if (gap > config_.warning_gap_sec + kTimestampTolerance) ++result.warning_intervals;
            if (gap > config_.maximum_gap_sec + kTimestampTolerance && !excessive_gap) {
                excessive_gap = true;
                result.offending_begin = left;
                result.offending_end = right;
            }
        }
        if (excessive_gap) return Reject(result, "imu_gap");
        return result;
    }

    // For a clock regression detected in a callback before the original queue
    // is cleared. The caller must serialize this with Check. There is purposely
    // no reset()/recover() method: a fault requires a NEW explicitly named run,
    // rather than silently splicing an unobservable interval into the same map.
    ImuContinuityResult ExternalFault(const std::string& reason, double previous, double current) {
        if (faulted_) return fault_;
        ImuContinuityResult result;
        result.begin = previous;
        result.end = current;
        result.offending_begin = previous;
        result.offending_end = current;
        if (config_.mode == ImuContinuityMode::Disabled) return result;
        result.checked = true;
        return Reject(result, reason);
    }

  private:
    ImuContinuityResult Reject(ImuContinuityResult result, const std::string& reason) {
        result.healthy = false;
        result.reason = reason;
        if (config_.mode == ImuContinuityMode::FailClosed) {
            result.accepted = false;
            result.latched = true;
            faulted_ = true;
            fault_ = result;
        }
        return result;
    }

    const ImuContinuityConfig config_;
    bool faulted_{false};
    ImuContinuityResult fault_;
};

}  // namespace faster_lio
