#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Transform.h"

namespace d1max_localization::perception_rays
{
// Version 1: raw acquisition geometry, NOT a deskewed or map-frame ray cloud.
constexpr uint32_t kPointStep = 64;
// The production GridMap decoder has the same bounded acquisition budget.
// Reject an oversized acquisition before allocating/projecting it; never
// silently sample away collision evidence to fit a downstream queue.
constexpr uint32_t kMaxAcquisitionPoints = 100000;
enum class Sensor : uint16_t {Front = 0, Rear = 1};

struct Options
{
  // Safety consumes finite measured returns, not the LIO registration range.
  // This is a plausibility bound, NOT a declaration of free-space coverage.
  double min_range{0.0};
  double max_range{1000.0};
  double scan_period{0.10};
  double relative_timestamp_scale{1.0};
  uint32_t max_input_points{kMaxAcquisitionPoints};
};

struct Result
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::string error;
  uint32_t input_points{0};
  uint32_t invalid_geometry{0};
  uint32_t outside_range{0};
  uint32_t invalid_time{0};
  double latest_timestamp{0.0};
  explicit operator bool() const {return error.empty() && cloud.width > 0;}
};

inline bool validOptions(const Options & options)
{
  return std::isfinite(options.min_range) && options.min_range >= 0.0 &&
         std::isfinite(options.max_range) && options.max_range > options.min_range &&
         options.max_range <= 1000.0 && std::isfinite(options.scan_period) &&
         options.scan_period > 0.0 && options.scan_period <= 0.15 &&
         std::isfinite(options.relative_timestamp_scale) &&
         options.relative_timestamp_scale > 0.0 && options.max_input_points > 0 &&
         options.max_input_points <= kMaxAcquisitionPoints;
}

inline double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

// The same interpretation as the existing Airy -> LIO adapter. No fabricated
// scan-order times, arrival-time restamping, or independent clock fitting.
inline double decodePointTime(
  double raw, double header_seconds, double relative_scale, double scan_period)
{
  if (std::isfinite(raw)) {
    constexpr std::array<double, 4> scales{{1.0, 1e-3, 1e-6, 1e-9}};
    double best = header_seconds;
    double best_error = std::numeric_limits<double>::max();
    for (const double scale : scales) {
      const double candidate = raw * scale;
      if (candidate > 1e8) {
        const double error = std::abs(candidate - header_seconds);
        if (error < best_error) {
          best = candidate;
          best_error = error;
        }
      }
    }
    if (best_error < 86400.0) {return best;}
    const double relative = raw * relative_scale;
    if (relative >= 0.0 && relative <= scan_period * 2.0) {
      return header_seconds + relative;
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

namespace detail
{
template<typename T>
T read(const uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

template<typename T>
void write(uint8_t * data, T value)
{
  std::memcpy(data, &value, sizeof(T));
}

inline std::optional<uint32_t> field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  uint8_t type, uint32_t size)
{
  std::optional<uint32_t> result;
  for (const auto & item : cloud.fields) {
    if (item.name != name) {continue;}
    if (result || item.datatype != type || item.count != 1 ||
      item.offset > cloud.point_step || size > cloud.point_step - item.offset)
    {
      return std::nullopt;
    }
    result = item.offset;
  }
  return result;
}

inline sensor_msgs::msg::PointField outputField(
  const char * name, uint32_t offset, uint8_t type)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = type;
  result.count = 1;
  return result;
}

struct Point
{
  tf2::Vector3 position;
  float intensity;
  uint16_t ring;
  uint32_t source_index;
  double source_time;
  double raw_time;
};
}  // namespace detail

// Pure conversion: no ROS context, TF lookup, I/O, or clock observation. The
// supplied transform is exactly T_target_sensor used by the LIO adapter.
inline Result convert(
  const sensor_msgs::msg::PointCloud2 & input, Sensor sensor,
  const std::string & expected_source_frame, const std::string & target_frame,
  const tf2::Transform & transform, double shared_clock_offset,
  double ros_now, const Options & options = {})
{
  Result result;
  const auto reject = [&result](const char * reason) {
      result.error = reason;
      return result;
    };
  if (!validOptions(options)) {return reject("invalid_options");}
  if (sensor != Sensor::Front && sensor != Sensor::Rear) {return reject("invalid_sensor_id");}
  if (expected_source_frame.empty() || target_frame.empty() ||
    input.header.frame_id != expected_source_frame)
  {
    return reject("source_frame_mismatch");
  }
  if (!std::isfinite(shared_clock_offset) || !std::isfinite(ros_now) || ros_now <= 0.0 ||
    input.header.stamp.sec < 0 || input.header.stamp.nanosec >= 1000000000U)
  {
    return reject("invalid_clock");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(transform.getOrigin()[axis])) {return reject("invalid_transform");}
    for (int other = 0; other < 3; ++other) {
      if (!std::isfinite(transform.getBasis()[axis][other])) {
        return reject("invalid_transform");
      }
      const double dot = transform.getBasis()[axis].dot(transform.getBasis()[other]);
      if (std::abs(dot - (axis == other ? 1.0 : 0.0)) > 1e-6) {
        return reject("invalid_transform");
      }
    }
  }
  if (std::abs(transform.getBasis().determinant() - 1.0) > 1e-6) {
    return reject("invalid_transform");
  }
  const uint64_t count = static_cast<uint64_t>(input.width) * input.height;
  if (count == 0 || count > options.max_input_points) {return reject("point_count_limit");}
  result.input_points = static_cast<uint32_t>(count);
  if (input.is_bigendian || input.point_step < 26 ||
    input.row_step < static_cast<uint64_t>(input.width) * input.point_step ||
    input.data.size() < static_cast<uint64_t>(input.row_step) * input.height ||
    input.data.size() > 64U * 1024U * 1024U)
  {
    return reject("invalid_layout");
  }
  using Field = sensor_msgs::msg::PointField;
  const auto x = detail::field(input, "x", Field::FLOAT32, 4);
  const auto y = detail::field(input, "y", Field::FLOAT32, 4);
  const auto z = detail::field(input, "z", Field::FLOAT32, 4);
  const auto intensity = detail::field(input, "intensity", Field::FLOAT32, 4);
  const auto ring = detail::field(input, "ring", Field::UINT16, 2);
  const auto time = detail::field(input, "timestamp", Field::FLOAT64, 8);
  if (!x || !y || !z || !intensity || !ring || !time) {return reject("invalid_schema");}
  const double source_header = stampSeconds(input.header.stamp);
  const double min_squared = options.min_range * options.min_range;
  const double max_squared = options.max_range * options.max_range;
  std::vector<detail::Point> points;
  points.reserve(count);
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  uint32_t index = 0;
  for (uint32_t row = 0; row < input.height; ++row) {
    const auto * row_data = input.data.data() + static_cast<size_t>(row) * input.row_step;
    for (uint32_t column = 0; column < input.width; ++column, ++index) {
      const auto * data = row_data + static_cast<size_t>(column) * input.point_step;
      const tf2::Vector3 point(
        detail::read<float>(data + *x), detail::read<float>(data + *y),
        detail::read<float>(data + *z));
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z()))
      {
        ++result.invalid_geometry;
        continue;
      }
      if (point.length2() < 1e-12 || point.length2() < min_squared || point.length2() > max_squared) {
        ++result.outside_range;
        continue;
      }
      const double raw_time = detail::read<double>(data + *time);
      const double source_time = decodePointTime(
        raw_time, source_header, options.relative_timestamp_scale, options.scan_period);
      if (!std::isfinite(source_time)) {
        ++result.invalid_time;
        continue;
      }
      const tf2::Vector3 transformed = transform * point;
      // FLOAT32 wire coordinates must remain finite, including the origin.
      bool representable = true;
      for (int axis = 0; axis < 3; ++axis) {
        representable = representable && std::isfinite(static_cast<float>(transformed[axis])) &&
          std::isfinite(static_cast<float>(transform.getOrigin()[axis]));
      }
      if (!representable) {++result.invalid_geometry; continue;}
      points.push_back({transformed, detail::read<float>(data + *intensity),
        detail::read<uint16_t>(data + *ring), index, source_time, raw_time});
      first = std::min(first, source_time);
      last = std::max(last, source_time);
    }
  }
  if (points.empty()) {return reject("no_valid_points");}
  const double duration = last - first;
  const double base = first + shared_clock_offset;
  const double age = ros_now - (last + shared_clock_offset);
  // Match the existing LIO output gates; never turn missing/stale input into
  // a new observation. A fresh InputClock offset is required by the caller.
  if (duration < 0.001 || duration > 0.15 || age > 0.5 || age < -0.10 ||
    !std::isfinite(base) || base < 0.0 || base >= 2147483647.0)
  {
    return reject("invalid_or_stale_point_time");
  }
  auto & output = result.cloud;
  result.latest_timestamp = last + shared_clock_offset;
  const int64_t stamp_ns = static_cast<int64_t>(std::llround(base * 1e9));
  output.header.frame_id = target_frame;
  output.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  output.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  output.height = 1;
  output.width = static_cast<uint32_t>(points.size());
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = kPointStep;
  output.row_step = output.width * output.point_step;
  output.fields = {
    detail::outputField("x", 0, Field::FLOAT32),
    detail::outputField("y", 4, Field::FLOAT32),
    detail::outputField("z", 8, Field::FLOAT32),
    detail::outputField("intensity", 12, Field::FLOAT32),
    detail::outputField("origin_x", 16, Field::FLOAT32),
    detail::outputField("origin_y", 20, Field::FLOAT32),
    detail::outputField("origin_z", 24, Field::FLOAT32),
    detail::outputField("sensor_id", 28, Field::UINT16),
    detail::outputField("ring", 30, Field::UINT16),
    detail::outputField("offset_time", 32, Field::UINT32),
    detail::outputField("source_index", 36, Field::UINT32),
    detail::outputField("timestamp", 40, Field::FLOAT64),
    detail::outputField("source_timestamp", 48, Field::FLOAT64),
    detail::outputField("raw_timestamp", 56, Field::FLOAT64)};
  output.data.resize(output.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    auto * data = output.data.data() + i * kPointStep;
    for (int axis = 0; axis < 3; ++axis) {
      detail::write<float>(data + axis * 4, point.position[axis]);
      detail::write<float>(data + 16 + axis * 4, transform.getOrigin()[axis]);
    }
    detail::write<float>(data + 12, point.intensity);
    detail::write<uint16_t>(data + 28, static_cast<uint16_t>(sensor));
    detail::write<uint16_t>(data + 30, point.ring);
    detail::write<uint32_t>(data + 32, static_cast<uint32_t>(
        std::llround((point.source_time - first) * 1e9)));
    detail::write<uint32_t>(data + 36, point.source_index);
    detail::write<double>(data + 40, point.source_time + shared_clock_offset);
    detail::write<double>(data + 48, point.source_time);
    detail::write<double>(data + 56, point.raw_time);
  }
  return result;
}
}  // namespace d1max_localization::perception_rays
