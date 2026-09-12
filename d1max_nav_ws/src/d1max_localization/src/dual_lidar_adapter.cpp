#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "d1max_localization/input_clock.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr uint32_t kOutputPointStep = 24;
constexpr uint8_t kLivoxValidTag = 0x10;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secondsToStamp(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

template<typename T>
T readValue(const uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

template<typename T>
void writeValue(uint8_t * data, const T & value)
{
  std::memcpy(data, &value, sizeof(T));
}

struct ParsedPoint
{
  float x{};
  float y{};
  float z{};
  float intensity{};
  uint16_t ring{};
  double time_seconds{};
};

struct CloudFields
{
  uint32_t x{};
  uint32_t y{};
  uint32_t z{};
  uint32_t intensity{};
  uint32_t ring{};
  uint32_t timestamp{};
};
}  // namespace

class DualLidarAdapter final : public rclcpp::Node
{
public:
  DualLidarAdapter()
  : Node("dual_lidar_adapter")
  {
    const auto clock_mode=declare_parameter<std::string>("input_clock_mode","strict");
    if(clock_mode!="strict" && clock_mode!="estimate_shared_epoch")throw std::invalid_argument("invalid input_clock_mode");
    const auto clock_samples=declare_parameter<int>("input_clock_samples",100);
    if(clock_samples<2 || clock_samples>10000)throw std::invalid_argument("invalid input_clock_samples");
    input_clock_=std::make_unique<d1max_localization::InputClock>(clock_mode=="estimate_shared_epoch",clock_samples,declare_parameter<double>("input_clock_min_span",1.0));
    clock_diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    clock_timer_=create_wall_timer(std::chrono::milliseconds(500),[this]{publishClockStatus();});
    replay_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr){
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if(input_clock_->approximate())input_clock_->disableForReplay();
    });
    front_topic_ = declare_parameter<std::string>("front_topic", "/front_lidar");
    rear_topic_ = declare_parameter<std::string>("rear_topic", "/rear_lidar");
    lidar_mode_ = declare_parameter<std::string>("lidar_mode", "dual");
    if (lidar_mode_ != "dual" && lidar_mode_ != "front" && lidar_mode_ != "rear") {
      throw std::invalid_argument("lidar_mode must be dual, front, or rear");
    }
    input_imu_topic_ =
      declare_parameter<std::string>("input_imu_topic", "/front_lidar/imu");
    output_cloud_topic_ =
      declare_parameter<std::string>("output_cloud_topic", "/d1max/localization/points");
    output_livox_topic_ = declare_parameter<std::string>(
      "output_livox_topic", "/d1max/localization/livox_points");
    output_imu_topic_ =
      declare_parameter<std::string>("output_imu_topic", "/d1max/localization/imu");
    target_frame_ = declare_parameter<std::string>("target_frame", "rslidar_head");
    front_frame_ = declare_parameter<std::string>("front_frame", "rslidar_head");
    rear_frame_ = declare_parameter<std::string>("rear_frame", "rslidar_tail");
    const auto load_transform = [this](const std::string & name) {
        const auto xyz = declare_parameter<std::vector<double>>(name + "_translation", {0.0, 0.0, 0.0});
        const auto q = declare_parameter<std::vector<double>>(name + "_rotation", {0.0, 0.0, 0.0, 1.0});
        if (xyz.size() != 3 || q.size() != 4) throw std::invalid_argument("invalid calibration dimensions");
        for (const auto v : xyz) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite calibration");
        for (const auto v : q) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite calibration");
        tf2::Quaternion rotation(q[0], q[1], q[2], q[3]);
        if (std::abs(rotation.length()-1.0) > 0.001) throw std::invalid_argument("invalid calibration quaternion");
        rotation.normalize();
        return tf2::Transform(rotation, tf2::Vector3(xyz[0],xyz[1],xyz[2]));
      };
    front_transform_ = load_transform("front");
    rear_transform_ = front_transform_ * load_transform("rear_to_front");
    max_sync_delta_sec_ = declare_parameter<double>("max_sync_delta_sec", 0.08);
    front_fallback_=declare_parameter("front_fallback",false);
    front_wait_sec_=declare_parameter("front_wait_sec",.03);
    if(!std::isfinite(front_wait_sec_)||front_wait_sec_<.005||front_wait_sec_>.08)
      throw std::invalid_argument("Invalid bounded rear lidar wait");
    if(front_fallback_ && lidar_mode_=="dual"){
      fallback_timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{
        if(front_cloud_ && steadySeconds()-front_received_at_>=front_wait_sec_){
          ++front_only_attempts_;publishSingle(*front_cloud_);front_cloud_.reset();
        }
      });
    }
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.15);
    scan_period_sec_ = declare_parameter<double>("scan_period_sec", 0.10);
    relative_timestamp_scale_ =
      declare_parameter<double>("relative_timestamp_scale", 1.0);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 120.0);
    rear_ring_offset_ = declare_parameter<int>("rear_ring_offset", 96);
    imu_acceleration_scale_ =
      declare_parameter<double>("imu_acceleration_scale", 9.80665);

    auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, output_qos);
    livox_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_livox_topic_, output_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, rclcpp::QoS(512).reliable());

    if (lidar_mode_ != "rear") {
      front_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        front_topic_, input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (lidar_mode_ == "front") {
            publishSingle(*msg);
          } else {
            front_cloud_ = std::move(msg);
            front_received_at_=steadySeconds();
            tryPublishPair();
          }
        });
    }
    if (lidar_mode_ != "front") {
      rear_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        rear_topic_, input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (lidar_mode_ == "rear") {
            publishSingle(*msg);
          } else {
            rear_cloud_ = std::move(msg);
            tryPublishPair();
          }
        });
    }
    imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;imu_options.callback_group=imu_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, rclcpp::SensorDataQoS().keep_last(512),
      std::bind(&DualLidarAdapter::imuCallback, this, std::placeholders::_1),imu_options);

    if (lidar_mode_ == "dual") {
      RCLCPP_INFO(
        get_logger(),
        "Merging %s and %s into %s; output cloud=%s, Livox=%s, corrected IMU=%s",
        front_topic_.c_str(), rear_topic_.c_str(), target_frame_.c_str(),
        output_cloud_topic_.c_str(), output_livox_topic_.c_str(), output_imu_topic_.c_str());
    } else {
      const auto & topic = lidar_mode_ == "front" ? front_topic_ : rear_topic_;
      RCLCPP_INFO(
        get_logger(), "Single-LiDAR mode=%s, input=%s, target=%s, IMU=%s, output=%s",
        lidar_mode_.c_str(), topic.c_str(), target_frame_.c_str(),
        input_imu_topic_.c_str(), output_cloud_topic_.c_str());
    }
  }

private:
  static double steadySeconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void publishClockStatus(){
    diagnostic_msgs::msg::DiagnosticArray report;report.header.stamp=now();
    {
    std::lock_guard<std::mutex> lock(clock_mutex_);
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="d1max_localization/input_clock";
    const auto offset=input_clock_->offset(steadySeconds());
    s.level=input_clock_->faulted()?2:offset?(input_clock_->approximate()?1:0):1;s.message=input_clock_->state();
    const auto add=[&s](const std::string& k,const std::string& v){diagnostic_msgs::msg::KeyValue p;p.key=k;p.value=v;s.values.push_back(p);};
    add("ready",offset?"true":"false");add("approximate",input_clock_->approximate()?"true":"false");
    add("offset_seconds",offset?std::to_string(*offset):"unknown");add("samples",std::to_string(input_clock_->count()));
    add("imu_received",std::to_string(imu_received_));add("imu_rejected",std::to_string(imu_rejected_));
    add("imu_max_source_gap_sec",std::to_string(imu_max_source_gap_));
    add("front_fallback_enabled",front_fallback_?"true":"false");
    add("front_only_attempts",std::to_string(front_only_attempts_));
    report.status.push_back(s);
    }
    clock_diagnostics_->publish(report);
  }
  std::optional<CloudFields> getFields(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto find_field = [&cloud](
      const std::string & name, uint8_t expected_type) -> std::optional<uint32_t>
      {
        const auto it = std::find_if(
          cloud.fields.begin(), cloud.fields.end(),
          [&name](const auto & field) {return field.name == name;});
        const uint32_t size = expected_type == sensor_msgs::msg::PointField::FLOAT64 ? 8U : expected_type == sensor_msgs::msg::PointField::UINT16 ? 2U : 4U;
        if (it == cloud.fields.end() || it->datatype != expected_type || it->count != 1 || it->offset + size > cloud.point_step) {
          return std::nullopt;
        }
        return it->offset;
      };

    const auto x = find_field("x", sensor_msgs::msg::PointField::FLOAT32);
    const auto y = find_field("y", sensor_msgs::msg::PointField::FLOAT32);
    const auto z = find_field("z", sensor_msgs::msg::PointField::FLOAT32);
    const auto intensity = find_field("intensity", sensor_msgs::msg::PointField::FLOAT32);
    const auto ring = find_field("ring", sensor_msgs::msg::PointField::UINT16);
    const auto timestamp = find_field("timestamp", sensor_msgs::msg::PointField::FLOAT64);
    if (!x || !y || !z || !intensity || !ring || !timestamp) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Airy96 cloud schema mismatch: expected x/y/z/intensity FLOAT32, ring UINT16, "
        "timestamp FLOAT64");
      return std::nullopt;
    }
    return CloudFields{*x, *y, *z, *intensity, *ring, *timestamp};
  }

  double decodePointTime(
    double raw, double header_seconds, size_t index, size_t point_count) const
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
      if (best_error < 86400.0) {
        return best;
      }

      const double relative = raw * relative_timestamp_scale_;
      if (relative >= 0.0 && relative <= scan_period_sec_ * 2.0) {
        return header_seconds + relative;
      }
    }

    // Never invent per-point acquisition times for localization deskew.
    (void)index; (void)point_count;
    return std::numeric_limits<double>::quiet_NaN();
  }

  bool appendCloud(
    const sensor_msgs::msg::PointCloud2 & cloud, uint16_t ring_offset,
    std::vector<ParsedPoint> & output)
  {
    if (cloud.is_bigendian || cloud.point_step < 26 || cloud.row_step < cloud.width * static_cast<uint64_t>(cloud.point_step) || cloud.data.size() < static_cast<uint64_t>(cloud.row_step) * cloud.height) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported");
      return false;
    }
    const auto fields = getFields(cloud);
    if (!fields) {
      return false;
    }

    // Use the same explicit Airy calibration as mapping, without publishing
    // competing robot /tf or depending on the removed URDF model.
    if (cloud.header.frame_id != front_frame_ && cloud.header.frame_id != rear_frame_) return false;
    const auto & transform = cloud.header.frame_id == front_frame_ ? front_transform_ : rear_transform_;

    const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;
    const double header_seconds = stampSeconds(cloud.header.stamp);
    const double min_range_squared = min_range_ * min_range_;
    const double max_range_squared = max_range_ * max_range_;
    size_t linear_index = 0;

    for (uint32_t row = 0; row < cloud.height; ++row) {
      const uint8_t * row_data = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step;
      for (uint32_t column = 0; column < cloud.width; ++column, ++linear_index) {
        const uint8_t * data = row_data + static_cast<size_t>(column) * cloud.point_step;
        const float x = readValue<float>(data + fields->x);
        const float y = readValue<float>(data + fields->y);
        const float z = readValue<float>(data + fields->z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        const double range_squared =
          static_cast<double>(x) * x + static_cast<double>(y) * y +
          static_cast<double>(z) * z;
        if (range_squared < min_range_squared || range_squared > max_range_squared) {
          continue;
        }

        const tf2::Vector3 transformed = transform * tf2::Vector3(x, y, z);
        const auto source_ring = readValue<uint16_t>(data + fields->ring);
        const auto raw_time = readValue<double>(data + fields->timestamp);
        const double point_time = decodePointTime(raw_time, header_seconds, linear_index, point_count);
        if (!std::isfinite(point_time)) continue;
        output.push_back(ParsedPoint{
          static_cast<float>(transformed.x()),
          static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z()),
          readValue<float>(data + fields->intensity),
          static_cast<uint16_t>(source_ring + ring_offset),
          point_time});
      }
    }
    return true;
  }

  void tryPublishPair()
  {
    if (!front_cloud_ || !rear_cloud_) {
      return;
    }

    const double front_stamp = stampSeconds(front_cloud_->header.stamp);
    const double rear_stamp = stampSeconds(rear_cloud_->header.stamp);
    const double delta = front_stamp - rear_stamp;
    if (std::abs(delta) > max_sync_delta_sec_) {
      if (delta < 0.0) {
        if(front_fallback_){++front_only_attempts_;publishSingle(*front_cloud_);}
        front_cloud_.reset();
      } else {
        rear_cloud_.reset();
      }
      return;
    }

    std::vector<ParsedPoint> points;
    points.reserve(
      static_cast<size_t>(front_cloud_->width) * front_cloud_->height +
      static_cast<size_t>(rear_cloud_->width) * rear_cloud_->height);
    const bool front_ok = appendCloud(*front_cloud_, 0, points);
    const bool rear_ok = appendCloud(
      *rear_cloud_, static_cast<uint16_t>(std::max(0, rear_ring_offset_)), points);
    front_cloud_.reset();
    rear_cloud_.reset();
    if (!front_ok || !rear_ok || points.empty()) {
      return;
    }

    std::sort(
      points.begin(), points.end(),
      [](const ParsedPoint & lhs, const ParsedPoint & rhs) {
        return lhs.time_seconds < rhs.time_seconds;
      });
    publishClouds(points);
  }

  void publishSingle(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<ParsedPoint> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    if (!appendCloud(cloud, 0, points) || points.empty()) {
      return;
    }
    std::sort(
      points.begin(), points.end(),
      [](const ParsedPoint & lhs, const ParsedPoint & rhs) {
        return lhs.time_seconds < rhs.time_seconds;
      });
    publishClouds(points);
  }

  void publishClouds(const std::vector<ParsedPoint> & points)
  {
    std::optional<double> offset;
    {std::lock_guard<std::mutex> lock(clock_mutex_);offset=input_clock_->offset(steadySeconds());}
    if(!offset)return;
    const double sensor_base_time=points.front().time_seconds;
    const double base_time = sensor_base_time + *offset;
    const double duration = points.back().time_seconds - sensor_base_time;
    const double age = now().seconds() - (points.back().time_seconds + *offset);
    if (duration < 0.001 || duration > 0.15 || age > 0.5 || age < -0.10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Reject invalid/stale Airy time: duration %.3f age %.3f", duration, age);
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = target_frame_;
    cloud.header.stamp = secondsToStamp(base_time);
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = kOutputPointStep;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(6);
    const auto make_field = [](
      const std::string & name, uint32_t offset, uint8_t datatype)
      {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = datatype;
        field.count = 1;
        return field;
      };
    cloud.fields[0] = make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    cloud.fields[1] = make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    cloud.fields[2] = make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    cloud.fields[3] = make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    cloud.fields[4] = make_field("ring", 16, sensor_msgs::msg::PointField::UINT16);
    cloud.fields[5] = make_field("time", 20, sensor_msgs::msg::PointField::FLOAT32);
    cloud.data.resize(cloud.row_step);

    livox_ros_driver2::msg::CustomMsg livox;
    livox.header = cloud.header;
    livox.timebase = static_cast<uint64_t>(std::llround(base_time * 1e9));
    livox.point_num = static_cast<uint32_t>(points.size());
    livox.lidar_id = 0;
    livox.points.resize(points.size());

    for (size_t index = 0; index < points.size(); ++index) {
      const auto & point = points[index];
      uint8_t * data = cloud.data.data() + index * cloud.point_step;
      const float relative_milliseconds =
        static_cast<float>((point.time_seconds - sensor_base_time) * 1000.0);
      writeValue(data + 0, point.x);
      writeValue(data + 4, point.y);
      writeValue(data + 8, point.z);
      writeValue(data + 12, point.intensity);
      writeValue(data + 16, point.ring);
      writeValue(data + 20, relative_milliseconds);

      auto & livox_point = livox.points[index];
      const double relative_nanoseconds = (point.time_seconds - sensor_base_time) * 1e9;
      livox_point.offset_time = static_cast<uint32_t>(std::clamp(
        relative_nanoseconds, 0.0,
        static_cast<double>(std::numeric_limits<uint32_t>::max())));
      livox_point.x = point.x;
      livox_point.y = point.y;
      livox_point.z = point.z;
      livox_point.reflectivity = static_cast<uint8_t>(std::clamp(point.intensity, 0.0F, 255.0F));
      livox_point.tag = kLivoxValidTag;
      // This FastLIO2 fork hard-codes the four Livox scan lines (line < 4).
      // Preserve all Airy96 points by folding only the compatibility field;
      // the standard PointCloud2 output keeps the original 0..191 ring value.
      livox_point.line = static_cast<uint8_t>(point.ring % 4U);
    }

    cloud_pub_->publish(cloud);
    livox_pub_->publish(livox);
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr input)
  {
    const double raw_stamp=stampSeconds(input->header.stamp);
    const double steady=steadySeconds();
    std::optional<double> offset;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);++imu_received_;
      if(last_raw_imu_>0 && raw_stamp>last_raw_imu_)imu_max_source_gap_=std::max(imu_max_source_gap_,raw_stamp-last_raw_imu_);
      last_raw_imu_=raw_stamp;
      if(!input_clock_->observe(raw_stamp,now().seconds(),steady)){++imu_rejected_;return;}
      offset=input_clock_->offset(steady);
    }
    if(!offset)return;
    sensor_msgs::msg::Imu output = *input;
    output.header.stamp=secondsToStamp(raw_stamp+*offset);
    output.header.frame_id = target_frame_;

    // Airy96 vectors are Y-up while ROS navigation is Z-up. Apply the same
    // Rx(+90 deg) rotation used by the cloud TF: (x, y, z) -> (x, -z, y).
    const auto input_acceleration = input->linear_acceleration;
    output.linear_acceleration.x =
      input_acceleration.x * imu_acceleration_scale_;
    output.linear_acceleration.y =
      -input_acceleration.z * imu_acceleration_scale_;
    output.linear_acceleration.z =
      input_acceleration.y * imu_acceleration_scale_;

    const auto input_angular_velocity = input->angular_velocity;
    output.angular_velocity.x = input_angular_velocity.x;
    output.angular_velocity.y = -input_angular_velocity.z;
    output.angular_velocity.z = input_angular_velocity.y;

    const auto rotate_covariance = [](const std::array<double, 9> & input_covariance) {
        std::array<double, 9> rotated{};
        constexpr std::array<std::size_t, 3> source_axis{0U, 2U, 1U};
        constexpr std::array<double, 3> axis_sign{1.0, -1.0, 1.0};
        for (std::size_t row = 0; row < 3; ++row) {
          for (std::size_t column = 0; column < 3; ++column) {
            rotated[row * 3 + column] = axis_sign[row] * axis_sign[column] *
              input_covariance[source_axis[row] * 3 + source_axis[column]];
          }
        }
        return rotated;
      };
    output.angular_velocity_covariance =
      rotate_covariance(input->angular_velocity_covariance);
    output.linear_acceleration_covariance =
      rotate_covariance(input->linear_acceleration_covariance);
    if (output.linear_acceleration_covariance[0] >= 0.0) {
      const double covariance_scale = imu_acceleration_scale_ * imu_acceleration_scale_;
      for (double & value : output.linear_acceleration_covariance) {
        value *= covariance_scale;
      }
    }
    // Airy does not publish an estimated orientation quaternion.
    output.orientation_covariance[0] = -1.0;
    imu_pub_->publish(output);
  }

  std::string front_topic_;
  std::mutex clock_mutex_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  uint64_t imu_received_{0},imu_rejected_{0};
  double last_raw_imu_{0},imu_max_source_gap_{0};
  std::unique_ptr<d1max_localization::InputClock> input_clock_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr clock_diagnostics_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr replay_sub_;
  std::string rear_topic_;
  std::string lidar_mode_;
  bool front_fallback_{false};
  double front_received_at_{0},front_wait_sec_{.03};
  uint64_t front_only_attempts_{0};
  rclcpp::TimerBase::SharedPtr fallback_timer_;
  std::string input_imu_topic_;
  std::string output_cloud_topic_;
  std::string output_livox_topic_;
  std::string output_imu_topic_;
  std::string target_frame_;
  double max_sync_delta_sec_{};
  double transform_timeout_sec_{};
  double scan_period_sec_{};
  double relative_timestamp_scale_{};
  double min_range_{};
  double max_range_{};
  int rear_ring_offset_{};
  double imu_acceleration_scale_{};

  std::string front_frame_, rear_frame_;
  tf2::Transform front_transform_, rear_transform_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr front_cloud_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr rear_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<DualLidarAdapter>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
  executor.add_node(node);executor.spin();executor.remove_node(node);node.reset();
  rclcpp::shutdown();
  return 0;
}
