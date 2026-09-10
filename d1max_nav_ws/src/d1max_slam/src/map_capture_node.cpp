#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
struct VoxelKey
{
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<int64_t>{}(key.x);
    const auto hy = std::hash<int64_t>{}(key.y);
    const auto hz = std::hash<int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};
}  // namespace

class MapCaptureNode final : public rclcpp::Node
{
public:
  MapCaptureNode()
  : Node("map_capture")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/d1max/faster_lio/cloud_registered");
    output_directory_ = declare_parameter<std::string>("output_directory", "/tmp/d1max_maps");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.08);
    auto_save_on_shutdown_ = declare_parameter<bool>("auto_save_on_shutdown", true);
    const auto map_topic =
      declare_parameter<std::string>("map_topic", "/d1max/slam/map_cloud");
    const auto publish_period_sec =
      declare_parameter<double>("publish_period_sec", 1.0);

    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&MapCaptureNode::cloudCallback, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save", std::bind(
        &MapCaptureNode::saveCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, publish_period_sec)),
      std::bind(&MapCaptureNode::publishMap, this));
    RCLCPP_INFO(
      get_logger(), "Capturing %s with %.3f m voxel resolution into %s",
      input_topic.c_str(), voxel_size_, output_directory_.c_str());
  }

  ~MapCaptureNode() override
  {
    if (auto_save_on_shutdown_ && !voxels_.empty()) {
      std::string ignored;
      saveMap(ignored);
    }
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*message, cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    map_frame_ = message->header.frame_id;
    latest_stamp_ = message->header.stamp;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const VoxelKey key{
        static_cast<int64_t>(std::floor(point.x / voxel_size_)),
        static_cast<int64_t>(std::floor(point.y / voxel_size_)),
        static_cast<int64_t>(std::floor(point.z / voxel_size_))};
      voxels_[key] = point;
    }
  }

  void publishMap()
  {
    if (map_publisher_->get_subscription_count() == 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::string frame;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (voxels_.empty()) {
        return;
      }
      cloud.points.reserve(voxels_.size());
      for (const auto & item : voxels_) {
        cloud.points.push_back(item.second);
      }
      frame = map_frame_;
      stamp = latest_stamp_;
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = frame;
    message.header.stamp = stamp;
    map_publisher_->publish(message);
  }

  void saveCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = saveMap(response->message);
  }

  bool saveMap(std::string & message)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (voxels_.empty()) {
        message = "No registered points have been received";
        return false;
      }
      cloud.points.reserve(voxels_.size());
      for (const auto & item : voxels_) {
        cloud.points.push_back(item.second);
      }
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;

    try {
      std::filesystem::create_directories(output_directory_);
      const auto now = std::chrono::system_clock::now();
      const auto time = std::chrono::system_clock::to_time_t(now);
      std::tm local_time{};
      localtime_r(&time, &local_time);
      std::ostringstream filename;
      filename << "d1max_map_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".pcd";
      const auto path = std::filesystem::path(output_directory_) / filename.str();
      if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
        message = "PCL failed to write " + path.string();
        return false;
      }

      const auto latest = std::filesystem::path(output_directory_) / "scans.pcd";
      std::error_code error;
      std::filesystem::remove(latest, error);
      error.clear();
      std::filesystem::create_symlink(path.filename(), latest, error);
      if (error) {
        RCLCPP_WARN(get_logger(), "Map saved, but scans.pcd symlink failed: %s", error.message().c_str());
      }
      message = "Saved " + std::to_string(cloud.points.size()) + " points to " + path.string();
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      return true;
    } catch (const std::exception & exception) {
      message = exception.what();
      RCLCPP_ERROR(get_logger(), "Map save failed: %s", exception.what());
      return false;
    }
  }

  std::string output_directory_;
  double voxel_size_{};
  bool auto_save_on_shutdown_{};
  std::string map_frame_;
  builtin_interfaces::msg::Time latest_stamp_;
  std::mutex mutex_;
  std::unordered_map<VoxelKey, pcl::PointXYZI, VoxelKeyHash> voxels_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
