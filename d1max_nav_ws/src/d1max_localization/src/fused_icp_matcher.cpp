#include "d1max_localization/fused_icp_gate.hpp"
#include "d1max_localization/rotation_deskewer.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>
#include "d1max_localization/registration_information.hpp"
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace d1max_localization {
namespace {

struct TimedPose {
  int64_t stamp_ns{0};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
};

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromPose(const Eigen::Matrix4d &pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

bool finitePose(const geometry_msgs::msg::Pose &pose) {
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

class FusedIcpMatcher : public rclcpp::Node {
public:
  FusedIcpMatcher()
      : Node("fused_icp_matcher"),
        map_cloud_(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {
    declareAndLoadParameters();
    if(input_deskewed_)imu_ready_=true;
    loadMap();
    // The static map never changes within a session. Reuse its search tree and
    // GICP covariances instead of rebuilding them for every 10 Hz scan.
    matcher_.setInputTarget(map_cloud_);
    quality_tree_.setInputCloud(map_cloud_);
    matcher_.setNumThreads(num_threads_);

    // GICP can take longer than one LiDAR period. Keep IMU ingestion in a
    // separate mutually-exclusive callback group so deskew data continues to
    // arrive while the default callback group performs scan matching.
    imu_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    rclcpp::SensorDataQoS imu_qos;
    imu_qos.keep_last(500);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&FusedIcpMatcher::onImu, this, std::placeholders::_1),
        imu_options);

    velocity_subscription_=create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        velocity_topic_,rclcpp::QoS(100),
        [this](geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr m){
          if(m->header.frame_id!=tracking_frame_)return;
          const int64_t t=rclcpp::Time(m->header.stamp).nanoseconds();
          const auto& v=m->twist.twist.linear;
          const Eigen::Vector3d velocity(v.x,v.y,v.z);
          if(t<=0 || !velocity.allFinite())return;
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          if(!velocity_buffer_.empty() && t<=velocity_buffer_.back().stamp_ns)return;
          velocity_buffer_.push_back({t,velocity});
          while(velocity_buffer_.size()>2 && (t-velocity_buffer_[1].stamp_ns)*1e-9>imu_buffer_duration_sec_)
            velocity_buffer_.pop_front();
        },imu_options);

    rclcpp::SensorDataQoS scan_qos;
    // Old scans are actively harmful to localization: after a slow match they
    // no longer describe the robot's current pose. DDS and the application
    // therefore both retain only the newest scan.
    scan_qos.keep_last(1);
    if(input_deskewed_){
      deskewed_subscription_=create_subscription<sensor_msgs::msg::PointCloud2>(scan_topic_,scan_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr input){
          if(input->header.frame_id!=tracking_frame_)return;
          pcl::PointCloud<pcl::PointXYZ> points;pcl::fromROSMsg(*input,points);
          auto m=std::make_shared<livox_ros_driver2::msg::CustomMsg>();m->header=input->header;
          m->timebase=rclcpp::Time(input->header.stamp).nanoseconds();m->points.reserve(points.size());
          for(const auto&p:points){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
            livox_ros_driver2::msg::CustomPoint q;q.x=p.x;q.y=p.y;q.z=p.z;q.offset_time=0;m->points.push_back(q);}
          m->point_num=m->points.size();onScan(m);
        });
    }else scan_subscription_ =
        create_subscription<livox_ros_driver2::msg::CustomMsg>(
            scan_topic_, scan_qos,
            std::bind(&FusedIcpMatcher::onScan, this, std::placeholders::_1));

    initial_pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, 10,
            std::bind(&FusedIcpMatcher::onInitialPose, this,
                      std::placeholders::_1));
    prediction_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prediction_topic_, 20,
            std::bind(&FusedIcpMatcher::onPrediction, this,
                      std::placeholders::_1));

    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_pose_topic_, 20);
    if(input_deskewed_)verified_publisher_=create_publisher<std_msgs::msg::String>(output_pose_topic_+"/verified",20);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    raw_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        raw_scan_topic_, rclcpp::SensorDataQoS());
    leveled_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        leveled_scan_topic_, rclcpp::SensorDataQoS());
    preview_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        preview_scan_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    diagnostic_timer_ = create_wall_timer(std::chrono::milliseconds(500),[this]{publishDiagnostic();});

    publishMap();
    map_timer_ = create_wall_timer(std::chrono::seconds(3),
                                   [this]() { publishMap(); });
    scan_processing_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { processPendingScan(); });

    RCLCPP_INFO(get_logger(),
                "Independent fused ICP matcher ready: map=%lu points",
                map_cloud_->size());
    RCLCPP_INFO(get_logger(), "Waiting for supervisor stationary-calibrated 3-axis IMU");
  }

private:
  void declareAndLoadParameters() {
    map_path_ = declare_parameter("map_pcd", "");
    scan_topic_ = declare_parameter("scan_topic", "/livox/lidar");
    imu_topic_ = declare_parameter("imu_topic", "/livox/imu");
    world_frame_ = declare_parameter("world_frame", "map");
    tracking_frame_ =
        declare_parameter("tracking_frame", "icp_tracking_frame");
    lidar_frame_ = declare_parameter("lidar_frame", "livox_frame");
    initial_pose_topic_ = declare_parameter(
        "initial_pose_topic", "/d1max/localization/fused_icp/initialpose");
    prediction_topic_ = declare_parameter(
        "prediction_topic", "/d1max/localization/fused_icp/prediction");
    output_pose_topic_ = declare_parameter(
        "output_pose_topic", "/d1max/localization/fused_icp/pose_raw");
    map_cloud_topic_ = declare_parameter("map_cloud_topic", "/map_cloud");
    raw_scan_topic_ =
        declare_parameter("raw_scan_topic", "/scan_converted");
    leveled_scan_topic_ =
        declare_parameter("leveled_scan_topic", "/scan_leveled");
    preview_scan_topic_=declare_parameter("preview_scan_topic","/d1max/localization/scan_initial_preview");

    voxel_leaf_ = declare_parameter("voxel_leaf", 0.15);
    max_corr_dist_ = declare_parameter("max_corr_dist", 1.5);
    max_translation_delta_ =
        declare_parameter("max_translation_delta", 0.45);
    max_translation_z_delta_ =
        declare_parameter("max_translation_z_delta", 0.35);
    max_yaw_delta_ = declare_parameter("max_yaw_delta", 0.60);
    max_tilt_ = declare_parameter("max_tilt", 0.35);
    max_rotation_delta_ =
        declare_parameter("max_rotation_delta", 0.70);
    max_fitness_score_ = declare_parameter("max_fitness_score", 1.0);
    quality_distance_ = declare_parameter("quality_inlier_distance", 0.50);
    min_inlier_ratio_ = declare_parameter("min_inlier_ratio", 0.35);
    max_inlier_rmse_ = declare_parameter("max_inlier_rmse", 0.30);
    confirmation_max_gap_sec_ = declare_parameter("confirmation_max_gap_sec", 0.35);
    if (!std::isfinite(quality_distance_) || quality_distance_ <= 0 ||
        !std::isfinite(min_inlier_ratio_) || min_inlier_ratio_ <= 0 || min_inlier_ratio_ > 1 ||
        !std::isfinite(max_inlier_rmse_) || max_inlier_rmse_ <= 0 ||
        !std::isfinite(confirmation_max_gap_sec_) || confirmation_max_gap_sec_ <= 0)
      throw std::runtime_error("invalid ICP quality / confirmation parameters");
    max_iterations_ = declare_parameter("max_iterations", 15);
    num_threads_ = declare_parameter("num_threads", 4);
    if (num_threads_ < 1 || num_threads_ > 16) throw std::runtime_error("num_threads must be between 1 and 16");

    acquisition_max_corr_dist_ =
        declare_parameter("acquisition_max_corr_dist", 3.0);
    acquisition_max_translation_delta_ =
        declare_parameter("acquisition_max_translation_delta", 2.5);
    acquisition_max_translation_z_delta_ =
        declare_parameter("acquisition_max_translation_z_delta", 1.0);
    acquisition_max_yaw_delta_ =
        declare_parameter("acquisition_max_yaw_delta", 1.57);
    acquisition_max_tilt_ =
        declare_parameter("acquisition_max_tilt", 0.35);
    acquisition_max_rotation_delta_ =
        declare_parameter("acquisition_max_rotation_delta", 1.80);
    acquisition_max_fitness_score_ =
        declare_parameter("acquisition_max_fitness_score", 1.0);
    acquisition_max_iterations_ =
        declare_parameter("acquisition_max_iterations", 50);
    acquisition_confirmation_count_ =
        declare_parameter("acquisition_confirmation_count", 3);

    prediction_timeout_sec_ =
        declare_parameter("prediction_timeout_sec", 0.25);
    prediction_buffer_duration_sec_ =
        declare_parameter("prediction_buffer_duration_sec", 2.0);
    deskew_enabled_ = declare_parameter("deskew_enabled", true);
    input_deskewed_=declare_parameter("input_deskewed",false);
    min_information_ratio_=declare_parameter("min_information_ratio",0.0);
    if(input_deskewed_ && deskew_enabled_)throw std::runtime_error("Refusing double deskew of LIO cloud");
    if(!std::isfinite(min_information_ratio_)||min_information_ratio_<0||min_information_ratio_>=1)
      throw std::runtime_error("Invalid information threshold");
    translation_deskew_enabled_=declare_parameter("translation_deskew_enabled",true);
    velocity_topic_=declare_parameter("velocity_topic","/d1max/localization/body_twist");
    max_velocity_gap_sec_=declare_parameter("max_velocity_gap_sec",.06);
    if((translation_deskew_enabled_ && !deskew_enabled_) ||
       !std::isfinite(max_velocity_gap_sec_) || max_velocity_gap_sec_<=0)
      throw std::runtime_error("translation deskew requires rotation deskew / valid velocity gap");
    max_imu_gap_sec_ = declare_parameter("max_imu_gap_sec", 0.02);
    max_scan_duration_sec_ =
        declare_parameter("max_scan_duration_sec", 0.15);
    max_scan_age_sec_ = declare_parameter("max_scan_age_sec", 0.50);
    imu_buffer_duration_sec_ =
        declare_parameter("imu_buffer_duration_sec", 2.0);

    const auto rotation_values = declare_parameter<std::vector<double>>(
        "rotation_lidar_from_imu",
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    rotation_lidar_from_imu_.setIdentity();
    if (rotation_values.size() == 9) {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          rotation_lidar_from_imu_(row, column) =
              rotation_values[row * 3 + column];
        }
      }
    }
    const Eigen::Matrix3d orthogonality =
        rotation_lidar_from_imu_ * rotation_lidar_from_imu_.transpose();
    if (rotation_values.size() != 9 ||
        !orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
        std::abs(rotation_lidar_from_imu_.determinant() - 1.0) > 1e-3) {
      throw std::runtime_error("rotation_lidar_from_imu must be a valid rigid rotation");
    }

    if (!rotation_lidar_from_imu_.isApprox(Eigen::Matrix3d::Identity(), 1e-9))
      throw std::runtime_error("corrected IMU and normalized LiDAR must share the fixed tracking basis");
    if (map_path_.empty()) {
      throw std::runtime_error("map_pcd parameter is empty");
    }
    if (voxel_leaf_ <= 0.0 || max_corr_dist_ <= 0.0 ||
        max_translation_delta_ <= 0.0 || max_yaw_delta_ <= 0.0 ||
        max_translation_z_delta_ <= 0.0 || max_tilt_ <= 0.0 ||
        max_rotation_delta_ <= 0.0 || max_fitness_score_ <= 0.0 ||
        max_iterations_ < 1) {
      throw std::runtime_error("tracking ICP parameters must be positive");
    }
    acquisition_max_corr_dist_ =
        std::max(acquisition_max_corr_dist_, max_corr_dist_);
    acquisition_max_translation_delta_ = std::max(
        acquisition_max_translation_delta_, max_translation_delta_);
    acquisition_max_translation_z_delta_ = std::max(
        acquisition_max_translation_z_delta_, max_translation_z_delta_);
    acquisition_max_yaw_delta_ =
        std::max(acquisition_max_yaw_delta_, max_yaw_delta_);
    acquisition_max_tilt_ = std::max(acquisition_max_tilt_, max_tilt_);
    acquisition_max_rotation_delta_ = std::max(
        acquisition_max_rotation_delta_, max_rotation_delta_);
    acquisition_max_iterations_ =
        std::max(acquisition_max_iterations_, max_iterations_);
    acquisition_confirmation_count_ =
        std::max(1, acquisition_confirmation_count_);
    if (acquisition_max_fitness_score_ <= 0.0) {
      acquisition_max_fitness_score_ = max_fitness_score_;
    }
    prediction_timeout_sec_ = std::max(0.01, prediction_timeout_sec_);
    prediction_buffer_duration_sec_ = std::max(
        prediction_buffer_duration_sec_, 2.0 * prediction_timeout_sec_);
    max_imu_gap_sec_ = std::max(0.001, max_imu_gap_sec_);
    max_scan_duration_sec_ = std::max(0.01, max_scan_duration_sec_);
    max_scan_age_sec_ = std::max(0.05, max_scan_age_sec_);
    imu_buffer_duration_sec_ = std::max(
        imu_buffer_duration_sec_,
        max_scan_duration_sec_ + 2.0 * max_imu_gap_sec_);

    gate_.configure(
        {max_translation_delta_, max_yaw_delta_, max_fitness_score_},
        {acquisition_max_translation_delta_, acquisition_max_yaw_delta_,
         acquisition_max_fitness_score_},
        acquisition_confirmation_count_);
  }

  void loadMap() {
    auto full_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *full_map) == -1) {
      throw std::runtime_error("failed to load ICP map: " + map_path_);
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(full_map);
    voxel_filter.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    voxel_filter.filter(*map_cloud_);
    if (map_cloud_->size() < 100) {
      throw std::runtime_error("downsampled ICP map has too few points");
    }
    RCLCPP_INFO(get_logger(), "Fused ICP map: %lu -> %lu points",
                full_map->size(), map_cloud_->size());
  }

  void publishMap() {
    if (!map_publisher_) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map_cloud_, message);
    message.header.frame_id = world_frame_;
    message.header.stamp = now();
    map_publisher_->publish(message);
  }

  void onInitialPose(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    const auto seed_ns=rclcpp::Time(message->header.stamp).nanoseconds();
    if(input_deskewed_ && (seed_ns<=active_seed_ns_ || seed_ns<=0 ||
       (now().nanoseconds()-seed_ns)*1e-9>5. || (seed_ns-now().nanoseconds())*1e-9>.1))return;
    if ((!message->header.frame_id.empty() &&
         message->header.frame_id != world_frame_) ||
        !finitePose(message->pose.pose)) {
      RCLCPP_ERROR(get_logger(), "Ignoring invalid fused ICP initial pose");
      return;
    }
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if (quaternion.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(),
                   "Ignoring fused ICP initial pose with zero quaternion");
      return;
    }
    quaternion.normalize();
    active_seed_ns_=seed_ns;
    last_pose_.setIdentity();
    last_pose_.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    last_pose_(0, 3) = position.x;
    last_pose_(1, 3) = position.y;
    last_pose_(2, 3) = position.z;
    prediction_buffer_.clear();
    gate_.reset();
    last_candidate_stamp_ns_ = 0;
    initialized_ = true;
    match_reason_="等待新扫描验证初值";match_attempts_=0;non_converged_=0;elapsed_ms_=0.;
    RCLCPP_INFO(
        get_logger(),
        "Fused ICP acquisition started at (%.2f, %.2f, %.2f); "
        "limits %.2fm %.2frad, confirmations=%d",
        position.x, position.y, position.z,
        acquisition_max_translation_delta_, acquisition_max_yaw_delta_,
        acquisition_confirmation_count_);
  }

  void onPrediction(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    if (!initialized_ ||
        (!message->header.frame_id.empty() &&
         message->header.frame_id != world_frame_) ||
        !finitePose(message->pose.pose)) {
      return;
    }
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if (quaternion.norm() < 1e-9) {
      return;
    }
    quaternion.normalize();
    TimedPose prediction;
    prediction.stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (prediction.stamp_ns <= 0) {
      return;  // Never assign receipt time to an unstamped prediction.
    }
    prediction.pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    prediction.pose(0, 3) = position.x;
    prediction.pose(1, 3) = position.y;
    prediction.pose(2, 3) = position.z;
    if (!prediction_buffer_.empty() &&
        prediction.stamp_ns <= prediction_buffer_.back().stamp_ns) {
      if (prediction.stamp_ns == prediction_buffer_.back().stamp_ns) {
        prediction_buffer_.back() = prediction;
        return;
      }
      prediction_buffer_.clear();
    }
    prediction_buffer_.push_back(prediction);
    const int64_t keep_after_ns =
        prediction.stamp_ns -
        static_cast<int64_t>(prediction_buffer_duration_sec_ * 1e9);
    while (prediction_buffer_.size() > 2 &&
           prediction_buffer_[1].stamp_ns < keep_after_ns) {
      prediction_buffer_.pop_front();
    }
  }

  bool predictionAt(int64_t target_stamp_ns, Eigen::Matrix4d *pose) const {
    if (!pose || prediction_buffer_.empty()) return false;
    auto right = std::lower_bound(prediction_buffer_.begin(), prediction_buffer_.end(), target_stamp_ns,
        [](const TimedPose& p, int64_t t){return p.stamp_ns < t;});
    if (right == prediction_buffer_.end()) return false;
    if (right->stamp_ns == target_stamp_ns) { *pose = right->pose; return true; }
    if (right == prediction_buffer_.begin()) return false;
    const auto left = std::prev(right);
    const auto gap = right->stamp_ns - left->stamp_ns;
    if (gap <= 0 || gap > static_cast<int64_t>(prediction_timeout_sec_ * 1e9)) return false;
    const double u = double(target_stamp_ns-left->stamp_ns)/double(gap);
    pose->setIdentity();
    pose->block<3,1>(0,3) = (1-u)*left->pose.block<3,1>(0,3) + u*right->pose.block<3,1>(0,3);
    const Eigen::Quaterniond q0(left->pose.block<3,3>(0,0)), q1(right->pose.block<3,3>(0,0));
    pose->block<3,3>(0,0) = q0.slerp(u,q1).normalized().toRotationMatrix();
    return true;
  }

  void onImu(sensor_msgs::msg::Imu::SharedPtr message) {
    // This topic is produced only after the supervisor's shared stationary
    // three-axis bias calibration. Do not level clouds or estimate a second bias.
    if (message->header.frame_id != tracking_frame_) return;
    const int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    const double age = (now().nanoseconds()-stamp_ns)*1e-9;
    if (stamp_ns <= 0 || age < -0.1 || age > max_scan_age_sec_) return;
    const Eigen::Vector3d gyro(message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z);
    if (!gyro.allFinite()) return;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (stamp_ns <= last_imu_stamp_ns_) return;
    imu_buffer_.push_back({stamp_ns, gyro});
    last_imu_stamp_ns_ = stamp_ns;
    imu_ready_ = true;
    pruneImuBuffer();
  }

  bool scanTimeRange(const livox_ros_driver2::msg::CustomMsg &message,
                     int64_t *start_ns, int64_t *end_ns) const {
    if (message.points.empty() || start_ns == nullptr || end_ns == nullptr) {
      return false;
    }
    const uint64_t base_time =
        message.timebase != 0
            ? message.timebase
            : static_cast<uint64_t>(
                  rclcpp::Time(message.header.stamp).nanoseconds());
    const auto max_offset = std::max_element(
        message.points.begin(), message.points.end(),
        [](const auto &left, const auto &right) {
          return left.offset_time < right.offset_time;
        })->offset_time;
    if (base_time == 0 ||
        base_time >
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) -
                max_offset) {
      return false;
    }
    *start_ns = static_cast<int64_t>(base_time);
    *end_ns = static_cast<int64_t>(base_time + max_offset);
    return true;
  }

  void onScan(livox_ros_driver2::msg::CustomMsg::SharedPtr message) {
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    if (!scanTimeRange(*message, &start_ns, &end_ns)) {
      return;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!imu_ready_) {
      return;
    }
    if (last_scan_stamp_ns_ != 0 && start_ns < last_scan_stamp_ns_) {
      pending_scans_.clear();
      RCLCPP_WARN(get_logger(),
                  "Airy scan time moved backwards; clearing fused ICP queue");
    }
    last_scan_stamp_ns_ = start_ns;
    if (!pending_scans_.empty()) {
      pending_scans_.pop_front();
      RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Fused ICP busy; replacing stale scan with newest");
    }
    pending_scans_.push_back(std::move(message));
  }

  void processPendingScan() {
    livox_ros_driver2::msg::CustomMsg::SharedPtr scan_message;
    std::deque<AngularVelocitySample> imu_snapshot;
    std::deque<LinearVelocitySample> velocity_snapshot;
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (pending_scans_.empty() || (deskew_enabled_ && imu_buffer_.empty())) {
        return;
      }
      if (!scanTimeRange(*pending_scans_.front(), &start_ns, &end_ns)) {
        pending_scans_.pop_front();
        return;
      }
      const double duration = static_cast<double>(end_ns - start_ns) * 1e-9;
      if (duration > max_scan_duration_sec_) {
        pending_scans_.pop_front();
        gate_.rejectAttempt();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Dropping fused ICP scan with %.1fms duration", duration * 1000.0);
        return;
      }
      if (translation_deskew_enabled_ &&
          (velocity_buffer_.empty() || end_ns>velocity_buffer_.back().stamp_ns) &&
          (now().nanoseconds()-end_ns)*1e-9<=max_scan_age_sec_) return;
      if (deskew_enabled_ && end_ns > imu_buffer_.back().stamp_ns) {
        return;
      }
      if(input_deskewed_ && initialized_){
        Eigen::Matrix4d prediction;
        if(!predictionAt(end_ns,&prediction)){
          match_reason_="等待扫描同一时刻的 LIO 预测";
          if((now().nanoseconds()-end_ns)*1e-9>max_scan_age_sec_){pending_scans_.pop_front();gate_.rejectAttempt();}
          return;
        }
      }
      scan_message = pending_scans_.front();
      pending_scans_.pop_front();
      imu_snapshot = imu_buffer_;
      velocity_snapshot = velocity_buffer_;
    }
    const double age_sec =
        static_cast<double>(now().nanoseconds() - end_ns) * 1e-9;
    if (age_sec > max_scan_age_sec_ || age_sec < -0.10) {
      gate_.rejectAttempt();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Dropping stale fused ICP scan (age %.3fs)", age_sec);
      return;
    }
    processScan(scan_message, start_ns, end_ns, imu_snapshot, velocity_snapshot);
  }

  void processScan(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr &message,
      int64_t start_ns, int64_t end_ns,
      const std::deque<AngularVelocitySample> &imu_samples,
      const std::deque<LinearVelocitySample> &velocity_samples) {
    // All early exits (including deskew / empty geometry) break acquisition.
    struct Attempt {
      FusedIcpGate& gate; bool keep{false};
      ~Attempt(){if (!keep) gate.rejectAttempt();}
    } attempt{gate_};
    if (last_candidate_stamp_ns_ &&
        (end_ns-last_candidate_stamp_ns_)*1e-9 > confirmation_max_gap_sec_)
      gate_.rejectAttempt();
    RotationDeskewer deskewer;
    const bool has_duration = end_ns > start_ns;
    if (deskew_enabled_ && has_duration) {
      std::string error;
      if (!deskewer.build(imu_samples, start_ns, end_ns, Eigen::Vector3d::Zero(),
                          rotation_lidar_from_imu_, max_imu_gap_sec_, &error)) {
        match_reason_="IMU 去畸变失败："+error;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Fused ICP deskew unavailable: %s",
                             error.c_str());
        return;
      }
    }

    if (translation_deskew_enabled_ && has_duration) {
      std::string error;
      if (!deskewer.addTranslation(velocity_samples,max_velocity_gap_sec_,&error)) {
        match_reason_="MC 平移去畸变缺少完整速度数据";
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Translation deskew unavailable: %s",error.c_str());
        return;
      }
    }
    auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    scan->reserve(message->points.size());
    for (const auto &point_message : message->points) {
      if (!std::isfinite(point_message.x) ||
          !std::isfinite(point_message.y) ||
          !std::isfinite(point_message.z)) {
        continue;
      }
      Eigen::Vector3d point(point_message.x, point_message.y, point_message.z);
      if (deskew_enabled_ && has_duration) {
        point = deskewer.compensate(point, start_ns + point_message.offset_time);
      }
      scan->emplace_back(static_cast<float>(point.x()),
                         static_cast<float>(point.y()),
                         static_cast<float>(point.z()));
    }
    if (scan->size() < 50) {
      return;
    }

    const rclcpp::Time scan_stamp(end_ns);
    sensor_msgs::msg::PointCloud2 raw_message;
    pcl::toROSMsg(*scan, raw_message);
    raw_message.header.frame_id = lidar_frame_;
    raw_message.header.stamp = scan_stamp;
    raw_scan_publisher_->publish(raw_message);

    // Legacy topic name retained for the one Foxglove layout. XYZ remain in
    // the physical fixed tracking basis, identical to normalized LiDAR/IMU.
    // Gravity/map tilt is estimated by the full SE(3) match, never baked here.
    sensor_msgs::msg::PointCloud2 leveled_message;
    pcl::toROSMsg(*scan, leveled_message);
    leveled_message.header.frame_id = tracking_frame_;
    leveled_message.header.stamp = scan_stamp;
    leveled_scan_publisher_->publish(leveled_message);

    if (!initialized_) {
      return;
    }
    if(input_deskewed_ && end_ns<active_seed_ns_)return;

    auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(scan);
    voxel_filter.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    voxel_filter.filter(*downsampled);
    if (downsampled->size() < 50) {
      return;
    }

    const bool acquiring = !gate_.locked();
    if(acquiring)publishPreview(*downsampled,last_pose_,scan_stamp);
    const double correspondence_distance =
        acquiring ? acquisition_max_corr_dist_ : max_corr_dist_;
    const int iterations =
        acquiring ? acquisition_max_iterations_ : max_iterations_;

    auto &matcher = matcher_;
    matcher.setInputSource(downsampled);
    matcher.setMaxCorrespondenceDistance(correspondence_distance);
    matcher.setMaximumIterations(iterations);

    Eigen::Matrix4d initial_guess = last_pose_;
    Eigen::Matrix4d timed_prediction;
    if ((!acquiring || input_deskewed_) && predictionAt(end_ns, &timed_prediction)) {
      // During a turn, a current-time prediction can be tens of degrees ahead
      // of the scan being matched. Select the buffered odometry prediction at
      // the LiDAR timestamp instead.
      initial_guess = timed_prediction;
      // Transport a provisional acquisition candidate with measured LIO motion.
      // Comparing every moving scan to the original arrow cannot confirm a pose.
      if(acquiring && last_candidate_stamp_ns_){
        Eigen::Matrix4d previous_prediction;
        if(predictionAt(last_candidate_stamp_ns_,&previous_prediction))
          initial_guess=last_pose_*previous_prediction.inverse()*timed_prediction;
      }
    }

    auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto started=std::chrono::steady_clock::now();++match_attempts_;match_reason_="匹配计算中";
    source_points_=downsampled->size();
    matcher.align(*aligned, initial_guess.cast<float>());
    elapsed_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    if (!matcher.hasConverged()) {
      ++non_converged_;match_reason_="GICP 未收敛";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Fused ICP did not converge");
      return;
    }

    const Eigen::Matrix4d candidate =
        matcher.getFinalTransformation().cast<double>();
    if(min_information_ratio_>0){
      double radius2=0.;for(const auto& p:downsampled->points)radius2+=p.getVector3fMap().squaredNorm();
      const double radius=std::clamp(std::sqrt(radius2/downsampled->size()),1.,30.);
      information_ratio_=informationRatio(matcher.getFinalHessian(),candidate.block<3,1>(0,3),radius);
      if(!std::isfinite(information_ratio_) || information_ratio_<min_information_ratio_){
        match_reason_="几何退化：拒绝不可靠校正";return;
      }
    }
    const Eigen::Matrix3d candidate_rotation =
        candidate.block<3, 3>(0, 0);
    const Eigen::Matrix3d initial_rotation =
        initial_guess.block<3, 3>(0, 0);
    const Eigen::Matrix3d rotation_orthogonality =
        candidate_rotation * candidate_rotation.transpose();
    if (!candidate.allFinite() ||
        !rotation_orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
        std::abs(candidate_rotation.determinant() - 1.0) > 1e-3) {
      match_reason_="匹配结果不是有效刚体变换";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Reject fused ICP with invalid 3D transform");
      return;
    }
    const Eigen::Vector3d translation =
        candidate.block<3, 1>(0, 3) - initial_guess.block<3, 1>(0, 3);
    const double translation_delta = translation.head<2>().norm();
    const double translation_z_delta = std::abs(translation.z());
    const double yaw_delta = std::abs(normalizeAngle(
        yawFromPose(candidate) - yawFromPose(initial_guess)));
    const double tilt = std::acos(std::clamp(
        candidate_rotation.col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
    const Eigen::AngleAxisd rotation_change(
        initial_rotation.transpose() * candidate_rotation);
    const double rotation_delta = std::abs(rotation_change.angle());
    const double translation_z_limit =
        acquiring ? acquisition_max_translation_z_delta_
                  : max_translation_z_delta_;
    const double tilt_limit = acquiring ? acquisition_max_tilt_ : max_tilt_;
    const double rotation_limit =
        acquiring ? acquisition_max_rotation_delta_ : max_rotation_delta_;
    if (translation_z_delta > translation_z_limit || tilt > tilt_limit ||
        rotation_delta > rotation_limit) {
      match_reason_="匹配的高度或旋转超出门限";
      // FastGICP is a full 6-DoF optimizer. In sparse or locally planar
      // geometry it can return a 90/180-degree roll or pitch solution whose
      // projected yaw still appears valid. Never let such a solution poison
      // last_pose_; the bridge's innovation gate is a second line of defence,
      // not a substitute for validating the matcher state itself.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reject fused ICP 3D jump: z=%.3fm tilt=%.3frad rotation=%.3frad "
          "mode=%s",
          translation_z_delta, tilt, rotation_delta,
          acquiring ? "acquisition" : "tracking");
      return;
    }
    // PCL getFitnessScore's max_range compares squared NN distances (m^2).
    const double fitness = matcher.getFitnessScore(correspondence_distance * correspondence_distance);
    std::vector<int> indices(1); std::vector<float> distances(1);
    size_t inliers=0; double squared_error=0.;
    for (const auto& point : *aligned) {
      if (quality_tree_.nearestKSearch(point,1,indices,distances)>0 &&
          distances[0] <= quality_distance_*quality_distance_) {
        ++inliers; squared_error += distances[0];
      }
    }
    inlier_ratio_ = double(inliers) / std::max<size_t>(1, aligned->size());
    inlier_rmse_ = inliers ? std::sqrt(squared_error/inliers) : quality_distance_;
    if (inliers < 50 || inlier_ratio_ < min_inlier_ratio_ || inlier_rmse_ > max_inlier_rmse_) {
      match_reason_="点云有效重叠不足或残差过大";
      return;
    }
    // Expensive GICP may finish after its scan expires. Never publish a fresh
    // looking absolute correction for a stale scan.
    if ((now().nanoseconds() - end_ns) * 1e-9 > max_scan_age_sec_) {match_reason_="匹配完成时扫描已过期";return;}
    const IcpGateDecision decision =
        gate_.evaluate(translation_delta, yaw_delta, fitness);
    if (!decision.update_candidate) {
      match_reason_="匹配偏移或残差超出门限";
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reject fused ICP: xy=%.3fm yaw=%.3frad fitness=%.3f mode=%s",
          translation_delta, yaw_delta, fitness,
          acquiring ? "acquisition" : "tracking");
      return;
    }

    attempt.keep = true;
    last_candidate_stamp_ns_ = end_ns;
    last_pose_ = candidate;
    match_reason_=decision.publish_pose?"匹配已连续确认":"候选匹配确认中";
    if (decision.restarted_confirmation) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Fused ICP acquisition moved; restarting confirmation");
    } else if (!decision.publish_pose) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Fused ICP acquisition candidate %d/%d",
          gate_.confirmationCount(), gate_.requiredConfirmations());
    }
    if (decision.just_locked) {
      pcl::PointCloud<pcl::PointXYZ> empty;
      publishPreview(empty,last_pose_,scan_stamp);
      RCLCPP_INFO(get_logger(),
                  "Fused ICP acquisition locked; enabling EKF prediction mode");
    }
    if (decision.publish_pose) {
      publishPose(last_pose_, scan_stamp);
    }
  }

  // Display-only equivalent of the old Go2 initial-seed overlay. It has no TF,
  // EKF or navigation authority. The original map/scans and Z remain unchanged.
  void publishPreview(const pcl::PointCloud<pcl::PointXYZ>& scan,const Eigen::Matrix4d& pose,const rclcpp::Time& stamp){
    pcl::PointCloud<pcl::PointXYZ> transformed;
    // PCL 1.12 transformPointCloud calls assign(..., cloud_in.width), which
    // divides by width. Bypass it for empty input and publish a valid 0x1 cloud.
    if(scan.empty()){transformed.width=0;transformed.height=1;}
    else pcl::transformPointCloud(scan,transformed,pose);
    sensor_msgs::msg::PointCloud2 message;pcl::toROSMsg(transformed,message);
    message.header.frame_id=world_frame_;message.header.stamp=stamp;preview_publisher_->publish(message);
  }
  void publishDiagnostic(){
    diagnostic_msgs::msg::DiagnosticArray report;report.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus status;status.name="d1max_localization/matcher";
    status.level=gate_.locked()?0:1;status.message=match_reason_;
    const auto add=[&](const std::string& k,const std::string& v){diagnostic_msgs::msg::KeyValue p;p.key=k;p.value=v;status.values.push_back(p);};
    add("attempts",std::to_string(match_attempts_));add("non_converged",std::to_string(non_converged_));
    add("elapsed_ms",std::to_string(elapsed_ms_));add("source_points",std::to_string(source_points_));
    add("map_points",std::to_string(map_cloud_->size()));add("confirmations",std::to_string(gate_.confirmationCount()));
    add("inlier_ratio",std::to_string(inlier_ratio_));add("inlier_rmse_m",std::to_string(inlier_rmse_));
    add("deskew_mode",input_deskewed_?"faster_lio_6d":translation_deskew_enabled_?"rotation_and_mc_translation":"rotation_only");
    add("information_ratio",std::to_string(information_ratio_));
    add("tracking_basis","fixed_normalized_front_imu");add("covariance_model","conservative_quality_scaled_not_calibrated");
    add("preview_only",gate_.locked()?"false":"true");report.status.push_back(status);diagnostic_publisher_->publish(report);
  }

  void publishPose(const Eigen::Matrix4d &pose,
                   const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.pose.pose.position.x = pose(0, 3);
    message.pose.pose.position.y = pose(1, 3);
    message.pose.pose.position.z = pose(2, 3);
    const Eigen::Quaterniond orientation(pose.block<3, 3>(0, 0));
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    // Conservative heuristic, not a calibrated GICP Hessian covariance.
    const double scale=std::clamp(std::max(1.,inlier_rmse_/0.10)/std::max(0.25,inlier_ratio_),1.,4.);
    for (const auto& entry : std::vector<std::pair<int,double>>{{0,.15},{7,.15},{14,.25},{21,.12},{28,.12},{35,.15}})
      message.pose.covariance[entry.first] = std::pow(entry.second*scale,2);
    pose_publisher_->publish(message);
    if(verified_publisher_){
      std::ostringstream data;data<<std::setprecision(17);
      data<<"{\"schema\":1,\"seed_ns\":\""<<active_seed_ns_<<"\",\"stamp_ns\":\""<<stamp.nanoseconds()
          <<"\",\"frame\":"<<std::quoted(world_frame_)<<",\"confirmations\":"<<gate_.confirmationCount()
          <<",\"position\":["<<pose(0,3)<<","<<pose(1,3)<<","<<pose(2,3)
          <<"],\"orientation\":["<<orientation.x()<<","<<orientation.y()<<","<<orientation.z()<<","<<orientation.w()<<"],\"covariance\":[";
      for(size_t i=0;i<36;++i){if(i)data<<",";data<<message.pose.covariance[i];}
      data<<"]}";std_msgs::msg::String verified;verified.data=data.str();verified_publisher_->publish(verified);
    }
  }

  void pruneImuBuffer() {
    if (imu_buffer_.size() < 3) {
      return;
    }
    int64_t keep_after_ns =
        imu_buffer_.back().stamp_ns -
        static_cast<int64_t>(imu_buffer_duration_sec_ * 1e9);
    if (!pending_scans_.empty()) {
      int64_t pending_start_ns = 0;
      int64_t pending_end_ns = 0;
      if (scanTimeRange(*pending_scans_.front(), &pending_start_ns,
                        &pending_end_ns)) {
        keep_after_ns = std::min(
            keep_after_ns,
            pending_start_ns -
                static_cast<int64_t>(max_imu_gap_sec_ * 1e9));
      }
    }
    while (imu_buffer_.size() > 2 &&
           imu_buffer_[1].stamp_ns < keep_after_ns) {
      imu_buffer_.pop_front();
    }
  }

  std::string map_path_;
  std::string scan_topic_;
  std::string imu_topic_;
  std::string world_frame_;
  std::string tracking_frame_;
  std::string lidar_frame_;
  std::string initial_pose_topic_;
  std::string prediction_topic_;
  std::string output_pose_topic_;
  std::string map_cloud_topic_;
  std::string raw_scan_topic_;
  std::string leveled_scan_topic_;

  double voxel_leaf_{0.15};
  double max_corr_dist_{1.5};
  double max_translation_delta_{0.45};
  double max_translation_z_delta_{0.35};
  double max_yaw_delta_{0.60};
  double max_tilt_{0.35};
  double max_rotation_delta_{0.70};
  double max_fitness_score_{1.0};
  int max_iterations_{15};
  int num_threads_{4};
  std::string preview_scan_topic_,match_reason_="等待初始位姿";
  uint64_t match_attempts_=0,non_converged_=0,source_points_=0;
  double elapsed_ms_=0.;
  double quality_distance_{.50},min_inlier_ratio_{.35},max_inlier_rmse_{.30},confirmation_max_gap_sec_{.35};
  double inlier_ratio_{0.},inlier_rmse_{0.};
  int64_t last_candidate_stamp_ns_{0};
  int64_t active_seed_ns_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr verified_publisher_;
  pcl::KdTreeFLANN<pcl::PointXYZ> quality_tree_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  double acquisition_max_corr_dist_{3.0};
  double acquisition_max_translation_delta_{2.5};
  double acquisition_max_translation_z_delta_{1.0};
  double acquisition_max_yaw_delta_{1.57};
  double acquisition_max_tilt_{0.35};
  double acquisition_max_rotation_delta_{1.80};
  double acquisition_max_fitness_score_{1.0};
  int acquisition_max_iterations_{50};
  int acquisition_confirmation_count_{3};
  double prediction_timeout_sec_{0.25};
  double prediction_buffer_duration_sec_{2.0};
  bool deskew_enabled_{true};
  bool input_deskewed_{false};
  double min_information_ratio_{0},information_ratio_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_subscription_;
  bool translation_deskew_enabled_{true};
  double max_velocity_gap_sec_{.06};
  std::string velocity_topic_;
  double max_imu_gap_sec_{0.02};
  double max_scan_duration_sec_{0.15};
  double max_scan_age_sec_{0.50};
  double imu_buffer_duration_sec_{2.0};

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> matcher_;
  FusedIcpGate gate_;
  Eigen::Matrix4d last_pose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix3d rotation_lidar_from_imu_{Eigen::Matrix3d::Identity()};
  std::deque<AngularVelocitySample> imu_buffer_;
  std::deque<LinearVelocitySample> velocity_buffer_;
  std::deque<TimedPose> prediction_buffer_;
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> pending_scans_;
  std::mutex sensor_mutex_;
  bool imu_ready_{false};
  bool initialized_{false};
  int64_t last_imu_stamp_ns_{0};
  int64_t last_scan_stamp_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      prediction_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      leveled_scan_publisher_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr scan_processing_timer_;
};

}  // namespace d1max_localization

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<d1max_localization::FusedIcpMatcher>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("fused_icp_matcher"), "%s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
