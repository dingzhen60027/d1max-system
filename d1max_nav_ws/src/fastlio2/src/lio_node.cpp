#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    std::string lidar_input_type = "livox_custom";
    bool print_time_cost = false;
    int imu_queue_depth = 2000;
    int lidar_queue_depth = 100;
    int path_publish_stride = 5;
    int path_max_poses = 2000;
    double imu_acc_scale = 9.80665;
    double max_scan_duration = 0.2;
    double point_time_scale_to_ms = 1.0;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_lidar_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_processing_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = m_imu_callback_group;
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = m_lidar_callback_group;
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic,
            rclcpp::QoS(rclcpp::KeepLast(m_node_config.imu_queue_depth)).reliable(),
            std::bind(&LIONode::imuCB, this, std::placeholders::_1), imu_options);
        if (m_node_config.lidar_input_type == "pointcloud2")
        {
            m_pointcloud_lidar_sub =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    m_node_config.lidar_topic,
                    rclcpp::QoS(
                        rclcpp::KeepLast(m_node_config.lidar_queue_depth))
                        .reliable(),
                    std::bind(
                        &LIONode::pointCloudLidarCB, this,
                        std::placeholders::_1),
                    lidar_options);
        }
        else
        {
            m_lidar_sub =
                this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    m_node_config.lidar_topic,
                    rclcpp::QoS(
                        rclcpp::KeepLast(m_node_config.lidar_queue_depth))
                        .reliable(),
                    std::bind(
                        &LIONode::livoxLidarCB, this,
                        std::placeholders::_1),
                    lidar_options);
        }
        RCLCPP_INFO(
            this->get_logger(), "LiDAR input: type=%s topic=%s",
            m_node_config.lidar_input_type.c_str(),
            m_node_config.lidar_topic.c_str());

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(
            20ms, std::bind(&LIONode::timerCB, this), m_processing_callback_group);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        if (config["lidar_input_type"])
            m_node_config.lidar_input_type =
                config["lidar_input_type"].as<std::string>();
        if (config["imu_queue_depth"])
            m_node_config.imu_queue_depth = config["imu_queue_depth"].as<int>();
        if (config["lidar_queue_depth"])
            m_node_config.lidar_queue_depth = config["lidar_queue_depth"].as<int>();
        if (config["imu_acc_scale"])
            m_node_config.imu_acc_scale = config["imu_acc_scale"].as<double>();
        if (config["path_publish_stride"])
            m_node_config.path_publish_stride = config["path_publish_stride"].as<int>();
        if (config["path_max_poses"])
            m_node_config.path_max_poses = config["path_max_poses"].as<int>();
        if (config["max_scan_duration"])
            m_node_config.max_scan_duration = config["max_scan_duration"].as<double>();
        if (config["point_time_scale_to_ms"])
            m_node_config.point_time_scale_to_ms =
                config["point_time_scale_to_ms"].as<double>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        if (config["imu_init_acc_norm_min"])
            m_builder_config.imu_init_acc_norm_min = config["imu_init_acc_norm_min"].as<double>();
        if (config["imu_init_acc_norm_max"])
            m_builder_config.imu_init_acc_norm_max = config["imu_init_acc_norm_max"].as<double>();
        if (config["imu_init_acc_std_max"])
            m_builder_config.imu_init_acc_std_max = config["imu_init_acc_std_max"].as<double>();
        if (config["imu_init_gyro_std_max"])
            m_builder_config.imu_init_gyro_std_max = config["imu_init_gyro_std_max"].as<double>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        if (config["min_effective_points"])
            m_builder_config.min_effective_points = config["min_effective_points"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        if (t_il_vec.size() != 3 || r_il_vec.size() != 9)
            throw std::runtime_error("FAST-LIO2 t_il/r_il dimensions are invalid");
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        if (m_node_config.imu_queue_depth < 1 || m_node_config.lidar_queue_depth < 1 ||
            m_node_config.path_publish_stride < 1 || m_node_config.path_max_poses < 1 ||
            m_node_config.imu_acc_scale <= 0.0 || m_node_config.max_scan_duration <= 0.0 ||
            m_node_config.point_time_scale_to_ms <= 0.0 ||
            (m_node_config.lidar_input_type != "livox_custom" &&
             m_node_config.lidar_input_type != "pointcloud2") ||
            m_builder_config.lidar_filter_num < 1 ||
            m_builder_config.lidar_min_range < 0.0 ||
            m_builder_config.lidar_max_range <= m_builder_config.lidar_min_range ||
            m_builder_config.scan_resolution <= 0.0 || m_builder_config.map_resolution <= 0.0 ||
            m_builder_config.imu_init_num < 10 || m_builder_config.near_search_num < 3 ||
            m_builder_config.min_effective_points < 3 ||
            m_builder_config.ieskf_max_iter < 1 ||
            m_builder_config.imu_init_acc_norm_min <= 0.0 ||
            m_builder_config.imu_init_acc_norm_max <= m_builder_config.imu_init_acc_norm_min ||
            m_builder_config.imu_init_acc_std_max <= 0.0 ||
            m_builder_config.imu_init_gyro_std_max <= 0.0)
        {
            throw std::runtime_error("invalid FAST-LIO2 configuration");
        }
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const V3D acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                               msg->linear_acceleration.z);
        const V3D angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y,
                                  msg->angular_velocity.z);
        if (!acceleration.allFinite() || !angular_velocity.allFinite())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring non-finite IMU sample");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
            double timestamp = Utils::getSec(msg->header);
            if (timestamp < m_state_data.last_imu_time)
            {
                RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
                std::deque<IMUData>().swap(m_state_data.imu_buffer);
            }
            m_state_data.imu_buffer.emplace_back(acceleration * m_node_config.imu_acc_scale,
                                                 angular_velocity,
                                                 timestamp);
            m_state_data.last_imu_time = timestamp;
            if (m_state_data.imu_buffer.size() > static_cast<size_t>(m_node_config.imu_queue_depth))
            {
                while (m_state_data.imu_buffer.size() > static_cast<size_t>(m_node_config.imu_queue_depth))
                    m_state_data.imu_buffer.pop_front();
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "IMU processing backlog reached its limit; dropping oldest samples");
            }
        }
    }
    void livoxLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        if (cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring an empty Livox cloud after filtering");
            return;
        }
        queueLidarCloud(cloud, Utils::getSec(msg->header));
    }

    void pointCloudLidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::pointCloud2ToPCL(
            msg,
            m_builder_config.lidar_filter_num,
            m_node_config.point_time_scale_to_ms,
            m_builder_config.lidar_min_range,
            m_builder_config.lidar_max_range);
        if (cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Ignoring PointCloud2: expected finite x/y/z/intensity/time fields");
            return;
        }
        queueLidarCloud(cloud, Utils::getSec(msg->header));
    }

    void queueLidarCloud(CloudType::Ptr cloud, double timestamp)
    {
        std::sort(cloud->points.begin(), cloud->points.end(), [](const PointType &p1, const PointType &p2)
                  { return p1.curvature < p2.curvature; });
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
            m_state_data.lidar_pushed = false;
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
        if (m_state_data.lidar_buffer.size() > static_cast<size_t>(m_node_config.lidar_queue_depth))
        {
            m_state_data.lidar_pushed = false;
            while (m_state_data.lidar_buffer.size() > static_cast<size_t>(m_node_config.lidar_queue_depth))
                m_state_data.lidar_buffer.pop_front();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "LiDAR processing backlog reached its limit; dropping oldest scans");
        }
    }

    bool syncPackage()
    {
        std::scoped_lock<std::mutex, std::mutex> lock(m_state_data.imu_mutex,
                                                      m_state_data.lidar_mutex);
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            if (!m_package.cloud || m_package.cloud->empty())
            {
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            const double scan_duration = m_package.cloud->points.back().curvature / 1000.0;
            if (!std::isfinite(scan_duration) || scan_duration < 0.0 ||
                scan_duration > m_node_config.max_scan_duration)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Rejecting LiDAR cloud with invalid duration %.3f s",
                                     scan_duration);
                m_package.cloud.reset();
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            m_package.cloud_end_time = m_package.cloud_start_time + scan_duration;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        ++m_path_publish_counter;
        if (m_path_publish_counter % static_cast<size_t>(m_node_config.path_publish_stride) != 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        if (m_state_data.path.poses.size() > static_cast<size_t>(m_node_config.path_max_poses))
        {
            const size_t excess = m_state_data.path.poses.size() -
                                  static_cast<size_t>(m_node_config.path_max_poses);
            m_state_data.path.poses.erase(m_state_data.path.poses.begin(),
                                          m_state_data.path.poses.begin() + excess);
        }
        m_state_data.path.header.stamp = pose.header.stamp;
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() == BuilderStatus::IMU_INIT)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IMU initialization: %s",
                                 m_builder->initializationStatus().c_str());
            return;
        }
        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        if (!m_mapping_ready_logged)
        {
            RCLCPP_INFO(this->get_logger(), "FAST-LIO2 mapping ready; IMU initialization passed");
            m_mapping_ready_logged = true;
        }

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        m_pointcloud_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::CallbackGroup::SharedPtr m_imu_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_lidar_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_processing_callback_group;
    bool m_mapping_ready_logged = false;
    size_t m_path_publish_counter = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LIONode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
