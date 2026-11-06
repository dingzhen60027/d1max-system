#include "plan_env/grid_map.h"
#include <plan_env/voxel_collision.hpp>
#include <plan_env/observed_ray.hpp>
#include <plan_env/parallel_observed_rays.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <type_traits>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rcl/timer.h>

namespace
{
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

GridMap::GridMap() {}

GridMap::~GridMap()
{
  // The executor owner joins all role callbacks before this destructor.
  // Remove the waitable too: the node must not retain a callable capturing
  // this map after the role owner has retired it.
  if(projected_writer_wake_) {
    projected_writer_wake_->close();
    if(node_)node_->get_node_waitables_interface()->remove_waitable(projected_writer_wake_,callback_group_);
  }
  if(occ_timer_)occ_timer_->cancel();
  if(vis_timer_)vis_timer_->cancel();
  stopVisualizationWorker();
}

void GridMap::initMap(rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  node_ = node;
  callback_group_ = std::move(callback_group);

  /* get parameter */
  double x_size, y_size, z_size;
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  load_parameter(node_, "grid_map.local_update_range_x", mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y", mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z", mp_.local_update_range_(2), z_size / 2.0);
  
  load_parameter(node_, "grid_map.obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.planning_obstacles_inflation_z_up", mp_.planning_obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.planning_obstacles_inflation_z_down", mp_.planning_obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  load_parameter(node_, "grid_map.collision_model", mp_.collision_model_, std::string("double_cylinder"));
  load_parameter(node_, "grid_map.box_length", mp_.box_length_, -1.0);
  load_parameter(node_, "grid_map.box_width", mp_.box_width_, -1.0);
  load_parameter(node_, "grid_map.box_nominal_length", mp_.box_nominal_length_, -1.0);
  load_parameter(node_, "grid_map.box_nominal_width", mp_.box_nominal_width_, -1.0);
  load_parameter(node_, "grid_map.stair_planar_clearance", mp_.stair_planar_clearance_, -1.0);
  load_parameter(node_, "grid_map.stair_subvoxel_collision", mp_.stair_subvoxel_collision_, false);
  load_parameter(node_, "grid_map.stair_contact_body_below", mp_.stair_contact_body_below_, -1.0);
  load_parameter(node_, "grid_map.body_below", mp_.body_below_, -1.0);
  load_parameter(node_, "grid_map.body_above", mp_.body_above_, -1.0);
  load_parameter(node_, "grid_map.planning_body_below", mp_.planning_body_below_, -1.0);
  load_parameter(node_, "grid_map.planning_body_above", mp_.planning_body_above_, -1.0);
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);

  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);
  load_parameter(node_, "grid_map.visualization_rate_hz", mp_.visualization_rate_hz_, 3.0);
  if (!std::isfinite(mp_.visualization_rate_hz_) || mp_.visualization_rate_hz_ < 1.0 ||
      mp_.visualization_rate_hz_ > 5.0)
    throw std::invalid_argument("visualization_rate_hz must be between 1 and 5");

  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);

  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_, string("lidar"));
  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);
  load_parameter(node_, "grid_map.strict_input_frames", mp_.strict_input_frames_, false);
  load_parameter(node_, "grid_map.maximum_cloud_pose_dt", mp_.maximum_cloud_pose_dt_, 0.25);
  load_parameter(node_, "grid_map.exact_cloud_pose_sync", mp_.exact_cloud_pose_sync_, mp_.strict_input_frames_);
  // False selects the upstream inflated-buffer collision policy. Sensor input
  // identity, complete per-ray integration and scan freshness are independent.
  load_parameter(node_, "grid_map.require_observed_free", mp_.require_observed_free_, false);
  load_parameter(node_, "grid_map.planning_collision_policy", mp_.planning_collision_policy_, string("official"));
  if (mp_.planning_collision_policy_!="official" && mp_.planning_collision_policy_!="observed_free")
    throw std::invalid_argument("grid_map.planning_collision_policy must be official or observed_free");
  if (mp_.planning_collision_policy_=="observed_free" && !mp_.require_observed_free_)
    throw std::invalid_argument("observed_free planning requires grid_map.require_observed_free=true");
  load_parameter(node_, "grid_map.use_projected_rays", mp_.use_projected_rays_, false);
  load_parameter(node_, "grid_map.preview_only", mp_.preview_only_, false);
  std::vector<std::int64_t> expected_ray_sensors;
  load_parameter(node_, "grid_map.expected_ray_sensor_ids", expected_ray_sensors, std::vector<std::int64_t>{0,1});
  if (mp_.use_projected_rays_ && expected_ray_sensors!=std::vector<std::int64_t>{0,1})
    throw std::invalid_argument("D1 Max projected-ray schema requires both sensor IDs [0,1]");
  load_parameter(node_, "grid_map.cloud_pose_pair_wait", mp_.cloud_pose_pair_wait_, 0.25);
  load_parameter(node_, "grid_map.cloud_pose_max_age", mp_.cloud_pose_max_age_, 0.5);
  load_parameter(node_, "grid_map.require_localization_context", mp_.require_localization_context_, false);
  load_parameter(node_, "grid_map.localization_session_id", mp_.localization_session_id_, std::string{});
  if (mp_.require_localization_context_ &&
      (mp_.localization_session_id_.empty() || (!mp_.exact_cloud_pose_sync_ && !mp_.use_projected_rays_) || mp_.sensor_type_ != "lidar"))
    throw std::invalid_argument("localization context requires an explicit session and exact lidar pairing");
  if (mp_.use_projected_rays_ && (!mp_.require_localization_context_ ||
      !mp_.cloud_is_world_ || mp_.need_extrinsic_))
    throw std::invalid_argument("projected rays require context-tagged map-frame input without another extrinsic");
  scan_planner::validateCloudPoseTiming(mp_.cloud_pose_pair_wait_,mp_.cloud_pose_max_age_,
      mp_.preview_only_,mp_.require_observed_free_,mp_.use_projected_rays_);

  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;

  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
                 mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
  rclcpp::SubscriptionOptions writer_options;
  writer_options.callback_group = callback_group_;

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data, writer_options);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data, writer_options);

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    if (mp_.use_projected_rays_) {
      // Formal sessions register this group with its own serial executor.
      // No-group archived callers preserve their original single-lane path.
      if(callback_group_) {
        projected_ingress_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
        projected_ingress_enabled_=true;
        projected_writer_wake_=std::make_shared<scan_planner::CoalescedWriterWake>(
          node_->get_node_base_interface()->get_context(),[this]{updateProjectedOccupancy(false);});
        node_->get_node_waitables_interface()->add_waitable(projected_writer_wake_,callback_group_);
      }
      auto ingress_options=writer_options;
      if(projected_ingress_group_)ingress_options.callback_group=projected_ingress_group_;
      projected_rays_sub_=node_->create_subscription<d1max_planning_interfaces::msg::ProjectedRays>(
          "projected_rays",rclcpp::SensorDataQoS().keep_last(scan_planner::kProjectedRayIngressDepth),
          [this](const d1max_planning_interfaces::msg::ProjectedRays::ConstSharedPtr message) {
            if(projected_ingress_enabled_) {
              projected_ingress_.recordCallback();
              projected_ingress_.accept(*message,node_->now().nanoseconds(),std::chrono::steady_clock::now());
              drainProjectedIngress();
              // Coalesce readiness into ONE turn of the existing writer,
              // never an integration/timer mutation on the receiver lane.
              // Original watchdog remains a fallback if guard trigger fails.
              projected_writer_wake_->notify();
              return;
            }
            ++ray_ingress_callback_count_;
            acceptProjectedRays(*message,node_->now().nanoseconds(),std::chrono::steady_clock::now());
            drainProjectedIngress();
            tryProjectedFusion(node_->now().nanoseconds(),std::chrono::steady_clock::now(),false);
            publishProjectedRaysStatus(node_->now().nanoseconds(),std::chrono::steady_clock::now());
            scheduleProjectedFusionWake();
          }, ingress_options);
      projected_rays_status_pub_=node_->create_publisher<std_msgs::msg::String>(
          "grid_map/projected_rays_status",rclcpp::QoS(1).reliable());
    } else {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1), writer_options);
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1), writer_options);
    }
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1), writer_options);

  if (!node_->has_parameter("grid_map.integration_rate_hz"))
    node_->declare_parameter<double>("grid_map.integration_rate_hz",5.0);
  const double integration_rate=node_->get_parameter("grid_map.integration_rate_hz").as_double();
  if (!std::isfinite(integration_rate) || integration_rate<=0. || integration_rate>20.)
    throw std::invalid_argument("invalid grid map integration rate");
  ray_integration_period_s_=1./integration_rate;
  // The cheap watchdog can notice a just-completed pair at the rate boundary;
  // actual ray integration remains capped by tryProjectedFusion's steady clock.
  occ_timer_ = node_->create_wall_timer(std::chrono::duration<double>(mp_.use_projected_rays_?.02:ray_integration_period_s_),
                                        std::bind(&GridMap::updateOccupancyCallback, this), callback_group_);
  vis_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / mp_.visualization_rate_hz_),
                                        std::bind(&GridMap::visCallback, this), callback_group_);

  if (mp_.require_localization_context_)
  {
    const auto context_qos = rclcpp::QoS(1).reliable().transient_local();
    localization_context_ack_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "grid_map/localization_context_ack", context_qos);
    localization_context_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "grid_map/localization_context", context_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (applyLocalizationContext(msg->data)) localization_context_ack_pub_->publish(*msg);
        }, writer_options);
  }

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", rclcpp::SensorDataQoS());
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", rclcpp::SensorDataQoS());
  sliding_map_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("grid_map/sensor_pose_extrinsic", 10);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets()
{
  if (mp_.collision_model_!="double_cylinder" && mp_.collision_model_!="oriented_box")
    throw std::invalid_argument("grid_map.collision_model must be double_cylinder or oriented_box");
  for (double extent:{mp_.body_below_,mp_.body_above_,mp_.planning_body_below_,mp_.planning_body_above_})
    if (!std::isfinite(extent) || (extent!=-1. && extent<0.))
      throw std::invalid_argument("box body extent must be -1 or finite nonnegative");
  const auto strict_body=bodyEnvelope(),planning_body=planningBodyEnvelope();
  scan_planner::validateBodyEnvelope(strict_body);
  scan_planner::validateBodyEnvelope(planning_body);
  if(!std::isfinite(mp_.stair_contact_body_below_) ||
      (mp_.stair_contact_body_below_!=-1. && (mp_.stair_contact_body_below_<0. ||
        mp_.stair_contact_body_below_>std::min(strict_body.below,planning_body.below) ||
        !mp_.stair_subvoxel_collision_ || strict_body.model!=scan_planner::CollisionModel::OrientedBox)))
    throw std::invalid_argument("stair contact requires a contained rigid box and original-hit evidence");
  if(!std::isfinite(mp_.stair_planar_clearance_) || mp_.stair_planar_clearance_<-1. ||
      (mp_.stair_planar_clearance_<0. && mp_.stair_planar_clearance_!=-1.))
    throw std::invalid_argument("stair planar clearance must be -1 or finite nonnegative");
  if(stairClearanceConfigured()) {
    if(strict_body.model!=scan_planner::CollisionModel::OrientedBox ||
        !std::isfinite(mp_.box_nominal_length_) || mp_.box_nominal_length_<=0. ||
        !std::isfinite(mp_.box_nominal_width_) || mp_.box_nominal_width_<=0. ||
        mp_.box_nominal_length_+2.*mp_.stair_planar_clearance_>strict_body.length+1e-12 ||
        mp_.box_nominal_width_+2.*mp_.stair_planar_clearance_>strict_body.width+1e-12)
      throw std::invalid_argument("stair box must retain positive nominal dimensions within normal box clearance");
  }
  if (planning_body.below>strict_body.below || planning_body.above>strict_body.above)
    throw std::invalid_argument("candidate body envelope must be contained by execution envelope");
  if (strict_body.model==scan_planner::CollisionModel::OrientedBox) {
    const double extent=scan_planner::bodyExtent(strict_body);
    const double xy=std::ceil(2.*extent/mp_.resolution_)+3.;
    const double z=std::ceil((strict_body.below+strict_body.above)/mp_.resolution_)+3.;
    if (!std::isfinite(mp_.resolution_) || mp_.resolution_<=0. || !std::isfinite(xy*z) || xy*xy*z>200000.)
      throw std::invalid_argument("oriented box footprint exceeds bounded voxel query budget");
  }
  const auto previous_planning_offsets=md_.planning_inflate_offsets_;
  {
    // Upstream ROS 1 348e8a59 / ros2-community d0b921c9 discretization.
    // In particular, XY offsets at exactly the radius are excluded; this is
    // not the larger voxel-AABB kernel retained by the optional strict policy.
    const double radius=std::max(0.0,mp_.double_cylinder_radius_);
    if (!std::isfinite(mp_.resolution_) || mp_.resolution_<=0. ||
        !std::isfinite(mp_.double_cylinder_radius_) ||
        !std::isfinite(mp_.obstacles_inflation_z_up) || mp_.obstacles_inflation_z_up<0. ||
        !std::isfinite(mp_.obstacles_inflation_z_down) || mp_.obstacles_inflation_z_down<0.)
      throw std::invalid_argument("invalid cylinder voxel geometry");
    const auto validate_planning=[](double value,double strict_value) {
      if (!std::isfinite(value) || (value!=-1.0 && (value<0. || value>strict_value)))
        throw std::invalid_argument("planning cylinder extent must be -1 or nonnegative and no larger than strict extent");
    };
    validate_planning(mp_.planning_obstacles_inflation_z_up,mp_.obstacles_inflation_z_up);
    validate_planning(mp_.planning_obstacles_inflation_z_down,mp_.obstacles_inflation_z_down);
    const auto planning=planningObstacleDilation();
    // The legacy non-strict buffer is also exposed by getInflateOccupancy.
    // Do not narrow it unless the independent strict raw-evidence layer exists.
    if (!mp_.require_observed_free_ &&
        (planning_body.below<strict_body.below || planning_body.above<strict_body.above))
      throw std::invalid_argument("narrow planning geometry requires independent strict evidence layer");
    const double xy=std::ceil(radius/mp_.resolution_);
    const double up=std::ceil(planning.up/mp_.resolution_);
    const double down=std::ceil(planning.down/mp_.resolution_);
    if ((2.*xy+1.)*(2.*xy+1.)*(up+down+1.)>200000.)
      throw std::invalid_argument("cylinder voxel kernel exceeds bounded budget");
    md_.planning_inflate_offsets_.clear();
    for (int x=-static_cast<int>(xy);x<=static_cast<int>(xy);++x)
      for (int y=-static_cast<int>(xy);y<=static_cast<int>(xy);++y) {
        const Eigen::Vector2d offset_xy(x*mp_.resolution_,y*mp_.resolution_);
        if (offset_xy.norm()>=radius) continue;
        for (int z=-static_cast<int>(down);z<=static_cast<int>(up);++z)
          md_.planning_inflate_offsets_.emplace_back(x,y,z);
      }
  }
  md_.planning_inflate_columns_=inflationColumns(md_.planning_inflate_offsets_);
  if (mp_.require_observed_free_) {
    md_.inflate_offsets_=scan_planner::conservativeCylinderInflation(
        mp_.resolution_,mp_.double_cylinder_radius_,
        mp_.obstacles_inflation_z_up,mp_.obstacles_inflation_z_down);
    md_.inflate_columns_=inflationColumns(md_.inflate_offsets_);
    // Only the fusion owner has incremental counters. Private strict proof
    // snapshots preserve copied geometry without allocating another writer.
    const bool planning_offsets_changed=previous_planning_offsets.size()!=md_.planning_inflate_offsets_.size() ||
      !std::equal(previous_planning_offsets.begin(),previous_planning_offsets.end(),
        md_.planning_inflate_offsets_.begin(),[](const auto &a,const auto &b){return (a.array()==b.array()).all();});
    if (!collision_snapshot_ && (planning_offsets_changed ||
        md_.planning_occupancy_buffer_inflate_.size()!=md_.occupancy_buffer_.size())) {
      md_.planning_occupancy_buffer_inflate_.assign(md_.occupancy_buffer_.size(),0);
      md_.planning_occupancy_buffer_inflate_cnt_.assign(md_.occupancy_buffer_.size(),0);
      for (std::size_t address=0;address<md_.occupancy_buffer_.size();++address) {
        if (md_.occupancy_buffer_[address]<=mp_.min_occupancy_log_) continue;
        Eigen::Vector3i id;hashIdToGlobalIndex(static_cast<int>(address),id);
        updateInflationLayer(id,1,md_.planning_inflate_columns_,md_.planning_occupancy_buffer_inflate_cnt_,
            md_.planning_occupancy_buffer_inflate_,nullptr);
      }
    }
  } else {
    md_.inflate_offsets_=md_.planning_inflate_offsets_;
    md_.inflate_columns_=md_.planning_inflate_columns_;
  }
  invalidateCollisionQueryCaches();
}

std::int64_t GridMap::collisionQueryClock()
{
  if (collision_snapshot_) {
    const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-snapshot_captured_).count();
    if (snapshot_clock_ns_<=0 || elapsed<0 ||
        elapsed>std::numeric_limits<std::int64_t>::max()-snapshot_clock_ns_) return 0;
    return snapshot_clock_ns_+elapsed;
  }
  if (!node_) return ray_query_clock_ns_; // deterministic offline injected clock
  return collisionQueryClockAt(node_->now().nanoseconds(),std::chrono::steady_clock::now());
}

void GridMap::copyCollisionSnapshotTo(GridMap &out,std::int64_t source_now_ns,
    std::chrono::steady_clock::time_point captured) const
{
  if(visualization_only_ || out.visualization_only_)
    throw std::logic_error("render-only state cannot become a collision snapshot");
  if (&out==this || out.node_) throw std::invalid_argument("snapshot destination must be a distinct memory-only map");
  out.mp_=mp_;
  out.md_.occupancy_buffer_=md_.occupancy_buffer_;
  out.md_.occupied_hit_subvoxels_=md_.occupied_hit_subvoxels_;
  out.md_.occupied_hit_bounds_=md_.occupied_hit_bounds_;
  out.md_.occupancy_buffer_inflate_=md_.occupancy_buffer_inflate_;
  out.md_.planning_occupancy_buffer_inflate_=md_.planning_occupancy_buffer_inflate_;
  // Incremental inflation counters belong to fusion only. Collision readers
  // use the committed flags/raw evidence and never update the map. Do not
  // allocate/copy four bytes per voxel into each of the three read slots.
  if(!out.md_.occupancy_buffer_inflate_cnt_.empty())
    std::vector<int>().swap(out.md_.occupancy_buffer_inflate_cnt_);
  if(!out.md_.planning_occupancy_buffer_inflate_cnt_.empty())
    std::vector<int>().swap(out.md_.planning_occupancy_buffer_inflate_cnt_);
  out.md_.inflate_offsets_=md_.inflate_offsets_;
  out.md_.planning_inflate_offsets_=md_.planning_inflate_offsets_;
  out.md_.inflate_columns_=md_.inflate_columns_;
  out.md_.planning_inflate_columns_=md_.planning_inflate_columns_;
  out.free_observation_stamps_=free_observation_stamps_;
  out.integrated_cloud_stamp_ns_=integrated_cloud_stamp_ns_;
  out.ray_integrated_stamps_=ray_integrated_stamps_;
  out.fusion_timing_=fusion_timing_;
  out.ray_integrated_receipts_=ray_integrated_receipts_;
  out.localization_context_sequence_=localization_context_sequence_;
  out.localization_epoch_=localization_epoch_;
  out.localization_seed_=localization_seed_;
  out.localization_context_barrier_ns_=localization_context_barrier_ns_;
  out.localization_context_payload_=localization_context_payload_;
  out.occupancy_revision_=occupancy_revision_;
  out.free_evidence_revision_=free_evidence_revision_;
  out.enforce_free_freshness_=enforce_free_freshness_;
  out.ray_clock_fault_=ray_clock_fault_;
  out.blocked_voxel_context_=blocked_voxel_context_;
  out.blocked_voxel_mailbox_=blocked_voxel_mailbox_;
  out.blocked_voxel_watch_snapshot_=blocked_voxel_watch_snapshot_;
  out.collision_snapshot_=true;
  out.parallel_ray_visits_.reset();
  out.last_parallel_ray_visits_=false;
  out.query_attitude_provider_={}; // A new snapshot cannot inherit another curve's attitude contract.
  out.query_support_height_provider_={};
  out.query_stair_clearance_provider_={};
  out.query_stair_contact_provider_={};
  out.query_hard_boundary_provider_={};
  out.query_hard_boundary_diagnostic_provider_={};
  out.snapshot_clock_ns_=mp_.use_projected_rays_ ? collisionQueryClockAt(source_now_ns,captured) : source_now_ns;
  out.snapshot_captured_=captured;
  out.invalidateCollisionQueryCaches();
  out.collision_cache_deadline_ns_=out.collision_cache_clock_ns_=0;
  out.unknown_collision_queries_=0;
  // No copied callback, ROS clock owner, pending acquisition, visualization or
  // shared mutable cache. Source acquisition stamps above remain unchanged.
}

std::int64_t GridMap::collisionQueryClockAt(std::int64_t source,PairReceipt now) const
{
  if(collision_snapshot_) {
    // Solver acquisition may copy an exclusively borrowed, completed owner
    // snapshot rather than touch the live writer. Such a map intentionally
    // has no writer tick/receipt state. Carry its already aged effective
    // clock forward instead of fabricating a fresh observation or resetting
    // the clock to zero. All per-ray/cloud/free acquisition stamps stay fixed.
    const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
        now-snapshot_captured_).count();
    if(source<=0 || snapshot_clock_ns_<=0 || elapsed<0 ||
       elapsed>std::numeric_limits<std::int64_t>::max()-snapshot_clock_ns_)return 0;
    return std::max(source,snapshot_clock_ns_+elapsed);
  }
  if (source<=0 || ray_tick_clock_ns_<=0 || source<ray_tick_clock_ns_) return 0;
  const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
      now-ray_tick_receipt_).count();
  if (elapsed<0 || elapsed>std::numeric_limits<std::int64_t>::max()-ray_tick_effective_ns_) return 0;
  // A paused source clock cannot preserve old free evidence indefinitely.
  return std::max(source,ray_tick_effective_ns_+elapsed);
}

void GridMap::advanceRayEvidenceClock(std::int64_t source,PairReceipt now,bool age_by_steady)
{
  const auto prior_effective=age_by_steady?collisionQueryClockAt(source,now):source;
  if(ray_tick_clock_ns_>0 && source<ray_tick_clock_ns_) {
    ray_clock_fault_=true;invalidateCollisionQueryCaches();
  }
  ray_tick_effective_ns_=std::max(source,prior_effective);
  ray_tick_clock_ns_=source;ray_tick_receipt_=now;ray_query_clock_ns_=source;
}

void GridMap::beginCollisionQuery()
{
  if (!mp_.require_observed_free_ || !mp_.use_projected_rays_ || !enforce_free_freshness_) return;
  const auto now=collisionQueryClock();
  if (collision_cache_clock_ns_>0 && (now<=0 || (!node_ && now<collision_cache_clock_ns_)))
    ray_clock_fault_=true;
  if (now<=0 || now<collision_cache_clock_ns_ || now>=collision_cache_deadline_ns_) {
    invalidateCollisionQueryCaches();
    collision_cache_deadline_ns_=std::numeric_limits<std::int64_t>::max();
  }
  collision_cache_clock_ns_=now>0?std::max(now,collision_cache_clock_ns_):now;
}

int GridMap::rawCollisionStatus(const Eigen::Vector3i &cell)
{
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if (!isInMap(cell)) return -1;
  const auto address=toAddress(cell);
  const double odds=md_.occupancy_buffer_[address];
  // Raw diagnostics retain unknown/insufficient evidence under either policy.
  // Official collision queries read inflation directly and never use this
  // status to turn unobserved map contents into measured free cells.
  const int raw=scan_planner::strictRawVoxelStatus(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_);
  if (raw!=0 || !mp_.require_observed_free_ || !mp_.use_projected_rays_ || !enforce_free_freshness_) return raw;
  if (ray_clock_fault_) return 2;
  const auto stamp=static_cast<std::size_t>(address)<free_observation_stamps_.size()?
      free_observation_stamps_[address]:0;
  const auto now=collision_cache_clock_ns_;
  // No future tolerance for FREE evidence, no expiry of OCCUPIED to free.
  if (stamp<=0 || now<stamp || !std::isfinite(mp_.cloud_pose_max_age_) ||
      mp_.cloud_pose_max_age_<=0. || mp_.cloud_pose_max_age_>.5) return 2;
  const auto age_limit=static_cast<std::int64_t>(mp_.cloud_pose_max_age_*1e9);
  if (now-stamp>=age_limit) return 2;
  if (stamp<=std::numeric_limits<std::int64_t>::max()-age_limit)
    collision_cache_deadline_ns_=std::min(collision_cache_deadline_ns_,stamp+age_limit);
  return 0;
}

int GridMap::collisionSnapshotStatus(const Eigen::Vector3i& cell,int* raw_status)
{
  if(!collision_snapshot_||node_||!mp_.use_projected_rays_)
    throw std::logic_error("SCAN swept query requires private raw-ray snapshot");
  if(!isInMap(cell)) {if(raw_status)*raw_status=-1;return -1;}
  const double odds=md_.occupancy_buffer_[toAddress(cell)];
  if(planningRequiresObservedFree()) {
    if(!mp_.require_observed_free_)throw std::logic_error("strict SCAN query requires observed snapshot");
    if(collision_cache_clock_ns_==0)beginCollisionQuery();
    const int state=rawCollisionStatus(cell);if(raw_status)*raw_status=state;return state;
  }
  if(mp_.planning_collision_policy_!="official")
    throw std::logic_error("unknown SCAN collision policy");
  const int raw=scan_planner::strictRawVoxelStatus(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_);
  if(raw_status)*raw_status=raw;
  // Invalid numeric map contents are not ordinary unknown observations.
  return !std::isfinite(odds)?2:odds>mp_.min_occupancy_log_?1:0;
}

int GridMap::observedCylinderStatus(const Eigen::Vector3d &center)
{
  if (!center.allFinite() || !isInMap(center)) return -1;
  Eigen::Vector3i id;
  posToIndex(center,id);
  const int address=toAddress(id);
  const auto body=bodyEnvelope();
  const auto span=scan_planner::verticalVoxelSpan(center.z(),body.below,body.above,mp_.resolution_);
  const int state=observed_cylinder_cache_.getExact(address,span.low,span.high,center.x(),center.y(),[&]() {
    return scan_planner::observedCylinderStatusAtPosition(center,id,span,md_.inflate_offsets_,
        mp_.resolution_,mp_.double_cylinder_radius_,
        [this](const Eigen::Vector3i &cell) {
          return rawCollisionStatus(cell);
        });
  });
  if (state==2) ++unknown_collision_queries_;
  return state;
}

int GridMap::boxOccupancy(const Eigen::Vector3d &position,double yaw,bool planning)
{
  if (!position.allFinite() || !std::isfinite(yaw) || !isInMap(position)) return -1;
  const auto body=queryBodyEnvelope(position,planning);
  if(query_hard_boundary_provider_&&query_hard_boundary_provider_(position,yaw,0.,0.,body))return 1;
  const bool strict=!planning && mp_.require_observed_free_;
  if (strict) beginCollisionQuery();
  const bool refine_vertical=stairSubvoxelQueryAllowed(position,strict);
  if(refine_vertical||(!strict&&stairContactConfigured()&&query_stair_contact_provider_)) {
    // Fine evidence makes exact body Z relevant even while its coarse Z span
    // stays unchanged. Do not use the legacy span-only upright pose cache.
    const scan_planner::OrientedBoxVoxel3DQuery fine_query(position,yaw,0.,0.,mp_.resolution_,body);
    std::optional<scan_planner::OrientedBoxVoxel3DQuery> rigid_query;
    if(stairContactConfigured()&&query_stair_contact_provider_) {
      auto rigid=body;rigid.below=mp_.stair_contact_body_below_;
      rigid_query.emplace(position,yaw,0.,0.,mp_.resolution_,rigid);
    }
    return scan_planner::orientedBoxStatus(position,yaw,mp_.resolution_,body,
      [&](const Eigen::Vector3i& cell) {
        if(!isInMap(cell))return -1;
        const int address=toAddress(cell);const double odds=md_.occupancy_buffer_[address];
        if(!std::isfinite(odds))return 2;
        if(odds<=mp_.min_occupancy_log_)return 0;
        return occupiedSubvoxelIntersects(cell,address,fine_query,nullptr,
          rigid_query?&*rigid_query:nullptr,refine_vertical)?1:0;
      });
  }
  Eigen::Vector3i index;posToIndex(position,index);
  const auto span=scan_planner::verticalVoxelSpan(position.z(),body.below,body.above,mp_.resolution_);
  auto &cache=strict ? observed_cylinder_cache_ : planning_box_cache_;
  const int state=cache.getPoseShapeExact(toAddress(index),span.low,span.high,
      position.x(),position.y(),yaw,body.length,body.width,[&]() {
    if (!strict) {
      // Same exact OBB voxel set and official occupied-only semantics as the
      // per-voxel query below. Every intersecting XY column shares a Z span:
      // check bounds/address once, then traverse the contiguous ring row.
      // No auxiliary map, yaw bin, stale cache or free evidence is introduced.
      int result=0;
      const int z_size=mp_.map_voxel_num_.z(),yz_stride=mp_.map_voxel_num_.y()*z_size;
      const auto &lo=mp_.map_bound_min_idx_,&hi=mp_.map_bound_max_idx_;
      scan_planner::forEachOrientedBoxColumn(position,yaw,mp_.resolution_,body,
        [&](int x,int y,const scan_planner::VerticalVoxelSpan &zs) {
          if (x<lo.x()||x>hi.x()||y<lo.y()||y>hi.y()) {result=-1;return true;}
          if (zs.low<lo.z()||zs.high>hi.z()) result=-1;
          const int first=std::max(zs.low,lo.z()),last=std::min(zs.high,hi.z());
          if (first>last) return true;
          const int row=getLocalIndex(x,0)*yz_stride+getLocalIndex(y,1)*z_size;
          int local_z=getLocalIndex(first,2);
          for (std::int64_t z=first;z<=last;++z) {
            const double odds=md_.occupancy_buffer_[row+local_z];
            if (std::isfinite(odds)) {
              if (odds>mp_.min_occupancy_log_) {result=1;return false;}
            } else if (result==0) result=2;
            if (++local_z==z_size) local_z=0;
          }
          return true;
        });
      return result;
    }
    return scan_planner::orientedBoxStatus(position,yaw,mp_.resolution_,body,
      [this,strict](const Eigen::Vector3i &cell) {
        if (!isInMap(cell)) return -1;
        if (strict) return rawCollisionStatus(cell);
        // Official candidate semantics: occupied only, without claiming that
        // unseen/weak evidence was observed free. Unlike a circular inflated
        // buffer, these cells are intersected by the ACTUAL requested OBB yaw.
        const double odds=md_.occupancy_buffer_[toAddress(cell)];
        return !std::isfinite(odds) ? 2 : odds>mp_.min_occupancy_log_ ? 1 : 0;
      });
  });
  if (strict && state==2) ++unknown_collision_queries_;
  if (strict && state==0 && mp_.use_projected_rays_ && enforce_free_freshness_) {
    const auto now=collisionQueryClock();
    if (ray_clock_fault_ || now<=0 || now<collision_cache_clock_ns_ ||
        now>=collision_cache_deadline_ns_) return 2;
  }
  return state;
}

GridMap::QueryAttitudeProvider GridMap::exchangeQueryAttitudeProvider(QueryAttitudeProvider provider)
{
  if(!provider&&!query_attitude_provider_)return {}; // Flat/no-profile scope is exactly a no-op.
  if(node_||visualization_only_)throw std::logic_error("query attitude requires a private collision reader");
  auto previous=std::move(query_attitude_provider_);query_attitude_provider_=std::move(provider);
  observed_cylinder_cache_.clear();planning_box_cache_.clear();
  return previous;
}

void GridMap::setQueryAttitudeProvider(QueryAttitudeProvider provider)
{exchangeQueryAttitudeProvider(std::move(provider));}
void GridMap::clearQueryAttitudeProvider()
{exchangeQueryAttitudeProvider({});}

GridMap::QuerySupportHeightProvider GridMap::exchangeQuerySupportHeightProvider(QuerySupportHeightProvider provider)
{
  if(!provider&&!query_support_height_provider_)return {};
  if(node_||visualization_only_)throw std::logic_error("query support requires a private collision reader");
  auto previous=std::move(query_support_height_provider_);
  query_support_height_provider_=std::move(provider);return previous;
}

GridMap::QueryStairClearanceProvider GridMap::exchangeQueryStairClearanceProvider(QueryStairClearanceProvider provider)
{
  if(!provider&&!query_stair_clearance_provider_)return {};
  if(node_||visualization_only_)throw std::logic_error("query clearance requires a private collision reader");
  auto previous=std::move(query_stair_clearance_provider_);
  query_stair_clearance_provider_=std::move(provider);invalidateCollisionQueryCaches();return previous;
}

GridMap::QueryStairContactProvider GridMap::exchangeQueryStairContactProvider(QueryStairContactProvider provider)
{
  if(!provider&&!query_stair_contact_provider_)return {};
  if(node_||visualization_only_)throw std::logic_error("query contacts require a private collision reader");
  auto previous=std::move(query_stair_contact_provider_);
  query_stair_contact_provider_=std::move(provider);invalidateCollisionQueryCaches();return previous;
}

GridMap::QueryHardBoundaryProvider GridMap::exchangeQueryHardBoundaryProvider(QueryHardBoundaryProvider provider)
{
  if(!provider&&!query_hard_boundary_provider_)return {};
  if(node_||visualization_only_)throw std::logic_error("query boundaries require a private collision reader");
  auto previous=std::move(query_hard_boundary_provider_);
  query_hard_boundary_provider_=std::move(provider);invalidateCollisionQueryCaches();return previous;
}

GridMap::QueryHardBoundaryDiagnosticProvider GridMap::exchangeQueryHardBoundaryDiagnosticProvider(
    QueryHardBoundaryDiagnosticProvider provider)
{
  if(!provider&&!query_hard_boundary_diagnostic_provider_)return {};
  if(node_||visualization_only_)throw std::logic_error("query boundary diagnostics require a private collision reader");
  auto previous=std::move(query_hard_boundary_diagnostic_provider_);
  query_hard_boundary_diagnostic_provider_=std::move(provider);return previous;
}

scan_planner::BodyEnvelope GridMap::queryBodyEnvelope(const Eigen::Vector3d& position,bool planning) const
{
  auto body=planning?planningBodyEnvelope():bodyEnvelope();
  if(body.model==scan_planner::CollisionModel::OrientedBox && stairClearanceConfigured() &&
      position.allFinite() && query_stair_clearance_provider_ && query_stair_clearance_provider_(position)) {
    body.length=mp_.box_nominal_length_+2.*mp_.stair_planar_clearance_;
    body.width=mp_.box_nominal_width_+2.*mp_.stair_planar_clearance_;
  }
  return body;
}

bool GridMap::stairSubvoxelQueryAllowed(const Eigen::Vector3d& position,bool strict) const
{
  return !strict && mp_.stair_subvoxel_collision_ &&
      mp_.collision_model_=="oriented_box" && position.allFinite() &&
      query_stair_clearance_provider_ && query_stair_clearance_provider_(position);
}

bool GridMap::occupiedSubvoxelIntersects(const Eigen::Vector3i& cell,int address,
    const scan_planner::OrientedBoxVoxel3DQuery& fine_query,
    scan_planner::SubvoxelCollisionCounts* diagnostic,
    const scan_planner::OrientedBoxVoxel3DQuery* rigid_query,bool refine_vertical) const
{
  const auto evidence=static_cast<std::size_t>(address)<md_.occupied_hit_subvoxels_.size()?
      md_.occupied_hit_subvoxels_[address]:std::uint16_t{0};
  if(!evidence || (evidence&0xff00U)) {
    if(diagnostic)++diagnostic->coarse_fallback;
    return true;
  }
  if(diagnostic)++diagnostic->refined_coarse;
  const bool has_bounds=static_cast<std::size_t>(address)<md_.occupied_hit_bounds_.size();
  if(refine_vertical&&has_bounds) {
    const auto bounds=md_.occupied_hit_bounds_[address].world(cell,mp_.resolution_);
    if(diagnostic)++diagnostic->tested_subcells;
    if(fine_query.allHitBoundsBelowBody(bounds.first,bounds.second)||
        !fine_query.intersectsVerticalBounds(cell,bounds.first.z(),bounds.second.z()))return false;
  }
  bool intersects=!refine_vertical;
  // The complete endpoint union remains 2x2x2, but this fix concerns vertical
  // stair quantization only. Do not reduce an obstacle's coarse XY footprint.
  for(int half=0;refine_vertical&&half<2;++half)if(evidence&(half?0xaaU:0x55U)) {
    if(diagnostic)++diagnostic->tested_subcells;
    if(fine_query.intersectsVerticalHalf(cell,half!=0)) {
      if(diagnostic)++diagnostic->intersecting_subcells;
      intersects=true;
      if(!diagnostic&&!rigid_query)return true;
    }
  }
  // Do not turn an occupied tread into FREE. Only explain a lower-envelope
  // contact, outside the rigid chassis SAT, by the ALL-hit bounds and
  // separately qualified original tread surface for this immutable version.
  // Unknown/coarse-only history returned above; a new/mixed hit cannot be
  // overwritten by subsequent ground observations.
  if(intersects&&rigid_query&&query_stair_contact_provider_&&has_bounds) {
    const auto bounds=md_.occupied_hit_bounds_[address].world(cell,mp_.resolution_);
    // Use the same conservative measured Z union as the full-envelope query.
    // A coarse voxel may cross the chassis bottom while every hit is below it;
    // testing the entire voxel here would reintroduce that quantization error.
    // For a tilted chassis, coarse XY can likewise cross its bottom while ALL
    // actual hits remain below that plane. Reuse the full body's separating
    // plane proof before the same SAT. Side faces keep coarse XY and contact
    // still needs independent qualified support; touching/mixed hits block.
    const bool rigid_intersects=refine_vertical?
        (!rigid_query->allHitBoundsBelowBody(bounds.first,bounds.second)&&
         rigid_query->intersectsVerticalBounds(cell,bounds.first.z(),bounds.second.z())):
        rigid_query->intersects(cell);
    if(!rigid_intersects&&query_stair_contact_provider_(bounds.first,bounds.second)) {
      if(diagnostic)++diagnostic->qualified_support_contacts;
      return false;
    }
  }
  return intersects;
}

void GridMap::markCoarseOnlyOccupancy(int address)
{
  if(static_cast<std::size_t>(address)>=md_.occupied_hit_subvoxels_.size())return;
  auto& evidence=md_.occupied_hit_subvoxels_[address];
  if(evidence&0xff00U)return;
  evidence|=scan_planner::kCoarseOnlyHitEvidence;
  invalidateCollisionQueryCaches();++occupancy_revision_;
}

void GridMap::recordOccupiedHit(const Eigen::Vector3d& point,const Eigen::Vector3i& cell,int address)
{
  if(static_cast<std::size_t>(address)>=md_.occupied_hit_subvoxels_.size())return;
  auto& evidence=md_.occupied_hit_subvoxels_[address];
  if(evidence&0xff00U)return;
  const auto bit=scan_planner::hitSubvoxelBit(point,cell,mp_.resolution_);
  // A hit cannot retroactively explain an existing coarse-only/weak history.
  // Keep union bits even while coarse odds are weak; clear only on actual
  // clamp-min free evidence, reset, or ring-cell reuse.
  if(!bit || !std::isfinite(md_.occupancy_buffer_[address]) ||
      (!evidence && md_.occupancy_buffer_[address]>mp_.clamp_min_log_)) {
    markCoarseOnlyOccupancy(address);return;
  }
  bool bounds_changed=false;
  if(static_cast<std::size_t>(address)<md_.occupied_hit_bounds_.size())
    bounds_changed=md_.occupied_hit_bounds_[address].include(point,cell,mp_.resolution_,evidence==0);
  if((evidence&bit)&&!bounds_changed)return;
  evidence|=bit;invalidateCollisionQueryCaches();++occupancy_revision_;
}

void GridMap::setOccupiedHit(const Eigen::Vector3d& original_endpoint)
{
  if(!original_endpoint.allFinite()||!isInMap(original_endpoint))return;
  Eigen::Vector3i cell;posToIndex(original_endpoint,cell);
  const int address=toAddress(cell);
  recordOccupiedHit(original_endpoint,cell,address);
  applyOccupancyUpdateAtIndex(cell,address,mp_.clamp_max_log_);
}

bool GridMap::setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,
    std::int64_t source_stamp_ns,const std::string& frame)
{return setVisualizationBodyEnvelopeImpl(body,{},0.,0.,source_stamp_ns,frame);}

bool GridMap::setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,double yaw,
    std::int64_t source_stamp_ns,const std::string& frame)
{return setVisualizationBodyEnvelopeImpl(body,yaw,0.,0.,source_stamp_ns,frame);}

bool GridMap::setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,double yaw,
    double pitch,double roll,std::int64_t source_stamp_ns,const std::string& frame)
{return setVisualizationBodyEnvelopeImpl(body,yaw,pitch,roll,source_stamp_ns,frame);}

bool GridMap::setVisualizationBodyEnvelopeImpl(const scan_planner::BodyEnvelope& body,
    std::optional<double> yaw,double pitch,double roll,
    std::int64_t source_stamp_ns,const std::string& frame)
{
  if(source_stamp_ns<=0 || frame.empty() || frame!=mp_.frame_id_ || (yaw&&!std::isfinite(*yaw)) ||
      !std::isfinite(pitch)||!std::isfinite(roll))return false;
  try {scan_planner::validateBodyEnvelope(body);} catch(const std::invalid_argument&) {return false;}
  std::shared_ptr<const VisualizationBodyEnvelope> value=
      std::make_shared<const VisualizationBodyEnvelope>(VisualizationBodyEnvelope{body,source_stamp_ns,frame,yaw,pitch,roll});
  auto previous=std::atomic_load_explicit(&visualization_body_envelope_,std::memory_order_acquire);
  do {
    if(previous&&previous->source_stamp_ns>source_stamp_ns)return false;
  } while(!std::atomic_compare_exchange_weak_explicit(&visualization_body_envelope_,&previous,value,
      std::memory_order_acq_rel,std::memory_order_acquire));
  return true;
}

int GridMap::getInflateOccupancy(Eigen::Vector3d position,double yaw,double pitch,double roll)
{
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if(!position.allFinite()||!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll))return -1;
  if(pitch==0.&&roll==0.)return getInflateOccupancyYawOnly(position,yaw);
  if(mp_.collision_model_!="oriented_box")return -1; // No invented tilted-cylinder approximation.
  return boxOccupancy(position,yaw,pitch,roll,false);
}

int GridMap::getPlanningInflateOccupancy(Eigen::Vector3d position,double yaw,double pitch,double roll)
{
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if(!position.allFinite()||!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll))return -1;
  if(pitch==0.&&roll==0.)return getPlanningInflateOccupancyYawOnly(position,yaw);
  if(planningRequiresObservedFree())
    return mp_.require_observed_free_?getInflateOccupancy(position,yaw,pitch,roll):2;
  if(mp_.collision_model_!="oriented_box")return -1;
  return boxOccupancy(position,yaw,pitch,roll,true);
}

int GridMap::boxOccupancy(const Eigen::Vector3d &position,double yaw,double pitch,double roll,bool planning)
{
  if(!position.allFinite()||!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll)||!isInMap(position))return -1;
  if(pitch==0.&&roll==0.)return boxOccupancy(position,yaw,planning);
  const bool strict=!planning&&mp_.require_observed_free_;
  if(strict)beginCollisionQuery();
  // First implementation intentionally has no pose cache: all XYZ/RPY affect
  // the intersected set, including asymmetric rotated body-Z extents.
  const auto body=queryBodyEnvelope(position,planning);
  if(query_hard_boundary_provider_&&query_hard_boundary_provider_(position,yaw,pitch,roll,body))return 1;
  int state=0;
  if(!strict) {
    // Occupied-only policy: finite nonoccupied raw cells cannot affect the
    // result. Test SAT only for occupied/invalid/outside candidates, avoiding
    // fifteen separating-axis evaluations on every empty voxel during A*.
    const scan_planner::OrientedBoxVoxel3DQuery query(position,yaw,pitch,roll,mp_.resolution_,body);
    std::optional<scan_planner::OrientedBoxVoxel3DQuery> fine_query;
    std::optional<scan_planner::OrientedBoxVoxel3DQuery> rigid_query;
    const bool refine_vertical=stairSubvoxelQueryAllowed(position,false);
    if(refine_vertical||(stairContactConfigured()&&query_stair_contact_provider_))
      fine_query.emplace(position,yaw,pitch,roll,mp_.resolution_,body);
    if(fine_query&&stairContactConfigured()&&query_stair_contact_provider_) {
      auto rigid=body;rigid.below=mp_.stair_contact_body_below_;
      rigid_query.emplace(position,yaw,pitch,roll,mp_.resolution_,rigid);
    }
    const auto xs=scan_planner::verticalVoxelSpan(query.center.x(),query.extent.x(),query.extent.x(),mp_.resolution_);
    const auto ys=scan_planner::verticalVoxelSpan(query.center.y(),query.extent.y(),query.extent.y(),mp_.resolution_);
    const auto zs=scan_planner::verticalVoxelSpan(query.center.z(),query.extent.z(),query.extent.z(),mp_.resolution_);
    const double cells=(double(xs.high)-xs.low+1.)*(double(ys.high)-ys.low+1.)*(double(zs.high)-zs.low+1.);
    if(!std::isfinite(cells)||cells>200000.)throw std::invalid_argument("spatial oriented box voxel query exceeds bounded budget");
    // The ring buffer is contiguous along Z. As in the existing upright
    // query, resolve its two horizontal modulo indices once per column,
    // not three integer divisions for each empty voxel. The visited AABB,
    // SAT contacts and outside/invalid/occupied priorities are unchanged.
    const int z_size=mp_.map_voxel_num_.z(),yz_stride=mp_.map_voxel_num_.y()*z_size;
    const auto &lo=mp_.map_bound_min_idx_,&hi=mp_.map_bound_max_idx_;
    const int initial_z=getLocalIndex(zs.low,2);
    for(std::int64_t x=xs.low;x<=xs.high&&state!=1;++x) {
      const bool inside_x=x>=lo.x()&&x<=hi.x();
      const int x_row=inside_x?getLocalIndex(static_cast<int>(x),0)*yz_stride:0;
      for(std::int64_t y=ys.low;y<=ys.high&&state!=1;++y) {
        const bool inside_xy=inside_x&&y>=lo.y()&&y<=hi.y();
        const int row=inside_xy?x_row+getLocalIndex(static_cast<int>(y),1)*z_size:0;
        int local_z=initial_z;
        for(std::int64_t z=zs.low;z<=zs.high;++z) {
          const Eigen::Vector3i cell(static_cast<int>(x),static_cast<int>(y),static_cast<int>(z));
          int next=-1;
          if(inside_xy&&z>=lo.z()&&z<=hi.z()) {
            const double odds=md_.occupancy_buffer_[row+local_z];
            if(++local_z==z_size)local_z=0;
            if(std::isfinite(odds)&&odds<=mp_.min_occupancy_log_)continue;
            next=std::isfinite(odds)?1:2;
          } else if(++local_z==z_size)local_z=0;
          if(query.intersects(cell)) {
            if(next==1 && fine_query && !occupiedSubvoxelIntersects(cell,toAddress(cell),*fine_query,
                nullptr,rigid_query?&*rigid_query:nullptr,refine_vertical))continue;
            state=scan_planner::mergeCollisionStatus(state,next);
          }
          if(state==1)break;
        }
      }
    }
  } else state=scan_planner::orientedBoxStatus(position,yaw,pitch,roll,mp_.resolution_,body,
      [&](const Eigen::Vector3i &cell) {
        if(!isInMap(cell))return -1;
        return rawCollisionStatus(cell);
      });
  if(strict&&state==2)++unknown_collision_queries_;
  if(strict&&state==0&&mp_.use_projected_rays_&&enforce_free_freshness_) {
    const auto now=collisionQueryClock();
    if(ray_clock_fault_||now<=0||now<collision_cache_clock_ns_||now>=collision_cache_deadline_ns_)return 2;
  }
  return state;
}

scan_planner::CollisionEvidence GridMap::inspectInflateOccupancy(const Eigen::Vector3d &position,double yaw,bool detailed)
{
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if(query_attitude_provider_ && position.allFinite() && std::isfinite(yaw)) {
    const auto attitude=query_attitude_provider_(position,yaw);
    if(!attitude) {
      scan_planner::CollisionEvidence evidence;evidence.attitude_resolved=false;return evidence;
    }
    return inspectInflateOccupancy(position,yaw,attitude->x(),attitude->y(),detailed);
  }
  return inspectInflateOccupancy(position,yaw,0.,0.,detailed);
}

scan_planner::CollisionEvidence GridMap::inspectInflateOccupancy(const Eigen::Vector3d &position,double yaw,double pitch,double roll,bool detailed)
{
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  // Rate-limited raw-evidence diagnostic, not the official collision decision.
  // Reverse the active inflation kernel; strict mode further intersects the
  // actual cylinder AABB. Counts include overlapping cells, and unknown raw
  // cells remain unknown even when the official inflated-buffer query is zero.
  scan_planner::CollisionEvidence evidence;
  beginCollisionQuery();
  std::map<std::array<int,3>,std::size_t> unique;
  if (!position.allFinite() || !std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(roll)) {evidence.counts[3]=1;return evidence;}
  evidence.modeled_hard_boundary_collision=query_hard_boundary_provider_&&
    query_hard_boundary_provider_(position,yaw,pitch,roll,queryBodyEnvelope(position,false));
  std::optional<scan_planner::OrientedBoxVoxel3DQuery> fine_query;
  std::optional<scan_planner::OrientedBoxVoxel3DQuery> rigid_query;
  const bool refine_vertical=stairSubvoxelQueryAllowed(position,mp_.require_observed_free_);
  if(refine_vertical||(!mp_.require_observed_free_&&stairContactConfigured()&&query_stair_contact_provider_)) {
    auto body=queryBodyEnvelope(position,false);
    fine_query.emplace(position,yaw,pitch,roll,mp_.resolution_,body);
    if(stairContactConfigured()&&query_stair_contact_provider_) {
      body.below=mp_.stair_contact_body_below_;
      rigid_query.emplace(position,yaw,pitch,roll,mp_.resolution_,body);
    }
    evidence.subvoxel_narrowphase=true;
  }
  const auto record=[&](const Eigen::Vector3i &cell,unsigned shape_mask) {
    int state=-1;
    double odds=std::numeric_limits<double>::quiet_NaN();
    if (isInMap(cell)) {
      odds=md_.occupancy_buffer_[toAddress(cell)];
      state=rawCollisionStatus(cell);
    }
    const int slot=state<0 ? 3:state;
    if (slot && !evidence.counts[slot]) evidence.first[slot-1]=cell;
    ++evidence.counts[slot];
    // Capture the result of the existing narrowphase call, not a second map
    // walk. Raw occupied counts intentionally remain unchanged.
    bool occupied_intersects=state==1;
    if(state==1&&fine_query)
      occupied_intersects=occupiedSubvoxelIntersects(cell,toAddress(cell),*fine_query,&evidence.subvoxels,
          rigid_query?&*rigid_query:nullptr,refine_vertical);
    if(detailed) {
      const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
      auto found=unique.find(key);
      if(found==unique.end()) {
        scan_planner::CollisionVoxelDiagnostic voxel;
        voxel.index=cell;voxel.log_odds=odds;voxel.native_state=state;
        voxel.occupied_intersects=occupied_intersects;
        voxel.occupied_narrowphase_applied=state==1&&fine_query.has_value();
        if(state==1&&static_cast<std::size_t>(toAddress(cell))<md_.occupied_hit_subvoxels_.size())
          voxel.occupied_hit_subvoxel_mask=md_.occupied_hit_subvoxels_[toAddress(cell)];
        voxel.raw_state=state<0?-1:scan_planner::strictRawVoxelStatus(
            odds,mp_.clamp_min_log_,mp_.min_occupancy_log_);
        if(state>=0 && static_cast<std::size_t>(toAddress(cell))<free_observation_stamps_.size())
          voxel.free_observation_stamp_ns=free_observation_stamps_[toAddress(cell)];
        voxel.classification=state<0 ? scan_planner::RawVoxelDiagnostic::Outside :
            scan_planner::diagnoseRawVoxel(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_,mp_.unknown_flag_);
        if (voxel.raw_state==0 && state==2) voxel.classification=scan_planner::RawVoxelDiagnostic::StaleFree;
        ++evidence.classification_counts[static_cast<std::size_t>(voxel.classification)];
        unique.emplace(key,evidence.voxels.size());evidence.voxels.push_back(voxel);
        ++evidence.unique_counts[slot];found=unique.find(key);
      }
      evidence.voxels[found->second].cylinder_mask|=shape_mask;
    }
  };
  if (mp_.collision_model_=="oriented_box") {
    scan_planner::forEachOrientedBoxVoxel(position,yaw,pitch,roll,mp_.resolution_,queryBodyEnvelope(position,false),
      [&](const Eigen::Vector3i &cell) {record(cell,4U);return true;});
    return evidence;
  }
  if(pitch!=0.||roll!=0.){evidence.counts[3]=1;return evidence;}
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
  for (int side:{-1,1}) {
    const Eigen::Vector3d center=position+side*mp_.double_cylinder_offset_*heading;
    const Eigen::Vector3i index=(center*mp_.resolution_inv_).array().floor().cast<int>();
    scan_planner::VerticalVoxelSpan span{0,0};
    if (mp_.require_observed_free_) {
      const auto body=bodyEnvelope();
      span=scan_planner::verticalVoxelSpan(center.z(),body.below,body.above,mp_.resolution_);
    }
    for (const auto &offset:md_.inflate_offsets_) {
      const Eigen::Vector3i cell=index-offset;
      if (mp_.require_observed_free_ && (cell.z()<span.low || cell.z()>span.high ||
          !scan_planner::cylinderIntersectsVoxelXY(center,cell,mp_.resolution_,mp_.double_cylinder_radius_))) continue;
      record(cell,side<0 ? 1U:2U);
    }
  }
  return evidence;
}

std::string GridMap::describeInflateOccupancy(const Eigen::Vector3d &position,double yaw)
{
  if (!position.allFinite() || !std::isfinite(yaw)) return "nonfinite_query";
  double pitch=0.,roll=0.;
  if(query_attitude_provider_) {
    const auto attitude=query_attitude_provider_(position,yaw);
    if(!attitude)return std::string("query_policy=")+collisionQueryPolicy()+" collision=2 attitude=unavailable evidence_counts=not_queried";
    pitch=attitude->x();roll=attitude->y();
    if(!std::isfinite(pitch)||!std::isfinite(roll))return "nonfinite_query_attitude";
  }
  return describeInflateOccupancy(position,yaw,pitch,roll);
}

std::string GridMap::describeInflateOccupancy(const Eigen::Vector3d &position,double yaw,double pitch,double roll)
{
  if(!position.allFinite()||!std::isfinite(yaw))return "nonfinite_query";
  if(!std::isfinite(pitch)||!std::isfinite(roll))return "nonfinite_query_attitude";
  // Resolve once: the query, witness counts and printed geometry must describe
  // the same immutable reader contract, never a yaw-only proxy of a tilted body.
  const auto evidence=inspectInflateOccupancy(position,yaw,pitch,roll,true);
  const auto &counts=evidence.counts;
  // The official occupied-only policy does not maintain the strict free-age
  // cache clock. Diagnostics must read the actual reader clock, not report
  // (zero - epoch stamp) as a huge negative age. Do not update evidence/cache
  // deadlines or manufacture a timestamp when this reader has no clock.
  const auto diagnostic_now=collisionQueryClock();
  const auto age_ms=[diagnostic_now](std::int64_t source) {
    return diagnostic_now>0 && source>0 && diagnostic_now>=source ?
        static_cast<double>(diagnostic_now-source)*1e-6 :
        std::numeric_limits<double>::quiet_NaN();
  };
  std::ostringstream out;out.precision(9);
  out<<"query_policy="<<collisionQueryPolicy();
  if (!mp_.require_observed_free_) out<<" collision="<<getInflateOccupancy(position,yaw,pitch,roll);
  out<<" evidence_counts="<<(mp_.require_observed_free_?"strict_with_free_expiry":"raw_evidence")
     <<" free="<<counts[0]<<" occupied="<<counts[1]<<" unknown="<<counts[2]<<" outside="<<counts[3]
     <<" raw_overlap=["<<counts[0]<<","<<counts[1]<<","<<counts[2]<<","<<counts[3]<<"]"
     <<" unique_voxels=["<<evidence.unique_counts[0]<<","<<evidence.unique_counts[1]<<","<<evidence.unique_counts[2]<<","<<evidence.unique_counts[3]<<"]";
  out<<" hit_subvoxel_narrowphase="<<evidence.subvoxel_narrowphase
     <<" hit_subvoxel_geometry="<<(stairContactConfigured()?"coarse_xy_all_hit_vertical_bounds":"coarse_xy_vertical_halves")
     <<" refined_coarse_witnesses="<<evidence.subvoxels.refined_coarse
     <<" hit_subcells_tested="<<evidence.subvoxels.tested_subcells
     <<" hit_subcells_intersecting="<<evidence.subvoxels.intersecting_subcells
     <<" coarse_witness_fallback="<<evidence.subvoxels.coarse_fallback;
  out<<" qualified_stair_contacts="<<evidence.subvoxels.qualified_support_contacts;
  out<<" modeled_hard_boundary_collision="<<evidence.modeled_hard_boundary_collision;
  const auto body=queryBodyEnvelope(position,false);
  if(evidence.modeled_hard_boundary_collision&&query_hard_boundary_diagnostic_provider_)
    out<<" model_hard_witness="<<query_hard_boundary_diagnostic_provider_(position,yaw,pitch,roll,body);
  double z_min=position.z()-body.below,z_max=position.z()+body.above;
  if (body.model==scan_planner::CollisionModel::OrientedBox) {
    out<<" body_xy_box=["<<body.length<<","<<body.width<<"] yaw="<<yaw<<" pitch="<<pitch<<" roll="<<roll;
    if(pitch!=0.||roll!=0.) {
      const scan_planner::OrientedBoxVoxel3DQuery box(position,yaw,pitch,roll,mp_.resolution_,body);
      z_min=box.center.z()-box.extent.z();z_max=box.center.z()+box.extent.z();
      out<<" body_geometry=spatial_obb body_center=["<<box.center.x()<<","<<box.center.y()<<","<<box.center.z()<<"]";
    }
  }
  out<<" body_z=["<<z_min<<","<<z_max<<"]"
     <<" never_observed="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::NeverObserved)]
     <<" observed_insufficient="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::Insufficient)]
     <<" stale_free="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::StaleFree)]
     <<" query_xyz=["<<position.x()<<","<<position.y()<<","<<position.z()<<"]"
     <<" map_revision="<<occupancyRevision()<<" frame="<<mp_.frame_id_
     <<" source_query_ns="<<diagnostic_now
     <<" front_source_ns="<<integratedRaySourceStamp(0)<<" rear_source_ns="<<integratedRaySourceStamp(1)
     <<" front_age_ms="<<age_ms(integratedRaySourceStamp(0))
     <<" rear_age_ms="<<age_ms(integratedRaySourceStamp(1));
  if(query_support_height_provider_) {
    const auto support=query_support_height_provider_(position);
    // This is the supported BODY reference Z, not the ground height.
    if(support && std::isfinite(*support))out<<" support_body_z="<<*support;
    else out<<" support_body_z=unavailable";
  }
  constexpr std::size_t max_witnesses=64;
  std::size_t reported=0;
  out<<" occupied_witnesses=[";
  for(const auto& voxel:evidence.voxels) {
    if(voxel.classification!=scan_planner::RawVoxelDiagnostic::Occupied)continue;
    if(reported==max_witnesses)break;
    if(reported++)out<<",";
    const Eigen::Vector3d point=(voxel.index.cast<double>().array()+.5)*mp_.resolution_;
    out<<"["<<point.x()<<","<<point.y()<<","<<point.z()<<"]";
  }
  out<<"] occupied_witnesses_omitted="<<(evidence.unique_counts[1]-reported);
  // Keep the historical coarse centre list, and separately expose which
  // source-backed cells actually touch this exact body. At most 64 detailed
  // records are formatted; finding the first true hit reuses the evidence
  // already gathered above, with no collision query or map mutation.
  const scan_planner::CollisionVoxelDiagnostic* first_effective=nullptr;
  std::array<const scan_planner::CollisionVoxelDiagnostic*,4> bounded_hits{};
  std::size_t effective_hits=0;
  reported=0;out<<" occupied_witness_details=[";
  for(const auto& voxel:evidence.voxels) {
    if(voxel.classification!=scan_planner::RawVoxelDiagnostic::Occupied)continue;
    if(!first_effective&&voxel.occupied_intersects)first_effective=&voxel;
    if(voxel.occupied_intersects) {
      if(effective_hits<bounded_hits.size())bounded_hits[effective_hits]=&voxel;
      ++effective_hits;
    }
    if(reported==max_witnesses)continue;
    if(reported++)out<<",";
    const Eigen::Vector3d point=(voxel.index.cast<double>().array()+.5)*mp_.resolution_;
    out<<"{\"cell\":["<<point.x()<<","<<point.y()<<","<<point.z()<<"],\"hit_mask\":"
       <<voxel.occupied_hit_subvoxel_mask<<",\"narrowphase\":"
       <<(voxel.occupied_narrowphase_applied?"true":"false")<<",\"refined_intersects\":"
       <<(voxel.occupied_intersects?"true":"false")<<"}";
  }
  out<<"] occupied_witness_details_omitted="<<(evidence.unique_counts[1]-reported);
  // The centres/mask above cannot distinguish voxel quantisation from actual
  // return height. Serialize only the already retained ALL-hit endpoint union
  // for at most four real collision witnesses. Coarse/untracked history has
  // no such evidence; never print its default bytes as measured coordinates.
  // No additional geometry/support query and no change to the verdict.
  out<<" collision_hit_bounds=[";
  const auto bounded_count=std::min(effective_hits,bounded_hits.size());
  for(std::size_t i=0;i<bounded_count;++i) {
    if(i)out<<",";
    const auto& voxel=*bounded_hits[i];const auto address=toAddress(voxel.index);
    const auto mask=voxel.occupied_hit_subvoxel_mask;
    const bool available=mask&&!(mask&0xff00U)&&address>=0&&
        static_cast<std::size_t>(address)<md_.occupied_hit_bounds_.size();
    const Eigen::Vector3d center=(voxel.index.cast<double>().array()+.5)*mp_.resolution_;
    out<<"{\"cell\":["<<center.x()<<","<<center.y()<<","<<center.z()<<"],\"available\":"
       <<(available?"true":"false");
    if(available) {
      const auto bounds=md_.occupied_hit_bounds_[address].world(voxel.index,mp_.resolution_);
      out<<",\"lo\":["<<bounds.first.x()<<","<<bounds.first.y()<<","<<bounds.first.z()
         <<"],\"hi\":["<<bounds.second.x()<<","<<bounds.second.y()<<","<<bounds.second.z()<<"]";
    }
    out<<"}";
  }
  out<<"] collision_hit_bounds_omitted="<<(effective_hits-bounded_count);
  if(bounded_count && blocked_voxel_mailbox_) {
    scan_planner::BlockedVoxelWatchRequest request;
    request.context_generation=blocked_voxel_context_;
    request.count=static_cast<std::uint8_t>(bounded_count);
    for(std::size_t i=0;i<bounded_count;++i) {
      const auto& key=bounded_hits[i]->index;
      request.cells[i]={{key.x(),key.y(),key.z()}};
    }
    // No map mutation, extra query, wait, or influence on this verdict.
    blocked_voxel_mailbox_->tryOffer(request);
  }
  const auto write_ray_witness=[&](const Eigen::Vector3i& cell,const char* label) {
    if(!near_field_diagnostics_.enabled())out<<label<<"disabled";
    else if(!near_field_diagnostics_.contains(cell))out<<label<<"outside_diagnostic_roi";
    else if(const auto* witnesses=near_field_diagnostics_.find(cell)) {
      for(const auto& witness:witnesses->last)if(witness) {
        const auto& w=*witness;const auto& m=w.metadata;
        out<<label<<"{sensor:"<<m.sensor_id<<",hit:"<<w.hit<<",vote:"<<w.contributed_vote
           <<",scan_ns:"<<m.scan_stamp_ns<<",acquisition_end_ns:"<<m.acquisition_end_ns
           <<",source_age_ms:"<<age_ms(m.acquisition_end_ns)
           <<",alignment_ns:"<<m.alignment_stamp_ns<<",received_ns:"<<m.received_ns
           <<",source_index:"<<m.source_index<<",offset_ns:"<<m.offset_time_ns
           <<",source_fields_available:"<<m.source_fields_available
           <<",origin:["<<w.origin.x()<<","<<w.origin.y()<<","<<w.origin.z()<<"]"
           <<",endpoint:["<<w.endpoint.x()<<","<<w.endpoint.y()<<","<<w.endpoint.z()<<"]}";
      }
    } else out<<label<<"not_retained";
  };
  if(first_effective) {
    const Eigen::Vector3d point=(first_effective->index.cast<double>().array()+.5)*mp_.resolution_;
    out<<" collision_occupied_cell=("<<point.x()<<","<<point.y()<<","<<point.z()<<")"
       <<" collision_occupied_hit_mask="<<first_effective->occupied_hit_subvoxel_mask;
    write_ray_witness(first_effective->index," collision_ray_witness=");
  } else out<<" collision_occupied_cell=none";
  std::int64_t oldest_free=std::numeric_limits<std::int64_t>::max();
  for(const auto& voxel:evidence.voxels)if(voxel.free_observation_stamp_ns>0)
    oldest_free=std::min(oldest_free,voxel.free_observation_stamp_ns);
  if(oldest_free!=std::numeric_limits<std::int64_t>::max())
    out<<" oldest_free_source_ns="<<oldest_free<<" oldest_free_age_ms="<<age_ms(oldest_free);
  const char *labels[]={" occupied_cell="," unknown_cell="," outside_cell="};
  for (int i=0;i<3;++i) if(counts[i+1]) {
    const Eigen::Vector3d point=(evidence.first[i].cast<double>().array()+.5)*mp_.resolution_;
    out<<labels[i]<<"("<<point.x()<<","<<point.y()<<","<<point.z()<<")";
    if(i<2)write_ray_witness(evidence.first[i]," ray_witness=");
  }
  return out.str();
}

void GridMap::resetAllMapData()
{
  blocked_voxel_watch_.setContext(++blocked_voxel_context_);
  blocked_voxel_watch_snapshot_=blocked_voxel_watch_.snapshot();
  // Invalidate an in-flight display without waiting for its rendering or ROS
  // publication. A frame already handed to middleware cannot be recalled.
  visualization_generation_.fetch_add(1,std::memory_order_release);
  std::atomic_store_explicit(&visualization_body_envelope_,
      std::shared_ptr<const VisualizationBodyEnvelope>{},std::memory_order_release);
  near_field_diagnostics_.clear();
  invalidateCollisionQueryCaches();
  collision_cache_deadline_ns_=collision_cache_clock_ns_=0;
  if(mp_.use_projected_rays_) free_observation_stamps_.assign(md_.occupancy_buffer_.size(),0);
  else free_observation_stamps_.clear();
  ++occupancy_revision_;
  // A reset map has no current measurement support until raycastProcess()
  // actually integrates the replacement cloud; an old fresh stamp is not
  // evidence for the newly empty buffer.
  integrated_cloud_stamp_ns_ = 0;
  ray_integrated_stamps_.fill(0);
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  if(mp_.stair_subvoxel_collision_)
    md_.occupied_hit_subvoxels_.assign(md_.occupancy_buffer_.size(),0);
  else std::vector<std::uint16_t>().swap(md_.occupied_hit_subvoxels_);
  if(stairContactConfigured())md_.occupied_hit_bounds_.assign(md_.occupancy_buffer_.size(),{});
  else std::vector<scan_planner::ObservedHitBounds>().swap(md_.occupied_hit_bounds_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.planning_occupancy_buffer_inflate_.begin(), md_.planning_occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.planning_occupancy_buffer_inflate_cnt_.begin(), md_.planning_occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

bool GridMap::applyLocalizationContext(const std::string &payload)
{
  // Acknowledgement is sent only after clearing actual evidence. The bridge
  // waits for it before publishing any pair in the replacement coordinates.
  if (payload.empty() || payload.size() > 4096) return false;
  try {
    const auto value = nlohmann::json::parse(payload);
    if (value.at("schema") != 1 ||
        value.at("session_id").get<std::string>() != mp_.localization_session_id_ ||
        !value.at("epoch").is_number_unsigned() || !value.at("sequence").is_number_unsigned() ||
        !value.at("barrier_ns").is_number_unsigned()) return false;
    const auto epoch = value.at("epoch").get<std::uint64_t>();
    const auto sequence = value.at("sequence").get<std::uint64_t>();
    const auto barrier = value.at("barrier_ns").get<std::uint64_t>();
    const auto seed = value.at("seed_id").get<std::string>();
    if (!epoch || !sequence || seed.empty() || seed.size() > 256 || !barrier ||
        barrier > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
        epoch < localization_epoch_ || sequence < localization_context_sequence_) return false;
    if (sequence == localization_context_sequence_)
      return payload == localization_context_payload_; // Reliable retry, never reset twice.
    invalidateCloudPosePairs(static_cast<std::int64_t>(barrier));
    // A new map-context sequence is an explicit evidence rebuild, separate
    // from goal/reference generation. ACK must never preserve the old grid.
    resetAllMapData();
    visualization_body_stamp_ns_=0;
    visualization_body_yaw_=std::numeric_limits<double>::quiet_NaN();
    visualization_obb_kernel_ready_=false;
    visualization_revision_[1]=std::numeric_limits<std::uint64_t>::max();
    ray_clock_fault_=false;ray_query_clock_ns_=ray_tick_clock_ns_=ray_tick_effective_ns_=0;
    localization_epoch_ = epoch;
    localization_seed_ = seed;
    localization_context_sequence_ = sequence;
    localization_context_barrier_ns_ = barrier;
    ray_integrations_.fill(0);ray_drops_.fill(0);ray_unattributed_drops_=0;
    localization_context_payload_ = payload;
    publishProjectedIngressContext();
    // Publish this already-reset, source-invalid context before its ACK. If
    // the new source never sends another ray, readers must still learn that
    // old-coordinate snapshots no longer describe the live map. A reliable
    // identical retry returned above and cannot renew or republish evidence.
    if(collision_update_callback_)collision_update_callback_();
    return true;
  } catch (const nlohmann::json::exception &) {
    return false;
  }
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

std::vector<InflationColumn> GridMap::inflationColumns(const vector<Eigen::Vector3i>& offsets)
{
  std::vector<InflationColumn> columns;
  for (const auto& offset:offsets) {
    if (!columns.empty() && columns.back().x==offset.x() &&
        columns.back().y==offset.y() &&
        static_cast<std::int64_t>(columns.back().end_z)+1==offset.z()) {
      columns.back().end_z=offset.z();
    } else {
      columns.push_back({offset.x(),offset.y(),offset.z(),offset.z()});
    }
  }
  return columns;
}

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const std::vector<InflationColumn>& columns,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  // The existing kernels are ordered in contiguous vertical columns. Clip
  // each column once and update its contiguous ring-buffer runs, rather than
  // repeating XYZ bounds/address arithmetic for every voxel. Do not replace
  // or approximate the kernel: holes, ordering, per-update zero clamp and
  // sliding's ignore mask retain their original meaning.
  const scan_planner::ObservedRayMapIndex index(mp_.map_bound_min_idx_,mp_.map_voxel_num_);
  const auto &minimum=mp_.map_bound_min_idx_,&maximum=mp_.map_bound_max_idx_;
  const int z_size=mp_.map_voxel_num_.z();
  int minimum_local_z=minimum.z()%z_size;
  if(minimum_local_z<0)minimum_local_z+=z_size;
  for(const auto& column:columns) {
    const int x=id.x()+column.x,y=id.y()+column.y;
    const int begin_z=std::max(id.z()+column.begin_z,minimum.z());
    const int end_z=std::min(id.z()+column.end_z,maximum.z());
    if(x<minimum.x()||x>maximum.x()||y<minimum.y()||y>maximum.y()||begin_z>end_z)continue;
    int address=index.address({x,y,begin_z});
    int local_z=begin_z-minimum.z()+minimum_local_z;
    if(local_z>=z_size)local_z-=z_size;
    int remaining=end_z-begin_z+1;
    while(remaining>0) {
      const int count=std::min(remaining,z_size-local_z);
      for(int at=address;at<address+count;++at) {
        if(ignore_mask&&(*ignore_mask)[at])continue;
        cnt_buffer[at]=std::max(0,cnt_buffer[at]+delta);
        flag_buffer[at]=cnt_buffer[at]>0?1:0;
      }
      remaining-=count;
      address+=count-z_size;local_z=0;
    }
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_columns_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
  if (!md_.planning_occupancy_buffer_inflate_cnt_.empty())
    updateInflationLayer(id,delta,md_.planning_inflate_columns_,md_.planning_occupancy_buffer_inflate_cnt_,
                         md_.planning_occupancy_buffer_inflate_,ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  if(!std::isfinite(new_log_odds)||new_log_odds>mp_.clamp_min_log_)
    markCoarseOnlyOccupancy(addr);
  applyOccupancyUpdateAtIndex(id,addr,new_log_odds);
}

void GridMap::applyOccupancyUpdateAtIndex(const Eigen::Vector3i& id,int addr,double new_log_odds)
{
  // Integration's cache queue already owns a bounds-checked cell and its ring
  // address in the current window. No sliding/reset happens while draining it.
  // Source stamps, hit/miss counters and witnesses have already been updated
  // by the complete-ray walk, outside this log-odds-only operation.
  const double old_log_odds=md_.occupancy_buffer_[addr];
  if(new_log_odds==mp_.clamp_min_log_ &&
      static_cast<std::size_t>(addr)<md_.occupied_hit_subvoxels_.size() &&
      md_.occupied_hit_subvoxels_[addr]) {
    md_.occupied_hit_subvoxels_[addr]=0;invalidateCollisionQueryCaches();++occupancy_revision_;
  }
  if(old_log_odds==new_log_odds &&
      (new_log_odds!=0. || std::signbit(old_log_odds)==std::signbit(new_log_odds))) return;
  const bool was_occ = old_log_odds > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;
  const bool was_known=old_log_odds>=mp_.clamp_min_log_;
  const bool now_known=new_log_odds>=mp_.clamp_min_log_;
  const bool free_changed=scan_planner::strictRawVoxelStatus(old_log_odds,
      mp_.clamp_min_log_,mp_.min_occupancy_log_) != scan_planner::strictRawVoxelStatus(
      new_log_odds,mp_.clamp_min_log_,mp_.min_occupancy_log_);
  if (was_known!=now_known || was_occ!=now_occ || (mp_.require_observed_free_ && free_changed)) {
    invalidateCollisionQueryCaches();
    ++occupancy_revision_; // Also wake strict planners when unknown becomes observed.
  }

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
  {
    updateInflation(id, now_occ ? 1 : -1);
  }
}

void GridMap::resetCellByAddress(int addr)
{
  invalidateCollisionQueryCaches();
  if(static_cast<std::size_t>(addr)<md_.occupied_hit_subvoxels_.size())md_.occupied_hit_subvoxels_[addr]=0;
  if(static_cast<std::size_t>(addr)<free_observation_stamps_.size()) free_observation_stamps_[addr]=0;
  if (md_.occupancy_buffer_[addr]>=mp_.clamp_min_log_) ++occupancy_revision_;
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  near_field_diagnostics_.erase(id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
  {
    ++occupancy_revision_;
    updateInflation(id_g, -1);
  }

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  invalidateCollisionQueryCaches();
  if(static_cast<std::size_t>(addr)<md_.occupied_hit_subvoxels_.size())md_.occupied_hit_subvoxels_[addr]=0;
  if(static_cast<std::size_t>(addr)<free_observation_stamps_.size()) free_observation_stamps_[addr]=0;
  if (md_.occupancy_buffer_[addr]>=mp_.clamp_min_log_) ++occupancy_revision_;
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  near_field_diagnostics_.erase(id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
  {
    ++occupancy_revision_;
    updateInflation(id_g, -1, &clear_mask);
  }
}

void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    if (!md_.planning_occupancy_buffer_inflate_.empty()) {
      md_.planning_occupancy_buffer_inflate_cnt_[addr] = 0;
      md_.planning_occupancy_buffer_inflate_[addr] = 0;
    }
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);
  if(occ==1)recordOccupiedHit(pos,id,idx_ctns);
  cacheOccupancyAtIndex(id,idx_ctns,occ);

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

bool GridMap::prepareParallelRayVisits(const std::array<std::int64_t,2>& sources)
{
  // Only the unchanged official projected-ray visitor is accelerated. Strict
  // evidence/diagnostics retain the original path, including failure prefixes.
  if(!parallel_ray_visits_enabled_||collision_snapshot_||mp_.require_observed_free_||
     !mp_.use_projected_rays_||near_field_diagnostics_.enabled()||
     !std::isfinite(mp_.max_ray_length_)||mp_.max_ray_length_<=0.||
     md_.proj_points_cnt<4096||static_cast<std::size_t>(md_.proj_points_cnt)>
       scan_planner::ParallelObservedRayVisits::kMaxRays)return false;
  try {
    if(!parallel_ray_visits_)
      parallel_ray_visits_=std::make_unique<scan_planner::ParallelObservedRayVisits>();
    auto& prepared=parallel_ray_visits_->rays();prepared.clear();prepared.reserve(md_.proj_points_cnt);
    for(int i=0;i<md_.proj_points_cnt;++i) {
      const auto& origin=md_.proj_origins_[i];auto end=md_.proj_points_[i];
      if(!isInMap(origin))return false;
      // Clipping changes endpoint inclusion, independently of whether the
      // original measured endpoint is a real hit or a censored self return.
      bool original_endpoint=true;
      if(!scan_planner::clipObservedRay(origin,mp_.local_update_range_,end,original_endpoint))return false;
      if(!isInMap(end)) {original_endpoint=false;end=closetPointInMap(end,origin);}
      const double length=(end-origin).norm();
      if(length>mp_.max_ray_length_) {
        original_endpoint=false;end=(end-origin)/length*mp_.max_ray_length_+origin;
      }
      bool include_end=!original_endpoint;
      if(static_cast<std::size_t>(i)<projected_ray_kinds_.size() && projected_ray_kinds_[i]==1 &&
          ((end/mp_.resolution_).array().floor()==
           (md_.proj_points_[i]/mp_.resolution_).array().floor()).all())
        include_end=false; // A sub-voxel clip must not clear the censored hit cell.
      prepared.push_back({origin,end,static_cast<unsigned char>(
          projected_ray_stamps_[i]==sources[0]?1:2),include_end});
    }
    // This preflight is conservative: otherwise the original traversal must
    // enforce the exact global 16M budget and partially integrated prefix.
    return parallel_ray_visits_->run(mp_.map_bound_min_idx_,mp_.map_voxel_num_,
                                    mp_.resolution_,16000000);
  } catch(...) {
    // Recoverable allocation/thread-creation/preflight failures precede job
    // publication. Submitted work is retired before a normal return; a broken
    // synchronization primitive after dispatch fails closed inside run().
    return false;
  }
}

bool GridMap::raycastProcess()
{
  const auto ray_started=std::chrono::steady_clock::now();
  map_work_timing_={};
  last_parallel_ray_visits_=false;
  const bool complete_rays=mp_.use_projected_rays_ || mp_.require_observed_free_;
  free_evidence_recovered_=false;
  invalidateCollisionQueryCaches();
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return false;

  updateSlidingMap(md_.ray_pos_);
  const scan_planner::ObservedRayMapIndex ray_index(mp_.map_bound_min_idx_,mp_.map_voxel_num_);
  if(near_field_diagnostics_.enabled()) near_field_diagnostics_.beginIntegration();
  // A fused production batch has at most one original acquisition stamp per
  // physical source. Keep two per-source visit bits in the existing scratch
  // byte instead of rereading a 64-bit evidence stamp on every repeated DDA
  // visit. The traversal, endpoint votes and first visit from EACH source are
  // unchanged. Arbitrary per-point source streams and diagnostics use the
  // original general visitor below.
  std::array<std::int64_t,2> visit_sources{{0,0}};
  bool source_visit_bits=mp_.use_projected_rays_&&!near_field_diagnostics_.enabled()&&
      projected_ray_stamps_.size()==static_cast<std::size_t>(md_.proj_points_cnt);
  if(source_visit_bits)for(const auto source:projected_ray_stamps_) {
    if(source<=0){source_visit_bits=false;break;}
    if(!visit_sources[0])visit_sources[0]=source;
    else if(source!=visit_sources[0]) {
      if(!visit_sources[1])visit_sources[1]=source;
      else if(source!=visit_sources[1]){source_visit_bits=false;break;}
    }
  }

  md_.raycast_num_ += 1;
  if (complete_rays) {
    // Complete per-sensor rays must not inherit the legacy wrapping-char
    // shortcuts when the collision policy does not require observed free.
    std::fill(md_.flag_traverse_.begin(),md_.flag_traverse_.end(),-1);
    md_.raycast_num_=0;
    if (mp_.use_projected_rays_)
      std::fill(md_.flag_rayend_.begin(),md_.flag_rayend_.end(),-1);
  }

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;
  std::size_t observed_ray_budget=16000000;
  bool complete=true;
  const auto traversal_started=std::chrono::steady_clock::now();
  last_parallel_ray_visits_=source_visit_bits&&prepareParallelRayVisits(visit_sources);
  if(last_parallel_ray_visits_)observed_ray_budget-=parallel_ray_visits_->steps();

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    // A successful speculative pass already evaluated the identical clipping
    // arithmetic. Reuse that immutable endpoint; do not normalize/floor the
    // same 120k real rays a second time during serial evidence commit.
    pt_w = last_parallel_ray_visits_?parallel_ray_visits_->rays()[i].end:md_.proj_points_[i];
    const bool has_ray_stamp=mp_.use_projected_rays_ &&
        static_cast<std::size_t>(i)<projected_ray_stamps_.size();
    const auto ray_source_stamp=has_ray_stamp ? projected_ray_stamps_[i] : std::int64_t{0};
    const unsigned char source_visit_mask=source_visit_bits?
        static_cast<unsigned char>(ray_source_stamp==visit_sources[0]?1:2):0;
    const bool record_ray_diagnostics=near_field_diagnostics_.enabled() &&
        static_cast<std::size_t>(i)<projected_diagnostics_.size();
    const Eigen::Vector3d &origin=mp_.use_projected_rays_ ? md_.proj_origins_[i] : md_.ray_pos_;
    if (!last_parallel_ray_visits_ && mp_.use_projected_rays_ && !isInMap(origin)) { complete=false;break; }
    const bool censored_endpoint=mp_.use_projected_rays_ &&
        static_cast<std::size_t>(i)<projected_ray_kinds_.size() && projected_ray_kinds_[i]==1;
    // A censored self return is not a new free-space endpoint. Preserve only
    // the observed prefix and exclude its original endpoint voxel, exactly as
    // for a real hit. A nearer range/map clip is free and may include its end.
    bool original_endpoint=last_parallel_ray_visits_?!parallel_ray_visits_->rays()[i].include_end:true;
    if (!last_parallel_ray_visits_ && !scan_planner::clipObservedRay(origin,mp_.local_update_range_,pt_w,original_endpoint))
      continue;

    // set flag for projected point

    if (!last_parallel_ray_visits_ && !isInMap(pt_w))
    {
      original_endpoint=false;
      pt_w = closetPointInMap(pt_w, origin);

      length = (pt_w - origin).norm();
      if (length > mp_.max_ray_length_)
      {
        original_endpoint=false;
        pt_w = (pt_w - origin) / length * mp_.max_ray_length_ + origin;
      }
      vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = last_parallel_ray_visits_?0.:(pt_w - origin).norm();

      if (length > mp_.max_ray_length_)
      {
        original_endpoint=false;
        pt_w = (pt_w - origin) / length * mp_.max_ray_length_ + origin;
        vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, 0);
      }
      else
      {
        if (mp_.use_projected_rays_ && original_endpoint && !censored_endpoint) {
          Eigen::Vector3i cell;posToIndex(pt_w,cell);vox_idx=ray_index.address(cell);
          // Every endpoint contributes geometry, even when this cell already
          // received its one probabilistic vote in this fused batch.
          recordOccupiedHit(pt_w,cell,vox_idx);
          if(record_ray_diagnostics) {
            scan_planner::RayWitness witness;
            witness.metadata=projected_diagnostics_[i];witness.origin=origin;
            witness.endpoint=md_.proj_points_[i];witness.integrated_endpoint=pt_w;witness.hit=true;
            witness.contributed_vote=md_.flag_rayend_[vox_idx]!=md_.raycast_num_;
            near_field_diagnostics_.record(cell,witness);
          }
          // One real-hit vote per cell avoids int16 overflow for dense scans.
          if (md_.flag_rayend_[vox_idx]!=md_.raycast_num_) {
            md_.flag_rayend_[vox_idx]=md_.raycast_num_;
            cacheOccupancyAtIndex(cell,vox_idx,1);
          }
        } else vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, original_endpoint ? 1 : 0);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));
    if (mp_.use_projected_rays_) {
      min_x=min(min_x,origin.x());min_y=min(min_y,origin.y());min_z=min(min_z,origin.z());
      max_x=max(max_x,origin.x());max_y=max(max_y,origin.y());max_z=max(max_z,origin.z());
    }

    if (complete_rays)
    {
      bool include_endpoint=!original_endpoint;
      if(censored_endpoint &&
          ((pt_w/mp_.resolution_).array().floor()==
           (md_.proj_points_[i]/mp_.resolution_).array().floor()).all())
        include_endpoint=false; // Original self-return voxel is never a miss.
      // Sharing an endpoint/crossed voxel does not mean the remaining rays
      // coincide. Complete real rays, but give each free cell only one vote.
      // DDA cells are component-wise between its own floor-indexed endpoints.
      // A fixed axis-aligned window is convex: when both are in bounds every
      // visited cell is in bounds too. Keep the original per-cell path when
      // diagnostics are enabled, and for any clipped/boundary ray that cannot
      // establish this exact predicate. Use division like DDA, not a rounded
      // inverse-resolution multiplication near a voxel face.
      const bool fast_visitor=!near_field_diagnostics_.enabled();
      bool ray_inside_window=last_parallel_ray_visits_;
      if(!ray_inside_window) {
        const Eigen::Vector3i start_cell=(origin/mp_.resolution_).array().floor().cast<int>();
        const Eigen::Vector3i end_cell=(pt_w/mp_.resolution_).array().floor().cast<int>();
        ray_inside_window=fast_visitor&&isInMap(start_cell)&&isInMap(end_cell);
      }
      // GCC -O3 otherwise outlines this closure even in the by-source hot
      // path (millions of calls per fusion). Keep exactly the same visitor
      // in the DDA loop; this is a GCC/Clang code-generation hint, not a
      // shortcut of ray visits, occupancy votes or original source clocks.
      const auto visitor=
          [this,i,&origin,&pt_w,has_ray_stamp,ray_source_stamp,record_ray_diagnostics,
           fast_visitor,ray_inside_window,source_visit_mask](const Eigen::Vector3i &cell,int address,auto source_mode)
#if defined(__GNUC__) || defined(__clang__)
              __attribute__((always_inline))
#endif
              {
            constexpr bool by_source=decltype(source_mode)::value;
            if (!ray_inside_window&&!isInMap(cell)) return;
            // -1 is still untouched scratch state. Encoded 0/1/2 mean first
            // source / second source / both; this scratch byte is NOT the
            // observed/free classification or a source-lease certificate.
            bool already_voted=false;
            if constexpr(by_source) {
              const auto visited=static_cast<unsigned char>(md_.flag_traverse_[address]+1);
              if(visited&source_visit_mask)return;
              already_voted=visited!=0;
              md_.flag_traverse_[address]=static_cast<char>((visited|source_visit_mask)-1);
            }
            // Return only from this visitor, NEVER stop DDA or refund its
            // budget. A different ray still traverses its entire remaining
            // segment. Same-round duplicate free votes with no newer real
            // source have nothing to change; diagnostic witnesses retain all
            // original visits on the slow path. A newer peer stamp must pass.
            if constexpr(!by_source) {
              if(fast_visitor&&md_.flag_traverse_[address]==md_.raycast_num_&&
                 (!has_ray_stamp||static_cast<std::size_t>(address)>=free_observation_stamps_.size()||
                  ray_source_stamp<=free_observation_stamps_[address]))return;
            }
            if(has_ray_stamp &&
                static_cast<std::size_t>(address)<free_observation_stamps_.size()) {
              auto &old=free_observation_stamps_[address];
              const auto incoming=ray_source_stamp;
              // Thousands of rays in one scan share the same source begin.
              // Avoid repeated floating-point checks and writes for them, but
              // retain a genuinely newer traversal from the other sensor.
              if(incoming>old) {
                const auto fresh=[this](std::int64_t stamp) {
                  return stamp>0 && collision_cache_clock_ns_>=stamp &&
                      (collision_cache_clock_ns_-stamp)*1e-9<mp_.cloud_pose_max_age_;
                };
                if(mp_.require_observed_free_ && enforce_free_freshness_ && !ray_clock_fault_ && !fresh(old) && fresh(incoming) &&
                    scan_planner::strictRawVoxelStatus(md_.occupancy_buffer_[address],
                        mp_.clamp_min_log_,mp_.min_occupancy_log_)==0)
                  free_evidence_recovered_=true;
                old=incoming;
              }
            }
            if constexpr(!by_source) {if(record_ray_diagnostics) {
              scan_planner::RayWitness witness;
              witness.metadata=projected_diagnostics_[i];witness.origin=origin;
              witness.endpoint=md_.proj_points_[i];witness.integrated_endpoint=pt_w;
              witness.contributed_vote=md_.flag_traverse_[address]!=md_.raycast_num_;
              near_field_diagnostics_.record(cell,witness);
            }}
            if constexpr(by_source) {
              if(already_voted)return;
            } else {
              if (md_.flag_traverse_[address]==md_.raycast_num_) return;
              md_.flag_traverse_[address]=md_.raycast_num_;
            }
            cacheOccupancyAtIndex(cell,address,0);
          };
      // Select the visitor once per ray, rather than branching on its source
      // encoding for millions of traversed cells.
      if(last_parallel_ray_visits_)
        parallel_ray_visits_->replay(static_cast<std::size_t>(i),
            [&](const Eigen::Vector3i& cell,int address){visitor(cell,address,std::true_type{});});
      else if(source_visit_mask)
        complete=scan_planner::visitObservedRayIndexed(origin,pt_w,mp_.resolution_,
            include_endpoint,observed_ray_budget,mp_.map_voxel_num_,
            [&](const Eigen::Vector3i& cell,int address){visitor(cell,address,std::true_type{});});
      else
        complete=scan_planner::visitObservedRayIndexed(origin,pt_w,mp_.resolution_,
            include_endpoint,observed_ray_budget,mp_.map_voxel_num_,
            [&](const Eigen::Vector3i& cell,int address){visitor(cell,address,std::false_type{});});
      if (!complete) break;
      continue;
    }

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  const auto traversal_finished=std::chrono::steady_clock::now();
  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  // Fixed diagnostic address set, resolved AFTER sliding. No per-ray work.
  const auto watched=blocked_voxel_watch_.snapshot();
  std::array<int,scan_planner::kBlockedVoxelWatchCells> watched_addresses{{-1,-1,-1,-1}};
  bool watching=false;
  if(mp_.use_projected_rays_)for(std::size_t i=0;i<watched.cells.size();++i) {
    const auto& cell=watched.cells[i];
    const Eigen::Vector3i index(cell.key[0],cell.key[1],cell.key[2]);
    if(cell.active&&isInMap(index)) {watched_addresses[i]=ray_index.address(index);watching=true;}
  }

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    const int idx_ctns = ray_index.address(idx);
    md_.cache_voxel_.pop();

    const bool watched_cell=watching&&std::find(watched_addresses.begin(),watched_addresses.end(),idx_ctns)!=watched_addresses.end();
    const auto old_odds=watched_cell?md_.occupancy_buffer_[idx_ctns]:0.;
    const auto hit_votes=watched_cell?md_.count_hit_[idx_ctns]:0;
    const auto miss_votes=watched_cell?md_.count_hit_and_miss_[idx_ctns]-hit_votes:0;
    const auto report_update=[&](scan_planner::BlockedVoxelUpdate outcome) {
      if(!watched_cell)return;
      const auto odds=md_.occupancy_buffer_[idx_ctns];
      blocked_voxel_watch_.recordUpdate({{idx.x(),idx.y(),idx.z()}},
          {old_odds,true,old_odds>mp_.min_occupancy_log_},
          {odds,true,odds>mp_.min_occupancy_log_},hit_votes,miss_votes,outcome);
    };

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      report_update(scan_planner::BlockedVoxelUpdate::HitSaturated);
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdateAtIndex(idx, idx_ctns, mp_.clamp_min_log_);
      report_update(scan_planner::BlockedVoxelUpdate::MissSaturated);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!mp_.use_projected_rays_ && !in_local)
    {
      // Never turn an out-of-window cell into observed free merely because
      // this frame's integration window does not include it.
      report_update(scan_planner::BlockedVoxelUpdate::OutsideLocalUpdate);
      continue;
    }

    // Projected rays were already clipped against EACH physical sensor's
    // measured origin, local range and map bounds. Cropping those real visits
    // again around the averaged sliding-window centre drops valid miss votes
    // when the second sensor joins a fusion. It can leave an occupied cell at
    // maximum odds even while its free-observation stamp keeps advancing.

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdateAtIndex(idx, idx_ctns, new_log_odds);
    report_update(scan_planner::BlockedVoxelUpdate::Applied);
  }
  const auto ray_finished=std::chrono::steady_clock::now();
  map_work_timing_.initialize_ms=std::chrono::duration<double,std::milli>(traversal_started-ray_started).count();
  map_work_timing_.traverse_ms=std::chrono::duration<double,std::milli>(traversal_finished-traversal_started).count();
  map_work_timing_.occupancy_ms=std::chrono::duration<double,std::milli>(ray_finished-traversal_finished).count();
  map_work_timing_.points=md_.proj_points_cnt;
  map_work_timing_.ray_steps=16000000-observed_ray_budget;
  map_work_timing_.raycast_end_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      ray_finished.time_since_epoch()).count();
  if (!complete && node_)
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[GridMap] observed-ray traversal incomplete; this scan cannot authorize planning");
  if (complete_rays && node_)
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[GridMap] complete ray integration: query_policy=%s points=%d steps=%zu duration_ms=%.3f complete=%d",
      collisionQueryPolicy(),md_.proj_points_cnt,16000000-observed_ray_budget,
      std::chrono::duration<double,std::milli>(ray_finished-ray_started).count(),
      static_cast<int>(complete));
  return complete;
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback()
{
  // Copy only display classifications on the fusion owner. The single worker
  // owns expensive OBB dilation, PointCloud2 conversion and all visual output.
  // If it is busy, skip this display frame: never queue behind it or take a
  // collision-reader slot away from current-curve renewal.
  startVisualizationWorker();
  submitVisualization(node_->now().nanoseconds(),
      map_pub_->get_subscription_count()>0,map_inf_pub_->get_subscription_count()>0,
      depth_cloud_pub_->get_subscription_count()>0);
}

bool GridMap::visualizationRawCell(std::size_t address,unsigned char flag) const
{
  if(visualization_only_)return (visualization_raw_flags_[address]&flag)!=0;
  const double odds=md_.occupancy_buffer_[address];
  if(flag==1)return odds>mp_.min_occupancy_log_; // Historical raw OBB display.
  if(flag==2)return std::isfinite(odds)&&odds>mp_.min_occupancy_log_; // OBB mask.
  return !(odds<mp_.min_occupancy_log_); // Historical non-OBB raw display.
}

rclcpp::Time GridMap::visualizationStamp() const
{
  // A display snapshot has its capture timestamp, not worker completion time.
  // This is not a ray source timestamp and grants no perception/control lease.
  return visualization_only_?rclcpp::Time(visualization_capture_ns_):node_->now();
}

bool GridMap::visualizationCurrent() const
{
  return !visualization_only_ || (visualization_current_&&visualization_current_());
}

void GridMap::copyVisualizationTo(GridMap& out,std::int64_t stamp,
    bool raw,bool inflated,bool depth) const
{
  if(&out==this || out.node_ || out.collision_snapshot_ ||
      !out.md_.occupancy_buffer_.empty() || !out.free_observation_stamps_.empty())
    throw std::logic_error("visualization destination must have no collision storage");
  out.visualization_only_=true;out.mp_=mp_;
  out.md_.has_ray_pose_=md_.has_ray_pose_;out.md_.ray_pos_=md_.ray_pos_;
  out.md_.sliding_map_frame_pos_=md_.sliding_map_frame_pos_;
  out.visualization_body_stamp_ns_=visualization_body_stamp_ns_;
  out.visualization_body_yaw_=visualization_body_yaw_;
  out.pair_barrier_ns_=pair_barrier_ns_;
  std::atomic_store_explicit(&out.visualization_body_envelope_,
      std::atomic_load_explicit(&visualization_body_envelope_,std::memory_order_acquire),
      std::memory_order_release);
  out.occupancy_revision_=occupancy_revision_;
  out.visualization_capture_ns_=stamp;
  out.visualization_capture_generation_=visualization_generation_.load(std::memory_order_acquire);
  out.visualization_raw_requested_=raw;out.visualization_inflated_requested_=inflated;
  out.visualization_depth_requested_=depth;
  if(raw || inflated) {
    out.visualization_raw_flags_.resize(md_.occupancy_buffer_.size());
    for(std::size_t i=0;i<md_.occupancy_buffer_.size();++i) {
      const double odds=md_.occupancy_buffer_[i];
      out.visualization_raw_flags_[i]=(odds>mp_.min_occupancy_log_?1:0)|
        (std::isfinite(odds)&&odds>mp_.min_occupancy_log_?2:0)|
        (!(odds<mp_.min_occupancy_log_)?4:0);
    }
    if(inflated && mp_.collision_model_!="oriented_box")
      out.md_.occupancy_buffer_inflate_=md_.occupancy_buffer_inflate_;
    else out.md_.occupancy_buffer_inflate_.clear();
  }
  out.md_.proj_points_cnt=depth?md_.proj_points_cnt:0;
  if(depth)out.md_.proj_points_.assign(md_.proj_points_.begin(),
      md_.proj_points_.begin()+md_.proj_points_cnt);
  else out.md_.proj_points_.clear();
  out.map_pub_=map_pub_;out.map_inf_pub_=map_inf_pub_;
  out.depth_cloud_pub_=depth_cloud_pub_;out.sliding_map_bbox_pub_=sliding_map_bbox_pub_;
  out.tf_broadcaster_=tf_broadcaster_;
}

void GridMap::startVisualizationWorker()
{
  if(visualization_worker_.joinable() || visualization_stop_)return;
  visualization_view_=std::make_unique<GridMap>();
  visualization_view_->visualization_current_=[this] {
    return !visualization_stop_.load(std::memory_order_acquire) &&
      visualization_view_->visualization_capture_generation_==
        visualization_generation_.load(std::memory_order_acquire);
  };
  visualization_worker_=std::thread([this]{runVisualizationWorker();});
}

void GridMap::stopVisualizationWorker()
{
  visualization_stop_.store(true,std::memory_order_release);
  visualization_generation_.fetch_add(1,std::memory_order_release);
  visualization_wake_.notify_all();
  if(visualization_worker_.joinable())visualization_worker_.join();
}

bool GridMap::submitVisualization(std::int64_t stamp,bool raw,bool inflated,bool depth)
{
  std::unique_lock<std::mutex> lock(visualization_mutex_,std::try_to_lock);
  if(!lock || visualization_stop_ || !visualization_view_ || visualization_ready_)return false;
  copyVisualizationTo(*visualization_view_,stamp,raw,inflated,depth);
  visualization_ready_=true;lock.unlock();visualization_wake_.notify_one();return true;
}

void GridMap::runVisualizationWorker()
{
  std::unique_lock<std::mutex> lock(visualization_mutex_);
  while(!visualization_stop_) {
    // stopVisualizationWorker deliberately never takes this mutex (rendering
    // may be busy). A bounded wait also closes the notify-before-wait shutdown
    // race without making the fusion owner or destructor wait for that mutex.
    visualization_wake_.wait_for(lock,std::chrono::milliseconds(100),
        [this]{return visualization_stop_||visualization_ready_;});
    if(visualization_stop_)break;
    if(!visualization_ready_)continue;
    visualization_ready_=false;
    // Keep this exclusive render lease through conversion/publication. The
    // writer ONLY try_locks it, so an arbitrarily slow display drops frames
    // instead of delaying fusion. There is no pending queue or growing pool.
    try {
      auto& view=*visualization_view_;
      if(!view.visualizationCurrent())continue;
      if(visualization_sink_)visualization_sink_(view);
      else {
        if(view.visualization_raw_requested_)view.publishMap();
        if(view.visualization_inflated_requested_)view.publishMapInflate(true);
        if(view.visualization_depth_requested_)view.publishDepthCloud();
        view.publishSlidingMapFrame();view.publishSlidingMapBBox();
      }
    } catch(const std::bad_alloc&) {std::terminate();}
    catch(const std::exception& error) {
      if(node_)RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
        "Display frame dropped: %s",error.what());
    }
  }
}

void GridMap::updateOccupancyCallback()
{
  if (mp_.use_projected_rays_) {
    updateProjectedOccupancy(true);
    return;
  }
  if (mp_.exact_cloud_pose_sync_ && mp_.sensor_type_ == "lidar")
  {
    processExactCloudPosePairs();
    // Pair receipt freshness is not sufficient: a queued scan must still be
    // fresh at the instant its actual rays enter the occupancy map.
    if (md_.occ_need_update_ && !cloudPoseStampFresh(projected_cloud_stamp_ns_))
    {
      md_.occ_need_update_ = false;
      md_.proj_points_cnt = 0;
    }
  }
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  const bool rays_complete=raycastProcess();
  if (!rays_complete) integrated_cloud_stamp_ns_=0;
  else if (md_.use_cloud_update_ && projected_cloud_stamp_ns_ > 0)
    integrated_cloud_stamp_ns_ = projected_cloud_stamp_ns_;
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

void GridMap::updateProjectedOccupancy(bool watchdog)
{
  // Timer and coalesced input wake both belong to the SAME exclusive writer.
  // A readiness wake does not get the watchdog's missing-peer permission.
  if(projected_ingress_enabled_)transferProjectedIngress(node_->now().nanoseconds(),std::chrono::steady_clock::now());
  else drainProjectedIngress();
  const auto now=std::chrono::steady_clock::now();
  tryProjectedFusion(node_->now().nanoseconds(),now,watchdog);
  publishProjectedRaysStatus(node_->now().nanoseconds(),std::chrono::steady_clock::now());
  scheduleProjectedFusionWake();
}

void GridMap::drainProjectedIngress()
{
  if(!projected_rays_sub_)return;
  rclcpp::MessageInfo info;
  if(projected_ingress_enabled_) {
    // Only the serial ingress executor takes this subscription in formal
    // sessions. Map watchdogs consume decoded mailboxes, never the RMW queue.
    projected_ingress_.recordDrain(scan_planner::drainProjectedRayIngress<d1max_planning_interfaces::msg::ProjectedRays>(
      [this,&info](auto& message){return projected_rays_sub_->take(message,info);},
      [this](const auto& message){return projected_ingress_.accept(message,node_->now().nanoseconds(),std::chrono::steady_clock::now());}));
    return;
  }
  ray_ingress_last_drain_={};
  ray_ingress_last_drain_=scan_planner::drainProjectedRayIngress<d1max_planning_interfaces::msg::ProjectedRays>(
    [this,&info](auto& message){return projected_rays_sub_->take(message,info);},
    [this](const auto& message){return acceptProjectedRays(message,node_->now().nanoseconds(),std::chrono::steady_clock::now());});
  ray_ingress_drain_taken_count_+=ray_ingress_last_drain_.taken;
  ray_ingress_drain_accepted_count_+=ray_ingress_last_drain_.accepted;
}

void GridMap::publishProjectedIngressContext()
{
  if(!projected_ingress_enabled_)return;
  auto context=std::make_shared<scan_planner::ProjectedRayContext>();
  context->session=mp_.localization_session_id_;context->seed=localization_seed_;
  context->frame=mp_.frame_id_;context->epoch=localization_epoch_;
  context->sequence=localization_context_sequence_;context->barrier=localization_context_barrier_ns_;
  context->pair_barrier=pair_barrier_ns_;context->maximum_age=mp_.cloud_pose_max_age_;
  context->diagnostics=near_field_diagnostics_.enabled();
  projected_ingress_context_=std::move(context);
  projected_ingress_.setContext(projected_ingress_context_);
}

void GridMap::transferProjectedIngress(std::int64_t source_now,std::chrono::steady_clock::time_point now)
{
  // Only the writer can call this or read pending_rays_. The ingress producer
  // keeps accepting into its own bounded slots during the subsequent raycast.
  const auto report=projected_ingress_.takeLatest(projected_ingress_context_,pending_rays_,source_now);
  ray_unattributed_drops_+=report.unattributed;
  ray_ingress_callback_count_=report.callbacks;
  ray_ingress_drain_taken_count_=report.drain_taken;ray_ingress_drain_accepted_count_=report.drain_accepted;
  ray_ingress_last_drain_=report.last_drain;
  for(std::size_t sensor=0;sensor<2;++sensor) {
    ray_drops_[sensor]+=report.drops[sensor];
    auto& pending=pending_rays_[sensor];
    if(!pending.ready)continue;
    // Second identity fence on the consumer boundary; a reset never imports
    // a partially decoded or former-coordinate batch, even if it was latest.
    if(!projected_ingress_context_ || pending.context!=projected_ingress_context_ ||
       pending.batch.stamp_ns<=pair_barrier_ns_) {pending.ready=false;++ray_drops_[sensor];continue;}
    if(report.taken[sensor])pending.writer_received=now;
    if(pending.batch.stamp_ns>ray_received_stamps_[sensor] &&
       pending.projection_sequence>ray_projection_sequences_[sensor]) {
      ray_received_stamps_[sensor]=pending.batch.stamp_ns;
      ray_projection_sequences_[sensor]=pending.projection_sequence;
      ray_accept_clock_ns_[sensor]=pending.accepted_ns;
    }
  }
}

namespace {
std::optional<std::chrono::nanoseconds> projectedFusionPeriod(double seconds)
{
  using Nanoseconds=std::chrono::nanoseconds;
  // Use one conservative integral duration for admission and timer scheduling.
  // In particular, 0.1 * 1.5 is slightly greater than 0.15 in double: comparing
  // those independently can reject the exact 150 ms watchdog boundary.
  // Leave room for the 3/2 watchdog duration before converting to int64.
  constexpr auto maximum=Nanoseconds::max()/2;
  if(!std::isfinite(seconds)||seconds<=0.||
     seconds>=std::chrono::duration<double>(maximum).count())return std::nullopt;
  const auto period=std::chrono::ceil<Nanoseconds>(std::chrono::duration<double>(seconds));
  if(period<=Nanoseconds::zero()||period>maximum)return std::nullopt;
  return period;
}
}

bool GridMap::tryProjectedFusion(std::int64_t now_ns,std::chrono::steady_clock::time_point now,bool watchdog)
{
  const auto period=projectedFusionPeriod(ray_integration_period_s_);
  if(!period)return false;
  const auto elapsed=now-ray_fusion_last_start_;
  if(ray_fusion_started_&&(now<ray_fusion_last_start_||elapsed<*period))return false;
  if(!pending_rays_[0].ready&&!pending_rays_[1].ready)return false; // No clock/lease renewal without new observations.
  std::int64_t pair=0;
  if(pending_rays_[0].ready&&pending_rays_[1].ready)pair=std::min(pending_rays_[0].batch.stamp_ns,pending_rays_[1].batch.stamp_ns);
  // Rate limiting belongs to the steady integration clock above. Requiring
  // another full period of *acquisition* advance here introduces a second
  // unsynchronised throttle: a fresh complete pair can miss the integration
  // boundary and wait another scan. Both sources must instead be genuinely
  // newer than their own last integrated observations. Their original stamps
  // and configured source-age deadlines are never extended by this scheduling.
  const bool pair_ready=pair>0 &&
      pending_rays_[0].batch.stamp_ns>ray_integrated_stamps_[0] &&
      pending_rays_[1].batch.stamp_ns>ray_integrated_stamps_[1];
  // Missing one sensor must still allow its peer to add OCCUPIED evidence;
  // the absent sensor's acquisition stamp remains old and will block motion.
  const auto watchdog_period=*period+(*period+std::chrono::nanoseconds(1))/2;
  if(!pair_ready&&(!watchdog||(ray_fusion_started_&&elapsed<watchdog_period)))return false;
  // A completed map can still await a validation slot. If its first retry
  // is busy, yield this executor turn once instead of immediately starting
  // another long raycast. The existing publication timer/next ray callback
  // can deliver it when a reader releases. Keep pending rays and all source,
  // integration and rate clocks untouched. A second runnable call must fuse
  // even if still busy, so a long owner cannot indefinitely stop new input.
  if(collision_publication_retry_callback_ && !collision_publication_retry_callback_() &&
     !ray_pending_publication_yielded_) {
    ray_pending_publication_yielded_=true;
    return false;
  }
  ray_fusion_started_=true;ray_fusion_last_start_=now;
  if(pair>0)ray_fusion_last_pair_stamp_=pair;
  processProjectedRays(now_ns,now);return true;
}

std::chrono::nanoseconds GridMap::projectedFusionWakeDelay(std::chrono::steady_clock::time_point now) const
{
  // This is a wake-up deadline, never an integration permission. The actual
  // start-to-start cap remains in tryProjectedFusion, including after a late
  // callback. No absolute-phase catch-up may shorten the configured period.
  constexpr std::int64_t maximum=20000000,minimum=1000000;
  const auto period=projectedFusionPeriod(ray_integration_period_s_);
  if(!ray_fusion_started_||!period||now<ray_fusion_last_start_)return std::chrono::nanoseconds(maximum);
  const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(now-ray_fusion_last_start_);
  const auto remaining=(*period-elapsed).count();
  // At/past due with no complete pair, continue bounded watchdog polling; ray
  // callbacks can still integrate immediately. Never busy-spin for data.
  if(remaining<=0)return std::chrono::nanoseconds(maximum);
  return std::chrono::nanoseconds(std::clamp(remaining,minimum,maximum));
}

void GridMap::scheduleProjectedFusionWake()
{
  if(!mp_.use_projected_rays_||!occ_timer_)return;
  ray_timer_delay_ns_=projectedFusionWakeDelay(std::chrono::steady_clock::now()).count();
  std::int64_t old_period=0;
  // All callers are on the existing mutually-exclusive map writer group.
  // Exchange then reset changes the NEXT deadline rather than inheriting the
  // old 20 ms polling phase. No new timer, thread or concurrent map mutation.
  if(rcl_timer_exchange_period(occ_timer_->get_timer_handle().get(),ray_timer_delay_ns_,&old_period)!=RCL_RET_OK) {
    ++ray_timer_schedule_errors_;
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"Cannot schedule projected-ray deadline");
    rcl_reset_error();return;
  }
  occ_timer_->reset();
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
  if (mp_.use_projected_rays_) return;
  if (mp_.require_localization_context_ && !localization_context_sequence_) return;
  if (mp_.sensor_type_ != "lidar")
    return;
  if ((mp_.strict_input_frames_ || mp_.exact_cloud_pose_sync_) && pose_msg->header.frame_id != mp_.frame_id_)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[GridMap] reject sensor pose in wrong world frame");
    return;
  }
  if (mp_.exact_cloud_pose_sync_)
  {
    const auto stamp = rclcpp::Time(pose_msg->header.stamp).nanoseconds();
    if (stamp <= std::max(pair_barrier_ns_, last_paired_stamp_ns_) || !cloudPoseStampFresh(stamp))
      return;
    const auto &pose = pose_msg->pose.pose;
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                         pose.orientation.y, pose.orientation.z);
    if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-6)
      return;
    pending_poses_.emplace(stamp, PendingPose{pose_msg, std::chrono::steady_clock::now()});
    while (pending_poses_.size() > 8)
    {
      pending_poses_.erase(pending_poses_.begin());
      ++cloud_pose_pair_drops_;
    }
    processExactCloudPosePairs();
    return;
  }
  applyLidarPose(pose_msg);
}

bool GridMap::applyLidarPose(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{

  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (!ray_q.coeffs().allFinite() || ray_q.norm() < 1e-6)
    return false;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return false;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  md_.ray_pose_stamp_ = rclcpp::Time(pose_msg->header.stamp);
  updateSlidingMap(md_.ray_pos_);

  // A paired pose alone supplies no free-space evidence. Its real ray traversal
  // includes the sensor-origin cell only when this actual cloud is integrated.
  return true;
}

bool GridMap::cloudPoseStampFresh(std::int64_t stamp) const
{
  if (stamp <= 0) return false;
  const double age = (node_->now().nanoseconds() - stamp) * 1e-9;
  return age >= -0.1 && age <= mp_.cloud_pose_max_age_;
}

bool GridMap::acceptProjectedRays(const d1max_planning_interfaces::msg::ProjectedRays &message,
    std::int64_t now_ns, std::chrono::steady_clock::time_point received)
{
  const auto stamp=[](const builtin_interfaces::msg::Time &time)->std::int64_t {
    if (time.sec<0 || time.nanosec>=1000000000U) return 0;
    return static_cast<std::int64_t>(time.sec)*1000000000LL+time.nanosec;
  };
  const auto begin=stamp(message.rays.header.stamp), end=stamp(message.acquisition_end);
  const auto alignment=stamp(message.alignment_stamp);
  if (!mp_.use_projected_rays_ || !localization_context_sequence_ ||
      message.session_id!=mp_.localization_session_id_ || message.epoch!=localization_epoch_ ||
      message.seed_id!=localization_seed_ || message.context_sequence!=localization_context_sequence_ ||
      message.barrier_ns!=localization_context_barrier_ns_ || !message.projection_sequence ||
      begin<=pair_barrier_ns_ || alignment<=pair_barrier_ns_ || end<begin || end-begin>250000000LL ||
      !scan_planner::rayStampFresh(begin,now_ns,mp_.cloud_pose_max_age_) ||
      !scan_planner::rayStampFresh(end,now_ns,mp_.cloud_pose_max_age_) ||
      !scan_planner::rayStampFresh(alignment,now_ns,mp_.cloud_pose_max_age_)) {
    ++ray_unattributed_drops_;return false;
  }
  if(!scan_planner::decodeProjectedRaysInto(message.rays,mp_.frame_id_,
      near_field_diagnostics_.enabled(),ray_decode_scratch_)) {
    ++ray_unattributed_drops_;return false;
  }
  auto &batch=ray_decode_scratch_;
  const auto sensor=batch.sensor_id;
  for(auto &meta:batch.diagnostics) {
    meta.context_sequence=message.context_sequence;meta.projection_sequence=message.projection_sequence;
    meta.received_ns=now_ns;meta.alignment_stamp_ns=alignment;meta.acquisition_end_ns=end;
  }
  if (begin<=ray_received_stamps_[sensor] || message.projection_sequence<=ray_projection_sequences_[sensor]) {
    ++ray_drops_[sensor];return false;
  }
  // One bounded latest acquisition per physical source, not one fused queue
  // where the faster front callback can continually overwrite the rear.
  if (pending_rays_[sensor].ready) ++ray_drops_[sensor];
  ray_received_stamps_[sensor]=begin;ray_projection_sequences_[sensor]=message.projection_sequence;
  ray_accept_clock_ns_[sensor]=now_ns;
  auto &pending=pending_rays_[sensor];
  std::swap(pending.batch,batch);pending.received=pending.writer_received=received;pending.ready=true;
  return true;
}

void GridMap::processProjectedRays(std::int64_t now_ns,std::chrono::steady_clock::time_point now)
{
  if (!mp_.use_projected_rays_) return;
  // This runnable transaction starts a new bounded publication opportunity;
  // tryProjectedFusion has already retried/yielded for its predecessor.
  ray_pending_publication_yielded_=false;
  map_work_timing_={};
  last_parallel_ray_visits_=false;
  ++fusion_timing_.sequence;
  fusion_timing_.begin_ns=now_ns;
  fusion_timing_.begin_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  // Clock progress expires free queries but never changes occupancy odds.
  advanceRayEvidenceClock(now_ns,now,node_!=nullptr);
  beginCollisionQuery();
  md_.proj_points_.clear();md_.proj_origins_.clear();md_.proj_points_cnt=0;
  projected_ray_stamps_.clear();projected_ray_kinds_.clear();
  projected_diagnostics_.clear();
  std::array<std::int64_t,2> included{{0,0}};
  std::array<PairReceipt,2> receipts;
  Eigen::Vector3d roi_center=Eigen::Vector3d::Zero();
  unsigned sources=0;
  bool fusion_complete=false;
  for (std::size_t sensor=0;sensor<2;++sensor) {
    auto &pending=pending_rays_[sensor];
    if (!pending.ready) continue;
    // The receiver is now independent of a busy writer. Keep first receipt
    // for integrated total age, but preserve the original pair-wait meaning:
    // time actually waiting in the writer's pending set, not prior fusion.
    const double receipt_age=std::chrono::duration<double>(now-pending.writer_received).count();
    const double first_receipt_age=std::chrono::duration<double>(now-pending.received).count();
    if (pending.batch.stamp_ns<=pair_barrier_ns_ || first_receipt_age<0. ||
        receipt_age<0. || receipt_age>mp_.cloud_pose_pair_wait_ ||
        !scan_planner::rayStampFresh(pending.batch.stamp_ns,now_ns,mp_.cloud_pose_max_age_)) {
      ++ray_drops_[sensor];pending.ready=false;continue;
    }
    const auto &batch=pending.batch;
    ray_integration_start_ns_[sensor]=now_ns;
    ray_integration_start_stamps_[sensor]=batch.stamp_ns;
    ray_pending_wait_s_[sensor]=receipt_age;
    // This representative is ONLY a sliding-window centre. It never replaces
    // a measured ray origin, and supplies no synthetic free evidence.
    roi_center+=batch.origins.front();++sources;
    included[sensor]=batch.stamp_ns;receipts[sensor]=pending.received;
    md_.proj_points_.insert(md_.proj_points_.end(),batch.endpoints.begin(),batch.endpoints.end());
    md_.proj_origins_.insert(md_.proj_origins_.end(),batch.origins.begin(),batch.origins.end());
    projected_ray_stamps_.insert(projected_ray_stamps_.end(),batch.endpoints.size(),batch.stamp_ns);
    if(batch.ray_kinds.empty())
      projected_ray_kinds_.insert(projected_ray_kinds_.end(),batch.endpoints.size(),0);
    else
      projected_ray_kinds_.insert(projected_ray_kinds_.end(),batch.ray_kinds.begin(),batch.ray_kinds.end());
    if(near_field_diagnostics_.enabled()) {
      for(auto meta:batch.diagnostics) {meta.integration_ns=now_ns;projected_diagnostics_.push_back(meta);}
    }
    pending.ready=false;
  }
  if (sources) {
    md_.ray_pos_=roi_center/static_cast<double>(sources);md_.ray_q_=Eigen::Quaterniond::Identity();
    md_.has_cloud_=md_.has_ray_pose_=true;md_.proj_points_cnt=static_cast<int>(md_.proj_points_.size());
    blocked_voxel_watch_.setContext(blocked_voxel_context_);
    scan_planner::BlockedVoxelWatchRequest watch_request;
    if(blocked_voxel_mailbox_&&blocked_voxel_mailbox_->tryTake(watch_request))
      blocked_voxel_watch_.acceptRequest(watch_request);
    scan_planner::BlockedVoxelFusion watch_fusion;
    watch_fusion.sequence=fusion_timing_.sequence;
    watch_fusion.source_stamps_ns=included;
    for(std::size_t i=0;i<2;++i)if(included[i])watch_fusion.projection_sequences[i]=ray_projection_sequences_[i];
    const auto watch_sampler=[this](const scan_planner::BlockedVoxelKey& key){return blockedVoxelRawState(key);};
    blocked_voxel_watch_.beginFusion(watch_fusion,watch_sampler);
    const bool complete=raycastProcess();
    blocked_voxel_watch_.finishFusion(watch_sampler);
    blocked_voxel_watch_snapshot_=blocked_voxel_watch_.snapshot();
    fusion_complete=complete;
    if(!complete) {
      // Incomplete batches cannot lend fresh regional leases to a later
      // successful remote batch. Keep obstacle odds; revoke free certificates
      // until genuine reobservation. Rare failure path, O(N).
      std::fill(free_observation_stamps_.begin(),free_observation_stamps_.end(),0);
      invalidateCollisionQueryCaches();
    } else if(free_evidence_recovered_) ++free_evidence_revision_;
    for (std::size_t sensor=0;sensor<2;++sensor) if (included[sensor]) {
      if (complete) {
        ray_integrated_stamps_[sensor]=included[sensor];ray_integrated_receipts_[sensor]=receipts[sensor];
        ++ray_integrations_[sensor];
      } else {ray_integrated_stamps_[sensor]=0;++ray_drops_[sensor];}
    }
  }
  // Neither packet receipt nor a continuing front stream renews rear support.
  // Check again after actual integration, not only before potentially expensive
  // ray traversal. Memory-only tests inject their clocks without a ROS node.
  if (node_) {now_ns=node_->now().nanoseconds();now=std::chrono::steady_clock::now();}
  integrated_cloud_stamp_ns_=std::min(ray_integrated_stamps_[0],ray_integrated_stamps_[1]);
  for (std::size_t sensor=0;sensor<2;++sensor) {
    const double age=std::chrono::duration<double>(now-ray_integrated_receipts_[sensor]).count();
    if (!scan_planner::rayStampFresh(ray_integrated_stamps_[sensor],now_ns,mp_.cloud_pose_max_age_) ||
        age<0. || age>mp_.cloud_pose_max_age_) integrated_cloud_stamp_ns_=0;
  }
  md_.occ_need_update_=md_.use_cloud_update_=false;
  fusion_timing_.end_ns=now_ns;
  fusion_timing_.end_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  fusion_timing_.source_stamps=ray_integrated_stamps_;
  if(fusion_complete)projected_ingress_.completedFusion(projected_ingress_context_,
      fusion_timing_.sequence,fusion_timing_.end_steady_ns);
  // Snapshot publication belongs to this transaction, not an unrelated periodic
  // timer that can copy the preceding frame just before fusion finishes.
  if(collision_update_callback_)collision_update_callback_();
  const auto transaction_end=std::chrono::steady_clock::now();
  const auto transaction_end_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      transaction_end.time_since_epoch()).count();
  map_work_timing_.writer_transaction_ms=(transaction_end_ns-fusion_timing_.begin_steady_ns)*1e-6;
  if(map_work_timing_.raycast_end_steady_ns)
    map_work_timing_.post_raycast_ms=(transaction_end_ns-map_work_timing_.raycast_end_steady_ns)*1e-6;
}

scan_planner::BlockedVoxelRawState GridMap::blockedVoxelRawState(const scan_planner::BlockedVoxelKey& key)
{
  const Eigen::Vector3i index(key[0],key[1],key[2]);
  if(!isInMap(index))return {};
  const auto odds=md_.occupancy_buffer_[toAddress(index)];
  return {odds,true,odds>mp_.min_occupancy_log_};
}

std::string GridMap::blockedVoxelWatchJson() const
{
  nlohmann::json cells=nlohmann::json::array();
  for(const auto& cell:blocked_voxel_watch_snapshot_.cells) {
    if(!cell.count)continue;
    nlohmann::json events=nlohmann::json::array();
    // Retain the fixed 16 actual fusion updates, including the clear event
    // after failure diagnostics cease. No pre-watch history is invented.
    for(std::size_t i=0;i<cell.count;++i) {
      const auto& sample=cell.samples[(cell.next+scan_planner::kBlockedVoxelWatchHistory-cell.count+i)%scan_planner::kBlockedVoxelWatchHistory];
      // Compress repeated no-change rounds in text only. The bounded native
      // history still retains every fusion, and the latest round is included.
      if(i+1<cell.count && sample.before.log_odds==sample.after.log_odds &&
         sample.transition==scan_planner::BlockedVoxelTransition::Unchanged)continue;
      events.push_back({{"fusion",sample.fusion.sequence},{"source_stamps_ns",sample.fusion.source_stamps_ns},
        {"projection_sequences",sample.fusion.projection_sequences},{"hit_votes",sample.hit_votes},
        {"miss_votes",sample.miss_votes},{"old_odds",sample.before.log_odds},{"new_odds",sample.after.log_odds},
        {"in_map",sample.after.in_map},{"occupied",sample.after.occupied},
        {"update",scan_planner::blockedVoxelUpdateName(sample.outcome)},
        {"transition",scan_planner::blockedVoxelTransitionName(sample.transition)}});
    }
    cells.push_back({{"index",cell.key},{"context",cell.context_generation},{"active",cell.active},{"events",events}});
  }
  return nlohmann::json{{"context",blocked_voxel_watch_snapshot_.context_generation},
    {"history_revision",blocked_voxel_watch_snapshot_.history_revision},
    {"vote_semantics","deduplicated_per_fusion_not_raw_ray_counts"},
    {"sources_semantics","fusion_inputs_not_per_cell_ray_attribution"},
    {"prewatch_history","unavailable"},{"cells",cells}}.dump();
}

void GridMap::publishProjectedRaysStatus(std::int64_t now_ns,std::chrono::steady_clock::time_point now)
{
  if (!projected_rays_status_pub_) return;
  const bool valid=integrated_cloud_stamp_ns_>0 &&
      scan_planner::rayStampFresh(integrated_cloud_stamp_ns_,now_ns,mp_.cloud_pose_max_age_);
  if (!ray_status_schedule_.due(now,integrated_cloud_stamp_ns_,valid,localization_context_sequence_)) return;
  if(node_&&blocked_voxel_watch_snapshot_.history_revision&&
     (blocked_voxel_last_log_==PairReceipt{}||now-blocked_voxel_last_log_>=std::chrono::seconds(1))) {
    blocked_voxel_last_log_=now;
    RCLCPP_INFO(node_->get_logger(),"BlockedVoxelTrace %s",blockedVoxelWatchJson().c_str());
  }
  nlohmann::json sources=nlohmann::json::array();
  const auto source_age=[](std::int64_t at,std::int64_t stamp)->nlohmann::json {
    return at>0 && stamp>0?nlohmann::json((at-stamp)*1e-9):nlohmann::json(nullptr);
  };
  for (std::size_t sensor=0;sensor<2;++sensor)
    sources.push_back({{"sensor_id",sensor},{"integrated_stamp_ns",ray_integrated_stamps_[sensor]},
      {"integrated_count",ray_integrations_[sensor]},{"drop_count",ray_drops_[sensor]},
      {"accepted_stamp_ns",ray_received_stamps_[sensor]},
      {"source_age_at_accept_s",source_age(ray_accept_clock_ns_[sensor],ray_received_stamps_[sensor])},
      {"integration_start_stamp_ns",ray_integration_start_stamps_[sensor]},
      {"source_age_at_integration_start_s",source_age(ray_integration_start_ns_[sensor],ray_integration_start_stamps_[sensor])},
      {"pending_wait_at_integration_start_s",ray_integration_start_ns_[sensor]>0?
          nlohmann::json(ray_pending_wait_s_[sensor]):nlohmann::json(nullptr)},
      {"integrated_source_age_s",source_age(now_ns,ray_integrated_stamps_[sensor])}});
  const double wall_time=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  std_msgs::msg::String result;
  result.data=nlohmann::json{{"schema",1},{"session_id",mp_.localization_session_id_},
    {"epoch",localization_epoch_},{"seed_id",localization_seed_},{"sequence",localization_context_sequence_},
    {"barrier_ns",localization_context_barrier_ns_},{"received_at_unix",wall_time},{"valid",valid},
    {"collision_query_policy",collisionQueryPolicy()},
    {"collision_model",mp_.collision_model_},
    {"inflated_visualization_semantics",inflatedVisualizationSemantics()},
    {"reason",valid?"integrated_both_sources":"waiting_fresh_integrated_rays"},
    {"source_stamp_ns",integrated_cloud_stamp_ns_},{"sources",sources},
    {"fusion",{{"sequence",fusion_timing_.sequence},{"begin_ns",fusion_timing_.begin_ns},
      {"end_ns",fusion_timing_.end_ns},{"begin_steady_ns",fusion_timing_.begin_steady_ns},
      {"end_steady_ns",fusion_timing_.end_steady_ns},
      {"front_begin_ns",fusion_timing_.source_stamps[0]},{"rear_begin_ns",fusion_timing_.source_stamps[1]},
      {"map_revision",occupancyRevision()},{"next_wake_delay_ns",ray_timer_delay_ns_},
      {"schedule_errors",ray_timer_schedule_errors_}}},
    {"integrated_counts",{{"0",ray_integrations_[0]},{"1",ray_integrations_[1]}}},
    {"work",{{"initialize_ms",map_work_timing_.initialize_ms},{"traverse_ms",map_work_timing_.traverse_ms},
      {"occupancy_ms",map_work_timing_.occupancy_ms},{"post_raycast_ms",map_work_timing_.post_raycast_ms},
      {"writer_transaction_ms",map_work_timing_.writer_transaction_ms},
      {"points",map_work_timing_.points},{"ray_steps",map_work_timing_.ray_steps},
      {"parallel_traversal",last_parallel_ray_visits_}}},
    {"ingress",{{"queue_depth",scan_planner::kProjectedRayIngressDepth},
      {"independent_receiver",projected_ingress_enabled_},
      {"accepted_clock_semantics",projected_ingress_enabled_?"ingress_callback_before_decode":"writer_callback_before_decode"},
      {"pending_wait_semantics","writer_admission_to_fusion"},
      {"writer_wake_failures",projected_writer_wake_?projected_writer_wake_->failureCount():0},
      {"callback_count",ray_ingress_callback_count_},{"drain_taken_count",ray_ingress_drain_taken_count_},
      {"drain_accepted_count",ray_ingress_drain_accepted_count_},
      {"last_drain_taken",ray_ingress_last_drain_.taken},{"last_drain_accepted",ray_ingress_last_drain_.accepted}}},
    {"unattributed_drop_count",ray_unattributed_drops_}}.dump();
  projected_rays_status_pub_->publish(result);
  if(node_) {
    const auto age_ms=[](std::int64_t at,std::int64_t stamp) {
      return at>0&&stamp>0?(at-stamp)*1e-6:std::numeric_limits<double>::quiet_NaN();
    };
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "RayPipelineTrace fusion=%lu valid=%d source_bound_ms=%.3f front_accept_age_ms=%.3f rear_accept_age_ms=%.3f front_pending_ms=%.3f rear_pending_ms=%.3f integration_ms=%.3f front_integrated_age_ms=%.3f rear_integrated_age_ms=%.3f front_source_ns=%ld rear_source_ns=%ld initialize_ms=%.3f traverse_ms=%.3f occupancy_ms=%.3f post_raycast_ms=%.3f writer_transaction_ms=%.3f points=%lu steps=%lu parallel=%d",
      fusion_timing_.sequence,static_cast<int>(valid),mp_.cloud_pose_max_age_*1000.,
      age_ms(ray_accept_clock_ns_[0],ray_received_stamps_[0]),age_ms(ray_accept_clock_ns_[1],ray_received_stamps_[1]),
      ray_pending_wait_s_[0]*1000.,ray_pending_wait_s_[1]*1000.,
      (fusion_timing_.end_steady_ns-fusion_timing_.begin_steady_ns)*1e-6,
      age_ms(now_ns,ray_integrated_stamps_[0]),age_ms(now_ns,ray_integrated_stamps_[1]),
      ray_integrated_stamps_[0],ray_integrated_stamps_[1],
      map_work_timing_.initialize_ms,map_work_timing_.traverse_ms,map_work_timing_.occupancy_ms,
      map_work_timing_.post_raycast_ms,map_work_timing_.writer_transaction_ms,
      map_work_timing_.points,map_work_timing_.ray_steps,static_cast<int>(last_parallel_ray_visits_));
  }
  ray_status_schedule_.markPublished(now,integrated_cloud_stamp_ns_,valid,localization_context_sequence_);
}

void GridMap::invalidateCloudPosePairs(std::int64_t barrier)
{
  // Close before clearing the map/context, so an old decode that finishes
  // during reset cannot become admissible after the new context ACK.
  projected_ingress_context_.reset();projected_ingress_.setContext({});
  pair_barrier_ns_ = std::max(pair_barrier_ns_, barrier);
  cloud_pose_pair_drops_ += pending_clouds_.size();
  pending_clouds_.clear();
  pending_poses_.clear();
  for (auto &pending:pending_rays_) pending.ready=false;
  ray_received_stamps_.fill(0);ray_integrated_stamps_.fill(0);ray_projection_sequences_.fill(0);
  ray_fusion_last_pair_stamp_=0;
  ray_accept_clock_ns_.fill(0);ray_integration_start_ns_.fill(0);ray_integration_start_stamps_.fill(0);
  ray_pending_wait_s_.fill(0.);
  ray_ingress_callback_count_=ray_ingress_drain_taken_count_=ray_ingress_drain_accepted_count_=0;
  ray_ingress_last_drain_={};
  md_.proj_origins_.clear();
  projected_cloud_stamp_ns_ = integrated_cloud_stamp_ns_ = 0;
  md_.occ_need_update_ = md_.use_cloud_update_ = false;
  md_.has_cloud_ = md_.has_ray_pose_ = false;
  md_.proj_points_cnt = 0;
  // Keep historical occupancy as historical data; do not pretend it is a new
  // empty/free map. latestCloudStamp()==0 prevents it authorizing a new plan.
}

void GridMap::processExactCloudPosePairs()
{
  if (!mp_.exact_cloud_pose_sync_) return;
  if (mp_.require_localization_context_ && !localization_context_sequence_) return;
  const auto now = std::chrono::steady_clock::now();
  const auto expired = [&](const auto &entry) {
    return entry.first <= std::max(pair_barrier_ns_, last_paired_stamp_ns_) ||
        !cloudPoseStampFresh(entry.first) ||
        std::chrono::duration<double>(now-entry.second.received).count() > mp_.cloud_pose_pair_wait_;
  };
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();)
    if (expired(*it)) { it = pending_clouds_.erase(it); ++cloud_pose_pair_drops_; }
    else ++it;
  for (auto it = pending_poses_.begin(); it != pending_poses_.end();)
    if (expired(*it)) { it = pending_poses_.erase(it); ++cloud_pose_pair_drops_; }
    else ++it;

  // Select the latest complete exact pair. A missing older counterpart cannot
  // block newer matched measurements or force use of the latest unrelated pose.
  std::int64_t selected = 0;
  for (auto it = pending_clouds_.rbegin(); it != pending_clouds_.rend(); ++it)
    if (pending_poses_.find(it->first) != pending_poses_.end()) { selected = it->first; break; }
  if (selected == 0) return;
  const auto cloud = pending_clouds_.at(selected).message;
  const auto pose = pending_poses_.at(selected).message;
  pending_clouds_.erase(pending_clouds_.begin(), pending_clouds_.upper_bound(selected));
  pending_poses_.erase(pending_poses_.begin(), pending_poses_.upper_bound(selected));
  last_paired_stamp_ns_ = selected;
  if (!applyLidarPose(pose)) { ++cloud_pose_pair_drops_; return; }
  // No callback mutates ray_pos_ in exact mode without replacing its cloud too.
  // This also removes the old cloud->latest pose race before the 50ms raycast timer.
  md_.occ_need_update_ = false;
  md_.proj_points_cnt = 0;
  processCloud(cloud);
  ++cloud_pose_pairs_;
}

void GridMap::slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  if (mp_.strict_input_frames_ && pose->header.frame_id != mp_.frame_id_) return;
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  const bool position_valid=std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z);
  if (position_valid) md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
  // body_pose is the measured robot pose. Projected-ray fusion deliberately
  // uses an identity ray quaternion; sensor extrinsics are not a body heading.
  // This history is exclusively for display and changes no map/query state.
  if (pose->header.frame_id!=mp_.frame_id_) return;
  if (pose->header.stamp.sec<0 || pose->header.stamp.nanosec>=1000000000U) return;
  const auto source=static_cast<std::int64_t>(pose->header.stamp.sec)*1000000000LL+
      pose->header.stamp.nanosec;
  if (source<=visualization_body_stamp_ns_ || source<=pair_barrier_ns_) return;
  const auto &q=pose->pose.pose.orientation;
  Eigen::Quaterniond body_q(q.w,q.x,q.y,q.z);
  const double norm=body_q.norm();
  if (!position_valid || !body_q.coeffs().allFinite() || !std::isfinite(norm) || norm<1e-6) {
    // A newer explicitly invalid body sample must not keep displaying an old
    // orientation as current. Foreign/duplicate/older samples cannot clear it.
    visualization_body_stamp_ns_=source;
    visualization_body_yaw_=std::numeric_limits<double>::quiet_NaN();
    visualization_obb_kernel_ready_=false;
    visualization_revision_[1]=std::numeric_limits<std::uint64_t>::max();
    return;
  }
  body_q.normalize();
  const auto rotation=body_q.toRotationMatrix();
  const double yaw=std::atan2(rotation(1,0),rotation(0,0));
  if (!std::isfinite(yaw)) return;
  visualization_body_stamp_ns_=source;
  visualization_body_yaw_=yaw;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  if (mp_.use_projected_rays_) return;
  if (mp_.sensor_type_ != "lidar")
    return;
  if (mp_.require_localization_context_ && !localization_context_sequence_) return;
  if (mp_.exact_cloud_pose_sync_)
  {
    const auto stamp = rclcpp::Time(img->header.stamp).nanoseconds();
    if ((mp_.cloud_is_world_ && img->header.frame_id != mp_.frame_id_) ||
        stamp <= pair_barrier_ns_ || !cloudPoseStampFresh(stamp))
    {
      ++cloud_pose_pair_drops_;
      return;
    }
    if (img->width == 0 && img->data.empty())
    {
      if (stamp >= last_paired_stamp_ns_) invalidateCloudPosePairs(stamp);
      return;
    }
    const auto points = static_cast<std::uint64_t>(img->width) * img->height;
    if (stamp <= last_paired_stamp_ns_ || points == 0 || points > 250000 ||
        img->data.size() > 32U * 1024U * 1024U || img->point_step < 12 ||
        static_cast<std::uint64_t>(img->row_step) * img->height != img->data.size() ||
        static_cast<std::uint64_t>(img->point_step) * img->width > img->row_step)
    {
      ++cloud_pose_pair_drops_;
      return;
    }
    pending_clouds_.emplace(stamp, PendingCloud{img, std::chrono::steady_clock::now()});
    while (pending_clouds_.size() > 3)
    {
      pending_clouds_.erase(pending_clouds_.begin());
      ++cloud_pose_pair_drops_;
    }
    processExactCloudPosePairs();
    return;
  }

  if (!md_.has_ray_pose_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] no sensor_pose received for lidar cloud update");
    return;
  }

  processCloud(img);
}

void GridMap::processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  if (mp_.strict_input_frames_)
  {
    const double pose_dt = std::abs((rclcpp::Time(img->header.stamp) - md_.ray_pose_stamp_).seconds());
    if ((mp_.cloud_is_world_ && img->header.frame_id != mp_.frame_id_) ||
        pose_dt > mp_.maximum_cloud_pose_dt_)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[GridMap] reject cloud: world-frame mismatch or cloud/pose skew %.3fs", pose_dt);
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  updateSlidingMap(ray_pos);

  md_.proj_points_cnt = 0;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    // Keep the measured endpoint. raycastProcess clips the segment and marks
    // its artificial endpoint as no-hit; dropping the entire segment here
    // discarded valid nearby free evidence from returns 6--8 m away.

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    md_.proj_points_cnt++;
  }

  if (md_.proj_points_cnt == 0)
    return;

  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
  projected_cloud_stamp_ns_ = rclcpp::Time(img->header.stamp).nanoseconds();
}

bool GridMap::prepareOrientedVisualizationMask()
{
  if (visualization_body_stamp_ns_<=0 || !std::isfinite(visualization_body_yaw_)) return false;
  auto body=planningBodyEnvelope();
  double yaw=visualization_body_yaw_,pitch=0.,roll=0.;
  const auto selected=std::atomic_load_explicit(&visualization_body_envelope_,std::memory_order_acquire);
  if(selected && selected->source_stamp_ns>pair_barrier_ns_ &&
      selected->frame==mp_.frame_id_ && selected->body.model==body.model) {
    if(!selected->yaw && selected->source_stamp_ns==visualization_body_stamp_ns_)body=selected->body;
    else if(selected->yaw && std::isfinite(mp_.cloud_pose_max_age_) && mp_.cloud_pose_max_age_>0. &&
        std::abs(static_cast<long double>(selected->source_stamp_ns)-visualization_body_stamp_ns_)<=
            mp_.cloud_pose_max_age_*1e9L) {
      // The independent owner and writer receive body_pose asynchronously.
      // Use the complete original-sample pair, never new yaw + old shape.
      body=selected->body;yaw=*selected->yaw;pitch=selected->pitch;roll=selected->roll;
    }
  }
  const std::array<double,8> key{{mp_.resolution_,body.length,body.width,
      body.below,body.above,yaw,pitch,roll}};
  bool kernel_changed=!visualization_obb_kernel_ready_;
  if (kernel_changed || key!=visualization_obb_kernel_key_) {
    // The same production SAT/face-contact rule as boxOccupancy(), sampled
    // only at lattice body centres. No quantized yaw or clearance is added.
    std::vector<Eigen::Vector2i> xy;
    std::vector<scan_planner::VerticalVoxelSpan> spans;
    const Eigen::Vector3d canonical=Eigen::Vector3d::Constant(mp_.resolution_*.5);
    if(pitch==0.&&roll==0.) {
      // Keep the existing level/yaw-only fast path and exact point ordering.
      scan_planner::forEachOrientedBoxColumn(canonical,yaw,mp_.resolution_,body,
        [&](int x,int y,const auto &span) {xy.emplace_back(x,y);spans.push_back(span);return true;});
    } else {
      // Reuse the same production 3-D SAT, not an upright bounding box.
      // A convex OBB intersects a contiguous range in each voxel column.
      scan_planner::forEachOrientedBoxVoxel(canonical,yaw,pitch,roll,mp_.resolution_,body,
        [&](const Eigen::Vector3i& cell) {
          if(xy.empty()||xy.back().x()!=cell.x()||xy.back().y()!=cell.y()) {
            xy.emplace_back(cell.x(),cell.y());spans.push_back({cell.z(),cell.z()});
          } else {
            if(cell.z()!=spans.back().high+1)throw std::logic_error("noncontiguous box visualization column");
            spans.back().high=cell.z();
          }
          return true;
        });
    }
    const bool same=visualization_obb_kernel_ready_ && xy.size()==visualization_obb_xy_offsets_.size() &&
      std::equal(xy.begin(),xy.end(),visualization_obb_xy_offsets_.begin(),
        [](const auto &a,const auto &b){return (a.array()==b.array()).all();}) &&
      spans.size()==visualization_obb_z_spans_.size() &&
      std::equal(spans.begin(),spans.end(),visualization_obb_z_spans_.begin(),
        [](const auto&a,const auto&b){return a.low==b.low&&a.high==b.high;});
    // Real yaw changes that intersect exactly the same discrete cells may
    // reuse that mask. This is set equality, not a yaw-bin approximation.
    kernel_changed=!same;
    visualization_obb_xy_offsets_=std::move(xy);visualization_obb_z_spans_=std::move(spans);
    visualization_obb_kernel_key_=key;visualization_obb_kernel_ready_=true;
    ++visualization_obb_kernel_builds_;
  }
  const auto &lo=mp_.map_bound_min_idx_,&hi=mp_.map_bound_max_idx_;
  if (!kernel_changed && visualization_obb_mask_revision_==occupancy_revision_ &&
      (visualization_obb_mask_min_.array()==lo.array()).all() &&
      (visualization_obb_mask_max_.array()==hi.array()).all()) return true;
  const Eigen::Vector3i count=hi-lo+Eigen::Vector3i::Ones();
  if ((count.array()<=0).any() || (count.array()!=mp_.map_voxel_num_.array()).any())
    throw std::logic_error("invalid oriented visualization map extent");
  const std::size_t nx=count.x(),ny=count.y(),nz=count.z(),row_size=nz+1;
  if (nx*ny*nz!=(visualization_only_?visualization_raw_flags_.size():md_.occupancy_buffer_.size()))
    throw std::logic_error("invalid oriented visualization raw buffer");
  visualization_obb_difference_.assign(nx*ny*row_size,0);
  visualization_obb_mask_.assign(nx*ny*nz,0);
  visualization_obb_interval_writes_=0;
  const int ring_z_size=mp_.map_voxel_num_.z(),ring_yz=mp_.map_voxel_num_.y()*ring_z_size;
  for (int x=lo.x();x<=hi.x();++x) for (int y=lo.y();y<=hi.y();++y) {
    const int raw_row=getLocalIndex(x,0)*ring_yz+getLocalIndex(y,1)*ring_z_size;
    int local_z=getLocalIndex(lo.z(),2),run_first=0;bool in_run=false;
    const auto emit=[&](int first,int last) {
      // Invert obstacle offsets to body-centre offsets, including asymmetric
      // body Z: centre=[hit.low-body.above,hit.high+body.below]. Higher hits
      // must be processed before applying the output's display-height clip.
      for (std::size_t i=0;i<visualization_obb_xy_offsets_.size();++i) {
        const auto &offset=visualization_obb_xy_offsets_[i];
        const auto &span=visualization_obb_z_spans_[i];
        const int lower=std::max(lo.z(),first-span.high);
        const int upper=std::min(hi.z(),last-span.low);
        if (lower>upper)continue;
        const int target_x=x-offset.x(),target_y=y-offset.y();
        if (target_x<lo.x() || target_x>hi.x() || target_y<lo.y() || target_y>hi.y()) continue;
        const std::size_t row=(static_cast<std::size_t>(target_x-lo.x())*ny+target_y-lo.y())*row_size;
        ++visualization_obb_difference_[row+lower-lo.z()];
        --visualization_obb_difference_[row+upper-lo.z()+1];
        ++visualization_obb_interval_writes_;
      }
    };
    for (int z=lo.z();z<=hi.z();++z) {
      const bool occupied=visualizationRawCell(raw_row+local_z,2);
      if (occupied && !in_run) {run_first=z;in_run=true;}
      if (!occupied && in_run) {emit(run_first,z-1);in_run=false;}
      if (++local_z==ring_z_size) local_z=0;
    }
    if (in_run) emit(run_first,hi.z());
  }
  for (std::size_t column=0;column<nx*ny;++column) {
    int overlaps=0;
    for (std::size_t z=0;z<nz;++z) {
      overlaps+=visualization_obb_difference_[column*row_size+z];
      visualization_obb_mask_[column*nz+z]=overlaps>0;
    }
  }
  visualization_obb_mask_revision_=occupancy_revision_;
  visualization_obb_mask_min_=lo;visualization_obb_mask_max_=hi;
  ++visualization_obb_mask_builds_;
  visualization_revision_[1]=std::numeric_limits<std::uint64_t>::max();
  return true;
}

sensor_msgs::msg::PointCloud2 &GridMap::cachedVisualization(bool inflated)
{
  const bool oriented=inflated && mp_.collision_model_=="oriented_box";
  const bool oriented_ready=oriented && prepareOrientedVisualizationMask();
  const std::size_t layer = inflated ? 1 : 0;
  const int clip = md_.has_ray_pose_ ? static_cast<int>(std::floor(
      (md_.ray_pos_.z() + mp_.vis_height_) * mp_.resolution_inv_ - .5)) :
      std::numeric_limits<int>::max();
  if (visualization_revision_[layer] == occupancy_revision_ && visualization_clip_[layer] == clip &&
      visualization_cache_[layer].header.frame_id==mp_.frame_id_ &&
      visualization_model_[layer]==mp_.collision_model_ &&
      (!oriented || visualization_obb_packet_ready_==oriented_ready))
    return visualization_cache_[layer];
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;
  // This is visualization only: omit exactly the same above-clip cells before
  // reading the ring buffers. Hoist X/Y ring addressing and advance Z locally
  // instead of performing three integer modulos for every displayed voxel.
  // Preserve the original global XYZ iteration order and point coordinates.
  max_cut.z() = std::min(max_cut.z(), clip);
  const int z_size = mp_.map_voxel_num_.z();
  const int yz_stride = mp_.map_voxel_num_.y() * z_size;
  const int first_local_z = getLocalIndex(min_cut.z(), 2);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
  {
    const int x_address = getLocalIndex(x, 0) * yz_stride;
    for (int y = min_cut(1); y <= max_cut(1); ++y)
    {
      const int row_address = x_address + getLocalIndex(y, 1) * z_size;
      int local_z = first_local_z;
      for (int z = min_cut(2); z <= max_cut(2); ++z,
           local_z = local_z + 1 == z_size ? 0 : local_z + 1)
      {
        const int address = row_address + local_z;
        const std::size_t visual_address=(static_cast<std::size_t>(x-min_cut.x())*mp_.map_voxel_num_.y()+
            y-min_cut.y())*mp_.map_voxel_num_.z()+z-min_cut.z();
        if (oriented ? !oriented_ready || visualization_obb_mask_[visual_address]==0 :
            inflated ? md_.occupancy_buffer_inflate_[address] == 0 :
            !visualizationRawCell(address,mp_.collision_model_=="oriented_box"?1:4))
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  pcl::toROSMsg(cloud, visualization_cache_[layer]);
  visualization_revision_[layer] = occupancy_revision_;
  visualization_clip_[layer] = clip;
  visualization_model_[layer]=mp_.collision_model_;
  if(oriented)visualization_obb_packet_ready_=oriented_ready;
  ++visualization_builds_[layer];
  return visualization_cache_[layer];
}

void GridMap::publishMap()
{
  if (map_pub_->get_subscription_count() == 0) return;
  auto &cloud_msg = cachedVisualization(false);
  if(!visualizationCurrent())return;
  cloud_msg.header.stamp = visualizationStamp();
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool /*all_info*/)
{
  if (map_inf_pub_->get_subscription_count() == 0) return;
  auto &cloud_msg = cachedVisualization(true);
  if(!visualizationCurrent())return;
  cloud_msg.header.stamp = visualizationStamp();
  map_inf_pub_->publish(cloud_msg);
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;
  if(!visualizationCurrent())return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = visualizationStamp();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;
  if(!visualizationCurrent())return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = visualizationStamp();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  unknown_pub_->publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  if(!visualizationCurrent())return;
  cloud_msg.header.stamp = visualizationStamp();
  depth_cloud_pub_->publish(cloud_msg);
}
