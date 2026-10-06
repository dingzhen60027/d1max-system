#include "plan_env/grid_map.h"
#include <plan_env/voxel_collision.hpp>
#include <plan_env/observed_ray.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <cstdlib>
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

void GridMap::initMap(rclcpp::Node *node)
{
  node_ = node;

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
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
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
  load_parameter(node_, "grid_map.use_projected_rays", mp_.use_projected_rays_, false);
  std::string simulation_clock_contract,transport_mode;
  load_parameter(node_, "grid_map.simulation_clock_contract", simulation_clock_contract,std::string{});
  load_parameter(node_, "grid_map.transport_mode", transport_mode,std::string{"live"});
  if(!simulation_clock_contract.empty()) {
    const auto env_equals=[](const char *key,const char *value) {
      const auto actual=std::getenv(key);return actual&&std::string(actual)==value;
    };
    if(simulation_clock_contract!="isaac_fixed_anchor_v1" || transport_mode!="isolated_mock" ||
        !node_->get_parameter("use_sim_time").as_bool() ||
        !env_equals("D1MAX_NAV_ISOLATED","1") || !env_equals("D1MAX_NAV_TRANSPORT","isolated_mock") ||
        !env_equals("RMW_IMPLEMENTATION","rmw_zenoh_cpp") || !mp_.use_projected_rays_)
      throw std::invalid_argument("simulation collision clock requires isolated Isaac projected rays and ROS simulation time");
    mp_.simulation_collision_clock_=true;
  }
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

  std::string collision_evidence_mode;
  load_parameter(node_,"grid_map.collision_evidence_mode",collision_evidence_mode,std::string{});
  if(!collision_evidence_mode.empty()) {
    if(collision_evidence_mode!="validated_static_prior"||!mp_.require_observed_free_||
        !mp_.use_projected_rays_||!mp_.require_localization_context_)
      throw std::invalid_argument("validated static prior requires strict context-tagged projected rays");
    scan_planner::StaticOccupancyPrior::Expected expected;
    std::string path;
    load_parameter(node_,"grid_map.static_prior_manifest_path",path,std::string{});
    load_parameter(node_,"grid_map.static_prior_manifest_sha256",expected.manifest_sha256,std::string{});
    load_parameter(node_,"grid_map.static_prior_geometry_sha256",expected.geometry_sha256,std::string{});
    load_parameter(node_,"grid_map.static_prior_map_version_id",expected.map_version,std::string{});
    load_parameter(node_,"grid_map.static_prior_map_frame",expected.frame_id,std::string{});
    load_parameter(node_,"grid_map.static_prior_transform_contract",expected.transform_contract,std::string{});
    expected.odom_frame_id=mp_.frame_id_;expected.resolution=mp_.resolution_;
    static_prior_=scan_planner::StaticOccupancyPrior::load(path,expected);
    mp_.validated_static_prior_=true;
    // Loading geometry is insufficient authority. A new localization context
    // with the same certified identity must be ACKed before any FREE is used.
    static_prior_context_valid_=false;
  }

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

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    if (mp_.use_projected_rays_) {
      projected_rays_sub_=node_->create_subscription<d1max_planning_interfaces::msg::ProjectedRays>(
          "projected_rays",rclcpp::SensorDataQoS().keep_last(scan_planner::kProjectedRayIngressDepth),
          [this](const d1max_planning_interfaces::msg::ProjectedRays::ConstSharedPtr message) {
            ++ray_ingress_callback_count_;
            acceptProjectedRays(*message,node_->now().nanoseconds(),std::chrono::steady_clock::now());
            drainProjectedIngress();
            tryProjectedFusion(node_->now().nanoseconds(),std::chrono::steady_clock::now(),false);
            publishProjectedRaysStatus(node_->now().nanoseconds(),std::chrono::steady_clock::now());
            scheduleProjectedFusionWake();
          });
      projected_rays_status_pub_=node_->create_publisher<std_msgs::msg::String>(
          "grid_map/projected_rays_status",rclcpp::QoS(1).reliable());
    } else {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
    }
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1));

  if (!node_->has_parameter("grid_map.integration_rate_hz"))
    node_->declare_parameter<double>("grid_map.integration_rate_hz",5.0);
  const double integration_rate=node_->get_parameter("grid_map.integration_rate_hz").as_double();
  if (!std::isfinite(integration_rate) || integration_rate<=0. || integration_rate>20.)
    throw std::invalid_argument("invalid grid map integration rate");
  ray_integration_period_s_=1./integration_rate;
  // The cheap watchdog can notice a just-completed pair at the rate boundary;
  // actual ray integration remains capped by tryProjectedFusion's steady clock.
  occ_timer_ = node_->create_wall_timer(std::chrono::duration<double>(mp_.use_projected_rays_?.02:ray_integration_period_s_),
                                        std::bind(&GridMap::updateOccupancyCallback, this));
  vis_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / mp_.visualization_rate_hz_),
                                        std::bind(&GridMap::visCallback, this));

  if (mp_.require_localization_context_)
  {
    const auto context_qos = rclcpp::QoS(1).reliable().transient_local();
    localization_context_ack_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "grid_map/localization_context_ack", context_qos);
    localization_context_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "grid_map/localization_context", context_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (applyLocalizationContext(msg->data)) localization_context_ack_pub_->publish(*msg);
        });
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
  if (mp_.require_observed_free_) {
    md_.inflate_offsets_=scan_planner::conservativeCylinderInflation(
        mp_.resolution_,mp_.double_cylinder_radius_,
        mp_.obstacles_inflation_z_up,mp_.obstacles_inflation_z_down);
  } else {
    // Upstream ROS 1 348e8a59 / ros2-community d0b921c9 discretization.
    // In particular, XY offsets at exactly the radius are excluded; this is
    // not the larger voxel-AABB kernel retained by the optional strict policy.
    const double radius=std::max(0.0,mp_.double_cylinder_radius_);
    if (!std::isfinite(mp_.resolution_) || mp_.resolution_<=0. ||
        !std::isfinite(mp_.double_cylinder_radius_) ||
        !std::isfinite(mp_.obstacles_inflation_z_up) || mp_.obstacles_inflation_z_up<0. ||
        !std::isfinite(mp_.obstacles_inflation_z_down) || mp_.obstacles_inflation_z_down<0.)
      throw std::invalid_argument("invalid cylinder voxel geometry");
    const double xy=std::ceil(radius/mp_.resolution_);
    const double up=std::ceil(mp_.obstacles_inflation_z_up/mp_.resolution_);
    const double down=std::ceil(mp_.obstacles_inflation_z_down/mp_.resolution_);
    if ((2.*xy+1.)*(2.*xy+1.)*(up+down+1.)>200000.)
      throw std::invalid_argument("cylinder voxel kernel exceeds bounded budget");
    md_.inflate_offsets_.clear();
    for (int x=-static_cast<int>(xy);x<=static_cast<int>(xy);++x)
      for (int y=-static_cast<int>(xy);y<=static_cast<int>(xy);++y) {
        const Eigen::Vector2d offset_xy(x*mp_.resolution_,y*mp_.resolution_);
        if (offset_xy.norm()>=radius) continue;
        for (int z=-static_cast<int>(down);z<=static_cast<int>(up);++z)
          md_.inflate_offsets_.emplace_back(x,y,z);
      }
  }
  observed_cylinder_cache_.clear();
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
  if (&out==this || out.node_) throw std::invalid_argument("snapshot destination must be a distinct memory-only map");
  out.mp_=mp_;
  out.md_.occupancy_buffer_=md_.occupancy_buffer_;
  out.md_.occupancy_buffer_inflate_=md_.occupancy_buffer_inflate_;
  // Incremental inflation counters belong to fusion only. Collision readers
  // use the committed flags/raw evidence and never update the map. Do not
  // allocate/copy four bytes per voxel into each of the three read slots.
  if(!out.md_.occupancy_buffer_inflate_cnt_.empty())
    std::vector<int>().swap(out.md_.occupancy_buffer_inflate_cnt_);
  out.md_.inflate_offsets_=md_.inflate_offsets_;
  out.free_observation_stamps_=free_observation_stamps_;
  out.free_observation_receipts_ns_=free_observation_receipts_ns_;
  out.integrated_cloud_stamp_ns_=integrated_cloud_stamp_ns_;
  out.ray_integrated_stamps_=ray_integrated_stamps_;
  out.fusion_timing_=fusion_timing_;
  out.ray_integrated_receipts_=ray_integrated_receipts_;
  out.localization_context_sequence_=localization_context_sequence_;
  out.localization_epoch_=localization_epoch_;
  out.localization_seed_=localization_seed_;
  out.localization_context_barrier_ns_=localization_context_barrier_ns_;
  out.localization_context_payload_=localization_context_payload_;
  out.static_prior_=static_prior_; // One immutable checked volume across all readers.
  out.static_prior_context_valid_=static_prior_context_valid_;
  out.static_prior_query_lease_valid_=false; // Reader clock/receipt must revalidate its own scope.
  out.static_prior_revoked_sequence_=static_prior_revoked_sequence_;
  out.static_prior_live_hits_=static_prior_live_hits_;
  out.occupancy_revision_=occupancy_revision_;
  out.free_evidence_revision_=free_evidence_revision_;
  out.enforce_free_freshness_=enforce_free_freshness_;
  out.ray_clock_fault_=ray_clock_fault_;
  out.collision_snapshot_=true;
  out.snapshot_clock_ns_=mp_.use_projected_rays_ ? collisionQueryClockAt(source_now_ns,captured) : source_now_ns;
  out.snapshot_captured_=captured;
  out.ray_tick_receipt_=captured;
  out.observed_cylinder_cache_.clear();
  out.prepareRawCollisionCache(); // Retain destination capacity, never copy the writer's decisions.
  out.collision_cache_deadline_ns_=out.collision_cache_clock_ns_=0;
  out.collision_cache_receipt_deadline_ns_=out.collision_cache_receipt_ns_=0;
  out.unknown_collision_queries_=0;
  // No copied callback, ROS clock owner, pending acquisition, visualization or
  // shared mutable cache. Source acquisition stamps above remain unchanged.
}

std::int64_t GridMap::collisionQueryClockAt(std::int64_t source,PairReceipt now) const
{
  if (source<=0 || ray_tick_clock_ns_<=0 || source<ray_tick_clock_ns_) return 0;
  if(mp_.simulation_collision_clock_) return source;
  const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
      now-ray_tick_receipt_).count();
  if (elapsed<0 || elapsed>std::numeric_limits<std::int64_t>::max()-ray_tick_effective_ns_) return 0;
  // A paused source clock cannot preserve old free evidence indefinitely.
  return std::max(source,ray_tick_effective_ns_+elapsed);
}

void GridMap::advanceRayEvidenceClock(std::int64_t source,PairReceipt now,bool age_by_steady)
{
  const auto prior_effective=age_by_steady&&!mp_.simulation_collision_clock_?collisionQueryClockAt(source,now):source;
  if(ray_tick_clock_ns_>0 && source<ray_tick_clock_ns_) {
    ray_clock_fault_=true;observed_cylinder_cache_.clear();
  }
  ray_tick_effective_ns_=std::max(source,prior_effective);
  ray_tick_clock_ns_=source;ray_tick_receipt_=now;ray_query_clock_ns_=source;
}

std::int64_t GridMap::collisionQueryReceiptNs() const
{
  const auto now=(node_||collision_snapshot_)?std::chrono::steady_clock::now():ray_tick_receipt_;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

std::int64_t GridMap::observedProofDeadlineNs() const
{
  if(!mp_.simulation_collision_clock_&&!mp_.validated_static_prior_) return collision_cache_deadline_ns_;
  const auto now=collisionQueryReceiptNs();
  if(now<=0 || now<collision_cache_receipt_ns_ || collision_cache_receipt_deadline_ns_<=now)
    return 1; // Positive, already expired; never an absent/unbounded proof.
  const auto remaining=collision_cache_receipt_deadline_ns_-now;
  if(collision_cache_clock_ns_<=0 || remaining>std::numeric_limits<std::int64_t>::max()-collision_cache_clock_ns_)
    return 1;
  return std::min(collision_cache_deadline_ns_,collision_cache_clock_ns_+remaining);
}

void GridMap::beginCollisionQuery()
{
  if (!mp_.require_observed_free_ || !mp_.use_projected_rays_ || !enforce_free_freshness_) return;
  const auto now=collisionQueryClock();
  const bool dual_clock=mp_.simulation_collision_clock_||mp_.validated_static_prior_;
  const auto receipt=dual_clock?collisionQueryReceiptNs():0;
  if (collision_cache_clock_ns_>0 && (now<=0 || (!node_ && now<collision_cache_clock_ns_)))
    ray_clock_fault_=true;
  if(dual_clock && (receipt<=0 || receipt<collision_cache_receipt_ns_))
    ray_clock_fault_=true;
  if (now<=0 || now<collision_cache_clock_ns_ || now>=collision_cache_deadline_ns_ ||
      (dual_clock && (receipt<=0 || receipt<collision_cache_receipt_ns_ ||
       receipt>=collision_cache_receipt_deadline_ns_))) {
    observed_cylinder_cache_.clear();
    collision_cache_deadline_ns_=std::numeric_limits<std::int64_t>::max();
    collision_cache_receipt_deadline_ns_=std::numeric_limits<std::int64_t>::max();
  }
  collision_cache_clock_ns_=now>0?std::max(now,collision_cache_clock_ns_):now;
  collision_cache_receipt_ns_=receipt;
  if(mp_.validated_static_prior_) {
    static_prior_query_lease_valid_=staticPriorLiveLeaseValid();
    if(!static_prior_query_lease_valid_) {
      observed_cylinder_cache_.clear();
      collision_cache_deadline_ns_=collision_cache_receipt_deadline_ns_=1;
    }
    prepareRawCollisionCache();
  }
}

bool GridMap::staticPriorLiveLeaseValid()
{
  if(!static_prior_||!static_prior_context_valid_||!localization_context_sequence_||
      !localization_epoch_||localization_seed_.empty()||ray_clock_fault_||
      collision_cache_clock_ns_<=0||collision_cache_receipt_ns_<=0||
      !std::isfinite(mp_.cloud_pose_max_age_)||mp_.cloud_pose_max_age_<=0.||mp_.cloud_pose_max_age_>.5)return false;
  const auto limit=static_cast<std::int64_t>(mp_.cloud_pose_max_age_*1e9);
  for(std::size_t sensor=0;sensor<2;++sensor) {
    const auto source=ray_integrated_stamps_[sensor];
    const auto receipt=std::chrono::duration_cast<std::chrono::nanoseconds>(ray_integrated_receipts_[sensor].time_since_epoch()).count();
    if(source<=0||collision_cache_clock_ns_<source||collision_cache_clock_ns_-source>=limit||
        receipt<=0||collision_cache_receipt_ns_<receipt||collision_cache_receipt_ns_-receipt>=limit)return false;
    collision_cache_deadline_ns_=std::min(collision_cache_deadline_ns_,source+limit);
    collision_cache_receipt_deadline_ns_=std::min(collision_cache_receipt_deadline_ns_,receipt+limit);
  }
  return true;
}

void GridMap::revokeStaticPriorContext(std::uint64_t sequence)
{
  if(!mp_.validated_static_prior_)return;
  static_prior_revoked_sequence_=std::max({static_prior_revoked_sequence_,sequence,localization_context_sequence_});
  static_prior_context_valid_=false;
  invalidateCloudPosePairs(static_cast<std::int64_t>(localization_context_barrier_ns_));
  resetAllMapData(); // No old cloud/context can continue to authorize motion.
}

void GridMap::recordStaticPriorHit(const Eigen::Vector3i &cell,std::int64_t stamp)
{
  if(!mp_.validated_static_prior_||!static_prior_||stamp<=0)return;
  const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
  if(static_prior_->state(key)!=0)return; // Only contradicts certified FREE.
  auto found=static_prior_live_hits_.find(key);
  if(found==static_prior_live_hits_.end()) {
    static_prior_live_hits_.emplace(key,stamp);observed_cylinder_cache_.clear();++occupancy_revision_;
  } else if(stamp>found->second)found->second=stamp;
}

void GridMap::clearStaticPriorHitAfterMiss(const Eigen::Vector3i &cell,int address)
{
  if(!mp_.validated_static_prior_||ray_clock_fault_||
      scan_planner::strictRawVoxelStatus(md_.occupancy_buffer_[address],mp_.clamp_min_log_,mp_.min_occupancy_log_)!=0)return;
  const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
  auto found=static_prior_live_hits_.find(key);if(found==static_prior_live_hits_.end())return;
  const auto source=free_observation_stamps_[address];
  const auto source_now=node_?collisionQueryClock():collision_cache_clock_ns_;
  const auto receipt_now=node_?collisionQueryReceiptNs():collision_cache_receipt_ns_;
  if(source<=found->second||source_now<source||
      (source_now-source)*1e-9>=mp_.cloud_pose_max_age_)return;
  if(mp_.simulation_collision_clock_||mp_.validated_static_prior_) {
    const auto receipt=free_observation_receipts_ns_[address];
    if(receipt<=0||receipt_now<receipt||
        (receipt_now-receipt)*1e-9>=mp_.cloud_pose_max_age_)return;
  }
  static_prior_live_hits_.erase(found);observed_cylinder_cache_.clear();++occupancy_revision_;
}

int GridMap::rawCollisionStatus(const Eigen::Vector3i &cell)
{
  if(!isInMap(cell))return -1;
  if(!mp_.validated_static_prior_)return uncachedRawCollisionStatus(cell);
  // All cells of the exact cylinder/swept volume are still visited. Reuse only
  // their raw evidence decision in this map/proof scope. A direct-address
  // int8 cache avoids hashing cylinder position/height for every raw cell.
  // Every original cache clear changes its independent 64-bit identifier,
  // including the 32-bit slot-generation wrap and automatic capacity flush.
  // The first evaluation binds the original source/receipt proof deadlines;
  // neither a hit nor a later lookup can extend that minimum lease.
  const int address=toAddress(cell);
  if(address<0||static_cast<std::size_t>(address)>=md_.occupancy_buffer_.size())return -1;
  if(raw_collision_cache_generation_!=observed_cylinder_cache_.generationIdentifier()||
      raw_collision_cache_.size()!=md_.occupancy_buffer_.size())prepareRawCollisionCache();
  if(!observed_cylinder_cache_.generationReusable()||raw_collision_cache_.empty())
    return uncachedRawCollisionStatus(cell);
  auto &cached=raw_collision_cache_[address];
  if(cached==uncached_raw_status_)cached=static_cast<std::int8_t>(uncachedRawCollisionStatus(cell));
  return cached;
}

void GridMap::prepareRawCollisionCache()
{
  if(!mp_.validated_static_prior_||!observed_cylinder_cache_.generationReusable()||
      md_.occupancy_buffer_.size()>static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    raw_collision_cache_.clear();raw_collision_cache_generation_=0;return;
  }
  const auto generation=observed_cylinder_cache_.generationIdentifier();
  if(raw_collision_cache_.size()!=md_.occupancy_buffer_.size())
    raw_collision_cache_.assign(md_.occupancy_buffer_.size(),uncached_raw_status_);
  else if(raw_collision_cache_generation_!=generation)
    std::fill(raw_collision_cache_.begin(),raw_collision_cache_.end(),uncached_raw_status_);
  raw_collision_cache_generation_=generation;
}

int GridMap::uncachedRawCollisionStatus(const Eigen::Vector3i &cell)
{
  if (!isInMap(cell)) return -1;
  const auto address=toAddress(cell);
  const double odds=md_.occupancy_buffer_[address];
  // Raw diagnostics retain unknown/insufficient evidence under either policy.
  // Official collision queries read inflation directly and never use this
  // status to turn unobserved map contents into measured free cells.
  const int raw=scan_planner::strictRawVoxelStatus(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_);
  const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
  const int prior=mp_.validated_static_prior_&&static_prior_&&static_prior_context_valid_?
      static_prior_->state(key):2;
  if(prior==1||raw==1)return 1; // Static occupied cannot be cleared by live misses.
  if(mp_.validated_static_prior_) {
    if(!static_prior_query_lease_valid_)return 2;
    if(static_prior_live_hits_.count(key))return 2;
    if(raw==2) {
      const auto category=scan_planner::diagnoseRawVoxel(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_,mp_.unknown_flag_);
      return prior==0&&category==scan_planner::RawVoxelDiagnostic::NeverObserved?0:2;
    }
  }
  if (raw!=0 || !mp_.require_observed_free_ || !mp_.use_projected_rays_ || !enforce_free_freshness_) return raw;
  if (ray_clock_fault_) return 2;
  const auto stamp=static_cast<std::size_t>(address)<free_observation_stamps_.size()?
      free_observation_stamps_[address]:0;
  const auto now=collision_cache_clock_ns_;
  // No future tolerance for FREE evidence, no expiry of OCCUPIED to free.
  if (stamp<=0 || now<stamp || !std::isfinite(mp_.cloud_pose_max_age_) ||
      mp_.cloud_pose_max_age_<=0. || mp_.cloud_pose_max_age_>.5) return 2;
  const auto age_limit=static_cast<std::int64_t>(mp_.cloud_pose_max_age_*1e9);
  // A zero/future/malformed witness is not expired evidence. The certified
  // prior may replace only real past evidence whose live lease has elapsed.
  const bool source_expired=now-stamp>=age_limit;
  bool receipt_expired=false;
  std::int64_t voxel_receipt_ns=0;
  if(mp_.simulation_collision_clock_||mp_.validated_static_prior_) {
    const auto receipt=static_cast<std::size_t>(address)<free_observation_receipts_ns_.size()?
        free_observation_receipts_ns_[address]:0;
    const auto receipt_now=collision_cache_receipt_ns_;
    if(receipt<=0 || receipt_now<receipt) return 2;
    voxel_receipt_ns=receipt;
    receipt_expired=receipt_now-receipt>=age_limit;
  }
  // A valid independent static FREE certificate already has its own finite
  // both-sensor source/receipt lease. Prefer it to a redundant, shorter live
  // FREE lease even just before that old observation expires. Invalid live
  // witnesses above, weak evidence and true-hit vetoes still fail closed.
  // No voxel observation or certificate deadline is rewritten or renewed.
  if(prior==0)return 0;
  if(!source_expired&&!receipt_expired&&voxel_receipt_ns>0&&
      voxel_receipt_ns<=std::numeric_limits<std::int64_t>::max()-age_limit)
    collision_cache_receipt_deadline_ns_=std::min(collision_cache_receipt_deadline_ns_,voxel_receipt_ns+age_limit);
  if(source_expired||receipt_expired) {
    // A rejected live certificate still leaves an explicitly expired proof
    // deadline. Preserve the default dual-clock contract instead of turning
    // UNKNOWN into an apparent unlimited/absent evidence deadline.
    if(stamp<=std::numeric_limits<std::int64_t>::max()-age_limit)
      collision_cache_deadline_ns_=std::min(collision_cache_deadline_ns_,stamp+age_limit);
    if(voxel_receipt_ns>0&&voxel_receipt_ns<=std::numeric_limits<std::int64_t>::max()-age_limit)
      collision_cache_receipt_deadline_ns_=std::min(collision_cache_receipt_deadline_ns_,voxel_receipt_ns+age_limit);
    return 2;
  }
  if (stamp<=std::numeric_limits<std::int64_t>::max()-age_limit)
    collision_cache_deadline_ns_=std::min(collision_cache_deadline_ns_,stamp+age_limit);
  return 0;
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

scan_planner::CollisionEvidence GridMap::inspectInflateOccupancy(const Eigen::Vector3d &position,double yaw,bool detailed)
{
  // Rate-limited raw-evidence diagnostic, not the official collision decision.
  // Reverse the active inflation kernel; strict mode further intersects the
  // actual cylinder AABB. Counts include overlapping cells, and unknown raw
  // cells remain unknown even when the official inflated-buffer query is zero.
  scan_planner::CollisionEvidence evidence;
  beginCollisionQuery();
  std::map<std::array<int,3>,std::size_t> unique;
  if (!position.allFinite() || !std::isfinite(yaw)) {evidence.counts[3]=1;return evidence;}
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
      int state=-1;
      double odds=std::numeric_limits<double>::quiet_NaN();
      if (isInMap(cell)) {
        odds=md_.occupancy_buffer_[toAddress(cell)];
        state=rawCollisionStatus(cell);
      }
      const int slot=state<0 ? 3:state;
      if (slot && !evidence.counts[slot]) evidence.first[slot-1]=cell;
      ++evidence.counts[slot];
      if(detailed) {
        const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
        auto found=unique.find(key);
        if(found==unique.end()) {
          scan_planner::CollisionVoxelDiagnostic voxel;
          voxel.index=cell;voxel.log_odds=odds;voxel.native_state=state;
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
        evidence.voxels[found->second].cylinder_mask|=side<0 ? 1U:2U;
      }
    }
  }
  return evidence;
}

std::string GridMap::describeInflateOccupancy(const Eigen::Vector3d &position,double yaw)
{
  if (!position.allFinite() || !std::isfinite(yaw)) return "nonfinite_query";
  const auto evidence=inspectInflateOccupancy(position,yaw,true);
  const auto &counts=evidence.counts;
  std::ostringstream out;out.precision(4);
  out<<"query_policy="<<collisionQueryPolicy();
  if (!mp_.require_observed_free_) out<<" collision="<<getInflateOccupancy(position,yaw);
  out<<" evidence_counts="<<(mp_.require_observed_free_?"strict_with_free_expiry":"raw_evidence")
     <<" free="<<counts[0]<<" occupied="<<counts[1]<<" unknown="<<counts[2]<<" outside="<<counts[3]
     <<" raw_overlap=["<<counts[0]<<","<<counts[1]<<","<<counts[2]<<","<<counts[3]<<"]"
     <<" unique_voxels=["<<evidence.unique_counts[0]<<","<<evidence.unique_counts[1]<<","<<evidence.unique_counts[2]<<","<<evidence.unique_counts[3]<<"]";
  const auto body=bodyEnvelope();
  out<<" body_z=["<<position.z()-body.below<<","<<position.z()+body.above<<"]"
     <<" never_observed="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::NeverObserved)]
     <<" observed_insufficient="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::Insufficient)]
     <<" stale_free="<<evidence.classification_counts[static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::StaleFree)]
     <<" source_query_ns="<<collision_cache_clock_ns_
     <<" front_source_ns="<<integratedRaySourceStamp(0)<<" rear_source_ns="<<integratedRaySourceStamp(1)
     <<" front_age_ms="<<(collision_cache_clock_ns_-integratedRaySourceStamp(0))*1e-6
     <<" rear_age_ms="<<(collision_cache_clock_ns_-integratedRaySourceStamp(1))*1e-6;
  if(mp_.validated_static_prior_) {
    std::map<std::string,std::size_t> sources;
    for(const auto &voxel:evidence.voxels)++sources[collisionEvidenceSource(voxel.index,voxel.native_state)];
    out<<" prior_context_valid="<<static_prior_context_valid_
       <<" certified_static_free="<<sources["certified_static_free"]
       <<" certified_static_occupied="<<sources["certified_static_occupied"]
       <<" live_observed_free="<<sources["live_observed_free"]
       <<" live_static_conflict="<<sources["live_static_conflict"]
       <<" prior_map_version="<<(static_prior_?static_prior_->mapVersion():"unavailable")
       <<" prior_manifest_sha256="<<(static_prior_?static_prior_->manifestSha256():"unavailable");
  }
  std::int64_t oldest_free=std::numeric_limits<std::int64_t>::max();
  for(const auto& voxel:evidence.voxels)if(voxel.free_observation_stamp_ns>0)
    oldest_free=std::min(oldest_free,voxel.free_observation_stamp_ns);
  if(oldest_free!=std::numeric_limits<std::int64_t>::max())
    out<<" oldest_free_source_ns="<<oldest_free<<" oldest_free_age_ms="<<(collision_cache_clock_ns_-oldest_free)*1e-6;
  const char *labels[]={" occupied_cell="," unknown_cell="," outside_cell="};
  for (int i=0;i<3;++i) if(counts[i+1]) {
    const Eigen::Vector3d point=(evidence.first[i].cast<double>().array()+.5)*mp_.resolution_;
    out<<labels[i]<<"("<<point.x()<<","<<point.y()<<","<<point.z()<<")";
    if(i<2) {
      const auto &cell=evidence.first[i];
      const auto address=toAddress(cell);
      const auto odds=md_.occupancy_buffer_[address];
      const auto category=scan_planner::diagnoseRawVoxel(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_,mp_.unknown_flag_);
      const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
      const auto hit=static_prior_live_hits_.find(key);
      const auto source=static_cast<std::size_t>(address)<free_observation_stamps_.size()?free_observation_stamps_[address]:0;
      const auto receipt=static_cast<std::size_t>(address)<free_observation_receipts_ns_.size()?free_observation_receipts_ns_[address]:0;
      out<<" first_cell_evidence={index:["<<cell.x()<<","<<cell.y()<<","<<cell.z()<<"]"
         <<",raw_log_odds:"<<odds<<",raw_category:"<<scan_planner::diagnosticName(category)
         <<",raw_state:"<<scan_planner::strictRawVoxelStatus(odds,mp_.clamp_min_log_,mp_.min_occupancy_log_)
         <<",prior_state:"<<(static_prior_&&static_prior_context_valid_?static_prior_->state(key):2)
         <<",hit_source_ns:"<<(hit!=static_prior_live_hits_.end()?hit->second:0)
         <<",free_source_ns:"<<source<<",free_receipt_ns:"<<receipt<<"}";
      if(!near_field_diagnostics_.enabled())out<<" ray_witness=disabled";
      else if(!near_field_diagnostics_.contains(evidence.first[i]))out<<" ray_witness=outside_diagnostic_roi";
      else if(const auto* witnesses=near_field_diagnostics_.find(evidence.first[i])) {
        for(const auto& witness:witnesses->last)if(witness) {
          const auto& w=*witness;const auto& m=w.metadata;
          out<<" ray_witness={sensor:"<<m.sensor_id<<",hit:"<<w.hit<<",vote:"<<w.contributed_vote
             <<",scan_ns:"<<m.scan_stamp_ns<<",acquisition_end_ns:"<<m.acquisition_end_ns
             <<",source_age_ms:"<<(collision_cache_clock_ns_-m.acquisition_end_ns)*1e-6
             <<",alignment_ns:"<<m.alignment_stamp_ns<<",received_ns:"<<m.received_ns
             <<",source_index:"<<m.source_index<<",offset_ns:"<<m.offset_time_ns
             <<",source_fields_available:"<<m.source_fields_available
             <<",origin:["<<w.origin.x()<<","<<w.origin.y()<<","<<w.origin.z()<<"]"
             <<",endpoint:["<<w.endpoint.x()<<","<<w.endpoint.y()<<","<<w.endpoint.z()<<"]}";
        }
      } else out<<" ray_witness=not_retained";
    }
  }
  return out.str();
}

const char *GridMap::collisionEvidenceSource(const Eigen::Vector3i &cell,int state)
{
  if(!isInMap(cell))return "outside";
  const std::array<int,3> key{{cell.x(),cell.y(),cell.z()}};
  if(static_prior_&&static_prior_context_valid_&&static_prior_->state(key)==1)return "certified_static_occupied";
  if(static_prior_live_hits_.count(key))return "live_static_conflict";
  if(state!=0)return state==1?"live_occupied":"unknown";
  if(mp_.validated_static_prior_&&static_prior_&&static_prior_context_valid_&&static_prior_->state(key)==0)
    return "certified_static_free"; // Actual independently selected certificate, including fresh raw FREE.
  const auto address=toAddress(cell);
  const auto category=scan_planner::diagnoseRawVoxel(md_.occupancy_buffer_[address],mp_.clamp_min_log_,mp_.min_occupancy_log_,mp_.unknown_flag_);
  if(category==scan_planner::RawVoxelDiagnostic::NeverObserved)return "certified_static_free";
  const auto source=free_observation_stamps_[address];
  if(source>0&&(collision_cache_clock_ns_-source)*1e-9>=mp_.cloud_pose_max_age_)return "certified_static_free";
  if((mp_.simulation_collision_clock_||mp_.validated_static_prior_)&&static_cast<std::size_t>(address)<free_observation_receipts_ns_.size()&&
      (collision_cache_receipt_ns_-free_observation_receipts_ns_[address])*1e-9>=mp_.cloud_pose_max_age_)return "certified_static_free";
  return "live_observed_free";
}

std::string GridMap::describeCollisionLease() const
{
  // Read the failed query's captured lease, before any later diagnostic query.
  // No live-map access, clock refresh, proof retry or deadline mutation.
  std::array<std::int64_t,2> receipts;
  for(std::size_t sensor=0;sensor<2;++sensor)
    receipts[sensor]=std::chrono::duration_cast<std::chrono::nanoseconds>(
        ray_integrated_receipts_[sensor].time_since_epoch()).count();
  return nlohmann::json{{"kind","captured_collision_query_lease"},
      {"query_source_ns",collision_cache_clock_ns_},{"query_receipt_ns",collision_cache_receipt_ns_},
      {"source_deadline_ns",collision_cache_deadline_ns_},{"receipt_deadline_ns",collision_cache_receipt_deadline_ns_},
      {"both_ray_source_ns",ray_integrated_stamps_},{"both_ray_receipt_ns",receipts},
      {"static_prior_loaded",!!static_prior_},{"static_prior_context_valid",static_prior_context_valid_},
      {"static_prior_manifest_sha256",static_prior_?static_prior_->manifestSha256():""},
      {"static_prior_geometry_sha256",static_prior_?static_prior_->geometrySha256():""},
      {"static_prior_map_version",static_prior_?static_prior_->mapVersion():""},
      {"cached_prior_lease_valid",static_prior_query_lease_valid_},{"ray_clock_fault",ray_clock_fault_},
      {"session_id",mp_.localization_session_id_},{"epoch",localization_epoch_},
      {"context_sequence",localization_context_sequence_},{"seed_id",localization_seed_},
      {"barrier_ns",localization_context_barrier_ns_},{"policy",collisionQueryPolicy()}}.dump();
}

void GridMap::resetAllMapData()
{
  static_prior_query_lease_valid_=false;
  near_field_diagnostics_.clear();
  observed_cylinder_cache_.clear();
  prepareRawCollisionCache();
  collision_cache_deadline_ns_=collision_cache_clock_ns_=0;
  collision_cache_receipt_deadline_ns_=collision_cache_receipt_ns_=0;
  if(mp_.use_projected_rays_) free_observation_stamps_.assign(md_.occupancy_buffer_.size(),0);
  else free_observation_stamps_.clear();
  if(mp_.simulation_collision_clock_||mp_.validated_static_prior_) free_observation_receipts_ns_.assign(md_.occupancy_buffer_.size(),0);
  else free_observation_receipts_ns_.clear();
  ++occupancy_revision_;
  // A reset map has no current measurement support until raycastProcess()
  // actually integrates the replacement cloud; an old fresh stamp is not
  // evidence for the newly empty buffer.
  integrated_cloud_stamp_ns_ = 0;
  ray_integrated_stamps_.fill(0);
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
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
  auto received_sequence=localization_context_sequence_;
  const auto reject=[this,&received_sequence]() {revokeStaticPriorContext(received_sequence);return false;};
  if (payload.empty() || payload.size() > 4096) return reject();
  try {
    const auto value = nlohmann::json::parse(payload);
    // A malformed new attestation for this session still establishes a revoke
    // barrier. Replaying an earlier reliable/latched ACK cannot re-arm it.
    if(mp_.validated_static_prior_&&value.value("schema",0)==1&&
        value.value("session_id",std::string{})==mp_.localization_session_id_&&
        value.contains("sequence")&&value.at("sequence").is_number_unsigned())
      received_sequence=value.at("sequence").get<std::uint64_t>();
    if (value.at("schema") != 1 ||
        value.at("session_id").get<std::string>() != mp_.localization_session_id_ ||
        !value.at("epoch").is_number_unsigned() || !value.at("sequence").is_number_unsigned() ||
        !value.at("barrier_ns").is_number_unsigned()) return reject();
    if(mp_.validated_static_prior_&&(!static_prior_||!static_prior_->matchesContext(value)))return reject();
    const auto epoch = value.at("epoch").get<std::uint64_t>();
    const auto sequence = value.at("sequence").get<std::uint64_t>();
    const auto barrier = value.at("barrier_ns").get<std::uint64_t>();
    const auto seed = value.at("seed_id").get<std::string>();
    if(mp_.validated_static_prior_&&static_prior_revoked_sequence_&&sequence<=static_prior_revoked_sequence_)
      return reject();
    if (!epoch || !sequence || seed.empty() || seed.size() > 256 || !barrier ||
        barrier > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
        epoch < localization_epoch_ || sequence < localization_context_sequence_) return reject();
    if (sequence == localization_context_sequence_) {
      if(payload!=localization_context_payload_)return reject();
      if(mp_.validated_static_prior_)static_prior_context_valid_=true;
      return true; // Reliable retry, never reset twice or renew a live lease.
    }
    invalidateCloudPosePairs(static_cast<std::int64_t>(barrier));
    // A new map-context sequence is an explicit evidence rebuild, separate
    // from goal/reference generation. ACK must never preserve the old grid.
    resetAllMapData();
    // The checked transform is fixed identity for this immutable map version.
    // Neither seed/epoch nor sequence changes the world indices; a context
    // rebuild therefore cannot erase an actual obstacle contradiction.
    static_prior_context_valid_=mp_.validated_static_prior_;
    ray_clock_fault_=false;ray_query_clock_ns_=ray_tick_clock_ns_=ray_tick_effective_ns_=0;
    localization_epoch_ = epoch;
    localization_seed_ = seed;
    localization_context_sequence_ = sequence;
    localization_context_barrier_ns_ = barrier;
    ray_integrations_.fill(0);ray_drops_.fill(0);ray_unattributed_drops_=0;
    localization_context_payload_ = payload;
    return true;
  } catch (const nlohmann::json::exception &) {
    return reject();
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

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  // Inflation may touch thousands of cells for each occupied-state change.
  // Their window is fixed and bounds are checked below: cache ring constants
  // once, rather than issue three integer divisions for every kernel cell.
  const scan_planner::ObservedRayMapIndex index(mp_.map_bound_min_idx_,mp_.map_voxel_num_);
  for (const auto& offset : offsets)
  {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = index.address(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  applyOccupancyUpdateAtIndex(id,addr,new_log_odds);
}

void GridMap::applyOccupancyUpdateAtIndex(const Eigen::Vector3i& id,int addr,double new_log_odds)
{
  // Integration's cache queue already owns a bounds-checked cell and its ring
  // address in the current window. No sliding/reset happens while draining it.
  // Source stamps, hit/miss counters and witnesses have already been updated
  // by the complete-ray walk, outside this log-odds-only operation.
  const double old_log_odds=md_.occupancy_buffer_[addr];
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
    observed_cylinder_cache_.clear();
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
  observed_cylinder_cache_.clear();
  if(static_cast<std::size_t>(addr)<free_observation_stamps_.size()) free_observation_stamps_[addr]=0;
  if(static_cast<std::size_t>(addr)<free_observation_receipts_ns_.size()) free_observation_receipts_ns_[addr]=0;
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
  observed_cylinder_cache_.clear();
  if(static_cast<std::size_t>(addr)<free_observation_stamps_.size()) free_observation_stamps_[addr]=0;
  if(static_cast<std::size_t>(addr)<free_observation_receipts_ns_.size()) free_observation_receipts_ns_[addr]=0;
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

bool GridMap::raycastProcess()
{
  const auto ray_started=std::chrono::steady_clock::now();
  const bool complete_rays=mp_.use_projected_rays_ || mp_.require_observed_free_;
  free_evidence_recovered_=false;
  observed_cylinder_cache_.clear();
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return false;

  updateSlidingMap(md_.ray_pos_);
  const scan_planner::ObservedRayMapIndex ray_index(mp_.map_bound_min_idx_,mp_.map_voxel_num_);
  // A later asynchronous miss must not borrow a newer witness from an earlier
  // hit/miss conflict. Only an actual source-stamp increase in THIS completed
  // batch is eligible to clear an existing contradiction. This sparse copy is
  // bounded by real conflicts, not by the number of rays or map voxels.
  std::map<int,std::int64_t> prior_free_before_batch;
  if(mp_.validated_static_prior_)for(const auto &hit:static_prior_live_hits_) {
    const Eigen::Vector3i cell(hit.first[0],hit.first[1],hit.first[2]);
    if(isInMap(cell)) {
      const auto address=ray_index.address(cell);
      prior_free_before_batch.emplace(address,free_observation_stamps_[address]);
    }
  }
  if(near_field_diagnostics_.enabled()) near_field_diagnostics_.beginIntegration();

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

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];
    const bool has_ray_stamp=mp_.use_projected_rays_ &&
        static_cast<std::size_t>(i)<projected_ray_stamps_.size();
    const auto ray_source_stamp=has_ray_stamp ? projected_ray_stamps_[i] : std::int64_t{0};
    const auto ray_receipt_ns=(mp_.simulation_collision_clock_||mp_.validated_static_prior_) &&
        static_cast<std::size_t>(i)<projected_ray_receipts_ns_.size()?projected_ray_receipts_ns_[i]:0;
    const bool record_ray_diagnostics=near_field_diagnostics_.enabled() &&
        static_cast<std::size_t>(i)<projected_diagnostics_.size();
    const Eigen::Vector3d &origin=mp_.use_projected_rays_ ? md_.proj_origins_[i] : md_.ray_pos_;
    if (mp_.use_projected_rays_ && !isInMap(origin)) { complete=false;break; }
    bool endpoint_is_hit=true;
    if (!scan_planner::clipObservedRay(origin,mp_.local_update_range_,pt_w,endpoint_is_hit))
      continue;

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      endpoint_is_hit=false;
      pt_w = closetPointInMap(pt_w, origin);

      length = (pt_w - origin).norm();
      if (length > mp_.max_ray_length_)
      {
        endpoint_is_hit=false;
        pt_w = (pt_w - origin) / length * mp_.max_ray_length_ + origin;
      }
      vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - origin).norm();

      if (length > mp_.max_ray_length_)
      {
        endpoint_is_hit=false;
        pt_w = (pt_w - origin) / length * mp_.max_ray_length_ + origin;
        vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, 0);
      }
      else
      {
        if (mp_.use_projected_rays_ && endpoint_is_hit) {
          Eigen::Vector3i cell;posToIndex(pt_w,cell);vox_idx=ray_index.address(cell);
          recordStaticPriorHit(cell,ray_source_stamp);
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
        } else vox_idx = mp_.use_projected_rays_ ? INVALID_IDX : setCacheOccupancy(pt_w, endpoint_is_hit ? 1 : 0);
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
      // Sharing an endpoint/crossed voxel does not mean the remaining rays
      // coincide. Complete real rays, but give each free cell only one vote.
      complete=scan_planner::visitObservedRayIndexed(origin,pt_w,mp_.resolution_,
          !endpoint_is_hit,observed_ray_budget,mp_.map_voxel_num_,
          [this,i,&origin,&pt_w,has_ray_stamp,ray_source_stamp,ray_receipt_ns,record_ray_diagnostics](const Eigen::Vector3i &cell,int address) {
            if (!isInMap(cell)) return;
            if(has_ray_stamp &&
                static_cast<std::size_t>(address)<free_observation_stamps_.size()) {
              auto &old=free_observation_stamps_[address];
              const auto incoming=ray_source_stamp;
              // Thousands of rays in one scan share the same source begin.
              // Avoid repeated floating-point checks and writes for them, but
              // retain a genuinely newer traversal from the other sensor.
              if(incoming>old) {
                const auto fresh=[this](std::int64_t stamp,std::int64_t receipt) {
                  const bool source_fresh=stamp>0 && collision_cache_clock_ns_>=stamp &&
                      (collision_cache_clock_ns_-stamp)*1e-9<mp_.cloud_pose_max_age_;
                  return source_fresh && (!(mp_.simulation_collision_clock_||mp_.validated_static_prior_) ||
                      (receipt>0 && collision_cache_receipt_ns_>=receipt &&
                       (collision_cache_receipt_ns_-receipt)*1e-9<mp_.cloud_pose_max_age_));
                };
                const auto old_receipt=static_cast<std::size_t>(address)<free_observation_receipts_ns_.size()?
                    free_observation_receipts_ns_[address]:0;
                if(mp_.require_observed_free_ && enforce_free_freshness_ && !ray_clock_fault_ &&
                    !fresh(old,old_receipt) && fresh(incoming,ray_receipt_ns) &&
                    scan_planner::strictRawVoxelStatus(md_.occupancy_buffer_[address],
                        mp_.clamp_min_log_,mp_.min_occupancy_log_)==0)
                  free_evidence_recovered_=true;
                old=incoming;
                if(static_cast<std::size_t>(address)<free_observation_receipts_ns_.size())
                  free_observation_receipts_ns_[address]=ray_receipt_ns;
              }
            }
            if(record_ray_diagnostics) {
              scan_planner::RayWitness witness;
              witness.metadata=projected_diagnostics_[i];witness.origin=origin;
              witness.endpoint=md_.proj_points_[i];witness.integrated_endpoint=pt_w;
              witness.contributed_vote=md_.flag_traverse_[address]!=md_.raycast_num_;
              near_field_diagnostics_.record(cell,witness);
            }
            if (md_.flag_traverse_[address]==md_.raycast_num_) return;
            md_.flag_traverse_[address]=md_.raycast_num_;
            cacheOccupancyAtIndex(cell,address,0);
          });
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

  std::vector<std::pair<Eigen::Vector3i,int>> prior_clear_candidates;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    const int idx_ctns = ray_index.address(idx);
    md_.cache_voxel_.pop();

    const bool batch_has_hit=md_.count_hit_[idx_ctns]>0;
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
    if(mp_.validated_static_prior_&&!batch_has_hit)prior_clear_candidates.emplace_back(idx,idx_ctns);

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdateAtIndex(idx, idx_ctns, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      // Never turn an out-of-window cell into observed free merely because
      // this frame's integration window does not include it.
      continue;
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdateAtIndex(idx, idx_ctns, new_log_odds);
  }
  // A partial ray traversal must not clear any contradiction. This collection
  // is formed only by actual miss votes from the present acquisition batch.
  if(complete)for(const auto &candidate:prior_clear_candidates) {
    const auto old=prior_free_before_batch.find(candidate.second);
    if(old!=prior_free_before_batch.end()&&free_observation_stamps_[candidate.second]>old->second)
      clearStaticPriorHitAfterMiss(candidate.first,candidate.second);
  }
  if (!complete && node_)
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[GridMap] observed-ray traversal incomplete; this scan cannot authorize planning");
  if (complete_rays && node_)
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[GridMap] complete ray integration: query_policy=%s points=%d steps=%zu duration_ms=%.3f complete=%d",
      collisionQueryPolicy(),md_.proj_points_cnt,16000000-observed_ray_budget,
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-ray_started).count(),
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

  publishMap();
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
}

void GridMap::updateOccupancyCallback()
{
  if (mp_.use_projected_rays_) {
    drainProjectedIngress();
    const auto now=std::chrono::steady_clock::now();
    const auto source_now=node_->now().nanoseconds();
    tryProjectedFusion(source_now,now,true);
    publishProjectedRaysStatus(node_->now().nanoseconds(),std::chrono::steady_clock::now());
    scheduleProjectedFusionWake();
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

void GridMap::drainProjectedIngress()
{
  // Subscription and watchdog share the default mutually-exclusive writer
  // group even when the node's control subscriptions run on another thread.
  ray_ingress_last_drain_={};
  if(!projected_rays_sub_)return;
  rclcpp::MessageInfo info;
  ray_ingress_last_drain_=scan_planner::drainProjectedRayIngress<d1max_planning_interfaces::msg::ProjectedRays>(
    [this,&info](auto& message){return projected_rays_sub_->take(message,info);},
    [this](const auto& message){return acceptProjectedRays(message,node_->now().nanoseconds(),std::chrono::steady_clock::now());});
  ray_ingress_drain_taken_count_+=ray_ingress_last_drain_.taken;
  ray_ingress_drain_accepted_count_+=ray_ingress_last_drain_.accepted;
}

bool GridMap::tryProjectedFusion(std::int64_t now_ns,std::chrono::steady_clock::time_point now,bool watchdog)
{
  const double elapsed=ray_fusion_started_?std::chrono::duration<double>(now-ray_fusion_last_start_).count():
    std::numeric_limits<double>::infinity();
  if(!std::isfinite(ray_integration_period_s_)||ray_integration_period_s_<=0.||elapsed<ray_integration_period_s_)return false;
  if(!pending_rays_[0]&&!pending_rays_[1])return false; // No clock/lease renewal without new observations.
  std::int64_t pair=0;
  if(pending_rays_[0]&&pending_rays_[1])pair=std::min(pending_rays_[0]->batch.stamp_ns,pending_rays_[1]->batch.stamp_ns);
  // Rate limiting belongs to the steady integration clock above. Requiring
  // another full period of *acquisition* advance here introduces a second
  // unsynchronised throttle: a fresh complete 10 Hz pair can miss the 5 Hz
  // boundary and wait another scan. Both sources must instead be genuinely
  // newer than their own last integrated observations. Their original stamps
  // and the 500 ms evidence deadlines are never extended by this scheduling.
  const bool pair_ready=pair>0 &&
      pending_rays_[0]->batch.stamp_ns>ray_integrated_stamps_[0] &&
      pending_rays_[1]->batch.stamp_ns>ray_integrated_stamps_[1];
  // Missing one sensor must still allow its peer to add OCCUPIED evidence;
  // the absent sensor's acquisition stamp remains old and will block motion.
  if(!pair_ready&&(!watchdog||elapsed<ray_integration_period_s_*1.5))return false;
  ray_fusion_started_=true;ray_fusion_last_start_=now;
  if(pair>0)ray_fusion_last_pair_stamp_=pair;
  processProjectedRays(now_ns,now);return true;
}

std::chrono::nanoseconds GridMap::projectedFusionWakeDelay(std::chrono::steady_clock::time_point now) const
{
  // This is a wake-up deadline, never an integration permission. The actual
  // start-to-start cap remains in tryProjectedFusion, including after a late
  // callback. No absolute-phase catch-up can create a <200 ms integration.
  constexpr std::int64_t maximum=20000000,minimum=1000000;
  if(!ray_fusion_started_||!std::isfinite(ray_integration_period_s_)||
     ray_integration_period_s_<=0.||now<ray_fusion_last_start_)return std::chrono::nanoseconds(maximum);
  const auto period=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ray_integration_period_s_));
  const auto remaining=std::chrono::duration_cast<std::chrono::nanoseconds>(ray_fusion_last_start_+period-now).count();
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
  auto batch=scan_planner::decodeProjectedRays(message.rays,mp_.frame_id_,near_field_diagnostics_.enabled());
  if (!batch) { ++ray_unattributed_drops_;return false; }
  const auto sensor=batch->sensor_id;
  for(auto &meta:batch->diagnostics) {
    meta.context_sequence=message.context_sequence;meta.projection_sequence=message.projection_sequence;
    meta.received_ns=now_ns;meta.alignment_stamp_ns=alignment;meta.acquisition_end_ns=end;
  }
  if (begin<=ray_received_stamps_[sensor] || message.projection_sequence<=ray_projection_sequences_[sensor]) {
    ++ray_drops_[sensor];return false;
  }
  // One bounded latest acquisition per physical source, not one fused queue
  // where the faster front callback can continually overwrite the rear.
  if (pending_rays_[sensor]) ++ray_drops_[sensor];
  ray_received_stamps_[sensor]=begin;ray_projection_sequences_[sensor]=message.projection_sequence;
  ray_accept_clock_ns_[sensor]=now_ns;
  pending_rays_[sensor]=PendingRays{std::move(*batch),received};
  return true;
}

void GridMap::processProjectedRays(std::int64_t now_ns,std::chrono::steady_clock::time_point now)
{
  if (!mp_.use_projected_rays_) return;
  ++fusion_timing_.sequence;
  fusion_timing_.begin_ns=now_ns;
  fusion_timing_.begin_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  // Clock progress expires free queries but never changes occupancy odds.
  advanceRayEvidenceClock(now_ns,now,node_!=nullptr);
  beginCollisionQuery();
  md_.proj_points_.clear();md_.proj_origins_.clear();md_.proj_points_cnt=0;
  projected_ray_stamps_.clear();
  projected_ray_receipts_ns_.clear();
  projected_diagnostics_.clear();
  std::array<std::int64_t,2> included{{0,0}};
  std::array<PairReceipt,2> receipts;
  Eigen::Vector3d roi_center=Eigen::Vector3d::Zero();
  unsigned sources=0;
  for (std::size_t sensor=0;sensor<2;++sensor) {
    auto &pending=pending_rays_[sensor];
    if (!pending) continue;
    const double receipt_age=std::chrono::duration<double>(now-pending->received).count();
    if (pending->batch.stamp_ns<=pair_barrier_ns_ || receipt_age<0. || receipt_age>mp_.cloud_pose_pair_wait_ ||
        !scan_planner::rayStampFresh(pending->batch.stamp_ns,now_ns,mp_.cloud_pose_max_age_)) {
      ++ray_drops_[sensor];pending.reset();continue;
    }
    const auto &batch=pending->batch;
    ray_integration_start_ns_[sensor]=now_ns;
    ray_integration_start_stamps_[sensor]=batch.stamp_ns;
    ray_pending_wait_s_[sensor]=receipt_age;
    // This representative is ONLY a sliding-window centre. It never replaces
    // a measured ray origin, and supplies no synthetic free evidence.
    roi_center+=batch.origins.front();++sources;
    included[sensor]=batch.stamp_ns;receipts[sensor]=pending->received;
    md_.proj_points_.insert(md_.proj_points_.end(),batch.endpoints.begin(),batch.endpoints.end());
    md_.proj_origins_.insert(md_.proj_origins_.end(),batch.origins.begin(),batch.origins.end());
    projected_ray_stamps_.insert(projected_ray_stamps_.end(),batch.endpoints.size(),batch.stamp_ns);
    if(mp_.simulation_collision_clock_||mp_.validated_static_prior_) {
      const auto receipt_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(pending->received.time_since_epoch()).count();
      projected_ray_receipts_ns_.insert(projected_ray_receipts_ns_.end(),batch.endpoints.size(),receipt_ns);
    }
    if(near_field_diagnostics_.enabled()) {
      for(auto meta:batch.diagnostics) {meta.integration_ns=now_ns;projected_diagnostics_.push_back(meta);}
    }
    pending.reset();
  }
  if (sources) {
    md_.ray_pos_=roi_center/static_cast<double>(sources);md_.ray_q_=Eigen::Quaterniond::Identity();
    md_.has_cloud_=md_.has_ray_pose_=true;md_.proj_points_cnt=static_cast<int>(md_.proj_points_.size());
    const bool complete=raycastProcess();
    if(!complete) {
      // Incomplete batches cannot lend fresh regional leases to a later
      // successful remote batch. Keep obstacle odds; revoke free certificates
      // until genuine reobservation. Rare failure path, O(N).
      std::fill(free_observation_stamps_.begin(),free_observation_stamps_.end(),0);
      std::fill(free_observation_receipts_ns_.begin(),free_observation_receipts_ns_.end(),0);
      observed_cylinder_cache_.clear();
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
  // Snapshot publication belongs to this transaction, not an unrelated 5 Hz
  // timer that can copy the preceding frame just before fusion finishes.
  if(collision_update_callback_)collision_update_callback_();
}

void GridMap::publishProjectedRaysStatus(std::int64_t now_ns,std::chrono::steady_clock::time_point now)
{
  if (!projected_rays_status_pub_) return;
  const bool valid=integrated_cloud_stamp_ns_>0 &&
      scan_planner::rayStampFresh(integrated_cloud_stamp_ns_,now_ns,mp_.cloud_pose_max_age_);
  if (!ray_status_schedule_.due(now,integrated_cloud_stamp_ns_,valid,localization_context_sequence_)) return;
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
  // Receipt freshness and measured source stamps use the node's ROS clock.
  // Keep wall time separately for diagnostics when use_sim_time is enabled.
  result.data=nlohmann::json{{"schema",1},{"session_id",mp_.localization_session_id_},
    {"epoch",localization_epoch_},{"seed_id",localization_seed_},{"sequence",localization_context_sequence_},
    {"barrier_ns",localization_context_barrier_ns_},{"received_at_unix",now_ns*1e-9},
    {"callback_wall_time",wall_time},{"valid",valid},
    {"collision_query_policy",collisionQueryPolicy()},
    {"reason",valid?"integrated_both_sources":"waiting_fresh_integrated_rays"},
    {"source_stamp_ns",integrated_cloud_stamp_ns_},{"sources",sources},
    {"fusion",{{"sequence",fusion_timing_.sequence},{"begin_ns",fusion_timing_.begin_ns},
      {"end_ns",fusion_timing_.end_ns},{"begin_steady_ns",fusion_timing_.begin_steady_ns},
      {"end_steady_ns",fusion_timing_.end_steady_ns},
      {"front_begin_ns",fusion_timing_.source_stamps[0]},{"rear_begin_ns",fusion_timing_.source_stamps[1]},
      {"map_revision",occupancyRevision()},{"next_wake_delay_ns",ray_timer_delay_ns_},
      {"schedule_errors",ray_timer_schedule_errors_}}},
    {"integrated_counts",{{"0",ray_integrations_[0]},{"1",ray_integrations_[1]}}},
    {"ingress",{{"queue_depth",scan_planner::kProjectedRayIngressDepth},
      {"callback_count",ray_ingress_callback_count_},{"drain_taken_count",ray_ingress_drain_taken_count_},
      {"drain_accepted_count",ray_ingress_drain_accepted_count_},
      {"last_drain_taken",ray_ingress_last_drain_.taken},{"last_drain_accepted",ray_ingress_last_drain_.accepted}}},
    {"unattributed_drop_count",ray_unattributed_drops_}}.dump();
  projected_rays_status_pub_->publish(result);
  ray_status_schedule_.markPublished(now,integrated_cloud_stamp_ns_,valid,localization_context_sequence_);
}

void GridMap::invalidateCloudPosePairs(std::int64_t barrier)
{
  pair_barrier_ns_ = std::max(pair_barrier_ns_, barrier);
  cloud_pose_pair_drops_ += pending_clouds_.size();
  pending_clouds_.clear();
  pending_poses_.clear();
  for (auto &pending:pending_rays_) pending.reset();
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
  if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) return;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
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

sensor_msgs::msg::PointCloud2 &GridMap::cachedVisualization(bool inflated)
{
  const std::size_t layer = inflated ? 1 : 0;
  const int clip = md_.has_ray_pose_ ? static_cast<int>(std::floor(
      (md_.ray_pos_.z() + mp_.vis_height_) * mp_.resolution_inv_ - .5)) :
      std::numeric_limits<int>::max();
  if (visualization_revision_[layer] == occupancy_revision_ && visualization_clip_[layer] == clip)
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
        if (inflated ? md_.occupancy_buffer_inflate_[address] == 0 :
                       md_.occupancy_buffer_[address] < mp_.min_occupancy_log_)
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
  ++visualization_builds_[layer];
  return visualization_cache_[layer];
}

void GridMap::publishMap()
{
  if (map_pub_->get_subscription_count() == 0) return;
  auto &cloud_msg = cachedVisualization(false);
  cloud_msg.header.stamp = node_->now();
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool /*all_info*/)
{
  if (map_inf_pub_->get_subscription_count() == 0) return;
  auto &cloud_msg = cachedVisualization(true);
  cloud_msg.header.stamp = node_->now();
  map_inf_pub_->publish(cloud_msg);
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
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

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
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
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
