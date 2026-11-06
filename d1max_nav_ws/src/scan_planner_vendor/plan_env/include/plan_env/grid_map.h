#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <array>
#include <limits>
#include <functional>
#include <optional>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tuple>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <d1max_planning_interfaces/msg/projected_rays.hpp>
#include <plan_env/projected_rays.hpp>
#include <plan_env/projected_ray_mailbox.hpp>
#include <plan_env/coalesced_writer_wake.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>
#include <plan_env/voxel_collision.hpp>
#include <plan_env/observed_hit_bounds.hpp>
#include <plan_env/blocked_voxel_watch.hpp>

namespace scan_planner {class ParallelObservedRayVisits;}

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
// voxel hashing
template <typename T>
struct matrix_hash {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  // Optional candidate-only geometry. -1 preserves the legacy strict values;
  // it never changes the measured-evidence body envelope or raw swept proof.
  double planning_obstacles_inflation_z_up{-1.0};
  double planning_obstacles_inflation_z_down{-1.0};
  double double_cylinder_radius_, double_cylinder_offset_;
  // Collision approximation only, never an entity-return filter. The box is
  // centred on the supplied body pose; its dimensions do not set body height.
  std::string collision_model_{"double_cylinder"};
  double box_length_{-1.},box_width_{-1.};
  // Optional stair-only extra XY clearance. Nominal entity dimensions are
  // never reduced; ordinary floor queries retain box_length_/box_width_.
  double box_nominal_length_{-1.},box_nominal_width_{-1.},stair_planar_clearance_{-1.};
  bool stair_subvoxel_collision_{false};
  // Disabled without explicitly qualified source tread evidence. This lower
  // rigid-body extent is used ONLY for matched stair contacts, never all hits.
  double stair_contact_body_below_{-1.};
  double body_below_{-1.},body_above_{-1.};
  double planning_body_below_{-1.},planning_body_above_{-1.};
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  double vis_height_, ground_height_;
  bool show_occ_time_;
  double visualization_rate_hz_{3.0};

  /* mapping sensor input */
  string sensor_type_;
  bool cloud_is_world_;
  bool need_extrinsic_;
  bool strict_input_frames_{false};
  double maximum_cloud_pose_dt_{0.25};
  bool exact_cloud_pose_sync_{false};
  bool require_observed_free_{false};
  std::string planning_collision_policy_{"official"};
  bool use_projected_rays_{false};
  bool preview_only_{false};  // Diagnostic lease only, never execution authority.
  double cloud_pose_pair_wait_{0.25};
  double cloud_pose_max_age_{0.5};
  bool require_localization_context_{false};
  std::string localization_session_id_;
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion
struct InflationColumn {
  int x, y, begin_z, end_z;
};

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  // Complete union of original endpoint octants, not observed-free evidence.
  // Low eight bits are 2x2x2 hit cells; bit eight marks coarse-only history.
  // Collision narrows only Z: these bits are grouped into lower/upper halves,
  // keeping the full coarse XY area (5x5x2.5 cm at 5 cm map resolution).
  // Missing/empty/tainted evidence always falls back to the whole coarse cell.
  std::vector<std::uint16_t> occupied_hit_subvoxels_;
  std::vector<scan_planner::ObservedHitBounds> occupied_hit_bounds_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;
  vector<Eigen::Vector3i> inflate_offsets_;
  std::vector<InflationColumn> inflate_columns_;
  // Official SCAN geometry is independent of the raw-evidence execution
  // policy. A strict writer retains a separate, bounded official layer.
  std::vector<char> planning_occupancy_buffer_inflate_;
  std::vector<int> planning_occupancy_buffer_inflate_cnt_;
  vector<Eigen::Vector3i> planning_inflate_offsets_;
  std::vector<InflationColumn> planning_inflate_columns_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data

  cv::Mat depth_image_;
  int image_cnt_;
  // flags of map state

  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;
  rclcpp::Time ray_pose_stamp_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  vector<Eigen::Vector3d> proj_origins_;  // Only the opt-in context-tagged ray input.
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap();
  ~GridMap();

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  void setOccupiedHit(const Eigen::Vector3d& original_endpoint);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);
  int getInflateOccupancy(Eigen::Vector3d pos,double yaw,double pitch,double roll);
  // Candidate geometry only. A zero is NOT observed-free motion evidence.
  inline int getPlanningInflateOccupancy(Eigen::Vector3d pos, double yaw);
  int getPlanningInflateOccupancy(Eigen::Vector3d pos,double yaw,double pitch,double roll);
  std::string describeInflateOccupancy(const Eigen::Vector3d &position,double yaw);
  // Literal-pose diagnostics. Unlike the two-argument query, this does not
  // substitute a predicted stair attitude for an acquired body quaternion.
  std::string describeInflateOccupancy(const Eigen::Vector3d &position,double yaw,double pitch,double roll);
  scan_planner::CollisionEvidence inspectInflateOccupancy(const Eigen::Vector3d &position,double yaw,bool detailed=false);
  scan_planner::CollisionEvidence inspectInflateOccupancy(const Eigen::Vector3d &position,double yaw,
      double pitch,double roll,bool detailed=false);
  // Reader-local, version-bound query attitude. Never attached to the live
  // writer or copied into a new snapshot. Optional failure means unavailable
  // geometry (status 2), not an invented upright/clear body. ROS pitch, roll.
  using QueryAttitudeProvider=std::function<std::optional<Eigen::Vector2d>(const Eigen::Vector3d&,double)>;
  QueryAttitudeProvider exchangeQueryAttitudeProvider(QueryAttitudeProvider provider);
  void setQueryAttitudeProvider(QueryAttitudeProvider provider);
  void clearQueryAttitudeProvider();
  // Optional stair-source geometry for the native 2.5-D search plane. This
  // supplies candidate height only; it never changes a raw collision result.
  using QuerySupportHeightProvider=std::function<std::optional<double>(const Eigen::Vector3d&)>;
  QuerySupportHeightProvider exchangeQuerySupportHeightProvider(QuerySupportHeightProvider provider);
  using QueryStairClearanceProvider=std::function<bool(const Eigen::Vector3d&)>;
  QueryStairClearanceProvider exchangeQueryStairClearanceProvider(QueryStairClearanceProvider provider);
  using QueryStairContactProvider=std::function<bool(const Eigen::Vector3d&,const Eigen::Vector3d&)>;
  QueryStairContactProvider exchangeQueryStairContactProvider(QueryStairContactProvider provider);
  // Independent, finite static structure prior. This returns a hard geometry
  // intersection, never observed-free evidence, and never edits the raw map.
  using QueryHardBoundaryProvider=std::function<bool(const Eigen::Vector3d&,double,double,double,
      const scan_planner::BodyEnvelope&)>;
  QueryHardBoundaryProvider exchangeQueryHardBoundaryProvider(QueryHardBoundaryProvider provider);
  // Diagnostic-only bounded context, invoked solely by the existing failure
  // description path. It cannot alter collision results or create map data.
  using QueryHardBoundaryDiagnosticProvider=std::function<std::string(const Eigen::Vector3d&,
      double,double,double,const scan_planner::BodyEnvelope&)>;
  QueryHardBoundaryDiagnosticProvider exchangeQueryHardBoundaryDiagnosticProvider(
      QueryHardBoundaryDiagnosticProvider provider);
  bool stairContactConfigured() const {return mp_.stair_contact_body_below_>=0.;}
  double collisionResolution() const {return mp_.resolution_;}
  bool stairClearanceConfigured() const {return mp_.stair_planar_clearance_>=0.;}
  bool stairSubvoxelConfigured() const {return mp_.stair_subvoxel_collision_;}
  bool hasQuerySupportHeightProvider() const {return bool(query_support_height_provider_);}
  std::optional<double> querySupportBodyHeight(const Eigen::Vector3d& expected_body) const {
    return query_support_height_provider_?query_support_height_provider_(expected_body):std::nullopt;
  }
  // Offline diagnostic enablement only; no live config or permission changed.
  scan_planner::NearFieldDiagnostics &nearFieldDiagnostics() {return near_field_diagnostics_;}

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  // One exclusive callback group owns all mutable map state and fusion.
  // Formal projected rays use a separate serial decode/mailbox lane; it owns
  // no map cells, source leases, timers or collision snapshot publication.
  // Display rendering/publication has its own bounded, non-authoritative worker.
  // A null group preserves the node default for
  // legacy callers; formal sessions provide their dedicated writer group.
  void initMap(rclcpp::Node* node, rclcpp::CallbackGroup::SharedPtr callback_group = {});
  rclcpp::CallbackGroup::SharedPtr projectedIngressGroup()const {return projected_ingress_group_;}
  scan_planner::NativePipelineHealth nativePipelineHealth()const {
    return projected_ingress_.pipelineHealth();
  }

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();
  // Source time of the cloud actually integrated into occupancy, not merely
  // the latest message arrival. Zero means explicitly invalidated/no cloud.
  double latestCloudStamp() const { return integrated_cloud_stamp_ns_ * 1e-9; }
  std::int64_t latestCloudStampNs() const { return integrated_cloud_stamp_ns_; }
  const std::string &collisionFrame() const { return mp_.frame_id_; }
  // Acquisition BEGIN of the physical ray batch actually integrated. Never
  // receipt time, alignment time or acquisition_end.
  std::int64_t integratedRaySourceStamp(unsigned sensor) const {
    return sensor<2?ray_integrated_stamps_[sensor]:0;
  }
  // Diagnostic transaction clocks only. Never used as evidence acquisition
  // times or to extend a collision lease.
  struct FusionTiming {
    std::uint64_t sequence{0};
    std::int64_t begin_ns{0},end_ns{0},begin_steady_ns{0},end_steady_ns{0};
    std::array<std::int64_t,2> source_stamps{{0,0}};
  };
  FusionTiming fusionTiming() const {return fusion_timing_;}
  // Writer-only elapsed-work diagnostics. Not copied into collision snapshots,
  // not source clocks, and never used to extend sensor/evidence validity.
  struct MapWorkTiming {
    double initialize_ms{0.},traverse_ms{0.},occupancy_ms{0.};
    double post_raycast_ms{0.},writer_transaction_ms{0.};
    std::uint64_t points{0},ray_steps{0};
    std::int64_t raycast_end_steady_ns{0};
  };
  MapWorkTiming mapWorkTiming() const {return map_work_timing_;}
  // Set before spinning. Runs on the existing serialized fusion writer only,
  // after all ray updates and source-lease checks have committed.
  void setCollisionUpdateCallback(std::function<void()> callback) {
    collision_update_callback_=std::move(callback);
  }
  // Serialized writer opportunity BEFORE the next expensive integration.
  // The owner may deliver only an already pending, completed snapshot here.
  // Return false only while that validation HEAD is still pending; owner
  // replicas/solver copies must not defer new input. This is not a map update,
  // source observation or extra publication timer.
  void setCollisionPublicationRetryCallback(std::function<bool()> callback) {
    collision_publication_retry_callback_=std::move(callback);
  }
  void requireObservedSnapshot() {
    if(!collision_snapshot_ || node_) throw std::logic_error("strict evidence query requires private snapshot");
    mp_.require_observed_free_=true;enforce_free_freshness_=true;
    rebuildInflationOffsets();
  }
  bool isRawRaySnapshot() const {return collision_snapshot_ && mp_.use_projected_rays_;}
  scan_planner::ObstacleDilation obstacleDilation() const {
    if (mp_.collision_model_=="oriented_box")
      return {mp_.double_cylinder_radius_,mp_.double_cylinder_offset_,
        mp_.body_below_==-1. ? mp_.obstacles_inflation_z_up : mp_.body_below_,
        mp_.body_above_==-1. ? mp_.obstacles_inflation_z_down : mp_.body_above_,
        scan_planner::CollisionModel::OrientedBox,mp_.box_length_,mp_.box_width_};
    return {mp_.double_cylinder_radius_,mp_.double_cylinder_offset_,
      mp_.obstacles_inflation_z_up,mp_.obstacles_inflation_z_down};
  }
  scan_planner::ObstacleDilation planningObstacleDilation() const {
    auto shape=obstacleDilation();
    if (mp_.planning_obstacles_inflation_z_up!=-1.) shape.up=mp_.planning_obstacles_inflation_z_up;
    if (mp_.planning_obstacles_inflation_z_down!=-1.) shape.down=mp_.planning_obstacles_inflation_z_down;
    if (shape.model==scan_planner::CollisionModel::OrientedBox) {
      if (mp_.planning_body_below_!=-1.) shape.up=mp_.planning_body_below_;
      if (mp_.planning_body_above_!=-1.) shape.down=mp_.planning_body_above_;
    }
    return shape;
  }
  scan_planner::BodyEnvelope bodyEnvelope() const {
    return scan_planner::reflectedBodyEnvelope(obstacleDilation());
  }
  scan_planner::BodyEnvelope planningBodyEnvelope() const {
    return scan_planner::reflectedBodyEnvelope(planningObstacleDilation());
  }
  scan_planner::BodyEnvelope queryBodyEnvelope(const Eigen::Vector3d& position,bool planning=true) const;
  // Immutable render-only metadata, copied atomically to the display view.
  // The legacy form requires an exact body-yaw source match. The full form
  // carries attitude from the same acquisition, tolerating asynchronous callbacks
  // within the existing pose-age budget. Neither form changes query geometry.
  bool setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,
      std::int64_t source_stamp_ns,const std::string& frame);
  bool setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,double yaw,
      std::int64_t source_stamp_ns,const std::string& frame);
  bool setVisualizationBodyEnvelope(const scan_planner::BodyEnvelope& body,double yaw,
      double pitch,double roll,std::int64_t source_stamp_ns,const std::string& frame);
  // Rotating a stair OBB can move a corner by its full 3D reference radius,
  // not just the yaw-only XY radius. Flat/no-profile sampling stays identical.
  double queryBodyExtent(bool planning=true) const {
    const auto body=planning?planningBodyEnvelope():bodyEnvelope();
    const double xy=scan_planner::bodyExtent(body);
    return query_attitude_provider_ && body.model==scan_planner::CollisionModel::OrientedBox
      ? std::hypot(xy,std::max(body.below,body.above)):xy;
  }
  // Compatibility for older probes: these are obstacle-kernel dimensions,
  // NOT body up/down. New collision consumers must use bodyEnvelope().
  using CollisionShape=scan_planner::ObstacleDilation;
  CollisionShape collisionShape() const { return obstacleDilation(); }
  // The command reachable-volume checker uses the production raw evidence,
  // not getOccupancy() (whose zero also includes unseen voxels). An exclusive
  // private snapshot lease is mandatory. Call beginObservedProof before the
  // batch; observedProofDeadlineNs must still be fresh after the entire batch.
  int observedRawSnapshotStatus(const Eigen::Vector3i& cell) {
    if(!collision_snapshot_||node_||!mp_.require_observed_free_||!mp_.use_projected_rays_)
      throw std::logic_error("raw swept query requires observed native snapshot");
    if(collision_cache_clock_ns_==0)beginCollisionQuery();
    return rawCollisionStatus(cell);
  }
  // The SCAN command reader uses the SAME selected policy as native search and
  // trajectory geometry. In official mode an in-map unknown/weak cell is not
  // an occupied veto; its raw state is still returned for diagnostics. This
  // does not mutate odds, create FREE observations, or grant motion authority.
  int collisionSnapshotStatus(const Eigen::Vector3i& cell,int* raw_status=nullptr);
  void beginCollisionSnapshotProof() {
    beginObservedProof();
    if(planningRequiresObservedFree()) {
      if(!mp_.require_observed_free_)throw std::logic_error("strict SCAN policy requires observed snapshot");
      beginCollisionQuery();
    } else collision_cache_deadline_ns_=std::numeric_limits<std::int64_t>::max();
  }
  // Acquired sensor time only; neither snapshot publication nor query time
  // renews it. The motion checker also retains each ray/demand/body deadline.
  bool collisionSnapshotSourceFresh() const {
    const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-snapshot_captured_).count();
    if(!collision_snapshot_||node_||!mp_.use_projected_rays_||ray_clock_fault_||
       snapshot_clock_ns_<=0||elapsed<0||
       elapsed>std::numeric_limits<std::int64_t>::max()-snapshot_clock_ns_)return false;
    const auto now=snapshot_clock_ns_+elapsed;
    return integratedCloudFreshAt(now)&&
      scan_planner::rayStampFresh(ray_integrated_stamps_[0],now,mp_.cloud_pose_max_age_)&&
      scan_planner::rayStampFresh(ray_integrated_stamps_[1],now,mp_.cloud_pose_max_age_);
  }
  std::int64_t observedProofDeadlineNs() const {return collision_cache_deadline_ns_;}
  // Each trajectory proof owns its queried evidence set. An earlier curve's
  // cached expiry must not become this curve's apparent minimum source time.
  void beginObservedProof() {
    if(!collision_snapshot_ || node_)throw std::logic_error("proof scope requires private snapshot");
    observed_cylinder_cache_.clear();planning_box_cache_.clear();
    collision_cache_deadline_ns_=collision_cache_clock_ns_=0;
  }
  std::uint64_t localizationContextSequence() const { return localization_context_sequence_; }
  // Metadata only: a borrowed query view must retain this source-clock fault,
  // not silently fall back to a backup from before the fault/context reset.
  bool rayEvidenceClockFault() const { return ray_clock_fault_; }
  // Consumers must use the map's validated source-age contract, not an
  // unrelated odometry timeout. This query never renews an integrated stamp.
  bool integratedCloudFreshAt(std::int64_t now_ns) const {
    return scan_planner::rayStampFresh(integrated_cloud_stamp_ns_, now_ns,
                                       mp_.cloud_pose_max_age_);
  }
  // Effective collision evidence changes also wake WAIT_ENVIRONMENT without
  // forcing regeneration of unchanged visual occupancy clouds.
  std::uint64_t occupancyRevision() const { return occupancy_revision_+free_evidence_revision_; }
  bool requiresObservedFree() const { return mp_.require_observed_free_; }
  bool planningRequiresObservedFree() const { return mp_.planning_collision_policy_=="observed_free"; }
  const char *planningCollisionQueryPolicy() const { return mp_.planning_collision_policy_.c_str(); }
  const char *collisionQueryPolicy() const {
    if (mp_.collision_model_=="oriented_box")
      return mp_.require_observed_free_ ? "strict_observed_oriented_box" : "official_occupied_oriented_box";
    return mp_.require_observed_free_ ? "strict_observed_double_cylinder" :
        "official_inflated_double_cylinder";
  }
  const char *inflatedVisualizationSemantics() const {
    return mp_.collision_model_=="oriented_box" ? "occupied_oriented_box_current_body_attitude_coarse_voxels" :
        "double_cylinder_obstacle_inflation";
  }
  void resetCollisionDiagnostics() { unknown_collision_queries_=0; }
  std::uint64_t unknownCollisionQueries() const { return unknown_collision_queries_; }
  // Separate from goal/reference generations: only a localization coordinate
  // identity change may invalidate all accumulated world-frame evidence.
  bool applyLocalizationContext(const std::string &payload);

  typedef std::shared_ptr<GridMap> Ptr;
  // Called only by the map writer, between completed updates. Destination is
  // a memory-only native map, never a ROS owner. Queries require an exclusive
  // lease because the native query cache/diagnostics are mutable.
  void copyCollisionSnapshotTo(GridMap &destination, std::int64_t source_now_ns,
                               std::chrono::steady_clock::time_point captured) const;
  std::size_t collisionSnapshotBytes() const {
    return md_.occupancy_buffer_.size()*sizeof(double)+
        md_.occupied_hit_subvoxels_.size()*sizeof(std::uint16_t)+
        md_.occupied_hit_bounds_.size()*sizeof(scan_planner::ObservedHitBounds)+
        md_.occupancy_buffer_inflate_.size()*sizeof(md_.occupancy_buffer_inflate_[0])+
        md_.occupancy_buffer_inflate_cnt_.size()*sizeof(md_.occupancy_buffer_inflate_cnt_[0])+
        md_.inflate_offsets_.size()*sizeof(Eigen::Vector3i)+
        md_.inflate_columns_.size()*sizeof(InflationColumn)+
        md_.planning_occupancy_buffer_inflate_.size()*sizeof(char)+
        md_.planning_occupancy_buffer_inflate_cnt_.size()*sizeof(int)+
        md_.planning_inflate_offsets_.size()*sizeof(Eigen::Vector3i)+
        md_.planning_inflate_columns_.size()*sizeof(InflationColumn)+
        free_observation_stamps_.size()*sizeof(std::int64_t);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend struct GridMapTestAccess;
  MappingParameters mp_;
  MappingData md_;
  std::function<void()> collision_update_callback_;
  std::function<bool()> collision_publication_retry_callback_;
  // One executor yield per completed fusion, never a wait loop or a sensor
  // deadline. The next runnable batch proceeds even if readers remain busy.
  bool ray_pending_publication_yielded_{false};
  // Writer-only scratch/worker. Never copied or shared with collision views.
  std::unique_ptr<scan_planner::ParallelObservedRayVisits> parallel_ray_visits_;
  bool parallel_ray_visits_enabled_{true},last_parallel_ray_visits_{false};
  bool prepareParallelRayVisits(const std::array<std::int64_t,2>& sources);

  // get depth image and sensor pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);
  bool applyLidarPose(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
  void processExactCloudPosePairs();
  bool acceptProjectedRays(const d1max_planning_interfaces::msg::ProjectedRays &message,
                           std::int64_t now_ns, std::chrono::steady_clock::time_point received);
  void processProjectedRays(std::int64_t now_ns, std::chrono::steady_clock::time_point now);
  void drainProjectedIngress();
  void updateProjectedOccupancy(bool watchdog);
  void transferProjectedIngress(std::int64_t source_now,std::chrono::steady_clock::time_point now);
  void publishProjectedIngressContext();
  bool tryProjectedFusion(std::int64_t now_ns,std::chrono::steady_clock::time_point now,bool watchdog);
  std::chrono::nanoseconds projectedFusionWakeDelay(std::chrono::steady_clock::time_point now) const;
  void scheduleProjectedFusionWake();
  void publishProjectedRaysStatus(std::int64_t now_ns, std::chrono::steady_clock::time_point now);
  bool cloudPoseStampFresh(std::int64_t stamp) const;
  void invalidateCloudPosePairs(std::int64_t barrier);
  using PairReceipt = std::chrono::steady_clock::time_point;
  struct PendingCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    PairReceipt received;
  };
  struct PendingPose {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
    PairReceipt received;
  };
  std::map<std::int64_t, PendingCloud> pending_clouds_;
  std::map<std::int64_t, PendingPose> pending_poses_;
  using PendingRays=scan_planner::PendingProjectedRays;
  // Writer-owned frozen/pending batches, distinct from the two ingress slots.
  // Legacy callers retain their original unpublished decode scratch.
  // Retiring ready retains bounded storage, never a reusable source lease.
  std::array<PendingRays, 2> pending_rays_;
  scan_planner::ProjectedRayBatch ray_decode_scratch_;
  scan_planner::ProjectedRayMailbox projected_ingress_;
  scan_planner::ProjectedRayContextPtr projected_ingress_context_; // writer only
  bool projected_ingress_enabled_{false}; // immutable after init
  std::array<std::int64_t, 2> ray_received_stamps_{{0,0}}, ray_integrated_stamps_{{0,0}};
  std::array<std::uint64_t, 2> ray_projection_sequences_{{0,0}}, ray_integrations_{{0,0}}, ray_drops_{{0,0}};
  std::array<PairReceipt, 2> ray_integrated_receipts_;
  // Diagnostics only: callback/take admission time is not middleware arrival
  // time. These clocks never replace source stamps or authorize freshness.
  std::array<std::int64_t, 2> ray_accept_clock_ns_{{0,0}};
  std::array<std::int64_t, 2> ray_integration_start_ns_{{0,0}}, ray_integration_start_stamps_{{0,0}};
  std::array<double, 2> ray_pending_wait_s_{{0.,0.}};
  std::uint64_t ray_ingress_callback_count_{0},ray_ingress_drain_taken_count_{0},ray_ingress_drain_accepted_count_{0};
  std::chrono::steady_clock::time_point ray_fusion_last_start_{};
  bool ray_fusion_started_{false};
  double ray_integration_period_s_{.2};
  std::int64_t ray_fusion_last_pair_stamp_{0};
  FusionTiming fusion_timing_;
  MapWorkTiming map_work_timing_;
  std::uint64_t ray_timer_schedule_errors_{0};
  std::int64_t ray_timer_delay_ns_{20000000};
  scan_planner::ProjectedRayIngressDrain ray_ingress_last_drain_;
  scan_planner::ProjectedRayStatusSchedule ray_status_schedule_;
  std::uint64_t ray_unattributed_drops_{0}, localization_context_barrier_ns_{0};
  std::int64_t pair_barrier_ns_{0}, last_paired_stamp_ns_{0};
  std::int64_t projected_cloud_stamp_ns_{0}, integrated_cloud_stamp_ns_{0};
  std::uint64_t occupancy_revision_{0};
  std::uint64_t cloud_pose_pairs_{0}, cloud_pose_pair_drops_{0};
  scan_planner::VoxelStatusCache observed_cylinder_cache_;
  scan_planner::VoxelStatusCache planning_box_cache_;
  scan_planner::NearFieldDiagnostics near_field_diagnostics_;
  // Diagnostic-only reader requests. The writer owns history; immutable map
  // readers only copy completed history and can nonblockingly request 4 keys.
  std::shared_ptr<scan_planner::BlockedVoxelWatchMailbox> blocked_voxel_mailbox_{
      std::make_shared<scan_planner::BlockedVoxelWatchMailbox>()};
  scan_planner::BlockedVoxelWatch blocked_voxel_watch_;
  scan_planner::BlockedVoxelWatchSnapshot blocked_voxel_watch_snapshot_;
  std::uint64_t blocked_voxel_context_{1};
  PairReceipt blocked_voxel_last_log_{};
  scan_planner::BlockedVoxelRawState blockedVoxelRawState(const scan_planner::BlockedVoxelKey& key);
  std::string blockedVoxelWatchJson() const;
  std::vector<scan_planner::RayDiagnosticMetadata> projected_diagnostics_;
  // Fixed-size ring sidecar, not diagnostic witnesses. Only real traversals
  // renew it. Scan BEGIN is a conservative source-time bound for every ray.
  std::vector<std::int64_t> free_observation_stamps_, projected_ray_stamps_;
  // Measured hits and censored self-return endpoints share the same measured
  // segment, but only the former may add occupied evidence at its endpoint.
  std::vector<std::uint8_t> projected_ray_kinds_;
  std::int64_t ray_query_clock_ns_{0}, ray_tick_clock_ns_{0};
  std::int64_t ray_tick_effective_ns_{0};
  PairReceipt ray_tick_receipt_{};
  std::int64_t collision_cache_deadline_ns_{0}, collision_cache_clock_ns_{0};
  // Memory-only probe may disable for legacy comparison; no ROS opt-out.
  bool enforce_free_freshness_{true};
  bool ray_clock_fault_{false};
  bool collision_snapshot_{false};
  std::int64_t snapshot_clock_ns_{0};
  PairReceipt snapshot_captured_{};
  std::uint64_t free_evidence_revision_{0};
  bool free_evidence_recovered_{false};
  std::int64_t collisionQueryClock();
  std::int64_t collisionQueryClockAt(std::int64_t source,PairReceipt now) const;
  void advanceRayEvidenceClock(std::int64_t source,PairReceipt now,bool age_by_steady);
  void beginCollisionQuery();
  int rawCollisionStatus(const Eigen::Vector3i &cell);
  std::uint64_t unknown_collision_queries_{0};
  int observedCylinderStatus(const Eigen::Vector3d &center);
  int boxOccupancy(const Eigen::Vector3d &position,double yaw,bool planning);
  int boxOccupancy(const Eigen::Vector3d &position,double yaw,double pitch,double roll,bool planning);
  bool stairSubvoxelQueryAllowed(const Eigen::Vector3d& position,bool strict) const;
  bool occupiedSubvoxelIntersects(const Eigen::Vector3i& cell,int address,
      const scan_planner::OrientedBoxVoxel3DQuery& fine_query,
      scan_planner::SubvoxelCollisionCounts* diagnostic=nullptr,
      const scan_planner::OrientedBoxVoxel3DQuery* rigid_query=nullptr,
      bool refine_vertical=true) const;
  void recordOccupiedHit(const Eigen::Vector3d& point,const Eigen::Vector3i& cell,int address);
  void markCoarseOnlyOccupancy(int address);
  inline int getInflateOccupancyYawOnly(Eigen::Vector3d pos,double yaw);
  inline int getPlanningInflateOccupancyYawOnly(Eigen::Vector3d pos,double yaw);
  QueryAttitudeProvider query_attitude_provider_;
  QuerySupportHeightProvider query_support_height_provider_;
  QueryStairClearanceProvider query_stair_clearance_provider_;
  QueryStairContactProvider query_stair_contact_provider_;
  QueryHardBoundaryProvider query_hard_boundary_provider_;
  QueryHardBoundaryDiagnosticProvider query_hard_boundary_diagnostic_provider_;
  void invalidateCollisionQueryCaches() {
    observed_cylinder_cache_.clear();planning_box_cache_.clear();
  }
  std::string localization_context_payload_, localization_seed_;
  std::uint64_t localization_context_sequence_{0}, localization_epoch_{0};
  std::array<sensor_msgs::msg::PointCloud2, 2> visualization_cache_;
  std::array<std::uint64_t, 2> visualization_revision_{{
      std::numeric_limits<std::uint64_t>::max(), std::numeric_limits<std::uint64_t>::max()}};
  std::array<int, 2> visualization_clip_{{0, 0}};
  std::array<std::string, 2> visualization_model_;
  bool visualization_obb_packet_ready_{false};
  std::array<std::uint64_t, 2> visualization_builds_{{0, 0}};
  // Display-only coarse-voxel C-space at the last real body attitude, not a collision map or
  // observed-free proof. The display worker owns one bounded mask/difference buffer;
  // none of these arrays are copied to collision snapshots or queried by A*.
  std::int64_t visualization_body_stamp_ns_{0};
  double visualization_body_yaw_{std::numeric_limits<double>::quiet_NaN()};
  struct VisualizationBodyEnvelope {
    scan_planner::BodyEnvelope body;
    std::int64_t source_stamp_ns;
    std::string frame;
    std::optional<double> yaw;
    double pitch{0.},roll{0.};
  };
  std::shared_ptr<const VisualizationBodyEnvelope> visualization_body_envelope_;
  bool setVisualizationBodyEnvelopeImpl(const scan_planner::BodyEnvelope& body,
      std::optional<double> yaw,double pitch,double roll,
      std::int64_t source_stamp_ns,const std::string& frame);
  bool visualization_obb_kernel_ready_{false};
  std::array<double, 8> visualization_obb_kernel_key_{};
  std::vector<Eigen::Vector2i> visualization_obb_xy_offsets_;
  // A tilted box has a different exact contiguous Z interval in each XY column.
  std::vector<scan_planner::VerticalVoxelSpan> visualization_obb_z_spans_;
  std::vector<int> visualization_obb_difference_;
  std::vector<char> visualization_obb_mask_;
  std::uint64_t visualization_obb_mask_revision_{std::numeric_limits<std::uint64_t>::max()};
  Eigen::Vector3i visualization_obb_mask_min_{0, 0, 0};
  Eigen::Vector3i visualization_obb_mask_max_{0, 0, 0};
  std::uint64_t visualization_obb_mask_builds_{0};
  std::uint64_t visualization_obb_kernel_builds_{0};
  std::size_t visualization_obb_interval_writes_{0};
  bool prepareOrientedVisualizationMask();
  sensor_msgs::msg::PointCloud2 &cachedVisualization(bool inflated);
  // Render-only state has one byte of exact display classifications per cell,
  // not another occupancy/evidence map. It can never lend collision evidence.
  bool visualization_only_{false};
  std::vector<unsigned char> visualization_raw_flags_;
  std::int64_t visualization_capture_ns_{0};
  std::uint64_t visualization_capture_generation_{0};
  bool visualization_raw_requested_{false},visualization_inflated_requested_{false};
  bool visualization_depth_requested_{false};
  std::function<bool()> visualization_current_;
  bool visualizationRawCell(std::size_t address,unsigned char flag) const;
  rclcpp::Time visualizationStamp() const;
  bool visualizationCurrent() const;
  void copyVisualizationTo(GridMap& out,std::int64_t stamp,bool raw,bool inflated,bool depth) const;
  void startVisualizationWorker();
  void stopVisualizationWorker();
  bool submitVisualization(std::int64_t stamp,bool raw,bool inflated,bool depth);
  void runVisualizationWorker();
  std::unique_ptr<GridMap> visualization_view_;
  std::mutex visualization_mutex_;
  std::condition_variable visualization_wake_;
  std::thread visualization_worker_;
  std::atomic<bool> visualization_stop_{false};
  std::atomic<std::uint64_t> visualization_generation_{0};
  bool visualization_ready_{false};
  // The private sink makes the exact same bounded handoff testable without
  // creating a ROS node. Production uses the existing cloud/TF publishers.
  std::function<void(GridMap&)> visualization_sink_;

  // update occupancy by raycasting
  void updateOccupancyCallback();
  void visCallback();

  // main update process
  void projectDepthImage();
  bool raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  inline int getLocalIndex(int id, int dim) const;
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  inline void cacheOccupancyAtIndex(const Eigen::Vector3i &id,int address,int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  void updateSlidingMap(const Eigen::Vector3d& center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int addr);
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
  void applyOccupancyUpdateAtIndex(const Eigen::Vector3i& id, int addr, double new_log_odds);
  void rebuildInflationOffsets();
  static std::vector<InflationColumn> inflationColumns(const vector<Eigen::Vector3i>& offsets);
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const std::vector<InflationColumn>& columns,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

  rclcpp::Node* node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr projected_ingress_group_;
  scan_planner::CoalescedWriterWake::SharedPtr projected_writer_wake_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> depth_pose_sub_;
  SynchronizerImagePose sync_image_pose_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sliding_map_frame_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::ProjectedRays>::SharedPtr projected_rays_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr projected_rays_status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_context_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_context_ack_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sliding_map_bbox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr extrinsic_pose_pub_;
  rclcpp::TimerBase::SharedPtr occ_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline void GridMap::cacheOccupancyAtIndex(const Eigen::Vector3i &id,int address,int occ) {
  // Caller has already checked the voxel and computed its current ring address.
  if (++md_.count_hit_and_miss_[address] == 1) md_.cache_voxel_.push(id);
  if (occ == 1) ++md_.count_hit_[address];
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if(!pos.allFinite()||!std::isfinite(yaw))return -1;
  if(query_attitude_provider_) {
    const auto attitude=query_attitude_provider_(pos,yaw);
    return attitude?getInflateOccupancy(pos,yaw,attitude->x(),attitude->y()):2;
  }
  return getInflateOccupancyYawOnly(pos,yaw);
}

inline int GridMap::getInflateOccupancyYawOnly(Eigen::Vector3d pos, double yaw) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if (!pos.allFinite() || !std::isfinite(yaw)) return -1;
  if (mp_.collision_model_=="oriented_box") return boxOccupancy(pos,yaw,false);
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  if (!mp_.require_observed_free_) {
    // Upstream SCAN queries the two cylinder centers in the inflated buffer.
    // Zero inflation is not a claim that the underlying raw voxel was seen.
    const int front_occ=getInflateOccupancyFromBuffer(front,md_.occupancy_buffer_inflate_);
    if (front_occ!=0) return front_occ;
    return getInflateOccupancyFromBuffer(rear,md_.occupancy_buffer_inflate_);
  }

  // Retained opt-in strict policy: actual-height raw-voxel evidence and expiry.
  beginCollisionQuery();
  const int front_state=observedCylinderStatus(front);
  // An unknown front cylinder is not proof that the rear has no obstacle.
  // Keep the same blocked set, but report definite occupancy first so native
  // blocked-entry recovery sees the actual cause instead of a traversal order.
  const int state=front_state==1 ? 1 : scan_planner::mergeCollisionStatus(
      front_state,observedCylinderStatus(rear));
  if(state==0 && mp_.use_projected_rays_ && enforce_free_freshness_) {
    const auto now=collisionQueryClock();
    // Do not return a cached free certificate that expired during this query.
    if(ray_clock_fault_ || now<=0 || now<collision_cache_clock_ns_ ||
        now>=collision_cache_deadline_ns_) return 2;
  }
  return state;
}

inline int GridMap::getPlanningInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if(!pos.allFinite()||!std::isfinite(yaw))return -1;
  if(query_attitude_provider_) {
    const auto attitude=query_attitude_provider_(pos,yaw);
    return attitude?getPlanningInflateOccupancy(pos,yaw,attitude->x(),attitude->y()):2;
  }
  return getPlanningInflateOccupancyYawOnly(pos,yaw);
}

inline int GridMap::getPlanningInflateOccupancyYawOnly(Eigen::Vector3d pos, double yaw) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if (!pos.allFinite() || !std::isfinite(yaw)) return -1;
  if (planningRequiresObservedFree()) {
    // Invalid mixed configurations must never silently weaken the explicitly
    // requested strict planning policy (initMap rejects them as well).
    return mp_.require_observed_free_ ? getInflateOccupancyYawOnly(pos,yaw) : 2;
  }
  if (mp_.collision_model_=="oriented_box") return boxOccupancy(pos,yaw,true);
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
  const auto front=(pos+mp_.double_cylinder_offset_*heading).eval();
  const auto rear=(pos-mp_.double_cylinder_offset_*heading).eval();
  if (!isInMap(front) || !isInMap(rear)) return -1;
  // Non-strict writers already carry the upstream layer. A strict proof
  // snapshot copied from such a writer keeps that official layer unchanged.
  const auto& buffer=md_.planning_occupancy_buffer_inflate_.empty() ?
      md_.occupancy_buffer_inflate_ : md_.planning_occupancy_buffer_inflate_;
  if (buffer.empty() || buffer.size()!=md_.occupancy_buffer_.size()) return -1;
  const int front_occ=getInflateOccupancyFromBuffer(front,buffer);
  return front_occ!=0 ? front_occ : getInflateOccupancyFromBuffer(rear,buffer);
}

inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(buffer[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if(visualization_only_)throw std::logic_error("render-only map has no collision evidence");
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (!pos.allFinite()) return false;
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
