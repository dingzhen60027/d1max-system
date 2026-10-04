// Diagnostic only: no ROS context, transport or control.
// Uses production GridMap buffers/queries and production validateMotionSweep.
// Uniform observed-free space and a measured plane are explicit synthetic
// evidence fixtures, NOT a replacement runtime map or real robot evidence.
#include <plan_manage/motion_sweep.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

struct GridMapTestAccess {
  static void configure(GridMap& map,double up,double down) {
    auto& p=map.mp_;auto& d=map.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={80,80,60};
    p.map_origin_idx_={0,0,10};map.updateMapBoundaryFromIndex();
    p.use_projected_rays_=p.require_observed_free_=true;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.20;
    p.obstacles_inflation_z_up=up;p.obstacles_inflation_z_down=down;p.cloud_pose_max_age_=.5;
    p.frame_id_="odom";p.map_sliding_en_=false;
    d.occupancy_buffer_.assign(80*80*60,-1.);
    d.occupancy_buffer_inflate_.assign(80*80*60,0);
    d.occupancy_buffer_inflate_cnt_.assign(80*80*60,0);
    map.rebuildInflationOffsets();
    map.free_observation_stamps_.assign(80*80*60,100000000000LL);
    map.ray_integrated_stamps_={100000000000LL,100000000000LL};
    map.integrated_cloud_stamp_ns_=100000000000LL;
    map.snapshot_clock_ns_=100100000000LL;
    map.snapshot_captured_=std::chrono::steady_clock::now();
    map.collision_snapshot_=true;map.enforce_free_freshness_=true;map.ray_clock_fault_=false;
  }
  static void plane(GridMap& map,double z) {
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y) {
      Eigen::Vector3i cell;map.posToIndex({x*.05,y*.05,z},cell);
      if(map.isInMap(cell))map.md_.occupancy_buffer_[map.toAddress(cell)]=2.;
    }
  }
};

scan_planner::MotionSupport support(double height) {
  d1max_planning_interfaces::msg::SupportReference s;
  s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.04;s.body_reference_height_m=height;
  s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int x=-60;x<=60;++x)for(int y=-60;y<=60;++y) {
    geometry_msgs::msg::Point point;point.x=x*.025;point.y=y*.025;s.support_ground_xyz.push_back(point);
  }
  return scan_planner::MotionSupport(s);
}

int main() {
  const scan_planner::BrakingModel model{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
  nlohmann::json rows=nlohmann::json::array();
  for(const auto& input:std::vector<std::array<double,4>>{
      {.50,.40,.45,0.}, {.55,.45,.45,0.}, {.50,.40,.45,.10}, {.50,.40,.45,.960}}) {
    const auto [height,up,down,plane]=input;
    GridMap map;GridMapTestAccess::configure(map,up,down);GridMapTestAccess::plane(map,plane);
    map.beginObservedProof();const auto evidence=map.inspectInflateOccupancy({0.,0.,height},0.,true);
    const int curve=map.getInflateOccupancy({0.,0.,height},0.);
    geometry_msgs::msg::Twist command,measured;command.linear.x=.02;command.angular.z=-.04;
    const auto out=scan_planner::validateMotionSweep(map,support(height),model,{0.,0.,height},
        -1.570796326794897,measured,command,"odom",.1);
    // Contrast only: derived from the production inflated-obstacle query's
    // explicitly reflected body interval. No alternate certificate is issued.
    const double pad=std::tan(.15)*.025+.01;
    rows.push_back({{"body_height_m",height},{"obstacle_dilation_up_m",up},
      {"obstacle_dilation_down_m",down},{"occupied_plane_z_m",plane},
      {"native_pose_query",curve},{"pose_unique_counts",evidence.unique_counts},
      {"pose_raw_overlap_counts",evidence.counts},{"motion_valid",out.valid},
      {"motion_reason",out.reason},{"motion_unique_voxels_before_exit",out.unique_voxels},
      {"motion_z_bounds",{out.min_z,out.max_z}},
      {"reflected_body_contract_z_bounds_with_same_pad",{height-up-pad,height+down+pad}}});
  }
  const bool consistent=rows[0]["native_pose_query"]==0&&rows[0]["motion_valid"]==true&&
    rows[1]["motion_valid"]==true&&rows[2]["motion_valid"]==false&&
    rows[3]["native_pose_query"]==1&&rows[3]["motion_valid"]==false&&
    rows[3]["motion_reason"]=="motion_sweep_occupied";
  std::cout<<nlohmann::json({{"scope","native_geometry_contract_probe_explicit_synthetic_evidence"},
    {"asymmetric_vertical_contract_consistent",consistent},{"native_source_behavior_changed",true},
    {"production_deployed",false},
    {"sdk_connected",false},{"physical_acceptance",false},{"cases",rows}}).dump(2)<<'\n';
  return consistent?0:2;
}
