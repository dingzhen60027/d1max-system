#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "plan_env/grid_map.h"
#include "plan_env/observed_ray.hpp"
#include "plan_env/collision_snapshot_pool.hpp"
#include <numeric>
#include <set>
#include <future>
#include <thread>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Actual map integration, memory only: no ROS context/node/executor/transport.
struct GridMapTestAccess {
  static auto at(std::int64_t ns) {return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));}
  static void independentIngress(GridMap& map) {map.projected_ingress_enabled_=true;}
  static bool receive(GridMap& map,const d1max_planning_interfaces::msg::ProjectedRays& message,
      std::int64_t now=102000000000LL) {return map.projected_ingress_.accept(message,now,at(now));}
  static void transfer(GridMap& map,std::int64_t now=102000000000LL) {map.transferProjectedIngress(now,at(now));}
  static auto token(GridMap& map) {return map.projected_ingress_.context();}
  static bool finishDecode(GridMap& map,const scan_planner::ProjectedRayContextPtr& token,
      scan_planner::ProjectedRayBatch& batch,std::uint64_t sequence,std::int64_t now) {
    return map.projected_ingress_.offerDecoded(token,batch,sequence,now,at(now),batch.stamp_ns,batch.stamp_ns);
  }
  static auto firstReceipt(const GridMap& map,unsigned sensor) {return map.pending_rays_[sensor].received;}
  static auto writerReceipt(const GridMap& map,unsigned sensor) {return map.pending_rays_[sensor].writer_received;}
  static auto integratedReceipt(const GridMap& map,unsigned sensor) {return map.ray_integrated_receipts_[sensor];}
  static void serialRayTraversal(GridMap& map) {map.parallel_ray_visits_enabled_=false;}
  static void disableBlockedWatch(GridMap& map) {map.blocked_voxel_mailbox_.reset();}
  static auto blockedWatch(const GridMap& map) {return map.blocked_voxel_watch_snapshot_;}
  static auto blockedJson(const GridMap& map) {return nlohmann::json::parse(map.blockedVoxelWatchJson());}
  static bool nearFieldEnabled(GridMap& map) {return map.nearFieldDiagnostics().enabled();}
  static void resetForBlockedWatch(GridMap& map) {map.resetAllMapData();}
  static bool usedParallelRayTraversal(const GridMap& map) {return map.last_parallel_ray_visits_;}
  static bool ownsParallelRayWorker(const GridMap& map) {return static_cast<bool>(map.parallel_ray_visits_);}
  static void expectSameContactTraversal(const GridMap& a,const GridMap& b) {
    expectSameCompleteRayState(a,b);
    EXPECT_EQ(a.md_.occupied_hit_subvoxels_,b.md_.occupied_hit_subvoxels_);
    ASSERT_EQ(a.md_.occupied_hit_bounds_.size(),b.md_.occupied_hit_bounds_.size());
    for(std::size_t i=0;i<a.md_.occupied_hit_bounds_.size();++i) {
      ASSERT_EQ(a.md_.occupied_hit_bounds_[i].low,b.md_.occupied_hit_bounds_[i].low)<<i;
      ASSERT_EQ(a.md_.occupied_hit_bounds_[i].high,b.md_.occupied_hit_bounds_[i].high)<<i;
    }
    EXPECT_EQ(a.md_.flag_traverse_,b.md_.flag_traverse_);
    EXPECT_EQ(a.mapWorkTiming().ray_steps,b.mapWorkTiming().ray_steps);
  }
  static void configure(GridMap &map,bool require_observed_free=true) {
    auto &p=map.mp_;auto &d=map.md_;
    p.use_projected_rays_=p.require_localization_context_=true;
    p.require_observed_free_=require_observed_free;
    p.localization_session_id_="session";p.sensor_type_="lidar";p.frame_id_="map";
    p.map_sliding_en_=false;p.local_update_range_={1.9,1.9,1.9};
    p.resolution_=.1;p.resolution_inv_=10.;p.map_voxel_num_={40,40,40};
    p.map_origin_idx_.setZero();map.updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.prob_hit_log_=3.;p.prob_miss_log_=-.5;p.max_ray_length_=4.;
    p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.1;p.obstacles_inflation_z_down=.1;
    p.ground_height_=0.;p.cloud_pose_max_age_=.5;p.cloud_pose_pair_wait_=.25;
    d.has_ray_pose_=false;d.ray_pos_.setZero();d.raycast_num_=0;
    const std::size_t size=64000;
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);
    d.count_hit_and_miss_.resize(size);d.flag_rayend_.resize(size);d.flag_traverse_.resize(size);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static bool accept(GridMap &map,const d1max_planning_interfaces::msg::ProjectedRays &msg,
      std::int64_t now=102000000000LL,double receipt=102.) {
    return map.acceptProjectedRays(msg,now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
        static_cast<std::int64_t>(receipt*1e9))));
  }
  static void integrate(GridMap &map,std::int64_t now=102000000000LL,double receipt=102.) {
    map.processProjectedRays(now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
        static_cast<std::int64_t>(receipt*1e9))));
  }
  static bool eventFusion(GridMap& map,std::int64_t now,bool watchdog=false) {
    return map.tryProjectedFusion(now,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(now)),watchdog);
  }
  static auto nextWake(GridMap& map,std::int64_t now) {
    return map.projectedFusionWakeDelay(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(now))).count();
  }
  static void fusionPeriod(GridMap& map,double period) {map.ray_integration_period_s_=period;}
  static double raw(GridMap &map,double x,double y=.05,double z=.05) {
    Eigen::Vector3i cell;map.posToIndex({x,y,z},cell);return map.md_.occupancy_buffer_[map.toAddress(cell)];
  }
  static auto counts(GridMap &map) {return map.ray_integrations_;}
  static auto drops(GridMap &map) {return map.ray_drops_;}
  static auto pending(GridMap &map) {return map.pending_rays_[0].ready+map.pending_rays_[1].ready;}
  static std::set<const void*> ingressStorage(const GridMap& map) {
    std::set<const void*> pointers;
    const auto add=[&](const scan_planner::ProjectedRayBatch& batch) {
      if(batch.endpoints.capacity())pointers.insert(batch.endpoints.data());
      if(batch.origins.capacity())pointers.insert(batch.origins.data());
      if(batch.diagnostics.capacity())pointers.insert(batch.diagnostics.data());
    };
    add(map.ray_decode_scratch_);for(const auto& pending:map.pending_rays_)add(pending.batch);
    return pointers;
  }
  static auto drain(GridMap &map,const std::vector<d1max_planning_interfaces::msg::ProjectedRays> &queue,
      std::size_t &cursor,std::int64_t now=102000000000LL,double receipt=102.) {
    return scan_planner::drainProjectedRayIngress<d1max_planning_interfaces::msg::ProjectedRays>(
        [&queue,&cursor](auto &message) {
          if(cursor==queue.size()) return false;
          message=queue[cursor++];return true;
        },[&map,now,receipt](const auto &message) {return accept(map,message,now,receipt);});
  }
  static auto acceptedClock(const GridMap &map) {return map.ray_accept_clock_ns_;}
  static auto integrationStart(const GridMap &map) {return map.ray_integration_start_ns_;}
  static auto integratedPoints(const GridMap &map) {return map.md_.proj_points_cnt;}
  static void legacy(GridMap &map) {map.mp_.use_projected_rays_=false;}
  static void subvoxels(GridMap& map) {map.mp_.stair_subvoxel_collision_=true;map.resetAllMapData();}
  static void stairDiagnosticBox(GridMap& map) {
    auto& p=map.mp_;p.collision_model_="oriented_box";p.box_length_=p.box_nominal_length_=.6;
    p.box_width_=p.box_nominal_width_=.4;p.stair_planar_clearance_=0.;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.planning_collision_policy_="official";p.require_observed_free_=false;
    p.stair_subvoxel_collision_=true;map.rebuildInflationOffsets();map.resetAllMapData();
    map.exchangeQueryStairClearanceProvider([](const Eigen::Vector3d&){return true;});
  }
  static void contactEvidence(GridMap& map) {
    stairDiagnosticBox(map);map.mp_.stair_contact_body_below_=.12;
    map.resetAllMapData();
  }
  static auto bounds(GridMap& map,const Eigen::Vector3d& p) {
    Eigen::Vector3i cell;map.posToIndex(p,cell);
    return map.md_.occupied_hit_bounds_.at(map.toAddress(cell)).world(cell,map.mp_.resolution_);
  }
  static auto hitMask(GridMap& map,const Eigen::Vector3d& point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    return map.md_.occupied_hit_subvoxels_.at(map.toAddress(cell));
  }
  static auto allHitMasks(const GridMap& map) {return map.md_.occupied_hit_subvoxels_;}
  static void previewTiming(GridMap &map,double age) {
    auto &p=map.mp_;p.preview_only_=true;p.cloud_pose_max_age_=age;
    scan_planner::validateCloudPoseTiming(p.cloud_pose_pair_wait_,age,
        p.preview_only_,p.require_observed_free_,p.use_projected_rays_);
  }
  static void sourceTiming(GridMap &map,double age) {
    map.mp_.cloud_pose_max_age_=age;
    scan_planner::validateCloudPoseTiming(map.mp_.cloud_pose_pair_wait_,age,
        false,map.mp_.require_observed_free_,map.mp_.use_projected_rays_);
  }
  static void diagnostics(GridMap &map) {
    map.nearFieldDiagnostics().configure(true,Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(5.),.1);
  }
  static bool directAcquisitions(GridMap& map,const std::vector<std::int64_t>& stamps,
      const std::vector<Eigen::Vector3d>& origins,const std::vector<Eigen::Vector3d>& ends,
      const std::vector<std::uint8_t>& kinds={}) {
    if(stamps.size()!=origins.size()||stamps.size()!=ends.size())throw std::invalid_argument("fixture shape");
    if(!kinds.empty()&&kinds.size()!=ends.size())throw std::invalid_argument("fixture kinds");
    map.md_.proj_origins_=origins;map.md_.proj_points_=ends;
    map.md_.proj_points_cnt=static_cast<int>(ends.size());map.projected_ray_stamps_=stamps;
    map.projected_ray_kinds_=kinds;
    map.md_.ray_pos_=origins.front();map.projected_diagnostics_.clear();
    map.beginCollisionQuery();return map.raycastProcess();
  }
  static void fullFixtureRayRange(GridMap& map) {
    map.mp_.local_update_range_={4.,4.,4.};map.mp_.max_ray_length_=8.;
  }
  static void clippingFixture(GridMap& map,double range,double maximum) {
    map.mp_.local_update_range_=Eigen::Vector3d::Constant(range);map.mp_.max_ray_length_=maximum;
  }
  static void inflationFixture(GridMap& map,const Eigen::Vector3i& origin,bool irregular,bool tall=false) {
    map.mp_.map_origin_idx_=origin;map.updateMapBoundaryFromIndex();
    map.mp_.double_cylinder_radius_=.29;
    map.mp_.obstacles_inflation_z_up=tall?3.1:.25;
    map.mp_.obstacles_inflation_z_down=tall?3.1:.15;
    map.rebuildInflationOffsets();map.resetAllMapData();
    if(irregular) {
      // Preserve the generic API: duplicate cells, gaps and reversed ordering
      // cannot accidentally become a filled vertical interval.
      const std::vector<Eigen::Vector3i> offsets={{0,0,-3},{0,0,-2},{0,0,1},{0,0,1},
        {0,0,0},{0,0,-1},{1,-2,-4},{1,-2,-3},{1,-2,3},{-2,1,2},{-2,1,3}};
      map.md_.inflate_offsets_=offsets;map.md_.planning_inflate_offsets_=offsets;
      map.md_.inflate_columns_=GridMap::inflationColumns(offsets);
      map.md_.planning_inflate_columns_=map.md_.inflate_columns_;
    }
  }
  static void inflationUpdate(GridMap& map,const Eigen::Vector3i& cell,int delta,
      const std::vector<char>* mask,bool legacy) {
    if(!legacy){map.updateInflation(cell,delta,mask);return;}
    const auto update=[&](const auto& offsets,auto& counts,auto& flags) {
      for(const auto& offset:offsets) {
        const Eigen::Vector3i target=cell+offset;
        if(!map.isInMap(target))continue;
        const auto address=map.toAddress(target);
        if(mask&&(*mask)[address])continue;
        counts[address]+=delta;if(counts[address]<0)counts[address]=0;
        flags[address]=counts[address]>0?1:0;
      }
    };
    update(map.md_.inflate_offsets_,map.md_.occupancy_buffer_inflate_cnt_,
      map.md_.occupancy_buffer_inflate_);
    if(!map.md_.planning_occupancy_buffer_inflate_cnt_.empty())
      update(map.md_.planning_inflate_offsets_,map.md_.planning_occupancy_buffer_inflate_cnt_,
        map.md_.planning_occupancy_buffer_inflate_);
  }
  static void expectSameInflation(const GridMap& a,const GridMap& b) {
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_cnt_,b.md_.occupancy_buffer_inflate_cnt_);
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_,b.md_.occupancy_buffer_inflate_);
    EXPECT_EQ(a.md_.planning_occupancy_buffer_inflate_cnt_,b.md_.planning_occupancy_buffer_inflate_cnt_);
    EXPECT_EQ(a.md_.planning_occupancy_buffer_inflate_,b.md_.planning_occupancy_buffer_inflate_);
    EXPECT_EQ(a.md_.occupancy_buffer_,b.md_.occupancy_buffer_);
    EXPECT_EQ(a.free_observation_stamps_,b.free_observation_stamps_);
  }
  static auto buffers(const GridMap &map) {return map.md_.occupancy_buffer_;}
  static void expectSameRayEvidence(const GridMap &a,const GridMap &b) {
    EXPECT_EQ(a.md_.occupancy_buffer_,b.md_.occupancy_buffer_);
    // Inflation intentionally follows a different query geometry in each policy.
    EXPECT_EQ(a.free_observation_stamps_,b.free_observation_stamps_);
    EXPECT_EQ(a.ray_integrations_,b.ray_integrations_);
    EXPECT_EQ(a.ray_integrated_stamps_,b.ray_integrated_stamps_);
    EXPECT_EQ(a.integrated_cloud_stamp_ns_,b.integrated_cloud_stamp_ns_);
  }
  static void pointQueryGeometry(GridMap &map) {
    map.mp_.double_cylinder_radius_=map.mp_.double_cylinder_offset_=0.;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=0.;
    map.rebuildInflationOffsets();
  }
  static void displayHeight(GridMap &map) {map.mp_.vis_height_=2.;}
  static sensor_msgs::msg::PointCloud2 display(GridMap &map,bool inflated,std::int64_t stamp) {
    GridMap view;map.copyVisualizationTo(view,stamp,true,true,false);
    return view.cachedVisualization(inflated);
  }
  static void centerOnlyOfficialGeometry(GridMap &map) {
    map.mp_.double_cylinder_radius_=.01;map.mp_.double_cylinder_offset_=0.;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=0.;
    map.rebuildInflationOffsets();
  }
  static auto freeStamp(GridMap &map,const Eigen::Vector3i &cell) {
    return map.free_observation_stamps_.at(map.toAddress(cell));
  }
  static void queryTime(GridMap &map,std::int64_t stamp) {map.ray_query_clock_ns_=stamp;}
  static auto steadyClockAdvance(GridMap &map,std::int64_t source,std::int64_t receipt) {
    const auto at=std::chrono::steady_clock::time_point(std::chrono::nanoseconds(receipt));
    map.advanceRayEvidenceClock(source,at,true);
    map.ray_query_clock_ns_=map.collisionQueryClockAt(source,at);
    return map.ray_query_clock_ns_;
  }
  static void expectSameEvidence(const GridMap &a,const GridMap &b) {
    EXPECT_EQ(a.md_.occupancy_buffer_,b.md_.occupancy_buffer_);
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_,b.md_.occupancy_buffer_inflate_);
    EXPECT_EQ(a.md_.occupancy_buffer_inflate_cnt_,b.md_.occupancy_buffer_inflate_cnt_);
    EXPECT_EQ(a.md_.count_hit_,b.md_.count_hit_);
    EXPECT_EQ(a.md_.count_hit_and_miss_,b.md_.count_hit_and_miss_);
    EXPECT_EQ(a.md_.flag_rayend_,b.md_.flag_rayend_);
    if(a.mp_.use_projected_rays_&&b.mp_.use_projected_rays_) {
      // The optimized projected visitor stores two source-visit bits in the
      // old scratch byte; compare canonical visit coverage, not its encoding.
      ASSERT_EQ(a.md_.flag_traverse_.size(),b.md_.flag_traverse_.size());
      for(std::size_t i=0;i<a.md_.flag_traverse_.size();++i)
        ASSERT_EQ(a.md_.flag_traverse_[i]!=static_cast<char>(-1),
                  b.md_.flag_traverse_[i]!=static_cast<char>(-1))<<i;
    } else EXPECT_EQ(a.md_.flag_traverse_,b.md_.flag_traverse_);
    EXPECT_EQ(a.ray_integrations_,b.ray_integrations_);
    EXPECT_EQ(a.ray_integrated_stamps_,b.ray_integrated_stamps_);
    EXPECT_EQ(a.occupancyRevision(),b.occupancyRevision());
  }
  static void expectSameCompleteRayState(const GridMap& a,const GridMap& b) {
    expectSameEvidence(a,b);
    EXPECT_EQ(a.free_observation_stamps_,b.free_observation_stamps_);
    EXPECT_EQ(a.ray_received_stamps_,b.ray_received_stamps_);
    EXPECT_EQ(a.ray_projection_sequences_,b.ray_projection_sequences_);
    EXPECT_EQ(a.ray_accept_clock_ns_,b.ray_accept_clock_ns_);
    EXPECT_EQ(a.ray_integration_start_ns_,b.ray_integration_start_ns_);
    EXPECT_EQ(a.ray_integration_start_stamps_,b.ray_integration_start_stamps_);
    EXPECT_EQ(a.ray_pending_wait_s_,b.ray_pending_wait_s_);
    EXPECT_EQ(a.ray_integrated_receipts_,b.ray_integrated_receipts_);
    EXPECT_EQ(a.ray_drops_,b.ray_drops_);EXPECT_EQ(a.ray_unattributed_drops_,b.ray_unattributed_drops_);
    EXPECT_EQ(a.integrated_cloud_stamp_ns_,b.integrated_cloud_stamp_ns_);
    EXPECT_EQ(a.free_evidence_recovered_,b.free_evidence_recovered_);
    EXPECT_EQ(a.fusion_timing_.sequence,b.fusion_timing_.sequence);
    EXPECT_EQ(a.fusion_timing_.begin_ns,b.fusion_timing_.begin_ns);
    EXPECT_EQ(a.fusion_timing_.end_ns,b.fusion_timing_.end_ns);
    EXPECT_EQ(a.fusion_timing_.begin_steady_ns,b.fusion_timing_.begin_steady_ns);
    EXPECT_EQ(a.fusion_timing_.end_steady_ns,b.fusion_timing_.end_steady_ns);
    EXPECT_EQ(a.fusion_timing_.source_stamps,b.fusion_timing_.source_stamps);
  }
  static void productionGeometry(GridMap &map) {
    // Exact current cylinder parameters; test map resolution remains .1 m.
    map.mp_.double_cylinder_radius_=.29;map.mp_.double_cylinder_offset_=.2;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=.45;
    map.rebuildInflationOffsets();
  }
  static void productionProbabilities(GridMap &map) {
    const auto odds=[](double p){return std::log(p/(1.-p));};
    map.mp_.clamp_min_log_=odds(.12);map.mp_.clamp_max_log_=odds(.98);
    map.mp_.min_occupancy_log_=odds(.8);map.mp_.prob_hit_log_=odds(.85);
    map.mp_.prob_miss_log_=odds(.3);map.resetAllMapData();
  }
  static void dynamicObstacleQueryGeometry(GridMap &map) {
    // A small native cylinder fixture isolates the occupancy/inflation update;
    // it is not a replacement for, or acceptance of, the D1 Max body geometry.
    map.mp_.double_cylinder_radius_=.15;map.mp_.double_cylinder_offset_=0.;
    map.mp_.obstacles_inflation_z_up=map.mp_.obstacles_inflation_z_down=.1;
    map.rebuildInflationOffsets();
  }
  static int inflationCount(GridMap &map,const Eigen::Vector3d &position) {
    Eigen::Vector3i cell;map.posToIndex(position,cell);
    return map.md_.occupancy_buffer_inflate_cnt_.at(map.toAddress(cell));
  }
  static Eigen::Vector3i index(GridMap &map,const Eigen::Vector3d &position) {
    Eigen::Vector3i result;map.posToIndex(position,result);return result;
  }
  static int address(GridMap &map,const Eigen::Vector3i &cell) {return map.toAddress(cell);}
  static void indexedUpdate(GridMap &map,const Eigen::Vector3i &cell,double value) {
    ASSERT_TRUE(map.isInMap(cell));
    const scan_planner::ObservedRayMapIndex index(map.mp_.map_bound_min_idx_,map.mp_.map_voxel_num_);
    map.applyOccupancyUpdateAtIndex(cell,index.address(cell),value);
  }
  // Frozen pre-optimization log-odds update: same map/buffers, no alternate
  // map model. Compare every state transition and inflation/reference count.
  static void legacyUpdate(GridMap &map,const Eigen::Vector3i &id,double value) {
    if(!map.isInMap(id)) return;
    const int addr=map.toAddress(id);
    const bool was_occ=map.md_.occupancy_buffer_[addr]>map.mp_.min_occupancy_log_;
    const bool now_occ=value>map.mp_.min_occupancy_log_;
    const bool was_known=map.md_.occupancy_buffer_[addr]>=map.mp_.clamp_min_log_;
    const bool now_known=value>=map.mp_.clamp_min_log_;
    const bool free_changed=scan_planner::strictRawVoxelStatus(map.md_.occupancy_buffer_[addr],
        map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_) !=
        scan_planner::strictRawVoxelStatus(value,map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_);
    if(was_known!=now_known || was_occ!=now_occ || (map.mp_.require_observed_free_ && free_changed)) {
      map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
    }
    map.md_.occupancy_buffer_[addr]=value;
    if(was_occ!=now_occ) map.updateInflation(id,now_occ?1:-1);
  }
  static scan_planner::RawVoxelDiagnostic category(GridMap &map,const Eigen::Vector3i &cell) {
    if(!map.isInMap(cell)) return scan_planner::RawVoxelDiagnostic::Outside;
    return scan_planner::diagnoseRawVoxel(map.md_.occupancy_buffer_[map.toAddress(cell)],
        map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_,map.mp_.unknown_flag_);
  }
  static void slide(GridMap &map,const Eigen::Vector3d &center) {
    map.mp_.map_sliding_en_=true;map.mp_.map_sliding_thresh_vox_=1;
    map.updateSlidingMap(center);
    map.mp_.map_sliding_en_=false; // Further packet integration holds this test window.
  }
};
static builtin_interfaces::msg::Time timeAt(std::int64_t ns) {
  builtin_interfaces::msg::Time out;out.sec=ns/1000000000LL;out.nanosec=ns%1000000000LL;return out;
}
static std::string context(unsigned epoch=1,unsigned sequence=1,const std::string &seed="seed") {
  return nlohmann::json{{"schema",1},{"session_id","session"},{"epoch",epoch},{"sequence",sequence},
    {"seed_id",seed},{"barrier_ns",100000000000ULL+sequence}}.dump();
}
static d1max_planning_interfaces::msg::ProjectedRays packet(std::uint16_t sensor=0,
    std::int64_t stamp=101900000000LL,std::uint64_t sequence=1,unsigned count=1,
    Eigen::Vector3d origin={.55,.05,.05},Eigen::Vector3d end={1.55,.05,.05}) {
  d1max_planning_interfaces::msg::ProjectedRays out;
  out.session_id="session";out.epoch=1;out.context_sequence=1;out.seed_id="seed";
  out.barrier_ns=100000000001ULL;out.projection_sequence=sequence;
  out.acquisition_end=timeAt(stamp+50000000);out.alignment_stamp=timeAt(stamp);
  auto &c=out.rays;c.header.frame_id="map";c.header.stamp=timeAt(stamp);
  c.width=count;c.height=1;c.point_step=64;c.row_step=count*64;c.data.resize(c.row_step);
  using F=sensor_msgs::msg::PointField;
  const std::pair<const char*,unsigned> fields[]={{"x",0},{"y",4},{"z",8},
    {"origin_x",16},{"origin_y",20},{"origin_z",24},{"sensor_id",28}};
  for (auto field:fields) {
    F f;f.name=field.first;f.offset=field.second;f.count=1;f.datatype=f.offset==28?F::UINT16:F::FLOAT32;
    c.fields.push_back(f);
  }
  const float xyz[]={float(end.x()),float(end.y()),float(end.z())};
  const float start[]={float(origin.x()),float(origin.y()),float(origin.z())};
  for(unsigned i=0;i<count;++i) {
    auto *bytes=c.data.data()+64*i;std::memcpy(bytes,xyz,12);std::memcpy(bytes+16,start,12);
    std::memcpy(bytes+28,&sensor,2);
  }
  return out;
}

TEST(ProjectedRayMailbox, LatestPerSourcePreservesCompleteProductionMapState) {
  for(unsigned rear_position=0;rear_position<5;++rear_position) {
    GridMap serial,split;for(auto* map:{&serial,&split})GridMapTestAccess::configure(*map,false);
    GridMapTestAccess::independentIngress(split);
    ASSERT_TRUE(serial.applyLocalizationContext(context()));ASSERT_TRUE(split.applyLocalizationContext(context()));
    for(unsigned i=0;i<5;++i) {
      auto message=packet(i==rear_position?1:0,101700000000LL+i*40000000LL,i+1,3);
      ASSERT_TRUE(GridMapTestAccess::accept(serial,message));ASSERT_TRUE(GridMapTestAccess::receive(split,message));
    }
    EXPECT_EQ(GridMapTestAccess::pending(split),0); // Receiver never writes the live map's pending set.
    GridMapTestAccess::transfer(split);
    GridMapTestAccess::integrate(serial);GridMapTestAccess::integrate(split);
    GridMapTestAccess::expectSameCompleteRayState(serial,split);
  }
}

TEST(ProjectedRayMailbox, BusySoleWriterDoesNotBlockIngressOrMutateFrozenBatch) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::receive(map,packet(sensor)));
  GridMapTestAccess::transfer(map);
  std::promise<void> writer_entered,release_writer;
  auto release=release_writer.get_future().share();
  map.setCollisionUpdateCallback([&]{writer_entered.set_value();release.wait();});
  auto writer=std::async(std::launch::async,[&]{GridMapTestAccess::integrate(map);});
  const auto entered=writer_entered.get_future().wait_for(std::chrono::seconds(2));
  if(entered!=std::future_status::ready) {release_writer.set_value();writer.wait();FAIL()<<"writer did not reach production completion";}
  // The writer is still inside the actual integration transaction/callback.
  auto receiver=std::async(std::launch::async,[&]{
    bool accepted=true;
    for(unsigned sensor=0;sensor<2;++sensor)
      accepted=GridMapTestAccess::receive(map,packet(sensor,102050000000LL,2),102100000000LL)&&accepted;
    return accepted;
  });
  const auto received=receiver.wait_for(std::chrono::seconds(2));
  release_writer.set_value();writer.get();
  ASSERT_EQ(received,std::future_status::ready);ASSERT_TRUE(receiver.get());
  EXPECT_EQ(map.integratedRaySourceStamp(0),101900000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
  map.setCollisionUpdateCallback({});
  GridMapTestAccess::transfer(map,102100000000LL);GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102050000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),102050000000LL);
}

TEST(ProjectedRayMailbox, ResetRejectsDecodedOldTokenAndDuplicateContextPreservesWaiting) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const auto old_token=GridMapTestAccess::token(map);
  auto delayed=scan_planner::decodeProjectedRays(packet().rays,"map");ASSERT_TRUE(delayed);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet()));
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new-seed")));
  EXPECT_FALSE(GridMapTestAccess::finishDecode(map,old_token,*delayed,99,102000000000LL));
  EXPECT_FALSE(GridMapTestAccess::receive(map,packet()));
  for(unsigned sensor=0;sensor<2;++sensor) {
    auto message=packet(sensor);message.epoch=2;message.context_sequence=2;
    message.seed_id="new-seed";message.barrier_ns=100000000002ULL;
    ASSERT_TRUE(GridMapTestAccess::receive(map,message));
  }
  const auto new_token=GridMapTestAccess::token(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new-seed")));
  EXPECT_EQ(GridMapTestAccess::token(map),new_token);
  EXPECT_FALSE(map.applyLocalizationContext(context()));
  GridMapTestAccess::transfer(map);ASSERT_EQ(GridMapTestAccess::pending(map),2);
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
}

TEST(ProjectedRayMailbox, MalformedLatePointCannotReplaceValidBatchOrAdvanceSequence) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0)));
  auto bad=packet(0,101950000000LL,99,3);const float nan=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(bad.rays.data.data()+2*64,&nan,sizeof(nan));
  ASSERT_FALSE(GridMapTestAccess::receive(map,bad));
  GridMapTestAccess::transfer(map);GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.integratedRaySourceStamp(0),101900000000LL);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,101950000000LL,2)));
  GridMapTestAccess::transfer(map);GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.integratedRaySourceStamp(0),101950000000LL);
}

TEST(ProjectedRayMailbox, EarlierReceiptDoesNotAccidentallyConsumeWriterPairWait) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::sourceTiming(map,.6);
  GridMapTestAccess::independentIngress(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::receive(map,packet(sensor),102000000000LL));
  // The previous fusion occupied the owner for 300ms. Source age is 400ms,
  // still valid. The old serial owner would only admit this packet now.
  GridMapTestAccess::transfer(map,102300000000LL);
  EXPECT_EQ(GridMapTestAccess::firstReceipt(map,0),GridMapTestAccess::at(102000000000LL));
  EXPECT_EQ(GridMapTestAccess::writerReceipt(map,0),GridMapTestAccess::at(102300000000LL));
  GridMapTestAccess::integrate(map,102300000000LL,102.3);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  EXPECT_EQ(GridMapTestAccess::integratedReceipt(map,0),GridMapTestAccess::at(102000000000LL));
  EXPECT_TRUE(map.integratedCloudFreshAt(102499999999LL));
  EXPECT_FALSE(map.integratedCloudFreshAt(102500000001LL)); // Original source+600ms, never admission+600ms.
}

TEST(ProjectedRayMailbox, RepeatedEmptyTransferDoesNotRenewPendingPairOrOldSource) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::sourceTiming(map,.6);
  GridMapTestAccess::independentIngress(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet()));
  GridMapTestAccess::transfer(map);GridMapTestAccess::transfer(map,102200000000LL);
  EXPECT_EQ(GridMapTestAccess::writerReceipt(map,0),GridMapTestAccess::at(102000000000LL));
  GridMapTestAccess::integrate(map,102250000001LL,102.250000001);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,102200000000LL,2),102300000000LL));
  GridMapTestAccess::transfer(map,102900000000LL);
  GridMapTestAccess::integrate(map,102900000000LL,102.9);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U); // Fresh owner admission cannot renew expired acquisition.
}

TEST(ProjectedRayMailbox, MissingPeerDoesNotRenewAndNewRealPeerRestoresOriginalEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::sourceTiming(map,.6);
  GridMapTestAccess::independentIngress(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::receive(map,packet(sensor)));
  GridMapTestAccess::transfer(map);GridMapTestAccess::integrate(map);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,102500000000LL,2),102600000000LL));
  GridMapTestAccess::transfer(map,102600000000LL);GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);EXPECT_EQ(map.latestCloudStampNs(),0);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(1,102550000000LL,2),102650000000LL));
  GridMapTestAccess::transfer(map,102650000000LL);GridMapTestAccess::integrate(map,102650000000LL,102.65);
  EXPECT_EQ(map.latestCloudStampNs(),102500000000LL);
}

TEST(ProjectedRayMailbox, WriterAdmissionDoesNotHideFirstReceiptClockRollback) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(),102200000000LL));
  GridMapTestAccess::transfer(map,102100000000LL);
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
  EXPECT_EQ(GridMapTestAccess::drops(map)[0],1U);
}

TEST(ProjectedRayMailbox, WriterRechecksOriginalAlignmentWithoutReplacingValidPending) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::sourceTiming(map,.6);
  GridMapTestAccess::independentIngress(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet()));
  GridMapTestAccess::transfer(map);
  auto newer=packet(0,101950000000LL,2);newer.alignment_stamp=timeAt(101500000000LL);
  ASSERT_TRUE(GridMapTestAccess::receive(map,newer)); // All three source clocks fresh here.
  GridMapTestAccess::transfer(map,102150000000LL); // Alignment expired, acquisition still fresh.
  EXPECT_EQ(GridMapTestAccess::pending(map),1);
  EXPECT_EQ(GridMapTestAccess::writerReceipt(map,0),GridMapTestAccess::at(102000000000LL));
  GridMapTestAccess::integrate(map,102150000000LL,102.15);
  EXPECT_EQ(map.integratedRaySourceStamp(0),101900000000LL);
  EXPECT_EQ(GridMapTestAccess::drops(map)[0],1U);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,102100000000LL,3),102200000000LL));
  GridMapTestAccess::transfer(map,102200000000LL);GridMapTestAccess::integrate(map,102200000000LL,102.2);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
}

TEST(NativePipelineHealth, AcceptedSourcesAdvanceWithoutWriterOrBorrowedSnapshot) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const auto initial=map.nativePipelineHealth();
  ASSERT_TRUE(initial.context);EXPECT_EQ(initial.context,GridMapTestAccess::token(map));
  EXPECT_EQ(initial.source_ns[0],0);EXPECT_EQ(initial.fusion_sequence,0U);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0)));
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(1)));
  auto accepted=map.nativePipelineHealth();
  EXPECT_EQ(accepted.source_ns,(std::array<std::int64_t,2>{{101900000000LL,101900000000LL}}));
  EXPECT_EQ(accepted.fusion_sequence,0U);EXPECT_EQ(map.integratedRaySourceStamp(0),0);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,101950000000LL,2)));
  EXPECT_FALSE(GridMapTestAccess::receive(map,packet(1,101800000000LL,2)));
  accepted=map.nativePipelineHealth();
  EXPECT_EQ(accepted.source_ns[0],101950000000LL);EXPECT_EQ(accepted.source_ns[1],101900000000LL);
  EXPECT_EQ(accepted.fusion_sequence,0U);EXPECT_FALSE(accepted.clock_fault);
  EXPECT_GE(accepted.sampled_steady_ns,initial.sampled_steady_ns);
}

TEST(NativePipelineHealth, OnlyCompletedRealFusionRenewsProgressNotEmptyWatchdogOrPublication) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::receive(map,packet(sensor)));
  GridMapTestAccess::transfer(map);GridMapTestAccess::integrate(map);
  const auto fused=map.nativePipelineHealth();
  EXPECT_GT(fused.fusion_sequence,0U);EXPECT_EQ(fused.fusion_completed_steady_ns,102000000000LL);
  GridMapTestAccess::integrate(map,102100000000LL,102.1); // no rays
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL,true));
  EXPECT_EQ(map.nativePipelineHealth().fusion_sequence,fused.fusion_sequence);
  EXPECT_EQ(map.nativePipelineHealth().fusion_completed_steady_ns,fused.fusion_completed_steady_ns);
  ASSERT_TRUE(GridMapTestAccess::receive(map,packet(0,102100000000LL,2),102200000000LL));
  EXPECT_EQ(map.nativePipelineHealth().fusion_sequence,fused.fusion_sequence);
}

TEST(NativePipelineHealth, ResetRetiresAllHealthAndOldGenerationCannotRenewIt) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::independentIngress(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::receive(map,packet(sensor)));
  GridMapTestAccess::transfer(map);GridMapTestAccess::integrate(map);
  const auto old=map.nativePipelineHealth();ASSERT_GT(old.fusion_sequence,0U);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new-seed")));
  const auto reset=map.nativePipelineHealth();EXPECT_NE(reset.context,old.context);
  EXPECT_EQ(reset.source_ns,(std::array<std::int64_t,2>{{0,0}}));
  EXPECT_EQ(reset.fusion_sequence,0U);EXPECT_EQ(reset.fusion_completed_steady_ns,0);
  EXPECT_FALSE(GridMapTestAccess::receive(map,packet()));
  EXPECT_EQ(map.nativePipelineHealth().source_ns,reset.source_ns);
}

TEST(NativePipelineHealth, MailboxRejectsOldCompletionAndLatchesBackwardCompletionClock) {
  scan_planner::ProjectedRayMailbox mailbox;
  const auto old=std::make_shared<const scan_planner::ProjectedRayContext>();
  const auto next=std::make_shared<const scan_planner::ProjectedRayContext>();
  mailbox.setContext(old);mailbox.completedFusion(old,1,100);
  EXPECT_EQ(mailbox.pipelineHealth().fusion_sequence,1U);
  mailbox.setContext(next);mailbox.completedFusion(old,2,200);
  EXPECT_EQ(mailbox.pipelineHealth().fusion_sequence,0U);
  mailbox.completedFusion(next,1,300);mailbox.completedFusion(next,2,299);
  const auto fault=mailbox.pipelineHealth();EXPECT_TRUE(fault.clock_fault);
  EXPECT_EQ(fault.fusion_sequence,1U);EXPECT_EQ(fault.fusion_completed_steady_ns,300);
  mailbox.completedFusion(next,3,400);EXPECT_TRUE(mailbox.pipelineHealth().clock_fault);
  mailbox.setContext({});const auto closed=mailbox.pipelineHealth();
  EXPECT_FALSE(closed.context);EXPECT_FALSE(closed.clock_fault);EXPECT_EQ(closed.fusion_sequence,0U);
}

TEST(ProjectedRayIngressStorage, WarmDecodeUsesOnlyThreeBoundedBatchesAndInactiveSlotsDoNotFuse) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  std::set<const void*> warmed;
  for(unsigned round=0;round<12;++round) {
    const auto source=101900000000LL+100000000LL*round,now=source+100000000LL;
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source,round+1,1024),now,now*1e-9));
    if(round==1)warmed=GridMapTestAccess::ingressStorage(map);
    if(round>=1) {
      EXPECT_EQ(warmed.size(),6U); // Endpoints/origins in two slots plus one unpublished scratch.
      EXPECT_EQ(GridMapTestAccess::ingressStorage(map),warmed);
    }
    GridMapTestAccess::integrate(map,now,now*1e-9);
    EXPECT_EQ(GridMapTestAccess::pending(map),0);
    const auto counts=GridMapTestAccess::counts(map);
    EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now+1000000000LL,true));
    EXPECT_EQ(GridMapTestAccess::counts(map),counts);
    if(round>=1){EXPECT_EQ(GridMapTestAccess::ingressStorage(map),warmed);}
  }
}

TEST(ProjectedRayIngressStorage, MalformedLatePointAndOutOfOrderScanNeverReplaceAcceptedPending) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,101900000000LL,1,8)));
  auto bad=packet(0,101950000000LL,2,16,{.55,.05,.05},{.85,.05,.05});
  const float nan=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(bad.rays.data.data()+15*64,&nan,sizeof(nan));
  EXPECT_FALSE(GridMapTestAccess::accept(map,bad));
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101850000000LL,3,4)));
  EXPECT_EQ(GridMapTestAccess::pending(map),2);
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.integratedRaySourceStamp(0),101900000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,1.55),1.99); // Initial unknown -1.01 plus one real hit +3.
  EXPECT_LT(GridMapTestAccess::raw(map,.85),1.); // Bad scratch endpoint never becomes a hit.
}

TEST(ProjectedRayIngressStorage, ContextResetRetiresPendingAndRetainedStorageCannotReviveIt) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,101900000000LL,1,8)));
  const auto storage=GridMapTestAccess::ingressStorage(map);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new_seed")));
  EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(GridMapTestAccess::ingressStorage(map),storage);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102000000000LL,true));
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101950000000LL,2,8)));
  EXPECT_EQ(map.integratedRaySourceStamp(0),0);
  EXPECT_EQ(map.integratedRaySourceStamp(1),0);
  EXPECT_LT(GridMapTestAccess::raw(map,1.55),1.);
}

TEST(ProjectedRayDynamicOccupancy, SaturatedObstacleNeedsThirdNewMissAndWithdrawsInflationInSameTransaction) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::productionProbabilities(map);
  GridMapTestAccess::dynamicObstacleQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d origin(.05,.05,.05),obstacle(.95,.05,.05),beyond(1.55,.05,.05);
  const Eigen::Vector3d inflated_neighbor(1.05,.05,.05);
  std::uint64_t sequence=0;
  const auto fuse_pair=[&](const Eigen::Vector3d &endpoint) {
    ++sequence;
    const std::int64_t source=101900000000LL+200000000LL*sequence;
    const std::int64_t now=source+100000000LL;
    // Repeated beams and both scanners in one real fusion transaction must
    // not turn a single miss round into multiple probability updates.
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source,sequence,8,origin,endpoint),
          now,now*1e-9));
    GridMapTestAccess::integrate(map,now,now*1e-9);
  };
  // Pre-observe the entire ray and saturate the farther endpoint. Subsequent
  // revision changes then isolate removal of this obstacle, not newly observed
  // background cells or the first hit on the background wall.
  for(unsigned round=0;round<4;++round) fuse_pair(beyond);
  for(unsigned round=0;round<4;++round) fuse_pair(obstacle);
  const double max_odds=std::log(.98/.02),miss_odds=std::log(.30/.70);
  ASSERT_NEAR(GridMapTestAccess::raw(map,obstacle.x()),max_odds,1e-12);
  ASSERT_EQ(map.getOccupancy(obstacle),1);
  ASSERT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),1);
  ASSERT_EQ(map.getPlanningInflateOccupancy(inflated_neighbor,0.),1);
  ASSERT_EQ(GridMapTestAccess::inflationCount(map,obstacle),1);
  ASSERT_EQ(GridMapTestAccess::inflationCount(map,inflated_neighbor),1);
  const auto occupied_revision=map.occupancyRevision();
  int completed_transactions=0,callback_query=-1,callback_inflation=-1;
  std::uint64_t callback_revision=0;
  map.setCollisionUpdateCallback([&] {
    ++completed_transactions;
    callback_query=map.getPlanningInflateOccupancy(inflated_neighbor,0.);
    callback_inflation=GridMapTestAccess::inflationCount(map,inflated_neighbor);
    callback_revision=map.occupancyRevision();
  });
  for(unsigned miss_round=1;miss_round<=3;++miss_round) {
    fuse_pair(beyond);
    EXPECT_NEAR(GridMapTestAccess::raw(map,obstacle.x()),max_odds+miss_round*miss_odds,1e-12);
    const bool remains_occupied=miss_round<3;
    EXPECT_EQ(map.getOccupancy(obstacle),remains_occupied?1:0);
    EXPECT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),remains_occupied?1:0);
    EXPECT_EQ(map.getPlanningInflateOccupancy(inflated_neighbor,0.),remains_occupied?1:0);
    EXPECT_EQ(GridMapTestAccess::inflationCount(map,obstacle),remains_occupied?1:0);
    EXPECT_EQ(GridMapTestAccess::inflationCount(map,inflated_neighbor),remains_occupied?1:0);
    EXPECT_EQ(completed_transactions,static_cast<int>(miss_round));
    EXPECT_EQ(callback_query,remains_occupied?1:0);
    EXPECT_EQ(callback_inflation,remains_occupied?1:0);
    if(remains_occupied) EXPECT_EQ(map.occupancyRevision(),occupied_revision);
    else {
      EXPECT_GT(map.occupancyRevision(),occupied_revision);
      EXPECT_EQ(callback_revision,map.occupancyRevision());
    }
  }
  // No fabrication of fully observed-free evidence: .7941 is below p_occ=.8,
  // but is not the saturated-free state required by the optional strict policy.
  EXPECT_EQ(GridMapTestAccess::category(map,GridMapTestAccess::index(map,obstacle)),
      scan_planner::RawVoxelDiagnostic::Insufficient);
}

TEST(ProjectedRayDynamicOccupancy, ActualHitWinsConflictingRearMissAndRetainsRealObstacle) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::productionProbabilities(map);
  GridMapTestAccess::dynamicObstacleQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d origin(.05,.05,.05),obstacle(.95,.05,.05),beyond(1.55,.05,.05);
  for(std::uint64_t sequence=1;sequence<=10;++sequence) {
    const auto source=101900000000LL+200000000LL*sequence,now=source+100000000LL;
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,source,sequence,1,origin,obstacle),now,now*1e-9));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,source,sequence,64,origin,
        sequence<=4?obstacle:beyond),now,now*1e-9));
    GridMapTestAccess::integrate(map,now,now*1e-9);
    if(sequence>=4) {
      EXPECT_NEAR(GridMapTestAccess::raw(map,obstacle.x()),std::log(.98/.02),1e-12);
      EXPECT_EQ(map.getOccupancy(obstacle),1);
      EXPECT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),1);
      EXPECT_EQ(GridMapTestAccess::inflationCount(map,obstacle),1);
    }
  }
}

TEST(ProjectedRayDynamicOccupancy, NoCrossingObservationNeverClearsObstacleByWallClockAge) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::productionProbabilities(map);
  GridMapTestAccess::dynamicObstacleQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d origin(.05,.05,.05),obstacle(.95,.05,.05);
  for(std::uint64_t sequence=1;sequence<=4;++sequence) {
    const auto source=101900000000LL+200000000LL*sequence,now=source+100000000LL;
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source,sequence,1,origin,obstacle),now,now*1e-9));
    GridMapTestAccess::integrate(map,now,now*1e-9);
  }
  const double occupied_odds=GridMapTestAccess::raw(map,obstacle.x());
  const auto revision=map.occupancyRevision();
  const auto integrations=GridMapTestAccess::counts(map);
  // Advance only the real integration/watchdog clocks, never the observation
  // source. Expiry can invalidate leases but must not change map log odds.
  for(const auto now:{104000000000LL,110000000000LL,1000000000000LL}) {
    GridMapTestAccess::integrate(map,now,now*1e-9);
    EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,obstacle.x()),occupied_odds);
    EXPECT_EQ(map.occupancyRevision(),revision);
    EXPECT_EQ(GridMapTestAccess::counts(map),integrations);
    EXPECT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),1);
    EXPECT_EQ(GridMapTestAccess::inflationCount(map,obstacle),1);
  }
  // Fresh scans elsewhere also do not count as a miss through this cell.
  const std::int64_t source=1000100000000LL,now=source+100000000LL;
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source,5,1,{.05,1.05,.05},
        {1.55,1.05,.05}),now,now*1e-9));
  GridMapTestAccess::integrate(map,now,now*1e-9);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,obstacle.x()),occupied_odds);
  EXPECT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),1);
  EXPECT_EQ(GridMapTestAccess::inflationCount(map,obstacle),1);
}

TEST(ProjectedRayIntegration, InflationColumnsExactlyMatchPerVoxelKernelAcrossWrapClippingAndClearMask) {
  for(bool strict:{false,true})for(bool irregular:{false,true})for(bool tall:{false,true})
    for(const Eigen::Vector3i origin:{Eigen::Vector3i(0,0,0),Eigen::Vector3i(-27,35,-31),Eigen::Vector3i(43,-39,57)}) {
      GridMap optimized,legacy;GridMapTestAccess::configure(optimized,strict);
      GridMapTestAccess::configure(legacy,strict);
      GridMapTestAccess::inflationFixture(optimized,origin,irregular,tall);
      GridMapTestAccess::inflationFixture(legacy,origin,irregular,tall);
      std::vector<char> mask(64000,0);
      for(std::size_t at=0;at<mask.size();++at)mask[at]=(at%7==0||at%13==0);
      for(int step=0;step<140;++step) {
        const Eigen::Vector3i cell=origin+Eigen::Vector3i(
          (step*13)%49-24,(step*17)%49-24,(step*19)%49-24);
        const int delta=step%3==0?-1:1;
        const auto* selected_mask=step%2?&mask:nullptr;
        GridMapTestAccess::inflationUpdate(optimized,cell,delta,selected_mask,false);
        GridMapTestAccess::inflationUpdate(legacy,cell,delta,selected_mask,true);
        if(step%10==0)GridMapTestAccess::expectSameInflation(optimized,legacy);
      }
      GridMapTestAccess::expectSameInflation(optimized,legacy);
    }
}

TEST(ProjectedRayIntegration, CachedAddressLogOddsUpdateExactlyMatchesLegacyTransitions) {
  for(bool strict:{false,true}) {
    GridMap legacy,indexed;
    GridMapTestAccess::configure(legacy,strict);GridMapTestAccess::configure(indexed,strict);
    for(const Eigen::Vector3i &cell:{Eigen::Vector3i(-20,-20,-20),Eigen::Vector3i(-1,0,1),
        Eigen::Vector3i(19,19,19),Eigen::Vector3i(0,0,0)}) {
      for(double value:{-1.01,-1.,-1.,-.5,0.,-0.,0.,1.,1.01,2.,2.,1.,-.5,-1.,-1.01}) {
        GridMapTestAccess::legacyUpdate(legacy,cell,value);
        GridMapTestAccess::indexedUpdate(indexed,cell,value);
        GridMapTestAccess::expectSameEvidence(legacy,indexed);
        // Signed zero is preserved too, so a raw-byte map hash is unchanged.
        const auto a=GridMapTestAccess::buffers(legacy),b=GridMapTestAccess::buffers(indexed);
        ASSERT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(double)),0);
      }
    }
  }
}
TEST(ProjectedRayIntegration, SnapshotHookSeesCompletedBothSourceTransactionWithoutExtraTimer) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  int publications=0;std::array<std::int64_t,2> sources{};std::int64_t integrated=0;
  map.setCollisionUpdateCallback([&]{++publications;sources={map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1)};
    integrated=map.latestCloudStampNs();});
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0)));ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  EXPECT_EQ(publications,0);GridMapTestAccess::integrate(map);
  EXPECT_EQ(publications,1);EXPECT_EQ(sources[0],101900000000LL);EXPECT_EQ(sources[1],101900000000LL);
  EXPECT_EQ(integrated,101900000000LL);
  GridMapTestAccess::integrate(map,102500000000LL,102.5);
  EXPECT_EQ(publications,2);EXPECT_EQ(integrated,0); // Expiry is published too, never renewed.
}
TEST(ProjectedRayIntegration, PendingCompletedSnapshotDeliveredBeforeNextIntegration) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  scan_planner::CollisionSnapshotPool pool;
  bool pending=false;unsigned attempts=0,hooks=0;std::int64_t now=102000000000LL;
  const auto retry=[&]{if(pending){++attempts;if(pool.publishValidation(map,now,
      std::chrono::steady_clock::now()))pending=false;}};
  map.setCollisionUpdateCallback([&]{pending=true;retry();});
  std::int64_t delivered_before_third=0;std::uint64_t revision_before_third=0;
  map.setCollisionPublicationRetryCallback([&]{
    ++hooks;const auto completed=map.fusionTiming().sequence;retry();
    if(completed==2){auto previous=pool.borrowValidation();EXPECT_TRUE(previous);if(!previous)return false;
      delivered_before_third=previous->integratedRaySourceStamp(0);
      revision_before_third=previous->occupancyRevision();}
    return !pending;
  });
  const auto pair=[&](std::int64_t stamp,std::uint64_t sequence){
    for(unsigned sensor=0;sensor<2;++sensor)EXPECT_TRUE(GridMapTestAccess::accept(
      map,packet(sensor,stamp,sequence),now,now*1e-9));
  };
  pair(101900000000LL,1);ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(map,now,std::chrono::steady_clock::now()));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
  now=102100000000LL;pair(102000000000LL,2);ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pending);const auto completed_revision=map.occupancyRevision();
  EXPECT_EQ(head->integratedRaySourceStamp(0),101900000000LL);
  EXPECT_EQ(owner->integratedRaySourceStamp(0),101900000000LL);
  head.reset();owner.reset(); // Readers release before the next fusion, not another timer callback.
  now=102200000000LL;pair(102100000000LL,3);ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  EXPECT_EQ(delivered_before_third,102000000000LL);EXPECT_EQ(revision_before_third,completed_revision);
  EXPECT_EQ(hooks,3U);EXPECT_EQ(attempts,4U);EXPECT_FALSE(pending);
  auto current=pool.borrowValidation();ASSERT_TRUE(current);
  EXPECT_EQ(current->integratedRaySourceStamp(0),102100000000LL);
  EXPECT_EQ(current->integratedRaySourceStamp(1),102100000000LL);
  EXPECT_FALSE(current->integratedCloudFreshAt(102600000001LL)); // Original 500 ms source deadline.
  EXPECT_EQ(map.fusionTiming().sequence,3U);EXPECT_EQ(map.fusionTiming().begin_ns,now);
  // Early/no-data callbacks neither invoke a new fusion retry nor copy again.
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now+10000000LL,true));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now+100000000LL,true));
  EXPECT_EQ(hooks,3U);EXPECT_EQ(attempts,4U);
}
TEST(ProjectedRayIntegration, PendingSnapshotBusyReadersDoNotBlockOrAcquireExtraSlots) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));scan_planner::CollisionSnapshotPool pool;
  bool pending=false;unsigned attempts=0;std::int64_t now=102000000000LL;
  const auto retry=[&]{if(pending){++attempts;if(pool.publishValidation(map,now,
      std::chrono::steady_clock::now()))pending=false;}};
  map.setCollisionUpdateCallback([&]{pending=true;retry();});
  map.setCollisionPublicationRetryCallback([&]{retry();return !pending;});
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(map,now,std::chrono::steady_clock::now()));
  ASSERT_TRUE(pool.publishSolver(map,now,std::chrono::steady_clock::now()));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation(),solver=pool.borrowSolver();
  ASSERT_TRUE(head&&owner&&solver);
  const auto publication=pool.readerDiagnostics().latest_publication_sequence;
  for(unsigned round=2;round<=4;++round){
    now=102000000000LL+(round-1)*100000000LL;
    for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(
      map,packet(sensor,now-100000000LL,round),now,now*1e-9));
    if(round>2) {
      const auto sequence=map.fusionTiming().sequence;
      const auto start=GridMapTestAccess::integrationStart(map);
      EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now));
      EXPECT_EQ(map.fusionTiming().sequence,sequence);
      EXPECT_EQ(GridMapTestAccess::integrationStart(map),start);
      EXPECT_EQ(GridMapTestAccess::pending(map),2);
    }
    ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now)); // Second call must run even with every reader busy.
    EXPECT_TRUE(pending);EXPECT_EQ(pool.readerDiagnostics().latest_publication_sequence,publication);
    EXPECT_EQ(head->integratedRaySourceStamp(0),101900000000LL);
    EXPECT_EQ(owner->integratedRaySourceStamp(0),101900000000LL);
    EXPECT_EQ(solver->integratedRaySourceStamp(0),101900000000LL);
    EXPECT_FALSE(pool.borrowValidation());EXPECT_FALSE(pool.borrowOwnerValidation());
  }
  EXPECT_EQ(attempts,8U); // One attempt per hook/completion plus two single yields, never a wait loop.
  EXPECT_EQ(map.fusionTiming().sequence,4U);EXPECT_EQ(map.integratedRaySourceStamp(0),102200000000LL);
}
TEST(ProjectedRayIntegration, PendingPublicationAfterContextResetCannotRepublishOldEvidence) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));scan_planner::CollisionSnapshotPool pool;
  bool pending=false;std::int64_t now=102000000000LL;
  const auto retry=[&]{if(pending&&pool.publishValidation(map,now,
      std::chrono::steady_clock::now()))pending=false;};
  map.setCollisionUpdateCallback([&]{pending=true;retry();});
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(map,now,std::chrono::steady_clock::now()));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"reset-seed")));ASSERT_TRUE(pending);
  head.reset();owner.reset();EXPECT_FALSE(pool.borrowValidation()); // The pool already saw the reset identity.
  unsigned before=0;
  map.setCollisionPublicationRetryCallback([&]{retry();auto reset=pool.borrowValidation();EXPECT_TRUE(reset);if(!reset)return false;
    ++before;EXPECT_EQ(reset->localizationContextSequence(),2U);
    EXPECT_EQ(reset->integratedRaySourceStamp(0),0);EXPECT_EQ(reset->latestCloudStampNs(),0);
    return !pending;
  });
  now=102200000000LL;
  for(unsigned sensor=0;sensor<2;++sensor){auto next=packet(sensor,102100000000LL,2);
    next.epoch=2;next.context_sequence=2;next.seed_id="reset-seed";next.barrier_ns=100000000002ULL;
    ASSERT_TRUE(GridMapTestAccess::accept(map,next,now,now*1e-9));}
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));EXPECT_EQ(before,1U);
  auto current=pool.borrowValidation();ASSERT_TRUE(current);EXPECT_EQ(current->localizationContextSequence(),2U);
  EXPECT_EQ(current->integratedRaySourceStamp(0),102100000000LL);EXPECT_FALSE(pending);
}
TEST(ProjectedRayIntegration, BusyPublicationYieldsOnceSoReleasedFastHeadPrecedesNextFusion) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));scan_planner::CollisionSnapshotPool pool;
  bool pending=false;unsigned attempts=0;std::int64_t now=102000000000LL;
  const auto retry=[&]{if(pending){++attempts;if(pool.publishValidation(map,now,
      std::chrono::steady_clock::now()))pending=false;}return !pending;};
  map.setCollisionUpdateCallback([&]{pending=true;retry();});
  map.setCollisionPublicationRetryCallback(retry);
  const auto pair=[&](std::int64_t stamp,std::uint64_t sequence){
    for(unsigned sensor=0;sensor<2;++sensor)EXPECT_TRUE(GridMapTestAccess::accept(
      map,packet(sensor,stamp,sequence),now,now*1e-9));
  };
  pair(101900000000LL,1);ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pool.refreshOwnerReplica(map,now,std::chrono::steady_clock::now()));
  auto head=pool.borrowValidation(),owner=pool.borrowOwnerValidation();ASSERT_TRUE(head&&owner);
  now=102100000000LL;pair(102000000000LL,2);ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  ASSERT_TRUE(pending);const auto completed=map.fusionTiming();
  const auto integrated=GridMapTestAccess::integrationStart(map);
  const auto counts=GridMapTestAccess::counts(map);
  now=102200000000LL;pair(102100000000LL,3);
  const auto received=GridMapTestAccess::acceptedClock(map);
  const auto drops=GridMapTestAccess::drops(map);
  ASSERT_FALSE(GridMapTestAccess::eventFusion(map,now));
  EXPECT_EQ(map.fusionTiming().sequence,completed.sequence);
  EXPECT_EQ(map.fusionTiming().begin_ns,completed.begin_ns);
  EXPECT_EQ(GridMapTestAccess::counts(map),counts);EXPECT_EQ(GridMapTestAccess::pending(map),2);
  EXPECT_EQ(GridMapTestAccess::integrationStart(map),integrated);
  EXPECT_EQ(GridMapTestAccess::acceptedClock(map),received);EXPECT_EQ(GridMapTestAccess::drops(map),drops);
  // Existing 10 ms writer callback, after only the fast reader releases. The
  // slower owner remains pinned; no ROS executor/timer or sleeping in this test.
  head.reset();EXPECT_FALSE(pool.borrowValidation());now+=10000000LL;ASSERT_TRUE(retry());
  auto delivered=pool.borrowValidation();ASSERT_TRUE(delivered);
  EXPECT_EQ(delivered->fusionTiming().sequence,completed.sequence);
  EXPECT_EQ(delivered->integratedRaySourceStamp(0),102000000000LL);
  EXPECT_EQ(delivered->integratedRaySourceStamp(1),102000000000LL);
  EXPECT_EQ(owner->integratedRaySourceStamp(0),101900000000LL);
  EXPECT_FALSE(delivered->integratedCloudFreshAt(102500000001LL)); // Never renew source age while queued.
  delivered.reset();
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now));
  EXPECT_EQ(map.fusionTiming().sequence,completed.sequence+1);
  EXPECT_EQ(map.fusionTiming().begin_ns,now);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),102100000000LL);
  EXPECT_EQ(attempts,5U);EXPECT_FALSE(pending);
  // Start-to-start rate cap is based on the actual resumed integration, not
  // the skipped invocation or snapshot copy.
  now=102300000000LL;pair(102200000000LL,4);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now));
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102310000000LL));
}
TEST(ProjectedRayIntegration, PublicationYieldDoesNotRefreshPendingSourcesPastTheirDeadline) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));unsigned retries=0;
  map.setCollisionPublicationRetryCallback([&]{++retries;return false;});
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102000000000LL));
  EXPECT_EQ(map.fusionTiming().sequence,0U);EXPECT_EQ(GridMapTestAccess::pending(map),2);
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102500000001LL)); // Busy again: input still progresses.
  EXPECT_EQ(retries,2U);EXPECT_EQ(map.fusionTiming().sequence,1U);
  EXPECT_EQ(map.latestCloudStampNs(),0);EXPECT_EQ(map.integratedRaySourceStamp(0),0);
  EXPECT_EQ(map.integratedRaySourceStamp(1),0);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_FALSE(map.integratedCloudFreshAt(102500000001LL));
}
TEST(ProjectedRayIntegration,NewDualSourceEventFusesImmediatelyButNeverExceedsFiveHz) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  int snapshots=0;map.setCollisionUpdateCallback([&]{++snapshots;});
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0)));EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102000000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  EXPECT_EQ(snapshots,1); // No 200 ms timer wait after the complete packet pair.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
    packet(sensor,102000000000LL,2),102100000000LL,102.1));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102100000000LL));
  // Both observations are newer than the integrated pair. Do not require
  // acquisition advance >=180 ms on top of the already elapsed 200 ms cap.
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102200000000LL,true));
  EXPECT_EQ(snapshots,2);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102000000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),102000000000LL);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,3),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,102100000000LL,3),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102399999999LL));
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102400000000LL));
  EXPECT_EQ(snapshots,3);EXPECT_EQ(GridMapTestAccess::counts(map)[0],3U);
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102410000000LL,true));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102600000000LL,true));
  EXPECT_EQ(snapshots,3);EXPECT_EQ(map.latestCloudStampNs(),102100000000LL);
  EXPECT_FALSE(map.integratedCloudFreshAt(102600000001LL)); // No-data watchdog never refreshes a source.
}
TEST(ProjectedRayIntegration,MissingRearWatchdogCanAddFrontHitsWithoutRefreshingRear) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,2),102200000000LL,102.2));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL));
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102310000000LL,true));
  EXPECT_EQ(map.integratedRaySourceStamp(0),102100000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
  EXPECT_FALSE(map.integratedCloudFreshAt(102410000000LL));
}

TEST(ProjectedRayIntegration, IntegralRateAndWatchdogBoundariesShareTheTimerDuration) {
  struct Timing {double seconds;std::int64_t period_ns,watchdog_ns;};
  for(const auto timing:{Timing{.05,50000000,75000000},
                        Timing{.1,100000000,150000000},
                        Timing{.2,200000000,300000000},
                        Timing{1./7.,142857143,214285715},
                        Timing{.100000001,100000001,150000002}}) {
    SCOPED_TRACE(timing.seconds);
    for(const bool both_sources:{false,true}) {
      SCOPED_TRACE(both_sources);
      GridMap map;GridMapTestAccess::configure(map,false);
      GridMapTestAccess::sourceTiming(map,.6);
      GridMapTestAccess::fusionPeriod(map,timing.seconds);
      ASSERT_TRUE(map.applyLocalizationContext(context()));
      constexpr std::int64_t first=102000000000LL;
      for(unsigned sensor=0;sensor<2;++sensor)
        ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,first-100000000LL,1),first,first*1e-9));
      ASSERT_TRUE(GridMapTestAccess::eventFusion(map,first));
      const auto due=first+(both_sources?timing.period_ns:timing.watchdog_ns);
      const auto accepted=first+timing.period_ns-1000000LL;
      for(unsigned sensor=0;sensor<(both_sources?2U:1U);++sensor)
        ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,first,2),accepted,accepted*1e-9));
      EXPECT_EQ(GridMapTestAccess::nextWake(map,first+timing.period_ns-7000000LL),7000000LL);
      EXPECT_FALSE(GridMapTestAccess::eventFusion(map,first-1,true));
      EXPECT_FALSE(GridMapTestAccess::eventFusion(map,due-1,true));
      ASSERT_TRUE(GridMapTestAccess::eventFusion(map,due,true));
      EXPECT_EQ(map.fusionTiming().begin_ns,due);
      EXPECT_EQ(map.integratedRaySourceStamp(0),first);
      EXPECT_EQ(map.integratedRaySourceStamp(1),both_sources?first:first-100000000LL);
      EXPECT_FALSE(GridMapTestAccess::eventFusion(map,due+1,true));
      EXPECT_EQ(map.fusionTiming().sequence,2U);
      EXPECT_FALSE(map.integratedCloudFreshAt((both_sources?first:first-100000000LL)+600000001LL));
    }
  }
}

TEST(ProjectedRayIntegration, ExactTenHzDualSourceCadenceNeverSkipsOrCatchesUp) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::sourceTiming(map,.6);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  constexpr std::int64_t first=102000000000LL,period=100000000LL;
  for(unsigned frame=0;frame<20;++frame) {
    const auto now=first+frame*period,source=now-period;
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source,frame+1),now,now*1e-9));
    if(frame)EXPECT_FALSE(GridMapTestAccess::eventFusion(map,now-1,true));
    ASSERT_TRUE(GridMapTestAccess::eventFusion(map,now,frame%2==0));
    EXPECT_EQ(map.fusionTiming().begin_ns,now);
    EXPECT_EQ(map.fusionTiming().sequence,frame+1);
    EXPECT_EQ(map.integratedRaySourceStamp(0),source);
    EXPECT_EQ(map.integratedRaySourceStamp(1),source);
  }
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],20U);
  EXPECT_EQ(GridMapTestAccess::counts(map)[1],20U);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,first+30*period,true));
}

TEST(ProjectedRayIntegration, UnrepresentablePeriodsDoNotConvertToIntegralDeadlines) {
  for(const double period:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),1e50}) {
    GridMap map;GridMapTestAccess::configure(map);
    GridMapTestAccess::fusionPeriod(map,period);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
    EXPECT_EQ(GridMapTestAccess::nextWake(map,102000000000LL),20000000LL);
    EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102000000000LL,true));
    EXPECT_EQ(GridMapTestAccess::pending(map),2);
  }
}

TEST(ProjectedRayIntegration,DeadlineWakeTargetsDueInsteadOfRoundingToPollingPhase) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102000000000LL),20000000);
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102180000000LL),20000000);
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102193000000LL),7000000);
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102199900000LL),1000000); // Bounded wake, never busy-spin.
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102200000000LL),20000000); // Due without data is not a poll storm.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102050000000LL,2),102150000000LL,102.15));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102207000000LL,true)); // A late executor callback.
  EXPECT_EQ(GridMapTestAccess::nextWake(map,102393000000LL),14000000); // Relative to actual late start.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102200000000LL,3),102300000000LL,102.3));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102400000000LL,true)); // No 193 ms phase catch-up.
  EXPECT_TRUE(GridMapTestAccess::eventFusion(map,102407000000LL,true));
  const auto before=map.fusionTiming();
  EXPECT_EQ(before.sequence,3U);EXPECT_EQ(before.begin_ns,102407000000LL);
  EXPECT_EQ(before.end_ns,102407000000LL); // Injected passive clock; no invented runtime duration.
  EXPECT_EQ(before.source_stamps[0],102200000000LL);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102607000000LL,true));
  EXPECT_EQ(map.fusionTiming().sequence,before.sequence);
  EXPECT_EQ(map.fusionTiming().source_stamps,before.source_stamps);
}
TEST(ProjectedRayIntegration, ConfiguredTenHzPeriodUsesActualStartNotCatchupAndNeverRenewsSilentSources) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102020000000LL,2),102040000000LL,102.04));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102099999999LL,true));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102117000000LL,true)); // 17 ms late, no invented earlier start.
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102120000000LL,3),102180000000LL,102.18));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102200000000LL,true));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102216999999LL,true));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102217000000LL,true));
  EXPECT_EQ(map.fusionTiming().begin_ns,102217000000LL);EXPECT_EQ(map.fusionTiming().sequence,3U);
  const auto timing=map.fusionTiming();const auto counts=GridMapTestAccess::counts(map);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102800000000LL,true));
  EXPECT_EQ(GridMapTestAccess::counts(map),counts);EXPECT_EQ(map.fusionTiming().source_stamps,timing.source_stamps);
  EXPECT_FALSE(map.integratedCloudFreshAt(102620000001LL)); // Original 500 ms TTL, not 10 Hz receipt age.
}
TEST(ProjectedRayIntegration, ConfiguredTenHzWatchdogPartialKeepsPeerOriginalStampAndFiveHundredMsExpiry) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::fusionPeriod(map,.1);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102000000000LL));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102000000000LL,2),102100000000LL,102.1));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102100000000LL,false));
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102149000000LL,true));
  ASSERT_TRUE(GridMapTestAccess::eventFusion(map,102151000000LL,true));
  EXPECT_EQ(map.integratedRaySourceStamp(0),102000000000LL);
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],2U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  EXPECT_FALSE(GridMapTestAccess::eventFusion(map,102500000000LL,true));
  EXPECT_FALSE(map.integratedCloudFreshAt(102400000001LL));
  EXPECT_EQ(map.integratedRaySourceStamp(1),101900000000LL);
}

TEST(ProjectedRayIntegration, SaturatedFreeNoOpStillRefreshesOnlyActualSourceEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  std::uint64_t saturated_revision=0;
  for(unsigned sequence=1;sequence<=8;++sequence) {
    const std::int64_t stamp=101700000000LL+sequence*20000000LL;
    for(unsigned sensor=0;sensor<2;++sensor)
      ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,stamp,sequence,128)));
    GridMapTestAccess::integrate(map);
    EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,.65),-1.);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{6,0,0}),stamp);
    EXPECT_EQ(GridMapTestAccess::counts(map)[0],sequence);
    EXPECT_EQ(GridMapTestAccess::counts(map)[1],sequence);
    EXPECT_DOUBLE_EQ(map.latestCloudStamp(),stamp*1e-9);
    if(sequence==1) saturated_revision=map.occupancyRevision();
    else EXPECT_EQ(map.occupancyRevision(),saturated_revision);
  }
}

TEST(ProjectedRayIngress, DrainIsBoundedEvenWhenProducerNeverEmpties) {
  unsigned takes=0,validations=0;
  const auto result=scan_planner::drainProjectedRayIngress<int>(
      [&takes](int &message) {message=++takes;return true;},
      [&validations](int message) {++validations;return message%2==0;});
  EXPECT_EQ(result.taken,5U);EXPECT_EQ(result.accepted,2U);
  EXPECT_EQ(takes,5U);EXPECT_EQ(validations,5U);
}

TEST(ProjectedRayIngress, SlowSourceSurvivesEveryPositionInMixedFiveMessageWindow) {
  for(unsigned rear_position=0;rear_position<5;++rear_position) {
    GridMap map;GridMapTestAccess::configure(map,false);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue;
    std::int64_t front_stamp=0;
    for(unsigned i=0;i<5;++i) {
      if(i==rear_position) queue.push_back(packet(1,101850000000LL,1));
      else {front_stamp=101700000000LL+i*40000000LL;queue.push_back(packet(0,front_stamp,i+1));}
    }
    std::size_t cursor=0;
    const auto result=GridMapTestAccess::drain(map,queue,cursor);
    ASSERT_EQ(result.taken,5U);ASSERT_EQ(result.accepted,5U);ASSERT_EQ(cursor,5U);
    ASSERT_EQ(GridMapTestAccess::pending(map),2);
    EXPECT_EQ(GridMapTestAccess::drops(map)[0],3U); // Whole older front batches only.
    EXPECT_EQ(GridMapTestAccess::drops(map)[1],0U);
    GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
    EXPECT_DOUBLE_EQ(map.latestCloudStamp(),std::min(front_stamp,std::int64_t{101850000000LL})*1e-9);
    EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],102000000000LL);
    EXPECT_EQ(GridMapTestAccess::integrationStart(map)[1],102000000000LL);
    ASSERT_TRUE(map.applyLocalizationContext(context(2,2)));
    EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],0);
    EXPECT_EQ(GridMapTestAccess::integrationStart(map)[1],0);
  }
}

TEST(ProjectedRayIngress, QueuedStaleSourceCannotRenewLeaseWhileOtherSourceContinues) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,101600000000LL,1)));
  GridMapTestAccess::integrate(map);
  ASSERT_GT(map.latestCloudStamp(),0.);
  const std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue{
    packet(1,101700000000LL,2),packet(0,102500000000LL,2),packet(0,102550000000LL,3)};
  std::size_t cursor=0;
  const auto result=GridMapTestAccess::drain(map,queue,cursor,102600000000LL,102.6);
  EXPECT_EQ(result.taken,3U);EXPECT_EQ(result.accepted,2U);
  EXPECT_EQ(GridMapTestAccess::acceptedClock(map)[1],102000000000LL); // Not refreshed by rejected take.
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],2U);EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),0.);
}

TEST(ProjectedRayIngress, CompleteSelectedBatchesMatchNativeBuffersAndRejectPartialMalformedBatch) {
  GridMap actual,reference;
  GridMapTestAccess::configure(actual,false);GridMapTestAccess::configure(reference,false);
  ASSERT_TRUE(actual.applyLocalizationContext(context()));ASSERT_TRUE(reference.applyLocalizationContext(context()));
  auto front=packet(0,101900000000LL,3,3);
  auto rear=packet(1,101850000000LL,1,3,{-.55,.05,.05},{-1.55,.05,.05});
  for(auto *message:{&front,&rear}) for(unsigned i=0;i<3;++i) {
    const float lane=.05f+i*.3f;
    std::memcpy(message->rays.data.data()+64*i+4,&lane,4);
    std::memcpy(message->rays.data.data()+64*i+20,&lane,4);
  }
  auto malformed=packet(0,101950000000LL,4,3);
  const float nan=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(malformed.rays.data.data()+64*2,&nan,4); // Last ray invalid: accept none.
  const std::vector<d1max_planning_interfaces::msg::ProjectedRays> queue{
    packet(0,101700000000LL,1),rear,packet(0,101800000000LL,2),front,malformed};
  std::size_t cursor=0;
  const auto result=GridMapTestAccess::drain(actual,queue,cursor);
  EXPECT_EQ(result.taken,5U);EXPECT_EQ(result.accepted,4U);
  ASSERT_TRUE(GridMapTestAccess::accept(reference,front));ASSERT_TRUE(GridMapTestAccess::accept(reference,rear));
  GridMapTestAccess::integrate(actual);GridMapTestAccess::integrate(reference);
  EXPECT_EQ(GridMapTestAccess::integratedPoints(actual),6);
  GridMapTestAccess::expectSameEvidence(actual,reference);
  GridMapTestAccess::expectSameRayEvidence(actual,reference);
  for(unsigned i=0;i<3;++i) {
    EXPECT_GT(GridMapTestAccess::raw(actual,1.55,.05+i*.3),1.);
    EXPECT_GT(GridMapTestAccess::raw(actual,-1.55,.05+i*.3),1.);
  }
}

TEST(ProjectedRaysPreviewTiming, ExplicitOfficialProjectedPreviewContractOnly) {
  using scan_planner::validateCloudPoseTiming;
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.5,false,true,true));
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.5,false,false,true));
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.75,true,false,true));
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,false,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,true,true,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.5,true,true,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.75,true,false,false),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,.750001,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.250001,.75,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,NAN,true,false,true),std::invalid_argument);
  EXPECT_THROW(validateCloudPoseTiming(.25,0.,true,false,true),std::invalid_argument);
}

TEST(ProjectedRaysTiming, OfficialExecutionUsesConfigured600msAndExpiresOriginalSources) {
  using scan_planner::validateCloudPoseTiming;
  EXPECT_NO_THROW(validateCloudPoseTiming(.25,.6,false,false,true));
  EXPECT_THROW(validateCloudPoseTiming(.25,.600001,false,false,true),std::invalid_argument);
  // Historical strict observed-free certificates keep their 500 ms ceiling.
  EXPECT_THROW(validateCloudPoseTiming(.25,.6,false,true,true),std::invalid_argument);
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::sourceTiming(map,.6);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  constexpr std::int64_t source=101900000000LL;
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source),source+440000000LL,102.34));
  GridMapTestAccess::integrate(map,source+510000000LL,102.41);
  EXPECT_EQ(map.integratedRaySourceStamp(0),source);
  EXPECT_EQ(map.integratedRaySourceStamp(1),source);
  EXPECT_TRUE(map.integratedCloudFreshAt(source+557000000LL));
  EXPECT_TRUE(map.integratedCloudFreshAt(source+600000000LL));
  EXPECT_FALSE(map.integratedCloudFreshAt(source+600001000LL));
  // A continuing peer does not renew the missing source or its expiry.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,source+200000000LL,2),source+610000000LL,102.51));
  GridMapTestAccess::integrate(map,source+610000000LL,102.51);
  EXPECT_EQ(map.integratedRaySourceStamp(1),source);
  EXPECT_FALSE(map.integratedCloudFreshAt(source+610000000LL));
}

TEST(ProjectedRaysTiming, EpochNanosecondsKeepExactConfiguredExpiryBoundary) {
  constexpr std::int64_t source=1792070886123456789LL;
  for(double bound:{.5,.6,.75}) {
    const auto limit=static_cast<std::int64_t>(std::llround(bound*1e9));
    EXPECT_TRUE(scan_planner::rayStampFresh(source,source+limit,bound));
    EXPECT_FALSE(scan_planner::rayStampFresh(source,source+limit+1,bound));
  }
  EXPECT_TRUE(scan_planner::rayStampFresh(source,source-100000000LL,.6));
  EXPECT_FALSE(scan_planner::rayStampFresh(source,source-100000001LL,.6));
}

TEST(ProjectedRaysPreviewTiming, OriginalSourceClockAcceptedThrough750msThenRevoked) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::previewTiming(map,.75);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  constexpr std::int64_t source=101900000000LL;
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor,source),source+750000000LL,102.65));
  GridMapTestAccess::integrate(map,source+750000000LL,102.65);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),101.9); // Never restamped to receipt/integration.
  EXPECT_TRUE(map.integratedCloudFreshAt(source+600000000LL));
  EXPECT_TRUE(map.integratedCloudFreshAt(source+750000000LL));
  EXPECT_FALSE(map.integratedCloudFreshAt(source+750001000LL));
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],1U);
  EXPECT_EQ(GridMapTestAccess::counts(map)[1],1U);
  GridMapTestAccess::integrate(map,source+750001000LL,102.650001);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),0.);
  EXPECT_FALSE(map.integratedCloudFreshAt(source+750001000LL));
  GridMap rejected;GridMapTestAccess::configure(rejected,false);
  GridMapTestAccess::previewTiming(rejected,.75);
  ASSERT_TRUE(rejected.applyLocalizationContext(context()));
  EXPECT_FALSE(GridMapTestAccess::accept(rejected,packet(0,source),source+750001000LL,102.650001));
  EXPECT_EQ(GridMapTestAccess::counts(rejected)[0],0U);
  GridMap normal;GridMapTestAccess::configure(normal,false);
  ASSERT_TRUE(normal.applyLocalizationContext(context()));
  EXPECT_FALSE(GridMapTestAccess::accept(normal,packet(0,source),source+500001000LL,102.400001));
  for(unsigned sensor=0;sensor<2;++sensor)
    ASSERT_TRUE(GridMapTestAccess::accept(normal,packet(sensor,source),source+100000000LL,102.0));
  GridMapTestAccess::integrate(normal,source+100000000LL,102.0);
  EXPECT_TRUE(normal.integratedCloudFreshAt(source+500000000LL));
  EXPECT_FALSE(normal.integratedCloudFreshAt(source+500001000LL));
}

static d1max_planning_interfaces::msg::ProjectedRays withSourceFields(
    d1max_planning_interfaces::msg::ProjectedRays out) {
  using F=sensor_msgs::msg::PointField;
  struct Field {const char *name;unsigned offset;unsigned type;};
  for(const auto &field:std::initializer_list<Field>{{"ring",30,F::UINT16},
      {"offset_time",32,F::UINT32},{"source_index",36,F::UINT32},
      {"timestamp",40,F::FLOAT64},{"source_timestamp",48,F::FLOAT64},{"raw_timestamp",56,F::FLOAT64}}) {
    F f;f.name=field.name;f.offset=field.offset;f.datatype=field.type;f.count=1;
    out.rays.fields.push_back(f);
  }
  for(unsigned i=0;i<out.rays.width;++i) {
    auto *bytes=out.rays.data.data()+64*i;
    const std::uint16_t ring=22;
    const std::uint32_t offset=100+i,index=12000+i;
    const double timestamp=101.9000001+i*1e-9,source=101.4+i*1e-9,raw=101.3+i*1e-9;
    std::memcpy(bytes+30,&ring,2);std::memcpy(bytes+32,&offset,4);std::memcpy(bytes+36,&index,4);
    std::memcpy(bytes+40,&timestamp,8);std::memcpy(bytes+48,&source,8);std::memcpy(bytes+56,&raw,8);
  }
  return out;
}

TEST(ProjectedRaysOfficialScan, UnknownQueriesDoNotInventObservedFreeEvidence) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::productionGeometry(official);GridMapTestAccess::productionGeometry(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  const auto raw=GridMapTestAccess::buffers(official);
  for(double yaw:{0.,.63,1.57}) {
    EXPECT_EQ(official.getInflateOccupancy({.05,.05,.05},yaw),0);
    EXPECT_EQ(strict.getInflateOccupancy({.05,.05,.05},yaw),2);
  }
  EXPECT_EQ(GridMapTestAccess::buffers(official),raw);
  EXPECT_EQ(GridMapTestAccess::category(official,{0,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  EXPECT_EQ(GridMapTestAccess::freeStamp(official,{0,0,0}),0);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),0.);
}

TEST(ProjectedRaysOfficialScan, RealOccupiedInflationBlocksBothCentersAtRotatedHeadings) {
  for(double yaw:{0.,1.5707963267948966}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,
        {.05,.25,.05},{.55,.25,.05})));
    GridMapTestAccess::integrate(map);
    ASSERT_EQ(GridMapTestAccess::category(map,{5,2,0}),scan_planner::RawVoxelDiagnostic::Occupied);
    // The center itself has no hit. An actual hit within the inflation radius
    // must block either member of the official two-center query.
    EXPECT_EQ(GridMapTestAccess::category(map,{5,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
    const Eigen::Vector3d center(.55,.05,.05),heading(std::cos(yaw),std::sin(yaw),0.);
    for(int side:{-1,1}) EXPECT_EQ(map.getInflateOccupancy(center+side*.2*heading,yaw),1);
    EXPECT_EQ(map.getInflateOccupancy({-.75,-.75,.05},yaw),0);
  }
}

TEST(ProjectedRaysOfficialScan, OutsideAndNonfiniteQueriesRemainBlocked) {
  GridMap map;GridMapTestAccess::configure(map,false);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),-1); // Front center outside.
  EXPECT_EQ(map.getInflateOccupancy({-1.85,.05,.05},0.),-1); // Rear center outside.
  EXPECT_EQ(map.getInflateOccupancy({.05,1.85,.05},1.5707963267948966),-1);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,2.05},0.),-1);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(map.getInflateOccupancy({nan,.05,.05},0.),-1);
  EXPECT_EQ(map.getInflateOccupancy({.05,.05,.05},nan),-1);
}

TEST(ProjectedRaysOfficialScan, SamePerPointOriginsAndSourceTimesProduceIdenticalEvidence) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::diagnostics(official);GridMapTestAccess::diagnostics(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) {
    const double sign=sensor?-1.:1.;const auto stamp=101900000000LL+sensor*50000000LL;
    auto rays=withSourceFields(packet(sensor,stamp,1,2,{sign*.55,.05,.05},{sign*1.55,.05,.05}));
    // Distinct origins within each physical source must survive decoding too.
    const float lane=.75f;
    std::memcpy(rays.rays.data.data()+64+4,&lane,4);
    std::memcpy(rays.rays.data.data()+64+20,&lane,4);
    ASSERT_TRUE(GridMapTestAccess::accept(official,rays));ASSERT_TRUE(GridMapTestAccess::accept(strict,rays));
  }
  GridMapTestAccess::integrate(official);GridMapTestAccess::integrate(strict);
  GridMapTestAccess::expectSameRayEvidence(official,strict);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),101.9);
  EXPECT_EQ(GridMapTestAccess::counts(official)[0],1U);EXPECT_EQ(GridMapTestAccess::counts(official)[1],1U);
  for(unsigned sensor=0;sensor<2;++sensor) {
    const double sign=sensor?-1.:1.;const auto stamp=101900000000LL+sensor*50000000LL;
    const Eigen::Vector3d traversed(sign*.95,.75,.05),end(sign*1.55,.75,.05);
    EXPECT_EQ(GridMapTestAccess::freeStamp(official,GridMapTestAccess::index(official,traversed)),stamp);
    const auto *history=official.nearFieldDiagnostics().find(GridMapTestAccess::index(official,end));
    ASSERT_NE(history,nullptr);ASSERT_TRUE(history->last[sensor*2]);
    const auto &w=*history->last[sensor*2];
    EXPECT_TRUE(w.origin.isApprox(Eigen::Vector3d(sign*.55,.75,.05),1e-6));
    EXPECT_TRUE(w.endpoint.isApprox(end,1e-6));
    EXPECT_EQ(w.metadata.sensor_id,sensor);EXPECT_EQ(w.metadata.scan_stamp_ns,stamp);
    EXPECT_EQ(w.metadata.offset_time_ns,101U);EXPECT_EQ(w.metadata.source_index,12001U);
    EXPECT_EQ(w.metadata.integration_ns,102000000000LL);
    EXPECT_DOUBLE_EQ(w.metadata.source_timestamp,101.4+1e-9);
    EXPECT_DOUBLE_EQ(w.metadata.raw_timestamp,101.3+1e-9);
  }
  for(const Eigen::Vector3d &position:{Eigen::Vector3d(.05,.75,.05),Eigen::Vector3d(1.75,.75,.05)})
    EXPECT_EQ(GridMapTestAccess::category(official,GridMapTestAccess::index(official,position)),
        scan_planner::RawVoxelDiagnostic::NeverObserved);
}

TEST(ProjectedRaysOfficialScan, FreeExpiryDoesNotChangeOfficialQueryOrRenewSourceLease) {
  GridMap official,strict;
  GridMapTestAccess::configure(official,false);GridMapTestAccess::configure(strict);
  GridMapTestAccess::centerOnlyOfficialGeometry(official);GridMapTestAccess::pointQueryGeometry(strict);
  ASSERT_TRUE(official.applyLocalizationContext(context()));ASSERT_TRUE(strict.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) {
    ASSERT_TRUE(GridMapTestAccess::accept(official,packet(sensor)));
    ASSERT_TRUE(GridMapTestAccess::accept(strict,packet(sensor)));
  }
  GridMapTestAccess::integrate(official);GridMapTestAccess::integrate(strict);
  ASSERT_GT(official.latestCloudStamp(),0.);
  GridMapTestAccess::integrate(official,102600000000LL,102.6);
  GridMapTestAccess::integrate(strict,102600000000LL,102.6);
  EXPECT_EQ(official.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(strict.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(official.getInflateOccupancy({1.55,.05,.05},0.),1);
  EXPECT_DOUBLE_EQ(official.latestCloudStamp(),0.);
  EXPECT_EQ(GridMapTestAccess::freeStamp(official,{9,0,0}),101900000000LL);
  GridMapTestAccess::expectSameRayEvidence(official,strict);
}

TEST(ProjectedRaysFreshness, FreshTraversalExpiresWithoutMutatingOddsOrOccupied) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0); // cached positive
  const auto raw=GridMapTestAccess::buffers(map);
  GridMapTestAccess::integrate(map,102400000000LL,102.4); // exact half-second expiry
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
  EXPECT_EQ(map.getInflateOccupancy({1.85,.05,.05},0.),2); // occluded / never observed
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);
  const auto detail=map.inspectInflateOccupancy({.95,.05,.05},0.,true);
  ASSERT_EQ(detail.voxels.size(),1U);
  EXPECT_EQ(detail.voxels[0].raw_state,0);
  EXPECT_EQ(detail.voxels[0].native_state,2);
  EXPECT_EQ(detail.voxels[0].classification,scan_planner::RawVoxelDiagnostic::StaleFree);
  EXPECT_EQ(detail.voxels[0].free_observation_stamp_ns,101900000000LL);
}

TEST(ProjectedRaysFreshness, FreshRemoteSourcesDoNotRenewNearRegionAndRealReobservationDoes) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,103350000000LL,2,1,{.55,.75,.05},{1.55,.75,.05}),103450000000LL,103.45));
  GridMapTestAccess::integrate(map,103450000000LL,103.45);
  EXPECT_GT(map.latestCloudStamp(),103.);
  EXPECT_EQ(map.getInflateOccupancy({.95,.75,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,103450000000LL,3),103550000000LL,103.55));
  GridMapTestAccess::integrate(map,103550000000LL,103.55);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, ClockRollbackCannotRevivePreviouslyExpiredFree) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  GridMapTestAccess::integrate(map,102000000000LL,102.7);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  // Even fresh data in a rewound clock cannot bypass the required new context.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102700000000LL,2),102800000000LL,102.8));
  GridMapTestAccess::integrate(map,102800000000LL,102.8);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
}

TEST(ProjectedRaysFreshness, QueryClockExpiresCacheWithoutAnIntegrationHeartbeat) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  GridMapTestAccess::queryTime(map,102500000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  GridMapTestAccess::queryTime(map,102000000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
}

TEST(ProjectedRaysFreshness, ResetAndSlidingClearEvidenceWithoutAliasingNewWorldCells) {
  for(bool slide:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
    if(slide) {
      GridMapTestAccess::slide(map,{3.,0.,0.});
      EXPECT_EQ(GridMapTestAccess::freeStamp(map,{49,0,0}),0);
      EXPECT_EQ(map.getInflateOccupancy({4.95,.05,.05},0.),2);
    } else {
      map.resetBuffer();
      EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
      EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
    }
  }
}

TEST(ProjectedRaysFreshness, PausedAndSlowSourceClockCannotRenewLeaseViaRepeatedTimerTicks) {
  for(bool slow:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
    std::int64_t effective=0;
    for(int i=1;i<=8;++i) {
      effective=GridMapTestAccess::steadyClockAdvance(map,102000000000LL+(slow?i*1000000LL:0LL),
          102000000000LL+i*100000000LL);
    }
    EXPECT_GE(effective,102800000000LL);
    EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  }
}

TEST(ProjectedRaysFreshness, EndpointHitNeverRenewsFreeLease) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,103350000000LL,2,1,
      {.55,.05,.05},{.95,.05,.05}),103450000000LL,103.45));
  GridMapTestAccess::integrate(map,103450000000LL,103.45);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),101900000000LL);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, PartialBatchCannotLendFreeEvidenceToLaterRemoteSuccess) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  // Front integrates near rays, rear has an origin outside the actual window.
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,1,1,{3.,0.,0.},{3.5,0.,0.})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.latestCloudStamp(),0.);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,
      packet(sensor,102000000000LL,2,1,{.55,.75,.05},{1.55,.75,.05}),102100000000LL,102.1));
  GridMapTestAccess::integrate(map,102100000000LL,102.1);
  EXPECT_GT(map.latestCloudStamp(),0.);
  EXPECT_EQ(map.getInflateOccupancy({.95,.75,.05},0.),0);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.getInflateOccupancy({1.55,.05,.05},0.),1);
}

TEST(ProjectedRaysFreshness, ActualFreeRecoveryWakesPlannerButFreshHeartbeatsDoNot) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::pointQueryGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned sensor=0;sensor<2;++sensor) ASSERT_TRUE(GridMapTestAccess::accept(map,packet(sensor)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  auto revision=map.occupancyRevision();
  GridMapTestAccess::integrate(map,102400000000LL,102.4);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),2);
  EXPECT_EQ(map.occupancyRevision(),revision);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102400000000LL,2),102500000000LL,102.5));
  GridMapTestAccess::integrate(map,102500000000LL,102.5);
  EXPECT_EQ(map.getInflateOccupancy({.95,.05,.05},0.),0);
  EXPECT_GT(map.occupancyRevision(),revision); // existing WAIT_ENVIRONMENT compares exactly this
  revision=map.occupancyRevision();
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102500000000LL,3),102600000000LL,102.6));
  GridMapTestAccess::integrate(map,102600000000LL,102.6);
  EXPECT_EQ(map.occupancyRevision(),revision); // no repeated search on steady fresh data
}

TEST(ProjectedRaysDiagnostics, EnablingDiagnosticsLeavesAllNativeBuffersRevisionAndCollisionUnchanged) {
  GridMap off,on;GridMapTestAccess::configure(off);GridMapTestAccess::configure(on);
  GridMapTestAccess::productionGeometry(off);GridMapTestAccess::productionGeometry(on);
  ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(on);
  for(unsigned iteration=0;iteration<3;++iteration) {
    const auto stamp=101900000000LL+iteration*10000000LL;
    for(unsigned sensor=0;sensor<2;++sensor) {
      const auto p=withSourceFields(packet(sensor,stamp,iteration+1,3,
          {.05,.05,.05},sensor?Eigen::Vector3d(1.55,.05,.05):Eigen::Vector3d(.55,.05,.05)));
      ASSERT_EQ(GridMapTestAccess::accept(off,p),GridMapTestAccess::accept(on,p));
    }
    GridMapTestAccess::integrate(off);GridMapTestAccess::integrate(on);
    GridMapTestAccess::expectSameEvidence(off,on);
    for(const Eigen::Vector3d &position: {Eigen::Vector3d(.05,.05,.05),Eigen::Vector3d(.55,.05,.05),
                                        Eigen::Vector3d(1.75,.05,.05)}) {
      for(double yaw:{0.,.71,1.57}) {
        const auto a=off.inspectInflateOccupancy(position,yaw,false);
        const auto b=on.inspectInflateOccupancy(position,yaw,true);
        EXPECT_EQ(a.counts,b.counts);
        EXPECT_EQ(off.getInflateOccupancy(position,yaw),on.getInflateOccupancy(position,yaw));
      }
    }
    GridMapTestAccess::expectSameEvidence(off,on); // A read-only query cannot mutate evidence.
  }
  EXPECT_EQ(off.nearFieldDiagnostics().size(),0U);
  EXPECT_GT(on.nearFieldDiagnostics().size(),0U);
}
TEST(ProjectedRaysDiagnostics, OfficialDescriptionUsesReaderClockWithoutChangingCollisionEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::productionGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,
      {.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map,102000000000LL,102.);
  const auto before=GridMapTestAccess::buffers(map);
  const auto revision=map.occupancyRevision();
  const Eigen::Vector3d body(.05,.05,.05);
  const int collision=map.getInflateOccupancy(body,0.);
  const auto description=map.describeInflateOccupancy(body,0.);
  EXPECT_NE(description.find("source_query_ns=102000000000"),std::string::npos)<<description;
  EXPECT_NE(description.find("front_age_ms=100"),std::string::npos)<<description;
  EXPECT_NE(description.find("rear_age_ms=nan"),std::string::npos)<<description;
  EXPECT_NE(description.find("occupied_witnesses=[[0.15,0.05,0.05]]"),std::string::npos)<<description;
  EXPECT_NE(description.find("occupied_witnesses_omitted=0"),std::string::npos);
  EXPECT_EQ(before,GridMapTestAccess::buffers(map));
  EXPECT_EQ(revision,map.occupancyRevision());
  EXPECT_EQ(collision,map.getInflateOccupancy(body,0.));
  GridMapTestAccess::queryTime(map,101800000000LL);
  EXPECT_NE(map.describeInflateOccupancy(body,0.).find("front_age_ms=nan"),std::string::npos);
}
TEST(ProjectedRaysDiagnostics, OccupiedWitnessExportIsBoundedWithoutTruncatingCounts) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::productionGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int z=-1;z<=1;++z)
    map.setOccupied({x*.1+.05,y*.1+.05,z*.1+.05});
  const auto evidence=map.inspectInflateOccupancy({.05,.05,.05},0.,true);
  ASSERT_GT(evidence.unique_counts[1],64U);
  const auto description=map.describeInflateOccupancy({.05,.05,.05},0.);
  const auto begin=description.find("occupied_witnesses=[");
  const auto end=description.find(" occupied_witnesses_omitted=",begin);
  ASSERT_NE(begin,std::string::npos);ASSERT_NE(end,std::string::npos);
  const auto entries=description.substr(begin,end-begin);
  EXPECT_EQ(std::count(entries.begin(),entries.end(),'['),65); // outer list + 64 witnesses
  EXPECT_NE(description.find("occupied_witnesses_omitted="+
      std::to_string(evidence.unique_counts[1]-64)),std::string::npos);
  EXPECT_NE(description.find("front_age_ms=nan"),std::string::npos); // no invented source clock
}
TEST(ProjectedRaysDiagnostics, CompleteRayFastVisitorsMatchSlowEvidenceAcrossStampsBoundarySlidingAndHitMissOverlap) {
  for(bool strict:{false,true})for(bool slide:{false,true}) {
    GridMap off,on;GridMapTestAccess::configure(off,strict);GridMapTestAccess::configure(on,strict);
    ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(on);
    for(unsigned round=0;round<6;++round) {
      if(slide&&round==3) {GridMapTestAccess::slide(off,{1.05,0.,0.});GridMapTestAccess::slide(on,{1.05,0.,0.});}
      const Eigen::Vector3d origin(round<3?-.55:.55,.05,.05);
      const auto base=101900000000LL+round*100000000LL,now=base+100000000LL;
      // Both stamp orderings matter: the second source may legitimately
      // refresh a previously visited free voxel, or be older than its peer.
      const auto front=base+(round%2?10000000LL:0LL),rear=base+(round%2?0LL:10000000LL);
      const std::vector<Eigen::Vector3d> ends{{.45,.05,.05},{1.55,.05,.05},{-3.,.05,.05},
          {3.,1.9,.05},{.45,-1.95,-1.95},{.45,1.95,1.95}};
      for(unsigned sensor=0;sensor<2;++sensor) {
        auto message=packet(sensor,sensor?rear:front,round+1,192,origin,ends[(round+sensor)%ends.size()]);
        ASSERT_EQ(GridMapTestAccess::accept(off,message,now,now*1e-9),
                  GridMapTestAccess::accept(on,message,now,now*1e-9));
      }
      GridMapTestAccess::integrate(off,now,now*1e-9);GridMapTestAccess::integrate(on,now,now*1e-9);
      GridMapTestAccess::expectSameCompleteRayState(off,on);
    }
  }
}
TEST(ProjectedRaysDiagnostics, CompleteDuplicateRayWorkloadReportsTimingWithIdenticalMapEvidence) {
  GridMap off,on;GridMapTestAccess::configure(off,false);GridMapTestAccess::configure(on,false);
  ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(on);std::vector<double> off_ms,on_ms;
  for(unsigned round=0;round<8;++round) {
    const auto source=101900000000LL+round*100000000LL,now=source+100000000LL;
    for(unsigned sensor=0;sensor<2;++sensor) {
      auto p=packet(sensor,source+sensor*10000000LL,round+1,8192,{-.55,.05,.05},{1.25,.05,.05});
      ASSERT_TRUE(GridMapTestAccess::accept(off,p,now,now*1e-9));
      ASSERT_TRUE(GridMapTestAccess::accept(on,p,now,now*1e-9));
    }
    for(auto pair:{std::make_pair(&off,&off_ms),std::make_pair(&on,&on_ms)}) {
      const auto begin=std::chrono::steady_clock::now();GridMapTestAccess::integrate(*pair.first,now,now*1e-9);
      pair.second->push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
    }
    GridMapTestAccess::expectSameCompleteRayState(off,on);
  }
  std::sort(off_ms.begin(),off_ms.end());std::sort(on_ms.begin(),on_ms.end());
  std::cout<<"complete_duplicate_ray_timing off_median_ms="<<(off_ms[3]+off_ms[4])*.5
      <<" diagnostic_on_median_ms="<<(on_ms[3]+on_ms[4])*.5<<" rays_per_round=16384 rounds=8\n";
}

TEST(ProjectedRaysDiagnostics, SourceVisitBitsPreserveGeneralSourceOrderAndThirdStampFallback) {
  constexpr std::int64_t base=101900000000LL;
  const std::vector<std::vector<std::int64_t>> patterns{
    {base},{base,base},{base,base+10000000LL},{base+10000000LL,base},
    {base,base+10000000LL,base,base+10000000LL},
    {base,base+10000000LL,base+20000000LL},{base,0,base}};
  const std::vector<Eigen::Vector3d> end_pattern{
    {1.55,.05,.05},{-.95,-.05,-.05},{2.8,1.8,-.05},
    {-.15,.05,.05},{-1.8,1.6,-1.4}};
  for(bool strict:{false,true})for(const auto& pattern:patterns) {
    GridMap off,on;GridMapTestAccess::configure(off,strict);GridMapTestAccess::configure(on,strict);
    ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(on);
    std::vector<std::int64_t> stamps;std::vector<Eigen::Vector3d> origins,ends;
    for(unsigned i=0;i<512;++i) {
      stamps.push_back(pattern[i%pattern.size()]);
      origins.emplace_back(-.55,.05,.05);ends.push_back(end_pattern[i%end_pattern.size()]);
    }
    for(unsigned repeat=0;repeat<2;++repeat) {
      ASSERT_EQ(GridMapTestAccess::directAcquisitions(off,stamps,origins,ends),
                GridMapTestAccess::directAcquisitions(on,stamps,origins,ends));
      GridMapTestAccess::expectSameEvidence(off,on);
      GridMapTestAccess::expectSameRayEvidence(off,on);
      EXPECT_EQ(off.mapWorkTiming().ray_steps,on.mapWorkTiming().ray_steps);
    }
  }
}

TEST(ProjectedRaysDiagnostics, SourceVisitBitsDoNotSkipTraversalOrRenewExhaustedBudget) {
  GridMap fast,general;GridMapTestAccess::configure(fast,false);GridMapTestAccess::configure(general,false);
  ASSERT_TRUE(fast.applyLocalizationContext(context()));ASSERT_TRUE(general.applyLocalizationContext(context()));
  GridMapTestAccess::fullFixtureRayRange(fast);GridMapTestAccess::fullFixtureRayRange(general);
  constexpr std::size_t count=200000;
  constexpr std::int64_t base=101900000000LL;
  std::vector<std::int64_t> stamps(count,base);
  std::fill(stamps.begin()+count/2,stamps.end(),base+10000000LL);
  const std::vector<Eigen::Vector3d> origins(count,Eigen::Vector3d(-1.8,-1.8,-1.8));
  const std::vector<Eigen::Vector3d> ends(count,Eigen::Vector3d(1.8,1.7,1.6));
  EXPECT_FALSE(GridMapTestAccess::directAcquisitions(fast,stamps,origins,ends));
  // A third stamp AFTER the eventual exhaustion forces the unchanged general
  // visitor, without changing any ray actually reached before the same limit.
  stamps.back()=base+20000000LL;
  EXPECT_FALSE(GridMapTestAccess::directAcquisitions(general,stamps,origins,ends));
  EXPECT_EQ(fast.mapWorkTiming().ray_steps,16000000U);
  EXPECT_EQ(general.mapWorkTiming().ray_steps,16000000U);
  GridMapTestAccess::expectSameEvidence(fast,general);
  GridMapTestAccess::expectSameRayEvidence(fast,general);
}

TEST(ProjectedRaysDiagnostics, DenseNearReturnsDistinguishRayVisitsFromActualVotes) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,withSourceFields(packet(0,101900000000LL,1,3,
      {.05,.05,.05},{.15,.05,.05}))));
  GridMapTestAccess::integrate(map);
  const auto *hit=map.nearFieldDiagnostics().find({1,0,0});
  ASSERT_NE(hit,nullptr);ASSERT_TRUE(hit->last[0]);
  EXPECT_EQ(hit->visits[0],3U);EXPECT_EQ(hit->votes[0],1U);
  EXPECT_EQ(hit->last[0]->metadata.source_index,12002U);
  EXPECT_EQ(hit->last[0]->metadata.ring,22U);
  EXPECT_EQ(hit->last[0]->metadata.scan_stamp_ns,101900000000LL);
  EXPECT_EQ(hit->last[0]->metadata.integration_ns,102000000000LL);
  EXPECT_TRUE(hit->last[0]->metadata.source_fields_available);
  EXPECT_GT(GridMapTestAccess::raw(map,.15),1.);
  const auto *miss=map.nearFieldDiagnostics().find({0,0,0});
  ASSERT_NE(miss,nullptr);ASSERT_TRUE(miss->last[1]);
  EXPECT_EQ(miss->votes[1],1U);
  EXPECT_GE(miss->visits[1],1U);
}

TEST(ProjectedRaysDiagnostics, ConflictingDuplicateOptionalFieldDisablesWitnessMetadataNotAdmission) {
  for(const char *duplicate_name:{"ring","offset_time","source_index","timestamp","source_timestamp","raw_timestamp"}) {
    GridMap off,on;GridMapTestAccess::configure(off);GridMapTestAccess::configure(on);
    ASSERT_TRUE(off.applyLocalizationContext(context()));ASSERT_TRUE(on.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(on);
    auto p=withSourceFields(packet(0,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05}));
    auto found=std::find_if(p.rays.fields.begin(),p.rays.fields.end(),
        [&](const auto &field){return field.name==duplicate_name;});
    ASSERT_NE(found,p.rays.fields.end());
    auto duplicate=*found;duplicate.offset=0;duplicate.datatype=sensor_msgs::msg::PointField::UINT8;
    p.rays.fields.push_back(duplicate);
    ASSERT_TRUE(GridMapTestAccess::accept(off,p));
    ASSERT_TRUE(GridMapTestAccess::accept(on,p));
    GridMapTestAccess::integrate(off);GridMapTestAccess::integrate(on);
    GridMapTestAccess::expectSameEvidence(off,on);
    const auto *hit=on.nearFieldDiagnostics().find({1,0,0});
    ASSERT_NE(hit,nullptr);ASSERT_TRUE(hit->last[0]);
    EXPECT_FALSE(hit->last[0]->metadata.source_fields_available)<<duplicate_name;
    EXPECT_EQ(hit->last[0]->metadata.scan_stamp_ns,101900000000LL);
    EXPECT_GT(GridMapTestAccess::raw(on,.15),1.);
  }
}

TEST(ProjectedRaysDiagnostics, DetailedCylinderOverlapCountsUniqueWorldVoxelsExactlyOnce) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d body(.05,.05,.05);
  const auto simple=map.inspectInflateOccupancy(body,.63,false);
  const auto detail=map.inspectInflateOccupancy(body,.63,true);
  EXPECT_EQ(simple.counts,detail.counts);
  const auto raw=std::accumulate(detail.counts.begin(),detail.counts.end(),std::size_t{0});
  const auto unique=std::accumulate(detail.unique_counts.begin(),detail.unique_counts.end(),std::size_t{0});
  EXPECT_GT(raw,unique);EXPECT_EQ(detail.voxels.size(),unique);
  std::set<std::array<int,3>> indices;std::size_t memberships=0,both=0;
  for(const auto &voxel:detail.voxels) {
    EXPECT_TRUE(indices.insert({voxel.index.x(),voxel.index.y(),voxel.index.z()}).second);
    EXPECT_GT(voxel.cylinder_mask,0U);EXPECT_LE(voxel.cylinder_mask,3U);
    memberships+=bool(voxel.cylinder_mask&1U)+bool(voxel.cylinder_mask&2U);
    both+=voxel.cylinder_mask==3U;
    EXPECT_EQ(voxel.classification,scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(voxel.native_state,2);
  }
  EXPECT_GT(both,0U);EXPECT_EQ(memberships,raw);
  EXPECT_NE(map.getInflateOccupancy(body,.63),0);
}

TEST(ProjectedRaysDiagnostics, SlideClearsWorldWitnessBeforeCircularBufferSlotReuse) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,
      {-1.55,.05,.05},{-1.85,.05,.05})));
  GridMapTestAccess::integrate(map);
  const Eigen::Vector3i old(-19,0,0), replacement(21,0,0);
  const int address=GridMapTestAccess::address(map,old);
  ASSERT_NE(map.nearFieldDiagnostics().find(old),nullptr);
  GridMapTestAccess::slide(map,{.55,0.,0.});
  EXPECT_EQ(map.nearFieldDiagnostics().find(old),nullptr);
  EXPECT_EQ(map.nearFieldDiagnostics().find(replacement),nullptr);
  EXPECT_EQ(GridMapTestAccess::address(map,replacement),address);
  EXPECT_EQ(GridMapTestAccess::category(map,replacement),scan_planner::RawVoxelDiagnostic::NeverObserved);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101920000000LL,2,1,
      {1.65,.05,.05},{2.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  ASSERT_NE(map.nearFieldDiagnostics().find(replacement),nullptr);
  EXPECT_FALSE(map.nearFieldDiagnostics().find(replacement)->last[0]);
  EXPECT_TRUE(map.nearFieldDiagnostics().find(replacement)->last[2]);
  EXPECT_EQ(map.nearFieldDiagnostics().find(old),nullptr);
}

TEST(ProjectedRaysDiagnostics, LargeWindowJumpAndNewLocalizationIdentityClearAllWitnesses) {
  for(bool relocation:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
    GridMapTestAccess::diagnostics(map);
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));GridMapTestAccess::integrate(map);
    ASSERT_GT(map.nearFieldDiagnostics().size(),0U);
    const auto generation=map.nearFieldDiagnostics().generation();
    if(relocation) ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"new-seed")));
    else GridMapTestAccess::slide(map,{6.,0.,0.});
    EXPECT_EQ(map.nearFieldDiagnostics().size(),0U);
    EXPECT_GT(map.nearFieldDiagnostics().generation(),generation);
    EXPECT_EQ(map.nearFieldDiagnostics().find({15,0,0}),nullptr);
  }
}

TEST(ProjectedRaysDiagnostics, SubthresholdRealHitIsInsufficientNotNeverObservedAndStillBlocked) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionProbabilities(map);
  GridMapTestAccess::productionGeometry(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  GridMapTestAccess::diagnostics(map);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::category(map,{1,0,0}),scan_planner::RawVoxelDiagnostic::Insufficient);
  EXPECT_EQ(GridMapTestAccess::category(map,{2,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  ASSERT_NE(map.nearFieldDiagnostics().find({1,0,0}),nullptr);
  EXPECT_TRUE(map.nearFieldDiagnostics().find({1,0,0})->last[0]);
  const auto detail=map.inspectInflateOccupancy({.05,.05,.05},0.,true);
  std::size_t insufficient=0;
  for(const auto &voxel:detail.voxels) insufficient+=voxel.classification==scan_planner::RawVoxelDiagnostic::Insufficient;
  EXPECT_GE(insufficient,1U);EXPECT_NE(map.getInflateOccupancy({.05,.05,.05},0.),0);
}

TEST(ProjectedRaysDiagnostics, LowOrThinExternalReturnRemainsOccupiedAndOccludedSpaceUnknown) {
  for(double height:{.12,.39}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::productionGeometry(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));GridMapTestAccess::diagnostics(map);
    const Eigen::Vector3d end(.15,.05,height);
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{-.15,.05,height},end)));
    GridMapTestAccess::integrate(map);
    const auto cell=GridMapTestAccess::index(map,end);
    EXPECT_EQ(GridMapTestAccess::category(map,cell),scan_planner::RawVoxelDiagnostic::Occupied);
    const auto behind=GridMapTestAccess::index(map,{.25,.05,height});
    EXPECT_EQ(GridMapTestAccess::category(map,behind),scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(map.nearFieldDiagnostics().find(behind),nullptr);
    const auto detail=map.inspectInflateOccupancy({.05,.05,.35},0.,true);
    EXPECT_GT(detail.unique_counts[1],0U);
    EXPECT_NE(map.getInflateOccupancy({.05,.05,.35},0.),0);
  }
}

TEST(ProjectedRays, DualOriginsDoNotInventFreeSpaceBetweenSensorsOrBeyondHits) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  GridMapTestAccess::integrate(map);EXPECT_EQ(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101950000000LL,2,1,{-.55,.05,.05},{-1.55,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_DOUBLE_EQ(map.latestCloudStamp(),101.9);
  EXPECT_EQ(GridMapTestAccess::raw(map,.75),-1.);EXPECT_EQ(GridMapTestAccess::raw(map,-.75),-1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.05),-1.);EXPECT_LT(GridMapTestAccess::raw(map,-.25),-1.);
  EXPECT_GT(GridMapTestAccess::raw(map,1.55),1.);EXPECT_GT(GridMapTestAccess::raw(map,-1.55),1.);
  EXPECT_LT(GridMapTestAccess::raw(map,1.75),-1.);EXPECT_LT(GridMapTestAccess::raw(map,-1.75),-1.);
}
TEST(ProjectedRays, NearReturnIsNotDiscardedAndDenseHitsCannotOverflowVotes) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,100000,{.05,.05,.05},{.15,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::raw(map,.05),-1.);EXPECT_GT(GridMapTestAccess::raw(map,.15),1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.25),-1.);
}
TEST(ProjectedRays, RealHitWinsAgainstCrossingFreeRayInSameIntegration) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{.55,.05,.05})));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,2,1,{.05,.05,.05},{1.55,.05,.05})));
  GridMapTestAccess::integrate(map);EXPECT_GT(GridMapTestAccess::raw(map,.55),1.);
}
TEST(ProjectedRays, ContextMismatchAndOldPacketsCannotRefillReplacementMap) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto p=packet();ASSERT_TRUE(GridMapTestAccess::accept(map,p));
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"replacement")));
  EXPECT_EQ(GridMapTestAccess::pending(map),0);EXPECT_FALSE(GridMapTestAccess::accept(map,p));
  p.epoch=2;p.context_sequence=2;p.seed_id="replacement";p.barrier_ns=100000000002ULL;
  EXPECT_TRUE(GridMapTestAccess::accept(map,p));GridMapTestAccess::integrate(map);
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::counts(map)[1],0U);
  for(unsigned field=0;field<5;++field) {
    auto bad=p;
    if(field==0) bad.session_id="other";
    if(field==1) --bad.epoch;
    if(field==2) --bad.context_sequence;
    if(field==3) --bad.barrier_ns;
    if(field==4) bad.seed_id="other";
    EXPECT_FALSE(GridMapTestAccess::accept(map,bad));
  }
}
TEST(ProjectedRays, BoundedIndependentQueuesRejectReplaysAndExpiredReceipts) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto p=packet();ASSERT_TRUE(GridMapTestAccess::accept(map,p));
  EXPECT_FALSE(GridMapTestAccess::accept(map,p));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101910000000LL,2)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101920000000LL,3)));
  EXPECT_EQ(GridMapTestAccess::pending(map),2);EXPECT_EQ(GridMapTestAccess::drops(map)[0],2U);
  GridMapTestAccess::integrate(map,102300000000LL,102.3);
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
}
TEST(ProjectedRays, OneSourceAndPausedSourceClockDoNotRenewBothSourceLease) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  GridMapTestAccess::integrate(map);ASSERT_GT(map.latestCloudStamp(),0.);
  GridMapTestAccess::integrate(map,102000000000LL,102.6);EXPECT_EQ(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102400000000LL,2),102500000000LL,102.6));
  GridMapTestAccess::integrate(map,102500000000LL,102.6);EXPECT_EQ(map.latestCloudStamp(),0.);
}
TEST(ProjectedRays, MalformedFieldsTimesAndUnboundedPayloadAreRejected) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  for(unsigned variant=0;variant<13;++variant) {
    auto p=packet();
    if(variant==0) p.rays.header.frame_id="odom";
    if(variant==1) p.rays.point_step=12;
    if(variant==2) p.rays.is_bigendian=true;
    if(variant==3) p.rays.fields[3].name="missing_origin";
    if(variant==4) p.rays.fields[3].datatype=sensor_msgs::msg::PointField::FLOAT64;
    if(variant==5) p.rays.fields.push_back(p.rays.fields[0]);
    if(variant==6) p.rays.row_step=1;
    if(variant==7) p.rays.header.stamp=timeAt(101000000000LL);
    if(variant==8) p.acquisition_end=timeAt(102500000000LL);
    if(variant==9) p.alignment_stamp=timeAt(99000000000LL);
    if(variant==10) p.rays.data[28]=2;
    if(variant==11) {const float nan=std::numeric_limits<float>::quiet_NaN();std::memcpy(p.rays.data.data(),&nan,4);}
    if(variant==12) p.rays.width=100001;
    EXPECT_FALSE(GridMapTestAccess::accept(map,p))<<variant;
  }
  GridMapTestAccess::legacy(map);EXPECT_FALSE(GridMapTestAccess::accept(map,packet()));
}
TEST(ProjectedRays, TraversalBudgetCannotReportIncompleteWorkAsComplete) {
  std::size_t budget=2;unsigned visits=0;
  EXPECT_FALSE(scan_planner::visitObservedRay({.05,.05,.05},{1.55,.05,.05},.1,false,budget,
      [&](const Eigen::Vector3i &){++visits;}));
  EXPECT_EQ(budget,0U);EXPECT_EQ(visits,2U);
}
TEST(ProjectedRays, ClippedDistantReturnPreservesNearFreeButNeverCreatesBoundaryHit) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{.05,.05,.05},{9.55,.05,.05})));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::raw(map,.15),-1.);
  EXPECT_EQ(GridMapTestAccess::raw(map,1.85),-1.);
  EXPECT_EQ(GridMapTestAccess::raw(map,1.95),-1.);
  EXPECT_LT(GridMapTestAccess::raw(map,.15,.25),-1.);
}
TEST(ProjectedRays, MixedSensorPacketAndNonfiniteOriginAreNotPartiallyApplied) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  auto mixed=packet(0,101900000000LL,1,2);mixed.rays.data[64+28]=1;
  EXPECT_FALSE(GridMapTestAccess::accept(map,mixed));
  auto corrupt=packet();const float inf=std::numeric_limits<float>::infinity();
  std::memcpy(corrupt.rays.data.data()+16,&inf,4);
  EXPECT_FALSE(GridMapTestAccess::accept(map,corrupt));
  GridMapTestAccess::integrate(map);EXPECT_LT(GridMapTestAccess::raw(map,.75),-1.);
}
TEST(ProjectedRays, SameCoordinateContextBarrierStillClearsPendingAndBothLeases) {
  GridMap map;GridMapTestAccess::configure(map);ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  GridMapTestAccess::integrate(map);ASSERT_GT(map.latestCloudStamp(),0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101910000000LL,2)));
  ASSERT_TRUE(map.applyLocalizationContext(context(1,2)));
  EXPECT_EQ(map.latestCloudStamp(),0.);EXPECT_EQ(GridMapTestAccess::pending(map),0);
  EXPECT_EQ(GridMapTestAccess::counts(map)[0],0U);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,1.55),-1.01); // New context ACK certifies actual rebuild.
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101920000000LL,3)));
}

static scan_planner::ProjectedRayStatusSchedule::Clock::time_point statusTime(double seconds) {
  return scan_planner::ProjectedRayStatusSchedule::Clock::time_point(
      std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(seconds*1e9))));
}
TEST(ProjectedRayStatus, CompletedIntegrationBypassesDiagnosticHeartbeatDelay) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  ASSERT_TRUE(schedule.due(statusTime(10.),9700000000LL,true,1));
  schedule.markPublished(statusTime(10.),9700000000LL,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.05),9700000000LL,true,1));
  // A newer actual completed integration must not wait until t=10.2.
  EXPECT_TRUE(schedule.due(statusTime(10.05),9800000000LL,true,1));
  schedule.markPublished(statusTime(10.05),9800000000LL,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.10),9800000000LL,true,1));
}
TEST(ProjectedRayStatus, HeartbeatRepeatsOriginalLeaseAndReportsExpiryImmediately) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  const auto source=10000000000LL;
  schedule.markPublished(statusTime(10.2),source,true,1);
  EXPECT_FALSE(schedule.due(statusTime(10.399),source,true,1));
  EXPECT_TRUE(schedule.due(statusTime(10.4),source,true,1));
  schedule.markPublished(statusTime(10.4),source,true,1);
  // The original source expires at .5 s, not .5 s after a status heartbeat.
  EXPECT_FALSE(scan_planner::rayStampFresh(source,10510000000LL,.5));
  EXPECT_TRUE(schedule.due(statusTime(10.51),0,false,1));
  schedule.markPublished(statusTime(10.51),0,false,1);
  EXPECT_FALSE(schedule.due(statusTime(10.55),0,false,1));
}
TEST(ProjectedRayStatus, FreshIntegrationDoesNotBecomeStaleOnlyBecauseHeartbeatWasThrottled) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  std::int64_t reported=9680000000LL;
  schedule.markPublished(statusTime(10.),reported,true,1);
  const auto integrated=9780000000LL; // just integrated at 10.10 s, age .32 s
  ASSERT_TRUE(schedule.due(statusTime(10.10),integrated,true,1));
  reported=integrated;schedule.markPublished(statusTime(10.10),reported,true,1);
  // At 10.19 s the old 5 Hz-only notification would have expired (.51 s)
  // although the current actual integration is only .41 s old.
  EXPECT_FALSE(scan_planner::rayStampFresh(9680000000LL,10190000000LL,.5));
  EXPECT_TRUE(scan_planner::rayStampFresh(reported,10190000000LL,.5));
}
TEST(ProjectedRayStatus, ContextChangeAndInvalidityCannotWaitForHeartbeat) {
  scan_planner::ProjectedRayStatusSchedule schedule;
  schedule.markPublished(statusTime(20.),0,false,1);
  EXPECT_TRUE(schedule.due(statusTime(20.01),0,false,2));
  schedule.markPublished(statusTime(20.01),0,false,2);
  EXPECT_TRUE(schedule.due(statusTime(20.02),19900000000LL,true,2));
  schedule.markPublished(statusTime(20.02),19900000000LL,true,2);
  EXPECT_TRUE(schedule.due(statusTime(20.03),0,false,2));
}

TEST(ProjectedRayHitSubvoxels, EveryEndpointContributesBeforeCoarseVoteDedupAndAtSaturation) {
  for(bool legacy:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
    if(legacy)GridMapTestAccess::legacy(map);
    const Eigen::Vector3d origin(.55,.05,.05),low(.011,.011,.011),high(.091,.091,.091);
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{101000000000LL},{origin},{low}));
    EXPECT_EQ(GridMapTestAccess::hitMask(map,low),1U);
    // Coarse cell is already at max odds. A later point in the same voxel is
    // still real geometry, even though neither the old hit vote nor old odds
    // may change. Preserve all duplicates before legacy ray-end shortcuts.
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{102000000000LL,102000000000LL,102000000000LL},
        {origin,origin,origin},{low,high,low}));
    EXPECT_EQ(GridMapTestAccess::hitMask(map,low),129U);
    EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,low.x(),low.y(),low.z()),2.);
  }
}

TEST(ProjectedRayHitSubvoxels, DualSourceUnionAndRejectedPacketsCannotEraseOrAddGeometry) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d origin(.55,.05,.05),low(.011,.011,.011),high(.091,.091,.091);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,3,origin,low)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,101900000000LL,1,3,origin,high)));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(GridMapTestAccess::hitMask(map,low),129U);
  const auto evidence=GridMapTestAccess::allHitMasks(map);
  const auto revision=map.occupancyRevision();
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet(0,101800000000LL,2,3,origin,{.011,.091,.011})));
  auto wrong=packet(0,101950000000LL,3,3,origin,{.011,.091,.011});wrong.rays.header.frame_id="wrong";
  EXPECT_FALSE(GridMapTestAccess::accept(map,wrong));
  GridMapTestAccess::integrate(map);
  EXPECT_EQ(evidence,GridMapTestAccess::allHitMasks(map));EXPECT_EQ(revision,map.occupancyRevision());
  ASSERT_TRUE(map.applyLocalizationContext(context(2,2,"next")));
  const auto cleared=GridMapTestAccess::allHitMasks(map);
  EXPECT_TRUE(std::all_of(cleared.begin(),cleared.end(),[](auto value){return value==0;}));
  EXPECT_FALSE(GridMapTestAccess::accept(map,packet()));
  EXPECT_EQ(cleared,GridMapTestAccess::allHitMasks(map));
}

TEST(ProjectedRayHitSubvoxels, MissesPreserveHitUnionUntilGenuineCoarseFree) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
  const Eigen::Vector3d origin(.55,.05,.05),hit(.95,.05,.05),beyond(1.55,.05,.05);
  ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{101000000000LL},{origin},{hit}));
  const auto mask=GridMapTestAccess::hitMask(map,hit);ASSERT_NE(mask,0U);
  for(int i=1;i<=6;++i) {
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{101000000000LL+i*100000000LL},{origin},{beyond}));
    if(i<6)EXPECT_EQ(GridMapTestAccess::hitMask(map,hit),mask);
  }
  EXPECT_EQ(GridMapTestAccess::hitMask(map,hit),0U);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::raw(map,hit.x()),-1.);
}

TEST(ProjectedRayHitSubvoxels, ClippedNoReturnRayNeverCreatesEndpointHitEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::subvoxels(map);
  ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{101000000000LL},{{.05,.05,.05}},{{6.,.05,.05}}));
  const auto masks=GridMapTestAccess::allHitMasks(map);
  EXPECT_TRUE(std::all_of(masks.begin(),masks.end(),[](auto value){return value==0;}));
}

TEST(ProjectedRaysDiagnostics, FirstEffectiveWitnessSkipsRawCellClearedByVerticalNarrowphase) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::stairDiagnosticBox(map);
  const Eigen::Vector3d body(0.,0.,.31),below(-.16,.025,.025),wall(.16,.025,.075);
  map.setOccupiedHit(below);
  ASSERT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  const auto first_raw=map.inspectInflateOccupancy(body,0.,true);
  ASSERT_EQ(first_raw.counts[1],1U);ASSERT_EQ(first_raw.unique_counts[1],1U);
  const auto before=GridMapTestAccess::buffers(map);
  const auto masks_before=GridMapTestAccess::allHitMasks(map);
  const auto clear_description=map.describeInflateOccupancy(body,0.);
  EXPECT_NE(clear_description.find("collision=0"),std::string::npos);
  EXPECT_NE(clear_description.find("occupied_cell=(-0.15,0.05,0.05)"),std::string::npos);
  EXPECT_NE(clear_description.find("collision_occupied_cell=none"),std::string::npos);
  EXPECT_NE(clear_description.find("\"refined_intersects\":false"),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::buffers(map),before);EXPECT_EQ(GridMapTestAccess::allHitMasks(map),masks_before);
  map.setOccupiedHit(wall);
  const auto revision=map.occupancyRevision();const int verdict=map.getPlanningInflateOccupancy(body,0.);
  ASSERT_EQ(verdict,1);const auto mixed=map.describeInflateOccupancy(body,0.);
  EXPECT_NE(mixed.find("occupied=2"),std::string::npos);
  EXPECT_NE(mixed.find("occupied_cell=(-0.15,0.05,0.05)"),std::string::npos);
  EXPECT_NE(mixed.find("collision_occupied_cell=(0.15,0.05,0.05)"),std::string::npos);
  EXPECT_NE(mixed.find("collision_occupied_hit_mask="),std::string::npos);
  EXPECT_NE(mixed.find("\"refined_intersects\":true"),std::string::npos);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),verdict);EXPECT_EQ(map.occupancyRevision(),revision);
}

TEST(ProjectedRaysDiagnostics, EffectiveWitnessRetainsCoarseFallbackAndBoundsOutput) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::stairDiagnosticBox(map);
  // No original endpoint: the manual coarse cell still intersects the body.
  map.setOccupancy({-.16,.025,.025},1.);
  ASSERT_EQ(map.getPlanningInflateOccupancy({0.,0.,.31},0.),1);
  const auto description=map.describeInflateOccupancy({0.,0.,.31},0.);
  EXPECT_NE(description.find("collision_occupied_hit_mask=256"),std::string::npos);
  EXPECT_NE(description.find("collision_occupied_cell=(-0.15,0.05,0.05)"),std::string::npos);
  for(int x=-3;x<=2;++x)for(int y=-2;y<=1;++y)for(int z=1;z<=4;++z)
    map.setOccupiedHit({(x+.5)*.1,(y+.5)*.1,(z+.5)*.1});
  const auto dense=map.describeInflateOccupancy({0.,0.,.31},0.);
  const auto start=dense.find("occupied_witness_details=");ASSERT_NE(start,std::string::npos);
  const auto end=dense.find(" occupied_witness_details_omitted=",start);ASSERT_NE(end,std::string::npos);
  const auto details=nlohmann::json::parse(dense.substr(start+25,end-start-25));
  EXPECT_EQ(details.size(),64U);EXPECT_GT(dense.size(),description.size());
  EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.31},0.),1);
}

TEST(ProjectedRaysDiagnostics, FourAllHitBoundsArePassiveExactAndDoNotInventCoarseEvidence) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::contactEvidence(map);
  const Eigen::Vector3d body(0.,0.,.31),a(.111,.011,.111),b(.142,.042,.142);
  map.setOccupiedHit(a);map.setOccupiedHit(b);
  const auto parse=[](const std::string& text) {
    const std::string label="collision_hit_bounds=";
    const auto begin=text.find(label),end=text.find(" collision_hit_bounds_omitted=",begin);
    EXPECT_NE(begin,std::string::npos);EXPECT_NE(end,std::string::npos);
    return nlohmann::json::parse(text.substr(begin+label.size(),end-begin-label.size()));
  };
  auto raw=GridMapTestAccess::buffers(map);auto masks=GridMapTestAccess::allHitMasks(map);
  auto revision=map.occupancyRevision();const auto verdict=map.getPlanningInflateOccupancy(body,0.);
  ASSERT_EQ(verdict,1);
  const auto actual=parse(map.describeInflateOccupancy(body,0.));ASSERT_EQ(actual.size(),1U);
  EXPECT_TRUE(actual[0]["available"].get<bool>());
  const auto expected=GridMapTestAccess::bounds(map,a);
  for(int k=0;k<3;++k) {
    EXPECT_NEAR(actual[0]["lo"][k].get<double>(),expected.first[k],1e-9);
    EXPECT_NEAR(actual[0]["hi"][k].get<double>(),expected.second[k],1e-9);
  }
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);EXPECT_EQ(GridMapTestAccess::allHitMasks(map),masks);
  EXPECT_EQ(map.occupancyRevision(),revision);EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),verdict);
  // A manual/coarse cell must not receive a made-up zero-sized endpoint box.
  map.setOccupancy({-.16,.025,.125},1.);
  const auto mixed=parse(map.describeInflateOccupancy(body,0.));
  ASSERT_EQ(mixed.size(),2U);int unavailable=0;
  for(const auto& entry:mixed)if(!entry["available"].get<bool>()) {
    ++unavailable;EXPECT_FALSE(entry.contains("lo"));EXPECT_FALSE(entry.contains("hi"));
  }
  EXPECT_EQ(unavailable,1);
  for(int x=-3;x<=2;++x)for(int y=-2;y<=1;++y)for(int z=1;z<=4;++z)
    map.setOccupiedHit({(x+.5)*.1,(y+.5)*.1,(z+.5)*.1});
  raw=GridMapTestAccess::buffers(map);masks=GridMapTestAccess::allHitMasks(map);revision=map.occupancyRevision();
  const auto dense=map.describeInflateOccupancy(body,0.);
  EXPECT_EQ(parse(dense).size(),4U);
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);EXPECT_EQ(GridMapTestAccess::allHitMasks(map),masks);
  EXPECT_EQ(map.occupancyRevision(),revision);EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
}

TEST(ProjectedRayContactBounds,SameOctantNewHitExtendsUnionBeforeProbabilityDedup) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::contactEvidence(map);
  const Eigen::Vector3d origin(.55,.05,.05),a(.011,.011,.011),b(.042,.042,.042);
  ASSERT_TRUE(GridMapTestAccess::directAcquisitions(map,{101000000000LL,101000000000LL},
      {origin,origin},{a,b}));
  EXPECT_EQ(GridMapTestAccess::hitMask(map,a),1U);
  const auto bounds=GridMapTestAccess::bounds(map,a);
  EXPECT_TRUE((bounds.first.array()<=a.array()).all());
  EXPECT_TRUE((bounds.second.array()>=b.array()).all());
  const auto revision=map.occupancyRevision();
  map.setOccupiedHit({.044,.044,.044});
  EXPECT_GT(map.occupancyRevision(),revision);
  EXPECT_GE(GridMapTestAccess::bounds(map,a).second.z(),.044);
}

TEST(ProjectedRayContactBounds,ExactZOnlyRefinesStairQueriesNeverRawOrFlat) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::contactEvidence(map);
  const Eigen::Vector3d hit(.011,.011,.011),body(0.,0.,.28);
  map.setOccupiedHit(hit);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0); // .011 below .03 underside
  const auto raw=GridMapTestAccess::buffers(map);
  map.exchangeQueryStairClearanceProvider({});
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1); // unchanged coarse flat query
  EXPECT_EQ(GridMapTestAccess::buffers(map),raw);
  map.exchangeQueryStairClearanceProvider([](const auto&){return true;});
  map.setOccupiedHit({.011,.011,.041});
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1); // same octant, new true hit
}

TEST(ProjectedRayContactBounds,CoarseHistoryNeverExplainedByLaterTreadHit) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::contactEvidence(map);
  const Eigen::Vector3d hit(.011,.011,.011),body(0.,0.,.28);
  map.setOccupancy(hit,1.);map.setOccupiedHit(hit);
  map.exchangeQueryStairContactProvider([](const auto&,const auto&){return true;});
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  EXPECT_NE(GridMapTestAccess::hitMask(map,hit)&0x100U,0U);
}

TEST(ProjectedRayContactBounds,SnapshotOwnsBoundsButNotAnotherCurvesContactProvider) {
  GridMap writer;GridMapTestAccess::configure(writer,false);GridMapTestAccess::contactEvidence(writer);
  const Eigen::Vector3d hit(.011,.011,.011);
  writer.setOccupiedHit(hit);
  writer.exchangeQueryStairContactProvider([](const auto&,const auto&){return true;});
  GridMap reader;writer.copyCollisionSnapshotTo(reader,102000000000LL,std::chrono::steady_clock::now());
  const auto bounds=GridMapTestAccess::bounds(reader,hit);
  writer.setOccupiedHit({.044,.044,.044});
  EXPECT_EQ(GridMapTestAccess::bounds(reader,hit),bounds);
  // No provider leaked into the next version. Its old plane cannot excuse a
  // hit in this map's unchanged lower envelope.
  EXPECT_EQ(reader.getPlanningInflateOccupancy({0.,0.,.22},0.),1);
}

TEST(ProjectedRayContactBounds,QuantizationContainsNegativeAndBoundaryEndpoints) {
  for(const double x:{-.100001,-.1,-.099999,-.000001,0.,.000001,.099999,.1}) {
    Eigen::Vector3d p(x,x,x);Eigen::Vector3i cell=(p/.1).array().floor().cast<int>();
    scan_planner::ObservedHitBounds record;record.include(p,cell,.1,true);
    const auto b=record.world(cell,.1);
    EXPECT_TRUE((b.first.array()<=p.array()+1e-14).all());
    EXPECT_TRUE((b.second.array()>=p.array()-1e-14).all());
    EXPECT_LE((b.second-b.first).maxCoeff(),.1/255.*2.+1e-12);
  }
}

TEST(ProjectedRaysParallel, OriginalEndpointFreeOrderPreservesContactBoundsRevisionsAndRingReuse) {
  GridMap serial,parallel;
  for(auto* map:{&serial,&parallel}) {
    GridMapTestAccess::configure(*map,false);GridMapTestAccess::contactEvidence(*map);
    GridMapTestAccess::productionProbabilities(*map);
  }
  GridMapTestAccess::serialRayTraversal(serial);
  std::vector<Eigen::Vector3d> origins(8192),ends(8192);
  std::vector<std::int64_t> stamps(8192);
  for(const auto& center:{Eigen::Vector3i::Zero().eval(),Eigen::Vector3i(-27,35,-31)}) {
    for(auto* map:{&serial,&parallel})GridMapTestAccess::inflationFixture(*map,center,false);
    for(int round=0;round<4;++round) {
      for(std::size_t i=0;i<ends.size();++i) {
        const double angle=(i*2654435761U%8192)*6.283185307179586/8192.;
        origins[i]=center.cast<double>()*.1+Eigen::Vector3d(i%2?.13:-.13,.001*(i%7),.001*(i%17));
        ends[i]=origins[i]+Eigen::Vector3d(1.65*std::cos(angle),1.65*std::sin(angle),
                                          .8*std::sin(i*.13+round*.27));
        // Exercise both local clipping and max-map clipping without fake hits.
        if(i%11==0)ends[i]=origins[i]+2.3*(ends[i]-origins[i]);
        stamps[i]=101900000000LL+round*100000000LL+(i%3?0:10000000LL);
      }
      ASSERT_TRUE(GridMapTestAccess::directAcquisitions(serial,stamps,origins,ends));
      ASSERT_TRUE(GridMapTestAccess::directAcquisitions(parallel,stamps,origins,ends));
      ASSERT_TRUE(GridMapTestAccess::usedParallelRayTraversal(parallel));
      GridMapTestAccess::expectSameContactTraversal(serial,parallel);
    }
  }
  ASSERT_TRUE(GridMapTestAccess::ownsParallelRayWorker(parallel));
  GridMap snapshot;parallel.copyCollisionSnapshotTo(snapshot,102000000000LL,std::chrono::steady_clock::now());
  EXPECT_FALSE(GridMapTestAccess::ownsParallelRayWorker(snapshot));
  EXPECT_FALSE(GridMapTestAccess::usedParallelRayTraversal(snapshot));
}

TEST(ProjectedRaysParallel, UnsafeOriginAndConservativeBudgetFallbackKeepOriginalPartialPrefix) {
  GridMap serial,parallel;
  for(auto* map:{&serial,&parallel}) {
    GridMapTestAccess::configure(*map,false);GridMapTestAccess::contactEvidence(*map);
  }
  GridMapTestAccess::serialRayTraversal(serial);
  std::vector<Eigen::Vector3d> origins(8192,Eigen::Vector3d(.01,.02,.03));
  std::vector<Eigen::Vector3d> ends(8192,Eigen::Vector3d(1.61,1.12,.87));
  std::vector<std::int64_t> stamps(8192,101900000000LL);
  origins[4097].x()=50.;
  EXPECT_FALSE(GridMapTestAccess::directAcquisitions(serial,stamps,origins,ends));
  EXPECT_FALSE(GridMapTestAccess::directAcquisitions(parallel,stamps,origins,ends));
  EXPECT_FALSE(GridMapTestAccess::usedParallelRayTraversal(parallel));
  GridMapTestAccess::expectSameContactTraversal(serial,parallel);
  // L1+2 conservatively exceeds 16M, although tied diagonal traversal itself
  // finishes below the real budget. Reject optimization, not the real rays.
  origins.assign(200000,Eigen::Vector3d(-1.81,-1.81,-1.81));
  ends.assign(200000,Eigen::Vector3d(1.81,1.81,1.81));stamps.assign(200000,102000000000LL);
  for(auto* map:{&serial,&parallel})GridMapTestAccess::fullFixtureRayRange(*map);
  ASSERT_TRUE(GridMapTestAccess::directAcquisitions(serial,stamps,origins,ends));
  ASSERT_TRUE(GridMapTestAccess::directAcquisitions(parallel,stamps,origins,ends));
  EXPECT_FALSE(GridMapTestAccess::usedParallelRayTraversal(parallel));
  GridMapTestAccess::expectSameContactTraversal(serial,parallel);
}

TEST(ProjectedRaysParallel, StrictAndDiagnosticModesNeverCreateWorkerOrChangeSourceEvidence) {
  for(bool diagnostics:{false,true}) {
    GridMap serial,parallel;
    for(auto* map:{&serial,&parallel}) {
      GridMapTestAccess::configure(*map,!diagnostics);
      if(diagnostics)GridMapTestAccess::diagnostics(*map);
    }
    GridMapTestAccess::serialRayTraversal(serial);
    const std::vector<Eigen::Vector3d> origins(4096,Eigen::Vector3d(.01,.02,.03));
    const std::vector<Eigen::Vector3d> ends(4096,Eigen::Vector3d(1.61,1.12,.87));
    const std::vector<std::int64_t> stamps(4096,101900000000LL);
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(serial,stamps,origins,ends));
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(parallel,stamps,origins,ends));
    EXPECT_FALSE(GridMapTestAccess::ownsParallelRayWorker(parallel));
    EXPECT_FALSE(GridMapTestAccess::usedParallelRayTraversal(parallel));
    GridMapTestAccess::expectSameContactTraversal(serial,parallel);
  }
}

TEST(ProjectedRaysParallel, PendingReplacementContextResetAndOriginalSourceLeasesStayIdentical) {
  GridMap serial,parallel;
  for(auto* map:{&serial,&parallel}) {
    GridMapTestAccess::configure(*map,false);GridMapTestAccess::contactEvidence(*map);
    ASSERT_TRUE(map->applyLocalizationContext(context()));
  }
  GridMapTestAccess::serialRayTraversal(serial);
  for(unsigned round=0;round<3;++round) {
    const auto source=101900000000LL+round*100000000LL,now=source+100000000LL;
    for(auto* map:{&serial,&parallel}) {
      ASSERT_TRUE(GridMapTestAccess::accept(*map,packet(0,source,round*2+1,8192),now,now*1e-9));
      ASSERT_TRUE(GridMapTestAccess::accept(*map,packet(0,source+10000000LL,round*2+2,8192),now,now*1e-9));
      ASSERT_TRUE(GridMapTestAccess::accept(*map,packet(1,source+20000000LL,round+1,8192),now,now*1e-9));
      GridMapTestAccess::integrate(*map,now,now*1e-9);
    }
    EXPECT_TRUE(GridMapTestAccess::usedParallelRayTraversal(parallel));
    GridMapTestAccess::expectSameContactTraversal(serial,parallel);
  }
  for(auto* map:{&serial,&parallel}) {
    ASSERT_TRUE(GridMapTestAccess::accept(*map,packet(0,102250000000LL,9,8192),102300000000LL,102.3));
    ASSERT_TRUE(map->applyLocalizationContext(context(2,2,"replacement")));
    EXPECT_EQ(GridMapTestAccess::pending(*map),0);
    EXPECT_EQ(map->integratedRaySourceStamp(0),0);EXPECT_EQ(map->integratedRaySourceStamp(1),0);
    EXPECT_FALSE(GridMapTestAccess::accept(*map,packet(1,102250000000LL,9,8192),102300000000LL,102.3));
    auto next=packet(0,102300000000LL,10,8192);
    next.epoch=2;next.context_sequence=2;next.seed_id="replacement";next.barrier_ns=100000000002LL;
    ASSERT_TRUE(GridMapTestAccess::accept(*map,next,102400000000LL,102.4));
    GridMapTestAccess::integrate(*map,102400000000LL,102.4);
    EXPECT_EQ(map->integratedRaySourceStamp(1),0);EXPECT_EQ(map->latestCloudStampNs(),0);
  }
  EXPECT_TRUE(GridMapTestAccess::usedParallelRayTraversal(parallel));
  GridMapTestAccess::expectSameContactTraversal(serial,parallel);
}

static auto withRayKind(d1max_planning_interfaces::msg::ProjectedRays message,std::uint8_t kind) {
  auto& cloud=message.rays;const auto old=cloud.data;
  cloud.point_step=68;cloud.row_step=cloud.width*68;cloud.data.assign(cloud.row_step,0);
  using F=sensor_msgs::msg::PointField;
  struct Extra {const char* name;std::uint32_t offset;std::uint8_t type;};
  for(const auto& extra:std::array<Extra,7>{{{"intensity",12,F::FLOAT32},{"ring",30,F::UINT16},
      {"offset_time",32,F::UINT32},{"source_index",36,F::UINT32},{"timestamp",40,F::FLOAT64},
      {"source_timestamp",48,F::FLOAT64},{"raw_timestamp",56,F::FLOAT64}}}) {
    F field;field.name=extra.name;field.offset=extra.offset;field.datatype=extra.type;field.count=1;
    cloud.fields.push_back(field);
  }
  sensor_msgs::msg::PointField field;field.name="ray_kind";field.offset=64;
  field.datatype=sensor_msgs::msg::PointField::UINT8;field.count=1;cloud.fields.push_back(field);
  for(std::size_t i=0;i<cloud.width;++i) {
    std::memcpy(cloud.data.data()+i*68,old.data()+i*64,64);cloud.data[i*68+64]=kind;
  }
  return message;
}

static bool cloudContains(const sensor_msgs::msg::PointCloud2& cloud,const Eigen::Vector3d& point) {
  if(!cloud.width)return false;
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  for(;x!=x.end();++x,++y,++z)
    if((Eigen::Vector3d(*x,*y,*z)-point).norm()<1e-4)return true;
  return false;
}

TEST(ProjectedRaysDynamicClear, DualSensorRoiCannotDiscardActualMissesOrKeepStaleDisplay) {
  // First one source saturates a real obstacle; when the second source resumes,
  // its origin shifts the ROI centre. Every later front ray still crosses the
  // obstacle. The old second crop incorrectly dropped all of those miss votes.
  for(const double direction:{1.,-1.})for(const bool parallel:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionProbabilities(map);
    // Upstream's strict '< radius' discrete kernel is empty at radius zero;
    // use the existing one-cell official fixture, not a degenerate envelope.
    GridMapTestAccess::centerOnlyOfficialGeometry(map);GridMapTestAccess::displayHeight(map);
    if(!parallel)GridMapTestAccess::serialRayTraversal(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    const Eigen::Vector3d front(direction*.3,.05,.05),rear(-direction*.5,.05,.05);
    const Eigen::Vector3d obstacle(direction*1.95,.05,.05),background(direction*3.95,.05,.05);
    std::int64_t stamp=101000000000LL;std::uint64_t sequence=0;
    const auto fuse=[&](bool hit,bool both) {
      stamp+=100000000LL;++sequence;
      EXPECT_TRUE(GridMapTestAccess::accept(map,packet(0,stamp,sequence,parallel?4096:1,
          front,hit?obstacle:background),stamp+50000000LL,(stamp+50000000LL)*1e-9));
      if(both) {EXPECT_TRUE(GridMapTestAccess::accept(map,packet(1,stamp,sequence,1,rear,
          {-direction*1.5,.05,.05}),stamp+50000000LL,(stamp+50000000LL)*1e-9));}
      GridMapTestAccess::integrate(map,stamp+50000000LL,(stamp+50000000LL)*1e-9);
      EXPECT_EQ(GridMapTestAccess::usedParallelRayTraversal(map),parallel);
    };
    for(int i=0;i<5;++i)fuse(true,false);
    ASSERT_EQ(map.getOccupancy(obstacle),1);
    GridMap old;map.copyCollisionSnapshotTo(old,stamp,std::chrono::steady_clock::now());
    for(int miss=1;miss<=3;++miss) {
      fuse(false,true);const bool occupied=miss<3;
      const auto cell=GridMapTestAccess::index(map,obstacle);
      EXPECT_EQ(GridMapTestAccess::freeStamp(map,cell),stamp);
      EXPECT_EQ(map.getOccupancy(obstacle),occupied?1:0);
      EXPECT_EQ(map.getPlanningInflateOccupancy(obstacle,0.),occupied?1:0);
      GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,stamp,std::chrono::steady_clock::now());
      EXPECT_EQ(snapshot.getPlanningInflateOccupancy(obstacle,0.),occupied?1:0);
      EXPECT_EQ(cloudContains(GridMapTestAccess::display(map,false,stamp),obstacle),occupied);
      EXPECT_EQ(cloudContains(GridMapTestAccess::display(map,true,stamp),obstacle),occupied);
      EXPECT_EQ(old.getOccupancy(obstacle),1); // Retained readers remain immutable.
    }
  }
}

TEST(ProjectedRaysDynamicClear, NewRaysElsewhereAndClockAdvanceDoNotEraseUnobservedObstacle) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionProbabilities(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  std::int64_t stamp=101000000000LL;
  for(unsigned round=1;round<=13;++round) {
    stamp+=100000000LL;
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,stamp,round,1,{.05,.05,.05},
        round<=5?Eigen::Vector3d(.95,.05,.05):Eigen::Vector3d(.95,.95,.05)),
        stamp+50000000LL,(stamp+50000000LL)*1e-9));
    GridMapTestAccess::integrate(map,stamp+50000000LL,(stamp+50000000LL)*1e-9);
  }
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.95,.05,.05)),1);
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
}

TEST(ProjectedRaysCensored, MeasuredPrefixClearsGhostButEndpointAndOccludedSpaceStayUnobserved) {
  for(const bool parallel:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionProbabilities(map);
    if(!parallel)GridMapTestAccess::serialRayTraversal(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));std::int64_t stamp=101000000000LL;
    for(unsigned round=1;round<=8;++round) {
      stamp+=100000000LL;auto message=packet(0,stamp,round,parallel?4096:1,{.05,.05,.05},
          round<=5?Eigen::Vector3d(.95,.05,.05):Eigen::Vector3d(1.55,.05,.05));
      if(round>5)message=withRayKind(message,1);
      ASSERT_TRUE(GridMapTestAccess::accept(map,message,stamp+50000000LL,(stamp+50000000LL)*1e-9));
      GridMapTestAccess::integrate(map,stamp+50000000LL,(stamp+50000000LL)*1e-9);
      EXPECT_EQ(GridMapTestAccess::usedParallelRayTraversal(map),parallel);
      if(round==5) {ASSERT_EQ(map.getOccupancy(Eigen::Vector3d(.95,.05,.05)),1);}
    }
    EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.95,.05,.05)),0);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{9,0,0}),stamp);
    EXPECT_EQ(GridMapTestAccess::category(map,{15,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(GridMapTestAccess::category(map,{16,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{15,0,0}),0);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{16,0,0}),0);
  }
}

TEST(ProjectedRaysCensored, PreviouslyOccupiedEndpointIsNotClearedAndPeerRealHitWins) {
  GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionProbabilities(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));std::int64_t stamp=101000000000LL;
  for(unsigned round=1;round<=13;++round) {
    stamp+=100000000LL;auto message=packet(0,stamp,round,1,{.05,.05,.05},{1.55,.05,.05});
    if(round>5)message=withRayKind(message,1);
    ASSERT_TRUE(GridMapTestAccess::accept(map,message,stamp+50000000LL,(stamp+50000000LL)*1e-9));
    ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1,stamp,round,1,{-.05,.05,.05},{.95,.05,.05}),
        stamp+50000000LL,(stamp+50000000LL)*1e-9));
    GridMapTestAccess::integrate(map,stamp+50000000LL,(stamp+50000000LL)*1e-9);
  }
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(1.55,.05,.05)),1); // Censoring is not a clearing operation.
  EXPECT_EQ(GridMapTestAccess::freeStamp(map,{15,0,0}),0);
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(.95,.05,.05)),1); // Real measured hit dominates same-round miss.
}

TEST(ProjectedRaysCensored, ClipsInsideOriginalEndpointVoxelDoNotInventFreeEvidence) {
  for(const bool max_range:{false,true})for(const bool parallel:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);GridMapTestAccess::productionProbabilities(map);
    if(!parallel)GridMapTestAccess::serialRayTraversal(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    const double distance=1.01;
    GridMapTestAccess::clippingFixture(map,max_range?1.9:distance,max_range?distance:4.);
    auto message=withRayKind(packet(0,101900000000LL,1,parallel?4096:1,
        {.05,.05,.05},{1.09,.05,.05}),1);
    ASSERT_TRUE(GridMapTestAccess::accept(map,message));GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::usedParallelRayTraversal(map),parallel);
    EXPECT_GT(GridMapTestAccess::freeStamp(map,{9,0,0}),0);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{10,0,0}),0);
    EXPECT_EQ(GridMapTestAccess::category(map,{10,0,0}),scan_planner::RawVoxelDiagnostic::NeverObserved);
  }
}

TEST(ProjectedRaysCensored, EarlierClippedVoxelIsFreeButNoEvidenceBeyondIt) {
  for(const bool parallel:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map,false);
    if(!parallel)GridMapTestAccess::serialRayTraversal(map);
    ASSERT_TRUE(map.applyLocalizationContext(context()));
    GridMapTestAccess::clippingFixture(map,1.9,1.01);
    auto message=withRayKind(packet(0,101900000000LL,1,parallel?4096:1,
        {.05,.05,.05},{1.55,.05,.05}),1);
    ASSERT_TRUE(GridMapTestAccess::accept(map,message));GridMapTestAccess::integrate(map);
    EXPECT_EQ(GridMapTestAccess::usedParallelRayTraversal(map),parallel);
    EXPECT_GT(GridMapTestAccess::freeStamp(map,{10,0,0}),0);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{11,0,0}),0);
    EXPECT_EQ(GridMapTestAccess::freeStamp(map,{15,0,0}),0);
  }
}

TEST(ProjectedRaysCensored, MixedKindsParallelMatchesSerialAcrossClipsAndNegativeCells) {
  GridMap serial,parallel;
  for(auto* map:{&serial,&parallel}) {
    GridMapTestAccess::configure(*map,false);GridMapTestAccess::contactEvidence(*map);
    GridMapTestAccess::productionProbabilities(*map);
  }
  GridMapTestAccess::serialRayTraversal(serial);
  std::vector<Eigen::Vector3d> origins(8192),ends(8192);
  std::vector<std::int64_t> stamps(8192);
  std::vector<std::uint8_t> kinds(8192);
  for(int round=0;round<4;++round) {
    for(std::size_t i=0;i<ends.size();++i) {
      const double angle=(i*2654435761U%8192)*6.283185307179586/8192.;
      origins[i]={i%2?.13:-.13,.001*(i%7),.001*(i%17)};
      ends[i]=origins[i]+Eigen::Vector3d(1.65*std::cos(angle),1.65*std::sin(angle),
          .8*std::sin(i*.13+round*.27));
      if(i%11==0)ends[i]=origins[i]+2.3*(ends[i]-origins[i]);
      stamps[i]=101900000000LL+round*100000000LL+(i%3?0:10000000LL);
      kinds[i]=(i+round)%3==0?1:0;
    }
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(serial,stamps,origins,ends,kinds));
    ASSERT_TRUE(GridMapTestAccess::directAcquisitions(parallel,stamps,origins,ends,kinds));
    ASSERT_TRUE(GridMapTestAccess::usedParallelRayTraversal(parallel));
    GridMapTestAccess::expectSameContactTraversal(serial,parallel);
  }
}

static const scan_planner::BlockedVoxelWatchCell* watchedCell(
    const scan_planner::BlockedVoxelWatchSnapshot& snapshot,const Eigen::Vector3i& key) {
  for(const auto& cell:snapshot.cells)
    if(cell.count && cell.key==scan_planner::BlockedVoxelKey{{key.x(),key.y(),key.z()}})return &cell;
  return nullptr;
}

TEST(BlockedVoxelIntegration, EnabledWatchIsBitIdenticalAndDoesNotDisableParallelTraversal) {
  GridMap off,on;
  for(auto* map:{&off,&on}) {
    GridMapTestAccess::configure(*map,false);
    GridMapTestAccess::contactEvidence(*map);
    GridMapTestAccess::productionProbabilities(*map);
    ASSERT_TRUE(map->applyLocalizationContext(context()));
  }
  GridMapTestAccess::disableBlockedWatch(off);
  const Eigen::Vector3d obstacle(1.55,.05,.05),background(1.85,.05,.05);
  const auto key=GridMapTestAccess::index(on,obstacle);
  std::int64_t stamp=101000000000LL;
  for(unsigned round=1;round<=10;++round) {
    stamp+=100000000LL;
    for(auto* map:{&off,&on}) {
      for(unsigned sensor=0;sensor<2;++sensor)
        ASSERT_TRUE(GridMapTestAccess::accept(*map,packet(sensor,stamp+sensor*1000000LL,
          round,4096,{.55-.1*sensor,.05,.05},round<=5?obstacle:background),
          stamp+50000000LL,(stamp+50000000LL)*1e-9));
      GridMapTestAccess::integrate(*map,stamp+50000000LL,(stamp+50000000LL)*1e-9);
      ASSERT_TRUE(GridMapTestAccess::usedParallelRayTraversal(*map));
      EXPECT_FALSE(GridMapTestAccess::nearFieldEnabled(*map));
      if(round==3) {
        ASSERT_EQ(map->getPlanningInflateOccupancy(obstacle,0.),1);
        EXPECT_NE(map->describeInflateOccupancy(obstacle,0.).find("collision_occupied_cell="),std::string::npos);
      }
    }
    GridMapTestAccess::expectSameContactTraversal(off,on);
    GridMapTestAccess::expectSameInflation(off,on);
    EXPECT_EQ(GridMapTestAccess::display(off,false,stamp).data,
              GridMapTestAccess::display(on,false,stamp).data);
    EXPECT_EQ(GridMapTestAccess::display(off,true,stamp).data,
              GridMapTestAccess::display(on,true,stamp).data);
    if(round==8) {
      const auto snapshot=GridMapTestAccess::blockedWatch(on);
      const auto* cell=watchedCell(snapshot,key);ASSERT_NE(cell,nullptr);
      ASSERT_NE(cell->latest(),nullptr);
      EXPECT_EQ(cell->latest()->transition,scan_planner::BlockedVoxelTransition::Cleared);
      EXPECT_EQ(cell->latest()->outcome,scan_planner::BlockedVoxelUpdate::Applied);
      EXPECT_EQ(cell->latest()->hit_votes,0U);EXPECT_EQ(cell->latest()->miss_votes,1U);
      EXPECT_EQ(cell->latest()->fusion.source_stamps_ns[0],stamp);
      EXPECT_EQ(cell->latest()->fusion.source_stamps_ns[1],stamp+1000000LL);
      EXPECT_EQ(cell->latest()->fusion.projection_sequences[0],round);
      EXPECT_EQ(cell->latest()->fusion.projection_sequences[1],round);
    }
  }
  EXPECT_EQ(GridMapTestAccess::blockedWatch(off).history_revision,0U);
  const auto report=GridMapTestAccess::blockedJson(on);
  EXPECT_EQ(report["vote_semantics"],"deduplicated_per_fusion_not_raw_ray_counts");
  EXPECT_EQ(report["sources_semantics"],"fusion_inputs_not_per_cell_ray_attribution");
  EXPECT_EQ(report["prewatch_history"],"unavailable");
  bool retained_clear=false;
  for(const auto& cell:report["cells"])for(const auto& event:cell["events"])
    retained_clear=retained_clear||event["transition"]=="cleared";
  EXPECT_TRUE(retained_clear);
  EXPECT_EQ(on.getOccupancy(obstacle),0);
}

TEST(BlockedVoxelIntegration, SnapshotRequestsWakeWatchAndSnapshotsRemainImmutableAfterClear) {
  GridMap writer;GridMapTestAccess::configure(writer,false);
  GridMapTestAccess::centerOnlyOfficialGeometry(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(context()));
  const Eigen::Vector3d obstacle(1.55,.05,.05),background(1.85,.05,.05);
  ASSERT_TRUE(GridMapTestAccess::accept(writer,packet()));GridMapTestAccess::integrate(writer);
  GridMap reader;writer.copyCollisionSnapshotTo(reader,102000000000LL,GridMapTestAccess::at(102000000000LL));
  EXPECT_NE(reader.describeInflateOccupancy(obstacle,0.).find("collision_occupied_cell="),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::blockedWatch(writer).history_revision,0U);
  for(unsigned i=1;i<=3;++i) {
    const auto stamp=101900000000LL+i*100000000LL,now=stamp+50000000LL;
    ASSERT_TRUE(GridMapTestAccess::accept(writer,packet(0,stamp,i+1,1,{.55,.05,.05},background),now,now*1e-9));
    GridMapTestAccess::integrate(writer,now,now*1e-9);
  }
  const auto snapshot=GridMapTestAccess::blockedWatch(writer);
  const auto* cell=watchedCell(snapshot,{15,0,0});ASSERT_NE(cell,nullptr);
  EXPECT_EQ(cell->count,3U);EXPECT_TRUE(cell->active);
  bool cleared=false;for(const auto& value:cell->samples)
    cleared=cleared||value.transition==scan_planner::BlockedVoxelTransition::Cleared;
  EXPECT_TRUE(cleared);
  EXPECT_EQ(reader.getOccupancy(obstacle),1); // Snapshot is immutable, not silently refreshed.
  EXPECT_EQ(GridMapTestAccess::blockedWatch(reader).history_revision,0U);
  GridMap current;writer.copyCollisionSnapshotTo(current,102250000000LL,GridMapTestAccess::at(102250000000LL));
  EXPECT_EQ(current.getOccupancy(obstacle),0);
  EXPECT_EQ(GridMapTestAccess::blockedWatch(current).history_revision,snapshot.history_revision);
}

TEST(BlockedVoxelIntegration, OldSnapshotWatchRequestCannotCrossResetIdentity) {
  GridMap writer;GridMapTestAccess::configure(writer,false);
  GridMapTestAccess::centerOnlyOfficialGeometry(writer);
  ASSERT_TRUE(writer.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(writer,packet()));GridMapTestAccess::integrate(writer);
  GridMap old;writer.copyCollisionSnapshotTo(old,102000000000LL,GridMapTestAccess::at(102000000000LL));
  const auto old_context=GridMapTestAccess::blockedWatch(writer).context_generation;
  GridMapTestAccess::resetForBlockedWatch(writer);
  EXPECT_GT(GridMapTestAccess::blockedWatch(writer).context_generation,old_context);
  EXPECT_NE(old.describeInflateOccupancy({1.55,.05,.05},0.).find("collision_occupied_cell="),std::string::npos);
  const auto stamp=102000000000LL,now=102050000000LL;
  ASSERT_TRUE(GridMapTestAccess::accept(writer,packet(0,stamp,2),now,now*1e-9));
  GridMapTestAccess::integrate(writer,now,now*1e-9);
  const auto snapshot=GridMapTestAccess::blockedWatch(writer);
  EXPECT_EQ(snapshot.history_revision,0U);
  for(const auto& cell:snapshot.cells)EXPECT_FALSE(cell.active);
  // A fresh snapshot in the new context can still request diagnostics normally.
  GridMap current;writer.copyCollisionSnapshotTo(current,now,GridMapTestAccess::at(now));
  current.describeInflateOccupancy({1.55,.05,.05},0.);
  ASSERT_TRUE(GridMapTestAccess::accept(writer,packet(0,stamp+100000000LL,3),now+100000000LL,(now+100000000LL)*1e-9));
  GridMapTestAccess::integrate(writer,now+100000000LL,(now+100000000LL)*1e-9);
  EXPECT_GT(GridMapTestAccess::blockedWatch(writer).history_revision,0U);
}

TEST(BlockedVoxelIntegration, NoObservationDoesNotClaimMissVotesOrReuseAbsentSourceTimestamp) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::centerOnlyOfficialGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0)));
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(1)));
  GridMapTestAccess::integrate(map);
  map.describeInflateOccupancy({1.55,.05,.05},0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102000000000LL,2,1,{-.55,.05,.05},{-1.55,.05,.05}),
                                      102050000000LL,102.05));
  GridMapTestAccess::integrate(map,102050000000LL,102.05);
  const auto snapshot=GridMapTestAccess::blockedWatch(map);
  const auto* cell=watchedCell(snapshot,{15,0,0});ASSERT_NE(cell,nullptr);
  const auto* sample=cell->latest();ASSERT_NE(sample,nullptr);
  EXPECT_EQ(sample->outcome,scan_planner::BlockedVoxelUpdate::NoObservation);
  EXPECT_EQ(sample->hit_votes,0U);EXPECT_EQ(sample->miss_votes,0U);
  EXPECT_TRUE(sample->after.occupied);
  EXPECT_DOUBLE_EQ(sample->before.log_odds,sample->after.log_odds);
  EXPECT_EQ(sample->fusion.source_stamps_ns[0],102000000000LL);
  EXPECT_EQ(sample->fusion.projection_sequences[0],2U);
  EXPECT_EQ(sample->fusion.source_stamps_ns[1],0);
  EXPECT_EQ(sample->fusion.projection_sequences[1],0U);
}

TEST(BlockedVoxelIntegration, SlideDoesNotAttachOldWorldVoxelHistoryToRingAddressAlias) {
  GridMap map;GridMapTestAccess::configure(map,false);
  GridMapTestAccess::centerOnlyOfficialGeometry(map);
  ASSERT_TRUE(map.applyLocalizationContext(context()));
  const Eigen::Vector3d old_point(-1.45,.05,.05),new_point(2.55,.05,.05);
  const Eigen::Vector3i old_key(-15,0,0),new_key(25,0,0);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,101900000000LL,1,1,{-.55,.05,.05},old_point)));
  GridMapTestAccess::integrate(map);map.describeInflateOccupancy(old_point,0.);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102000000000LL,2,1,{-.55,.05,.05},old_point),
                                      102050000000LL,102.05));
  GridMapTestAccess::integrate(map,102050000000LL,102.05);
  const auto old_address=GridMapTestAccess::address(map,old_key);
  GridMapTestAccess::slide(map,{2.05,.05,.05});
  EXPECT_EQ(GridMapTestAccess::address(map,new_key),old_address);
  ASSERT_TRUE(GridMapTestAccess::accept(map,packet(0,102100000000LL,3,1,{2.05,.05,.05},new_point),
                                      102150000000LL,102.15));
  GridMapTestAccess::integrate(map,102150000000LL,102.15);
  const auto snapshot=GridMapTestAccess::blockedWatch(map);
  const auto* cell=watchedCell(snapshot,old_key);ASSERT_NE(cell,nullptr);
  EXPECT_FALSE(cell->active);
  EXPECT_EQ(cell->latest()->transition,scan_planner::BlockedVoxelTransition::SlidOut);
  EXPECT_EQ(cell->latest()->outcome,scan_planner::BlockedVoxelUpdate::SlidOut);
  EXPECT_EQ(cell->latest()->hit_votes,0U);
  EXPECT_EQ(cell->latest()->miss_votes,0U);
  EXPECT_EQ(watchedCell(snapshot,new_key),nullptr);
  EXPECT_EQ(map.getOccupancy(new_point),1);
}
