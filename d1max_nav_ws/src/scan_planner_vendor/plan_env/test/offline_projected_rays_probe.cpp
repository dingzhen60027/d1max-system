// Read-only real-bag replay of actual GridMap methods. No ROS init/node/graph.
#include <plan_env/grid_map.h>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <iomanip>
#include <sstream>

using Json=nlohmann::json;
template<class T> static Json xyz(const T &p) {return Json::array({p.x(),p.y(),p.z()});}
static Json witnessJson(const scan_planner::RayWitness &w) {
  const auto &m=w.metadata;
  Json out={{"sensor_id",m.sensor_id},{"scan_stamp_ns",m.scan_stamp_ns},{"received_ns",m.received_ns},
    {"integration_call_start_ns",m.integration_ns},{"context_sequence",m.context_sequence},
    {"alignment_stamp_ns",m.alignment_stamp_ns},{"acquisition_end_ns",m.acquisition_end_ns},
    {"projection_sequence",m.projection_sequence},{"origin",xyz(w.origin)},
    {"measured_endpoint",xyz(w.endpoint)},{"integrated_endpoint",xyz(w.integrated_endpoint)},
    {"hit",w.hit},{"contributed_vote",w.contributed_vote},{"source_fields_available",m.source_fields_available}};
  if(m.source_fields_available) out.update({{"source_index",m.source_index},{"ring",m.ring},
    {"offset_time_ns",m.offset_time_ns},{"point_stamp_ns",m.scan_stamp_ns+m.offset_time_ns},
    {"timestamp",m.timestamp},{"source_timestamp",m.source_timestamp},{"raw_timestamp",m.raw_timestamp}});
  else out["missing"]="source_index/ring/point_time unavailable; not inferred";
  return out;
}
struct GridMapTestAccess {
  static void freshness(GridMap &map,bool enabled) {map.enforce_free_freshness_=enabled;}
  static void queryClock(GridMap &map,std::int64_t now) {
    if(now>0) map.ray_query_clock_ns_=now;
    map.beginCollisionQuery();
  }
  static void configure(GridMap &map,const Json &v) {
    auto &p=map.mp_;auto &d=map.md_;
    auto number=[&](const char *key){return v.at(std::string("grid_map.")+key).get<double>();};
    p.use_projected_rays_=v.value("grid_map.use_projected_rays",true);
    p.require_localization_context_=true;
    p.require_observed_free_=v.value("grid_map.require_observed_free",true);
    p.preview_only_=v.value("grid_map.preview_only",false);
    p.localization_session_id_=v.at("grid_map.localization_session_id").get<std::string>();
    p.frame_id_=v.at("grid_map.frame_id").get<std::string>();p.sensor_type_="lidar";
    p.resolution_=number("resolution");p.resolution_inv_=1./p.resolution_;
    p.map_size_={number("sliding_map_size_x"),number("sliding_map_size_y"),number("sliding_map_size_z")};
    if(p.resolution_<.01 || p.resolution_>.5 || !p.map_size_.allFinite() ||
       p.map_size_.minCoeff()<=0 || p.map_size_.maxCoeff()>30) throw std::runtime_error("unsafe fixture map bounds");
    p.local_update_range_={number("local_update_range_x"),number("local_update_range_y"),number("local_update_range_z")};
    p.map_sliding_en_=v.at("grid_map.map_sliding_en").get<bool>();p.map_sliding_thresh_=number("map_sliding_thresh");
    p.map_sliding_thresh_vox_=std::max(1,static_cast<int>(std::ceil(p.map_sliding_thresh_*p.resolution_inv_)));
    p.ground_height_=number("ground_height");p.map_origin_={-p.map_size_.x()/2.,-p.map_size_.y()/2.,p.ground_height_};
    for(int i=0;i<3;++i) p.map_voxel_num_[i]=std::ceil(p.map_size_[i]/p.resolution_);
    map.posToIndex(p.map_origin_,p.map_bound_min_idx_);
    p.map_origin_idx_=p.map_bound_min_idx_+p.map_voxel_num_/2;map.updateMapBoundaryFromIndex();
    const auto odds=[](double probability){return std::log(probability/(1.-probability));};
    p.clamp_min_log_=odds(number("p_min"));p.clamp_max_log_=odds(number("p_max"));
    p.prob_hit_log_=odds(number("p_hit"));p.prob_miss_log_=odds(number("p_miss"));
    p.min_occupancy_log_=odds(number("p_occ"));p.unknown_flag_=.01;
    p.max_ray_length_=number("max_ray_length");p.double_cylinder_radius_=number("double_cylinder_radius");
    p.double_cylinder_offset_=number("double_cylinder_offset");
    p.obstacles_inflation_z_up=number("obstacles_inflation_z_up");p.obstacles_inflation_z_down=number("obstacles_inflation_z_down");
    p.cloud_pose_max_age_=v.value("grid_map.cloud_pose_max_age",.5);
    p.cloud_pose_pair_wait_=v.value("grid_map.cloud_pose_pair_wait",.25);p.vis_height_=number("vis_height");
    scan_planner::validateCloudPoseTiming(p.cloud_pose_pair_wait_,p.cloud_pose_max_age_,
        p.preview_only_,p.require_observed_free_,p.use_projected_rays_);
    d.has_ray_pose_=d.has_cloud_=d.occ_need_update_=d.use_cloud_update_=false;
    d.ray_pos_.setZero();d.ray_q_=Eigen::Quaterniond::Identity();d.raycast_num_=0;
    const auto size=static_cast<std::size_t>(p.map_voxel_num_.prod());
    if(size>8000000) throw std::runtime_error("fixture exceeds bounded voxel budget");
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);d.count_hit_and_miss_.resize(size);
    d.flag_rayend_.resize(size);d.flag_traverse_.resize(size);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static auto clock(std::int64_t ns) {return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));}
  static bool accept(GridMap &map,const d1max_planning_interfaces::msg::ProjectedRays &message,std::int64_t ns) {
    return map.acceptProjectedRays(message,ns,clock(ns));
  }
  static void integrate(GridMap &map,std::int64_t ns) {map.processProjectedRays(ns,clock(ns));}
  static void slide(GridMap &map,const Json &p) {
    map.updateSlidingMap({p.at(0).get<double>(),p.at(1).get<double>(),p.at(2).get<double>()});
  }
  static Json voxel(GridMap &map,const Eigen::Vector3i &cell) {
    const bool inside=map.isInMap(cell);
    const double odds=inside?map.md_.occupancy_buffer_[map.toAddress(cell)]:NAN;
    const auto kind=inside ? scan_planner::diagnoseRawVoxel(odds,map.mp_.clamp_min_log_,
        map.mp_.min_occupancy_log_,map.mp_.unknown_flag_):scan_planner::RawVoxelDiagnostic::Outside;
    Json out={{"global_index",xyz(cell)},{"log_odds",std::isfinite(odds)?Json(odds):Json(nullptr)},
      {"native_state",inside?scan_planner::strictRawVoxelStatus(odds,map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_):-1},
      {"classification",scan_planner::diagnosticName(kind)},
      {"aabb_min",xyz(cell.cast<double>()*map.mp_.resolution_)},
      {"aabb_max",xyz((cell.cast<double>().array()+1.).matrix()*map.mp_.resolution_)}};
    out["effective_state"]=map.rawCollisionStatus(cell);
    const auto free_stamp=inside && static_cast<std::size_t>(map.toAddress(cell))<map.free_observation_stamps_.size()?
        map.free_observation_stamps_[map.toAddress(cell)]:0;
    out["free_observation_scan_begin_ns"]=free_stamp;
    out["free_evidence_age_s"]=free_stamp>0 && map.collision_cache_clock_ns_>0?
        Json((map.collision_cache_clock_ns_-free_stamp)*1e-9):Json(nullptr);
    out["free_evidence_max_age_s"]=map.mp_.cloud_pose_max_age_;
    if(out["native_state"]==0 && out["effective_state"]==2) out["classification"]="stale_free";
    const auto &diag=map.near_field_diagnostics_;
    const auto *history=diag.find(cell);
    out["witnesses"]=Json::array();
    if(history) {
      out["ray_visits_hit_miss_by_source"]=history->visits;
      out["deduplicated_votes_hit_miss_by_source"]=history->votes;
      for(const auto &w:history->last) if(w) {
        auto witness=witnessJson(*w);
        witness["acquisition_source_age_s"]=w->metadata.acquisition_end_ns>0&&map.collision_cache_clock_ns_>0?
          Json((map.collision_cache_clock_ns_-w->metadata.acquisition_end_ns)*1e-9):Json(nullptr);
        witness["receipt_age_s_diagnostic_only"]=w->metadata.received_ns>0&&map.collision_cache_clock_ns_>0?
          Json((map.collision_cache_clock_ns_-w->metadata.received_ns)*1e-9):Json(nullptr);
        out["witnesses"].push_back(std::move(witness));
      }
    }
    out["provenance_status"]=history?"bounded_recent_witnesses":!diag.enabled()?"disabled":
        !diag.contains(cell)?"outside_diagnostic_roi":"no_retained_witness";
    return out;
  }
  static Json state(GridMap &map,bool summary=false) {
    Json out={{"source_stamp_ns",map.integrated_cloud_stamp_ns_},{"integrated_counts",map.ray_integrations_},
      {"source_stamps_ns",map.ray_integrated_stamps_},{"drops",map.ray_drops_},
      {"unattributed_drops",map.ray_unattributed_drops_},{"revision",map.occupancyRevision()}};
    out["raw_occupancy_revision"]=map.occupancy_revision_;
    out["free_evidence_revision"]=map.free_evidence_revision_;
    if(summary) {
      out["require_observed_free"]=map.requiresObservedFree();
      out["query_policy"]=map.collisionQueryPolicy();
      out["enforce_free_freshness"]=map.enforce_free_freshness_;
      out["free_evidence_memory_bytes"]=map.free_observation_stamps_.capacity()*sizeof(std::int64_t);
      std::array<std::size_t,3> cells{{0,0,0}};
      for(double value:map.md_.occupancy_buffer_) ++cells[scan_planner::strictRawVoxelStatus(
          value,map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_)];
      out["raw_voxel_counts_free_occupied_unknown"]=cells;
      // Diagnostic equality digest, not a cryptographic artifact identity.
      std::uint64_t hash=14695981039346656037ULL;
      for(double value:map.md_.occupancy_buffer_) {
        const auto *p=reinterpret_cast<const unsigned char*>(&value);
        for(std::size_t i=0;i<sizeof(value);++i) {hash^=p[i];hash*=1099511628211ULL;}
      }
      std::ostringstream digest;digest<<std::hex<<hash;out["raw_buffer_fnv1a64"]=digest.str();
    }
    return out;
  }
  static Json query(GridMap &map,const Json &v,std::int64_t now_ns,const std::string &config_hash) {
    const auto p=v.at("position").get<std::vector<double>>();const double yaw=v.at("yaw").get<double>();
    if(p.size()!=3) throw std::runtime_error("query must have XYZ");
    const bool detailed=v.value("detailed",false);
    const Eigen::Vector3d body(p[0],p[1],p[2]);const auto detail=map.inspectInflateOccupancy(body,yaw,detailed);
    Json first=Json::array();
    for(std::size_t i=0;i<detail.first.size();++i) {
      if(detail.counts[i+1]==0) {first.push_back(nullptr);continue;}
      const auto &index=detail.first[i];
      const auto xyz=(index.cast<double>().array()+.5)*map.mp_.resolution_;
      first.push_back({xyz.x(),xyz.y(),xyz.z()});
    }
    Json result={{"label",v.value("label","query")},{"position",p},{"yaw",yaw},
      {"collision",map.getInflateOccupancy(body,yaw)},
      {"query_policy",map.collisionQueryPolicy()},
      {"counts_semantics",map.requiresObservedFree()?"strict_with_free_expiry":"raw_evidence"},
      {"counts_free_occupied_unknown_outside",detail.counts},{"first_occupied_unknown_outside",first}};
    if(!detailed) return result;
    const auto &d=map.near_field_diagnostics_;
    result["query_source"]=v.value("query_source","offline_projected_rays_probe");
    result["query_now_ns"]=now_ns;
    result["query_source_stamp_ns"]=v.value("source_stamp_ns",Json(nullptr));
    result["diagnostic_wall_unix_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    result["identity"]={{"session_id",map.mp_.localization_session_id_},{"epoch",map.localization_epoch_},
      {"seed_id",map.localization_seed_},{"context_sequence",map.localization_context_sequence_},
      {"barrier_ns",map.localization_context_barrier_ns_},{"map_revision",map.occupancyRevision()}};
    result["effective_parameters_sha256"]=config_hash.empty()?Json(nullptr):Json(config_hash);
    result["source_stamps_ns"]=map.ray_integrated_stamps_;
    result["source_age_s"]=Json::array();
    for(auto stamp:map.ray_integrated_stamps_) result["source_age_s"].push_back(stamp>0&&now_ns>0?
        Json((now_ns-stamp)*1e-9):Json(nullptr));
    result["integrated_counts"]=map.ray_integrations_;
    result["unique_counts_free_occupied_unknown_outside"]=detail.unique_counts;
    result["raw_overlap_count"]=detail.counts;
    result["unique_voxel_count"]=detail.voxels.size();
    result["diagnostic_counts"]=Json::object();
    for(const auto *name:{"observed_free","occupied","never_observed","observed_insufficient","outside","invalid","stale_free"})
      result["diagnostic_counts"][name]=0;
    const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
    const auto body_envelope=map.bodyEnvelope();const auto kernel=map.obstacleDilation();
    result["collision_geometry"]={{"actual_query_yaw",yaw},{"radius",body_envelope.radius},
      {"rear_center",xyz(body-map.mp_.double_cylinder_offset_*heading)},
      {"front_center",xyz(body+map.mp_.double_cylinder_offset_*heading)},
      {"z_min",body.z()-body_envelope.below},{"z_max",body.z()+body_envelope.above},
      {"body_below_m",body_envelope.below},{"body_above_m",body_envelope.above},
      {"obstacle_dilation_up_m",kernel.up},{"obstacle_dilation_down_m",kernel.down},
      {"vertical_contract","body_is_reflected_obstacle_dilation"},
      {"model","upright_double_cylinder_yaw_only"},{"physical_envelope_validated",false}};
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
    const bool measured_orientation=v.contains("orientation_xyzw");
    if(measured_orientation) {
      const auto a=v.at("orientation_xyzw").get<std::vector<double>>();
      if(a.size()!=4) throw std::runtime_error("orientation requires xyzw");
      q=Eigen::Quaterniond(a[3],a[0],a[1],a[2]);
      if(!q.coeffs().allFinite() || std::abs(q.norm()-1.)>1e-3) throw std::runtime_error("invalid measured orientation");
      q.normalize();
    }
    result["body_orientation_xyzw"]=measured_orientation?v.at("orientation_xyzw"):Json(nullptr);
    result["body_coordinates_basis"]=measured_orientation?"full_recorded_orientation":"yaw_only_roll_pitch_missing";
    result["measured_ground_z"]=v.value("measured_ground_z",Json(nullptr));
    result["verified_entity_self_geometry"]=nullptr;
    result["safety_margin_vs_entity_classification"]="unresolved_without_physical_measurement";
    result["diagnostic_storage"]={{"enabled",d.enabled()},{"generation",d.generation()},
      {"center",xyz(d.center())},{"half_extent",xyz(d.halfExtent())},{"cells",d.size()},
      {"cell_limit",d.capacity()},{"events_per_integration_limit",d.eventLimit()},
      {"dropped_events",d.droppedEvents()},{"dropped_cells",d.droppedCells()},
      {"history_is_complete",false}};
    result["voxels"]=Json::array();
    for(const auto &cell:detail.voxels) {
      const auto name=scan_planner::diagnosticName(cell.classification);
      result["diagnostic_counts"][name]=result["diagnostic_counts"][name].get<std::size_t>()+1;
      if(!v.value("export_voxels",true)) continue;
      if(result["voxels"].size()>=4096) continue;
      Json row=voxel(map,cell.index);row["cylinder_membership_mask"]=cell.cylinder_mask;
      const Eigen::Vector3d center=(cell.index.cast<double>().array()+.5)*map.mp_.resolution_;
      row["center_in_body_coordinates"]=xyz(q.conjugate()*(center-body));
      // Exact transformed eight corners, not an upright AABB mislabeled body space.
      row["corners_in_body_coordinates"]=Json::array();
      for(int x:{0,1}) for(int y:{0,1}) for(int z:{0,1})
        row["corners_in_body_coordinates"].push_back(xyz(q.conjugate()*
          ((cell.index.cast<double>()+Eigen::Vector3d(x,y,z))*map.mp_.resolution_-body)));
      row["relative_to_measured_ground_z"]=nullptr;
      if(result["measured_ground_z"].is_number()) {
        const double ground=result["measured_ground_z"].get<double>();
        row["relative_to_measured_ground_z"]={cell.index.z()*map.mp_.resolution_-ground,
            (cell.index.z()+1)*map.mp_.resolution_-ground};
      }
      row["verified_self_intersection"]=nullptr;
      result["voxels"].push_back(std::move(row));
    }
    result["exported_voxels"]=result["voxels"].size();
    result["export_truncated"]=v.value("export_voxels",true) && detail.voxels.size()>4096;
    return result;
  }
};

int main() {
  GridMap map;bool configured=false;std::string line,config_hash;
  while(std::getline(std::cin,line)) {
    try {
      if(line.size()>65536) throw std::runtime_error("oversized request");
      const Json request=Json::parse(line);const auto type=request.at("type").get<std::string>();
      const auto started=std::chrono::steady_clock::now();Json result;
      if(type=="configure" && !configured) {
        GridMapTestAccess::configure(map,request.at("parameters"));
        GridMapTestAccess::freshness(map,request.value("enforce_free_freshness",true));
        configured=map.applyLocalizationContext(request.at("context").dump());
        if(!configured) throw std::runtime_error("context rejected");
        config_hash=request.value("effective_parameters_sha256",std::string{});
        if(request.contains("diagnostics")) {
          const auto &d=request.at("diagnostics");
          const auto c=d.at("center").get<std::vector<double>>(),h=d.at("half_extent").get<std::vector<double>>();
          if(c.size()!=3 || h.size()!=3) throw std::runtime_error("diagnostic ROI requires XYZ");
          map.nearFieldDiagnostics().configure(d.value("enabled",false),{c[0],c[1],c[2]},
              {h[0],h[1],h[2]},request.at("parameters").at("grid_map.resolution").get<double>(),
              d.value("max_cells",16384U),d.value("max_events_per_integration",200000U));
        }
        result={{"configured",true},{"no_ros_initialized",true},
          {"require_observed_free",map.requiresObservedFree()}};
      } else if(!configured) throw std::runtime_error("configure first");
      else if(type=="rays") {
        std::ifstream stream(request.at("file").get<std::string>(),std::ios::binary|std::ios::ate);
        const auto length=stream.tellg();
        if(!stream || length<=0 || length>16*1024*1024) throw std::runtime_error("invalid CDR input length");
        stream.seekg(0);rclcpp::SerializedMessage serialized(static_cast<std::size_t>(length));
        auto &buffer=serialized.get_rcl_serialized_message();
        stream.read(reinterpret_cast<char*>(buffer.buffer),length);buffer.buffer_length=length;
        if(!stream) throw std::runtime_error("incomplete CDR input");
        d1max_planning_interfaces::msg::ProjectedRays message;
        rclcpp::Serialization<d1max_planning_interfaces::msg::ProjectedRays> converter;
        converter.deserialize_message(&serialized,&message);
        result={{"accepted",GridMapTestAccess::accept(map,message,request.at("now_ns").get<std::int64_t>())}};
      } else if(type=="tick" || type=="summary") {
        if(type=="tick") GridMapTestAccess::integrate(map,request.at("now_ns").get<std::int64_t>());
        GridMapTestAccess::queryClock(map,request.value("now_ns",std::int64_t(0)));
        result=GridMapTestAccess::state(map,type=="summary");
        result["queries"]=Json::array();
        for(const auto &query:request.value("queries",Json::array())) result["queries"].push_back(
            GridMapTestAccess::query(map,query,request.value("now_ns",std::int64_t(0)),config_hash));
      } else if(type=="context") {
        result={{"accepted",map.applyLocalizationContext(request.at("context").dump())}};
      } else if(type=="slide") {
        GridMapTestAccess::slide(map,request.at("position"));result=GridMapTestAccess::state(map);
      } else if(type=="reset") {
        map.resetBuffer();result=GridMapTestAccess::state(map);
      } else if(type=="raw_query") {
        GridMapTestAccess::queryClock(map,request.value("now_ns",std::int64_t(0)));
        result["voxels"]=Json::array();
        if(request.at("cells").size()>4096) throw std::runtime_error("bounded raw query exceeded");
        for(const auto &cell:request.at("cells")) result["voxels"].push_back(GridMapTestAccess::voxel(map,
            {cell.at(0).get<int>(),cell.at(1).get<int>(),cell.at(2).get<int>()}));
      } else throw std::runtime_error("unknown request type");
      result["processing_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
      std::cout<<result.dump()<<std::endl;
    } catch(const std::exception &error) {std::cout<<Json{{"error",error.what()}}.dump()<<std::endl;}
  }
}
