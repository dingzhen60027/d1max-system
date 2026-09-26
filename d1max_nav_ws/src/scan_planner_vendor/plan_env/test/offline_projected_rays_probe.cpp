// Read-only real-bag replay of actual GridMap methods. No ROS init/node/graph.
#include <plan_env/grid_map.h>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iterator>

using Json=nlohmann::json;
struct GridMapTestAccess {
  static void configure(GridMap &map,const Json &v) {
    auto &p=map.mp_;auto &d=map.md_;
    auto number=[&](const char *key){return v.at(std::string("grid_map.")+key).get<double>();};
    p.use_projected_rays_=p.require_observed_free_=p.require_localization_context_=true;
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
    p.cloud_pose_max_age_=.5;p.cloud_pose_pair_wait_=.25;p.vis_height_=number("vis_height");
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
  static Json state(GridMap &map,bool summary=false) {
    Json out={{"source_stamp_ns",map.integrated_cloud_stamp_ns_},{"integrated_counts",map.ray_integrations_},
      {"source_stamps_ns",map.ray_integrated_stamps_},{"drops",map.ray_drops_},
      {"unattributed_drops",map.ray_unattributed_drops_},{"revision",map.occupancyRevision()}};
    if(summary) {
      std::array<std::size_t,3> cells{{0,0,0}};
      for(double value:map.md_.occupancy_buffer_) ++cells[scan_planner::strictRawVoxelStatus(
          value,map.mp_.clamp_min_log_,map.mp_.min_occupancy_log_)];
      out["raw_voxel_counts_free_occupied_unknown"]=cells;
    }
    return out;
  }
  static Json query(GridMap &map,const Json &v) {
    const auto p=v.at("position").get<std::vector<double>>();const double yaw=v.at("yaw").get<double>();
    if(p.size()!=3) throw std::runtime_error("query must have XYZ");
    const Eigen::Vector3d body(p[0],p[1],p[2]);const auto detail=map.inspectInflateOccupancy(body,yaw);
    Json first=Json::array();
    for(std::size_t i=0;i<detail.first.size();++i) {
      if(detail.counts[i+1]==0) {first.push_back(nullptr);continue;}
      const auto &index=detail.first[i];
      const auto xyz=(index.cast<double>().array()+.5)*map.mp_.resolution_;
      first.push_back({xyz.x(),xyz.y(),xyz.z()});
    }
    return {{"label",v.value("label","query")},{"position",p},{"yaw",yaw},
      {"collision",map.getInflateOccupancy(body,yaw)},
      {"counts_free_occupied_unknown_outside",detail.counts},{"first_occupied_unknown_outside",first}};
  }
};

int main() {
  GridMap map;bool configured=false;std::string line;
  while(std::getline(std::cin,line)) {
    try {
      if(line.size()>65536) throw std::runtime_error("oversized request");
      const Json request=Json::parse(line);const auto type=request.at("type").get<std::string>();
      const auto started=std::chrono::steady_clock::now();Json result;
      if(type=="configure" && !configured) {
        GridMapTestAccess::configure(map,request.at("parameters"));
        configured=map.applyLocalizationContext(request.at("context").dump());
        if(!configured) throw std::runtime_error("context rejected");
        result={{"configured",true},{"no_ros_initialized",true}};
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
        result=GridMapTestAccess::state(map,type=="summary");
        result["queries"]=Json::array();
        for(const auto &query:request.value("queries",Json::array())) result["queries"].push_back(GridMapTestAccess::query(map,query));
      } else throw std::runtime_error("unknown request type");
      result["processing_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
      std::cout<<result.dump()<<std::endl;
    } catch(const std::exception &error) {std::cout<<Json{{"error",error.what()}}.dump()<<std::endl;}
  }
}
