#include <gtest/gtest.h>
#include <plan_manage/motion_sweep.hpp>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <map>

// The same production GridMap buffers and strictRawVoxelStatus implementation
// are exercised in memory. No rclcpp context, ROS graph, sensor subscription or
// alternate fake occupancy implementation is used.
struct GridMapTestAccess {
  static void configure(GridMap& m,const Eigen::Vector3i &dimensions={80,80,60}) {
    auto& p=m.mp_;auto& d=m.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_=dimensions;
    p.map_origin_idx_={0,0,10};m.updateMapBoundaryFromIndex();
    p.use_projected_rays_=p.require_observed_free_=true;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.1;p.cloud_pose_max_age_=.5;
    p.frame_id_="odom";p.map_sliding_en_=false;
    const auto cells=static_cast<std::size_t>(dimensions.x())*dimensions.y()*dimensions.z();
    d.occupancy_buffer_.assign(cells,-1.);d.occupancy_buffer_inflate_.assign(cells,0);
    d.occupancy_buffer_inflate_cnt_.assign(cells,0);m.rebuildInflationOffsets();
    m.free_observation_stamps_.assign(cells,100000000000LL);
    m.ray_integrated_stamps_={100000000000LL,100000000000LL};m.integrated_cloud_stamp_ns_=100000000000LL;
    m.snapshot_clock_ns_=100100000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
    m.collision_snapshot_=true;m.enforce_free_freshness_=true;m.ray_clock_fault_=false;
  }
  static void voxel(GridMap& m,const Eigen::Vector3d& p,double odds) {
    Eigen::Vector3i id;m.posToIndex(p,id);ASSERT_TRUE(m.isInMap(id));m.md_.occupancy_buffer_[m.toAddress(id)]=odds;
  }
  static void expire(GridMap& m) {
    m.snapshot_clock_ns_=100600000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
  }
  static void deployedEnvelope(GridMap& m) {
    m.mp_.double_cylinder_radius_=.29;m.mp_.double_cylinder_offset_=.20;
    m.mp_.obstacles_inflation_z_up=m.mp_.obstacles_inflation_z_down=.45;
    m.rebuildInflationOffsets();
  }
  static void asymmetricEnvelope(GridMap& m) {
    m.mp_.double_cylinder_radius_=.29;m.mp_.double_cylinder_offset_=.20;
    m.mp_.obstacles_inflation_z_up=.40;m.mp_.obstacles_inflation_z_down=.45;
    m.rebuildInflationOffsets();
  }
  static void officialSpotEnvelope(GridMap &m) {
    m.mp_.double_cylinder_radius_=.608;m.mp_.double_cylinder_offset_=.3125;
    m.mp_.obstacles_inflation_z_up=.10;m.mp_.obstacles_inflation_z_down=.55;
    m.rebuildInflationOffsets();
  }
  static void plane(GridMap& m,double z) {
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y)voxel(m,{x*.05,y*.05,z},2.);
  }
  static void stamp(GridMap& m,const Eigen::Vector3d& p,std::int64_t value) {
    Eigen::Vector3i id;m.posToIndex(p,id);m.free_observation_stamps_[m.toAddress(id)]=value;
  }
  static void floorContact(GridMap& m,std::int64_t reachable=6000000000LL) {
    const auto low=m.mp_.map_bound_min_idx_,dimensions=m.mp_.map_voxel_num_;
    const std::array<int,3> origin{{low.x(),low.y(),low.z()}},shape{{dimensions.x(),dimensions.y(),dimensions.z()}};
    std::vector<std::uint8_t> data(static_cast<std::size_t>(shape[0])*shape[1]*shape[2],0);
    for(int x=0;x<shape[0];++x)for(int y=0;y<shape[1];++y)for(int z=0;z<=-origin[2];++z)
      data[(x*shape[1]+y)*shape[2]+z]=z>=-origin[2]-1?3:2;
    const auto digest=[](const std::string &s){return scan_planner::StaticOccupancyPrior::sha256(s.data(),s.size());};
    nlohmann::json contact{{"schema",1},{"kind","flat_plane_support_contact_v1"},
      {"floor_path","/World/GroundPlane/collisionPlane"},{"floor_z",0.},{"penetration_allowance_m",.02},
      {"semantics","whole_closed_cell_floor_intersection_inside_authorized_xy_and_disjoint_from_nonfloor_static_solids_plus_margin"}};
    nlohmann::json bounds{{"min",{origin[0]*.05-1.,origin[1]*.05-1.,0.}},
      {"max",{(origin[0]+shape[0])*.05+1.,(origin[1]+shape[1])*.05+1.,3.}},
      {"semantics","whole_closed_cell_strictly_inside"}};
    nlohmann::json manifest{{"schema",1},{"kind","certified_static_occupancy_prior"},{"provenance","isaac_closed_collision_geometry_v1"},
      {"frame_id","odom"},{"map_version","flat-test"},{"voxel_resolution",.05},{"origin_index",origin},{"shape",shape},
      {"storage_order","C_xyz_z_fastest"},{"state_codes",{{"free",0},{"occupied",1},{"unknown",2},{"support_contact",3}}},
      {"dtype","uint8"},{"meters_per_unit",1.},{"up_axis","Z"},{"geometry_margin_m",0.},{"closed_world_bounds",bounds},
      {"scene_sha256",digest("scene")},{"spec_sha256",digest("spec")},{"collider_sha256",digest("geometry")},
      {"data_sha256",scan_planner::StaticOccupancyPrior::sha256(data.data(),data.size())},{"data_size_bytes",data.size()},{"data_file","data.u8"},
      {"map_from_odom",{{"transform_contract","fixed_identity_map_from_odom_v1"},{"from_frame","odom"},{"to_frame","odom"},
        {"translation",{0.,0.,0.}},{"rotation_xyzw",{0.,0.,0.,1.}}}},
      {"flat_support_contact",contact},{"dynamic_actor_registry_sha256",digest("actors")},
      {"collision_geometry",{{"flat_support_contact",contact},{"closed_world_bounds",bounds},
        {"dynamic_actor_registry_sha256",digest("actors")},{"dynamic_actor_registry",nlohmann::json::array({{{"id","actor"}}})},
        {"boxes",nlohmann::json::array()},{"floor",{{"path","/World/GroundPlane/collisionPlane"},{"type","Plane"},{"axis","Z"},{"extent","infinite"},{"z",0.}}}}}};
    const auto text=manifest.dump();const auto dir=std::filesystem::temp_directory_path()/std::filesystem::path("d1max-motion-support-"+digest(text).substr(0,16));
    std::filesystem::create_directories(dir);
    {std::ofstream f(dir/"data.u8",std::ios::binary);f.write(reinterpret_cast<const char*>(data.data()),data.size());}
    {std::ofstream f(dir/"prior.json");f<<text;}
    scan_planner::StaticOccupancyPrior::Expected expected;expected.manifest_sha256=digest(text);expected.geometry_sha256=digest("geometry");
    expected.map_version="flat-test";expected.frame_id=expected.odom_frame_id="odom";expected.resolution=.05;
    expected.allow_floor_contact=true;expected.support_floor_z=0.;expected.support_penetration_m=.02;expected.dynamic_registry_sha256=digest("actors");
    m.static_prior_=scan_planner::StaticOccupancyPrior::load((dir/"prior.json").string(),expected);std::filesystem::remove_all(dir);
    m.mp_.validated_static_prior_=m.mp_.simulation_collision_clock_=m.static_prior_context_valid_=true;
    m.mp_.localization_session_id_="session";m.localization_epoch_=m.localization_context_sequence_=1;m.localization_seed_="seed";
    const auto receipt=m.snapshot_captured_;const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(receipt.time_since_epoch()).count();
    m.ray_integrated_receipts_.fill(receipt);m.free_observation_receipts_ns_.assign(data.size(),ns);
    m.dynamic_oracle_.configure(digest("actors"),"odom",{"actor"},.05);
    const auto packet=nlohmann::json{{"schema",1},{"kind","isaac_dynamic_occupancy_oracle_v1"},
      {"session_id","session"},{"seed_id","seed"},{"epoch",1},{"context_sequence",1},{"sequence",1},{"frame_id","odom"},
      {"registry_sha256",digest("actors")},{"source_stamp_ns",100100000000LL},{"valid_until_ns",100400000000LL},{"complete",true},
      {"reachable_horizon_ns",reachable},{"reachable_until_ns",100100000000LL+reachable},
      {"actors",nlohmann::json::array({{{"actor_id","actor"},{"regions",nlohmann::json::array({
        {{"state",2},{"min",{5.,5.,0.}},{"max",{6.,6.,2.}}}})}}})}};
    ASSERT_TRUE(m.applyDynamicOccupancyOracle(packet.dump(),ns));
  }
  static void nonfloorHit(GridMap& m,const Eigen::Vector3d& p) {
    voxel(m,p,2.);Eigen::Vector3i cell;m.posToIndex(p,cell);m.recordStaticPriorHit(cell,100100000001LL,p.z());
  }
  static int referenceColumn(GridMap &m,const Eigen::Vector3i &first,int high,std::size_t &visited) {
    visited=0;if(high<first.z())return 2;
    for(std::int64_t z=first.z();z<=static_cast<std::int64_t>(high);++z) {
      ++visited;const int status=m.observedRawSnapshotStatus({first.x(),first.y(),static_cast<int>(z)});
      if(status!=0)return status;
    }
    return 0;
  }
  static Eigen::Vector3i lowerIndex(const GridMap &m) {return m.mp_.map_bound_min_idx_;}
  static Eigen::Vector3i upperIndex(const GridMap &m) {return m.mp_.map_bound_max_idx_;}
  static int address(GridMap &m,const Eigen::Vector3i &cell) {return m.toAddress(cell);}
  static std::int64_t rayReceipt(const GridMap &m,unsigned sensor) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(m.ray_integrated_receipts_[sensor].time_since_epoch()).count();
  }
  static std::array<std::int64_t,2> proofDeadlines(const GridMap &m) {
    return {m.collision_cache_deadline_ns_,m.collision_cache_receipt_deadline_ns_};
  }
  static void expireReceipt(GridMap &m,unsigned sensor=0) {
    m.ray_integrated_receipts_[sensor]=std::chrono::steady_clock::now()-std::chrono::milliseconds(500);
  }
  static void expireRaySource(GridMap &m,unsigned sensor) {
    m.ray_integrated_stamps_[sensor]=m.snapshot_clock_ns_-500000001LL;
  }
  static void forceExpiredProof(GridMap &m) {
    m.collision_cache_receipt_deadline_ns_=1;
  }
  static void moveWindowWithoutClearingForFixture(GridMap &m,int dx) {
    // Only tests use this to model a committed new immutable window. The new
    // proof performs the same generation reset as a real snapshot replacement.
    m.mp_.map_origin_idx_.x()+=dx;m.updateMapBoundaryFromIndex();m.beginObservedProof();
  }
  static void freshOracle(GridMap &m,const nlohmann::json &regions,std::uint64_t sequence=2,
      std::int64_t source=100100000001LL) {
    const auto receipt=std::chrono::steady_clock::now();
    m.snapshot_captured_=receipt;m.snapshot_clock_ns_=source;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(receipt.time_since_epoch()).count();
    const auto digest=[](const std::string &s){return scan_planner::StaticOccupancyPrior::sha256(s.data(),s.size());};
    const auto packet=nlohmann::json{{"schema",1},{"kind","isaac_dynamic_occupancy_oracle_v1"},
      {"session_id","session"},{"seed_id","seed"},{"epoch",1},{"context_sequence",1},{"sequence",sequence},
      {"frame_id","odom"},{"registry_sha256",digest("actors")},{"source_stamp_ns",source},
      {"valid_until_ns",source+300000000LL},{"complete",true},{"reachable_horizon_ns",6000000000LL},
      {"reachable_until_ns",source+6000000000LL},{"actors",nlohmann::json::array({{{"actor_id","actor"},{"regions",regions}}})}};
    ASSERT_TRUE(m.applyDynamicOccupancyOracle(packet.dump(),ns));
  }
  static void newBenchmarkAcquisition(GridMap &m,unsigned iteration) {
    const std::int64_t source=100100000000LL+static_cast<std::int64_t>(iteration+1)*1000000000LL;
    const auto receipt=std::chrono::steady_clock::now();
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(receipt.time_since_epoch()).count();
    // Each cold benchmark has a separate injected acquisition/oracle identity;
    // refresh/setup is outside timing and never asserted as real sensor data.
    m.ray_integrated_stamps_.fill(source-100000000LL);m.integrated_cloud_stamp_ns_=source-100000000LL;
    m.ray_integrated_receipts_.fill(receipt);m.free_observation_stamps_.assign(m.md_.occupancy_buffer_.size(),source-100000000LL);
    m.free_observation_receipts_ns_.assign(m.md_.occupancy_buffer_.size(),ns);
    freshOracle(m,nlohmann::json::array({{{"state",2},{"min",{5.,5.,0.}},{"max",{6.,6.,2.}}}}),iteration+2,source);
  }
};
namespace {
using namespace scan_planner;
BrakingModel model() {return {.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};}
BrakingModel physicalSpotModel() {
  auto out=model();out.max_speed=.6;out.max_yaw=.8;out.reaction=.67;out.distance=.5;
  out.yaw=.4;out.latency=3.;out.tracking_error=.1;out.heading_error=.1;
  out.isolated_spot_model=true;out.command_max_speed=.15;out.command_max_yaw=.3;return out;
}
d1max_planning_interfaces::msg::SupportReference ground() {
  d1max_planning_interfaces::msg::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";
  s.required_mode="general";s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.04;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int x=-40;x<=40;++x)for(int y=-40;y<=40;++y){geometry_msgs::msg::Point p;p.x=x*.025;p.y=y*.025;s.support_ground_xyz.push_back(p);}return s;
}
MotionSupport physicalSpotGround() {
  auto data=ground();data.support_ground_xyz.clear();data.support_xy_radius_m=.2;data.body_reference_height_m=.48;
  for(int x=-60;x<=60;++x)for(int y=-60;y<=60;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;data.support_ground_xyz.push_back(p);
  }
  return MotionSupport(data);
}
struct FrozenPhysicalSpotCellVolume {
  std::size_t aabb_voxels{0},inside_voxels{0},inside_columns{0};
  Eigen::Vector3i low,high;
  double min_z{0.},max_z{0.};
};
// Independent, frozen pre-v29 geometric calculation for this fixture. It uses
// the physical record/envelope constants directly, enumerates every Z cell,
// and never calls a production motion-footprint or resource-counting helper.
FrozenPhysicalSpotCellVolume frozenPhysicalSpotCellVolume(GridMap &map,
    const Eigen::Vector3d &body,double yaw,double measured_vertical_speed=0.) {
  FrozenPhysicalSpotCellVolume out;
  const double resolution=.05,offset=.3125,length=.6*.67+.5;
  const double turn=.8*.67+.4+.1,xmin=-length-.1,xmax=length+.1,lateral=length+.1;
  const double radius=.608+2.*offset*std::sin(std::min(turn,3.141592653589793)*.5);
  const double padded_radius=radius+resolution*std::sqrt(.5);
  const double c=std::cos(yaw),s=std::sin(yaw);
  Eigen::Vector3d lower=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),upper=-lower;
  for(double x:{xmin-offset-radius,xmax+offset+radius})for(double y:{-lateral-radius,lateral+radius}) {
    const Eigen::Vector3d point(body.x()+c*x-s*y,body.y()+s*x+c*y,0.);
    lower=lower.cwiseMin(point);upper=upper.cwiseMax(point);
  }
  out.min_z=-.02; // The fixture's fixed, certified complete floor-contact slab.
  out.max_z=body.z()+std::tan(.15)*.025+.1+std::abs(measured_vertical_speed)*.67+.55;
  lower.z()=out.min_z;upper.z()=out.max_z;
  map.posToIndex(lower,out.low);map.posToIndex(upper,out.high);
  for(int x=out.low.x();x<=out.high.x();++x)for(int y=out.low.y();y<=out.high.y();++y) {
    Eigen::Vector3d center;map.indexToPos({x,y,out.low.z()},center);
    const double dx=center.x()-body.x(),dy=center.y()-body.y(),u=c*dx+s*dy,v=-s*dx+c*dy;
    bool original_inside=false;
    for(double cylinder_offset:{-offset,offset}) {
      const double qx=std::max({xmin+cylinder_offset-u,0.,u-(xmax+cylinder_offset)});
      const double qy=std::max(0.,std::abs(v)-lateral);
      original_inside=original_inside||(qx*qx+qy*qy<=padded_radius*padded_radius);
    }
    if(original_inside)++out.inside_columns;
    for(int z=out.low.z();z<=out.high.z();++z) {
      ++out.aabb_voxels;if(original_inside)++out.inside_voxels;
    }
  }
  return out;
}
// Frozen pre-pruning algorithm: same .1 m bin keys, a/b order, insertion order
// inside each bin and <= last-wins tie. This supplies an independent oracle
// for pruning/seed changes, including equal XY distances with different Z.
class LegacyOrderedSupportNearest {
  using Point=geometry_msgs::msg::Point;
  std::map<std::pair<int,int>,std::vector<Point>> bins_;
  double radius_;
public:
  explicit LegacyOrderedSupportNearest(const d1max_planning_interfaces::msg::SupportReference &source):
      radius_(source.support_xy_radius_m) {
    for(const auto &p:source.support_ground_xyz)
      bins_[{int(std::floor(p.x/.1)),int(std::floor(p.y/.1))}].push_back(p);
  }
  std::optional<Point> nearest(double x,double y) const {
    if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>1e5||std::abs(y)>1e5)return {};
    const int kx=int(std::floor(x/.1)),ky=int(std::floor(y/.1)),n=int(std::ceil(radius_/.1))+1;
    double best=radius_*radius_;std::optional<Point> out;
    for(int a=-n;a<=n;++a)for(int b=-n;b<=n;++b) {
      const auto it=bins_.find({kx+a,ky+b});if(it==bins_.end())continue;
      for(const auto &p:it->second) {
        const double d=(p.x-x)*(p.x-x)+(p.y-y)*(p.y-y);if(d<=best){best=d;out=p;}
      }
    }
    return out;
  }
};
void expectColumnMatchesCells(GridMap &map,const Eigen::Vector3i &first,int high,int expected) {
  SCOPED_TRACE("column "+std::to_string(first.x())+","+std::to_string(first.y())+","+
      std::to_string(first.z())+".."+std::to_string(high));
  map.beginObservedProof();std::size_t old_visits=0;
  const auto old_status=GridMapTestAccess::referenceColumn(map,first,high,old_visits);
  const auto original_deadlines=GridMapTestAccess::proofDeadlines(map);
  map.beginObservedProof();std::size_t visits=0;
  EXPECT_EQ(map.observedRawSnapshotColumnStatus(first,high,&visits),old_status);
  EXPECT_EQ(old_status,expected);EXPECT_EQ(visits,old_visits);
  if(old_status==0)EXPECT_EQ(GridMapTestAccess::proofDeadlines(map),original_deadlines);
}
MotionSweepResult run(GridMap& m,const geometry_msgs::msg::Twist& command,
    const geometry_msgs::msg::Twist& measured={},const BrakingModel& braking=model(),double budget=.025) {
  MotionSupport s(ground());return validateMotionSweep(m,s,braking,{0.,0.,.55},0.,measured,command,"odom",budget);
}
std::string digest(const std::string& bytes) {
  unsigned char out[EVP_MAX_MD_SIZE];unsigned n=0;EVP_Digest(bytes.data(),bytes.size(),out,&n,EVP_sha256(),nullptr);
  std::ostringstream s;for(unsigned i=0;i<n;++i)s<<std::hex<<std::setfill('0')<<std::setw(2)<<int(out[i]);return s.str();
}
}
TEST(MotionSweep,ActualNativeObservedMapProvesMovingAndTurningReachableEnvelope) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;v.angular.z=.5;
  const auto out=run(m,v);EXPECT_TRUE(out.valid)<<out.reason;EXPECT_GT(out.unique_voxels,100U);
  EXPECT_NEAR(out.length,.2,1e-9);EXPECT_NEAR(out.heading_bound,.43,1e-9);
  EXPECT_GT(m.observedProofDeadlineNs(),100100000000LL);
}
TEST(MotionSweep,CertifiedFlatContactChecksFullFootSlabAtFixedHeightInCurveAndRawMotion) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map);
  geometry_msgs::msg::Twist command;command.linear.x=.3;
  const auto good=run(map,command,{},model(),.1);ASSERT_TRUE(good.valid)<<good.reason;
  EXPECT_DOUBLE_EQ(good.min_z,-.02);
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy({0.,0.,.55},0.),0);
  GridMapTestAccess::nonfloorHit(map,{.4,.025,.02});
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy({0.,0.,.55},0.),0);
  const auto blocked=run(map,command,{},model(),.1);EXPECT_FALSE(blocked.valid);
  EXPECT_EQ(blocked.reason,"motion_sweep_unknown_or_expired"); // Real unseen-in-scan foot obstacle veto.
  GridMapTestAccess::nonfloorHit(map,{.2,.025,.02});
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy({0.,0.,.55},0.),2); // Full slab even with legacy .1m below kernel.
}
TEST(MotionSweep,FlatContactCertificateRejectsDifferentSupportHeightInsteadOfFollowingGaitZ) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map);
  auto data=ground();for(auto &p:data.support_ground_xyz)p.z=.001;
  MotionSupport support(data);geometry_msgs::msg::Twist command;command.linear.x=.1;
  const auto out=validateMotionSweep(map,support,model(),{0.,0.,.55},0.,{},command,"odom",.1);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_support_differs_from_certified_flat_plane");
}
TEST(MotionSweep,FreshOracleLeaseCannotAuthorizeBrakingBeyondItsActorPredictionDuration) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map,300000000LL);
  geometry_msgs::msg::Twist command;command.linear.x=.1;
  const auto out=run(map,command,{},model(),.1);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_dynamic_reachable_horizon_insufficient_or_expired");
}
TEST(MotionSweep,AsymmetricReflectedEnvelopeDoesNotInventGroundAndCannotMissHeadObstacle) {
  auto g=ground();g.body_reference_height_m=.50;const MotionSupport support(g);
  const Eigen::Vector3d body(0.,0.,.50);
  geometry_msgs::msg::Twist command;command.linear.x=.02;command.angular.z=-.04;
  for(double plane:{.0,.10,.960})for(double yaw:{0.,-1.570796326794897}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::asymmetricEnvelope(map);
    const auto kernel=map.obstacleDilation();const auto envelope=map.bodyEnvelope();
    ASSERT_DOUBLE_EQ(kernel.up,.40);ASSERT_DOUBLE_EQ(kernel.down,.45);
    ASSERT_DOUBLE_EQ(envelope.below,.40);ASSERT_DOUBLE_EQ(envelope.above,.45);
    GridMapTestAccess::plane(map,plane);map.beginObservedProof();
    const int pose=map.getInflateOccupancy(body,yaw);
    const auto motion=validateMotionSweep(map,support,model(),body,yaw,{},command,"odom",.1);
    if(plane==0.) {EXPECT_EQ(pose,0);EXPECT_TRUE(motion.valid)<<motion.reason;}
    else {EXPECT_EQ(pose,1);EXPECT_FALSE(motion.valid);EXPECT_EQ(motion.reason,"motion_sweep_occupied");}
    const double pad=std::tan(.15)*.025+.01;
    EXPECT_NEAR(motion.min_z,.50-.40-pad,1e-12);
    EXPECT_NEAR(motion.max_z,.50+.45+pad,1e-12);
  }
}
TEST(MotionSweep,FailedEnvelopeDiagnosticsSeparateClassesAndUniqueFromRawOverlap) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::asymmetricEnvelope(map);
  GridMapTestAccess::voxel(map,{.025,.025,.50},-1.01);
  GridMapTestAccess::voxel(map,{.025,.025,.60},0.);
  GridMapTestAccess::voxel(map,{.025,.025,.70},2.);
  GridMapTestAccess::stamp(map,{.025,.025,.80},99000000000LL);
  const auto evidence=map.inspectInflateOccupancy({0.,0.,.5},0.,true);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::NeverObserved)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::Insufficient)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::Occupied)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::StaleFree)],1U);
  EXPECT_EQ(evidence.unique_counts[1],1U);EXPECT_EQ(evidence.unique_counts[2],3U);
  EXPECT_GT(evidence.counts[2],evidence.unique_counts[2]);
  const auto detail=map.describeInflateOccupancy({0.,0.,.5},0.);
  EXPECT_NE(detail.find("never_observed=1"),std::string::npos);
  EXPECT_NE(detail.find("observed_insufficient=1"),std::string::npos);
  EXPECT_NE(detail.find("stale_free=1"),std::string::npos);
  EXPECT_NE(detail.find("front_source_ns=100000000000"),std::string::npos);
  EXPECT_NE(detail.find("ray_witness=disabled"),std::string::npos);
  map.nearFieldDiagnostics().configure(true,{0.,0.,.5},{1.,1.,1.},.05);
  Eigen::Vector3i occupied;map.posToIndex({.025,.025,.70},occupied);
  RayWitness witness;witness.hit=witness.contributed_vote=true;witness.metadata.sensor_id=1;
  witness.metadata.scan_stamp_ns=100000000000LL;witness.metadata.acquisition_end_ns=100050000000LL;
  witness.origin={-.4,0.,.5};witness.endpoint={.025,.025,.70};
  map.nearFieldDiagnostics().record(occupied,witness);
  const auto provenance=map.describeInflateOccupancy({0.,0.,.5},0.);
  EXPECT_NE(provenance.find("ray_witness={sensor:1,hit:1,vote:1,scan_ns:100000000000"),std::string::npos);
  EXPECT_NE(provenance.find("origin:[-0.4,0,0.5]"),std::string::npos);
  map.beginObservedProof();EXPECT_NE(map.getInflateOccupancy({0.,0.,.5},0.),0);
  // Diagnostic occupied has decision precedence for reporting only; native
  // first-hit policy and all unknown-space blocking remain unchanged.
}
TEST(MotionSweep,NewBodyOrNewProofDoesNotRenewOriginalDemandBodyOrExpiredRaySource) {
  constexpr std::int64_t t=100000000000LL;
  EXPECT_TRUE(motionSourceFresh(t,t+99999999LL));EXPECT_FALSE(motionSourceFresh(t,t+100000001LL));
  EXPECT_FALSE(motionSourceFresh(t+1,t));EXPECT_FALSE(motionSourceFresh(0,t));
  auto deadline=motionEvidenceDeadline(t,t-50000000LL,t+20000000LL,t+100000000LL,
    t+250000000LL,t+250000000LL,t+500000000LL,t,t);
  EXPECT_EQ(deadline,t+50000000LL); // Old demand-body evidence, not newest body.
  deadline=motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+10000000LL,t-490000000LL,t);
  EXPECT_EQ(deadline,t+10000000LL);
  EXPECT_FALSE(deadline>t+10000000LL); // Crossing TTL during computation cannot commit.
}
TEST(MotionSweep,FreeCurrentBodyAndShortNominalCurveDoNotProveBrakingDistance) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.5,.025,.55},2.);
  m.beginObservedProof();EXPECT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  EXPECT_EQ(m.getInflateOccupancy({.02,0,.55},0.),0);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.05;actual.linear.x=.3;
  auto out=run(m,command,actual);EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_occupied");
}
TEST(MotionSweep,PlanningCorridorIncludesFutureBrakingAndDoesNotReplaceCommandProof) {
  GridMap m;GridMapTestAccess::configure(m);MotionSupport support(ground());
  const Eigen::Vector3d begin(0.,0.,.55),target(.25,0.,.55);
  ASSERT_TRUE(validateStraightBrakingCorridor(m,support,model(),begin,target,"odom").valid);
  // Past the target's pose footprint, still inside its future stop volume.
  GridMapTestAccess::voxel(m,{.70,.025,.55},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy(target,0.),0);
  auto future=validateStraightBrakingCorridor(m,support,model(),begin,target,"odom");
  EXPECT_FALSE(future.valid);EXPECT_EQ(future.reason,"motion_sweep_occupied");
  geometry_msgs::msg::Twist command;command.linear.x=.3;
  EXPECT_TRUE(run(m,command).valid); // Current body has not reached that obstacle.
}
TEST(MotionSweep,TurningSweepCoversCylinderRotationAwayFromFreeCurrentPose) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.275,.20,.55},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  geometry_msgs::msg::Twist v;v.angular.z=.5;const auto out=run(m,v);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_occupied");
}
TEST(MotionSweep,NeverObservedAndInsufficientEvidenceAreNotAssumedFree) {
  for(double evidence:{-1.01,0.}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.5,.025,.55},evidence);
    geometry_msgs::msg::Twist v;v.linear.x=.3;auto out=run(m,v);
    EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_unknown_or_expired");
  }
}
TEST(MotionSweep,TTLExpiryDoesNotClearOccupancyOrRenewFreeEvidence) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::expire(m);
  geometry_msgs::msg::Twist v;v.linear.x=.2;auto out=run(m,v);EXPECT_FALSE(out.valid);
  EXPECT_EQ(out.reason,"motion_sweep_unknown_or_expired");
}
TEST(MotionSweep,SourceGroundGapWrongFloorOrMissingBodySupportRejects) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(int which=0;which<3;++which) {
    auto g=ground();if(which==0)g.floor_id="floor2";
    if(which==1)for(auto& p:g.support_ground_xyz)p.z+=3.;
    if(which==2)g.support_ground_xyz.erase(std::remove_if(g.support_ground_xyz.begin(),g.support_ground_xyz.end(),
      [](const auto& p){return p.x>.06;}),g.support_ground_xyz.end());
    MotionSupport s(g);auto out=validateMotionSweep(m,s,model(),{0,0,.55},0.,{},v,"odom");EXPECT_FALSE(out.valid);
  }
}
// These use the deployed geometric envelope and the actual native swept-volume
// query. Only initial raw odds/free timestamps are injected; raw projection is
// covered separately by plan_env/test_projected_rays, not claimed by this test.
TEST(MotionSweep,DeployedEnvelopeRejectsLowObstacleInsideBrakingSweep) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
  const auto shape=m.collisionShape();ASSERT_DOUBLE_EQ(shape.radius,.29);
  ASSERT_DOUBLE_EQ(shape.offset,.20);ASSERT_DOUBLE_EQ(shape.up,.45);ASSERT_DOUBLE_EQ(shape.down,.45);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.05;actual.linear.x=.3;
  const auto clear=run(m,command,actual);ASSERT_TRUE(clear.valid)<<clear.reason;
  // The obstacle is outside the current body, but inside reaction + stopping
  // reach. Its 10--15 cm voxel intersects the protected lower body envelope.
  GridMapTestAccess::voxel(m,{.65,.025,.12},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  const auto blocked=run(m,command,actual);
  EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,"motion_sweep_occupied");
  EXPECT_LT(blocked.min_z,.12);EXPECT_GT(blocked.max_z,.12);
}
TEST(MotionSweep,DeployedEnvelopeDoesNotClaimContactBelowOrAboveProtectedHeight) {
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(double height:{.025,1.225}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
    GridMapTestAccess::voxel(m,{.65,.025,height},2.);
    const auto result=run(m,v);EXPECT_TRUE(result.valid)<<result.reason;
    // The occupied voxel is wholly outside this geometric envelope. This is
    // NOT proof that a physical wheel/foot can safely traverse a 2.5 cm object.
    if(height<.55) EXPECT_LT(.05,result.min_z);
    else EXPECT_GT(1.20,result.max_z);
  }
}
TEST(MotionSweep,DeployedEnvelopeRejectsLowUnknownAndInsufficientEvidence) {
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(double odds:{-1.01,0.}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
    GridMapTestAccess::voxel(m,{.65,.025,.12},odds);
    const auto result=run(m,v);EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason,"motion_sweep_unknown_or_expired");
  }
}
TEST(MotionSweep,DeployedEnvelopeRejectsMissingSupportEvenWhenAllRawVoxelsAreFree) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  ASSERT_TRUE(run(m,v).valid);
  auto g=ground();g.support_ground_xyz.erase(std::remove_if(g.support_ground_xyz.begin(),g.support_ground_xyz.end(),
    [](const auto& p){return p.x>.06;}),g.support_ground_xyz.end());
  MotionSupport support(g);const auto result=validateMotionSweep(m,support,model(),{0,0,.55},0.,{},v,"odom");
  EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"braking_sweep_support_unverified");
}
TEST(MotionSweep,EitherRawRaySourceIndependentlyCapsCommandEvidenceDeadline) {
  constexpr std::int64_t t=100000000000LL;
  const auto deadline=[&](std::int64_t front,std::int64_t rear) {
    return motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
      t+500000000LL,front,rear);
  };
  EXPECT_EQ(deadline(t,t),t+100000000LL);
  EXPECT_EQ(deadline(t,t-490000000LL),t+10000000LL);
  EXPECT_EQ(deadline(t-490000000LL,t),t+10000000LL);
  EXPECT_LE(deadline(t,t-500000001LL),t); // Fresh front cannot renew stale rear.
  EXPECT_LE(deadline(t-500000001LL,t),t); // Nor can a fresh rear renew stale front.
  EXPECT_LE(deadline(t,0),t);EXPECT_LE(deadline(0,t),t);
}
TEST(MotionSweep,BoundsBudgetAndNonForwardCommandFailClosed) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;
  auto bad=model();bad.heading_error=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(run(m,v,{},bad).valid);
  EXPECT_FALSE(run(m,v,{},model(),1e-12).valid);
  v.linear.x=-.01;EXPECT_FALSE(run(m,v).valid);v.linear.x=.31;EXPECT_FALSE(run(m,v).valid);
  v.linear.x=.1;v.linear.y=.01;EXPECT_FALSE(run(m,v).valid);
}
TEST(MotionSweep,ExactFileBytesAndTransportAreBoundToMeasuredModel) {
  nlohmann::json j={{"schema_version",3},{"transport_mode","isolated_mock"},{"fixture_only",true},
    {"model","reaction_braking_reachable_v1"},
    {"execution_timing",{{"sensor_source_age_bound_s",.2},{"command_pipeline_bound_s",.1},
      {"writer_period_s",.05},{"source_time_uncertainty_s",.02}}},
    {"measurements",{{"max_speed_mps",.3},{"max_yaw_radps",.5},
    {"reaction_bound_s",.4},{"stopping_distance_m",.08},{"stopping_yaw_rad",.2},{"stop_latency_bound_s",.5},
    {"tracking_error_bound_m",.01},{"heading_error_bound_rad",.03}}}};
  const auto path=std::filesystem::temp_directory_path()/
    ("d1max_braking_model_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  auto bytes=j.dump();{std::ofstream f(path);f<<bytes;}
  const auto loaded=BrakingModel::load(path.string(),digest(bytes),"isolated_mock");
  EXPECT_TRUE(loaded.valid());EXPECT_EQ(loaded.raySourceAgeNs(),200000000LL);
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"live"),std::invalid_argument);
  {std::ofstream f(path);f<<bytes<<'\n';}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  auto incomplete=j;incomplete.erase("execution_timing");bytes=incomplete.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  auto underbounded=j;underbounded["execution_timing"]["sensor_source_age_bound_s"]=.5;
  bytes=underbounded.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  j["measurements"].erase("heading_error_bound_rad");bytes=j.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  std::filesystem::remove(path);
}
TEST(MotionSweep,RecordSourceAgeOverridesLegacyDefaultAndCannotRenewEitherSensor) {
  const auto t=100000000000LL;
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t-150000000LL,t,200000000LL),t+50000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t,t-150000000LL,200000000LL),t+50000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t,t,0LL),0LL);
}
TEST(MotionSweep,ExplicitSpotRecordBindsActualReachabilityWithoutRaisingCommandAuthority) {
  nlohmann::json j={{"schema_version",3},{"transport_mode","isolated_mock"},{"fixture_only",true},
    {"model","reaction_braking_reachable_v1"},
    {"execution_timing",{{"sensor_source_age_bound_s",.2},{"command_pipeline_bound_s",.1},
      {"writer_period_s",.05},{"source_time_uncertainty_s",.02}}},
    {"measurements",{{"max_speed_mps",.6},{"max_yaw_radps",.8},{"reaction_bound_s",.4},
      {"stopping_distance_m",.08},{"stopping_yaw_rad",.2},{"stop_latency_bound_s",.5},
      {"tracking_error_bound_m",.01},{"heading_error_bound_rad",.03}}},
    {"isolated_platform_model",{{"schema",1},{"kind","official_spot_physx"},
      {"command_max_speed_mps",.15},{"command_max_yaw_radps",.3},
      {"reachable_max_speed_mps",.6},{"reachable_max_yaw_radps",.8},
      {"source_scope","isolated_simulation_physx_measured_model"}}}};
  const auto path=std::filesystem::temp_directory_path()/
    ("d1max_spot_braking_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  const auto load=[&](const nlohmann::json& value,const std::string& mode="isolated_mock") {
    const auto bytes=value.dump();{std::ofstream f(path);f<<bytes;}
    return BrakingModel::load(path.string(),digest(bytes),mode);
  };
  const auto spot=load(j);ASSERT_TRUE(spot.valid());EXPECT_TRUE(spot.isolated_spot_model);
  EXPECT_EQ(spot.commandMaxSpeed(),.15);EXPECT_EQ(spot.commandMaxYaw(),.3);
  EXPECT_EQ(spot.raySourceAgeNs(),200000000LL);
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist command,actual;
  command.linear.x=.15;actual.linear.x=.35;
  EXPECT_TRUE(run(m,command,actual,spot,.1).valid);
  EXPECT_FALSE(run(m,command,actual,model(),.1).valid);
  command.linear.x=.1501;EXPECT_FALSE(run(m,command,actual,spot,.1).valid);
  command.linear.x=.15;command.angular.z=.3001;EXPECT_FALSE(run(m,command,actual,spot,.1).valid);
  command.angular.z=0.;actual.linear.x=.6001;EXPECT_FALSE(run(m,command,actual,spot,.1).valid);
  actual.linear.x=.35;actual.angular.z=.8001;EXPECT_FALSE(run(m,command,actual,spot,.1).valid);
  auto bad=j;bad.erase("isolated_platform_model");EXPECT_ANY_THROW(load(bad));
  EXPECT_ANY_THROW(load(j,"live"));
  bad=j;bad["fixture_only"]=false;bad["transport_mode"]="live";EXPECT_ANY_THROW(load(bad,"live"));
  for(const auto& pair:std::vector<std::pair<std::string,nlohmann::json>>{
      {"schema",true},{"schema",1.},{"kind","official_go2_physx"},{"source_scope","live"},
      {"command_max_speed_mps",.301},{"command_max_yaw_radps",.501},
      {"reachable_max_speed_mps",.601},{"reachable_max_yaw_radps",.801},
      {"reachable_max_speed_mps",.59},{"reachable_max_yaw_radps",true}}) {
    bad=j;bad["isolated_platform_model"][pair.first]=pair.second;EXPECT_ANY_THROW(load(bad));
  }
  for(const auto* field:{"schema","kind","command_max_speed_mps","command_max_yaw_radps",
      "reachable_max_speed_mps","reachable_max_yaw_radps","source_scope"}) {
    bad=j;bad["isolated_platform_model"].erase(field);EXPECT_ANY_THROW(load(bad));
  }
  bad=j;bad["isolated_platform_model"]["extra_permission"]=true;EXPECT_ANY_THROW(load(bad));
  for(const auto& pair:std::vector<std::pair<std::string,double>>{
      {"stop_latency_bound_s",3.01},{"stopping_distance_m",1.01},{"reaction_bound_s",1.01}}) {
    bad=j;bad["measurements"][pair.first]=pair.second;EXPECT_ANY_THROW(load(bad));
  }
  auto default_model=model();default_model.max_speed=.6;EXPECT_FALSE(default_model.valid());
  std::filesystem::remove(path);
}
TEST(MotionSweep,SpotFutureGaitFromNearZeroCannotEscapeReactionOrYawEnvelope) {
  auto spot=model();spot.isolated_spot_model=true;spot.max_speed=.6;spot.max_yaw=.8;
  spot.command_max_speed=.15;spot.command_max_yaw=.3;
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  for(const Eigen::Vector3d& possible_obstacle:std::vector<Eigen::Vector3d>{
      {.65,0.,.55},{-.65,0.,.55},{0.,.5,.55},{0.,-.5,.55}}) {
    GridMap map;GridMapTestAccess::configure(map);
    const auto clear=run(map,command,actual,spot,.1);ASSERT_TRUE(clear.valid)<<clear.reason;
    EXPECT_NEAR(clear.length,.6*.4+.08,1e-12);
    EXPECT_NEAR(clear.heading_bound,.8*.4+.2+.03,1e-12);
    const auto legacy=run(map,command,actual,model(),.1);ASSERT_TRUE(legacy.valid)<<legacy.reason;
    EXPECT_NEAR(legacy.length,.01*.4+.08,1e-12);
    EXPECT_NEAR(legacy.heading_bound,.2+.03,1e-12);
    GridMapTestAccess::voxel(map,possible_obstacle,2.);
    // Forward, reverse and lateral future gait must all query their unchanged
    // original occupancy even when current measured velocity is near zero.
    EXPECT_TRUE(run(map,command,actual,model(),.1).valid);
    const auto blocked=run(map,command,actual,spot,.1);
    EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,"motion_sweep_occupied");
  }
}

TEST(RawSnapshotColumn, EveryCellAndFirstNonfreeMatchRawQueriesIncludingFootAndTop) {
  for(int kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map);
    int expected=0;
    if(kind==1){GridMapTestAccess::nonfloorHit(map,{.025,.025,-.025});expected=2;}
    if(kind==2){GridMapTestAccess::voxel(map,{.025,.025,1.025},2.);expected=1;}
    if(kind==3){GridMapTestAccess::voxel(map,{.025,.025,.075},0.);expected=2;}
    if(kind==4) {
      // Lower weak evidence must win over a higher occupied cell, exactly as
      // in the original bottom-to-top traversal, including cached repeats.
      GridMapTestAccess::voxel(map,{.025,.025,.075},0.);
      GridMapTestAccess::voxel(map,{.025,.025,1.025},2.);expected=2;
    }
    expectColumnMatchesCells(map,{0,0,-1},20,expected);
    expectColumnMatchesCells(map,{0,0,-1},20,expected);
  }
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::voxel(map,{.025,.025,.025},-1.01);
  expectColumnMatchesCells(map,{0,0,0},20,2); // No static certificate: never seen still blocks.
}

TEST(RawSnapshotColumn, PublicBatchRequiresPrivateObservedSnapshot) {
  GridMap writer;
  EXPECT_THROW(writer.observedRawSnapshotColumnStatus({0,0,0},5),std::logic_error);
}

TEST(MotionSupportNearest, ExactLegacyBinOrderRandomQueriesTiesAndWorldBounds) {
  const auto compare=[](const MotionSupport &support,const LegacyOrderedSupportNearest &legacy,double x,double y) {
    SCOPED_TRACE("support query "+std::to_string(x)+","+std::to_string(y));
    const auto old_point=legacy.nearest(x,y),point=support.nearest(x,y);
    ASSERT_EQ(bool(point),bool(old_point));if(!point)return;
    EXPECT_DOUBLE_EQ(point->x,old_point->x);EXPECT_DOUBLE_EQ(point->y,old_point->y);EXPECT_DOUBLE_EQ(point->z,old_point->z);
  };
  const auto support=physicalSpotGround();ASSERT_TRUE(support.valid);
  const LegacyOrderedSupportNearest legacy(support.source);
  std::uint64_t random=0x1827304aULL;
  const auto sample=[&]() {random=random*6364136223846793005ULL+1442695040888963407ULL;
    return (double(random>>32)/4294967295.-.5)*6.8;};
  for(unsigned i=0;i<300;++i){const double x=sample(),y=sample();compare(support,legacy,x,y);}
  for(double x:{-.2,-.1,-.025,0.,.025,.1,.2})for(double y:{-.2,-.025,0.,.025,.2})compare(support,legacy,x,y);
  auto data=ground();data.support_ground_xyz.clear();data.support_xy_radius_m=.2;
  for(const Eigen::Vector3d &xyz:std::vector<Eigen::Vector3d>{
      {-.05,0.,.01},{.05,0.,.02},{.05,0.,.03},{.05,0.,.04}}) {
    geometry_msgs::msg::Point p;p.x=xyz.x();p.y=xyz.y();p.z=xyz.z();data.support_ground_xyz.push_back(p);
  }
  const MotionSupport ties(data);ASSERT_TRUE(ties.valid);const LegacyOrderedSupportNearest tie_reference(data);
  compare(ties,tie_reference,0.,0.);compare(ties,tie_reference,.05,0.);
  ASSERT_TRUE(ties.nearest(0.,0.));EXPECT_DOUBLE_EQ(ties.nearest(0.,0.)->z,.04);
  data.support_ground_xyz.clear();
  for(double sign:{-1.,1.})for(double offset:{0.,.05,.1}) {
    geometry_msgs::msg::Point p;p.x=sign*(100000.-offset);p.y=sign*(100000.-offset);p.z=offset;
    data.support_ground_xyz.push_back(p);
  }
  const MotionSupport edge(data);ASSERT_TRUE(edge.valid);const LegacyOrderedSupportNearest edge_reference(data);
  for(double sign:{-1.,1.})for(double offset:{-.000001,0.,.025,.15,.25})
    compare(edge,edge_reference,sign*(100000.-offset),sign*(100000.-offset));
  compare(edge,edge_reference,std::numeric_limits<double>::quiet_NaN(),0.);
  compare(edge,edge_reference,0.,std::numeric_limits<double>::infinity());
}

TEST(RawSnapshotColumn, DynamicClosedXYAndAllZVetoMatchFirstNonfreeOrdering) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map);
  GridMapTestAccess::freshOracle(map,nlohmann::json::array({
    {{"state",2},{"min",{.05,.05,.20}},{"max",{.10,.10,.25}}},
    {{"state",1},{"min",{.05,.05,.80}},{"max",{.10,.10,.85}}}}));
  // Closed cell AABBs touching x/y=.05 intersect the exact original oracle.
  expectColumnMatchesCells(map,{0,0,-1},20,2);
  expectColumnMatchesCells(map,{0,0,6},20,1);
  expectColumnMatchesCells(map,{-1,0,-1},20,0);
  expectColumnMatchesCells(map,{0,-1,-1},20,0);
  expectColumnMatchesCells(map,{3,3,-1},20,0);
}

TEST(RawSnapshotColumn, OutsideInvalidRangeAndRingAddressReuseNeverBorrowAnotherColumn) {
  GridMap map;GridMapTestAccess::configure(map);
  const auto low=GridMapTestAccess::lowerIndex(map),high=GridMapTestAccess::upperIndex(map);
  expectColumnMatchesCells(map,{low.x()-1,0,0},5,-1);
  expectColumnMatchesCells(map,{0,0,high.z()-1},high.z()+1,-1);
  expectColumnMatchesCells(map,{0,0,1},0,2);
  const Eigen::Vector3i old_cell{-30,0,0};
  expectColumnMatchesCells(map,old_cell,10,0);
  const auto old_address=GridMapTestAccess::address(map,old_cell);
  GridMapTestAccess::moveWindowWithoutClearingForFixture(map,80);
  const Eigen::Vector3i new_cell=old_cell+Eigen::Vector3i(80,0,0);
  ASSERT_EQ(GridMapTestAccess::address(map,new_cell),old_address);
  GridMapTestAccess::voxel(map,{new_cell.x()*.05+.025,.025,.025},2.);
  expectColumnMatchesCells(map,new_cell,10,1);
}

TEST(RawSnapshotColumn, BatchEndChecksOriginalSourceAndSteadyDeadlinesWithoutRenewal) {
  for(bool receipt_expiry:{false,true}) {
    SCOPED_TRACE(receipt_expiry);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::floorContact(map);
    map.beginObservedProof();ASSERT_EQ(map.observedRawSnapshotColumnStatus({0,0,-1},20),0);
    ASSERT_TRUE(map.observedProofFresh());
    const auto front_source=map.integratedRaySourceStamp(0),rear_source=map.integratedRaySourceStamp(1);
    const auto front_receipt=GridMapTestAccess::rayReceipt(map,0),rear_receipt=GridMapTestAccess::rayReceipt(map,1);
    // End-of-batch validity is mandatory even if a previously cached complete
    // column remains FREE. Mark only the proof deadline, leaving evidence bytes.
    GridMapTestAccess::forceExpiredProof(map);EXPECT_FALSE(map.observedProofFresh());
    EXPECT_EQ(map.integratedRaySourceStamp(0),front_source);EXPECT_EQ(map.integratedRaySourceStamp(1),rear_source);
    EXPECT_EQ(GridMapTestAccess::rayReceipt(map,0),front_receipt);EXPECT_EQ(GridMapTestAccess::rayReceipt(map,1),rear_receipt);
    if(receipt_expiry)GridMapTestAccess::expireReceipt(map);else GridMapTestAccess::expire(map);
    map.beginObservedProof();EXPECT_EQ(map.observedRawSnapshotColumnStatus({0,0,-1},20),2);
    EXPECT_FALSE(map.observedProofFresh());EXPECT_EQ(map.observedProofDeadlineNs(),1);
  }
}

TEST(MotionSweep, PhysicalSpotEnvelopeRetainsIsotropicReactionFootTopAndBudgetVetoes) {
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();ASSERT_TRUE(braking.valid());
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  for(const Eigen::Vector3d &hit:std::vector<Eigen::Vector3d>{
      {1.5,0.,.025},{-1.5,0.,.025},{0.,1.5,.025},{0.,-1.5,.025},{1.5,0.,1.075}}) {
    GridMap map;GridMapTestAccess::configure(map,{160,160,60});
    GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
    const auto good=validateMotionSweep(map,support,braking,{0.,0.,.48},0.,actual,command,"odom",.1);
    ASSERT_TRUE(good.valid)<<good.reason;EXPECT_NEAR(good.length,.902,1e-12);
    EXPECT_NEAR(good.heading_bound,1.036,1e-12);EXPECT_DOUBLE_EQ(good.min_z,-.02);
    EXPECT_GT(good.unique_voxels,150000U);EXPECT_LE(good.unique_voxels,200000U);
    map.beginObservedProof();ASSERT_EQ(map.getInflateOccupancy({0.,0.,.48},0.),0);
    GridMapTestAccess::nonfloorHit(map,hit);
    map.beginObservedProof();ASSERT_EQ(map.getInflateOccupancy({0.,0.,.48},0.),0);
    const auto blocked=validateMotionSweep(map,support,braking,{0.,0.,.48},0.,actual,command,"odom",.1);
    EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,hit.z()<.05?
        "motion_sweep_unknown_or_expired":"motion_sweep_occupied");
  }
  GridMap map;GridMapTestAccess::configure(map,{160,160,60});
  GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
  const auto over_budget=validateMotionSweep(map,support,braking,{0.,0.,.48},0.,actual,command,"odom",1e-12);
  EXPECT_FALSE(over_budget.valid);EXPECT_EQ(over_budget.reason,"motion_check_budget_exhausted");
}

TEST(MotionSweep, PhysicalSpotRotatedAabbDoesNotConsumeUnqueriedCornerVoxelBudget) {
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();
  const Eigen::Vector3d body(0.,0.,.48);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  for(double yaw:{0.,-10.508*3.141592653589793/180.,3.141592653589793/4.,
      -3.141592653589793/4.,3.141592653589793/2.}) {
    SCOPED_TRACE("body yaw "+std::to_string(yaw));
    GridMap map;GridMapTestAccess::configure(map,{160,160,60});
    GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
    const auto frozen=frozenPhysicalSpotCellVolume(map,body,yaw);
    ASSERT_GT(frozen.inside_voxels,150000U);ASSERT_LE(frozen.inside_voxels,200000U);
    EXPECT_EQ(frozen.inside_voxels,frozen.inside_columns*24U);
    if(std::abs(yaw)>.01&&std::abs(yaw)<1.5)EXPECT_GT(frozen.aabb_voxels,200000U);
    const auto result=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
    ASSERT_TRUE(result.valid)<<result.reason;EXPECT_EQ(result.reason,"observed_free_motion_sweep");
    EXPECT_EQ(result.unique_voxels,frozen.inside_voxels);
    EXPECT_DOUBLE_EQ(result.min_z,frozen.min_z);EXPECT_NEAR(result.max_z,frozen.max_z,1e-12);
    EXPECT_NEAR(result.length,.902,1e-12);EXPECT_NEAR(result.heading_bound,1.036,1e-12);
    EXPECT_TRUE(map.observedProofFresh());
  }
}

TEST(MotionSweep, PhysicalSpotTrueInsideVoxelOverflowStillRejectsEntireVolume) {
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();
  const Eigen::Vector3d body(0.,0.,.48);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;actual.linear.z=.5;
  for(double yaw:{0.,-10.508*3.141592653589793/180.,3.141592653589793/4.}) {
    SCOPED_TRACE("body yaw "+std::to_string(yaw));
    GridMap map;GridMapTestAccess::configure(map,{160,160,60});
    GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
    const auto frozen=frozenPhysicalSpotCellVolume(map,body,yaw,actual.linear.z);
    ASSERT_GT(frozen.inside_voxels,200000U);EXPECT_EQ(frozen.inside_voxels,frozen.inside_columns*31U);
    const auto result=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
    EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"motion_volume_budget_exhausted");
    EXPECT_DOUBLE_EQ(result.min_z,-.02);EXPECT_NEAR(result.max_z,frozen.max_z,1e-12);
    EXPECT_NEAR(result.length,.902,1e-12);EXPECT_NEAR(result.heading_bound,1.036,1e-12);
  }
}

TEST(MotionSweep, PhysicalSpotRotatedFullVolumeStillFindsFootAndTopLiveObstacles) {
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();
  const Eigen::Vector3d body(0.,0.,.48);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  for(double yaw:{-10.508*3.141592653589793/180.,3.141592653589793/4.})
    for(double hit_z:{.025,1.075}) {
      SCOPED_TRACE("body yaw "+std::to_string(yaw)+", hit z "+std::to_string(hit_z));
      GridMap map;GridMapTestAccess::configure(map,{160,160,60});
      GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
      const auto frozen=frozenPhysicalSpotCellVolume(map,body,yaw);
      ASSERT_GT(frozen.aabb_voxels,200000U);ASSERT_LE(frozen.inside_voxels,200000U);
      const auto clear=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
      ASSERT_TRUE(clear.valid)<<clear.reason;EXPECT_EQ(clear.unique_voxels,frozen.inside_voxels);
      // The obstacle is beyond the current body's forward envelope, but is
      // inside the unchanged isotropic future reach at the rotated pose.
      const Eigen::Vector3d hit(1.5*std::cos(yaw),1.5*std::sin(yaw),hit_z);
      GridMapTestAccess::nonfloorHit(map,hit);
      map.beginObservedProof();ASSERT_EQ(map.getInflateOccupancy(body,yaw),0);
      const auto blocked=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
      EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,hit_z<.05?
          "motion_sweep_unknown_or_expired":"motion_sweep_occupied");
      EXPECT_GT(blocked.unique_voxels,0U);EXPECT_LE(blocked.unique_voxels,frozen.inside_voxels);
      EXPECT_DOUBLE_EQ(blocked.min_z,frozen.min_z);EXPECT_NEAR(blocked.max_z,frozen.max_z,1e-12);
    }
}

TEST(MotionSweep, PhysicalSpotRotatedCountPassCannotRenewEitherRaySourceOrReceipt) {
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();
  const Eigen::Vector3d body(0.,0.,.48);const double yaw=-10.508*3.141592653589793/180.;
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  for(unsigned sensor:{0U,1U})for(bool receipt_expiry:{false,true}) {
    SCOPED_TRACE("sensor "+std::to_string(sensor)+", receipt expiry "+std::to_string(receipt_expiry));
    GridMap map;GridMapTestAccess::configure(map,{160,160,60});
    GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
    const auto frozen=frozenPhysicalSpotCellVolume(map,body,yaw);
    ASSERT_GT(frozen.aabb_voxels,200000U);ASSERT_LE(frozen.inside_voxels,200000U);
    const auto clear=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
    ASSERT_TRUE(clear.valid)<<clear.reason;
    if(receipt_expiry)GridMapTestAccess::expireReceipt(map,sensor);
    else GridMapTestAccess::expireRaySource(map,sensor);
    const std::array<std::int64_t,2> sources{{map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1)}};
    const std::array<std::int64_t,2> receipts{{GridMapTestAccess::rayReceipt(map,0),GridMapTestAccess::rayReceipt(map,1)}};
    for(unsigned repeat=0;repeat<2;++repeat) {
      const auto rejected=validateMotionSweep(map,support,braking,body,yaw,actual,command,"odom",.04);
      EXPECT_FALSE(rejected.valid);EXPECT_EQ(rejected.reason,"motion_sweep_unknown_or_expired");
      EXPECT_FALSE(map.observedProofFresh());EXPECT_EQ(map.observedProofDeadlineNs(),1);
      for(unsigned ray=0;ray<2;++ray) {
        EXPECT_EQ(map.integratedRaySourceStamp(ray),sources[ray]);
        EXPECT_EQ(GridMapTestAccess::rayReceipt(map,ray),receipts[ray]);
      }
    }
  }
}

TEST(MotionSweepBenchmark, PhysicalSpotColdProofWallP50P95KeepsEveryOriginalCell) {
  GridMap map;GridMapTestAccess::configure(map,{160,160,60});
  GridMapTestAccess::officialSpotEnvelope(map);GridMapTestAccess::floorContact(map);
  const auto support=physicalSpotGround();const auto braking=physicalSpotModel();
  geometry_msgs::msg::Twist command,actual;command.linear.x=.01;actual.linear.x=.001;
  std::vector<double> wall_ms;std::set<std::size_t> covered_counts;
  for(unsigned iteration=0;iteration<25;++iteration) {
    GridMapTestAccess::newBenchmarkAcquisition(map,iteration);
    const auto begin=std::chrono::steady_clock::now();
    const auto result=validateMotionSweep(map,support,braking,{0.,0.,.48},0.,actual,command,"odom",.1);
    wall_ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
    ASSERT_TRUE(result.valid)<<result.reason;EXPECT_NEAR(result.length,.902,1e-12);
    EXPECT_NEAR(result.heading_bound,1.036,1e-12);EXPECT_DOUBLE_EQ(result.min_z,-.02);
    EXPECT_GT(result.unique_voxels,150000U);EXPECT_LE(result.unique_voxels,200000U);
    EXPECT_TRUE(map.observedProofFresh());covered_counts.insert(result.unique_voxels);
  }
  EXPECT_EQ(covered_counts.size(),1U);std::sort(wall_ms.begin(),wall_ms.end());
  std::cout<<nlohmann::json{{"probe","physical_spot_cold_motion_columns"},{"iterations",wall_ms.size()},
    {"wall_ms_p50",wall_ms[wall_ms.size()/2]},{"wall_ms_p95",wall_ms[(wall_ms.size()*95-1)/100]},
    {"unique_voxels",*covered_counts.begin()},{"map_shape",{160,160,60}},{"resolution",.05},
    {"body_radius",.608},{"body_offset",.3125},{"above",.55},{"floor_lower",-.02},
    {"reaction_length",.902},{"yaw_bound",1.036},{"clock","steady_clock"},
    {"setup_excluded",true},{"performance_assertion",false}}.dump()<<std::endl;
}
