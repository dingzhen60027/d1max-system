#include <gtest/gtest.h>
#include <plan_manage/reference_failure_diagnostics.hpp>

struct GridMapTestAccess {
  static void configure(GridMap& map) {
    auto &p=map.mp_;auto &d=map.md_;
    p.require_observed_free_=false;p.use_projected_rays_=false;
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.body_below_=p.body_above_=.25;p.frame_id_="odom";
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={64,64,64};p.map_origin_idx_.setZero();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.34;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    const auto size=64U*64U*64U;
    d.occupancy_buffer_.resize(size);d.occupancy_buffer_inflate_.resize(size);
    d.occupancy_buffer_inflate_cnt_.resize(size);d.count_hit_.resize(size);d.count_hit_and_miss_.resize(size);
    d.flag_rayend_.resize(size);d.flag_traverse_.resize(size);d.has_ray_pose_=false;
    map.updateMapBoundaryFromIndex();map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
};

namespace {
using namespace scan_planner;
MeasuredBodyPose acquired(){return {{0.,0.,.5},Eigen::Quaterniond::Identity(),10.,"odom"};}
ReferenceSearchFailureDiagnostic internal(){return {{.2,-.23,.5},{.2,-1.,.5},{0.,0.,.5},{0.,-1.,.5},-std::acos(-1.)*.5,4,false};}

TEST(ReferenceFailureDiagnostics, InternalAnchorIsNotMislabelledMeasuredBody) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupied({.225,-.625,.325});
  auto query=internal();
  ASSERT_EQ(map.getInflateOccupancy(query.start,query.query_yaw),1);
  ASSERT_EQ(map.getInflateOccupancy(acquired().position,0.,0.,0.),0);
  const auto raw=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
  const auto diagnostic=describeReferenceSearchFailure(map,query,acquired(),"odom",10.2,.4,{});
  EXPECT_NE(diagnostic.find("role=intermediate_reference_anchor"),std::string::npos);
  EXPECT_NE(diagnostic.find("search_start_evidence={query_policy=official_occupied_oriented_box collision=1"),std::string::npos);
  EXPECT_NE(diagnostic.find("acquired_body_evidence={query_policy=official_occupied_oriented_box collision=0"),std::string::npos);
  EXPECT_NE(diagnostic.find("acquired_body_source_s=10.000000000"),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);EXPECT_EQ(map.occupancyRevision(),revision);
  EXPECT_EQ(map.getInflateOccupancy(query.start,query.query_yaw),1);
  EXPECT_EQ(map.getInflateOccupancy(acquired().position,0.,0.,0.),0);
}

TEST(ReferenceFailureDiagnostics, SolveStartMayBeSpliceAndIsNeverCalledAcquired) {
  GridMap map;GridMapTestAccess::configure(map);auto query=internal();
  query.solve_start=query.start;query.planned_splice=true;
  const auto diagnostic=describeReferenceSearchFailure(map,query,acquired(),"odom",10.2,.4,{});
  EXPECT_STREQ(referenceSearchStartRole(query),"solve_start");
  EXPECT_NE(diagnostic.find("planned_splice=1"),std::string::npos);
  const std::string marker="query_to_acquired_body_m=";
  const auto distance=diagnostic.find(marker);ASSERT_NE(distance,std::string::npos);
  EXPECT_NEAR(std::stod(diagnostic.substr(distance+marker.size())),std::hypot(.2,.23),1e-9);
  EXPECT_EQ(diagnostic.find("role=measured_start"),std::string::npos);
}

TEST(ReferenceFailureDiagnostics, LiteralAttitudeDoesNotUsePredictedSupportAtBody) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupied({.425,.025,.325});
  auto body=acquired();body.position={.025,.025,.5};
  auto query=internal();query.start=body.position;query.query_yaw=0.;
  map.setQueryAttitudeProvider([](const auto&,double){return std::optional<Eigen::Vector2d>{{-std::acos(-1.)/6.,0.}};});
  const auto diagnostic=describeReferenceSearchFailure(map,query,body,"odom",10.2,.4,{});
  EXPECT_NE(diagnostic.find("search_start_evidence={query_policy=official_occupied_oriented_box collision=0"),std::string::npos);
  EXPECT_NE(diagnostic.find("acquired_body_evidence={query_policy=official_occupied_oriented_box collision=1"),std::string::npos);
  EXPECT_NE(diagnostic.find("acquired_body_rpy=[0,-0,0]"),std::string::npos);
}

TEST(ReferenceFailureDiagnostics, MissingBudgetDoesNotCreateNewDeadline) {
  GridMap map;GridMapTestAccess::configure(map);int calls=0;
  map.setQueryAttitudeProvider([&](const auto&,double){++calls;return std::optional<Eigen::Vector2d>{{0.,0.}};});
  const auto clock=SolveBudget::Time{};
  const auto budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(5),[&]{return clock;});
  const auto diagnostic=describeReferenceSearchFailure(map,internal(),acquired(),"odom",10.2,.4,budget);
  EXPECT_EQ(calls,0);EXPECT_NE(diagnostic.find("evidence=skipped_diagnostic_budget"),std::string::npos);
  EXPECT_TRUE(budget->allowed());EXPECT_DOUBLE_EQ(budget->remainingSeconds(),.005);
}

TEST(ReferenceFailureDiagnostics, CancelledBudgetDoesNotInspectOrRewriteStopReason) {
  GridMap map;GridMapTestAccess::configure(map);int calls=0;
  map.setQueryAttitudeProvider([&](const auto&,double){++calls;return std::optional<Eigen::Vector2d>{{0.,0.}};});
  const auto budget=std::make_shared<SolveBudget>();budget->cancel();
  const auto diagnostic=describeReferenceSearchFailure(map,internal(),acquired(),"odom",10.2,.4,budget);
  EXPECT_EQ(calls,0);EXPECT_STREQ(budget->reason(),"solve_cancelled");
  EXPECT_NE(diagnostic.find("evidence=skipped_diagnostic_budget"),std::string::npos);
}

TEST(ReferenceFailureDiagnostics, WrongFrameOrStalePoseDoesNotInventAcquiredEvidence) {
  GridMap map;GridMapTestAccess::configure(map);
  auto body=acquired();body.frame="map";
  const auto wrong=describeReferenceSearchFailure(map,internal(),body,"map",10.2,.4,{});
  EXPECT_NE(wrong.find("acquired_body_evidence=unavailable_frame_pose_or_source"),std::string::npos);
  const auto stale=describeReferenceSearchFailure(map,internal(),acquired(),"odom",11.,.4,{});
  EXPECT_NE(stale.find("acquired_body_evidence=unavailable_frame_pose_or_source"),std::string::npos);
  EXPECT_EQ(wrong.find("acquired_body_evidence={"),std::string::npos);
  EXPECT_EQ(stale.find("acquired_body_evidence={"),std::string::npos);
}
TEST(ReferenceFailureDiagnostics, CurrentCurveWitnessKeepsQuerySeparateFromActualBodyWithoutMutation) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupied({.225,-.625,.325});
  const auto q=internal();const auto raw=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
  const auto detail=describeCurveCollisionWitness(map,q.start,q.query_yaw,acquired());
  EXPECT_NE(detail.find("query_evidence={query_policy=official_occupied_oriented_box collision=1"),std::string::npos);
  EXPECT_NE(detail.find("acquired_body_evidence={query_policy=official_occupied_oriented_box collision=0"),std::string::npos);
  EXPECT_NE(detail.find("raw_overlap="),std::string::npos);EXPECT_NE(detail.find("unique_voxels="),std::string::npos);
  EXPECT_NE(detail.find("front_source_ns="),std::string::npos);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);EXPECT_EQ(map.occupancyRevision(),revision);
}
} // namespace
