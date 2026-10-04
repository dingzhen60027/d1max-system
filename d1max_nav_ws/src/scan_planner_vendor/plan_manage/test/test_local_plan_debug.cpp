#include <gtest/gtest.h>
#include <bspline_opt/reference_path.hpp>
#include <plan_manage/local_plan_debug.hpp>

#include <limits>

using Eigen::Vector3d;

namespace {
std_msgs::msg::Header mapHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "d1max_loc_map";
  header.stamp.sec = 42;
  header.stamp.nanosec = 123;
  return header;
}
}  // namespace

TEST(LocalPlanDebug, CarriesExactlyTheNativeSelectedReferenceAndTrajectoryId) {
  scan_planner::DiscreteReference reference;
  reference.set({Vector3d(0, 0, .55), Vector3d(2, 0, .55),
                 Vector3d(2, 2, .55), Vector3d(4, 2, .55)});
  const Vector3d actual_start(.7, .1, .55);
  const double progress = reference.project(actual_start, 0.0, 2.0);
  const double target_arc = std::min(reference.length(), progress + 2.5);
  const auto selected = reference.slice(progress, target_arc, actual_start);
  const auto message = scan_planner::makeAcceptedLocalPlanDebug(
      mapHeader(), "live_session", 7, 17, reference.sample(progress),
      reference.sample(target_arc), progress, target_arc, selected);

  ASSERT_TRUE(message.has_value());
  EXPECT_TRUE(message->valid);
  EXPECT_EQ(message->phase, "accepted");
  EXPECT_EQ(message->session_id, "live_session");
  EXPECT_EQ(message->generation, 7U);
  EXPECT_EQ(message->plan_id, 17U);
  EXPECT_FALSE(message->predecessor_safe);
  EXPECT_EQ(message->predecessor_id, 0U);
  EXPECT_EQ(message->predecessor_check_stamp.sec, 0);
  EXPECT_DOUBLE_EQ(message->progress_arc_m, progress);
  EXPECT_DOUBLE_EQ(message->target_arc_m, target_arc);
  EXPECT_EQ(message->selected_reference.header.frame_id, "d1max_loc_map");
  EXPECT_EQ(message->selected_reference.header.stamp.sec, 42);
  ASSERT_EQ(message->selected_reference.poses.size(), selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i) {
    const auto &pose = message->selected_reference.poses[i];
    EXPECT_EQ(pose.header.frame_id, "d1max_loc_map");
    EXPECT_EQ(pose.header.stamp.sec, 42);
    EXPECT_TRUE(Vector3d(pose.pose.position.x, pose.pose.position.y,
                         pose.pose.position.z).isApprox(selected[i]));
    EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
  }
  EXPECT_TRUE(Vector3d(message->selected_reference.poses.back().pose.position.x,
                       message->selected_reference.poses.back().pose.position.y,
                       message->selected_reference.poses.back().pose.position.z)
                  .isApprox(reference.sample(target_arc)));
}

TEST(LocalPlanDebug, InvalidAlwaysClearsGeometryAndCarriesLastPlanId) {
  const auto message = scan_planner::makeInvalidLocalPlanDebug(
      mapHeader(), "live_session", 8, 17, "cancelled");
  EXPECT_FALSE(message.valid);
  EXPECT_FALSE(message.predecessor_safe);
  EXPECT_EQ(message.predecessor_id, 0U);
  EXPECT_EQ(message.phase, "cancelled");
  EXPECT_EQ(message.plan_id, 17U);
  EXPECT_TRUE(message.selected_reference.poses.empty());
  EXPECT_EQ(message.selected_reference.header.frame_id, "d1max_loc_map");
}

TEST(LocalPlanDebug, ReferenceRejectionsRetainIdentityAndSourceStampWithoutGeometry) {
  for (const char *phase : {"reference_rejected_odometry", "reference_rejected_frame",
                            "reference_rejected_geometry"}) {
    const auto message = scan_planner::makeInvalidLocalPlanDebug(
        mapHeader(), "live_session", 9, 17, phase);
    EXPECT_FALSE(message.valid);
    EXPECT_EQ(message.phase, phase);
    EXPECT_EQ(message.session_id, "live_session");
    EXPECT_EQ(message.generation, 9U);
    EXPECT_EQ(message.header.frame_id, "d1max_loc_map");
    EXPECT_EQ(message.header.stamp.sec, 42);
    EXPECT_EQ(message.header.stamp.nanosec, 123U);
    EXPECT_EQ(message.selected_reference.header, message.header);
    EXPECT_TRUE(message.selected_reference.poses.empty());
    EXPECT_FALSE(message.predecessor_safe);
  }
}

TEST(LocalPlanDebug, FailsClosedOnOversizedOrInvalidDiagnosticWithoutChangingPlannerInput) {
  std::vector<Vector3d> selected(scan_planner::kMaxLocalDebugReferencePoints + 1,
                                 Vector3d(0, 0, .55));
  selected.back() = Vector3d(1, 0, .55);
  EXPECT_FALSE(scan_planner::makeAcceptedLocalPlanDebug(
      mapHeader(), "live_session", 8, 18, Vector3d(0, 0, .55),
      Vector3d(1, 0, .55), 0.0, 1.0, selected).has_value());
  EXPECT_EQ(selected.size(), scan_planner::kMaxLocalDebugReferencePoints + 1);

  selected = {Vector3d(0, 0, .55), Vector3d(1, 0, .55)};
  selected.front().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scan_planner::makeAcceptedLocalPlanDebug(
      mapHeader(), "live_session", 8, 18, Vector3d(0, 0, .55),
      Vector3d(1, 0, .55), 0.0, 1.0, selected).has_value());
}

TEST(LocalPlanDebug, RecheckPreservesAcceptedIdentityGeometryAndOriginalEvidence) {
  const std::vector<Vector3d> selected{Vector3d(0,0,.55),Vector3d(1,0,.55)};
  auto accepted=scan_planner::makeAcceptedLocalPlanDebug(mapHeader(),"session",7,17,
      selected.front(),selected.back(),0.,1.,selected);
  ASSERT_TRUE(accepted);
  auto header=mapHeader(); header.stamp.sec=45;
  auto proof=scan_planner::makeRevalidatedLocalPlanDebug(*accepted,header,
      44700000000LL,44900000000LL,100,2);
  ASSERT_TRUE(proof);
  EXPECT_EQ(proof->phase,"revalidated");
  EXPECT_TRUE(proof->valid);
  EXPECT_EQ(proof->generation,accepted->generation);
  EXPECT_EQ(proof->plan_id,accepted->plan_id);
  EXPECT_EQ(proof->session_id,accepted->session_id);
  EXPECT_EQ(proof->projection,accepted->projection);
  EXPECT_EQ(proof->local_target,accepted->local_target);
  EXPECT_EQ(proof->progress_arc_m,accepted->progress_arc_m);
  EXPECT_EQ(proof->target_arc_m,accepted->target_arc_m);
  ASSERT_EQ(proof->selected_reference.poses.size(),accepted->selected_reference.poses.size());
  for (std::size_t i=0;i<selected.size();++i) {
    EXPECT_EQ(proof->selected_reference.poses[i].pose,accepted->selected_reference.poses[i].pose);
    EXPECT_EQ(proof->selected_reference.poses[i].header,header);
  }
  EXPECT_EQ(proof->checked_map_source_stamp_ns,44700000000LL);
  EXPECT_EQ(proof->checked_body_source_stamp_ns,44900000000LL);
  EXPECT_EQ(proof->checked_map_revision,100U);
  EXPECT_EQ(proof->checked_context_sequence,2U);
  EXPECT_EQ(accepted->header.stamp.sec,42); // no renewed original lease
  EXPECT_EQ(accepted->phase,"accepted");
  EXPECT_EQ(accepted->checked_map_source_stamp_ns,0);
  EXPECT_FALSE(proof->predecessor_safe); // cannot masquerade as controller handover
}

TEST(LocalPlanDebug, RecheckCannotCreateAcceptanceOrRenewInvalidIdentity) {
  const std::vector<Vector3d> selected{Vector3d(0,0,.55),Vector3d(1,0,.55)};
  auto accepted=*scan_planner::makeAcceptedLocalPlanDebug(mapHeader(),"session",7,17,
      selected.front(),selected.back(),0.,1.,selected);
  auto header=mapHeader(); header.stamp.sec=45;
  const auto proof=[&](const auto &original,const auto &check,std::int64_t map=44700000000LL,
                       std::int64_t body=44900000000LL,std::uint64_t context=2) {
    return scan_planner::makeRevalidatedLocalPlanDebug(original,check,map,body,100,context);
  };
  auto invalid=accepted; invalid.valid=false;
  EXPECT_FALSE(proof(invalid,header));
  invalid=accepted; invalid.phase="revalidated";
  EXPECT_FALSE(proof(invalid,header));
  invalid=accepted; invalid.plan_id=0;
  EXPECT_FALSE(proof(invalid,header));
  invalid=accepted; invalid.selected_reference.poses.clear();
  EXPECT_FALSE(proof(invalid,header));
  auto wrong=header; wrong.frame_id="foreign";
  EXPECT_FALSE(proof(accepted,wrong));
  EXPECT_FALSE(proof(accepted,accepted.header));
  EXPECT_FALSE(proof(accepted,header,0));
  EXPECT_FALSE(proof(accepted,header,46000000000LL));
  EXPECT_FALSE(proof(accepted,header,44700000000LL,0));
  EXPECT_FALSE(proof(accepted,header,44700000000LL,46000000000LL));
  EXPECT_FALSE(proof(accepted,header,44700000000LL,44900000000LL,0));
}
