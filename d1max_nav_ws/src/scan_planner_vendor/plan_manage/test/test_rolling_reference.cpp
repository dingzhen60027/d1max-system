#include <gtest/gtest.h>
#include <plan_manage/rolling_reference.hpp>
#include <d1max_planning_interfaces/msg/reference_path.hpp>
#include <d1max_trajectory_tracker/velocity_smoother.hpp>

namespace {
using Ref=d1max_planning_interfaces::msg::ReferencePath;
using Point=Eigen::Vector3d;
Ref window(const scan_planner::DiscreteReference& route,double start,std::uint64_t generation) {
  Ref r;r.schema_version=2;r.session_id="session";r.task_id="task";r.route_id="route";
  r.route_hash="hash";r.map_version_id="map";r.segment_id="floor";r.segment_kind="corridor";
  r.required_mode="walk";r.point_reference="body_center";r.anchor_id="anchor";
  r.anchor_revision=1;r.context_sequence=1;r.localization_epoch=1;r.localization_seed_id="seed";
  r.path.header.frame_id="odom";r.generation=generation;
  // The real ReferenceCallbacks contract: np.linspace(start,start+4,41),
  // not preservation of every original route vertex at a shifted window.
  for(int i=0;i<=40;++i) {
    const auto p=route.sample(start+4.*i/40.);geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id="odom";pose.pose.orientation.w=1.;
    pose.pose.position.x=p.x();pose.pose.position.y=p.y();pose.pose.position.z=p.z();
    r.path.poses.push_back(pose);r.point_segment_ids.push_back("floor");
    r.point_segment_kinds.push_back("corridor");r.point_required_modes.push_back("walk");
  }
  return r;
}
scan_planner::DiscreteReference line(){scan_planner::DiscreteReference r;r.set({Point(0,0,.55),Point(10,0,.55)});return r;}
Ref stairWindow(double start,std::uint64_t generation,double end_x=10.) {
  scan_planner::DiscreteReference route;
  route.set({Point(0,0,.55),Point(2,0,.55),Point(5,0,2.05),Point(end_x,0,2.05)});
  auto r=window(route,start,generation);
  for(std::size_t i=0;i<r.path.poses.size();++i) {
    const double x=r.path.poses[i].pose.position.x;
    r.point_segment_ids[i]=x<2.?"floor1":x<5.?"stair_lower":"landing";
    r.point_segment_kinds[i]=x>=2.&&x<5.?"stair_up":"floor";
    r.point_required_modes[i]=x>=2.&&x<5.?"stair":"general";
  }
  r.segment_id=r.point_segment_ids.front();r.segment_kind=r.point_segment_kinds.front();
  r.required_mode=r.point_required_modes.front();return r;
}
}

TEST(RollingReference, ActualWindowResamplingPreservesStraightCornerAndCurvedRoute) {
  auto straight=line();
  EXPECT_TRUE(scan_planner::rollingReferenceExtension(window(straight,0,1),window(straight,.25,2)));
  scan_planner::DiscreteReference corner;
  corner.set({Point(0,0,.55),Point(2.035,0,.55),Point(2.035,5,.55)});
  for(double start:{0.,.25,.5,.75,1.})
    EXPECT_TRUE(scan_planner::rollingReferenceExtension(window(corner,start,1),window(corner,start+.25,2)))<<start;
  std::vector<Point> arc;
  for(int i=0;i<=160;++i){const double t=.025*i;arc.emplace_back(2.*std::sin(t),2.*(1.-std::cos(t)),.55);}
  scan_planner::DiscreteReference curved;curved.set(arc);
  EXPECT_TRUE(scan_planner::rollingReferenceExtension(window(curved,0.,1),window(curved,.25,2)));
}

TEST(RollingReference, SemanticAnchorTaskAndHeightChangesAreNotSoftUpdates) {
  const auto a=window(line(),0,1),b=window(line(),.25,2);
  auto changed=b;changed.task_id="other";EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;++changed.anchor_revision;EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.segment_id="stairs";EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.point_segment_kinds[10]="stairs";EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.path.header.frame_id="map";EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;for(auto& p:changed.path.poses)p.pose.position.z+=3.;
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;for(auto& p:changed.path.poses)p.pose.position.y+=.2;
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
}

TEST(RollingReference, ReverseShortenedCancelledAndMalformedWindowsRemainHard) {
  const auto a=window(line(),0,1),b=window(line(),.25,2);
  auto changed=b;std::reverse(changed.path.poses.begin(),changed.path.poses.end());
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.path.poses.clear();EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.path.poses.back().pose.position.x=3.;EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.path.poses.back().pose.position.x=1e200;EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.path.poses[3].pose.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
  changed=b;changed.generation=a.generation;EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed));
}

TEST(RollingReference, WorkReserveMatchesControllerSoftSStopWithoutChangingStopBoundary) {
  for(double speed:{0.,.1,.3,.45})for(double acceleration:{0.,.1,.35})for(double jerk:{0.,.7,2.}) {
    const double stop=d1max_trajectory_tracker::jerkStoppingDistance(speed,acceleration,.175,.5*jerk);
    EXPECT_NEAR(scan_planner::continuationPlanningDistance(speed,acceleration,.35,jerk),
      stop+speed*(.4+.8+.05),1e-12);
  }
  EXPECT_NEAR(scan_planner::continuationPlanningDistance(.3,0.,.35,.7),.707142857,1e-8);
  EXPECT_GT(scan_planner::continuationPlanningDistance(.3,.35,.35,.7),.96);
  EXPECT_EQ(scan_planner::continuationPlanningDistance(.3,0.,0.,.7),0.);
  EXPECT_EQ(scan_planner::continuationPlanningDistance(std::numeric_limits<double>::infinity(),0.,.35),0.);
}

TEST(RollingReference, FixedStairKeepsOrderedMixedWindowsAcrossEntryAndExit) {
  for(const auto start:{0.,1.9,5.25}) {
    const auto a=stairWindow(start,1),b=stairWindow(start+.25,2);
    EXPECT_TRUE(scan_planner::rollingReferenceExtension(a,b,true))<<start;
    EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,b))<<start;
  }
  EXPECT_TRUE(scan_planner::continuousStairReferenceWindow(stairWindow(0.,1)));
  EXPECT_FALSE(scan_planner::continuousStairReferenceWindow(window(line(),0.,1)));
}

TEST(RollingReference, FixedStairCannotRelabelOverlapSkipPortalOrChangeCoordinateIdentity) {
  const auto a=stairWindow(1.9,1),b=stairWindow(2.15,2);
  ASSERT_TRUE(scan_planner::rollingReferenceExtension(a,b,true));
  auto changed=b;changed.segment_id="landing";changed.segment_kind="floor";changed.required_mode="general";
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true)); // not immediate next header
  changed=b;changed.point_segment_ids[8]="unrelated_stair";
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true)); // non-contiguous labels
  changed=b;for(std::size_t i=0;i<changed.path.poses.size();++i)if(changed.point_segment_ids[i]=="stair_lower") {
    changed.point_segment_ids[i]="other_stair";
  }
  changed.segment_id="other_stair";EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true));
  changed=b;++changed.anchor_revision;EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true));
  changed=b;for(auto& p:changed.path.poses)p.pose.position.z+=3.;
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true));
}

TEST(RollingReference, FixedStairDoesNotChangeFlatRollingClassification) {
  const auto a=window(line(),0,1),b=window(line(),.25,2);
  EXPECT_TRUE(scan_planner::rollingReferenceExtension(a,b,true));
  auto changed=b;changed.segment_id="another_floor_section";
  for(auto& id:changed.point_segment_ids)id=changed.segment_id;
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true));
}

TEST(RollingReference, FixedStairClipsPrefetchedFinalTailWithoutDestructiveReferenceReplacement) {
  const auto a=stairWindow(5.25,1,6.),b=stairWindow(5.5,2,6.);
  ASSERT_EQ(a.segment_id,"stair_lower");ASSERT_EQ(b.segment_id,"landing");
  ASSERT_EQ(a.path.poses.back(),b.path.poses.back());
  EXPECT_TRUE(scan_planner::rollingReferenceExtension(a,b,true));
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,b));
  auto shortened=b;for(auto& p:shortened.path.poses)if(p.pose.position.x>5.8)p.pose.position.x=5.8;
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,shortened,true));
}

TEST(RollingReference, FixedStairLiteralPortalMayChangeOnlyItsFirstVertexOwner) {
  auto a=stairWindow(2.,1);auto b=stairWindow(2.,2);
  a.segment_id=a.point_segment_ids.front()="floor1";
  a.segment_kind=a.point_segment_kinds.front()="floor";
  a.required_mode=a.point_required_modes.front()="general";
  EXPECT_TRUE(scan_planner::rollingReferenceExtension(a,b,true));
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,b));
  auto changed=a;changed.point_segment_ids[1]="floor1";
  changed.point_segment_kinds[1]="floor";changed.point_required_modes[1]="general";
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(changed,b,true));
  changed=b;changed.segment_id="landing";changed.segment_kind="floor";changed.required_mode="general";
  EXPECT_FALSE(scan_planner::rollingReferenceExtension(a,changed,true));
}
