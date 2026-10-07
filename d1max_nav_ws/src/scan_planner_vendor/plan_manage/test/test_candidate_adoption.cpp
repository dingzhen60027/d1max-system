// Actual cubic projection/adoption transaction and production GridMap queries.
// In-memory native map inputs and injected clocks only: no ROS init or robot.
#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <plan_manage/candidate_adoption.hpp>
#include <plan_manage/worker_lifecycle.hpp>
#include <plan_env/collision_snapshot_pool.hpp>
#include <future>

struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.1;
    p.obstacles_inflation_z_up=.15;p.obstacles_inflation_z_down=.15;
    p.cloud_pose_max_age_=.5;
    d.occupancy_buffer_.assign(64*64*32,-1.);
    d.occupancy_buffer_inflate_.assign(64*64*32,0);
    d.occupancy_buffer_inflate_cnt_.assign(64*64*32,0);
    map->integrated_cloud_stamp_ns_=10300000000LL;
    map->localization_context_sequence_=7;
    map->rebuildInflationOffsets();return map;
  }
  static void raw(GridMap &map,const Eigen::Vector3d &p,int value) {
    Eigen::Vector3i cell;map.posToIndex(p,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=value==1?2.:-1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static void source(GridMap &map,std::int64_t time) {map.integrated_cloud_stamp_ns_=time;}
  static void revision(GridMap &map) {++map.occupancy_revision_;}
  static void context(GridMap &map) {++map.localization_context_sequence_;}
};

namespace {
using namespace scan_planner;
UniformBspline line(double speed=.3) {
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i) points.col(i)=Eigen::Vector3d((i-1)*speed*.2,0.,.55);
  return UniformBspline(points,3,.2);
}
struct Fixture {
  GridMap::Ptr map=GridMapTestAccess::make();
  UniformBspline curve=line();
  MeasuredBodyPose original{{0.,0.,.55},Eigen::Quaterniond::Identity(),10.,"map"};
  MeasuredBodyPose measured{{.09,0.,.55},Eigen::Quaterniond::Identity(),10.3,"map"};
  Eigen::Vector3d velocity{.3,0.,0.};
  SolveBudget::Time steady=SolveBudget::Clock::now();
  SolveBudget::Ptr budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(400),[this]{return steady;});
  double now=10.31,body_max_age=.5;
  bool single_segment=true;
  std::uint64_t candidate_context=7;
  int checks=0;
  std::function<void()> during_check=[]{};
  std::optional<CandidateJoinEvidence> certify() {
    return certifyCandidateAdoption(curve,original,measured,velocity,.08,.3,.6,
        single_segment,candidate_context,budget,[&] {
      return CandidateSourceLease{map->latestCloudStampNs(),map->localizationContextSequence(),
          map->occupancyRevision(),measured.source_stamp};
    },[&] {
      double yaw=0.;
      return std::make_pair(map->integratedCloudFreshAt(std::llround(now*1e9)),
          measuredBodyYaw(measured,"map",now,body_max_age,yaw));
    },[&](double time) {
      ++checks;during_check();
      return wholeCurveCollisionFree(curve,.08,.2,
          [&](const Eigen::Vector3d &p,double yaw){return map->getInflateOccupancy(p,yaw);},
          measured,"map",10.31,.5,200000,budget->remainingSeconds(),time);
    });
  }
};
}

TEST(CandidateAdoption, MovingNineCentimetersJoinsOriginalCurveInsteadOfTZero) {
  Fixture f;const auto controls=f.curve.getControlPoint();const auto knots=f.curve.getKnot();
  EXPECT_FALSE(wholeCurveCollisionFree(f.curve,.08,.2,
      [&](const Eigen::Vector3d &p,double yaw){return f.map->getInflateOccupancy(p,yaw);},
      f.measured,"map",f.now,.5)); // Frozen t=0 production bug.
  const auto result=f.certify();ASSERT_TRUE(result);
  EXPECT_NEAR(result->curve_time,.3,.02);EXPECT_NEAR(result->arc_length,.09,.006);
  EXPECT_TRUE(result->measured.position.isApprox(f.measured.position));
  EXPECT_EQ(result->measured.source_stamp,10.3);EXPECT_TRUE(result->velocity.isApprox(f.velocity));
  EXPECT_FALSE(result->acceleration_valid); // No invented measured acceleration/C2 claim.
  EXPECT_TRUE(f.curve.getControlPoint().isApprox(controls));EXPECT_TRUE(f.curve.getKnot().isApprox(knots));
}

TEST(CandidateAdoption, UnixSourceOrderAndElapsedBoundUseOriginalNanoseconds) {
  constexpr std::int64_t ns=1791124691902856036LL;
  Fixture f;f.original.source_stamp=f.measured.source_stamp=static_cast<double>(ns)*1e-9;
  f.original.source_stamp_ns=f.measured.source_stamp_ns=ns;
  f.measured.position=f.original.position;
  const auto join=[&] {return measuredCandidateJoin(f.curve,f.original,f.measured,f.velocity,.08,.3,.6,true,f.budget);};
  const auto equal=join();ASSERT_TRUE(equal);EXPECT_EQ(equal->measured.source_stamp_ns,ns);
  f.measured.source_stamp_ns=ns-1;EXPECT_FALSE(join()); // the doubles still compare equal
  f.measured.source_stamp_ns=ns+400000000LL;EXPECT_TRUE(join());
  f.measured.source_stamp_ns=ns+400000001LL;EXPECT_FALSE(join());
}

TEST(CandidateAdoption, IndependentXyzReferenceAndTravelKeepRawZAndMeasuredEntry) {
  Fixture f;const Eigen::Vector3d raw{.36,0.,.30};
  Eigen::MatrixXd controls(3,23);
  for(int i=0;i<23;++i)controls.col(i)=f.original.position+(i-1)*.2*raw;
  f.curve=UniformBspline(controls,3,.2);f.velocity=raw;
  f.measured.position=f.original.position+.3*raw;
  EXPECT_FALSE(measuredCandidateJoin(f.curve,f.original,f.measured,raw,.08,.15,.6,true,f.budget));
  const auto joined=measuredCandidateJoin(f.curve,f.original,f.measured,raw,.08,.5,.6,true,f.budget,.5);
  ASSERT_TRUE(joined);EXPECT_NEAR(joined->curve_time,.3,.02);
  EXPECT_TRUE(joined->velocity.isApprox(raw));EXPECT_DOUBLE_EQ(joined->velocity.z(),.30);
  EXPECT_TRUE(joined->measured.position.isApprox(f.measured.position));
  EXPECT_FALSE(joined->acceleration_valid);EXPECT_TRUE(f.curve.getControlPoint().isApprox(controls));
  EXPECT_FALSE(measuredCandidateJoin(f.curve,f.original,f.measured,raw,.08,.5,.6,true,f.budget,.15));
}

TEST(CandidateAdoption, ExpandedReferenceDomainStillRequiresOriginalXyzC1AndSourceLease) {
  Fixture f;f.curve=line(.4);f.velocity={.4,0.,0.};f.measured.position={.12,0.,.55};
  const auto join=[&](const Eigen::Vector3d& raw) {
    return measuredCandidateJoin(f.curve,f.original,f.measured,raw,.08,.5,.6,true,f.budget,.5);
  };
  EXPECT_TRUE(join({.4,0.,.049}));EXPECT_FALSE(join({.4,0.,.050001}));
  EXPECT_FALSE(join({.500001,0.,0.}));
  f.measured.source_stamp=10.400001;EXPECT_FALSE(join(f.velocity));
}

TEST(CandidateAdoption, IndependentTravelDoesNotReplaceWholeCurveCollisionOrFinalSourceGate) {
  Fixture f;f.curve=line(.4);f.velocity={.4,0.,0.};f.measured.position={.12,0.,.55};
  bool clear=false;
  const auto certify=[&] {
    return certifyCandidateAdoption(f.curve,f.original,f.measured,f.velocity,.08,.5,.6,true,
      f.candidate_context,f.budget,[&] {
        return CandidateSourceLease{f.map->latestCloudStampNs(),f.map->localizationContextSequence(),
          f.map->occupancyRevision(),f.measured.source_stamp};
      },[]{return std::make_pair(true,true);},[&](double){++f.checks;f.during_check();return clear;},.5);
  };
  EXPECT_FALSE(certify());EXPECT_EQ(f.checks,1);
  clear=true;f.during_check=[&]{GridMapTestAccess::context(*f.map);};
  EXPECT_FALSE(certify());EXPECT_EQ(f.checks,2);
}

TEST(CandidateAdoption, NanosecondBodyChangeInvalidatesSourceLeaseAcrossFullCheck) {
  constexpr std::int64_t ns=1791124691902856036LL;
  Fixture f;
  CandidateSourceLease before{10300000000LL,7,2,static_cast<double>(ns)*1e-9,ns};
  auto after=before;EXPECT_TRUE(candidateLeaseStillCurrent(before,after,true,true,f.budget));
  ++after.body_source_ns;
  EXPECT_FALSE(candidateLeaseStillCurrent(before,after,true,true,f.budget));
  EXPECT_EQ(before.body_source,after.body_source);
}

TEST(CandidateAdoption, StationaryInputCannotAdvanceWithClock) {
  Fixture f;f.curve=line(0.);f.velocity.setZero();f.measured.position=f.original.position;
  for(int i=0;i<10;++i) {
    const auto result=f.certify();ASSERT_TRUE(result);EXPECT_EQ(result->curve_time,0.);
    EXPECT_EQ(result->arc_length,0.);f.now+=.001;
  }
}

TEST(CandidateAdoption, WrongFloorFarLateralAndWrongVelocityCannotJoin) {
  for(int mode=0;mode<4;++mode) {
    Fixture f;
    if(mode==0)f.measured.position.z()+=3.;
    if(mode==1)f.measured.position.x()+=1.;
    if(mode==2)f.measured.position.y()+=.021;
    if(mode==3)f.velocity.x()=-.3;
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,0);
  }
}

TEST(CandidateAdoption, EntryCannotSkipSemanticSegmentBoundaryOrContext) {
  Fixture f;f.single_segment=false;EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
  f.single_segment=true;f.candidate_context=6;EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
}

TEST(CandidateAdoption, ArcWindowRejectsLaterOverlappingBranchDespiteNearbyXYZ) {
  Fixture f;
  // A geometrically near branch is not legal progress. The robot only reports
  // 1 ms new source time, but the first compatible point is 9 cm along curve.
  f.measured.source_stamp=10.001;
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
}

TEST(CandidateAdoption, RegressedOrOverBudgetSourceCannotLicenseProjection) {
  for(double stamp:{9.99,10.401}) {Fixture f;f.measured.source_stamp=stamp;
    EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);}
}

TEST(CandidateAdoption, RealNativeOccupiedAndUnknownRemainderStillBlock) {
  for(int value:{1,2}) {Fixture f;GridMapTestAccess::raw(*f.map,{.65,0.,.55},value);
    EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);}
}

TEST(CandidateAdoption, CollisionBehindEntryIsStillCheckedConservatively) {
  Fixture f;GridMapTestAccess::raw(*f.map,{-.13,0.,.55},1);
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);
}

TEST(CandidateAdoption, FullCurveVelocityAndAccelerationBoundsAreNotRelaxed) {
  Fixture f;f.curve=line(.31);f.velocity.x()=.31;EXPECT_FALSE(f.certify());
  f.curve=line();auto controls=f.curve.getControlPoint();controls(1,9)+=.5;
  f.curve=UniformBspline(controls,3,.2);EXPECT_FALSE(f.certify());
}

TEST(CandidateAdoption, FreshAtStartButExpiredMapAtEndCannotCommit) {
  Fixture f;GridMapTestAccess::source(*f.map,9850000000LL); // age .46 s initially
  f.during_check=[&]{f.now+=.10;};
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1); // Same clear geometry, crossed TTL.
}

TEST(CandidateAdoption, FreshAtStartButExpiredBodyAtEndCannotCommit) {
  Fixture f;f.body_max_age=.05;f.during_check=[&]{f.now+=.10;};
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);
}

TEST(CandidateAdoption, SourceContextRevisionAndBodyReplacementDuringCheckCannotCommit) {
  for(int mode=0;mode<4;++mode) {
    Fixture f;f.during_check=[&]{
      if(mode==0)GridMapTestAccess::source(*f.map,10301000000LL);
      if(mode==1)GridMapTestAccess::context(*f.map);
      if(mode==2)GridMapTestAccess::revision(*f.map);
      if(mode==3)f.measured.source_stamp+=.001;
    };
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,1);
  }
}

TEST(CandidateAdoption, CanceledOrFourHundredMillisecondLateCheckCannotCommit) {
  for(int mode=0;mode<2;++mode) {
    Fixture f;f.during_check=[&]{if(mode==0)f.budget->cancel();
      else f.steady+=std::chrono::milliseconds(400);};
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,1);
  }
}

TEST(WorkerLifecycle, ActualSnapshotLeaseReleasedOnNormalAndExceptionalWorkerExit) {
  for(int mode=0;mode<3;++mode) {
    Fixture f;CollisionSnapshotPool pool;
    ASSERT_TRUE(pool.publish(*f.map,10310000000LL,SolveBudget::Clock::now()));
    auto snapshot=pool.borrowLatest();ASSERT_TRUE(snapshot);
    auto future=std::async(std::launch::async,[&,borrowed=snapshot]() mutable {
      SolveScopeExit release([&]() noexcept {borrowed.reset();});
      if(mode==1)throw std::runtime_error("optimizer");
      if(mode==2)throw std::bad_alloc{};
    });
    snapshot.reset();
    try {future.get();} catch(const std::exception &) {}
    EXPECT_TRUE(pool.borrowLatest()); // Real slot cannot re-lease while still held.
  }
}
