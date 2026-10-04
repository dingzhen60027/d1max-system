// Pure header tests: no ROS context, graph, services, SDK or robot connection.
#include <gtest/gtest.h>
#include <plan_env/near_field_diagnostics.hpp>
#include <plan_env/voxel_collision.hpp>
#include <cmath>
#include <limits>

namespace {
using Cell=Eigen::Vector3i;
using Point=Eigen::Vector3d;
using scan_planner::NearFieldDiagnostics;
using scan_planner::RayWitness;
using scan_planner::RawVoxelDiagnostic;

RayWitness witness(unsigned sensor,bool hit,std::int64_t stamp=1000,bool vote=true) {
  RayWitness result;
  result.metadata.sensor_id=sensor;
  result.metadata.ring=42+sensor;
  result.metadata.source_index=701+sensor;
  result.metadata.offset_time_ns=123+sensor;
  result.metadata.scan_stamp_ns=stamp;
  result.metadata.received_ns=stamp+10;
  result.metadata.integration_ns=stamp+20;
  result.metadata.context_sequence=8;
  result.metadata.projection_sequence=9+sensor;
  result.metadata.timestamp=stamp*1e-9;
  result.metadata.source_timestamp=stamp*1e-9-.5;
  result.metadata.raw_timestamp=stamp*1e-9-.7;
  result.metadata.source_fields_available=true;
  result.origin={double(sensor),.1,.2};
  result.endpoint={.3,.4,.5};
  result.integrated_endpoint={.25,.4,.5};
  result.hit=hit;
  result.contributed_vote=vote;
  return result;
}

void configure(NearFieldDiagnostics &diagnostic,std::size_t cells=16,std::size_t events=32) {
  diagnostic.configure(true,Point::Zero(),Point::Ones(),.1,cells,events);
}
} // namespace

TEST(NearFieldDiagnostics, DefaultDisabledRetainsNoWitnesses) {
  NearFieldDiagnostics diagnostic;
  EXPECT_FALSE(diagnostic.enabled());
  diagnostic.record(Cell::Zero(),witness(0,true));
  EXPECT_EQ(diagnostic.size(),0U);
  EXPECT_EQ(diagnostic.find(Cell::Zero()),nullptr);
  EXPECT_FALSE(diagnostic.contains(Cell::Zero()));
}

TEST(NearFieldDiagnostics, DisablingClearsEvidenceAndDoesNotChangeNativeClassification) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  diagnostic.record(Cell::Zero(),witness(0,true));
  ASSERT_EQ(diagnostic.size(),1U);
  const int before=scan_planner::strictRawVoxelStatus(.5,-2.,1.);
  diagnostic.configure(false,Point::Zero(),Point::Ones(),.1);
  EXPECT_FALSE(diagnostic.enabled());
  EXPECT_EQ(diagnostic.find(Cell::Zero()),nullptr);
  diagnostic.record(Cell::Zero(),witness(1,false));
  EXPECT_EQ(diagnostic.size(),0U);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(.5,-2.,1.),before);
}

TEST(NearFieldDiagnostics, DistinctGlobalIndicesDoNotAliasLikeRingBufferAddresses) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  const Cell a(-2,0,0), b(2,0,0);
  diagnostic.record(a,witness(0,true,1000));
  diagnostic.record(b,witness(1,false,2000));
  ASSERT_EQ(diagnostic.size(),2U);
  ASSERT_NE(diagnostic.find(a),nullptr);
  ASSERT_NE(diagnostic.find(b),nullptr);
  EXPECT_TRUE(diagnostic.find(a)->last[0].has_value());
  EXPECT_FALSE(diagnostic.find(a)->last[3].has_value());
  EXPECT_TRUE(diagnostic.find(b)->last[3].has_value());
  EXPECT_FALSE(diagnostic.find(b)->last[0].has_value());
}

TEST(NearFieldDiagnostics, RepeatedWorldIndexCountsVisitsAndVotesWithoutDuplicates) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  const Cell cell(-2,1,3);
  diagnostic.record(cell,witness(0,true,1000,true));
  diagnostic.record(cell,witness(0,true,2000,false));
  ASSERT_EQ(diagnostic.size(),1U);
  const auto *record=diagnostic.find(cell);
  ASSERT_NE(record,nullptr);
  EXPECT_EQ(record->visits[0],2U);
  EXPECT_EQ(record->votes[0],1U);
  ASSERT_TRUE(record->last[0].has_value());
  EXPECT_EQ(record->last[0]->metadata.scan_stamp_ns,2000);
  EXPECT_FALSE(record->last[0]->contributed_vote);
}

TEST(NearFieldDiagnostics, BothSensorsAndHitMissKeepIndependentOriginalMetadata) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  for(unsigned sensor=0;sensor<2;++sensor) for(bool hit:{true,false})
    diagnostic.record(Cell::Zero(),witness(sensor,hit,1000+100*sensor+!hit,hit));
  const auto *record=diagnostic.find(Cell::Zero());
  ASSERT_NE(record,nullptr);
  for(unsigned sensor=0;sensor<2;++sensor) for(bool hit:{true,false}) {
    const auto slot=2*sensor+(hit?0:1);
    ASSERT_TRUE(record->last[slot].has_value());
    const auto &value=*record->last[slot];
    EXPECT_EQ(value.metadata.sensor_id,sensor);
    EXPECT_EQ(value.metadata.ring,42+sensor);
    EXPECT_EQ(value.metadata.source_index,701+sensor);
    EXPECT_EQ(value.metadata.offset_time_ns,123+sensor);
    EXPECT_EQ(value.metadata.scan_stamp_ns,1000+100*sensor+!hit);
    EXPECT_EQ(value.metadata.received_ns,value.metadata.scan_stamp_ns+10);
    EXPECT_EQ(value.metadata.integration_ns,value.metadata.scan_stamp_ns+20);
    EXPECT_EQ(value.metadata.context_sequence,8U);
    EXPECT_EQ(value.metadata.projection_sequence,9+sensor);
    EXPECT_TRUE(value.metadata.source_fields_available);
    EXPECT_DOUBLE_EQ(value.metadata.source_timestamp,value.metadata.timestamp-.5);
    EXPECT_DOUBLE_EQ(value.metadata.raw_timestamp,value.metadata.timestamp-.7);
    EXPECT_TRUE(value.origin.isApprox(Point(double(sensor),.1,.2)));
    EXPECT_TRUE(value.endpoint.isApprox(Point(.3,.4,.5)));
    EXPECT_TRUE(value.integrated_endpoint.isApprox(Point(.25,.4,.5)));
    EXPECT_EQ(record->visits[slot],1U);
    EXPECT_EQ(record->votes[slot],hit?1U:0U);
  }
}

TEST(NearFieldDiagnostics, ExplicitMissingSourceMetadataStaysMissing) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  RayWitness value;
  value.hit=true;
  diagnostic.record(Cell::Zero(),value);
  ASSERT_TRUE(diagnostic.find(Cell::Zero())->last[0].has_value());
  const auto &record=diagnostic.find(Cell::Zero())->last[0]->metadata;
  EXPECT_FALSE(record.source_fields_available);
  EXPECT_EQ(record.scan_stamp_ns,0);
  EXPECT_DOUBLE_EQ(record.source_timestamp,0.);
}

TEST(NearFieldDiagnostics, BoundedCellStorageKeepsOldEntriesAndReportsDroppedCells) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic,2,32);
  diagnostic.record({0,0,0},witness(0,true));
  diagnostic.record({1,0,0},witness(0,true));
  diagnostic.record({2,0,0},witness(0,true));
  EXPECT_EQ(diagnostic.size(),2U);
  EXPECT_EQ(diagnostic.capacity(),2U);
  EXPECT_EQ(diagnostic.droppedCells(),1U);
  EXPECT_EQ(diagnostic.find({2,0,0}),nullptr);
  diagnostic.record({0,0,0},witness(1,false));
  EXPECT_EQ(diagnostic.size(),2U);
  ASSERT_NE(diagnostic.find({0,0,0}),nullptr);
  EXPECT_TRUE(diagnostic.find({0,0,0})->last[3].has_value());
}

TEST(NearFieldDiagnostics, EventBudgetResetsPerIntegrationButHistoryDoesNotRefreshItself) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic,16,2);
  diagnostic.record({0,0,0},witness(0,true,1000));
  diagnostic.record({1,0,0},witness(0,true,2000));
  diagnostic.record({2,0,0},witness(0,true,3000));
  EXPECT_EQ(diagnostic.droppedEvents(),1U);
  EXPECT_EQ(diagnostic.eventLimit(),2U);
  EXPECT_EQ(diagnostic.find({2,0,0}),nullptr);
  diagnostic.beginIntegration();
  diagnostic.record({2,0,0},witness(1,false,4000));
  ASSERT_NE(diagnostic.find({2,0,0}),nullptr);
  EXPECT_EQ(diagnostic.find({0,0,0})->last[0]->metadata.scan_stamp_ns,1000);
  EXPECT_EQ(diagnostic.droppedEvents(),1U);
}

TEST(NearFieldDiagnostics, RoiUsesSignedWorldIndicesAndRejectsOutsideEvidence) {
  NearFieldDiagnostics diagnostic;
  diagnostic.configure(true,Point(-.05,.05,.05),Point(.2,.2,.2),.1,32,32);
  EXPECT_TRUE(diagnostic.contains({-3,-2,-2}));
  EXPECT_TRUE(diagnostic.contains({1,2,2}));
  EXPECT_FALSE(diagnostic.contains({-4,0,0}));
  EXPECT_FALSE(diagnostic.contains({2,0,0}));
  diagnostic.record({-4,0,0},witness(0,true));
  diagnostic.record({2,0,0},witness(1,false));
  EXPECT_EQ(diagnostic.size(),0U);
  diagnostic.record({-3,-2,-2},witness(0,true));
  diagnostic.record({1,2,2},witness(1,false));
  EXPECT_EQ(diagnostic.size(),2U);
}

TEST(NearFieldDiagnostics, SlidingErasePreventsWitnessReuseForNewWorldCell) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  const Cell old_cell(-2,0,0), replacement(2,0,0);
  diagnostic.record(old_cell,witness(0,true,1000));
  diagnostic.erase(old_cell);
  EXPECT_EQ(diagnostic.find(old_cell),nullptr);
  EXPECT_EQ(diagnostic.find(replacement),nullptr);
  diagnostic.record(replacement,witness(1,false,2000));
  EXPECT_EQ(diagnostic.find(old_cell),nullptr);
  ASSERT_NE(diagnostic.find(replacement),nullptr);
  EXPECT_FALSE(diagnostic.find(replacement)->last[0].has_value());
  EXPECT_EQ(diagnostic.find(replacement)->last[3]->metadata.scan_stamp_ns,2000);
}

TEST(NearFieldDiagnostics, ResetAndReconfigureClearWitnessesAndAdvanceGeneration) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic,1,1);
  auto generation=diagnostic.generation();
  diagnostic.record(Cell::Zero(),witness(0,true));
  diagnostic.record(Cell::Zero(),witness(1,false));
  EXPECT_EQ(diagnostic.droppedEvents(),1U);
  diagnostic.clear();
  EXPECT_GT(diagnostic.generation(),generation);
  EXPECT_EQ(diagnostic.size(),0U);
  EXPECT_EQ(diagnostic.droppedEvents(),0U);
  EXPECT_EQ(diagnostic.droppedCells(),0U);
  generation=diagnostic.generation();
  diagnostic.record(Cell::Zero(),witness(0,true));
  diagnostic.configure(true,Point(2.,0.,0.),Point::Ones(),.1);
  EXPECT_GT(diagnostic.generation(),generation);
  EXPECT_EQ(diagnostic.find(Cell::Zero()),nullptr);
  EXPECT_FALSE(diagnostic.contains(Cell::Zero()));
}

TEST(NearFieldDiagnostics, InvalidOrUnboundedConfigurationRejected) {
  NearFieldDiagnostics diagnostic;
  const Point center=Point::Zero(), half=Point::Ones();
  EXPECT_THROW(diagnostic.configure(true,center,half,.1,0,100),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,half,.1,32769,100),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,half,.1,10,0),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,half,.1,10,1000001),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,half,0.),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,Point::Constant(5.01),.1),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,center,Point::Zero(),.1),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,Point(INFINITY,0.,0.),half,.1),std::invalid_argument);
}

TEST(NearFieldDiagnostics, FiniteCoordinatesAndTinyResolutionCannotOverflowWorldIndex) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  diagnostic.record(Cell::Zero(),witness(0,true));
  const auto generation=diagnostic.generation();
  EXPECT_THROW(diagnostic.configure(true,Point(1e20,0.,0.),Point::Ones(),.1),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,Point(-1e20,0.,0.),Point::Ones(),.1),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,Point::Zero(),Point::Ones(),1e-20),std::invalid_argument);
  EXPECT_THROW(diagnostic.configure(true,Point::Zero(),Point::Ones(),
      std::numeric_limits<double>::denorm_min()),std::invalid_argument);
  // A rejected reconfiguration must not silently erase the current snapshot.
  EXPECT_EQ(diagnostic.generation(),generation);
  ASSERT_NE(diagnostic.find(Cell::Zero()),nullptr);
  EXPECT_TRUE(diagnostic.find(Cell::Zero())->last[0]);
}

TEST(RawVoxelDiagnostic, NeverObservedIsNotTheSameAsIntermediateHitEvidence) {
  constexpr double free=-2., occupied=1.4, unknown=.01;
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(free-unknown,free,occupied,unknown),RawVoxelDiagnostic::NeverObserved);
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(free,free,occupied,unknown),RawVoxelDiagnostic::Free);
  // One p_hit=.85 update from unknown is still strictly unknown, not unobserved.
  const double weak_hit=free-unknown+std::log(.85/.15);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(weak_hit,free,occupied),2);
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(weak_hit,free,occupied,unknown),RawVoxelDiagnostic::Insufficient);
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(occupied,free,occupied,unknown),RawVoxelDiagnostic::Insufficient);
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(occupied+.001,free,occupied,unknown),RawVoxelDiagnostic::Occupied);
}

TEST(RawVoxelDiagnostic, DiagnosisDoesNotChangeStrictBoundaryPolicy) {
  constexpr double free=-2., occupied=1.4, unknown=.01;
  for(double value:{free-unknown,free,free+5e-10,free+.01,-.5,0.,occupied,occupied+.001}) {
    const int before=scan_planner::strictRawVoxelStatus(value,free,occupied);
    const auto detail=scan_planner::diagnoseRawVoxel(value,free,occupied,unknown);
    EXPECT_EQ(scan_planner::strictRawVoxelStatus(value,free,occupied),before);
    if(detail==RawVoxelDiagnostic::Free) EXPECT_EQ(before,0);
    else if(detail==RawVoxelDiagnostic::Occupied) EXPECT_EQ(before,1);
    else EXPECT_EQ(before,2);
  }
}

TEST(RawVoxelDiagnostic, NonfiniteEvidenceIsInvalidAndRemainsBlocked) {
  for(double value:{std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_EQ(scan_planner::diagnoseRawVoxel(value,-2.,1.4,.01),RawVoxelDiagnostic::Invalid);
    EXPECT_EQ(scan_planner::strictRawVoxelStatus(value,-2.,1.4),2);
  }
}

TEST(RawVoxelDiagnostic, ConflictingWitnessesAreEvidenceNotProofOfFreeSpace) {
  NearFieldDiagnostics diagnostic;
  configure(diagnostic);
  diagnostic.record(Cell::Zero(),witness(0,true,1000));
  diagnostic.record(Cell::Zero(),witness(1,false,1100));
  ASSERT_TRUE(diagnostic.find(Cell::Zero())->last[0].has_value());
  ASSERT_TRUE(diagnostic.find(Cell::Zero())->last[3].has_value());
  // Log-odds alone cannot distinguish whether intermediate evidence is weak
  // or conflicting. Witnesses explain history but never authorize a voxel.
  EXPECT_EQ(scan_planner::diagnoseRawVoxel(.2,-2.,1.4,.01),RawVoxelDiagnostic::Insufficient);
  EXPECT_EQ(scan_planner::strictRawVoxelStatus(.2,-2.,1.4),2);
}
