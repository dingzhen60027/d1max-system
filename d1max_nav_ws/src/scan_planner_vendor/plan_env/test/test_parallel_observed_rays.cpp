#include <gtest/gtest.h>
#include <plan_env/parallel_observed_rays.hpp>
#include <atomic>
#include <random>
#include <system_error>

namespace scan_planner {
struct ParallelObservedRayVisitsTestAccess {
  static void constructWithFailure(unsigned fail_at,unsigned& started,std::atomic<unsigned>& completed) {
    unsigned attempted=0;
    ParallelObservedRayVisits work([&](auto callback) {
      if(attempted++==fail_at) {
        throw std::system_error(std::make_error_code(std::errc::resource_unavailable_try_again));
      }
      auto worker=std::thread([callback=std::move(callback),&completed]() mutable {callback();++completed;});
      ++started;
      return worker;
    });
  }
  static auto constructTracked(unsigned& started,std::atomic<unsigned>& completed) {
    return std::unique_ptr<ParallelObservedRayVisits>(new ParallelObservedRayVisits([&](auto callback) {
      auto worker=std::thread([callback=std::move(callback),&completed]() mutable {callback();++completed;});
      ++started;
      return worker;
    }));
  }
};
}

namespace {
using Traversal=scan_planner::ParallelObservedRayVisits;
using Event=std::array<int,5>;
void expectOrderedEquality(Traversal& work,const Eigen::Vector3i& minimum,
                           const Eigen::Vector3i& size,double resolution) {
  ASSERT_TRUE(work.run(minimum,size,resolution,Traversal::kMaxSteps));
  std::vector<unsigned char> serial_bits(size.prod(),0),replayed_bits(size.prod(),0);
  std::vector<Event> serial,replayed;
  std::size_t budget=Traversal::kMaxSteps;
  for(std::size_t i=0;i<work.rays().size();++i) {
    const auto& ray=work.rays()[i];
    ASSERT_TRUE(scan_planner::visitObservedRayIndexed(ray.origin,ray.end,resolution,ray.include_end,
        budget,size,[&](const Eigen::Vector3i& cell,int address) {
          auto& bits=serial_bits.at(address);
          if(bits&ray.source_mask) {
            return;
          }
          bits|=ray.source_mask;
          serial.push_back({static_cast<int>(i),cell.x(),cell.y(),cell.z(),address});
        }));
    work.replay(i,[&](const Eigen::Vector3i& cell,int address) {
      auto& bits=replayed_bits.at(address);
      if(bits&ray.source_mask) {
        return;
      }
      bits|=ray.source_mask;
      replayed.push_back({static_cast<int>(i),cell.x(),cell.y(),cell.z(),address});
    });
  }
  EXPECT_EQ(work.steps(),Traversal::kMaxSteps-budget);
  EXPECT_EQ(serial,replayed);
  EXPECT_EQ(serial_bits,replayed_bits);
}
}

TEST(ParallelObservedRays, ExactFirstSourceVisitOrderAcrossNegativeRingWindowsAndReuse) {
  Traversal work;
  std::mt19937 random(1104);
  std::uniform_real_distribution<double> unit(.001,.999);
  for(const auto& minimum:{Eigen::Vector3i(-257,241,-43),Eigen::Vector3i(317,-416,79),Eigen::Vector3i(-2,-2,-2)}) {
    const Eigen::Vector3i size(40,39,30);
    work.rays().clear();
    for(int i=0;i<4096;++i) {
      Eigen::Vector3d origin,end;
      for(int axis=0;axis<3;++axis) {
        origin[axis]=(minimum[axis]+unit(random)*size[axis])*.1;
        end[axis]=(minimum[axis]+unit(random)*size[axis])*.1;
      }
      work.rays().push_back({origin,end,static_cast<unsigned char>(i%3?1:2),i%7==0});
    }
    expectOrderedEquality(work,minimum,size,.1);
  }
}

TEST(ParallelObservedRays, FacesCornersNegativeDirectionZeroLengthAndDuplicateRays) {
  Traversal work;
  for(int i=0;i<32;++i) {
    for(unsigned char source:{1,2}) {
      for(bool include:{false,true}) {
        work.rays().push_back({{.5,.5,.5},{-.5,-.5,-.5},source,include});
        work.rays().push_back({{-.5,0.,.5},{.5,-.5,0.},source,include});
        work.rays().push_back({{0.,0.,0.},{0.,0.,0.},source,include});
        work.rays().push_back({{-.1,.2,.3},{-.10000000000001,.7,-.6},source,include});
      }
    }
  }
  expectOrderedEquality(work,{-10,-10,-10},{20,20,20},.1);
}

TEST(ParallelObservedRays, SourceMaskIsPerRayAndReloadedAcrossRepeatedRuns) {
  Traversal work;
  // Identical geometry still contributes once from EACH source. Alternate
  // source inside each partition, not only at lane boundaries: caching a
  // lane-wide or previous ray's source bit would lose/reorder evidence.
  for(int i=0;i<129;++i) {
    const Eigen::Vector3d origin(.05,.05,.05);
    const Eigen::Vector3d end(i%3==0?-.85:.85,.35,-.25);
    work.rays().push_back({origin,end,static_cast<unsigned char>(i%2?1:2),i%5==0});
  }
  expectOrderedEquality(work,{-10,-10,-10},{20,20,20},.1);
  // Between completed jobs the caller may replace every source. Nothing from
  // the previous capture may survive, including per-lane first-source bits.
  for(auto& ray:work.rays())ray.source_mask=static_cast<unsigned char>(3-ray.source_mask);
  expectOrderedEquality(work,{-10,-10,-10},{20,20,20},.1);
  // Rejected preflight cannot expose stale offsets, even after a good job.
  work.rays()[64].source_mask=0;
  EXPECT_FALSE(work.run({-10,-10,-10},{20,20,20},.1,Traversal::kMaxSteps));
  EXPECT_THROW(work.replay(0,[](const Eigen::Vector3i&,int){}),std::logic_error);
  work.rays()[64].source_mask=2;
  expectOrderedEquality(work,{-10,-10,-10},{20,20,20},.1);
}

TEST(ParallelObservedRays, TinyBudgetsInvalidInputsAndRejectedJobCannotReplayStaleEvents) {
  Traversal work;
  work.rays()={{{0.,0.,0.},{0.,0.,0.},1,true},{{0.,0.,0.},{0.,0.,0.},2,true}};
  for(std::size_t budget:{0,1,2,3}) {
    EXPECT_FALSE(work.run({-10,-10,-10},{20,20,20},.1,budget));
    EXPECT_THROW(work.replay(0,[](const Eigen::Vector3i&,int){}),std::logic_error);
  }
  ASSERT_TRUE(work.run({-10,-10,-10},{20,20,20},.1,4));
  EXPECT_EQ(work.steps(),2U);
  work.rays()[0].end.x()=100.;
  EXPECT_FALSE(work.run({-10,-10,-10},{20,20,20},.1,16000000));
  EXPECT_THROW(work.replay(0,[](const Eigen::Vector3i&,int){}),std::logic_error);
  work.rays()[0].end.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(work.run({-10,-10,-10},{20,20,20},.1,16000000));
  work.rays()[0].end.x()=0.;
  work.rays()[0].source_mask=3;
  EXPECT_FALSE(work.run({-10,-10,-10},{20,20,20},.1,16000000));
  work.rays()[0].source_mask=1;
  expectOrderedEquality(work,{-10,-10,-10},{20,20,20},.1);
}

TEST(ParallelObservedRays, DestructionJoinsBothWorkersAfterRepeatedSuccessfulAndRejectedJobs) {
  for(int i=0;i<24;++i) {
    unsigned started=0;std::atomic<unsigned> completed{0};
    {
      auto work=scan_planner::ParallelObservedRayVisitsTestAccess::constructTracked(started,completed);
      ASSERT_EQ(started,2U);EXPECT_EQ(completed.load(),0U);
      work->rays()={{{0.,0.,0.},{.8,.3,-.4},1,true},{{0.,0.,0.},{-.8,-.3,.4},2,false}};
      ASSERT_TRUE(work->run({-10,-10,-10},{20,20,20},.1,16000000));
      ASSERT_FALSE(work->run({-10,-10,-10},{20,20,20},.1,1));
    }
    // Assert completion synchronized by the real production join(), not the
    // asynchronous removal of exiting Linux tasks from /proc/self/task.
    EXPECT_EQ(completed.load(),started);
  }
}

TEST(ParallelObservedRays, FirstOrSecondHelperConstructionFailureRetiresCreatedThread) {
  for(unsigned i=0;i<24;++i) {
    unsigned started=0;std::atomic<unsigned> completed{0};
    EXPECT_THROW(scan_planner::ParallelObservedRayVisitsTestAccess::constructWithFailure(i%2,started,completed),
                 std::system_error);
    EXPECT_EQ(started,i%2);
    EXPECT_EQ(completed.load(),started);
  }
}
