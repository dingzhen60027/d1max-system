#include <gtest/gtest.h>
#include <limits>
#include "common_lib.h"
#include "use-ikfom.hpp"
#include "ivox3d/ivox3d.h"
#include "registration_quality.hpp"

using faster_lio::state_ikfom;
using faster_lio::input_ikfom;

TEST(RegistrationQuality, FiniteWeakGeometryCanRecoverAfterMoreThanHalfASecond) {
    using D=faster_lio::LocalizationRegistrationDisposition;
    state_ikfom state;
    esekfom::esekf<state_ikfom,12,input_ikfom>::cov covariance=
        esekfom::esekf<state_ikfom,12,input_ikfom>::cov::Identity();
    constexpr double last_good=10.;
    bool old_elapsed_latch=false;
    for(double source:{10.1,10.3,10.6,11.,12.}) {
        // Same production state/covariance and 39/40 original feature gate.
        // The old Run branch became permanently fatal after source10.6;
        // source elapsed is not numeric corruption or a new local epoch.
        old_elapsed_latch=old_elapsed_latch||(source-last_good>.5);
        EXPECT_EQ(faster_lio::AssessLocalizationRegistration(39,40,state,covariance),D::WeakGeometry);
    }
    ASSERT_TRUE(old_elapsed_latch);
    EXPECT_EQ(faster_lio::AssessLocalizationRegistration(40,40,state,covariance),D::Accepted);
    EXPECT_EQ(faster_lio::AssessLocalizationRegistration(80,40,state,covariance),D::Accepted);
    EXPECT_EQ(faster_lio::AssessLocalizationRegistration(0,40,state,covariance),D::WeakGeometry);
    // The policy does not lower the gate or fabricate a successful posterior.
    EXPECT_EQ(faster_lio::AssessLocalizationRegistration(39,40,state,covariance),D::WeakGeometry);
}

TEST(RegistrationQuality, NumericCorruptionIsHardEvenWhenFeaturesAreWeakOrHealthy) {
    using D=faster_lio::LocalizationRegistrationDisposition;
    using Covariance=esekfom::esekf<state_ikfom,12,input_ikfom>::cov;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(int variant=0;variant<8;++variant) {
        for(int features:{0,39,40,80}) {
            state_ikfom state;auto covariance=Covariance::Identity().eval();
            if(variant==0)state.pos.x()=nan;
            if(variant==1)state.vel.y()=nan;
            if(variant==2)state.rot.coeffs()[0]=nan;
            if(variant==3)covariance(0,0)=nan;
            if(variant==4)state.bg.x()=nan;
            if(variant==5)state.ba.y()=nan;
            if(variant==6)state.offset_T_L_I.z()=nan;
            if(variant==7)state.offset_R_L_I.coeffs()[0]=nan;
            EXPECT_FALSE(faster_lio::FiniteLocalizationFilterState(state,covariance));
            EXPECT_EQ(faster_lio::AssessLocalizationRegistration(features,40,state,covariance),D::NonfiniteFilter);
        }
    }
}

TEST(RegistrationQuality, RejectedDistanceClearsStaleResidual) {
    float residual = .07F;
    EXPECT_FALSE(faster_lio::SelectSurfaceResidual(1.F, .3F, residual));
    EXPECT_FLOAT_EQ(residual, 0.F);
    EXPECT_TRUE(faster_lio::SelectSurfaceResidual(1.F, .05F, residual));
    EXPECT_FLOAT_EQ(residual, .05F);
    EXPECT_FALSE(faster_lio::SelectSurfaceResidual(1.F, std::numeric_limits<float>::quiet_NaN(), residual));
    EXPECT_FLOAT_EQ(residual, 0.F);
}

TEST(RegistrationQuality, EmptyVoxelQueryCannotReusePreviousNeighbours) {
    using Grid = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
    Grid grid(Grid::Options{});
    PointType p{};
    p.x = 1.F; p.y = 2.F; p.z = 3.F;
    Grid::PointVector points{p};
    grid.AddPoints(points);
    Grid::PointVector neighbours;
    ASSERT_TRUE(grid.GetClosestPoint(p, neighbours));
    ASSERT_FALSE(neighbours.empty());
    p.x = 100.F;
    EXPECT_FALSE(grid.GetClosestPoint(p, neighbours));
    EXPECT_TRUE(neighbours.empty());
}
