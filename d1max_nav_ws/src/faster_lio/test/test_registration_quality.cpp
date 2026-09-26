#include <gtest/gtest.h>
#include <limits>
#include "common_lib.h"
#include "ivox3d/ivox3d.h"
#include "registration_quality.hpp"

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
