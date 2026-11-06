#include <gtest/gtest.h>
#include <bspline_opt/lbfgs.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace {
struct Interval {
  double x{0.},fx{4.5020031126203639},dx{-4127.072315350405};
  double y{.015566069337286735},fy{1058.0659222380266},dy{1.};
  double trial{2.4909338492717117e-5},ft{4.4005022831377421},dt{-4022.9387213378172};
  double lower{0.},upper{.015566069337286735};
  int bracket{1};
  int update(bool corrected) {
    return lbfgs::update_trial_interval(&x,&fx,&dx,&y,&fy,&dy,&trial,&ft,&dt,
        lower,upper,&bracket,corrected);
  }
  int updateDefault() {
    return lbfgs::update_trial_interval(&x,&fx,&dx,&y,&fy,&dy,&trial,&ft,&dt,
        lower,upper,&bracket);
  }
  bool inside()const {return trial>std::min(x,y)&&trial<std::max(x,y);}
};
double quadratic(void*,const double* x,double* g,int) {
  g[0]=x[0]-1.;g[1]=4.*(x[1]+2.);
  return .5*(x[0]-1.)*(x[0]-1.)+2.*(x[1]+2.)*(x[1]+2.);
}
}

// Captured zero-terminal-velocity stair seed: lower cost, same derivative
// sign, smaller derivative magnitude. The cubic discriminant is negative,
// gamma=0. Its fallback bound must follow d=v-u, not overwritten theta/s.
TEST(LbfgsCubicBound,CapturedStairCaseChoosesInteriorForwardTrial) {
  Interval interval;
  ASSERT_EQ(interval.update(true),0);
  EXPECT_DOUBLE_EQ(interval.x,2.4909338492717117e-5);
  EXPECT_NEAR(interval.trial,.0009872187958341102,1e-16);
  EXPECT_TRUE(interval.inside());
}

TEST(LbfgsCubicBound,CoordinateReflectionChoosesInteriorBackwardTrial) {
  Interval interval;
  interval.y=-interval.y;interval.dy=-interval.dy;interval.trial=-interval.trial;
  interval.dx=-interval.dx;interval.dt=-interval.dt;
  interval.lower=-interval.upper;interval.upper=0.;
  ASSERT_EQ(interval.update(true),0);
  EXPECT_NEAR(interval.trial,-.0009872187958341102,1e-16);
  EXPECT_TRUE(interval.inside());
}

TEST(LbfgsCubicBound,DefaultAndExplicitLegacyKeepHistoricalResult) {
  Interval implicit,explicit_legacy;
  ASSERT_EQ(implicit.updateDefault(),0);
  ASSERT_EQ(explicit_legacy.update(false),0);
  EXPECT_DOUBLE_EQ(implicit.trial,0.);
  EXPECT_DOUBLE_EQ(implicit.trial,explicit_legacy.trial);
  EXPECT_DOUBLE_EQ(implicit.x,explicit_legacy.x);
  EXPECT_DOUBLE_EQ(implicit.y,explicit_legacy.y);
  // Preserve this exact flat/legacy behavior until separately authorized;
  // recording its defect is not permission to enable the stair change there.
  EXPECT_FALSE(implicit.inside());
}

TEST(LbfgsCubicBound,OrdinaryQuadraticInterpolationIsUnchanged) {
  Interval legacy;
  legacy.fx=.49;legacy.dx=-1.4;legacy.y=1.;legacy.fy=.09;legacy.dy=.6;
  legacy.trial=.1;legacy.ft=.36;legacy.dt=-1.2;legacy.upper=1.;
  Interval corrected=legacy;
  ASSERT_EQ(legacy.update(false),0);
  ASSERT_EQ(corrected.update(true),0);
  EXPECT_DOUBLE_EQ(corrected.trial,legacy.trial);
  EXPECT_NEAR(corrected.trial,.694,1e-14);
  EXPECT_TRUE(corrected.inside());
}

TEST(LbfgsCubicBound,InvalidBracketStillRejected) {
  Interval interval;interval.trial=interval.y;
  EXPECT_EQ(interval.update(true),lbfgs::LBFGSERR_OUTOFINTERVAL);
}

TEST(LbfgsCubicBound,DefaultParameterDoesNotEnableCorrection) {
  lbfgs::lbfgs_parameter_t parameters;
  lbfgs::lbfgs_load_default_parameters(&parameters);
  EXPECT_FALSE(parameters.corrected_cubic_bound);
}

TEST(LbfgsCubicBound,CompleteQuadraticSolveIsUnchanged) {
  std::array<std::array<double,2>,2> values{{{{7.,-4.}},{{7.,-4.}}}};
  for(int corrected=0;corrected<2;++corrected) {
    lbfgs::lbfgs_parameter_t parameters;
    lbfgs::lbfgs_load_default_parameters(&parameters);
    parameters.corrected_cubic_bound=corrected;
    parameters.max_iterations=200;
    double cost;
    const int result=lbfgs::lbfgs_optimize(2,values[corrected].data(),&cost,
        quadratic,nullptr,nullptr,nullptr,&parameters);
    EXPECT_EQ(result,lbfgs::LBFGS_CONVERGENCE);
    EXPECT_NEAR(values[corrected][0],1.,1e-4);
    EXPECT_NEAR(values[corrected][1],-2.,1e-4);
    EXPECT_LT(cost,1e-8);
  }
  EXPECT_DOUBLE_EQ(values[0][0],values[1][0]);
  EXPECT_DOUBLE_EQ(values[0][1],values[1][1]);
}
