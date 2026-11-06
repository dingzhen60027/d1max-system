#include <bspline_opt/soft_clearance.hpp>
#include <bspline_opt/planar_curvature_cost.hpp>
#include <bspline_opt/forward_entry_heading.hpp>
#include <bspline_opt/near_curve_direction.hpp>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace scan_planner {
namespace {
constexpr int kMaxSections=12,kMaxControls=96;
constexpr std::size_t kMaxQueries=768;
struct Section {
  double time{0.},left{0.},right{0.};
  Eigen::Vector2d normal{Eigen::Vector2d::Zero()};
  Eigen::Vector2d displacement{Eigen::Vector2d::Zero()};
};
double penalty(const std::vector<Section>& sections,double comfort) {
  double sum=0.;
  for(const auto& section:sections) {
    const double a=std::max(0.,comfort-section.left),b=std::max(0.,comfort-section.right);
    sum+=a*a+b*b;
  }
  return sum;
}
double minimumSideMargin(const std::vector<Section>& sections) {
  double result=std::numeric_limits<double>::infinity();
  for(const auto& section:sections)result=std::min(result,std::min(section.left,section.right));
  return result;
}
bool sectionsFor(UniformBspline& curve,int count,double resolution,double comfort,
    const SplineHeadingContract& heading,const SolveBudget::Ptr& budget,
    const std::function<bool()>& source_allowed,
    const SoftClearanceQuery& query,SoftClearanceReport& report,
    std::vector<Section>& sections,const SupportHeightCoupling* support_height) {
  const auto allowed=[&]{return solveAllowed(budget)&&source_allowed();};
  auto velocity=curve.getDerivative();const double duration=curve.getTimeSum();
  const double step=resolution*.5,range=2.*comfort;
  const int steps=std::max(1,static_cast<int>(std::ceil(range/step)));
  if(steps>32)return false;
  // Comfort must query the same curve-local heading as normal admission,
  // including a backwards acquired prefix and low-speed heading memory.
  // Derive only a local copy: never rewrite the caller's frozen contract.
  auto query_heading=heading;
  std::optional<std::uint32_t> heading_intervals;
  if(measuredHeadingEnabled(heading)) {
    double speed_bound=0.;const auto points=velocity.getControlPoint();
    for(int i=0;i<points.cols();++i)speed_bound=std::max(speed_bound,points.col(i).norm());
    heading_intervals=bodyHeadingSamplingIntervals(duration,speed_bound,resolution);
    if(!heading_intervals||!resolveForwardEntryHeading(query_heading,duration,*heading_intervals,
        [&](double t){return curve.evaluateDeBoorT(t);},
        [&](double t){return velocity.evaluateDeBoorT(t);},allowed))return false;
    if(query_heading.align_then_forward&&query_heading.policy==BodyHeadingPolicy::PathTangent&&
        velocity.evaluateDeBoorT(0.).head<2>().norm()<query_heading.minimum_direction_speed) {
      const auto direction=nearCurveDirection(
          [&](double t){const auto p=curve.evaluateDeBoorT(t);return CurveDirectionPoint{p.x(),p.y(),p.z()};},
          [&](double t){const auto v=velocity.evaluateDeBoorT(t);return CurveDirectionPoint{v.x(),v.y(),v.z()};},
          0.,duration,allowed);
      if(direction)query_heading.measured_yaw=std::atan2(direction->direction[1],direction->direction[0]);
    }
  }
  const auto checked_query=[&](const Eigen::Vector3d& p,double yaw)->std::optional<int> {
    if(!allowed()||report.queries>=kMaxQueries)return {};
    ++report.queries;return query(p,yaw);
  };
  sections.clear();sections.reserve(count);
  for(int i=0;i<count;++i) {
    if(!allowed())return false;
    Section section;section.time=duration*static_cast<double>(i+1)/(count+1);
    const Eigen::Vector3d p=curve.evaluateDeBoorT(section.time);
    const Eigen::Vector3d v=velocity.evaluateDeBoorT(section.time);
    if(!p.allFinite()||!v.allFinite()||v.head<2>().norm()<1e-5)return false;
    const Eigen::Vector2d tangent=v.head<2>().normalized();
    section.normal={-tangent.y(),tangent.x()};
    double yaw=std::atan2(tangent.y(),tangent.x());
    if(heading_intervals) {
      const auto sampled=sampledBodyHeadingAt(section.time,duration,*heading_intervals,query_heading,
          [&](double t){return velocity.evaluateDeBoorT(t);},allowed);
      if(!sampled)return false;
      yaw=*sampled;
    }
    const auto center=checked_query(p,yaw);
    if(!center||*center!=0)return false; // Quality cannot certify a blocked input.
    for(int side=0;side<2;++side) {
      double clear=0.;
      for(int n=1;n<=steps;++n) {
        const double offset=std::min(range,n*step);
        Eigen::Vector3d probe=p;probe.head<2>()+=(side==0?1.:-1.)*offset*section.normal;
        if(support_height) support_height->applyProbe(p,probe);
        const auto state=checked_query(probe,yaw);if(!state)return false;
        if(*state!=0)break;
        clear=offset;
      }
      if(side==0)section.left=clear;else section.right=clear;
    }
    // Saturate in open areas. Do not chase a distant mathematical midpoint
    // or respond to a one-grid imbalance once both sides are comfortable.
    const double difference=section.left-section.right;
    if(std::min(section.left,section.right)+1e-9<comfort &&
        std::abs(difference)>resolution+1e-9) {
      // One coarse-cell lateral imbalance can alternate with quantized sensor
      // endpoints. Do not steer toward it. Beyond that band the preference
      // grows continuously; this never changes obstacle/unknown decisions.
      const double shift=std::copysign(std::min(comfort,(std::abs(difference)-resolution)*.5),difference);
      section.displacement=shift*section.normal;
    }
    sections.push_back(section);
  }
  return allowed();
}
Eigen::MatrixXd displacementFor(const Eigen::MatrixXd& controls,double interval,
    const std::vector<Section>& sections) {
  const int n=controls.cols(),variables=n-6;
  Eigen::MatrixXd matrix=Eigen::MatrixXd::Zero(sections.size(),variables);
  Eigen::MatrixXd desired=Eigen::MatrixXd::Zero(sections.size(),2);
  for(std::size_t i=0;i<sections.size();++i) {
    const double span_time=sections[i].time/interval;
    const int span=std::min(n-4,static_cast<int>(std::floor(span_time)));
    const double u=std::clamp(span_time-span,0.,1.),u2=u*u,u3=u2*u;
    const std::array<double,4> basis{{std::pow(1.-u,3)/6.,(3.*u3-6.*u2+4.)/6.,
        (-3.*u3+3.*u2+3.*u+1.)/6.,u3/6.}};
    for(int j=0;j<4;++j)if(span+j>=3&&span+j<n-3)matrix(i,span+j-3)=basis[j];
    desired.row(i)=sections[i].displacement.transpose();
  }
  // A single bounded regularized least-squares solve. Smooth the displacement
  // field rather than moving each sample independently. Fixed boundary
  // controls participate as zero displacement, preserving endpoint P/V/A.
  Eigen::MatrixXd bending=Eigen::MatrixXd::Zero(n-2,variables);
  for(int row=0;row<n-2;++row)for(int j=0;j<3;++j) {
    const int column=row+j;
    if(column>=3&&column<n-3)bending(row,column-3)=j==1?-2.:1.;
  }
  Eigen::MatrixXd hessian=matrix.transpose()*matrix+.5*bending.transpose()*bending;
  hessian.diagonal().array()+=.08;
  const Eigen::MatrixXd solution=hessian.ldlt().solve(matrix.transpose()*desired);
  Eigen::MatrixXd displacement=Eigen::MatrixXd::Zero(3,n);
  if(solution.allFinite())displacement.block(0,3,2,variables)=solution.transpose();
  return displacement;
}
}  // namespace

SoftClearanceReport improveSoftClearance(UniformBspline& accepted,double resolution,
    const SoftClearanceOptions& options,const SplineHeadingContract& heading,
    const SolveBudget::Ptr& parent,const SoftClearanceQuery& query,
    const SoftClearanceAdmission& admission,const SoftClearanceWorkContext& context,
    const SupportHeightCoupling* support_height) {
  SoftClearanceReport report;const auto began=SolveBudget::Clock::now();
  const auto finish=[&](const char* reason) {
    report.reason=reason;report.elapsed_ms=std::chrono::duration<double,std::milli>(
        SolveBudget::Clock::now()-began).count();return report;
  };
  if(options.distance<=0.)return finish("disabled");
  if(!solveAllowed(parent))return finish("parent_stopped");
  if(!std::isfinite(options.distance)||options.distance>.5||!std::isfinite(resolution)||
      resolution<=0.||options.work_budget_ms<=0||options.work_budget_ms>100||
      !std::isfinite(options.preferred_turn_radius)||options.preferred_turn_radius<=0.||!query||!admission||
      !std::isfinite(context.completion_reserve_s)||context.completion_reserve_s<soft_clearance_publication_reserve_s)
    return finish("invalid_quality_options");
  const double available=parent?parent->remainingSeconds()-context.completion_reserve_s:1.;
  if(available<options.work_budget_ms*.001)return finish("publication_budget_reserved");
  const int milliseconds=std::min(options.work_budget_ms,static_cast<int>(available*1000.));
  if(milliseconds<5)return finish("publication_budget_reserved");
  // Include even the first external source-clock read in optional work. A
  // blocked read cannot be preempted, but must not earn another full budget
  // after returning and delay publication of the already valid baseline.
  const auto budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(milliseconds),
      []{return SolveBudget::Clock::now();},parent);
  bool source_stopped=false;double previous_source_remaining=std::numeric_limits<double>::infinity();
  const auto source_available=[&](double required) {
    if(!context.remaining_source)return true;
    if(source_stopped)return false;
    const double value=context.remaining_source();
    if(!std::isfinite(value)||value>previous_source_remaining+1e-9||value<required) {
      source_stopped=true;return false;
    }
    previous_source_remaining=value;return true;
  };
  // Source age includes time before request submission. The steady parent
  // alone cannot preserve the original solve-to-adoption source window.
  const bool initial_source_available=source_available(options.work_budget_ms*.001+context.completion_reserve_s);
  if(!solveAllowed(budget))return finish("quality_deadline");
  if(!initial_source_available)return finish("source_publication_reserved");
  const auto source_allowed=[&] {
    return solveAllowed(budget)&&source_available(context.completion_reserve_s)&&solveAllowed(budget);
  };
  const Eigen::MatrixXd original=accepted.getControlPoint();const auto knots=accepted.getKnot();
  const double interval=accepted.getInterval();
  if(accepted.getOrder()!=3||original.rows()!=3||original.cols()<=6||original.cols()>kMaxControls||
      !original.allFinite()||!std::isfinite(interval)||interval<=0.||knots.size()!=original.cols()+4)
    return finish("unsupported_quality_geometry");
  for(int i=1;i<knots.size();++i)
    if(!std::isfinite(knots[i])||std::abs(knots[i]-knots[i-1]-interval)>1e-8*std::max(1.,interval))
      return finish("nonuniform_quality_geometry");
  report.attempted=true;
  Eigen::MatrixXd curvature_gradient;
  report.initial_curvature_penalty=report.final_curvature_penalty=
      planarCurvatureCost(original,options.preferred_turn_radius,curvature_gradient);
  std::vector<Section> before;
  const int count=std::min(kMaxSections,static_cast<int>(original.cols())-3);
  if(!sectionsFor(accepted,count,resolution,options.distance,heading,budget,source_allowed,query,report,before,support_height))
    return finish(source_stopped?"source_publication_reserved":
        solveAllowed(budget)?"quality_probe_unavailable":"quality_deadline");
  report.sections=before.size();report.initial_penalty=report.final_penalty=penalty(before,options.distance);
  report.initial_minimum_side_margin=report.final_minimum_side_margin=minimumSideMargin(before);
  if(report.initial_penalty<1e-12)return finish("already_comfortable");
  Eigen::MatrixXd displacement=displacementFor(original,interval,before);
  if(!source_allowed())return finish(solveAllowed(budget)?"source_publication_reserved":"quality_deadline");
  if(!solveAllowed(budget))return finish("quality_deadline");
  double maximum=0.;for(int i=3;i<original.cols()-3;++i)maximum=std::max(maximum,displacement.col(i).norm());
  if(maximum<resolution*.05)return finish("balanced_or_fixed_boundary");
  if(maximum>options.distance)displacement*=options.distance/maximum;
  for(double blend:{1.,.5,.25}) {
    if(!source_allowed())return finish(solveAllowed(budget)?"source_publication_reserved":"quality_deadline");
    if(!solveAllowed(budget))return finish("quality_deadline");
    ++report.candidates;
    Eigen::MatrixXd adjusted=original;
    adjusted.block(0,3,2,original.cols()-6)+=blend*displacement.block(0,3,2,original.cols()-6);
    // Map before *all* cost, side-clearance and native admission evaluations;
    // mapping only inside admission would certify a different curve.
    if(support_height) support_height->apply(adjusted);
    const double curvature_penalty=planarCurvatureCost(adjusted,options.preferred_turn_radius,curvature_gradient);
    // Side clearance is not worth introducing an unnecessarily sharp bend.
    // This can only decline the optional improvement, never the baseline.
    if(curvature_penalty>report.initial_curvature_penalty+1e-7){++report.curvature_rejections;continue;}
    UniformBspline candidate(adjusted,3,interval);candidate.setKnot(knots);
    std::vector<Section> after;
    if(!sectionsFor(candidate,count,resolution,options.distance,heading,budget,source_allowed,query,report,after,support_height))
      continue;
    const double final_penalty=penalty(after,options.distance);
    // Do not trade the tightest sampled side margin for a better average.
    // This only declines an optional replacement: the proven baseline stays
    // available even when the requested comfort margin cannot be achieved.
    const double minimum_margin=minimumSideMargin(after);
    if(minimum_margin+1e-9<report.initial_minimum_side_margin){++report.clearance_rejections;continue;}
    if(final_penalty>=report.initial_penalty-1e-8){++report.clearance_rejections;continue;}
    if(!source_allowed()||!admission(candidate,budget)||!source_allowed()||!solveAllowed(budget))
      {++report.admission_rejections;continue;}
    accepted=std::move(candidate);report.changed=true;report.final_penalty=final_penalty;
    report.final_minimum_side_margin=minimum_margin;
    report.final_curvature_penalty=curvature_penalty;
    report.maximum_shift=std::min(maximum,options.distance)*blend;
    return finish("improved");
  }
  return finish(source_stopped?"source_publication_reserved":solveAllowed(budget)?"baseline_preserved":"quality_deadline");
}
}  // namespace scan_planner
