#include "measured_curve_entry.hpp"
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <random>
#ifdef D1MAX_VENDOR_CURVE_DIFF
#include <bspline_opt/uniform_bspline.h>
#endif
using namespace d1monitor::execution3;
namespace {
bool count_allocations=false;std::size_t allocations=0;
struct Time {std::int32_t sec=0;std::uint32_t nanosec=0;};
Time stampAt(double t){return {static_cast<std::int32_t>(t),static_cast<std::uint32_t>(std::round((t-std::floor(t))*1e9))};}
struct Vector {double x=0.,y=0.,z=0.;};
struct Quaternion {double x=0.,y=0.,z=0.,w=1.;};
struct Pose {Vector position;Quaternion orientation;};
struct Twist {Vector linear,angular;};
struct Header {Time stamp;std::string frame_id;};
struct PoseStamped {Header header;Pose pose;};
struct PoseCov {Pose pose;};struct TwistCov {Twist twist;};
struct Odom {Header header;std::string child_frame_id="d1max_loc_base_link";PoseCov pose;TwistCov twist;};
struct Version {unsigned schema_version=3;std::string session_id="session",task_id="task",route_id="route",
  route_hash=std::string(64,'a'),map_version_id="map",localization_seed_id="seed",segment_id="segment",anchor_id="anchor";
  std::uint64_t localization_epoch=1,reference_generation=2,anchor_revision=3,context_sequence=4,map_geometry_revision=5;};
struct Spline {int order=3;std::int64_t traj_id=9;std::vector<double> knots,yaw_pts;std::vector<Vector> pos_pts;double yaw_dt=0.;};
struct Tagged:Version {std::uint64_t generation=2;std::string frame_id="d1max_loc_odom",point_reference="body_center",
  segment_kind="floor",required_mode="general";Spline trajectory;Tagged(){schema_version=2;}};
struct Demand {Version version;std::int64_t trajectory_id=9;Time body_source_stamp;};
struct Prepared {Demand demand;Time entry_source_stamp;PoseStamped measured_pose;Twist measured_twist;
  Pose curve_entry_pose;Twist curve_entry_twist;double curve_time=.2;};
struct Local {unsigned schema_version=2;bool usable=true;std::string session_id="session",map_version_id="map",localization_seed_id="seed";
  std::uint64_t localization_epoch=1;Time source_stamp,valid_until;Odom local_odometry;};
Vector vector(const EntryVector& p){return {p[0],p[1],p[2]};}
Tagged line(double axis_x=1.,double axis_y=0.) {
  Tagged t;for(int i=0;i<7;++i)t.trajectory.pos_pts.push_back({(i-1)*.1*axis_x,(i-1)*.1*axis_y,.5});
  for(int i=0;i<11;++i)t.trajectory.knots.push_back((i-3)/3.);
  return t;
}
Prepared prepared(const MeasuredCurveEntryCache& c){Prepared p;p.entry_source_stamp=p.demand.body_source_stamp=stampAt(10.);
  p.measured_pose.header.stamp=p.entry_source_stamp;p.measured_pose.header.frame_id="d1max_loc_odom";
  p.measured_pose.pose.position=p.curve_entry_pose.position=vector(c.positionAt(p.curve_time));
  p.measured_twist.linear=p.curve_entry_twist.linear=vector(c.velocityAt(p.curve_time));return p;
}
Local body(const MeasuredCurveEntryCache& c,double time=10.05,double curve_time=.25){Local m;m.source_stamp=stampAt(time);
  m.valid_until=stampAt(time+.1);
  m.local_odometry.header.stamp=m.source_stamp;m.local_odometry.header.frame_id="d1max_loc_odom";
  m.local_odometry.pose.pose.position=vector(c.positionAt(curve_time));
  m.local_odometry.twist.twist.linear=vector(c.velocityAt(curve_time));return m;
}
void geometryChecks(){
  const auto tagged=line();const auto c=MeasuredCurveEntryCache::build(tagged);assert(c&&c->matches(tagged));
  const auto p=prepared(*c);auto m=body(*c);
  // Ordinary 50 ms movement at 0.3 m/s exceeds the obsolete fixed-point test,
  // but the real body remains exactly on this same immutable candidate.
  assert(m.local_odometry.pose.pose.position.x-p.curve_entry_pose.position.x>.0125);
  const auto observed=c->observe(p,m,.3,10.05);assert(observed);
  assert(std::abs(observed->curve_time-.25)<1e-12&&observed->position_error<1e-12&&observed->velocity_error<1e-12);
  assert(p.curve_time==.2&&p.entry_source_stamp.sec==10&&p.entry_source_stamp.nanosec==0);
  // Shared end-to-end budget is supplied by the caller. A fresh actual body
  // may be reobserved after 125 ms; the old fixed 100 ms pipeline gate must not
  // overrule that profile. Original proof/permit expiry remains the caller's.
  const auto delayed=body(*c,10.125,.325);
  assert(c->observe(p,delayed,.3,10.125,.2));
  assert(!c->observe(p,body(*c,10.201,.401),.3,10.201,.2));
  // Acquired time bounds only the neighborhood: unequal wall-clock time cannot
  // move an unchanged measured body along the curve.
  auto stationary_body=body(*c,10.05,.2);const auto stationary=c->observe(p,stationary_body,.3,10.06);
  assert(stationary&&std::abs(stationary->curve_time-.2)<1e-12);
  for(int which=0;which<14;++which){auto bad=m;auto entry=p;double now=10.05;
    if(which==0)bad.localization_epoch++;
    if(which==1)bad.localization_seed_id="foreign";
    if(which==2)bad.map_version_id="foreign";
    if(which==3)bad.session_id="foreign";
    if(which==4)bad.local_odometry.header.frame_id="map";
    if(which==5)bad.local_odometry.header.stamp=stampAt(10.04);
    if(which==6)bad.source_stamp=bad.local_odometry.header.stamp=stampAt(9.99);
    if(which==7)now=10.151;
    if(which==8)bad.local_odometry.pose.pose.position.x+=.12;
    if(which==9)bad.local_odometry.pose.pose.position.z+=.013;
    if(which==10)bad.local_odometry.twist.twist.linear.y=.051;
    if(which==11)entry.demand.version.anchor_revision++;
    if(which==12)entry.curve_entry_pose.position.x+=.002;
    if(which==13){bad=body(*c,10.11,.31);now=10.11;}
    assert(!c->observe(entry,bad,.3,now));
  }
  for(int which=0;which<7;++which){auto replacement=tagged;
    if(which==0)replacement.trajectory.pos_pts[3].y=.001;
    if(which==1)replacement.trajectory.knots[4]+=.001;
    if(which==2)replacement.trajectory.yaw_pts.push_back(.1);
    if(which==3)replacement.localization_epoch++;
    if(which==4)replacement.context_sequence++;
    if(which==5)replacement.generation++;
    if(which==6)replacement.required_mode="stairs";
    assert(!c->matches(replacement));
  }
  for(int which=0;which<5;++which){auto invalid=tagged;
    if(which==0)invalid.trajectory.knots[4]=invalid.trajectory.knots[3];
    if(which==1)invalid.trajectory.knots[4]=invalid.trajectory.knots[3]+1e-10;
    if(which==2)invalid.trajectory.knots[4]=std::numeric_limits<double>::quiet_NaN();
    if(which==3)invalid.trajectory.pos_pts[2].x=std::numeric_limits<double>::infinity();
    if(which==4)invalid.trajectory.order=4;
    assert(!MeasuredCurveEntryCache::build(invalid));
  }
  // Holonomic lateral motion must compare the actual child-frame velocity after
  // quaternion rotation to odom, not reject a correct world-frame tangent.
  const auto lateral=MeasuredCurveEntryCache::build(line(0.,1.));assert(lateral);
  const auto q=prepared(*lateral);auto sideways=body(*lateral);
  sideways.local_odometry.pose.pose.orientation.z=std::sqrt(.5);
  sideways.local_odometry.pose.pose.orientation.w=std::sqrt(.5);
  sideways.local_odometry.twist.twist.linear={.3,0.,0.};
  assert(lateral->observe(q,sideways,.3,10.05));
  sideways.local_odometry.twist.twist.linear={0.,.3,0.};assert(!lateral->observe(q,sideways,.3,10.05));
  const auto before=allocations;count_allocations=true;
  for(unsigned i=0;i<10000;++i)assert(c->observe(p,m,.3,10.05));
  count_allocations=false;assert(allocations==before);
  // Largest allowed control cache: the 20 Hz lane searches its short acquired
  // arc neighborhood, not all 10k controls or a newly constructed curve.
  auto maximum=line();maximum.trajectory.pos_pts.clear();maximum.trajectory.knots.clear();
  constexpr unsigned controls=10000;const double interval=119.9/(controls-3);
  for(unsigned i=0;i<controls;++i)maximum.trajectory.pos_pts.push_back({(static_cast<double>(i)-1.)*.3*interval,0.,.5});
  for(unsigned i=0;i<controls+4;++i)maximum.trajectory.knots.push_back((static_cast<double>(i)-3.)*interval);
  const auto large=MeasuredCurveEntryCache::build(maximum);assert(large);
  auto lp=prepared(*large);lp.curve_time=60.;lp.measured_pose.pose.position=lp.curve_entry_pose.position=vector(large->positionAt(60.));
  lp.measured_twist.linear=lp.curve_entry_twist.linear=vector(large->velocityAt(60.));
  const auto lm=body(*large,10.05,60.05);const auto begin=std::chrono::steady_clock::now();
  const auto prior=allocations;count_allocations=true;
  for(unsigned i=0;i<10000;++i)assert(large->observe(lp,lm,.3,10.05));
  count_allocations=false;assert(allocations==prior);
  const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count()/10000.;
  std::cout<<"max-control cached observation average_us="<<us<<" (passive host diagnostic, not real-time guarantee)\n";
}
#ifdef D1MAX_VENDOR_CURVE_DIFF
void vendorChecks(){
  for(bool nonuniform:{false,true}){
    Tagged t;t.trajectory.pos_pts.clear();t.trajectory.knots.clear();
    for(unsigned i=0;i<127;++i)t.trajectory.pos_pts.push_back({std::sin(i*.13),std::cos(i*.31),i*.0123});
    t.trajectory.knots.push_back(-1.3);
    for(unsigned i=1;i<131;++i)t.trajectory.knots.push_back(t.trajectory.knots.back()+(nonuniform?.01+.013*(i%7):.2));
    const auto c=MeasuredCurveEntryCache::build(t);assert(c);
    Eigen::MatrixXd points(3,127);Eigen::VectorXd knots(131);
    for(unsigned i=0;i<127;++i)points.col(i)=Eigen::Vector3d(t.trajectory.pos_pts[i].x,t.trajectory.pos_pts[i].y,t.trajectory.pos_pts[i].z);
    for(unsigned i=0;i<131;++i)knots[i]=t.trajectory.knots[i];
    scan_planner::UniformBspline vendor(points,3,.2);vendor.setKnot(knots);auto velocity=vendor.getDerivative();
    const auto same=[&](double time){const auto pos=vendor.evaluateDeBoorT(time),vel=velocity.evaluateDeBoorT(time);
      const auto a=c->positionAt(time),b=c->velocityAt(time);
      for(unsigned axis=0;axis<3;++axis){assert(std::memcmp(&a[axis],&pos[axis],sizeof(double))==0);
        assert(std::memcmp(&b[axis],&vel[axis],sizeof(double))==0);}};
    const double infinity=std::numeric_limits<double>::infinity();
    for(double time:{-100.,-0.,0.,c->duration(),c->duration()+100.,infinity,-infinity,
        std::numeric_limits<double>::quiet_NaN()})same(time);
    for(unsigned i=3;i<=127;++i){const double time=knots[i]-knots[3];same(time);
      same(std::nextafter(time,-INFINITY));same(std::nextafter(time,INFINITY));}
    std::mt19937 generator(832520);std::uniform_real_distribution<double> distribution(-.1,c->duration()+.1);
    for(unsigned i=0;i<2000;++i)same(distribution(generator));
  }
}
#endif
}
void* operator new(std::size_t size){if(count_allocations)++allocations;if(void* p=std::malloc(size))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{std::free(p);}
void operator delete(void* p,std::size_t)noexcept{std::free(p);}
int main(){geometryChecks();
#ifdef D1MAX_VENDOR_CURVE_DIFF
  vendorChecks();
#endif
  std::cout<<"measured curve entry: moving/source/identity/geometry bounds + allocation checks passed\n";
}
