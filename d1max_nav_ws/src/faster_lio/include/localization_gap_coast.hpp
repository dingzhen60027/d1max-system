#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace faster_lio {
// Prediction model, NOT missing IMU integration or a measured posterior.
// No acceleration/bias/gravity update, no map update and no source timestamp
// are produced here. The caller requires a wholly post-gap deskewed scan and
// actual laser constraints before publishing another localization posterior.
struct LocalizationCoastLimits {
    double duration{.5}, distance{.25}, rotation{.30};
    double accel_sigma{2.}, angular_accel_sigma{1.};
};
inline bool ValidLocalizationCoastLimits(const LocalizationCoastLimits& l) {
    return std::isfinite(l.duration)&&l.duration>0&&l.duration<=.5&&
        std::isfinite(l.distance)&&l.distance>0&&l.distance<=.25&&
        std::isfinite(l.rotation)&&l.rotation>0&&l.rotation<=.30&&
        std::isfinite(l.accel_sigma)&&l.accel_sigma>=2.&&l.accel_sigma<=20.&&
        std::isfinite(l.angular_accel_sigma)&&l.angular_accel_sigma>=1.&&l.angular_accel_sigma<=20.;
}
template<class State,class Covariance>
bool CoastLocalizationFilter(State& state,Covariance& covariance,double dt,
    const Eigen::Vector3d& angular,const LocalizationCoastLimits& limits={}) {
    if(!ValidLocalizationCoastLimits(limits)||!std::isfinite(dt)||dt<=0||dt>limits.duration+1e-6||
       !state.pos.allFinite()||!state.vel.allFinite()||!state.rot.coeffs().allFinite()||
       !angular.allFinite()||!covariance.allFinite()||covariance.rows()!=23||covariance.cols()!=23||
       state.vel.norm()*dt>limits.distance||angular.norm()*dt>limits.rotation)return false;
    const auto norm=state.rot.coeffs().norm();
    if(!std::isfinite(norm)||std::abs(norm-1.)>.001)return false;
    auto trial=state;
    trial.pos+=trial.vel*dt;
    Eigen::Quaterniond q(state.rot.toRotationMatrix());
    const double angle=angular.norm()*dt;
    if(angle>1e-12)q=q*Eigen::Quaterniond(Eigen::AngleAxisd(angle,angular.normalized()));
    trial.rot.coeffs()=q.normalized().coeffs();
    Covariance f=Covariance::Identity();
    f.template block<3,3>(0,12)=Eigen::Matrix3d::Identity()*dt;
    Covariance p=f*covariance*f.transpose();
    const double a2=limits.accel_sigma*limits.accel_sigma, w2=limits.angular_accel_sigma*limits.angular_accel_sigma;
    for(int i=0;i<3;++i) {
        p(i,i)+=.25*a2*std::pow(dt,4);
        p(12+i,12+i)+=a2*dt*dt;
        p(i,12+i)+=.5*a2*dt*dt*dt;p(12+i,i)+=.5*a2*dt*dt*dt;
        p(3+i,3+i)+=.25*w2*std::pow(dt,4)+.04*dt*dt;
    }
    if(!trial.pos.allFinite()||!trial.rot.coeffs().allFinite()||!p.allFinite())return false;
    state=trial;covariance=(p+p.transpose())*.5;
    return true;
}
} // namespace faster_lio
