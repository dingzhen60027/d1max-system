// Included in laser_mapping.cc: upstream IMU/IKFoM headers contain non-inline definitions.
// Opt-in bounded live localization runtime. The mapping launch remains compatible.
#include "laser_mapping.h"
#include <sstream>
#include <cmath>
#include <nlohmann/json.hpp>
namespace faster_lio {
static double stamp(const sensor_msgs::msg::Imu& m){return rclcpp::Time(m.header.stamp).seconds();}
void LaserMapping::InitRobustRuntime(){
    const auto lidar_topic=get_parameter("common.lid_topic").as_string();
    const auto imu_topic=get_parameter("common.imu_topic").as_string();
    if(preprocess_->GetLidarType()!=LidarType::AVIA || time_sync_en_)
        throw std::runtime_error("Localization requires timestamped CustomMsg and common sensor clock");
    robust_max_gap_=declare_parameter("localization.max_imu_gap_sec",.03);
    robust_soft_gap_=declare_parameter("localization.imu_gap_warning_sec",.015);
    robust_gap_rotation_=declare_parameter("localization.max_gap_rotation_rad",.08);
    const int threads=declare_parameter("localization.processing_threads",4);
    if(threads<1||threads>8 || !std::isfinite(robust_soft_gap_) || robust_soft_gap_<=0 ||
       robust_soft_gap_>robust_max_gap_ || !std::isfinite(robust_gap_rotation_) || robust_gap_rotation_<=0 || robust_gap_rotation_>.15)
        throw std::runtime_error("Invalid bounded LIO propagation settings");
    robust_max_age_=declare_parameter("localization.max_scan_age_sec",.3);
    robust_min_features_=declare_parameter("localization.min_features",40);
    robust_recovery_budget_.configure(
      declare_parameter("localization.reinitialize_on_imu_gap",false),
      declare_parameter("localization.max_imu_reinitializations",3),
      declare_parameter("localization.imu_recovery_window_sec",60.));
    if(!std::isfinite(robust_max_gap_)||robust_max_gap_<=0||robust_max_gap_>.05 ||
       !std::isfinite(robust_max_age_)||robust_max_age_<=0||robust_max_age_>1. ||robust_min_features_<10)
        throw std::runtime_error("Invalid bounded LIO admission parameters");
    p_imu_->initialization_samples=declare_parameter("localization.initialization_samples",200);
    if(p_imu_->initialization_samples<100||p_imu_->initialization_samples>2000)
        throw std::runtime_error("Invalid stationary initialization sample count");
    p_imu_->debug_file_enabled=false;
    const int capacity=declare_parameter("localization.map_capacity",100000);
    if(capacity<10000||capacity>500000)throw std::runtime_error("Invalid bounded local map capacity");
    ivox_options_.capacity_=static_cast<size_t>(capacity);
    pcd_save_en_=false;path_save_en_=false;path_pub_en_=false;
    robust_status_pub_=create_publisher<std_msgs::msg::String>("status",10);
    // Fault/reset events and valid samples share ONE ordered publisher. A status
    // on another topic cannot safely authorize an odometry from a different epoch.
    robust_sample_pub_=create_publisher<std_msgs::msg::String>("local_sample",rclcpp::QoS(20).reliable());
    sub_livox_=create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic,rclcpp::QoS(2),
      [this](livox_ros_driver2::msg::CustomMsg::SharedPtr m){
        if(m->points.size()<100||m->point_num!=m->points.size())return;
        std::lock_guard<std::mutex> lock(mtx_buffer_);
        if(robust_pending_)++robust_scans_replaced_;
        robust_pending_=std::move(m);
      });
    robust_imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;options.callback_group=robust_imu_group_;
    sub_imu_=create_subscription<sensor_msgs::msg::Imu>(imu_topic,rclcpp::QoS(512),
      [this](sensor_msgs::msg::Imu::SharedPtr m){
        const double t=stamp(*m);const auto& a=m->linear_acceleration;const auto& g=m->angular_velocity;
        if(!std::isfinite(t)||t<=0||!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||
           !std::isfinite(g.x)||!std::isfinite(g.y)||!std::isfinite(g.z))return;
        std::lock_guard<std::mutex> lock(mtx_buffer_);++robust_imu_received_;
        if(last_timestamp_imu_>0 && t<last_timestamp_imu_-.05){robust_imu_clock_reset_=true;robust_fault_=true;}
        if(t<=last_timestamp_imu_)return;
        last_timestamp_imu_=t;imu_buffer_.push_back(std::move(m));
        while(imu_buffer_.size()>1024)imu_buffer_.pop_front();
      },options);
}
void LaserMapping::RequestImuRecovery(const std::string& reason){
    robust_reason_=reason;robust_fault_=true;
    const double steady=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    robust_reset_pending_=robust_recovery_budget_.allow(reason,steady);
    robust_last_reset_reason_=reason;
    if(!robust_reset_pending_ && robust_recovery_budget_.enabled())robust_reason_="imu_recovery_limit";
    PublishRobustSample(false);
}
void LaserMapping::RobustRecovery(){
    if(!robust_reset_pending_ || robust_imu_clock_reset_)return;
    robust_reset_pending_=false;
    {
      std::lock_guard<std::mutex> lock(mtx_buffer_);
      robust_pending_.reset();imu_buffer_.clear();
    }
    lidar_buffer_.clear();time_buffer_.clear();measures_.imu_.clear();measures_.lidar_.reset();
    p_imu_->Reset();
    state_point_=state_ikfom();kf_.change_x(state_point_);
    decltype(kf_)::cov initial_covariance=decltype(kf_)::cov::Identity();
    kf_.change_P(initial_covariance);
    ivox_=std::make_shared<IVoxType>(ivox_options_);
    scan_undistort_->clear();scan_down_body_->clear();scan_down_world_->clear();
    nearest_points_.clear();corr_pts_.clear();corr_norm_.clear();
    residuals_.clear();point_selected_surf_.clear();plane_coef_.clear();
    flg_first_scan_=true;flg_EKF_inited_=false;localmap_initialized_=false;effect_feat_num_=0;
    first_lidar_time_=0;lidar_end_time_=0;robust_last_end_=0;robust_last_good_=0;
    ++robust_epoch_;++robust_resets_;robust_epoch_start_=now().seconds();
    robust_recovering_=true;robust_reason_="imu_reinitializing";
    // Never clear an independently observed sensor clock fault.
    robust_fault_=robust_imu_clock_reset_.load();
    if(robust_fault_)robust_reason_="imu_clock_reset";
    PublishRobustSample(false);robust_last_status_=0;
    RCLCPP_WARN(get_logger(),"IMU coverage lost: local epoch %lu reset; keep still, then submit a NEW map pose. No gap extrapolation.",robust_epoch_);
}
void LaserMapping::PublishRobustSample(bool valid){
    using nlohmann::json;
    json value={{"schema",1},{"epoch",robust_epoch_},{"valid",valid},
      {"fault",robust_fault_.load()},{"reason",robust_reason_},
      {"received_at_unix",now().seconds()}};
    if(valid){
      const auto& m=odom_aft_mapped_;const auto& p=m.pose.pose;const auto& v=m.twist.twist;
      value["stamp_ns"]=std::to_string(static_cast<int64_t>(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec);
      value["frame"]=m.header.frame_id;value["child_frame"]=m.child_frame_id;
      value["position"]={p.position.x,p.position.y,p.position.z};
      value["orientation"]={p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
      value["pose_covariance"]=m.pose.covariance;value["twist_covariance"]=m.twist.covariance;
      value["linear"]={v.linear.x,v.linear.y,v.linear.z};value["angular"]={v.angular.x,v.angular.y,v.angular.z};
      // Posterior at the exact scan-end stamp, not an unrelated/latest IMU state.
      // A separate downstream predictor can propagate this immutable snapshot;
      // it never changes this filter, its local map or the deskewed cloud.
      value["inertial"]={{"schema",1},
        {"world_velocity",{state_point_.vel.x(),state_point_.vel.y(),state_point_.vel.z()}},
        {"gyro_bias",{state_point_.bg.x(),state_point_.bg.y(),state_point_.bg.z()}},
        {"accel_bias",{state_point_.ba.x(),state_point_.ba.y(),state_point_.ba.z()}},
        {"gravity",{state_point_.grav[0],state_point_.grav[1],state_point_.grav[2]}},
        {"accel_scale",p_imu_->AccelerationScale()}};
    }
    std_msgs::msg::String m;m.data=value.dump();robust_sample_pub_->publish(m);
}
bool LaserMapping::RobustSync(){
    if(robust_fault_ || robust_imu_clock_reset_)return false;
    // The Run worker is the only owner of preprocessing, filter and map state.
    if(lidar_buffer_.empty()){
        livox_ros_driver2::msg::CustomMsg::SharedPtr m;
        {std::lock_guard<std::mutex> lock(mtx_buffer_);m.swap(robust_pending_);}
        if(!m)return false;
        const double start=rclcpp::Time(m->header.stamp).seconds();
        if(start<robust_epoch_start_)return false;
        if(start<=last_timestamp_lidar_){robust_fault_=true;robust_reason_="lidar_clock_reset";return false;}
        PointCloudType::Ptr scan(new PointCloudType);preprocess_->Process(m,scan);
        if(scan->size()<100)return false;
        std::sort(scan->begin(),scan->end(),[](const PointType&a,const PointType&b){return a.curvature<b.curvature;});
        last_timestamp_lidar_=start;lidar_buffer_.push_back(scan);time_buffer_.push_back(start);
    }
    const double start=time_buffer_.front(),end=start+lidar_buffer_.front()->back().curvature*.001;
    auto discard=[&](const char* reason){robust_reason_=reason;lidar_buffer_.pop_front();time_buffer_.pop_front();return false;};
    if(end-start<=0||end-start>.15)return discard("invalid_point_time");
    if(now().seconds()-end>robust_max_age_ || end-now().seconds()>.1)return discard("stale_scan");
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if(imu_buffer_.size()<2 || stamp(*imu_buffer_.back())<end)return false;
    const double begin=robust_last_end_>0?robust_last_end_:start;
    if(stamp(*imu_buffer_.front())>begin){
        if(robust_last_end_>0){
          RequestImuRecovery("imu_start_uncovered");
          lidar_buffer_.pop_front();time_buffer_.pop_front();return false;
        }
        return discard("imu_start_uncovered");
    }
    auto left=std::upper_bound(imu_buffer_.begin(),imu_buffer_.end(),begin,[](double t,const auto&m){return t<stamp(*m);});
    if(left!=imu_buffer_.begin())--left;
    auto right=std::lower_bound(left,imu_buffer_.end(),end,[](const auto&m,double t){return stamp(*m)<t;});
    if(right==imu_buffer_.end())return false;
    for(auto it=left;it!=right;++it){
        const double gap=stamp(**std::next(it))-stamp(**it);
        const auto& ga=(*it)->angular_velocity;const auto& gb=(*std::next(it))->angular_velocity;
        const double rotation=gap*std::max(std::sqrt(ga.x*ga.x+ga.y*ga.y+ga.z*ga.z),std::sqrt(gb.x*gb.x+gb.y*gb.y+gb.z*gb.z));
        robust_observed_gap_=std::max(robust_observed_gap_,gap);
        if(gap>robust_max_gap_ || (gap>robust_soft_gap_ && rotation>robust_gap_rotation_)){
            ++robust_gap_rejections_;robust_last_gap_=gap;
            RequestImuRecovery("imu_gap");
            lidar_buffer_.pop_front();time_buffer_.pop_front();return false;
        }
        // Integrate the measured endpoints over their REAL dt with process noise;
        // do not invent samples, repeat zero gyro, or claim a higher sensor rate.
        if(gap>robust_soft_gap_ && stamp(**std::next(it))>robust_last_end_)++robust_short_gaps_;
        if(!p_imu_->Initialized()){
            const auto& a=(*it)->linear_acceleration;const auto& g=(*it)->angular_velocity;
            const double an=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z),gn=std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z);
            if(an<8.5||an>11.1||gn>.10){p_imu_->Reset();return discard("keep_still_for_initialization");}
        }
    }
    measures_.lidar_=lidar_buffer_.front();measures_.lidar_bag_time_=start;measures_.lidar_end_time_=end;
    measures_.imu_.clear();
    for(auto it=left;it!=imu_buffer_.end() && stamp(**it)<=end;++it)measures_.imu_.push_back(*it);
    if(measures_.imu_.size()<2)return discard("insufficient_imu");
    // Preserve the tail sample for next frame integration; no unbounded queue.
    while(imu_buffer_.size()>2 && stamp(*imu_buffer_[1])<=end)imu_buffer_.pop_front();
    lidar_end_time_=end;robust_last_end_=end;
    lidar_buffer_.pop_front();time_buffer_.pop_front();return true;
}
void LaserMapping::RobustStatus(){
    if(robust_imu_clock_reset_)robust_reason_="imu_clock_reset";
    const double t=now().seconds();if(t-robust_last_status_<.2)return;robust_last_status_=t;
    const bool fresh=robust_last_good_>0 && t-robust_last_good_>=-.1 && t-robust_last_good_<robust_max_age_;
    std::ostringstream s;s<<"{\"backend\":\"faster_lio\",\"ready\":"<<(fresh&&!robust_fault_?"true":"false")
      <<",\"fault\":"<<(robust_fault_?"true":"false")<<",\"reason\":\""<<robust_reason_
      <<"\",\"imu_received\":"<<robust_imu_received_<<",\"imu_gap_rejections\":"<<robust_gap_rejections_
      <<",\"scans_replaced\":"<<robust_scans_replaced_<<",\"features\":"<<effect_feat_num_
      <<",\"short_imu_gaps\":"<<robust_short_gaps_<<",\"max_observed_imu_gap_sec\":"<<robust_observed_gap_
      <<",\"epoch\":"<<robust_epoch_<<",\"reset_count\":"<<robust_resets_
      <<",\"recovering\":"<<(robust_recovering_?"true":"false")
      <<",\"last_imu_gap_sec\":"<<robust_last_gap_<<",\"imu_gap_limit_sec\":"<<robust_max_gap_
      <<",\"last_reset_reason\":\""<<robust_last_reset_reason_<<"\""
      <<",\"initializing\":"<<(!p_imu_->Initialized()?"true":"false")
      <<",\"initialization_samples\":"<<p_imu_->InitializationSamples()
      <<",\"initialization_required\":"<<p_imu_->initialization_samples
      <<",\"gyro_bias\":["<<state_point_.bg.x()<<","<<state_point_.bg.y()<<","<<state_point_.bg.z()<<"]}";
    std_msgs::msg::String m;m.data=s.str();robust_status_pub_->publish(m);
    if(!fresh||robust_fault_)PublishRobustSample(false);
}
} // namespace faster_lio
