// Included in laser_mapping.cc: upstream IMU/IKFoM headers contain non-inline definitions.
// Opt-in bounded live localization runtime. The mapping launch remains compatible.
#include "laser_mapping.h"
#include <sstream>
#include <cmath>
#include <nlohmann/json.hpp>
#include "localization_imu_gap.hpp"
namespace faster_lio {
static double stamp(const sensor_msgs::msg::Imu& m){return rclcpp::Time(m.header.stamp).seconds();}
void LaserMapping::RecordRobustTrace(LocalizationPipelineTrace::Stage stage,int64_t source_ns,
                                    int64_t auxiliary_ns,uint32_t value,bool anomaly){
    if(!robust_mode_)return;
    LocalizationPipelineTrace::Event event{source_ns,now().nanoseconds(),
      LocalizationPipelineTrace::SteadyNs(),auxiliary_ns,robust_trace_epoch_.load(),value,stage};
    if(anomaly)robust_trace_.request(event);else robust_trace_.record(event);
}
int64_t LaserMapping::BeginRobustPhase(LocalizationPipelineTrace::Stage stage,double source){
    if(!robust_mode_)return 0;
    const auto began=LocalizationPipelineTrace::SteadyNs();
    RecordRobustTrace(stage,static_cast<int64_t>(source*1e9));return began;
}
void LaserMapping::EndRobustPhase(LocalizationPipelineTrace::Stage stage,double source,int64_t began){
    if(!began)return;
    const auto duration=LocalizationPipelineTrace::SteadyNs()-began;
    RecordRobustTrace(stage,static_cast<int64_t>(source*1e9),duration);
    if(duration>100000000LL)RecordRobustTrace(LocalizationPipelineTrace::SlowPhase,
      static_cast<int64_t>(source*1e9),duration,static_cast<uint32_t>(stage),true);
}
void LaserMapping::StartRobustTraceReporter(){
    robust_trace_thread_=std::thread([this]{
      while(robust_trace_.wait()){
        auto snapshot=robust_trace_.take();if(!snapshot)continue;
        try{
          nlohmann::json rows=nlohmann::json::array(),statistics=nlohmann::json::array();
          const auto summary=LocalizationPipelineTrace::summarize(*snapshot);
          for(std::size_t i=1;i<summary.size();++i){const auto&s=summary[i];if(!s.count)continue;
            statistics.push_back({i,s.count,s.source_gap,s.source_left.source_ns,s.source_right.source_ns,
              s.receipt_gap,s.receipt_left.steady_ns,s.receipt_right.steady_ns,
              s.receipt_left.source_ns,s.receipt_right.source_ns,s.max_duration,s.max_coverage_lag});}
          const auto begin=snapshot->count>8?snapshot->count-8:0;
          for(std::size_t i=begin;i<snapshot->count;++i){const auto&e=snapshot->events[i];
            rows.push_back({e.stage,e.source_ns,e.ros_ns,e.steady_ns,e.auxiliary_ns,e.epoch,e.value});}
          const auto&t=snapshot->trigger;
          const nlohmann::json report={{"schema",1},{"capacity",LocalizationPipelineTrace::kCapacity},
            {"columns",{"stage","source_ns","ros_ns","steady_ns","auxiliary_ns","epoch","value"}},
            {"trigger",{t.stage,t.source_ns,t.ros_ns,t.steady_ns,t.auxiliary_ns,t.epoch,t.value}},
            {"summary_columns",{"stage","count","max_source_gap_ns","source_left_ns","source_right_ns",
              "max_receipt_gap_ns","receipt_left_steady_ns","receipt_right_steady_ns",
              "receipt_left_source_ns","receipt_right_source_ns","max_duration_ns","max_imu_coverage_lag_ns"}},
            {"summary",std::move(statistics)},
            {"source_basis","scan/imu receive/store/take: original header ns; processing: native double scan-end to wire ns"},
            {"stage_names",{"unused","scan_received","imu_received","scan_taken","preprocess_begin",
              "preprocess_end","sync_waiting_imu","sync_accepted","sync_discarded","imu_process_begin",
              "imu_process_end","solve_begin","solve_end","posterior_begin","posterior_end",
              "map_begin","map_end","legacy_odom_begin","legacy_odom_end","body_cloud_begin",
              "body_cloud_end","source_expired","posterior_source_gap","slow_phase","invalid_notice",
              "imu_stored","scan_stored"}},
            {"overwritten",snapshot->overwritten},{"contention_drops",snapshot->contention_drops},
            {"suppressed_requests",snapshot->suppressed_requests},{"window_events",snapshot->count},
            {"window_begin_steady_ns",snapshot->count?snapshot->events[0].steady_ns:0},
            {"window_end_steady_ns",snapshot->count?snapshot->events[snapshot->count-1].steady_ns:0},
            {"events",std::move(rows)}};
          // Formatting and logging are NEVER performed on input/solve/publish
          // threads. This independent exporter has one fixed-size pending trace.
          auto bounded=report;
          auto payload=bounded.dump();
          while(payload.size()>7500 && !bounded["events"].empty()){
            bounded["events"].erase(bounded["events"].begin());payload=bounded.dump();}
          if(payload.size()>7500){
            // A defensive bound even for extreme numeric widths. Keep the
            // trigger and coverage accounting; never send an oversized log.
            bounded.erase("stage_names");bounded.erase("source_basis");payload=bounded.dump();}
          if(payload.size()<=7500)RCLCPP_WARN(get_logger(),"LioPipelineTrace %s",payload.c_str());
          else RCLCPP_WARN(get_logger(),"LioPipelineTrace summary exceeded bound; events=%zu dropped=%lu",
            snapshot->count,static_cast<unsigned long>(snapshot->contention_drops));
        }catch(const std::exception&){
          RCLCPP_WARN(get_logger(),"LioPipelineTrace export failed; estimation is unaffected");
        }
      }
    });
}
void LaserMapping::InitRobustRuntime(){
    const auto lidar_topic=get_parameter("common.lid_topic").as_string();
    const auto imu_topic=get_parameter("common.imu_topic").as_string();
    if(preprocess_->GetLidarType()!=LidarType::AVIA || time_sync_en_)
        throw std::runtime_error("Localization requires timestamped CustomMsg and common sensor clock");
    robust_max_gap_=declare_parameter("localization.max_imu_gap_sec",.03);
    robust_soft_gap_=declare_parameter("localization.imu_gap_warning_sec",.015);
    robust_gap_rotation_=declare_parameter("localization.max_gap_rotation_rad",.08);
    robust_coast_gap_=declare_parameter("localization.max_coast_gap_sec",.5);
    robust_coast_distance_=declare_parameter("localization.max_coast_distance_m",.25);
    robust_coast_rotation_=declare_parameter("localization.max_coast_rotation_rad",.30);
    if(!ValidLocalizationCoastLimits({robust_coast_gap_,robust_coast_distance_,robust_coast_rotation_,2.,1.}))
        throw std::runtime_error("Invalid localization missing-IMU model bounds");
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
    if(!std::isfinite(robust_max_gap_)||robust_max_gap_<=0||robust_max_gap_>.1 ||
       !std::isfinite(robust_max_age_)||robust_max_age_<=0||robust_max_age_>1. ||robust_min_features_<10)
        throw std::runtime_error("Invalid bounded LIO admission parameters");
    p_imu_->initialization_samples=declare_parameter("localization.initialization_samples",200);
    if(p_imu_->initialization_samples<100||p_imu_->initialization_samples>2000)
        throw std::runtime_error("Invalid stationary initialization sample count");
    p_imu_->debug_file_enabled=false;
    p_imu_->bounded_scan_endpoints=true;
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
        RecordRobustTrace(LocalizationPipelineTrace::ScanReceived,rclcpp::Time(m->header.stamp).nanoseconds(),0,m->point_num);
        if(m->points.size()<100||m->point_num!=m->points.size())return;
        std::lock_guard<std::mutex> lock(mtx_buffer_);
        if(robust_pending_)++robust_scans_replaced_;
        robust_pending_=std::move(m);
        RecordRobustTrace(LocalizationPipelineTrace::ScanStored,
          rclcpp::Time(robust_pending_->header.stamp).nanoseconds(),0,robust_pending_->point_num);
      });
    robust_imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;options.callback_group=robust_imu_group_;
    sub_imu_=create_subscription<sensor_msgs::msg::Imu>(imu_topic,rclcpp::QoS(512),
      [this](sensor_msgs::msg::Imu::SharedPtr m){
        RecordRobustTrace(LocalizationPipelineTrace::ImuReceived,rclcpp::Time(m->header.stamp).nanoseconds());
        const double t=stamp(*m);const auto& a=m->linear_acceleration;const auto& g=m->angular_velocity;
        if(!std::isfinite(t)||t<=0||!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||
           !std::isfinite(g.x)||!std::isfinite(g.y)||!std::isfinite(g.z))return;
        std::lock_guard<std::mutex> lock(mtx_buffer_);++robust_imu_received_;
        if(last_timestamp_imu_>0 && t<last_timestamp_imu_-.05){robust_imu_clock_reset_=true;robust_fault_=true;}
        if(t<=last_timestamp_imu_)return;
        last_timestamp_imu_=t;imu_buffer_.push_back(std::move(m));
        RecordRobustTrace(LocalizationPipelineTrace::ImuStored,rclcpp::Time(imu_buffer_.back()->header.stamp).nanoseconds());
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
    robust_scan_degraded_=false;robust_scan_gap_=0;robust_scan_noise_scale_=1;
    robust_coast_pending_=false;
    ++robust_epoch_;++robust_resets_;robust_epoch_start_=now().seconds();
    robust_trace_epoch_.store(robust_epoch_);
    robust_recovering_=true;robust_reason_="imu_reinitializing";
    // Never clear an independently observed sensor clock fault.
    robust_fault_=robust_imu_clock_reset_.load();
    if(robust_fault_)robust_reason_="imu_clock_reset";
    PublishRobustSample(false);robust_last_status_=0;
    RCLCPP_WARN(get_logger(),"Unrecoverable IMU coverage: local epoch %lu reset; waiting for fresh initialization and confirmed map localization. Old map identity invalidated.",robust_epoch_);
}
void LaserMapping::PublishRobustSample(bool valid){
    using nlohmann::json;
    const auto phase=BeginRobustPhase(LocalizationPipelineTrace::PosteriorBegin,robust_last_good_);
    if(!valid)RecordRobustTrace(LocalizationPipelineTrace::InvalidNotice,
      static_cast<int64_t>(robust_last_good_*1e9),static_cast<int64_t>(robust_last_end_*1e9),robust_fault_.load());
    json value={{"schema",1},{"epoch",robust_epoch_},{"valid",valid},
      {"fault",robust_fault_.load()},{"reason",robust_reason_},
      {"received_at_unix",now().seconds()}};
    value["degraded"]=robust_scan_degraded_;
    value["scan_imu_gap_sec"]=robust_scan_gap_;
    value["imu_quality_scope"]="last_admitted_scan";
    value["imu_quality_scan_end_sec"]=robust_last_end_;
    // A weak scan has no new posterior. A consumer may only retain an already
    // saved real posterior with this exact source and its original deadline;
    // neither scan-end nor this report's receipt can renew prediction life.
    value["last_good_posterior_sec"]=robust_last_good_;
    value["finite_filter_state"]=FiniteLocalizationFilterState(state_point_,kf_.get_P());
    value["gap_coasting"]=robust_coast_pending_;
    value["coast_gap_limit_sec"]=robust_coast_gap_;
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
    EndRobustPhase(LocalizationPipelineTrace::PosteriorEnd,robust_last_good_,phase);
}
bool LaserMapping::RobustSync(){
    if(robust_fault_ || robust_imu_clock_reset_)return false;
    // The Run worker is the only owner of preprocessing, filter and map state.
    if(lidar_buffer_.empty()){
        livox_ros_driver2::msg::CustomMsg::SharedPtr m;
        {std::lock_guard<std::mutex> lock(mtx_buffer_);m.swap(robust_pending_);}
        if(!m)return false;
        RecordRobustTrace(LocalizationPipelineTrace::ScanTaken,rclcpp::Time(m->header.stamp).nanoseconds(),0,m->point_num);
        const double start=rclcpp::Time(m->header.stamp).seconds();
        if(start<robust_epoch_start_)return false;
        if(start<=last_timestamp_lidar_){robust_fault_=true;robust_reason_="lidar_clock_reset";return false;}
        const auto phase=BeginRobustPhase(LocalizationPipelineTrace::PreprocessBegin,start);
        PointCloudType::Ptr scan(new PointCloudType);preprocess_->Process(m,scan);
        EndRobustPhase(LocalizationPipelineTrace::PreprocessEnd,start,phase);
        if(scan->size()<100)return false;
        std::sort(scan->begin(),scan->end(),[](const PointType&a,const PointType&b){return a.curvature<b.curvature;});
        last_timestamp_lidar_=start;lidar_buffer_.push_back(scan);time_buffer_.push_back(start);
    }
    const double start=time_buffer_.front(),end=start+lidar_buffer_.front()->back().curvature*.001;
    auto discard=[&](const char* reason){robust_reason_=reason;
      RecordRobustTrace(LocalizationPipelineTrace::SyncDiscarded,static_cast<int64_t>(end*1e9));
      lidar_buffer_.pop_front();time_buffer_.pop_front();return false;};
    if(end-start<=0||end-start>.15)return discard("invalid_point_time");
    if(now().seconds()-end>robust_max_age_ || end-now().seconds()>.1)return discard("stale_scan");
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if(imu_buffer_.size()<2 || stamp(*imu_buffer_.back())<end){
      RecordRobustTrace(LocalizationPipelineTrace::SyncWaitingImu,static_cast<int64_t>(end*1e9),
        imu_buffer_.empty()?0:rclcpp::Time(imu_buffer_.back()->header.stamp).nanoseconds(),imu_buffer_.size());
      return false;
    }
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
    double scan_gap=0,noise_scale=1;
    uint64_t scan_gap_intervals=0;
    bool degraded=false;
    for(auto it=left;it!=right;){
        const double gap=stamp(**std::next(it))-stamp(**it);
        const auto& ga=(*it)->angular_velocity;const auto& gb=(*std::next(it))->angular_velocity;
        const double rotation=gap*std::max(std::sqrt(ga.x*ga.x+ga.y*ga.y+ga.z*ga.z),std::sqrt(gb.x*gb.x+gb.y*gb.y+gb.z*gb.z));
        robust_observed_gap_=std::max(robust_observed_gap_,gap);
        const auto quality=AssessLocalizationImuGap(gap,rotation,robust_soft_gap_,robust_max_gap_,robust_gap_rotation_);
        if(!quality.accepted){
            ++robust_gap_rejections_;robust_last_gap_=gap;
            const auto restart=std::next(it);
            const double restart_time=stamp(**restart);
            if(p_imu_->Initialized()&&robust_last_good_>0&&gap<=robust_coast_gap_+1e-6&&
               now().seconds()-robust_last_good_<=robust_coast_gap_+.25&&
               FiniteLocalizationFilterState(state_point_,kf_.get_P())) {
                robust_coast_pending_=true;robust_reason_="imu_gap_coasting";
                if(start<restart_time) {
                    // No complete motion history exists for this scan. Drop
                    // it, retain the measured posterior/map/epoch, and wait for
                    // a scan whose EVERY point is after a real restart sample.
                    ++robust_coast_scans_discarded_;PublishRobustSample(false);
                    return discard("imu_gap_coasting");
                }
                if(restart_time>now().seconds())return false; // Never consume a future restart state.
                if(p_imu_->ResumeAfterMissingImu(*restart,kf_,
                    {robust_coast_gap_,robust_coast_distance_,robust_coast_rotation_,2.,1.})) {
                    state_point_=kf_.get_x();robust_last_end_=restart_time;
                    it=left=restart;degraded=true;
                    PublishRobustSample(false);
                    continue; // Validate/integrate only the real post-gap pairs.
                }
            }
            RequestImuRecovery("imu_gap");
            lidar_buffer_.pop_front();time_buffer_.pop_front();return false;
        }
        scan_gap=std::max(scan_gap,gap);noise_scale=std::max(noise_scale,quality.noise_scale);
        degraded=degraded||quality.degraded;
        // Integrate the measured endpoints over their REAL dt with process noise;
        // do not invent samples, repeat zero gyro, or claim a higher sensor rate.
        if(quality.degraded)++scan_gap_intervals;
        if(!p_imu_->Initialized()){
            const auto& a=(*it)->linear_acceleration;const auto& g=(*it)->angular_velocity;
            const double an=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z),gn=std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z);
            if(an<8.5||an>11.1||gn>.10){p_imu_->Reset();return discard("keep_still_for_initialization");}
        }
        ++it;
    }
    measures_.lidar_=lidar_buffer_.front();measures_.lidar_bag_time_=start;measures_.lidar_end_time_=end;
    measures_.imu_.clear();
    for(auto it=left;it!=std::next(right);++it){
        // Initialization estimates a stationary posterior at scan end and must
        // not consume a future measurement. Propagation retains both real ends.
        if(!p_imu_->Initialized() && stamp(**it)>end)break;
        measures_.imu_.push_back(*it);
    }
    if(measures_.imu_.size()<2)return discard("insufficient_imu");
    robust_scan_gap_=scan_gap;robust_scan_noise_scale_=noise_scale;robust_scan_degraded_=degraded;
    robust_short_gaps_+=scan_gap_intervals;
    // Preserve the tail sample for next frame integration; no unbounded queue.
    while(imu_buffer_.size()>2 && stamp(*imu_buffer_[1])<=end)imu_buffer_.pop_front();
    lidar_end_time_=end;robust_last_end_=end;
    RecordRobustTrace(LocalizationPipelineTrace::SyncAccepted,static_cast<int64_t>(end*1e9),
      rclcpp::Time(measures_.imu_.back()->header.stamp).nanoseconds(),measures_.imu_.size());
    lidar_buffer_.pop_front();time_buffer_.pop_front();return true;
}
void LaserMapping::RobustStatus(){
    if(robust_imu_clock_reset_)robust_reason_="imu_clock_reset";
    const double t=now().seconds();if(t-robust_last_status_<.2)return;robust_last_status_=t;
    const bool fresh=!robust_coast_pending_&&robust_reason_!="weak_geometry"&&robust_last_good_>0 &&
      t-robust_last_good_>=-.1 && t-robust_last_good_<robust_max_age_;
    if(robust_last_good_>0 && t-robust_last_good_>=robust_max_age_)
      RecordRobustTrace(LocalizationPipelineTrace::SourceExpired,static_cast<int64_t>(robust_last_good_*1e9),
        static_cast<int64_t>(robust_last_end_*1e9),0,true);
    std::ostringstream s;s<<"{\"backend\":\"faster_lio\",\"ready\":"<<(fresh&&!robust_fault_?"true":"false")
      <<",\"fault\":"<<(robust_fault_?"true":"false")<<",\"reason\":\""<<robust_reason_
      <<"\",\"imu_received\":"<<robust_imu_received_<<",\"imu_gap_rejections\":"<<robust_gap_rejections_
      <<",\"scans_replaced\":"<<robust_scans_replaced_<<",\"features\":"<<effect_feat_num_
      <<",\"short_imu_gaps\":"<<robust_short_gaps_<<",\"max_observed_imu_gap_sec\":"<<robust_observed_gap_
      <<",\"imu_gap_count_scope\":\"accepted_scan_gap_intervals_not_unique_packets\""
      <<",\"imu_quality_scope\":\"last_admitted_scan\",\"imu_quality_scan_end_sec\":"<<std::to_string(robust_last_end_)
      <<",\"degraded\":"<<(robust_scan_degraded_?"true":"false")
      <<",\"scan_imu_gap_sec\":"<<robust_scan_gap_<<",\"imu_gap_warning_sec\":"<<robust_soft_gap_
      <<",\"imu_gap_noise_scale\":"<<robust_scan_noise_scale_
      <<",\"epoch\":"<<robust_epoch_<<",\"reset_count\":"<<robust_resets_
      <<",\"recovering\":"<<(robust_recovering_?"true":"false")
      <<",\"last_imu_gap_sec\":"<<robust_last_gap_<<",\"imu_gap_limit_sec\":"<<robust_max_gap_
      <<",\"last_reset_reason\":\""<<robust_last_reset_reason_<<"\""
      <<",\"gap_coasting\":"<<(robust_coast_pending_?"true":"false")
      <<",\"coast_recoveries\":"<<robust_coast_recoveries_
      <<",\"coast_scans_discarded\":"<<robust_coast_scans_discarded_
      <<",\"coast_gap_limit_sec\":"<<robust_coast_gap_
      <<",\"initializing\":"<<(!p_imu_->Initialized()?"true":"false")
      <<",\"initialization_samples\":"<<p_imu_->InitializationSamples()
      <<",\"initialization_required\":"<<p_imu_->initialization_samples
      <<",\"gyro_bias\":["<<state_point_.bg.x()<<","<<state_point_.bg.y()<<","<<state_point_.bg.z()<<"]}";
    std_msgs::msg::String m;m.data=s.str();robust_status_pub_->publish(m);
    if(!fresh||robust_fault_)PublishRobustSample(false);
}
} // namespace faster_lio
