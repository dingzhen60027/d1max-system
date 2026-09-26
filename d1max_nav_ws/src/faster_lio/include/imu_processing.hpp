#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include <glog/logging.h>
#include <rclcpp/time.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"
#include "utils.h"
#include "stationary_initialization.hpp"

namespace faster_lio {

constexpr int MAX_INI_COUNT = 20;

bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); };

/// IMU Process and undistortion
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const common::V3D &transl, const common::M3D &rot);
    void SetGyrCov(const common::V3D &scaler);
    void SetAccCov(const common::V3D &scaler);
    void SetGyrBiasCov(const common::V3D &b_g);
    void SetAccBiasCov(const common::V3D &b_a);
    void Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudType::Ptr pcl_un_);

    common::M3D gravity_rotation_;    // rotation to align IMU up → world +z
    int initialization_samples{MAX_INI_COUNT};
    bool debug_file_enabled{true};
    bool stationary_initialization_enabled{false};
    bool gravity_aligned_world{false};
    // Localization-only conservative uncertainty for an admitted short gap.
    // Default 1.0 preserves all existing mapping algorithms/configurations.
    double integration_noise_scale{1.0};
    // Only robust localization supplies a real bracketing right endpoint.
    // Legacy mapping keeps its original integration path unchanged.
    bool bounded_scan_endpoints{false};
    StationaryInitialization initialization_gate;
    bool Initialized() const { return !imu_need_init_; }
    int InitializationSamples() const { return std::max(0, init_iter_num_ - 1); }
    double AccelerationScale() const { return common::G_m_s2 / mean_acc_.norm(); }

    std::ofstream fout_imu_;
    Eigen::Matrix<double, 12, 12> Q_;
    common::V3D cov_acc_;
    common::V3D cov_gyr_;
    common::V3D cov_acc_scale_;
    common::V3D cov_gyr_scale_;
    common::V3D cov_bias_gyr_;
    common::V3D cov_bias_acc_;

   private:
    void IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                      PointCloudType &pcl_out);

    PointCloudType::Ptr cur_pcl_un_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> v_imu_;
    std::vector<common::Pose6D> IMUpose_;
    std::vector<common::M3D> v_rot_pcl_;
    common::M3D Lidar_R_wrt_IMU_;
    common::V3D Lidar_T_wrt_IMU_;
    common::V3D mean_acc_;
    common::V3D mean_gyr_;
    common::V3D angvel_last_;
    common::V3D acc_s_last_;
    double last_lidar_end_time_ = 0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> initialization_window_;
};

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_ = process_noise_cov();
    cov_acc_ = common::V3D(0.1, 0.1, 0.1);
    cov_gyr_ = common::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = common::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = common::V3D(0.0001, 0.0001, 0.0001);
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    Lidar_T_wrt_IMU_ = common::Zero3d;
    Lidar_R_wrt_IMU_ = common::Eye3d;
    last_imu_.reset(new sensor_msgs::msg::Imu());
    gravity_rotation_.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    b_first_frame_ = true;
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    acc_s_last_ = common::Zero3d;
    last_lidar_end_time_ = 0.;
    gravity_rotation_.setIdentity();
    imu_need_init_ = true;
    init_iter_num_ = 1;
    v_imu_.clear();
    initialization_gate.clear();
    initialization_window_.clear();
    IMUpose_.clear();
    last_imu_.reset(new sensor_msgs::msg::Imu());
    cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const common::V3D &transl, const common::M3D &rot) {
    Lidar_T_wrt_IMU_ = transl;
    Lidar_R_wrt_IMU_ = rot;
}

void ImuProcess::SetGyrCov(const common::V3D &scaler) { cov_gyr_scale_ = scaler; }

void ImuProcess::SetAccCov(const common::V3D &scaler) { cov_acc_scale_ = scaler; }

void ImuProcess::SetGyrBiasCov(const common::V3D &b_g) { cov_bias_gyr_ = b_g; }

void ImuProcess::SetAccBiasCov(const common::V3D &b_a) { cov_bias_acc_ = b_a; }

void ImuProcess::IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    common::V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * common::G_m_s2);

    // Store rotation to level PCD (align IMU up → world +z)
    common::V3D up_dir = mean_acc_.normalized();
    gravity_rotation_ = Eigen::Quaterniond::FromTwoVectors(up_dir, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if (gravity_aligned_world) {
        init_state.rot = SO3(gravity_rotation_);
        init_state.grav = S2(common::V3D(0, 0, -common::G_m_s2));
        // Clouds, odometry and TF all share this frame from the first scan.
        // Do not rotate only the saved PCD a second time.
        gravity_rotation_.setIdentity();
    }

    init_state.bg = mean_gyr_;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
    last_imu_ = meas.imu_.back();
}

void ImuProcess::UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudType &pcl_out) {
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu_;
    const auto sample_time=[this](const auto& imu) {
        if (bounded_scan_endpoints) return rclcpp::Time(imu->header.stamp).seconds();
        return imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
    };
    if (!bounded_scan_endpoints || sample_time(last_imu_) < sample_time(v_imu.front()))
        v_imu.push_front(last_imu_);
    const double imu_beg_time = sample_time(v_imu.front());
    const double imu_end_time = sample_time(v_imu.back());
    const double &pcl_beg_time = meas.lidar_bag_time_;
    const double &pcl_end_time = meas.lidar_end_time_;

    if (bounded_scan_endpoints) {
        // Validate before mutating filter state. There is no extrapolation in
        // this branch: every integrated interval has two received endpoints.
        if (v_imu.size()<2 || imu_beg_time>last_lidar_end_time_ ||
            imu_end_time<pcl_end_time || pcl_end_time<=last_lidar_end_time_) {
            pcl_out.clear(); return;
        }
        for (size_t i=1; i<v_imu.size(); ++i) {
            if (sample_time(v_imu[i])<=sample_time(v_imu[i-1])) {
                pcl_out.clear(); return;
            }
        }
    }

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar_);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose_.clear();
    IMUpose_.push_back(common::set_pose6d(bounded_scan_endpoints ? last_lidar_end_time_-pcl_beg_time : 0.0,
                                          acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                          imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu_ point ***/
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        // if (tail->header.stamp.toSec() < last_lidar_end_time_) {
        //     continue;
        // }
        if (sample_time(tail) < last_lidar_end_time_) {
            continue;
        }


        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

        // if (head->header.stamp.toSec() < last_lidar_end_time_) {
        //     dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        // } else {
        //     dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        // }

        // ROS2
        if (head->header.stamp.sec + head->header.stamp.nanosec * 1e-9 < last_lidar_end_time_) {
            dt = tail->header.stamp.sec+ tail->header.stamp.nanosec * 1e-9 - last_lidar_end_time_;

        } else {
            dt = (tail->header.stamp.sec + tail->header.stamp.nanosec * 1e-9) - (head->header.stamp.sec + head->header.stamp.nanosec * 1e-9);
        }
        double integrated_end=sample_time(tail);
        if (bounded_scan_endpoints) {
            const double ta=sample_time(head),tb=sample_time(tail);
            const double begin=std::max(ta,last_lidar_end_time_);
            integrated_end=std::min(tb,pcl_end_time);
            if (integrated_end<=begin) continue;
            dt=integrated_end-begin;
            // Midpoint of the clipped interval, not the midpoint of the whole
            // IMU gap. These are integration inputs, not fabricated messages.
            const double alpha=((begin-ta)+(integrated_end-ta))*.5/(tb-ta);
            angvel_avr << head->angular_velocity.x+alpha*(tail->angular_velocity.x-head->angular_velocity.x),
                head->angular_velocity.y+alpha*(tail->angular_velocity.y-head->angular_velocity.y),
                head->angular_velocity.z+alpha*(tail->angular_velocity.z-head->angular_velocity.z);
            acc_avr << head->linear_acceleration.x+alpha*(tail->linear_acceleration.x-head->linear_acceleration.x),
                head->linear_acceleration.y+alpha*(tail->linear_acceleration.y-head->linear_acceleration.y),
                head->linear_acceleration.z+alpha*(tail->linear_acceleration.z-head->linear_acceleration.z);
            acc_avr *= common::G_m_s2 / mean_acc_.norm();
        }
        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_ * integration_noise_scale;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_ * integration_noise_scale;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf_state.predict(dt, Q_, in);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_last_ = angvel_avr - imu_state.bg;
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav[i];
        }

        const double offs_t = integrated_end - pcl_beg_time;
        IMUpose_.emplace_back(common::set_pose6d(offs_t, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    if (!bounded_scan_endpoints) {
        double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
        dt = note * (pcl_end_time - imu_end_time);
        kf_state.predict(dt, Q_, in);
    }

    imu_state = kf_state.get_x();
    if (bounded_scan_endpoints) {
        // Keep the real left tail. The future right endpoint remains in the
        // upstream queue and is not prepended ahead of older next-frame data.
        for (auto it=meas.imu_.rbegin();it!=meas.imu_.rend();++it) {
            if (sample_time(*it)<=pcl_end_time) { last_imu_=*it; break; }
        }
    } else last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.empty()) {
        return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose_.end() - 1; it_kp != IMUpose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = common::MatFromArray(head->rot);
        vel_imu = common::VecFromArray(head->vel);
        pos_imu = common::VecFromArray(head->pos);
        acc_imu = common::VecFromArray(tail->acc);
        angvel_avr = common::VecFromArray(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            common::M3D R_i(R_imu * Exp(angvel_avr, dt));

            common::V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            common::V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            common::V3D p_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudType::Ptr cur_pcl_un_) {
    if (meas.imu_.empty()) {
        return;
    }

    // ROS_ASSERT(meas.lidar_ != nullptr);
    LOG_ASSERT(meas.lidar_ != nullptr);

    if (imu_need_init_) {
        /// The very first lidar frame
        if (stationary_initialization_enabled) {
            for (const auto& imu : meas.imu_) {
                const auto& a = imu->linear_acceleration;
                const auto& w = imu->angular_velocity;
                const double t = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
                if (initialization_gate.add(t, {a.x,a.y,a.z}, {w.x,w.y,w.z}))
                    initialization_window_.push_back(imu);
            }
            if (initialization_gate.samples.empty()) { initialization_window_.clear(); return; }
            while (!initialization_window_.empty()) {
                const auto& first = initialization_window_.front()->header.stamp;
                if (first.sec + first.nanosec * 1e-9 >= initialization_gate.samples.front().t) break;
                initialization_window_.pop_front();
            }
            if (!initialization_gate.ready(initialization_samples)) return;
            auto stable_meas = meas;
            stable_meas.imu_.clear();
            for (const auto& imu : initialization_window_)
                stable_meas.imu_.push_back(std::make_shared<sensor_msgs::msg::Imu>(*imu));
            IMUInit(stable_meas, kf_state, init_iter_num_);
            LOG(INFO) << "Stationary initialization: " << stable_meas.imu_.size()
                      << " samples, gravity-aligned world=" << gravity_aligned_world
                      << ", mean acc=" << mean_acc_.transpose()
                      << ", gyro bias=" << mean_gyr_.transpose();
            initialization_window_.clear(); initialization_gate.clear();
        } else {
            IMUInit(meas, kf_state, init_iter_num_);
        }

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();
        if (bounded_scan_endpoints) last_lidar_end_time_=meas.lidar_end_time_;

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num_ > initialization_samples) {
            cov_acc_ *= pow(common::G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            LOG(INFO) << "IMU Initial Done";
            if(debug_file_enabled)fout_imu_.open(common::DEBUG_FILE_DIR("imu_.txt"), std::ios::out);
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, *cur_pcl_un_); }, "Undistort Pcl");
}
}  // namespace faster_lio

#endif
