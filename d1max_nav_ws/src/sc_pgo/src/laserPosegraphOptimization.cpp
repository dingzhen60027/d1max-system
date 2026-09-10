// Copyright 2024 SC_PGO_ROS2 Contributors
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>  // Fix: Required for savePCDFileBinary
#include <pcl/filters/filter.h>  // Fix: Required for removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>
// #include <pcl/octree/octree_pointcloud_voxelcentroid.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <sensor_msgs/Imu.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int32.hpp>

// #include <eigen3/Eigen/Dense>

// #include <ceres/ceres.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0;
double rotaionAccumulated = 1000000.0;

bool isNowKeyFrame = false;

Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init
Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init pose is zero

std::queue<std::shared_ptr<nav_msgs::msg::Odometry>> odometryBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> fullResBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::NavSatFix>> gpsBuf;
std::queue<std::pair<int, int>> scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(
  new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
std::atomic<bool> gtSAMgraphMade{false};
gtsam::Values initialEstimate;
std::unique_ptr<gtsam::ISAM2> isam;  // Fix: Use smart pointer instead of raw pointer
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

bool publish_tf = false;
bool use_current_stamp_for_aft_pgo_odom = true;
bool save_keyframe_scans = false;
std::string map_frame = "map";
std::string odom_frame = "camera_init";
std::string body_frame = "body";

double scancontext_filter_size = 0.2;
double icp_filter_size = 0.2;
double icp_max_correspondence_distance = 4.0;
double icp_fitness_threshold = 0.15;
double icp_max_correction_translation = 6.0;
double icp_max_correction_rotation = 0.6;
double icp_max_vertical_correction = 0.3;
double icp_max_tilt_correction = 0.12;
double icp_overlap_max_distance = 0.35;
double icp_min_overlap_ratio = 0.6;
double loop_max_relative_translation = 2.0;
double max_sync_offset_sec = 0.02;
double loop_closure_frequency = 2.0;
double map_publish_frequency = 0.2;
double odom_rotation_stddev = 0.01;
double odom_translation_stddev = 0.03;
double loop_rotation_stddev = 0.02;
double loop_translation_stddev = 0.08;
double loop_robust_kernel_scale = 10.0;
int history_keyframe_search_num = 15;
int map_skip_frames = 1;
int minimum_keyframe_points = 200;
int loop_min_keyframe_separation = 60;
int loop_confirmation_count = 3;
int loop_confirmation_window_keyframes = 5;
int loop_candidate_index_tolerance = 8;
int loop_accept_cooldown_keyframes = 15;
int last_loop_queued_current_idx = -1;
int last_loop_checked_current_idx = -1;
int last_loop_candidate_current_idx = -1;
int last_loop_candidate_history_idx = -1;
int loop_candidate_streak = 0;
std::atomic<uint32_t> accepted_loop_count{0};
std::atomic<int64_t> latest_odom_stamp_ns{0};

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(
  new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
// bool useGPS = false;
sensor_msgs::msg::NavSatFix::SharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;
rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pubLoopCount;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;
std::ofstream loopEventSaveStream;
std::mutex mLoopEvent;

std::shared_ptr<rclcpp::Node> nh;

// Fix: Shutdown flag for graceful thread termination
std::atomic<bool> shutdown_requested{false};

std::string padZeros(int val, int num_digits = 6)
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

void logLoopEvent(
  const std::string & event, int history_keyframe, int current_keyframe,
  double value1 = 0.0, double value2 = 0.0)
{
  std::lock_guard<std::mutex> lock(mLoopEvent);
  if (!loopEventSaveStream.is_open()) {
    return;
  }
  const double ros_time = nh ? nh->get_clock()->now().seconds() : 0.0;
  loopEventSaveStream << std::fixed << std::setprecision(9) << ros_time << ','
                      << event << ',' << history_keyframe << ',' << current_keyframe
                      << ',' << value1 << ',' << value2 << '\n';
  loopEventSaveStream.flush();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
    gtsam::Point3(p.x, p.y, p.z));
}  // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);
  for (const auto & _pose6d : keyframePoses) {
    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void saveOptimizedVerticesKITTIformat(
  gtsam::Values _estimates,
  std::string _filename)
{
  using namespace gtsam;

  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);

  for (const auto & key_value : _estimates) {
    auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
    if (!p) {continue;}

    const Pose3 & pose = p->value();

    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void laserOdometryHandler(
  const nav_msgs::msg::Odometry::SharedPtr _laserOdometry)
{
  latest_odom_stamp_ns.store(
    rclcpp::Time(_laserOdometry->header.stamp).nanoseconds(),
    std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
  odometryBuf.push(_laserOdometry);
}  // laserOdometryHandler

void laserCloudFullResHandler(
  sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes)
{
  std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
  fullResBuf.push(_laserCloudFullRes);
}  // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps)
{
  if (useGPS) {
    std::lock_guard<std::mutex> lock(mBuf);  // Fix: RAII lock guard
    gpsBuf.push(_gps);
  }
}  // gpsHandler

void initNoises(void)
{
  gtsam::Vector priorNoiseVector6(6);
  priorNoiseVector6 << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8;
  priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

  gtsam::Vector odomNoiseVector6(6);
  odomNoiseVector6 <<
    odom_rotation_stddev * odom_rotation_stddev,
    odom_rotation_stddev * odom_rotation_stddev,
    odom_rotation_stddev * odom_rotation_stddev,
    odom_translation_stddev * odom_translation_stddev,
    odom_translation_stddev * odom_translation_stddev,
    odom_translation_stddev * odom_translation_stddev;
  odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

  gtsam::Vector robustNoiseVector6(
    6);    // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 <<
    loop_rotation_stddev * loop_rotation_stddev,
    loop_rotation_stddev * loop_rotation_stddev,
    loop_rotation_stddev * loop_rotation_stddev,
    loop_translation_stddev * loop_translation_stddev,
    loop_translation_stddev * loop_translation_stddev,
    loop_translation_stddev * loop_translation_stddev;
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(
      loop_robust_kernel_scale),
               // Cauchy is empirically good.
    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  double bigNoiseTolerentToXY = 1000000000.0;  // 1e9
  double gpsAltitudeNoiseScore = 250.0;  // if height is misaligned after loop
                                         // clsosing, use this value bigger
  gtsam::Vector robustNoiseVector3(3);   // gps factor has 3 elements (xyz)
  robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
    gpsAltitudeNoiseScore;    // means only caring altitude here. (because
                              // LOAM-like-methods tends to be asymptotically
                              // flyging)
  robustGPSNoise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(
      1),      // optional: replacing Cauchy by DCS or GemanMcClure is okay but
               // Cauchy is empirically good.
    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}  // initNoises

Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr & _odom)
{
  auto tx = _odom->pose.pose.position.x;
  auto ty = _odom->pose.pose.position.y;
  auto tz = _odom->pose.pose.position.z;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  return Pose6D{tx, ty, tz, roll, pitch, yaw};
}  // getOdom

Pose6D diffTransformation(const Pose6D & _p1, const Pose6D & _p2)
{
  Eigen::Affine3f SE3_p1 =
    pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
  Eigen::Affine3f SE3_p2 =
    pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
  Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
  Eigen::Affine3f SE3_delta;
  SE3_delta.matrix() = SE3_delta0;
  float dx, dy, dz, droll, dpitch, dyaw;
  pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
  // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll
  // << ", " << dpitch << ", " << dyaw << std::endl;

  return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)),
    double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
}  // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(
  const pcl::PointCloud<PointType>::Ptr & cloudIn, const Pose6D & tf)
{
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur =
    pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

  int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto & pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
      transCur(0, 1) * pointFrom.y +
      transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
      transCur(1, 1) * pointFrom.y +
      transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
      transCur(2, 1) * pointFrom.y +
      transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }

  return cloudOut;
}

void pubPath(void)
{
  // Publish odom and path
  nav_msgs::msg::Odometry odomAftPGO;
  nav_msgs::msg::Path pathAftPGO;
  pathAftPGO.header.frame_id = map_frame;
  mKF.lock();
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    const Pose6D & pose_est =
      keyframePosesUpdated.at(node_idx);    // Updated poses

    nav_msgs::msg::Odometry odomAftPGOthis;
    odomAftPGOthis.header.frame_id = map_frame;
    odomAftPGOthis.child_frame_id = body_frame;
    odomAftPGOthis.header.stamp =
      rclcpp::Time(keyframeTimes.at(node_idx) * 1e9);
    odomAftPGOthis.pose.pose.position.x = pose_est.x;
    odomAftPGOthis.pose.pose.position.y = pose_est.y;
    odomAftPGOthis.pose.pose.position.z = pose_est.z;

    tf2::Quaternion q;
    q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
    odomAftPGOthis.pose.pose.orientation.x = q.x();
    odomAftPGOthis.pose.pose.orientation.y = q.y();
    odomAftPGOthis.pose.pose.orientation.z = q.z();
    odomAftPGOthis.pose.pose.orientation.w = q.w();
    odomAftPGO = odomAftPGOthis;

    geometry_msgs::msg::PoseStamped poseStampAftPGO;
    poseStampAftPGO.header = odomAftPGOthis.header;
    poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

    pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
    pathAftPGO.header.frame_id = map_frame;
    pathAftPGO.poses.push_back(poseStampAftPGO);
  }
  mKF.unlock();
  if (use_current_stamp_for_aft_pgo_odom) {
    odomAftPGO.header.stamp = nh->get_clock()->now();
  }
  pubOdomAftPGO->publish(odomAftPGO);  // Last pose
  pubPathAftPGO->publish(pathAftPGO);  // Poses

  if (publish_tf && map_frame != odom_frame) {
    gtsam::Pose3 map_to_odom;
    bool correction_available = false;
    {
      std::lock_guard<std::mutex> lock(mKF);
      const int idx = recentIdxUpdated - 1;
      if (idx >= 0 && idx < static_cast<int>(keyframePoses.size()) &&
        idx < static_cast<int>(keyframePosesUpdated.size()))
      {
        const gtsam::Pose3 odom_to_body = Pose6DtoGTSAMPose3(keyframePoses[idx]);
        const gtsam::Pose3 map_to_body = Pose6DtoGTSAMPose3(keyframePosesUpdated[idx]);
        map_to_odom = map_to_body.compose(odom_to_body.inverse());
        correction_available = true;
      }
    }
    if (!correction_available) {
      return;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    const int64_t input_stamp_ns = latest_odom_stamp_ns.load(std::memory_order_relaxed);
    transformStamped.header.stamp = input_stamp_ns > 0 ?
      rclcpp::Time(input_stamp_ns) : rclcpp::Time(odomAftPGO.header.stamp);
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = odom_frame;
    transformStamped.transform.translation.x = map_to_odom.translation().x();
    transformStamped.transform.translation.y = map_to_odom.translation().y();
    transformStamped.transform.translation.z = map_to_odom.translation().z();
    tf2::Quaternion correction_q;
    correction_q.setRPY(
      map_to_odom.rotation().roll(), map_to_odom.rotation().pitch(),
      map_to_odom.rotation().yaw());
    transformStamped.transform.rotation.x = correction_q.x();
    transformStamped.transform.rotation.y = correction_q.y();
    transformStamped.transform.rotation.z = correction_q.z();
    transformStamped.transform.rotation.w = correction_q.w();

    if (!tf_broadcaster) {
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
    }
    tf_broadcaster->sendTransform(transformStamped);
  }
}  // pubPath

void updatePoses(void)
{
  mKF.lock();
  for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size());
    node_idx++)
  {
    Pose6D & p = keyframePosesUpdated[node_idx];
    p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
    p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
    p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
    p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
    p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
    p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
  }
  mKF.unlock();

  mtxRecentPose.lock();
  const gtsam::Pose3 & lastOptimizedPose =
    isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size()) - 1);
  recentOptimizedX = lastOptimizedPose.translation().x();
  recentOptimizedY = lastOptimizedPose.translation().y();

  // Fix: Set to size() instead of size()-1 so loops with < recentIdxUpdated include all keyframes
  recentIdxUpdated = int(keyframePosesUpdated.size());

  mtxRecentPose.unlock();
}  // updatePoses

void runISAM2opt(void)
{
  // called when a variable added
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  isamCurrentEstimate = isam->calculateEstimate();
  updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(
  pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType * pointFrom;

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(
    transformIn.translation().x(), transformIn.translation().y(),
    transformIn.translation().z(), transformIn.rotation().roll(),
    transformIn.rotation().pitch(), transformIn.rotation().yaw());

  int numberOfCores = 8;  // TODO move to yaml
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom->x +
      transCur(0, 1) * pointFrom->y +
      transCur(0, 2) * pointFrom->z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom->x +
      transCur(1, 1) * pointFrom->y +
      transCur(1, 2) * pointFrom->z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom->x +
      transCur(2, 1) * pointFrom->y +
      transCur(2, 2) * pointFrom->z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom->intensity;
  }
  return cloudOut;
}  // transformPointCloud

void loopFindNearKeyframesCloud(
  pcl::PointCloud<PointType>::Ptr & nearKeyframes,
  const int & key, const int & submap_size,
  const int & root_idx)
{
  // extract and stacking near keyframes (in global coord)
  nearKeyframes->clear();
  for (int i = -submap_size; i <= submap_size; ++i) {
    int keyNear = key + i;
    if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size())) {continue;}

    mKF.lock();
    // Fix: Use pose corresponding to each keyNear, not root_idx
    *nearKeyframes += *local2global(
      keyframeLaserClouds[keyNear],
      keyframePosesUpdated[keyNear]);
    mKF.unlock();
  }

  if (nearKeyframes->empty()) {return;}

  // downsample near keyframes
  pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
  downSizeFilterICP.setInputCloud(nearKeyframes);
  downSizeFilterICP.filter(*cloud_temp);
  *nearKeyframes = *cloud_temp;
}  // loopFindNearKeyframesCloud

std::optional<gtsam::Pose3> doICPVirtualRelative(
  int _loop_kf_idx,
  int _curr_kf_idx)
{
  // parse pointclouds
  pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
    new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(
    new pcl::PointCloud<PointType>());
  loopFindNearKeyframesCloud(
    cureKeyframeCloud, _curr_kf_idx, 0,
    _loop_kf_idx);                           // use same root of loop kf idx
  loopFindNearKeyframesCloud(
    targetKeyframeCloud, _loop_kf_idx,
    history_keyframe_search_num, _loop_kf_idx);

  if (cureKeyframeCloud->size() < static_cast<size_t>(minimum_keyframe_points) ||
    targetKeyframeCloud->size() < static_cast<size_t>(minimum_keyframe_points))
  {
    logLoopEvent(
      "reject_sparse", _loop_kf_idx, _curr_kf_idx,
      cureKeyframeCloud->size(), targetKeyframeCloud->size());
    RCLCPP_WARN(nh->get_logger(), "Reject loop: insufficient ICP points");
    return std::nullopt;
  }

  // loop verification
  sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
  pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
  cureKeyframeCloudMsg.header.frame_id = map_frame;
  cureKeyframeCloudMsg.header.stamp = nh->get_clock()->now();
  pubLoopScanLocal->publish(cureKeyframeCloudMsg);

  sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
  pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
  targetKeyframeCloudMsg.header.frame_id = map_frame;
  targetKeyframeCloudMsg.header.stamp = nh->get_clock()->now();
  pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // Align pointclouds
  icp.setInputSource(cureKeyframeCloud);
  icp.setInputTarget(targetKeyframeCloud);
  pcl::PointCloud<PointType>::Ptr unused_result(
    new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  if (icp.hasConverged() == false ||
    icp.getFitnessScore() > icp_fitness_threshold)
  {
    logLoopEvent(
      "reject_fitness", _loop_kf_idx, _curr_kf_idx,
      icp.getFitnessScore(), icp_fitness_threshold);
    std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore()
              << " > " << icp_fitness_threshold << "). Reject this SC loop."
              << std::endl;
    return std::nullopt;
  } else {
    std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore()
              << " < " << icp_fitness_threshold << "). Add this SC loop."
              << std::endl;
  }

  pcl::KdTreeFLANN<PointType> target_tree;
  target_tree.setInputCloud(targetKeyframeCloud);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  size_t overlap_points = 0;
  const double overlap_distance_sq =
    icp_overlap_max_distance * icp_overlap_max_distance;
  for (const auto & point : unused_result->points) {
    if (target_tree.nearestKSearch(
        point, 1, nearest_index, nearest_squared_distance) > 0 &&
      nearest_squared_distance.front() <= overlap_distance_sq)
    {
      ++overlap_points;
    }
  }
  const double overlap_ratio = unused_result->empty() ? 0.0 :
    static_cast<double>(overlap_points) / static_cast<double>(unused_result->size());
  if (overlap_ratio < icp_min_overlap_ratio) {
    logLoopEvent(
      "reject_overlap", _loop_kf_idx, _curr_kf_idx,
      overlap_ratio, icp_min_overlap_ratio);
    RCLCPP_WARN(
      nh->get_logger(), "Reject loop: ICP overlap %.3f < %.3f",
      overlap_ratio, icp_min_overlap_ratio);
    return std::nullopt;
  }

  // Get pose transformation
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionLidarFrame;
  correctionLidarFrame = icp.getFinalTransformation();
  pcl::getTranslationAndEulerAngles(
    correctionLidarFrame, x, y, z, roll, pitch,
    yaw);
  const gtsam::Pose3 correction =
    Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  const double correction_translation = correction.translation().norm();
  const double correction_rotation = correction.rotation().rpy().norm();
  if (correction_translation > icp_max_correction_translation ||
    correction_rotation > icp_max_correction_rotation ||
    std::abs(z) > icp_max_vertical_correction ||
    std::hypot(roll, pitch) > icp_max_tilt_correction)
  {
    logLoopEvent(
      "reject_correction", _loop_kf_idx, _curr_kf_idx,
      correction_translation, correction_rotation);
    RCLCPP_WARN(
      nh->get_logger(),
      "Reject loop: correction %.3f m / %.3f rad, z %.3f m, tilt %.3f rad exceeds gate",
      correction_translation, correction_rotation, std::abs(z), std::hypot(roll, pitch));
    return std::nullopt;
  }

  gtsam::Pose3 loop_pose;
  gtsam::Pose3 current_pose;
  {
    std::lock_guard<std::mutex> lock(mKF);
    loop_pose = Pose6DtoGTSAMPose3(keyframePosesUpdated.at(_loop_kf_idx));
    current_pose = Pose6DtoGTSAMPose3(keyframePosesUpdated.at(_curr_kf_idx));
  }

  // ICP maps the current cloud (already expressed in the global frame) onto the
  // historical submap. Compose that global correction with the current pose,
  // then express the corrected pose relative to the historical keyframe.
  const gtsam::Pose3 corrected_current_pose = correction.compose(current_pose);
  const gtsam::Pose3 loop_relative_pose = loop_pose.between(corrected_current_pose);
  if (loop_relative_pose.translation().norm() > loop_max_relative_translation) {
    logLoopEvent(
      "reject_relative_pose", _loop_kf_idx, _curr_kf_idx,
      loop_relative_pose.translation().norm(), loop_max_relative_translation);
    RCLCPP_WARN(
      nh->get_logger(), "Reject loop: matched poses remain %.3f m apart",
      loop_relative_pose.translation().norm());
    return std::nullopt;
  }
  RCLCPP_INFO(
    nh->get_logger(), "Verified loop geometry: fitness %.5f, overlap %.3f",
    icp.getFitnessScore(), overlap_ratio);
  return loop_relative_pose;
}  // doICPVirtualRelative

template<typename PointT>
void removeNaNAndInfiniteInPlace(typename pcl::PointCloud<PointT>::Ptr & cloud)
{
  if (!cloud || cloud->empty()) {return;}

  // First pass: remove NaNs using PCL’s built-in
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Second pass: remove infinities in-place
  size_t write_idx = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      cloud->points[write_idx++] = pt;
    }
  }

  cloud->points.resize(write_idx);
  cloud->width = static_cast<uint32_t>(write_idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

void process_pg()
{
  while (!shutdown_requested && rclcpp::ok()) {  // Fix: Check shutdown flag
    while (!shutdown_requested && !odometryBuf.empty() && !fullResBuf.empty()) {
      //
      // pop and check keyframe is or not
      //
      mBuf.lock();
      while (!odometryBuf.empty() &&
        rclcpp::Time(odometryBuf.front()->header.stamp).seconds() <
        rclcpp::Time(fullResBuf.front()->header.stamp).seconds())
      {
        odometryBuf.pop();
      }
      if (odometryBuf.empty()) {
        mBuf.unlock();
        break;
      }

      // Time equal check
      timeLaserOdometry =
        rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
      timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
      if (std::abs(timeLaserOdometry - timeLaser) > max_sync_offset_sec) {
        RCLCPP_WARN(
          nh->get_logger(), "Drop unsynchronized cloud: odom-cloud offset %.6f s",
          timeLaserOdometry - timeLaser);
        fullResBuf.pop();
        mBuf.unlock();
        continue;
      }

      laserCloudFullRes->clear();
      pcl::PointCloud<PointType>::Ptr thisKeyFrame(
        new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
      fullResBuf.pop();

      Pose6D pose_curr = getOdom(odometryBuf.front());
      odometryBuf.pop();

      // find nearest gps
      double eps = 0.1;  // find a gps topioc arrived within eps second
      while (!gpsBuf.empty()) {
        auto thisGPS = gpsBuf.front();
        auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
        if (abs(thisGPSTime - timeLaserOdometry) < eps) {
          currGPS = thisGPS;
          hasGPSforThisKF = true;
          break;
        } else {
          hasGPSforThisKF = false;
        }
        gpsBuf.pop();
      }
      mBuf.unlock();

      //
      // Early reject by counting local delta movement (for equi-spereated kf
      // drop)
      //
      odom_pose_prev = odom_pose_curr;
      odom_pose_curr = pose_curr;
      Pose6D dtf = diffTransformation(
        odom_pose_prev, odom_pose_curr);    // dtf means delta_transform

      double delta_translation = sqrt(
        dtf.x * dtf.x + dtf.y * dtf.y +
        dtf.z * dtf.z);                                // note: absolute value.
      translationAccumulated += delta_translation;
      rotaionAccumulated +=
        (dtf.roll + dtf.pitch + dtf.yaw);    // sum just naive approach.

      if (translationAccumulated > keyframeMeterGap ||
        rotaionAccumulated > keyframeRadGap)
      {
        isNowKeyFrame = true;
        translationAccumulated = 0.0;  // reset
        rotaionAccumulated = 0.0;      // reset
      } else {
        isNowKeyFrame = false;
      }

      if (!isNowKeyFrame) {continue;}

      if (!gpsOffsetInitialized) {
        if (hasGPSforThisKF) {  // if the very first frame
          gpsAltitudeInitOffset = currGPS->altitude;
          gpsOffsetInitialized = true;
        }
      }

      //
      // Save data and Add consecutive node
      //
      pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(
        new pcl::PointCloud<PointType>());
      downSizeFilterScancontext.setInputCloud(thisKeyFrame);
      downSizeFilterScancontext.filter(*thisKeyFrameDS);
      removeNaNAndInfiniteInPlace<PointType>(thisKeyFrameDS);
      if (thisKeyFrameDS->size() < static_cast<size_t>(minimum_keyframe_points)) {
        RCLCPP_WARN(
          nh->get_logger(), "Drop sparse keyframe with %zu points",
          thisKeyFrameDS->size());
        continue;
      }

      mKF.lock();
      keyframeLaserClouds.push_back(thisKeyFrameDS);
      keyframePoses.push_back(pose_curr);
      keyframePosesUpdated.push_back(pose_curr);  // init
      keyframeTimes.push_back(timeLaserOdometry);

      scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

      laserCloudMapPGORedraw = true;
      mKF.unlock();

      const int prev_node_idx = keyframePoses.size() - 2;
      const int curr_node_idx =
        keyframePoses.size() -
        1;    // becuase cpp starts with 0 (actually this index could be any
              // number, but for simple implementation, we follow sequential
              // indexing)
      if (!gtSAMgraphMade /* prior node */) {
        const int init_node_idx = 0;
        gtsam::Pose3 poseOrigin =
          Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
        // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
        // gtsam::Point3(0.0, 0.0, 0.0));

        mtxPosegraph.lock();
        {
          // prior factor
          gtSAMgraph.add(
            gtsam::PriorFactor<gtsam::Pose3>(
              init_node_idx, poseOrigin, priorNoise));
          initialEstimate.insert(init_node_idx, poseOrigin);
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

        gtSAMgraphMade = true;

        cout << "posegraph prior node " << init_node_idx << " added" << endl;
      } else { /* consecutive node (and odom factor) after the prior added */
               // == keyframePoses.size() > 1
        gtsam::Pose3 poseFrom =
          Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
        gtsam::Pose3 poseTo =
          Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

        mtxPosegraph.lock();
        {
          // odom factor
          gtSAMgraph.add(
            gtsam::BetweenFactor<gtsam::Pose3>(
              prev_node_idx, curr_node_idx, poseFrom.between(poseTo),
              odomNoise));

          // gps factor
          if (hasGPSforThisKF) {
            double curr_altitude_offseted =
              currGPS->altitude - gpsAltitudeInitOffset;
            mtxRecentPose.lock();
            gtsam::Point3 gpsConstraint(
              recentOptimizedX, recentOptimizedY,
              curr_altitude_offseted);    // in this example, only adjusting
                                          // altitude (for x and y, very big
                                          // noises are set)
            mtxRecentPose.unlock();
            gtSAMgraph.add(
              gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
            cout << "GPS factor added at node " << curr_node_idx << endl;
          }
          initialEstimate.insert(curr_node_idx, poseTo);
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

        if (curr_node_idx % 100 == 0) {
          cout << "posegraph odom node " << curr_node_idx << " added." << endl;
        }
      }
      // if want to print the current graph, use gtSAMgraph.print("\nFactor
      // Graph:\n");

      // save utility
      std::string curr_node_idx_str = padZeros(curr_node_idx);
      if (save_keyframe_scans) {
        pcl::io::savePCDFileBinary(
          pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame);
      }
      pgTimeSaveStream << timeLaser << std::endl;  // path
    }

    // ps.
    // scan context detector is running in another thread (in constant Hz, e.g.,
    // 1 Hz) pub path and point cloud in another thread

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_pg

void performSCLoopClosure(void)
{
  std::pair<int, float> detectResult;
  int curr_node_idx = -1;
  {
    std::lock_guard<std::mutex> lock(mKF);
    if (int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT + 1) {
      return;
    }
    curr_node_idx = static_cast<int>(keyframePoses.size()) - 1;
    if (curr_node_idx == last_loop_checked_current_idx) {
      return;
    }
    last_loop_checked_current_idx = curr_node_idx;
    detectResult = scManager.detectLoopClosureID();
  }

  int SCclosestHistoryFrameID = detectResult.first;
  if (SCclosestHistoryFrameID != -1) {
    const int prev_node_idx = SCclosestHistoryFrameID;
    if (curr_node_idx - prev_node_idx < loop_min_keyframe_separation) {
      logLoopEvent(
        "reject_separation", prev_node_idx, curr_node_idx,
        curr_node_idx - prev_node_idx, loop_min_keyframe_separation);
      loop_candidate_streak = 0;
      return;
    }

    const int current_delta = curr_node_idx - last_loop_candidate_current_idx;
    const int history_delta = std::abs(
      prev_node_idx - last_loop_candidate_history_idx);
    const int allowed_history_delta = loop_candidate_index_tolerance +
      2 * std::max(1, current_delta);
    const bool candidate_is_continuous =
      last_loop_candidate_current_idx >= 0 && current_delta > 0 &&
      current_delta <= loop_confirmation_window_keyframes &&
      history_delta <= allowed_history_delta;
    loop_candidate_streak = candidate_is_continuous ? loop_candidate_streak + 1 : 1;
    last_loop_candidate_current_idx = curr_node_idx;
    last_loop_candidate_history_idx = prev_node_idx;

    if (loop_candidate_streak < loop_confirmation_count) {
      logLoopEvent(
        "candidate_hold", prev_node_idx, curr_node_idx,
        loop_candidate_streak, loop_confirmation_count);
      RCLCPP_INFO(
        nh->get_logger(), "Hold loop candidate %d <-> %d: confirmation %d/%d",
        prev_node_idx, curr_node_idx, loop_candidate_streak, loop_confirmation_count);
      return;
    }
    if (last_loop_queued_current_idx >= 0 &&
      curr_node_idx - last_loop_queued_current_idx < loop_accept_cooldown_keyframes)
    {
      return;
    }
    if (curr_node_idx == last_loop_queued_current_idx) {
      return;
    }
    last_loop_queued_current_idx = curr_node_idx;
    logLoopEvent(
      "candidate_queued", prev_node_idx, curr_node_idx,
      loop_candidate_streak, current_delta);
    cout << "Loop detected! - between " << prev_node_idx << " and "
         << curr_node_idx << "" << endl;

    mBuf.lock();
    scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    // addding actual 6D constraints in the other thread, icp_calculation.
    mBuf.unlock();
  } else if (
    last_loop_candidate_current_idx >= 0 &&
    curr_node_idx - last_loop_candidate_current_idx >
    loop_confirmation_window_keyframes)
  {
    loop_candidate_streak = 0;
    last_loop_candidate_current_idx = -1;
    last_loop_candidate_history_idx = -1;
  }
}  // performSCLoopClosure

void process_lcd()
{
  rclcpp::Rate rate(loop_closure_frequency);

  while (!shutdown_requested && rclcpp::ok()) {
    rate.sleep();
    performSCLoopClosure();
    // performRSLoopClosure(); // TODO
  }
}

void process_icp(void)
{
  while (!shutdown_requested && rclcpp::ok()) {  // Fix: Check shutdown flag
    while (!scLoopICPBuf.empty()) {
      if (scLoopICPBuf.size() > 30) {
        RCLCPP_WARN(
          nh->get_logger(),
          "Too many loop closure candidates to be ICPed is waiting ... "
          "Do process_lcd less frequently (adjust loopClosureFrequency)");
      }

      mBuf.lock();
      std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
      scLoopICPBuf.pop();
      mBuf.unlock();

      const int prev_node_idx = loop_idx_pair.first;
      const int curr_node_idx = loop_idx_pair.second;
      auto relative_pose_optional =
        doICPVirtualRelative(prev_node_idx, curr_node_idx);

      if (relative_pose_optional) {
        gtsam::Pose3 relative_pose = relative_pose_optional.value();
        mtxPosegraph.lock();
        gtSAMgraph.add(
          gtsam::BetweenFactor<gtsam::Pose3>(
            prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
        mtxPosegraph.unlock();

        std_msgs::msg::UInt32 loop_count_msg;
        loop_count_msg.data = ++accepted_loop_count;
        pubLoopCount->publish(loop_count_msg);
        logLoopEvent(
          "accepted", prev_node_idx, curr_node_idx,
          loop_count_msg.data, 0.0);
        RCLCPP_INFO(
          nh->get_logger(), "Accepted loop %u: keyframe %d <-> %d",
          loop_count_msg.data, prev_node_idx, curr_node_idx);
      }
    }

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_icp

void process_viz_path()
{
  float hz = 10.0;
  rclcpp::Rate rate(hz);

  while (!shutdown_requested && rclcpp::ok()) {
    rate.sleep();
    if (recentIdxUpdated > 0) {
      pubPath();
    }
  }
}

void process_isam()
{
  float hz = 10.0;
  rclcpp::Rate rate(hz);

  while (!shutdown_requested && rclcpp::ok()) {
    rate.sleep();
    bool optimized = false;
    {
      std::lock_guard<std::mutex> lock(mtxPosegraph);
      if (gtSAMgraphMade && (gtSAMgraph.size() > 0 || !initialEstimate.empty())) {
        runISAM2opt();
        optimized = true;
      }
    }
    if (optimized) {
      saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat);
      saveOdometryVerticesKITTIformat(odomKITTIformat);
    }
  }
}

void pubMap(bool publish_message = true)
{
  int counter = 0;

  laserCloudMapPGO->clear();

  mKF.lock();
  // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size());
  // node_idx++) {
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    if (counter % map_skip_frames == 0) {
      *laserCloudMapPGO += *local2global(
        keyframeLaserClouds[node_idx],
        keyframePosesUpdated[node_idx]);
    }
    counter++;
  }
  mKF.unlock();

  downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
  downSizeFilterMapPGO.filter(*laserCloudMapPGO);

  if (publish_message) {
    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = map_frame;
    laserCloudMapPGOMsg.header.stamp = nh->get_clock()->now();
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
  }
}

void process_viz_map()
{
  rclcpp::Rate rate(map_publish_frequency);

  while (!shutdown_requested && rclcpp::ok()) {
    rate.sleep();
    if (recentIdxUpdated > 0) {
      pubMap();
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("laserPGO");

  nh->declare_parameter<std::string>("save_directory", "/");
  save_directory = nh->get_parameter("save_directory").as_string();
  if (save_directory.empty()) {
    save_directory = "./";
  }
  if (save_directory.back() != '/') {
    save_directory.push_back('/');
  }

  try {
    std::filesystem::create_directories(save_directory);
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_FATAL(
      nh->get_logger(), "Failed to create save directory %s: %s",
      save_directory.c_str(), e.what());
    return 2;
  }

  nh->declare_parameter<double>("keyframe_meter_gap", 2.0);
  keyframeMeterGap = nh->get_parameter("keyframe_meter_gap").as_double();

  nh->declare_parameter<double>("keyframe_deg_gap", 10.0);
  keyframeDegGap = nh->get_parameter("keyframe_deg_gap").as_double();

  nh->declare_parameter<bool>("publish_tf", false);
  publish_tf = nh->get_parameter("publish_tf").as_bool();
  nh->declare_parameter<bool>("use_current_stamp_for_aft_pgo_odom", true);
  use_current_stamp_for_aft_pgo_odom =
    nh->get_parameter("use_current_stamp_for_aft_pgo_odom").as_bool();

  nh->declare_parameter<std::string>("map_frame", "map");
  nh->declare_parameter<std::string>("odom_frame", "camera_init");
  nh->declare_parameter<std::string>("body_frame", "body");
  map_frame = nh->get_parameter("map_frame").as_string();
  odom_frame = nh->get_parameter("odom_frame").as_string();
  body_frame = nh->get_parameter("body_frame").as_string();

  nh->declare_parameter<bool>("use_gps", false);
  nh->declare_parameter<bool>("save_keyframe_scans", false);
  useGPS = nh->get_parameter("use_gps").as_bool();
  save_keyframe_scans = nh->get_parameter("save_keyframe_scans").as_bool();

  odomKITTIformat = save_directory + "odom_poses.txt";
  pgKITTIformat = save_directory + "optimized_poses.txt";
  pgTimeSaveStream =
    std::fstream(save_directory + "times.txt", std::fstream::out);
  if (!pgTimeSaveStream.is_open()) {
    RCLCPP_FATAL(
      nh->get_logger(), "Failed to open trajectory timestamp file in %s",
      save_directory.c_str());
    return 2;
  }
  pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
  loopEventSaveStream.open(save_directory + "loop_events.csv", std::ios::out);
  if (!loopEventSaveStream.is_open()) {
    RCLCPP_FATAL(
      nh->get_logger(), "Failed to open loop event file in %s",
      save_directory.c_str());
    return 2;
  }
  loopEventSaveStream <<
    "ros_time,event,history_keyframe,current_keyframe,value1,value2\n";
  loopEventSaveStream.flush();
  pgScansDirectory = save_directory + "Scans/";

  // Fix: Replace system() calls with std::filesystem (prevents shell injection)
  try {
    if (std::filesystem::exists(pgScansDirectory)) {
      std::filesystem::remove_all(pgScansDirectory);
    }
    std::filesystem::create_directories(pgScansDirectory);
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(
      nh->get_logger(), "Failed to setup directory %s: %s",
      pgScansDirectory.c_str(), e.what());
    throw;
  }

  keyframeRadGap = deg2rad(keyframeDegGap);

  nh->declare_parameter<double>("sc_dist_thres", 0.2);
  scDistThres = nh->get_parameter("sc_dist_thres").as_double();

  nh->declare_parameter<double>(
    "sc_max_radius",
    20.0);    // 80 is recommended for outdoor, and lower (e.g., 20, 40) values
              // are recommended for indoor
  scMaximumRadius = nh->get_parameter("sc_max_radius").as_double();

  nh->declare_parameter<double>("scancontext_filter_size", 0.2);
  nh->declare_parameter<double>("icp_filter_size", 0.2);
  nh->declare_parameter<double>("icp_max_correspondence_distance", 4.0);
  nh->declare_parameter<double>("icp_fitness_threshold", 0.15);
  nh->declare_parameter<double>("icp_max_correction_translation", 6.0);
  nh->declare_parameter<double>("icp_max_correction_rotation", 0.6);
  nh->declare_parameter<double>("icp_max_vertical_correction", 0.3);
  nh->declare_parameter<double>("icp_max_tilt_correction", 0.12);
  nh->declare_parameter<double>("icp_overlap_max_distance", 0.35);
  nh->declare_parameter<double>("icp_min_overlap_ratio", 0.6);
  nh->declare_parameter<double>("loop_max_relative_translation", 2.0);
  nh->declare_parameter<double>("max_sync_offset_sec", 0.02);
  nh->declare_parameter<double>("loop_closure_frequency", 2.0);
  nh->declare_parameter<double>("map_publish_frequency", 0.2);
  nh->declare_parameter<double>("odom_rotation_stddev", 0.01);
  nh->declare_parameter<double>("odom_translation_stddev", 0.03);
  nh->declare_parameter<double>("loop_rotation_stddev", 0.02);
  nh->declare_parameter<double>("loop_translation_stddev", 0.08);
  nh->declare_parameter<double>("loop_robust_kernel_scale", 10.0);
  nh->declare_parameter<int>("history_keyframe_search_num", 15);
  nh->declare_parameter<int>("map_skip_frames", 1);
  nh->declare_parameter<int>("minimum_keyframe_points", 200);
  nh->declare_parameter<int>("loop_min_keyframe_separation", 60);
  nh->declare_parameter<int>("loop_confirmation_count", 3);
  nh->declare_parameter<int>("loop_confirmation_window_keyframes", 5);
  nh->declare_parameter<int>("loop_candidate_index_tolerance", 8);
  nh->declare_parameter<int>("loop_accept_cooldown_keyframes", 15);
  scancontext_filter_size = nh->get_parameter("scancontext_filter_size").as_double();
  icp_filter_size = nh->get_parameter("icp_filter_size").as_double();
  icp_max_correspondence_distance =
    nh->get_parameter("icp_max_correspondence_distance").as_double();
  icp_fitness_threshold = nh->get_parameter("icp_fitness_threshold").as_double();
  icp_max_correction_translation =
    nh->get_parameter("icp_max_correction_translation").as_double();
  icp_max_correction_rotation =
    nh->get_parameter("icp_max_correction_rotation").as_double();
  icp_max_vertical_correction =
    nh->get_parameter("icp_max_vertical_correction").as_double();
  icp_max_tilt_correction =
    nh->get_parameter("icp_max_tilt_correction").as_double();
  icp_overlap_max_distance =
    nh->get_parameter("icp_overlap_max_distance").as_double();
  icp_min_overlap_ratio = nh->get_parameter("icp_min_overlap_ratio").as_double();
  loop_max_relative_translation =
    nh->get_parameter("loop_max_relative_translation").as_double();
  max_sync_offset_sec = nh->get_parameter("max_sync_offset_sec").as_double();
  loop_closure_frequency = nh->get_parameter("loop_closure_frequency").as_double();
  map_publish_frequency = nh->get_parameter("map_publish_frequency").as_double();
  odom_rotation_stddev = nh->get_parameter("odom_rotation_stddev").as_double();
  odom_translation_stddev = nh->get_parameter("odom_translation_stddev").as_double();
  loop_rotation_stddev = nh->get_parameter("loop_rotation_stddev").as_double();
  loop_translation_stddev = nh->get_parameter("loop_translation_stddev").as_double();
  loop_robust_kernel_scale =
    nh->get_parameter("loop_robust_kernel_scale").as_double();
  history_keyframe_search_num =
    nh->get_parameter("history_keyframe_search_num").as_int();
  map_skip_frames = std::max(
    1, static_cast<int>(nh->get_parameter("map_skip_frames").as_int()));
  minimum_keyframe_points = nh->get_parameter("minimum_keyframe_points").as_int();
  loop_min_keyframe_separation =
    nh->get_parameter("loop_min_keyframe_separation").as_int();
  loop_confirmation_count =
    nh->get_parameter("loop_confirmation_count").as_int();
  loop_confirmation_window_keyframes =
    nh->get_parameter("loop_confirmation_window_keyframes").as_int();
  loop_candidate_index_tolerance =
    nh->get_parameter("loop_candidate_index_tolerance").as_int();
  loop_accept_cooldown_keyframes =
    nh->get_parameter("loop_accept_cooldown_keyframes").as_int();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = std::make_unique<ISAM2>(parameters);  // Fix: Use make_unique instead of raw new
  initNoises();

  scManager.setSCdistThres(scDistThres);
  scManager.setMaximumRadius(scMaximumRadius);

  downSizeFilterScancontext.setLeafSize(
    scancontext_filter_size, scancontext_filter_size, scancontext_filter_size);
  downSizeFilterICP.setLeafSize(icp_filter_size, icp_filter_size, icp_filter_size);

  double mapVizFilterSize;
  nh->declare_parameter<double>("mapviz_filter_size", 0.4);
  mapVizFilterSize = nh->get_parameter("mapviz_filter_size")
    .as_double();                       // pose assignment every k frames
  downSizeFilterMapPGO.setLeafSize(
    mapVizFilterSize, mapVizFilterSize,
    mapVizFilterSize);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    subLaserCloudFullRes;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;

  subLaserCloudFullRes = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

  subLaserOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", 100, laserOdometryHandler);

  subGPS = nh->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 100,
    gpsHandler);

  pubOdomAftPGO = nh->create_publisher<nav_msgs::msg::Odometry>(
    "/aft_pgo_odom", rclcpp::QoS(100));
  pubOdomRepubVerifier = nh->create_publisher<nav_msgs::msg::Odometry>(
    "/repub_odom", rclcpp::QoS(100));
  pubPathAftPGO = nh->create_publisher<nav_msgs::msg::Path>(
    "/aft_pgo_path",
    rclcpp::QoS(100));
  pubMapAftPGO = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/aft_pgo_map", rclcpp::QoS(100));

  pubLoopScanLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/loop_scan_local", rclcpp::QoS(100));
  pubLoopSubmapLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/loop_submap_local", rclcpp::QoS(100));
  pubLoopCount = nh->create_publisher<std_msgs::msg::UInt32>(
    "/pgo_loop_count", rclcpp::QoS(10).transient_local());

  std::thread posegraph_slam{process_pg};  // pose graph construction
  std::thread lc_detection{process_lcd};   // loop closure detection
  std::thread icp_calculation{
    process_icp};    // loop constraint calculation via icp
  std::thread isam_update{
    process_isam};    // if you want to call less isam2 run (for saving
                      // redundant computations and no real-time visulization is
                      // required), uncommment this and comment all the above
                      // runisam2opt when node is added.

  std::thread viz_map{process_viz_map};  // visualization - map (low frequency
                                         // because it is heavy)
  std::thread viz_path{
    process_viz_path};    // visualization - path (high frequency)

  rclcpp::spin(nh);

  // Fix: Graceful shutdown - signal threads and wait for completion
  RCLCPP_INFO(nh->get_logger(), "Shutting down threads...");
  shutdown_requested = true;

  if (posegraph_slam.joinable()) {
    posegraph_slam.join();
  }
  if (lc_detection.joinable()) {
    lc_detection.join();
  }
  if (icp_calculation.joinable()) {
    icp_calculation.join();
  }
  if (isam_update.joinable()) {
    isam_update.join();
  }
  if (viz_map.joinable()) {
    viz_map.join();
  }
  if (viz_path.joinable()) {
    viz_path.join();
  }

  if (recentIdxUpdated > 0) {
    pubMap(false);
    const std::string optimized_map_path = save_directory + "optimized_map.pcd";
    try {
      std::filesystem::create_directories(save_directory);
      const int save_result =
        pcl::io::savePCDFileBinary(optimized_map_path, *laserCloudMapPGO);
      if (save_result == 0) {
        RCLCPP_INFO(
          nh->get_logger(), "Saved optimized map (%zu points) to %s",
          laserCloudMapPGO->size(), optimized_map_path.c_str());
      } else {
        RCLCPP_ERROR(
          nh->get_logger(), "Failed to save optimized map to %s",
          optimized_map_path.c_str());
      }
    } catch (const pcl::IOException & e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to save optimized map: %s", e.what());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to prepare map output: %s", e.what());
    }
  }

  if (loopEventSaveStream.is_open()) {
    loopEventSaveStream.flush();
    loopEventSaveStream.close();
  }

  RCLCPP_INFO(nh->get_logger(), "All threads terminated. Releasing ROS entities.");

  // rmw_zenoh owns a Tokio runtime. Global ROS shared_ptr instances must be
  // destroyed before process-exit TLS teardown, otherwise their static
  // destructors can call Zenoh after the runtime has already disappeared.
  subGPS.reset();
  subLaserOdometry.reset();
  subLaserCloudFullRes.reset();
  tf_broadcaster.reset();
  pubLoopCount.reset();
  pubLoopSubmapLocal.reset();
  pubLoopScanLocal.reset();
  pubOdomRepubVerifier.reset();
  pubMapAftPGO.reset();
  pubPathAftPGO.reset();
  pubOdomAftPGO.reset();
  nh.reset();
  return 0;
}
