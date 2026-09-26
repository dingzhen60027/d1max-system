"""Static architecture guards for the native map/bridge timestamp boundary.

These inspect source only; they are not a substitute for a native ROS pairing
runtime test. Actual Python queue behavior is exercised in test_live_scan_contract.
Keeping this suite ROS-free avoids starting transport, sensors or controllers.
"""

import ast
from pathlib import Path


SRC = Path(__file__).resolve().parents[2]
GRID = (SRC / 'scan_planner_vendor/plan_env/src/grid_map.cpp').read_text()
HEADER = (SRC / 'scan_planner_vendor/plan_env/include/plan_env/grid_map.h').read_text()
BRIDGE = (SRC / 'd1max_pct_scan/d1max_pct_scan/live_scan_bridge.py').read_text()


def cpp_function(name, following):
    return GRID.split(f'GridMap::{name}(', 1)[1].split(f'GridMap::{following}(', 1)[0]


def python_method(name):
    node = next(node for node in ast.walk(ast.parse(BRIDGE))
                if isinstance(node, ast.FunctionDef) and node.name == name)
    return ast.get_source_segment(BRIDGE, node)


def test_native_pairs_only_identical_original_stamps_newest_complete_first():
    source = cpp_function('processExactCloudPosePairs', 'slidingMapFrameCallback')
    assert 'pending_clouds_.rbegin()' in source
    assert 'pending_poses_.find(it->first)' in source
    assert 'pending_clouds_.at(selected).message' in source
    assert 'pending_poses_.at(selected).message' in source
    assert source.index('applyLidarPose(pose)') < source.index('processCloud(cloud)')
    assert 'pending_clouds_.upper_bound(selected)' in source
    assert 'pending_poses_.upper_bound(selected)' in source
    assert 'last_paired_stamp_ns_ = selected' in source


def test_native_exact_pose_callback_cannot_replace_origin_without_a_cloud():
    source = cpp_function('sensorPoseCallback', 'applyLidarPose')
    exact = source.split('if (mp_.exact_cloud_pose_sync_)', 1)[1]
    branch = exact.split('applyLidarPose(pose_msg)', 1)[0]
    assert 'rclcpp::Time(pose_msg->header.stamp).nanoseconds()' in branch
    assert 'pending_poses_.emplace(stamp,' in branch
    assert 'processExactCloudPosePairs();\n    return;' in branch
    assert 'md_.ray_pos_ =' not in branch


def test_native_pair_queues_and_waits_are_bounded_without_relaxing_source_age():
    poses = cpp_function('sensorPoseCallback', 'applyLidarPose')
    clouds = cpp_function('cloudCallback', 'processCloud')
    pairs = cpp_function('processExactCloudPosePairs', 'slidingMapFrameCallback')
    assert 'pending_poses_.size() > 8' in poses
    assert 'pending_clouds_.size() > 3' in clouds
    assert 'points > 250000' in clouds
    assert 'std::chrono::steady_clock::now()' in pairs
    assert 'mp_.cloud_pose_pair_wait_' in pairs
    assert '!cloudPoseStampFresh(entry.first)' in pairs
    assert 'mp_.cloud_pose_pair_wait_ > 0.25' in GRID
    assert 'mp_.cloud_pose_max_age_ > 0.5' in GRID


def test_clearing_cloud_revokes_integration_authority_and_old_queue_entries():
    source = cpp_function('invalidateCloudPosePairs', 'processExactCloudPosePairs')
    clouds = cpp_function('cloudCallback', 'processCloud')
    assert 'if (stamp >= last_paired_stamp_ns_) invalidateCloudPosePairs(stamp)' in clouds
    assert 'pair_barrier_ns_ = std::max(pair_barrier_ns_, barrier)' in source
    assert 'pending_clouds_.clear()' in source
    assert 'pending_poses_.clear()' in source
    assert 'projected_cloud_stamp_ns_ = integrated_cloud_stamp_ns_ = 0' in source
    assert 'md_.has_cloud_ = md_.has_ray_pose_ = false' in source
    assert 'return integrated_cloud_stamp_ns_ * 1e-9' in HEADER


def test_native_map_freshness_uses_actual_integrated_source_not_receipt_time():
    source = cpp_function('updateOccupancyCallback', 'sensorPoseCallback')
    assert source.index('!cloudPoseStampFresh(projected_cloud_stamp_ns_)') < source.index('raycastProcess()')
    assert source.index('raycastProcess()') < source.index('integrated_cloud_stamp_ns_ = projected_cloud_stamp_ns_')
    assert 'projected_cloud_stamp_ns_ = rclcpp::Time(img->header.stamp).nanoseconds()' in GRID


def test_bridge_preserves_one_exact_source_stamp_for_tf_cloud_and_sensor():
    source = python_method('publish_pending')
    assert 'Time.from_msg(message.header.stamp)' in source
    assert 'cloud = self.cloud_message(points, message.header.stamp)' in source
    assert "sensor.header.frame_id, sensor.header.stamp = self.p['map_frame'], message.header.stamp" in source
    assert "cloud.header.frame_id, cloud.header.stamp = self.p['map_frame'], stamp" in python_method('cloud_message')
    assert 'stamp <= self.sensor_barrier' in source
    assert 'stamp <= self.last_output_stamp' in source
    assert 'context != current_context' in source
