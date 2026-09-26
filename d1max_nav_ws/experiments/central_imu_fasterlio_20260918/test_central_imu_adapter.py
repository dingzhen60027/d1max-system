import math
import unittest
from unittest.mock import Mock, patch
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from sensor_msgs.msg import Imu
from central_imu_adapter import cleanup_ros_node, normalize_message


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(frame_id='central_imu', acceleration_scale=9.80665,
                           gyro_scale=1.0, timestamp_offset_sec=-.013)
        self.msg = Imu()
        self.msg.header.stamp.sec = 1772316441
        self.msg.header.stamp.nanosec = 5_000_123
        self.msg.linear_acceleration.x = .1
        self.msg.linear_acceleration.y = .2
        self.msg.linear_acceleration.z = -1.0
        self.msg.angular_velocity.x = .03
        self.msg.angular_velocity.y = -.02
        self.msg.angular_velocity.z = .01

    def test_si_without_airy_axis_rotation(self):
        out, _ = normalize_message(self.msg, self.config)
        self.assertAlmostEqual(out.linear_acceleration.x, .980665)
        self.assertAlmostEqual(out.linear_acceleration.y, 1.96133)
        self.assertAlmostEqual(out.linear_acceleration.z, -9.80665)
        self.assertEqual(out.angular_velocity.y, -.02)
        self.assertEqual(out.angular_velocity.z, .01)
        self.assertEqual(out.orientation_covariance[0], -1)
        self.assertEqual(out.header.frame_id, 'central_imu')

    def test_integer_stamp_borrows_second(self):
        out, stamp = normalize_message(self.msg, self.config)
        self.assertEqual(out.header.stamp.sec, 1772316440)
        self.assertEqual(out.header.stamp.nanosec, 992000123)
        self.assertEqual(stamp, 1772316440992000123)

    def test_unknown_covariance_preserved(self):
        out, _ = normalize_message(self.msg, self.config)
        self.assertTrue(all(x == 0 for x in out.linear_acceleration_covariance))

    def test_known_covariance_scaled(self):
        self.msg.linear_acceleration_covariance[0] = .01
        out, _ = normalize_message(self.msg, self.config)
        self.assertAlmostEqual(out.linear_acceleration_covariance[0], .01*9.80665**2)

    def test_invalid_is_rejected(self):
        self.msg.angular_velocity.z = math.nan
        with self.assertRaises(ValueError):
            normalize_message(self.msg, self.config)

    def test_cleanup_uses_nodes_own_context(self):
        node = Mock()
        with patch('central_imu_adapter.rclpy.try_shutdown') as shutdown:
            cleanup_ros_node(node)
        node.destroy_node.assert_called_once_with()
        shutdown.assert_called_once_with(context=node.context)

    def test_cleanup_closes_context_even_if_node_cleanup_fails(self):
        node = Mock()
        node.destroy_node.side_effect = RuntimeError('node cleanup failure')
        with patch('central_imu_adapter.rclpy.try_shutdown') as shutdown:
            with self.assertRaisesRegex(RuntimeError, 'node cleanup failure'):
                cleanup_ros_node(node)
        shutdown.assert_called_once_with(context=node.context)

    def test_cleanup_ignores_already_shutdown_race_only_when_context_is_closed(self):
        node = Mock()
        node.context.ok.return_value = False
        error = _rclpy.RCLError(
            'failed to shutdown: rcl_shutdown already called on the given context, '
            'at ./src/rcl/init.c:241')
        with patch('central_imu_adapter.rclpy.try_shutdown', side_effect=error) as shutdown:
            cleanup_ros_node(node)
        node.destroy_node.assert_called_once_with()
        node.context.ok.assert_called_once_with()
        shutdown.assert_called_once_with(context=node.context)

    def test_cleanup_propagates_already_shutdown_error_if_context_remains_active(self):
        node = Mock()
        node.context.ok.return_value = True
        error = _rclpy.RCLError('rcl_shutdown already called on the given context')
        with patch('central_imu_adapter.rclpy.try_shutdown', side_effect=error):
            with self.assertRaises(_rclpy.RCLError) as caught:
                cleanup_ros_node(node)
        self.assertIs(caught.exception, error)

    def test_cleanup_propagates_other_rcl_errors_even_if_context_is_closed(self):
        node = Mock()
        node.context.ok.return_value = False
        error = _rclpy.RCLError('failed to shutdown: different middleware failure')
        with patch('central_imu_adapter.rclpy.try_shutdown', side_effect=error):
            with self.assertRaises(_rclpy.RCLError) as caught:
                cleanup_ros_node(node)
        self.assertIs(caught.exception, error)

    def test_cleanup_does_not_swallow_other_exception_types_with_same_text(self):
        node = Mock()
        node.context.ok.return_value = False
        error = RuntimeError('rcl_shutdown already called on the given context')
        with patch('central_imu_adapter.rclpy.try_shutdown', side_effect=error):
            with self.assertRaises(RuntimeError) as caught:
                cleanup_ros_node(node)
        self.assertIs(caught.exception, error)

    def test_cleanup_preserves_node_cleanup_failure_after_benign_shutdown_race(self):
        node = Mock()
        node.context.ok.return_value = False
        node_error = RuntimeError('node cleanup failure')
        node.destroy_node.side_effect = node_error
        shutdown_error = _rclpy.RCLError('rcl_shutdown already called on the given context')
        with patch('central_imu_adapter.rclpy.try_shutdown', side_effect=shutdown_error):
            with self.assertRaises(RuntimeError) as caught:
                cleanup_ros_node(node)
        self.assertIs(caught.exception, node_error)


if __name__ == '__main__':
    unittest.main()
