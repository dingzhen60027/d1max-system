"""Unit tests are passive. ROS graph tests live in explicit offline runners."""
import pytest


@pytest.fixture(autouse=True)
def no_ros_graph_in_unit_tests(monkeypatch):
    import rclpy
    def forbidden(*args, **kwargs):
        raise AssertionError('Unit tests must not initialize ROS; use the isolated loopback runner')
    monkeypatch.setattr(rclpy, 'init', forbidden)
