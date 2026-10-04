"""Default pytest runs must not join a robot's inherited ROS graph.

Legacy graph tests (test_supervisor/test_icp_fusion_bridge) are not passive
unit tests and are explicitly excluded by reproduce_offline.sh. A future
integration runner must own and verify a private loopback Zenoh router/domain;
do not add a blanket environment-variable bypass to this safety boundary.
"""
import pytest


@pytest.fixture(autouse=True)
def forbid_implicit_ros_graph(monkeypatch):
    import rclpy
    def forbidden(*args, **kwargs):
        raise AssertionError('No implicit ROS graph in localization tests; use reviewed isolated integration runner')
    monkeypatch.setattr(rclpy,'init',forbidden)
