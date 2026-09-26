"""Shutdown race regression without rclpy.init, nodes or a ROS graph."""
import importlib
from unittest.mock import Mock

import pytest
from rclpy._rclpy_pybind11 import RCLError


@pytest.fixture(params=[('navigation_output', 'NavigationOutput'), ('lio_predictor', 'LioPredictor')])
def boundary(request, monkeypatch):
    name, constructor = request.param
    module = importlib.import_module('d1max_localization.' + name)
    node = Mock()
    monkeypatch.setattr(module, constructor, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'spin', Mock(side_effect=module.rclpy.executors.ExternalShutdownException()))
    monkeypatch.setattr(module.rclpy, 'try_shutdown', Mock())
    monkeypatch.setattr(module.rclpy, 'ok', Mock(return_value=False))
    monkeypatch.setattr(module.signal, 'signal', Mock())
    return module, node


def test_regular_shutdown_still_destroys_node(boundary):
    module, node = boundary
    module.main()
    node.destroy_node.assert_called_once_with()
    module.rclpy.try_shutdown.assert_called_once_with()


def test_already_closed_context_race_is_idempotent(boundary):
    module, node = boundary
    module.rclpy.try_shutdown.side_effect = RCLError(
        'failed to shutdown: rcl_shutdown already called on the given context')
    module.main()
    node.destroy_node.assert_called_once_with()
    module.rclpy.ok.assert_called_once_with()


def test_same_error_while_context_is_live_is_not_swallowed(boundary):
    module, node = boundary
    module.rclpy.ok.return_value = True
    module.rclpy.try_shutdown.side_effect = RCLError('rcl_shutdown already called')
    with pytest.raises(RCLError, match='already called'):
        module.main()
    node.destroy_node.assert_called_once_with()


def test_unrelated_rcl_error_on_closed_context_is_not_swallowed(boundary):
    module, node = boundary
    module.rclpy.try_shutdown.side_effect = RCLError('unexpected middleware shutdown failure')
    with pytest.raises(RCLError, match='unexpected middleware'):
        module.main()
    node.destroy_node.assert_called_once_with()


def test_unrelated_python_error_is_not_swallowed(boundary):
    module, node = boundary
    module.rclpy.try_shutdown.side_effect = RuntimeError('unexpected Python failure')
    with pytest.raises(RuntimeError, match='unexpected Python'):
        module.main()
    node.destroy_node.assert_called_once_with()
