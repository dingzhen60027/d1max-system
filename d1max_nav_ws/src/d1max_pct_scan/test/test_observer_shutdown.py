"""Exercise both production entrypoints with a fake ROS context, never ROS init."""
import importlib
import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
import rclpy.node
from rclpy._rclpy_pybind11 import RCLError


@pytest.fixture(params=['committed_route_view', 'component_health_monitor'])
def runtime(request, monkeypatch, tmp_path):
    module = importlib.import_module('d1max_pct_scan.' + request.param)
    state = SimpleNamespace(active=True, shutdown_error=None, close_before_error=False,
                            spin_error=None, signal_before_cleanup=False, nodes=[], events=[])

    class Node:
        def __init__(self, name):
            state.nodes.append(self)

        def declare_parameter(self, name, default):
            return SimpleNamespace(value={
                'session_id': 'shutdown-fixture', 'output_directory': str(tmp_path),
            }.get(name, default))

        def create_publisher(self, *args, **kwargs):
            return Mock()

        def create_subscription(self, *args, **kwargs):
            pass

        def create_timer(self, *args, **kwargs):
            pass

        def destroy_node(self):
            state.events.append('destroy')

    def spin(_):
        if state.signal_before_cleanup:
            state.active = False
            raise rclpy.executors.ExternalShutdownException()
        if state.spin_error is not None:
            raise state.spin_error

    def shutdown():
        state.events.append('shutdown')
        if state.close_before_error:
            state.active = False
        if state.shutdown_error is not None:
            raise state.shutdown_error
        if not state.active:
            raise RCLError('rcl_shutdown already called on the given context')
        state.active = False

    def try_shutdown():
        # The native signal thread can invalidate the context after this check,
        # even while Humble's Python Context.try_shutdown holds its own lock.
        if state.active:
            shutdown()

    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'spin', spin)
    monkeypatch.setattr(rclpy, 'ok', lambda: state.active)
    monkeypatch.setattr(rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(rclpy, 'try_shutdown', try_shutdown)
    monkeypatch.setattr(rclpy.node, 'Node', Node)
    (tmp_path / 'session.json').write_text(json.dumps(dict(
        id='shutdown-fixture', version_id='map-fixture', frame_id='map',
        pipeline_contract='single_floor_v3')))
    monkeypatch.setattr(sys, 'argv', ['observer', '--session', str(tmp_path)])
    state.run = module.main
    return state


def test_normal_exit_destroys_node_and_closes_context(runtime):
    runtime.run()
    assert len(runtime.nodes) == 1
    assert runtime.events == ['destroy', 'shutdown']
    assert not runtime.active


def test_signal_closed_context_before_cleanup_is_success(runtime):
    runtime.signal_before_cleanup = True
    runtime.run()
    assert runtime.events == ['destroy']
    assert not runtime.active


def test_signal_wins_shutdown_check_same_input_no_longer_raises(runtime):
    runtime.close_before_error = True
    runtime.shutdown_error = RCLError(
        'failed to shutdown: rcl_shutdown already called on the given context')
    runtime.run()
    assert runtime.events == ['destroy', 'shutdown']
    assert not runtime.active


@pytest.mark.parametrize('error,closed', [
    (RCLError('rcl_shutdown already called on the given context'), False),
    (RCLError('unexpected middleware shutdown failure'), True),
    (RuntimeError('unexpected Python shutdown failure'), True),
])
def test_real_shutdown_errors_propagate_unchanged(runtime, error, closed):
    runtime.shutdown_error = error
    runtime.close_before_error = closed
    with pytest.raises(type(error)) as caught:
        runtime.run()
    assert caught.value is error
    assert runtime.events == ['destroy', 'shutdown']


def test_callback_error_propagates_after_cleanup(runtime):
    error = RuntimeError('observer callback failed')
    runtime.spin_error = error
    with pytest.raises(RuntimeError) as caught:
        runtime.run()
    assert caught.value is error
    assert runtime.events == ['destroy', 'shutdown']
    assert not runtime.active
