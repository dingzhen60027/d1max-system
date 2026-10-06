"""A signal during publication must finish before its ROS context is closed."""
import signal
import sys
from types import ModuleType, SimpleNamespace

import pytest

import bridge


def fake_ros(monkeypatch, *, delivered_signal=None, publish_failure=None):
    events = []
    handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    original_handlers = dict(handlers)
    no_handlers = object()

    class Context:
        valid = False

        def ok(self):
            return self.valid

        def try_shutdown(self):
            events.append('context_shutdown')
            self.valid = False

    context = Context()

    class Node:
        def destroy_node(self):
            # Mirrors the existing node's final UDP zero before socket close.
            assert context.ok()
            events.extend(['plant_zero', 'node_destroy'])

    node = Node()

    class Executor:
        def __init__(self, *, context):
            assert context is fake_context

        def add_node(self, value):
            assert value is node

        def wake(self):
            events.append('wake')

        def spin_once(self, *, timeout_sec):
            assert 0 < timeout_sec <= .1 and context.ok()
            events.append('callback_begin')
            if delivered_signal is not None:
                handlers[delivered_signal](delivered_signal, None)
            # In the failing graph SIGTERM invalidated this context before the
            # next publish in the already-running measurement callback.
            assert context.ok()
            events.append('publish_with_valid_context')
            if publish_failure is not None:
                raise publish_failure
            events.append('callback_end')

        def shutdown(self, *, timeout_sec):
            assert context.ok() and timeout_sec == .1
            events.append('executor_shutdown')
            if delivered_signal is not None:
                # A second signal during cleanup must not wake destroyed guards.
                handlers[delivered_signal](delivered_signal, None)
            return True

    class ExternalShutdownException(Exception):
        pass

    def init(*, context, signal_handler_options):
        assert context is fake_context and signal_handler_options is no_handlers
        context.valid = True
        events.append('context_init_no_ros_signal_handlers')

    fake_context = context
    modules = {
        'rclpy': {'init': init},
        'rclpy.context': {'Context': lambda: fake_context},
        'rclpy.executors': {'SingleThreadedExecutor': Executor,
                           'ExternalShutdownException': ExternalShutdownException},
        'rclpy.signals': {'SignalHandlerOptions': SimpleNamespace(NO=no_handlers)},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    def install(sig, handler):
        previous = handlers[sig]
        handlers[sig] = handler
        return previous

    monkeypatch.setattr(bridge.signal, 'signal', install)

    def factory(*, context):
        assert context is fake_context and context.ok()
        return node

    return factory, events, handlers, original_handlers, context


@pytest.mark.parametrize('sig', [signal.SIGTERM, signal.SIGINT])
def test_signal_during_publish_finishes_callback_before_context_shutdown(monkeypatch, sig):
    factory, events, handlers, original, context = fake_ros(monkeypatch, delivered_signal=sig)
    bridge.run_bridge(factory)
    assert events == ['context_init_no_ros_signal_handlers', 'callback_begin', 'wake',
        'publish_with_valid_context', 'callback_end', 'executor_shutdown',
        'plant_zero', 'node_destroy', 'context_shutdown']
    assert not context.ok() and handlers == original


def test_runtime_publish_error_still_propagates_after_cleanup(monkeypatch):
    class RCLError(Exception):
        pass

    failure = RCLError('real publication failure while context is valid')
    factory, events, handlers, original, context = fake_ros(monkeypatch, publish_failure=failure)
    with pytest.raises(RCLError) as caught:
        bridge.run_bridge(factory)
    assert caught.value is failure
    assert events[-4:] == ['executor_shutdown', 'plant_zero', 'node_destroy', 'context_shutdown']
    assert not context.ok() and handlers == original
