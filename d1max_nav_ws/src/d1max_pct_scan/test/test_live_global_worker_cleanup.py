"""No ROS or native bindings: cancellation cleanup cannot block callbacks."""
import pytest

from d1max_pct_scan.live_global_worker_cleanup import RetiredChildren


class FakePipe:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FakeChild:
    pid = 123

    def __init__(self, *, alive=True, exits_on_terminate=False):
        self.alive = alive
        self.exits_on_terminate = exits_on_terminate
        self.join_timeouts = []
        self.terminates = self.kills = 0

    def is_alive(self):
        return self.alive

    def join(self, timeout=None):
        self.join_timeouts.append(timeout)

    def terminate(self):
        self.terminates += 1
        if self.exits_on_terminate:
            self.alive = False

    def kill(self):
        self.kills += 1
        self.alive = False


def test_retire_does_not_wait_in_ros_callback_and_reap_kills_after_grace():
    current = [10.]
    cleanup = RetiredChildren(terminate_grace_s=.5, clock=lambda: current[0])
    child, pipe = FakeChild(), FakePipe()
    cleanup.retire(child, pipe)
    assert pipe.closed
    assert child.terminates == 1
    assert child.join_timeouts and set(child.join_timeouts) == {0}
    assert cleanup.reap() == 1
    current[0] += .51
    assert cleanup.reap() == 0
    assert child.kills == 1
    assert set(child.join_timeouts) == {0}


def test_exited_and_unstarted_children_leave_no_retired_process():
    cleanup = RetiredChildren()
    done, pipe = FakeChild(alive=False), FakePipe()
    cleanup.retire(done, pipe)
    assert pipe.closed and cleanup.pending == [] and done.terminates == 0
    unstarted, pipe = FakeChild(), FakePipe()
    unstarted.pid = None
    cleanup.retire(unstarted, pipe)
    assert pipe.closed and cleanup.pending == [] and unstarted.terminates == 0


def test_immediate_termination_is_reaped_without_scheduled_kill():
    cleanup = RetiredChildren()
    child = FakeChild(exits_on_terminate=True)
    cleanup.retire(child, FakePipe())
    assert cleanup.pending == [] and child.kills == 0


def test_invalid_cleanup_grace_rejected():
    with pytest.raises(ValueError):
        RetiredChildren(terminate_grace_s=0)
    with pytest.raises(ValueError):
        RetiredChildren(terminate_grace_s=3)
