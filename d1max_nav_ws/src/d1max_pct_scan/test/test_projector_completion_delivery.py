"""Real projector ownership callbacks, no ROS graph, SDK or bag replay."""
from concurrent.futures import Future

from d1max_pct_scan.ray_projection import AwaitingCoverage, PoseCoverageGap
from test_perception_ray_projector_boundary import (
    node, ready, raw_cloud, context, message, stamp, EPOCH)


class ControlledWorker:
    """Only execution timing is supplied; production projection is unchanged."""
    def __init__(self, *, immediate=False):
        self.jobs = []
        self.immediate = immediate

    def submit(self, function, *args, **kwargs):
        sensor = args[1].sensor_id
        assert not any(not job[0].done() and job[2][1].sensor_id == sensor for job in self.jobs)
        assert sum(not job[0].done() for job in self.jobs) < 2
        future = Future()
        future.set_running_or_notify_cancel()
        self.jobs.append((future, function, args, kwargs))
        if self.immediate:
            self.complete()
        return future

    def complete(self, error=None, *, sensor=None):
        future, function, args, kwargs = next(job for job in reversed(self.jobs)
            if not job[0].done() and (sensor is None or job[2][1].sensor_id == sensor))
        try:
            if error is not None:
                raise error
            result = function(*args, **kwargs)
        except Exception as caught:
            future.set_exception(caught)
        else:
            future.set_result(result)

    def shutdown(self, **kwargs):
        # A running Future may finish during the real join. Tests explicitly
        # deliver it after the owner's stopping barrier to cover that race.
        pass


def controlled(node, **kwargs):
    node.worker.shutdown(wait=True, cancel_futures=True)
    node.worker = ControlledWorker(**kwargs)
    return node.worker


def test_real_worker_completions_publish_both_sources_without_timer(node):
    ready(node)
    front, rear = raw_cloud(0), raw_cloud(1)
    node.on_rays(front)
    node.on_rays(rear)
    rear_receipt = node.pending[1][3]
    node.tick()  # Initial ordinary timer opportunity; no later timer ticks.
    for work in node.inflight:
        work[0].result(timeout=2.)
    assert node.completion_guard.ready.wait(2.)
    assert not node.publisher.messages  # Worker cannot publish on its own.
    assert node.completion_guard.callback_group is node.default_callback_group
    assert node.timers[-1][0][0] == .01
    assert node.timers[-1][1]['callback_group'] is node.default_callback_group

    assert node.inflight[1][1][3] == rear_receipt
    node.completion_guard.dispatch()
    assert len(node.publisher.messages) == 2
    assert node.inflight == [None, None]
    for result in node.publisher.messages:
        assert result.rays.header.stamp == stamp(EPOCH)
        assert result.acquisition_end == stamp(EPOCH+100000000)
        assert result.alignment_stamp == stamp(EPOCH+100000000)
    assert [result.projection_sequence for result in node.publisher.messages] == [1, 2]
    node.tick()  # Timer/guard convergence must not duplicate a publication.
    assert len(node.publisher.messages) == 2


def test_stage_receipt_is_original_across_coverage_retry_and_new_pending(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud(0))
    original = node.pending[0]
    ingress = dict(original[4])
    node.tick()
    first_submit = node.inflight[0][4]['submitted_steady_ns']
    worker.complete(AwaitingCoverage('waiting_local_pose_coverage'), sensor=0)
    node.tick()
    assert node.inflight[0][1] is original
    assert node.inflight[0][1][4] == ingress
    assert node.inflight[0][4]['submitted_steady_ns'] >= first_submit
    # A newer acquisition occupies its own bounded pending slot. Completing
    # the old one must not borrow or overwrite the newer receipt metadata.
    node.on_rays(raw_cloud(0, EPOCH+1_000_000))
    newer = node.pending[0]
    worker.complete(sensor=0)
    node.tick()
    first_stage = node.projection_timing[0]
    assert first_stage['source_ns'] == EPOCH
    assert all(first_stage['stages'][key] == value for key, value in ingress.items())
    assert node.inflight[0][1] is newer and newer[4] is not original[4]
    worker.complete(sensor=0)
    node.tick()
    assert node.projection_timing[0]['source_ns'] == EPOCH+1_000_000
    assert all(node.projection_timing[0]['stages'][key] == value for key, value in newer[4].items())
    assert [row.rays.header.stamp for row in node.publisher.messages] == [
        stamp(EPOCH), stamp(EPOCH+1_000_000)]
    assert node.inflight == [None, None]


def test_already_completed_future_wakes_only_after_owner_ledger_is_assigned(node):
    ready(node)
    worker = controlled(node, immediate=True)
    node.on_rays(raw_cloud())
    node.tick()
    assert len(worker.jobs) == 1 and node.inflight[0][0].done()
    assert node.completion_guard.triggers == 2 and not node.publisher.messages
    node.completion_guard.dispatch()
    assert len(node.publisher.messages) == 1 and node.inflight == [None, None]


def test_completion_preserves_per_sensor_latest_only_without_cross_source_wait(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud(0))
    node.tick()
    node.on_rays(raw_cloud(1, EPOCH+1000000))
    node.on_rays(raw_cloud(1, EPOCH+2000000))
    node.on_rays(raw_cloud(0, EPOCH+3000000))
    rear_receipt, front_receipt = node.pending[1][3], node.pending[0][3]
    node.completion_guard.dispatch()  # Rear enters while front is still running.
    assert len(worker.jobs) == 2
    assert node.inflight[1][1][1].start_ns == EPOCH+2000000
    assert node.inflight[1][1][3] == rear_receipt
    worker.complete(sensor=1)
    node.completion_guard.dispatch()
    assert len(worker.jobs) == 2
    assert node.pending[0][1].start_ns == EPOCH+3000000
    assert node.last_published == [0, EPOCH+2000000]
    worker.complete(sensor=0)
    node.completion_guard.dispatch()
    assert len(worker.jobs) == 3
    assert node.inflight[0][1][1].start_ns == EPOCH+3000000
    assert node.inflight[0][1][3] == front_receipt
    assert node.counts['overwritten'] == 1
    worker.complete()
    node.completion_guard.dispatch()
    assert len(node.publisher.messages) == 3 and node.inflight == [None, None]


def test_old_context_completion_cannot_publish_or_recreate_discarded_pending(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.on_context(message(context(sequence=2, barrier=EPOCH+90000000)))
    node.on_ack(message(context(sequence=2, barrier=EPOCH+90000000)))
    worker.complete()
    node.completion_guard.dispatch()
    assert not node.publisher.messages and node.pending == [None, None]
    assert node.last_published == [0, 0]


def test_completion_does_not_refresh_an_expired_original_acquisition(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.tick()
    worker.complete()
    node._now = EPOCH+650000000
    node.completion_guard.dispatch()
    assert not node.publisher.messages and node.last_published == [0, 0]
    assert node.error == 'projection_expired_during_conversion'


def test_shutdown_barrier_prevents_late_worker_from_triggering_destroyed_guard(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.tick()
    node.destroy_node()
    worker.complete()
    assert node.completion_guard.destroyed
    assert node.completion_guard.triggers == 1  # only input delivery, before shutdown
    assert node.completion_wake_failures == 0
    node.tick()
    assert not node.publisher.messages


def test_guard_failure_leaves_timer_fallback_without_renewing_source(node, monkeypatch):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.tick()
    def shutdown_rcl():
        raise RuntimeError('context is shutting down')
    monkeypatch.setattr(node.completion_guard, 'trigger', shutdown_rcl)
    worker.complete()
    assert node.completion_wake_failures == 1 and not node.publisher.messages
    node.tick()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].rays.header.stamp == stamp(EPOCH)


def test_missing_pose_coverage_does_not_self_wake_and_resubmit_in_busy_loop(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    receipt = node.pending[0][3]
    node.tick()
    worker.complete(AwaitingCoverage('waiting_same_stamp_map_alignment_pair'))
    assert node.completion_guard.triggers == 1 and len(worker.jobs) == 1
    # Keep the pre-existing timer retry behavior, not a new fast retry loop.
    node.tick()
    assert len(worker.jobs) == 2 and node.inflight[0][1][3] == receipt
    worker.complete()
    node.completion_guard.dispatch()
    assert len(node.publisher.messages) == 1


def test_cancelled_future_is_not_an_owner_wake_or_publication(node):
    future = Future()
    assert future.cancel()
    node.projection_completed(future)
    assert node.completion_guard.triggers == 0
    assert node.completion_wake_failures == 0 and not node.publisher.messages


def test_missing_bracket_diagnostic_preserves_exact_worker_evidence(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.completion_guard.dispatch()
    gap = PoseCoverageGap(EPOCH, EPOCH+80_000_000, EPOCH+20_000_000, 60_000_000)
    worker.complete(gap)
    node.completion_guard.dispatch()
    assert node.last_drop['pose_gap'] == gap.pose_gap
    assert node.last_drop['reason'] == 'local_pose_bracketing_gap'
    assert not node.publisher.messages and node.last_published == [0, 0]


def test_ingress_gap_is_per_source_bounded_and_does_not_change_pending(node):
    ready(node)
    controlled(node)
    node.on_rays(raw_cloud(0))
    node.on_rays(raw_cloud(1))
    first_receipt = node._now
    node._now += 300_000_000
    node.on_rays(raw_cloud(1, EPOCH+300_000_000))
    event = dict(previous_source_ns=EPOCH, source_ns=EPOCH+300_000_000,
                 previous_received_ns=first_receipt, received_ns=node._now)
    assert node.last_raw_gap == [{}, event]
    assert node.pending[1][1].start_ns == EPOCH+300_000_000
    assert node.last_published == [0, 0]
    # An ordinary next packet does not erase the gap before status samples it.
    node._now += 100_000_000
    node.on_rays(raw_cloud(1, EPOCH+400_000_000))
    assert node.last_raw_gap == [{}, event]
    node.on_context(message(context(sequence=2)))
    assert node.last_raw_gap == [{}, {}] and node.last_raw_receipt_ns == [0, 0]


def test_ready_raw_input_starts_without_waiting_for_fallback_timer(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    receipt = node.pending[0][3]
    assert node.inflight == [None, None] and node.completion_guard.triggers == 1
    node.completion_guard.dispatch()
    assert len(worker.jobs) == 1 and node.inflight[0][1][3] == receipt
    worker.complete()
    node.completion_guard.dispatch()
    assert len(node.publisher.messages) == 1
    assert node.publisher.messages[0].rays.header.stamp == stamp(EPOCH)


def test_wake_does_not_authorize_missing_pose_or_create_retry_spin(node):
    ready(node)
    worker = controlled(node)
    node.authorized_pose_ns = EPOCH
    node.on_rays(raw_cloud())
    assert node.completion_guard.triggers == 0
    node.tick()
    assert not worker.jobs and node.pending[0] is not None
    node.authorized_pose_ns = EPOCH+100000000
    node.wake_ready_input()
    node.completion_guard.dispatch()
    assert len(worker.jobs) == 1
    worker.complete(AwaitingCoverage('waiting_local_pose_coverage'))
    assert node.completion_guard.triggers == 1


def test_drop_cause_survives_later_success_and_has_bounded_storage(node):
    ready(node)
    worker = controlled(node)
    node.on_rays(raw_cloud())
    node.completion_guard.dispatch()
    worker.complete(ValueError('local_pose_bracketing_gap'))
    node.completion_guard.dispatch()
    assert node.drop_stages['geometry'] == 1
    assert node.last_drop['source_ns'] == EPOCH
    node.on_rays(raw_cloud(1))
    node.completion_guard.dispatch()
    worker.complete()
    node.completion_guard.dispatch()
    assert node.error == '' and node.counts['published'] == 1
    assert node.last_drop['reason'] == 'local_pose_bracketing_gap'
    for _ in range(100):
        node.record_drop('envelope', 'x'*1024)
    assert len(node.drop_stages) == 7 and len(node.last_drop) == 5
    assert len(node.last_drop['reason']) == 256
