from threading import Event
from d1max_pct_scan.bounded_preparation import LatestPreparation


def take_when_done(worker):
    with worker._cv:
        assert worker._cv.wait_for(lambda:worker._result is not None,timeout=1.)
    return worker.take()


def test_one_running_one_latest_pending_no_old_result_resurrection():
    started,release=Event(),Event()
    calls=[]
    def prepare(value):
        calls.append(value)
        if value==1:
            started.set();assert release.wait(1.)
        return value*10
    worker=LatestPreparation(prepare)
    try:
        worker.submit('old',1);assert started.wait(1.)
        for i in range(2,100):worker.submit(str(i),i)
        release.set()
        result=take_when_done(worker)
        assert result.key=='99' and result.value==990 and calls==[1,99]
        assert worker.take() is None
    finally:release.set();assert worker.close()


def test_cancel_fences_completed_and_inflight_work_without_waiting():
    started,release=Event(),Event()
    def prepare(value):started.set();assert release.wait(1.);return value
    worker=LatestPreparation(prepare)
    try:
        worker.submit('old',1);assert started.wait(1.)
        worker.invalidate();assert worker.take() is None
        worker.submit('new',2);release.set()
        assert take_when_done(worker).key=='new'
    finally:release.set();assert worker.close()


def test_worker_failure_is_a_versioned_result_not_an_owner_callback():
    def broken(value):raise ValueError('bad_support')
    worker=LatestPreparation(broken)
    try:
        worker.submit('candidate',1)
        assert take_when_done(worker).error=='ValueError:bad_support'
    finally:assert worker.close()


def test_completion_wakes_owner_after_unlock_without_taking_its_result():
    from threading import get_ident
    owner=get_ident();woken=Event();threads=[]
    worker=None
    def notify():
        # Taking the condition here would deadlock if notification ran while
        # the producer still held it. The result still belongs to the owner.
        with worker._cv:
            assert worker._result.key=='candidate'
        threads.append(get_ident());woken.set()
    worker=LatestPreparation(lambda value:value*2,notify_ready=notify)
    try:
        worker.submit('candidate',7);assert woken.wait(1.)
        assert threads and threads[0]!=owner
        result=worker.take()
        assert result.key=='candidate' and result.value==14
        assert worker.take() is None
    finally:assert worker.close()


def test_invalidated_inflight_work_does_not_wake_or_overwrite_new_task():
    started,release,woken=Event(),Event(),Event()
    notifications=[]
    def prepare(value):
        if value=='old':started.set();assert release.wait(1.)
        return value
    def notify():notifications.append(True);woken.set()
    worker=LatestPreparation(prepare,notify_ready=notify)
    try:
        worker.submit('old-task','old');assert started.wait(1.)
        worker.invalidate();worker.submit('new-task','new');release.set()
        assert woken.wait(1.)
        result=worker.take()
        assert result.key=='new-task' and result.value=='new'
        assert len(notifications)==1
    finally:release.set();assert worker.close()


def test_notification_shutdown_failure_preserves_timer_fallback_and_worker():
    called=Event()
    def unavailable_executor():called.set();raise RuntimeError('context closed')
    worker=LatestPreparation(lambda value:value,notify_ready=unavailable_executor)
    try:
        worker.submit('first',1);assert called.wait(1.)
        assert take_when_done(worker).key=='first'
        # A wake failure cannot kill the worker or strand later preparation.
        worker.submit('second',2)
        assert take_when_done(worker).key=='second'
    finally:assert worker.close()
