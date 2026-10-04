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
