"""Static proof never blocks ROS callbacks and stale generations never commit."""
import threading
import time

import pytest

from d1max_pct_scan.static_route_validation import StaticRouteValidator


def drain(validator):
    deadline = time.monotonic()+2.
    while time.monotonic() < deadline:
        result = validator.poll()
        if result is not None:
            return result
        time.sleep(.001)
    pytest.fail('bounded validation did not finish')


def test_submission_is_nonblocking_and_records_actual_static_time():
    validator = StaticRouteValidator()
    try:
        began = time.monotonic()
        validator.submit(1,lambda:(time.sleep(.14),'checked')[1])
        assert time.monotonic()-began < .05
        assert validator.poll() is None
        generation,result,error,elapsed = drain(validator)
        assert generation == 1 and result == 'checked' and error is None
        assert elapsed >= .14
    finally:
        validator.close()


def test_cancelled_running_generation_is_discarded_and_only_latest_pending_kept():
    validator = StaticRouteValidator()
    started,release = threading.Event(),threading.Event()
    called = []
    def old():
        started.set()
        release.wait(2.)
        return 'obsolete'
    try:
        validator.submit(1,old)
        assert started.wait(1.)
        validator.invalidate()
        for generation in range(2,102):
            validator.submit(generation,lambda g=generation:called.append(g) or g)
        assert validator.running_generation == 1 and validator.pending[0] == 101
        release.set()
        generation,result,error,_ = drain(validator)
        assert generation == result == 101 and error is None
        assert called == [101]
    finally:
        release.set()
        validator.close()


def test_static_error_is_reported_not_silently_promoted():
    validator = StaticRouteValidator()
    try:
        validator.submit(1,lambda:(_ for _ in ()).throw(ValueError('bad map')))
        generation,result,error,_ = drain(validator)
        assert generation == 1 and result is None and isinstance(error,ValueError)
    finally:
        validator.close()


def test_close_revokes_pending_authority_and_refuses_future_submissions():
    validator = StaticRouteValidator()
    validator.close()
    with pytest.raises(ValueError,match='fresh_static'):
        validator.submit(2,lambda:None)
